// SPDX-License-Identifier: GPL-2.0-only
/*
 * UFS Host Controller driver for Exynos specific extensions
 *
 * Copyright (C) 2014-2015 Samsung Electronics Co., Ltd.
 * Author: Seungwon Jeon  <essuuj@gmail.com>
 * Author: Alim Akhtar <alim.akhtar@samsung.com>
 *
 */

#include <linux/unaligned.h>
#include <crypto/aes.h>
#include <linux/arm-smccc.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/mfd/syscon.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/sys_soc.h>
#include <linux/soc/samsung/greatlte-l4.h>

#include <ufs/ufshcd.h>
#include "ufshcd-pltfrm.h"
#include <ufs/ufshci.h>
#include <ufs/unipro.h>

#include "ufs-exynos.h"

#define DATA_UNIT_SIZE		4096

/*
 * Exynos's Vendor specific registers for UFSHCI
 */
#define HCI_TXPRDT_ENTRY_SIZE	0x00
#define PRDT_PREFETCH_EN	BIT(31)
#define HCI_RXPRDT_ENTRY_SIZE	0x04
#define HCI_1US_TO_CNT_VAL	0x0C
#define CNT_VAL_1US_MASK	0x3FF
#define HCI_UTRL_NEXUS_TYPE	0x40
#define HCI_UTMRL_NEXUS_TYPE	0x44
#define HCI_SW_RST		0x50
#define UFS_LINK_SW_RST		BIT(0)
#define UFS_UNIPRO_SW_RST	BIT(1)
#define UFS_SW_RST_MASK		(UFS_UNIPRO_SW_RST | UFS_LINK_SW_RST)
#define HCI_DATA_REORDER	0x60
#define HCI_UNIPRO_APB_CLK_CTRL	0x68
#define UNIPRO_APB_CLK(v, x)	(((v) & ~0xF) | ((x) & 0xF))
#define HCI_AXIDMA_RWDATA_BURST_LEN	0x6C
#define WLU_EN			BIT(31)
#define WLU_BURST_LEN(x)	((x) << 27 | ((x) & 0xF))
#define HCI_GPIO_OUT		0x70
#define HCI_ERR_EN_PA_LAYER	0x78
#define HCI_ERR_EN_DL_LAYER	0x7C
#define HCI_ERR_EN_N_LAYER	0x80
#define HCI_ERR_EN_T_LAYER	0x84
#define HCI_ERR_EN_DME_LAYER	0x88
#define HCI_V2P1_CTRL		0x8C
#define IA_TICK_SEL		BIT(16)
#define HCI_CLKSTOP_CTRL	0xB0
#define REFCLKOUT_STOP		BIT(4)
#define MPHY_APBCLK_STOP	BIT(3)
#define REFCLK_STOP		BIT(2)
#define UNIPRO_MCLK_STOP	BIT(1)
#define UNIPRO_PCLK_STOP	BIT(0)
#define CLK_STOP_MASK		(REFCLKOUT_STOP | REFCLK_STOP |\
				 UNIPRO_MCLK_STOP | MPHY_APBCLK_STOP|\
				 UNIPRO_PCLK_STOP)
/* HCI_MISC is also known as HCI_FORCE_HCS */
#define HCI_MISC		0xB4
#define REFCLK_CTRL_EN		BIT(7)
#define UNIPRO_PCLK_CTRL_EN	BIT(6)
#define UNIPRO_MCLK_CTRL_EN	BIT(5)
#define HCI_CORECLK_CTRL_EN	BIT(4)
#define CLK_CTRL_EN_MASK	(REFCLK_CTRL_EN |\
				 UNIPRO_PCLK_CTRL_EN |\
				 UNIPRO_MCLK_CTRL_EN)

#define HCI_IOP_ACG_DISABLE	0x100
#define HCI_IOP_ACG_DISABLE_EN	BIT(0)

/* Exynos 8895 (stock UFS hw-rev 4) vendor HCI registers, S:drivers/scsi/ufs/ufs-exynos.h */
#define HCI_UFS_ACG_DISABLE		0xFC	/* :112 */
#define HCI_UFS_ACG_DISABLE_EN		BIT(0)	/* :113 */
#define HCI_MPHY_REFCLK_SEL		0x108	/* :116 */
#define MPHY_REFCLK_SEL			BIT(0)	/* :117; 1: alternative (TCXO) path */
/* HCI_MISC (stock HCI_FORCE_HCS 0xB4): REFCLKOUT_STOP_EN | MPHY_APBCLK_STOP_EN |
 * UFSP_DRCG_EN | REFCLK_STOP_EN | UNIPRO_PCLK_STOP_EN | UNIPRO_MCLK_STOP_EN = 0xDE0 */
#define EXYNOS8895_CLK_STOP_CTRL_EN_ALL	(BIT(11) | BIT(10) | BIT(8) | BIT(7) | \
					 BIT(6) | BIT(5))	/* :82-96 */
/* HCI_CLKSTOP_CTRL bits 4, 2, 1, 0 = 0x17; MPHY_APBCLK_STOP is not part of it */
#define EXYNOS8895_CLK_STOP_ALL		(REFCLKOUT_STOP | REFCLK_STOP | \
					 UNIPRO_MCLK_STOP | UNIPRO_PCLK_STOP)	/* :65-75 */

/* Device fatal error */
#define DFES_ERR_EN		BIT(31)
#define DFES_DEF_L2_ERRS	(UIC_DATA_LINK_LAYER_ERROR_RX_BUF_OF |\
				 UIC_DATA_LINK_LAYER_ERROR_PA_INIT)
#define DFES_DEF_L3_ERRS	(UIC_NETWORK_UNSUPPORTED_HEADER_TYPE |\
				 UIC_NETWORK_BAD_DEVICEID_ENC |\
				 UIC_NETWORK_LHDR_TRAP_PACKET_DROPPING)
#define DFES_DEF_L4_ERRS	(UIC_TRANSPORT_UNSUPPORTED_HEADER_TYPE |\
				 UIC_TRANSPORT_UNKNOWN_CPORTID |\
				 UIC_TRANSPORT_NO_CONNECTION_RX |\
				 UIC_TRANSPORT_BAD_TC)

/* UFS Shareability */
#define UFS_EXYNOSAUTO_WR_SHARABLE	BIT(2)
#define UFS_EXYNOSAUTO_RD_SHARABLE	BIT(1)
#define UFS_EXYNOSAUTO_SHARABLE		(UFS_EXYNOSAUTO_WR_SHARABLE | \
					 UFS_EXYNOSAUTO_RD_SHARABLE)
#define UFS_GS101_WR_SHARABLE		BIT(1)
#define UFS_GS101_RD_SHARABLE		BIT(0)
#define UFS_GS101_SHARABLE		(UFS_GS101_WR_SHARABLE | \
					 UFS_GS101_RD_SHARABLE)
#define UFS_SHAREABILITY_OFFSET		0x710

/* Multi-host registers */
#define MHCTRL			0xC4
#define MHCTRL_EN_VH_MASK	(0xE)
#define MHCTRL_EN_VH(vh)	(vh << 1)
#define PH2VH_MBOX		0xD8

#define MH_MSG_MASK		(0xFF)

#define MH_MSG(id, msg)		((id << 8) | (msg & 0xFF))
#define MH_MSG_PH_READY		0x1
#define MH_MSG_VH_READY		0x2

#define ALLOW_INQUIRY		BIT(25)
#define ALLOW_MODE_SELECT	BIT(24)
#define ALLOW_MODE_SENSE	BIT(23)
#define ALLOW_PRE_FETCH		GENMASK(22, 21)
#define ALLOW_READ_CMD_ALL	GENMASK(20, 18)	/* read_6/10/16 */
#define ALLOW_READ_BUFFER	BIT(17)
#define ALLOW_READ_CAPACITY	GENMASK(16, 15)
#define ALLOW_REPORT_LUNS	BIT(14)
#define ALLOW_REQUEST_SENSE	BIT(13)
#define ALLOW_SYNCHRONIZE_CACHE	GENMASK(8, 7)
#define ALLOW_TEST_UNIT_READY	BIT(6)
#define ALLOW_UNMAP		BIT(5)
#define ALLOW_VERIFY		BIT(4)
#define ALLOW_WRITE_CMD_ALL	GENMASK(3, 1)	/* write_6/10/16 */

#define ALLOW_TRANS_VH_DEFAULT	(ALLOW_INQUIRY | ALLOW_MODE_SELECT | \
				 ALLOW_MODE_SENSE | ALLOW_PRE_FETCH | \
				 ALLOW_READ_CMD_ALL | ALLOW_READ_BUFFER | \
				 ALLOW_READ_CAPACITY | ALLOW_REPORT_LUNS | \
				 ALLOW_REQUEST_SENSE | ALLOW_SYNCHRONIZE_CACHE | \
				 ALLOW_TEST_UNIT_READY | ALLOW_UNMAP | \
				 ALLOW_VERIFY | ALLOW_WRITE_CMD_ALL)

#define HCI_MH_ALLOWABLE_TRAN_OF_VH		0x30C
#define HCI_MH_IID_IN_TASK_TAG			0X308

#define PH_READY_TIMEOUT_MS			(5 * MSEC_PER_SEC)

enum {
	UNIPRO_L1_5 = 0,/* PHY Adapter */
	UNIPRO_L2,	/* Data Link */
	UNIPRO_L3,	/* Network */
	UNIPRO_L4,	/* Transport */
	UNIPRO_DME,	/* DME */
};

/*
 * UNIPRO registers
 */
#define UNIPRO_DME_POWERMODE_REQ_LOCALL2TIMER0	0x7888
#define UNIPRO_DME_POWERMODE_REQ_LOCALL2TIMER1	0x788c
#define UNIPRO_DME_POWERMODE_REQ_LOCALL2TIMER2	0x7890
#define UNIPRO_DME_POWERMODE_REQ_REMOTEL2TIMER0	0x78B8
#define UNIPRO_DME_POWERMODE_REQ_REMOTEL2TIMER1	0x78BC
#define UNIPRO_DME_POWERMODE_REQ_REMOTEL2TIMER2	0x78C0
/* UniPro DME result registers of the Exynos 8895 host, S:drivers/scsi/ufs/ufs-exynos.h */
#define UNIPRO_DME_LINKSTARTUP_CNF_RESULT	0x7854	/* :175 */
#define UNIPRO_DME_PWR_CNF_RESULT		0x78E8	/* :190 */
#define UNIPRO_DME_PWR_IND_RESULT		0x78EC	/* :191 */

/*
 * UFS Protector registers
 */
#define UFSPRSECURITY	0x010
#define NSSMU		BIT(14)
#define UFSPSBEGIN0	0x200
#define UFSPSEND0	0x204
#define UFSPSLUN0	0x208
#define UFSPSCTRL0	0x20C

#define CNTR_DIV_VAL 40

#if IS_ENABLED(CONFIG_GREATLTE_HB)
/*
 * DEBUG: Exynos 8895 bring-up switches (run tokens, no rebuild), spec/L4_storage.md 8.2:
 * t_ufs5 - T-UFS-5 clean-failure image: hold the device in reset at every link
 *          startup, so DME_LINKSTARTUP fails and the probe must fail cleanly.
 * d1_skip_connt - deviation D-1 / T-UFS-6: skip the generic post-link CPort
 *          re-establishment, as stock rev 4 does.
 */
static bool gl_t_ufs5;
module_param_named(t_ufs5, gl_t_ufs5, bool, 0444);
MODULE_PARM_DESC(t_ufs5, "DEBUG 8895: hold the UFS device in reset at link startup (T-UFS-5)");
static bool gl_d1_skip_connt;
module_param_named(d1_skip_connt, gl_d1_skip_connt, bool, 0444);
MODULE_PARM_DESC(d1_skip_connt, "DEBUG 8895: no generic post-link CPort setup (D-1, T-UFS-6)");
#else
#define gl_t_ufs5		false
#define gl_d1_skip_connt	false
#endif
#define EXYNOS_UFS_OPT_DBG_SKIP_POST_LINK_CONNT	BIT(31)	/* DEBUG: gl_d1_skip_connt */

static void exynos_ufs_auto_ctrl_hcc(struct exynos_ufs *ufs, bool en);
static void exynos_ufs_ctrl_clkstop(struct exynos_ufs *ufs, bool en);

static inline void exynos_ufs_enable_auto_ctrl_hcc(struct exynos_ufs *ufs)
{
	exynos_ufs_auto_ctrl_hcc(ufs, true);
}

static inline void exynos_ufs_disable_auto_ctrl_hcc(struct exynos_ufs *ufs)
{
	exynos_ufs_auto_ctrl_hcc(ufs, false);
}

static inline void exynos_ufs_disable_auto_ctrl_hcc_save(
					struct exynos_ufs *ufs, u32 *val)
{
	*val = hci_readl(ufs, HCI_MISC);
	exynos_ufs_auto_ctrl_hcc(ufs, false);
}

static inline void exynos_ufs_auto_ctrl_hcc_restore(
					struct exynos_ufs *ufs, u32 *val)
{
	hci_writel(ufs, *val, HCI_MISC);
}

static inline void exynos_ufs_gate_clks(struct exynos_ufs *ufs)
{
	exynos_ufs_ctrl_clkstop(ufs, true);
}

static inline void exynos_ufs_ungate_clks(struct exynos_ufs *ufs)
{
	exynos_ufs_ctrl_clkstop(ufs, false);
}

static int exynos_ufs_shareability(struct exynos_ufs *ufs)
{
	/* IO Coherency setting */
	if (ufs->sysreg) {
		return regmap_update_bits(ufs->sysreg,
					  ufs->iocc_offset,
					  ufs->iocc_mask, ufs->iocc_val);
	}

	return 0;
}

static int gs101_ufs_drv_init(struct exynos_ufs *ufs)
{
	struct ufs_hba *hba = ufs->hba;
	u32 reg;

	/* Enable WriteBooster */
	hba->caps |= UFSHCD_CAP_WB_EN;

	/* Enable clock gating and hibern8 */
	hba->caps |= UFSHCD_CAP_CLK_GATING | UFSHCD_CAP_HIBERN8_WITH_CLK_GATING;

	/* set ACG to be controlled by UFS_ACG_DISABLE */
	reg = hci_readl(ufs, HCI_IOP_ACG_DISABLE);
	hci_writel(ufs, reg & (~HCI_IOP_ACG_DISABLE_EN), HCI_IOP_ACG_DISABLE);

	return exynos_ufs_shareability(ufs);
}

static int exynosauto_ufs_drv_init(struct exynos_ufs *ufs)
{
	return exynos_ufs_shareability(ufs);
}

static int exynosauto_ufs_post_hce_enable(struct exynos_ufs *ufs)
{
	struct ufs_hba *hba = ufs->hba;

	/* Enable Virtual Host #1 */
	ufshcd_rmwl(hba, MHCTRL_EN_VH_MASK, MHCTRL_EN_VH(1), MHCTRL);
	/* Default VH Transfer permissions */
	hci_writel(ufs, ALLOW_TRANS_VH_DEFAULT, HCI_MH_ALLOWABLE_TRAN_OF_VH);
	/* IID information is replaced in TASKTAG[7:5] instead of IID in UCD */
	hci_writel(ufs, 0x1, HCI_MH_IID_IN_TASK_TAG);

	return 0;
}

static int exynosauto_ufs_pre_link(struct exynos_ufs *ufs)
{
	struct ufs_hba *hba = ufs->hba;
	int i;
	u32 tx_line_reset_period, rx_line_reset_period;

	rx_line_reset_period = (RX_LINE_RESET_TIME * ufs->mclk_rate) / NSEC_PER_MSEC;
	tx_line_reset_period = (TX_LINE_RESET_TIME * ufs->mclk_rate) / NSEC_PER_MSEC;

	ufshcd_dme_set(hba, UIC_ARG_MIB(0x200), 0x40);
	for_each_ufs_rx_lane(ufs, i) {
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_CLK_PRD, i),
			       DIV_ROUND_UP(NSEC_PER_SEC, ufs->mclk_rate));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_CLK_PRD_EN, i), 0x0);

		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_LINERESET_VALUE2, i),
			       (rx_line_reset_period >> 16) & 0xFF);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_LINERESET_VALUE1, i),
			       (rx_line_reset_period >> 8) & 0xFF);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_LINERESET_VALUE0, i),
			       (rx_line_reset_period) & 0xFF);

		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x2f, i), 0x79);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x84, i), 0x1);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x25, i), 0xf6);
	}

	for_each_ufs_tx_lane(ufs, i) {
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_TX_CLK_PRD, i),
			       DIV_ROUND_UP(NSEC_PER_SEC, ufs->mclk_rate));
		/* Not to affect VND_TX_LINERESET_PVALUE to VND_TX_CLK_PRD */
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_TX_CLK_PRD_EN, i),
			       0x02);

		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_TX_LINERESET_PVALUE2, i),
			       (tx_line_reset_period >> 16) & 0xFF);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_TX_LINERESET_PVALUE1, i),
			       (tx_line_reset_period >> 8) & 0xFF);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_TX_LINERESET_PVALUE0, i),
			       (tx_line_reset_period) & 0xFF);

		/* TX PWM Gear Capability / PWM_G1_ONLY */
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x04, i), 0x1);
	}

	ufshcd_dme_set(hba, UIC_ARG_MIB(0x200), 0x0);

	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_LOCAL_TX_LCC_ENABLE), 0x0);

	ufshcd_dme_set(hba, UIC_ARG_MIB(0xa011), 0x8000);

	return 0;
}

static int exynosauto_ufs_pre_pwr_change(struct exynos_ufs *ufs,
					 struct ufs_pa_layer_attr *pwr)
{
	struct ufs_hba *hba = ufs->hba;

	/* PACP_PWR_req and delivered to the remote DME */
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_PWRMODEUSERDATA0), 12000);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_PWRMODEUSERDATA1), 32000);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_PWRMODEUSERDATA2), 16000);

	return 0;
}

static int exynosauto_ufs_post_pwr_change(struct exynos_ufs *ufs,
					  const struct ufs_pa_layer_attr *pwr)
{
	struct ufs_hba *hba = ufs->hba;
	u32 enabled_vh;

	enabled_vh = ufshcd_readl(hba, MHCTRL) & MHCTRL_EN_VH_MASK;

	/* Send physical host ready message to virtual hosts */
	ufshcd_writel(hba, MH_MSG(enabled_vh, MH_MSG_PH_READY), PH2VH_MBOX);

	return 0;
}

static int exynos7_ufs_pre_link(struct exynos_ufs *ufs)
{
	struct exynos_ufs_uic_attr *attr = ufs->drv_data->uic_attr;
	u32 val = attr->pa_dbg_opt_suite1_val;
	struct ufs_hba *hba = ufs->hba;
	int i;

	exynos_ufs_enable_ov_tm(hba);
	for_each_ufs_tx_lane(ufs, i)
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x297, i), 0x17);
	for_each_ufs_rx_lane(ufs, i) {
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x362, i), 0xff);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x363, i), 0x00);
	}
	exynos_ufs_disable_ov_tm(hba);

	for_each_ufs_tx_lane(ufs, i)
		ufshcd_dme_set(hba,
			UIC_ARG_MIB_SEL(TX_HIBERN8_CONTROL, i), 0x0);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_DBG_TXPHY_CFGUPDT), 0x1);
	udelay(1);
	ufshcd_dme_set(hba, UIC_ARG_MIB(attr->pa_dbg_opt_suite1_off),
					val | (1 << 12));
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_DBG_SKIP_RESET_PHY), 0x1);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_DBG_SKIP_LINE_RESET), 0x1);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_DBG_LINE_RESET_REQ), 0x1);
	udelay(1600);
	ufshcd_dme_set(hba, UIC_ARG_MIB(attr->pa_dbg_opt_suite1_off), val);

	return 0;
}

static int exynos7_ufs_post_link(struct exynos_ufs *ufs)
{
	struct ufs_hba *hba = ufs->hba;
	int i;

	exynos_ufs_enable_ov_tm(hba);
	for_each_ufs_tx_lane(ufs, i) {
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x28b, i), 0x83);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x29a, i), 0x07);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x277, i),
			TX_LINERESET_N(exynos_ufs_calc_time_cntr(ufs, 200000)));
	}
	exynos_ufs_disable_ov_tm(hba);

	exynos_ufs_enable_dbg_mode(hba);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_SAVECONFIGTIME), 0xbb8);
	exynos_ufs_disable_dbg_mode(hba);

	return 0;
}

static int exynos7_ufs_pre_pwr_change(struct exynos_ufs *ufs,
						struct ufs_pa_layer_attr *pwr)
{
	unipro_writel(ufs, 0x22, UNIPRO_DBG_FORCE_DME_CTRL_STATE);

	return 0;
}

static int exynos7_ufs_post_pwr_change(struct exynos_ufs *ufs,
				       const struct ufs_pa_layer_attr *pwr)
{
	struct ufs_hba *hba = ufs->hba;
	int lanes = max_t(u32, pwr->lane_rx, pwr->lane_tx);

	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_DBG_RXPHY_CFGUPDT), 0x1);

	if (lanes == 1) {
		exynos_ufs_enable_dbg_mode(hba);
		ufshcd_dme_set(hba, UIC_ARG_MIB(PA_CONNECTEDTXDATALANES), 0x1);
		exynos_ufs_disable_dbg_mode(hba);
	}

	return 0;
}

/*
 * exynos_ufs_auto_ctrl_hcc - HCI core clock control by h/w
 * Control should be disabled in the below cases
 * - Before host controller S/W reset
 * - Access to UFS protector's register
 */
static void exynos_ufs_auto_ctrl_hcc(struct exynos_ufs *ufs, bool en)
{
	u32 misc = hci_readl(ufs, HCI_MISC);

	if (en)
		hci_writel(ufs, misc | HCI_CORECLK_CTRL_EN, HCI_MISC);
	else
		hci_writel(ufs, misc & ~HCI_CORECLK_CTRL_EN, HCI_MISC);
}

static void exynos_ufs_ctrl_clkstop(struct exynos_ufs *ufs, bool en)
{
	u32 ctrl = hci_readl(ufs, HCI_CLKSTOP_CTRL);
	u32 misc = hci_readl(ufs, HCI_MISC);

	if (en) {
		hci_writel(ufs, misc | CLK_CTRL_EN_MASK, HCI_MISC);
		hci_writel(ufs, ctrl | CLK_STOP_MASK, HCI_CLKSTOP_CTRL);
	} else {
		hci_writel(ufs, ctrl & ~CLK_STOP_MASK, HCI_CLKSTOP_CTRL);
		hci_writel(ufs, misc & ~CLK_CTRL_EN_MASK, HCI_MISC);
	}
}

static int exynos_ufs_get_clk_info(struct exynos_ufs *ufs)
{
	struct ufs_hba *hba = ufs->hba;
	struct list_head *head = &hba->clk_list_head;
	struct ufs_clk_info *clki;
	unsigned long pclk_rate;
	u32 f_min, f_max;
	u8 div = 0;
	int ret = 0;

	if (list_empty(head))
		goto out;

	list_for_each_entry(clki, head, list) {
		if (!IS_ERR(clki->clk)) {
			if (!strcmp(clki->name, "core_clk"))
				ufs->clk_hci_core = clki->clk;
			else if (!strcmp(clki->name, "sclk_unipro_main"))
				ufs->clk_unipro_main = clki->clk;
		}
	}

	if (!ufs->clk_hci_core || !ufs->clk_unipro_main) {
		dev_err(hba->dev, "failed to get clk info\n");
		ret = -EINVAL;
		goto out;
	}

	ufs->mclk_rate = clk_get_rate(ufs->clk_unipro_main);

	if (ufs->opts & EXYNOS_UFS_OPT_UNIPRO_PCLK) {
		/* the UniPro clock is also the PCS/UniPro APB clock: no divider
		 * (stock rev 4, S:drivers/scsi/ufs/ufs-exynos.c:486-489) */
		if (ufs->mclk_rate < UNIPRO_PCLK_MIN ||
		    ufs->mclk_rate > UNIPRO_PCLK_MAX) {
			dev_err(hba->dev, "8895: implausible sclk_unipro_main rate %lu\n",
				ufs->mclk_rate);
			ret = -EINVAL;
			goto out;
		}
		ufs->pclk_rate = ufs->mclk_rate;
		ufs->pclk_div = 0;
		goto out;
	}

	pclk_rate = clk_get_rate(ufs->clk_hci_core);
	f_min = ufs->pclk_avail_min;
	f_max = ufs->pclk_avail_max;

	if (ufs->opts & EXYNOS_UFS_OPT_HAS_APB_CLK_CTRL) {
		do {
			pclk_rate /= (div + 1);

			if (pclk_rate <= f_max)
				break;
			div++;
		} while (pclk_rate >= f_min);
	}

	if (unlikely(pclk_rate < f_min || pclk_rate > f_max)) {
		dev_err(hba->dev, "not available pclk range %lu\n", pclk_rate);
		ret = -EINVAL;
		goto out;
	}

	ufs->pclk_rate = pclk_rate;
	ufs->pclk_div = div;

out:
	return ret;
}

static void exynos_ufs_set_unipro_pclk_div(struct exynos_ufs *ufs)
{
	if (ufs->opts & EXYNOS_UFS_OPT_HAS_APB_CLK_CTRL) {
		u32 val;

		val = hci_readl(ufs, HCI_UNIPRO_APB_CLK_CTRL);
		hci_writel(ufs, UNIPRO_APB_CLK(val, ufs->pclk_div),
			   HCI_UNIPRO_APB_CLK_CTRL);
	}
}

static void exynos_ufs_set_pwm_clk_div(struct exynos_ufs *ufs)
{
	struct ufs_hba *hba = ufs->hba;
	struct exynos_ufs_uic_attr *attr = ufs->drv_data->uic_attr;

	ufshcd_dme_set(hba,
		UIC_ARG_MIB(CMN_PWM_CLK_CTRL), attr->cmn_pwm_clk_ctrl);
}

static void exynos_ufs_calc_pwm_clk_div(struct exynos_ufs *ufs)
{
	struct ufs_hba *hba = ufs->hba;
	struct exynos_ufs_uic_attr *attr = ufs->drv_data->uic_attr;
	const unsigned int div = 30, mult = 20;
	const unsigned long pwm_min = 3 * 1000 * 1000;
	const unsigned long pwm_max = 9 * 1000 * 1000;
	const int divs[] = {32, 16, 8, 4};
	unsigned long clk = 0, _clk, clk_period;
	int i = 0, clk_idx = -1;

	clk_period = UNIPRO_PCLK_PERIOD(ufs);
	for (i = 0; i < ARRAY_SIZE(divs); i++) {
		_clk = NSEC_PER_SEC * mult / (clk_period * divs[i] * div);
		if (_clk >= pwm_min && _clk <= pwm_max) {
			if (_clk > clk) {
				clk_idx = i;
				clk = _clk;
			}
		}
	}

	if (clk_idx == -1) {
		ufshcd_dme_get(hba, UIC_ARG_MIB(CMN_PWM_CLK_CTRL), &clk_idx);
		dev_err(hba->dev,
			"failed to decide pwm clock divider, will not change\n");
	}

	attr->cmn_pwm_clk_ctrl = clk_idx & PWM_CLK_CTRL_MASK;
}

long exynos_ufs_calc_time_cntr(struct exynos_ufs *ufs, long period)
{
	const int precise = 10;
	long pclk_rate = ufs->pclk_rate;
	long clk_period, fraction;

	clk_period = UNIPRO_PCLK_PERIOD(ufs);
	fraction = ((NSEC_PER_SEC % pclk_rate) * precise) / pclk_rate;

	return (period * precise) / ((clk_period * precise) + fraction);
}

static void exynos_ufs_specify_phy_time_attr(struct exynos_ufs *ufs)
{
	struct exynos_ufs_uic_attr *attr = ufs->drv_data->uic_attr;
	struct ufs_phy_time_cfg *t_cfg = &ufs->t_cfg;

	if (ufs->opts & EXYNOS_UFS_OPT_SKIP_CONFIG_PHY_ATTR)
		return;

	t_cfg->tx_linereset_p =
		exynos_ufs_calc_time_cntr(ufs, attr->tx_dif_p_nsec);
	t_cfg->tx_linereset_n =
		exynos_ufs_calc_time_cntr(ufs, attr->tx_dif_n_nsec);
	t_cfg->tx_high_z_cnt =
		exynos_ufs_calc_time_cntr(ufs, attr->tx_high_z_cnt_nsec);
	t_cfg->tx_base_n_val =
		exynos_ufs_calc_time_cntr(ufs, attr->tx_base_unit_nsec);
	t_cfg->tx_gran_n_val =
		exynos_ufs_calc_time_cntr(ufs, attr->tx_gran_unit_nsec);
	t_cfg->tx_sleep_cnt =
		exynos_ufs_calc_time_cntr(ufs, attr->tx_sleep_cnt);

	t_cfg->rx_linereset =
		exynos_ufs_calc_time_cntr(ufs, attr->rx_dif_p_nsec);
	t_cfg->rx_hibern8_wait =
		exynos_ufs_calc_time_cntr(ufs, attr->rx_hibern8_wait_nsec);
	t_cfg->rx_base_n_val =
		exynos_ufs_calc_time_cntr(ufs, attr->rx_base_unit_nsec);
	t_cfg->rx_gran_n_val =
		exynos_ufs_calc_time_cntr(ufs, attr->rx_gran_unit_nsec);
	t_cfg->rx_sleep_cnt =
		exynos_ufs_calc_time_cntr(ufs, attr->rx_sleep_cnt);
	t_cfg->rx_stall_cnt =
		exynos_ufs_calc_time_cntr(ufs, attr->rx_stall_cnt);
}

static void exynos_ufs_config_phy_time_attr(struct exynos_ufs *ufs)
{
	struct ufs_hba *hba = ufs->hba;
	struct ufs_phy_time_cfg *t_cfg = &ufs->t_cfg;
	int i;

	exynos_ufs_set_pwm_clk_div(ufs);

	exynos_ufs_enable_ov_tm(hba);

	for_each_ufs_rx_lane(ufs, i) {
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(RX_FILLER_ENABLE, i),
				ufs->drv_data->uic_attr->rx_filler_enable);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(RX_LINERESET_VAL, i),
				RX_LINERESET(t_cfg->rx_linereset));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(RX_BASE_NVAL_07_00, i),
				RX_BASE_NVAL_L(t_cfg->rx_base_n_val));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(RX_BASE_NVAL_15_08, i),
				RX_BASE_NVAL_H(t_cfg->rx_base_n_val));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(RX_GRAN_NVAL_07_00, i),
				RX_GRAN_NVAL_L(t_cfg->rx_gran_n_val));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(RX_GRAN_NVAL_10_08, i),
				RX_GRAN_NVAL_H(t_cfg->rx_gran_n_val));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(RX_OV_SLEEP_CNT_TIMER, i),
				RX_OV_SLEEP_CNT(t_cfg->rx_sleep_cnt));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(RX_OV_STALL_CNT_TIMER, i),
				RX_OV_STALL_CNT(t_cfg->rx_stall_cnt));
	}

	for_each_ufs_tx_lane(ufs, i) {
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(TX_LINERESET_P_VAL, i),
				TX_LINERESET_P(t_cfg->tx_linereset_p));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(TX_HIGH_Z_CNT_07_00, i),
				TX_HIGH_Z_CNT_L(t_cfg->tx_high_z_cnt));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(TX_HIGH_Z_CNT_11_08, i),
				TX_HIGH_Z_CNT_H(t_cfg->tx_high_z_cnt));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(TX_BASE_NVAL_07_00, i),
				TX_BASE_NVAL_L(t_cfg->tx_base_n_val));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(TX_BASE_NVAL_15_08, i),
				TX_BASE_NVAL_H(t_cfg->tx_base_n_val));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(TX_GRAN_NVAL_07_00, i),
				TX_GRAN_NVAL_L(t_cfg->tx_gran_n_val));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(TX_GRAN_NVAL_10_08, i),
				TX_GRAN_NVAL_H(t_cfg->tx_gran_n_val));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(TX_OV_SLEEP_CNT_TIMER, i),
				TX_OV_H8_ENTER_EN |
				TX_OV_SLEEP_CNT(t_cfg->tx_sleep_cnt));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(TX_MIN_ACTIVATETIME, i),
				ufs->drv_data->uic_attr->tx_min_activatetime);
	}

	exynos_ufs_disable_ov_tm(hba);
}

static void exynos_ufs_config_phy_cap_attr(struct exynos_ufs *ufs)
{
	struct ufs_hba *hba = ufs->hba;
	struct exynos_ufs_uic_attr *attr = ufs->drv_data->uic_attr;
	int i;

	exynos_ufs_enable_ov_tm(hba);

	for_each_ufs_rx_lane(ufs, i) {
		ufshcd_dme_set(hba,
				UIC_ARG_MIB_SEL(RX_HS_G1_SYNC_LENGTH_CAP, i),
				attr->rx_hs_g1_sync_len_cap);
		ufshcd_dme_set(hba,
				UIC_ARG_MIB_SEL(RX_HS_G2_SYNC_LENGTH_CAP, i),
				attr->rx_hs_g2_sync_len_cap);
		ufshcd_dme_set(hba,
				UIC_ARG_MIB_SEL(RX_HS_G3_SYNC_LENGTH_CAP, i),
				attr->rx_hs_g3_sync_len_cap);
		ufshcd_dme_set(hba,
				UIC_ARG_MIB_SEL(RX_HS_G1_PREP_LENGTH_CAP, i),
				attr->rx_hs_g1_prep_sync_len_cap);
		ufshcd_dme_set(hba,
				UIC_ARG_MIB_SEL(RX_HS_G2_PREP_LENGTH_CAP, i),
				attr->rx_hs_g2_prep_sync_len_cap);
		ufshcd_dme_set(hba,
				UIC_ARG_MIB_SEL(RX_HS_G3_PREP_LENGTH_CAP, i),
				attr->rx_hs_g3_prep_sync_len_cap);
	}

	if (attr->rx_adv_fine_gran_sup_en == 0) {
		for_each_ufs_rx_lane(ufs, i) {
			ufshcd_dme_set(hba,
				UIC_ARG_MIB_SEL(RX_ADV_GRANULARITY_CAP, i), 0);

			if (attr->rx_min_actv_time_cap)
				ufshcd_dme_set(hba,
					UIC_ARG_MIB_SEL(
					RX_MIN_ACTIVATETIME_CAPABILITY, i),
					attr->rx_min_actv_time_cap);

			if (attr->rx_hibern8_time_cap)
				ufshcd_dme_set(hba,
					UIC_ARG_MIB_SEL(RX_HIBERN8TIME_CAP, i),
						attr->rx_hibern8_time_cap);
		}
	} else if (attr->rx_adv_fine_gran_sup_en == 1) {
		for_each_ufs_rx_lane(ufs, i) {
			if (attr->rx_adv_fine_gran_step)
				ufshcd_dme_set(hba,
					UIC_ARG_MIB_SEL(RX_ADV_GRANULARITY_CAP,
						i), RX_ADV_FINE_GRAN_STEP(
						attr->rx_adv_fine_gran_step));

			if (attr->rx_adv_min_actv_time_cap)
				ufshcd_dme_set(hba,
					UIC_ARG_MIB_SEL(
						RX_ADV_MIN_ACTIVATETIME_CAP, i),
						attr->rx_adv_min_actv_time_cap);

			if (attr->rx_adv_hibern8_time_cap)
				ufshcd_dme_set(hba,
					UIC_ARG_MIB_SEL(RX_ADV_HIBERN8TIME_CAP,
						i),
						attr->rx_adv_hibern8_time_cap);
		}
	}

	exynos_ufs_disable_ov_tm(hba);
}

static void exynos_ufs_establish_connt(struct exynos_ufs *ufs)
{
	struct ufs_hba *hba = ufs->hba;
	enum {
		DEV_ID		= 0x00,
		PEER_DEV_ID	= 0x01,
		PEER_CPORT_ID	= 0x00,
		TRAFFIC_CLASS	= 0x00,
	};

	/* allow cport attributes to be set */
	ufshcd_dme_set(hba, UIC_ARG_MIB(T_CONNECTIONSTATE), CPORT_IDLE);

	/* local unipro attributes */
	ufshcd_dme_set(hba, UIC_ARG_MIB(N_DEVICEID), DEV_ID);
	ufshcd_dme_set(hba, UIC_ARG_MIB(N_DEVICEID_VALID), true);
	ufshcd_dme_set(hba, UIC_ARG_MIB(T_PEERDEVICEID), PEER_DEV_ID);
	ufshcd_dme_set(hba, UIC_ARG_MIB(T_PEERCPORTID), PEER_CPORT_ID);
	ufshcd_dme_set(hba, UIC_ARG_MIB(T_CPORTFLAGS), CPORT_DEF_FLAGS);
	ufshcd_dme_set(hba, UIC_ARG_MIB(T_TRAFFICCLASS), TRAFFIC_CLASS);
	ufshcd_dme_set(hba, UIC_ARG_MIB(T_CONNECTIONSTATE), CPORT_CONNECTED);
}

static void exynos_ufs_config_smu(struct exynos_ufs *ufs)
{
	u32 reg, val;

	if (ufs->opts & EXYNOS_UFS_OPT_UFSPR_SECURE)
		return;

	exynos_ufs_disable_auto_ctrl_hcc_save(ufs, &val);

	/* make encryption disabled by default */
	reg = ufsp_readl(ufs, UFSPRSECURITY);
	ufsp_writel(ufs, reg | NSSMU, UFSPRSECURITY);
	ufsp_writel(ufs, 0x0, UFSPSBEGIN0);
	ufsp_writel(ufs, 0xffffffff, UFSPSEND0);
	ufsp_writel(ufs, 0xff, UFSPSLUN0);
	ufsp_writel(ufs, 0xf1, UFSPSCTRL0);

	exynos_ufs_auto_ctrl_hcc_restore(ufs, &val);
}

static void exynos_ufs_config_sync_pattern_mask(struct exynos_ufs *ufs,
					struct ufs_pa_layer_attr *pwr)
{
	struct ufs_hba *hba = ufs->hba;
	u8 g = max_t(u32, pwr->gear_rx, pwr->gear_tx);
	u32 mask, sync_len;
	enum {
		SYNC_LEN_G1 = 80 * 1000, /* 80us */
		SYNC_LEN_G2 = 40 * 1000, /* 40us */
		SYNC_LEN_G3 = 20 * 1000, /* 20us */
	};
	int i;

	if (g == 1)
		sync_len = SYNC_LEN_G1;
	else if (g == 2)
		sync_len = SYNC_LEN_G2;
	else if (g == 3)
		sync_len = SYNC_LEN_G3;
	else
		return;

	mask = exynos_ufs_calc_time_cntr(ufs, sync_len);
	mask = (mask >> 8) & 0xff;

	exynos_ufs_enable_ov_tm(hba);

	for_each_ufs_rx_lane(ufs, i)
		ufshcd_dme_set(hba,
			UIC_ARG_MIB_SEL(RX_SYNC_MASK_LENGTH, i), mask);

	exynos_ufs_disable_ov_tm(hba);
}

#define UFS_HW_VER_MAJOR_MASK   GENMASK(15, 8)

static u32 exynos_ufs_get_hs_gear(struct ufs_hba *hba)
{
	u8 major;

	major = FIELD_GET(UFS_HW_VER_MAJOR_MASK, hba->ufs_version);

	if (major >= 3)
		return UFS_HS_G4;

	/* Default is HS-G3 */
	return UFS_HS_G3;
}

static int exynos_ufs_pre_pwr_mode(struct ufs_hba *hba,
				struct ufs_pa_layer_attr *dev_req_params)
{
	struct exynos_ufs *ufs = ufshcd_get_variant(hba);
	struct phy *generic_phy = ufs->phy;
	int ret;

	if (!dev_req_params) {
		pr_err("%s: incoming dev_req_params is NULL\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	if (ufs->drv_data->pre_pwr_change)
		ufs->drv_data->pre_pwr_change(ufs, dev_req_params);

	if (ufshcd_is_hs_mode(dev_req_params)) {
		if (!(ufs->opts & EXYNOS_UFS_OPT_SKIP_SYNC_PATTERN_MASK))
			exynos_ufs_config_sync_pattern_mask(ufs, dev_req_params);

		switch (dev_req_params->hs_rate) {
		case PA_HS_MODE_A:
		case PA_HS_MODE_B:
			phy_calibrate(generic_phy);
			break;
		}
	}

	/* setting for three timeout values for traffic class #0 */
	ufshcd_dme_set(hba, UIC_ARG_MIB(DL_FC0PROTTIMEOUTVAL), 8064);
	ufshcd_dme_set(hba, UIC_ARG_MIB(DL_TC0REPLAYTIMEOUTVAL), 28224);
	ufshcd_dme_set(hba, UIC_ARG_MIB(DL_AFC0REQTIMEOUTVAL), 20160);

	return 0;
out:
	return ret;
}

#define PWR_MODE_STR_LEN	64
static int exynos_ufs_post_pwr_mode(struct ufs_hba *hba,
				const struct ufs_pa_layer_attr *pwr_req)
{
	struct exynos_ufs *ufs = ufshcd_get_variant(hba);
	struct phy *generic_phy = ufs->phy;
	int gear = max_t(u32, pwr_req->gear_rx, pwr_req->gear_tx);
	int lanes = max_t(u32, pwr_req->lane_rx, pwr_req->lane_tx);
	char pwr_str[PWR_MODE_STR_LEN] = "";

	/* let default be PWM Gear 1, Lane 1 */
	if (!gear)
		gear = 1;

	if (!lanes)
		lanes = 1;

	if (ufs->drv_data->post_pwr_change)
		ufs->drv_data->post_pwr_change(ufs, pwr_req);

	if ((ufshcd_is_hs_mode(pwr_req))) {
		switch (pwr_req->hs_rate) {
		case PA_HS_MODE_A:
		case PA_HS_MODE_B:
			phy_calibrate(generic_phy);
			break;
		}

		snprintf(pwr_str, PWR_MODE_STR_LEN, "%s series_%s G_%d L_%d",
			"FAST",	pwr_req->hs_rate == PA_HS_MODE_A ? "A" : "B",
			gear, lanes);
	} else {
		snprintf(pwr_str, PWR_MODE_STR_LEN, "%s G_%d L_%d",
			"SLOW", gear, lanes);
	}

	dev_info(hba->dev, "Power mode changed to : %s\n", pwr_str);

	return 0;
}

static void exynos_ufs_specify_nexus_t_xfer_req(struct ufs_hba *hba,
						int tag, bool is_scsi_cmd)
{
	struct exynos_ufs *ufs = ufshcd_get_variant(hba);
	u32 type;

	type =  hci_readl(ufs, HCI_UTRL_NEXUS_TYPE);

	if (is_scsi_cmd)
		hci_writel(ufs, type | (1 << tag), HCI_UTRL_NEXUS_TYPE);
	else
		hci_writel(ufs, type & ~(1 << tag), HCI_UTRL_NEXUS_TYPE);
}

static void exynos_ufs_specify_nexus_t_tm_req(struct ufs_hba *hba,
						int tag, u8 func)
{
	struct exynos_ufs *ufs = ufshcd_get_variant(hba);
	u32 type;

	type =  hci_readl(ufs, HCI_UTMRL_NEXUS_TYPE);

	switch (func) {
	case UFS_ABORT_TASK:
	case UFS_QUERY_TASK:
		hci_writel(ufs, type | (1 << tag), HCI_UTMRL_NEXUS_TYPE);
		break;
	case UFS_ABORT_TASK_SET:
	case UFS_CLEAR_TASK_SET:
	case UFS_LOGICAL_RESET:
	case UFS_QUERY_TASK_SET:
		hci_writel(ufs, type & ~(1 << tag), HCI_UTMRL_NEXUS_TYPE);
		break;
	}
}

static int exynos_ufs_phy_init(struct exynos_ufs *ufs)
{
	struct ufs_hba *hba = ufs->hba;
	struct phy *generic_phy = ufs->phy;
	int ret = 0;

	if (ufs->avail_ln_rx == 0 || ufs->avail_ln_tx == 0) {
		ufshcd_dme_get(hba, UIC_ARG_MIB(PA_AVAILRXDATALANES),
			&ufs->avail_ln_rx);
		ufshcd_dme_get(hba, UIC_ARG_MIB(PA_AVAILTXDATALANES),
			&ufs->avail_ln_tx);
		WARN(ufs->avail_ln_rx != ufs->avail_ln_tx,
			"available data lane is not equal(rx:%d, tx:%d)\n",
			ufs->avail_ln_rx, ufs->avail_ln_tx);
	}

	phy_set_bus_width(generic_phy, ufs->avail_ln_rx);

	if (generic_phy->power_count) {
		phy_power_off(generic_phy);
		phy_exit(generic_phy);
	}

	ret = phy_init(generic_phy);
	if (ret) {
		dev_err(hba->dev, "%s: phy init failed, ret = %d\n",
			__func__, ret);
		return ret;
	}

	ret = phy_power_on(generic_phy);
	if (ret)
		goto out_exit_phy;

	return 0;

out_exit_phy:
	phy_exit(generic_phy);

	return ret;
}

static void exynos_ufs_config_unipro(struct exynos_ufs *ufs)
{
	struct exynos_ufs_uic_attr *attr = ufs->drv_data->uic_attr;
	struct ufs_hba *hba = ufs->hba;

	if (attr->pa_dbg_clk_period_off)
		ufshcd_dme_set(hba, UIC_ARG_MIB(attr->pa_dbg_clk_period_off),
			       DIV_ROUND_UP(NSEC_PER_SEC, ufs->mclk_rate));

	if (attr->tx_trailingclks)
		ufshcd_dme_set(hba, UIC_ARG_MIB(PA_TXTRAILINGCLOCKS),
			       attr->tx_trailingclks);

	if (attr->pa_dbg_opt_suite1_off)
		ufshcd_dme_set(hba, UIC_ARG_MIB(attr->pa_dbg_opt_suite1_off),
			       attr->pa_dbg_opt_suite1_val);

	if (attr->pa_dbg_opt_suite2_off)
		ufshcd_dme_set(hba, UIC_ARG_MIB(attr->pa_dbg_opt_suite2_off),
			       attr->pa_dbg_opt_suite2_val);
}

static void exynos_ufs_config_intr(struct exynos_ufs *ufs, u32 errs, u8 index)
{
	switch (index) {
	case UNIPRO_L1_5:
		hci_writel(ufs, DFES_ERR_EN | errs, HCI_ERR_EN_PA_LAYER);
		break;
	case UNIPRO_L2:
		hci_writel(ufs, DFES_ERR_EN | errs, HCI_ERR_EN_DL_LAYER);
		break;
	case UNIPRO_L3:
		hci_writel(ufs, DFES_ERR_EN | errs, HCI_ERR_EN_N_LAYER);
		break;
	case UNIPRO_L4:
		hci_writel(ufs, DFES_ERR_EN | errs, HCI_ERR_EN_T_LAYER);
		break;
	case UNIPRO_DME:
		hci_writel(ufs, DFES_ERR_EN | errs, HCI_ERR_EN_DME_LAYER);
		break;
	}
}

static int exynos_ufs_setup_clocks(struct ufs_hba *hba, bool on,
				   enum ufs_notify_change_status status)
{
	struct exynos_ufs *ufs = ufshcd_get_variant(hba);

	if (!ufs)
		return 0;

	/*
	 * The vendor HCI registers are only reachable while core_clk runs: touch
	 * them after the clocks are enabled and before they are disabled.
	 */
	if (on && status == POST_CHANGE) {
		if (ufs->opts & EXYNOS_UFS_OPT_BROKEN_AUTO_CLK_CTRL)
			exynos_ufs_disable_auto_ctrl_hcc(ufs);
		exynos_ufs_ungate_clks(ufs);
	} else if (!on && status == PRE_CHANGE) {
		exynos_ufs_gate_clks(ufs);
		if (ufs->opts & EXYNOS_UFS_OPT_BROKEN_AUTO_CLK_CTRL)
			exynos_ufs_enable_auto_ctrl_hcc(ufs);
	}

	return 0;
}

static int exynos_ufs_pre_link(struct ufs_hba *hba)
{
	struct exynos_ufs *ufs = ufshcd_get_variant(hba);

	/* hci */
	exynos_ufs_config_intr(ufs, DFES_DEF_L2_ERRS, UNIPRO_L2);
	exynos_ufs_config_intr(ufs, DFES_DEF_L3_ERRS, UNIPRO_L3);
	exynos_ufs_config_intr(ufs, DFES_DEF_L4_ERRS, UNIPRO_L4);
	exynos_ufs_set_unipro_pclk_div(ufs);

	/* the clocks are on during link startup */
	exynos_ufs_setup_clocks(hba, true, POST_CHANGE);

	/* unipro */
	exynos_ufs_config_unipro(ufs);

	if (ufs->drv_data->pre_link)
		ufs->drv_data->pre_link(ufs);

	/* m-phy */
	exynos_ufs_phy_init(ufs);
	if (!(ufs->opts & EXYNOS_UFS_OPT_SKIP_CONFIG_PHY_ATTR)) {
		exynos_ufs_config_phy_time_attr(ufs);
		exynos_ufs_config_phy_cap_attr(ufs);
	}

	return 0;
}

static void exynos_ufs_fit_aggr_timeout(struct exynos_ufs *ufs)
{
	u32 val;

	/* Select function clock (mclk) for timer tick */
	if (ufs->opts & EXYNOS_UFS_OPT_TIMER_TICK_SELECT) {
		val = hci_readl(ufs, HCI_V2P1_CTRL);
		val |= IA_TICK_SEL;
		hci_writel(ufs, val, HCI_V2P1_CTRL);
	}

	val = exynos_ufs_calc_time_cntr(ufs, IATOVAL_NSEC / CNTR_DIV_VAL);
	hci_writel(ufs, val & CNT_VAL_1US_MASK, HCI_1US_TO_CNT_VAL);
}

static int exynos_ufs_post_link(struct ufs_hba *hba)
{
	struct exynos_ufs *ufs = ufshcd_get_variant(hba);
	struct phy *generic_phy = ufs->phy;
	struct exynos_ufs_uic_attr *attr = ufs->drv_data->uic_attr;
	u32 val = ilog2(DATA_UNIT_SIZE);

	if (!(ufs->opts & EXYNOS_UFS_OPT_DBG_SKIP_POST_LINK_CONNT))	/* DEBUG: D-1 */
		exynos_ufs_establish_connt(ufs);
	exynos_ufs_fit_aggr_timeout(ufs);

	hci_writel(ufs, 0xa, HCI_DATA_REORDER);

	if (hba->caps & UFSHCD_CAP_CRYPTO)
		val |= PRDT_PREFETCH_EN;
	hci_writel(ufs, val, HCI_TXPRDT_ENTRY_SIZE);

	hci_writel(ufs, ilog2(DATA_UNIT_SIZE), HCI_RXPRDT_ENTRY_SIZE);
	hci_writel(ufs, BIT(hba->nutrs) - 1, HCI_UTRL_NEXUS_TYPE);
	hci_writel(ufs, BIT(hba->nutmrs) - 1, HCI_UTMRL_NEXUS_TYPE);
	hci_writel(ufs, 0xf, HCI_AXIDMA_RWDATA_BURST_LEN);

	if (ufs->opts & EXYNOS_UFS_OPT_SKIP_CONNECTION_ESTAB)
		ufshcd_dme_set(hba,
			UIC_ARG_MIB(T_DBG_SKIP_INIT_HIBERN8_EXIT), true);

	if (attr->pa_granularity) {
		exynos_ufs_enable_dbg_mode(hba);
		ufshcd_dme_set(hba, UIC_ARG_MIB(PA_GRANULARITY),
				attr->pa_granularity);
		exynos_ufs_disable_dbg_mode(hba);

		if (attr->pa_tactivate)
			ufshcd_dme_set(hba, UIC_ARG_MIB(PA_TACTIVATE),
					attr->pa_tactivate);
		if (attr->pa_hibern8time &&
		    !(ufs->opts & EXYNOS_UFS_OPT_USE_SW_HIBERN8_TIMER))
			ufshcd_dme_set(hba, UIC_ARG_MIB(PA_HIBERN8TIME),
					attr->pa_hibern8time);
	}

	if (ufs->opts & EXYNOS_UFS_OPT_USE_SW_HIBERN8_TIMER) {
		if (!attr->pa_granularity)
			ufshcd_dme_get(hba, UIC_ARG_MIB(PA_GRANULARITY),
					&attr->pa_granularity);
		if (!attr->pa_hibern8time)
			ufshcd_dme_get(hba, UIC_ARG_MIB(PA_HIBERN8TIME),
					&attr->pa_hibern8time);
		/*
		 * not wait for HIBERN8 time to exit hibernation
		 */
		ufshcd_dme_set(hba, UIC_ARG_MIB(PA_HIBERN8TIME), 0);

		if (attr->pa_granularity < 1 || attr->pa_granularity > 6) {
			/* Valid range for granularity: 1 ~ 6 */
			dev_warn(hba->dev,
				"%s: pa_granularity %d is invalid, assuming backwards compatibility\n",
				__func__,
				attr->pa_granularity);
			attr->pa_granularity = 6;
		}
	}

	phy_calibrate(generic_phy);

	if (ufs->drv_data->post_link)
		ufs->drv_data->post_link(ufs);

	return 0;
}

static int exynos_ufs_parse_dt(struct device *dev, struct exynos_ufs *ufs)
{
	struct device_node *np = dev->of_node;
	struct exynos_ufs_uic_attr *attr;
	int ret = 0;

	ufs->drv_data = device_get_match_data(dev);

	if (ufs->drv_data && ufs->drv_data->uic_attr) {
		attr = ufs->drv_data->uic_attr;
	} else {
		dev_err(dev, "failed to get uic attributes\n");
		ret = -EINVAL;
		goto out;
	}

	ufs->sysreg = syscon_regmap_lookup_by_phandle(np, "samsung,sysreg");
	if (IS_ERR(ufs->sysreg))
		ufs->sysreg = NULL;
	else {
		if (of_property_read_u32_index(np, "samsung,sysreg", 1,
					       &ufs->iocc_offset)) {
			dev_warn(dev, "can't get an offset from sysreg. Set to default value\n");
			ufs->iocc_offset = UFS_SHAREABILITY_OFFSET;
		}
	}

	ufs->iocc_mask = ufs->drv_data->iocc_mask;
	/*
	 * no 'dma-coherent' property means the descriptors are
	 * non-cacheable so iocc shareability should be disabled.
	 */
	if (of_dma_is_coherent(dev->of_node))
		ufs->iocc_val = ufs->iocc_mask;
	else
		ufs->iocc_val = 0;

	ufs->pclk_avail_min = PCLK_AVAIL_MIN;
	ufs->pclk_avail_max = PCLK_AVAIL_MAX;

	attr->rx_adv_fine_gran_sup_en = RX_ADV_FINE_GRAN_SUP_EN;
	attr->rx_adv_fine_gran_step = RX_ADV_FINE_GRAN_STEP_VAL;
	attr->rx_adv_min_actv_time_cap = RX_ADV_MIN_ACTV_TIME_CAP;
	attr->pa_granularity = PA_GRANULARITY_VAL;
	attr->pa_tactivate = PA_TACTIVATE_VAL;
	attr->pa_hibern8time = PA_HIBERN8TIME_VAL;

out:
	return ret;
}

static inline void exynos_ufs_priv_init(struct ufs_hba *hba,
					struct exynos_ufs *ufs)
{
	ufs->hba = hba;
	ufs->opts = ufs->drv_data->opts;
	ufs->rx_sel_idx = PA_MAXDATALANES;
	if (ufs->opts & EXYNOS_UFS_OPT_BROKEN_RX_SEL_IDX)
		ufs->rx_sel_idx = 0;
	hba->priv = (void *)ufs;
	hba->quirks = ufs->drv_data->quirks;
}

#ifdef CONFIG_SCSI_UFS_CRYPTO

/*
 * Support for Flash Memory Protector (FMP), which is the inline encryption
 * hardware on Exynos and Exynos-based SoCs.  The interface to this hardware is
 * not compatible with the standard UFS crypto.  It requires that encryption be
 * configured in the PRDT using a nonstandard extension.
 */

enum fmp_crypto_algo_mode {
	FMP_BYPASS_MODE = 0,
	FMP_ALGO_MODE_AES_CBC = 1,
	FMP_ALGO_MODE_AES_XTS = 2,
};
enum fmp_crypto_key_length {
	FMP_KEYLEN_256BIT = 1,
};

/**
 * struct fmp_sg_entry - nonstandard format of PRDT entries when FMP is enabled
 *
 * @base: The standard PRDT entry, but with nonstandard bitfields in the high
 *	bits of the 'size' field, i.e. the last 32-bit word.  When these
 *	nonstandard bitfields are zero, the data segment won't be encrypted or
 *	decrypted.  Otherwise they specify the algorithm and key length with
 *	which the data segment will be encrypted or decrypted.
 * @file_iv: The initialization vector (IV) with all bytes reversed
 * @file_enckey: The first half of the AES-XTS key with all bytes reserved
 * @file_twkey: The second half of the AES-XTS key with all bytes reserved
 * @disk_iv: Unused
 * @reserved: Unused
 */
struct fmp_sg_entry {
	struct ufshcd_sg_entry base;
	__be64 file_iv[2];
	__be64 file_enckey[4];
	__be64 file_twkey[4];
	__be64 disk_iv[2];
	__be64 reserved[2];
};

#define SMC_CMD_FMP_SECURITY	\
	ARM_SMCCC_CALL_VAL(ARM_SMCCC_FAST_CALL, ARM_SMCCC_SMC_64, \
			   ARM_SMCCC_OWNER_SIP, 0x1810)
#define SMC_CMD_SMU		\
	ARM_SMCCC_CALL_VAL(ARM_SMCCC_FAST_CALL, ARM_SMCCC_SMC_64, \
			   ARM_SMCCC_OWNER_SIP, 0x1850)
#define SMC_CMD_FMP_SMU_RESUME	\
	ARM_SMCCC_CALL_VAL(ARM_SMCCC_FAST_CALL, ARM_SMCCC_SMC_64, \
			   ARM_SMCCC_OWNER_SIP, 0x1860)
#define SMU_EMBEDDED			0
#define SMU_INIT			0
#define CFG_DESCTYPE_3			3

static void exynos_ufs_fmp_init(struct ufs_hba *hba, struct exynos_ufs *ufs)
{
	struct blk_crypto_profile *profile = &hba->crypto_profile;
	struct arm_smccc_res res;
	int err;

	/*
	 * Check for the standard crypto support bit, since it's available even
	 * though the rest of the interface to FMP is nonstandard.
	 *
	 * This check should have the effect of preventing the driver from
	 * trying to use FMP on old Exynos SoCs that don't have FMP.
	 */
	if (!(ufshcd_readl(hba, REG_CONTROLLER_CAPABILITIES) &
	      MASK_CRYPTO_SUPPORT))
		return;

	/*
	 * The below sequence of SMC calls to enable FMP can be found in the
	 * downstream driver source for gs101 and other Exynos-based SoCs.  It
	 * is the only way to enable FMP that works on SoCs such as gs101 that
	 * don't make the FMP registers accessible to Linux.  It probably works
	 * on other Exynos-based SoCs too, and might even still be the only way
	 * that works.  But this hasn't been properly tested, and this code is
	 * mutually exclusive with exynos_ufs_config_smu().  So for now only
	 * enable FMP support on SoCs with EXYNOS_UFS_OPT_UFSPR_SECURE.
	 */
	if (!(ufs->opts & EXYNOS_UFS_OPT_UFSPR_SECURE))
		return;

	/*
	 * This call (which sets DESCTYPE to 0x3 in the FMPSECURITY0 register)
	 * is needed to make the hardware use the larger PRDT entry size.
	 */
	BUILD_BUG_ON(sizeof(struct fmp_sg_entry) != 128);
	arm_smccc_smc(SMC_CMD_FMP_SECURITY, 0, SMU_EMBEDDED, CFG_DESCTYPE_3,
		      0, 0, 0, 0, &res);
	if (res.a0) {
		dev_warn(hba->dev,
			 "SMC_CMD_FMP_SECURITY failed on init: %ld.  Disabling FMP support.\n",
			 res.a0);
		return;
	}
	ufshcd_set_sg_entry_size(hba, sizeof(struct fmp_sg_entry));

	/*
	 * This is needed to initialize FMP.  Without it, errors occur when
	 * inline encryption is used.
	 */
	arm_smccc_smc(SMC_CMD_SMU, SMU_INIT, SMU_EMBEDDED, 0, 0, 0, 0, 0, &res);
	if (res.a0) {
		dev_err(hba->dev,
			"SMC_CMD_SMU(SMU_INIT) failed: %ld.  Disabling FMP support.\n",
			res.a0);
		return;
	}

	/* Advertise crypto capabilities to the block layer. */
	err = devm_blk_crypto_profile_init(hba->dev, profile, 0);
	if (err) {
		/* Only ENOMEM should be possible here. */
		dev_err(hba->dev, "Failed to initialize crypto profile: %d\n",
			err);
		return;
	}
	profile->max_dun_bytes_supported = AES_BLOCK_SIZE;
	profile->key_types_supported = BLK_CRYPTO_KEY_TYPE_RAW;
	profile->dev = hba->dev;
	profile->modes_supported[BLK_ENCRYPTION_MODE_AES_256_XTS] =
		DATA_UNIT_SIZE;

	/* Advertise crypto support to ufshcd-core. */
	hba->caps |= UFSHCD_CAP_CRYPTO;

	/* Advertise crypto quirks to ufshcd-core. */
	hba->quirks |= UFSHCD_QUIRK_CUSTOM_CRYPTO_PROFILE |
		       UFSHCD_QUIRK_BROKEN_CRYPTO_ENABLE |
		       UFSHCD_QUIRK_KEYS_IN_PRDT;

}

static void exynos_ufs_fmp_resume(struct ufs_hba *hba)
{
	struct arm_smccc_res res;

	if (!(hba->caps & UFSHCD_CAP_CRYPTO))
		return;

	arm_smccc_smc(SMC_CMD_FMP_SECURITY, 0, SMU_EMBEDDED, CFG_DESCTYPE_3,
		      0, 0, 0, 0, &res);
	if (res.a0)
		dev_err(hba->dev,
			"SMC_CMD_FMP_SECURITY failed on resume: %ld\n", res.a0);

	arm_smccc_smc(SMC_CMD_FMP_SMU_RESUME, 0, SMU_EMBEDDED, 0, 0, 0, 0, 0,
		      &res);
	if (res.a0)
		dev_err(hba->dev,
			"SMC_CMD_FMP_SMU_RESUME failed: %ld\n", res.a0);
}

static inline __be64 fmp_key_word(const u8 *key, int j)
{
	return cpu_to_be64(get_unaligned_le64(
			key + AES_KEYSIZE_256 - (j + 1) * sizeof(u64)));
}

/* Fill the PRDT for a request according to the given encryption context. */
static int exynos_ufs_fmp_fill_prdt(struct ufs_hba *hba,
				    const struct bio_crypt_ctx *crypt_ctx,
				    void *prdt, unsigned int num_segments)
{
	struct fmp_sg_entry *fmp_prdt = prdt;
	const u8 *enckey = crypt_ctx->bc_key->bytes;
	const u8 *twkey = enckey + AES_KEYSIZE_256;
	u64 dun_lo = crypt_ctx->bc_dun[0];
	u64 dun_hi = crypt_ctx->bc_dun[1];
	unsigned int i;

	/* If FMP wasn't enabled, we shouldn't get any encrypted requests. */
	if (WARN_ON_ONCE(!(hba->caps & UFSHCD_CAP_CRYPTO)))
		return -EIO;

	/* Configure FMP on each segment of the request. */
	for (i = 0; i < num_segments; i++) {
		struct fmp_sg_entry *prd = &fmp_prdt[i];
		int j;

		/* Each segment must be exactly one data unit. */
		if (prd->base.size != cpu_to_le32(DATA_UNIT_SIZE - 1)) {
			dev_err(hba->dev,
				"data segment is misaligned for FMP\n");
			return -EIO;
		}

		/* Set the algorithm and key length. */
		prd->base.size |= cpu_to_le32((FMP_ALGO_MODE_AES_XTS << 28) |
					      (FMP_KEYLEN_256BIT << 26));

		/* Set the IV. */
		prd->file_iv[0] = cpu_to_be64(dun_hi);
		prd->file_iv[1] = cpu_to_be64(dun_lo);

		/* Set the key. */
		for (j = 0; j < AES_KEYSIZE_256 / sizeof(u64); j++) {
			prd->file_enckey[j] = fmp_key_word(enckey, j);
			prd->file_twkey[j] = fmp_key_word(twkey, j);
		}

		/* Increment the data unit number. */
		dun_lo++;
		if (dun_lo == 0)
			dun_hi++;
	}
	return 0;
}

#else /* CONFIG_SCSI_UFS_CRYPTO */

static void exynos_ufs_fmp_init(struct ufs_hba *hba, struct exynos_ufs *ufs)
{
}

static void exynos_ufs_fmp_resume(struct ufs_hba *hba)
{
}

#define exynos_ufs_fmp_fill_prdt NULL

#endif /* !CONFIG_SCSI_UFS_CRYPTO */

static int exynos_ufs_init(struct ufs_hba *hba)
{
	struct device *dev = hba->dev;
	struct platform_device *pdev = to_platform_device(dev);
	struct exynos_ufs *ufs;
	int ret;

	ufs = devm_kzalloc(dev, sizeof(*ufs), GFP_KERNEL);
	if (!ufs)
		return -ENOMEM;

	/* exynos-specific hci */
	ufs->reg_hci = devm_platform_ioremap_resource_byname(pdev, "vs_hci");
	if (IS_ERR(ufs->reg_hci)) {
		dev_err(dev, "cannot ioremap for hci vendor register\n");
		return PTR_ERR(ufs->reg_hci);
	}

	/* unipro */
	ufs->reg_unipro = devm_platform_ioremap_resource_byname(pdev, "unipro");
	if (IS_ERR(ufs->reg_unipro)) {
		dev_err(dev, "cannot ioremap for unipro register\n");
		return PTR_ERR(ufs->reg_unipro);
	}

	/* ufs protector */
	ufs->reg_ufsp = devm_platform_ioremap_resource_byname(pdev, "ufsp");
	if (IS_ERR(ufs->reg_ufsp)) {
		dev_err(dev, "cannot ioremap for ufs protector register\n");
		return PTR_ERR(ufs->reg_ufsp);
	}

	ret = exynos_ufs_parse_dt(dev, ufs);
	if (ret) {
		dev_err(dev, "failed to get dt info.\n");
		goto out;
	}

	ufs->phy = devm_phy_get(dev, "ufs-phy");
	if (IS_ERR(ufs->phy)) {
		ret = PTR_ERR(ufs->phy);
		dev_err(dev, "failed to get ufs-phy\n");
		goto out;
	}

	exynos_ufs_priv_init(hba, ufs);

	exynos_ufs_fmp_init(hba, ufs);

	if (ufs->drv_data->drv_init) {
		ret = ufs->drv_data->drv_init(ufs);
		if (ret) {
			dev_err(dev, "failed to init drv-data\n");
			goto out;
		}
	}

	ret = exynos_ufs_get_clk_info(ufs);
	if (ret)
		goto out;
	exynos_ufs_specify_phy_time_attr(ufs);

	exynos_ufs_config_smu(ufs);

	hba->host->dma_alignment = DATA_UNIT_SIZE - 1;
	return 0;

out:
	hba->priv = NULL;
	return ret;
}

static void exynos_ufs_exit(struct ufs_hba *hba)
{
	struct exynos_ufs *ufs = ufshcd_get_variant(hba);

	phy_power_off(ufs->phy);
	phy_exit(ufs->phy);
}

static int exynos_ufs_host_reset(struct ufs_hba *hba)
{
	struct exynos_ufs *ufs = ufshcd_get_variant(hba);
	unsigned long timeout = jiffies + msecs_to_jiffies(1);
	u32 val;
	int ret = 0;

	exynos_ufs_disable_auto_ctrl_hcc_save(ufs, &val);

	hci_writel(ufs, UFS_SW_RST_MASK, HCI_SW_RST);

	do {
		if (!(hci_readl(ufs, HCI_SW_RST) & UFS_SW_RST_MASK))
			goto out;
	} while (time_before(jiffies, timeout));

	dev_err(hba->dev, "timeout host sw-reset\n");
	ret = -ETIMEDOUT;

out:
	exynos_ufs_auto_ctrl_hcc_restore(ufs, &val);
	return ret;
}

static void exynos_ufs_dev_hw_reset(struct ufs_hba *hba)
{
	struct exynos_ufs *ufs = ufshcd_get_variant(hba);

	hci_writel(ufs, 0 << 0, HCI_GPIO_OUT);
	udelay(5);
	hci_writel(ufs, 1 << 0, HCI_GPIO_OUT);
}

static void exynos_ufs_pre_hibern8(struct ufs_hba *hba, enum uic_cmd_dme cmd)
{
	struct exynos_ufs *ufs = ufshcd_get_variant(hba);
	struct exynos_ufs_uic_attr *attr = ufs->drv_data->uic_attr;

	if (cmd == UIC_CMD_DME_HIBER_EXIT) {
		if (ufs->opts & EXYNOS_UFS_OPT_BROKEN_AUTO_CLK_CTRL)
			exynos_ufs_disable_auto_ctrl_hcc(ufs);
		exynos_ufs_ungate_clks(ufs);

		if (ufs->opts & EXYNOS_UFS_OPT_USE_SW_HIBERN8_TIMER) {
			static const unsigned int granularity_tbl[] = {
				1, 4, 8, 16, 32, 100
			};
			int h8_time = attr->pa_hibern8time *
				granularity_tbl[attr->pa_granularity - 1];
			unsigned long us;
			s64 delta;

			do {
				delta = h8_time - ktime_us_delta(ktime_get(),
							ufs->entry_hibern8_t);
				if (delta <= 0)
					break;

				us = min_t(s64, delta, USEC_PER_MSEC);
				if (us >= 10)
					usleep_range(us, us + 10);
			} while (1);
		}
	}
}

static void exynos_ufs_post_hibern8(struct ufs_hba *hba, enum uic_cmd_dme cmd)
{
	struct exynos_ufs *ufs = ufshcd_get_variant(hba);

	if (cmd == UIC_CMD_DME_HIBER_ENTER) {
		ufs->entry_hibern8_t = ktime_get();
		exynos_ufs_gate_clks(ufs);
		if (ufs->opts & EXYNOS_UFS_OPT_BROKEN_AUTO_CLK_CTRL)
			exynos_ufs_enable_auto_ctrl_hcc(ufs);
	}
}

static int exynos_ufs_hce_enable_notify(struct ufs_hba *hba,
					enum ufs_notify_change_status status)
{
	struct exynos_ufs *ufs = ufshcd_get_variant(hba);
	int ret = 0;

	switch (status) {
	case PRE_CHANGE:
		/*
		 * The maximum segment size must be set after scsi_host_alloc()
		 * has been called and before LUN scanning starts
		 * (ufshcd_async_scan()). Note: this callback may also be called
		 * from other functions than ufshcd_init().
		 */
		hba->host->max_segment_size = DATA_UNIT_SIZE;

		if (ufs->drv_data->pre_hce_enable) {
			ret = ufs->drv_data->pre_hce_enable(ufs);
			if (ret)
				return ret;
		}

		ret = exynos_ufs_host_reset(hba);
		if (ret)
			return ret;
		exynos_ufs_dev_hw_reset(hba);
		break;
	case POST_CHANGE:
		exynos_ufs_calc_pwm_clk_div(ufs);
		if (!(ufs->opts & EXYNOS_UFS_OPT_BROKEN_AUTO_CLK_CTRL))
			exynos_ufs_enable_auto_ctrl_hcc(ufs);

		if (ufs->drv_data->post_hce_enable)
			ret = ufs->drv_data->post_hce_enable(ufs);

		break;
	}

	return ret;
}

static int exynos_ufs_link_startup_notify(struct ufs_hba *hba,
					  enum ufs_notify_change_status status)
{
	int ret = 0;

	switch (status) {
	case PRE_CHANGE:
		ret = exynos_ufs_pre_link(hba);
		break;
	case POST_CHANGE:
		ret = exynos_ufs_post_link(hba);
		break;
	}

	return ret;
}

static int exynos_ufs_negotiate_pwr_mode(struct ufs_hba *hba,
					 const struct ufs_pa_layer_attr *dev_max_params,
					 struct ufs_pa_layer_attr *dev_req_params)
{
	struct ufs_host_params host_params;

	ufshcd_init_host_params(&host_params);

	/* This driver only support symmetric gear setting e.g. hs_tx_gear == hs_rx_gear */
	host_params.hs_tx_gear = exynos_ufs_get_hs_gear(hba);
	host_params.hs_rx_gear = exynos_ufs_get_hs_gear(hba);

	return ufshcd_negotiate_pwr_params(&host_params, dev_max_params, dev_req_params);
}

static int exynos_ufs_pwr_change_notify(struct ufs_hba *hba,
				enum ufs_notify_change_status status,
				struct ufs_pa_layer_attr *dev_req_params)
{
	int ret = 0;

	switch (status) {
	case PRE_CHANGE:
		ret = exynos_ufs_pre_pwr_mode(hba, dev_req_params);
		break;
	case POST_CHANGE:
		ret = exynos_ufs_post_pwr_mode(hba, dev_req_params);
		break;
	}

	return ret;
}

static void exynos_ufs_hibern8_notify(struct ufs_hba *hba,
				     enum uic_cmd_dme cmd,
				     enum ufs_notify_change_status notify)
{
	switch ((u8)notify) {
	case PRE_CHANGE:
		exynos_ufs_pre_hibern8(hba, cmd);
		break;
	case POST_CHANGE:
		exynos_ufs_post_hibern8(hba, cmd);
		break;
	}
}

static int gs101_ufs_suspend(struct exynos_ufs *ufs)
{
	hci_writel(ufs, 0 << 0, HCI_GPIO_OUT);
	return 0;
}

static int exynos_ufs_suspend(struct ufs_hba *hba, enum ufs_pm_op pm_op,
	enum ufs_notify_change_status status)
{
	struct exynos_ufs *ufs = ufshcd_get_variant(hba);

	if (status == PRE_CHANGE)
		return 0;

	if (ufs->drv_data->suspend)
		ufs->drv_data->suspend(ufs);

	if (!ufshcd_is_link_active(hba))
		phy_power_off(ufs->phy);

	return 0;
}

static int exynos_ufs_resume(struct ufs_hba *hba, enum ufs_pm_op pm_op)
{
	struct exynos_ufs *ufs = ufshcd_get_variant(hba);

	if (ufs->drv_data->resume) {
		int ret = ufs->drv_data->resume(ufs);

		if (ret)
			return ret;
	}

	if (!ufshcd_is_link_active(hba))
		phy_power_on(ufs->phy);

	exynos_ufs_config_smu(ufs);
	exynos_ufs_fmp_resume(hba);
	return 0;
}

static int exynosauto_ufs_vh_link_startup_notify(struct ufs_hba *hba,
						 enum ufs_notify_change_status status)
{
	if (status == POST_CHANGE) {
		ufshcd_set_link_active(hba);
		ufshcd_set_ufs_dev_active(hba);
	}

	return 0;
}

static int exynosauto_ufs_vh_wait_ph_ready(struct ufs_hba *hba)
{
	u32 mbox;
	ktime_t start, stop;

	start = ktime_get();
	stop = ktime_add(start, ms_to_ktime(PH_READY_TIMEOUT_MS));

	do {
		mbox = ufshcd_readl(hba, PH2VH_MBOX);
		/* TODO: Mailbox message protocols between the PH and VHs are
		 * not implemented yet. This will be supported later
		 */
		if ((mbox & MH_MSG_MASK) == MH_MSG_PH_READY)
			return 0;

		usleep_range(40, 50);
	} while (ktime_before(ktime_get(), stop));

	return -ETIME;
}

static int exynosauto_ufs_vh_init(struct ufs_hba *hba)
{
	struct device *dev = hba->dev;
	struct platform_device *pdev = to_platform_device(dev);
	struct exynos_ufs *ufs;
	int ret;

	ufs = devm_kzalloc(dev, sizeof(*ufs), GFP_KERNEL);
	if (!ufs)
		return -ENOMEM;

	/* exynos-specific hci */
	ufs->reg_hci = devm_platform_ioremap_resource_byname(pdev, "vs_hci");
	if (IS_ERR(ufs->reg_hci)) {
		dev_err(dev, "cannot ioremap for hci vendor register\n");
		return PTR_ERR(ufs->reg_hci);
	}

	ret = exynosauto_ufs_vh_wait_ph_ready(hba);
	if (ret)
		return ret;

	ufs->drv_data = device_get_match_data(dev);
	if (!ufs->drv_data)
		return -ENODEV;

	exynos_ufs_priv_init(hba, ufs);

	return 0;
}

static int fsd_ufs_pre_link(struct exynos_ufs *ufs)
{
	struct exynos_ufs_uic_attr *attr = ufs->drv_data->uic_attr;
	struct ufs_hba *hba = ufs->hba;
	int i;

	ufshcd_dme_set(hba, UIC_ARG_MIB(attr->pa_dbg_clk_period_off),
		       DIV_ROUND_UP(NSEC_PER_SEC,  ufs->mclk_rate));
	ufshcd_dme_set(hba, UIC_ARG_MIB(0x201), 0x12);
	ufshcd_dme_set(hba, UIC_ARG_MIB(0x200), 0x40);

	for_each_ufs_tx_lane(ufs, i) {
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0xAA, i),
			       DIV_ROUND_UP(NSEC_PER_SEC, ufs->mclk_rate));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x8F, i), 0x3F);
	}

	for_each_ufs_rx_lane(ufs, i) {
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x12, i),
			       DIV_ROUND_UP(NSEC_PER_SEC, ufs->mclk_rate));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x5C, i), 0x38);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x0F, i), 0x0);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x65, i), 0x1);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x69, i), 0x1);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x21, i), 0x0);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x22, i), 0x0);
	}

	ufshcd_dme_set(hba, UIC_ARG_MIB(0x200), 0x0);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_DBG_AUTOMODE_THLD), 0x4E20);

	ufshcd_dme_set(hba, UIC_ARG_MIB(attr->pa_dbg_opt_suite1_off),
		       0x2e820183);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_LOCAL_TX_LCC_ENABLE), 0x0);

	exynos_ufs_establish_connt(ufs);

	return 0;
}

static int fsd_ufs_post_link(struct exynos_ufs *ufs)
{
	int i;
	struct ufs_hba *hba = ufs->hba;
	u32 hw_cap_min_tactivate;
	u32 peer_rx_min_actv_time_cap;
	u32 max_rx_hibern8_time_cap;

	ufshcd_dme_get(hba, UIC_ARG_MIB_SEL(0x8F, 4),
			&hw_cap_min_tactivate); /* HW Capability of MIN_TACTIVATE */
	ufshcd_dme_get(hba, UIC_ARG_MIB(PA_TACTIVATE),
			&peer_rx_min_actv_time_cap);    /* PA_TActivate */
	ufshcd_dme_get(hba, UIC_ARG_MIB(PA_HIBERN8TIME),
			&max_rx_hibern8_time_cap);      /* PA_Hibern8Time */

	if (peer_rx_min_actv_time_cap >= hw_cap_min_tactivate)
		ufshcd_dme_peer_set(hba, UIC_ARG_MIB(PA_TACTIVATE),
					peer_rx_min_actv_time_cap + 1);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_HIBERN8TIME), max_rx_hibern8_time_cap + 1);

	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_DBG_MODE), 0x01);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_SAVECONFIGTIME), 0xFA);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_DBG_MODE), 0x00);

	ufshcd_dme_set(hba, UIC_ARG_MIB(0x200), 0x40);

	for_each_ufs_rx_lane(ufs, i) {
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x35, i), 0x05);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x73, i), 0x01);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x41, i), 0x02);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x42, i), 0xAC);
	}

	ufshcd_dme_set(hba, UIC_ARG_MIB(0x200), 0x0);

	return 0;
}

static int fsd_ufs_pre_pwr_change(struct exynos_ufs *ufs,
					struct ufs_pa_layer_attr *pwr)
{
	struct ufs_hba *hba = ufs->hba;

	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_TXTERMINATION), 0x1);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_RXTERMINATION), 0x1);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_PWRMODEUSERDATA0), 12000);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_PWRMODEUSERDATA1), 32000);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_PWRMODEUSERDATA2), 16000);

	unipro_writel(ufs, 12000, UNIPRO_DME_POWERMODE_REQ_REMOTEL2TIMER0);
	unipro_writel(ufs, 32000, UNIPRO_DME_POWERMODE_REQ_REMOTEL2TIMER1);
	unipro_writel(ufs, 16000, UNIPRO_DME_POWERMODE_REQ_REMOTEL2TIMER2);

	return 0;
}

static int fsd_ufs_suspend(struct exynos_ufs *ufs)
{
	exynos_ufs_gate_clks(ufs);
	hci_writel(ufs, 0, HCI_GPIO_OUT);
	return 0;
}

static inline u32 get_mclk_period_unipro_18(struct exynos_ufs *ufs)
{
	return (16 * 1000 * 1000000UL / ufs->mclk_rate);
}

static int gs101_ufs_pre_link(struct exynos_ufs *ufs)
{
	struct ufs_hba *hba = ufs->hba;
	int i;
	u32 tx_line_reset_period, rx_line_reset_period;

	rx_line_reset_period = (RX_LINE_RESET_TIME * ufs->mclk_rate)
				/ NSEC_PER_MSEC;
	tx_line_reset_period = (TX_LINE_RESET_TIME * ufs->mclk_rate)
				/ NSEC_PER_MSEC;

	unipro_writel(ufs, get_mclk_period_unipro_18(ufs), COMP_CLK_PERIOD);

	ufshcd_dme_set(hba, UIC_ARG_MIB(0x200), 0x40);

	for_each_ufs_rx_lane(ufs, i) {
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_CLK_PRD, i),
			       DIV_ROUND_UP(NSEC_PER_SEC, ufs->mclk_rate));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_CLK_PRD_EN, i), 0x0);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_LINERESET_VALUE2, i),
			       (rx_line_reset_period >> 16) & 0xFF);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_LINERESET_VALUE1, i),
			       (rx_line_reset_period >> 8) & 0xFF);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_LINERESET_VALUE0, i),
			       (rx_line_reset_period) & 0xFF);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x2f, i), 0x69);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x84, i), 0x1);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x25, i), 0xf6);
	}

	for_each_ufs_tx_lane(ufs, i) {
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_TX_CLK_PRD, i),
			       DIV_ROUND_UP(NSEC_PER_SEC, ufs->mclk_rate));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_TX_CLK_PRD_EN, i),
			       0x02);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_TX_LINERESET_PVALUE2, i),
			       (tx_line_reset_period >> 16) & 0xFF);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_TX_LINERESET_PVALUE1, i),
			       (tx_line_reset_period >> 8) & 0xFF);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_TX_LINERESET_PVALUE0, i),
			       (tx_line_reset_period) & 0xFF);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x04, i), 1);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x7F, i), 0);
	}

	ufshcd_dme_set(hba, UIC_ARG_MIB(0x200), 0x0);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_LOCAL_TX_LCC_ENABLE), 0x0);
	ufshcd_dme_set(hba, UIC_ARG_MIB(N_DEVICEID), 0x0);
	ufshcd_dme_set(hba, UIC_ARG_MIB(N_DEVICEID_VALID), 0x1);
	ufshcd_dme_set(hba, UIC_ARG_MIB(T_PEERDEVICEID), 0x1);
	ufshcd_dme_set(hba, UIC_ARG_MIB(T_CONNECTIONSTATE), CPORT_CONNECTED);
	ufshcd_dme_set(hba, UIC_ARG_MIB(0xA006), 0x8000);

	return 0;
}

static int gs101_ufs_post_link(struct exynos_ufs *ufs)
{
	struct ufs_hba *hba = ufs->hba;

	/*
	 * Enable Write Line Unique. This field has to be 0x3
	 * to support Write Line Unique transaction on gs101.
	 */
	hci_writel(ufs, WLU_EN | WLU_BURST_LEN(3), HCI_AXIDMA_RWDATA_BURST_LEN);

	exynos_ufs_enable_dbg_mode(hba);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_SAVECONFIGTIME), 0x3e8);
	exynos_ufs_disable_dbg_mode(hba);

	return 0;
}

static int gs101_ufs_pre_pwr_change(struct exynos_ufs *ufs,
					 struct ufs_pa_layer_attr *pwr)
{
	struct ufs_hba *hba = ufs->hba;

	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_PWRMODEUSERDATA0), 12000);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_PWRMODEUSERDATA1), 32000);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_PWRMODEUSERDATA2), 16000);
	unipro_writel(ufs, 8064, UNIPRO_DME_POWERMODE_REQ_LOCALL2TIMER0);
	unipro_writel(ufs, 28224, UNIPRO_DME_POWERMODE_REQ_LOCALL2TIMER1);
	unipro_writel(ufs, 20160, UNIPRO_DME_POWERMODE_REQ_LOCALL2TIMER2);
	unipro_writel(ufs, 12000, UNIPRO_DME_POWERMODE_REQ_REMOTEL2TIMER0);
	unipro_writel(ufs, 32000, UNIPRO_DME_POWERMODE_REQ_REMOTEL2TIMER1);
	unipro_writel(ufs, 16000, UNIPRO_DME_POWERMODE_REQ_REMOTEL2TIMER2);

	return 0;
}

static const struct ufs_hba_variant_ops ufs_hba_exynos_ops = {
	.name				= "exynos_ufs",
	.init				= exynos_ufs_init,
	.exit				= exynos_ufs_exit,
	.hce_enable_notify		= exynos_ufs_hce_enable_notify,
	.link_startup_notify		= exynos_ufs_link_startup_notify,
	.negotiate_pwr_mode		= exynos_ufs_negotiate_pwr_mode,
	.pwr_change_notify		= exynos_ufs_pwr_change_notify,
	.setup_clocks			= exynos_ufs_setup_clocks,
	.setup_xfer_req			= exynos_ufs_specify_nexus_t_xfer_req,
	.setup_task_mgmt		= exynos_ufs_specify_nexus_t_tm_req,
	.hibern8_notify			= exynos_ufs_hibern8_notify,
	.suspend			= exynos_ufs_suspend,
	.resume				= exynos_ufs_resume,
	.fill_crypto_prdt		= exynos_ufs_fmp_fill_prdt,
};

static struct ufs_hba_variant_ops ufs_hba_exynosauto_vh_ops = {
	.name				= "exynosauto_ufs_vh",
	.init				= exynosauto_ufs_vh_init,
	.link_startup_notify		= exynosauto_ufs_vh_link_startup_notify,
};

/*
 * Exynos 8895 (stock UFS hw-rev 4). Every value below is the stock SM-N950N
 * kernel's (4.4.111, drivers/scsi/ufs/ufs-exynos.c and the ufs node of
 * exynos8895.dtsi); the PMA rows are in phy-exynos8895-ufs.c.
 * Citations: sufs.c:N / sufs.h:N = S:drivers/scsi/ufs/ufs-exynos.{c,h}:N,
 * dtsi:N = S:arch/arm64/boot/dts/exynos/exynos8895.dtsi:N,
 * smc.h:N = S:include/linux/smc.h:N.
 */
#define EXYNOS8895_OV_TM		0x200	/* PA_DBG_OV_TM, not 0x9540: sufs.h:382 */
#define EXYNOS8895_OV_TM_ON		0x40	/* dtsi:888 */
#define EXYNOS8895_RX_MIN_ACTV_TIME_CAP	3	/* ufs-rx-min-activate-time-cap, dtsi:881 */
#define EXYNOS8895_SYSREG_TCXO_SEL	0x1150	/* FSYS0 sysreg, dtsi:1094-1098 */
#define EXYNOS8895_TCXO_SEL_ALT		BIT(0)	/* alternative (TCXO) path, dtsi:1096-1097 */
#define EXYNOS8895_PMU_TCXO_CTRL	0x620	/* bit 0 from EVT 1.1 on, sufs.c:1417-1419 */
#define EXYNOS8895_PMU_UFS_PHY_CTRL	0x724	/* bit 0: isolation released, dtsi:1072 */

/* secure monitor calls of the stock driver: smc.h:75, 77, 79 */
#define EXYNOS8895_SMC_FMP_SECURITY	ARM_SMCCC_CALL_VAL(ARM_SMCCC_FAST_CALL, ARM_SMCCC_SMC_64, \
						   ARM_SMCCC_OWNER_SIP, 0x1810)	/* 0xC2001810 */
#define EXYNOS8895_SMC_SMU		ARM_SMCCC_CALL_VAL(ARM_SMCCC_FAST_CALL, ARM_SMCCC_SMC_64, \
						   ARM_SMCCC_OWNER_SIP, 0x1830)	/* 0xC2001830 */
#define EXYNOS8895_SMC_FMP_DUMP		ARM_SMCCC_CALL_VAL(ARM_SMCCC_FAST_CALL, ARM_SMCCC_SMC_64, \
						   ARM_SMCCC_OWNER_SIP, 0x1850)	/* 0xC2001850, read-only */
#define EXYNOS8895_UFSPR_DESCTYPE(v)	(((v) >> 19) & 0x3)	/* UFSPRSECURITY[20:19], sufs.h:296 */
#define EXYNOS8895_ID_FMP_UFS_MMC	0	/* smc.h:68 */
#define EXYNOS8895_FMP_DESCTYPE_0	0	/* 16-byte PRDT, no crypto: smc.h:71 */
#define EXYNOS8895_FMP_SMU_INIT		0	/* smc.h:119 */

/* SoC revisions before EVT 1.1 (0x11, sufs.c:1417-1418): "0".."f" and "10"
 * (exynos-chipid prints %x) */
static const struct soc_device_attribute exynos8895_pre_evt11[] = {
	{ .soc_id = "EXYNOS8895", .revision = "[0-9a-f]" },
	{ .soc_id = "EXYNOS8895", .revision = "10" },
	{ /* sentinel */ }
};

/*
 * Stock exynos_ufs_init_system() + exynos_ufs_config_smu(), at init and again
 * after every link-off resume (stock __exynos_ufs_resume()).
 */
static int exynos8895_ufs_sys_init(struct exynos_ufs *ufs, bool log)
{
	struct ufs_hba *hba = ufs->hba;
	struct arm_smccc_res res;
	unsigned long ufspr_sec;
	struct regmap *pmu;
	u32 phy_ctrl = 0;
	long sec, smu;
	int ret;

	/* IO coherency: FSYS0 sysreg +0x700 bits 9:8, dtsi:1082-1086 */
	if (!ufs->sysreg) {
		dev_err(hba->dev, "8895: samsung,sysreg missing\n");
		return -EINVAL;
	}
	ret = exynos_ufs_shareability(ufs);
	if (ret)
		return ret;

	/* M-PHY reference clock from the TCXO "alternative" path, sufs.c:1457 */
	ret = regmap_update_bits(ufs->sysreg, EXYNOS8895_SYSREG_TCXO_SEL,
				 EXYNOS8895_TCXO_SEL_ALT, EXYNOS8895_TCXO_SEL_ALT);
	if (ret)
		return ret;

	/* PMU: TCXO control from EVT 1.1 on, PHY isolation release */
	pmu = syscon_regmap_lookup_by_compatible("samsung,exynos8895-pmu");
	if (IS_ERR(pmu))
		return PTR_ERR(pmu);
	if (!soc_device_match(exynos8895_pre_evt11))
		regmap_update_bits(pmu, EXYNOS8895_PMU_TCXO_CTRL, BIT(0), BIT(0));
	regmap_read(pmu, EXYNOS8895_PMU_UFS_PHY_CTRL, &phy_ctrl);
	regmap_update_bits(pmu, EXYNOS8895_PMU_UFS_PHY_CTRL, BIT(0), BIT(0));

	/*
	 * The SMCs make EL3 access the UFS protector, which needs the HCI core
	 * clock: stock issues them with the HCI auto core-clock control off. It
	 * stays off, as stock leaves it and as BROKEN_AUTO_CLK_CTRL keeps it
	 * while the clocks run.
	 */
	exynos_ufs_disable_auto_ctrl_hcc(ufs);

	/* FMP/SMU setup: the two stock SMCs (sufs.c:870, 875), with the non-FMP
	 * descriptor type */
	arm_smccc_smc(EXYNOS8895_SMC_FMP_SECURITY, 0, EXYNOS8895_ID_FMP_UFS_MMC,
		      EXYNOS8895_FMP_DESCTYPE_0, 0, 0, 0, 0, &res);
	sec = res.a0;
	arm_smccc_smc(EXYNOS8895_SMC_SMU, EXYNOS8895_FMP_SMU_INIT,
		      EXYNOS8895_ID_FMP_UFS_MMC, 0, 0, 0, 0, 0, &res);
	smu = res.a0;

	/*
	 * Read UFSPRSECURITY back through the secure monitor, as stock's
	 * register dump does (S:drivers/scsi/ufs/ufs-exynos-dbg.c:955): its
	 * DESCTYPE field must be 0 (16-byte PRDT). An
	 * unknown SMC returns -1, whose DESCTYPE field reads 3, so it fails too.
	 */
	arm_smccc_smc(EXYNOS8895_SMC_FMP_DUMP, 0, 0, UFSPRSECURITY, 0, 0, 0, 0, &res);
	ufspr_sec = res.a0;

	if (log)
		dev_info(hba->dev, "8895: pmu724=%#x->%#x smc_sec=%ld smc_smu=%ld ufsprsecurity=%#lx desctype=%lu\n",
			 phy_ctrl, (u32)(phy_ctrl | BIT(0)), sec, smu, ufspr_sec,
			 EXYNOS8895_UFSPR_DESCTYPE(ufspr_sec));

	/*
	 * With the FMP in another descriptor mode the controller would walk our
	 * 16-byte PRDT with the wrong stride: fail before any UTP transfer.
	 */
	if (sec || smu || EXYNOS8895_UFSPR_DESCTYPE(ufspr_sec)) {
		dev_err(hba->dev, "8895: FMP not in 16-byte PRDT mode (sec=%ld smu=%ld ufsprsecurity=%#lx)\n",
			sec, smu, ufspr_sec);
		return -EIO;
	}

	return 0;
}

static int exynos8895_ufs_drv_init(struct exynos_ufs *ufs)
{
	struct ufs_hba *hba = ufs->hba;
	struct exynos_ufs_uic_attr *attr = ufs->drv_data->uic_attr;

	/*
	 * PM policy: system suspend powers the device down with the link off
	 * (stock spm_lvl 5); stock had no runtime PM at all. rpm_lvl 0 keeps
	 * device and link active on a W-LUN runtime suspend, and runtime
	 * suspend of the host itself (which gates the clocks with the link
	 * active) is forbidden.
	 */
	hba->spm_lvl = UFS_PM_LVL_5;
	hba->rpm_lvl = UFS_PM_LVL_0;
	pm_runtime_forbid(hba->dev);

	if (gl_d1_skip_connt)		/* DEBUG */
		ufs->opts |= EXYNOS_UFS_OPT_DBG_SKIP_POST_LINK_CONNT;

	/* no fixed hibern8 parameters: stock uses the capability exchange */
	attr->pa_granularity = 0;
	attr->pa_tactivate = 0;
	attr->pa_hibern8time = 0;

	return exynos8895_ufs_sys_init(ufs, true);
}

/* Only after a real power-down (link off): not on a W-LUN runtime resume. */
static int exynos8895_ufs_resume(struct exynos_ufs *ufs)
{
	if (!ufshcd_is_link_off(ufs->hba))
		return 0;

	return exynos8895_ufs_sys_init(ufs, false);
}

enum exynos8895_uic_kind {
	K_MIB,		/* one DME_SET of .attr */
	K_RX,		/* .attr on every RX lane (selector 4 + lane) */
	K_TX,		/* .attr on every TX lane (selector lane) */
	K_RX_PRD,	/* RX lanes: 1e9 / pclk, truncated */
	K_TX_PRD,	/* TX lanes: 1e9 / pclk, truncated */
	K_MCLK_PRD,	/* one DME_SET: 1e9 / mclk, truncated */
};

struct exynos8895_uic {
	u16 attr;
	u8 kind;
	u32 val;
};

/* phy-init, UniPro/PCS rows 1-20 (dtsi:887-908) in stock order; the PMA rows are the PHY's */
static const struct exynos8895_uic exynos8895_pre_link_tbl[] = {
	{ PA_DBG_CLK_PERIOD,		K_MCLK_PRD,	0 }, /* dtsi:887 */
	{ EXYNOS8895_OV_TM,		K_MIB,		EXYNOS8895_OV_TM_ON }, /* dtsi:888 */
	{ VND_RX_CLK_PRD,		K_RX_PRD,	0 }, /* dtsi:889 */
	{ VND_TX_CLK_PRD,		K_TX_PRD,	0 }, /* dtsi:890 */
	{ 0x5C,				K_RX,		0x38 }, /* dtsi:891 */
	{ 0x0F,				K_RX,		0x00 }, /* dtsi:892 */
	{ 0x65,				K_RX,		0x01 }, /* dtsi:893 */
	{ 0x69,				K_RX,		0x01 }, /* dtsi:894 */
	{ 0x21,				K_RX,		0x00 }, /* dtsi:895 */
	{ 0x22,				K_RX,		0x00 }, /* dtsi:896 */
	{ 0x84,				K_RX,		0x01 }, /* dtsi:897 */
	{ 0x04,				K_TX,		0x01 }, /* dtsi:898 */
	{ EXYNOS8895_OV_TM,		K_MIB,		0x0 }, /* dtsi:899 */
	{ PA_DBG_AUTOMODE_THLD,		K_MIB,		0x4E20 }, /* dtsi:900 */
	{ PA_DBG_OPTION_SUITE,		K_MIB,		0x2e820183 }, /* dtsi:901 */
	{ PA_LOCAL_TX_LCC_ENABLE,	K_MIB,		0x0 }, /* dtsi:902 */
	{ N_DEVICEID,			K_MIB,		0x0 }, /* dtsi:904 */
	{ N_DEVICEID_VALID,		K_MIB,		0x1 }, /* dtsi:905 */
	{ T_PEERDEVICEID,		K_MIB,		0x1 }, /* dtsi:906 */
	{ T_CONNECTIONSTATE,		K_MIB,		0x1 }, /* dtsi:908 */
};

/* post-phy-init, dtsi:924-932 */
static const struct exynos8895_uic exynos8895_post_link_tbl[] = {
	{ PA_DBG_MODE,			K_MIB,		0x1 }, /* dtsi:924 */
	{ PA_SAVECONFIGTIME,		K_MIB,		0xFA }, /* dtsi:925 */
	{ PA_DBG_MODE,			K_MIB,		0x0 }, /* dtsi:926 */
	{ EXYNOS8895_OV_TM,		K_MIB,		EXYNOS8895_OV_TM_ON }, /* dtsi:927 */
	{ 0x35,				K_RX,		0x05 }, /* dtsi:928 */
	{ 0x73,				K_RX,		0x01 }, /* dtsi:929 */
	{ 0x41,				K_RX,		0x02 }, /* dtsi:930 */
	{ 0x42,				K_RX,		0xAC }, /* dtsi:931 */
	{ EXYNOS8895_OV_TM,		K_MIB,		0x0 }, /* dtsi:932 */
};

/* row by row, all lanes of a row before the next row (stock loop order, sufs.c:999-1071) */
static void exynos8895_ufs_apply(struct exynos_ufs *ufs,
				 const struct exynos8895_uic *t, int n)
{
	struct ufs_hba *hba = ufs->hba;
	u32 mprd = NSEC_PER_SEC / ufs->mclk_rate;
	u32 pprd = NSEC_PER_SEC / ufs->pclk_rate;
	int i, l;

	for (i = 0; i < n; i++, t++) {
		switch (t->kind) {
		case K_MIB:
			ufshcd_dme_set(hba, UIC_ARG_MIB(t->attr), t->val);
			break;
		case K_MCLK_PRD:
			ufshcd_dme_set(hba, UIC_ARG_MIB(t->attr), mprd);
			break;
		case K_RX:
		case K_RX_PRD:
			for_each_ufs_rx_lane(ufs, l)
				ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(t->attr, l),
					       t->kind == K_RX ? t->val : pprd);
			break;
		case K_TX:
		case K_TX_PRD:
			for_each_ufs_tx_lane(ufs, l)
				ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(t->attr, l),
					       t->kind == K_TX ? t->val : pprd);
			break;
		}
	}
}

static int exynos8895_ufs_pre_link(struct exynos_ufs *ufs)
{
	struct ufs_hba *hba = ufs->hba;

	if (gl_t_ufs5) {		/* DEBUG: T-UFS-5 */
		hci_writel(ufs, 0, HCI_GPIO_OUT);
		return 0;
	}

	/*
	 * The per-lane rows need the lane counts, which exynos_ufs_phy_init()
	 * would only read after this hook: read them first, as stock does.
	 */
	if (!ufs->avail_ln_rx || !ufs->avail_ln_tx) {
		ufshcd_dme_get(hba, UIC_ARG_MIB(PA_AVAILRXDATALANES), &ufs->avail_ln_rx);
		ufshcd_dme_get(hba, UIC_ARG_MIB(PA_AVAILTXDATALANES), &ufs->avail_ln_tx);
	}

	/* stock link-startup PRE_CHANGE (sufs.c:1694-1701): clock-stop enables, refclk
	 * path, HWACG off */
	hci_writel(ufs, hci_readl(ufs, HCI_MISC) | EXYNOS8895_CLK_STOP_CTRL_EN_ALL,
		   HCI_MISC);
	hci_writel(ufs, hci_readl(ufs, HCI_MPHY_REFCLK_SEL) | MPHY_REFCLK_SEL,
		   HCI_MPHY_REFCLK_SEL);
	hci_writel(ufs, hci_readl(ufs, HCI_UFS_ACG_DISABLE) | HCI_UFS_ACG_DISABLE_EN,
		   HCI_UFS_ACG_DISABLE);

	/* phy-init UniPro/PCS rows; the PMA rows follow in phy_power_on() */
	exynos8895_ufs_apply(ufs, exynos8895_pre_link_tbl,
			     ARRAY_SIZE(exynos8895_pre_link_tbl));
	return 0;
}

static int exynos8895_ufs_post_link(struct exynos_ufs *ufs)
{
	struct ufs_hba *hba = ufs->hba;
	u32 tact = 0, h8 = 0;

	/* stock rev-4 HCI setup where the generic post_link differs; these win:
	 * TXPRDT sufs.c:1336-1337, burst :1342-1346 (sufs.h:51-52), 1 us tick :536-543 */
	hci_writel(ufs, PRDT_PREFETCH_EN | ilog2(DATA_UNIT_SIZE), HCI_TXPRDT_ENTRY_SIZE);
	hci_writel(ufs, WLU_EN | WLU_BURST_LEN(3), HCI_AXIDMA_RWDATA_BURST_LEN);
	hci_writel(ufs, hci_readl(ufs, HCI_V2P1_CTRL) | IA_TICK_SEL, HCI_V2P1_CTRL);
	hci_writel(ufs, (ufs->mclk_rate / USEC_PER_SEC) & CNT_VAL_1US_MASK,
		   HCI_1US_TO_CNT_VAL);

	/* hibern8 values by capability exchange, sufs.c:851-856 */
	ufshcd_dme_get(hba, UIC_ARG_MIB(PA_TACTIVATE), &tact);
	ufshcd_dme_get(hba, UIC_ARG_MIB(PA_HIBERN8TIME), &h8);
	if (EXYNOS8895_RX_MIN_ACTV_TIME_CAP <= tact)
		ufshcd_dme_peer_set(hba, UIC_ARG_MIB(PA_TACTIVATE), tact + 1);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_HIBERN8TIME), h8 + 1);

	exynos8895_ufs_apply(ufs, exynos8895_post_link_tbl,
			     ARRAY_SIZE(exynos8895_post_link_tbl));

	dev_info(hba->dev, "8895: link up, unipro 0x7854=%#x hcs=%#x tact=%u h8=%u\n",
		 unipro_readl(ufs, UNIPRO_DME_LINKSTARTUP_CNF_RESULT),
		 ufshcd_readl(hba, REG_CONTROLLER_STATUS), tact, h8);
	return 0;
}

static int exynos8895_ufs_pre_pwr_change(struct exynos_ufs *ufs,
					 struct ufs_pa_layer_attr *pwr)
{
	struct ufs_hba *hba = ufs->hba;

	if (!ufshcd_is_hs_mode(pwr) || pwr->hs_rate != PA_HS_MODE_B) {
		dev_warn(hba->dev, "8895: only HS rate B is calibrated (req %u/%u)\n",
			 pwr->pwr_rx, pwr->hs_rate);
		return 0;
	}

	/* L2 timers (stock exynos_ufs_set_l2_timer, sufs.c:1152-1170; dtsi:861-862) */
	ufshcd_dme_set(hba, UIC_ARG_MIB(DL_FC0PROTTIMEOUTVAL), 8000);
	ufshcd_dme_set(hba, UIC_ARG_MIB(DL_TC0REPLAYTIMEOUTVAL), 28000);
	ufshcd_dme_set(hba, UIC_ARG_MIB(DL_AFC0REQTIMEOUTVAL), 20000);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_PWRMODEUSERDATA0), 12000);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_PWRMODEUSERDATA1), 32000);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_PWRMODEUSERDATA2), 16000);
	unipro_writel(ufs, 8000, UNIPRO_DME_POWERMODE_REQ_LOCALL2TIMER0);
	unipro_writel(ufs, 28000, UNIPRO_DME_POWERMODE_REQ_LOCALL2TIMER1);
	unipro_writel(ufs, 20000, UNIPRO_DME_POWERMODE_REQ_LOCALL2TIMER2);
	unipro_writel(ufs, 12000, UNIPRO_DME_POWERMODE_REQ_REMOTEL2TIMER0);
	unipro_writel(ufs, 32000, UNIPRO_DME_POWERMODE_REQ_REMOTEL2TIMER1);
	unipro_writel(ufs, 16000, UNIPRO_DME_POWERMODE_REQ_REMOTEL2TIMER2);

	/* calib-of-hs-rate-b, UniPro rows, dtsi:994-1009 (rate B only, dtsi:877) */
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_TXTERMINATION), 0x1);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_RXTERMINATION), 0x1);
	ufshcd_dme_set(hba, UIC_ARG_MIB(DL_FC0PROTTIMEOUTVAL), 8064);
	ufshcd_dme_set(hba, UIC_ARG_MIB(DL_TC0REPLAYTIMEOUTVAL), 28224);
	ufshcd_dme_set(hba, UIC_ARG_MIB(DL_AFC0REQTIMEOUTVAL), 20160);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_PWRMODEUSERDATA0), 12000);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_PWRMODEUSERDATA1), 32000);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_PWRMODEUSERDATA2), 16000);
	unipro_writel(ufs, 8064, UNIPRO_DME_POWERMODE_REQ_LOCALL2TIMER0);
	unipro_writel(ufs, 28224, UNIPRO_DME_POWERMODE_REQ_LOCALL2TIMER1);
	unipro_writel(ufs, 20160, UNIPRO_DME_POWERMODE_REQ_LOCALL2TIMER2);
	unipro_writel(ufs, 12000, UNIPRO_DME_POWERMODE_REQ_REMOTEL2TIMER0);
	unipro_writel(ufs, 32000, UNIPRO_DME_POWERMODE_REQ_REMOTEL2TIMER1);
	unipro_writel(ufs, 16000, UNIPRO_DME_POWERMODE_REQ_REMOTEL2TIMER2);

	/* the PMA rows follow in phy_calibrate(CFG_PRE_PWR_HS) */
	return 0;
}

static int exynos8895_ufs_post_pwr_change(struct exynos_ufs *ufs,
					  const struct ufs_pa_layer_attr *pwr)
{
	struct ufs_hba *hba = ufs->hba;

	/* HCS.UPMCRS against the UniPro power-mode results stock reads instead */
	dev_info(hba->dev, "8895: pmc upmcrs(hcs)=%u unipro 0x78e8=%#x 0x78ec=%#x\n",
		 (ufshcd_readl(hba, REG_CONTROLLER_STATUS) >> 8) & 0x7,
		 unipro_readl(ufs, UNIPRO_DME_PWR_CNF_RESULT),
		 unipro_readl(ufs, UNIPRO_DME_PWR_IND_RESULT));
	return 0;
}

/*
 * Stock pre_setup_clocks(on) (sufs.c:1617-1634), which ran before every HCE; here the generic
 * setup_clocks hook runs before hba->priv exists at probe and the generic
 * ungate comes after HCE, so do it right before the host reset.
 */
static int exynos8895_ufs_pre_hce_enable(struct exynos_ufs *ufs)
{
	exynos_ufs_disable_auto_ctrl_hcc(ufs);
	hci_writel(ufs, hci_readl(ufs, HCI_CLKSTOP_CTRL) & ~EXYNOS8895_CLK_STOP_ALL,
		   HCI_CLKSTOP_CTRL);
	hci_writel(ufs, hci_readl(ufs, HCI_UFS_ACG_DISABLE) | HCI_UFS_ACG_DISABLE_EN,
		   HCI_UFS_ACG_DISABLE);
	return 0;
}

static int exynos8895_ufs_post_hce_enable(struct exynos_ufs *ufs)
{
	struct ufs_hba *hba = ufs->hba;
	static bool once;

	/* HWACG controlled by UFS_ACG_DISABLE, re-applied after every HCI_SW_RST
	 * (sufs.c:1358-1359) */
	hci_writel(ufs, hci_readl(ufs, HCI_IOP_ACG_DISABLE) & ~HCI_IOP_ACG_DISABLE_EN,
		   HCI_IOP_ACG_DISABLE);

	if (!once) {
		once = true;
		dev_info(hba->dev, "8895: mclk=%lu pclk=%u aclk=%lu refclk_sel(after reset)=%#x ver=%#x cap=%#x\n",
			 ufs->mclk_rate, ufs->pclk_rate,
			 clk_get_rate(ufs->clk_hci_core),
			 hci_readl(ufs, HCI_MPHY_REFCLK_SEL),
			 ufshcd_readl(hba, REG_UFS_VERSION),
			 ufshcd_readl(hba, REG_CONTROLLER_CAPABILITIES));
	}

	/*
	 * Stock waits 5 ms between the host enable and link startup
	 * (S:drivers/scsi/ufs/ufshcd.c:5628-5635, "guarantee auto flush for hynix
	 * ufs"); this hook is the last one before ufshcd_link_startup() at probe
	 * and in host_reset_and_restore().
	 */
	usleep_range(5000, 5100);
	return 0;
}

/* Only with the link off (system suspend, shutdown): assert the device reset
 * (sufs.c:1608-1615, 1843). */
static int exynos8895_ufs_suspend(struct exynos_ufs *ufs)
{
	if (!ufshcd_is_link_off(ufs->hba))
		return 0;

	hci_writel(ufs, 0 << 0, HCI_GPIO_OUT);
	return 0;
}

static int exynos_ufs_probe(struct platform_device *pdev)
{
	int err;
	struct device *dev = &pdev->dev;
	const struct ufs_hba_variant_ops *vops = &ufs_hba_exynos_ops;
	const struct exynos_ufs_drv_data *drv_data =
		device_get_match_data(dev);

	if (drv_data && drv_data->vops)
		vops = drv_data->vops;

	err = ufshcd_pltfrm_init(pdev, vops);
	if (err)
		dev_err(dev, "ufshcd_pltfrm_init() failed %d\n", err);
	greatlte_l4_ufs_probe_done(dev, err);	/* DEBUG: L4 heartbeat ufs= */

	return err;
}

static void exynos_ufs_remove(struct platform_device *pdev)
{
	ufshcd_pltfrm_remove(pdev);
}

static struct exynos_ufs_uic_attr exynos7_uic_attr = {
	.tx_trailingclks		= 0x10,
	.tx_dif_p_nsec			= 3000000,	/* unit: ns */
	.tx_dif_n_nsec			= 1000000,	/* unit: ns */
	.tx_high_z_cnt_nsec		= 20000,	/* unit: ns */
	.tx_base_unit_nsec		= 100000,	/* unit: ns */
	.tx_gran_unit_nsec		= 4000,		/* unit: ns */
	.tx_sleep_cnt			= 1000,		/* unit: ns */
	.tx_min_activatetime		= 0xa,
	.rx_filler_enable		= 0x2,
	.rx_dif_p_nsec			= 1000000,	/* unit: ns */
	.rx_hibern8_wait_nsec		= 4000000,	/* unit: ns */
	.rx_base_unit_nsec		= 100000,	/* unit: ns */
	.rx_gran_unit_nsec		= 4000,		/* unit: ns */
	.rx_sleep_cnt			= 1280,		/* unit: ns */
	.rx_stall_cnt			= 320,		/* unit: ns */
	.rx_hs_g1_sync_len_cap		= SYNC_LEN_COARSE(0xf),
	.rx_hs_g2_sync_len_cap		= SYNC_LEN_COARSE(0xf),
	.rx_hs_g3_sync_len_cap		= SYNC_LEN_COARSE(0xf),
	.rx_hs_g1_prep_sync_len_cap	= PREP_LEN(0xf),
	.rx_hs_g2_prep_sync_len_cap	= PREP_LEN(0xf),
	.rx_hs_g3_prep_sync_len_cap	= PREP_LEN(0xf),
	.pa_dbg_clk_period_off		= PA_DBG_CLK_PERIOD,
	.pa_dbg_opt_suite1_val		= 0x30103,
	.pa_dbg_opt_suite1_off		= PA_DBG_OPTION_SUITE,
};

static const struct exynos_ufs_drv_data exynosauto_ufs_drvs = {
	.uic_attr		= &exynos7_uic_attr,
	.quirks			= UFSHCD_QUIRK_PRDT_BYTE_GRAN |
				  UFSHCI_QUIRK_SKIP_RESET_INTR_AGGR |
				  UFSHCD_QUIRK_BROKEN_OCS_FATAL_ERROR |
				  UFSHCD_QUIRK_SKIP_DEF_UNIPRO_TIMEOUT_SETTING,
	.opts			= EXYNOS_UFS_OPT_BROKEN_AUTO_CLK_CTRL |
				  EXYNOS_UFS_OPT_SKIP_CONFIG_PHY_ATTR |
				  EXYNOS_UFS_OPT_BROKEN_RX_SEL_IDX,
	.iocc_mask		= UFS_EXYNOSAUTO_SHARABLE,
	.drv_init		= exynosauto_ufs_drv_init,
	.post_hce_enable	= exynosauto_ufs_post_hce_enable,
	.pre_link		= exynosauto_ufs_pre_link,
	.pre_pwr_change		= exynosauto_ufs_pre_pwr_change,
	.post_pwr_change	= exynosauto_ufs_post_pwr_change,
};

static const struct exynos_ufs_drv_data exynosauto_ufs_vh_drvs = {
	.vops			= &ufs_hba_exynosauto_vh_ops,
	.quirks			= UFSHCD_QUIRK_PRDT_BYTE_GRAN |
				  UFSHCI_QUIRK_SKIP_RESET_INTR_AGGR |
				  UFSHCD_QUIRK_BROKEN_OCS_FATAL_ERROR |
				  UFSHCI_QUIRK_BROKEN_HCE |
				  UFSHCD_QUIRK_BROKEN_UIC_CMD |
				  UFSHCD_QUIRK_SKIP_PH_CONFIGURATION |
				  UFSHCD_QUIRK_SKIP_DEF_UNIPRO_TIMEOUT_SETTING,
	.opts			= EXYNOS_UFS_OPT_BROKEN_RX_SEL_IDX,
};

static const struct exynos_ufs_drv_data exynos_ufs_drvs = {
	.uic_attr		= &exynos7_uic_attr,
	.quirks			= UFSHCD_QUIRK_PRDT_BYTE_GRAN |
				  UFSHCI_QUIRK_BROKEN_REQ_LIST_CLR |
				  UFSHCI_QUIRK_BROKEN_HCE |
				  UFSHCI_QUIRK_SKIP_RESET_INTR_AGGR |
				  UFSHCD_QUIRK_BROKEN_OCS_FATAL_ERROR |
				  UFSHCI_QUIRK_SKIP_MANUAL_WB_FLUSH_CTRL |
				  UFSHCD_QUIRK_SKIP_DEF_UNIPRO_TIMEOUT_SETTING,
	.opts			= EXYNOS_UFS_OPT_HAS_APB_CLK_CTRL |
				  EXYNOS_UFS_OPT_BROKEN_AUTO_CLK_CTRL |
				  EXYNOS_UFS_OPT_BROKEN_RX_SEL_IDX |
				  EXYNOS_UFS_OPT_SKIP_CONNECTION_ESTAB |
				  EXYNOS_UFS_OPT_USE_SW_HIBERN8_TIMER,
	.pre_link		= exynos7_ufs_pre_link,
	.post_link		= exynos7_ufs_post_link,
	.pre_pwr_change		= exynos7_ufs_pre_pwr_change,
	.post_pwr_change	= exynos7_ufs_post_pwr_change,
};

static struct exynos_ufs_uic_attr gs101_uic_attr = {
	.tx_trailingclks		= 0xff,
	.pa_dbg_opt_suite1_val		= 0x90913C1C,
	.pa_dbg_opt_suite1_off		= PA_GS101_DBG_OPTION_SUITE1,
	.pa_dbg_opt_suite2_val		= 0xE01C115F,
	.pa_dbg_opt_suite2_off		= PA_GS101_DBG_OPTION_SUITE2,
};

static struct exynos_ufs_uic_attr fsd_uic_attr = {
	.tx_trailingclks		= 0x10,
	.tx_dif_p_nsec			= 3000000,	/* unit: ns */
	.tx_dif_n_nsec			= 1000000,	/* unit: ns */
	.tx_high_z_cnt_nsec		= 20000,	/* unit: ns */
	.tx_base_unit_nsec		= 100000,	/* unit: ns */
	.tx_gran_unit_nsec		= 4000,		/* unit: ns */
	.tx_sleep_cnt			= 1000,		/* unit: ns */
	.tx_min_activatetime		= 0xa,
	.rx_filler_enable		= 0x2,
	.rx_dif_p_nsec			= 1000000,	/* unit: ns */
	.rx_hibern8_wait_nsec		= 4000000,	/* unit: ns */
	.rx_base_unit_nsec		= 100000,	/* unit: ns */
	.rx_gran_unit_nsec		= 4000,		/* unit: ns */
	.rx_sleep_cnt			= 1280,		/* unit: ns */
	.rx_stall_cnt			= 320,		/* unit: ns */
	.rx_hs_g1_sync_len_cap		= SYNC_LEN_COARSE(0xf),
	.rx_hs_g2_sync_len_cap		= SYNC_LEN_COARSE(0xf),
	.rx_hs_g3_sync_len_cap		= SYNC_LEN_COARSE(0xf),
	.rx_hs_g1_prep_sync_len_cap	= PREP_LEN(0xf),
	.rx_hs_g2_prep_sync_len_cap	= PREP_LEN(0xf),
	.rx_hs_g3_prep_sync_len_cap	= PREP_LEN(0xf),
	.pa_dbg_clk_period_off		= PA_DBG_CLK_PERIOD,
	.pa_dbg_opt_suite1_val		= 0x2E820183,
	.pa_dbg_opt_suite1_off		= PA_DBG_OPTION_SUITE,
};

static const struct exynos_ufs_drv_data fsd_ufs_drvs = {
	.uic_attr               = &fsd_uic_attr,
	.quirks                 = UFSHCD_QUIRK_PRDT_BYTE_GRAN |
				  UFSHCI_QUIRK_BROKEN_REQ_LIST_CLR |
				  UFSHCD_QUIRK_BROKEN_OCS_FATAL_ERROR |
				  UFSHCD_QUIRK_SKIP_DEF_UNIPRO_TIMEOUT_SETTING |
				  UFSHCI_QUIRK_SKIP_RESET_INTR_AGGR,
	.opts                   = EXYNOS_UFS_OPT_HAS_APB_CLK_CTRL |
				  EXYNOS_UFS_OPT_BROKEN_AUTO_CLK_CTRL |
				  EXYNOS_UFS_OPT_SKIP_CONFIG_PHY_ATTR |
				  EXYNOS_UFS_OPT_BROKEN_RX_SEL_IDX,
	.pre_link               = fsd_ufs_pre_link,
	.post_link              = fsd_ufs_post_link,
	.pre_pwr_change         = fsd_ufs_pre_pwr_change,
	.suspend                = fsd_ufs_suspend,
};

static const struct exynos_ufs_drv_data gs101_ufs_drvs = {
	.uic_attr		= &gs101_uic_attr,
	.quirks			= UFSHCD_QUIRK_PRDT_BYTE_GRAN |
				  UFSHCI_QUIRK_SKIP_RESET_INTR_AGGR |
				  UFSHCI_QUIRK_BROKEN_REQ_LIST_CLR |
				  UFSHCD_QUIRK_BROKEN_OCS_FATAL_ERROR |
				  UFSHCI_QUIRK_SKIP_MANUAL_WB_FLUSH_CTRL |
				  UFSHCD_QUIRK_SKIP_DEF_UNIPRO_TIMEOUT_SETTING,
	.opts			= EXYNOS_UFS_OPT_SKIP_CONFIG_PHY_ATTR |
				  EXYNOS_UFS_OPT_UFSPR_SECURE |
				  EXYNOS_UFS_OPT_TIMER_TICK_SELECT,
	.iocc_mask		= UFS_GS101_SHARABLE,
	.drv_init		= gs101_ufs_drv_init,
	.pre_link		= gs101_ufs_pre_link,
	.post_link		= gs101_ufs_post_link,
	.pre_pwr_change		= gs101_ufs_pre_pwr_change,
	.suspend		= gs101_ufs_suspend,
};

static struct exynos_ufs_uic_attr exynos8895_uic_attr = {
	/*
	 * All zero: no PA_TXTRAILINGCLOCKS; 0x9514 and 0x9564 are written by
	 * exynos8895_ufs_pre_link() at their stock positions; the phy-time and
	 * capability fields are unused with SKIP_CONFIG_PHY_ATTR.
	 */
	.tx_trailingclks	= 0,
	.pa_dbg_clk_period_off	= 0,
	.pa_dbg_opt_suite1_off	= 0,
};

static const struct exynos_ufs_drv_data exynos8895_ufs_drvs = {
	.uic_attr		= &exynos8895_uic_attr,
	.quirks			= UFSHCD_QUIRK_PRDT_BYTE_GRAN |
				  UFSHCI_QUIRK_BROKEN_REQ_LIST_CLR |
				  UFSHCI_QUIRK_SKIP_RESET_INTR_AGGR |
				  UFSHCD_QUIRK_BROKEN_OCS_FATAL_ERROR |
				  UFSHCD_QUIRK_SKIP_DEF_UNIPRO_TIMEOUT_SETTING |
				  UFSHCD_QUIRK_BROKEN_AUTO_HIBERN8,
	.opts			= EXYNOS_UFS_OPT_BROKEN_AUTO_CLK_CTRL |
				  EXYNOS_UFS_OPT_SKIP_CONFIG_PHY_ATTR |
				  EXYNOS_UFS_OPT_UFSPR_SECURE |
				  EXYNOS_UFS_OPT_TIMER_TICK_SELECT |
				  EXYNOS_UFS_OPT_UNIPRO_PCLK |
				  EXYNOS_UFS_OPT_SKIP_SYNC_PATTERN_MASK,
	.iocc_mask		= BIT(9) | BIT(8),	/* dtsi:1082-1086 */
	.drv_init		= exynos8895_ufs_drv_init,
	.pre_hce_enable		= exynos8895_ufs_pre_hce_enable,
	.post_hce_enable	= exynos8895_ufs_post_hce_enable,
	.pre_link		= exynos8895_ufs_pre_link,
	.post_link		= exynos8895_ufs_post_link,
	.pre_pwr_change		= exynos8895_ufs_pre_pwr_change,
	.post_pwr_change	= exynos8895_ufs_post_pwr_change,
	.suspend		= exynos8895_ufs_suspend,
	.resume			= exynos8895_ufs_resume,
};

static const struct of_device_id exynos_ufs_of_match[] = {
	{ .compatible = "google,gs101-ufs",
	  .data	      = &gs101_ufs_drvs },
	{ .compatible = "samsung,exynos7-ufs",
	  .data	      = &exynos_ufs_drvs },
	{ .compatible = "samsung,exynos8895-ufs",
	  .data	      = &exynos8895_ufs_drvs },
	{ .compatible = "samsung,exynosautov9-ufs",
	  .data	      = &exynosauto_ufs_drvs },
	{ .compatible = "samsung,exynosautov9-ufs-vh",
	  .data	      = &exynosauto_ufs_vh_drvs },
	{ .compatible = "tesla,fsd-ufs",
	  .data       = &fsd_ufs_drvs },
	{},
};
MODULE_DEVICE_TABLE(of, exynos_ufs_of_match);

static const struct dev_pm_ops exynos_ufs_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ufshcd_system_suspend, ufshcd_system_resume)
	SET_RUNTIME_PM_OPS(ufshcd_runtime_suspend, ufshcd_runtime_resume, NULL)
	.prepare	 = ufshcd_suspend_prepare,
	.complete	 = ufshcd_resume_complete,
};

static struct platform_driver exynos_ufs_pltform = {
	.probe	= exynos_ufs_probe,
	.remove = exynos_ufs_remove,
	.driver	= {
		.name	= "exynos-ufshc",
		.pm	= &exynos_ufs_pm_ops,
		.of_match_table = exynos_ufs_of_match,
	},
};
module_platform_driver(exynos_ufs_pltform);

MODULE_AUTHOR("Alim Akhtar <alim.akhtar@samsung.com>");
MODULE_AUTHOR("Seungwon Jeon  <essuuj@gmail.com>");
MODULE_DESCRIPTION("Exynos UFS HCI Driver");
MODULE_LICENSE("GPL v2");
