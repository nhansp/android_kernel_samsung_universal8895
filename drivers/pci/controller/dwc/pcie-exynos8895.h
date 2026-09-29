/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Samsung Exynos8895 PCIe RC0 controller driver
 * Constants and driver structures (spec L6D 5.1.2, 5.1.3).
 */

#ifndef __PCIE_EXYNOS8895_H
#define __PCIE_EXYNOS8895_H

#include <linux/clk.h>
#include <linux/gpio/consumer.h>
#include <linux/io.h>
#include <linux/notifier.h>
#include <linux/pci.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

#include "pcie-designware.h"

/* ELBI, S:drivers/pci/host/pci-exynos.h:110-195 */
#define EX_ELBI_IRQ_PULSE		0x000	/* :111 */
#define  EX_IRQ_INTA_ASSERT		BIT(0)	/* :112 */
#define  EX_IRQ_INTB_ASSERT		BIT(2)	/* :113 */
#define  EX_IRQ_INTC_ASSERT		BIT(4)	/* :114 */
#define  EX_IRQ_INTD_ASSERT		BIT(6)	/* :115 */
#define  EX_IRQ_RADM_PM_TO_ACK		BIT(18)	/* :118 */
#define  EX_IRQ_LINK_DOWN		BIT(30)	/* :128 */
#define EX_ELBI_IRQ_LEVEL		0x004	/* :120 */
#define EX_ELBI_IRQ_SPECIAL		0x008	/* :122 */
#define EX_ELBI_IRQ_EN_PULSE		0x00c	/* :123 */
#define  EX_IRQ_LINKDOWN_ENABLE		BIT(30)	/* :129 */
#define EX_ELBI_IRQ_EN_LEVEL		0x010	/* :126 */
#define EX_ELBI_IRQ_EN_SPECIAL		0x014	/* :130 */
#define EX_ELBI_SW_WAKE			0x018	/* :131; bit1 cleared = "Bus number enable", pci-exynos.c:762-765 */
#define EX_ELBI_APP_LTSSM_ENABLE	0x02c	/* :135; 1 = enable, 0 = disable (:143-144) */
#define EX_ELBI_IRQ_TOE_EN_PULSE2	0x034	/* :137 */
#define EX_ELBI_IRQ_TOE_EN_LEVEL	0x038	/* :138 */
#define EX_ELBI_L1_BUG_FIX_ENABLE	0x038	/* :139 (same offset, written 1 in establish_link :632) */
#define EX_ELBI_APP_REQ_EXIT_L1		0x040	/* :140 */
#define EX_ELBI_CXPL_DEBUG_INFO_H	0x070	/* :141 */
#define EX_ELBI_RDLH_LINKUP		0x074	/* :142; LTSSM = val & 0x1f (pci-exynos.c:650) */
#define EX_ELBI_PM_DSTATE		0x088	/* :145 */
#define EX_ELBI_PME_A8			0x0a8	/* PME_Turn_Off message block, pci-exynos.c:1777-1781,1823 (unnamed in stock) */
#define EX_ELBI_PME_AC			0x0ac	/* :1778 */
#define EX_ELBI_PME_B0			0x0b0	/* :1779 */
#define EX_ELBI_PME_D0			0x0d0	/* :1780 */
#define EX_ELBI_APP_REQ_EXIT_L1_MODE	0x0f4	/* :147 */
#define  EX_APP_REQ_EXIT_L1_MODE	BIT(0)	/* :148 */
#define  EX_L1_REQ_NAK_CONTROL_MASTER	BIT(4)	/* :150 */
#define EX_ELBI_HISTORY_REG(x)		(0x138 + (x) * 4)	/* :151; fields :152-154 */
#define EX_ELBI_LINKDOWN_RST_CTRL_SEL	0x1b8	/* :155 */
#define  EX_LINKDOWN_RST_MANUAL		BIT(1)	/* :156 */
#define EX_ELBI_SOFT_CORE_RESET		0x1d0	/* :160 */
#define EX_ELBI_STATE_HISTORY_CHECK	0x274	/* :161 */
#define  EX_HISTORY_BUFFER_ENABLE	BIT(0)	/* :162 */
#define EX_ELBI_PHY_PCS_GRST		0x288	/* pcs_g_rst, cal:177-183 */
#define EX_ELBI_PHY_CMN_RST		0x28c	/* PCS&PMA(CMN) reset, cal:174-176, :217-219 */
#define EX_ELBI_MAC_RST			0x290	/* MAC reset, cal:171-173, :213-215 */
#define EX_ELBI_QCH_SEL			0x2c8	/* :164; PMU/APB/AXI gating masks 0xf00|0x0f0|0x00f (:173,:179,:185) */
#define EX_ELBI_DMA_MONITOR(n)		(0x2cc + (n) * 4)	/* :186-188 */
#define EX_LTSSM_MIN			0x0d	/* pci-exynos.c:651 */
#define EX_LTSSM_MAX			0x14	/* :651 */
#define EX_LTSSM_L23_READY		0x15	/* :1802 */
#define EX_LINK_POLL_MAX		2000	/* MAX_TIMEOUT pci-exynos.h:16; 10 us per poll (:656) */
#define EX_LINK_TRIES			10	/* pci-exynos.c:663 */

/* PMU (syscon), S:drivers/pci/host/pci-exynos.h:197-205 */
#define EX_PMU_WAKEUP_MASK		0x610	/* :202 */
#define  EX_WAKEUP_MASK_PCIE_WIFI	5	/* :203, bit (5 + ch) */
#define EX_PMU_PCIE_PHY_CONTROL		0x71c	/* :204 (+ ch * 4, pci-exynos.c:1352) */
#define  EX_PCIE_PHY_CONTROL_MASK	0x1	/* :205 */

/* FSYS1 sysreg (syscon_fsys1 + 0x1044 = stock "sysreg" 0x11421044, FDT:8220) */
#define EX_SYSREG_PHY0			0x0	/* bit1 cleared by phy_config, cal:168 */
#define EX_SYSREG_WIFI0_PCIE_PHY_CONTROL 0xc	/* pci-exynos.h:268 */
#define  EX_BIFURCATION_MODE_DISABLE	BIT(16)	/* :269 */
#define  EX_LINK0_ENABLE		BIT(14)	/* :271 */
#define  EX_PCS_LANE0_ENABLE		BIT(12)	/* :273 */

/* PCS */
#define EX_PCS_RX_ELECIDLE		0xec	/* bit3 = "avoid checking rx elecidle when access DBI", cal:29-38 */

/* DBI (RC own config / port logic), S:drivers/pci/host/pci-exynos.h:207-233 */
#define EX_DBI_PM_CAP			0x40	/* :208 */
#define EX_DBI_EXP_CAP			0x70	/* :209 */
#define EX_DBI_L1SS_CONTROL		0x158	/* :214 */
#define  EX_PORT_LINK_TCOMMON_32US	(0x20 << 8)	/* :215 */
#define  EX_PORT_LINK_L1SS_ENABLE	0xf	/* :217 */
#define EX_DBI_L1SS_CONTROL2		0x15c	/* :216 */
#define  EX_PORT_LINK_TPOWERON_130US	0x69	/* :218 */
#define EX_DBI_MISC_CONTROL		0x8bc	/* :226; DBI_RO_WR_EN = 1 (:227) */
#define  DBI_RO_WR_EN			0x1	/* :227 */
#define EX_DBI_AUX_CLK_FREQ		0xb40	/* :229; 26 MHz = 0x1a (:231) */
#define  PCIE_AUX_CLK_FREQ_26MHZ	0x1a	/* :231 */
#define EX_DBI_L1_SUBSTATES		0xb44	/* :232; value 0xea (:233) */
#define  EX_DBI_L1_SUB_VAL		0xea	/* :233 */
#define EX_RC_DEVICE_ID			0xecec	/* PCI_DEVICE_ID_EXYNOS :25 (+ ch 0) */

/* EP config-space offsets written by stock L1SS code (BCM4361 caps), pci-exynos.c:123-128,1507-1527 */
#define EX_EP_LNKCTL			0xbc	/* [1:0] ASPM; enable |= 0x142 after &= ~3 */
#define EX_EP_L1SS_CTL1			0x248	/* enable |= 0xa0f, disable &= ~0xf */
#define EX_EP_L1SS_CTL2			0x24c	/* := 0x69 (T_POWER_ON 130 us) */
#define EX_EP_LTR_1B4			0x1b4	/* := 0x10031003 */
#define EX_EP_DEVCTL2			0xd4	/* |= BIT(10) */

/* Memory map (D4) */
#define EX_MEM_BASE			0x11800000	/* FDT:8231 */
#define EX_MEM_SIZE			0x400000	/* FDT:8231 */
#define EX_MEM_ATU1_SIZE		(EX_MEM_SIZE - 0x100000)	/* pci-exynos.c:571-572 "- 1 - 0x100000" */
#define EX_BAR0_CPU			0x11b00000	/* pci-exynos.c:552 */
#define EX_BAR0_PCI			0x11c00000	/* :555, cal:263 */
#define EX_BAR0_ATU_SIZE		0x10000	/* :554 limit 0x11B0ffff */
#define EX_BRCM_BAR2_SIZE		0x300000	/* cal:246-247 */

enum ex_state {
	EX_LINK_DOWN,
	EX_LINK_UP_TRY,
	EX_LINK_DOWN_TRY,
	EX_LINK_UP
};

struct exynos8895_pcie {
	struct dw_pcie pci;
	void __iomem *phy, *pcs;
	struct regmap *pmu, *sysreg;
	u32 sysreg_off;
	struct gpio_desc *perst;
	struct clk_bulk_data *clks;
	int nclks;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default, *pins_clkreq;
	int irq;
	enum ex_state state;
	bool probe_ok;
	bool atu_ok;
	bool irq_on;
	bool no_atu;			/* post_init: < 3 outbound iATU windows, power-on refused */
	int l1ss_ctrl_id_state;
	int boot_cnt;
	int work_l1ss_cnt;
	int linkdown_cnt;
	spinlock_t conf_lock;
	struct mutex pwr_lock;
	struct pci_dev *rp;
	struct pci_saved_state *rp_saved;
	struct pci_dev *ep;
	struct exynos_pcie_register_event *event_reg;
	struct workqueue_struct *wq;
	struct work_struct linkdown_work;
	struct delayed_work l1ss_boot_work;
	struct notifier_block panic_nb;	/* stock ss_dma_mon_nb */
};

#define to_ex(p) container_of((p), struct exynos8895_pcie, pci)

static inline void ex_elbi_wr(struct exynos8895_pcie *ep, u32 reg, u32 val)
{
	writel(val, ep->pci.elbi_base + reg);
}

static inline u32 ex_elbi_rd(struct exynos8895_pcie *ep, u32 reg)
{
	return readl(ep->pci.elbi_base + reg);
}

void ex_phy_config(struct exynos8895_pcie *ep);
void ex_phy_all_pwrdn(struct exynos8895_pcie *ep);
void ex_phy_all_pwrdn_clear(struct exynos8895_pcie *ep);

#endif /* __PCIE_EXYNOS8895_H */
