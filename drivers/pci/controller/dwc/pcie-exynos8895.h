/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Samsung Exynos8895 PCIe host controller
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

/* ELBI */
#define EX_ELBI_IRQ_PULSE		0x000
#define  EX_IRQ_INTA_ASSERT		BIT(0)
#define  EX_IRQ_INTB_ASSERT		BIT(2)
#define  EX_IRQ_INTC_ASSERT		BIT(4)
#define  EX_IRQ_INTD_ASSERT		BIT(6)
#define  EX_IRQ_RADM_PM_TO_ACK		BIT(18)
#define  EX_IRQ_LINK_DOWN		BIT(30)
#define EX_ELBI_IRQ_LEVEL		0x004
#define EX_ELBI_IRQ_SPECIAL		0x008
#define EX_ELBI_IRQ_EN_PULSE		0x00c
#define  EX_IRQ_LINKDOWN_ENABLE		BIT(30)
#define EX_ELBI_IRQ_EN_LEVEL		0x010
#define EX_ELBI_IRQ_EN_SPECIAL		0x014
#define EX_ELBI_SW_WAKE			0x018
#define  EX_SW_WAKE_BUS_NUM_EN		BIT(1)	/* cleared to enable */
#define EX_ELBI_APP_LTSSM_ENABLE	0x02c
#define EX_ELBI_IRQ_TOE_EN_PULSE2	0x034
#define EX_ELBI_IRQ_TOE_EN_LEVEL	0x038
#define EX_ELBI_L1_BUG_FIX_ENABLE	0x038	/* same offset as IRQ_TOE_EN_LEVEL */
#define EX_ELBI_APP_REQ_EXIT_L1		0x040
#define EX_ELBI_CXPL_DEBUG_INFO_H	0x070
#define EX_ELBI_RDLH_LINKUP		0x074
#define  EX_LTSSM_STATE_MASK		0x1f
#define EX_ELBI_PM_DSTATE		0x088
#define  EX_PM_DSTATE_MASK		0x7
/* PME_Turn_Off message request block, unnamed in the vendor driver */
#define EX_ELBI_PME_A8			0x0a8
#define EX_ELBI_PME_AC			0x0ac
#define EX_ELBI_PME_B0			0x0b0
#define EX_ELBI_PME_D0			0x0d0
#define EX_ELBI_APP_REQ_EXIT_L1_MODE	0x0f4
#define  EX_APP_REQ_EXIT_L1_MODE	BIT(0)
#define  EX_L1_REQ_NAK_CONTROL_MASTER	BIT(4)
#define EX_ELBI_HISTORY_REG(x)		(0x138 + (x) * 4)
#define EX_ELBI_LINKDOWN_RST_CTRL_SEL	0x1b8
#define  EX_LINKDOWN_RST_MANUAL		BIT(1)
#define EX_ELBI_SOFT_CORE_RESET		0x1d0
#define EX_ELBI_STATE_HISTORY_CHECK	0x274
#define  EX_HISTORY_BUFFER_ENABLE	BIT(0)
#define EX_ELBI_PHY_PCS_GRST		0x288
#define EX_ELBI_PHY_CMN_RST		0x28c
#define EX_ELBI_MAC_RST			0x290
#define EX_ELBI_QCH_SEL			0x2c8
#define  EX_QCH_SEL_MASK		0xfff
#define EX_ELBI_DMA_MONITOR(n)		(0x2cc + (n) * 4)
#define EX_ELBI_DUMP_END		0x2c4

#define EX_LTSSM_MIN			0x0d
#define EX_LTSSM_MAX			0x14
#define EX_LTSSM_L23_READY		0x15
#define EX_LINK_POLL_MAX		2000
#define EX_LINK_TRIES			10
#define EX_L1SS_BOOT_DELAY_MS		40000

/* PMU */
#define EX_PMU_WAKEUP_MASK		0x610
#define  EX_WAKEUP_MASK_PCIE_WIFI	5
#define EX_PMU_PCIE_PHY_CONTROL		0x71c
#define  EX_PCIE_PHY_CONTROL_MASK	0x1

/* FSYS1 sysreg, relative to the samsung,fsys-sysreg offset */
#define EX_SYSREG_PHY0			0x0
#define EX_SYSREG_WIFI0_PCIE_PHY_CONTROL 0xc
#define  EX_BIFURCATION_MODE_DISABLE	BIT(16)
#define  EX_LINK0_ENABLE		BIT(14)
#define  EX_PCS_LANE0_ENABLE		BIT(12)

/* PHY: one byte per 32-bit register */
#define EX_PHY_REG(n)			((n) * 4)
#define EX_PHY_CMN_BASE			0x00
#define EX_PHY_CMN_PWRDN		0x20
#define EX_PHY_TRSV_BASE		0x30
#define EX_PHY_TRSV0_PWRDN		0x57
#define EX_PHY_TX_AMPLITUDE		0x5c
#define EX_PHY_TRSV1_PWRDN		0xa7
#define EX_PHY_DUMP_END			0x180

/* PCS */
#define EX_PCS_L1SS_TIMEOUT		0x0c
#define EX_PCS_RX_ELECIDLE		0xec
#define  EX_PCS_RX_ELECIDLE_IGNORE	BIT(3)
#define EX_PCS_TX_LATENCY		0xf8
#define EX_PCS_REFCLK_OUT0		0x100
#define EX_PCS_REFCLK_OUT1		0x104

/* DBI */
#define EX_DBI_PM_CAP			0x40
#define EX_DBI_EXP_CAP			0x70
#define EX_DBI_AER_CAP			0x100
#define EX_DBI_L1SS_CONTROL		0x158
#define  EX_PORT_LINK_TCOMMON_32US	(0x20 << 8)
#define  EX_PORT_LINK_L1SS_ENABLE	0xf
#define EX_DBI_L1SS_CONTROL2		0x15c
#define  EX_PORT_LINK_TPOWERON_130US	0x69
#define EX_DBI_AUX_CLK_FREQ		0xb40
#define  EX_AUX_CLK_FREQ_26MHZ		0x1a
#define EX_DBI_L1_SUBSTATES		0xb44
#define  EX_DBI_L1_SUB_VAL		0xea
#define EX_RC_DEVICE_ID			0xecec

/* Endpoint config space offsets programmed by the vendor L1SS code (BCM4361) */
#define EX_EP_LNKCTL			0xbc
#define EX_EP_L1SS_CTL1			0x248
#define EX_EP_L1SS_CTL2			0x24c
#define EX_EP_LTR_1B4			0x1b4
#define EX_EP_DEVCTL2			0xd4

/* Fixed memory layout of the vendor driver */
#define EX_MEM_BASE			0x11800000
#define EX_MEM_SIZE			0x400000
#define EX_MEM_ATU1_SIZE		(EX_MEM_SIZE - 0x100000)
#define EX_BAR0_CPU			0x11b00000
#define EX_BAR0_PCI			0x11c00000
#define EX_BAR0_ATU_SIZE		0x10000
#define EX_BRCM_BAR2_SIZE		0x300000

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
	bool no_atu;
	int l1ss_ctrl_id_state;
	int boot_cnt;
	int work_l1ss_cnt;
	int linkdown_cnt;
	spinlock_t conf_lock;		/* link state vs. endpoint L1SS config */
	struct mutex pwr_lock;		/* power on/off */
	struct pci_dev *rp;
	struct pci_saved_state *rp_saved;
	struct pci_dev *ep;
	struct exynos_pcie_register_event *event_reg;
	struct workqueue_struct *wq;
	struct work_struct linkdown_work;
	struct delayed_work l1ss_boot_work;
	struct notifier_block panic_nb;
};

#define to_ex(p) container_of((p), struct exynos8895_pcie, pci)

static inline void ex_elbi_wr(struct exynos8895_pcie *pcie, u32 reg, u32 val)
{
	writel(val, pcie->pci.elbi_base + reg);
}

static inline u32 ex_elbi_rd(struct exynos8895_pcie *pcie, u32 reg)
{
	return readl(pcie->pci.elbi_base + reg);
}

void ex_phy_config(struct exynos8895_pcie *pcie);
void ex_phy_all_pwrdn(struct exynos8895_pcie *pcie);
void ex_phy_all_pwrdn_clear(struct exynos8895_pcie *pcie);

#endif /* __PCIE_EXYNOS8895_H */
