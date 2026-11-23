// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung Exynos8895 PCIe host controller
 *
 * Copyright (C) 2013 Samsung Electronics Co., Ltd.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/exynos-pci-ctrl.h>
#include <linux/exynos-pci-noti.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/panic_notifier.h>
#include <linux/pci.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

#include "pcie-designware.h"
#include "pcie-exynos8895.h"

static bool ex_l1ss_allowed;
module_param_named(l1ss, ex_l1ss_allowed, bool, 0444);
MODULE_PARM_DESC(l1ss, "Enable PCIe L1SS (default: 0)");

static struct exynos8895_pcie *ex_g;

/* DBI accesses need the PCS to ignore Rx electrical idle while the link is down */
static void ex_pcs_rx_elecidle(struct exynos8895_pcie *pcie, int val)
{
	u32 reg = readl(pcie->pcs + EX_PCS_RX_ELECIDLE);

	reg &= ~EX_PCS_RX_ELECIDLE_IGNORE;
	if (val)
		reg |= EX_PCS_RX_ELECIDLE_IGNORE;
	writel(reg, pcie->pcs + EX_PCS_RX_ELECIDLE);
}

static bool ex_link_is_up_hw(struct exynos8895_pcie *pcie)
{
	u32 val = ex_elbi_rd(pcie, EX_ELBI_RDLH_LINKUP) & EX_LTSSM_STATE_MASK;

	return (val >= EX_LTSSM_MIN && val <= EX_LTSSM_MAX);
}

static u32 ex_dw_read_dbi(struct dw_pcie *pci, void __iomem *base, u32 reg, size_t size)
{
	struct exynos8895_pcie *pcie = to_ex(pci);
	bool linked = ex_link_is_up_hw(pcie);
	u32 val = 0;

	if (!linked)
		ex_pcs_rx_elecidle(pcie, 1);
	dw_pcie_read(base + reg, size, &val);
	if (!linked)
		ex_pcs_rx_elecidle(pcie, 0);

	return val;
}

static void ex_dw_write_dbi(struct dw_pcie *pci, void __iomem *base, u32 reg, size_t size, u32 val)
{
	struct exynos8895_pcie *pcie = to_ex(pci);
	bool linked = ex_link_is_up_hw(pcie);

	if (!linked)
		ex_pcs_rx_elecidle(pcie, 1);
	dw_pcie_write(base + reg, size, val);
	if (!linked)
		ex_pcs_rx_elecidle(pcie, 0);
}

static bool ex_dw_link_up(struct dw_pcie *pci)
{
	struct exynos8895_pcie *pcie = to_ex(pci);

	return pcie->state == EX_LINK_UP;
}

static const struct dw_pcie_ops ex_dw_ops = {
	.read_dbi = ex_dw_read_dbi,
	.write_dbi = ex_dw_write_dbi,
	.link_up = ex_dw_link_up,
};

static int ex_own_conf_read(struct pci_bus *bus, unsigned int devfn, int where, int size, u32 *val)
{
	struct dw_pcie_rp *pp = bus->sysdata;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct exynos8895_pcie *pcie = to_ex(pci);
	bool linked = ex_link_is_up_hw(pcie);
	int ret;

	if (!linked)
		ex_pcs_rx_elecidle(pcie, 1);
	ret = pci_generic_config_read(bus, devfn, where, size, val);
	if (!linked)
		ex_pcs_rx_elecidle(pcie, 0);

	return ret;
}

static int ex_own_conf_write(struct pci_bus *bus, unsigned int devfn, int where, int size, u32 val)
{
	struct dw_pcie_rp *pp = bus->sysdata;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct exynos8895_pcie *pcie = to_ex(pci);
	bool linked = ex_link_is_up_hw(pcie);
	int ret;

	if (!linked)
		ex_pcs_rx_elecidle(pcie, 1);
	ret = pci_generic_config_write(bus, devfn, where, size, val);
	if (!linked)
		ex_pcs_rx_elecidle(pcie, 0);

	return ret;
}

static struct pci_ops ex_own_pci_ops = {
	.map_bus = dw_pcie_own_conf_map_bus,
	.read = ex_own_conf_read,
	.write = ex_own_conf_write,
};

static void ex_enable_interrupts(struct exynos8895_pcie *pcie)
{
	u32 val = EX_IRQ_INTA_ASSERT | EX_IRQ_INTB_ASSERT |
		  EX_IRQ_INTC_ASSERT | EX_IRQ_INTD_ASSERT;

	ex_elbi_wr(pcie, EX_ELBI_IRQ_EN_PULSE, val);
	ex_elbi_wr(pcie, EX_ELBI_IRQ_TOE_EN_PULSE2, 0x0);
	ex_elbi_wr(pcie, EX_ELBI_IRQ_TOE_EN_LEVEL, 0x0);
	ex_elbi_wr(pcie, EX_ELBI_IRQ_EN_LEVEL, 0x0);
	ex_elbi_wr(pcie, EX_ELBI_IRQ_EN_SPECIAL, 0x0);
}

static void ex_assert_phy_reset(struct exynos8895_pcie *pcie)
{
	ex_phy_config(pcie);
	ex_elbi_wr(pcie, EX_ELBI_SW_WAKE,
		   ex_elbi_rd(pcie, EX_ELBI_SW_WAKE) & ~EX_SW_WAKE_BUS_NUM_EN);
}

static void ex_resumed_phydown(struct exynos8895_pcie *pcie)
{
	ex_enable_interrupts(pcie);
	regmap_update_bits(pcie->pmu, EX_PMU_PCIE_PHY_CONTROL, EX_PCIE_PHY_CONTROL_MASK, 1);
	ex_assert_phy_reset(pcie);
	ex_phy_all_pwrdn(pcie);
}

static void ex_setup_rc(struct exynos8895_pcie *pcie)
{
	struct dw_pcie *pci = &pcie->pci;
	u32 val;

	dw_pcie_writel_dbi(pci, PCIE_MISC_CONTROL_1_OFF, PCIE_DBI_RO_WR_EN);
	dw_pcie_writew_dbi(pci, PCI_VENDOR_ID, PCI_VENDOR_ID_SAMSUNG);
	dw_pcie_writew_dbi(pci, PCI_DEVICE_ID, EX_RC_DEVICE_ID);

	val = dw_pcie_readl_dbi(pci, EX_DBI_EXP_CAP + PCI_EXP_LNKCAP);
	val &= ~(PCI_EXP_LNKCAP_L1EL | PCI_EXP_LNKCAP_MLW | PCI_EXP_LNKCAP_SLS);
	val |= FIELD_PREP(PCI_EXP_LNKCAP_L1EL, 7) | FIELD_PREP(PCI_EXP_LNKCAP_MLW, 1) |
	       PCI_EXP_LNKCAP_SLS_5_0GB;
	dw_pcie_writel_dbi(pci, EX_DBI_EXP_CAP + PCI_EXP_LNKCAP, val);

	dw_pcie_writel_dbi(pci, EX_DBI_AUX_CLK_FREQ, EX_AUX_CLK_FREQ_26MHZ);
	dw_pcie_writel_dbi(pci, EX_DBI_L1_SUBSTATES, EX_DBI_L1_SUB_VAL);
	dw_pcie_writel_dbi(pci, EX_DBI_PM_CAP + PCI_PM_CTRL, 0x0);

	val = dw_pcie_readl_dbi(pci, EX_DBI_EXP_CAP + PCI_EXP_LNKCTL);
	val |= PCI_EXP_LNKCTL_RL;
	dw_pcie_writel_dbi(pci, EX_DBI_EXP_CAP + PCI_EXP_LNKCTL, val);

	val = dw_pcie_readl_dbi(pci, EX_DBI_EXP_CAP + PCI_EXP_LNKCTL2);
	val = (val & ~PCI_EXP_LNKCTL2_TLS) | PCI_EXP_LNKCTL2_TLS_2_5GT;
	dw_pcie_writel_dbi(pci, EX_DBI_EXP_CAP + PCI_EXP_LNKCTL2, val);

	regmap_update_bits(pcie->sysreg, pcie->sysreg_off + EX_SYSREG_WIFI0_PCIE_PHY_CONTROL,
			   EX_BIFURCATION_MODE_DISABLE | EX_LINK0_ENABLE | EX_PCS_LANE0_ENABLE,
			   EX_LINK0_ENABLE | EX_PCS_LANE0_ENABLE);
}

static void ex_prog_atu(struct exynos8895_pcie *pcie)
{
	struct dw_pcie *pci = &pcie->pci;
	struct dw_pcie_ob_atu_cfg atu1 = {
		.index = 1,
		.type = PCIE_ATU_TYPE_MEM,
		.parent_bus_addr = EX_MEM_BASE,
		.pci_addr = EX_MEM_BASE,
		.size = EX_MEM_ATU1_SIZE,
	};
	struct dw_pcie_ob_atu_cfg atu2 = {
		.index = 2,
		.type = PCIE_ATU_TYPE_MEM,
		.parent_bus_addr = EX_BAR0_CPU,
		.pci_addr = EX_BAR0_PCI,
		.size = EX_BAR0_ATU_SIZE,
	};

	dw_pcie_prog_outbound_atu(pci, &atu1);
	dw_pcie_prog_outbound_atu(pci, &atu2);
}

static void ex_print_link_history(struct exynos8895_pcie *pcie)
{
	u32 history_buffer[32];
	int i;

	for (i = 31; i >= 0; i--)
		history_buffer[i] = ex_elbi_rd(pcie, EX_ELBI_HISTORY_REG(i));

	for (i = 31; i >= 0; i--)
		dev_info(pcie->pci.dev, "LTSSM: 0x%02x, L1sub: 0x%x, D state: 0x%x\n",
			 (history_buffer[i] >> 16) & 0x3f,
			 history_buffer[i] & 0x7,
			 (history_buffer[i] >> 8) & 0x7);
}

static int exynos_pcie_establish_link(struct exynos8895_pcie *pcie)
{
	struct dw_pcie *pci = &pcie->pci;
	int tries = 0;
	int n;
	u32 v;

retry:
	ex_pcs_rx_elecidle(pcie, 1);
	ex_elbi_wr(pcie, EX_ELBI_SOFT_CORE_RESET, 0);
	udelay(20);
	ex_elbi_wr(pcie, EX_ELBI_SOFT_CORE_RESET, 1);

	gpiod_set_value(pcie->perst, 0);
	usleep_range(18000, 20000);

	v = ex_elbi_rd(pcie, EX_ELBI_APP_REQ_EXIT_L1_MODE);
	v |= EX_APP_REQ_EXIT_L1_MODE | EX_L1_REQ_NAK_CONTROL_MASTER;
	ex_elbi_wr(pcie, EX_ELBI_APP_REQ_EXIT_L1_MODE, v);

	ex_elbi_wr(pcie, EX_ELBI_LINKDOWN_RST_CTRL_SEL, EX_LINKDOWN_RST_MANUAL);

	v = ex_elbi_rd(pcie, EX_ELBI_QCH_SEL);
	v &= ~EX_QCH_SEL_MASK;
	ex_elbi_wr(pcie, EX_ELBI_QCH_SEL, v);

	ex_assert_phy_reset(pcie);

	ex_elbi_wr(pcie, EX_ELBI_L1_BUG_FIX_ENABLE, 1);

	dw_pcie_setup_rc(&pci->pp);
	ex_setup_rc(pcie);
	ex_prog_atu(pcie);

	ex_pcs_rx_elecidle(pcie, 0);

	dev_dbg(pci->dev, "D state %x, LTSSM %#x\n",
		ex_elbi_rd(pcie, EX_ELBI_PM_DSTATE) & EX_PM_DSTATE_MASK,
		ex_elbi_rd(pcie, EX_ELBI_RDLH_LINKUP));

	ex_elbi_wr(pcie, EX_ELBI_APP_LTSSM_ENABLE, 1);

	for (n = 0; n < EX_LINK_POLL_MAX; n++) {
		v = ex_elbi_rd(pcie, EX_ELBI_RDLH_LINKUP) & EX_LTSSM_STATE_MASK;
		if (v >= EX_LTSSM_MIN && v <= EX_LTSSM_MAX)
			break;
		udelay(10);
	}

	if (n >= EX_LINK_POLL_MAX) {
		tries++;
		dev_err(pci->dev, "link not up, try %d, LTSSM %#x\n",
			tries, ex_elbi_rd(pcie, EX_ELBI_RDLH_LINKUP));
		if (tries < EX_LINK_TRIES) {
			gpiod_set_value(pcie->perst, 1);
			ex_elbi_wr(pcie, EX_ELBI_APP_LTSSM_ENABLE, 0);
			goto retry;
		}
		ex_print_link_history(pcie);
		return -EPIPE;
	}

	v = ex_elbi_rd(pcie, EX_ELBI_RDLH_LINKUP) & EX_LTSSM_STATE_MASK;
	if (v < EX_LTSSM_MIN || v > EX_LTSSM_MAX) {
		dev_warn(pci->dev, "link lost after training, LTSSM %#x\n", v);
		gpiod_set_value(pcie->perst, 1);
		ex_elbi_wr(pcie, EX_ELBI_APP_LTSSM_ENABLE, 0);
		goto retry;
	}

	dev_info(pci->dev, "link up, LTSSM %#x, %d tries\n", v, tries + 1);

	ex_elbi_wr(pcie, EX_ELBI_IRQ_PULSE, ex_elbi_rd(pcie, EX_ELBI_IRQ_PULSE));
	ex_elbi_wr(pcie, EX_ELBI_IRQ_EN_PULSE,
		   ex_elbi_rd(pcie, EX_ELBI_IRQ_EN_PULSE) | EX_IRQ_LINKDOWN_ENABLE);

	return 0;
}

static void exynos_pcie_notify_callback(struct exynos8895_pcie *pcie, int event)
{
	if (pcie->event_reg && pcie->event_reg->callback &&
	    (pcie->event_reg->events & event)) {
		struct exynos_pcie_notify *notify = &pcie->event_reg->notify;

		notify->event = event;
		notify->user = pcie->event_reg->user;
		pcie->event_reg->callback(notify);
	} else {
		dev_dbg(pcie->pci.dev, "no client registered for event %d\n", event);
	}
}

static void ex_linkdown_work(struct work_struct *work)
{
	struct exynos8895_pcie *pcie = container_of(work, struct exynos8895_pcie, linkdown_work);

	if (pcie->state == EX_LINK_DOWN)
		return;

	pcie->linkdown_cnt++;
	dev_info(pcie->pci.dev, "link down, recovery %d\n", pcie->linkdown_cnt);
	exynos_pcie_notify_callback(pcie, EXYNOS_PCIE_EVENT_LINKDOWN);
}

static int exynos_pcie_set_l1ss(struct exynos8895_pcie *pcie, int enable, int id)
{
	struct dw_pcie *pci = &pcie->pci;
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&pcie->conf_lock, flags);
	if (pcie->state != EX_LINK_UP || !pcie->atu_ok) {
		if (enable)
			pcie->l1ss_ctrl_id_state &= ~id;
		else
			pcie->l1ss_ctrl_id_state |= id;
		spin_unlock_irqrestore(&pcie->conf_lock, flags);
		dev_dbg(pci->dev, "L1SS deferred until link up (mask %#x, id %#x)\n",
			pcie->l1ss_ctrl_id_state, id);
		return -1;
	}

	if (enable) {
		pcie->l1ss_ctrl_id_state &= ~id;
		if (pcie->l1ss_ctrl_id_state == 0) {
			if (ex_l1ss_allowed) {
				pci_read_config_dword(pcie->ep, EX_EP_LNKCTL, &val);
				val &= ~0x3;
				val |= 0x142;
				pci_write_config_dword(pcie->ep, EX_EP_LNKCTL, val);
				pci_read_config_dword(pcie->ep, EX_EP_L1SS_CTL1, &val);
				pci_write_config_dword(pcie->ep, EX_EP_L1SS_CTL1, val | 0xa0f);
				dev_dbg(pci->dev, "L1SS enabled (id %#x)\n", id);
			} else {
				dev_dbg(pci->dev, "L1SS not allowed (id %#x)\n", id);
			}
		} else {
			dev_dbg(pci->dev, "L1SS still held off (mask %#x, id %#x)\n",
				pcie->l1ss_ctrl_id_state, id);
		}
	} else if (enable == 0) {
		if (pcie->l1ss_ctrl_id_state) {
			pcie->l1ss_ctrl_id_state |= id;
			dev_dbg(pci->dev, "L1SS already off (mask %#x, id %#x)\n",
				pcie->l1ss_ctrl_id_state, id);
		} else {
			pcie->l1ss_ctrl_id_state |= id;
			pci_read_config_dword(pcie->ep, EX_EP_LNKCTL, &val);
			pci_write_config_dword(pcie->ep, EX_EP_LNKCTL, val & ~0x3);
			pci_read_config_dword(pcie->ep, EX_EP_L1SS_CTL1, &val);
			pci_write_config_dword(pcie->ep, EX_EP_L1SS_CTL1, val & ~0xf);
			dev_dbg(pci->dev, "L1SS disabled (id %#x)\n", id);
		}
	}
	spin_unlock_irqrestore(&pcie->conf_lock, flags);

	return 0;
}

static void ex_l1ss_boot_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct exynos8895_pcie *pcie = container_of(dwork, struct exynos8895_pcie, l1ss_boot_work);

	if (pcie->work_l1ss_cnt == 0) {
		exynos_pcie_set_l1ss(pcie, 1, PCIE_L1SS_CTRL_BOOT);
		pcie->work_l1ss_cnt++;
	}
}

static void exynos_pcie_config_l1ss(struct exynos8895_pcie *pcie)
{
	struct dw_pcie *pci = &pcie->pci;
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&pcie->conf_lock, flags);
	pcie->atu_ok = !!pcie->ep;
	if (pcie->atu_ok) {
		if (pcie->l1ss_ctrl_id_state == 0) {
			if (ex_l1ss_allowed) {
				pci_read_config_dword(pcie->ep, EX_EP_LNKCTL, &val);
				val &= ~0x3;
				val |= 0x142;
				pci_write_config_dword(pcie->ep, EX_EP_LNKCTL, val);
				pci_read_config_dword(pcie->ep, EX_EP_L1SS_CTL1, &val);
				pci_write_config_dword(pcie->ep, EX_EP_L1SS_CTL1, val | 0xa0f);
				dev_dbg(pci->dev, "L1SS enabled\n");
			} else {
				dev_dbg(pci->dev, "L1SS not allowed\n");
			}
		} else {
			pci_read_config_dword(pcie->ep, EX_EP_LNKCTL, &val);
			pci_write_config_dword(pcie->ep, EX_EP_LNKCTL, val & ~0x3);
			pci_read_config_dword(pcie->ep, EX_EP_L1SS_CTL1, &val);
			pci_write_config_dword(pcie->ep, EX_EP_L1SS_CTL1, val & ~0xf);
			dev_dbg(pci->dev, "L1SS held off (mask %#x)\n", pcie->l1ss_ctrl_id_state);
		}
		spin_unlock_irqrestore(&pcie->conf_lock, flags);

		pci_write_config_dword(pcie->ep, EX_EP_L1SS_CTL2, EX_PORT_LINK_TPOWERON_130US);
		pci_write_config_dword(pcie->ep, EX_EP_LTR_1B4, 0x10031003);
		pci_read_config_dword(pcie->ep, EX_EP_DEVCTL2, &val);
		pci_write_config_dword(pcie->ep, EX_EP_DEVCTL2, val | BIT(10));

		val = dw_pcie_readl_dbi(pci, EX_DBI_L1SS_CONTROL);
		val |= EX_PORT_LINK_TCOMMON_32US | EX_PORT_LINK_L1SS_ENABLE;
		dw_pcie_writel_dbi(pci, EX_DBI_L1SS_CONTROL, val);
		dw_pcie_writel_dbi(pci, EX_DBI_L1SS_CONTROL2, EX_PORT_LINK_TPOWERON_130US);
		dw_pcie_writel_dbi(pci, EX_DBI_L1_SUBSTATES, EX_DBI_L1_SUB_VAL);

		val = dw_pcie_readl_dbi(pci, EX_DBI_EXP_CAP + PCI_EXP_LNKCTL);
		val &= ~PCI_EXP_LNKCTL_ASPMC;
		val |= PCI_EXP_LNKCTL_CCC | PCI_EXP_LNKCTL_ASPM_L1;
		dw_pcie_writel_dbi(pci, EX_DBI_EXP_CAP + PCI_EXP_LNKCTL, val);
		dw_pcie_writel_dbi(pci, EX_DBI_EXP_CAP + PCI_EXP_DEVCTL2, PCI_EXP_DEVCTL2_LTR_EN);
	} else {
		spin_unlock_irqrestore(&pcie->conf_lock, flags);
	}
}

static void exynos_pcie_send_pme_turn_off(struct exynos8895_pcie *pcie)
{
	struct dw_pcie *pci = &pcie->pci;
	int count = 0;
	u32 val;

	val = ex_elbi_rd(pcie, EX_ELBI_RDLH_LINKUP) & EX_LTSSM_STATE_MASK;
	if (!(val >= EX_LTSSM_MIN && val <= EX_LTSSM_MAX)) {
		dev_dbg(pci->dev, "link not up (LTSSM %#x), no PME_Turn_Off\n", val);
		return;
	}

	ex_elbi_wr(pcie, EX_ELBI_APP_REQ_EXIT_L1, 0x1);
	val = ex_elbi_rd(pcie, EX_ELBI_APP_REQ_EXIT_L1_MODE);
	val &= ~EX_APP_REQ_EXIT_L1_MODE;
	val |= EX_L1_REQ_NAK_CONTROL_MASTER;
	ex_elbi_wr(pcie, EX_ELBI_APP_REQ_EXIT_L1_MODE, val);

	ex_elbi_wr(pcie, EX_ELBI_PME_A8, 0x0);
	ex_elbi_wr(pcie, EX_ELBI_PME_AC, 0x1);
	ex_elbi_wr(pcie, EX_ELBI_PME_B0, 0x13);
	ex_elbi_wr(pcie, EX_ELBI_PME_D0, 0x19);
	ex_elbi_wr(pcie, EX_ELBI_PME_A8, 0x1);

	while (count < EX_LINK_POLL_MAX) {
		if (ex_elbi_rd(pcie, EX_ELBI_IRQ_PULSE) & EX_IRQ_RADM_PM_TO_ACK)
			break;
		udelay(10);
		count++;
	}

	if (count >= EX_LINK_POLL_MAX)
		dev_err(pci->dev, "no PME_TO_Ack from the endpoint\n");

	ex_elbi_wr(pcie, EX_ELBI_APP_REQ_EXIT_L1, 0x0);

	do {
		val = ex_elbi_rd(pcie, EX_ELBI_RDLH_LINKUP) & EX_LTSSM_STATE_MASK;
		if (val == EX_LTSSM_L23_READY)
			break;
		udelay(10);
		count++;
	} while (count < EX_LINK_POLL_MAX);

	if (count >= EX_LINK_POLL_MAX)
		dev_err(pci->dev, "no PM_Enter_L23 from the endpoint\n");

	ex_elbi_wr(pcie, EX_ELBI_APP_REQ_EXIT_L1, 0x1);
	val = ex_elbi_rd(pcie, EX_ELBI_APP_REQ_EXIT_L1_MODE);
	val &= ~EX_APP_REQ_EXIT_L1_MODE;
	val |= EX_L1_REQ_NAK_CONTROL_MASTER;
	ex_elbi_wr(pcie, EX_ELBI_APP_REQ_EXIT_L1_MODE, val);
	ex_elbi_wr(pcie, EX_ELBI_PME_A8, 0x0);
	ex_elbi_wr(pcie, EX_ELBI_APP_REQ_EXIT_L1, 0x0);
}

static irqreturn_t ex_irq(int irq, void *arg)
{
	struct exynos8895_pcie *pcie = arg;
	struct dw_pcie *pci = &pcie->pci;
	u32 v, l, s;

	v = ex_elbi_rd(pcie, EX_ELBI_IRQ_PULSE);
	ex_elbi_wr(pcie, EX_ELBI_IRQ_PULSE, v);

	if (v & EX_IRQ_LINK_DOWN) {
		dev_err(pci->dev, "link down\n");
		ex_print_link_history(pcie);
		exynos_pcie_dump_link_down_status(0);
		exynos_pcie_register_dump(0);
		queue_work(pcie->wq, &pcie->linkdown_work);
	}

	l = ex_elbi_rd(pcie, EX_ELBI_IRQ_LEVEL);
	ex_elbi_wr(pcie, EX_ELBI_IRQ_LEVEL, l);

	s = ex_elbi_rd(pcie, EX_ELBI_IRQ_SPECIAL);
	ex_elbi_wr(pcie, EX_ELBI_IRQ_SPECIAL, s);

	return (v | l | s) ? IRQ_HANDLED : IRQ_NONE;
}

static void __ex_poweroff(struct exynos8895_pcie *pcie)
{
	struct dw_pcie *pci = &pcie->pci;
	unsigned long flags;
	u32 val;

	if (pcie->state == EX_LINK_UP || pcie->state == EX_LINK_DOWN_TRY) {
		pcie->state = EX_LINK_DOWN_TRY;

		if (pcie->irq_on) {
			disable_irq(pcie->irq);
			pcie->irq_on = false;
		}

		ex_elbi_wr(pcie, EX_ELBI_IRQ_EN_PULSE,
			   ex_elbi_rd(pcie, EX_ELBI_IRQ_EN_PULSE) & ~EX_IRQ_LINKDOWN_ENABLE);

		spin_lock_irqsave(&pcie->conf_lock, flags);
		exynos_pcie_send_pme_turn_off(pcie);
		pcie->state = EX_LINK_DOWN;

		val = ex_elbi_rd(pcie, EX_ELBI_STATE_HISTORY_CHECK);
		ex_elbi_wr(pcie, EX_ELBI_STATE_HISTORY_CHECK, val & ~EX_HISTORY_BUFFER_ENABLE);
		gpiod_set_value(pcie->perst, 1);
		ex_elbi_wr(pcie, EX_ELBI_APP_LTSSM_ENABLE, 0);
		ex_phy_all_pwrdn(pcie);
		spin_unlock_irqrestore(&pcie->conf_lock, flags);

		pcie->atu_ok = false;
		pinctrl_select_state(pcie->pinctrl, pcie->pins_clkreq);
		pm_runtime_put_sync(pci->dev);
	}

	regmap_update_bits(pcie->pmu, EX_PMU_WAKEUP_MASK,
			   BIT(EX_WAKEUP_MASK_PCIE_WIFI),
			   BIT(EX_WAKEUP_MASK_PCIE_WIFI));
}

static int __ex_poweron(struct exynos8895_pcie *pcie)
{
	struct dw_pcie *pci = &pcie->pci;
	struct dw_pcie_rp *pp = &pci->pp;
	struct pci_bus *child_bus;
	struct pci_dev *ep_dev;
	u32 val;
	int ret;

	if (pcie->no_atu)
		return -ENODEV;

	if (pcie->state == EX_LINK_DOWN) {
		regmap_update_bits(pcie->pmu, EX_PMU_WAKEUP_MASK, BIT(EX_WAKEUP_MASK_PCIE_WIFI), 0);
		pm_runtime_get_sync(pci->dev);
		pinctrl_select_state(pcie->pinctrl, pcie->pins_default);
		regmap_update_bits(pcie->pmu, EX_PMU_PCIE_PHY_CONTROL, EX_PCIE_PHY_CONTROL_MASK, 1);
		ex_phy_all_pwrdn_clear(pcie);
		pcie->state = EX_LINK_UP_TRY;
		val = ex_elbi_rd(pcie, EX_ELBI_STATE_HISTORY_CHECK);
		ex_elbi_wr(pcie, EX_ELBI_STATE_HISTORY_CHECK, val | EX_HISTORY_BUFFER_ENABLE);

		ret = exynos_pcie_establish_link(pcie);
		if (ret)
			goto fail;

		pcie->state = EX_LINK_UP;

		if (!pcie->probe_ok) {
			pci_lock_rescan_remove();
			pci_rescan_bus(pp->bridge->bus);
			pci_unlock_rescan_remove();

			pcie->rp = pci_get_slot(pp->bridge->bus, PCI_DEVFN(0, 0));
			child_bus = pci_find_bus(pci_domain_nr(pp->bridge->bus), 1);
			if (!child_bus) {
				dev_err(pci->dev, "child bus 1 not found\n");
				goto fail_first;
			}
			ep_dev = pci_get_slot(child_bus, PCI_DEVFN(0, 0));
			if (!ep_dev) {
				dev_err(pci->dev, "endpoint device 01:00.0 not found\n");
				goto fail_first;
			}
			pcie->ep = ep_dev;

			if (pci_resource_start(ep_dev, 0) != EX_BAR0_CPU ||
			    pci_resource_start(ep_dev, 2) != EX_MEM_BASE ||
			    pci_resource_len(ep_dev, 2) != EX_BRCM_BAR2_SIZE) {
				dev_err(pci->dev, "unexpected BAR layout: BAR0 %pR BAR2 %pR\n",
					&ep_dev->resource[0], &ep_dev->resource[2]);
				goto fail_first;
			}

			exynos_pcie_config_l1ss(pcie);

			if (pcie->rp) {
				pci_save_state(pcie->rp);
				pcie->rp_saved = pci_store_saved_state(pcie->rp);
			}
			pcie->probe_ok = true;
		} else {
			if (pcie->boot_cnt == 0) {
				schedule_delayed_work(&pcie->l1ss_boot_work,
						      msecs_to_jiffies(EX_L1SS_BOOT_DELAY_MS));
				pcie->boot_cnt++;
				pcie->l1ss_ctrl_id_state |= PCIE_L1SS_CTRL_BOOT;
			}
			exynos_pcie_config_l1ss(pcie);
			if (pcie->rp && pcie->rp_saved) {
				pci_load_saved_state(pcie->rp, pcie->rp_saved);
				pci_restore_state(pcie->rp);
			}
		}

		enable_irq(pcie->irq);
		pcie->irq_on = true;
	}

	return 0;

fail_first:
	pci_dev_put(pcie->ep);
	pcie->ep = NULL;
	pci_dev_put(pcie->rp);
	pcie->rp = NULL;
fail:
	pcie->state = EX_LINK_UP;
	__ex_poweroff(pcie);
	return -EPIPE;
}

int exynos_pcie_poweron(int ch)
{
	int ret;

	if (ch != 0 || !ex_g)
		return -ENODEV;

	mutex_lock(&ex_g->pwr_lock);
	ret = __ex_poweron(ex_g);
	mutex_unlock(&ex_g->pwr_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(exynos_pcie_poweron);

void exynos_pcie_poweroff(int ch)
{
	if (ch != 0 || !ex_g)
		return;

	mutex_lock(&ex_g->pwr_lock);
	__ex_poweroff(ex_g);
	mutex_unlock(&ex_g->pwr_lock);
}
EXPORT_SYMBOL_GPL(exynos_pcie_poweroff);

int exynos_pcie_pm_resume(int ch)
{
	return exynos_pcie_poweron(ch);
}
EXPORT_SYMBOL_GPL(exynos_pcie_pm_resume);

void exynos_pcie_pm_suspend(int ch)
{
	unsigned long flags;

	if (ch != 0 || !ex_g)
		return;

	if (ex_g->state == EX_LINK_DOWN) {
		dev_dbg(ex_g->pci.dev, "link already off\n");
		return;
	}

	spin_lock_irqsave(&ex_g->conf_lock, flags);
	ex_g->state = EX_LINK_DOWN_TRY;
	spin_unlock_irqrestore(&ex_g->conf_lock, flags);

	exynos_pcie_poweroff(ch);
}
EXPORT_SYMBOL_GPL(exynos_pcie_pm_suspend);

int exynos_pcie_l1ss_ctrl(int enable, int id)
{
	if (!ex_g)
		return -ENODEV;

	return exynos_pcie_set_l1ss(ex_g, enable, id);
}
EXPORT_SYMBOL_GPL(exynos_pcie_l1ss_ctrl);

int exynos_pcie_register_event(struct exynos_pcie_register_event *reg)
{
	struct pci_dev *pdev;
	struct dw_pcie_rp *pp;
	struct exynos8895_pcie *pcie;

	if (!reg) {
		pr_err("PCIe: Event registration is NULL\n");
		return -ENODEV;
	}
	if (!reg->user) {
		pr_err("PCIe: User of event registration is NULL\n");
		return -ENODEV;
	}

	pdev = reg->user;
	if (!pdev->bus || !pdev->bus->sysdata) {
		pr_err("PCIe: did not find RC for pci endpoint device\n");
		return -ENODEV;
	}
	pp = pdev->bus->sysdata;
	pcie = to_ex(to_dw_pcie_from_pp(pp));

	pcie->event_reg = reg;
	dev_dbg(pcie->pci.dev, "events %#x registered\n", reg->events);

	return 0;
}
EXPORT_SYMBOL_GPL(exynos_pcie_register_event);

int exynos_pcie_deregister_event(struct exynos_pcie_register_event *reg)
{
	struct pci_dev *pdev;
	struct dw_pcie_rp *pp;
	struct exynos8895_pcie *pcie;

	if (!reg) {
		pr_err("PCIe: Event deregistration is NULL\n");
		return -ENODEV;
	}
	if (!reg->user) {
		pr_err("PCIe: User of event deregistration is NULL\n");
		return -ENODEV;
	}

	pdev = reg->user;
	if (!pdev->bus || !pdev->bus->sysdata) {
		pr_err("PCIe: did not find RC for pci endpoint device\n");
		return -ENODEV;
	}
	pp = pdev->bus->sysdata;
	pcie = to_ex(to_dw_pcie_from_pp(pp));

	pcie->event_reg = NULL;
	dev_dbg(pcie->pci.dev, "events deregistered\n");

	return 0;
}
EXPORT_SYMBOL_GPL(exynos_pcie_deregister_event);

enum ex_dump_space {
	EX_DUMP_ELBI,
	EX_DUMP_PHY,
	EX_DUMP_DBI,
};

static u32 ex_dump_rd(struct exynos8895_pcie *pcie, enum ex_dump_space space, u32 off)
{
	switch (space) {
	case EX_DUMP_ELBI:
		return ex_elbi_rd(pcie, off);
	case EX_DUMP_PHY:
		return readl(pcie->phy + off);
	default:
		return dw_pcie_readl_dbi(&pcie->pci, off);
	}
}

static void ex_dump_regs(struct exynos8895_pcie *pcie, const char *name,
			 enum ex_dump_space space, u32 end)
{
	char line[4 * 9 + 1];
	u32 off, reg;
	int len;

	for (off = 0; off < end; off += 0x10) {
		len = 0;
		for (reg = off; reg < off + 0x10 && reg < end; reg += 4)
			len += scnprintf(line + len, sizeof(line) - len, " %08x",
					 ex_dump_rd(pcie, space, reg));
		dev_warn(pcie->pci.dev, "%s %04x:%s\n", name, off, line);
	}
}

void exynos_pcie_register_dump(int ch)
{
	struct dw_pcie *pci;
	u32 devctl, lnkctl, uncor, cor, root;

	if (ch != 0 || !ex_g)
		return;

	pci = &ex_g->pci;
	ex_dump_regs(ex_g, "ELBI", EX_DUMP_ELBI, EX_ELBI_DUMP_END);
	ex_dump_regs(ex_g, "PHY", EX_DUMP_PHY, EX_PHY_DUMP_END);
	ex_dump_regs(ex_g, "DBI", EX_DUMP_DBI, PCI_STD_HEADER_SIZEOF);

	devctl = dw_pcie_readl_dbi(pci, EX_DBI_EXP_CAP + PCI_EXP_DEVCTL);
	lnkctl = dw_pcie_readl_dbi(pci, EX_DBI_EXP_CAP + PCI_EXP_LNKCTL);
	uncor = dw_pcie_readl_dbi(pci, EX_DBI_AER_CAP + PCI_ERR_UNCOR_STATUS);
	cor = dw_pcie_readl_dbi(pci, EX_DBI_AER_CAP + PCI_ERR_COR_STATUS);
	root = dw_pcie_readl_dbi(pci, EX_DBI_AER_CAP + PCI_ERR_ROOT_STATUS);
	dev_warn(pci->dev, "DEVCTL %08x LNKCTL %08x AER UNCOR %08x COR %08x ROOT %08x\n",
		 devctl, lnkctl, uncor, cor, root);
}
EXPORT_SYMBOL_GPL(exynos_pcie_register_dump);

int exynos_pcie_dump_link_down_status(int ch)
{
	if (ch != 0 || !ex_g)
		return -ENODEV;

	if (ex_g->state == EX_LINK_UP) {
		dev_info(ex_g->pci.dev, "LTSSM: 0x%08x\n",
			 ex_elbi_rd(ex_g, EX_ELBI_RDLH_LINKUP));
		dev_info(ex_g->pci.dev, "LTSSM_H: 0x%08x\n",
			 ex_elbi_rd(ex_g, EX_ELBI_CXPL_DEBUG_INFO_H));
		dev_info(ex_g->pci.dev, "DMA_MONITOR1: 0x%08x\n",
			 ex_elbi_rd(ex_g, EX_ELBI_DMA_MONITOR(0)));
		dev_info(ex_g->pci.dev, "DMA_MONITOR2: 0x%08x\n",
			 ex_elbi_rd(ex_g, EX_ELBI_DMA_MONITOR(1)));
		dev_info(ex_g->pci.dev, "DMA_MONITOR3: 0x%08x\n",
			 ex_elbi_rd(ex_g, EX_ELBI_DMA_MONITOR(2)));
	} else {
		dev_info(ex_g->pci.dev, "PCIE link state is %d\n", ex_g->state);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(exynos_pcie_dump_link_down_status);

#ifdef CONFIG_PM_SLEEP
static int exynos_pcie_suspend_noirq(struct device *dev)
{
	struct exynos8895_pcie *pcie = dev_get_drvdata(dev);

	if (pcie->state == EX_LINK_DOWN) {
		dev_dbg(dev, "link already off\n");
		return 0;
	}

	exynos_pcie_send_pme_turn_off(pcie);
	gpiod_set_value(pcie->perst, 1);

	return 0;
}

static int exynos_pcie_resume_noirq(struct device *dev)
{
	struct exynos8895_pcie *pcie = dev_get_drvdata(dev);

	if (pcie->state == EX_LINK_DOWN) {
		ex_resumed_phydown(pcie);
		return 0;
	}

	ex_enable_interrupts(pcie);
	regmap_update_bits(pcie->pmu, EX_PMU_PCIE_PHY_CONTROL, EX_PCIE_PHY_CONTROL_MASK, 1);
	exynos_pcie_establish_link(pcie);
	exynos_pcie_config_l1ss(pcie);

	return 0;
}
#endif

static const struct dev_pm_ops exynos8895_pcie_pm_ops = {
	NOIRQ_SYSTEM_SLEEP_PM_OPS(exynos_pcie_suspend_noirq, exynos_pcie_resume_noirq)
};

static int exynos_pcie_dma_mon_event(struct notifier_block *nb, unsigned long event, void *data)
{
	int ret;

	/* Unlike the vendor driver, no register dump: it pushes the oops out of pstore */
	ret = exynos_pcie_dump_link_down_status(0);

	return notifier_from_errno(ret);
}

static void ex_destroy_wq(void *wq)
{
	destroy_workqueue(wq);
}

static int ex_host_init(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct exynos8895_pcie *pcie = to_ex(pci);
	struct resource *cfg = platform_get_resource_byname(to_platform_device(pci->dev),
							    IORESOURCE_MEM, "config");
	struct resource_entry *win;

	if (!cfg)
		return -EINVAL;

	win = resource_list_first_type(&pp->bridge->windows, IORESOURCE_MEM);
	if (!win || win->res->start != EX_MEM_BASE || resource_size(win->res) != EX_MEM_SIZE) {
		dev_err(pci->dev, "unexpected MEM window\n");
		return -EINVAL;
	}

	/* The config page lies inside the MEM window: release its claim, keep the mapping */
	devm_release_mem_region(pci->dev, cfg->start, resource_size(cfg));

	ex_setup_rc(pcie);

	pp->bridge->ops = &ex_own_pci_ops;

	return 0;
}

static int ex_host_msi_init(struct dw_pcie_rp *pp)
{
	return 0;
}

static void ex_host_post_init(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct exynos8895_pcie *pcie = to_ex(pci);

	if (pci->num_ob_windows < 3) {
		dev_err(pci->dev, "need 3 outbound iATU windows, have %u\n", pci->num_ob_windows);
		pcie->no_atu = true;
	}
}

static const struct dw_pcie_host_ops ex_host_ops = {
	.init = ex_host_init,
	.msi_init = ex_host_msi_init,
	.post_init = ex_host_post_init,
};

static int exynos8895_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct exynos8895_pcie *pcie;
	struct dw_pcie *pci;
	struct resource *res;
	int ret;

	pcie = devm_kzalloc(dev, sizeof(*pcie), GFP_KERNEL);
	if (!pcie)
		return -ENOMEM;

	pci = &pcie->pci;
	pci->dev = dev;
	pci->ops = &ex_dw_ops;
	pci->pp.ops = &ex_host_ops;
	pci->l1ss_support = true;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dbi");
	pci->dbi_base = devm_pci_remap_cfg_resource(dev, res);
	if (IS_ERR(pci->dbi_base))
		return PTR_ERR(pci->dbi_base);
	pci->dbi_phys_addr = res->start;

	pci->elbi_base = devm_platform_ioremap_resource_byname(pdev, "elbi");
	if (IS_ERR(pci->elbi_base))
		return PTR_ERR(pci->elbi_base);

	pcie->phy = devm_platform_ioremap_resource_byname(pdev, "phy");
	if (IS_ERR(pcie->phy))
		return PTR_ERR(pcie->phy);

	pcie->pcs = devm_platform_ioremap_resource_byname(pdev, "pcs");
	if (IS_ERR(pcie->pcs))
		return PTR_ERR(pcie->pcs);

	pcie->pmu = syscon_regmap_lookup_by_phandle(np, "samsung,pmu-syscon");
	if (IS_ERR(pcie->pmu))
		return PTR_ERR(pcie->pmu);

	pcie->sysreg = syscon_regmap_lookup_by_phandle_args(np, "samsung,fsys-sysreg", 1,
							    &pcie->sysreg_off);
	if (IS_ERR(pcie->sysreg))
		return PTR_ERR(pcie->sysreg);

	pcie->perst = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(pcie->perst))
		return PTR_ERR(pcie->perst);

	pcie->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(pcie->pinctrl))
		return PTR_ERR(pcie->pinctrl);

	pcie->pins_default = pinctrl_lookup_state(pcie->pinctrl, "default");
	if (IS_ERR(pcie->pins_default))
		return PTR_ERR(pcie->pins_default);

	pcie->pins_clkreq = pinctrl_lookup_state(pcie->pinctrl, "clkreq_output");
	if (IS_ERR(pcie->pins_clkreq))
		return PTR_ERR(pcie->pins_clkreq);

	pinctrl_select_state(pcie->pinctrl, pcie->pins_clkreq);

	pcie->nclks = devm_clk_bulk_get_all_enabled(dev, &pcie->clks);
	if (pcie->nclks < 0)
		return pcie->nclks;

	spin_lock_init(&pcie->conf_lock);
	mutex_init(&pcie->pwr_lock);

	pcie->wq = alloc_ordered_workqueue("pcie_wq", WQ_FREEZABLE);
	if (!pcie->wq)
		return -ENOMEM;
	ret = devm_add_action_or_reset(dev, ex_destroy_wq, pcie->wq);
	if (ret)
		return ret;

	INIT_WORK(&pcie->linkdown_work, ex_linkdown_work);
	INIT_DELAYED_WORK(&pcie->l1ss_boot_work, ex_l1ss_boot_work);

	pcie->irq = platform_get_irq(pdev, 0);
	if (pcie->irq < 0)
		return pcie->irq;

	ret = devm_request_irq(dev, pcie->irq, ex_irq, IRQF_SHARED, "exynos-pcie", pcie);
	if (ret)
		return ret;

	ex_resumed_phydown(pcie);
	regmap_update_bits(pcie->pmu, EX_PMU_PCIE_PHY_CONTROL, EX_PCIE_PHY_CONTROL_MASK, 1);

	pm_runtime_no_callbacks(dev);
	pm_suspend_ignore_children(dev, true);
	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	pci->pp.use_linkup_irq = true;

	ret = dw_pcie_host_init(&pci->pp);
	if (ret)
		return ret;

	disable_irq(pcie->irq);
	pcie->irq_on = false;

	platform_set_drvdata(pdev, pcie);
	ex_g = pcie;

	pcie->panic_nb.notifier_call = exynos_pcie_dma_mon_event;
	atomic_notifier_chain_register(&panic_notifier_list, &pcie->panic_nb);

	return 0;
}

static const struct of_device_id exynos8895_pcie_of_match[] = {
	{ .compatible = "samsung,exynos8895-pcie" },
	{ }
};

static struct platform_driver exynos8895_pcie_driver = {
	.probe = exynos8895_pcie_probe,
	.driver = {
		.name = "exynos8895-pcie",
		.of_match_table = exynos8895_pcie_of_match,
		.suppress_bind_attrs = true,
		.pm = &exynos8895_pcie_pm_ops,
	},
};
builtin_platform_driver(exynos8895_pcie_driver);

static bool ex_is_our_device(struct pci_dev *dev)
{
	struct pci_host_bridge *bridge = pci_find_host_bridge(dev->bus);

	if (!bridge || !bridge->dev.parent)
		return false;

	return of_device_is_compatible(bridge->dev.parent->of_node, "samsung,exynos8895-pcie");
}

static void ex_quirk_brcm_header(struct pci_dev *dev)
{
	if (!ex_is_our_device(dev))
		return;

	/* 3 MiB, so that BAR0 still fits in the 4 MiB MEM window */
	pci_resource_end(dev, 2) = pci_resource_start(dev, 2) + EX_BRCM_BAR2_SIZE - 1;
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_BROADCOM, PCI_ANY_ID, ex_quirk_brcm_header);

static void ex_quirk_brcm_enable(struct pci_dev *dev)
{
	int count = 0;
	u32 reg;

	if (!ex_is_our_device(dev))
		return;

	/* ATU 2 translates to this bus address; like the vendor driver, rewrite until it sticks */
	pci_write_config_dword(dev, PCI_BASE_ADDRESS_0, EX_BAR0_PCI);
	while (1) {
		pci_read_config_dword(dev, PCI_BASE_ADDRESS_0, &reg);
		reg &= PCI_BASE_ADDRESS_MEM_MASK;
		if (reg == EX_BAR0_PCI)
			break;
		pci_dbg(dev, "BAR0 reads back %#x, retry %d\n", reg, ++count);
		pci_write_config_dword(dev, PCI_BASE_ADDRESS_0, EX_BAR0_PCI);
		if (count > 100)
			break;
	}
}
DECLARE_PCI_FIXUP_ENABLE(PCI_VENDOR_ID_BROADCOM, PCI_ANY_ID, ex_quirk_brcm_enable);

static void ex_quirk_async(struct pci_dev *dev)
{
	if (!ex_is_our_device(dev))
		return;

	device_disable_async_suspend(&dev->dev);

	/* The driver programs L1SS itself, keep the PCI core from restoring it */
	dev->l1ss = 0;
}
DECLARE_PCI_FIXUP_FINAL(PCI_ANY_ID, PCI_ANY_ID, ex_quirk_async);

static void ex_quirk_rc_header(struct pci_dev *dev)
{
	int i;

	if (!ex_is_our_device(dev) || !pci_is_root_bus(dev->bus))
		return;

	/* The root port BARs are unused and must not take MEM window space */
	for (i = 0; i < 2; i++) {
		if (dev->resource[i].flags) {
			pci_dbg(dev, "RC BAR%d %pR unused, dropped\n", i, &dev->resource[i]);
			dev->resource[i].flags = 0;
		}
	}
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_SAMSUNG, EX_RC_DEVICE_ID, ex_quirk_rc_header);
