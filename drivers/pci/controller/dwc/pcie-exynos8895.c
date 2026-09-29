// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung Exynos8895 PCIe RC0 controller driver (DesignWare host glue)
 * Spec L6D 5.1.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/panic_notifier.h>
#include <linux/pci.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/workqueue.h>
#include <linux/exynos-pci-ctrl.h>
#include <linux/exynos-pci-noti.h>

#include "pcie-designware.h"
#include "pcie-exynos8895.h"

static bool ex_l1ss_allowed;
module_param_named(l1ss, ex_l1ss_allowed, bool, 0444);
MODULE_PARM_DESC(l1ss, "Enable PCIe L1SS (default: 0)");

static struct exynos8895_pcie *ex_g;

static void ex_pcs_rx_elecidle(struct exynos8895_pcie *ep, int val)
{
	u32 reg = readl(ep->pcs + EX_PCS_RX_ELECIDLE);

	reg &= ~BIT(3);
	if (val)
		reg |= BIT(3);
	writel(reg, ep->pcs + EX_PCS_RX_ELECIDLE);
}

static bool ex_link_is_up_hw(struct exynos8895_pcie *ep)
{
	u32 val = ex_elbi_rd(ep, EX_ELBI_RDLH_LINKUP) & 0x1f;

	return (val >= EX_LTSSM_MIN && val <= EX_LTSSM_MAX);
}

static u32 ex_dw_read_dbi(struct dw_pcie *pci, void __iomem *base, u32 reg, size_t size)
{
	struct exynos8895_pcie *ep = to_ex(pci);
	bool linked = ex_link_is_up_hw(ep);
	u32 val = 0;

	if (!linked)
		ex_pcs_rx_elecidle(ep, 1);
	dw_pcie_read(base + reg, size, &val);
	if (!linked)
		ex_pcs_rx_elecidle(ep, 0);

	return val;
}

static void ex_dw_write_dbi(struct dw_pcie *pci, void __iomem *base, u32 reg, size_t size, u32 val)
{
	struct exynos8895_pcie *ep = to_ex(pci);
	bool linked = ex_link_is_up_hw(ep);

	if (!linked)
		ex_pcs_rx_elecidle(ep, 1);
	dw_pcie_write(base + reg, size, val);
	if (!linked)
		ex_pcs_rx_elecidle(ep, 0);
}

static bool ex_dw_link_up(struct dw_pcie *pci)
{
	struct exynos8895_pcie *ep = to_ex(pci);

	return ep->state == EX_LINK_UP;
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
	struct exynos8895_pcie *ep = to_ex(pci);
	bool linked = ex_link_is_up_hw(ep);
	int ret;

	if (!linked)
		ex_pcs_rx_elecidle(ep, 1);
	ret = pci_generic_config_read(bus, devfn, where, size, val);
	if (!linked)
		ex_pcs_rx_elecidle(ep, 0);

	return ret;
}

static int ex_own_conf_write(struct pci_bus *bus, unsigned int devfn, int where, int size, u32 val)
{
	struct dw_pcie_rp *pp = bus->sysdata;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct exynos8895_pcie *ep = to_ex(pci);
	bool linked = ex_link_is_up_hw(ep);
	int ret;

	if (!linked)
		ex_pcs_rx_elecidle(ep, 1);
	ret = pci_generic_config_write(bus, devfn, where, size, val);
	if (!linked)
		ex_pcs_rx_elecidle(ep, 0);

	return ret;
}

static struct pci_ops ex_own_pci_ops = {
	.map_bus = dw_pcie_own_conf_map_bus,
	.read = ex_own_conf_read,
	.write = ex_own_conf_write,
};

static void ex_enable_interrupts(struct exynos8895_pcie *ep)
{
	u32 val = EX_IRQ_INTA_ASSERT | EX_IRQ_INTB_ASSERT |
		  EX_IRQ_INTC_ASSERT | EX_IRQ_INTD_ASSERT;

	ex_elbi_wr(ep, EX_ELBI_IRQ_EN_PULSE, val);
	ex_elbi_wr(ep, EX_ELBI_IRQ_TOE_EN_PULSE2, 0x0);
	ex_elbi_wr(ep, EX_ELBI_IRQ_TOE_EN_LEVEL, 0x0);
	ex_elbi_wr(ep, EX_ELBI_IRQ_EN_LEVEL, 0x0);
	ex_elbi_wr(ep, EX_ELBI_IRQ_EN_SPECIAL, 0x0);
}

static void ex_assert_phy_reset(struct exynos8895_pcie *ep)
{
	ex_phy_config(ep);
	ex_elbi_wr(ep, EX_ELBI_SW_WAKE, ex_elbi_rd(ep, EX_ELBI_SW_WAKE) & ~BIT(1));
}

static void ex_resumed_phydown(struct exynos8895_pcie *ep)
{
	ex_enable_interrupts(ep);
	regmap_update_bits(ep->pmu, EX_PMU_PCIE_PHY_CONTROL, EX_PCIE_PHY_CONTROL_MASK, 1);
	ex_assert_phy_reset(ep);
	ex_phy_all_pwrdn(ep);
}

static void ex_setup_rc(struct exynos8895_pcie *ep)
{
	struct dw_pcie *pci = &ep->pci;
	u32 val;

	dw_pcie_writel_dbi(pci, EX_DBI_MISC_CONTROL, DBI_RO_WR_EN);
	dw_pcie_writew_dbi(pci, PCI_VENDOR_ID, PCI_VENDOR_ID_SAMSUNG);
	dw_pcie_writew_dbi(pci, PCI_DEVICE_ID, EX_RC_DEVICE_ID);

	val = dw_pcie_readl_dbi(pci, EX_DBI_EXP_CAP + PCI_EXP_LNKCAP);
	val &= ~(PCI_EXP_LNKCAP_L1EL | PCI_EXP_LNKCAP_MLW | PCI_EXP_LNKCAP_SLS);
	val |= (0x7 << 15) | (0x1 << 4) | PCI_EXP_LNKCAP_SLS_5_0GB;
	dw_pcie_writel_dbi(pci, EX_DBI_EXP_CAP + PCI_EXP_LNKCAP, val);

	dw_pcie_writel_dbi(pci, EX_DBI_AUX_CLK_FREQ, PCIE_AUX_CLK_FREQ_26MHZ);
	dw_pcie_writel_dbi(pci, EX_DBI_L1_SUBSTATES, EX_DBI_L1_SUB_VAL);
	dw_pcie_writel_dbi(pci, EX_DBI_PM_CAP + PCI_PM_CTRL, 0x0);

	val = dw_pcie_readl_dbi(pci, EX_DBI_EXP_CAP + PCI_EXP_LNKCTL);
	val |= PCI_EXP_LNKCTL_RL;
	dw_pcie_writel_dbi(pci, EX_DBI_EXP_CAP + PCI_EXP_LNKCTL, val);

	val = dw_pcie_readl_dbi(pci, EX_DBI_EXP_CAP + PCI_EXP_LNKCTL2);
	val = (val & ~0xf) | 0x1;
	dw_pcie_writel_dbi(pci, EX_DBI_EXP_CAP + PCI_EXP_LNKCTL2, val);

	regmap_update_bits(ep->sysreg, ep->sysreg_off + EX_SYSREG_WIFI0_PCIE_PHY_CONTROL,
			   EX_BIFURCATION_MODE_DISABLE | EX_LINK0_ENABLE | EX_PCS_LANE0_ENABLE,
			   EX_LINK0_ENABLE | EX_PCS_LANE0_ENABLE);
}

static void ex_prog_atu(struct exynos8895_pcie *ep)
{
	struct dw_pcie *pci = &ep->pci;
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

static void ex_print_link_history(struct exynos8895_pcie *ep)
{
	u32 history_buffer[32];
	int i;

	for (i = 31; i >= 0; i--)
		history_buffer[i] = ex_elbi_rd(ep, EX_ELBI_HISTORY_REG(i));

	for (i = 31; i >= 0; i--)
		dev_info(ep->pci.dev, "LTSSM: 0x%02x, L1sub: 0x%x, D state: 0x%x\n",
			 (history_buffer[i] >> 16) & 0x3f,
			 history_buffer[i] & 0x7,
			 (history_buffer[i] >> 8) & 0x7);
}

static int exynos_pcie_establish_link(struct exynos8895_pcie *ep)
{
	struct dw_pcie *pci = &ep->pci;
	int tries = 0;
	int n;
	u32 v;

retry:
	ex_pcs_rx_elecidle(ep, 1);
	ex_elbi_wr(ep, EX_ELBI_SOFT_CORE_RESET, 0);
	udelay(20);
	ex_elbi_wr(ep, EX_ELBI_SOFT_CORE_RESET, 1);

	gpiod_set_value(ep->perst, 0);
	usleep_range(18000, 20000);

	v = ex_elbi_rd(ep, EX_ELBI_APP_REQ_EXIT_L1_MODE);
	v |= EX_APP_REQ_EXIT_L1_MODE | EX_L1_REQ_NAK_CONTROL_MASTER;
	ex_elbi_wr(ep, EX_ELBI_APP_REQ_EXIT_L1_MODE, v);

	ex_elbi_wr(ep, EX_ELBI_LINKDOWN_RST_CTRL_SEL, EX_LINKDOWN_RST_MANUAL);

	v = ex_elbi_rd(ep, EX_ELBI_QCH_SEL);
	v &= ~0xfff;
	ex_elbi_wr(ep, EX_ELBI_QCH_SEL, v);

	ex_assert_phy_reset(ep);

	ex_elbi_wr(ep, EX_ELBI_L1_BUG_FIX_ENABLE, 1);

	dw_pcie_setup_rc(&pci->pp);
	ex_setup_rc(ep);
	ex_prog_atu(ep);

	ex_pcs_rx_elecidle(ep, 0);

	dev_info(pci->dev, "D state: %x, %x\n",
		 ex_elbi_rd(ep, EX_ELBI_PM_DSTATE) & 0x7,
		 ex_elbi_rd(ep, EX_ELBI_RDLH_LINKUP));

	ex_elbi_wr(ep, EX_ELBI_APP_LTSSM_ENABLE, 1);

	for (n = 0; n < EX_LINK_POLL_MAX; n++) {
		v = ex_elbi_rd(ep, EX_ELBI_RDLH_LINKUP) & 0x1f;
		if (v >= EX_LTSSM_MIN && v <= EX_LTSSM_MAX)
			break;
		udelay(10);
	}

	if (n >= EX_LINK_POLL_MAX) {
		tries++;
		dev_err(pci->dev, "%s: Link is not up, try count: %d, %x\n",
			__func__, tries, ex_elbi_rd(ep, EX_ELBI_RDLH_LINKUP));
		if (tries < EX_LINK_TRIES) {
			gpiod_set_value(ep->perst, 1);
			ex_elbi_wr(ep, EX_ELBI_APP_LTSSM_ENABLE, 0);
			goto retry;
		}
		ex_print_link_history(ep);
		return -EPIPE;
	}

	v = ex_elbi_rd(ep, EX_ELBI_RDLH_LINKUP) & 0x1f;
	if (v < EX_LTSSM_MIN || v > EX_LTSSM_MAX) {
		dev_info(pci->dev, "%s: Link state:%x\n", __func__, v);
		gpiod_set_value(ep->perst, 1);
		ex_elbi_wr(ep, EX_ELBI_APP_LTSSM_ENABLE, 0);
		goto retry;
	}

	dev_info(pci->dev, "%s: Link up:%x\n", __func__, v);
	dev_info(pci->dev, "link up, LTSSM 0x%02x after %d tries\n", v, tries + 1);

	ex_elbi_wr(ep, EX_ELBI_IRQ_PULSE, ex_elbi_rd(ep, EX_ELBI_IRQ_PULSE));
	ex_elbi_wr(ep, EX_ELBI_IRQ_EN_PULSE, ex_elbi_rd(ep, EX_ELBI_IRQ_EN_PULSE) | EX_IRQ_LINKDOWN_ENABLE);

	return 0;
}

static void exynos_pcie_notify_callback(struct exynos8895_pcie *ep, int event)
{
	if (ep->event_reg && ep->event_reg->callback &&
	    (ep->event_reg->events & event)) {
		struct exynos_pcie_notify *notify = &ep->event_reg->notify;

		notify->event = event;
		notify->user = ep->event_reg->user;
		dev_info(ep->pci.dev, "Callback for the event : %d\n", event);
		ep->event_reg->callback(notify);
	} else {
		dev_info(ep->pci.dev,
			 "Client driver does not have registration of the event : %d\n", event);
	}
}

static void ex_linkdown_work(struct work_struct *work)
{
	struct exynos8895_pcie *ep = container_of(work, struct exynos8895_pcie, linkdown_work);

	if (ep->state == EX_LINK_DOWN)
		return;

	ep->linkdown_cnt++;
	dev_info(ep->pci.dev, "link down and recovery cnt: %d\n", ep->linkdown_cnt);
	exynos_pcie_notify_callback(ep, EXYNOS_PCIE_EVENT_LINKDOWN);
}

static int exynos_pcie_set_l1ss(struct exynos8895_pcie *ep, int enable, int id)
{
	struct dw_pcie *pci = &ep->pci;
	unsigned long flags;
	u32 val;

	dev_info(pci->dev, "%s: START (state = 0x%x, id = 0x%x, enable = %d)\n",
		 __func__, ep->l1ss_ctrl_id_state, id, enable);

	spin_lock_irqsave(&ep->conf_lock, flags);
	if (ep->state != EX_LINK_UP || !ep->atu_ok) {
		if (enable)
			ep->l1ss_ctrl_id_state &= ~id;
		else
			ep->l1ss_ctrl_id_state |= id;
		spin_unlock_irqrestore(&ep->conf_lock, flags);
		dev_info(pci->dev, "%s: Link is not up. This will be set later. (state = 0x%x, id = 0x%x)\n",
			 __func__, ep->l1ss_ctrl_id_state, id);
		return -1;
	}

	if (enable) {
		ep->l1ss_ctrl_id_state &= ~id;
		if (ep->l1ss_ctrl_id_state == 0) {
			if (ex_l1ss_allowed) {
				pci_read_config_dword(ep->ep, EX_EP_LNKCTL, &val);
				val &= ~0x3;
				val |= 0x142;
				pci_write_config_dword(ep->ep, EX_EP_LNKCTL, val);
				pci_read_config_dword(ep->ep, EX_EP_L1SS_CTL1, &val);
				pci_write_config_dword(ep->ep, EX_EP_L1SS_CTL1, val | 0xa0f);
				dev_info(pci->dev, "%s: L1ss enabled. (state = 0x%x, id = 0x%x)\n",
					 __func__, ep->l1ss_ctrl_id_state, id);
			} else {
				/* first-pass guard (L6D 5.1.4 item 17): mask updated, EP L1SS left off */
				dev_info(pci->dev, "%s: L1ss enable skipped, l1ss=0. (state = 0x%x, id = 0x%x)\n",
					 __func__, ep->l1ss_ctrl_id_state, id);
			}
		} else {
			dev_info(pci->dev, "%s: Can't enable L1ss. (state = 0x%x, id = 0x%x)\n",
				 __func__, ep->l1ss_ctrl_id_state, id);
		}
	} else if (enable == 0) {
		if (ep->l1ss_ctrl_id_state) {
			ep->l1ss_ctrl_id_state |= id;
			dev_info(pci->dev, "%s: L1ss is already disabled. (state = 0x%x, id = 0x%x)\n",
				 __func__, ep->l1ss_ctrl_id_state, id);
		} else {
			ep->l1ss_ctrl_id_state |= id;
			pci_read_config_dword(ep->ep, EX_EP_LNKCTL, &val);
			pci_write_config_dword(ep->ep, EX_EP_LNKCTL, val & ~0x3);
			pci_read_config_dword(ep->ep, EX_EP_L1SS_CTL1, &val);
			pci_write_config_dword(ep->ep, EX_EP_L1SS_CTL1, val & ~0xf);
			dev_info(pci->dev, "%s: L1ss disabled. (state = 0x%x, id = 0x%x)\n",
				 __func__, ep->l1ss_ctrl_id_state, id);
		}
	}
	spin_unlock_irqrestore(&ep->conf_lock, flags);
	dev_info(pci->dev, "%s: END (state = 0x%x, id = 0x%x, enable = %d)\n",
		 __func__, ep->l1ss_ctrl_id_state, id, enable);
	return 0;
}

static void ex_l1ss_boot_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct exynos8895_pcie *ep = container_of(dwork, struct exynos8895_pcie, l1ss_boot_work);

	if (ep->work_l1ss_cnt == 0) {
		dev_info(ep->pci.dev, "[%s]boot_cnt: %d, work_l1ss_cnt: %d\n",
			 __func__, ep->boot_cnt, ep->work_l1ss_cnt);
		exynos_pcie_set_l1ss(ep, 1, PCIE_L1SS_CTRL_BOOT);
		ep->work_l1ss_cnt++;
	} else {
		dev_info(ep->pci.dev, "[%s]boot_cnt: %d, work_l1ss_cnt: %d\n",
			 __func__, ep->boot_cnt, ep->work_l1ss_cnt);
	}
}

static void exynos_pcie_config_l1ss(struct exynos8895_pcie *ep)
{
	struct dw_pcie *pci = &ep->pci;
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&ep->conf_lock, flags);
	ep->atu_ok = (ep->ep != NULL);
	if (ep->atu_ok) {
		if (ep->l1ss_ctrl_id_state == 0) {
			if (ex_l1ss_allowed) {
				pci_read_config_dword(ep->ep, EX_EP_LNKCTL, &val);
				val &= ~0x3;
				val |= 0x142;
				pci_write_config_dword(ep->ep, EX_EP_LNKCTL, val);
				pci_read_config_dword(ep->ep, EX_EP_L1SS_CTL1, &val);
				pci_write_config_dword(ep->ep, EX_EP_L1SS_CTL1, val | 0xa0f);
				dev_err(pci->dev, "l1ss enabled(0x%x)\n", ep->l1ss_ctrl_id_state);
			} else {
				dev_err(pci->dev, "l1ss enable skipped, l1ss=0 (0x%x)\n", ep->l1ss_ctrl_id_state);
			}
		} else {
			pci_read_config_dword(ep->ep, EX_EP_LNKCTL, &val);
			pci_write_config_dword(ep->ep, EX_EP_LNKCTL, val & ~0x3);
			pci_read_config_dword(ep->ep, EX_EP_L1SS_CTL1, &val);
			pci_write_config_dword(ep->ep, EX_EP_L1SS_CTL1, val & ~0xf);
			dev_err(pci->dev, "l1ss disabled(0x%x)\n", ep->l1ss_ctrl_id_state);
		}
		spin_unlock_irqrestore(&ep->conf_lock, flags);

		pci_write_config_dword(ep->ep, EX_EP_L1SS_CTL2, EX_PORT_LINK_TPOWERON_130US);
		pci_write_config_dword(ep->ep, EX_EP_LTR_1B4, 0x10031003);
		pci_read_config_dword(ep->ep, EX_EP_DEVCTL2, &val);
		pci_write_config_dword(ep->ep, EX_EP_DEVCTL2, val | BIT(10));

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
		spin_unlock_irqrestore(&ep->conf_lock, flags);
	}
}

static void exynos_pcie_send_pme_turn_off(struct exynos8895_pcie *ep)
{
	struct dw_pcie *pci = &ep->pci;
	int count = 0;
	u32 val;

	val = ex_elbi_rd(ep, EX_ELBI_RDLH_LINKUP) & 0x1f;
	dev_info(pci->dev, "%s: link state:%x\n", __func__, val);
	if (!(val >= EX_LTSSM_MIN && val <= EX_LTSSM_MAX)) {
		dev_info(pci->dev, "%s, pcie link is not up\n", __func__);
		return;
	}

	ex_elbi_wr(ep, EX_ELBI_APP_REQ_EXIT_L1, 0x1);
	val = ex_elbi_rd(ep, EX_ELBI_APP_REQ_EXIT_L1_MODE);
	val &= ~EX_APP_REQ_EXIT_L1_MODE;
	val |= EX_L1_REQ_NAK_CONTROL_MASTER;
	ex_elbi_wr(ep, EX_ELBI_APP_REQ_EXIT_L1_MODE, val);

	ex_elbi_wr(ep, EX_ELBI_PME_A8, 0x0);
	ex_elbi_wr(ep, EX_ELBI_PME_AC, 0x1);
	ex_elbi_wr(ep, EX_ELBI_PME_B0, 0x13);
	ex_elbi_wr(ep, EX_ELBI_PME_D0, 0x19);
	ex_elbi_wr(ep, EX_ELBI_PME_A8, 0x1);

	while (count < EX_LINK_POLL_MAX) {
		if (ex_elbi_rd(ep, EX_ELBI_IRQ_PULSE) & EX_IRQ_RADM_PM_TO_ACK) {
			dev_err(pci->dev, "ack message is ok\n");
			break;
		}
		udelay(10);
		count++;
	}

	if (count >= EX_LINK_POLL_MAX)
		dev_err(pci->dev, "cannot receive ack message from wifi\n");

	ex_elbi_wr(ep, EX_ELBI_APP_REQ_EXIT_L1, 0x0);

	do {
		val = ex_elbi_rd(ep, EX_ELBI_RDLH_LINKUP) & 0x1f;
		if (val == EX_LTSSM_L23_READY) {
			dev_err(pci->dev, "received Enter_L23_READY DLLP packet\n");
			break;
		}
		udelay(10);
		count++;
	} while (count < EX_LINK_POLL_MAX);

	if (count >= EX_LINK_POLL_MAX)
		dev_err(pci->dev, "cannot receive L23_READY DLLP packet\n");

	ex_elbi_wr(ep, EX_ELBI_APP_REQ_EXIT_L1, 0x1);
	val = ex_elbi_rd(ep, EX_ELBI_APP_REQ_EXIT_L1_MODE);
	val &= ~EX_APP_REQ_EXIT_L1_MODE;
	val |= EX_L1_REQ_NAK_CONTROL_MASTER;
	ex_elbi_wr(ep, EX_ELBI_APP_REQ_EXIT_L1_MODE, val);
	ex_elbi_wr(ep, EX_ELBI_PME_A8, 0x0);
	ex_elbi_wr(ep, EX_ELBI_APP_REQ_EXIT_L1, 0x0);
}

static irqreturn_t ex_irq(int irq, void *arg)
{
	struct exynos8895_pcie *ep = arg;
	struct dw_pcie *pci = &ep->pci;
	u32 v, l, s;

	v = ex_elbi_rd(ep, EX_ELBI_IRQ_PULSE);
	ex_elbi_wr(ep, EX_ELBI_IRQ_PULSE, v);

	if (v & EX_IRQ_LINK_DOWN) {
		dev_info(pci->dev, "!!!PCIE LINK DOWN!!!\n");
		ex_print_link_history(ep);
		exynos_pcie_dump_link_down_status(0);
		exynos_pcie_register_dump(0);
		queue_work(ep->wq, &ep->linkdown_work);
	}

	l = ex_elbi_rd(ep, EX_ELBI_IRQ_LEVEL);
	ex_elbi_wr(ep, EX_ELBI_IRQ_LEVEL, l);

	s = ex_elbi_rd(ep, EX_ELBI_IRQ_SPECIAL);
	ex_elbi_wr(ep, EX_ELBI_IRQ_SPECIAL, s);

	return (v | l | s) ? IRQ_HANDLED : IRQ_NONE;
}

static void __ex_poweroff(struct exynos8895_pcie *ep)
{
	struct dw_pcie *pci = &ep->pci;
	unsigned long flags;

	if (ep->state == EX_LINK_UP || ep->state == EX_LINK_DOWN_TRY) {
		ep->state = EX_LINK_DOWN_TRY;

		if (ep->irq_on) {
			disable_irq(ep->irq);
			ep->irq_on = false;
		}

		ex_elbi_wr(ep, EX_ELBI_IRQ_EN_PULSE,
			   ex_elbi_rd(ep, EX_ELBI_IRQ_EN_PULSE) & ~EX_IRQ_LINKDOWN_ENABLE);

		spin_lock_irqsave(&ep->conf_lock, flags);
		exynos_pcie_send_pme_turn_off(ep);
		ep->state = EX_LINK_DOWN;

		ex_elbi_wr(ep, EX_ELBI_STATE_HISTORY_CHECK,
			   ex_elbi_rd(ep, EX_ELBI_STATE_HISTORY_CHECK) & ~EX_HISTORY_BUFFER_ENABLE);
		gpiod_set_value(ep->perst, 1);
		ex_elbi_wr(ep, EX_ELBI_APP_LTSSM_ENABLE, 0);
		ex_phy_all_pwrdn(ep);
		spin_unlock_irqrestore(&ep->conf_lock, flags);

		ep->atu_ok = false;
		pinctrl_select_state(ep->pinctrl, ep->pins_clkreq);
		pm_runtime_put_sync(pci->dev);
	}

	regmap_update_bits(ep->pmu, EX_PMU_WAKEUP_MASK,
			   BIT(EX_WAKEUP_MASK_PCIE_WIFI),
			   BIT(EX_WAKEUP_MASK_PCIE_WIFI));
}

static int __ex_poweron(struct exynos8895_pcie *ep)
{
	struct dw_pcie *pci = &ep->pci;
	struct dw_pcie_rp *pp = &pci->pp;
	struct pci_bus *child_bus;
	struct pci_dev *ep_dev;
	int ret;

	if (ep->no_atu)
		return -ENODEV;	/* post_init found < 3 outbound iATU windows (L6D 5.1.4 item 6) */

	if (ep->state == EX_LINK_DOWN) {
		regmap_update_bits(ep->pmu, EX_PMU_WAKEUP_MASK, BIT(EX_WAKEUP_MASK_PCIE_WIFI), 0);
		pm_runtime_get_sync(pci->dev);
		pinctrl_select_state(ep->pinctrl, ep->pins_default);
		regmap_update_bits(ep->pmu, EX_PMU_PCIE_PHY_CONTROL, EX_PCIE_PHY_CONTROL_MASK, 1);
		ex_phy_all_pwrdn_clear(ep);
		ep->state = EX_LINK_UP_TRY;
		ex_elbi_wr(ep, EX_ELBI_STATE_HISTORY_CHECK,
			   ex_elbi_rd(ep, EX_ELBI_STATE_HISTORY_CHECK) | EX_HISTORY_BUFFER_ENABLE);

		ret = exynos_pcie_establish_link(ep);
		if (ret)
			goto fail;

		ep->state = EX_LINK_UP;

		if (!ep->probe_ok) {
			pci_lock_rescan_remove();
			pci_rescan_bus(pp->bridge->bus);
			pci_unlock_rescan_remove();

			ep->rp = pci_get_slot(pp->bridge->bus, PCI_DEVFN(0, 0));
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
			ep->ep = ep_dev;

			if (pci_resource_start(ep_dev, 0) != EX_BAR0_CPU ||
			    pci_resource_start(ep_dev, 2) != EX_MEM_BASE ||
			    pci_resource_len(ep_dev, 2) != EX_BRCM_BAR2_SIZE) {
				dev_err(pci->dev, "unexpected BAR layout: BAR0 %pR BAR2 %pR\n",
					&ep_dev->resource[0], &ep_dev->resource[2]);
				goto fail_first;
			}

			dev_info(pci->dev, "ep %s BAR0 cpu 0x%llx -> pci 0x11c00000 (ATU 2)\n",
				 pci_name(ep_dev), (unsigned long long)pci_resource_start(ep_dev, 0));

			exynos_pcie_config_l1ss(ep);

			if (ep->rp) {
				pci_save_state(ep->rp);
				ep->rp_saved = pci_store_saved_state(ep->rp);
			}
			ep->probe_ok = true;
		} else {
			if (ep->boot_cnt == 0) {
				schedule_delayed_work(&ep->l1ss_boot_work, msecs_to_jiffies(40000));
				ep->boot_cnt++;
				ep->l1ss_ctrl_id_state |= PCIE_L1SS_CTRL_BOOT;
			}
			exynos_pcie_config_l1ss(ep);
			if (ep->rp && ep->rp_saved) {
				pci_load_saved_state(ep->rp, ep->rp_saved);
				pci_restore_state(ep->rp);
			}
		}

		enable_irq(ep->irq);
		ep->irq_on = true;
	}

	return 0;

fail_first:
	/* the next power-on rescans and looks both devices up again */
	pci_dev_put(ep->ep);
	ep->ep = NULL;
	pci_dev_put(ep->rp);
	ep->rp = NULL;
fail:
	ep->state = EX_LINK_UP;
	__ex_poweroff(ep);
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
		dev_info(ex_g->pci.dev, "RC%d already off\n", ch);
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
	struct exynos8895_pcie *ep;

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
	ep = to_ex(to_dw_pcie_from_pp(pp));

	ep->event_reg = reg;
	dev_info(ep->pci.dev, "Event 0x%x is registered for RC 0\n", reg->events);

	return 0;
}
EXPORT_SYMBOL_GPL(exynos_pcie_register_event);

int exynos_pcie_deregister_event(struct exynos_pcie_register_event *reg)
{
	struct pci_dev *pdev;
	struct dw_pcie_rp *pp;
	struct exynos8895_pcie *ep;

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
	ep = to_ex(to_dw_pcie_from_pp(pp));

	ep->event_reg = NULL;
	dev_info(ep->pci.dev, "Event is deregistered for RC 0\n");

	return 0;
}
EXPORT_SYMBOL_GPL(exynos_pcie_deregister_event);

void exynos_pcie_register_dump(int ch)
{
	u32 i, j, val;

	if (ch != 0 || !ex_g)
		return;

	/* Link Reg : 0x0 ~ 0x2C4 */
	for (i = 0; i < 45; i++) {
		printk("RC Driver Reg 0x%04x: ", i * 0x10);
		for (j = 0; j < 4; j++) {
			if (((i * 0x10) + (j * 4)) < 0x2C4)
				printk("0x%08x ", ex_elbi_rd(ex_g, (i * 0x10) + (j * 4)));
		}
		printk("\n");
	}

	/* PHY Reg : 0x0 ~ 0x180 */
	for (i = 0; i < 24; i++) {
		printk("PHY reg 0x%04x(%02x): ", i * 0x10, i * 4);
		for (j = 0; j < 4; j++) {
			if (((i * 0x10) + (j * 4)) < 0x180)
				printk("0x%08x ", readl(ex_g->phy + (i * 0x10) + (j * 4)));
		}
		printk("\n");
	}

	/* RC Conf : 0x0 ~ 0x40 */
	for (i = 0; i < 4; i++) {
		printk("RC Conf 0x%04x: ", i * 0x10);
		for (j = 0; j < 4; j++) {
			if (((i * 0x10) + (j * 4)) < 0x40) {
				val = dw_pcie_readl_dbi(&ex_g->pci, (i * 0x10) + (j * 4));
				printk("0x%08x ", val);
			}
		}
		printk("\n");
	}

	val = dw_pcie_readl_dbi(&ex_g->pci, 0x78);
	printk("RC Conf 0x0078(Device Status Register): 0x%08x\n", val);
	val = dw_pcie_readl_dbi(&ex_g->pci, 0x80);
	printk("RC Conf 0x0080(Link Status Register): 0x%08x\n", val);
	val = dw_pcie_readl_dbi(&ex_g->pci, 0x104);
	printk("RC Conf 0x0104(AER Registers): 0x%08x\n", val);
	val = dw_pcie_readl_dbi(&ex_g->pci, 0x110);
	printk("RC Conf 0x0110(AER Registers): 0x%08x\n", val);
	val = dw_pcie_readl_dbi(&ex_g->pci, 0x130);
	printk("RC Conf 0x0130(AER Registers): 0x%08x\n", val);
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
	struct exynos8895_pcie *ep = dev_get_drvdata(dev);

	if (ep->state == EX_LINK_DOWN) {
		dev_info(dev, "RC0 already off\n");
		return 0;
	}

	exynos_pcie_send_pme_turn_off(ep);
	gpiod_set_value(ep->perst, 1);

	return 0;
}

static int exynos_pcie_resume_noirq(struct device *dev)
{
	struct exynos8895_pcie *ep = dev_get_drvdata(dev);

	if (ep->state == EX_LINK_DOWN) {
		ex_resumed_phydown(ep);
		return 0;
	}

	ex_enable_interrupts(ep);
	regmap_update_bits(ep->pmu, EX_PMU_PCIE_PHY_CONTROL, EX_PCIE_PHY_CONTROL_MASK, 1);
	exynos_pcie_establish_link(ep);
	exynos_pcie_config_l1ss(ep);

	return 0;
}
#endif

static const struct dev_pm_ops exynos8895_pcie_pm_ops = {
	NOIRQ_SYSTEM_SLEEP_PM_OPS(exynos_pcie_suspend_noirq, exynos_pcie_resume_noirq)
};

/* S:drivers/pci/host/pci-exynos.c:350-360, registered at :1374-1378 */
static int exynos_pcie_dma_mon_event(struct notifier_block *nb, unsigned long event, void *data)
{
	struct exynos8895_pcie *ep = container_of(nb, struct exynos8895_pcie, panic_nb);
	int ret;

	ret = exynos_pcie_dump_link_down_status(0);
	if (ep->state == EX_LINK_UP)
		exynos_pcie_register_dump(0);

	return notifier_from_errno(ret);
}

static void ex_destroy_wq(void *wq)
{
	destroy_workqueue(wq);
}

static int ex_host_init(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct exynos8895_pcie *ep = to_ex(pci);
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

	/* (a) F-01: drop only the iomem claim of the config page 0x11bff000 */
	devm_release_mem_region(pci->dev, cfg->start, resource_size(cfg));

	/* (b) F-02: RC ids 0x144d/0xecec, LNKCAP, aux clk, L1SS timing, sysreg bifurcation bits */
	ex_setup_rc(ep);

	/* (c) own-config accessors with the rx-elecidle toggle */
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
	struct exynos8895_pcie *ep = to_ex(pci);
	struct pci_dev *rp;

	if (pci->num_ob_windows < 3) {
		dev_err(pci->dev, "need 3 outbound iATU windows, have %u\n", pci->num_ob_windows);
		ep->no_atu = true;
		return;
	}

	rp = pci_get_slot(pp->bridge->bus, PCI_DEVFN(0, 0));
	if (rp) {
		dev_info(pci->dev, "RC %04x:%04x BAR0 %pR BAR1 %pR\n",
			 rp->vendor, rp->device, &rp->resource[0], &rp->resource[1]);
		pci_dev_put(rp);
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
	struct exynos8895_pcie *ep;
	struct dw_pcie *pci;
	struct resource *res;
	int ret, i;

	ep = devm_kzalloc(dev, sizeof(*ep), GFP_KERNEL);
	if (!ep)
		return -ENOMEM;

	pci = &ep->pci;
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

	ep->phy = devm_platform_ioremap_resource_byname(pdev, "phy");
	if (IS_ERR(ep->phy))
		return PTR_ERR(ep->phy);

	ep->pcs = devm_platform_ioremap_resource_byname(pdev, "pcs");
	if (IS_ERR(ep->pcs))
		return PTR_ERR(ep->pcs);

	ep->pmu = syscon_regmap_lookup_by_phandle(np, "samsung,pmu-syscon");
	if (IS_ERR(ep->pmu))
		return PTR_ERR(ep->pmu);

	ep->sysreg = syscon_regmap_lookup_by_phandle_args(np, "samsung,fsys-sysreg",
							  1, &ep->sysreg_off);
	if (IS_ERR(ep->sysreg))
		return PTR_ERR(ep->sysreg);

	ep->perst = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ep->perst))
		return PTR_ERR(ep->perst);

	ep->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(ep->pinctrl))
		return PTR_ERR(ep->pinctrl);

	ep->pins_default = pinctrl_lookup_state(ep->pinctrl, "default");
	if (IS_ERR(ep->pins_default))
		return PTR_ERR(ep->pins_default);

	ep->pins_clkreq = pinctrl_lookup_state(ep->pinctrl, "clkreq_output");
	if (IS_ERR(ep->pins_clkreq))
		return PTR_ERR(ep->pins_clkreq);

	pinctrl_select_state(ep->pinctrl, ep->pins_clkreq);

	ep->nclks = devm_clk_bulk_get_all_enabled(dev, &ep->clks);
	if (ep->nclks < 0)
		return ep->nclks;

	for (i = 0; i < ep->nclks; i++) {
		if (ep->clks[i].id && !strcmp(ep->clks[i].id, "ref")) {
			dev_info(dev, "PHY_REF %lu Hz\n", clk_get_rate(ep->clks[i].clk));
			break;
		}
	}

	/*
	 * Everything ex_irq touches exists before the shared SPI 345 is requested (the line is live from
	 * devm_request_irq until the disable_irq below). The workqueue is released by a devm action added
	 * before the IRQ, so devm frees the IRQ first on every failure path.
	 */
	spin_lock_init(&ep->conf_lock);
	mutex_init(&ep->pwr_lock);

	ep->wq = alloc_ordered_workqueue("pcie_wq", WQ_FREEZABLE);	/* stock create_freezable_workqueue :1380 */
	if (!ep->wq)
		return -ENOMEM;
	ret = devm_add_action_or_reset(dev, ex_destroy_wq, ep->wq);
	if (ret)
		return ret;

	INIT_WORK(&ep->linkdown_work, ex_linkdown_work);
	INIT_DELAYED_WORK(&ep->l1ss_boot_work, ex_l1ss_boot_work);

	ep->irq = platform_get_irq(pdev, 0);
	if (ep->irq < 0)
		return ep->irq;

	ret = devm_request_irq(dev, ep->irq, ex_irq, IRQF_SHARED, "exynos-pcie", ep);
	if (ret)
		return ret;

	ex_resumed_phydown(ep);
	regmap_update_bits(ep->pmu, EX_PMU_PCIE_PHY_CONTROL, EX_PCIE_PHY_CONTROL_MASK, 1);

	pm_runtime_no_callbacks(dev);
	pm_suspend_ignore_children(dev, true);
	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	pci->pp.use_linkup_irq = true;

	ret = dw_pcie_host_init(&pci->pp);
	if (ret)
		return ret;

	disable_irq(ep->irq);
	ep->irq_on = false;

	platform_set_drvdata(pdev, ep);
	ex_g = ep;

	ep->panic_nb.notifier_call = exynos_pcie_dma_mon_event;
	atomic_notifier_chain_register(&panic_notifier_list, &ep->panic_nb);

	dev_info(dev, "probe ok: ip-ver 0x889500, ch 0, phy powered down\n");

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

/* PCI Quirks (spec L6D 5.1.4 item 23) */
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

	/* Change BAR2 Size from 4MB to 3MB (cal:246-247) */
	pci_resource_end(dev, 2) = pci_resource_start(dev, 2) + 0x300000 - 1;
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_BROADCOM, PCI_ANY_ID, ex_quirk_brcm_header);

static void ex_quirk_brcm_enable(struct pci_dev *dev)
{
	int count = 0;
	u32 reg;

	if (!ex_is_our_device(dev))
		return;

	pci_info(dev, "Before changing BAR0\n");
	pci_read_config_dword(dev, PCI_COMMAND, &reg);
	pci_info(dev, "CMD Register = 0x%x, ", reg);
	pci_read_config_dword(dev, PCI_BASE_ADDRESS_0, &reg);
	pci_info(dev, "BAR0 = 0x%x\n", reg);

	pci_info(dev, "Change BAR0 to fit 0x11C00000\n");
	pci_write_config_dword(dev, PCI_BASE_ADDRESS_0, 0x11C00000);

	pci_info(dev, "BAR0 retry code added.\n");
	while (1) {
		pci_read_config_dword(dev, PCI_BASE_ADDRESS_0, &reg);
		reg &= 0xfffffff0;
		if (reg == 0x11C00000)
			break;
		pci_info(dev, "BAR0 resizing retry - %d %x\n", ++count, reg);
		pci_write_config_dword(dev, PCI_BASE_ADDRESS_0, 0x11C00000);
		if (count > 100)
			break;
	}

	pci_info(dev, "After changing BAR0\n");
	pci_read_config_dword(dev, PCI_COMMAND, &reg);
	pci_info(dev, "CMD Register = 0x%x, ", reg);
	pci_read_config_dword(dev, PCI_BASE_ADDRESS_0, &reg);
	pci_info(dev, "BAR0 = 0x%x\n", reg);
}
DECLARE_PCI_FIXUP_ENABLE(PCI_VENDOR_ID_BROADCOM, PCI_ANY_ID, ex_quirk_brcm_enable);

static void ex_quirk_async(struct pci_dev *dev)
{
	if (!ex_is_our_device(dev))
		return;

	device_disable_async_suspend(&dev->dev);
	pci_info(dev, "[%s] async suspend disabled\n", __func__);

	/* (final, R3-13) L1SS owned by exynos8895-pcie */
	if (dev->l1ss) {
		pci_info(dev, "L1SS owned by exynos8895-pcie, core save/restore off\n");
		dev->l1ss = 0;
	}
}
DECLARE_PCI_FIXUP_FINAL(PCI_ANY_ID, PCI_ANY_ID, ex_quirk_async);

static void ex_quirk_rc_header(struct pci_dev *dev)
{
	int i;

	if (!ex_is_our_device(dev) || !pci_is_root_bus(dev->bus))
		return;

	for (i = 0; i < 2; i++) {
		if (dev->resource[i].flags) {
			pci_info(dev, "RC BAR%d %pR unused, dropped\n", i, &dev->resource[i]);
			dev->resource[i].flags = 0;
		}
	}
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_SAMSUNG, EX_RC_DEVICE_ID, ex_quirk_rc_header);
