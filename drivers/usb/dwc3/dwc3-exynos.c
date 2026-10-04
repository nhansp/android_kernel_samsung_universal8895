// SPDX-License-Identifier: GPL-2.0
/*
 * dwc3-exynos.c - Samsung Exynos DWC3 Specific Glue layer
 *
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Author: Anton Tikhomirov <av.tikhomirov@samsung.com>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/notifier.h>
#include <linux/pm_wakeup.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/regulator/consumer.h>
#include <linux/usb/gadget.h>
#include <linux/usb/role.h>

#include "core.h"
#include "glue.h"

#define DWC3_EXYNOS_MAX_CLOCKS	4

struct dwc3_exynos_driverdata {
	const char		*clk_names[DWC3_EXYNOS_MAX_CLOCKS];
	int			num_clks;
	int			suspend_clk_idx;
	bool			vbus_session;	/* greatlte: Type-C owns the role */
};

struct dwc3_exynos {
	struct device		*dev;

	const char		**clk_names;
	struct clk		*clks[DWC3_EXYNOS_MAX_CLOCKS];
	int			num_clks;
	int			suspend_clk_idx;

	struct regulator	*vdd33;
	struct regulator	*vdd10;

	/* greatlte: the bind notifier of the dwc3 core of_platform_populate() adds */
	struct notifier_block	core_nb;
	bool			watch_core;

	/*
	 * greatlte: wakeup source held for the whole VBUS session. Stock kept
	 * the system awake from gadget/host start to stop with the "dwc3-otg"
	 * wake_lock (S:drivers/usb/dwc3/otg.c:714 init, :226 host on, :270
	 * host off, :308 gadget on, :344 gadget off); without it every
	 * suspend attempt suspends dwc3, which soft-disconnects the pull-up
	 * and the host sees a disconnect (the 3-7 s cable-in flap). Only used
	 * when vbus_session is set; vbus_lock guards vbus_held because the
	 * role switch may run where sleeping is not allowed.
	 */
	struct wakeup_source	*vbus_ws;
	spinlock_t		vbus_lock;
	bool			vbus_held;
};

/*
 * greatlte (spec L6G 5.4, G-6): the S2MM005 port controller owns the USB role
 * through the dwc3 role switch, and dwc3 maps USB_ROLE_NONE back to the default
 * mode (A:drivers/usb/dwc3/drd.c:445-470), which in device mode leaves the gadget
 * untouched: __dwc3_set_mode() returns as soon as the mode does not change
 * (A:drivers/usb/dwc3/core.c:187-189). So a detach has to reach the gadget
 * itself, exactly where stock did it: the CCIC/MUIC notifier called
 * dwc3_exynos_vbus_event() (S:drivers/usb/notify/usb_notifier.c:326 ->
 * S:drivers/usb/dwc3/dwc3-exynos.c:378-403), which ran the OTG FSM and
 * dwc3_otg_start/stop_gadget() (S:drivers/usb/dwc3/otg.c:282-353). 6.18 hands a
 * glue every role switch through dwc3_glue_ops.pre_set_role
 * (A:drivers/usb/dwc3/core.h:1006-1014, :1651-1655), which
 * dwc3_usb_role_switch_set() calls before dwc3_set_mode()
 * (A:drivers/usb/dwc3/drd.c:467-468), so stock's sequence is reproduced with the
 * 6.18 VBUS session API:
 * - no cable (USB_ROLE_NONE): the VBUS session ends, i.e. the pull-up is dropped,
 *   the UDC state goes not-attached and the uevent the USB HAL waits for is sent
 *   (A:drivers/usb/gadget/udc/core.c:758-790 -> gadget->ops->pullup,
 *   A:drivers/usb/dwc3/gadget.c:2791). That pull-up path is also where stock
 *   re-tuned the PHY after its core reset (S:drivers/usb/dwc3/gadget.c:1962-1978),
 *   which the L3 vendor hook already covers (spec/L3_usb.md 3.D step 5b);
 * - a cable (USB_ROLE_DEVICE): the VBUS session starts, the pull-up again;
 * - USB_ROLE_HOST: nothing, dwc3_set_mode() tears the gadget down and adds xHCI
 *   itself.
 *
 * The session goes to usb_udc_vbus_handler(), not to the soft-connect API:
 * usb_gadget_connect()/usb_gadget_disconnect() (A:drivers/usb/gadget/udc/core.c:
 * 705-826) are what configfs' soft_connect and a UDC bind use, and they only move
 * gadget->connected - every later UDC bind runs usb_udc_connect_control()
 * (A:drivers/usb/gadget/udc/core.c:1608-1641), which connects again from
 * udc->vbus (A:drivers/usb/gadget/udc/core.c:1143-1149) and undoes a detach as
 * soon as Android rebinds the UDC. usb_udc_vbus_handler() records the state in
 * the UDC and offloads the pull-up to the UDC's own work
 * (A:drivers/usb/gadget/udc/core.c:1151-1186), so every later connect decision
 * honours it; it is idempotent in both directions (the disconnect path returns
 * early when nothing is connected, A:drivers/usb/gadget/udc/core.c:768-769, and
 * the pull-up itself compares the state first, A:drivers/usb/dwc3/gadget.c:
 * 2824-2827), and it may be called from atomic context.
 *
 * The session also holds exynos->vbus_ws from USB_ROLE_DEVICE/USB_ROLE_HOST
 * until USB_ROLE_NONE, the 6.18 form of stock's "dwc3-otg" wake_lock
 * (S:drivers/usb/dwc3/otg.c:714, :226/:270 host, :308/:344 gadget): stock never
 * let the system suspend while a gadget session was up, and neither do we.
 * With no session (NONE) nothing is held, so the cable-out suspend path stays
 * free to sleep.
 */
static void dwc3_exynos_vbus_hold(struct dwc3_exynos *exynos, bool hold)
{
	unsigned long flags;

	if (!exynos || !exynos->vbus_ws)
		return;

	spin_lock_irqsave(&exynos->vbus_lock, flags);
	if (hold && !exynos->vbus_held) {
		__pm_stay_awake(exynos->vbus_ws);
		exynos->vbus_held = true;
	} else if (!hold && exynos->vbus_held) {
		__pm_relax(exynos->vbus_ws);
		exynos->vbus_held = false;
	}
	spin_unlock_irqrestore(&exynos->vbus_lock, flags);
}

static void dwc3_exynos_set_role(struct dwc3 *dwc, enum usb_role role)
{
	struct usb_gadget *gadget = dwc->gadget;
	struct dwc3_exynos *exynos = NULL;

	/*
	 * Only this glue installs these ops, and it installs them only on the
	 * core of_platform_populate() added under its own device
	 * (dwc3_exynos_glue_core()/dwc3_exynos_core_bound() check
	 * dev->parent), so the parent's driver data is this glue. The wakeup
	 * state is tracked here rather than on the dwc3 core because the core
	 * is upstream-owned; a missing glue (never happens) only loses the
	 * suspend block, never the VBUS session.
	 */
	if (dwc->dev && dwc->dev->parent)
		exynos = dev_get_drvdata(dwc->dev->parent);

	/* The wakeup source is role state, not gadget state: host mode has no
	 * gadget (dwc3_gadget_exit() freed it) but must still block suspend.
	 */
	if (role == USB_ROLE_NONE)
		dwc3_exynos_vbus_hold(exynos, false);
	else
		dwc3_exynos_vbus_hold(exynos, true);

	if (!gadget)
		return;			/* host mode: dwc3_gadget_exit() freed it */

	switch (role) {
	case USB_ROLE_NONE:
		usb_udc_vbus_handler(gadget, false);
		dev_info(dwc->dev, "no cable: VBUS session ended\n");
		break;
	case USB_ROLE_DEVICE:
		usb_udc_vbus_handler(gadget, true);
		dev_info(dwc->dev, "cable: VBUS session started\n");
		break;
	case USB_ROLE_HOST:
		break;
	}
}

static const struct dwc3_glue_ops dwc3_exynos_8895_glue_ops = {
	.pre_set_role	= dwc3_exynos_set_role,
};

/*
 * Attach the VBUS session to a dwc3 core. dwc3_probe() clears dwc->glue_ops
 * (A:drivers/usb/dwc3/core.c:2389) and only a glue that probes the core itself
 * can set it before that probe (the in-tree user is dwc3-qcom.c,
 * A:drivers/usb/dwc3/dwc3-qcom.c:605-608,701), but the field is read on every
 * role switch, so assigning it once the core has probed is enough. This is L3's
 * O19 (spec/L3_usb.md:750) done without converting the glue: the child device,
 * its name (10c00000.usb, which the UDC takes,
 * A:drivers/usb/gadget/udc/core.c:1404-1406) and its whole property set stay
 * what of_platform_populate() made them.
 */
static void dwc3_exynos_glue_core(struct dwc3_exynos *exynos, struct device *dev)
{
	struct dwc3 *dwc;

	if (!of_device_is_compatible(dev->of_node, "snps,dwc3"))
		return;

	/*
	 * dwc3_core_probe() stores the core in the driver data of its own
	 * platform device (A:drivers/usb/dwc3/core.c:2275), and .probe has
	 * returned by the time either caller below runs, so the core is there.
	 */
	dwc = dev_get_drvdata(dev);
	if (!dwc) {
		dev_err(exynos->dev, "%s: no dwc3 core, no VBUS session\n",
			dev_name(dev));
		return;
	}

	if (dwc->glue_ops == &dwc3_exynos_8895_glue_ops)
		return;

	dwc->glue_ops = &dwc3_exynos_8895_glue_ops;
	dev_info(exynos->dev, "USB VBUS session on %s\n", dev_name(dev));
}

/*
 * of_platform_populate() only binds the child core synchronously when nothing
 * defers its probe - its PHY, the fw_devlink suppliers of the port graph to the
 * Type-C connector on i2c 30, its regulators or clocks - and a deferred probe
 * happens long after dwc3_exynos_probe() has returned. The platform bus reports
 * every successful bind, bus_notify(BUS_NOTIFY_BOUND_DRIVER) from driver_bound()
 * (A:drivers/base/dd.c:459-482), and really_probe() calls it only after .probe
 * returned without error (A:drivers/base/dd.c:769), i.e. after dwc3_probe() has
 * cleared dwc->glue_ops (A:drivers/usb/dwc3/core.c:2389). Watching the bus
 * therefore covers the deferred case as well; the explicit lookup after
 * of_platform_populate() covers a core that bound synchronously.
 */
static int dwc3_exynos_core_bound(struct notifier_block *nb, unsigned long action,
				  void *data)
{
	struct dwc3_exynos *exynos = container_of(nb, struct dwc3_exynos, core_nb);
	struct device *dev = data;

	if (action != BUS_NOTIFY_BOUND_DRIVER)
		return NOTIFY_DONE;

	if (dev->parent != exynos->dev)		/* only the core of this glue */
		return NOTIFY_DONE;

	dwc3_exynos_glue_core(exynos, dev);
	return NOTIFY_OK;
}

static void dwc3_exynos_set_vbus_session(struct dwc3_exynos *exynos,
					 struct device_node *node)
{
	struct device_node *child;
	struct platform_device *core;

	for (child = of_get_next_available_child(node, NULL);
	     child;
	     child = of_get_next_available_child(node, child)) {
		if (of_device_is_compatible(child, "snps,dwc3"))
			break;
	}
	if (!child)
		return;

	core = of_find_device_by_node(child);
	if (!core) {
		dev_err(exynos->dev, "no dwc3 core device, no VBUS session\n");
		return;
	}

	dwc3_exynos_glue_core(exynos, &core->dev);
	put_device(&core->dev);
}

static int dwc3_exynos_probe(struct platform_device *pdev)
{
	struct dwc3_exynos	*exynos;
	struct device		*dev = &pdev->dev;
	struct device_node	*node = dev->of_node;
	const struct dwc3_exynos_driverdata *driver_data;
	int			i, ret;

	exynos = devm_kzalloc(dev, sizeof(*exynos), GFP_KERNEL);
	if (!exynos)
		return -ENOMEM;

	driver_data = of_device_get_match_data(dev);
	exynos->dev = dev;
	exynos->num_clks = driver_data->num_clks;
	exynos->clk_names = (const char **)driver_data->clk_names;
	exynos->suspend_clk_idx = driver_data->suspend_clk_idx;

	platform_set_drvdata(pdev, exynos);

	for (i = 0; i < exynos->num_clks; i++) {
		exynos->clks[i] = devm_clk_get(dev, exynos->clk_names[i]);
		if (IS_ERR(exynos->clks[i])) {
			dev_err(dev, "failed to get clock: %s\n",
				exynos->clk_names[i]);
			return PTR_ERR(exynos->clks[i]);
		}
	}

	for (i = 0; i < exynos->num_clks; i++) {
		ret = clk_prepare_enable(exynos->clks[i]);
		if (ret) {
			while (i-- > 0)
				clk_disable_unprepare(exynos->clks[i]);
			return ret;
		}
	}

	if (exynos->suspend_clk_idx >= 0)
		clk_prepare_enable(exynos->clks[exynos->suspend_clk_idx]);

	exynos->vdd33 = devm_regulator_get(dev, "vdd33");
	if (IS_ERR(exynos->vdd33)) {
		ret = PTR_ERR(exynos->vdd33);
		goto vdd33_err;
	}
	ret = regulator_enable(exynos->vdd33);
	if (ret) {
		dev_err(dev, "Failed to enable VDD33 supply\n");
		goto vdd33_err;
	}

	exynos->vdd10 = devm_regulator_get(dev, "vdd10");
	if (IS_ERR(exynos->vdd10)) {
		ret = PTR_ERR(exynos->vdd10);
		goto vdd10_err;
	}
	ret = regulator_enable(exynos->vdd10);
	if (ret) {
		dev_err(dev, "Failed to enable VDD10 supply\n");
		goto vdd10_err;
	}

	if (!node) {
		dev_err(dev, "no device node, failed to add dwc3 core\n");
		ret = -ENODEV;
		goto populate_err;
	}

	if (driver_data->vbus_session) {
		spin_lock_init(&exynos->vbus_lock);
		exynos->vbus_ws = wakeup_source_register(dev,
							  "dwc3-exynos-vbus");
		exynos->core_nb.notifier_call = dwc3_exynos_core_bound;
		ret = bus_register_notifier(&platform_bus_type,
					 &exynos->core_nb);
		if (ret) {
			dev_err(dev, "failed to watch the dwc3 core\n");
			goto populate_err;
		}
		exynos->watch_core = true;
	}

	ret = of_platform_populate(node, NULL, NULL, dev);
	if (ret) {
		dev_err(dev, "failed to add dwc3 core\n");
		goto notifier_err;
	}

	if (driver_data->vbus_session)
		dwc3_exynos_set_vbus_session(exynos, node);

	return 0;

notifier_err:
	if (exynos->watch_core)
		bus_unregister_notifier(&platform_bus_type,
					 &exynos->core_nb);
populate_err:
	if (exynos->vbus_ws) {
		dwc3_exynos_vbus_hold(exynos, false);
		wakeup_source_unregister(exynos->vbus_ws);
		exynos->vbus_ws = NULL;
	}
	regulator_disable(exynos->vdd10);
vdd10_err:
	regulator_disable(exynos->vdd33);
vdd33_err:
	for (i = exynos->num_clks - 1; i >= 0; i--)
		clk_disable_unprepare(exynos->clks[i]);

	if (exynos->suspend_clk_idx >= 0)
		clk_disable_unprepare(exynos->clks[exynos->suspend_clk_idx]);

	return ret;
}

static void dwc3_exynos_remove(struct platform_device *pdev)
{
	struct dwc3_exynos	*exynos = platform_get_drvdata(pdev);
	int i;

	if (exynos->watch_core)
		bus_unregister_notifier(&platform_bus_type,
					 &exynos->core_nb);

	if (exynos->vbus_ws) {
		dwc3_exynos_vbus_hold(exynos, false);
		wakeup_source_unregister(exynos->vbus_ws);
		exynos->vbus_ws = NULL;
	}

	of_platform_depopulate(&pdev->dev);

	for (i = exynos->num_clks - 1; i >= 0; i--)
		clk_disable_unprepare(exynos->clks[i]);

	if (exynos->suspend_clk_idx >= 0)
		clk_disable_unprepare(exynos->clks[exynos->suspend_clk_idx]);

	regulator_disable(exynos->vdd33);
	regulator_disable(exynos->vdd10);
}

static const struct dwc3_exynos_driverdata exynos2200_drvdata = {
	.clk_names = { "link_aclk" },
	.num_clks = 1,
	.suspend_clk_idx = -1,
};

static const struct dwc3_exynos_driverdata exynos5250_drvdata = {
	.clk_names = { "usbdrd30" },
	.num_clks = 1,
	.suspend_clk_idx = -1,
};

static const struct dwc3_exynos_driverdata exynos5433_drvdata = {
	.clk_names = { "aclk", "susp_clk", "pipe_pclk", "phyclk" },
	.num_clks = 4,
	.suspend_clk_idx = 1,
};

static const struct dwc3_exynos_driverdata exynos7_drvdata = {
	.clk_names = { "usbdrd30", "usbdrd30_susp_clk", "usbdrd30_axius_clk" },
	.num_clks = 3,
	.suspend_clk_idx = 1,
};

static const struct dwc3_exynos_driverdata exynos7870_drvdata = {
	.clk_names = { "bus_early", "ref", "ctrl" },
	.num_clks = 3,
	.suspend_clk_idx = -1,
};

static const struct dwc3_exynos_driverdata exynos850_drvdata = {
	.clk_names = { "bus_early", "ref" },
	.num_clks = 2,
	.suspend_clk_idx = -1,
};

/*
 * Exynos8895: stock glue claims {"aclk", "sclk"} (S:drivers/usb/dwc3/dwc3-exynos.c:125); the
 * suspend clock sits on the core node (PWRDNSCALE), so it is not claimed here. The
 * USBTVH gates belong to the same stock USBTV vclk
 * (S:drivers/soc/samsung/cal-if/exynos8895/cmucal-vclk.c:1442-1449).
 */
static const struct dwc3_exynos_driverdata exynos8895_drvdata = {
	.clk_names = { "aclk", "ref", "usbtvh_ahb", "usbtvh_core" },
	.num_clks = 4,
	.suspend_clk_idx = -1,
	/* L6G G2: the S2MM005 port controller owns the role (spec L6G 5.4) */
	.vbus_session = true,
};

static const struct dwc3_exynos_driverdata gs101_drvdata = {
	.clk_names = { "bus_early", "susp_clk", "link_aclk", "link_pclk" },
	.num_clks = 4,
	.suspend_clk_idx = 1,
};

static const struct dwc3_exynos_driverdata exynosautov920_drvdata = {
	.clk_names = { "ref", "susp_clk"},
	.num_clks = 2,
	.suspend_clk_idx = 1,
};

static const struct of_device_id exynos_dwc3_match[] = {
	{
		.compatible = "samsung,exynos2200-dwusb3",
		.data = &exynos2200_drvdata,
	}, {
		.compatible = "samsung,exynos5250-dwusb3",
		.data = &exynos5250_drvdata,
	}, {
		.compatible = "samsung,exynos5433-dwusb3",
		.data = &exynos5433_drvdata,
	}, {
		.compatible = "samsung,exynos7-dwusb3",
		.data = &exynos7_drvdata,
	}, {
		.compatible = "samsung,exynos7870-dwusb3",
		.data = &exynos7870_drvdata,
	}, {
		.compatible = "samsung,exynos850-dwusb3",
		.data = &exynos850_drvdata,
	}, {
		.compatible = "samsung,exynos8895-dwusb3",
		.data = &exynos8895_drvdata,
	}, {
		.compatible = "samsung,exynosautov920-dwusb3",
		.data = &exynosautov920_drvdata,
	}, {
		.compatible = "google,gs101-dwusb3",
		.data = &gs101_drvdata,
	}, {
	}
};
MODULE_DEVICE_TABLE(of, exynos_dwc3_match);

static int dwc3_exynos_suspend(struct device *dev)
{
	struct dwc3_exynos *exynos = dev_get_drvdata(dev);
	int i;

	for (i = exynos->num_clks - 1; i >= 0; i--)
		clk_disable_unprepare(exynos->clks[i]);

	regulator_disable(exynos->vdd33);
	regulator_disable(exynos->vdd10);

	return 0;
}

static int dwc3_exynos_resume(struct device *dev)
{
	struct dwc3_exynos *exynos = dev_get_drvdata(dev);
	int i, ret;

	ret = regulator_enable(exynos->vdd33);
	if (ret) {
		dev_err(dev, "Failed to enable VDD33 supply\n");
		return ret;
	}
	ret = regulator_enable(exynos->vdd10);
	if (ret) {
		dev_err(dev, "Failed to enable VDD10 supply\n");
		return ret;
	}

	for (i = 0; i < exynos->num_clks; i++) {
		ret = clk_prepare_enable(exynos->clks[i]);
		if (ret) {
			while (i-- > 0)
				clk_disable_unprepare(exynos->clks[i]);
			return ret;
		}
	}

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(dwc3_exynos_dev_pm_ops,
				dwc3_exynos_suspend, dwc3_exynos_resume);

static struct platform_driver dwc3_exynos_driver = {
	.probe		= dwc3_exynos_probe,
	.remove		= dwc3_exynos_remove,
	.driver		= {
		.name	= "exynos-dwc3",
		.of_match_table = exynos_dwc3_match,
		.pm	= pm_sleep_ptr(&dwc3_exynos_dev_pm_ops),
	},
};

module_platform_driver(dwc3_exynos_driver);

MODULE_AUTHOR("Anton Tikhomirov <av.tikhomirov@samsung.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("DesignWare USB3 Exynos Glue Layer");
