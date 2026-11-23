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

#define DWC3_EXYNOS_MAX_CLOCKS	4

struct dwc3_exynos_driverdata {
	const char		*clk_names[DWC3_EXYNOS_MAX_CLOCKS];
	int			num_clks;
	int			suspend_clk_idx;
	bool			vbus_session;
};

struct dwc3_exynos {
	struct device		*dev;

	const char		**clk_names;
	struct clk		*clks[DWC3_EXYNOS_MAX_CLOCKS];
	int			num_clks;
	int			suspend_clk_idx;

	struct regulator	*vdd33;
	struct regulator	*vdd10;

	struct notifier_block	core_nb;
	bool			watch_core;

	/* held from attach to detach */
	struct wakeup_source	*vbus_ws;
	spinlock_t		vbus_lock;	/* protects vbus_held */
	bool			vbus_held;
};

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

/*
 * dwc3 maps USB_ROLE_NONE to its default mode, which in device mode changes
 * nothing, so a detach has to end the UDC's VBUS session from here.
 */
static void dwc3_exynos_set_role(struct dwc3 *dwc, enum usb_role role)
{
	struct dwc3_exynos *exynos = NULL;

	/* only the core under this glue gets these ops */
	if (dwc->dev && dwc->dev->parent)
		exynos = dev_get_drvdata(dwc->dev->parent);

	/* keep the system awake for a host session as well */
	if (role == USB_ROLE_NONE)
		dwc3_exynos_vbus_hold(exynos, false);
	else
		dwc3_exynos_vbus_hold(exynos, true);

	/*
	 * A switch to host frees the gadget without clearing dwc->gadget, and a
	 * switch back to device adds a new one with the session on.
	 */
	mutex_lock(&dwc->mutex);
	if (dwc->current_dr_role != DWC3_GCTL_PRTCAP_DEVICE || !dwc->gadget)
		goto out;

	switch (role) {
	case USB_ROLE_NONE:
		usb_udc_vbus_handler(dwc->gadget, false);
		dev_dbg(dwc->dev, "no cable: VBUS session ended\n");
		break;
	case USB_ROLE_DEVICE:
		usb_udc_vbus_handler(dwc->gadget, true);
		dev_dbg(dwc->dev, "cable: VBUS session started\n");
		break;
	case USB_ROLE_HOST:
		break;
	}
out:
	mutex_unlock(&dwc->mutex);
}

static const struct dwc3_glue_ops dwc3_exynos_8895_glue_ops = {
	.pre_set_role	= dwc3_exynos_set_role,
};

/*
 * dwc3_probe() clears dwc->glue_ops, so the ops are set once the core has
 * bound; dwc3 reads them on every role switch.
 */
static void dwc3_exynos_glue_core(struct dwc3_exynos *exynos, struct device *dev)
{
	struct dwc3 *dwc;

	if (!of_device_is_compatible(dev->of_node, "snps,dwc3"))
		return;

	dwc = dev_get_drvdata(dev);
	if (!dwc) {
		dev_err(exynos->dev, "%s: no dwc3 core, no VBUS session\n",
			dev_name(dev));
		return;
	}

	if (dwc->glue_ops == &dwc3_exynos_8895_glue_ops)
		return;

	dwc->glue_ops = &dwc3_exynos_8895_glue_ops;
	dev_dbg(exynos->dev, "USB VBUS session on %s\n", dev_name(dev));
}

/* the core may bind long after of_platform_populate() when its probe defers */
static int dwc3_exynos_core_bound(struct notifier_block *nb, unsigned long action,
				  void *data)
{
	struct dwc3_exynos *exynos = container_of(nb, struct dwc3_exynos, core_nb);
	struct device *dev = data;

	if (action != BUS_NOTIFY_BOUND_DRIVER)
		return NOTIFY_DONE;

	if (dev->parent != exynos->dev)
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
		exynos->vbus_ws = wakeup_source_register(dev, "dwc3-exynos-vbus");
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

static const struct dwc3_exynos_driverdata exynos8895_drvdata = {
	.clk_names = { "aclk", "ref", "usbtvh_ahb", "usbtvh_core" },
	.num_clks = 4,
	.suspend_clk_idx = -1,
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
