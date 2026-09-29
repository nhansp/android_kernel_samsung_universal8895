// SPDX-License-Identifier: GPL-2.0-only
/*
 * bcmdhd4361 platform glue for Exynos8895 (greatlte) - L6D 5.3.2.
 *
 * Replaces stock S:drivers/net/wireless/bcmdhd4361/dhd_custom_exynos.c.
 * Built outside the dhd object (no DHDCFLAGS) like stock
 * (S:drivers/net/wireless/Makefile:60-61).
 *
 * Platform driver on the DT "wlan" node (compatible "samsung,brcm-wlan"),
 * defers until the PCIe host is bound (D7), drives WLAN_REG_ON via gpiod,
 * calls the stock PCIe power API, preallocates memory, then runs
 * dhd_module_init() itself (replaces the stock deferred_module_init).
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/wlan_plat.h>
#include <linux/exynos-pci-ctrl.h>

/* Entry points of the dhd driver (stock glue externs, S:.../dhd_custom_exynos.c:72) */
#if IS_ENABLED(CONFIG_BCMDHD_EXYNOS8895_GLUE_ONLY)
/* bring-up stub (T-F1 only, L6D 6.2 R3-03) */
static int dhd_module_init(void) { return 0; }
static int dhd_init_wlan_mem(void) { return 0; }
#else
#include "dhd_custom_exynos8895.h"
#endif

#define WIFI_TURNON_DELAY	200	/* S:.../dhd_custom_exynos.c:76 */
#define SAMSUNG_PCIE_CH_NUM	0	/* S:.../dhd_custom_exynos.c:96-98 */

static struct gpio_desc *wlan_reg_on;

/* S:.../dhd_custom_exynos.c:299-309 */
struct resource dhd_wlan_resources = {
	.name	= "bcmdhd_wlan_irq",
	.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_SHAREABLE |
		  IORESOURCE_IRQ_HIGHEDGE,
};
EXPORT_SYMBOL(dhd_wlan_resources);

/* S:.../dhd_custom_exynos.c:115-143 verbatim logic, strings kept */
static int dhd_wlan_power(int onoff)
{
	printk(KERN_INFO "%s Enter: power %s\n", __func__,
	       onoff ? "on" : "off");
	if (!onoff)
		exynos_pcie_pm_suspend(SAMSUNG_PCIE_CH_NUM);
	gpiod_set_value(wlan_reg_on, onoff);	/* S:.../dhd_custom_exynos.c:131 */
	if (onoff) {
		printk(KERN_ERR "%s Disable L1ss EP side\n", __func__);	/* :139 */
		exynos_pcie_l1ss_ctrl(0, PCIE_L1SS_CTRL_WIFI);		/* :140 */
		exynos_pcie_pm_resume(SAMSUNG_PCIE_CH_NUM);		/* :142 */
	}
	return 0;
}

/* S:.../dhd_custom_exynos.c:176-180 */
static int dhd_wlan_reset(int onoff) { return 0; }

/* S:.../dhd_custom_exynos.c:312-321 */
struct wifi_platform_data dhd_wlan_control = {
	.set_power	= dhd_wlan_power,
	.set_reset	= dhd_wlan_reset,
#if !IS_ENABLED(CONFIG_BCMDHD_EXYNOS8895_GLUE_ONLY)
	.mem_prealloc	= dhd_wlan_mem_prealloc,
#endif
};
EXPORT_SYMBOL(dhd_wlan_control);

/* = dhd_wlan_init + dhd_wlan_init_gpio, S:.../dhd_custom_exynos.c:202-352 */
static int bcmdhd_exynos8895_probe(struct platform_device *pdev)
{
	struct device_node *pn = of_parse_phandle(pdev->dev.of_node,
						  "samsung,pcie", 0);
	struct platform_device *pp = pn ? of_find_device_by_node(pn) : NULL;
	bool host_ready;
	int irq, ret;

	of_node_put(pn);
	host_ready = pp && platform_get_drvdata(pp);
	if (pp)
		put_device(&pp->dev);	/* (final, R3-15b) drop of_find ref;
					 * host is suppress_bind_attrs + built in */
	if (!host_ready)
		return -EPROBE_DEFER;			/* host not bound yet (D7) */

	/* S:.../dhd_custom_exynos.c:221-232: request + drive 1 */
	wlan_reg_on = devm_gpiod_get(&pdev->dev, "wlan-reg-on", GPIOD_OUT_HIGH);
	if (IS_ERR(wlan_reg_on))
		return PTR_ERR(wlan_reg_on);

	dev_info(&pdev->dev, "WLAN_REG_ON 1, wait %d ms\n",
		 WIFI_TURNON_DELAY);				/* 8.3 S3 */
	msleep(WIFI_TURNON_DELAY);				/* :233 */

	ret = exynos_pcie_pm_resume(SAMSUNG_PCIE_CH_NUM);	/* :241-243 */
	if (ret)
		dev_err(&pdev->dev,
			"first PCIe power-on failed: %d\n", ret);

	irq = platform_get_irq(pdev, 0);			/* :247-262 */
	if (irq < 0)
		return irq;

	dhd_wlan_resources.start = dhd_wlan_resources.end = irq;	/* :338-339 */

	ret = dhd_init_wlan_mem();					/* :342-348 */
	if (ret < 0) {
		dev_err(&pdev->dev,
			"failed to alloc reserved memory, ret=%d\n", ret);
		/* (run 6) stock continued with dangling pointers; dropping
		 * .mem_prealloc makes wifi_platform_prealloc return NULL and
		 * dhd fails cleanly on any section request. */
		dhd_wlan_control.mem_prealloc = NULL;
	}

	return dhd_module_init();		/* replaces deferred_module_init_sync (D7) */
}

static const struct of_device_id bcmdhd_exynos8895_of[] = {
	{ .compatible = "samsung,brcm-wlan" },	/* FDT:11161 */
	{ }
};

static struct platform_driver bcmdhd_exynos8895_drv = {
	.probe	= bcmdhd_exynos8895_probe,
	.driver	= {
		.name		= "bcmdhd-exynos8895",
		.of_match_table	= bcmdhd_exynos8895_of,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(bcmdhd_exynos8895_drv);
