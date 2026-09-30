/* sound/soc/samsung/abox/abox_vss.c
 *
 * ALSA SoC Audio Layer - Samsung Abox VSS driver
 *
 * Copyright (c) 2016 Samsung Electronics Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
/* #define DEBUG */
#include <linux/device.h>
#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#if IS_ENABLED(CONFIG_SHM_IPC)
#include <linux/shm_ipc.h>
#endif

#include "abox.h"

#ifndef VSS_MAGIC_OFFSET
#define VSS_MAGIC_OFFSET (0x500000)
#endif

static int samsung_abox_vss_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
#if IS_ENABLED(CONFIG_SHM_IPC)
	phys_addr_t pa;
	u32 *magic;

	if (!shm_get_vss_size()) {
		dev_info(dev, "vss: modem shm absent\n");
		return 0;
	}
	pa = shm_get_phys_base() + shm_get_cp_size() + VSS_MAGIC_OFFSET;
	magic = memremap(pa, sizeof(u32), MEMREMAP_WB);
	if (!magic)
		return -ENOMEM;
	WRITE_ONCE(*magic, 0);
	memunmap(magic);
	return 0;
#else
	dev_info(dev, "vss: modem shm absent\n");
	return 0;
#endif
}

static void samsung_abox_vss_remove(struct platform_device *pdev)
{
}

static const struct of_device_id samsung_abox_vss_match[] = {
	{
		.compatible = "samsung,exynos8895-abox-vss",
	},
	{},
};
MODULE_DEVICE_TABLE(of, samsung_abox_vss_match);

static struct platform_driver samsung_abox_vss_driver = {
	.probe  = samsung_abox_vss_probe,
	.remove = samsung_abox_vss_remove,
	.driver = {
		.name = "samsung-abox-vss",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(samsung_abox_vss_match),
	},
};

module_platform_driver(samsung_abox_vss_driver);

MODULE_AUTHOR("Gyeongtaek Lee, <gt82.lee@samsung.com>");
MODULE_DESCRIPTION("Samsung ASoC A-Box VSS Driver");
MODULE_ALIAS("platform:samsung-abox-vss");
MODULE_LICENSE("GPL");
