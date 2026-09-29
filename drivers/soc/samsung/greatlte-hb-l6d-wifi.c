// SPDX-License-Identifier: GPL-2.0-only
/*
 * greatlte L6D Wi-Fi heartbeat rows (spec L6D 1.1, 5.8). DEBUG ONLY.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/soc/samsung/greatlte-hb.h>

static const struct greatlte_hb_item l6d_wifi_rows[] = {
	/* FDT:11161-11165, driver "bcmdhd-exynos8895" (L6D 5.3.2) */
	{ .layer = 6, .tag = "wlan", .kind = GREATLTE_HB_DEV, .path = "/wlan" },
	/* WLAN_HOST_WAKE: FDT:11162 gpa0-7, IRQ_TYPE_EDGE_RISING */
	{ .layer = 6, .tag = "wlw",  .kind = GREATLTE_HB_IRQ, .path = "/wlan", .idx = 0 },
};

static int __init l6d_wifi_hb_init(void)
{
	greatlte_hb_add_items(l6d_wifi_rows, ARRAY_SIZE(l6d_wifi_rows));
	return 0;
}
late_initcall(l6d_wifi_hb_init);
