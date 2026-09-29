// SPDX-License-Identifier: GPL-2.0-only
/*
 * greatlte L6F (power supply, input, misc) heartbeat rows. DEBUG ONLY.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/moduleparam.h>
#include <linux/soc/samsung/greatlte-hb.h>

#define L6F_TS_PATH	"/soc@0/usi@108e0000/i2c@0/touchscreen@48"

static const struct greatlte_hb_item l6f_rows[] = {
	{ .layer = 6, .tag = "ts",  .kind = GREATLTE_HB_DEV, .path = L6F_TS_PATH },
	{ .layer = 6, .tag = "tsi", .kind = GREATLTE_HB_IRQ, .path = L6F_TS_PATH, .idx = 0 },
	{ .layer = 6, .tag = "l34", .kind = GREATLTE_HB_REG, .path = "tsp_io" },
	{ .layer = 6, .tag = "l35", .kind = GREATLTE_HB_REG, .path = "tsp_avdd" },
	/* Keys and hall (L6F slice A) */
	{ .layer = 6, .tag = "keys", .kind = GREATLTE_HB_DEV, .path = "/gpio-keys" },
	{ .layer = 6, .tag = "hall", .kind = GREATLTE_HB_DEV, .path = "/hall-keys" },

};

static int __init l6f_hb_init(void)
{
	greatlte_hb_add_items(l6f_rows, ARRAY_SIZE(l6f_rows));
	return 0;
}
late_initcall(l6f_hb_init);
