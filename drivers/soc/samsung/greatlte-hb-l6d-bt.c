// SPDX-License-Identifier: GPL-2.0-only
/*
 * greatlte L6D (Bluetooth) heartbeat rows (spec L6D 1.1, 5.8). DEBUG ONLY.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/soc/samsung/greatlte-hb.h>

/* DT path is the root node of L6D section 4.4 */
static const struct greatlte_hb_item l6d_bt_rows[] = {
	/* driver "bcm43xx-bt-lpm": L6D 1.1, 5.7, 5.8 */
	{ .layer = 6, .tag = "bt",  .kind = GREATLTE_HB_DEV, .path = "/bluetooth" },
	/* BT_HOST_WAKE gpa2-3: FDT:11977, S:drivers/bluetooth/bcm43xx.c:254-256, L6D 1.1, 5.8 */
	{ .layer = 6, .tag = "btw", .kind = GREATLTE_HB_IRQ, .path = "/bluetooth", .idx = 0 },
};

static int __init l6d_bt_hb_init(void)
{
	greatlte_hb_add_items(l6d_bt_rows, ARRAY_SIZE(l6d_bt_rows));
	return 0;
}
late_initcall(l6d_bt_hb_init);
