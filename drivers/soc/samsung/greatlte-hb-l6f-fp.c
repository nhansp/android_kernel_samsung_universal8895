// SPDX-License-Identifier: GPL-2.0-only
/*
 * greatlte L6F fingerprint heartbeat rows (spec L6F 1.2 rows fp, fpi). DEBUG ONLY.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/soc/samsung/greatlte-hb.h>

/* DT path is the root node of L6F 4.7 */
static const struct greatlte_hb_item l6f_fp_rows[] = {
	/* driver "vfsspi-exynos8895": L6F 5.10 */
	{ .layer = 6, .tag = "fp",  .kind = GREATLTE_HB_DEV, .path = "/fingerprint" },
	/* DRDY gpa1-3, interrupts-extended index 0 (L6F 4.7): grows on touch while the HAL waits */
	{ .layer = 6, .tag = "fpi", .kind = GREATLTE_HB_IRQ, .path = "/fingerprint", .idx = 0 },
};

static int __init l6f_fp_hb_init(void)
{
	greatlte_hb_add_items(l6f_fp_rows, ARRAY_SIZE(l6f_fp_rows));
	return 0;
}
late_initcall(l6f_fp_hb_init);
