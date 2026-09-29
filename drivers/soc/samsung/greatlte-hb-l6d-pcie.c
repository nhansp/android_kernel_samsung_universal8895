// SPDX-License-Identifier: GPL-2.0-only
/*
 * greatlte L6D PCIe RC0 heartbeat rows (spec L6D 1.1, 5.8). DEBUG ONLY.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/soc/samsung/greatlte-hb.h>

static const struct greatlte_hb_item l6d_pcie_rows[] = {
	/* FDT:8217-8251, driver "exynos8895-pcie" */
	{ .layer = 6, .tag = "pcie", .kind = GREATLTE_HB_DEV, .path = "/soc@0/pcie@11700000" },
	/* SPI 345: FDT:8222, FDT:8234 */
	{ .layer = 6, .tag = "pcii", .kind = GREATLTE_HB_IRQ, .path = "/soc@0/pcie@11700000", .idx = 0 },
};

static int __init l6d_pcie_hb_init(void)
{
	greatlte_hb_add_items(l6d_pcie_rows, ARRAY_SIZE(l6d_pcie_rows));
	return 0;
}
late_initcall(l6d_pcie_hb_init);
