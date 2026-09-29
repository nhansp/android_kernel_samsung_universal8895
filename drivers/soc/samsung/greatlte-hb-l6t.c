// SPDX-License-Identifier: GPL-2.0-only
/*
 * greatlte L6T (Trustonic Kinibi TEE, secure RPMB proxy, secure-world log)
 * heartbeat rows (spec L6T 1.1, 5.10). DEBUG ONLY.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/moduleparam.h>
#include <linux/soc/samsung/greatlte-hb.h>

/* DT paths are the root nodes of L6T section 4 */
static const struct greatlte_hb_item l6t_rows[] = {
	/* driver "mcd": S:drivers/gud/gud-exynos8895_kinibi400/MobiCoreDriver/main.c:756 */
	{ .layer = 6, .tag = "tee",   .kind = GREATLTE_HB_DEV, .path = "/tee" },
	/* S-SIQ, SPI 233: FDT:8297, S:drivers/gud/gud-exynos8895_kinibi400/MobiCoreDriver/nq.c:650-651 */
	{ .layer = 6, .tag = "teei",  .kind = GREATLTE_HB_IRQ, .path = "/tee", .idx = 0 },
	/* driver "exynos-ufs-srpmb": S:drivers/scsi/scsi_srpmb.c:341 */
	{ .layer = 6, .tag = "rpmb",  .kind = GREATLTE_HB_DEV, .path = "/ufs-srpmb" },
	/* SPI 69: FDT:8292 */
	{ .layer = 6, .tag = "rpmbi", .kind = GREATLTE_HB_IRQ, .path = "/ufs-srpmb", .idx = 0 },
	/* driver "exynos-seclog": S:drivers/soc/samsung/exynos-seclog.c:337 */
	{ .layer = 6, .tag = "slog",  .kind = GREATLTE_HB_DEV, .path = "/seclog" },
	/* SPI 68: FDT:8287 */
	{ .layer = 6, .tag = "slogi", .kind = GREATLTE_HB_IRQ, .path = "/seclog", .idx = 0 },
};

static int __init l6t_hb_init(void)
{
	greatlte_hb_add_items(l6t_rows, ARRAY_SIZE(l6t_rows));
	return 0;
}
late_initcall(l6t_hb_init);
