// SPDX-License-Identifier: GPL-2.0-only
/*
 * greatlte L6K (certify hall, DRAM temperature trip, AP/WF/PA thermistors)
 * heartbeat rows (spec L6K 1.1, 5.3). DEBUG ONLY.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/soc/samsung/greatlte-hb.h>

/* DT paths and zone types of L6K section 4 (exynos8895-greatlte-l6k.dtsi, exynos8895.dtsi) */
static const struct greatlte_hb_item l6k_rows[] = {
	/* gpio-keys device "certify_hall" (4.1); its IRQ comes from the GPIO, so no IRQ row (1.1) */
	{ .layer = 6, .tag = "chall", .kind = GREATLTE_HB_DEV, .path = "/certify-hall-keys" },
	/* driver "exynos8895-mcinfo" (5.2) */
	{ .layer = 6, .tag = "mci",   .kind = GREATLTE_HB_DEV, .path = "/soc@0/mcinfo@1603004c" },
	/* DMC0..3 over-temperature SPIs 115/122/129/136 (FDT:6709): any count > 0 is followed by a panic */
	{ .layer = 6, .tag = "mci0",  .kind = GREATLTE_HB_IRQ, .path = "/soc@0/mcinfo@1603004c", .idx = 0 },
	{ .layer = 6, .tag = "mci1",  .kind = GREATLTE_HB_IRQ, .path = "/soc@0/mcinfo@1603004c", .idx = 1 },
	{ .layer = 6, .tag = "mci2",  .kind = GREATLTE_HB_IRQ, .path = "/soc@0/mcinfo@1603004c", .idx = 2 },
	{ .layer = 6, .tag = "mci3",  .kind = GREATLTE_HB_IRQ, .path = "/soc@0/mcinfo@1603004c", .idx = 3 },
	/* thermal zones of 4.3: milli-degC */
	{ .layer = 6, .tag = "tap",   .kind = GREATLTE_HB_TZ,  .path = "sec-ap-thermistor" },
	{ .layer = 6, .tag = "twf",   .kind = GREATLTE_HB_TZ,  .path = "sec-wf-thermistor" },
	{ .layer = 6, .tag = "tpa",   .kind = GREATLTE_HB_TZ,  .path = "sec-pa-thermistor" },
};

static int __init l6k_hb_init(void)
{
	greatlte_hb_add_items(l6k_rows, ARRAY_SIZE(l6k_rows));
	return 0;
}
late_initcall(l6k_hb_init);
