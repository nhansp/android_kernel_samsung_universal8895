// SPDX-License-Identifier: GPL-2.0-only
/*
 * greatlte L6I (NFC controller) heartbeat rows (spec L6I 1.1, 5.3). DEBUG ONLY.
 * PMU 0xA0C (CLKOUT1) itself is already printed as "ck1" by greatlte-hs.c.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/soc/samsung/greatlte-hb.h>

/* DT path of L6I 4.1 (exynos8895-greatlte-l6i.dtsi) */
static const struct greatlte_hb_item l6i_rows[] = {
	/* driver "sec-nfc" (S:drivers/nfc/sec_nfc.c:1077) */
	{ .layer = 6, .tag = "nfc",  .kind = GREATLTE_HB_DEV,
	  .path = "/soc@0/usi@10900000/i2c@10000/nfc@27" },
	/* NFC IRQ gpa1-2 (FDT:5949-5950): grows while a tag is in the field */
	{ .layer = 6, .tag = "nfci", .kind = GREATLTE_HB_IRQ,
	  .path = "/soc@0/usi@10900000/i2c@10000/nfc@27", .idx = 0 },
	/* CLKOUT1 "oscclk_nfc" (FDT:5960): 26000000:0 with NFC off, :1 while the chip requests it */
	{ .layer = 6, .tag = "nfck", .kind = GREATLTE_HB_CLK,
	  .path = "/soc@0/usi@10900000/i2c@10000/nfc@27", .name = "oscclk_nfc" },
	/* LDO39 (FDT:7254-7258): 1:1800000 after probe (not in LPM charging) */
	{ .layer = 6, .tag = "nfcv", .kind = GREATLTE_HB_REG, .path = "VDD_NFC_1P8" },
};

static int __init l6i_hb_init(void)
{
	greatlte_hb_add_items(l6i_rows, ARRAY_SIZE(l6i_rows));
	return 0;
}
late_initcall(l6i_hb_init);
