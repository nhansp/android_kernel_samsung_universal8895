// SPDX-License-Identifier: GPL-2.0-only
/*
 * greatlte L6C heartbeat rows (spec L6C 1.1, 2.5; L6 1.1): modem and sensor hub. DEBUG ONLY.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/soc/samsung/greatlte-hb.h>

static const struct greatlte_hb_item l6c_rows[] = {
	/* modem interface bound (mif_sipc5, L6C 4.1 node /mif_pdata, V-7) */
	{ .layer = 6, .tag = "cp",   .kind = GREATLTE_HB_DEV, .path = "/mif_pdata" },
	/* AP<->CP mailbox interrupt count (SPI 97, FDT:10658) */
	{ .layer = 6, .tag = "cpmb", .kind = GREATLTE_HB_IRQ, .path = "/soc@0/mailbox@15b40000", .idx = 0 },
	/* BCM4773 SSP hub on spi_10 bound (L6C 4.2) */
	{ .layer = 6, .tag = "hub",  .kind = GREATLTE_HB_DEV, .path = "/soc@0/usi@108c0000/spi@0/gnss-hub@0" },
	/* hub MCU rail LDO38 (FDT:7249) */
	{ .layer = 6, .tag = "l38",  .kind = GREATLTE_HB_REG, .path = "vdd_mcu_1p8" },
};

static int __init l6c_hb_init(void)
{
	greatlte_hb_add_items(l6c_rows, ARRAY_SIZE(l6c_rows));
	return 0;
}
late_initcall(l6c_hb_init);
