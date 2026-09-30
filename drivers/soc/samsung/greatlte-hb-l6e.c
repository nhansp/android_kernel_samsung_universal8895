// SPDX-License-Identifier: GPL-2.0-only
/*
 * greatlte L6E (audio) heartbeat rows (spec L6E 1.1). DEBUG ONLY.
 *
 * The "jack" row of 1.1 (the extcon device of the madera-extcon MFD cell) has
 * no DT node to name (the cell is created by madera-core without an of_node),
 * so no GREATLTE_HB_DEV row can point at it; its binding shows in "cdc" (the
 * parent MFD) and in the "jack detection ready" log line (P-A3).
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/soc/samsung/greatlte-hb.h>

static const struct greatlte_hb_item l6e_rows[] = {
	/* CS47L93 on spi3 (L6E 4.1): bound; IRQ grows on jack plug / button */
	{ .layer = 6, .tag = "cdc",   .kind = GREATLTE_HB_DEV, .path = "/soc@0/usi@10460000/spi@0/codec@0" },
	{ .layer = 6, .tag = "cdci",  .kind = GREATLTE_HB_IRQ, .path = "/soc@0/usi@10460000/spi@0/codec@0", .idx = 0 },
	/* codec rails LDO32/LDO33 (L2 4.3, regulator-name) */
	{ .layer = 6, .tag = "l32",   .kind = GREATLTE_HB_REG, .path = "VDD_1.8V_CODEC" },
	{ .layer = 6, .tag = "l33",   .kind = GREATLTE_HB_REG, .path = "VDD_1.2V_CODEC" },
	/* MAX98506 at 11-0031 (L6E 4.2) */
	{ .layer = 6, .tag = "amp",   .kind = GREATLTE_HB_DEV, .path = "/soc@0/usi@104a0000/i2c@0/amplifier@31" },
	/* ABOX root node (F-10), its GIC IRQ (Calliope IPC) and the AUD PLL mux "pll" */
	{ .layer = 6, .tag = "abox",  .kind = GREATLTE_HB_DEV, .path = "/abox@13e50000" },
	{ .layer = 6, .tag = "aboxi", .kind = GREATLTE_HB_IRQ, .path = "/soc@0/interrupt-controller@13ef1000", .idx = 0 },
	{ .layer = 6, .tag = "apll",  .kind = GREATLTE_HB_CLK, .path = "/abox@13e50000", .name = "pll" },
	/* VTS root node and the VTS mailbox parent IRQ */
	{ .layer = 6, .tag = "vts",   .kind = GREATLTE_HB_DEV, .path = "/vts@14020000" },
	{ .layer = 6, .tag = "vtsi",  .kind = GREATLTE_HB_IRQ, .path = "/soc@0/mailbox@14040000", .idx = 0 },
};

static int __init l6e_hb_init(void)
{
	greatlte_hb_add_items(l6e_rows, ARRAY_SIZE(l6e_rows));
	return 0;
}
late_initcall(l6e_hb_init);
