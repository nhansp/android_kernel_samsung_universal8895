// SPDX-License-Identifier: GPL-2.0-only
/*
 * greatlte L6G heartbeat rows (spec L6G 1.2): S2MM005 Type-C port controller, MAX77865
 * MUIC and the dwc3 role switch. Data only, as the other layers' DEBUG files. DEBUG ONLY,
 * never upstream.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/soc/samsung/greatlte-hb.h>

static const struct greatlte_hb_item l6g_rows[] = {
	/* S2MM005 bound on hsi2c_30 (usi12), L6G 4.3 */
	{ .layer = 6, .tag = "cc",   .kind = GREATLTE_HB_DEV,
	  .path = "/soc@0/usi@10940000/i2c@10000/typec@33" },
	/* its interrupt, gpa1-7 falling edge: grows by >= 1 per plug and per unplug */
	{ .layer = 6, .tag = "cci",  .kind = GREATLTE_HB_IRQ,
	  .path = "/soc@0/usi@10940000/i2c@10000/typec@33", .idx = 0 },
	/* MAX77865 MUIC bound on hsi2c_32 (usi13), L6G 4.3 */
	{ .layer = 6, .tag = "muic", .kind = GREATLTE_HB_DEV,
	  .path = "/soc@0/usi@10960000/i2c@10000/muic@25" },
	/* the dwc3 core behind the role switch, L6G 5.4 (the L3 usb row) */
	{ .layer = 6, .tag = "drd",  .kind = GREATLTE_HB_DEV,
	  .path = "/soc@0/usb@10c00000/usb@0" },
	/* L6F's 5.02 V OTG boost: off with nothing or a charger attached, on with an OTG
	 * adapter - and only when CONFIG_TYPEC_S2MM005_OTG_VBUS is set (spec L6G 1.3 G2) */
	{ .layer = 6, .tag = "vbus", .kind = GREATLTE_HB_REG,
	  .path = "usb_otg_vbus" },
};

static int __init l6g_hb_init(void)
{
	greatlte_hb_add_items(l6g_rows, ARRAY_SIZE(l6g_rows));
	return 0;
}
late_initcall(l6g_hb_init);