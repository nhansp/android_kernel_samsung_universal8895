// SPDX-License-Identifier: GPL-2.0-only
/*
 * greatlte L6F (power supply, input, misc) heartbeat rows. DEBUG ONLY.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/moduleparam.h>
#include <linux/power_supply.h>	/* slice pwr: enum power_supply_property for the PSY rows */
#include <linux/soc/samsung/greatlte-hb.h>

#define L6F_TS_PATH	"/soc@0/usi@108e0000/i2c@0/touchscreen@48"
#define L6F_PEN_PATH	"/soc@0/usi@10480000/i2c@10000/digitizer@56"

/* slice pwr (L6F 1.2) */
#define L6F_ADC_PATH	"/soc@0/adc@15b70000"
#define L6F_PMIC_PATH	"/soc@0/usi@10960000/i2c@10000/pmic@66"

static const struct greatlte_hb_item l6f_rows[] = {
	{ .layer = 6, .tag = "ts",  .kind = GREATLTE_HB_DEV, .path = L6F_TS_PATH },
	{ .layer = 6, .tag = "tsi", .kind = GREATLTE_HB_IRQ, .path = L6F_TS_PATH, .idx = 0 },
	{ .layer = 6, .tag = "l34", .kind = GREATLTE_HB_REG, .path = "tsp_io" },
	{ .layer = 6, .tag = "l35", .kind = GREATLTE_HB_REG, .path = "tsp_avdd" },
	/* Keys and hall (L6F slice A) */
	{ .layer = 6, .tag = "keys", .kind = GREATLTE_HB_DEV, .path = "/gpio-keys" },
	{ .layer = 6, .tag = "hall", .kind = GREATLTE_HB_DEV, .path = "/hall-keys" },

	/* greatlte L6F slice pen */
	{ .layer = 6, .tag = "pen",  .kind = GREATLTE_HB_DEV, .path = L6F_PEN_PATH },
	{ .layer = 6, .tag = "peni", .kind = GREATLTE_HB_IRQ, .path = L6F_PEN_PATH, .idx = 0 },
	{ .layer = 6, .tag = "pdct", .kind = GREATLTE_HB_IRQ, .path = L6F_PEN_PATH, .idx = 1 },

	/* slice pwr: AP ADC + NTC, MAX77865, fuel gauge, charger (L6F 1.2) */
	{ .layer = 6, .tag = "adc",   .kind = GREATLTE_HB_DEV, .path = L6F_ADC_PATH },
	{ .layer = 6, .tag = "adci",  .kind = GREATLTE_HB_IRQ, .path = L6F_ADC_PATH, .idx = 0 },
	{ .layer = 6, .tag = "ntc",   .kind = GREATLTE_HB_TZ, .path = "battery-ntc" },
	{ .layer = 6, .tag = "untc",  .kind = GREATLTE_HB_TZ, .path = "usb-ntc" },
	{ .layer = 6, .tag = "cntc",  .kind = GREATLTE_HB_TZ, .path = "chg-ntc" },
	{ .layer = 6, .tag = "pmic",  .kind = GREATLTE_HB_DEV, .path = L6F_PMIC_PATH },
	{ .layer = 6, .tag = "pmici", .kind = GREATLTE_HB_IRQ, .path = L6F_PMIC_PATH, .idx = 0 },
	{ .layer = 6, .tag = "fgv", .kind = GREATLTE_HB_PSY, .path = "battery",
	  .idx = POWER_SUPPLY_PROP_VOLTAGE_NOW },
	{ .layer = 6, .tag = "fgc", .kind = GREATLTE_HB_PSY, .path = "battery",
	  .idx = POWER_SUPPLY_PROP_CAPACITY },
	{ .layer = 6, .tag = "fgi", .kind = GREATLTE_HB_PSY, .path = "battery",
	  .idx = POWER_SUPPLY_PROP_CURRENT_NOW },
	{ .layer = 6, .tag = "fgt", .kind = GREATLTE_HB_PSY, .path = "battery",
	  .idx = POWER_SUPPLY_PROP_TEMP },
	{ .layer = 6, .tag = "chgo", .kind = GREATLTE_HB_PSY, .path = "max77865-charger",
	  .idx = POWER_SUPPLY_PROP_ONLINE },
	{ .layer = 6, .tag = "chgs", .kind = GREATLTE_HB_PSY, .path = "max77865-charger",
	  .idx = POWER_SUPPLY_PROP_STATUS },
};

static int __init l6f_hb_init(void)
{
	greatlte_hb_add_items(l6f_rows, ARRAY_SIZE(l6f_rows));
	return 0;
}
late_initcall(l6f_hb_init);
