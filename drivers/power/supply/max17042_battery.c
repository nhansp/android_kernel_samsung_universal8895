// SPDX-License-Identifier: GPL-2.0+
//
// Fuel gauge driver for Maxim 17042 / 8966 / 8997
//  Note that Maxim 8966 and 8997 are mfd and this is its subdevice.
//
// Copyright (C) 2011 Samsung Electronics
// MyungJoo Ham <myungjoo.ham@samsung.com>
//
// This driver is based on max17040_battery.c

#include <linux/acpi.h>
#include <linux/devm-helpers.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/mod_devicetable.h>
#include <linux/power_supply.h>
#include <linux/power/max17042_battery.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/mfd/max77865-private.h>

/* Status register bits */
#define STATUS_POR_BIT         (1 << 1)
#define STATUS_BST_BIT         (1 << 3)
#define STATUS_VMN_BIT         (1 << 8)
#define STATUS_TMN_BIT         (1 << 9)
#define STATUS_SMN_BIT         (1 << 10)
#define STATUS_BI_BIT          (1 << 11)
#define STATUS_VMX_BIT         (1 << 12)
#define STATUS_TMX_BIT         (1 << 13)
#define STATUS_SMX_BIT         (1 << 14)
#define STATUS_BR_BIT          (1 << 15)

/* Interrupt mask bits */
#define CFG_ALRT_BIT_ENBL	(1 << 2)

#define VFSOC0_LOCK		0x0000
#define VFSOC0_UNLOCK		0x0080
#define MODEL_UNLOCK1	0X0059
#define MODEL_UNLOCK2	0X00C4
#define MODEL_LOCK1		0X0000
#define MODEL_LOCK2		0X0000

#define dQ_ACC_DIV	0x4
#define dP_ACC_100	0x1900
#define dP_ACC_200	0x3200

#define MAX17042_VMAX_TOLERANCE		50 /* 50 mV */

#if IS_ENABLED(CONFIG_CHARGER_MAX77865_TTF)
/* struct cv_slope of Samsung's drivers/battery_v2/include/fuelgauge/max77865_fuelgauge.h */
struct cv_slope {
	u32 fg_current;
	u32 soc;
	u32 time;
};
#endif

struct max17042_chip {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *battery;
	enum max170xx_chip_type chip_type;
	struct max17042_platform_data *pdata;
	struct work_struct work;
	int    init_complete;
	int    irq;
	/* MAX77865 (Galaxy Note 8): host-written temperature, spec L6F 5.4 */
	bool   host_temp;
	bool   low_temp_wa;
	const char *tz_name;
	struct power_supply_desc psy_desc;
	struct delayed_work temp_work;
	/* MAX77865: Samsung's capacity and full-charge policy, SOC values in 0.1 % */
	struct delayed_work soc_work;
	struct mutex soc_lock;
	bool   soc_ready;
	int    capacity_max_cfg;
	int    capacity_margin;
	int    capacity_min;
	int    full_vcell_mv;
	int    full_soc;
	int    rechg_vcell_mv;
	int    capacity_max;
	int    capacity_old;
	int    capacity;
	int    soc_permille;
	bool   initial_update;
	bool   full;
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_TTF)
	struct cv_slope *cv_data;	/* maxim,ttf-cv-data */
	int    cv_data_length;
#endif
};

#if IS_ENABLED(CONFIG_CHARGER_MAX77865_TTF)
/*
 * max17042_battery_props below plus TIME_TO_FULL_NOW, kept in front of the two current
 * properties so that dropping the last two (no current sense) works as for the list below
 */
static enum power_supply_property max77865_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MIN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TEMP_ALERT_MIN,
	POWER_SUPPLY_PROP_TEMP_ALERT_MAX,
	POWER_SUPPLY_PROP_TEMP_MIN,
	POWER_SUPPLY_PROP_TEMP_MAX,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_SCOPE,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	// these two have to be at the end on the list
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
};
#endif

static enum power_supply_property max17042_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MIN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TEMP_ALERT_MIN,
	POWER_SUPPLY_PROP_TEMP_ALERT_MAX,
	POWER_SUPPLY_PROP_TEMP_MIN,
	POWER_SUPPLY_PROP_TEMP_MAX,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_SCOPE,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	// these two have to be at the end on the list
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
};

static int max17042_get_temperature(struct max17042_chip *chip, int *temp)
{
	int ret;
	u32 data;
	struct regmap *map = chip->regmap;

	ret = regmap_read(map, MAX17042_TEMP, &data);
	if (ret < 0)
		return ret;

	*temp = sign_extend32(data, 15);
	/* The value is converted into deci-centigrade scale */
	/* Units of LSB = 1 / 256 degree Celsius */
	*temp = *temp * 10 / 256;
	return 0;
}

static int max17042_get_status(struct max17042_chip *chip, int *status)
{
	int ret, charge_full, charge_now;
	int avg_current;
	u32 data;

	/*
	 * MAX77865: while supplied, report the charger's status (NOT_CHARGING during
	 * thermal/OVP/timer stops), as stock reported sec_battery's. The charger's end of
	 * charge only counts as FULL once max77865_soc_work() has taken it
	 * (sec_bat_fullcharged_check() in Samsung's drivers/battery_v2/sec_battery.c).
	 * The charger never reads this STATUS.
	 */
	if (chip->host_temp && power_supply_am_i_supplied(chip->battery) > 0) {
		union power_supply_propval v;

		ret = power_supply_get_property_from_supplier(chip->battery,
							      POWER_SUPPLY_PROP_STATUS, &v);
		if (ret == 0) {
			if (v.intval == POWER_SUPPLY_STATUS_CHARGING ||
			    v.intval == POWER_SUPPLY_STATUS_FULL)
				*status = READ_ONCE(chip->full) ? POWER_SUPPLY_STATUS_FULL :
								  POWER_SUPPLY_STATUS_CHARGING;
			else
				*status = v.intval;
			return 0;
		}
	}

	ret = power_supply_am_i_supplied(chip->battery);
	if (ret < 0) {
		*status = POWER_SUPPLY_STATUS_UNKNOWN;
		return 0;
	}
	if (ret == 0) {
		*status = POWER_SUPPLY_STATUS_DISCHARGING;
		return 0;
	}

	/*
	 * The MAX170xx has builtin end-of-charge detection and will update
	 * FullCAP to match RepCap when it detects end of charging.
	 *
	 * When this cycle the battery gets charged to a higher (calculated)
	 * capacity then the previous cycle then FullCAP will get updated
	 * continuously once end-of-charge detection kicks in, so allow the
	 * 2 to differ a bit.
	 */

	ret = regmap_read(chip->regmap, MAX17042_FullCAP, &charge_full);
	if (ret < 0)
		return ret;

	ret = regmap_read(chip->regmap, MAX17042_RepCap, &charge_now);
	if (ret < 0)
		return ret;

	if ((charge_full - charge_now) <= MAX17042_FULL_THRESHOLD) {
		*status = POWER_SUPPLY_STATUS_FULL;
		return 0;
	}

	/*
	 * Even though we are supplied, we may still be discharging if the
	 * supply is e.g. only delivering 5V 0.5A. Check current if available.
	 */
	if (!chip->pdata->enable_current_sense) {
		*status = POWER_SUPPLY_STATUS_CHARGING;
		return 0;
	}

	ret = regmap_read(chip->regmap, MAX17042_AvgCurrent, &data);
	if (ret < 0)
		return ret;

	avg_current = sign_extend32(data, 15);
	avg_current *= 1562500 / chip->pdata->r_sns;

	if (avg_current > 0)
		*status = POWER_SUPPLY_STATUS_CHARGING;
	else
		*status = POWER_SUPPLY_STATUS_DISCHARGING;

	return 0;
}

static int max17042_get_battery_health(struct max17042_chip *chip, int *health)
{
	int temp, vavg, vbatt, ret;
	u32 val;

	ret = regmap_read(chip->regmap, MAX17042_AvgVCELL, &val);
	if (ret < 0)
		goto health_error;

	/* bits [0-3] unused */
	vavg = val * 625 / 8;
	/* Convert to millivolts */
	vavg /= 1000;

	ret = regmap_read(chip->regmap, MAX17042_VCELL, &val);
	if (ret < 0)
		goto health_error;

	/* bits [0-3] unused */
	vbatt = val * 625 / 8;
	/* Convert to millivolts */
	vbatt /= 1000;

	if (vavg < chip->pdata->vmin) {
		*health = POWER_SUPPLY_HEALTH_DEAD;
		goto out;
	}

	if (vbatt > size_add(chip->pdata->vmax, MAX17042_VMAX_TOLERANCE)) {
		*health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		goto out;
	}

	ret = max17042_get_temperature(chip, &temp);
	if (ret < 0)
		goto health_error;

	if (temp < chip->pdata->temp_min) {
		*health = POWER_SUPPLY_HEALTH_COLD;
		goto out;
	}

	if (temp > chip->pdata->temp_max) {
		*health = POWER_SUPPLY_HEALTH_OVERHEAT;
		goto out;
	}

	*health = POWER_SUPPLY_HEALTH_GOOD;

out:
	return 0;

health_error:
	return ret;
}

#if IS_ENABLED(CONFIG_CHARGER_MAX77865_TTF)
/* stock calc_ttf() of Samsung's drivers/battery_v2/max77865_fuelgauge.c */
static int max17042_calc_ttf(struct max17042_chip *chip, int charge_current)
{
	int i, cc_time = 0, cv_time = 0;
	int soc, design_cap, ret;
	struct cv_slope *cv_data = chip->cv_data;
	unsigned int full_cap;
	u64 data64;

	if (!cv_data || chip->cv_data_length <= 0 || charge_current <= 0)
		return -ENODATA;

	/* the scaled SOC in 0.1 %, as stock's raw_capacity */
	soc = READ_ONCE(chip->soc_permille);

	/*
	 * Stock uses its fixed fuelgauge,ttf_capacity (0xce4 = 3300 mAh); the
	 * learned FullCAP keeps the CC time right for an aged cell.
	 */
	ret = regmap_read(chip->regmap, MAX17042_FullCAP, &full_cap);
	if (ret < 0)
		return ret;
	data64 = full_cap * 5000ull;
	do_div(data64, chip->pdata->r_sns);
	design_cap = data64;
	if (!design_cap)
		return -ENODATA;

	for (i = 0; i < chip->cv_data_length; i++) {
		if (charge_current >= cv_data[i].fg_current)
			break;
	}
	i = (i >= chip->cv_data_length) ? chip->cv_data_length - 1 : i;

	if (cv_data[i].soc < soc) {
		for (i = 0; i < chip->cv_data_length; i++) {
			if (soc <= cv_data[i].soc)
				break;
		}
		if (i > 0 && i < chip->cv_data_length && (cv_data[i].soc != cv_data[i - 1].soc)) {
			cv_time = ((cv_data[i - 1].time - cv_data[i].time) * (cv_data[i].soc - soc) /
				   (cv_data[i].soc - cv_data[i - 1].soc)) + cv_data[i].time;
		} else {
			cv_time = (i < chip->cv_data_length) ? cv_data[i].time : 0;
		}
	} else {
		cv_time = cv_data[i].time;
		cc_time = (design_cap * (cv_data[i].soc - soc) / charge_current) * 3600 / 1000;
		if (cc_time < 0)
			cc_time = 0;
	}

	if (cv_time + cc_time >= 0)
		return cv_time + cc_time + 60;
	else
		return 60;
}
#endif

static int max17042_get_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    union power_supply_propval *val)
{
	struct max17042_chip *chip = power_supply_get_drvdata(psy);
	struct regmap *map = chip->regmap;
	int ret;
	u32 data;
	u64 data64;

	if (!chip->init_complete)
		return -EAGAIN;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		ret = max17042_get_status(chip, &val->intval);
		if (ret < 0)
			return ret;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		ret = regmap_read(map, MAX17042_STATUS, &data);
		if (ret < 0)
			return ret;

		if (data & MAX17042_STATUS_BattAbsent)
			val->intval = 0;
		else
			val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		ret = regmap_read(map, MAX17042_Cycles, &data);
		if (ret < 0)
			return ret;

		val->intval = data;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		ret = regmap_read(map, MAX17042_MinMaxVolt, &data);
		if (ret < 0)
			return ret;

		val->intval = data >> 8;
		val->intval *= 20000; /* Units of LSB = 20mV */
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
		ret = regmap_read(map, MAX17042_MinMaxVolt, &data);
		if (ret < 0)
			return ret;

		val->intval = (data & 0xff) * 20000; /* Units of 20mV */
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		if (chip->chip_type == MAXIM_DEVICE_TYPE_MAX17042)
			ret = regmap_read(map, MAX17042_V_empty, &data);
		else
			ret = regmap_read(map, MAX17047_V_empty, &data);
		if (ret < 0)
			return ret;

		val->intval = data >> 7;
		val->intval *= 10000; /* Units of LSB = 10mV */
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = regmap_read(map, MAX17042_VCELL, &data);
		if (ret < 0)
			return ret;

		val->intval = data * 625 / 8;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		ret = regmap_read(map, MAX17042_AvgVCELL, &data);
		if (ret < 0)
			return ret;

		val->intval = data * 625 / 8;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		ret = regmap_read(map, MAX17042_OCVInternal, &data);
		if (ret < 0)
			return ret;

		val->intval = data * 625 / 8;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (chip->host_temp) {
			/* sec_bat_get_property(): 100 while the battery is full */
			val->intval = READ_ONCE(chip->full) ? 100 : READ_ONCE(chip->capacity);
			break;
		}
		if (chip->pdata->enable_current_sense)
			ret = regmap_read(map, MAX17042_RepSOC, &data);
		else
			ret = regmap_read(map, MAX17042_VFSOC, &data);
		if (ret < 0)
			return ret;

		val->intval = data >> 8;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		ret = regmap_read(map, MAX17042_DesignCap, &data);
		if (ret < 0)
			return ret;

		data64 = data * 5000000ll;
		do_div(data64, chip->pdata->r_sns);
		val->intval = data64;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = regmap_read(map, MAX17042_FullCAP, &data);
		if (ret < 0)
			return ret;

		data64 = data * 5000000ll;
		do_div(data64, chip->pdata->r_sns);
		val->intval = data64;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		ret = regmap_read(map, MAX17042_RepCap, &data);
		if (ret < 0)
			return ret;

		data64 = data * 5000000ll;
		do_div(data64, chip->pdata->r_sns);
		val->intval = data64;
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		if (chip->host_temp) {
			/*
			 * The remaining charge, design capacity times the scaled SOC, as
			 * Samsung's max77865_fg_get_property() reports it. QH is a
			 * coulomb counter without a zero point.
			 */
			ret = regmap_read(map, MAX17042_DesignCap, &data);
			if (ret < 0)
				return ret;

			data64 = data * 5000000ll;
			do_div(data64, chip->pdata->r_sns);
			val->intval = div_u64(data64 * READ_ONCE(chip->soc_permille), 1000);
			break;
		}
		ret = regmap_read(map, MAX17042_QH, &data);
		if (ret < 0)
			return ret;

		data64 = sign_extend64(data, 15) * 5000000ll;
		val->intval = div_s64(data64, chip->pdata->r_sns);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = max17042_get_temperature(chip, &val->intval);
		if (ret < 0)
			return ret;
		break;
	case POWER_SUPPLY_PROP_TEMP_ALERT_MIN:
		ret = regmap_read(map, MAX17042_TALRT_Th, &data);
		if (ret < 0)
			return ret;
		/* LSB is Alert Minimum. In deci-centigrade */
		val->intval = sign_extend32(data & 0xff, 7) * 10;
		break;
	case POWER_SUPPLY_PROP_TEMP_ALERT_MAX:
		ret = regmap_read(map, MAX17042_TALRT_Th, &data);
		if (ret < 0)
			return ret;
		/* MSB is Alert Maximum. In deci-centigrade */
		val->intval = sign_extend32(data >> 8, 7) * 10;
		break;
	case POWER_SUPPLY_PROP_TEMP_MIN:
		val->intval = chip->pdata->temp_min;
		break;
	case POWER_SUPPLY_PROP_TEMP_MAX:
		val->intval = chip->pdata->temp_max;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		ret = max17042_get_battery_health(chip, &val->intval);
		if (ret < 0)
			return ret;
		break;
	case POWER_SUPPLY_PROP_SCOPE:
		val->intval = POWER_SUPPLY_SCOPE_SYSTEM;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		if (chip->pdata->enable_current_sense) {
			ret = regmap_read(map, MAX17042_Current, &data);
			if (ret < 0)
				return ret;

			data64 = sign_extend64(data, 15) * 1562500ll;
			val->intval = div_s64(data64, chip->pdata->r_sns);
		} else {
			return -EINVAL;
		}
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		if (chip->pdata->enable_current_sense) {
			ret = regmap_read(map, MAX17042_AvgCurrent, &data);
			if (ret < 0)
				return ret;

			data64 = sign_extend64(data, 15) * 1562500ll;
			val->intval = div_s64(data64, chip->pdata->r_sns);
		} else {
			return -EINVAL;
		}
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		ret = regmap_read(map, MAX17042_ICHGTerm, &data);
		if (ret < 0)
			return ret;

		data64 = data * 1562500ll;
		val->intval = div_s64(data64, chip->pdata->r_sns);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		ret = regmap_read(map, MAX17042_TTE, &data);
		if (ret < 0)
			return ret;

		val->intval = data * 5625 / 1000;
		break;
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_TTF)
	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW: {
		union power_supply_propval val_supplier;

		if (!chip->host_temp)
			return -ENODATA;
		ret = power_supply_get_property_from_supplier(chip->battery,
				POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
				&val_supplier);
		if (ret == 0 && val_supplier.intval > 0) {
			int ttf = max17042_calc_ttf(chip, val_supplier.intval / 1000);

			if (ttf < 0)
				return ttf;
			val->intval = ttf;
			return 0;
		}
		return -ENODATA;
	}
#endif
	default:
		return -EINVAL;
	}
	return 0;
}

static int max17042_set_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    const union power_supply_propval *val)
{
	struct max17042_chip *chip = power_supply_get_drvdata(psy);
	struct regmap *map = chip->regmap;
	int ret = 0;
	u32 data;
	int8_t temp;

	switch (psp) {
	case POWER_SUPPLY_PROP_TEMP_ALERT_MIN:
		ret = regmap_read(map, MAX17042_TALRT_Th, &data);
		if (ret < 0)
			return ret;

		/* Input in deci-centigrade, convert to centigrade */
		temp = val->intval / 10;
		/* force min < max */
		if (temp >= (int8_t)(data >> 8))
			temp = (int8_t)(data >> 8) - 1;
		/* Write both MAX and MIN ALERT */
		data = (data & 0xff00) + temp;
		ret = regmap_write(map, MAX17042_TALRT_Th, data);
		break;
	case POWER_SUPPLY_PROP_TEMP_ALERT_MAX:
		ret = regmap_read(map, MAX17042_TALRT_Th, &data);
		if (ret < 0)
			return ret;

		/* Input in Deci-Centigrade, convert to centigrade */
		temp = val->intval / 10;
		/* force max > min */
		if (temp <= (int8_t)(data & 0xff))
			temp = (int8_t)(data & 0xff) + 1;
		/* Write both MAX and MIN ALERT */
		data = (data & 0xff) + (temp << 8);
		ret = regmap_write(map, MAX17042_TALRT_Th, data);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int max17042_property_is_writeable(struct power_supply *psy,
		enum power_supply_property psp)
{
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_TEMP_ALERT_MIN:
	case POWER_SUPPLY_PROP_TEMP_ALERT_MAX:
		ret = 1;
		break;
	default:
		ret = 0;
	}

	return ret;
}

static int max17042_write_verify_reg(struct regmap *map, u8 reg, u32 value)
{
	int retries = 8;
	int ret;
	u32 read_value;

	do {
		ret = regmap_write(map, reg, value);
		regmap_read(map, reg, &read_value);
		if (read_value != value) {
			ret = -EIO;
			retries--;
		}
	} while (retries && read_value != value);

	if (ret < 0)
		pr_err("%s: err %d\n", __func__, ret);

	return ret;
}

static inline void max17042_override_por(struct regmap *map,
					 u8 reg, u16 value)
{
	if (value)
		regmap_write(map, reg, value);
}

static inline void max17042_unlock_model(struct max17042_chip *chip)
{
	struct regmap *map = chip->regmap;

	regmap_write(map, MAX17042_MLOCKReg1, MODEL_UNLOCK1);
	regmap_write(map, MAX17042_MLOCKReg2, MODEL_UNLOCK2);
}

static inline void max17042_lock_model(struct max17042_chip *chip)
{
	struct regmap *map = chip->regmap;

	regmap_write(map, MAX17042_MLOCKReg1, MODEL_LOCK1);
	regmap_write(map, MAX17042_MLOCKReg2, MODEL_LOCK2);
}

static inline void max17042_write_model_data(struct max17042_chip *chip,
					u8 addr, int size)
{
	struct regmap *map = chip->regmap;
	int i;

	for (i = 0; i < size; i++)
		regmap_write(map, addr + i,
			chip->pdata->config_data->cell_char_tbl[i]);
}

static inline void max17042_read_model_data(struct max17042_chip *chip,
					u8 addr, u16 *data, int size)
{
	struct regmap *map = chip->regmap;
	int i;
	u32 tmp;

	for (i = 0; i < size; i++) {
		regmap_read(map, addr + i, &tmp);
		data[i] = (u16)tmp;
	}
}

static inline int max17042_model_data_compare(struct max17042_chip *chip,
					u16 *data1, u16 *data2, int size)
{
	int i;

	if (memcmp(data1, data2, size)) {
		dev_err(chip->dev, "%s compare failed\n", __func__);
		for (i = 0; i < size; i++)
			dev_info(chip->dev, "0x%x, 0x%x",
				data1[i], data2[i]);
		dev_info(chip->dev, "\n");
		return -EINVAL;
	}
	return 0;
}

static int max17042_init_model(struct max17042_chip *chip)
{
	int ret;
	int table_size = ARRAY_SIZE(chip->pdata->config_data->cell_char_tbl);
	u16 *temp_data;

	temp_data = kcalloc(table_size, sizeof(*temp_data), GFP_KERNEL);
	if (!temp_data)
		return -ENOMEM;

	max17042_unlock_model(chip);
	max17042_write_model_data(chip, MAX17042_MODELChrTbl,
				table_size);
	max17042_read_model_data(chip, MAX17042_MODELChrTbl, temp_data,
				table_size);

	ret = max17042_model_data_compare(
		chip,
		chip->pdata->config_data->cell_char_tbl,
		temp_data,
		table_size);

	max17042_lock_model(chip);
	kfree(temp_data);

	return ret;
}

static int max17042_verify_model_lock(struct max17042_chip *chip)
{
	int i;
	int table_size = ARRAY_SIZE(chip->pdata->config_data->cell_char_tbl);
	u16 *temp_data;
	int ret = 0;

	temp_data = kcalloc(table_size, sizeof(*temp_data), GFP_KERNEL);
	if (!temp_data)
		return -ENOMEM;

	max17042_read_model_data(chip, MAX17042_MODELChrTbl, temp_data,
				table_size);
	for (i = 0; i < table_size; i++)
		if (temp_data[i])
			ret = -EINVAL;

	kfree(temp_data);
	return ret;
}

static void max17042_write_config_regs(struct max17042_chip *chip)
{
	struct max17042_config_data *config = chip->pdata->config_data;
	struct regmap *map = chip->regmap;

	regmap_write(map, MAX17042_CONFIG, config->config);
	regmap_write(map, MAX17042_LearnCFG, config->learn_cfg);
	regmap_write(map, MAX17042_FilterCFG,
			config->filter_cfg);
	regmap_write(map, MAX17042_RelaxCFG, config->relax_cfg);
	if (chip->chip_type == MAXIM_DEVICE_TYPE_MAX17047 ||
			chip->chip_type == MAXIM_DEVICE_TYPE_MAX17050 ||
			chip->chip_type == MAXIM_DEVICE_TYPE_MAX17055)
		regmap_write(map, MAX17047_FullSOCThr,
						config->full_soc_thresh);
}

static void  max17042_write_custom_regs(struct max17042_chip *chip)
{
	struct max17042_config_data *config = chip->pdata->config_data;
	struct regmap *map = chip->regmap;

	max17042_write_verify_reg(map, MAX17042_RCOMP0, config->rcomp0);
	max17042_write_verify_reg(map, MAX17042_TempCo,	config->tcompc0);
	max17042_write_verify_reg(map, MAX17042_ICHGTerm, config->ichgt_term);
	if (chip->chip_type == MAXIM_DEVICE_TYPE_MAX17042) {
		regmap_write(map, MAX17042_EmptyTempCo,	config->empty_tempco);
		max17042_write_verify_reg(map, MAX17042_K_empty0,
					config->kempty0);
	} else {
		max17042_write_verify_reg(map, MAX17047_QRTbl00,
						config->qrtbl00);
		max17042_write_verify_reg(map, MAX17047_QRTbl10,
						config->qrtbl10);
		max17042_write_verify_reg(map, MAX17047_QRTbl20,
						config->qrtbl20);
		max17042_write_verify_reg(map, MAX17047_QRTbl30,
						config->qrtbl30);
	}
}

static void max17042_update_capacity_regs(struct max17042_chip *chip)
{
	struct max17042_config_data *config = chip->pdata->config_data;
	struct regmap *map = chip->regmap;

	max17042_write_verify_reg(map, MAX17042_FullCAP,
				config->fullcap);
	regmap_write(map, MAX17042_DesignCap, config->design_cap);
	max17042_write_verify_reg(map, MAX17042_FullCAPNom,
				config->fullcapnom);
}

static void max17042_reset_vfsoc0_reg(struct max17042_chip *chip)
{
	unsigned int vfSoc;
	struct regmap *map = chip->regmap;

	regmap_read(map, MAX17042_VFSOC, &vfSoc);
	regmap_write(map, MAX17042_VFSOC0Enable, VFSOC0_UNLOCK);
	max17042_write_verify_reg(map, MAX17042_VFSOC0, vfSoc);
	regmap_write(map, MAX17042_VFSOC0Enable, VFSOC0_LOCK);
}

static void max17042_load_new_capacity_params(struct max17042_chip *chip)
{
	u32 full_cap0, rep_cap, dq_acc, vfSoc;
	u32 rem_cap;

	struct max17042_config_data *config = chip->pdata->config_data;
	struct regmap *map = chip->regmap;

	regmap_read(map, MAX17042_FullCAP0, &full_cap0);
	regmap_read(map, MAX17042_VFSOC, &vfSoc);

	/* fg_vfSoc needs to shifted by 8 bits to get the
	 * perc in 1% accuracy, to get the right rem_cap multiply
	 * full_cap0, fg_vfSoc and devide by 100
	 */
	rem_cap = ((vfSoc >> 8) * full_cap0) / 100;
	max17042_write_verify_reg(map, MAX17042_RemCap, rem_cap);

	rep_cap = rem_cap;
	max17042_write_verify_reg(map, MAX17042_RepCap, rep_cap);

	/* Write dQ_acc to 200% of Capacity and dP_acc to 200% */
	dq_acc = config->fullcap / dQ_ACC_DIV;
	max17042_write_verify_reg(map, MAX17042_dQacc, dq_acc);
	max17042_write_verify_reg(map, MAX17042_dPacc, dP_ACC_200);

	max17042_write_verify_reg(map, MAX17042_FullCAP,
			config->fullcap);
	regmap_write(map, MAX17042_DesignCap,
			config->design_cap);
	max17042_write_verify_reg(map, MAX17042_FullCAPNom,
			config->fullcapnom);
	/* Update SOC register with new SOC */
	regmap_write(map, MAX17042_RepSOC, vfSoc);
}

/*
 * Block write all the override values coming from platform data.
 * This function MUST be called before the POR initialization procedure
 * specified by maxim.
 */
static inline void max17042_override_por_values(struct max17042_chip *chip)
{
	struct regmap *map = chip->regmap;
	struct max17042_config_data *config = chip->pdata->config_data;

	max17042_override_por(map, MAX17042_TGAIN, config->tgain);
	max17042_override_por(map, MAX17042_TOFF, config->toff);
	max17042_override_por(map, MAX17042_CGAIN, config->cgain);
	max17042_override_por(map, MAX17042_COFF, config->coff);

	max17042_override_por(map, MAX17042_VALRT_Th, config->valrt_thresh);
	max17042_override_por(map, MAX17042_TALRT_Th, config->talrt_thresh);
	max17042_override_por(map, MAX17042_SALRT_Th,
						config->soc_alrt_thresh);
	max17042_override_por(map, MAX17042_CONFIG, config->config);
	max17042_override_por(map, MAX17042_SHDNTIMER, config->shdntimer);

	max17042_override_por(map, MAX17042_DesignCap, config->design_cap);
	max17042_override_por(map, MAX17042_ICHGTerm, config->ichgt_term);

	max17042_override_por(map, MAX17042_AtRate, config->at_rate);
	max17042_override_por(map, MAX17042_LearnCFG, config->learn_cfg);
	max17042_override_por(map, MAX17042_FilterCFG, config->filter_cfg);
	max17042_override_por(map, MAX17042_RelaxCFG, config->relax_cfg);
	max17042_override_por(map, MAX17042_MiscCFG, config->misc_cfg);

	max17042_override_por(map, MAX17042_FullCAP, config->fullcap);
	max17042_override_por(map, MAX17042_FullCAPNom, config->fullcapnom);
	max17042_override_por(map, MAX17042_dQacc, config->dqacc);
	max17042_override_por(map, MAX17042_dPacc, config->dpacc);

	max17042_override_por(map, MAX17042_RCOMP0, config->rcomp0);
	max17042_override_por(map, MAX17042_TempCo, config->tcompc0);

	if (chip->chip_type == MAXIM_DEVICE_TYPE_MAX17042) {
		max17042_override_por(map, MAX17042_MaskSOC, config->masksoc);
		max17042_override_por(map, MAX17042_SOC_empty, config->socempty);
		max17042_override_por(map, MAX17042_V_empty, config->vempty);
		max17042_override_por(map, MAX17042_EmptyTempCo, config->empty_tempco);
		max17042_override_por(map, MAX17042_K_empty0, config->kempty0);
	}

	if ((chip->chip_type == MAXIM_DEVICE_TYPE_MAX17042) ||
	    (chip->chip_type == MAXIM_DEVICE_TYPE_MAX17047) ||
	    (chip->chip_type == MAXIM_DEVICE_TYPE_MAX17050)) {
		max17042_override_por(map, MAX17042_IAvg_empty, config->iavg_empty);
		max17042_override_por(map, MAX17042_TempNom, config->temp_nom);
		max17042_override_por(map, MAX17042_TempLim, config->temp_lim);
		max17042_override_por(map, MAX17042_FCTC, config->fctc);
	}

	if ((chip->chip_type == MAXIM_DEVICE_TYPE_MAX17047) ||
	    (chip->chip_type == MAXIM_DEVICE_TYPE_MAX17050) ||
	    (chip->chip_type == MAXIM_DEVICE_TYPE_MAX17055)) {
		max17042_override_por(map, MAX17047_V_empty, config->vempty);
	}
}

static int max17042_init_chip(struct max17042_chip *chip)
{
	struct regmap *map = chip->regmap;
	int ret;

	max17042_override_por_values(chip);
	/* After Power up, the MAX17042 requires 500mS in order
	 * to perform signal debouncing and initial SOC reporting
	 */
	msleep(500);

	/* Initialize configuration */
	max17042_write_config_regs(chip);

	/* write cell characterization data */
	ret = max17042_init_model(chip);
	if (ret) {
		dev_err(chip->dev, "%s init failed\n",
			__func__);
		return -EIO;
	}

	ret = max17042_verify_model_lock(chip);
	if (ret) {
		dev_err(chip->dev, "%s lock verify failed\n",
			__func__);
		return -EIO;
	}
	/* write custom parameters */
	max17042_write_custom_regs(chip);

	/* update capacity params */
	max17042_update_capacity_regs(chip);

	/* delay must be atleast 350mS to allow VFSOC
	 * to be calculated from the new configuration
	 */
	msleep(350);

	/* reset vfsoc0 reg */
	max17042_reset_vfsoc0_reg(chip);

	/* load new capacity params */
	max17042_load_new_capacity_params(chip);

	/* Init complete, Clear the POR bit */
	regmap_update_bits(map, MAX17042_STATUS, STATUS_POR_BIT, 0x0);
	return 0;
}

static void max17042_set_soc_threshold(struct max17042_chip *chip, u16 off)
{
	struct regmap *map = chip->regmap;
	u32 soc, soc_tr;

	/* program interrupt thresholds such that we should
	 * get interrupt for every 'off' perc change in the soc
	 */
	if (chip->pdata->enable_current_sense)
		regmap_read(map, MAX17042_RepSOC, &soc);
	else
		regmap_read(map, MAX17042_VFSOC, &soc);
	soc >>= 8;
	soc_tr = (soc + off) << 8;
	if (off < soc)
		soc_tr |= soc - off;
	regmap_write(map, MAX17042_SALRT_Th, soc_tr);
}

static irqreturn_t max17042_thread_handler(int id, void *dev)
{
	struct max17042_chip *chip = dev;
	u32 val;
	int ret;

	ret = regmap_read(chip->regmap, MAX17042_STATUS, &val);
	if (ret)
		return IRQ_HANDLED;

	if ((val & STATUS_SMN_BIT) || (val & STATUS_SMX_BIT)) {
		dev_dbg(chip->dev, "SOC threshold INTR\n");
		max17042_set_soc_threshold(chip, 1);
	}

	/* we implicitly handle all alerts via power_supply_changed */
	regmap_clear_bits(chip->regmap, MAX17042_STATUS,
			  0xFFFF & ~(STATUS_POR_BIT | STATUS_BST_BIT));

	power_supply_changed(chip->battery);
	return IRQ_HANDLED;
}

static void max17042_init_worker(struct work_struct *work)
{
	struct max17042_chip *chip = container_of(work,
				struct max17042_chip, work);
	int ret;

	/* Initialize registers according to values from the platform data */
	if (chip->pdata->enable_por_init && chip->pdata->config_data) {
		ret = max17042_init_chip(chip);
		if (ret)
			return;
	}

	chip->init_complete = 1;
}

/*
 * MAX77865: the battery NTC (thermal zone of samsung,battery-ntc) written to TEMP every
 * 10 s, as stock sec_battery pushed its ADC temperature through set_property(TEMP)
 * (S:drivers/battery_v2/sec_battery.c:2686-2689, max77865_fuelgauge.c:1776-1803),
 * spec L6F 5.4 items 2, 2a. A failed reading keeps the last value (sec_adc.c:267-269).
 */
static void max17042_temp_worker(struct work_struct *work)
{
	struct max17042_chip *chip = container_of(to_delayed_work(work),
						  struct max17042_chip, temp_work);
	struct regmap *map = chip->regmap;
	struct thermal_zone_device *tz;
	int t, d;
	u32 q, val;

	tz = thermal_zone_get_zone_by_name(chip->tz_name);
	if (!IS_ERR(tz) && !max77865_ntc_avg(tz, &t)) {
		d = t / 100;	/* 0.1 degC */
		/*
		 * two's complement 1/256 degC, rounded away from zero so that
		 * max17042_get_temperature() reads d back exactly (L6F 5.4 item 2)
		 */
		q = DIV_ROUND_UP(abs(d) * 256, 10);
		regmap_write(map, MAX17042_TEMP, (u16)(d < 0 ? -(s32)q : (s32)q));

		/* FilterCFG below 0 / above 3.0 degC (max77865_fuelgauge.c:1787-1799,
		 * FDT:12349-12350)
		 */
		if (d < 0 && !chip->low_temp_wa) {
			chip->low_temp_wa = true;
			regmap_write(map, MAX17042_FilterCFG, 0xce97);
		} else if (d > 30 && chip->low_temp_wa) {
			chip->low_temp_wa = false;
			regmap_write(map, MAX17042_FilterCFG, 0xce94);
		}

		/* max77865_fg_check_qrtable() (max77865_fuelgauge.c:1334-1352, FDT:12347-12348) */
		if (!regmap_read(map, MAX17047_QRTbl20, &val) && val != 0x0f82)
			regmap_write(map, MAX17047_QRTbl20, 0x0f82);
		if (!regmap_read(map, MAX17047_QRTbl30, &val) && val != 0x0b04)
			regmap_write(map, MAX17047_QRTbl30, 0x0b04);
	}

	/* the zone's own polling period (L6F 4.4) */
	queue_delayed_work(system_freezable_wq, &chip->temp_work, 10 * HZ);
}

/*
 * MAX77865: the capacity and full-charge policy of Samsung's max77865_fuelgauge.c and
 * sec_battery.c (drivers/battery_v2), run every 30 s (battery,polling_time) and on every
 * supply change. The gauge's RepSOC is scaled by capacity_max (get_scaled_capacity()) and
 * moves one percent per pass (get_atomic_capacity(), capacity_calculation_type 0x1c). An end
 * of charge from the charger with the cell at full_condition_vcell counts as FULL only at a
 * scaled 100 %; below that, capacity_max is lowered to what the gauge reads
 * (calculate_dynamic_scale(), from sec_bat_fullcharged_check()), so the level climbs to 100 %
 * and then reads FULL. A FULL battery is charging again once it is below full_condition_soc
 * and recharge_condition_vcell - 50 mV (sec_bat_voltage_check()).
 *
 * Not ported: the extra capacity_max correction in constant voltage between top-off and
 * top-off + 300 mA (it needs the charger's CV state), and the software V-empty mode (it needs
 * the gauge's ALRT interrupt, which this board does not wire).
 */
static void max77865_dynamic_scale(struct max17042_chip *chip, int raw, int capacity)
{
	int lo = chip->capacity_max_cfg - chip->capacity_margin;

	if (raw < lo)
		return;

	chip->capacity_max = clamp_t(int, raw * 100 / (capacity + 1), lo,
				     chip->capacity_max_cfg);
	chip->capacity_old = capacity;
}

static void max77865_soc_work(struct work_struct *work)
{
	struct max17042_chip *chip = container_of(to_delayed_work(work),
						  struct max17042_chip, soc_work);
	union power_supply_propval v;
	bool supplied, chg_full, full, changed;
	int raw, soc, cap, vcell_mv, iavg;
	u32 data;

	mutex_lock(&chip->soc_lock);
	full = chip->full;
	cap = chip->capacity;

	if (regmap_read(chip->regmap, MAX17042_RepSOC, &data))
		goto rearm;
	raw = min_t(int, data * 100 / 256 / 10, 1000);
	if (regmap_read(chip->regmap, MAX17042_VCELL, &data))
		goto rearm;
	vcell_mv = data * 625 / 8 / 1000;
	if (regmap_read(chip->regmap, MAX17042_AvgCurrent, &data))
		goto rearm;
	iavg = sign_extend32(data, 15);

	/* max77865_get_fuelgauge_soc() */
	soc = (vcell_mv < 3400 && iavg < 0 && raw <= 10) ? 0 : raw;

	supplied = power_supply_am_i_supplied(chip->battery) > 0;

	/* max77865_fg_get_scaled_capacity() */
	soc = soc < chip->capacity_min ? 0 :
	      (soc - chip->capacity_min) * 1000 / (chip->capacity_max - chip->capacity_min);
	soc = clamp(soc, 0, 1000);
	WRITE_ONCE(chip->soc_permille, soc);
	cap = soc / 10;

	/* max77865_fg_get_atomic_capacity(); a fresh start after probe and resume */
	if (chip->initial_update) {
		chip->initial_update = false;
	} else {
		if (chip->capacity_old < cap)
			cap = chip->capacity_old + 1;
		else if (chip->capacity_old > cap)
			cap = chip->capacity_old - 1;
		if (!supplied && chip->capacity_old < cap)
			cap = chip->capacity_old;
	}
	chip->capacity_old = cap;

	chg_full = false;
	if (supplied && !power_supply_get_property_from_supplier(chip->battery,
								  POWER_SUPPLY_PROP_STATUS, &v)) {
		if (v.intval == POWER_SUPPLY_STATUS_FULL)
			chg_full = true;
		else if (v.intval != POWER_SUPPLY_STATUS_CHARGING)
			full = false;
	} else {
		full = false;
	}

	if (full && cap < chip->full_soc && vcell_mv < chip->rechg_vcell_mv - 50) {
		full = false;
		dev_info(chip->dev, "full -> charging: capacity %d%%, %d mV\n", cap, vcell_mv);
	} else if (!full && chg_full && vcell_mv >= chip->full_vcell_mv) {
		int old_max = chip->capacity_max;

		max77865_dynamic_scale(chip, raw, cap);
		if (cap >= 100)
			full = true;
		if (full || chip->capacity_max != old_max)
			dev_info(chip->dev,
				 "end of charge at %d%% (gauge %d.%d%%, %d mV): capacity_max %d%s\n",
				 cap, raw / 10, raw % 10, vcell_mv, chip->capacity_max,
				 full ? ", full" : "");
	}

rearm:
	changed = cap != chip->capacity || full != chip->full;
	WRITE_ONCE(chip->capacity, cap);
	WRITE_ONCE(chip->full, full);
	if (chip->soc_ready)
		queue_delayed_work(system_freezable_wq, &chip->soc_work, 30 * HZ);
	mutex_unlock(&chip->soc_lock);

	if (changed)
		power_supply_changed(chip->battery);
}

static void max77865_external_power_changed(struct power_supply *psy)
{
	struct max17042_chip *chip = power_supply_get_drvdata(psy);

	mutex_lock(&chip->soc_lock);
	if (power_supply_am_i_supplied(psy) <= 0)
		WRITE_ONCE(chip->full, false);
	if (chip->soc_ready)
		mod_delayed_work(system_freezable_wq, &chip->soc_work, 0);
	mutex_unlock(&chip->soc_lock);
	power_supply_changed(psy);
}

static void max77865_soc_stop(void *data)
{
	struct max17042_chip *chip = data;

	mutex_lock(&chip->soc_lock);
	chip->soc_ready = false;
	mutex_unlock(&chip->soc_lock);
	cancel_delayed_work_sync(&chip->soc_work);
}

static int max77865_soc_parse(struct max17042_chip *chip)
{
	struct device_node *np = chip->dev->of_node;
	u32 max, margin, min, full_uv, full_soc, rechg_uv;

	if (of_property_read_u32(np, "samsung,capacity-max", &max) ||
	    of_property_read_u32(np, "samsung,capacity-max-margin", &margin) ||
	    of_property_read_u32(np, "samsung,capacity-min", &min) ||
	    of_property_read_u32(np, "samsung,full-condition-microvolt", &full_uv) ||
	    of_property_read_u32(np, "samsung,full-condition-percent", &full_soc) ||
	    of_property_read_u32(np, "samsung,recharge-condition-microvolt", &rechg_uv))
		return dev_err_probe(chip->dev, -EINVAL, "capacity policy properties missing\n");
	if (max > 1000 || full_soc > 100 || margin >= max || min >= max - margin ||
	    full_uv > 5000000 || rechg_uv > 5000000)
		return dev_err_probe(chip->dev, -EINVAL, "capacity policy limits out of range\n");

	chip->capacity_max_cfg = max;
	chip->capacity_margin = margin;
	chip->capacity_min = min;
	chip->full_soc = full_soc;
	chip->full_vcell_mv = full_uv / 1000;
	chip->rechg_vcell_mv = rechg_uv / 1000;
	chip->capacity_max = chip->capacity_max_cfg;
	chip->initial_update = true;
	mutex_init(&chip->soc_lock);
	INIT_DELAYED_WORK(&chip->soc_work, max77865_soc_work);
	return 0;
}

#ifdef CONFIG_OF
static struct max17042_platform_data *
max17042_get_of_pdata(struct max17042_chip *chip)
{
	struct device *dev = chip->dev;
	struct device_node *np = dev->of_node;
	u32 prop;
	struct max17042_platform_data *pdata;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return NULL;

	/*
	 * Require current sense resistor value to be specified for
	 * current-sense functionality to be enabled at all.
	 */
	if (of_property_read_u32(np, "maxim,rsns-microohm", &prop) == 0) {
		pdata->r_sns = prop;
		pdata->enable_current_sense = true;
	}

	if (of_property_read_s32(np, "maxim,cold-temp", &pdata->temp_min))
		pdata->temp_min = INT_MIN;
	if (of_property_read_s32(np, "maxim,over-heat-temp", &pdata->temp_max))
		pdata->temp_max = INT_MAX;
	if (of_property_read_s32(np, "maxim,dead-volt", &pdata->vmin))
		pdata->vmin = INT_MIN;
	if (of_property_read_s32(np, "maxim,over-volt", &pdata->vmax))
		pdata->vmax = INT_MAX;

	return pdata;
}
#endif

static struct max17042_reg_data max17047_default_pdata_init_regs[] = {
	/*
	 * Some firmwares do not set FullSOCThr, Enable End-of-Charge Detection
	 * when the voltage FG reports 95%, as recommended in the datasheet.
	 */
	{ MAX17047_FullSOCThr, MAX17042_BATTERY_FULL << 8 },
};

static struct max17042_platform_data *
max17042_get_default_pdata(struct max17042_chip *chip)
{
	struct device *dev = chip->dev;
	struct max17042_platform_data *pdata;
	int ret, misc_cfg;

	/*
	 * The MAX17047 gets used on x86 where we might not have pdata, assume
	 * the firmware will already have initialized the fuel-gauge and provide
	 * default values for the non init bits to make things work.
	 */
	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return pdata;

	if ((chip->chip_type == MAXIM_DEVICE_TYPE_MAX17047) ||
	    (chip->chip_type == MAXIM_DEVICE_TYPE_MAX17050)) {
		pdata->init_data = max17047_default_pdata_init_regs;
		pdata->num_init_data =
			ARRAY_SIZE(max17047_default_pdata_init_regs);
	}

	ret = regmap_read(chip->regmap, MAX17042_MiscCFG, &misc_cfg);
	if (ret < 0)
		return NULL;

	/* If bits 0-1 are set to 3 then only Voltage readings are used */
	if ((misc_cfg & 0x3) == 0x3)
		pdata->enable_current_sense = false;
	else
		pdata->enable_current_sense = true;

	pdata->vmin = MAX17042_DEFAULT_VMIN;
	pdata->vmax = MAX17042_DEFAULT_VMAX;
	pdata->temp_min = MAX17042_DEFAULT_TEMP_MIN;
	pdata->temp_max = MAX17042_DEFAULT_TEMP_MAX;

	return pdata;
}

static struct max17042_platform_data *
max17042_get_pdata(struct max17042_chip *chip)
{
	struct device *dev = chip->dev;

#ifdef CONFIG_OF
	if (dev->of_node)
		return max17042_get_of_pdata(chip);
#endif
	if (dev->platform_data)
		return dev->platform_data;

	return max17042_get_default_pdata(chip);
}

static const struct regmap_config max17042_regmap_config = {
	.name = "max17042",
	.reg_bits = 8,
	.val_bits = 16,
	.val_format_endian = REGMAP_ENDIAN_NATIVE,
};

static const struct power_supply_desc max17042_psy_desc = {
	.name		= "max170xx_battery",
	.type		= POWER_SUPPLY_TYPE_BATTERY,
	.get_property	= max17042_get_property,
	.set_property	= max17042_set_property,
	.property_is_writeable	= max17042_property_is_writeable,
	.external_power_changed	= power_supply_changed,
	.properties	= max17042_battery_props,
	.num_properties	= ARRAY_SIZE(max17042_battery_props),
};

static const struct power_supply_desc max17042_no_current_sense_psy_desc = {
	.name		= "max170xx_battery",
	.type		= POWER_SUPPLY_TYPE_BATTERY,
	.get_property	= max17042_get_property,
	.set_property	= max17042_set_property,
	.property_is_writeable	= max17042_property_is_writeable,
	.properties	= max17042_battery_props,
	.num_properties	= ARRAY_SIZE(max17042_battery_props) - 2,
};

static int max17042_probe(struct i2c_client *client, struct device *dev, int irq,
			  enum max170xx_chip_type chip_type)
{
	struct i2c_adapter *adapter = client->adapter;
	const struct power_supply_desc *max17042_desc = &max17042_psy_desc;
	struct power_supply_config psy_cfg = {};
	struct max17042_chip *chip;
	int ret;
	int i;
	u32 val;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA))
		return -EIO;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = dev;
	chip->chip_type = chip_type;
	chip->regmap = devm_regmap_init_i2c(client, &max17042_regmap_config);
	if (IS_ERR(chip->regmap)) {
		dev_err(dev, "Failed to initialize regmap\n");
		return -EINVAL;
	}

	chip->host_temp = of_device_is_compatible(dev->of_node, "maxim,max77865-battery");
	if (chip->host_temp) {
		ret = of_property_read_string(dev->of_node, "samsung,battery-ntc",
					      &chip->tz_name);
		if (ret)
			return dev_err_probe(dev, ret, "no samsung,battery-ntc\n");
		/*
		 * CONFIG.TEX: temperature written by the host, as stock does when the
		 * thermal source is not the FG (S:drivers/battery_v2/max77865_fuelgauge.c:
		 * 1005-1018; battery,thermal_source 2 = ADC FDT:12182).
		 */
		regmap_update_bits(chip->regmap, MAX17042_CONFIG, BIT(8), BIT(8));
		if (!regmap_read(chip->regmap, MAX17042_CONFIG, &val))
			dev_info(dev, "CONFIG=0x%04x TEX=%d (host temperature)\n",
				 val, !!(val & BIT(8)));

		ret = max77865_soc_parse(chip);
		if (ret)
			return ret;
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_TTF)
		{
			int count = of_property_count_u32_elems(dev->of_node, "maxim,ttf-cv-data");

			if (count > 0 && count % 3 == 0) {
				chip->cv_data = devm_kcalloc(dev, count / 3,
							     sizeof(struct cv_slope),
							     GFP_KERNEL);
				if (chip->cv_data &&
				    !of_property_read_u32_array(dev->of_node, "maxim,ttf-cv-data",
								(u32 *)chip->cv_data, count))
					chip->cv_data_length = count / 3;
			}
		}
#endif
	}

	chip->pdata = max17042_get_pdata(chip);
	if (!chip->pdata) {
		dev_err(dev, "no platform data provided\n");
		return -EINVAL;
	}

	dev_set_drvdata(dev, chip);
	psy_cfg.drv_data = chip;
	psy_cfg.fwnode = dev_fwnode(dev);

	/* When current is not measured,
	 * CURRENT_NOW and CURRENT_AVG properties should be invisible. */
	if (!chip->pdata->enable_current_sense)
		max17042_desc = &max17042_no_current_sense_psy_desc;

	if (chip->pdata->r_sns == 0)
		chip->pdata->r_sns = MAX17042_DEFAULT_SNS_RESISTOR;

	if (chip->pdata->init_data)
		for (i = 0; i < chip->pdata->num_init_data; i++)
			regmap_write(chip->regmap,
					chip->pdata->init_data[i].addr,
					chip->pdata->init_data[i].data);

	if (!chip->pdata->enable_current_sense) {
		regmap_write(chip->regmap, MAX17042_CGAIN, 0x0000);
		regmap_write(chip->regmap, MAX17042_MiscCFG, 0x0003);
		regmap_write(chip->regmap, MAX17042_LearnCFG, 0x0007);
	}

	/*
	 * MAX77865: Android's battery supply keeps the stock name "battery"
	 * (battery/temp and battery/voltage_now blob readers, spec L6F 5.4)
	 */
	if (chip->host_temp) {
		chip->psy_desc = *max17042_desc;
		chip->psy_desc.name = "battery";
		chip->psy_desc.external_power_changed = max77865_external_power_changed;
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_TTF)
		chip->psy_desc.properties = max77865_battery_props;
		chip->psy_desc.num_properties = max17042_desc->num_properties + 1;
#endif
		max17042_desc = &chip->psy_desc;
	}

	chip->battery = devm_power_supply_register(dev, max17042_desc,
						   &psy_cfg);
	if (IS_ERR(chip->battery)) {
		dev_err(dev, "failed: power supply register\n");
		return PTR_ERR(chip->battery);
	}
	if (chip->host_temp) {
		dev_info(dev, "registered as battery (max77865)\n");

		ret = devm_add_action_or_reset(dev, max77865_soc_stop, chip);
		if (ret)
			return ret;
		mutex_lock(&chip->soc_lock);
		chip->soc_ready = true;
		mod_delayed_work(system_freezable_wq, &chip->soc_work, 0);
		mutex_unlock(&chip->soc_lock);
		/* the level must exist before anyone reads it */
		flush_delayed_work(&chip->soc_work);
	}

	if (irq) {
		unsigned int flags = IRQF_ONESHOT | IRQF_SHARED | IRQF_PROBE_SHARED;

		ret = devm_request_threaded_irq(dev, irq,
						NULL,
						max17042_thread_handler, flags,
						chip->battery->desc->name,
						chip);
		if (!ret) {
			regmap_update_bits(chip->regmap, MAX17042_CONFIG,
					CFG_ALRT_BIT_ENBL,
					CFG_ALRT_BIT_ENBL);
			max17042_set_soc_threshold(chip, 1);
		} else {
			irq = 0;
			if (ret != -EBUSY)
				dev_err(dev, "Failed to get IRQ\n");
		}
	}
	/* Not able to update the charge threshold when exceeded? -> disable */
	if (!irq)
		regmap_write(chip->regmap, MAX17042_SALRT_Th, 0xff00);

	chip->irq = irq;

	regmap_read(chip->regmap, MAX17042_STATUS, &val);
	if (val & STATUS_POR_BIT) {
		ret = devm_work_autocancel(dev, &chip->work,
					   max17042_init_worker);
		if (ret)
			return ret;
		schedule_work(&chip->work);
	} else {
		chip->init_complete = 1;
	}

	if (chip->host_temp) {
		ret = devm_delayed_work_autocancel(dev, &chip->temp_work,
						   max17042_temp_worker);
		if (ret)
			return ret;
		queue_delayed_work(system_freezable_wq, &chip->temp_work, 0);
	}

	return 0;
}

static int max17042_i2c_probe(struct i2c_client *client)
{
	const struct i2c_device_id *id = i2c_client_get_device_id(client);
	const struct acpi_device_id *acpi_id = NULL;
	struct device *dev = &client->dev;
	enum max170xx_chip_type chip_type;

	if (id) {
		chip_type = id->driver_data;
	} else {
		acpi_id = acpi_match_device(dev->driver->acpi_match_table, dev);
		if (!acpi_id)
			return -ENODEV;

		chip_type = acpi_id->driver_data;
	}

	return max17042_probe(client, dev, client->irq, chip_type);
}

static int max17042_platform_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct i2c_client *i2c;
	const struct platform_device_id *id;
	int irq;

	i2c = to_i2c_client(pdev->dev.parent);
	if (!i2c)
		return -EINVAL;

	dev->of_node = dev->parent->of_node;
	id = platform_get_device_id(pdev);
	irq = platform_get_irq(pdev, 0);

	return max17042_probe(i2c, dev, irq, id->driver_data);
}

#ifdef CONFIG_PM_SLEEP
static int max17042_suspend(struct device *dev)
{
	struct max17042_chip *chip = dev_get_drvdata(dev);

	/*
	 * disable the irq and enable irq_wake
	 * capability to the interrupt line.
	 */
	if (chip->irq) {
		disable_irq(chip->irq);
		enable_irq_wake(chip->irq);
	}

	return 0;
}

static int max17042_resume(struct device *dev)
{
	struct max17042_chip *chip = dev_get_drvdata(dev);

	if (chip->irq) {
		disable_irq_wake(chip->irq);
		enable_irq(chip->irq);
		/* re-program the SOC thresholds to 1% change */
		max17042_set_soc_threshold(chip, 1);
	}

	if (chip->host_temp) {
		/* max77865_fuelgauge_resume(): the level after a sleep is taken as read */
		mutex_lock(&chip->soc_lock);
		chip->initial_update = true;
		if (chip->soc_ready)
			mod_delayed_work(system_freezable_wq, &chip->soc_work, 0);
		mutex_unlock(&chip->soc_lock);
	}

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(max17042_pm_ops, max17042_suspend,
			max17042_resume);

#ifdef CONFIG_ACPI
static const struct acpi_device_id max17042_acpi_match[] = {
	{ "MAX17047", MAXIM_DEVICE_TYPE_MAX17047 },
	{ }
};
MODULE_DEVICE_TABLE(acpi, max17042_acpi_match);
#endif

#ifdef CONFIG_OF
/*
 * Device may be instantiated through parent MFD device and device matching is done
 * through platform_device_id.
 *
 * However if device's DT node contains proper clock compatible and driver is
 * built as a module, then the *module* matching will be done trough DT aliases.
 * This requires of_device_id table.  In the same time this will not change the
 * actual *device* matching so do not add .of_match_table.
 */
static const struct of_device_id max17042_dt_match[] __used = {
	{ .compatible = "maxim,max17042",
		.data = (void *) MAXIM_DEVICE_TYPE_MAX17042 },
	{ .compatible = "maxim,max17047",
		.data = (void *) MAXIM_DEVICE_TYPE_MAX17047 },
	{ .compatible = "maxim,max17050",
		.data = (void *) MAXIM_DEVICE_TYPE_MAX17050 },
	{ .compatible = "maxim,max17055",
		.data = (void *) MAXIM_DEVICE_TYPE_MAX17055 },
	{ .compatible = "maxim,max77705-battery",
		.data = (void *) MAXIM_DEVICE_TYPE_MAX17047 },
	{ .compatible = "maxim,max77849-battery",
		.data = (void *) MAXIM_DEVICE_TYPE_MAX17047 },
	{ .compatible = "maxim,max77865-battery",
		.data = (void *) MAXIM_DEVICE_TYPE_MAX17047 },
	{ },
};
MODULE_DEVICE_TABLE(of, max17042_dt_match);
#endif

static const struct i2c_device_id max17042_id[] = {
	{ "max17042", MAXIM_DEVICE_TYPE_MAX17042 },
	{ "max17047", MAXIM_DEVICE_TYPE_MAX17047 },
	{ "max17050", MAXIM_DEVICE_TYPE_MAX17050 },
	{ "max17055", MAXIM_DEVICE_TYPE_MAX17055 },
	{ "max77849-battery", MAXIM_DEVICE_TYPE_MAX17047 },
	{ "max77865-battery", MAXIM_DEVICE_TYPE_MAX17047 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max17042_id);

static const struct platform_device_id max17042_platform_id[] = {
	{ "max17042", MAXIM_DEVICE_TYPE_MAX17042 },
	{ "max17047", MAXIM_DEVICE_TYPE_MAX17047 },
	{ "max17050", MAXIM_DEVICE_TYPE_MAX17050 },
	{ "max17055", MAXIM_DEVICE_TYPE_MAX17055 },
	{ "max77705-battery", MAXIM_DEVICE_TYPE_MAX17047 },
	{ "max77849-battery", MAXIM_DEVICE_TYPE_MAX17047 },
	{ }
};
MODULE_DEVICE_TABLE(platform, max17042_platform_id);

static struct i2c_driver max17042_i2c_driver = {
	.driver	= {
		.name	= "max17042",
		.acpi_match_table = ACPI_PTR(max17042_acpi_match),
		.of_match_table = of_match_ptr(max17042_dt_match),
		.pm	= &max17042_pm_ops,
	},
	.probe		= max17042_i2c_probe,
	.id_table	= max17042_id,
};

static struct platform_driver max17042_platform_driver = {
	.driver	= {
		.name	= "max17042",
		.acpi_match_table = ACPI_PTR(max17042_acpi_match),
		.pm	= &max17042_pm_ops,
	},
	.probe		= max17042_platform_probe,
	.id_table	= max17042_platform_id,
};

static int __init max17042_init(void)
{
	int ret;

	ret = platform_driver_register(&max17042_platform_driver);
	if (ret)
		return ret;

	ret = i2c_add_driver(&max17042_i2c_driver);
	if (ret) {
		platform_driver_unregister(&max17042_platform_driver);
		return ret;
	}

	return 0;
}
module_init(max17042_init);

static void __exit max17042_exit(void)
{
	i2c_del_driver(&max17042_i2c_driver);
	platform_driver_unregister(&max17042_platform_driver);
}
module_exit(max17042_exit);

MODULE_AUTHOR("MyungJoo Ham <myungjoo.ham@samsung.com>");
MODULE_DESCRIPTION("MAX17042 Fuel Gauge");
MODULE_LICENSE("GPL");
