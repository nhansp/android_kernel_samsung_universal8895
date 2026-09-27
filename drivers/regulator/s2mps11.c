// SPDX-License-Identifier: GPL-2.0+
//
// Copyright (c) 2012-2014 Samsung Electronics Co., Ltd
//              http://www.samsung.com

#include <linux/bug.h>
#include <linux/cleanup.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/platform_device.h>
#include <linux/regulator/coupler.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/mfd/samsung/core.h>
#include <linux/mfd/samsung/s2mps11.h>
#include <linux/mfd/samsung/s2mps13.h>
#include <linux/mfd/samsung/s2mps14.h>
#include <linux/mfd/samsung/s2mps15.h>
#include <linux/mfd/samsung/s2mps17.h>
#include <linux/mfd/samsung/s2mpu02.h>
#include <linux/mfd/samsung/s2mpu05.h>

/* The highest number of possible regulators for supported devices. */
#define S2MPS_REGULATOR_MAX		S2MPS13_REGULATOR_MAX
struct s2mps11_info {
	int ramp_delay2;
	int ramp_delay34;
	int ramp_delay5;
	int ramp_delay16;
	int ramp_delay7810;
	int ramp_delay9;

	enum sec_device_type dev_type;

	/*
	 * One bit for each S2MPS11/S2MPS13/S2MPS14/S2MPU02 regulator whether
	 * the suspend mode was enabled.
	 */
	DECLARE_BITMAP(suspend_state, S2MPS_REGULATOR_MAX);

	/*
	 * Array (size: number of regulators) with GPIO-s for external
	 * sleep control.
	 */
	struct gpio_desc **ext_control_gpiod;

	/* S2MPS17: operating mode written by enable(), per regulator id */
	unsigned int opmode[S2MPS_REGULATOR_MAX];
};

static int get_ramp_delay(int ramp_delay)
{
	unsigned char cnt = 0;

	ramp_delay /= 6250;

	while (true) {
		ramp_delay = ramp_delay >> 1;
		if (ramp_delay == 0)
			break;
		cnt++;
	}

	if (cnt > 3)
		cnt = 3;

	return cnt;
}

static int s2mps11_regulator_set_voltage_time_sel(struct regulator_dev *rdev,
				   unsigned int old_selector,
				   unsigned int new_selector)
{
	struct s2mps11_info *s2mps11 = rdev_get_drvdata(rdev);
	int rdev_id = rdev_get_id(rdev);
	unsigned int ramp_delay = 0;
	int old_volt, new_volt;

	switch (rdev_id) {
	case S2MPS11_BUCK2:
		ramp_delay = s2mps11->ramp_delay2;
		break;
	case S2MPS11_BUCK3:
	case S2MPS11_BUCK4:
		ramp_delay = s2mps11->ramp_delay34;
		break;
	case S2MPS11_BUCK5:
		ramp_delay = s2mps11->ramp_delay5;
		break;
	case S2MPS11_BUCK6:
	case S2MPS11_BUCK1:
		ramp_delay = s2mps11->ramp_delay16;
		break;
	case S2MPS11_BUCK7:
	case S2MPS11_BUCK8:
	case S2MPS11_BUCK10:
		ramp_delay = s2mps11->ramp_delay7810;
		break;
	case S2MPS11_BUCK9:
		ramp_delay = s2mps11->ramp_delay9;
	}

	if (ramp_delay == 0)
		ramp_delay = rdev->desc->ramp_delay;

	old_volt = rdev->desc->min_uV + (rdev->desc->uV_step * old_selector);
	new_volt = rdev->desc->min_uV + (rdev->desc->uV_step * new_selector);

	return DIV_ROUND_UP(abs(new_volt - old_volt), ramp_delay);
}

static int s2mps11_set_ramp_delay(struct regulator_dev *rdev, int ramp_delay)
{
	struct s2mps11_info *s2mps11 = rdev_get_drvdata(rdev);
	unsigned int ramp_val, ramp_shift, ramp_reg = S2MPS11_REG_RAMP_BUCK;
	unsigned int ramp_enable = 1, enable_shift = 0;
	int rdev_id = rdev_get_id(rdev);
	int ret;

	switch (rdev_id) {
	case S2MPS11_BUCK1:
		if (ramp_delay > s2mps11->ramp_delay16)
			s2mps11->ramp_delay16 = ramp_delay;
		else
			ramp_delay = s2mps11->ramp_delay16;

		ramp_shift = S2MPS11_BUCK16_RAMP_SHIFT;
		break;
	case S2MPS11_BUCK2:
		enable_shift = S2MPS11_BUCK2_RAMP_EN_SHIFT;
		if (!ramp_delay) {
			ramp_enable = 0;
			break;
		}

		s2mps11->ramp_delay2 = ramp_delay;
		ramp_shift = S2MPS11_BUCK2_RAMP_SHIFT;
		ramp_reg = S2MPS11_REG_RAMP;
		break;
	case S2MPS11_BUCK3:
		enable_shift = S2MPS11_BUCK3_RAMP_EN_SHIFT;
		if (!ramp_delay) {
			ramp_enable = 0;
			break;
		}

		if (ramp_delay > s2mps11->ramp_delay34)
			s2mps11->ramp_delay34 = ramp_delay;
		else
			ramp_delay = s2mps11->ramp_delay34;

		ramp_shift = S2MPS11_BUCK34_RAMP_SHIFT;
		ramp_reg = S2MPS11_REG_RAMP;
		break;
	case S2MPS11_BUCK4:
		enable_shift = S2MPS11_BUCK4_RAMP_EN_SHIFT;
		if (!ramp_delay) {
			ramp_enable = 0;
			break;
		}

		if (ramp_delay > s2mps11->ramp_delay34)
			s2mps11->ramp_delay34 = ramp_delay;
		else
			ramp_delay = s2mps11->ramp_delay34;

		ramp_shift = S2MPS11_BUCK34_RAMP_SHIFT;
		ramp_reg = S2MPS11_REG_RAMP;
		break;
	case S2MPS11_BUCK5:
		s2mps11->ramp_delay5 = ramp_delay;
		ramp_shift = S2MPS11_BUCK5_RAMP_SHIFT;
		break;
	case S2MPS11_BUCK6:
		enable_shift = S2MPS11_BUCK6_RAMP_EN_SHIFT;
		if (!ramp_delay) {
			ramp_enable = 0;
			break;
		}

		if (ramp_delay > s2mps11->ramp_delay16)
			s2mps11->ramp_delay16 = ramp_delay;
		else
			ramp_delay = s2mps11->ramp_delay16;

		ramp_shift = S2MPS11_BUCK16_RAMP_SHIFT;
		break;
	case S2MPS11_BUCK7:
	case S2MPS11_BUCK8:
	case S2MPS11_BUCK10:
		if (ramp_delay > s2mps11->ramp_delay7810)
			s2mps11->ramp_delay7810 = ramp_delay;
		else
			ramp_delay = s2mps11->ramp_delay7810;

		ramp_shift = S2MPS11_BUCK7810_RAMP_SHIFT;
		break;
	case S2MPS11_BUCK9:
		s2mps11->ramp_delay9 = ramp_delay;
		ramp_shift = S2MPS11_BUCK9_RAMP_SHIFT;
		break;
	default:
		return 0;
	}

	if (!ramp_enable)
		goto ramp_disable;

	/* Ramp delay can be enabled/disabled only for buck[2346] */
	if ((rdev_id >= S2MPS11_BUCK2 && rdev_id <= S2MPS11_BUCK4) ||
	    rdev_id == S2MPS11_BUCK6)  {
		ret = regmap_update_bits(rdev->regmap, S2MPS11_REG_RAMP,
					 1 << enable_shift, 1 << enable_shift);
		if (ret) {
			dev_err(&rdev->dev, "failed to enable ramp rate\n");
			return ret;
		}
	}

	ramp_val = get_ramp_delay(ramp_delay);

	return regmap_update_bits(rdev->regmap, ramp_reg, 0x3 << ramp_shift,
				  ramp_val << ramp_shift);

ramp_disable:
	return regmap_update_bits(rdev->regmap, S2MPS11_REG_RAMP,
				  1 << enable_shift, 0);
}

static int s2mps11_regulator_enable(struct regulator_dev *rdev)
{
	struct s2mps11_info *s2mps11 = rdev_get_drvdata(rdev);
	int rdev_id = rdev_get_id(rdev);
	unsigned int val;

	switch (s2mps11->dev_type) {
	case S2MPS11X:
		if (test_bit(rdev_id, s2mps11->suspend_state))
			val = S2MPS14_ENABLE_SUSPEND;
		else
			val = rdev->desc->enable_mask;
		break;
	case S2MPS13X:
	case S2MPS14X:
		if (test_bit(rdev_id, s2mps11->suspend_state))
			val = S2MPS14_ENABLE_SUSPEND;
		else if (s2mps11->ext_control_gpiod[rdev_id])
			val = S2MPS14_ENABLE_EXT_CONTROL;
		else
			val = rdev->desc->enable_mask;
		break;
	case S2MPU02:
		if (test_bit(rdev_id, s2mps11->suspend_state))
			val = S2MPU02_ENABLE_SUSPEND;
		else
			val = rdev->desc->enable_mask;
		break;
	case S2MPU05:
		val = rdev->desc->enable_mask;
		break;
	default:
		return -EINVAL;
	}

	return regmap_update_bits(rdev->regmap, rdev->desc->enable_reg,
			rdev->desc->enable_mask, val);
}

static int s2mps11_regulator_set_suspend_disable(struct regulator_dev *rdev)
{
	int ret;
	unsigned int val, state;
	struct s2mps11_info *s2mps11 = rdev_get_drvdata(rdev);
	int rdev_id = rdev_get_id(rdev);

	/* Below LDO should be always on or does not support suspend mode. */
	switch (s2mps11->dev_type) {
	case S2MPS11X:
		switch (rdev_id) {
		case S2MPS11_LDO2:
		case S2MPS11_LDO36:
		case S2MPS11_LDO37:
		case S2MPS11_LDO38:
			return 0;
		default:
			state = S2MPS14_ENABLE_SUSPEND;
			break;
		}
		break;
	case S2MPS13X:
	case S2MPS14X:
		switch (rdev_id) {
		case S2MPS14_LDO3:
			return 0;
		default:
			state = S2MPS14_ENABLE_SUSPEND;
			break;
		}
		break;
	case S2MPU02:
		switch (rdev_id) {
		case S2MPU02_LDO13:
		case S2MPU02_LDO14:
		case S2MPU02_LDO15:
		case S2MPU02_LDO17:
		case S2MPU02_BUCK7:
			state = S2MPU02_DISABLE_SUSPEND;
			break;
		default:
			state = S2MPU02_ENABLE_SUSPEND;
			break;
		}
		break;
	default:
		return -EINVAL;
	}

	ret = regmap_read(rdev->regmap, rdev->desc->enable_reg, &val);
	if (ret < 0)
		return ret;

	set_bit(rdev_id, s2mps11->suspend_state);
	/*
	 * Don't enable suspend mode if regulator is already disabled because
	 * this would effectively for a short time turn on the regulator after
	 * resuming.
	 * However we still want to toggle the suspend_state bit for regulator
	 * in case if it got enabled before suspending the system.
	 */
	if (!(val & rdev->desc->enable_mask))
		return 0;

	return regmap_update_bits(rdev->regmap, rdev->desc->enable_reg,
				  rdev->desc->enable_mask, state);
}

static const struct regulator_ops s2mps11_ldo_ops = {
	.list_voltage		= regulator_list_voltage_linear,
	.map_voltage		= regulator_map_voltage_linear,
	.is_enabled		= regulator_is_enabled_regmap,
	.enable			= s2mps11_regulator_enable,
	.disable		= regulator_disable_regmap,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_voltage_sel	= regulator_set_voltage_sel_regmap,
	.set_voltage_time_sel	= regulator_set_voltage_time_sel,
	.set_suspend_disable	= s2mps11_regulator_set_suspend_disable,
};

static const struct regulator_ops s2mps11_buck_ops = {
	.list_voltage		= regulator_list_voltage_linear,
	.map_voltage		= regulator_map_voltage_linear,
	.is_enabled		= regulator_is_enabled_regmap,
	.enable			= s2mps11_regulator_enable,
	.disable		= regulator_disable_regmap,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_voltage_sel	= regulator_set_voltage_sel_regmap,
	.set_voltage_time_sel	= s2mps11_regulator_set_voltage_time_sel,
	.set_ramp_delay		= s2mps11_set_ramp_delay,
	.set_suspend_disable	= s2mps11_regulator_set_suspend_disable,
};

#define regulator_desc_s2mps11_ldo(num, step) {		\
	.name		= "LDO"#num,			\
	.id		= S2MPS11_LDO##num,		\
	.ops		= &s2mps11_ldo_ops,		\
	.type		= REGULATOR_VOLTAGE,		\
	.owner		= THIS_MODULE,			\
	.ramp_delay	= RAMP_DELAY_12_MVUS,		\
	.min_uV		= MIN_800_MV,			\
	.uV_step	= step,				\
	.n_voltages	= S2MPS11_LDO_N_VOLTAGES,	\
	.vsel_reg	= S2MPS11_REG_L1CTRL + num - 1,	\
	.vsel_mask	= S2MPS11_LDO_VSEL_MASK,	\
	.enable_reg	= S2MPS11_REG_L1CTRL + num - 1,	\
	.enable_mask	= S2MPS11_ENABLE_MASK		\
}

#define regulator_desc_s2mps11_buck1_4(num) {			\
	.name		= "BUCK"#num,				\
	.id		= S2MPS11_BUCK##num,			\
	.ops		= &s2mps11_buck_ops,			\
	.type		= REGULATOR_VOLTAGE,			\
	.owner		= THIS_MODULE,				\
	.min_uV		= MIN_650_MV,				\
	.uV_step	= STEP_6_25_MV,				\
	.linear_min_sel	= 8,					\
	.n_voltages	= S2MPS11_BUCK12346_N_VOLTAGES,		\
	.ramp_delay	= S2MPS11_RAMP_DELAY,			\
	.vsel_reg	= S2MPS11_REG_B1CTRL2 + (num - 1) * 2,	\
	.vsel_mask	= S2MPS11_BUCK_VSEL_MASK,		\
	.enable_reg	= S2MPS11_REG_B1CTRL1 + (num - 1) * 2,	\
	.enable_mask	= S2MPS11_ENABLE_MASK			\
}

#define regulator_desc_s2mps11_buck5 {				\
	.name		= "BUCK5",				\
	.id		= S2MPS11_BUCK5,			\
	.ops		= &s2mps11_buck_ops,			\
	.type		= REGULATOR_VOLTAGE,			\
	.owner		= THIS_MODULE,				\
	.min_uV		= MIN_650_MV,				\
	.uV_step	= STEP_6_25_MV,				\
	.linear_min_sel	= 8,					\
	.n_voltages	= S2MPS11_BUCK5_N_VOLTAGES,		\
	.ramp_delay	= S2MPS11_RAMP_DELAY,			\
	.vsel_reg	= S2MPS11_REG_B5CTRL2,			\
	.vsel_mask	= S2MPS11_BUCK_VSEL_MASK,		\
	.enable_reg	= S2MPS11_REG_B5CTRL1,			\
	.enable_mask	= S2MPS11_ENABLE_MASK			\
}

#define regulator_desc_s2mps11_buck67810(num, min, step, min_sel, voltages) {	\
	.name		= "BUCK"#num,				\
	.id		= S2MPS11_BUCK##num,			\
	.ops		= &s2mps11_buck_ops,			\
	.type		= REGULATOR_VOLTAGE,			\
	.owner		= THIS_MODULE,				\
	.min_uV		= min,					\
	.uV_step	= step,					\
	.linear_min_sel	= min_sel,				\
	.n_voltages	= voltages,				\
	.ramp_delay	= S2MPS11_RAMP_DELAY,			\
	.vsel_reg	= S2MPS11_REG_B6CTRL2 + (num - 6) * 2,	\
	.vsel_mask	= S2MPS11_BUCK_VSEL_MASK,		\
	.enable_reg	= S2MPS11_REG_B6CTRL1 + (num - 6) * 2,	\
	.enable_mask	= S2MPS11_ENABLE_MASK			\
}

#define regulator_desc_s2mps11_buck9 {				\
	.name		= "BUCK9",				\
	.id		= S2MPS11_BUCK9,			\
	.ops		= &s2mps11_buck_ops,			\
	.type		= REGULATOR_VOLTAGE,			\
	.owner		= THIS_MODULE,				\
	.min_uV		= MIN_3000_MV,				\
	.uV_step	= STEP_25_MV,				\
	.n_voltages	= S2MPS11_BUCK9_N_VOLTAGES,		\
	.ramp_delay	= S2MPS11_RAMP_DELAY,			\
	.vsel_reg	= S2MPS11_REG_B9CTRL2,			\
	.vsel_mask	= S2MPS11_BUCK9_VSEL_MASK,		\
	.enable_reg	= S2MPS11_REG_B9CTRL1,			\
	.enable_mask	= S2MPS11_ENABLE_MASK			\
}

static const struct regulator_desc s2mps11_regulators[] = {
	regulator_desc_s2mps11_ldo(1, STEP_25_MV),
	regulator_desc_s2mps11_ldo(2, STEP_50_MV),
	regulator_desc_s2mps11_ldo(3, STEP_50_MV),
	regulator_desc_s2mps11_ldo(4, STEP_50_MV),
	regulator_desc_s2mps11_ldo(5, STEP_50_MV),
	regulator_desc_s2mps11_ldo(6, STEP_25_MV),
	regulator_desc_s2mps11_ldo(7, STEP_50_MV),
	regulator_desc_s2mps11_ldo(8, STEP_50_MV),
	regulator_desc_s2mps11_ldo(9, STEP_50_MV),
	regulator_desc_s2mps11_ldo(10, STEP_50_MV),
	regulator_desc_s2mps11_ldo(11, STEP_25_MV),
	regulator_desc_s2mps11_ldo(12, STEP_50_MV),
	regulator_desc_s2mps11_ldo(13, STEP_50_MV),
	regulator_desc_s2mps11_ldo(14, STEP_50_MV),
	regulator_desc_s2mps11_ldo(15, STEP_50_MV),
	regulator_desc_s2mps11_ldo(16, STEP_50_MV),
	regulator_desc_s2mps11_ldo(17, STEP_50_MV),
	regulator_desc_s2mps11_ldo(18, STEP_50_MV),
	regulator_desc_s2mps11_ldo(19, STEP_50_MV),
	regulator_desc_s2mps11_ldo(20, STEP_50_MV),
	regulator_desc_s2mps11_ldo(21, STEP_50_MV),
	regulator_desc_s2mps11_ldo(22, STEP_25_MV),
	regulator_desc_s2mps11_ldo(23, STEP_25_MV),
	regulator_desc_s2mps11_ldo(24, STEP_50_MV),
	regulator_desc_s2mps11_ldo(25, STEP_50_MV),
	regulator_desc_s2mps11_ldo(26, STEP_50_MV),
	regulator_desc_s2mps11_ldo(27, STEP_25_MV),
	regulator_desc_s2mps11_ldo(28, STEP_50_MV),
	regulator_desc_s2mps11_ldo(29, STEP_50_MV),
	regulator_desc_s2mps11_ldo(30, STEP_50_MV),
	regulator_desc_s2mps11_ldo(31, STEP_50_MV),
	regulator_desc_s2mps11_ldo(32, STEP_50_MV),
	regulator_desc_s2mps11_ldo(33, STEP_50_MV),
	regulator_desc_s2mps11_ldo(34, STEP_50_MV),
	regulator_desc_s2mps11_ldo(35, STEP_25_MV),
	regulator_desc_s2mps11_ldo(36, STEP_50_MV),
	regulator_desc_s2mps11_ldo(37, STEP_50_MV),
	regulator_desc_s2mps11_ldo(38, STEP_50_MV),
	regulator_desc_s2mps11_buck1_4(1),
	regulator_desc_s2mps11_buck1_4(2),
	regulator_desc_s2mps11_buck1_4(3),
	regulator_desc_s2mps11_buck1_4(4),
	regulator_desc_s2mps11_buck5,
	regulator_desc_s2mps11_buck67810(6, MIN_650_MV, STEP_6_25_MV, 8,
					 S2MPS11_BUCK12346_N_VOLTAGES),
	regulator_desc_s2mps11_buck67810(7, MIN_750_MV, STEP_12_5_MV, 0,
					 S2MPS11_BUCK7810_N_VOLTAGES),
	regulator_desc_s2mps11_buck67810(8, MIN_750_MV, STEP_12_5_MV, 0,
					 S2MPS11_BUCK7810_N_VOLTAGES),
	regulator_desc_s2mps11_buck9,
	regulator_desc_s2mps11_buck67810(10, MIN_750_MV, STEP_12_5_MV, 0,
					 S2MPS11_BUCK7810_N_VOLTAGES),
};

static const struct regulator_ops s2mps14_reg_ops;

#define regulator_desc_s2mps13_ldo(num, min, step, min_sel) {	\
	.name		= "LDO"#num,				\
	.id		= S2MPS13_LDO##num,			\
	.ops		= &s2mps14_reg_ops,			\
	.type		= REGULATOR_VOLTAGE,			\
	.owner		= THIS_MODULE,				\
	.min_uV		= min,					\
	.uV_step	= step,					\
	.linear_min_sel	= min_sel,				\
	.n_voltages	= S2MPS14_LDO_N_VOLTAGES,		\
	.vsel_reg	= S2MPS13_REG_L1CTRL + num - 1,		\
	.vsel_mask	= S2MPS14_LDO_VSEL_MASK,		\
	.enable_reg	= S2MPS13_REG_L1CTRL + num - 1,		\
	.enable_mask	= S2MPS14_ENABLE_MASK			\
}

#define regulator_desc_s2mps13_buck(num, min, step, min_sel) {	\
	.name		= "BUCK"#num,				\
	.id		= S2MPS13_BUCK##num,			\
	.ops		= &s2mps14_reg_ops,			\
	.type		= REGULATOR_VOLTAGE,			\
	.owner		= THIS_MODULE,				\
	.min_uV		= min,					\
	.uV_step	= step,					\
	.linear_min_sel	= min_sel,				\
	.n_voltages	= S2MPS14_BUCK_N_VOLTAGES,		\
	.ramp_delay	= S2MPS13_BUCK_RAMP_DELAY,		\
	.vsel_reg	= S2MPS13_REG_B1OUT + (num - 1) * 2,	\
	.vsel_mask	= S2MPS14_BUCK_VSEL_MASK,		\
	.enable_reg	= S2MPS13_REG_B1CTRL + (num - 1) * 2,	\
	.enable_mask	= S2MPS14_ENABLE_MASK			\
}

#define regulator_desc_s2mps13_buck7(num, min, step, min_sel) {	\
	.name		= "BUCK"#num,				\
	.id		= S2MPS13_BUCK##num,			\
	.ops		= &s2mps14_reg_ops,			\
	.type		= REGULATOR_VOLTAGE,			\
	.owner		= THIS_MODULE,				\
	.min_uV		= min,					\
	.uV_step	= step,					\
	.linear_min_sel	= min_sel,				\
	.n_voltages	= S2MPS14_BUCK_N_VOLTAGES,		\
	.ramp_delay	= S2MPS13_BUCK_RAMP_DELAY,		\
	.vsel_reg	= S2MPS13_REG_B1OUT + (num) * 2 - 1,	\
	.vsel_mask	= S2MPS14_BUCK_VSEL_MASK,		\
	.enable_reg	= S2MPS13_REG_B1CTRL + (num - 1) * 2,	\
	.enable_mask	= S2MPS14_ENABLE_MASK			\
}

#define regulator_desc_s2mps13_buck8_10(num, min, step, min_sel) {	\
	.name		= "BUCK"#num,				\
	.id		= S2MPS13_BUCK##num,			\
	.ops		= &s2mps14_reg_ops,			\
	.type		= REGULATOR_VOLTAGE,			\
	.owner		= THIS_MODULE,				\
	.min_uV		= min,					\
	.uV_step	= step,					\
	.linear_min_sel	= min_sel,				\
	.n_voltages	= S2MPS14_BUCK_N_VOLTAGES,		\
	.ramp_delay	= S2MPS13_BUCK_RAMP_DELAY,		\
	.vsel_reg	= S2MPS13_REG_B1OUT + (num) * 2 - 1,	\
	.vsel_mask	= S2MPS14_BUCK_VSEL_MASK,		\
	.enable_reg	= S2MPS13_REG_B1CTRL + (num) * 2 - 1,	\
	.enable_mask	= S2MPS14_ENABLE_MASK			\
}

static const struct regulator_desc s2mps13_regulators[] = {
	regulator_desc_s2mps13_ldo(1,  MIN_800_MV,  STEP_12_5_MV, 0x00),
	regulator_desc_s2mps13_ldo(2,  MIN_1400_MV, STEP_50_MV,   0x0C),
	regulator_desc_s2mps13_ldo(3,  MIN_1000_MV, STEP_25_MV,   0x08),
	regulator_desc_s2mps13_ldo(4,  MIN_800_MV,  STEP_12_5_MV, 0x00),
	regulator_desc_s2mps13_ldo(5,  MIN_800_MV,  STEP_12_5_MV, 0x00),
	regulator_desc_s2mps13_ldo(6,  MIN_800_MV,  STEP_12_5_MV, 0x00),
	regulator_desc_s2mps13_ldo(7,  MIN_1000_MV, STEP_25_MV,   0x08),
	regulator_desc_s2mps13_ldo(8,  MIN_1000_MV, STEP_25_MV,   0x08),
	regulator_desc_s2mps13_ldo(9,  MIN_1000_MV, STEP_25_MV,   0x08),
	regulator_desc_s2mps13_ldo(10, MIN_1400_MV, STEP_50_MV,   0x0C),
	regulator_desc_s2mps13_ldo(11, MIN_800_MV,  STEP_25_MV,   0x10),
	regulator_desc_s2mps13_ldo(12, MIN_800_MV,  STEP_25_MV,   0x10),
	regulator_desc_s2mps13_ldo(13, MIN_800_MV,  STEP_25_MV,   0x10),
	regulator_desc_s2mps13_ldo(14, MIN_800_MV,  STEP_12_5_MV, 0x00),
	regulator_desc_s2mps13_ldo(15, MIN_800_MV,  STEP_12_5_MV, 0x00),
	regulator_desc_s2mps13_ldo(16, MIN_1400_MV, STEP_50_MV,   0x0C),
	regulator_desc_s2mps13_ldo(17, MIN_1400_MV, STEP_50_MV,   0x0C),
	regulator_desc_s2mps13_ldo(18, MIN_1000_MV, STEP_25_MV,   0x08),
	regulator_desc_s2mps13_ldo(19, MIN_1000_MV, STEP_25_MV,   0x08),
	regulator_desc_s2mps13_ldo(20, MIN_1400_MV, STEP_50_MV,   0x0C),
	regulator_desc_s2mps13_ldo(21, MIN_1000_MV, STEP_25_MV,   0x08),
	regulator_desc_s2mps13_ldo(22, MIN_1000_MV, STEP_25_MV,   0x08),
	regulator_desc_s2mps13_ldo(23, MIN_800_MV,  STEP_12_5_MV, 0x00),
	regulator_desc_s2mps13_ldo(24, MIN_800_MV,  STEP_12_5_MV, 0x00),
	regulator_desc_s2mps13_ldo(25, MIN_1400_MV, STEP_50_MV,   0x0C),
	regulator_desc_s2mps13_ldo(26, MIN_1400_MV, STEP_50_MV,   0x0C),
	regulator_desc_s2mps13_ldo(27, MIN_1400_MV, STEP_50_MV,   0x0C),
	regulator_desc_s2mps13_ldo(28, MIN_1000_MV, STEP_25_MV,   0x08),
	regulator_desc_s2mps13_ldo(29, MIN_1400_MV, STEP_50_MV,   0x0C),
	regulator_desc_s2mps13_ldo(30, MIN_1400_MV, STEP_50_MV,   0x0C),
	regulator_desc_s2mps13_ldo(31, MIN_1000_MV, STEP_25_MV,   0x08),
	regulator_desc_s2mps13_ldo(32, MIN_1000_MV, STEP_25_MV,   0x08),
	regulator_desc_s2mps13_ldo(33, MIN_1400_MV, STEP_50_MV,   0x0C),
	regulator_desc_s2mps13_ldo(34, MIN_1000_MV, STEP_25_MV,   0x08),
	regulator_desc_s2mps13_ldo(35, MIN_1400_MV, STEP_50_MV,   0x0C),
	regulator_desc_s2mps13_ldo(36, MIN_800_MV,  STEP_12_5_MV, 0x00),
	regulator_desc_s2mps13_ldo(37, MIN_1000_MV, STEP_25_MV,   0x08),
	regulator_desc_s2mps13_ldo(38, MIN_1400_MV, STEP_50_MV,   0x0C),
	regulator_desc_s2mps13_ldo(39, MIN_1000_MV, STEP_25_MV,   0x08),
	regulator_desc_s2mps13_ldo(40, MIN_1400_MV, STEP_50_MV,   0x0C),
	regulator_desc_s2mps13_buck(1,  MIN_500_MV,  STEP_6_25_MV, 0x10),
	regulator_desc_s2mps13_buck(2,  MIN_500_MV,  STEP_6_25_MV, 0x10),
	regulator_desc_s2mps13_buck(3,  MIN_500_MV,  STEP_6_25_MV, 0x10),
	regulator_desc_s2mps13_buck(4,  MIN_500_MV,  STEP_6_25_MV, 0x10),
	regulator_desc_s2mps13_buck(5,  MIN_500_MV,  STEP_6_25_MV, 0x10),
	regulator_desc_s2mps13_buck(6,  MIN_500_MV,  STEP_6_25_MV, 0x10),
	regulator_desc_s2mps13_buck7(7,  MIN_500_MV,  STEP_6_25_MV, 0x10),
	regulator_desc_s2mps13_buck8_10(8,  MIN_1000_MV, STEP_12_5_MV, 0x20),
	regulator_desc_s2mps13_buck8_10(9,  MIN_1000_MV, STEP_12_5_MV, 0x20),
	regulator_desc_s2mps13_buck8_10(10, MIN_500_MV,  STEP_6_25_MV, 0x10),
};

static const struct regulator_ops s2mps14_reg_ops = {
	.list_voltage		= regulator_list_voltage_linear,
	.map_voltage		= regulator_map_voltage_linear,
	.is_enabled		= regulator_is_enabled_regmap,
	.enable			= s2mps11_regulator_enable,
	.disable		= regulator_disable_regmap,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_voltage_sel	= regulator_set_voltage_sel_regmap,
	.set_voltage_time_sel	= regulator_set_voltage_time_sel,
	.set_suspend_disable	= s2mps11_regulator_set_suspend_disable,
};

#define regulator_desc_s2mps14_ldo(num, min, step) {	\
	.name		= "LDO"#num,			\
	.id		= S2MPS14_LDO##num,		\
	.ops		= &s2mps14_reg_ops,		\
	.type		= REGULATOR_VOLTAGE,		\
	.owner		= THIS_MODULE,			\
	.min_uV		= min,				\
	.uV_step	= step,				\
	.n_voltages	= S2MPS14_LDO_N_VOLTAGES,	\
	.vsel_reg	= S2MPS14_REG_L1CTRL + num - 1,	\
	.vsel_mask	= S2MPS14_LDO_VSEL_MASK,	\
	.enable_reg	= S2MPS14_REG_L1CTRL + num - 1,	\
	.enable_mask	= S2MPS14_ENABLE_MASK		\
}

#define regulator_desc_s2mps14_buck(num, min, step, min_sel) {	\
	.name		= "BUCK"#num,				\
	.id		= S2MPS14_BUCK##num,			\
	.ops		= &s2mps14_reg_ops,			\
	.type		= REGULATOR_VOLTAGE,			\
	.owner		= THIS_MODULE,				\
	.min_uV		= min,					\
	.uV_step	= step,					\
	.n_voltages	= S2MPS14_BUCK_N_VOLTAGES,		\
	.linear_min_sel = min_sel,				\
	.ramp_delay	= S2MPS14_BUCK_RAMP_DELAY,		\
	.vsel_reg	= S2MPS14_REG_B1CTRL2 + (num - 1) * 2,	\
	.vsel_mask	= S2MPS14_BUCK_VSEL_MASK,		\
	.enable_reg	= S2MPS14_REG_B1CTRL1 + (num - 1) * 2,	\
	.enable_mask	= S2MPS14_ENABLE_MASK			\
}

static const struct regulator_desc s2mps14_regulators[] = {
	regulator_desc_s2mps14_ldo(1, MIN_800_MV, STEP_12_5_MV),
	regulator_desc_s2mps14_ldo(2, MIN_800_MV, STEP_12_5_MV),
	regulator_desc_s2mps14_ldo(3, MIN_800_MV, STEP_25_MV),
	regulator_desc_s2mps14_ldo(4, MIN_800_MV, STEP_25_MV),
	regulator_desc_s2mps14_ldo(5, MIN_800_MV, STEP_12_5_MV),
	regulator_desc_s2mps14_ldo(6, MIN_800_MV, STEP_12_5_MV),
	regulator_desc_s2mps14_ldo(7, MIN_800_MV, STEP_25_MV),
	regulator_desc_s2mps14_ldo(8, MIN_1800_MV, STEP_25_MV),
	regulator_desc_s2mps14_ldo(9, MIN_800_MV, STEP_12_5_MV),
	regulator_desc_s2mps14_ldo(10, MIN_800_MV, STEP_12_5_MV),
	regulator_desc_s2mps14_ldo(11, MIN_800_MV, STEP_25_MV),
	regulator_desc_s2mps14_ldo(12, MIN_1800_MV, STEP_25_MV),
	regulator_desc_s2mps14_ldo(13, MIN_1800_MV, STEP_25_MV),
	regulator_desc_s2mps14_ldo(14, MIN_1800_MV, STEP_25_MV),
	regulator_desc_s2mps14_ldo(15, MIN_1800_MV, STEP_25_MV),
	regulator_desc_s2mps14_ldo(16, MIN_1800_MV, STEP_25_MV),
	regulator_desc_s2mps14_ldo(17, MIN_1800_MV, STEP_25_MV),
	regulator_desc_s2mps14_ldo(18, MIN_1800_MV, STEP_25_MV),
	regulator_desc_s2mps14_ldo(19, MIN_800_MV, STEP_25_MV),
	regulator_desc_s2mps14_ldo(20, MIN_800_MV, STEP_25_MV),
	regulator_desc_s2mps14_ldo(21, MIN_800_MV, STEP_25_MV),
	regulator_desc_s2mps14_ldo(22, MIN_800_MV, STEP_12_5_MV),
	regulator_desc_s2mps14_ldo(23, MIN_800_MV, STEP_25_MV),
	regulator_desc_s2mps14_ldo(24, MIN_1800_MV, STEP_25_MV),
	regulator_desc_s2mps14_ldo(25, MIN_1800_MV, STEP_25_MV),
	regulator_desc_s2mps14_buck(1, MIN_600_MV, STEP_6_25_MV,
				    S2MPS14_BUCK1235_START_SEL),
	regulator_desc_s2mps14_buck(2, MIN_600_MV, STEP_6_25_MV,
				    S2MPS14_BUCK1235_START_SEL),
	regulator_desc_s2mps14_buck(3, MIN_600_MV, STEP_6_25_MV,
				    S2MPS14_BUCK1235_START_SEL),
	regulator_desc_s2mps14_buck(4, MIN_1400_MV, STEP_12_5_MV,
				    S2MPS14_BUCK4_START_SEL),
	regulator_desc_s2mps14_buck(5, MIN_600_MV, STEP_6_25_MV,
				    S2MPS14_BUCK1235_START_SEL),
};

static const struct regulator_ops s2mps15_reg_ldo_ops = {
	.list_voltage		= regulator_list_voltage_linear_range,
	.map_voltage		= regulator_map_voltage_linear_range,
	.is_enabled		= regulator_is_enabled_regmap,
	.enable			= regulator_enable_regmap,
	.disable		= regulator_disable_regmap,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_voltage_sel	= regulator_set_voltage_sel_regmap,
};

static const struct regulator_ops s2mps15_reg_buck_ops = {
	.list_voltage		= regulator_list_voltage_linear_range,
	.map_voltage		= regulator_map_voltage_linear_range,
	.is_enabled		= regulator_is_enabled_regmap,
	.enable			= regulator_enable_regmap,
	.disable		= regulator_disable_regmap,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_voltage_sel	= regulator_set_voltage_sel_regmap,
	.set_voltage_time_sel	= regulator_set_voltage_time_sel,
};

#define regulator_desc_s2mps15_ldo(num, range) {	\
	.name		= "LDO"#num,			\
	.id		= S2MPS15_LDO##num,		\
	.ops		= &s2mps15_reg_ldo_ops,		\
	.type		= REGULATOR_VOLTAGE,		\
	.owner		= THIS_MODULE,			\
	.linear_ranges	= range,			\
	.n_linear_ranges = ARRAY_SIZE(range),		\
	.n_voltages	= S2MPS15_LDO_N_VOLTAGES,	\
	.vsel_reg	= S2MPS15_REG_L1CTRL + num - 1,	\
	.vsel_mask	= S2MPS15_LDO_VSEL_MASK,	\
	.enable_reg	= S2MPS15_REG_L1CTRL + num - 1,	\
	.enable_mask	= S2MPS15_ENABLE_MASK		\
}

#define regulator_desc_s2mps15_buck(num, range) {			\
	.name		= "BUCK"#num,					\
	.id		= S2MPS15_BUCK##num,				\
	.ops		= &s2mps15_reg_buck_ops,			\
	.type		= REGULATOR_VOLTAGE,				\
	.owner		= THIS_MODULE,					\
	.linear_ranges	= range,					\
	.n_linear_ranges = ARRAY_SIZE(range),				\
	.ramp_delay	= 12500,					\
	.n_voltages	= S2MPS15_BUCK_N_VOLTAGES,			\
	.vsel_reg	= S2MPS15_REG_B1CTRL2 + ((num - 1) * 2),	\
	.vsel_mask	= S2MPS15_BUCK_VSEL_MASK,			\
	.enable_reg	= S2MPS15_REG_B1CTRL1 + ((num - 1) * 2),	\
	.enable_mask	= S2MPS15_ENABLE_MASK				\
}

/* voltage range for s2mps15 LDO 3, 5, 15, 16, 18, 20, 23 and 27 */
static const struct linear_range s2mps15_ldo_voltage_ranges1[] = {
	REGULATOR_LINEAR_RANGE(1000000, 0xc, 0x38, 25000),
};

/* voltage range for s2mps15 LDO 2, 6, 14, 17, 19, 21, 24 and 25 */
static const struct linear_range s2mps15_ldo_voltage_ranges2[] = {
	REGULATOR_LINEAR_RANGE(1800000, 0x0, 0x3f, 25000),
};

/* voltage range for s2mps15 LDO 4, 11, 12, 13, 22 and 26 */
static const struct linear_range s2mps15_ldo_voltage_ranges3[] = {
	REGULATOR_LINEAR_RANGE(700000, 0x0, 0x34, 12500),
};

/* voltage range for s2mps15 LDO 7, 8, 9 and 10 */
static const struct linear_range s2mps15_ldo_voltage_ranges4[] = {
	REGULATOR_LINEAR_RANGE(700000, 0x10, 0x20, 25000),
};

/* voltage range for s2mps15 LDO 1 */
static const struct linear_range s2mps15_ldo_voltage_ranges5[] = {
	REGULATOR_LINEAR_RANGE(500000, 0x0, 0x20, 12500),
};

/* voltage range for s2mps15 BUCK 1, 2, 3, 4, 5, 6 and 7 */
static const struct linear_range s2mps15_buck_voltage_ranges1[] = {
	REGULATOR_LINEAR_RANGE(500000, 0x20, 0xc0, 6250),
};

/* voltage range for s2mps15 BUCK 8, 9 and 10 */
static const struct linear_range s2mps15_buck_voltage_ranges2[] = {
	REGULATOR_LINEAR_RANGE(1000000, 0x20, 0x78, 12500),
};

static const struct regulator_desc s2mps15_regulators[] = {
	regulator_desc_s2mps15_ldo(1, s2mps15_ldo_voltage_ranges5),
	regulator_desc_s2mps15_ldo(2, s2mps15_ldo_voltage_ranges2),
	regulator_desc_s2mps15_ldo(3, s2mps15_ldo_voltage_ranges1),
	regulator_desc_s2mps15_ldo(4, s2mps15_ldo_voltage_ranges3),
	regulator_desc_s2mps15_ldo(5, s2mps15_ldo_voltage_ranges1),
	regulator_desc_s2mps15_ldo(6, s2mps15_ldo_voltage_ranges2),
	regulator_desc_s2mps15_ldo(7, s2mps15_ldo_voltage_ranges4),
	regulator_desc_s2mps15_ldo(8, s2mps15_ldo_voltage_ranges4),
	regulator_desc_s2mps15_ldo(9, s2mps15_ldo_voltage_ranges4),
	regulator_desc_s2mps15_ldo(10, s2mps15_ldo_voltage_ranges4),
	regulator_desc_s2mps15_ldo(11, s2mps15_ldo_voltage_ranges3),
	regulator_desc_s2mps15_ldo(12, s2mps15_ldo_voltage_ranges3),
	regulator_desc_s2mps15_ldo(13, s2mps15_ldo_voltage_ranges3),
	regulator_desc_s2mps15_ldo(14, s2mps15_ldo_voltage_ranges2),
	regulator_desc_s2mps15_ldo(15, s2mps15_ldo_voltage_ranges1),
	regulator_desc_s2mps15_ldo(16, s2mps15_ldo_voltage_ranges1),
	regulator_desc_s2mps15_ldo(17, s2mps15_ldo_voltage_ranges2),
	regulator_desc_s2mps15_ldo(18, s2mps15_ldo_voltage_ranges1),
	regulator_desc_s2mps15_ldo(19, s2mps15_ldo_voltage_ranges2),
	regulator_desc_s2mps15_ldo(20, s2mps15_ldo_voltage_ranges1),
	regulator_desc_s2mps15_ldo(21, s2mps15_ldo_voltage_ranges2),
	regulator_desc_s2mps15_ldo(22, s2mps15_ldo_voltage_ranges3),
	regulator_desc_s2mps15_ldo(23, s2mps15_ldo_voltage_ranges1),
	regulator_desc_s2mps15_ldo(24, s2mps15_ldo_voltage_ranges2),
	regulator_desc_s2mps15_ldo(25, s2mps15_ldo_voltage_ranges2),
	regulator_desc_s2mps15_ldo(26, s2mps15_ldo_voltage_ranges3),
	regulator_desc_s2mps15_ldo(27, s2mps15_ldo_voltage_ranges1),
	regulator_desc_s2mps15_buck(1, s2mps15_buck_voltage_ranges1),
	regulator_desc_s2mps15_buck(2, s2mps15_buck_voltage_ranges1),
	regulator_desc_s2mps15_buck(3, s2mps15_buck_voltage_ranges1),
	regulator_desc_s2mps15_buck(4, s2mps15_buck_voltage_ranges1),
	regulator_desc_s2mps15_buck(5, s2mps15_buck_voltage_ranges1),
	regulator_desc_s2mps15_buck(6, s2mps15_buck_voltage_ranges1),
	regulator_desc_s2mps15_buck(7, s2mps15_buck_voltage_ranges1),
	regulator_desc_s2mps15_buck(8, s2mps15_buck_voltage_ranges2),
	regulator_desc_s2mps15_buck(9, s2mps15_buck_voltage_ranges2),
	regulator_desc_s2mps15_buck(10, s2mps15_buck_voltage_ranges2),
};

static int s2mps14_pmic_enable_ext_control(struct s2mps11_info *s2mps11,
		struct regulator_dev *rdev)
{
	return regmap_update_bits(rdev->regmap, rdev->desc->enable_reg,
			rdev->desc->enable_mask, S2MPS14_ENABLE_EXT_CONTROL);
}

static void s2mps14_pmic_dt_parse_ext_control_gpio(struct platform_device *pdev,
		struct of_regulator_match *rdata, struct s2mps11_info *s2mps11)
{
	struct gpio_desc **gpio = s2mps11->ext_control_gpiod;
	unsigned int i;
	unsigned int valid_regulators[3] = { S2MPS14_LDO10, S2MPS14_LDO11,
		S2MPS14_LDO12 };

	for (i = 0; i < ARRAY_SIZE(valid_regulators); i++) {
		unsigned int reg = valid_regulators[i];

		if (!rdata[reg].init_data || !rdata[reg].of_node)
			continue;

		gpio[reg] = devm_fwnode_gpiod_get(&pdev->dev,
				of_fwnode_handle(rdata[reg].of_node),
				"samsung,ext-control",
				GPIOD_OUT_HIGH | GPIOD_FLAGS_BIT_NONEXCLUSIVE,
				"s2mps11-regulator");
		if (PTR_ERR(gpio[reg]) == -ENOENT)
			gpio[reg] = NULL;
		else if (IS_ERR(gpio[reg])) {
			dev_err(&pdev->dev, "Failed to get control GPIO for %d/%s\n",
				reg, rdata[reg].name);
			gpio[reg] = NULL;
			continue;
		}
		if (gpio[reg])
			dev_dbg(&pdev->dev, "Using GPIO for ext-control over %d/%s\n",
				reg, rdata[reg].name);
	}
}

static int s2mps11_pmic_dt_parse(struct platform_device *pdev,
		struct of_regulator_match *rdata, struct s2mps11_info *s2mps11,
		unsigned int rdev_num)
{
	struct device_node *reg_np;

	reg_np = of_get_child_by_name(pdev->dev.parent->of_node, "regulators");
	if (!reg_np) {
		dev_err(&pdev->dev, "could not find regulators sub-node\n");
		return -EINVAL;
	}

	of_regulator_match(&pdev->dev, reg_np, rdata, rdev_num);
	if (s2mps11->dev_type == S2MPS14X)
		s2mps14_pmic_dt_parse_ext_control_gpio(pdev, rdata, s2mps11);

	of_node_put(reg_np);

	return 0;
}

static int s2mpu02_set_ramp_delay(struct regulator_dev *rdev, int ramp_delay)
{
	unsigned int ramp_val, ramp_shift, ramp_reg;
	int rdev_id = rdev_get_id(rdev);

	switch (rdev_id) {
	case S2MPU02_BUCK1:
		ramp_shift = S2MPU02_BUCK1_RAMP_SHIFT;
		break;
	case S2MPU02_BUCK2:
		ramp_shift = S2MPU02_BUCK2_RAMP_SHIFT;
		break;
	case S2MPU02_BUCK3:
		ramp_shift = S2MPU02_BUCK3_RAMP_SHIFT;
		break;
	case S2MPU02_BUCK4:
		ramp_shift = S2MPU02_BUCK4_RAMP_SHIFT;
		break;
	default:
		return 0;
	}
	ramp_reg = S2MPU02_REG_RAMP1;
	ramp_val = get_ramp_delay(ramp_delay);

	return regmap_update_bits(rdev->regmap, ramp_reg,
				  S2MPU02_BUCK1234_RAMP_MASK << ramp_shift,
				  ramp_val << ramp_shift);
}

static const struct regulator_ops s2mpu02_ldo_ops = {
	.list_voltage		= regulator_list_voltage_linear,
	.map_voltage		= regulator_map_voltage_linear,
	.is_enabled		= regulator_is_enabled_regmap,
	.enable			= s2mps11_regulator_enable,
	.disable		= regulator_disable_regmap,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_voltage_sel	= regulator_set_voltage_sel_regmap,
	.set_voltage_time_sel	= regulator_set_voltage_time_sel,
	.set_suspend_disable	= s2mps11_regulator_set_suspend_disable,
};

static const struct regulator_ops s2mpu02_buck_ops = {
	.list_voltage		= regulator_list_voltage_linear,
	.map_voltage		= regulator_map_voltage_linear,
	.is_enabled		= regulator_is_enabled_regmap,
	.enable			= s2mps11_regulator_enable,
	.disable		= regulator_disable_regmap,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_voltage_sel	= regulator_set_voltage_sel_regmap,
	.set_voltage_time_sel	= regulator_set_voltage_time_sel,
	.set_suspend_disable	= s2mps11_regulator_set_suspend_disable,
	.set_ramp_delay		= s2mpu02_set_ramp_delay,
};

#define regulator_desc_s2mpu02_ldo1(num) {		\
	.name		= "LDO"#num,			\
	.id		= S2MPU02_LDO##num,		\
	.ops		= &s2mpu02_ldo_ops,		\
	.type		= REGULATOR_VOLTAGE,		\
	.owner		= THIS_MODULE,			\
	.min_uV		= S2MPU02_LDO_MIN_900MV,	\
	.uV_step	= S2MPU02_LDO_STEP_12_5MV,	\
	.linear_min_sel	= S2MPU02_LDO_GROUP1_START_SEL,	\
	.n_voltages	= S2MPU02_LDO_N_VOLTAGES,	\
	.vsel_reg	= S2MPU02_REG_L1CTRL,		\
	.vsel_mask	= S2MPU02_LDO_VSEL_MASK,	\
	.enable_reg	= S2MPU02_REG_L1CTRL,		\
	.enable_mask	= S2MPU02_ENABLE_MASK		\
}
#define regulator_desc_s2mpu02_ldo2(num) {		\
	.name		= "LDO"#num,			\
	.id		= S2MPU02_LDO##num,		\
	.ops		= &s2mpu02_ldo_ops,		\
	.type		= REGULATOR_VOLTAGE,		\
	.owner		= THIS_MODULE,			\
	.min_uV		= S2MPU02_LDO_MIN_1050MV,	\
	.uV_step	= S2MPU02_LDO_STEP_25MV,	\
	.linear_min_sel	= S2MPU02_LDO_GROUP2_START_SEL,	\
	.n_voltages	= S2MPU02_LDO_N_VOLTAGES,	\
	.vsel_reg	= S2MPU02_REG_L2CTRL1,		\
	.vsel_mask	= S2MPU02_LDO_VSEL_MASK,	\
	.enable_reg	= S2MPU02_REG_L2CTRL1,		\
	.enable_mask	= S2MPU02_ENABLE_MASK		\
}
#define regulator_desc_s2mpu02_ldo3(num) {		\
	.name		= "LDO"#num,			\
	.id		= S2MPU02_LDO##num,		\
	.ops		= &s2mpu02_ldo_ops,		\
	.type		= REGULATOR_VOLTAGE,		\
	.owner		= THIS_MODULE,			\
	.min_uV		= S2MPU02_LDO_MIN_900MV,	\
	.uV_step	= S2MPU02_LDO_STEP_12_5MV,	\
	.linear_min_sel	= S2MPU02_LDO_GROUP1_START_SEL,	\
	.n_voltages	= S2MPU02_LDO_N_VOLTAGES,	\
	.vsel_reg	= S2MPU02_REG_L3CTRL + num - 3,	\
	.vsel_mask	= S2MPU02_LDO_VSEL_MASK,	\
	.enable_reg	= S2MPU02_REG_L3CTRL + num - 3,	\
	.enable_mask	= S2MPU02_ENABLE_MASK		\
}
#define regulator_desc_s2mpu02_ldo4(num) {		\
	.name		= "LDO"#num,			\
	.id		= S2MPU02_LDO##num,		\
	.ops		= &s2mpu02_ldo_ops,		\
	.type		= REGULATOR_VOLTAGE,		\
	.owner		= THIS_MODULE,			\
	.min_uV		= S2MPU02_LDO_MIN_1050MV,	\
	.uV_step	= S2MPU02_LDO_STEP_25MV,	\
	.linear_min_sel	= S2MPU02_LDO_GROUP2_START_SEL,	\
	.n_voltages	= S2MPU02_LDO_N_VOLTAGES,	\
	.vsel_reg	= S2MPU02_REG_L3CTRL + num - 3,	\
	.vsel_mask	= S2MPU02_LDO_VSEL_MASK,	\
	.enable_reg	= S2MPU02_REG_L3CTRL + num - 3,	\
	.enable_mask	= S2MPU02_ENABLE_MASK		\
}
#define regulator_desc_s2mpu02_ldo5(num) {		\
	.name		= "LDO"#num,			\
	.id		= S2MPU02_LDO##num,		\
	.ops		= &s2mpu02_ldo_ops,		\
	.type		= REGULATOR_VOLTAGE,		\
	.owner		= THIS_MODULE,			\
	.min_uV		= S2MPU02_LDO_MIN_1600MV,	\
	.uV_step	= S2MPU02_LDO_STEP_50MV,	\
	.linear_min_sel	= S2MPU02_LDO_GROUP3_START_SEL,	\
	.n_voltages	= S2MPU02_LDO_N_VOLTAGES,	\
	.vsel_reg	= S2MPU02_REG_L3CTRL + num - 3,	\
	.vsel_mask	= S2MPU02_LDO_VSEL_MASK,	\
	.enable_reg	= S2MPU02_REG_L3CTRL + num - 3,	\
	.enable_mask	= S2MPU02_ENABLE_MASK		\
}

#define regulator_desc_s2mpu02_buck1234(num) {			\
	.name		= "BUCK"#num,				\
	.id		= S2MPU02_BUCK##num,			\
	.ops		= &s2mpu02_buck_ops,			\
	.type		= REGULATOR_VOLTAGE,			\
	.owner		= THIS_MODULE,				\
	.min_uV		= S2MPU02_BUCK1234_MIN_600MV,		\
	.uV_step	= S2MPU02_BUCK1234_STEP_6_25MV,		\
	.n_voltages	= S2MPU02_BUCK_N_VOLTAGES,		\
	.linear_min_sel = S2MPU02_BUCK1234_START_SEL,		\
	.ramp_delay	= S2MPU02_BUCK_RAMP_DELAY,		\
	.vsel_reg	= S2MPU02_REG_B1CTRL2 + (num - 1) * 2,	\
	.vsel_mask	= S2MPU02_BUCK_VSEL_MASK,		\
	.enable_reg	= S2MPU02_REG_B1CTRL1 + (num - 1) * 2,	\
	.enable_mask	= S2MPU02_ENABLE_MASK			\
}
#define regulator_desc_s2mpu02_buck5(num) {			\
	.name		= "BUCK"#num,				\
	.id		= S2MPU02_BUCK##num,			\
	.ops		= &s2mpu02_ldo_ops,			\
	.type		= REGULATOR_VOLTAGE,			\
	.owner		= THIS_MODULE,				\
	.min_uV		= S2MPU02_BUCK5_MIN_1081_25MV,		\
	.uV_step	= S2MPU02_BUCK5_STEP_6_25MV,		\
	.n_voltages	= S2MPU02_BUCK_N_VOLTAGES,		\
	.linear_min_sel = S2MPU02_BUCK5_START_SEL,		\
	.ramp_delay	= S2MPU02_BUCK_RAMP_DELAY,		\
	.vsel_reg	= S2MPU02_REG_B5CTRL2,			\
	.vsel_mask	= S2MPU02_BUCK_VSEL_MASK,		\
	.enable_reg	= S2MPU02_REG_B5CTRL1,			\
	.enable_mask	= S2MPU02_ENABLE_MASK			\
}
#define regulator_desc_s2mpu02_buck6(num) {			\
	.name		= "BUCK"#num,				\
	.id		= S2MPU02_BUCK##num,			\
	.ops		= &s2mpu02_ldo_ops,			\
	.type		= REGULATOR_VOLTAGE,			\
	.owner		= THIS_MODULE,				\
	.min_uV		= S2MPU02_BUCK6_MIN_1700MV,		\
	.uV_step	= S2MPU02_BUCK6_STEP_2_50MV,		\
	.n_voltages	= S2MPU02_BUCK_N_VOLTAGES,		\
	.linear_min_sel = S2MPU02_BUCK6_START_SEL,		\
	.ramp_delay	= S2MPU02_BUCK_RAMP_DELAY,		\
	.vsel_reg	= S2MPU02_REG_B6CTRL2,			\
	.vsel_mask	= S2MPU02_BUCK_VSEL_MASK,		\
	.enable_reg	= S2MPU02_REG_B6CTRL1,			\
	.enable_mask	= S2MPU02_ENABLE_MASK			\
}
#define regulator_desc_s2mpu02_buck7(num) {			\
	.name		= "BUCK"#num,				\
	.id		= S2MPU02_BUCK##num,			\
	.ops		= &s2mpu02_ldo_ops,			\
	.type		= REGULATOR_VOLTAGE,			\
	.owner		= THIS_MODULE,				\
	.min_uV		= S2MPU02_BUCK7_MIN_900MV,		\
	.uV_step	= S2MPU02_BUCK7_STEP_6_25MV,		\
	.n_voltages	= S2MPU02_BUCK_N_VOLTAGES,		\
	.linear_min_sel = S2MPU02_BUCK7_START_SEL,		\
	.ramp_delay	= S2MPU02_BUCK_RAMP_DELAY,		\
	.vsel_reg	= S2MPU02_REG_B7CTRL2,			\
	.vsel_mask	= S2MPU02_BUCK_VSEL_MASK,		\
	.enable_reg	= S2MPU02_REG_B7CTRL1,			\
	.enable_mask	= S2MPU02_ENABLE_MASK			\
}

static const struct regulator_desc s2mpu02_regulators[] = {
	regulator_desc_s2mpu02_ldo1(1),
	regulator_desc_s2mpu02_ldo2(2),
	regulator_desc_s2mpu02_ldo4(3),
	regulator_desc_s2mpu02_ldo5(4),
	regulator_desc_s2mpu02_ldo4(5),
	regulator_desc_s2mpu02_ldo3(6),
	regulator_desc_s2mpu02_ldo3(7),
	regulator_desc_s2mpu02_ldo4(8),
	regulator_desc_s2mpu02_ldo5(9),
	regulator_desc_s2mpu02_ldo3(10),
	regulator_desc_s2mpu02_ldo4(11),
	regulator_desc_s2mpu02_ldo5(12),
	regulator_desc_s2mpu02_ldo5(13),
	regulator_desc_s2mpu02_ldo5(14),
	regulator_desc_s2mpu02_ldo5(15),
	regulator_desc_s2mpu02_ldo5(16),
	regulator_desc_s2mpu02_ldo4(17),
	regulator_desc_s2mpu02_ldo5(18),
	regulator_desc_s2mpu02_ldo3(19),
	regulator_desc_s2mpu02_ldo4(20),
	regulator_desc_s2mpu02_ldo5(21),
	regulator_desc_s2mpu02_ldo5(22),
	regulator_desc_s2mpu02_ldo5(23),
	regulator_desc_s2mpu02_ldo4(24),
	regulator_desc_s2mpu02_ldo5(25),
	regulator_desc_s2mpu02_ldo4(26),
	regulator_desc_s2mpu02_ldo5(27),
	regulator_desc_s2mpu02_ldo5(28),
	regulator_desc_s2mpu02_buck1234(1),
	regulator_desc_s2mpu02_buck1234(2),
	regulator_desc_s2mpu02_buck1234(3),
	regulator_desc_s2mpu02_buck1234(4),
	regulator_desc_s2mpu02_buck5(5),
	regulator_desc_s2mpu02_buck6(6),
	regulator_desc_s2mpu02_buck7(7),
};

#define regulator_desc_s2mpu05_ldo_reg(num, min, step, reg) {	\
	.name		= "ldo"#num,				\
	.id		= S2MPU05_LDO##num,			\
	.ops		= &s2mpu02_ldo_ops,			\
	.type		= REGULATOR_VOLTAGE,			\
	.owner		= THIS_MODULE,				\
	.min_uV		= min,					\
	.uV_step	= step,					\
	.n_voltages	= S2MPU05_LDO_N_VOLTAGES,		\
	.vsel_reg	= reg,					\
	.vsel_mask	= S2MPU05_LDO_VSEL_MASK,		\
	.enable_reg	= reg,					\
	.enable_mask	= S2MPU05_ENABLE_MASK,			\
	.enable_time	= S2MPU05_ENABLE_TIME_LDO		\
}

#define regulator_desc_s2mpu05_ldo(num, reg, min, step) \
	regulator_desc_s2mpu05_ldo_reg(num, min, step, S2MPU05_REG_L##num##reg)

#define regulator_desc_s2mpu05_ldo1(num, reg) \
	regulator_desc_s2mpu05_ldo(num, reg, S2MPU05_LDO_MIN1, S2MPU05_LDO_STEP1)

#define regulator_desc_s2mpu05_ldo2(num, reg) \
	regulator_desc_s2mpu05_ldo(num, reg, S2MPU05_LDO_MIN1, S2MPU05_LDO_STEP2)

#define regulator_desc_s2mpu05_ldo3(num, reg) \
	regulator_desc_s2mpu05_ldo(num, reg, S2MPU05_LDO_MIN2, S2MPU05_LDO_STEP2)

#define regulator_desc_s2mpu05_ldo4(num, reg) \
	regulator_desc_s2mpu05_ldo(num, reg, S2MPU05_LDO_MIN3, S2MPU05_LDO_STEP2)

#define regulator_desc_s2mpu05_buck(num, which) {	\
	.name		= "buck"#num,			\
	.id		= S2MPU05_BUCK##num,		\
	.ops		= &s2mpu02_buck_ops,		\
	.type		= REGULATOR_VOLTAGE,		\
	.owner		= THIS_MODULE,			\
	.min_uV		= S2MPU05_BUCK_MIN##which,	\
	.uV_step	= S2MPU05_BUCK_STEP##which,	\
	.n_voltages	= S2MPU05_BUCK_N_VOLTAGES,	\
	.vsel_reg	= S2MPU05_REG_B##num##CTRL2,	\
	.vsel_mask	= S2MPU05_BUCK_VSEL_MASK,	\
	.enable_reg	= S2MPU05_REG_B##num##CTRL1,	\
	.enable_mask	= S2MPU05_ENABLE_MASK,		\
	.enable_time	= S2MPU05_ENABLE_TIME_BUCK##num	\
}

#define regulator_desc_s2mpu05_buck123(num) regulator_desc_s2mpu05_buck(num, 1)
#define regulator_desc_s2mpu05_buck45(num) regulator_desc_s2mpu05_buck(num, 2)

static const struct regulator_desc s2mpu05_regulators[] = {
	regulator_desc_s2mpu05_ldo4(1, CTRL),
	regulator_desc_s2mpu05_ldo3(2, CTRL),
	regulator_desc_s2mpu05_ldo2(3, CTRL),
	regulator_desc_s2mpu05_ldo1(4, CTRL),
	regulator_desc_s2mpu05_ldo1(5, CTRL),
	regulator_desc_s2mpu05_ldo1(6, CTRL),
	regulator_desc_s2mpu05_ldo2(7, CTRL),
	regulator_desc_s2mpu05_ldo3(8, CTRL),
	regulator_desc_s2mpu05_ldo4(9, CTRL1),
	regulator_desc_s2mpu05_ldo4(10, CTRL),
	/* LDOs 11-24 are used for CP. They aren't documented. */
	regulator_desc_s2mpu05_ldo2(25, CTRL),
	regulator_desc_s2mpu05_ldo3(26, CTRL),
	regulator_desc_s2mpu05_ldo2(27, CTRL),
	regulator_desc_s2mpu05_ldo3(28, CTRL),
	regulator_desc_s2mpu05_ldo3(29, CTRL),
	regulator_desc_s2mpu05_ldo2(30, CTRL),
	regulator_desc_s2mpu05_ldo3(31, CTRL),
	regulator_desc_s2mpu05_ldo3(32, CTRL),
	regulator_desc_s2mpu05_ldo3(33, CTRL),
	regulator_desc_s2mpu05_ldo3(34, CTRL),
	regulator_desc_s2mpu05_ldo3(35, CTRL),
	regulator_desc_s2mpu05_buck123(1),
	regulator_desc_s2mpu05_buck123(2),
	regulator_desc_s2mpu05_buck123(3),
	regulator_desc_s2mpu05_buck45(4),
	regulator_desc_s2mpu05_buck45(5),
};

/*
 * S2MPS17 (Exynos8895), as the vendor driver S:drivers/regulator/s2mps17.c:
 * a regulator is on while CTRL[7:6] (the "operating mode") is non-zero; the
 * DT regulator-initial-mode selects the mode written at registration and by
 * every later enable: 1 = on in standby (PWREN), 2 = on with PWREN_MIF,
 * 3 = always on (:60-90, :107-130).
 */
static unsigned int s2mps17_of_map_mode(unsigned int val)
{
	switch (val) {
	case 1:		/* SEC_OPMODE_SUSPEND -> 0x1 */
		return REGULATOR_MODE_STANDBY;
	case 2:		/* SEC_OPMODE_MIF / LOWPOWER -> 0x2 */
		return REGULATOR_MODE_IDLE;
	default:	/* SEC_OPMODE_ON and anything else -> 0x3 */
		return REGULATOR_MODE_NORMAL;
	}
}

static int s2mps17_set_mode(struct regulator_dev *rdev, unsigned int mode)
{
	struct s2mps11_info *s2mps11 = rdev_get_drvdata(rdev);
	unsigned int field, val;
	int ret;

	switch (mode) {
	case REGULATOR_MODE_STANDBY:
		field = 0x1;
		break;
	case REGULATOR_MODE_IDLE:
		field = 0x2;
		break;
	default:
		field = 0x3;
		break;
	}
	val = field << S2MPS17_ENABLE_SHIFT;

	ret = regmap_update_bits(rdev->regmap, rdev->desc->enable_reg,
				 rdev->desc->enable_mask, val);
	if (ret)
		return ret;

	s2mps11->opmode[rdev_get_id(rdev)] = val;

	return 0;
}

static int s2mps17_enable(struct regulator_dev *rdev)
{
	struct s2mps11_info *s2mps11 = rdev_get_drvdata(rdev);

	return regmap_update_bits(rdev->regmap, rdev->desc->enable_reg,
				  rdev->desc->enable_mask,
				  s2mps11->opmode[rdev_get_id(rdev)]);
}

/* Vendor s2m_set_voltage_time_sel() (:321-349): a delay only when raising. */
static int s2mps17_set_voltage_time_sel(struct regulator_dev *rdev,
					unsigned int old_selector,
					unsigned int new_selector)
{
	unsigned int ramp_delay = rdev->constraints->ramp_delay ?:
				  rdev->desc->ramp_delay;
	int old_uV, new_uV;

	if (!ramp_delay)
		return -EINVAL;
	if (old_selector >= new_selector)
		return 0;

	old_uV = rdev->desc->ops->list_voltage(rdev, old_selector);
	new_uV = rdev->desc->ops->list_voltage(rdev, new_selector);

	return DIV_ROUND_UP(new_uV - old_uV, ramp_delay);
}

/* Vendor s2m_set_ramp_delay() and get_ramp_delay() (:188-244), BUCKRAMP 0x36 */
static int s2mps17_set_ramp_delay(struct regulator_dev *rdev, int ramp_delay)
{
	unsigned int shift, cnt = 0, x = ramp_delay / 1000 / 6;

	while (x >>= 1)
		cnt++;

	switch (rdev_get_id(rdev)) {
	case S2MPS17_BUCK2:
	case S2MPS17_BUCK4:
	case S2MPS17_BUCK5:
	case S2MPS17_BUCK10:
	case S2MPS17_BUCK12:
		shift = 6;
		break;
	case S2MPS17_BUCK1:
	case S2MPS17_BUCK3:
	case S2MPS17_BUCK6:
		shift = 4;
		break;
	case S2MPS17_BUCK7:
	case S2MPS17_BUCK11:
		shift = 2;
		break;
	case S2MPS17_BUCK8:
	case S2MPS17_BUCK9:
		shift = 0;
		break;
	default:
		return -EINVAL;
	}

	return regmap_update_bits(rdev->regmap, S2MPS17_PMIC_REG_BUCKRAMP,
				  0x3 << shift, cnt << shift);
}

static const struct regulator_ops s2mps17_ldo_ops = {
	.list_voltage		= regulator_list_voltage_linear,
	.map_voltage		= regulator_map_voltage_linear,
	.is_enabled		= regulator_is_enabled_regmap,
	.enable			= s2mps17_enable,
	.disable		= regulator_disable_regmap,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_voltage_sel	= regulator_set_voltage_sel_regmap,
	.set_voltage_time_sel	= s2mps17_set_voltage_time_sel,
	.set_mode		= s2mps17_set_mode,
};

static const struct regulator_ops s2mps17_buck_ops = {
	.list_voltage		= regulator_list_voltage_linear,
	.map_voltage		= regulator_map_voltage_linear,
	.is_enabled		= regulator_is_enabled_regmap,
	.enable			= s2mps17_enable,
	.disable		= regulator_disable_regmap,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_voltage_sel	= regulator_set_voltage_sel_regmap,
	.set_voltage_time_sel	= s2mps17_set_voltage_time_sel,
	.set_mode		= s2mps17_set_mode,
	.set_ramp_delay		= s2mps17_set_ramp_delay,
};

/*
 * BUCK6 (vdd_g3d) is switched by the G3D power-down pin (g3d_en, see the
 * post-registration init); the vendor driver ignores enable/disable for it
 * and reports it off (:113-115, :139-140, :168-174). Without enable ops the
 * core treats it as always on (A:drivers/regulator/core.c:3244-3246).
 */
static const struct regulator_ops s2mps17_buck6_ops = {
	.list_voltage		= regulator_list_voltage_linear,
	.map_voltage		= regulator_map_voltage_linear,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_voltage_sel	= regulator_set_voltage_sel_regmap,
	.set_voltage_time_sel	= s2mps17_set_voltage_time_sel,
	.set_mode		= s2mps17_set_mode,
	.set_ramp_delay		= s2mps17_set_ramp_delay,
};

#define regulator_desc_s2mps17_ldo(num, reg, min, step) {	\
	.name		= "LDO"#num,				\
	.id		= S2MPS17_LDO##num,			\
	.ops		= &s2mps17_ldo_ops,			\
	.type		= REGULATOR_VOLTAGE,			\
	.owner		= THIS_MODULE,				\
	.min_uV		= S2MPS17_LDO_MIN##min,			\
	.uV_step	= S2MPS17_LDO_STEP##step,		\
	.n_voltages	= S2MPS17_LDO_N_VOLTAGES,		\
	.vsel_reg	= S2MPS17_PMIC_REG_##reg,		\
	.vsel_mask	= S2MPS17_LDO_VSEL_MASK,		\
	.enable_reg	= S2MPS17_PMIC_REG_##reg,		\
	.enable_mask	= S2MPS17_ENABLE_MASK,			\
	.enable_time	= S2MPS17_ENABLE_TIME_LDO,		\
	.ramp_delay	= S2MPS17_RAMP_DELAY,			\
	.of_map_mode	= s2mps17_of_map_mode,			\
}

#define regulator_desc_s2mps17_buck(num, ops_, vsel, min, step) {	\
	.name		= "BUCK"#num,				\
	.id		= S2MPS17_BUCK##num,			\
	.ops		= ops_,					\
	.type		= REGULATOR_VOLTAGE,			\
	.owner		= THIS_MODULE,				\
	.min_uV		= S2MPS17_BUCK_MIN##min,		\
	.uV_step	= S2MPS17_BUCK_STEP##step,		\
	.n_voltages	= S2MPS17_BUCK_N_VOLTAGES,		\
	.vsel_reg	= S2MPS17_PMIC_REG_##vsel,		\
	.vsel_mask	= S2MPS17_BUCK_VSEL_MASK,		\
	.enable_reg	= S2MPS17_PMIC_REG_B##num##CTRL,	\
	.enable_mask	= S2MPS17_ENABLE_MASK,			\
	.enable_time	= S2MPS17_ENABLE_TIME_BUCK##num,	\
	.ramp_delay	= S2MPS17_RAMP_DELAY,			\
	.of_map_mode	= s2mps17_of_map_mode,			\
}

/* S:drivers/regulator/s2mps17.c:426-537 (BB is not in the board DT, not registered) */
static const struct regulator_desc s2mps17_regulators[] = {
	regulator_desc_s2mps17_ldo(1, L1CTRL, 2, 1),
	regulator_desc_s2mps17_ldo(2, L2CTRL, 3, 2),
	regulator_desc_s2mps17_ldo(3, L3CTRL, 1, 2),
	regulator_desc_s2mps17_ldo(4, L4CTRL, 2, 1),
	regulator_desc_s2mps17_ldo(5, L5CTRL, 1, 2),
	regulator_desc_s2mps17_ldo(6, L6CTRL, 3, 2),
	regulator_desc_s2mps17_ldo(7, L7CTRL, 4, 2),
	regulator_desc_s2mps17_ldo(8, L8CTRL, 4, 2),
	regulator_desc_s2mps17_ldo(9, L9CTRL1, 4, 2),
	regulator_desc_s2mps17_ldo(10, L10CTRL, 2, 1),
	regulator_desc_s2mps17_ldo(11, L11CTRL, 2, 1),
	regulator_desc_s2mps17_ldo(12, L12CTRL, 2, 1),
	regulator_desc_s2mps17_ldo(13, L13CTRL, 1, 1),
	regulator_desc_s2mps17_ldo(20, L20CTRL, 1, 1),
	regulator_desc_s2mps17_ldo(29, L29CTRL, 3, 2),
	regulator_desc_s2mps17_ldo(30, L30CTRL, 3, 2),
	regulator_desc_s2mps17_ldo(31, L31CTRL, 3, 2),
	regulator_desc_s2mps17_ldo(32, L32CTRL, 1, 2),
	regulator_desc_s2mps17_ldo(33, L33CTRL, 1, 1),
	regulator_desc_s2mps17_ldo(34, L34CTRL, 1, 2),
	regulator_desc_s2mps17_ldo(35, L35CTRL, 3, 2),
	regulator_desc_s2mps17_ldo(36, L36CTRL, 3, 2),
	regulator_desc_s2mps17_ldo(37, L37CTRL, 3, 2),
	regulator_desc_s2mps17_ldo(38, L38CTRL, 1, 2),
	regulator_desc_s2mps17_ldo(39, L39CTRL, 1, 2),
	regulator_desc_s2mps17_ldo(40, L40CTRL, 1, 1),
	regulator_desc_s2mps17_ldo(41, L41CTRL, 3, 2),
	regulator_desc_s2mps17_ldo(42, L42CTRL, 1, 2),
	regulator_desc_s2mps17_ldo(43, L43CTRL, 1, 1),
	regulator_desc_s2mps17_buck(1, &s2mps17_buck_ops, B1OUT, 1, 1),
	regulator_desc_s2mps17_buck(2, &s2mps17_buck_ops, B2OUT, 1, 1),
	regulator_desc_s2mps17_buck(3, &s2mps17_buck_ops, B3OUT, 1, 1),
	regulator_desc_s2mps17_buck(4, &s2mps17_buck_ops, B4OUT, 1, 1),
	regulator_desc_s2mps17_buck(5, &s2mps17_buck_ops, B5OUT, 1, 1),
	regulator_desc_s2mps17_buck(6, &s2mps17_buck6_ops, B6OUT1, 1, 1),
	regulator_desc_s2mps17_buck(7, &s2mps17_buck_ops, B7OUT, 1, 1),
	regulator_desc_s2mps17_buck(8, &s2mps17_buck_ops, B8OUT, 1, 1),
	regulator_desc_s2mps17_buck(9, &s2mps17_buck_ops, B9OUT, 1, 1),
	regulator_desc_s2mps17_buck(10, &s2mps17_buck_ops, B10OUT, 1, 1),
	regulator_desc_s2mps17_buck(11, &s2mps17_buck_ops, B11OUT, 2, 3),
	regulator_desc_s2mps17_buck(12, &s2mps17_buck_ops, B12OUT1, 1, 1),
};

/*
 * S2MPS17 board setup after registration, in the vendor driver's order
 * (S:drivers/regulator/s2mps17.c:818-857). The board values come from the
 * greatlte PMIC node (g3d_en = <1>, smpl_warn_en = <1>, smpl_warn_vth =
 * <0x80>, smpl_warn_hys = <0x00>; S:arch/arm64/boot/dts/exynos/
 * exynos8895-greatlte_common.dtsi:223-232), which have no binding here.
 */
static int s2mps17_init(struct device *dev, struct regmap *pmic)
{
	int ret;

	/* g3d_en: BUCK6 follows the G3D power-down pin, LDO9 syncs with it (:818-832) */
	ret = regmap_update_bits(pmic, S2MPS17_PMIC_REG_B6CTRL, 0xc0, 0x80);
	if (ret)
		return dev_err_probe(dev, ret, "s2mps17: buck6 gpio control error\n");
	ret = regmap_update_bits(pmic, S2MPS17_PMIC_REG_L9CTRL1, 0xc0, 0x40);
	if (ret)
		return dev_err_probe(dev, ret, "s2mps17: regulator sync enable error\n");

	/* smpl_warn: threshold then hysteresis (:834-852) */
	ret = regmap_update_bits(pmic, S2MPS17_PMIC_REG_CTRL2, 0xe0, 0x80);
	if (ret)
		return dev_err_probe(dev, ret, "s2mps17: smpl_warn vth error\n");
	ret = regmap_update_bits(pmic, S2MPS17_PMIC_REG_CTRL2, 0x18, 0x00);
	if (ret)
		return dev_err_probe(dev, ret, "s2mps17: smpl_warn hys error\n");

	/* the vendor driver ignores the result of the next two writes (:854-857) */
	regmap_update_bits(pmic, S2MPS17_PMIC_REG_RTCBUF, 0x04, 0x04);
	regmap_write(pmic, S2MPS17_PMIC_REG_SELMIF1, 0xd6);	/* "SELMIF : LDO4,5,8,11,12" */

	dev_info(dev, "s2mps17: init g3d_en smpl_warn rtcbuf selmif1=0xd6\n");

	return 0;
}

/* One line per S2MPS17 regulator after registration (L2 pass line P8). */
static void s2mps17_log_regulators(struct device *dev, struct regmap *regmap,
				   const struct regulator_desc *regulators,
				   struct regulator_dev **rdevs,
				   unsigned int rdev_num)
{
	unsigned int i, ctrl;

	for (i = 0; i < rdev_num; i++) {
		if (regmap_read(regmap, regulators[i].enable_reg, &ctrl)) {
			dev_err(dev, "s2mps17: %s read failed\n", regulators[i].name);
			continue;
		}
		dev_info(dev, "s2mps17: %s %s ctrl=0x%02x en=%u %d uV\n",
			 regulators[i].name, rdev_get_name(rdevs[i]), ctrl,
			 !!(ctrl & S2MPS17_ENABLE_MASK),
			 regulator_get_voltage_rdev(rdevs[i]));
	}
}

static int s2mps11_pmic_probe(struct platform_device *pdev)
{
	struct sec_pmic_dev *iodev = dev_get_drvdata(pdev->dev.parent);
	struct regulator_config config = { };
	struct s2mps11_info *s2mps11;
	unsigned int rdev_num = 0;
	int i, ret = 0;
	const struct regulator_desc *regulators;

	s2mps11 = devm_kzalloc(&pdev->dev, sizeof(struct s2mps11_info),
				GFP_KERNEL);
	if (!s2mps11)
		return -ENOMEM;

	s2mps11->dev_type = platform_get_device_id(pdev)->driver_data;
	switch (s2mps11->dev_type) {
	case S2MPS11X:
		rdev_num = ARRAY_SIZE(s2mps11_regulators);
		regulators = s2mps11_regulators;
		BUILD_BUG_ON(S2MPS_REGULATOR_MAX < ARRAY_SIZE(s2mps11_regulators));
		break;
	case S2MPS13X:
		rdev_num = ARRAY_SIZE(s2mps13_regulators);
		regulators = s2mps13_regulators;
		BUILD_BUG_ON(S2MPS_REGULATOR_MAX < ARRAY_SIZE(s2mps13_regulators));
		break;
	case S2MPS14X:
		rdev_num = ARRAY_SIZE(s2mps14_regulators);
		regulators = s2mps14_regulators;
		BUILD_BUG_ON(S2MPS_REGULATOR_MAX < ARRAY_SIZE(s2mps14_regulators));
		break;
	case S2MPS15X:
		rdev_num = ARRAY_SIZE(s2mps15_regulators);
		regulators = s2mps15_regulators;
		BUILD_BUG_ON(S2MPS_REGULATOR_MAX < ARRAY_SIZE(s2mps15_regulators));
		break;
	case S2MPS17X:
		rdev_num = ARRAY_SIZE(s2mps17_regulators);
		regulators = s2mps17_regulators;
		BUILD_BUG_ON(S2MPS_REGULATOR_MAX < ARRAY_SIZE(s2mps17_regulators));
		break;
	case S2MPU02:
		rdev_num = ARRAY_SIZE(s2mpu02_regulators);
		regulators = s2mpu02_regulators;
		BUILD_BUG_ON(S2MPS_REGULATOR_MAX < ARRAY_SIZE(s2mpu02_regulators));
		break;
	case S2MPU05:
		rdev_num = ARRAY_SIZE(s2mpu05_regulators);
		regulators = s2mpu05_regulators;
		BUILD_BUG_ON(S2MPS_REGULATOR_MAX < ARRAY_SIZE(s2mpu05_regulators));
		break;
	default:
		dev_err(&pdev->dev, "Invalid device type: %u\n",
				    s2mps11->dev_type);
		return -EINVAL;
	}

	s2mps11->ext_control_gpiod = devm_kcalloc(&pdev->dev, rdev_num,
			       sizeof(*s2mps11->ext_control_gpiod), GFP_KERNEL);
	if (!s2mps11->ext_control_gpiod)
		return -ENOMEM;

	struct of_regulator_match *rdata __free(kfree) =
		kcalloc(rdev_num, sizeof(*rdata), GFP_KERNEL);
	if (!rdata)
		return -ENOMEM;

	struct regulator_dev **rdevs __free(kfree) =
		kcalloc(rdev_num, sizeof(*rdevs), GFP_KERNEL);
	if (!rdevs)
		return -ENOMEM;

	for (i = 0; i < rdev_num; i++) {
		rdata[i].name = regulators[i].name;
		/*
		 * S2MPS17: hand the desc to the DT parser, so its of_map_mode()
		 * turns regulator-initial-mode into constraints
		 * (A:drivers/regulator/of_regulator.c:197-207, 444-446), as the
		 * vendor driver did (S:drivers/regulator/s2mps17.c:632-634).
		 */
		if (s2mps11->dev_type == S2MPS17X) {
			rdata[i].desc = &regulators[i];
			/* S:drivers/regulator/s2mps17.c:801-802 */
			s2mps11->opmode[regulators[i].id] = regulators[i].enable_mask;
		}
	}

	ret = s2mps11_pmic_dt_parse(pdev, rdata, s2mps11, rdev_num);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, s2mps11);

	config.dev = &pdev->dev;
	config.regmap = iodev->regmap_pmic;
	config.driver_data = s2mps11;
	for (i = 0; i < rdev_num; i++) {
		struct regulator_dev *regulator;

		config.init_data = rdata[i].init_data;
		config.of_node = rdata[i].of_node;
		config.ena_gpiod = s2mps11->ext_control_gpiod[i];
		/*
		 * Hand the GPIO descriptor management over to the regulator
		 * core, remove it from devres management.
		 */
		if (config.ena_gpiod)
			devm_gpiod_unhinge(&pdev->dev, config.ena_gpiod);
		regulator = devm_regulator_register(&pdev->dev,
						&regulators[i], &config);
		if (IS_ERR(regulator)) {
			dev_err(&pdev->dev, "regulator init failed for %d\n",
				i);
			return PTR_ERR(regulator);
		}
		rdevs[i] = regulator;

		if (config.ena_gpiod) {
			ret = s2mps14_pmic_enable_ext_control(s2mps11,
					regulator);
			if (ret < 0) {
				dev_err(&pdev->dev,
						"failed to enable GPIO control over %s: %d\n",
						regulator->desc->name, ret);
				return ret;
			}
		}
	}

	if (s2mps11->dev_type == S2MPS17X) {
		ret = s2mps17_init(&pdev->dev, iodev->regmap_pmic);
		if (ret)
			return ret;
		s2mps17_log_regulators(&pdev->dev, iodev->regmap_pmic,
				       regulators, rdevs, rdev_num);
	}

	return 0;
}

static const struct platform_device_id s2mps11_pmic_id[] = {
	{ "s2mps11-regulator", S2MPS11X},
	{ "s2mps13-regulator", S2MPS13X},
	{ "s2mps14-regulator", S2MPS14X},
	{ "s2mps15-regulator", S2MPS15X},
	{ "s2mps17-regulator", S2MPS17X},
	{ "s2mpu02-regulator", S2MPU02},
	{ "s2mpu05-regulator", S2MPU05},
	{ },
};
MODULE_DEVICE_TABLE(platform, s2mps11_pmic_id);

static struct platform_driver s2mps11_pmic_driver = {
	.driver = {
		.name = "s2mps11-pmic",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe = s2mps11_pmic_probe,
	.id_table = s2mps11_pmic_id,
};

module_platform_driver(s2mps11_pmic_driver);

/* Module information */
MODULE_AUTHOR("Sangbeom Kim <sbkim73@samsung.com>");
MODULE_DESCRIPTION("Samsung S2MPS11/14/15/S2MPU02/05 Regulator Driver");
MODULE_LICENSE("GPL");
