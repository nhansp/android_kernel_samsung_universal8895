// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * MAXIM MAX77693/MAX77843 Haptic device driver
 *
 * Copyright (C) 2014,2015 Samsung Electronics
 * Jaewon Kim <jaewon02.kim@samsung.com>
 * Krzysztof Kozlowski <krzk@kernel.org>
 *
 * This program is not provided / owned by Maxim Integrated Products.
 */

#include <linux/err.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/slab.h>
#include <linux/string_choices.h>
#include <linux/workqueue.h>
#include <linux/regulator/consumer.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/property.h>
#include <linux/mfd/max77693.h>
#include <linux/mfd/max77693-common.h>
#include <linux/mfd/max77693-private.h>
#include <linux/mfd/max77705-private.h>
#include <linux/mfd/max77843-private.h>
#include <linux/mfd/max77865-private.h>

#define MAX_MAGNITUDE_SHIFT	16

enum max77693_haptic_motor_type {
	MAX77693_HAPTIC_ERM = 0,
	MAX77693_HAPTIC_LRA,
};

enum max77693_haptic_pulse_mode {
	MAX77693_HAPTIC_EXTERNAL_MODE = 0,
	MAX77693_HAPTIC_INTERNAL_MODE,
};

enum max77693_haptic_pwm_divisor {
	MAX77693_HAPTIC_PWM_DIVISOR_32 = 0,
	MAX77693_HAPTIC_PWM_DIVISOR_64,
	MAX77693_HAPTIC_PWM_DIVISOR_128,
	MAX77693_HAPTIC_PWM_DIVISOR_256,
};

struct max77693_haptic {
	enum max77693_types dev_type;

	struct regmap *regmap_pmic;
	struct regmap *regmap_haptic;
	struct device *dev;
	struct input_dev *input_dev;
	struct pwm_device *pwm_dev;
	struct regulator *motor_reg;

	bool enabled;
	bool suspend_state;
	unsigned int magnitude;
	unsigned int pwm_duty;
	enum max77693_haptic_motor_type type;
	enum max77693_haptic_pulse_mode mode;

	/* MAX77865 fields (spec L6F 5.7) */
	u32 max_duty_ns;
	u32 max_timeout_ms;
	unsigned int duty_ns;
	unsigned int intensity;
	struct hrtimer timer;
	struct device *tout_dev;

	struct work_struct work;
};

/* S:drivers/motor/max77865_haptic.c:188-215, spec L6F 5.7 */
static int max77865_haptic_init_reg(struct max77693_haptic *haptic)
{
	struct regmap *map = haptic->regmap_haptic;
	int ret;

	usleep_range(6000, 6100);

	ret = regmap_update_bits(map, MAX77865_PMIC_REG_MAINCTRL1,
				 MAX77865_MAINCTRL1_BIASEN,
				 MAX77865_MAINCTRL1_BIASEN);
	if (ret)
		return ret;

	ret = regmap_update_bits(map, MAX77865_PMIC_REG_MCONFIG,
				 BIT(6) /* MOTOR_EN */, 0);
	if (ret)
		return ret;

	ret = regmap_update_bits(map, MAX77865_PMIC_REG_MCONFIG,
				 BIT(7) /* MOTOR_LRA */, BIT(7));
	if (ret)
		return ret;

	ret = regmap_update_bits(map, MAX77865_PMIC_REG_MCONFIG,
				 BIT(1) /* DIVIDER_128 */, BIT(1));
	if (ret)
		return ret;

	return 0;
}

/* S:drivers/motor/max77865_haptic.c:174-186, spec L6F 5.7 */
static int max77865_haptic_boost(struct max77693_haptic *haptic, bool on)
{
	struct regmap *map = haptic->regmap_haptic;

	if (on) {
		regmap_update_bits(map, MAX77865_PMIC_REG_BOOSTCONTROL1,
				   MAX77865_BOOSTCONTROL1_BSTOUT, 0x00);
		return regmap_update_bits(map, MAX77865_PMIC_REG_BOOSTCONTROL2,
					  MAX77865_BOOSTCONTROL2_FORCE_EN,
					  MAX77865_BOOSTCONTROL2_FORCE_EN);
	}

	return regmap_update_bits(map, MAX77865_PMIC_REG_BOOSTCONTROL2,
				  MAX77865_BOOSTCONTROL2_FORCE_EN, 0x00);
}

/* S:drivers/motor/max77865_haptic.c:114-131, spec L6F 5.7 */
static unsigned int max77865_haptic_duty(struct max77693_haptic *haptic, int intensity)
{
	struct pwm_args pargs;
	unsigned int p, d;

	pwm_get_args(haptic->pwm_dev, &pargs);
	p = pargs.period;
	d = haptic->max_duty_ns;

	if (intensity == 10000)
		return d;
	if (intensity == 0)
		return p / 2;

	return p / 2 + (int)(d - p / 2) * intensity / 10000;
}

static int max77693_haptic_set_duty_cycle(struct max77693_haptic *haptic)
{
	struct pwm_state state;
	int error;

	pwm_init_state(haptic->pwm_dev, &state);
	if (haptic->dev_type == TYPE_MAX77865)
		state.duty_cycle = haptic->duty_ns;
	else
		state.duty_cycle = (state.period + haptic->pwm_duty) / 2;

	error = pwm_apply_might_sleep(haptic->pwm_dev, &state);
	if (error) {
		dev_err(haptic->dev,
			"failed to set pwm duty cycle: %d\n", error);
		return error;
	}

	return 0;
}

static int max77843_haptic_bias(struct max77693_haptic *haptic, bool on)
{
	int error;

	if (haptic->dev_type != TYPE_MAX77843)
		return 0;

	error = regmap_update_bits(haptic->regmap_haptic,
				   MAX77843_SYS_REG_MAINCTRL1,
				   MAX77843_MAINCTRL1_BIASEN_MASK,
				   on << MAINCTRL1_BIASEN_SHIFT);
	if (error) {
		dev_err(haptic->dev, "failed to %s bias: %d\n",
			str_enable_disable(on), error);
		return error;
	}

	return 0;
}

static int max77693_haptic_configure(struct max77693_haptic *haptic,
				     bool enable)
{
	unsigned int value, config_reg;
	int error;

	switch (haptic->dev_type) {
	case TYPE_MAX77865:
		/* S:drivers/motor/max77865_haptic.c:217-227, spec L6F 5.7 */
		return regmap_update_bits(haptic->regmap_haptic,
					  MAX77865_PMIC_REG_MCONFIG,
					  BIT(6), enable ? BIT(6) : 0);
	case TYPE_MAX77693:
		value = ((haptic->type << MAX77693_CONFIG2_MODE) |
			(enable << MAX77693_CONFIG2_MEN) |
			(haptic->mode << MAX77693_CONFIG2_HTYP) |
			MAX77693_HAPTIC_PWM_DIVISOR_128);
		config_reg = MAX77693_HAPTIC_REG_CONFIG2;
		break;
	case TYPE_MAX77705:
		value = ((haptic->type << MAX77693_CONFIG2_MODE) |
			(enable << MAX77693_CONFIG2_MEN) |
			(haptic->mode << MAX77693_CONFIG2_HTYP) |
			MAX77693_HAPTIC_PWM_DIVISOR_128);
		config_reg = MAX77705_PMIC_REG_MCONFIG;
		break;
	case TYPE_MAX77843:
		value = (haptic->type << MCONFIG_MODE_SHIFT) |
			(enable << MCONFIG_MEN_SHIFT) |
			MAX77693_HAPTIC_PWM_DIVISOR_128;
		config_reg = MAX77843_HAP_REG_MCONFIG;
		break;
	default:
		return -EINVAL;
	}

	error = regmap_write(haptic->regmap_haptic,
			     config_reg, value);
	if (error) {
		dev_err(haptic->dev,
			"failed to update haptic config: %d\n", error);
		return error;
	}

	return 0;
}

static int max77693_haptic_lowsys(struct max77693_haptic *haptic, bool enable)
{
	int error;

	if (haptic->dev_type != TYPE_MAX77693)
		return 0;

	error = regmap_update_bits(haptic->regmap_pmic,
				   MAX77693_PMIC_REG_LSCNFG,
				   MAX77693_PMIC_LOW_SYS_MASK,
				   enable << MAX77693_PMIC_LOW_SYS_SHIFT);
	if (error) {
		dev_err(haptic->dev, "cannot update pmic regmap: %d\n", error);
		return error;
	}

	return 0;
}

static void max77693_haptic_enable(struct max77693_haptic *haptic)
{
	struct pwm_state state;
	int error;

	if (haptic->enabled)
		return;

	pwm_init_state(haptic->pwm_dev, &state);
	if (haptic->dev_type == TYPE_MAX77865)
		state.duty_cycle = haptic->duty_ns;
	else
		state.duty_cycle = (state.period + haptic->pwm_duty) / 2;
	state.enabled = true;

	error = pwm_apply_might_sleep(haptic->pwm_dev, &state);
	if (error) {
		dev_err(haptic->dev,
			"failed to enable haptic pwm device: %d\n", error);
		return;
	}

	error = max77693_haptic_lowsys(haptic, true);
	if (error)
		goto err_enable_lowsys;

	error = max77693_haptic_configure(haptic, true);
	if (error)
		goto err_enable_config;

	haptic->enabled = true;

	return;

err_enable_config:
	max77693_haptic_lowsys(haptic, false);
err_enable_lowsys:
	pwm_disable(haptic->pwm_dev);
}

static void max77693_haptic_disable(struct max77693_haptic *haptic)
{
	int error;

	if (!haptic->enabled)
		return;

	error = max77693_haptic_configure(haptic, false);
	if (error)
		return;

	error = max77693_haptic_lowsys(haptic, false);
	if (error)
		goto err_disable_lowsys;

	pwm_disable(haptic->pwm_dev);
	haptic->enabled = false;

	return;

err_disable_lowsys:
	max77693_haptic_configure(haptic, true);
}

static void max77693_haptic_play_work(struct work_struct *work)
{
	struct max77693_haptic *haptic =
			container_of(work, struct max77693_haptic, work);

	if (!haptic->magnitude)
		max77693_haptic_disable(haptic);
	else if (haptic->enabled)
		max77693_haptic_set_duty_cycle(haptic);
	else
		max77693_haptic_enable(haptic);
}

static int max77693_haptic_play_effect(struct input_dev *dev, void *data,
				       struct ff_effect *effect)
{
	struct max77693_haptic *haptic = input_get_drvdata(dev);
	struct pwm_args pargs;
	u64 period_mag_multi;

	haptic->magnitude = effect->u.rumble.strong_magnitude;
	if (!haptic->magnitude)
		haptic->magnitude = effect->u.rumble.weak_magnitude;

	/*
	 * The magnitude comes from force-feedback interface.
	 * For MAX77865, map 0..0xffff to 0..10000 intensity (spec L6F 5.7).
	 * Otherwise convert magnitude to pwm_duty as:
	 * - pwm_duty = (magnitude * pwm_period) / MAX_MAGNITUDE(0xFFFF)
	 */
	if (haptic->dev_type == TYPE_MAX77865) {
		haptic->duty_ns = max77865_haptic_duty(haptic,
				(int)(haptic->magnitude * 10000 / 0xffff));
	} else {
		pwm_get_args(haptic->pwm_dev, &pargs);
		period_mag_multi = (u64)pargs.period * haptic->magnitude;
		haptic->pwm_duty = (unsigned int)(period_mag_multi >>
							MAX_MAGNITUDE_SHIFT);
	}

	schedule_work(&haptic->work);

	return 0;
}

static int max77693_haptic_open(struct input_dev *dev)
{
	struct max77693_haptic *haptic = input_get_drvdata(dev);
	int error;

	error = max77843_haptic_bias(haptic, true);
	if (error)
		return error;

	error = regulator_enable(haptic->motor_reg);
	if (error) {
		dev_err(haptic->dev,
			"failed to enable regulator: %d\n", error);
		return error;
	}

	return 0;
}

static void max77693_haptic_close(struct input_dev *dev)
{
	struct max77693_haptic *haptic = input_get_drvdata(dev);
	int error;

	cancel_work_sync(&haptic->work);
	max77693_haptic_disable(haptic);

	error = regulator_disable(haptic->motor_reg);
	if (error)
		dev_err(haptic->dev,
			"failed to disable regulator: %d\n", error);

	max77843_haptic_bias(haptic, false);
}

/* S:drivers/motor/max77865_haptic.c:599-608, spec L6F 5.7 */
static enum hrtimer_restart max77865_tout_timer(struct hrtimer *timer)
{
	struct max77693_haptic *haptic =
		container_of(timer, struct max77693_haptic, timer);

	haptic->magnitude = 0;
	schedule_work(&haptic->work);
	return HRTIMER_NORESTART;
}

/* S:drivers/motor/max77865_haptic.c:526-597, spec L6F 5.7 */
static ssize_t enable_store(struct device *dev,
			    struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct max77693_haptic *haptic = dev_get_drvdata(dev);
	int ms, error;

	error = kstrtoint(buf, 0, &ms);
	if (error)
		return -EINVAL;

	hrtimer_cancel(&haptic->timer);
	ms = min_t(int, ms, (int)haptic->max_timeout_ms);

	if (ms > 0) {
		haptic->magnitude = 1;
		schedule_work(&haptic->work);
		hrtimer_start(&haptic->timer, ms_to_ktime(ms), HRTIMER_MODE_REL);
	} else {
		haptic->magnitude = 0;
		schedule_work(&haptic->work);
	}

	return count;
}

/* S:drivers/motor/max77865_haptic.c:511-524, spec L6F 5.7 */
static ssize_t enable_show(struct device *dev,
			   struct device_attribute *attr,
			   char *buf)
{
	struct max77693_haptic *haptic = dev_get_drvdata(dev);
	int rem_ms = 0;

	if (hrtimer_active(&haptic->timer))
		rem_ms = ktime_to_ms(hrtimer_get_remaining(&haptic->timer));

	return sysfs_emit(buf, "%d\n", rem_ms);
}
static DEVICE_ATTR(enable, 0664, enable_show, enable_store);

/* S:drivers/motor/max77865_haptic.c:114-139,356, spec L6F 5.7 */
static ssize_t intensity_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct max77693_haptic *haptic = dev_get_drvdata(dev);
	int i, error;

	error = kstrtoint(buf, 0, &i);
	if (error)
		return -EINVAL;

	if (i < 0 || i > 10000)
		return -EINVAL;

	haptic->intensity = i;
	haptic->duty_ns = max77865_haptic_duty(haptic, i);

	if (haptic->enabled)
		schedule_work(&haptic->work);

	return count;
}

static ssize_t intensity_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	struct max77693_haptic *haptic = dev_get_drvdata(dev);

	return sysfs_emit(buf, "intensity: %u\n", haptic->intensity);
}
static DEVICE_ATTR(intensity, 0664, intensity_show, intensity_store);

static struct attribute *max77865_tout_attrs[] = {
	&dev_attr_enable.attr,
	&dev_attr_intensity.attr,
	NULL,
};
ATTRIBUTE_GROUPS(max77865_tout);

static struct class timed_output_class = {
	.name = "timed_output",
};
static bool tout_class_registered;

static void max77865_tout_unregister(void *data)
{
	struct device *dev = data;

	device_unregister(dev);
}

static void max77865_haptic_cleanup_action(void *data)
{
	struct max77693_haptic *haptic = data;

	hrtimer_cancel(&haptic->timer);
	cancel_work_sync(&haptic->work);
	max77693_haptic_disable(haptic);
	max77865_haptic_boost(haptic, false);
}

static int max77693_haptic_probe(struct platform_device *pdev)
{
	struct max77693_dev *max77693 = dev_get_drvdata(pdev->dev.parent);
	struct max77693_haptic *haptic;
	int error;

	haptic = devm_kzalloc(&pdev->dev, sizeof(*haptic), GFP_KERNEL);
	if (!haptic)
		return -ENOMEM;

	haptic->regmap_pmic = max77693->regmap;
	haptic->dev = &pdev->dev;
	haptic->type = MAX77693_HAPTIC_LRA;
	haptic->mode = MAX77693_HAPTIC_EXTERNAL_MODE;
	haptic->suspend_state = false;

	/* Variant-specific init */
	haptic->dev_type = max77693->type;
	switch (haptic->dev_type) {
	case TYPE_MAX77693:
		haptic->regmap_haptic = max77693->regmap_haptic;
		break;
	case TYPE_MAX77705:
	case TYPE_MAX77843:
	case TYPE_MAX77865:
		haptic->regmap_haptic = max77693->regmap;
		break;
	default:
		dev_err(&pdev->dev, "unsupported device type: %u\n",
			haptic->dev_type);
		return -EINVAL;
	}

	if (haptic->dev_type == TYPE_MAX77865) {
		error = device_property_read_u32(&pdev->dev, "maxim,max-duty-ns",
						 &haptic->max_duty_ns);
		if (error)
			return dev_err_probe(&pdev->dev, error, "missing maxim,max-duty-ns\n");

		error = device_property_read_u32(&pdev->dev, "maxim,max-timeout-ms",
						 &haptic->max_timeout_ms);
		if (error)
			return dev_err_probe(&pdev->dev, error, "missing maxim,max-timeout-ms\n");

		/* S:drivers/motor/max77865_haptic.c:836-837, spec L6F 5.7 */
		haptic->intensity = 8000;
		haptic->duty_ns = haptic->max_duty_ns;
	}

	INIT_WORK(&haptic->work, max77693_haptic_play_work);

	/* Get pwm and regulatot for haptic device */
	haptic->pwm_dev = devm_pwm_get(&pdev->dev, NULL);
	if (IS_ERR(haptic->pwm_dev)) {
		dev_err(&pdev->dev, "failed to get pwm device\n");
		return PTR_ERR(haptic->pwm_dev);
	}

	haptic->motor_reg = devm_regulator_get(&pdev->dev, "haptic");
	if (IS_ERR(haptic->motor_reg)) {
		dev_err(&pdev->dev, "failed to get regulator\n");
		return PTR_ERR(haptic->motor_reg);
	}

	/* Initialize input device for haptic device */
	haptic->input_dev = devm_input_allocate_device(&pdev->dev);
	if (!haptic->input_dev) {
		dev_err(&pdev->dev, "failed to allocate input device\n");
		return -ENOMEM;
	}

	haptic->input_dev->name = "max77693-haptic";
	haptic->input_dev->id.version = 1;
	haptic->input_dev->dev.parent = &pdev->dev;
	haptic->input_dev->open = max77693_haptic_open;
	haptic->input_dev->close = max77693_haptic_close;
	input_set_drvdata(haptic->input_dev, haptic);
	input_set_capability(haptic->input_dev, EV_FF, FF_RUMBLE);

	error = input_ff_create_memless(haptic->input_dev, NULL,
				max77693_haptic_play_effect);
	if (error) {
		dev_err(&pdev->dev, "failed to create force-feedback\n");
		return error;
	}

	error = input_register_device(haptic->input_dev);
	if (error) {
		dev_err(&pdev->dev, "failed to register input device\n");
		return error;
	}

	platform_set_drvdata(pdev, haptic);

	if (haptic->dev_type == TYPE_MAX77865) {
		unsigned int mconfig = 0, mainctrl1 = 0;

		hrtimer_setup(&haptic->timer, max77865_tout_timer, CLOCK_MONOTONIC,
			      HRTIMER_MODE_REL);

		error = max77865_haptic_init_reg(haptic);
		if (error)
			return dev_err_probe(&pdev->dev, error,
					     "failed to init haptic registers\n");

		regmap_read(haptic->regmap_haptic, MAX77865_PMIC_REG_MCONFIG, &mconfig);
		regmap_read(haptic->regmap_haptic, MAX77865_PMIC_REG_MAINCTRL1, &mainctrl1);
		dev_info(haptic->dev, "max77865: MCONFIG 0x%02x MAINCTRL1 0x%02x\n",
			 mconfig, mainctrl1);

		error = devm_add_action_or_reset(&pdev->dev,
						 max77865_haptic_cleanup_action,
						 haptic);
		if (error)
			return error;

		if (!tout_class_registered) {
			error = class_register(&timed_output_class);
			if (error)
				return dev_err_probe(&pdev->dev, error,
						     "failed to register timed_output class\n");
			tout_class_registered = true;
		}

		haptic->tout_dev = device_create_with_groups(&timed_output_class,
							     NULL, MKDEV(0, 0),
							     haptic,
							     max77865_tout_groups,
							     "vibrator");
		if (IS_ERR(haptic->tout_dev))
			return dev_err_probe(&pdev->dev, PTR_ERR(haptic->tout_dev),
					     "failed to create vibrator tout device\n");

		error = devm_add_action_or_reset(&pdev->dev,
						 max77865_tout_unregister,
						 haptic->tout_dev);
		if (error)
			return error;
	}

	return 0;
}

static int max77693_haptic_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct max77693_haptic *haptic = platform_get_drvdata(pdev);

	if (haptic->dev_type == TYPE_MAX77865) {
		hrtimer_cancel(&haptic->timer);
		cancel_work_sync(&haptic->work);
		haptic->magnitude = 0;
		max77693_haptic_disable(haptic);
		max77865_haptic_boost(haptic, false);
		return 0;
	}

	if (haptic->enabled) {
		max77693_haptic_disable(haptic);
		haptic->suspend_state = true;
	}

	return 0;
}

static int max77693_haptic_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct max77693_haptic *haptic = platform_get_drvdata(pdev);

	if (haptic->dev_type == TYPE_MAX77865) {
		max77865_haptic_init_reg(haptic);
		max77865_haptic_boost(haptic, true);
		return 0;
	}

	if (haptic->suspend_state) {
		max77693_haptic_enable(haptic);
		haptic->suspend_state = false;
	}

	return 0;
}

/* S:drivers/motor/max77865_haptic.c:1037-1043, spec L6F 5.7 */
static void max77693_haptic_shutdown(struct platform_device *pdev)
{
	struct max77693_haptic *haptic = platform_get_drvdata(pdev);

	if (haptic->dev_type == TYPE_MAX77865) {
		hrtimer_cancel(&haptic->timer);
		cancel_work_sync(&haptic->work);
		max77693_haptic_disable(haptic);
		max77865_haptic_boost(haptic, false);
	}
}

static DEFINE_SIMPLE_DEV_PM_OPS(max77693_haptic_pm_ops,
				max77693_haptic_suspend,
				max77693_haptic_resume);

static const struct platform_device_id max77693_haptic_id[] = {
	{ "max77693-haptic", },
	{ "max77705-haptic", },
	{ "max77843-haptic", },
	{ "max77865-haptic", },
	{},
};
MODULE_DEVICE_TABLE(platform, max77693_haptic_id);

static const struct of_device_id of_max77693_haptic_dt_match[] = {
	{ .compatible = "maxim,max77693-haptic", },
	{ .compatible = "maxim,max77705-haptic", },
	{ .compatible = "maxim,max77843-haptic", },
	{ .compatible = "maxim,max77865-haptic", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, of_max77693_haptic_dt_match);

static struct platform_driver max77693_haptic_driver = {
	.driver		= {
		.name	= "max77693-haptic",
		.pm	= pm_sleep_ptr(&max77693_haptic_pm_ops),
		.of_match_table = of_max77693_haptic_dt_match,
	},
	.probe		= max77693_haptic_probe,
	.shutdown	= max77693_haptic_shutdown,
	.id_table	= max77693_haptic_id,
};
module_platform_driver(max77693_haptic_driver);

MODULE_AUTHOR("Jaewon Kim <jaewon02.kim@samsung.com>");
MODULE_AUTHOR("Krzysztof Kozlowski <krzk@kernel.org>");
MODULE_DESCRIPTION("MAXIM 77693/77705/77843/77865 Haptic driver");
MODULE_LICENSE("GPL");
