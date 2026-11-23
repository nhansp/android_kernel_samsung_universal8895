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
#include <linux/input/max77865-haptic.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
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

#define MAX77865_MCONFIG_DIVIDER_128	BIT(1)
#define MAX77865_MCONFIG_MOTOR_EN	BIT(6)
#define MAX77865_MCONFIG_MOTOR_LRA	BIT(7)

/*
 * The pressure home key tick of the vendor driver: entries 5 (press) and 6
 * (release) of the per-frequency duty and period tables, for 7 ms, at an
 * intensity of -10000..10000.
 */
#define MAX77865_HOMEKEY_PRESS_FREQ	5
#define MAX77865_HOMEKEY_RELEASE_FREQ	6
#define MAX77865_HOMEKEY_DURATION_MS	7
#define MAX77865_MAX_INTENSITY		10000
#define MAX77865_MULTI_FREQ		7

/*
 * multi_freq in 0.1 Hz: the PWM runs at 128 times the drive frequency
 * (DIVIDER_128), period 78125000 / f ns, duty 79 % without overdrive.
 */
#define MAX77865_FREQ_MIN_DHZ		1200
#define MAX77865_FREQ_MAX_DHZ		3500
#define MAX77865_FREQ_PERIOD_NUM	78125000U
#define MAX77865_FREQ_DUTY_PCT		79
#define MAX77865_FREQ_OD_DUTY_PCT	95

#define MAX77865_PACKET_MAX		1000

struct max77865_packet {
	int time_ms;
	int intensity;
	int freq;
	int overdrive;
};

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

	/* MAX77865 */
	u32 period_ns;
	u32 max_duty_ns;
	int freq_num;
	u32 max_timeout_ms;
	unsigned int duty_ns;
	unsigned int intensity;
	struct hrtimer timer;
	struct device *tout_dev;

	/*
	 * The home key tick. lock covers magnitude and the tick state, which
	 * the timers, the force-feedback callback and the sysfs nodes change.
	 */
	spinlock_t lock;
	int force_touch_intensity;
	u32 freq_duty_ns[MAX77865_MULTI_FREQ];
	u32 freq_period_ns[MAX77865_MULTI_FREQ];
	bool has_freq;
	bool tick;			/* the motor plays a tick */
	bool tick_init;			/* set the motor up before the tick */
	bool tick_pending;		/* a tick asked for while suspended */
	bool suspended;
	unsigned int tick_period_ns;
	unsigned int tick_duty_ns;
	unsigned int tick_seq;		/* the tick asked for last */
	unsigned int tick_timed;	/* the tick whose duration runs */
	struct hrtimer tick_timer;

	/*
	 * haptic_engine: pkt_lock covers the packet array; lock covers the
	 * run state, which the timer advances.
	 */
	struct mutex pkt_lock;
	struct max77865_packet *pkt;
	unsigned int pkt_count;
	unsigned int pkt_idx;
	bool pkt_loaded;
	bool pkt_running;
	bool pkt_step;
	bool pkt_applied;
	bool overdrive;

	struct work_struct work;
};

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
				 MAX77865_MCONFIG_MOTOR_EN, 0);
	if (ret)
		return ret;

	ret = regmap_update_bits(map, MAX77865_PMIC_REG_MCONFIG,
				 MAX77865_MCONFIG_MOTOR_LRA,
				 MAX77865_MCONFIG_MOTOR_LRA);
	if (ret)
		return ret;

	ret = regmap_update_bits(map, MAX77865_PMIC_REG_MCONFIG,
				 MAX77865_MCONFIG_DIVIDER_128,
				 MAX77865_MCONFIG_DIVIDER_128);
	if (ret)
		return ret;

	return 0;
}

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

/* +-10000 drives the full duty or its mirror image, 0 the neutral half period */
static unsigned int max77865_haptic_calc_duty(unsigned int period,
					      unsigned int max_duty, int intensity)
{
	int half = period / 2;

	return half + ((int)max_duty - half) * intensity / MAX77865_MAX_INTENSITY;
}

static unsigned int max77865_haptic_duty(struct max77693_haptic *haptic, int intensity)
{
	return max77865_haptic_calc_duty(haptic->period_ns, haptic->max_duty_ns, intensity);
}

/* @num is an index into the DT tables or a frequency in 0.1 Hz */
static int max77865_haptic_freq(struct max77693_haptic *haptic, int num, bool overdrive,
				u32 *period, u32 *max_duty)
{
	if (haptic->has_freq && num >= 0 && num < MAX77865_MULTI_FREQ) {
		*period = haptic->freq_period_ns[num];
		*max_duty = haptic->freq_duty_ns[num];
	} else if (num >= MAX77865_FREQ_MIN_DHZ && num <= MAX77865_FREQ_MAX_DHZ) {
		*period = MAX77865_FREQ_PERIOD_NUM / num;
		*max_duty = *period * (overdrive ? MAX77865_FREQ_OD_DUTY_PCT :
						   MAX77865_FREQ_DUTY_PCT) / 100;
	} else {
		return -EINVAL;
	}

	return 0;
}

/* the PWM period and duty: the tick's while it plays */
static void max77865_haptic_pwm_state(struct max77693_haptic *haptic,
				      struct pwm_state *state)
{
	unsigned long flags;

	spin_lock_irqsave(&haptic->lock, flags);
	if (haptic->tick) {
		state->period = haptic->tick_period_ns;
		state->duty_cycle = haptic->tick_duty_ns;
	} else {
		state->period = haptic->period_ns;
		state->duty_cycle = haptic->duty_ns;
	}
	spin_unlock_irqrestore(&haptic->lock, flags);
}

static int max77693_haptic_set_duty_cycle(struct max77693_haptic *haptic)
{
	struct pwm_state state;
	int error;

	pwm_init_state(haptic->pwm_dev, &state);
	if (haptic->dev_type == TYPE_MAX77865)
		max77865_haptic_pwm_state(haptic, &state);
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
		return regmap_update_bits(haptic->regmap_haptic,
					  MAX77865_PMIC_REG_MCONFIG,
					  MAX77865_MCONFIG_MOTOR_EN,
					  enable ? MAX77865_MCONFIG_MOTOR_EN : 0);
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
		max77865_haptic_pwm_state(haptic, &state);
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

	WRITE_ONCE(haptic->enabled, true);

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
	WRITE_ONCE(haptic->enabled, false);

	return;

err_disable_lowsys:
	max77693_haptic_configure(haptic, true);
}

/*
 * One haptic_engine step: the next packet's frequency, overdrive and intensity
 * for its time. Intensity 0 rests the motor while the list runs on, and a
 * frequency out of range keeps the previous one. False when no packet runs.
 */
static bool max77865_engine_step(struct max77693_haptic *haptic, int *time_ms)
{
	struct max77865_packet p;
	unsigned long flags;
	u32 period, max_duty;
	bool run, freq_ok;

	mutex_lock(&haptic->pkt_lock);
	spin_lock_irqsave(&haptic->lock, flags);
	run = haptic->pkt_running;
	if (run && haptic->pkt_step) {
		haptic->pkt_step = false;
		haptic->pkt_idx++;
	} else if (run && haptic->pkt_applied) {
		/* woken for something else: the current packet keeps its time */
		spin_unlock_irqrestore(&haptic->lock, flags);
		mutex_unlock(&haptic->pkt_lock);
		return false;
	}
	haptic->pkt_applied = run;
	if (run && haptic->pkt_idx >= haptic->pkt_count) {
		haptic->pkt_running = false;
		haptic->pkt_loaded = false;
		haptic->pkt_count = 0;
		haptic->magnitude = 0;
		run = false;
	}
	if (run)
		p = haptic->pkt[haptic->pkt_idx];
	spin_unlock_irqrestore(&haptic->lock, flags);
	mutex_unlock(&haptic->pkt_lock);

	if (!run)
		return false;

	freq_ok = !max77865_haptic_freq(haptic, p.freq, p.overdrive, &period, &max_duty);

	spin_lock_irqsave(&haptic->lock, flags);
	haptic->overdrive = p.overdrive;
	if (freq_ok) {
		haptic->period_ns = period;
		haptic->max_duty_ns = max_duty;
		haptic->freq_num = p.freq;
	}
	haptic->duty_ns = max77865_haptic_calc_duty(haptic->period_ns, haptic->max_duty_ns,
						    p.intensity);
	haptic->magnitude = p.intensity != 0;
	spin_unlock_irqrestore(&haptic->lock, flags);

	*time_ms = p.time_ms;
	return true;
}

static void max77693_haptic_play_work(struct work_struct *work)
{
	struct max77693_haptic *haptic =
			container_of(work, struct max77693_haptic, work);
	unsigned long flags;
	unsigned int magnitude, tick_seq;
	bool tick, tick_init, engine = false;
	int time_ms = 0;

	if (haptic->dev_type == TYPE_MAX77865)
		engine = max77865_engine_step(haptic, &time_ms);

	spin_lock_irqsave(&haptic->lock, flags);
	magnitude = haptic->magnitude;
	tick = haptic->tick;
	tick_seq = haptic->tick_seq;
	tick_init = haptic->tick_init;
	haptic->tick_init = false;
	spin_unlock_irqrestore(&haptic->lock, flags);

	/* as in the vendor driver, the motor is set up again before a tick */
	if (magnitude && tick_init && !haptic->enabled) {
		max77865_haptic_init_reg(haptic);
		max77865_haptic_boost(haptic, true);
	}

	if (!magnitude)
		max77693_haptic_disable(haptic);
	else if (haptic->enabled)
		max77693_haptic_set_duty_cycle(haptic);
	else
		max77693_haptic_enable(haptic);

	if (engine) {
		/* an enable or a new packet list may have stopped the run meanwhile */
		spin_lock_irqsave(&haptic->lock, flags);
		if (haptic->pkt_running)
			hrtimer_start(&haptic->timer, ms_to_ktime(time_ms), HRTIMER_MODE_REL);
		spin_unlock_irqrestore(&haptic->lock, flags);
		return;
	}

	if (!tick)
		return;

	/*
	 * As in the vendor driver, the 7 ms start once the motor runs at the
	 * tick's period and duty. A newer tick is timed by its own run of this
	 * work, and a vibration that took over is not timed.
	 */
	spin_lock_irqsave(&haptic->lock, flags);
	if (haptic->tick && haptic->tick_seq == tick_seq &&
	    haptic->tick_timed != tick_seq) {
		haptic->tick_timed = tick_seq;
		hrtimer_start(&haptic->tick_timer,
			      ms_to_ktime(MAX77865_HOMEKEY_DURATION_MS),
			      HRTIMER_MODE_REL);
	}
	spin_unlock_irqrestore(&haptic->lock, flags);
}

static int max77693_haptic_play_effect(struct input_dev *dev, void *data,
				       struct ff_effect *effect)
{
	struct max77693_haptic *haptic = input_get_drvdata(dev);
	struct pwm_args pargs;
	u64 period_mag_multi;
	unsigned long flags;

	spin_lock_irqsave(&haptic->lock, flags);
	haptic->tick = false;
	haptic->pkt_running = false;
	haptic->magnitude = effect->u.rumble.strong_magnitude;
	if (!haptic->magnitude)
		haptic->magnitude = effect->u.rumble.weak_magnitude;
	spin_unlock_irqrestore(&haptic->lock, flags);

	/*
	 * The magnitude comes from force-feedback interface.
	 * For MAX77865, map 0..0xffff to 0..10000 intensity.
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

static enum hrtimer_restart max77865_tout_timer(struct hrtimer *timer)
{
	struct max77693_haptic *haptic =
		container_of(timer, struct max77693_haptic, timer);
	unsigned long flags;

	spin_lock_irqsave(&haptic->lock, flags);
	if (haptic->pkt_running)
		haptic->pkt_step = true;
	else if (!haptic->tick)
		haptic->magnitude = 0;
	spin_unlock_irqrestore(&haptic->lock, flags);
	schedule_work(&haptic->work);
	return HRTIMER_NORESTART;
}

/*
 * end of the home key tick, unless a vibration took the motor over or a newer
 * tick has not started yet
 */
static enum hrtimer_restart max77865_tick_timer(struct hrtimer *timer)
{
	struct max77693_haptic *haptic =
		container_of(timer, struct max77693_haptic, tick_timer);
	unsigned long flags;

	spin_lock_irqsave(&haptic->lock, flags);
	if (haptic->tick && haptic->tick_timed == haptic->tick_seq) {
		haptic->tick = false;
		haptic->magnitude = 0;
		schedule_work(&haptic->work);
	}
	spin_unlock_irqrestore(&haptic->lock, flags);
	return HRTIMER_NORESTART;
}

/* timed_output "enable": vibrate for the given milliseconds */
static ssize_t enable_store(struct device *dev,
			    struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct max77693_haptic *haptic = dev_get_drvdata(dev);
	unsigned long flags;
	int ms, error;
	bool engine;

	error = kstrtoint(buf, 0, &ms);
	if (error)
		return -EINVAL;

	spin_lock_irqsave(&haptic->lock, flags);
	haptic->pkt_running = false;
	haptic->pkt_step = false;
	spin_unlock_irqrestore(&haptic->lock, flags);
	hrtimer_cancel(&haptic->timer);
	ms = min_t(int, ms, (int)haptic->max_timeout_ms);

	/* loaded packets play on any time > 0, 0 drops them */
	spin_lock_irqsave(&haptic->lock, flags);
	haptic->tick = false;
	engine = ms > 0 && haptic->pkt_loaded;
	if (engine) {
		haptic->pkt_running = true;
		haptic->pkt_idx = 0;
		haptic->pkt_applied = false;
	} else {
		haptic->magnitude = ms > 0;
		if (ms <= 0) {
			haptic->pkt_loaded = false;
			haptic->pkt_count = 0;
		}
	}
	spin_unlock_irqrestore(&haptic->lock, flags);

	schedule_work(&haptic->work);
	if (ms > 0 && !engine)
		hrtimer_start(&haptic->timer, ms_to_ktime(ms), HRTIMER_MODE_REL);

	return count;
}

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

/* a table index, or a frequency in 0.1 Hz that also sets the intensity to full */
static ssize_t multi_freq_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct max77693_haptic *haptic = dev_get_drvdata(dev);
	u32 period, max_duty;
	unsigned long flags;
	int num, error;

	error = kstrtoint(buf, 0, &num);
	if (error)
		return -EINVAL;

	error = max77865_haptic_freq(haptic, num, READ_ONCE(haptic->overdrive), &period,
				     &max_duty);
	if (error)
		return error;

	spin_lock_irqsave(&haptic->lock, flags);
	haptic->period_ns = period;
	haptic->max_duty_ns = max_duty;
	haptic->freq_num = num;
	if (num >= MAX77865_FREQ_MIN_DHZ)
		haptic->intensity = MAX77865_MAX_INTENSITY;
	haptic->duty_ns = max77865_haptic_duty(haptic, haptic->intensity);
	spin_unlock_irqrestore(&haptic->lock, flags);

	if (READ_ONCE(haptic->enabled))
		schedule_work(&haptic->work);

	return count;
}

static ssize_t multi_freq_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct max77693_haptic *haptic = dev_get_drvdata(dev);

	return sysfs_emit(buf, "frequency: %d\n", READ_ONCE(haptic->freq_num));
}
static DEVICE_ATTR(multi_freq, 0660, multi_freq_show, multi_freq_store);

/*
 * "<4n> t i f o ..." loads n packets of time (ms), intensity (-10000..10000),
 * frequency (as multi_freq) and overdrive; the next enable > 0 plays them,
 * enable 0 drops them.
 */
static ssize_t haptic_engine_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct max77693_haptic *haptic = dev_get_drvdata(dev);
	struct max77865_packet *p;
	char *str, *cur, *tok;
	unsigned long flags;
	int v[4], words, n, i, j, ret = 0;

	str = kstrndup(buf, count, GFP_KERNEL);
	if (!str)
		return -ENOMEM;
	cur = strim(str);

	tok = strsep(&cur, " \t\n");
	if (!tok || kstrtoint(tok, 0, &words) || words <= 0 || words % 4 ||
	    words > MAX77865_PACKET_MAX * 4) {
		ret = -EINVAL;
		goto out;
	}
	n = words / 4;

	spin_lock_irqsave(&haptic->lock, flags);
	haptic->pkt_running = false;
	haptic->pkt_step = false;
	haptic->pkt_loaded = false;
	haptic->magnitude = 0;
	spin_unlock_irqrestore(&haptic->lock, flags);
	hrtimer_cancel(&haptic->timer);

	mutex_lock(&haptic->pkt_lock);
	for (i = 0; i < n && !ret; i++) {
		for (j = 0; j < 4; j++) {
			do {
				tok = strsep(&cur, " \t\n");
			} while (tok && !*tok);
			if (!tok || kstrtoint(tok, 0, &v[j])) {
				ret = -EINVAL;
				break;
			}
		}
		if (ret)
			break;
		if (v[0] < 0 || v[1] < -MAX77865_MAX_INTENSITY || v[1] > MAX77865_MAX_INTENSITY) {
			ret = -EINVAL;
			break;
		}
		p = &haptic->pkt[i];
		p->time_ms = v[0];
		p->intensity = v[1];
		p->freq = v[2];
		p->overdrive = !!v[3];
	}
	if (!ret) {
		spin_lock_irqsave(&haptic->lock, flags);
		haptic->pkt_count = n;
		haptic->pkt_loaded = true;
		spin_unlock_irqrestore(&haptic->lock, flags);
	}
	mutex_unlock(&haptic->pkt_lock);

	schedule_work(&haptic->work);
out:
	kfree(str);
	return ret ? ret : count;
}

static ssize_t haptic_engine_show(struct device *dev, struct device_attribute *attr,
				  char *buf)
{
	struct max77693_haptic *haptic = dev_get_drvdata(dev);
	const struct max77865_packet *p;
	unsigned int i;
	int len = 0;

	mutex_lock(&haptic->pkt_lock);
	for (i = 0; i < READ_ONCE(haptic->pkt_count) && READ_ONCE(haptic->pkt_loaded); i++) {
		p = &haptic->pkt[i];
		len += sysfs_emit_at(buf, len, "%d,%d,%d,%d,", p->time_ms, p->intensity,
				     p->freq, p->overdrive);
	}
	mutex_unlock(&haptic->pkt_lock);
	len += sysfs_emit_at(buf, len, "\n");

	return len;
}
static DEVICE_ATTR(haptic_engine, 0660, haptic_engine_show, haptic_engine_store);

/*
 * force_touch_intensity: the intensity of the home key tick, -10000..10000;
 * 0, the default, plays none. The vendor node also programs the PWM at once,
 * with the motor off; here the value is only stored for the next tick.
 */
static ssize_t force_touch_intensity_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	struct max77693_haptic *haptic = dev_get_drvdata(dev);
	int i, error;

	error = kstrtoint(buf, 0, &i);
	if (error)
		return -EINVAL;

	if (i < -MAX77865_MAX_INTENSITY || i > MAX77865_MAX_INTENSITY)
		return -EINVAL;

	WRITE_ONCE(haptic->force_touch_intensity, i);

	return count;
}

static ssize_t force_touch_intensity_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct max77693_haptic *haptic = dev_get_drvdata(dev);

	return sysfs_emit(buf, "force touch intensity: %d\n",
			  READ_ONCE(haptic->force_touch_intensity));
}
static DEVICE_ATTR(force_touch_intensity, 0660, force_touch_intensity_show,
		   force_touch_intensity_store);

static struct attribute *max77865_tout_attrs[] = {
	&dev_attr_enable.attr,
	&dev_attr_intensity.attr,
	&dev_attr_force_touch_intensity.attr,
	&dev_attr_multi_freq.attr,
	&dev_attr_haptic_engine.attr,
	NULL,
};
ATTRIBUTE_GROUPS(max77865_tout);

static DEFINE_MUTEX(max77865_homekey_lock);
static struct max77693_haptic *max77865_homekey_haptic;

/*
 * A vibration in progress is left alone, where the vendor driver's tick cut it
 * short; while the system is suspended the tick waits for the resume of this
 * device, which sets the motor up again.
 */
int max77865_haptic_homekey(bool press)
{
	struct max77693_haptic *haptic;
	unsigned int idx = press ? MAX77865_HOMEKEY_PRESS_FREQ : MAX77865_HOMEKEY_RELEASE_FREQ;
	unsigned long flags;
	int intensity, ret = 0;
	bool play = false;

	mutex_lock(&max77865_homekey_lock);

	haptic = max77865_homekey_haptic;
	if (!haptic || !haptic->has_freq) {
		ret = -ENODEV;
		goto out;
	}

	intensity = READ_ONCE(haptic->force_touch_intensity);
	if (!intensity)
		goto out;

	spin_lock_irqsave(&haptic->lock, flags);
	if (haptic->tick || (!haptic->magnitude && !READ_ONCE(haptic->enabled))) {
		haptic->tick_period_ns = haptic->freq_period_ns[idx];
		haptic->tick_duty_ns = max77865_haptic_calc_duty(haptic->tick_period_ns,
								 haptic->freq_duty_ns[idx],
								 intensity);
		if (haptic->suspended) {
			haptic->tick_pending = true;
		} else {
			haptic->tick = true;
			haptic->tick_seq++;
			haptic->tick_init |= press;
			haptic->magnitude = 1;
			play = true;
		}
	}
	spin_unlock_irqrestore(&haptic->lock, flags);

	/* the work sets the motor up, starts it and then times the tick */
	if (play)
		schedule_work(&haptic->work);
out:
	mutex_unlock(&max77865_homekey_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(max77865_haptic_homekey);

static void max77865_homekey_unregister(void *data)
{
	mutex_lock(&max77865_homekey_lock);
	max77865_homekey_haptic = NULL;
	mutex_unlock(&max77865_homekey_lock);
}

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
	unsigned long flags;

	/* the work may not time a tick any more */
	spin_lock_irqsave(&haptic->lock, flags);
	haptic->tick = false;
	spin_unlock_irqrestore(&haptic->lock, flags);

	hrtimer_cancel(&haptic->timer);
	hrtimer_cancel(&haptic->tick_timer);
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
	spin_lock_init(&haptic->lock);

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

		/* the vendor driver's defaults */
		haptic->intensity = 8000;
		haptic->duty_ns = haptic->max_duty_ns;

		/* without the tables there is no home key tick */
		haptic->has_freq =
			!device_property_read_u32_array(&pdev->dev, "maxim,multi-freq-duty-ns",
							haptic->freq_duty_ns,
							MAX77865_MULTI_FREQ) &&
			!device_property_read_u32_array(&pdev->dev, "maxim,multi-freq-period-ns",
							haptic->freq_period_ns,
							MAX77865_MULTI_FREQ);
	}

	INIT_WORK(&haptic->work, max77693_haptic_play_work);

	/* Get pwm and regulatot for haptic device */
	haptic->pwm_dev = devm_pwm_get(&pdev->dev, NULL);
	if (IS_ERR(haptic->pwm_dev)) {
		dev_err(&pdev->dev, "failed to get pwm device\n");
		return PTR_ERR(haptic->pwm_dev);
	}

	if (haptic->dev_type == TYPE_MAX77865) {
		struct pwm_args pargs;

		pwm_get_args(haptic->pwm_dev, &pargs);
		haptic->period_ns = pargs.period;

		haptic->pkt = devm_kcalloc(&pdev->dev, MAX77865_PACKET_MAX,
					   sizeof(*haptic->pkt), GFP_KERNEL);
		if (!haptic->pkt)
			return -ENOMEM;
		error = devm_mutex_init(&pdev->dev, &haptic->pkt_lock);
		if (error)
			return error;
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
		hrtimer_setup(&haptic->timer, max77865_tout_timer, CLOCK_MONOTONIC,
			      HRTIMER_MODE_REL);
		hrtimer_setup(&haptic->tick_timer, max77865_tick_timer, CLOCK_MONOTONIC,
			      HRTIMER_MODE_REL);

		error = max77865_haptic_init_reg(haptic);
		if (error)
			return dev_err_probe(&pdev->dev, error,
					     "failed to init haptic registers\n");

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

		mutex_lock(&max77865_homekey_lock);
		max77865_homekey_haptic = haptic;
		mutex_unlock(&max77865_homekey_lock);
		error = devm_add_action_or_reset(&pdev->dev,
						 max77865_homekey_unregister, NULL);
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
		unsigned long flags;

		spin_lock_irqsave(&haptic->lock, flags);
		haptic->suspended = true;
		haptic->tick = false;
		haptic->tick_pending = false;
		haptic->magnitude = 0;
		spin_unlock_irqrestore(&haptic->lock, flags);

		hrtimer_cancel(&haptic->timer);
		hrtimer_cancel(&haptic->tick_timer);
		cancel_work_sync(&haptic->work);
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
		unsigned long flags;
		bool tick;

		max77865_haptic_init_reg(haptic);
		max77865_haptic_boost(haptic, true);

		/* a home key press woke the system: its tick plays now */
		spin_lock_irqsave(&haptic->lock, flags);
		haptic->suspended = false;
		tick = haptic->tick_pending;
		haptic->tick_pending = false;
		if (tick) {
			haptic->tick = true;
			haptic->tick_seq++;
			haptic->magnitude = 1;
		}
		spin_unlock_irqrestore(&haptic->lock, flags);

		if (tick)
			schedule_work(&haptic->work);
		return 0;
	}

	if (haptic->suspend_state) {
		max77693_haptic_enable(haptic);
		haptic->suspend_state = false;
	}

	return 0;
}

static void max77693_haptic_shutdown(struct platform_device *pdev)
{
	struct max77693_haptic *haptic = platform_get_drvdata(pdev);

	if (haptic->dev_type == TYPE_MAX77865) {
		unsigned long flags;

		/* no tick from here on */
		spin_lock_irqsave(&haptic->lock, flags);
		haptic->suspended = true;
		haptic->tick = false;
		haptic->tick_pending = false;
		spin_unlock_irqrestore(&haptic->lock, flags);

		hrtimer_cancel(&haptic->timer);
		hrtimer_cancel(&haptic->tick_timer);
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
