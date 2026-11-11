// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Maxim MAX77865 RGB LED driver (Samsung Galaxy Note 8, greatlte)
 *
 * Spec rewrite/spec/L6F_power_input_misc.md 5.9;
 * stock drivers/leds/leds-max77865-rgb.c.
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/leds.h>
#include <linux/mfd/max77693-common.h>
#include <linux/mfd/max77865-private.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/sec_sysfs.h>

#define MAX77865_LED_NUM_CHANNELS	4
#define MAX77865_LED_MAX_BRIGHTNESS	0xff

/* S:drivers/leds/leds-max77865-rgb.c:75-83 */
#define LEDBLNK_ON(time)	((time < 100) ? 0 :			\
				(time < 500) ? (time) / 100 - 1 :	\
				(time < 3250) ? ((time) - 500) / 250 + 4 : 15)

#define LEDBLNK_OFF(time)	((time < 1) ? 0x00 :			\
				(time < 500) ? 0x01 :			\
				(time < 5000) ? (time) / 500 :		\
				(time < 8000) ? ((time) - 5000) / 1000 + 10 : \
				(time < 12000) ? ((time) - 8000) / 2000 + 13 : 15)

enum max77865_led_color {
	MAX77865_LED_WHITE = 0,
	MAX77865_LED_RED   = 1,
	MAX77865_LED_GREEN = 2,
	MAX77865_LED_BLUE  = 3,
};

static const char * const max77865_led_names[MAX77865_LED_NUM_CHANNELS] = {
	"led_w",
	"led_r",
	"led_g",
	"led_b",
};

struct max77865_led;

struct max77865_led_channel {
	struct led_classdev cdev;
	struct max77865_led *led;
	unsigned int id;
};

struct max77865_led {
	struct device *dev;
	struct regmap *regmap;
	struct device *sec_dev;
	struct mutex lock;

	u32 normal_current;
	u32 lowpower_current;
	u32 ratio[3];
	u32 ratio_low[3];
	bool lowpower;			/* sec led_lowpower */

	struct max77865_led_channel channels[MAX77865_LED_NUM_CHANNELS];
};

static int max77865_led_brightness_set(struct led_classdev *cdev,
				       enum led_brightness brightness)
{
	struct max77865_led_channel *chan =
		container_of(cdev, struct max77865_led_channel, cdev);
	struct max77865_led *led = chan->led;
	int ret;

	mutex_lock(&led->lock);
	if (brightness == LED_OFF) {
		/* S:drivers/leds/leds-max77865-rgb.c:177-185 */
		ret = regmap_update_bits(led->regmap, MAX77865_RGBLED_REG_LEDEN,
					 3 << (2 * chan->id), 0);
	} else {
		/* S:drivers/leds/leds-max77865-rgb.c:186-193,255-256 */
		ret = regmap_write(led->regmap,
				   MAX77865_RGBLED_REG_LED0BRT + chan->id,
				   brightness);
		if (!ret)
			ret = regmap_update_bits(led->regmap,
						 MAX77865_RGBLED_REG_LEDEN,
						 3 << (2 * chan->id),
						 1 << (2 * chan->id));
	}
	mutex_unlock(&led->lock);

	return ret;
}

#define MAX77865_LED_DISABLE	0
#define MAX77865_LED_ALWAYS_ON	1
#define MAX77865_LED_BLINK	2

/* the canned patterns of Samsung's framework, written to led_pattern */
enum max77865_led_pattern {
	MAX77865_PATTERN_OFF,
	MAX77865_PATTERN_CHARGING,
	MAX77865_PATTERN_CHARGING_ERR,
	MAX77865_PATTERN_MISSED_NOTI,
	MAX77865_PATTERN_LOW_BATTERY,
	MAX77865_PATTERN_FULLY_CHARGED,
	MAX77865_PATTERN_POWERING,
};

/* Samsung's kernel (drivers/leds/leds-max77865-rgb.c), max77865_rgb_ramp() */
static int max77865_led_ramp(struct max77865_led *led, int ramp_up, int ramp_down)
{
	if (ramp_up > 800)
		ramp_up = (ramp_up - 800) * 2 + 800;
	if (ramp_down > 800)
		ramp_down = (ramp_down - 800) * 2 + 800;

	return regmap_write(led->regmap, MAX77865_RGBLED_REG_LEDRMP,
			    (ramp_down / 100) | ((ramp_up / 100) << 4));
}

/* Samsung's kernel, max77865_rgb_blink() */
static int max77865_led_blink(struct max77865_led *led, int on, int off)
{
	return regmap_write(led->regmap, MAX77865_RGBLED_REG_LEDBLNK,
			    (LEDBLNK_ON(on) << 4) | LEDBLNK_OFF(off));
}

/*
 * Samsung's kernel, max77865_rgb_set_state(): the colour's ratio of the active
 * power mode scales the current, and a non-zero current never scales to zero.
 * Samsung's kernel truncates a result above 0xff to the 8-bit register; it is
 * clamped here. Caller holds the lock.
 */
static int max77865_led_set_state(struct max77865_led *led, unsigned int id,
				  u32 brightness, unsigned int state)
{
	int ret;

	if (brightness) {
		const u32 *ratio = led->lowpower ? led->ratio_low : led->ratio;

		brightness = brightness * ratio[id - MAX77865_LED_RED] / 100;
		brightness = clamp(brightness, 1U, (u32)MAX77865_LED_MAX_BRIGHTNESS);
		ret = regmap_write(led->regmap, MAX77865_RGBLED_REG_LED0BRT + id,
				   brightness);
		if (ret)
			return ret;
	} else {
		state = MAX77865_LED_DISABLE;
	}

	return regmap_update_bits(led->regmap, MAX77865_RGBLED_REG_LEDEN,
				  3 << (2 * id), state << (2 * id));
}

/* Samsung's kernel, max77865_rgb_reset(): red, green and blue off, no ramp */
static int max77865_led_reset(struct max77865_led *led)
{
	int ret;

	ret = regmap_update_bits(led->regmap, MAX77865_RGBLED_REG_LEDEN,
				 (3 << (2 * MAX77865_LED_RED)) |
				 (3 << (2 * MAX77865_LED_GREEN)) |
				 (3 << (2 * MAX77865_LED_BLUE)), 0);
	if (!ret)
		ret = max77865_led_ramp(led, 0, 0);

	return ret;
}

static u32 max77865_led_mode_current(struct max77865_led *led)
{
	return led->lowpower ? led->lowpower_current : led->normal_current;
}

/*
 * led_blink, "0xRRGGBB on_ms off_ms" (the LineageOS Samsung lights HAL and
 * Samsung's framework), Samsung's kernel store_max77865_rgb_blink(): each
 * colour is scaled to the current of the power mode, then all of them together
 * down to what the strongest colour may draw at the normal current. A black
 * colour leaves the LED off but still sets the blink timing.
 */
static ssize_t led_blink_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct max77865_led *led = dev_get_drvdata(dev);
	unsigned int rgb, total = 0, max_br, i;
	int on = 0, off = 0;
	u32 cur, c[3];

	if (sscanf(buf, "0x%8x %5d %5d", &rgb, &on, &off) < 1)
		return -EINVAL;

	c[0] = (rgb >> 16) & 0xff;
	c[1] = (rgb >> 8) & 0xff;
	c[2] = rgb & 0xff;

	mutex_lock(&led->lock);

	cur = max77865_led_mode_current(led);
	max77865_led_reset(led);

	for (i = 0; i < 3; i++) {
		if (c[i])
			c[i] = max(c[i] * cur / MAX77865_LED_MAX_BRIGHTNESS, 1U);
		total += c[i] * led->ratio[i] / 100;
	}

	max_br = led->normal_current *
		 max3(led->ratio[0], led->ratio[1], led->ratio[2]) / 100;
	if (total > max_br) {
		for (i = 0; i < 3; i++)
			if (c[i])
				c[i] = max(c[i] * max_br / total, 1U);
	}

	for (i = 0; i < 3; i++)
		if (c[i])
			max77865_led_set_state(led, MAX77865_LED_RED + i, c[i],
					       MAX77865_LED_BLINK);

	max77865_led_blink(led, on, off);

	mutex_unlock(&led->lock);

	return count;
}
static DEVICE_ATTR(led_blink, 0660, NULL, led_blink_store);

/*
 * led_r, led_g, led_b, Samsung's kernel store_led_r() and friends: one colour
 * always on at the given level (scaled by the ratio of the power mode), 0 off.
 */
static ssize_t max77865_led_color_store(struct max77865_led *led, unsigned int id,
					const char *buf, size_t count)
{
	unsigned int brightness;
	int ret;

	ret = kstrtouint(buf, 0, &brightness);
	if (ret)
		return ret;

	brightness = min_t(unsigned int, brightness, MAX77865_LED_MAX_BRIGHTNESS);

	mutex_lock(&led->lock);
	ret = max77865_led_set_state(led, id, brightness, MAX77865_LED_ALWAYS_ON);
	mutex_unlock(&led->lock);

	return ret ?: count;
}

#define MAX77865_LED_COLOR_ATTR(_name, _id)					\
static ssize_t _name##_store(struct device *dev,				\
			     struct device_attribute *attr,			\
			     const char *buf, size_t count)			\
{										\
	return max77865_led_color_store(dev_get_drvdata(dev), _id, buf, count);\
}										\
static DEVICE_ATTR(_name, 0660, NULL, _name##_store)

MAX77865_LED_COLOR_ATTR(led_r, MAX77865_LED_RED);
MAX77865_LED_COLOR_ATTR(led_g, MAX77865_LED_GREEN);
MAX77865_LED_COLOR_ATTR(led_b, MAX77865_LED_BLUE);

/* led_pattern, Samsung's kernel store_max77865_rgb_pattern() */
static ssize_t led_pattern_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct max77865_led *led = dev_get_drvdata(dev);
	unsigned int mode;
	u32 cur;

	if (sscanf(buf, "%1u", &mode) != 1)
		return -EINVAL;

	mutex_lock(&led->lock);

	max77865_led_reset(led);
	cur = max77865_led_mode_current(led);

	switch (mode) {
	case MAX77865_PATTERN_CHARGING:
		max77865_led_set_state(led, MAX77865_LED_RED, cur, MAX77865_LED_ALWAYS_ON);
		break;
	case MAX77865_PATTERN_CHARGING_ERR:
		max77865_led_blink(led, 500, 500);
		max77865_led_set_state(led, MAX77865_LED_RED, cur, MAX77865_LED_BLINK);
		break;
	case MAX77865_PATTERN_MISSED_NOTI:
		max77865_led_blink(led, 500, 5000);
		max77865_led_set_state(led, MAX77865_LED_BLUE, cur, MAX77865_LED_BLINK);
		break;
	case MAX77865_PATTERN_LOW_BATTERY:
		max77865_led_blink(led, 500, 5000);
		max77865_led_set_state(led, MAX77865_LED_RED, cur, MAX77865_LED_BLINK);
		break;
	case MAX77865_PATTERN_FULLY_CHARGED:
		max77865_led_set_state(led, MAX77865_LED_GREEN, cur, MAX77865_LED_ALWAYS_ON);
		break;
	case MAX77865_PATTERN_POWERING:
		max77865_led_ramp(led, 800, 800);
		max77865_led_blink(led, 200, 200);
		max77865_led_set_state(led, MAX77865_LED_BLUE, cur, MAX77865_LED_ALWAYS_ON);
		max77865_led_set_state(led, MAX77865_LED_GREEN, cur, MAX77865_LED_BLINK);
		break;
	default:
		break;
	}

	mutex_unlock(&led->lock);

	return count;
}
static DEVICE_ATTR(led_pattern, 0660, NULL, led_pattern_store);

/*
 * led_lowpower, Samsung's kernel store_max77865_rgb_lowpower(): 1 selects the
 * low-power current and ratios for what is set next.
 */
static ssize_t led_lowpower_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct max77865_led *led = dev_get_drvdata(dev);
	u8 lowpower;
	int ret;

	ret = kstrtou8(buf, 0, &lowpower);
	if (ret)
		return ret;

	mutex_lock(&led->lock);
	led->lowpower = lowpower == 1;
	mutex_unlock(&led->lock);

	return count;
}
static DEVICE_ATTR(led_lowpower, 0660, NULL, led_lowpower_store);

/*
 * led_brightness, Samsung's kernel store_max77865_rgb_brightness(): leaves the
 * low-power mode. The level it also stores is read only by Samsung's per-LED
 * "blink" attribute, which this driver does not have: led_blink and
 * led_pattern take the current from the power mode, as in Samsung's kernel.
 */
static ssize_t led_brightness_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct max77865_led *led = dev_get_drvdata(dev);
	u8 brightness;
	int ret;

	ret = kstrtou8(buf, 0, &brightness);
	if (ret)
		return ret;

	mutex_lock(&led->lock);
	led->lowpower = false;
	mutex_unlock(&led->lock);

	return count;
}
static DEVICE_ATTR(led_brightness, 0660, NULL, led_brightness_store);

/* Samsung's kernel, sec_led_attributes */
static struct attribute *max77865_led_attrs[] = {
	&dev_attr_led_r.attr,
	&dev_attr_led_g.attr,
	&dev_attr_led_b.attr,
	&dev_attr_led_pattern.attr,
	&dev_attr_led_blink.attr,
	&dev_attr_led_brightness.attr,
	&dev_attr_led_lowpower.attr,
	NULL,
};
ATTRIBUTE_GROUPS(max77865_led);

static void max77865_led_destroy_action(void *data)
{
	struct max77865_led *led = data;

	sec_device_destroy(led->sec_dev);
	max77865_led_reset(led);
}

static int max77865_led_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct max77693_dev *max = dev_get_drvdata(dev->parent);
	struct max77865_led *led;
	int ret, i;

	led = devm_kzalloc(dev, sizeof(*led), GFP_KERNEL);
	if (!led)
		return -ENOMEM;

	led->dev = dev;
	led->regmap = max->regmap;
	mutex_init(&led->lock);

	/* DT constants of spec L6F 4.3 */
	led->normal_current = 0x0a;
	device_property_read_u32(dev, "maxim,normal-current", &led->normal_current);

	led->lowpower_current = 0x0a;
	device_property_read_u32(dev, "maxim,lowpower-current", &led->lowpower_current);

	led->ratio[0] = 0x2d0;
	led->ratio[1] = 0x46;
	led->ratio[2] = 0x96;
	device_property_read_u32_array(dev, "maxim,brightness-ratio", led->ratio, 3);

	led->ratio_low[0] = 0x5a;
	led->ratio_low[1] = 0x0a;
	led->ratio_low[2] = 0x14;
	device_property_read_u32_array(dev, "maxim,brightness-ratio-low", led->ratio_low, 3);

	max77865_led_reset(led);

	for (i = 0; i < MAX77865_LED_NUM_CHANNELS; i++) {
		led->channels[i].id = i;
		led->channels[i].led = led;
		led->channels[i].cdev.name = max77865_led_names[i];
		led->channels[i].cdev.max_brightness = MAX77865_LED_MAX_BRIGHTNESS;
		led->channels[i].cdev.brightness_set_blocking = max77865_led_brightness_set;

		ret = devm_led_classdev_register(dev, &led->channels[i].cdev);
		if (ret)
			return dev_err_probe(dev, ret, "failed to register led %s\n",
					     max77865_led_names[i]);
	}

	led->sec_dev = sec_device_create_with_groups(led, max77865_led_groups, "led");
	if (IS_ERR(led->sec_dev))
		return dev_err_probe(dev, PTR_ERR(led->sec_dev),
				     "failed to create sec led device\n");

	ret = devm_add_action_or_reset(dev, max77865_led_destroy_action, led);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, led);

	return 0;
}

static const struct of_device_id of_max77865_led_match[] = {
	{ .compatible = "maxim,max77865-rgb" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_max77865_led_match);

static const struct platform_device_id max77865_led_id[] = {
	{ "max77865-rgb", 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(platform, max77865_led_id);

static struct platform_driver max77865_led_driver = {
	.driver = {
		.name = "max77865-rgb",
		.of_match_table = of_max77865_led_match,
	},
	.probe = max77865_led_probe,
	.id_table = max77865_led_id,
};
module_platform_driver(max77865_led_driver);

MODULE_DESCRIPTION("Maxim MAX77865 RGB LED driver");
MODULE_LICENSE("GPL");
