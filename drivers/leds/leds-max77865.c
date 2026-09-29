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

/* S:drivers/leds/leds-max77865-rgb.c:538-545, spec L6F 5.9 */
static int max77865_led_reset(struct max77865_led *led)
{
	int ret;

	/* Reset R, G, B to off (bits 2..7 of LEDEN) and LEDRMP := 0 */
	ret = regmap_update_bits(led->regmap, MAX77865_RGBLED_REG_LEDEN,
				 (3 << 2) | (3 << 4) | (3 << 6), 0);
	if (!ret)
		ret = regmap_write(led->regmap, MAX77865_RGBLED_REG_LEDRMP, 0);

	return ret;
}

/* S:drivers/leds/leds-max77865-rgb.c:648-750, spec L6F 5.9 */
static ssize_t led_blink_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct max77865_led *led = dev_get_drvdata(dev);
	int led_brightness = 0, delay_on_time = 0, delay_off_time = 0;
	u32 led_r_brightness = 0, led_g_brightness = 0, led_b_brightness = 0;
	u32 led_total_br = 0, led_max_br = 0;
	u32 max_ratio;
	int ret;

	ret = sscanf(buf, "0x%8x %5d %5d", &led_brightness,
		     &delay_on_time, &delay_off_time);
	if (ret < 3)
		return -EINVAL;

	mutex_lock(&led->lock);

	/* Reset led: R/G/B off + LEDRMP := 0 */
	max77865_led_reset(led);

	led_r_brightness = (led_brightness & 0x00FF0000) >> 16;
	led_g_brightness = (led_brightness & 0x0000FF00) >> 8;
	led_b_brightness = led_brightness & 0x000000FF;

	/* If all 0, reset already turned all off (HAL's off: "0x00000000 0 0") */
	if (!led_r_brightness && !led_g_brightness && !led_b_brightness) {
		mutex_unlock(&led->lock);
		return count;
	}

	/* Scale each nonzero 8-bit colour c = c * normal_current / 0xff (min 1) */
	if (led_r_brightness) {
		led_r_brightness = led_r_brightness * led->normal_current / 0xff;
		if (!led_r_brightness)
			led_r_brightness = 1;
	}
	if (led_g_brightness) {
		led_g_brightness = led_g_brightness * led->normal_current / 0xff;
		if (!led_g_brightness)
			led_g_brightness = 1;
	}
	if (led_b_brightness) {
		led_b_brightness = led_b_brightness * led->normal_current / 0xff;
		if (!led_b_brightness)
			led_b_brightness = 1;
	}

	/* total = r*ratio[0]/100 + g*ratio[1]/100 + b*ratio[2]/100 */
	led_total_br = (led_r_brightness * led->ratio[0] +
			led_g_brightness * led->ratio[1] +
			led_b_brightness * led->ratio[2]) / 100;

	/* max = normal_current * max_ratio / 100 */
	max_ratio = max(led->ratio[0], max(led->ratio[1], led->ratio[2]));
	led_max_br = led->normal_current * max_ratio / 100;

	/* if total > max scale all by max/total (min 1) */
	if (led_total_br > led_max_br && led_total_br > 0) {
		if (led_r_brightness) {
			led_r_brightness = led_r_brightness * led_max_br / led_total_br;
			if (!led_r_brightness)
				led_r_brightness = 1;
		}
		if (led_g_brightness) {
			led_g_brightness = led_g_brightness * led_max_br / led_total_br;
			if (!led_g_brightness)
				led_g_brightness = 1;
		}
		if (led_b_brightness) {
			led_b_brightness = led_b_brightness * led_max_br / led_total_br;
			if (!led_b_brightness)
				led_b_brightness = 1;
		}
	}

	/* per colour set_state(c * ratio / 100 (min 1), BLINK=2) (S:...:218-256) */
	if (led_r_brightness) {
		u32 br = led_r_brightness * led->ratio[0] / 100;
		if (!br)
			br = 1;
		regmap_write(led->regmap, MAX77865_RGBLED_REG_LED0BRT + MAX77865_LED_RED, br);
		regmap_update_bits(led->regmap, MAX77865_RGBLED_REG_LEDEN,
				   3 << (2 * MAX77865_LED_RED),
				   2 << (2 * MAX77865_LED_RED));
	}
	if (led_g_brightness) {
		u32 br = led_g_brightness * led->ratio[1] / 100;
		if (!br)
			br = 1;
		regmap_write(led->regmap, MAX77865_RGBLED_REG_LED0BRT + MAX77865_LED_GREEN, br);
		regmap_update_bits(led->regmap, MAX77865_RGBLED_REG_LEDEN,
				   3 << (2 * MAX77865_LED_GREEN),
				   2 << (2 * MAX77865_LED_GREEN));
	}
	if (led_b_brightness) {
		u32 br = led_b_brightness * led->ratio[2] / 100;
		if (!br)
			br = 1;
		regmap_write(led->regmap, MAX77865_RGBLED_REG_LED0BRT + MAX77865_LED_BLUE, br);
		regmap_update_bits(led->regmap, MAX77865_RGBLED_REG_LEDEN,
				   3 << (2 * MAX77865_LED_BLUE),
				   2 << (2 * MAX77865_LED_BLUE));
	}

	/* LEDBLNK = LEDBLNK_ON(on) << 4 | LEDBLNK_OFF(off) */
	regmap_write(led->regmap, MAX77865_RGBLED_REG_LEDBLNK,
		     (LEDBLNK_ON(delay_on_time) << 4) | LEDBLNK_OFF(delay_off_time));

	mutex_unlock(&led->lock);

	return count;
}
static DEVICE_ATTR(led_blink, 0660, NULL, led_blink_store);

static struct attribute *max77865_led_attrs[] = {
	&dev_attr_led_blink.attr,
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
