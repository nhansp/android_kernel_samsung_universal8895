// SPDX-License-Identifier: GPL-2.0
/*
 * GPS standby (nSTDBY) line of the BCM4774 GNSS/sensor-hub combo, exposed where lhd expects it:
 * /sys/class/sec/gps/GPS_PWR_EN/value (VEN:vendor/etc/sensor/lhd.conf GpioNStdbyPath).
 * Replaces the stock gpio_export()/gpio_export_link() of S:drivers/gps/sec_gps_bcm47531.c:222-352 (L6C 5.4, D-12).
 */
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/kstrtox.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sec_sysfs.h>
#include <linux/sysfs.h>

struct sec_gps_pwr {
	struct gpio_desc *en;
	struct device *gps_dev;
};

static ssize_t value_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sec_gps_pwr *p = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", gpiod_get_value_cansleep(p->en));
}

static ssize_t value_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct sec_gps_pwr *p = dev_get_drvdata(dev);
	bool on;
	int ret = kstrtobool(buf, &on);

	if (ret)
		return ret;
	gpiod_set_value_cansleep(p->en, on);
	return count;
}
static DEVICE_ATTR_RW(value);	/* 0644; init chmods it 0664 (VEN:vendor/etc/init/init.gps.rc:3-4) */

static struct attribute *gps_pwr_en_attrs[] = {
	&dev_attr_value.attr,
	NULL
};

static const struct attribute_group gps_pwr_en_group = {
	.name = "GPS_PWR_EN",
	.attrs = gps_pwr_en_attrs,
};

static const struct attribute_group *sec_gps_pwr_groups[] = {
	&gps_pwr_en_group,
	NULL
};

static void sec_gps_pwr_remove(struct platform_device *pdev)
{
	struct sec_gps_pwr *p = platform_get_drvdata(pdev);

	sec_device_destroy(p->gps_dev);
}

static int sec_gps_pwr_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sec_gps_pwr *p;

	p = devm_kzalloc(dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	/* output low as stock (S:drivers/gps/sec_gps_bcm47531.c:253); pinctrl "default" applied by the core */
	p->en = devm_gpiod_get(dev, NULL, GPIOD_OUT_LOW);
	if (IS_ERR(p->en))
		return dev_err_probe(dev, PTR_ERR(p->en), "GPS_PWR_EN gpio\n");

	/* L6C 5.5: the GPS_PWR_EN group is created with the device, before its uevent */
	p->gps_dev = sec_device_create_with_groups(p, sec_gps_pwr_groups, "gps");
	if (IS_ERR(p->gps_dev))
		return dev_err_probe(dev, PTR_ERR(p->gps_dev), "sec/gps device\n");

	platform_set_drvdata(pdev, p);
	dev_info(dev, "GPS_PWR_EN at /sys/class/sec/gps/GPS_PWR_EN/value\n");
	return 0;
}

static const struct of_device_id sec_gps_pwr_of_match[] = {
	{ .compatible = "samsung,exynos8895-gps-pwr" },	/* L6C 4.2; stock "samsung,exynos54xx-bcm4753" FDT:12534 */
	{ }
};
MODULE_DEVICE_TABLE(of, sec_gps_pwr_of_match);

static struct platform_driver sec_gps_pwr_driver = {
	.probe = sec_gps_pwr_probe,
	.remove = sec_gps_pwr_remove,	/* void remove, A:include/linux/platform_device.h:241 */
	.driver = {
		.name = "sec-gps-pwr",
		.of_match_table = sec_gps_pwr_of_match,
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(sec_gps_pwr_driver);

MODULE_DESCRIPTION("Samsung GPS standby line (GPS_PWR_EN) for lhd");
MODULE_LICENSE("GPL");
