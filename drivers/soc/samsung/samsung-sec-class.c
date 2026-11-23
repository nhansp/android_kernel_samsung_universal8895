// SPDX-License-Identifier: GPL-2.0
/*
 * The "sec" device class (/sys/class/sec) of the Samsung vendor kernels,
 * whose device nodes (tsp, led, ...) the device HALs use.
 */
#include <linux/device.h>
#include <linux/export.h>
#include <linux/init.h>
#include <linux/sec_sysfs.h>

static struct class sec_class = {
	.name = "sec",
};

struct device *sec_device_create_with_groups(void *drvdata,
					     const struct attribute_group **groups,
					     const char *name)
{
	return device_create_with_groups(&sec_class, NULL, MKDEV(0, 0), drvdata,
					 groups, "%s", name);
}
EXPORT_SYMBOL_GPL(sec_device_create_with_groups);

void sec_device_destroy(struct device *dev)
{
	device_unregister(dev);
}
EXPORT_SYMBOL_GPL(sec_device_destroy);

static int __init sec_class_init(void)
{
	return class_register(&sec_class);
}
subsys_initcall(sec_class_init);
