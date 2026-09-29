/* SPDX-License-Identifier: GPL-2.0 */
/*
 * The Samsung "sec" device class (/sys/class/sec), for the device nodes the
 * LineageOS Samsung HALs use. Same header name and helpers as the stock kernel
 * (S:include/linux/sec_sysfs.h:11-29); spec L6F 5.9.
 */
#ifndef _SEC_SYSFS_H
#define _SEC_SYSFS_H

#include <linux/device.h>

#if IS_ENABLED(CONFIG_SAMSUNG_SEC_CLASS)
struct device *sec_device_create_with_groups(void *drvdata,
					     const struct attribute_group **groups,
					     const char *name);
void sec_device_destroy(struct device *dev);
#else
static inline struct device *sec_device_create_with_groups(void *drvdata,
							   const struct attribute_group **groups,
							   const char *name)
{
	return ERR_PTR(-ENODEV);
}

static inline void sec_device_destroy(struct device *dev)
{
}
#endif

static inline struct device *sec_device_create(void *drvdata, const char *name)
{
	return sec_device_create_with_groups(drvdata, NULL, name);
}

#endif /* _SEC_SYSFS_H */
