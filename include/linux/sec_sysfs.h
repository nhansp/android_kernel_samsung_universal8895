/* SPDX-License-Identifier: GPL-2.0 */
/*
 * The "sec" device class (/sys/class/sec) of the Samsung vendor kernels.
 * Same header name and helpers as those kernels.
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
