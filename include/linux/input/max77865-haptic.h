/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_INPUT_MAX77865_HAPTIC_H
#define _LINUX_INPUT_MAX77865_HAPTIC_H

#include <linux/errno.h>
#include <linux/kconfig.h>
#include <linux/types.h>

#if IS_REACHABLE(CONFIG_INPUT_MAX77693_HAPTIC)
/*
 * Tick the MAX77865 motor for a pressure home key press (@press) or release,
 * at the intensity of sysfs force_touch_intensity; 0 plays nothing. May sleep.
 */
int max77865_haptic_homekey(bool press);
#else
static inline int max77865_haptic_homekey(bool press)
{
	return -ENODEV;
}
#endif

#endif /* _LINUX_INPUT_MAX77865_HAPTIC_H */
