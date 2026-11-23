/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Backlight and lcd class devices of the S6E3HA6 panel and its brightness
 * sequence.
 */

#ifndef __S6E3HA6_BRIGHT_H__
#define __S6E3HA6_BRIGHT_H__

#include <linux/backlight.h>
#include <linux/device.h>
#include <linux/lcd.h>
#include <linux/mutex.h>
#include <linux/types.h>

#include <drm/drm_mipi_dsi.h>

#define S6E3HA6_GAMMA_CMD_CNT	36

struct s6e3ha6_bright;

struct s6e3ha6_bright *s6e3ha6_bright_register(struct device *dev,
					       struct mipi_dsi_device *dsi,
					       const char *variant);
int s6e3ha6_bright_register_lcd(struct s6e3ha6_bright *b);
void s6e3ha6_bright_unregister(struct s6e3ha6_bright *b);
int s6e3ha6_bright_prepare(struct s6e3ha6_bright *b);
int s6e3ha6_bright_enable(struct s6e3ha6_bright *b);

#endif	/* __S6E3HA6_BRIGHT_H__ */
