/*
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *	      http://www.samsung.com/
 *
 * Samsung's Panel Driver
 * Author: Minwoo Kim <minwoo7945.kim@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __PANEL_DRV_H__
#define __PANEL_DRV_H__

#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/notifier.h>
#include <linux/kernel.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/workqueue.h>
#include <linux/miscdevice.h>

#include "decon_lcd.h"
#include "panel.h"
#include "mdnie.h"
#include "copr.h"
#include "panel_poc.h"

extern int panel_log_level;

#define panel_err(fmt, ...)							\
	do {									\
		if (panel_log_level >= 3) {					\
			pr_err(pr_fmt(fmt), ##__VA_ARGS__);			\
		}								\
	} while (0)

#define panel_warn(fmt, ...)							\
	do {									\
		if (panel_log_level >= 4) {					\
			pr_warn(pr_fmt(fmt), ##__VA_ARGS__);			\
		}								\
	} while (0)
#define panel_info(fmt, ...)							\
	do {									\
		if (panel_log_level >= 6)					\
			pr_info(pr_fmt(fmt), ##__VA_ARGS__);			\
	} while (0)
#define panel_dbg(fmt, ...)							\
	do {									\
		if (panel_log_level >= 6)					\
			pr_info(pr_fmt(fmt), ##__VA_ARGS__);			\
	} while (0)


enum {
	REGULATOR_3p0V = 0,
	REGULATOR_1p8V,
	REGULATOR_1p6V,
	REGULATOR_MAX
};

/*
 * The DRM panel owns the pads and fills them in, the rails in the vendor
 * order. The board has no pcd or err_fg line.
 */
struct panel_device;

struct panel_pad {
	struct gpio_desc *gpio_reset;
	struct gpio_desc *gpio_disp_det;

	/* switches the disp-det interrupt, which the DRM panel owns */
	void (*disp_det_irq)(struct panel_device *panel, bool on);

	struct regulator *regulator[REGULATOR_MAX];
};

struct mipi_drv_ops {
	int (*read)(u32 id, u8 addr, u8 *buf, int size);
	int (*write)(u32 id, u8 cmd_id, const u8 *cmd, int size);
	enum dsim_state (*get_state)(u32 id);
};

#define PANEL_INIT_KERNEL 		0
#define PANEL_INIT_BOOT 		1

#define PANEL_DISCONNECT		0
#define PANEL_CONNECT			1

enum panel_active_state {
	PANEL_STATE_OFF = 0,
	PANEL_STATE_ON,
	PANEL_STATE_NORMAL,
	PANEL_STATE_ALPM,
};

enum panel_power {
	PANEL_POWER_OFF = 0,
	PANEL_POWER_ON
};

enum {
	PANEL_DISPLAY_OFF = 0,
	PANEL_DISPLAY_ON,
};

enum {
	PANEL_HMD_OFF = 0,
	PANEL_HMD_ON,
};

struct panel_state {
	int init_at;
	int connect_panel;
	int cur_state;
	int power;
	int disp_on;
	int hmd_on;
};

enum {
	IMG_UPDATE_NEED = 0,
	IMG_UPDATE_DONE,
};

struct act_clk_info {
	u32 en;
	u32 interval;
	u32 time_hr;
	u32 time_min;
	u32 time_sec;
	u32 time_ms;
	u32 pos_x;
	u32 pos_y;
/* flag to check need to update side ram img */
	u32 update_img;
	char *img_buf;
	u32 img_buf_size;
	u32 update_time;
};

struct act_blink_info {
	u32 en;
	u32 interval;
	u32 radius;
	u32 color;
	u32 line_width;
	u32 pos1_x;
	u32 pos1_y;
	u32 pos2_x;
	u32 pos2_y;
};

struct act_drawer_info {
	u32 sd_line_color;
	u32 sd2_line_color;
	u32 sd_radius;
};

struct act_clock_dev {
	struct miscdevice dev;
	struct act_clk_info act_info;
	struct act_blink_info blink_info;
	struct act_drawer_info draw_info;
	u32 opened;
};


struct copr_spi_gpios {
	struct gpio_desc *gpio_sck;
	struct gpio_desc *gpio_miso;
	struct gpio_desc *gpio_mosi;
	struct gpio_desc *gpio_cs;
};
struct panel_device {
	int id;
	int dsi_id;

	struct spi_device *spi;
	struct copr_info copr;
	struct copr_spi_gpios spi_gpio;

	/* mutex lock for panel operations */
	struct mutex op_lock;
	/* mutex lock for panel's data */
	struct mutex data_lock;
	/* mutex lock for panel's ioctl */
	struct mutex io_lock;

	struct device *dev;
	struct mdnie_info mdnie;

	struct lcd_device *lcd;
	struct panel_bl_device panel_bl;

	unsigned char panel_id[3];

	struct panel_pad pad;

	struct decon_lcd lcd_info;

	struct mipi_drv_ops mipi_drv;

	struct panel_state state;

	struct panel_info panel_data;

	struct act_clock_dev act_clk_dev;

	struct panel_poc_device poc_dev;

	ktime_t ktime_panel_on;
	ktime_t ktime_panel_off;
};


/*
 * The vendor DSIM and DECON sent these through the panel's v4l2 subdev; the
 * DRM panel calls panel_drv_ioctl(). The numbers are the vendor's.
 */
#define PANEL_IOC_BASE	'P'

#define PANEL_IOC_PANEL_PROBE			_IO(PANEL_IOC_BASE, 5)

#define PANEL_IOC_SET_POWER				_IO(PANEL_IOC_BASE, 6)

#define PANEL_IOC_SLEEP_IN				_IO(PANEL_IOC_BASE, 7)
#define PANEL_IOC_SLEEP_OUT				_IO(PANEL_IOC_BASE, 8)

#define PANEL_IOC_DISP_ON				_IO(PANEL_IOC_BASE, 9)

#define PANEL_IOC_PANEL_DUMP			_IO(PANEL_IOC_BASE, 10)

#define PANEL_IOC_EVT_FRAME_DONE		_IOW(PANEL_IOC_BASE, 11, struct timespec64 *)
#define PANEL_IOC_EVT_VSYNC			_IOW(PANEL_IOC_BASE, 12, struct timespec64 *)

#define PANEL_IOC_DOZE					_IO(PANEL_IOC_BASE, 13)
#define PANEL_IOC_SET_DSU				_IOW(PANEL_IOC_BASE, 14, struct dsu_info *)

int panel_drv_init(struct panel_device *panel, struct device *dev,
		   bool init_at_boot, bool connected);
long panel_drv_ioctl(struct panel_device *panel, unsigned int cmd, void *arg);
#endif //__PANEL_DRV_H__
