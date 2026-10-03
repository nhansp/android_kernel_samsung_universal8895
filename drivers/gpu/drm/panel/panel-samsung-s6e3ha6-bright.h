// SPDX-License-Identifier: GPL-2.0-only
/*
 * Command payloads and tuning tables of the SET_BL_SEQ brightness sequence of
 * the two S6E3HA6 panel variants.
 *
 * Verbatim table data of the stock 4.4.111
 * drivers/video/fbdev/exynos/panel/s6e3ha6/s6e3ha6_great_a3_sa_panel.h
 * (S:drivers/video/fbdev/exynos/panel/s6e3ha6/s6e3ha6_great_a3_sa_panel.h)
 * and s6e3ha6_great_a2_s3_panel.h, the byte arrays behind
 * great_a3_sa_set_bl_cmdtbl / great_a2_s3_set_bl_cmdtbl
 * (a3_sa :2033-2047, a2_s3 :2052-2066), the maptbl rows they index
 * (a3_sa :1361-1416, a2_s3 :1412-1467) and the level 1/2/3 keys of
 * great_a3_sa_res_init_cmdtbl / great_a2_s3_res_init_cmdtbl
 * (a3_sa :1981-2002, a2_s3 :2032-2053). Only "static const" was added; the
 * stock #if PANEL_BACKLIGHT_PAC_STEPS == 512 and #ifdef CONFIG_LCD_EXTEND_HBM
 * guards are kept (the stock build takes both: IKC:3128
 * CONFIG_PANEL_BACKLIGHT_PAC_3_0=y, and panel_bl.h:17 defines
 * CONFIG_LCD_EXTEND_HBM).
 *
 * The rows that stock reads from a maptbl at runtime are named *_tbl here and
 * filled by s6e3ha6_bright_apply() exactly where panel_update_packet_data()
 * (S:.../panel/panel.c:430-478) copies them into the payload.
 */

#ifndef __S6E3HA6_BRIGHT_H__
#define __S6E3HA6_BRIGHT_H__

#include <linux/backlight.h>
#include <linux/device.h>
#include <linux/lcd.h>
#include <linux/mutex.h>
#include <linux/types.h>

#include <drm/drm_mipi_dsi.h>

/* panel-samsung-s6e3ha6-brt.h is deliberately not included here: it carries the
 * vendor dimming interface (panel-samsung-s6e3ha6-dimming.h) and only the
 * table half of this driver needs it.
 */

#define S6E3HA6_GAMMA_CMD_CNT	36	/* s6e3ha6.h:28 GAMMA_CMD_CNT */

struct s6e3ha6_bright;

struct s6e3ha6_bright *s6e3ha6_bright_register(struct device *dev,
					      struct mipi_dsi_device *dsi,
					      const char *variant);
int s6e3ha6_bright_register_lcd(struct s6e3ha6_bright *b);
void s6e3ha6_bright_unregister(struct s6e3ha6_bright *b);
int s6e3ha6_bright_prepare(struct s6e3ha6_bright *b);
int s6e3ha6_bright_enable(struct s6e3ha6_bright *b);

/*
 * The payloads and tuning tables this header used to carry are in
 * panel-samsung-s6e3ha6-bright-tbl.h, which only
 * panel-samsung-s6e3ha6-bright.c includes: they need the vendor dimming
 * interface of panel-samsung-s6e3ha6-brt.h, which the panel driver cannot
 * pull in next to the vendor panel framework's own dimming.h.
 */
#endif	/* __S6E3HA6_BRIGHT_H__ */
