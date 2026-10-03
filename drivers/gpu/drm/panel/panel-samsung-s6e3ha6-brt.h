// SPDX-License-Identifier: GPL-2.0-only
/*
 * Brightness tables and the tuning-point description of the S6E3HA6 panel.
 *
 * Port of the stock 4.4.111
 * drivers/video/fbdev/exynos/panel/panel_bl.h:21-50,63-81 and
 * struct panel_dimming_info of
 * drivers/video/fbdev/exynos/panel/panel_dimming.h, plus the shared tuning
 * point table and luminance counts of
 * drivers/video/fbdev/exynos/panel/s6e3ha6/s6e3ha6_dimming.h
 * (S:drivers/video/fbdev/exynos/panel/s6e3ha6/s6e3ha6_dimming.h:18-56).
 *
 * The stock ikconfig has CONFIG_PANEL_BACKLIGHT_PAC_3_0=y (IKC:3128) and
 * panel_bl.h:21 defines CONFIG_LCD_EXTEND_HBM unconditionally, so BRT_SCALE is
 * 100, PANEL_BACKLIGHT_PAC_STEPS is 512 and EXTEND_BRIGHTNESS is
 * BRT(366) = 36600: the max_brightness the Samsung lights HAL reads.
 */

#ifndef __S6E3HA6_PANEL_BRT_H__
#define __S6E3HA6_PANEL_BRT_H__

#include "panel-samsung-s6e3ha6-dimming.h"

/* panel_bl.h:21 */
#define CONFIG_LCD_EXTEND_HBM
/* panel_bl.h:23-26, IKC:3128 */
#define BRT_SCALE	(100)
#define PANEL_BACKLIGHT_PAC_STEPS	(512)

#define BRT_SCALE_UP(_val_)	((_val_) * BRT_SCALE)
#define BRT_SCALE_DN(_val_)	((_val_) / BRT_SCALE)
#define BRT_TBL_INDEX(_val_)	(BRT_SCALE_DN((_val_) + ((BRT_SCALE) - 1)))
#define BRT(brt)		(BRT_SCALE_UP(brt))
#define BRT_USER(brt)		(BRT_SCALE_DN(brt))

#define UI_MIN_BRIGHTNESS		(BRT(0))
#define UI_DEF_BRIGHTNESS		(BRT(128))
#define UI_MAX_BRIGHTNESS		(BRT(255))
#define UI_BRIGHTNESS_STEPS		(UI_MAX_BRIGHTNESS - UI_MIN_BRIGHTNESS + 1)
#define HBM_MIN_BRIGHTNESS		(UI_MAX_BRIGHTNESS + 1)
#define HBM_MAX_BRIGHTNESS		(BRT(365))
#define HBM_BRIGHTNESS_STEPS		(8)
#define EXTEND_HBM_MIN_BRIGHTNESS	(HBM_MAX_BRIGHTNESS + 1)
#define EXTEND_HBM_MAX_BRIGHTNESS	(BRT(366))
#define EXTEND_HBM_BRIGHTNESS_STEPS	(1)

#define EXTEND_BRIGHTNESS		(EXTEND_HBM_MAX_BRIGHTNESS)

#define IS_UI_BRIGHTNESS(br)		(((br) <= UI_MAX_BRIGHTNESS) ? 1 : 0)
#define IS_HBM_BRIGHTNESS(br)		((((br) >= HBM_MIN_BRIGHTNESS) && ((br) <= HBM_MAX_BRIGHTNESS)) ? 1 : 0)
#define IS_EXT_HBM_BRIGHTNESS(br)	((((br) >= EXTEND_HBM_MIN_BRIGHTNESS) && ((br) <= EXTEND_HBM_MAX_BRIGHTNESS)) ? 1 : 0)

enum SEARCH_TYPE {
	SEARCH_TYPE_EXACT,
	SEARCH_TYPE_LOWER,
	SEARCH_TYPE_UPPER,
	MAX_SEARCH_TYPE,
};

/* panel_bl.h:63-81 */
struct brightness_table {
	/* brightness to step mapping table */
	const u32 *brt_to_step;
	u32 sz_brt_to_step;
	/* brightness to brightness mapping table */
	const u32 (*brt_to_brt)[2];
	u32 sz_brt_to_brt;
	/* brightness table */
	const u32 *brt;
	u32 sz_brt;
	/* actual brightness table */
	const u32 *lum;
	u32 sz_lum;
	u32 sz_ui_lum;
	u32 sz_hbm_lum;
	u32 sz_ext_hbm_lum;
	u32 max_ui_lum;
	u32 max_hbm_lum;
};

/* panel_dimming.h:17-28 */
struct panel_dimming_info {
	const char *name;
	struct dimming_init_info dim_init_info;
	struct dimming_info *dim_info;
	s32 extend_hbm_target_luminance;
	s32 nr_extend_hbm_luminance;
	s32 hbm_target_luminance;
	s32 nr_hbm_luminance;
	s32 target_luminance;
	s32 nr_luminance;
	const struct brightness_table *brt_tbl;
};

/* s6e3ha6_dimming.h:18-56 */
#define S6E3HA6_NR_TP (11)

#define S6E3HA6_NR_LUMINANCE (74)
#define S6E3HA6_TARGET_LUMINANCE (420)

#define S6E3HA6_NR_HBM_LUMINANCE (7)
#define S6E3HA6_TARGET_HBM_LUMINANCE (600)

#define S6E3HA6_HMD_NR_LUMINANCE (37)
#define S6E3HA6_HMD_TARGET_LUMINANCE (420)

#ifdef CONFIG_LCD_EXTEND_HBM
#define S6E3HA6_NR_EXTEND_HBM_LUMINANCE	(1)
#define S6E3HA6_TARGET_EXTEND_HBM_LUMINANCE	(720)
#define S6E3HA6_TOTAL_NR_LUMINANCE (S6E3HA6_NR_LUMINANCE + S6E3HA6_NR_HBM_LUMINANCE + S6E3HA6_NR_EXTEND_HBM_LUMINANCE)
#define S6E3HA6_HMD_TOTAL_NR_LUMINANCE (S6E3HA6_HMD_NR_LUMINANCE)
#else
#define S6E3HA6_TOTAL_NR_LUMINANCE (S6E3HA6_NR_LUMINANCE + S6E3HA6_NR_HBM_LUMINANCE)
#define S6E3HA6_HMD_TOTAL_NR_LUMINANCE (S6E3HA6_HMD_NR_LUMINANCE)
#endif

#define S6E3HA6_TOTAL_PAC_STEPS		(PANEL_BACKLIGHT_PAC_STEPS + S6E3HA6_NR_HBM_LUMINANCE + S6E3HA6_NR_EXTEND_HBM_LUMINANCE)

static struct tp s6e3ha6_tp[S6E3HA6_NR_TP] = {
	{ .level = 0, .volt_src = VREG_OUT, .name = "VT", .center = { 0x0, 0x0, 0x0 }, .numerator = 0, .denominator = 860 },
	{ .level = 1, .volt_src = V0_OUT, .name = "V1", .center = { 0x80, 0x80, 0x80 }, .numerator = 0, .denominator = 256 },
	{ .level = 7, .volt_src = V0_OUT, .name = "V7", .center = { 0x80, 0x80, 0x80 }, .numerator = 64, .denominator = 320 },
	{ .level = 11, .volt_src = VT_OUT, .name = "V11", .center = { 0x80, 0x80, 0x80 }, .numerator = 64, .denominator = 320 },
	{ .level = 23, .volt_src = VT_OUT, .name = "V23", .center = { 0x80, 0x80, 0x80 }, .numerator = 64, .denominator = 320 },
	{ .level = 35, .volt_src = VT_OUT, .name = "V35", .center = { 0x80, 0x80, 0x80 }, .numerator = 64, .denominator = 320 },
	{ .level = 51, .volt_src = VT_OUT, .name = "V51", .center = { 0x80, 0x80, 0x80 }, .numerator = 64, .denominator = 320 },
	{ .level = 87, .volt_src = VT_OUT, .name = "V87", .center = { 0x80, 0x80, 0x80 }, .numerator = 64, .denominator = 320 },
	{ .level = 151, .volt_src = VT_OUT, .name = "V151", .center = { 0x80, 0x80, 0x80 }, .numerator = 64, .denominator = 320 },
	{ .level = 203, .volt_src = VT_OUT, .name = "V203", .center = { 0x80, 0x80, 0x80 }, .numerator = 64, .denominator = 320 },
	{ .level = 255, .volt_src = VREG_OUT, .name = "V255", .center = { 0x100, 0x100, 0x100 }, .numerator = 129, .denominator = 860 },
};

#endif	/* __S6E3HA6_PANEL_BRT_H__ */