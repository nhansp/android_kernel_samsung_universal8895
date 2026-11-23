/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Samsung AID (Adaptive Interface Dimming) engine of the S6E3HA6 panel,
 * from the Samsung vendor panel driver.
 *
 * Copyright (c) 2016 Samsung Electronics
 * Gwanghui Lee <gwanghui.lee@samsung.com>
 */

#ifndef _S6E3HA6_DIMMING_H_
#define _S6E3HA6_DIMMING_H_

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/types.h>

#define DIMMING_CALC_PRECISE
#define BIT_SHIFT	(24)
#define DIMMING_BITSHIFT	(BIT_SHIFT)
#define GRAY_SCALE_MAX	(256)
#define ERROR_DETECT_BOUND	(1)

enum color {
	RED,
	GREEN,
	BLUE,
	MAX_COLOR,
};

enum gamma_degree {
	GAMMA_NONE = 0,
	GAMMA_MIN,
	GAMMA_1_60 = GAMMA_MIN,
	GAMMA_1_65,
	GAMMA_1_70,
	GAMMA_1_75,
	GAMMA_1_80,
	GAMMA_1_85,
	GAMMA_1_90,
	GAMMA_1_95,
	GAMMA_2_00,
	GAMMA_2_05,
	GAMMA_2_10,
	GAMMA_2_12,
	GAMMA_2_13,
	GAMMA_2_15,
	GAMMA_2_20,
	GAMMA_2_25,
	GAMMA_MAX,
};

enum {
	VREG_OUT,
	V0_OUT,
	VT_OUT,
	MAX_VOLT_SRC,
};

struct gamma_curve {
	int gamma;
	const int *gamma_curve_table;
};

struct dimming_tp_output {
	u64 L;
	u32 M_GRAY;
	s64 vout[MAX_COLOR];
	s32 center[MAX_COLOR];
};

struct dimming_lut {
	/* input data from user setting */
	u32 luminance;
	u32 aor;
	u32 base_luminance;
	u32 base_gray;
	enum gamma_degree g_curve_degree;
	const s32 *gray_scale_offset;
	const s32 *rgb_color_offset;

	/* output(reversed distortion) data will be stored */
	struct dimming_tp_output *tpout;
};

struct gray_scale {
	u64 gamma_value;
	s64 vout[MAX_COLOR];
};

struct tp {
	int level;			/* gray_level of each tp */
	int volt_src;			/* voltage source of each tp */
	const char *name;		/* name of each tp */
	s32 center[MAX_COLOR];		/* center gamma's fixed value of tp */
	s32 offset[MAX_COLOR];		/* mtp offset value of tp */
	s64 vout[MAX_COLOR];		/* voltage output of tp */
	u32 denominator;		/* denominator of tp */
	u32 numerator;			/* numerator of tp */
};

struct dimming_info {
	s64 vregout;
	u32 vt_voltage[16];		/* vt voltage table */
	int nr_tp;			/* number of turning point */
	struct tp *tp;			/* informations of panel's tuning point */

	int nr_luminance;		/* number of luminance */
	struct gray_scale gray_scale_lut[GRAY_SCALE_MAX];
	struct dimming_lut *dim_lut;
	int target_luminance;
	enum gamma_degree target_g_curve_degree;

	s32 hbm_luminance;
	s32 (*hbm_gamma_tbl)[MAX_COLOR];
};

struct dimming_init_info {
	const char *name;
	int nr_tp;			/* number of turning point */
	struct tp *tp;			/* informations of panel's tuning point */
	int nr_luminance;		/* number of luminance */
	s64 vregout;			/* vregout * (1 << bitshift) */
	int bitshift;			/* bitshift for precise calculation */
	u32 vt_voltage[16];		/* vt voltage table */
	int target_luminance;		/* target luminance */
	int target_gamma;		/* target gamma value */
	struct dimming_lut *dim_lut;

	s32 hbm_luminance;
};

/* DIM_LUT_V0 : INSERT BASE_LUMINANCE DIRECTLY */
#define DIM_LUT_V0_INIT(l, bl, gma, rtbl, ctbl)		\
{							\
	.luminance = (l),				\
	.base_luminance = (bl),			\
	.base_gray = (0),				\
	.g_curve_degree = (gma),			\
	.gray_scale_offset = (rtbl),			\
	.rgb_color_offset = (ctbl),			\
}

#define DIM_LUT_INIT(l, bl, gma, rtbl, ctbl)		\
	DIM_LUT_V0_INIT(l, bl, gma, rtbl, ctbl)

enum gamma_degree gamma_value_to_degree(int gamma);
int gamma_curve_value(enum gamma_degree g_curve, int idx);
int gamma_table_interpolation(s32 (*from)[MAX_COLOR], s32 (*to)[MAX_COLOR],
			      s32 (*out)[MAX_COLOR], int nr_tp, int cur_step,
			      int total_step);
int alloc_dimming_info(struct dimming_info *dim_info, int nr_tp, int nr_luminance);
void free_dimming_info(struct dimming_info *dim_info);
void prepare_dim_info(struct dimming_info *dim_info);
int init_dimming_info(struct dimming_info *dim_info, struct dimming_init_info *src);
int init_dimming_mtp(struct dimming_info *dim_info, s32 (*mtp)[MAX_COLOR]);
int init_dimming_hbm_info(struct dimming_info *dim_info, s32 (*hbm_gamma_tbl)[MAX_COLOR],
			  u32 hbm_luminance);
int process_dimming(struct dimming_info *dim_info);
void get_dimming_gamma_compact(struct dimming_info *dim_info, u32 luminance, u8 *output);
void get_dimming_gamma(struct dimming_info *dim_info, u32 luminance, u8 *output);
int find_luminance(struct dimming_info *dim_info, u32 luminance);
int get_luminance(struct dimming_info *dim_info, u32 index);
void mtp_ctoi(s32 (*dst)[MAX_COLOR], char *src, int nr_tp);
int hbm_ctoi(s32 (*dst)[MAX_COLOR], u8 *hbm, int nr_tp);

#endif	/* _S6E3HA6_DIMMING_H_ */
