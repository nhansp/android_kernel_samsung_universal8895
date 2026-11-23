// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung AID dimming algorithm of the S6E3HA6 panel, from the Samsung
 * vendor panel driver, with the MTP decoders of its S6E3HA6 part.
 *
 * Copyright (c) 2016 Samsung Electronics
 * Gwanghui Lee <gwanghui.lee@samsung.com>
 */

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>

#include "panel-samsung-s6e3ha6-dimming.h"
#include "panel-samsung-s6e3ha6-dimming-gamma.h"

static int NR_LUMINANCE;
static int NR_TP;
static int TP_VT = 0;
static int TP_2ND = -1;
static int TP_3RD = -1;
static int TP_V255 = -1;
static int TARGET_LUMINANCE;
static enum gamma_degree TARGET_G_CURVE_DEGREE;

static int *calc_gray_order;
static int *calc_vout_order;

const char *color_name[] = {
	"RED", "GREEN", "BLUE",
};

/* gamma_curve_lut */
static const struct gamma_curve gamma_curve_lut[] = {
	{ 0, NULL },
	{ 160, gamma_curve_1p60 },
	{ 165, gamma_curve_1p65 },
	{ 170, gamma_curve_1p70 },
	{ 175, gamma_curve_1p75 },
	{ 180, gamma_curve_1p80 },
	{ 185, gamma_curve_1p85 },
	{ 190, gamma_curve_1p90 },
	{ 195, gamma_curve_1p95 },
	{ 200, gamma_curve_2p00 },
	{ 205, gamma_curve_2p05 },
	{ 210, gamma_curve_2p10 },
	{ 212, gamma_curve_2p12 },
	{ 213, gamma_curve_2p13 },
	{ 215, gamma_curve_2p15 },
	{ 220, gamma_curve_2p20 },
	{ 225, gamma_curve_2p25 },
};

static s64 disp_pow(s64 num, u32 digits)
{
	s64 res = num;
	u32 i;

	if (digits == 0)
		return 1;

	for (i = 1; i < digits; i++)
		res *= num;
	return res;
}

static s64 disp_round(s64 num, u32 digits)
{
	int sign = (num < 0 ? -1 : 1);
	u64 tnum;

	tnum = num * sign;
	tnum *= disp_pow(10, digits + 1);
	tnum += 5;
	do_div(tnum, (u32)disp_pow(10, digits + 1));

	return sign * (s64)tnum;
}

static s64 disp_div64(s64 num, s64 den)
{
	int sign_num = (num < 0 ? -1 : 1);
	int sign_den = (den < 0 ? -1 : 1);
	int sign = sign_num * sign_den;
	u64 tnum, tden;

	if (unlikely(den == 0)) {
		pr_err("%s, den should not be zero (%lld %lld)\n", __func__, num, den);
		return 0;
	}

	tnum = num * sign_num;
	tden = den * sign_den;

	if (tden >= (1ULL << 31))
		pr_err("%s, out of range denominator %llu\n", __func__, tden);

	do_div(tnum, (u32)tden);

	return sign * tnum;
}

static s64 disp_div64_round(s64 num, s64 den, u32 digits)
{
	int sign_num = (num < 0 ? -1 : 1);
	int sign_den = (den < 0 ? -1 : 1);
	int sign = sign_num * sign_den;
	u64 tnum, tden, rem;

	if (unlikely(den == 0)) {
		pr_err("%s, den should not be zero (%lld %lld %u)\n",
		       __func__, num, den, digits);
		return 0;
	}

	tnum = num * sign_num;
	tden = den * sign_den;

	rem = do_div(tnum, (u32)tden);
	rem *= disp_pow(10, digits + 2);
	do_div(rem, (u32)tden);

	/*
	 * floating point should not be used.
	 * Some variables are inaccurate in most case (e.g. vregout...)
	 * To compensate it, round_up function use (+ 6) instead of (+ 5).
	 */
	rem += 5;
	do_div(rem, (u32)disp_pow(10, digits + 2));

	return sign * (tnum + rem);
}

/*
 * generate_order() is an helper function can make an order
 * that mixed an initial order value and range based order values.
 */
static int *generate_order(int s, int from, int to)
{
	int i, len = abs(from - to) + 2;
	int sign = ((to - from) < 0) ? -1 : 1;
	int *t_order = (int *)kmalloc(len * sizeof(int), GFP_KERNEL);

	if (!t_order) {
		pr_err("%s, failed to allocate memory\n", __func__);
		return NULL;
	}

	t_order[0] = s;
	t_order[1] = from;
	for (i = 2; i < len; i++)
		t_order[i] = t_order[i - 1] + sign;

	return t_order;
}

static int find_gamma_curve(int gamma)
{
	int index;

	for (index = 0; index < ARRAY_SIZE(gamma_curve_lut); index++)
		if (gamma_curve_lut[index].gamma == gamma)
			break;

	if (index == ARRAY_SIZE(gamma_curve_lut)) {
		pr_err("%s, gamma %d curve not found\n", __func__, gamma);
		return -1;
	}

	return index;
}

enum gamma_degree gamma_value_to_degree(int gamma)
{
	int g_curve_index;

	g_curve_index = find_gamma_curve(gamma);
	if (g_curve_index < 0) {
		pr_err("%s, gamma %d not exist\n", __func__, gamma);
		return GAMMA_NONE;
	}

	return (enum gamma_degree)g_curve_index;
}

/*
 * @ g_curve : gamma curve
 * @ idx : gray scale level 0 ~ 255.
 * @ return ((idx/255)^g_curve)*(2^22)
 */
int gamma_curve_value(enum gamma_degree g_curve, int idx)
{
	if (g_curve == GAMMA_NONE || idx >= GRAY_SCALE_MAX)
		return -1;

	return gamma_curve_lut[(int)g_curve].gamma_curve_table[idx];
}

static u64 mtp_offset_to_vregout(struct dimming_info *dim_info,
		int v, enum color c)
{
	s64 num, den, vreg, vsrc, res = 0;
	struct tp *tp = dim_info->tp;
	u32 *vt_voltage = dim_info->vt_voltage;
	s64 VREGOUT = dim_info->vregout;

	if (!tp || v < 0 || v >= NR_TP) {
		pr_err("%s, invalid tp %d\n", __func__, v);
		return -EINVAL;
	}

	den = (s64)tp[v].denominator;

	if (tp[v].volt_src == VREG_OUT || tp[v].volt_src == V0_OUT)
		vsrc = VREGOUT;
	else if (tp[v].volt_src == VT_OUT)
		vsrc = tp[TP_VT].vout[c];
	else {
		vsrc = tp[TP_VT].vout[c];
		pr_warn("%s unknown tp[%d].volt_src %d\n",
			__func__, v, tp[v].volt_src);
	}

	if (v == TP_VT) {
		s32 offset = tp[v].center[c] + tp[v].offset[c];

		if (offset > 0xF) {
			pr_err("%s, error : ivt (%d) out of range(0x0 ~ 0xF)\n", __func__, offset);
			return -EINVAL;
		}
		vreg = vsrc;
		num = vreg * (s64)vt_voltage[offset];
		res = vsrc - disp_div64(num, den);
	} else if (v == TP_V255) {
		vreg = vsrc;
		num = vreg * ((s64)tp[v].numerator + tp[v].center[c] + tp[v].offset[c]);
		res = vsrc - disp_div64(num, den);
	} else {
		vreg = vsrc - tp[v + 1].vout[c];
		num = vreg * ((s64)tp[v].numerator + tp[v].center[c] + tp[v].offset[c]);
		res = vsrc - disp_div64(num, den);
	}

	if (res < 0)
		pr_err("%s, res %lld should not be minus value\n", __func__, res);

	return res;
}

static s32 mtp_vregout_to_offset(struct dimming_info *dim_info,
		int v, enum color c, int luminance_index)
{
	s64 num, den, res, vsrc;
	u32 upper = ((v == TP_V255) ? 511 : 255);
	const s32 *rgb_offset;
	s64 VREGOUT = dim_info->vregout;
	struct dimming_lut *dim_lut = dim_info->dim_lut;
	struct tp *tp = dim_info->tp;
	struct dimming_tp_output *tpout;

	if (!tp || v < 0 || v >= NR_TP) {
		pr_err("%s, invalid tp %d\n", __func__, v);
		return -EINVAL;
	}

	if (luminance_index < 0 || luminance_index >= NR_LUMINANCE) {
		pr_err("%s, out of range luminance index (%d)\n", __func__, luminance_index);
		return 0;
	}

	/* the table is a flat s32[NR_TP * MAX_COLOR] */
	rgb_offset = dim_lut[luminance_index].rgb_color_offset;
	tpout = dim_lut[luminance_index].tpout;

	if (tp[v].volt_src == VREG_OUT || tp[v].volt_src == V0_OUT)
		vsrc = VREGOUT;
	else if (tp[v].volt_src == VT_OUT)
		vsrc = tpout[TP_VT].vout[c];
	else {
		vsrc = tpout[TP_VT].vout[c];
		pr_warn("%s unknown tp[%d].volt_src %d\n", __func__, v, tp[v].volt_src);
	}

	if (v == TP_V255) {
		num = (vsrc - tpout[v].vout[c]) * (s64)tp[v].denominator;
		den = vsrc;
	} else {
		num = (vsrc - tpout[v].vout[c]) * (s64)tp[v].denominator;
		den = vsrc - tpout[v + 1].vout[c];
	}

	/* DIMMING_CALC_PRECISE branch: to get a precise value use disp_div64_round() */
	num -= den * (tp[v].numerator);
	num = disp_round(num, 7) - den * (s64)tp[v].offset[c];
	res = disp_div64_round(num, den, 2);
	res += rgb_offset[v * MAX_COLOR + c];

	if (res < 0)
		res = 0;
	if (res > upper)
		res = upper;

	return (s32)res;
}

/*
 * sams as excel macro function min_diff_gray().
 */
static int find_gray_scale_gamma_value(struct gray_scale *gray_scale_lut, u64 gamma_value)
{
	int l = 0, m, r = GRAY_SCALE_MAX - 1;
	s64 delta_l, delta_r;

	if (unlikely(!gray_scale_lut)) {
		pr_err("%s, gray_scale_lut not exist\n", __func__);
		return -EINVAL;
	}

	if ((gray_scale_lut[l].gamma_value > gamma_value) ||
			(gray_scale_lut[r].gamma_value < gamma_value)) {
		pr_err("%s, out of range([l:%d, r:%d]) candela(%lld)\n",
			__func__, l, r, gamma_value);
		return -EINVAL;
	}

	while (l <= r) {
		m = l + (r - l) / 2;
		if (gray_scale_lut[m].gamma_value == gamma_value)
			return m;
		if (gray_scale_lut[m].gamma_value < gamma_value)
			l = m + 1;
		else
			r = m - 1;
	}
	delta_l = gamma_value - gray_scale_lut[r].gamma_value;
	delta_r = gray_scale_lut[l].gamma_value - gamma_value;

	return delta_l <= delta_r ? r : l;
}

static s64 interpolation(s64 from, s64 to, int cur_step, int total_step)
{
	s64 num, den;

	if (cur_step == total_step || from == to)
		return to;

	num = (to - from) * (s64)cur_step;
	den = (s64)total_step;
	num = from + disp_div64(num, den);

	return num;
}

int gamma_table_interpolation(s32 (*from)[MAX_COLOR], s32 (*to)[MAX_COLOR],
		s32 (*out)[MAX_COLOR], int nr_tp, int cur_step, int total_step)
{
	int i, c;

	if (unlikely(!from || !to || !out)) {
		pr_err("%s, invalid parameter\n", __func__);
		return -EINVAL;
	}

	if (unlikely(nr_tp < 0 || cur_step > total_step)) {
		pr_err("%s, out of range (nr_tp %d, cur_step %d, total_step %d)\n",
			__func__, nr_tp, cur_step, total_step);
		return -EINVAL;
	}

	for (i = 0; i < nr_tp; i++)
		for (c = 0; c < MAX_COLOR; c++)
			out[i][c] = interpolation(from[i][c], to[i][c], cur_step, total_step);

	return 0;
}

/*
 * ########### BASE LUMINANCE GRAY SCALE TABLE FUNCTIONS ###########
 */
static int generate_gray_scale(struct dimming_info *dim_info)
{
	int i, c, iv = 0, iv_upper = 0, iv_lower = 0, ret = 0, cur_step, total_step = 0;
	s64 vout, v_upper, v_lower;
	struct gray_scale *gray_scale_lut = dim_info->gray_scale_lut;
	struct tp *tp = dim_info->tp;
	int nr_tp = dim_info->nr_tp;

	for (i = 0, iv = 0; iv < nr_tp && i < GRAY_SCALE_MAX; i++) {
		if (i == tp[iv].level) {
			iv_upper = tp[((iv == nr_tp - 1) ? iv : iv + 1)].level;
			iv_lower = tp[iv].level;
			total_step = iv_upper - iv_lower;
			iv++;
			continue;
		}

		cur_step = i - iv_lower;
		for (c = 0; c < MAX_COLOR; c++) {
			v_upper = (s64)gray_scale_lut[iv_upper].vout[c];
			v_lower = (s64)gray_scale_lut[iv_lower].vout[c];
			vout = interpolation(v_lower, v_upper, cur_step, total_step);
			if (vout < 0) {
				pr_warn("%s, from %lld, to %lld (idx %-3d %-3d), cur_step %-3d, total_step %-3d\n",
					__func__, v_lower, v_upper, iv_lower, iv_upper,
					cur_step, total_step);
				ret = -EINVAL;
			}
			gray_scale_lut[i].vout[c] = vout;
		}
	}

	for (i = 0; i < GRAY_SCALE_MAX; i++) {
		/* base luminance gamma curve value */
		gray_scale_lut[i].gamma_value = TARGET_LUMINANCE *
			(u64)gamma_curve_value(TARGET_G_CURVE_DEGREE, i);
	}

	return ret;
}

/*
 * ########### GAMMA CORRECTION TABLE FUNCTIONS ###########
 */
int find_luminance(struct dimming_info *dim_info, u32 luminance)
{
	int index, nr_luminance;
	struct dimming_lut *dim_lut;

	if (unlikely(!dim_info)) {
		pr_err("%s, invalid dim_info\n", __func__);
		return -1;
	}

	dim_lut = dim_info->dim_lut;
	nr_luminance = dim_info->nr_luminance;

	if (unlikely(!dim_lut)) {
		pr_err("%s, invalid dim_lut\n", __func__);
		return -1;
	}

	for (index = 0; index < nr_luminance; index++)
		if (dim_lut[index].luminance == luminance)
			return index;

	return -1;
}

int get_luminance(struct dimming_info *dim_info, u32 index)
{
	int nr_luminance;
	struct dimming_lut *dim_lut;

	if (unlikely(!dim_info)) {
		pr_err("%s, invalid dim_info\n", __func__);
		return 0;
	}

	dim_lut = dim_info->dim_lut;
	nr_luminance = dim_info->nr_luminance;

	if (unlikely(!dim_lut)) {
		pr_err("%s, invalid dim_lut\n", __func__);
		return 0;
	}

	if (index >= nr_luminance) {
		pr_warn("%s, exceed max %d, index %d\n",
			__func__, nr_luminance - 1, index);
		return dim_lut[nr_luminance - 1].luminance;
	}
	return dim_lut[index].luminance;
}

static void copy_tpout_center(struct dimming_info *dim_info, u32 luminance, u8 *output, bool compact)
{
	struct dimming_lut *dim_lut = dim_info->dim_lut;
	struct tp *tp = dim_info->tp;
	int ilum = find_luminance(dim_info, luminance);
	int i, c, value, idst = 0;

	if (unlikely(!tp || !dim_lut)) {
		pr_err("%s, invalid tp or dim_lut\n", __func__);
		return;
	}

	if (unlikely(ilum < 0)) {
		pr_err("%s, luminance(%d) not found\n", __func__, luminance);
		return;
	}

	for (i = TP_V255; i >= TP_VT; i--) {
		for (c = 0; c < MAX_COLOR; c++) {
			value = dim_lut[ilum].tpout[i].center[c];
			if (i == TP_V255) {
				output[idst++] = ((value & 0x0100) ? 1 : 0);
				output[idst++] = value & 0x00FF;
				if (value > 511)
					pr_err("%s, error : exceed output range %s[%s] %d !!\n",
					       __func__, tp[i].name, color_name[c], value);
			} else if (i == TP_VT) {
				if (compact) {
					if (c == RED) {
						output[idst] &= ~(0xF << 4);
						output[idst] |= (value & 0xF) << 4;
					} else {
						output[idst] &= ~0xF;
						output[idst] |= value & 0xF;
						idst++;
					}
				} else {
					output[idst++] = value & 0xF;
				}
				if (value > 15)
					pr_err("%s, error : exceed output range %s[%s] %d !!\n",
					       __func__, tp[i].name, color_name[c], value);
			} else {
				output[idst++] = value & 0x00FF;
				if (value > 255)
					pr_err("%s, error : exceed output range %s[%s] %d !!\n",
					       __func__, tp[i].name, color_name[c], value);
			}
		}
	}
}

static void copy_tpout_hbm_center(struct dimming_info *dim_info, u32 luminance, u8 *output, bool compact)
{
	struct dimming_lut *dim_lut = dim_info->dim_lut;
	struct tp *tp = dim_info->tp;
	int i, c, value, ilum, idst = 0;
	int nr_tp = dim_info->nr_tp;
	s32 (*target_gamma_tbl)[MAX_COLOR];
	s32 (*output_gamma_tbl)[MAX_COLOR];

	if (unlikely(!tp || !dim_lut)) {
		pr_err("%s, invalid tp or dim_lut\n", __func__);
		return;
	}

	target_gamma_tbl = kzalloc(sizeof(s32) * nr_tp * MAX_COLOR, GFP_KERNEL);
	if (unlikely(!target_gamma_tbl)) {
		pr_err("%s. failed to allocate target_gamma_tbl\n", __func__);
		goto err;
	}

	output_gamma_tbl = kzalloc(sizeof(s32) * nr_tp * MAX_COLOR, GFP_KERNEL);
	if (unlikely(!output_gamma_tbl)) {
		pr_err("%s. failed to allocate output_gamma_tbl\n", __func__);
		goto err_alloc;
	}

	/* make target gamma table */
	ilum = find_luminance(dim_info, dim_info->target_luminance);
	if (unlikely(ilum < 0)) {
		pr_err("%s, target_luminance(%d) not found\n",
			__func__, dim_info->target_luminance);
		goto err_find;
	}

	for (i = 0; i < nr_tp; i++)
		for (c = 0; c < MAX_COLOR; c++)
			target_gamma_tbl[i][c] = dim_lut[ilum].tpout[i].center[c];

	gamma_table_interpolation(target_gamma_tbl, dim_info->hbm_gamma_tbl, output_gamma_tbl,
		dim_info->nr_tp, luminance - dim_info->target_luminance,
		dim_info->hbm_luminance - dim_info->target_luminance);

	for (i = TP_V255; i >= TP_VT; i--) {
		for (c = 0; c < MAX_COLOR; c++) {
			value = output_gamma_tbl[i][c];
			if (i == TP_V255) {
				output[idst++] = ((value & 0x0100) ? 1 : 0);
				output[idst++] = value & 0x00FF;
				if (value > 511)
					pr_err("%s, error : exceed output range %s[%s] %d !!\n",
					       __func__, tp[i].name, color_name[c], value);
			} else if (i == TP_VT) {
				if (compact) {
					if (c == RED) {
						output[idst] &= ~(0xF << 4);
						output[idst] |= (value & 0xF) << 4;
					} else {
						output[idst] &= ~0xF;
						output[idst] |= value & 0xF;
						idst++;
					}
				} else {
					output[idst++] = value & 0xF;
				}
				if (value > 15)
					pr_err("%s, error : exceed output range %s[%s] %d !!\n",
					       __func__, tp[i].name, color_name[c], value);
			} else {
				output[idst++] = value & 0x00FF;
				if (value > 255)
					pr_err("%s, error : exceed output range %s[%s] %d !!\n",
					       __func__, tp[i].name, color_name[c], value);
			}
		}
	}

err_find:
	kfree(output_gamma_tbl);
err_alloc:
	kfree(target_gamma_tbl);
err:
	return;
}

void get_dimming_gamma_compact(struct dimming_info *dim_info, u32 luminance, u8 *output)
{
	if (unlikely(!dim_info || !output)) {
		pr_err("%s, invalid dim_info or output\n", __func__);
		return;
	}

	if (luminance <= dim_info->target_luminance)
		copy_tpout_center(dim_info, luminance, output, true);
	else if (dim_info->hbm_luminance && luminance <= dim_info->hbm_luminance)
		copy_tpout_hbm_center(dim_info, luminance, output, true);
	else
		pr_warn("%s, out of range (hbm_luminance %d, luminance %d)\n",
			__func__, dim_info->hbm_luminance, luminance);
}

void get_dimming_gamma(struct dimming_info *dim_info, u32 luminance, u8 *output)
{
	if (unlikely(!dim_info || !output)) {
		pr_err("%s, invalid dim_info or output\n", __func__);
		return;
	}

	if (luminance <= dim_info->target_luminance)
		copy_tpout_center(dim_info, luminance, output, false);
	else if (dim_info->hbm_luminance && luminance <= dim_info->hbm_luminance)
		copy_tpout_hbm_center(dim_info, luminance, output, false);
	else
		pr_warn("%s, out of range (hbm_luminance %d, luminance %d)\n",
			__func__, dim_info->hbm_luminance, luminance);
}

static int generate_gamma_table(struct dimming_info *dim_info)
{
	int i, v, c, ilum;
	enum gamma_degree g_curve_degree;
	struct dimming_tp_output *tpout;
	struct dimming_lut *dim_lut = dim_info->dim_lut;
	struct gray_scale *gray_scale_lut = dim_info->gray_scale_lut;
	struct tp *tp = dim_info->tp;
	int NR_TP = dim_info->nr_tp;
	int TARGET_LUMINANCE = dim_info->target_luminance;
	enum gamma_degree TARGET_G_CURVE_DEGREE = dim_info->target_g_curve_degree;

	if (unlikely(!calc_gray_order)) {
		pr_err("%s, calc_gray_order not exist!!\n", __func__);
		return -EINVAL;
	}

	if (unlikely(!calc_vout_order)) {
		pr_err("%s, calc_vout_order not exist!!\n", __func__);
		return -EINVAL;
	}

	for (ilum = 0; ilum < NR_LUMINANCE; ilum++) {
		tpout = dim_lut[ilum].tpout;
		for (i = 0; i < NR_TP; i++) {
			v = calc_gray_order[i];
			g_curve_degree = dim_lut[ilum].g_curve_degree;

			if (v == TP_V255)
				tpout[v].L = (dim_lut[ilum].base_luminance) ?
					((u64)dim_lut[ilum].base_luminance << BIT_SHIFT) :
					(u64)TARGET_LUMINANCE *
					(u64)gamma_curve_value(TARGET_G_CURVE_DEGREE, dim_lut[ilum].base_gray);
			else if (v == TP_VT)
				tpout[v].L = (dim_lut[ilum].luminance < 251) ? 0 :
					((tpout[TP_V255].L *
					  (u64)gamma_curve_value(g_curve_degree, tp[v].level)) >> BIT_SHIFT);
			else
				tpout[v].L = (tpout[TP_V255].L *
					(u64)gamma_curve_value(g_curve_degree, tp[v].level)) >> BIT_SHIFT;

			/*
			 * In two special case, M_GRAY should be same with tp value
			 * 1. tp is VT (v == TP_VT)
			 * 2. tp's value is 1 (tp[v].level == 1)
			 */
			tpout[v].M_GRAY = (tp[v].level <= 1) ? tp[v].level :
				find_gray_scale_gamma_value(gray_scale_lut, tpout[v].L) +
				dim_lut[ilum].gray_scale_offset[v];

			if (tpout[v].M_GRAY > 255)
				tpout[v].M_GRAY = 255;

			for (c = 0; c < MAX_COLOR; c++) {
				dim_lut[ilum].tpout[v].vout[c] =
					(v == TP_VT) ? tp[v].vout[c] : gray_scale_lut[tpout[v].M_GRAY].vout[c];
			}
		}

		for (i = 0; i < NR_TP; i++) {
			v = calc_vout_order[i];
			for (c = 0; c < MAX_COLOR; c++) {
				dim_lut[ilum].tpout[v].center[c] =
					(v == TP_VT) ? tp[v].center[c] :
					mtp_vregout_to_offset(dim_info, v, (enum color)c, ilum);
			}
		}
	}

	return 0;
}

int alloc_dimming_info(struct dimming_info *dim_info, int nr_tp, int nr_luminance)
{
	dim_info->nr_tp = nr_tp;
	dim_info->nr_luminance = nr_luminance;

	return 0;
}

/*
 * dim_lut, tp, gray_scale_offset and rgb_color_offset are the tuning tables
 * of the panel variant, so only tpout and hbm_gamma_tbl are freed.
 */
void free_dimming_info(struct dimming_info *dim_info)
{
	int i;

	for (i = 0; i < dim_info->nr_luminance; i++)
		kfree(dim_info->dim_lut[i].tpout);
	kfree(dim_info->hbm_gamma_tbl);

	dim_info->nr_luminance = 0;
	dim_info->nr_tp = 0;
}

void prepare_dim_info(struct dimming_info *dim_info)
{
	NR_TP = dim_info->nr_tp;
	NR_LUMINANCE = dim_info->nr_luminance;
	TARGET_LUMINANCE = dim_info->target_luminance;
	TARGET_G_CURVE_DEGREE = dim_info->target_g_curve_degree;

	TP_VT = 0;
	TP_2ND = 1;
	TP_3RD = 2;
	TP_V255 = dim_info->nr_tp - 1;

	if (!calc_gray_order)
		calc_gray_order = generate_order(NR_TP - 1, 0, NR_TP - 2);

	if (!calc_vout_order)
		calc_vout_order = generate_order(0, NR_TP - 1, 1);
}

int init_dimming_info(struct dimming_info *dim_info, struct dimming_init_info *src)
{
	int ilum;

	memcpy(dim_info->vt_voltage, src->vt_voltage, sizeof(src->vt_voltage));
	dim_info->vregout = src->vregout;
	dim_info->nr_tp = src->nr_tp;
	dim_info->tp = src->tp;
	dim_info->nr_luminance = src->nr_luminance;
	dim_info->dim_lut = src->dim_lut;

	/* allocation of struct dimming_lut lut and members */
	for (ilum = 0; ilum < dim_info->nr_luminance; ilum++)
		dim_info->dim_lut[ilum].tpout = kzalloc(sizeof(struct dimming_tp_output) *
							dim_info->nr_tp, GFP_KERNEL);

	dim_info->target_luminance = src->target_luminance;
	dim_info->target_g_curve_degree = gamma_value_to_degree(src->target_gamma);
	if (unlikely(dim_info->target_g_curve_degree == GAMMA_NONE)) {
		pr_err("%s, invalid target gamma degree %d\n",
			__func__, dim_info->target_g_curve_degree);
		return -EINVAL;
	}

	return 0;
}

int init_dimming_mtp(struct dimming_info *dim_info, s32 (*mtp)[MAX_COLOR])
{
	int i, c;

	for (i = 0; i < dim_info->nr_tp; i++)
		for (c = 0; c < MAX_COLOR; c++)
			dim_info->tp[i].offset[c] = mtp[i][c];

	return 0;
}

int init_dimming_hbm_info(struct dimming_info *dim_info, s32 (*hbm_gamma_tbl)[MAX_COLOR],
			  u32 hbm_luminance)
{
	int i, c, nr_tp;

	if (unlikely(!dim_info || !hbm_gamma_tbl)) {
		pr_err("%s, invalid parameter\n", __func__);
		return -EINVAL;
	}

	if (unlikely(hbm_luminance < dim_info->target_luminance)) {
		pr_err("%s, out of range (hbm_luminance %d < target_luminance %d)\n",
			__func__, hbm_luminance, dim_info->target_luminance);
		return -EINVAL;
	}

	nr_tp = dim_info->nr_tp;
	if (!dim_info->hbm_gamma_tbl)
		dim_info->hbm_gamma_tbl = kzalloc(sizeof(s32) * nr_tp * MAX_COLOR, GFP_KERNEL);

	for (i = 0; i < dim_info->nr_tp; i++)
		for (c = 0; c < MAX_COLOR; c++)
			dim_info->hbm_gamma_tbl[i][c] = hbm_gamma_tbl[i][c];
	dim_info->hbm_luminance = hbm_luminance;

	return 0;
}

int process_dimming(struct dimming_info *dim_info)
{
	int i, c, v;
	struct tp *tp;
	s64 VREGOUT;

	if (unlikely(!dim_info)) {
		pr_err("%s, invalid dim_info\n", __func__);
		return -EINVAL;
	}
	VREGOUT = dim_info->vregout;
	prepare_dim_info(dim_info);

	tp = dim_info->tp;
	if (unlikely(!tp)) {
		pr_err("%s, tuning point not prepared\n", __func__);
		return -EINVAL;
	}

	/*
	 * Generate RGB voltage output table of TP using MTP(TP_VT ~ TP_V255).
	 */
	for (i = 0; i < dim_info->nr_tp; i++) {
		v = calc_vout_order[i];
		for (c = 0; c < MAX_COLOR; c++) {
			tp[v].vout[c] = mtp_offset_to_vregout(dim_info, v, (enum color)c);
			if (tp[v].vout[c] < 0)
				pr_warn("%s, invalid vout[%s][%s] %lld\n",
					__func__, tp[v].name, color_name[c], tp[v].vout[c]);
			dim_info->gray_scale_lut[tp[v].level].vout[c] =
				(v == TP_VT) ? (u64)VREGOUT : (u64)tp[v].vout[c];
		}
	}

	/*
	 * Fill in the all RGB voltate output of gray scale by interpolation.
	 */
	generate_gray_scale(dim_info);

	/*
	 * Generate mtp offset of given luminances.
	 */
	generate_gamma_table(dim_info);

	return 0;
}

/*
 * The MTP and HBM_GAMMA read-back registers pack the tuning-point offsets in a
 * word stream.
 */
int hbm_ctoi(s32 (*dst)[MAX_COLOR], u8 *hbm, int nr_tp)
{
	unsigned int i, c, pos = 1;

	for (i = nr_tp - 1; i > 0; i--) {
		for (c = 0; c < MAX_COLOR; c++) {
			if (i == nr_tp - 1)
				dst[i][c] = ((hbm[0] >> c) & 0x01) << 8;
			dst[i][c] |= hbm[pos++];
		}
	}

	dst[0][RED] = (hbm[pos] >> 4) & 0xF;
	dst[0][GREEN] = hbm[pos] & 0xF;
	dst[0][BLUE] = hbm[pos + 1] & 0xF;

	return 0;
}

void mtp_ctoi(s32 (*dst)[MAX_COLOR], char *src, int nr_tp)
{
	int i, c, pos = 0, sign;

	for (i = nr_tp - 1; i > 0; i--) {
		for (c = 0; c < MAX_COLOR; c++) {
			if (i == nr_tp - 1) {
				sign = (src[pos] & 0x01) ? -1 : 1;
				dst[i][c] = sign * src[pos + 1];
				pos += 2;
			} else {
				sign = (src[pos] & 0x80) ? -1 : 1;
				dst[i][c] = sign * (src[pos] & 0x7F);
				pos += 1;
			}
		}
	}
	dst[0][RED] = (src[pos] >> 4) & 0xF;
	dst[0][GREEN] = src[pos] & 0xF;
	dst[0][BLUE] = (src[pos + 1] >> 4) & 0xF;
}
