/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Exynos8895 bus traffic shaper (BTS): bandwidth -> MIF/INT floors and the
 * scenario API (stock S:include/soc/samsung/bts.h:14-106, spec L2b 5.5.2).
 *
 * Copyright (c) 2014 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 */

#ifndef __EXYNOS_BTS_H_
#define __EXYNOS_BTS_H_

#include <linux/types.h>

/*
 * The types are visible whenever the bandwidth part is built
 * (CONFIG_EXYNOS8895_BTS_BW); stock hid them behind CONFIG_EXYNOS8895_BTS.
 */
#define BUS_WIDTH		16
#define DISP_UTIL		75

enum bts_scen_type {
	BS_DEFAULT,
	BS_MIF_CHANGE,
	BS_MFC_UHD,
	BS_MFC_UHD_10BIT,
	BS_G3D_PEFORMANCE,
	BS_DP_DEFAULT,
	BS_CAMERA_DEFAULT,
	BS_MAX,
};

enum bts_bw_type {
	/* RT */
	BTS_BW_DECON0,
	BTS_BW_DECON1,
	BTS_BW_DECON2,
	BTS_BW_CAMERA,
	BTS_BW_AUDIO,
	BTS_BW_CP,
	/* non-RT */
	BTS_BW_G2D,
	BTS_BW_MFC,
	BTS_BW_MCSL,
	BTS_BW_IVA,
	BTS_BW_DSP,
	BTS_BW_MAX,
	BTS_BW_RT = BTS_BW_G2D,
};

enum bts_dpp_type {
	BTS_DPP0,
	BTS_DPP1,
	BTS_DPP2,
	BTS_DPP3,
	BTS_DPP4,
	BTS_DPP5,
	BTS_DPP_MAX,
};

enum bts_dpu_type {
	BTS_DPU0,
	BTS_DPU1,
	BTS_DPU2,
	BTS_DPU_MAX,
};

struct bts_layer_position {
	unsigned int x1;
	/* x2 = x1 + width */
	unsigned int x2;
	unsigned int y1;
	/* y2 = y1 + height */
	unsigned int y2;
};

struct bts_dpp_info {
	bool used;
	unsigned int bpp;
	unsigned int src_h;
	unsigned int src_w;
	struct bts_layer_position dst;
	unsigned int bw;
};

struct bts_decon_info {
	struct bts_dpp_info dpp[BTS_DPP_MAX];
	/* Khz */
	unsigned int vclk;
	unsigned int lcd_w;
	unsigned int lcd_h;
};

struct bts_bw {
	unsigned int peak;
	unsigned int read;
	unsigned int write;
};

/*
 * Stated deviation (spec L2b V-8): stock's stubs were statement macros, which
 * break "bw = bts_calc_bw(...)"; these are static inline functions. Callers may
 * sleep (class mutex, media_mutex): process context only, never under a spinlock.
 */
#if IS_ENABLED(CONFIG_EXYNOS8895_BTS_BW)
/* bandwidth (KB/s) */
void bts_update_bw(enum bts_bw_type type, struct bts_bw bw);
unsigned int bts_calc_bw(enum bts_bw_type type, void *data);
#else
static inline void bts_update_bw(enum bts_bw_type type, struct bts_bw bw)
{
}

static inline unsigned int bts_calc_bw(enum bts_bw_type type, void *data)
{
	return 0;
}
#endif

#if IS_ENABLED(CONFIG_EXYNOS8895_BTS)
void bts_update_scen(enum bts_scen_type type, unsigned int val);
#else
static inline void bts_update_scen(enum bts_scen_type type, unsigned int val)
{
}
#endif

#if IS_ENABLED(CONFIG_EXYNOS8895_BTS) && IS_ENABLED(CONFIG_GREATLTE_HB)
/* DEBUG: heartbeat "bts" row (spec L2b 1.2), lock-free */
int bts_hb(char *buf, size_t size);
#endif

#endif /* __EXYNOS_BTS_H_ */
