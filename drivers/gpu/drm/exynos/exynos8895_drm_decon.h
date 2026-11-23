/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Exynos8895 DECON0: interface for in-kernel clients of the CRTC, such as
 * the S3CFB front-end.
 */
#ifndef _EXYNOS8895_DRM_DECON_H_
#define _EXYNOS8895_DRM_DECON_H_

#include <linux/ktime.h>
#include <linux/types.h>

#include <drm/drm_rect.h>
#include <drm/exynos8895_panel.h>

struct device;
struct device_node;
struct drm_connector;
struct drm_crtc;
struct drm_device;
struct drm_plane;
struct drm_plane_state;

/* plane index = DPP channel: G0 G1 VG0 VG1 VGF0 VGF1 */
#define EXYNOS8895_DECON_NR_PLANES	6
#define EXYNOS8895_DECON_NR_WINDOWS	6

struct exynos8895_decon_handle {
	struct device *dev;
	struct drm_device *drm;
	struct drm_crtc *crtc;
	struct drm_plane *planes[EXYNOS8895_DECON_NR_PLANES];
	struct drm_connector *connector;
};

/* a window filled with one colour, without DMA */
struct exynos8895_color_win {
	struct drm_rect dst;
	u32 argb;		/* A[31:24] R[23:16] G[15:8] B[7:0]; alpha is forced to 0xff */
	u8 plane;		/* channel the composer named, by its DECON channel code */
	u16 alpha;		/* plane alpha, 16 bit like the DRM "alpha" property */
	u16 blend;		/* DRM_MODE_BLEND_PIXEL_NONE, _PREMULTI or _COVERAGE */
};

#if IS_ENABLED(CONFIG_DRM_EXYNOS8895_DECON)
int exynos8895_decon_get(struct device_node *np, struct exynos8895_decon_handle *h);
int exynos8895_decon_plane_set_win(struct drm_plane_state *st, int win, bool afbc);
int exynos8895_decon_set_color_win(struct device *dev, unsigned int win,
				   const struct exynos8895_color_win *cw);
void exynos8895_decon_set_vsync_cb(struct device *dev,
				   void (*cb)(void *data, ktime_t ts), void *data);
void exynos8895_decon_set_frame_start_cb(struct device *dev,
					 void (*cb)(void *data), void *data);
void exynos8895_decon_set_oneshot(struct device *dev, bool oneshot);
int exynos8895_decon_panel_power(enum exynos8895_panel_power req, bool active);
void exynos8895_decon_set_dsu(struct device *dev, const struct dsu_info *dsu);
#endif

#endif /* _EXYNOS8895_DRM_DECON_H_ */
