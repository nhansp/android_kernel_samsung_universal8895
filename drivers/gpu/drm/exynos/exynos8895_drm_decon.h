/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Exynos 8895 DECON0: interface for in-kernel clients of the CRTC, such as
 * the S3CFB front-end of the stock composer (L6A REQ-L5-3/4/7).
 */
#ifndef _EXYNOS8895_DRM_DECON_H_
#define _EXYNOS8895_DRM_DECON_H_

#include <linux/ktime.h>
#include <linux/types.h>

#include <drm/drm_rect.h>

struct device;
struct device_node;
struct drm_connector;
struct drm_crtc;
struct drm_device;
struct drm_plane;
struct drm_plane_state;

/* plane index = the vendor enum decon_idma_type: G0 G1 VG0 VG1 VGF0 VGF1 */
#define EXYNOS8895_DECON_NR_PLANES	6
#define EXYNOS8895_DECON_NR_WINDOWS	6

struct exynos8895_decon_handle {
	struct device *dev;
	struct drm_device *drm;
	struct drm_crtc *crtc;
	struct drm_plane *planes[EXYNOS8895_DECON_NR_PLANES];
	struct drm_connector *connector;
};

/* a window filled with one colour, without DMA (vendor DECON_WIN_STATE_COLOR) */
struct exynos8895_color_win {
	struct drm_rect dst;
	u32 argb;		/* A[31:24] R[23:16] G[15:8] B[7:0]; alpha is forced to 0xff */
	u8 plane;		/* channel the composer named; only its DECON channel code is used */
	u16 alpha;		/* plane alpha, 16 bit like the DRM "alpha" property */
	u16 blend;		/* DRM_MODE_BLEND_PIXEL_NONE, _PREMULTI or _COVERAGE */
};

#if IS_ENABLED(CONFIG_DRM_EXYNOS8895_DECON)
int exynos8895_decon_get(struct device_node *np, struct exynos8895_decon_handle *h);
int exynos8895_decon_plane_set_win(struct drm_plane_state *st, int win, bool afbc);
int exynos8895_decon_set_color_win(struct device *dev, unsigned int win,
				   const struct exynos8895_color_win *cw);
#endif

#endif /* _EXYNOS8895_DRM_DECON_H_ */
