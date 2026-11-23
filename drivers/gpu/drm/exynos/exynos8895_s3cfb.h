/* SPDX-License-Identifier: GPL-2.0-only */
/* Exynos8895 S3CFB front-end: entry points called by exynos_drm_drv.c */
#ifndef _EXYNOS8895_S3CFB_H_
#define _EXYNOS8895_S3CFB_H_

struct drm_device;

#if IS_ENABLED(CONFIG_DRM_EXYNOS8895_S3CFB)
void exynos8895_s3cfb_init(struct drm_device *drm);
void exynos8895_s3cfb_fini(struct drm_device *drm);
#else
static inline void exynos8895_s3cfb_init(struct drm_device *drm)
{
}

static inline void exynos8895_s3cfb_fini(struct drm_device *drm)
{
}
#endif

#endif /* _EXYNOS8895_S3CFB_H_ */
