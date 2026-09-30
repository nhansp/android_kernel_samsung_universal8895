/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Exynos8895 multimedia transplants (MFC, FIMC-IS2, SMFC, MSCL): the few stock
 * 4.4 names they keep that 6.18 does not provide (rewrite/spec/L6B_multimedia.md
 * 5 intro and 5.5-5.9). Userspace-visible values are bit-exact with stock.
 */
#ifndef __MEDIA_EXYNOS8895_COMPAT_H
#define __MEDIA_EXYNOS8895_COMPAT_H

/*
 * Video node numbers (S:include/video/videonode.h:16-24): the OMX, camera HAL,
 * libhwjpeg and the HWC open these fixed /dev/videoN numbers.
 */
#define EXYNOS_VIDEONODE_MFC_DEC		6
#define EXYNOS_VIDEONODE_MFC_ENC		7
#define EXYNOS_VIDEONODE_MFC_DEC_DRM		8
#define EXYNOS_VIDEONODE_MFC_ENC_DRM		9

#define EXYNOS_VIDEONODE_SCALER(x)		(50 + x)

/* 100 ~ 149 is used by FIMC-IS */
#define EXYNOS_VIDEONODE_FIMC_IS		(100)

#endif /* __MEDIA_EXYNOS8895_COMPAT_H */
