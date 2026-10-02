/*
 * Exynos V4L2 media IDs used by the MSCL scaler driver.
 *
 * 6.18 has no linux/videodev2_exynos_media.h or video/videonode.h; the IDs
 * below are copied verbatim from the stock 4.4 tree so the userspace ABI
 * (ioctls, pixel formats, buffer geometry, /dev/video50) stays bit-exact.
 * Each value cites its stock line.
 */

#ifndef SCALER_EXYNOS_MEDIA_H_
#define SCALER_EXYNOS_MEDIA_H_

#include <linux/videodev2.h>

/* CID base for Exynos controls (USER_CLASS) */
#define V4L2_CID_EXYNOS_BASE		(V4L2_CTRL_CLASS_USER | 0x2000) /* S:include/uapi/linux/videodev2_exynos_media.h:24 */

/* for color space conversion equation selection */
#define V4L2_CID_CSC_EQ			(V4L2_CID_EXYNOS_BASE + 101) /* S:include/uapi/linux/videodev2_exynos_media.h:31 */
#define V4L2_CID_CSC_RANGE		(V4L2_CID_EXYNOS_BASE + 102) /* S:include/uapi/linux/videodev2_exynos_media.h:32 */

/* for DRM playback scenario */
#define V4L2_CID_CONTENT_PROTECTION	(V4L2_CID_EXYNOS_BASE + 201) /* S:include/uapi/linux/videodev2_exynos_media.h:35 */

/*
 *	V I D E O   I M A G E   F O R M A T
 */
/* 1 plane -- one Y, one Cb + Cr interleaved, non contiguous  */
#define V4L2_PIX_FMT_NV12N		v4l2_fourcc('N', 'N', '1', '2') /* S:include/uapi/linux/videodev2_exynos_media.h:41 */

/* 1 plane -- one Y, one Cb, one Cr, non contiguous */
#define V4L2_PIX_FMT_YUV420N		v4l2_fourcc('Y', 'N', '1', '2') /* S:include/uapi/linux/videodev2_exynos_media.h:45 */

/* 1 plane -- 8bit Y, 2bit Y, 8bit Cb + Cr interleaved, 2bit Cb + Cr interleaved, non contiguous */
#define V4L2_PIX_FMT_NV12N_10B		v4l2_fourcc('B', 'N', '1', '2') /* S:include/uapi/linux/videodev2_exynos_media.h:48 */

/* helper macros */
#ifndef __ALIGN_UP
#define __ALIGN_UP(x, a)		(((x) + ((a) - 1)) & ~((a) - 1)) /* S:include/uapi/linux/videodev2_exynos_media.h:51-53 */
#endif

#define NV12N_Y_SIZE(w, h)		(__ALIGN_UP((w), 16) * __ALIGN_UP((h), 16) + 256) /* S:include/uapi/linux/videodev2_exynos_media.h:55 */
#define NV12N_CBCR_SIZE(w, h)		(__ALIGN_UP((__ALIGN_UP((w), 16) * (__ALIGN_UP((h), 16) / 2) + 256), 16)) /* S:include/uapi/linux/videodev2_exynos_media.h:56 */
#define NV12N_CBCR_BASE(base, w, h)	((base) + NV12N_Y_SIZE((w), (h))) /* S:include/uapi/linux/videodev2_exynos_media.h:57 */
#define NV12N_10B_Y_2B_SIZE(w,h)	((__ALIGN_UP((w) / 4, 16) * __ALIGN_UP((h), 16) + 64)) /* S:include/uapi/linux/videodev2_exynos_media.h:58 */
#define NV12N_10B_CBCR_2B_SIZE(w,h)	((__ALIGN_UP((w) / 4, 16) * (__ALIGN_UP((h), 16) / 2) + 64)) /* S:include/uapi/linux/videodev2_exynos_media.h:59 */
#define NV12N_10B_CBCR_BASE(base,w,h)	((base) + NV12N_Y_SIZE((w), (h)) + NV12N_10B_Y_2B_SIZE((w), (h))) /* S:include/uapi/linux/videodev2_exynos_media.h:60 */

#define YUV420N_Y_SIZE(w, h)		(__ALIGN_UP((w), 16) * __ALIGN_UP((h), 16) + 256) /* S:include/uapi/linux/videodev2_exynos_media.h:62 */
#define YUV420N_CB_SIZE(w, h)		(__ALIGN_UP((__ALIGN_UP((w) / 2, 16) * (__ALIGN_UP((h), 16) / 2) + 256), 16)) /* S:include/uapi/linux/videodev2_exynos_media.h:63 */
#define YUV420N_CR_SIZE(w, h)		(__ALIGN_UP((__ALIGN_UP((w) / 2, 16) * (__ALIGN_UP((h), 16) / 2) + 256), 16)) /* S:include/uapi/linux/videodev2_exynos_media.h:64 */
#define YUV420N_CB_BASE(base, w, h)	((base) + YUV420N_Y_SIZE((w), (h))) /* S:include/uapi/linux/videodev2_exynos_media.h:65 */
#define YUV420N_CR_BASE(base, w, h)	(YUV420N_CB_BASE((base), (w), (h)) + YUV420N_CB_SIZE((w), (h))) /* S:include/uapi/linux/videodev2_exynos_media.h:66 */

/* Video node number for scaler instance x: /dev/video50 (S:include/video/videonode.h:21) */
#define EXYNOS_VIDEONODE_SCALER(x)		(50 + x)

#endif /* SCALER_EXYNOS_MEDIA_H_ */
