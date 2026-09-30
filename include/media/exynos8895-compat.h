/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Exynos8895 multimedia transplants (MFC, FIMC-IS2, SMFC, MSCL): the few stock
 * 4.4 names they keep that 6.18 does not provide (rewrite/spec/L6B_multimedia.md
 * 5 intro and 5.5-5.9). Userspace-visible values are bit-exact with stock.
 */
#ifndef __MEDIA_EXYNOS8895_COMPAT_H
#define __MEDIA_EXYNOS8895_COMPAT_H

#include <linux/time32.h>
#include <linux/timekeeping.h>
#include <linux/videodev2.h>

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

/*
 * Samsung GIC multi-target flag: 6.18 delivers each SPI to one CPU (spec 5.6.4).
 */
#ifndef IRQF_GIC_MULTI_TARGET
#define IRQF_GIC_MULTI_TARGET			0
#endif

/*
 * Subdev control routing via core.ioctl (spec 5.6.8).
 */
#define FIMC_IS_SUBDEV_G_CTRL			_IOWR('V', BASE_VIDIOC_PRIVATE + 1, struct v4l2_control)
#define FIMC_IS_SUBDEV_S_CTRL			_IOWR('V', BASE_VIDIOC_PRIVATE + 2, struct v4l2_control)

/*
 * Compat timeval and do_gettimeofday (spec 5.6.8).
 */
#define timeval __kernel_old_timeval

static inline void do_gettimeofday(struct __kernel_old_timeval *tv)
{
	struct timespec64 ts;

	ktime_get_real_ts64(&ts);
	tv->tv_sec = ts.tv_sec;
	tv->tv_usec = ts.tv_nsec / NSEC_PER_USEC;
}

static inline struct __kernel_old_timeval ns_to_timeval(s64 nsec)
{
	return ns_to_kernel_old_timeval(nsec);
}

#endif /* __MEDIA_EXYNOS8895_COMPAT_H */
