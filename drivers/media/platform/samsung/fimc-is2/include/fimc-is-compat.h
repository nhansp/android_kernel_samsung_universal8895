/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Exynos8895 FIMC-IS2 transplant: 4.4 kernel API names kept by the stock
 * sources, mapped to their 6.18 equivalents (rewrite/spec/L6B_multimedia.md
 * 5.3, 5.6.8). Force-included into every fimc-is2 object by the Makefile, so
 * the kernel headers whose declarations the macros below would collide with
 * are included first.
 */
#ifndef __FIMC_IS_COMPAT_H
#define __FIMC_IS_COMPAT_H

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kthread.h>
#include <linux/ktime.h>
#include <linux/minmax.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_qos.h>
#include <linux/random.h>
#include <linux/sched/clock.h>
#include <linux/slab.h>
#include <linux/time32.h>
#include <linux/timekeeping.h>
#include <linux/timer.h>
#include <linux/vmalloc.h>
#include <soc/samsung/exynos8895-mm-qos.h>
#include <media/exynos8895-compat.h>
#include "../fimc-is-fpsimd.h"

/*
 * Private core.ioctl codes for the removed subdev ops (5.6.8); G/S_CTRL are in
 * <media/exynos8895-compat.h>. Only the transplant's own subdevs use them.
 */
#define FIMC_IS_SUBDEV_G_EXT_CTRLS	_IOWR('V', BASE_VIDIOC_PRIVATE + 3, struct v4l2_ext_controls)
#define FIMC_IS_SUBDEV_S_EXT_CTRLS	_IOWR('V', BASE_VIDIOC_PRIVATE + 4, struct v4l2_ext_controls)
#define FIMC_IS_SUBDEV_S_PARM		_IOWR('V', BASE_VIDIOC_PRIVATE + 5, struct v4l2_streamparm)
#define FIMC_IS_SUBDEV_G_PARM		_IOWR('V', BASE_VIDIOC_PRIVATE + 6, struct v4l2_streamparm)

/* stock vb2 fence flag (S:include/uapi/linux/videodev2.h:905), cleared from QBUF as stock */
#define V4L2_BUF_FLAG_USE_SYNC		0x00008000

/* removed printk alias */
#define pr_warning(fmt, ...)		pr_warn(fmt, ##__VA_ARGS__)

/* 4.4 subdev name length (S:include/media/v4l2-subdev.h:677); 6.18 sd->name is longer */
#define V4L2_SUBDEV_NAME_SIZE		32

/* 4.4 gpio raw setter */
#define __gpio_set_value(g, v)		gpio_set_value(g, v)

/* struct timeval, do_gettimeofday, ns_to_timeval: <media/exynos8895-compat.h> */

/* timers (5.6.8): del_timer[_sync] became timer_delete[_sync] (A:include/linux/timer.h) */
#define del_timer(t)			timer_delete(t)
#define del_timer_sync(t)		timer_delete_sync(t)

/* kthread worker 4.4 names (5.6.8; A:include/linux/kthread.h:186,262,274) */
#define init_kthread_worker(w)		kthread_init_worker(w)
#define init_kthread_work(w, f)		kthread_init_work(w, f)
#define queue_kthread_work(w, k)	kthread_queue_work(w, k)
#define flush_kthread_work(k)		kthread_flush_work(k)
#define flush_kthread_worker(w)		kthread_flush_worker(w)

/* get_random_int -> get_random_u32 (A:include/linux/random.h:42) */
#define get_random_int()		get_random_u32()

/*
 * Samsung pm_qos classes -> the multimedia QoS shim (5.3; class map
 * INTCAM/CAM/DEVICE->INT/BUS->MIF/CLUSTERn/CPU_ONLINE_MIN).
 * <linux/pm_qos.h> is included above, so its own struct pm_qos_request is
 * declared before the rename.
 */
#define pm_qos_request			mm_qos_request
#define pm_qos_add_request		mm_qos_add_request
#define pm_qos_update_request		mm_qos_update_request
#define pm_qos_remove_request		mm_qos_remove_request
#define pm_qos_request_active		mm_qos_request_active
/* stock timeout variant: the timeout only bounded the boost; the value is applied as an update */
#define pm_qos_update_request_timeout(r, v, t)	mm_qos_update_request(r, v)

#define PM_QOS_INTCAM_THROUGHPUT	MM_QOS_INTCAM
#define PM_QOS_CAM_THROUGHPUT		MM_QOS_CAM
#define PM_QOS_CLUSTER0_FREQ_MIN	MM_QOS_CL0_MIN
#define PM_QOS_CLUSTER0_FREQ_MAX	MM_QOS_CL0_MAX
#define PM_QOS_CLUSTER1_FREQ_MIN	MM_QOS_CL1_MIN
#define PM_QOS_CLUSTER1_FREQ_MAX	MM_QOS_CL1_MAX
#define PM_QOS_DEVICE_THROUGHPUT	MM_QOS_INT
#define PM_QOS_BUS_THROUGHPUT		MM_QOS_MIF
#define PM_QOS_CPU_ONLINE_MIN		MM_QOS_CPU_ONLINE_MIN

/*
 * File I/O of the factory/diagnostic paths (5.6.8): 6.18 has no set_fs(); the
 * stock set_fs(KERNEL_DS)/vfs_read/vfs_write sequences become kernel_read/
 * kernel_write (A:include/linux/fs.h), the address-limit calls vanish.
 */
typedef struct { int unused; } mm_segment_t;
#define get_fs()			((mm_segment_t){ 0 })
#define set_fs(x)			do { (void)(x); } while (0)
#define KERNEL_DS			((mm_segment_t){ 0 })
#define USER_DS				((mm_segment_t){ 0 })
#define vfs_read(f, b, n, p)		kernel_read(f, (void *)(b), n, p)
#define vfs_write(f, b, n, p)		kernel_write(f, (const void *)(b), n, p)

#endif /* __FIMC_IS_COMPAT_H */
