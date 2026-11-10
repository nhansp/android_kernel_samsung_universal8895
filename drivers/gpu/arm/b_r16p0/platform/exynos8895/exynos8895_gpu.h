/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Exynos 8895 kbase platform state, shared by mali_kbase_config_exynos8895.c
 * (spec L6A_graphics.md 5.1.5) and gpu_sysfs.c (the port of the stock Samsung
 * platform sysfs that the LineageOS power HAL writes, brief l6agpu). Not
 * included by mali_kbase_defs.h, unlike mali_kbase_exynos8895.h, because it
 * pulls in the bus QoS and PM QoS headers.
 */

#ifndef _EXYNOS8895_GPU_H_
#define _EXYNOS8895_GPU_H_

#include <linux/clk.h>
#include <linux/kobject.h>
#include <linux/mutex.h>
#include <linux/pm_qos.h>
#include <linux/types.h>

#include <soc/samsung/exynos8895-bus-qos.h>
#include <soc/samsung/exynos8895-mm-qos.h>

struct kbase_device;
struct kbase_pm_policy;

struct exynos8895_gpu {
	struct kbase_device *kbdev;
	struct clk *clk;
	struct mutex lock;		/* cur_rate, want_rate */
	unsigned long cur_rate, want_rate;
	u32 autosuspend_ms;
	struct exynos_bus_qos_request mif_req;
	struct mm_qos_request cl0_min_req;	/* little cluster min */
	struct mm_qos_request cl1_max_req;	/* big cluster max */

	/* The sysfs state of gpu_sysfs.c. Every clock is in kHz, as the stock
	 * DVFS table (S:drivers/gpu/arm/b_r16p0/platform/exynos/gpu_dvfs_api.c:562-573
	 * gpu_dvfs_get_clock) and the powerhint.json values
	 * (recon/trees/legacy8895/android_device_samsung_universal8895-common/
	 * configs/power/powerhint.json:101-121) are. */
	struct mutex sysfs_lock;	/* the fields below only. Never held across a
					 * PM QoS or a PM policy call: the PM QoS
					 * notifier runs the kbase devfreq target,
					 * which takes gpu->lock
					 * (mali_kbase_config_exynos8895.c:76). */
	unsigned int *level_khz;	/* the DVFS table, ascending */
	int nlevels;			/* 0 = no OPP table, no sysfs */
	int lock_min_khz;		/* stock min_lock, 0 = unlocked */
	int lock_min_input;		/* stock user_min_lock_input */
	int lock_max_khz;		/* stock max_lock, 0 = unlocked */
	int lock_max_input;		/* stock user_max_lock_input */
	int pin_khz;			/* the stock "clock" node, 0 = off */
	const struct kbase_pm_policy *pin_policy;	/* restored by pin_khz = 0 */
	struct dev_pm_qos_request min_qos;		/* the min_lock */
	struct dev_pm_qos_request max_qos;		/* the max_lock */
	struct dev_pm_qos_request pin_min_qos;		/* the "clock" node */
	struct dev_pm_qos_request pin_max_qos;
	struct kobject *gpu_kobj;			/* /sys/kernel/gpu */
	bool sysfs_ready;				/* create_sysfs() succeeded */
};

/* gpu_sysfs.c */
int exynos8895_gpu_create_sysfs(struct exynos8895_gpu *gpu);
void exynos8895_gpu_delete_sysfs(struct exynos8895_gpu *gpu);

#endif /* _EXYNOS8895_GPU_H_ */
