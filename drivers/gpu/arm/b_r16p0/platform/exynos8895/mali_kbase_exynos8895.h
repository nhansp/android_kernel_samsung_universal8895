/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Exynos 8895 kbase platform: the vendor interface the r16p0 core expects
 * from Samsung's platform/exynos (spec L6A_graphics.md 5.1.2), which is not
 * ported. Included by mali_kbase_defs.h in place of
 * K:platform/exynos/gpu_integration_defs.h (K: = the stock
 * drivers/gpu/arm/b_r16p0).
 */

#ifndef _MALI_KBASE_EXYNOS8895_H_
#define _MALI_KBASE_EXYNOS8895_H_

#include <linux/types.h>

struct kbase_context;
struct kbase_device;
struct seq_file;
struct devfreq_table_interactive_data;

/* K:platform/exynos/gpu_integration_defs.h:26-30 */
/* kctx initialized with zero from vzalloc, so initialized value required only */
#define CTX_UNINITIALIZED 0x0
#define CTX_INITIALIZED 0x1
#define CTX_DESTROYED 0x2
#define CTX_NAME_SIZE 32

/* K:platform/exynos/gpu_integration_defs.h:35-40 */
/* MALI_SEC_INTEGRATION */
#define KBASE_PM_TIME_SHIFT			8

/* MALI_SEC_INTEGRATION */
#define MEM_FREE_LIMITS 16384
#define MEM_FREE_DEFAULT 16384

/*
 * K:platform/exynos/gpu_integration_defs.h:49-63. Every member stays NULL
 * here; the core tests each one before calling it.
 */
struct kbase_vendor_callbacks {
	void (*create_context)(void *ctx);
	void (*destroy_context)(void *ctx);
	void (*pm_metrics_init)(void *dev);
	void (*pm_metrics_term)(void *dev);
	void (*cl_boost_init)(void *dev);
	void (*cl_boost_update_utilization)(void *dev, void *atom, u64 microseconds_spent);
	int (*get_core_mask)(void *dev);
	int (*init_hw)(void *dev);
	void (*debug_pagetable_info)(void *ctx, u64 vaddr);
	void (*jd_done_worker)(void *dev);
	void (*update_status)(void *dev, char *str, u32 val);
	bool (*mem_profile_check_kctx)(void *ctx);
	int (*register_dump)(void);
};

/*
 * Provided by mali_kbase_config_exynos8895.c (5.1.5); prototypes as stock
 * (K:platform/exynos/gpu_integration_defs.h:43,47, gpu_control.c:79,
 * mali_kbase_platform.h:316).
 */
int gpu_vendor_dispatch(struct kbase_context *kctx, u32 flags);
int gpu_is_power_on(void);
int gpu_memory_seq_show(struct seq_file *sfile, void *data);
struct kbase_device *gpu_get_device_structure(void);

/* The stock DVFS table and interactive_info as governor data for kbase devfreq */
extern struct devfreq_table_interactive_data exynos8895_gpu_dvfs_data;

/* The G3D rate for kbase devfreq, applied only with pd_g3d on (L6A 5.1.4) */
int exynos8895_gpu_set_rate(struct kbase_device *kbdev, unsigned long rate);

#endif /* _MALI_KBASE_EXYNOS8895_H_ */
