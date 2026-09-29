/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Exynos8895 multimedia QoS requests (rewrite/spec/L6B_multimedia.md 5.3).
 *
 * A stand-in for the Samsung pm_qos classes the stock 4.4 camera and MFC
 * drivers vote on (S:include/linux/pm_qos.h:13-45): one list of requests per
 * class, aggregated as stock pm_qos does (a MIN class takes the largest active
 * value, a MAX class the smallest) and applied to the 6.18 owner of the class.
 * Callers must be able to sleep, as with stock pm_qos (blocking notifiers).
 */
#ifndef __SOC_SAMSUNG_EXYNOS8895_MM_QOS_H
#define __SOC_SAMSUNG_EXYNOS8895_MM_QOS_H

#include <linux/list.h>
#include <linux/types.h>

struct device;

enum mm_qos_class {
	MM_QOS_INTCAM,		/* PM_QOS_INTCAM_THROUGHPUT: ACPM dvfs_intcam */
	MM_QOS_CAM,		/* PM_QOS_CAM_THROUGHPUT: ACPM dvfs_cam */
	MM_QOS_CL0_MIN,		/* PM_QOS_CLUSTER0_FREQ_MIN: cpus 0-3 */
	MM_QOS_CL0_MAX,		/* PM_QOS_CLUSTER0_FREQ_MAX */
	MM_QOS_CL1_MIN,		/* PM_QOS_CLUSTER1_FREQ_MIN: cpus 4-7 */
	MM_QOS_CL1_MAX,		/* PM_QOS_CLUSTER1_FREQ_MAX */
	MM_QOS_INT,		/* PM_QOS_DEVICE_THROUGHPUT: not applied */
	MM_QOS_MIF,		/* PM_QOS_BUS_THROUGHPUT: not applied */
	MM_QOS_CPU_ONLINE_MIN,	/* PM_QOS_CPU_ONLINE_MIN: not applied */
	MM_QOS_NR
};

struct mm_qos_request {
	struct list_head node;
	enum mm_qos_class cls;
	s32 value;
	bool active;
};

#if IS_ENABLED(CONFIG_EXYNOS8895_MM_QOS)
void mm_qos_add_request(struct mm_qos_request *r, enum mm_qos_class cls, s32 value);
void mm_qos_update_request(struct mm_qos_request *r, s32 value);
void mm_qos_remove_request(struct mm_qos_request *r);
int mm_qos_request_active(struct mm_qos_request *r);
int mm_qos_init(struct device *dev);
#else
static inline void mm_qos_add_request(struct mm_qos_request *r,
				      enum mm_qos_class cls, s32 value)
{
	r->cls = cls;
	r->value = value;
	r->active = true;
}
static inline void mm_qos_update_request(struct mm_qos_request *r, s32 value)
{
	r->value = value;
}
static inline void mm_qos_remove_request(struct mm_qos_request *r)
{
	r->active = false;
}
static inline int mm_qos_request_active(struct mm_qos_request *r)
{
	return r->active;
}
static inline int mm_qos_init(struct device *dev)
{
	return 0;
}
#endif

#endif /* __SOC_SAMSUNG_EXYNOS8895_MM_QOS_H */
