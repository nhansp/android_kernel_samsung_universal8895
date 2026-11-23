/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Exynos8895 bus DVFS floors with the semantics of the vendor kernel's PM QoS
 * throughput classes, aggregated by the Exynos8895 bus devfreq driver.
 *
 * Every function may sleep. It may be called from another device's devfreq
 * target or governor and with dev_pm_qos_mtx held.
 */
#ifndef __SOC_SAMSUNG_EXYNOS8895_BUS_QOS_H
#define __SOC_SAMSUNG_EXYNOS8895_BUS_QOS_H

#include <linux/list.h>
#include <linux/types.h>
#include <linux/workqueue.h>

enum exynos_bus_qos_class {
	EXYNOS_BUS_QOS_MIF,	/* PM_QOS_BUS_THROUGHPUT */
	EXYNOS_BUS_QOS_INT,	/* PM_QOS_DEVICE_THROUGHPUT */
	EXYNOS_BUS_QOS_INTCAM,	/* PM_QOS_INTCAM_THROUGHPUT */
	EXYNOS_BUS_QOS_CAM,	/* PM_QOS_CAM_THROUGHPUT */
	EXYNOS_BUS_QOS_DISP,	/* PM_QOS_DISPLAY_THROUGHPUT */
	EXYNOS_BUS_QOS_NR
};

struct exynos_bus_qos_request {
	struct list_head node;
	struct delayed_work timeout;
	enum exynos_bus_qos_class cls;
	s32 khz;			/* 0 = no floor */
	bool active;
};

#if IS_ENABLED(CONFIG_ARM_EXYNOS8895_BUS_DEVFREQ)
void exynos_bus_qos_add_request(struct exynos_bus_qos_request *req,
				enum exynos_bus_qos_class cls, s32 khz);
void exynos_bus_qos_update_request(struct exynos_bus_qos_request *req, s32 khz);
void exynos_bus_qos_update_request_timeout(struct exynos_bus_qos_request *req,
					   s32 khz, unsigned long timeout_us);
void exynos_bus_qos_remove_request(struct exynos_bus_qos_request *req);
int exynos_bus_qos_request_active(struct exynos_bus_qos_request *req);
/* 1: the bus devfreq is live; 0: no such device or no devfreq; -EPROBE_DEFER: not probed yet */
int exynos_bus_qos_class_live(enum exynos_bus_qos_class cls);
#else
static inline void exynos_bus_qos_add_request(struct exynos_bus_qos_request *req,
					      enum exynos_bus_qos_class cls, s32 khz)
{
}

static inline void exynos_bus_qos_update_request(struct exynos_bus_qos_request *req,
						 s32 khz)
{
}

static inline void exynos_bus_qos_update_request_timeout(struct exynos_bus_qos_request *req,
							 s32 khz, unsigned long timeout_us)
{
}

static inline void exynos_bus_qos_remove_request(struct exynos_bus_qos_request *req)
{
}

static inline int exynos_bus_qos_request_active(struct exynos_bus_qos_request *req)
{
	return 0;
}

static inline int exynos_bus_qos_class_live(enum exynos_bus_qos_class cls)
{
	return 0;
}
#endif

#endif /* __SOC_SAMSUNG_EXYNOS8895_BUS_QOS_H */
