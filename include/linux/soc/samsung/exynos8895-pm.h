/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Exynos8895 power management interfaces shared by the PMU CPU hooks, the
 * cpuidle driver, the system PM driver and the pin controller.
 */
#ifndef __LINUX_SOC_SAMSUNG_EXYNOS8895_PM_H
#define __LINUX_SOC_SAMSUNG_EXYNOS8895_PM_H

#include <linux/types.h>

/* exynos8895-pmu.c */
void e8895_pmu_cpu_down(unsigned int cpu);
void e8895_pmu_cpu_up(unsigned int cpu);
bool e8895_pmu_cpu_conf_ok(unsigned int cpu);
void e8895_pmu_cluster_down(unsigned int cpu);
int e8895_pmu_cluster_up(unsigned int cpu);
unsigned int e8895_phycluster(unsigned int cpu);
bool e8895_pmu_cpu_hooks_ready(void);

#endif /* __LINUX_SOC_SAMSUNG_EXYNOS8895_PM_H */
