/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Exynos8895 power management interfaces shared by the PMU CPU hooks, the
 * cpuidle driver, the system PM driver and the pin controller.
 */
#ifndef __LINUX_SOC_SAMSUNG_EXYNOS8895_PM_H
#define __LINUX_SOC_SAMSUNG_EXYNOS8895_PM_H

#include <linux/types.h>

/*
 * PSCI CPU_SUSPEND power_state values of the EL3 monitor. The cluster and
 * system states have the StateType bit clear, yet the core may lose its
 * context: they are entered with cpu_resume as the resume address.
 */
#define E8895_PSCI_C2		0x00010000
#define E8895_PSCI_CPD		0x01000000
#define E8895_PSCI_SICD		0x00000001
#define E8895_PSCI_SLEEP	0x03000000

/* exynos8895-pm.c */
int e8895_psci_suspend(u32 power_state);	/* 0 after a power-down and wakeup */
int e8895_sicd_prepare(void);			/* 0: armed for E8895_PSCI_SICD */
void e8895_sicd_exit(bool early_wakeup);

/* exynos8895-pmu.c */
void e8895_pmu_cpu_down(unsigned int cpu);
void e8895_pmu_cpu_up(unsigned int cpu);
bool e8895_pmu_cpu_conf_ok(unsigned int cpu);
void e8895_pmu_cluster_down(unsigned int cpu);
int e8895_pmu_cluster_up(unsigned int cpu);
unsigned int e8895_phycluster(unsigned int cpu);
bool e8895_pmu_cpu_hooks_ready(void);

/* pinctrl-exynos.c */
u32 exynos_pinctrl_get_eint_wake_mask(void);

#endif /* __LINUX_SOC_SAMSUNG_EXYNOS8895_PM_H */
