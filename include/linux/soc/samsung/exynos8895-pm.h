/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Exynos 8895 power management: interfaces shared by the PMU CPU/cluster hooks
 * (drivers/soc/samsung/exynos8895-pmu.c), the cpuidle driver
 * (drivers/cpuidle/cpuidle-exynos8895.c), the system PM driver
 * (drivers/soc/samsung/exynos8895-pm.c) and the samsung pinctrl EINT wake mask.
 */
#ifndef __LINUX_SOC_SAMSUNG_EXYNOS8895_PM_H
#define __LINUX_SOC_SAMSUNG_EXYNOS8895_PM_H

#include <linux/atomic.h>
#include <linux/types.h>

/*
 * PSCI CPU_SUSPEND power_state values. ID [15:0], TYPE bit 16, AFFL [25:24]
 * (A:include/uapi/linux/psci.h:75-80). Stock packs the Exynos modes with TYPE 0
 * and still passes cpu_resume (S:arch/arm64/kernel/psci.c:212-255).
 */
#define E8895_PSCI_C2		0x00010000	/* FDT:4363 arm,psci-suspend-param */
#define E8895_PSCI_CPD		0x01000000	/* PSCI_CLUSTER_SLEEP 128: pack(0,0,1), S:arch/arm64/kernel/psci.c:233-235 */
#define E8895_PSCI_SICD		0x00000001	/* PSCI_SYSTEM_IDLE 129: pack(1,0,0), S:arch/arm64/kernel/psci.c:236-239 */
#define E8895_PSCI_SLEEP	0x03000000	/* PSCI_SYSTEM_SLEEP 133: pack(0,0,3), S:arch/arm64/kernel/psci.c:246-248; FDT:4418 <0x85> */

struct e8895_idle_stats {		/* read by the L7 heartbeat printer */
	atomic_t c2[8];			/* successful C2 (any flavour) per logical CPU */
	atomic_t cpd;			/* cluster 0 power-downs completed */
	atomic_t sicd;			/* SICD entries completed */
	atomic_t sicd_try;		/* SICD candidate (all CPUs idle long enough) */
	atomic_t sicd_veto;		/* candidate refused by a veto */
	atomic_t conf_fix;		/* core CONFIGURATION/DD found off after a successful C2 */
	atomic_t ubt;			/* UART_BT pad releases */
};
extern struct e8895_idle_stats e8895_stats;

/* exynos8895-pm.c */
int e8895_psci_suspend(u32 power_state);	/* 0 after a real power-down and wake, <0 on early return */
int e8895_sicd_prepare(void);			/* 0: armed, caller passes E8895_PSCI_SICD; <0: vetoed */
void e8895_sicd_exit(bool early_wakeup);
const char *e8895_sicd_last_veto(void);

/* exynos8895-pmu.c (CPU and cluster power hooks) */
void e8895_pmu_cpu_down(unsigned int cpu);
void e8895_pmu_cpu_up(unsigned int cpu);
bool e8895_pmu_cpu_conf_ok(unsigned int cpu);
u32 e8895_pmu_cpu_status(unsigned int cpu);		/* DEBUG */
void e8895_pmu_cluster_down(unsigned int cpu);
int e8895_pmu_cluster_up(unsigned int cpu);
bool e8895_pmu_cluster_is_on(unsigned int cpu);
unsigned int e8895_phycluster(unsigned int cpu);
bool e8895_pmu_cpu_hooks_ready(void);		/* true once the DD ioremap and the cpuhp states succeeded */

/* pinctrl-exynos.c */
u32 exynos_pinctrl_get_eint_wake_mask(void);

#endif /* __LINUX_SOC_SAMSUNG_EXYNOS8895_PM_H */
