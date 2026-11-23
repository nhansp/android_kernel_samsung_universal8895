// SPDX-License-Identifier: GPL-2.0-only
/*
 * Exynos8895 cpuidle: WFI and C2, upgraded to a Mongoose cluster power-down
 * (CPD) or to system idle clock-down (SICD) when the other CPUs allow it.
 *
 * The firmware takes PSCI 0.1 power_state values in the Exynos encoding, and
 * the cluster and system power-down are prepared in the PMU by the kernel
 * before the SMC, which cpuidle-psci cannot express.
 */

#include <linux/cpu_cooling.h>
#include <linux/cpu_pm.h>
#include <linux/cpufreq.h>
#include <linux/cpuidle.h>
#include <linux/init.h>
#include <linux/ktime.h>
#include <linux/minmax.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/reboot.h>
#include <linux/slab.h>
#include <linux/smp.h>
#include <linux/soc/samsung/exynos8895-pm.h>
#include <linux/spinlock.h>
#include <asm/proc-fns.h>

#include "dt_idle_states.h"

#define E8895_CPD_RES_NS	(3000 * NSEC_PER_USEC)
#define E8895_SICD_RES_NS	(3000 * NSEC_PER_USEC)

static bool cpd_enabled = true;
static bool sicd_enabled = true;
module_param(cpd_enabled, bool, 0644);
module_param(sicd_enabled, bool, 0644);
/* SICD from s2idle; the vendor kernel had no s2idle, so it is opt-in */
static bool sicd_s2idle;
module_param(sicd_s2idle, bool, 0644);

static DEFINE_RAW_SPINLOCK(c2_lock);
static struct cpumask c2_mask;		/* CPUs inside C2 */
static bool cluster_down[2];
static bool sicd_entered;
static bool cpd_blocking;
static struct cpuidle_driver e8895_drv[2];	/* one per physical cluster */

static bool e8895_boot_cluster(unsigned int cpu)
{
	return e8895_phycluster(cpu) == e8895_phycluster(0);
}

/*
 * Every online CPU of @mask is in C2 with its next wake-up at least @res
 * away. The vendor driver used the calling CPU's own sleep length for every
 * CPU; the cpuidle core's per-CPU next_hrtimer is stricter.
 */
static bool e8895_cpus_idle_for(const struct cpumask *mask, ktime_t now, u64 res)
{
	unsigned int c;

	for_each_cpu_and(c, cpu_online_mask, mask) {
		struct cpuidle_device *d = per_cpu(cpuidle_devices, c);
		ktime_t w;

		if (!cpumask_test_cpu(c, &c2_mask) || !d)
			return false;
		w = READ_ONCE(d->next_hrtimer);
		if (!w || ktime_to_ns(ktime_sub(w, now)) < (s64)res)
			return false;
	}
	return true;
}

static void e8895_cluster_cpus(unsigned int cpu, struct cpumask *m)
{
	unsigned int c;

	cpumask_clear(m);
	for_each_possible_cpu(c)
		if (e8895_phycluster(c) == e8895_phycluster(cpu))
			cpumask_set_cpu(c, m);
}

/*
 * s2idle: every other online CPU is inside the state, so this is the last
 * CPU to enter. No residency test: the tick is frozen and next_hrtimer
 * carries no wake time here.
 */
static bool e8895_s2idle_last(unsigned int cpu)
{
	unsigned int c;

	for_each_online_cpu(c) {
		if (c == cpu)
			continue;
		if (!cpumask_test_cpu(c, &c2_mask))
			return false;
	}
	return true;
}

static u32 e8895_c2_prepare(unsigned int cpu, bool s2idle)
{
	struct cpuidle_device *self = per_cpu(cpuidle_devices, cpu);
	u32 state = E8895_PSCI_C2;
	struct cpumask m;
	ktime_t now;

	e8895_pmu_cpu_down(cpu);

	raw_spin_lock(&c2_lock);
	cpumask_set_cpu(cpu, &c2_mask);

	if (s2idle) {
		/* no clock read: timekeeping may already be suspended */
		if (READ_ONCE(sicd_enabled) && READ_ONCE(sicd_s2idle) &&
		    e8895_s2idle_last(cpu) && e8895_boot_cluster(cpu) &&
		    !e8895_sicd_prepare()) {
			sicd_entered = true;
			state = E8895_PSCI_SICD;
		}
		goto out;
	}

	if (!self)
		goto out;
	now = ktime_get();
	if (ktime_to_ns(ktime_sub(READ_ONCE(self->next_hrtimer), now)) <
	    (s64)min(E8895_CPD_RES_NS, E8895_SICD_RES_NS))
		goto out;

	if (e8895_boot_cluster(cpu)) {
		if (READ_ONCE(sicd_enabled) &&
		    e8895_cpus_idle_for(cpu_online_mask, now, E8895_SICD_RES_NS) &&
		    !e8895_sicd_prepare()) {
			sicd_entered = true;
			state = E8895_PSCI_SICD;
		}
	} else if (READ_ONCE(cpd_enabled) && !READ_ONCE(cpd_blocking)) {
		e8895_cluster_cpus(cpu, &m);
		if (e8895_cpus_idle_for(&m, now, E8895_CPD_RES_NS)) {
			e8895_pmu_cluster_down(cpu);
			cluster_down[e8895_phycluster(cpu)] = true;
			state = E8895_PSCI_CPD;
		}
	}
out:
	raw_spin_unlock(&c2_lock);
	return state;
}

static void e8895_c2_finish(unsigned int cpu, bool failed)
{
	/*
	 * A core whose PMU configuration is still off after a wake-up would be
	 * powered down outside PSCI by its next WFI.
	 */
	if (failed || !e8895_pmu_cpu_conf_ok(cpu))
		e8895_pmu_cpu_up(cpu);

	raw_spin_lock(&c2_lock);
	if (cluster_down[e8895_phycluster(cpu)]) {
		e8895_pmu_cluster_up(cpu);
		cluster_down[e8895_phycluster(cpu)] = false;
	}
	if (sicd_entered) {
		e8895_sicd_exit(failed);
		sicd_entered = false;
	}
	cpumask_clear_cpu(cpu, &c2_mask);
	raw_spin_unlock(&c2_lock);
}

static __cpuidle int e8895_enter_wfi(struct cpuidle_device *dev,
				     struct cpuidle_driver *drv, int idx)
{
	cpu_do_idle();
	return idx;
}

static __cpuidle int __e8895_enter_c2(struct cpuidle_device *dev, int idx, bool s2idle)
{
	unsigned int cpu = dev->cpu;
	u32 state;
	int ret;

	if (cpu_pm_enter())
		return -EBUSY;
	state = e8895_c2_prepare(cpu, s2idle);
	ret = e8895_psci_suspend(state);
	e8895_c2_finish(cpu, ret != 0);
	cpu_pm_exit();

	return ret ? -EBUSY : idx;
}

static __cpuidle int e8895_enter_c2(struct cpuidle_device *dev,
				    struct cpuidle_driver *drv, int idx)
{
	return __e8895_enter_c2(dev, idx, false);
}

/*
 * SICD is armed only when every CPU is inside the state with IRQs masked,
 * and the first CPU out runs the exit before IRQs come back, so no code runs
 * while the SICD clock gates are set.
 */
static __cpuidle int e8895_enter_c2_s2idle(struct cpuidle_device *dev,
					   struct cpuidle_driver *drv, int idx)
{
	return __e8895_enter_c2(dev, idx, true);
}

static void e8895_nop(void *info)
{
}

/* no cluster power-down during a Mongoose frequency change */
static int e8895_cpufreq_cb(struct notifier_block *nb, unsigned long val, void *data)
{
	struct cpufreq_freqs *f = data;
	unsigned int cpu = f->policy->cpu;

	if (e8895_boot_cluster(cpu) || !READ_ONCE(cpd_enabled))
		return NOTIFY_OK;
	if (val == CPUFREQ_PRECHANGE) {
		WRITE_ONCE(cpd_blocking, true);
		if (READ_ONCE(cluster_down[e8895_phycluster(cpu)]))
			smp_call_function_single(cpu, e8895_nop, NULL, 0);
	} else if (val == CPUFREQ_POSTCHANGE) {
		WRITE_ONCE(cpd_blocking, false);
	}
	return NOTIFY_OK;
}

static struct notifier_block e8895_cpufreq_nb = {
	.notifier_call = e8895_cpufreq_cb,
};

static int e8895_reboot_cb(struct notifier_block *nb, unsigned long ev, void *cmd)
{
	if (ev == SYS_POWER_OFF || ev == SYS_RESTART)
		cpuidle_pause();
	return NOTIFY_OK;
}

static struct notifier_block e8895_reboot_nb = {
	.notifier_call = e8895_reboot_cb,
};

static const struct of_device_id e8895_idle_match[] __initconst = {
	{ .compatible = "arm,idle-state", .data = e8895_enter_c2 },
	{ },
};

static int __init e8895_idle_init_one(struct cpuidle_driver *drv, unsigned int cluster)
{
	struct cpumask *mask = kzalloc(cpumask_size(), GFP_KERNEL);
	unsigned int c;
	int ret;

	if (!mask)
		return -ENOMEM;
	for_each_possible_cpu(c)
		if (e8895_phycluster(c) == cluster)
			cpumask_set_cpu(c, mask);
	drv->name = "exynos8895_idle";
	drv->owner = THIS_MODULE;
	drv->cpumask = mask;
	drv->states[0].enter = e8895_enter_wfi;
	drv->states[0].exit_latency = 1;
	drv->states[0].target_residency = 1;
	drv->states[0].power_usage = UINT_MAX;
	strscpy(drv->states[0].name, "WFI");
	strscpy(drv->states[0].desc, "ARM WFI");
	ret = dt_init_idle_driver(drv, e8895_idle_match, 1);
	if (ret <= 0) {
		ret = ret ? : -ENODEV;
		goto err;
	}
	drv->states[1].enter_s2idle = e8895_enter_c2_s2idle;
	ret = cpuidle_register(drv, NULL);
	if (ret)
		goto err;
	return 0;
err:
	drv->cpumask = NULL;
	kfree(mask);
	return ret;
}

static int __init e8895_idle_init(void)
{
	int ret;

	if (!of_machine_is_compatible("samsung,exynos8895"))
		return -ENODEV;
	if (!e8895_pmu_cpu_hooks_ready()) {
		pr_err("exynos8895_idle: PMU CPU hooks not ready, staying at WFI\n");
		return -ENODEV;
	}
	ret = e8895_idle_init_one(&e8895_drv[0], e8895_phycluster(0));
	if (!ret) {
		ret = e8895_idle_init_one(&e8895_drv[1], !e8895_phycluster(0));
		if (ret) {
			/* no cluster may idle in C2 without the reboot notifier */
			cpuidle_unregister(&e8895_drv[0]);
			kfree(e8895_drv[0].cpumask);
			e8895_drv[0].cpumask = NULL;
		}
	}
	if (ret) {
		pr_err("exynos8895_idle: init failed %d\n", ret);
		return ret;
	}
	cpuidle_cooling_register(&e8895_drv[0]);
	cpuidle_cooling_register(&e8895_drv[1]);
	cpufreq_register_notifier(&e8895_cpufreq_nb, CPUFREQ_TRANSITION_NOTIFIER);
	register_reboot_notifier(&e8895_reboot_nb);
	return 0;
}
device_initcall(e8895_idle_init);
