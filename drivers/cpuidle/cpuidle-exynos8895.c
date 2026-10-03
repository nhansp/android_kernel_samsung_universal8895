// SPDX-License-Identifier: GPL-2.0-only
/*
 * Exynos 8895 cpuidle: WFI + C2 with the stock CPD (Mongoose cluster power
 * down) and SICD (system idle clock down) upgrades.
 *
 * Port of S:drivers/cpuidle/cpuidle-exynos64.c and the CPU-PM part of
 * S:drivers/soc/samsung/exynos-powermode.c. The firmware is PSCI 0.1 with the
 * Exynos power_state encodings, and the cluster/system power-down is
 * coordinated in the kernel (PMU writes before the SMC), which cpuidle-psci
 * cannot express (spec L7 5.0).
 */

#include <linux/context_tracking.h>
#include <linux/cpu.h>
#include <linux/cpu_cooling.h>
#include <linux/cpu_pm.h>
#include <linux/cpufreq.h>
#include <linux/cpuidle.h>
#include <linux/delay.h>
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
#include <linux/suspend.h>
#include <linux/workqueue.h>
#include <asm/cpuidle.h>
#include <asm/proc-fns.h>

#include "dt_idle_states.h"

#define E8895_CPD_RES_NS	(3000 * NSEC_PER_USEC)	/* FDT:4425 cpd_residency = <0xbb8> */
#define E8895_SICD_RES_NS	(3000 * NSEC_PER_USEC)	/* FDT:4426 sicd_residency = <0xbb8> */

static bool cpd_enabled = true;		/* FDT:4427 cpd_enabled = <0x01> */
static bool sicd_enabled = true;	/* FDT:4428 sicd_enabled = <0x01> */
/*
 * /sys/module/cpuidle_exynos8895/parameters/{cpd,sicd}_enabled replace stock's
 * /sys/power/cpd and /sys/power/sicd (S:drivers/soc/samsung/exynos-powermode.c:611-617);
 * no LOS user. Images with a live UART console run with sicd_enabled=0.
 */
module_param(cpd_enabled, bool, 0644);
module_param(sicd_enabled, bool, 0644);
/*
 * sicd_s2idle: let the ->enter_s2idle state take SICD (l7sicd). Off by default:
 * the combination is new (stock had no s2idle at all) and the bring-up images
 * keep the state that reached them. The normal idle path (stock's own SICD
 * entry) is not affected by this switch.
 */
static bool sicd_s2idle;
module_param(sicd_s2idle, bool, 0644);

/*
 * DEBUG (bring-up of image 0074, which hung ~7 ms after this driver registered):
 * boot-time switches that isolate one variable of the C2 path per boot.
 *   c2_mode=0  stock C2: PMU core-down writes + PSCI CPU_SUSPEND 0x10000 (default)
 *   c2_mode=1  state1 runs a plain WFI: no PMU write, no PSCI; with timer_stop=1
 *              the cpuidle core still stops the arch timer and arms the MCT
 *              local timer as the oneshot wakeup device (tests that path alone)
 *   c2_mode=2  PSCI CPU_SUSPEND 0x10000 without the PMU core-down/up writes
 *   c2_cpus=<mask>  CPUs that run the c2_mode body; the others run a plain WFI
 *   timer_stop=0    drop CPUIDLE_FLAG_TIMER_STOP from state1 (honoured with c2_mode=1 only)
 *   c2_trace=1      print the first state1 entry and return of every CPU
 *   c2_trace=2      also, every 2 s from an unbound worker: per-CPU entries/returns
 *                   of the c2_mode body and the PMU CONFIGURATION/STATUS of each
 *                   core; a CPU that stayed inside for two samples is sent an IPI
 *                   (SGI) once, to tell "not woken" from "woken but stuck"
 *   c2_recycle=1    before registering, take every non-boot CPU offline and online
 *                   once (PSCI CPU_OFF/CPU_ON through the PMU hotplug hooks), so each
 *                   secondary core has been powered up after L1's lpm_init; stock
 *                   ran lpm_init at time_init, before the secondaries were booted
 */
static unsigned int c2_mode;
static unsigned int c2_cpus = 0xff;
static bool timer_stop = true;
static unsigned int c2_trace;
module_param(c2_mode, uint, 0444);
module_param(c2_cpus, uint, 0444);
module_param(timer_stop, bool, 0444);
module_param(c2_trace, uint, 0444);
static bool c2_recycle;
module_param(c2_recycle, bool, 0444);
static unsigned long c2_traced_in, c2_traced_out;	/* bit per CPU */
static atomic_t c2_dbg_in[8], c2_dbg_out[8];

static DEFINE_RAW_SPINLOCK(c2_lock);		/* S:drivers/soc/samsung/exynos-powermode.c:374 */
static struct cpumask c2_mask;			/* :46, update_c2_state() :376-382 */
static int e8895_s2idle_cpu = -1;		/* the CPU that runs s2idle_enter(), ->prepare below */
static bool cluster_down[2];			/* cluster_idle_state, :497-507 */
static bool sicd_entered;			/* :52 */
static bool cpd_blocking;			/* :49 */
static struct cpuidle_driver e8895_drv[2];	/* one per physical cluster */

/* S:drivers/soc/samsung/exynos-powermode.c:418-424 */
static bool e8895_boot_cluster(unsigned int cpu)
{
	return e8895_phycluster(cpu) == e8895_phycluster(0);
}

/*
 * S:drivers/soc/samsung/exynos-powermode.c:389-411 (is_cpus_busy), with the
 * 6.18 per-CPU wake time: the cpuidle core stores each idle CPU's next
 * hrtimer/tick in dev->next_hrtimer before calling ->enter and clears it on
 * exit (A:drivers/cpuidle/cpuidle.c:397,404); the genpd CPU-domain governor
 * makes the same test (A:drivers/pmdomain/governor.c:383-392). Stock compared
 * the calling CPU's own tick_nohz_get_sleep_length() for every CPU
 * (get_next_event_time_us() ignores its argument, :384-387): this is stricter.
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
 * s2idle: is this CPU the last one that is still awake?
 *
 * stock tests is_cpus_busy(sicd_residency, cpu_online_mask) (:389-411): every online
 * CPU must be in c2_mask and have at least sicd_residency before its next event.
 * The second half cannot be made in s2idle - there is no next event to compare, the
 * state is entered by every idle CPU and left only by a wakeup - so this tests the
 * first half only, which in s2idle is exactly the same moment: the CPU that enters the
 * state while every other online CPU is already inside it is the last CPU awake.
 *
 * One online CPU is awake by construction and is not in an idle state: the one that runs
 * the s2idle transition. s2idle_enter() pushes the other CPUs into the idle loop and then
 * parks itself in swait_event_exclusive() (A:kernel/power/suspend.c:113-120), and a
 * sleeping task never reaches the idle loop - only the idle task does, schedule_idle()'s
 * WARN_ON_ONCE(current->__state) (A:kernel/sched/core.c:7968-7980) - so that CPU never
 * sets its c2_mask bit. It is e8895_s2idle_cpu, recorded by e8895_s2idle_prepare(), and it
 * is skipped here: it is the CPU the wakeup returns to, exactly as one CPU is awake in
 * stock's own SICD entry. Tested under c2_lock, so no CPU can leave or enter c2_mask in
 * between: the decision is the same instant stock's is_cpus_busy() takes.
 */
static bool e8895_s2idle_last(unsigned int cpu)
{
	unsigned int c;

	if (e8895_s2idle_cpu < 0)		/* no ->prepare ran: never arm SICD here */
		return false;
	for_each_online_cpu(c) {
		if (c == cpu || c == (unsigned int)e8895_s2idle_cpu)
			continue;
		if (!cpumask_test_cpu(c, &c2_mask))
			return false;
	}
	return true;
}

/* S:drivers/soc/samsung/exynos-powermode.c:513-550 exynos_cpu_pm_enter() */
static u32 e8895_c2_prepare(unsigned int cpu, bool s2idle)
{
	struct cpuidle_device *self = per_cpu(cpuidle_devices, cpu);
	u32 state = E8895_PSCI_C2;
	struct cpumask m;
	ktime_t now;

	e8895_pmu_cpu_down(cpu);				/* :515 */

	raw_spin_lock(&c2_lock);
	cpumask_set_cpu(cpu, &c2_mask);				/* :518 */

	if (s2idle) {
		/*
		 * No clock read here: enter_s2idle_proper() froze the tick before
		 * calling us (A:drivers/cpuidle/cpuidle.c:155,167) and the freeze of
		 * the last online CPU suspends timekeeping
		 * (A:kernel/time/tick-common.c:525-545), which ktime_get() warns
		 * about (A:kernel/time/timekeeping.c:814-821). That same freeze is
		 * also why the residency half of stock's test has no meaning here.
		 */
		if (READ_ONCE(sicd_enabled) && READ_ONCE(sicd_s2idle) &&
		    e8895_s2idle_last(cpu)) {
			atomic_inc(&e8895_stats.sicd_try);
			if (!e8895_boot_cluster(cpu))	/* :473-479, kept as stock */
				atomic_inc(&e8895_stats.sicd_nb);
			else if (!e8895_sicd_prepare()) {	/* :536-537 */
				sicd_entered = true;
				state = E8895_PSCI_SICD;		/* :544 */
				atomic_inc(&e8895_stats.sicd_s2);
			} else {
				atomic_inc(&e8895_stats.sicd_veto);
			}
		}
		goto out;
	}

	if (!self)
		goto out;
	now = ktime_get();
	if (ktime_to_ns(ktime_sub(READ_ONCE(self->next_hrtimer), now)) <	/* :524-526 */
	    (s64)min(E8895_CPD_RES_NS, E8895_SICD_RES_NS))
		goto out;

	/* is_cpd_available(), :426-449 */
	if (READ_ONCE(cpd_enabled) && !READ_ONCE(cpd_blocking) && !e8895_boot_cluster(cpu)) {
		e8895_cluster_cpus(cpu, &m);
		if (e8895_cpus_idle_for(&m, now, E8895_CPD_RES_NS)) {
			e8895_pmu_cluster_down(cpu);		/* :529 */
			cluster_down[e8895_phycluster(cpu)] = true;
			state = E8895_PSCI_CPD;			/* :532 */
		}
	}

	/* is_sicd_available(), :465-488 */
	if (READ_ONCE(sicd_enabled) && e8895_boot_cluster(cpu) &&
	    e8895_cpus_idle_for(cpu_online_mask, now, E8895_SICD_RES_NS)) {
		atomic_inc(&e8895_stats.sicd_try);
		if (!e8895_sicd_prepare()) {			/* :536-537 */
			sicd_entered = true;
			state = E8895_PSCI_SICD;		/* :544 */
		} else {
			atomic_inc(&e8895_stats.sicd_veto);
		}
	}
out:
	raw_spin_unlock(&c2_lock);
	return state;
}

/* S:drivers/soc/samsung/exynos-powermode.c:552-575 exynos_cpu_pm_exit() */
static void e8895_c2_finish(unsigned int cpu, bool failed)
{
	if (failed) {
		e8895_pmu_cpu_up(cpu);				/* :554-555 */
	} else if (!e8895_pmu_cpu_conf_ok(cpu)) {
		/*
		 * Defensive, counted (spec L7 O7-2): stock assumes the wake path
		 * restored CONFIGURATION[3:0] = 0xF (and DD bit 0 on M2). If not,
		 * the next plain WFI would power the core down outside PSCI.
		 */
		e8895_pmu_cpu_up(cpu);
		atomic_inc(&e8895_stats.conf_fix);
	}

	raw_spin_lock(&c2_lock);
	if (cluster_down[e8895_phycluster(cpu)]) {		/* :559-562 */
		e8895_pmu_cluster_up(cpu);
		cluster_down[e8895_phycluster(cpu)] = false;
		atomic_inc(&e8895_stats.cpd);
	}
	if (sicd_entered) {					/* :564-570: the first CPU out runs the exit */
		e8895_sicd_exit(failed);
		sicd_entered = false;
		atomic_inc(&e8895_stats.sicd);
	}
	cpumask_clear_cpu(cpu, &c2_mask);			/* :572 */
	raw_spin_unlock(&c2_lock);
}

/* sicd_entered is written with c2_lock held, so read it the same way */
bool e8895_sicd_armed(void)
{
	bool armed;

	raw_spin_lock(&c2_lock);
	armed = sicd_entered;
	raw_spin_unlock(&c2_lock);
	return armed;
}

static __cpuidle int e8895_enter_wfi(struct cpuidle_device *dev,
				     struct cpuidle_driver *drv, int idx)
{
	cpu_do_idle();		/* S:drivers/cpuidle/cpuidle-exynos64.c:131-134; model A:drivers/cpuidle/cpuidle-psci.c:386-391 */
	return idx;
}

/*
 * State 1 carries CPUIDLE_FLAG_RCU_IDLE (A:drivers/cpuidle/dt_idle_states.c:79):
 * arm64 cpu_suspend() does ct_cpuidle_enter/exit and the pseudo-NMI PMR switch
 * itself (A:arch/arm64/kernel/suspend.c:97-162), as for cpuidle-psci's
 * context-losing path (A:drivers/firmware/psci/psci.c:525-534).
 */
/* DEBUG: plain WFI inside the RCU-idle state1 (model: A:drivers/firmware/psci/psci.c:516-523) */
static __cpuidle void e8895_dbg_wfi(void)
{
	struct arm_cpuidle_irq_context context;

	ct_cpuidle_enter();
	arm_cpuidle_save_irq_context(&context);
	cpu_do_idle();
	arm_cpuidle_restore_irq_context(&context);
	ct_cpuidle_exit();
}

static __cpuidle int __e8895_enter_c2(struct cpuidle_device *dev, int idx, bool s2idle)
{
	unsigned int cpu = dev->cpu;
	unsigned int mode = READ_ONCE(c2_mode);
	u32 state;
	int ret;

	if (!(READ_ONCE(c2_cpus) & BIT(cpu)))
		mode = 1;
	if (mode == 1) {
		e8895_dbg_wfi();
		return idx;
	}
	if (cpu_pm_enter())					/* S:drivers/cpuidle/cpuidle-exynos64.c:105 */
		return -EBUSY;
	if (mode == 2) {
		state = E8895_PSCI_C2;
	} else {
		state = e8895_c2_prepare(cpu, s2idle);
	}
	if (c2_trace && cpu < 8)
		atomic_inc(&c2_dbg_in[cpu]);
	if (c2_trace && !test_and_set_bit(cpu, &c2_traced_in))
		pr_info("exynos8895_idle: cpu%u mode=%u enter 0x%08x conf_ok=%d\n",
			cpu, mode, state, e8895_pmu_cpu_conf_ok(cpu));
	ret = e8895_psci_suspend(state);			/* S:drivers/cpuidle/cpuidle-exynos64.c:136 */
	if (mode != 2)
		e8895_c2_finish(cpu, ret != 0);
	if (c2_trace && cpu < 8)
		atomic_inc(&c2_dbg_out[cpu]);
	if (c2_trace && !test_and_set_bit(cpu, &c2_traced_out))
		pr_info("exynos8895_idle: cpu%u mode=%u return %d (finisher %d) conf_ok=%d status=%x\n",
			cpu, mode, ret, ret ? per_cpu(e8895_dbg_fin_ret, cpu) : 0,
			e8895_pmu_cpu_conf_ok(cpu), e8895_pmu_cpu_status(cpu));
	cpu_pm_exit();
	if (ret)
		return -EBUSY;					/* S:drivers/cpuidle/cpuidle-exynos64.c:156-164 */
	atomic_inc(&e8895_stats.c2[cpu]);
	return idx;
}

static __cpuidle int e8895_enter_c2(struct cpuidle_device *dev,
				    struct cpuidle_driver *drv, int idx)
{
	/* the normal idle path (stock's own CPD/SICD upgrades), not s2idle */
	return __e8895_enter_c2(dev, idx, false);
}

/*
 * s2idle: dt_idle_states gives every DT state ->enter and ->enter_s2idle
 * (A:drivers/cpuidle/dt_idle_states.c:32,38), so states[1] is the one cpuidle_enter_s2idle()
 * finds (A:drivers/cpuidle/cpuidle.c:101,201). CPD stays out of s2idle: it is decided from
 * the tick-based wake times, which are frozen there. SICD is taken when sicd_s2idle is on,
 * see e8895_c2_prepare().
 *
 * Why this is not the PSCI context that hangs (deep suspend, job l7el3):
 *  - the SMC below is the per-CPU CPU_SUSPEND of an ordinary cpuidle state, issued from the
 *    idle loop with the RCU-idle, tick-freeze and cpu_pm bookkeeping of that state
 *    (enter_s2idle_proper(), A:drivers/cpuidle/cpuidle.c:145-179). That is the same context
 *    that already issues power_state 0x00010000 (= E8895_PSCI_C2, FDT:4363
 *    arm,psci-suspend-param) on every idle entry of this device and that woke up on the s2idle
 *    cycles of image 0143, so the SMC path itself is known good here.
 *  - the only additions are the power_state value 0x00000001 - stock's PSCI_SYSTEM_IDLE,
 *    psci_power_state_pack(1,0,0) (S:arch/arm64/kernel/psci.c:212-220,236-239) - and the
 *    PMUCAL SICD enter/exit lists, which stock runs from this same per-CPU path
 *    (exynos_prepare_sys_powerdown(SYS_SICD) inside exynos_cpu_pm_enter(),
 *    S:drivers/soc/samsung/exynos-powermode.c:536-537).
 *  - none of the system-wide PSCI SLEEP steps runs here: for s2idle
 *    e8895_pm_syscore_suspend() returns before its first register write unless
 *    pm_suspend_target_state is PM_SUSPEND_MEM (drivers/soc/samsung/exynos8895-pm.c:1458-1463),
 *    suspend_enter() takes the s2idle_loop() branch (A:kernel/power/suspend.c:454-458) and so
 *    never reaches pm_sleep_disable_secondary_cpus()/arch_suspend_disable_irqs()
 *    (:460-467), no secondary CPU is taken offline, and the SICD enter/exit lists are undone
 *    by the first CPU that leaves the state (e8895_c2_finish()).
 */
static __cpuidle int e8895_enter_c2_s2idle(struct cpuidle_device *dev,
					   struct cpuidle_driver *drv, int idx)
{
	return __e8895_enter_c2(dev, idx, true);
}

/*
 * platform_s2idle_ops::prepare() runs on the CPU that will run s2idle_enter(): the PM
 * core calls it in that task, before dpm_suspend_noirq() and before s2idle_loop()
 * (A:kernel/power/suspend.c:274-278,434-455). This is how the idle side learns which one
 * online CPU stays awake (see e8895_s2idle_last()). Should that task migrate between
 * here and s2idle_enter(), the recorded CPU is an idle one and e8895_s2idle_last()
 * refuses - the safe direction, no SICD.
 *
 * What this combination adds to stock, and why sicd_s2idle therefore defaults to 0: the
 * recorded CPU stays awake in swait_event_exclusive() with its IRQs **enabled**, while in
 * stock's own SICD entry every other CPU is inside C2 with IRQs masked. The SICD enter
 * list blocks the CENTRAL interrupt path of every core (DIS_IRQ_*_CENTRAL = 1,
 * CLKSTOP_CMU_TOP/CMU_MIF, DISABLE_PLL_*, TOP_BUS_MIF/PSCDC_MIF,
 * CAL:pmucal_system_exynos8895.h:203-249), so an interrupt serviced on that CPU inside
 * the armed window is the one hazard no test here can rule out.
 */
static int e8895_s2idle_prepare(void)
{
	WRITE_ONCE(e8895_s2idle_cpu, (int)smp_processor_id());
	return 0;
}

/*
 * The SICD exit runs on the first CPU that leaves the idle state (stock: the first
 * CPU that calls exynos_cpu_pm_exit(), :564-570). The CPU that ran s2idle_enter() is
 * not one of them and does not wait for it - s2idle_enter() only kicks the idle CPUs
 * (A:kernel/power/suspend.c:117-124) and returns - so without a barrier it would
 * resume devices while the PMU is still in the SICD state. SLEEP needs the same
 * barrier and gets it for free: syscore_resume() runs the PMUCAL exit list on the CPU
 * that issued the SLEEP SMC, before dpm_resume_noirq() (A:kernel/power/suspend.c:471-498).
 * platform_s2idle_ops::restore_early() is the one hook that runs in that gap for s2idle
 * (platform_resume_noirq(), A:kernel/power/suspend.c:289-297,498). Only ->prepare and
 * ->restore_early are filled in: the other six hooks stay NULL, so platform_suspend_begin()
 * still falls through to suspend_ops->begin() (A:kernel/power/suspend.c:310-317) and our
 * platform_suspend_ops has no .begin (drivers/soc/samsung/exynos8895-pm.c:1646).
 */
static void e8895_s2idle_restore_early(void)
{
	unsigned int t;

	if (!READ_ONCE(sicd_s2idle))
		return;
	/* 1000 x udelay(1), the bound stock's PMUCAL WAIT gives up after
	 * (S:drivers/soc/samsung/cal-if/pmucal_rae.c:84-103) */
	for (t = 0; t < 1000 && e8895_sicd_armed(); t++)
		udelay(1);
	if (e8895_sicd_armed()) {
		atomic_inc(&e8895_stats.sicd_wait);
		pr_err("exynos8895_idle: SICD still armed 1000 us after the s2idle CPUs were kicked\n");
	}
}

static struct platform_s2idle_ops e8895_s2idle_ops = {
	.prepare = e8895_s2idle_prepare,
	.restore_early = e8895_s2idle_restore_early,
};

static void e8895_nop(void *info)
{
}

/* DEBUG (c2_trace=2): periodic view of the C2 body from a CPU that is awake */
static void e8895_c2_watch_fn(struct work_struct *w);
static DECLARE_DELAYED_WORK(e8895_c2_watch, e8895_c2_watch_fn);
static unsigned int c2_dbg_stuck[8];
static unsigned long c2_dbg_poked;

static void e8895_c2_watch_fn(struct work_struct *w)
{
	char b[256];
	unsigned int c;
	int k = 0;

	for_each_possible_cpu(c) {
		int in, out;

		if (c >= 8)
			break;
		in = atomic_read(&c2_dbg_in[c]);
		out = atomic_read(&c2_dbg_out[c]);
		k += scnprintf(b + k, sizeof(b) - k, " %u:%d/%d/c%d/s%x", c, in, out,
			       e8895_pmu_cpu_conf_ok(c), e8895_pmu_cpu_status(c));
		c2_dbg_stuck[c] = in > out ? c2_dbg_stuck[c] + 1 : 0;
		if (c2_dbg_stuck[c] == 2 && !test_and_set_bit(c, &c2_dbg_poked)) {
			k += scnprintf(b + k, sizeof(b) - k, "(poke)");
			smp_call_function_single(c, e8895_nop, NULL, 0);
		}
	}
	pr_info("exynos8895_idle: watch in/out/conf_ok/status:%s\n", b);
	queue_delayed_work(system_unbound_wq, &e8895_c2_watch, 2 * HZ);
}

/* S:drivers/soc/samsung/exynos-powermode.c:737-767: no cluster power-down during a DVFS change */
static int e8895_cpufreq_cb(struct notifier_block *nb, unsigned long val, void *data)
{
	struct cpufreq_freqs *f = data;
	unsigned int cpu = f->policy->cpu;		/* stock freq->cpu; A:include/linux/cpufreq.h:191-192 */

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

/* S:drivers/cpuidle/cpuidle-exynos64.c:170-185 */
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
	drv->states[0].enter = e8895_enter_wfi;		/* A:drivers/cpuidle/cpuidle-psci.c:386-391 */
	drv->states[0].exit_latency = 1;		/* S:drivers/cpuidle/cpuidle-exynos64.c:193-195 */
	drv->states[0].target_residency = 1;
	drv->states[0].power_usage = UINT_MAX;
	strscpy(drv->states[0].name, "WFI");
	strscpy(drv->states[0].desc, "ARM WFI");
	ret = dt_init_idle_driver(drv, e8895_idle_match, 1);	/* A:drivers/cpuidle/dt_idle_states.c:144 */
	if (ret <= 0) {
		ret = ret ? : -ENODEV;
		goto err;
	}
	drv->states[1].enter_s2idle = e8895_enter_c2_s2idle;
	if (!timer_stop && c2_mode == 1)			/* DEBUG */
		drv->states[1].flags &= ~CPUIDLE_FLAG_TIMER_STOP;
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
	/* C2 needs the PMU core hooks and their droop-detector mapping */
	if (!e8895_pmu_cpu_hooks_ready()) {
		pr_err("exynos8895_idle: PMU CPU hooks not ready, staying at WFI\n");
		return -ENODEV;
	}
	if (c2_recycle) {						/* DEBUG */
		unsigned int c;

		for_each_online_cpu(c) {
			int r1, r2;

			if (!c)
				continue;
			r1 = remove_cpu(c);
			r2 = r1 ? 0 : add_cpu(c);
			pr_info("exynos8895_idle: DEBUG recycle cpu%u off %d on %d status %x\n",
				c, r1, r2, e8895_pmu_cpu_status(c));
		}
	}
	/* boot cluster = A53 = PMU cluster 1 first */
	ret = e8895_idle_init_one(&e8895_drv[0], e8895_phycluster(0));
	if (!ret) {
		ret = e8895_idle_init_one(&e8895_drv[1], !e8895_phycluster(0));
		if (ret) {
			/* never leave one cluster in C2 without the reboot notifier below */
			cpuidle_unregister(&e8895_drv[0]);
			kfree(e8895_drv[0].cpumask);
			e8895_drv[0].cpumask = NULL;
		}
	}
	if (ret) {
		pr_err("exynos8895_idle: init failed %d\n", ret);
		return ret;
	}
	cpuidle_cooling_register(&e8895_drv[0]);	/* A:drivers/cpuidle/cpuidle-psci.c:417 */
	cpuidle_cooling_register(&e8895_drv[1]);
	cpufreq_register_notifier(&e8895_cpufreq_nb, CPUFREQ_TRANSITION_NOTIFIER);
	register_reboot_notifier(&e8895_reboot_nb);
	/* the SICD exit barrier for s2idle; inert while sicd_s2idle is off */
	s2idle_set_ops(&e8895_s2idle_ops);		/* A:include/linux/suspend.h:317 */
	pr_info("exynos8895_idle: C2 on 8 CPUs, cpd=%d sicd=%d sicd_s2idle=%d\n",
		cpd_enabled, sicd_enabled, sicd_s2idle);
	if (c2_trace >= 2)						/* DEBUG */
		queue_delayed_work(system_unbound_wq, &e8895_c2_watch, 2 * HZ);
	if (c2_mode || c2_cpus != 0xff || !timer_stop || c2_trace || c2_recycle)	/* DEBUG */
		pr_info("exynos8895_idle: DEBUG c2_mode=%u c2_cpus=0x%02x timer_stop=%d c2_trace=%u c2_recycle=%d\n",
			c2_mode, c2_cpus, !(c2_mode == 1 && !timer_stop), c2_trace, c2_recycle);
	return 0;
}
device_initcall(e8895_idle_init);			/* S:drivers/cpuidle/cpuidle-exynos64.c:293 */
