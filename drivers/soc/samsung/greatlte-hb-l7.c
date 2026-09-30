// SPDX-License-Identifier: GPL-2.0-only
/*
 * greatlte L7 (deep idle, suspend) heartbeat printers "L7", "L7t" and "L7kg"
 * (spec L7 1.2, 5.8). DEBUG ONLY.
 *
 * L7 (every 30 s): per-CPU C2 counts, CPD/SICD counters and the last SICD veto,
 * the PMU core and cluster power status, WAKEUP_STAT and IDLE_IP3.
 * L7t (every 60 s): arch-counter ticks and RTC seconds across the last
 * suspend (O7-3), PMU CP_STAT.
 * L7kg (CONFIG_GREATLTE_HB_L7_KG, T7-KG boots only): the L1-kept CMU gate
 * registers that differ before/after the last suspend (P17, L1 O-C7).
 * Values are sampled in process context by a PM notifier; the printers only
 * format stored values and read PMU registers (always-on block, T rule 4).
 */

#include <linux/atomic.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/rtc.h>
#include <linux/sizes.h>
#include <linux/soc/samsung/exynos-pmu.h>
#include <linux/soc/samsung/exynos8895-pm.h>
#include <linux/soc/samsung/greatlte-hb.h>
#include <linux/suspend.h>
#include <clocksource/arm_arch_timer.h>

#include "exynos-pmu.h"

static int l7_fmt(char *b, size_t n)
{
	int i, k = 0;
	u32 cpu = 0;

	if (!pmu_base_addr)		/* PMU driver not bound */
		return -ENODEV;
	/*
	 * PMU CLUSTERx_CPUy_STATUS[3:0] != 0 -> bit i, logical order: logical
	 * 0-3 = physical 4-7 (A53), logical 4-7 = physical 0-3 (M2); status
	 * 0x2004 + phycpu * 0x80 (CAL:pmucal_cpu_exynos8895.h:11-80)
	 */
	for (i = 0; i < 8; i++)
		cpu |= ((pmu_raw_readl(0x2004 + ((i < 4 ? 4 + i : i - 4) * 0x80)) & 0xf) ? 1 : 0) << i;
	k += scnprintf(b + k, n - k,
		       "c2=%u,%u,%u,%u,%u,%u,%u,%u cpd=%u sicd=%u sicd_try=%u veto=%u/%s conffix=%u ubt=%u pmu_cpu=%02x",
		       atomic_read(&e8895_stats.c2[0]), atomic_read(&e8895_stats.c2[1]),
		       atomic_read(&e8895_stats.c2[2]), atomic_read(&e8895_stats.c2[3]),
		       atomic_read(&e8895_stats.c2[4]), atomic_read(&e8895_stats.c2[5]),
		       atomic_read(&e8895_stats.c2[6]), atomic_read(&e8895_stats.c2[7]),
		       atomic_read(&e8895_stats.cpd), atomic_read(&e8895_stats.sicd),
		       atomic_read(&e8895_stats.sicd_try), atomic_read(&e8895_stats.sicd_veto),
		       e8895_sicd_last_veto(), atomic_read(&e8895_stats.conf_fix),
		       atomic_read(&e8895_stats.ubt), cpu);
	k += scnprintf(b + k, n - k, " cl0=%x/%x cl1=%x/%x wk=%08x ipi3=%08x",
		       pmu_raw_readl(0x2404) & 0xf, pmu_raw_readl(0x2604) & 0x7,	/* S:drivers/soc/samsung/exynos-pmu.c:84-89 */
		       pmu_raw_readl(0x2424) & 0xf, pmu_raw_readl(0x2624) & 0x7,
		       pmu_raw_readl(0x0600),						/* WAKEUP_STAT */
		       pmu_raw_readl(0x03EC));						/* IDLE_IP3 (APM bit 0, O7-4) */
	return k;
}

static struct greatlte_hb_printer l7_p = {
	.name = "L7", .layer = 7, .every_ms = 30000, .fmt = l7_fmt,
};

static u64 l7_cnt0, l7_cnt_d;
static s64 l7_rtc0 = -1, l7_rtc_d = -1;

#ifdef CONFIG_GREATLTE_HB_L7_KG
#include "greatlte-hb-l7-kg.h"

static u32 l7kg_before[ARRAY_SIZE(l7kg_regs)], l7kg_after[ARRAY_SIZE(l7kg_regs)];
static void __iomem *l7kg_va[ARRAY_SIZE(l7kg_regs)];	/* per entry: its CMU base, mapped once */
static bool l7kg_valid;
#endif

static s64 l7_rtc_now(void)
{
	struct rtc_device *rtc = rtc_class_open("rtc0");
	struct rtc_time tm;
	s64 t = -1;

	if (rtc) {
		if (!rtc_read_time(rtc, &tm))
			t = rtc_tm_to_time64(&tm);
		rtc_class_close(rtc);
	}
	return t;
}

static int l7_pm_notify(struct notifier_block *nb, unsigned long ev, void *unused)
{
#ifdef CONFIG_GREATLTE_HB_L7_KG
	unsigned int i;
#endif

	if (ev == PM_SUSPEND_PREPARE) {
		l7_cnt0 = arch_timer_read_counter();
		l7_rtc0 = l7_rtc_now();
#ifdef CONFIG_GREATLTE_HB_L7_KG
		for (i = 0; i < ARRAY_SIZE(l7kg_regs); i++)
			l7kg_before[i] = l7kg_va[i] ? readl_relaxed(l7kg_va[i] + l7kg_regs[i].off) : 0;
#endif
	} else if (ev == PM_POST_SUSPEND) {
		s64 now = l7_rtc_now();

		WRITE_ONCE(l7_cnt_d, arch_timer_read_counter() - l7_cnt0);
		WRITE_ONCE(l7_rtc_d, (now >= 0 && l7_rtc0 >= 0) ? now - l7_rtc0 : -1);
#ifdef CONFIG_GREATLTE_HB_L7_KG
		for (i = 0; i < ARRAY_SIZE(l7kg_regs); i++)
			l7kg_after[i] = l7kg_va[i] ? readl_relaxed(l7kg_va[i] + l7kg_regs[i].off) : 0;
		WRITE_ONCE(l7kg_valid, true);
#endif
	}
	return NOTIFY_OK;
}

static struct notifier_block l7_pm_nb = {
	.notifier_call = l7_pm_notify,
};

static int l7t_fmt(char *b, size_t n)
{
	s64 rd = READ_ONCE(l7_rtc_d);
	int k;

	if (!pmu_base_addr)
		return -ENODEV;
	k = scnprintf(b, n, "cnt_d=%llu rtc_d=", READ_ONCE(l7_cnt_d));	/* 26 MHz ticks */

	k += rd < 0 ? scnprintf(b + k, n - k, "na") : scnprintf(b + k, n - k, "%lld", rd);
	return k + scnprintf(b + k, n - k, " cp=%08x", pmu_raw_readl(0x0038));	/* CP_STAT */
}

static struct greatlte_hb_printer l7t_p = {
	.name = "L7t", .layer = 7, .every_ms = 60000, .fmt = l7t_fmt,
};

#ifdef CONFIG_GREATLTE_HB_L7_KG
static int l7kg_fmt(char *b, size_t n)
{
	unsigned int i, same = 0;
	int k = 0;

	if (!READ_ONCE(l7kg_valid))
		return scnprintf(b, n, "kg=na");
	for (i = 0; i < ARRAY_SIZE(l7kg_regs); i++) {
		if (l7kg_before[i] == l7kg_after[i]) {
			same++;
			continue;
		}
		k += scnprintf(b + k, n - k, "%s=%08x/%08x ", l7kg_regs[i].name,
			       l7kg_before[i], l7kg_after[i]);
	}
	return k + scnprintf(b + k, n - k, "kg_same=%u", same);
}

static struct greatlte_hb_printer l7kg_p = {
	.name = "L7kg", .layer = 7, .every_ms = 0, .fmt = l7kg_fmt,
};

/* one mapping per distinct CMU base (the six always-on CMUs); SZ_16K covers every offset */
static void __init l7kg_map(void)
{
	unsigned int i, j;

	for (i = 0; i < ARRAY_SIZE(l7kg_regs); i++) {
		for (j = 0; j < i; j++)
			if (l7kg_regs[j].base == l7kg_regs[i].base)
				break;
		l7kg_va[i] = j < i ? l7kg_va[j] : ioremap(l7kg_regs[i].base, SZ_16K);
	}
}
#endif

static int __init l7_hb_init(void)
{
#ifdef CONFIG_GREATLTE_HB_L7_KG
	l7kg_map();
#endif
	register_pm_notifier(&l7_pm_nb);
	greatlte_hb_register(&l7_p);
	greatlte_hb_register(&l7t_p);
#ifdef CONFIG_GREATLTE_HB_L7_KG
	greatlte_hb_register(&l7kg_p);
#endif
	return 0;
}
late_initcall(l7_hb_init);
