// SPDX-License-Identifier: GPL-2.0-only
/*
 * Exynos8895 system power management: system suspend to the SLEEP state and
 * its VTS_ON/AUD_ON variants, the SICD entry, exit and veto used by the
 * cpuidle driver, the suspend voltages of the BUCK8-11 rails (PM-DVS) and the
 * UART_BT pad retention release.
 *
 * The PMU sequences are the vendor PMUCAL system lists, kept as tables in
 * exynos8895-pm-tables.h and run by a port of the PMUCAL table engine.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/psci.h>
#include <linux/pwm.h>
#include <linux/regulator/consumer.h>
#include <linux/rtmutex.h>
#include <linux/slab.h>
#include <linux/soc/samsung/exynos-pmu.h>
#include <linux/soc/samsung/exynos-regs-pmu.h>
#include <linux/soc/samsung/exynos8895-pm.h>
#include <linux/suspend.h>
#include <linux/syscore_ops.h>
#include <asm/memory.h>
#include <asm/suspend.h>
#include <dt-bindings/soc/samsung,exynos8895-pm.h>

#include "exynos-pmu.h"

#define E8895_WAKEUP_STAT	0x0600
#define E8895_EINT_WAKEUP_MASK	0x060c
#define E8895_EXTRA_WAKEUP_STAT	0x0640
#define E8895_CP_STAT		0x0038
#define E8895_IDLE_IP(x)	(0x03e0 + (x) * 4)
#define E8895_IDLE_IP_MASK(x)	(0x03f0 + (x) * 4)
/* IDLE_IP index the APM firmware clears while its DVFS work is pending */
#define E8895_FIX_IDLE_IP	0x60
#define E8895_PAD_RET_UART_BT	0x3188

/* PMUCAL system power-down modes */
enum {
	E8895_SYS_SICD = 0,
	E8895_SYS_SLEEP = 8,
	E8895_SYS_SLEEP_VTS_ON = 9,
	E8895_SYS_SLEEP_AUD_ON = 10,
};

/* WAKEUP_MASK, WAKEUP_MASK2, WAKEUP_MASK3 and the extra wakeup mask */
static const u32 e8895_wkmask_off[4] = { 0x610, 0x614, 0x618, 0x644 };
static const u32 e8895_wkmask_sicd[4] = { 0x400001e0, 0x0, 0x0, 0x4 };
static const u32 e8895_wkmask_sleep[4] = { 0x500f7e7e, 0xffff00ff, 0xffff00ff, 0x4 };

/* Power domain STATUS registers; sicd: the domain must be off for SICD */
static const struct {
	const char *name;
	u16 off;
	bool sicd;
} e8895_pd_stat[] = {
	{ "abox",  0x4004, false },
	{ "cam",   0x4024, true  },
	{ "dbg",   0x4044, false },
	{ "dcam",  0x5224, true  },
	{ "dpu0",  0x4064, true  },
	{ "dpu1",  0x4084, true  },
	{ "dsp",   0x40a4, true  },
	{ "g2d",   0x40c4, true  },
	{ "g3d",   0x40e4, true  },
	{ "isphq", 0x4104, true  },
	{ "isplp", 0x4124, true  },
	{ "iva",   0x4144, true  },
	{ "mfc",   0x4164, true  },
	{ "srdz",  0x5424, true  },
	{ "vpu",   0x4184, true  },
	{ "vts",   0x41a4, false },
};

#define E8895_PD_ON(off)	((pmu_raw_readl(off) & 0xf) == 0xf)
#define E8895_PD_ABOX		0x4004
#define E8895_PD_VTS		0x41a4

enum e8895_seq_type {
	E8895_WRITE,
	E8895_COND_WRITE,
	E8895_SAVE_RESTORE,
	E8895_COND_SAVE_RESTORE,
	E8895_COND_CHECK_SKIP,
	E8895_WAIT,
	E8895_READ,
};

#define E8895_NO_COND	0xff

struct e8895_seq {
	u8 type, base;
	u16 off;
	u32 mask, val;
	u8 cbase;			/* condition block, E8895_NO_COND if none */
	u16 coff;
	u32 cmask, cval;
	u32 saved;
	bool need_restore, need_skip;
};

#include "exynos8895-pm-tables.h"

static void __iomem *e8895_va[ARRAY_SIZE(e8895_pm_bases)];

static bool e8895_cond(const struct e8895_seq *s)
{
	return (readl_relaxed(e8895_va[s->cbase] + s->coff) & s->cmask) == s->cval;
}

static bool e8895_val_ok(const struct e8895_seq *s)
{
	return (readl_relaxed(e8895_va[s->base] + s->off) & s->mask) == s->val;
}

static void e8895_write(const struct e8895_seq *s, u32 v)
{
	void __iomem *a = e8895_va[s->base] + s->off;

	writel_relaxed(s->mask == U32_MAX ? v : (readl_relaxed(a) & ~s->mask) | v, a);
}

static int e8895_wait(const struct e8895_seq *s)
{
	unsigned int t = 0;

	while (!e8895_val_ok(s)) {
		t++;
		udelay(1);
		if (t > 1000)
			return -ETIMEDOUT;
	}
	return 0;
}

/* enter, exit and early wakeup lists: only WRITE, COND_WRITE and WAIT entries */
static int e8895_handle(struct e8895_seq *s, unsigned int n)
{
	unsigned int i;

	for (i = 0; i < n; i++, s++) {
		switch (s->type) {
		case E8895_COND_WRITE:
			if (!e8895_cond(s))
				break;
			fallthrough;
		case E8895_WRITE:
			e8895_write(s, s->val);
			break;
		case E8895_WAIT:
			if (e8895_wait(s))
				return -ETIMEDOUT;
			break;
		default:
			break;
		}
	}
	return 0;
}

static void e8895_save(struct e8895_seq *s, unsigned int n)
{
	for (; n--; s++) {
		if (s->type == E8895_COND_SAVE_RESTORE && !e8895_cond(s))
			continue;
		if (s->type == E8895_SAVE_RESTORE || s->type == E8895_COND_SAVE_RESTORE) {
			s->saved = readl_relaxed(e8895_va[s->base] + s->off) & s->mask;
			s->need_restore = true;
		}
	}
}

static int e8895_restore(struct e8895_seq *s, unsigned int n)
{
	unsigned int i;

	for (i = 0; i < n; i++) {
		struct e8895_seq *e = &s[i];

		if (e->need_skip) {
			e->need_skip = false;
			continue;
		}
		switch (e->type) {
		case E8895_COND_SAVE_RESTORE:
			if (!e8895_cond(e))
				break;
			fallthrough;
		case E8895_SAVE_RESTORE:
			if (e->need_restore) {
				e->need_restore = false;
				e8895_write(e, e->saved);
			}
			break;
		case E8895_COND_CHECK_SKIP:
			/* skip the PLL lock wait when the domain is off or already locked */
			if (i + 1 < n)
				s[i + 1].need_skip = !e8895_cond(e) || e8895_val_ok(e);
			break;
		case E8895_WAIT:
			if (e8895_wait(e))
				return -ETIMEDOUT;
			break;
		case E8895_WRITE:
			e8895_write(e, e->val);
			break;
		default:
			break;
		}
	}
	return 0;
}

/* One set of lists per SLEEP flavour: 0 enter, 1 save, 2 exit, 3 early wakeup */
struct e8895_tabs {
	struct e8895_seq *t[4];
	unsigned int n[4];
};

#define E8895_TABS(x) {								\
	{ e8895_enter_##x, e8895_save_##x, e8895_exit_##x, e8895_earlywkup_##x },	\
	{ ARRAY_SIZE(e8895_enter_##x), ARRAY_SIZE(e8895_save_##x),		\
	  ARRAY_SIZE(e8895_exit_##x), ARRAY_SIZE(e8895_earlywkup_##x) } }

static struct e8895_tabs e8895_sleep_tabs[3] = {
	E8895_TABS(sleep), E8895_TABS(sleep_vts_on), E8895_TABS(sleep_aud_on)
};

static struct e8895_tabs *e8895_tab(int mode)
{
	return &e8895_sleep_tabs[mode - E8895_SYS_SLEEP];
}

struct e8895_veto {
	u8 kind;			/* E8895_SICD_VETO_* */
	struct clk *clk;
	struct device *dev;
	struct i2c_adapter *adap;
};

static struct e8895_veto *e8895_vetoes;
static unsigned int e8895_nveto;
static bool e8895_veto_ready;

/*
 * Lock-free reads of framework state, called with IRQs off from the idle
 * path. They stand in for the IDLE_IP votes of the vendor drivers.
 */
static bool e8895_veto_busy(const struct e8895_veto *v)
{
	switch (v->kind) {
	case E8895_SICD_VETO_CLK:
		return __clk_is_enabled(v->clk);
	case E8895_SICD_VETO_RPM:
		return !pm_runtime_enabled(v->dev) || !pm_runtime_status_suspended(v->dev);
	case E8895_SICD_VETO_I2C:
		return rt_mutex_base_is_locked(&v->adap->bus_lock.rtmutex);
	case E8895_SICD_VETO_PWM: {
		/*
		 * Busy while a channel the PWM framework drives is enabled. The
		 * timer control register is no proxy: the boot firmware leaves
		 * timer 4 free-running with no Linux user.
		 */
		struct pwm_chip *chip;
		unsigned int i;

		if (!READ_ONCE(v->dev->driver))
			return false;
		chip = dev_get_drvdata(v->dev);
		if (!chip)
			return false;
		for (i = 0; i < chip->npwm; i++)
			if (READ_ONCE(chip->pwms[i].state.enabled))
				return true;
		return false;
	}
	}
	return true;
}

static bool e8895_sicd_vetoed(void)
{
	unsigned int i;

	/* pairs with smp_store_release() in e8895_pm_resolve_vetoes_locked() */
	if (!smp_load_acquire(&e8895_veto_ready))
		return true;
	for (i = 0; i < ARRAY_SIZE(e8895_pd_stat); i++)
		if (e8895_pd_stat[i].sicd && E8895_PD_ON(e8895_pd_stat[i].off))
			return true;
	if (!(pmu_raw_readl(E8895_IDLE_IP(E8895_FIX_IDLE_IP / 32)) & BIT(E8895_FIX_IDLE_IP % 32)))
		return true;
	for (i = 0; i < e8895_nveto; i++)
		if (e8895_veto_busy(&e8895_vetoes[i]))
			return true;
	return false;
}

/* Called by the cpuidle driver with its state lock held and IRQs off */
int e8895_sicd_prepare(void)
{
	unsigned int i;

	if (e8895_sicd_vetoed())
		return -EBUSY;
	/*
	 * Only the APM index is left unmasked: no driver votes in the software
	 * IDLE_IP words, the veto above makes that decision instead.
	 */
	for (i = 0; i < 4; i++)
		pmu_raw_writel(i == E8895_FIX_IDLE_IP / 32 ?
			       (u32)~BIT(E8895_FIX_IDLE_IP % 32) : ~0u, E8895_IDLE_IP_MASK(i));
	pmu_raw_writel(exynos_pinctrl_get_eint_wake_mask(), E8895_EINT_WAKEUP_MASK);
	for (i = 0; i < 4; i++)
		pmu_raw_writel(e8895_wkmask_sicd[i], e8895_wkmask_off[i]);
	return e8895_handle(e8895_enter_sicd, ARRAY_SIZE(e8895_enter_sicd));
}

void e8895_sicd_exit(bool early)
{
	if (early)
		e8895_handle(e8895_earlywkup_sicd, ARRAY_SIZE(e8895_earlywkup_sicd));
	else
		e8895_handle(e8895_exit_sicd, ARRAY_SIZE(e8895_exit_sicd));
}

static int e8895_mode;
static bool e8895_early;
static bool e8895_sleep_armed;

/*
 * The vendor kernel passes cpu_resume for every Exynos state, including the
 * ones whose power_state has the StateType bit clear.
 */
static noinstr int e8895_psci_finisher(unsigned long state)
{
	return psci_ops.cpu_suspend(state, __pa_symbol_nodebug((unsigned long)cpu_resume));
}

int e8895_psci_suspend(u32 state)
{
	return cpu_suspend(state, e8895_psci_finisher);
}

/*
 * A domain whose power-off is still pending (CONFIGURATION != STATUS) stalls
 * the SLEEP power-down with no wakeup source working. The vendor kernel
 * entered SLEEP regardless; refuse it instead.
 */
static int e8895_pd_check(void)
{
	unsigned int i;
	int ret = 0;

	for (i = 0; i < ARRAY_SIZE(e8895_pd_stat); i++) {
		u16 st = e8895_pd_stat[i].off;
		u32 c = pmu_raw_readl(st - 4) & 0xf, v = pmu_raw_readl(st) & 0xf;

		if (c != v) {
			pr_err("exynos8895-pm: pd %s CONFIGURATION 0x%x STATUS 0x%x: no SLEEP\n",
			       e8895_pd_stat[i].name, c, v);
			ret = -EBUSY;
		}
	}
	return ret;
}

/* The idle and wakeup masks before arming, written back on a refusal */
static u32 e8895_save_ipm[4], e8895_save_wk[4], e8895_save_eint;

static void e8895_restore_masks(void)
{
	unsigned int i;

	for (i = 0; i < 4; i++) {
		pmu_raw_writel(e8895_save_ipm[i], E8895_IDLE_IP_MASK(i));
		pmu_raw_writel(e8895_save_wk[i], e8895_wkmask_off[i]);
	}
	pmu_raw_writel(e8895_save_eint, E8895_EINT_WAKEUP_MASK);
}

static int e8895_sleep_arm(const char **why)
{
	struct e8895_tabs *t;
	unsigned int i;

	for (i = 0; i < 4; i++)
		pmu_raw_writel(~0u, E8895_IDLE_IP_MASK(i));
	pmu_raw_writel(exynos_pinctrl_get_eint_wake_mask(), E8895_EINT_WAKEUP_MASK);
	for (i = 0; i < 4; i++)
		pmu_raw_writel(e8895_wkmask_sleep[i], e8895_wkmask_off[i]);
	t = e8895_tab(e8895_mode);
	e8895_save(t->t[1], t->n[1]);
	if (e8895_handle(t->t[0], t->n[0])) {
		/* the vendor kernel ignores this timeout; roll back and abort instead */
		e8895_handle(t->t[3], t->n[3]);
		e8895_restore(t->t[1], t->n[1]);
		e8895_restore_masks();
		*why = "enter list wait timed out";
		return -EIO;
	}
	/*
	 * Early wakeup until ->enter reports a real power-down: suspend_enter()
	 * still runs syscore_resume() when it skips ->enter.
	 */
	e8895_early = true;
	e8895_sleep_armed = true;
	pr_debug("exynos8895-pm: SLEEP mode=%d\n", e8895_mode);
	return 0;
}

/*
 * A failing syscore suspend op gets no resume call, so every refusal leaves
 * the PMU as it found it.
 */
static int e8895_pm_syscore_suspend(void)
{
	const char *why;
	unsigned int i;
	int ret;

	e8895_sleep_armed = false;
	/* hibernation runs the syscore ops too */
	if (pm_suspend_target_state != PM_SUSPEND_MEM)
		return 0;

	if (!pmu_raw_readl(E8895_CP_STAT)) {
		why = "CP reset";
		ret = -EINVAL;
		goto refuse;
	}
	if (e8895_pd_check()) {
		why = "power domain half-way";
		ret = -EBUSY;
		goto refuse;
	}
	if (E8895_PD_ON(E8895_PD_ABOX))
		e8895_mode = E8895_SYS_SLEEP_AUD_ON;
	else if (E8895_PD_ON(E8895_PD_VTS))
		e8895_mode = E8895_SYS_SLEEP_VTS_ON;
	else
		e8895_mode = E8895_SYS_SLEEP;

	for (i = 0; i < 4; i++) {
		e8895_save_ipm[i] = pmu_raw_readl(E8895_IDLE_IP_MASK(i));
		e8895_save_wk[i] = pmu_raw_readl(e8895_wkmask_off[i]);
	}
	e8895_save_eint = pmu_raw_readl(E8895_EINT_WAKEUP_MASK);

	ret = e8895_sleep_arm(&why);
	if (ret)
		goto refuse;
	return 0;

refuse:
	pr_err("exynos8895-pm: SLEEP refused: %s (%d)\n", why, ret);
	return ret;
}

static void e8895_pm_syscore_resume(void)
{
	struct e8895_tabs *t;
	u32 stat;

	if (!e8895_sleep_armed)
		return;
	e8895_sleep_armed = false;
	t = e8895_tab(e8895_mode);
	e8895_handle(t->t[e8895_early ? 3 : 2], t->n[e8895_early ? 3 : 2]);
	if (e8895_restore(t->t[1], t->n[1]))
		pr_err("exynos8895-pm: restore: PLL/OSC lock timeout\n");
	stat = pmu_raw_readl(E8895_WAKEUP_STAT);
	pr_info("exynos8895-pm: %sWAKEUP_STAT=0x%08x 0x640=0x%08x eint_mask=0x%08x%s%s%s\n",
		e8895_early ? "early " : "", stat, pmu_raw_readl(E8895_EXTRA_WAKEUP_STAT),
		pmu_raw_readl(E8895_EINT_WAKEUP_MASK),
		stat & BIT(0) ? " (EINT)" : "", stat & BIT(1) ? " (RTC alarm)" : "",
		stat & BIT(24) ? " (MBOX)" : "");
}

static struct syscore_ops e8895_pm_syscore_ops = {
	.suspend = e8895_pm_syscore_suspend,
	.resume  = e8895_pm_syscore_resume,
};

static int e8895_pm_enter(suspend_state_t state)
{
	int ret;

	ret = e8895_psci_suspend(E8895_PSCI_SLEEP);
	e8895_early = ret != 0;
	if (e8895_early)
		pr_info("exynos8895-pm: return to originator\n");
	return e8895_early ? -EBUSY : 0;
}

static const struct platform_suspend_ops e8895_pm_ops = {
	.valid = suspend_valid_only_mem,
	.enter = e8895_pm_enter,
};

/*
 * Releases the UART_BT pad retention after every device resumed, as the
 * vendor serial driver did after resuming the Bluetooth port.
 */
static void e8895_pm_complete(struct device *dev)
{
	/* still set here: it is reset only after dpm_resume_end() */
	if (pm_suspend_target_state != PM_SUSPEND_MEM)
		return;
	pmu_raw_writel(pmu_raw_readl(E8895_PAD_RET_UART_BT) | EXYNOS_WAKEUP_FROM_LOWPWR,
		       E8895_PAD_RET_UART_BT);
}

static const struct {
	const char *supply;
	u32 suspend_uv, init_uv;
} e8895_dvs[] = {
	{ "vdd-mldo",  1887500, 1950000 },
	{ "vdd-lldo1",  900000,  950000 },
	{ "vdd-lldo2", 1050000, 1100000 },
	{ "vdd-lldo3", 1250000, 1300000 },
};

#define E8895_DVS_RANGE		25000

static struct regulator *e8895_dvs_reg[ARRAY_SIZE(e8895_dvs)];
static bool e8895_dvs_lowered;

static int e8895_dvs_set(unsigned int i, bool suspend)
{
	u32 uv = suspend ? e8895_dvs[i].suspend_uv : e8895_dvs[i].init_uv;
	int ret = regulator_set_voltage(e8895_dvs_reg[i], uv, uv + E8895_DVS_RANGE);

	if (ret)
		pr_err("exynos8895-pm: dvs %s -> %u: %d\n", e8895_dvs[i].supply, uv, ret);
	return ret;
}

static int e8895_dvs_suspend_late(struct device *dev)
{
	unsigned int i;
	int ret;

	if (pm_suspend_target_state != PM_SUSPEND_MEM)
		return 0;
	for (i = 0; i < ARRAY_SIZE(e8895_dvs); i++) {
		if (!e8895_dvs_reg[i])
			continue;
		ret = e8895_dvs_set(i, true);
		if (ret) {
			/* no resume_early follows a failed suspend_late: undo here */
			while (i--)
				if (e8895_dvs_reg[i])
					e8895_dvs_set(i, false);
			return ret;
		}
	}
	e8895_dvs_lowered = true;
	return 0;
}

static int e8895_dvs_resume_early(struct device *dev)
{
	unsigned int i;
	int ret, err = 0;

	if (!e8895_dvs_lowered)
		return 0;
	e8895_dvs_lowered = false;
	/* unlike the vendor driver, a failure does not stop the other rails */
	for (i = 0; i < ARRAY_SIZE(e8895_dvs); i++) {
		if (!e8895_dvs_reg[i])
			continue;
		ret = e8895_dvs_set(i, false);
		err = err ? : ret;
	}
	return err;
}

/* system suspend only: no PM-DVS around hibernation */
static const struct dev_pm_ops e8895_pm_dev_ops = {
	.suspend_late = e8895_dvs_suspend_late,
	.resume_early = e8895_dvs_resume_early,
	.complete     = e8895_pm_complete,
};

static DEFINE_MUTEX(e8895_veto_lock);
static bool e8895_veto_late;		/* late_initcall_sync has run */
static bool e8895_pm_probed;		/* e8895_va[] mapped for good */
static int e8895_veto_last_bad = -1;
static void e8895_pm_try_resolve(void);

static int e8895_pm_bus_notify(struct notifier_block *nb, unsigned long action, void *data)
{
	/* pairs with smp_store_release() in e8895_pm_resolve_vetoes_locked() */
	if (action == BUS_NOTIFY_BOUND_DRIVER && !smp_load_acquire(&e8895_veto_ready))
		e8895_pm_try_resolve();
	return NOTIFY_DONE;
}

static struct notifier_block e8895_pm_bus_nb = {
	.notifier_call = e8895_pm_bus_notify,
};

static int e8895_pm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(e8895_pm_bases); i++) {
		/* not requested: the CMU and SYSREG drivers own these blocks */
		e8895_va[i] = devm_ioremap(dev, e8895_pm_bases[i], e8895_pm_sizes[i]);
		if (!e8895_va[i])
			return -ENOMEM;
	}
	for (i = 0; i < ARRAY_SIZE(e8895_dvs); i++) {
		struct regulator *r = devm_regulator_get_optional(dev, e8895_dvs[i].supply);

		if (IS_ERR(r)) {
			if (PTR_ERR(r) == -EPROBE_DEFER)
				return -EPROBE_DEFER;
			dev_warn(dev, "no %s supply: PM-DVS skips it\n", e8895_dvs[i].supply);
			r = NULL;
		}
		e8895_dvs_reg[i] = r;
	}
	/* SLEEP offlines CPUs 1-7 through the PMU CPU hooks */
	if (!e8895_pmu_cpu_hooks_ready()) {
		dev_err(dev, "PMU CPU hooks not ready: no SLEEP, mem falls back to s2idle\n");
		return -ENODEV;
	}
	WRITE_ONCE(e8895_pm_probed, true);
	register_syscore_ops(&e8895_pm_syscore_ops);
	suspend_set_ops(&e8895_pm_ops);
	bus_register_notifier(&platform_bus_type, &e8895_pm_bus_nb);
	e8895_pm_try_resolve();
	return 0;
}

static const struct of_device_id e8895_pm_of_match[] = {
	{ .compatible = "samsung,exynos8895-pm" },
	{ }
};

static struct platform_driver e8895_pm_driver = {
	.probe = e8895_pm_probe,
	.driver = {
		.name = "exynos8895-pm",
		.of_match_table = e8895_pm_of_match,
		.pm = pm_sleep_ptr(&e8895_pm_dev_ops),
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(e8895_pm_driver);

static void e8895_veto_put(struct e8895_veto *v)
{
	if (v->clk)
		clk_put(v->clk);
	if (v->dev)
		put_device(v->dev);
	if (v->adap)
		put_device(&v->adap->dev);
}

/* Publishes the samsung,sicd-veto list only once every entry resolved */
static void e8895_pm_resolve_vetoes_locked(void)
{
	struct device_node *pm = of_find_compatible_node(NULL, NULL, "samsung,exynos8895-pm");
	struct e8895_veto *arr;
	struct of_phandle_args a;
	int n, i, k = 0, bad = 0;

	if (!pm)
		return;
	n = of_property_count_u32_elems(pm, "samsung,sicd-veto") / 2;
	if (n <= 0)
		goto out_node;
	arr = kcalloc(n, sizeof(*arr), GFP_KERNEL);
	if (!arr)
		goto out_node;
	for (i = 0; i < n; i++) {
		struct e8895_veto *v = &arr[k];
		struct platform_device *pdev;

		if (of_parse_phandle_with_fixed_args(pm, "samsung,sicd-veto", 1, i, &a)) {
			bad++;
			continue;
		}
		if (!of_device_is_available(a.np))
			goto next;
		v->kind = a.args[0];
		switch (v->kind) {
		case E8895_SICD_VETO_CLK: {
			struct of_phandle_args ca;

			v->clk = of_clk_get(a.np, 0);
			if (IS_ERR(v->clk)) {
				v->clk = NULL;
				bad++;
				break;
			}
			/*
			 * __clk_is_enabled() of a gate whose CMU uses runtime PM
			 * would call into runtime PM from the idle path.
			 */
			if (!of_parse_phandle_with_args(a.np, "clocks", "#clock-cells", 0, &ca)) {
				struct platform_device *cp = of_find_device_by_node(ca.np);
				bool rpm = cp && pm_runtime_enabled(&cp->dev);

				if (cp)
					put_device(&cp->dev);
				of_node_put(ca.np);
				if (rpm) {
					e8895_veto_put(v);
					memset(v, 0, sizeof(*v));
					bad++;
					break;
				}
			}
			k++;
			break;
		}
		case E8895_SICD_VETO_RPM:
		case E8895_SICD_VETO_PWM:
			pdev = of_find_device_by_node(a.np);
			if (!pdev) {
				bad++;
				break;
			}
			v->dev = &pdev->dev;
			k++;
			break;
		case E8895_SICD_VETO_I2C:
			v->adap = of_find_i2c_adapter_by_node(a.np);
			if (!v->adap) {
				bad++;
				break;
			}
			k++;
			break;
		default:
			bad++;
		}
next:
		of_node_put(a.np);
	}
	if (bad) {
		for (i = 0; i < k; i++)
			e8895_veto_put(&arr[i]);
		kfree(arr);
		if (bad != e8895_veto_last_bad)
			pr_warn("exynos8895-pm: %d SICD veto entries unresolved: SICD off until they bind\n",
				bad);
		e8895_veto_last_bad = bad;
		goto out_node;
	}
	e8895_vetoes = arr;
	e8895_nveto = k;
	/* the array and its count are visible before the flag */
	smp_store_release(&e8895_veto_ready, true);
out_node:
	of_node_put(pm);
}

static void e8895_pm_try_resolve(void)
{
	mutex_lock(&e8895_veto_lock);
	/*
	 * Not before a successful probe: e8895_sicd_prepare() uses e8895_va[]
	 * as soon as the list is published. The acquire pairs with the release
	 * in e8895_pm_resolve_vetoes_locked().
	 */
	if (!smp_load_acquire(&e8895_veto_ready) && READ_ONCE(e8895_veto_late) &&
	    READ_ONCE(e8895_pm_probed))
		e8895_pm_resolve_vetoes_locked();
	mutex_unlock(&e8895_veto_lock);
}

/* Every built-in driver had its first chance to probe; later binds retry */
static int __init e8895_pm_resolve_vetoes(void)
{
	WRITE_ONCE(e8895_veto_late, true);
	e8895_pm_try_resolve();
	return 0;
}
late_initcall_sync(e8895_pm_resolve_vetoes);
