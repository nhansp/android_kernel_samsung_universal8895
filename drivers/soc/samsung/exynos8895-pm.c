// SPDX-License-Identifier: GPL-2.0-only
/*
 * Exynos 8895 system power management: system suspend to SLEEP (and its
 * VTS_ON/AUD_ON variants) with the stock PMUCAL sequences, the SICD entry/exit
 * and veto for the cpuidle driver, the BUCK8-11 suspend voltages (PM-DVS), the
 * UART_BT pad release and the optional APM sleep handshake.
 *
 * Port of S:drivers/soc/samsung/exynos-pm.c, the system-power-mode part of
 * S:drivers/soc/samsung/exynos-powermode.c, S:drivers/soc/samsung/exynos-pm-dvs.c
 * and the PMUCAL table engine S:drivers/soc/samsung/cal-if/pmucal_rae.c; the
 * sequences are generated from CAL:pmucal_system_exynos8895.h (CAL: =
 * S:drivers/soc/samsung/cal-if/exynos8895/) into exynos8895-pm-tables.h.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/firmware/samsung/exynos-acpm-protocol.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/psci.h>
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

struct e8895_idle_stats e8895_stats;

/* ---- registers and constants ---- */

#define E8895_WAKEUP_STAT	0x0600	/* S:drivers/soc/samsung/exynos-pm.c:52 */
#define E8895_EINT_WAKEUP_MASK	0x060C	/* S:drivers/soc/samsung/exynos-pm.c:53; S:drivers/soc/samsung/exynos-powermode.c:624 */
#define E8895_EXTRA_WAKEUP_STAT	0x0640	/* FDT:4421 extra_wakeup_stat = <0x640> */
#define E8895_CP_STAT		0x0038	/* S:drivers/soc/samsung/exynos-pmu.c:193 */
#define E8895_IDLE_IP(x)	(0x03E0 + (x) * 4)	/* S:drivers/soc/samsung/exynos-powermode.c:112,114 */
#define E8895_IDLE_IP_MASK(x)	(0x03F0 + (x) * 4)	/* S:drivers/soc/samsung/exynos-powermode.c:113,115 */
#define E8895_FIX_IDLE_IP	0x60	/* FDT:4430-4431 fix-idle-ip "acpm_dvfs", <0x60>: IDLE_IP3 bit 0 */
#define E8895_PAD_RET_UART_BT	0x3188	/* S:drivers/tty/serial/samsung.c:133; CAL:pmucal_system_exynos8895.h:1515 (commented out) */
#define E8895_PWM_TCON		0x08	/* A:drivers/pwm/pwm-samsung.c:31 REG_TCON */
#define E8895_PWM_TCON_START	0x00111101	/* TCON_START() of TCON channels 0, 2, 3, 4, 5, A:drivers/pwm/pwm-samsung.c:50,114-117 */

/* S:drivers/soc/samsung/cal-if/pmucal_system.h:20-34 (enum sys_powerdown) */
enum {
	E8895_SYS_SICD = 0,
	E8895_SYS_SLEEP = 8,
	E8895_SYS_SLEEP_VTS_ON = 9,
	E8895_SYS_SLEEP_AUD_ON = 10,
};

/* wakeup-mask words, one column per mode (FDT:4441-4462) */
static const u32 e8895_wkmask_off[4] = { 0x610, 0x614, 0x618, 0x644 };	/* FDT:4445,4450,4455,4460 */
static const u32 e8895_wkmask_sicd[4] = { 0x400001e0, 0x0, 0x0, 0x4 };	/* FDT:4444,4449,4454,4459 column 0 */
static const u32 e8895_wkmask_sleep[4] = { 0x500f7e7e, 0xffff00ff, 0xffff00ff, 0x4 };	/* columns 8, 9, 10 (equal) */

/*
 * PD STATUS registers: the first WAIT on each <PD>_STATUS of
 * CAL:pmucal_local_exynos8895.h (line in the comment). sicd = the PD's
 * idle-ip index is in the SYS_SICD ref-idle-ip list (FDT:4437).
 */
static const struct {
	const char *name;
	u16 off;
	bool sicd;
} e8895_pd_stat[] = {
	{ "abox",  0x4004, false },	/* :3   idle-ip 0x39, not in the SICD ref list */
	{ "cam",   0x4024, true  },	/* :73  0x3a */
	{ "dbg",   0x4044, false },	/* :125 0x3b, not in the SICD ref list */
	{ "dcam",  0x5224, true  },	/* :169 0x3c */
	{ "dpu0",  0x4064, true  },	/* :200 0x3d */
	{ "dpu1",  0x4084, true  },	/* :244 0x3e */
	{ "dsp",   0x40a4, true  },	/* :281 0x3f */
	{ "g2d",   0x40c4, true  },	/* :316 0x40 */
	{ "g3d",   0x40e4, true  },	/* :362 EMBEDDED_G3D_STATUS, 0x41 */
	{ "isphq", 0x4104, true  },	/* :382 0x42 */
	{ "isplp", 0x4124, true  },	/* :413 0x43 */
	{ "iva",   0x4144, true  },	/* :448 0x44 */
	{ "mfc",   0x4164, true  },	/* :499 0x45 */
	{ "srdz",  0x5424, true  },	/* :533 0x46 */
	{ "vpu",   0x4184, true  },	/* :568 0x47 */
	{ "vts",   0x41a4, false },	/* :600 0x48, not in the SICD ref list */
};

/* ON = STATUS[3:0] == 0xF, the COND mask/value of the stock save list (CAL:pmucal_system_exynos8895.h:924) */
#define E8895_PD_ON(off)	((pmu_raw_readl(off) & 0xf) == 0xf)
#define E8895_PD_ABOX		0x4004
#define E8895_PD_VTS		0x41a4

/* ---- PMUCAL table engine: S:drivers/soc/samsung/cal-if/pmucal_rae.c:62-243 ---- */

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
	u8 type, base;			/* generated, in this order */
	u16 off;
	u32 mask, val;
	u8 cbase;			/* generated: condition block, E8895_NO_COND if none */
	u16 coff;
	u32 cmask, cval;
	u32 saved;			/* runtime */
	bool need_restore, need_skip;
};

#include "exynos8895-pm-tables.h"

static void __iomem *e8895_va[ARRAY_SIZE(e8895_pm_bases)];

/* pmucal_rae_check_condition(), :62-71 */
static bool e8895_cond(const struct e8895_seq *s)
{
	return (readl_relaxed(e8895_va[s->cbase] + s->coff) & s->cmask) == s->cval;
}

/* pmucal_rae_check_value(), :73-82 */
static bool e8895_val_ok(const struct e8895_seq *s)
{
	return (readl_relaxed(e8895_va[s->base] + s->off) & s->mask) == s->val;
}

/* pmucal_rae_write(), :112-122: full write for a full mask, else RMW */
static void e8895_write(const struct e8895_seq *s, u32 v)
{
	void __iomem *a = e8895_va[s->base] + s->off;

	writel_relaxed(s->mask == U32_MAX ? v : (readl_relaxed(a) & ~s->mask) | v, a);
}

/* pmucal_rae_wait(), :84-103: check, udelay(1), give up after 1000 */
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

/* enter/exit/early-wakeup lists, pmucal_rae_handle_seq(), :219-250 (these lists hold only
 * WRITE, COND_WRITE and WAIT entries) */
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

/* pmucal_rae_save_seq(), :124-149 */
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

/* pmucal_rae_restore_seq(), :151-208 */
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
			/* :180-193: skip the next entry (the PLL lock WAIT) when the PD is
			 * off or the check value already holds */
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

/* One set of lists per SLEEP flavour: 0 enter, 1 save, 2 exit, 3 early wakeup
 * (S:drivers/soc/samsung/cal-if/pmucal_system.c:12-143) */
struct e8895_tabs {
	struct e8895_seq *t[4];
	unsigned int n[4];
};

#define E8895_TABS(x) {								\
	{ e8895_enter_##x, e8895_save_##x, e8895_exit_##x, e8895_earlywkup_##x },	\
	{ ARRAY_SIZE(e8895_enter_##x), ARRAY_SIZE(e8895_save_##x),		\
	  ARRAY_SIZE(e8895_exit_##x), ARRAY_SIZE(e8895_earlywkup_##x) } }

static struct e8895_tabs e8895_sleep_tabs[3] = {		/* modes 8, 9, 10 */
	E8895_TABS(sleep), E8895_TABS(sleep_vts_on), E8895_TABS(sleep_aud_on)
};

static struct e8895_tabs *e8895_tab(int mode)
{
	return &e8895_sleep_tabs[mode - E8895_SYS_SLEEP];
}

/* ---- SICD veto and SICD entry/exit (cpuidle driver, c2_lock held, IRQs off) ---- */

struct e8895_veto {
	u8 kind;			/* E8895_SICD_VETO_* */
	const char *name;		/* clock name, device name or i2c-<n>, for the heartbeat */
	struct clk *clk;		/* CLK */
	struct device *dev;		/* RPM, PWM */
	struct i2c_adapter *adap;	/* I2C */
	void __iomem *tcon;		/* PWM */
};

static struct e8895_veto *e8895_vetoes;
static unsigned int e8895_nveto;
static bool e8895_veto_ready;		/* published with smp_store_release() by the resolver */
static const char *e8895_last_veto = "init";

const char *e8895_sicd_last_veto(void)
{
	return READ_ONCE(e8895_last_veto);
}

/*
 * The 6.18 replacement for stock's software IDLE_IP votes: lock-free reads of
 * framework state that is busy exactly when stock's vote was.
 */
static bool e8895_veto_busy(const struct e8895_veto *v)
{
	switch (v->kind) {
	case E8895_SICD_VETO_CLK:
		/* gate of an always-on CMU without runtime PM: a plain register read,
		 * A:drivers/clk/clk.c:315-361 */
		return __clk_is_enabled(v->clk);
	case E8895_SICD_VETO_RPM:
		/* runtime PM off counts as busy, A:include/linux/pm_runtime.h */
		return !pm_runtime_enabled(v->dev) || !pm_runtime_status_suspended(v->dev);
	case E8895_SICD_VETO_I2C:
		/* held for every transfer, A:include/linux/i2c.h:741; A:include/linux/rtmutex.h:42-45 */
		return rt_mutex_base_is_locked(&v->adap->bus_lock.rtmutex);
	case E8895_SICD_VETO_PWM:
		/* unbound driver: the clock may be off and nothing runs */
		return READ_ONCE(v->dev->driver) && (readl_relaxed(v->tcon) & E8895_PWM_TCON_START);
	}
	return true;
}

static bool e8895_sicd_vetoed(void)
{
	unsigned int i;

	/* pairs with the release in e8895_pm_resolve_vetoes_locked(): array and count visible */
	if (!smp_load_acquire(&e8895_veto_ready))
		return true;
	/* stock pd-* idle-IPs, S:drivers/soc/samsung/exynos-pd.c:65,95 */
	for (i = 0; i < ARRAY_SIZE(e8895_pd_stat); i++)
		if (e8895_pd_stat[i].sicd && E8895_PD_ON(e8895_pd_stat[i].off)) {
			WRITE_ONCE(e8895_last_veto, e8895_pd_stat[i].name);
			return true;
		}
	/* fixed idle-ip "acpm_dvfs" (FDT:4430-4431), written by the APM firmware */
	if (!(pmu_raw_readl(E8895_IDLE_IP(E8895_FIX_IDLE_IP / 32)) & BIT(E8895_FIX_IDLE_IP % 32))) {
		WRITE_ONCE(e8895_last_veto, "apm");
		return true;
	}
	for (i = 0; i < e8895_nveto; i++)
		if (e8895_veto_busy(&e8895_vetoes[i])) {
			WRITE_ONCE(e8895_last_veto, e8895_vetoes[i].name);
			return true;
		}
	return false;
}

/* S:drivers/soc/samsung/exynos-powermode.c:465-497 + 638-663 (mode SICD) */
int e8895_sicd_prepare(void)
{
	unsigned int i;

	if (e8895_sicd_vetoed())
		return -EBUSY;
	/*
	 * IDLE_IP_MASK: only the fixed APM index unmasked. No 6.18 driver writes
	 * the software IDLE_IP0-2 bits, so they hold whatever S-Boot left; the
	 * same decision is made in software just above.
	 */
	for (i = 0; i < 4; i++)
		pmu_raw_writel(i == E8895_FIX_IDLE_IP / 32 ? (u32)~BIT(E8895_FIX_IDLE_IP % 32) : ~0u,
			       E8895_IDLE_IP_MASK(i));
	pmu_raw_writel(exynos_pinctrl_get_eint_wake_mask(), E8895_EINT_WAKEUP_MASK);	/* :625-631 */
	for (i = 0; i < 4; i++)
		pmu_raw_writel(e8895_wkmask_sicd[i], e8895_wkmask_off[i]);		/* :633-635 */
	return e8895_handle(e8895_enter_sicd, ARRAY_SIZE(e8895_enter_sicd));		/* :647 */
}

/* S:drivers/soc/samsung/exynos-powermode.c:665-679 */
void e8895_sicd_exit(bool early)
{
	if (early)
		e8895_handle(e8895_earlywkup_sicd, ARRAY_SIZE(e8895_earlywkup_sicd));
	else
		e8895_handle(e8895_exit_sicd, ARRAY_SIZE(e8895_exit_sicd));
}

/* ---- system suspend (SLEEP) ---- */

static int e8895_mode;
static bool e8895_early;
static bool e8895_sleep_armed;		/* syscore suspend programmed the PMU for SLEEP */
static bool cp_check = true;		/* stock always checks CP; 0 only for boots without a running CP */
module_param(cp_check, bool, 0644);

/* = A:drivers/firmware/psci/psci.c:502-510, but used for the TYPE-0 Exynos states too:
 * stock always passes cpu_resume (S:arch/arm64/kernel/psci.c:212-255) */
/* DEBUG: the finisher's own return (psci_to_linux_errno() of the SMC result) of the last
 * call on this CPU; cpu_suspend() turns a PSCI SUCCESS (0) into -EOPNOTSUPP
 * (A:arch/arm64/kernel/suspend.c:153-154), so -95 alone does not tell SUCCESS from NOT_SUPPORTED */
DEFINE_PER_CPU(int, e8895_dbg_fin_ret);

static noinstr int e8895_psci_finisher(unsigned long state)
{
	int ret = psci_ops.cpu_suspend(state, __pa_symbol_nodebug((unsigned long)cpu_resume));

	__this_cpu_write(e8895_dbg_fin_ret, ret);
	return ret;
}

int e8895_psci_suspend(u32 state)
{
	/* A:arch/arm64/kernel/suspend.c:97; S:arch/arm64/kernel/psci.c:254,274 */
	return cpu_suspend(state, e8895_psci_finisher);
}

/* S:drivers/soc/samsung/exynos-pm.c:231-255 */
static int e8895_pm_syscore_suspend(void)
{
	struct e8895_tabs *t;
	unsigned int i;

	e8895_sleep_armed = false;
	/* hibernation's create_image() runs syscore_suspend()/syscore_resume() too
	 * (A:kernel/power/hibernate.c:346,373); only a mem suspend sets this state
	 * (A:kernel/power/suspend.c:525, reset at :562). s2idle runs no syscore ops. */
	if (pm_suspend_target_state != PM_SUSPEND_MEM)
		return 0;
	if (cp_check && !pmu_raw_readl(E8895_CP_STAT)) {	/* :233-237 */
		pr_info("exynos8895-pm: sleep canceled by CP reset\n");
		return -EINVAL;
	}
	if (E8895_PD_ON(E8895_PD_ABOX))			/* abox_is_on(), :239 */
		e8895_mode = E8895_SYS_SLEEP_AUD_ON;	/* FDT:4419 cp_call_mode_idx = <0x0a> */
	else if (E8895_PD_ON(E8895_PD_VTS))		/* vts_is_on(), :245 */
		e8895_mode = E8895_SYS_SLEEP_VTS_ON;	/* S:drivers/soc/samsung/exynos-pm.c:227 SLEEP_VTS_ON 9 */
	else
		e8895_mode = E8895_SYS_SLEEP;		/* FDT:4417 suspend_mode_idx = <0x08> */

	/* only SYS_SICD has a ref-idle-ip node (FDT:4433-4439): the SLEEP masks are all-ones,
	 * S:drivers/soc/samsung/exynos-powermode.c:171-182 */
	for (i = 0; i < 4; i++)
		pmu_raw_writel(~0u, E8895_IDLE_IP_MASK(i));
	pmu_raw_writel(exynos_pinctrl_get_eint_wake_mask(), E8895_EINT_WAKEUP_MASK);	/* :625-631 */
	for (i = 0; i < 4; i++)
		pmu_raw_writel(e8895_wkmask_sleep[i], e8895_wkmask_off[i]);		/* :633-635 */
	t = e8895_tab(e8895_mode);
	e8895_save(t->t[1], t->n[1]);			/* S:drivers/soc/samsung/cal-if/pmucal_system.c:31 */
	if (e8895_handle(t->t[0], t->n[0])) {		/* :33-39; only the VTS_ON list has a WAIT */
		/* deviation: stock drops this error and enters SLEEP anyway
		 * (S:drivers/soc/samsung/exynos-pm.c:241-248); here the half-programmed
		 * PMU is rolled back and the suspend aborted */
		pr_err("exynos8895-pm: SLEEP mode=%d enter list: wait timed out\n", e8895_mode);
		e8895_handle(t->t[3], t->n[3]);
		e8895_restore(t->t[1], t->n[1]);
		return -EIO;
	}
	/*
	 * Until ->enter reports a real power-down the correct unwind is the
	 * early-wakeup list: suspend_enter() skips ->enter when a wakeup is
	 * pending or a later syscore op fails, but still runs syscore_resume()
	 * (A:kernel/power/suspend.c:471-484). Stock kept the previous cycle's
	 * flag (S:drivers/soc/samsung/exynos-pm.c:257-266,320).
	 */
	e8895_early = true;
	e8895_sleep_armed = true;
	pr_info("exynos8895-pm: SLEEP mode=%d\n", e8895_mode);
	return 0;
}

/* S:drivers/soc/samsung/exynos-pm.c:257-273 */
static void e8895_pm_syscore_resume(void)
{
	struct e8895_tabs *t;
	u32 stat;

	if (!e8895_sleep_armed)			/* hibernation snapshot, or nothing programmed */
		return;
	e8895_sleep_armed = false;
	t = e8895_tab(e8895_mode);
	/* S:drivers/soc/samsung/cal-if/pmucal_system.c:74,123 */
	e8895_handle(t->t[e8895_early ? 3 : 2], t->n[e8895_early ? 3 : 2]);
	if (e8895_restore(t->t[1], t->n[1]))		/* :82-88, :131-137 */
		pr_err("exynos8895-pm: restore: PLL/OSC lock timeout\n");
	stat = pmu_raw_readl(E8895_WAKEUP_STAT);
	/* S:drivers/soc/samsung/exynos-pm.c:44-47,156-169 */
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

/* S:drivers/soc/samsung/exynos-pm.c:295-326; cp_call_psci_idx = suspend_psci_idx = 0x85
 * (FDT:4418,4420), so every mode uses PSCI_SYSTEM_SLEEP */
static int e8895_pm_enter(suspend_state_t state)
{
	e8895_early = e8895_psci_suspend(E8895_PSCI_SLEEP) != 0;
	if (e8895_early)
		pr_info("exynos8895-pm: return to originator\n");	/* :321-323 */
	return e8895_early ? -EBUSY : 0;
}

static const struct platform_suspend_ops e8895_pm_ops = {
	.valid = suspend_valid_only_mem,	/* S:drivers/soc/samsung/exynos-pm.c:333 */
	.enter = e8895_pm_enter,
};

/* ---- device PM callbacks ---- */

/* after every device's .resume (A:drivers/base/power/main.c dpm_complete()) */
static void e8895_pm_complete(struct device *dev)
{
	/* still MEM here: it is reset only after dpm_resume_end() (A:kernel/power/suspend.c:554,562) */
	if (pm_suspend_target_state != PM_SUSPEND_MEM)
		return;
	/*
	 * S:drivers/tty/serial/samsung.c:2123-2124: exynos_pmu_update(
	 * PAD_RETENTION_UART_BT_OPTION, 0x10000000, 0x10000000) after
	 * uart_resume_port() of the BT port. RMW like stock; harmless when no
	 * retention was active.
	 */
	pmu_raw_writel(pmu_raw_readl(E8895_PAD_RET_UART_BT) | EXYNOS_WAKEUP_FROM_LOWPWR,
		       E8895_PAD_RET_UART_BT);
	atomic_inc(&e8895_stats.ubt);
}

/* PM-DVS: FDT:12134-12164 (stock list order), S:drivers/soc/samsung/exynos-pm-dvs.c:24-127 */
static const struct {
	const char *supply;
	u32 suspend_uv, init_uv;
} e8895_dvs[] = {
	{ "vdd-mldo",  1887500, 1950000 },	/* FDT:12139-12140 <0x1ccd0c>/<0x1dc130> BUCK11 */
	{ "vdd-lldo1",  900000,  950000 },	/* FDT:12146-12147 <0xdbba0>/<0xe7ef0>  BUCK8 */
	{ "vdd-lldo2", 1050000, 1100000 },	/* FDT:12153-12154 <0x100590>/<0x10c8e0> BUCK9 */
	{ "vdd-lldo3", 1250000, 1300000 },	/* FDT:12160-12161 <0x1312d0>/<0x13d620> BUCK10 */
};

#define E8895_DVS_RANGE		25000		/* FDT:12141 volt_range_step = <0x61a8> */

static struct regulator *e8895_dvs_reg[ARRAY_SIZE(e8895_dvs)];
static bool e8895_dvs_lowered;		/* suspend_late lowered the rails */

/* pr_info with a fixed prefix: dev_info() would print the full device name */
static int e8895_dvs_set(unsigned int i, bool suspend)
{
	u32 uv = suspend ? e8895_dvs[i].suspend_uv : e8895_dvs[i].init_uv;
	int ret = regulator_set_voltage(e8895_dvs_reg[i], uv, uv + E8895_DVS_RANGE);

	pr_info("exynos8895-pm: dvs %s -> %u: %d\n", e8895_dvs[i].supply, uv, ret);
	return ret;
}

/* S:drivers/soc/samsung/exynos-pm-dvs.c:98-112 */
static int e8895_dvs_suspend_late(struct device *dev)
{
	unsigned int i;
	int ret;

	/* only around SLEEP, as stock (stock had no s2idle and no hibernation) */
	if (pm_suspend_target_state != PM_SUSPEND_MEM)
		return 0;
	for (i = 0; i < ARRAY_SIZE(e8895_dvs); i++) {
		if (!e8895_dvs_reg[i])
			continue;
		ret = e8895_dvs_set(i, true);
		if (ret) {
			/*
			 * Stock breaks and returns (:106-109), which aborts the
			 * suspend. Deviation: the PM core does not call resume_early
			 * for a device whose suspend_late failed, so the rails
			 * already lowered would stay low; restore them here.
			 */
			while (i--)
				if (e8895_dvs_reg[i])
					e8895_dvs_set(i, false);
			return ret;
		}
	}
	e8895_dvs_lowered = true;
	return 0;
}

/* S:drivers/soc/samsung/exynos-pm-dvs.c:114-127 */
static int e8895_dvs_resume_early(struct device *dev)
{
	unsigned int i;
	int ret, err = 0;

	if (!e8895_dvs_lowered)
		return 0;
	e8895_dvs_lowered = false;
	for (i = 0; i < ARRAY_SIZE(e8895_dvs); i++) {
		if (!e8895_dvs_reg[i])
			continue;
		ret = e8895_dvs_set(i, false);
		/* deviation: stock stops at the first error; here every rail is still restored */
		err = err ? : ret;
	}
	return err;
}

/*
 * APM sleep handshake of the MIF and INT DVFS domains (stock devfreq
 * .suspend/.resume, S:drivers/devfreq/exynos/exynos-devfreq.c:1866-2016).
 * Off by default: L2 does not port the boot-time DATA_INIT half of the
 * protocol (spec L7 O7-12).
 */
static bool apm_release;
module_param(apm_release, bool, 0644);

#define E8895_DVFS_ACPM_CH	1	/* FDT:6729,6749 acpm-ipc-channel = <0x01> */

/* DEVFREQ_MIF, DEVFREQ_INT: S:include/soc/samsung/exynos-devfreq.h:25-27; use_acpm FDT:6730,6750 */
static const u8 e8895_dvfs_types[] = { 0, 1 };
static struct acpm_handle *e8895_acpm;
static bool e8895_apm_sent;		/* the "AP sleeping" message went out in this cycle */

static void e8895_apm_release(bool running)
{
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(e8895_dvfs_types); i++) {
		ret = e8895_acpm->ops.dvfs_ops.sleep_release(e8895_acpm, E8895_DVFS_ACPM_CH,
							     e8895_dvfs_types[i], running);
		pr_info("exynos8895-pm: apm release type=%u running=%d: %d\n",
			e8895_dvfs_types[i], running, ret);
	}
}

static int e8895_pm_suspend(struct device *dev)
{
	if (pm_suspend_target_state == PM_SUSPEND_MEM && READ_ONCE(apm_release) && e8895_acpm) {
		e8895_apm_release(false);
		e8895_apm_sent = true;
	}
	return 0;
}

static int e8895_pm_resume(struct device *dev)
{
	if (e8895_apm_sent) {
		e8895_apm_sent = false;
		e8895_apm_release(true);
	}
	return 0;
}

/*
 * Explicit members only: SYSTEM_SLEEP_PM_OPS/LATE_SYSTEM_SLEEP_PM_OPS would
 * also fill freeze/thaw/poweroff/restore (A:include/linux/pm.h:316-330), i.e.
 * run PM-DVS and the APM message during hibernation. Stock set only
 * .suspend_late/.resume_early (S:drivers/soc/samsung/exynos-pm-dvs.c:129-132).
 */
static const struct dev_pm_ops e8895_pm_dev_ops = {
	.suspend      = e8895_pm_suspend,
	.resume       = e8895_pm_resume,
	.suspend_late = e8895_dvs_suspend_late,
	.resume_early = e8895_dvs_resume_early,
	.complete     = e8895_pm_complete,
};

/* ---- probe, veto resolution ---- */

static DEFINE_MUTEX(e8895_veto_lock);
static bool e8895_veto_late;		/* late_initcall_sync has run */
static bool e8895_pm_probed;		/* probe succeeded: e8895_va[] mapped for good */
static int e8895_veto_last_bad = -1;	/* print "unresolved" once per distinct count */
static void e8895_pm_try_resolve(void);

/*
 * Every veto target (clock provider, I2C controller, RPM/PWM device) is a
 * platform device; BUS_NOTIFY_BOUND_DRIVER is sent after its probe returned 0
 * (A:drivers/base/dd.c driver_bound()).
 */
static int e8895_pm_bus_notify(struct notifier_block *nb, unsigned long action, void *data)
{
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
	struct device_node *np;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(e8895_pm_bases); i++) {
		/* no region claim: CMU/SYSREG blocks also mapped by their drivers */
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
	np = of_parse_phandle(dev->of_node, "samsung,acpm", 0);
	if (np) {
		e8895_acpm = devm_acpm_get_by_node(dev, np);
		of_node_put(np);
		if (IS_ERR(e8895_acpm)) {
			if (PTR_ERR(e8895_acpm) == -EPROBE_DEFER)
				return -EPROBE_DEFER;
			e8895_acpm = NULL;
		}
	}
	/* SLEEP offlines CPUs 1-7 through the PMU CPU hooks' DYING callback */
	if (!e8895_pmu_cpu_hooks_ready()) {
		dev_err(dev, "PMU CPU hooks not ready: no SLEEP, mem falls back to s2idle\n");
		return -ENODEV;
	}
	WRITE_ONCE(e8895_pm_probed, true);
	register_syscore_ops(&e8895_pm_syscore_ops);
	suspend_set_ops(&e8895_pm_ops);			/* A:kernel/power/suspend.c:224-241: mem = deep */
	bus_register_notifier(&platform_bus_type, &e8895_pm_bus_nb);
	dev_info(dev, "SLEEP via PSCI 0x%08x, %zu PMUCAL blocks\n", E8895_PSCI_SLEEP,
		 ARRAY_SIZE(e8895_pm_bases));
	e8895_pm_try_resolve();				/* no-op before late_initcall_sync */
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

/* drop what one resolved entry holds */
static void e8895_veto_put(struct e8895_veto *v)
{
	if (v->clk)
		clk_put(v->clk);
	if (v->tcon)
		iounmap(v->tcon - E8895_PWM_TCON);
	if (v->dev)
		put_device(v->dev);
	if (v->adap)
		put_device(&v->adap->dev);	/* A:include/linux/i2c.h:1033: "must call put_device()" */
}

/* Resolve samsung,sicd-veto into a private array; publish it only when every entry resolved. */
static void e8895_pm_resolve_vetoes_locked(void)
{
	struct device_node *pm = of_find_compatible_node(NULL, NULL, "samsung,exynos8895-pm");
	struct e8895_veto *arr;
	struct of_phandle_args a;
	int n, i, k = 0, bad = 0;

	if (!pm)
		return;
	n = of_property_count_u32_elems(pm, "samsung,sicd-veto") / 2;	/* phandle + 1 cell */
	if (n <= 0) {
		WRITE_ONCE(e8895_last_veto, "nolist");
		goto out_node;
	}
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
		if (!of_device_is_available(a.np))	/* disabled node: its IP never runs */
			goto next;
		v->kind = a.args[0];
		switch (v->kind) {
		case E8895_SICD_VETO_CLK: {
			struct of_phandle_args ca;

			v->clk = of_clk_get(a.np, 0);	/* -EPROBE_DEFER until the CMU exists: retried */
			if (IS_ERR(v->clk)) {
				v->clk = NULL;
				bad++;
				break;
			}
			/* the gate's CMU must not use runtime PM: __clk_is_enabled() would then
			 * call pm_runtime_put() from the idle path (A:drivers/clk/clk.c:336-358) */
			if (!of_parse_phandle_with_args(a.np, "clocks", "#clock-cells", 0, &ca)) {
				/* NULL for a CLK_OF_DECLARE CMU */
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
			v->name = __clk_get_name(v->clk);
			k++;
			break;
		}
		case E8895_SICD_VETO_RPM:
		case E8895_SICD_VETO_PWM:
			pdev = of_find_device_by_node(a.np);	/* reference kept */
			if (!pdev) {
				bad++;
				break;
			}
			v->dev = &pdev->dev;
			if (v->kind == E8895_SICD_VETO_PWM) {
				v->tcon = of_iomap(a.np, 0);
				if (!v->tcon) {
					e8895_veto_put(v);
					memset(v, 0, sizeof(*v));
					bad++;
					break;
				}
				v->tcon += E8895_PWM_TCON;
			}
			v->name = dev_name(v->dev);
			k++;
			break;
		case E8895_SICD_VETO_I2C:
			/* NULL until the controller bound; A:include/linux/i2c.h:1033-1037 */
			v->adap = of_find_i2c_adapter_by_node(a.np);
			if (!v->adap) {
				bad++;
				break;
			}
			v->name = dev_name(&v->adap->dev);	/* "i2c-<n>" */
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
		WRITE_ONCE(e8895_last_veto, "unresolved");
		if (bad != e8895_veto_last_bad)
			pr_warn("exynos8895-pm: %d SICD veto entries unresolved: SICD off until they bind\n",
				bad);
		e8895_veto_last_bad = bad;
		goto out_node;
	}
	e8895_vetoes = arr;
	e8895_nveto = k;
	smp_store_release(&e8895_veto_ready, true);	/* pairs with smp_load_acquire() in e8895_sicd_vetoed() */
	pr_info("exynos8895-pm: %u SICD veto entries\n", k);
out_node:
	of_node_put(pm);
}

static void e8895_pm_try_resolve(void)
{
	mutex_lock(&e8895_veto_lock);
	/* not before a successful probe: a deferred probe releases e8895_va[], and
	 * e8895_sicd_prepare() uses them as soon as the veto list is published */
	if (!smp_load_acquire(&e8895_veto_ready) && READ_ONCE(e8895_veto_late) &&
	    READ_ONCE(e8895_pm_probed))
		e8895_pm_resolve_vetoes_locked();
	mutex_unlock(&e8895_veto_lock);
}

/* first attempt once every built-in driver had its first chance to probe; the
 * probe and the platform-bus notifier retry until every entry resolved */
static int __init e8895_pm_resolve_vetoes(void)
{
	WRITE_ONCE(e8895_veto_late, true);
	e8895_pm_try_resolve();
	return 0;
}
late_initcall_sync(e8895_pm_resolve_vetoes);
