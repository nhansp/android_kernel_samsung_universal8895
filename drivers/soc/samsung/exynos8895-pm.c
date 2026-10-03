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
#include <linux/pwm.h>
#include <linux/regulator/consumer.h>
#include <linux/rtmutex.h>
#include <linux/slab.h>
#include <linux/soc/samsung/exynos-pmu.h>
#include <linux/soc/samsung/exynos-regs-pmu.h>
#include <linux/soc/samsung/exynos8895-pm.h>
#include <linux/suspend.h>
#include <linux/syscore_ops.h>
#include <trace/events/power.h>
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

/*
 * DEBUG (sleep_trace=1, image 0127: SLEEP is reached but nothing wakes it):
 * read back every WRITE of the enter list right after it ran and print the
 * words that do not hold the written value (a write the hardware ignored,
 * a self-clearing bit, or another master changing it). Works under
 * pm_test=core, i.e. without a real SLEEP.
 */
static void e8895_verify(const struct e8895_seq *s, unsigned int n)
{
	unsigned int i, bad = 0;

	for (i = 0; i < n; i++, s++) {
		u32 v;

		if (s->type != E8895_WRITE)
			continue;
		v = readl_relaxed(e8895_va[s->base] + s->off);
		if ((v & s->mask) != s->val) {
			bad++;
			pr_info("exynos8895-pm: DEBUG verify [%u] base %u +0x%04x = 0x%08x, wrote 0x%08x/0x%08x\n",
				i, s->base, s->off, v, s->val, s->mask);
		}
	}
	pr_info("exynos8895-pm: DEBUG verify: %u entries, %u differ\n", n, bad);
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
	case E8895_SICD_VETO_PWM: {
		/*
		 * Stock voted busy while its PWM driver had the timer gate enabled, i.e.
		 * while a channel it drives is enabled (S:drivers/pwm/pwm-samsung.c:
		 * 121-152 pwm_samsung_clk_enable()/_disable()), never on TCON itself.
		 * TCON is no proxy: S-Boot/EL3 leave timer 4 free-running (image 0131:
		 * TCON 0x0054440c, TCNTB4 0xffffffff, no Linux user), and stock leaves
		 * an unrequested channel alone. Busy = a channel the PWM framework has
		 * enabled; drvdata is the struct pwm_chip (A:drivers/pwm/pwm-samsung.c:609).
		 */
		struct pwm_chip *chip;
		unsigned int i;

		if (!READ_ONCE(v->dev->driver))		/* unbound: nothing of ours runs */
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

/*
 * DEBUG (stage L7-C, image 0096: no resume, last console line "Calling
 * samsung_clk_suspend", i.e. the last syscore suspend op before ->enter):
 *   sleep_trace=1      print the PMU wake/sequencer state right before the SLEEP SMC,
 *                      the SMC result right after it (the first code that runs after
 *                      a wake, before any syscore resume op) and the syscore resume entry
 *   sleep_mode=8|9|10  force SLEEP / SLEEP_VTS_ON / SLEEP_AUD_ON instead of the
 *                      ABOX/VTS power-domain choice (0 = stock choice)
 *   sleep_wkmask0=<v>  write <v> to WAKEUP_MASK (0x610) instead of the stock SLEEP word
 *                      0x500f7e7e (FDT:4444), e.g. 0 to unmask every wake source
 * (image 0097, same end; the power key did not wake it either):
 *   sleep_trace=1      also lists every exynos8895-pd CONFIGURATION/STATUS before the SMC
 *                      (dpu1 and srdz fail to power off in suspend_noirq) and prints the
 *                      PMU wake/sequencer registers once at probe (post-mortem after a
 *                      forced reset, if S-Boot leaves them)
 *   sleep_psci=<ps>    PSCI power_state for the SLEEP SMC instead of 0x03000000, e.g.
 *                      0x10000 (C2, core only) with the whole SLEEP PMU programming in place
 * (image 0098, sleep_mode=8: same hang; ABOX and VTS PDs are on at the SMC, S-Boot leaves
 * them on and no 6.18 driver turns them off, while stock's ABOX/VTS drivers keep them off):
 *   sleep_pdoff=1      power ABOX and VTS off in syscore suspend with stock's abox_off/vts_off
 *                      lists (CAL:pmucal_local_exynos8895.h:61-65,636-641) before the mode
 *                      choice; a wait timeout aborts the suspend (-EIO). They are powered on
 *                      again in syscore resume or on a refusal (as sleep_alloff).
 * The sleep_trace dumps run at the end of syscore suspend, so pm_test=core prints them too.
 */
static bool sleep_trace;
static unsigned int sleep_mode;
static int sleep_wkmask0 = -1;
static unsigned int sleep_psci;
module_param(sleep_trace, bool, 0644);
module_param(sleep_mode, uint, 0644);
module_param(sleep_wkmask0, int, 0644);
module_param(sleep_psci, uint, 0644);
static bool sleep_pdoff;
module_param(sleep_pdoff, bool, 0644);
static u32 e8895_alloff_done;	/* DEBUG: e8895_pd_stat[] bits powered off by pdoff/alloff */

/*
 * DEBUG E6 control (l7re H3: sleep_psci=0x10000, i.e. C2, with the SLEEP PMU
 * programming in place still hangs, so the trigger is in the shared part:
 * CENTRAL_SEQ armed + cpu WFI, not mode-specific EL3 code): with
 * sleep_noarm=1 the syscore suspend leaves the CENTRAL sequencer disarmed,
 * so arm/no-arm can be compared without a real SLEEP.
 *
 * Which register/bit: CENTRAL_SEQ_CONFIGURATION 0x0200 bit16 and its MIF
 * pair CENTRAL_SEQ_MIF_CONFIGURATION 0x0240 bit16. The SLEEP enter list
 * clears both (CAL:pmucal_system_exynos8895.h:918-919; same pair in the SICD
 * enter list at :348-349); the exit and early-wakeup lists set both
 * (:1516-1517, :1581-1582). Clearing is the arming step: the exit/earlywkup
 * value (set) is the run-time idle state EL3/the PMU return to, while the
 * enter value (clear) is what the WFI-time sequence runs under. EL3 performs
 * its own arming on top (cluster pairs at SB:0x74cd4, CENTRAL ORs at
 * SB:0x751c8, MIF gate at SB:0x76628); this param touches only the kernel
 * side, restoring each bit to its pre-enter value so no polarity is assumed.
 * The resume path needs no change: the early-wakeup/exit lists set bit16
 * themselves. Default 0, no behaviour change.
 */
static bool sleep_noarm;
module_param(sleep_noarm, bool, 0644);

/*
 * DEBUG (image 0120: dpu1/g2d/vts/srdz power-off times out with STATUS still 0xf,
 * i.e. the PMU never starts the sequence). Runtime probes, no suspend needed:
 *   echo <pmu off> > /sys/module/exynos8895_pm/parameters/pd_poke
 *       CONFIGURATION offset of one domain (0x4080 dpu1, 0x40c0 g2d, 0x41a0 vts,
 *       0x5420 srdz): request off, trace STATUS for up to 20 ms, then request on
 *       again and wait (genpd keeps these domains on after their failed power-off)
 *   echo <phys> > /sys/module/exynos8895_pm/parameters/peek
 *       print one 32-bit register (only blocks that are clocked and powered:
 *       CMU_TOP 0x15a80000, CMU_DPU0 0x12800000, PMU 0x16480000)
 */
static int e8895_pd_poke_set(const char *val, const struct kernel_param *kp)
{
	u32 off, st, last, trace[8];
	unsigned int t, n = 0, i;
	int ret = kstrtou32(val, 0, &off);

	if (ret)
		return ret;
	if (!pmu_base_addr || off < 0x4000 || off > 0x5fe0 || off & 0x1f)
		return -EINVAL;
	pr_info("exynos8895-pm: DEBUG poke %x: CONFIGURATION %x STATUS %x OPTION %08x\n", off,
		pmu_raw_readl(off) & 0xf, pmu_raw_readl(off + 4) & 0xf, pmu_raw_readl(off + 8));
	/* an off domain proves nothing, and powering it on would desync genpd */
	if (!(pmu_raw_readl(off + 4) & 0xf)) {
		pr_info("exynos8895-pm: DEBUG poke %x: already off, not poked\n", off);
		return 0;
	}
	pmu_raw_writel(pmu_raw_readl(off) & ~0xf, off);
	last = pmu_raw_readl(off + 4);
	trace[n++] = last;
	for (t = 0; t < 20000 && (last & 0xf); t++) {
		udelay(1);
		st = pmu_raw_readl(off + 4);
		if (st != last && n < ARRAY_SIZE(trace))
			trace[n++] = st;
		last = st;
	}
	pr_info("exynos8895-pm: DEBUG poke %x: off %s after %u us, STATUS trace:\n", off,
		last & 0xf ? "FAILED" : "ok", t);
	for (i = 0; i < n; i++)
		pr_info("exynos8895-pm: DEBUG poke %x:   %08x\n", off, trace[i]);
	pmu_raw_writel(pmu_raw_readl(off) | 0xf, off);
	for (t = 0; t < 20000 && (pmu_raw_readl(off + 4) & 0xf) != 0xf; t++)
		udelay(1);
	pr_info("exynos8895-pm: DEBUG poke %x: on again: STATUS %x after %u us\n", off,
		pmu_raw_readl(off + 4) & 0xf, t);
	return 0;
}

static const struct kernel_param_ops e8895_pd_poke_ops = { .set = e8895_pd_poke_set };
module_param_cb(pd_poke, &e8895_pd_poke_ops, NULL, 0200);

static int e8895_peek_set(const char *val, const struct kernel_param *kp)
{
	void __iomem *va;
	u64 pa;
	int ret = kstrtou64(val, 0, &pa);

	if (ret)
		return ret;
	if (pa & 3)
		return -EINVAL;
	va = ioremap(pa, 4);
	if (!va)
		return -ENOMEM;
	pr_info("exynos8895-pm: DEBUG peek 0x%llx = 0x%08x\n", pa, readl(va));
	iounmap(va);
	return 0;
}

static const struct kernel_param_ops e8895_peek_ops = { .set = e8895_peek_set };
module_param_cb(peek, &e8895_peek_ops, NULL, 0200);

/*
 * DEBUG stage_poke=<mask> (image 0123: cam and iva refuse to power off at syscore time,
 * dpu1/srdz/g2d/vts in genpd noirq, while the same pokes succeed at run time). During a
 * mem suspend, power each selected domain that is on off and on again (pd_poke) at every
 * PM phase boundary (suspend_resume tracepoint) and after every device PM callback
 * (device_pm_callback_end), from PM_SUSPEND_PREPARE up to and including syscore suspend,
 * and once around each PM-DVS rail change. The first failure per domain is printed with
 * the step before it; later probes of that domain stop. Bit i = e8895_pd_stat[i];
 * 0x802 = cam + iva (no driver, no genpd: safe to cycle). Use with
 * echo 0 > /sys/power/pm_async, so that "after" names one callback.
 *   dvs_skip=1  PM-DVS suspend_late/resume_early do nothing (A/B for the rail drop)
 */
static unsigned int stage_poke;
module_param(stage_poke, uint, 0644);
static bool dvs_skip;
module_param(dvs_skip, bool, 0644);
static DEFINE_RAW_SPINLOCK(e8895_sp_lock);
static bool e8895_sp_active;
static u32 e8895_sp_failed;
static unsigned int e8895_sp_n;
static char e8895_sp_last[96] = "PM_SUSPEND_PREPARE";

/* 0: off and on again, 1: domain off (skipped), -1: power-off did not happen within 2 ms */
static int e8895_sp_poke1(u16 st)
{
	unsigned int t;
	bool ok;

	if (!(pmu_raw_readl(st) & 0xf))
		return 1;
	pmu_raw_writel(pmu_raw_readl(st - 4) & ~0xf, st - 4);
	for (t = 0; t < 2000 && (pmu_raw_readl(st) & 0xf); t++)
		udelay(1);
	ok = !(pmu_raw_readl(st) & 0xf);
	pmu_raw_writel(pmu_raw_readl(st - 4) | 0xf, st - 4);
	for (t = 0; t < 2000 && (pmu_raw_readl(st) & 0xf) != 0xf; t++)
		udelay(1);
	if ((pmu_raw_readl(st) & 0xf) != 0xf)
		pr_err("exynos8895-pm: DEBUG stagepoke: STATUS %x after on again\n",
		       pmu_raw_readl(st) & 0xf);
	return ok ? 0 : -1;
}

static void e8895_sp_probe(const char *a, const char *b)
{
	unsigned long flags;
	unsigned int i;
	u32 mask = READ_ONCE(stage_poke);

	if (!mask || !READ_ONCE(e8895_sp_active))
		return;
	raw_spin_lock_irqsave(&e8895_sp_lock, flags);
	e8895_sp_n++;
	for (i = 0; i < ARRAY_SIZE(e8895_pd_stat); i++) {
		if (!(mask & BIT(i)) || (e8895_sp_failed & BIT(i)))
			continue;
		if (e8895_sp_poke1(e8895_pd_stat[i].off) < 0) {
			e8895_sp_failed |= BIT(i);
			pr_info("exynos8895-pm: DEBUG stagepoke: %s FAILED at probe %u [%s %s], last ok [%s]\n",
				e8895_pd_stat[i].name, e8895_sp_n, a, b, e8895_sp_last);
		}
	}
	snprintf(e8895_sp_last, sizeof(e8895_sp_last), "%s %s", a, b);
	raw_spin_unlock_irqrestore(&e8895_sp_lock, flags);
}

static void e8895_sp_tp_dev(void *data, struct device *dev, int error)
{
	char who[96];

	snprintf(who, sizeof(who), "%s %s", dev_driver_string(dev), dev_name(dev));
	e8895_sp_probe("after", who);
}

static void e8895_sp_tp_phase(void *data, const char *action, int val, bool start)
{
	e8895_sp_probe(start ? "start" : "end", action);
}

static int e8895_sp_pm_nb_fn(struct notifier_block *nb, unsigned long ev, void *p)
{
	if (ev == PM_SUSPEND_PREPARE) {
		e8895_sp_failed = 0;
		e8895_sp_n = 0;
		strscpy(e8895_sp_last, "PM_SUSPEND_PREPARE", sizeof(e8895_sp_last));
		WRITE_ONCE(e8895_sp_active, true);
		e8895_sp_probe("PM_SUSPEND_PREPARE", "notifier");
	} else if (ev == PM_POST_SUSPEND) {
		if (READ_ONCE(stage_poke))
			pr_info("exynos8895-pm: DEBUG stagepoke: %u probes, failed %04x\n",
				e8895_sp_n, e8895_sp_failed);
		WRITE_ONCE(e8895_sp_active, false);
	}
	return NOTIFY_DONE;
}

static struct notifier_block e8895_sp_pm_nb = { .notifier_call = e8895_sp_pm_nb_fn };

static void e8895_sp_init(void)
{
	int r1, r2;

	register_pm_notifier(&e8895_sp_pm_nb);
	r1 = register_trace_device_pm_callback_end(e8895_sp_tp_dev, NULL);
	r2 = register_trace_suspend_resume(e8895_sp_tp_phase, NULL);
	if (r1 || r2)
		pr_warn("exynos8895-pm: DEBUG stagepoke tracepoints: %d %d\n", r1, r2);
}

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

/* DEBUG (sleep_trace=1): CONFIGURATION[3:0]/STATUS[3:0] of every PMU power domain in the DT */
static void e8895_pd_dump(void)
{
	struct device_node *np;
	struct resource res;
	char buf[160];
	int n = 0;

	for_each_compatible_node(np, NULL, "samsung,exynos8895-pd") {
		u32 off;

		if (of_address_to_resource(np, 0, &res) || res.start < 0x16480000 ||
		    res.start >= 0x16480000 + 0x6000)
			continue;
		off = res.start - 0x16480000;
		n += scnprintf(buf + n, sizeof(buf) - n, " %x:%x/%x", off,
			       pmu_raw_readl(off) & 0xf, pmu_raw_readl(off + 4) & 0xf);
		if (n > 120) {
			pr_info("exynos8895-pm: DEBUG pre-SMC pd%s\n", buf);
			n = 0;
		}
	}
	if (n)
		pr_info("exynos8895-pm: DEBUG pre-SMC pd%s\n", buf);
}

/* ================= DEBUG (l7wake): the wake path itself =================
 *
 * image 0139: no wake-up reaches CPU0 in suspend (RTC alarm through the PMIC
 * EINT gpa3-6 and the power key through gpa2-4, both armed as wake sources,
 * both dead). Everything the PMU shows at the SMC matches stock, so the
 * remaining kernel-visible hops - the ALIVE EINT block and the GIC that
 * carries it - were never dumped. This block prints them, and the two DEBUG
 * params after it can force them.
 *
 * Register map (all offsets, all opened this run):
 *   GIC distributor / CPU interface, GICv2 - the 8895 interrupt controller is
 *   "arm,gic-400" (exynos8895.dtsi:544) / "arm,cortex-a15-gic" (FDT:4542), i.e.
 *   drivers/irqchip/irq-gic.c (IRQCHIP_DECLARE(gic_400) :1516), NOT the v3
 *   driver; A:include/linux/irqchip/arm-gic.h:6-56
 *     GICD_CTLR 0x000, GICD_TYPER 0x004, GICD_ICTR 0x008,
 *     GICD_IGROUPR 0x080, GICD_ISENABLER 0x100, GICD_ISPENDR 0x200,
 *     GICD_IPRIORITYR 0x400, GICD_ITARGETSR 0x800 (4 bits per INTID)
 *     GICC_CTLR 0x000, GICC_PMR 0x004, GICC_BPR 0x008, GICC_EOI 0x010,
 *     GICC_RPR 0x014, GICC_HIGHPRI 0x018, GICC_HPPIR 0x04c
 *   GICC_DIR (0x1000) is NOT dumped: image 0142 (this patch on the test line)
 *   took a translation fault (0x96000007, level 3 = VA not mapped) in readl()
 *   from e8895_wake_dump+0x33c, which the 0142 vmlinux resolves to this very
 *   read (`add x0, x8, #0x1, lsl #12` then `bl readl`, x8 = e8895_gicc_base).
 *   DT reg #1 - the window of_iomap(node, 1) maps - is 0x1000 bytes
 *   (exynos8895.dtsi:546), so offset 0x1000 is the first byte outside the
 *   mapping. Stock never reads GICC_DIR either: its CPU interface window is
 *   mapped the same two reg entries (S:drivers/irqchip/irq-gic.c:1249 dist =
 *   of_iomap(node, 0), :1252 cpu = of_iomap(node, 1)), and the only path that
 *   makes +0x1000 reachable is gic_check_eoimode()'s "Move the base up by 60kB,
 *   so that we have a 8kB contiguous region, which allows us to use GICC_DIR at
 *   its normal offset" (:1224-1233), which that function itself skips when the
 *   CPU interface window is smaller than 8kB (:1207-1208) - 0x1000 here. Every
 *   read of this DEBUG block goes through e8895_rd() below, which refuses an
 *   offset the mapped size does not cover instead of faulting on it.
 *   ALIVE EINT block, 0x164b0000 (exynos8895.dtsi pinctrl_alive, FDT:88-92,
 *   reg 0x164b0000/0x1000): the wkup chip of the DT is
 *   samsung,exynos7-wakeup-eint (FDT:216), whose offsets are
 *   A:drivers/pinctrl/samsung/pinctrl-exynos.h:30-32 (ECON 0x700, EMASK 0x900,
 *   EPEND 0xA00) and the driver adds +bank->eint_offset (pinctrl-exynos.c:71,92);
 *   EFLTCON 0x800 is the same file, :23. PEND_SET 0xB00 is EXYNOS_SVC_OFFSET
 *   0xB08 minus 8 (same file, :32) minus the ECON base 0x700, i.e. the register
 *   the SVC (software clear) block follows; INT_ENABLE 0xC00 and LEVEL 0xD00 are
 *   the next two 0x100 steps of that stride - [INFERRED]: nothing in S: or A:
 *   names them (no EXYNOS_*_INT_ENABLE/INT_LEVEL exists in either pinctrl
 *   header), this DEBUG block is the only reader of them.
 *   Bank list S:drivers/pinctrl/samsung/pinctrl-exynos.c:1785-1790.
 */
#define E8895_GICD_CTLR		0x000
#define E8895_GICD_TYPER	0x004
#define E8895_GICD_ICTR		0x008
#define E8895_GICD_IGROUPR	0x080
#define E8895_GICD_ISENABLER	0x100
#define E8895_GICD_ISPENDR	0x200
#define E8895_GICD_ITARGETSR	0x800
#define E8895_GICC_CTLR		0x000
#define E8895_GICC_PMR		0x004
#define E8895_GICC_BPR		0x008
#define E8895_GICC_RPR		0x014
#define E8895_GICC_HIGHPRI	0x018
#define E8895_GICC_HPPIR		0x04c

/*
 * GICD arithmetic, taken from the GICv2 driver of this tree and nothing else:
 * GICD_ISENABLER holds one bit per INTID, 32 INTIDs per word, so SPI s (INTID
 * 32+s) is word (32+s)/32, bit (32+s)%32 -
 * A:drivers/irqchip/irq-gic.c:180-185 (gic_poke_irq: "offset + (hwirq / 32) * 4",
 * "1 << (hwirq % 32)"), with hwirq == INTID because the domain is created linear
 * over INTIDs (:1213 irq_domain_create_linear(handle, gic_irqs, ...) with
 * handle = 32), so irq_create_mapping() gives hwirq == INTID.
 * GICD_ITARGETSR is one word per four INTIDs counted from INTID 0, i.e. SPI s is
 * at 0x800 + 32 + s - A:drivers/irqchip/irq-gic.c:480-481 (gic_dist_init writes
 * GIC_DIST_TARGET + i * 4 / 4 for i = 32, 36, 40 ... with one cpumask, i.e. every
 * word from 0x820 upwards and none below it). Under that map the EINT carriers
 * (SPI 0-15 per bank, exynos8895-pinctrl.dtsi:37-91, and SPI 16 = the WKUP_MUX
 * demux, exynos8895.dtsi:2635) are the words 0x820..0x840, while 0x800..0x81c are
 * the SGI/PPI words, which a GICv2 distributor does not implement and which read
 * as zero. [the driver's arithmetic is what this file uses; the ARM manual is not
 * in this tree, so the dump prints the whole range offset by offset - where the
 * first non-zero word sits on hardware is the answer.]
 */
#define E8895_GICD_ENABLE_SPI(s)	(E8895_GICD_ISENABLER + ((32 + (s)) / 32) * 4)
#define E8895_GICD_ENABLE_BIT(s)	BIT((32 + (s)) % 32)
#define E8895_GICD_TARGET_SPI(s)	(E8895_GICD_ITARGETSR + 32 + (s))
/* ITARGETSR words 0x800..0x840 = 17 words: the eight SGI/PPI words plus the nine
 * that hold SPI 0..19 under the map above, i.e. every word that either reading of
 * the carrier map can mean */
#define E8895_GICD_TARGET_WORDS	17

#define E8895_EINT_ECON		0x700
#define E8895_EINT_EFLTCON	0x800
#define E8895_EINT_EMASK	0x900
#define E8895_EINT_EPEND	0xa00
#define E8895_EINT_EPEND_SET	0xb00
#define E8895_EINT_ENABLE	0xc00
#define E8895_EINT_LEVEL	0xd00

/* S:pinctrl-exynos.c:1785-1790 (etc1 included; our table has no etc1 bank,
 * pinctrl-exynos-arm64.c:1489-1495 - inert, no DT pin group uses it) */
static const struct {
	const char *name;
	u32 eint_offset;
	u8 nr_pins;
} e8895_alive_banks[] = {
	{ "etc1", 0x20, 6 }, { "gpa0", 0x00, 8 }, { "gpa1", 0x04, 8 },
	{ "gpa2", 0x08, 8 }, { "gpa3", 0x0c, 8 }, { "gpa4", 0x24, 7 },
};

static void __iomem *e8895_gicd_base, *e8895_gicc_base, *e8895_alive_base;
static u32 e8895_gicd_size, e8895_gicc_size, e8895_alive_size;
static unsigned int e8895_unmapped;

/* The mapped size of the block a base belongs to. All three come from the DT:
 * reg #0 and #1 of the GIC (exynos8895.dtsi:545-547, the two windows
 * drivers/irqchip/irq-gic.c:1397 /:1401 maps with of_iomap(node, 0/1)) and reg #0
 * of the pinctrl_alive node (exynos8895.dtsi:2632, mapped with
 * devm_ioremap(resource_size(&res)) below). */
static u32 e8895_map_size(void __iomem *base)
{
	if (base == e8895_gicc_base)
		return e8895_gicc_size;
	if (base == e8895_gicd_base)
		return e8895_gicd_size;
	return e8895_alive_size;
}

/* image 0142 showed that a read outside the mapped window is a translation fault
 * (0x96000007, level 3) and panics the box before the SMC, so an offset the size
 * does not cover is refused and counted here instead of being issued. ~0U is the
 * refused value; e8895_wake_dump() prints the counter, so a refused access can
 * never be silent. */
static u32 e8895_rd(void __iomem *base, u32 off)
{
	if (!base || off + 4 > e8895_map_size(base)) {
		e8895_unmapped++;
		return ~0U;
	}
	return readl(base + off);
}

/* the same check for the experiment writes below */
static void e8895_wr(void __iomem *base, u32 off, u32 val)
{
	if (!base || off + 4 > e8895_map_size(base)) {
		e8895_unmapped++;
		return;
	}
	writel(val, base + off);
}

/* GPIO_ALIVE is the first exynos8895-pinctrl node (FDT:88) and the GIC the only
 * interrupt-controller with a "arm,gic-*" compatible (exynos8895.dtsi:543);
 * the v2m/gic-400 compatible is listed first because that is what this DT has */
static void e8895_map_wake_blocks(struct device *dev)
{
	static const char * const gic_compat[] = {
		"arm,gic-400", "arm,gic-v3", "arm,gic-v4", "arm,cortex-a15-gic",
	};
	struct device_node *np;
	unsigned int i;

	if (e8895_alive_base)
		return;
	np = of_find_compatible_node(NULL, NULL, "samsung,exynos8895-pinctrl");
	if (np) {
		struct resource res;

		if (!of_address_to_resource(np, 0, &res)) {
			e8895_alive_base = devm_ioremap(dev, res.start,
							resource_size(&res));
			e8895_alive_size = resource_size(&res);
		}
		of_node_put(np);
	}
	for (i = 0; i < ARRAY_SIZE(gic_compat) && !e8895_gicd_base; i++) {
		struct resource res;

		np = of_find_compatible_node(NULL, NULL, gic_compat[i]);
		if (!np)
			continue;
		e8895_gicd_base = of_iomap(np, 0);
		e8895_gicc_base = of_iomap(np, 1);
		/* the sizes of those two of_iomap() calls used, read from the very
		 * same DT reg entries the irq-gic driver reads
		 * (drivers/irqchip/irq-gic.c:1397 /:1401). The GICC window is 0x1000
		 * (exynos8895.dtsi:546), i.e. GICC_DIR (0x1000) is NOT inside it -
		 * that is the image 0142 panic. */
		if (!of_address_to_resource(np, 0, &res))
			e8895_gicd_size = resource_size(&res);
		if (!of_address_to_resource(np, 1, &res))
			e8895_gicc_size = resource_size(&res);
		of_node_put(np);
		if (!e8895_gicd_base || !e8895_gicc_base) {
			dev_warn(dev, "DEBUG: GIC regmap %d unusable\n", (int)i);
			e8895_gicd_base = NULL;
			e8895_gicc_base = NULL;
			e8895_gicd_size = 0;
			e8895_gicc_size = 0;
		}
	}
	dev_info(dev, "DEBUG: wake blocks alive=%p/%x gicd=%p/%x gicc=%p/%x\n",
		 e8895_alive_base, e8895_alive_size,
		 e8895_gicd_base, e8895_gicd_size,
		 e8895_gicc_base, e8895_gicc_size);
}

/* one line per ALIVE bank: EINT_CON / INT_MASK / INT_PEND / INT_ENABLE /
 * INT_LEVEL / FLTCON, so a masked or un-armed wake pin is visible */
static void e8895_eint_dump(const char *tag)
{
	unsigned int i;

	if (!e8895_alive_base)
		return;
	for (i = 0; i < ARRAY_SIZE(e8895_alive_banks); i++) {
		const char *n = e8895_alive_banks[i].name;
		u32 o = e8895_alive_banks[i].eint_offset;

		pr_info("exynos8895-pm: DEBUG %s eint %s con=%08x flt=%08x mask=%08x pend=%08x en=%08x lvl=%08x\n",
			tag, n,
			e8895_rd(e8895_alive_base, E8895_EINT_ECON + o),
			e8895_rd(e8895_alive_base, E8895_EINT_EFLTCON + o),
			e8895_rd(e8895_alive_base, E8895_EINT_EMASK + o),
			e8895_rd(e8895_alive_base, E8895_EINT_EPEND + o),
			e8895_rd(e8895_alive_base, E8895_EINT_ENABLE + o),
			e8895_rd(e8895_alive_base, E8895_EINT_LEVEL + o));
	}
}

/* GICD/GICC: words 0-1 of IGROUPR/ISENABLER/ISPENDR cover INTID 0-63 = SGI 0-15,
 * PPI 0-15 and SPI 0-31, i.e. every EINT carrier; the ITARGETSR words are printed
 * offset by offset (see E8895_GICD_TARGET_WORDS) */
static void e8895_gic_dump(const char *tag)
{
	unsigned int b;

	if (!e8895_gicd_base || !e8895_gicc_base)
		return;
	pr_info("exynos8895-pm: DEBUG %s gicd ctlr=%08x typer=%08x ictr=%08x\n", tag,
		e8895_rd(e8895_gicd_base, E8895_GICD_CTLR),
		e8895_rd(e8895_gicd_base, E8895_GICD_TYPER),
		e8895_rd(e8895_gicd_base, E8895_GICD_ICTR));
	pr_info("exynos8895-pm: DEBUG %s gicd igrp=%08x %08x en=%08x %08x pend=%08x %08x\n", tag,
		e8895_rd(e8895_gicd_base, E8895_GICD_IGROUPR),
		e8895_rd(e8895_gicd_base, E8895_GICD_IGROUPR + 4),
		e8895_rd(e8895_gicd_base, E8895_GICD_ISENABLER),
		e8895_rd(e8895_gicd_base, E8895_GICD_ISENABLER + 4),
		e8895_rd(e8895_gicd_base, E8895_GICD_ISPENDR),
		e8895_rd(e8895_gicd_base, E8895_GICD_ISPENDR + 4));
	for (b = 0; b < E8895_GICD_TARGET_WORDS; b++) {
		if (b && !(b % 6))
			pr_info("exynos8895-pm: DEBUG %s gicd target\n", tag);
		pr_info(" %x:%08x", E8895_GICD_ITARGETSR + b * 4,
			e8895_rd(e8895_gicd_base, E8895_GICD_ITARGETSR + b * 4));
	}
	pr_info("\n");
	pr_info("exynos8895-pm: DEBUG %s gicc ctlr=%08x pmr=%08x bpr=%08x rpr=%08x hppir=%08x\n",
		tag,
		e8895_rd(e8895_gicc_base, E8895_GICC_CTLR),
		e8895_rd(e8895_gicc_base, E8895_GICC_PMR),
		e8895_rd(e8895_gicc_base, E8895_GICC_BPR),
		e8895_rd(e8895_gicc_base, E8895_GICC_RPR),
		e8895_rd(e8895_gicc_base, E8895_GICC_HPPIR));
}

/* the EINT wake mask bits that are CLEAR are the wake-enabled pins; print them
 * by name, so a wrong bit convention (bank offset, 32-bit truncation) shows up */
static void e8895_wake_mask_dump(u32 mask, const char *tag)
{
	char buf[192];
	unsigned int b, p, n = 0;
	bool any = false;

	buf[0] = 0;
	for (b = 0; b < ARRAY_SIZE(e8895_alive_banks); b++) {
		for (p = 0; p < e8895_alive_banks[b].nr_pins; p++) {
			unsigned int bit = 2 * e8895_alive_banks[b].eint_offset + p;

			if (bit >= 32 || (mask & BIT(bit)))
				continue;
			n += scnprintf(buf + n, sizeof(buf) - n, "%s%s-%u",
					n ? " " : "", e8895_alive_banks[b].name, p);
			any = true;
			if (n > 150)
				goto out;
		}
	}
out:
	pr_info("exynos8895-pm: DEBUG %s eint_wake=%08x armed=%s%s\n", tag, mask,
		any ? buf : "-", any ? "" : " (none)");
}

static void e8895_wake_dump(const char *tag)
{
	e8895_gic_dump(tag);
	e8895_eint_dump(tag);
	e8895_wake_mask_dump(pmu_raw_readl(E8895_EINT_WAKEUP_MASK), tag);
	/* the words stock's exynos_show_wakeup_reason() prints after the wake
	 * (S:drivers/soc/samsung/exynos-pm.c:119-134), plus 0x604/0x608 which
	 * nothing dumped before: on 8895 0x600-0x608 are WAKEUP_STAT/2/3
	 * (S:drivers/soc/samsung/pwrcal/S5E8890/S5E8890-pmusfr.h:131-133) and
	 * WAKEUP_STAT2 is also the "wrong" EXYNOS_EINT_WAKEUP_MASK 0x0604 that the
	 * exynos4210 wkup chip would use (A:...exynos-regs-pmu.h:47,
	 * A:...pinctrl-exynos.c:630) - this DT selects exynos7_wkup_irq_chip
	 * instead, whose mask register is EXYNOS5433_EINT_WAKEUP_MASK = 0x060c
	 * (A:...pinctrl-exynos.c:650), i.e. the same word this driver writes at
	 * :1021 and the same one stock writes at
	 * S:...exynos-powermode.c:624-631 */
	pr_info("exynos8895-pm: DEBUG %s pmustat=%08x %08x %08x wkmask=%08x %08x %08x %08x wkmif=%08x\n",
		tag, pmu_raw_readl(E8895_WAKEUP_STAT), pmu_raw_readl(0x0604),
		pmu_raw_readl(0x0608), pmu_raw_readl(0x0610), pmu_raw_readl(0x0614),
		pmu_raw_readl(0x0618), pmu_raw_readl(0x061c), pmu_raw_readl(0x0628));
	/* anything refused above prints as ffffffff; the count makes it visible */
	pr_info("exynos8895-pm: DEBUG %s maps alive=%x gicd=%x gicc=%x unmapped=%u\n",
		tag, e8895_alive_size, e8895_gicd_size, e8895_gicc_size,
		e8895_unmapped);
}

/*
 * DEBUG E1 (l7re H1: EL3's SLEEP gates read PMU words that are in no kernel
 * PMUCAL list and were never dumped, so a divergence is invisible to the
 * 145-entry verify): print those words right before the SLEEP SMC, next to
 * the existing DEBUG pre-SMC lines, and on demand at late boot through
 * sleep_dump (runtime-vs-pre-SMC comparison). All reads use pmu_raw_readl(),
 * the driver's existing PMU accessor. Default off, no behaviour change.
 */
static void e8895_sleep_regs(const char *tag)
{
	/*
	 * EL3 SLEEP gate inputs (l7re REPORT section 4): 0x400 bit16
	 * (SB:0xab838), 0x404 == 0x10000000 with bits 24/23/20 clear
	 * (SB:0xab6a0, SB:0xab02c, SB:0xab838), 0x420 bit4 selects the SLEEP
	 * variant (SB:0x75a80), 0x500 (0xF01 on enter, bit0 required;
	 * SB:0x75390, SB:0x75a08), 0x808 == 0xAC9E (SB:0xab02c),
	 * 0x2604 & 7 == 7 (SB:0x76628).
	 */
	pr_info("exynos8895-pm: DEBUG %s el3=%08x %08x %08x %08x %08x %08x\n", tag,
		pmu_raw_readl(0x0400), pmu_raw_readl(0x0404), pmu_raw_readl(0x0420),
		pmu_raw_readl(0x0500), pmu_raw_readl(0x0808), pmu_raw_readl(0x2604));
	/*
	 * Sequencer words EL3 programs itself: [0x2008] = 0x10A with
	 * [0x2000] &= ~0xF (same pairs 0x2080/0x2088, 0x2100/0x2108,
	 * 0x2180/0x2188) at SB:0x74cd4; 0x200C/0x208C/0x210C/0x218C |= 0x1300,
	 * [0x244C] |= 0x2040000, [0x240C] |= 0x80010000 at SB:0x751c8.
	 */
	pr_info("exynos8895-pm: DEBUG %s seq0=%08x %08x %08x %08x %08x %08x %08x %08x\n",
		tag, pmu_raw_readl(0x2000), pmu_raw_readl(0x2004),
		pmu_raw_readl(0x2008), pmu_raw_readl(0x200C),
		pmu_raw_readl(0x2080), pmu_raw_readl(0x2084),
		pmu_raw_readl(0x2088), pmu_raw_readl(0x208C));
	pr_info("exynos8895-pm: DEBUG %s seq1=%08x %08x %08x %08x %08x %08x %08x %08x %08x %08x\n",
		tag, pmu_raw_readl(0x2100), pmu_raw_readl(0x2108), pmu_raw_readl(0x210C),
		pmu_raw_readl(0x2180), pmu_raw_readl(0x2188), pmu_raw_readl(0x218C),
		pmu_raw_readl(0x2400), pmu_raw_readl(0x240C),
		pmu_raw_readl(0x2440), pmu_raw_readl(0x244C));
	/*
	 * Per-CPU CONFIGURATION/STATUS of every cluster CPU (l7re H6: an
	 * offline CPU must read 0/0): 0x2000 + pc * 0x80, STATUS + 4
	 * (CAL:pmucal_cpu_exynos8895.h:3,11 for pc 0, same shape pc 1-7:
	 * 0x2080/0x2084 ... 0x2380/0x2384).
	 */
	pr_info("exynos8895-pm: DEBUG %s cpu0=%x/%x %x/%x %x/%x %x/%x\n", tag,
		pmu_raw_readl(0x2000) & 0xf, pmu_raw_readl(0x2004) & 0xf,
		pmu_raw_readl(0x2080) & 0xf, pmu_raw_readl(0x2084) & 0xf,
		pmu_raw_readl(0x2100) & 0xf, pmu_raw_readl(0x2104) & 0xf,
		pmu_raw_readl(0x2180) & 0xf, pmu_raw_readl(0x2184) & 0xf);
	pr_info("exynos8895-pm: DEBUG %s cpu1=%x/%x %x/%x %x/%x %x/%x\n", tag,
		pmu_raw_readl(0x2200) & 0xf, pmu_raw_readl(0x2204) & 0xf,
		pmu_raw_readl(0x2280) & 0xf, pmu_raw_readl(0x2284) & 0xf,
		pmu_raw_readl(0x2300) & 0xf, pmu_raw_readl(0x2304) & 0xf,
		pmu_raw_readl(0x2380) & 0xf, pmu_raw_readl(0x2384) & 0xf);
	/* WAKEUP_STAT, EINT mask, WAKEUP_MASK (S:exynos-pm.c:52-53, FDT:4445) */
	pr_info("exynos8895-pm: DEBUG %s wake=%08x %08x %08x\n", tag,
		pmu_raw_readl(E8895_WAKEUP_STAT), pmu_raw_readl(E8895_EINT_WAKEUP_MASK),
		pmu_raw_readl(0x0610));
}

/*
 * DEBUG E1 boot half: echo 1 > /sys/module/exynos8895_pm/parameters/sleep_dump
 * prints the same set at late boot / run time, for runtime-vs-pre-SMC
 * comparison without suspending.
 */
static int e8895_sleep_dump_set(const char *val, const struct kernel_param *kp)
{
	u32 v;
	int ret = kstrtou32(val, 0, &v);

	if (ret)
		return ret;
	if (v) {
		if (!pmu_base_addr)
			return -ENODEV;
		e8895_sleep_regs("boot");
	}
	return 0;
}

static const struct kernel_param_ops e8895_sleep_dump_ops = { .set = e8895_sleep_dump_set };
module_param_cb(sleep_dump, &e8895_sleep_dump_ops, NULL, 0200);

/* DEBUG (l7wake): same trigger idiom as sleep_dump, for the GIC/EINT dump */
static int e8895_wake_dump_set(const char *val, const struct kernel_param *kp)
{
	u32 v;
	int ret = kstrtou32(val, 0, &v);

	if (ret)
		return ret;
	if (v) {
		if (!pmu_base_addr)
			return -ENODEV;
		e8895_wake_dump("boot");
	}
	return 0;
}

static const struct kernel_param_ops e8895_wake_dump_ops = { .set = e8895_wake_dump_set };
module_param_cb(wake_dump, &e8895_wake_dump_ops, NULL, 0200);
/*
 * DEBUG (l7wake) sleep_wakearm=<mask>: write the stock suspend-time hardware
 * state of the two hops the kernel only reads today, then print the readback.
 *
 * Stock leaves every GIC SPI that the generic irq core does not touch hardware
 * -enabled at suspend (S:kernel/irq/pm.c:88-98 masks only when the chip sets
 * IRQCHIP_MASK_ON_SUSPEND; our chip does, S:drivers/irqchip/irq-gic.c:431-432,
 * but the wake IRQs and the chained EINT carriers are skipped: :73-75) - so
 * nothing forces them back. Bit 1 of the mask does, exactly that, for SPI 0-31
 * (the EINT carriers: exynos8895-pinctrl.dtsi:37-91 for SPI 0-15, .dtsi:2635 for
 * SPI 16), and pins every ITARGETSR word the dump prints to CPU0.
 *
 * Bit 2 re-asserts INT_ENABLE in the ALIVE EINT block for every pin whose
 * EINT_WAKEUP_MASK (PMU 0x60C) bit is clear, i.e. exactly the pins that
 * enable_irq_wake() armed (S:pinctrl-exynos.c:418-431 clears the bit). If the
 * EINT block masked one of them at some point in suspend, this puts it back.
 *
 * Bit 4 forces GICC_PMR/BPR to "no priority mask" (0xff/0xff, the values
 * gic_cpu_if_up() leaves behind) in case the CPU interface is masked.
 *
 * Default 0: no write at all. Everything is printed before and after, so a
 * refused write is visible.
 */
#define E8895_WAKEARM_GIC	1
#define E8895_WAKEARM_EINT	2
#define E8895_WAKEARM_PMR	4
static u32 sleep_wakearm;
module_param(sleep_wakearm, uint, 0644);

/*
 * DEBUG (l7wake) sleep_bare=1: skip the whole PMU SLEEP program and issue only
 * the PSCI call (see the use in e8895_pm_syscore_suspend()). sleep_bare is used
 * together with sleep_psci=0x10000 so that the SMC asks for C2 (core only).
 * sleep_swpend=<mask>: bits of the ALIVE banks' pins (see e8895_alive_banks[]):
 * set INT_PEND_SET for that pin right before the SMC, i.e. raise an EINT from
 * software (EXYNOS_GPIO_EPEND + 0x100, the next register after EPEND), and route
 * it to CPU0. This asks the only question that matters for "does any interrupt
 * reach CPU0 in this context" without any external hardware: EINT -> SPI 16 ->
 * GIC -> CPU0. Bit 30 = gpa3-6 (the PMIC, what stock woke from), bit 20 =
 * gpa2-4 (the power key). A wake means the whole chain is alive and the PMU SLEEP
 * program is at fault; a hang means the chain is dead at the GIC or below.
 */
static bool sleep_bare;
module_param(sleep_bare, bool, 0644);
static u32 sleep_swpend;
module_param(sleep_swpend, uint, 0644);

/* DEBUG (l7el3) sleep_c2prep=1: in the sleep_bare path, run the per-CPU PMU
 * preparation the cpuidle C2 state runs right before its SMC -
 * e8895_pmu_cpu_down() clears DD0_DD_DA_INPUT_0 bit 0 and
 * CLUSTER0_CPU0_CONFIGURATION[3:0] (CAL:pmucal_cpu_exynos8895.h:5-9, what
 * stock's cpu_disable() plays in exynos_cpu_pm_enter(),
 * S:drivers/soc/samsung/exynos-powermode.c:513-517,80-84) - and undo it after
 * the SMC returns. It is the only PMU write the two contexts do not share, so
 * if the EL3 CPU_SUSPEND handler reads the core's CONFIGURATION/STATUS this is
 * the switch that decides whether the bare suspend returns.
 * sleep_bare_eint=1: put PMU EINT_WAKEUP_MASK (0x60c) back to the value it has
 * at run time. The pinctrl syscore op writes the EINT wake mask in suspend but
 * never in cpuidle, so the bare path is the one context where 0x60c differs
 * from a C2 that works. */
static bool sleep_c2prep;
module_param(sleep_c2prep, bool, 0644);
static bool sleep_bare_eint;
module_param(sleep_bare_eint, bool, 0644);
static u32 eint_wake_boot;			/* 0x60c at probe (== the cpuidle value) */

/* power_state of the SMC: sleep_bare forces C2 (0x00010000), otherwise the
 * stock SLEEP word unless sleep_psci overrides it */
static unsigned int e8895_smc_state(void)
{
	if (sleep_bare)
		return 0x00010000;
	return sleep_psci ?: E8895_PSCI_SLEEP;
}

static void e8895_swpend(void)
{
	unsigned int i, p;

	if (!sleep_swpend || !e8895_alive_base || !e8895_gicd_base)
		return;
	/* route the EINT carriers to CPU0 and make sure SPI 16 is enabled. The
	 * whole ITARGETSR range the dump prints is written, so the carriers are
	 * covered whether they sit at 0x800 + SPI or at 0x800 + 32 + SPI. */
	for (i = 0; i < E8895_GICD_TARGET_WORDS; i++)
		e8895_wr(e8895_gicd_base, E8895_GICD_ITARGETSR + i * 4, 0x11111111);
	/* SPI 16 = INTID 48 = ISENABLER word 1 bit 16 (A:irq-gic.c:180-185).
	 * Writing ISENABLER + 0 would enable INTID 16, a PPI, not the carrier. */
	e8895_wr(e8895_gicd_base, E8895_GICD_ENABLE_SPI(16), E8895_GICD_ENABLE_BIT(16));
	for (i = 0; i < ARRAY_SIZE(e8895_alive_banks); i++) {
		u32 o = e8895_alive_banks[i].eint_offset;
		u32 pend = 0;

		for (p = 0; p < e8895_alive_banks[i].nr_pins; p++)
			if (sleep_swpend & BIT(2 * e8895_alive_banks[i].eint_offset + p))
				pend |= BIT(p);
		if (pend)
			e8895_wr(e8895_alive_base, E8895_EINT_EPEND_SET + o, pend);
		pr_info("exynos8895-pm: DEBUG swpend %s pend=%08x en=%08x mask=%08x\n",
			e8895_alive_banks[i].name, pend,
			e8895_rd(e8895_alive_base, E8895_EINT_ENABLE + o),
			e8895_rd(e8895_alive_base, E8895_EINT_EMASK + o));
	}
	pr_info("exynos8895-pm: DEBUG swpend: gicd en=%08x %08x\n",
		e8895_rd(e8895_gicd_base, E8895_GICD_ISENABLER),
		e8895_rd(e8895_gicd_base, E8895_GICD_ISENABLER + 4));
}

static void e8895_wakearm(void)
{
	u32 mask, en, i;

	if (!sleep_wakearm)
		return;
	mask = exynos_pinctrl_get_eint_wake_mask();
	pr_info("exynos8895-pm: DEBUG wakearm=%x eint_wake=%08x gicd=%p alive=%p\n",
		sleep_wakearm, mask, e8895_gicd_base, e8895_alive_base);
	if ((sleep_wakearm & E8895_WAKEARM_EINT) && e8895_alive_base) {
		for (i = 0; i < ARRAY_SIZE(e8895_alive_banks); i++) {
			u32 o = e8895_alive_banks[i].eint_offset;
			u32 bits = 0;
			unsigned int p;

			for (p = 0; p < e8895_alive_banks[i].nr_pins; p++) {
				unsigned int bit = 2 * e8895_alive_banks[i].eint_offset + p;

				if (bit < 32 && !(mask & BIT(bit)))
					bits |= BIT(p);
			}
			en = e8895_rd(e8895_alive_base, E8895_EINT_ENABLE + o);
			if (en != bits)
				e8895_wr(e8895_alive_base, E8895_EINT_ENABLE + o, bits);
			pr_info("exynos8895-pm: DEBUG wakearm eint %s en=%08x -> %08x\n",
				e8895_alive_banks[i].name, en,
				e8895_rd(e8895_alive_base, E8895_EINT_ENABLE + o));
		}
	}
	if ((sleep_wakearm & (E8895_WAKEARM_GIC | E8895_WAKEARM_PMR)) && e8895_gicd_base) {
		if (sleep_wakearm & E8895_WAKEARM_GIC) {
			for (i = 0; i < 2; i++) {
				u32 v = e8895_rd(e8895_gicd_base, E8895_GICD_ISENABLER + i * 4);

				e8895_wr(e8895_gicd_base, E8895_GICD_ISENABLER + i * 4, 0xffffffff);
				pr_info("exynos8895-pm: DEBUG wakearm gicd en[%d]=%08x -> %08x\n",
					(int)i, v,
					e8895_rd(e8895_gicd_base, E8895_GICD_ISENABLER + i * 4));
			}
			/* ITARGETSR: 4 bits per INTID, one word per four INTIDs from
			 * INTID 0, so SPI s is at E8895_GICD_TARGET_SPI(s) and the
			 * carriers (SPI 0-16) are the words 0x820..0x840. The whole
			 * range the dump prints is written, which covers the other
			 * possible reading of the map as well. */
			for (i = 0; i < E8895_GICD_TARGET_WORDS; i++)
				e8895_wr(e8895_gicd_base, E8895_GICD_ITARGETSR + i * 4,
					 0x11111111);
			pr_info("exynos8895-pm: DEBUG wakearm gicd target words:");
			for (i = 0; i < E8895_GICD_TARGET_WORDS; i++)
				pr_info(" %x=%08x", E8895_GICD_ITARGETSR + i * 4,
					e8895_rd(e8895_gicd_base, E8895_GICD_ITARGETSR + i * 4));
			pr_info("\n");
		}
		if ((sleep_wakearm & E8895_WAKEARM_PMR) && e8895_gicc_base) {
			u32 pmr = e8895_rd(e8895_gicc_base, E8895_GICC_PMR);
			u32 bpr = e8895_rd(e8895_gicc_base, E8895_GICC_BPR);

			e8895_wr(e8895_gicc_base, E8895_GICC_PMR, 0xff);
			e8895_wr(e8895_gicc_base, E8895_GICC_BPR, 0xff);
			pr_info("exynos8895-pm: DEBUG wakearm gicc pmr=%08x -> %08x bpr=%08x -> %08x\n",
				pmr, e8895_rd(e8895_gicc_base, E8895_GICC_PMR),
				bpr, e8895_rd(e8895_gicc_base, E8895_GICC_BPR));
		}
	}
}



static void e8895_sleep_dump(void)
{
	pr_info("exynos8895-pm: DEBUG pre-SMC mode=%d wk=%08x %08x %08x %08x eint=%08x cp=%08x abox=%x vts=%x\n",
		e8895_mode, pmu_raw_readl(0x610), pmu_raw_readl(0x614), pmu_raw_readl(0x618),
		pmu_raw_readl(0x644), pmu_raw_readl(E8895_EINT_WAKEUP_MASK), pmu_raw_readl(E8895_CP_STAT),
		pmu_raw_readl(E8895_PD_ABOX) & 0xf, pmu_raw_readl(E8895_PD_VTS) & 0xf);
	/* CENTRAL_SEQ_CONFIGURATION/_OPTION, CENTRAL_SEQ_MIF_CONFIGURATION (CAL:pmucal_system_exynos8895.h
	 * exit_sleep 0x0200/0x0240, enter 0x0208), IDLE_IP3, CLUSTER1_CPU0 (cpu0) CONFIGURATION/STATUS */
	pr_info("exynos8895-pm: DEBUG pre-SMC seq=%08x opt=%08x mif=%08x ipi3=%08x cpu0=%x/%x\n",
		pmu_raw_readl(0x0200), pmu_raw_readl(0x0208), pmu_raw_readl(0x0240),
		pmu_raw_readl(E8895_IDLE_IP(3)), pmu_raw_readl(0x2200) & 0xf, pmu_raw_readl(0x2204) & 0xf);
	e8895_sleep_regs("pre-SMC");			/* DEBUG E1 */
	e8895_pd_dump();
	e8895_wake_dump("pre-SMC");			/* DEBUG l7wake */
}

/* DEBUG sleep_pdoff: bounded like e8895_wait() (pmucal_rae_wait(), 1000 x 1 us) */
static int e8895_dbg_wait(void __iomem *a, u32 mask, u32 val)
{
	unsigned int t;

	for (t = 0; (readl_relaxed(a) & mask) != val; t++) {
		if (t > 1000)
			return -ETIMEDOUT;
		udelay(1);
	}
	return 0;
}

#define E8895_VA_VTS	4	/* e8895_pm_bases[4] = 0x14000000, CMU_VTS */

/* stock abox_off (CAL:pmucal_local_exynos8895.h:61-65) and vts_off (:636-641) */
static int e8895_dbg_pdoff(void)
{
	void __iomem *pmu = pmu_base_addr, *vts = e8895_va[E8895_VA_VTS];
	int ra = 0, rv = 0;

	if (WARN_ON(e8895_pm_bases[E8895_VA_VTS] != 0x14000000))
		return -EINVAL;
	if (readl_relaxed(pmu + 0x4004) & 0xf) {
		writel_relaxed(readl_relaxed(pmu + 0x2528) | BIT(16), pmu + 0x2528);	/* ABOX_CA7_OPTION */
		writel_relaxed(readl_relaxed(pmu + 0x4000) & ~0xf, pmu + 0x4000);	/* ABOX_CONFIGURATION */
		e8895_alloff_done |= BIT(0);		/* e8895_pd_stat[0] abox: on again in allon */
		ra = e8895_dbg_wait(pmu + 0x4004, 0xf, 0);				/* ABOX_STATUS */
	}
	if (readl_relaxed(pmu + 0x41a4) & 0xf) {
		writel_relaxed(readl_relaxed(vts + 0x1000) & ~BIT(0), vts + 0x1000);	/* MUX_CLK_VTS_CMUREF */
		rv = e8895_dbg_wait(vts + 0x1000, BIT(16), 0);
		if (!rv) {
			writel_relaxed(readl_relaxed(pmu + 0x41a0) & ~0xf, pmu + 0x41a0);	/* VTS_CONFIGURATION */
			e8895_alloff_done |= BIT(15);	/* e8895_pd_stat[15] vts */
			rv = e8895_dbg_wait(pmu + 0x41a4, 0xf, 0);			/* VTS_STATUS */
		}
	}
	pr_info("exynos8895-pm: DEBUG pdoff abox=%d (status %x) vts=%d (status %x)\n",
		ra, readl_relaxed(pmu + 0x4004) & 0xf, rv, readl_relaxed(pmu + 0x41a4) & 0xf);
	return ra ? : rv;
}

/*
 * DEBUG sleep_alloff=<mask> (image 0121: every domain consistent, abox dbg dpu0 dpu1 g2d iva
 * srdz vts cam/isp on at the SMC, still no return even with a C2 power_state): power the
 * selected domains off in syscore suspend, as stock has them before SLEEP, and on again in
 * syscore resume (or when the suspend aborts). Bit i = e8895_pd_stat[i]: 0 abox, 1 cam,
 * 2 dbg, 3 dcam, 4 dpu0, 5 dpu1, 6 dsp, 7 g2d, 8 g3d, 9 isphq, 10 isplp, 11 iva, 12 mfc,
 * 13 srdz, 14 vpu, 15 vts; 0xfeff = all but g3d. abox and vts get their stock pre-off step
 * (CAL:pmucal_local_exynos8895.h:62, :637-638). A domain that does not reach STATUS 0
 * within 1 ms is requested on again and the suspend aborts (-EIO). The domain contents
 * (ABOX firmware, SYSREG DRCG_EN) are lost; the CMUs restore themselves on resume.
 */
static unsigned int sleep_alloff;
module_param(sleep_alloff, uint, 0644);

static void e8895_dbg_allon(void)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(e8895_pd_stat); i++) {
		u16 st = e8895_pd_stat[i].off;

		if (!(e8895_alloff_done & BIT(i)))
			continue;
		pmu_raw_writel(pmu_raw_readl(st - 4) | 0xf, st - 4);
		if (e8895_dbg_wait(pmu_base_addr + st, 0xf, 0xf))
			pr_err("exynos8895-pm: DEBUG alloff: %s did not come back on (STATUS %x)\n",
			       e8895_pd_stat[i].name, pmu_raw_readl(st) & 0xf);
	}
	if (e8895_alloff_done)
		pr_info("exynos8895-pm: DEBUG alloff: on again %04x\n", e8895_alloff_done);
	e8895_alloff_done = 0;
}

static int e8895_dbg_alloff(void)
{
	void __iomem *vts = e8895_va[E8895_VA_VTS];
	unsigned int i;
	int ret = 0;

	/*
	 * Never dbg (bit 2) or g3d (bit 8): stock never power-managed pd-dbg at run
	 * time (FDT:3889-3895 status = "disabled") and enters SLEEP with it on; the
	 * raw cycle here skips CMU_DBG's PMUCAL restore (CAL:pmucal_local_exynos8895.h:128-156), and image
	 * 0126 died in a C2 idle hard lockup after ten raw dbg cycles. g3d has its own
	 * PLL steps in the PD driver.
	 */
	u32 mask = sleep_alloff & ~(BIT(2) | BIT(8));

	for (i = 0; i < ARRAY_SIZE(e8895_pd_stat); i++) {
		u16 st = e8895_pd_stat[i].off;

		if (!(mask & BIT(i)) || !(pmu_raw_readl(st) & 0xf))
			continue;
		if (st == E8895_PD_ABOX)
			pmu_raw_writel(pmu_raw_readl(0x2528) | BIT(16), 0x2528);	/* ABOX_CA7_OPTION */
		if (st == E8895_PD_VTS) {
			writel_relaxed(readl_relaxed(vts + 0x1000) & ~BIT(0), vts + 0x1000);
			e8895_dbg_wait(vts + 0x1000, BIT(16), 0);
		}
		pmu_raw_writel(pmu_raw_readl(st - 4) & ~0xf, st - 4);
		e8895_alloff_done |= BIT(i);
		if (e8895_dbg_wait(pmu_base_addr + st, 0xf, 0)) {
			pr_err("exynos8895-pm: DEBUG alloff: %s stays on (STATUS %x)\n",
			       e8895_pd_stat[i].name, pmu_raw_readl(st) & 0xf);
			ret = -EIO;
			break;
		}
	}
	pr_info("exynos8895-pm: DEBUG alloff: off %04x (mask %04x): %d\n",
		e8895_alloff_done, mask, ret);
	if (ret)
		e8895_dbg_allon();
	return ret;
}

/*
 * Deviation from stock, which entered SLEEP regardless: a domain whose
 * power-off timed out with CONFIGURATION still 0 (STATUS on) stalls the system
 * power-down with no wake (image 0120: dpu1/g2d/vts/srdz CONFIGURATION 0 STATUS
 * 0xf, no return from the SLEEP SMC). Refuse SLEEP while any PMU domain's
 * CONFIGURATION[3:0] differs from its STATUS[3:0].
 */
static int e8895_pd_check(void)
{
	unsigned int i;
	int ret = 0;

	for (i = 0; i < ARRAY_SIZE(e8895_pd_stat); i++) {
		u16 st = e8895_pd_stat[i].off;		/* CONFIGURATION = STATUS - 4 */
		u32 c = pmu_raw_readl(st - 4) & 0xf, v = pmu_raw_readl(st) & 0xf;

		if (c != v) {
			pr_err("exynos8895-pm: pd %s CONFIGURATION 0x%x STATUS 0x%x: no SLEEP\n",
			       e8895_pd_stat[i].name, c, v);
			ret = -EBUSY;
		}
	}
	return ret;
}

/* S:drivers/soc/samsung/exynos-pm.c:231-255 */
static int e8895_pm_syscore_suspend(void)
{
	u32 ipm[4], wk[4], eint;
	struct e8895_tabs *t;
	const char *why;
	unsigned int i;
	u32 arm0, arm1;		/* DEBUG sleep_noarm: pre-enter CENTRAL bits */
	int ret;

	e8895_sleep_armed = false;
	/* hibernation's create_image() runs syscore_suspend()/syscore_resume() too
	 * (A:kernel/power/hibernate.c:346,373); only a mem suspend sets this state
	 * (A:kernel/power/suspend.c:525, reset at :562). s2idle runs no syscore ops. */
	if (pm_suspend_target_state != PM_SUSPEND_MEM)
		return 0;
	e8895_sp_probe("in", "e8895_pm_syscore_suspend");	/* DEBUG stage_poke */
	WRITE_ONCE(e8895_sp_active, false);

	/*
	 * Every refusal below leaves the PMU as it was: a failing syscore
	 * suspend op gets no syscore_resume() call (syscore_suspend()
	 * resumes only the ops that already succeeded), so nothing armed
	 * here may survive an error return (A:drivers/base/syscore.c:66-82). A
	 * refusal is logged at pr_err.
	 */
	if (cp_check && !pmu_raw_readl(E8895_CP_STAT)) {	/* :233-237 */
		why = "CP reset";
		ret = -EINVAL;
		goto refuse;
	}
	if (e8895_pd_check()) {
		why = "power domain half-way";
		ret = -EBUSY;
		goto refuse;
	}
	if (sleep_pdoff && e8895_dbg_pdoff()) {			/* DEBUG */
		why = "sleep_pdoff";
		ret = -EIO;
		goto refuse;
	}
	if (sleep_alloff && e8895_dbg_alloff()) {		/* DEBUG */
		why = "sleep_alloff";
		ret = -EIO;
		goto refuse;
	}
	if (E8895_PD_ON(E8895_PD_ABOX))			/* abox_is_on(), :239 */
		e8895_mode = E8895_SYS_SLEEP_AUD_ON;	/* FDT:4419 cp_call_mode_idx = <0x0a> */
	else if (E8895_PD_ON(E8895_PD_VTS))		/* vts_is_on(), :245 */
		e8895_mode = E8895_SYS_SLEEP_VTS_ON;	/* S:drivers/soc/samsung/exynos-pm.c:227 SLEEP_VTS_ON 9 */
	else
		e8895_mode = E8895_SYS_SLEEP;		/* FDT:4417 suspend_mode_idx = <0x08> */
	if (sleep_mode >= E8895_SYS_SLEEP && sleep_mode <= E8895_SYS_SLEEP_AUD_ON)	/* DEBUG */
		e8895_mode = sleep_mode;

	/* kept for a refusal; a completed cycle leaves the SLEEP values, as stock */
	for (i = 0; i < 4; i++) {
		ipm[i] = pmu_raw_readl(E8895_IDLE_IP_MASK(i));
		wk[i] = pmu_raw_readl(e8895_wkmask_off[i]);
	}
	eint = pmu_raw_readl(E8895_EINT_WAKEUP_MASK);

	/*
	 * DEBUG (l7wake) sleep_bare=1: run nothing but the PSCI call, i.e. skip
	 * the IDLE_IP masks, the EINT/WAKEUP_MASK writes, the PMUCAL save and the
	 * whole enter list, and go straight from the mode choice to the SMC with
	 * power_state 0x00010000 (C2). The cluster stays powered and the PMU is
	 * in its run-time state, so this is the same core-suspend the cpuidle C2
	 * state does millions of times, only with the CPUs 1-7 offline, the
	 * clockevents stopped and the non-wake IRQs suspended.
	 *   wakes  => the PMU SLEEP programming (enter list, sequencer, wakeup
	 *             masks) is what costs the wake-up; C2 + the suspend context
	 *             are fine.
	 *   hangs  => even a bare C2 does not wake from this context, so the
	 *             break is in the irq/CPU state that suspend adds, not in the
	 *             PMU program.
	 * The refusal and resume paths below are skipped as well (nothing was
	 * programmed, nothing to unwind), so e8895_sleep_armed stays false.
	 */
	if (sleep_bare) {
		e8895_mode = E8895_SYS_SLEEP;		/* label only: t[] is unused below */
		pr_info("exynos8895-pm: DEBUG bare: no PMU program, PSCI 0x%08x\n",
			e8895_smc_state());
		if (sleep_bare_eint && pmu_raw_readl(E8895_EINT_WAKEUP_MASK) != eint_wake_boot) {
			pmu_raw_writel(eint_wake_boot, E8895_EINT_WAKEUP_MASK);	/* DEBUG l7el3 */
			pr_info("exynos8895-pm: DEBUG bare eint: 0x60c back to the boot value 0x%08x\n",
				eint_wake_boot);
		}
		e8895_wakearm();				/* still available */
		e8895_swpend();					/* still available */
		if (sleep_trace)
			e8895_sleep_dump();
		e8895_early = true;
		return 0;
	}

	/* only SYS_SICD has a ref-idle-ip node (FDT:4433-4439): the SLEEP masks are all-ones,
	 * S:drivers/soc/samsung/exynos-powermode.c:171-182 */
	for (i = 0; i < 4; i++)
		pmu_raw_writel(~0u, E8895_IDLE_IP_MASK(i));
	pmu_raw_writel(exynos_pinctrl_get_eint_wake_mask(), E8895_EINT_WAKEUP_MASK);	/* :625-631 */
	for (i = 0; i < 4; i++)
		pmu_raw_writel(e8895_wkmask_sleep[i], e8895_wkmask_off[i]);		/* :633-635 */
	if (sleep_wkmask0 >= 0)								/* DEBUG */
		pmu_raw_writel(sleep_wkmask0, e8895_wkmask_off[0]);
	t = e8895_tab(e8895_mode);
	e8895_save(t->t[1], t->n[1]);			/* S:drivers/soc/samsung/cal-if/pmucal_system.c:31 */
	arm0 = pmu_raw_readl(0x0200);			/* DEBUG sleep_noarm */
	arm1 = pmu_raw_readl(0x0240);
	if (e8895_handle(t->t[0], t->n[0])) {		/* :33-39; only the VTS_ON list has a WAIT */
		/* deviation: stock drops this error and enters SLEEP anyway
		 * (S:drivers/soc/samsung/exynos-pm.c:241-248); here the half-programmed
		 * PMU is rolled back and the suspend aborted */
		e8895_handle(t->t[3], t->n[3]);
		e8895_restore(t->t[1], t->n[1]);
		for (i = 0; i < 4; i++) {
			pmu_raw_writel(ipm[i], E8895_IDLE_IP_MASK(i));
			pmu_raw_writel(wk[i], e8895_wkmask_off[i]);
		}
		pmu_raw_writel(eint, E8895_EINT_WAKEUP_MASK);
		why = "enter list wait timed out";
		ret = -EIO;
		goto refuse;
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
	if (sleep_noarm) {					/* DEBUG E6 */
		pmu_raw_writel((pmu_raw_readl(0x0200) & ~BIT(16)) | (arm0 & BIT(16)), 0x0200);
		pmu_raw_writel((pmu_raw_readl(0x0240) & ~BIT(16)) | (arm1 & BIT(16)), 0x0240);
		pr_info("exynos8895-pm: DEBUG noarm: CENTRAL_SEQ %08x MIF %08x (pre-enter %08x %08x)\n",
			pmu_raw_readl(0x0200), pmu_raw_readl(0x0240), arm0, arm1);
	}
	e8895_wakearm();					/* DEBUG l7wake */
	e8895_swpend();					/* DEBUG l7wake */
	if (sleep_trace) {					/* DEBUG */
		e8895_verify(t->t[0], t->n[0]);
		e8895_sleep_dump();
	}
	return 0;

refuse:
	e8895_dbg_allon();					/* DEBUG: sleep_alloff/sleep_pdoff */
	pr_err("exynos8895-pm: SLEEP refused: %s (%d)\n", why, ret);
	return ret;
}

/* S:drivers/soc/samsung/exynos-pm.c:257-273 */
static void e8895_pm_syscore_resume(void)
{
	struct e8895_tabs *t;
	u32 stat;

	if (!e8895_sleep_armed)			/* hibernation snapshot, or nothing programmed */
		return;
	if (sleep_trace)						/* DEBUG */
		pr_info("exynos8895-pm: DEBUG syscore resume early=%d mode=%d\n", e8895_early, e8895_mode);
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
	e8895_dbg_allon();					/* DEBUG */
}

static struct syscore_ops e8895_pm_syscore_ops = {
	.suspend = e8895_pm_syscore_suspend,
	.resume  = e8895_pm_syscore_resume,
};

/* S:drivers/soc/samsung/exynos-pm.c:295-326; cp_call_psci_idx = suspend_psci_idx = 0x85
 * (FDT:4418,4420), so every mode uses PSCI_SYSTEM_SLEEP */
static int e8895_pm_enter(suspend_state_t state)
{
	unsigned int cpu = smp_processor_id();
	int ret;

	if (sleep_trace)					/* DEBUG */
		pr_info("exynos8895-pm: DEBUG SMC power_state=0x%08x\n", e8895_smc_state());
	if (sleep_c2prep) {					/* DEBUG l7el3 */
		pr_info("exynos8895-pm: DEBUG c2prep: CONF ok=%d STATUS %x -> down\n",
			e8895_pmu_cpu_conf_ok(cpu), e8895_pmu_cpu_status(cpu));
		e8895_pmu_cpu_down(cpu);
		pr_info("exynos8895-pm: DEBUG c2prep: CONF ok=%d STATUS %x\n",
			e8895_pmu_cpu_conf_ok(cpu), e8895_pmu_cpu_status(cpu));
	}
	ret = e8895_psci_suspend(e8895_smc_state());		/* DEBUG sleep_psci */
	if (sleep_c2prep) {					/* DEBUG l7el3 */
		e8895_pmu_cpu_up(cpu);
		pr_info("exynos8895-pm: DEBUG c2prep: back %d, CONF ok=%d STATUS %x\n",
			ret, e8895_pmu_cpu_conf_ok(cpu), e8895_pmu_cpu_status(cpu));
	}
	if (sleep_trace)					/* DEBUG: first code after a wake */
		pr_info("exynos8895-pm: DEBUG back from SLEEP SMC: %d (finisher %d) WAKEUP_STAT=%08x\n",
			ret, ret ? this_cpu_read(e8895_dbg_fin_ret) : 0, pmu_raw_readl(E8895_WAKEUP_STAT));
	e8895_early = ret != 0;
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
	if (pm_suspend_target_state != PM_SUSPEND_MEM || dvs_skip)	/* DEBUG dvs_skip */
		return 0;
	for (i = 0; i < ARRAY_SIZE(e8895_dvs); i++) {
		if (!e8895_dvs_reg[i])
			continue;
		e8895_sp_probe("pm-dvs before", e8895_dvs[i].supply);	/* DEBUG stage_poke */
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
	e8895_sp_probe("pm-dvs", "all rails lowered");		/* DEBUG stage_poke */
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
 * Explicit members only: SYSTEM_SLEEP_PM_OPS/LATE_SYSTEM_SLEEP_PM_OPS would
 * also fill freeze/thaw/poweroff/restore (A:include/linux/pm.h:316-330), i.e.
 * run PM-DVS during hibernation. Stock set only
 * .suspend_late/.resume_early (S:drivers/soc/samsung/exynos-pm-dvs.c:129-132).
 */
static const struct dev_pm_ops e8895_pm_dev_ops = {
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
	/* SLEEP offlines CPUs 1-7 through the PMU CPU hooks' DYING callback */
	if (!e8895_pmu_cpu_hooks_ready()) {
		dev_err(dev, "PMU CPU hooks not ready: no SLEEP, mem falls back to s2idle\n");
		return -ENODEV;
	}
	if (sleep_trace)		/* DEBUG: what a forced reset after a SLEEP attempt left behind */
		pr_info("exynos8895-pm: DEBUG boot WAKEUP_STAT=%08x 0x640=%08x seq=%08x opt=%08x mif=%08x wk=%08x eint=%08x\n",
			pmu_raw_readl(E8895_WAKEUP_STAT), pmu_raw_readl(E8895_EXTRA_WAKEUP_STAT),
			pmu_raw_readl(0x0200), pmu_raw_readl(0x0208), pmu_raw_readl(0x0240),
			pmu_raw_readl(0x610), pmu_raw_readl(E8895_EINT_WAKEUP_MASK));
	WRITE_ONCE(e8895_pm_probed, true);
	/* DEBUG (l7el3): the value 0x60c has during cpuidle, before the pinctrl
	 * syscore op overwrites it with the EINT wake mask (sleep_bare_eint) */
	eint_wake_boot = pmu_raw_readl(E8895_EINT_WAKEUP_MASK);
	register_syscore_ops(&e8895_pm_syscore_ops);
	suspend_set_ops(&e8895_pm_ops);			/* A:kernel/power/suspend.c:224-241: mem = deep */
	bus_register_notifier(&platform_bus_type, &e8895_pm_bus_nb);
	e8895_sp_init();					/* DEBUG stage_poke */
	e8895_map_wake_blocks(dev);			/* DEBUG l7wake */
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
