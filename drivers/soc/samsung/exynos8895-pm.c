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
#include <asm/proc-fns.h>
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
static const u32 e8895_wkmask_aftr[4] = { 0x0, 0x0, 0x0, 0x4 };	/* DEBUG sleep_aftr: FDT:4444,4449,4454,4459 column 2 */

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

/* DEBUG sleep_sicd: set while the system suspend arms SICD; runtime PM is disabled for every
 * device then (A:drivers/base/power/main.c device_suspend_late()), which the RPM vetoes read
 * as busy, and every device has been suspended anyway */
static bool e8895_sicd_in_suspend;

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
	for (i = 0; i < e8895_nveto && !e8895_sicd_in_suspend; i++)
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

/*
 * DEBUG sleep_sicd=1 (L7, 2026-10-06): program SICD instead of SLEEP in the system suspend
 * (stock's SICD masks and enter list, e8895_sicd_prepare(), and power_state 0x00000001 =
 * PSCI_SYSTEM_IDLE) and run SICD's exit/early-wakeup list in syscore resume. SICD shares the
 * start of the SLEEP hardware sequence (CPU and cluster power-down, CMU clock stop) but
 * keeps TOP powered; stock's SLEEP and SLEEP_AUD_ON lists differ only in ABOX, MIF arming,
 * CENTRAL_SEQ_OPTION bit 1 and VTS (CAL:pmucal_system_exynos8895.h:773-920, 2403-), and
 * both hang. Returns => the shared start works and the stall is in the TOP power-down part.
 */
static bool sleep_sicd;
module_param(sleep_sicd, bool, 0644);
static bool e8895_sicd_armed;

/*
 * DEBUG sleep_aftr=1 (L7, 2026-10-06): program stock's AFTR ("ARM off, TOP running") instead of
 * SLEEP, with the SLEEP power_state so EL3 still flushes and prepares as for SLEEP. AFTR
 * powers the CPU clusters, L2 and DBG off through the central sequencer but keeps every PLL,
 * the DDRPHY DLL and the MIF bus running (CAL:pmucal_system_exynos8895.h:393-540); SICD,
 * which hung in this context (image 0170, work/l7c/deep/sicd2), does the opposite. Returns
 * => cluster power-down and the sequencer/wake path work, the stall is in the clock-down
 * (PLL off / CMU clock stop) part shared by SICD and SLEEP.
 */
static bool sleep_aftr;
module_param(sleep_aftr, bool, 0644);
static bool e8895_aftr_armed;

/*
 * DEBUG (L7, 2026-10-06) sleep_emul=<mask>: after the mode's enter list, set PMU emulation bits so
 * the central sequencer skips that handshake: bits 3/1/0 = TOP_BUS_MIF_OPTION EMULATION_APM_IF /
 * EMULATION_BUS / EMULATION_BUSMASTER (0x2c88, CAL:pmucal_system_exynos8895.h:317-320), bit 31 =
 * TOP_PWR_APM_OPTION[31] EMULATION (0x82c8, :344). Bit 2 (EMULATION_DRAM) is never set: without the
 * DRAM handshake the MIF clocks stop with no self-refresh and RAM is lost. Stock's SLEEP list clears
 * all of them (:1550-1553,1576); its SICD list sets BUS and BUSMASTER (:319-320). Reading for SLEEP:
 * no reset at sleep_wdt any more = TOP powered down, so a skipped handshake was the stall.
 */
#define E8895_TOP_BUS_MIF_OPTION	0x2c88
#define E8895_TOP_PWR_APM_OPTION	0x82c8
#define E8895_EMUL_OK			(BIT(31) | BIT(3) | BIT(1) | BIT(0))
static unsigned int sleep_emul;
module_param(sleep_emul, uint, 0644);
static u32 e8895_emul_save[2];
static bool e8895_emul_set;

static void e8895_emul_apply(void)
{
	u32 m = READ_ONCE(sleep_emul) & E8895_EMUL_OK;

	e8895_emul_set = false;
	if (!m)
		return;
	e8895_emul_save[0] = pmu_raw_readl(E8895_TOP_BUS_MIF_OPTION);
	e8895_emul_save[1] = pmu_raw_readl(E8895_TOP_PWR_APM_OPTION);
	pmu_raw_writel(e8895_emul_save[0] | (m & 0xf), E8895_TOP_BUS_MIF_OPTION);
	if (m & BIT(31))
		pmu_raw_writel(e8895_emul_save[1] | BIT(31), E8895_TOP_PWR_APM_OPTION);
	e8895_emul_set = true;
	pr_info("exynos8895-pm: DEBUG emul: TOP_BUS_MIF_OPTION %08x -> %08x TOP_PWR_APM_OPTION %08x -> %08x\n",
		e8895_emul_save[0], pmu_raw_readl(E8895_TOP_BUS_MIF_OPTION),
		e8895_emul_save[1], pmu_raw_readl(E8895_TOP_PWR_APM_OPTION));
}

/* the exit lists clear the bits again; this also covers the early-wakeup and refusal paths */
static void e8895_emul_restore(void)
{
	if (!e8895_emul_set)
		return;
	e8895_emul_set = false;
	pmu_raw_writel(e8895_emul_save[0], E8895_TOP_BUS_MIF_OPTION);
	pmu_raw_writel(e8895_emul_save[1], E8895_TOP_PWR_APM_OPTION);
}

/*
 * DEBUG (L7, 2026-10-06) sleep_ovr="off:val[,off:val...]" (hex, up to 16): after the SLEEP enter
 * list, rewrite PMU words that the list itself writes (PMU base, WRITE rows only; the union of
 * that offset's row masks applies), e.g. a SYS_PWR register back to its SICD/AFTR value so the
 * central sequencer leaves that block alone. Bisection tool for the SLEEP entry stall: with
 * sleep_wdt, an override that removes the stall shows as no reset at the watchdog. Syscore
 * resume puts the saved words back before the exit list runs.
 */
#define E8895_OVR_MAX	16
static char sleep_ovr[200];
module_param_string(sleep_ovr, sleep_ovr, sizeof(sleep_ovr), 0644);
static struct {
	u16 off;
	u32 old;
} e8895_ovr_save[E8895_OVR_MAX];
static unsigned int e8895_ovr_n;

static void e8895_ovr_apply(const struct e8895_seq *s, unsigned int n)
{
	char buf[sizeof(sleep_ovr)], *p, *tok;
	unsigned int i, off, val;
	u32 mask;

	e8895_ovr_n = 0;
	strscpy(buf, sleep_ovr, sizeof(buf));
	p = strim(buf);
	while ((tok = strsep(&p, ",")) && e8895_ovr_n < E8895_OVR_MAX) {
		if (!*tok)
			continue;
		if (sscanf(tok, "%x:%x", &off, &val) != 2) {
			pr_err("exynos8895-pm: DEBUG ovr: bad token '%s'\n", tok);
			continue;
		}
		for (mask = 0, i = 0; i < n; i++)
			if (s[i].type == E8895_WRITE && s[i].base == 0 && s[i].off == off)
				mask |= s[i].mask;
		if (!mask) {
			pr_err("exynos8895-pm: DEBUG ovr: 0x%x is not written by this enter list\n", off);
			continue;
		}
		e8895_ovr_save[e8895_ovr_n].off = off;
		e8895_ovr_save[e8895_ovr_n].old = pmu_raw_readl(off);
		pmu_raw_writel((e8895_ovr_save[e8895_ovr_n].old & ~mask) | (val & mask), off);
		pr_info("exynos8895-pm: DEBUG ovr: 0x%04x %08x -> %08x (mask %08x)\n", off,
			e8895_ovr_save[e8895_ovr_n].old, pmu_raw_readl(off), mask);
		e8895_ovr_n++;
	}
}

static void e8895_ovr_restore(void)
{
	while (e8895_ovr_n) {
		e8895_ovr_n--;
		pmu_raw_writel(e8895_ovr_save[e8895_ovr_n].old, e8895_ovr_save[e8895_ovr_n].off);
	}
}
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

/*
 * DEBUG (L7, 2026-10-05) sleep_wfi=1 (with sleep_bare=1 only): ->enter runs a plain WFI
 * (cpu_do_idle()) instead of the PSCI SMC, i.e. the whole deep-suspend context (device
 * noirq stage, CPUs 1-7 offline, syscore suspend) without any EL3 call. sleep_bare=1 keeps
 * the PMU in its run-time state, so the WFI cannot start the CENTRAL sequencer. A wake
 * proves that the wake interrupt reaches CPU0 through the EINT block and the GIC in this
 * context; no wake means the Linux side of the wake path is broken. (sleep_bare alone =
 * cpuidle c2_mode=2, which hangs at run time too: L7 B3/B4, work/full/l7/STATUS.md:113,138.)
 */
static bool sleep_wfi;
module_param(sleep_wfi, bool, 0644);
static bool sleep_bare_eint;
module_param(sleep_bare_eint, bool, 0644);
static u32 eint_wake_boot;			/* 0x60c at probe (== the cpuidle value) */

/*
 * DEBUG (L7, 2026-10-05) sleep_wdt=<s>: arm the PERIS watchdog for a reset after <s> seconds
 * right before the SMC (or the sleep_wfi WFI) and disarm it as soon as ->enter runs again.
 * The setup is the boot code's, which arms this timer on every boot (SB:0x282a8-0x28300:
 * WTCON prescaler 0xee, DIV128, ENABLE, RSTEN, INTEN off; WTDAT = WTCNT = 0xff60, 77 s;
 * PMU 0x408/0x40c = 0xfeffffff, i.e. bit 24 = this timer neither auto-disabled nor masked).
 * Stock enters SLEEP with the timer stopped (tmr_atboot 0, S:drivers/watchdog/s3c2410_wdt.c:
 * 68,846-855; WTCON 0x5c00 restored after the wake, work/l7c/stock44-suspend/
 * dmesg-real.txt:1447) but with bit 24 clear in 0x408/0x40c (:1442; its syscore suspend
 * writes neither, S:...s3c2410_wdt.c:656-669), while our driver's suspend sets bit 24 in
 * both (drivers/watchdog/s3c2410_wdt.c s3c2410wdt_suspend()). So the PMU bits are stock's
 * here and the running timer is the instrument.
 * The counter runs on OSCCLK in CMU_PERIS (DT watchdog_src), i.e. in the TOP domain:
 *   reset after <s> s => the SoC stayed powered and clocked: SLEEP was never entered (CPU0
 *                        parked in a deaf WFI, or a power-down sequence that stalled before
 *                        the oscillator / TOP went off); the reboot also ends the attempt
 *                        without a forced reset, with DRAM (pstore, EL3 page) kept
 *   no reset          => TOP or its oscillator went down (SLEEP entered, then no wake)
 * The reboot shows as RST_STAT bit 24 (the 8895 variant of drivers/watchdog/s3c2410_wdt.c,
 * rst_stat_bit 24 as S:...s3c2410_wdt.c:179-185) in the probe line below. 0 = off; at most
 * 77 (16-bit WTCNT at the boot code's prescaler).
 * sleep_wdt_auto=1: leave 0x408 bit 24 (AUTOMATIC_WDT_RESET_DISABLE, set by our driver's
 * suspend) as it is: the PMU then drops the watchdog reset itself once it enters a
 * low-power mode, so a reset with it set says the PMU never began one at all.
 */
static unsigned int sleep_wdt;
module_param(sleep_wdt, uint, 0644);
static bool sleep_wdt_auto;
module_param(sleep_wdt_auto, bool, 0644);

#define E8895_WTCON		0x00	/* drivers/watchdog/s3c2410_wdt.c:32-35 */
#define E8895_WTDAT		0x04
#define E8895_WTCNT		0x08
#define E8895_WTCLRINT		0x0c
#define E8895_WTCON_RSTEN	BIT(0)	/* :40-53 */
#define E8895_WTCON_INTEN	BIT(2)
#define E8895_WTCON_DIV128	(3 << 3)
#define E8895_WTCON_DIV_MASK	(3 << 3)
#define E8895_WTCON_ENABLE	BIT(5)
#define E8895_WTCON_PRESCALE(x)	((x) << 8)
#define E8895_WTCON_PRESCALE_MASK (0xff << 8)
#define E8895_WDT_PRESCALE	0xee	/* SB:0x282e0-0x282e4: orr #0xe00, orr #0xe000 */
#define E8895_WDT_OSC		26000000	/* oscclk, the DT watchdog_src clock */
#define E8895_WDT_DISABLE	0x0408	/* EXYNOS5_WDT_DISABLE_REG_OFFSET; SB:0x282cc */
#define E8895_WDT_MASK_RESET	0x040c	/* EXYNOS5_WDT_MASK_RESET_REG_OFFSET; SB:0x282d0 */
#define E8895_WDT_BIT		BIT(24)	/* S:drivers/watchdog/s3c2410_wdt.c:182 mask_bit */
#define E8895_RST_STAT		0x0404	/* EXYNOS5_RST_STAT_REG_OFFSET */

static void __iomem *e8895_wdt_base;
static u32 e8895_wdt_con, e8895_wdt_dat, e8895_wdt_dis, e8895_wdt_mask;
static bool e8895_wdt_armed;

/* timer + PMU setup shared by the SLEEP path (verbose) and the s2idle SICD path (quiet: it runs
 * in the idle path with IRQs off, so nothing there may print) */
static u32 e8895_wdt_start(unsigned int secs, bool keep_auto)
{
	void __iomem *w = e8895_wdt_base;
	u32 cnt, con;

	cnt = min_t(u64, 0xffff, DIV_ROUND_UP((u64)secs * E8895_WDT_OSC,
					      128 * (E8895_WDT_PRESCALE + 1)));
	e8895_wdt_con = readl(w + E8895_WTCON);
	e8895_wdt_dat = readl(w + E8895_WTDAT);
	e8895_wdt_dis = pmu_raw_readl(E8895_WDT_DISABLE);
	e8895_wdt_mask = pmu_raw_readl(E8895_WDT_MASK_RESET);
	/* the boot code's order: stop, clear the interrupt, open the PMU, load, start */
	writel(e8895_wdt_con & ~(E8895_WTCON_ENABLE | E8895_WTCON_RSTEN), w + E8895_WTCON);
	writel(1, w + E8895_WTCLRINT);
	if (!keep_auto)
		pmu_raw_writel(e8895_wdt_dis & ~E8895_WDT_BIT, E8895_WDT_DISABLE);
	pmu_raw_writel(e8895_wdt_mask & ~E8895_WDT_BIT, E8895_WDT_MASK_RESET);
	writel(cnt, w + E8895_WTDAT);
	writel(cnt, w + E8895_WTCNT);
	con = (e8895_wdt_con & ~(E8895_WTCON_INTEN | E8895_WTCON_PRESCALE_MASK |
				 E8895_WTCON_DIV_MASK)) |
	      E8895_WTCON_PRESCALE(E8895_WDT_PRESCALE) | E8895_WTCON_DIV128 |
	      E8895_WTCON_ENABLE | E8895_WTCON_RSTEN;
	writel(con, w + E8895_WTCON);
	e8895_wdt_armed = true;
	return cnt;
}

/* after a real SLEEP the timer is the one the wake path left; stopped either way. Returns the
 * count that was left. */
static u32 e8895_wdt_stop(void)
{
	void __iomem *w = e8895_wdt_base;
	u32 left;

	e8895_wdt_armed = false;
	writel(readl(w + E8895_WTCON) & ~(E8895_WTCON_ENABLE | E8895_WTCON_RSTEN),
	       w + E8895_WTCON);
	left = readl(w + E8895_WTCNT);
	writel(1, w + E8895_WTCLRINT);
	pmu_raw_writel(e8895_wdt_dis, E8895_WDT_DISABLE);
	pmu_raw_writel(e8895_wdt_mask, E8895_WDT_MASK_RESET);
	writel(e8895_wdt_dat, w + E8895_WTDAT);
	writel(e8895_wdt_con, w + E8895_WTCON);
	return left;
}

static void e8895_wdt_arm(void)
{
	u32 cnt;

	if (!sleep_wdt)
		return;
	if (!e8895_wdt_base) {
		pr_warn("exynos8895-pm: DEBUG wdt: no watchdog mapping, sleep_wdt ignored\n");
		return;
	}
	cnt = e8895_wdt_start(sleep_wdt, sleep_wdt_auto);
	pr_info("exynos8895-pm: DEBUG wdt: reset in %u s: WTCON %08x -> %08x WTCNT %04x, 0x408 %08x -> %08x 0x40c %08x -> %08x\n",
		sleep_wdt, e8895_wdt_con, readl(e8895_wdt_base + E8895_WTCON), cnt,
		e8895_wdt_dis, pmu_raw_readl(E8895_WDT_DISABLE), e8895_wdt_mask,
		pmu_raw_readl(E8895_WDT_MASK_RESET));
}

static void e8895_wdt_disarm(void)
{
	u32 left;

	if (!e8895_wdt_armed)
		return;
	left = e8895_wdt_stop();
	pr_info("exynos8895-pm: DEBUG wdt: disarmed, WTCNT %04x left, WTCON %08x\n",
		left, readl(e8895_wdt_base + E8895_WTCON));
}

/*
 * DEBUG (L7, 2026-10-05) sleep_hwacg=1: put the 103 gates our clock driver took out of
 * hardware auto-gating back into it for the SMC, and restore them as soon as ->enter runs
 * again. exynos_arm64_init_clocks() sets GATE_MANUAL (bit 20) and clears GATE_ENABLE_HWACG
 * (bit 28) on every gate register of a CMU's clk_regs[] (drivers/clk/samsung/
 * clk-exynos-arm64.c:142-170). Stock never wrote the MANUAL bit: ra_set_gate()/ra_get_gate()
 * drive CG_VALUE or ENABLE_AUTOMATIC_CLKGATING in whatever mode the gate is in
 * (S:drivers/soc/samsung/cal-if/ra.c:378-419), so its gates entered SLEEP in the boot mode,
 * auto-gating, apart from the USI/UAIF/DSIF/JPEG set it took off the Q-channel (VCLK_QCH_DIS,
 * S:drivers/clk/samsung/clk-exynos8895.c:48-53,273,372-418). The SLEEP enter list asks
 * CMU_TOP for a clock stop (CAL:pmucal_system_exynos8895.h:802 CLKSTOP_CMU_TOP = 1) before
 * TOP powers off (:816 TOP_PWR = 0); if that stop waits for the gates' Q-channel handshakes,
 * a manual gate with CG_VAL set (forced on) can hold it. The table = the gate entries
 * (offsets 0x2000-0x2fff, clk-exynos-arm64.c:32-33) of top/peris/fsys0/fsys1/peric0/
 * peric1_clk_regs[] (drivers/clk/samsung/clk-exynos8895.c:304,1334,1546,1804,2065,2304),
 * the six CMUs outside every power domain; the CMUs inside one are off at the SMC (the
 * sleep_trace pd dump reads 0/0 for all but dbg). With sleep_trace=1, ->enter lists the
 * forced-on gates whether or not sleep_hwacg is set.
 */
static bool sleep_hwacg;
module_param(sleep_hwacg, bool, 0644);

#define E8895_GATE_MANUAL	BIT(20)	/* clk-exynos-arm64.c:24 */
#define E8895_GATE_CG_VAL	BIT(21)	/* the enable bit of every GATE() in clk-exynos8895.c */
#define E8895_GATE_HWACG	BIT(28)	/* clk-exynos-arm64.c:25 */

static const u16 e8895_hwacg_top[] = {	/* 25 */
	0x2060, 0x2064, 0x2068, 0x206c, 0x2074, 0x2078, 0x207c, 0x20b4, 0x20b8, 0x20bc,
	0x20c0, 0x20c8, 0x20cc, 0x20d0, 0x20d4, 0x20d8, 0x20dc, 0x20e0, 0x20e4, 0x20e8,
	0x20ec, 0x20f0, 0x20f4, 0x20f8, 0x20fc
};
static const u16 e8895_hwacg_peris[] = {	/* 1 */
	0x202c
};
static const u16 e8895_hwacg_fsys0[] = {	/* 11 */
	0x202c, 0x2030, 0x2050, 0x2054, 0x206c, 0x2070, 0x2078, 0x207c, 0x2080, 0x2084,
	0x2088
};
static const u16 e8895_hwacg_fsys1[] = {	/* 17 */
	0x2000, 0x2034, 0x2038, 0x203c, 0x2040, 0x2044, 0x2048, 0x204c, 0x2050, 0x2054,
	0x2058, 0x205c, 0x2060, 0x2094, 0x2098, 0x209c, 0x20a0
};
static const u16 e8895_hwacg_peric0[] = {	/* 10 */
	0x2028, 0x2030, 0x2040, 0x2044, 0x2048, 0x204c, 0x2050, 0x2054, 0x2058, 0x205c
};
static const u16 e8895_hwacg_peric1[] = {	/* 39 */
	0x200c, 0x202c, 0x2030, 0x2034, 0x2038, 0x2048, 0x204c, 0x2050, 0x2054, 0x2058,
	0x205c, 0x2060, 0x2064, 0x2068, 0x206c, 0x2070, 0x2074, 0x207c, 0x2080, 0x2084,
	0x2088, 0x208c, 0x2090, 0x2094, 0x2098, 0x209c, 0x20a0, 0x20a4, 0x20a8, 0x20ac,
	0x20b0, 0x20b4, 0x20b8, 0x20bc, 0x20c0, 0x20c4, 0x20c8, 0x20cc, 0x20d0
};

/* bases: the cmu_* nodes of exynos8895.dtsi (:2789, :425, :1664, :1734, :557, :860) */
static const struct {
	const char *name;
	phys_addr_t base;
	const u16 *off;
	unsigned int n;
} e8895_hwacg_cmus[] = {
	{ "top",    0x15a80000, e8895_hwacg_top,    ARRAY_SIZE(e8895_hwacg_top) },
	{ "peris",  0x10010000, e8895_hwacg_peris,  ARRAY_SIZE(e8895_hwacg_peris) },
	{ "fsys0",  0x11000000, e8895_hwacg_fsys0,  ARRAY_SIZE(e8895_hwacg_fsys0) },
	{ "fsys1",  0x11400000, e8895_hwacg_fsys1,  ARRAY_SIZE(e8895_hwacg_fsys1) },
	{ "peric0", 0x10400000, e8895_hwacg_peric0, ARRAY_SIZE(e8895_hwacg_peric0) },
	{ "peric1", 0x10800000, e8895_hwacg_peric1, ARRAY_SIZE(e8895_hwacg_peric1) },
};

#define E8895_HWACG_N	(ARRAY_SIZE(e8895_hwacg_top) + ARRAY_SIZE(e8895_hwacg_peris) + \
			 ARRAY_SIZE(e8895_hwacg_fsys0) + ARRAY_SIZE(e8895_hwacg_fsys1) + \
			 ARRAY_SIZE(e8895_hwacg_peric0) + ARRAY_SIZE(e8895_hwacg_peric1))

static void __iomem *e8895_hwacg_va[ARRAY_SIZE(e8895_hwacg_cmus)];
static u32 e8895_hwacg_saved[E8895_HWACG_N];
static bool e8895_hwacg_done;

/* all six or none, so that the saved values always line up with the table */
static void e8895_hwacg_map(struct device *dev)
{
	unsigned int c;

	for (c = 0; c < ARRAY_SIZE(e8895_hwacg_cmus); c++) {
		/* no region claim: the clock driver owns these blocks */
		e8895_hwacg_va[c] = devm_ioremap(dev, e8895_hwacg_cmus[c].base, SZ_16K);
		if (!e8895_hwacg_va[c]) {
			dev_warn(dev, "DEBUG: cannot map cmu_%s, no sleep_hwacg\n",
				 e8895_hwacg_cmus[c].name);
			memset(e8895_hwacg_va, 0, sizeof(e8895_hwacg_va));
			return;
		}
	}
}

/* DEBUG sleep_trace: per CMU, how many listed gates are manual and which are forced on */
static void e8895_hwacg_dump(void)
{
	unsigned int c, i;

	if (!e8895_hwacg_va[0])
		return;
	for (c = 0; c < ARRAY_SIZE(e8895_hwacg_cmus); c++) {
		unsigned int manual = 0, forced = 0;
		char buf[128];
		int n = 0;

		buf[0] = 0;
		for (i = 0; i < e8895_hwacg_cmus[c].n; i++) {
			u16 off = e8895_hwacg_cmus[c].off[i];
			u32 v = readl(e8895_hwacg_va[c] + off);

			if (!(v & E8895_GATE_MANUAL))
				continue;
			manual++;
			if (v & E8895_GATE_CG_VAL) {
				forced++;
				n += scnprintf(buf + n, sizeof(buf) - n, " %x", off);
			}
		}
		pr_info("exynos8895-pm: DEBUG pre-SMC gates cmu_%s: %u listed, %u manual, %u forced on:%s\n",
			e8895_hwacg_cmus[c].name, e8895_hwacg_cmus[c].n, manual, forced,
			forced ? buf : " -");
	}
}

/* gate switch shared by the SLEEP path (verbose) and the s2idle SICD path (quiet) */
static unsigned int e8895_hwacg_switch(void)
{
	unsigned int c, i, k = 0, changed = 0;

	for (c = 0; c < ARRAY_SIZE(e8895_hwacg_cmus); c++) {
		for (i = 0; i < e8895_hwacg_cmus[c].n; i++) {
			void __iomem *a = e8895_hwacg_va[c] + e8895_hwacg_cmus[c].off[i];
			u32 v = readl(a);

			e8895_hwacg_saved[k++] = v;
			if ((v & E8895_GATE_MANUAL) || !(v & E8895_GATE_HWACG)) {
				writel((v & ~E8895_GATE_MANUAL) | E8895_GATE_HWACG, a);
				changed++;
			}
		}
	}
	e8895_hwacg_done = true;
	return changed;
}

static void e8895_hwacg_unswitch(void)
{
	unsigned int c, i, k = 0;

	e8895_hwacg_done = false;
	for (c = 0; c < ARRAY_SIZE(e8895_hwacg_cmus); c++)
		for (i = 0; i < e8895_hwacg_cmus[c].n; i++)
			writel(e8895_hwacg_saved[k++],
			       e8895_hwacg_va[c] + e8895_hwacg_cmus[c].off[i]);
}

static void e8895_hwacg_set(void)
{
	unsigned int changed;

	if (!sleep_hwacg)
		return;
	if (!e8895_hwacg_va[0]) {
		pr_warn("exynos8895-pm: DEBUG hwacg: no CMU mapping, sleep_hwacg ignored\n");
		return;
	}
	changed = e8895_hwacg_switch();
	pr_info("exynos8895-pm: DEBUG hwacg: %u of %u gates back to auto-gating\n", changed,
		(unsigned int)E8895_HWACG_N);
}

static void e8895_hwacg_restore(void)
{
	if (!e8895_hwacg_done)
		return;
	e8895_hwacg_unswitch();
	pr_info("exynos8895-pm: DEBUG hwacg: %u gates back to manual\n", (unsigned int)E8895_HWACG_N);
}

/*
 * DEBUG (L7, 2026-10-05) the same two instruments for SICD taken from s2idle
 * (cpuidle_exynos8895.sicd_enabled=1 sicd_s2idle=1). SICD runs the CENTRAL sequencer and asks
 * CMU_TOP for the same clock stop as SLEEP (CAL:pmucal_system_exynos8895.h:232
 * CLKSTOP_CMU_TOP = 1) but keeps TOP powered (:246 TOP_PWR = 3), so it tests the clock stop
 * on its own:
 *   sicd_wdt=<s>  arm the watchdog when the s2idle path takes SICD, disarm when the first CPU
 *                 leaves it. Keep <s> above the wake interval (e.g. sicd_wdt=20 with the RTC at
 *                 +10 s): the counter runs on OSCCLK, which SICD keeps (:252 OSCCLK_SLEEP = 1),
 *                 so a long but healthy SICD would also fire it.
 *   sicd_hwacg=1  the sleep_hwacg gate switch around the SICD SMC.
 * Both run under the cpuidle driver's c2_lock with IRQs off and print nothing there;
 * PM_POST_SUSPEND reports the counts and the smallest count the watchdog had left.
 */
static unsigned int sicd_wdt;
module_param(sicd_wdt, uint, 0644);
static bool sicd_hwacg;
module_param(sicd_hwacg, bool, 0644);
static unsigned int e8895_sicd_dbg_arms, e8895_sicd_dbg_hwacg;
static u32 e8895_sicd_dbg_left_min = U32_MAX;

void e8895_dbg_sicd_enter(void)
{
	if (READ_ONCE(sicd_hwacg) && e8895_hwacg_va[0]) {
		e8895_hwacg_switch();
		e8895_sicd_dbg_hwacg++;
	}
	if (READ_ONCE(sicd_wdt) && e8895_wdt_base) {
		e8895_wdt_start(READ_ONCE(sicd_wdt), false);
		e8895_sicd_dbg_arms++;
	}
}

void e8895_dbg_sicd_leave(void)
{
	if (e8895_wdt_armed)
		e8895_sicd_dbg_left_min = min(e8895_sicd_dbg_left_min, e8895_wdt_stop());
	if (e8895_hwacg_done)
		e8895_hwacg_unswitch();
}

static int e8895_sicd_dbg_pm_fn(struct notifier_block *nb, unsigned long ev, void *p)
{
	if (ev == PM_SUSPEND_PREPARE) {
		e8895_sicd_dbg_arms = 0;
		e8895_sicd_dbg_hwacg = 0;
		e8895_sicd_dbg_left_min = U32_MAX;
	} else if (ev == PM_POST_SUSPEND && (e8895_sicd_dbg_arms || e8895_sicd_dbg_hwacg)) {
		pr_info("exynos8895-pm: DEBUG sicd: watchdog armed %u times (min WTCNT left %04x), hwacg %u times\n",
			e8895_sicd_dbg_arms, e8895_sicd_dbg_left_min, e8895_sicd_dbg_hwacg);
	}
	return NOTIFY_DONE;
}

static struct notifier_block e8895_sicd_dbg_pm_nb = { .notifier_call = e8895_sicd_dbg_pm_fn };

/*
 * DEBUG (L7, 2026-10-05, sleep_trace): the APM side right before the SMC. SLEEP powers the APM
 * down too (CAL:pmucal_system_exynos8895.h:883 TOP_PWR_APM = 0, :886 CPU_QCH_CORTEXM3_APM = 0),
 * which needs its Cortex-M3 idle; an IPC either side has not taken yet keeps it busy.
 * AP2APM mailbox INTSR0/INTMSR0 (AP -> APM pending, raw / unmasked) 0x14/0x18 and INTSR1/INTMSR1
 * (APM -> AP) 0x28/0x2c (S:drivers/soc/samsung/acpm/acpm_ipc.h:78-87). No PMU word here: the
 * CORTEXM3_APM block (S:include/linux/apm-exynos.h:117-119) is not known to be open to the
 * normal world, and image 0159 took a synchronous external abort reading PMU 0x0030
 * (CP_CTRL_NS) through peek (work/l7c/deep/boot0159/crash), so the PMU is not uniformly readable.
 */
static void __iomem *e8895_mbox_base;

/*
 * DEBUG (L7, 2026-10-05) sleep_mbox_clr=1: clear the APM -> AP status bits of the polled
 * (masked) mailbox channels right before the SMC. Stock clears a polled channel's INTSR1 bit as
 * soon as its RX queue is drained (S:drivers/soc/samsung/acpm/acpm_ipc.c:287-288 "if (rear ==
 * front) __raw_writel((1 << channel->id), acpm_ipc->intr + INTCR1)"), so it enters SLEEP with
 * INTSR1 = 0; our mailbox driver clears it only right before the next doorbell
 * (drivers/mailbox/exynos-mailbox.c exynos_mbox_send_data()), so the last pre-suspend IPCs leave
 * their bits set through SLEEP (image 0160: sr1 = 0x17, all masked). INTCR1 0x20, INTMR1 0x24,
 * INTSR1 0x28 (S:drivers/soc/samsung/acpm/acpm_ipc.h:84-86).
 */
static bool sleep_mbox_clr;
module_param(sleep_mbox_clr, bool, 0644);

static void e8895_mbox_clr(void)
{
	u32 sr1, mr1;

	if (!sleep_mbox_clr || !e8895_mbox_base)
		return;
	sr1 = readl(e8895_mbox_base + 0x28);
	mr1 = readl(e8895_mbox_base + 0x24);
	writel(sr1 & mr1, e8895_mbox_base + 0x20);
	pr_info("exynos8895-pm: DEBUG mbox clr: sr1 %08x (mr1 %08x) -> %08x\n", sr1, mr1,
		readl(e8895_mbox_base + 0x28));
}

/*
 * DEBUG (L7, 2026-10-06) the SPEEDY IP batcher of the AP (PMIC bus, S:arch/arm64/boot/dts/exynos/
 * exynos8895.dtsi:2699-2711 "SPEEDY IP_BATCHER_AP" speedy@15B50000). Its multi-processor APB
 * semaphore arbitrates the one SPEEDY master between the AP, CP and APM batchers. 6.18 has no
 * SPEEDY driver, and at run time IPBATCHER_STATE reads 0x00100008: MP_APBSEMA_CH_LOCK_STATUS
 * (bit 20, S:drivers/i2c/busses/i2c-speedy.c:181) set, i.e. the AP channel still holds the
 * semaphore S-Boot took for its own PMIC accesses. Stock's probe releases it ("release semaphore
 * after direct SPEEDY SFR access", :1044-1045, write 1 to IPBATCHER_SEMA_REL 0x200, :444-447).
 *   speedy_rel=1    release it now (process context), log IPBATCHER_STATE before and after
 *   sleep_speedy=1  the same in ->enter right before the SMC
 * Only the release: stock's controller/batcher software resets (:1029, :1048) are not done at
 * run time, the APM may be using the SPEEDY master.
 */
#define E8895_SPEEDY_AP		0x15b50000
#define E8895_IPB_SEMA_REL	0x200		/* S:drivers/i2c/busses/i2c-speedy.c:152 */
#define E8895_IPB_CON		0x500		/* :137 */
#define E8895_IPB_STATE		0x504		/* :138 */
#define E8895_IPB_LOCK		BIT(20)		/* :181 MP_APBSEMA_CH_LOCK_STATUS */
static void __iomem *e8895_speedy;
static bool sleep_speedy;
module_param(sleep_speedy, bool, 0644);

static void e8895_speedy_release(const char *tag)
{
	u32 st0, st1;

	if (!e8895_speedy)
		return;
	st0 = readl(e8895_speedy + E8895_IPB_STATE);
	if (st0 & E8895_IPB_LOCK)
		writel(0x1, e8895_speedy + E8895_IPB_SEMA_REL);
	st1 = readl(e8895_speedy + E8895_IPB_STATE);
	pr_info("exynos8895-pm: DEBUG speedy %s: IPBATCHER_STATE %08x -> %08x CON %08x\n",
		tag, st0, st1, readl(e8895_speedy + E8895_IPB_CON));
}

static int e8895_speedy_rel_set(const char *val, const struct kernel_param *kp)
{
	if (!e8895_speedy)
		e8895_speedy = ioremap(E8895_SPEEDY_AP, SZ_4K);
	if (!e8895_speedy)
		return -ENOMEM;
	e8895_speedy_release("now");
	return 0;
}
static const struct kernel_param_ops e8895_speedy_rel_ops = { .set = e8895_speedy_rel_set };
module_param_cb(speedy_rel, &e8895_speedy_rel_ops, NULL, 0200);

/*
 * DEBUG (L7, 2026-10-06): the APM firmware-log ring indices (SRAM 0x16502cd0 ktime index,
 * 0x16502cd8 rear, 0x16502cdc front; entries at 0x16502ce0, 50 x 16 bytes). Stock's APM logs
 * a whole SLEEP round between the SMC and the wake (work/l7c/regsnap/stock44-apm-*.txt:
 * irq_num 0x40, plug_in 0xe, sci_init, plg_in 1, ..., timeout 0xe); the front index at the
 * SMC tells which ring slots a stalled attempt wrote, read back at the next boot (exynos-acpm
 * "fwlog boot[]" dump).
 */
static void __iomem *e8895_apmlog;

/* DEBUG: the whole APM ring as it was at the end of the last syscore suspend / at the SMC,
 * for pm_test=core runs (no SMC, no hang); printed oldest -> newest by reading apmlog_last */
#define E8895_APMLOG_WORDS	(3 + 50 * 4)	/* ktime index, rear, front; 50 x 16-byte entries */
static u32 e8895_apmlog_copy[E8895_APMLOG_WORDS];
static const char *e8895_apmlog_where = "none";

static void e8895_apmlog_take(const char *where)
{
	int i;

	if (!e8895_apmlog)
		return;
	e8895_apmlog_copy[0] = readl(e8895_apmlog + 0xcd0);
	e8895_apmlog_copy[1] = readl(e8895_apmlog + 0xcd8);
	e8895_apmlog_copy[2] = readl(e8895_apmlog + 0xcdc);
	for (i = 0; i < 50 * 4; i++)
		e8895_apmlog_copy[3 + i] = readl(e8895_apmlog + 0xce0 + 4 * i);
	e8895_apmlog_where = where;
}

static int e8895_apmlog_get(char *buf, const struct kernel_param *kp)
{
	u32 front = e8895_apmlog_copy[2];
	int len, n;

	len = scnprintf(buf, PAGE_SIZE, "%s: ktime_index=%u rear=%u front=%u (oldest first)\n",
			e8895_apmlog_where, e8895_apmlog_copy[0], e8895_apmlog_copy[1], front);
	for (n = 0; n < 50 && front < 50; n++) {
		const u32 *e = &e8895_apmlog_copy[3 + ((front + n) % 50) * 4];
		char str[9];

		memcpy(str, &e[1], 8);
		str[8] = '\0';
		len += scnprintf(buf + len, PAGE_SIZE - len, "%02u %08x cnt=%7u %-8s %08x\n",
				 (front + n) % 50, e[0], e[0] & 0x7fffff, str, e[3]);
	}
	return len;
}
static const struct kernel_param_ops e8895_apmlog_ops = { .get = e8895_apmlog_get };
module_param_cb(apmlog_last, &e8895_apmlog_ops, NULL, 0444);

static void e8895_apmlog_print(void)
{
	if (e8895_apmlog)
		pr_info("exynos8895-pm: DEBUG pre-SMC apm fwlog ktime_index=%u rear=%u front=%u\n",
			readl(e8895_apmlog + 0xcd0), readl(e8895_apmlog + 0xcd8),
			readl(e8895_apmlog + 0xcdc));
}

static void e8895_apm_dump(void)
{
	if (!e8895_mbox_base)
		return;
	pr_info("exynos8895-pm: DEBUG pre-SMC apm mbox sr0=%08x msr0=%08x sr1=%08x msr1=%08x speedy_ipb=%08x\n",
		readl(e8895_mbox_base + 0x14), readl(e8895_mbox_base + 0x18),
		readl(e8895_mbox_base + 0x28), readl(e8895_mbox_base + 0x2c),
		e8895_speedy ? readl(e8895_speedy + E8895_IPB_STATE) : 0xdeadbeef);
}

/*
 * DEBUG (L7, 2026-10-06) stock register baseline. work/l7c/regsnap/regsnap.ko, loaded into the LOS
 * 4.4.302 kernel, read 1442 registers (every CMU SFR of the always-on CMU blocks, every address
 * the stock PMUCAL tables write outside power-gated blocks) at run time and in a kprobe on
 * exynos_pm_enter(), i.e. after every syscore op and right before stock's SLEEP SMC; stock then
 * slept and woke normally. exynos8895-regsnap.h holds both stock values per address.
 *   regdiff_now=1|2  print now every address whose value differs from stock's run-time (1) or
 *                    pre-SMC (2) value
 *   sleep_regdiff=1  the same against stock's pre-SMC values in ->enter, right before the SMC
 *                    (after sleep_hwacg/sleep_mbox_clr, before the watchdog is armed)
 * 4 differences per line (pa=ours/stock): the pstore console zone is 16K.
 */
#include "exynos8895-regsnap.h"

static bool sleep_regdiff;
static void __iomem *e8895_rs_va[ARRAY_SIZE(e8895_rs)];
static void e8895_rs_map(void);

static int e8895_rs_sleep_set(const char *val, const struct kernel_param *kp)
{
	int ret = param_set_bool(val, kp);

	if (!ret && sleep_regdiff)
		e8895_rs_map();				/* process context, before any suspend */
	return ret;
}
static const struct kernel_param_ops e8895_rs_sleep_ops = {
	.set = e8895_rs_sleep_set, .get = param_get_bool,
};
module_param_cb(sleep_regdiff, &e8895_rs_sleep_ops, &sleep_regdiff, 0644);

/*
 * Mapped on first use in process context (regdiff_now, or PM_SUSPEND_PREPARE with
 * sleep_regdiff), never at probe: image 0162 mapped the 82 pages in probe and never reached
 * userspace (stuck at the splash, no panic record), while 0161 without it boots.
 */
static bool e8895_rs_mapped;

static void e8895_rs_map(void)
{
	void __iomem *page = NULL;
	u32 cur = 0;
	int i;

	if (e8895_rs_mapped)
		return;
	for (i = 0; i < ARRAY_SIZE(e8895_rs); i++) {	/* sorted: one mapping per 4K page */
		u32 pg = e8895_rs[i].pa & ~0xfffu;

		if (!page || pg != cur) {
			page = ioremap(pg, SZ_4K);
			cur = pg;
		}
		e8895_rs_va[i] = page ? page + (e8895_rs[i].pa & 0xfff) : NULL;
	}
	e8895_rs_mapped = true;
	pr_info("exynos8895-pm: DEBUG rd: %zu registers mapped\n", ARRAY_SIZE(e8895_rs));
}

static void e8895_rs_diff(bool sus, const char *tag)
{
	char line[160];
	int i, len = 0, k = 0, n = 0, skipped = 0;

	if (!e8895_rs_mapped) {
		pr_info("exynos8895-pm: DEBUG rd %s: not mapped\n", tag);
		return;
	}
	for (i = 0; i < ARRAY_SIZE(e8895_rs); i++) {
		u32 v, exp = sus ? e8895_rs[i].sus : e8895_rs[i].rt;

		if (!e8895_rs_va[i]) {
			skipped++;
			continue;
		}
		v = readl(e8895_rs_va[i]);
		if (v == exp)
			continue;
		n++;
		len += scnprintf(line + len, sizeof(line) - len, " %08x=%08x/%08x",
				 e8895_rs[i].pa, v, exp);
		if (++k == 4) {
			pr_info("exynos8895-pm: DEBUG rd %s:%s\n", tag, line);
			len = k = 0;
		}
	}
	if (k)
		pr_info("exynos8895-pm: DEBUG rd %s:%s\n", tag, line);
	pr_info("exynos8895-pm: DEBUG rd %s: %d of %zu differ from stock %s (%d unmapped)\n",
		tag, n, ARRAY_SIZE(e8895_rs), sus ? "pre-SMC" : "run time", skipped);
}

/*
 * DEBUG sleep_stockcmu=<mask> (L7, 2026-10-06): right before the SMC, write stock's pre-SMC value
 * (exynos8895-regsnap.h .sus) into every always-on-CMU register of the selected classes that
 * differs from it; put the old values back right after the SMC returns.
 *   1  CLK_CON_GAT_*  stock 0x00200000 = MANUAL 0, ENABLE_AUTOMATIC_CLKGATING 0: the clock
 *                     passes (S:drivers/soc/samsung/cal-if/ra.c:378-417 ra_set_gate/ra_get_gate);
 *                     ours is MANUAL with many gates off, or HWACG with sleep_hwacg
 *   2  QCH_CON_*      stock 0x003f0000 = Q-channel off; ours 0x003f0003 (USI01-USI13)
 *   4  *_USER muxes   stock 0 = OSCCLK (PLL_MUX_SEL bit 4, S:.../cal-if/ra.h:33-34); ours 0x10
 * Image 0164 run "rdiff" (work/l7c/deep/rdiff): at our SMC these were the differences left
 * besides dividers of powered-off domains, PMU_DEBUG bit 8 and two big-core OPTION words.
 */
static unsigned int sleep_stockcmu;
module_param(sleep_stockcmu, uint, 0644);
static u32 e8895_sc_save[ARRAY_SIZE(e8895_rs)];
static DECLARE_BITMAP(e8895_sc_done, ARRAY_SIZE(e8895_rs));

static void e8895_stockcmu_set(void)
{
	int i, n[5] = { 0 };

	if (!sleep_stockcmu || !e8895_rs_mapped)
		return;
	for (i = 0; i < ARRAY_SIZE(e8895_rs); i++) {
		u32 v;

		if (!(e8895_rs[i].cls & sleep_stockcmu) || !e8895_rs_va[i])
			continue;
		v = readl(e8895_rs_va[i]);
		if (v == e8895_rs[i].sus)
			continue;
		e8895_sc_save[i] = v;
		writel(e8895_rs[i].sus, e8895_rs_va[i]);
		__set_bit(i, e8895_sc_done);
		n[e8895_rs[i].cls]++;
	}
	pr_info("exynos8895-pm: DEBUG stockcmu 0x%x: %d gates, %d qch, %d user muxes set to stock pre-SMC\n",
		sleep_stockcmu, n[1], n[2], n[4]);
}

static void e8895_stockcmu_restore(void)
{
	int i;

	for_each_set_bit(i, e8895_sc_done, ARRAY_SIZE(e8895_rs)) {
		writel(e8895_sc_save[i], e8895_rs_va[i]);
		__clear_bit(i, e8895_sc_done);
	}
}

static int e8895_rs_now_set(const char *val, const struct kernel_param *kp)
{
	unsigned int which;
	int ret = kstrtouint(val, 0, &which);

	if (ret)
		return ret;
	if (which != 1 && which != 2)
		return -EINVAL;
	e8895_rs_map();
	e8895_rs_diff(which == 2, which == 2 ? "now-vs-presmc" : "now-vs-runtime");
	return 0;
}
static const struct kernel_param_ops e8895_rs_now_ops = { .set = e8895_rs_now_set };
module_param_cb(regdiff_now, &e8895_rs_now_ops, NULL, 0200);

/* power_state of the SMC: sleep_bare forces C2 (0x00010000), otherwise the
 * stock SLEEP word unless sleep_psci overrides it */
static unsigned int e8895_smc_state(void)
{
	if (e8895_sicd_armed)					/* DEBUG sleep_sicd */
		return E8895_PSCI_SICD;
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
/* the PMU wake/idle masks before arming, restored on a refusal */
static u32 e8895_save_ipm[4], e8895_save_wk[4], e8895_save_eint;

/*
 * DEBUG (L7, 2026-10-05) sleep_late=1: arm the PMU for SLEEP (IDLE_IP masks, wake masks,
 * PMUCAL save + enter list) in ->enter right before the SMC instead of in this syscore
 * op. Stock's exynos_pm_syscore_suspend() ran after timekeeping's syscore op (stock log:
 * "mct_tick_stop (cpu0)" then "Enter Suspend scenario", work/l7c/stock44-suspend/
 * dmesg-real.txt:1427-1428), while ours runs first and the pinctrl, cpu_pm and clk syscore
 * ops still touch hardware after the PMU is armed (work/l7c/run-R1-0128/console.txt).
 */
static bool sleep_late;
module_param(sleep_late, bool, 0644);
static bool e8895_late;			/* the deferred arming is pending */

/* S:drivers/soc/samsung/exynos-powermode.c:638-663 exynos_prepare_sys_powerdown() */
static int e8895_sleep_arm(const char **why)
{
	struct e8895_tabs *t;
	unsigned int i;
	u32 arm0, arm1;		/* DEBUG sleep_noarm: pre-enter CENTRAL bits */

	if (sleep_aftr) {					/* DEBUG sleep_aftr */
		for (i = 0; i < 4; i++)
			pmu_raw_writel(~0u, E8895_IDLE_IP_MASK(i));
		pmu_raw_writel(exynos_pinctrl_get_eint_wake_mask(), E8895_EINT_WAKEUP_MASK);
		for (i = 0; i < 4; i++)
			pmu_raw_writel(e8895_wkmask_aftr[i], e8895_wkmask_off[i]);
		if (e8895_handle(e8895_enter_aftr, ARRAY_SIZE(e8895_enter_aftr))) {
			e8895_handle(e8895_earlywkup_aftr, ARRAY_SIZE(e8895_earlywkup_aftr));
			for (i = 0; i < 4; i++) {
				pmu_raw_writel(e8895_save_ipm[i], E8895_IDLE_IP_MASK(i));
				pmu_raw_writel(e8895_save_wk[i], e8895_wkmask_off[i]);
			}
			pmu_raw_writel(e8895_save_eint, E8895_EINT_WAKEUP_MASK);
			*why = "sleep_aftr: enter list failed";
			return -EIO;
		}
		e8895_aftr_armed = true;
		e8895_early = true;
		e8895_sleep_armed = true;
		pr_info("exynos8895-pm: DEBUG sleep_aftr: AFTR programmed instead of SLEEP mode %d\n",
			e8895_mode);
		return 0;
	}
	if (sleep_sicd) {					/* DEBUG sleep_sicd */
		int r;

		e8895_sicd_in_suspend = true;
		r = e8895_sicd_prepare();
		e8895_sicd_in_suspend = false;

		if (r) {
			*why = r == -EBUSY ? "sleep_sicd: SICD vetoed" : "sleep_sicd: SICD enter list failed";
			pr_info("exynos8895-pm: DEBUG sleep_sicd: refused (%d, last veto %s)\n", r,
				READ_ONCE(e8895_last_veto) ?: "-");
			for (i = 0; i < 4; i++) {
				pmu_raw_writel(e8895_save_ipm[i], E8895_IDLE_IP_MASK(i));
				pmu_raw_writel(e8895_save_wk[i], e8895_wkmask_off[i]);
			}
			pmu_raw_writel(e8895_save_eint, E8895_EINT_WAKEUP_MASK);
			return r;
		}
		e8895_sicd_armed = true;
		e8895_early = true;
		e8895_sleep_armed = true;
		pr_info("exynos8895-pm: DEBUG sleep_sicd: SICD programmed instead of SLEEP mode %d\n",
			e8895_mode);
		e8895_emul_apply();				/* DEBUG sleep_emul */
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
			pmu_raw_writel(e8895_save_ipm[i], E8895_IDLE_IP_MASK(i));
			pmu_raw_writel(e8895_save_wk[i], e8895_wkmask_off[i]);
		}
		pmu_raw_writel(e8895_save_eint, E8895_EINT_WAKEUP_MASK);
		*why = "enter list wait timed out";
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
	e8895_emul_apply();					/* DEBUG sleep_emul */
	e8895_ovr_apply(t->t[0], t->n[0]);			/* DEBUG sleep_ovr */
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
}

static int e8895_pm_syscore_suspend(void)
{
	const char *why;
	unsigned int i;
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
		e8895_save_ipm[i] = pmu_raw_readl(E8895_IDLE_IP_MASK(i));
		e8895_save_wk[i] = pmu_raw_readl(e8895_wkmask_off[i]);
	}
	e8895_save_eint = pmu_raw_readl(E8895_EINT_WAKEUP_MASK);

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

	if (sleep_late) {					/* DEBUG */
		e8895_late = true;
		pr_info("exynos8895-pm: DEBUG late: PMU program deferred to ->enter (mode %d)\n",
			e8895_mode);
		return 0;
	}
	ret = e8895_sleep_arm(&why);
	if (ret)
		goto refuse;
	e8895_apmlog_take("syscore");				/* DEBUG: pm_test=core reads it */
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

	e8895_late = false;			/* DEBUG sleep_late: ->enter never ran */
	if (!e8895_sleep_armed)			/* hibernation snapshot, or nothing programmed */
		return;
	if (sleep_trace)						/* DEBUG */
		pr_info("exynos8895-pm: DEBUG syscore resume early=%d mode=%d\n", e8895_early, e8895_mode);
	e8895_sleep_armed = false;
	e8895_ovr_restore();					/* DEBUG sleep_ovr */
	e8895_emul_restore();					/* DEBUG sleep_emul */
	if (e8895_aftr_armed) {					/* DEBUG sleep_aftr */
		unsigned int i;

		e8895_aftr_armed = false;
		if (e8895_early)
			e8895_handle(e8895_earlywkup_aftr, ARRAY_SIZE(e8895_earlywkup_aftr));
		else
			e8895_handle(e8895_exit_aftr, ARRAY_SIZE(e8895_exit_aftr));
		for (i = 0; i < 4; i++) {
			pmu_raw_writel(e8895_save_ipm[i], E8895_IDLE_IP_MASK(i));
			pmu_raw_writel(e8895_save_wk[i], e8895_wkmask_off[i]);
		}
		pmu_raw_writel(e8895_save_eint, E8895_EINT_WAKEUP_MASK);
		stat = pmu_raw_readl(E8895_WAKEUP_STAT);
		pr_info("exynos8895-pm: DEBUG sleep_aftr: back, %sWAKEUP_STAT=0x%08x\n",
			e8895_early ? "early " : "", stat);
		e8895_dbg_allon();
		return;
	}
	if (e8895_sicd_armed) {					/* DEBUG sleep_sicd */
		unsigned int i;

		e8895_sicd_armed = false;
		e8895_sicd_exit(e8895_early);
		for (i = 0; i < 4; i++) {
			pmu_raw_writel(e8895_save_ipm[i], E8895_IDLE_IP_MASK(i));
			pmu_raw_writel(e8895_save_wk[i], e8895_wkmask_off[i]);
		}
		pmu_raw_writel(e8895_save_eint, E8895_EINT_WAKEUP_MASK);
		stat = pmu_raw_readl(E8895_WAKEUP_STAT);
		pr_info("exynos8895-pm: DEBUG sleep_sicd: back, %sWAKEUP_STAT=0x%08x\n",
			e8895_early ? "early " : "", stat);
		e8895_dbg_allon();
		return;
	}
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

	if (e8895_late) {					/* DEBUG sleep_late */
		const char *why;

		e8895_late = false;
		ret = e8895_sleep_arm(&why);
		if (ret) {
			e8895_dbg_allon();
			pr_err("exynos8895-pm: SLEEP refused: %s (%d)\n", why, ret);
			return ret;
		}
	}
	if (sleep_trace)					/* DEBUG */
		pr_info("exynos8895-pm: DEBUG SMC power_state=0x%08x\n", e8895_smc_state());
	if (sleep_c2prep) {					/* DEBUG l7el3 */
		pr_info("exynos8895-pm: DEBUG c2prep: CONF ok=%d STATUS %x -> down\n",
			e8895_pmu_cpu_conf_ok(cpu), e8895_pmu_cpu_status(cpu));
		e8895_pmu_cpu_down(cpu);
		pr_info("exynos8895-pm: DEBUG c2prep: CONF ok=%d STATUS %x\n",
			e8895_pmu_cpu_conf_ok(cpu), e8895_pmu_cpu_status(cpu));
	}
	if (sleep_trace) {					/* DEBUG */
		e8895_hwacg_dump();
		e8895_apm_dump();
	}
	e8895_mbox_clr();					/* DEBUG sleep_mbox_clr */
	if (sleep_speedy)					/* DEBUG sleep_speedy */
		e8895_speedy_release("pre-SMC");
	e8895_hwacg_set();					/* DEBUG sleep_hwacg */
	e8895_stockcmu_set();					/* DEBUG sleep_stockcmu */
	if (sleep_regdiff)					/* DEBUG sleep_regdiff */
		e8895_rs_diff(true, "pre-SMC");
	e8895_apmlog_print();					/* DEBUG APM fwlog front at the SMC */
	e8895_apmlog_take("pre-SMC");
	e8895_wdt_arm();					/* DEBUG sleep_wdt */
	if (sleep_wfi && sleep_bare) {				/* DEBUG */
		cpu_do_idle();
		ret = 0;
	} else {
		ret = e8895_psci_suspend(e8895_smc_state());	/* DEBUG sleep_psci */
	}
	e8895_wdt_disarm();					/* DEBUG sleep_wdt */
	e8895_stockcmu_restore();				/* DEBUG sleep_stockcmu */
	e8895_hwacg_restore();					/* DEBUG sleep_hwacg */
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
	struct device_node *np;
	struct resource res;
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
	if (sleep_trace) {		/* DEBUG: what a forced reset after a SLEEP attempt left behind */
		pr_info("exynos8895-pm: DEBUG boot WAKEUP_STAT=%08x 0x640=%08x seq=%08x opt=%08x mif=%08x wk=%08x eint=%08x\n",
			pmu_raw_readl(E8895_WAKEUP_STAT), pmu_raw_readl(E8895_EXTRA_WAKEUP_STAT),
			pmu_raw_readl(0x0200), pmu_raw_readl(0x0208), pmu_raw_readl(0x0240),
			pmu_raw_readl(0x610), pmu_raw_readl(E8895_EINT_WAKEUP_MASK));
		/* DEBUG sleep_wdt: RST_STAT bit 24 = the reset came from the PERIS watchdog */
		pr_info("exynos8895-pm: DEBUG boot RST_STAT=%08x%s wdt 0x408=%08x 0x40c=%08x seqstat=%08x mifstat=%08x 0x500=%08x\n",
			pmu_raw_readl(E8895_RST_STAT),
			pmu_raw_readl(E8895_RST_STAT) & E8895_WDT_BIT ? " (watchdog reset)" : "",
			pmu_raw_readl(E8895_WDT_DISABLE), pmu_raw_readl(E8895_WDT_MASK_RESET),
			pmu_raw_readl(0x0204), pmu_raw_readl(0x0244), pmu_raw_readl(0x0500));
	}
	/* DEBUG sleep_wdt: the s3c2410_wdt driver owns the region, so no claim here */
	np = of_find_compatible_node(NULL, NULL, "samsung,exynos8895-wdt");
	if (np) {
		if (!of_address_to_resource(np, 0, &res))
			e8895_wdt_base = devm_ioremap(dev, res.start, resource_size(&res));
		of_node_put(np);
	}
	e8895_hwacg_map(dev);				/* DEBUG sleep_hwacg */
	e8895_speedy = devm_ioremap(dev, E8895_SPEEDY_AP, SZ_4K);	/* DEBUG sleep_speedy, no driver owns it */
	e8895_apmlog = devm_ioremap(dev, 0x16502000, SZ_4K);	/* DEBUG APM fwlog ring indices */
	register_pm_notifier(&e8895_sicd_dbg_pm_nb);	/* DEBUG sicd_wdt/sicd_hwacg */
	/* DEBUG e8895_apm_dump(): the mailbox driver owns the region, so no claim here */
	np = of_find_compatible_node(NULL, NULL, "samsung,exynos8895-mbox");
	if (np) {
		if (!of_address_to_resource(np, 0, &res))
			e8895_mbox_base = devm_ioremap(dev, res.start, resource_size(&res));
		of_node_put(np);
	}
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
