// SPDX-License-Identifier: GPL-2.0+
/*
 * Maxim MAX77865 charger
 *
 * Charger page at I2C address 0x69: the "max77865-charger", "ac" and "usb"
 * supplies, the USB OTG VBUS boost and the restart into LP charging when the
 * system powers off with a charger online. With charge control, the vendor
 * charger initialisation and charge policy (thermal, swelling, full, recharge,
 * safety timer, input classes) as the vendor sec_battery driver ran them.
 */

#include <linux/alarmtimer.h>
#include <linux/bitfield.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/devm-helpers.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/mfd/max77693-common.h>
#include <linux/mfd/max77865-private.h>
#include <linux/mfd/syscon.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/pm_wakeup.h>
#include <linux/power_supply.h>
#include <linux/reboot.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/seq_file.h>
#include <linux/string.h>
#include <linux/thermal.h>
#include <linux/timekeeping.h>
#include <linux/workqueue.h>
#include <linux/usb/s2mm005.h>
#include <linux/power/idt_mfc_wpc.h>
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_AFC)
#include <linux/power/max77865_muic_hv.h>
#endif

/* float voltage: CNFG_04 CV = (V[0.1 mV] - 40500) / 125 */
#define CV_CODE_NORMAL		0x18	/* 4350 mV */
#define CV_CODE_SWELLING	0x08	/* 4150 mV */
#define CV_CODE_TO_MV(c)	(((c) * 125 + 40500) / 10)
#define FLOAT_MV		4350
#define SWELLING_DROP_MV	4150

/* battery temperature thresholds, 0.1 degC, the same in LP charging */
#define T_HIGHLIMIT		800	/* USB connector NTC */
#define T_HIGHLIMIT_REC		750
#define T_HIGH			500
#define T_HIGH_REC		450
#define T_LOW			0
#define T_LOW_REC		50

/* wireless temperature thresholds, 0.1 degC */
#define WPC_T_HIGH		450
#define WPC_T_HIGH_REC		400
#define WPC_T_LOW		0
#define WPC_T_LOW_REC		50
#define WPC_BAT_TEMP_LIMIT	380
#define WPC_BAT_TEMP_REC	360
#define WPC_BAT_TEMP_CURRENT	600	/* mA */
#define WPC_SW_CURRENT		750	/* mA */

/* swelling and mix-temperature thresholds, 0.1 degC */
#define SW_HIGH_BLOCK		410
#define SW_HIGH_REC		390
#define SW_LOW_BLOCK_1ST	150
#define SW_LOW_REC_1ST		200
#define SW_LOW_BLOCK_2ND	50
#define SW_LOW_REC_2ND		100
#define MIX_HIGH_TEMP		420
#define MIX_HIGH_CHG_TEMP	500
#define MIX_HIGH_TEMP_REC	390
#define T_DEFAULT		250	/* a missing USB or charger NTC, or ntc_required=N */

/* currents, mA */
#define SW_LOW_CURRENT		750
#define SW_LOW_TOPOFF		200
#define SW_HIGH_CURRENT		1150
#define SW_HIGH_TOPOFF		200
#define TOPOFF_1ST		300
#define TOPOFF_2ND		200

/* input limit while both the battery and the charger are hot */
static inline int max77865_mix_input_ma(int in_mv)
{
	int v = in_mv / 1000;

	if (v <= 0)
		v = 5;
	return min((int)(TOPOFF_1ST + 50),
		   (int)(FLOAT_MV * (TOPOFF_1ST + 50) / (v * 90) / 10));
}

/* charger NTC limit of fast-charging inputs: 0.1 degC, mA */
#define CHG_HIGH_TEMP		490
#define CHG_HIGH_TEMP_REC	470
#define CHG_HIGH_TEMP_HV_IN	1000
#define CHG_HIGH_TEMP_HV_CC	1500
#define CHG_HIGH_TEMP_DEFAULT_IN 1800
#define CHG_HIGH_TEMP_DEFAULT_CC 2100
#define CHG_HIGH_TEMP_PWR_THR	14500	/* mW */

#if IS_ENABLED(CONFIG_CHARGER_MAX77865_AFC)
/* input step before AFC: mA, ms */
#define PRE_AFC_IN		1000
#define PRE_AFC_DELAY_MS	2000
#define PRE_AFC_WC_IN		500
#define PRE_AFC_WC_DELAY_MS	4000
#endif

/* full / recharge, mV and % */
#define FULL_COND_VCELL		4250
#define FULL_COND_SOC		93
#define RECHG_VCELL		4280
#define SW_RECHG_MV		4000

#if IS_ENABLED(CONFIG_CHARGER_MAX77865_AGE_FORECAST)
struct max77865_age_data {
	int cycle;
	int float_mv_tenth;
	int recharge_vcell;
	int full_vcell;
	int full_soc;
};

/* cycle count, float voltage in 0.1 mV, recharge and full voltage, full SOC */
static const struct max77865_age_data max77865_age_data[] = {
	{    0, 43500, 4280, 4250, 93 },
	{  300, 43250, 4255, 4225, 92 },
	{  400, 43125, 4242, 4212, 91 },
	{  700, 42875, 4217, 4187, 90 },
	{ 1000, 42375, 4167, 4137, 89 },
};

#define MAX77865_RECHG_VCELL(chg)	((chg)->recharge_condition_vcell)
#define MAX77865_FULL_VCELL(chg)	((chg)->full_condition_vcell)
#define MAX77865_FULL_SOC(chg)		((chg)->full_condition_soc)
#else
#define MAX77865_RECHG_VCELL(chg)	RECHG_VCELL
#define MAX77865_FULL_VCELL(chg)	FULL_COND_VCELL
#define MAX77865_FULL_SOC(chg)		FULL_COND_SOC
#endif

/* PMU registers for the restart into LP charging */
#define EXYNOS8895_PMU_SWRESET		0x0400
#define EXYNOS8895_PMU_INFORM2		0x0808
/* the bootloader starts LP charging after a reset with this in INFORM2 */
#define SBOOT_INFORM2_POWER_OFF		0x0

/* safety timer */
#define EXPIRED_MS		11160000U
#define RECHG_EXPIRED_MS	5400000U
#define STANDARD_CURR		2350	/* mA the safety time is scaled to */

/* scheduling */
#define PASS_PERIOD		(10 * HZ)
#define ALARM_PERIOD_S		30		/* wakes a suspended system while plugged */
#define CABLE_RETRY		HZ
#define CABLE_RETRIES		5

/*
 * The S2MM005's PD sink status is read for the HV guard, Type-C Rp and PD (PD
 * depends on TYPEC_RP).
 */
#define MAX77865_CCIC	(IS_ENABLED(CONFIG_CHARGER_MAX77865_HV_GUARD) || \
			 IS_ENABLED(CONFIG_CHARGER_MAX77865_TYPEC_RP))

enum max77865_input { INPUT_NONE, INPUT_WIRED, INPUT_WCIN };

enum max77865_cable {
	CABLE_NONE,
	CABLE_SDP,
	CABLE_CDP,
	CABLE_DCP,
	CABLE_UNKNOWN,
	CABLE_AFC_PREPARE,
	CABLE_HV9,
	CABLE_PD,
	WLS_NV,
	WLS_PACK,
	WLS_HV,
	WLS_PREP_HV,
};

/* top-off stage */
enum max77865_chg_stage { STAGE_NONE, STAGE_1ST, STAGE_2ND };

enum max77865_mode { MODE_CHARGING, MODE_CHARGING_OFF, MODE_BUCK_OFF };

/* battery health as the policy keeps it; the values are reported in debugfs */
enum max77865_health {
	H_GOOD, H_OVERHEAT, H_COLD, H_OVERHEATLIMIT, H_OVERVOLTAGE, H_UNDERVOLTAGE, H_SAFETY_TIMER,
	H_VSYS_OVP, H_VBAT_OVP,
};

enum max77865_swelling { SWELLING_NONE, SWELLING_CHARGING, SWELLING_FULL };

/* current limit events */
#define EV_LOW_TEMP_SWELLING	BIT(0)
#define EV_HIGH_TEMP_SWELLING	BIT(1)
#define EV_LOW_TEMP		BIT(2)
#define EV_SWELLING_MODE	(EV_LOW_TEMP_SWELLING | EV_LOW_TEMP | EV_HIGH_TEMP_SWELLING)

static bool ntc_required = true;
module_param(ntc_required, bool, 0644);
MODULE_PARM_DESC(ntc_required, "charge only with a valid battery-ntc reading");

static const char * const health_name[] = {
	[H_GOOD] = "good", [H_OVERHEAT] = "overheat", [H_COLD] = "cold",
	[H_OVERHEATLIMIT] = "overheatlimit", [H_OVERVOLTAGE] = "overvoltage",
	[H_UNDERVOLTAGE] = "undervoltage", [H_SAFETY_TIMER] = "safety-timer-expire",
	[H_VSYS_OVP] = "vsys-ovp", [H_VBAT_OVP] = "vbat-ovp",
};

static const char * const mode_name[] = {
	[MODE_CHARGING] = "charging", [MODE_CHARGING_OFF] = "charging-off",
	[MODE_BUCK_OFF] = "buck-off",
};

static const char * const cable_name[] = {
	[CABLE_NONE] = "none", [CABLE_SDP] = "sdp", [CABLE_CDP] = "cdp",
	[CABLE_DCP] = "dcp", [CABLE_UNKNOWN] = "unknown",
	[CABLE_AFC_PREPARE] = "afc-prepare",
	[CABLE_HV9] = "hv9",
	[CABLE_PD] = "pd",
	[WLS_NV] = "wls-nv",
	[WLS_PACK] = "wls-pack",
	[WLS_HV] = "wls-hv",
	[WLS_PREP_HV] = "wls-prep-hv",
};

/* the readings of one pass */
struct max77865_sample {
	unsigned int int_ok, dtls0, dtls1, cnfg00;
	enum max77865_input input;
	enum max77865_cable cable;
	bool ntc_valid, ntc_escape;
	int t, t_mc, tu, tc;		/* 0.1 degC; t_mc milli-degC */
	int vcell, cap;			/* mV, %; -1 = unknown */
	bool iavg_valid;
	int iavg;			/* uA */
	int hc;
	struct {
		bool online;
		bool fod;		/* the pad reported a foreign object */
		int pad;		/* enum idt_mfc_pad */
		enum max77865_cable class;
		int vout_mv;
	} wls;
	int hv_mv;			/* AFC/QC voltage in mV, 0 = none */
	int pd_mv, pd_ma;		/* USB-PD contract voltage and current */
	bool pd_usb;			/* USB data partner present during PD */
	int rp;				/* Type-C Rp level */
	bool aicl_ok;			/* CHG_INT_OK bit 7 */
};

/* input voltage of the cable class, in mV */
static int max77865_in_mv(const struct max77865_sample *s)
{
	switch (s->cable) {
	case CABLE_HV9:
		return 9000;
	case CABLE_PD:
		return s->pd_mv ? s->pd_mv : 5000;
	case WLS_HV:
	case WLS_PREP_HV:
		/* 10 V pads and their preparation; a battery pack TA stays at 5 V */
		return s->wls.pad == IDT_MFC_PAD_WPC_PACK_TA ? 5000 : 10000;
	default:
		return 5000;
	}
}

struct max77865_charger {
	struct device *dev;
	struct i2c_client *client;
	struct regmap *regmap;
	struct regmap *pmu;
	struct power_supply *psy_chg, *psy_ac, *psy_usb;
	struct power_supply *fg;
	struct regulator_dev *otg_rdev;
	struct gpio_desc *jig;
	const char *tz_name;
	struct mutex lock;		/* policy state and the OTG sequence */
	struct delayed_work pass_dwork;
	struct work_struct chgin_work;
	struct alarm alarm;

	bool charge_control;
	bool works_ready;
	bool lpcharge;
	bool stopping;
	bool poweroff_restart;

	/* policy state, under lock */
	enum max77865_input input;
	enum max77865_cable cable;
	int cable_tries;
	u32 mode;			/* enum max77865_chg_stage */
	u32 status;			/* POWER_SUPPLY_STATUS_* */
	u32 health;			/* enum max77865_health */
	u32 hc;				/* health decoded from the charger */
	u32 swelling;
	unsigned int events;
	bool recharging;
	bool mix_limit;
	u32 chg_mode;			/* enum max77865_mode */
	bool chg_block;
	u64 chg_start_s;		/* 0 = not charging */
	u32 budget_ms;			/* safety time left */
	u64 prev_safety_s;
	int dis_cnt;
	bool ntc_valid;
	bool ntc_seen;			/* start-up gate: one reading with -200 < T < 900 */
	u32 fault;			/* 0 none, 1 charge-off fault, 2 kick stopped */
	bool otg_on;
	unsigned int otg_saved_mask;
	unsigned int otg_saved_cnfg00, otg_saved_cnfg12;
	bool wdt_kick;
	bool hv_guard_logged;
	bool usb_ntc_logged, chg_ntc_logged, no_rtc_logged;
	bool wake_held;
	unsigned long last_log;
	bool log_armed;

	/* last written values; -1 = unknown */
	int hw_mode, in_w, chg_w, topoff_w, cv_w, cv_want;
	bool chginsel_pending;
	int in_ma, chg_ma;		/* this pass's policy currents */
	bool wpc_otg, wpc_wired;	/* the receiver's RX stays off while set */

	/* reported values (change detection) */
	int rep_online, rep_status, rep_health, rep_ac, rep_usb, rep_ac_ma, rep_usb_ma;
	int rep_ac_mv;
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_TTF)
	int rep_ttf_ma;
#endif

	/* battery over-voltage stop */
	int vbat_cnt;
	bool vbat_ovp_done;

#if IS_ENABLED(CONFIG_CHARGER_MAX77865_AICL)
	struct delayed_work aicl_dwork;
#endif
	u32 aicl_ma;
	u32 max_power;
	u32 ttf_ma;

#if IS_ENABLED(CONFIG_CHARGER_MAX77865_SYSOVLO)
	int sysovlo_irq;
	bool sysovlo_armed;
	bool sysovlo;
#endif

#if IS_ENABLED(CONFIG_CHARGER_MAX77865_AGE_FORECAST)
	u32 age_step;
	int cv_normal;			/* CV code of the current age step */
	int recharge_condition_vcell;
	int full_condition_vcell;
	int full_condition_soc;
	bool age_inited;
#endif

#if IS_ENABLED(CONFIG_CHARGER_MAX77865_AFC) || IS_ENABLED(CONFIG_CHARGER_MAX77865_QC) || \
	IS_ENABLED(CONFIG_CHARGER_MAX77865_TYPEC_RP) || IS_ENABLED(CONFIG_CHARGER_MAX77865_PD)
	bool chg_high_temp_limit;
#endif
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_AFC)
	struct notifier_block hv_nb;
	int hv_event;			/* enum max77865_hv_event, NONE when detached */
	bool hv_ready_sent;		/* max77865_hv_charger_ready() called once */
	unsigned long pre_afc_until;	/* pre-AFC input step deadline (jiffies), 0 = none */
	u32 hv_mv_dbg;			/* last HV voltage in mV, for debugfs */
#endif

#if IS_ENABLED(CONFIG_WIRELESS_CHARGER_IDT_MFC_NV)
	struct delayed_work wc_current_dwork;
	int wc_target_ma;
	int wc_pre_ma;
	int wc_cap;			/* capacity and pad of the last pass, for the ramp's end */
	int wc_pad;
	bool wc_ramp_pending;
	bool wpc_temp_limit;
	bool wpc_cv_mode;
#endif
#if IS_ENABLED(CONFIG_WIRELESS_CHARGER_IDT_MFC_HV)
	bool wpc_hv_granted;		/* the 10 V grant given to the receiver */
	int wls_vout_mv;		/* last requested Vout seen, for the AICL reset */
#endif

	/* debugfs overrides */
	u32 override_usb_mc, override_chg_mc, override_t_mc, override_hc;

#if MAX77865_CCIC
	/* the CCIC's last PD sink status, under lock */
	struct notifier_block ccic_nb;
	struct s2mm005_pd_status pd;
	unsigned long pd_event;		/* enum s2mm005_pd_event of the last notification */
#endif
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_USB_SS)
	bool usb_ss;			/* a SuperSpeed host configured 900 mA */
#endif
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_PD)
	/* USB PD source list and contract, under lock */
	bool pd_new;			/* a notification the pass has not handled */
	bool pd_attached;
	bool pd_ready;			/* the requested contract is in place */
	bool pd_limit;			/* charger NTC limit */
	int pd_mv[S2MM005_MAX_PDO_NUM + 1];	/* power_list after the 15 W and 3 A clamps */
	int pd_ma[S2MM005_MAX_PDO_NUM + 1];
	struct {
		int mv, ma, pdo;
	} pd_list[S2MM005_MAX_PDO_NUM];
	int pd_count;
	int pd_index;			/* -1 below the list (thermal) */
	int pd_target;			/* list index the policy wants */
	int pd_cap_ma;			/* input cap of this pass, 0 = none */
	int pd_want;			/* PDO this pass asks for, 0 = none */
	int pd_req;			/* last select_pdo() argument */
#endif
};

static const struct regmap_range max77865_chg_rd_ranges[] = {
	regmap_reg_range(MAX77865_CHG_REG_INT, MAX77865_CHG_REG_SAFEOUT_CTRL),
};

static const struct regmap_range max77865_chg_wr_ranges[] = {
	regmap_reg_range(MAX77865_CHG_REG_INT_MASK, MAX77865_CHG_REG_INT_MASK),
	regmap_reg_range(MAX77865_CHG_REG_CNFG_00, MAX77865_CHG_REG_SAFEOUT_CTRL),
};

static const struct regmap_range max77865_chg_volatile_ranges[] = {
	/* INT .. DTLS_03 */
	regmap_reg_range(MAX77865_CHG_REG_INT, 0xB6),
};

/*
 * CHG_INT clears on read: keep it out of a debugfs register dump, which would
 * eat a pending CHGIN_I and delay the attach/detach pass to the next 10 s tick.
 */
static const struct regmap_range max77865_chg_precious_ranges[] = {
	regmap_reg_range(MAX77865_CHG_REG_INT, MAX77865_CHG_REG_INT),
};

static const struct regmap_access_table max77865_chg_precious_table = {
	.yes_ranges = max77865_chg_precious_ranges,
	.n_yes_ranges = ARRAY_SIZE(max77865_chg_precious_ranges),
};

static const struct regmap_access_table max77865_chg_rd_table = {
	.yes_ranges = max77865_chg_rd_ranges,
	.n_yes_ranges = ARRAY_SIZE(max77865_chg_rd_ranges),
};

static const struct regmap_access_table max77865_chg_wr_table = {
	.yes_ranges = max77865_chg_wr_ranges,
	.n_yes_ranges = ARRAY_SIZE(max77865_chg_wr_ranges),
};

static const struct regmap_access_table max77865_chg_volatile_table = {
	.yes_ranges = max77865_chg_volatile_ranges,
	.n_yes_ranges = ARRAY_SIZE(max77865_chg_volatile_ranges),
};

static const struct regmap_config max77865_chg_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = MAX77865_CHG_REG_SAFEOUT_CTRL,
	.rd_table = &max77865_chg_rd_table,
	.wr_table = &max77865_chg_wr_table,
	.volatile_table = &max77865_chg_volatile_table,
	.precious_table = &max77865_chg_precious_table,
	.cache_type = REGCACHE_NONE,
};

#if IS_ENABLED(CONFIG_WIRELESS_CHARGER_IDT_MFC_NV)
/* the receiver's RX enable follows this; set once probe has initialised the charger */
static struct max77865_charger *g_max77865_charger;

bool max77865_charger_wpc_allowed(void)
{
	struct max77865_charger *chg = READ_ONCE(g_max77865_charger);

	return chg && chg->charge_control && !READ_ONCE(chg->fault) &&
	       !READ_ONCE(chg->wpc_otg) && !READ_ONCE(chg->wpc_wired);
}
EXPORT_SYMBOL_GPL(max77865_charger_wpc_allowed);
#endif

/*
 * The wireless receiver stays off while OTG boosts and while a wired source of
 * 4 W or more is attached, as in the vendor driver. Called under the lock.
 */
static void max77865_wpc_gate(struct max77865_charger *chg, bool otg, bool wired)
{
#if IS_ENABLED(CONFIG_WIRELESS_CHARGER_IDT_MFC_NV)
	if (otg == chg->wpc_otg && wired == chg->wpc_wired)
		return;
	WRITE_ONCE(chg->wpc_otg, otg);
	WRITE_ONCE(chg->wpc_wired, wired);
	idt_mfc_wpc_rx_update();
#endif
}

/* DETAILS_01[3:0] */
static int max77865_dtls_status(unsigned int dtls1)
{
	switch (FIELD_GET(MAX77865_CHG_DTLS, dtls1)) {
	case 0x0: case 0x1: case 0x2:
		return POWER_SUPPLY_STATUS_CHARGING;
	case 0x3: case 0x4:
		return POWER_SUPPLY_STATUS_FULL;
	case 0x5: case 0x6: case 0x7:
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
	case 0x8: case 0xA: case 0xB:
		return POWER_SUPPLY_STATUS_DISCHARGING;
	default:
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}
}

/*
 * Health from the charger: OVERVOLTAGE when the input DTLS reads 2; UNDERVOLTAGE
 * when it reads 0/1 with CHG_DTLS bit 3, BUCK and CHG on, wired, and kept while
 * the input reads 0/1. WCIN uses WCIN_DTLS and is never UNDERVOLTAGE.
 */
static int max77865_decode_hc(const struct max77865_sample *s, int h)
{
	bool wcin = s->input == INPUT_WCIN;
	unsigned int vbus = wcin ? FIELD_GET(MAX77865_WCIN_DTLS, s->dtls0) :
				   FIELD_GET(MAX77865_CHGIN_DTLS, s->dtls0);

	if (s->input == INPUT_NONE)
		return H_GOOD;
	if (vbus == 0x02)
		return H_OVERVOLTAGE;
	if (vbus <= 0x01 && !wcin && (FIELD_GET(MAX77865_CHG_DTLS, s->dtls1) & 0x08) &&
	    (s->cnfg00 & MAX77865_CNFG_00_BUCK) && (s->cnfg00 & MAX77865_CNFG_00_CHG))
		return H_UNDERVOLTAGE;
	if (h == H_UNDERVOLTAGE && vbus <= 0x01 && !wcin)
		return H_UNDERVOLTAGE;
	return H_GOOD;
}

/* WDTCLR kick: not once switching charging off failed, not with debugfs wdt_kick = N */
static void max77865_kick(struct max77865_charger *chg, bool wdten)
{
	if (chg->fault == 2 || !chg->wdt_kick || !wdten)
		return;
	regmap_update_bits(chg->regmap, MAX77865_CHG_REG_CNFG_06, MAX77865_CNFG_06_WDTCLR,
			   MAX77865_CNFG_06_WDTCLR);
}

/* buck, then charger enable, as the vendor driver; DISSKIP is not touched on WCIN */
static void max77865_write_mode(struct max77865_charger *chg, int mode)
{
	bool on = mode == MODE_CHARGING;
	int ret;

	ret = regmap_update_bits(chg->regmap, MAX77865_CHG_REG_CNFG_00, MAX77865_CNFG_00_BUCK,
				 mode == MODE_BUCK_OFF ? 0 : MAX77865_CNFG_00_BUCK);
	ret = ret ?: regmap_update_bits(chg->regmap, MAX77865_CHG_REG_CNFG_00,
					MAX77865_CNFG_00_CHG, on ? MAX77865_CNFG_00_CHG : 0);
	if (!ret && chg->input != INPUT_WCIN)
		ret = regmap_update_bits(chg->regmap, MAX77865_CHG_REG_CNFG_12,
					 MAX77865_CNFG_12_DISSKIP,
					 on ? MAX77865_CNFG_12_DISSKIP : 0);
	chg->hw_mode = ret ? -1 : mode;
}

/* the mode the apply step writes, and the safety timer start */
static void max77865_set_mode(struct max77865_charger *chg, int mode)
{
	chg->chg_mode = mode;
	if (mode == MODE_CHARGING) {
		if (!chg->chg_start_s)
			chg->chg_start_s = max_t(u64, ktime_get_boottime_seconds(), 1);
		chg->chg_block = false;
	} else {
		chg->chg_start_s = 0;
		chg->chg_block = true;
	}
}

static void max77865_set_events(struct max77865_charger *chg, unsigned int val, unsigned int mask)
{
	chg->events = (chg->events & ~mask) | val;
}

static void max77865_reset_budget(struct max77865_charger *chg, u32 ms)
{
	chg->budget_ms = ms;
	chg->prev_safety_s = 0;
}

/* the vendor safety time, scaled by the charge current, without its screen-on pause */
static void max77865_safety_time(struct max77865_charger *chg, const struct max77865_sample *s)
{
	u64 now = ktime_get_boottime_seconds();
	u64 expired = chg->budget_ms;
	s64 input_power, charging_power;
	int curr, in = chg->input == INPUT_WCIN ? chg->chg_ma : chg->in_w;

	if (s->iavg_valid && s->iavg < 0)
		chg->dis_cnt++;
	else
		chg->dis_cnt = 0;
	if (chg->dis_cnt >= 5) {
		max77865_reset_budget(chg, EXPIRED_MS);
		chg->dis_cnt = 0;
		return;
	}

	if (!chg->prev_safety_s)
		chg->prev_safety_s = now;
	if (in <= 0 || chg->chg_w <= 0)
		return;

	input_power = (s64)in * (max77865_in_mv(s) / 1000) * 1000;
	charging_power = (s64)chg->chg_w * FLOAT_MV;
	if (input_power > charging_power)
		curr = chg->chg_w;
	else
		curr = div_s64(input_power, FLOAT_MV) * 9 / 10;
	if (!curr)
		return;

	expired = div_u64(expired * STANDARD_CURR, curr);
	if (expired <= (now - chg->prev_safety_s) * 1000)
		expired = 0;
	else
		expired -= (now - chg->prev_safety_s) * 1000;
	chg->budget_ms = div_u64(expired * curr, STANDARD_CURR);
	chg->prev_safety_s = now;
}

#if IS_ENABLED(CONFIG_WIRELESS_CHARGER_IDT_MFC_NV)
#define WC_CURRENT_STEP		100
#define WC_CURRENT_START	480
#define WC_CURRENT_WORK_STEP	1000	/* ms */

/*
 * The WCIN limit (CNFG_10 = mA / 25, at most 1575 mA) moves 100 mA a second
 * toward the target; at the target the receiver gets the headroom of the class.
 */
static void max77865_wc_current_work(struct work_struct *work)
{
	struct max77865_charger *chg = container_of(to_delayed_work(work), struct max77865_charger,
						    wc_current_dwork);
	int headroom, cur;

	mutex_lock(&chg->lock);
	if (!chg->charge_control || chg->fault || chg->input != INPUT_WCIN || chg->stopping) {
		chg->wc_ramp_pending = false;
		mutex_unlock(&chg->lock);
		return;
	}
	cur = chg->wc_pre_ma;
	if (cur != chg->wc_target_ma) {
		if (cur > chg->wc_target_ma)
			cur -= min(cur - chg->wc_target_ma, WC_CURRENT_STEP);
		else
			cur += min(chg->wc_target_ma - cur, WC_CURRENT_STEP);
		if (!regmap_update_bits(chg->regmap, MAX77865_CHG_REG_CNFG_10,
					MAX77865_CNFG_10_WCIN_LIM, min(cur, 1575) / 25))
			chg->in_w = cur;
		chg->wc_pre_ma = cur;
		queue_delayed_work(system_freezable_wq, &chg->wc_current_dwork,
				   msecs_to_jiffies(WC_CURRENT_WORK_STEP));
		mutex_unlock(&chg->lock);
		return;
	}
	chg->wc_ramp_pending = false;
	/* 5 V pads: headroom 4 below 85 %, else 5; 10 V pads: 5 */
	if (chg->cable == WLS_HV && chg->wc_pad != IDT_MFC_PAD_WPC_PACK_TA)
		headroom = 5;
	else
		headroom = chg->wc_cap >= 0 && chg->wc_cap < 85 ? 4 : 5;
	mutex_unlock(&chg->lock);
	msleep(500);
	idt_mfc_wpc_set_headroom(headroom);
}
#endif

/* health from the battery (@t) and USB connector (@tu) temperatures */
static int max77865_thermal_h(int h, int t, int tu, bool wcin)
{
	int t_high = wcin ? WPC_T_HIGH : T_HIGH;
	int t_high_rec = wcin ? WPC_T_HIGH_REC : T_HIGH_REC;
	int t_low = wcin ? WPC_T_LOW : T_LOW;
	int t_low_rec = wcin ? WPC_T_LOW_REC : T_LOW_REC;

	if (tu >= T_HIGHLIMIT)
		return H_OVERHEATLIMIT;
	if (h == H_OVERHEATLIMIT && tu > T_HIGHLIMIT_REC)
		return h;
	if (t >= t_high)
		return H_OVERHEAT;
	if (t <= t_high_rec && t >= t_low_rec) {
		if (h == H_OVERHEAT || h == H_OVERHEATLIMIT || h == H_COLD)
			return (h == H_OVERHEATLIMIT && t > t_high_rec) ? H_OVERHEAT : H_GOOD;
		return h;
	}
	if (t <= t_low)
		return H_COLD;
	return h;
}

#if IS_ENABLED(CONFIG_CHARGER_MAX77865_AGE_FORECAST)
/* float, recharge and full conditions by battery cycle count */
static void max77865_aging_check(struct max77865_charger *chg, const struct max77865_sample *s)
{
	union power_supply_propval val;
	int cycle, step;
	const struct max77865_age_data *row;
	u8 cv_code;

	if (!chg->fg || s->t < 50)
		return;

	if (power_supply_get_property(chg->fg, POWER_SUPPLY_PROP_CYCLE_COUNT, &val) ||
	    val.intval < 0)
		return;

	cycle = val.intval;
	for (step = ARRAY_SIZE(max77865_age_data) - 1; step >= 0; step--) {
		if (max77865_age_data[step].cycle <= cycle)
			break;
	}
	if (step < 0)
		step = 0;

	if (chg->age_inited && step == chg->age_step)
		return;

	chg->age_step = step;
	row = &max77865_age_data[step];
	chg->recharge_condition_vcell = row->recharge_vcell;
	chg->full_condition_vcell = row->full_vcell;
	chg->full_condition_soc = row->full_soc;
	cv_code = (row->float_mv_tenth - 40500) / 125;

	/*
	 * every later return to the normal float (detach, swelling end, thermal
	 * recovery) uses this step's; the CV itself changes only outside swelling
	 */
	chg->cv_normal = cv_code;
	if (chg->swelling == SWELLING_NONE)
		chg->cv_want = cv_code;

	dev_info(chg->dev, "age step %d (cycle %d) float %d mV\n",
		 step, cycle, row->float_mv_tenth / 10);
}
#endif

/* the normal (non-swelling) float */
static int max77865_cv_normal(struct max77865_charger *chg)
{
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_AGE_FORECAST)
	return chg->cv_normal;
#else
	return CV_CODE_NORMAL;
#endif
}

/* the charger's end of charge: a second top-off cycle at 200 mA, then off */
static void max77865_do_full(struct max77865_charger *chg, const struct max77865_sample *s)
{
	chg->status = POWER_SUPPLY_STATUS_FULL;
	if (chg->mode == STAGE_1ST) {
		/* off now; the apply step switches on again after the new currents */
		chg->mode = STAGE_2ND;
		if (chg->hw_mode == MODE_CHARGING)
			max77865_write_mode(chg, MODE_CHARGING_OFF);
		max77865_set_mode(chg, MODE_CHARGING);
		dev_dbg(chg->dev, "full (1st), vcell %d mV\n", s->vcell);
	} else {
		chg->mode = STAGE_NONE;
		chg->recharging = false;
		max77865_set_mode(chg, MODE_CHARGING_OFF);
		dev_dbg(chg->dev, "full (2nd), vcell %d mV\n", s->vcell);
	}
#if IS_ENABLED(CONFIG_WIRELESS_CHARGER_IDT_MFC_NV)
	if (s->input == INPUT_WCIN)
		idt_mfc_wpc_full();
#endif
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_AGE_FORECAST)
	max77865_aging_check(chg, s);
#endif
}

static void max77865_log_thermal(struct max77865_charger *chg, int pre,
				 const struct max77865_sample *s)
{
	dev_info(chg->dev, "thermal %s -> %s, T %d Tu %d, mode %s\n", health_name[pre],
		 health_name[chg->health], s->t, s->tu, mode_name[chg->chg_mode]);
}

static void max77865_log_swelling(struct max77865_charger *chg, const char *what,
				  const struct max77865_sample *s)
{
	dev_info(chg->dev, "swelling %s, T %d vcell %d mV, mode %s\n", what, s->t, s->vcell,
		 mode_name[chg->chg_mode]);
}

#if IS_ENABLED(CONFIG_CHARGER_MAX77865_PD)
static void max77865_pd_reset(struct max77865_charger *chg);
#endif

/* cable attach and detach */
static void max77865_cable_event(struct max77865_charger *chg, bool attach)
{
	chg->recharging = false;
	chg->health = H_GOOD;
	chg->dis_cnt = 0;
	chg->hv_guard_logged = false;
	chg->vbat_cnt = 0;
	chg->vbat_ovp_done = false;
	chg->aicl_ma = 0;
	chg->max_power = 0;
	chg->ttf_ma = 0;
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_AFC) || IS_ENABLED(CONFIG_CHARGER_MAX77865_QC) || \
	IS_ENABLED(CONFIG_CHARGER_MAX77865_TYPEC_RP) || IS_ENABLED(CONFIG_CHARGER_MAX77865_PD)
	chg->chg_high_temp_limit = false;
#endif
	max77865_reset_budget(chg, EXPIRED_MS);
	if (attach) {
#if IS_ENABLED(CONFIG_WIRELESS_CHARGER_IDT_MFC_NV)
		chg->wpc_temp_limit = false;
		chg->wpc_cv_mode = false;
#endif

#if IS_ENABLED(CONFIG_CHARGER_MAX77865_SYSOVLO)
		chg->sysovlo = false;
#endif
		/* rewrite every limit for the new input */
		chg->hw_mode = -1;
		chg->in_w = -1;
		chg->chg_w = -1;
		chg->topoff_w = -1;
		chg->mode = STAGE_1ST;
		chg->status = POWER_SUPPLY_STATUS_CHARGING;
		max77865_set_mode(chg, MODE_CHARGING);
	} else {
#if IS_ENABLED(CONFIG_WIRELESS_CHARGER_IDT_MFC_NV)
		cancel_delayed_work(&chg->wc_current_dwork);
		chg->wc_pre_ma = WC_CURRENT_START;
		chg->wc_target_ma = 0;
		chg->wc_ramp_pending = false;
		chg->wpc_temp_limit = false;
		chg->wpc_cv_mode = false;
#endif
#if IS_ENABLED(CONFIG_WIRELESS_CHARGER_IDT_MFC_HV)
		if (chg->wpc_hv_granted) {		/* back to 5 V first */
			chg->wpc_hv_granted = false;
			idt_mfc_wpc_hv_grant(false);
		}
		chg->wls_vout_mv = 0;
#endif
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_SYSOVLO)
		/* armed again on every removal */
		chg->sysovlo = false;
		if (chg->sysovlo_irq > 0 && !chg->sysovlo_armed) {
			enable_irq(chg->sysovlo_irq);
			chg->sysovlo_armed = true;
		}
#endif
		chg->mode = STAGE_NONE;
		chg->status = POWER_SUPPLY_STATUS_DISCHARGING;
		chg->swelling = SWELLING_NONE;
		chg->cv_want = max77865_cv_normal(chg);
		max77865_set_events(chg, 0, EV_SWELLING_MODE);
		chg->mix_limit = false;
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_AFC)
		chg->hv_event = MAX77865_HV_EV_NONE;
		chg->pre_afc_until = 0;
#endif
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_USB_SS)
		chg->usb_ss = false;
#endif
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_PD)
		max77865_pd_reset(chg);
#endif
		max77865_set_mode(chg, MODE_CHARGING_OFF);
	}
}

static void max77865_policy(struct max77865_charger *chg, struct max77865_sample *s)
{
	int pre, h, dtls_status;
	unsigned int v;
	bool en_swelling = false, en_rechg = false;

	/* CHGINSEL on every wired attach */
	if (chg->input == INPUT_NONE && s->input != INPUT_NONE) {
		max77865_cable_event(chg, true);
		chg->cable_tries = 0;
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_AFC)
		if (s->input == INPUT_WIRED)
			chg->pre_afc_until = jiffies + msecs_to_jiffies(PRE_AFC_DELAY_MS);
		else if (s->input == INPUT_WCIN)
			chg->pre_afc_until = jiffies + msecs_to_jiffies(PRE_AFC_WC_DELAY_MS);
#endif
	} else if (chg->input != INPUT_NONE && s->input == INPUT_NONE) {
		max77865_cable_event(chg, false);
	}
	if (s->input == INPUT_WIRED && chg->input != INPUT_WIRED)
		chg->chginsel_pending = true;
	chg->input = s->input;

	/* battery over-voltage stop */
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_VBAT_OVP)
	if ((chg->status == POWER_SUPPLY_STATUS_CHARGING ||
	     chg->status == POWER_SUPPLY_STATUS_FULL) && !chg->vbat_ovp_done) {
		int vcell = s->vcell;

		/* a failed read (-1) is not above */
		if (vcell > 4400)
			chg->vbat_cnt++;
		else
			chg->vbat_cnt = 0;

		if (chg->vbat_cnt >= 2) {
			chg->vbat_ovp_done = true;
			chg->health = H_VBAT_OVP;
			chg->recharging = false;
			chg->mode = STAGE_NONE;
			chg->status = POWER_SUPPLY_STATUS_NOT_CHARGING;
			max77865_set_mode(chg, MODE_CHARGING_OFF);
			dev_info(chg->dev, "vbat ovp %d mV, charging off\n", vcell);
			return;
		}
	}
#endif

#if IS_ENABLED(CONFIG_CHARGER_MAX77865_SYSOVLO)
	if (chg->sysovlo && chg->health != H_VSYS_OVP &&
	    chg->status != POWER_SUPPLY_STATUS_DISCHARGING) {
		chg->health = H_VSYS_OVP;
		chg->recharging = false;
		chg->mode = STAGE_NONE;
		chg->status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		max77865_set_mode(chg, MODE_CHARGING_OFF);
		dev_info(chg->dev, "vsys ovp, charging off\n");
		return;
	}
#endif

	/* charger input over- and under-voltage, not while discharging */
	if (chg->status != POWER_SUPPLY_STATUS_DISCHARGING &&
	    !(chg->status == POWER_SUPPLY_STATUS_FULL && chg->mode == STAGE_NONE) &&
	    (chg->health == H_GOOD || chg->health == H_OVERVOLTAGE ||
	     chg->health == H_UNDERVOLTAGE) && s->hc != chg->health) {
		chg->health = s->hc;
		if (s->hc == H_GOOD) {
			chg->status = POWER_SUPPLY_STATUS_CHARGING;
			chg->mode = STAGE_1ST;
			if (chg->swelling == SWELLING_NONE)
				max77865_set_mode(chg, MODE_CHARGING);
			dev_info(chg->dev, "input good, charging on\n");
		} else {
			chg->status = POWER_SUPPLY_STATUS_NOT_CHARGING;
			max77865_set_mode(chg, MODE_CHARGING_OFF);
			chg->mode = STAGE_NONE;
			chg->recharging = false;
			dev_info(chg->dev, "input %s, charging off\n", health_name[s->hc]);
		}
		return;
	}

	if (chg->status != POWER_SUPPLY_STATUS_DISCHARGING) {
		if (chg->status == POWER_SUPPLY_STATUS_FULL &&
		    (chg->mode == STAGE_2ND || chg->recharging || chg->swelling != SWELLING_NONE) &&
		    s->cap >= 0 && s->vcell >= 0 && s->cap < MAX77865_FULL_SOC(chg) &&
		    s->vcell < MAX77865_RECHG_VCELL(chg) - 50) {
			chg->status = POWER_SUPPLY_STATUS_CHARGING;
			return;
		}
		/* recharge, not in swelling */
		if (chg->swelling == SWELLING_NONE && chg->mode == STAGE_NONE) {
			bool rechg = false;

			if (chg->status == POWER_SUPPLY_STATUS_CHARGING) {
				rechg = true;
			} else if (chg->status == POWER_SUPPLY_STATUS_FULL && s->vcell >= 0) {
				/* float - 150 mV with the low-temp event */
				int vr = (chg->events & EV_LOW_TEMP) ? FLOAT_MV - 150 :
					 MAX77865_RECHG_VCELL(chg);

				if (s->vcell <= vr) {
					max77865_reset_budget(chg, RECHG_EXPIRED_MS);
					rechg = true;
					dev_dbg(chg->dev, "recharge, vcell %d mV <= %d mV\n",
						s->vcell, vr);
				}
			}
			if (rechg) {
				chg->mode = STAGE_1ST;
				chg->recharging = true;
				max77865_set_mode(chg, MODE_CHARGING);
				return;
			}
		}
	}

	/* safety timer expiry, only while charging */
	if (chg->chg_start_s &&
	    (chg->status == POWER_SUPPLY_STATUS_CHARGING ||
	     chg->status == POWER_SUPPLY_STATUS_FULL) &&
	    !chg->budget_ms) {
		u64 passed = ktime_get_boottime_seconds() - chg->chg_start_s;

		chg->mode = STAGE_NONE;
		chg->health = H_SAFETY_TIMER;
		chg->status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		chg->recharging = false;
		max77865_set_mode(chg, MODE_CHARGING_OFF);
		dev_info(chg->dev, "safety timer expired after %u s, charging off\n", (u32)passed);
		return;
	}

	/* the rest needs a temperature */
	if (!s->ntc_valid)
		return;

#if IS_ENABLED(CONFIG_WIRELESS_CHARGER_IDT_MFC_NV)
	if (s->input == INPUT_WCIN) {
		/* battery temperature limit of wireless charging */
		if (!chg->wpc_temp_limit && s->t >= WPC_BAT_TEMP_LIMIT)
			chg->wpc_temp_limit = true;
		else if (chg->wpc_temp_limit && s->t <= WPC_BAT_TEMP_REC)
			chg->wpc_temp_limit = false;

		/*
		 * CV mode on WPC, A4WP, stand and battery pack TA pads after 10 s of
		 * charging at 85 % or more: the receiver switches to 5.5 V and CV FOD
		 */
		if (!chg->wpc_cv_mode && chg->chg_start_s && s->cap >= 85 &&
		    ktime_get_boottime_seconds() - chg->chg_start_s > 10 &&
		    (s->wls.pad == IDT_MFC_PAD_WPC || s->wls.pad == IDT_MFC_PAD_A4WP ||
		     s->wls.pad == IDT_MFC_PAD_WPC_STAND ||
		     s->wls.pad == IDT_MFC_PAD_WPC_PACK_TA)) {
			chg->wpc_cv_mode = true;
			idt_mfc_wpc_set_cv_mode();
		}
	}
#endif

	if (chg->status != POWER_SUPPLY_STATUS_DISCHARGING) {
		if (chg->health != H_GOOD && chg->health != H_OVERHEAT &&
		    chg->health != H_COLD && chg->health != H_OVERHEATLIMIT)
			return;
		pre = chg->health;
		h = max77865_thermal_h(pre, s->t, s->tu,
				       IS_ENABLED(CONFIG_WIRELESS_CHARGER_IDT_MFC_NV) &&
				       s->input == INPUT_WCIN);
		chg->health = h;
		if (h == H_OVERHEAT || h == H_COLD || h == H_OVERHEATLIMIT) {
			if (h != pre) {
#if IS_ENABLED(CONFIG_WIRELESS_CHARGER_IDT_MFC_NV)
				if (s->input == INPUT_WCIN)
					idt_mfc_wpc_send_ept();
#endif
				chg->status = POWER_SUPPLY_STATUS_NOT_CHARGING;
				max77865_set_mode(chg, h == H_COLD ? MODE_CHARGING_OFF :
								  MODE_BUCK_OFF);
				max77865_log_thermal(chg, pre, s);
				return;
			}
			/* an unknown VCELL counts as 0, as in the vendor driver */
			if (h != H_COLD && chg->chg_mode == MODE_BUCK_OFF &&
			    s->vcell < SWELLING_DROP_MV)
				max77865_set_mode(chg, MODE_CHARGING_OFF);
		} else if (chg->status == POWER_SUPPLY_STATUS_NOT_CHARGING) {
			/* recovery */
			chg->status = s->cap >= 100 ? POWER_SUPPLY_STATUS_FULL :
						      POWER_SUPPLY_STATUS_CHARGING;
			if (s->t > SW_HIGH_REC || s->t < SW_LOW_REC_2ND) {
				chg->swelling = SWELLING_CHARGING;
				max77865_set_mode(chg, MODE_BUCK_OFF);
				if (s->t > SW_HIGH_REC)
					max77865_set_events(chg, EV_HIGH_TEMP_SWELLING,
							    EV_HIGH_TEMP_SWELLING);
				else
					max77865_set_events(chg, EV_LOW_TEMP_SWELLING,
							    EV_LOW_TEMP_SWELLING);
			} else {
				chg->cv_want = max77865_cv_normal(chg);
				if (chg->status == POWER_SUPPLY_STATUS_FULL &&
				    chg->mode == STAGE_NONE)
					max77865_set_mode(chg, MODE_CHARGING_OFF);
				else
					max77865_set_mode(chg, MODE_CHARGING);
				if (s->t <= SW_LOW_BLOCK_1ST ||
				    (s->t < SW_LOW_REC_1ST && pre == H_COLD))
					max77865_set_events(chg, EV_LOW_TEMP, EV_SWELLING_MODE);
			}
			max77865_log_thermal(chg, pre, s);
			return;
		}
	}

	if (chg->status == POWER_SUPPLY_STATUS_DISCHARGING ||
	    chg->status == POWER_SUPPLY_STATUS_NOT_CHARGING) {
		chg->swelling = SWELLING_NONE;
		max77865_set_events(chg, 0, EV_SWELLING_MODE);
	} else {
		if (chg->swelling == SWELLING_NONE) {
			if (s->t >= SW_HIGH_BLOCK || s->t <= SW_LOW_BLOCK_2ND) {
				chg->swelling = SWELLING_CHARGING;
				max77865_set_mode(chg, MODE_BUCK_OFF);
				max77865_set_events(chg, s->t >= SW_HIGH_BLOCK ?
						    EV_HIGH_TEMP_SWELLING : EV_LOW_TEMP_SWELLING,
						    EV_SWELLING_MODE);
				en_swelling = true;
			} else if (s->t <= SW_LOW_BLOCK_1ST && !(chg->events & EV_LOW_TEMP)) {
				max77865_set_events(chg, EV_LOW_TEMP, EV_LOW_TEMP);
			} else if (s->t >= SW_LOW_REC_1ST && (chg->events & EV_LOW_TEMP)) {
				max77865_set_events(chg, 0, EV_LOW_TEMP);
			}
		}

		if (s->vcell < 0)
			goto full_check;

		if (chg->swelling != SWELLING_NONE) {
			if (s->t <= SW_HIGH_REC && s->t >= SW_LOW_REC_2ND) {
				bool low = s->t <= SW_LOW_BLOCK_1ST ||
					   (s->t < SW_LOW_REC_1ST &&
					    (chg->events & EV_LOW_TEMP_SWELLING));

				chg->swelling = SWELLING_NONE;
				chg->mode = STAGE_1ST;
				max77865_set_events(chg, low ? EV_LOW_TEMP : 0, EV_SWELLING_MODE);
				max77865_set_mode(chg, MODE_CHARGING);
				chg->cv_want = max77865_cv_normal(chg);
				max77865_log_swelling(chg, "end", s);
			} else if (s->vcell < SW_RECHG_MV && chg->chg_block) {
				chg->mode = STAGE_1ST;
				en_rechg = true;
				chg->cv_want = CV_CODE_SWELLING;
				max77865_set_mode(chg, MODE_CHARGING);
				if (s->t < SW_LOW_REC_2ND)
					max77865_set_events(chg, EV_LOW_TEMP_SWELLING,
							    EV_SWELLING_MODE);
				else if (s->t > SW_HIGH_REC)
					max77865_set_events(chg, EV_HIGH_TEMP_SWELLING,
							    EV_SWELLING_MODE);
				max77865_log_swelling(chg, "recharge", s);
			}
		}

		if (en_swelling && !en_rechg) {
			max77865_reset_budget(chg, EXPIRED_MS);
			max77865_log_swelling(chg, "start", s);
		}
	}

full_check:
	/* swelling full or normal full check on the charger's current status */
	if (regmap_read(chg->regmap, MAX77865_CHG_REG_DETAILS_01, &v))
		return;
	dtls_status = max77865_dtls_status(v);

	if ((chg->swelling == SWELLING_CHARGING || chg->swelling == SWELLING_FULL) &&
	    !chg->chg_block) {
		if (dtls_status == POWER_SUPPLY_STATUS_FULL) {
			chg->mode = STAGE_NONE;
			chg->recharging = false;
			chg->swelling = SWELLING_FULL;
			max77865_set_mode(chg, MODE_CHARGING_OFF);
			max77865_reset_budget(chg, EXPIRED_MS);
			max77865_log_swelling(chg, "full", s);
		}
	} else if (chg->mode != STAGE_NONE && chg->status != POWER_SUPPLY_STATUS_NOT_CHARGING) {
		/* full needs the cell voltage too */
		if (s->vcell >= MAX77865_FULL_VCELL(chg) && dtls_status == POWER_SUPPLY_STATUS_FULL)
			max77865_do_full(chg, s);
	}
}

/* input, charge and top-off currents of this pass, in mA */
static void max77865_currents(struct max77865_charger *chg, const struct max77865_sample *s,
			      int *in, int *cc, int *topoff)
{
	switch (s->cable) {
	case CABLE_DCP:
		*in = 1800;
		*cc = 2100;
		break;
	case CABLE_CDP:
		*in = 1500;
		*cc = 1500;
		break;
	case CABLE_AFC_PREPARE:
		*in = 1000;
		*cc = 1000;
		break;
	case CABLE_HV9:
		*in = 1650;
		*cc = 2350;
		break;
	case CABLE_PD:		/* the contract's PDO */
		*in = s->pd_ma;
		*cc = min(s->pd_mv * s->pd_ma / FLOAT_MV, 2350);
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_PD)
		if (chg->pd_cap_ma)		/* lower input before a PDO change */
			*in = min(*in, chg->pd_cap_ma);
#endif
		break;
	case WLS_NV:
		*in = 900;
		*cc = 1300;
		break;
	case WLS_PACK:
		*in = 700;
		*cc = 1400;
		break;
	case WLS_HV:
		*in = 650;
		*cc = 1300;
		break;
	case WLS_PREP_HV:
		*in = 500;
		*cc = 1300;
		break;
	default:		/* SDP and unknown */
		*in = 500;
		*cc = 500;
		break;
	}

#if IS_ENABLED(CONFIG_CHARGER_MAX77865_USB_SS)
	/* a SuperSpeed configuration allows 900 mA; an Rp above default wins */
	if (s->cable == CABLE_SDP && chg->usb_ss) {
		*in = 900;
		*cc = 900;
	}
#endif

#if IS_ENABLED(CONFIG_CHARGER_MAX77865_TYPEC_RP)
	/* Type-C Rp of a USB or TA source without a PD contract; default Rp keeps the table */
	if ((s->cable == CABLE_SDP || s->cable == CABLE_DCP) && !s->pd_mv) {
		if (s->rp == S2MM005_RP_CURRENT_LEVEL3) {
			*in = 3000;
			*cc = 2350;
		} else if (s->rp == S2MM005_RP_CURRENT_LEVEL2) {
			*in = 1500;
			*cc = 1500;
		}
	}
#endif

#if IS_ENABLED(CONFIG_CHARGER_MAX77865_AFC)
	/* a lower input for a while after attach; the pass runs again at its end */
	if (chg->pre_afc_until && time_before(jiffies, chg->pre_afc_until)) {
		if (s->input == INPUT_WIRED &&
		    (s->cable == CABLE_DCP || s->cable == CABLE_AFC_PREPARE ||
		     s->cable == CABLE_HV9))
			*in = min(*in, PRE_AFC_IN);
		else if (s->input == INPUT_WCIN)
			*in = min(*in, PRE_AFC_WC_IN);
	}
#endif

	/* mix-temperature input limit, wired only; kept while there is no temperature */
	if (s->input != INPUT_WIRED) {
		chg->mix_limit = false;
	} else if (s->ntc_valid) {
		if ((!chg->mix_limit && s->t >= MIX_HIGH_TEMP && s->tc >= MIX_HIGH_CHG_TEMP) ||
		    (chg->mix_limit && s->t > MIX_HIGH_TEMP_REC))
			chg->mix_limit = true;
		else
			chg->mix_limit = false;
	}
	if (s->input == INPUT_WIRED && chg->mix_limit)
		*in = max77865_mix_input_ma(max77865_in_mv(s));

#if IS_ENABLED(CONFIG_CHARGER_MAX77865_AFC) || IS_ENABLED(CONFIG_CHARGER_MAX77865_QC) || \
	IS_ENABLED(CONFIG_CHARGER_MAX77865_TYPEC_RP) || IS_ENABLED(CONFIG_CHARGER_MAX77865_PD)
	/*
	 * Charger NTC limit of 9 V and 15 W inputs; PD has its own PDO step and the
	 * mix limit wins. Once limited, both cases hold the 9 V limit currents.
	 */
	if (s->input == INPUT_WIRED && s->cable != CABLE_PD && !chg->mix_limit) {
		bool hv = s->cable == CABLE_HV9;
		bool pwr = chg->max_power >= CHG_HIGH_TEMP_PWR_THR;

		if (!chg->chg_high_temp_limit && hv && s->tc > CHG_HIGH_TEMP) {
			*in = CHG_HIGH_TEMP_HV_IN;
			*cc = CHG_HIGH_TEMP_HV_CC;
			chg->chg_high_temp_limit = true;
		} else if (!chg->chg_high_temp_limit && pwr && s->tc > CHG_HIGH_TEMP) {
			*in = CHG_HIGH_TEMP_DEFAULT_IN;
			*cc = CHG_HIGH_TEMP_DEFAULT_CC;
			chg->chg_high_temp_limit = true;
		} else if (chg->chg_high_temp_limit && (hv || pwr)) {
			if (s->tc < CHG_HIGH_TEMP_REC) {
				chg->chg_high_temp_limit = false;	/* the class values stay */
			} else {
				*in = CHG_HIGH_TEMP_HV_IN;
				*cc = CHG_HIGH_TEMP_HV_CC;
			}
		}
	}
#endif

	*topoff = chg->mode == STAGE_2ND ? TOPOFF_2ND : TOPOFF_1ST;

	if (chg->events & EV_LOW_TEMP_SWELLING) {
		*cc = min(*cc, SW_LOW_CURRENT);
		*topoff = min(*topoff, SW_LOW_TOPOFF);
	} else if (chg->events & EV_HIGH_TEMP_SWELLING) {
		*cc = min(*cc, SW_HIGH_CURRENT);
		*topoff = min(*topoff, SW_HIGH_TOPOFF);
	} else if (chg->events & EV_LOW_TEMP) {
		*cc = min(*cc, SW_LOW_CURRENT);
	}

#if IS_ENABLED(CONFIG_WIRELESS_CHARGER_IDT_MFC_NV)
	if (s->input == INPUT_WCIN) {
		if (chg->wpc_temp_limit)
			*in = min(*in, WPC_BAT_TEMP_CURRENT);
		if (chg->wpc_cv_mode)
			*in = min(*in, s->wls.pad == IDT_MFC_PAD_WPC_PACK ?
				  630 : 820);
		if (chg->status == POWER_SUPPLY_STATUS_FULL && chg->mode == STAGE_2ND)
			*in = min(*in, 600);
		if (chg->status == POWER_SUPPLY_STATUS_FULL && chg->mode == STAGE_NONE)
			*in = 100;
		if (chg->events & (EV_LOW_TEMP_SWELLING | EV_HIGH_TEMP_SWELLING | EV_LOW_TEMP))
			*cc = min(*cc, WPC_SW_CURRENT);
	}
#endif

	*cc = min(*cc, 2350);
}

/* write what changed, with read-modify-write as the vendor driver */
static void max77865_apply(struct max77865_charger *chg, const struct max77865_sample *s)
{
	int cv = chg->cv_want;
	int in = 0, cc = 0, topoff = 0, eff;

	if (chg->fault == 2)
		return;		/* switching charging off failed: write nothing */

	if (s->input != INPUT_NONE && !chg->otg_on) {
		max77865_currents(chg, s, &in, &cc, &topoff);
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_AICL)
		/* the AICL result is a cap the pass can only lower */
		if (chg->aicl_ma)
			in = min(in, (int)chg->aicl_ma);
#endif
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_HV_GUARD) && !IS_ENABLED(CONFIG_CHARGER_MAX77865_PD)
		/*
		 * Without the PD policy, a contract above 5 V that the S2MM005 firmware
		 * chose by itself caps the input at 500 mA.
		 */
		if (s->input == INPUT_WIRED) {
			int pd_mv = s->pd_mv;

			if (pd_mv > 5000) {
				in = min(in, 500);
				if (!chg->hv_guard_logged)
					dev_info(chg->dev,
						 "input above 5 V (%s %d mV) without stage, input capped 500 mA\n",
						 "pd", pd_mv);
				chg->hv_guard_logged = true;
			}
		}
#endif
		chg->in_ma = s->input == INPUT_WIRED ? in : 0;
		chg->chg_ma = cc;
	}
	max77865_wpc_gate(chg, chg->otg_on, s->input == INPUT_WIRED && in * 5 >= 4000);

	if (!chg->fault) {
		if (chg->chginsel_pending && !chg->otg_on &&
		    !regmap_update_bits(chg->regmap, MAX77865_CHG_REG_CNFG_12,
					MAX77865_CNFG_12_CHGINSEL, MAX77865_CNFG_12_CHGINSEL))
			chg->chginsel_pending = false;
#if IS_ENABLED(CONFIG_WIRELESS_CHARGER_IDT_MFC_NV)
		if (s->input == INPUT_WCIN && !chg->otg_on) {
			regmap_update_bits(chg->regmap, MAX77865_CHG_REG_CNFG_12,
					   MAX77865_CNFG_12_CHGINSEL, 0);

			if (cc != chg->chg_w &&
			    !regmap_update_bits(chg->regmap, MAX77865_CHG_REG_CNFG_02,
						MAX77865_CNFG_02_CHG_CC, min(cc, 3150) / 50))
				chg->chg_w = cc;

			/* WCIN input through the ramp: headroom 1 (270 mV), then the work */
			chg->wc_cap = s->cap;
			chg->wc_pad = s->wls.pad;
			if (in != chg->in_w && in != chg->wc_target_ma) {
				chg->wc_target_ma = in;
				if (!chg->wc_ramp_pending) {
					chg->wc_ramp_pending = true;
					idt_mfc_wpc_set_headroom(1);
					queue_delayed_work(system_freezable_wq,
							   &chg->wc_current_dwork,
							   msecs_to_jiffies(500));
				}
			}
		}
#endif
		if (s->input == INPUT_WIRED && !chg->otg_on && in != chg->in_w &&
		    !regmap_update_bits(chg->regmap, MAX77865_CHG_REG_CNFG_09,
					MAX77865_CNFG_09_CHGIN_LIM, min(in, 3175) / 25))
			chg->in_w = in;
		if (s->input != INPUT_NONE && !chg->otg_on && cc != chg->chg_w &&
		    !regmap_update_bits(chg->regmap, MAX77865_CHG_REG_CNFG_02,
					MAX77865_CNFG_02_CHG_CC, min(cc, 3150) / 50))
			chg->chg_w = cc;
		if (s->input != INPUT_NONE && !chg->otg_on && topoff != chg->topoff_w &&
		    !regmap_update_bits(chg->regmap, MAX77865_CHG_REG_CNFG_03,
					MAX77865_CNFG_03_TO_ITH,
					(clamp(topoff, 150, 500) - 150) / 50))
			chg->topoff_w = topoff;
		if (cv != chg->cv_w &&
		    !regmap_update_bits(chg->regmap, MAX77865_CHG_REG_CNFG_04,
					MAX77865_CNFG_04_CHG_CV_PRM, cv))
			chg->cv_w = cv;
	}

	if (chg->otg_on)
		return;					/* the input counts as detached */

	/* off in fault mode and without a valid NTC reading */
	eff = chg->chg_mode;
#if IS_ENABLED(CONFIG_WIRELESS_CHARGER_IDT_MFC_NV)
	if (s->input == INPUT_WCIN && s->wls.fod)
		eff = MODE_CHARGING_OFF;		/* unlike the vendor driver */
#endif
	if (eff == MODE_CHARGING && (chg->fault || !chg->ntc_seen || !s->ntc_valid))
		eff = MODE_CHARGING_OFF;
	/* a limit that did not reach the charger keeps it off until a later pass writes it */
	if (eff == MODE_CHARGING && s->input != INPUT_NONE &&
	    ((s->input == INPUT_WIRED && chg->in_w != in) || chg->chg_w != cc ||
	     chg->topoff_w != topoff || chg->cv_w != cv)) {
		dev_warn_ratelimited(chg->dev, "charge limits not written, charging off\n");
		eff = MODE_CHARGING_OFF;
	}
	if (eff != chg->hw_mode)
		max77865_write_mode(chg, eff);
}

static int max77865_zone_avg(const char *name, int *mc)
{
	struct thermal_zone_device *tz = thermal_zone_get_zone_by_name(name);

	if (IS_ERR(tz))
		return PTR_ERR(tz);
	return max77865_ntc_avg(tz, mc);
}

/* battery, USB-connector and charger thermistors */
static void max77865_read_temps(struct max77865_charger *chg, struct max77865_sample *s)
{
	int mc, ret;

	ret = max77865_zone_avg(chg->tz_name, &mc);
	if ((s32)chg->override_t_mc) {
		mc = (s32)chg->override_t_mc;
		ret = 0;
	}
	if (!ret) {
		s->t_mc = mc;
		s->t = mc / 100;
		s->ntc_valid = true;
		/* table end points clamp to -200 / 900: not valid for the gate */
		if (s->t > -200 && s->t < 900)
			chg->ntc_seen = true;
	} else if (!ntc_required) {
		s->t = T_DEFAULT;
		s->t_mc = T_DEFAULT * 100;
		s->ntc_valid = true;
		s->ntc_escape = true;
		chg->ntc_seen = true;
	}

	ret = max77865_zone_avg("usb-ntc", &mc);
	if ((s32)chg->override_usb_mc) {
		mc = (s32)chg->override_usb_mc;
		ret = 0;
	}
	if (ret) {
		mc = T_DEFAULT * 100;
		if (!chg->usb_ntc_logged)
			dev_warn(chg->dev, "%s invalid, counted as 25.0 degC\n", "usb-ntc");
		chg->usb_ntc_logged = true;
	}
	s->tu = mc / 100;

	ret = max77865_zone_avg("chg-ntc", &mc);
	if ((s32)chg->override_chg_mc) {
		mc = (s32)chg->override_chg_mc;
		ret = 0;
	}
	if (ret) {
		mc = T_DEFAULT * 100;
		if (!chg->chg_ntc_logged)
			dev_warn(chg->dev, "%s invalid, counted as 25.0 degC\n", "chg-ntc");
		chg->chg_ntc_logged = true;
	}
	s->tc = mc / 100;
}

/* FG VCELL / IAVG / CAPACITY; -1 = unknown */
static void max77865_read_fg(struct max77865_charger *chg, struct max77865_sample *s)
{
	union power_supply_propval v;

	s->vcell = -1;
	s->cap = -1;
	if (!chg->fg)
		chg->fg = power_supply_get_by_name("battery");
	if (!chg->fg)
		return;
	if (!power_supply_get_property(chg->fg, POWER_SUPPLY_PROP_VOLTAGE_NOW, &v))
		s->vcell = v.intval / 1000;
	if (!power_supply_get_property(chg->fg, POWER_SUPPLY_PROP_CURRENT_AVG, &v)) {
		s->iavg = v.intval;
		s->iavg_valid = true;
	}
	if (!power_supply_get_property(chg->fg, POWER_SUPPLY_PROP_CAPACITY, &v))
		s->cap = v.intval;
}

/*
 * BC1.2 result from the MUIC's STATUS1_BC, read on the adapter: the MUIC driver
 * owns 0x25. ChgTyp 0 or ChgDetRun = detection not finished.
 */
static enum max77865_cable max77865_read_cable(struct max77865_charger *chg, bool *pending)
{
	union i2c_smbus_data d;

	*pending = false;
	if (i2c_smbus_xfer(chg->client->adapter, MAX77865_MUIC_I2C_ADDR, 0, I2C_SMBUS_READ,
			   MAX77865_MUIC_REG_STATUS1_BC, I2C_SMBUS_BYTE_DATA, &d))
		return CABLE_UNKNOWN;

	if ((d.byte & MAX77865_BC_STATUS1_CHGDETRUN) ||
	    FIELD_GET(MAX77865_BC_STATUS1_CHGTYP, d.byte) == MAX77865_CHGTYP_NO_VOLTAGE) {
		*pending = true;
		return CABLE_UNKNOWN;
	}
	switch (FIELD_GET(MAX77865_BC_STATUS1_CHGTYP, d.byte)) {
	case MAX77865_CHGTYP_USB:
		return CABLE_SDP;
	case MAX77865_CHGTYP_CDP:
		return CABLE_CDP;
	default:
		return CABLE_DCP;
	}
}

#if MAX77865_CCIC
/* the PD contract the S2MM005 reported: the PDO the firmware or the last request made current */
static void max77865_read_pd(struct max77865_charger *chg, struct max77865_sample *s)
{
	unsigned int pos = chg->pd.current_pdo_num;

	s->rp = chg->pd.rp_currentlvl;		/* NONE after a detach */
	if ((chg->pd_event != S2MM005_PD_EVENT_PD_SINK &&
	     chg->pd_event != S2MM005_PD_EVENT_PD_SINK_CAP) ||
	    pos < 1 || pos > S2MM005_MAX_PDO_NUM)
		return;
	s->pd_mv = chg->pd.power_list[pos].max_voltage;
	s->pd_ma = chg->pd.power_list[pos].max_current;
}
#endif

#if IS_ENABLED(CONFIG_WIRELESS_CHARGER_IDT_MFC_HV)
/*
 * The 10 V wireless grant, after the apply step of every pass so that no early
 * return of the policy skips a withdrawal; a withdrawal steps Vout down to 5 V.
 */
static void max77865_wls_hv_decide(struct max77865_charger *chg, const struct max77865_sample *s)
{
	bool grant = s->input == INPUT_WCIN && s->wls.online && !s->wls.fod &&
		     chg->health == H_GOOD && chg->swelling == SWELLING_NONE &&
		     !(chg->events & EV_SWELLING_MODE) && !chg->fault && chg->ntc_seen &&
		     s->ntc_valid && !chg->otg_on && chg->status != POWER_SUPPLY_STATUS_FULL &&
		     !chg->wpc_temp_limit;

	if (grant != chg->wpc_hv_granted) {
		chg->wpc_hv_granted = grant;
		idt_mfc_wpc_hv_grant(grant);
	}
	if (s->input == INPUT_WCIN && s->wls.vout_mv != chg->wls_vout_mv) {
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_AICL)
		if (chg->wls_vout_mv)
			chg->aicl_ma = 0;
#endif
		chg->wls_vout_mv = s->wls.vout_mv;
	}
}
#endif

#if IS_ENABLED(CONFIG_CHARGER_MAX77865_AFC)
/*
 * The AFC/QC 9 V grant, after the apply step of every pass so that no early
 * return of the policy skips a withdrawal. Unlike the vendor driver, every
 * condition that stops charging also asks for 5 V.
 */
static void max77865_hv_decide(struct max77865_charger *chg, const struct max77865_sample *s)
{
	int hv = READ_ONCE(chg->hv_event);
	bool grant;

	if (s->input != INPUT_WIRED)
		return;
	grant = (s->cable == CABLE_DCP || s->cable == CABLE_AFC_PREPARE ||
		 s->cable == CABLE_HV9) &&
		(hv == MAX77865_HV_EV_5V || hv == MAX77865_HV_EV_PREPARE ||
		 hv == MAX77865_HV_EV_9V || hv == MAX77865_HV_EV_ERR_V) &&
		chg->status == POWER_SUPPLY_STATUS_CHARGING && chg->health == H_GOOD &&
		chg->swelling == SWELLING_NONE && !(chg->events & EV_SWELLING_MODE) &&
		chg->ntc_seen && s->ntc_valid && !chg->fault && !chg->otg_on &&
		s->tc <= CHG_HIGH_TEMP && !chg->chg_high_temp_limit;

	if (grant) {
		/*
		 * only from 5 V: the request itself notifies PREPARE, and asking
		 * again there would restart the handshake on every pass
		 */
		if (hv == MAX77865_HV_EV_5V)
			max77865_hv_set_voltage(9000);
	} else {
		if (hv == MAX77865_HV_EV_9V || hv == MAX77865_HV_EV_ERR_V)
			max77865_hv_set_voltage(5000);
	}
}

/* the pass reads the event without the lock; this must not sleep */
static int max77865_hv_notify(struct notifier_block *nb, unsigned long action, void *data)
{
	struct max77865_charger *chg = container_of(nb, struct max77865_charger, hv_nb);

	WRITE_ONCE(chg->hv_event, (int)action);
	if (!chg->stopping)
		mod_delayed_work(system_freezable_wq, &chg->pass_dwork, 0);
	return NOTIFY_OK;
}

static void max77865_hv_unregister(void *data)
{
	struct max77865_charger *chg = data;

	max77865_hv_unregister_notifier(&chg->hv_nb);
}
#endif

#if IS_ENABLED(CONFIG_CHARGER_MAX77865_PD)
#define PD_CHARGE_POWER		15000	/* mW */
#define PD_MAX_INPUT_MV		12000
#define PD_MAX_INPUT_MA		3000
/* the vendor device tree leaves it unset: 5 V at the 1800 mA default input */
#define PD_NV_CHARGE_POWER	9000

static void max77865_pd_reset(struct max77865_charger *chg)
{
	chg->pd_attached = false;
	chg->pd_ready = false;
	chg->pd_limit = false;
	chg->pd_count = 0;
	chg->pd_index = 0;
	chg->pd_target = 0;
	chg->pd_req = 0;
}

/* one PDO per watt step up to 15 W, the lowest voltage of each, in ascending voltage */
static void max77865_pd_make_list(struct max77865_charger *chg)
{
	int avail = min_t(int, chg->pd.available_pdo_num, S2MM005_MAX_PDO_NUM);
	int base, i, j, n = 0;

	for (base = PD_CHARGE_POWER * 1000; base >= 1000000; base -= 1000000) {
		int sel_mv = PD_MAX_INPUT_MV + 1, sel = 0;

		for (i = 1; i <= avail; i++) {
			long p = (long)chg->pd_mv[i] * chg->pd_ma[i];

			if (p > base - 1000000 && p <= base && chg->pd_mv[i] < sel_mv) {
				sel_mv = chg->pd_mv[i];
				sel = i;
				chg->pd_ma[i] = min(chg->pd_ma[i], PD_MAX_INPUT_MA);
			}
		}
		if (sel && n < S2MM005_MAX_PDO_NUM) {
			chg->pd_list[n].mv = chg->pd_mv[sel];
			chg->pd_list[n].ma = chg->pd_ma[sel];
			chg->pd_list[n].pdo = sel;
			n++;
		}
	}
	for (i = 0; i < n - 1; i++) {			/* ascending voltage */
		int min = i;

		for (j = i + 1; j < n; j++)
			if (chg->pd_list[j].mv < chg->pd_list[min].mv)
				min = j;
		swap(chg->pd_list[i], chg->pd_list[min]);
	}
	for (i = 0; i < n; i++)
		dev_dbg(chg->dev, "pd list %d:%dmV/%dmA (pdo %d)\n", i, chg->pd_list[i].mv,
			chg->pd_list[i].ma, chg->pd_list[i].pdo);
	chg->pd_count = n;
}

/* a notification from the S2MM005 */
static void max77865_pd_event(struct max77865_charger *chg)
{
	int i, avail;

	chg->pd_new = false;
	chg->pd_req = 0;				/* the firmware reported its contract */
	if (chg->pd_event != S2MM005_PD_EVENT_PD_SINK &&
	    chg->pd_event != S2MM005_PD_EVENT_PD_SINK_CAP) {
		if (chg->pd_event == S2MM005_PD_EVENT_DETACH)
			max77865_pd_reset(chg);
		return;
	}
	if (chg->pd_attached && chg->pd_event == S2MM005_PD_EVENT_PD_SINK) {
		/* a later contract: index of the current PDO, ready */
		chg->pd_index = 0;
		for (i = 0; i < chg->pd_count; i++)
			if (chg->pd_list[i].pdo == chg->pd.current_pdo_num)
				chg->pd_index = i;
		chg->pd_ready = true;
		return;
	}
	/* the first contract or new source capabilities: clamp every PDO to 15 W */
	avail = min_t(int, chg->pd.available_pdo_num, S2MM005_MAX_PDO_NUM);
	for (i = 0; i <= S2MM005_MAX_PDO_NUM; i++) {
		chg->pd_mv[i] = chg->pd.power_list[i].max_voltage;
		chg->pd_ma[i] = chg->pd.power_list[i].max_current;
		if (i >= 1 && i <= avail && chg->pd_mv[i] > 0 &&
		    (long)chg->pd_mv[i] * chg->pd_ma[i] > PD_CHARGE_POWER * 1000L)
			chg->pd_ma[i] = PD_CHARGE_POWER * 1000 / chg->pd_mv[i];
	}
	chg->pd_index = 0;
	chg->pd_limit = false;
	max77865_pd_make_list(chg);
	if (!chg->pd_count) {
		dev_dbg(chg->dev, "pd list empty\n");
		return;
	}
	chg->pd_attached = true;
	chg->pd_target = chg->pd_count - 1;		/* the highest power */
	chg->pd_ready = chg->pd.selected_pdo_num == chg->pd_list[chg->pd_target].pdo;
	if (chg->pd_ready)
		chg->pd_index = chg->pd_target;
}

/*
 * Before the apply step: the charger NTC step through the PDO list, then the
 * PDO this pass asks for and the input cap to write before the request. Unlike
 * the vendor driver, anything that stops charging asks for the 5 V PDO.
 */
static void max77865_pd_step(struct max77865_charger *chg, struct max77865_sample *s)
{
	int idx, want, pos = chg->pd.current_pdo_num;
	bool grant;

	if (chg->pd_new)
		max77865_pd_event(chg);
	chg->pd_cap_ma = 0;
	chg->pd_want = 0;
	if (s->input != INPUT_WIRED || !chg->pd_attached || !chg->pd_count ||
	    pos < 1 || pos > S2MM005_MAX_PDO_NUM)
		return;
	s->cable = CABLE_PD;
	s->pd_mv = chg->pd_mv[pos];
	s->pd_ma = chg->pd_ma[pos];			/* the clamped value */

	/* not under the mix limit */
	if (chg->pd_ready && !chg->mix_limit && s->ntc_valid) {
		idx = chg->pd_index;
		if (!chg->pd_limit) {
			if (s->tc >= CHG_HIGH_TEMP) {
				chg->pd_limit = true;
				idx--;
			} else {
				idx++;
			}
		} else {
			if (s->tc <= CHG_HIGH_TEMP_REC)
				chg->pd_limit = false;
			else if (s->tc >= CHG_HIGH_TEMP)
				idx--;
		}
		if (idx < 0) {
			/* a PDO below 1 V (malformed source) gets no current at all */
			if (chg->pd_limit)
				chg->pd_cap_ma = chg->pd_list[0].mv >= 1000 ?
					PD_NV_CHARGE_POWER / (chg->pd_list[0].mv / 1000) : 1;
			chg->pd_index = chg->pd_limit ? -1 : 0;
		} else {
			chg->pd_target = min(idx, chg->pd_count - 1);
		}
	}

	grant = chg->status == POWER_SUPPLY_STATUS_CHARGING && chg->health == H_GOOD &&
		chg->swelling == SWELLING_NONE && !(chg->events & EV_SWELLING_MODE) &&
		chg->ntc_seen && s->ntc_valid && !chg->fault && !chg->otg_on;
	want = grant ? chg->pd_list[chg->pd_target].pdo : 1;
	if (want != pos && want != chg->pd_req) {
		int ma = chg->pd_ma[want];

		chg->pd_cap_ma = chg->pd_cap_ma ? min(chg->pd_cap_ma, ma) : ma;
		chg->pd_want = want;
	}
}

/* after the apply step wrote the lower input: request the PDO */
static void max77865_pd_request(struct max77865_charger *chg)
{
	int want = chg->pd_want;

	if (!want)
		return;
	dev_dbg(chg->dev, "select pdo %d (%d mV)\n", want, chg->pd_mv[want]);
	chg->pd_ready = false;
	chg->pd_req = want;
	s2mm005_select_pdo(want);
}

static int max77865_pd_list_show(struct seq_file *m, void *unused)
{
	struct max77865_charger *chg = m->private;
	int i;

	mutex_lock(&chg->lock);
	for (i = 0; i < chg->pd_count; i++)
		seq_printf(m, "%d:%dmV/%dmA pdo %d%s\n", i, chg->pd_list[i].mv, chg->pd_list[i].ma,
			   chg->pd_list[i].pdo, i == chg->pd_index ? " *" : "");
	mutex_unlock(&chg->lock);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(max77865_pd_list);
#endif

static void max77865_pass_work(struct work_struct *work)
{
	struct max77865_charger *chg = container_of(to_delayed_work(work),
						    struct max77865_charger, pass_dwork);
	struct max77865_sample s = { .cable = CABLE_NONE };
	bool pending = false, requeued, plugged, read_ok = false;
	bool changed_chg = false, changed_ac = false, changed_usb = false;
	int rep_status, rep_health, ac, usb, ac_ma, usb_ma, ac_mv, online, topoff;
	unsigned int wd;

	mutex_lock(&chg->lock);
	if (chg->stopping) {
		mutex_unlock(&chg->lock);
		pm_relax(chg->dev);
		return;
	}

	/* watchdog kick: always with charge control, else while WDTEN */
	if (chg->charge_control)
		max77865_kick(chg, true);
	else if (!regmap_read(chg->regmap, MAX77865_CHG_REG_CNFG_00, &wd))
		max77865_kick(chg, wd & MAX77865_CNFG_00_WDTEN);

	if (regmap_read(chg->regmap, MAX77865_CHG_REG_INT_OK, &s.int_ok) ||
	    regmap_read(chg->regmap, MAX77865_CHG_REG_DETAILS_00, &s.dtls0) ||
	    regmap_read(chg->regmap, MAX77865_CHG_REG_DETAILS_01, &s.dtls1) ||
	    regmap_read(chg->regmap, MAX77865_CHG_REG_CNFG_00, &s.cnfg00)) {
		dev_err_ratelimited(chg->dev, "status read failed, pass skipped\n");
		plugged = chg->input != INPUT_NONE;
		goto rearm;
	}
	if (chg->otg_on)
		s.input = INPUT_NONE;
	else if (s.int_ok & MAX77865_CHGIN_OK)
		s.input = INPUT_WIRED;
	else if (s.int_ok & MAX77865_WCIN_OK)
		s.input = INPUT_WCIN;
	s.aicl_ok = !!(s.int_ok & MAX77865_AICL_OK);
	if (s.input == INPUT_WIRED)
		s.cable = max77865_read_cable(chg, &pending);
	if (!pending)
		chg->cable_tries = 0;
#if MAX77865_CCIC
	if (s.input == INPUT_WIRED)
		max77865_read_pd(chg, &s);
#endif
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_AFC)
	if (chg->charge_control && s.input == INPUT_WIRED && s.cable == CABLE_DCP) {
		int hv = READ_ONCE(chg->hv_event);

		if (hv == MAX77865_HV_EV_9V || hv == MAX77865_HV_EV_ERR_V) {
			s.cable = CABLE_HV9;
			s.hv_mv = 9000;
		} else if (hv == MAX77865_HV_EV_PREPARE) {
			s.cable = CABLE_AFC_PREPARE;
			s.hv_mv = 5000;
		} else if (hv == MAX77865_HV_EV_5V) {
			s.hv_mv = 5000;
		}
	}
	if (chg->charge_control)
		chg->hv_mv_dbg = s.hv_mv;
#endif

#if IS_ENABLED(CONFIG_WIRELESS_CHARGER_IDT_MFC_NV)
	if (chg->charge_control && s.input == INPUT_WCIN) {
		struct idt_mfc_wpc_status ws;

		if (!idt_mfc_wpc_get_status(&ws)) {
			s.wls.online = ws.online;
			s.wls.fod = ws.fod;
			s.wls.pad = ws.pad;
			s.wls.vout_mv = ws.vout_mv;
			switch (ws.pad) {
			case IDT_MFC_PAD_WPC_PACK:
				s.wls.class = WLS_PACK;
				break;
			case IDT_MFC_PAD_WPC_AFC:
			case IDT_MFC_PAD_WPC_PACK_TA:
			case IDT_MFC_PAD_WPC_STAND_HV:
			case IDT_MFC_PAD_WPC_VEHICLE_HV:
				s.wls.class = WLS_HV;
				break;
			case IDT_MFC_PAD_PREPARE_HV:
				s.wls.class = WLS_PREP_HV;
				break;
			default:
				s.wls.class = WLS_NV;
				break;
			}
			s.cable = s.wls.class;
		}
	}
#endif

	if (chg->charge_control || IS_ENABLED(CONFIG_CHARGER_MAX77865_TTF)) {
		if (chg->charge_control)
			max77865_read_temps(chg, &s);
		max77865_read_fg(chg, &s);
	}
	s.hc = max77865_decode_hc(&s, chg->charge_control ? chg->health : chg->hc);
	if (chg->override_hc == H_OVERVOLTAGE || chg->override_hc == H_UNDERVOLTAGE)
		s.hc = chg->override_hc;
	chg->hc = s.hc;
	chg->ntc_valid = s.ntc_valid;

#if IS_ENABLED(CONFIG_CHARGER_MAX77865_AGE_FORECAST)
	if (chg->charge_control && !chg->age_inited && chg->fg && s.ntc_valid) {
		max77865_aging_check(chg, &s);
		chg->age_inited = true;
	}
#endif

	/* vendor workaround for these states while charging: CHG off, CV again, CHG on */
	if (chg->charge_control && !chg->fault && chg->hw_mode == MODE_CHARGING &&
	    s.input != INPUT_NONE && chg->ntc_seen && s.ntc_valid &&
	    (FIELD_GET(MAX77865_CHG_DTLS, s.dtls1) == 0x08 ||
	     FIELD_GET(MAX77865_CHG_DTLS, s.dtls1) == 0x0B)) {
		dev_info(chg->dev, "chg_dtls 0x%x while charging, re-arming\n",
			 (unsigned int)FIELD_GET(MAX77865_CHG_DTLS, s.dtls1));
		max77865_write_mode(chg, MODE_CHARGING_OFF);
		regmap_update_bits(chg->regmap, MAX77865_CHG_REG_CNFG_04,
				   MAX77865_CNFG_04_CHG_CV_PRM, chg->cv_w);
		max77865_write_mode(chg, MODE_CHARGING);
	}

	if (chg->charge_control) {
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_AFC)
		/* the AFC/QC detection waits for the charger, once */
		if (!chg->hv_ready_sent) {
			chg->hv_ready_sent = true;
			max77865_hv_charger_ready();
		}
#endif
		max77865_policy(chg, &s);
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_PD)
		max77865_pd_step(chg, &s);
#endif
		/* safety time runs while the policy has CHG on and it is written */
		if (!chg->chg_block && chg->hw_mode == MODE_CHARGING)
			max77865_safety_time(chg, &s);
		max77865_apply(chg, &s);
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_PD)
		max77865_pd_request(chg);
#endif
#if IS_ENABLED(CONFIG_WIRELESS_CHARGER_IDT_MFC_HV)
		max77865_wls_hv_decide(chg, &s);
#endif
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_AFC)
		max77865_hv_decide(chg, &s);
#endif
	} else {
		if (chg->input == INPUT_NONE && s.input != INPUT_NONE)
			chg->cable_tries = 0;
		if (chg->input != s.input)
			chg->max_power = 0;
		chg->input = s.input;
		if (s.input != INPUT_NONE)
			max77865_currents(chg, &s, &chg->in_ma, &chg->chg_ma, &topoff);
	}
	read_ok = true;
	chg->cable = s.input == INPUT_WIRED ? s.cable :
		     (s.input == INPUT_WCIN ? s.cable : CABLE_NONE);
	plugged = s.input != INPUT_NONE;

	online = !!(s.int_ok & (MAX77865_CHGIN_OK | MAX77865_WCIN_OK)) && !chg->otg_on;
	if (chg->charge_control) {
		rep_status = chg->status;
		if (chg->fault && plugged)
			rep_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		rep_health = chg->health;
	} else {
		rep_status = online ? max77865_dtls_status(s.dtls1) :
				      POWER_SUPPLY_STATUS_DISCHARGING;
		rep_health = chg->hc;
	}

	/* the highest input power since attach, for time to full and the NTC limit */
	if (s.input != INPUT_NONE) {
		u32 power = (u32)max77865_in_mv(&s) * chg->in_ma / 1000;

		if (power > chg->max_power)
			chg->max_power = power;
	}

#if IS_ENABLED(CONFIG_CHARGER_MAX77865_TTF)
	/* the charge current the fuel gauge's time to full assumes, 0 = none */
	if (rep_status == POWER_SUPPLY_STATUS_CHARGING ||
	    (rep_status == POWER_SUPPLY_STATUS_FULL && s.cap >= 0 && s.cap != 100)) {
		int charge;

		if (s.cable == CABLE_HV9 || chg->max_power >= 2350 * 5)
			charge = 2350;
		else if (s.input == INPUT_WCIN && max77865_in_mv(&s) == 10000)
			charge = chg->otg_on ? 950 : 1200;
		else if (s.input == INPUT_WCIN)
			charge = 950;
		else
			charge = chg->max_power / 5;
		chg->ttf_ma = charge;
	} else {
		chg->ttf_ma = 0;
	}
#endif

	ac = s.input == INPUT_WIRED &&
	     (chg->cable == CABLE_DCP || chg->cable == CABLE_UNKNOWN ||
	      chg->cable == CABLE_AFC_PREPARE || chg->cable == CABLE_HV9 ||
	      (chg->cable == CABLE_PD && !s.pd_usb));
	usb = s.input == INPUT_WIRED &&
	      (chg->cable == CABLE_SDP || chg->cable == CABLE_CDP ||
	       (chg->cable == CABLE_PD && s.pd_usb));
	ac_ma = ac ? chg->in_ma : 0;
	usb_ma = usb ? chg->in_ma : 0;
	ac_mv = max77865_in_mv(&s);
	changed_chg = online != chg->rep_online || rep_status != chg->rep_status ||
		      rep_health != chg->rep_health;
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_TTF)
	if (chg->ttf_ma != chg->rep_ttf_ma) {
		changed_chg = true;
		chg->rep_ttf_ma = chg->ttf_ma;
	}
#endif
	changed_ac = ac != chg->rep_ac || ac_ma != chg->rep_ac_ma || ac_mv != chg->rep_ac_mv;
	changed_usb = usb != chg->rep_usb || usb_ma != chg->rep_usb_ma;
	chg->rep_online = online;
	chg->rep_status = rep_status;
	chg->rep_health = rep_health;
	chg->rep_ac = ac;
	chg->rep_usb = usb;
	chg->rep_ac_ma = ac_ma;
	chg->rep_usb_ma = usb_ma;
	chg->rep_ac_mv = ac_mv;

	if (chg->charge_control && plugged &&
	    (!chg->log_armed || time_after(jiffies, chg->last_log + 60 * HZ))) {
		const char *what = "ok";

		if (!s.ntc_valid)
			what = "ntc-error";
		else if (chg->health == H_OVERHEATLIMIT)
			what = "usb-hot";
		else if (chg->health == H_OVERHEAT)
			what = "hot";
		else if (chg->health == H_COLD)
			what = "cold";
		else if (chg->swelling != SWELLING_NONE)
			what = "swelling";
		else if (chg->mix_limit)
			what = "mix";
		dev_dbg(chg->dev, "ntc %d mC cable %s in %d mA chg %d mA %s\n", s.t_mc,
			s.input == INPUT_WCIN ? "wcin" : cable_name[chg->cable],
			chg->in_ma, chg->chg_ma, what);
		if (s.ntc_escape)
			dev_warn(chg->dev,
				 "ntc invalid, ntc_required=N: charging without battery temperature\n");
		chg->last_log = jiffies;
		chg->log_armed = true;
	}

rearm:
	/* while plugged, an RTC alarm every 30 s, or stay awake without one */
	if (plugged) {
		if (alarmtimer_get_rtcdev()) {
			alarm_start_relative(&chg->alarm, ktime_set(ALARM_PERIOD_S, 0));
			chg->wake_held = false;
		} else {
			if (!chg->no_rtc_logged)
				dev_warn(chg->dev, "no rtc alarm, staying awake while plugged\n");
			chg->no_rtc_logged = true;
			chg->wake_held = true;
		}
	} else {
		alarm_try_to_cancel(&chg->alarm);
		chg->wake_held = false;
	}

	requeued = delayed_work_pending(&chg->pass_dwork);
	if (!requeued) {
		unsigned long delay = PASS_PERIOD;

		if (pending && chg->cable_tries < CABLE_RETRIES) {
			chg->cable_tries++;
			delay = CABLE_RETRY;
		}
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_AFC)
		/* end the input step before AFC on time */
		if (plugged && chg->pre_afc_until &&
		    time_before(jiffies, chg->pre_afc_until))
			delay = min(delay, chg->pre_afc_until - jiffies);
#endif
		queue_delayed_work(system_freezable_wq, &chg->pass_dwork, delay);
	}
	if (chg->wake_held)
		pm_stay_awake(chg->dev);
	else if (!requeued)
		pm_relax(chg->dev);
	mutex_unlock(&chg->lock);

	if (!read_ok)
		return;
	if (changed_chg)
		power_supply_changed(chg->psy_chg);
	if (changed_ac)
		power_supply_changed(chg->psy_ac);
	if (changed_usb)
		power_supply_changed(chg->psy_usb);
}

static void max77865_alarm_cb(struct alarm *alarm, ktime_t now)
{
	struct max77865_charger *chg = container_of(alarm, struct max77865_charger, alarm);

	pm_stay_awake(chg->dev);
	mod_delayed_work(system_freezable_wq, &chg->pass_dwork, 0);
}

/*
 * CHGIN debounce as the vendor driver does it: read CHGIN_DTLS every 100 ms
 * until it reads the same more than 10 times in a row, unmask CHGIN_I, then
 * run one pass. A failed read counts as 0.
 */
static void max77865_chgin_work(struct work_struct *work)
{
	struct max77865_charger *chg = container_of(work, struct max77865_charger, chgin_work);
	unsigned int v, dtls, prev = 0xff;
	int stable = 0;

	pm_stay_awake(chg->dev);
	for (;;) {
		dtls = regmap_read(chg->regmap, MAX77865_CHG_REG_DETAILS_00, &v) ? 0 :
			FIELD_GET(MAX77865_CHGIN_DTLS, v);
		if (dtls == prev)
			stable++;
		else
			stable = 0;
		if (stable > 10)
			break;
		prev = dtls;
		msleep(100);
	}
	regmap_update_bits(chg->regmap, MAX77865_CHG_REG_INT_MASK, MAX77865_CHG_CHGIN, 0);

	mutex_lock(&chg->lock);
	if (!chg->stopping) {
		pm_stay_awake(chg->dev);	/* again: a pass may have relaxed meanwhile */
		mod_delayed_work(system_freezable_wq, &chg->pass_dwork, 0);
	} else {
		pm_relax(chg->dev);
	}
	mutex_unlock(&chg->lock);
}

#if IS_ENABLED(CONFIG_CHARGER_MAX77865_AICL)
/* AICL: lower the input limit in 100 mA steps while the charger reports AICL */
static void max77865_aicl_work(struct work_struct *work)
{
	struct max77865_charger *chg = container_of(to_delayed_work(work),
						    struct max77865_charger, aicl_dwork);
	unsigned int ok, reg, mask, v, orig_lim, lim;
	int n = 0, initial_ma, final_ma;
	bool wcin;

	mutex_lock(&chg->lock);
	if (!chg->charge_control || chg->fault || chg->input == INPUT_NONE)
		goto out_unmask;

	/* CHGPROT unlock */
	regmap_update_bits(chg->regmap, MAX77865_CHG_REG_CNFG_06,
			   MAX77865_CNFG_06_CHGPROT, MAX77865_CNFG_06_CHGPROT);

	wcin = chg->input == INPUT_WCIN;
	reg = wcin ? MAX77865_CHG_REG_CNFG_10 : MAX77865_CHG_REG_CNFG_09;
	mask = wcin ? MAX77865_CNFG_10_WCIN_LIM : MAX77865_CNFG_09_CHGIN_LIM;

	if (regmap_read(chg->regmap, reg, &v))
		goto out_unmask;
	orig_lim = v & mask;
	lim = orig_lim;
	initial_ma = (lim <= 4 ? 100 : lim * 25);

	if (regmap_read(chg->regmap, MAX77865_CHG_REG_INT_OK, &ok))
		goto out_unmask;

	while (!(ok & MAX77865_AICL_OK) && chg->input != INPUT_NONE) {
		if (++n >= 2) {
			/* down to 300 mA (code 12) */
			if (lim <= 12 || lim <= 4)
				break;
			lim = (lim > 16) ? lim - 4 : 12;
			regmap_update_bits(chg->regmap, reg, mask, lim);
			n = 0;
		}
		mutex_unlock(&chg->lock);
		msleep(50);
		mutex_lock(&chg->lock);
		if (chg->fault || chg->input == INPUT_NONE)
			goto out_unmask;
		if (regmap_read(chg->regmap, MAX77865_CHG_REG_INT_OK, &ok))
			goto out_unmask;
		if (lim <= 12)
			break;
	}

	final_ma = (lim <= 4 ? 100 : lim * 25);
	if (lim != orig_lim) {
		chg->aicl_ma = final_ma;
		chg->in_w = final_ma;
		dev_info(chg->dev, "aicl %d mA (from %d)\n", final_ma, initial_ma);
		mod_delayed_work(system_freezable_wq, &chg->pass_dwork, 0);
	}

out_unmask:
	regmap_update_bits(chg->regmap, MAX77865_CHG_REG_INT_MASK,
			   MAX77865_CHG_AICL, 0);
	mutex_unlock(&chg->lock);
}
#endif

/* under the lock */
static void max77865_otg_off(struct max77865_charger *chg)
{
	regmap_update_bits(chg->regmap, MAX77865_CHG_REG_CNFG_00,
			   MAX77865_CNFG_00_OTG_CTRL, 0);
	regmap_write(chg->regmap, MAX77865_CHG_REG_CNFG_11, 0x00);	/* 3.485 V */
	msleep(50);
	regmap_write(chg->regmap, MAX77865_CHG_REG_INT_MASK, chg->otg_saved_mask);
	/*
	 * Without charge control nothing else sets CHG and DISSKIP again: restore
	 * them, or the bootloader's charging stays off until a reboot.
	 */
	if (!chg->charge_control) {
		regmap_update_bits(chg->regmap, MAX77865_CHG_REG_CNFG_00, MAX77865_CNFG_00_CHG,
				   chg->otg_saved_cnfg00);
		regmap_update_bits(chg->regmap, MAX77865_CHG_REG_CNFG_12, MAX77865_CNFG_12_DISSKIP,
				   chg->otg_saved_cnfg12);
	}
	chg->otg_on = false;
	max77865_wpc_gate(chg, false, chg->wpc_wired);
}

static int max77865_otg_enable(struct regulator_dev *rdev)
{
	struct max77865_charger *chg = rdev_get_drvdata(rdev);
	int ret = 0;

	mutex_lock(&chg->lock);
	if (chg->otg_on || chg->lpcharge || chg->stopping)
		goto out;
	if (chg->fault == 2) {
		ret = -EIO;
		goto out;
	}
	/* every read before the first write: a failed read leaves CHG and DISSKIP as they were */
	if (!chg->charge_control) {
		ret = regmap_read(chg->regmap, MAX77865_CHG_REG_CNFG_00, &chg->otg_saved_cnfg00);
		ret = ret ?: regmap_read(chg->regmap, MAX77865_CHG_REG_CNFG_12,
					 &chg->otg_saved_cnfg12);
	}
	ret = ret ?: regmap_read(chg->regmap, MAX77865_CHG_REG_INT_MASK, &chg->otg_saved_mask);
	if (ret)
		goto out;
	/*
	 * CHG off first, as the vendor driver for an OTG cable: the pass writes no
	 * mode while boosting, so a CHARGING left from before would stay set.
	 */
	regmap_update_bits(chg->regmap, MAX77865_CHG_REG_CNFG_00, MAX77865_CNFG_00_CHG, 0);
	regmap_update_bits(chg->regmap, MAX77865_CHG_REG_CNFG_12, MAX77865_CNFG_12_DISSKIP, 0);
	chg->hw_mode = -1;
	max77865_wpc_gate(chg, true, chg->wpc_wired);
	regmap_update_bits(chg->regmap, MAX77865_CHG_REG_INT_MASK,
			   MAX77865_CHG_CHG | MAX77865_CHG_CHGIN | MAX77865_CHG_BYP,
			   MAX77865_CHG_CHG | MAX77865_CHG_CHGIN);
	regmap_write(chg->regmap, MAX77865_CHG_REG_CNFG_11, 0x16);	/* 5.020 V */
	regmap_update_bits(chg->regmap, MAX77865_CHG_REG_CNFG_00,	/* OTG off, boost on */
			   MAX77865_CNFG_00_OTG_CTRL, MAX77865_CNFG_00_BOOST);
	msleep(100);
	regmap_update_bits(chg->regmap, MAX77865_CHG_REG_CNFG_00,	/* OTG on, boost on */
			   MAX77865_CNFG_00_OTG_CTRL, MAX77865_CNFG_00_OTG_CTRL);
	chg->otg_on = true;
	mod_delayed_work(system_freezable_wq, &chg->pass_dwork, 0);
out:
	mutex_unlock(&chg->lock);
	return ret;
}

static int max77865_otg_disable(struct regulator_dev *rdev)
{
	struct max77865_charger *chg = rdev_get_drvdata(rdev);

	mutex_lock(&chg->lock);
	if (chg->otg_on) {
		max77865_otg_off(chg);
		if (!chg->stopping)
			mod_delayed_work(system_freezable_wq, &chg->pass_dwork, 0);
	}
	mutex_unlock(&chg->lock);
	return 0;
}

static int max77865_otg_is_enabled(struct regulator_dev *rdev)
{
	struct max77865_charger *chg = rdev_get_drvdata(rdev);

	return READ_ONCE(chg->otg_on);
}

static const struct regulator_ops max77865_otg_ops = {
	.enable = max77865_otg_enable,
	.disable = max77865_otg_disable,
	.is_enabled = max77865_otg_is_enabled,
};

static const struct regulator_desc max77865_otg_desc = {
	.name = "usb_otg_vbus",
	.of_match = "usb-otg-vbus",
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.ops = &max77865_otg_ops,
	.n_voltages = 1,
	.fixed_uV = 5020000,		/* CNFG_11 0x16 */
};

#if IS_ENABLED(CONFIG_CHARGER_MAX77865_SYSOVLO)
static irqreturn_t max77865_sysovlo_irq(int irq, void *data)
{
	struct max77865_charger *chg = data;

	disable_irq_nosync(irq);
	pm_wakeup_event(chg->dev, 5000);
	mutex_lock(&chg->lock);
	chg->sysovlo_armed = false;
	chg->sysovlo = true;
	if (!chg->stopping)
		mod_delayed_work(system_freezable_wq, &chg->pass_dwork, 0);
	mutex_unlock(&chg->lock);
	return IRQ_HANDLED;
}
#endif

static irqreturn_t max77865_chg_irq(int irq, void *data)
{
	struct max77865_charger *chg = data;
	unsigned int v, d2;
	bool oc = false;

	/* CHG_INT is clear-on-read */
	if (regmap_read(chg->regmap, MAX77865_CHG_REG_INT, &v))
		return IRQ_NONE;

	if (v & MAX77865_CHG_CHGIN) {
		/* masked until the work has debounced it, as the vendor driver */
		regmap_update_bits(chg->regmap, MAX77865_CHG_REG_INT_MASK, MAX77865_CHG_CHGIN,
				   MAX77865_CHG_CHGIN);
		pm_stay_awake(chg->dev);
		queue_work(system_freezable_wq, &chg->chgin_work);
	}

	if (v & MAX77865_CHG_BYP) {
		/* bypass over-current while boosting */
		mutex_lock(&chg->lock);
		if (chg->otg_on && !regmap_read(chg->regmap, MAX77865_CHG_REG_DETAILS_02, &d2) &&
		    (FIELD_GET(MAX77865_BYP_DTLS, d2) & 0x1)) {
			max77865_otg_off(chg);
			oc = true;
			if (!chg->stopping)
				mod_delayed_work(system_freezable_wq, &chg->pass_dwork, 0);
		}
		mutex_unlock(&chg->lock);
		if (oc) {
			dev_warn(chg->dev, "otg bypass over-current, boost off\n");
			regulator_notifier_call_chain(chg->otg_rdev, REGULATOR_EVENT_OVER_CURRENT,
						      NULL);
		}
	}

#if IS_ENABLED(CONFIG_CHARGER_MAX77865_AICL)
	if (v & MAX77865_CHG_AICL) {
		bool skip_aicl = false;

		mutex_lock(&chg->lock);
		if (chg->cable == CABLE_HV9 || chg->cable == CABLE_AFC_PREPARE)
			skip_aicl = true;
		mutex_unlock(&chg->lock);

		if (!skip_aicl) {
			regmap_update_bits(chg->regmap, MAX77865_CHG_REG_INT_MASK,
					   MAX77865_CHG_AICL, MAX77865_CHG_AICL);
			queue_delayed_work(system_freezable_wq, &chg->aicl_dwork,
					   msecs_to_jiffies(50));
		}
	}
#endif

	return IRQ_HANDLED;
}

static enum power_supply_property max77865_chg_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_PRESENT,
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_TTF)
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
#endif
};

static const int max77865_psy_health[] = {
	[H_GOOD] = POWER_SUPPLY_HEALTH_GOOD,
	[H_OVERHEAT] = POWER_SUPPLY_HEALTH_OVERHEAT,
	[H_COLD] = POWER_SUPPLY_HEALTH_COLD,
	[H_OVERHEATLIMIT] = POWER_SUPPLY_HEALTH_OVERHEAT,
	[H_OVERVOLTAGE] = POWER_SUPPLY_HEALTH_OVERVOLTAGE,
	[H_UNDERVOLTAGE] = POWER_SUPPLY_HEALTH_UNDERVOLTAGE,
	[H_SAFETY_TIMER] = POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE,
	[H_VSYS_OVP] = POWER_SUPPLY_HEALTH_OVERVOLTAGE,
	[H_VBAT_OVP] = POWER_SUPPLY_HEALTH_OVERVOLTAGE,
};

static int max77865_chg_get_property(struct power_supply *psy, enum power_supply_property psp,
				     union power_supply_propval *val)
{
	struct max77865_charger *chg = power_supply_get_drvdata(psy);
	unsigned int v, v2;
	int ret;

	switch (psp) {
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_TTF)
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		mutex_lock(&chg->lock);
		val->intval = chg->ttf_ma * 1000;
		mutex_unlock(&chg->lock);
		return 0;
#endif
	case POWER_SUPPLY_PROP_ONLINE:
		ret = regmap_read(chg->regmap, MAX77865_CHG_REG_INT_OK, &v);
		if (ret)
			return ret;
		val->intval = !!(v & (MAX77865_CHGIN_OK | MAX77865_WCIN_OK)) &&
			      !READ_ONCE(chg->otg_on);
		return 0;
	case POWER_SUPPLY_PROP_STATUS:
		mutex_lock(&chg->lock);
		val->intval = chg->rep_status;
		mutex_unlock(&chg->lock);
		return 0;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		ret = regmap_read(chg->regmap, MAX77865_CHG_REG_CNFG_00, &v);
		ret = ret ?: regmap_read(chg->regmap, MAX77865_CHG_REG_INT_OK, &v2);
		if (ret)
			return ret;
		if ((v & MAX77865_CNFG_00_CHG) &&
		    (v2 & (MAX77865_CHGIN_OK | MAX77865_WCIN_OK))) {
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_AICL)
			if (chg->input == INPUT_WIRED && chg->aicl_ma && chg->aicl_ma <= 400)
				val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
			else
#endif
				val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
		} else {
			val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
		}
		return 0;
	case POWER_SUPPLY_PROP_HEALTH:
		mutex_lock(&chg->lock);
		val->intval = max77865_psy_health[chg->rep_health];
		mutex_unlock(&chg->lock);
		return 0;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = regmap_read(chg->regmap, MAX77865_CHG_REG_CNFG_09, &v);
		if (ret)
			return ret;
		v &= MAX77865_CNFG_09_CHGIN_LIM;
		val->intval = (v <= 4 ? 100 : v >= 0x7F ? 3175 : v * 25) * 1000;
		return 0;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = regmap_read(chg->regmap, MAX77865_CHG_REG_CNFG_02, &v);
		if (ret)
			return ret;
		v &= MAX77865_CNFG_02_CHG_CC;
		val->intval = (v <= 2 ? 100 : v * 50) * 1000;
		return 0;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:	/* 0.1 mV */
		ret = regmap_read(chg->regmap, MAX77865_CHG_REG_CNFG_04, &v);
		if (ret)
			return ret;
		val->intval = ((v & MAX77865_CNFG_04_CHG_CV_PRM) * 125 + 40500) * 100;
		return 0;
	case POWER_SUPPLY_PROP_PRESENT:
		ret = regmap_read(chg->regmap, MAX77865_CHG_REG_INT_OK, &v);
		ret = ret ?: regmap_read(chg->regmap, MAX77865_CHG_REG_DETAILS_00, &v2);
		if (ret)
			return ret;
		val->intval = (v & MAX77865_BATP_OK) || !(v2 & MAX77865_BATP_DTLS);
		return 0;
	default:
		return -EINVAL;
	}
}

static enum power_supply_property max77865_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_USB_SS)
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,	/* set by dwc3 */
#endif
};

static enum power_supply_property max77865_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_AFC) || \
	IS_ENABLED(CONFIG_CHARGER_MAX77865_QC) || \
	IS_ENABLED(CONFIG_CHARGER_MAX77865_PD)
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
#endif
};

static int max77865_in_get_property(struct power_supply *psy, enum power_supply_property psp,
				    union power_supply_propval *val)
{
	struct max77865_charger *chg = power_supply_get_drvdata(psy);
	bool ac = psy->desc->type == POWER_SUPPLY_TYPE_MAINS;

	mutex_lock(&chg->lock);
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = ac ? chg->rep_ac : chg->rep_usb;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = (ac ? chg->rep_ac_ma : chg->rep_usb_ma) * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = (ac && chg->rep_ac ? chg->rep_ac_mv : 5000) * 1000;
		break;
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_USB_SS)
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		val->intval = chg->usb_ss ? 900000 : 500000;
		break;
#endif
	default:
		mutex_unlock(&chg->lock);
		return -EINVAL;
	}
	mutex_unlock(&chg->lock);
	return 0;
}

#if IS_ENABLED(CONFIG_WIRELESS_CHARGER_IDT_MFC_NV)
/* the wireless receiver's supply changed */
static void max77865_ext_changed(struct power_supply *psy)
{
	struct max77865_charger *chg = power_supply_get_drvdata(psy);

	mutex_lock(&chg->lock);
	if (chg->works_ready && !chg->stopping)
		mod_delayed_work(system_freezable_wq, &chg->pass_dwork, 0);
	mutex_unlock(&chg->lock);
}
#endif

static const struct power_supply_desc max77865_chg_desc = {
	.name = "max77865-charger",
	.type = POWER_SUPPLY_TYPE_UNKNOWN,
	.properties = max77865_chg_props,
	.num_properties = ARRAY_SIZE(max77865_chg_props),
	.get_property = max77865_chg_get_property,
#if IS_ENABLED(CONFIG_WIRELESS_CHARGER_IDT_MFC_NV)
	.external_power_changed = max77865_ext_changed,
#endif
};

/* the supply names the vendor kernel used */
static const struct power_supply_desc max77865_ac_desc = {
	.name = "ac",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = max77865_ac_props,
	.num_properties = ARRAY_SIZE(max77865_ac_props),
	.get_property = max77865_in_get_property,
};

#if IS_ENABLED(CONFIG_CHARGER_MAX77865_USB_SS)
/*
 * The current the USB host configured, set by dwc3. Values below 500 mA do not
 * lower the SDP current.
 */
static int max77865_usb_set_property(struct power_supply *psy, enum power_supply_property psp,
				     const union power_supply_propval *val)
{
	struct max77865_charger *chg = power_supply_get_drvdata(psy);
	bool ss = val->intval >= 900000;

	if (psp != POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT)
		return -EINVAL;
	mutex_lock(&chg->lock);
	if (ss != chg->usb_ss) {
		chg->usb_ss = ss;
		dev_dbg(chg->dev, "usb configured %d mA\n", val->intval / 1000);
		if (!chg->stopping)
			mod_delayed_work(system_freezable_wq, &chg->pass_dwork, 0);
	}
	mutex_unlock(&chg->lock);
	return 0;
}

/* read-only in sysfs: userspace cannot select 900 mA */
static int max77865_usb_writeable(struct power_supply *psy, enum power_supply_property psp)
{
	return 0;
}
#endif

static const struct power_supply_desc max77865_usb_desc = {
	.name = "usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = max77865_usb_props,
	.num_properties = ARRAY_SIZE(max77865_usb_props),
	.get_property = max77865_in_get_property,
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_USB_SS)
	.set_property = max77865_usb_set_property,
	.property_is_writeable = max77865_usb_writeable,
#endif
};

struct max77865_readback {
	u8 reg, mask, val;
};

/* what the init below writes, read back before charging is allowed */
static const struct max77865_readback max77865_readback[] = {
	{ MAX77865_CHG_REG_CNFG_06, 0x6C, 0x6C },
	{ MAX77865_CHG_REG_CNFG_01, 0xFF, 0x38 },
	{ MAX77865_CHG_REG_CNFG_02, 0xC0, 0xC0 },
	{ MAX77865_CHG_REG_CNFG_05, 0x07, 0x04 },
	{ MAX77865_CHG_REG_CNFG_03, 0xFF, 0x18 },
	{ MAX77865_CHG_REG_CNFG_04, 0x3F, CV_CODE_NORMAL },
	{ MAX77865_CHG_REG_CNFG_12, 0x1E, 0x12 },
	{ MAX77865_CHG_REG_CNFG_07, 0x04, 0x04 },
	{ MAX77865_CHG_REG_CNFG_00, 0x10, 0x10 },
};

static void max77865_chg_init(struct max77865_charger *chg, struct regmap *core)
{
	struct regmap *map = chg->regmap;
	unsigned int v;
	int i, jig = 0;

	/* CHGPROT unlock, 10 tries 20 ms apart */
	for (i = 0; i < 10; i++) {
		if (!regmap_read(map, MAX77865_CHG_REG_CNFG_06, &v) &&
		    FIELD_GET(MAX77865_CNFG_06_CHGPROT, v) == 0x03)
			break;
		regmap_update_bits(map, MAX77865_CHG_REG_CNFG_06, MAX77865_CNFG_06_CHGPROT,
				   MAX77865_CNFG_06_CHGPROT);
		msleep(20);
	}
	/* CHGPROT and the slowest LX slope */
	regmap_update_bits(map, MAX77865_CHG_REG_CNFG_06, 0x6C, 0x6C);
	/* restart threshold and fast-charge timer off */
	regmap_write(map, MAX77865_CHG_REG_CNFG_01, 0x38);
	/* OTG current limit field [7:6] = 3 */
	regmap_update_bits(map, MAX77865_CHG_REG_CNFG_02, 0xC0, 0xC0);
	/* BAT to SYS OCP */
	regmap_update_bits(map, MAX77865_CHG_REG_CNFG_05, 0x07, 0x04);
	/* top-off 150 mA, 30 min */
	regmap_write(map, MAX77865_CHG_REG_CNFG_03, 0x18);
	/* float voltage 4350 mV */
	regmap_update_bits(map, MAX77865_CHG_REG_CNFG_04, MAX77865_CNFG_04_CHG_CV_PRM,
			   CV_CODE_NORMAL);
	/* VCHGIN and WCIN regulation 4.3 V, UVLO 4.5 V */
	regmap_update_bits(map, MAX77865_CHG_REG_CNFG_12,
			   MAX77865_CNFG_12_VCHGIN_REG | MAX77865_CNFG_12_WCIN_REG, 0x12);
	regmap_update_bits(map, MAX77865_CHG_REG_CNFG_07, MAX77865_CNFG_07_FMBST,
			   MAX77865_CNFG_07_FMBST);
	/* the fuel gauge is supplied from SYS with a JIG cable */
	if (chg->jig)
		jig = gpiod_get_value_cansleep(chg->jig) > 0;
	regmap_update_bits(map, MAX77865_CHG_REG_CNFG_07, MAX77865_CNFG_07_FGSRC,
			   jig ? MAX77865_CNFG_07_FGSRC : 0);
	regmap_update_bits(map, MAX77865_CHG_REG_CNFG_00, MAX77865_CNFG_00_WDTEN,
			   MAX77865_CNFG_00_WDTEN);
	/* active discharge */
	regmap_update_bits(core, MAX77865_PMIC_REG_MAINCTRL1, MAX77865_MAINCTRL1_ADEN,
			   MAX77865_MAINCTRL1_ADEN);

	/* on a mismatch, charging stays off (fault mode) */
	for (i = 0; i < ARRAY_SIZE(max77865_readback); i++) {
		const struct max77865_readback *r = &max77865_readback[i];
		int ret = regmap_read(map, r->reg, &v);

		if (ret || (v & r->mask) != r->val) {
			if (ret)
				dev_err(chg->dev, "readback CNFG 0x%02x failed (%d)\n",
					r->reg, ret);
			dev_err(chg->dev,
				"readback CNFG 0x%02x=0x%02x expected 0x%02x/0x%02x, charging off (fault mode)\n",
				r->reg, ret ? 0 : v, r->mask, r->val);
			chg->fault = 1;
			break;
		}
	}
	if (!chg->fault)
		return;

	/* charging off, buck kept */
	regmap_update_bits(map, MAX77865_CHG_REG_CNFG_00, MAX77865_CNFG_00_CHG, 0);
	regmap_update_bits(map, MAX77865_CHG_REG_CNFG_12, MAX77865_CNFG_12_DISSKIP, 0);
	/* a failed read counts as still on */
	if (!regmap_read(map, MAX77865_CHG_REG_CNFG_00, &v) && !(v & MAX77865_CNFG_00_CHG)) {
		chg->hw_mode = MODE_CHARGING_OFF;
		dev_warn(chg->dev, "fault mode, CNFG_00=0x%02x\n", v);
		return;
	}
	chg->fault = 2;
	dev_err(chg->dev,
		"cannot switch charging off (CNFG_00=0x%02x), watchdog kick stopped\n", v);
}

static int max77865_poweroff_prepare(struct sys_off_data *data)
{
	struct max77865_charger *chg = data->cb_data;
	unsigned int v;

	/* read before the PMIC is shut down */
	chg->poweroff_restart = !regmap_read(chg->regmap, MAX77865_CHG_REG_INT_OK, &v) &&
				(v & (MAX77865_CHGIN_OK | MAX77865_WCIN_OK)) && !chg->lpcharge;
	return NOTIFY_DONE;
}

static int max77865_poweroff(struct sys_off_data *data)
{
	struct max77865_charger *chg = data->cb_data;

	if (!chg->poweroff_restart)
		return NOTIFY_DONE;

	dev_emerg(chg->dev, "power-off with charger online, restarting to LP charging\n");
	regmap_write(chg->pmu, EXYNOS8895_PMU_INFORM2, SBOOT_INFORM2_POWER_OFF);
	regmap_write(chg->pmu, EXYNOS8895_PMU_SWRESET, 0x1);
	mdelay(1000);
	dev_emerg(chg->dev, "SWRESET did not reset, falling through to power-off\n");
	return NOTIFY_DONE;
}

static int max77865_mode_show(struct seq_file *m, void *unused)
{
	struct max77865_charger *chg = m->private;
	static const char * const stage[] = { "none", "1st", "2nd" };

	seq_printf(m, "%s\n", stage[READ_ONCE(chg->mode)]);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(max77865_mode);

static void max77865_debugfs_remove(void *data)
{
	debugfs_remove_recursive(data);
}

static int max77865_debugfs_init(struct max77865_charger *chg)
{
	struct dentry *d = debugfs_create_dir("max77865-charger", NULL);
	/*
	 * The overrides replace T, Tu, Tc and Hc and can stop the watchdog kick:
	 * with charge control they could hide a hot battery, so they are read-only.
	 */
	umode_t ovr = !chg->charge_control ? 0600 : 0400;

	debugfs_create_u32("budget_ms", 0400, d, &chg->budget_ms);
	debugfs_create_file("mode", 0400, d, chg, &max77865_mode_fops);
	debugfs_create_u32("status", 0400, d, &chg->status);
	debugfs_create_u32("swelling", 0400, d, &chg->swelling);
	debugfs_create_u32("health", 0400, d, &chg->health);
	debugfs_create_u32("hc", 0400, d, &chg->hc);
	debugfs_create_u32("fault", 0400, d, &chg->fault);
	debugfs_create_u32("chg_mode", 0400, d, &chg->chg_mode);
	debugfs_create_bool("mix_limit", 0400, d, &chg->mix_limit);
	debugfs_create_bool("ntc_valid", 0400, d, &chg->ntc_valid);
	debugfs_create_u32("override_usb_mc", ovr, d, &chg->override_usb_mc);
	debugfs_create_u32("override_chg_mc", ovr, d, &chg->override_chg_mc);
	debugfs_create_u32("override_t_mc", ovr, d, &chg->override_t_mc);
	debugfs_create_u32("override_hc", ovr, d, &chg->override_hc);
	debugfs_create_bool("wdt_kick", ovr, d, &chg->wdt_kick);
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_AICL)
	debugfs_create_u32("aicl_ma", 0400, d, &chg->aicl_ma);
#endif
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_TTF)
	debugfs_create_u32("ttf_ma", 0400, d, &chg->ttf_ma);
#endif
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_AGE_FORECAST)
	debugfs_create_u32("age_step", 0400, d, &chg->age_step);
#endif
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_AFC) || IS_ENABLED(CONFIG_CHARGER_MAX77865_QC) || \
	IS_ENABLED(CONFIG_CHARGER_MAX77865_TYPEC_RP) || IS_ENABLED(CONFIG_CHARGER_MAX77865_PD)
	debugfs_create_bool("chg_high_temp", 0400, d, &chg->chg_high_temp_limit);
#endif
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_PD)
	debugfs_create_file("pd_list", 0400, d, chg, &max77865_pd_list_fops);
#endif
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_AFC)
	debugfs_create_u32("hv_mv", 0400, d, &chg->hv_mv_dbg);
#endif

	return devm_add_action_or_reset(chg->dev, max77865_debugfs_remove, d);
}

static void max77865_fg_put(void *data)
{
	struct max77865_charger *chg = data;

	if (chg->fg)
		power_supply_put(chg->fg);
}

/* runs before the works are cancelled: nothing re-arms the pass after this */
static void max77865_stop(void *data)
{
	struct max77865_charger *chg = data;

#if IS_ENABLED(CONFIG_WIRELESS_CHARGER_IDT_MFC_NV)
	WRITE_ONCE(g_max77865_charger, NULL);
#endif
#if IS_ENABLED(CONFIG_WIRELESS_CHARGER_IDT_MFC_HV)
	if (chg->wpc_hv_granted) {
		chg->wpc_hv_granted = false;
		idt_mfc_wpc_hv_grant(false);
	}
#endif
	mutex_lock(&chg->lock);
	chg->stopping = true;
	mutex_unlock(&chg->lock);
	alarm_cancel(&chg->alarm);
}

static void max77865_put_core(void *data)
{
	struct i2c_client *core = data;

	put_device(&core->dev);
}

static struct regmap *max77865_core_regmap(struct device *dev, struct i2c_client **core)
{
	struct device_node *np = of_parse_phandle(dev->of_node, "maxim,pmic", 0);
	struct max77693_dev *max;

	if (!np)
		return ERR_PTR(-ENODEV);
	*core = of_find_i2c_device_by_node(np);
	of_node_put(np);
	if (!*core)
		return ERR_PTR(-EPROBE_DEFER);
	max = i2c_get_clientdata(*core);
	if (!max || !max->regmap) {
		put_device(&(*core)->dev);
		return ERR_PTR(-EPROBE_DEFER);
	}
	return max->regmap;
}

#if MAX77865_CCIC
/*
 * The S2MM005 calls the chain after releasing its own lock, so chg->lock ->
 * S2MM005 lock (a PDO request from the pass) is the only nesting.
 */
static int max77865_ccic_notify(struct notifier_block *nb, unsigned long action, void *data)
{
	struct max77865_charger *chg = container_of(nb, struct max77865_charger, ccic_nb);
	const struct s2mm005_pd_status *st = data;

	mutex_lock(&chg->lock);
	if (action == S2MM005_PD_EVENT_DETACH || !st)
		memset(&chg->pd, 0, sizeof(chg->pd));
	else
		chg->pd = *st;
	chg->pd_event = action;
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_PD)
	chg->pd_new = true;
#endif
	if (!chg->stopping)
		mod_delayed_work(system_freezable_wq, &chg->pass_dwork, 0);
	mutex_unlock(&chg->lock);
	return NOTIFY_OK;
}

static void max77865_ccic_unregister(void *data)
{
	struct max77865_charger *chg = data;

	s2mm005_unregister_notifier(&chg->ccic_nb);
}
#endif

static int max77865_charger_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct power_supply_config psy_cfg = {};
	struct regulator_config reg_cfg = {};
	struct regmap *core_map = NULL;
	struct i2c_client *core = NULL;
	struct max77865_charger *chg;
	unsigned int cnfg[13];
	int i, ret;

	chg = devm_kzalloc(dev, sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;

	chg->dev = dev;
	chg->client = client;
	mutex_init(&chg->lock);
	i2c_set_clientdata(client, chg);
	chg->wdt_kick = true;
	chg->hw_mode = -1;
	chg->in_w = -1;
	chg->chg_w = -1;
	chg->topoff_w = -1;
	chg->cv_w = -1;
	chg->cv_want = CV_CODE_NORMAL;
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_AGE_FORECAST)
	chg->recharge_condition_vcell = RECHG_VCELL;
	chg->full_condition_vcell = FULL_COND_VCELL;
	chg->full_condition_soc = FULL_COND_SOC;
	chg->age_step = 0;
	chg->cv_normal = CV_CODE_NORMAL;
#endif
	chg->status = POWER_SUPPLY_STATUS_DISCHARGING;
	chg->rep_status = POWER_SUPPLY_STATUS_DISCHARGING;
	chg->rep_ac_mv = 5000;
	chg->mode = STAGE_NONE;
	chg->chg_mode = MODE_CHARGING_OFF;
	chg->chg_block = true;
	chg->budget_ms = EXPIRED_MS;
	chg->lpcharge = !!strstr(saved_command_line, "androidboot.mode=charger");
	chg->charge_control = of_property_read_bool(dev->of_node, "samsung,charge-control");
	if (chg->charge_control && !IS_ENABLED(CONFIG_CHARGER_MAX77865_CHARGE_CONTROL)) {
		dev_warn(dev, "samsung,charge-control ignored: CONFIG_CHARGER_MAX77865_CHARGE_CONTROL is not set\n");
		chg->charge_control = false;
	}

	ret = of_property_read_string(dev->of_node, "samsung,battery-ntc", &chg->tz_name);
	if (ret)
		return dev_err_probe(dev, ret, "no samsung,battery-ntc\n");

	chg->regmap = devm_regmap_init_i2c(client, &max77865_chg_regmap_config);
	if (IS_ERR(chg->regmap))
		return dev_err_probe(dev, PTR_ERR(chg->regmap), "regmap init failed\n");

	chg->pmu = syscon_regmap_lookup_by_phandle(dev->of_node, "samsung,pmu");
	if (IS_ERR(chg->pmu))
		return dev_err_probe(dev, PTR_ERR(chg->pmu), "no PMU syscon\n");

	/* only read, never driven */
	chg->jig = devm_gpiod_get_optional(dev, "jig", GPIOD_ASIS);
	if (IS_ERR(chg->jig))
		return dev_err_probe(dev, PTR_ERR(chg->jig), "jig gpio\n");

	if (chg->charge_control) {
		core_map = max77865_core_regmap(dev, &core);
		if (IS_ERR(core_map))
			return dev_err_probe(dev, PTR_ERR(core_map), "no MAX77865 core\n");
		ret = devm_add_action_or_reset(dev, max77865_put_core, core);
		if (ret)
			return ret;
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_SYSOVLO)
		{
			struct max77693_dev *max = i2c_get_clientdata(core);

			if (max->sys_irq_data)
				chg->sysovlo_irq = regmap_irq_get_virq(max->sys_irq_data, 5);
		}
#endif
	}

	psy_cfg.drv_data = chg;
	psy_cfg.fwnode = dev_fwnode(dev);	/* the fuel gauge's power-supplies */
	chg->psy_chg = devm_power_supply_register(dev, &max77865_chg_desc, &psy_cfg);
	if (IS_ERR(chg->psy_chg))
		return dev_err_probe(dev, PTR_ERR(chg->psy_chg), "charger supply\n");
	psy_cfg.fwnode = NULL;
	chg->psy_ac = devm_power_supply_register(dev, &max77865_ac_desc, &psy_cfg);
	if (IS_ERR(chg->psy_ac))
		return dev_err_probe(dev, PTR_ERR(chg->psy_ac), "ac supply\n");
	chg->psy_usb = devm_power_supply_register(dev, &max77865_usb_desc, &psy_cfg);
	if (IS_ERR(chg->psy_usb))
		return dev_err_probe(dev, PTR_ERR(chg->psy_usb), "usb supply\n");

	ret = devm_add_action_or_reset(dev, max77865_fg_put, chg);
	if (ret)
		return ret;

	ret = max77865_debugfs_init(chg);
	if (ret)
		return ret;

	ret = devm_device_init_wakeup(dev);
	if (ret)
		return ret;

	/* the works exist before the regulator, whose ops queue the pass */
	alarm_init(&chg->alarm, ALARM_BOOTTIME, max77865_alarm_cb);
	ret = devm_delayed_work_autocancel(dev, &chg->pass_dwork, max77865_pass_work);
	if (ret)
		return ret;
	ret = devm_work_autocancel(dev, &chg->chgin_work, max77865_chgin_work);
	if (ret)
		return ret;
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_AICL)
	ret = devm_delayed_work_autocancel(dev, &chg->aicl_dwork, max77865_aicl_work);
	if (ret)
		return ret;
#endif
#if IS_ENABLED(CONFIG_WIRELESS_CHARGER_IDT_MFC_NV)
	ret = devm_delayed_work_autocancel(dev, &chg->wc_current_dwork, max77865_wc_current_work);
	if (ret)
		return ret;
#endif
#if IS_ENABLED(CONFIG_WIRELESS_CHARGER_IDT_MFC_NV)
	mutex_lock(&chg->lock);
	chg->wc_pre_ma = WC_CURRENT_START;
	chg->works_ready = true;
	mutex_unlock(&chg->lock);
#endif
	ret = devm_add_action_or_reset(dev, max77865_stop, chg);
	if (ret)
		return ret;
#if MAX77865_CCIC
	/* the S2MM005 binds after the OTG regulator below, its VBUS supply */
	chg->ccic_nb.notifier_call = max77865_ccic_notify;
	ret = s2mm005_register_notifier(&chg->ccic_nb);
	if (ret)
		return dev_err_probe(dev, ret, "S2MM005 notifier\n");
	ret = devm_add_action_or_reset(dev, max77865_ccic_unregister, chg);
	if (ret)
		return ret;
#endif
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_AFC)
	/* the chain is static: this works before the AFC/QC device binds, or without it */
	chg->hv_nb.notifier_call = max77865_hv_notify;
	ret = max77865_hv_register_notifier(&chg->hv_nb);
	if (ret)
		return dev_err_probe(dev, ret, "MUIC HV notifier\n");
	ret = devm_add_action_or_reset(dev, max77865_hv_unregister, chg);
	if (ret)
		return ret;
#endif

	reg_cfg.dev = dev;
	reg_cfg.driver_data = chg;
	chg->otg_rdev = devm_regulator_register(dev, &max77865_otg_desc, &reg_cfg);
	if (IS_ERR(chg->otg_rdev))
		return dev_err_probe(dev, PTR_ERR(chg->otg_rdev), "otg regulator\n");

	/* before the PMIC shutdown over ACPM at SYS_OFF_PRIO_DEFAULT */
	ret = devm_register_sys_off_handler(dev, SYS_OFF_MODE_POWER_OFF_PREPARE, SYS_OFF_PRIO_HIGH,
					    max77865_poweroff_prepare, chg);
	if (ret)
		return dev_err_probe(dev, ret, "power-off prepare handler\n");
	ret = devm_register_sys_off_handler(dev, SYS_OFF_MODE_POWER_OFF, SYS_OFF_PRIO_HIGH,
					    max77865_poweroff, chg);
	if (ret)
		return dev_err_probe(dev, ret, "power-off handler\n");

	/* CNFG_00..CNFG_12 as the bootloader left them */
	for (i = 0; i < ARRAY_SIZE(cnfg); i++) {
		ret = regmap_read(chg->regmap, MAX77865_CHG_REG_CNFG_00 + i, &cnfg[i]);
		if (ret)
			return dev_err_probe(dev, ret, "CNFG read failed\n");
	}

	/* after everything that can fail or defer; a failed IRQ request undoes WDTEN */
	if (chg->charge_control) {
		max77865_chg_init(chg, core_map);
		if (!chg->fault) {
			chg->cv_w = CV_CODE_NORMAL;
			dev_info(dev, "charge control, float %d mV\n",
				 CV_CODE_TO_MV(CV_CODE_NORMAL));
		}
	} else {
		dev_info(dev, "monitor-only mode, CNFG_00=0x%02x WDTEN=%d\n", cnfg[0],
			 !!(cnfg[0] & MAX77865_CNFG_00_WDTEN));
	}

	/* only CHGIN_I unmasked, and AICL_I with the AICL policy */
	ret = regmap_write(chg->regmap, MAX77865_CHG_REG_INT_MASK,
			   IS_ENABLED(CONFIG_CHARGER_MAX77865_AICL) && chg->charge_control ?
			   0x3F : 0xBF);
	if (ret)
		return dev_err_probe(dev, ret, "CHG_INT_MASK write failed\n");

	ret = devm_request_threaded_irq(dev, client->irq, NULL, max77865_chg_irq,
					IRQF_ONESHOT,
					"max77865-charger", chg);
	if (ret) {
		/* nothing will kick the watchdog: the shutdown value */
		if (chg->charge_control && chg->fault != 2)
			regmap_write(chg->regmap, MAX77865_CHG_REG_CNFG_00, 0x04);
		return dev_err_probe(dev, ret, "irq %d\n", client->irq);
	}

	/* CHGIN_I must wake the system, or a detach goes unseen until resume */
	if (enable_irq_wake(client->irq) < 0)
		dev_warn(dev, "failed to enable wakeup source\n");

#if IS_ENABLED(CONFIG_CHARGER_MAX77865_SYSOVLO)
	/* requested masked and armed on the first removal, as the vendor driver */
	if (chg->sysovlo_irq > 0) {
		regmap_update_bits(core_map, MAX77865_PMIC_REG_MAINCTRL1, MAX77865_MAINCTRL1_BIASEN,
				   MAX77865_MAINCTRL1_BIASEN);
		ret = devm_request_threaded_irq(dev, chg->sysovlo_irq, NULL, max77865_sysovlo_irq,
						IRQF_ONESHOT | IRQF_NO_AUTOEN,
						"max77865-sysovlo", chg);
		if (ret)
			return dev_err_probe(dev, ret, "sysovlo irq %d\n", chg->sysovlo_irq);
		enable_irq_wake(chg->sysovlo_irq);
	}
#endif

#if IS_ENABLED(CONFIG_WIRELESS_CHARGER_IDT_MFC_NV)
	/* charge control and fault mode are final now: let the receiver follow them */
	WRITE_ONCE(g_max77865_charger, chg);
	idt_mfc_wpc_rx_update();
#endif
	queue_delayed_work(system_freezable_wq, &chg->pass_dwork, 0);
	return 0;
}

/*
 * Stop the policy and the boost; with charge control, the vendor driver's
 * shutdown values, so that the bootloader charges with the watchdog off.
 */
static void max77865_charger_shutdown(struct i2c_client *client)
{
	struct max77865_charger *chg = i2c_get_clientdata(client);

	max77865_stop(chg);
	cancel_work_sync(&chg->chgin_work);
	cancel_delayed_work_sync(&chg->pass_dwork);
#if IS_ENABLED(CONFIG_WIRELESS_CHARGER_IDT_MFC_NV)
	cancel_delayed_work_sync(&chg->wc_current_dwork);
#endif

#if IS_ENABLED(CONFIG_CHARGER_MAX77865_AFC)
	/*
	 * Do not leave an AFC/QC adapter at 9 V for the bootloader; the vendor MUIC
	 * driver opened D+/D- at shutdown. The pass is cancelled: no lock.
	 */
	if (chg->charge_control) {
		int hv = READ_ONCE(chg->hv_event);

		if (hv == MAX77865_HV_EV_9V || hv == MAX77865_HV_EV_ERR_V)
			max77865_hv_set_voltage(5000);
	}
#endif

	mutex_lock(&chg->lock);
	if (chg->otg_on)
		max77865_otg_off(chg);
	if (chg->charge_control && chg->fault != 2) {
		regmap_write(chg->regmap, MAX77865_CHG_REG_CNFG_00, 0x04);
		regmap_write(chg->regmap, MAX77865_CHG_REG_CNFG_09, 0x0F);
		regmap_write(chg->regmap, MAX77865_CHG_REG_CNFG_10, 0x10);
		regmap_write(chg->regmap, MAX77865_CHG_REG_CNFG_12, 0x60);
	}
	mutex_unlock(&chg->lock);
}

static const struct of_device_id max77865_charger_of_match[] = {
	{ .compatible = "maxim,max77865-charger" },
	{ }
};
MODULE_DEVICE_TABLE(of, max77865_charger_of_match);

static struct i2c_driver max77865_charger_driver = {
	.driver = {
		.name = "max77865-charger",
		.of_match_table = max77865_charger_of_match,
		.suppress_bind_attrs = true,	/* a charge policy is not unbound at run time */
	},
	.probe = max77865_charger_probe,
	.shutdown = max77865_charger_shutdown,
};
module_i2c_driver(max77865_charger_driver);

MODULE_DESCRIPTION("Maxim MAX77865 charger driver");
MODULE_LICENSE("GPL");
