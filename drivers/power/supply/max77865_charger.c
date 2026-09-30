// SPDX-License-Identifier: GPL-2.0+
/*
 * Maxim MAX77865 charger (Samsung Galaxy Note 8, greatlte)
 *
 * Charger page at I2C 0x69. Supplies "max77865-charger", "ac" and "usb"; in charge
 * control (DT "samsung,charge-control", honoured only with
 * CONFIG_CHARGER_MAX77865_CHARGE_CONTROL) the stock init and the stock sec_battery
 * charge policy (thermal, swelling, full, recharge, safety timer); the USB OTG VBUS
 * boost as a regulator; the power-off-with-charger restart into LP charging.
 * Spec rewrite/spec/L6F_power_input_misc.md 5.3 (L6F below).
 * S: = stock 4.4 tree, CHG: = S:drivers/battery_v2/max77865_charger.c,
 * BAT: = S:drivers/battery_v2/sec_battery.c, FDT: = shipped DTB (battery node
 * FDT:12166-12327, cable-info FDT:12359-12430).
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

/* float voltage: CNFG_04 CV = (V[0.1 mV] - 40500) / 125, read reg * 125 + 40500 (CHG:201-228) */
#define CV_CODE_NORMAL		0x18	/* 43500: battery,chg_float_voltage 0xa9ec FDT:12281 */
#define CV_CODE_SWELLING	0x08	/* 41500: battery,swelling_drop_float_voltage 0xa21c FDT:12295 */
#define CV_CODE_TO_MV(c)	(((c) * 125 + 40500) / 10)	/* chg_float_voltage_conv 0x0a FDT:12282 */
#define FLOAT_MV		4350	/* 0xa9ec / 0x0a, FDT:12281-12282 */
#define SWELLING_DROP_MV	4150	/* 0xa21c / 0x0a, FDT:12295, :12282 */

/* battery temperature thresholds, 0.1 degC (the _normal ones = the _lpm ones, FDT:12223-12234) */
#define T_HIGHLIMIT		800	/* temp_highlimit_threshold_normal 0x320 FDT:12223 (usb-ntc) */
#define T_HIGHLIMIT_REC		750	/* temp_highlimit_recovery_normal 0x2ee FDT:12224 */
#define T_HIGH			500	/* temp_high_threshold_normal 0x1f4 FDT:12225 */
#define T_HIGH_REC		450	/* temp_high_recovery_normal 0x1c2 FDT:12226 */
#define T_LOW			0	/* temp_low_threshold_normal 0x00 FDT:12227 */
#define T_LOW_REC		50	/* temp_low_recovery_normal 0x32 FDT:12228 */
#define SW_HIGH_BLOCK		410	/* swelling_high_temp_block 0x19a FDT:12283 */
#define SW_HIGH_REC		390	/* swelling_high_temp_recov 0x186 FDT:12284 */
#define SW_LOW_BLOCK_1ST	150	/* swelling_low_temp_block_1st 0x96 FDT:12285 */
#define SW_LOW_REC_1ST		200	/* swelling_low_temp_recov_1st 0xc8 FDT:12286 */
#define SW_LOW_BLOCK_2ND	50	/* swelling_low_temp_block_2nd 0x32 FDT:12287 */
#define SW_LOW_REC_2ND		100	/* swelling_low_temp_recov_2nd 0x64 FDT:12288 */
#define MIX_HIGH_TEMP		420	/* mix_high_temp 0x1a4 FDT:12265 */
#define MIX_HIGH_CHG_TEMP	500	/* mix_high_chg_temp 0x1f4 FDT:12266 */
#define MIX_HIGH_TEMP_REC	390	/* mix_high_temp_recovery 0x186 FDT:12267 */
#define T_DEFAULT		250	/* 25.0 degC: missing usb/chg zone, ntc_required=N (L6F 5.3.3) */

/* currents, mA */
#define SW_LOW_CURRENT		750	/* swelling_low_temp_current 0x2ee FDT:12289 */
#define SW_LOW_TOPOFF		200	/* swelling_low_temp_topoff 0xc8 FDT:12290 */
#define SW_HIGH_CURRENT		1150	/* swelling_high_temp_current 0x47e FDT:12291 */
#define SW_HIGH_TOPOFF		200	/* swelling_high_temp_topoff 0xc8 FDT:12292 */
#define TOPOFF_1ST		300	/* full_check_current_1st 0x12c FDT:12362 */
#define TOPOFF_2ND		200	/* full_check_current_2nd 0xc8 FDT:12363 */
/* stock sec_bat_check_mix_temp() */
static inline int max77865_mix_input_ma(int in_mv)
{
	int v = in_mv / 1000;

	if (v <= 0)
		v = 5;
	return min((int)(TOPOFF_1ST + 50),
		   (int)(FLOAT_MV * (TOPOFF_1ST + 50) / (v * 90) / 10));
}

/* chg_high_temp limits (stock sec_bat_check_afc_temp()) */
#define CHG_HIGH_TEMP		490	/* battery,chg_high_temp 0x1ea */
#define CHG_HIGH_TEMP_REC	470	/* battery,chg_high_temp_recovery 0x1d6 */
#define CHG_HIGH_TEMP_HV_IN	1000	/* battery,chg_input_limit_current 0x3e8 */
#define CHG_HIGH_TEMP_HV_CC	1500	/* battery,chg_charging_limit_current 0x5dc */
#define CHG_HIGH_TEMP_DEFAULT_IN 1800	/* battery,default_input_current 0x708 */
#define CHG_HIGH_TEMP_DEFAULT_CC 2100	/* battery,default_charging_current 0x834 */
#define CHG_HIGH_TEMP_PWR_THR	14500	/* battery,pd_charging_charge_power 15000 - 500 mW */

/* full / recharge, mV and % */
#define FULL_COND_VCELL		4250	/* full_condition_vcell 0x109a FDT:12270 */
#define FULL_COND_SOC		93	/* full_condition_soc 0x5d FDT:12269 */
#define RECHG_VCELL		4280	/* recharge_condition_vcell 0x10b8 FDT:12274 */
#define SW_RECHG_MV		4000	/* swelling_high/low_rechg_voltage 0xfa0 FDT:12296-12297 */

#if IS_ENABLED(CONFIG_CHARGER_MAX77865_AGE_FORECAST)
struct max77865_age_data {
	int cycle;
	int float_mv_tenth;
	int recharge_vcell;
	int full_vcell;
	int full_soc;
};

/* stock battery,age_data rows (struct sec_age_data) */
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

/* safety timer */
#define EXPIRED_MS		11160000U	/* battery,expired_time 0x2b98 s FDT:12319, x1000 BAT:8010-8019 */
/* battery,recharging_expired_time 0x1518 s FDT:12320, x1000 BAT:8021-8029 */
#define RECHG_EXPIRED_MS	5400000U
#define STANDARD_CURR		2350	/* battery,standard_curr 0x92e FDT:12318 */

/* scheduling (L6F 5.3.3 "Scheduling") */
#define PASS_PERIOD		(10 * HZ)	/* polling_time[0] BASIC 0x0a FDT:12187 */
#define ALARM_PERIOD_S		30		/* polling_time[1] CHARGING 0x1e FDT:12187 */
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
	/* charging-extras cable classes */
	CABLE_AFC_PREPARE,
	CABLE_HV9,
	CABLE_PD,
	WLS_NV,
	WLS_PACK,
	WLS_HV,
	WLS_PREP_HV,
};

/* stock sec_battery_charging_mode, S:drivers/battery_v2/include/sec_charging_common.h:300-309 */
enum max77865_chg_stage { STAGE_NONE, STAGE_1ST, STAGE_2ND };

/* stock charger modes, S:drivers/battery_v2/include/sec_charging_common.h:294-296 */
enum max77865_mode { MODE_CHARGING, MODE_CHARGING_OFF, MODE_BUCK_OFF };

/* the single health H of L6F 5.3.3 (debugfs values of 5.3.5) */
enum max77865_health {
	H_GOOD, H_OVERHEAT, H_COLD, H_OVERHEATLIMIT, H_OVERVOLTAGE, H_UNDERVOLTAGE, H_SAFETY_TIMER,
	H_VSYS_OVP, H_VBAT_OVP,
};

/* stock swelling_mode, S:drivers/battery_v2/include/sec_battery.h:142-144 */
enum max77865_swelling { SWELLING_NONE, SWELLING_CHARGING, SWELLING_FULL };

/* stock current events, S:drivers/battery_v2/include/sec_battery.h:59-60,66-67 */
#define EV_LOW_TEMP_SWELLING	BIT(0)
#define EV_HIGH_TEMP_SWELLING	BIT(1)
#define EV_LOW_TEMP		BIT(2)
#define EV_SWELLING_MODE	(EV_LOW_TEMP_SWELLING | EV_LOW_TEMP | EV_HIGH_TEMP_SWELLING)

static bool ntc_required = true;
module_param(ntc_required, bool, 0644);
MODULE_PARM_DESC(ntc_required, "charge only with a valid battery-ntc reading (L6F 5.3.3)");

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

/* one pass's readings (L6F 5.3.3 step (2)) */
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
	/* charging-extras sample fields */
	struct {
		bool online;
		enum max77865_cable class;
		int vout_mv;
	} wls;
	int hv_mv;			/* AFC/QC voltage in mV, 0 = none */
	int pd_mv, pd_ma;		/* USB-PD contract voltage and current */
	bool pd_usb;			/* USB data partner present during PD */
	int rp;				/* Type-C Rp level */
	bool aicl_ok;			/* CHG_INT_OK bit 7 */
};

/* stock sec_bat_check_input_voltage() */
static int max77865_in_mv(const struct max77865_sample *s)
{
	switch (s->cable) {
	case CABLE_HV9:
		return 9000;
	case CABLE_PD:
		return s->pd_mv ? s->pd_mv : 5000;
	case WLS_HV:
	case WLS_PREP_HV:
		return 10000;
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
	bool lpcharge;
	bool stopping;
	bool poweroff_restart;

	/* policy state (L6F 5.3.3), under lock */
	enum max77865_input input;
	enum max77865_cable cable;
	int cable_tries;
	u32 mode;			/* enum max77865_chg_stage */
	u32 status;			/* POWER_SUPPLY_STATUS_* */
	u32 health;			/* H */
	u32 hc;				/* raw charger decode */
	u32 swelling;
	unsigned int events;
	bool recharging;
	bool mix_limit;
	u32 chg_mode;			/* enum max77865_mode: stock charger_mode */
	bool chg_block;			/* stock charging_block */
	u64 chg_start_s;		/* stock charging_start_time, 0 = not charging */
	u32 budget_ms;			/* stock expired_time */
	u64 prev_safety_s;		/* stock prev_safety_time */
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

	/* debugfs (L6F 5.3.5) */
	u32 override_usb_mc, override_chg_mc, override_t_mc, override_hc;

#if MAX77865_CCIC
	/* the CCIC's last PD sink status, under lock */
	struct s2mm005 *ccic;
	struct notifier_block ccic_nb;
	struct s2mm005_pd_status pd;
	unsigned long pd_event;		/* enum s2mm005_pd_event of the last notification */
#endif
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_TEST) && IS_ENABLED(CONFIG_CHARGER_MAX77865_HV_GUARD)
	u32 override_pd_mv;		/* test override: feeds only the HV guard */
#endif
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_TEST) && IS_ENABLED(CONFIG_CHARGER_MAX77865_VBAT_OVP)
	u32 override_vcell_mv;		/* test override: only raises the VBAT-OVP input */
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
	/* INT .. DTLS_03 (S:include/linux/mfd/max77865-private.h:63-69) */
	regmap_reg_range(MAX77865_CHG_REG_INT, 0xB6),
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

/* L6F 5.3.1: 8-bit registers, no cache, volatile 0xB0-0xB6 */
static const struct regmap_config max77865_chg_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = MAX77865_CHG_REG_SAFEOUT_CTRL,
	.rd_table = &max77865_chg_rd_table,
	.wr_table = &max77865_chg_wr_table,
	.volatile_table = &max77865_chg_volatile_table,
	.cache_type = REGCACHE_NONE,
};

/* ---- register decoders (read only) ---- */

/* DETAILS_01[3:0] (CHG:146-185) */
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
 * Charger health decode Hc (CHG:230-325): OVERVOLTAGE when the input DTLS reads 2;
 * UNDERVOLTAGE when it reads 0/1 with CHG_DTLS bit 3, BUCK and CHG on, wired; kept
 * UNDERVOLTAGE while the input reads 0/1 (the "battery health" of :317-321 is @h).
 * WCIN-only uses WCIN_DTLS (:109-121) and is never UNDERVOLTAGE.
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

/* ---- hardware writes ---- */

/* WDTCLR kick (CHG:239-241); not after L6F 5.3.2 step 13c, not with debugfs wdt_kick = N */
static void max77865_kick(struct max77865_charger *chg, bool wdten)
{
	if (chg->fault == 2 || !chg->wdt_kick || !wdten)
		return;
	regmap_update_bits(chg->regmap, MAX77865_CHG_REG_CNFG_06, MAX77865_CNFG_06_WDTCLR,
			   MAX77865_CNFG_06_WDTCLR);
}

/*
 * Stock sec_bat_set_charge() -> CHARGING_ENABLED: max77865_set_buck() then
 * max77865_set_charger_state() (CHG:1041-1055, :432-446, :569-588). WCIN-only
 * input: no CNFG_12 write (L6F 5.3.3 "WCIN only").
 */
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
					 MAX77865_CNFG_12_DISSKIP, on ? MAX77865_CNFG_12_DISSKIP : 0);
	chg->hw_mode = ret ? -1 : mode;
}

/* ---- policy helpers (stock sec_battery, BAT:) ---- */

/* stock sec_bat_set_charge() bookkeeping (BAT:1042-1110, :1068-1088); written by the apply step */
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
	chg->events = (chg->events & ~mask) | val;	/* BAT:327-341 */
}

static void max77865_reset_budget(struct max77865_charger *chg, u32 ms)
{
	chg->budget_ms = ms;
	chg->prev_safety_s = 0;
}

/* stock sec_bat_calculate_safety_time() (BAT:3436-3503) without the screen-on pause */
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
	if (chg->dis_cnt >= 5) {			/* :3451-3457 */
		max77865_reset_budget(chg, EXPIRED_MS);
		chg->dis_cnt = 0;
		return;
	}

	if (!chg->prev_safety_s)
		chg->prev_safety_s = now;
	if (in <= 0 || chg->chg_w <= 0)
		return;

	input_power = (s64)in * (max77865_in_mv(s) / 1000) * 1000;
	charging_power = (s64)chg->chg_w * FLOAT_MV;		/* :3442 */
	if (input_power > charging_power)
		curr = chg->chg_w;				/* :3469-3470 */
	else
		curr = div_s64(input_power, FLOAT_MV) * 9 / 10;	/* :3472-3473 */
	if (!curr)
		return;

	expired = div_u64(expired * STANDARD_CURR, curr);	/* :3488 */
	if (expired <= (now - chg->prev_safety_s) * 1000)
		expired = 0;
	else
		expired -= (now - chg->prev_safety_s) * 1000;
	chg->budget_ms = div_u64(expired * curr, STANDARD_CURR);	/* :3498 */
	chg->prev_safety_s = now;
}

/* new H of the thermal step (BAT:1942-2055), usb_thermal_source set (FDT:12185) */
static int max77865_thermal_h(int h, int t, int tu)
{
	if (tu >= T_HIGHLIMIT)
		return H_OVERHEATLIMIT;
	if (h == H_OVERHEATLIMIT && tu > T_HIGHLIMIT_REC)
		return h;
	if (t >= T_HIGH)
		return H_OVERHEAT;
	if (t <= T_HIGH_REC && t >= T_LOW_REC) {
		if (h == H_OVERHEAT || h == H_OVERHEATLIMIT || h == H_COLD)
			return (h == H_OVERHEATLIMIT && t > T_HIGH_REC) ? H_OVERHEAT : H_GOOD;
		return h;
	}
	if (t <= T_LOW)
		return H_COLD;
	return h;
}

#if IS_ENABLED(CONFIG_CHARGER_MAX77865_AGE_FORECAST)
/* stock sec_bat_aging_check() and sec_bat_set_aging_step() */
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
	 * swelling_normal_float_voltage := the row's float: every later
	 * "normal float" (detach, swelling end, thermal recovery) uses it; the CV itself only
	 * outside swelling, written by the apply step (never in fault mode)
	 */
	chg->cv_normal = cv_code;
	if (chg->swelling == SWELLING_NONE)
		chg->cv_want = cv_code;

	dev_info(chg->dev, "age step %d (cycle %d) float %d mV\n",
		 step, cycle, row->float_mv_tenth / 10);
}
#endif

/* the normal (non-swelling) float: the age step's with the age forecast, else battery,chg_float_voltage */
static int max77865_cv_normal(struct max77865_charger *chg)
{
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_AGE_FORECAST)
	return chg->cv_normal;
#else
	return CV_CODE_NORMAL;
#endif
}

/* stock sec_bat_do_fullcharged() (BAT:2540-2600) after a full count of 1 (FDT:12241) */
static void max77865_do_full(struct max77865_charger *chg, const struct max77865_sample *s)
{
	chg->status = POWER_SUPPLY_STATUS_FULL;
	if (chg->mode == STAGE_1ST) {
		/*
		 * second top-off cycle: CHARGING_OFF, currents (top-off 200 mA), CHARGING
		 * (:2556-2567). The off write is immediate; apply re-enables after the currents.
		 */
		chg->mode = STAGE_2ND;
		if (chg->hw_mode == MODE_CHARGING)
			max77865_write_mode(chg, MODE_CHARGING_OFF);
		max77865_set_mode(chg, MODE_CHARGING);
		dev_info(chg->dev, "full (1st), vcell %d mV\n", s->vcell);
	} else {
		chg->mode = STAGE_NONE;			/* :2571-2594 */
		chg->recharging = false;
		max77865_set_mode(chg, MODE_CHARGING_OFF);
		dev_info(chg->dev, "full (2nd), vcell %d mV\n", s->vcell);
	}
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

/* (3) cable attach / detach (stock sec_bat_cable_work(), BAT:3887-4036) */
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
	max77865_reset_budget(chg, EXPIRED_MS);		/* BAT:1262-1263 */
	if (attach) {
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_SYSOVLO)
		chg->sysovlo = false;		/* stays armed while charging, as stock */
#endif
		/* rewrite everything for the new input, as stock's ONLINE path did (CHG:1057-1061) */
		chg->hw_mode = -1;
		chg->in_w = -1;
		chg->chg_w = -1;
		chg->topoff_w = -1;
		chg->mode = STAGE_1ST;
		chg->status = POWER_SUPPLY_STATUS_CHARGING;
		max77865_set_mode(chg, MODE_CHARGING);	/* :4032-4036 */
	} else {
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_SYSOVLO)
		/* cleared on detach; unmasked on every removal */
		chg->sysovlo = false;
		if (chg->sysovlo_irq > 0 && !chg->sysovlo_armed) {
			enable_irq(chg->sysovlo_irq);
			chg->sysovlo_armed = true;
		}
#endif
		chg->mode = STAGE_NONE;			/* :3928 */
		chg->status = POWER_SUPPLY_STATUS_DISCHARGING;
		chg->swelling = SWELLING_NONE;		/* :3887-3896 */
		chg->cv_want = max77865_cv_normal(chg);	/* swelling_normal_float_voltage */
		max77865_set_events(chg, 0, EV_SWELLING_MODE);
		chg->mix_limit = false;			/* :3946 */
		max77865_set_mode(chg, MODE_CHARGING_OFF);	/* :3974 */
	}
}

/* ---- the policy (L6F 5.3.3), charge-control and charge-off fault mode ---- */
static void max77865_policy(struct max77865_charger *chg, struct max77865_sample *s)
{
	int pre, h, dtls_status;
	unsigned int v;
	bool en_swelling = false, en_rechg = false;

	/* (3) attach / detach; CHGINSEL on every wired attach (CHG:448-463, :1061) */
	if (chg->input == INPUT_NONE && s->input != INPUT_NONE) {
		max77865_cable_event(chg, true);
		chg->cable_tries = 0;
	} else if (chg->input != INPUT_NONE && s->input == INPUT_NONE) {
		max77865_cable_event(chg, false);
	}
	if (s->input == INPUT_WIRED && chg->input != INPUT_WIRED)
		chg->chginsel_pending = true;
	chg->input = s->input;

	/* (3a) battery over-voltage stop */
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_VBAT_OVP)
	if ((chg->status == POWER_SUPPLY_STATUS_CHARGING ||
	     chg->status == POWER_SUPPLY_STATUS_FULL) && !chg->vbat_ovp_done) {
		int vcell = s->vcell;

#if IS_ENABLED(CONFIG_CHARGER_MAX77865_TEST)
		vcell = max_t(int, vcell, chg->override_vcell_mv);	/* only raises */
#endif
		/* cisd_max_voltage_thr 0x1130 = 4400 mV; a failed read (-1) is not above */
		if (vcell > 4400)
			chg->vbat_cnt++;
		else
			chg->vbat_cnt = 0;

		if (chg->vbat_cnt >= 2) {		/* ab_vbat_max_count 2 */
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

	/* (4) OVP/UVLO on Hc (BAT:1417-1467, _result :1372-1415), not while discharging (:1560-1564) */
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

	/* (5) voltage check (BAT:1574-1616) */
	if (chg->status != POWER_SUPPLY_STATUS_DISCHARGING) {
		/* (5a) FULL -> CHARGING (:1574-1595) */
		if (chg->status == POWER_SUPPLY_STATUS_FULL &&
		    (chg->mode == STAGE_2ND || chg->recharging || chg->swelling != SWELLING_NONE) &&
		    s->cap >= 0 && s->vcell >= 0 && s->cap < MAX77865_FULL_SOC(chg) &&
		    s->vcell < MAX77865_RECHG_VCELL(chg) - 50) {
			chg->status = POWER_SUPPLY_STATUS_CHARGING;
			return;
		}
		/* (5b) recharge (:1469-1554), not in swelling charging/full (:1471-1477) */
		if (chg->swelling == SWELLING_NONE && chg->mode == STAGE_NONE) {
			bool rechg = false;

			if (chg->status == POWER_SUPPLY_STATUS_CHARGING) {
				rechg = true;			/* NOTIMEFULL, :1479-1487 */
			} else if (chg->status == POWER_SUPPLY_STATUS_FULL && s->vcell >= 0) {
				/* float - 150 mV with the low-temp event (:1491-1498) */
				int vr = (chg->events & EV_LOW_TEMP) ? FLOAT_MV - 150 : MAX77865_RECHG_VCELL(chg);

				if (s->vcell <= vr) {		/* :1526-1535 */
					max77865_reset_budget(chg, RECHG_EXPIRED_MS);
					rechg = true;
					dev_info(chg->dev, "recharge, vcell %d mV <= %d mV\n",
						 s->vcell, vr);
				}
			}
			if (rechg) {				/* :1597-1616 */
				chg->mode = STAGE_1ST;
				chg->recharging = true;
				max77865_set_mode(chg, MODE_CHARGING);
				return;
			}
		}
	}

	/* (6) safety timer expiry (BAT:2297-2383): only while charging (charging_start_time, :2303) */
	if (chg->chg_start_s &&
	    (chg->status == POWER_SUPPLY_STATUS_CHARGING || chg->status == POWER_SUPPLY_STATUS_FULL) &&
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

	/* (7)-(9) need a temperature (L6F 5.3.3 "read ERROR") */
	if (!s->ntc_valid)
		return;

	/* (7) thermal (BAT:1898-2178) */
	if (chg->status != POWER_SUPPLY_STATUS_DISCHARGING) {
		if (chg->health != H_GOOD && chg->health != H_OVERHEAT &&
		    chg->health != H_COLD && chg->health != H_OVERHEATLIMIT)
			return;					/* :1911-1917 */
		pre = chg->health;
		h = max77865_thermal_h(pre, s->t, s->tu);
		chg->health = h;
		if (h == H_OVERHEAT || h == H_COLD || h == H_OVERHEATLIMIT) {
			if (h != pre) {				/* :2066-2104 */
				chg->status = POWER_SUPPLY_STATUS_NOT_CHARGING;
				max77865_set_mode(chg, h == H_COLD ? MODE_CHARGING_OFF : MODE_BUCK_OFF);
				max77865_log_thermal(chg, pre, s);
				return;
			}
			/* :2105-2115; an unknown VCELL counts as stock's 0 */
			if (h != H_COLD && chg->chg_mode == MODE_BUCK_OFF &&
			    s->vcell < SWELLING_DROP_MV)
				max77865_set_mode(chg, MODE_CHARGING_OFF);
		} else if (chg->status == POWER_SUPPLY_STATUS_NOT_CHARGING) {
			/* recovery (:2117-2174) */
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
				if (chg->status == POWER_SUPPLY_STATUS_FULL && chg->mode == STAGE_NONE)
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

	/* (8) swelling (BAT:1622-1758) */
	if (chg->status == POWER_SUPPLY_STATUS_DISCHARGING ||
	    chg->status == POWER_SUPPLY_STATUS_NOT_CHARGING) {
		chg->swelling = SWELLING_NONE;			/* :1644-1651 */
		max77865_set_events(chg, 0, EV_SWELLING_MODE);
	} else {
		if (chg->swelling == SWELLING_NONE) {
			if (s->t >= SW_HIGH_BLOCK || s->t <= SW_LOW_BLOCK_2ND) {	/* :1653-1677 */
				chg->swelling = SWELLING_CHARGING;
				max77865_set_mode(chg, MODE_BUCK_OFF);
				max77865_set_events(chg, s->t >= SW_HIGH_BLOCK ?
						    EV_HIGH_TEMP_SWELLING : EV_LOW_TEMP_SWELLING,
						    EV_SWELLING_MODE);
				en_swelling = true;
			} else if (s->t <= SW_LOW_BLOCK_1ST && !(chg->events & EV_LOW_TEMP)) {
				max77865_set_events(chg, EV_LOW_TEMP, EV_LOW_TEMP);	/* :1678-1682 */
			} else if (s->t >= SW_LOW_REC_1ST && (chg->events & EV_LOW_TEMP)) {
				max77865_set_events(chg, 0, EV_LOW_TEMP);		/* :1683-1687 */
			}
		}

		if (s->vcell < 0)				/* :1690-1691 */
			goto full_check;

		if (chg->swelling != SWELLING_NONE) {
			if (s->t <= SW_HIGH_REC && s->t >= SW_LOW_REC_2ND) {	/* :1698-1715 */
				bool low = s->t <= SW_LOW_BLOCK_1ST ||
					   (s->t < SW_LOW_REC_1ST &&
					    (chg->events & EV_LOW_TEMP_SWELLING));

				chg->swelling = SWELLING_NONE;
				chg->mode = STAGE_1ST;
				max77865_set_events(chg, low ? EV_LOW_TEMP : 0, EV_SWELLING_MODE);
				max77865_set_mode(chg, MODE_CHARGING);
				chg->cv_want = max77865_cv_normal(chg);
				max77865_log_swelling(chg, "end", s);
			} else if (s->vcell < SW_RECHG_MV && chg->chg_block) {	/* :1720-1742 */
				chg->mode = STAGE_1ST;
				en_rechg = true;
				chg->cv_want = CV_CODE_SWELLING;
				max77865_set_mode(chg, MODE_CHARGING);
				if (s->t < SW_LOW_REC_2ND)
					max77865_set_events(chg, EV_LOW_TEMP_SWELLING, EV_SWELLING_MODE);
				else if (s->t > SW_HIGH_REC)
					max77865_set_events(chg, EV_HIGH_TEMP_SWELLING, EV_SWELLING_MODE);
				max77865_log_swelling(chg, "recharge", s);
			}
		}

		if (en_swelling && !en_rechg) {			/* :1750-1754 */
			max77865_reset_budget(chg, EXPIRED_MS);
			max77865_log_swelling(chg, "start", s);
		}
	}

full_check:
	/* (9) swelling full or normal full check (BAT:3611-3615), charger STATUS read now */
	if (regmap_read(chg->regmap, MAX77865_CHG_REG_DETAILS_01, &v))
		return;
	dtls_status = max77865_dtls_status(v);

	if ((chg->swelling == SWELLING_CHARGING || chg->swelling == SWELLING_FULL) &&
	    !chg->chg_block) {
		if (dtls_status == POWER_SUPPLY_STATUS_FULL) {	/* BAT:3047-3075 */
			chg->mode = STAGE_NONE;
			chg->recharging = false;
			chg->swelling = SWELLING_FULL;
			max77865_set_mode(chg, MODE_CHARGING_OFF);
			max77865_reset_budget(chg, EXPIRED_MS);
			max77865_log_swelling(chg, "full", s);
		}
	} else if (chg->mode != STAGE_NONE && chg->status != POWER_SUPPLY_STATUS_NOT_CHARGING) {
		/* :2621-2640; condition VCELL (full_condition_type 9, FDT:12268; :2220-2229) */
		if (s->vcell >= MAX77865_FULL_VCELL(chg) && dtls_status == POWER_SUPPLY_STATUS_FULL)
			max77865_do_full(chg, s);
	}
}

/* (11) currents of stock sec_bat_set_charging_current() (BAT:883-1040), wired branch */
static void max77865_currents(struct max77865_charger *chg, const struct max77865_sample *s,
			      int *in, int *cc, int *topoff)
{
	switch (s->cable) {
	case CABLE_DCP:		/* cable 3 TA, default group FDT:12360-12361 */
		*in = 1800;
		*cc = 2100;
		break;
	case CABLE_CDP:		/* cable 5 USB_CDP, current_group_3 FDT:12379-12380 */
		*in = 1500;
		*cc = 1500;
		break;
	case CABLE_AFC_PREPARE:	/* cable 2 PREPARE_TA */
		*in = 1000;
		*cc = 1000;
		break;
	case CABLE_HV9:		/* cable 6 9V_TA, current_group_4 */
		*in = 1650;
		*cc = 2350;
		break;
	case CABLE_PD:		/* cable 19 PDIC: the contract's PDO */
		*in = s->pd_ma;
		*cc = min(s->pd_mv * s->pd_ma / FLOAT_MV, 2350);
		break;
	case WLS_NV:		/* cable 10,12,15,27 */
		*in = 900;
		*cc = 1300;
		break;
	case WLS_PACK:		/* cable 13 WIRELESS_PACK */
		*in = 700;
		*cc = 1400;
		break;
	case WLS_HV:		/* cable 11,14,16,28, current_group_10 */
		*in = 650;
		*cc = 1300;
		break;
	case WLS_PREP_HV:	/* cable 29 PREPARE_WIRELESS_HV, current_group_11 */
		*in = 500;
		*cc = 1300;
		break;
	default:		/* cable 4 USB (and unknown), current_group_1 FDT:12367-12368 */
		*in = 500;
		*cc = 500;
		break;
	}

	/*
	 * mix-temperature input limit (BAT:565-596), wired only (chg_temp_check 1
	 * FDT:12180; off otherwise, :592-594); needs T (kept while there is none)
	 */
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
	 * chg_high_temp: stock sec_bat_check_afc_temp(), CONFIG_MUIC_HV branch with siop 100,
	 * for wired cables other than PDIC (PD uses its own
	 * PDO step). Skipped while the mix limit holds, as stock's SKIP_HEATING_CONTROL event
	 * keeps the mix value. Stock leaves the limit flag as it is when no
	 * branch matches. While limited, both cases keep chg_input/charging_limit_current
	 * (for the >= 14.5 W case only the first step uses default 1800/2100).
	 */
	if (s->input == INPUT_WIRED && s->cable != CABLE_PD && !chg->mix_limit) {
		bool hv = s->cable == CABLE_HV9;			/* is_hv_wire_type() */
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

	/* top-off (:889, :938-943; full_check_type_2nd 7 = CHGPSY FDT:12240) */
	*topoff = chg->mode == STAGE_2ND ? TOPOFF_2ND : TOPOFF_1ST;

	/* swelling caps, wired branch (:961-975) */
	if (chg->events & EV_LOW_TEMP_SWELLING) {
		*cc = min(*cc, SW_LOW_CURRENT);
		*topoff = min(*topoff, SW_LOW_TOPOFF);
	} else if (chg->events & EV_HIGH_TEMP_SWELLING) {
		*cc = min(*cc, SW_HIGH_CURRENT);
		*topoff = min(*topoff, SW_HIGH_TOPOFF);
	} else if (chg->events & EV_LOW_TEMP) {
		*cc = min(*cc, SW_LOW_CURRENT);
	}

	/* max_charging_current 2350 is applied as the last CC cap for every class */
	*cc = min(*cc, 2350);
}

/* (11) apply: only changed fields, stock read-modify-write setters (L6F 5.3.3 "apply") */
static void max77865_apply(struct max77865_charger *chg, const struct max77865_sample *s)
{
	int cv = chg->cv_want;
	int in = 0, cc = 0, topoff = 0, eff;

	if (chg->fault == 2)
		return;					/* L6F 5.3.2 step 13c: write nothing */

	if (s->input != INPUT_NONE && !chg->otg_on) {
		max77865_currents(chg, s, &in, &cc, &topoff);
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_AICL)
		/* the AICL result is a cap the pass can only lower */
		if (chg->aicl_ma)
			in = min(in, (int)chg->aicl_ma);
#endif
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_HV_GUARD) && !IS_ENABLED(CONFIG_CHARGER_MAX77865_PD)
		/*
		 * HV guard, the last cap: a PD contract above 5 V that the S2MM005
		 * firmware chose by itself caps the input at current_group_1's 500 mA
		 * unless CONFIG_CHARGER_MAX77865_PD selects the PDO; lowers only.
		 */
		if (s->input == INPUT_WIRED) {
			int pd_mv = s->pd_mv;

#if IS_ENABLED(CONFIG_CHARGER_MAX77865_TEST)
			pd_mv = max_t(int, pd_mv, chg->override_pd_mv);
#endif
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

	if (!chg->fault) {
		if (chg->chginsel_pending && !chg->otg_on &&	/* CHG:448-463 */
		    !regmap_update_bits(chg->regmap, MAX77865_CHG_REG_CNFG_12,
					MAX77865_CNFG_12_CHGINSEL, MAX77865_CNFG_12_CHGINSEL))
			chg->chginsel_pending = false;
		if (s->input == INPUT_WIRED && !chg->otg_on && in != chg->in_w &&
		    !regmap_update_bits(chg->regmap, MAX77865_CHG_REG_CNFG_09,	/* CHG:465-501 */
					MAX77865_CNFG_09_CHGIN_LIM, min(in, 3175) / 25))
			chg->in_w = in;
		if (s->input != INPUT_NONE && !chg->otg_on && cc != chg->chg_w &&
		    !regmap_update_bits(chg->regmap, MAX77865_CHG_REG_CNFG_02,	/* CHG:503-525 */
					MAX77865_CNFG_02_CHG_CC, min(cc, 3150) / 50))
			chg->chg_w = cc;
		if (s->input != INPUT_NONE && !chg->otg_on && topoff != chg->topoff_w &&
		    !regmap_update_bits(chg->regmap, MAX77865_CHG_REG_CNFG_03,	/* CHG:550-567 */
					MAX77865_CNFG_03_TO_ITH,
					(clamp(topoff, 150, 500) - 150) / 50))
			chg->topoff_w = topoff;
		if (cv != chg->cv_w &&
		    !regmap_update_bits(chg->regmap, MAX77865_CHG_REG_CNFG_04,	/* CHG:201-215 */
					MAX77865_CNFG_04_CHG_CV_PRM, cv))
			chg->cv_w = cv;
	}

	if (chg->otg_on)
		return;					/* override (c): input treated as detached */

	/* overrides (a) fault mode, (b) no valid NTC reading (L6F 5.3.3 "overrides") */
	eff = chg->chg_mode;
	if (eff == MODE_CHARGING && (chg->fault || !chg->ntc_seen || !s->ntc_valid))
		eff = MODE_CHARGING_OFF;
	if (eff != chg->hw_mode)
		max77865_write_mode(chg, eff);
}

/* ---- readings ---- */

static int max77865_zone_avg(const char *name, int *mc)
{
	struct thermal_zone_device *tz = thermal_zone_get_zone_by_name(name);

	if (IS_ERR(tz))
		return PTR_ERR(tz);
	return max77865_ntc_avg(tz, mc);
}

/* battery, USB-connector and charger thermistors (L6F 4.4, 5.3.3) */
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
		/* table end points clamp to -200 / 900 (FDT:12188-12189): not valid for the gate */
		if (s->t > -200 && s->t < 900)
			chg->ntc_seen = true;
	} else if (!ntc_required) {
		s->t = T_DEFAULT;
		s->t_mc = T_DEFAULT * 100;
		s->ntc_valid = true;
		s->ntc_escape = true;
		chg->ntc_seen = true;
	}

	/* usb_thermal_source 2 / chg_thermal_source 1 (FDT:12183-12185), BAT:2695-2707 */
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

/* FG VCELL / IAVG / CAPACITY (L6F 5.3.3 "VCELL"); -1 = unknown */
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
 * BC1.2 result from the MUIC's STATUS1_BC by an adapter-level transfer, no client
 * at 0x25 (L6F 5.3.1, REQ-L3-1). ChgTyp 0 or ChgDetRun = detection not finished.
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
/*
 * (2) the PD contract the CCIC reported: the PDO the firmware or the last
 * select_pdo() made current (stock power_list[current_pdo_num])
 */
static void max77865_read_pd(struct max77865_charger *chg, struct max77865_sample *s)
{
	unsigned int pos = chg->pd.current_pdo_num;

	if ((chg->pd_event != S2MM005_PD_EVENT_PD_SINK &&
	     chg->pd_event != S2MM005_PD_EVENT_PD_SINK_CAP) ||
	    pos < 1 || pos > S2MM005_MAX_PDO_NUM)
		return;
	s->pd_mv = chg->pd.power_list[pos].max_voltage;
	s->pd_ma = chg->pd.power_list[pos].max_current;
}
#endif

/* ---- the pass (L6F 5.3.3): one delayed work for every trigger ---- */

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

	/* (1) watchdog kick: charge control always, monitor-only while WDTEN (L6F 5.3.2) */
	if (chg->charge_control)
		max77865_kick(chg, true);
	else if (!regmap_read(chg->regmap, MAX77865_CHG_REG_CNFG_00, &wd))
		max77865_kick(chg, wd & MAX77865_CNFG_00_WDTEN);

	/* (2) readings */
	if (regmap_read(chg->regmap, MAX77865_CHG_REG_INT_OK, &s.int_ok) ||
	    regmap_read(chg->regmap, MAX77865_CHG_REG_DETAILS_00, &s.dtls0) ||
	    regmap_read(chg->regmap, MAX77865_CHG_REG_DETAILS_01, &s.dtls1) ||
	    regmap_read(chg->regmap, MAX77865_CHG_REG_CNFG_00, &s.cnfg00)) {
		dev_err_ratelimited(chg->dev, "status read failed, pass skipped\n");
		plugged = chg->input != INPUT_NONE;
		goto rearm;
	}
	if (chg->otg_on)
		s.input = INPUT_NONE;			/* L6F 5.3.8 policy interplay */
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

	/* stock workaround while charging (CHG:287-292): CHG off, CV again, CHG on */
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
		/* (3)-(9) */
		max77865_policy(chg, &s);
		/* (10) safety-time accounting while CHG is on (BAT:3637-3641); not while an override holds it off */
		if (!chg->chg_block && chg->hw_mode == MODE_CHARGING)
			max77865_safety_time(chg, &s);
		/* (11) */
		max77865_apply(chg, &s);
	} else {
		if (chg->input == INPUT_NONE && s.input != INPUT_NONE)
			chg->cable_tries = 0;
		if (chg->input != s.input)
			chg->max_power = 0;		/* attach/detach */
		chg->input = s.input;
		if (s.input != INPUT_NONE)
			max77865_currents(chg, &s, &chg->in_ma, &chg->chg_ma, &topoff);
	}
	read_ok = true;
	chg->cable = s.input == INPUT_WIRED ? s.cable : CABLE_NONE;
	plugged = s.input != INPUT_NONE;

	/* reported values (L6F 5.3.1, 5.3.3 "reported") */
	online = !!(s.int_ok & (MAX77865_CHGIN_OK | MAX77865_WCIN_OK)) && !chg->otg_on;
	if (chg->charge_control) {
		rep_status = chg->status;
		if (chg->fault && plugged)
			rep_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		rep_health = chg->health;
	} else {
		rep_status = online ? max77865_dtls_status(s.dtls1) : POWER_SUPPLY_STATUS_DISCHARGING;
		rep_health = chg->hc;
	}

	/*
	 * stock charge_power / max_charge_power = input voltage x input current,
	 * reset on attach/detach; the TTF source and chg_high_temp read it
	 */
	if (s.input != INPUT_NONE) {
		u32 power = (u32)max77865_in_mv(&s) * chg->in_ma / 1000;

		if (power > chg->max_power)
			chg->max_power = power;
	}

#if IS_ENABLED(CONFIG_CHARGER_MAX77865_TTF)
	/* stock sec_bat_calc_time_to_full() charge current, 0 = none */
	if (rep_status == POWER_SUPPLY_STATUS_CHARGING ||
	    (rep_status == POWER_SUPPLY_STATUS_FULL && s.cap >= 0 && s.cap != 100)) {
		int charge;

		if (s.cable == CABLE_HV9 || chg->max_power >= 2350 * 5)	/* max_charging_current */
			charge = 2350;		/* ttf_hv_charge_current 0x92e */
		else if (s.cable == WLS_HV || s.cable == WLS_PREP_HV)	/* OTG: normal mode */
			charge = chg->otg_on ? 950 : 1200;	/* ttf_hv_wireless 0x4b0 */
		else if (s.input == INPUT_WCIN)
			charge = 950;		/* ttf_wireless_charge_current 0x3b6 */
		else
			charge = chg->max_power / 5;
		chg->ttf_ma = charge;
	} else {
		chg->ttf_ma = 0;
	}
#endif

	ac = s.input == INPUT_WIRED && (chg->cable == CABLE_DCP || chg->cable == CABLE_UNKNOWN ||
					chg->cable == CABLE_AFC_PREPARE || chg->cable == CABLE_HV9 ||
					(chg->cable == CABLE_PD && !s.pd_usb));
	usb = s.input == INPUT_WIRED && (chg->cable == CABLE_SDP || chg->cable == CABLE_CDP ||
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

	/* 1-min log while plugged (L6F 5.3.5, 8.3) */
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
		dev_info(chg->dev, "ntc %d mC cable %s in %d mA chg %d mA %s\n", s.t_mc,
			 s.input == INPUT_WCIN ? "wcin" : cable_name[chg->cable],
			 chg->in_ma, chg->chg_ma, what);
		if (s.ntc_escape)
			dev_warn(chg->dev,
				 "ntc invalid, ntc_required=N: charging without battery temperature\n");
		chg->last_log = jiffies;
		chg->log_armed = true;
	}

rearm:
	/* scheduling (L6F 5.3.3): 30 s RTC alarm while plugged, else stay awake while plugged */
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
 * Stock max77865_chgin_isr_work() (CHG:1580-1669): hold a wakeup, read CHGIN_DTLS every
 * 100 ms until it read the same more than 10 times in a row, unmask CHGIN_I, then one
 * pass at once (it relaxes the wakeup). A failed read counts as 0, the value stock's
 * re-mask of its previous (already masked) value gives.
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
		if (stable > 10)			/* :1612-1616 */
			break;
		prev = dtls;
		msleep(100);				/* :1664 */
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
/* stock aicl_isr_work() */
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
			/*
			 * reduce_input_current(REDUCE_CURRENT_STEP 100): stop at
			 * MINIMUM_INPUT_CURRENT 300 (code 12) or the step (code 4), floor 12
			 */
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
	/* unmask AICL_IM */
	regmap_update_bits(chg->regmap, MAX77865_CHG_REG_INT_MASK,
			   MAX77865_CHG_AICL, 0);
	mutex_unlock(&chg->lock);
}
#endif

/* ---- OTG VBUS boost (L6F 5.3.8), stock max77865_set_otg() CHG:623-697 ---- */

/* under lock */
static void max77865_otg_off(struct max77865_charger *chg)
{
	regmap_update_bits(chg->regmap, MAX77865_CHG_REG_CNFG_00,	/* :669-671 */
			   MAX77865_CNFG_00_OTG_CTRL, 0);
	regmap_write(chg->regmap, MAX77865_CHG_REG_CNFG_11, 0x00);	/* :673-675 3.485 V */
	msleep(50);							/* :676 mdelay(50) */
	regmap_write(chg->regmap, MAX77865_CHG_REG_INT_MASK, chg->otg_saved_mask);	/* :678-680 */
	/*
	 * Without charge control nothing else sets CHG and DISSKIP again: give the
	 * session's charge-off back as found, or S-Boot's charging stays off until a reboot.
	 */
	if (!chg->charge_control) {
		regmap_update_bits(chg->regmap, MAX77865_CHG_REG_CNFG_00, MAX77865_CNFG_00_CHG,
				   chg->otg_saved_cnfg00);
		regmap_update_bits(chg->regmap, MAX77865_CHG_REG_CNFG_12, MAX77865_CNFG_12_DISSKIP,
				   chg->otg_saved_cnfg12);
	}
	chg->otg_on = false;
}

static int max77865_otg_enable(struct regulator_dev *rdev)
{
	struct max77865_charger *chg = rdev_get_drvdata(rdev);
	int ret = 0;

	mutex_lock(&chg->lock);
	if (chg->otg_on || chg->lpcharge || chg->stopping)	/* :631 */
		goto out;
	if (chg->fault == 2) {			/* L6F 5.3.2 step 13c: write nothing */
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
	 * CHG off first (stock max77865_set_charger_state(0), CHG:569-588): the pass skips
	 * every mode write while boosting, so a CHARGING left from before would stay set.
	 * Samsung's sec_battery charges off the same way for an OTG cable
	 * (SEC_BAT_CHG_MODE_CHARGING_OFF in sec_bat_cable_work()).
	 */
	regmap_update_bits(chg->regmap, MAX77865_CHG_REG_CNFG_00, MAX77865_CNFG_00_CHG, 0);
	regmap_update_bits(chg->regmap, MAX77865_CHG_REG_CNFG_12, MAX77865_CNFG_12_DISSKIP, 0);
	chg->hw_mode = -1;
	/* mask CHG and CHGIN, unmask BYP (:647-654) */
	regmap_update_bits(chg->regmap, MAX77865_CHG_REG_INT_MASK,
			   MAX77865_CHG_CHG | MAX77865_CHG_CHGIN | MAX77865_CHG_BYP,
			   MAX77865_CHG_CHG | MAX77865_CHG_CHGIN);
	regmap_write(chg->regmap, MAX77865_CHG_REG_CNFG_11, 0x16);	/* :656-658 5.020 V */
	regmap_update_bits(chg->regmap, MAX77865_CHG_REG_CNFG_00,	/* :659-661 OTG off, boost on */
			   MAX77865_CNFG_00_OTG_CTRL, MAX77865_CNFG_00_BOOST);
	msleep(100);							/* :663 */
	regmap_update_bits(chg->regmap, MAX77865_CHG_REG_CNFG_00,	/* :665-667 OTG on, boost on */
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
	.fixed_uV = 5020000,		/* CNFG_11 0x16 "5.020V", CHG:656-658 */
};

/* ---- IRQ (L6F 5.3.1) ---- */

#if IS_ENABLED(CONFIG_CHARGER_MAX77865_SYSOVLO)
static irqreturn_t max77865_sysovlo_irq(int irq, void *data)
{
	struct max77865_charger *chg = data;

	disable_irq_nosync(irq);			/* stock masks in the handler */
	pm_wakeup_event(chg->dev, 5000);		/* stock 5 s wake lock */
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

	/* CHG_INT is clear-on-read (S:drivers/mfd/max77865-irq.c:172-176) */
	if (regmap_read(chg->regmap, MAX77865_CHG_REG_INT, &v))
		return IRQ_NONE;

	if (v & MAX77865_CHG_CHGIN) {
		/* storm guard: mask CHGIN_I until the work has debounced (max77865-irq.c:178-189) */
		regmap_update_bits(chg->regmap, MAX77865_CHG_REG_INT_MASK, MAX77865_CHG_CHGIN,
				   MAX77865_CHG_CHGIN);
		pm_stay_awake(chg->dev);
		queue_work(system_freezable_wq, &chg->chgin_work);
	}

	if (v & MAX77865_CHG_BYP) {
		/* bypass over-current while boosting (CHG:1462-1506, BYP_DTLS bit 0 :1488) */
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
			/* mask AICL_IM and queue aicl_dwork after 50 ms */
			regmap_update_bits(chg->regmap, MAX77865_CHG_REG_INT_MASK,
					   MAX77865_CHG_AICL, MAX77865_CHG_AICL);
			queue_delayed_work(system_freezable_wq, &chg->aicl_dwork,
					   msecs_to_jiffies(50));
		}
	}
#endif

	return IRQ_HANDLED;
}

/* ---- power supplies (L6F 5.3.1) ---- */

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
	[H_OVERHEATLIMIT] = POWER_SUPPLY_HEALTH_OVERHEAT,	/* 6.18 has no OVERHEATLIMIT */
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
	case POWER_SUPPLY_PROP_ONLINE:		/* INT_OK CHGIN_OK | WCIN_OK (charger.h:80-83) */
		ret = regmap_read(chg->regmap, MAX77865_CHG_REG_INT_OK, &v);
		if (ret)
			return ret;
		val->intval = !!(v & (MAX77865_CHGIN_OK | MAX77865_WCIN_OK)) && !READ_ONCE(chg->otg_on);
		return 0;
	case POWER_SUPPLY_PROP_STATUS:
		mutex_lock(&chg->lock);
		val->intval = chg->rep_status;
		mutex_unlock(&chg->lock);
		return 0;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:	/* CHG:919-929 */
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
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:	/* CHG:360-370 */
		ret = regmap_read(chg->regmap, MAX77865_CHG_REG_CNFG_09, &v);
		if (ret)
			return ret;
		v &= MAX77865_CNFG_09_CHGIN_LIM;
		val->intval = (v <= 4 ? 100 : v >= 0x7F ? 3175 : v * 25) * 1000;
		return 0;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:	/* CHG:327-339 */
		ret = regmap_read(chg->regmap, MAX77865_CHG_REG_CNFG_02, &v);
		if (ret)
			return ret;
		v &= MAX77865_CNFG_02_CHG_CC;
		val->intval = (v <= 2 ? 100 : v * 50) * 1000;
		return 0;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:	/* CHG:217-228, 0.1 mV */
		ret = regmap_read(chg->regmap, MAX77865_CHG_REG_CNFG_04, &v);
		if (ret)
			return ret;
		val->intval = ((v & MAX77865_CNFG_04_CHG_CV_PRM) * 125 + 40500) * 100;
		return 0;
	case POWER_SUPPLY_PROP_PRESENT:		/* CHG:410-430 */
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
	default:
		mutex_unlock(&chg->lock);
		return -EINVAL;
	}
	mutex_unlock(&chg->lock);
	return 0;
}

/* stock supply name and type (CHG:1837-1845) */
static const struct power_supply_desc max77865_chg_desc = {
	.name = "max77865-charger",
	.type = POWER_SUPPLY_TYPE_UNKNOWN,
	.properties = max77865_chg_props,
	.num_properties = ARRAY_SIZE(max77865_chg_props),
	.get_property = max77865_chg_get_property,
};

/* stock sec_battery supply names (S:drivers/staging/samsung/sec_reboot.c:107-110) */
static const struct power_supply_desc max77865_ac_desc = {
	.name = "ac",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = max77865_ac_props,
	.num_properties = ARRAY_SIZE(max77865_ac_props),
	.get_property = max77865_in_get_property,
};

static const struct power_supply_desc max77865_usb_desc = {
	.name = "usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = max77865_usb_props,
	.num_properties = ARRAY_SIZE(max77865_usb_props),
	.get_property = max77865_in_get_property,
};

/* ---- charge-control init (L6F 5.3.2) = stock max77865_charger_initialize() CHG:718-794 ---- */

struct max77865_readback {
	u8 reg, mask, val;
};

/* L6F 5.3.2 step 13: the values of steps 2-11 */
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

	/* 1. CHGPROT unlock, 10 tries 20 ms apart (CHG:57-82) */
	for (i = 0; i < 10; i++) {
		if (!regmap_read(map, MAX77865_CHG_REG_CNFG_06, &v) &&
		    FIELD_GET(MAX77865_CNFG_06_CHGPROT, v) == 0x03)
			break;
		regmap_update_bits(map, MAX77865_CHG_REG_CNFG_06, MAX77865_CNFG_06_CHGPROT,
				   MAX77865_CNFG_06_CHGPROT);
		msleep(20);
	}
	/* 2. CHGPROT 3 << 2 | slowest LX slope 0x60 (:726-732) */
	regmap_update_bits(map, MAX77865_CHG_REG_CNFG_06, 0x6C, 0x6C);
	/* 3. restart threshold off 0x03 << 4 | 0x08, fast-charge timer off (:734-741) */
	regmap_write(map, MAX77865_CHG_REG_CNFG_01, 0x38);
	/* 4. OTG current limit field [7:6] = 3 (:743-747) */
	regmap_update_bits(map, MAX77865_CHG_REG_CNFG_02, 0xC0, 0xC0);
	/* 5. BAT to SYS OCP: probe default vsys_ocp = 0x04 (:1890, mask :750) */
	regmap_update_bits(map, MAX77865_CHG_REG_CNFG_05, 0x07, 0x04);
	/* 6. top-off 150 mA / 30 min (:752-757) */
	regmap_write(map, MAX77865_CHG_REG_CNFG_03, 0x18);
	/* 7. float voltage 43500 (FDT:12281; :759-763, encoding :201-215) */
	regmap_update_bits(map, MAX77865_CHG_REG_CNFG_04, MAX77865_CNFG_04_CHG_CV_PRM,
			   CV_CODE_NORMAL);
	/* 8. VCHGIN / WCIN regulation 4.3 V, UVLO 4.5 V (:765-770) */
	regmap_update_bits(map, MAX77865_CHG_REG_CNFG_12,
			   MAX77865_CNFG_12_VCHGIN_REG | MAX77865_CNFG_12_WCIN_REG, 0x12);
	/* 9. FMBST (:772-774) */
	regmap_update_bits(map, MAX77865_CHG_REG_CNFG_07, MAX77865_CNFG_07_FMBST,
			   MAX77865_CNFG_07_FMBST);
	/* 10. FGSRC := JIG level (:776-783; fuelgauge,jig_gpio + jig_low_active FDT:12332-12333) */
	if (chg->jig)
		jig = gpiod_get_value_cansleep(chg->jig) > 0;
	regmap_update_bits(map, MAX77865_CHG_REG_CNFG_07, MAX77865_CNFG_07_FGSRC,
			   jig ? MAX77865_CNFG_07_FGSRC : 0);
	/* 11. watchdog enable (:785-787) */
	regmap_update_bits(map, MAX77865_CHG_REG_CNFG_00, MAX77865_CNFG_00_WDTEN,
			   MAX77865_CNFG_00_WDTEN);
	/* 12. PMIC MAINCTRL1 active discharge (:789-791) */
	regmap_update_bits(core, MAX77865_PMIC_REG_MAINCTRL1, MAX77865_MAINCTRL1_ADEN,
			   MAX77865_MAINCTRL1_ADEN);

	/* 13. read-back guard, charge-off fault mode on a mismatch (L6F 5.3.2 step 13) */
	for (i = 0; i < ARRAY_SIZE(max77865_readback); i++) {
		const struct max77865_readback *r = &max77865_readback[i];
		int ret = regmap_read(map, r->reg, &v);

		if (ret || (v & r->mask) != r->val) {
			if (ret)
				dev_err(chg->dev, "readback CNFG 0x%02x failed (%d)\n", r->reg, ret);
			dev_err(chg->dev,
				"readback CNFG 0x%02x=0x%02x expected 0x%02x/0x%02x, charging off (fault mode)\n",
				r->reg, ret ? 0 : v, r->mask, r->val);
			chg->fault = 1;
			break;
		}
	}
	if (!chg->fault)
		return;

	/* 13a. stock CHARGING_OFF: CHG 0, DISSKIP 0, buck kept (CHG:569-588, :1046-1054) */
	regmap_update_bits(map, MAX77865_CHG_REG_CNFG_00, MAX77865_CNFG_00_CHG, 0);
	regmap_update_bits(map, MAX77865_CHG_REG_CNFG_12, MAX77865_CNFG_12_DISSKIP, 0);
	/* 13b/c; a failed read counts as still on */
	if (!regmap_read(map, MAX77865_CHG_REG_CNFG_00, &v) && !(v & MAX77865_CNFG_00_CHG)) {
		chg->hw_mode = MODE_CHARGING_OFF;
		dev_warn(chg->dev, "fault mode, CNFG_00=0x%02x\n", v);
		return;
	}
	chg->fault = 2;
	dev_err(chg->dev, "cannot switch charging off (CNFG_00=0x%02x), watchdog kick stopped\n", v);
}

/* ---- power-off with a charger online (L6F 5.3.7) ---- */

static int max77865_poweroff_prepare(struct sys_off_data *data)
{
	struct max77865_charger *chg = data->cb_data;
	unsigned int v;

	/* stock sec_power_off() read ac/usb before the ACPM stop (sec_reboot.c:107-110, :117) */
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
	regmap_write(chg->pmu, 0x0808 /* INFORM2 */, 0x0 /* SEC_POWER_OFF */);
	/* S:drivers/staging/samsung/sec_reboot.c:43, :126; offset S:include/linux/sec_ext.h:27 */
	regmap_write(chg->pmu, 0x0400 /* SWRESET */, 0x1);
	/* stock mach_restart = exynos_reboot(): S:drivers/soc/samsung/exynos-reboot.c:75, :233-234 */
	mdelay(1000);
	dev_emerg(chg->dev, "SWRESET did not reset, falling through to power-off\n");
	return NOTIFY_DONE;
}

/* ---- debugfs (L6F 5.3.5) ---- */

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
	debugfs_create_u32("override_usb_mc", 0600, d, &chg->override_usb_mc);
	debugfs_create_u32("override_chg_mc", 0600, d, &chg->override_chg_mc);
	debugfs_create_u32("override_t_mc", 0600, d, &chg->override_t_mc);
	debugfs_create_u32("override_hc", 0600, d, &chg->override_hc);
	debugfs_create_bool("wdt_kick", 0600, d, &chg->wdt_kick);
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
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_TEST) && IS_ENABLED(CONFIG_CHARGER_MAX77865_HV_GUARD)
	debugfs_create_u32("override_pd_mv", 0600, d, &chg->override_pd_mv);
#endif
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_TEST) && IS_ENABLED(CONFIG_CHARGER_MAX77865_VBAT_OVP)
	debugfs_create_u32("override_vcell_mv", 0600, d, &chg->override_vcell_mv);
#endif

	return devm_add_action_or_reset(chg->dev, max77865_debugfs_remove, d);
}

/* ---- probe / shutdown ---- */

static void max77865_fg_put(void *data)
{
	struct max77865_charger *chg = data;

	if (chg->fg)
		power_supply_put(chg->fg);
}

/* runs before the work cancels on unbind: no pass, alarm or OTG op re-arms anything */
static void max77865_stop(void *data)
{
	struct max77865_charger *chg = data;

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
 * CCIC notifications: a copy of the sink status for the pass. The S2MM005
 * calls the chain after releasing its own lock, so chg->lock -> S2MM005
 * lock (select_pdo from the pass) is the only nesting.
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
	if (!chg->stopping)
		mod_delayed_work(system_freezable_wq, &chg->pass_dwork, 0);
	mutex_unlock(&chg->lock);
	return NOTIFY_OK;
}

static void max77865_ccic_put(void *data)
{
	s2mm005_put(data);
}

static void max77865_ccic_unregister(void *data)
{
	struct max77865_charger *chg = data;

	s2mm005_unregister_notifier(chg->ccic, &chg->ccic_nb);
}

/* before any charger write: a deferral here has no side effects */
static int max77865_ccic_get(struct max77865_charger *chg)
{
	struct device_node *np = of_find_compatible_node(NULL, NULL, "samsung,s2mm005");
	struct s2mm005 *ccic;

	if (!np || !of_device_is_available(np)) {
		of_node_put(np);
		dev_info(chg->dev, "no S2MM005, no PD sink status\n");
		return 0;
	}
	ccic = s2mm005_get(np);
	of_node_put(np);
	if (IS_ERR(ccic))
		return dev_err_probe(chg->dev, PTR_ERR(ccic), "S2MM005\n");
	chg->ccic = ccic;
	return devm_add_action_or_reset(chg->dev, max77865_ccic_put, ccic);
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
	/* stock lpcharge (S:drivers/staging/samsung/sec_batt.c:24-33) */
	chg->lpcharge = !!strstr(saved_command_line, "androidboot.mode=charger");
	chg->charge_control = of_property_read_bool(dev->of_node, "samsung,charge-control");
	if (chg->charge_control && !IS_ENABLED(CONFIG_CHARGER_MAX77865_CHARGE_CONTROL)) {
		/* charging control is built only on request (Kconfig help) */
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

	/* stock only reads the level, never sets a direction (CHG:776-779; V4-16) */
	chg->jig = devm_gpiod_get_optional(dev, "jig", GPIOD_ASIS);
	if (IS_ERR(chg->jig))
		return dev_err_probe(dev, PTR_ERR(chg->jig), "jig gpio\n");

#if MAX77865_CCIC
	ret = max77865_ccic_get(chg);
	if (ret)
		return ret;
#endif

	/* PMIC core regmap for MAINCTRL1 (L6F 5.3.2 step 12) */
	if (chg->charge_control) {
		core_map = max77865_core_regmap(dev, &core);
		if (IS_ERR(core_map))
			return dev_err_probe(dev, PTR_ERR(core_map), "no MAX77865 core\n");
		ret = devm_add_action_or_reset(dev, max77865_put_core, core);
		if (ret)
			return ret;
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_SYSOVLO)
		/* the SYSOVLO virq of the MFD's SYSTEM_INT sub-chip */
		{
			struct max77693_dev *max = i2c_get_clientdata(core);

			if (max->sys_irq_data)
				chg->sysovlo_irq = regmap_irq_get_virq(max->sys_irq_data, 5);
		}
#endif
	}

	psy_cfg.drv_data = chg;
	psy_cfg.fwnode = dev_fwnode(dev);	/* the FG's power-supplies phandle */
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

	/*
	 * Works before the regulator (its ops queue the pass); on unbind the stop action
	 * runs first, then the CHGIN work and the pass are cancelled.
	 */
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
	ret = devm_add_action_or_reset(dev, max77865_stop, chg);
	if (ret)
		return ret;
#if MAX77865_CCIC
	/* after the pass work exists; unregistered before it is cancelled */
	if (chg->ccic) {
		chg->ccic_nb.notifier_call = max77865_ccic_notify;
		ret = s2mm005_register_notifier(chg->ccic, &chg->ccic_nb);
		if (ret)
			return dev_err_probe(dev, ret, "S2MM005 notifier\n");
		ret = devm_add_action_or_reset(dev, max77865_ccic_unregister, chg);
		if (ret)
			return ret;
	}
#endif

	reg_cfg.dev = dev;
	reg_cfg.driver_data = chg;
	chg->otg_rdev = devm_regulator_register(dev, &max77865_otg_desc, &reg_cfg);
	if (IS_ERR(chg->otg_rdev))
		return dev_err_probe(dev, PTR_ERR(chg->otg_rdev), "otg regulator\n");

	/* before L2's ACPM stop at SYS_OFF_PRIO_DEFAULT (L6F 5.3.7, V-6) */
	ret = devm_register_sys_off_handler(dev, SYS_OFF_MODE_POWER_OFF_PREPARE, SYS_OFF_PRIO_HIGH,
					    max77865_poweroff_prepare, chg);
	if (ret)
		return dev_err_probe(dev, ret, "power-off prepare handler\n");
	ret = devm_register_sys_off_handler(dev, SYS_OFF_MODE_POWER_OFF, SYS_OFF_PRIO_HIGH,
					    max77865_poweroff, chg);
	if (ret)
		return dev_err_probe(dev, ret, "power-off handler\n");

	/* CNFG_00..CNFG_12 as found (L6F 5.3.2) */
	for (i = 0; i < ARRAY_SIZE(cnfg); i++) {
		ret = regmap_read(chg->regmap, MAX77865_CHG_REG_CNFG_00 + i, &cnfg[i]);
		if (ret)
			return dev_err_probe(dev, ret, "CNFG read failed\n");
	}

	/* after every registration that can fail or defer (the IRQ request undoes WDTEN) */
	if (chg->charge_control) {
		max77865_chg_init(chg, core_map);
		if (!chg->fault) {
			chg->cv_w = CV_CODE_NORMAL;
			dev_info(dev, "charge control, float %d mV\n", CV_CODE_TO_MV(CV_CODE_NORMAL));
		}
	} else {
		dev_info(dev, "monitor-only mode, CNFG_00=0x%02x WDTEN=%d\n", cnfg[0],
			 !!(cnfg[0] & MAX77865_CNFG_00_WDTEN));
	}

	/*
	 * only CHGIN_I unmasked; AICL_I too with CONFIG_CHARGER_MAX77865_AICL in
	 * charge control (0x3F instead of 0xBF)
	 */
	ret = regmap_write(chg->regmap, MAX77865_CHG_REG_INT_MASK,
			   IS_ENABLED(CONFIG_CHARGER_MAX77865_AICL) && chg->charge_control ?
			   0x3F : 0xBF);
	if (ret)
		return dev_err_probe(dev, ret, "CHG_INT_MASK write failed\n");

	ret = devm_request_threaded_irq(dev, client->irq, NULL, max77865_chg_irq,
					IRQF_ONESHOT,
					"max77865-charger", chg);
	if (ret) {
		/* nothing will kick the watchdog: stock shutdown value (CHG:2109-2111) */
		if (chg->charge_control && chg->fault != 2)
			regmap_write(chg->regmap, MAX77865_CHG_REG_CNFG_00, 0x04);
		return dev_err_probe(dev, ret, "irq %d\n", client->irq);
	}

	/*
	 * stock does this on the next line of the same if block
	 * (S:drivers/battery_v2/max77865_charger.c:1968-1972), the port had dropped it.
	 * There is no IRQF_WAKEUP on arm64 (6.18 keeps it for ppc64 only), so the wake
	 * source is armed the way stock arms it.
	 *
	 * CHGIN_I is the only unmasked source (CHG_INT_MASK 0xBF above) and it is what
	 * turns "usb online" off on a detach; without the wake source an unplug that
	 * Android answers with a suspend is not seen at all, because the pass that clears
	 * ONLINE is max77865_pass_work (:917-919), a delayed work on system_freezable_wq
	 * (:1080), and is frozen for the whole suspend. The pad itself is armed by the
	 * parent (mfd/max77865.c suspend: enable_irq_wake before disable_irq); the DT needs
	 * no change, wakeup-source is already on pmic@66
	 * (arch/arm64/boot/dts/exynos/exynos8895-greatlte-l6f.dtsi:237).
	 */
	if (enable_irq_wake(client->irq) < 0)
		dev_warn(dev, "failed to enable wakeup source\n");

#if IS_ENABLED(CONFIG_CHARGER_MAX77865_SYSOVLO)
	/*
	 * Requested masked (stock MFD init masks SYSTEM_INT) and armed on the first
	 * removal; BIAS on as stock. After every lock/work the
	 * handler touches.
	 */
	if (chg->sysovlo_irq > 0) {
		regmap_update_bits(core_map, MAX77865_PMIC_REG_MAINCTRL1, MAX77865_MAINCTRL1_BIASEN,
				   MAX77865_MAINCTRL1_BIASEN);
		ret = devm_request_threaded_irq(dev, chg->sysovlo_irq, NULL, max77865_sysovlo_irq,
						IRQF_ONESHOT | IRQF_NO_AUTOEN, "max77865-sysovlo", chg);
		if (ret)
			return dev_err_probe(dev, ret, "sysovlo irq %d\n", chg->sysovlo_irq);
		enable_irq_wake(chg->sysovlo_irq);	/* stock wake lock 5 s */
	}
#endif

	queue_delayed_work(system_freezable_wq, &chg->pass_dwork, 0);
	return 0;
}

/*
 * L6F 5.3.6: stop the policy; a boost left on goes off (5.3.8); in charge control the
 * stock four writes (CHG:2098-2122) so S-Boot starts with charging and the WDT off.
 */
static void max77865_charger_shutdown(struct i2c_client *client)
{
	struct max77865_charger *chg = i2c_get_clientdata(client);

	max77865_stop(chg);
	cancel_work_sync(&chg->chgin_work);
	cancel_delayed_work_sync(&chg->pass_dwork);

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
