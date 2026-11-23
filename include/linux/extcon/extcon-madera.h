/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * extcon-madera.h - accessory detection for Cirrus Logic Madera codecs
 *
 * Copyright 2015-2016 Cirrus Logic
 */

#ifndef EXTCON_MADERA_H
#define EXTCON_MADERA_H

#include <linux/kernel.h>
#include <linux/mfd/madera/registers.h>

/* Register bits the upstream registers.h does not define */
#define MADERA_EDRE_MANUAL		0x44A
#define MADERA_HP_DONE			0x8000

#define MADERA_MAX_ACCESSORY		1

/* Treat INT_MAX impedance as open circuit */
#define MADERA_HP_Z_OPEN		INT_MAX

/* output clamp pin for cirrus,hpd-pins */
#define MADERA_HPD_OUT_OUT1L		0
#define MADERA_HPD_OUT_OUT1R		1
#define MADERA_HPD_OUT_OUT2L		2
#define MADERA_HPD_OUT_OUT2R		3

/* Sense pin selection for cirrus,hpd-pins */
#define MADERA_HPD_SENSE_MICDET1	0
#define MADERA_HPD_SENSE_MICDET2	1
#define MADERA_HPD_SENSE_MICDET3	2
#define MADERA_HPD_SENSE_MICDET4	3
#define MADERA_HPD_SENSE_HPDET1		4
#define MADERA_HPD_SENSE_HPDET2		5
#define MADERA_HPD_SENSE_JD1		6
#define MADERA_HPD_SENSE_JD2		7

/* source setting for cirrus,micd-configs (cs47l35, cs47l85) */
#define MADERA_ACCD_SENSE_MICDET1	0
#define MADERA_ACCD_SENSE_MICDET2	1

/* bias setting for  cirrus,micd-configs (cs47l35, cs47l85) */
#define MADERA_ACCD_BIAS_SRC_MICBIAS1	1
#define MADERA_ACCD_BIAS_SRC_MICBIAS2	2

/* source setting for cirrus,micd-configs */
#define MADERA_MICD1_SENSE_MICDET1	0
#define MADERA_MICD1_SENSE_MICDET2	1
#define MADERA_MICD1_SENSE_MICDET3	2
#define MADERA_MICD1_SENSE_MICDET4	3

/* ground setting for cirrus,micd-configs */
#define MADERA_MICD1_GND_MICDET1	0
#define MADERA_MICD1_GND_MICDET2	1
#define MADERA_MICD1_GND_MICDET3	2
#define MADERA_MICD1_GND_MICDET4	3

/* bias setting for cirrus,micd-configs */
#define MADERA_MICD_BIAS_SRC_MICBIAS1A	0x0
#define MADERA_MICD_BIAS_SRC_MICBIAS1B	0x1
#define MADERA_MICD_BIAS_SRC_MICBIAS1C	0x2
#define MADERA_MICD_BIAS_SRC_MICBIAS1D	0x3
#define MADERA_MICD_BIAS_SRC_MICBIAS2A	0x4
#define MADERA_MICD_BIAS_SRC_MICBIAS2B	0x5
#define MADERA_MICD_BIAS_SRC_MICBIAS2C	0x6
#define MADERA_MICD_BIAS_SRC_MICBIAS2D	0x7
#define MADERA_MICD_BIAS_SRC_MICVDD	0xF

/* HP gnd setting for cirrus,micd-configs */
#define MADERA_HPD_GND_HPOUTFB1		0
#define MADERA_HPD_GND_HPOUTFB2		1
#define MADERA_HPD_GND_HPOUTFB3		2
#define MADERA_HPD_GND_HPOUTFB4		3

#define MADERA_MICD_LVL_1_TO_7 \
		(MADERA_MICD_LVL_1 | MADERA_MICD_LVL_2 | \
		 MADERA_MICD_LVL_3 | MADERA_MICD_LVL_4 | \
		 MADERA_MICD_LVL_5 | MADERA_MICD_LVL_6 | \
		 MADERA_MICD_LVL_7)

#define MADERA_MICD_LVL_0_TO_7 (MADERA_MICD_LVL_0 | MADERA_MICD_LVL_1_TO_7)

#define MADERA_MICD_LVL_0_TO_8 (MADERA_MICD_LVL_0_TO_7 | MADERA_MICD_LVL_8)

/* Notify data structure for MADERA_NOTIFY_HPDET */
struct madera_hpdet_notify_data {
	unsigned int impedance_x100;	/* ohms * 100 */
};

/* Notify data structure for MADERA_NOTIFY_MICDET */
struct madera_micdet_notify_data {
	unsigned int impedance_x100;	/* ohms * 100 */
	bool present;
	int out_num;			/* 1 = OUT1, 2 = OUT2 */
};

struct madera_extcon_info;
struct madera_jd_state;

/** Bias and sense configuration for probing MICDET */
struct madera_micd_config {
	/** cs47l35, cs47l85, WM1840: value of the ACCDET_SRC field
	 *  other codecs: value of MICDn_SENSE_SEL field
	 * See datasheet for field values
	 */
	unsigned int src;

	/** cs47l35, cs47l85, wm1840: unused
	 *  other codecs: value of MICDn_GND_SEL
	 *
	 * See datasheet for field values
	 */
	unsigned int gnd;

	/** MICBIAS number to enable */
	unsigned int bias;

	/** State of polarity gpio during probe (true == gpio asserted) */
	bool gpio;

	/** cs47l35, cs47l85, wm1840: unused
	 *  other codecs: value of HPn_GND_SEL
	 *
	 * See datasheet for field values
	 */
	unsigned int hp_gnd;

};

struct madera_micd_range {
	int max;  /** Ohms */
	int key;  /** Key to report to input layer */
};

/** pdata sub-structure for accessory detect configuration */
struct madera_accdet_pdata {
	/** set to true to enable this accessory detect */
	bool enabled;

	/** Output this accdet pdata refers */
	unsigned int output;

	/** Time in milliseconds to keep wake lock during jack detection */
	int jd_wake_time;

	/** Set whether JD2 is used for jack detection */
	bool jd_use_jd2;

	/** set to true if jackdet contact opens on insert */
	bool jd_invert;

	/** If non-zero don't run headphone detection, just report this value
	 * Specified as hundredths-of-an-ohm, that is (ohms * 100)
	 */
	int fixed_hpdet_imp_x100;

	/** Impedance of external series resistor on hpdet.
	 * Specified as hundredths-of-an-ohm, that is (ohms * 100)
	 */
	int hpdet_ext_res_x100;

	/** If non-zero, specifies the maximum impedance in ohms
	 * that will be considered as a short circuit.
	 */
	int hpdet_short_circuit_imp;

	/**
	 * Channel to use for headphone detection, valid values are 0 for
	 * left and 1 for right
	 */
	int hpdet_channel;

	/** Extra debounce timeout during initial mic detect (milliseconds) */
	int micd_detect_debounce_ms;

	/** Extra software debounces during button detection */
	int micd_manual_debounce;

	/** Mic detect ramp rate */
	int micd_bias_start_time;

	/** Setting for the codec MICD_RATE field
	 * See the  datasheet for documentation of the field values
	 */
	int micd_rate;

	/** Mic detect debounce level */
	int micd_dbtime;

	/** Mic detect timeout (milliseconds) */
	int micd_timeout_ms;

	/** Mic detect clamp function */
	unsigned int micd_clamp_mode;

	/** Force MICBIAS on for mic detect */
	bool micd_force_micbias;

	/** Declare an open circuit as a 4 pole jack */
	bool micd_open_circuit_declare;

	/** Use software comparison to determine mic presence */
	bool micd_software_compare;

	/** Mic detect level parameters */
	const struct madera_micd_range *micd_ranges;
	int num_micd_ranges;


	/** Headset polarity configurations */
	const struct madera_micd_config *micd_configs;
	int num_micd_configs;

	/**
	 * 4 entries:
	 * [HPL_clamp_pin, HPL_impedance_measurement_pin,
	 * HPR_clamp_pin, HPR_impedance_measurement_pin]
	 *
	 * 0 = use default setting
	 * values 0x1..0xffff = set to this value
	 * >0xffff = set to 0
	 */
	unsigned int hpd_pins[4];

	/** Moisture detection pin */
	int moisture_pin;
	/** Moisture detection impedance threshold */
	int moisture_imp;
	/** Number of debounces to perform on moisture detection */
	int moisture_debounce;

	/** Delay between jack detection and any futher action */
	int init_delay;

	/**
	 * Specify an input to mute during headset button presses:
	 * 1 - IN1L, 2 - IN1R, ..., n - IN[n]R
	 */
	u32 hs_mic;

	/** Override the normal jack detection */
	const struct madera_jd_state *custom_jd;
};

enum madera_accdet_mode {
	MADERA_ACCDET_MODE_MIC,
	MADERA_ACCDET_MODE_HPL,
	MADERA_ACCDET_MODE_HPR,
	MADERA_ACCDET_MODE_HPM,
	MADERA_ACCDET_MODE_ADC,
	MADERA_ACCDET_MODE_INVALID,
};

struct madera_jd_state {
	enum madera_accdet_mode mode;

	int (*start)(struct madera_extcon_info *);
	void (*restart)(struct madera_extcon_info *);
	int (*reading)(struct madera_extcon_info *, int);
	void (*stop)(struct madera_extcon_info *);

	int (*timeout_ms)(struct madera_extcon_info *);
	void (*timeout)(struct madera_extcon_info *);
};

int madera_jds_set_state(struct madera_extcon_info *info,
			 const struct madera_jd_state *new_state);

extern void madera_set_headphone_imp(struct madera_extcon_info *info,
				     int imp);

extern const struct madera_jd_state madera_hpdet_left;
extern const struct madera_jd_state madera_hpdet_right;
extern const struct madera_jd_state madera_hpdet_moisture;
extern const struct madera_jd_state madera_micd_button;
extern const struct madera_jd_state madera_micd_microphone;
extern const struct madera_jd_state madera_micd_adc_mic;

extern int madera_hpdet_start(struct madera_extcon_info *info);
extern void madera_hpdet_restart(struct madera_extcon_info *info);
extern void madera_hpdet_stop(struct madera_extcon_info *info);
extern int madera_hpdet_reading(struct madera_extcon_info *info, int val);

extern int madera_micd_start(struct madera_extcon_info *info);
extern void madera_micd_stop(struct madera_extcon_info *info);
extern int madera_micd_button_start(struct madera_extcon_info *info);
extern int madera_micd_button_reading(struct madera_extcon_info *info, int val);

extern int madera_micd_mic_start(struct madera_extcon_info *info);
extern void madera_micd_mic_stop(struct madera_extcon_info *info);
extern int madera_micd_mic_reading(struct madera_extcon_info *info, int val);
extern int madera_micd_mic_timeout_ms(struct madera_extcon_info *info);
extern void madera_micd_mic_timeout(struct madera_extcon_info *info);

extern void madera_extcon_report(struct madera_extcon_info *info, int state);

extern int madera_extcon_manual_mic_reading(struct madera_extcon_info *info);

#endif
