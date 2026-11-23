// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * exynos_tmu.c - Samsung Exynos TMU (Thermal Management Unit)
 *
 *  Copyright (C) 2014 Samsung Electronics
 *  Bartlomiej Zolnierkiewicz <b.zolnierkie@samsung.com>
 *  Lukasz Majewski <l.majewski@samsung.com>
 *
 *  Copyright (C) 2011 Samsung Electronics
 *  Donggeun Kim <dg77.kim@samsung.com>
 *  Amit Daniel Kachhap <amit.kachhap@linaro.org>
 */

#include <linux/cleanup.h>
#include <linux/clk.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/pm_qos.h>
#include <linux/regulator/consumer.h>
#include <linux/thermal.h>
#include <linux/workqueue.h>

#include <dt-bindings/thermal/thermal_exynos.h>

#if IS_BUILTIN(CONFIG_EXYNOS_THERMAL)
/* thermal_zone_device_set_policy() and struct thermal_instance are core-internal */
#include "../thermal_core.h"
#endif

/* Exynos generic registers */
#define EXYNOS_TMU_REG_TRIMINFO		0x0
#define EXYNOS_TMU_REG_CONTROL		0x20
#define EXYNOS_TMU_REG_STATUS		0x28
#define EXYNOS_TMU_REG_CURRENT_TEMP	0x40
#define EXYNOS_TMU_REG_INTEN		0x70
#define EXYNOS_TMU_REG_INTSTAT		0x74
#define EXYNOS_TMU_REG_INTCLEAR		0x78

#define EXYNOS_TMU_TEMP_MASK		0xff
#define EXYNOS_TMU_REF_VOLTAGE_SHIFT	24
#define EXYNOS_TMU_REF_VOLTAGE_MASK	0x1f
#define EXYNOS_TMU_BUF_SLOPE_SEL_MASK	0xf
#define EXYNOS_TMU_BUF_SLOPE_SEL_SHIFT	8
#define EXYNOS_TMU_CORE_EN_SHIFT	0

/* Exynos3250 specific registers */
#define EXYNOS_TMU_TRIMINFO_CON1	0x10

/* Exynos4210 specific registers */
#define EXYNOS4210_TMU_REG_THRESHOLD_TEMP	0x44
#define EXYNOS4210_TMU_REG_TRIG_LEVEL0	0x50

/* Exynos5250, Exynos4412, Exynos3250 specific registers */
#define EXYNOS_TMU_TRIMINFO_CON2	0x14
#define EXYNOS_THD_TEMP_RISE		0x50
#define EXYNOS_THD_TEMP_FALL		0x54
#define EXYNOS_EMUL_CON		0x80

#define EXYNOS_TRIMINFO_RELOAD_ENABLE	1
#define EXYNOS_TRIMINFO_25_SHIFT	0
#define EXYNOS_TRIMINFO_85_SHIFT	8
#define EXYNOS_TMU_TRIP_MODE_SHIFT	13
#define EXYNOS_TMU_TRIP_MODE_MASK	0x7
#define EXYNOS_TMU_THERM_TRIP_EN_SHIFT	12

#define EXYNOS_TMU_INTEN_RISE0_SHIFT	0
#define EXYNOS_TMU_INTEN_FALL0_SHIFT	16

#define EXYNOS_EMUL_TIME	0x57F0
#define EXYNOS_EMUL_TIME_MASK	0xffff
#define EXYNOS_EMUL_TIME_SHIFT	16
#define EXYNOS_EMUL_DATA_SHIFT	8
#define EXYNOS_EMUL_DATA_MASK	0xFF
#define EXYNOS_EMUL_ENABLE	0x1

/* Exynos5260 specific */
#define EXYNOS5260_TMU_REG_INTEN		0xC0
#define EXYNOS5260_TMU_REG_INTSTAT		0xC4
#define EXYNOS5260_TMU_REG_INTCLEAR		0xC8
#define EXYNOS5260_EMUL_CON			0x100

/* Exynos4412 specific */
#define EXYNOS4412_MUX_ADDR_VALUE          6
#define EXYNOS4412_MUX_ADDR_SHIFT          20

/* Exynos5433 specific registers */
#define EXYNOS5433_THD_TEMP_RISE3_0		0x050
#define EXYNOS5433_THD_TEMP_RISE7_4		0x054
#define EXYNOS5433_THD_TEMP_FALL3_0		0x060
#define EXYNOS5433_THD_TEMP_FALL7_4		0x064
#define EXYNOS5433_TMU_REG_INTEN		0x0c0
#define EXYNOS5433_TMU_REG_INTPEND		0x0c8
#define EXYNOS5433_TMU_EMUL_CON			0x110
#define EXYNOS5433_TMU_PD_DET_EN		0x130

#define EXYNOS5433_TRIMINFO_SENSOR_ID_SHIFT	16
#define EXYNOS5433_TRIMINFO_CALIB_SEL_SHIFT	23
#define EXYNOS5433_TRIMINFO_SENSOR_ID_MASK	\
			(0xf << EXYNOS5433_TRIMINFO_SENSOR_ID_SHIFT)
#define EXYNOS5433_TRIMINFO_CALIB_SEL_MASK	BIT(23)

#define EXYNOS5433_TRIMINFO_ONE_POINT_TRIMMING	0
#define EXYNOS5433_TRIMINFO_TWO_POINT_TRIMMING	1

#define EXYNOS5433_PD_DET_EN			1

#define EXYNOS5433_G3D_BASE			0x10070000

/* Exynos7 specific registers */
#define EXYNOS7_THD_TEMP_RISE7_6		0x50
#define EXYNOS7_THD_TEMP_FALL7_6		0x60
#define EXYNOS7_TMU_REG_INTEN			0x110
#define EXYNOS7_TMU_REG_INTPEND			0x118
#define EXYNOS7_TMU_REG_EMUL_CON		0x160

#define EXYNOS7_TMU_TEMP_MASK			0x1ff
#define EXYNOS7_PD_DET_EN_SHIFT			23
#define EXYNOS7_TMU_INTEN_RISE0_SHIFT		0
#define EXYNOS7_EMUL_DATA_SHIFT			7
#define EXYNOS7_EMUL_DATA_MASK			0x1ff

/* Exynos8895 specific registers: up to eight sensors per block */
#define EXYNOS8895_TMU_REG_TRIMINFO1		0x4
#define EXYNOS8895_TMU_REG_TRIMINFO2		0x8
#define EXYNOS8895_TMU_REG_CONTROL1		0x24
#define EXYNOS8895_TMU_REG_AVG_CON		0x38
#define EXYNOS8895_TMU_REG_CURRENT_TEMP1_0	0x40
#define EXYNOS8895_THD_TEMP_RISE7_6		0x50
#define EXYNOS8895_THD_TEMP_FALL7_6		0x60
#define EXYNOS8895_THD_TEMP_R_OFFSET		0x100
#define EXYNOS8895_THD_TEMP_R_STRIDE		0x20
#define EXYNOS8895_TMU_REG_INTEN0		0x110
#define EXYNOS8895_TMU_REG_INTPEND0		0x118
#define EXYNOS8895_TMU_REG_INTEN5		0x310
#define EXYNOS8895_TMU_REG_INTPEND5		0x318
#define EXYNOS8895_TMU_REG_INT_STRIDE		0x10
#define EXYNOS8895_TMU_NUM_SENSORS		8
#define EXYNOS8895_TMU_TEMP_MASK		0x1ff
#define EXYNOS8895_TMU_TEMP_SHIFT		9
#define EXYNOS8895_TMU_CALIB_SEL_SHIFT		23
#define EXYNOS8895_TMU_NUM_PROBE_SHIFT		16
#define EXYNOS8895_TMU_NUM_PROBE_MASK		0x7
#define EXYNOS8895_TMU_T_BUF_VREF_SEL_SHIFT	18
#define EXYNOS8895_TMU_T_BUF_VREF_SEL_MASK	0x1f
#define EXYNOS8895_TMU_T_BUF_SLOPE_SEL_SHIFT	18
#define EXYNOS8895_TMU_T_BUF_SLOPE_SEL_MASK	0xf
#define EXYNOS8895_TMU_AVG_CON_SHIFT		18
#define EXYNOS8895_TMU_AVG_CON_MASK		0x3
#define EXYNOS8895_TMU_AVG_MODE_MASK		0x7
#define EXYNOS8895_TMU_AVG_MODE_DEFAULT		0x0
#define EXYNOS8895_TMU_AVG_MODE_4		0x6
#define EXYNOS8895_TMU_DEM_SHIFT		4
#define EXYNOS8895_TMU_INTEN_RISE6		6
#define EXYNOS8895_TMU_INTEN_RISE7		7
#define EXYNOS8895_TMU_BALANCE_OFFSET		20
#define EXYNOS8895_TMU_MIN_TEMP			10
#define EXYNOS8895_TMU_MAX_TEMP			125

#define EXYNOS_FIRST_POINT_TRIM			25
#define EXYNOS_SECOND_POINT_TRIM		85

#define EXYNOS_NOISE_CANCEL_MODE		4

#define MCELSIUS	1000

enum soc_type {
	SOC_ARCH_EXYNOS3250 = 1,
	SOC_ARCH_EXYNOS4210,
	SOC_ARCH_EXYNOS4412,
	SOC_ARCH_EXYNOS5250,
	SOC_ARCH_EXYNOS5260,
	SOC_ARCH_EXYNOS5420,
	SOC_ARCH_EXYNOS5420_TRIMINFO,
	SOC_ARCH_EXYNOS5433,
	SOC_ARCH_EXYNOS7,
	SOC_ARCH_EXYNOS8895,
};

/**
 * struct exynos_tmu_data : A structure to hold the private data of the TMU
 *			    driver
 * @base: base address of the single instance of the TMU controller.
 * @base_second: base address of the common registers of the TMU controller.
 * @irq: irq number of the TMU controller.
 * @soc: id of the SOC type.
 * @lock: lock to implement synchronization.
 * @clk: pointer to the clock structure.
 * @clk_sec: pointer to the clock structure for accessing the base_second.
 * @sclk: pointer to the clock structure for accessing the tmu special clk.
 * @cal_type: calibration type for temperature
 * @efuse_value: SoC defined fuse value
 * @min_efuse_value: minimum valid trimming data
 * @max_efuse_value: maximum valid trimming data
 * @temp_error1: fused value of the first point trim.
 * @temp_error2: fused value of the second point trim.
 * @gain: gain of amplifier in the positive-TC generator block
 *	0 < gain <= 15
 * @reference_voltage: reference voltage of amplifier
 *	in the positive-TC generator block
 *	0 < reference_voltage <= 31
 * @tzd: pointer to thermal_zone_device structure
 * @enabled: current status of TMU device
 * @tmu_set_low_temp: SoC specific method to set trip (falling threshold)
 * @tmu_set_high_temp: SoC specific method to set trip (rising threshold)
 * @tmu_set_crit_temp: SoC specific method to set critical temperature
 * @tmu_disable_low: SoC specific method to disable an interrupt (falling threshold)
 * @tmu_disable_high: SoC specific method to disable an interrupt (rising threshold)
 * @tmu_initialize: SoC specific TMU initialization method
 * @tmu_control: SoC specific TMU control method
 * @tmu_read: SoC specific TMU temperature read method
 * @tmu_set_emulation: SoC specific TMU emulation setting method
 * @tmu_clear_irqs: SoC specific TMU interrupts clearing method
 * @sensors: Exynos8895: bit mask of the sensors of this instance.
 * @balance: Exynos8895: "balance" sensing mode, else "max".
 * @num_probe: Exynos8895: CONTROL1 NUM_PROBE as read at probe.
 * @num_probe_read: Exynos8895: @num_probe was read.
 * @s_cal_type: Exynos8895: calibration type per used sensor.
 * @s_err1: Exynos8895: first-point trim per used sensor.
 * @s_err2: Exynos8895: second-point trim per used sensor.
 * @avg_con: Exynos8895: AVG_CON value written with CONTROL.
 * @dev: the TMU device.
 * @ipa: Exynos8895: the zone runs on power_allocator once its cooling
 *	devices are power actors (samsung,ipa-* properties).
 * @ipa_k_po: Exynos8895: samsung,ipa-k-po, mW per degree Celsius.
 * @ipa_k_pu: Exynos8895: samsung,ipa-k-pu, mW per degree Celsius.
 * @ipa_k_i: Exynos8895: samsung,ipa-k-i, mW per degree Celsius.
 * @ipa_integral_max: Exynos8895: samsung,ipa-integral-max, mW.
 * @ipa_integral_cutoff: Exynos8895: samsung,ipa-integral-cutoff, degrees.
 * @ipa_node: Exynos8895: entry in the list of zones waiting for the
 *	power_allocator selection.
 * @hp_cpus: Exynos8895: samsung,hotplug-cpus (empty: no hotplug throttle).
 * @hp_off: Exynos8895: the CPUs of @hp_cpus this driver took offline.
 * @hp_out_temp: Exynos8895: samsung,hotplug-out-temp, millicelsius.
 * @hp_in_temp: Exynos8895: samsung,hotplug-in-temp, millicelsius.
 * @hp_on: Exynos8895: the hotplug throttle is on.
 * @hp_retry: Exynos8895: the last hotplug pass left a CPU in the wrong state.
 * @hp_work: Exynos8895: takes @hp_cpus offline or brings them back.
 */
struct exynos_tmu_data {
	void __iomem *base;
	void __iomem *base_second;
	int irq;
	enum soc_type soc;
	struct mutex lock;
	struct clk *clk, *clk_sec, *sclk;
	u32 cal_type;
	u32 efuse_value;
	u32 min_efuse_value;
	u32 max_efuse_value;
	u16 temp_error1, temp_error2;
	u8 gain;
	u8 reference_voltage;
	struct thermal_zone_device *tzd;
	bool enabled;

	void (*tmu_set_low_temp)(struct exynos_tmu_data *data, u8 temp);
	void (*tmu_set_high_temp)(struct exynos_tmu_data *data, u8 temp);
	void (*tmu_set_crit_temp)(struct exynos_tmu_data *data, u8 temp);
	void (*tmu_disable_low)(struct exynos_tmu_data *data);
	void (*tmu_disable_high)(struct exynos_tmu_data *data);
	void (*tmu_initialize)(struct platform_device *pdev);
	void (*tmu_control)(struct platform_device *pdev, bool on);
	int (*tmu_read)(struct exynos_tmu_data *data);
	void (*tmu_set_emulation)(struct exynos_tmu_data *data, int temp);
	void (*tmu_clear_irqs)(struct exynos_tmu_data *data);

	u32 sensors;
	bool balance;
	u32 num_probe;
	bool num_probe_read;
	u8 s_cal_type[EXYNOS8895_TMU_NUM_SENSORS];
	u16 s_err1[EXYNOS8895_TMU_NUM_SENSORS];
	u16 s_err2[EXYNOS8895_TMU_NUM_SENSORS];
	u32 avg_con;
	struct device *dev;
	bool ipa;
	u32 ipa_k_po;
	u32 ipa_k_pu;
	u32 ipa_k_i;
	u32 ipa_integral_max;
	u32 ipa_integral_cutoff;
	struct list_head ipa_node;
	struct cpumask hp_cpus;
	struct cpumask hp_off;
	int hp_out_temp;
	int hp_in_temp;
	bool hp_on;
	bool hp_retry;
	struct work_struct hp_work;
};

/*
 * TMU treats temperature as a mapped temperature code.
 * The temperature is converted differently depending on the calibration type.
 */
static int temp_to_code(struct exynos_tmu_data *data, u8 temp)
{
	if (data->cal_type == TYPE_ONE_POINT_TRIMMING)
		return temp + data->temp_error1 - EXYNOS_FIRST_POINT_TRIM;

	return (temp - EXYNOS_FIRST_POINT_TRIM) *
		(data->temp_error2 - data->temp_error1) /
		(EXYNOS_SECOND_POINT_TRIM - EXYNOS_FIRST_POINT_TRIM) +
		data->temp_error1;
}

/*
 * Calculate a temperature value from a temperature code.
 * The unit of the temperature is degree Celsius.
 */
static int code_to_temp(struct exynos_tmu_data *data, u16 temp_code)
{
	if (data->cal_type == TYPE_ONE_POINT_TRIMMING)
		return temp_code - data->temp_error1 + EXYNOS_FIRST_POINT_TRIM;

	return (temp_code - data->temp_error1) *
		(EXYNOS_SECOND_POINT_TRIM - EXYNOS_FIRST_POINT_TRIM) /
		(data->temp_error2 - data->temp_error1) +
		EXYNOS_FIRST_POINT_TRIM;
}

static void sanitize_temp_error(struct exynos_tmu_data *data, u32 trim_info)
{
	u16 tmu_temp_mask =
		(data->soc == SOC_ARCH_EXYNOS7) ? EXYNOS7_TMU_TEMP_MASK
						: EXYNOS_TMU_TEMP_MASK;

	data->temp_error1 = trim_info & tmu_temp_mask;
	data->temp_error2 = ((trim_info >> EXYNOS_TRIMINFO_85_SHIFT) &
				EXYNOS_TMU_TEMP_MASK);

	if (!data->temp_error1 ||
	    (data->min_efuse_value > data->temp_error1) ||
	    (data->temp_error1 > data->max_efuse_value))
		data->temp_error1 = data->efuse_value & EXYNOS_TMU_TEMP_MASK;

	if (!data->temp_error2)
		data->temp_error2 =
			(data->efuse_value >> EXYNOS_TRIMINFO_85_SHIFT) &
			EXYNOS_TMU_TEMP_MASK;
}

static int exynos_tmu_initialize(struct platform_device *pdev)
{
	struct exynos_tmu_data *data = platform_get_drvdata(pdev);
	unsigned int status;
	int ret = 0;

	mutex_lock(&data->lock);
	clk_enable(data->clk);
	if (!IS_ERR(data->clk_sec))
		clk_enable(data->clk_sec);

	/* the Exynos8895 vendor driver does not check STATUS */
	status = data->soc == SOC_ARCH_EXYNOS8895 ? 1 :
		 readb(data->base + EXYNOS_TMU_REG_STATUS);
	if (!status) {
		ret = -EBUSY;
	} else {
		data->tmu_initialize(pdev);
		data->tmu_clear_irqs(data);
	}

	if (!IS_ERR(data->clk_sec))
		clk_disable(data->clk_sec);
	clk_disable(data->clk);
	mutex_unlock(&data->lock);

	return ret;
}

static int exynos_thermal_zone_configure(struct platform_device *pdev)
{
	struct exynos_tmu_data *data = platform_get_drvdata(pdev);
	struct thermal_zone_device *tzd = data->tzd;
	int ret, temp;

	ret = thermal_zone_get_crit_temp(tzd, &temp);
	if (ret) {
		/* FIXME: Remove this special case */
		if (data->soc == SOC_ARCH_EXYNOS5433)
			return 0;

		dev_err(&pdev->dev,
			"No CRITICAL trip point defined in device tree!\n");
		return ret;
	}

	mutex_lock(&data->lock);
	clk_enable(data->clk);

	data->tmu_set_crit_temp(data, temp / MCELSIUS);

	clk_disable(data->clk);
	mutex_unlock(&data->lock);

	return 0;
}

static u32 get_con_reg(struct exynos_tmu_data *data, u32 con)
{
	if (data->soc == SOC_ARCH_EXYNOS4412 ||
	    data->soc == SOC_ARCH_EXYNOS3250)
		con |= (EXYNOS4412_MUX_ADDR_VALUE << EXYNOS4412_MUX_ADDR_SHIFT);

	con &= ~(EXYNOS_TMU_REF_VOLTAGE_MASK << EXYNOS_TMU_REF_VOLTAGE_SHIFT);
	con |= data->reference_voltage << EXYNOS_TMU_REF_VOLTAGE_SHIFT;

	con &= ~(EXYNOS_TMU_BUF_SLOPE_SEL_MASK << EXYNOS_TMU_BUF_SLOPE_SEL_SHIFT);
	con |= (data->gain << EXYNOS_TMU_BUF_SLOPE_SEL_SHIFT);

	con &= ~(EXYNOS_TMU_TRIP_MODE_MASK << EXYNOS_TMU_TRIP_MODE_SHIFT);
	con |= (EXYNOS_NOISE_CANCEL_MODE << EXYNOS_TMU_TRIP_MODE_SHIFT);

	return con;
}

static void exynos_tmu_control(struct platform_device *pdev, bool on)
{
	struct exynos_tmu_data *data = platform_get_drvdata(pdev);

	mutex_lock(&data->lock);
	clk_enable(data->clk);
	data->tmu_control(pdev, on);
	data->enabled = on;
	clk_disable(data->clk);
	mutex_unlock(&data->lock);
}

static void exynos_tmu_update_bit(struct exynos_tmu_data *data, int reg_off,
				  int bit_off, bool enable)
{
	u32 interrupt_en;

	interrupt_en = readl(data->base + reg_off);
	if (enable)
		interrupt_en |= BIT(bit_off);
	else
		interrupt_en &= ~BIT(bit_off);
	writel(interrupt_en, data->base + reg_off);
}

static void exynos_tmu_update_temp(struct exynos_tmu_data *data, int reg_off,
				   int bit_off, u8 temp)
{
	u16 tmu_temp_mask;
	u32 th;

	tmu_temp_mask =
		(data->soc == SOC_ARCH_EXYNOS7) ? EXYNOS7_TMU_TEMP_MASK
						: EXYNOS_TMU_TEMP_MASK;

	th = readl(data->base + reg_off);
	th &= ~(tmu_temp_mask << bit_off);
	th |= temp_to_code(data, temp) << bit_off;
	writel(th, data->base + reg_off);
}

static void exynos4210_tmu_set_low_temp(struct exynos_tmu_data *data, u8 temp)
{
	/*
	 * Failing thresholds are not supported on Exynos 4210.
	 * We use polling instead.
	 */
}

static void exynos4210_tmu_set_high_temp(struct exynos_tmu_data *data, u8 temp)
{
	temp = temp_to_code(data, temp);
	writeb(temp, data->base + EXYNOS4210_TMU_REG_TRIG_LEVEL0 + 4);
	exynos_tmu_update_bit(data, EXYNOS_TMU_REG_INTEN,
			      EXYNOS_TMU_INTEN_RISE0_SHIFT + 4, true);
}

static void exynos4210_tmu_disable_low(struct exynos_tmu_data *data)
{
	/* Again, this is handled by polling. */
}

static void exynos4210_tmu_disable_high(struct exynos_tmu_data *data)
{
	exynos_tmu_update_bit(data, EXYNOS_TMU_REG_INTEN,
			      EXYNOS_TMU_INTEN_RISE0_SHIFT + 4, false);
}

static void exynos4210_tmu_set_crit_temp(struct exynos_tmu_data *data, u8 temp)
{
	/*
	 * Hardware critical temperature handling is not supported on Exynos 4210.
	 * We still set the critical temperature threshold, but this is only to
	 * make sure it is handled as soon as possible. It is just a normal interrupt.
	 */

	temp = temp_to_code(data, temp);
	writeb(temp, data->base + EXYNOS4210_TMU_REG_TRIG_LEVEL0 + 12);
	exynos_tmu_update_bit(data, EXYNOS_TMU_REG_INTEN,
			      EXYNOS_TMU_INTEN_RISE0_SHIFT + 12, true);
}

static void exynos4210_tmu_initialize(struct platform_device *pdev)
{
	struct exynos_tmu_data *data = platform_get_drvdata(pdev);

	sanitize_temp_error(data, readl(data->base + EXYNOS_TMU_REG_TRIMINFO));

	writeb(0, data->base + EXYNOS4210_TMU_REG_THRESHOLD_TEMP);
}

static void exynos4412_tmu_set_low_temp(struct exynos_tmu_data *data, u8 temp)
{
	exynos_tmu_update_temp(data, EXYNOS_THD_TEMP_FALL, 0, temp);
	exynos_tmu_update_bit(data, EXYNOS_TMU_REG_INTEN,
			      EXYNOS_TMU_INTEN_FALL0_SHIFT, true);
}

static void exynos4412_tmu_set_high_temp(struct exynos_tmu_data *data, u8 temp)
{
	exynos_tmu_update_temp(data, EXYNOS_THD_TEMP_RISE, 8, temp);
	exynos_tmu_update_bit(data, EXYNOS_TMU_REG_INTEN,
			      EXYNOS_TMU_INTEN_RISE0_SHIFT + 4, true);
}

static void exynos4412_tmu_disable_low(struct exynos_tmu_data *data)
{
	exynos_tmu_update_bit(data, EXYNOS_TMU_REG_INTEN,
			      EXYNOS_TMU_INTEN_FALL0_SHIFT, false);
}

static void exynos4412_tmu_set_crit_temp(struct exynos_tmu_data *data, u8 temp)
{
	exynos_tmu_update_temp(data, EXYNOS_THD_TEMP_RISE, 24, temp);
	exynos_tmu_update_bit(data, EXYNOS_TMU_REG_CONTROL,
			      EXYNOS_TMU_THERM_TRIP_EN_SHIFT, true);
}

static void exynos4412_tmu_initialize(struct platform_device *pdev)
{
	struct exynos_tmu_data *data = platform_get_drvdata(pdev);
	unsigned int trim_info, ctrl;

	if (data->soc == SOC_ARCH_EXYNOS3250 ||
	    data->soc == SOC_ARCH_EXYNOS4412 ||
	    data->soc == SOC_ARCH_EXYNOS5250) {
		if (data->soc == SOC_ARCH_EXYNOS3250) {
			ctrl = readl(data->base + EXYNOS_TMU_TRIMINFO_CON1);
			ctrl |= EXYNOS_TRIMINFO_RELOAD_ENABLE;
			writel(ctrl, data->base + EXYNOS_TMU_TRIMINFO_CON1);
		}
		ctrl = readl(data->base + EXYNOS_TMU_TRIMINFO_CON2);
		ctrl |= EXYNOS_TRIMINFO_RELOAD_ENABLE;
		writel(ctrl, data->base + EXYNOS_TMU_TRIMINFO_CON2);
	}

	/* On exynos5420 the triminfo register is in the shared space */
	if (data->soc == SOC_ARCH_EXYNOS5420_TRIMINFO)
		trim_info = readl(data->base_second + EXYNOS_TMU_REG_TRIMINFO);
	else
		trim_info = readl(data->base + EXYNOS_TMU_REG_TRIMINFO);

	sanitize_temp_error(data, trim_info);
}

static void exynos5433_tmu_set_low_temp(struct exynos_tmu_data *data, u8 temp)
{
	exynos_tmu_update_temp(data, EXYNOS5433_THD_TEMP_FALL3_0, 0, temp);
	exynos_tmu_update_bit(data, EXYNOS5433_TMU_REG_INTEN,
			      EXYNOS_TMU_INTEN_FALL0_SHIFT, true);
}

static void exynos5433_tmu_set_high_temp(struct exynos_tmu_data *data, u8 temp)
{
	exynos_tmu_update_temp(data, EXYNOS5433_THD_TEMP_RISE3_0, 8, temp);
	exynos_tmu_update_bit(data, EXYNOS5433_TMU_REG_INTEN,
			      EXYNOS7_TMU_INTEN_RISE0_SHIFT + 1, true);
}

static void exynos5433_tmu_disable_low(struct exynos_tmu_data *data)
{
	exynos_tmu_update_bit(data, EXYNOS5433_TMU_REG_INTEN,
			      EXYNOS_TMU_INTEN_FALL0_SHIFT, false);
}

static void exynos5433_tmu_disable_high(struct exynos_tmu_data *data)
{
	exynos_tmu_update_bit(data, EXYNOS5433_TMU_REG_INTEN,
			      EXYNOS7_TMU_INTEN_RISE0_SHIFT + 1, false);
}

static void exynos5433_tmu_set_crit_temp(struct exynos_tmu_data *data, u8 temp)
{
	exynos_tmu_update_temp(data, EXYNOS5433_THD_TEMP_RISE7_4, 24, temp);
	exynos_tmu_update_bit(data, EXYNOS_TMU_REG_CONTROL,
			      EXYNOS_TMU_THERM_TRIP_EN_SHIFT, true);
	exynos_tmu_update_bit(data, EXYNOS5433_TMU_REG_INTEN,
			      EXYNOS7_TMU_INTEN_RISE0_SHIFT + 7, true);
}

static void exynos5433_tmu_initialize(struct platform_device *pdev)
{
	struct exynos_tmu_data *data = platform_get_drvdata(pdev);
	unsigned int trim_info;
	int sensor_id, cal_type;

	trim_info = readl(data->base + EXYNOS_TMU_REG_TRIMINFO);
	sanitize_temp_error(data, trim_info);

	/* Read the temperature sensor id */
	sensor_id = (trim_info & EXYNOS5433_TRIMINFO_SENSOR_ID_MASK)
				>> EXYNOS5433_TRIMINFO_SENSOR_ID_SHIFT;
	dev_info(&pdev->dev, "Temperature sensor ID: 0x%x\n", sensor_id);

	/* Read the calibration mode */
	writel(trim_info, data->base + EXYNOS_TMU_REG_TRIMINFO);
	cal_type = (trim_info & EXYNOS5433_TRIMINFO_CALIB_SEL_MASK)
				>> EXYNOS5433_TRIMINFO_CALIB_SEL_SHIFT;

	switch (cal_type) {
	case EXYNOS5433_TRIMINFO_TWO_POINT_TRIMMING:
		data->cal_type = TYPE_TWO_POINT_TRIMMING;
		break;
	case EXYNOS5433_TRIMINFO_ONE_POINT_TRIMMING:
	default:
		data->cal_type = TYPE_ONE_POINT_TRIMMING;
		break;
	}

	dev_info(&pdev->dev, "Calibration type is %d-point calibration\n",
			cal_type ?  2 : 1);
}

static void exynos7_tmu_set_low_temp(struct exynos_tmu_data *data, u8 temp)
{
	exynos_tmu_update_temp(data, EXYNOS7_THD_TEMP_FALL7_6 + 12, 0, temp);
	exynos_tmu_update_bit(data, EXYNOS7_TMU_REG_INTEN,
			      EXYNOS_TMU_INTEN_FALL0_SHIFT + 0, true);
}

static void exynos7_tmu_set_high_temp(struct exynos_tmu_data *data, u8 temp)
{
	exynos_tmu_update_temp(data, EXYNOS7_THD_TEMP_RISE7_6 + 12, 16, temp);
	exynos_tmu_update_bit(data, EXYNOS7_TMU_REG_INTEN,
			      EXYNOS7_TMU_INTEN_RISE0_SHIFT + 1, true);
}

static void exynos7_tmu_disable_low(struct exynos_tmu_data *data)
{
	exynos_tmu_update_bit(data, EXYNOS7_TMU_REG_INTEN,
			      EXYNOS_TMU_INTEN_FALL0_SHIFT + 0, false);
}

static void exynos7_tmu_disable_high(struct exynos_tmu_data *data)
{
	exynos_tmu_update_bit(data, EXYNOS7_TMU_REG_INTEN,
			      EXYNOS7_TMU_INTEN_RISE0_SHIFT + 1, false);
}

static void exynos7_tmu_set_crit_temp(struct exynos_tmu_data *data, u8 temp)
{
	/*
	 * Like Exynos 4210, Exynos 7 does not seem to support critical temperature
	 * handling in hardware. Again, we still set a separate interrupt for it.
	 */
	exynos_tmu_update_temp(data, EXYNOS7_THD_TEMP_RISE7_6 + 0, 16, temp);
	exynos_tmu_update_bit(data, EXYNOS7_TMU_REG_INTEN,
			      EXYNOS7_TMU_INTEN_RISE0_SHIFT + 7, true);
}

static void exynos7_tmu_initialize(struct platform_device *pdev)
{
	struct exynos_tmu_data *data = platform_get_drvdata(pdev);
	unsigned int trim_info;

	trim_info = readl(data->base + EXYNOS_TMU_REG_TRIMINFO);
	sanitize_temp_error(data, trim_info);
}

/* offset of the threshold registers of sensor n from those of sensor 0 */
static u32 exynos8895_thd_base(unsigned int n)
{
	return n ? EXYNOS8895_THD_TEMP_R_OFFSET + EXYNOS8895_THD_TEMP_R_STRIDE * n : 0;
}

static u32 exynos8895_inten(unsigned int n)
{
	return n < 5 ? EXYNOS8895_TMU_REG_INTEN0 + EXYNOS8895_TMU_REG_INT_STRIDE * n :
		       EXYNOS8895_TMU_REG_INTEN5 + EXYNOS8895_TMU_REG_INT_STRIDE * (n - 5);
}

static u32 exynos8895_intpend(unsigned int n)
{
	return n < 5 ? EXYNOS8895_TMU_REG_INTPEND0 + EXYNOS8895_TMU_REG_INT_STRIDE * n :
		       EXYNOS8895_TMU_REG_INTPEND5 + EXYNOS8895_TMU_REG_INT_STRIDE * (n - 5);
}

/* temp_to_code() with the trims of sensor n */
static int exynos8895_temp_to_code(struct exynos_tmu_data *data,
				   unsigned int n, int temp)
{
	temp = clamp(temp, EXYNOS8895_TMU_MIN_TEMP, EXYNOS8895_TMU_MAX_TEMP);

	if (data->s_cal_type[n] == TYPE_TWO_POINT_TRIMMING)
		return (temp - EXYNOS_FIRST_POINT_TRIM) *
			(data->s_err2[n] - data->s_err1[n]) /
			(EXYNOS_SECOND_POINT_TRIM - EXYNOS_FIRST_POINT_TRIM) +
			data->s_err1[n];

	return temp + data->s_err1[n] - EXYNOS_FIRST_POINT_TRIM;
}

/* code_to_temp() with the trims of sensor n */
static int exynos8895_code_to_temp(struct exynos_tmu_data *data,
				   unsigned int n, int code)
{
	int temp;

	if (data->s_cal_type[n] == TYPE_TWO_POINT_TRIMMING)
		temp = (code - data->s_err1[n]) *
			(EXYNOS_SECOND_POINT_TRIM - EXYNOS_FIRST_POINT_TRIM) /
			(data->s_err2[n] - data->s_err1[n]) +
			EXYNOS_FIRST_POINT_TRIM;
	else
		temp = code - data->s_err1[n] + EXYNOS_FIRST_POINT_TRIM;

	return clamp(temp, EXYNOS8895_TMU_MIN_TEMP, EXYNOS8895_TMU_MAX_TEMP);
}

/* Write one threshold field of every sensor of the instance. */
static void exynos8895_tmu_set_thd(struct exynos_tmu_data *data, u32 reg,
				   unsigned int shift, int temp)
{
	unsigned int n;
	u32 v;

	for (n = 0; n < EXYNOS8895_TMU_NUM_SENSORS; n++) {
		if (!(data->sensors & BIT(n)))
			continue;
		v = readl(data->base + reg + exynos8895_thd_base(n));
		v &= ~(EXYNOS8895_TMU_TEMP_MASK << shift);
		v |= (exynos8895_temp_to_code(data, n, temp) &
		      EXYNOS8895_TMU_TEMP_MASK) << shift;
		writel(v, data->base + reg + exynos8895_thd_base(n));
	}
}

static void exynos8895_tmu_set_inten(struct exynos_tmu_data *data,
				     unsigned int bit, bool enable)
{
	unsigned int n;

	for (n = 0; n < EXYNOS8895_TMU_NUM_SENSORS; n++)
		if (data->sensors & BIT(n))
			exynos_tmu_update_bit(data, exynos8895_inten(n), bit, enable);
}

/* Level 7 holds the critical trip, level 6 the set_trips window. */
static void exynos8895_tmu_set_crit_temp(struct exynos_tmu_data *data, u8 temp)
{
	exynos8895_tmu_set_thd(data, EXYNOS8895_THD_TEMP_RISE7_6, 16, temp);
	exynos8895_tmu_set_inten(data, EXYNOS8895_TMU_INTEN_RISE7, true);
}

static void exynos8895_tmu_set_high_temp(struct exynos_tmu_data *data, u8 temp)
{
	exynos8895_tmu_set_thd(data, EXYNOS8895_THD_TEMP_RISE7_6, 0, temp);
	exynos8895_tmu_set_inten(data, EXYNOS8895_TMU_INTEN_RISE6, true);
}

static void exynos8895_tmu_set_low_temp(struct exynos_tmu_data *data, u8 temp)
{
	exynos8895_tmu_set_thd(data, EXYNOS8895_THD_TEMP_FALL7_6, 0, temp);
	exynos8895_tmu_set_inten(data, EXYNOS_TMU_INTEN_FALL0_SHIFT +
				 EXYNOS8895_TMU_INTEN_RISE6, true);
}

static void exynos8895_tmu_disable_high(struct exynos_tmu_data *data)
{
	exynos8895_tmu_set_inten(data, EXYNOS8895_TMU_INTEN_RISE6, false);
}

static void exynos8895_tmu_disable_low(struct exynos_tmu_data *data)
{
	exynos8895_tmu_set_inten(data, EXYNOS_TMU_INTEN_FALL0_SHIFT +
				 EXYNOS8895_TMU_INTEN_RISE6, false);
}

/*
 * Unlike the vendor driver, this does not program the trip thresholds:
 * set_trips and the critical trip setup own them.
 */
static void exynos8895_tmu_initialize(struct platform_device *pdev)
{
	struct exynos_tmu_data *data = platform_get_drvdata(pdev);
	u32 trim, cal, avg, n;

	if (!data->num_probe_read) {
		/* keep the value found at probe, control writes it back */
		data->num_probe = (readl(data->base + EXYNOS8895_TMU_REG_CONTROL1) >>
				   EXYNOS8895_TMU_NUM_PROBE_SHIFT) &
				  EXYNOS8895_TMU_NUM_PROBE_MASK;
		data->num_probe_read = true;
	}

	cal = (readl(data->base + EXYNOS_TMU_REG_TRIMINFO) >>
	       EXYNOS8895_TMU_CALIB_SEL_SHIFT) & 0x1;

	for (n = 0; n < EXYNOS8895_TMU_NUM_SENSORS; n++) {
		if (!(data->sensors & BIT(n)))
			continue;

		trim = readl(data->base + EXYNOS_TMU_REG_TRIMINFO + 4 * n);
		data->s_cal_type[n] = cal ? TYPE_TWO_POINT_TRIMMING :
					    TYPE_ONE_POINT_TRIMMING;
		data->s_err1[n] = trim & EXYNOS8895_TMU_TEMP_MASK;
		if (!data->s_err1[n])
			data->s_err1[n] = data->efuse_value & EXYNOS8895_TMU_TEMP_MASK;
		data->s_err2[n] = (trim >> EXYNOS8895_TMU_TEMP_SHIFT) &
				  EXYNOS8895_TMU_TEMP_MASK;
		if (!data->s_err2[n])
			data->s_err2[n] = (data->efuse_value >> EXYNOS8895_TMU_TEMP_SHIFT) &
					  EXYNOS8895_TMU_TEMP_MASK;

		/* a two-point sensor with err2 <= err1 would divide by <= 0 */
		if (data->s_cal_type[n] == TYPE_TWO_POINT_TRIMMING &&
		    data->s_err2[n] <= data->s_err1[n]) {
			dev_warn(&pdev->dev, "sensor %u: err2 %u <= err1 %u, using one-point trimming\n",
				 n, data->s_err2[n], data->s_err1[n]);
			data->s_cal_type[n] = TYPE_ONE_POINT_TRIMMING;
		}

		dev_dbg(&pdev->dev, "sensor %u: %s-point trim, err1 %u, err2 %u\n", n,
			data->s_cal_type[n] == TYPE_TWO_POINT_TRIMMING ? "two" : "one",
			data->s_err1[n], data->s_err2[n]);
	}

	avg = readl(data->base + EXYNOS8895_TMU_REG_AVG_CON);
	if ((readl(data->base + EXYNOS8895_TMU_REG_TRIMINFO2) >>
	     EXYNOS8895_TMU_AVG_CON_SHIFT) & EXYNOS8895_TMU_AVG_CON_MASK) {
		avg |= (avg & EXYNOS8895_TMU_AVG_MODE_MASK) | EXYNOS8895_TMU_AVG_MODE_DEFAULT;
		avg &= ~BIT(EXYNOS8895_TMU_DEM_SHIFT);
	} else {
		avg |= (avg & EXYNOS8895_TMU_AVG_MODE_MASK) | EXYNOS8895_TMU_AVG_MODE_4;
		avg |= BIT(EXYNOS8895_TMU_DEM_SHIFT);
	}
	data->avg_con = avg;

	dev_dbg(&pdev->dev, "sensors 0x%x, %s mode, AVG_CON 0x%x\n",
		data->sensors, data->balance ? "balance" : "max", avg);
}

/*
 * The reference voltage and buffer slope come from the fuses. The interrupt
 * enables belong to set_trips and are left alone.
 */
static void exynos8895_tmu_control(struct platform_device *pdev, bool on)
{
	struct exynos_tmu_data *data = platform_get_drvdata(pdev);
	u32 con, con1;

	con = readl(data->base + EXYNOS_TMU_REG_CONTROL);
	con &= ~(BIT(EXYNOS_TMU_CORE_EN_SHIFT) | BIT(EXYNOS_TMU_THERM_TRIP_EN_SHIFT));
	writel(con, data->base + EXYNOS_TMU_REG_CONTROL);

	con1 = readl(data->base + EXYNOS8895_TMU_REG_CONTROL1);
	con1 &= ~(EXYNOS8895_TMU_NUM_PROBE_MASK << EXYNOS8895_TMU_NUM_PROBE_SHIFT);
	con1 |= data->num_probe << EXYNOS8895_TMU_NUM_PROBE_SHIFT;
	writel(con1, data->base + EXYNOS8895_TMU_REG_CONTROL1);

	con = get_con_reg(data, readl(data->base + EXYNOS_TMU_REG_CONTROL));

	if (on) {
		con |= ((readl(data->base + EXYNOS_TMU_REG_TRIMINFO) >>
			 EXYNOS8895_TMU_T_BUF_VREF_SEL_SHIFT) &
			EXYNOS8895_TMU_T_BUF_VREF_SEL_MASK) << EXYNOS_TMU_REF_VOLTAGE_SHIFT;
		con |= ((readl(data->base + EXYNOS8895_TMU_REG_TRIMINFO1) >>
			 EXYNOS8895_TMU_T_BUF_SLOPE_SEL_SHIFT) &
			EXYNOS8895_TMU_T_BUF_SLOPE_SEL_MASK) << EXYNOS_TMU_BUF_SLOPE_SEL_SHIFT;
		con |= BIT(EXYNOS_TMU_CORE_EN_SHIFT) |
		       BIT(EXYNOS_TMU_THERM_TRIP_EN_SHIFT);
	} else {
		con &= ~(BIT(EXYNOS_TMU_CORE_EN_SHIFT) |
			 BIT(EXYNOS_TMU_THERM_TRIP_EN_SHIFT));
	}

	writel(con, data->base + EXYNOS_TMU_REG_CONTROL);
	writel(data->avg_con, data->base + EXYNOS8895_TMU_REG_AVG_CON);
}

static int exynos8895_sensor_temp(struct exynos_tmu_data *data, unsigned int n)
{
	u32 reg, shift, code;

	if (n < 2) {
		reg = 0;
		shift = EXYNOS8895_TMU_TEMP_SHIFT * n;
	} else {
		reg = ((n - 2) / 3 + 1) * 4;
		shift = EXYNOS8895_TMU_TEMP_SHIFT * ((n - 2) % 3);
	}
	code = (readl(data->base + EXYNOS8895_TMU_REG_CURRENT_TEMP1_0 + reg) >>
		shift) & EXYNOS8895_TMU_TEMP_MASK;

	return exynos8895_code_to_temp(data, n, code);
}

/*
 * Returns degrees Celsius: the highest sensor reading, in "balance" mode after
 * lowering every sensor but sensor 0 by EXYNOS8895_TMU_BALANCE_OFFSET.
 */
static int exynos8895_tmu_read(struct exynos_tmu_data *data)
{
	int result = 0, temp;
	u32 n;

	for (n = 0; n < EXYNOS8895_TMU_NUM_SENSORS; n++) {
		if (!(data->sensors & BIT(n)))
			continue;

		temp = exynos8895_sensor_temp(data, n);

		if (data->balance && n)
			temp = temp >= EXYNOS8895_TMU_BALANCE_OFFSET ?
			       temp - EXYNOS8895_TMU_BALANCE_OFFSET : 0;
		result = max(result, temp);
	}

	return result;
}

static void exynos8895_tmu_clear_irqs(struct exynos_tmu_data *data)
{
	unsigned int n;
	u32 v;

	for (n = 0; n < EXYNOS8895_TMU_NUM_SENSORS; n++) {
		if (!(data->sensors & BIT(n)))
			continue;
		v = readl(data->base + exynos8895_intpend(n));
		writel(v, data->base + exynos8895_intpend(n));
	}
}

static void exynos4210_tmu_control(struct platform_device *pdev, bool on)
{
	struct exynos_tmu_data *data = platform_get_drvdata(pdev);
	unsigned int con;

	con = get_con_reg(data, readl(data->base + EXYNOS_TMU_REG_CONTROL));

	if (on)
		con |= BIT(EXYNOS_TMU_CORE_EN_SHIFT);
	else
		con &= ~BIT(EXYNOS_TMU_CORE_EN_SHIFT);

	writel(con, data->base + EXYNOS_TMU_REG_CONTROL);
}

static void exynos5433_tmu_control(struct platform_device *pdev, bool on)
{
	struct exynos_tmu_data *data = platform_get_drvdata(pdev);
	unsigned int con, pd_det_en;

	con = get_con_reg(data, readl(data->base + EXYNOS_TMU_REG_CONTROL));

	if (on)
		con |= BIT(EXYNOS_TMU_CORE_EN_SHIFT);
	else
		con &= ~BIT(EXYNOS_TMU_CORE_EN_SHIFT);

	pd_det_en = on ? EXYNOS5433_PD_DET_EN : 0;

	writel(pd_det_en, data->base + EXYNOS5433_TMU_PD_DET_EN);
	writel(con, data->base + EXYNOS_TMU_REG_CONTROL);
}

static void exynos7_tmu_control(struct platform_device *pdev, bool on)
{
	struct exynos_tmu_data *data = platform_get_drvdata(pdev);
	unsigned int con;

	con = get_con_reg(data, readl(data->base + EXYNOS_TMU_REG_CONTROL));

	if (on) {
		con |= BIT(EXYNOS_TMU_CORE_EN_SHIFT);
		con |= BIT(EXYNOS7_PD_DET_EN_SHIFT);
	} else {
		con &= ~BIT(EXYNOS_TMU_CORE_EN_SHIFT);
		con &= ~BIT(EXYNOS7_PD_DET_EN_SHIFT);
	}

	writel(con, data->base + EXYNOS_TMU_REG_CONTROL);
}

/*
 * The vendor kernel caps the number of online CPUs through a PM QoS class
 * that does not exist here, so the CPUs are taken offline directly.
 */
static void exynos8895_hotplug_work(struct work_struct *work)
{
	struct exynos_tmu_data *data = container_of(work, struct exynos_tmu_data,
						    hp_work);
	bool on = READ_ONCE(data->hp_on);
	bool retry = false;
	unsigned int cpu;
	int ret;

	for_each_cpu(cpu, &data->hp_cpus) {
		if (on) {
			if (cpumask_test_cpu(cpu, &data->hp_off))
				continue;
			ret = remove_cpu(cpu);
			/* > 0: already offline, not this driver's to bring back */
			if (!ret)
				cpumask_set_cpu(cpu, &data->hp_off);
		} else {
			if (!cpumask_test_cpu(cpu, &data->hp_off))
				continue;
			ret = add_cpu(cpu);
			/* > 0: already online */
			if (ret >= 0)
				cpumask_clear_cpu(cpu, &data->hp_off);
		}
		if (ret < 0) {
			retry = true;
			dev_warn_ratelimited(data->dev, "cpu%u %s failed: %d\n", cpu,
					     on ? "offline" : "online", ret);
		}
	}

	WRITE_ONCE(data->hp_retry, retry);
}

/* Temperature reads, and so these calls, are serialized by the zone lock. */
static void exynos8895_hotplug_check(struct exynos_tmu_data *data, int temp)
{
	bool on = READ_ONCE(data->hp_on);
	bool change = on ? temp < data->hp_in_temp : temp >= data->hp_out_temp;

	if (change) {
		on = !on;
		WRITE_ONCE(data->hp_on, on);
		dev_dbg(data->dev, "hotplug throttle %s at %d mC\n",
			on ? "on" : "off", temp);
	}

	/* a failed pass is retried at the next read */
	if (change || READ_ONCE(data->hp_retry))
		queue_work(system_unbound_wq, &data->hp_work);
}

static void exynos8895_hotplug_stop(void *arg)
{
	struct exynos_tmu_data *data = arg;
	unsigned int cpu;

	cancel_work_sync(&data->hp_work);
	for_each_cpu(cpu, &data->hp_off)
		add_cpu(cpu);
	cpumask_clear(&data->hp_off);
}

static int exynos8895_hotplug_parse(struct platform_device *pdev,
				    struct exynos_tmu_data *data)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *cpu_np;
	u32 out_temp, in_temp;
	int i, n, cpu;

	n = of_count_phandle_with_args(np, "samsung,hotplug-cpus", NULL);
	if (n <= 0 ||
	    of_property_read_u32(np, "samsung,hotplug-out-temp", &out_temp) ||
	    of_property_read_u32(np, "samsung,hotplug-in-temp", &in_temp)) {
		dev_err(&pdev->dev, "incomplete samsung,hotplug-* properties\n");
		return -EINVAL;
	}
	data->hp_out_temp = out_temp;
	data->hp_in_temp = in_temp;

	for (i = 0; i < n; i++) {
		cpu_np = of_parse_phandle(np, "samsung,hotplug-cpus", i);
		cpu = of_cpu_node_to_id(cpu_np);
		of_node_put(cpu_np);
		if (cpu < 0) {
			dev_err(&pdev->dev, "samsung,hotplug-cpus[%d] is not a CPU\n", i);
			return -EINVAL;
		}
		cpumask_set_cpu(cpu, &data->hp_cpus);
	}

	return 0;
}

/*
 * Exynos8895 boot QoS: the vendor cpufreq driver holds each cluster at a boot
 * level for a while after init, lowered when sensor 0 already reads hot.
 */
#define EXYNOS8895_BOOT_QOS_MAX_CL	2

struct exynos8895_boot_qos;

struct exynos8895_boot_qos_cl {
	struct exynos8895_boot_qos *bq;
	int cpu;
	bool used;
	bool active;
	struct freq_qos_request min_req;
	struct freq_qos_request max_req;
	struct delayed_work work;
};

struct exynos8895_boot_qos {
	struct exynos_tmu_data *data;
	struct notifier_block nb;
	struct mutex lock;	/* the requests of every cluster */
	unsigned long delay;
	unsigned int nr_cl;
	unsigned int nr_rows;
	const u32 *rows;	/* nr_rows x (1 + nr_cl): millicelsius, kHz per cluster */
	struct exynos8895_boot_qos_cl cl[EXYNOS8895_BOOT_QOS_MAX_CL];
};

static void exynos8895_boot_qos_drop(struct exynos8895_boot_qos_cl *cl)
{
	lockdep_assert_held(&cl->bq->lock);

	if (!cl->active)
		return;
	freq_qos_remove_request(&cl->max_req);
	freq_qos_remove_request(&cl->min_req);
	cl->active = false;
}

static void exynos8895_boot_qos_expire(struct work_struct *work)
{
	struct exynos8895_boot_qos_cl *cl =
		container_of(to_delayed_work(work), struct exynos8895_boot_qos_cl, work);

	guard(mutex)(&cl->bq->lock);
	if (cl->active)
		dev_dbg(cl->bq->data->dev, "cpu%d boot QoS released\n", cl->cpu);
	exynos8895_boot_qos_drop(cl);
}

/* the boot level of cluster @i in kHz, 0 for the cluster minimum, -1 for no limit */
static long exynos8895_boot_qos_limit(struct exynos8895_boot_qos *bq,
				      unsigned int i, int *temp)
{
	struct exynos_tmu_data *data = bq->data;
	unsigned int row, cols = 1 + bq->nr_cl;

	mutex_lock(&data->lock);
	clk_enable(data->clk);
	*temp = exynos8895_sensor_temp(data, 0) * MCELSIUS;
	clk_disable(data->clk);
	mutex_unlock(&data->lock);

	for (row = 0; row < bq->nr_rows; row++)
		if (*temp > (int)bq->rows[row * cols])
			return bq->rows[row * cols + 1 + i];

	return -1;
}

static void exynos8895_boot_qos_apply(struct exynos8895_boot_qos *bq,
				      struct cpufreq_policy *policy)
{
	unsigned int i, khz;
	long limit;
	int temp, ret;

	guard(mutex)(&bq->lock);
	for (i = 0; i < bq->nr_cl; i++) {
		struct exynos8895_boot_qos_cl *cl = &bq->cl[i];

		if (cl->used || !cpumask_test_cpu(cl->cpu, policy->related_cpus))
			continue;
		cl->used = true;

		khz = policy->cpuinfo.max_freq;
		limit = exynos8895_boot_qos_limit(bq, i, &temp);
		if (limit >= 0)
			khz = min_t(unsigned int, khz,
				    max_t(unsigned int, limit, policy->cpuinfo.min_freq));

		ret = freq_qos_add_request(&policy->constraints, &cl->min_req,
					   FREQ_QOS_MIN, khz);
		if (ret < 0)
			goto fail;
		ret = freq_qos_add_request(&policy->constraints, &cl->max_req,
					   FREQ_QOS_MAX, khz);
		if (ret < 0) {
			freq_qos_remove_request(&cl->min_req);
			goto fail;
		}
		cl->active = true;
		schedule_delayed_work(&cl->work, bq->delay);
		dev_dbg(bq->data->dev, "cpu%d boot QoS %u kHz for %u ms (sensor 0 at %d mC)\n",
			cl->cpu, khz, jiffies_to_msecs(bq->delay), temp);
		continue;
fail:
		dev_err(bq->data->dev, "cpu%d boot QoS not applied: %d\n", cl->cpu, ret);
	}
}

static void exynos8895_boot_qos_remove(struct exynos8895_boot_qos *bq,
				       struct cpufreq_policy *policy)
{
	unsigned int i;

	guard(mutex)(&bq->lock);
	for (i = 0; i < bq->nr_cl; i++)
		if (cpumask_test_cpu(bq->cl[i].cpu, policy->related_cpus))
			exynos8895_boot_qos_drop(&bq->cl[i]);
}

static int exynos8895_boot_qos_notify(struct notifier_block *nb,
				      unsigned long event, void *ptr)
{
	struct exynos8895_boot_qos *bq = container_of(nb, struct exynos8895_boot_qos, nb);

	if (event == CPUFREQ_CREATE_POLICY)
		exynos8895_boot_qos_apply(bq, ptr);
	else if (event == CPUFREQ_REMOVE_POLICY)
		exynos8895_boot_qos_remove(bq, ptr);

	return NOTIFY_OK;
}

static void exynos8895_boot_qos_stop(void *arg)
{
	struct exynos8895_boot_qos *bq = arg;
	unsigned int i;

	cpufreq_unregister_notifier(&bq->nb, CPUFREQ_POLICY_NOTIFIER);
	for (i = 0; i < bq->nr_cl; i++) {
		cancel_delayed_work_sync(&bq->cl[i].work);
		scoped_guard(mutex, &bq->lock)
			exynos8895_boot_qos_drop(&bq->cl[i]);
	}
}

static int exynos8895_boot_qos_init(struct platform_device *pdev,
				    struct exynos_tmu_data *data)
{
	struct device_node *np = pdev->dev.of_node;
	struct exynos8895_boot_qos *bq;
	struct cpufreq_policy *policy;
	struct device_node *cpu_np;
	u32 ms, *rows;
	int i, n, len, ret;

	n = of_count_phandle_with_args(np, "samsung,boot-qos-cpus", NULL);
	len = of_property_count_u32_elems(np, "samsung,boot-max-freq");
	if (n <= 0 || n > EXYNOS8895_BOOT_QOS_MAX_CL || len <= 0 || len % (1 + n) ||
	    of_property_read_u32(np, "samsung,boot-qos-ms", &ms) ||
	    !(data->sensors & BIT(0))) {
		dev_err(&pdev->dev, "invalid samsung,boot-* properties\n");
		return -EINVAL;
	}

	bq = devm_kzalloc(&pdev->dev, sizeof(*bq), GFP_KERNEL);
	rows = devm_kcalloc(&pdev->dev, len, sizeof(*rows), GFP_KERNEL);
	if (!bq || !rows)
		return -ENOMEM;
	ret = of_property_read_u32_array(np, "samsung,boot-max-freq", rows, len);
	if (ret)
		return ret;

	bq->data = data;
	bq->rows = rows;
	bq->nr_rows = len / (1 + n);
	bq->nr_cl = n;
	bq->delay = msecs_to_jiffies(ms);
	mutex_init(&bq->lock);
	for (i = 0; i < n; i++) {
		cpu_np = of_parse_phandle(np, "samsung,boot-qos-cpus", i);
		bq->cl[i].cpu = of_cpu_node_to_id(cpu_np);
		of_node_put(cpu_np);
		if (bq->cl[i].cpu < 0) {
			dev_err(&pdev->dev, "samsung,boot-qos-cpus[%d] is not a CPU\n", i);
			return -EINVAL;
		}
		bq->cl[i].bq = bq;
		INIT_DELAYED_WORK(&bq->cl[i].work, exynos8895_boot_qos_expire);
	}

	bq->nb.notifier_call = exynos8895_boot_qos_notify;
	ret = cpufreq_register_notifier(&bq->nb, CPUFREQ_POLICY_NOTIFIER);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(&pdev->dev, exynos8895_boot_qos_stop, bq);
	if (ret)
		return ret;

	/* policies created before the notifier was registered */
	for (i = 0; i < n; i++) {
		policy = cpufreq_cpu_get(bq->cl[i].cpu);
		if (policy) {
			exynos8895_boot_qos_apply(bq, policy);
			cpufreq_cpu_put(policy);
		}
	}

	return 0;
}

static int exynos_get_temp(struct thermal_zone_device *tz, int *temp)
{
	struct exynos_tmu_data *data = thermal_zone_device_priv(tz);
	int value, ret = 0;

	if (!data || !data->tmu_read)
		return -EINVAL;
	else if (!data->enabled)
		/*
		 * Called too early, probably
		 * from thermal_zone_of_sensor_register().
		 */
		return -EAGAIN;

	mutex_lock(&data->lock);
	clk_enable(data->clk);

	value = data->tmu_read(data);
	if (value < 0)
		ret = value;
	else if (data->soc == SOC_ARCH_EXYNOS8895)
		*temp = value * MCELSIUS;	/* already degrees Celsius */
	else
		*temp = code_to_temp(data, value) * MCELSIUS;

	clk_disable(data->clk);
	mutex_unlock(&data->lock);

	if (!ret && !cpumask_empty(&data->hp_cpus))
		exynos8895_hotplug_check(data, *temp);

	return ret;
}

#ifdef CONFIG_THERMAL_EMULATION
static u32 get_emul_con_reg(struct exynos_tmu_data *data, unsigned int val,
			    int temp)
{
	if (temp) {
		temp /= MCELSIUS;

		val &= ~(EXYNOS_EMUL_TIME_MASK << EXYNOS_EMUL_TIME_SHIFT);
		val |= (EXYNOS_EMUL_TIME << EXYNOS_EMUL_TIME_SHIFT);
		if (data->soc == SOC_ARCH_EXYNOS7) {
			val &= ~(EXYNOS7_EMUL_DATA_MASK <<
				EXYNOS7_EMUL_DATA_SHIFT);
			val |= (temp_to_code(data, temp) <<
				EXYNOS7_EMUL_DATA_SHIFT) |
				EXYNOS_EMUL_ENABLE;
		} else {
			val &= ~(EXYNOS_EMUL_DATA_MASK <<
				EXYNOS_EMUL_DATA_SHIFT);
			val |= (temp_to_code(data, temp) <<
				EXYNOS_EMUL_DATA_SHIFT) |
				EXYNOS_EMUL_ENABLE;
		}
	} else {
		val &= ~EXYNOS_EMUL_ENABLE;
	}

	return val;
}

static void exynos4412_tmu_set_emulation(struct exynos_tmu_data *data,
					 int temp)
{
	unsigned int val;
	u32 emul_con;

	if (data->soc == SOC_ARCH_EXYNOS5260)
		emul_con = EXYNOS5260_EMUL_CON;
	else if (data->soc == SOC_ARCH_EXYNOS5433)
		emul_con = EXYNOS5433_TMU_EMUL_CON;
	else if (data->soc == SOC_ARCH_EXYNOS7)
		emul_con = EXYNOS7_TMU_REG_EMUL_CON;
	else
		emul_con = EXYNOS_EMUL_CON;

	val = readl(data->base + emul_con);
	val = get_emul_con_reg(data, val, temp);
	writel(val, data->base + emul_con);
}

static int exynos_tmu_set_emulation(struct thermal_zone_device *tz, int temp)
{
	struct exynos_tmu_data *data = thermal_zone_device_priv(tz);
	int ret = -EINVAL;

	if (data->soc == SOC_ARCH_EXYNOS4210)
		goto out;

	if (temp && temp < MCELSIUS)
		goto out;

	if (!data->tmu_set_emulation)
		goto out;

	mutex_lock(&data->lock);
	clk_enable(data->clk);
	data->tmu_set_emulation(data, temp);
	clk_disable(data->clk);
	mutex_unlock(&data->lock);
	return 0;
out:
	return ret;
}
#else
#define exynos4412_tmu_set_emulation NULL
static int exynos_tmu_set_emulation(struct thermal_zone_device *tz, int temp)
	{ return -EINVAL; }
#endif /* CONFIG_THERMAL_EMULATION */

static int exynos4210_tmu_read(struct exynos_tmu_data *data)
{
	int ret = readb(data->base + EXYNOS_TMU_REG_CURRENT_TEMP);

	/* "temp_code" should range between 75 and 175 */
	return (ret < 75 || ret > 175) ? -ENODATA : ret;
}

static int exynos4412_tmu_read(struct exynos_tmu_data *data)
{
	return readb(data->base + EXYNOS_TMU_REG_CURRENT_TEMP);
}

static int exynos7_tmu_read(struct exynos_tmu_data *data)
{
	return readw(data->base + EXYNOS_TMU_REG_CURRENT_TEMP) &
		EXYNOS7_TMU_TEMP_MASK;
}

static irqreturn_t exynos_tmu_threaded_irq(int irq, void *id)
{
	struct exynos_tmu_data *data = id;

	thermal_zone_device_update(data->tzd, THERMAL_EVENT_UNSPECIFIED);

	mutex_lock(&data->lock);
	clk_enable(data->clk);

	/* TODO: take action based on particular interrupt */
	data->tmu_clear_irqs(data);

	clk_disable(data->clk);
	mutex_unlock(&data->lock);

	return IRQ_HANDLED;
}

static void exynos4210_tmu_clear_irqs(struct exynos_tmu_data *data)
{
	unsigned int val_irq;
	u32 tmu_intstat, tmu_intclear;

	if (data->soc == SOC_ARCH_EXYNOS5260) {
		tmu_intstat = EXYNOS5260_TMU_REG_INTSTAT;
		tmu_intclear = EXYNOS5260_TMU_REG_INTCLEAR;
	} else if (data->soc == SOC_ARCH_EXYNOS7) {
		tmu_intstat = EXYNOS7_TMU_REG_INTPEND;
		tmu_intclear = EXYNOS7_TMU_REG_INTPEND;
	} else if (data->soc == SOC_ARCH_EXYNOS5433) {
		tmu_intstat = EXYNOS5433_TMU_REG_INTPEND;
		tmu_intclear = EXYNOS5433_TMU_REG_INTPEND;
	} else {
		tmu_intstat = EXYNOS_TMU_REG_INTSTAT;
		tmu_intclear = EXYNOS_TMU_REG_INTCLEAR;
	}

	val_irq = readl(data->base + tmu_intstat);
	/*
	 * Clear the interrupts.  Please note that the documentation for
	 * Exynos3250, Exynos4412, Exynos5250 and Exynos5260 incorrectly
	 * states that INTCLEAR register has a different placing of bits
	 * responsible for FALL IRQs than INTSTAT register.  Exynos5420
	 * and Exynos5440 documentation is correct (Exynos4210 doesn't
	 * support FALL IRQs at all).
	 */
	writel(val_irq, data->base + tmu_intclear);
}

static const struct of_device_id exynos_tmu_match[] = {
	{
		.compatible = "samsung,exynos3250-tmu",
		.data = (const void *)SOC_ARCH_EXYNOS3250,
	}, {
		.compatible = "samsung,exynos4210-tmu",
		.data = (const void *)SOC_ARCH_EXYNOS4210,
	}, {
		.compatible = "samsung,exynos4412-tmu",
		.data = (const void *)SOC_ARCH_EXYNOS4412,
	}, {
		.compatible = "samsung,exynos5250-tmu",
		.data = (const void *)SOC_ARCH_EXYNOS5250,
	}, {
		.compatible = "samsung,exynos5260-tmu",
		.data = (const void *)SOC_ARCH_EXYNOS5260,
	}, {
		.compatible = "samsung,exynos5420-tmu",
		.data = (const void *)SOC_ARCH_EXYNOS5420,
	}, {
		.compatible = "samsung,exynos5420-tmu-ext-triminfo",
		.data = (const void *)SOC_ARCH_EXYNOS5420_TRIMINFO,
	}, {
		.compatible = "samsung,exynos5433-tmu",
		.data = (const void *)SOC_ARCH_EXYNOS5433,
	}, {
		.compatible = "samsung,exynos7-tmu",
		.data = (const void *)SOC_ARCH_EXYNOS7,
	}, {
		.compatible = "samsung,exynos8895-tmu",
		.data = (const void *)SOC_ARCH_EXYNOS8895,
	},
	{ },
};
MODULE_DEVICE_TABLE(of, exynos_tmu_match);

static int exynos_map_dt_data(struct platform_device *pdev)
{
	struct exynos_tmu_data *data = platform_get_drvdata(pdev);
	struct resource res;

	if (!data || !pdev->dev.of_node)
		return -ENODEV;

	data->irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (data->irq <= 0) {
		dev_err(&pdev->dev, "failed to get IRQ\n");
		return -ENODEV;
	}

	if (of_address_to_resource(pdev->dev.of_node, 0, &res)) {
		dev_err(&pdev->dev, "failed to get Resource 0\n");
		return -ENODEV;
	}

	data->base = devm_ioremap(&pdev->dev, res.start, resource_size(&res));
	if (!data->base) {
		dev_err(&pdev->dev, "Failed to ioremap memory\n");
		return -EADDRNOTAVAIL;
	}

	data->soc = (uintptr_t)of_device_get_match_data(&pdev->dev);

	switch (data->soc) {
	case SOC_ARCH_EXYNOS4210:
		data->tmu_set_low_temp = exynos4210_tmu_set_low_temp;
		data->tmu_set_high_temp = exynos4210_tmu_set_high_temp;
		data->tmu_disable_low = exynos4210_tmu_disable_low;
		data->tmu_disable_high = exynos4210_tmu_disable_high;
		data->tmu_set_crit_temp = exynos4210_tmu_set_crit_temp;
		data->tmu_initialize = exynos4210_tmu_initialize;
		data->tmu_control = exynos4210_tmu_control;
		data->tmu_read = exynos4210_tmu_read;
		data->tmu_clear_irqs = exynos4210_tmu_clear_irqs;
		data->gain = 15;
		data->reference_voltage = 7;
		data->efuse_value = 55;
		data->min_efuse_value = 40;
		data->max_efuse_value = 100;
		break;
	case SOC_ARCH_EXYNOS3250:
	case SOC_ARCH_EXYNOS4412:
	case SOC_ARCH_EXYNOS5250:
	case SOC_ARCH_EXYNOS5260:
	case SOC_ARCH_EXYNOS5420:
	case SOC_ARCH_EXYNOS5420_TRIMINFO:
		data->tmu_set_low_temp = exynos4412_tmu_set_low_temp;
		data->tmu_set_high_temp = exynos4412_tmu_set_high_temp;
		data->tmu_disable_low = exynos4412_tmu_disable_low;
		data->tmu_disable_high = exynos4210_tmu_disable_high;
		data->tmu_set_crit_temp = exynos4412_tmu_set_crit_temp;
		data->tmu_initialize = exynos4412_tmu_initialize;
		data->tmu_control = exynos4210_tmu_control;
		data->tmu_read = exynos4412_tmu_read;
		data->tmu_set_emulation = exynos4412_tmu_set_emulation;
		data->tmu_clear_irqs = exynos4210_tmu_clear_irqs;
		data->gain = 8;
		data->reference_voltage = 16;
		data->efuse_value = 55;
		if (data->soc != SOC_ARCH_EXYNOS5420 &&
		    data->soc != SOC_ARCH_EXYNOS5420_TRIMINFO)
			data->min_efuse_value = 40;
		else
			data->min_efuse_value = 0;
		data->max_efuse_value = 100;
		break;
	case SOC_ARCH_EXYNOS5433:
		data->tmu_set_low_temp = exynos5433_tmu_set_low_temp;
		data->tmu_set_high_temp = exynos5433_tmu_set_high_temp;
		data->tmu_disable_low = exynos5433_tmu_disable_low;
		data->tmu_disable_high = exynos5433_tmu_disable_high;
		data->tmu_set_crit_temp = exynos5433_tmu_set_crit_temp;
		data->tmu_initialize = exynos5433_tmu_initialize;
		data->tmu_control = exynos5433_tmu_control;
		data->tmu_read = exynos4412_tmu_read;
		data->tmu_set_emulation = exynos4412_tmu_set_emulation;
		data->tmu_clear_irqs = exynos4210_tmu_clear_irqs;
		data->gain = 8;
		if (res.start == EXYNOS5433_G3D_BASE)
			data->reference_voltage = 23;
		else
			data->reference_voltage = 16;
		data->efuse_value = 75;
		data->min_efuse_value = 40;
		data->max_efuse_value = 150;
		break;
	case SOC_ARCH_EXYNOS7:
		data->tmu_set_low_temp = exynos7_tmu_set_low_temp;
		data->tmu_set_high_temp = exynos7_tmu_set_high_temp;
		data->tmu_disable_low = exynos7_tmu_disable_low;
		data->tmu_disable_high = exynos7_tmu_disable_high;
		data->tmu_set_crit_temp = exynos7_tmu_set_crit_temp;
		data->tmu_initialize = exynos7_tmu_initialize;
		data->tmu_control = exynos7_tmu_control;
		data->tmu_read = exynos7_tmu_read;
		data->tmu_set_emulation = exynos4412_tmu_set_emulation;
		data->tmu_clear_irqs = exynos4210_tmu_clear_irqs;
		data->gain = 9;
		data->reference_voltage = 17;
		data->efuse_value = 75;
		data->min_efuse_value = 15;
		data->max_efuse_value = 100;
		break;
	case SOC_ARCH_EXYNOS8895: {
		const char *mode;

		data->tmu_set_low_temp = exynos8895_tmu_set_low_temp;
		data->tmu_set_high_temp = exynos8895_tmu_set_high_temp;
		data->tmu_disable_low = exynos8895_tmu_disable_low;
		data->tmu_disable_high = exynos8895_tmu_disable_high;
		data->tmu_set_crit_temp = exynos8895_tmu_set_crit_temp;
		data->tmu_initialize = exynos8895_tmu_initialize;
		data->tmu_control = exynos8895_tmu_control;
		data->tmu_read = exynos8895_tmu_read;
		data->tmu_clear_irqs = exynos8895_tmu_clear_irqs;
		data->gain = 0;
		data->reference_voltage = 0;
		data->efuse_value = 55;
		/* the vendor driver does not range-check the trims */
		data->min_efuse_value = 0;
		data->max_efuse_value = EXYNOS8895_TMU_TEMP_MASK;

		if (of_property_read_u32(pdev->dev.of_node, "samsung,tmu-sensors",
					 &data->sensors) ||
		    !data->sensors || data->sensors > 0xff) {
			dev_err(&pdev->dev, "missing or bad samsung,tmu-sensors\n");
			return -EINVAL;
		}
		if (of_property_read_string(pdev->dev.of_node,
					    "samsung,tmu-sensing-mode", &mode)) {
			dev_err(&pdev->dev, "missing samsung,tmu-sensing-mode\n");
			return -EINVAL;
		}
		data->balance = !strcmp(mode, "balance");

		if (of_property_present(pdev->dev.of_node, "samsung,ipa-k-po")) {
			struct device_node *np = pdev->dev.of_node;

			if (of_property_read_u32(np, "samsung,ipa-k-po", &data->ipa_k_po) ||
			    of_property_read_u32(np, "samsung,ipa-k-pu", &data->ipa_k_pu) ||
			    of_property_read_u32(np, "samsung,ipa-k-i", &data->ipa_k_i) ||
			    of_property_read_u32(np, "samsung,ipa-integral-max",
						 &data->ipa_integral_max) ||
			    of_property_read_u32(np, "samsung,ipa-integral-cutoff",
						 &data->ipa_integral_cutoff)) {
				dev_err(&pdev->dev, "incomplete samsung,ipa-* properties\n");
				return -EINVAL;
			}
			data->ipa = true;
		}

		if (of_property_present(pdev->dev.of_node, "samsung,hotplug-cpus")) {
			int ret = exynos8895_hotplug_parse(pdev, data);

			if (ret)
				return ret;
		}
		break;
	}
	default:
		dev_err(&pdev->dev, "Platform not supported\n");
		return -EINVAL;
	}

	data->cal_type = TYPE_ONE_POINT_TRIMMING;

	/*
	 * Check if the TMU shares some registers and then try to map the
	 * memory of common registers.
	 */
	if (data->soc != SOC_ARCH_EXYNOS5420_TRIMINFO)
		return 0;

	if (of_address_to_resource(pdev->dev.of_node, 1, &res)) {
		dev_err(&pdev->dev, "failed to get Resource 1\n");
		return -ENODEV;
	}

	data->base_second = devm_ioremap(&pdev->dev, res.start,
					resource_size(&res));
	if (!data->base_second) {
		dev_err(&pdev->dev, "Failed to ioremap memory\n");
		return -ENOMEM;
	}

	return 0;
}

static int exynos_set_trips(struct thermal_zone_device *tz, int low, int high)
{
	struct exynos_tmu_data *data = thermal_zone_device_priv(tz);

	mutex_lock(&data->lock);
	clk_enable(data->clk);

	if (low > INT_MIN)
		data->tmu_set_low_temp(data, low / MCELSIUS);
	else
		data->tmu_disable_low(data);
	if (high < INT_MAX)
		data->tmu_set_high_temp(data, high / MCELSIUS);
	else
		data->tmu_disable_high(data);

	clk_disable(data->clk);
	mutex_unlock(&data->lock);

	return 0;
}

static const struct thermal_zone_device_ops exynos_sensor_ops = {
	.get_temp = exynos_get_temp,
	.set_emul_temp = exynos_tmu_set_emulation,
	.set_trips = exynos_set_trips,
};

#if IS_BUILTIN(CONFIG_EXYNOS_THERMAL)
/*
 * The cooling devices register long after the TMU and power_allocator ignores
 * those that are not power actors, so a zone switches only at late init and
 * only when all of its cooling devices are power actors.
 */
static LIST_HEAD(exynos8895_ipa_list);
static DEFINE_MUTEX(exynos8895_ipa_lock);	/* the list and the flag */
static bool exynos8895_ipa_late;		/* the late selection has run */

/* FRAC_BITS of gov_power_allocator.c */
#define EXYNOS8895_IPA_FRAC_BITS	10

/* DT gains are mW per degree Celsius, the governor's Q10 mW per millidegree */
static s32 exynos8895_ipa_k(u32 k)
{
	return DIV_ROUND_CLOSEST(k << EXYNOS8895_IPA_FRAC_BITS, MCELSIUS);
}

static void exynos8895_ipa_select(struct exynos_tmu_data *data)
{
	struct thermal_zone_device *tz = data->tzd;
	const char *type = thermal_zone_device_type(tz);
	char policy[] = "power_allocator";	/* strim()ed by the core */
	char bad[THERMAL_NAME_LENGTH] = "";
	const struct thermal_trip_desc *td;
	struct thermal_instance *instance;
	int n = 0, ret;

	scoped_guard(thermal_zone, tz) {
		for_each_trip_desc(tz, td) {
			list_for_each_entry(instance, &td->thermal_instances, trip_node) {
				n++;
				if (!bad[0] && !cdev_is_power_actor(instance->cdev))
					strscpy(bad, instance->cdev->type);
			}
		}
	}

	if (!n) {
		dev_warn(data->dev, "%s: no cooling device, keeping the default governor\n",
			 type);
		return;
	}
	if (bad[0]) {
		dev_warn(data->dev, "%s: %s is not a power actor, keeping the default governor\n",
			 type, bad);
		return;
	}

	ret = thermal_zone_device_set_policy(tz, policy);
	if (ret) {
		dev_err(data->dev, "%s: power_allocator not selected: %d\n", type, ret);
		return;
	}

	/* replace the gains the governor estimated at bind */
	scoped_guard(thermal_zone, tz) {
		tz->tzp->k_po = exynos8895_ipa_k(data->ipa_k_po);
		tz->tzp->k_pu = exynos8895_ipa_k(data->ipa_k_pu);
		tz->tzp->k_i = exynos8895_ipa_k(data->ipa_k_i);
		tz->tzp->integral_cutoff = data->ipa_integral_cutoff * MCELSIUS;
		tz->tzp->integral_max = data->ipa_integral_max;
	}

	dev_dbg(data->dev,
		"%s: power_allocator k_po=%u k_pu=%u k_i=%u (mW/C) integral_max=%u mW integral_cutoff=%u C\n",
		type, data->ipa_k_po, data->ipa_k_pu, data->ipa_k_i,
		data->ipa_integral_max, data->ipa_integral_cutoff);
}

static void exynos8895_ipa_del(void *arg)
{
	struct exynos_tmu_data *data = arg;

	guard(mutex)(&exynos8895_ipa_lock);
	list_del(&data->ipa_node);
}

static int exynos8895_ipa_add(struct exynos_tmu_data *data)
{
	int ret;

	if (!data->ipa)
		return 0;

	/* list_del() of a self-linked node is harmless if the add fails */
	INIT_LIST_HEAD(&data->ipa_node);
	ret = devm_add_action_or_reset(data->dev, exynos8895_ipa_del, data);
	if (ret)
		return ret;

	guard(mutex)(&exynos8895_ipa_lock);
	list_add_tail(&data->ipa_node, &exynos8895_ipa_list);
	if (exynos8895_ipa_late)
		exynos8895_ipa_select(data);

	return 0;
}
#else
static int exynos8895_ipa_add(struct exynos_tmu_data *data)
{
	if (data->ipa)
		dev_warn(data->dev, "power_allocator selection needs a built-in driver\n");
	return 0;
}
#endif

static int exynos_tmu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct exynos_tmu_data *data;
	int ret;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	platform_set_drvdata(pdev, data);
	mutex_init(&data->lock);
	data->dev = dev;

	/*
	 * Try enabling the regulator if found
	 * TODO: Add regulator as an SOC feature, so that regulator enable
	 * is a compulsory call.
	 */
	ret = devm_regulator_get_enable_optional(dev, "vtmu");
	switch (ret) {
	case 0:
	case -ENODEV:
		break;
	case -EPROBE_DEFER:
		return -EPROBE_DEFER;
	default:
		dev_err(dev, "Failed to get enabled regulator: %d\n", ret);
		return ret;
	}

	ret = exynos_map_dt_data(pdev);
	if (ret)
		return ret;

	data->clk = devm_clk_get(dev, "tmu_apbif");
	if (IS_ERR(data->clk))
		return dev_err_probe(dev, PTR_ERR(data->clk), "Failed to get clock\n");

	data->clk_sec = devm_clk_get(dev, "tmu_triminfo_apbif");
	if (IS_ERR(data->clk_sec)) {
		if (data->soc == SOC_ARCH_EXYNOS5420_TRIMINFO)
			return dev_err_probe(dev, PTR_ERR(data->clk_sec),
					     "Failed to get triminfo clock\n");
	} else {
		ret = clk_prepare(data->clk_sec);
		if (ret) {
			dev_err(dev, "Failed to get clock\n");
			return ret;
		}
	}

	ret = clk_prepare(data->clk);
	if (ret) {
		dev_err(dev, "Failed to get clock\n");
		goto err_clk_sec;
	}

	switch (data->soc) {
	case SOC_ARCH_EXYNOS5433:
	case SOC_ARCH_EXYNOS7:
		data->sclk = devm_clk_get(dev, "tmu_sclk");
		if (IS_ERR(data->sclk)) {
			ret = dev_err_probe(dev, PTR_ERR(data->sclk), "Failed to get sclk\n");
			goto err_clk;
		} else {
			ret = clk_prepare_enable(data->sclk);
			if (ret) {
				dev_err(dev, "Failed to enable sclk\n");
				goto err_clk;
			}
		}
		break;
	default:
		break;
	}

	ret = exynos_tmu_initialize(pdev);
	if (ret) {
		dev_err(dev, "Failed to initialize TMU\n");
		goto err_sclk;
	}

	/* before the zone: its reads queue the work, its teardown cancels it */
	if (!cpumask_empty(&data->hp_cpus)) {
		INIT_WORK(&data->hp_work, exynos8895_hotplug_work);
		ret = devm_add_action_or_reset(dev, exynos8895_hotplug_stop, data);
		if (ret)
			goto err_sclk;
	}

	data->tzd = devm_thermal_of_zone_register(dev, 0, data,
						  &exynos_sensor_ops);
	if (IS_ERR(data->tzd)) {
		ret = dev_err_probe(dev, PTR_ERR(data->tzd), "Failed to register sensor\n");
		goto err_sclk;
	}

	ret = exynos_thermal_zone_configure(pdev);
	if (ret) {
		dev_err(dev, "Failed to configure the thermal zone\n");
		goto err_sclk;
	}

	/* Exynos8895 keeps the DT trigger type; two instances share each line */
	ret = devm_request_threaded_irq(dev, data->irq, NULL,
					exynos_tmu_threaded_irq,
					(data->soc == SOC_ARCH_EXYNOS8895 ?
					 0 : IRQF_TRIGGER_RISING)
						| IRQF_SHARED | IRQF_ONESHOT,
					dev_name(dev), data);
	if (ret) {
		dev_err(dev, "Failed to request irq: %d\n", data->irq);
		goto err_sclk;
	}

	exynos_tmu_control(pdev, true);

	/* not fatal: the zone works without the boot QoS */
	if (data->soc == SOC_ARCH_EXYNOS8895 &&
	    of_property_present(dev->of_node, "samsung,boot-qos-cpus") &&
	    exynos8895_boot_qos_init(pdev, data))
		dev_err(dev, "boot QoS not armed\n");

	/* not fatal: the zone stays on its default governor */
	if (data->soc == SOC_ARCH_EXYNOS8895 && exynos8895_ipa_add(data))
		dev_err(dev, "power_allocator selection not armed\n");

	return 0;

err_sclk:
	clk_disable_unprepare(data->sclk);
err_clk:
	clk_unprepare(data->clk);
err_clk_sec:
	if (!IS_ERR(data->clk_sec))
		clk_unprepare(data->clk_sec);
	return ret;
}

static void exynos_tmu_remove(struct platform_device *pdev)
{
	struct exynos_tmu_data *data = platform_get_drvdata(pdev);

	exynos_tmu_control(pdev, false);

	clk_disable_unprepare(data->sclk);
	clk_unprepare(data->clk);
	if (!IS_ERR(data->clk_sec))
		clk_unprepare(data->clk_sec);
}

#ifdef CONFIG_PM_SLEEP
static int exynos_tmu_suspend(struct device *dev)
{
	exynos_tmu_control(to_platform_device(dev), false);

	return 0;
}

static int exynos_tmu_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);

	exynos_tmu_initialize(pdev);
	exynos_tmu_control(pdev, true);

	return 0;
}

static SIMPLE_DEV_PM_OPS(exynos_tmu_pm,
			 exynos_tmu_suspend, exynos_tmu_resume);
#define EXYNOS_TMU_PM	(&exynos_tmu_pm)
#else
#define EXYNOS_TMU_PM	NULL
#endif

static struct platform_driver exynos_tmu_driver = {
	.driver = {
		.name   = "exynos-tmu",
		.pm     = EXYNOS_TMU_PM,
		.of_match_table = exynos_tmu_match,
	},
	.probe = exynos_tmu_probe,
	.remove = exynos_tmu_remove,
};

module_platform_driver(exynos_tmu_driver);

#if IS_BUILTIN(CONFIG_EXYNOS_THERMAL)
/* After late_initcall's deferred-probe flush: the cooling devices exist. */
static int __init exynos8895_ipa_late_init(void)
{
	struct exynos_tmu_data *data;

	guard(mutex)(&exynos8895_ipa_lock);
	exynos8895_ipa_late = true;
	list_for_each_entry(data, &exynos8895_ipa_list, ipa_node)
		exynos8895_ipa_select(data);

	return 0;
}
late_initcall_sync(exynos8895_ipa_late_init);
#endif

MODULE_DESCRIPTION("Exynos TMU Driver");
MODULE_AUTHOR("Donggeun Kim <dg77.kim@samsung.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:exynos-tmu");
