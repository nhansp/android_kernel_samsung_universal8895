// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  Driver for Madera CODECs on Exynos8895
 *
 *  Copyright (c) 2017 Samsung Electronics Co., Ltd.
 *
 * Transplanted from S:sound/soc/samsung/dream_madera.c (Samsung
 * N950NKSU5DZE2 OSRC 4.4.111, the ASHETON card of the SM-N950N), spec L6E
 * 5.9: the "Exynos8895-Madera" card of the Galaxy Note 8 (ABOX DMA front
 * ends, UAIF/DSIF back ends to the CS47L93 codec and the MAX98506 amplifier,
 * VTS). Not carried (Samsung-only, F-10): the earjack sysfs class
 * (jack_madera_sysfs_cb), the sec_audio_debug log and the "region1"
 * impedance-table selector.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/extcon/extcon-madera.h>
#include <linux/mfd/madera/core.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/pm_wakeup.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dpcm.h>
#include <sound/tlv.h>

#include <sound/samsung/abox.h>
#include <sound/samsung/vts.h>

#include "../codecs/madera.h"

#define MADERA_DAI_OFFSET			(13)

/* stock S:include/linux/mfd/madera/registers.h:1371-1386 (not in ACK) */
#define MADERA_GPIO3_CTRL_1			0x1704
#define MADERA_GPIO3_CTRL_2			0x1705
#define MADERA_GPIO4_CTRL_1			0x1706
#define MADERA_GPIO4_CTRL_2			0x1707
#define MADERA_GPIO9_CTRL_1			0x1710
#define MADERA_GPIO9_CTRL_2			0x1711
#define MADERA_GPIO10_CTRL_1			0x1712
#define MADERA_GPIO10_CTRL_2			0x1713

#define GPIO_AUX_FN_1 0x2280
#define GPIO_AUX_FN_2 0x1000
#define GPIO_AUX_FN_3 0x2281
#define GPIO_AUX_FN_4 0x1000

#define GPIO_HIZ_FN_1 0x2001
#define GPIO_HIZ_FN_2 0x9000

#define GPIO_PDM_FN_1 0x2000
#define GPIO_PDM_FN_2 0x9000

#define GPIO_AUXPDM_MASK_1 0x0fff
#define GPIO_AUXPDM_MASK_2 0xf000

#define SYSCLK_RATE_CHECK_PARAM 8000
#define SYSCLK_RATE_48KHZ   98304000
#define SYSCLK_RATE_441KHZ 90316800

static DECLARE_TLV_DB_SCALE(digital_tlv, -6400, 50, 0);

struct clk_conf {
	int id;
	int source;
	int rate;
};

struct exynos8895_drvdata {
	struct device *dev;
	struct snd_soc_component *codec;
	struct madera *madera;

	struct clk_conf fll1_refclk;
	struct clk_conf fll2_refclk;
	struct clk_conf sysclk;
	struct clk_conf asyncclk;
	struct clk_conf dspclk;
	struct clk_conf outclk;

	struct notifier_block nb;
	struct wakeup_source *ws;	/* stock wake_lock "madera-sound" */

	unsigned int hp_impedance_step;
	bool hiz_val;
	bool auxpdm_gpio;
	bool spkpdm_gpio;
	bool ear_mic;
	bool vts_state;
	int micbias_mode;
	unsigned int wake_lock_switch;
};

struct gain_table {
	int min;           /* Minimum impedance */
	int max;           /* Maximum impedance */
	unsigned int gain; /* Register value to set for this measurement */
};

static struct impedance_table {
	struct gain_table hp_gain_table[8];
} imp_table = {
	.hp_gain_table = {
		{    0,      13, 0 },
		{   14,      25, 5 },
		{   26,      42, 7 },
		{   43,     100, 14 },
		{  101,     200, 18 },
		{  201,     450, 20 },
		{  451,    1000, 20 },
		{ 1001, INT_MAX, 16 },
	},
};

static struct snd_soc_card exynos8895;

static struct exynos8895_drvdata exynos8895_drvdata;

static void exynos8895_uaif0_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_card *card = rtd->card;
	struct exynos8895_drvdata *drvdata = snd_soc_card_get_drvdata(card);
	struct snd_soc_dai *codec_dai = snd_soc_rtd_to_codec(rtd, 0);
	unsigned int playback_active, capture_active;
	int ret;

	playback_active = snd_soc_dai_stream_active(codec_dai,
			SNDRV_PCM_STREAM_PLAYBACK);
	capture_active = snd_soc_dai_stream_active(codec_dai,
			SNDRV_PCM_STREAM_CAPTURE);

	dev_info(card->dev, "%s\n", __func__);
	dev_info(card->dev, "codec_dai: %s\n", codec_dai->name);
	dev_info(card->dev, "%s-%d playback: %d, capture: %d, active: %d\n",
			rtd->dai_link->name, substream->stream,
			playback_active, capture_active,
			snd_soc_component_active(drvdata->codec));

	if (!playback_active && !capture_active &&
	    !snd_soc_component_active(drvdata->codec)) {
		snd_soc_component_update_bits(drvdata->codec,
				MADERA_OUTPUT_ENABLES_1,
				MADERA_OUT2L_ENA | MADERA_OUT2R_ENA, 0);
		msleep(1);

		ret = snd_soc_component_set_sysclk(drvdata->codec,
						drvdata->asyncclk.id,
						0, 0, 0);
		if (ret != 0)
			dev_err(drvdata->dev,
					"Failed to set ASYNCCLK: %d\n", ret);

		ret = snd_soc_component_set_pll(drvdata->codec,
						drvdata->fll2_refclk.id,
						0, 0, 0);
		if (ret < 0)
			dev_err(card->dev,
				"Failed to stop ASYNCCLK FLL REF: %d\n", ret);
	}
}

static int exynos8895_uaif0_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_card *card = rtd->card;
	struct exynos8895_drvdata *drvdata = snd_soc_card_get_drvdata(card);

	dev_info(card->dev, "%s\n", __func__);
	dev_info(card->dev, "%s-%d %dch, %dHz, %dbytes, %dbit\n",
			rtd->dai_link->name, substream->stream,
			params_channels(params), params_rate(params),
			params_buffer_bytes(params), params_width(params));

	drvdata->fll2_refclk.rate = params_rate(params) * params_width(params) * 2;

	if (params_rate(params) % SYSCLK_RATE_CHECK_PARAM)
		drvdata->asyncclk.rate = SYSCLK_RATE_441KHZ;
	else
		drvdata->asyncclk.rate = SYSCLK_RATE_48KHZ;

	return 0;
}

static int exynos8895_uaif0_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_card *card = rtd->card;
	struct exynos8895_drvdata *drvdata = snd_soc_card_get_drvdata(card);
	int ret;

	dev_info(card->dev, "%s-%d prepare\n",
			rtd->dai_link->name, substream->stream);

	ret = snd_soc_component_set_sysclk(drvdata->codec, drvdata->asyncclk.id,
				       drvdata->asyncclk.source,
				       drvdata->asyncclk.rate,
				       SND_SOC_CLOCK_IN);
	if (ret < 0)
		dev_err(card->dev, "Failed to set ASYNCCLK to FLL2: %d\n", ret);

	ret = snd_soc_component_set_pll(drvdata->codec, drvdata->fll2_refclk.id,
					drvdata->fll2_refclk.source,
					drvdata->fll2_refclk.rate,
					drvdata->asyncclk.rate);
	if (ret < 0)
		dev_err(card->dev,
				"Failed to start ASYNCCLK FLL REF: %d\n", ret);

	return 0;
}

static const struct snd_soc_ops uaif0_ops = {
	.hw_params = exynos8895_uaif0_hw_params,
	.shutdown = exynos8895_uaif0_shutdown,
	.prepare = exynos8895_uaif0_prepare,
};

static void exynos8895_madera_hpdet_cb(unsigned int meas)
{
	struct snd_soc_card *card = &exynos8895;
	struct exynos8895_drvdata *drvdata = snd_soc_card_get_drvdata(card);
	int jack_det;
	int i, num_hp_gain_table;

	if (meas == (INT_MAX / 100))
		jack_det = 0;
	else
		jack_det = 1;

	dev_info(card->dev, "%s(%d) meas(%d)\n", __func__, jack_det, meas);

	num_hp_gain_table = (int) ARRAY_SIZE(imp_table.hp_gain_table);
	for (i = 0; i < num_hp_gain_table; i++) {
		if (meas < imp_table.hp_gain_table[i].min
				|| meas > imp_table.hp_gain_table[i].max)
			continue;

		dev_info(card->dev, "SET GAIN %d step for %d ohms\n",
			 imp_table.hp_gain_table[i].gain, meas);
		drvdata->hp_impedance_step = imp_table.hp_gain_table[i].gain;
	}
}

static void exynos8895_madera_micd_cb(bool mic)
{
	struct snd_soc_card *card = &exynos8895;
	struct exynos8895_drvdata *drvdata = snd_soc_card_get_drvdata(card);

	drvdata->ear_mic = mic;
	dev_info(card->dev, "%s: ear_mic = %d\n", __func__, drvdata->ear_mic);
}

/* "imp_table" is absent from the shipped DT (F-24): the stock table stays */
static void exynos8895_update_impedance_table(struct device_node *np)
{
	struct snd_soc_card *card = &exynos8895;
	int len = ARRAY_SIZE(imp_table.hp_gain_table);
	u32 data[ARRAY_SIZE(imp_table.hp_gain_table) * 3];
	int i, ret;

	ret = of_property_read_u32_array(np, "imp_table", data, (len * 3));

	if (!ret) {
		dev_info(card->dev, "%s: data from DT\n", __func__);

		for (i = 0; i < len; i++) {
			imp_table.hp_gain_table[i].min = data[i * 3];
			imp_table.hp_gain_table[i].max = data[(i * 3) + 1];
			imp_table.hp_gain_table[i].gain = data[(i * 3) + 2];
		}
	}
}

static int madera_put_impedance_volsw(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct snd_soc_card *card = codec->card;
	struct exynos8895_drvdata *drvdata = snd_soc_card_get_drvdata(card);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	unsigned int shift = mc->shift;
	int max = mc->max;
	unsigned int mask = (1 << fls(max)) - 1;
	unsigned int invert = mc->invert;
	int err;
	unsigned int val, val_mask;

	val = (unsigned int) (ucontrol->value.integer.value[0] & mask);
	val += drvdata->hp_impedance_step;
	dev_info(card->dev,
			 "SET GAIN %d according to impedance, moved %d step\n",
			 val, drvdata->hp_impedance_step);

	if (invert)
		val = max - val;
	val_mask = mask << shift;
	val = val << shift;

	err = snd_soc_component_update_bits(codec, reg, val_mask, val);
	if (err < 0)
		return err;

	return err;
}

static int dsif_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *hw_params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	unsigned int tx_slot[] = {0, 1};

	/* bclk ratio 64 for DSD64, 128 for DSD128 */
	snd_soc_dai_set_bclk_ratio(cpu_dai, 64);

	/* channel map 0 1 if left is first, 1 0 if right is first */
	snd_soc_dai_set_channel_map(cpu_dai, 2, tx_slot, 0, NULL);
	return 0;
}

static const struct snd_soc_ops dsif_ops = {
	.hw_params = dsif_hw_params,
};

static struct clk *xclkout;

static int exynos8895_set_bias_level(struct snd_soc_card *card,
				  struct snd_soc_dapm_context *dapm,
				  enum snd_soc_bias_level level)
{
	struct snd_soc_pcm_runtime *rtd;
	struct snd_soc_dai *codec_dai;
	struct exynos8895_drvdata *drvdata = snd_soc_card_get_drvdata(card);
	int ret;

	rtd = snd_soc_get_pcm_runtime(card, &card->dai_link[MADERA_DAI_OFFSET]);
	if (!rtd)
		return 0;
	codec_dai = snd_soc_rtd_to_codec(rtd, 0);

	if (snd_soc_dapm_to_dev(dapm) != codec_dai->dev)
		return 0;

	/* the codec component is known from late_probe on, as in stock */
	if (!drvdata->codec)
		return 0;

	switch (level) {
	case SND_SOC_BIAS_STANDBY:
		if (snd_soc_dapm_get_bias_level(dapm) == SND_SOC_BIAS_OFF) {
			clk_enable(xclkout);

			ret = snd_soc_component_set_sysclk(drvdata->codec,
						drvdata->sysclk.id,
						drvdata->sysclk.source,
						drvdata->sysclk.rate,
						SND_SOC_CLOCK_IN);
			if (ret < 0)
				dev_err(card->dev,
					"Failed to set SYSCLK to FLL1: %d\n",
					ret);

			ret = snd_soc_component_set_pll(drvdata->codec,
						drvdata->fll1_refclk.id,
						drvdata->fll1_refclk.source,
						drvdata->fll1_refclk.rate,
						drvdata->sysclk.rate);
			if (ret < 0)
				dev_err(card->dev,
					"Failed to start SYSCLK FLL REF: %d\n",
					ret);
		}
		break;
	case SND_SOC_BIAS_OFF:
		clk_disable(xclkout);

		ret = snd_soc_component_set_sysclk(drvdata->codec,
						drvdata->sysclk.id,
						0, 0, 0);
		if (ret != 0)
			dev_err(drvdata->dev,
					"Failed to set SYSCLK: %d\n", ret);

		ret = snd_soc_component_set_pll(drvdata->codec,
						drvdata->fll1_refclk.id,
						0, 0, 0);
		if (ret < 0)
			dev_err(card->dev,
				"Failed to stop SYSCLK FLL REF: %d\n", ret);
		break;
	default:
		break;
	}

	dev_dbg(card->dev, "%s: %d\n", __func__, level);

	return 0;
}

static int exynos8895_set_bias_level_post(struct snd_soc_card *card,
				       struct snd_soc_dapm_context *dapm,
				       enum snd_soc_bias_level level)
{
	dev_dbg(card->dev, "%s: %d\n", __func__, level);

	return 0;
}

static int exynos8895_suspend_post(struct snd_soc_card *card)
{
	struct exynos8895_drvdata *drvdata = snd_soc_card_get_drvdata(card);
	int ret;

	if (!drvdata->codec)
		return 0;

	if (!snd_soc_component_active(drvdata->codec)) {
		if (drvdata->ear_mic) {
			if (!vts_is_recognitionrunning())
				madera_enable_force_bypass(drvdata->codec);
		} else if (vts_is_recognitionrunning()) {
			madera_enable_force_bypass(drvdata->codec);

			ret = snd_soc_component_set_sysclk(drvdata->codec,
							drvdata->sysclk.id,
							0, 0, 0);
			if (ret != 0)
				dev_err(drvdata->dev,
					"Failed to set SYSCLK: %d\n", ret);

			ret = snd_soc_component_set_pll(drvdata->codec,
							drvdata->fll1_refclk.id,
							0, 0, 0);
			if (ret < 0)
				dev_err(card->dev,
					"Failed to stop SYSCLK FLL REF: %d\n",
					ret);
		}
	}

	return 0;
}

static int exynos8895_resume_pre(struct snd_soc_card *card)
{
	struct exynos8895_drvdata *drvdata = snd_soc_card_get_drvdata(card);
	int ret;

	if (!drvdata->codec)
		return 0;

	if (!snd_soc_component_active(drvdata->codec)) {
		if (drvdata->ear_mic) {
			if (!vts_is_recognitionrunning())
				madera_disable_force_bypass(drvdata->codec);
		} else if (vts_is_recognitionrunning()) {
			ret = snd_soc_component_set_sysclk(drvdata->codec,
						drvdata->sysclk.id,
						drvdata->sysclk.source,
						drvdata->sysclk.rate,
						SND_SOC_CLOCK_IN);
			if (ret < 0)
				dev_err(card->dev,
					"Failed to set SYSCLK to FLL1: %d\n",
					ret);

			ret = snd_soc_component_set_pll(drvdata->codec,
						drvdata->fll1_refclk.id,
						drvdata->fll1_refclk.source,
						drvdata->fll1_refclk.rate,
						drvdata->sysclk.rate);
			if (ret < 0)
				dev_err(card->dev,
					"Failed to start SYSCLK FLL REF: %d\n",
					ret);

			madera_disable_force_bypass(drvdata->codec);
		}
	}

	return 0;
}

static int get_sound_wakelock(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct exynos8895_drvdata *drvdata = snd_soc_card_get_drvdata(codec->card);

	ucontrol->value.integer.value[0] = drvdata->wake_lock_switch;

	return 0;
}

static int set_sound_wakelock(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct snd_soc_card *card = codec->card;
	struct exynos8895_drvdata *drvdata = snd_soc_card_get_drvdata(card);
	unsigned int val;

	val = (unsigned int)ucontrol->value.integer.value[0];
	drvdata->wake_lock_switch = val;

	dev_info(card->dev, "%s: %d\n", __func__, drvdata->wake_lock_switch);

	if (drvdata->wake_lock_switch)
		__pm_stay_awake(drvdata->ws);
	else
		__pm_relax(drvdata->ws);

	return 0;
}

static int get_auxpdm_hiz_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct exynos8895_drvdata *drvdata = snd_soc_card_get_drvdata(codec->card);

	ucontrol->value.integer.value[0] = drvdata->hiz_val;

	return 0;
}

static int set_auxpdm_hiz_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct snd_soc_card *card = codec->card;
	struct exynos8895_drvdata *drvdata = snd_soc_card_get_drvdata(card);

	drvdata->hiz_val = ucontrol->value.integer.value[0];

	dev_info(card->dev, "%s ev: %d\n", __func__, drvdata->hiz_val);

	if (drvdata->hiz_val) {
		if (drvdata->spkpdm_gpio) {
			snd_soc_component_update_bits(codec, MADERA_GPIO3_CTRL_1,
					GPIO_AUXPDM_MASK_1,
					GPIO_PDM_FN_1);
			snd_soc_component_update_bits(codec, MADERA_GPIO3_CTRL_2,
					GPIO_AUXPDM_MASK_2,
					GPIO_PDM_FN_2);
			snd_soc_component_update_bits(codec, MADERA_GPIO4_CTRL_1,
					GPIO_AUXPDM_MASK_1,
					GPIO_PDM_FN_1);
			snd_soc_component_update_bits(codec, MADERA_GPIO4_CTRL_2,
					GPIO_AUXPDM_MASK_2,
					GPIO_PDM_FN_2);
		} else if (drvdata->auxpdm_gpio) {
			snd_soc_component_update_bits(codec, MADERA_GPIO3_CTRL_1,
					GPIO_AUXPDM_MASK_1,
					GPIO_AUX_FN_1);
			snd_soc_component_update_bits(codec, MADERA_GPIO3_CTRL_2,
					GPIO_AUXPDM_MASK_2,
					GPIO_AUX_FN_2);
			snd_soc_component_update_bits(codec, MADERA_GPIO4_CTRL_1,
					GPIO_AUXPDM_MASK_1,
					GPIO_AUX_FN_3);
			snd_soc_component_update_bits(codec, MADERA_GPIO4_CTRL_2,
					GPIO_AUXPDM_MASK_2,
					GPIO_AUX_FN_4);
		} else {
			snd_soc_component_update_bits(codec, MADERA_GPIO10_CTRL_1,
					GPIO_AUXPDM_MASK_1,
					GPIO_AUX_FN_1);
			snd_soc_component_update_bits(codec, MADERA_GPIO10_CTRL_2,
					GPIO_AUXPDM_MASK_2,
					GPIO_AUX_FN_2);
			snd_soc_component_update_bits(codec, MADERA_GPIO9_CTRL_1,
					GPIO_AUXPDM_MASK_1,
					GPIO_AUX_FN_3);
			snd_soc_component_update_bits(codec, MADERA_GPIO9_CTRL_2,
					GPIO_AUXPDM_MASK_2,
					GPIO_AUX_FN_4);
		}
	} else {
		if (drvdata->spkpdm_gpio || drvdata->auxpdm_gpio) {
			snd_soc_component_update_bits(codec, MADERA_GPIO3_CTRL_1,
					GPIO_AUXPDM_MASK_1,
					GPIO_HIZ_FN_1);
			snd_soc_component_update_bits(codec, MADERA_GPIO3_CTRL_2,
					GPIO_AUXPDM_MASK_2,
					GPIO_HIZ_FN_2);
			snd_soc_component_update_bits(codec, MADERA_GPIO4_CTRL_1,
					GPIO_AUXPDM_MASK_1,
					GPIO_HIZ_FN_1);
			snd_soc_component_update_bits(codec, MADERA_GPIO4_CTRL_2,
					GPIO_AUXPDM_MASK_2,
					GPIO_HIZ_FN_2);
		} else {
			snd_soc_component_update_bits(codec, MADERA_GPIO10_CTRL_1,
					GPIO_AUXPDM_MASK_1,
					GPIO_HIZ_FN_1);
			snd_soc_component_update_bits(codec, MADERA_GPIO10_CTRL_2,
					GPIO_AUXPDM_MASK_2,
					GPIO_HIZ_FN_2);
			snd_soc_component_update_bits(codec, MADERA_GPIO9_CTRL_1,
					GPIO_AUXPDM_MASK_1,
					GPIO_HIZ_FN_1);
			snd_soc_component_update_bits(codec, MADERA_GPIO9_CTRL_2,
					GPIO_AUXPDM_MASK_2,
					GPIO_HIZ_FN_2);
		}
	}

	return 0;
}

static int get_vts_en_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct exynos8895_drvdata *drvdata = snd_soc_card_get_drvdata(codec->card);

	ucontrol->value.integer.value[0] = drvdata->vts_state;

	return 0;
}

static int set_vts_en_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct exynos8895_drvdata *drvdata = snd_soc_card_get_drvdata(codec->card);

	drvdata->vts_state = ucontrol->value.integer.value[0];

	return 0;
}

static int get_micbias_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct exynos8895_drvdata *drvdata = snd_soc_card_get_drvdata(codec->card);

	ucontrol->value.integer.value[0] = drvdata->micbias_mode;

	return 0;
}

static int set_micbias_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct exynos8895_drvdata *drvdata = snd_soc_card_get_drvdata(codec->card);

	drvdata->micbias_mode = ucontrol->value.integer.value[0];

	if (drvdata->micbias_mode) {
		madera_enable_force_bypass(drvdata->codec);
	} else {
		madera_disable_force_bypass(drvdata->codec);
	}

	return 0;
}


static const struct snd_kcontrol_new exynos8895_codec_controls[] = {
	SOC_SINGLE_EXT_TLV("HPOUT2L Impedance Volume",
		MADERA_DAC_DIGITAL_VOLUME_2L,
		MADERA_OUT2L_VOL_SHIFT, 0xbf, 0,
		snd_soc_get_volsw, madera_put_impedance_volsw,
		digital_tlv),

	SOC_SINGLE_EXT_TLV("HPOUT2R Impedance Volume",
		MADERA_DAC_DIGITAL_VOLUME_2R,
		MADERA_OUT2L_VOL_SHIFT, 0xbf, 0,
		snd_soc_get_volsw, madera_put_impedance_volsw,
		digital_tlv),

	SOC_SINGLE_BOOL_EXT("AUXPDM Switch",
			0, get_auxpdm_hiz_mode, set_auxpdm_hiz_mode),

	SOC_SINGLE_BOOL_EXT("VTS Enable",
			0, get_vts_en_mode, set_vts_en_mode),

	SOC_SINGLE_BOOL_EXT("MICBias Bypass Mode",
			0, get_micbias_mode, set_micbias_mode),

	SOC_SINGLE_BOOL_EXT("Sound Wakelock",
			0, get_sound_wakelock, set_sound_wakelock),
};

static int exynos8895_madera_notify(struct notifier_block *nb,
				   unsigned long event, void *data)
{
	const struct madera_hpdet_notify_data *hp_inf;
	const struct madera_micdet_notify_data *md_inf;
	const struct exynos8895_drvdata *drvdata =
		container_of(nb, struct exynos8895_drvdata, nb);

	switch (event) {
	case MADERA_NOTIFY_HPDET:
		hp_inf = data;
		exynos8895_madera_hpdet_cb((hp_inf->impedance_x100 / 100));
		dev_dbg(drvdata->dev, "HPDET val=%d.%02d ohms\n",
			 hp_inf->impedance_x100 / 100,
			 hp_inf->impedance_x100 % 100);
		break;
	case MADERA_NOTIFY_MICDET:
		md_inf = data;
		exynos8895_madera_micd_cb(md_inf->present);
		dev_dbg(drvdata->dev, "MICDET present=%c val=%d.%02d ohms\n",
			 md_inf->present ? 'Y' : 'N',
			 md_inf->impedance_x100 / 100,
			 md_inf->impedance_x100 % 100);
		break;
	default:
		dev_info(drvdata->dev, "notifier event=0x%lx data=0x%p\n",
			 event, data);
		break;
	}

	return NOTIFY_DONE;
}

static int exynos8895_late_probe(struct snd_soc_card *card)
{
	struct exynos8895_drvdata *drvdata = snd_soc_card_get_drvdata(card);
	struct snd_soc_pcm_runtime *rtd;
	struct snd_soc_dai *aif1_dai;
	struct snd_soc_component *codec;
	struct snd_soc_component *cpu;
	int ret;

	rtd = snd_soc_get_pcm_runtime(card, &card->dai_link[0]);
	aif1_dai = snd_soc_rtd_to_cpu(rtd, 0);
	cpu = aif1_dai->component;

	rtd = snd_soc_get_pcm_runtime(card, &card->dai_link[MADERA_DAI_OFFSET]);
	aif1_dai = snd_soc_rtd_to_codec(rtd, 0);
	codec = aif1_dai->component;
	drvdata->codec = codec;
	/* the codec is a cell of the madera MFD (ACK madera-core drvdata) */
	drvdata->madera = dev_get_drvdata(codec->dev->parent);

	ret = snd_soc_add_component_controls(codec, exynos8895_codec_controls,
					ARRAY_SIZE(exynos8895_codec_controls));
	if (ret < 0) {
		dev_err(card->dev,
			"Failed to add controls to codec: %d\n", ret);
		return ret;
	}

	ret = snd_soc_component_set_sysclk(codec, drvdata->sysclk.id,
				       drvdata->sysclk.source,
				       drvdata->sysclk.rate,
				       SND_SOC_CLOCK_IN);
	if (ret != 0) {
		dev_err(drvdata->dev, "Failed to set SYSCLK: %d\n", ret);
		return ret;
	}

	ret = snd_soc_component_set_sysclk(codec, drvdata->asyncclk.id,
				       drvdata->asyncclk.source,
				       drvdata->asyncclk.rate,
				       SND_SOC_CLOCK_IN);
	if (ret != 0) {
		dev_err(drvdata->dev, "Failed to set ASYNCCLK: %d\n", ret);
		return ret;
	}

	ret = snd_soc_component_set_sysclk(codec, drvdata->dspclk.id,
				       drvdata->dspclk.source,
				       drvdata->dspclk.rate,
				       SND_SOC_CLOCK_IN);
	if (ret != 0) {
		dev_err(drvdata->dev, "Failed to set SYSCLK: %d\n", ret);
		return ret;
	}

	ret = snd_soc_component_set_sysclk(codec, drvdata->outclk.id,
				       drvdata->outclk.source,
				       drvdata->outclk.rate,
				       SND_SOC_CLOCK_IN);
	if (ret != 0) {
		dev_err(drvdata->dev,
				"Failed to set OUTCLK to FLL2: %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(aif1_dai, drvdata->asyncclk.id, 0, 0);
	if (ret != 0) {
		dev_err(drvdata->dev, "Failed to set AIF1 clock: %d\n", ret);
		return ret;
	}

	snd_soc_dapm_ignore_suspend(&card->dapm, "VOUTPUT");
	snd_soc_dapm_ignore_suspend(&card->dapm, "VINPUT1");
	snd_soc_dapm_ignore_suspend(&card->dapm, "VINPUT2");
	snd_soc_dapm_ignore_suspend(&card->dapm, "VOUTPUTCALL");
	snd_soc_dapm_ignore_suspend(&card->dapm, "VINPUTCALL");
	snd_soc_dapm_ignore_suspend(&card->dapm, "HEADSETMIC");
	snd_soc_dapm_ignore_suspend(&card->dapm, "RECEIVER");
	snd_soc_dapm_ignore_suspend(&card->dapm, "HEADPHONE");
	snd_soc_dapm_ignore_suspend(&card->dapm, "SPEAKER");
	snd_soc_dapm_ignore_suspend(&card->dapm, "BLUETOOTH MIC");
	snd_soc_dapm_ignore_suspend(&card->dapm, "BLUETOOTH SPK");
	snd_soc_dapm_ignore_suspend(&card->dapm, "DMIC1");
	snd_soc_dapm_ignore_suspend(&card->dapm, "DMIC2");
	snd_soc_dapm_ignore_suspend(&card->dapm, "VTS Virtual Output");
	snd_soc_dapm_sync(&card->dapm);

	snd_soc_dapm_ignore_suspend(snd_soc_component_get_dapm(codec),
							"AIF1 Playback");
	snd_soc_dapm_ignore_suspend(snd_soc_component_get_dapm(codec),
							"AIF1 Capture");
	snd_soc_dapm_ignore_suspend(snd_soc_component_get_dapm(codec),
							"AUXPDM1");
	snd_soc_dapm_sync(snd_soc_component_get_dapm(codec));

	/*
	 * Stock passed the prefixed names ("ABOX RDMA0 Playback"); the 6.18
	 * lookup adds the component's "ABOX" prefix itself (F-41).
	 */
	snd_soc_dapm_ignore_suspend(snd_soc_component_get_dapm(cpu),
							"RDMA0 Playback");
	snd_soc_dapm_ignore_suspend(snd_soc_component_get_dapm(cpu),
							"RDMA1 Playback");
	snd_soc_dapm_ignore_suspend(snd_soc_component_get_dapm(cpu),
							"RDMA2 Playback");
	snd_soc_dapm_ignore_suspend(snd_soc_component_get_dapm(cpu),
							"RDMA3 Playback");
	snd_soc_dapm_ignore_suspend(snd_soc_component_get_dapm(cpu),
							"RDMA4 Playback");
	snd_soc_dapm_ignore_suspend(snd_soc_component_get_dapm(cpu),
							"RDMA5 Playback");
	snd_soc_dapm_ignore_suspend(snd_soc_component_get_dapm(cpu),
							"RDMA6 Playback");
	snd_soc_dapm_ignore_suspend(snd_soc_component_get_dapm(cpu),
							"RDMA7 Playback");
	snd_soc_dapm_ignore_suspend(snd_soc_component_get_dapm(cpu),
							"WDMA0 Capture");
	snd_soc_dapm_ignore_suspend(snd_soc_component_get_dapm(cpu),
							"WDMA1 Capture");
	snd_soc_dapm_ignore_suspend(snd_soc_component_get_dapm(cpu),
							"WDMA2 Capture");
	snd_soc_dapm_ignore_suspend(snd_soc_component_get_dapm(cpu),
							"WDMA3 Capture");
	snd_soc_dapm_ignore_suspend(snd_soc_component_get_dapm(cpu),
							"WDMA4 Capture");
	snd_soc_dapm_sync(snd_soc_component_get_dapm(cpu));

	drvdata->wake_lock_switch = 0;

	drvdata->nb.notifier_call = exynos8895_madera_notify;
	blocking_notifier_chain_register(&drvdata->madera->notifier,
			&drvdata->nb);

	return 0;
}

/*
 * Card unbind (a component going away, or driver removal): stock left the
 * notifier to the platform remove; a rebind would register it twice.
 */
static int exynos8895_card_remove(struct snd_soc_card *card)
{
	struct exynos8895_drvdata *drvdata = snd_soc_card_get_drvdata(card);

	if (drvdata->madera)
		blocking_notifier_chain_unregister(&drvdata->madera->notifier,
				&drvdata->nb);
	drvdata->madera = NULL;
	drvdata->codec = NULL;

	return 0;
}

/*
 * cpu DAI names preset as stock .cpu_dai_name; the of_nodes, codecs and
 * platforms come from the "sound" node's children at probe (spec L6E 5.9).
 */
static struct snd_soc_dai_link_component exynos8895_cpus[] = {
	{ .dai_name = "RDMA0" }, { .dai_name = "RDMA1" },
	{ .dai_name = "RDMA2" }, { .dai_name = "RDMA3" },
	{ .dai_name = "RDMA4" }, { .dai_name = "RDMA5" },
	{ .dai_name = "RDMA6" }, { .dai_name = "RDMA7" },
	{ .dai_name = "WDMA0" }, { .dai_name = "WDMA1" },
	{ .dai_name = "WDMA2" }, { .dai_name = "WDMA3" },
	{ .dai_name = "WDMA4" },
	{ .dai_name = "UAIF0" }, { .dai_name = "UAIF1" },
	{ .dai_name = "UAIF2" }, { .dai_name = "UAIF3" },
	{ .dai_name = "UAIF4" },
	{ .dai_name = "DSIF" },
	{ .dai_name = "Internal" },
	{ .dai_name = "vts-tri" },
	{ .dai_name = "vts-rec" },
};

static struct snd_soc_dai_link_component
		exynos8895_codecs[ARRAY_SIZE(exynos8895_cpus)];
static struct snd_soc_dai_link_component
		exynos8895_platforms[ARRAY_SIZE(exynos8895_cpus)];

/*
 * ABOX DMA front end: stock .dynamic, .ignore_suspend, trigger
 * {POST_PRE, PRE_POST} = 6.18 {POST, PRE} (F-36), empty rdma/wdma_ops,
 * .dpcm_playback/.dpcm_capture = one-way link.
 */
#define EXYNOS8895_FE_LINK(xname, idx, xdir)				\
	{								\
		.name = xname,						\
		.stream_name = xname,					\
		.cpus = &exynos8895_cpus[idx],				\
		.num_cpus = 1,						\
		.dynamic = 1,						\
		.ignore_suspend = 1,					\
		.trigger = {						\
			SND_SOC_DPCM_TRIGGER_POST,			\
			SND_SOC_DPCM_TRIGGER_PRE			\
		},							\
		.xdir = 1,						\
	}

static struct snd_soc_dai_link exynos8895_dai[] = {
	EXYNOS8895_FE_LINK("RDMA0", 0, playback_only),
	EXYNOS8895_FE_LINK("RDMA1", 1, playback_only),
	EXYNOS8895_FE_LINK("RDMA2", 2, playback_only),
	EXYNOS8895_FE_LINK("RDMA3", 3, playback_only),
	EXYNOS8895_FE_LINK("RDMA4", 4, playback_only),
	EXYNOS8895_FE_LINK("RDMA5", 5, playback_only),
	EXYNOS8895_FE_LINK("RDMA6", 6, playback_only),
	EXYNOS8895_FE_LINK("RDMA7", 7, playback_only),
	EXYNOS8895_FE_LINK("WDMA0", 8, capture_only),
	EXYNOS8895_FE_LINK("WDMA1", 9, capture_only),
	EXYNOS8895_FE_LINK("WDMA2", 10, capture_only),
	EXYNOS8895_FE_LINK("WDMA3", 11, capture_only),
	EXYNOS8895_FE_LINK("WDMA4", 12, capture_only),
	{
		.name = "UAIF0",
		.stream_name = "UAIF0",
		.cpus = &exynos8895_cpus[13],
		.num_cpus = 1,
		.dai_fmt = SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBC_CFC,
		.no_pcm = 1,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.be_hw_params_fixup = abox_hw_params_fixup_helper,
		.ops = &uaif0_ops,
	},
	{
		.name = "UAIF1",
		.stream_name = "UAIF1",
		.cpus = &exynos8895_cpus[14],
		.num_cpus = 1,
		.dai_fmt = SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBP_CFP,
		.no_pcm = 1,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.be_hw_params_fixup = abox_hw_params_fixup_helper,
	},
	{
		.name = "UAIF2",
		.stream_name = "UAIF2",
		.cpus = &exynos8895_cpus[15],
		.num_cpus = 1,
		.dai_fmt = SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBC_CFC,
		.no_pcm = 1,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.be_hw_params_fixup = abox_hw_params_fixup_helper,
	},
	{
		.name = "UAIF3",
		.stream_name = "UAIF3",
		.cpus = &exynos8895_cpus[16],
		.num_cpus = 1,
		.dai_fmt = SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBC_CFC,
		.no_pcm = 1,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.be_hw_params_fixup = abox_hw_params_fixup_helper,
	},
	{
		.name = "UAIF4",
		.stream_name = "UAIF4",
		.cpus = &exynos8895_cpus[17],
		.num_cpus = 1,
		.dai_fmt = SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBC_CFC,
		.no_pcm = 1,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.be_hw_params_fixup = abox_hw_params_fixup_helper,
	},
	{
		.name = "DSIF",
		.stream_name = "DSIF",
		.cpus = &exynos8895_cpus[18],
		.num_cpus = 1,
		.dai_fmt = SND_SOC_DAIFMT_PDM |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBC_CFC,
		.no_pcm = 1,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.be_hw_params_fixup = abox_hw_params_fixup_helper,
		.ops = &dsif_ops,
		.playback_only = 1,
	},
	{
		.name = "ABOX Internal",
		.stream_name = "ABOX Internal",
		.cpus = &exynos8895_cpus[19],
		.num_cpus = 1,
		.dai_fmt = SND_SOC_DAIFMT_PDM |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBC_CFC,
		.no_pcm = 1,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
	},
	{
		.name = "VTS-Trigger",
		.stream_name = "VTS-Trigger",
		.cpus = &exynos8895_cpus[20],
		.num_cpus = 1,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.capture_only = 1,
	},
	{
		.name = "VTS-Record",
		.stream_name = "VTS-Record",
		.cpus = &exynos8895_cpus[21],
		.num_cpus = 1,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.capture_only = 1,
	},
	/* stock "DP Audio" (O-E22) and "CPU-DSP trace" (O-E9) links: not provided */
};

static int dream_dmic1(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_card *card = w->dapm->card;

	dev_info(card->dev, "%s ev: %d\n", __func__, event);

	return 0;
}

static int dream_dmic2(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_card *card = w->dapm->card;

	dev_info(card->dev, "%s ev: %d\n", __func__, event);

	return 0;
}

static int dream_headsetmic(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_card *card = w->dapm->card;

	dev_info(card->dev, "%s ev: %d\n", __func__, event);

	return 0;
}

static int dream_receiver(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_card *card = w->dapm->card;

	dev_info(card->dev, "%s ev: %d\n", __func__, event);

	return 0;
}

static int dream_headphone(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_card *card = w->dapm->card;

	dev_info(card->dev, "%s ev: %d\n", __func__, event);

	return 0;
}

static int dream_speaker(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_card *card = w->dapm->card;

	dev_info(card->dev, "%s ev: %d\n", __func__, event);

	return 0;
}

static int dream_btsco_tx(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_card *card = w->dapm->card;

	dev_info(card->dev, "%s ev: %d\n", __func__, event);

	return 0;
}

static int dream_btsco_rx(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_card *card = w->dapm->card;

	dev_info(card->dev, "%s ev: %d\n", __func__, event);

	return 0;
}

static const char * const vts_output_texts[] = {
	"None",
	"DMIC1",
};

static const struct soc_enum vts_output_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, ARRAY_SIZE(vts_output_texts),
			vts_output_texts);

static const struct snd_kcontrol_new vts_output_mux[] = {
	SOC_DAPM_ENUM("VTS Virtual Output Mux", vts_output_enum),
};

static const struct snd_kcontrol_new exynos8895_controls[] = {
	SOC_DAPM_PIN_SWITCH("DMIC1"),
	SOC_DAPM_PIN_SWITCH("DMIC2"),
};

static const struct snd_soc_dapm_widget exynos8895_widgets[] = {
	SND_SOC_DAPM_OUTPUT("VOUTPUT"),
	SND_SOC_DAPM_INPUT("VINPUT1"),
	SND_SOC_DAPM_INPUT("VINPUT2"),
	SND_SOC_DAPM_OUTPUT("VOUTPUTCALL"),
	SND_SOC_DAPM_INPUT("VINPUTCALL"),
	SND_SOC_DAPM_MIC("DMIC1", dream_dmic1),
	SND_SOC_DAPM_MIC("DMIC2", dream_dmic2),
	SND_SOC_DAPM_MIC("HEADSETMIC", dream_headsetmic),
	SND_SOC_DAPM_SPK("RECEIVER", dream_receiver),
	SND_SOC_DAPM_HP("HEADPHONE", dream_headphone),
	SND_SOC_DAPM_SPK("SPEAKER", dream_speaker),
	SND_SOC_DAPM_MIC("BLUETOOTH MIC", dream_btsco_tx),
	SND_SOC_DAPM_SPK("BLUETOOTH SPK", dream_btsco_rx),
	SND_SOC_DAPM_OUTPUT("VTS Virtual Output"),
	SND_SOC_DAPM_MUX("VTS Virtual Output Mux",
			SND_SOC_NOPM, 0, 0, &vts_output_mux[0]),
};

static const struct snd_soc_dapm_route exynos8895_routes[] = {
	{"VTS Virtual Output Mux", "DMIC1", "DMIC1"},
};

/* .dlc.of_node from "samsung,codec" at probe */
static struct snd_soc_codec_conf codec_conf[] = {
	{
		.name_prefix = "ABOX",
	},
	{
		.name_prefix = "VTS",
	},
};

/* stock aux "EFFECT": .dlc.of_node from "samsung,aux" at probe (no .name in 6.18) */
static struct snd_soc_aux_dev aux_dev[1];

static struct snd_soc_card exynos8895 = {
	.name = "Exynos8895-Madera",
	.owner = THIS_MODULE,
	.dai_link = exynos8895_dai,
	.num_links = ARRAY_SIZE(exynos8895_dai),

	.late_probe = exynos8895_late_probe,
	.remove = exynos8895_card_remove,

	.controls = exynos8895_controls,
	.num_controls = ARRAY_SIZE(exynos8895_controls),
	.dapm_widgets = exynos8895_widgets,
	.num_dapm_widgets = ARRAY_SIZE(exynos8895_widgets),
	.dapm_routes = exynos8895_routes,
	.num_dapm_routes = ARRAY_SIZE(exynos8895_routes),

	.set_bias_level = exynos8895_set_bias_level,
	.set_bias_level_post = exynos8895_set_bias_level_post,

	.drvdata = (void *)&exynos8895_drvdata,

	.suspend_post = exynos8895_suspend_post,
	.resume_pre = exynos8895_resume_pre,

	.codec_conf = codec_conf,
	.num_configs = ARRAY_SIZE(codec_conf),

	.aux_dev = aux_dev,
	.num_aux_devs = ARRAY_SIZE(aux_dev),
};

static int exynos8895_read_clk_conf(struct device_node *np,
				   const char * const prop,
				   struct clk_conf *conf)
{
	u32 tmp;
	int ret;

	ret = of_property_read_u32_index(np, prop, 0, &tmp);
	if (ret)
		return ret;

	conf->id = tmp;

	ret = of_property_read_u32_index(np, prop, 1, &tmp);
	if (ret)
		return ret;

	conf->source = tmp;
	conf->source--;

	ret = of_property_read_u32_index(np, prop, 2, &tmp);
	if (ret)
		return ret;

	conf->rate = tmp;

	return 0;
}

/*
 * Stock exynos8895_read_dai() with a preset DAI name
 * (S:sound/soc/samsung/dream_madera.c:1428-1456, :1524-1552): the cpu and
 * platform children give an of_node; a codec child is resolved to its DAI
 * (-EPROBE_DEFER until the codec is registered); a link without a codec
 * child gets the dummy codec, one without a platform child none.
 */
static int exynos8895_read_link(struct device *dev, struct device_node *np,
		struct snd_soc_dai_link *link, int i)
{
	struct device_node *child;
	int ret;

	/* a retry after -EPROBE_DEFER parses the nodes again */
	of_node_put(exynos8895_cpus[i].of_node);
	exynos8895_cpus[i].of_node = NULL;
	of_node_put(exynos8895_platforms[i].of_node);
	exynos8895_platforms[i].of_node = NULL;
	of_node_put(exynos8895_codecs[i].of_node);
	exynos8895_codecs[i].of_node = NULL;

	child = of_get_child_by_name(np, "cpu");
	if (child) {
		exynos8895_cpus[i].of_node = of_parse_phandle(child,
				"sound-dai", 0);
		of_node_put(child);
	}
	if (!exynos8895_cpus[i].of_node)
		dev_warn(dev, "Failed to parse cpu DAI for %pOFn\n", np);

	child = of_get_child_by_name(np, "platform");
	if (child) {
		exynos8895_platforms[i].of_node = of_parse_phandle(child,
				"sound-dai", 0);
		of_node_put(child);
		link->platforms = &exynos8895_platforms[i];
		link->num_platforms = 1;
	}

	child = of_get_child_by_name(np, "codec");
	if (child) {
		ret = snd_soc_of_get_dlc(child, NULL, &exynos8895_codecs[i], 0);
		of_node_put(child);
		if (ret) {
			if (ret != -EPROBE_DEFER)
				dev_warn(dev,
					"Failed to parse codec DAI for %pOFn:%d\n",
					np, ret);
			return ret;
		}
		link->codecs = &exynos8895_codecs[i];
	} else {
		link->codecs = &snd_soc_dummy_dlc;
	}
	link->num_codecs = 1;

	return 0;
}

static void exynos8895_unregister_ws(void *ws)
{
	wakeup_source_unregister(ws);
}

static int exynos8895_audio_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &exynos8895;
	struct exynos8895_drvdata *drvdata = snd_soc_card_get_drvdata(card);
	struct device_node *np = pdev->dev.of_node;
	struct platform_device *abox_pdev, *vts_pdev;
	struct device_node *dai;
	int nlink = 0, n;
	int ret;

	card->dev = &pdev->dev;
	drvdata->dev = card->dev;

	exynos8895_update_impedance_table(np);

	drvdata->auxpdm_gpio =
		of_property_read_bool(np, "auxpdm-gpio");

	drvdata->spkpdm_gpio =
		of_property_read_bool(np, "spkpdm-gpio");

	/* PMU CLKOUT, not in a power domain: prepared once, as stock */
	xclkout = devm_clk_get_prepared(&pdev->dev, "xclkout");
	if (IS_ERR(xclkout)) {
		dev_err(&pdev->dev, "xclkout get failed\n");
		return PTR_ERR(xclkout);
	}

	ret = exynos8895_read_clk_conf(np, "cirrus,sysclk", &drvdata->sysclk);
	if (ret) {
		dev_err(card->dev, "Failed to parse sysclk: %d\n", ret);
		return ret;
	}
	ret = exynos8895_read_clk_conf(np, "cirrus,asyncclk",
							&drvdata->asyncclk);
	if (ret) {
		dev_err(card->dev, "Failed to parse asyncclk: %d\n", ret);
		return ret;
	}
	ret = exynos8895_read_clk_conf(np, "cirrus,dspclk", &drvdata->dspclk);
	if (ret) {
		dev_err(card->dev, "Failed to parse dspclk: %d\n", ret);
		return ret;
	}
	ret = exynos8895_read_clk_conf(np, "cirrus,outclk", &drvdata->outclk);
	if (ret) {
		dev_err(card->dev, "Failed to parse outclk: %d\n", ret);
		return ret;
	}
	ret = exynos8895_read_clk_conf(np, "cirrus,fll1-refclk",
				      &drvdata->fll1_refclk);
	if (ret) {
		dev_err(card->dev, "Failed to parse fll1-refclk: %d\n", ret);
		return ret;
	}
	ret = exynos8895_read_clk_conf(np, "cirrus,fll2-refclk",
				      &drvdata->fll2_refclk);
	if (ret) {
		dev_err(card->dev, "Failed to parse fll2-refclk: %d\n", ret);
		return ret;
	}

	for_each_available_child_of_node(np, dai) {
		ret = exynos8895_read_link(card->dev, dai,
				&exynos8895_dai[nlink], nlink);
		if (ret) {
			of_node_put(dai);
			return ret;
		}

		if (++nlink == card->num_links) {
			of_node_put(dai);
			break;
		}
	}

	if (nlink != card->num_links) {
		dev_err(card->dev, "%d of %d DAI links specified\n", nlink,
				card->num_links);
		return -EINVAL;
	}

	if (of_property_present(np, "samsung,routing")) {
		ret = snd_soc_of_parse_audio_routing(card, "samsung,routing");
		if (ret) {
			dev_err(&pdev->dev,
				"Failed to parse audio routing: %d\n", ret);
			return ret;
		}
	}

	for (n = 0; n < ARRAY_SIZE(codec_conf); n++) {
		of_node_put(codec_conf[n].dlc.of_node);
		codec_conf[n].dlc.of_node = of_parse_phandle(np,
							"samsung,codec", n);

		if (!codec_conf[n].dlc.of_node) {
			dev_err(&pdev->dev,
				"Property 'samsung,codec' missing\n");
			return -EINVAL;
		}
	}

	for (n = 0; n < ARRAY_SIZE(aux_dev); n++) {
		of_node_put(aux_dev[n].dlc.of_node);
		aux_dev[n].dlc.of_node = of_parse_phandle(np,
							"samsung,aux", n);

		if (!aux_dev[n].dlc.of_node) {
			dev_err(&pdev->dev,
				"Property 'samsung,aux' missing\n");
			return -EINVAL;
		}
	}

	drvdata->ws = wakeup_source_register(&pdev->dev, "madera-sound");
	if (!drvdata->ws)
		return -ENOMEM;
	ret = devm_add_action_or_reset(&pdev->dev, exynos8895_unregister_ws,
			drvdata->ws);
	if (ret)
		return ret;

	/*
	 * Bind-time power window (F-52): stock ABOX and VTS held a runtime-PM
	 * reference from their probes to a late_initcall
	 * (S:sound/soc/samsung/abox/abox.c:5561-5562, :5656-5667,
	 * S:sound/soc/samsung/vts/vts.c:2266-2267, :2339-2348), so the card
	 * bound with both blocks powered. The ports start suspended; hold the
	 * two blocks on around the synchronous card registration instead.
	 */
	abox_pdev = of_find_device_by_node(codec_conf[0].dlc.of_node);
	vts_pdev = of_find_device_by_node(codec_conf[1].dlc.of_node);
	if (!abox_pdev || !vts_pdev || !abox_pdev->dev.driver ||
			!vts_pdev->dev.driver) {
		ret = -EPROBE_DEFER;
		goto out_put;
	}

	pm_runtime_get_sync(&abox_pdev->dev);
	pm_runtime_get_sync(&vts_pdev->dev);
	ret = devm_snd_soc_register_card(card->dev, card);
	pm_runtime_put(&vts_pdev->dev);
	pm_runtime_put(&abox_pdev->dev);
	if (ret)
		dev_err_probe(card->dev, ret, "snd_soc_register_card() failed\n");

out_put:
	if (vts_pdev)
		put_device(&vts_pdev->dev);
	if (abox_pdev)
		put_device(&abox_pdev->dev);

	return ret;
}

static const struct of_device_id exynos8895_of_match[] = {
	{ .compatible = "samsung,exynos8895-madera", },
	{},
};
MODULE_DEVICE_TABLE(of, exynos8895_of_match);

static struct platform_driver exynos8895_audio_driver = {
	.driver		= {
		.name	= "exynos8895-madera",
		.pm = &snd_soc_pm_ops,
		.of_match_table = exynos8895_of_match,
	},

	.probe		= exynos8895_audio_probe,
};

module_platform_driver(exynos8895_audio_driver);

MODULE_DESCRIPTION("ALSA SoC Exynos8895 Madera Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:exynos8895-madera");
