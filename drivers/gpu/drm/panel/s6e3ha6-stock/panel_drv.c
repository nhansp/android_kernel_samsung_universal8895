/*
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *	      http://www.samsung.com/
 *
 * Samsung's Panel Driver
 * Author: Minwoo Kim <minwoo7945.kim@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/err.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/fb.h>
#include <linux/gpio/consumer.h>
#include <video/mipi_display.h>
#include "panel_dpu.h"
#include "panel.h"
#include "panel_drv.h"
#include "mdnie.h"
#include "spi.h"
#include "active_clock.h"
#include "panel_poc.h"

int panel_log_level = 6;
static int panel_prepare(struct panel_device *panel,
		struct common_panel_info *info);
static struct common_panel_info *panel_detect(struct panel_device *panel);

#define BUCK_ALPM_VOLT		1500000
#define BUCK_NORMAL_VOLT	1600000
static int __set_panel_power(struct panel_device *panel, int power)
{
	int i;
	int ret = 0;
	struct panel_pad *pad = &panel->pad;

	if (panel->state.power == power) {
		panel_warn("PANEL:WARN:%s:same status.. skip..\n", __func__);
		goto set_err;
	}

	if (power == PANEL_POWER_ON) {
		for (i = 0; i < REGULATOR_MAX; i++) {
			ret = regulator_enable(pad->regulator[i]);
			if (ret) {
				panel_err("PANEL:%s:faield to enable regulator : %d :%d\n",
					__func__, i, ret);
				while (--i >= 0)
					regulator_disable(pad->regulator[i]);
				goto set_err;
			}
		}
		usleep_range(10000, 10000);
		gpiod_set_raw_value_cansleep(pad->gpio_reset, 1);
		usleep_range(5000, 5000);
	} else {
		gpiod_set_raw_value_cansleep(pad->gpio_reset, 0);
		for (i = REGULATOR_MAX - 1; i >= 0; i--) {
			ret = regulator_disable(pad->regulator[i]);
			if (ret) {
				panel_err("PANEL:%s:faield to disable regulator : %d :%d\n",
					__func__, i, ret);
				goto set_err;
			}
		}
	}
	panel->state.power = power;
set_err:
	return ret;
}


static int __panel_seq_display_on(struct panel_device *panel)
{
	int ret = 0;

	ret = panel_do_seqtbl_by_index(panel, PANEL_DISPLAY_ON_SEQ);
	if (unlikely(ret < 0)) {
		panel_err("PANEL:ERR:%s, failed to write init seqtbl\n", __func__);
	}

	return ret;
}


static int __panel_seq_display_off(struct panel_device *panel)
{
	int ret;

	ret = panel_do_seqtbl_by_index(panel, PANEL_DISPLAY_OFF_SEQ);
	if (unlikely(ret < 0)) {
		panel_err("PANEL:ERR:%s, failed to write init seqtbl\n", __func__);
	}

	return ret;
}

#define PANEL_DISP_DET_HIGH 	1
#define PANEL_DISP_DET_LOW		0

static int __panel_seq_res_init(struct panel_device *panel)
{
	int ret;

	ret = panel_do_seqtbl_by_index(panel, PANEL_RES_INIT_SEQ);
	if (unlikely(ret < 0)) {
		panel_err("PANEL:ERR:%s, failed to write resource init seqtbl\n",
				__func__);
	}

	return ret;
}

static int __panel_seq_init(struct panel_device *panel)
{
	int ret = 0;
	int retry = 200;
	struct gpio_desc *disp_det = panel->pad.gpio_disp_det;

	if (gpiod_get_raw_value_cansleep(disp_det) == PANEL_DISP_DET_HIGH) {
		panel_warn("PANEL:WARN:%s:disp det already set to 1\n",
			__func__);
		return 0;
	}


	ret = panel_do_seqtbl_by_index(panel, PANEL_INIT_SEQ);
	if (unlikely(ret < 0)) {
		panel_err("PANEL:ERR:%s, failed to write init seqtbl\n", __func__);
	}

check_disp_det:
	if (gpiod_get_raw_value_cansleep(disp_det) == PANEL_DISP_DET_LOW) {
		usleep_range(100, 100);
		if (retry--)
			goto check_disp_det;
		goto do_exit;
	}

	/* the DRM panel owns the disp-det interrupt */
	if (panel->pad.disp_det_irq)
		panel->pad.disp_det_irq(panel, true);

	panel_info("PANEL:INFO:%s:check disp det .. success\n", __func__);
	return 0;

do_exit:
	panel_err("PANEL:ERR:%s:failed to panel power on\n", __func__);
	return -EAGAIN;
}

static int __panel_seq_exit(struct panel_device *panel)
{
	int ret;

	/* the DRM panel owns the disp-det interrupt */
	if (panel->pad.disp_det_irq)
		panel->pad.disp_det_irq(panel, false);

	ret = panel_do_seqtbl_by_index(panel, PANEL_EXIT_SEQ);
	if (unlikely(ret < 0)) {
		panel_err("PANEL:ERR:%s, failed to write exit seqtbl\n", __func__);
	}

	return ret;
}

static int __panel_seq_hmd_on(struct panel_device *panel)
{
	int ret = 0;

	if (!panel) {
		panel_err("PANEL:ERR:%s:pane is null\n", __func__);
		return 0;
	}

	panel_info("PANEK:INFO:%s:hmd was on, setting hmd on seq\n", __func__);
	ret = panel_do_seqtbl_by_index(panel, PANEL_HMD_ON_SEQ);
	if (ret) {
		panel_err("PANEL:ERR:%s:failed to set hmd on seq\n",
			__func__);
	}

	return ret;
}
static int __panel_seq_exit_alpm(struct panel_device *panel)
{
	int ret = 0;
	struct backlight_device *bd = panel->panel_bl.bd;
	int volt;
	struct panel_pad *pad = &panel->pad;
	struct act_clock_dev *act_dev = &panel->act_clk_dev;
	struct act_clk_info *act_info = &act_dev->act_info;

	panel_info("PANEL:INFO:%s:was called\n", __func__);
	volt = regulator_get_voltage(pad->regulator[REGULATOR_1p6V]);
	panel_info("PANEL:INFO:%s:regulator voltage  : %d\n", __func__, volt);
	if (volt != BUCK_NORMAL_VOLT) {
		ret = regulator_set_voltage(pad->regulator[REGULATOR_1p6V],
			BUCK_NORMAL_VOLT, BUCK_NORMAL_VOLT);
		if (ret)
			panel_warn("PANEL:WARN:%s:failed to set volt to 1.6\n",
				__func__);
	}
	ret = panel_do_seqtbl_by_index(panel, PANEL_ALPM_EXIT_SEQ);
	if (ret) {
		panel_err("PANEL:ERR:%s, failed to write init seqtbl\n",
			__func__);
	}

	if ((act_info != NULL) && (act_info->en)) {
		panel_info("PANEL:INFO:%s:act_info %d -> disable\n",
			__func__, act_info->en);
		act_info->en = 0;
	}

	backlight_update_status(bd);
	return ret;
}


/* delay to prevent current leackage when alpm */
/* according to ha6 opmanual, the dealy value is 126msec */
static void __delay_normal_alpm(struct panel_device *panel)
{
	u32 gap;
	u32 delay = 0;
	struct seqinfo *seqtbl;
	struct delayinfo *delaycmd;

	seqtbl = find_index_seqtbl(&panel->panel_data, PANEL_ALPM_DELAY_SEQ);
	if (unlikely(!seqtbl))
		goto exit_delay;

	delaycmd = (struct delayinfo *)seqtbl->cmdtbl[0];
	if (delaycmd->type != CMD_TYPE_DELAY) {
		panel_err("PANEL:ERR:%s:can't find value\n", __func__);
		goto exit_delay;
	}

	if (ktime_after(ktime_get(), panel->ktime_panel_on)) {
		gap = ktime_to_us(ktime_sub(ktime_get(), panel->ktime_panel_on));
		if (gap > delaycmd->usec)
			goto exit_delay;

		delay = delaycmd->usec - gap;
		usleep_range(delay, delay);
	}
	panel_info("PANEL:INFO:%stotal elapsed time : %d\n", __func__,
		(int)ktime_to_us(ktime_sub(ktime_get(), panel->ktime_panel_on)));
exit_delay:
	return;
}

static int __panel_seq_set_alpm(struct panel_device *panel)
{
	int ret = 0;
	int volt = 0;
	struct panel_pad *pad = &panel->pad;
	panel_info("PANEL:INFO:%s:was called\n", __func__);

	__delay_normal_alpm(panel);

	ret = panel_do_seqtbl_by_index(panel, PANEL_ALPM_ENTER_SEQ);
	if (ret) {
		panel_err("PANEL:ERR:%s, failed to write init seqtbl\n",
			__func__);
	}
	volt = regulator_get_voltage(pad->regulator[REGULATOR_1p6V]);
	panel_info("PANEL:INFO:%s:regulator voltage : %d\n", __func__, volt);
	if (volt != BUCK_ALPM_VOLT) {
		ret = regulator_set_voltage(pad->regulator[REGULATOR_1p6V],
			BUCK_ALPM_VOLT, BUCK_ALPM_VOLT);
		if (ret)
			panel_warn("PANEL:WARN:%s:failed to set volt to 1.5\n",
				__func__);
	}
	return ret;
}

static int __panel_seq_active_clock(struct panel_device *panel, int send_img)
{
	int ret = 0;
	struct act_clock_dev *act_dev = &panel->act_clk_dev;
	struct act_clk_info *act_info = &act_dev->act_info;
	struct act_blink_info *blink_info = &act_dev->blink_info;

	if (act_info->en) {
		panel_dbg("PANEL:DBG:%s:active clock was enabed\n", __func__);
		if (send_img) {
			panel_dbg("PANEL:DBG:%s:send active clock's image\n", __func__);

			if (act_info->update_img == IMG_UPDATE_NEED) {
				ret = panel_do_seqtbl_by_index(panel, PANEL_ACTIVE_CLK_IMG_SEQ);
				if (unlikely(ret < 0)) {
					panel_err("PANEL:ERR:%s, failed to write init seqtbl\n", __func__);
				}
				act_info->update_img = IMG_UPDATE_DONE;
			}
		}
		usleep_range(5, 5);

		ret = panel_do_seqtbl_by_index(panel, PANEL_ACTIVE_CLK_CTRL_SEQ);
		if (unlikely(ret < 0)) {
			panel_err("PANEL:ERR:%s, failed to write init seqtbl\n", __func__);
		}
	}

	if (blink_info->en) {
		panel_dbg("PANEL:DBG:%s:active blink was enabed\n", __func__);

		ret = panel_do_seqtbl_by_index(panel, PANEL_ACTIVE_CLK_CTRL_SEQ);
		if (unlikely(ret < 0)) {
			panel_err("PANEL:ERR:%s, failed to write init seqtbl\n", __func__);
		}
	}
	return ret;
}

static int __panel_seq_dump(struct panel_device *panel)
{
	int ret;

	ret = panel_do_seqtbl_by_index(panel, PANEL_DUMP_SEQ);
	if (unlikely(ret < 0)) {
		panel_err("PANEL:ERR:%s, failed to write dump seqtbl\n", __func__);
	}

	return ret;
}

static int panel_debug_dump(struct panel_device *panel)
{
	int ret, disp_det;
	struct panel_pad *pad = &panel->pad;

	if (unlikely(!panel)) {
		panel_err("PANEL:ERR:%s:panel is null\n", __func__);
		goto dump_exit;
	}

	if (!IS_PANEL_ACTIVE(panel)) {
		panel_info("PANEL:INFO:Current state:%d\n", panel->state.cur_state);
		goto dump_exit;
	}

	ret = __panel_seq_dump(panel);
	if (ret) {
		panel_err("PANEL:ERR:%s:failed to dump\n", __func__);
		return ret;
	}

	disp_det = gpiod_get_raw_value_cansleep(pad->gpio_disp_det);
	panel_info("PANEL:INFO:%s: disp gpio : %d\n", __func__, disp_det);
dump_exit:
	return 0;
}

static int panel_display_on(struct panel_device *panel)
{
	int ret = 0;
	struct panel_state *state = &panel->state;

	if (state->connect_panel == PANEL_DISCONNECT) {
		panel_warn("PANEL:WANR:%s:panel disconnected\n", __func__);
		goto do_exit;
	}

	if (state->cur_state == PANEL_STATE_OFF) {
		panel_warn("PANEL:WANR:%s:panel off\n", __func__);
		goto do_exit;
	}

	ret = __panel_seq_display_on(panel);
	if (ret) {
		panel_err("PANEL:ERR:%s:failed to display on\n", __func__);
		return ret;
	}

	state->disp_on = PANEL_DISPLAY_ON;
	copr_enable(&panel->copr);
	mdnie_enable(&panel->mdnie);

	return 0;

do_exit:
	return ret;
}

static int panel_display_off(struct panel_device *panel)
{
	int ret = 0;
	struct panel_state *state = &panel->state;

	if (state->connect_panel == PANEL_DISCONNECT) {
		panel_warn("PANEL:WANR:%s:panel disconnected\n", __func__);
		goto do_exit;
	}

	if (state->cur_state == PANEL_STATE_OFF) {
		panel_warn("PANEL:WANR:%s:panel off\n", __func__);
		goto do_exit;
	}

	ret = __panel_seq_display_off(panel);
	if (unlikely(ret < 0)) {
		panel_err("PANEL:ERR:%s, failed to write init seqtbl\n",
			__func__);
	}
	state->disp_on = PANEL_DISPLAY_OFF;

	return 0;
do_exit:
	return ret;
}

static struct common_panel_info *panel_detect(struct panel_device *panel)
{
	u8 id[3];
	u32 panel_id;
	int ret = 0;
	struct common_panel_info *info;
	struct panel_info *panel_data;

	if (panel == NULL) {
		panel_err("%s, panel is null\n", __func__);
		return NULL;
	}
	panel_data = &panel->panel_data;

	if (!panel_data->ldi_name) {
		panel_err("%s, panel ldi_name is empty\n", __func__);
		return NULL;
	}

	memset(id, 0, sizeof(id));
 	ret = read_panel_id(panel, id);
	if (unlikely(ret < 0)) {
		panel_err("%s, failed to read id(ret %d)\n", __func__, ret);
	}

	panel_id = (id[0] << 16) | (id[1] << 8) | id[2];
	memcpy(panel_data->id, id, sizeof(id));

	panel_dbg("%s, ldi_name %s, panel_id 0x%08X\n",
			__func__, panel_data->ldi_name, panel_id);


	info = find_panel(panel, panel_id);
	if (unlikely(!info)) {
		panel_err("%s, panel not found (name %s id 0x%08X)\n",
				__func__, panel_data->ldi_name, panel_id);
		return NULL;
	}

	return info;
}

static int panel_prepare(struct panel_device *panel, struct common_panel_info *info)
{
	int i;
	struct panel_info *panel_data = &panel->panel_data;

	mutex_lock(&panel->op_lock);
	panel_data->maptbl = info->maptbl;
	panel_data->nr_maptbl = info->nr_maptbl;
	panel_data->seqtbl = info->seqtbl;
	panel_data->nr_seqtbl = info->nr_seqtbl;
	panel_data->rditbl = info->rditbl;
	panel_data->nr_rditbl = info->nr_rditbl;
	panel_data->restbl = info->restbl;
	panel_data->nr_restbl = info->nr_restbl;
	panel_data->dumpinfo = info->dumpinfo;
	panel_data->nr_dumpinfo = info->nr_dumpinfo;
	panel_data->panel_dim_info[0] = info->panel_dim_info[0];
	panel_data->panel_dim_info[1] = info->panel_dim_info[1];
	for (i = 0; i < panel_data->nr_maptbl; i++)
		panel_data->maptbl[i].pdata = panel;
	for (i = 0; i < panel_data->nr_restbl; i++)
		panel_data->restbl[i].state = RES_UNINITIALIZED;
	mutex_unlock(&panel->op_lock);

	__panel_seq_res_init(panel);
	print_panel_resource(panel);

	mutex_lock(&panel->op_lock);
	for (i = 0; i < panel_data->nr_maptbl; i++)
		maptbl_init(&panel_data->maptbl[i]);
	mutex_unlock(&panel->op_lock);

	return 0;
}

static int panel_probe(struct panel_device *panel)
{
	int i, ret = 0;
	struct decon_lcd *lcd_info;
	struct panel_info *panel_data;
	struct common_panel_info *info;

	panel_dbg("%s was callled\n", __func__);

	if (panel == NULL) {
		panel_err("PANEL:ERR:%s:panel is null\n", __func__);
		return -EINVAL;
	}

	lcd_info = &panel->lcd_info;
	panel_data = &panel->panel_data;
	panel_data->ldi_name = lcd_info->ldi_name;

	info = panel_detect(panel);
	if (unlikely(!info)) {
		panel_err("PANEL:ERR:%s:panel detection failed\n", __func__);
		return -ENODEV;
	}

	/* TODO : move to parse dt function
	   1. get panel_spi device node.
	   2. get spi_device of node
	 */
	panel->spi = of_find_panel_spi_by_node(NULL);
	if (!panel->spi)
		panel_warn("%s, panel spi device unsupported\n", __func__);

	mutex_lock(&panel->op_lock);
	panel_data->props.temperature = NORMAL_TEMPERATURE;
	panel_data->props.alpm_mode = ALPM_OFF;
	panel_data->props.cur_alpm_mode = ALPM_OFF;

	for (i = 0; i < MAX_CMD_LEVEL; i++)
		panel_data->props.key[i] = 0;
	mutex_unlock(&panel->op_lock);

	mutex_lock(&panel->panel_bl.lock);
	panel_data->props.adaptive_control = ACL_OPR_MAX - 1;
	panel_data->props.poc_onoff = POC_ONOFF_ON;
	panel_data->props.irc_onoff = IRC_ONOFF_ON;

	mutex_unlock(&panel->panel_bl.lock);

	ret = panel_prepare(panel, info);
	if (unlikely(ret)) {
		pr_err("%s, failed to prepare common panel driver\n", __func__);
		return -ENODEV;
	}

	panel->lcd = lcd_device_register("panel", panel->dev, panel, NULL);
	if (IS_ERR(panel->lcd)) {
		panel_err("%s, faield to register lcd device\n", __func__);
		return PTR_ERR(panel->lcd);
	}

	ret = panel_bl_probe(panel);
	if (unlikely(ret)) {
		pr_err("%s, failed to probe backlight driver\n", __func__);
		return -ENODEV;
	}

	ret = panel_sysfs_probe(panel);
	if (unlikely(ret)) {
		pr_err("%s, failed to init sysfs\n", __func__);
		return -ENODEV;
	}

	ret = mdnie_probe(panel, info->mdnie_tune);
	if (unlikely(ret)) {
		pr_err("%s, failed to probe mdnie driver\n", __func__);
		return -ENODEV;
	}

	ret = copr_probe(panel, info->copr_data);
	if (unlikely(ret)) {
		pr_err("%s, failed to probe copr driver\n", __func__);
		BUG();
		return -ENODEV;
	}

	ret = probe_live_clock_drv(&panel->act_clk_dev);
	if (unlikely(ret)) {
		pr_err("%s, failed to probe copr driver\n", __func__);
		BUG();
		return -ENODEV;
	}

	ret = panel_poc_probe(&panel->poc_dev);
	if (unlikely(ret)) {
		pr_err("%s, failed to probe poc driver\n", __func__);
		BUG();
		return -ENODEV;
	}

	return 0;
}

static int panel_sleep_in(struct panel_device *panel)
{
	int ret = 0;
	struct panel_state *state = &panel->state;

	if (state->connect_panel == PANEL_DISCONNECT) {
		panel_warn("PANEL:WANR:%s:panel disconnected\n", __func__);
		goto do_exit;
	}
	switch(state->cur_state) {
		case PANEL_STATE_ON:
			panel_warn("PANEL:WANR:%s:panel state is aready on : %d\n",
				__func__, state->cur_state);
			goto do_exit;
		case PANEL_STATE_NORMAL:
		case PANEL_STATE_ALPM:
			mdnie_disable(&panel->mdnie);
			ret = panel_display_off(panel);
			if (unlikely(ret < 0)) {
				panel_err("PANEL:ERR:%s, failed to write display_off seqtbl\n",
					__func__);
			}
			ret = __panel_seq_exit(panel);
			if (unlikely(ret < 0)) {
				panel_err("PANEL:ERR:%s, failed to write exit seqtbl\n",
					__func__);
			}
			copr_disable(&panel->copr);
			break;
		default:
			panel_err("PANEL:ERR:%s:invalid state : %d\n",
				__func__,state->cur_state);
			goto do_exit;
	}

	state->cur_state = PANEL_STATE_ON;
	return 0;

do_exit:
	return ret;
}

static int panel_power_on(struct panel_device *panel)
{
	int ret = 0;
	struct panel_state *state = &panel->state;

	if (state->connect_panel == PANEL_DISCONNECT) {
		panel_warn("PANEL:WANR:%s:panel disconnected\n", __func__);
		goto do_exit;
	}

	if (state->cur_state == PANEL_STATE_OFF) {
		ret = __set_panel_power(panel, PANEL_POWER_ON);
		if (ret) {
			panel_err("PANEL:ERR:%s:failed to panel power on\n",
				__func__);
			goto do_exit;
		}
		state->cur_state = PANEL_STATE_ON;
	}
	return 0;

do_exit:
	return ret;
}


static int panel_power_off(struct panel_device *panel)
{
	int ret = -EINVAL;
	struct panel_state *state = &panel->state;
	struct act_clock_dev *act_clk;
	if (state->connect_panel == PANEL_DISCONNECT) {
		panel_warn("PANEL:WANR:%s:panel disconnected\n", __func__);
		goto do_exit;
	}

	switch (state->cur_state)  {
		case PANEL_STATE_OFF:
			panel_warn("PANEL:WANR:%s:panel state is already off: %d\n",
				__func__, state->cur_state);
			goto do_exit;
		case PANEL_STATE_ALPM:
		case PANEL_STATE_NORMAL:
			panel_sleep_in(panel);
			fallthrough;
		case PANEL_STATE_ON:
			ret = __set_panel_power(panel, PANEL_POWER_OFF);
			if (ret) {
				panel_err("PANEL:ERR:%s:failed to panel power off\n",
					__func__);
				goto do_exit;
			}
			break;
		default:
			panel_err("PANEL:ERR:%s:invalid state : %d\n",
				__func__, state->cur_state);
			goto do_exit;
	}

	state->cur_state = PANEL_STATE_OFF;
    /*clock image for active clock on ddi side ram was remove when ddi was turned off */
	act_clk = &panel->act_clk_dev;
	act_clk->act_info.update_img = IMG_UPDATE_NEED;
	return 0;

do_exit:
	return ret;

}

static int panel_sleep_out(struct panel_device *panel)
{
	int ret = 0, retry = 30;
	struct panel_state *state = &panel->state;

	if (state->connect_panel == PANEL_DISCONNECT) {
		panel_warn("PANEL:WANR:%s:panel disconnected\n", __func__);
		goto do_exit;
	}

retry_sleep_out:
	switch (state->cur_state) {
		case PANEL_STATE_NORMAL:
			panel_warn("PANEL:WARN:%s:disp det already normal\n",
				__func__);
			goto do_exit;
		case PANEL_STATE_ALPM:
			ret = __panel_seq_exit_alpm(panel);
			if (ret) {
				panel_err("PANEL:ERR:%s:failed to panel exit alpm\n",
					__func__);
				goto do_exit;
			}
			break;
		case PANEL_STATE_OFF:
			ret = panel_power_on(panel);
			if (ret) {
				panel_err("PANEL:ERR:%s:failed to power on\n", __func__);
				goto do_exit;
			}
			fallthrough;
		case PANEL_STATE_ON:
			ret = __panel_seq_init(panel);
			if (ret) {
				if (ret == -EAGAIN && retry--) {
					panel_power_off(panel);
					usleep_range(100000, 100000);
					goto retry_sleep_out;
				} else {
					panel_err("PANEL:ERR:%s:failed to panel init seq\n",
						__func__);
					goto do_exit;
				}
			}
			break;
		default:
			panel_warn("PANEL:WARN:%s:invalid cur state : %d\n",
				__func__, state->cur_state);
			goto do_exit;
	}
	state->cur_state = PANEL_STATE_NORMAL;
	panel->ktime_panel_on = ktime_get();
	if (state->hmd_on == PANEL_HMD_ON) {
		panel_info("PANEK:INFO:%s:hmd was on, setting hmd on seq\n", __func__);
		ret = __panel_seq_hmd_on(panel);
		if (ret) {
			panel_err("PANEL:ERR:%s:failed to set hmd on seq\n",
				__func__);
		}
	}
	return 0;

do_exit:
	return ret;
}


static int panel_update_alpm(struct panel_device *panel)
{
	int ret = 0;
	struct panel_properties *props = &panel->panel_data.props;
	struct panel_bl_device *panel_bl = &panel->panel_bl;

	mutex_lock(&panel_bl->lock);
	if (props->cur_alpm_mode != panel_bl_get_lpm_mode(panel)) {
		ret = __panel_seq_set_alpm(panel);
		if (ret) {
			panel_err("PANEL:ERR:%s, failed to write alpm\n",
				__func__);
			props->cur_alpm_mode = ALPM_OFF;
		}
	}
	mutex_unlock(&panel_bl->lock);
	return ret;
}

static int panel_doze(struct panel_device *panel)
{
	int ret = 0;
	struct panel_state *state = &panel->state;
	struct panel_bl_device *panel_bl = &panel->panel_bl;

	if (state->connect_panel == PANEL_DISCONNECT) {
		panel_warn("PANEL:WANR:%s:panel disconnected\n", __func__);
		goto do_exit;
	}

	switch(state->cur_state) {
		case PANEL_STATE_ALPM:
			ret = panel_update_alpm(panel);
			if (ret) {
				panel_err("PANEL:ERR:%s:failed to update alpm mode\n",
					__func__);
				goto do_exit;
			}
			break;
		case PANEL_POWER_ON:
		case PANEL_POWER_OFF:
			ret = panel_sleep_out(panel);
			if (ret) {
				panel_err("PANEL:ERR:%s:failed to set normal\n", __func__);
				goto do_exit;
			}
			fallthrough;
		case PANEL_STATE_NORMAL:
			ret = __panel_seq_active_clock(panel, 1);
			if (ret) {
				panel_err("PANEL:ERR:%s, failed to write active seqtbl\n",
				__func__);
			}
			mutex_lock(&panel_bl->lock);
			ret = __panel_seq_set_alpm(panel);
			if (ret) {
				panel_err("PANEL:ERR:%s, failed to write alpm\n",
					__func__);
			}
			state->cur_state = PANEL_STATE_ALPM;
			mutex_unlock(&panel_bl->lock);
			break;
		default:
			break;
	}
	panel_info("PANEL:INFO:%s:was called\n", __func__);

do_exit:
	return ret;
}





static int panel_set_dsu(struct panel_device *panel, struct dsu_info *dsu)
{
	int ret = 0;
	int xres, yres;
	int actual_mode;
	struct decon_lcd *lcd_info;
	struct dsu_info_dt *dt_dsu_info;

	if (unlikely(!panel)) {
		panel_err("PANEL:ERR:%s:panel is null\n", __func__);
		ret = -EINVAL;
		goto do_exit;
	}
	if (unlikely(!dsu)) {
		panel_err("PANEL:ERR:%s:dsu is null\n", __func__);
		ret = -EINVAL;
		goto do_exit;
	}

	lcd_info = &panel->lcd_info;
	dt_dsu_info = &lcd_info->dt_dsu_info;

	if (dt_dsu_info->dsu_en == 0) {
		panel_err("PANEL:ERR:%s:this panel does not support dsu\n", __func__);
		ret = -EINVAL;
		goto do_exit;
	}

	if (dsu->right > dsu->left)
		xres = dsu->right - dsu->left;
	else
		xres = dsu->left - dsu->right;

	if (dsu->bottom > dsu->top)
		yres = dsu->bottom - dsu->top;
	else
		yres = dsu->top - dsu->bottom;

	panel_info("dsu mode : %d, %d:%d-%d:%d", dsu->mode,
		dsu->left, dsu->top, dsu->right, dsu->bottom);
	panel_info("dsu_mode : xres : %d, yres : %d\n", xres, yres);

	actual_mode = dsu->mode - DSU_MODE_1;
	if (actual_mode < 0 || actual_mode >= dt_dsu_info->dsu_number) {
		panel_err("PANEL:ERR:%s:Wrong actual mode : %d , number : %d\n",
			__func__, actual_mode, dt_dsu_info->dsu_number);
		actual_mode = 0;
	}
	lcd_info->xres = xres;
	lcd_info->yres = yres;
	lcd_info->dsu_mode = dsu->mode;

	lcd_info->dsc_enabled = dt_dsu_info->res_info[actual_mode].dsc_en;
	lcd_info->dsc_slice_h = dt_dsu_info->res_info[actual_mode].dsc_height;

	ret = panel_do_seqtbl_by_index(panel, PANEL_DSU_SEQ);
	if (unlikely(ret < 0)) {
		panel_err("PANEL:ERR:%s, failed to write init seqtbl\n", __func__);
	}

	return 0;

do_exit:
	return ret;
}

static int panel_ioctl_display_on(struct panel_device *panel, void *arg)
{
	int ret = 0;
	int *disp_on = (int *)arg;

	if (disp_on == NULL) {
		panel_err("PANEL:ERR:%s:invalid arg\n", __func__);
		return -EINVAL;
	}
	if (*disp_on == 0)
		ret = panel_display_off(panel);
	else
		ret = panel_display_on(panel);

	return ret;
}

static int panel_ioctl_set_power(struct panel_device *panel, void *arg)
{
	int ret = 0;
	int *disp_on = (int *)arg;

	if (disp_on == NULL) {
		panel_err("PANEL:ERR:%s:invalid arg\n", __func__);
		return -EINVAL;
	}
	if (*disp_on == 0)
		ret = panel_power_off(panel);
	else
		ret = panel_power_on(panel);

	return ret;
}

/* the vendor panel_core_ioctl(), without the v4l2 subdev */
long panel_drv_ioctl(struct panel_device *panel, unsigned int cmd, void *arg)
{
	int ret = 0;

	mutex_lock(&panel->io_lock);

	switch(cmd) {
		case PANEL_IOC_PANEL_PROBE:
			panel_info("PANEL:INFO:%s:PANEL_IOC_PANEL_PROBE\n", __func__);
			ret = panel_probe(panel);
			break;

		case PANEL_IOC_SLEEP_IN:
			panel_info("PANEL:INFO:%s:PANEL_IOC_SLEEP_IN\n", __func__);
			ret = panel_sleep_in(panel);
			break;

		case PANEL_IOC_SLEEP_OUT:
			panel_info("PANEL:INFO:%s:PANEL_IOC_SLEEP_OUT\n", __func__);
			ret = panel_sleep_out(panel);
			break;

		case PANEL_IOC_SET_POWER:
			panel_info("PANEL:INFO:%s:PANEL_IOC_SET_POWER\n", __func__);
			ret = panel_ioctl_set_power(panel, arg);
			break;

		case PANEL_IOC_PANEL_DUMP :
			panel_info("PANEL:INFO:%s:PANEL_IOC_PANEL_DUMP\n", __func__);
			ret = panel_debug_dump(panel);
			break;
		case PANEL_IOC_DOZE:
			panel_info("PANEL:INFO:%s:PANEL_IOC_DOZE\n", __func__);
			ret = panel_doze(panel);
			mdnie_update(&panel->mdnie);
			break;
		case PANEL_IOC_SET_DSU:
			ret = panel_set_dsu(panel, arg);
			break;
		case PANEL_IOC_DISP_ON:
			panel_info("PANEL:INFO:%s:PANEL_IOC_DISP_ON\n", __func__);
			ret = panel_ioctl_display_on(panel, arg);
			break;

		case PANEL_IOC_EVT_FRAME_DONE:
			//panel_info("PANEL:INFO:%s:PANEL_IOC_EVT_FRAME_DONE\n", __func__);
			copr_update_start(&panel->copr, 3);
			break;
		case PANEL_IOC_EVT_VSYNC:
			//panel_dbg("PANEL:INFO:%s:PANEL_IOC_EVT_VSYNC\n", __func__);
			break;
		default:
			panel_err("PANEL:ERR:%s:undefined command\n", __func__);
			ret = -EINVAL;
			break;

	}

	if (ret) {
		panel_err("PANEL:ERR:%s:failed to ioctl panel cmd : %d\n",
			__func__,  _IOC_NR(cmd));
	}
	mutex_unlock(&panel->io_lock);

	return (long)ret;
}

/*
 * PANEL_IOC_EVT_FRAME_DONE from the frame-done interrupt: copr_update_start()
 * does not sleep, so it is called without io_lock.
 */
void panel_drv_frame_done(struct panel_device *panel)
{
	copr_update_start(&panel->copr, 3);
}

/* panel-lookup of the vendor device tree: panel names, (id, mask, index) */
static const char * const panel_great_names[] = {
	"s6e3ha6_great_a2_s3", "s6e3ha6_great_a3_sa",
};

static const struct panel_lut panel_great_lut[] = {
	{ 0x806143, 0xf0ffff, 0 },
	{ 0x000000, 0x400000, 0 },
	{ 0xc00140, 0xf00fff, 1 },
	{ 0x400000, 0x400000, 1 },
	{ 0x000000, 0x000000, 0 },
};

/*
 * The vendor panel_drv_probe() for the DRM panel, which owns the pads and passes
 * the reset (init_at_boot) and disp-det (connected) levels it read at probe.
 */
int panel_drv_init(struct panel_device *panel, struct device *dev,
		   bool init_at_boot, bool connected)
{
	struct panel_lut_info *lut_info = &panel->panel_data.lut_info;
	struct decon_lcd *lcd_info = &panel->lcd_info;
	int i, ret;

	panel->dev = dev;

	panel->state.init_at = PANEL_INIT_BOOT;
	panel->state.connect_panel = PANEL_CONNECT;
	panel->state.cur_state = PANEL_STATE_OFF;
	panel->state.power = PANEL_POWER_OFF;
	panel->state.disp_on = PANEL_DISPLAY_OFF;
	panel->ktime_panel_on = ktime_get();
	panel->state.hmd_on = PANEL_HMD_OFF;

	panel->lcd_info.dsu_mode = DSU_MODE_1;
	panel->act_clk_dev.act_info.update_img = IMG_UPDATE_NEED;
	mutex_init(&panel->op_lock);
	mutex_init(&panel->data_lock);
	mutex_init(&panel->io_lock);
	mutex_init(&panel->panel_bl.lock);
	mutex_init(&panel->mdnie.lock);
	mutex_init(&panel->copr.lock);

	panel->id = 0;

	if (init_at_boot) {
		panel->state.init_at = PANEL_INIT_BOOT;
		if (!connected) {
			panel->state.connect_panel = PANEL_DISCONNECT;
		} else {
			panel->state.connect_panel = PANEL_CONNECT;
			panel->state.cur_state = PANEL_STATE_NORMAL;
			panel->state.power = PANEL_POWER_ON;
			panel->state.disp_on = PANEL_DISPLAY_ON;
		}
	} else {
		panel->state.init_at = PANEL_INIT_KERNEL;
	}

	if (panel->state.init_at == PANEL_INIT_BOOT) {
		ret = regulator_set_voltage(panel->pad.regulator[REGULATOR_1p6V],
					    1600000, 1600000);
		if (ret)
			panel_err("PANEL:%s:failed to set volatege\n", __func__);
	}

	/* ddi_info of the vendor device tree */
	lcd_info->ldi_name = "s6e3ha6";
	lcd_info->xres = 1440;
	lcd_info->yres = 2960;
	lcd_info->dsc_enabled = 1;
	lcd_info->dsc_cnt = 2;
	lcd_info->dsc_slice_num = 2;
	lcd_info->dsc_slice_h = 40;

	for (i = 0; i < ARRAY_SIZE(panel_great_names); i++)
		lut_info->names[i] = panel_great_names[i];
	lut_info->nr_panel = ARRAY_SIZE(panel_great_names);
	for (i = 0; i < ARRAY_SIZE(panel_great_lut); i++)
		lut_info->lut[i] = panel_great_lut[i];
	lut_info->nr_lut = ARRAY_SIZE(panel_great_lut);
	print_panel_lut(lut_info);

	return 0;
}
