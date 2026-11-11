// SPDX-License-Identifier: GPL-2.0
// Samsung S6SY761 Touchscreen device driver
//
// Copyright (c) 2017 Samsung Electronics Co., Ltd.
// Copyright (c) 2017 Andi Shyti <andi@etezian.org>

#include <linux/unaligned.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/pm_wakeup.h>
#include <linux/regulator/consumer.h>
#include <linux/sec_sysfs.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>
#include <drm/drm_panel.h>

/* commands */
#define S6SY761_SENSE_ON		0x10
#define S6SY761_SENSE_OFF		0x11
#define S6SY761_TOUCH_FUNCTION		0x30 /* R/W for get/set */
#define S6SY761_FIRMWARE_INTEGRITY	0x21
#define S6SY761_PANEL_INFO		0x23
#define S6SY761_DEVICE_ID		0x52
#define S6SY761_BOOT_STATUS		0x55
#define S6SY761_READ_ONE_EVENT		0x60
#define S6SY761_READ_ALL_EVENT		0x61
#define S6SY761_CLEAR_EVENT_STACK	0x62
#define S6SY761_APPLICATION_MODE	0xe4

/* events */
#define S6SY761_EVENT_INFO		0x02
#define S6SY761_EVENT_VENDOR_INFO	0x07

/* info */
#define S6SY761_INFO_BOOT_COMPLETE	0x00

/* firmware status */
#define S6SY761_FW_OK			0x80

/*
 * the functionalities are put as a reference
 * as in the device I am using none of them
 * works therefore not used in this driver yet.
 */
/* touchscreen functionalities */
#define S6SY761_MASK_TOUCH		BIT(0)
#define S6SY761_MASK_HOVER		BIT(1)
#define S6SY761_MASK_COVER		BIT(2)
#define S6SY761_MASK_GLOVE		BIT(3)
#define S6SY761_MASK_STYLUS		BIT(4)
#define S6SY761_MASK_PALM		BIT(5)
#define S6SY761_MASK_WET		BIT(6)
#define S6SY761_MASK_PROXIMITY		BIT(7)

/* boot status (BS) */
#define S6SY761_BS_BOOT_LOADER		0x10
#define S6SY761_BS_APPLICATION		0x20

/* event id */
#define S6SY761_EVENT_ID_COORDINATE	0x00
#define S6SY761_EVENT_ID_STATUS		0x01

/* event register masks */
#define S6SY761_MASK_TOUCH_STATE	0xc0 /* byte 0 */
#define S6SY761_MASK_TID		0x3c
#define S6SY761_MASK_EID		0x03
#define S6SY761_MASK_X			0xf0 /* byte 3 */
#define S6SY761_MASK_Y			0x0f
#define S6SY761_MASK_Z			0x3f /* byte 6 */
#define S6SY761_MASK_LEFT_EVENTS	0x3f /* byte 7 */
#define S6SY761_MASK_TOUCH_TYPE		0xc0 /* MSB in byte 6, LSB in byte 7 */

/* event touch state values */
#define S6SY761_TS_NONE			0x00
#define S6SY761_TS_PRESS		0x01
#define S6SY761_TS_MOVE			0x02
#define S6SY761_TS_RELEASE		0x03

/* application modes */
#define S6SY761_APP_NORMAL		0x0
#define S6SY761_APP_LOW_POWER		0x1
#define S6SY761_APP_TEST		0x2
#define S6SY761_APP_FLASH		0x3
#define S6SY761_APP_SLEEP		0x4

#define S6SY761_EVENT_SIZE		8
#define S6SY761_EVENT_COUNT		32
#define S6SY761_DEVID_SIZE		3
#define S6SY761_PANEL_ID_SIZE		11
#define S6SY761_TS_STATUS_SIZE		5
#define S6SY761_MAX_FINGERS		10

#define S6SY761_DEV_NAME	"s6sy761"

/*
 * S6SY661 (Galaxy Note 8, stock driver sec_ts_y661): the S6SY761 protocol (event record
 * S:drivers/input/touchscreen/sec_ts_y661/sec_ts.h:613-627, commands :174-255, boot status
 * :282) with the stock power sequence and defaults below.
 */
#define S6SY661_DEV_NAME		"sec_touchscreen"	/* S:.../sec_ts_y661/sec_ts.c:2301 */
#define S6SY661_MAX_SLOTS		10	/* MAX_SUPPORT_TOUCH_COUNT, sec_ts.h:112; sec_ts.c:2070 */
#define S6SY661_WAIT_RETRY_CNT		100	/* SEC_TS_WAIT_RETRY_CNT, sec_ts.h:483 */
/* SEC_TS_DEFAULT_ENABLE_BIT_SETFUNC = TOUCH | PALM | WET (bits 0, 5, 6), sec_ts.h:339-349 */
#define S6SY661_FUNC_DEFAULT		0x0061
#define S6SY661_FUNC_GLOVE		BIT(3)	/* SEC_TS_BIT_SETFUNC_GLOVE, sec_ts.h:343 */
/* touch types reported as a finger, sec_ts.c:1190-1193 (SEC_TS_TOUCHTYPE_*, sec_ts.h:304-310) */
#define S6SY661_TT_NORMAL		0
#define S6SY661_TT_GLOVE		3
#define S6SY661_TT_PALM			5
#define S6SY661_TT_WET			6
/*
 * The under-display pressure home key. The touch firmware's "sponge" library reports a hard
 * press on the lower screen; stock calls it SPONGE FORCE_KEY. Paths are S:.../sec_ts_y661/.
 */
#define S6SY661_SPONGE_GET_INFO		0x90	/* SEC_TS_CMD_SPONGE_GET_INFO, sec_ts.h:217 */
#define S6SY661_SPONGE_WRITE_PARAM	0x91	/* sec_ts.h:218 */
#define S6SY661_SPONGE_NOTIFY_PACKET	0x93	/* sec_ts.h:220 */
#define S6SY661_SPONGE_INFO_SIZE	10	/* u8 data[10], sec_ts.c:1993-1998 */
#define S6SY661_SPONGE_MODEL_LEN	4	/* strncmp(data, model_name, 4), sec_ts.c:2009 */
#define S6SY661_MODE_AOD		BIT(2)	/* SEC_TS_MODE_SPONGE_AOD, sec_ts.h:486 */
#define S6SY661_MODE_SINGLE_TAP		BIT(3)	/* SEC_TS_MODE_SPONGE_SINGLE_TAP, sec_ts.h:487 */
#define S6SY661_MODE_FORCE_KEY		BIT(6)	/* SEC_TS_MODE_SPONGE_FORCE_KEY, sec_ts.h:488 */
/* SEC_TS_MODE_LOWPOWER_FLAG (sec_ts.h:490-491) without SPAY (Samsung Pay), not ported */
#define S6SY661_MODE_LOWPOWER_FLAG	(S6SY661_MODE_AOD | S6SY661_MODE_SINGLE_TAP | \
					 S6SY661_MODE_FORCE_KEY)
/* the AOD rectangle: sponge offset 2, then w, h, x, y as le16 (set_aod_rect(), sec_ts_fn.c:5131-5172) */
#define S6SY661_SPONGE_OFFSET_AOD_RECT	0x02
/* gesture event (SEC_TS_GESTURE_EVENT, sec_ts.h:290; struct sec_ts_gesture_status, :583-592) */
#define S6SY661_EVENT_ID_GESTURE	0x02
#define S6SY661_GESTURE_DOUBLE_TAP	0x01	/* SEC_TS_GESTURE_CODE_DOUBLE_TAP, sec_ts.h:296 */
#define S6SY661_GESTURE_SINGLE_TAP	0x02	/* SEC_TS_GESTURE_CODE_SINGLE_TAP, sec_ts.h:297 */
/* Samsung's KEY_BLACK_UI_GESTURE, S:include/uapi/linux/input-event-codes.h:525; free upstream */
#define S6SY661_KEY_BLACK_UI_GESTURE	0x1c7
#define S6SY661_WAKEUP_GESTURE_MODE	0x39	/* SEC_TS_CMD_WAKEUP_GESTURE_MODE, sec_ts.h:195 */
#define S6SY661_PM_DELAY_MS		50	/* sec_ts_delay(50), sec_ts.c:2662 */
#define S6SY661_PM_TRIES		5	/* retrycnt < 5, sec_ts.c:2670-2674 */
#define S6SY661_LPM_WAKE_MS		(3 * MSEC_PER_SEC)	/* wake_lock_timeout, sec_ts.c:951-953 */
/* status event type SPONGE_INFO (sec_ts.h:100) in byte 0 [5:2], id FORCE_KEY (sec_ts.h:328) */
#define S6SY661_MASK_STYPE		0x3c
#define S6SY661_STATUS_SPONGE_INFO	6
#define S6SY661_SPONGE_FORCE_KEY	0x00
/* status_data_1 (byte 2), sec_ts.h:497-498 */
#define S6SY661_SPONGE_TOUCHED		BIT(6)
#define S6SY661_SPONGE_RELEASED		BIT(7)
/* the sec/tsp command node, S:drivers/input/sec_cmd.c, S:include/linux/input/sec_cmd.h */
#define S6SY661_CMD_STR_LEN		256	/* SEC_CMD_STR_LEN, sec_cmd.h:37 */
#define S6SY661_CMD_PARAM_NUM		8	/* SEC_CMD_PARAM_NUM, sec_cmd.h:39 */

/*
 * Stock tries every transfer up to SEC_TS_I2C_RETRY_CNT times, 1 ms apart (sec_ts_i2c_write(),
 * sec_ts_i2c_read(), sec_ts.c:409-470, 484-541; sec_ts.h:482). The chip NACKs the transfer that
 * wakes it from the low-power mode, so the transfers made with the chip in that mode, or right
 * after switching it there, are retried as stock does.
 */
#define S6SY661_I2C_TRIES		3
#define s6sy661_i2c(expr) ({						\
	int __ret, __try = 0;						\
									\
	while ((__ret = (expr)) < 0 && ++__try < S6SY661_I2C_TRIES)	\
		usleep_range(1000, 1100);				\
	__ret;								\
})

enum s6sy761_regulators {
	S6SY761_REGULATOR_VDD,
	S6SY761_REGULATOR_AVDD,
};

struct s6sy761_variant {
	const char *name;
	bool s6sy661;
};

static const struct s6sy761_variant s6sy761_variant = {
	.name = S6SY761_DEV_NAME,
};

static const struct s6sy761_variant s6sy661_variant = {
	.name = S6SY661_DEV_NAME,
	.s6sy661 = true,
};

struct s6sy761_data {
	const struct s6sy761_variant *variant;
	struct i2c_client *client;
	struct regulator_bulk_data regulators[2];
	struct input_dev *input;
	struct touchscreen_properties prop;

	u8 data[S6SY761_EVENT_SIZE * S6SY761_EVENT_COUNT];

	u16 devid;
	u8 tx_channel;

	/* S6SY661 only */
	struct mutex power_lock;	/* powered, the rails, the IRQ disable depth */
	bool powered;
	u16 touch_functions;		/* TOUCH_FUNCTION word, rewritten at every power-on */
	bool pressure_key;		/* the firmware has the sponge library: home key reported */
	u8 lowpower_mode;		/* sponge mode byte, stock ts->lowpower_mode */
	u16 aod_rect[4];		/* w, h, x, y, stock ts->rect_data */
	bool lpm;			/* in the low-power mode; written under power_lock */
	bool irq_wake;			/* enable_irq_wake() is in effect */
	bool home_key_down;		/* IRQ thread only */
	bool wake_key_down;		/* IRQ thread only */
	struct drm_panel_follower follower;
	struct work_struct power_on_work;
	struct mutex cmd_lock;
	bool cmd_running;
	const char *cmd_state;
	char cmd[S6SY661_CMD_STR_LEN];
	char cmd_result[S6SY661_CMD_STR_LEN + 16];
};

/*
 * We can't simply use i2c_smbus_read_i2c_block_data because we
 * need to read more than 255 bytes
 */
static int s6sy761_read_events(struct s6sy761_data *sdata, u16 n_events)
{
	u8 cmd = S6SY761_READ_ALL_EVENT;
	struct i2c_msg msgs[2] = {
		{
			.addr	= sdata->client->addr,
			.len	= 1,
			.buf	= &cmd,
		},
		{
			.addr	= sdata->client->addr,
			.flags	= I2C_M_RD,
			.len	= (n_events * S6SY761_EVENT_SIZE),
			.buf	= sdata->data + S6SY761_EVENT_SIZE,
		},
	};
	int ret;

	ret = s6sy661_i2c(i2c_transfer(sdata->client->adapter, msgs, ARRAY_SIZE(msgs)));
	if (ret < 0)
		return ret;

	return ret == ARRAY_SIZE(msgs) ? 0 : -EIO;
}

/*
 * End of a report. The S6SY661 variant also emits BTN_TOUCH (with the pointer emulation of
 * INPUT_MT_DIRECT) through input_mt_sync_frame(): stock reports BTN_TOUCH 1 on every press and
 * 0 when the last finger is released (S:drivers/input/touchscreen/sec_ts_y661/sec_ts.c:1221,
 * 1273,1324), and input_mt_init_slots(INPUT_MT_DIRECT) advertises BTN_TOUCH
 * (A:drivers/input/input-mt.c:62-64), so without it Android's multi-touch mapper sees BTN_TOUCH
 * supported but never pressed and reads every contact as a hover.
 */
static void s6sy761_sync(struct s6sy761_data *sdata)
{
	if (sdata->variant->s6sy661)
		input_mt_sync_frame(sdata->input);
	input_sync(sdata->input);
}

static void s6sy761_report_coordinates(struct s6sy761_data *sdata,
				       u8 *event, u8 tid)
{
	u8 major = event[4];
	u8 minor = event[5];
	u8 z = event[6] & S6SY761_MASK_Z;
	u16 x = (event[1] << 4) | ((event[3] & S6SY761_MASK_X) >> 4);
	u16 y = (event[2] << 4) | (event[3] & S6SY761_MASK_Y);

	input_mt_slot(sdata->input, tid);

	input_mt_report_slot_state(sdata->input, MT_TOOL_FINGER, true);
	input_report_abs(sdata->input, ABS_MT_POSITION_X, x);
	input_report_abs(sdata->input, ABS_MT_POSITION_Y, y);
	input_report_abs(sdata->input, ABS_MT_TOUCH_MAJOR, major);
	input_report_abs(sdata->input, ABS_MT_TOUCH_MINOR, minor);
	input_report_abs(sdata->input, ABS_MT_PRESSURE, z);

	s6sy761_sync(sdata);
}

static void s6sy761_report_release(struct s6sy761_data *sdata,
				   u8 *event, u8 tid)
{
	input_mt_slot(sdata->input, tid);
	input_mt_report_slot_state(sdata->input, MT_TOOL_FINGER, false);

	s6sy761_sync(sdata);
}

/*
 * Status event SPONGE_INFO / FORCE_KEY: a press (bit 6) or release (bit 7) of the pressure home
 * key, stock sec_ts.c:1070-1118. With the screen off (low-power mode) stock treats everything
 * but a release as a press and reports KEY_HOMEPAGE (:1089-1107); Android's default key layout
 * does not wake on KEY_HOMEPAGE, so this port reports KEY_WAKEUP, which it does, and it is the
 * only way to wake a phone whose power button is broken. With the screen on stock reports the
 * Samsung-only KEY_BLACK_UI_GESTURE (:1072-1087); this port reports KEY_HOMEPAGE for the press
 * and the release. A release releases whichever key is down, as stock's does for a press that
 * started in the low-power mode and ends after the screen came on (:1074-1080).
 */
static void s6sy661_handle_pressure(struct s6sy761_data *sdata, u8 *event)
{
	bool lpm = READ_ONCE(sdata->lpm);
	bool down;

	if (((event[0] & S6SY661_MASK_STYPE) >> 2) != S6SY661_STATUS_SPONGE_INFO ||
	    event[1] != S6SY661_SPONGE_FORCE_KEY)
		return;

	down = lpm ? !(event[2] & S6SY661_SPONGE_RELEASED) : event[2] & S6SY661_SPONGE_TOUCHED;
	if (down) {
		if (lpm) {
			sdata->wake_key_down = true;
			input_report_key(sdata->input, KEY_WAKEUP, 1);
		} else {
			sdata->home_key_down = true;
			input_report_key(sdata->input, KEY_HOMEPAGE, 1);
		}
	} else {
		sdata->wake_key_down = false;
		sdata->home_key_down = false;
		input_report_key(sdata->input, KEY_WAKEUP, 0);
		input_report_key(sdata->input, KEY_HOMEPAGE, 0);
	}
	input_sync(sdata->input);
}

/*
 * Gesture event, stock sec_ts.c:1367-1425: with the AOD or single-tap sponge mode on, the
 * firmware reports a double tap (in the AOD rectangle) or a single tap as gesture id 1 or 2
 * (subtype 0). Stock reads the sponge mode byte back, picks AOD double tap over single tap
 * when both are on, and reports KEY_BLACK_UI_GESTURE with the kind in sysfs scrub_pos for the
 * Samsung framework. This port takes the kind from the gesture id, so both can be on at once,
 * and reports:
 * - a double tap as KEY_WAKEUP, which Android wakes on, as for the pressure key (the
 *   LineageOS power HAL turns it on for double-tap-to-wake with aod_enable/set_aod_rect);
 * - a single tap as KEY_BLACK_UI_GESTURE, the key the LineageOS Samsung touch HAL gives its
 *   "Single Tap" touchscreen gesture (hardware/samsung aidl/touch/TouchscreenGesture.cpp:25-26),
 *   which it turns on with singletap_enable.
 */
static void s6sy661_handle_gesture(struct s6sy761_data *sdata, u8 *event)
{
	u8 mode = READ_ONCE(sdata->lowpower_mode);
	unsigned int key;

	if ((event[0] & S6SY661_MASK_STYPE) >> 2)
		return;

	switch (event[1]) {
	case S6SY661_GESTURE_DOUBLE_TAP:
		if (!(mode & S6SY661_MODE_AOD))
			return;
		key = KEY_WAKEUP;
		break;
	case S6SY661_GESTURE_SINGLE_TAP:
		if (!(mode & S6SY661_MODE_SINGLE_TAP))
			return;
		key = S6SY661_KEY_BLACK_UI_GESTURE;
		break;
	default:
		return;
	}

	input_report_key(sdata->input, key, 1);
	input_sync(sdata->input);
	input_report_key(sdata->input, key, 0);
	input_sync(sdata->input);
}

static void s6sy761_handle_coordinates(struct s6sy761_data *sdata, u8 *event)
{
	u8 tid;
	u8 touch_state;

	if (unlikely(!(event[0] & S6SY761_MASK_TID)))
		return;

	tid = ((event[0] & S6SY761_MASK_TID) >> 2) - 1;
	touch_state = (event[0] & S6SY761_MASK_TOUCH_STATE) >> 6;

	if (sdata->variant->s6sy661) {
		/* ttype_3_2 = byte 6 [7:6], ttype_1_0 = byte 7 [7:6] (sec_ts.h:623-626); stock
		 * reports press, move and release only for these types (sec_ts.c:1190-1193) */
		u8 ttype = (event[6] >> 6) << 2 | event[7] >> 6;

		if (ttype != S6SY661_TT_NORMAL && ttype != S6SY661_TT_GLOVE &&
		    ttype != S6SY661_TT_PALM && ttype != S6SY661_TT_WET)
			return;
	}

	switch (touch_state) {

	case S6SY761_TS_NONE:
		break;
	case S6SY761_TS_RELEASE:
		s6sy761_report_release(sdata, event, tid);
		break;
	case S6SY761_TS_PRESS:
	case S6SY761_TS_MOVE:
		s6sy761_report_coordinates(sdata, event, tid);
		break;
	}
}

static int s6sy661_sponge_write(struct s6sy761_data *sdata);

static void s6sy761_handle_events(struct s6sy761_data *sdata, u8 n_events)
{
	int i;

	for (i = 0; i < n_events; i++) {
		u8 *event = &sdata->data[i * S6SY761_EVENT_SIZE];
		u8 event_id = event[0] & S6SY761_MASK_EID;

		if (!event[0])
			continue;

		switch (event_id) {

		case S6SY761_EVENT_ID_COORDINATE:
			/* sec_ts.c:1135-1138: "device is closed" */
			if (sdata->variant->s6sy661 && READ_ONCE(sdata->lpm))
				break;
			s6sy761_handle_coordinates(sdata, event);
			break;

		case S6SY761_EVENT_ID_STATUS:
			if (sdata->pressure_key)
				s6sy661_handle_pressure(sdata, event);
			break;

		case S6SY661_EVENT_ID_GESTURE:
			if (sdata->pressure_key)
				s6sy661_handle_gesture(sdata, event);
			break;

		default:
			break;
		}
	}
}

static irqreturn_t s6sy761_irq_handler(int irq, void *dev)
{
	struct s6sy761_data *sdata = dev;
	int ret;
	u8 n_events;

	/* keep the system awake while the event is delivered, sec_ts.c:951-953 */
	if (sdata->variant->s6sy661 && READ_ONCE(sdata->lpm))
		pm_wakeup_event(&sdata->client->dev, S6SY661_LPM_WAKE_MS);

	ret = s6sy661_i2c(i2c_smbus_read_i2c_block_data(sdata->client,
							S6SY761_READ_ONE_EVENT,
							S6SY761_EVENT_SIZE,
							sdata->data));
	if (ret < 0) {
		dev_err(&sdata->client->dev, "failed to read events\n");
		return IRQ_HANDLED;
	}

	if (!sdata->data[0])
		return IRQ_HANDLED;

	n_events = sdata->data[7] & S6SY761_MASK_LEFT_EVENTS;
	if (unlikely(n_events > S6SY761_EVENT_COUNT - 1)) {
		/* event buffer overflow: clear event stack, stock sec_ts.c:993-1001 */
		if (sdata->variant->s6sy661)
			s6sy661_i2c(i2c_smbus_write_byte(sdata->client, S6SY761_CLEAR_EVENT_STACK));
		return IRQ_HANDLED;
	}

	if (n_events) {
		ret = s6sy761_read_events(sdata, n_events);
		if (ret < 0) {
			dev_err(&sdata->client->dev, "failed to read events\n");
			return IRQ_HANDLED;
		}
	}

	s6sy761_handle_events(sdata, n_events +  1);

	return IRQ_HANDLED;
}

static int s6sy761_input_open(struct input_dev *dev)
{
	struct s6sy761_data *sdata = input_get_drvdata(dev);
	int ret;

	/* stock sec_ts_start_device(): sec_ts_set_custom_library() before SENSE_ON, sec_ts.c:2961,3000 */
	if (sdata->variant->s6sy661 && sdata->pressure_key) {
		mutex_lock(&sdata->power_lock);
		if (sdata->powered && !sdata->lpm) {
			ret = s6sy661_sponge_write(sdata);
			if (ret)
				dev_warn(&sdata->client->dev, "failed to configure the pressure key\n");
		}
		mutex_unlock(&sdata->power_lock);
	}

	return i2c_smbus_write_byte(sdata->client, S6SY761_SENSE_ON);
}

static void s6sy761_input_close(struct input_dev *dev)
{
	struct s6sy761_data *sdata = input_get_drvdata(dev);
	int ret;

	ret = i2c_smbus_write_byte(sdata->client, S6SY761_SENSE_OFF);
	if (ret)
		dev_err(&sdata->client->dev, "failed to turn off sensing\n");
}

static ssize_t s6sy761_sysfs_devid(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct s6sy761_data *sdata = dev_get_drvdata(dev);

	return sprintf(buf, "%#x\n", sdata->devid);
}

static DEVICE_ATTR(devid, 0444, s6sy761_sysfs_devid, NULL);

static struct attribute *s6sy761_sysfs_attrs[] = {
	&dev_attr_devid.attr,
	NULL
};
ATTRIBUTE_GROUPS(s6sy761_sysfs);

static int s6sy761_power_on(struct s6sy761_data *sdata)
{
	u8 buffer[S6SY761_EVENT_SIZE];
	u8 event;
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(sdata->regulators),
				    sdata->regulators);
	if (ret)
		return ret;

	msleep(140);

	/* double check whether the touch is functional */
	ret = i2c_smbus_read_i2c_block_data(sdata->client,
					    S6SY761_READ_ONE_EVENT,
					    S6SY761_EVENT_SIZE,
					    buffer);
	if (ret < 0)
		return ret;

	event = (buffer[0] >> 2) & 0xf;

	if ((event != S6SY761_EVENT_INFO &&
	     event != S6SY761_EVENT_VENDOR_INFO) ||
	    buffer[1] != S6SY761_INFO_BOOT_COMPLETE) {
		return -ENODEV;
	}

	ret = i2c_smbus_read_byte_data(sdata->client, S6SY761_BOOT_STATUS);
	if (ret < 0)
		return ret;

	/* for some reasons the device might be stuck in the bootloader */
	if (ret != S6SY761_BS_APPLICATION)
		return -ENODEV;

	/* enable touch functionality */
	ret = i2c_smbus_write_word_data(sdata->client,
					S6SY761_TOUCH_FUNCTION,
					S6SY761_MASK_TOUCH);
	if (ret)
		return ret;

	return 0;
}

/*
 * Tell the sponge library which features to run, stock sec_ts_set_custom_library()
 * (sec_ts.c:1971-1990): three bytes at SPONGE_WRITE_PARAM (offset 0, 0, then the mode byte),
 * then a NOTIFY_PACKET with no data. Stock sends it at every start (sec_ts.c:864-865) and at
 * the end of probe (:2354-2357). Caller holds power_lock or is probe.
 */
static int s6sy661_sponge_write(struct s6sy761_data *sdata)
{
	u8 data[3] = { 0, 0, sdata->lowpower_mode };
	int ret;

	ret = s6sy661_i2c(i2c_smbus_write_i2c_block_data(sdata->client, S6SY661_SPONGE_WRITE_PARAM,
							 sizeof(data), data));
	if (ret)
		return ret;

	return s6sy661_i2c(i2c_smbus_write_byte(sdata->client, S6SY661_SPONGE_NOTIFY_PACKET));
}

/*
 * The AOD rectangle a double tap must land in, stock set_aod_rect() (sec_ts_fn.c:5131-5172):
 * ten bytes at SPONGE_WRITE_PARAM (offset 2, 0, then w, h, x, y as le16), then NOTIFY_PACKET.
 * Caller holds power_lock, the chip is powered.
 */
static int s6sy661_sponge_write_rect(struct s6sy761_data *sdata)
{
	u8 data[10] = { S6SY661_SPONGE_OFFSET_AOD_RECT, 0 };
	int i, ret;

	for (i = 0; i < 4; i++)
		put_unaligned_le16(sdata->aod_rect[i], &data[i * 2 + 2]);

	ret = s6sy661_i2c(i2c_smbus_write_i2c_block_data(sdata->client, S6SY661_SPONGE_WRITE_PARAM,
							 sizeof(data), data));
	if (ret)
		return ret;

	return s6sy661_i2c(i2c_smbus_write_byte(sdata->client, S6SY661_SPONGE_NOTIFY_PACKET));
}

/*
 * S6SY661 power-on, stock sec_ts_power(on) + sec_ts_start_device()
 * (S:drivers/input/touchscreen/sec_ts_y661/sec_ts.c:1673-1686, 2911-2923): tsp_io ("vdd")
 * first, 1 ms, tsp_avdd ("avdd"), 70 ms. Only the power-on at probe skips the 70 ms when
 * both rails were already on, as stock does for sec,regulator_boot_on (FDT:5698,
 * sec_ts.c:2204-2206). Then sec_ts_wait_for_ready() (sec_ts.c:776-800): read one event until
 * an INFO or VENDOR_INFO event carries BOOT_COMPLETE, 20 ms apart, stopping at a failed read
 * or after SEC_TS_WAIT_RETRY_CNT + 2 reads; a timeout is not fatal (the return value is
 * ignored at sec_ts.c:2210), BOOT_STATUS decides. Caller holds power_lock or is probe.
 */
static int s6sy661_power_on(struct s6sy761_data *sdata, bool probe)
{
	struct regulator *vdd = sdata->regulators[S6SY761_REGULATOR_VDD].consumer;
	struct regulator *avdd = sdata->regulators[S6SY761_REGULATOR_AVDD].consumer;
	u8 buffer[S6SY761_EVENT_SIZE];
	bool boot_on;
	int retry = 0;
	int ret;

	boot_on = probe && regulator_is_enabled(vdd) > 0 && regulator_is_enabled(avdd) > 0;

	ret = regulator_enable(vdd);
	if (ret)
		return ret;
	usleep_range(1000, 1100);	/* sec_ts_delay(1), sec_ts.c:1680 */
	ret = regulator_enable(avdd);
	if (ret)
		goto err_vdd;
	if (!boot_on)
		msleep(70);		/* sec_ts.c:2922-2923 */

	while (i2c_smbus_read_i2c_block_data(sdata->client, S6SY761_READ_ONE_EVENT,
					     S6SY761_EVENT_SIZE, buffer) > 0) {
		u8 event = (buffer[0] >> 2) & 0xf;

		if ((event == S6SY761_EVENT_INFO || event == S6SY761_EVENT_VENDOR_INFO) &&
		    buffer[1] == S6SY761_INFO_BOOT_COMPLETE)
			break;
		if (retry++ > S6SY661_WAIT_RETRY_CNT)
			break;
		msleep(20);
	}

	ret = i2c_smbus_read_byte_data(sdata->client, S6SY761_BOOT_STATUS);
	if (ret < 0)
		goto err_avdd;
	if (ret != S6SY761_BS_APPLICATION) {
		ret = -ENODEV;
		goto err_avdd;
	}

	/* sec_ts.c:2277-2278: the stored functions, glove bit included (item 12) */
	ret = i2c_smbus_write_word_data(sdata->client, S6SY761_TOUCH_FUNCTION,
					sdata->touch_functions);
	if (ret)
		goto err_avdd;

	if (sdata->pressure_key && s6sy661_sponge_write(sdata))
		dev_warn(&sdata->client->dev, "failed to configure the pressure key\n");

	/* a power cycle loses the rectangle as a reset does; stock writes it again after
	 * one (sec_ts_reinit(), sec_ts.c:883-900) */
	if (sdata->pressure_key && (sdata->lowpower_mode & S6SY661_MODE_AOD) &&
	    s6sy661_sponge_write_rect(sdata))
		dev_warn(&sdata->client->dev, "failed to set the AOD rectangle\n");

	sdata->powered = true;
	return 0;

err_avdd:
	regulator_disable(avdd);
err_vdd:
	regulator_disable(vdd);
	return ret;
}

/*
 * Switch the application mode, stock sec_ts_set_lowpowermode() (sec_ts.c:2632-2712): write
 * SET_POWER_MODE, wait 50 ms, read it back, up to five times until it reads back as written;
 * stock goes on after the last try either way. Caller holds power_lock.
 */
static int s6sy661_set_power_mode(struct s6sy761_data *sdata, u8 mode)
{
	int i, ret;

	for (i = 0; i < S6SY661_PM_TRIES; i++) {
		ret = s6sy661_i2c(i2c_smbus_write_byte_data(sdata->client,
							    S6SY761_APPLICATION_MODE, mode));
		if (ret)
			return ret;
		msleep(S6SY661_PM_DELAY_MS);
		ret = s6sy661_i2c(i2c_smbus_read_byte_data(sdata->client, S6SY761_APPLICATION_MODE));
		if (ret < 0)
			return ret;
		if (ret == mode)
			return 0;
	}
	dev_warn(&sdata->client->dev, "power mode %u reads back as %d\n", mode, ret);

	return 0;
}

/*
 * Screen off with the pressure key enabled: stock does not power the chip down, it enters the
 * low-power mode (sec_ts_input_close(), sec_ts.c:2786-2799, sec_ts_set_lowpowermode(1),
 * :2640-2711): sponge mode byte, WAKEUP_GESTURE_MODE with the mode bits shifted down by one,
 * SET_POWER_MODE 1, arm the IRQ as a wake source, clear the event stack, release all fingers.
 * Caller holds power_lock, the chip is powered.
 */
static int s6sy661_enter_lpm(struct s6sy761_data *sdata)
{
	struct i2c_client *client = sdata->client;
	int i, ret;

	ret = s6sy661_sponge_write(sdata);
	if (ret)
		return ret;

	ret = s6sy661_i2c(i2c_smbus_write_byte_data(client, S6SY661_WAKEUP_GESTURE_MODE,
				(sdata->lowpower_mode & S6SY661_MODE_LOWPOWER_FLAG) >> 1));
	if (ret)
		return ret;

	ret = s6sy661_set_power_mode(sdata, S6SY761_APP_LOW_POWER);
	if (ret)
		return ret;

	if (device_may_wakeup(&client->dev) && !enable_irq_wake(client->irq))
		sdata->irq_wake = true;

	ret = s6sy661_i2c(i2c_smbus_write_byte(client, S6SY761_CLEAR_EVENT_STACK));
	if (ret)
		goto err_wake;

	for (i = 0; i < S6SY661_MAX_SLOTS; i++) {
		input_mt_slot(sdata->input, i);
		input_mt_report_slot_inactive(sdata->input);
	}
	/* sec_ts_locked_release_all_finger() also releases KEY_HOMEPAGE, sec_ts.c:2482,2536 */
	if (sdata->home_key_down || sdata->wake_key_down) {
		input_report_key(sdata->input, KEY_HOMEPAGE, 0);
		input_report_key(sdata->input, KEY_WAKEUP, 0);
		sdata->home_key_down = false;
		sdata->wake_key_down = false;
	}
	s6sy761_sync(sdata);

	WRITE_ONCE(sdata->lpm, true);
	return 0;

err_wake:
	if (sdata->irq_wake) {
		disable_irq_wake(client->irq);
		sdata->irq_wake = false;
	}
	return ret;
}

/* screen on again, stock sec_ts_input_open() (sec_ts.c:2737-2742): SET_POWER_MODE 0 */
static int s6sy661_exit_lpm(struct s6sy761_data *sdata)
{
	int ret;

	ret = s6sy661_set_power_mode(sdata, S6SY761_APP_NORMAL);
	if (ret)
		return ret;

	if (sdata->irq_wake) {
		disable_irq_wake(sdata->client->irq);
		sdata->irq_wake = false;
	}
	WRITE_ONCE(sdata->lpm, false);
	return 0;
}

/*
 * S6SY661 power-off, stock sec_ts_stop_device() (sec_ts.c:2885-2898): IRQ off, every
 * finger released (sec_ts_locked_release_all_finger()), rails off in stock's order
 * tsp_io then tsp_avdd (sec_ts_power(off), sec_ts.c:1688-1689), pins to "sleep" = stock
 * off_state (FDT:275-281). No-op when already off. Caller holds power_lock.
 */
static void s6sy661_power_off(struct s6sy761_data *sdata)
{
	int i;

	if (!sdata->powered)
		return;

	disable_irq(sdata->client->irq);
	if (sdata->input) {
		for (i = 0; i < S6SY661_MAX_SLOTS; i++) {
			input_mt_slot(sdata->input, i);
			input_mt_report_slot_inactive(sdata->input);
		}
		/* sec_ts_locked_release_all_finger() also releases KEY_HOMEPAGE, sec_ts.c:2482,2536 */
		if (sdata->home_key_down || sdata->wake_key_down) {
			input_report_key(sdata->input, KEY_HOMEPAGE, 0);
			input_report_key(sdata->input, KEY_WAKEUP, 0);
			sdata->home_key_down = false;
			sdata->wake_key_down = false;
		}
		s6sy761_sync(sdata);
	}
	if (sdata->irq_wake) {
		disable_irq_wake(sdata->client->irq);
		sdata->irq_wake = false;
	}
	WRITE_ONCE(sdata->lpm, false);
	regulator_disable(sdata->regulators[S6SY761_REGULATOR_VDD].consumer);
	regulator_disable(sdata->regulators[S6SY761_REGULATOR_AVDD].consumer);
	pinctrl_pm_select_sleep_state(&sdata->client->dev);
	sdata->powered = false;
}

/*
 * S6SY661 power-on after a power-off (sec_ts_start_device(), sec_ts.c:2911-3004): pins to
 * "default" before the rails, power-on, SENSE_ON if the input device is open (stock sends
 * it at the end of every start, :2998-3002; upstream only from input_open, which does not
 * run again after a resume), IRQ on. Caller holds power_lock.
 */
static int s6sy661_start(struct s6sy761_data *sdata)
{
	int ret;

	if (sdata->lpm) {
		ret = s6sy661_exit_lpm(sdata);
		if (!ret) {
			/* stock sec_ts_start_device(): sec_ts_set_custom_library(), sec_ts.c:2961 */
			if (sdata->pressure_key && s6sy661_sponge_write(sdata))
				dev_warn(&sdata->client->dev, "failed to configure the pressure key\n");
			return 0;
		}
		/* the chip did not leave the low-power mode: power-cycle it */
		dev_err(&sdata->client->dev, "leaving the low-power mode failed: %d\n", ret);
		s6sy661_power_off(sdata);
	}

	if (sdata->powered) {
		if (sdata->pressure_key && s6sy661_sponge_write(sdata))
			dev_warn(&sdata->client->dev, "failed to configure the pressure key\n");
		return 0;
	}

	pinctrl_pm_select_default_state(&sdata->client->dev);
	ret = s6sy661_power_on(sdata, false);
	if (ret)
		return ret;

	/* input_device_enabled() needs the input mutex (A:drivers/input/input.c:2252-2257) */
	mutex_lock(&sdata->input->mutex);
	if (input_device_enabled(sdata->input))
		ret = i2c_smbus_write_byte(sdata->client, S6SY761_SENSE_ON);
	mutex_unlock(&sdata->input->mutex);
	if (ret)
		dev_err(&sdata->client->dev, "failed to turn on sensing\n");

	enable_irq(sdata->client->irq);
	return 0;
}

static void s6sy661_devm_power_off(void *data)
{
	struct s6sy761_data *sdata = data;

	mutex_lock(&sdata->power_lock);
	s6sy661_power_off(sdata);
	mutex_unlock(&sdata->power_lock);
}

/*
 * Screen-off power (spec L6F 5.12 item 13): the touch chip follows the DSI panel through
 * the DRM panel-follower API (A:drivers/gpu/drm/drm_panel.c:580-583), as i2c-hid does
 * (A:drivers/hid/i2c-hid/i2c-hid-core.c:1161-1189). With the pressure home key enabled the
 * chip is not powered down but put in stock's low-power mode (sec_ts.c:2786-2808), where it
 * reports a hard press as KEY_WAKEUP and wakes the system.
 */
static void s6sy661_power_on_work(struct work_struct *work)
{
	struct s6sy761_data *sdata = container_of(work, struct s6sy761_data, power_on_work);
	int ret;

	mutex_lock(&sdata->power_lock);
	ret = s6sy661_start(sdata);
	mutex_unlock(&sdata->power_lock);
	if (ret)
		dev_err(&sdata->client->dev, "power-on after panel prepare failed: %d\n", ret);
}

/* 70 ms + up to ~2 s of ready wait: must not block the panel's power-up */
static int s6sy661_panel_prepared(struct drm_panel_follower *follower)
{
	struct s6sy761_data *sdata = container_of(follower, struct s6sy761_data, follower);

	queue_work(system_wq, &sdata->power_on_work);
	return 0;
}

/* before the panel powers down: synchronous */
static int s6sy661_panel_unpreparing(struct drm_panel_follower *follower)
{
	struct s6sy761_data *sdata = container_of(follower, struct s6sy761_data, follower);

	cancel_work_sync(&sdata->power_on_work);
	mutex_lock(&sdata->power_lock);
	/* stock: low-power mode if any sponge feature is on, else power off (sec_ts.c:2795-2808) */
	if (sdata->powered && sdata->pressure_key && sdata->lowpower_mode &&
	    !s6sy661_enter_lpm(sdata)) {
		mutex_unlock(&sdata->power_lock);
		return 0;
	}
	s6sy661_power_off(sdata);
	mutex_unlock(&sdata->power_lock);
	return 0;
}

static const struct drm_panel_follower_funcs s6sy661_follower_funcs = {
	.panel_prepared = s6sy661_panel_prepared,
	.panel_unpreparing = s6sy661_panel_unpreparing,
};

static void s6sy661_cancel_power_on_work(void *data)
{
	struct s6sy761_data *sdata = data;

	cancel_work_sync(&sdata->power_on_work);
}

/*
 * stock sec_ts_check_custom_library() (sec_ts.c:1993-2018): read the 10 info bytes at
 * SPONGE_GET_INFO and compare the first four characters with the model name from the DT; a
 * match means the firmware has the sponge library. Stock additionally needs the DT
 * pressure-sensor property (:2192), which samsung,sponge-model stands for. Chip is powered.
 */
static void s6sy661_pressure_init(struct s6sy761_data *sdata)
{
	struct device *dev = &sdata->client->dev;
	u8 info[S6SY661_SPONGE_INFO_SIZE];
	const char *model;
	int ret;

	if (device_property_read_string(dev, "samsung,sponge-model", &model))
		return;

	ret = i2c_smbus_read_i2c_block_data(sdata->client, S6SY661_SPONGE_GET_INFO,
					    sizeof(info), info);
	if (ret != sizeof(info) || strncmp(info, model, S6SY661_SPONGE_MODEL_LEN)) {
		dev_info(dev, "no sponge library for %s, pressure key off\n", model);
		return;
	}

	/* stock sec_ts.c:2192-2193: the key is on by default */
	sdata->lowpower_mode = S6SY661_MODE_FORCE_KEY;
	sdata->pressure_key = true;
	if (s6sy661_sponge_write(sdata))
		dev_warn(dev, "failed to configure the pressure key\n");
}

static int s6sy761_hw_init(struct s6sy761_data *sdata,
			   unsigned int *max_x, unsigned int *max_y)
{
	u8 buffer[S6SY761_PANEL_ID_SIZE]; /* larger read size */
	int ret;

	if (sdata->variant->s6sy661)
		ret = s6sy661_power_on(sdata, true);
	else
		ret = s6sy761_power_on(sdata);
	if (ret)
		return ret;

	ret = i2c_smbus_read_i2c_block_data(sdata->client,
					    S6SY761_DEVICE_ID,
					    S6SY761_DEVID_SIZE,
					    buffer);
	if (ret < 0)
		return ret;

	sdata->devid = get_unaligned_be16(buffer + 1);

	ret = i2c_smbus_read_i2c_block_data(sdata->client,
					    S6SY761_PANEL_INFO,
					    S6SY761_PANEL_ID_SIZE,
					    buffer);
	if (ret < 0)
		return ret;

	*max_x = get_unaligned_be16(buffer);
	*max_y = get_unaligned_be16(buffer + 2);
	if (sdata->variant->s6sy661) {
		/* the chip reports the size; the last coordinate is size - 1 (sec_ts.c:1922-1926) */
		*max_x -= 1;
		*max_y -= 1;
	}

	/* if no tx channels defined, at least keep one */
	sdata->tx_channel = max_t(u8, buffer[8], 1);

	ret = i2c_smbus_read_byte_data(sdata->client,
				       S6SY761_FIRMWARE_INTEGRITY);
	if (ret < 0)
		return ret;
	else if (sdata->variant->s6sy661 ? !(ret & S6SY761_FW_OK) : ret != S6SY761_FW_OK)
		return -ENODEV;	/* S6SY661: only bit 7 counts, sec_ts.c:2226 */

	return 0;
}

static void s6sy761_power_off(void *data)
{
	struct s6sy761_data *sdata = data;

	disable_irq(sdata->client->irq);
	regulator_bulk_disable(ARRAY_SIZE(sdata->regulators),
						sdata->regulators);
}

/*
 * aod_enable / singletap_enable (sec_ts_fn.c:5226-5281): set or clear the sponge mode bit and
 * send the mode byte. Without the sponge library stock's aod_enable succeeds and does nothing
 * and its singletap_enable fails. The chip takes the bit at once when powered, else at the next
 * power-on (s6sy661_power_on()).
 */
static int s6sy661_cmd_sponge_mode(struct s6sy761_data *sdata, u8 bit, int on)
{
	int ret = 0;

	if (on != 0 && on != 1)
		return -EINVAL;
	if (!sdata->pressure_key)
		return bit == S6SY661_MODE_SINGLE_TAP ? -ENODEV : 0;

	mutex_lock(&sdata->power_lock);
	if (on)
		sdata->lowpower_mode |= bit;
	else
		sdata->lowpower_mode &= ~bit;
	if (sdata->powered)
		ret = s6sy661_sponge_write(sdata);
	mutex_unlock(&sdata->power_lock);

	return ret;
}

/* set_aod_rect,w,h,x,y (sec_ts_fn.c:5131-5172): stored, and sent with the sponge library */
static int s6sy661_cmd_aod_rect(struct s6sy761_data *sdata, const int *param)
{
	int i, ret = 0;

	mutex_lock(&sdata->power_lock);
	for (i = 0; i < 4; i++)
		sdata->aod_rect[i] = param[i];
	if (sdata->pressure_key && sdata->powered)
		ret = s6sy661_sponge_write_rect(sdata);
	mutex_unlock(&sdata->power_lock);

	return ret;
}

/*
 * sec/tsp command node for the LineageOS touch HAL (glove mode, spec L6F 5.12 item 12; the
 * single-tap gesture) and power HAL (double-tap-to-wake): the synchronous path of stock sec_cmd
 * (S:drivers/input/sec_cmd.c) with the commands below.
 */
static int s6sy661_cmd_glove_mode(struct s6sy761_data *sdata, int mode)
{
	int ret = 0;

	if (mode != 0 && mode != 1)
		return -EINVAL;

	mutex_lock(&sdata->power_lock);
	/* stored first, so the next power-on applies it (sec_ts.c:1495-1507) */
	if (mode)
		sdata->touch_functions |= S6SY661_FUNC_GLOVE | S6SY661_FUNC_DEFAULT;
	else
		sdata->touch_functions = (sdata->touch_functions & ~S6SY661_FUNC_GLOVE) |
					 S6SY661_FUNC_DEFAULT;
	if (!sdata->powered)
		ret = -EIO;	/* stock: power off -> NG, sec_ts.c:1504-1508 */
	else
		ret = i2c_smbus_write_word_data(sdata->client, S6SY761_TOUCH_FUNCTION,
						sdata->touch_functions);
	mutex_unlock(&sdata->power_lock);

	return ret;
}

static ssize_t cmd_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct s6sy761_data *sdata = dev_get_drvdata(dev);
	int param[S6SY661_CMD_PARAM_NUM] = { };
	char params[S6SY661_CMD_STR_LEN];
	char *name, *cur, *tok;
	const char *result, *state;
	size_t len = count;
	int nparam = 0;
	int ret;

	/* sec_cmd.c:85-96 */
	if (count >= S6SY661_CMD_STR_LEN || strlen(buf) >= S6SY661_CMD_STR_LEN)
		return -EINVAL;

	mutex_lock(&sdata->cmd_lock);
	if (sdata->cmd_running) {
		mutex_unlock(&sdata->cmd_lock);
		return -EBUSY;
	}
	sdata->cmd_running = true;
	sdata->cmd_state = "RUNNING";
	mutex_unlock(&sdata->cmd_lock);

	/* the command text without one trailing newline (sec_cmd.c:111-116) */
	if (len && buf[len - 1] == '\n')
		len--;
	memcpy(sdata->cmd, buf, len);
	sdata->cmd[len] = '\0';

	/* name up to the first ',', then up to 8 decimal parameters (sec_cmd.c:117-161) */
	strscpy(params, sdata->cmd, sizeof(params));
	cur = params;
	name = strsep(&cur, ",");
	while (cur && nparam < S6SY661_CMD_PARAM_NUM) {
		tok = strsep(&cur, ",");
		if (kstrtoint(tok, 10, &param[nparam++]) < 0) {
			/*
			 * Stock returns here with the command still marked running
			 * (sec_cmd.c:178-179), which blocks every later command; this
			 * port reports NG instead.
			 */
			result = "NG";
			state = "FAIL";
			goto out;
		}
	}

	/* each: OK, or NG on a bad value or I/O failure */
	if (!strcmp(name, "glove_mode")) {
		ret = s6sy661_cmd_glove_mode(sdata, param[0]);
	} else if (!strcmp(name, "aod_enable")) {
		ret = s6sy661_cmd_sponge_mode(sdata, S6SY661_MODE_AOD, param[0]);
	} else if (!strcmp(name, "singletap_enable")) {
		ret = s6sy661_cmd_sponge_mode(sdata, S6SY661_MODE_SINGLE_TAP, param[0]);
	} else if (!strcmp(name, "set_aod_rect")) {
		ret = s6sy661_cmd_aod_rect(sdata, param);
	} else {
		/* not_support_cmd(), sec_ts_fn.c:5745-5756 */
		result = "NA";
		state = "NOT_APPLICABLE";
		goto out;
	}
	result = ret ? "NG" : "OK";
	state = ret ? "FAIL" : "OK";

out:
	mutex_lock(&sdata->cmd_lock);
	/* "<command text>:<result>" (sec_cmd_set_default_result() + _cmd_result(), sec_cmd.c:54-65) */
	snprintf(sdata->cmd_result, sizeof(sdata->cmd_result), "%s:%s", sdata->cmd, result);
	sdata->cmd_state = state;
	sdata->cmd_running = false;
	mutex_unlock(&sdata->cmd_lock);

	return count;
}
static DEVICE_ATTR(cmd, 0220, NULL, cmd_store);	/* S_IWUSR | S_IWGRP, sec_cmd.c:429 */

static ssize_t cmd_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct s6sy761_data *sdata = dev_get_drvdata(dev);
	ssize_t ret;

	mutex_lock(&sdata->cmd_lock);
	ret = sysfs_emit(buf, "%s\n", sdata->cmd_state);	/* sec_cmd.c:358-387 */
	mutex_unlock(&sdata->cmd_lock);

	return ret;
}
static DEVICE_ATTR_RO(cmd_status);

static ssize_t cmd_result_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct s6sy761_data *sdata = dev_get_drvdata(dev);
	ssize_t ret;

	mutex_lock(&sdata->cmd_lock);
	ret = sysfs_emit(buf, "%s\n", sdata->cmd_result);	/* sec_cmd.c:389-407 */
	sdata->cmd_state = "WAITING";
	mutex_unlock(&sdata->cmd_lock);

	return ret;
}
static DEVICE_ATTR_RO(cmd_result);

/*
 * Header, one name per line, then a final "\n" (sec_cmd.c:409-427). The LineageOS touch HAL
 * offers a feature for each command it finds here, so the sponge commands are listed only when
 * the firmware has the library (stock lists every command it has, sec_ts_fn.c:183-187).
 */
static ssize_t cmd_list_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct s6sy761_data *sdata = dev_get_drvdata(dev);

	return sysfs_emit(buf, "++factory command list++\nglove_mode\n%s\n",
			  sdata->pressure_key ? "set_aod_rect\naod_enable\nsingletap_enable\n" : "");
}
static DEVICE_ATTR_RO(cmd_list);

/*
 * /sys/class/sec/tsp/pressure_enable, stock pressure_enable_show()/_strore()
 * (sec_ts_y661/sec_ts_fn.c:528-590, mode S_IRUGO | S_IWUSR | S_IWGRP, :783): shows whether the
 * FORCE_KEY bit is set; takes "0"/"2" (off) or "1"/"3" (on; 0/1 factory and game tools, 2/3
 * the settings app) and sends the sponge mode byte again. Anything else, or a chip without the
 * library, is -EINVAL. Stock sends it at once; here it is also sent at the next power-on when
 * the chip is off, as stock does at every start (sec_ts.c:864-865).
 */
static ssize_t pressure_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct s6sy761_data *sdata = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", !!(sdata->lowpower_mode & S6SY661_MODE_FORCE_KEY));
}

static ssize_t pressure_enable_store(struct device *dev, struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct s6sy761_data *sdata = dev_get_drvdata(dev);
	unsigned long value;
	int ret;

	if (count > 2)
		return -EINVAL;

	ret = kstrtoul(buf, 10, &value);
	if (ret)
		return ret;

	if (!sdata->pressure_key)
		return -EINVAL;

	if (value != 0 && value != 1 && value != 2 && value != 3)
		return -EINVAL;

	mutex_lock(&sdata->power_lock);
	if (value == 1 || value == 3)
		sdata->lowpower_mode |= S6SY661_MODE_FORCE_KEY;
	else
		sdata->lowpower_mode &= ~S6SY661_MODE_FORCE_KEY;
	if (sdata->powered)
		ret = s6sy661_sponge_write(sdata);
	mutex_unlock(&sdata->power_lock);

	return ret ?: count;
}
static DEVICE_ATTR(pressure_enable, 0664, pressure_enable_show, pressure_enable_store);

static struct attribute *s6sy661_cmd_attrs[] = {
	&dev_attr_cmd.attr,
	&dev_attr_cmd_status.attr,
	&dev_attr_cmd_result.attr,
	&dev_attr_cmd_list.attr,
	&dev_attr_pressure_enable.attr,
	NULL
};
ATTRIBUTE_GROUPS(s6sy661_cmd);

static void s6sy661_sec_device_destroy(void *data)
{
	sec_device_destroy(data);
}

/*
 * The rest of an S6SY661 probe, after the IRQ is requested. No runtime PM: stock never puts
 * the chip to sleep with SET_POWER_MODE (spec L6F 5.12 item 9).
 */
static int s6sy661_probe_finish(struct s6sy761_data *sdata)
{
	struct device *dev = &sdata->client->dev;
	struct device *sec_dev;
	int err;

	/* the pressure key wakes the system from the low-power mode: sec_ts.c:2352 */
	if (sdata->pressure_key) {
		err = devm_device_init_wakeup(dev);
		if (err)
			return err;
	}

	if (drm_is_panel_follower(dev)) {
		/* registered first, so it runs after the follower is removed */
		err = devm_add_action_or_reset(dev, s6sy661_cancel_power_on_work, sdata);
		if (err)
			return err;

		/* a prepared panel calls panel_prepared at once: powered by probe, no-op */
		sdata->follower.funcs = &s6sy661_follower_funcs;
		err = devm_drm_panel_add_follower(dev, &sdata->follower);
		if (err)
			return err;
	}

	/* /sys/class/sec/tsp (sec_cmd.c:494-505); absent without CONFIG_SAMSUNG_SEC_CLASS */
	sec_dev = sec_device_create_with_groups(sdata, s6sy661_cmd_groups, "tsp");
	if (IS_ERR(sec_dev)) {
		if (PTR_ERR(sec_dev) != -ENODEV)
			dev_warn(dev, "no sec/tsp node: %ld\n", PTR_ERR(sec_dev));
		return 0;
	}

	return devm_add_action_or_reset(dev, s6sy661_sec_device_destroy, sec_dev);
}

static int s6sy761_probe(struct i2c_client *client)
{
	struct s6sy761_data *sdata;
	unsigned int max_x, max_y;
	struct drm_panel *panel;
	struct device_node *np;
	int err;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C |
						I2C_FUNC_SMBUS_BYTE_DATA |
						I2C_FUNC_SMBUS_I2C_BLOCK))
		return -ENODEV;

	sdata = devm_kzalloc(&client->dev, sizeof(*sdata), GFP_KERNEL);
	if (!sdata)
		return -ENOMEM;

	i2c_set_clientdata(client, sdata);
	sdata->client = client;
	sdata->variant = device_get_match_data(&client->dev) ?: &s6sy761_variant;

	if (sdata->variant->s6sy661) {
		mutex_init(&sdata->power_lock);
		mutex_init(&sdata->cmd_lock);
		sdata->cmd_state = "WAITING";
		sdata->touch_functions = S6SY661_FUNC_DEFAULT;
		INIT_WORK(&sdata->power_on_work, s6sy661_power_on_work);

		/*
		 * A panel follower waits for the panel before powering anything
		 * (A:drivers/gpu/drm/drm_panel.c:450-467): L5's DSI host registers it.
		 */
		if (drm_is_panel_follower(&client->dev)) {
			np = of_parse_phandle(client->dev.of_node, "panel", 0);
			panel = of_drm_find_panel(np);
			of_node_put(np);
			if (IS_ERR(panel))
				return dev_err_probe(&client->dev, PTR_ERR(panel),
						     "panel not available\n");
		}
	}

	sdata->regulators[S6SY761_REGULATOR_VDD].supply = "vdd";
	sdata->regulators[S6SY761_REGULATOR_AVDD].supply = "avdd";
	err = devm_regulator_bulk_get(&client->dev,
				      ARRAY_SIZE(sdata->regulators),
				      sdata->regulators);
	if (err)
		return err;

	err = devm_add_action_or_reset(&client->dev,
				       sdata->variant->s6sy661 ? s6sy661_devm_power_off :
								 s6sy761_power_off,
				       sdata);
	if (err)
		return err;

	err = s6sy761_hw_init(sdata, &max_x, &max_y);
	if (err)
		return err;

	if (sdata->variant->s6sy661)
		s6sy661_pressure_init(sdata);

	sdata->input = devm_input_allocate_device(&client->dev);
	if (!sdata->input)
		return -ENOMEM;

	sdata->input->name = sdata->variant->name;
	sdata->input->id.bustype = BUS_I2C;
	sdata->input->open = s6sy761_input_open;
	sdata->input->close = s6sy761_input_close;

	input_set_abs_params(sdata->input, ABS_MT_POSITION_X, 0, max_x, 0, 0);
	input_set_abs_params(sdata->input, ABS_MT_POSITION_Y, 0, max_y, 0, 0);
	input_set_abs_params(sdata->input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(sdata->input, ABS_MT_TOUCH_MINOR, 0, 255, 0, 0);
	input_set_abs_params(sdata->input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(sdata->input, ABS_MT_TOUCH_MINOR, 0, 255, 0, 0);
	input_set_abs_params(sdata->input, ABS_MT_PRESSURE, 0, 255, 0, 0);

	if (sdata->pressure_key) {
		input_set_capability(sdata->input, EV_KEY, KEY_HOMEPAGE);	/* sec_ts.c:2055 */
		input_set_capability(sdata->input, EV_KEY, KEY_WAKEUP);
		/* sec_ts.c:2037 */
		input_set_capability(sdata->input, EV_KEY, S6SY661_KEY_BLACK_UI_GESTURE);
	}

	touchscreen_parse_properties(sdata->input, true, &sdata->prop);

	if (!input_abs_get_max(sdata->input, ABS_X) ||
	    !input_abs_get_max(sdata->input, ABS_Y)) {
		dev_warn(&client->dev, "the axis have not been set\n");
	}

	err = input_mt_init_slots(sdata->input,
				  sdata->variant->s6sy661 ? S6SY661_MAX_SLOTS : sdata->tx_channel,
				  INPUT_MT_DIRECT);
	if (err)
		return err;

	input_set_drvdata(sdata->input, sdata);

	err = input_register_device(sdata->input);
	if (err)
		return err;

	err = devm_request_threaded_irq(&client->dev, client->irq, NULL,
					s6sy761_irq_handler,
					IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					"s6sy761_irq", sdata);
	if (err)
		return err;

	if (sdata->variant->s6sy661)
		return s6sy661_probe_finish(sdata);

	pm_runtime_enable(&client->dev);

	return 0;
}

static void s6sy761_remove(struct i2c_client *client)
{
	struct s6sy761_data *sdata = i2c_get_clientdata(client);

	if (!sdata->variant->s6sy661)
		pm_runtime_disable(&client->dev);
}

static int s6sy761_runtime_suspend(struct device *dev)
{
	struct s6sy761_data *sdata = dev_get_drvdata(dev);

	return i2c_smbus_write_byte_data(sdata->client,
				S6SY761_APPLICATION_MODE, S6SY761_APP_SLEEP);
}

static int s6sy761_runtime_resume(struct device *dev)
{
	struct s6sy761_data *sdata = dev_get_drvdata(dev);

	return i2c_smbus_write_byte_data(sdata->client,
				S6SY761_APPLICATION_MODE, S6SY761_APP_NORMAL);
}

/*
 * S6SY661 system sleep: nothing to do for a panel follower, whose power follows the panel;
 * the DRM suspend unprepares a lit panel in its .prepare phase, before any .suspend
 * (A:drivers/gpu/drm/exynos/exynos_drm_drv.c:128,138-141; as i2c-hid,
 * A:drivers/hid/i2c-hid/i2c-hid-core.c:1360-1388).
 */
static int s6sy661_suspend(struct s6sy761_data *sdata)
{
	if (drm_is_panel_follower(&sdata->client->dev))
		return 0;

	mutex_lock(&sdata->power_lock);
	s6sy661_power_off(sdata);
	mutex_unlock(&sdata->power_lock);

	return 0;
}

static int s6sy661_resume(struct s6sy761_data *sdata)
{
	int ret;

	if (drm_is_panel_follower(&sdata->client->dev))
		return 0;

	mutex_lock(&sdata->power_lock);
	ret = s6sy661_start(sdata);
	mutex_unlock(&sdata->power_lock);

	return ret;
}

static int s6sy761_suspend(struct device *dev)
{
	struct s6sy761_data *sdata = dev_get_drvdata(dev);

	if (sdata->variant->s6sy661)
		return s6sy661_suspend(sdata);

	s6sy761_power_off(sdata);

	return 0;
}

static int s6sy761_resume(struct device *dev)
{
	struct s6sy761_data *sdata = dev_get_drvdata(dev);

	if (sdata->variant->s6sy661)
		return s6sy661_resume(sdata);

	enable_irq(sdata->client->irq);

	return s6sy761_power_on(sdata);
}

static const struct dev_pm_ops s6sy761_pm_ops = {
	SYSTEM_SLEEP_PM_OPS(s6sy761_suspend, s6sy761_resume)
	RUNTIME_PM_OPS(s6sy761_runtime_suspend, s6sy761_runtime_resume, NULL)
};

#ifdef CONFIG_OF
static const struct of_device_id s6sy761_of_match[] = {
	{ .compatible = "samsung,s6sy761", .data = &s6sy761_variant },
	{ .compatible = "samsung,s6sy661", .data = &s6sy661_variant },
	{ },
};
MODULE_DEVICE_TABLE(of, s6sy761_of_match);
#endif

static const struct i2c_device_id s6sy761_id[] = {
	{ "s6sy761" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, s6sy761_id);

static struct i2c_driver s6sy761_driver = {
	.driver = {
		.name = S6SY761_DEV_NAME,
		.dev_groups = s6sy761_sysfs_groups,
		.of_match_table = of_match_ptr(s6sy761_of_match),
		.pm = pm_ptr(&s6sy761_pm_ops),
	},
	.probe = s6sy761_probe,
	.remove = s6sy761_remove,
	.id_table = s6sy761_id,
};

module_i2c_driver(s6sy761_driver);

MODULE_AUTHOR("Andi Shyti <andi.shyti@samsung.com>");
MODULE_DESCRIPTION("Samsung S6SY761 Touch Screen");
MODULE_LICENSE("GPL v2");
