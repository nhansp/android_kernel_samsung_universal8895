// SPDX-License-Identifier: GPL-2.0-only
/*
 * Wacom W9018 digitizer, the S Pen of the Samsung Galaxy Note 8: the subset
 * of the "G5" I2C protocol of the vendor driver that pen input needs.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/pm_wakeirq.h>
#include <linux/pm_wakeup.h>
#include <linux/regulator/consumer.h>
#include <linux/unaligned.h>

#define COM_COORD_NUM			13
#define COM_QUERY_POS			14
#define COM_QUERY_BUFFER		29

#define COM_QUERY_RETRY			3

/* query data, from COM_QUERY_POS */
#define EPEN_REG_HEADER			0x00
#define EPEN_REG_X1			0x01
#define EPEN_REG_X2			0x02
#define EPEN_REG_Y1			0x03
#define EPEN_REG_Y2			0x04
#define EPEN_REG_PRESSURE1		0x05
#define EPEN_REG_PRESSURE2		0x06
#define EPEN_REG_FWVER1			0x07
#define EPEN_REG_FWVER2			0x08
#define EPEN_REG_TILT_X			0x0B
#define EPEN_REG_TILT_Y			0x0C
#define EPEN_REG_HEIGHT			0x0D

#define EPEN_HEADER_QUERY		0x0f

/* coordinate packet, byte 0 */
#define EPEN_READY			BIT(7)
#define EPEN_RUBBER			BIT(6)
#define EPEN_STYLUS			BIT(5)
#define EPEN_PROX			BIT(4)

/* coordinate packet, byte 12: a touch on a virtual soft key */
#define EPEN_SOFTKEY			BIT(7)

struct wacom_w9018_data {
	struct i2c_client *client;
	struct input_dev *input;
	struct regulator *vddo;
	struct gpio_desc *fwe_gpio;
	struct gpio_desc *pdct_gpio;
	int irq_data;
	int irq_pdct;
	u32 max_x;
	u32 max_y;
	u32 pressure;
	u16 fw_version;
	s8 tilt_x;
	s8 tilt_y;
	u8 height;
	bool xy_switch;
	bool x_invert;
	bool pen_prox;
	unsigned int tool;
};

/* the vendor kernel delays every transfer on this bus by 50 us */
static int wacom_w9018_recv(struct i2c_client *client, void *buf, int count)
{
	usleep_range(50, 60);
	return i2c_master_recv(client, buf, count);
}

static void wacom_w9018_power_off(void *data)
{
	struct wacom_w9018_data *w9018 = data;

	regulator_disable(w9018->vddo);
}

static irqreturn_t wacom_w9018_data_irq(int irq, void *dev_id)
{
	struct wacom_w9018_data *w9018 = dev_id;
	/* one byte more than is used, as the vendor driver reads it */
	u8 d[COM_COORD_NUM + 1];
	int ret;
	int retry = 3;
	bool rdy;

	while (retry--) {
		ret = wacom_w9018_recv(w9018->client, d, sizeof(d));
		if (ret >= 0)
			break;
	}

	if (ret < 0) {
		if (w9018->pen_prox) {
			input_report_key(w9018->input, BTN_STYLUS, 0);
			input_report_key(w9018->input, BTN_TOUCH, 0);
			input_report_abs(w9018->input, ABS_PRESSURE, 0);
			input_sync(w9018->input);

			input_report_abs(w9018->input, ABS_DISTANCE, 0);
			input_report_key(w9018->input, BTN_TOOL_RUBBER, 0);
			input_report_key(w9018->input, BTN_TOOL_PEN, 0);
			input_sync(w9018->input);

			w9018->pen_prox = false;
		}
		return IRQ_HANDLED;
	}

	rdy = !!(d[0] & EPEN_READY);
	if (rdy) {
		int x, y, p, gain;
		s8 tilt_x, tilt_y;
		bool prox, side;

		if (d[12] & EPEN_SOFTKEY)
			return IRQ_HANDLED;

		prox = !!(d[0] & EPEN_PROX);
		side = !!(d[0] & EPEN_STYLUS);
		x = get_unaligned_be16(&d[1]);
		y = get_unaligned_be16(&d[3]);
		p = get_unaligned_be16(&d[5]);
		gain = d[7];
		tilt_x = (s8)d[9];
		tilt_y = -(s8)d[8];

		if (w9018->x_invert)
			x = w9018->max_x - x;

		if (w9018->xy_switch)
			swap(x, y);

		if (w9018->xy_switch) {
			if (x < 0 || y < 0 || x > w9018->max_y || y > w9018->max_x)
				return IRQ_HANDLED;
		} else {
			if (x < 0 || y < 0 || x > w9018->max_x || y > w9018->max_y)
				return IRQ_HANDLED;
		}

		/* the first packet in range only selects the tool, as in the vendor driver */
		if (!w9018->pen_prox) {
			w9018->pen_prox = true;
			w9018->tool = (d[0] & EPEN_RUBBER) ?
				      BTN_TOOL_RUBBER : BTN_TOOL_PEN;
			return IRQ_HANDLED;
		}

		input_report_abs(w9018->input, ABS_X, x);
		input_report_abs(w9018->input, ABS_Y, y);
		input_report_key(w9018->input, BTN_STYLUS, side);
		input_report_key(w9018->input, BTN_TOUCH, prox);
		input_report_abs(w9018->input, ABS_PRESSURE, p);
		input_report_abs(w9018->input, ABS_DISTANCE, gain);
		input_report_abs(w9018->input, ABS_TILT_X, tilt_x);
		input_report_abs(w9018->input, ABS_TILT_Y, tilt_y);
		input_report_key(w9018->input, w9018->tool, 1);
		input_sync(w9018->input);
	} else {
		if (w9018->pen_prox) {
			input_report_key(w9018->input, BTN_STYLUS, 0);
			input_report_key(w9018->input, BTN_TOUCH, 0);
			input_report_abs(w9018->input, ABS_PRESSURE, 0);
			input_sync(w9018->input);

			input_report_abs(w9018->input, ABS_DISTANCE, 0);
			input_report_key(w9018->input, BTN_TOOL_RUBBER, 0);
			input_report_key(w9018->input, BTN_TOOL_PEN, 0);
			input_sync(w9018->input);

			w9018->pen_prox = false;
		}
	}

	return IRQ_HANDLED;
}

static irqreturn_t wacom_w9018_pdct_irq(int irq, void *dev_id)
{
	struct wacom_w9018_data *w9018 = dev_id;
	int val;

	val = gpiod_get_value_cansleep(w9018->pdct_gpio);
	if (val >= 0) {
		input_report_switch(w9018->input, SW_PEN_INSERTED, val);
		input_sync(w9018->input);
	}

	return IRQ_HANDLED;
}

static int wacom_w9018_input_open(struct input_dev *dev)
{
	struct wacom_w9018_data *w9018 = input_get_drvdata(dev);

	enable_irq(w9018->irq_data);
	return 0;
}

static void wacom_w9018_input_close(struct input_dev *dev)
{
	struct wacom_w9018_data *w9018 = input_get_drvdata(dev);

	disable_irq(w9018->irq_data);
}

static int wacom_w9018_query(struct wacom_w9018_data *w9018)
{
	u8 q[COM_QUERY_BUFFER];
	int ret;
	int i;

	for (i = 0; i < COM_QUERY_RETRY; i++) {
		ret = wacom_w9018_recv(w9018->client, q, sizeof(q));
		if (ret == sizeof(q) &&
		    q[COM_QUERY_POS + EPEN_REG_HEADER] == EPEN_HEADER_QUERY)
			break;
	}

	if (i == COM_QUERY_RETRY)
		return -ENODEV;

	w9018->max_x = get_unaligned_be16(&q[COM_QUERY_POS + EPEN_REG_X1]);
	w9018->max_y = get_unaligned_be16(&q[COM_QUERY_POS + EPEN_REG_Y1]);
	w9018->pressure = get_unaligned_be16(&q[COM_QUERY_POS + EPEN_REG_PRESSURE1]);
	w9018->fw_version = get_unaligned_be16(&q[COM_QUERY_POS + EPEN_REG_FWVER1]);
	w9018->tilt_x = q[COM_QUERY_POS + EPEN_REG_TILT_X];
	w9018->tilt_y = q[COM_QUERY_POS + EPEN_REG_TILT_Y];
	w9018->height = q[COM_QUERY_POS + EPEN_REG_HEIGHT];

	return 0;
}

static int wacom_w9018_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct wacom_w9018_data *w9018;
	int val;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -EIO;

	w9018 = devm_kzalloc(dev, sizeof(*w9018), GFP_KERNEL);
	if (!w9018)
		return -ENOMEM;

	w9018->client = client;

	w9018->vddo = devm_regulator_get(dev, "vddo");
	if (IS_ERR(w9018->vddo))
		return dev_err_probe(dev, PTR_ERR(w9018->vddo),
				      "failed to get vddo regulator\n");

	w9018->pdct_gpio = devm_gpiod_get(dev, "pdct", GPIOD_IN);
	if (IS_ERR(w9018->pdct_gpio))
		return dev_err_probe(dev, PTR_ERR(w9018->pdct_gpio),
				      "failed to get pdct gpio\n");

	w9018->fwe_gpio = devm_gpiod_get(dev, "fwe", GPIOD_OUT_LOW);
	if (IS_ERR(w9018->fwe_gpio))
		return dev_err_probe(dev, PTR_ERR(w9018->fwe_gpio),
				      "failed to get fwe gpio\n");

	w9018->xy_switch = device_property_read_bool(dev, "wacom,xy-switch");
	w9018->x_invert = device_property_read_bool(dev, "wacom,x-invert");

	w9018->irq_data = of_irq_get_byname(dev->of_node, "data");
	if (w9018->irq_data < 0)
		return dev_err_probe(dev, w9018->irq_data,
				      "failed to get data irq\n");

	w9018->irq_pdct = of_irq_get_byname(dev->of_node, "pdct");
	if (w9018->irq_pdct < 0)
		return dev_err_probe(dev, w9018->irq_pdct,
				      "failed to get pdct irq\n");

	/* pulse FWE, as the vendor driver does, to leave a latched flash mode */
	gpiod_set_value_cansleep(w9018->fwe_gpio, 1);
	gpiod_set_value_cansleep(w9018->fwe_gpio, 0);

	ret = regulator_enable(w9018->vddo);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable vddo regulator\n");

	ret = devm_add_action_or_reset(dev, wacom_w9018_power_off, w9018);
	if (ret)
		return ret;

	msleep(100);

	ret = wacom_w9018_query(w9018);
	if (ret)
		return dev_err_probe(dev, ret, "failed to query wacom features\n");

	dev_info(dev, "fw 0x%04x max %dx%d pressure %d\n",
		 w9018->fw_version, w9018->max_x, w9018->max_y, w9018->pressure);

	w9018->input = devm_input_allocate_device(dev);
	if (!w9018->input)
		return -ENOMEM;

	w9018->input->name = "sec_e-pen";
	w9018->input->id.bustype = BUS_I2C;
	w9018->input->open = wacom_w9018_input_open;
	w9018->input->close = wacom_w9018_input_close;
	input_set_drvdata(w9018->input, w9018);

	input_set_capability(w9018->input, EV_KEY, BTN_TOOL_PEN);
	input_set_capability(w9018->input, EV_KEY, BTN_TOOL_RUBBER);
	input_set_capability(w9018->input, EV_KEY, BTN_TOUCH);
	input_set_capability(w9018->input, EV_KEY, BTN_STYLUS);
	input_set_capability(w9018->input, EV_SW, SW_PEN_INSERTED);

	if (w9018->xy_switch) {
		input_set_abs_params(w9018->input, ABS_X, 0, w9018->max_y, 4, 0);
		input_set_abs_params(w9018->input, ABS_Y, 0, w9018->max_x, 4, 0);
	} else {
		input_set_abs_params(w9018->input, ABS_X, 0, w9018->max_x, 4, 0);
		input_set_abs_params(w9018->input, ABS_Y, 0, w9018->max_y, 4, 0);
	}

	input_set_abs_params(w9018->input, ABS_PRESSURE, 0, w9018->pressure, 0, 0);
	input_set_abs_params(w9018->input, ABS_DISTANCE, 0, w9018->height, 0, 0);
	input_set_abs_params(w9018->input, ABS_TILT_X, -w9018->tilt_x, w9018->tilt_x, 0, 0);
	input_set_abs_params(w9018->input, ABS_TILT_Y, -w9018->tilt_y, w9018->tilt_y, 0, 0);

	ret = devm_request_threaded_irq(dev, w9018->irq_data, NULL,
					wacom_w9018_data_irq,
					IRQF_ONESHOT | IRQF_NO_AUTOEN,
					"sec_epen_irq", w9018);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request data irq\n");

	ret = devm_request_threaded_irq(dev, w9018->irq_pdct, NULL,
					wacom_w9018_pdct_irq,
					IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					"sec_epen_pdct", w9018);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request pdct irq\n");

	/* pen removal and insertion wake the system; pen data does not */
	ret = devm_device_init_wakeup(dev);
	if (ret)
		return ret;
	ret = devm_pm_set_wake_irq(dev, w9018->irq_pdct);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set pdct wake irq\n");

	ret = input_register_device(w9018->input);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register input device\n");

	val = gpiod_get_value_cansleep(w9018->pdct_gpio);
	if (val >= 0) {
		input_report_switch(w9018->input, SW_PEN_INSERTED, val);
		input_sync(w9018->input);
	}

	i2c_set_clientdata(client, w9018);

	return 0;
}

static const struct of_device_id wacom_w9018_of_match[] = {
	{ .compatible = "wacom,w9018" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, wacom_w9018_of_match);

static const struct i2c_device_id wacom_w9018_id[] = {
	{ "w9018" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, wacom_w9018_id);

static struct i2c_driver wacom_w9018_driver = {
	.driver = {
		.name = "wacom_w9018",
		.of_match_table = wacom_w9018_of_match,
	},
	.probe = wacom_w9018_probe,
	.id_table = wacom_w9018_id,
};
module_i2c_driver(wacom_w9018_driver);

MODULE_DESCRIPTION("Wacom W9018 digitizer driver");
MODULE_LICENSE("GPL");
