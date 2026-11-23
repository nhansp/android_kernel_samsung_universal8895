// SPDX-License-Identifier: GPL-2.0
/*
 * Maxim MAX77865 MUIC driver: the USB D+/D- path switch and the BC1.2
 * charger detection, driven by the S2MM005 Type-C port controller and by the
 * MUIC's own VBUS interrupt. The high-voltage charger signalling is left to
 * the driver of the "hv" auxiliary device.
 */

#include <linux/auxiliary_bus.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/string.h>
#include <linux/usb/max77865-muic.h>

#define MUIC_REG_ID		0x00
#define MUIC_REG_INT_MAIN	0x01
#define MUIC_REG_INT_BC		0x02
#define MUIC_REG_INT_FC		0x03
#define MUIC_REG_INT_GP		0x04
#define MUIC_REG_INTMASK_MAIN	0x07
#define MUIC_REG_INTMASK_BC	0x08
#define MUIC_REG_INTMASK_FC	0x09
#define MUIC_REG_INTMASK_GP	0x0a
#define MUIC_REG_STATUS1_BC	0x0d
#define MUIC_REG_CONTROL1_BC	0x13
#define MUIC_REG_CCDET		0x15
#define MUIC_REG_CONTROL1	0x19
/* the last register of the HV message window the "hv" driver uses */
#define MUIC_REG_MAX		0x22

#define STATUS1_BC_VBVOLT	BIT(7)
#define STATUS1_BC_CHGTYP	GENMASK(1, 0)
#define CONTROL1_BC_CHGDETMAN	BIT(1)
#define CONTROL1_BC_CHGDETEN	BIT(0)

#define CHGTYP_USB		0x01
#define CHGTYP_CDP		0x02
#define CHGTYP_DEDICATED	0x03

/* CONTROL1 switch settings: D+/D- open, to the AP USB, AP UART or CP UART */
#define COM_OPEN		0x80
#define COM_USB_AP		0x89
#define COM_UART_AP		0x9b
#define COM_UART_CP		0xad

/* interrupt clear values, and the masks restored after a chip reset */
#define INT_MAIN_CLEAR		0x1a
#define INT_BC_CLEAR		0xff
#define INT_FC_CLEAR		0xfc
#define INT_GP_CLEAR		0x1f
#define INT_BC_VBUS		BIT(7)
#define INT_GP_RESET		BIT(1)
#define INTMASK_MAIN_RESTORE	0x05
#define INTMASK_BC_RESTORE	0x00
#define INTMASK_FC_RESTORE	0x77
#define INTMASK_GP_RESTORE	0xe0
#define CCDET_RESTORE		0x00

/* pmic_info= boot parameter bits */
#define SWITCH_SEL_UART_MASK		0x2
#define SWITCH_SEL_RUSTPROOF_MASK	0x8

/* resistor ID codes reported by the port controller */
#define RID_UNDEFINED	0
#define RID_000K	1
#define RID_001K	2
#define RID_255K	3
#define RID_301K	4
#define RID_523K	5
#define RID_619K	6
#define RID_OPEN	7

struct max77865_muic {
	struct device *dev;
	struct i2c_client *client;
	struct regmap *regmap;
	struct mutex lock;		/* protects the state below */
	struct blocking_notifier_head nh;

	unsigned int pmic_info;
	bool uart_ap;
	bool rustproof;

	bool attached;
	bool rprd;			/* the phone is the source */
	unsigned int cc_rp;		/* Rp current level of the attached source */
	bool water;
	int attached_dev;

	void (*hv_irq)(void *ctx, u8 int_bc, u8 int_fc, u8 int_gp);
	void *hv_ctx;
};

static struct i2c_driver max77865_muic_driver;

/* an I2C error is retried after 100 ms, up to four times */
static int muic_read(struct max77865_muic *m, u8 reg)
{
	unsigned int val;
	int ret, retry = 0;

	ret = regmap_read(m->regmap, reg, &val);
	while (ret < 0) {
		if (retry > 3) {
			dev_err(m->dev, "read 0x%02x failed (%d)\n", reg, ret);
			return ret;
		}
		msleep(100);
		ret = regmap_read(m->regmap, reg, &val);
		retry++;
	}
	return val;
}

static int muic_write(struct max77865_muic *m, u8 reg, u8 val)
{
	int ret, retry = 0;

	ret = regmap_write(m->regmap, reg, val);
	while (ret < 0) {
		if (retry > 3) {
			dev_err(m->dev, "write 0x%02x <- 0x%02x failed (%d)\n", reg, val, ret);
			return ret;
		}
		msleep(100);
		ret = regmap_write(m->regmap, reg, val);
		retry++;
	}
	return 0;
}

/* write and read back; rewrite after 100 ms while it differs, at most 7 times */
static int muic_guaranteed_write(struct max77865_muic *m, u8 reg, u8 val)
{
	int ret, written, retry = 0;

	ret = muic_write(m, reg, val);
	written = muic_read(m, reg);
	while (written != val) {
		dev_err(m->dev, "reg 0x%02x: written 0x%02x != 0x%02x\n", reg, written, val);
		if (retry > 5)
			break;
		msleep(100);
		retry++;
		ret = muic_write(m, reg, val);
		written = muic_read(m, reg);
	}
	return ret;
}

static int muic_update(struct max77865_muic *m, u8 reg, u8 mask, u8 val)
{
	int curr, ret;
	u8 new;

	curr = muic_read(m, reg);
	if (curr < 0)
		return curr;

	new = (curr & ~mask) | (val & mask);
	if (new == curr)
		return 0;

	ret = muic_guaranteed_write(m, reg, new);
	if (ret < 0)
		return ret;

	ret = muic_read(m, reg);
	dev_dbg(m->dev, "reg 0x%02x: 0x%02x -> 0x%02x (=0x%02x)\n", reg, curr, new, ret);
	return ret < 0 ? ret : 0;
}

static int muic_com(struct max77865_muic *m, u8 val)
{
	int ret = muic_update(m, MUIC_REG_CONTROL1, 0xff, val);

	if (ret < 0)
		dev_err(m->dev, "CONTROL1 write 0x%02x failed (%d)\n", val, ret);
	return ret;
}

static int muic_chgdet(struct max77865_muic *m, bool en)
{
	return muic_update(m, MUIC_REG_CONTROL1_BC, CONTROL1_BC_CHGDETEN,
			   en ? CONTROL1_BC_CHGDETEN : 0);
}

static bool muic_vbus(struct max77865_muic *m)
{
	int st = muic_read(m, MUIC_REG_STATUS1_BC);

	return st >= 0 && (st & STATUS1_BC_VBVOLT);
}

/* VBUS now, or within five more reads 10 ms apart */
static bool muic_vbus_valid(struct max77865_muic *m)
{
	int i;

	if (muic_vbus(m))
		return true;

	for (i = 0; i < 5; i++) {
		msleep(10);
		if (muic_vbus(m))
			return true;
	}
	return false;
}

/* returns the value to post, or -1 when attached_dev did not change */
static int muic_set_attached(struct max77865_muic *m, int dev)
{
	if (m->attached_dev == dev)
		return -1;
	m->attached_dev = dev;
	return dev;
}

static void muic_post(struct max77865_muic *m, unsigned long ev, int val)
{
	if (val < 0)
		return;
	blocking_notifier_call_chain(&m->nh, ev, &val);
}

/*
 * The port controller reports no RID change for an ordinary cable, so the
 * USB path is set on a sink attach, and on a VBUS interrupt, once VBUS is valid.
 */
static void muic_usb_path_update(struct max77865_muic *m)
{
	if (m->attached && !m->rprd && muic_vbus_valid(m))
		muic_com(m, COM_USB_AP);
}

static int muic_detach_locked(struct max77865_muic *m)
{
	if (m->rprd)
		muic_chgdet(m, true);
	muic_com(m, COM_OPEN);
	m->rprd = false;
	m->attached = false;
	m->cc_rp = 0;
	return muic_set_attached(m, MAX77865_MUIC_ATTACHED_NONE);
}

/* the phone is the source: no charger detection, USB to the AP */
int max77865_muic_attach_rprd(struct max77865_muic *m)
{
	int post;

	mutex_lock(&m->lock);
	m->attached = true;
	m->rprd = true;
	muic_chgdet(m, false);
	post = muic_set_attached(m, MAX77865_MUIC_ATTACHED_OTG);
	muic_com(m, COM_USB_AP);
	mutex_unlock(&m->lock);

	muic_post(m, MAX77865_MUIC_EV_ATTACHED_DEV, post);
	return 0;
}
EXPORT_SYMBOL_GPL(max77865_muic_attach_rprd);

/* the phone is the sink; an attach also means the port is dry */
int max77865_muic_attach_sink(struct max77865_muic *m, unsigned int cc_rp)
{
	mutex_lock(&m->lock);
	m->attached = true;
	m->cc_rp = cc_rp;
	m->water = false;
	muic_usb_path_update(m);
	mutex_unlock(&m->lock);
	return 0;
}
EXPORT_SYMBOL_GPL(max77865_muic_attach_sink);

/*
 * A charger's VBUS that outlives the CC detach keeps the path. While the
 * phone is the source, the VBUS seen is still its own boost, which the port
 * controller switches off after this call, so the rule does not apply.
 */
int max77865_muic_detach(struct max77865_muic *m)
{
	int post = -1;

	mutex_lock(&m->lock);
	if (!m->rprd && muic_vbus(m))
		dev_dbg(m->dev, "valid VBUS, detach ignored\n");
	else
		post = muic_detach_locked(m);
	mutex_unlock(&m->lock);

	muic_post(m, MAX77865_MUIC_EV_ATTACHED_DEV, post);
	return 0;
}
EXPORT_SYMBOL_GPL(max77865_muic_detach);

int max77865_muic_rid(struct max77865_muic *m, unsigned int rid)
{
	int post = -1, chgtyp;
	bool vbus;

	if (rid > RID_OPEN)
		return -EINVAL;

	mutex_lock(&m->lock);
	if (!m->attached) {
		dev_dbg(m->dev, "RID %u without attach, discarded\n", rid);
		goto out;
	}

	switch (rid) {
	case RID_000K:
		muic_com(m, COM_USB_AP);
		break;
	case RID_001K:
		break;
	case RID_255K:
		muic_com(m, COM_USB_AP);
		post = muic_set_attached(m, MAX77865_MUIC_ATTACHED_JIG_USB_OFF);
		break;
	case RID_301K:
		muic_com(m, COM_USB_AP);
		post = muic_set_attached(m, MAX77865_MUIC_ATTACHED_JIG_USB_ON);
		break;
	case RID_523K:
	case RID_619K:
		vbus = muic_vbus(m);
		muic_com(m, m->rustproof ? COM_OPEN : (m->uart_ap ? COM_UART_AP : COM_UART_CP));
		if (rid == RID_523K)
			post = muic_set_attached(m, MAX77865_MUIC_ATTACHED_JIG_UART_OFF);
		else
			post = muic_set_attached(m, vbus ? MAX77865_MUIC_ATTACHED_JIG_UART_ON_VB :
							   MAX77865_MUIC_ATTACHED_JIG_UART_ON);
		break;
	case RID_OPEN:
	case RID_UNDEFINED:
		if (muic_vbus_valid(m)) {
			muic_com(m, COM_USB_AP);
			/* give the BC1.2 detection time before reading the charger type */
			msleep(200);
			chgtyp = muic_read(m, MUIC_REG_STATUS1_BC);
			if (chgtyp < 0)
				break;
			switch (chgtyp & STATUS1_BC_CHGTYP) {
			case CHGTYP_USB:
				post = muic_set_attached(m, MAX77865_MUIC_ATTACHED_USB);
				break;
			case CHGTYP_CDP:
				post = muic_set_attached(m, MAX77865_MUIC_ATTACHED_CDP);
				break;
			case CHGTYP_DEDICATED:
				post = muic_set_attached(m, MAX77865_MUIC_ATTACHED_TA);
				break;
			default:
				break;
			}
		} else {
			post = muic_detach_locked(m);
		}
		break;
	}
out:
	mutex_unlock(&m->lock);

	muic_post(m, MAX77865_MUIC_EV_ATTACHED_DEV, post);
	return 0;
}
EXPORT_SYMBOL_GPL(max77865_muic_rid);

void max77865_muic_set_water(struct max77865_muic *m, bool water)
{
	mutex_lock(&m->lock);
	m->water = water;
	mutex_unlock(&m->lock);
}
EXPORT_SYMBOL_GPL(max77865_muic_set_water);

struct regmap *max77865_muic_regmap(struct max77865_muic *m)
{
	return m->regmap;
}
EXPORT_SYMBOL_GPL(max77865_muic_regmap);

struct mutex *max77865_muic_lock(struct max77865_muic *m)
{
	return &m->lock;
}
EXPORT_SYMBOL_GPL(max77865_muic_lock);

int max77865_muic_set_hv_irq(struct max77865_muic *m,
			     void (*hv_irq)(void *ctx, u8 int_bc, u8 int_fc, u8 int_gp),
			     void *ctx)
{
	mutex_lock(&m->lock);
	m->hv_irq = hv_irq;
	m->hv_ctx = hv_irq ? ctx : NULL;
	mutex_unlock(&m->lock);
	return 0;
}
EXPORT_SYMBOL_GPL(max77865_muic_set_hv_irq);

int max77865_muic_attached_dev(struct max77865_muic *m)
{
	int dev;

	mutex_lock(&m->lock);
	dev = m->attached_dev;
	mutex_unlock(&m->lock);
	return dev;
}
EXPORT_SYMBOL_GPL(max77865_muic_attached_dev);

/* start a manual charger detection; a plain read-modify-write, no read-back */
int max77865_muic_run_chgdet(struct max77865_muic *m)
{
	int val, ret;

	mutex_lock(&m->lock);
	val = muic_read(m, MUIC_REG_CONTROL1_BC);
	ret = val < 0 ? val : muic_write(m, MUIC_REG_CONTROL1_BC, val | CONTROL1_BC_CHGDETMAN);
	mutex_unlock(&m->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(max77865_muic_run_chgdet);

unsigned int max77865_muic_cc_rp(struct max77865_muic *m)
{
	unsigned int rp;

	mutex_lock(&m->lock);
	rp = m->cc_rp;
	mutex_unlock(&m->lock);
	return rp;
}
EXPORT_SYMBOL_GPL(max77865_muic_cc_rp);

bool max77865_muic_water(struct max77865_muic *m)
{
	bool water;

	mutex_lock(&m->lock);
	water = m->water;
	mutex_unlock(&m->lock);
	return water;
}
EXPORT_SYMBOL_GPL(max77865_muic_water);

int max77865_muic_register_notifier(struct max77865_muic *m, struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&m->nh, nb);
}
EXPORT_SYMBOL_GPL(max77865_muic_register_notifier);

int max77865_muic_unregister_notifier(struct max77865_muic *m, struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&m->nh, nb);
}
EXPORT_SYMBOL_GPL(max77865_muic_unregister_notifier);

/* takes a reference on the MUIC's I2C client; release it with max77865_muic_put() */
struct max77865_muic *max77865_muic_get(struct device_node *np)
{
	struct i2c_client *client = of_find_i2c_device_by_node(np);
	struct max77865_muic *m;

	if (!client)
		return ERR_PTR(-EPROBE_DEFER);

	m = client->dev.driver == &max77865_muic_driver.driver ? i2c_get_clientdata(client) : NULL;
	if (!m) {
		put_device(&client->dev);
		return ERR_PTR(-EPROBE_DEFER);
	}
	return m;
}
EXPORT_SYMBOL_GPL(max77865_muic_get);

void max77865_muic_put(struct max77865_muic *m)
{
	put_device(m->dev);
}
EXPORT_SYMBOL_GPL(max77865_muic_put);

struct max77865_muic *max77865_muic_from_aux(struct auxiliary_device *adev)
{
	return dev_get_drvdata(adev->dev.parent);
}
EXPORT_SYMBOL_GPL(max77865_muic_from_aux);

static void muic_restore(struct max77865_muic *m)
{
	int ccdet;

	muic_write(m, MUIC_REG_INTMASK_MAIN, INTMASK_MAIN_RESTORE);
	muic_write(m, MUIC_REG_INTMASK_BC, INTMASK_BC_RESTORE);
	muic_write(m, MUIC_REG_INTMASK_FC, INTMASK_FC_RESTORE);
	muic_write(m, MUIC_REG_INTMASK_GP, INTMASK_GP_RESTORE);
	muic_write(m, MUIC_REG_CCDET, CCDET_RESTORE);
	ccdet = muic_read(m, MUIC_REG_CCDET);
	dev_info(m->dev, "chip reset, interrupt masks restored, CCDET 0x%02x\n", ccdet);
}

static irqreturn_t max77865_muic_irq(int irq, void *data)
{
	struct max77865_muic *m = data;
	int imain, bc, fc, gp, post = -1;
	bool reset = false;

	mutex_lock(&m->lock);

	imain = muic_read(m, MUIC_REG_INT_MAIN);
	bc = muic_read(m, MUIC_REG_INT_BC);
	fc = muic_read(m, MUIC_REG_INT_FC);
	gp = muic_read(m, MUIC_REG_INT_GP);
	muic_write(m, MUIC_REG_INT_MAIN, INT_MAIN_CLEAR);
	muic_write(m, MUIC_REG_INT_BC, INT_BC_CLEAR);
	muic_write(m, MUIC_REG_INT_FC, INT_FC_CLEAR);
	muic_write(m, MUIC_REG_INT_GP, INT_GP_CLEAR);

	if (imain < 0 || bc < 0 || fc < 0 || gp < 0) {
		dev_err(m->dev, "interrupt status read failed [0x%x 0x%x 0x%x 0x%x]\n",
			imain, bc, fc, gp);
		goto out;
	}
	dev_dbg(m->dev, "intr [0x%x 0x%x 0x%x 0x%x]\n", imain, bc, fc, gp);

	if (!imain && !bc && !fc && !gp)
		goto out;

	if (gp & INT_GP_RESET) {
		muic_restore(m);
		post = muic_set_attached(m, MAX77865_MUIC_ATTACHED_NONE);
		reset = true;
		goto out;
	}

	if (bc & INT_BC_VBUS)
		muic_usb_path_update(m);

	if (m->hv_irq)
		m->hv_irq(m->hv_ctx, bc, fc, gp);
out:
	mutex_unlock(&m->lock);

	muic_post(m, MAX77865_MUIC_EV_ATTACHED_DEV, post);
	if (reset)
		blocking_notifier_call_chain(&m->nh, MAX77865_MUIC_EV_RESET, NULL);
	return IRQ_HANDLED;
}

/* the pmic_info= boot parameter, 12 bits, 0 when absent */
static unsigned int max77865_muic_pmic_info(void)
{
	static const char key[] = "pmic_info=";
	char *p = saved_command_line;
	int val = 0;

	while ((p = strstr(p, key))) {
		if (p == saved_command_line || p[-1] == ' ') {
			p += sizeof(key) - 1;
			get_option(&p, &val);
			break;
		}
		p += sizeof(key) - 1;
	}
	return val & 0xfff;
}

static const struct regmap_config max77865_muic_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = MUIC_REG_MAX,
	.cache_type = REGCACHE_NONE,	/* shared with the "hv" driver */
};

static int max77865_muic_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct max77865_muic *m;
	int id, ret;

	m = devm_kzalloc(dev, sizeof(*m), GFP_KERNEL);
	if (!m)
		return -ENOMEM;

	m->dev = dev;
	m->client = client;
	ret = devm_mutex_init(dev, &m->lock);
	if (ret)
		return ret;
	BLOCKING_INIT_NOTIFIER_HEAD(&m->nh);

	m->regmap = devm_regmap_init_i2c(client, &max77865_muic_regmap_config);
	if (IS_ERR(m->regmap))
		return dev_err_probe(dev, PTR_ERR(m->regmap), "regmap init failed\n");

	m->pmic_info = max77865_muic_pmic_info();
	m->uart_ap = m->pmic_info & SWITCH_SEL_UART_MASK;
	m->rustproof = !(m->pmic_info & SWITCH_SEL_RUSTPROOF_MASK);

	/* the chip keeps its reset values: nothing is written at probe */
	id = muic_read(m, MUIC_REG_ID);
	if (id < 0)
		return dev_err_probe(dev, id, "no answer at 0x%02x\n", client->addr);
	dev_dbg(dev, "id 0x%02x pmic_info 0x%03x\n", id, m->pmic_info);

	i2c_set_clientdata(client, m);

	if (!devm_auxiliary_device_create(dev, "hv", NULL))
		return dev_err_probe(dev, -ENOMEM, "cannot create the hv auxiliary device\n");

	ret = devm_request_threaded_irq(dev, client->irq, NULL, max77865_muic_irq, IRQF_ONESHOT,
					"muic-irq", m);
	if (ret)
		return dev_err_probe(dev, ret, "cannot request IRQ %d\n", client->irq);

	return 0;
}

static const struct of_device_id max77865_muic_of_match[] = {
	{ .compatible = "maxim,max77865-muic" },
	{ }
};
MODULE_DEVICE_TABLE(of, max77865_muic_of_match);

static struct i2c_driver max77865_muic_driver = {
	.driver = {
		.name = "max77865-muic",
		.of_match_table = max77865_muic_of_match,
		/* the port controller and the "hv" driver hold pointers to it */
		.suppress_bind_attrs = true,
	},
	.probe = max77865_muic_probe,
};
module_i2c_driver(max77865_muic_driver);

MODULE_DESCRIPTION("Maxim MAX77865 MUIC base driver");
MODULE_LICENSE("GPL");
