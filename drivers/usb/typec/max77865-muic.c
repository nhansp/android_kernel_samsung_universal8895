// SPDX-License-Identifier: GPL-2.0
/*
 * MAX77865 MUIC base driver (Samsung Galaxy Note 8, greatlte): the D+/D- COM switch
 * policy and the BC1.2 detection enable that the stock MUIC ran on the S2MM005 port
 * controller's events and on its own VBUS interrupt, plus the interrupt read/clear.
 * Written on the stock register protocol (S:drivers/muic/universal/muic_ccic.c,
 * muic_regmap_max77865.c, muic_task_max77865.c; spec L6G 5.2). No high-voltage
 * (AFC/QC) register is written here: that is the auxiliary "hv" device's driver.
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

/* registers, S:drivers/muic/universal/muic_regmap_max77865.c:64-86 */
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
/* HVTXBYTE / HVRXBYTE window of the AFC module, S:include/linux/mfd/max77865-private.h:141-167 (L6H) */
#define MUIC_REG_MAX		0x22

/* fields, muic_regmap_max77865.c:156-176 */
#define STATUS1_BC_VBVOLT	BIT(7)		/* :156 */
#define STATUS1_BC_CHGTYP	GENMASK(1, 0)	/* :160 */
#define CONTROL1_BC_CHGDETMAN	BIT(1)		/* :175, :462-463 */
#define CONTROL1_BC_CHGDETEN	BIT(0)		/* :176 */

/* ChgTyp values, S:drivers/muic/universal/muic_vps.h:9-12 */
#define CHGTYP_USB		0x01
#define CHGTYP_CDP		0x02
#define CHGTYP_DEDICATED	0x03

/*
 * CONTROL1 COM values: COM_VALUE(dm, open, comp) = dm << 3 | dm | open << 6 | comp << 7
 * with _D_OPEN 0, _D_USB 1, _D_UART 3, _D_UART_CP 5, _ID_OPEN 0, _NO_BC_COMP_ON 1
 * (muic_regmap_max77865.c:222-244)
 */
#define COM_OPEN		0x80	/* :239-240 _COM_OPEN(_WITH_V_BUS) */
#define COM_USB_AP		0x89	/* :244 */
#define COM_UART_AP		0x9b	/* :241 */
#define COM_UART_CP		0xad	/* :242 */

/* interrupt clear and chip-reset restore values, S:drivers/muic/universal/muic_regmap_max77865.h:8-28 */
#define INT_MAIN_CLEAR		0x1a
#define INT_BC_CLEAR		0xff
#define INT_FC_CLEAR		0xfc
#define INT_GP_CLEAR		0x1f
#define INT_BC_VBUS		BIT(7)		/* MAX77865_VBUS_MASK */
#define INT_GP_RESET		BIT(1)		/* MAX77865_RESET_MASK */
#define INTMASK_MAIN_RESTORE	0x05
#define INTMASK_BC_RESTORE	0x00
#define INTMASK_FC_RESTORE	0x77
#define INTMASK_GP_RESTORE	0xe0
#define CCDET_RESTORE		0x00

/* pmic_info= boot parameter bits, S:include/linux/muic/muic.h:72-74 */
#define SWITCH_SEL_UART_MASK		0x2
#define SWITCH_SEL_RUSTPROOF_MASK	0x8

/* CCIC RID codes, S:include/linux/ccic/ccic_notifier.h:130-137 */
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
	struct mutex lock;		/* stock muic_mutex, muic_task_max77865.c:270; all state below */
	struct blocking_notifier_head nh;

	unsigned int pmic_info;
	bool uart_ap;
	bool rustproof;

	bool attached;			/* stock mdev_desc.ccic_evt_attached == ATTACH */
	bool rprd;			/* stock mdev_desc.ccic_evt_rprd */
	unsigned int cc_rp;		/* stock is_ccic_afc_enable (the CCIC Rp value) */
	bool water;			/* stock afc_water_disable */
	int attached_dev;

	void (*hv_irq)(void *ctx, u8 int_bc, u8 int_fc, u8 int_gp);
	void *hv_ctx;
};

static struct i2c_driver max77865_muic_driver;

/*
 * Byte access with the stock retry rule: on an I2C error retry after msleep(100),
 * giving up when the retry count exceeds 3 (S:drivers/muic/universal/muic_i2c.c:46-97).
 */
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

/*
 * Stock muic_i2c_guaranteed_wbyte() (muic_i2c.c:99-123): write, read back, and while the
 * read-back differs retry the write after msleep(100), at most 6 more times.
 */
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

/*
 * Stock regmap_write_value() (S:drivers/muic/universal/muic_regmap.c:63-120): read, modify
 * (or overwrite), write only if the value changes, with the guaranteed write, then read back.
 */
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

/* CONTROL1 path switch: stock regmap_com_to() overwrites the register (muic_regmap.c:162-173) */
static int muic_com(struct max77865_muic *m, u8 val)
{
	int ret = muic_update(m, MUIC_REG_CONTROL1, 0xff, val);

	if (ret < 0)
		dev_err(m->dev, "CONTROL1 write 0x%02x failed (%d)\n", val, ret);
	return ret;
}

/* ChgDetEn, stock max77865_enable_chgdet() (muic_regmap_max77865.c:445-460) */
static int muic_chgdet(struct max77865_muic *m, bool en)
{
	return muic_update(m, MUIC_REG_CONTROL1_BC, CONTROL1_BC_CHGDETEN,
			   en ? CONTROL1_BC_CHGDETEN : 0);
}

/* VBVolt of STATUS1_BC (muic_regmap_max77865.c:156; stock read its vps copy, muic_ccic.c:204-207) */
static bool muic_vbus(struct max77865_muic *m)
{
	int st = muic_read(m, MUIC_REG_STATUS1_BC);

	return st >= 0 && (st & STATUS1_BC_VBVOLT);
}

/* stock mdev_is_valid_RID_OPEN(): VBUS now, or within 5 re-reads 10 ms apart (muic_ccic.c:451-466) */
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
 * USB path back to the AP on a sink attach with valid VBUS. Stock reached this through its
 * VBUS interrupt and muic_detect_dev() (muic_task_max77865.c:209,222-226,272-277 ->
 * S:drivers/muic/universal/muic_state.c:280-284 -> muic_apis.c:325-338,430-445), because the
 * S2MM005 posts a RID event only when the RID changes (S:drivers/ccic/s2mm005_cc.c:1094,
 * 1105-1112) and an ordinary cable reads RID_OPEN (spec L6G 5.2.3, V-1).
 */
static void muic_usb_path_update(struct max77865_muic *m)
{
	if (m->attached && !m->rprd && muic_vbus_valid(m))
		muic_com(m, COM_USB_AP);
}

/* stock mdev_handle_ccic_detach() (muic_ccic.c:221-265) minus the HV and jig callbacks */
static int muic_detach_locked(struct max77865_muic *m)
{
	if (m->rprd)
		muic_chgdet(m, true);			/* :233-236 */
	muic_com(m, COM_OPEN);				/* :237 */
	m->rprd = false;
	m->attached = false;
	m->cc_rp = 0;
	return muic_set_attached(m, MAX77865_MUIC_ATTACHED_NONE);
}

/* the phone is source/host: stock muic_handle_ccic_ATTACH() rprd branch (muic_ccic.c:478-479, 532-540) */
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

/*
 * The phone is sink: stock muic_handle_ccic_ATTACH() without rprd (muic_ccic.c:478-485,
 * "CCIC ATTACH means NO WATER" :548-552); the Rp-dependent AFC start (:487-519) is the HV
 * module's. The USB path is set here when VBUS is valid (spec L6G 5.2.3, V-1).
 */
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
 * Stock detach branch (muic_ccic.c:590-601): a charger's VBUS that outlives the CC detach
 * keeps the path. While the phone was the source the VBUS seen is its own boost, still on
 * because the port controller switches it off after this call (S:drivers/ccic/s2mm005_cc.c:
 * 888-891 then :899-901), so the rule is skipped then (spec L6G 5.2.3, V-2).
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

/* stock muic_handle_ccic_RID() (muic_ccic.c:650-715) and mdev_handle_factory_jig() (:606-648) */
int max77865_muic_rid(struct max77865_muic *m, unsigned int rid)
{
	int post = -1, chgtyp;
	bool vbus;

	if (rid > RID_OPEN)
		return -EINVAL;

	mutex_lock(&m->lock);
	if (!m->attached) {				/* :665-668 */
		dev_dbg(m->dev, "RID %u without attach, discarded\n", rid);
		goto out;
	}

	switch (rid) {
	case RID_000K:					/* :674-679 */
		muic_com(m, COM_USB_AP);
		break;
	case RID_001K:					/* :680-682 MHL, discarded */
		break;
	case RID_255K:					/* :614-619 */
		muic_com(m, COM_USB_AP);
		post = muic_set_attached(m, MAX77865_MUIC_ATTACHED_JIG_USB_OFF);	/* :87 */
		break;
	case RID_301K:
		muic_com(m, COM_USB_AP);
		post = muic_set_attached(m, MAX77865_MUIC_ATTACHED_JIG_USB_ON);	/* :88 */
		break;
	case RID_523K:					/* :620-625, uart path S:drivers/muic/universal/ */
	case RID_619K:					/* muic_apis.c:272-312 incl. rustproof -> OPEN */
		vbus = muic_vbus(m);
		muic_com(m, m->rustproof ? COM_OPEN : (m->uart_ap ? COM_UART_AP : COM_UART_CP));
		if (rid == RID_523K)
			post = muic_set_attached(m, MAX77865_MUIC_ATTACHED_JIG_UART_OFF);	/* :89 */
		else
			post = muic_set_attached(m, vbus ? MAX77865_MUIC_ATTACHED_JIG_UART_ON_VB :
							   MAX77865_MUIC_ATTACHED_JIG_UART_ON);	/* :443-446, :90 */
		break;
	case RID_OPEN:					/* :690-708 */
	case RID_UNDEFINED:
		if (muic_vbus_valid(m)) {
			muic_com(m, COM_USB_AP);	/* "USB team's requirement" :697-701 */
			/*
			 * stock mdev_handle_legacy_TA_USB() (:383-417): msleep(200), then the
			 * charger type; ChgTyp 1/2/3 -> USB/CDP/TA (S:drivers/muic/universal/
			 * muic_vps.c:382-391)
			 */
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
			post = muic_detach_locked(m);	/* "RID OPEN + No VBUS = Assume detach" :705-706 */
		}
		break;
	}
out:
	mutex_unlock(&m->lock);

	muic_post(m, MAX77865_MUIC_EV_ATTACHED_DEV, post);
	return 0;
}
EXPORT_SYMBOL_GPL(max77865_muic_rid);

/* CCIC water event to the MUIC: stock afc_water_disable (muic_ccic.c:717-730) */
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

/* stock max77865_run_chgdet(pdesc, 1): ChgDetMan := 1, plain read/write (muic_regmap_max77865.c:465-486) */
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

/* a reference on the MUIC's I2C client; released with max77865_muic_put() */
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

/* stock max77865_restore() after a chip reset (muic_task_max77865.c:149-167) */
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

/* stock muic_irq_thread() + max77865_muic_irq_handler() (muic_task_max77865.c:169-312) */
static irqreturn_t max77865_muic_irq(int irq, void *data)
{
	struct max77865_muic *m = data;
	int imain, bc, fc, gp, post = -1;
	bool reset = false;

	mutex_lock(&m->lock);

	/* read and clear, :180-189 */
	imain = muic_read(m, MUIC_REG_INT_MAIN);
	bc = muic_read(m, MUIC_REG_INT_BC);
	fc = muic_read(m, MUIC_REG_INT_FC);
	gp = muic_read(m, MUIC_REG_INT_GP);
	muic_write(m, MUIC_REG_INT_MAIN, INT_MAIN_CLEAR);
	muic_write(m, MUIC_REG_INT_BC, INT_BC_CLEAR);
	muic_write(m, MUIC_REG_INT_FC, INT_FC_CLEAR);
	muic_write(m, MUIC_REG_INT_GP, INT_GP_CLEAR);

	if (imain < 0 || bc < 0 || fc < 0 || gp < 0) {		/* :191-195 */
		dev_err(m->dev, "interrupt status read failed [0x%x 0x%x 0x%x 0x%x]\n",
			imain, bc, fc, gp);
		goto out;
	}
	dev_dbg(m->dev, "intr [0x%x 0x%x 0x%x 0x%x]\n", imain, bc, fc, gp);

	if (!imain && !bc && !fc && !gp)				/* :203-207 */
		goto out;

	if (gp & INT_GP_RESET) {				/* :214-220 */
		muic_restore(m);
		post = muic_set_attached(m, MAX77865_MUIC_ATTACHED_NONE);
		reset = true;
		goto out;
	}

	if (bc & INT_BC_VBUS)					/* :209, :222-226 */
		muic_usb_path_update(m);

	if (m->hv_irq)						/* HV demux :239-259 (AFC module) */
		m->hv_irq(m->hv_ctx, bc, fc, gp);
out:
	mutex_unlock(&m->lock);

	muic_post(m, MAX77865_MUIC_EV_ATTACHED_DEV, post);
	if (reset)
		blocking_notifier_call_chain(&m->nh, MAX77865_MUIC_EV_RESET, NULL);
	return IRQ_HANDLED;
}

/*
 * pmic_info= of the boot command line: stock __setup("pmic_info=") masked with 0xfff
 * (S:drivers/muic/muic-core.c:436-445); absent -> 0, the stock static default.
 */
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
	.cache_type = REGCACHE_NONE,	/* L3's PHY and the AFC module write these registers too */
};

static int max77865_muic_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct max77865_muic *m;
	int id, ctrl1, ctrl1_bc, st1_bc, intmask_bc, ret;

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

	/* uart path and rustproof, S:drivers/muic/muic-core.c:513-518, muic_task_max77865.c:455-461 */
	m->pmic_info = max77865_muic_pmic_info();
	m->uart_ap = m->pmic_info & SWITCH_SEL_UART_MASK;
	m->rustproof = !(m->pmic_info & SWITCH_SEL_RUSTPROOF_MASK);

	/* stock's MAX77865 regmap init writes nothing (muic_regmap_max77865.c:688-693): read only */
	id = muic_read(m, MUIC_REG_ID);
	if (id < 0)
		return dev_err_probe(dev, id, "no answer at 0x%02x\n", client->addr);
	ctrl1 = muic_read(m, MUIC_REG_CONTROL1);
	ctrl1_bc = muic_read(m, MUIC_REG_CONTROL1_BC);
	st1_bc = muic_read(m, MUIC_REG_STATUS1_BC);
	intmask_bc = muic_read(m, MUIC_REG_INTMASK_BC);
	dev_info(dev, "id 0x%02x ctrl1 0x%02x ctrl1_bc 0x%02x status1_bc 0x%02x intmask_bc 0x%02x pmic_info 0x%03x\n",
		 id, ctrl1, ctrl1_bc, st1_bc, intmask_bc, m->pmic_info);

	/* the API and the AFC module's parent lookup work from here on */
	i2c_set_clientdata(client, m);

	if (!devm_auxiliary_device_create(dev, "hv", NULL))
		return dev_err_probe(dev, -ENOMEM, "cannot create the hv auxiliary device\n");

	/* level low, one-shot, "muic-irq" as stock (muic_task_max77865.c:339-341); trigger from DT */
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
		.suppress_bind_attrs = true,	/* handles are given to the port controller and AFC module */
	},
	.probe = max77865_muic_probe,
};
module_i2c_driver(max77865_muic_driver);

MODULE_DESCRIPTION("Maxim MAX77865 MUIC base driver");
MODULE_LICENSE("GPL");
