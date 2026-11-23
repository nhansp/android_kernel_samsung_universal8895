// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung S2MM005 USB Type-C / USB-PD port controller. Its firmware runs both
 * protocols; the driver reads the state over I2C (16-bit register addresses)
 * and commands the chip through register 0x10.
 */

#include <linux/bitfield.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_wakeup.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/sec_sysfs.h>
#include <linux/sysfs.h>
#include <linux/unaligned.h>
#include <linux/usb/max77865-muic.h>
#include <linux/usb/role.h>
#include <linux/usb/s2mm005.h>
#include <linux/usb/typec.h>
#include <linux/usb/typec_dp.h>
#include <linux/usb/typec_mux.h>
#include <linux/workqueue.h>

#define REG_HW_VER		0x0000
#define REG_SW_VER		0x0008
#define REG_I2C_SLV_CMD		0x0010
#define REG_FUNC_STATE		0x0020
#define REG_INT_STATUS		0x0030
#define REG_RID			0x0050
#define REG_LP_STATE		0x0060
#define REG_TX_SINK_CAPA_MSG	0x0220
#define REG_TX_REQUEST_MSG	0x0240
#define REG_RX_SRC_CAPA_MSG	0x0260
#define REG_FTRIM_RSP		0x0014

/* FUNC_STATE */
#define FS_PD_STATE		GENMASK(7, 0)
#define FS_IS_SOURCE		BIT(25)
#define FS_RP_CURRENTLVL	GENMASK(28, 27)
#define FS_VBUS_CC_SHORT	BIT(29)
#define FS_VBUS_SBU_SHORT	BIT(30)
#define FS_RESET		BIT(31)

/* LP_STATE */
#define LP_AUTO_LP_ENABLE	BIT(0)
#define LP_WATER_DET		BIT(3)
#define LP_RUN_DRY		BIT(5)
#define LP_BOOTING_RUN_DRY	BIT(7)
#define LP_SLEEP_CABLE_DETECT	BIT(8)
#define LP_PDSTATE29_SBU_DONE	BIT(9)

/* MSG_IRQ_STATUS */
#define MSG_GET_SINK_CAP	BIT(8)
#define MSG_DR_SWAP		BIT(9)
#define MSG_PR_SWAP		BIT(10)
#define MSG_SRC_CAPABILITY	BIT(17)

/* AP_REQ_GET_STATUS */
#define AP_REQ_ALT_MODE_BY_I2C	GENMASK(29, 28)

/* PD message fields */
#define MSG_HDR_NUM_OF_OBJ	GENMASK(14, 12)
#define PDO_TYPE		GENMASK(31, 30)
#define PDO_FIXED_VOLTAGE	GENMASK(19, 10)
#define PDO_FIXED_MAX_CURRENT	GENMASK(9, 0)
#define REQ_OBJECT_POSITION	GENMASK(30, 28)
#define AVAILABLE_VOLTAGE	12000
#define UNIT_FOR_VOLTAGE	50
#define UNIT_FOR_CURRENT	10

/* firmware policy engine states */
#define PD_INITIAL_DETACH		0
#define PD_SRC_SEND_CAPABILITIES	3
#define PD_SRC_DISABLED			7
#define PD_SRC_WAIT_NEW_CAPABILITIES	14
#define PD_SNK_WAIT_FOR_CAPABILITIES	17
#define PD_SNK_EVALUATE_CAPABILITY	18
#define PD_SNK_READY			21
#define PD_ERROR_RECOVERY		29
#define PD_PRS_SRC_SNK_TRANSITION_OFF	52
#define PD_PRS_SNK_SRC_SOURCE_ON	64

/* forced port modes */
#define TYPE_C_ATTACH_DFP	1
#define TYPE_C_ATTACH_UFP	2
#define TYPE_C_ATTACH_DRP	3

/* Rp current levels beyond the S2MM005_RP_CURRENT_LEVEL* ones */
#define RP_SBU_CHECK		0
#define RP_ABNORMAL		4

/* resistor ID codes */
#define RID_523K		5
#define RID_619K		6
#define RID_OPEN		7

#define DUAL_ROLE_SET_MODE_WAIT_MS	1500
#define CCIC_I2C_VALUE_INVALID		274
#define FW_CHECK_RETRY			5

/*
 * Version of the firmware image the vendor kernel bundles for this chip. The
 * driver never flashes the chip; the version is only reported.
 */
#define BUNDLED_FW_BOOT		0x07
#define BUNDLED_FW_MAIN0	0x46
#define BUNDLED_FW_MAIN1	0x00
#define BUNDLED_FW_MAIN2	0x0a

struct s2mm005_version {
	u8 main[3];
	u8 boot;
	u8 ver2[4];
};

struct s2mm005 {
	struct device *dev;
	struct i2c_client *client;
	int irq;

	struct mutex io_lock;		/* I2C transfers and xfer_buf */
	u8 xfer_buf[2 + 256];

	/* protects the state below; taken before io_lock and the MUIC lock */
	struct mutex lock;
	u8 pd_state;
	u32 func_state;
	bool is_host;
	bool is_client;
	unsigned int is_dr_swap;
	unsigned int is_pr_swap;
	bool water_det;
	bool run_dry;
	bool booting_run_dry;
	int i2c_err;
	u8 cur_rid;
	u8 prev_rid;
	u8 firm_ver[4];
	bool check_at_booting;
	int try_state;
	enum typec_role pwr_role;
	bool no_vbus_logged;

	/* USB-PD sink status */
	struct s2mm005_pd_status pd;
	bool pd_attached;

	struct fwnode_handle *connector;
	struct typec_capability cap;
	struct typec_port *port;
	struct typec_partner *partner;
	struct usb_role_switch *role_sw;
	struct regulator *vbus;
	bool vbus_on;
	struct gpio_desc *orient;
	struct max77865_muic *muic;
	struct pinctrl *i2c_pinctrl;
	struct pinctrl_state *i2c_default;
	struct pinctrl_state *i2c_reset;
	struct device *ccic_dev;
	/* DisplayPort alternate mode; dp_mux is NULL without a mode switch */
	struct typec_mux *dp_mux;
	bool alt_started;
	u16 svid0;
	u16 svid1;
	u16 vid;
	u16 pid;
	u32 pin_caps;
	int dp_pin;		/* DP_PIN_A..DP_PIN_F, 0 = unknown */
	bool pin_sent;
	bool dp_connected;
	u32 dp_status;		/* raw status VDO */

	struct completion role_done;
	struct delayed_work swap_check;
	struct delayed_work boot_check;
};

/* PD notifications collected under the lock and posted after it is released */
struct s2mm005_pd_note {
	unsigned int n;
	struct {
		unsigned long event;
		struct s2mm005_pd_status status;
	} ev[2];			/* a PR swap detach, then the pass's own event */
};

static struct i2c_driver s2mm005_driver;

/* a failed transfer is repeated up to five times */
static int s2mm005_read(struct s2mm005 *s, u16 reg, void *buf, u16 len)
{
	u8 wbuf[2] = { reg >> 8, reg & 0xff };
	struct i2c_msg msg[2] = {
		{ .addr = s->client->addr, .flags = 0, .len = 2, .buf = wbuf },
		{ .addr = s->client->addr, .flags = I2C_M_RD, .len = len, .buf = buf },
	};
	int ret, retry = 0;

	mutex_lock(&s->io_lock);
	do {
		ret = i2c_transfer(s->client->adapter, msg, ARRAY_SIZE(msg));
	} while (ret < 0 && retry++ < 5);
	mutex_unlock(&s->io_lock);

	if (ret < 0)
		dev_err(s->dev, "i2c read16 fail reg:0x%x error %d\n", reg, ret);
	return ret;
}

/* one byte, not retried */
static int s2mm005_read_byte16(struct s2mm005 *s, u16 reg, u8 *val)
{
	u8 wbuf[2] = { reg >> 8, reg & 0xff };
	struct i2c_msg msg[2] = {
		{ .addr = s->client->addr, .flags = 0, .len = 2, .buf = wbuf },
		{ .addr = s->client->addr, .flags = I2C_M_RD, .len = 1, .buf = val },
	};
	int ret;

	mutex_lock(&s->io_lock);
	ret = i2c_transfer(s->client->adapter, msg, ARRAY_SIZE(msg));
	mutex_unlock(&s->io_lock);

	if (ret < 0)
		dev_err(s->dev, "i2c read16 fail reg(0x%x), error %d\n", reg, ret);
	return ret;
}

/* at most 256 data bytes, retried like a read */
static int s2mm005_write(struct s2mm005 *s, u16 reg, const u8 *val, u16 len)
{
	struct i2c_msg msg = {
		.addr = s->client->addr, .flags = 0, .len = len + 2, .buf = s->xfer_buf,
	};
	int ret, retry = 0;

	if (len > 256) {
		dev_err(s->dev, "I2C error, over the size %d\n", len);
		return -EIO;
	}

	mutex_lock(&s->io_lock);
	s->xfer_buf[0] = reg >> 8;
	s->xfer_buf[1] = reg & 0xff;
	memcpy(&s->xfer_buf[2], val, len);
	do {
		ret = i2c_transfer(s->client->adapter, &msg, 1);
	} while (ret < 0 && retry++ < 5);
	mutex_unlock(&s->io_lock);

	if (ret < 0)
		dev_err(s->dev, "i2c write fail reg:0x%x error %d\n", reg, ret);
	return ret;
}

static int s2mm005_cmd(struct s2mm005 *s, const u8 *cmd, u16 len)
{
	return s2mm005_write(s, REG_I2C_SLV_CMD, cmd, len);
}

/* not retried */
static void s2mm005_int_clear(struct s2mm005 *s)
{
	u8 wbuf[3] = { REG_I2C_SLV_CMD >> 8, REG_I2C_SLV_CMD & 0xff, 0x01 };
	struct i2c_msg msg = { .addr = s->client->addr, .flags = 0, .len = 3, .buf = wbuf };
	int ret;

	mutex_lock(&s->io_lock);
	ret = i2c_transfer(s->client->adapter, &msg, 1);
	mutex_unlock(&s->io_lock);

	if (ret < 0)
		dev_err(s->dev, "i2c write fail reg(0x10:1), error %d\n", ret);
}

/* flash read command, then one read, not retried */
static int s2mm005_read_flash(struct s2mm005 *s, u16 reg, u8 *buf, u16 len)
{
	static const u8 cmd[] = { 0xaa };
	u8 wbuf[2] = { reg >> 8, reg & 0xff };
	struct i2c_msg msg[2] = {
		{ .addr = s->client->addr, .flags = 0, .len = 2, .buf = wbuf },
		{ .addr = s->client->addr, .flags = I2C_M_RD, .len = len, .buf = buf },
	};
	int ret;

	udelay(20);
	s2mm005_cmd(s, cmd, sizeof(cmd));
	udelay(20);

	mutex_lock(&s->io_lock);
	ret = i2c_transfer(s->client->adapter, msg, ARRAY_SIZE(msg));
	mutex_unlock(&s->io_lock);

	if (ret < 0)
		dev_err(s->dev, "i2c read16 fail reg:0x%x error %d\n", reg, ret);
	return ret;
}

/* dummy reads wake the chip up before a command */
static void s2mm005_wake(struct s2mm005 *s)
{
	u8 dummy;
	int i;

	for (i = 0; i < 5; i++)
		s2mm005_read(s, 0x8, &dummy, 1);
	udelay(10);
}

static void s2mm005_manual_lpm(struct s2mm005 *s, u8 lpm)
{
	u8 cmd[2] = { 0x0f, lpm };

	dev_dbg(s->dev, "manual LPM cmd=0x%x (fw=0x%x)\n", lpm, s->firm_ver[2]);
	s2mm005_wake(s);
	s2mm005_cmd(s, cmd, sizeof(cmd));
}

static void s2mm005_reset(struct s2mm005 *s)
{
	static const u8 cmd[] = { 0x02, 0x01, 0x1c, 0x10, 0x01 };

	s2mm005_wake(s);
	s2mm005_cmd(s, cmd, sizeof(cmd));
	msleep(100);				/* reset stable time */
}

/* SCL and SDA held low for 10 ms through the bus controller's "hard_reset" pin state */
static void s2mm005_hard_reset(struct s2mm005 *s)
{
	struct i2c_adapter *adap = s->client->adapter;

	i2c_lock_bus(adap, I2C_LOCK_ROOT_ADAPTER);
	if (pinctrl_select_state(s->i2c_pinctrl, s->i2c_reset))
		dev_err(s->dev, "could not set reset pins\n");
	usleep_range(10 * 1000, 10 * 1000);
	if (pinctrl_select_state(s->i2c_pinctrl, s->i2c_default))
		dev_err(s->dev, "could not set default pins\n");
	usleep_range(8 * 1000, 8 * 1000);
	i2c_unlock_bus(adap, I2C_LOCK_ROOT_ADAPTER);
}

static void s2mm005_rprd_mode_change(struct s2mm005 *s, int mode)
{
	u8 detach[2] = { 0x03, 0x12 };
	u8 cmd[5];

	dev_dbg(s->dev, "rprd mode change 0x%x\n", mode);
	switch (mode) {
	case TYPE_C_ATTACH_DFP:
	case TYPE_C_ATTACH_UFP:
		s2mm005_cmd(s, detach, sizeof(detach));
		msleep(1000);
		cmd[0] = 0x03;
		cmd[1] = mode == TYPE_C_ATTACH_DFP ? 0x13 : 0x14;
		s2mm005_cmd(s, cmd, 2);
		break;
	case TYPE_C_ATTACH_DRP:
		cmd[0] = 0x02;
		cmd[1] = 0x01;
		cmd[2] = 0x00;
		cmd[3] = 0x50;
		cmd[4] = TYPE_C_ATTACH_DRP;
		s2mm005_cmd(s, cmd, 5);
		break;
	}
}

static void s2mm005_get_hwversion(struct s2mm005 *s, struct s2mm005_version *v)
{
	s2mm005_read_flash(s, REG_HW_VER + 0x0, &v->boot, 1);
	s2mm005_read_flash(s, REG_HW_VER + 0x1, v->main, 3);
	s2mm005_read_flash(s, REG_HW_VER + 0x4, v->ver2, 4);
}

static bool s2mm005_valid_main(const u8 *main)
{
	return !(main[0] == 0xff && main[1] == 0xff) && !(main[0] == 0x00 && main[1] == 0x00);
}

static void s2mm005_get_swversion(struct s2mm005 *s, struct s2mm005_version *v)
{
	int i;

	for (i = 0; i < FW_CHECK_RETRY; i++) {
		s2mm005_read_flash(s, REG_SW_VER + 0x0, &v->boot, 1);
		if (v->boot == 0x7)
			break;
	}
	for (i = 0; i < FW_CHECK_RETRY; i++) {
		s2mm005_read_flash(s, REG_SW_VER + 0x1, v->main, 3);
		if (s2mm005_valid_main(v->main))
			break;
	}
	for (i = 0; i < FW_CHECK_RETRY; i++)
		s2mm005_read_flash(s, REG_SW_VER + 0x4, v->ver2, 4);
}

static void s2mm005_store_swversion(struct s2mm005 *s, const struct s2mm005_version *v)
{
	s->firm_ver[0] = v->main[2];
	s->firm_ver[1] = v->main[1];
	s->firm_ver[2] = v->main[0];
	s->firm_ver[3] = v->boot;
}

/* re-read a firmware version that looks invalid */
static int s2mm005_fw_ver_check(struct s2mm005 *s)
{
	struct s2mm005_version hw = { }, sw = { };

	if (!((s->firm_ver[1] == 0xff && s->firm_ver[2] == 0xff) ||
	      (s->firm_ver[1] == 0x00 && s->firm_ver[2] == 0x00) || s->firm_ver[3] != 0x07))
		return 0;

	s2mm005_get_hwversion(s, &hw);
	s2mm005_get_swversion(s, &sw);
	dev_err(s->dev, "chip hw %02x.%02x.%02x boot %u, sw %02x.%02x.%02x boot %u\n",
		hw.main[2], hw.main[1], hw.main[0], hw.boot,
		sw.main[2], sw.main[1], sw.main[0], sw.boot);
	if (!s2mm005_valid_main(sw.main) || sw.boot != 0x07) {
		dev_err(s->dev, "Invalid FW version\n");
		return -EINVAL;
	}
	s2mm005_store_swversion(s, &sw);
	return 0;
}

static void s2mm005_partner_detach(struct s2mm005 *s)
{
	if (s->partner) {
		typec_unregister_partner(s->partner);
		s->partner = NULL;
	}
}

static void s2mm005_partner_attach(struct s2mm005 *s, enum typec_data_role role)
{
	struct typec_partner_desc desc = {
		.usb_pd = false,
		.accessory = TYPEC_ACCESSORY_NONE,
		.identity = NULL,
	};

	if (s->try_state)
		complete(&s->role_done);

	s2mm005_partner_detach(s);
	typec_set_pwr_opmode(s->port, TYPEC_PWR_MODE_USB);
	typec_set_data_role(s->port, role);
	s->partner = typec_register_partner(s->port, &desc);
	if (IS_ERR(s->partner)) {
		dev_err(s->dev, "cannot register the partner (%ld)\n", PTR_ERR(s->partner));
		s->partner = NULL;
	}
}

static void s2mm005_usb_role(struct s2mm005 *s, enum usb_role role)
{
	int ret = usb_role_switch_set_role(s->role_sw, role);

	if (ret)
		dev_err(s->dev, "cannot set USB role %d (%d)\n", role, ret);
}

/* VBUS is the charger's OTG boost; switching it on needs CONFIG_TYPEC_S2MM005_OTG_VBUS */
static void s2mm005_vbus(struct s2mm005 *s, bool on)
{
	int ret;

	if (!on) {
		if (s->vbus_on) {
			regulator_disable(s->vbus);
			s->vbus_on = false;
		}
		return;
	}

	if (!IS_ENABLED(CONFIG_TYPEC_S2MM005_OTG_VBUS) || !s->vbus) {
		if (!s->no_vbus_logged)
			dev_info(s->dev, "no vbus-supply, host role without VBUS\n");
		s->no_vbus_logged = true;
		return;
	}
	if (s->vbus_on)
		return;

	ret = regulator_enable(s->vbus);
	if (ret)
		dev_err(s->dev, "cannot switch VBUS on (%d)\n", ret);
	else
		s->vbus_on = true;
}

/* the orientation output is high for a reversed plug (CC2); read before the role is set */
static void s2mm005_orientation(struct s2mm005 *s)
{
	int level, ret;

	if (!s->orient)
		return;

	level = gpiod_get_raw_value_cansleep(s->orient);
	if (level < 0) {
		dev_err(s->dev, "cannot read the orientation input (%d)\n", level);
		return;
	}
	ret = typec_set_orientation(s->port, level ? TYPEC_ORIENTATION_REVERSE :
						      TYPEC_ORIENTATION_NORMAL);
	if (ret)
		dev_err(s->dev, "cannot set the orientation (%d)\n", ret);
}

static void s2mm005_uevent(struct s2mm005 *s, const char *event)
{
	char buf[16];
	char *envp[] = { buf, NULL };

	if (!s->ccic_dev)
		return;
	strscpy(buf, event, sizeof(buf));
	kobject_uevent_env(&s->ccic_dev->kobj, KOBJ_CHANGE, envp);
}

static u32 s2mm005_water(struct s2mm005 *s)
{
	u8 buf[4] = { };
	u32 lp;

	s->i2c_err = s2mm005_read(s, REG_LP_STATE, buf, 4);
	lp = get_unaligned_le32(buf);
	dev_dbg(s->dev, "WATER reg:0x%02X WATER=%d DRY=%d\n", buf[0],
		!!(lp & LP_WATER_DET), !!(lp & LP_RUN_DRY));

	if (s->i2c_err < 0) {
		lp &= ~(LP_WATER_DET | LP_BOOTING_RUN_DRY);
		lp |= LP_RUN_DRY | (s->booting_run_dry ? LP_BOOTING_RUN_DRY : 0);
	} else if (!(lp & (LP_WATER_DET | LP_RUN_DRY | LP_BOOTING_RUN_DRY))) {
		s->i2c_err = s2mm005_read(s, REG_LP_STATE, buf, 4);
		lp = get_unaligned_le32(buf);
		dev_dbg(s->dev, "Re: WATER reg:0x%02X WATER=%d DRY=%d\n", buf[0],
			!!(lp & LP_WATER_DET), !!(lp & LP_RUN_DRY));
		if (!(lp & LP_AUTO_LP_ENABLE) && ((lp & LP_WATER_DET) || !(lp & LP_RUN_DRY)))
			s->i2c_err = -CCIC_I2C_VALUE_INVALID;
		if (s->i2c_err < 0) {
			lp &= ~(LP_WATER_DET | LP_BOOTING_RUN_DRY);
			lp |= LP_RUN_DRY | (s->booting_run_dry ? LP_BOOTING_RUN_DRY : 0);
		}
	}

	if ((lp & LP_RUN_DRY) && !s->run_dry) {
		dev_info(s->dev, "== WATER RUN-DRY DETECT ==\n");
		s2mm005_uevent(s, "CCIC=DRY");
	}
	s->run_dry = lp & LP_RUN_DRY;

	if ((lp & LP_WATER_DET) && !s->water_det) {
		dev_info(s->dev, "== WATER DETECT ==\n");
		s2mm005_uevent(s, "CCIC=WATER");
		max77865_muic_set_water(s->muic, true);
	}
	s->water_det = lp & LP_WATER_DET;

	return lp;
}

static void s2mm005_dp_exit(struct s2mm005 *s);
static void s2mm005_alt_mode_start(struct s2mm005 *s);
static void s2mm005_dock_uevent(struct s2mm005 *s, int type);

static void s2mm005_src_attach(struct s2mm005 *s)
{
	if (s->is_client) {
		max77865_muic_detach(s->muic);
		s2mm005_usb_role(s, USB_ROLE_NONE);
		s2mm005_partner_detach(s);
		s->is_client = false;
		msleep(300);
	}
	if (s->is_host)
		return;

	s2mm005_orientation(s);
	max77865_muic_attach_rprd(s->muic);
	s->is_host = true;
	s->pwr_role = TYPEC_SOURCE;
	typec_set_pwr_role(s->port, TYPEC_SOURCE);
	s2mm005_partner_attach(s, TYPEC_HOST);
	s2mm005_usb_role(s, USB_ROLE_HOST);
	s2mm005_vbus(s, true);
	dev_dbg(s->dev, "attach src (pd_state %d), role host, vbus %s\n", s->pd_state,
		s->vbus_on ? "on" : "off");
}

static void s2mm005_snk_attach(struct s2mm005 *s, u32 lp, u32 fs)
{
	unsigned int rp;

	if (s->is_host) {
		s2mm005_dp_exit(s);
		max77865_muic_detach(s->muic);
		s2mm005_vbus(s, false);
		s2mm005_usb_role(s, USB_ROLE_NONE);
		s2mm005_partner_detach(s);
		s->is_host = false;
		msleep(300);
	}

	if (lp & LP_PDSTATE29_SBU_DONE)
		rp = (fs & (FS_VBUS_CC_SHORT | FS_VBUS_SBU_SHORT)) ? RP_ABNORMAL :
			FIELD_GET(FS_RP_CURRENTLVL, fs);
	else
		rp = RP_SBU_CHECK;
	max77865_muic_attach_sink(s->muic, rp);

	if (s->is_client || s->is_host)
		return;

	s2mm005_orientation(s);
	s->is_client = true;
	s->pwr_role = TYPEC_SINK;
	typec_set_pwr_role(s->port, TYPEC_SINK);
	s2mm005_partner_attach(s, TYPEC_DEVICE);
	s2mm005_usb_role(s, USB_ROLE_DEVICE);
	dev_dbg(s->dev, "attach snk (pd_state %d), rp %u, role device\n", s->pd_state, rp);
}

/* dock types reported in SWITCH_STATE */
#define DOCK_DETACHED		0
#define DOCK_HMT		105
#define DOCK_MPA		109
#define DOCK_DEX		110
#define DOCK_HDMI		111
#define DOCK_UVDM		113
#define DOCK_DEXPAD		114
#define DOCK_NEW		200

static void s2mm005_detach(struct s2mm005 *s)
{
	s->is_dr_swap = 0;
	s->is_pr_swap = 0;

	s2mm005_dp_exit(s);
	s->vid = 0;
	s->pid = 0;
	s2mm005_dock_uevent(s, DOCK_DETACHED);

	/* before VBUS is switched off */
	max77865_muic_detach(s->muic);

	if (!s->is_host && !s->is_client)
		return;

	if (s->is_host || s->pwr_role == TYPEC_SOURCE)
		s2mm005_vbus(s, false);
	s->is_host = false;
	s->is_client = false;
	s->pwr_role = TYPEC_SINK;
	typec_set_pwr_role(s->port, TYPEC_SINK);
	typec_set_data_role(s->port, TYPEC_DEVICE);
	typec_set_pwr_opmode(s->port, TYPEC_PWR_MODE_USB);
	s2mm005_partner_detach(s);
	if (s->orient)
		typec_set_orientation(s->port, TYPEC_ORIENTATION_NONE);
	s2mm005_usb_role(s, USB_ROLE_NONE);
	dev_dbg(s->dev, "detach\n");

	if (!s->try_state)
		s2mm005_rprd_mode_change(s, TYPE_C_ATTACH_DRP);
}

static void s2mm005_cc_attach(struct s2mm005 *s, bool *attach_done)
{
	u8 buf[4];
	u32 lp, fs;

	*attach_done = false;
	lp = s2mm005_water(s);

	if (s->water_det)
		return;

	if (!s->run_dry || !s->booting_run_dry) {
		dev_info(s->dev, "Water? No Dry\n");
		s2mm005_uevent(s, "CCIC=WATER");
		max77865_muic_set_water(s->muic, true);
		if (s->i2c_err >= 0 && s2mm005_read(s, REG_FUNC_STATE, buf, 4) >= 0)
			dev_dbg(s->dev, "PD_State:%02d\n", buf[0]);
		return;
	}

	if (s->i2c_err < 0) {
		if (s->pd_state == PD_INITIAL_DETACH)
			return;
		fs = 0;
	} else {
		memset(buf, 0, sizeof(buf));
		s2mm005_read(s, REG_FUNC_STATE, buf, 4);
		fs = get_unaligned_le32(buf);
	}
	s->pd_state = FIELD_GET(FS_PD_STATE, fs);
	s->func_state = fs;
	dev_dbg(s->dev, "func_state 0x%x pd_state %d\n", fs, s->pd_state);

	if (fs & FS_RESET) {
		dev_info(s->dev, "ccic reset detected\n");
		if (!(lp & LP_AUTO_LP_ENABLE))
			s2mm005_manual_lpm(s, 0x06);
		s->alt_started = false;
		s2mm005_alt_mode_start(s);
	}

	if (s->pd_state == PD_SRC_WAIT_NEW_CAPABILITIES && (lp & LP_SLEEP_CABLE_DETECT)) {
		s2mm005_manual_lpm(s, 0x0d);
		return;
	}

	if (s->pd_state == PD_INITIAL_DETACH) {
		s2mm005_detach(s);
		return;
	}

	*attach_done = true;
	if (s->is_dr_swap || s->is_pr_swap) {
		dev_dbg(s->dev, "pd_state ignored during a %s\n",
			s->is_dr_swap ? "dr_swap" : "pr_swap");
		return;
	}

	switch (s->pd_state) {
	case PD_SRC_SEND_CAPABILITIES ... PD_SRC_DISABLED:
		s2mm005_src_attach(s);
		break;
	case PD_SNK_WAIT_FOR_CAPABILITIES:
	case PD_SNK_EVALUATE_CAPABILITY:
	case PD_SNK_READY:
	case PD_ERROR_RECOVERY:
		s2mm005_snk_attach(s, lp, fs);
		break;
	case PD_PRS_SRC_SNK_TRANSITION_OFF:
		s2mm005_vbus(s, false);
		break;
	case PD_PRS_SNK_SRC_SOURCE_ON:
		s2mm005_vbus(s, true);
		break;
	default:
		break;
	}
}

static void s2mm005_dr_swap(struct s2mm005 *s)
{
	s->is_dr_swap++;
	dev_dbg(s->dev, "is_dr_swap count : 0x%x\n", s->is_dr_swap);

	if (s->dp_connected)
		return;

	if (s->is_host) {
		s2mm005_usb_role(s, USB_ROLE_NONE);
		s2mm005_partner_detach(s);
		msleep(300);
		max77865_muic_attach_sink(s->muic, RP_SBU_CHECK);
		s2mm005_partner_attach(s, TYPEC_DEVICE);
		s2mm005_usb_role(s, USB_ROLE_DEVICE);
		s->is_host = false;
		s->is_client = true;
	} else if (s->is_client) {
		s2mm005_usb_role(s, USB_ROLE_NONE);
		s2mm005_partner_detach(s);
		msleep(300);
		max77865_muic_attach_rprd(s->muic);
		s2mm005_partner_attach(s, TYPEC_HOST);
		s2mm005_usb_role(s, USB_ROLE_HOST);
		s->is_host = true;
		s->is_client = false;
	}
}

/*
 * DisplayPort alternate mode: the firmware runs the VDM exchange and the
 * driver follows its VDM interrupts while host. The DP state goes to the
 * mode switch; no partner altmode is registered.
 */

/* VDM message windows */
#define DP_REG_DIS_ID		0x0520
#define DP_REG_DIS_SVID		0x0540
#define DP_REG_MODE		0x0560
#define DP_REG_ENTER_MODE	0x0580
#define DP_REG_ATTENTION	0x05C0
#define DP_REG_STATUS_UPDATE	0x0600
#define DP_REG_CONFIGURE	0x0620

/* VDM_IRQ_STATUS */
#define DP_VDM_DISCOVER_ID	BIT(1)
#define DP_VDM_DISCOVER_SVIDS	BIT(2)
#define DP_VDM_DISCOVER_MODES	BIT(3)
#define DP_VDM_ENTER_MODE	BIT(4)
#define DP_VDM_ATTENTION	BIT(6)
#define DP_VDM_STATUS_UPDATE	BIT(16)
#define DP_VDM_CONFIGURE	BIT(17)

/* alternate mode commands */
#define DP_CMD_MODE_IFACE	0x03
#define DP_CMD_NEXT_STATE	0x02
#define DP_CMD_SAMSUNG_MODE	89
#define DP_CMD_ALT_MODE_REQ	0x30
#define DP_CMD_SVID_SELECT	0x07
#define DP_CMD_START		0x31
#define DP_CMD_CLEAR_DISCOVER	0x32
#define DP_CMD_STOP		0x33

/* DP pin assignment bits */
#define DP_PIN_BIT_A		0x01
#define DP_PIN_BIT_B		0x02
#define DP_PIN_BIT_C		0x04
#define DP_PIN_BIT_D		0x08
#define DP_PIN_BIT_E		0x10
#define DP_PIN_BIT_F		0x20

/* DP pin assignments */
#define DP_PIN_UNKNOWN		0
#define DP_PIN_A		1
#define DP_PIN_B		2
#define DP_PIN_C		3
#define DP_PIN_D		4
#define DP_PIN_E		5
#define DP_PIN_F		6

static void s2mm005_dp_mux_set(struct s2mm005 *s, unsigned long mode,
			       u32 status, u32 conf)
{
	struct typec_displayport_data data = {
		.status = status,
		.conf = conf,
	};
	struct typec_mux_state st = {
		.alt = NULL,
		.mode = mode,
		.data = &data,
	};

	typec_mux_set(s->dp_mux, &st);
}

static void s2mm005_dp_exit(struct s2mm005 *s)
{
	if (!s->dp_mux)
		return;
	s2mm005_dp_mux_set(s, TYPEC_STATE_USB, 0, 0);
	s->dp_connected = false;
	s->pin_sent = false;
	s->dp_status = 0;
}

static void s2mm005_alt_mode_start(struct s2mm005 *s)
{
	static const u8 start[] = { DP_CMD_MODE_IFACE, DP_CMD_START };
#if IS_ENABLED(CONFIG_TYPEC_S2MM005_PD_SELECT)
	static const u8 nego[] = { DP_CMD_MODE_IFACE, 0x42 };
#endif

	s2mm005_cmd(s, start, sizeof(start));
#if IS_ENABLED(CONFIG_TYPEC_S2MM005_PD_SELECT)
	s2mm005_cmd(s, nego, sizeof(nego));
#endif
	s->alt_started = true;
}

static void s2mm005_dock_uevent(struct s2mm005 *s, int type)
{
	char state_str[32], ids_str[32];
	char *envp[] = { state_str, ids_str, NULL };

	snprintf(state_str, sizeof(state_str), "SWITCH_STATE=%d", type);
	snprintf(ids_str, sizeof(ids_str), "USBPD_IDS=%04x:%04x", s->vid,
		 s->pid);
	kobject_uevent_env(&s->ccic_dev->kobj, KOBJ_CHANGE, envp);
}

/* classify Samsung accessories for the dock uevent */
static void s2mm005_dp_discover_id(struct s2mm005 *s)
{
	u8 msg[32] = { };
	u16 vid, pid;
	int type = DOCK_NEW;

	if (s2mm005_read(s, DP_REG_DIS_ID, msg, sizeof(msg)) < 0)
		return;
	vid = get_unaligned_le16(&msg[8]);
	pid = get_unaligned_le16(&msg[18]);
	s->vid = vid;
	s->pid = pid;
	s->pin_sent = false;
	dev_dbg(s->dev, "dp discover id vid 0x%04x pid 0x%04x\n", vid, pid);

	if (vid == 0x04e8) {
		switch (pid) {
		case 0xa500:
		case 0xa501:
		case 0xa502:
		case 0xa503:
		case 0xa504:
		case 0xa505:
			type = DOCK_HMT;
			break;
		case 0xa020:
			type = DOCK_DEX;
			break;
		case 0xa029:
			type = DOCK_DEXPAD;
			break;
		case 0xa025:
			type = DOCK_HDMI;
			break;
		case 0xa028:
			type = DOCK_UVDM;
			break;
		default:
			break;
		}
	} else if (vid == 0x04b4 && pid == 0x2122) {
		type = DOCK_MPA;
	}
	s2mm005_dock_uevent(s, type);
}

/*
 * A DP partner makes the phone the USB host, with VBUS only when the phone is
 * the source. The vendor driver's 2 s wait for the host controller is not
 * needed: the role switch only queues work.
 */
static void s2mm005_dp_discover_svids(struct s2mm005 *s)
{
	u8 msg[32] = { };

	if (s2mm005_read(s, DP_REG_DIS_SVID, msg, sizeof(msg)) < 0)
		return;
	s->svid0 = get_unaligned_le16(&msg[8]);
	s->svid1 = get_unaligned_le16(&msg[10]);
	dev_dbg(s->dev, "dp discover svids 0x%04x 0x%04x\n", s->svid0,
		s->svid1);

	if (s->svid0 != 0xff01)
		return;

	if (s->is_client) {
		max77865_muic_detach(s->muic);
		s2mm005_usb_role(s, USB_ROLE_NONE);
		s2mm005_partner_detach(s);
		s->is_client = false;
		msleep(300);
	}
	if (!s->is_host) {
		bool is_src = s->func_state & FS_IS_SOURCE;

		s2mm005_orientation(s);
		max77865_muic_attach_rprd(s->muic);
		s->is_host = true;
		s->pwr_role = is_src ? TYPEC_SOURCE : TYPEC_SINK;
		typec_set_pwr_role(s->port, s->pwr_role);
		s2mm005_partner_attach(s, TYPEC_HOST);
		s2mm005_usb_role(s, USB_ROLE_HOST);
		if (is_src)
			s2mm005_vbus(s, true);
	}
	s->dp_connected = true;
	s2mm005_dp_mux_set(s, TYPEC_STATE_SAFE, 0, 0);
}

/* the pin assignments the partner offers, from its port capability and receptacle */
static void s2mm005_dp_discover_modes(struct s2mm005 *s)
{
	u8 msg[32] = { };
	u32 vdo, vdm;
	u8 nobj, port_cap, recept;
	static const u8 samsung[] = { DP_CMD_MODE_IFACE, DP_CMD_NEXT_STATE,
				     DP_CMD_SAMSUNG_MODE };
	static const u8 clear[] = { DP_CMD_MODE_IFACE, DP_CMD_CLEAR_DISCOVER };

	if (s2mm005_read(s, DP_REG_MODE, msg, sizeof(msg)) < 0)
		return;
	vdo = get_unaligned_le32(&msg[8]);
	vdm = get_unaligned_le32(&msg[4]);
	nobj = msg[0] & 0x7;
	port_cap = vdo & 0x3;
	recept = (vdo >> 6) & 0x1;

	if ((vdm >> 16) == 0xff01 && s->svid0 == 0xff01 && nobj > 1) {
		if ((port_cap == 1 && recept == 1) ||
		    (port_cap == 2 && recept == 0))
			s->pin_caps = (vdo >> 16) & 0xff;
		else if ((port_cap == 2 && recept == 1) ||
			 (port_cap == 1 && recept == 0))
			s->pin_caps = (vdo >> 8) & 0xff;
		else if (port_cap == 3)
			s->pin_caps = recept == 0 ? (vdo >> 8) & 0xff :
						    (vdo >> 16) & 0xff;
		dev_dbg(s->dev, "dp pin caps 0x%02x\n", s->pin_caps);
	}
	if ((vdm >> 16) == 0x04e8)
		s2mm005_cmd(s, samsung, sizeof(samsung));
	s2mm005_cmd(s, clear, sizeof(clear));
}

static void s2mm005_dp_enter_mode(struct s2mm005 *s)
{
	u8 msg[32] = { };
	u32 vdm;

	if (s2mm005_read(s, DP_REG_ENTER_MODE, msg, sizeof(msg)) < 0)
		return;
	vdm = get_unaligned_le32(&msg[4]);
	dev_dbg(s->dev, "dp enter mode svid 0x%04x %s\n", vdm >> 16,
		((vdm >> 6) & 0x3) == 1 ? "ACK" : "NAK");
}

/*
 * Pin preference: D, B, F for a multi-function partner, else C, E, A, D, B, F.
 * Returns the pin bit, 0 for none.
 */
static u32 s2mm005_dp_choose_pin(struct s2mm005 *s, u32 status)
{
	bool multi = status & DP_STATUS_PREFER_MULTI_FUNC;
	u32 bit = 0;
	int pin = DP_PIN_UNKNOWN;

	if (multi) {
		if (s->pin_caps & DP_PIN_BIT_D) {
			bit = DP_PIN_BIT_D;
			pin = DP_PIN_D;
		} else if (s->pin_caps & DP_PIN_BIT_B) {
			bit = DP_PIN_BIT_B;
			pin = DP_PIN_B;
		} else if (s->pin_caps & DP_PIN_BIT_F) {
			bit = DP_PIN_BIT_F;
			pin = DP_PIN_F;
		}
	} else {
		if (s->pin_caps & DP_PIN_BIT_C) {
			bit = DP_PIN_BIT_C;
			pin = DP_PIN_C;
		} else if (s->pin_caps & DP_PIN_BIT_E) {
			bit = DP_PIN_BIT_E;
			pin = DP_PIN_E;
		} else if (s->pin_caps & DP_PIN_BIT_A) {
			bit = DP_PIN_BIT_A;
			pin = DP_PIN_A;
		} else if (s->pin_caps & DP_PIN_BIT_D) {
			bit = DP_PIN_BIT_D;
			pin = DP_PIN_D;
		} else if (s->pin_caps & DP_PIN_BIT_B) {
			bit = DP_PIN_BIT_B;
			pin = DP_PIN_B;
		} else if (s->pin_caps & DP_PIN_BIT_F) {
			bit = DP_PIN_BIT_F;
			pin = DP_PIN_F;
		}
	}
	if (!bit) {
		dev_info(s->dev, "dp wrong pin assignment caps 0x%02x\n",
			 s->pin_caps);
		return 0;
	}
	s->dp_pin = pin;
	dev_dbg(s->dev, "dp pin %c multi %d\n", "ABCDEF"[pin - 1], multi);
	return bit;
}

static u32 s2mm005_dp_pin_bit(int pin)
{
	switch (pin) {
	case DP_PIN_A: return DP_PIN_BIT_A;
	case DP_PIN_B: return DP_PIN_BIT_B;
	case DP_PIN_C: return DP_PIN_BIT_C;
	case DP_PIN_D: return DP_PIN_BIT_D;
	case DP_PIN_E: return DP_PIN_BIT_E;
	case DP_PIN_F: return DP_PIN_BIT_F;
	default: return 0;
	}
}

static void s2mm005_dp_hpd(struct s2mm005 *s, u32 status, u32 pin_bit)
{
	unsigned long mode = TYPEC_DP_STATE_A + (s->dp_pin - 1);
	u32 conf = pin_bit << DP_CONF_PIN_ASSIGNEMENT_SHIFT;

	s->dp_status = status;
	dev_dbg(s->dev, "dp connect vid 0x%04x pid 0x%04x, pin %c, hpd %d\n",
		s->vid, s->pid, "ABCDEF"[s->dp_pin - 1],
		!!(status & DP_STATUS_HPD_STATE));
	s2mm005_dp_mux_set(s, mode, status, conf);
}

static void s2mm005_dp_status_update(struct s2mm005 *s, u16 reg, bool attention)
{
	u8 msg[32] = { };
	u32 status;

	if (s2mm005_read(s, reg, msg, sizeof(msg)) < 0)
		return;
	status = get_unaligned_le32(&msg[8]);
	dev_dbg(s->dev, "dp status 0x%08x\n", status);

	if (s->svid0 != 0xff01)
		return;
	if (!attention && (status & 0x3) == 0x0) {
		dev_dbg(s->dev, "dp status: port disconnected\n");
		return;
	}
	if (!s->pin_sent) {
		u8 req[3] = { DP_CMD_MODE_IFACE, DP_CMD_ALT_MODE_REQ, 0 };
		u32 bit = s2mm005_dp_choose_pin(s, status);

		if (!bit)
			return;
		req[2] = bit;
		s2mm005_cmd(s, req, sizeof(req));
		s->pin_sent = true;
	}
	s2mm005_dp_hpd(s, status, s2mm005_dp_pin_bit(s->dp_pin));
}

static void s2mm005_dp_configure(struct s2mm005 *s)
{
	u8 msg[32] = { };
	u32 vdm;
	static const u8 samsung[] = { DP_CMD_MODE_IFACE, DP_CMD_SVID_SELECT,
				     0x01 };

	if (s2mm005_read(s, DP_REG_CONFIGURE, msg, sizeof(msg)) < 0)
		return;
	vdm = get_unaligned_le32(&msg[4]);
	dev_dbg(s->dev, "dp configure svid 0x%04x\n", vdm >> 16);
	if (s->svid0 != 0xff01 || !s->pin_sent || s->dp_pin <= 0)
		return;
	s2mm005_dp_hpd(s, s->dp_status, s2mm005_dp_pin_bit(s->dp_pin));
	if ((vdm >> 16) == 0xff01 && s->svid1 == 0x04e8)
		s2mm005_cmd(s, samsung, sizeof(samsung));
}

static void s2mm005_dp_vdm(struct s2mm005 *s, u32 vdm_irq)
{
	if (!s->dp_mux || !s->is_host)
		return;

	if (vdm_irq & DP_VDM_DISCOVER_ID)
		s2mm005_dp_discover_id(s);
	if (vdm_irq & DP_VDM_DISCOVER_SVIDS)
		s2mm005_dp_discover_svids(s);
	if (vdm_irq & DP_VDM_DISCOVER_MODES)
		s2mm005_dp_discover_modes(s);
	if (vdm_irq & DP_VDM_ENTER_MODE)
		s2mm005_dp_enter_mode(s);
	if (vdm_irq & DP_VDM_STATUS_UPDATE)
		s2mm005_dp_status_update(s, DP_REG_STATUS_UPDATE, false);
	if (vdm_irq & DP_VDM_CONFIGURE)
		s2mm005_dp_configure(s);
	if (vdm_irq & DP_VDM_ATTENTION)
		s2mm005_dp_status_update(s, DP_REG_ATTENTION, true);
}

static u32 s2mm005_get_int_status(struct s2mm005 *s, u32 *vdm_irq)
{
	u8 buf[48] = { };
	u32 msg_irq, ap_req;

	s2mm005_read(s, REG_INT_STATUS, buf, sizeof(buf));
	s2mm005_int_clear(s);

	msg_irq = get_unaligned_le32(&buf[4 * 4]);
	*vdm_irq = get_unaligned_le32(&buf[5 * 4]);
	ap_req = get_unaligned_le32(&buf[7 * 4]);
	dev_dbg(s->dev, "MSG IRQ 0x%08x VDM IRQ 0x%08x SSM IRQ 0x%08x AP REQ GET 0x%08x\n",
		msg_irq, *vdm_irq, get_unaligned_le32(&buf[6 * 4]), ap_req);

	if (msg_irq & MSG_DR_SWAP)
		s2mm005_dr_swap(s);

	/* the firmware lost the alternate mode request */
	if (!FIELD_GET(AP_REQ_ALT_MODE_BY_I2C, ap_req) && !s->alt_started)
		s2mm005_alt_mode_start(s);

	return msg_irq;
}

/* only fixed supply PDOs are kept */
static int s2mm005_src_caps(struct s2mm005 *s, const u8 *msg, bool *do_power_nego, char *str,
			    size_t len)
{
	unsigned int num, i, pos = 0;
	int available = 0;

	num = min_t(unsigned int, FIELD_GET(MSG_HDR_NUM_OF_OBJ, get_unaligned_le32(msg)),
		    S2MM005_MAX_PDO_NUM);
	str[0] = '\0';
	for (i = 0; i < num; i++) {
		u32 pdo = get_unaligned_le32(&msg[4 * (i + 1)]);
		int mv, ma;

		if (FIELD_GET(PDO_TYPE, pdo) != 0) {
			pos += scnprintf(str + pos, len - pos, "%s%s", pos ? " " : "",
					 FIELD_GET(PDO_TYPE, pdo) == 2 ? "var" : "bat");
			continue;
		}
		mv = FIELD_GET(PDO_FIXED_VOLTAGE, pdo) * UNIT_FOR_VOLTAGE;
		ma = FIELD_GET(PDO_FIXED_MAX_CURRENT, pdo) * UNIT_FOR_CURRENT;
		if (mv <= AVAILABLE_VOLTAGE)
			available = i + 1;
		if (!*do_power_nego && (s->pd.power_list[i + 1].max_voltage != mv ||
					s->pd.power_list[i + 1].max_current != ma))
			*do_power_nego = true;
		s->pd.power_list[i + 1].max_voltage = mv;
		s->pd.power_list[i + 1].max_current = ma;
		pos += scnprintf(str + pos, len - pos, "%s%d/%d", pos ? " " : "", mv, ma);
	}
	s->pd.available_pdo_num = available;
	return available;
}

static void s2mm005_pd_queue(struct s2mm005 *s, struct s2mm005_pd_note *note,
			     unsigned long event)
{
	if (note->n < ARRAY_SIZE(note->ev)) {
		note->ev[note->n].event = event;
		note->ev[note->n].status = s->pd;
		note->n++;
	}
}

/* the events are queued in @note and posted once the lock is released */
static void s2mm005_pd(struct s2mm005 *s, bool attach_done, u32 msg_irq,
		       struct s2mm005_pd_note *note)
{
	unsigned int rp = FIELD_GET(FS_RP_CURRENTLVL, s->func_state);
	bool is_src = s->func_state & FS_IS_SOURCE;
	bool pdic_attach = false;
	unsigned long event = S2MM005_PD_EVENT_DETACH;
	int i;

	if (msg_irq & MSG_PR_SWAP) {
		s->is_pr_swap++;
		dev_dbg(s->dev, "PR_Swap requested to %s\n", is_src ? "SOURCE" : "SINK");
		if (is_src && s->pwr_role == TYPEC_SINK) {
			s2mm005_pd_queue(s, note, S2MM005_PD_EVENT_DETACH);
			s->pd_attached = false;
		}
		s2mm005_vbus(s, is_src);
		s->pwr_role = is_src ? TYPEC_SOURCE : TYPEC_SINK;
		typec_set_pwr_role(s->port, s->pwr_role);
		typec_set_pwr_opmode(s->port, TYPEC_PWR_MODE_USB);
	}

	if (msg_irq & MSG_SRC_CAPABILITY) {
		u8 msg[32] = { };
		char caps[8 * 16];
		bool do_power_nego = false;
		int available;
		unsigned int pos;

		event = S2MM005_PD_EVENT_PD_SINK;
		s2mm005_read(s, REG_RX_SRC_CAPA_MSG, msg, sizeof(msg));
		available = s2mm005_src_caps(s, msg, &do_power_nego, caps, sizeof(caps));

		memset(msg, 0, sizeof(msg));
		s2mm005_read(s, REG_TX_REQUEST_MSG, msg, sizeof(msg));
		pos = FIELD_GET(REQ_OBJECT_POSITION, get_unaligned_le32(&msg[4]));
		s->pd.current_pdo_num = pos;

		if (available > 0) {
			if (pos != s->pd.selected_pdo_num) {
				if (s->pd.selected_pdo_num == 0)
					s->pd.selected_pdo_num = s->pd.current_pdo_num;
			} else if (do_power_nego) {
				s->pd.selected_pdo_num = 0;
				event = S2MM005_PD_EVENT_PD_SINK_CAP;
			}
			pdic_attach = true;
		}

		/* the firmware's own request, which may be above 5 V */
		dev_dbg(s->dev, "pd src caps %u: %s, fw requested pos %u (%u mV %u mA)\n",
			available, caps, pos, s->pd.power_list[pos].max_voltage,
			s->pd.power_list[pos].max_current);
	}

	if (msg_irq & MSG_GET_SINK_CAP)
		dev_dbg(s->dev, "SRC requested SINK Cap\n");

	if (attach_done) {
		if (pdic_attach) {
			/* PD charger */
		} else if (!is_src && (s->pd_state == PD_SNK_WAIT_FOR_CAPABILITIES ||
				       s->pd_state == PD_ERROR_RECOVERY) &&
			   rp != s->pd.rp_currentlvl && rp >= S2MM005_RP_CURRENT_LEVEL_DEFAULT) {
			s->pd.rp_currentlvl = rp;
			event = S2MM005_PD_EVENT_CCIC_ATTACH;
		} else {
			return;
		}
		s2mm005_pd_queue(s, note, event);
		s->pd_attached = true;
	} else {
		for (i = 0; i < S2MM005_MAX_PDO_NUM + 1; i++) {
			s->pd.power_list[i].max_current = 0;
			s->pd.power_list[i].max_voltage = 0;
		}
		s->pd.rp_currentlvl = S2MM005_RP_CURRENT_LEVEL_NONE;
		s->pd.available_pdo_num = 0;
		s->pd.selected_pdo_num = 0;
		s->pd.current_pdo_num = 0;
		if (s->pd_attached) {
			s2mm005_pd_queue(s, note, S2MM005_PD_EVENT_DETACH);
			s->pd_attached = false;
		}
	}
}

static void s2mm005_cc_rid(struct s2mm005 *s)
{
	u8 rid;

	if (s2mm005_read_byte16(s, REG_RID, &rid) < 0)
		return;
	dev_dbg(s->dev, "prev_rid:%x , RID:%x\n", s->prev_rid, rid);
	s->cur_rid = rid;

	if (!rid)
		return;
	if (s->prev_rid != rid) {
		max77865_muic_rid(s->muic, rid);
		if (rid == RID_523K || rid == RID_619K) {
			s->is_client = false;
			s2mm005_usb_role(s, USB_ROLE_NONE);
			s2mm005_partner_detach(s);
		}
	}
	s->prev_rid = rid;
}

static void s2mm005_water_det(struct s2mm005 *s)
{
	s2mm005_int_clear(s);
	if (s->water_det)
		s2mm005_manual_lpm(s, 0x09);
}

/* one interrupt pass, with the lock held */
static void s2mm005_pass(struct s2mm005 *s, struct s2mm005_pd_note *note)
{
	bool attach_done;
	u32 msg_irq, vdm_irq;

	pm_wakeup_event(s->dev, 1000);

	if (s2mm005_fw_ver_check(s))
		return;

	s2mm005_cc_attach(s, &attach_done);
	if (s->i2c_err < 0)
		return;

	if (s->water_det || !s->run_dry || !s->booting_run_dry) {
		s2mm005_water_det(s);
		return;
	}

	msg_irq = s2mm005_get_int_status(s, &vdm_irq);
	s2mm005_dp_vdm(s, vdm_irq);
	s2mm005_pd(s, attach_done, msg_irq, note);
	s2mm005_cc_rid(s);
}

/*
 * Static, so that the charger can register before this device binds: the port
 * controller depends on the charger's OTG regulator for its VBUS.
 */
static BLOCKING_NOTIFIER_HEAD(s2mm005_pd_nh);

static void s2mm005_post(struct s2mm005_pd_note *note)
{
	unsigned int i;

	for (i = 0; i < note->n; i++)
		blocking_notifier_call_chain(&s2mm005_pd_nh, note->ev[i].event,
					     &note->ev[i].status);
}

static irqreturn_t s2mm005_irq_thread(int irq, void *data)
{
	struct s2mm005 *s = data;
	struct s2mm005_pd_note note = { };

	mutex_lock(&s->lock);
	if (s->check_at_booting) {
		s->check_at_booting = false;
		/* not _sync: the work takes the lock and re-checks the flag */
		cancel_delayed_work(&s->boot_check);
	}
	s2mm005_pass(s, &note);
	mutex_unlock(&s->lock);

	s2mm005_post(&note);
	return IRQ_HANDLED;
}

/* one pass at boot if no interrupt came */
static void s2mm005_boot_check(struct work_struct *work)
{
	struct s2mm005 *s = container_of(to_delayed_work(work), struct s2mm005, boot_check);
	struct s2mm005_pd_note note = { };

	mutex_lock(&s->lock);
	dev_dbg(s->dev, "check state=%d\n", s->check_at_booting);
	if (s->check_at_booting) {
		s->check_at_booting = false;
		s2mm005_pass(s, &note);
	}
	mutex_unlock(&s->lock);

	s2mm005_post(&note);
}

static void s2mm005_swap_check(struct work_struct *work)
{
	struct s2mm005 *s = container_of(to_delayed_work(work), struct s2mm005, swap_check);
	u8 pd_state;

	mutex_lock(&s->lock);
	s->try_state = 0;
	pd_state = s->pd_state;
	mutex_unlock(&s->lock);

	if (pd_state == PD_INITIAL_DETACH) {
		dev_err(s->dev, "port type change failed, back to DRP\n");
		disable_irq(s->irq);
		s2mm005_rprd_mode_change(s, TYPE_C_ATTACH_DRP);
		enable_irq(s->irq);
	}
}

/*
 * Force the firmware into source or sink and wait up to 1.5 s for the new
 * role, else go back to DRP. The interrupt thread completes the wait, so the
 * lock is not held across it.
 */
static int s2mm005_port_type_set(struct typec_port *port, enum typec_port_type type)
{
	struct s2mm005 *s = typec_get_drvdata(port);
	int mode;

	switch (type) {
	case TYPEC_PORT_SRC:
		mode = TYPE_C_ATTACH_DFP;
		mutex_lock(&s->lock);
		break;
	case TYPEC_PORT_SNK:
		mode = TYPE_C_ATTACH_UFP;
		mutex_lock(&s->lock);
		s2mm005_vbus(s, false);
		max77865_muic_detach(s->muic);
		break;
	case TYPEC_PORT_DRP:
		return 0;
	default:
		return -EINVAL;
	}
	dev_dbg(s->dev, "port type change to %s\n", type == TYPEC_PORT_SRC ? "source" : "sink");
	s->try_state = mode;
	reinit_completion(&s->role_done);
	mutex_unlock(&s->lock);

	s2mm005_rprd_mode_change(s, mode);

	if (!wait_for_completion_timeout(&s->role_done,
					 msecs_to_jiffies(DUAL_ROLE_SET_MODE_WAIT_MS))) {
		dev_err(s->dev, "port type change failed, back to DRP\n");
		disable_irq(s->irq);
		mutex_lock(&s->lock);
		s->try_state = 0;
		mutex_unlock(&s->lock);
		s2mm005_rprd_mode_change(s, TYPE_C_ATTACH_DRP);
		enable_irq(s->irq);
		return -EIO;
	}

	schedule_delayed_work(&s->swap_check, msecs_to_jiffies(DUAL_ROLE_SET_MODE_WAIT_MS));
	return 0;
}

static const struct typec_operations s2mm005_typec_ops = {
	.port_type_set = s2mm005_port_type_set,
};

static ssize_t cur_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct s2mm005 *s = dev_get_drvdata(dev);
	struct s2mm005_version sw = { };
	u8 v[4];

	mutex_lock(&s->lock);
	s2mm005_get_swversion(s, &sw);
	s2mm005_store_swversion(s, &sw);
	memcpy(v, s->firm_ver, sizeof(v));
	mutex_unlock(&s->lock);

	return sysfs_emit(buf, "%02X %02X %02X %02X\n", v[0], v[1], v[2], v[3]);
}
static DEVICE_ATTR_RO(cur_version);

static ssize_t src_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%02X %02X %02X %02X\n", BUNDLED_FW_MAIN2, BUNDLED_FW_MAIN1,
			  BUNDLED_FW_MAIN0, BUNDLED_FW_BOOT);
}
static DEVICE_ATTR_RO(src_version);

static ssize_t state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct s2mm005 *s = dev_get_drvdata(dev);
	int pd_state;

	mutex_lock(&s->lock);
	pd_state = s->pd_state;
	mutex_unlock(&s->lock);

	return sysfs_emit(buf, "%d\n", pd_state);
}
static DEVICE_ATTR_RO(state);

static ssize_t water_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct s2mm005 *s = dev_get_drvdata(dev);
	int water;

	mutex_lock(&s->lock);
	water = s->water_det | !s->run_dry;
	mutex_unlock(&s->lock);

	return sysfs_emit(buf, "%d\n", water);
}
static DEVICE_ATTR_RO(water);

static struct attribute *s2mm005_ccic_attrs[] = {
	&dev_attr_cur_version.attr,
	&dev_attr_src_version.attr,
	&dev_attr_state.attr,
	&dev_attr_water.attr,
	NULL
};
ATTRIBUTE_GROUPS(s2mm005_ccic);

int s2mm005_register_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&s2mm005_pd_nh, nb);
}
EXPORT_SYMBOL_GPL(s2mm005_register_notifier);

int s2mm005_unregister_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&s2mm005_pd_nh, nb);
}
EXPORT_SYMBOL_GPL(s2mm005_unregister_notifier);

#if IS_ENABLED(CONFIG_TYPEC_S2MM005_PD_SELECT)
/* the port whose PDO the charger selects */
static struct s2mm005 *s2mm005_pd_port;

void s2mm005_select_pdo(int num)
{
	struct s2mm005 *s = READ_ONCE(s2mm005_pd_port);
	u8 cmd[3];

	if (!s)
		return;

	mutex_lock(&s->lock);
	if (s->pd.selected_pdo_num == num)
		goto out;
	else if (num > s->pd.available_pdo_num)
		s->pd.selected_pdo_num = s->pd.available_pdo_num;
	else if (num < 1)
		s->pd.selected_pdo_num = 1;
	else
		s->pd.selected_pdo_num = num;
	dev_dbg(s->dev, "PDO(%d) is selected to change\n", s->pd.selected_pdo_num);

	cmd[0] = 0x03;
	cmd[1] = 0x03;
	cmd[2] = s->pd.selected_pdo_num;
	s2mm005_cmd(s, cmd, 3);
	cmd[0] = 0x03;
	cmd[1] = 0x02;
	cmd[2] = PD_SNK_WAIT_FOR_CAPABILITIES;
	s2mm005_cmd(s, cmd, 3);
out:
	mutex_unlock(&s->lock);
}

static void s2mm005_pd_port_clear(void *data)
{
	WRITE_ONCE(s2mm005_pd_port, NULL);
}
#else
void s2mm005_select_pdo(int num)
{
	pr_info_once("s2mm005: select_pdo(%d) ignored (5 V only)\n", num);
}
#endif
EXPORT_SYMBOL_GPL(s2mm005_select_pdo);

static void s2mm005_fwnode_put(void *data)
{
	fwnode_handle_put(data);
}

static void s2mm005_muic_put(void *data)
{
	max77865_muic_put(data);
}

static void s2mm005_role_sw_put(void *data)
{
	usb_role_switch_put(data);
}

static void s2mm005_mux_put(void *data)
{
	typec_mux_put(data);
}

static void s2mm005_pinctrl_put(void *data)
{
	pinctrl_put(data);
}

static void s2mm005_vbus_off(void *data)
{
	struct s2mm005 *s = data;

	if (s->vbus_on) {
		regulator_disable(s->vbus);
		s->vbus_on = false;
	}
}

static void s2mm005_typec_release(void *data)
{
	struct s2mm005 *s = data;

	s2mm005_partner_detach(s);
	typec_unregister_port(s->port);
}

static void s2mm005_ccic_destroy(void *data)
{
	sec_device_destroy(data);
}

static void s2mm005_works_disable(void *data)
{
	struct s2mm005 *s = data;

	disable_delayed_work_sync(&s->boot_check);
	disable_delayed_work_sync(&s->swap_check);
}

static int s2mm005_get_resources(struct s2mm005 *s)
{
	struct device *dev = s->dev;
	struct device_node *np;
	int ret;

	s->orient = devm_gpiod_get_optional(dev, "orientation", GPIOD_IN);
	if (IS_ERR(s->orient))
		return dev_err_probe(dev, PTR_ERR(s->orient), "orientation gpio\n");

	np = of_parse_phandle(dev->of_node, "samsung,muic", 0);
	if (!np)
		return dev_err_probe(dev, -ENODEV, "no samsung,muic\n");
	s->muic = max77865_muic_get(np);
	of_node_put(np);
	if (IS_ERR(s->muic))
		return dev_err_probe(dev, PTR_ERR(s->muic), "MUIC\n");
	ret = devm_add_action_or_reset(dev, s2mm005_muic_put, s->muic);
	if (ret)
		return ret;

	s->connector = device_get_named_child_node(dev, "connector");
	if (!s->connector)
		return dev_err_probe(dev, -ENODEV, "no connector node\n");
	ret = devm_add_action_or_reset(dev, s2mm005_fwnode_put, s->connector);
	if (ret)
		return ret;

	s->role_sw = fwnode_usb_role_switch_get(s->connector);
	if (IS_ERR(s->role_sw))
		return dev_err_probe(dev, PTR_ERR(s->role_sw), "USB role switch\n");
	if (s->role_sw) {
		ret = devm_add_action_or_reset(dev, s2mm005_role_sw_put, s->role_sw);
		if (ret)
			return ret;
	}

	s->vbus = devm_of_regulator_get_optional(dev, to_of_node(s->connector), "vbus");
	if (IS_ERR(s->vbus)) {
		if (PTR_ERR(s->vbus) != -ENODEV)
			return dev_err_probe(dev, PTR_ERR(s->vbus), "vbus-supply\n");
		s->vbus = NULL;
	} else {
		ret = devm_add_action_or_reset(dev, s2mm005_vbus_off, s);
		if (ret)
			return ret;
	}

	s->dp_mux = fwnode_typec_mux_get(s->connector);
	if (IS_ERR(s->dp_mux))
		return dev_err_probe(dev, PTR_ERR(s->dp_mux), "DP mux\n");
	if (s->dp_mux) {
		ret = devm_add_action_or_reset(dev, s2mm005_mux_put,
					       s->dp_mux);
		if (ret)
			return ret;
	}

	s->i2c_pinctrl = pinctrl_get(s->client->adapter->dev.parent);
	if (IS_ERR(s->i2c_pinctrl))
		return dev_err_probe(dev, PTR_ERR(s->i2c_pinctrl), "I2C controller pinctrl\n");
	ret = devm_add_action_or_reset(dev, s2mm005_pinctrl_put, s->i2c_pinctrl);
	if (ret)
		return ret;
	s->i2c_reset = pinctrl_lookup_state(s->i2c_pinctrl, "hard_reset");
	if (IS_ERR(s->i2c_reset))
		return dev_err_probe(dev, PTR_ERR(s->i2c_reset), "no hard_reset pin state\n");
	s->i2c_default = pinctrl_lookup_state(s->i2c_pinctrl, PINCTRL_STATE_DEFAULT);
	if (IS_ERR(s->i2c_default))
		return dev_err_probe(dev, PTR_ERR(s->i2c_default), "no default pin state\n");

	return 0;
}

static int s2mm005_chip_init(struct s2mm005 *s)
{
	struct s2mm005_version hw = { }, sw = { };
	static const u8 lpm_enable[] = { 0x0f, 0x06 };
	u8 msg[32] = { };
	u8 w[8], r[4] = { }, ftrim;
	int ret;

	ret = s2mm005_read(s, REG_TX_SINK_CAPA_MSG, msg, sizeof(msg));
	if (ret < 0) {
		s2mm005_hard_reset(s);
		msleep(1000);
		ret = s2mm005_read(s, REG_TX_SINK_CAPA_MSG, msg, sizeof(msg));
		if (ret < 0) {
			dev_err(s->dev, "no answer at 0x220 after hard reset (%d)\n", ret);
			return -ENODEV;
		}
	}

	s2mm005_get_hwversion(s, &hw);

	/* early chips need their oscillator trim corrected */
	if (hw.boot <= 2) {
		w[0] = 0x02; w[1] = 0x40; w[2] = 0x04; w[3] = 0x11;
		s2mm005_cmd(s, w, 4);
		s2mm005_read(s, REG_FTRIM_RSP, r, 4);
		dev_dbg(s->dev, "ftrim:%02X %02X %02X %02X\n", r[0], r[1], r[2], r[3]);
		ftrim = ((r[1] & 0xf8) >> 3) - 2;
		r[1] = (ftrim << 3) + (r[1] & 0x7);
		dev_dbg(s->dev, "ftrim:%02X %02X %02X %02X\n", r[0], r[1], r[2], r[3]);
		w[0] = 0x02; w[1] = 0x04; w[2] = 0x04; w[3] = 0x11;
		w[4] = r[0]; w[5] = r[1]; w[6] = r[2]; w[7] = r[3];
		s2mm005_cmd(s, w, 8);
		w[0] = 0x02; w[1] = 0x40; w[2] = 0x04; w[3] = 0x11;
		s2mm005_cmd(s, w, 4);
		s2mm005_read(s, REG_FTRIM_RSP, r, 4);
		dev_dbg(s->dev, "ftrim:%02X %02X %02X %02X\n", r[0], r[1], r[2], r[3]);
	}

	s2mm005_get_swversion(s, &sw);
	dev_info(s->dev, "hw %02x.%02x.%02x boot %u, chip fw %02x.%02x.%02x boot %u, bundled fw %02x.%02x.%02x\n",
		 hw.main[2], hw.main[1], hw.main[0], hw.boot,
		 sw.main[2], sw.main[1], sw.main[0], sw.boot,
		 BUNDLED_FW_MAIN2, BUNDLED_FW_MAIN1, BUNDLED_FW_MAIN0);

	/* the condition under which the vendor driver would flash the bundled image */
	if (sw.boot == 0x7 &&
	    (sw.main[0] < BUNDLED_FW_MAIN0 ||
	     (sw.main[0] == BUNDLED_FW_MAIN0 && sw.main[1] < BUNDLED_FW_MAIN1) ||
	     sw.main[2] != BUNDLED_FW_MAIN2 ||
	     (((sw.main[2] == 0xff && sw.main[1] == 0xa5) || sw.main[2] == 0x00) &&
	      BUNDLED_FW_MAIN2 != 0x0)))
		dev_info(s->dev, "chip fw %02x.%02x older than bundled %02x.%02x, not updated\n",
			 sw.main[1], sw.main[0], BUNDLED_FW_MAIN1, BUNDLED_FW_MAIN0);

	s2mm005_store_swversion(s, &sw);

	s2mm005_cmd(s, lpm_enable, sizeof(lpm_enable));
	return 0;
}

static int s2mm005_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct s2mm005 *s;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return dev_err_probe(dev, -EIO, "i2c functionality check error\n");

	s = devm_kzalloc(dev, sizeof(*s), GFP_KERNEL);
	if (!s)
		return -ENOMEM;
	s->dev = dev;
	s->client = client;
	s->irq = client->irq;
	s->run_dry = true;
	s->booting_run_dry = true;
	s->prev_rid = RID_OPEN;
	s->pwr_role = TYPEC_SINK;

	ret = devm_mutex_init(dev, &s->io_lock);
	if (ret)
		return ret;
	ret = devm_mutex_init(dev, &s->lock);
	if (ret)
		return ret;
	init_completion(&s->role_done);
	INIT_DELAYED_WORK(&s->swap_check, s2mm005_swap_check);
	INIT_DELAYED_WORK(&s->boot_check, s2mm005_boot_check);

	ret = s2mm005_get_resources(s);
	if (ret)
		return ret;

	ret = s2mm005_chip_init(s);
	if (ret)
		return ret;

	s->cap.type = TYPEC_PORT_DRP;
	s->cap.data = TYPEC_PORT_DRD;
	s->cap.revision = USB_TYPEC_REV_1_2;
	s->cap.pd_revision = 0x300;
	s->cap.prefer_role = TYPEC_NO_PREFERRED_ROLE;
	s->cap.orientation_aware = 1;
	s->cap.fwnode = s->connector;
	s->cap.driver_data = s;
	s->cap.ops = &s2mm005_typec_ops;
	s->port = typec_register_port(dev, &s->cap);
	if (IS_ERR(s->port))
		return dev_err_probe(dev, PTR_ERR(s->port), "cannot register the Type-C port\n");
	ret = devm_add_action_or_reset(dev, s2mm005_typec_release, s);
	if (ret)
		return ret;

	s->ccic_dev = sec_device_create_with_groups(s, s2mm005_ccic_groups, "ccic");
	if (IS_ERR(s->ccic_dev)) {
		dev_err(dev, "cannot create sec/ccic (%ld)\n", PTR_ERR(s->ccic_dev));
		s->ccic_dev = NULL;
	} else {
		ret = devm_add_action_or_reset(dev, s2mm005_ccic_destroy, s->ccic_dev);
		if (ret)
			return ret;
	}

	/*
	 * Only with alternate mode started does the firmware interrupt early
	 * enough on a source attach for VBUS to come up before the sink gives up.
	 */
	s2mm005_alt_mode_start(s);

	/* after everything the works use, before the IRQ */
	ret = devm_add_action_or_reset(dev, s2mm005_works_disable, s);
	if (ret)
		return ret;

	s2mm005_int_clear(s);

	s->check_at_booting = true;
	schedule_delayed_work(&s->boot_check, msecs_to_jiffies(200));

	device_init_wakeup(dev, true);
	ret = devm_request_threaded_irq(dev, s->irq, NULL, s2mm005_irq_thread, IRQF_ONESHOT,
					"s2mm005-usbpd", s);
	if (ret)
		return dev_err_probe(dev, ret, "cannot request IRQ %d\n", s->irq);

	s2mm005_int_clear(s);

#if IS_ENABLED(CONFIG_TYPEC_S2MM005_PD_SELECT)
	WRITE_ONCE(s2mm005_pd_port, s);
	ret = devm_add_action_or_reset(dev, s2mm005_pd_port_clear, NULL);
	if (ret)
		return ret;
#endif
	i2c_set_clientdata(client, s);
	return 0;
}

static void s2mm005_shutdown(struct i2c_client *client)
{
	struct s2mm005 *s = i2c_get_clientdata(client);

	if (!s)
		return;

	disable_irq(s->irq);
	mutex_lock(&s->lock);
	if (s->cur_rid != RID_523K && s->cur_rid != RID_619K) {
		dev_dbg(s->dev, "shutdown: pd_state=%d, water=%d, dry=%d\n", s->pd_state,
			s->water_det, s->run_dry);
		if (s->water_det) {
			s2mm005_hard_reset(s);
		} else {
			if (s->pd_state) {
				s2mm005_manual_lpm(s, 0x0b);
				mdelay(110);
			}
			s2mm005_reset(s);
		}
	}
	mutex_unlock(&s->lock);
}

static int s2mm005_suspend(struct device *dev)
{
	struct s2mm005 *s = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		enable_irq_wake(s->irq);
	disable_irq(s->irq);
	return 0;
}

static int s2mm005_resume(struct device *dev)
{
	struct s2mm005 *s = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		disable_irq_wake(s->irq);
	enable_irq(s->irq);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(s2mm005_pm_ops, s2mm005_suspend, s2mm005_resume);

static const struct i2c_device_id s2mm005_id[] = {
	{ "usbpd-s2mm005" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, s2mm005_id);

static const struct of_device_id s2mm005_of_match[] = {
	{ .compatible = "samsung,s2mm005" },
	{ }
};
MODULE_DEVICE_TABLE(of, s2mm005_of_match);

static struct i2c_driver s2mm005_driver = {
	.driver = {
		.name = "s2mm005",
		.of_match_table = s2mm005_of_match,
		.pm = pm_sleep_ptr(&s2mm005_pm_ops),
		.suppress_bind_attrs = true,
	},
	.probe = s2mm005_probe,
	.shutdown = s2mm005_shutdown,
	.id_table = s2mm005_id,
};
module_i2c_driver(s2mm005_driver);

MODULE_DESCRIPTION("Samsung S2MM005 USB Type-C / PD port controller");
MODULE_LICENSE("GPL");
