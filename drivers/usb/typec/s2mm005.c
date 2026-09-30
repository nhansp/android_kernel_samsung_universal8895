// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung S2MM005 USB Type-C / USB-PD port controller (Samsung Galaxy Note 8, greatlte).
 *
 * The chip's firmware runs Type-C and USB-PD; the AP reads its state words over I2C
 * (16-bit big-endian register addresses) and drives it through the command register
 * 0x10. Written (not transplanted) on the stock register protocol of
 * S:drivers/ccic/s2mm005.c, s2mm005_cc.c and s2mm005_pd.c (spec L6G 5.1). Stock fanned
 * the events out through a CCIC/manager notifier chain; here they go straight to the
 * Type-C class, the USB role switch, the VBUS regulator and the MAX77865 MUIC.
 *
 * The kernel never asks for more than 5 V: no PDO selection, sink-capability write or
 * power negotiation command is sent unless CONFIG_TYPEC_S2MM005_PD_SELECT is set, and
 * VBUS is never switched on unless CONFIG_TYPEC_S2MM005_OTG_VBUS is set (spec L6G D-13,
 * 5.1.7; both default n). DisplayPort alternate mode runs only with a DP mode-switch
 * in the DT (spec L6G 5.1.8); without one the firmware is told to stop it.
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


/* registers, S:include/linux/ccic/s2mm005.h:50-53 and the EXT_SRAM map :450-475 */
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
#define REG_FTRIM_RSP		0x0014	/* I2C_SLV_RSP, S:drivers/ccic/s2mm005.c:876 */

/* FUNC_STATE_Type, s2mm005.h:179-204 */
#define FS_PD_STATE		GENMASK(7, 0)
#define FS_IS_SOURCE		BIT(25)
#define FS_RP_CURRENTLVL	GENMASK(28, 27)
#define FS_VBUS_CC_SHORT	BIT(29)
#define FS_VBUS_SBU_SHORT	BIT(30)
#define FS_RESET		BIT(31)

/* LP_STATE_Type, s2mm005.h:206-223 */
#define LP_AUTO_LP_ENABLE	BIT(0)
#define LP_WATER_DET		BIT(3)
#define LP_RUN_DRY		BIT(5)
#define LP_BOOTING_RUN_DRY	BIT(7)
#define LP_SLEEP_CABLE_DETECT	BIT(8)
#define LP_PDSTATE29_SBU_DONE	BIT(9)

/* MSG_IRQ_STATUS_Type, s2mm005.h:319-358 */
#define MSG_GET_SINK_CAP	BIT(8)
#define MSG_DR_SWAP		BIT(9)
#define MSG_PR_SWAP		BIT(10)
#define MSG_SRC_CAPABILITY	BIT(17)

/* AP_REQ_GET_STATUS_Type, s2mm005.h:380-399 */
#define AP_REQ_ALT_MODE_BY_I2C	GENMASK(29, 28)

/* PD message fields, s2mm005.h:524-567, :698-709 */
#define MSG_HDR_NUM_OF_OBJ	GENMASK(14, 12)
#define PDO_TYPE		GENMASK(31, 30)
#define PDO_FIXED_VOLTAGE	GENMASK(19, 10)
#define PDO_FIXED_MAX_CURRENT	GENMASK(9, 0)
#define REQ_OBJECT_POSITION	GENMASK(30, 28)
#define AVAILABLE_VOLTAGE	12000	/* s2mm005.h:46-48 */
#define UNIT_FOR_VOLTAGE	50
#define UNIT_FOR_CURRENT	10

/* function_status_t, s2mm005.h:735-776 */
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

/* CCIC_OTP_MODE, s2mm005.h:778-784 */
#define TYPE_C_ATTACH_DFP	1
#define TYPE_C_ATTACH_UFP	2
#define TYPE_C_ATTACH_DRP	3

/* CCIC_RP_CurrentLvl, s2mm005.h:800-807 */
#define RP_SBU_CHECK		0
#define RP_ABNORMAL		4

/* RID codes, S:include/linux/ccic/ccic_notifier.h:130-137 */
#define RID_523K		5
#define RID_619K		6
#define RID_OPEN		7

#define DUAL_ROLE_SET_MODE_WAIT_MS	1500	/* s2mm005.h:786 */
#define CCIC_I2C_VALUE_INVALID		274	/* s2mm005.h:56 */
#define FW_CHECK_RETRY			5	/* S:drivers/ccic/s2mm005_fw.c:15 */
#define PRODUCT_NUM_GREAT		0x0a	/* S:include/linux/ccic/s2mm005_fw.h:40 */

/*
 * The bundled firmware version stock compared against: the first four bytes of
 * BOOT_FLASH_FW_0x0A_BOOT7[] (S:include/linux/ccic/BOOT_FLASH_FW_0x0A_BOOT7.h:3,
 * struct s2mm005_fw S:include/linux/ccic/s2mm005_fw.h:49-51, selected for boot 7 and
 * product 0x0A by S:drivers/ccic/s2mm005_fw.c:447-451). Logged only; no flashing (D-14).
 */
#define BUNDLED_FW_BOOT		0x07
#define BUNDLED_FW_MAIN0	0x46
#define BUNDLED_FW_MAIN1	0x00
#define BUNDLED_FW_MAIN2	0x0a

/* struct s2mm005_version, s2mm005_fw.h:43-47 */
struct s2mm005_version {
	u8 main[3];
	u8 boot;
	u8 ver2[4];
};

struct s2mm005 {
	struct device *dev;
	struct i2c_client *client;
	int irq;

	struct mutex io_lock;		/* stock i2c_mutex (S:drivers/ccic/s2mm005.c:79,104); xfer_buf */
	u8 xfer_buf[2 + 256];		/* stock stack buffer buf[258] (:152) */

	/*
	 * All state below; taken by the IRQ thread, the boot check and role-swap works,
	 * port_type_set and the sysfs shows. Lock order: lock -> io_lock; lock -> MUIC lock.
	 */
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

	/* USB-PD sink status (stock pd_noti, S:drivers/ccic/s2mm005_pd.c:30) */
	struct s2mm005_pd_status pd;
	bool pd_attached;
	bool pd_raw_logged;
	struct blocking_notifier_head pd_nh;

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
	/* DisplayPort alt mode (L6G 5.1.8, G4): NULL without a DP mode-switch
	 * in the DT (G1-G3); the S2MM005 then tells the firmware to stop */
	struct typec_mux *dp_mux;
	bool alt_started;
	u16 svid0;
	u16 svid1;
	u16 vid;
	u16 pid;
	u32 pin_caps;
	int dp_pin;		/* CCIC_NOTIFY pin A..F, 0 = unknown */
	bool pin_sent;
	bool dp_connected;
	u32 dp_status;		/* raw status VDO */
#if IS_ENABLED(CONFIG_ANDROID_SWITCH_COMPAT)
	struct switch_compat *dock;		/* /sys/class/switch/ccic_dock (L6G 5.7.5) */
#endif

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

/* ---- I2C access (S:drivers/ccic/s2mm005.c:70-265) ---- */

/* stock s2mm005_read_byte(): the transfer is repeated while it fails, at most 5 more times */
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

/* stock s2mm005_read_byte_16(): one byte, no retry (:192-226) */
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

/* stock s2mm005_write_byte(): at most 256 data bytes, retried like the read (:150-190) */
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

/* stock s2mm005_int_clear() = s2mm005_write_byte_16(0x10, 0x1): one write, no retry (:228-265) */
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

/*
 * stock s2mm005_read_byte_flash(): udelay(20), command 0xAA, udelay(20), then one read
 * without retry (:109-148)
 */
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

/* stock "for Wake up": 5 dummy reads of register 0x8, then udelay(10) (:276-282) */
static void s2mm005_wake(struct s2mm005 *s)
{
	u8 dummy;
	int i;

	for (i = 0; i < 5; i++)
		s2mm005_read(s, 0x8, &dummy, 1);
	udelay(10);
}

/* stock s2mm005_manual_LPM() (:412-433) */
static void s2mm005_manual_lpm(struct s2mm005 *s, u8 lpm)
{
	u8 cmd[2] = { 0x0f, lpm };

	dev_info(s->dev, "manual LPM cmd=0x%x (fw=0x%x)\n", lpm, s->firm_ver[2]);
	s2mm005_wake(s);
	s2mm005_cmd(s, cmd, sizeof(cmd));
}

/* stock s2mm005_reset() (:267-294) */
static void s2mm005_reset(struct s2mm005 *s)
{
	static const u8 cmd[] = { 0x02, 0x01, 0x1c, 0x10, 0x01 };

	s2mm005_wake(s);
	s2mm005_cmd(s, cmd, sizeof(cmd));
	msleep(100);				/* reset stable time */
}

/*
 * stock s2mm005_hard_reset() (:337-357): SCL/SDA held low for 10 ms through the bus
 * controller's "hard_reset" pin state, under the adapter lock
 */
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

/* stock toggling controls (:470-501) and s2mm005_rprd_mode_change() (:552-572) */
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

/*
 * Alternate mode: without a DisplayPort mode switch (G1-G3) the firmware is told to stop
 * alternate mode, {03 33} (S:drivers/ccic/ccic_alternate.c:232-240), at probe, after a
 * chip reset and when it reports the AP's alternate-mode request lost (RESET re-sends the
 * last command, :191-211; spec L6G 5.1.8).
 */
static void s2mm005_alt_mode_stop(struct s2mm005 *s)
{
	static const u8 cmd[] = { 0x03, 0x33 };

	s2mm005_cmd(s, cmd, sizeof(cmd));
}

/* ---- versions (S:drivers/ccic/s2mm005_fw.c:466-494) ---- */

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

/* stock s2mm005_fw_ver_check() (S:drivers/ccic/s2mm005.c:503-533) */
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

/* ---- Type-C class, role switch, VBUS, orientation ---- */

static void s2mm005_partner_detach(struct s2mm005 *s)
{
	if (s->partner) {
		typec_unregister_partner(s->partner);
		s->partner = NULL;
	}
}

/*
 * Type-C side of stock ccic_event_work() for a USB attach (S:drivers/ccic/s2mm005_cc.c:
 * 177-203): complete a pending role change, opmode USB (s2mm005_get_pd_support() is
 * constant USB, :498-501), data role, partner without PD and without identity.
 */
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

/*
 * VBUS = the OTG boost regulator, switched where stock called vbus_turn_on_ctrl()
 * (S:drivers/ccic/s2mm005_pd.c:83-105). Switching it on raises VBUS and is built only
 * with CONFIG_TYPEC_S2MM005_OTG_VBUS (default n, L6H 1A.1 row V1); off is unconditional.
 */
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

/*
 * Orientation from USB3.1_COM_SEL (gpi1-7), read at every attach before the role is set:
 * raw level 1 = CC2 = reverse, 0 = CC1 = normal (stock DP labels,
 * S:drivers/video/fbdev/exynos/dpu/displayport_drv.c:2326-2331; spec L6G 5.1.6).
 */
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

/* ---- the interrupt pass (S:drivers/ccic/s2mm005.c:574-629, s2mm005_cc.c, s2mm005_pd.c) ---- */

/* stock process_cc_water() (s2mm005_cc.c:540-627); water_detect_support = 1 (no DT property) */
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

	/* dry, :608-616 (stock CCIC WATER detach -> uevent CCIC=DRY, ccic_notifier.c:124-146) */
	if ((lp & LP_RUN_DRY) && !s->run_dry) {
		dev_info(s->dev, "== WATER RUN-DRY DETECT ==\n");
		s2mm005_uevent(s, "CCIC=DRY");
	}
	s->run_dry = lp & LP_RUN_DRY;

	/* water, :618-626 (also to the MUIC, S:drivers/usb/manager/usb_typec_manager_notifier.c:573-586) */
	if ((lp & LP_WATER_DET) && !s->water_det) {
		dev_info(s->dev, "== WATER DETECT ==\n");
		s2mm005_uevent(s, "CCIC=WATER");
		max77865_muic_set_water(s->muic, true);
	}
	s->water_det = lp & LP_WATER_DET;

	return lp;
}

/* the DisplayPort mode-switch helpers, defined with the DP state below */
static void s2mm005_dp_exit(struct s2mm005 *s);
static void s2mm005_dp_start(struct s2mm005 *s);
static void s2mm005_dock_uevent(struct s2mm005 *s, int type);

static void s2mm005_src_attach(struct s2mm005 *s)
{
	if (s->is_client) {				/* s2mm005_cc.c:735-753 */
		max77865_muic_detach(s->muic);
		s2mm005_usb_role(s, USB_ROLE_NONE);
		s2mm005_partner_detach(s);
		s->is_client = false;
		msleep(300);
	}
	if (s->is_host)
		return;

	s2mm005_orientation(s);				/* :754-783 */
	max77865_muic_attach_rprd(s->muic);
	s->is_host = true;
	s->pwr_role = TYPEC_SOURCE;
	typec_set_pwr_role(s->port, TYPEC_SOURCE);
	s2mm005_partner_attach(s, TYPEC_HOST);
	s2mm005_usb_role(s, USB_ROLE_HOST);
	s2mm005_vbus(s, true);
	dev_info(s->dev, "attach src (pd_state %d), role host, vbus %s\n", s->pd_state,
		 s->vbus_on ? "on" : "off");
}

static void s2mm005_snk_attach(struct s2mm005 *s, u32 lp, u32 fs)
{
	unsigned int rp;

	if (s->is_host) {				/* s2mm005_cc.c:801-824 */
		s2mm005_dp_exit(s);
		max77865_muic_detach(s->muic);
		s2mm005_vbus(s, false);
		s2mm005_usb_role(s, USB_ROLE_NONE);
		s2mm005_partner_detach(s);
		s->is_host = false;
		msleep(300);
	}

	if (lp & LP_PDSTATE29_SBU_DONE)			/* :826-837 */
		rp = (fs & (FS_VBUS_CC_SHORT | FS_VBUS_SBU_SHORT)) ? RP_ABNORMAL :
			FIELD_GET(FS_RP_CURRENTLVL, fs);
	else
		rp = RP_SBU_CHECK;
	max77865_muic_attach_sink(s->muic, rp);

	if (s->is_client || s->is_host)			/* :839-857 */
		return;

	s2mm005_orientation(s);
	s->is_client = true;
	s->pwr_role = TYPEC_SINK;
	typec_set_pwr_role(s->port, TYPEC_SINK);
	s2mm005_partner_attach(s, TYPEC_DEVICE);
	s2mm005_usb_role(s, USB_ROLE_DEVICE);
	dev_info(s->dev, "attach snk (pd_state %d), rp %u, role device\n", s->pd_state, rp);
}

/* dock types (S:include/linux/ccic/ccic_alternate.h:336-345) */
#define DOCK_DETACHED		0
#define DOCK_HMT		105
#define DOCK_MPA		109
#define DOCK_DEX		110
#define DOCK_HDMI		111
#define DOCK_UVDM		113
#define DOCK_DEXPAD		114
#define DOCK_NEW		200

/* stock detach branch of process_cc_attach() (s2mm005_cc.c:870-930) */
static void s2mm005_detach(struct s2mm005 *s)
{
	s->is_dr_swap = 0;
	s->is_pr_swap = 0;

	s2mm005_dp_exit(s);			/* DP detach first (:870-875) */
	s->vid = 0;				/* accessory ids off before the uevent */
	s->pid = 0;
	s2mm005_dock_uevent(s, DOCK_DETACHED);	/* ccic_dock state + uevent (:886-889) */

	max77865_muic_detach(s->muic);			/* :888-891, before VBUS off */

	if (!s->is_host && !s->is_client)
		return;

	if (s->is_host || s->pwr_role == TYPEC_SOURCE)	/* :899-901 */
		s2mm005_vbus(s, false);
	s->is_host = false;
	s->is_client = false;
	s->pwr_role = TYPEC_SINK;			/* :910-914 */
	typec_set_pwr_role(s->port, TYPEC_SINK);
	typec_set_data_role(s->port, TYPEC_DEVICE);
	typec_set_pwr_opmode(s->port, TYPEC_PWR_MODE_USB);
	s2mm005_partner_detach(s);
	if (s->orient)
		typec_set_orientation(s->port, TYPEC_ORIENTATION_NONE);
	s2mm005_usb_role(s, USB_ROLE_NONE);
	dev_info(s->dev, "detach\n");

	if (!s->try_state)				/* :926-927 */
		s2mm005_rprd_mode_change(s, TYPE_C_ATTACH_DRP);
}

/* stock process_cc_attach() (s2mm005_cc.c:632-931) */
static void s2mm005_cc_attach(struct s2mm005 *s, bool *attach_done)
{
	u8 buf[4];
	u32 lp, fs;

	*attach_done = false;
	lp = s2mm005_water(s);

	if (s->water_det)				/* :649-651 */
		return;

	if (!s->run_dry || !s->booting_run_dry) {	/* :652-668 */
		dev_info(s->dev, "Water? No Dry\n");
		s2mm005_uevent(s, "CCIC=WATER");
		max77865_muic_set_water(s->muic, true);
		if (s->i2c_err >= 0 && s2mm005_read(s, REG_FUNC_STATE, buf, 4) >= 0)
			dev_dbg(s->dev, "PD_State:%02d\n", buf[0]);
		return;
	}

	if (s->i2c_err < 0) {				/* :670-674 */
		if (s->pd_state == PD_INITIAL_DETACH)
			return;
		fs = 0;
	} else {
		memset(buf, 0, sizeof(buf));
		s2mm005_read(s, REG_FUNC_STATE, buf, 4);
		fs = get_unaligned_le32(buf);
	}
	s->pd_state = FIELD_GET(FS_PD_STATE, fs);	/* :690-694 */
	s->func_state = fs;
	dev_dbg(s->dev, "func_state 0x%x pd_state %d\n", fs, s->pd_state);

	if (fs & FS_RESET) {				/* :696-705 */
		dev_info(s->dev, "ccic reset detected\n");
		if (!(lp & LP_AUTO_LP_ENABLE))
			s2mm005_manual_lpm(s, 0x06);
		s->alt_started = false;
		if (s->dp_mux)
			s2mm005_dp_start(s);
		else
			s2mm005_alt_mode_stop(s);
	}

	if (s->pd_state == PD_SRC_WAIT_NEW_CAPABILITIES && (lp & LP_SLEEP_CABLE_DETECT)) {
		s2mm005_manual_lpm(s, 0x0d);		/* :707-711 */
		return;
	}

	if (s->pd_state == PD_INITIAL_DETACH) {
		s2mm005_detach(s);
		return;
	}

	*attach_done = true;				/* :718-726 */
	if (s->is_dr_swap || s->is_pr_swap) {
		dev_dbg(s->dev, "pd_state ignored during a %s\n", s->is_dr_swap ? "dr_swap" : "pr_swap");
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
	case PD_PRS_SRC_SNK_TRANSITION_OFF:		/* :859-866 */
		s2mm005_vbus(s, false);
		break;
	case PD_PRS_SNK_SRC_SOURCE_ON:
		s2mm005_vbus(s, true);
		break;
	default:
		break;
	}
}

/* stock DR_Swap handling of process_cc_get_int_status() (s2mm005_cc.c:1032-1074) */
static void s2mm005_dr_swap(struct s2mm005 *s)
{
	s->is_dr_swap++;
	dev_info(s->dev, "is_dr_swap count : 0x%x\n", s->is_dr_swap);

	if (s->dp_connected)			/* :1032-1034, DP connected: skip */
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

/* ---- DisplayPort alternate mode (L6G 5.1.8, G4) ----
 *
 * The firmware runs the VDM exchange; the driver reacts to the VDM IRQ flags
 * (S:include/linux/ccic/s2mm005.h:366-376) while host
 * (S:drivers/ccic/s2mm005_cc.c:1076-1081), in stock order ID, SVIDs, Modes,
 * Enter, Status Update, Configure, Attention, each stopping the chain on an
 * I2C error (S:drivers/ccic/ccic_alternate.c:899-946). DP events reach the
 * controller and the USB PHY through typec_mux (D-8); no partner altmode
 * device is registered, so the upstream typec_displayport driver never binds.
 */

/* message windows (S:include/linux/ccic/ccic_alternate.h:460-468) */
#define DP_REG_DIS_ID		0x0520
#define DP_REG_DIS_SVID		0x0540
#define DP_REG_MODE		0x0560
#define DP_REG_ENTER_MODE	0x0580
#define DP_REG_ATTENTION	0x05C0
#define DP_REG_STATUS_UPDATE	0x0600
#define DP_REG_CONFIGURE	0x0620

/* VDM IRQ flags (S:include/linux/ccic/s2mm005.h:366-376) */
#define DP_VDM_DISCOVER_ID	BIT(1)
#define DP_VDM_DISCOVER_SVIDS	BIT(2)
#define DP_VDM_DISCOVER_MODES	BIT(3)
#define DP_VDM_ENTER_MODE	BIT(4)
#define DP_VDM_ATTENTION	BIT(6)
#define DP_VDM_STATUS_UPDATE	BIT(16)
#define DP_VDM_CONFIGURE	BIT(17)

/* mode-interface commands (S:include/linux/ccic/ccic_alternate.h:472-478) */
#define DP_CMD_MODE_IFACE	0x03
#define DP_CMD_NEXT_STATE	0x02
#define DP_CMD_SAMSUNG_MODE	89
#define DP_CMD_ALT_MODE_REQ	0x30
#define DP_CMD_SVID_SELECT	0x07
#define DP_CMD_START		0x31
#define DP_CMD_CLEAR_DISCOVER	0x32
#define DP_CMD_STOP		0x33

/* DP pin assignment bits (S:include/linux/ccic/ccic_alternate.h:511-516),
 * equal to the alt-mode pin bits (L6G 5.1.8) */
#define DP_PIN_BIT_A		0x01
#define DP_PIN_BIT_B		0x02
#define DP_PIN_BIT_C		0x04
#define DP_PIN_BIT_D		0x08
#define DP_PIN_BIT_E		0x10
#define DP_PIN_BIT_F		0x20

/* CCIC_NOTIFY pin numbers (S:include/linux/ccic/ccic_notifier.h:106-112) */
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

/* stock dp_detach() (S:drivers/ccic/s2mm005_cc.c:519-535) */
static void s2mm005_dp_exit(struct s2mm005 *s)
{
	if (!s->dp_mux)
		return;
	s2mm005_dp_mux_set(s, TYPEC_STATE_USB, 0, 0);
	s->dp_connected = false;
	s->pin_sent = false;
	s->dp_status = 0;
}

static void s2mm005_dp_start(struct s2mm005 *s)
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

#if IS_ENABLED(CONFIG_ANDROID_SWITCH_COMPAT)
	if (s->dock && type != DOCK_NEW)
		switch_compat_set(s->dock, type);
#endif
	snprintf(state_str, sizeof(state_str), "SWITCH_STATE=%d", type);
	snprintf(ids_str, sizeof(ids_str), "USBPD_IDS=%04x:%04x", s->vid,
		 s->pid);
	kobject_uevent_env(&s->ccic_dev->kobj, KOBJ_CHANGE, envp);
}

/* stock process_check_accessory() (S:drivers/ccic/ccic_alternate.c:283-369):
 * DeX/Gear VR protocols are not ported (L6G 10), only the classification
 * and the dock state/uevent. */
static void s2mm005_dp_discover_id(struct s2mm005 *s)
{
	u8 msg[32] = { };
	u16 vid, pid;
	int type = DOCK_NEW;

	if (s2mm005_read(s, DP_REG_DIS_ID, msg, sizeof(msg)) < 0)
		return;
	s->vid = vid = get_unaligned_le16(&msg[8]);
	s->pid = pid = get_unaligned_le16(&msg[18]);
	s->pin_sent = false;
	dev_info(s->dev, "dp discover id vid 0x%04x pid 0x%04x\n", vid, pid);

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

/* stock Discover SVIDs (S:drivers/ccic/ccic_alternate.c:407-502): with
 * SVID0 = DP, tear down a client session, become host (as a source attach,
 * VBUS only when source, S:.../ccic_alternate.c:452-462) and report the DP
 * CONNECT + USB-DP (4-lane) event through the mux. The 2 s host-start wait
 * (:486-489) is not done: role-switch set only queues dwc3 work (L6G 5.1.8).
 */
static void s2mm005_dp_discover_svids(struct s2mm005 *s)
{
	u8 msg[32] = { };

	if (s2mm005_read(s, DP_REG_DIS_SVID, msg, sizeof(msg)) < 0)
		return;
	s->svid0 = get_unaligned_le16(&msg[8]);
	s->svid1 = get_unaligned_le16(&msg[10]);
	dev_info(s->dev, "dp discover svids 0x%04x 0x%04x\n", s->svid0,
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

/* stock Discover Modes (S:drivers/ccic/ccic_alternate.c:504-587): pin
 * capability by the port-capability/receptacle rule; Samsung SVID select;
 * then clear discover. */
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
		if (((port_cap == 1) && (recept == 1)) ||
		    ((port_cap == 2) && (recept == 0)))
			s->pin_caps = (vdo >> 16) & 0xff;
		else if (((port_cap == 2) && (recept == 1)) ||
			 ((port_cap == 1) && (recept == 0)))
			s->pin_caps = (vdo >> 8) & 0xff;
		else if (port_cap == 3)
			s->pin_caps = (recept == 0) ? ((vdo >> 8) & 0xff) :
						       ((vdo >> 16) & 0xff);
		dev_info(s->dev, "dp pin caps 0x%02x\n", s->pin_caps);
	}
	if ((vdm >> 16) == 0x04e8)
		s2mm005_cmd(s, samsung, sizeof(samsung));
	s2mm005_cmd(s, clear, sizeof(clear));
}

/* stock Enter Mode (S:drivers/ccic/ccic_alternate.c:589-621): log ACK/NAK */
static void s2mm005_dp_enter_mode(struct s2mm005 *s)
{
	u8 msg[32] = { };
	u32 vdm;

	if (s2mm005_read(s, DP_REG_ENTER_MODE, msg, sizeof(msg)) < 0)
		return;
	vdm = get_unaligned_le32(&msg[4]);
	dev_info(s->dev, "dp enter mode svid 0x%04x %s\n", vdm >> 16,
		 ((vdm >> 6) & 0x3) == 1 ? "ACK" : "NAK");
}

/* first-time pin choice, stock Attention/Status Update
 * (S:drivers/ccic/ccic_alternate.c:623-735,737-852): multi-function
 * preferred D, B, F; else C, E, A, D, B, F. Returns the pin bit, 0 = none. */
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
	dev_info(s->dev, "dp pin %c multi %d\n", "ABCDEF"[pin - 1], multi);
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

/* stock Status Update / Attention HPD handling with the pin configured
 * (L6G 5.1.8): mux_set with the mode of the chosen pin and the new status. */
static void s2mm005_dp_hpd(struct s2mm005 *s, u32 status, u32 pin_bit)
{
	unsigned long mode = TYPEC_DP_STATE_A + (s->dp_pin - 1);
	u32 conf = pin_bit << DP_CONF_PIN_ASSIGNEMENT_SHIFT;

	s->dp_status = status;
	dev_info(s->dev, "dp connect vid 0x%04x pid 0x%04x, pin %c, hpd %d\n",
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
	dev_info(s->dev, "dp status 0x%08x\n", status);

	if (s->svid0 != 0xff01)
		return;
	if (!attention && (status & 0x3) == 0x0) {
		dev_info(s->dev, "dp status: port disconnected\n");
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

/* stock DP Configure (S:drivers/ccic/ccic_alternate.c:854-897): LINK_CONF
 * with the chosen pin, then the current HPD. */
static void s2mm005_dp_configure(struct s2mm005 *s)
{
	u8 msg[32] = { };
	u32 vdm;
	static const u8 samsung[] = { DP_CMD_MODE_IFACE, DP_CMD_SVID_SELECT,
				     0x01 };

	if (s2mm005_read(s, DP_REG_CONFIGURE, msg, sizeof(msg)) < 0)
		return;
	vdm = get_unaligned_le32(&msg[4]);
	dev_info(s->dev, "dp configure svid 0x%04x\n", vdm >> 16);
	if (s->svid0 != 0xff01 || !s->pin_sent || s->dp_pin <= 0)
		return;
	s2mm005_dp_hpd(s, s->dp_status, s2mm005_dp_pin_bit(s->dp_pin));
	if ((vdm >> 16) == 0xff01 && s->svid1 == 0x04e8)
		s2mm005_cmd(s, samsung, sizeof(samsung));
}

/* VDM dispatch, only while host (S:drivers/ccic/s2mm005_cc.c:1076-1081) */
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

/* stock process_cc_get_int_status() (s2mm005_cc.c:971-1085), without VDM/SSM (G4) */
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

	if (!FIELD_GET(AP_REQ_ALT_MODE_BY_I2C, ap_req)) {	/* :1082-1083 */
		/*
		 * G4 (L6G 5.1.8): with a DP mode-switch in the DT the port starts
		 * the mode once per attach and the DP driver is told the pin
		 * assignment; without one (G1-G3) the firmware is told to stop,
		 * as stock does with no mode-switch.
		 */
		if (s->dp_mux && !s->alt_started)
			s2mm005_dp_start(s);
		else if (!s->dp_mux)
			s2mm005_alt_mode_stop(s);
	}

	return msg_irq;
}

/* stock s2mm005_src_capacity_information() (s2mm005_pd.c:107-195), fixed PDOs only kept */
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

		if (FIELD_GET(PDO_TYPE, pdo) != 0) {	/* variable / battery: logged only */
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

static void s2mm005_log_raw(struct s2mm005 *s, u16 reg)
{
	u8 buf[32] = { };

	s2mm005_read(s, reg, buf, sizeof(buf));
	dev_info(s->dev, "0x%04x: %08x %08x %08x %08x %08x %08x %08x %08x\n", reg,
		 get_unaligned_le32(&buf[0]), get_unaligned_le32(&buf[4]),
		 get_unaligned_le32(&buf[8]), get_unaligned_le32(&buf[12]),
		 get_unaligned_le32(&buf[16]), get_unaligned_le32(&buf[20]),
		 get_unaligned_le32(&buf[24]), get_unaligned_le32(&buf[28]));
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

/*
 * stock process_pd() (s2mm005_pd.c:197-357, CONFIG_USB_TYPEC_MANAGER_NOTIFIER branch as
 * IKC:3586): read-only; the event goes to the notifier after the lock is released
 */
static void s2mm005_pd(struct s2mm005 *s, bool attach_done, u32 msg_irq,
		       struct s2mm005_pd_note *note)
{
	unsigned int rp = FIELD_GET(FS_RP_CURRENTLVL, s->func_state);
	bool is_src = s->func_state & FS_IS_SOURCE;
	bool pdic_attach = false;
	unsigned long event = S2MM005_PD_EVENT_DETACH;
	int i;

	if (msg_irq & MSG_PR_SWAP) {			/* :217-249 */
		s->is_pr_swap++;
		dev_info(s->dev, "PR_Swap requested to %s\n", is_src ? "SOURCE" : "SINK");
		if (is_src && s->pwr_role == TYPEC_SINK) {	/* battery detach, :226-228 */
			s2mm005_pd_queue(s, note, S2MM005_PD_EVENT_DETACH);
			s->pd_attached = false;
		}
		s2mm005_vbus(s, is_src);
		s->pwr_role = is_src ? TYPEC_SOURCE : TYPEC_SINK;
		typec_set_pwr_role(s->port, s->pwr_role);
		typec_set_pwr_opmode(s->port, TYPEC_PWR_MODE_USB);
	}

	if (msg_irq & MSG_SRC_CAPABILITY) {		/* :251-294 */
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

		/* the firmware's own request, which may be above 5 V (spec L6G 5.1.10, V-15) */
		dev_info(s->dev, "pd src caps %u: %s, fw requested pos %u (%u mV %u mA)\n",
			 available, caps, pos, s->pd.power_list[pos].max_voltage,
			 s->pd.power_list[pos].max_current);
		if (!s->pd_raw_logged) {		/* L6H O-H1, read-only */
			s2mm005_log_raw(s, REG_TX_SINK_CAPA_MSG);
			s2mm005_log_raw(s, REG_TX_REQUEST_MSG);
			s->pd_raw_logged = true;
		}
	}

	if (msg_irq & MSG_GET_SINK_CAP)
		dev_dbg(s->dev, "SRC requested SINK Cap\n");

	if (attach_done) {				/* :303-329 */
		if (pdic_attach) {
			/* PD charger is detected by the CCIC */
		} else if (!is_src && (s->pd_state == PD_SNK_WAIT_FOR_CAPABILITIES ||
				       s->pd_state == PD_ERROR_RECOVERY) &&
			   rp != s->pd.rp_currentlvl && rp >= S2MM005_RP_CURRENT_LEVEL_DEFAULT) {
			s->pd.rp_currentlvl = rp;	/* 5 V 0.5/1.5/3 A Rp charger */
			event = S2MM005_PD_EVENT_CCIC_ATTACH;
		} else {
			return;
		}
		s2mm005_pd_queue(s, note, event);
		s->pd_attached = true;
	} else {					/* :330-340 */
		for (i = 0; i < S2MM005_MAX_PDO_NUM + 1; i++) {
			s->pd.power_list[i].max_current = 0;
			s->pd.power_list[i].max_voltage = 0;
		}
		s->pd.rp_currentlvl = S2MM005_RP_CURRENT_LEVEL_NONE;
		s->pd.available_pdo_num = 0;
		s->pd.selected_pdo_num = 0;
		s->pd.current_pdo_num = 0;
		if (s->pd_attached) {		/* posted once per detach */
			s2mm005_pd_queue(s, note, S2MM005_PD_EVENT_DETACH);
			s->pd_attached = false;
		}
	}
}

/* stock process_cc_rid() (s2mm005_cc.c:1090-1132) */
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
		if (rid == RID_523K || rid == RID_619K) {	/* :1114-1126 */
			s->is_client = false;
			s2mm005_usb_role(s, USB_ROLE_NONE);
			s2mm005_partner_detach(s);
		}
	}
	s->prev_rid = rid;
}

/* stock process_cc_water_det() (s2mm005_cc.c:504-517) */
static void s2mm005_water_det(struct s2mm005 *s)
{
	s2mm005_int_clear(s);
	if (s->water_det)
		s2mm005_manual_lpm(s, 0x09);
}

/* one pass of stock s2mm005_usbpd_irq_thread() (S:drivers/ccic/s2mm005.c:574-629), lock held */
static void s2mm005_pass(struct s2mm005 *s, struct s2mm005_pd_note *note)
{
	bool attach_done;
	u32 msg_irq, vdm_irq;

	pm_wakeup_event(s->dev, 1000);			/* stock wake_lock_timeout(HZ) :594 */

	if (s2mm005_fw_ver_check(s))			/* :596-598 */
		return;

	s2mm005_cc_attach(s, &attach_done);		/* :601 */
	if (s->i2c_err < 0)				/* :603-604 */
		return;

	if (s->water_det || !s->run_dry || !s->booting_run_dry) {
		s2mm005_water_det(s);			/* :606-609 */
		return;
	}

	msg_irq = s2mm005_get_int_status(s, &vdm_irq);	/* :612 */
	s2mm005_dp_vdm(s, vdm_irq);		/* alternate mode, only while host */
	s2mm005_pd(s, attach_done, msg_irq, note);	/* :615 */
	s2mm005_cc_rid(s);				/* :618 */
}

static void s2mm005_post(struct s2mm005 *s, struct s2mm005_pd_note *note)
{
	unsigned int i;

	for (i = 0; i < note->n; i++)
		blocking_notifier_call_chain(&s->pd_nh, note->ev[i].event, &note->ev[i].status);
}

static irqreturn_t s2mm005_irq_thread(int irq, void *data)
{
	struct s2mm005 *s = data;
	struct s2mm005_pd_note note = { };

	mutex_lock(&s->lock);
	if (s->check_at_booting) {			/* :586-589 */
		s->check_at_booting = false;
		/* not _sync: the work takes the lock and re-checks the flag */
		cancel_delayed_work(&s->boot_check);
	}
	s2mm005_pass(s, &note);
	mutex_unlock(&s->lock);

	s2mm005_post(s, &note);
	return IRQ_HANDLED;
}

/* stock ccic_state_check_work() (S:drivers/ccic/s2mm005.c:670-680): one pass if no IRQ came */
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

	s2mm005_post(s, &note);
}

/* stock typec_role_swap_check() (S:drivers/ccic/s2mm005_cc.c:412-431) */
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
 * stock s2mm005_port_type_set() (S:drivers/ccic/s2mm005_cc.c:433-496): force the firmware
 * into source or sink, wait up to 1.5 s for the new role, else back to DRP. The lock is
 * not held across the wait (the IRQ thread completes it).
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
		s2mm005_vbus(s, false);			/* "turns off VBUS first" :454-455 */
		max77865_muic_detach(s->muic);		/* :456-460 */
		break;
	case TYPEC_PORT_DRP:				/* :464-468 */
		return 0;
	default:
		return -EINVAL;
	}
	dev_info(s->dev, "port type change to %s\n", type == TYPEC_PORT_SRC ? "source" : "sink");
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

/* ---- sec/ccic sysfs (S:drivers/ccic/ccic_sysfs.c; LOS USB HAL reads "water") ---- */

static ssize_t cur_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct s2mm005 *s = dev_get_drvdata(dev);
	struct s2mm005_version sw = { };
	u8 v[4];

	mutex_lock(&s->lock);				/* ccic_sysfs.c:34-53 */
	s2mm005_get_swversion(s, &sw);
	s2mm005_store_swversion(s, &sw);
	memcpy(v, s->firm_ver, sizeof(v));
	mutex_unlock(&s->lock);

	return sysfs_emit(buf, "%02X %02X %02X %02X\n", v[0], v[1], v[2], v[3]);
}
static DEVICE_ATTR_RO(cur_version);

static ssize_t src_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* ccic_sysfs.c:56-67: the bundled header's product, sub, main, boot */
	return sysfs_emit(buf, "%02X %02X %02X %02X\n", BUNDLED_FW_MAIN2, BUNDLED_FW_MAIN1,
			  BUNDLED_FW_MAIN0, BUNDLED_FW_BOOT);
}
static DEVICE_ATTR_RO(src_version);

static ssize_t state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct s2mm005 *s = dev_get_drvdata(dev);
	int pd_state;

	mutex_lock(&s->lock);				/* ccic_sysfs.c:125-137 */
	pd_state = s->pd_state;
	mutex_unlock(&s->lock);

	return sysfs_emit(buf, "%d\n", pd_state);
}
static DEVICE_ATTR_RO(state);

static ssize_t water_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct s2mm005 *s = dev_get_drvdata(dev);
	int water;

	mutex_lock(&s->lock);				/* ccic_sysfs.c:675-689 */
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

/* ---- exported interface (spec L6G 5.1.10, L6H REQ-L6G-3) ---- */

struct s2mm005 *s2mm005_get(struct device_node *np)
{
	struct i2c_client *client = of_find_i2c_device_by_node(np);
	struct s2mm005 *s;

	if (!client)
		return ERR_PTR(-EPROBE_DEFER);

	s = client->dev.driver == &s2mm005_driver.driver ? i2c_get_clientdata(client) : NULL;
	if (!s) {
		put_device(&client->dev);
		return ERR_PTR(-EPROBE_DEFER);
	}
	return s;
}
EXPORT_SYMBOL_GPL(s2mm005_get);

void s2mm005_put(struct s2mm005 *s)
{
	put_device(s->dev);
}
EXPORT_SYMBOL_GPL(s2mm005_put);

int s2mm005_register_notifier(struct s2mm005 *s, struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&s->pd_nh, nb);
}
EXPORT_SYMBOL_GPL(s2mm005_register_notifier);

int s2mm005_unregister_notifier(struct s2mm005 *s, struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&s->pd_nh, nb);
}
EXPORT_SYMBOL_GPL(s2mm005_unregister_notifier);

#if IS_ENABLED(CONFIG_TYPEC_S2MM005_PD_SELECT)
/* the port whose PDO the charger selects (stock pd_noti.pusbpd, S:drivers/ccic/s2mm005.c:808) */
static struct s2mm005 *s2mm005_pd_port;

/* stock s2mm005_select_pdo() (S:drivers/ccic/s2mm005_pd.c:52-75) */
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
	dev_info(s->dev, "PDO(%d) is selected to change\n", s->pd.selected_pdo_num);

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

/* ---- probe ---- */

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

	/* what the DT hangs on the connector is looked up on the connector (spec L6G 5.1.3, V-9) */
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
		s->vbus = NULL;				/* G1 DT: no VBUS */
	} else {
		ret = devm_add_action_or_reset(dev, s2mm005_vbus_off, s);
		if (ret)
			return ret;
	}

	/* DisplayPort mode-switch (G4, L6G 5.1.8): NULL without a DP
	 * mode-switch in the DT (then the firmware is told to stop);
	 * -EPROBE_DEFER while a linked provider is not bound yet */
	s->dp_mux = fwnode_typec_mux_get(s->connector);
	if (IS_ERR(s->dp_mux))
		return dev_err_probe(dev, PTR_ERR(s->dp_mux), "DP mux\n");
	if (s->dp_mux) {
		ret = devm_add_action_or_reset(dev, s2mm005_mux_put,
					       s->dp_mux);
		if (ret)
			return ret;
	}

	/*
	 * ccic_dock for the camera HAL (L6G 5.1.8, 5.7.5). The Android switch
	 * class device is a G4 file (drivers/misc/android-switch-compat.c, the
	 * G-15 commit) and is not in this image (D-16, O-G13), so the dock state
	 * is only published as the uevent above (V-14).
	 */
#if IS_ENABLED(CONFIG_ANDROID_SWITCH_COMPAT)
	ret = devm_switch_compat_register(dev, "ccic_dock", &s->dock);
	if (ret)
		return ret;
#endif

	/* the bus controller's "hard_reset" pin state (4.1); stock looked it up per reset (:340-352) */
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

/* probe steps 4-7 of spec L6G 5.1.3 (S:drivers/ccic/s2mm005.c:849-996) */
static int s2mm005_chip_init(struct s2mm005 *s)
{
	struct s2mm005_version hw = { }, sw = { };
	static const u8 lpm_enable[] = { 0x0f, 0x06 };
	u8 msg[32] = { };
	u8 w[8], r[4] = { }, ftrim;
	int ret;

	ret = s2mm005_read(s, REG_TX_SINK_CAPA_MSG, msg, sizeof(msg));
	if (ret < 0) {					/* :855-865, stock panicked here */
		s2mm005_hard_reset(s);
		msleep(1000);
		ret = s2mm005_read(s, REG_TX_SINK_CAPA_MSG, msg, sizeof(msg));
		if (ret < 0) {
			dev_err(s->dev, "no answer at 0x220 after hard reset (%d)\n", ret);
			return -ENODEV;
		}
	}

	s2mm005_get_hwversion(s, &hw);			/* :867-871 */

	if (hw.boot <= 2) {				/* ftrim fix-up, :873-893 */
		w[0] = 0x02; w[1] = 0x40; w[2] = 0x04; w[3] = 0x11;
		s2mm005_cmd(s, w, 4);
		s2mm005_read(s, REG_FTRIM_RSP, r, 4);
		dev_err(s->dev, "ftrim:%02X %02X %02X %02X\n", r[0], r[1], r[2], r[3]);
		ftrim = ((r[1] & 0xf8) >> 3) - 2;
		r[1] = (ftrim << 3) + (r[1] & 0x7);
		dev_err(s->dev, "ftrim:%02X %02X %02X %02X\n", r[0], r[1], r[2], r[3]);
		w[0] = 0x02; w[1] = 0x04; w[2] = 0x04; w[3] = 0x11;
		w[4] = r[0]; w[5] = r[1]; w[6] = r[2]; w[7] = r[3];
		s2mm005_cmd(s, w, 8);
		w[0] = 0x02; w[1] = 0x40; w[2] = 0x04; w[3] = 0x11;
		s2mm005_cmd(s, w, 4);
		s2mm005_read(s, REG_FTRIM_RSP, r, 4);
		dev_err(s->dev, "ftrim:%02X %02X %02X %02X\n", r[0], r[1], r[2], r[3]);
	}

	s2mm005_get_swversion(s, &sw);			/* :895-899 */
	dev_info(s->dev, "hw %02x.%02x.%02x boot %u, chip fw %02x.%02x.%02x boot %u, bundled fw %02x.%02x.%02x\n",
		 hw.main[2], hw.main[1], hw.main[0], hw.boot,
		 sw.main[2], sw.main[1], sw.main[0], sw.boot,
		 BUNDLED_FW_MAIN2, BUNDLED_FW_MAIN1, BUNDLED_FW_MAIN0);

	/* stock's non-factory flash condition (:918, :935-941); the port never flashes (D-14) */
	if (sw.boot == 0x7 &&
	    (sw.main[0] < BUNDLED_FW_MAIN0 ||
	     (sw.main[0] == BUNDLED_FW_MAIN0 && sw.main[1] < BUNDLED_FW_MAIN1) ||
	     sw.main[2] != BUNDLED_FW_MAIN2 ||
	     (((sw.main[2] == 0xff && sw.main[1] == 0xa5) || sw.main[2] == 0x00) &&
	      BUNDLED_FW_MAIN2 != 0x0)))
		dev_info(s->dev, "chip fw %02x.%02x older than bundled %02x.%02x, stock would update (not done)\n",
			 sw.main[1], sw.main[0], BUNDLED_FW_MAIN1, BUNDLED_FW_MAIN0);

	s2mm005_store_swversion(s, &sw);		/* :951-956 */

	/* sink capabilities, read only: stock's write is commented out (:958-989, D-13) */
	memset(msg, 0, sizeof(msg));
	s2mm005_read(s, REG_TX_SINK_CAPA_MSG, msg, sizeof(msg));
	dev_info(s->dev, "sink caps %08x %08x %08x %08x %08x %08x %08x %08x\n",
		 get_unaligned_le32(&msg[0]), get_unaligned_le32(&msg[4]),
		 get_unaligned_le32(&msg[8]), get_unaligned_le32(&msg[12]),
		 get_unaligned_le32(&msg[16]), get_unaligned_le32(&msg[20]),
		 get_unaligned_le32(&msg[24]), get_unaligned_le32(&msg[28]));

	/* CONFIG_CCIC_LPM_ENABLE (IKC:4742): plain command write, no wake prologue (:991-996) */
	s2mm005_cmd(s, lpm_enable, sizeof(lpm_enable));
	return 0;
}

static int s2mm005_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct s2mm005 *s;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))	/* :766-769 */
		return dev_err_probe(dev, -EIO, "i2c functionality check error\n");

	s = devm_kzalloc(dev, sizeof(*s), GFP_KERNEL);
	if (!s)
		return -ENOMEM;
	s->dev = dev;
	s->client = client;
	s->irq = client->irq;
	/* stock initial values, :811-825 and s2mm005_cc.c:1094 */
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
	BLOCKING_INIT_NOTIFIER_HEAD(&s->pd_nh);

	ret = s2mm005_get_resources(s);
	if (ret)
		return ret;

	ret = s2mm005_chip_init(s);
	if (ret)
		return ret;

	/* Type-C class port, stock values (:1022-1035; spec L6G 5.1.5) */
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

	/* /sys/devices/virtual/sec/ccic (spec L6G 5.1.9) */
	s->ccic_dev = sec_device_create_with_groups(s, s2mm005_ccic_groups, "ccic");
	if (IS_ERR(s->ccic_dev)) {
		dev_err(dev, "cannot create sec/ccic (%ld)\n", PTR_ERR(s->ccic_dev));
		s->ccic_dev = NULL;
	} else {
		ret = devm_add_action_or_reset(dev, s2mm005_ccic_destroy, s->ccic_dev);
		if (ret)
			return ret;
	}

	s2mm005_alt_mode_stop(s);

	/* registered after everything the works use and before the IRQ (bug class 2) */
	ret = devm_add_action_or_reset(dev, s2mm005_works_disable, s);
	if (ret)
		return ret;

	s2mm005_int_clear(s);				/* :1065 */

	s->check_at_booting = true;			/* :1068-1070 */
	schedule_delayed_work(&s->boot_check, msecs_to_jiffies(200));

	device_init_wakeup(dev, true);			/* :807 */
	/* stock "s2mm005-usbpd", falling edge from DT (:1072-1073) */
	ret = devm_request_threaded_irq(dev, s->irq, NULL, s2mm005_irq_thread, IRQF_ONESHOT,
					"s2mm005-usbpd", s);
	if (ret)
		return dev_err_probe(dev, ret, "cannot request IRQ %d\n", s->irq);

	s2mm005_int_clear(s);				/* :1106 */

#if IS_ENABLED(CONFIG_TYPEC_S2MM005_PD_SELECT)
	WRITE_ONCE(s2mm005_pd_port, s);
	ret = devm_add_action_or_reset(dev, s2mm005_pd_port_clear, NULL);
	if (ret)
		return ret;
#endif
	i2c_set_clientdata(client, s);
	return 0;
}

/* stock s2mm005_usbpd_shutdown() (S:drivers/ccic/s2mm005.c:1155-1191) */
static void s2mm005_shutdown(struct i2c_client *client)
{
	struct s2mm005 *s = i2c_get_clientdata(client);

	if (!s)
		return;

	disable_irq(s->irq);
	mutex_lock(&s->lock);
	if (s->cur_rid != RID_523K && s->cur_rid != RID_619K) {
		dev_info(s->dev, "shutdown: pd_state=%d, water=%d, dry=%d\n", s->pd_state,
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

/* stock s2mm005_suspend()/s2mm005_resume() (S:drivers/ccic/s2mm005.c:1194-1226) */
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
	{ "usbpd-s2mm005" },			/* stock USBPD005_DEV_NAME, s2mm005.h:61 */
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
