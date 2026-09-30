// SPDX-License-Identifier: GPL-2.0
/*
 * IDT wireless charging receiver of the Samsung Galaxy Note 8 (greatlte), I2C 0x3b on
 * hsi2c_29. Port of Samsung's drivers/battery_v2/mfc_charger.c (registers from its
 * include/charger/mfc_charger.h).
 *
 * CONFIG_WIRELESS_CHARGER_IDT_MFC: read and report only; the RX output stays
 * disabled (wpc_en high) and nothing but the interrupt enable/clear registers is written.
 * CONFIG_WIRELESS_CHARGER_IDT_MFC_NV: the RX output follows the charger's
 * charge control, plus the stock 5 V commands (FOD, RPP, headroom, CV Vout, CS100, EPT).
 * CONFIG_WIRELESS_CHARGER_IDT_MFC_HV: 10 V on HV pads, only on the charger's
 * grant. The current/thermal policy is the MAX77865 charger's.
 *
 * Not ported: firmware update and UNO boost, raw register sysfs, MST, A4WP
 * commands, LED/fan pad commands.
 */

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/pm_wakeup.h>
#include <linux/power/idt_mfc_wpc.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/seq_file.h>
#include <linux/workqueue.h>

/* registers */
#define IDT_CHIP_ID_L			0x00
#define IDT_CHIP_REVISION		0x02
#define IDT_FW_MAJOR_REV_L		0x04
#define IDT_FW_MINOR_REV_L		0x06
#define IDT_INT_A_L			0x22
#define IDT_INT_A_H			0x23
#define IDT_INT_A_ENABLE_L		0x24
#define IDT_INT_A_CLEAR_L		0x26
#define IDT_INT_A_CLEAR_H		0x27
#define IDT_SYS_OP_MODE			0x2B
#define IDT_BATTERY_CHG_STATUS		0x3A
#define IDT_EPT				0x3B
#define IDT_ADC_VOUT_L			0x3C
#define IDT_VOUT_SET			0x3E
#define IDT_VRECT_ADJ			0x3F
#define IDT_ADC_VRECT_L			0x40
#define IDT_AP2MFC_CMD_L		0x4E
#define IDT_WPC_PCKT_HEADER		0x50
#define IDT_WPC_RX_DATA_COM		0x51
#define IDT_WPC_RX_DATA_VALUE0		0x52
#define IDT_WPC_TX_DATA_COM		0x58
#define IDT_WPC_TX_DATA_VALUE0		0x59
#define IDT_RX_COMM_MOD_FET		0x6B
#define IDT_WPC_FOD_0A			0x70
#define IDT_PMA_FOD_0A			0x80
#define IDT_BT2AP_DATA_COM		0xA6	/* A4WP, logged only */
#define IDT_RPP_SCALE_COEF		0xF0
#define IDT_MAX_REGISTER		0xF0
#define IDT_NUM_FOD_REG			12

#define IDT_CHIP_LSI			0x40

/* INT_A / STATUS bits */
#define IDT_STAT_L_OP_MODE		BIT(5)
#define IDT_STAT_L_OVER_VOL		BIT(4)
#define IDT_STAT_L_OVER_CURR		BIT(3)
#define IDT_STAT_L_OVER_TEMP		BIT(2)
#define IDT_STAT_L_BT2AP_DATA		BIT(0)
#define IDT_STAT_H_TX_DATA_RECEIVED	BIT(7)

/* COM_L bits */
#define IDT_CMD_CLEAR_INT		BIT(5)
#define IDT_CMD_SEND_CHG_STS		BIT(4)
#define IDT_CMD_SEND_EOP		BIT(3)
#define IDT_CMD_MCU_RESET		BIT(2)
#define IDT_CMD_SEND_RX_DATA		BIT(0)

/* Sys_Op_Mode [7:5] */
#define PAD_MODE_WPC_BASIC		1
#define PAD_MODE_WPC_ADV		2
#define PAD_MODE_PMA_SR1		3
#define PAD_MODE_PMA_SR1E		4
#define PAD_MODE_A4WP			5
#define PAD_MODE_A4WP_LPM		6

/* TX -> RX data */
#define WPC_TX_COM_TX_ID		0x01
#define WPC_TX_COM_AFC_SET		0x02
#define WPC_TX_COM_CHG_ERR		0x05
#define TX_AFC_SET_5V			0x00
#define TX_AFC_SET_10V			0x01
#define TX_ID_VEHICLE_PAD		0x11
#define TX_ID_STAND_TYPE_START		0x30
#define TX_ID_BATT_PACK			0x40
#define TX_ID_BATT_PACK_TA		0x41
#define TX_CHG_ERR_OTP			0x12
#define TX_CHG_ERR_OCP			0x13
#define TX_CHG_ERR_DARKZONE		0x14
#define TX_CHG_ERR_FOD_FIRST		0x20	/* TX_CHG_ERR_FOD 0x20 ... 0x27 */
#define TX_CHG_ERR_FOD_LAST		0x27

#define IDT_CMD_CNT			3	/* CMD_CNT */

/* VOUT_SET codes of the IDT chip, index = enum idt_mfc_vout (mfc_idt_vout_val) */
static const u8 idt_vout_val[] = { 0x0F, 0x19, 0x23, 0x2D, 0x37, 0x41, 0x14 };
static const int idt_vout_mv[] = { 5000, 6000, 7000, 8000, 9000, 10000, 5500 };

struct idt_mfc_wpc {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *psy;
	struct mutex lock;		/* state below and every command sequence */
	struct gpio_desc *det_gpio;
	struct gpio_desc *int_gpio;
	struct gpio_desc *en_gpio;	/* "disable": 1 = physical high = RX output off */
	int irq_det, irq_int;
	struct delayed_work det_work;
	struct delayed_work isr_work;

	bool wc_w_state;		/* det level the work last handled (stock wc_w_state) */
	bool online;
	bool fod;
	enum idt_mfc_pad pad;
	enum idt_mfc_vout vout;

	u8 fod_wpc[IDT_NUM_FOD_REG];
	u8 fod_pma[IDT_NUM_FOD_REG];
	u8 fod_wpc_cv[IDT_NUM_FOD_REG];
	u8 fod_pma_cv[IDT_NUM_FOD_REG];
	u8 fod_hero_5v[IDT_NUM_FOD_REG];
	bool fod_ok;			/* every FOD table present (stock fod_data_check) */
	u8 rpp_hv;
#if IS_ENABLED(CONFIG_WIRELESS_CHARGER_IDT_MFC_NV)
	bool rx_on;
#endif
#if IS_ENABLED(CONFIG_WIRELESS_CHARGER_IDT_MFC_HV)
	struct delayed_work afc_vout_work;
	struct delayed_work vout_step_work;
	bool hv_grant;			/* the charger's 10 V grant */
	bool pad_10v;			/* stock pad_vout == PAD_VOUT_10V */
	bool full;			/* stock is_full_status */
#endif
};

/* the receiver; set at the end of probe, never cleared (no unbind: suppress_bind_attrs) */
static struct idt_mfc_wpc *idt_wpc;

static const char * const idt_pad_name[] = {
	[IDT_MFC_PAD_NONE] = "none", [IDT_MFC_PAD_WPC] = "wpc", [IDT_MFC_PAD_WPC_AFC] = "wpc-hv",
	[IDT_MFC_PAD_WPC_PACK] = "pack", [IDT_MFC_PAD_WPC_PACK_TA] = "pack-ta",
	[IDT_MFC_PAD_WPC_STAND] = "stand", [IDT_MFC_PAD_WPC_STAND_HV] = "stand-hv",
	[IDT_MFC_PAD_PMA] = "pma", [IDT_MFC_PAD_WPC_VEHICLE] = "vehicle",
	[IDT_MFC_PAD_WPC_VEHICLE_HV] = "vehicle-hv", [IDT_MFC_PAD_PREPARE_HV] = "prepare-hv",
	[IDT_MFC_PAD_A4WP] = "a4wp",
};

static const struct regmap_config idt_regmap_config = {
	.reg_bits = 16,			/* 16-bit address, big endian, then data */
	.val_bits = 8,
	.max_register = IDT_MAX_REGISTER,
	.cache_type = REGCACHE_NONE,
};

/* stock mfc_set_cmd_l_reg(), loop and exits copied */
static void idt_set_cmd_l(struct idt_mfc_wpc *w, u8 val, u8 mask)
{
	unsigned int temp = 0;
	int ret, i = 0;

	do {
		ret = regmap_update_bits(w->regmap, IDT_AP2MFC_CMD_L, mask, val);
		if (ret >= 0) {
			ret = regmap_read(w->regmap, IDT_AP2MFC_CMD_L, &temp);
			if (ret < 0 || i > 3)
				break;
		}
		i++;
	} while (temp != 0 && i < 3);
}

/* stock mfc_get_adc() VOUT/VRECT: low byte, high byte; -1 on a failed read */
static int idt_adc(struct idt_mfc_wpc *w, unsigned int reg_l)
{
	unsigned int l, h;

	if (regmap_read(w->regmap, reg_l, &l) || regmap_read(w->regmap, reg_l + 1, &h))
		return -1;
	return l | h << 8;
}

/* stock INT clear: write the read sources back, then CLEAR_INT */
static void idt_clear_int(struct idt_mfc_wpc *w, unsigned int l, unsigned int h)
{
	regmap_write(w->regmap, IDT_INT_A_CLEAR_L, l);
	regmap_write(w->regmap, IDT_INT_A_CLEAR_H, h);
	idt_set_cmd_l(w, IDT_CMD_CLEAR_INT, IDT_CMD_CLEAR_INT);
}

#if IS_ENABLED(CONFIG_WIRELESS_CHARGER_IDT_MFC_NV)
/* RX output on only while the charger has charge control and no fault */
static void idt_rx_set(struct idt_mfc_wpc *w)
{
	bool on = max77865_charger_wpc_allowed();

	if (on == w->rx_on)
		return;
	gpiod_set_value_cansleep(w->en_gpio, !on);
	w->rx_on = on;
	dev_info(w->dev, "wpc_en %s (rx %s)\n", on ? "low" : "high", on ? "on" : "off");
}

/* stock mfc_set_vout(), IDT codes */
static void idt_set_vout(struct idt_mfc_wpc *w, enum idt_mfc_vout v)
{
	regmap_write(w->regmap, IDT_VOUT_SET, idt_vout_val[v]);
	msleep(100);
	w->vout = v;
	dev_info(w->dev, "vout %d mV (adc %d mV)\n", idt_vout_mv[v], idt_adc(w, IDT_ADC_VOUT_L));
}

static void idt_write_fod(struct idt_mfc_wpc *w, unsigned int base, const u8 *fod)
{
	int i;

	for (i = 0; i < IDT_NUM_FOD_REG; i++)
		regmap_write(w->regmap, base + i, fod[i]);
}

/* stock mfc_fod_set() without the A4WP table */
static void idt_fod_set(struct idt_mfc_wpc *w, bool cv)
{
	if (!w->fod_ok)
		return;
	switch (w->pad) {
	case IDT_MFC_PAD_PMA:
		idt_write_fod(w, IDT_PMA_FOD_0A, cv ? w->fod_pma_cv : w->fod_pma);
		break;
	case IDT_MFC_PAD_WPC:
	case IDT_MFC_PAD_WPC_AFC:
	case IDT_MFC_PAD_WPC_PACK:
	case IDT_MFC_PAD_WPC_PACK_TA:
	case IDT_MFC_PAD_WPC_STAND:
	case IDT_MFC_PAD_WPC_STAND_HV:
		idt_write_fod(w, IDT_WPC_FOD_0A, cv ? w->fod_wpc_cv : w->fod_wpc);
		break;
	default:
		break;
	}
}

/* stock mfc_rpp_set() without the LED cover input */
static void idt_rpp_set(struct idt_mfc_wpc *w)
{
	unsigned int v = 0;

	regmap_write(w->regmap, IDT_RPP_SCALE_COEF, w->rpp_hv);
	msleep(5);
	if (!regmap_read(w->regmap, IDT_RPP_SCALE_COEF, &v))
		dev_info(w->dev, "rpp scaling coefficient 0x%02x\n", v);
}

/* stock mfc_set_vrect_adjust(): six writes 50 ms apart */
static void idt_headroom(struct idt_mfc_wpc *w, int headroom)
{
	static const u8 adj[] = { 0x00, 0x36, 0x61, 0x7f, 0x06, 0x10 };
	int i;

	for (i = 0; i < 6; i++) {
		regmap_write(w->regmap, IDT_VRECT_ADJ, adj[headroom]);
		msleep(50);
	}
}
#endif

#if IS_ENABLED(CONFIG_WIRELESS_CHARGER_IDT_MFC_HV)
/* stock mfc_send_packet() WPC branch with one data byte */
static void idt_send_packet(struct idt_mfc_wpc *w, u8 header, u8 com, u8 data)
{
	regmap_write(w->regmap, IDT_WPC_PCKT_HEADER, header);
	regmap_write(w->regmap, IDT_WPC_RX_DATA_COM, com);
	regmap_write(w->regmap, IDT_WPC_RX_DATA_VALUE0, data);
	idt_set_cmd_l(w, IDT_CMD_SEND_RX_DATA, IDT_CMD_SEND_RX_DATA);
}

static bool idt_pad_is_hv(enum idt_mfc_pad pad)
{
	return pad == IDT_MFC_PAD_WPC_AFC || pad == IDT_MFC_PAD_WPC_STAND_HV ||
	       pad == IDT_MFC_PAD_WPC_VEHICLE_HV;
}

/*
 * stock mfc_wpc_vout_mode_work() WIRELESS_VOUT_5V_STEP: one step
 * down per 250 ms, then at <= 5 V and full on an HV pad the "TX 5 V" packet
 */
static void idt_vout_step_work(struct work_struct *work)
{
	struct idt_mfc_wpc *w = container_of(to_delayed_work(work), struct idt_mfc_wpc,
					     vout_step_work);

	mutex_lock(&w->lock);
	if (!w->wc_w_state)
		goto out;
	if (w->vout > IDT_MFC_VOUT_5V && w->vout <= IDT_MFC_VOUT_10V) {
		idt_set_vout(w, w->vout - 1);
		queue_delayed_work(system_freezable_wq, &w->vout_step_work, msecs_to_jiffies(250));
		goto out;
	}
	if (w->vout == IDT_MFC_VOUT_5_5V)	/* unlike stock: never step "down" from 5.5 V to 10 V */
		idt_set_vout(w, IDT_MFC_VOUT_5V);
	if (idt_pad_is_hv(w->pad) && w->full) {
		idt_send_packet(w, 0x28, 0x06, 0x05);	/* HEADER_AFC_CONF, WPC_COM_AFC_SET, 5 V */
		dev_info(w->dev, "tx 5 V after cs100\n");
	}
out:
	mutex_unlock(&w->lock);
}

/*
 * stock mfc_wpc_afc_vout_work(), queued 8 s after the AFC_CONF_10V request:
 * HV pad type, three AFC_SET 10 V packets, all CM FETs, then Vout 10 V. Unlike
 * stock, only while the charger's grant holds (it also covers stock's swelling check).
 */
static void idt_afc_vout_work(struct work_struct *work)
{
	struct idt_mfc_wpc *w = container_of(to_delayed_work(work), struct idt_mfc_wpc,
					     afc_vout_work);
	unsigned int v = 0;
	int i;

	mutex_lock(&w->lock);
	if (!w->wc_w_state || !w->online)
		goto out;
	if (w->pad != IDT_MFC_PAD_WPC_STAND_HV && w->pad != IDT_MFC_PAD_WPC_VEHICLE_HV)
		w->pad = IDT_MFC_PAD_WPC_AFC;
	if (!w->hv_grant) {
		dev_info(w->dev, "10 V withdrawn before the pad switched\n");
		goto out;
	}
	for (i = 0; i < IDT_CMD_CNT; i++) {
		idt_send_packet(w, 0x28, 0x06, 0x2c);	/* HEADER_AFC_CONF, WPC_COM_AFC_SET, 10 V */
		msleep(100);
	}
	regmap_write(w->regmap, IDT_RX_COMM_MOD_FET, 0x00);
	if (!regmap_read(w->regmap, IDT_RX_COMM_MOD_FET, &v))
		dev_info(w->dev, "cm fet 0x%02x\n", v);
	if (!w->full)
		idt_set_vout(w, IDT_MFC_VOUT_10V);
out:
	mutex_unlock(&w->lock);
	power_supply_changed(w->psy);
}
#endif

/* stock mfc_wpc_det_work() */
static void idt_det_work(struct work_struct *work)
{
	struct idt_mfc_wpc *w = container_of(to_delayed_work(work), struct idt_mfc_wpc, det_work);
	int det = gpiod_get_value_cansleep(w->det_gpio);
	unsigned int id = 0, adj = 0, mode = 0;
	int vrect, vout;

	mutex_lock(&w->lock);
	if (!w->wc_w_state && det == 1) {
		w->vout = IDT_MFC_VOUT_5V;
		w->fod = false;
		regmap_read(w->regmap, IDT_CHIP_ID_L, &id);
		/* mode change interrupt enable */
		regmap_update_bits(w->regmap, IDT_INT_A_ENABLE_L, IDT_STAT_L_OP_MODE,
				   IDT_STAT_L_OP_MODE);
		regmap_read(w->regmap, IDT_VRECT_ADJ, &adj);
		regmap_read(w->regmap, IDT_SYS_OP_MODE, &mode);
		mode >>= 5;
		switch (mode) {
		case PAD_MODE_PMA_SR1:
		case PAD_MODE_PMA_SR1E:
			w->pad = IDT_MFC_PAD_PMA;
			break;
		case PAD_MODE_WPC_BASIC:
		case PAD_MODE_WPC_ADV:
			w->pad = IDT_MFC_PAD_WPC;
			break;
		case PAD_MODE_A4WP:
		case PAD_MODE_A4WP_LPM:
			w->pad = IDT_MFC_PAD_A4WP;	/* charged as a 5 V WPC pad */
			break;
		default:
			w->pad = IDT_MFC_PAD_NONE;	/* stock reports nothing for it */
			break;
		}
		w->online = w->pad != IDT_MFC_PAD_NONE;
		vrect = idt_adc(w, IDT_ADC_VRECT_L);
		vout = idt_adc(w, IDT_ADC_VOUT_L);
		dev_info(w->dev, "pad %s (op mode %u) vrect %d mV vout %d mV, chip id 0x%02x vrect_adj 0x%02x\n",
			 idt_pad_name[w->pad], mode, vrect, vout, id, adj);
#if IS_ENABLED(CONFIG_WIRELESS_CHARGER_IDT_MFC_NV)
		idt_rx_set(w);
		idt_fod_set(w, false);
		/* a pad already at 10 V: kept only with the charger's grant */
		if (vrect >= 8500 && vout >= 8500) {
#if IS_ENABLED(CONFIG_WIRELESS_CHARGER_IDT_MFC_HV)
			if (w->hv_grant && w->pad != IDT_MFC_PAD_A4WP) {
				w->pad_10v = true;
				w->pad = IDT_MFC_PAD_WPC_AFC;
				idt_set_vout(w, IDT_MFC_VOUT_10V);
			} else {
				idt_set_vout(w, IDT_MFC_VOUT_5V);
			}
#endif
#if !IS_ENABLED(CONFIG_WIRELESS_CHARGER_IDT_MFC_HV)
			idt_set_vout(w, IDT_MFC_VOUT_5V);
#endif
		}
		idt_rpp_set(w);
#endif
	} else if (w->wc_w_state && det == 0) {
		w->online = false;
		w->fod = false;
		w->pad = IDT_MFC_PAD_NONE;
		w->vout = IDT_MFC_VOUT_5V;
		cancel_delayed_work(&w->isr_work);
#if IS_ENABLED(CONFIG_WIRELESS_CHARGER_IDT_MFC_HV)
		w->pad_10v = false;
		w->full = false;
		cancel_delayed_work(&w->afc_vout_work);
		cancel_delayed_work(&w->vout_step_work);
#endif
		dev_info(w->dev, "pad removed\n");
#if IS_ENABLED(CONFIG_WIRELESS_CHARGER_IDT_MFC_NV)
		/* M0 restart if the coil still powers the chip without Vout */
		msleep(1000);
		if (idt_adc(w, IDT_ADC_VRECT_L) >= 3000 && idt_adc(w, IDT_ADC_VOUT_L) <= 2000) {
			dev_info(w->dev, "restart M0\n");
			idt_set_cmd_l(w, IDT_CMD_MCU_RESET, IDT_CMD_MCU_RESET);
		}
#endif
	}
	if (det >= 0)
		w->wc_w_state = det;
	mutex_unlock(&w->lock);

	power_supply_changed(w->psy);
	pm_relax(w->dev);
}

/* stock mfc_wpc_isr_work() WPC/PMA branch */
static void idt_isr_work(struct work_struct *work)
{
	struct idt_mfc_wpc *w = container_of(to_delayed_work(work), struct idt_mfc_wpc, isr_work);
	unsigned int cmd = 0, val = 0;

	mutex_lock(&w->lock);
	if (!w->wc_w_state)
		goto out;
	if (w->pad == IDT_MFC_PAD_A4WP) {		/* nothing but a log */
		regmap_read(w->regmap, IDT_BT2AP_DATA_COM, &cmd);
		regmap_read(w->regmap, IDT_BT2AP_DATA_COM + 1, &val);
		dev_info(w->dev, "a4wp data cmd 0x%02x val 0x%02x\n", cmd, val);
		goto out;
	}
	regmap_read(w->regmap, IDT_WPC_TX_DATA_COM, &cmd);
	regmap_read(w->regmap, IDT_WPC_TX_DATA_VALUE0, &val);
	dev_info(w->dev, "tx data cmd 0x%02x val 0x%02x\n", cmd, val);

	if (cmd == WPC_TX_COM_AFC_SET) {
		switch (val) {
		case TX_AFC_SET_5V:
#if IS_ENABLED(CONFIG_WIRELESS_CHARGER_IDT_MFC_HV)
			w->pad_10v = false;
#endif
			break;
		case TX_AFC_SET_10V:
#if IS_ENABLED(CONFIG_WIRELESS_CHARGER_IDT_MFC_HV)
			if (gpiod_get_value_cansleep(w->det_gpio) != 1 || idt_pad_is_hv(w->pad) ||
			    w->pad == IDT_MFC_PAD_PREPARE_HV)
				break;
			if (!w->hv_grant) {
				dev_info(w->dev, "afc set 10 V ignored: no grant\n");
				break;
			}
			/* AFC_CONF_10V: the 10 V work after 8 s */
			queue_delayed_work(system_freezable_wq, &w->afc_vout_work,
					   msecs_to_jiffies(8000));
			msleep(500);
			w->pad = IDT_MFC_PAD_PREPARE_HV;
			w->pad_10v = true;
#endif
#if !IS_ENABLED(CONFIG_WIRELESS_CHARGER_IDT_MFC_HV)
			dev_info(w->dev, "afc set 10 V ignored (5 V stage)\n");
#endif
			break;
		case TX_ID_VEHICLE_PAD:
			w->pad = IDT_MFC_PAD_WPC_VEHICLE;
			break;
		case TX_ID_BATT_PACK:
			w->pad = IDT_MFC_PAD_WPC_PACK;
			break;
		case TX_ID_BATT_PACK_TA:
			w->pad = IDT_MFC_PAD_WPC_PACK_TA;
			break;
		default:
			break;
		}
	} else if (cmd == WPC_TX_COM_TX_ID) {
		bool hv = false;

#if IS_ENABLED(CONFIG_WIRELESS_CHARGER_IDT_MFC_HV)
		hv = w->pad_10v;
#endif
		switch (val) {
		case TX_ID_VEHICLE_PAD:
			w->pad = hv ? IDT_MFC_PAD_WPC_VEHICLE_HV : IDT_MFC_PAD_WPC_VEHICLE;
			break;
		case TX_ID_STAND_TYPE_START:
			w->pad = hv ? IDT_MFC_PAD_WPC_STAND_HV : IDT_MFC_PAD_WPC_STAND;
#if IS_ENABLED(CONFIG_WIRELESS_CHARGER_IDT_MFC_NV)
			if (!hv && w->fod_ok)		/* mfc_fod_set_hero_5v() */
				idt_write_fod(w, IDT_WPC_FOD_0A, w->fod_hero_5v);
#endif
			break;
		case TX_ID_BATT_PACK:
			w->pad = IDT_MFC_PAD_WPC_PACK;
			break;
		case TX_ID_BATT_PACK_TA:
			w->pad = IDT_MFC_PAD_WPC_PACK_TA;
			break;
		default:
			break;
		}
	} else if (cmd == WPC_TX_COM_CHG_ERR) {
		if (val >= TX_CHG_ERR_FOD_FIRST && val <= TX_CHG_ERR_FOD_LAST) {
			/* unlike stock, no charging while the pad itself reports FOD */
			dev_info(w->dev, "pad reports fod (0x%02x)\n", val);
			w->fod = true;
			w->online = false;
		} else if (val == TX_CHG_ERR_OTP || val == TX_CHG_ERR_OCP ||
			   val == TX_CHG_ERR_DARKZONE) {
			dev_info(w->dev, "pad reports error 0x%02x\n", val);
		}
	}
out:
	mutex_unlock(&w->lock);
	power_supply_changed(w->psy);
}

static irqreturn_t idt_det_irq(int irq, void *data)
{
	struct idt_mfc_wpc *w = data;

	pm_stay_awake(w->dev);			/* stock wpc_wake_lock over the work */
	queue_delayed_work(system_freezable_wq, &w->det_work, 0);
	return IRQ_HANDLED;
}

/* stock mfc_wpc_irq_thread() without the MST / AC-missing routine */
static irqreturn_t idt_int_irq(int irq, void *data)
{
	struct idt_mfc_wpc *w = data;
	unsigned int l = 0, h = 0, mode;
	int ret;

	ret = regmap_read(w->regmap, IDT_INT_A_L, &l);
	ret = ret ?: regmap_read(w->regmap, IDT_INT_A_H, &h);
	if (ret) {
		idt_clear_int(w, l, h);
		return IRQ_NONE;
	}
	if ((l & IDT_STAT_L_OP_MODE) && !regmap_read(w->regmap, IDT_SYS_OP_MODE, &mode))
		dev_info(w->dev, "mode change (0x%02x)\n", mode & 0x0C);
	if (l & (IDT_STAT_L_OVER_VOL | IDT_STAT_L_OVER_CURR | IDT_STAT_L_OVER_TEMP))
		dev_info(w->dev, "abnormal stat 0x%02x\n", l);
	if ((l & IDT_STAT_L_BT2AP_DATA) && !delayed_work_pending(&w->isr_work))
		queue_delayed_work(system_freezable_wq, &w->isr_work, msecs_to_jiffies(1000));
	if (h & IDT_STAT_H_TX_DATA_RECEIVED) {
		enum idt_mfc_pad pad = READ_ONCE(w->pad);

		if (pad != IDT_MFC_PAD_WPC_STAND && pad != IDT_MFC_PAD_WPC_STAND_HV &&
		    !delayed_work_pending(&w->isr_work))
			queue_delayed_work(system_freezable_wq, &w->isr_work,
					   msecs_to_jiffies(500));
	}
	idt_clear_int(w, l, h);
	return IRQ_HANDLED;
}

/* ---- power supply ---- */

/* input current of the charger's class */
static int idt_class_ua(enum idt_mfc_pad pad)
{
	switch (pad) {
	case IDT_MFC_PAD_WPC_PACK:
		return 700000;
	case IDT_MFC_PAD_WPC_AFC:
	case IDT_MFC_PAD_WPC_PACK_TA:
	case IDT_MFC_PAD_WPC_STAND_HV:
	case IDT_MFC_PAD_WPC_VEHICLE_HV:
		return 650000;
	case IDT_MFC_PAD_PREPARE_HV:
		return 500000;
	default:
		return 900000;
	}
}

static int idt_get_property(struct power_supply *psy, enum power_supply_property psp,
			    union power_supply_propval *val)
{
	struct idt_mfc_wpc *w = power_supply_get_drvdata(psy);
	int ret = 0, mv;

	mutex_lock(&w->lock);
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = w->online;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		mv = w->wc_w_state ? idt_adc(w, IDT_ADC_VOUT_L) : 0;
		val->intval = max(mv, 0) * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = idt_vout_mv[w->vout] * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = w->online ? idt_class_ua(w->pad) : 0;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = idt_pad_name[w->pad];
		break;
	default:
		ret = -EINVAL;
		break;
	}
	mutex_unlock(&w->lock);
	return ret;
}

static enum power_supply_property idt_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_MODEL_NAME,
};

static const struct power_supply_desc idt_psy_desc = {
	.name = "wireless",
	.type = POWER_SUPPLY_TYPE_WIRELESS,
	.properties = idt_props,
	.num_properties = ARRAY_SIZE(idt_props),
	.get_property = idt_get_property,
};

/* the charger is the only consumer */
static char *idt_supplied_to[] = { "max77865-charger" };

/* ---- interface to the charger (include/linux/power/idt_mfc_wpc.h) ---- */

int idt_mfc_wpc_get_status(struct idt_mfc_wpc_status *st)
{
	struct idt_mfc_wpc *w = READ_ONCE(idt_wpc);

	if (!w)
		return -ENODEV;
	mutex_lock(&w->lock);
	st->online = w->online;
	st->fod = w->fod;
	st->pad = w->pad;
	st->vout_mv = idt_vout_mv[w->vout];
	mutex_unlock(&w->lock);
	return 0;
}
EXPORT_SYMBOL_GPL(idt_mfc_wpc_get_status);

#if IS_ENABLED(CONFIG_WIRELESS_CHARGER_IDT_MFC_NV)
void idt_mfc_wpc_rx_update(void)
{
	struct idt_mfc_wpc *w = READ_ONCE(idt_wpc);

	if (!w)
		return;
	mutex_lock(&w->lock);
	idt_rx_set(w);
	mutex_unlock(&w->lock);
}
EXPORT_SYMBOL_GPL(idt_mfc_wpc_rx_update);

void idt_mfc_wpc_set_headroom(int headroom)
{
	struct idt_mfc_wpc *w = READ_ONCE(idt_wpc);

	if (!w || headroom < 0 || headroom > 5)
		return;
	mutex_lock(&w->lock);
	if (w->online)
		idt_headroom(w, headroom);
	mutex_unlock(&w->lock);
}
EXPORT_SYMBOL_GPL(idt_mfc_wpc_set_headroom);

/* stock sec_bat_wc_cv_mode_check() requests: Vout 5.5 V, headroom 5, CV FOD */
void idt_mfc_wpc_set_cv_mode(void)
{
	struct idt_mfc_wpc *w = READ_ONCE(idt_wpc);

	if (!w)
		return;
	mutex_lock(&w->lock);
	if (w->online && w->vout == IDT_MFC_VOUT_5V) {
		idt_set_vout(w, IDT_MFC_VOUT_5_5V);
		idt_headroom(w, 5);
		idt_fod_set(w, true);			/* mfc_fod_set_cv() */
	}
	mutex_unlock(&w->lock);
}
EXPORT_SYMBOL_GPL(idt_mfc_wpc_set_cv_mode);

/* stock STATUS_FULL: CS100 (plain WPC pads with the 0x7f FOD first) */
void idt_mfc_wpc_full(void)
{
	struct idt_mfc_wpc *w = READ_ONCE(idt_wpc);
	static const u8 fod_cs100[IDT_NUM_FOD_REG] = {
		0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f };
	int i;

	if (!w)
		return;
	mutex_lock(&w->lock);
	if (!w->online)
		goto out;
	if (w->pad == IDT_MFC_PAD_WPC)			/* SEC_WIRELESS_PAD_WPC = 1 */
		idt_write_fod(w, IDT_WPC_FOD_0A, fod_cs100);	/* mfc_fod_set_cs100() */
	for (i = 0; i < IDT_CMD_CNT; i++) {		/* mfc_send_cs100() */
		if (regmap_write(w->regmap, IDT_BATTERY_CHG_STATUS, 100))
			break;
		idt_set_cmd_l(w, IDT_CMD_SEND_CHG_STS, IDT_CMD_SEND_CHG_STS);
		msleep(250);
	}
	dev_info(w->dev, "cs100\n");
#if IS_ENABLED(CONFIG_WIRELESS_CHARGER_IDT_MFC_HV)
	w->full = true;					/* HV pads step down to 5 V */
	if (idt_pad_is_hv(w->pad))
		queue_delayed_work(system_freezable_wq, &w->vout_step_work, msecs_to_jiffies(250));
#endif
out:
	mutex_unlock(&w->lock);
}
EXPORT_SYMBOL_GPL(idt_mfc_wpc_full);

/* stock mfc_send_eop() for OVERHEAT/OVERHEATLIMIT/COLD */
void idt_mfc_wpc_send_ept(void)
{
	struct idt_mfc_wpc *w = READ_ONCE(idt_wpc);
	unsigned int code;
	int i;

	if (!w)
		return;
	mutex_lock(&w->lock);
	if (!w->online || w->pad == IDT_MFC_PAD_A4WP)	/* A4WP: log only */
		goto out;
	code = w->pad == IDT_MFC_PAD_PMA ? 1 : 3;	/* EPT_END_OF_CHG / OVER_TEMP */
	for (i = 0; i < IDT_CMD_CNT; i++) {
		if (regmap_write(w->regmap, IDT_EPT, code))
			break;
		idt_set_cmd_l(w, IDT_CMD_SEND_EOP, IDT_CMD_SEND_EOP);
		msleep(250);
	}
	dev_info(w->dev, "ept %u sent\n", code);
out:
	mutex_unlock(&w->lock);
}
EXPORT_SYMBOL_GPL(idt_mfc_wpc_send_ept);
#endif

#if IS_ENABLED(CONFIG_WIRELESS_CHARGER_IDT_MFC_HV)
/*
 * The charger's 10 V grant: a grant sends MFC_REQUEST_AFC_TX
 * (header 0x28, WPC_COM_REQ_AFC_TX 0x0C, data 0x00); a withdrawal stops a pending
 * switch and steps Vout down to 5 V.
 */
void idt_mfc_wpc_hv_grant(bool grant)
{
	struct idt_mfc_wpc *w = READ_ONCE(idt_wpc);

	if (!w)
		return;
	mutex_lock(&w->lock);
	if (w->hv_grant == grant)
		goto out;
	w->hv_grant = grant;
	dev_info(w->dev, "10 V %s\n", grant ? "granted" : "withdrawn");
	if (grant) {
		if (w->online && !w->pad_10v && w->pad != IDT_MFC_PAD_A4WP)
			idt_send_packet(w, 0x28, 0x0C, 0x00);
	} else {
		cancel_delayed_work(&w->afc_vout_work);
		if (w->vout > IDT_MFC_VOUT_5V && w->vout <= IDT_MFC_VOUT_10V)
			queue_delayed_work(system_freezable_wq, &w->vout_step_work,
					   msecs_to_jiffies(250));
	}
out:
	mutex_unlock(&w->lock);
}
EXPORT_SYMBOL_GPL(idt_mfc_wpc_hv_grant);
#endif

/* ---- debugfs: read-only register dump instead of the stock raw write sysfs ---- */

static int idt_regs_show(struct seq_file *m, void *unused)
{
	struct idt_mfc_wpc *w = m->private;
	unsigned int reg, v;

	if (gpiod_get_value_cansleep(w->det_gpio) != 1) {
		seq_puts(m, "no pad (the receiver is not powered)\n");
		return 0;
	}
	for (reg = 0x00; reg <= 0x7B; reg++)
		if (!regmap_read(w->regmap, reg, &v))
			seq_printf(m, "%04x: %02x\n", reg, v);
	if (!regmap_read(w->regmap, IDT_RPP_SCALE_COEF, &v))
		seq_printf(m, "%04x: %02x\n", IDT_RPP_SCALE_COEF, v);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(idt_regs);

static void idt_debugfs_remove(void *data)
{
	debugfs_remove_recursive(data);
}

/* ---- probe ---- */

static void idt_cancel_works(void *data)
{
	struct idt_mfc_wpc *w = data;

	cancel_delayed_work_sync(&w->det_work);
	cancel_delayed_work_sync(&w->isr_work);
#if IS_ENABLED(CONFIG_WIRELESS_CHARGER_IDT_MFC_HV)
	cancel_delayed_work_sync(&w->afc_vout_work);
	cancel_delayed_work_sync(&w->vout_step_work);
#endif
}

static int idt_parse_dt(struct idt_mfc_wpc *w)
{
	struct device *dev = w->dev;

	w->fod_ok =
		!device_property_read_u8_array(dev, "samsung,fod-wpc", w->fod_wpc, IDT_NUM_FOD_REG) &&
		!device_property_read_u8_array(dev, "samsung,fod-pma", w->fod_pma, IDT_NUM_FOD_REG) &&
		!device_property_read_u8_array(dev, "samsung,fod-wpc-cv", w->fod_wpc_cv,
					       IDT_NUM_FOD_REG) &&
		!device_property_read_u8_array(dev, "samsung,fod-pma-cv", w->fod_pma_cv,
					       IDT_NUM_FOD_REG) &&
		!device_property_read_u8_array(dev, "samsung,fod-hero-5v", w->fod_hero_5v,
					       IDT_NUM_FOD_REG);
	if (!w->fod_ok)
		dev_warn(dev, "FOD tables incomplete, no FOD writes\n");
	if (device_property_read_u8(dev, "samsung,rpp-hv", &w->rpp_hv))
		w->rpp_hv = 0x40;		/* wc_hv_rpp default */
	return 0;
}

static int idt_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct power_supply_config psy_cfg = {};
	unsigned int id = 0, rev = 0, maj_l = 0, maj_h = 0, min_l = 0, min_h = 0;
	struct idt_mfc_wpc *w;
	struct dentry *d;
	int en, det, ret;

	w = devm_kzalloc(dev, sizeof(*w), GFP_KERNEL);
	if (!w)
		return -ENOMEM;
	w->dev = dev;
	mutex_init(&w->lock);
	i2c_set_clientdata(client, w);

	w->regmap = devm_regmap_init_i2c(client, &idt_regmap_config);
	if (IS_ERR(w->regmap))
		return dev_err_probe(dev, PTR_ERR(w->regmap), "regmap\n");
	idt_parse_dt(w);

	/* the RX enable as S-Boot left it, then RX output off, or per charge control with _NV */
	w->en_gpio = devm_gpiod_get(dev, "disable", GPIOD_ASIS);
	if (IS_ERR(w->en_gpio))
		return dev_err_probe(dev, PTR_ERR(w->en_gpio), "disable gpio\n");
	en = gpiod_get_raw_value_cansleep(w->en_gpio);
	ret = gpiod_direction_output(w->en_gpio, 1);
	if (ret)
		return dev_err_probe(dev, ret, "disable gpio\n");
	dev_info(dev, "wpc_en was %s at probe\n", en < 0 ? "unreadable" : en ? "high" : "low");

	w->det_gpio = devm_gpiod_get(dev, "det", GPIOD_IN);
	if (IS_ERR(w->det_gpio))
		return dev_err_probe(dev, PTR_ERR(w->det_gpio), "det gpio\n");
	w->int_gpio = devm_gpiod_get(dev, "int", GPIOD_IN);
	if (IS_ERR(w->int_gpio))
		return dev_err_probe(dev, PTR_ERR(w->int_gpio), "int gpio\n");

	/* powered from the coil only: the chip answers only on a pad */
	det = gpiod_get_value_cansleep(w->det_gpio);
	if (det == 1) {
		ret = regmap_read(w->regmap, IDT_CHIP_ID_L, &id);
		if (!ret && id == IDT_CHIP_LSI)
			return dev_err_probe(dev, -ENODEV, "chip id 0x40 (LSI) not supported\n");
		regmap_read(w->regmap, IDT_CHIP_REVISION, &rev);
		regmap_read(w->regmap, IDT_FW_MAJOR_REV_L, &maj_l);
		regmap_read(w->regmap, IDT_FW_MAJOR_REV_L + 1, &maj_h);
		regmap_read(w->regmap, IDT_FW_MINOR_REV_L, &min_l);
		regmap_read(w->regmap, IDT_FW_MINOR_REV_L + 1, &min_h);
		dev_info(dev, "chip id 0x%02x rev 0x%02x fw 0x%04x/0x%04x, wpc_en high (rx off)\n",
			 id, rev, maj_l | maj_h << 8, min_l | min_h << 8);
	} else {
		dev_info(dev, "no pad, chip not read, wpc_en high (rx off)\n");
	}

	INIT_DELAYED_WORK(&w->det_work, idt_det_work);
	INIT_DELAYED_WORK(&w->isr_work, idt_isr_work);
#if IS_ENABLED(CONFIG_WIRELESS_CHARGER_IDT_MFC_HV)
	INIT_DELAYED_WORK(&w->afc_vout_work, idt_afc_vout_work);
	INIT_DELAYED_WORK(&w->vout_step_work, idt_vout_step_work);
#endif
	/* before the IRQs: cancelled only after both IRQs are freed */
	ret = devm_add_action_or_reset(dev, idt_cancel_works, w);
	if (ret)
		return ret;
	ret = devm_device_init_wakeup(dev);
	if (ret)
		return ret;

	psy_cfg.drv_data = w;
	psy_cfg.supplied_to = idt_supplied_to;
	psy_cfg.num_supplicants = ARRAY_SIZE(idt_supplied_to);
	w->psy = devm_power_supply_register(dev, &idt_psy_desc, &psy_cfg);
	if (IS_ERR(w->psy))
		return dev_err_probe(dev, PTR_ERR(w->psy), "wireless supply\n");

	/* wpc_det both edges, wpc_int falling, from the DT */
	w->irq_det = fwnode_irq_get_byname(dev_fwnode(dev), "wpc-det");
	if (w->irq_det < 0)
		return dev_err_probe(dev, w->irq_det, "wpc-det irq\n");
	w->irq_int = fwnode_irq_get_byname(dev_fwnode(dev), "wpc-int");
	if (w->irq_int < 0)
		return dev_err_probe(dev, w->irq_int, "wpc-int irq\n");
	ret = devm_request_threaded_irq(dev, w->irq_det, NULL, idt_det_irq, IRQF_ONESHOT,
					"wpc-det", w);
	if (ret)
		return dev_err_probe(dev, ret, "wpc-det irq\n");
	ret = devm_request_threaded_irq(dev, w->irq_int, NULL, idt_int_irq, IRQF_ONESHOT,
					"wpc-int", w);
	if (ret)
		return dev_err_probe(dev, ret, "wpc-int irq\n");

	/* already on a pad: clear what is pending, as stock, then detect */
	if (det == 1) {
		unsigned int l = 0, h = 0;

		regmap_read(w->regmap, IDT_INT_A_L, &l);
		regmap_read(w->regmap, IDT_INT_A_H, &h);
		idt_clear_int(w, l, h);
		pm_stay_awake(dev);
		queue_delayed_work(system_freezable_wq, &w->det_work, 0);
	}

	d = debugfs_create_dir("idt-mfc-wpc", NULL);
	debugfs_create_file("regs", 0400, d, w, &idt_regs_fops);
	ret = devm_add_action_or_reset(dev, idt_debugfs_remove, d);
	if (ret)
		return ret;

	WRITE_ONCE(idt_wpc, w);
#if IS_ENABLED(CONFIG_WIRELESS_CHARGER_IDT_MFC_NV)
	idt_mfc_wpc_rx_update();		/* the charger may have probed first */
#endif
	return 0;
}

/* stock mfc_charger_shutdown(), only with _NV (the read-only level writes no command) */
static void idt_shutdown(struct i2c_client *client)
{
#if IS_ENABLED(CONFIG_WIRELESS_CHARGER_IDT_MFC_NV)
	struct idt_mfc_wpc *w = i2c_get_clientdata(client);

	mutex_lock(&w->lock);
	if (w->online) {
		idt_headroom(w, 1);
		if (w->vout == IDT_MFC_VOUT_5_5V)
			idt_set_vout(w, IDT_MFC_VOUT_5V);
	}
	mutex_unlock(&w->lock);
#endif
}

/* stock suspend/resume: wake arming only; 6.18 disables non-wake IRQs itself */
static int idt_suspend(struct device *dev)
{
	struct idt_mfc_wpc *w = dev_get_drvdata(dev);

	if (device_may_wakeup(dev)) {
		enable_irq_wake(w->irq_int);
		enable_irq_wake(w->irq_det);
	}
	return 0;
}

static int idt_resume(struct device *dev)
{
	struct idt_mfc_wpc *w = dev_get_drvdata(dev);

	if (device_may_wakeup(dev)) {
		disable_irq_wake(w->irq_int);
		disable_irq_wake(w->irq_det);
	}
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(idt_pm_ops, idt_suspend, idt_resume);

static const struct of_device_id idt_of_match[] = {
	{ .compatible = "idt,mfc-charger" },	/* stock compatible */
	{ }
};
MODULE_DEVICE_TABLE(of, idt_of_match);

static struct i2c_driver idt_driver = {
	.driver = {
		.name = "idt-mfc-wpc",
		.of_match_table = idt_of_match,
		.pm = pm_sleep_ptr(&idt_pm_ops),
		.suppress_bind_attrs = true,
	},
	.probe = idt_probe,
	.shutdown = idt_shutdown,
};
module_i2c_driver(idt_driver);

MODULE_DESCRIPTION("IDT wireless charging receiver (greatlte)");
MODULE_LICENSE("GPL");
