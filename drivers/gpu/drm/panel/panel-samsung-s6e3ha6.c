// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung S6E3HA6 AMOLED panel of the Galaxy Note 8: MIPI DSI command mode,
 * four lanes, VESA DSC with two 720x40 slices.
 *
 * The command sequences are those of the vendor kernel. Prepare only switches
 * the rails and the reset line; every command is sent in enable, after the DSI
 * link is up. If the boot loader already initialised the panel (reset and panel
 * detect high), the first enable leaves it as it is.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/string.h>
#include <linux/workqueue.h>

#include <drm/display/drm_dsc.h>
#include <drm/drm_bridge.h>
#include <drm/drm_bridge_helper.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_modeset_lock.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>
#include <drm/exynos8895_panel.h>

/* the vendor panel framework */
#include "s6e3ha6-stock/panel_dpu.h"
/* the AID dimming engine of the fixed sequences */
#include "panel-samsung-s6e3ha6-bright.h"

#define S6E3HA6_INIT_TRIES	3	/* the vendor retries up to 30 times */

/* regulators in the vendor enable order */
enum { S6E3HA6_VDDI3P0, S6E3HA6_VDDI1P8, S6E3HA6_VDDI1P6, S6E3HA6_NR_SUPPLIES };
static const char * const s6e3ha6_supply_names[S6E3HA6_NR_SUPPLIES] = {
	"vddi3p0", "vddi1p8", "vddi1p6",
};

struct s6e3ha6 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct drm_dsc_config dsc;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *det_gpio;
	struct regulator *supplies[S6E3HA6_NR_SUPPLIES];
	/*
	 * The AID dimming engine with its backlight and lcd class devices,
	 * for the fixed sequences below. NULL with the vendor framework, which
	 * registers devices of the same name, "panel", itself.
	 */
	struct s6e3ha6_bright *bright;
	bool booted;		/* the boot loader left the panel on and initialised */
	bool powered;		/* rails on, at probe or by prepare */
	bool on;		/* initialised */

	/* ESD: disp-det falling edge */
	int det_irq;
	bool det_irq_on;
	struct workqueue_struct *det_wq;
	struct work_struct det_work;
	u32 lcdtype;
	const char *variant;
	struct dsu_info_dt dsu_dt;
	unsigned int dsu_mode;

	/*
	 * The vendor panel framework, called as the vendor DSIM driver called
	 * it. It is probed at the first enable, when the link is up; if that
	 * fails the fixed sequences below drive the panel.
	 */
	struct panel_device *stock;
	bool stock_probed;
	bool stock_failed;
	struct exynos8895_panel_hooks decon_hooks;
	/*
	 * S3CFB DOZE and DOZE_SUSPEND, under stock_lock: DOZE with the display
	 * off makes the next enable a PANEL_IOC_DOZE; DOZE_SUSPEND of an active
	 * display makes the next disable and unprepare leave the panel powered
	 * in its low-power mode.
	 */
	struct mutex stock_lock;
	enum { S6E3HA6_DOZE_NONE, S6E3HA6_DOZE_ENTER, S6E3HA6_DOZE_SUSPEND } doze;
};

static inline struct s6e3ha6 *to_s6e3ha6(struct drm_panel *panel)
{
	return container_of(panel, struct s6e3ha6, panel);
}

static inline struct panel_device *s6e3ha6_stock(struct s6e3ha6 *ctx)
{
	if (!IS_ENABLED(CONFIG_DRM_PANEL_SAMSUNG_S6E3HA6_STOCK) || ctx->stock_failed)
		return NULL;
	return ctx->stock;
}

/* per DSU mode PPS, scaler, CASET and PASET */
static const u8 s6e3ha6_pps_table[MAX_DSU_RES_NUMBER][128] = {
	{
		/* DSU_MODE_1: 1440x2960, slice 720x40 */
		0x11, 0x00, 0x00, 0x89, 0x30, 0x80, 0x0B, 0x90,
		0x05, 0xA0, 0x00, 0x28, 0x02, 0xD0, 0x02, 0xD0,
		0x02, 0x00, 0x02, 0x68, 0x00, 0x20, 0x04, 0x6C,
		0x00, 0x0A, 0x00, 0x0C, 0x02, 0x77, 0x01, 0xE9,
		0x18, 0x00, 0x10, 0xF0, 0x03, 0x0C, 0x20, 0x00,
		0x06, 0x0B, 0x0B, 0x33, 0x0E, 0x1C, 0x2A, 0x38,
		0x46, 0x54, 0x62, 0x69, 0x70, 0x77, 0x79, 0x7B,
		0x7D, 0x7E, 0x01, 0x02, 0x01, 0x00, 0x09, 0x40,
		0x09, 0xBE, 0x19, 0xFC, 0x19, 0xFA, 0x19, 0xF8,
		0x1A, 0x38, 0x1A, 0x78, 0x1A, 0xB6, 0x2A, 0xF6,
		0x2B, 0x34, 0x2B, 0x74, 0x3B, 0x74, 0x6B, 0xF4,
	},
	{
		/* DSU_MODE_2: 1080x2220, slice 540x30 */
		0x11, 0x00, 0x00, 0x89, 0x30, 0x80, 0x08, 0xAC,
		0x04, 0x38, 0x00, 0x1E, 0x02, 0x1C, 0x02, 0x1C,
		0x02, 0x00, 0x02, 0x0E, 0x00, 0x20, 0x02, 0xE3,
		0x00, 0x07, 0x00, 0x0C, 0x03, 0x50, 0x03, 0x64,
		0x18, 0x00, 0x10, 0xF0, 0x03, 0x0C, 0x20, 0x00,
		0x06, 0x0B, 0x0B, 0x33, 0x0E, 0x1C, 0x2A, 0x38,
		0x46, 0x54, 0x62, 0x69, 0x70, 0x77, 0x79, 0x7B,
		0x7D, 0x7E, 0x01, 0x02, 0x01, 0x00, 0x09, 0x40,
		0x09, 0xBE, 0x19, 0xFC, 0x19, 0xFA, 0x19, 0xF8,
		0x1A, 0x38, 0x1A, 0x78, 0x1A, 0xB6, 0x2A, 0xF6,
		0x2B, 0x34, 0x2B, 0x74, 0x3B, 0x74, 0x6B, 0xF4,
	},
	{
		/* DSU_MODE_3: 720x1048, slice 360x74 */
		0x11, 0x00, 0x00, 0x89, 0x30, 0x80, 0x05, 0xC8,
		0x02, 0xD0, 0x00, 0x4A, 0x01, 0x68, 0x01, 0x68,
		0x02, 0x00, 0x01, 0xB4, 0x00, 0x20, 0x05, 0xBA,
		0x00, 0x05, 0x00, 0x0C, 0x01, 0x51, 0x02, 0x10,
		0x18, 0x00, 0x10, 0xF0, 0x03, 0x0C, 0x20, 0x00,
		0x06, 0x0B, 0x0B, 0x33, 0x0E, 0x1C, 0x2A, 0x38,
		0x46, 0x54, 0x62, 0x69, 0x70, 0x77, 0x79, 0x7B,
		0x7D, 0x7E, 0x01, 0x02, 0x01, 0x00, 0x09, 0x40,
		0x09, 0xBE, 0x19, 0xFC, 0x19, 0xFA, 0x19, 0xF8,
		0x1A, 0x38, 0x1A, 0x78, 0x1A, 0xB6, 0x2A, 0xF6,
		0x2B, 0x34, 0x2B, 0x74, 0x3B, 0x74, 0x6B, 0xF4,
	},
};

static const u8 s6e3ha6_scaler_table[MAX_DSU_RES_NUMBER][7] = {
	{ 0x01, 0x02, 0x06, 0x08, 0x08, 0x1E, 0x03 },	/* 1440x2960, scaler off */
	{ 0x02, 0x02, 0x06, 0x08, 0x08, 0x1E, 0x03 },	/* 1080x2220, 1.78x scaler */
	{ 0x00, 0x02, 0x06, 0x08, 0x08, 0x1E, 0x03 },	/* 720x1048, 4x scaler */
};

static const u8 s6e3ha6_caset_table[MAX_DSU_RES_NUMBER][4] = {
	{ 0x00, 0x00, 0x05, 0x9F },	/* 0..1439 */
	{ 0x00, 0x00, 0x04, 0x37 },	/* 0..1079 */
	{ 0x00, 0x00, 0x02, 0xCF },	/* 0..719 */
};

static const u8 s6e3ha6_paset_table[MAX_DSU_RES_NUMBER][4] = {
	{ 0x00, 0x00, 0x0B, 0x8F },	/* 0..2959 */
	{ 0x00, 0x00, 0x08, 0xAB },	/* 0..2219 */
	{ 0x00, 0x00, 0x05, 0xC7 },	/* 0..1479 */
};

/* test keys */
#define s6e3ha6_key1_on(ctx)	mipi_dsi_dcs_write_seq_multi(ctx, 0x9f, 0xa5, 0xa5)
#define s6e3ha6_key1_off(ctx)	mipi_dsi_dcs_write_seq_multi(ctx, 0x9f, 0x5a, 0x5a)
#define s6e3ha6_key2_on(ctx)	mipi_dsi_dcs_write_seq_multi(ctx, 0xf0, 0x5a, 0x5a)
#define s6e3ha6_key2_off(ctx)	mipi_dsi_dcs_write_seq_multi(ctx, 0xf0, 0xa5, 0xa5)
#define s6e3ha6_key3_on(ctx)	mipi_dsi_dcs_write_seq_multi(ctx, 0xfc, 0x5a, 0x5a)
#define s6e3ha6_key3_off(ctx)	mipi_dsi_dcs_write_seq_multi(ctx, 0xfc, 0xa5, 0xa5)

/*
 * The vendor init sequence without its brightness part, which is sent
 * separately, the self diagnosis read and COPR.
 */
static void s6e3ha6_init_seq(struct mipi_dsi_multi_context *ctx,
			     const struct s6e3ha6 *panel_ctx)
{
	unsigned int idx = (panel_ctx->dsu_mode >= DSU_MODE_1 &&
			    panel_ctx->dsu_mode < DSU_MODE_MAX) ?
			   (panel_ctx->dsu_mode - DSU_MODE_1) : 0;
	bool dsc_en = panel_ctx->dsu_dt.dsu_en ?
		      (panel_ctx->dsu_dt.res_info[idx].dsc_en != 0) : true;
	const u8 *caset = s6e3ha6_caset_table[idx];
	const u8 *paset = s6e3ha6_paset_table[idx];
	const u8 *sc = s6e3ha6_scaler_table[idx];

	mipi_dsi_msleep(ctx, 5);
	s6e3ha6_key1_on(ctx);
	mipi_dsi_compression_mode_multi(ctx, dsc_en);
	mipi_dsi_picture_parameter_set_multi(ctx,
					     (const struct drm_dsc_picture_parameter_set *)
					     s6e3ha6_pps_table[idx]);
	s6e3ha6_key1_off(ctx);
	mipi_dsi_dcs_exit_sleep_mode_multi(ctx);
	mipi_dsi_msleep(ctx, 10);

	/* scaler, CASET and PASET of the active DSU mode, then OMOK */
	s6e3ha6_key2_on(ctx);
	mipi_dsi_dcs_write_var_seq_multi(ctx, 0xba,
					 sc[0], sc[1], sc[2], sc[3], sc[4], sc[5], sc[6]);
	s6e3ha6_key2_off(ctx);
	mipi_dsi_dcs_write_var_seq_multi(ctx, 0x2a, caset[0], caset[1], caset[2], caset[3]);
	mipi_dsi_dcs_write_var_seq_multi(ctx, 0x2b, paset[0], paset[1], paset[2], paset[3]);
	s6e3ha6_key3_on(ctx);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb0, 0x05);		/* OMOK 2CH latch */
	mipi_dsi_dcs_write_seq_multi(ctx, 0xff, 0x01);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb0, 0x07);		/* OMOK zero black */
	mipi_dsi_dcs_write_seq_multi(ctx, 0xf2, 0x3c, 0xd0);
	s6e3ha6_key3_off(ctx);

	s6e3ha6_key1_on(ctx);
	mipi_dsi_dcs_write_seq_multi(ctx, 0x1a, 0x1f, 0x00, 0x00, 0x00, 0x00);	/* SET_AREA */
	mipi_dsi_dcs_write_seq_multi(ctx, 0x35, 0x00);	/* TE_ON, a long packet on the host */
	s6e3ha6_key1_off(ctx);
	s6e3ha6_key2_on(ctx);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xed, 0x4c);	/* ERR_FG */
	mipi_dsi_dcs_write_seq_multi(ctx, 0xc5, 0x0d, 0x08, 0x40, 0x19, 0xb7, 0x07, 0x1f,
				     0x81, 0x25, 0xce, 0x79, 0x77, 0x73, 0x00, 0xff, 0x78,
				     0x8f, 0x00, 0x00);	/* FFC */
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb9, 0x01, 0x0b, 0x91, 0x00, 0x09);	/* TSP_HSYNC */
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb0, 0x03);	/* ISC */
	mipi_dsi_dcs_write_seq_multi(ctx, 0xf6, 0x53);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb0, 0x08);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xf6, 0x80);
	s6e3ha6_key2_off(ctx);
}

/* the DSU mode switch of the fixed sequences */
static int s6e3ha6_dsu_seq(struct s6e3ha6 *ctx, unsigned int mode)
{
	struct mipi_dsi_multi_context dctx = { .dsi = ctx->dsi };
	unsigned int idx = (mode >= DSU_MODE_1 && mode < DSU_MODE_MAX) ?
			   (mode - DSU_MODE_1) : 0;
	bool dsc_en = ctx->dsu_dt.res_info[idx].dsc_en != 0;
	const u8 *caset = s6e3ha6_caset_table[idx];
	const u8 *paset = s6e3ha6_paset_table[idx];
	const u8 *sc = s6e3ha6_scaler_table[idx];

	s6e3ha6_key1_on(&dctx);
	mipi_dsi_compression_mode_multi(&dctx, dsc_en);
	mipi_dsi_picture_parameter_set_multi(&dctx,
					     (const struct drm_dsc_picture_parameter_set *)
					     s6e3ha6_pps_table[idx]);
	s6e3ha6_key1_off(&dctx);
	mipi_dsi_dcs_write_var_seq_multi(&dctx, 0x2a, caset[0], caset[1], caset[2], caset[3]);
	mipi_dsi_dcs_write_var_seq_multi(&dctx, 0x2b, paset[0], paset[1], paset[2], paset[3]);
	s6e3ha6_key2_on(&dctx);
	mipi_dsi_dcs_write_var_seq_multi(&dctx, 0xba,
					 sc[0], sc[1], sc[2], sc[3], sc[4], sc[5], sc[6]);
	s6e3ha6_key2_off(&dctx);

	return dctx.accum_err;
}

static int s6e3ha6_power_on(struct s6e3ha6 *ctx)
{
	int i, ret;

	for (i = 0; i < S6E3HA6_NR_SUPPLIES; i++) {
		ret = regulator_enable(ctx->supplies[i]);
		if (ret) {
			dev_err(&ctx->dsi->dev, "cannot enable %s: %d\n",
				s6e3ha6_supply_names[i], ret);
			while (--i >= 0)
				regulator_disable(ctx->supplies[i]);
			return ret;
		}
	}
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);		/* deasserted: line high */
	usleep_range(5000, 6000);
	ctx->powered = true;

	return 0;
}

static void s6e3ha6_power_off(struct s6e3ha6 *ctx)
{
	int i;

	if (!ctx->powered)
		return;
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	for (i = S6E3HA6_NR_SUPPLIES - 1; i >= 0; i--)
		regulator_disable(ctx->supplies[i]);
	ctx->powered = false;
}

static int s6e3ha6_stock_set_power(struct s6e3ha6 *ctx, int power);

static int s6e3ha6_prepare(struct drm_panel *panel)
{
	struct s6e3ha6 *ctx = to_s6e3ha6(panel);

	/* power before the link */
	if (s6e3ha6_stock(ctx))
		return s6e3ha6_stock_set_power(ctx, 1);

	/* the boot loader's rails are held by the references of probe */
	if (ctx->booted || ctx->powered)
		return 0;

	return s6e3ha6_power_on(ctx);
}

/* poll the panel detect line for 20 ms */
static bool s6e3ha6_det_ok(struct s6e3ha6 *ctx)
{
	int i;

	if (!ctx->det_gpio)
		return true;
	for (i = 0; i <= 200; i++) {
		if (gpiod_get_value_cansleep(ctx->det_gpio))
			return true;
		usleep_range(100, 110);
	}
	return false;
}

/*
 * The disp-det interrupt is on while the panel is initialised: enabled after
 * a good init, disabled before the exit sequence. The flag keeps the depth
 * balanced when an enable fails.
 */
static void s6e3ha6_det_irq(struct s6e3ha6 *ctx, bool on)
{
	if (ctx->det_irq <= 0 || ctx->det_irq_on == on)
		return;
	if (on)
		enable_irq(ctx->det_irq);
	else
		disable_irq(ctx->det_irq);
	ctx->det_irq_on = on;
}

/*
 * The link state the framework's command path checks, switched under the
 * lock its sequences run under.
 */
static void s6e3ha6_stock_link(struct panel_device *stock, bool on)
{
	mutex_lock(&stock->op_lock);
	stock->link_on = on;
	mutex_unlock(&stock->op_lock);
}

/* the framework's disp-det interrupt control */
static void s6e3ha6_stock_det_irq(struct panel_device *stock, bool on)
{
	struct s6e3ha6 *ctx = dev_get_drvdata(stock->dev);

	/* on after a good init, off before the exit sequence: the panel is initialised */
	ctx->on = on;
	s6e3ha6_det_irq(ctx, on);
}

static int s6e3ha6_stock_set_power(struct s6e3ha6 *ctx, int power)
{
	struct panel_device *stock = ctx->stock;
	int ret;

	ret = panel_drv_ioctl(stock, PANEL_IOC_SET_POWER, &power);

	return ret;
}

/* the DECON's frame done interrupt */
static void s6e3ha6_stock_frame_done(void *data)
{
	struct s6e3ha6 *ctx = data;

	panel_drv_frame_done(ctx->stock);
}

/* the DECON's HMD check */
static bool s6e3ha6_stock_hmd_on(void *data)
{
	struct s6e3ha6 *ctx = data;

	return READ_ONCE(ctx->stock->state.hmd_on) == PANEL_HMD_ON;
}

/* a DSU mode switch from the DECON */
static int s6e3ha6_set_dsu(void *data, const struct dsu_info *dsu)
{
	struct s6e3ha6 *ctx = data;
	struct panel_device *stock = s6e3ha6_stock(ctx);

	if (!dsu || !ctx->dsu_dt.dsu_en)
		return -EINVAL;
	if (dsu->mode < DSU_MODE_1 || dsu->mode >= DSU_MODE_MAX)
		return -EINVAL;

	ctx->dsu_mode = dsu->mode;
	if (stock && ctx->stock_probed)
		return panel_drv_ioctl(stock, PANEL_IOC_SET_DSU, (void *)dsu);

	if (stock) {
		unsigned int idx = dsu->mode - DSU_MODE_1;

		stock->lcd_info.xres = ctx->dsu_dt.res_info[idx].width;
		stock->lcd_info.yres = ctx->dsu_dt.res_info[idx].height;
		stock->lcd_info.dsu_mode = dsu->mode;
		stock->lcd_info.dsc_enabled = ctx->dsu_dt.res_info[idx].dsc_en;
		stock->lcd_info.dsc_slice_h = ctx->dsu_dt.res_info[idx].dsc_height;
	}
	if (!ctx->on)
		return 0;
	return s6e3ha6_dsu_seq(ctx, dsu->mode);
}

/*
 * The S3CFB power requests, passed on as the vendor DSIM driver did. A DOZE
 * with the display off is left to the next enable, a DOZE_SUSPEND to the
 * next disable; any other request cancels a pending one.
 */
static int s6e3ha6_stock_power(void *data, enum exynos8895_panel_power req, bool active)
{
	struct s6e3ha6 *ctx = data;
	struct panel_device *stock = s6e3ha6_stock(ctx);
	int ret = 0;

	if (!stock)
		return 0;

	mutex_lock(&ctx->stock_lock);
	ctx->doze = S6E3HA6_DOZE_NONE;
	switch (req) {
	case EXYNOS8895_PANEL_UNBLANK:
		if (active) {
			ret = panel_drv_ioctl(stock, PANEL_IOC_SLEEP_OUT, NULL);
			mdnie_unblank(&stock->mdnie);
		}
		break;
	case EXYNOS8895_PANEL_DOZE:
		if (active)
			ret = panel_drv_ioctl(stock, PANEL_IOC_DOZE, NULL);
		else
			ctx->doze = S6E3HA6_DOZE_ENTER;
		break;
	case EXYNOS8895_PANEL_DOZE_SUSPEND:
		if (active)
			ctx->doze = S6E3HA6_DOZE_SUSPEND;
		break;
	case EXYNOS8895_PANEL_POWERDOWN:
		break;
	}
	mutex_unlock(&ctx->stock_lock);

	return ret;
}

/*
 * The framework probe, which needs the link: panel id, resources, and the
 * lcd, backlight, mdnie, copr, active clock and poc devices. Run at the
 * first enable. On failure the fixed sequences take over, from the power
 * state the framework left.
 */
static bool s6e3ha6_stock_probe(struct s6e3ha6 *ctx)
{
	struct panel_device *stock = ctx->stock;
	int ret;

	if (ctx->stock_probed)
		return true;

	s6e3ha6_stock_link(stock, true);
	ret = panel_drv_ioctl(stock, PANEL_IOC_PANEL_PROBE, NULL);
	if (!ret) {
		ctx->stock_probed = true;
		/* the DECON's panel events, once the framework's COPR exists */
		ctx->decon_hooks.frame_done = s6e3ha6_stock_frame_done;
		ctx->decon_hooks.power = s6e3ha6_stock_power;
		ctx->decon_hooks.hmd_on = s6e3ha6_stock_hmd_on;
		ctx->decon_hooks.data = ctx;
		exynos8895_decon_set_panel_hooks(&ctx->decon_hooks);
		return true;
	}

	dev_err(&ctx->dsi->dev, "panel framework probe failed (%d), using the fixed sequences\n",
		ret);
	s6e3ha6_stock_link(stock, false);
	ctx->stock_failed = true;
	ctx->powered = stock->state.power == PANEL_POWER_ON;
	return false;
}

/*
 * SLEEP_OUT with the link up, DISP_ON while the panel's display is off, and
 * the mDNIe unblank.
 */
static int s6e3ha6_stock_enable(struct s6e3ha6 *ctx)
{
	struct panel_device *stock = ctx->stock;
	int disp_on = 1;
	bool doze;
	int ret;

	mutex_lock(&ctx->stock_lock);
	doze = ctx->doze == S6E3HA6_DOZE_ENTER;
	ctx->doze = S6E3HA6_DOZE_NONE;
	mutex_unlock(&ctx->stock_lock);

	s6e3ha6_stock_link(stock, true);
	/* a DOZE requested while off */
	ret = panel_drv_ioctl(stock, doze ? PANEL_IOC_DOZE : PANEL_IOC_SLEEP_OUT, NULL);
	if (ret)
		return ret;

	if (stock->state.disp_on == PANEL_DISPLAY_OFF) {
		ret = panel_drv_ioctl(stock, PANEL_IOC_DISP_ON, &disp_on);
		if (ret)
			return ret;
	}

	/* the fb0 unblank of the vendor mDNIe notifier; DOZE is no unblank */
	if (!doze)
		mdnie_unblank(&stock->mdnie);

	return 0;
}

static int s6e3ha6_enable(struct drm_panel *panel)
{
	struct s6e3ha6 *ctx = to_s6e3ha6(panel);
	struct mipi_dsi_multi_context dctx = { .dsi = ctx->dsi };
	int try, ret;

	if (s6e3ha6_stock(ctx) && s6e3ha6_stock_probe(ctx))
		return s6e3ha6_stock_enable(ctx);

	/*
	 * The MTP and HBM gamma registers the AID dimming table is built from
	 * are only reachable over the DSI link, which is up here.
	 */
	if (ctx->bright) {
		ret = s6e3ha6_bright_prepare(ctx->bright);
		if (ret)
			dev_err(&ctx->dsi->dev,
				"no AID gamma table: %d, brightness is not sent\n",
				ret);
	}

	/* the first unblank after boot does not reach the panel */
	if (ctx->booted)
		return 0;
	ctx->on = false;

	for (try = 1; try <= S6E3HA6_INIT_TRIES; try++) {
		dctx.accum_err = 0;
		s6e3ha6_init_seq(&dctx, ctx);
		if (!dctx.accum_err && s6e3ha6_det_ok(ctx))
			break;
		dev_err(&ctx->dsi->dev, "disp-det low after init (try %d/%d), err %d\n",
			try, S6E3HA6_INIT_TRIES, dctx.accum_err);
		if (try == S6E3HA6_INIT_TRIES)
			return dctx.accum_err ?: -EIO;
		/* power cycle with the vendor retry gap */
		s6e3ha6_power_off(ctx);
		msleep(100);
		ret = s6e3ha6_power_on(ctx);
		if (ret)
			return ret;
	}

	ctx->on = true;
	s6e3ha6_det_irq(ctx, true);

	/* the brightness part of the init sequence */
	if (ctx->bright) {
		ret = s6e3ha6_bright_enable(ctx->bright);
		if (ret)
			dev_warn(&ctx->dsi->dev, "no AID brightness: %d\n", ret);
	}

	dctx.accum_err = 0;
	s6e3ha6_key1_on(&dctx);
	mipi_dsi_dcs_set_display_on_multi(&dctx);
	s6e3ha6_key1_off(&dctx);

	return dctx.accum_err;
}

static int s6e3ha6_disable(struct drm_panel *panel)
{
	struct s6e3ha6 *ctx = to_s6e3ha6(panel);
	struct mipi_dsi_multi_context dctx = { .dsi = ctx->dsi };
	struct panel_device *stock = s6e3ha6_stock(ctx);
	int ret;

	/*
	 * SLEEP_IN, then the link off; after DOZE_SUSPEND, PANEL_IOC_DOZE and
	 * the link off, the panel staying in ALPM.
	 */
	if (stock) {
		bool suspend;

		mutex_lock(&ctx->stock_lock);
		suspend = ctx->doze == S6E3HA6_DOZE_SUSPEND;
		mutex_unlock(&ctx->stock_lock);

		ret = panel_drv_ioctl(stock, suspend ? PANEL_IOC_DOZE : PANEL_IOC_SLEEP_IN, NULL);
		s6e3ha6_stock_link(stock, false);
		if (!suspend)
			ctx->on = false;
		return ret;
	}

	/* disp-det off first */
	s6e3ha6_det_irq(ctx, false);
	ctx->on = false;

	s6e3ha6_key1_on(&dctx);
	mipi_dsi_dcs_set_display_off_multi(&dctx);
	mipi_dsi_dcs_write_seq_multi(&dctx, 0x53, 0x02);	/* DISPLAY_OFF_HLPM_ON */
	mipi_dsi_usleep_range(&dctx, 16700, 17700);		/* one frame at 60 Hz */
	s6e3ha6_key1_off(&dctx);
	s6e3ha6_key1_on(&dctx);
	mipi_dsi_dcs_enter_sleep_mode_multi(&dctx);
	s6e3ha6_key1_off(&dctx);
	mipi_dsi_msleep(&dctx, 105);

	ctx->booted = false;

	return dctx.accum_err;
}

static int s6e3ha6_unprepare(struct drm_panel *panel)
{
	struct s6e3ha6 *ctx = to_s6e3ha6(panel);

	/* the framework's first power off drops the boot references of probe */
	if (s6e3ha6_stock(ctx)) {
		bool suspend;

		/* DOZE_SUSPEND leaves the panel powered */
		mutex_lock(&ctx->stock_lock);
		suspend = ctx->doze == S6E3HA6_DOZE_SUSPEND;
		ctx->doze = S6E3HA6_DOZE_NONE;
		mutex_unlock(&ctx->stock_lock);
		if (suspend)
			return 0;
		return s6e3ha6_stock_set_power(ctx, 0);
	}

	/* the first unprepare after boot drops the boot references of probe */
	ctx->booted = false;
	s6e3ha6_power_off(ctx);

	return 0;
}

/*
 * The vendor DECON switches the link and the panel off and on again and
 * sends the last frame again. Here the same is a full modeset of the CRTC
 * this panel is on. An inactive CRTC is left alone.
 */
static int s6e3ha6_reset_pipe(struct s6e3ha6 *ctx)
{
	struct drm_bridge *bridge = of_drm_find_bridge(ctx->dsi->dev.of_node);
	struct drm_modeset_acquire_ctx actx;
	int ret;

	if (!bridge || !bridge->encoder)
		return -ENODEV;

	drm_modeset_acquire_init(&actx, 0);
retry:
	ret = drm_bridge_helper_reset_crtc(bridge, &actx);
	if (ret == -EDEADLK) {
		drm_modeset_backoff(&actx);
		goto retry;
	}
	drm_modeset_drop_locks(&actx);
	drm_modeset_acquire_fini(&actx);

	return ret;
}

/*
 * disp-det low while the panel is on is an ESD hit: interrupt off, 200 ms
 * debounce, reset the display pipe, interrupt on again. The vendor driver
 * cleared the pending EINT, a register pinctrl owns, so a stale edge is
 * ignored here.
 */
static void s6e3ha6_det_work(struct work_struct *work)
{
	struct s6e3ha6 *ctx = container_of(work, struct s6e3ha6, det_work);
	int det = gpiod_get_value_cansleep(ctx->det_gpio);
	int ret;

	dev_info(&ctx->dsi->dev, "disp_det: %d\n", det);
	if (!ctx->on || det != 0)
		return;

	disable_irq(ctx->det_irq);
	usleep_range(200000, 210000);
	dev_err(&ctx->dsi->dev, "disp_det is abnormal state, resetting the display\n");
	ret = s6e3ha6_reset_pipe(ctx);
	if (ret)
		dev_err(&ctx->dsi->dev, "failed to reset panel: %d\n", ret);
	if (s6e3ha6_stock(ctx) && ctx->stock_probed)
		backlight_update_status(ctx->stock->panel_bl.bd);
	enable_irq(ctx->det_irq);
}

static irqreturn_t s6e3ha6_det_irq_handler(int irq, void *data)
{
	struct s6e3ha6 *ctx = data;

	queue_work(ctx->det_wq, &ctx->det_work);
	return IRQ_HANDLED;
}

/* 1440x2960 at 60 Hz; the porches are informational in command mode */
static const struct drm_display_mode s6e3ha6_mode = {
	.clock = (1440 + 2 + 2 + 2) * (2960 + 3 + 1 + 15) * 60 / 1000,
	.hdisplay = 1440,
	.hsync_start = 1440 + 2,
	.hsync_end = 1440 + 2 + 2,
	.htotal = 1440 + 2 + 2 + 2,
	.vdisplay = 2960,
	.vsync_start = 2960 + 3,
	.vsync_end = 2960 + 3 + 1,
	.vtotal = 2960 + 3 + 1 + 15,
	.width_mm = 71,
	.height_mm = 145,
	.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
};

static int s6e3ha6_get_modes(struct drm_panel *panel, struct drm_connector *connector)
{
	return drm_connector_helper_get_modes_fixed(connector, &s6e3ha6_mode);
}

static const struct drm_panel_funcs s6e3ha6_panel_funcs = {
	.prepare = s6e3ha6_prepare,
	.enable = s6e3ha6_enable,
	.disable = s6e3ha6_disable,
	.unprepare = s6e3ha6_unprepare,
	.get_modes = s6e3ha6_get_modes,
};

/* the "lcdtype=<id>" the boot loader appends to /chosen/bootargs */
static u32 s6e3ha6_boot_lcdtype(void)
{
	const char *args, *p;
	char num[16];
	size_t n;
	u32 id;

	if (!of_chosen || of_property_read_string(of_chosen, "bootargs", &args))
		return 0;
	for (p = strstr(args, "lcdtype="); p; p = strstr(p + 1, "lcdtype=")) {
		if (p != args && p[-1] != ' ')
			continue;
		p += strlen("lcdtype=");
		n = strcspn(p, " ");
		if (!n || n >= sizeof(num))
			return 0;
		memcpy(num, p, n);
		num[n] = '\0';
		return kstrtou32(num, 0, &id) ? 0 : id;
	}
	return 0;
}

/*
 * The vendor panel lookup table, first match of (id & mask). Both variants
 * use the same fixed sequences.
 */
static const char *s6e3ha6_variant(u32 id)
{
	static const struct { u32 id, mask; const char *name; } lut[] = {
		{ 0x806143, 0xf0ffff, "a2_s3" },
		{ 0x000000, 0x400000, "a2_s3" },
		{ 0xc00140, 0xf00fff, "a3_sa" },
		{ 0x400000, 0x400000, "a3_sa" },
	};
	int i;

	for (i = 0; i < ARRAY_SIZE(lut); i++)
		if ((lut[i].id & lut[i].mask) == (id & lut[i].mask))
			return lut[i].name;
	return "a2_s3";
}

static const struct dsu_info_dt *s6e3ha6_dsu_dt;

const struct dsu_info_dt *exynos8895_panel_get_dsu_dt(void)
{
	return READ_ONCE(s6e3ha6_dsu_dt);
}
EXPORT_SYMBOL_GPL(exynos8895_panel_get_dsu_dt);

/* DSU mode table of the panel node */
static void s6e3ha6_parse_dsu_dt(struct device *dev, struct dsu_info_dt *dt_dsu_info)
{
	struct device_node *node = dev->of_node;
	u32 dsu_res[MAX_DSU_RES_NUMBER];
	u32 dsu_number = 0;
	unsigned int i;

	of_property_read_u32(node, "dsu_en", &dt_dsu_info->dsu_en);
	dev_dbg(dev, "dsu_en : %u\n", dt_dsu_info->dsu_en);
	if (!dt_dsu_info->dsu_en)
		return;

	of_property_read_u32(node, "dsu_number", &dsu_number);
	dev_dbg(dev, "dsu_number : %u\n", dsu_number);
	if (dsu_number > MAX_DSU_RES_NUMBER) {
		dev_err(dev, "Exceed dsu res number : %u\n", dsu_number);
		dsu_number = MAX_DSU_RES_NUMBER;
	}
	dt_dsu_info->dsu_number = dsu_number;

	if (!of_property_read_u32_array(node, "dsu_width", dsu_res, dsu_number)) {
		for (i = 0; i < dsu_number; i++) {
			dt_dsu_info->res_info[i].width = dsu_res[i];
			dev_dbg(dev, "width[%u]:%u\n", i, dsu_res[i]);
		}
	}
	if (!of_property_read_u32_array(node, "dsu_height", dsu_res, dsu_number)) {
		for (i = 0; i < dsu_number; i++) {
			dt_dsu_info->res_info[i].height = dsu_res[i];
			dev_dbg(dev, "height[%u]:%u\n", i, dsu_res[i]);
		}
	}
	if (!of_property_read_u32_array(node, "dsu_dsc_en", dsu_res, dsu_number)) {
		for (i = 0; i < dsu_number; i++) {
			dt_dsu_info->res_info[i].dsc_en = dsu_res[i];
			dev_dbg(dev, "dsc_en[%u]:%u\n", i, dsu_res[i]);
		}
	}
	if (!of_property_read_u32_array(node, "dsu_dsc_width", dsu_res, dsu_number)) {
		for (i = 0; i < dsu_number; i++) {
			dt_dsu_info->res_info[i].dsc_width = dsu_res[i];
			dev_dbg(dev, "dsc_width[%u]:%u\n", i, dsu_res[i]);
		}
	}
	if (!of_property_read_u32_array(node, "dsu_dsc_height", dsu_res, dsu_number)) {
		for (i = 0; i < dsu_number; i++) {
			dt_dsu_info->res_info[i].dsc_height = dsu_res[i];
			dev_dbg(dev, "dsc_height[%u]:%u\n", i, dsu_res[i]);
		}
	}
}

/*
 * The framework's panel_device with the GPIOs and rails this driver took and
 * its command path on this DSI device. Never freed once probed: the vendor
 * framework registers class, misc and backlight devices and a kthread and
 * has no teardown, so this driver cannot be unbound.
 */
static int s6e3ha6_stock_init(struct s6e3ha6 *ctx, bool init_at_boot, bool connected)
{
	struct panel_device *stock;
	int i;

	stock = kzalloc(sizeof(*stock), GFP_KERNEL);
	if (!stock)
		return -ENOMEM;

	stock->pad.gpio_reset = ctx->reset_gpio;
	stock->pad.gpio_disp_det = ctx->det_gpio;
	stock->pad.disp_det_irq = s6e3ha6_stock_det_irq;
	for (i = 0; i < S6E3HA6_NR_SUPPLIES; i++)
		stock->pad.regulator[i] = ctx->supplies[i];

	panel_drv_init(stock, &ctx->dsi->dev, init_at_boot, connected);
	stock->lcd_info.dt_dsu_info = ctx->dsu_dt;
	panel_dsi_init(stock, ctx->dsi);
	mutex_init(&ctx->stock_lock);
	ctx->stock = stock;

	return 0;
}

static int s6e3ha6_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct s6e3ha6 *ctx;
	bool init_at_boot, connected;
	int i, ret;

	ctx = devm_drm_panel_alloc(dev, struct s6e3ha6, panel, &s6e3ha6_panel_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	for (i = 0; i < S6E3HA6_NR_SUPPLIES; i++) {
		ctx->supplies[i] = devm_regulator_get(dev, s6e3ha6_supply_names[i]);
		if (IS_ERR(ctx->supplies[i]))
			return dev_err_probe(dev, PTR_ERR(ctx->supplies[i]), "no %s supply\n",
					     s6e3ha6_supply_names[i]);
	}
	/* as is: the level tells whether the boot loader initialised the panel */
	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_ASIS);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio), "no reset-gpios\n");
	ctx->det_gpio = devm_gpiod_get_optional(dev, "disp-det", GPIOD_IN);
	if (IS_ERR(ctx->det_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->det_gpio), "no disp-det-gpios\n");

	/*
	 * Reset line high: initialised by the boot loader; then disp-det high
	 * means connected (keep it running), low not connected (hold it in
	 * reset). Reset low: the kernel initialises the panel.
	 */
	init_at_boot = gpiod_get_raw_value_cansleep(ctx->reset_gpio) == 1;
	if (init_at_boot) {
		if (!ctx->det_gpio || gpiod_get_value_cansleep(ctx->det_gpio) == 1) {
			ctx->booted = true;
			gpiod_direction_output(ctx->reset_gpio, 0);
		} else {
			gpiod_direction_output(ctx->reset_gpio, 1);
		}
	} else {
		gpiod_direction_output(ctx->reset_gpio, 1);
	}

	/* references of the rails the boot loader left on */
	if (ctx->booted) {
		for (i = 0; i < S6E3HA6_NR_SUPPLIES; i++) {
			ret = regulator_enable(ctx->supplies[i]);
			if (ret) {
				while (--i >= 0)
					regulator_disable(ctx->supplies[i]);
				return dev_err_probe(dev, ret, "cannot hold %s\n",
						     s6e3ha6_supply_names[i]);
			}
		}
		ctx->powered = true;
	}

	ctx->lcdtype = s6e3ha6_boot_lcdtype();
	ctx->variant = s6e3ha6_variant(ctx->lcdtype);
	dev_dbg(dev, "booted=%d lcdtype=0x%06x variant=%s\n", ctx->booted, ctx->lcdtype,
		ctx->variant);

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);
	ctx->on = ctx->booted;
	ctx->dsu_mode = DSU_MODE_1;
	s6e3ha6_parse_dsu_dt(dev, &ctx->dsu_dt);
	WRITE_ONCE(s6e3ha6_dsu_dt, &ctx->dsu_dt);
	ctx->decon_hooks.set_dsu = s6e3ha6_set_dsu;
	ctx->decon_hooks.data = ctx;
	exynos8895_decon_set_panel_hooks(&ctx->decon_hooks);

	/* connected unless the boot loader left it off */
	connected = ctx->booted || !init_at_boot;
	if (IS_ENABLED(CONFIG_DRM_PANEL_SAMSUNG_S6E3HA6_STOCK)) {
		ret = s6e3ha6_stock_init(ctx, init_at_boot, connected);
		if (ret)
			goto err_rails;
	}

	/*
	 * The backlight and lcd class devices of the vendor driver, which the
	 * vendor lights HAL and init scripts use; the vendor framework above
	 * registers its own.
	 */
	if (!IS_ENABLED(CONFIG_DRM_PANEL_SAMSUNG_S6E3HA6_STOCK)) {
		ctx->bright = s6e3ha6_bright_register(dev, dsi, ctx->variant);
		if (IS_ERR(ctx->bright)) {
			ret = PTR_ERR(ctx->bright);
			ctx->bright = NULL;
			goto err_wq;
		}
		ret = s6e3ha6_bright_register_lcd(ctx->bright);
		if (ret) {
			s6e3ha6_bright_unregister(ctx->bright);
			ctx->bright = NULL;
			goto err_wq;
		}
	}

	/*
	 * ESD: the workqueue exists before the interrupt is requested; the
	 * interrupt is requested off and is on while the panel is initialised.
	 */
	if (ctx->det_gpio) {
		INIT_WORK(&ctx->det_work, s6e3ha6_det_work);
		ctx->det_wq = alloc_ordered_workqueue("disp_det", 0);
		if (!ctx->det_wq) {
			ret = -ENOMEM;
			goto err_rails;
		}
		ctx->det_irq = gpiod_to_irq(ctx->det_gpio);
		if (ctx->det_irq < 0) {
			ret = dev_err_probe(dev, ctx->det_irq, "no disp-det interrupt\n");
			goto err_wq;
		}
		ret = request_irq(ctx->det_irq, s6e3ha6_det_irq_handler,
				  IRQF_TRIGGER_FALLING | IRQF_NO_AUTOEN, "disp_det", ctx);
		if (ret) {
			dev_err_probe(dev, ret, "cannot request the disp-det interrupt\n");
			goto err_wq;
		}
		if (ctx->on)
			s6e3ha6_det_irq(ctx, true);
	}

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS;

	/* the PPS above: DSC 1.1, 8 bpc, 8 bpp, line buffer 9, block prediction, RGB */
	ctx->dsc.dsc_version_major = 1;
	ctx->dsc.dsc_version_minor = 1;
	ctx->dsc.bits_per_component = 8;
	ctx->dsc.line_buf_depth = 9;
	ctx->dsc.block_pred_enable = true;
	ctx->dsc.convert_rgb = true;
	ctx->dsc.bits_per_pixel = 8 << 4;
	ctx->dsc.pic_width = 1440;
	ctx->dsc.pic_height = 2960;
	ctx->dsc.slice_width = 720;
	ctx->dsc.slice_height = 40;
	ctx->dsc.slice_count = 2;
	dsi->dsc = &ctx->dsc;

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err_probe(dev, ret, "cannot attach to the DSI host\n");
		drm_panel_remove(&ctx->panel);
		goto err_irq;
	}

	return 0;

err_irq:
	if (ctx->det_irq > 0)
		free_irq(ctx->det_irq, ctx);
err_wq:
	kfree(ctx->stock);
	if (ctx->det_wq)
		destroy_workqueue(ctx->det_wq);
	/* every path above reaches err_rails, so the bright devices go here */
err_rails:
	exynos8895_decon_set_panel_hooks(NULL);
	WRITE_ONCE(s6e3ha6_dsu_dt, NULL);
	if (ctx->bright) {
		s6e3ha6_bright_unregister(ctx->bright);
		ctx->bright = NULL;
	}
	if (ctx->booted)
		for (i = S6E3HA6_NR_SUPPLIES - 1; i >= 0; i--)
			regulator_disable(ctx->supplies[i]);
	return ret;
}

static void s6e3ha6_remove(struct mipi_dsi_device *dsi)
{
	struct s6e3ha6 *ctx = mipi_dsi_get_drvdata(dsi);

	WRITE_ONCE(s6e3ha6_dsu_dt, NULL);
	/* the DECON must not call into this ctx once it is gone */
	exynos8895_decon_set_panel_hooks(NULL);
	if (ctx->bright) {
		s6e3ha6_bright_unregister(ctx->bright);
		ctx->bright = NULL;
	}
	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
	if (ctx->det_irq > 0)
		free_irq(ctx->det_irq, ctx);
	if (ctx->det_wq)
		destroy_workqueue(ctx->det_wq);
	s6e3ha6_power_off(ctx);
}

static const struct of_device_id s6e3ha6_of_match[] = {
	{ .compatible = "samsung,s6e3ha6" },
	{ }
};
MODULE_DEVICE_TABLE(of, s6e3ha6_of_match);

static struct mipi_dsi_driver s6e3ha6_driver = {
	.probe = s6e3ha6_probe,
	.remove = s6e3ha6_remove,
	.driver = {
		.name = "panel-samsung-s6e3ha6",
		.of_match_table = s6e3ha6_of_match,
		.suppress_bind_attrs = IS_ENABLED(CONFIG_DRM_PANEL_SAMSUNG_S6E3HA6_STOCK),
	},
};
module_mipi_dsi_driver(s6e3ha6_driver);

MODULE_DESCRIPTION("Samsung S6E3HA6 DSI command-mode panel with DSC");
MODULE_LICENSE("GPL");
