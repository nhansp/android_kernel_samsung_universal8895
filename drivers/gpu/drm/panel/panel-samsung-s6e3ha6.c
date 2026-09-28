// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung S6E3HA6 AMOLED panel (Galaxy Note 8, greatlte): MIPI DSI command
 * mode, four lanes, VESA DSC with two 720x40 slices.
 *
 * The command sequences are the vendor kernel's s6e3ha6 tables
 * (S:drivers/video/fbdev/exynos/panel/s6e3ha6/s6e3ha6_great_a3_sa_panel.h;
 * the great_a2_s3 tables carry the same bytes for every packet used here).
 * The power sequence is the vendor's __set_panel_power(): prepare only
 * switches the rails and the reset line, every command is sent in enable,
 * after the DSI link is up, which is the vendor order power -> link -> init.
 *
 * S-Boot initialises the panel and shows its logo. The vendor driver detects
 * that (reset line high, panel detect high) and then neither powers nor
 * initialises the panel on the first unblank; this driver does the same
 * ("booted"), so the first modeset after boot keeps S-Boot's panel state.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/soc/samsung/greatlte-l5.h>
#include <linux/string.h>

#include <drm/display/drm_dsc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

#define S6E3HA6_INIT_TRIES	3	/* the vendor retries up to 30 times (panel_drv.c:880-889) */

/* DEBUG: greatlte heartbeat dpuc (greatlte-hb-l5.c), only when built in */
#define GL5_ON			IS_BUILTIN(CONFIG_DRM_PANEL_SAMSUNG_S6E3HA6)

/* regulators in the vendor enable order (S:.../panel/panel_drv.h:65-68,84-86) */
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
	bool booted;		/* S-Boot left the panel on and initialised */
	bool powered;		/* rails on (by S-Boot's boot reference or prepare) */
	u32 lcdtype;
	const char *variant;
};

static inline struct s6e3ha6 *to_s6e3ha6(struct drm_panel *panel)
{
	return container_of(panel, struct s6e3ha6, panel);
}

/*
 * PPS of DSU mode 1, 1440x2960 with 720x40 slices, sent verbatim
 * (great_a3_sa_pps_table[0], s6e3ha6_great_a3_sa_panel.h:1230-1249;
 * great_a2_s3_pps_table[0] :1281-1300 is identical).
 */
static const u8 s6e3ha6_pps[128] = {
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
};

/* test keys (s6e3ha6_great_a3_sa_panel.h:1420-1425) */
#define s6e3ha6_key1_on(ctx)	mipi_dsi_dcs_write_seq_multi(ctx, 0x9f, 0xa5, 0xa5)
#define s6e3ha6_key1_off(ctx)	mipi_dsi_dcs_write_seq_multi(ctx, 0x9f, 0x5a, 0x5a)
#define s6e3ha6_key2_on(ctx)	mipi_dsi_dcs_write_seq_multi(ctx, 0xf0, 0x5a, 0x5a)
#define s6e3ha6_key2_off(ctx)	mipi_dsi_dcs_write_seq_multi(ctx, 0xf0, 0xa5, 0xa5)
#define s6e3ha6_key3_on(ctx)	mipi_dsi_dcs_write_seq_multi(ctx, 0xfc, 0x5a, 0x5a)
#define s6e3ha6_key3_off(ctx)	mipi_dsi_dcs_write_seq_multi(ctx, 0xfc, 0xa5, 0xa5)

/*
 * great_a3_sa_init_cmdtbl (s6e3ha6_great_a3_sa_panel.h:1936-1979) without its
 * last entry, the brightness sequence (SET_BL_SEQ needs the AID dimming
 * tables computed from MTP reads; L5 OPEN-B1), and without the
 * DUMP_SELF_DIAG read (CONFIG_DISPLAY_USE_INFO) and COPR.
 */
static void s6e3ha6_init_seq(struct mipi_dsi_multi_context *ctx)
{
	mipi_dsi_msleep(ctx, 5);				/* wait_5msec :1911 */
	s6e3ha6_key1_on(ctx);
	/* DSI_PKT_TYPE_COMP 0x01 (great_a3_sa_dsc_table[1], :1225-1228) */
	mipi_dsi_compression_mode_multi(ctx, true);
	/* DSI_PKT_TYPE_PPS (:1734) */
	mipi_dsi_picture_parameter_set_multi(ctx,
		(const struct drm_dsc_picture_parameter_set *)s6e3ha6_pps);
	s6e3ha6_key1_off(ctx);
	mipi_dsi_dcs_exit_sleep_mode_multi(ctx);		/* 0x11 :1426 */
	mipi_dsi_msleep(ctx, 10);				/* wait_sleep_out :1912 */

	/* CONFIG_SUPPORT_DSU block: scaler off, full-size CASET/PASET, OMOK */
	s6e3ha6_key2_on(ctx);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xba, 0x01, 0x02, 0x06, 0x08, 0x08, 0x1e, 0x03);
	s6e3ha6_key2_off(ctx);
	mipi_dsi_dcs_write_seq_multi(ctx, 0x2a, 0x00, 0x00, 0x05, 0x9f);	/* 0..1439 */
	mipi_dsi_dcs_write_seq_multi(ctx, 0x2b, 0x00, 0x00, 0x0b, 0x8f);	/* 0..2959 */
	s6e3ha6_key3_on(ctx);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb0, 0x05);		/* OMOK 2CH latch :1340-1346 */
	mipi_dsi_dcs_write_seq_multi(ctx, 0xff, 0x01);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb0, 0x07);		/* OMOK zero black :1348-1354 */
	mipi_dsi_dcs_write_seq_multi(ctx, 0xf2, 0x3c, 0xd0);
	s6e3ha6_key3_off(ctx);

	s6e3ha6_key1_on(ctx);
	mipi_dsi_dcs_write_seq_multi(ctx, 0x1a, 0x1f, 0x00, 0x00, 0x00, 0x00);	/* SET_AREA :1474 */
	mipi_dsi_dcs_write_seq_multi(ctx, 0x35, 0x00);	/* TE_ON :1518, a long packet on the host */
	s6e3ha6_key1_off(ctx);
	s6e3ha6_key2_on(ctx);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xed, 0x4c);	/* ERR_FG :1520 */
	mipi_dsi_dcs_write_seq_multi(ctx, 0xc5, 0x0d, 0x08, 0x40, 0x19, 0xb7, 0x07, 0x1f,
				     0x81, 0x25, 0xce, 0x79, 0x77, 0x73, 0x00, 0xff, 0x78,
				     0x8f, 0x00, 0x00);	/* FFC :1521-1526 */
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb9, 0x01, 0x0b, 0x91, 0x00, 0x09);	/* TSP_HSYNC :1528 */
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb0, 0x03);	/* ISC :1669-1672 */
	mipi_dsi_dcs_write_seq_multi(ctx, 0xf6, 0x53);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb0, 0x08);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xf6, 0x80);
	s6e3ha6_key2_off(ctx);
}

static int s6e3ha6_power_on(struct s6e3ha6 *ctx)
{
	int i, ret;

	/* __set_panel_power(ON) (S:.../panel/panel_drv.c:111-122) */
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
	greatlte_l5_set(GL5_ON, pw, true);

	return 0;
}

static void s6e3ha6_power_off(struct s6e3ha6 *ctx)
{
	int i;

	if (!ctx->powered)
		return;
	/* __set_panel_power(OFF) (S:.../panel/panel_drv.c:123-132) */
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	for (i = S6E3HA6_NR_SUPPLIES - 1; i >= 0; i--)
		regulator_disable(ctx->supplies[i]);
	ctx->powered = false;
	greatlte_l5_set(GL5_ON, pw, false);
}

static int s6e3ha6_prepare(struct drm_panel *panel)
{
	struct s6e3ha6 *ctx = to_s6e3ha6(panel);

	/* S-Boot's rails are held by the boot references taken at probe */
	if (ctx->booted || ctx->powered)
		return 0;

	return s6e3ha6_power_on(ctx);
}

/* poll the panel detect line, 100 us x 200 (S:.../panel/panel_drv.c:215-221) */
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

static int s6e3ha6_enable(struct drm_panel *panel)
{
	struct s6e3ha6 *ctx = to_s6e3ha6(panel);
	struct mipi_dsi_multi_context dctx = { .dsi = ctx->dsi };
	int try, ret;

	/* the first unblank after boot does not reach the panel (decon_core.c:634-639) */
	if (ctx->booted)
		return 0;

	for (try = 1; try <= S6E3HA6_INIT_TRIES; try++) {
		dctx.accum_err = 0;
		s6e3ha6_init_seq(&dctx);
		if (!dctx.accum_err && s6e3ha6_det_ok(ctx))
			break;
		dev_err(&ctx->dsi->dev, "disp-det low after init (try %d/%d), err %d\n",
			try, S6E3HA6_INIT_TRIES, dctx.accum_err);
		if (try == S6E3HA6_INIT_TRIES)
			return dctx.accum_err ?: -EIO;
		/* power-cycle with the vendor retry gap (panel_drv.c:880-883) */
		s6e3ha6_power_off(ctx);
		msleep(100);
		ret = s6e3ha6_power_on(ctx);
		if (ret)
			return ret;
	}

	/* great_a3_sa_display_on_cmdtbl (:2080-2084) */
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

	/* great_a3_sa_display_off_cmdtbl (:2086-2092) */
	s6e3ha6_key1_on(&dctx);
	mipi_dsi_dcs_set_display_off_multi(&dctx);
	mipi_dsi_dcs_write_seq_multi(&dctx, 0x53, 0x02);	/* DISPLAY_OFF_HLPM_ON :1674 */
	mipi_dsi_usleep_range(&dctx, 16700, 17700);		/* one frame at 60 Hz :1914 */
	s6e3ha6_key1_off(&dctx);
	/* great_a3_sa_exit_cmdtbl (:2094-2105) */
	s6e3ha6_key1_on(&dctx);
	mipi_dsi_dcs_enter_sleep_mode_multi(&dctx);
	s6e3ha6_key1_off(&dctx);
	mipi_dsi_msleep(&dctx, 105);				/* wait_sleep_in :1913 */

	ctx->booted = false;
	greatlte_l5_set(GL5_ON, bt, false);

	return dctx.accum_err;
}

static int s6e3ha6_unprepare(struct drm_panel *panel)
{
	struct s6e3ha6 *ctx = to_s6e3ha6(panel);

	/* the first unprepare after boot drops the boot references of probe */
	ctx->booted = false;
	greatlte_l5_set(GL5_ON, bt, false);
	s6e3ha6_power_off(ctx);

	return 0;
}

/*
 * 1440x2960 at 60 Hz; porches of the DDI node (hbp, hfp, hsa = 2, 2, 2; vbp,
 * vfp, vsa = 15, 3, 1), informational in command mode.
 */
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

/*
 * "lcdtype=<id>" that S-Boot appends to the command line in /chosen/bootargs
 * (SB:0xc9009fc8 ldr w2, [x28, #0x124] -> " lcdtype=%d" SB:0xc9088958); the
 * vendor driver reads it with early_param("lcdtype") (panel_drv.c:2128-2137).
 */
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
 * panel-lut of the board DT (S:arch/arm64/boot/dts/exynos/
 * exynos8895-greatlte_common.dtsi:3383-3394), first match of
 * (lut.id & mask) == (id & mask) as find_panel_lut() (S:.../panel/panel.c:155-176).
 * Only reported: both variants use the same packets.
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

static int s6e3ha6_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct s6e3ha6 *ctx;
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
	/* as is: the level S-Boot left tells whether it initialised the panel */
	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_ASIS);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio), "no reset-gpios\n");
	ctx->det_gpio = devm_gpiod_get_optional(dev, "disp-det", GPIOD_IN);
	if (IS_ERR(ctx->det_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->det_gpio), "no disp-det-gpios\n");

	/*
	 * panel_drv_set_gpios() (S:.../panel/panel_drv.c:1386-1403): reset line
	 * high = initialised by the boot loader; then disp-det high = connected
	 * (keep it running), low = not connected (hold it in reset). Reset low =
	 * the kernel initialises the panel.
	 */
	if (gpiod_get_raw_value_cansleep(ctx->reset_gpio) == 1) {
		if (!ctx->det_gpio || gpiod_get_value_cansleep(ctx->det_gpio) == 1) {
			ctx->booted = true;
			gpiod_direction_output(ctx->reset_gpio, 0);
		} else {
			gpiod_direction_output(ctx->reset_gpio, 1);
		}
	} else {
		gpiod_direction_output(ctx->reset_gpio, 1);
	}

	/* boot references of the rails S-Boot left on (panel_drv.c:1520-1526) */
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
	dev_info(dev, "booted=%d lcdtype=0x%06x variant=%s\n", ctx->booted, ctx->lcdtype,
		 ctx->variant);
	greatlte_l5_set(GL5_ON, bt, ctx->booted);
	greatlte_l5_set(GL5_ON, pw, ctx->powered);
	greatlte_l5_set(GL5_ON, det, ctx->det_gpio);
	greatlte_l5_set(GL5_ON, lt, ctx->lcdtype);
	greatlte_l5_set(GL5_ON, var, ctx->variant);
	greatlte_l5_set(GL5_ON, panel, true);

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;			/* FDT:10098 data_lane = <0x04> */
	dsi->format = MIPI_DSI_FMT_RGB888;
	/* command mode, non-continuous clock (S:.../dpu/dsim_reg.c:1317-1318) */
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
		drm_panel_remove(&ctx->panel);
		if (ctx->booted)
			for (i = S6E3HA6_NR_SUPPLIES - 1; i >= 0; i--)
				regulator_disable(ctx->supplies[i]);
		return dev_err_probe(dev, ret, "cannot attach to the DSI host\n");
	}

	return 0;
}

static void s6e3ha6_remove(struct mipi_dsi_device *dsi)
{
	struct s6e3ha6 *ctx = mipi_dsi_get_drvdata(dsi);

	greatlte_l5_set(GL5_ON, panel, false);
	greatlte_l5_set(GL5_ON, det, NULL);
	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
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
	},
};
module_mipi_dsi_driver(s6e3ha6_driver);

MODULE_DESCRIPTION("Samsung S6E3HA6 DSI command-mode panel with DSC");
MODULE_LICENSE("GPL");
