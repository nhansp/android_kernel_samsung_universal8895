// SPDX-License-Identifier: GPL-2.0-only
/*
 * Backlight device "panel" of the Samsung S6E3HA6 panel and the brightness
 * sequence behind it, after the vendor panel driver: maximum brightness 36600,
 * default 12800, as the vendor lights HAL expects.
 */

#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/kobject.h>
#include <linux/lcd.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/string.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/exynos8895_hiber.h>

#include "panel-samsung-s6e3ha6-bright.h"
#include "panel-samsung-s6e3ha6-bright-tbl.h"
#include "panel-samsung-s6e3ha6-great-a2-s3-dimming.h"
#include "panel-samsung-s6e3ha6-great-a3-sa-dimming.h"

#define S6E3HA6_NORMAL_TEMPERATURE	25
#define S6E3HA6_UNDER_MINUS_15(t)	((t) <= -15)
#define S6E3HA6_UNDER_0(t)		((t) <= 0)

/* the temperature ranges of the ELVSS table */
enum s6e3ha6_elvss_plane {
	S6E3HA6_OVER_ZERO,
	S6E3HA6_UNDER_ZERO,
	S6E3HA6_UNDER_MINUS_FIFTEEN,
};

enum s6e3ha6_poc_onoff {
	S6E3HA6_POC_ONOFF_OFF,
	S6E3HA6_POC_ONOFF_ON,
};

enum s6e3ha6_irc_onoff {
	S6E3HA6_IRC_ONOFF_OFF,
	S6E3HA6_IRC_ONOFF_ON,
};

enum s6e3ha6_acl_pwrsave {
	S6E3HA6_ACL_PWRSAVE_OFF,
	S6E3HA6_ACL_PWRSAVE_ON,
};

enum s6e3ha6_acl_opr {
	S6E3HA6_ACL_OPR_OFF,
	S6E3HA6_ACL_OPR_03P,
	S6E3HA6_ACL_OPR_06P,
	S6E3HA6_ACL_OPR_08P,
	S6E3HA6_ACL_OPR_12P,
	S6E3HA6_ACL_OPR_15P,
	S6E3HA6_ACL_OPR_MAX,
};

/* registers the AID dimming reads back */
#define S6E3HA6_MTP_REG		0xC8
#define S6E3HA6_MTP_OFS		0
#define S6E3HA6_MTP_LEN		35
#define S6E3HA6_HBM_GAMMA_REG	0xB3
#define S6E3HA6_HBM_GAMMA_OFS	2
#define S6E3HA6_HBM_GAMMA_LEN	33
#define S6E3HA6_ELVSS_TEMP_0_REG	0xB5
#define S6E3HA6_ELVSS_TEMP_0_OFS	23
#define S6E3HA6_ELVSS_TEMP_1_REG	0xB5
#define S6E3HA6_ELVSS_TEMP_1_OFS	24

/* the other panel registers the lcd class attributes print */
#define S6E3HA6_ID_REG		0x04
#define S6E3HA6_ID_OFS		0
#define S6E3HA6_ID_LEN		3
#define S6E3HA6_COORDINATE_REG	0xA1
#define S6E3HA6_COORDINATE_OFS	0
#define S6E3HA6_COORDINATE_LEN	4
#define S6E3HA6_DATE_REG	0xA1
#define S6E3HA6_DATE_OFS	4
#define S6E3HA6_DATE_LEN	7
#define S6E3HA6_CODE_REG	0xD6
#define S6E3HA6_CODE_OFS	0
#define S6E3HA6_CODE_LEN	5
#define S6E3HA6_OCTA_ID_REG	0xC9
#define S6E3HA6_OCTA_ID_OFS	1
#define S6E3HA6_OCTA_ID_LEN	20

/*
 * Every table and every payload of one panel variant. The offset and the
 * copy size of each table entry in its payload are spelled out in
 * s6e3ha6_bright_apply().
 */
struct s6e3ha6_bright_variant {
	const struct panel_dimming_info *dim;

	/* SET_BL_SEQ payload templates (the command byte is payload[0]) */
	const u8 *gamma;
	size_t gamma_len;
	const u8 *aor;
	size_t aor_len;
	const u8 *tset_mps_elvss;
	size_t tset_mps_elvss_len;
	const u8 *vgh_vint;
	size_t vgh_vint_len;
	const u8 *acl_control;
	size_t acl_control_len;
	const u8 *acl_onoff;
	size_t acl_onoff_len;
	const u8 *poc_onoff;
	size_t poc_onoff_len;
	const u8 *poc_comp_gpara;
	size_t poc_comp_gpara_len;
	const u8 *poc_comp;
	size_t poc_comp_len;
	const u8 *irc_on;
	size_t irc_on_len;
	const u8 *gamma_update_enable;
	size_t gamma_update_enable_len;
	const u8 *key1_on;
	const u8 *key1_off;
	const u8 *key2_on;
	const u8 *key2_off;
	const u8 *key3_on;
	const u8 *key3_off;

	/* maptbl rows */
	const u8 (*aor_tbl)[2];
	const u8 (*mps_tbl)[1];
	const u8 (*elvss_tbl)[S6E3HA6_TOTAL_NR_LUMINANCE][1];
	const u8 (*vint_tbl)[1];
	const u8 (*acl_onoff_tbl)[1];
	const u8 (*acl_frame_avg_tbl)[1];
	const u8 (*acl_start_point_tbl)[1];
	const u8 (*acl_opr_tbl)[1];
	const u8 (*irc_onoff_tbl)[2];
	const u8 (*irc_tbl)[20];
	const u8 (*poc_onoff_tbl)[48];
	const u8 (*poc_comp_tbl)[1];
};

static const struct s6e3ha6_bright_variant s6e3ha6_variant_a3_sa = {
	.dim = &s6e3ha6_great_a3_sa_panel_dimming_info,
	.gamma = GREAT_A3_SA_GAMMA,		.gamma_len = sizeof(GREAT_A3_SA_GAMMA),
	.aor = GREAT_A3_SA_AOR,			.aor_len = sizeof(GREAT_A3_SA_AOR),
	.tset_mps_elvss = GREAT_A3_SA_TSET_MPS_ELVSS,
	.tset_mps_elvss_len = sizeof(GREAT_A3_SA_TSET_MPS_ELVSS),
	.vgh_vint = GREAT_A3_SA_VGH_VINT,	.vgh_vint_len = sizeof(GREAT_A3_SA_VGH_VINT),
	.acl_control = GREAT_A3_SA_ACL_CONTROL,	.acl_control_len = sizeof(GREAT_A3_SA_ACL_CONTROL),
	.acl_onoff = GREAT_A3_SA_ACL_ONOFF,	.acl_onoff_len = sizeof(GREAT_A3_SA_ACL_ONOFF),
	.poc_onoff = GREAT_A3_SA_POC_ONOFF,	.poc_onoff_len = sizeof(GREAT_A3_SA_POC_ONOFF),
	.poc_comp_gpara = GREAT_A3_SA_POC_COMP_GPARA,
	.poc_comp_gpara_len = sizeof(GREAT_A3_SA_POC_COMP_GPARA),
	.poc_comp = GREAT_A3_SA_POC_COMP,	.poc_comp_len = sizeof(GREAT_A3_SA_POC_COMP),
	.irc_on = GREAT_A3_SA_IRC_ON,		.irc_on_len = sizeof(GREAT_A3_SA_IRC_ON),
	.gamma_update_enable = GREAT_A3_SA_GAMMA_UPDATE_ENABLE,
	.gamma_update_enable_len = sizeof(GREAT_A3_SA_GAMMA_UPDATE_ENABLE),
	.key1_on = GREAT_A3_SA_KEY1_ENABLE,
	.key1_off = GREAT_A3_SA_KEY1_DISABLE,
	.key2_on = GREAT_A3_SA_KEY2_ENABLE,
	.key2_off = GREAT_A3_SA_KEY2_DISABLE,
	.key3_on = GREAT_A3_SA_KEY3_ENABLE,
	.key3_off = GREAT_A3_SA_KEY3_DISABLE,
	.aor_tbl = great_a3_sa_aor_table,
	.mps_tbl = great_a3_sa_mps_table,
	.elvss_tbl = great_a3_sa_elvss_table,
	.vint_tbl = great_a3_sa_vint_table,
	.acl_onoff_tbl = great_a3_sa_acl_onoff_table,
	.acl_frame_avg_tbl = great_a3_sa_acl_frame_avg_table,
	.acl_start_point_tbl = great_a3_sa_acl_start_point_table,
	.acl_opr_tbl = great_a3_sa_acl_opr_table,
	.irc_onoff_tbl = great_a3_sa_irc_onoff_table,
	.irc_tbl = great_a3_sa_irc_table,
	.poc_onoff_tbl = great_a3_sa_poc_onoff_table,
	.poc_comp_tbl = great_a3_sa_poc_comp_table,
};

static const struct s6e3ha6_bright_variant s6e3ha6_variant_a2_s3 = {
	.dim = &s6e3ha6_great_a2_s3_panel_dimming_info,
	.gamma = GREAT_A2_S3_GAMMA,		.gamma_len = sizeof(GREAT_A2_S3_GAMMA),
	.aor = GREAT_A2_S3_AOR,			.aor_len = sizeof(GREAT_A2_S3_AOR),
	.tset_mps_elvss = GREAT_A2_S3_TSET_MPS_ELVSS,
	.tset_mps_elvss_len = sizeof(GREAT_A2_S3_TSET_MPS_ELVSS),
	.vgh_vint = GREAT_A2_S3_VGH_VINT,	.vgh_vint_len = sizeof(GREAT_A2_S3_VGH_VINT),
	.acl_control = GREAT_A2_S3_ACL_CONTROL,	.acl_control_len = sizeof(GREAT_A2_S3_ACL_CONTROL),
	.acl_onoff = GREAT_A2_S3_ACL_ONOFF,	.acl_onoff_len = sizeof(GREAT_A2_S3_ACL_ONOFF),
	.poc_onoff = GREAT_A2_S3_POC_ONOFF,	.poc_onoff_len = sizeof(GREAT_A2_S3_POC_ONOFF),
	.poc_comp_gpara = GREAT_A2_S3_POC_COMP_GPARA,
	.poc_comp_gpara_len = sizeof(GREAT_A2_S3_POC_COMP_GPARA),
	.poc_comp = GREAT_A2_S3_POC_COMP,	.poc_comp_len = sizeof(GREAT_A2_S3_POC_COMP),
	.irc_on = GREAT_A2_S3_IRC_ON,		.irc_on_len = sizeof(GREAT_A2_S3_IRC_ON),
	.gamma_update_enable = GREAT_A2_S3_GAMMA_UPDATE_ENABLE,
	.gamma_update_enable_len = sizeof(GREAT_A2_S3_GAMMA_UPDATE_ENABLE),
	.key1_on = GREAT_A2_S3_KEY1_ENABLE,
	.key1_off = GREAT_A2_S3_KEY1_DISABLE,
	.key2_on = GREAT_A2_S3_KEY2_ENABLE,
	.key2_off = GREAT_A2_S3_KEY2_DISABLE,
	.key3_on = GREAT_A2_S3_KEY3_ENABLE,
	.key3_off = GREAT_A2_S3_KEY3_DISABLE,
	.aor_tbl = great_a2_s3_aor_table,
	.mps_tbl = great_a2_s3_mps_table,
	.elvss_tbl = great_a2_s3_elvss_table,
	.vint_tbl = great_a2_s3_vint_table,
	.acl_onoff_tbl = great_a2_s3_acl_onoff_table,
	.acl_frame_avg_tbl = great_a2_s3_acl_frame_avg_table,
	.acl_start_point_tbl = great_a2_s3_acl_start_point_table,
	.acl_opr_tbl = great_a2_s3_acl_opr_table,
	.irc_onoff_tbl = great_a2_s3_irc_onoff_table,
	.irc_tbl = great_a2_s3_irc_table,
	.poc_onoff_tbl = great_a2_s3_poc_onoff_table,
	.poc_comp_tbl = great_a2_s3_poc_comp_table,
};

struct s6e3ha6_bright {
	struct device *dev;
	struct mipi_dsi_device *dsi;
	const struct s6e3ha6_bright_variant *v;
	struct backlight_device *bd;
	struct lcd_device *lcd;
	struct mutex lock;		/* properties and the brightness sequence */

	/* backlight properties */
	int brightness;
	int actual_brightness;		/* luminance, in nits */
	int actual_brightness_index;	/* row of the gamma / poc_comp table */
	int acl_pwrsave;
	int acl_opr;
	int step;			/* PAC step, row of aor / irc */

	/* lcd class properties */
	int adaptive_control;
	int temperature;
	int poc_onoff;
	int irc_onoff;

	/*
	 * The panel registers, read once over the DSI link. The vendor driver
	 * also reads ELVSS but uses nothing of it, so it is not read here.
	 */
	u8 id[S6E3HA6_ID_LEN];
	u8 coordinate[S6E3HA6_COORDINATE_LEN];
	u8 date[S6E3HA6_DATE_LEN];
	u8 code[S6E3HA6_CODE_LEN];
	u8 octa_id[S6E3HA6_OCTA_ID_LEN];
	u8 mtp[S6E3HA6_MTP_LEN];
	u8 hbm_gamma[S6E3HA6_HBM_GAMMA_LEN];
	bool res_ready;

	/* AID dimming */
	struct dimming_info dim;
	struct dimming_init_info dim_init;
	u8 gamma[S6E3HA6_TOTAL_NR_LUMINANCE][S6E3HA6_GAMMA_CMD_CNT - 1];
	u8 elvss_temp[S6E3HA6_TOTAL_NR_LUMINANCE];
	bool gamma_ready;
};

/*
 * search_tbl - binary search an array of integer elements
 * The table contents must be sorted ascending.
 */
static int s6e3ha6_search_tbl(const u32 *tbl, int sz, enum SEARCH_TYPE type, int value)
{
	int l = 0, m, r = sz - 1;

	if (unlikely(!tbl || sz == 0)) {
		pr_err("%s, invalid parameter(sz %d)\n", __func__, sz);
		return -EINVAL;
	}

	if (tbl[l] > (u32)value || tbl[r] < (u32)value) {
		pr_err("%s, out of range([%u, %u]) value (%d)\n",
		       __func__, tbl[l], tbl[r], value);
		return -EINVAL;
	}

	while (l <= r) {
		m = l + (r - l) / 2;
		if (tbl[m] == (u32)value)
			return m;
		if (tbl[m] < (u32)value)
			l = m + 1;
		else
			r = m - 1;
	}

	if (type == SEARCH_TYPE_LOWER)
		return (r < 0) ? -1 : r;
	if (type == SEARCH_TYPE_UPPER)
		return (l > sz - 1) ? -1 : l;

	return -1;
}

static int s6e3ha6_get_max_brightness(const struct brightness_table *brt_tbl)
{
	if (unlikely(!brt_tbl || !brt_tbl->brt || !brt_tbl->sz_brt))
		return -EINVAL;

	return brt_tbl->brt[brt_tbl->sz_brt - 1];
}

static bool s6e3ha6_is_valid_brightness(struct s6e3ha6_bright *b, int brightness)
{
	const struct brightness_table *brt_tbl = b->v->dim->brt_tbl;
	int max_brightness;

	if (!brt_tbl->brt || !brt_tbl->sz_brt) {
		dev_err(b->dev, "brightness table not exist\n");
		return false;
	}

	max_brightness = s6e3ha6_get_max_brightness(brt_tbl);
	if (brightness < 0 || brightness > max_brightness) {
		dev_err(b->dev, "out of range %d, (min:0, max:%d)\n", brightness,
			max_brightness);
		return false;
	}

	return true;
}

/*
 * Without the vendor's per-subdevice cache, which only saved this binary
 * search.
 */
static int s6e3ha6_get_index(struct s6e3ha6_bright *b, int brightness)
{
	const struct brightness_table *brt_tbl = b->v->dim->brt_tbl;
	int index;

	if (!s6e3ha6_is_valid_brightness(b, brightness)) {
		dev_err(b->dev, "invalid brightness\n");
		return -EINVAL;
	}

	index = s6e3ha6_search_tbl(brt_tbl->brt, brt_tbl->sz_brt, SEARCH_TYPE_UPPER,
				   brightness);
	if (index < 0) {
		dev_err(b->dev, "failed to search %d, ret %d\n", brightness, index);
		return index;
	}

	if (index > (int)brt_tbl->sz_brt - 1)
		index = brt_tbl->sz_brt - 1;

	return index;
}

static int s6e3ha6_get_pac_step(struct s6e3ha6_bright *b, int brightness)
{
	const struct brightness_table *brt_tbl = b->v->dim->brt_tbl;
	int index;

	if (!s6e3ha6_is_valid_brightness(b, brightness))
		return -EINVAL;

	index = s6e3ha6_search_tbl(brt_tbl->brt_to_step, brt_tbl->sz_brt_to_step,
				   SEARCH_TYPE_UPPER, brightness);
	if (index < 0) {
		dev_err(b->dev, "failed to search %d, ret %d\n", brightness, index);
		return index;
	}

	if (index > (int)brt_tbl->sz_brt_to_step - 1)
		index = brt_tbl->sz_brt_to_step - 1;

	return index;
}

static int s6e3ha6_get_luminance(struct s6e3ha6_bright *b, int brightness)
{
	const struct brightness_table *brt_tbl = b->v->dim->brt_tbl;
	int index = s6e3ha6_get_index(b, brightness);

	if (index < 0)
		return index;

	return brt_tbl->lum[index];
}

static bool s6e3ha6_is_hbm_brightness(struct s6e3ha6_bright *b, int brightness)
{
	const struct brightness_table *brt_tbl = b->v->dim->brt_tbl;
	int luminance = s6e3ha6_get_luminance(b, brightness);
	int sz_ui_lum = brt_tbl->sz_ui_lum;

	if (sz_ui_lum <= 0 || sz_ui_lum > brt_tbl->sz_lum) {
		dev_err(b->dev, "sz_ui_lum %d out of range\n", sz_ui_lum);
		return false;
	}

	return luminance > (int)brt_tbl->lum[sz_ui_lum - 1];
}

static void s6e3ha6_update_acl_state(struct s6e3ha6_bright *b)
{
	if (s6e3ha6_is_hbm_brightness(b, b->brightness))
		b->acl_opr = S6E3HA6_ACL_OPR_08P;
	else
		b->acl_opr = S6E3HA6_ACL_OPR_15P;

	b->acl_pwrsave = (b->adaptive_control == S6E3HA6_ACL_OPR_OFF) ?
			 S6E3HA6_ACL_PWRSAVE_OFF : S6E3HA6_ACL_PWRSAVE_ON;
}

/* one DSI write */
static int s6e3ha6_dcs_write(struct s6e3ha6_bright *b, const u8 *buf, size_t len)
{
	ssize_t ret;
	int hret;

	/*
	 * Every packet leaves display hibernation first and blocks a new enter
	 * while it is in flight: while the DECON and the DSIM are powered down
	 * the DSI registers are not reachable. A refused exit fails the packet.
	 */
	hret = exynos8895_decon_hiber_block_exit();
	if (hret) {
		dev_err(b->dev, "hibernation exit failed: %d\n", hret);
		/* the block is taken even when the exit fails */
		exynos8895_decon_hiber_unblock();
		return hret;
	}

	ret = mipi_dsi_dcs_write_buffer(b->dsi, buf, len);
	exynos8895_decon_hiber_unblock();
	if (ret != len) {
		dev_err(b->dev, "dcs write 0x%02x x%zu failed: %zd\n", buf[0], len, ret);
		return ret < 0 ? (int)ret : -EIO;
	}

	return 0;
}

/*
 * A register is read at a byte offset by first programming the 0xB0
 * general parameter register. The vendor driver splits reads at 40 bytes;
 * every read here is shorter.
 */
static int s6e3ha6_dcs_read(struct s6e3ha6_bright *b, u8 reg, int ofs, u8 *buf,
			    size_t len)
{
	u8 gpara[2] = { 0xB0, 0x00 };
	ssize_t ret;
	int hret;

	/* the general-parameter register is left alone when the offset is 0 */
	if (ofs) {
		gpara[1] = ofs;
		ret = s6e3ha6_dcs_write(b, gpara, sizeof(gpara));
		if (ret)
			return ret;
	}

	/* as s6e3ha6_dcs_write() above */
	hret = exynos8895_decon_hiber_block_exit();
	if (hret) {
		dev_err(b->dev, "hibernation exit failed: %d\n", hret);
		/* as s6e3ha6_dcs_write() above */
		exynos8895_decon_hiber_unblock();
		return hret;
	}

	ret = mipi_dsi_dcs_read(b->dsi, reg, buf, len);
	exynos8895_decon_hiber_unblock();
	if (ret != len) {
		/* the vendor driver returned 0 here */
		dev_err(b->dev, "dcs read 0x%02x ofs %d len %zu failed: %zd\n",
			reg, ofs, len, ret);
		return ret < 0 ? (int)ret : -EIO;
	}

	return 0;
}

/*
 * The brightness sequence with the table rows copied into the payloads;
 * one local payload per packet, so that the templates stay const.
 */
static int s6e3ha6_bright_apply(struct s6e3ha6_bright *b)
{
	const struct s6e3ha6_bright_variant *v = b->v;
	u8 buf[64];
	int ret, plane, row;

	ret = s6e3ha6_dcs_write(b, v->key2_on, 3);		/* level2_key_enable */
	if (ret)
		return ret;

	/* gamma at offset 1 */
	memcpy(buf, v->gamma, v->gamma_len);
	memcpy(&buf[1], b->gamma[b->actual_brightness_index], S6E3HA6_GAMMA_CMD_CNT - 1);
	ret = s6e3ha6_dcs_write(b, buf, v->gamma_len);
	if (ret)
		return ret;

	/* AOR at offset 1, 2 bytes */
	memcpy(buf, v->aor, v->aor_len);
	memcpy(&buf[1], v->aor_tbl[b->step], 2);
	ret = s6e3ha6_dcs_write(b, buf, v->aor_len);
	if (ret)
		return ret;

	/*
	 * TSET (the panel temperature) at offset 1, MPS at 2, ELVSS at 3 and
	 * ELVSS_TEMP at 24
	 */
	memcpy(buf, v->tset_mps_elvss, v->tset_mps_elvss_len);
	buf[1] = (b->temperature < 0) ? BIT(7) | abs(b->temperature) : b->temperature;
	/* ACL off below 39 nits */
	row = (b->actual_brightness <= 39) ? 0 : 1;
	buf[2] = v->mps_tbl[row][0];
	plane = S6E3HA6_UNDER_MINUS_15(b->temperature) ? S6E3HA6_UNDER_MINUS_FIFTEEN :
		(S6E3HA6_UNDER_0(b->temperature) ? S6E3HA6_UNDER_ZERO : S6E3HA6_OVER_ZERO);
	buf[3] = v->elvss_tbl[plane][b->actual_brightness_index][0];
	buf[24] = b->elvss_temp[b->actual_brightness_index];
	ret = s6e3ha6_dcs_write(b, buf, v->tset_mps_elvss_len);
	if (ret)
		return ret;

	/* VINT at offset 2 */
	memcpy(buf, v->vgh_vint, v->vgh_vint_len);
	if (S6E3HA6_UNDER_MINUS_15(b->temperature)) {
		row = 9;					/* the last row */
	} else {
		static const int vint_index[] = { 5, 6, 7, 8, 9, 10, 11, 12, 13 };
		int nit = b->actual_brightness;

		for (row = 0; row < (int)ARRAY_SIZE(vint_index); row++)
			if (nit <= vint_index[row])
				break;
	}
	buf[2] = v->vint_tbl[row][0];
	ret = s6e3ha6_dcs_write(b, buf, v->vgh_vint_len);
	if (ret)
		return ret;

	/* ACL frame average at 2, start point at 3, OPR at 7 */
	memcpy(buf, v->acl_control, v->acl_control_len);
	buf[2] = v->acl_frame_avg_tbl[b->acl_pwrsave][0];
	buf[3] = v->acl_start_point_tbl[b->acl_opr][0];
	buf[7] = v->acl_opr_tbl[b->acl_opr][0];
	ret = s6e3ha6_dcs_write(b, buf, v->acl_control_len);
	if (ret)
		return ret;

	/* ACL power save at offset 1 */
	memcpy(buf, v->acl_onoff, v->acl_onoff_len);
	buf[1] = v->acl_onoff_tbl[b->acl_pwrsave][0];
	ret = s6e3ha6_dcs_write(b, buf, v->acl_onoff_len);
	if (ret)
		return ret;

	/* POC at offset 1, 48 bytes */
	memcpy(buf, v->poc_onoff, v->poc_onoff_len);
	memcpy(&buf[1], v->poc_onoff_tbl[b->poc_onoff ? S6E3HA6_POC_ONOFF_ON :
					    S6E3HA6_POC_ONOFF_OFF], 48);
	ret = s6e3ha6_dcs_write(b, buf, v->poc_onoff_len);
	if (ret)
		return ret;

	ret = s6e3ha6_dcs_write(b, v->poc_comp_gpara, v->poc_comp_gpara_len);
	if (ret)
		return ret;

	/* POC compensation at offset 3 */
	memcpy(buf, v->poc_comp, v->poc_comp_len);
	buf[3] = v->poc_comp_tbl[b->actual_brightness_index][0];
	ret = s6e3ha6_dcs_write(b, buf, v->poc_comp_len);
	if (ret)
		return ret;

	/*
	 * IRC (20 bytes) and then IRC on/off (2 bytes) both at offset 1, so the
	 * second overwrites the first two bytes, as in the vendor driver.
	 */
	memcpy(buf, v->irc_on, v->irc_on_len);
	memcpy(&buf[1], v->irc_tbl[b->step], 20);
	memcpy(&buf[1], v->irc_onoff_tbl[b->irc_onoff ? S6E3HA6_IRC_ONOFF_ON :
					S6E3HA6_IRC_ONOFF_OFF], 2);
	ret = s6e3ha6_dcs_write(b, buf, v->irc_on_len);
	if (ret)
		return ret;

	ret = s6e3ha6_dcs_write(b, v->gamma_update_enable, v->gamma_update_enable_len);
	if (ret)
		return ret;

	return s6e3ha6_dcs_write(b, v->key2_off, 3);		/* level2_key_disable */
}

/*
 * The properties are updated first, then the sequence goes out unless the
 * caller did not force it or the luminance is zero.
 */
static int s6e3ha6_bright_set(struct s6e3ha6_bright *b, int brightness, bool force)
{
	const struct brightness_table *brt_tbl = b->v->dim->brt_tbl;
	int lum, index, step;

	if (!s6e3ha6_is_valid_brightness(b, brightness)) {
		dev_err(b->dev, "invalid brightness %d\n", brightness);
		return -EINVAL;
	}

	index = s6e3ha6_get_index(b, brightness);
	lum = s6e3ha6_get_luminance(b, brightness);
	step = s6e3ha6_get_pac_step(b, brightness);
	if (index < 0 || step < 0) {
		dev_err(b->dev, "invalid pac step %d\n", step);
		return -EINVAL;
	}

	b->brightness = brightness;
	b->actual_brightness = lum;
	b->actual_brightness_index = index;
	b->step = step;
	s6e3ha6_update_acl_state(b);

	dev_dbg(b->dev, "br:%d step:%d brt:%u nit:%d acl:%s(%d)\n", brightness, step,
		brt_tbl->brt_to_step[step], lum, b->acl_pwrsave ? "on" : "off", b->acl_opr);

	if (!force || !lum)
		return 0;

	if (!b->gamma_ready) {
		/*
		 * Without a gamma table the vendor driver sends an all-zero
		 * gamma payload; the sequence is held back instead.
		 */
		dev_err_ratelimited(b->dev, "no AID gamma table, brightness not sent\n");
		return 0;
	}

	return s6e3ha6_bright_apply(b);
}

/*
 * The level 1/2/3 keys around the reads of the panel registers, in the
 * vendor order. A DRM panel has no DSI link at probe, so this runs from the
 * panel's enable(), like the gamma table below.
 */
static int s6e3ha6_bright_read_res(struct s6e3ha6_bright *b)
{
	const struct s6e3ha6_bright_variant *v = b->v;
	int ret;

	if (b->res_ready)
		return 0;

	ret = s6e3ha6_dcs_write(b, v->key1_on, 3);
	if (ret)
		return ret;
	ret = s6e3ha6_dcs_write(b, v->key2_on, 3);
	if (ret)
		return ret;
	ret = s6e3ha6_dcs_write(b, v->key3_on, 3);
	if (ret)
		return ret;

	ret = s6e3ha6_dcs_read(b, S6E3HA6_COORDINATE_REG, S6E3HA6_COORDINATE_OFS,
			       b->coordinate, sizeof(b->coordinate));
	if (ret)
		return ret;
	ret = s6e3ha6_dcs_read(b, S6E3HA6_CODE_REG, S6E3HA6_CODE_OFS, b->code,
			       sizeof(b->code));
	if (ret)
		return ret;
	ret = s6e3ha6_dcs_read(b, S6E3HA6_ELVSS_TEMP_0_REG, S6E3HA6_ELVSS_TEMP_0_OFS,
			       &b->elvss_temp[S6E3HA6_NR_LUMINANCE - 1], 1);
	if (ret)
		return ret;
	ret = s6e3ha6_dcs_read(b, S6E3HA6_ELVSS_TEMP_1_REG, S6E3HA6_ELVSS_TEMP_1_OFS,
			       &b->elvss_temp[S6E3HA6_TOTAL_NR_LUMINANCE - 1], 1);
	if (ret) {
		/* the vendor driver drops the whole read sequence when this fails */
		dev_warn(b->dev, "no ELVSS temperature 1, using 0: %d\n", ret);
		b->elvss_temp[S6E3HA6_TOTAL_NR_LUMINANCE - 1] =
			b->elvss_temp[S6E3HA6_NR_LUMINANCE - 1];
	}
	ret = s6e3ha6_dcs_read(b, S6E3HA6_MTP_REG, S6E3HA6_MTP_OFS, b->mtp,
			       sizeof(b->mtp));
	if (ret)
		return ret;
	ret = s6e3ha6_dcs_read(b, S6E3HA6_DATE_REG, S6E3HA6_DATE_OFS, b->date,
			       sizeof(b->date));
	if (ret)
		return ret;
	if (b->v->dim->nr_hbm_luminance > 0) {
		ret = s6e3ha6_dcs_read(b, S6E3HA6_HBM_GAMMA_REG, S6E3HA6_HBM_GAMMA_OFS,
				       b->hbm_gamma, sizeof(b->hbm_gamma));
		if (ret)
			return ret;
	}
	ret = s6e3ha6_dcs_read(b, S6E3HA6_OCTA_ID_REG, S6E3HA6_OCTA_ID_OFS, b->octa_id,
			       sizeof(b->octa_id));
	if (ret)
		return ret;

	/* the ID of the panel itself */
	ret = s6e3ha6_dcs_read(b, S6E3HA6_ID_REG, S6E3HA6_ID_OFS, b->id, sizeof(b->id));
	if (ret)
		return ret;

	ret = s6e3ha6_dcs_write(b, v->key3_off, 3);
	if (ret)
		return ret;
	ret = s6e3ha6_dcs_write(b, v->key2_off, 3);
	if (ret)
		return ret;
	ret = s6e3ha6_dcs_write(b, v->key1_off, 3);
	if (ret)
		return ret;

	b->res_ready = true;
	dev_dbg(b->dev, "panel id %02X%02X%02X code %02X%02X%02X%02X%02X\n",
		b->id[0], b->id[1], b->id[2], b->code[0], b->code[1], b->code[2],
		b->code[3], b->code[4]);

	return 0;
}

/* the gamma table needs the panel's MTP and HBM gamma, read over the link */
static int s6e3ha6_bright_build_gamma(struct s6e3ha6_bright *b)
{
	const struct panel_dimming_info *pdim = b->v->dim;
	s32 mtp[S6E3HA6_NR_TP][MAX_COLOR];
	s32 hbm[S6E3HA6_NR_TP][MAX_COLOR];
	unsigned int i;
	int ret;

	mtp_ctoi(mtp, (char *)b->mtp, S6E3HA6_NR_TP);

	if (pdim->nr_hbm_luminance > 0)
		hbm_ctoi(hbm, b->hbm_gamma, S6E3HA6_NR_TP);

	/*
	 * ELVSS_TEMP is the last two bytes of the 0xB5 packet: elvss_temp_0 for
	 * every row below S6E3HA6_NR_LUMINANCE and elvss_temp_1 for the rest.
	 */
	for (i = 0; i < S6E3HA6_NR_LUMINANCE; i++)
		b->elvss_temp[i] = b->elvss_temp[S6E3HA6_NR_LUMINANCE - 1];
	for (i = S6E3HA6_NR_LUMINANCE; i < S6E3HA6_TOTAL_NR_LUMINANCE; i++)
		b->elvss_temp[i] = b->elvss_temp[S6E3HA6_TOTAL_NR_LUMINANCE - 1];

	/* a previous attempt may have allocated the dimming tables already */
	free_dimming_info(&b->dim);
	b->dim_init = pdim->dim_init_info;
	b->dim = (struct dimming_info){ 0 };
	ret = init_dimming_info(&b->dim, &b->dim_init);
	if (ret) {
		dev_err(b->dev, "cannot init dimming info: %d\n", ret);
		return ret;
	}

	init_dimming_mtp(&b->dim, mtp);
	if (pdim->nr_hbm_luminance > 0 &&
	    pdim->hbm_target_luminance > pdim->target_luminance) {
		ret = init_dimming_hbm_info(&b->dim, hbm, pdim->hbm_target_luminance);
		if (ret) {
			dev_err(b->dev, "cannot init hbm gamma: %d\n", ret);
			return ret;
		}
	}

	ret = process_dimming(&b->dim);
	if (ret) {
		dev_err(b->dev, "cannot process dimming: %d\n", ret);
		return ret;
	}

	for (i = 0; i < b->v->dim->brt_tbl->sz_lum; i++) {
		int lum = b->v->dim->brt_tbl->lum[i];

		if (i >= pdim->nr_luminance + pdim->nr_hbm_luminance)
			lum = b->v->dim->brt_tbl->lum[pdim->nr_luminance +
							pdim->nr_hbm_luminance - 1];
		get_dimming_gamma_compact(&b->dim, lum, b->gamma[i]);
	}

	b->gamma_ready = true;
	dev_dbg(b->dev, "gamma table initialized (%u luminances)\n",
		b->v->dim->brt_tbl->sz_lum);

	return 0;
}

/* the backlight class reads this as the current brightness */
static int s6e3ha6_bl_get_brightness(struct backlight_device *bd)
{
	struct s6e3ha6_bright *b = bl_get_data(bd);
	int ret;

	ret = s6e3ha6_get_luminance(b, bd->props.brightness);

	return ret < 0 ? 0 : ret;
}

static int s6e3ha6_bl_set_brightness(struct backlight_device *bd)
{
	struct s6e3ha6_bright *b = bl_get_data(bd);
	int brightness = bd->props.brightness;
	int ret;

	mutex_lock(&b->lock);
	if (brightness < UI_MIN_BRIGHTNESS || brightness > EXTEND_BRIGHTNESS) {
		dev_alert(b->dev, "Brightness %d is out of range\n", brightness);
		ret = -EINVAL;
		goto exit_set;
	}

	ret = s6e3ha6_bright_set(b, brightness, true);
	if (ret)
		dev_err(b->dev, "fail to set brightness\n");

exit_set:
	mutex_unlock(&b->lock);
	return ret;
}

static const struct backlight_ops s6e3ha6_backlight_ops = {
	.get_brightness = s6e3ha6_bl_get_brightness,
	.update_status = s6e3ha6_bl_set_brightness,
};

struct s6e3ha6_bright *s6e3ha6_bright_register(struct device *dev,
					       struct mipi_dsi_device *dsi,
					       const char *variant)
{
	struct s6e3ha6_bright *b;

	b = kzalloc(sizeof(*b), GFP_KERNEL);
	if (!b)
		return ERR_PTR(-ENOMEM);

	b->dev = dev;
	b->dsi = dsi;
	b->v = !strcmp(variant, "a3_sa") ? &s6e3ha6_variant_a3_sa :
					    &s6e3ha6_variant_a2_s3;
	mutex_init(&b->lock);

	b->temperature = S6E3HA6_NORMAL_TEMPERATURE;
	b->adaptive_control = S6E3HA6_ACL_OPR_MAX - 1;
	b->poc_onoff = S6E3HA6_POC_ONOFF_ON;
	b->irc_onoff = S6E3HA6_IRC_ONOFF_ON;

	b->bd = backlight_device_register("panel", dev, b, &s6e3ha6_backlight_ops, NULL);
	if (IS_ERR(b->bd)) {
		int ret = PTR_ERR(b->bd);

		kfree(b);
		return ERR_PTR(ret);
	}
	b->bd->props.max_brightness = EXTEND_BRIGHTNESS;
	b->bd->props.brightness = UI_DEF_BRIGHTNESS;

	b->acl_pwrsave = S6E3HA6_ACL_PWRSAVE_OFF;
	b->acl_opr = S6E3HA6_ACL_OPR_OFF;

	return b;
}

int s6e3ha6_bright_prepare(struct s6e3ha6_bright *b)
{
	int ret;

	mutex_lock(&b->lock);
	ret = s6e3ha6_bright_read_res(b);
	if (!ret && !b->gamma_ready)
		ret = s6e3ha6_bright_build_gamma(b);
	if (!ret)
		/* adopt the requested brightness, but do not send it yet */
		ret = s6e3ha6_bright_set(b, b->bd->props.brightness, false);
	mutex_unlock(&b->lock);

	return ret;
}

int s6e3ha6_bright_enable(struct s6e3ha6_bright *b)
{
	int ret;

	mutex_lock(&b->lock);
	ret = s6e3ha6_bright_set(b, b->bd->props.brightness, true);
	mutex_unlock(&b->lock);

	return ret;
}

/*
 * The lcd class device "panel" with the attributes of the vendor driver that
 * the vendor lights HAL and init scripts use and that need nothing but this
 * panel.
 */
static struct s6e3ha6_bright *panel_from_lcd(struct device *dev)
{
	return lcd_get_data(to_lcd_device(dev));
}

static ssize_t lcd_type_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct s6e3ha6_bright *b = panel_from_lcd(dev);

	return sysfs_emit(buf, "SDC_%02X%02X%02X\n", b->id[0], b->id[1], b->id[2]);
}

static ssize_t manufacture_code_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct s6e3ha6_bright *b = panel_from_lcd(dev);

	return sysfs_emit(buf, "%02X%02X%02X%02X%02X\n", b->code[0], b->code[1],
			  b->code[2], b->code[3], b->code[4]);
}

static ssize_t cell_id_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct s6e3ha6_bright *b = panel_from_lcd(dev);

	return sysfs_emit(buf, "%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X\n",
			  b->date[0], b->date[1], b->date[2], b->date[3], b->date[4],
			  b->date[5], b->date[6], b->coordinate[0], b->coordinate[1],
			  b->coordinate[2], b->coordinate[3]);
}

static ssize_t color_coordinate_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct s6e3ha6_bright *b = panel_from_lcd(dev);

	return sysfs_emit(buf, "%u, %u\n", b->coordinate[0] << 8 | b->coordinate[1],
			  b->coordinate[2] << 8 | b->coordinate[3]);
}

static ssize_t manufacture_date_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct s6e3ha6_bright *b = panel_from_lcd(dev);
	int year, month, day, hour, min;

	year = ((b->date[0] & 0xF0) >> 4) + 2011;
	month = b->date[0] & 0xF;
	day = b->date[1] & 0x1F;
	hour = b->date[2] & 0x1F;
	min = b->date[3] & 0x3F;

	return sysfs_emit(buf, "%d, %d, %d, %d:%d\n", year, month, day, hour, min);
}

static ssize_t octa_id_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct s6e3ha6_bright *b = panel_from_lcd(dev);
	const u8 *octa_id = b->octa_id;
	int site, rework, poc, i, off = 0;
	bool cell_id_exist = true;

	site = (octa_id[0] >> 4) & 0x0F;
	rework = octa_id[0] & 0x0F;
	poc = octa_id[1] & 0x0F;

	for (i = 0; i < 16; i++) {
		if (!isalnum(octa_id[i + 4])) {
			cell_id_exist = false;
			break;
		}
	}

	off += sysfs_emit(buf, "%d%d%d%02x%02x", site, rework, poc, octa_id[2], octa_id[3]);
	if (cell_id_exist)
		for (i = 0; i < 16; i++)
			off += sysfs_emit_at(buf, off, "%c", octa_id[i + 4]);
	off += sysfs_emit_at(buf, off, "\n");

	return off;
}

/* 367 lines of at most 10 bytes fit the sysfs buffer */
static ssize_t brightness_table_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct s6e3ha6_bright *b = panel_from_lcd(dev);
	int br, off = 0;

	for (br = 0; br <= EXTEND_BRIGHTNESS; br += BRT_SCALE)
		off += sysfs_emit_at(buf, off, "%5d %3d\n", br,
				     s6e3ha6_get_luminance(b, br));

	return off;
}

static ssize_t adaptive_control_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct s6e3ha6_bright *b = panel_from_lcd(dev);

	return sysfs_emit(buf, "%d\n", b->adaptive_control);
}

static ssize_t adaptive_control_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t size)
{
	struct s6e3ha6_bright *b = panel_from_lcd(dev);
	unsigned int value;
	int ret;

	ret = kstrtouint(buf, 0, &value);
	if (ret)
		return ret;

	mutex_lock(&b->lock);
	if (b->adaptive_control == (int)value) {
		mutex_unlock(&b->lock);
		return size;
	}
	b->adaptive_control = value;
	mutex_unlock(&b->lock);
	backlight_update_status(b->bd);

	dev_dbg(dev, "%s, adaptive_control %d\n", __func__, b->adaptive_control);

	return size;
}

/* the vendor driver returns this fixed string, not the temperature */
static ssize_t temperature_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "-15, -14, 0, 1\n");
}

static ssize_t temperature_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	struct s6e3ha6_bright *b = panel_from_lcd(dev);
	int value, ret;

	ret = kstrtoint(buf, 10, &value);
	if (ret)
		return ret;

	mutex_lock(&b->lock);
	b->temperature = value;
	mutex_unlock(&b->lock);
	backlight_update_status(b->bd);

	dev_dbg(dev, "%s, temperature %d\n", __func__, b->temperature);

	return size;
}

static ssize_t poc_onoff_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct s6e3ha6_bright *b = panel_from_lcd(dev);

	return sysfs_emit(buf, "%d\n", b->poc_onoff);
}

static ssize_t poc_onoff_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct s6e3ha6_bright *b = panel_from_lcd(dev);
	int value, ret;

	ret = kstrtoint(buf, 0, &value);
	if (ret)
		return ret;

	dev_dbg(dev, "%s: %d -> %d\n", __func__, b->poc_onoff, value);

	if (b->poc_onoff != value) {
		mutex_lock(&b->lock);
		b->poc_onoff = value;
		mutex_unlock(&b->lock);
		backlight_update_status(b->bd);
	}

	return size;
}

static ssize_t irc_onoff_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct s6e3ha6_bright *b = panel_from_lcd(dev);

	return sysfs_emit(buf, "%d\n", b->irc_onoff);
}

static ssize_t irc_onoff_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct s6e3ha6_bright *b = panel_from_lcd(dev);
	int value, ret;

	ret = kstrtoint(buf, 0, &value);
	if (ret)
		return ret;

	dev_dbg(dev, "%s: %d -> %d\n", __func__, b->irc_onoff, value);

	if (b->irc_onoff != value) {
		mutex_lock(&b->lock);
		b->irc_onoff = value;
		mutex_unlock(&b->lock);
		backlight_update_status(b->bd);
	}

	return size;
}

/* the two SVC attributes print the cell and OCTA ids */
static ssize_t SVC_OCTA_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return cell_id_show(dev, attr, buf);
}

static ssize_t SVC_OCTA_CHIPID_show(struct device *dev, struct device_attribute *attr,
				    char *buf)
{
	return octa_id_show(dev, attr, buf);
}

static DEVICE_ATTR_RO(lcd_type);
static DEVICE_ATTR_RO(manufacture_code);
static DEVICE_ATTR_RO(cell_id);
static DEVICE_ATTR_RO(SVC_OCTA);
static DEVICE_ATTR_RO(SVC_OCTA_CHIPID);
static DEVICE_ATTR_RO(color_coordinate);
static DEVICE_ATTR_RO(manufacture_date);
static DEVICE_ATTR_RO(brightness_table);
static DEVICE_ATTR_RW(adaptive_control);
static DEVICE_ATTR_RW(temperature);
static DEVICE_ATTR_RW(poc_onoff);
static DEVICE_ATTR_RW(irc_onoff);

static struct device_attribute *s6e3ha6_lcd_attrs[] = {
	&dev_attr_lcd_type,
	&dev_attr_manufacture_code,
	&dev_attr_cell_id,
	&dev_attr_SVC_OCTA,
	&dev_attr_SVC_OCTA_CHIPID,
	&dev_attr_color_coordinate,
	&dev_attr_manufacture_date,
	&dev_attr_brightness_table,
	&dev_attr_adaptive_control,
	&dev_attr_temperature,
	&dev_attr_poc_onoff,
	&dev_attr_irc_onoff,
	NULL
};

int s6e3ha6_bright_register_lcd(struct s6e3ha6_bright *b)
{
	struct kernfs_node *svc_sd;
	struct kobject *svc, *kobj;
	int i, ret;

	b->lcd = lcd_device_register("panel", b->dev, b, NULL);
	if (IS_ERR(b->lcd))
		return PTR_ERR(b->lcd);

	for (i = 0; s6e3ha6_lcd_attrs[i]; i++) {
		ret = device_create_file(&b->lcd->dev, s6e3ha6_lcd_attrs[i]);
		if (ret) {
			dev_err(&b->lcd->dev, "cannot add %s: %d\n",
				s6e3ha6_lcd_attrs[i]->attr.name, ret);
			goto err_unregister;
		}
	}

	/*
	 * /sys/devices/svc/OCTA links to the lcd device, for the vendor SVC
	 * HAL. devices_kset is not exported, so the "devices" kobject is found
	 * by walking up from the lcd device.
	 */
	kobj = &b->lcd->dev.kobj;
	while (kobj->parent && kobject_name(kobj->parent) &&
	       strcmp(kobject_name(kobj->parent), "devices"))
		kobj = kobj->parent;
	if (!kobj->parent || !kobject_name(kobj->parent) ||
	    strcmp(kobject_name(kobj->parent), "devices")) {
		dev_err(&b->lcd->dev, "no /sys/devices for the SVC link\n");
		return 0;
	}

	svc_sd = sysfs_get_dirent(kobj->parent->sd, "svc");
	if (IS_ERR_OR_NULL(svc_sd)) {
		svc = kobject_create_and_add("svc", kobj->parent);
		if (IS_ERR_OR_NULL(svc))
			dev_err(&b->lcd->dev, "cannot create /sys/devices/svc\n");
	} else {
		svc = svc_sd->priv;
	}
	if (!IS_ERR_OR_NULL(svc)) {
		ret = sysfs_create_link(svc, &b->lcd->dev.kobj, "OCTA");
		if (ret)
			dev_err(&b->lcd->dev, "cannot create /sys/devices/svc/OCTA\n");
	}

	return 0;

err_unregister:
	lcd_device_unregister(b->lcd);
	b->lcd = NULL;
	return ret;
}

void s6e3ha6_bright_unregister(struct s6e3ha6_bright *b)
{
	if (!b)
		return;

	if (b->lcd)
		/* the sysfs files go away with the device */
		lcd_device_unregister(b->lcd);
	if (b->bd)
		backlight_device_unregister(b->bd);

	free_dimming_info(&b->dim);
	kfree(b);
}
