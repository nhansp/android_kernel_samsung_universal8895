/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Exynos8895 DECON0 to panel events: frame done, the power requests of the
 * S3CFB front-end and DSU mode switches, for the S6E3HA6 panel driver.
 */
#ifndef __DRM_EXYNOS8895_PANEL_H__
#define __DRM_EXYNOS8895_PANEL_H__

/* the power requests of fb0 FBIOBLANK and S3CFB_POWER_MODE */
enum exynos8895_panel_power {
	EXYNOS8895_PANEL_UNBLANK,
	EXYNOS8895_PANEL_POWERDOWN,
	EXYNOS8895_PANEL_DOZE,
	EXYNOS8895_PANEL_DOZE_SUSPEND,
};

/*
 * Dynamic resolution (DSU) mode indices, DT mode table and per-frame
 * request, laid out as in the vendor driver.
 */
enum {
	DSU_MODE_NONE = 0,
	DSU_MODE_1,
	DSU_MODE_2,
	DSU_MODE_3,
	DSU_MODE_MAX,
};

#define MAX_DSU_RES_NUMBER 5

struct dsu_res_info {
	unsigned int width;
	unsigned int height;
	unsigned int dsc_en;
	unsigned int dsc_width;
	unsigned int dsc_height;
};

struct dsu_info_dt {
	unsigned int dsu_en;
	unsigned int dsu_number;
	struct dsu_res_info res_info[MAX_DSU_RES_NUMBER];
};

struct dsu_info {
	int left;
	int top;
	int right;
	int bottom;
	int mode;
	int needupdate;
};

struct exynos8895_panel_hooks {
	/*
	 * A frame went out. Called from the DECON's frame done interrupt, in
	 * hard interrupt context, so it must not sleep.
	 */
	void (*frame_done)(void *data);
	/*
	 * A power request, before the S3CFB acts on it, with whether the CRTC
	 * is active. Process context.
	 */
	int (*power)(void *data, enum exynos8895_panel_power req, bool active);
	/*
	 * The panel is in HMD mode: the DECON leaves its HW trigger running
	 * after an update. Called with a spinlock held, must not sleep.
	 */
	bool (*hmd_on)(void *data);
	/* switch the panel to a DSU mode. Process context. */
	int (*set_dsu)(void *data, const struct dsu_info *dsu);
	void *data;
};

#if IS_ENABLED(CONFIG_DRM_EXYNOS8895_DECON)
void exynos8895_decon_set_panel_hooks(const struct exynos8895_panel_hooks *hooks);
#else
static inline void exynos8895_decon_set_panel_hooks(const struct exynos8895_panel_hooks *hooks)
{
}
#endif

#if IS_ENABLED(CONFIG_DRM_PANEL_SAMSUNG_S6E3HA6)
const struct dsu_info_dt *exynos8895_panel_get_dsu_dt(void);
#else
static inline const struct dsu_info_dt *exynos8895_panel_get_dsu_dt(void)
{
	return NULL;
}
#endif

#endif /* __DRM_EXYNOS8895_PANEL_H__ */
