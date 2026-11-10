/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Exynos 8895 DECON0 -> panel events: what the vendor DECON sent to the
 * panel driver through its v4l2 subdevs (frame done, S:drivers/video/fbdev/
 * exynos/dpu/decon_core.c:3462-3498; the power requests), for the S6E3HA6
 * vendor panel framework. The hooks are registered once and stay valid.
 */
#ifndef __DRM_EXYNOS8895_PANEL_H__
#define __DRM_EXYNOS8895_PANEL_H__

/*
 * The S3CFB power requests of the stock composer (fb0 FBIOBLANK and
 * S3CFB_POWER_MODE), which the vendor DECON passed on to the panel
 * (S:drivers/video/fbdev/exynos/dpu/decon_core.c:640-649, 2782-2945).
 */
enum exynos8895_panel_power {
	EXYNOS8895_PANEL_UNBLANK,
	EXYNOS8895_PANEL_POWERDOWN,
	EXYNOS8895_PANEL_DOZE,
	EXYNOS8895_PANEL_DOZE_SUSPEND,
};

/*
 * Dynamic Screen Scaling (DSU) mode indices, DT mode table, and per-frame
 * request structure (S:drivers/video/fbdev/exynos/dpu/decon.h:83-98,
 * S:drivers/video/fbdev/exynos/dpu/decon_lcd.h:46-60).
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
	 * PANEL_IOC_EVT_FRAME_DONE: a frame went out. Called from the DECON's
	 * frame-done interrupt (hard IRQ context), so it must not sleep.
	 */
	void (*frame_done)(void *data);
	/*
	 * A power request, before the S3CFB acts on it, with whether the CRTC
	 * is active. Process context.
	 */
	int (*power)(void *data, enum exynos8895_panel_power req, bool active);
	/*
	 * The panel is in HMD mode (panel_state.hmd_on): the DECON leaves its
	 * HW trigger running after an update (decon_core.c:2224-2228,
	 * 2320-2323). Called with a spinlock held, must not sleep.
	 */
	bool (*hmd_on)(void *data);
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
