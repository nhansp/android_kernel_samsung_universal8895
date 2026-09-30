/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Exynos 8895 DECON0 -> panel events: what the vendor DECON sent to the
 * panel driver through its v4l2 subdev (S:drivers/video/fbdev/exynos/dpu/
 * decon_core.c:3462-3498), for the S6E3HA6 vendor panel framework.
 */
#ifndef __DRM_EXYNOS8895_PANEL_H__
#define __DRM_EXYNOS8895_PANEL_H__

struct exynos8895_panel_hooks {
	/*
	 * PANEL_IOC_EVT_FRAME_DONE: a frame went out. Called from the DECON's
	 * frame-done interrupt (hard IRQ context), so it must not sleep.
	 */
	void (*frame_done)(void *data);
	void *data;
};

#if IS_ENABLED(CONFIG_DRM_EXYNOS8895_DECON)
void exynos8895_decon_set_panel_hooks(const struct exynos8895_panel_hooks *hooks);
#else
static inline void exynos8895_decon_set_panel_hooks(const struct exynos8895_panel_hooks *hooks)
{
}
#endif

#endif /* __DRM_EXYNOS8895_PANEL_H__ */
