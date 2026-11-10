/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Exynos 8895 display hibernation: the DECON powers itself and the DSIM down
 * between frames on a static screen (S:drivers/video/fbdev/exynos/dpu/
 * decon_dsi.c:1037-1199, dsim_drv.c:1368-1475; work/full/l5-ext/DESIGN.md
 * 2.5). The DSIM driver registers its ULPS entry points once at probe; the
 * panel transport takes the hibernation block around every command so no
 * packet is in flight across a power-down. The hooks stay valid.
 */
#ifndef __DRM_EXYNOS8895_HIBER_H__
#define __DRM_EXYNOS8895_HIBER_H__

struct dsu_info;

struct exynos8895_dsim_hooks {
	/* samsung_dsim_enter_ulps() / samsung_dsim_exit_ulps(). Process context. */
	int (*enter_ulps)(void *data);
	int (*exit_ulps)(void *data);
	/* DSIM_IOC_SET_DSU / dsim_set_dsu_update() (S:dpu/dsim_drv.c:1511-1522). Process context. */
	int (*set_dsu)(void *data, const struct dsu_info *dsu);
	void *data;
};

#if IS_ENABLED(CONFIG_DRM_EXYNOS8895_DECON)
void exynos8895_decon_set_dsim_hooks(const struct exynos8895_dsim_hooks *hooks);
/*
 * decon_hiber_block_exit() (S:.../dpu/decon_dsi.c:1201-1212): block a
 * hibernation enter and leave hibernation if active. Returns the exit
 * result (0 when nothing was active).
 */
int exynos8895_decon_hiber_block_exit(void);
void exynos8895_decon_hiber_unblock(void);
#else
static inline void exynos8895_decon_set_dsim_hooks(const struct exynos8895_dsim_hooks *hooks)
{
}

static inline int exynos8895_decon_hiber_block_exit(void)
{
	return 0;
}

static inline void exynos8895_decon_hiber_unblock(void)
{
}
#endif

#endif /* __DRM_EXYNOS8895_HIBER_H__ */
