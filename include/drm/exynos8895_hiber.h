/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Exynos8895 display hibernation: the DECON powers itself and the DSIM down
 * between frames on a static screen. The DSIM driver registers its ULPS
 * entry points once at probe; the panel driver blocks hibernation around
 * every command, so that no packet is in flight across a power-down.
 */
#ifndef __DRM_EXYNOS8895_HIBER_H__
#define __DRM_EXYNOS8895_HIBER_H__

struct dsu_info;

struct exynos8895_dsim_hooks {
	/* samsung_dsim_enter_ulps() / samsung_dsim_exit_ulps(). Process context. */
	int (*enter_ulps)(void *data);
	int (*exit_ulps)(void *data);
	/* switch the link to a DSU mode. Process context. */
	int (*set_dsu)(void *data, const struct dsu_info *dsu);
	void *data;
};

#if IS_ENABLED(CONFIG_DRM_EXYNOS8895_DECON)
void exynos8895_decon_set_dsim_hooks(const struct exynos8895_dsim_hooks *hooks);
/*
 * Block hibernation and leave it if active. Returns the exit result, 0 when
 * the pipe was not hibernating.
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
