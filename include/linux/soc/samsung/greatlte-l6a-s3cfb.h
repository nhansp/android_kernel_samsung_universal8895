/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * greatlte L6A S3CFB front-end heartbeat counters. DEBUG ONLY, never upstream.
 * Spec: rewrite/spec/L6A_graphics.md 8.1 (fields s3f, s3e, s3t). The S3CFB
 * front-end (drivers/gpu/drm/exynos/exynos8895_s3cfb.c) counts here; the rows
 * of drivers/soc/samsung/greatlte-hb-l6a-s3cfb.c read them. The driver passes
 * IS_BUILTIN(CONFIG_DRM_EXYNOS) as @on, so a modular build of it compiles the
 * calls away and needs no export (the pattern of greatlte-l5.h).
 */
#ifndef __LINUX_SOC_SAMSUNG_GREATLTE_L6A_S3CFB_H
#define __LINUX_SOC_SAMSUNG_GREATLTE_L6A_S3CFB_H

#include <linux/atomic.h>

struct greatlte_l6a_s3cfb_stats {
	atomic_t s3f;	/* frames applied (L6A 5.3.5 worker step 5) */
	atomic_t s3e;	/* WIN_CONFIG commits rejected (worker step 4) */
	atomic_t s3t;	/* acquire-fence timeouts (worker step 1) */
};

#if IS_ENABLED(CONFIG_GREATLTE_HB_L6A_S3CFB)
extern struct greatlte_l6a_s3cfb_stats greatlte_l6a_s3cfb;
#define greatlte_l6a_s3cfb_inc(on, f)	do { if (on) atomic_inc(&greatlte_l6a_s3cfb.f); } while (0)
#else
#define greatlte_l6a_s3cfb_inc(on, f)	do { } while (0)
#endif

#endif /* __LINUX_SOC_SAMSUNG_GREATLTE_L6A_S3CFB_H */
