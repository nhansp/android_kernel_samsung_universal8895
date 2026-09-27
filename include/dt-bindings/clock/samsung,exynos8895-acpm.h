/* SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause) */
/*
 * Device Tree binding constants for the Exynos8895 ACPM DVFS clocks.
 *
 * The id is the ACPM DVFS domain index the firmware takes in a request: the
 * vendor kernel's ACPM_DVFS_* ids (include/dt-bindings/clock/exynos8895.h,
 * 0x0B040000..0x0B040009) masked with MASK_OF_ID 0xFFFF.
 */

#ifndef _DT_BINDINGS_CLOCK_SAMSUNG_EXYNOS8895_ACPM_H
#define _DT_BINDINGS_CLOCK_SAMSUNG_EXYNOS8895_ACPM_H

#define EXYNOS8895_CLK_ACPM_DVFS_MIF		0
#define EXYNOS8895_CLK_ACPM_DVFS_INT		1
#define EXYNOS8895_CLK_ACPM_DVFS_CPUCL0		2	/* Mongoose cluster, cpus 4-7 */
#define EXYNOS8895_CLK_ACPM_DVFS_CPUCL1		3	/* Cortex-A53 cluster, cpus 0-3 */
#define EXYNOS8895_CLK_ACPM_DVFS_G3D		4
#define EXYNOS8895_CLK_ACPM_DVFS_INTCAM		5
#define EXYNOS8895_CLK_ACPM_DVFS_CAM		6
#define EXYNOS8895_CLK_ACPM_DVFS_DISP		7
#define EXYNOS8895_CLK_ACPM_DVFS_G3DM		8
#define EXYNOS8895_CLK_ACPM_DVFS_CP		9

#endif /* _DT_BINDINGS_CLOCK_SAMSUNG_EXYNOS8895_ACPM_H */
