// SPDX-License-Identifier: GPL-2.0-only
/*
 * Exynos 8895 (greatlte) secure monitor call helper (spec L6F 5.11): stock
 * exynos_smc() (S:arch/arm64/kernel/exynos-smc.S, S:drivers/soc/samsung/exynos-smc.c)
 * over arm_smccc_smc(), without the exynos-snapshot logging.
 */

#include <linux/arm-smccc.h>
#include <linux/export.h>
#include <asm/barrier.h>
#include <soc/samsung/exynos8895-smc.h>

/* unsigned long arguments keep stock's sign extension (S:drivers/soc/samsung/exynos-smc.c:37) */
unsigned long exynos8895_smc(unsigned long id, unsigned long a1, unsigned long a2, unsigned long a3)
{
	struct arm_smccc_res res;

	dsb(sy);			/* S:arch/arm64/kernel/exynos-smc.S:39-40 dsb sy before smc #0 */
	arm_smccc_smc(id, a1, a2, a3, 0, 0, 0, 0, &res);	/* A:include/linux/arm-smccc.h:626 */
	return res.a0;
}
EXPORT_SYMBOL_GPL(exynos8895_smc);
