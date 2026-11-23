// SPDX-License-Identifier: GPL-2.0-only
/*
 * Exynos8895 secure monitor call helper: the vendor exynos_smc() over
 * arm_smccc_smc().
 */

#include <linux/arm-smccc.h>
#include <linux/export.h>
#include <asm/barrier.h>
#include <soc/samsung/exynos8895-smc.h>

/* unsigned long arguments keep the vendor sign extension of negative ids */
unsigned long exynos8895_smc(unsigned long id, unsigned long a1, unsigned long a2, unsigned long a3)
{
	struct arm_smccc_res res;

	/* the vendor helper issues a full barrier before the SMC */
	dsb(sy);
	arm_smccc_smc(id, a1, a2, a3, 0, 0, 0, 0, &res);
	return res.a0;
}
EXPORT_SYMBOL_GPL(exynos8895_smc);
