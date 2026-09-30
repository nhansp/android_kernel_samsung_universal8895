/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Exynos8895 FIMC-IS2 FP/SIMD helper (rewrite/spec/L6B_multimedia.md 5.4).
 * Replaces fpsimd_get/put, fpsimd_set_task_using, fimc_is_enter/exit_lib_isr.
 */
#ifndef __FIMC_IS_FPSIMD_H__
#define __FIMC_IS_FPSIMD_H__

#include <asm/fpsimd.h>
#include <asm/neon.h>

enum fimc_is_fp_mode {
	FP_MODE_NONE = 0,
	FP_MODE_A,
	FP_MODE_B,
	FP_MODE_NESTED,
};

struct fimc_is_fp_ctx {
	struct user_fpsimd_state st;
	enum fimc_is_fp_mode mode;
};

void fimc_is_fp_enter(struct fimc_is_fp_ctx *ctx);
void fimc_is_fp_exit(struct fimc_is_fp_ctx *ctx);

int fimc_is_neon_kthread_fn(void *data);

#endif /* __FIMC_IS_FPSIMD_H__ */
