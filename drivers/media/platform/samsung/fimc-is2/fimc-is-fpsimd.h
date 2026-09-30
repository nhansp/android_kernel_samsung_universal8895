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

/*
 * Stock fpsimd_get()/fpsimd_put() pairs around library calls and
 * fimc_is_enter/exit_lib_isr() around library ISR handlers (4.4
 * kernel_neon_begin/end, S:.../ischain/fimc-is-v5_2_0/fimc-is-hw-chain.c:
 * 110-118): each pair sits in one block, so the pair shares a context
 * declared by the opening call (spec L6B 5.4).
 */
#define fpsimd_get()						\
	struct fimc_is_fp_ctx __fimc_is_fp;			\
	fimc_is_fp_enter(&__fimc_is_fp)
#define fpsimd_put()		fimc_is_fp_exit(&__fimc_is_fp)
#define fimc_is_enter_lib_isr()	fpsimd_get()
#define fimc_is_exit_lib_isr()	fpsimd_put()

#endif /* __FIMC_IS_FPSIMD_H__ */
