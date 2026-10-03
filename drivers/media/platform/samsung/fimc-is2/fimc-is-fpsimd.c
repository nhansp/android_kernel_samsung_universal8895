// SPDX-License-Identifier: GPL-2.0-only
/*
 * Exynos8895 FIMC-IS2 FP/SIMD helper (rewrite/spec/L6B_multimedia.md 5.4).
 * Replaces fpsimd_get/put, fpsimd_set_task_using, fimc_is_enter/exit_lib_isr.
 *
 * Exynos 8895 cores are ARMv8.0 without SVE/SME, and FPEN stays enabled at EL1.
 * Mode A: atomic/interrupt contexts (full raw save/restore of V0-V31+FPSR+FPCR).
 * Mode B: task context (kernel_neon_begin with user_fpsimd_state).
 */

#include <linux/kernel.h>
#include <linux/preempt.h>
#include <linux/interrupt.h>
#include <linux/percpu.h>
#include <linux/kthread.h>
#include <asm/fpsimd.h>
#include <asm/neon.h>
#include <asm/simd.h>

#include "fimc-is-fpsimd.h"

struct fimc_is_fp_level {
	struct user_fpsimd_state st;
	int depth;
};

static DEFINE_PER_CPU(struct fimc_is_fp_level, fp_hardirq);
static DEFINE_PER_CPU(struct fimc_is_fp_level, fp_softirq);
static DEFINE_PER_CPU(struct fimc_is_fp_level, fp_atomic);

void fimc_is_fp_enter(struct fimc_is_fp_ctx *ctx)
{
	if (in_hardirq() || in_serving_softirq() || irqs_disabled() || !preemptible()) {
		struct fimc_is_fp_level *l = in_hardirq() ? this_cpu_ptr(&fp_hardirq) :
					     in_serving_softirq() ? this_cpu_ptr(&fp_softirq) :
					     this_cpu_ptr(&fp_atomic);
		if (l->depth++ == 0)
			fpsimd_save_state(&l->st);
		ctx->mode = FP_MODE_A;
		return;
	}

	WARN_ON(in_hardirq());
	if (test_thread_flag(TIF_KERNEL_FPSTATE)) {
		ctx->mode = FP_MODE_NESTED;
		return;
	}

	kernel_neon_begin(&ctx->st);
	ctx->mode = FP_MODE_B;
}
EXPORT_SYMBOL_GPL(fimc_is_fp_enter);

void fimc_is_fp_exit(struct fimc_is_fp_ctx *ctx)
{
	if (ctx->mode == FP_MODE_A) {
		struct fimc_is_fp_level *l = in_hardirq() ? this_cpu_ptr(&fp_hardirq) :
					     in_serving_softirq() ? this_cpu_ptr(&fp_softirq) :
					     this_cpu_ptr(&fp_atomic);
		if (--l->depth == 0)
			fpsimd_load_state(&l->st);
	} else if (ctx->mode == FP_MODE_B) {
		kernel_neon_end(&ctx->st);
	}
}
EXPORT_SYMBOL_GPL(fimc_is_fp_exit);

int fimc_is_neon_kthread_fn(void *data)
{
	struct user_fpsimd_state st;
	int ret;

	kernel_neon_begin(&st);
	ret = kthread_worker_fn(data);
	kernel_neon_end(&st);
	return ret;
}
EXPORT_SYMBOL_GPL(fimc_is_neon_kthread_fn);
