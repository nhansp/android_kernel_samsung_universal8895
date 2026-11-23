// SPDX-License-Identifier: GPL-2.0
//
// Exynos8895 PMU data: the vendor boot-time PMU programming (PMUCAL lpm_init)
// and the restart defaults of the boot loader's INFORM registers.

#include <linux/array_size.h>
#include <linux/compiler.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/notifier.h>
#include <linux/printk.h>
#include <linux/reboot.h>
#include <linux/soc/samsung/exynos-pmu.h>
#include <linux/types.h>

#include "exynos-pmu.h"

/*
 * The PMU writes of the vendor lpm_init sequence, in vendor order. A full mask
 * is a plain write, any other mask a read-modify-write.
 */
struct exynos8895_pmu_seq {
	u32 off, mask, val;
};

static const struct exynos8895_pmu_seq exynos8895_lpm_init[] = {
	{ 0x2608, 0x00020000, 0x00000000 },	/* CLUSTER0_L2_OPTION */
	{ 0x2628, 0x00020000, 0x00000000 },	/* CLUSTER1_L2_OPTION */
	{ 0x2008, 0x00001000, 0x00000000 },	/* CLUSTER0_CPU0_OPTION */
	{ 0x2088, 0x00001000, 0x00000000 },	/* CLUSTER0_CPU1_OPTION */
	{ 0x2108, 0x00001000, 0x00000000 },	/* CLUSTER0_CPU2_OPTION */
	{ 0x2188, 0x00001000, 0x00000000 },	/* CLUSTER0_CPU3_OPTION */
	{ 0x2208, 0x00001000, 0x00000000 },	/* CLUSTER1_CPU0_OPTION */
	{ 0x2288, 0x00001000, 0x00000000 },	/* CLUSTER1_CPU1_OPTION */
	{ 0x2308, 0x00001000, 0x00000000 },	/* CLUSTER1_CPU2_OPTION */
	{ 0x2388, 0x00001000, 0x00000000 },	/* CLUSTER1_CPU3_OPTION */
	{ 0x1438, 0x0000000f, 0x00000000 },	/* EMBEDDED_CMU_CPUCL0_SYS_PWR_REG */
	{ 0x1384, 0x00000001, 0x00000001 },	/* CPU_QCH_EMBEDDED_CMU_CPUCL0 */
	{ 0x13f8, 0x00000003, 0x00000003 },	/* CLKSTOP_OPEN_CMU_EMBEDDED_CMU_CPUCL0 */
	{ 0x14b8, 0x00000001, 0x00000001 },	/* CLKSTOP_CMU_EMBEDDED_CMU_CPUCL0 */
	{ 0x143c, 0x0000000f, 0x00000000 },	/* EMBEDDED_CMU_CPUCL1_SYS_PWR_REG */
	{ 0x13fc, 0x00000003, 0x00000000 },	/* CLKSTOP_OPEN_CMU_EMBEDDED_CMU_CPUCL1 */
	{ 0x2008, 0x0000003f, 0x0000001f },	/* CLUSTER0_CPU0_OPTION */
	{ 0x2088, 0x0000003f, 0x0000001f },	/* CLUSTER0_CPU1_OPTION */
	{ 0x2108, 0x0000003f, 0x0000001f },	/* CLUSTER0_CPU2_OPTION */
	{ 0x2188, 0x0000003f, 0x0000001f },	/* CLUSTER0_CPU3_OPTION */
	{ 0x2208, 0x0000003f, 0x0000001f },	/* CLUSTER1_CPU0_OPTION */
	{ 0x2288, 0x0000003f, 0x0000001f },	/* CLUSTER1_CPU1_OPTION */
	{ 0x2308, 0x0000003f, 0x0000001f },	/* CLUSTER1_CPU2_OPTION */
	{ 0x2388, 0x0000003f, 0x0000001f },	/* CLUSTER1_CPU3_OPTION */
	{ 0x2408, 0x00003f00, 0x00001f00 },	/* CLUSTER0_NONCPU_OPTION */
	{ 0x2428, 0x00003f00, 0x00001f00 },	/* CLUSTER1_NONCPU_OPTION */
	{ 0x2008, 0x10000000, 0x10000000 },	/* CLUSTER0_CPU0_OPTION */
	{ 0x2088, 0x10000000, 0x10000000 },	/* CLUSTER0_CPU1_OPTION */
	{ 0x2108, 0x10000000, 0x10000000 },	/* CLUSTER0_CPU2_OPTION */
	{ 0x2188, 0x10000000, 0x10000000 },	/* CLUSTER0_CPU3_OPTION */
	{ 0x2208, 0x10000000, 0x10000000 },	/* CLUSTER1_CPU0_OPTION */
	{ 0x2288, 0x10000000, 0x10000000 },	/* CLUSTER1_CPU1_OPTION */
	{ 0x2308, 0x10000000, 0x10000000 },	/* CLUSTER1_CPU2_OPTION */
	{ 0x2388, 0x10000000, 0x10000000 },	/* CLUSTER1_CPU3_OPTION */
	{ 0x2c48, 0x00000003, 0x00000001 },	/* TOP_PWR_OPTION */
	{ 0x2cc8, 0x00000003, 0x00000001 },	/* TOP_PWR_MIF_OPTION */
	{ 0x82c8, 0x00000003, 0x00000002 },	/* TOP_PWR_APM_OPTION */
	{ 0x4008, 0x00000003, 0x00000002 },	/* ABOX_OPTION */
	{ 0x4028, 0x00000003, 0x00000002 },	/* CAM_OPTION */
	{ 0x4048, 0x00000003, 0x00000002 },	/* DBG_OPTION */
	{ 0x4068, 0x00000003, 0x00000002 },	/* DPU0_OPTION */
	{ 0x4088, 0x00000003, 0x00000002 },	/* DPU1_OPTION */
	{ 0x40a8, 0x00000003, 0x00000002 },	/* DSP_OPTION */
	{ 0x40c8, 0x00000003, 0x00000002 },	/* G2D_OPTION */
	{ 0x40e8, 0x00000001, 0x00000001 },	/* EMBEDDED_G3D_OPTION */
	{ 0x4108, 0x00000003, 0x00000002 },	/* ISPHQ_OPTION */
	{ 0x4128, 0x00000003, 0x00000002 },	/* ISPLP_OPTION */
	{ 0x4148, 0x00000003, 0x00000002 },	/* IVA_OPTION */
	{ 0x4168, 0x00000003, 0x00000002 },	/* MFC_OPTION */
	{ 0x4188, 0x00000003, 0x00000002 },	/* VPU_OPTION */
	{ 0x41a8, 0x00000003, 0x00000002 },	/* VTS_OPTION */
	{ 0x5228, 0x00000003, 0x00000002 },	/* DCAM_OPTION */
	{ 0x5428, 0x00000003, 0x00000002 },	/* SRDZ_OPTION */
	{ 0x2e08, 0x00000003, 0x00000002 },	/* MEMORY_TOP_OPTION */
	{ 0x2f08, 0x00000003, 0x00000002 },	/* MEMORY_ALIVEIRAM_OPTION */
	{ 0x2f28, 0x00000003, 0x00000002 },	/* MEMORY_MIF_TOP_OPTION */
	{ 0x8228, 0x00000003, 0x00000002 },	/* MEMORY_APM_TOP_OPTION */
	{ 0x4a08, 0x00000003, 0x00000002 },	/* MEMORY_ABOX_OPTION */
	{ 0x4a28, 0x00000003, 0x00000002 },	/* MEMORY_CAM_OPTION */
	{ 0x4a48, 0x00000003, 0x00000002 },	/* MEMORY_DBG_OPTION */
	{ 0x4a68, 0x00000003, 0x00000002 },	/* MEMORY_DPU0_OPTION */
	{ 0x4a88, 0x00000003, 0x00000002 },	/* MEMORY_DPU1_OPTION */
	{ 0x4aa8, 0x00000003, 0x00000002 },	/* MEMORY_DSP_OPTION */
	{ 0x4ac8, 0x00000003, 0x00000002 },	/* MEMORY_G2D_OPTION */
	{ 0x4ae8, 0x00000003, 0x00000002 },	/* MEMORY_EMBEDDED_G3D_OPTION */
	{ 0x4b08, 0x00000003, 0x00000002 },	/* MEMORY_ISPHQ_OPTION */
	{ 0x4b28, 0x00000003, 0x00000002 },	/* MEMORY_ISPLP_OPTION */
	{ 0x4b48, 0x00000003, 0x00000002 },	/* MEMORY_IVA_OPTION */
	{ 0x4b68, 0x00000003, 0x00000002 },	/* MEMORY_MFC_OPTION */
	{ 0x4b88, 0x00000003, 0x00000002 },	/* MEMORY_VPU_OPTION */
	{ 0x4ba8, 0x00000003, 0x00000002 },	/* MEMORY_VTS_OPTION */
	{ 0x52c8, 0x00000003, 0x00000002 },	/* MEMORY_DCAM_OPTION */
	{ 0x54c8, 0x00000003, 0x00000002 },	/* MEMORY_SRDZ_OPTION */
	{ 0x330c, 0x80000000, 0x80000000 },	/* PS_HOLD_CONTROL */
	{ 0x330c, 0x00000100, 0x00000100 },	/* PS_HOLD_CONTROL */
	{ 0x82fc, 0x000fffff, 0x00000658 },	/* TCXO_DURATION3 */
	{ 0x363c, 0x000fffff, 0x00000bad },	/* EXT_REGULATOR_MIF_DURATION3 */
	{ 0x361c, 0x000fffff, 0x0000080f },	/* EXT_REGULATOR_DURATION3 */
	{ 0x0120, 0x00000003, 0x00000003 },	/* UP_SCHEDULER */
	{ 0x0420, 0x00000001, 0x00000001 },	/* RESET_LPI_TIMEOUT__TIMEOUT_ENABLE */
	{ 0x0420, 0xffff0000, 0x00200000 },	/* RESET_LPI_TIMEOUT__TIMEOUT_DURATION */
	{ 0x3028, 0x20000000, 0x00000000 },	/* PAD_RETENTION_MIF_OPTION */
	{ 0x2c48, 0xc0000000, 0xc0000000 },	/* TOP_PWR_OPTION */
	{ 0x2cc8, 0xc0000000, 0xc0000000 },	/* TOP_PWR_MIF_OPTION */
	{ 0x0220, 0xffffffff, 0x80640066 },	/* SEQ_TRANSITION0 */
	{ 0x0224, 0xffffffff, 0x80690065 },	/* SEQ_TRANSITION1 */
	{ 0x0228, 0xffffffff, 0x80650080 },	/* SEQ_TRANSITION2 */
	{ 0x0260, 0xffffffff, 0x80640066 },	/* SEQ_MIF_TRANSITION0 */
	{ 0x0264, 0xffffffff, 0x80690065 },	/* SEQ_MIF_TRANSITION1 */
	{ 0x0268, 0xffffffff, 0x80650080 },	/* SEQ_MIF_TRANSITION2 */
	{ 0x026c, 0xffffffff, 0x80010003 },	/* SEQ_MIF_TRANSITION3 */
	{ 0x0a00, 0x00000001, 0x00000001 },	/* PMU_DEBUG */
	{ 0x0a0c, 0x00000001, 0x00000001 },	/* PMU_DEBUG1 */
	{ 0x2968, 0x00000001, 0x00000001 },	/* RESET_CMU_MIF_OPTION */
	{ 0x2d90, 0x00ff0000, 0x00330000 },	/* LOGIC_RESET_MIF_DURATION0 */
	{ 0x8250, 0x00ff0000, 0x00330000 },	/* LOGIC_RESET_APM_DURATION0 */
	{ 0x2918, 0x000f0000, 0x00010000 },	/* CLKSTOP_CMU_MIF_DURATION2 */
	{ 0x8178, 0x000f0000, 0x00010000 },	/* CLKSTOP_CMU_APM_DURATION2 */
	{ 0x8198, 0x000f0000, 0x00010000 },	/* CLKSTOP_OPEN_CMU_APM_DURATION2 */
	{ 0x2608, 0x00010000, 0x00010000 },	/* CLUSTER0_L2_OPTION */
	{ 0x2628, 0x00010000, 0x00010000 },	/* CLUSTER1_L2_OPTION */
};

/*
 * The CMU and SYSREG writes of lpm_init: hardware clock gating of the CMU muxes
 * and dynamic root clock gating of the bus components, which the boot loader
 * leaves clear. pd_st: STATUS of the domain that must be on for the write.
 */
struct exynos8895_cmu_seq {
	u32 pd_st, pa, mask, val;
};

/* index in exynos8895_lpm_init[] before which the vendor writes the CMU rows */
#define EXYNOS8895_LPM_CMU_AT	81

static const struct exynos8895_cmu_seq exynos8895_lpm_init_cmu[] = {
	{      0, 0x15a80000, 0x00000001, 0x00000001 },	/* CMU_HCHGEN_CLKMUX */
	{      0, 0x15810850, 0x00000001, 0x00000001 },	/* CORE_HCHGEN_CLKMUX */
	{      0, 0x16800850, 0x0000007f, 0x00000001 },	/* CPUCL0_HCHGEN_CLKMUX */
	{      0, 0x16900850, 0x0000007f, 0x00000001 },	/* CPUCL1_HCHGEN_CLKMUX */
	{      0, 0x16000850, 0x00000001, 0x00000001 },	/* MIF0_HCHGEN_CLKMUX */
	{      0, 0x16100850, 0x00000001, 0x00000001 },	/* MIF1_HCHGEN_CLKMUX */
	{      0, 0x16200850, 0x00000001, 0x00000001 },	/* MIF2_HCHGEN_CLKMUX */
	{      0, 0x16300850, 0x00000001, 0x00000001 },	/* MIF3_HCHGEN_CLKMUX */
	{      0, 0x10010850, 0x0000001f, 0x00000000 },	/* PERIS_HCHGEN_CLKMUX */
	{ 0x4004, 0x13e20200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG_EN */
	{      0, 0x16420200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG_EN */
	{ 0x4044, 0x17c20200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG_EN */
	{      0, 0x15c20200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG_EN */
	{      0, 0x15a20200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG_EN */
	{ 0x4024, 0x12c20200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG2_EN */
	{      0, 0x16820200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG_EN */
	{      0, 0x16920200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG2_EN */
	{ 0x4044, 0x17c20200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG_EN */
	{ 0x5224, 0x12e20200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG_EN */
	{ 0x4064, 0x12820200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG_EN */
	{ 0x4084, 0x12a20200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG_EN */
	{ 0x40a4, 0x13420200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG_EN */
	{      0, 0x11020200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG_EN */
	{      0, 0x11420200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG_EN */
	{ 0x40c4, 0x13a20200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG_EN */
	{ 0x40e4, 0x13820200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG_EN */
	{      0, 0x15e20200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG_EN */
	{ 0x4104, 0x13120200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG_EN */
	{ 0x4124, 0x13020200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG_EN */
	{ 0x4144, 0x13520200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG_EN */
	{ 0x4164, 0x13c20200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG_EN */
	{      0, 0x16020200, 0xffffffff, 0xffffefff },	/* BUS_COMPONENT_DRCG_EN */
	{      0, 0x16120200, 0xffffffff, 0xffffefff },	/* BUS_COMPONENT_DRCG_EN */
	{      0, 0x16220200, 0xffffffff, 0xffffefff },	/* BUS_COMPONENT_DRCG_EN */
	{      0, 0x16320200, 0xffffffff, 0xffffefff },	/* BUS_COMPONENT_DRCG_EN */
	{      0, 0x10420200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG_EN */
	{      0, 0x10820200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG_EN */
	{      0, 0x10030200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG_EN */
	{ 0x5424, 0x14220200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG_EN */
	{ 0x4184, 0x13220200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG_EN */
	{      0, 0x15830200, 0xffffffff, 0xfffffff0 },	/* BUS_COMPONENT_DRCG_EN */
	{ 0x41a4, 0x14020200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG_EN */
};

static void exynos8895_lpm_init_cmu_apply(void)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(exynos8895_lpm_init_cmu); i++) {
		const struct exynos8895_cmu_seq *e = &exynos8895_lpm_init_cmu[i];
		void __iomem *va;
		u32 v;

		if (e->pd_st && (pmu_raw_readl(e->pd_st) & 0xf) != 0xf)
			continue;
		va = ioremap(e->pa & PAGE_MASK, PAGE_SIZE);
		if (!va)
			continue;
		v = readl(va + (e->pa & ~PAGE_MASK));
		writel(e->mask == U32_MAX ? e->val : (v & ~e->mask) | e->val,
		       va + (e->pa & ~PAGE_MASK));
		iounmap(va);
	}
}

#define EXYNOS8895_INFORM2		0x0808
#define EXYNOS8895_INFORM3		0x080c
#define SEC_POWER_RESET			0x12345678
#define SEC_RESET_REASON_UNKNOWN	0x12345670

static bool exynos8895_inform_done;
static bool exynos8895_hooks_registered;

/*
 * The boot loader takes the low nibble of a 0x1234567X INFORM3 as its boot
 * target and keeps the previous target for any other value: without these
 * defaults a reboot after "reboot recovery" boots recovery again.
 */
static void exynos8895_inform_defaults(void)
{
	pmu_raw_writel(SEC_POWER_RESET, EXYNOS8895_INFORM2);
	pmu_raw_writel(SEC_RESET_REASON_UNKNOWN, EXYNOS8895_INFORM3);
}

static int exynos8895_inform_notify(struct notifier_block *nb,
				    unsigned long action, void *cmd)
{
	if (action == SYS_RESTART) {
		exynos8895_inform_defaults();
		WRITE_ONCE(exynos8895_inform_done, true);
	}

	return NOTIFY_DONE;
}

/*
 * syscon-reboot-mode (priority 0) runs after this notifier: its INFORM3 write
 * wins for the commands it knows, the defaults stay for any other command.
 */
static struct notifier_block exynos8895_inform_nb = {
	.notifier_call	= exynos8895_inform_notify,
	.priority	= 128,
};

/*
 * emergency_restart() (the panic path) skips the reboot notifiers and reaches
 * the restart handlers directly: write the defaults there if the notifier has
 * not run, then continue to syscon-reboot (priority 192, SWRESET).
 */
static int exynos8895_inform_restart(struct sys_off_data *data)
{
	if (!READ_ONCE(exynos8895_inform_done))
		exynos8895_inform_defaults();

	return NOTIFY_DONE;
}

static void exynos8895_pmu_init(void)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(exynos8895_lpm_init); i++) {
		const struct exynos8895_pmu_seq *e = &exynos8895_lpm_init[i];
		u32 v;

		if (i == EXYNOS8895_LPM_CMU_AT)
			exynos8895_lpm_init_cmu_apply();
		v = pmu_raw_readl(e->off);
		pmu_raw_writel(e->mask == U32_MAX ? e->val : (v & ~e->mask) | e->val,
			       e->off);
	}

	/* the PMU driver may be re-bound; register the hooks once */
	if (exynos8895_hooks_registered)
		return;
	exynos8895_hooks_registered = true;

	register_reboot_notifier(&exynos8895_inform_nb);
	register_sys_off_handler(SYS_OFF_MODE_RESTART, SYS_OFF_PRIO_HIGH + 1,
				 exynos8895_inform_restart, NULL);
}

const struct exynos_pmu_data exynos8895_pmu_data = {
	.pmu_init = exynos8895_pmu_init,
};
