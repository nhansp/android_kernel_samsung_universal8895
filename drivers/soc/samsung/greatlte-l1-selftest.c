// SPDX-License-Identifier: GPL-2.0-only
/*
 * greatlte L1 self-test: read-only snapshot of the PMU, the power domains, the
 * L1 clocks and the pin controllers. DEBUG ONLY, never upstream.
 * Spec: rewrite/spec/L1_platform.md 5.9 (pass lines P5-P11 of 1), with
 * REWRITE_PLAN.md 5 X-5/X-25: the periodic values go through the heartbeat
 * printer hook (T_tooling.md 5.7.6) as "L1" (fast: pd_on, rst) and "L1id"
 * (every 300 s: the hand-off values and the post-boot snapshot) instead of an
 * own L1hb line; the one-shot lines are printed once.
 *
 * 1. early_initcall (before exynos_pd_probe and the leaf CMUs at core_initcall
 *    and before exynos8895_pmu_init at postcore_initcall): of_iomap() of the
 *    PMU node (non-requesting), reads only, prints the S-Boot hand-off:
 *      L1pmu0 rst_stat= inform2= inform3= ps_hold= rsc= wdtdis= swdis=
 *      L1pd0 <label> cfg= st= opt=                         (16, spec 4.4 order)
 *      L1lpm0 :<CAL line> <pa> val= want=<mask>/<val>      (each mismatch)
 *      L1lpm0 checked= mismatch= skipped=
 *    over all 143 targets of stock pmucal_lpm_init: PMU words through the PMU
 *    mapping, the others through a temporary ioremap() of their 4 KiB page; a
 *    SYSREG inside a power domain only while that domain's STATUS[3:0] = 0xF
 *    (else "skipped"). Expected (reg & mask) == val = stock's write.
 * 2. probe (device_initcall): syscon regmap, devm_clk_bulk_get_all() (get
 *    only, never prepare/enable), of_iomap() of every greatlte,cmus node.
 * 3. late_initcall_sync arms a work that waits for SYSTEM_RUNNING (after
 *    clk_disable_unused and genpd_power_off_unused, which are
 *    late_initcall_sync themselves) plus 5 s for deferred sync_state, then
 *    prints once:
 *      L1pmu sys= up= rst_stat= inform2= inform3= ps_hold= rsc= wdtdis= swdis=
 *      L1pd <label> cfg= st= opt=                          (16, via the regmap)
 *      L1clk <name> rate= en=                             (each node clock)
 *      L1raw <pll> con0=                                  (shared0-4, cpucl0/1)
 *      L1cpu <n> mpidr= st=                               (CLUSTERx_CPUy_STATUS)
 *      L1pinctrl <node> bound= avail=                     (8 pin controllers)
 *      L1dev <dev> bound=                                 (greatlte,uart)
 * 4. Trigger parameters (spec 8.4), acted on once at 60 s uptime, the only
 *    writes this file can cause: greatlte_l1_selftest.reboot=<cmd> (empty:
 *    kernel_restart(NULL)), .panic=1, .poweroff=1, .wdt_hang=1 (every CPU
 *    spins with interrupts off; with s3c2410_wdt.tmr_atboot=1
 *    s3c2410_wdt.soft_noboot=0 the watchdog must reset the SoC).
 * All lines are KERN_ERR so that they pass S-Boot's loglevel=4.
 */
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/soc/samsung/greatlte-hb.h>
#include <linux/stop_machine.h>
#include <linux/timekeeping.h>
#include <linux/workqueue.h>
#include <asm/smp_plat.h>

#define L1_PMU_SIZE	0x10000		/* S:arch/arm64/boot/dts/exynos/exynos8895.dtsi:301 */
#define L1_PD_STATUS	0x4		/* CONFIGURATION + 4 = STATUS, + 8 = OPTION (spec 5.8) */
#define L1_PD_OPTION	0x8

static char *l1_reboot;
module_param_named(reboot, l1_reboot, charp, 0444);
MODULE_PARM_DESC(reboot, "at 60 s: kernel_restart(<cmd>); empty = kernel_restart(NULL)");
static bool l1_panic;
module_param_named(panic, l1_panic, bool, 0444);
MODULE_PARM_DESC(panic, "at 60 s: panic()");
static bool l1_poweroff;
module_param_named(poweroff, l1_poweroff, bool, 0444);
MODULE_PARM_DESC(poweroff, "at 60 s: kernel_power_off()");
static bool l1_wdt_hang;
module_param_named(wdt_hang, l1_wdt_hang, bool, 0444);
MODULE_PARM_DESC(wdt_hang, "at 60 s: every CPU spins with interrupts off");
static unsigned int l1_hb_ms = 10000;
module_param_named(hb_ms, l1_hb_ms, uint, 0444);
MODULE_PARM_DESC(hb_ms, "cadence of the L1 heartbeat line (REWRITE_PLAN.md X-5: 10000; all-in image 60000)");
static unsigned int l1_id_ms = 300000;
module_param_named(id_ms, l1_id_ms, uint, 0444);
MODULE_PARM_DESC(id_ms, "cadence of the L1id line (X-5: 300000; 60000 when the run ends by a reboot at ~243 s)");
static bool l1_lpm_lines = true;
module_param_named(lpm_lines, l1_lpm_lines, bool, 0444);
MODULE_PARM_DESC(lpm_lines, "print one L1lpm0 line per lpm_init mismatch");

/* PMU words of the L1pmu0/L1pmu lines (spec 5.8/5.9) */
static const u32 l1_pmu_off[] = {
	0x0404,		/* RST_STAT: S:drivers/watchdog/s3c2410_wdt.c:71 */
	0x0808,		/* INFORM2: S:include/linux/sec_ext.h:27 */
	0x080c,		/* INFORM3: S:include/linux/sec_ext.h:28 */
	0x330c,		/* PS_HOLD_CONTROL: S:drivers/soc/samsung/exynos-reboot.c:77 */
	0x0500,		/* RESET_SEQUENCER_CONFIGURATION: S:drivers/soc/samsung/exynos-reboot.c:76 */
	0x0414,		/* CPU_RESET_DISABLE_FROM_WDTRESET: S:drivers/soc/samsung/exynos-reboot.c:71 */
	0x041c,		/* CPU_RESET_DISABLE_FROM_SOFTRESET: S:drivers/soc/samsung/exynos-reboot.c:70 */
};

/* The 16 domains in spec 4.4 order: CONFIGURATION offsets, CAL:pmucal_local_exynos8895.h */
static const struct {
	const char *label;
	u32 conf;
} l1_pd[] = {
	{ "abox", 0x4000 },	/* :2 */
	{ "cam", 0x4020 },	/* :72 */
	{ "dbg", 0x4040 },	/* :124 */
	{ "dpu0", 0x4060 },	/* :199 */
	{ "dpu1", 0x4080 },	/* :243 */
	{ "dsp", 0x40a0 },	/* :280 */
	{ "g2d", 0x40c0 },	/* :315 */
	{ "g3d", 0x40e0 },	/* :361 */
	{ "isphq", 0x4100 },	/* :381 */
	{ "isplp", 0x4120 },	/* :412 */
	{ "iva", 0x4140 },	/* :447 */
	{ "mfc", 0x4160 },	/* :498 */
	{ "vpu", 0x4180 },	/* :567 */
	{ "vts", 0x41a0 },	/* :599 */
	{ "dcam", 0x5220 },	/* :168 */
	{ "srdz", 0x5420 },	/* :532 */
};

/*
 * All 143 writes of stock pmucal_lpm_init (CAL:pmucal_system_exynos8895.h:3-145,
 * CAL: = S:drivers/soc/samsung/cal-if/exynos8895/): {CAL line, STATUS offset of
 * the hosting domain (0 = always readable), physical address, mask, value}.
 * Generated by rewrite/work/scratch/L1_final/gen_lpm.py (spec Appendix C).
 */
struct l1_lpm {
	u16 line;
	u16 pd_st;
	u32 pa, mask, val;
};

static const struct l1_lpm l1_lpm[] __initconst = {
	{   3, 0, 0x16482608, 0x00020000, 0x00000000 },	/* CLUSTER0_L2_OPTION */
	{   4, 0, 0x16482628, 0x00020000, 0x00000000 },	/* CLUSTER1_L2_OPTION */
	{   5, 0, 0x16482008, 0x00001000, 0x00000000 },	/* CLUSTER0_CPU0_OPTION */
	{   6, 0, 0x16482088, 0x00001000, 0x00000000 },	/* CLUSTER0_CPU1_OPTION */
	{   7, 0, 0x16482108, 0x00001000, 0x00000000 },	/* CLUSTER0_CPU2_OPTION */
	{   8, 0, 0x16482188, 0x00001000, 0x00000000 },	/* CLUSTER0_CPU3_OPTION */
	{   9, 0, 0x16482208, 0x00001000, 0x00000000 },	/* CLUSTER1_CPU0_OPTION */
	{  10, 0, 0x16482288, 0x00001000, 0x00000000 },	/* CLUSTER1_CPU1_OPTION */
	{  11, 0, 0x16482308, 0x00001000, 0x00000000 },	/* CLUSTER1_CPU2_OPTION */
	{  12, 0, 0x16482388, 0x00001000, 0x00000000 },	/* CLUSTER1_CPU3_OPTION */
	{  13, 0, 0x16481438, 0x0000000f, 0x00000000 },	/* EMBEDDED_CMU_CPUCL0_SYS_PWR_REG */
	{  14, 0, 0x16481384, 0x00000001, 0x00000001 },	/* CPU_QCH_EMBEDDED_CMU_CPUCL0 */
	{  15, 0, 0x164813f8, 0x00000003, 0x00000003 },	/* CLKSTOP_OPEN_CMU_EMBEDDED_CMU_CPUCL0 */
	{  16, 0, 0x164814b8, 0x00000001, 0x00000001 },	/* CLKSTOP_CMU_EMBEDDED_CMU_CPUCL0 */
	{  17, 0, 0x1648143c, 0x0000000f, 0x00000000 },	/* EMBEDDED_CMU_CPUCL1_SYS_PWR_REG */
	{  18, 0, 0x164813fc, 0x00000003, 0x00000000 },	/* CLKSTOP_OPEN_CMU_EMBEDDED_CMU_CPUCL1 */
	{  19, 0, 0x16482008, 0x0000003f, 0x0000001f },	/* CLUSTER0_CPU0_OPTION */
	{  20, 0, 0x16482088, 0x0000003f, 0x0000001f },	/* CLUSTER0_CPU1_OPTION */
	{  21, 0, 0x16482108, 0x0000003f, 0x0000001f },	/* CLUSTER0_CPU2_OPTION */
	{  22, 0, 0x16482188, 0x0000003f, 0x0000001f },	/* CLUSTER0_CPU3_OPTION */
	{  23, 0, 0x16482208, 0x0000003f, 0x0000001f },	/* CLUSTER1_CPU0_OPTION */
	{  24, 0, 0x16482288, 0x0000003f, 0x0000001f },	/* CLUSTER1_CPU1_OPTION */
	{  25, 0, 0x16482308, 0x0000003f, 0x0000001f },	/* CLUSTER1_CPU2_OPTION */
	{  26, 0, 0x16482388, 0x0000003f, 0x0000001f },	/* CLUSTER1_CPU3_OPTION */
	{  27, 0, 0x16482408, 0x00003f00, 0x00001f00 },	/* CLUSTER0_NONCPU_OPTION */
	{  28, 0, 0x16482428, 0x00003f00, 0x00001f00 },	/* CLUSTER1_NONCPU_OPTION */
	{  29, 0, 0x16482008, 0x10000000, 0x10000000 },	/* CLUSTER0_CPU0_OPTION */
	{  30, 0, 0x16482088, 0x10000000, 0x10000000 },	/* CLUSTER0_CPU1_OPTION */
	{  31, 0, 0x16482108, 0x10000000, 0x10000000 },	/* CLUSTER0_CPU2_OPTION */
	{  32, 0, 0x16482188, 0x10000000, 0x10000000 },	/* CLUSTER0_CPU3_OPTION */
	{  33, 0, 0x16482208, 0x10000000, 0x10000000 },	/* CLUSTER1_CPU0_OPTION */
	{  34, 0, 0x16482288, 0x10000000, 0x10000000 },	/* CLUSTER1_CPU1_OPTION */
	{  35, 0, 0x16482308, 0x10000000, 0x10000000 },	/* CLUSTER1_CPU2_OPTION */
	{  36, 0, 0x16482388, 0x10000000, 0x10000000 },	/* CLUSTER1_CPU3_OPTION */
	{  37, 0, 0x16482c48, 0x00000003, 0x00000001 },	/* TOP_PWR_OPTION */
	{  38, 0, 0x16482cc8, 0x00000003, 0x00000001 },	/* TOP_PWR_MIF_OPTION */
	{  39, 0, 0x164882c8, 0x00000003, 0x00000002 },	/* TOP_PWR_APM_OPTION */
	{  40, 0, 0x16484008, 0x00000003, 0x00000002 },	/* ABOX_OPTION */
	{  41, 0, 0x16484028, 0x00000003, 0x00000002 },	/* CAM_OPTION */
	{  42, 0, 0x16484048, 0x00000003, 0x00000002 },	/* DBG_OPTION */
	{  43, 0, 0x16484068, 0x00000003, 0x00000002 },	/* DPU0_OPTION */
	{  44, 0, 0x16484088, 0x00000003, 0x00000002 },	/* DPU1_OPTION */
	{  45, 0, 0x164840a8, 0x00000003, 0x00000002 },	/* DSP_OPTION */
	{  46, 0, 0x164840c8, 0x00000003, 0x00000002 },	/* G2D_OPTION */
	{  47, 0, 0x164840e8, 0x00000001, 0x00000001 },	/* EMBEDDED_G3D_OPTION */
	{  48, 0, 0x16484108, 0x00000003, 0x00000002 },	/* ISPHQ_OPTION */
	{  49, 0, 0x16484128, 0x00000003, 0x00000002 },	/* ISPLP_OPTION */
	{  50, 0, 0x16484148, 0x00000003, 0x00000002 },	/* IVA_OPTION */
	{  51, 0, 0x16484168, 0x00000003, 0x00000002 },	/* MFC_OPTION */
	{  52, 0, 0x16484188, 0x00000003, 0x00000002 },	/* VPU_OPTION */
	{  53, 0, 0x164841a8, 0x00000003, 0x00000002 },	/* VTS_OPTION */
	{  54, 0, 0x16485228, 0x00000003, 0x00000002 },	/* DCAM_OPTION */
	{  55, 0, 0x16485428, 0x00000003, 0x00000002 },	/* SRDZ_OPTION */
	{  56, 0, 0x16482e08, 0x00000003, 0x00000002 },	/* MEMORY_TOP_OPTION */
	{  57, 0, 0x16482f08, 0x00000003, 0x00000002 },	/* MEMORY_ALIVEIRAM_OPTION */
	{  58, 0, 0x16482f28, 0x00000003, 0x00000002 },	/* MEMORY_MIF_TOP_OPTION */
	{  59, 0, 0x16488228, 0x00000003, 0x00000002 },	/* MEMORY_APM_TOP_OPTION */
	{  60, 0, 0x16484a08, 0x00000003, 0x00000002 },	/* MEMORY_ABOX_OPTION */
	{  61, 0, 0x16484a28, 0x00000003, 0x00000002 },	/* MEMORY_CAM_OPTION */
	{  62, 0, 0x16484a48, 0x00000003, 0x00000002 },	/* MEMORY_DBG_OPTION */
	{  63, 0, 0x16484a68, 0x00000003, 0x00000002 },	/* MEMORY_DPU0_OPTION */
	{  64, 0, 0x16484a88, 0x00000003, 0x00000002 },	/* MEMORY_DPU1_OPTION */
	{  65, 0, 0x16484aa8, 0x00000003, 0x00000002 },	/* MEMORY_DSP_OPTION */
	{  66, 0, 0x16484ac8, 0x00000003, 0x00000002 },	/* MEMORY_G2D_OPTION */
	{  67, 0, 0x16484ae8, 0x00000003, 0x00000002 },	/* MEMORY_EMBEDDED_G3D_OPTION */
	{  68, 0, 0x16484b08, 0x00000003, 0x00000002 },	/* MEMORY_ISPHQ_OPTION */
	{  69, 0, 0x16484b28, 0x00000003, 0x00000002 },	/* MEMORY_ISPLP_OPTION */
	{  70, 0, 0x16484b48, 0x00000003, 0x00000002 },	/* MEMORY_IVA_OPTION */
	{  71, 0, 0x16484b68, 0x00000003, 0x00000002 },	/* MEMORY_MFC_OPTION */
	{  72, 0, 0x16484b88, 0x00000003, 0x00000002 },	/* MEMORY_VPU_OPTION */
	{  73, 0, 0x16484ba8, 0x00000003, 0x00000002 },	/* MEMORY_VTS_OPTION */
	{  74, 0, 0x164852c8, 0x00000003, 0x00000002 },	/* MEMORY_DCAM_OPTION */
	{  75, 0, 0x164854c8, 0x00000003, 0x00000002 },	/* MEMORY_SRDZ_OPTION */
	{  76, 0, 0x1648330c, 0x80000000, 0x80000000 },	/* PS_HOLD_CONTROL */
	{  77, 0, 0x1648330c, 0x00000100, 0x00000100 },	/* PS_HOLD_CONTROL */
	{  78, 0, 0x164882fc, 0x000fffff, 0x00000658 },	/* TCXO_DURATION3 */
	{  79, 0, 0x1648363c, 0x000fffff, 0x00000bad },	/* EXT_REGULATOR_MIF_DURATION3 */
	{  80, 0, 0x1648361c, 0x000fffff, 0x0000080f },	/* EXT_REGULATOR_DURATION3 */
	{  81, 0, 0x16480120, 0x00000003, 0x00000003 },	/* UP_SCHEDULER */
	{  82, 0, 0x16480420, 0x00000001, 0x00000001 },	/* RESET_LPI_TIMEOUT__TIMEOUT_ENABLE */
	{  83, 0, 0x16480420, 0xffff0000, 0x00200000 },	/* RESET_LPI_TIMEOUT__TIMEOUT_DURATION */
	{  84, 0, 0x15a80000, 0x00000001, 0x00000001 },	/* CMU_HCHGEN_CLKMUX */
	{  85, 0, 0x15810850, 0x00000001, 0x00000001 },	/* CORE_HCHGEN_CLKMUX */
	{  86, 0, 0x16800850, 0x0000007f, 0x00000001 },	/* CPUCL0_HCHGEN_CLKMUX */
	{  87, 0, 0x16900850, 0x0000007f, 0x00000001 },	/* CPUCL1_HCHGEN_CLKMUX */
	{  88, 0, 0x16000850, 0x00000001, 0x00000001 },	/* MIF0_HCHGEN_CLKMUX */
	{  89, 0, 0x16100850, 0x00000001, 0x00000001 },	/* MIF1_HCHGEN_CLKMUX */
	{  90, 0, 0x16200850, 0x00000001, 0x00000001 },	/* MIF2_HCHGEN_CLKMUX */
	{  91, 0, 0x16300850, 0x00000001, 0x00000001 },	/* MIF3_HCHGEN_CLKMUX */
	{  92, 0, 0x10010850, 0x0000001f, 0x00000000 },	/* PERIS_HCHGEN_CLKMUX */
	{  93, 0x4004, 0x13e20200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG_EN in pd_abox */
	{  94, 0, 0x16420200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG_EN */
	{  95, 0x4044, 0x17c20200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG_EN in pd_dbg */
	{  96, 0, 0x15c20200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG_EN */
	{  97, 0, 0x15a20200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG_EN */
	{  98, 0x4024, 0x12c20200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG2_EN in pd_cam */
	{  99, 0, 0x16820200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG_EN */
	{ 100, 0, 0x16920200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG2_EN */
	{ 101, 0x4044, 0x17c20200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG_EN in pd_dbg */
	{ 102, 0x5224, 0x12e20200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG_EN in pd_dcam */
	{ 103, 0x4064, 0x12820200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG_EN in pd_dpu0 */
	{ 104, 0x4084, 0x12a20200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG_EN in pd_dpu1 */
	{ 105, 0x40a4, 0x13420200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG_EN in pd_dsp */
	{ 106, 0, 0x11020200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG_EN */
	{ 107, 0, 0x11420200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG_EN */
	{ 108, 0x40c4, 0x13a20200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG_EN in pd_g2d */
	{ 109, 0x40e4, 0x13820200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG_EN in pd_g3d */
	{ 110, 0, 0x15e20200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG_EN */
	{ 111, 0x4104, 0x13120200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG_EN in pd_isphq */
	{ 112, 0x4124, 0x13020200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG_EN in pd_isplp */
	{ 113, 0x4144, 0x13520200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG_EN in pd_iva */
	{ 114, 0x4164, 0x13c20200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG_EN in pd_mfc */
	{ 115, 0, 0x16020200, 0xffffffff, 0xffffefff },	/* BUS_COMPONENT_DRCG_EN */
	{ 116, 0, 0x16120200, 0xffffffff, 0xffffefff },	/* BUS_COMPONENT_DRCG_EN */
	{ 117, 0, 0x16220200, 0xffffffff, 0xffffefff },	/* BUS_COMPONENT_DRCG_EN */
	{ 118, 0, 0x16320200, 0xffffffff, 0xffffefff },	/* BUS_COMPONENT_DRCG_EN */
	{ 119, 0, 0x10420200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG_EN */
	{ 120, 0, 0x10820200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG_EN */
	{ 121, 0, 0x10030200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG_EN */
	{ 122, 0x5424, 0x14220200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG_EN in pd_srdz */
	{ 123, 0x4184, 0x13220200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG_EN in pd_vpu */
	{ 124, 0, 0x15830200, 0xffffffff, 0xfffffff0 },	/* BUS_COMPONENT_DRCG_EN */
	{ 125, 0x41a4, 0x14020200, 0xffffffff, 0xffffffff },	/* BUS_COMPONENT_DRCG_EN in pd_vts */
	{ 126, 0, 0x16483028, 0x20000000, 0x00000000 },	/* PAD_RETENTION_MIF_OPTION */
	{ 127, 0, 0x16482c48, 0xc0000000, 0xc0000000 },	/* TOP_PWR_OPTION */
	{ 128, 0, 0x16482cc8, 0xc0000000, 0xc0000000 },	/* TOP_PWR_MIF_OPTION */
	{ 129, 0, 0x16480220, 0xffffffff, 0x80640066 },	/* SEQ_TRANSITION0 */
	{ 130, 0, 0x16480224, 0xffffffff, 0x80690065 },	/* SEQ_TRANSITION1 */
	{ 131, 0, 0x16480228, 0xffffffff, 0x80650080 },	/* SEQ_TRANSITION2 */
	{ 132, 0, 0x16480260, 0xffffffff, 0x80640066 },	/* SEQ_MIF_TRANSITION0 */
	{ 133, 0, 0x16480264, 0xffffffff, 0x80690065 },	/* SEQ_MIF_TRANSITION1 */
	{ 134, 0, 0x16480268, 0xffffffff, 0x80650080 },	/* SEQ_MIF_TRANSITION2 */
	{ 135, 0, 0x1648026c, 0xffffffff, 0x80010003 },	/* SEQ_MIF_TRANSITION3 */
	{ 136, 0, 0x16480a00, 0x00000001, 0x00000001 },	/* PMU_DEBUG */
	{ 137, 0, 0x16480a0c, 0x00000001, 0x00000001 },	/* PMU_DEBUG1 */
	{ 138, 0, 0x16482968, 0x00000001, 0x00000001 },	/* RESET_CMU_MIF_OPTION */
	{ 139, 0, 0x16482d90, 0x00ff0000, 0x00330000 },	/* LOGIC_RESET_MIF_DURATION0 */
	{ 140, 0, 0x16488250, 0x00ff0000, 0x00330000 },	/* LOGIC_RESET_APM_DURATION0 */
	{ 141, 0, 0x16482918, 0x000f0000, 0x00010000 },	/* CLKSTOP_CMU_MIF_DURATION2 */
	{ 142, 0, 0x16488178, 0x000f0000, 0x00010000 },	/* CLKSTOP_CMU_APM_DURATION2 */
	{ 143, 0, 0x16488198, 0x000f0000, 0x00010000 },	/* CLKSTOP_OPEN_CMU_APM_DURATION2 */
	{ 144, 0, 0x16482608, 0x00010000, 0x00010000 },	/* CLUSTER0_L2_OPTION */
	{ 145, 0, 0x16482628, 0x00010000, 0x00010000 },	/* CLUSTER1_L2_OPTION */
};

#define L1_PMU_PA	0x16480000	/* S:arch/arm64/boot/dts/exynos/exynos8895.dtsi:301 */

static void __iomem *l1_pmu;		/* of_iomap of the PMU node, kept for the "L1" line */
static u32 l1_pmu0[ARRAY_SIZE(l1_pmu_off)];
static u32 l1_pd0[ARRAY_SIZE(l1_pd)][3];
static unsigned int l1_lpm_chk, l1_lpm_mis, l1_lpm_skp;
static bool l1_early_done;

static u32 __init l1_lpm_read(const struct l1_lpm *e, bool *skip)
{
	void __iomem *p;
	u32 v;

	*skip = false;
	if (e->pa >= L1_PMU_PA && e->pa < L1_PMU_PA + L1_PMU_SIZE)
		return readl(l1_pmu + (e->pa - L1_PMU_PA));
	if (e->pd_st && (readl(l1_pmu + e->pd_st) & 0xf) != 0xf) {
		*skip = true;
		return 0;
	}
	p = ioremap(e->pa & PAGE_MASK, PAGE_SIZE);
	if (!p) {
		*skip = true;
		return 0;
	}
	v = readl(p + (e->pa & ~PAGE_MASK));
	iounmap(p);
	return v;
}

static int __init l1_early(void)
{
	struct device_node *np, *pmu;
	unsigned int i;
	bool skip;
	u32 v;

	np = of_find_compatible_node(NULL, NULL, "greatlte,l1-selftest");
	if (!np)
		return 0;
	if (!of_device_is_available(np))
		goto out;
	pmu = of_parse_phandle(np, "samsung,pmu-syscon", 0);
	if (!pmu) {
		pr_err("L1pmu0 !-19\n");
		goto out;
	}
	l1_pmu = of_iomap(pmu, 0);
	of_node_put(pmu);
	if (!l1_pmu) {
		pr_err("L1pmu0 !-12\n");
		goto out;
	}

	for (i = 0; i < ARRAY_SIZE(l1_pmu_off); i++)
		l1_pmu0[i] = readl(l1_pmu + l1_pmu_off[i]);
	pr_err("L1pmu0 rst_stat=%08x inform2=%08x inform3=%08x ps_hold=%08x rsc=%08x wdtdis=%08x swdis=%08x\n",
	       l1_pmu0[0], l1_pmu0[1], l1_pmu0[2], l1_pmu0[3], l1_pmu0[4], l1_pmu0[5], l1_pmu0[6]);

	for (i = 0; i < ARRAY_SIZE(l1_pd); i++) {
		l1_pd0[i][0] = readl(l1_pmu + l1_pd[i].conf);
		l1_pd0[i][1] = readl(l1_pmu + l1_pd[i].conf + L1_PD_STATUS);
		l1_pd0[i][2] = readl(l1_pmu + l1_pd[i].conf + L1_PD_OPTION);
		pr_err("L1pd0 %s cfg=%08x st=%08x opt=%08x\n", l1_pd[i].label,
		       l1_pd0[i][0], l1_pd0[i][1], l1_pd0[i][2]);
	}

	for (i = 0; i < ARRAY_SIZE(l1_lpm); i++) {
		const struct l1_lpm *e = &l1_lpm[i];

		v = l1_lpm_read(e, &skip);
		if (skip) {
			l1_lpm_skp++;
			continue;
		}
		l1_lpm_chk++;
		if ((v & e->mask) == e->val)
			continue;
		l1_lpm_mis++;
		if (l1_lpm_lines)
			pr_err("L1lpm0 :%u %08x val=%08x want=%08x/%08x\n",
			       e->line, e->pa, v, e->mask, e->val);
	}
	pr_err("L1lpm0 checked=%u mismatch=%u skipped=%u\n",
	       l1_lpm_chk, l1_lpm_mis, l1_lpm_skp);
	WRITE_ONCE(l1_early_done, true);
out:
	of_node_put(np);
	return 0;
}
early_initcall(l1_early);

/* ---- probe and the post-boot snapshot ---- */

#define L1_MAX_CMU	16
#define L1_SNAP_TEXT	1024

static struct {
	struct device *dev;
	struct regmap *pmu;
	struct clk_bulk_data *clks;
	int nclks;
	void __iomem *cmu[L1_MAX_CMU];
	struct device_node *cmu_np[L1_MAX_CMU];
	unsigned int ncmu;
	struct device_node *uart;
} l1;

static char l1_snap[L1_SNAP_TEXT];	/* "L1id" records of the snapshot, written once */
static bool l1_snap_done;

static void l1_cpu_line(unsigned int cpu)
{
	u64 mpidr = cpu_logical_map(cpu);
	/* CLUSTERx_CPUy_STATUS = 0x2004 + ((aff1 << 2) | aff0) * 0x80: S:drivers/soc/samsung/exynos-pmu.c:54-61 */
	u32 off = 0x2004 + ((MPIDR_AFFINITY_LEVEL(mpidr, 1) << 2) |
			    MPIDR_AFFINITY_LEVEL(mpidr, 0)) * 0x80;
	u32 v = 0;
	int r = regmap_read(l1.pmu, off, &v);

	if (r)
		pr_err("L1cpu %u mpidr=%llx st=!%d\n", cpu, mpidr, r);
	else
		pr_err("L1cpu %u mpidr=%llx st=%08x\n", cpu, mpidr, v);
}

/*
 * PLL_CON0 words: CMU_TOP PLL_CON0_PLL_SHARED0..4 (A:drivers/clk/samsung/clk-exynos8895.c:37-41),
 * CMU_CPUCL0/1 PLL_CON0_PLL_CPUCL0/1 at 0x120 (spec Appendix B)
 */
static const struct {
	const char *name;
	const char *cmu;	/* node name@unit of the CMU in greatlte,cmus */
	u32 off;
} l1_raw[] = {
	{ "shared0", "clock-controller@15a80000", 0x120 },
	{ "shared1", "clock-controller@15a80000", 0x140 },
	{ "shared2", "clock-controller@15a80000", 0x160 },
	{ "shared3", "clock-controller@15a80000", 0x180 },
	{ "shared4", "clock-controller@15a80000", 0x1a0 },
	{ "cpucl0", "clock-controller@16800000", 0x120 },
	{ "cpucl1", "clock-controller@16900000", 0x120 },
};

static int l1_raw_read(unsigned int i, u32 *v)
{
	unsigned int k;

	for (k = 0; k < l1.ncmu; k++) {
		if (!of_node_name_eq(l1.cmu_np[k], "clock-controller") ||
		    strcmp(kbasename(l1.cmu_np[k]->full_name), l1_raw[i].cmu))
			continue;
		if (!l1.cmu[k])
			return -ENOMEM;
		*v = readl(l1.cmu[k] + l1_raw[i].off);
		return 0;
	}
	return -ENODEV;
}

static bool l1_dev_bound(struct device_node *np, bool *avail)
{
	struct platform_device *pdev = of_find_device_by_node(np);
	bool b = false;

	*avail = of_device_is_available(np);
	if (pdev) {
		b = device_is_bound(&pdev->dev);
		put_device(&pdev->dev);
	}
	return b;
}

static void l1_snapshot(void)
{
	char *s = l1_snap;
	size_t n = sizeof(l1_snap), len = 0;
	struct device_node *np;
	u32 v[ARRAY_SIZE(l1_pmu_off)], pd_on = 0, w;
	unsigned int i, npc = 0, nav = 0, nb = 0;
	bool avail, b;
	int r;

	for (i = 0; i < ARRAY_SIZE(l1_pmu_off); i++)
		if (regmap_read(l1.pmu, l1_pmu_off[i], &v[i]))
			v[i] = 0xdeadbeef;
	pr_err("L1pmu sys=%d up=%llu rst_stat=%08x inform2=%08x inform3=%08x ps_hold=%08x rsc=%08x wdtdis=%08x swdis=%08x\n",
	       system_state, ktime_get_boottime_ns() / NSEC_PER_MSEC,
	       v[0], v[1], v[2], v[3], v[4], v[5], v[6]);
	len += scnprintf(s + len, n - len, "snap up=%llu rst=%08x inf2=%08x inf3=%08x psh=%08x",
			 ktime_get_boottime_ns() / NSEC_PER_MSEC, v[0], v[1], v[2], v[3]);

	for (i = 0; i < ARRAY_SIZE(l1_pd); i++) {
		u32 c = 0, st = 0, o = 0;

		regmap_read(l1.pmu, l1_pd[i].conf, &c);
		regmap_read(l1.pmu, l1_pd[i].conf + L1_PD_STATUS, &st);
		regmap_read(l1.pmu, l1_pd[i].conf + L1_PD_OPTION, &o);
		pr_err("L1pd %s cfg=%08x st=%08x opt=%08x\n", l1_pd[i].label, c, st, o);
		if ((st & 0xf) == 0xf)
			pd_on |= BIT(i);
	}
	len += scnprintf(s + len, n - len, " pd=%04x\nclk", pd_on);

	for (i = 0; i < l1.nclks; i++) {
		unsigned long rate = clk_get_rate(l1.clks[i].clk);
		bool en = __clk_is_enabled(l1.clks[i].clk);

		pr_err("L1clk %s rate=%lu en=%d\n", l1.clks[i].id, rate, en);
		len += scnprintf(s + len, n - len, " %s=%lu:%d", l1.clks[i].id, rate, en);
	}
	if (!l1.nclks)
		len += scnprintf(s + len, n - len, " n=0");
	len += scnprintf(s + len, n - len, "\nraw");

	for (i = 0; i < ARRAY_SIZE(l1_raw); i++) {
		r = l1_raw_read(i, &w);
		if (r) {
			pr_err("L1raw %s con0=!%d\n", l1_raw[i].name, r);
			len += scnprintf(s + len, n - len, " %s=!%d", l1_raw[i].name, r);
		} else {
			pr_err("L1raw %s con0=%08x\n", l1_raw[i].name, w);
			len += scnprintf(s + len, n - len, " %s=%08x", l1_raw[i].name, w);
		}
	}

	for_each_possible_cpu(i)
		l1_cpu_line(i);

	for_each_compatible_node(np, NULL, "samsung,exynos8895-pinctrl") {
		b = l1_dev_bound(np, &avail);
		pr_err("L1pinctrl %pOFP bound=%d avail=%d\n", np, b, avail);
		npc++;
		nav += avail;
		nb += b;
	}
	len += scnprintf(s + len, n - len, "\nbound pinctrl=%u/%u/%u", nb, nav, npc);

	if (l1.uart) {
		b = l1_dev_bound(l1.uart, &avail);
		pr_err("L1dev %pOFP bound=%d\n", l1.uart, b);
		len += scnprintf(s + len, n - len, " uart=%d", b);
	} else {
		len += scnprintf(s + len, n - len, " uart=na");
	}
	/* publish l1_snap to l1_id_fmt(), which runs in the hb thread */
	smp_store_release(&l1_snap_done, true);
}

static void l1_trigger(void)
{
	if (l1_reboot) {
		pr_err("L1trig reboot=%s\n", l1_reboot);
		kernel_restart(*l1_reboot ? l1_reboot : NULL);
	}
	if (l1_panic)
		panic("L1trig panic");
	if (l1_poweroff) {
		pr_err("L1trig poweroff\n");
		kernel_power_off();
	}
}

static int l1_spin(void *arg)
{
	for (;;)
		cpu_relax();
	return 0;
}

static void l1_work_fn(struct work_struct *w);
static DECLARE_DELAYED_WORK(l1_work, l1_work_fn);
static unsigned int l1_phase;	/* 0: wait for SYSTEM_RUNNING, 1: +5 s, 2: triggers at 60 s */

static void l1_work_fn(struct work_struct *w)
{
	u64 up = ktime_get_boottime_ns() / NSEC_PER_MSEC;

	switch (l1_phase) {
	case 0:
		if (system_state != SYSTEM_RUNNING) {
			schedule_delayed_work(&l1_work, HZ);
			return;
		}
		l1_phase = 1;
		schedule_delayed_work(&l1_work, 5 * HZ);
		return;
	case 1:
		if (l1.pmu)
			l1_snapshot();
		l1_phase = 2;
		fallthrough;
	default:
		if (!l1_reboot && !l1_panic && !l1_poweroff && !l1_wdt_hang)
			return;
		if (up < 60000) {
			schedule_delayed_work(&l1_work, msecs_to_jiffies(60000 - up));
			return;
		}
		l1_trigger();
		if (l1_wdt_hang) {
			pr_err("L1trig wdt_hang\n");
			stop_machine(l1_spin, NULL, cpu_online_mask);
		}
	}
}

static int l1_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	unsigned int i;
	int n;

	l1.pmu = syscon_regmap_lookup_by_phandle(np, "samsung,pmu-syscon");
	if (IS_ERR(l1.pmu))
		return dev_err_probe(dev, PTR_ERR(l1.pmu), "samsung,pmu-syscon\n");

	n = devm_clk_bulk_get_all(dev, &l1.clks);
	if (n < 0)
		return dev_err_probe(dev, n, "clocks\n");
	l1.nclks = n;

	n = of_count_phandle_with_args(np, "greatlte,cmus", NULL);
	for (i = 0; n > 0 && i < n && i < L1_MAX_CMU; i++) {
		l1.cmu_np[i] = of_parse_phandle(np, "greatlte,cmus", i);
		if (l1.cmu_np[i])
			l1.cmu[i] = of_iomap(l1.cmu_np[i], 0);	/* read only, non-requesting */
	}
	l1.ncmu = i;
	l1.uart = of_parse_phandle(np, "greatlte,uart", 0);
	l1.dev = dev;
	return 0;
}

static const struct of_device_id l1_of_match[] = {
	{ .compatible = "greatlte,l1-selftest" },
	{ }
};

static struct platform_driver l1_driver = {
	.probe = l1_probe,
	.driver = {
		.name = "greatlte-l1-selftest",
		.of_match_table = l1_of_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(l1_driver);

/* ---- heartbeat printers (T_tooling.md 5.7.6; layer 1) ---- */

/* "L1": fast, hb thread; PMU is an always-on block, readl() allowed (5.7.6 rule 4) */
static int l1_hb_fmt(char *buf, size_t size)
{
	u32 pd_on = 0;
	unsigned int i;

	if (!l1_pmu)
		return scnprintf(buf, size, "pd_on=na rst=na");
	for (i = 0; i < ARRAY_SIZE(l1_pd); i++)
		if ((readl(l1_pmu + l1_pd[i].conf + L1_PD_STATUS) & 0xf) == 0xf)
			pd_on |= BIT(i);
	return scnprintf(buf, size, "pd_on=%04x rst=%08x", pd_on, readl(l1_pmu + 0x0404));
}

/* "L1id": stored values only (no MMIO), every 300 s: hand-off + post-boot snapshot */
static int l1_id_fmt(char *buf, size_t size)
{
	size_t len = 0;
	unsigned int i;

	if (!READ_ONCE(l1_early_done))
		return scnprintf(buf, size, "pmu0=na\npd0=na\nlpm0=na\nsnap=na");
	len += scnprintf(buf + len, size - len,
			 "pmu0 rst=%08x inf2=%08x inf3=%08x psh=%08x rsc=%08x wd=%08x sw=%08x\npd0",
			 l1_pmu0[0], l1_pmu0[1], l1_pmu0[2], l1_pmu0[3], l1_pmu0[4],
			 l1_pmu0[5], l1_pmu0[6]);
	for (i = 0; i < ARRAY_SIZE(l1_pd); i++)
		len += scnprintf(buf + len, size - len, " %s=%x/%x/%x", l1_pd[i].label,
				 l1_pd0[i][0], l1_pd0[i][1], l1_pd0[i][2]);
	len += scnprintf(buf + len, size - len, "\nlpm0 chk=%u mis=%u skp=%u\n",
			 l1_lpm_chk, l1_lpm_mis, l1_lpm_skp);
	/* pairs with the smp_store_release() at the end of l1_snapshot() */
	if (smp_load_acquire(&l1_snap_done))
		len += scnprintf(buf + len, size - len, "%s", l1_snap);
	else
		len += scnprintf(buf + len, size - len, "snap=na");
	return len;
}

static struct greatlte_hb_printer l1_hb = {
	.name = "L1",
	.layer = 1,
	.fmt = l1_hb_fmt,
};

static struct greatlte_hb_printer l1_id = {
	.name = "L1id",
	.layer = 1,
	.every_ms = 300000,
	.fmt = l1_id_fmt,
};

static int __init l1_arm(void)
{
	struct device_node *np = of_find_compatible_node(NULL, NULL, "greatlte,l1-selftest");
	bool on = np && of_device_is_available(np);

	of_node_put(np);
	if (!on)
		return 0;
	l1_hb.every_ms = l1_hb_ms;
	l1_id.every_ms = l1_id_ms;
	greatlte_hb_register(&l1_hb);
	greatlte_hb_register(&l1_id);
	schedule_delayed_work(&l1_work, HZ);
	return 0;
}
late_initcall_sync(l1_arm);
