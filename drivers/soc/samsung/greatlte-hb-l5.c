// SPDX-License-Identifier: GPL-2.0-only
/*
 * greatlte L5 (display) heartbeat printers. DEBUG ONLY, never upstream.
 * Spec: rewrite/spec/L5_display.md 5.9 (fields), 8.2 (expected values), through
 * the heartbeat hooks of T_tooling.md 5.7.6 (contract items 1-6): both printers
 * are static, registered once from this file's late_initcall, layer 5, slow
 * (they do MMIO, so they run in hb-probe; if a read hangs a core, the other
 * CPUs keep printing and that CPU's tk freezes, R-HB2). Every call prints the
 * same keys in the same order; an unreadable value prints "na".
 *
 * "dpu" every greatlte_hb_l5.dpu_ms (default 120000): the raw registers S-Boot
 * left (stage 5a) and their later state. ioremap() only, never
 * request_mem_region(), so it cannot conflict with the drivers:
 *   pd0 pd1 c0 c1 iso tg | gt gw g1s | gc tr d0 d1 d2 ip sh dsc | g0e g0c g0y dmav |
 *   m1c m1f m1p m1v m1a m1b | ls3 dps cfg pll | lp dl te drcg0 drcg1 | drv
 * PMU and CMU_TOP are always on. With pd0 != 0xf every other key is "na" (a
 * read of an unpowered DPU block hangs the bus). An IP is read unless its
 * CMU_DPU0 gate is manual with CG_VAL 0 (gate code & 3 == 1; S:drivers/soc/
 * samsung/cal-if/ra.c:381-384 "MANUAL(status) 1 : CG_VALUE(offset) control /
 * 0 : ENABLE_AUTOMATIC_CLKGATING(enable) control"); the DSIM also needs pd1.
 * greatlte_hb_l5.ip=0 turns every IP read into "na". Once a driver is bound to
 * the DECON device (drv=1), the DECON, DMA, SysMMU and DSIM keys are "na":
 * "dpuc" reads them under the drivers' runtime PM.
 *
 * "dpuc" every greatlte_hb_l5.dpuc_ms (default 30000): the drivers' counters
 * (include/linux/soc/samsung/greatlte-l5.h), live only for built-in drivers:
 *   fd fs to rc er ude dpe tec he hx ho rpm gc tr g0y d0 m1c m1f | ur ecc st ls3 dps |
 *   bt pw dd lt var
 */
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/gpio/consumer.h>
#include <linux/soc/samsung/greatlte-hb.h>
#include <linux/soc/samsung/greatlte-l5.h>

struct greatlte_l5_stats greatlte_l5;

static unsigned int l5_dpu_ms = 120000;
module_param_named(dpu_ms, l5_dpu_ms, uint, 0444);
MODULE_PARM_DESC(dpu_ms, "period of the dpu printer (ms)");

static unsigned int l5_dpuc_ms = 30000;
module_param_named(dpuc_ms, l5_dpuc_ms, uint, 0444);
MODULE_PARM_DESC(dpuc_ms, "period of the dpuc printer (ms)");

static bool l5_ip = true;
module_param_named(ip, l5_ip, bool, 0644);
MODULE_PARM_DESC(ip, "read the DPU IP registers in dpu (0: print na)");

/*
 * Blocks and offsets (spec 5.9, 8.2):
 * PMU 0x16480000: DPU0 CONFIGURATION/STATUS +0x4060/+0x4064, DPU1 +0x4080/+0x4084
 *   (S:drivers/soc/samsung/cal-if/exynos8895/pmucal_local_exynos8895.h:199-200,243-244),
 *   MIPI D-PHY isolation +0x710 bit 0 (S:arch/arm64/boot/dts/exynos/exynos8895.dtsi:2916).
 * CMU_TOP 0x15a80000 GATE_CLKCMU_DPU_BUS +0x2054 (CAL:cmucal-sfr.c:13,514).
 * CMU_DPU0 0x12800000 gates (CAL:cmucal-sfr.c:19): DECON0 ACLK +0x207c (:971), DPU_DMA
 *   ACLK_CH1 +0x2090 (:1006), ACLK_SFR +0x209c (:1010), SYSMMU_DPUD1 ACLK +0x20ec (:1024),
 *   PCLK_NONSECURE +0x20f0 (:985), AD_APB_MIPI_DSIM0 PCLKS +0x204c (:994), SYSREG_DPU0
 *   PCLK +0x2108 (:978); gate fields MANUAL 20, CG_VAL 21, ENABLE_AUTOMATIC_CLKGATING 28
 *   (CAL:cmucal-sfr.c:4815-4817).
 * CMU_DPU1 0x12a00000 SYSREG_DPU1 PCLK +0x2048 (CAL:cmucal-sfr.c:20,1064).
 * DECON0 0x12860000 (FDT:7530): GLOBAL_CONTROL +0x0, INTERRUPT_PENDING +0x4c,
 *   SHADOW_REG_UPDATE_REQ +0x60, HW_SW_TRIG_CONTROL +0x70, DATA_PATH_CONTROL_0/1/2
 *   +0x214/+0x218/+0x230, DSC0 CONTROL0 +0x4000 (S:drivers/video/fbdev/exynos/dpu/
 *   regs-decon.h:129,172,187,195,518,530).
 * DPU_DMA 0x128b0000: DPU_DMA_VERSION +0x0; G0 at +0x1000: IDMA_ENABLE +0x0, IDMA_IN_CON
 *   +0x8, IDMA_IN_BASE_ADDR_Y +0x40 (S:drivers/video/fbdev/exynos/dpu/regs-dpp.h:24,69,
 *   102,178; FDT:7385).
 * SYSMMU_DPU1 0x12910000 (serves G0, FDT:7352,7387): CTRL +0x0, CFG +0x4, PT_BASE +0xc,
 *   VERSION +0x34, CAPA0/CAPA1 +0x870/+0x874 (S:drivers/iommu/exynos-iommu.h:100-103,131,
 *   150; A:drivers/iommu/exynos-iommu.c:170-171).
 * DSIM0 0x12870000 (FDT:7485): LINK_STATUS3 +0x14, DPHY_STATUS +0x1c, CONFIG +0x4c,
 *   PLLCTRL +0xa0 (S:drivers/video/fbdev/exynos/dpu/regs-dsim.h:32,42,137,316).
 * SYSREG_DPU0 0x12820000: DRCG +0x200 (S:.../pmucal_local_exynos8895.h:230), LPMUX
 *   +0x1000, DLMUX +0x1004, TE_SEL +0x100c (FDT:7465 base 0x12821000,
 *   S:drivers/video/fbdev/exynos/dpu/regs-decon.h:21,27,50).
 * SYSREG_DPU1 0x12a20000: DRCG +0x200 (S:.../pmucal_local_exynos8895.h:267).
 */
#define L5_PMU		0x16480000
#define L5_CMU_TOP	0x15a82000
#define L5_CMU_DPU0	0x12802000
#define L5_CMU_DPU1	0x12a02000
#define L5_DECON	0x12860000
#define L5_DMA		0x128b0000
#define L5_SMMU1	0x12910000
#define L5_DSIM		0x12870000
#define L5_SYSREG0	0x12820000
#define L5_SYSREG1	0x12a20000

enum { G_DECON, G_CH1, G_SFR, G_SMMU_A, G_SMMU_P, G_DSIM, G_SYSREG, G_NR };
static const u16 l5_gate_ofs[G_NR] = { 0x07c, 0x090, 0x09c, 0x0ec, 0x0f0, 0x04c, 0x108 };

static void __iomem *l5_pmu, *l5_top, *l5_dpu0, *l5_dpu1, *l5_decon, *l5_dma, *l5_smmu1,
		    *l5_dsim, *l5_sys0, *l5_sys1;
static struct device_node *l5_decon_np;

/* 3-bit gate code: bit 2 ENABLE_AUTOMATIC_CLKGATING (28), bit 1 CG_VAL (21), bit 0 MANUAL (20) */
static unsigned int l5_code(u32 w)
{
	return ((w >> 26) & 4) | ((w >> 20) & 3);
}

/* manual with CG_VAL 0: the IP clock is stopped, do not read */
static bool l5_gated(unsigned int code)
{
	return (code & 3) == 1;
}

static int l5_x(char *buf, size_t size, const char *key, void __iomem *base, u32 ofs, bool ok)
{
	if (!ok || !base)
		return scnprintf(buf, size, " %s=na", key);
	return scnprintf(buf, size, " %s=%08x", key, readl(base + ofs));
}

static bool l5_driver_bound(void)
{
	struct platform_device *pdev;
	bool bound;

	if (!l5_decon_np)
		return false;
	pdev = of_find_device_by_node(l5_decon_np);
	if (!pdev)
		return false;
	bound = READ_ONCE(pdev->dev.driver) != NULL;
	put_device(&pdev->dev);
	return bound;
}

static int l5_dpu_fmt(char *buf, size_t size)
{
	unsigned int code[G_NR], g1s = 0;
	bool pd0, pd1, drv = l5_driver_bound(), ip;
	u32 gw[G_NR], w;
	int len, i;

	w = readl(l5_pmu + 0x4064) & 0xf;
	pd0 = w == 0xf;
	len = scnprintf(buf, size, "pd0=%x", w);
	w = readl(l5_pmu + 0x4084) & 0xf;
	pd1 = w == 0xf;
	len += scnprintf(buf + len, size - len, " pd1=%x c0=%08x c1=%08x iso=%u tg=%u", w,
			 readl(l5_pmu + 0x4060), readl(l5_pmu + 0x4080),
			 readl(l5_pmu + 0x710) & 1, l5_code(readl(l5_top + 0x054)));

	if (pd0) {
		for (i = 0; i < G_NR; i++) {
			gw[i] = readl(l5_dpu0 + l5_gate_ofs[i]);
			code[i] = l5_code(gw[i]);
		}
		len += scnprintf(buf + len, size - len, " gt=");
		for (i = 0; i < G_NR; i++)
			len += scnprintf(buf + len, size - len, "%u", code[i]);
		len += scnprintf(buf + len, size - len, " gw=");
		for (i = 0; i < G_NR; i++)
			len += scnprintf(buf + len, size - len, "%s%08x", i ? "/" : "", gw[i]);
		if (pd1) {
			g1s = l5_code(readl(l5_dpu1 + 0x048));
			len += scnprintf(buf + len, size - len, " g1s=%u", g1s);
		} else {
			len += scnprintf(buf + len, size - len, " g1s=na");
		}
	} else {
		memset(code, 0, sizeof(code));
		len += scnprintf(buf + len, size - len, " gt=na gw=na g1s=na");
	}

	/* DECON, DMA, SysMMU, DSIM: only before a driver owns the DECON */
	ip = pd0 && l5_ip && !drv;
	len += l5_x(buf + len, size - len, "gc", l5_decon, 0x000, ip && !l5_gated(code[G_DECON]));
	len += l5_x(buf + len, size - len, "tr", l5_decon, 0x070, ip && !l5_gated(code[G_DECON]));
	len += l5_x(buf + len, size - len, "d0", l5_decon, 0x214, ip && !l5_gated(code[G_DECON]));
	len += l5_x(buf + len, size - len, "d1", l5_decon, 0x218, ip && !l5_gated(code[G_DECON]));
	len += l5_x(buf + len, size - len, "d2", l5_decon, 0x230, ip && !l5_gated(code[G_DECON]));
	len += l5_x(buf + len, size - len, "ip", l5_decon, 0x04c, ip && !l5_gated(code[G_DECON]));
	len += l5_x(buf + len, size - len, "sh", l5_decon, 0x060, ip && !l5_gated(code[G_DECON]));
	len += l5_x(buf + len, size - len, "dsc", l5_decon, 0x4000, ip && !l5_gated(code[G_DECON]));
	len += l5_x(buf + len, size - len, "g0e", l5_dma, 0x1000, ip && !l5_gated(code[G_CH1]));
	len += l5_x(buf + len, size - len, "g0c", l5_dma, 0x1008, ip && !l5_gated(code[G_CH1]));
	len += l5_x(buf + len, size - len, "g0y", l5_dma, 0x1040, ip && !l5_gated(code[G_CH1]));
	len += l5_x(buf + len, size - len, "dmav", l5_dma, 0x0000, ip && !l5_gated(code[G_SFR]));
	ip = ip && !l5_gated(code[G_SMMU_A]) && !l5_gated(code[G_SMMU_P]);
	len += l5_x(buf + len, size - len, "m1c", l5_smmu1, 0x000, ip);
	len += l5_x(buf + len, size - len, "m1f", l5_smmu1, 0x004, ip);
	len += l5_x(buf + len, size - len, "m1p", l5_smmu1, 0x00c, ip);
	len += l5_x(buf + len, size - len, "m1v", l5_smmu1, 0x034, ip);
	len += l5_x(buf + len, size - len, "m1a", l5_smmu1, 0x870, ip);
	/* CAPA1 exists only if CAPA0 bit 11 says so (A:drivers/iommu/exynos-iommu.c:502-507);
	 * on 8895 a read of +0x874 without it is a synchronous external abort (image 0043) */
	len += l5_x(buf + len, size - len, "m1b", l5_smmu1, 0x874,
		    ip && (readl(l5_smmu1 + 0x870) & BIT(11)));
	ip = pd0 && pd1 && l5_ip && !drv && !l5_gated(code[G_DSIM]);
	len += l5_x(buf + len, size - len, "ls3", l5_dsim, 0x14, ip);
	len += l5_x(buf + len, size - len, "dps", l5_dsim, 0x1c, ip);
	len += l5_x(buf + len, size - len, "cfg", l5_dsim, 0x4c, ip);
	len += l5_x(buf + len, size - len, "pll", l5_dsim, 0xa0, ip);

	/* sysregs: kept after the driver binds */
	ip = pd0 && l5_ip && !l5_gated(code[G_SYSREG]);
	len += l5_x(buf + len, size - len, "lp", l5_sys0, 0x1000, ip);
	len += l5_x(buf + len, size - len, "dl", l5_sys0, 0x1004, ip);
	len += l5_x(buf + len, size - len, "te", l5_sys0, 0x100c, ip);
	len += l5_x(buf + len, size - len, "drcg0", l5_sys0, 0x0200, ip);
	len += l5_x(buf + len, size - len, "drcg1", l5_sys1, 0x0200,
		    pd0 && pd1 && l5_ip && !l5_gated(g1s));
	len += scnprintf(buf + len, size - len, " drv=%d", drv);

	return len;
}

static int l5_cnt(char *buf, size_t size, const char *key, atomic_t *v)
{
	return scnprintf(buf, size, " %s=%d", key, atomic_read(v));
}

static int l5_dpuc_fmt(char *buf, size_t size)
{
	struct greatlte_l5_stats *s = &greatlte_l5;
	struct device *dev = READ_ONCE(s->decon_dev);
	void __iomem *regs = READ_ONCE(s->decon_regs), *dma = READ_ONCE(s->decon_dma);
	const unsigned int *state = READ_ONCE(s->dsim_state);
	struct gpio_desc *det = READ_ONCE(s->det);
	const char *var = READ_ONCE(s->var);
	int len, act;

	len = scnprintf(buf, size, "fd=%d", atomic_read(&s->fd));
	len += l5_cnt(buf + len, size - len, "fs", &s->fs);
	len += l5_cnt(buf + len, size - len, "to", &s->to);
	len += l5_cnt(buf + len, size - len, "rc", &s->rc);
	len += l5_cnt(buf + len, size - len, "er", &s->er);
	len += l5_cnt(buf + len, size - len, "ude", &s->ude);
	len += l5_cnt(buf + len, size - len, "dpe", &s->dpe);
	len += l5_cnt(buf + len, size - len, "tec", &s->tec);
	len += l5_cnt(buf + len, size - len, "he", &s->he);
	len += l5_cnt(buf + len, size - len, "hx", &s->hx);
	if (dev) {
		len += scnprintf(buf + len, size - len, " ho=%d rpm=%s", READ_ONCE(s->ho),
				 pm_runtime_active(dev) ? "active" : "suspended");
		act = pm_runtime_get_if_active(dev);
	} else {
		len += scnprintf(buf + len, size - len, " ho=na rpm=na");
		act = 0;
	}
	len += l5_x(buf + len, size - len, "gc", regs, 0x000, act > 0);
	len += l5_x(buf + len, size - len, "tr", regs, 0x070, act > 0);
	len += l5_x(buf + len, size - len, "g0y", dma, 0x1040, act > 0);
	len += l5_x(buf + len, size - len, "d0", regs, 0x214, act > 0);
	/* the SysMMUs are resumed with the DECON (device links) */
	len += l5_x(buf + len, size - len, "m1c", l5_smmu1, 0x000, act > 0);
	len += l5_x(buf + len, size - len, "m1f", l5_smmu1, 0x004, act > 0);
	if (act > 0)
		pm_runtime_put(dev);

	len += l5_cnt(buf + len, size - len, "ur", &s->ur);
	len += l5_cnt(buf + len, size - len, "ecc", &s->ecc);
	dev = READ_ONCE(s->dsim_dev);
	len += state ? scnprintf(buf + len, size - len, " st=%x", READ_ONCE(*state)) :
		       scnprintf(buf + len, size - len, " st=na");
	act = dev ? pm_runtime_get_if_active(dev) : 0;
	len += l5_x(buf + len, size - len, "ls3", READ_ONCE(s->dsim_regs), 0x14, act > 0);
	len += l5_x(buf + len, size - len, "dps", READ_ONCE(s->dsim_regs), 0x1c, act > 0);
	if (act > 0)
		pm_runtime_put(dev);

	if (READ_ONCE(s->panel))
		len += scnprintf(buf + len, size - len, " bt=%d pw=%d dd=%d lt=0x%06x var=%s",
				 READ_ONCE(s->bt), READ_ONCE(s->pw),
				 det ? gpiod_get_value(det) : -1, READ_ONCE(s->lt),
				 var ? var : "na");
	else
		len += scnprintf(buf + len, size - len, " bt=na pw=na dd=na lt=na var=na");

	return len;
}

static struct greatlte_hb_printer l5_dpu_prn = {
	.name = "dpu",
	.layer = 5,
	.slow = true,
	.fmt = l5_dpu_fmt,
};

static struct greatlte_hb_printer l5_dpuc_prn = {
	.name = "dpuc",
	.layer = 5,
	.slow = true,
	.fmt = l5_dpuc_fmt,
};

static int __init l5_hb_init(void)
{
	l5_decon_np = of_find_compatible_node(NULL, NULL, "samsung,exynos8895-decon");
	if (!l5_decon_np)
		return 0;	/* not an Exynos 8895 DT with the display */

	l5_pmu = ioremap(L5_PMU, 0x5000);
	l5_top = ioremap(L5_CMU_TOP, 0x1000);
	l5_dpu0 = ioremap(L5_CMU_DPU0, 0x1000);
	l5_dpu1 = ioremap(L5_CMU_DPU1, 0x1000);
	l5_decon = ioremap(L5_DECON, 0x5000);
	l5_dma = ioremap(L5_DMA, 0x2000);
	l5_smmu1 = ioremap(L5_SMMU1, 0x1000);
	l5_dsim = ioremap(L5_DSIM, 0x100);
	l5_sys0 = ioremap(L5_SYSREG0, 0x2000);
	l5_sys1 = ioremap(L5_SYSREG1, 0x1000);
	if (!l5_pmu || !l5_top || !l5_dpu0 || !l5_dpu1)
		return 0;	/* the always-read blocks are required */

	l5_dpu_prn.every_ms = l5_dpu_ms;
	l5_dpuc_prn.every_ms = l5_dpuc_ms;
	greatlte_hb_register(&l5_dpu_prn);
	greatlte_hb_register(&l5_dpuc_prn);
	return 0;
}
late_initcall(l5_hb_init);
