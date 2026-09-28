// SPDX-License-Identifier: GPL-2.0-only
/*
 * greatlte L3 (USB peripheral mode) heartbeat rows and printers. DEBUG ONLY, never upstream.
 * Spec: rewrite/spec/L3_usb.md 8.1 (fields and read order), 8.3 (R1-R11), 8.4 (stall shortcut),
 * 1.2 (P1, P2, P4, P10, P11), through the heartbeat hooks of T_tooling.md 5.7.3/5.7.6 as
 * extended by FULL_IMAGE_PLAN.md 2.2 T14 (rows from the layer's own DEBUG file, plan X-55).
 *
 * Rows, on the "hb: v1 L3f seq=<n> ..." line of every period:
 *   irq (interrupt count of the dwc3 core, P10), dwc3 glue phy (bound driver or U).
 * Printers (both slow: they run in hb-probe and do MMIO, T 5.7.6 rule 4):
 *   "usb" every greatlte_hb_l3.hb_ms (default 30000, REWRITE_PLAN.md 5 X-5, which replaces the
 *   10 s change-gated work of spec 8.1; the all-in image uses 120000, FULL_IMAGE_PLAN.md 3.4): the
 *   spec 8.1 field line in full every time (X-5: no change gate; its t= is dropped, the
 *   heartbeat's seq/up replace it)
 *     irq= irqdis= rpm= dwc=<ok|gated|gated-dp|gated-q> evsz= evcnt= dsts= ls= spd= dctl= dcfg= devten=
 *     gctl= gusb2= gusb3= guctl1= gsbus=<0>/<1> phy=<ok|gated|gated-q|isolated> v0= utmi= utmi1=
 *     ls8895= resume= clkrst= p0= test0=<w0>/<w1> refclk= pmu704= cg=<2078>/<207c>/<2080> cgd=<7 gates> qch=
 *     umux= gpio= port= muic=<19>,<1c>,<0d>,<0e>,<0f>|na
 *   "usbid" every greatlte_hb_l3.id_ms (default 120000, X-5; plan 3.4 "once": a huge id_ms):
 *     gsnpsid= hwp0= hwp1= hwp2= hwp3= grxthr= aclk= ref= susp= phyref= idle=<cpu0>/<cpu4>
 *     ver0= ver1= shift= port= inits= retunes= crto= cr1002= armed=
 *   (P1, P4, P11 and R2-R4, R8, R9, R11 values that survive the 16 KB console wrap).
 * All values hex without 0x unless they are counts or rates; an unreadable value prints !<errno>.
 *
 * Read order (spec 8.1; a gated or isolated block can hang the bus, F2): CMU_FSYS0 gate registers
 * first; the dwc3 is read only if its AXI gate (0x2078) and the seven data-path gates are open, the
 * PHY only if 0x2078 is open and PMU 0x704 bit 0 is set, and both only while the link Q-channel
 * QCH_CON 0x3044 has ENABLE or CLOCK_REQ set (else dwc=gated-q / phy=gated-q; guard (h) of the K9
 * hs usb block, REWRITE_PLAN.md 4.4). A gate counts as closed when it is in manual mode (bit 20)
 * with CG_VAL (bit 21) clear, as stock ra_get_gate() reads it: MANUAL 1 = CG_VAL decides, MANUAL 0
 * = the gate runs or auto-gates on demand (S:drivers/soc/samsung/cal-if/ra.c:404-418); the same
 * rule as the hs census (codes 1 and 5 gated). CMU_FSYS0 (pclk CLK_IS_CRITICAL) and the PMU are
 * always readable. Nothing is written anywhere; the MUIC is read with plain SMBus transfers on the
 * adapter (no client is registered, so the MAX77865 driver can still claim 0x25) and never its
 * clear-on-read interrupt registers 0x01-0x04.
 */
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/cpu.h>
#include <linux/cpuidle.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/kernel_stat.h>
#include <linux/mfd/syscon.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/soc/samsung/greatlte-hb.h>
#include <linux/soc/samsung/greatlte-l3.h>
#include <linux/string.h>

#define L3_GLUE_PATH	"/soc@0/usb@10c00000"
#define L3_DWC3_PATH	L3_GLUE_PATH "/usb@0"
#define L3_PHY_PATH	"/soc@0/phy@10e00000"
#define L3_MUIC_BUS	"/soc@0/usi@10960000/i2c@10000"	/* hsi2c at 0x10970000, FDT:6149-6166 */

/* dwc3 0x10c00000 size 0x10000 (FDT:7858); offsets from the xHCI base, A:drivers/usb/dwc3/io.h:24-29 */
#define L3_DWC3_BASE		0x10c00000
#define L3_DWC3_SIZE		0x10000
#define L3_GSBUSCFG0		0xc100		/* A:drivers/usb/dwc3/core.h:98 */
#define L3_GSBUSCFG1		0xc104		/* core.h:99 */
#define L3_GRXTHRCFG		0xc10c		/* core.h:101 */
#define L3_GCTL			0xc110		/* core.h:102 */
#define L3_GUCTL1		0xc11c		/* core.h:105 */
#define L3_GSNPSID		0xc120		/* core.h:106 */
#define L3_GHWPARAMS(n)		(0xc140 + 4 * (n))	/* core.h:114-117 */
#define L3_GUSB2PHYCFG0		0xc200		/* core.h:138 */
#define L3_GUSB3PIPECTL0	0xc2c0		/* core.h:143 */
#define L3_GEVNTSIZ0		0xc408		/* core.h:150 */
#define L3_GEVNTCOUNT0		0xc40c		/* core.h:151 */
#define L3_DCFG			0xc700		/* core.h:159 */
#define L3_DCTL			0xc704		/* core.h:160 */
#define L3_DEVTEN		0xc708		/* core.h:161 */
#define L3_DSTS			0xc70c		/* core.h:162 */
#define L3_DSTS_LS(v)		(((v) >> 18) & 0xf)	/* USBLNKST, core.h:523-524 */
#define L3_DSTS_SPD(v)		((v) & 0x7)		/* CONNECTSPD, core.h:531 */

/* PHY windows (FDT:7882) and the refclk select word 0x10e50070 + 0x0c (FDT:7901-7902) */
#define L3_PHY0_BASE		0x10e00000
#define L3_PHY1_BASE		0x10e10000
#define L3_PHY_SIZE		0x100
#define L3_REFCLK_BASE		0x10e50070
#define L3_REFCLK_SIZE		0x10
#define L3_REFCLK_CTRL		0x0c
/*
 * PHY registers after the +4 shift (S:drivers/phy/phy-samsung-usb3-cal.c:855-869);
 * cal.h = S:drivers/phy/phy-samsung-usb3-cal.h
 */
#define L3_PHY_LINKSYSTEM	0x04		/* cal.h:4 */
#define L3_PHY_PHYUTMI		0x08		/* cal.h:15 */
#define L3_PHY_PHYCLKRST	0x10		/* cal.h:33 */
#define L3_PHY_PHYPARAM0	0x1c		/* cal.h:70 */
#define L3_PHY_PHYTEST		0x28		/* cal.h:116 */
#define L3_PHY_PHYRESUME	0x34		/* cal.h:137 */

/* CMU_FSYS0: CAL:cmucal-sfr.c:22 "SFR_BLOCK(FSYS0, 0x11000000, 0x8000)" */
#define L3_CMU_BASE		0x11000000
#define L3_CMU_SIZE		0x4000
#define L3_CMU_UMUX		0x0180		/* PLL_CON0_MUX_CLKCMU_FSYS0_USBDRD30_USER, CAL:cmucal-sfr.c:1140 */
#define L3_CMU_QCH		0x3044		/* QCH_CON_USBTV_QCH_USB30DRD_LINK, CAL:cmucal-sfr.c:1198 */
#define L3_CG_MANUAL		BIT(20)		/* CAL:cmucal-sfr.c:5323 */
#define L3_CG_VAL		BIT(21)		/* CAL:cmucal-sfr.c:5322 */
#define L3_CG_HWACG		BIT(28)		/* ENABLE_AUTOMATIC_CLKGATING, CAL:cmucal-sfr.c:5324 */
/* QCH_CON ENABLE (bit 0) or CLOCK_REQ (bit 1) set: link clock not requested off, CAL:cmucal-sfr.c:5472-5473 */
#define L3_QCH_ON(q)		((q) & 0x3)

/* the three gates the USB nodes claim */
static const u16 l3_cg[] = {
	0x2078,		/* USBTV_I_USB30DRD_ACLK, CAL:cmucal-sfr.c:1174 */
	0x207c,		/* USBTV_I_USB30DRD_REF_CLK, CAL:cmucal-sfr.c:1168 */
	0x2080,		/* USBTV_I_USB30DRD_SUSPEND_CLK, CAL:cmucal-sfr.c:1167 */
};

/* the seven data-path gates no USB node claims (spec 7.1 D4) */
static const u16 l3_cgd[] = {
	0x2018,		/* AXI2AHB_USB_FSYS0_ACLK, CAL:cmucal-sfr.c:1150 */
	0x2040,		/* LHM_AXI_D_USBTV_I_CLK, CAL:cmucal-sfr.c:1147 */
	0x2084,		/* USBTV_I_USBTVH_AHB_CLK, CAL:cmucal-sfr.c:1172 */
	0x2088,		/* USBTV_I_USBTVH_CORE_CLK, CAL:cmucal-sfr.c:1166 */
	0x208c,		/* USBTV_I_USBTVH_XIU_CLK, CAL:cmucal-sfr.c:1173 */
	0x2090,		/* US_D_FSYS0_USB_ACLK, CAL:cmucal-sfr.c:1148 */
	0x2098,		/* XIU_D_FSYS0_USB_ACLK, CAL:cmucal-sfr.c:1164 */
};

/*
 * gpi1 DAT: FSYS0 pinctrl 0x11050000 + bank 0x020 + DAT 0x04 (A:drivers/pinctrl/samsung/
 * pinctrl-exynos-arm64.c:74-77,1512 = S:drivers/pinctrl/samsung/pinctrl-exynos.c:67-70,1808)
 */
#define L3_PINCTRL_BASE		0x11050000
#define L3_PINCTRL_SIZE		0x100
#define L3_GPI1_DAT		0x24
#define L3_GPI1_7		BIT(7)

/* PMU USBDEV_PHY_CONTROL 0x704 bit 0: S:drivers/phy/phy-exynos-usbdrd.h:16-17 */
#define L3_PMU_USB		0x704

/* MAX77865 MUIC at 7-bit 0x25 (S:drivers/mfd/max77865.c:45 "(0x4A >> 1)") */
#define L3_MUIC_ADDR		0x25
static const u8 l3_muic_regs[] = {
	0x19,		/* CONTROL1, S:drivers/muic/universal/muic_regmap_max77865.c:80 */
	0x1c,		/* CONTROL4, :83 */
	0x0d,		/* STATUS1_BC, :74 */
	0x0e,		/* STATUS2_BC, :75 */
	0x0f,		/* STATUS_GP, :76 */
};

static unsigned int l3_hb_ms = 30000;		/* REWRITE_PLAN.md 5 X-5 */
module_param_named(hb_ms, l3_hb_ms, uint, 0444);
MODULE_PARM_DESC(hb_ms, "period of the usb printer (ms)");

static unsigned int l3_id_ms = 120000;		/* REWRITE_PLAN.md 5 X-5 */
module_param_named(id_ms, l3_id_ms, uint, 0444);
MODULE_PARM_DESC(id_ms, "period of the usbid printer (ms)");

static const struct greatlte_hb_item l3_rows[] = {
	{ .layer = 3, .tag = "irq",  .kind = GREATLTE_HB_IRQ, .path = L3_DWC3_PATH, .idx = 0 },	/* SPI 337 */
	{ .layer = 3, .tag = "dwc3", .kind = GREATLTE_HB_DEV, .path = L3_DWC3_PATH },
	{ .layer = 3, .tag = "glue", .kind = GREATLTE_HB_DEV, .path = L3_GLUE_PATH },
	{ .layer = 3, .tag = "phy",  .kind = GREATLTE_HB_DEV, .path = L3_PHY_PATH },
};

static void __iomem *l3_dwc, *l3_phy0, *l3_phy1, *l3_refclk, *l3_cmu, *l3_pinctrl;
static struct device_node *l3_dwc_np;
static unsigned int l3_virq;
static struct regmap *l3_pmu;

/* manual mode with CG_VAL clear: stock ra_get_gate(), S:drivers/soc/samsung/cal-if/ra.c:404-418 */
static bool l3_gate_closed(u32 v)
{
	return (v & L3_CG_MANUAL) && !(v & L3_CG_VAL);
}

/* digit of spec 8.1 "cgd": bit 8 = HWACG, bit 1 = CG_VAL, bit 0 = MANUAL */
static u32 l3_cg_digit(u32 v)
{
	return (v >> 20) & 0x103;
}

/* PHY window base, +4 when offset 0 of window 0 reads non-zero (same rule as the CAL) */
static void __iomem *l3_phy_base(void __iomem *win, u32 v0)
{
	return v0 ? win + 4 : win;
}

static int l3_rpm(void)
{
	struct platform_device *pdev = of_find_device_by_node(l3_dwc_np);
	int r;

	if (!pdev)
		return -ENODEV;
	r = pm_runtime_status_suspended(&pdev->dev);
	put_device(&pdev->dev);
	return r;
}

static int l3_muic(char *buf, size_t size)
{
#if IS_REACHABLE(CONFIG_I2C)
	struct device_node *np = of_find_node_by_path(L3_MUIC_BUS);
	struct i2c_adapter *adap = NULL;
	union i2c_smbus_data data;
	unsigned int i;
	int len = 0, r;

	if (np && of_device_is_available(np))
		adap = of_find_i2c_adapter_by_node(np);
	of_node_put(np);
	if (!adap)
		return scnprintf(buf, size, " muic=na");
	len += scnprintf(buf, size, " muic=");
	for (i = 0; i < ARRAY_SIZE(l3_muic_regs); i++) {
		r = i2c_smbus_xfer(adap, L3_MUIC_ADDR, 0, I2C_SMBUS_READ, l3_muic_regs[i],
				   I2C_SMBUS_BYTE_DATA, &data);
		len += r < 0 ? scnprintf(buf + len, size - len, "%s!%d", i ? "," : "", r) :
			       scnprintf(buf + len, size - len, "%s%02x", i ? "," : "", data.byte);
	}
	i2c_put_adapter(adap);
	return len;
#else
	return scnprintf(buf, size, " muic=na");
#endif
}

/* the usb line without irq= (spec 8.1); returns its length */
static int l3_body(char *buf, size_t size)
{
	u32 cg[ARRAY_SIZE(l3_cg)], cgd[ARRAY_SIZE(l3_cgd)], v, v0, pmu = 0, qch;
	bool aclk_ok, dp_ok = true, phy_ok, q_ok;
	struct irq_data *d = l3_virq ? irq_get_irq_data(l3_virq) : NULL;
	unsigned int i;
	int len, rpm, rp;

	rpm = l3_rpm();
	len = scnprintf(buf, size, "irqdis=%s rpm=%d", d ? (irqd_irq_disabled(d) ? "1" : "0") : "na",
			rpm);

	/* (1) CMU_FSYS0 gates */
	for (i = 0; i < ARRAY_SIZE(l3_cg); i++)
		cg[i] = readl(l3_cmu + l3_cg[i]);
	for (i = 0; i < ARRAY_SIZE(l3_cgd); i++) {
		cgd[i] = readl(l3_cmu + l3_cgd[i]);
		if (l3_gate_closed(cgd[i]))
			dp_ok = false;
	}
	aclk_ok = !l3_gate_closed(cg[0]);
	qch = readl(l3_cmu + L3_CMU_QCH);
	q_ok = L3_QCH_ON(qch);

	/* (2) PMU 0x704 */
	rp = l3_pmu ? regmap_read(l3_pmu, L3_PMU_USB, &pmu) : -ENODEV;
	phy_ok = aclk_ok && !rp && (pmu & 1);

	/* (3) dwc3 */
	if (!aclk_ok) {
		len += scnprintf(buf + len, size - len, " dwc=gated");
	} else if (!dp_ok) {
		len += scnprintf(buf + len, size - len, " dwc=gated-dp");
	} else if (!q_ok) {
		len += scnprintf(buf + len, size - len, " dwc=gated-q");
	} else {
		v = readl(l3_dwc + L3_DSTS);
		len += scnprintf(buf + len, size - len,
				 " dwc=ok evsz=%08x evcnt=%x dsts=%08x ls=%u spd=%u dctl=%08x dcfg=%08x devten=%08x gctl=%08x gusb2=%08x gusb3=%08x guctl1=%08x gsbus=%08x/%08x",
				 readl(l3_dwc + L3_GEVNTSIZ0), readl(l3_dwc + L3_GEVNTCOUNT0), v,
				 L3_DSTS_LS(v), L3_DSTS_SPD(v), readl(l3_dwc + L3_DCTL),
				 readl(l3_dwc + L3_DCFG), readl(l3_dwc + L3_DEVTEN),
				 readl(l3_dwc + L3_GCTL), readl(l3_dwc + L3_GUSB2PHYCFG0),
				 readl(l3_dwc + L3_GUSB3PIPECTL0), readl(l3_dwc + L3_GUCTL1),
				 readl(l3_dwc + L3_GSBUSCFG0), readl(l3_dwc + L3_GSBUSCFG1));
	}

	/* PHY */
	if (!aclk_ok) {
		len += scnprintf(buf + len, size - len, " phy=gated");
	} else if (!q_ok) {
		len += scnprintf(buf + len, size - len, " phy=gated-q");
	} else if (!phy_ok) {
		len += scnprintf(buf + len, size - len, " phy=isolated");
	} else {
		void __iomem *b0, *b1;

		v0 = readl(l3_phy0);
		b0 = l3_phy_base(l3_phy0, v0);
		b1 = l3_phy_base(l3_phy1, v0);
		len += scnprintf(buf + len, size - len,
				 " phy=ok v0=%x utmi=%08x utmi1=%08x ls8895=%08x resume=%08x clkrst=%08x p0=%08x test0=%x/%x refclk=%08x",
				 v0, readl(b0 + L3_PHY_PHYUTMI),
				 readl(b1 + L3_PHY_PHYUTMI),	/* spec 8.4 H8: bits 0-1 set, S:.../phy-samsung-usb3-cal.c:729-732 */
				 readl(b0 + L3_PHY_LINKSYSTEM),
				 readl(b0 + L3_PHY_PHYRESUME), readl(b0 + L3_PHY_PHYCLKRST),
				 readl(b0 + L3_PHY_PHYPARAM0), readl(b0 + L3_PHY_PHYTEST),
				 readl(b1 + L3_PHY_PHYTEST), readl(l3_refclk + L3_REFCLK_CTRL));
	}

	len += rp ? scnprintf(buf + len, size - len, " pmu704=!%d", rp) :
		    scnprintf(buf + len, size - len, " pmu704=%x", pmu);
	len += scnprintf(buf + len, size - len, " cg=%08x/%08x/%08x cgd=", cg[0], cg[1], cg[2]);
	for (i = 0; i < ARRAY_SIZE(l3_cgd); i++)
		len += scnprintf(buf + len, size - len, "%s%x", i ? "/" : "", l3_cg_digit(cgd[i]));
	v = readl(l3_pinctrl + L3_GPI1_DAT);
	len += scnprintf(buf + len, size - len, " qch=%x umux=%x gpio=%d port=%d",
			 qch, readl(l3_cmu + L3_CMU_UMUX),
			 !!(v & L3_GPI1_7), !(v & L3_GPI1_7));	/* port = !level, S:phy-exynos-usbdrd.c:767-768 */
	len += l3_muic(buf + len, size - len);
	return len;
}

/* "usb": slow printer, the full field line every time (X-5) */
static int l3_fmt(char *buf, size_t size)
{
	int len = scnprintf(buf, size, "irq=%u ", l3_virq ? kstat_irqs_usr(l3_virq) : 0);

	return len + l3_body(buf + len, size - len);
}

static int l3_rate(char *buf, size_t size, const char *key, const char *path, const char *name)
{
	struct device_node *np = of_find_node_by_path(path);
	struct clk *c;
	int len;

	if (!np)
		return scnprintf(buf, size, " %s=!%d", key, -ENODEV);
	c = of_clk_get_by_name(np, name);
	of_node_put(np);
	if (IS_ERR(c))
		return scnprintf(buf, size, " %s=!%ld", key, PTR_ERR(c));
	len = scnprintf(buf, size, " %s=%lu", key, clk_get_rate(c));
	clk_put(c);
	return len;
}

/* cpuidle state names of one CPU, spec 8.1 "idle=" (R11) */
static int l3_idle(char *buf, size_t size, unsigned int cpu)
{
	struct cpuidle_driver *drv = NULL;
	int i, len = 0;

	if (IS_ENABLED(CONFIG_CPU_IDLE))
		drv = cpuidle_get_cpu_driver(per_cpu(cpuidle_devices, cpu));
	if (!drv || !drv->state_count)
		return scnprintf(buf, size, "-");
	for (i = 0; i < drv->state_count; i++)
		len += scnprintf(buf + len, size - len, "%s%s", i ? "," : "", drv->states[i].name);
	return len;
}

/* "usbid": slow printer */
static int l3id_fmt(char *buf, size_t size)
{
	struct greatlte_l3_phy_snap s;
	int len = 0, r, i;
	bool dwc_ok = true;

	for (i = 0; i < ARRAY_SIZE(l3_cgd); i++)
		if (l3_gate_closed(readl(l3_cmu + l3_cgd[i])))
			dwc_ok = false;
	if (l3_gate_closed(readl(l3_cmu + l3_cg[0])) || !L3_QCH_ON(readl(l3_cmu + L3_CMU_QCH)))
		dwc_ok = false;
	if (dwc_ok)
		len += scnprintf(buf, size, "gsnpsid=%08x hwp0=%08x hwp1=%08x hwp2=%08x hwp3=%08x grxthr=%08x",
				 readl(l3_dwc + L3_GSNPSID), readl(l3_dwc + L3_GHWPARAMS(0)),
				 readl(l3_dwc + L3_GHWPARAMS(1)), readl(l3_dwc + L3_GHWPARAMS(2)),
				 readl(l3_dwc + L3_GHWPARAMS(3)), readl(l3_dwc + L3_GRXTHRCFG));
	else
		len += scnprintf(buf, size, "gsnpsid=gated");
	len += l3_rate(buf + len, size - len, "aclk", L3_GLUE_PATH, "aclk");
	len += l3_rate(buf + len, size - len, "ref", L3_GLUE_PATH, "ref");
	len += l3_rate(buf + len, size - len, "susp", L3_DWC3_PATH, "suspend");
	len += l3_rate(buf + len, size - len, "phyref", L3_PHY_PATH, "ref");
	len += scnprintf(buf + len, size - len, " idle=");
	len += l3_idle(buf + len, size - len, 0);	/* A53 cluster, FDT:4277 */
	len += scnprintf(buf + len, size - len, "/");
	len += l3_idle(buf + len, size - len, 4);	/* Mongoose cluster, FDT:4320 */

	r = exynos8895_usbdrd_get_snap(&s);
	if (r)
		len += scnprintf(buf + len, size - len, " ver0=!%d", r);
	else
		len += scnprintf(buf + len, size - len,
				 " ver0=%x ver1=%x shift=%d port=%d inits=%u retunes=%u crto=%u cr1002=%x armed=%d",
				 s.ver[0], s.ver[1], s.probed ? s.shift : -1, s.port, s.inits,
				 s.retunes, s.cr_timeouts, s.cr1002_before < 0 ? 0xffff : s.cr1002_before,
				 s.armed);
	return len;
}

static struct greatlte_hb_printer l3_prn = {
	.name = "usb",
	.layer = 3,
	.slow = true,
	.fmt = l3_fmt,
};

static struct greatlte_hb_printer l3id_prn = {
	.name = "usbid",
	.layer = 3,
	.slow = true,
	.fmt = l3id_fmt,
};

static void __iomem *l3_map(phys_addr_t pa, size_t size)
{
	void __iomem *p = ioremap(pa, size);

	if (!p)
		pr_err("greatlte-hb-l3: cannot map %pa\n", &pa);
	return p;
}

static int __init l3_hb_init(void)
{
	struct device_node *glue = of_find_node_by_path(L3_GLUE_PATH);
	bool on = glue && of_device_is_available(glue);

	of_node_put(glue);
	if (!on)
		return 0;

	l3_dwc = l3_map(L3_DWC3_BASE, L3_DWC3_SIZE);
	l3_phy0 = l3_map(L3_PHY0_BASE, L3_PHY_SIZE);
	l3_phy1 = l3_map(L3_PHY1_BASE, L3_PHY_SIZE);
	l3_refclk = l3_map(L3_REFCLK_BASE, L3_REFCLK_SIZE);
	l3_cmu = l3_map(L3_CMU_BASE, L3_CMU_SIZE);
	l3_pinctrl = l3_map(L3_PINCTRL_BASE, L3_PINCTRL_SIZE);
	if (!l3_dwc || !l3_phy0 || !l3_phy1 || !l3_refclk || !l3_cmu || !l3_pinctrl)
		return 0;

	l3_dwc_np = of_find_node_by_path(L3_DWC3_PATH);	/* reference kept */
	if (l3_dwc_np)
		l3_virq = irq_of_parse_and_map(l3_dwc_np, 0);
	l3_pmu = syscon_regmap_lookup_by_compatible("samsung,exynos8895-pmu");
	if (IS_ERR(l3_pmu))
		l3_pmu = NULL;

	greatlte_hb_add_items(l3_rows, ARRAY_SIZE(l3_rows));
	l3_prn.every_ms = l3_hb_ms;
	l3id_prn.every_ms = l3_id_ms;
	greatlte_hb_register(&l3_prn);
	greatlte_hb_register(&l3id_prn);
	return 0;
}
late_initcall(l3_hb_init);
