// SPDX-License-Identifier: GPL-2.0-only
/*
 * greatlte L2 (power) heartbeat rows and printers. DEBUG ONLY, never upstream.
 * Spec: rewrite/spec/L2_power.md 5.9 and 8.4 (pass lines P11-P13 of 1), through
 * the heartbeat hooks of T_tooling.md 5.7.3/5.7.6 as extended by
 * FULL_IMAGE_PLAN.md 2.2 T14 (rows registered from the layer's own DEBUG file,
 * plan X-55). No L2 code runs its own periodic print.
 *
 * Rows, on the "hb: v1 L2f seq=<n> ..." line of every period:
 *   f0 f4 (cpufreq kHz of policy0/policy4), mbx pmirq tmui (interrupt counts of
 *   the AP2APM mailbox, the S2MPS17 line and the big TMU), acpm pmic tmu (bound
 *   driver or U), vqmmc ldo6 ldo10 dp18 (<enabled>:<uV> of LDO2/6/10/37).
 * Printers (both slow: they run in hb-probe, T 5.7.6 rule 4):
 *   "L2" every greatlte_hb_l2.hb_ms (default 20000, spec 5.9; 60000 in the
 *   all-in image, plan 3.4):
 *     f0=<cpufreq>/<pll> f4=<cpufreq>/<pll> tx0=<c>/<e> tx2=<c>/<e> tx4=<c>/<e>
 *     n5=<c> late=<c> fwl=<c> cold=<mask> t=<big>/<little>/<gpu>/<isp>
 *   "L2id" every greatlte_hb_l2.id_ms (default 300000):
 *     asv= chans= plugins= dvfs_plugin=<id>:<att>/<stay>:<dp_a> dpd=<id>:<res>
 *     fvmap=<lv0>/<lv1> pmicid=0x<v> rev=0x<r> fwlog=<front>/<rear>/<len>
 *     then one record per ACPM channel:
 *     ch=<i> id= field=0x poll= type= q=<qlen>x<mlen> stale=
 * An unreadable value prints !<errno>. Registered at late_initcall when the
 * ACPM node is enabled; nothing is sent to the APM or the PMIC at print time.
 */
#include <linux/cpufreq.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/soc/samsung/greatlte-hb.h>
#include <linux/soc/samsung/greatlte-l2.h>
#include <linux/thermal.h>

#include <dt-bindings/clock/samsung,exynos8895-acpm.h>

#define L2_ACPM_PATH	"/firmware/power-management"

static unsigned int l2_hb_ms = 20000;
module_param_named(hb_ms, l2_hb_ms, uint, 0444);
MODULE_PARM_DESC(hb_ms, "period of the L2 printer (ms)");

static unsigned int l2_id_ms = 300000;
module_param_named(id_ms, l2_id_ms, uint, 0444);
MODULE_PARM_DESC(id_ms, "period of the L2id printer (ms)");

static const struct greatlte_hb_item l2_rows[] = {
	/* L2_power.md 5.9 (a) */
	{ .layer = 2, .tag = "f0",    .kind = GREATLTE_HB_FREQ, .idx = 0 },	/* policy0, A53 */
	{ .layer = 2, .tag = "f4",    .kind = GREATLTE_HB_FREQ, .idx = 4 },	/* policy4, Mongoose */
	{ .layer = 2, .tag = "mbx",   .kind = GREATLTE_HB_IRQ, .path = "/soc@0/mailbox@16440000", .idx = 0 },	/* SPI 39, 4.2 */
	{ .layer = 2, .tag = "pmirq", .kind = GREATLTE_HB_IRQ, .path = L2_ACPM_PATH "/pmic", .idx = 0 },	/* gpa3-6, 4.3 */
	{ .layer = 2, .tag = "tmui",  .kind = GREATLTE_HB_IRQ, .path = "/soc@0/tmu-big@10080000", .idx = 0 },	/* SPI 451, 4.6 */
	{ .layer = 2, .tag = "acpm",  .kind = GREATLTE_HB_DEV, .path = L2_ACPM_PATH },
	{ .layer = 2, .tag = "pmic",  .kind = GREATLTE_HB_DEV, .path = L2_ACPM_PATH "/pmic" },
	{ .layer = 2, .tag = "tmu",   .kind = GREATLTE_HB_DEV, .path = "/soc@0/tmu-big@10080000" },
	{ .layer = 2, .tag = "vqmmc", .kind = GREATLTE_HB_REG, .path = "vqmmc" },	/* LDO2 */
	{ .layer = 2, .tag = "ldo6",  .kind = GREATLTE_HB_REG, .path = "vdd_ldo6" },	/* USB HS PHY */
	{ .layer = 2, .tag = "ldo10", .kind = GREATLTE_HB_REG, .path = "vdd_ldo10" },	/* USB SS PHY */
	{ .layer = 2, .tag = "dp18",  .kind = GREATLTE_HB_REG, .path = "VDD_DP_1P8" },	/* LDO37 */
};

/* thermal zone types = DT node names (A:drivers/thermal/thermal_of.c:415) */
static const char * const l2_zones[] = {
	"big-thermal", "little-thermal", "gpu-thermal", "isp-thermal",
};

static struct device_node *l2_acpm_np;
static struct thermal_zone_device *l2_tz[ARRAY_SIZE(l2_zones)];
static struct acpm_info_snap l2_info;	/* hb-probe only */

/* "<pre><key>=<a>/<b>", each value or !<errno> */
static int l2_pair(char *buf, size_t size, const char *pre, const char *key,
		   long a, long b)
{
	int len = scnprintf(buf, size, "%s%s=", pre, key);

	len += a < 0 ? scnprintf(buf + len, size - len, "!%ld", a) :
		       scnprintf(buf + len, size - len, "%ld", a);
	len += b < 0 ? scnprintf(buf + len, size - len, "/!%ld", b) :
		       scnprintf(buf + len, size - len, "/%ld", b);
	return len;
}

static int l2_tx(char *buf, size_t size, unsigned int ch)
{
	struct acpm_chan_stats_snap st;
	int r = acpm_get_chan_stats(l2_acpm_np, ch, &st);

	if (r)
		return scnprintf(buf, size, " tx%u=!%d", ch, r);
	return scnprintf(buf, size, " tx%u=%u/%u", ch, st.xfer, st.err);
}

/* zone @i in whole degrees C into *t; 0 or -errno */
static int l2_temp(unsigned int i, int *t)
{
	int r;

	if (!l2_tz[i]) {
		struct thermal_zone_device *tz = thermal_zone_get_zone_by_name(l2_zones[i]);

		if (IS_ERR(tz))
			return PTR_ERR(tz);
		l2_tz[i] = tz;
	}
	r = thermal_zone_get_temp(l2_tz[i], t);
	if (!r)
		*t /= 1000;
	return r;
}

/* "L2": slow printer (thermal zone lock, PLL MMIO behind the online-CPU check) */
static int l2_fmt(char *buf, size_t size)
{
	struct acpm_chan_stats_snap st;
	unsigned int ch, late = 0;
	int len, r, n5 = -ENODEV, i, c;

	len = l2_pair(buf, size, "", "f0", cpufreq_quick_get(0),
		      exynos8895_acpm_clk_pll_khz(EXYNOS8895_CLK_ACPM_DVFS_CPUCL1));
	len += l2_pair(buf + len, size - len, " ", "f4", cpufreq_quick_get(4),
		       exynos8895_acpm_clk_pll_khz(EXYNOS8895_CLK_ACPM_DVFS_CPUCL0));
	len += l2_tx(buf + len, size - len, 0);
	len += l2_tx(buf + len, size - len, 2);
	len += l2_tx(buf + len, size - len, 4);

	for (ch = 0, r = 0; ch < GREATLTE_L2_MAX_CHANS; ch++) {
		r = acpm_get_chan_stats(l2_acpm_np, ch, &st);
		if (r)
			break;
		late += st.late;
		if (ch == 5)
			n5 = st.notify;
	}
	/* the loop ends with -EINVAL past the last channel */
	if (r == -EINVAL && ch)
		r = 0;
	if (r && n5 == -ENODEV)
		n5 = r;
	len += n5 < 0 ? scnprintf(buf + len, size - len, " n5=!%d", n5) :
			scnprintf(buf + len, size - len, " n5=%d", n5);
	len += r ? scnprintf(buf + len, size - len, " late=!%d", r) :
		   scnprintf(buf + len, size - len, " late=%u", late);

	r = acpm_get_info(l2_acpm_np, &l2_info);
	len += r ? scnprintf(buf + len, size - len, " fwl=!%d", r) :
		   scnprintf(buf + len, size - len, " fwl=%u", l2_info.fwlog_entries);

	c = exynos8895_acpm_clk_cold();
	len += c < 0 ? scnprintf(buf + len, size - len, " cold=!%d", c) :
		       scnprintf(buf + len, size - len, " cold=%d", c);

	len += scnprintf(buf + len, size - len, " t=");
	for (i = 0; i < ARRAY_SIZE(l2_zones); i++) {
		int t, e = l2_temp(i, &t);

		len += scnprintf(buf + len, size - len, "%s%s%d", i ? "/" : "",
				 e ? "!" : "", e ? e : t);
	}

	return len;
}

/* "L2id": identity and probe results (P2-P7, P9), repeated for the 16 KB wrap */
static int l2id_fmt(char *buf, size_t size)
{
	const struct acpm_info_snap *s = &l2_info;
	int len = 0, r, ri, asv = exynos8895_acpm_clk_asv_ver();
	unsigned int i;
	u8 id, rev;

	len += asv < 0 ? scnprintf(buf, size, "asv=!%d", asv) :
			 scnprintf(buf, size, "asv=%d", asv);

	ri = acpm_get_info(l2_acpm_np, &l2_info);
	if (ri) {
		len += scnprintf(buf + len, size - len, " chans=!%d", ri);
	} else {
		len += scnprintf(buf + len, size - len,
				 " chans=%u plugins=%u dvfs_plugin=%d:%u/%u:%s dpd=%d:%s fvmap=%u/%u",
				 s->num_chans, s->nplugins, s->dvfs_plugin_id,
				 s->dvfs_attached, s->dvfs_stay, s->dvfs_dpa,
				 s->dpd_plugin_id, s->dpd_res, s->fvmap_lv[0],
				 s->fvmap_lv[1]);
	}

	r = sec_pmic_acpm_get_id(&id, &rev);
	len += r ? scnprintf(buf + len, size - len, " pmicid=!%d", r) :
		   scnprintf(buf + len, size - len, " pmicid=0x%02x rev=0x%02x", id, rev);

	if (ri)
		len += scnprintf(buf + len, size - len, " fwlog=!%d", ri);
	else if (!s->fwlog_ok)
		len += scnprintf(buf + len, size - len, " fwlog=off");
	else
		len += scnprintf(buf + len, size - len, " fwlog=%u/%u/%u",
				 s->fwlog_front, s->fwlog_rear, s->fwlog_len);

	for (i = 0; !ri && i < s->num_chans && i < GREATLTE_L2_MAX_CHANS; i++) {
		const struct acpm_chan_info_snap *c = &s->chan[i];

		len += scnprintf(buf + len, size - len,
				 "\nch=%u id=%u field=0x%x poll=%u type=%u q=%ux%u stale=%u",
				 i, c->id, c->field, c->poll, c->type, c->qlen,
				 c->mlen, c->stale);
	}

	return len;
}

static struct greatlte_hb_printer l2_prn = {
	.name = "L2",
	.layer = 2,
	.slow = true,
	.fmt = l2_fmt,
};

static struct greatlte_hb_printer l2id_prn = {
	.name = "L2id",
	.layer = 2,
	.slow = true,
	.fmt = l2id_fmt,
};

static int __init l2_hb_init(void)
{
	struct device_node *np = of_find_node_by_path(L2_ACPM_PATH);

	if (!np || !of_device_is_available(np)) {
		of_node_put(np);
		return 0;
	}
	l2_acpm_np = np;	/* reference kept */

	greatlte_hb_add_items(l2_rows, ARRAY_SIZE(l2_rows));
	l2_prn.every_ms = l2_hb_ms;
	l2id_prn.every_ms = l2_id_ms;
	greatlte_hb_register(&l2_prn);
	greatlte_hb_register(&l2id_prn);
	return 0;
}
late_initcall(l2_hb_init);
