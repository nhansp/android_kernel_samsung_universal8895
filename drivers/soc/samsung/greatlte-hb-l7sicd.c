// SPDX-License-Identifier: GPL-2.0-only
/*
 * greatlte L7-sicd heartbeat rows: the s2idle SICD path of WP l7sicd
 * (drivers/cpuidle/cpuidle-exynos8895.c). DEBUG ONLY.
 *
 * The L7 printer line already carries sicd=, sicd_try= and veto=, but sicd=
 * counts every SICD exit, and the normal idle path takes SICD on its own between
 * suspend cycles, so a rise of sicd= across a cycle cannot be attributed. The rows
 * here read only stored counters (hb rule 4, no PMU and no clock read).
 */

#include <linux/atomic.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/soc/samsung/exynos8895-pm.h>
#include <linux/soc/samsung/greatlte-hb.h>

static u32 l7sicd_prev;		/* sicd_s2 at the previous heartbeat period */

static int l7sicd_hb_s2(char *buf, size_t size)
{
	u32 now = atomic_read(&e8895_stats.sicd_s2);
	u32 d = now - READ_ONCE(l7sicd_prev);

	WRITE_ONCE(l7sicd_prev, now);
	return scnprintf(buf, size, "%u/%u", d, now);	/* delta/total, one token */
}

/* s2idle SICD candidates refused: last CPU not in the boot cluster (stock's rule) */
static int l7sicd_hb_nb(char *buf, size_t size)
{
	return scnprintf(buf, size, "%u", atomic_read(&e8895_stats.sicd_nb));
}

static const struct greatlte_hb_item l7sicd_rows[] = {
	{ .layer = 7, .tag = "sicds2", .kind = GREATLTE_HB_FN, .fn = l7sicd_hb_s2 },
	{ .layer = 7, .tag = "sicdnb", .kind = GREATLTE_HB_FN, .fn = l7sicd_hb_nb },
};

static int __init l7sicd_hb_init(void)
{
	greatlte_hb_add_items(l7sicd_rows, ARRAY_SIZE(l7sicd_rows));
	return 0;
}
late_initcall(l7sicd_hb_init);
