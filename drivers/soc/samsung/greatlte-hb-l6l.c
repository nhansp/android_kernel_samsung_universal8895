// SPDX-License-Identifier: GPL-2.0-only
/*
 * greatlte L6L (G2D blitter + m2m1shot2) heartbeat rows (spec L6L 5.4).
 * DEBUG ONLY.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/soc/samsung/greatlte-hb.h>

struct g2d_hb_stats {
	u32 g2dj;
	u32 g2de;
	u32 g2dt;
	u32 g2dff;
};
extern struct g2d_hb_stats g2d_stats;

/* jobs finished */
static int l6l_hb_g2dj(char *buf, size_t size)
{
	return scnprintf(buf, size, "%u", g2d_stats.g2dj);
}

/* jobs with abnormal/error interrupt bits */
static int l6l_hb_g2de(char *buf, size_t size)
{
	return scnprintf(buf, size, "%u", g2d_stats.g2de);
}

/* timeout count */
static int l6l_hb_g2dt(char *buf, size_t size)
{
	return scnprintf(buf, size, "%u", g2d_stats.g2dt);
}

/* NV12 flip-flop workaround executions */
static int l6l_hb_g2dff(char *buf, size_t size)
{
	return scnprintf(buf, size, "%u", g2d_stats.g2dff);
}

static const struct greatlte_hb_item l6l_rows[] = {
	{ .layer = 6, .tag = "g2d",   .kind = GREATLTE_HB_DEV, .path = "/soc@0/g2d@13a30000" },
	{ .layer = 6, .tag = "g2dj",  .kind = GREATLTE_HB_FN,  .fn = l6l_hb_g2dj },
	{ .layer = 6, .tag = "g2de",  .kind = GREATLTE_HB_FN,  .fn = l6l_hb_g2de },
	{ .layer = 6, .tag = "g2dt",  .kind = GREATLTE_HB_FN,  .fn = l6l_hb_g2dt },
	{ .layer = 6, .tag = "g2dff", .kind = GREATLTE_HB_FN,  .fn = l6l_hb_g2dff },
};

static int __init l6l_hb_init(void)
{
	greatlte_hb_add_items(l6l_rows, ARRAY_SIZE(l6l_rows));
	return 0;
}
late_initcall(l6l_hb_init);
