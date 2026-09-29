// SPDX-License-Identifier: GPL-2.0-only
/*
 * greatlte L6A S3CFB front-end heartbeat rows (spec L6A 8.1: s3f, s3e, s3t).
 * DEBUG ONLY. The counters are written by drivers/gpu/drm/exynos/
 * exynos8895_s3cfb.c when it is built in (include/linux/soc/samsung/
 * greatlte-l6a-s3cfb.h); the FN rows only read them (lock-free, T 5.7.6 rule 4).
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/soc/samsung/greatlte-hb.h>
#include <linux/soc/samsung/greatlte-l6a-s3cfb.h>

struct greatlte_l6a_s3cfb_stats greatlte_l6a_s3cfb;

/* frames applied: grows at the UI frame rate (L6A 1.2 L6A.2 (c)) */
static int l6a_s3f(char *buf, size_t size)
{
	return scnprintf(buf, size, "%u", (unsigned int)atomic_read(&greatlte_l6a_s3cfb.s3f));
}

/* WIN_CONFIG commits rejected by atomic_check: 0 */
static int l6a_s3e(char *buf, size_t size)
{
	return scnprintf(buf, size, "%u", (unsigned int)atomic_read(&greatlte_l6a_s3cfb.s3e));
}

/* acquire-fence timeouts: 0 */
static int l6a_s3t(char *buf, size_t size)
{
	return scnprintf(buf, size, "%u", (unsigned int)atomic_read(&greatlte_l6a_s3cfb.s3t));
}

static const struct greatlte_hb_item l6a_s3cfb_rows[] = {
	{ .layer = 6, .tag = "s3f", .kind = GREATLTE_HB_FN, .fn = l6a_s3f },
	{ .layer = 6, .tag = "s3e", .kind = GREATLTE_HB_FN, .fn = l6a_s3e },
	{ .layer = 6, .tag = "s3t", .kind = GREATLTE_HB_FN, .fn = l6a_s3t },
};

static int __init l6a_s3cfb_hb_init(void)
{
	greatlte_hb_add_items(l6a_s3cfb_rows, ARRAY_SIZE(l6a_s3cfb_rows));
	return 0;
}
late_initcall(l6a_s3cfb_hb_init);
