// SPDX-License-Identifier: GPL-2.0-only
/*
 * greatlte L4 (storage: UFS and SD) heartbeat rows and printer. DEBUG ONLY,
 * never upstream. Spec: rewrite/spec/L4_storage.md 1.3 and 8.1, through the
 * heartbeat hooks of T_tooling.md 5.7.3/5.7.6 as extended by
 * FULL_IMAGE_PLAN.md 2.2 T14 (rows registered from the layer's own DEBUG
 * file, plan X-55). Cadence: plan 3.4 row L4 (60 s).
 *
 * Rows, on the "hb: v1 L4f seq=<n> ..." line of every period:
 *   ufs ufsphy sd (bound driver or U), ufsirq sdirq (interrupt counts),
 *   vcc vmmc (<enabled>:<uV> of the regulators "ufs-vcc" and "vmmc"; vqmmc is
 *   an L2 row).
 * Printer "L4" (slow: runs in hb-probe, T 5.7.6 rule 4), every
 * greatlte_hb_l4.hb_ms (default 60000):
 *   ufs=<probe rc> ufs_pmc=G<gear>L<lanes><A|B> ufs_luns=<n> sda_parts=<n>
 *   sys_sz=<sectors>@<partno> vcc=<0|1> mmc_cd=<0|1> mmc=<state>
 *   ufs_st=<ufshcd_state> ufs_evt=<sum of the UFS event counters>
 * with (spec 8.1):
 *   ufs=      0 once 11120000.ufs is bound, else the last probe return value
 *             (greatlte_l4_ufs_probe_done() from ufs-exynos), "-" before any;
 *   ufs_pmc=  hba->pwr_info (PWM modes print P<gear>L<lanes>);
 *   ufs_luns= sd* disks below the UFS host; sda_parts= partitions of sda;
 *   sys_sz=   size and number of the partition whose PARTNAME is SYSTEM (the
 *             "@15" checks GPT index = PIT identifier, spec U-8);
 *   vcc=      regulator_is_enabled("ufs-vcc");
 *   mmc_cd=   level of gpa1-5 (the DT's cd-gpios is active low: 0 = card in);
 *   mmc=      none | init (card detected, not enumerated) | err:<cmd>:<errno>
 *             (card detected, not enumerated, last failed request) |
 *             ok:<SD|SDHC|MMC|SDIO|t<n>>/<bus kHz> (SDHC = block-addressed,
 *             i.e. also SDXC).
 * An unreadable value prints !<errno>. Registered at late_initcall when the
 * UFS or the SD node is enabled; nothing is written to the hardware.
 */
#include <linux/atomic.h>
#include <linux/blkdev.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mmc/card.h>
#include <linux/mmc/core.h>
#include <linux/mmc/host.h>
#include <linux/mmc/slot-gpio.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/soc/samsung/greatlte-hb.h>
#include <linux/soc/samsung/greatlte-l4.h>
#include <linux/string.h>
#include <ufs/ufshcd.h>

#include <trace/events/mmc.h>

#define L4_UFS_PATH	"/soc@0/ufs@11120000"
#define L4_PHY_PATH	"/soc@0/phy@11124000"
#define L4_SD_PATH	"/soc@0/mmc@11500000"

static unsigned int l4_hb_ms = 60000;
module_param_named(hb_ms, l4_hb_ms, uint, 0444);
MODULE_PARM_DESC(hb_ms, "period of the L4 printer (ms)");

static const struct greatlte_hb_item l4_rows[] = {
	{ .layer = 4, .tag = "ufs",    .kind = GREATLTE_HB_DEV, .path = L4_UFS_PATH },
	{ .layer = 4, .tag = "ufsphy", .kind = GREATLTE_HB_DEV, .path = L4_PHY_PATH },
	{ .layer = 4, .tag = "ufsirq", .kind = GREATLTE_HB_IRQ, .path = L4_UFS_PATH, .idx = 0 },	/* SPI 334 */
	{ .layer = 4, .tag = "sd",     .kind = GREATLTE_HB_DEV, .path = L4_SD_PATH },
	{ .layer = 4, .tag = "sdirq",  .kind = GREATLTE_HB_IRQ, .path = L4_SD_PATH, .idx = 0 },	/* SPI 341 */
	{ .layer = 4, .tag = "vcc",    .kind = GREATLTE_HB_REG, .path = "ufs-vcc" },	/* gpg0-0 */
	{ .layer = 4, .tag = "vmmc",   .kind = GREATLTE_HB_REG, .path = "vmmc" },	/* gpb1-1 */
};

/* ---- UFS ---- */

#define L4_RC_NONE	INT_MIN		/* no probe has returned yet */

static atomic_t l4_ufs_rc = ATOMIC_INIT(L4_RC_NONE);
static struct device_node *l4_ufs_np, *l4_sd_np;

void greatlte_l4_ufs_probe_done(struct device *dev, int err)
{
	atomic_set(&l4_ufs_rc, err);
}
EXPORT_SYMBOL_GPL(greatlte_l4_ufs_probe_done);

/*
 * The bound platform device of @np with a reference, or NULL. Bound means
 * probe() returned 0 (driver_bound(), A:drivers/base/dd.c:453-456): dev->driver
 * is already set while probe() runs, when the UFS drvdata is still NULL or a
 * half-built hba, and until a failed probe is cleaned up.
 */
static struct device *l4_bound_dev(struct device_node *np)
{
	struct device *dev;

	if (!np)
		return NULL;
	dev = bus_find_device_by_of_node(&platform_bus_type, np);
	if (dev && !device_is_bound(dev)) {
		put_device(dev);
		dev = NULL;
	}
	return dev;
}

struct l4_blk {
	struct device *host;	/* the UFS platform device */
	int luns;
	int sda_parts;
	sector_t sys_sz;
	int sys_part;
};

static bool l4_below(struct device *dev, struct device *anc)
{
	for (; dev; dev = dev->parent)
		if (dev == anc)
			return true;
	return false;
}

static int l4_blk_one(struct device *dev, void *data)
{
	struct l4_blk *b = data;
	struct block_device *bdev = dev_to_bdev(dev);

	if (!l4_below(dev, b->host))
		return 0;
	if (!bdev_is_partition(bdev)) {
		b->luns++;
		return 0;
	}
	if (!strcmp(bdev->bd_disk->disk_name, "sda"))
		b->sda_parts++;
	if (bdev->bd_meta_info && !strcmp(bdev->bd_meta_info->volname, "SYSTEM")) {
		b->sys_sz = bdev_nr_sectors(bdev);
		b->sys_part = bdev_partno(bdev);
	}
	return 0;
}

static int l4_fmt_ufs(char *buf, size_t size)
{
	struct device *dev = l4_bound_dev(l4_ufs_np);
	struct l4_blk b = { .sys_part = -1 };
	int rc = atomic_read(&l4_ufs_rc);
	unsigned long long evt = 0;
	struct ufs_hba *hba;
	int len, i;

	if (dev)
		len = scnprintf(buf, size, "ufs=0");
	else if (rc == L4_RC_NONE)
		len = scnprintf(buf, size, "ufs=-");
	else
		len = scnprintf(buf, size, "ufs=%d", rc ? rc : -ENODEV);	/* 0 but unbound: removed */

	if (!dev)
		return len + scnprintf(buf + len, size - len,
				       " ufs_pmc=- ufs_luns=0 sda_parts=0 sys_sz=- ufs_st=- ufs_evt=-");

	hba = dev_get_drvdata(dev);
	if (hba->pwr_info.pwr_rx == FAST_MODE || hba->pwr_info.pwr_rx == FASTAUTO_MODE)
		len += scnprintf(buf + len, size - len, " ufs_pmc=G%uL%u%c",
				 hba->pwr_info.gear_rx, hba->pwr_info.lane_rx,
				 hba->pwr_info.hs_rate == PA_HS_MODE_B ? 'B' : 'A');
	else
		len += scnprintf(buf + len, size - len, " ufs_pmc=P%uL%u",
				 hba->pwr_info.gear_rx, hba->pwr_info.lane_rx);

	b.host = dev;
	class_for_each_device(&block_class, NULL, &b, l4_blk_one);
	len += scnprintf(buf + len, size - len, " ufs_luns=%d sda_parts=%d", b.luns, b.sda_parts);
	if (b.sys_part < 0)
		len += scnprintf(buf + len, size - len, " sys_sz=-");
	else
		len += scnprintf(buf + len, size - len, " sys_sz=%llu@%d",
				 (unsigned long long)b.sys_sz, b.sys_part);

	for (i = 0; i < UFS_EVT_CNT; i++)
		evt += hba->ufs_stats.event[i].cnt;
	len += scnprintf(buf + len, size - len, " ufs_st=%d ufs_evt=%llu",
			 hba->ufshcd_state, evt);

	put_device(dev);
	return len;
}

static struct regulator *l4_vcc;

static int l4_fmt_vcc(char *buf, size_t size)
{
	if (!l4_vcc) {
		struct regulator *r = regulator_get_optional(NULL, "ufs-vcc");

		if (IS_ERR(r))
			return scnprintf(buf, size, " vcc=!%ld", PTR_ERR(r));
		l4_vcc = r;	/* kept */
	}
	return scnprintf(buf, size, " vcc=%d", regulator_is_enabled(l4_vcc) > 0);
}

/* ---- SD ---- */

/* last failed request on the SD host (mmc_request_done tracepoint, any context) */
static atomic_t l4_sd_err_op = ATOMIC_INIT(-1);
static atomic_t l4_sd_err = ATOMIC_INIT(0);

static void l4_mmc_done(void *data, struct mmc_host *host, struct mmc_request *mrq)
{
	int e = 0;

	if (!l4_sd_np || !host->parent || host->parent->of_node != l4_sd_np)
		return;
	if (mrq->sbc && mrq->sbc->error)
		e = mrq->sbc->error;
	else if (mrq->cmd && mrq->cmd->error)
		e = mrq->cmd->error;
	else if (mrq->data && mrq->data->error)
		e = mrq->data->error;
	else if (mrq->stop && mrq->stop->error)
		e = mrq->stop->error;
	if (!e)
		return;
	atomic_set(&l4_sd_err_op, mrq->cmd ? mrq->cmd->opcode : -1);
	atomic_set(&l4_sd_err, e);
}

static int l4_is_mmc_host(struct device *dev, const void *data)
{
	return dev->class && !strcmp(dev->class->name, "mmc_host");
}

static int l4_first_child(struct device *dev, const void *data)
{
	return 1;
}

static const char *l4_card_type(struct mmc_card *card)
{
	switch (card->type) {
	case MMC_TYPE_SD:
		return mmc_card_is_blockaddr(card) ? "SDHC" : "SD";
	case MMC_TYPE_MMC:
		return "MMC";
	case MMC_TYPE_SDIO:
	case MMC_TYPE_SD_COMBO:
		return "SDIO";
	default:
		return NULL;
	}
}

static int l4_fmt_sd(char *buf, size_t size)
{
	struct device *pdev = l4_bound_dev(l4_sd_np);
	struct device *hdev, *cdev;
	struct mmc_host *host;
	int len, cd;

	if (!pdev)
		return scnprintf(buf, size, " mmc_cd=- mmc=!%d", -ENODEV);

	hdev = device_find_child(pdev, NULL, l4_is_mmc_host);
	put_device(pdev);
	if (!hdev)
		return scnprintf(buf, size, " mmc_cd=- mmc=!%d", -ENODEV);
	host = container_of(hdev, struct mmc_host, class_dev);

	cd = mmc_gpio_get_cd(host);	/* logical: 1 = card present */
	if (cd < 0)
		len = scnprintf(buf, size, " mmc_cd=!%d", cd);
	else
		len = scnprintf(buf, size, " mmc_cd=%d", !cd);	/* cd-gpios is active low */

	cdev = device_find_child(hdev, NULL, l4_first_child);	/* the card, mmc0:<rca> */
	if (cdev) {
		struct mmc_card *card = container_of(cdev, struct mmc_card, dev);
		const char *t = l4_card_type(card);

		if (t)
			len += scnprintf(buf + len, size - len, " mmc=ok:%s/%u", t,
					 READ_ONCE(host->ios.clock) / 1000);
		else
			len += scnprintf(buf + len, size - len, " mmc=ok:t%u/%u", card->type,
					 READ_ONCE(host->ios.clock) / 1000);
		put_device(cdev);
	} else if (cd <= 0) {
		len += scnprintf(buf + len, size - len, " mmc=none");
	} else if (atomic_read(&l4_sd_err)) {
		len += scnprintf(buf + len, size - len, " mmc=err:%d:%d",
				 atomic_read(&l4_sd_err_op), atomic_read(&l4_sd_err));
	} else {
		len += scnprintf(buf + len, size - len, " mmc=init");
	}

	put_device(hdev);
	return len;
}

/* "L4": slow printer (block class walk, regulator lock, device lookups) */
static int l4_fmt(char *buf, size_t size)
{
	int len = l4_fmt_ufs(buf, size);

	len += l4_fmt_vcc(buf + len, size - len);
	len += l4_fmt_sd(buf + len, size - len);
	return len;
}

static struct greatlte_hb_printer l4_prn = {
	.name = "L4",
	.layer = 4,
	.slow = true,
	.fmt = l4_fmt,
};

static struct device_node *l4_node(const char *path)
{
	struct device_node *np = of_find_node_by_path(path);

	if (np && !of_device_is_available(np)) {
		of_node_put(np);
		np = NULL;
	}
	return np;	/* reference kept */
}

static int __init l4_hb_init(void)
{
	l4_ufs_np = l4_node(L4_UFS_PATH);
	l4_sd_np = l4_node(L4_SD_PATH);
	if (!l4_ufs_np && !l4_sd_np)
		return 0;

	if (l4_sd_np)
		register_trace_mmc_request_done(l4_mmc_done, NULL);
	greatlte_hb_add_items(l4_rows, ARRAY_SIZE(l4_rows));
	l4_prn.every_ms = l4_hb_ms;
	greatlte_hb_register(&l4_prn);
	return 0;
}
late_initcall(l4_hb_init);
