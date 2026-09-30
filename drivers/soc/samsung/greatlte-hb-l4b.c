// SPDX-License-Identifier: GPL-2.0-only
/*
 * greatlte L4b (UFS inline encryption FMP, secure-world TRNG) heartbeat rows
 * and FMP known-answer test. DEBUG ONLY, never upstream.
 * Spec: rewrite/spec/L4b_fmp_rng.md 5.5 and 5.6, through the heartbeat hooks
 * of T_tooling.md 5.7.6 as extended by FULL_IMAGE_PLAN.md 2.2 T14 (rows
 * registered from the layer's own DEBUG file, plan X-55).
 *
 * Rows, on the "hb: v1 L4f seq=<n> ..." line of every period:
 *   l4bfmp  c<UFSHCD_CAP_CRYPTO 0|1>_d<DESCTYPE read back at probe>, "-"
 *           before the 8895 UFS probe ran its system init;
 *   l4brng  n<sessions opened>_op<x1 of the last SMC_CMD_RANDOM>_r<its
 *           result>, "-" before the first call.
 *
 * With CONFIG_SCSI_UFS_EXYNOS8895_FMP_KAT: debugfs greatlte-l4b/fmp_kat
 * (0200). Writing "<major>:<minor> <sector>" of a partition on the UFS LU
 * reads 4096 bytes at that sector (512-byte units, 4 KiB aligned) without
 * and then with an inline-encryption context (raw AES-256-XTS key K[n] = n,
 * DUN 0x12345678), once per candidate key layout, and compares the results
 * with software xts(aes). Nothing is written to the flash: FMP only decrypts
 * into host memory, and the trial layout applies to this key only.
 */
#include <crypto/skcipher.h>
#include <linux/atomic.h>
#include <linux/bio.h>
#include <linux/blk-crypto.h>
#include <linux/blkdev.h>
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/soc/samsung/greatlte-hb.h>
#include <linux/soc/samsung/greatlte-l4b.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/unaligned.h>

/* ---- rows (spec 5.5) ---- */

#define L4B_HWRNG_INIT		0	/* x1 of SMC_CMD_RANDOM, S:include/linux/smc.h:126 */
#define L4B_UFSPR_DESCTYPE(v)	(((v) >> 19) & 0x3)	/* S:drivers/scsi/ufs/ufs-exynos.h:296 */

static bool l4b_fmp_seen;
static bool l4b_fmp_crypto;
static unsigned long l4b_fmp_ufspr;

static atomic_t l4b_rng_sessions = ATOMIC_INIT(0);
static bool l4b_rng_seen;
static unsigned long l4b_rng_op;
static int l4b_rng_ret;

/* process context, from the UFS probe */
void greatlte_l4b_fmp_probe(bool crypto, unsigned long ufspr_security)
{
	WRITE_ONCE(l4b_fmp_crypto, crypto);
	WRITE_ONCE(l4b_fmp_ufspr, ufspr_security);
	WRITE_ONCE(l4b_fmp_seen, true);
}
EXPORT_SYMBOL_GPL(greatlte_l4b_fmp_probe);

/* called under the TRNG spinlock with interrupts off: lock-free stores only */
void greatlte_l4b_rng_smc(unsigned long op, int ret)
{
	if (op == L4B_HWRNG_INIT && !ret)
		atomic_inc(&l4b_rng_sessions);
	WRITE_ONCE(l4b_rng_op, op);
	WRITE_ONCE(l4b_rng_ret, ret);
	WRITE_ONCE(l4b_rng_seen, true);
}
EXPORT_SYMBOL_GPL(greatlte_l4b_rng_smc);

static int l4b_fmp_fn(char *buf, size_t size)
{
	if (!READ_ONCE(l4b_fmp_seen))
		return scnprintf(buf, size, "-");

	return scnprintf(buf, size, "c%d_d%lu", READ_ONCE(l4b_fmp_crypto),
			 L4B_UFSPR_DESCTYPE(READ_ONCE(l4b_fmp_ufspr)));
}

static int l4b_rng_fn(char *buf, size_t size)
{
	if (!READ_ONCE(l4b_rng_seen))
		return scnprintf(buf, size, "-");

	return scnprintf(buf, size, "n%d_op%lu_r%d",
			 atomic_read(&l4b_rng_sessions), READ_ONCE(l4b_rng_op),
			 READ_ONCE(l4b_rng_ret));
}

static const struct greatlte_hb_item l4b_rows[] = {
	{ .layer = 4, .tag = "l4bfmp", .kind = GREATLTE_HB_FN, .fn = l4b_fmp_fn },
	{ .layer = 4, .tag = "l4brng", .kind = GREATLTE_HB_FN, .fn = l4b_rng_fn },
};

/* ---- FMP known-answer test (spec 5.6) ---- */

#if IS_ENABLED(CONFIG_SCSI_UFS_EXYNOS8895_FMP_KAT)

#define L4B_KAT_LEN	4096		/* one FMP data unit (DATA_UNIT_SIZE) */
#define L4B_KAT_UNIT	512		/* bitmap granule, stock sector unit */
#define L4B_KAT_UNITS	(L4B_KAT_LEN / L4B_KAT_UNIT)
#define L4B_KAT_DUN	0x12345678ULL
#define L4B_KAT_KEYLEN	64		/* AES-256-XTS */

static DEFINE_MUTEX(l4b_kat_lock);	/* one KAT at a time: one armed key */

static const struct {
	u8 layout;
	const char *name;
} l4b_kat_layouts[] = {
	{ GREATLTE_L4B_KAT_WHOLE_REVERSED, "stock" },
	{ GREATLTE_L4B_KAT_HALVES, "gs101" },
};

/* read L4B_KAT_LEN bytes at @sector into @page, with @key's context if set */
static int l4b_kat_read(struct block_device *bdev, sector_t sector,
			struct page *page, const struct blk_crypto_key *key)
{
	struct bio *bio;
	int ret;

	bio = bio_alloc(bdev, 1, REQ_OP_READ, GFP_KERNEL);
	bio->bi_iter.bi_sector = sector;
	__bio_add_page(bio, page, L4B_KAT_LEN, 0);
	if (key) {
		u64 dun[BLK_CRYPTO_DUN_ARRAY_SIZE] = { L4B_KAT_DUN };

		bio_crypt_set_ctx(bio, key, dun, GFP_KERNEL);
	}
	ret = submit_bio_wait(bio);
	bio_put(bio);

	return ret;
}

/* software xts(aes) decryption of @len bytes in place, fscrypt IV layout */
static int l4b_kat_sw(struct crypto_skcipher *tfm, u8 *buf, unsigned int len,
		      u64 index)
{
	struct skcipher_request *req;
	DECLARE_CRYPTO_WAIT(wait);
	struct scatterlist sg;
	u8 iv[16] = {};
	int ret;

	req = skcipher_request_alloc(tfm, GFP_KERNEL);
	if (!req)
		return -ENOMEM;

	/* IV = le64(index), then zeroes: A:fs/crypto/crypto.c:94-106 */
	put_unaligned_le64(index, iv);
	sg_init_one(&sg, buf, len);
	skcipher_request_set_callback(req, CRYPTO_TFM_REQ_MAY_BACKLOG |
					   CRYPTO_TFM_REQ_MAY_SLEEP,
				      crypto_req_done, &wait);
	skcipher_request_set_crypt(req, &sg, &sg, len, iv);
	ret = crypto_wait_req(crypto_skcipher_decrypt(req), &wait);
	skcipher_request_free(req);

	return ret;
}

/* bit k set: 512-byte unit k of @p equals that of @s */
static u8 l4b_kat_match(const u8 *p, const u8 *s)
{
	u8 m = 0;
	int k;

	for (k = 0; k < L4B_KAT_UNITS; k++)
		if (!memcmp(p + k * L4B_KAT_UNIT, s + k * L4B_KAT_UNIT,
			    L4B_KAT_UNIT))
			m |= BIT(k);

	return m;
}

/*
 * Buffers: c = ciphertext (plain read), p[i] = FMP output with layout i,
 * s4k = software, one 4096-byte unit (fscrypt model), s512 = software,
 * eight 512-byte units with IV index + k (stock-hardware model).
 */
struct l4b_kat_bufs {
	u8 c[L4B_KAT_LEN];
	u8 p[ARRAY_SIZE(l4b_kat_layouts)][L4B_KAT_LEN];
	u8 s4k[L4B_KAT_LEN];
	u8 s512[L4B_KAT_LEN];
	u8 raw[L4B_KAT_KEYLEN];
	struct blk_crypto_key key;
};

static int l4b_kat_compare(struct l4b_kat_bufs *b)
{
	u8 match[ARRAY_SIZE(l4b_kat_layouts)], match512;
	struct crypto_skcipher *tfm;
	const char *result;
	int ret, i, k;

	tfm = crypto_alloc_skcipher("xts(aes)", 0, 0);
	if (IS_ERR(tfm))
		return PTR_ERR(tfm);

	ret = crypto_skcipher_setkey(tfm, b->raw, L4B_KAT_KEYLEN);
	if (ret)
		goto out;

	memcpy(b->s4k, b->c, L4B_KAT_LEN);
	ret = l4b_kat_sw(tfm, b->s4k, L4B_KAT_LEN, L4B_KAT_DUN);
	if (ret)
		goto out;

	memcpy(b->s512, b->c, L4B_KAT_LEN);
	for (k = 0; k < L4B_KAT_UNITS; k++) {
		ret = l4b_kat_sw(tfm, b->s512 + k * L4B_KAT_UNIT, L4B_KAT_UNIT,
				 L4B_KAT_DUN + k);
		if (ret)
			goto out;
	}

	for (i = 0; i < ARRAY_SIZE(l4b_kat_layouts); i++) {
		match[i] = l4b_kat_match(b->p[i], b->s4k);
		match512 = l4b_kat_match(b->p[i], b->s512);
		pr_info("l4b: fmp_kat layout=%s match=%02x match512=%02x\n",
			l4b_kat_layouts[i].name, match[i], match512);
	}

	/* a layout counts only with a full match and no full match for the other */
	if (match[0] == 0xff && match[1] == 0xff)
		result = "both";
	else if (match[0] == 0xff)
		result = l4b_kat_layouts[0].name;
	else if (match[1] == 0xff)
		result = l4b_kat_layouts[1].name;
	else
		result = "none";
	pr_info("l4b: fmp_kat result=%s\n", result);

out:
	crypto_free_skcipher(tfm);
	return ret;
}

static int l4b_kat_run(dev_t dev, sector_t sector)
{
	struct block_device *bdev;
	struct l4b_kat_bufs *b;
	struct page *page;
	struct file *bf;
	int ret, i, n;

	bf = bdev_file_open_by_dev(dev, BLK_OPEN_READ, NULL, NULL);
	if (IS_ERR(bf))
		return PTR_ERR(bf);
	bdev = file_bdev(bf);

	/* one whole, 4 KiB aligned logical block inside the partition */
	if (sector % (L4B_KAT_LEN >> SECTOR_SHIFT) ||
	    sector + (L4B_KAT_LEN >> SECTOR_SHIFT) > bdev_nr_sectors(bdev)) {
		ret = -EINVAL;
		goto out_file;
	}

	b = kzalloc(sizeof(*b), GFP_KERNEL);
	page = alloc_page(GFP_KERNEL);
	if (!b || !page) {
		ret = -ENOMEM;
		goto out_mem;
	}

	/* ciphertext as stored */
	ret = l4b_kat_read(bdev, sector, page, NULL);
	if (ret)
		goto out_mem;
	memcpy_from_page(b->c, page, 0, L4B_KAT_LEN);

	for (n = 0; n < L4B_KAT_KEYLEN; n++)
		b->raw[n] = n;
	ret = blk_crypto_init_key(&b->key, b->raw, L4B_KAT_KEYLEN,
				  BLK_CRYPTO_KEY_TYPE_RAW,
				  BLK_ENCRYPTION_MODE_AES_256_XTS, sizeof(u32),
				  L4B_KAT_LEN);
	if (ret)
		goto out_mem;

	/* a fallback read would decrypt in software: both layouts would match */
	if (!blk_crypto_config_supported_natively(bdev, &b->key.crypto_cfg)) {
		pr_info("l4b: fmp_kat native=0 (fallback)\n");
		ret = -EOPNOTSUPP;
		goto out_mem;
	}
	pr_info("l4b: fmp_kat native=1\n");

	ret = blk_crypto_start_using_key(bdev, &b->key);
	if (ret)
		goto out_mem;

	for (i = 0; i < ARRAY_SIZE(l4b_kat_layouts); i++) {
		exynos8895_ufs_fmp_kat_arm(&b->key, l4b_kat_layouts[i].layout);
		ret = l4b_kat_read(bdev, sector, page, &b->key);
		exynos8895_ufs_fmp_kat_disarm();
		if (ret)
			goto out_evict;
		memcpy_from_page(b->p[i], page, 0, L4B_KAT_LEN);
	}

	ret = l4b_kat_compare(b);

out_evict:
	blk_crypto_evict_key(bdev, &b->key);
out_mem:
	if (page)
		__free_page(page);
	kfree_sensitive(b);
out_file:
	fput(bf);
	return ret;
}

static ssize_t l4b_kat_write(struct file *file, const char __user *ubuf,
			     size_t count, loff_t *ppos)
{
	unsigned long long sector;
	unsigned int maj, min;
	char buf[48];
	int ret;

	if (!count || count >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;
	buf[count] = '\0';

	if (sscanf(buf, "%u:%u %llu", &maj, &min, &sector) != 3)
		return -EINVAL;
	if (maj >= (1U << (32 - MINORBITS)) || min > MINORMASK)
		return -EINVAL;

	mutex_lock(&l4b_kat_lock);
	ret = l4b_kat_run(MKDEV(maj, min), sector);
	mutex_unlock(&l4b_kat_lock);

	return ret ? ret : count;
}

static const struct file_operations l4b_kat_fops = {
	.owner	= THIS_MODULE,
	.open	= simple_open,
	.write	= l4b_kat_write,
	.llseek	= noop_llseek,
};

static void __init l4b_kat_init(void)
{
	struct dentry *dir = debugfs_create_dir("greatlte-l4b", NULL);

	debugfs_create_file("fmp_kat", 0200, dir, NULL, &l4b_kat_fops);
}
#else
static inline void l4b_kat_init(void)
{
}
#endif /* CONFIG_SCSI_UFS_EXYNOS8895_FMP_KAT */

static int __init l4b_hb_init(void)
{
	greatlte_hb_add_items(l4b_rows, ARRAY_SIZE(l4b_rows));
	l4b_kat_init();
	return 0;
}
late_initcall(l4b_hb_init);
