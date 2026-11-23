// SPDX-License-Identifier: GPL-2.0
/*
 * Legacy Exynos ION: heaps
 *
 * Copyright (C) 2011 Google, Inc.
 * Copyright (C) Linaro 2012
 */

#include <linux/cacheflush.h>
#include <linux/cma.h>
#include <linux/device.h>
#include <linux/genalloc.h>
#include <linux/gfp.h>
#include <linux/mm.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/sort.h>
#include <linux/string.h>

#include <soc/samsung/exynos8895-smc.h>

#include "ion_legacy.h"

#define IONL_HIGH_ORDER_GFP	((GFP_HIGHUSER | __GFP_ZERO | __GFP_NOWARN | __GFP_NORETRY | \
				  __GFP_COMP) & ~__GFP_RECLAIM)
#define IONL_LOW_ORDER_GFP	(GFP_HIGHUSER | __GFP_ZERO | __GFP_NOWARN | __GFP_COMP)

static const unsigned int ionl_orders[] = { 8, 4, 0 };

static struct page *ionl_alloc_largest_available(unsigned long size, unsigned int max_order)
{
	struct page *page;
	int i;

	for (i = 0; i < ARRAY_SIZE(ionl_orders); i++) {
		if (size < (PAGE_SIZE << ionl_orders[i]))
			continue;
		if (max_order < ionl_orders[i])
			continue;

		page = alloc_pages(ionl_orders[i] ? IONL_HIGH_ORDER_GFP : IONL_LOW_ORDER_GFP,
				   ionl_orders[i]);
		if (!page)
			continue;

		return page;
	}

	return NULL;
}

static int ionl_system_allocate(struct ionl_buffer *buf, unsigned long size,
				unsigned long align)
{
	struct scatterlist *sg;
	struct list_head pages;
	struct page *page, *tmp_page;
	int i = 0;
	unsigned long size_remaining = PAGE_ALIGN(size);
	unsigned int max_order = ionl_orders[0];

	if (align > PAGE_SIZE)
		return -EINVAL;

	if (size / PAGE_SIZE > totalram_pages() / 2)
		return -ENOMEM;

	INIT_LIST_HEAD(&pages);
	while (size_remaining > 0) {
		page = ionl_alloc_largest_available(size_remaining, max_order);
		if (!page)
			goto free_pages;
		list_add_tail(&page->lru, &pages);
		size_remaining -= PAGE_SIZE << compound_order(page);
		max_order = compound_order(page);
		i++;
	}

	if (sg_alloc_table(&buf->sgt, i, GFP_KERNEL))
		goto free_pages;

	sg = buf->sgt.sgl;
	list_for_each_entry_safe(page, tmp_page, &pages, lru) {
		sg_set_page(sg, page, PAGE_SIZE << compound_order(page), 0);
		sg = sg_next(sg);
		list_del(&page->lru);
	}

	return 0;

free_pages:
	list_for_each_entry_safe(page, tmp_page, &pages, lru) {
		list_del(&page->lru);
		__free_pages(page, compound_order(page));
	}

	return -ENOMEM;
}

static void ionl_system_free(struct ionl_buffer *buf)
{
	struct scatterlist *sg;
	int i;

	for_each_sgtable_sg(&buf->sgt, sg, i)
		__free_pages(sg_page(sg), compound_order(sg_page(sg)));
}

static int ionl_cma_allocate(struct ionl_heap *heap, struct ionl_buffer *buf,
			     unsigned long len, unsigned long align)
{
	unsigned long count = PAGE_ALIGN(len) >> PAGE_SHIFT;
	unsigned int order = align ? get_order(align) : 0;
	struct page *page;
	int ret;

	if (order > CONFIG_CMA_ALIGNMENT)
		order = CONFIG_CMA_ALIGNMENT;

	page = cma_alloc(heap->cma, count, order, false);
	if (!page)
		return -ENOMEM;

	memset(page_address(page), 0, len);

	ret = sg_alloc_table(&buf->sgt, 1, GFP_KERNEL);
	if (ret) {
		cma_release(heap->cma, page, count);
		return ret;
	}
	sg_set_page(buf->sgt.sgl, page, len, 0);
	buf->cma_page = page;

	return 0;
}

#define IONL_PROTECTED_BUF_ALIGN	SZ_64K
#define IONL_SECURE_IOVA_BASE		0x10000000UL
#define IONL_SECURE_IOVA_END		0xE0000000UL
#define IONL_SMC_DRM_PPMP_PROT		0x82002110
#define IONL_SMC_DRM_PPMP_UNPROT	0x82002111
#define IONL_DRMDRV_OK			0

static struct gen_pool *ionl_siova_pool;

static int ionl_siova_pool_create(void)
{
	if (ionl_siova_pool)
		return 0;

	ionl_siova_pool = gen_pool_create(PAGE_SHIFT, -1);
	if (!ionl_siova_pool)
		return -ENOMEM;

	if (gen_pool_add(ionl_siova_pool, IONL_SECURE_IOVA_BASE,
			 IONL_SECURE_IOVA_END - IONL_SECURE_IOVA_BASE, -1)) {
		gen_pool_destroy(ionl_siova_pool);
		ionl_siova_pool = NULL;
		return -ENOMEM;
	}

	return 0;
}

/* Unlike the vendor driver, the secure IOVA is given back when EL3 refuses. */
static int ionl_secure_allocate(struct ionl_heap *heap, struct ionl_buffer *buf,
				unsigned long len, unsigned long align)
{
	struct genpool_data_align data = { .align = IONL_PROTECTED_BUF_ALIGN };
	unsigned long plen = ALIGN(len, IONL_PROTECTED_BUF_ALIGN);
	unsigned long paddr, iova;
	struct ionl_prot_info *prot;
	int ret;

	if ((align > PAGE_SIZE) && (align & (align - 1)))
		return -EINVAL;

	/* a power-of-two kmalloc is size-aligned: no other data shares its cache lines */
	prot = kzalloc(ARCH_DMA_MINALIGN, GFP_KERNEL);
	if (!prot)
		return -ENOMEM;

	paddr = gen_pool_alloc_algo(heap->pool, plen, gen_pool_first_fit_align, &data);
	if (!paddr) {
		ret = -ENOMEM;
		goto err_prot;
	}

	iova = gen_pool_alloc(ionl_siova_pool, plen);
	if (!iova) {
		pr_err("%s: no secure IOVA for %lu bytes\n", __func__, plen);
		ret = -ENOMEM;
		goto err_pool;
	}

	ret = sg_alloc_table(&buf->sgt, 1, GFP_KERNEL);
	if (ret)
		goto err_iova;
	sg_set_page(buf->sgt.sgl, pfn_to_page(PFN_DOWN(paddr)), len, 0);

	prot->chunk_count = 1;
	prot->flags = heap->id;
	prot->chunk_size = plen;
	prot->bus_address = paddr;
	prot->dma_addr = iova;
	dcache_clean_inval_poc((unsigned long)prot, (unsigned long)prot + sizeof(*prot));

	ret = (int)exynos8895_smc(IONL_SMC_DRM_PPMP_PROT, virt_to_phys(prot), 0, 0);
	if (ret != IONL_DRMDRV_OK) {
		pr_err("%s: SMC_DRM_PPMP_PROT failed, err=%#x\n", __func__, ret);
		ret = -EIO;
		goto err_sgt;
	}

	buf->prot = prot;
	return 0;

err_sgt:
	sg_free_table(&buf->sgt);
err_iova:
	gen_pool_free(ionl_siova_pool, iova, plen);
err_pool:
	gen_pool_free(heap->pool, paddr, plen);
err_prot:
	kfree(prot);
	return ret;
}

/*
 * When EL3 refuses to unprotect, the range stays out of the pool, where the
 * vendor driver gave it back.
 */
static void ionl_secure_free(struct ionl_buffer *buf)
{
	struct ionl_prot_info *prot = buf->prot;
	unsigned long plen = ALIGN(buf->size, IONL_PROTECTED_BUF_ALIGN);
	int ret;

	ret = (int)exynos8895_smc(IONL_SMC_DRM_PPMP_UNPROT, virt_to_phys(prot), 0, 0);
	gen_pool_free(ionl_siova_pool, prot->dma_addr, plen);
	if (ret != IONL_DRMDRV_OK)
		pr_err("%s: SMC_DRM_PPMP_UNPROT failed, err=%#x; %#lx stays protected\n",
		       __func__, ret, prot->bus_address);
	else
		gen_pool_free(buf->heap->pool, prot->bus_address, plen);

	kfree(prot);
	buf->prot = NULL;
}

static int ionl_secure_heap_init(struct device *dev, struct ionl_heap *h,
				 struct reserved_mem *rmem)
{
	int ret;

	ret = ionl_siova_pool_create();
	if (ret)
		return ret;

	h->pool = devm_gen_pool_create(dev, PAGE_SHIFT, -1, NULL);
	if (IS_ERR(h->pool))
		return PTR_ERR(h->pool);
	ret = gen_pool_add(h->pool, rmem->base, rmem->size, -1);
	if (ret)
		return ret;

	/* no dirty line may be written back into a buffer after EL3 protects it */
	if (pfn_valid(PHYS_PFN(rmem->base)))
		dcache_clean_inval_poc((unsigned long)phys_to_virt(rmem->base),
				       (unsigned long)phys_to_virt(rmem->base) + rmem->size);

	h->type = IONL_HEAP_SECURE;
	return 0;
}

int ionl_heap_allocate(struct ionl_heap *heap, struct ionl_buffer *buf,
		       unsigned long len, unsigned long align)
{
	if (heap->type == IONL_HEAP_SECURE)
		return ionl_secure_allocate(heap, buf, len, align);

	if (heap->type == IONL_HEAP_CMA)
		return ionl_cma_allocate(heap, buf, len, align);

	return ionl_system_allocate(buf, len, align);
}

void ionl_heap_free(struct ionl_buffer *buf)
{
	if (buf->heap->type == IONL_HEAP_SECURE) {
		ionl_secure_free(buf);
		sg_free_table(&buf->sgt);
		return;
	}

	if (buf->heap->type == IONL_HEAP_CMA)
		cma_release(buf->heap->cma, buf->cma_page,
			    PAGE_ALIGN(buf->size) >> PAGE_SHIFT);
	else
		ionl_system_free(buf);

	sg_free_table(&buf->sgt);
}

/*
 * A CMA buffer is zeroed through the cached linear map. Unlike the vendor
 * driver, clean it also for ION_FLAG_CACHED_NEEDS_SYNC, which user space maps
 * write-combine: an evicted zero line would overwrite its data.
 */
bool ionl_heap_flush_at_alloc(struct ionl_buffer *buf)
{
	if (ionl_buffer_protected(buf))
		return false;

	if (!(buf->flags & ION_FLAG_CACHED))
		return true;

	if (buf->heap->type == IONL_HEAP_CMA)
		return !!(buf->flags & ION_FLAG_CACHED_NEEDS_SYNC);

	return !!(buf->flags & ION_FLAG_SYNC_FORCE);
}

static int ionl_heap_cmp(const void *a, const void *b)
{
	const struct ionl_heap *ha = a, *hb = b;

	return ha->id < hb->id ? 1 : (ha->id > hb->id ? -1 : 0);
}

int ionl_heaps_init(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct ionl_heap *heaps;
	int n, i, j;

	n = of_count_phandle_with_args(np, "memory-region", NULL);
	if (n == -ENOENT)
		n = 0;
	if (n < 0)
		return n;
	if (n > IONL_MAX_HEAPS - 1)
		return -EINVAL;

	heaps = devm_kcalloc(dev, n + 1, sizeof(*heaps), GFP_KERNEL);
	if (!heaps)
		return -ENOMEM;

	heaps[0].id = IONL_SYSTEM_HEAP_ID;
	heaps[0].type = IONL_HEAP_SYSTEM;
	heaps[0].name = "ion_system_heap";

	for (i = 0; i < n; i++) {
		struct ionl_heap *h = &heaps[i + 1];
		struct reserved_mem *rmem = NULL;
		struct device_node *rnp;
		const char *rname;
		bool cma_pool, secure;
		int ret;

		rnp = of_parse_phandle(np, "memory-region", i);
		if (!rnp)
			return -EINVAL;
		secure = of_property_read_bool(rnp, "ion,secure");
		cma_pool = !secure && of_device_is_compatible(rnp, "shared-dma-pool") &&
			   of_property_read_bool(rnp, "reusable");
		if (secure || cma_pool)
			rmem = of_reserved_mem_lookup(rnp);
		of_node_put(rnp);
		if (!rmem || (!secure && (!rmem->priv ||
					  cma_get_base(rmem->priv) != rmem->base))) {
			dev_err(dev,
				"memory-region %d is neither an ion,secure region nor a reusable shared-dma-pool CMA area\n",
				i);
			return -EINVAL;
		}

		ret = of_property_read_u32_index(np, "samsung,ion-heap-ids", i, &h->id);
		if (ret)
			return ret;
		ret = of_property_read_u32_index(np, "samsung,ion-compat-ids", i, &h->compat_ids);
		if (ret)
			return ret;
		ret = of_property_read_string_index(np, "memory-region-names", i, &rname);
		if (ret)
			return ret;

		if (h->id == IONL_SYSTEM_HEAP_ID || h->id >= IONL_NUM_HEAPS) {
			dev_err(dev, "bad heap id %u for %s\n", h->id, rname);
			return -EINVAL;
		}
		for (j = 1; j < i + 1; j++) {
			if (heaps[j].id == h->id) {
				dev_err(dev, "duplicate heap id %u\n", h->id);
				return -EINVAL;
			}
		}

		if (secure) {
			ret = ionl_secure_heap_init(dev, h, rmem);
			if (ret)
				return ret;
		} else {
			h->type = IONL_HEAP_CMA;
			h->cma = rmem->priv;
		}
		h->name = devm_kasprintf(dev, GFP_KERNEL, "%s_heap", rname);
		if (!h->name)
			return -ENOMEM;
	}

	sort(heaps, n + 1, sizeof(*heaps), ionl_heap_cmp, NULL);
	ionl_dev.heaps = heaps;
	ionl_dev.nr_heaps = n + 1;

	return 0;
}
