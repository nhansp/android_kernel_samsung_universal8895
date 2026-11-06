// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2011 Google, Inc.
 * Copyright (C) Linaro 2012
 *
 * greatlte legacy ION ABI shim, page providers and heap table (spec L6A
 * 5.2.5 items 5-7, 9; 5.2.8 item 2). The system heap reproduces
 * S:drivers/staging/android/ion/ion_system_heap.c:32-35,108-224 without the
 * page pools, the CMA heaps S:drivers/staging/android/ion/ion_cma_heap.c:38-143
 * on the struct cma that rmem_cma_setup() created for each reusable
 * shared-dma-pool region (A:kernel/dma/contiguous.c:475-517, rmem->priv :510).
 */

#include <linux/cma.h>
#include <linux/device.h>
#include <linux/genalloc.h>
#include <linux/gfp.h>
#include <linux/mm.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/slab.h>
#include <linux/sort.h>
#include <linux/string.h>
#include <linux/sizes.h>

#include <asm/cacheflush.h>
#include <soc/samsung/exynos8895-smc.h>

#include "ion_legacy.h"

/* S:drivers/staging/android/ion/ion_system_heap.c:32-35; __GFP_COMP as every stock page pool (ion_page_pool.c:175) */
#define IONL_HIGH_ORDER_GFP	((GFP_HIGHUSER | __GFP_ZERO | __GFP_NOWARN | __GFP_NORETRY | \
				  __GFP_COMP) & ~__GFP_RECLAIM)
#define IONL_LOW_ORDER_GFP	(GFP_HIGHUSER | __GFP_ZERO | __GFP_NOWARN | __GFP_COMP)

static const unsigned int ionl_orders[] = { 8, 4, 0 };

/* S:drivers/staging/android/ion/ion_system_heap.c:108-130; orders > 0 use the high-order flags (:361-362) */
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

/* S:drivers/staging/android/ion/ion_system_heap.c:135-201: one sg entry per chunk */
static int ionl_system_allocate(struct ionl_buffer *buf, unsigned long size,
				unsigned long align)
{
	struct scatterlist *sg;
	struct list_head pages;
	struct page *page, *tmp_page;
	int i = 0;
	unsigned long size_remaining = PAGE_ALIGN(size);
	unsigned int max_order = ionl_orders[0];	/* fixed_max_order = orders[0], :371 */

	if (align > PAGE_SIZE)				/* :151-152 */
		return -EINVAL;

	if (size / PAGE_SIZE > totalram_pages() / 2)	/* :154-155 */
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

/* S:drivers/staging/android/ion/ion_system_heap.c:203-224, without the pools: pages go back to the buddy allocator */
static void ionl_system_free(struct ionl_buffer *buf)
{
	struct scatterlist *sg;
	int i;

	for_each_sgtable_sg(&buf->sgt, sg, i)
		__free_pages(sg_page(sg), compound_order(sg_page(sg)));
}

/* S:drivers/staging/android/ion/ion_cma_heap.c:38-118 (non-PROTECTED path), one sg entry */
static int ionl_cma_allocate(struct ionl_heap *heap, struct ionl_buffer *buf,
			     unsigned long len, unsigned long align)
{
	unsigned long count = PAGE_ALIGN(len) >> PAGE_SHIFT;
	unsigned int order = align ? get_order(align) : 0;	/* ion_cma_heap.c:65-67 */
	struct page *page;
	int ret;

	/* stock dma_alloc_from_contiguous() capped the order: S:drivers/base/dma-contiguous.c:193-194 */
	if (order > CONFIG_CMA_ALIGNMENT)
		order = CONFIG_CMA_ALIGNMENT;

	page = cma_alloc(heap->cma, count, order, false);
	if (!page)
		return -ENOMEM;				/* :68-72 */

	memset(page_address(page), 0, len);		/* :75-76; CMA pages are in the linear map */

	ret = sg_alloc_table(&buf->sgt, 1, GFP_KERNEL);
	if (ret) {
		cma_release(heap->cma, page, count);	/* :111-113 */
		return ret;
	}
	sg_set_page(buf->sgt.sgl, page, len, 0);
	buf->cma_page = page;

	return 0;
}


/*
 * The secure_camera heap (id 11, mask 0x800), iris path only (spec L6N_iris G-6).
 * Stock: a carveout heap over the "ion,secure" reserved region (FDT:42-47,
 * S:drivers/staging/android/ion/exynos/exynos_ion.c:380,451-453,474) whose
 * ION_FLAG_PROTECTED buffers are handed to EL3 with SMC_DRM_PPMP_PROT
 * (S:drivers/staging/android/ion/ion_carveout_heap.c:124-150, exynos_ion.c:208-261)
 * together with a secure IOVA from a pool the secure SysMMU context uses.  Stock
 * builds all of it under CONFIG_EXYNOS_CONTENT_PATH_PROTECTION=y (IKC:4327;
 * exynos_ion.c:126, ion.c:2675); this shim has it for the secure_camera heap only.
 */
#define IONL_PROTECTED_BUF_ALIGN	SZ_64K		/* S:include/linux/exynos_ion.h:113 */
#define IONL_SECURE_IOVA_BASE		0x10000000UL	/* S:.../exynos_ion.c:32 */
#define IONL_SECURE_IOVA_END		0xE0000000UL	/* S:.../exynos_ion.c:33 */
#define IONL_SMC_DRM_PPMP_PROT		0x82002110	/* S:include/linux/smc.h:47 */
#define IONL_SMC_DRM_PPMP_UNPROT	0x82002111	/* S:include/linux/smc.h:48 */
#define IONL_DRMDRV_OK			0		/* DRMDRV_OK, S:include/linux/smc.h:163 */

static struct gen_pool *ionl_siova_pool;

/* stock ion_secure_iova_pool_create() (S:.../exynos_ion.c:175-191); gen_pool has its own lock, stock's siova_pool_lock :129 */
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

/*
 * stock ion_carveout_heap_allocate() PROTECTED branch + ion_secure_protect()
 * (S:.../ion_carveout_heap.c:98-161, exynos_ion.c:208-261). Deviations: ionl_alloc()
 * never offers this heap to a request without ION_FLAG_PROTECTED (stock handed the
 * CPU the region's memory as the secure world left it, zeroed only at free,
 * ion_carveout_heap.c:168-179), the secure IOVA is returned when EL3
 * refuses (stock leaked it, exynos_ion.c:230-234), and an error is -errno instead
 * of the raw DRMDRV code.  Every buffer reaching here is PROTECTED.
 */
static int ionl_secure_allocate(struct ionl_heap *heap, struct ionl_buffer *buf,
				unsigned long len, unsigned long align)
{
	struct genpool_data_align data = { .align = IONL_PROTECTED_BUF_ALIGN };
	unsigned long plen = ALIGN(len, IONL_PROTECTED_BUF_ALIGN);	/* ion_carveout_heap.c:125-126 */
	unsigned long paddr, iova;
	struct ionl_prot_info *prot;
	int ret;

	if ((align > PAGE_SIZE) && (align & (align - 1)))		/* ion_carveout_heap.c:110-111 */
		return -EINVAL;

	/* kmalloc of a power-of-two size is size-aligned: the descriptor owns its cache lines */
	prot = kzalloc(ARCH_DMA_MINALIGN, GFP_KERNEL);
	if (!prot)
		return -ENOMEM;

	paddr = gen_pool_alloc_algo(heap->pool, plen, gen_pool_first_fit_align, &data);
	if (!paddr) {
		ret = -ENOMEM;
		goto err_prot;
	}

	iova = gen_pool_alloc(ionl_siova_pool, plen);			/* exynos_ion.c:219-223 */
	if (!iova) {
		pr_err("%s: no secure IOVA for %lu bytes\n", __func__, plen);
		ret = -ENOMEM;
		goto err_pool;
	}

	ret = sg_alloc_table(&buf->sgt, 1, GFP_KERNEL);
	if (ret)
		goto err_iova;
	sg_set_page(buf->sgt.sgl, pfn_to_page(PFN_DOWN(paddr)), len, 0);	/* ion_carveout_heap.c:136 */

	prot->chunk_count = 1;						/* ion_carveout_heap.c:139-143 */
	prot->flags = heap->id;
	prot->chunk_size = plen;
	prot->bus_address = paddr;
	prot->dma_addr = iova;						/* exynos_ion.c:223 */
	dcache_clean_inval_poc((unsigned long)prot, (unsigned long)prot + sizeof(*prot));

	ret = (int)exynos8895_smc(IONL_SMC_DRM_PPMP_PROT, virt_to_phys(prot), 0, 0);	/* exynos_ion.c:230 */
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
 * stock ion_carveout_heap_free() PROTECTED branch + ion_secure_unprotect()
 * (S:.../ion_carveout_heap.c:163-188, exynos_ion.c:263-304). No zeroing: the
 * memory is EL3's until SMC_DRM_PPMP_UNPROT. The secure IOVA goes back to its pool
 * whatever EL3 answers, as stock (exynos_ion.c:270-276): it is only a number of
 * the secure window, not memory.  The physical range is the fail-closed part: if
 * EL3 refuses UNPROT it stays out of the pool (still protected), where stock
 * returned it to the allocator (ion_carveout_heap.c:181-185).
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

/*
 * stock ion_carveout_heap_create() (S:.../ion_carveout_heap.c:231-262): a pool
 * over the region (:245-252), and the region cleaned and invalidated once (:257-260) so that no
 * dirty line can be written back into it after EL3 protects a buffer.
 */
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

/* L6A 5.2.5 item 9: the heap's pages, then the table */
void ionl_heap_free(struct ionl_buffer *buf)
{
	if (buf->heap->type == IONL_HEAP_SECURE) {
		ionl_secure_free(buf);
		sg_free_table(&buf->sgt);
		return;
	}

	if (buf->heap->type == IONL_HEAP_CMA)
		cma_release(buf->heap->cma, buf->cma_page,
			    PAGE_ALIGN(buf->size) >> PAGE_SHIFT);	/* ion_cma_heap.c:134-137 */
	else
		ionl_system_free(buf);

	sg_free_table(&buf->sgt);
}

/*
 * Clean+invalidate after zeroing (L6A 5.2.5 item 7), per heap:
 * system heap as stock: !CACHED || SYNC_FORCE (S:drivers/staging/android/ion/ion_system_heap.c:132-133,180-184);
 * CMA heaps: every buffer that user space maps write-combine, i.e. !CACHED and also
 * CACHED|CACHED_NEEDS_SYNC (ionl_mmap()). Stock flushed only !CACHED
 * (S:drivers/staging/android/ion/ion_cma_heap.c:83-89) although it too zeroed through the
 * cached linear map (:75-76) and mapped CACHED_NEEDS_SYNC buffers write-combine: the dirty
 * zero lines could later be evicted over data written through the WC mapping (the camera's
 * reserved cached buffers, heap 10 flags 3, F-G15, which never call ion_sync_fd). The extra
 * clean is invisible to user space. PROTECTED never gets here.
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

	/* descending id: stock plist_node_init(&heap->node, -heap->id), S:drivers/staging/android/ion/ion.c:2235 */
	return ha->id < hb->id ? 1 : (ha->id > hb->id ? -1 : 0);
}

/*
 * Heap table from DT (L6A 4.3, 5.2.8 item 2): heap 0 is the system heap,
 * one CMA heap per memory-region entry i with id samsung,ion-heap-ids[i],
 * compat ids samsung,ion-compat-ids[i] and name memory-region-names[i] "_heap".
 */
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
	heaps[0].name = "ion_system_heap";	/* libion heap-name table id 0 (L6A 5.2.8 item 2, F-G4) */

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
		/* stock "ion,secure" (S:.../exynos_ion.c:380): a carveout whose buffers EL3 protects */
		secure = of_property_read_bool(rnp, "ion,secure");
		/* only rmem_cma_setup() accepts a reusable shared-dma-pool (A:kernel/dma/contiguous.c:483-485) */
		cma_pool = !secure && of_device_is_compatible(rnp, "shared-dma-pool") &&
			   of_property_read_bool(rnp, "reusable");
		if (secure || cma_pool)
			rmem = of_reserved_mem_lookup(rnp);
		of_node_put(rnp);
		if (!rmem || (!secure && (!rmem->priv ||
					  cma_get_base(rmem->priv) != rmem->base))) {
			dev_err(dev, "memory-region %d is neither an ion,secure region nor a reusable shared-dma-pool CMA area\n", i);
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

		/* id 0 is the system heap; stock rejects ids >= ION_NUM_HEAPS (exynos_ion.c:400-404) */
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
