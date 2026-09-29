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
#include <linux/gfp.h>
#include <linux/mm.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/slab.h>
#include <linux/sort.h>
#include <linux/string.h>

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

int ionl_heap_allocate(struct ionl_heap *heap, struct ionl_buffer *buf,
		       unsigned long len, unsigned long align)
{
	if (heap->type == IONL_HEAP_CMA)
		return ionl_cma_allocate(heap, buf, len, align);

	return ionl_system_allocate(buf, len, align);
}

/* L6A 5.2.5 item 9: the heap's pages, then the table */
void ionl_heap_free(struct ionl_buffer *buf)
{
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
		bool cma_pool;
		int ret;

		rnp = of_parse_phandle(np, "memory-region", i);
		if (!rnp)
			return -EINVAL;
		/* only rmem_cma_setup() accepts a reusable shared-dma-pool (A:kernel/dma/contiguous.c:483-485) */
		cma_pool = of_device_is_compatible(rnp, "shared-dma-pool") &&
			   of_property_read_bool(rnp, "reusable");
		if (cma_pool)
			rmem = of_reserved_mem_lookup(rnp);
		of_node_put(rnp);
		if (!rmem || !rmem->priv ||
		    cma_get_base(rmem->priv) != rmem->base) {
			dev_err(dev, "memory-region %d is not a reusable shared-dma-pool CMA area\n", i);
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

		h->type = IONL_HEAP_CMA;
		h->cma = rmem->priv;
		h->name = devm_kasprintf(dev, GFP_KERNEL, "%s_heap", rname);
		if (!h->name)
			return -ENOMEM;
	}

	sort(heaps, n + 1, sizeof(*heaps), ionl_heap_cmp, NULL);
	ionl_dev.heaps = heaps;
	ionl_dev.nr_heaps = n + 1;

	return 0;
}
