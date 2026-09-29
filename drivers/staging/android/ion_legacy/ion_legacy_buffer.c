// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2011 Google, Inc.
 *
 * greatlte legacy ION ABI shim, buffers and dma-buf exporter (spec L6A
 * 5.2.5 items 7-9, 5.2.6, 5.2.7). Reproduces the buffer, dma-buf and SYNC
 * paths of S:drivers/staging/android/ion/ion.c (:399-548, :1324-1639,
 * :1784-1898) on the 6.18 dma-buf core and DMA API. Cache maintenance runs
 * on the ION platform device, which has no IOMMU and no dma-coherent (L6A
 * 4.3): its DMA addresses are physical and every dma_sync_*() is a real
 * clean (for device) or invalidate (for cpu, unless DMA_TO_DEVICE),
 * A:kernel/dma/direct.h:56-64 -> A:arch/arm64/mm/dma-mapping.c:16-33.
 */

#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/fcntl.h>
#include <linux/iosys-map.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#include "ion_legacy.h"

/* S:drivers/staging/android/ion/ion_priv.h:269-285 */
static bool ionl_buffer_cached(struct ionl_buffer *buf)
{
	return !!(buf->flags & ION_FLAG_CACHED);
}

/* stock ion_buffer_fault_user_mappings(): the "auto-sync" class (L6A 5.2.6 item 2) */
static bool ionl_buffer_autosync(struct ionl_buffer *buf)
{
	return (buf->flags & ION_FLAG_CACHED) &&
		!(buf->flags & ION_FLAG_CACHED_NEEDS_SYNC);
}

/*
 * L6A 5.2.7: clean then invalidate every page, the 6.18 form of stock's
 * __dma_flush_range() over each sg entry (S:drivers/staging/android/ion/ion.c:1816-1822).
 */
static void ionl_flush(struct ionl_buffer *buf)
{
	dma_sync_sgtable_for_device(ionl_dev.dev, &buf->sgt, DMA_BIDIRECTIONAL);
	dma_sync_sgtable_for_cpu(ionl_dev.dev, &buf->sgt, DMA_BIDIRECTIONAL);
}

/*
 * L6A 5.2.5 items 4-8, stock ion_buffer_create() (S:drivers/staging/android/ion/ion.c:399-494):
 * heap pages, one DMA mapping of the table for the ION device (L6A 5.2.5 item 8,
 * A:include/linux/dma-mapping.h:158), the allocation-time flush, then the buffer
 * becomes visible in the device list with its creator's pid (stock
 * ion_buffer_set_task_info(), S:.../ion.c:282-289, built with IKC:4141
 * CONFIG_ION_EXYNOS_STAT_LOG=y).
 */
struct ionl_buffer *ionl_buffer_create(struct ionl_heap *heap, unsigned long len,
				       unsigned long align, unsigned long flags)
{
	struct ionl_buffer *buf;
	int ret;

	buf = kzalloc(sizeof(*buf), GFP_KERNEL);
	if (!buf)
		return ERR_PTR(-ENOMEM);

	buf->heap = heap;
	buf->flags = flags;
	buf->size = len;
	kref_init(&buf->ref);
	mutex_init(&buf->lock);
	INIT_LIST_HEAD(&buf->node);

	ret = ionl_heap_allocate(heap, buf, len, align);
	if (ret)
		goto err_free;

	ret = dma_map_sgtable(ionl_dev.dev, &buf->sgt, DMA_BIDIRECTIONAL,
			      DMA_ATTR_SKIP_CPU_SYNC);
	if (ret)
		goto err_heap;

	if (ionl_heap_flush_at_alloc(buf))
		ionl_flush(buf);

	mutex_lock(&ionl_dev.buffers_lock);
	buf->pid = task_pid_nr(current->group_leader);
	list_add_tail(&buf->node, &ionl_dev.buffers);
	mutex_unlock(&ionl_dev.buffers_lock);

	atomic_inc(&heap->nr_buffers);
	atomic_long_add(buf->size, &heap->nr_bytes);
	atomic_inc(&ionl_stats.buffers);
	atomic_long_add(buf->size, &ionl_stats.bytes);

	return buf;

err_heap:
	ionl_heap_free(buf);
err_free:
	mutex_destroy(&buf->lock);
	kfree(buf);
	return ERR_PTR(ret);
}

/* stock ion_buffer_destroy() + _ion_buffer_destroy() (S:drivers/staging/android/ion/ion.c:496-539), L6A 5.2.5 item 9 */
static void ionl_buffer_release(struct kref *kref)
{
	struct ionl_buffer *buf = container_of(kref, struct ionl_buffer, ref);
	struct ionl_heap *heap = buf->heap;

	mutex_lock(&ionl_dev.buffers_lock);
	list_del(&buf->node);
	mutex_unlock(&ionl_dev.buffers_lock);

	atomic_dec(&heap->nr_buffers);
	atomic_long_sub(buf->size, &heap->nr_bytes);
	atomic_dec(&ionl_stats.buffers);
	atomic_long_sub(buf->size, &ionl_stats.bytes);

	/* the dma-buf core vunmaps before release; stock warned the same way (:505-506) */
	if (WARN_ON(buf->vmap_cnt > 0))
		vunmap(buf->vaddr);

	dma_unmap_sgtable(ionl_dev.dev, &buf->sgt, DMA_BIDIRECTIONAL, DMA_ATTR_SKIP_CPU_SYNC);
	ionl_heap_free(buf);
	mutex_destroy(&buf->lock);
	kfree(buf);
}

void ionl_buffer_get(struct ionl_buffer *buf)
{
	kref_get(&buf->ref);
}

void ionl_buffer_put(struct ionl_buffer *buf)
{
	kref_put(&buf->ref, ionl_buffer_release);
}

/* per-importer copy of the page list (L6A 5.2.6 item 1; pattern A:drivers/dma-buf/heaps/system_heap.c:107-136) */
struct ionl_attachment {
	struct sg_table table;
};

static int ionl_dma_buf_attach(struct dma_buf *dmabuf, struct dma_buf_attachment *attachment)
{
	struct ionl_buffer *buf = dmabuf->priv;
	struct ionl_attachment *a;
	struct scatterlist *sg, *new_sg;
	int ret, i;

	a = kzalloc(sizeof(*a), GFP_KERNEL);
	if (!a)
		return -ENOMEM;

	ret = sg_alloc_table(&a->table, buf->sgt.orig_nents, GFP_KERNEL);
	if (ret) {
		kfree(a);
		return ret;
	}

	new_sg = a->table.sgl;
	for_each_sgtable_sg(&buf->sgt, sg, i) {
		sg_set_page(new_sg, sg_page(sg), sg->length, sg->offset);
		new_sg = sg_next(new_sg);
	}

	attachment->priv = a;
	return 0;
}

static void ionl_dma_buf_detach(struct dma_buf *dmabuf, struct dma_buf_attachment *attachment)
{
	struct ionl_attachment *a = attachment->priv;

	sg_free_table(&a->table);
	kfree(a);
}

/*
 * L6A 5.2.6 items 2-3: stock ion_map_dma_buf() did cache work only for the
 * auto-sync class (S:drivers/staging/android/ion/ion.c:1324-1335 ->
 * ion_buffer_sync_for_device() returns at :1373-1380 otherwise), so only that
 * class maps without DMA_ATTR_SKIP_CPU_SYNC.
 */
static unsigned long ionl_map_attrs(struct ionl_buffer *buf)
{
	return ionl_buffer_autosync(buf) ? 0 : DMA_ATTR_SKIP_CPU_SYNC;
}

static struct sg_table *ionl_map_dma_buf(struct dma_buf_attachment *attachment,
					 enum dma_data_direction direction)
{
	struct ionl_buffer *buf = attachment->dmabuf->priv;
	struct ionl_attachment *a = attachment->priv;
	int ret;

	ret = dma_map_sgtable(attachment->dev, &a->table, direction, ionl_map_attrs(buf));
	if (ret)
		return ERR_PTR(ret);

	return &a->table;
}

static void ionl_unmap_dma_buf(struct dma_buf_attachment *attachment,
			       struct sg_table *table,
			       enum dma_data_direction direction)
{
	struct ionl_buffer *buf = attachment->dmabuf->priv;

	dma_unmap_sgtable(attachment->dev, table, direction, ionl_map_attrs(buf));
}

/*
 * L6A 5.2.6 item 4, stock ion_mmap() (S:drivers/staging/android/ion/ion.c:1459-1523)
 * with ion_heap_map_user() (S:drivers/staging/android/ion/ion_heap.c:72-105).
 * Page protection as stock: an uncached buffer is write-combine (ion.c:1506-1507);
 * a CMA heap buffer that is not auto-sync was mapped by dma_mmap_coherent()
 * (ion_cma_heap.c:174-183) on a non-coherent device (exynos_ion.c:435-436), whose
 * __swiotlb_mmap() made it write-combine too (S:arch/arm64/mm/dma-mapping.c:32-38,
 * 317-318). The auto-sync class is mapped cached up front instead of stock's
 * page-by-page fault path (:1494-1504) and cleaned at map_dma_buf.
 */
static int ionl_mmap(struct dma_buf *dmabuf, struct vm_area_struct *vma)
{
	struct ionl_buffer *buf = dmabuf->priv;
	struct scatterlist *sg;
	unsigned long addr = vma->vm_start;
	unsigned long offset = vma->vm_pgoff * PAGE_SIZE;
	int i, ret;

	if (buf->flags & ION_FLAG_NOZEROED)			/* :1466-1470 */
		return -EINVAL;

	if (buf->flags & ION_FLAG_PROTECTED)			/* :1472-1476 */
		return -EPERM;

	if (((vma->vm_pgoff << PAGE_SHIFT) >= buf->size) ||	/* :1478-1483 */
	    ((vma->vm_end - vma->vm_start) >
	     (buf->size - (vma->vm_pgoff << PAGE_SHIFT))))
		return -EINVAL;

	if (!ionl_buffer_cached(buf) ||
	    (buf->heap->type == IONL_HEAP_CMA && !ionl_buffer_autosync(buf)))
		vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	for_each_sgtable_sg(&buf->sgt, sg, i) {
		struct page *page = sg_page(sg);
		unsigned long remainder = vma->vm_end - addr;
		unsigned long len = sg->length;

		if (offset >= sg->length) {
			offset -= sg->length;
			continue;
		} else if (offset) {
			page += offset / PAGE_SIZE;
			len = sg->length - offset;
			offset = 0;
		}
		len = min(len, remainder);
		ret = remap_pfn_range(vma, addr, page_to_pfn(page), len,
				      vma->vm_page_prot);
		if (ret)
			return ret;
		addr += len;
		if (addr >= vma->vm_end)
			return 0;
	}

	return 0;
}

/* stock ion_dma_buf_release() (S:drivers/staging/android/ion/ion.c:1525-1530); L6A 5.2.6 item 7 */
static void ionl_dma_buf_release(struct dma_buf *dmabuf)
{
	ionl_buffer_put(dmabuf->priv);
}

/*
 * L6A 5.2.6 item 5: DMA_BUF_IOCTL_SYNC and in-kernel CPU users. A cached buffer
 * gets a real invalidate/clean; others need none. (Stock only kmapped here,
 * S:drivers/staging/android/ion/ion.c:1571-1599.)
 */
static int ionl_dma_buf_begin_cpu_access(struct dma_buf *dmabuf,
					 enum dma_data_direction direction)
{
	struct ionl_buffer *buf = dmabuf->priv;

	if (ionl_buffer_cached(buf))
		dma_sync_sgtable_for_cpu(ionl_dev.dev, &buf->sgt, direction);

	return 0;
}

static int ionl_dma_buf_end_cpu_access(struct dma_buf *dmabuf,
				       enum dma_data_direction direction)
{
	struct ionl_buffer *buf = dmabuf->priv;

	if (ionl_buffer_cached(buf))
		dma_sync_sgtable_for_device(ionl_dev.dev, &buf->sgt, direction);

	return 0;
}

/*
 * L6A 5.2.6 item 6: refcounted kernel mapping, pages and protection as stock
 * ion_heap_map_kernel() (S:drivers/staging/android/ion/ion_heap.c:29-64).
 */
static int ionl_dma_buf_vmap(struct dma_buf *dmabuf, struct iosys_map *map)
{
	struct ionl_buffer *buf = dmabuf->priv;
	unsigned int npages = PAGE_ALIGN(buf->size) / PAGE_SIZE;
	struct sg_page_iter piter;
	struct page **pages, **tmp;
	void *vaddr;
	int ret = 0;

	mutex_lock(&buf->lock);
	if (buf->vmap_cnt) {
		buf->vmap_cnt++;
		iosys_map_set_vaddr(map, buf->vaddr);
		goto out;
	}

	pages = vmalloc_array(npages, sizeof(*pages));
	if (!pages) {
		ret = -ENOMEM;
		goto out;
	}

	tmp = pages;
	for_each_sgtable_page(&buf->sgt, &piter, 0) {
		if (WARN_ON(tmp - pages >= npages))
			break;
		*tmp++ = sg_page_iter_page(&piter);
	}

	vaddr = vmap(pages, npages, VM_MAP,
		     ionl_buffer_cached(buf) ? PAGE_KERNEL : pgprot_writecombine(PAGE_KERNEL));
	vfree(pages);
	if (!vaddr) {
		ret = -ENOMEM;
		goto out;
	}

	buf->vaddr = vaddr;
	buf->vmap_cnt++;
	iosys_map_set_vaddr(map, vaddr);
out:
	mutex_unlock(&buf->lock);
	return ret;
}

static void ionl_dma_buf_vunmap(struct dma_buf *dmabuf, struct iosys_map *map)
{
	struct ionl_buffer *buf = dmabuf->priv;

	mutex_lock(&buf->lock);
	if (!WARN_ON(buf->vmap_cnt <= 0) && !--buf->vmap_cnt) {
		vunmap(buf->vaddr);
		buf->vaddr = NULL;
	}
	mutex_unlock(&buf->lock);
	iosys_map_clear(map);
}

const struct dma_buf_ops ionl_dma_buf_ops = {
	.attach = ionl_dma_buf_attach,
	.detach = ionl_dma_buf_detach,
	.map_dma_buf = ionl_map_dma_buf,
	.unmap_dma_buf = ionl_unmap_dma_buf,
	.release = ionl_dma_buf_release,
	.begin_cpu_access = ionl_dma_buf_begin_cpu_access,
	.end_cpu_access = ionl_dma_buf_end_cpu_access,
	.mmap = ionl_mmap,
	.vmap = ionl_dma_buf_vmap,
	.vunmap = ionl_dma_buf_vunmap,
};

/*
 * L6A 5.2.4 item 4: a new dma-buf for the buffer; the caller's buffer reference
 * passes to the dma-buf and is dropped in .release (stock ion_share_dma_buf(),
 * S:drivers/staging/android/ion/ion.c:1641-1672). On error the reference is dropped here.
 */
struct dma_buf *ionl_buffer_export(struct ionl_buffer *buf)
{
	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);
	struct dma_buf *dmabuf;

	exp_info.exp_name = buf->heap->name;
	exp_info.ops = &ionl_dma_buf_ops;
	exp_info.size = buf->size;
	exp_info.flags = O_RDWR;
	exp_info.priv = buf;

	dmabuf = dma_buf_export(&exp_info);
	if (IS_ERR(dmabuf))
		ionl_buffer_put(buf);

	return dmabuf;
}

/*
 * True for an ION-legacy dma-buf of the auto-sync class (CACHED without
 * CACHED_NEEDS_SYNC): an importer that keeps its mapping must clean it per use
 * with dma_buf_end_cpu_access(dmabuf, DMA_TO_DEVICE) (L6A 5.2.6 items 2 and 4).
 */
bool ionl_dmabuf_is_autosync(struct dma_buf *dmabuf)
{
	if (dmabuf->ops != &ionl_dma_buf_ops)
		return false;

	return ionl_buffer_autosync(dmabuf->priv);
}
EXPORT_SYMBOL_GPL(ionl_dmabuf_is_autosync);

/*
 * The fd's ION buffer if it needs a user-requested sync, else NULL with *ret set:
 * foreign exporter -> -EINVAL, not CACHED or auto-sync -> 0
 * (S:drivers/staging/android/ion/ion.c:1793-1810, :1842-1859).
 * On success the caller holds the dma-buf reference.
 */
static struct ionl_buffer *ionl_sync_get(int fd, struct dma_buf **dmabufp, int *ret)
{
	struct dma_buf *dmabuf;
	struct ionl_buffer *buf;

	dmabuf = dma_buf_get(fd);
	if (IS_ERR(dmabuf)) {
		*ret = PTR_ERR(dmabuf);
		return NULL;
	}

	if (dmabuf->ops != &ionl_dma_buf_ops) {
		dma_buf_put(dmabuf);
		*ret = -EINVAL;
		return NULL;
	}
	buf = dmabuf->priv;

	if (!ionl_buffer_cached(buf) || ionl_buffer_autosync(buf)) {
		dma_buf_put(dmabuf);
		*ret = 0;
		return NULL;
	}

	*dmabufp = dmabuf;
	return buf;
}

/* ION_IOC_SYNC: stock ion_sync_for_device() (S:drivers/staging/android/ion/ion.c:1784-1830) */
int ionl_sync_fd(int fd)
{
	struct dma_buf *dmabuf;
	struct ionl_buffer *buf;
	int ret;

	buf = ionl_sync_get(fd, &dmabuf, &ret);
	if (!buf)
		return ret;

	ionl_flush(buf);

	dma_buf_put(dmabuf);
	return 0;
}

/*
 * ION_IOC_SYNC_PARTIAL: clean only [offset, offset + len), stock
 * ion_sync_partial_for_device() (S:drivers/staging/android/ion/ion.c:1832-1898):
 * the __dma_map_area(..., DMA_TO_DEVICE) loop of :1868-1889 as
 * dma_sync_single_for_device() per touched entry (L6A 5.2.7,
 * A:include/linux/dma-mapping.h:425-429). A negative offset, which stock
 * turned into a clean before the first entry (:1870, :1875, :1883), is refused.
 */
int ionl_sync_partial_fd(int fd, off_t offset, size_t len)
{
	struct dma_buf *dmabuf;
	struct ionl_buffer *buf;
	struct scatterlist *sg;
	size_t remained = len;
	int i, ret;

	buf = ionl_sync_get(fd, &dmabuf, &ret);
	if (!buf)
		return ret;

	if (offset < 0) {
		dma_buf_put(dmabuf);
		return -EINVAL;
	}

	for_each_sgtable_dma_sg(&buf->sgt, sg, i) {
		size_t len_to_flush;

		if (offset >= sg_dma_len(sg)) {
			offset -= sg_dma_len(sg);
			continue;
		}

		len_to_flush = sg_dma_len(sg) - offset;
		if (remained < len_to_flush) {
			len_to_flush = remained;
			remained = 0;
		} else {
			remained -= len_to_flush;
		}

		dma_sync_single_for_device(ionl_dev.dev, sg_dma_address(sg) + offset,
					   len_to_flush, DMA_TO_DEVICE);

		if (remained == 0)
			break;
		offset = 0;
	}

	dma_buf_put(dmabuf);
	return 0;
}
