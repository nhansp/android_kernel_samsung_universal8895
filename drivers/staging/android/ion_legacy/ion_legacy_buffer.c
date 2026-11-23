// SPDX-License-Identifier: GPL-2.0
/*
 * Legacy Exynos ION: buffers and their dma-buf exporter
 *
 * Copyright (C) 2011 Google, Inc.
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

static bool ionl_buffer_cached(struct ionl_buffer *buf)
{
	return !!(buf->flags & ION_FLAG_CACHED);
}

/* Cached, but user space never syncs it: the exporter keeps it clean for devices. */
static bool ionl_buffer_autosync(struct ionl_buffer *buf)
{
	return (buf->flags & ION_FLAG_CACHED) &&
		!(buf->flags & ION_FLAG_CACHED_NEEDS_SYNC);
}

static void ionl_flush(struct ionl_buffer *buf)
{
	dma_sync_sgtable_for_device(ionl_dev.dev, &buf->sgt, DMA_BIDIRECTIONAL);
	dma_sync_sgtable_for_cpu(ionl_dev.dev, &buf->sgt, DMA_BIDIRECTIONAL);
}

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

	if (!ionl_buffer_protected(buf)) {
		ret = dma_map_sgtable(ionl_dev.dev, &buf->sgt, DMA_BIDIRECTIONAL,
				      DMA_ATTR_SKIP_CPU_SYNC);
		if (ret)
			goto err_heap;

		if (ionl_heap_flush_at_alloc(buf))
			ionl_flush(buf);
	}

	mutex_lock(&ionl_dev.buffers_lock);
	buf->pid = task_pid_nr(current->group_leader);
	list_add_tail(&buf->node, &ionl_dev.buffers);
	mutex_unlock(&ionl_dev.buffers_lock);

	atomic_inc(&heap->nr_buffers);
	atomic_long_add(buf->size, &heap->nr_bytes);

	return buf;

err_heap:
	ionl_heap_free(buf);
err_free:
	mutex_destroy(&buf->lock);
	kfree(buf);
	return ERR_PTR(ret);
}

static void ionl_buffer_release(struct kref *kref)
{
	struct ionl_buffer *buf = container_of(kref, struct ionl_buffer, ref);
	struct ionl_heap *heap = buf->heap;

	mutex_lock(&ionl_dev.buffers_lock);
	list_del(&buf->node);
	mutex_unlock(&ionl_dev.buffers_lock);

	atomic_dec(&heap->nr_buffers);
	atomic_long_sub(buf->size, &heap->nr_bytes);

	if (WARN_ON(buf->vmap_cnt > 0))
		vunmap(buf->vaddr);

	if (!ionl_buffer_protected(buf))
		dma_unmap_sgtable(ionl_dev.dev, &buf->sgt, DMA_BIDIRECTIONAL,
				  DMA_ATTR_SKIP_CPU_SYNC);
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

/* Only the auto-sync class gets cache maintenance at map time, as in the vendor driver. */
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

	/* devices address a protected buffer by the secure IOVA given to EL3 */
	if (ionl_buffer_protected(buf)) {
		if (!buf->prot || !buf->prot->dma_addr)
			return ERR_PTR(-EINVAL);
		sg_dma_address(a->table.sgl) = buf->prot->dma_addr;
		sg_dma_len(a->table.sgl) = buf->size;
		return &a->table;
	}

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

	if (ionl_buffer_protected(buf))
		return;

	dma_unmap_sgtable(attachment->dev, table, direction, ionl_map_attrs(buf));
}

/*
 * Uncached buffers, and CMA buffers outside the auto-sync class, are mapped
 * write-combine as the vendor driver did. The auto-sync class is mapped cached
 * up front instead of the vendor's page-by-page fault path.
 */
static int ionl_mmap(struct dma_buf *dmabuf, struct vm_area_struct *vma)
{
	struct ionl_buffer *buf = dmabuf->priv;
	struct scatterlist *sg;
	unsigned long addr = vma->vm_start;
	unsigned long offset = vma->vm_pgoff * PAGE_SIZE;
	int i, ret;

	if (buf->flags & ION_FLAG_NOZEROED)
		return -EINVAL;

	if (buf->flags & ION_FLAG_PROTECTED)
		return -EPERM;

	if (((vma->vm_pgoff << PAGE_SHIFT) >= buf->size) ||
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

static void ionl_dma_buf_release(struct dma_buf *dmabuf)
{
	ionl_buffer_put(dmabuf->priv);
}

/*
 * The vendor driver did no cache maintenance here, and allowed CPU access to
 * protected buffers.
 */
static int ionl_dma_buf_begin_cpu_access(struct dma_buf *dmabuf,
					 enum dma_data_direction direction)
{
	struct ionl_buffer *buf = dmabuf->priv;

	if (ionl_buffer_protected(buf))
		return -EPERM;

	if (ionl_buffer_cached(buf))
		dma_sync_sgtable_for_cpu(ionl_dev.dev, &buf->sgt, direction);

	return 0;
}

static int ionl_dma_buf_end_cpu_access(struct dma_buf *dmabuf,
				       enum dma_data_direction direction)
{
	struct ionl_buffer *buf = dmabuf->priv;

	if (ionl_buffer_protected(buf))
		return -EPERM;

	if (ionl_buffer_cached(buf))
		dma_sync_sgtable_for_device(ionl_dev.dev, &buf->sgt, direction);

	return 0;
}

static int ionl_dma_buf_vmap(struct dma_buf *dmabuf, struct iosys_map *map)
{
	struct ionl_buffer *buf = dmabuf->priv;
	unsigned int npages = PAGE_ALIGN(buf->size) / PAGE_SIZE;
	struct sg_page_iter piter;
	struct page **pages, **tmp;
	void *vaddr;
	int ret = 0;

	if (ionl_buffer_protected(buf))
		return -EPERM;

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

/* The dma-buf takes over the caller's buffer reference; on error it is dropped. */
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

/**
 * ionl_dmabuf_is_autosync() - check for an auto-sync legacy ION buffer
 * @dmabuf: the dma-buf to check
 *
 * Return: true if @dmabuf was exported by this driver with ION_FLAG_CACHED
 * and without ION_FLAG_CACHED_NEEDS_SYNC.
 */
bool ionl_dmabuf_is_autosync(struct dma_buf *dmabuf)
{
	if (dmabuf->ops != &ionl_dma_buf_ops)
		return false;

	return ionl_buffer_autosync(dmabuf->priv);
}
EXPORT_SYMBOL_GPL(ionl_dmabuf_is_autosync);

/*
 * The fd's buffer if it needs a sync, else NULL with *ret set: -EINVAL for a
 * foreign dma-buf, 0 for a buffer that needs none. On success the caller holds
 * the dma-buf reference.
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

	if (!ionl_buffer_cached(buf) || ionl_buffer_autosync(buf) ||
	    ionl_buffer_protected(buf)) {
		dma_buf_put(dmabuf);
		*ret = 0;
		return NULL;
	}

	*dmabufp = dmabuf;
	return buf;
}

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

/* The vendor driver cleaned from before the buffer for a negative offset; refuse it. */
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
