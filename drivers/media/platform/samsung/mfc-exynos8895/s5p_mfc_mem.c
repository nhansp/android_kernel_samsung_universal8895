/*
 * drivers/media/platform/exynos/mfc/s5p_mfc_mem.c
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/dma-buf.h>
#include <linux/iommu.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>

#include "s5p_mfc_mem.h"

/*
 * Memory layer on vb2-dma-contig and the DMA API (spec L6B B-MFC-2, B-V7,
 * B-V9). Stock: vb2_ion (S:drivers/media/platform/exynos/mfc/s5p_mfc_mem.c).
 */

void *s5p_mfc_mem_alloc(struct device *dev, size_t size)
{
	struct s5p_mfc_mem_cookie *cookie;

	cookie = kzalloc(sizeof(*cookie), GFP_KERNEL);
	if (!cookie)
		return ERR_PTR(-ENOMEM);

	cookie->vaddr = dma_alloc_attrs(dev, size, &cookie->daddr, GFP_KERNEL,
					DMA_ATTR_WRITE_COMBINE);
	if (!cookie->vaddr) {
		kfree(cookie);
		return ERR_PTR(-ENOMEM);
	}
	cookie->dev = dev;
	cookie->size = size;

	return cookie;
}

/*
 * Map an allocation again at a fixed IOVA of the device's DMA domain. The DT
 * reserves that IOVA from dma-iommu (memory-region with iommu-addresses), so
 * nothing else is mapped there; the DMA API mapping is kept for the free.
 */
int s5p_mfc_mem_map_fixed(void *cookie, dma_addr_t iova)
{
	struct s5p_mfc_mem_cookie *c = cookie;
	struct iommu_domain *domain = iommu_get_domain_for_dev(c->dev);
	struct sg_table sgt;
	ssize_t mapped;
	int ret;

	if (!domain)
		return -ENODEV;

	ret = dma_get_sgtable_attrs(c->dev, &sgt, c->vaddr, c->daddr, c->size,
				    DMA_ATTR_WRITE_COMBINE);
	if (ret)
		return ret;

	mapped = iommu_map_sgtable(domain, iova, &sgt, IOMMU_READ | IOMMU_WRITE);
	sg_free_table(&sgt);
	if (mapped < 0)
		return mapped;
	if (mapped < c->size) {
		iommu_unmap(domain, iova, mapped);
		return -ENOMEM;
	}

	c->fixed = iova;
	return 0;
}

void s5p_mfc_mem_free(void *cookie)
{
	struct s5p_mfc_mem_cookie *c = cookie;

	if (IS_ERR_OR_NULL(c))
		return;

	if (c->fixed)
		iommu_unmap(iommu_get_domain_for_dev(c->dev), c->fixed,
			    PAGE_ALIGN(c->size));
	dma_free_attrs(c->dev, c->size, c->vaddr, c->daddr, DMA_ATTR_WRITE_COMBINE);
	kfree(c);
}

const struct vb2_mem_ops *s5p_mfc_mem_ops(void)
{
	return &vb2_dma_contig_memops;
}

/*
 * The stock hint switched the vb2_ion context between cached and uncached
 * MMAP allocations (vb2_ion_set_cached). vb2-dma-contig MMAP buffers are
 * DMA-coherent allocations, so there is nothing to switch; the control value
 * is still stored and reported (V4L2_CID_CACHEABLE).
 */
void s5p_mfc_mem_set_cacheable(struct device *dev, bool cacheable)
{
}

/*
 * Internal buffers are write-combined: CPU writes reach memory in order before
 * the MFC is started through writel(), which orders them; nothing to clean or
 * invalidate (stock synced its cached firmware buffer here).
 */
void s5p_mfc_mem_clean(void *vb_priv, void *start, off_t offset,
							size_t size)
{
}

void s5p_mfc_mem_invalidate(void *vb_priv, void *start, off_t offset,
							size_t size)
{
}

static enum dma_data_direction mfc_mem_vb_dir(struct vb2_buffer *vb)
{
	return V4L2_TYPE_IS_OUTPUT(vb->type) ? DMA_TO_DEVICE : DMA_FROM_DEVICE;
}

/*
 * DMABUF planes: 6.18 vb2 maps them once and does no cache maintenance
 * (A:drivers/media/common/videobuf2/videobuf2-core.c:1472-1483), stock synced
 * every plane for the device before use and for the CPU after it
 * (S:drivers/media/v4l2-core/videobuf2-ion.c:916-950). The exporter (L6A ION)
 * does the maintenance in its cpu_access ops. MMAP and USERPTR planes are
 * synced by vb2-dma-contig itself.
 */
static void mfc_mem_vb_for_device(struct vb2_buffer *vb, u32 num_planes)
{
	enum dma_data_direction dir = mfc_mem_vb_dir(vb);
	unsigned int i;

	if (vb->memory != VB2_MEMORY_DMABUF)
		return;

	for (i = 0; i < num_planes && i < vb->num_planes; i++)
		if (vb->planes[i].dbuf)
			dma_buf_end_cpu_access(vb->planes[i].dbuf, dir);
}

static void mfc_mem_vb_for_cpu(struct vb2_buffer *vb, u32 num_planes,
			       enum dma_data_direction dir)
{
	unsigned int i;

	if (vb->memory != VB2_MEMORY_DMABUF)
		return;

	for (i = 0; i < num_planes && i < vb->num_planes; i++)
		if (vb->planes[i].dbuf)
			dma_buf_begin_cpu_access(vb->planes[i].dbuf, dir);
}

int s5p_mfc_mem_buf_prepare(struct vb2_buffer *vb)
{
	mfc_mem_vb_for_device(vb, vb->num_planes);

	return 0;
}

void s5p_mfc_mem_buf_finish(struct vb2_buffer *vb)
{
	mfc_mem_vb_for_cpu(vb, vb->num_planes, mfc_mem_vb_dir(vb));
}

/* stock: vb2_ion_sync_for_device(cookie, 0, length, DMA_TO_DEVICE) per plane */
int s5p_mfc_mem_clean_vb(struct vb2_buffer *vb, u32 num_planes)
{
	mfc_mem_vb_for_device(vb, num_planes);

	return 0;
}

/* stock: vb2_ion_sync_for_device(cookie, 0, length, DMA_FROM_DEVICE) per plane */
int s5p_mfc_mem_inv_vb(struct vb2_buffer *vb, u32 num_planes)
{
	mfc_mem_vb_for_cpu(vb, num_planes, DMA_FROM_DEVICE);

	return 0;
}

/*
 * User shared handle (V4L2_CID_MPEG_MFC_SET_USER_SHARED_HANDLE): a dma-buf fd
 * the driver writes the DPB reference information into. Stock imported it
 * into its ION client and mapped it with ion_map_kernel(); a handle set again
 * replaced the old one without releasing it, here the old one is released
 * first.
 */
int s5p_mfc_mem_get_user_shared_handle(struct s5p_mfc_ctx *ctx,
	struct mfc_user_shared_handle *handle)
{
	int fd = handle->fd;
	int ret;

	if (handle->dmabuf) {
		handle->fd = 0;
		s5p_mfc_mem_cleanup_user_shared_handle(ctx, handle);
		handle->fd = fd;
	}

	handle->dmabuf = dma_buf_get(handle->fd);
	if (IS_ERR(handle->dmabuf)) {
		mfc_err_ctx("Failed to import fd\n");
		ret = PTR_ERR(handle->dmabuf);
		handle->dmabuf = NULL;
		return ret;
	}

	ret = dma_buf_vmap_unlocked(handle->dmabuf, &handle->map);
	if (ret || handle->map.is_iomem) {
		mfc_err_ctx("Failed to get kernel virtual address\n");
		if (!ret)
			dma_buf_vunmap_unlocked(handle->dmabuf, &handle->map);
		dma_buf_put(handle->dmabuf);
		handle->dmabuf = NULL;
		return -EINVAL;
	}
	handle->vaddr = handle->map.vaddr;

	mfc_debug(2, "User Handle: fd = %d, virtual addr = 0x%p\n",
				handle->fd, handle->vaddr);

	return 0;
}

int s5p_mfc_mem_cleanup_user_shared_handle(struct s5p_mfc_ctx *ctx,
		struct mfc_user_shared_handle *handle)
{
	if (handle->fd == -1 || !handle->dmabuf)
		return 0;

	if (handle->vaddr)
		dma_buf_vunmap_unlocked(handle->dmabuf, &handle->map);

	dma_buf_put(handle->dmabuf);
	handle->dmabuf = NULL;
	handle->vaddr = NULL;

	return 0;
}

/*
 * The decoder gives the firmware its own mapping of every DMABUF capture
 * buffer instead of the vb2 one. The firmware reads a decoded picture as a
 * reference until it reports the index released (dynamic_used), and user
 * space may queue another dma-buf at the index before that, at which point
 * vb2 unmaps the old one. Samsung's kernel kept the old buffer mapped
 * (s5p_mfc_get_iovmm()/s5p_mfc_put_iovmm() in
 * drivers/media/platform/exynos/mfc/s5p_mfc_mem.c, called from
 * s5p_mfc_store_dpb()); its ION IOVA manager had one address per dma-buf, so
 * that mapping was the one the firmware had been given. Here every
 * attachment has its own IOVA, so the pinned mapping has to be the address
 * written to the firmware.
 */
static bool mfc_dpb_pin_valid(const struct s5p_mfc_dpb_pin *pin)
{
	return pin->dbuf[0];
}

static void mfc_dpb_pin_release(struct s5p_mfc_dpb_pin *pin)
{
	int i;

	for (i = 0; i < MFC_MAX_PLANES; i++) {
		if (pin->sgt[i])
			dma_buf_unmap_attachment_unlocked(pin->attach[i],
							  pin->sgt[i], pin->dir);
		if (pin->attach[i])
			dma_buf_detach(pin->dbuf[i], pin->attach[i]);
		if (pin->dbuf[i])
			dma_buf_put(pin->dbuf[i]);
	}
	memset(pin, 0, sizeof(*pin));
}

static unsigned long mfc_dpb_contiguous_size(struct sg_table *sgt)
{
	dma_addr_t expected = sg_dma_address(sgt->sgl);
	unsigned long size = 0;
	struct scatterlist *sg;
	unsigned int i;

	for_each_sgtable_dma_sg(sgt, sg, i) {
		if (sg_dma_address(sg) != expected)
			break;
		expected += sg_dma_len(sg);
		size += sg_dma_len(sg);
	}

	return size;
}

static int mfc_dpb_pin_acquire(struct s5p_mfc_ctx *ctx, struct vb2_buffer *vb,
			       struct s5p_mfc_dpb_pin *pin)
{
	struct s5p_mfc_dev *dev = ctx->dev;
	int i, ret;

	if (vb->num_planes > MFC_MAX_PLANES)
		return -EINVAL;

	pin->dir = vb->vb2_queue->dma_dir;

	for (i = 0; i < vb->num_planes; i++) {
		struct dma_buf_attachment *attach;
		struct sg_table *sgt;

		if (!vb->planes[i].dbuf) {
			ret = -EINVAL;
			goto err;
		}
		get_dma_buf(vb->planes[i].dbuf);
		pin->dbuf[i] = vb->planes[i].dbuf;

		attach = dma_buf_attach(pin->dbuf[i], dev->device);
		if (IS_ERR(attach)) {
			ret = PTR_ERR(attach);
			mfc_err_ctx("[IOVMM] Failed to attach plane %d (err %d)\n",
				    i, ret);
			goto err;
		}
		pin->attach[i] = attach;

		sgt = dma_buf_map_attachment_unlocked(attach, pin->dir);
		if (IS_ERR(sgt)) {
			ret = PTR_ERR(sgt);
			mfc_err_ctx("[IOVMM] Failed to map plane %d (err %d)\n",
				    i, ret);
			goto err;
		}
		pin->sgt[i] = sgt;

		if (mfc_dpb_contiguous_size(sgt) < vb2_plane_size(vb, i)) {
			mfc_err_ctx("[IOVMM] plane %d is not contiguous in IOVA\n", i);
			ret = -EFAULT;
			goto err;
		}
		pin->addr[i] = sg_dma_address(sgt->sgl);

		mfc_debug(2, "[IOVMM] index %d buf[%d] addr: %#llx\n",
			  vb->index, i, pin->addr[i]);
	}

	return 0;

err:
	mfc_dpb_pin_release(pin);
	return ret;
}

static bool mfc_dpb_pin_same(const struct s5p_mfc_dpb_pin *pin,
			     struct vb2_buffer *vb)
{
	int i;

	for (i = 0; i < vb->num_planes; i++)
		if (pin->dbuf[i] != vb->planes[i].dbuf)
			return false;

	return true;
}

/* The index is with the firmware or holds one of its references */
static bool mfc_dpb_fw_may_ref(struct s5p_mfc_dec *dec, int index)
{
	return test_bit(index, &dec->available_dpb) ||
		(dec->dynamic_used & BIT(index));
}

static void mfc_dpb_pin_sweep(struct s5p_mfc_dec *dec)
{
	int i;

	for (i = 0; i < MFC_MAX_DPBS; i++)
		if (mfc_dpb_pin_valid(&dec->dpb_pin_prev[i]) &&
		    !mfc_dpb_fw_may_ref(dec, i))
			mfc_dpb_pin_release(&dec->dpb_pin_prev[i]);
}

/*
 * Pin the dma-bufs queued at vb->index and return the addresses of their
 * memory planes for the firmware.
 */
int s5p_mfc_dpb_pin_get(struct s5p_mfc_ctx *ctx, struct vb2_buffer *vb,
			dma_addr_t addr[MFC_MAX_PLANES])
{
	struct s5p_mfc_dev *dev = ctx->dev;
	struct s5p_mfc_dec *dec = ctx->dec_priv;
	struct s5p_mfc_dpb_pin fresh = {};
	struct s5p_mfc_dpb_pin *cur, *prev;
	int index = vb->index;
	int i, ret = 0;

	if (index >= MFC_MAX_DPBS)
		return -EINVAL;

	mutex_lock(&dec->dpb_mutex);

	mfc_dpb_pin_sweep(dec);

	cur = &dec->dpb_pin[index];
	prev = &dec->dpb_pin_prev[index];
	if (!mfc_dpb_pin_valid(cur) || !mfc_dpb_pin_same(cur, vb)) {
		ret = mfc_dpb_pin_acquire(ctx, vb, &fresh);
		if (ret)
			goto out;

		if (mfc_dpb_pin_valid(cur)) {
			mfc_debug(2, "[IOVMM] DPB[%d] was changed %#llx->%#llx (used: %#x)\n",
				  index, cur->addr[0], fresh.addr[0],
				  dec->dynamic_used);
			MFC_TRACE_CTX("DPB[%d] %#llx->%#llx (%#x)\n",
				      index, cur->addr[0], fresh.addr[0],
				      dec->dynamic_used);

			if (!mfc_dpb_fw_may_ref(dec, index)) {
				mfc_dpb_pin_release(cur);
				mfc_dpb_pin_release(prev);
			} else if (test_bit(index, &dec->dpb_sent)) {
				/*
				 * The buffer replaced before cur was decoded
				 * over by cur, which is back with user space.
				 */
				mfc_dpb_pin_release(prev);
				*prev = *cur;
			} else {
				/* never given to the firmware */
				mfc_dpb_pin_release(cur);
			}
		}
		*cur = fresh;
		clear_bit(index, &dec->dpb_sent);
	}

	for (i = 0; i < vb->num_planes; i++)
		addr[i] = cur->addr[i];
out:
	mutex_unlock(&dec->dpb_mutex);

	return ret;
}

void s5p_mfc_dpb_pin_release_all(struct s5p_mfc_ctx *ctx)
{
	struct s5p_mfc_dec *dec = ctx->dec_priv;
	int i;

	mutex_lock(&dec->dpb_mutex);

	for (i = 0; i < MFC_MAX_DPBS; i++) {
		mfc_dpb_pin_release(&dec->dpb_pin[i]);
		mfc_dpb_pin_release(&dec->dpb_pin_prev[i]);
	}
	dec->dpb_sent = 0;

	mutex_unlock(&dec->dpb_mutex);
}
