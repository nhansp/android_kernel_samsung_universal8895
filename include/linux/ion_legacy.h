/* SPDX-License-Identifier: GPL-2.0 */
/*
 * greatlte legacy ION ABI shim: what in-kernel dma-buf importers ask of an ION
 * buffer (spec L6A 5.2.6 items 2 and 4, 5.3.7; s5p-smem).
 */

#ifndef _LINUX_ION_LEGACY_H
#define _LINUX_ION_LEGACY_H

#include <linux/errno.h>
#include <linux/types.h>

struct dma_buf;

#if IS_ENABLED(CONFIG_ION_LEGACY_EXYNOS)
/*
 * True for an ION-legacy dma-buf of the auto-sync class (CACHED without
 * CACHED_NEEDS_SYNC): user space never syncs it, so an importer that keeps
 * its mapping across uses must clean it before each device use with
 * dma_buf_end_cpu_access(dmabuf, DMA_TO_DEVICE).
 */
bool ionl_dmabuf_is_autosync(struct dma_buf *dmabuf);

/* physical start and size of a contiguous (secure or CMA) ION buffer */
int ionl_dmabuf_phys(struct dma_buf *dmabuf, phys_addr_t *addr, size_t *len);
#else
static inline bool ionl_dmabuf_is_autosync(struct dma_buf *dmabuf)
{
	return false;
}

static inline int ionl_dmabuf_phys(struct dma_buf *dmabuf, phys_addr_t *addr, size_t *len)
{
	return -ENODEV;
}
#endif

#endif /* _LINUX_ION_LEGACY_H */
