/* SPDX-License-Identifier: GPL-2.0 */
/*
 * In-kernel queries on dma-bufs exported by the legacy Exynos ION driver
 */

#ifndef _LINUX_ION_LEGACY_H
#define _LINUX_ION_LEGACY_H

#include <linux/errno.h>
#include <linux/types.h>

struct dma_buf;

#if IS_ENABLED(CONFIG_ION_LEGACY_EXYNOS)
/*
 * User space never syncs an auto-sync buffer (ION_FLAG_CACHED without
 * ION_FLAG_CACHED_NEEDS_SYNC). An importer that keeps it mapped across uses
 * cleans it with dma_buf_end_cpu_access(dmabuf, DMA_TO_DEVICE) before each one.
 */
bool ionl_dmabuf_is_autosync(struct dma_buf *dmabuf);

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
