/* SPDX-License-Identifier: GPL-2.0 */
/*
 * greatlte legacy ION ABI shim (spec L6A 5.2): objects shared by ion_legacy.c
 * (device, clients, handles, ioctls), ion_legacy_buffer.c (buffers, dma-buf
 * exporter, SYNC) and ion_legacy_heap.c (heap table, page providers), and the
 * counters of the L6A heartbeat row (L6A 5.2.8 item 5, 8.1).
 *
 * Locks (outermost first; no spinlock anywhere in the shim):
 *   ionl_dev.clients_lock -> ionl_client.lock -> ionl_dev.buffers_lock.
 *   ionl_buffer.lock is taken alone (never together with another shim lock).
 */

#ifndef _ION_LEGACY_H
#define _ION_LEGACY_H

#include <linux/atomic.h>
#include <linux/dma-buf.h>
#include <linux/ion_legacy.h>
#include <linux/kref.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/scatterlist.h>
#include <linux/types.h>

#include "ion_legacy_uapi.h"

struct cma;
struct device;
struct page;

/* heap id 0 is the system heap (S:drivers/staging/android/ion/exynos/exynos_ion.c:52-56) */
#define IONL_SYSTEM_HEAP_ID	0
/* ION_NUM_HEAPS (S:drivers/staging/android/uapi/ion.h:47); stock rejects DT ids >= it (exynos_ion.c:401-404) */
#define IONL_NUM_HEAPS		16
/* 1 system heap + at most one heap per remaining id */
#define IONL_MAX_HEAPS		IONL_NUM_HEAPS

enum ionl_heap_type {
	IONL_HEAP_SYSTEM,	/* page allocator, stock ion_system_heap.c */
	IONL_HEAP_CMA,		/* a reusable shared-dma-pool, stock ion_cma_heap.c */
};

/* Immutable after probe; the counters are atomics (no lock). */
struct ionl_heap {
	unsigned int id;		/* legacy heap id, bit in heap_id_mask */
	u32 compat_ids;			/* EXYNOS contig ids served (exynos_ion.c:406-411) */
	enum ionl_heap_type type;
	const char *name;		/* "<memory-region-name>_heap"; dma-buf exp_name (L6A 5.2.8 item 2) */
	struct cma *cma;		/* IONL_HEAP_CMA: rmem->priv of rmem_cma_setup */
	atomic_t nr_buffers;		/* live buffers (debugfs heaps) */
	atomic_long_t nr_bytes;		/* their bytes */
};

/*
 * One allocation (L6A 5.2.5 item 8). ref: one per handle, one per exported
 * dma-buf, one transient per ioctl; the last put frees the pages.
 * heap, flags, size, sgt, cma_page: set at creation, then constant.
 * lock: protects vaddr and vmap_cnt only.
 * node, pid, handle_count: protected by ionl_dev.buffers_lock.
 */
struct ionl_buffer {
	struct kref ref;
	struct ionl_heap *heap;
	unsigned long flags;
	size_t size;
	struct sg_table sgt;		/* DMA-mapped once for the ION device */
	struct page *cma_page;		/* IONL_HEAP_CMA: first page of the cma_alloc() range */
	struct mutex lock;		/* protects vaddr, vmap_cnt */
	void *vaddr;
	int vmap_cnt;
	struct list_head node;		/* in ionl_dev.buffers */
	pid_t pid;			/* stock buffer->pid (S:.../ion.c:287, :575-577) */
	int handle_count;		/* stock buffer->handle_count (S:.../ion.c:551-580) */
};

/*
 * The one ION device. dev, heaps, nr_heaps: set by probe, then constant.
 * clients_lock: protects the clients list.
 * buffers_lock: protects the buffers list and each buffer's node, pid and
 * handle_count.
 */
struct ionl_device {
	struct device *dev;		/* DMA proxy device: no IOMMU, not coherent (L6A 4.3, 5.2.5 item 8) */
	struct ionl_heap *heaps;	/* sorted by descending id (stock plist prio -id, S:.../ion.c:2235) */
	unsigned int nr_heaps;
	struct mutex clients_lock;	/* protects clients */
	struct list_head clients;
	struct mutex buffers_lock;	/* protects buffers, each buffer's node/pid/handle_count */
	struct list_head buffers;
	struct dentry *debug_root;
};

extern struct ionl_device ionl_dev;

/* heartbeat counters (L6A 5.2.8 item 5, 8.1); read lock-free */
struct ionl_stats {
	int probe;			/* last probe result: 0 or -errno; -ENODEV = never probed */
	atomic_t buffers;		/* live buffers ("ionb") */
	atomic_long_t bytes;		/* their bytes */
};

extern struct ionl_stats ionl_stats;

/* ion_legacy_heap.c */
int ionl_heap_allocate(struct ionl_heap *heap, struct ionl_buffer *buf,
		       unsigned long len, unsigned long align);
void ionl_heap_free(struct ionl_buffer *buf);
bool ionl_heap_flush_at_alloc(struct ionl_buffer *buf);
int ionl_heaps_init(struct device *dev);

/* ion_legacy_buffer.c */
extern const struct dma_buf_ops ionl_dma_buf_ops;
struct ionl_buffer *ionl_buffer_create(struct ionl_heap *heap, unsigned long len,
				       unsigned long align, unsigned long flags);
void ionl_buffer_get(struct ionl_buffer *buf);
void ionl_buffer_put(struct ionl_buffer *buf);
struct dma_buf *ionl_buffer_export(struct ionl_buffer *buf);
int ionl_sync_fd(int fd);
int ionl_sync_partial_fd(int fd, off_t offset, size_t len);

#endif /* _ION_LEGACY_H */
