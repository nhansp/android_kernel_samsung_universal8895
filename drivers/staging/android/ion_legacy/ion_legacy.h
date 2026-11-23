/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Lock order: ionl_dev.clients_lock -> ionl_client.lock -> ionl_dev.buffers_lock.
 * ionl_buffer.lock is never held together with another of these locks.
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

#define IONL_SYSTEM_HEAP_ID	0
/* ION_NUM_HEAPS of the vendor ABI: heap ids are 0..15 */
#define IONL_NUM_HEAPS		16
#define IONL_MAX_HEAPS		IONL_NUM_HEAPS

enum ionl_heap_type {
	IONL_HEAP_SYSTEM,
	IONL_HEAP_CMA,
};

/* Constant after probe, except the counters. */
struct ionl_heap {
	unsigned int id;
	u32 compat_ids;			/* Exynos contig ids served, as a bit mask */
	enum ionl_heap_type type;
	const char *name;
	struct cma *cma;		/* IONL_HEAP_CMA */
	atomic_t nr_buffers;
	atomic_long_t nr_bytes;
};

/*
 * heap, flags, size, sgt and cma_page are constant after creation.
 * node, pid and handle_count are protected by ionl_dev.buffers_lock.
 */
struct ionl_buffer {
	struct kref ref;
	struct ionl_heap *heap;
	unsigned long flags;
	size_t size;
	struct sg_table sgt;		/* mapped for ionl_dev.dev */
	struct page *cma_page;		/* IONL_HEAP_CMA */
	struct mutex lock;		/* protects vaddr, vmap_cnt */
	void *vaddr;
	int vmap_cnt;
	struct list_head node;
	pid_t pid;
	int handle_count;
};

struct ionl_device {
	struct device *dev;
	struct ionl_heap *heaps;	/* by descending id */
	unsigned int nr_heaps;
	struct mutex clients_lock;	/* protects clients */
	struct list_head clients;
	struct mutex buffers_lock;	/* protects buffers, each buffer's node/pid/handle_count */
	struct list_head buffers;
	struct dentry *debug_root;
};

extern struct ionl_device ionl_dev;

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
