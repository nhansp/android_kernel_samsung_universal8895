/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2011 Google, Inc.
 *
 * greatlte legacy ION ABI (spec L6A 5.2.2, 5.2.3): private copy of the stock
 * 4.4 ION user ABI, never exported to userspace. The structs, flags and
 * commands are verbatim from S:drivers/staging/android/uapi/ion.h (structs
 * :101-150, flags :63-81, commands :173-243), the 32-bit compat structs and
 * commands verbatim from S:drivers/staging/android/ion/compat_ion.c:25-56.
 * The prebuilt libion_exynos.so and AOSP libion.so blobs are built against
 * these layouts; ion_legacy.c checks every command value at build time.
 */

#ifndef _ION_LEGACY_UAPI_H
#define _ION_LEGACY_UAPI_H

#include <linux/ioctl.h>
#include <linux/types.h>
#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

typedef int ion_user_handle_t;					/* S:.../uapi/ion.h:23 */

/* allocation flags: S:drivers/staging/android/uapi/ion.h:63-81 */
#define ION_FLAG_CACHED 1		/* mappings of this buffer should be
					   cached, ion will do cache
					   maintenance when the buffer is
					   mapped for dma */
#define ION_FLAG_CACHED_NEEDS_SYNC 2	/* mappings of this buffer will created
					   at mmap time, if this is set
					   caches must be managed manually */
#define ION_FLAG_NOZEROED 8		/* Allocated buffer is not initialized
					   with zero value and userspace is not
					   able to access the buffer
					 */
#define ION_FLAG_PROTECTED 16		/* this buffer would be used in secure
					   world. if this is set, all cpu accesses
					   are prohibited.
					 */
#define ION_FLAG_SYNC_FORCE 32		/* cache sync forcely at allocation */
#define ION_FLAG_MAY_HWRENDER 64	/* buffer is cachable but it can be
					accessd by H/W on both sharable domain, non-sharable domain,
					so it sholud flush the cache before sharable domain accesses */

/* structs: S:drivers/staging/android/uapi/ion.h:101-150 */
struct ion_allocation_data {
	size_t len;
	size_t align;
	unsigned int heap_id_mask;
	unsigned int flags;
	ion_user_handle_t handle;
};

struct ion_fd_data {
	ion_user_handle_t handle;
	int fd;
};

struct ion_fd_partial_data {
	ion_user_handle_t handle;
	int fd;
	off_t offset;
	size_t len;
};

struct ion_handle_data {
	ion_user_handle_t handle;
};

struct ion_custom_data {
	unsigned int cmd;
	unsigned long arg;
};

/*
 * commands: S:drivers/staging/android/uapi/ion.h:173-243. ION_IOC_PRELOAD_ALLOC
 * (:235) is kept for the record only: stock's switch does not handle it
 * (S:drivers/staging/android/ion/ion.c:1939-2028), it answers -ENOTTY (L6A 5.2.3),
 * and struct ion_preload_data (:166-171) is deliberately not copied.
 */
#define ION_IOC_MAGIC		'I'

#define ION_IOC_ALLOC		_IOWR(ION_IOC_MAGIC, 0, \
				      struct ion_allocation_data)
#define ION_IOC_FREE		_IOWR(ION_IOC_MAGIC, 1, struct ion_handle_data)
#define ION_IOC_MAP		_IOWR(ION_IOC_MAGIC, 2, struct ion_fd_data)
#define ION_IOC_SHARE		_IOWR(ION_IOC_MAGIC, 4, struct ion_fd_data)
#define ION_IOC_IMPORT		_IOWR(ION_IOC_MAGIC, 5, struct ion_fd_data)
#define ION_IOC_SYNC		_IOWR(ION_IOC_MAGIC, 7, struct ion_fd_data)
#define ION_IOC_SYNC_PARTIAL	_IOWR(ION_IOC_MAGIC, 9, struct ion_fd_partial_data)
#define ION_IOC_PRELOAD_ALLOC	_IOW(ION_IOC_MAGIC, 8, struct ion_preload_data)
#define ION_IOC_CUSTOM		_IOWR(ION_IOC_MAGIC, 6, struct ion_custom_data)

#ifdef CONFIG_COMPAT
/* 32-bit ABI: S:drivers/staging/android/ion/compat_ion.c:25-56 */
struct compat_ion_allocation_data {
	compat_size_t len;
	compat_size_t align;
	compat_uint_t heap_id_mask;
	compat_uint_t flags;
	compat_int_t handle;
};

struct compat_ion_custom_data {
	compat_uint_t cmd;
	compat_ulong_t arg;
};

struct compat_ion_handle_data {
	compat_int_t handle;
};

struct compat_ion_fd_partial_data {
	compat_int_t handle;
	compat_int_t fd;
	compat_off_t offset;
	compat_size_t len;
};

#define COMPAT_ION_IOC_ALLOC	_IOWR(ION_IOC_MAGIC, 0, \
				      struct compat_ion_allocation_data)
#define COMPAT_ION_IOC_FREE	_IOWR(ION_IOC_MAGIC, 1, \
				      struct compat_ion_handle_data)
#define COMPAT_ION_IOC_CUSTOM	_IOWR(ION_IOC_MAGIC, 6, \
				      struct compat_ion_custom_data)
#define COMPAT_ION_IOC_SYNC_PARTIAL	_IOWR(ION_IOC_MAGIC, 9, \
					struct compat_ion_fd_partial_data)
#endif /* CONFIG_COMPAT */

#endif /* _ION_LEGACY_UAPI_H */
