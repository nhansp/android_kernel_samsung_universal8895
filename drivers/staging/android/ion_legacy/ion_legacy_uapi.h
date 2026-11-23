/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ION user ABI of the Samsung Exynos 4.4 kernels, as the prebuilt libion
 * users were built against it. Private to this driver, not exported.
 *
 * Copyright (C) 2011 Google, Inc.
 */

#ifndef _ION_LEGACY_UAPI_H
#define _ION_LEGACY_UAPI_H

#include <linux/ioctl.h>
#include <linux/types.h>
#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

#define ION_FLAG_CACHED			1
#define ION_FLAG_CACHED_NEEDS_SYNC	2
#define ION_FLAG_NOZEROED		8
#define ION_FLAG_PROTECTED		16
#define ION_FLAG_SYNC_FORCE		32
#define ION_FLAG_MAY_HWRENDER		64

struct ion_allocation_data {
	size_t len;
	size_t align;
	unsigned int heap_id_mask;
	unsigned int flags;
	int handle;
};

struct ion_fd_data {
	int handle;
	int fd;
};

struct ion_fd_partial_data {
	int handle;
	int fd;
	off_t offset;
	size_t len;
};

struct ion_handle_data {
	int handle;
};

struct ion_custom_data {
	unsigned int cmd;
	unsigned long arg;
};

#define ION_IOC_MAGIC		'I'

#define ION_IOC_ALLOC		_IOWR(ION_IOC_MAGIC, 0, \
				      struct ion_allocation_data)
#define ION_IOC_FREE		_IOWR(ION_IOC_MAGIC, 1, struct ion_handle_data)
#define ION_IOC_MAP		_IOWR(ION_IOC_MAGIC, 2, struct ion_fd_data)
#define ION_IOC_SHARE		_IOWR(ION_IOC_MAGIC, 4, struct ion_fd_data)
#define ION_IOC_IMPORT		_IOWR(ION_IOC_MAGIC, 5, struct ion_fd_data)
#define ION_IOC_CUSTOM		_IOWR(ION_IOC_MAGIC, 6, struct ion_custom_data)
#define ION_IOC_SYNC		_IOWR(ION_IOC_MAGIC, 7, struct ion_fd_data)
#define ION_IOC_SYNC_PARTIAL	_IOWR(ION_IOC_MAGIC, 9, struct ion_fd_partial_data)

#ifdef CONFIG_COMPAT
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
