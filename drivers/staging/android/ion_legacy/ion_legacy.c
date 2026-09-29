// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2011 Google, Inc.
 *
 * greatlte legacy ION ABI shim (spec L6A 5.2, D-2): /dev/ion with the stock
 * 4.4 Exynos ION user ABI (client handles, ION_IOC_ALLOC/FREE/SHARE/MAP/
 * IMPORT/SYNC/SYNC_PARTIAL, 32-bit compat) for the prebuilt libion_exynos.so,
 * gralloc1 and the camera HAL, as its own dma-buf exporter over the page
 * allocator (heap 0) and the reusable CMA areas of the DT (heaps 1, 3, 10).
 * Clients, handles and the ioctl dispatcher reproduce
 * S:drivers/staging/android/ion/ion.c:582-909, :1179-1298, :1641-1737,
 * :1900-2078; the heap-id parsing S:drivers/staging/android/ion/exynos/exynos_ion.c:318-347;
 * the compat ioctl S:drivers/staging/android/ion/compat_ion.c:58-244.
 * Deliberate deviations (L6A 5.2.5 item 3, 4.3): no protected or secure heap,
 * ION_FLAG_PROTECTED fails -ENODEV.
 */

#define pr_fmt(fmt) "ion-legacy: " fmt

#include <linux/bitops.h>
#include <linux/build_bug.h>
#include <linux/debugfs.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/idr.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/rbtree.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

#include "ion_legacy.h"

/* L6A 5.2.3: the ABI of the blobs, byte-exact (arm64 values; S:.../uapi/ion.h:181-243) */
static_assert(sizeof(struct ion_allocation_data) == 32);
static_assert(sizeof(struct ion_fd_partial_data) == 24);
static_assert(ION_IOC_ALLOC == 0xc0204900);
static_assert(ION_IOC_FREE == 0xc0044901);
static_assert(ION_IOC_MAP == 0xc0084902);
static_assert(ION_IOC_SHARE == 0xc0084904);
static_assert(ION_IOC_IMPORT == 0xc0084905);
static_assert(ION_IOC_CUSTOM == 0xc0104906);
static_assert(ION_IOC_SYNC == 0xc0084907);
static_assert(ION_IOC_SYNC_PARTIAL == 0xc0184909);
#ifdef CONFIG_COMPAT
/* S:drivers/staging/android/ion/compat_ion.c:25-56 */
static_assert(sizeof(struct compat_ion_allocation_data) == 20);
static_assert(sizeof(struct compat_ion_fd_partial_data) == 16);
static_assert(COMPAT_ION_IOC_ALLOC == 0xc0144900);
static_assert(COMPAT_ION_IOC_FREE == 0xc0044901);
static_assert(COMPAT_ION_IOC_CUSTOM == 0xc0084906);
static_assert(COMPAT_ION_IOC_SYNC_PARTIAL == 0xc0104909);
#endif

/* EXYNOS_ION_HEAP_EXYNOS_CONTIG_MASK = 1 << 4 (S:include/linux/exynos_ion.h:26,30-31) */
#define IONL_EXYNOS_CONTIG_MASK		(1U << 4)
/*
 * EXYNOS_ION_CONTIG_ID_MASK = ~((1 << (32 - EXYNOS_ION_CONTIG_ID_NUM)) - 1) with
 * EXYNOS_ION_CONTIG_ID_NUM = 14 (S:include/linux/exynos_ion.h:35-51,84-85) = 0xfffc0000
 */
#define IONL_EXYNOS_CONTIG_ID_MASK	(~((1U << (32 - 14)) - 1))

struct ionl_device ionl_dev = {
	.clients_lock = __MUTEX_INITIALIZER(ionl_dev.clients_lock),
	.clients = LIST_HEAD_INIT(ionl_dev.clients),
	.buffers_lock = __MUTEX_INITIALIZER(ionl_dev.buffers_lock),
	.buffers = LIST_HEAD_INIT(ionl_dev.buffers),
};

struct ionl_stats ionl_stats = {
	.probe = -ENODEV,
};

/*
 * One per open() of /dev/ion (stock ion_client, S:drivers/staging/android/ion/ion.c:99-111).
 * node: in ionl_dev.clients, under ionl_dev.clients_lock.
 * lock: protects idr, handles, and each handle's user_ref_count and last kref
 * put (a handle is destroyed only under it). pid: constant.
 */
struct ionl_client {
	struct list_head node;
	struct mutex lock;	/* protects idr, handles, handle user_ref_count and destroy */
	struct idr idr;
	struct rb_root handles;
	pid_t pid;
};

/*
 * A client-local reference to a buffer (stock ion_handle, S:.../ion.c:125-133).
 * ref: the user reference (held while user_ref_count > 0) plus transient ones.
 * id, node: set once when added under client->lock.
 */
struct ionl_handle {
	struct kref ref;
	unsigned int user_ref_count;
	struct ionl_client *client;
	struct ionl_buffer *buf;
	struct rb_node node;
	int id;
};

/* stock ion_buffer_add_to_handle() / ion_buffer_remove_from_handle() (S:.../ion.c:551-580) */
static void ionl_buffer_add_to_handle(struct ionl_buffer *buf)
{
	mutex_lock(&ionl_dev.buffers_lock);
	buf->handle_count++;
	mutex_unlock(&ionl_dev.buffers_lock);
}

static void ionl_buffer_remove_from_handle(struct ionl_buffer *buf)
{
	mutex_lock(&ionl_dev.buffers_lock);
	if (!WARN_ON(buf->handle_count <= 0) && !--buf->handle_count)
		buf->pid = task_pid_nr(current->group_leader);
	mutex_unlock(&ionl_dev.buffers_lock);
}

/* stock ion_handle_create() (S:.../ion.c:589-605): takes a buffer reference */
static struct ionl_handle *ionl_handle_create(struct ionl_client *client,
					      struct ionl_buffer *buf)
{
	struct ionl_handle *handle;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return ERR_PTR(-ENOMEM);
	kref_init(&handle->ref);
	RB_CLEAR_NODE(&handle->node);
	handle->client = client;
	ionl_buffer_get(buf);
	ionl_buffer_add_to_handle(buf);
	handle->buf = buf;

	return handle;
}

/* stock ion_handle_destroy() (S:.../ion.c:609-628); called with client->lock held */
static void ionl_handle_destroy(struct kref *kref)
{
	struct ionl_handle *handle = container_of(kref, struct ionl_handle, ref);
	struct ionl_client *client = handle->client;
	struct ionl_buffer *buf = handle->buf;

	lockdep_assert_held(&client->lock);

	if (handle->id)
		idr_remove(&client->idr, handle->id);
	if (!RB_EMPTY_NODE(&handle->node))
		rb_erase(&handle->node, &client->handles);

	ionl_buffer_remove_from_handle(buf);
	ionl_buffer_put(buf);

	kfree(handle);
}

/* stock ion_handle_put_nolock() (S:.../ion.c:649-652) */
static void ionl_handle_put_nolock(struct ionl_handle *handle)
{
	kref_put(&handle->ref, ionl_handle_destroy);
}

static void ionl_handle_put(struct ionl_handle *handle)
{
	struct ionl_client *client = handle->client;

	mutex_lock(&client->lock);
	ionl_handle_put_nolock(handle);
	mutex_unlock(&client->lock);
}

/*
 * stock user_ion_free_nolock() + user_ion_handle_put_nolock() (S:.../ion.c:704-713,
 * :934-950): drop one user reference, the last one drops the user kref.
 * A handle without user references is left alone (stock warned and returned).
 */
static void ionl_user_handle_put_nolock(struct ionl_handle *handle)
{
	if (!handle->user_ref_count)
		return;
	if (--handle->user_ref_count == 0)
		ionl_handle_put_nolock(handle);
}

/* stock ion_handle_lookup() (S:.../ion.c:715-731): the client's handle of a buffer */
static struct ionl_handle *ionl_handle_lookup(struct ionl_client *client,
					      struct ionl_buffer *buf)
{
	struct rb_node *n = client->handles.rb_node;

	while (n) {
		struct ionl_handle *entry = rb_entry(n, struct ionl_handle, node);

		if (buf < entry->buf)
			n = n->rb_left;
		else if (buf > entry->buf)
			n = n->rb_right;
		else
			return entry;
	}
	return NULL;
}

/* stock ion_handle_add() (S:.../ion.c:757-788): ids start at 1; called with client->lock held */
static int ionl_handle_add(struct ionl_client *client, struct ionl_handle *handle)
{
	struct rb_node **p = &client->handles.rb_node;
	struct rb_node *parent = NULL;
	struct ionl_handle *entry;
	int id;

	id = idr_alloc(&client->idr, handle, 1, 0, GFP_KERNEL);
	if (id < 0)
		return id;

	handle->id = id;

	while (*p) {
		parent = *p;
		entry = rb_entry(parent, struct ionl_handle, node);

		if (handle->buf < entry->buf)
			p = &(*p)->rb_left;
		else
			p = &(*p)->rb_right;
	}

	rb_link_node(&handle->node, parent, p);
	rb_insert_color(&handle->node, &client->handles);

	return 0;
}

/*
 * stock ion_buffer_get_total_size_by_pid() (S:.../ion.c:792-811): bytes of the
 * live buffers attributed to this client's process.
 */
static size_t ionl_total_size_by_pid(struct ionl_client *client)
{
	struct ionl_buffer *buf;
	size_t total = 0;

	mutex_lock(&ionl_dev.buffers_lock);
	list_for_each_entry(buf, &ionl_dev.buffers, node)
		if (buf->pid == client->pid)
			total += buf->size;
	mutex_unlock(&ionl_dev.buffers_lock);

	return total;
}

/*
 * stock ion_parse_heap_id() (S:drivers/staging/android/ion/exynos/exynos_ion.c:318-347),
 * L6A 5.2.5 item 2: a mask other than the contig mask is used as is; the contig
 * mask takes the id from the top contig-id bit of flags (default 1) and matches
 * a served heap by id or by compat id; no match -> 0.
 */
static unsigned int ionl_parse_heap_id(unsigned int heap_id_mask, unsigned int flags)
{
	unsigned int heap_id = 1;
	unsigned int i;

	if (heap_id_mask != IONL_EXYNOS_CONTIG_MASK)
		return heap_id_mask;

	if (flags & IONL_EXYNOS_CONTIG_ID_MASK)
		heap_id = __fls(flags & IONL_EXYNOS_CONTIG_ID_MASK);

	for (i = 0; i < ionl_dev.nr_heaps; i++) {
		struct ionl_heap *heap = &ionl_dev.heaps[i];

		if (heap->id == IONL_SYSTEM_HEAP_ID)	/* stock loop starts at plat_heaps[1] */
			continue;
		if (heap->id == heap_id || (heap->compat_ids & BIT(heap_id)))
			return BIT(heap->id);
	}

	pr_err("%s: bad heap flags %#x\n", __func__, flags);	/* exynos_ion.c:338-341 */
	return 0;
}

/*
 * stock __ion_alloc(..., grab_handle = true) + pass_to_user() (S:.../ion.c:691-701,
 * :813-909), L6A 5.2.5. Returns the handle with one user reference and one extra
 * reference that the caller drops (stock cleanup_handle, :1959, :2041-2042).
 */
static struct ionl_handle *ionl_alloc(struct ionl_client *client, size_t len,
				      size_t align, unsigned int heap_id_mask,
				      unsigned int flags)
{
	struct ionl_buffer *buf = NULL;
	struct ionl_handle *handle;
	unsigned int i;
	int ret;

	len = PAGE_ALIGN(len);					/* :834-839 */
	if (!len)
		return ERR_PTR(-EINVAL);

	if (len / PAGE_SIZE > totalram_pages() / 4) {		/* :841-849 */
		size_t pid_total_size = ionl_total_size_by_pid(client);

		if ((len + pid_total_size) / PAGE_SIZE > totalram_pages() / 2)
			return ERR_PTR(-EINVAL);
	}

	heap_id_mask = ionl_parse_heap_id(heap_id_mask, flags);	/* :852-854 */
	if (heap_id_mask == 0)
		return ERR_PTR(-EINVAL);

	/* L6A 5.2.5 item 3: no protected content path in this build (4.3, O-6A-4) */
	if (flags & ION_FLAG_PROTECTED)
		return ERR_PTR(-ENODEV);

	for (i = 0; i < ionl_dev.nr_heaps; i++) {		/* :856-870, descending id */
		struct ionl_heap *heap = &ionl_dev.heaps[i];

		if (!(BIT(heap->id) & heap_id_mask))
			continue;
		buf = ionl_buffer_create(heap, len, align, flags);
		if (!IS_ERR(buf))
			break;
	}

	if (!buf)
		return ERR_PTR(-ENODEV);
	if (IS_ERR(buf))
		return ERR_CAST(buf);

	handle = ionl_handle_create(client, buf);
	/* the buffer's creation reference is dropped; the handle holds its own (:878-884) */
	ionl_buffer_put(buf);
	if (IS_ERR(handle))
		return handle;

	mutex_lock(&client->lock);
	kref_get(&handle->ref);					/* grab_handle, :893-894 */
	ret = ionl_handle_add(client, handle);
	if (ret) {
		ionl_handle_put_nolock(handle);
		ionl_handle_put_nolock(handle);
		mutex_unlock(&client->lock);
		return ERR_PTR(ret);
	}
	/* pass_to_user(): the creation reference becomes the user reference (:672-677, :691-701) */
	handle->user_ref_count = 1;
	mutex_unlock(&client->lock);

	return handle;
}

/* ION_IOC_FREE (S:.../ion.c:1962-1976, handle lookup :733-743): unknown id -> -EINVAL */
static int ionl_free_id(struct ionl_client *client, int id)
{
	struct ionl_handle *handle;

	mutex_lock(&client->lock);
	handle = idr_find(&client->idr, id);
	if (!handle) {
		mutex_unlock(&client->lock);
		return -EINVAL;
	}
	kref_get(&handle->ref);
	ionl_user_handle_put_nolock(handle);
	ionl_handle_put_nolock(handle);
	mutex_unlock(&client->lock);

	return 0;
}

/*
 * ION_IOC_SHARE / ION_IOC_MAP, first half (S:.../ion.c:1982-1984, ion_share_dma_buf()
 * :1649-1658): the buffer of handle id, with a reference for the dma-buf; unknown id -> -EINVAL.
 */
static struct ionl_buffer *ionl_share_get(struct ionl_client *client, int id)
{
	struct ionl_handle *handle;
	struct ionl_buffer *buf;

	mutex_lock(&client->lock);
	handle = idr_find(&client->idr, id);
	if (!handle) {
		mutex_unlock(&client->lock);
		return ERR_PTR(-EINVAL);
	}
	buf = handle->buf;
	ionl_buffer_get(buf);
	mutex_unlock(&client->lock);

	return buf;
}

/*
 * Second half (ion_share_dma_buf() :1660-1671, ion_share_dma_buf_fd() :1675-1689):
 * a new dma-buf that takes over the buffer reference, and an O_CLOEXEC fd of it.
 */
static int ionl_share_fd(struct ionl_buffer *buf)
{
	struct dma_buf *dmabuf;
	int fd;

	dmabuf = ionl_buffer_export(buf);
	if (IS_ERR(dmabuf))
		return PTR_ERR(dmabuf);

	fd = dma_buf_fd(dmabuf, O_CLOEXEC);
	if (fd < 0)
		dma_buf_put(dmabuf);

	return fd;
}

/*
 * ION_IOC_IMPORT (S:.../ion.c:1692-1737, then pass_to_user() :1999): only this
 * exporter's dma-bufs; an existing handle of the client for the buffer gets one
 * more user reference, else a new handle. Returns the handle id.
 */
static int ionl_import_fd(struct ionl_client *client, int fd)
{
	struct ionl_handle *handle;
	struct ionl_buffer *buf;
	struct dma_buf *dmabuf;
	int ret;

	dmabuf = dma_buf_get(fd);
	if (IS_ERR(dmabuf))
		return PTR_ERR(dmabuf);

	if (dmabuf->ops != &ionl_dma_buf_ops) {
		pr_err("can not import dmabuf from another exporter\n");
		dma_buf_put(dmabuf);
		return -EINVAL;
	}
	buf = dmabuf->priv;

	mutex_lock(&client->lock);
	handle = ionl_handle_lookup(client, buf);
	if (handle) {
		/* user_ion_handle_get_check_overflow() (:680-686) */
		if (handle->user_ref_count + 1 == 0) {
			ret = -EOVERFLOW;
		} else {
			if (handle->user_ref_count++ == 0)
				kref_get(&handle->ref);
			ret = handle->id;
		}
		goto out_unlock;
	}

	handle = ionl_handle_create(client, buf);
	if (IS_ERR(handle)) {
		ret = PTR_ERR(handle);
		goto out_unlock;
	}

	ret = ionl_handle_add(client, handle);
	if (ret) {
		ionl_handle_put_nolock(handle);
		goto out_unlock;
	}
	handle->user_ref_count = 1;	/* pass_to_user() of a new handle */
	ret = handle->id;

out_unlock:
	mutex_unlock(&client->lock);
	dma_buf_put(dmabuf);
	return ret;
}

/* fix up the cases where the ioctl direction bits are incorrect (S:.../ion.c:1900-1912) */
static unsigned int ionl_ioctl_dir(unsigned int cmd)
{
	switch (cmd) {
	case ION_IOC_SYNC:
	case ION_IOC_SYNC_PARTIAL:
	case ION_IOC_FREE:
	case ION_IOC_CUSTOM:
		return _IOC_WRITE;
	default:
		return _IOC_DIR(cmd);
	}
}

static void ionl_alloc_failed(const struct ion_allocation_data *a, long ret)
{
	/* S:.../ion.c:1949-1953 */
	pr_err("%s: len %zu align %zu heap_id_mask %u flags %x (ret %ld)\n",
	       "ALLOC", a->len, a->align, a->heap_id_mask, a->flags, ret);
}

/* stock ion_ioctl() (S:drivers/staging/android/ion/ion.c:1914-2044), L6A 5.2.4 */
static long ionl_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct ionl_client *client = filp->private_data;
	struct ionl_handle *cleanup_handle = NULL;
	int ret = 0;
	unsigned int dir;

	union {
		struct ion_fd_data fd;
		struct ion_fd_partial_data fd_partial;
		struct ion_allocation_data allocation;
		struct ion_handle_data handle;
		struct ion_custom_data custom;
	} data = {};

	dir = ionl_ioctl_dir(cmd);

	if (_IOC_SIZE(cmd) > sizeof(data))
		return -EINVAL;

	if (dir & _IOC_WRITE)
		if (copy_from_user(&data, (void __user *)arg, _IOC_SIZE(cmd)))
			return -EFAULT;

	switch (cmd) {
	case ION_IOC_ALLOC:
	{
		struct ionl_handle *handle;

		handle = ionl_alloc(client, data.allocation.len,
				    data.allocation.align,
				    data.allocation.heap_id_mask,
				    data.allocation.flags);
		if (IS_ERR(handle)) {
			ionl_alloc_failed(&data.allocation, PTR_ERR(handle));
			return PTR_ERR(handle);
		}
		data.allocation.handle = handle->id;

		cleanup_handle = handle;
		break;
	}
	case ION_IOC_FREE:
		ret = ionl_free_id(client, data.handle.handle);
		if (ret)
			return ret;
		break;
	case ION_IOC_SHARE:
	case ION_IOC_MAP:
	{
		struct ionl_buffer *buf;

		buf = ionl_share_get(client, data.handle.handle);
		if (IS_ERR(buf))			/* :1982-1984: no copy back */
			return PTR_ERR(buf);
		data.fd.fd = ionl_share_fd(buf);
		if (data.fd.fd < 0)			/* :1987-1988: copied back */
			ret = data.fd.fd;
		break;
	}
	case ION_IOC_IMPORT:
		ret = ionl_import_fd(client, data.fd.fd);
		if (ret >= 0) {
			data.handle.handle = ret;
			ret = 0;
		}
		break;
	case ION_IOC_SYNC:
		ret = ionl_sync_fd(data.fd.fd);
		break;
	case ION_IOC_SYNC_PARTIAL:
		ret = ionl_sync_partial_fd(data.fd_partial.fd,
					   data.fd_partial.offset, data.fd_partial.len);
		break;
	case ION_IOC_CUSTOM:
		/* no custom_ioctl on Exynos (exynos_ion.c:644 ion_device_create(NULL); ion.c:2020-2021) */
		return -ENOTTY;
	default:
		return -ENOTTY;
	}

	if (dir & _IOC_READ) {
		if (copy_to_user((void __user *)arg, &data, _IOC_SIZE(cmd))) {
			if (cleanup_handle) {			/* :2032-2037 */
				mutex_lock(&client->lock);
				ionl_user_handle_put_nolock(cleanup_handle);
				ionl_handle_put_nolock(cleanup_handle);
				mutex_unlock(&client->lock);
			}
			return -EFAULT;
		}
	}
	if (cleanup_handle)
		ionl_handle_put(cleanup_handle);
	return ret;
}

#ifdef CONFIG_COMPAT
/* stock compat_ion_ioctl() (S:drivers/staging/android/ion/compat_ion.c:155-244), L6A 5.2.4 item 7 */
static long ionl_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct ionl_client *client = filp->private_data;

	switch (cmd) {
	case COMPAT_ION_IOC_ALLOC:
	{
		struct compat_ion_allocation_data __user *data32 = compat_ptr(arg);
		struct ion_allocation_data data;
		struct ionl_handle *handle;
		compat_size_t s;
		compat_uint_t u;
		compat_int_t i;
		int err;

		/* compat_get_ion_allocation_data(), :58-79 */
		err = get_user(s, &data32->len);
		data.len = s;
		err |= get_user(s, &data32->align);
		data.align = s;
		err |= get_user(u, &data32->heap_id_mask);
		data.heap_id_mask = u;
		err |= get_user(u, &data32->flags);
		data.flags = u;
		err |= get_user(i, &data32->handle);
		data.handle = i;
		if (err)
			return err;

		handle = ionl_alloc(client, data.len, data.align,
				    data.heap_id_mask, data.flags);
		if (IS_ERR(handle)) {
			ionl_alloc_failed(&data, PTR_ERR(handle));
			return PTR_ERR(handle);
		}
		data.handle = handle->id;
		ionl_handle_put(handle);

		/* compat_put_ion_allocation_data(), :94-115; a failed put keeps the handle (:179-180) */
		err = put_user((compat_size_t)data.len, &data32->len);
		err |= put_user((compat_size_t)data.align, &data32->align);
		err |= put_user(data.heap_id_mask, &data32->heap_id_mask);
		err |= put_user(data.flags, &data32->flags);
		err |= put_user(data.handle, &data32->handle);
		return err;
	}
	case COMPAT_ION_IOC_FREE:
	{
		struct compat_ion_handle_data __user *data32 = compat_ptr(arg);
		compat_int_t i;

		if (get_user(i, &data32->handle))		/* :81-92 */
			return -EFAULT;

		return ionl_free_id(client, i);
	}
	case COMPAT_ION_IOC_CUSTOM:
	{
		struct compat_ion_custom_data __user *data32 = compat_ptr(arg);
		compat_uint_t ccmd;
		compat_ulong_t carg;
		int err;

		err = get_user(ccmd, &data32->cmd);		/* :117-131 */
		err |= get_user(carg, &data32->arg);
		if (err)
			return err;

		return -ENOTTY;					/* no custom_ioctl, ion.c:2020-2021 */
	}
	case COMPAT_ION_IOC_SYNC_PARTIAL:
	{
		struct compat_ion_fd_partial_data __user *data32 = compat_ptr(arg);
		compat_int_t fd, handle;
		compat_off_t offset;
		compat_size_t len;
		int err;

		err = get_user(handle, &data32->handle);	/* :133-153; offset sign-extends */
		err |= get_user(fd, &data32->fd);
		err |= get_user(offset, &data32->offset);
		err |= get_user(len, &data32->len);
		if (err)
			return err;

		return ionl_sync_partial_fd(fd, (off_t)offset, (size_t)len);
	}
	case ION_IOC_SHARE:
	case ION_IOC_MAP:
	case ION_IOC_IMPORT:
	case ION_IOC_SYNC:
		return ionl_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
	default:
		return -ENOIOCTLCMD;
	}
}
#endif

/* stock ion_open() + ion_client_create() (S:.../ion.c:2055-2070, :1179-1267) */
static int ionl_open(struct inode *inode, struct file *file)
{
	struct ionl_client *client;

	client = kzalloc(sizeof(*client), GFP_KERNEL);
	if (!client)
		return -ENOMEM;

	mutex_init(&client->lock);
	idr_init(&client->idr);
	client->handles = RB_ROOT;
	client->pid = task_pid_nr(current->group_leader);

	mutex_lock(&ionl_dev.clients_lock);
	list_add_tail(&client->node, &ionl_dev.clients);
	mutex_unlock(&ionl_dev.clients_lock);

	file->private_data = client;
	return 0;
}

/*
 * stock ion_release() + ion_client_destroy() (S:.../ion.c:2046-2053, :1270-1297):
 * every handle of the client goes, whatever its counts; a buffer that an exported
 * dma-buf still holds lives on until that dma-buf is released.
 */
static int ionl_release(struct inode *inode, struct file *file)
{
	struct ionl_client *client = file->private_data;
	struct rb_node *n;

	mutex_lock(&ionl_dev.clients_lock);
	list_del(&client->node);
	mutex_unlock(&ionl_dev.clients_lock);

	mutex_lock(&client->lock);
	while ((n = rb_first(&client->handles))) {
		struct ionl_handle *handle = rb_entry(n, struct ionl_handle, node);

		ionl_handle_destroy(&handle->ref);
	}
	mutex_unlock(&client->lock);

	idr_destroy(&client->idr);
	mutex_destroy(&client->lock);
	kfree(client);
	return 0;
}

/* stock ion_fops (S:.../ion.c:2072-2078) */
static const struct file_operations ionl_fops = {
	.owner = THIS_MODULE,
	.open = ionl_open,
	.release = ionl_release,
	.unlocked_ioctl = ionl_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = ionl_compat_ioctl,
#endif
};

/* L6A 5.2.8 item 3: /dev/ion (mode and owner from ueventd, DEV:configs/init/ueventd.samsungexynos8895.rc:9) */
static struct miscdevice ionl_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ion",
	.fops = &ionl_fops,
};

/* L6A 5.2.8 item 5: debugfs ion_legacy/heaps (per heap: live buffers, bytes) */
static int ionl_heaps_show(struct seq_file *s, void *unused)
{
	unsigned int i;

	seq_printf(s, "%4s %-16s %8s %12s\n", "id", "heap", "buffers", "bytes");
	for (i = 0; i < ionl_dev.nr_heaps; i++) {
		struct ionl_heap *heap = &ionl_dev.heaps[i];

		seq_printf(s, "%4u %-16s %8d %12ld\n", heap->id, heap->name,
			   atomic_read(&heap->nr_buffers), atomic_long_read(&heap->nr_bytes));
	}
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(ionl_heaps);

/* L6A 5.2.8 item 5: debugfs ion_legacy/clients (pid, handles) */
static int ionl_clients_show(struct seq_file *s, void *unused)
{
	struct ionl_client *client;

	seq_printf(s, "%8s %8s\n", "pid", "handles");
	mutex_lock(&ionl_dev.clients_lock);
	list_for_each_entry(client, &ionl_dev.clients, node) {
		unsigned int nr = 0;
		struct rb_node *n;

		mutex_lock(&client->lock);
		for (n = rb_first(&client->handles); n; n = rb_next(n))
			nr++;
		mutex_unlock(&client->lock);
		seq_printf(s, "%8d %8u\n", client->pid, nr);
	}
	mutex_unlock(&ionl_dev.clients_lock);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(ionl_clients);

/* L6A 5.2.8 */
static int ionl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	char heaps[128], compat[64];
	size_t hl = 0, cl = 0;
	unsigned int i;
	int ret;

	if (ionl_dev.dev)				/* one /dev/ion */
		return -EBUSY;

	ret = dma_coerce_mask_and_coherent(dev, DMA_BIT_MASK(64));	/* item 1; as A:.../system_heap.c:540 */
	if (ret)
		goto out;

	ret = ionl_heaps_init(dev);			/* item 2 */
	if (ret) {
		dev_err(dev, "heap table: %d\n", ret);
		goto out;
	}
	ionl_dev.dev = dev;

	ret = misc_register(&ionl_miscdev);		/* item 3 */
	if (ret) {
		/* the devm heap table goes with the failed probe */
		ionl_dev.dev = NULL;
		ionl_dev.heaps = NULL;
		ionl_dev.nr_heaps = 0;
		goto out;
	}

	ionl_dev.debug_root = debugfs_create_dir("ion_legacy", NULL);	/* item 5 */
	debugfs_create_file("heaps", 0444, ionl_dev.debug_root, NULL, &ionl_heaps_fops);
	debugfs_create_file("clients", 0444, ionl_dev.debug_root, NULL, &ionl_clients_fops);

	/* item 4, heaps in ascending id (the table is descending) */
	heaps[0] = '\0';
	compat[0] = '\0';
	for (i = ionl_dev.nr_heaps; i-- > 0; ) {
		struct ionl_heap *heap = &ionl_dev.heaps[i];
		int nlen = strlen(heap->name) - strlen("_heap");

		if (heap->id == IONL_SYSTEM_HEAP_ID)
			nlen = strlen("system");
		hl += scnprintf(heaps + hl, sizeof(heaps) - hl, "%s%u:%.*s",
				hl ? " " : "", heap->id, nlen,
				heap->id == IONL_SYSTEM_HEAP_ID ? "system" : heap->name);
		if (heap->compat_ids)
			cl += scnprintf(compat + cl, sizeof(compat) - cl, ", %.*s compat 0x%08x",
					nlen, heap->name, heap->compat_ids);
	}
	pr_info("/dev/ion ready: heaps %s%s\n", heaps, compat);

out:
	WRITE_ONCE(ionl_stats.probe, ret);
	return ret;
}

static const struct of_device_id ionl_of_match[] = {
	{ .compatible = "samsung,exynos8895-ion-legacy" },	/* L6A 4.3 */
	{ }
};

static struct platform_driver ionl_driver = {
	.probe = ionl_probe,
	.driver = {
		.name = "ion-legacy",
		.of_match_table = ionl_of_match,
		/* /dev/ion and its clients live as long as the kernel, as stock's subsys_initcall ION (exynos_ion.c:657) */
		.suppress_bind_attrs = true,
	},
};

static int __init ionl_init(void)
{
	return platform_driver_register(&ionl_driver);
}
device_initcall(ionl_init);

MODULE_IMPORT_NS("DMA_BUF");
MODULE_DESCRIPTION("greatlte legacy ION ABI over dma-buf and CMA");
MODULE_LICENSE("GPL");
