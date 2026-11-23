// SPDX-License-Identifier: GPL-2.0
/*
 * Legacy Exynos ION: /dev/ion with the ION user ABI of the Samsung Exynos
 * 4.4 kernels, for prebuilt user space
 *
 * Copyright (C) 2011 Google, Inc.
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
static_assert(sizeof(struct compat_ion_allocation_data) == 20);
static_assert(sizeof(struct compat_ion_fd_partial_data) == 16);
static_assert(COMPAT_ION_IOC_ALLOC == 0xc0144900);
static_assert(COMPAT_ION_IOC_FREE == 0xc0044901);
static_assert(COMPAT_ION_IOC_CUSTOM == 0xc0084906);
static_assert(COMPAT_ION_IOC_SYNC_PARTIAL == 0xc0104909);
#endif

/* heap_id_mask that selects a heap by the contig id in the flags */
#define IONL_EXYNOS_CONTIG_MASK		BIT(4)
/* the 14 Exynos contig ids are flag bits 18..31 */
#define IONL_EXYNOS_CONTIG_ID_MASK	(~((1U << (32 - 14)) - 1))

struct ionl_device ionl_dev = {
	.clients_lock = __MUTEX_INITIALIZER(ionl_dev.clients_lock),
	.clients = LIST_HEAD_INIT(ionl_dev.clients),
	.buffers_lock = __MUTEX_INITIALIZER(ionl_dev.buffers_lock),
	.buffers = LIST_HEAD_INIT(ionl_dev.buffers),
};

/* One per open() of /dev/ion. pid is constant. */
struct ionl_client {
	struct list_head node;
	struct mutex lock;	/* protects idr, handles, handle user_ref_count and destroy */
	struct idr idr;
	struct rb_root handles;
	pid_t pid;
};

/*
 * A client's reference to a buffer. ref counts the user reference (held while
 * user_ref_count > 0) and transient ones.
 */
struct ionl_handle {
	struct kref ref;
	unsigned int user_ref_count;
	struct ionl_client *client;
	struct ionl_buffer *buf;
	struct rb_node node;
	int id;
};

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

static void ionl_user_handle_put_nolock(struct ionl_handle *handle)
{
	if (!handle->user_ref_count)
		return;
	if (--handle->user_ref_count == 0)
		ionl_handle_put_nolock(handle);
}

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

		if (heap->id == IONL_SYSTEM_HEAP_ID)
			continue;
		if (heap->id == heap_id || (heap->compat_ids & BIT(heap_id)))
			return BIT(heap->id);
	}

	pr_err("%s: bad heap flags %#x\n", __func__, flags);
	return 0;
}

/*
 * Returns the handle with one user reference and one more reference that the
 * caller drops.
 */
static struct ionl_handle *ionl_alloc(struct ionl_client *client, size_t len,
				      size_t align, unsigned int heap_id_mask,
				      unsigned int flags)
{
	struct ionl_buffer *buf = NULL;
	struct ionl_handle *handle;
	unsigned int i;
	int ret;

	len = PAGE_ALIGN(len);
	if (!len)
		return ERR_PTR(-EINVAL);

	if (len / PAGE_SIZE > totalram_pages() / 4) {
		size_t pid_total_size = ionl_total_size_by_pid(client);

		if ((len + pid_total_size) / PAGE_SIZE > totalram_pages() / 2)
			return ERR_PTR(-EINVAL);
	}

	heap_id_mask = ionl_parse_heap_id(heap_id_mask, flags);
	if (heap_id_mask == 0)
		return ERR_PTR(-EINVAL);

	for (i = 0; i < ionl_dev.nr_heaps; i++) {
		struct ionl_heap *heap = &ionl_dev.heaps[i];

		if (!(BIT(heap->id) & heap_id_mask))
			continue;
		/*
		 * Only a secure heap serves ION_FLAG_PROTECTED, and only that. The vendor
		 * driver also served unprotected requests from the secure carveout.
		 */
		if (!!(flags & ION_FLAG_PROTECTED) != (heap->type == IONL_HEAP_SECURE))
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
	ionl_buffer_put(buf);
	if (IS_ERR(handle))
		return handle;

	mutex_lock(&client->lock);
	kref_get(&handle->ref);
	ret = ionl_handle_add(client, handle);
	if (ret) {
		ionl_handle_put_nolock(handle);
		ionl_handle_put_nolock(handle);
		mutex_unlock(&client->lock);
		return ERR_PTR(ret);
	}
	handle->user_ref_count = 1;
	mutex_unlock(&client->lock);

	return handle;
}

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

/* The new dma-buf takes over the caller's buffer reference. */
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
	handle->user_ref_count = 1;
	ret = handle->id;

out_unlock:
	mutex_unlock(&client->lock);
	dma_buf_put(dmabuf);
	return ret;
}

/* The ABI defines these commands with the wrong direction bits. */
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
	pr_err("%s: len %zu align %zu heap_id_mask %u flags %x (ret %ld)\n",
	       "ALLOC", a->len, a->align, a->heap_id_mask, a->flags, ret);
}

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
		if (IS_ERR(buf))
			return PTR_ERR(buf);
		data.fd.fd = ionl_share_fd(buf);
		if (data.fd.fd < 0)
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
		/* the Exynos ION device has no custom ioctl handler */
		return -ENOTTY;
	default:
		return -ENOTTY;
	}

	if (dir & _IOC_READ) {
		if (copy_to_user((void __user *)arg, &data, _IOC_SIZE(cmd))) {
			if (cleanup_handle) {
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

		/* as in the vendor driver, a failed copy-out keeps the handle */
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

		if (get_user(i, &data32->handle))
			return -EFAULT;

		return ionl_free_id(client, i);
	}
	case COMPAT_ION_IOC_CUSTOM:
	{
		struct compat_ion_custom_data __user *data32 = compat_ptr(arg);
		compat_uint_t ccmd;
		compat_ulong_t carg;
		int err;

		err = get_user(ccmd, &data32->cmd);
		err |= get_user(carg, &data32->arg);
		if (err)
			return err;

		return -ENOTTY;
	}
	case COMPAT_ION_IOC_SYNC_PARTIAL:
	{
		struct compat_ion_fd_partial_data __user *data32 = compat_ptr(arg);
		compat_int_t fd, handle;
		compat_off_t offset;
		compat_size_t len;
		int err;

		err = get_user(handle, &data32->handle);
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
 * Every handle of the client goes, whatever its counts. A buffer that an
 * exported dma-buf still holds lives until that dma-buf is released.
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

static const struct file_operations ionl_fops = {
	.owner = THIS_MODULE,
	.open = ionl_open,
	.release = ionl_release,
	.unlocked_ioctl = ionl_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = ionl_compat_ioctl,
#endif
};

static struct miscdevice ionl_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ion",
	.fops = &ionl_fops,
};

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

static int ionl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	unsigned int i;
	int ret;

	if (ionl_dev.dev)
		return -EBUSY;

	ret = dma_coerce_mask_and_coherent(dev, DMA_BIT_MASK(64));
	if (ret)
		return ret;

	ret = ionl_heaps_init(dev);
	if (ret) {
		dev_err(dev, "heap table: %d\n", ret);
		return ret;
	}
	ionl_dev.dev = dev;

	ret = misc_register(&ionl_miscdev);
	if (ret) {
		ionl_dev.dev = NULL;
		ionl_dev.heaps = NULL;
		ionl_dev.nr_heaps = 0;
		return ret;
	}

	ionl_dev.debug_root = debugfs_create_dir("ion_legacy", NULL);
	debugfs_create_file("heaps", 0444, ionl_dev.debug_root, NULL, &ionl_heaps_fops);
	debugfs_create_file("clients", 0444, ionl_dev.debug_root, NULL, &ionl_clients_fops);

	for (i = 0; i < ionl_dev.nr_heaps; i++)
		dev_dbg(dev, "heap %u: %s, compat ids %#x\n", ionl_dev.heaps[i].id,
			ionl_dev.heaps[i].name, ionl_dev.heaps[i].compat_ids);

	return 0;
}

static const struct of_device_id ionl_of_match[] = {
	{ .compatible = "samsung,exynos8895-ion-legacy" },
	{ }
};

static struct platform_driver ionl_driver = {
	.probe = ionl_probe,
	.driver = {
		.name = "ion-legacy",
		.of_match_table = ionl_of_match,
		.suppress_bind_attrs = true,
	},
};

static int __init ionl_init(void)
{
	return platform_driver_register(&ionl_driver);
}
device_initcall(ionl_init);

MODULE_IMPORT_NS("DMA_BUF");
MODULE_DESCRIPTION("Legacy Exynos ION ABI over dma-buf and CMA");
MODULE_LICENSE("GPL");
