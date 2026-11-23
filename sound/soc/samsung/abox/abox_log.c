// SPDX-License-Identifier: GPL-2.0-only
/*
 * sound/soc/samsung/abox/abox_log.c - Samsung ABOX log driver
 *
 * Copyright (c) 2016 Samsung Electronics Co. Ltd.
 */
/* #define DEBUG */
#include <linux/debugfs.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <sound/samsung/abox.h>

#include "abox.h"
#include "abox_dbg.h"
#include "abox_log.h"

#define SIZE_OF_BUFFER (SZ_2M)

#define S_IRWUG (S_IRUSR|S_IRGRP|S_IWUSR|S_IWGRP)

struct abox_log_kernel_buffer {
	char *buffer;
	unsigned int index;
	bool wrap;
	bool updated;
	wait_queue_head_t wq;
};

struct abox_log_buffer_info {
	struct list_head list;
	struct device *dev;
	int id;
	struct mutex lock;
	struct ABOX_LOG_BUFFER *log_buffer;
	struct abox_log_kernel_buffer kernel_buffer;
};

static LIST_HEAD(abox_log_list_head);

static void abox_log_memcpy(struct device *dev,
	struct abox_log_kernel_buffer *kernel_buffer,
	const char *src, size_t size)
{
	size_t left_size = SIZE_OF_BUFFER - kernel_buffer->index;

	dev_dbg(dev, "%s(%zu)\n", __func__, size);

	if (left_size < size) {
		memcpy_fromio(kernel_buffer->buffer + kernel_buffer->index,
				(const void __iomem *)src, left_size);
		src += left_size;
		size -= left_size;
		kernel_buffer->index = 0;
		kernel_buffer->wrap = true;
	}
	memcpy_fromio(kernel_buffer->buffer + kernel_buffer->index,
			(const void __iomem *)src, size);
	kernel_buffer->index += size;
}

static void abox_log_flush(struct device *dev,
		struct abox_log_buffer_info *info)
{
	struct ABOX_LOG_BUFFER *log_buffer = info->log_buffer;
	unsigned int index_writer = log_buffer->index_writer;
	struct abox_log_kernel_buffer *kernel_buffer = &info->kernel_buffer;

	if (log_buffer->index_reader == index_writer)
		return;

	dev_dbg(dev, "%s(%d): index_writer=%u, index_reader=%u, size=%u\n",
			__func__, info->id, index_writer,
			log_buffer->index_reader, log_buffer->size);

	mutex_lock(&info->lock);

	if (log_buffer->index_reader > index_writer) {
		abox_log_memcpy(info->dev, kernel_buffer,
				log_buffer->buffer + log_buffer->index_reader,
				log_buffer->size - log_buffer->index_reader);
		log_buffer->index_reader = 0;
	}
	abox_log_memcpy(info->dev, kernel_buffer,
			log_buffer->buffer + log_buffer->index_reader,
			index_writer - log_buffer->index_reader);
	log_buffer->index_reader = index_writer;
	mutex_unlock(&info->lock);

	kernel_buffer->updated = true;
	wake_up_interruptible(&kernel_buffer->wq);
}

void abox_log_flush_all(struct device *dev)
{
	struct abox_log_buffer_info *info;

	dev_dbg(dev, "%s\n", __func__);

	list_for_each_entry(info, &abox_log_list_head, list) {
		abox_log_flush(info->dev, info);
	}
}
EXPORT_SYMBOL(abox_log_flush_all);

static unsigned long abox_log_flush_all_work_rearm_self;
static void abox_log_flush_all_work_func(struct work_struct *work);
static DECLARE_DEFERRABLE_WORK(abox_log_flush_all_work,
		abox_log_flush_all_work_func);

static void abox_log_flush_all_work_func(struct work_struct *work)
{
	abox_log_flush_all(NULL);
	schedule_delayed_work(&abox_log_flush_all_work, msecs_to_jiffies(3000));
	set_bit(0, &abox_log_flush_all_work_rearm_self);
}

void abox_log_schedule_flush_all(struct device *dev)
{
	if (test_and_clear_bit(0, &abox_log_flush_all_work_rearm_self))
		cancel_delayed_work(&abox_log_flush_all_work);

	schedule_delayed_work(&abox_log_flush_all_work, msecs_to_jiffies(100));
}
EXPORT_SYMBOL(abox_log_schedule_flush_all);

static ssize_t abox_log_file_index;

static int abox_log_file_open(struct inode *inode, struct  file *file)
{
	struct abox_log_buffer_info *info = inode->i_private;

	dev_dbg(info->dev, "%s\n", __func__);

	file->private_data = inode->i_private;
	abox_log_file_index = -1;

	return 0;
}

static ssize_t abox_log_file_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	struct abox_log_buffer_info *info = file->private_data;
	struct abox_log_kernel_buffer *kernel_buffer = &info->kernel_buffer;
	char *buffer = kernel_buffer->buffer;
	unsigned int index;
	size_t end, size;
	bool first = (abox_log_file_index < 0);
	int result;

	dev_dbg(info->dev, "%s(%zu, %lld)\n", __func__, count, *ppos);

	mutex_lock(&info->lock);

	if (abox_log_file_index < 0) {
		abox_log_file_index = likely(kernel_buffer->wrap) ?
				kernel_buffer->index : 0;
	}

	do {
		index = kernel_buffer->index;
		end = ((abox_log_file_index < index) ||
				((abox_log_file_index == index)	&& !first)) ?
				index : SIZE_OF_BUFFER;
		size = min(end - abox_log_file_index, count);
		if (size == 0) {
			mutex_unlock(&info->lock);
			if (file->f_flags & O_NONBLOCK) {
				dev_dbg(info->dev, "non block\n");
				return -EAGAIN;
			}
			kernel_buffer->updated = false;

			result = wait_event_interruptible(kernel_buffer->wq,
					kernel_buffer->updated);
			if (result != 0) {
				dev_dbg(info->dev, "interrupted\n");
				return result;
			}
			mutex_lock(&info->lock);
		}
	} while (size == 0);

	dev_dbg(info->dev, "start=%zd, end=%zd size=%zd\n",
			abox_log_file_index, end, size);
	if (copy_to_user(buf, buffer + abox_log_file_index, size)) {
		mutex_unlock(&info->lock);
		return -EFAULT;
	}

	abox_log_file_index += size;
	if (abox_log_file_index >= SIZE_OF_BUFFER)
		abox_log_file_index = 0;

	mutex_unlock(&info->lock);

	dev_dbg(info->dev, "%s: size = %zd\n", __func__, size);

	return size;
}

static __poll_t abox_log_file_poll(struct file *file, poll_table *wait)
{
	struct abox_log_buffer_info *info = file->private_data;
	struct abox_log_kernel_buffer *kernel_buffer = &info->kernel_buffer;

	dev_dbg(info->dev, "%s\n", __func__);

	poll_wait(file, &kernel_buffer->wq, wait);
	return EPOLLIN | EPOLLRDNORM;
}

static const struct file_operations abox_log_fops = {
	.open = abox_log_file_open,
	.read = abox_log_file_read,
	.poll = abox_log_file_poll,
	.llseek = generic_file_llseek,
	.owner = THIS_MODULE,
};

static struct abox_log_buffer_info abox_log_buffer_info_new;

static void abox_log_register_buffer_work_func(struct work_struct *work)
{
	struct device *dev;
	int id;
	struct ABOX_LOG_BUFFER *buffer;
	struct abox_log_buffer_info *info;
	char name[16];

	dev = abox_log_buffer_info_new.dev;
	id = abox_log_buffer_info_new.id;
	buffer = abox_log_buffer_info_new.log_buffer;
	abox_log_buffer_info_new.dev = NULL;
	abox_log_buffer_info_new.id = 0;
	abox_log_buffer_info_new.log_buffer = NULL;

	dev_info(dev, "%s(%d)\n", __func__, id);

	info = vmalloc(sizeof(*info));
	if (!info)
		return;
	info->kernel_buffer.buffer = vzalloc(SIZE_OF_BUFFER);
	if (!info->kernel_buffer.buffer) {
		vfree(info);
		return;
	}
	mutex_init(&info->lock);
	info->id = id;
	info->kernel_buffer.index = 0;
	info->kernel_buffer.wrap = false;
	init_waitqueue_head(&info->kernel_buffer.wq);
	info->dev = dev;
	info->log_buffer = buffer;
	list_add_tail(&info->list, &abox_log_list_head);

	snprintf(name, sizeof(name), "log-%02d", id);
	debugfs_create_file(name, S_IRWUG, abox_dbg_get_root_dir(), info,
			&abox_log_fops);
}

static DECLARE_WORK(abox_log_register_buffer_work,
		abox_log_register_buffer_work_func);

int abox_log_register_buffer(struct device *dev, int id,
		struct ABOX_LOG_BUFFER *buffer)
{
	struct abox_log_buffer_info *info;

	dev_dbg(dev, "%s(%d)\n", __func__, id);

	if (abox_log_buffer_info_new.dev != NULL ||
			abox_log_buffer_info_new.id > 0 ||
			abox_log_buffer_info_new.log_buffer != NULL) {
		return -EBUSY;
	}

	list_for_each_entry(info, &abox_log_list_head, list) {
		if (info->id == id) {
			dev_dbg(dev, "already registered log: %d\n", id);
			return 0;
		}
	}

	abox_log_buffer_info_new.dev = dev;
	abox_log_buffer_info_new.id = id;
	abox_log_buffer_info_new.log_buffer = buffer;
	schedule_work(&abox_log_register_buffer_work);

	return 0;
}
EXPORT_SYMBOL(abox_log_register_buffer);
