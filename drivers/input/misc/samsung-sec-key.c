// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung sec_key device: the flip cover state of the "hall" and
 * "certify_hall" switch input devices, in the sysfs nodes of the vendor kernel.
 */

#include <linux/device.h>
#include <linux/export.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/input/sec_key.h>
#include <linux/module.h>
#include <linux/sec_sysfs.h>
#include <linux/slab.h>
#include <linux/string.h>

static int lid;
static int certify;
static struct device *sec_key_dev;

bool sec_key_hall_closed(void)
{
	return READ_ONCE(lid) != 0;
}
EXPORT_SYMBOL_GPL(sec_key_hall_closed);

static ssize_t hall_detect_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, READ_ONCE(lid) ? "CLOSE\n" : "OPEN\n");
}
static DEVICE_ATTR_RO(hall_detect);

static ssize_t certify_hall_detect_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, READ_ONCE(certify) ? "CLOSE\n" : "OPEN\n");
}
static DEVICE_ATTR_RO(certify_hall_detect);

static struct attribute *sec_key_attrs[] = {
	&dev_attr_hall_detect.attr,
	&dev_attr_certify_hall_detect.attr,
	NULL,
};
ATTRIBUTE_GROUPS(sec_key);

static void sec_key_event(struct input_handle *handle, unsigned int type,
			  unsigned int code, int value)
{
	if (type == EV_SW && code == SW_LID)
		WRITE_ONCE(lid, value);
	else if (type == EV_SW && code == SW_MACHINE_COVER)
		WRITE_ONCE(certify, value);
}

static int sec_key_connect(struct input_handler *handler, struct input_dev *dev,
			   const struct input_device_id *id)
{
	struct input_handle *handle;
	bool is_certify;
	int error;

	if (!dev->name)
		return -ENODEV;
	if (!strcmp(dev->name, "hall"))
		is_certify = false;
	else if (!strcmp(dev->name, "certify_hall"))
		is_certify = true;
	else
		return -ENODEV;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = is_certify ? "sec_key_certify" : "sec_key_hall";

	error = input_register_handle(handle);
	if (error)
		goto err_free_handle;

	error = input_open_device(handle);
	if (error)
		goto err_unregister_handle;

	if (is_certify) {
		WRITE_ONCE(certify, test_bit(SW_MACHINE_COVER, dev->sw));
		dev_dbg(&dev->dev, "certify_hall_detect bound\n");
	} else {
		WRITE_ONCE(lid, test_bit(SW_LID, dev->sw));
		dev_dbg(&dev->dev, "hall_detect bound\n");
	}

	return 0;

err_unregister_handle:
	input_unregister_handle(handle);
err_free_handle:
	kfree(handle);
	return error;
}

static void sec_key_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id sec_key_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT | INPUT_DEVICE_ID_MATCH_SWBIT,
		.evbit = { BIT_MASK(EV_SW) },
		.swbit = { [BIT_WORD(SW_LID)] = BIT_MASK(SW_LID) },
	},
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT | INPUT_DEVICE_ID_MATCH_SWBIT,
		.evbit = { BIT_MASK(EV_SW) },
		.swbit = { [BIT_WORD(SW_MACHINE_COVER)] = BIT_MASK(SW_MACHINE_COVER) },
	},
	{ },
};
MODULE_DEVICE_TABLE(input, sec_key_ids);

static struct input_handler sec_key_handler = {
	.event = sec_key_event,
	.connect = sec_key_connect,
	.disconnect = sec_key_disconnect,
	.name = "sec_key_hall",
	.id_table = sec_key_ids,
};

/* late: the sec class is registered at subsys_initcall */
static int __init sec_key_init(void)
{
	int error;

	sec_key_dev = sec_device_create_with_groups(NULL, sec_key_groups, "sec_key");
	if (IS_ERR(sec_key_dev))
		return PTR_ERR(sec_key_dev);

	error = input_register_handler(&sec_key_handler);
	if (error) {
		sec_device_destroy(sec_key_dev);
		return error;
	}

	return 0;
}
late_initcall(sec_key_init);

MODULE_DESCRIPTION("Samsung sec_key sysfs device and hall input handler");
MODULE_LICENSE("GPL");
