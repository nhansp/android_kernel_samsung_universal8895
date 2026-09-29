// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung sec_key sysfs device and hall input handler.
 * Spec: L6F 5.14 (lines 2462-2508).
 * Provides /sys/devices/virtual/sec/sec_key/hall_detect and sec_key_hall_closed().
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

/*
 * Switch state: 1 = closed, 0 = open (spec L6F 5.14 lines 2478, 2485-2486).
 * Seeded from test_bit(SW_LID, dev->sw) when the hall input device connects.
 */
static int lid;
static struct device *sec_key_dev;

/*
 * Kernel interface for other drivers (spec L6F 5.14 lines 2487-2502; run 7 / V-20).
 * Callable from any context without locking.
 */
bool sec_key_hall_closed(void)
{
	return READ_ONCE(lid) != 0;
}
EXPORT_SYMBOL_GPL(sec_key_hall_closed);

/*
 * /sys/devices/virtual/sec/sec_key/hall_detect
 * Stock S:drivers/input/hall.c:44-49, :53, :270; spec L6F 5.14 lines 2482-2486.
 */
static ssize_t hall_detect_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, READ_ONCE(lid) ? "CLOSE\n" : "OPEN\n");
}
static DEVICE_ATTR_RO(hall_detect);

static struct attribute *sec_key_attrs[] = {
	&dev_attr_hall_detect.attr,
	NULL,
};
ATTRIBUTE_GROUPS(sec_key);

/*
 * Runs with dev->event_lock held and IRQs off (A:drivers/input/input.c:391-395, :2533-2544).
 * Spec L6F 5.14 lines 2477-2478.
 */
static void sec_key_event(struct input_handle *handle, unsigned int type,
			  unsigned int code, int value)
{
	if (type == EV_SW && code == SW_LID)
		WRITE_ONCE(lid, value);
}

/*
 * Connect to the "hall" input device (A:include/linux/input.h:330, :430, :436;
 * A:drivers/input/keyboard/gpio_keys.c:720-736, :778, :879; spec L6F 5.14 lines 2468-2476).
 */
static int sec_key_connect(struct input_handler *handler, struct input_dev *dev,
			   const struct input_device_id *id)
{
	struct input_handle *handle;
	int error;

	if (!dev->name || strcmp(dev->name, "hall") != 0)
		return -ENODEV;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "sec_key_hall";

	error = input_register_handle(handle);
	if (error)
		goto err_free_handle;

	error = input_open_device(handle);
	if (error)
		goto err_unregister_handle;

	WRITE_ONCE(lid, test_bit(SW_LID, dev->sw));
	pr_info("samsung-sec-key: hall_detect bound to %s\n", dev_name(&dev->dev));

	return 0;

err_unregister_handle:
	input_unregister_handle(handle);
err_free_handle:
	kfree(handle);
	return error;
}

/*
 * Disconnect handler (A:include/linux/input.h:331; spec L6F 5.14 line 2476).
 */
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

/*
 * Late initcall: runs after sec_class (subsys_initcall, spec L6F 5.9, 5.14 lines 2479-2481).
 */
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
