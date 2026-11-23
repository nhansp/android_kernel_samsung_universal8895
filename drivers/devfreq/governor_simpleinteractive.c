// SPDX-License-Identifier: GPL-2.0-only
/*
 * Simple-interactive devfreq governor: runs the device at the PM QoS floor
 * its driver hands in, optionally holding a higher frequency for a while
 * before a decrease.
 *
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 */

#include <linux/devfreq.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/units.h>
#include <linux/workqueue.h>

#include "governor.h"

/* the hold before a decrease is rounded up to a multiple of this */
#define SIMPLE_INTERACTIVE_HOLD_STEP_MS	10

static void devfreq_simple_interactive_hold(struct work_struct *work)
{
	struct devfreq_simple_interactive_data *data =
		container_of(to_delayed_work(work),
			     struct devfreq_simple_interactive_data, hold_work);

	mutex_lock(&data->df->lock);
	update_devfreq(data->df);
	mutex_unlock(&data->df->lock);
}

static int devfreq_simple_interactive_func(struct devfreq *df,
					   unsigned long *freq)
{
	struct devfreq_simple_interactive_data *data = df->data;
	unsigned long now = jiffies;
	unsigned long min_freq, max_freq, qos_freq;

	if (!data)
		return -ENODATA;

	devfreq_get_freq_range(df, &min_freq, &max_freq);
	qos_freq = data->qos_khz ?
		(unsigned long)READ_ONCE(*data->qos_khz) * HZ_PER_KHZ : 0;
	*freq = max3(min_freq, qos_freq, data->coupled_hz);

	if (!data->delay_ms)
		goto out;

	if (data->prev_freq != df->previous_freq)
		data->expires = data->changed_time +
			msecs_to_jiffies(roundup(data->delay_ms,
						 SIMPLE_INTERACTIVE_HOLD_STEP_MS));

	if (max_freq > df->previous_freq && *freq < df->previous_freq &&
	    time_before(now, data->expires)) {
		*freq = df->previous_freq;
		if (!delayed_work_pending(&data->hold_work))
			mod_delayed_work(system_wq, &data->hold_work,
					 data->expires - now);
		goto out;
	}

	/* not the _sync variant: the work takes df->lock, which is held here */
	cancel_delayed_work(&data->hold_work);
	data->changed_time = now;

out:
	data->prev_freq = df->previous_freq;
	*freq = min(max_freq, *freq);

	return 0;
}

static int devfreq_simple_interactive_handler(struct devfreq *devfreq,
					      unsigned int event, void *data)
{
	struct devfreq_simple_interactive_data *gov = devfreq->data;
	int ret = 0;

	switch (event) {
	case DEVFREQ_GOV_START:
		if (!gov)
			return -EINVAL;
		gov->df = devfreq;
		INIT_DELAYED_WORK(&gov->hold_work, devfreq_simple_interactive_hold);
		gov->changed_time = jiffies;
		mutex_lock(&devfreq->lock);
		ret = update_devfreq(devfreq);
		mutex_unlock(&devfreq->lock);
		break;

	case DEVFREQ_GOV_STOP:
		if (gov)
			cancel_delayed_work_sync(&gov->hold_work);
		break;

	default:
		break;
	}

	return ret;
}

static struct devfreq_governor devfreq_simple_interactive = {
	.name = DEVFREQ_GOV_SIMPLE_INTERACTIVE,
	.flags = DEVFREQ_GOV_FLAG_IMMUTABLE,
	.get_target_freq = devfreq_simple_interactive_func,
	.event_handler = devfreq_simple_interactive_handler,
};

static int __init devfreq_simple_interactive_init(void)
{
	return devfreq_add_governor(&devfreq_simple_interactive);
}
subsys_initcall(devfreq_simple_interactive_init);

static void __exit devfreq_simple_interactive_exit(void)
{
	int ret;

	ret = devfreq_remove_governor(&devfreq_simple_interactive);
	if (ret)
		pr_err("%s: failed remove governor %d\n", __func__, ret);
}
module_exit(devfreq_simple_interactive_exit);
MODULE_DESCRIPTION("DEVFREQ Simple Interactive governor");
MODULE_LICENSE("GPL");
