// SPDX-License-Identifier: GPL-2.0-only
/*
 * Simple-interactive devfreq governor: the target is the PM QoS floor its
 * device's driver hands in, with an optional hold before a decrease.
 *
 * Port of the stock governor (S:drivers/devfreq/governor_simpleinteractive.c)
 * without its CONFIG_EXYNOS_WD_DVFS load path, which stock built without
 * (IKC:4313 "# CONFIG_EXYNOS_WD_DVFS is not set"); spec L2b 5.2.
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

/*
 * Stock devfreq_change_freq_task() (S:drivers/devfreq/governor_simpleinteractive.c:249-266),
 * which the hold timer woke (:268-274): re-evaluate once the hold expired.
 */
static void devfreq_simple_interactive_hold(struct work_struct *work)
{
	struct devfreq_simple_interactive_data *data =
		container_of(to_delayed_work(work),
			     struct devfreq_simple_interactive_data, hold_work);

	mutex_lock(&data->df->lock);
	update_devfreq(data->df);
	mutex_unlock(&data->df->lock);
}

/* Called by the devfreq core with df->lock held. */
static int devfreq_simple_interactive_func(struct devfreq *df,
					   unsigned long *freq)
{
	struct devfreq_simple_interactive_data *data = df->data;
	unsigned long now = jiffies;
	unsigned long min_freq, max_freq, qos_freq;

	/* S:drivers/devfreq/governor_simpleinteractive.c:160-163 */
	if (!data)
		return -ENODATA;

	/*
	 * Stock: *freq = pm_qos_request(class), capped by the class_max request
	 * (:165-180). Here the driver's own floor/ceiling are dev_pm_qos limits
	 * that devfreq_get_freq_range() folds in, and the class aggregate is the
	 * driver's (spec L2b 5.1), read without its lock.
	 */
	devfreq_get_freq_range(df, &min_freq, &max_freq);
	qos_freq = data->qos_khz ?
		(unsigned long)READ_ONCE(*data->qos_khz) * HZ_PER_KHZ : 0;
	*freq = max3(min_freq, qos_freq, data->coupled_hz);

	/* :192-193 */
	if (!data->delay_ms)
		goto out;

	/* :195-208: the hold is counted from the last frequency change; unit 10 ms */
	if (data->prev_freq != df->previous_freq)
		data->expires = data->changed_time +
			msecs_to_jiffies(roundup(data->delay_ms, DELAY_TIME_RANGE));

	/* :210-217: a decrease inside the hold keeps the frequency and re-checks at expiry */
	if (max_freq > df->previous_freq && *freq < df->previous_freq &&
	    time_before(now, data->expires)) {
		*freq = df->previous_freq;
		if (!delayed_work_pending(&data->hold_work))
			mod_delayed_work(system_wq, &data->hold_work,
					 data->expires - now);
		goto out;
	}

	/* :218-222; not the _sync variant: the work takes df->lock, held here */
	cancel_delayed_work(&data->hold_work);
	data->changed_time = now;

out:
	/* :229-230 */
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
		/* stock :287-288 */
		if (!gov)
			return -EINVAL;
		gov->df = devfreq;
		INIT_DELAYED_WORK(&gov->hold_work, devfreq_simple_interactive_hold);
		gov->changed_time = jiffies;
		/*
		 * Stock registered PM QoS notifiers here (:293-306); the class owner
		 * now calls update_devfreq() itself. First evaluation as
		 * governor_powersave does.
		 */
		mutex_lock(&devfreq->lock);
		ret = update_devfreq(devfreq);
		mutex_unlock(&devfreq->lock);
		break;

	case DEVFREQ_GOV_STOP:
		/* stock :379-380 */
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
