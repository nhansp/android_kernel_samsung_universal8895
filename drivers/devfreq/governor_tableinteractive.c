// SPDX-License-Identifier: GPL-2.0-only
/*
 * Table-interactive devfreq governor: the stock Exynos Mali DVFS "interactive"
 * governor (S:drivers/gpu/arm/b_r16p0/platform/exynos/gpu_dvfs_governor.c:115-156
 * gpu_dvfs_governor_interactive(); S: = the stock kernel tree).
 *
 * One step per poll: up one level when the load is above the level's
 * max_load, or, when also above highspeed_load and below the high-speed level,
 * straight to the high-speed level; down one level after down_staycount polls
 * below the level's min_load. The table is the stock gpu_dvfs_table columns
 * 1-3 and interactive_info, handed in by the device driver as governor data.
 *
 * The stock step is its own state; here the level is that of the frequency the
 * device runs at, so a clamp by the devfreq limits (a cooling device, a
 * user min/max) is taken as the new step, as stock did for its own limit
 * (:135-136, :97-98).
 */

#include <linux/devfreq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/math64.h>
#include <linux/minmax.h>

#include "governor.h"

/* the highest level not above @freq, 0 if every level is above it */
static unsigned int tbl_level_floor(const struct devfreq_table_interactive_data *d,
				    unsigned long freq)
{
	unsigned int i, lvl = 0;

	for (i = 0; i < d->nlevels; i++)
		if (d->levels[i].freq_hz <= freq)
			lvl = i;
	return lvl;
}

/* the lowest level not below @freq, the top level if every level is below it */
static unsigned int tbl_level_ceil(const struct devfreq_table_interactive_data *d,
				   unsigned long freq)
{
	unsigned int i;

	for (i = 0; i < d->nlevels; i++)
		if (d->levels[i].freq_hz >= freq)
			return i;
	return d->nlevels - 1;
}

static int devfreq_table_interactive_func(struct devfreq *df, unsigned long *freq)
{
	struct devfreq_table_interactive_data *d = df->data;
	unsigned long min_freq, max_freq;
	unsigned int cur, lo, hi, hs, headroom;
	int util, err;
	struct devfreq_dev_status *stat;

	if (!d || !d->nlevels)
		return -EINVAL;

	err = devfreq_update_stats(df);
	if (err)
		return err;
	stat = &df->last_status;

	devfreq_get_freq_range(df, &min_freq, &max_freq);
	lo = tbl_level_ceil(d, min_freq);	/* stock gpu_min_clock / min lock */
	hi = tbl_level_floor(d, max_freq);	/* stock gpu_max_clock / max limit */
	if (lo > hi)
		lo = hi;

	/* stock utilisation = 100 * busy / (busy + idle); -1 = no data (gpu_integration_callbacks.c:598-611) */
	util = stat->total_time ? div64_u64(100ULL * stat->busy_time, stat->total_time) : -1;

	/* the load was measured at the raised clock, but the table steps from its own level */
	if (d->base_level >= 0)
		cur = d->base_level;
	else
		cur = tbl_level_floor(d, stat->current_frequency);
	if (cur > hi)
		cur = hi;
	if (cur < lo)
		cur = lo;

	if (cur < hi && util > (int)d->levels[cur].max_load) {
		/* :122-124 */
		hs = tbl_level_floor(d, d->highspeed_hz);
		if (d->highspeed_hz && cur < hs && util > (int)d->highspeed_load) {
			/* :122-130 */
			if (d->delay_count == d->highspeed_delay) {
				cur = hs;
				d->delay_count = 0;
			} else {
				d->delay_count++;
			}
		} else {
			/* :131-134 */
			cur++;
			d->delay_count = 0;
		}
		cur = min(cur, hi);	/* :135-136 */
		d->down_requirement = d->levels[cur].down_staycount;
	} else if (cur > lo && util < (int)d->levels[cur].min_load) {
		/* :138-145 */
		d->delay_count = 0;
		if (d->down_requirement <= 1) {
			cur--;
			d->down_requirement = d->levels[cur].down_staycount;
		} else {
			d->down_requirement--;
		}
	} else {
		/* :146-149 */
		d->delay_count = 0;
		d->down_requirement = d->levels[cur].down_staycount;
	}

	*freq = d->levels[cur].freq_hz;
	WRITE_ONCE(d->base_level, -1);

	/* pixel_gpu_dvfs_governor.c level_for_capacity(): the lowest level covering the headroom */
	headroom = READ_ONCE(d->capacity_headroom_khz);
	if (headroom) {
		unsigned long want = d->levels[cur].freq_hz + headroom * 1000UL;
		unsigned int l = tbl_level_ceil(d, want);

		if (l > hi)
			l = hi;
		if (l > cur) {
			*freq = d->levels[l].freq_hz;
			WRITE_ONCE(d->base_level, cur);
		}
	}
	return 0;
}

static int devfreq_table_interactive_handler(struct devfreq *df,
					     unsigned int event, void *data)
{
	struct devfreq_table_interactive_data *d = df->data;

	switch (event) {
	case DEVFREQ_GOV_START:
		if (!d || !d->nlevels)
			return -EINVAL;
		/* stock gpu_dvfs_governor_setting(): down_requirement = 1 (:337) */
		d->down_requirement = 1;
		d->delay_count = 0;
		d->base_level = -1;
		devfreq_monitor_start(df);
		break;

	case DEVFREQ_GOV_STOP:
		devfreq_monitor_stop(df);
		break;

	case DEVFREQ_GOV_UPDATE_INTERVAL:
		devfreq_update_interval(df, (unsigned int *)data);
		break;

	case DEVFREQ_GOV_SUSPEND:
		devfreq_monitor_suspend(df);
		break;

	case DEVFREQ_GOV_RESUME:
		devfreq_monitor_resume(df);
		break;

	default:
		break;
	}

	return 0;
}

static struct devfreq_governor devfreq_table_interactive = {
	.name = DEVFREQ_GOV_TABLE_INTERACTIVE,
	.attrs = DEVFREQ_GOV_ATTR_POLLING_INTERVAL | DEVFREQ_GOV_ATTR_TIMER,
	.get_target_freq = devfreq_table_interactive_func,
	.event_handler = devfreq_table_interactive_handler,
};

static int __init devfreq_table_interactive_init(void)
{
	return devfreq_add_governor(&devfreq_table_interactive);
}
subsys_initcall(devfreq_table_interactive_init);
