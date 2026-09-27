// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2020 Samsung Electronics Co., Ltd.
 * Copyright 2020 Google LLC.
 * Copyright 2024 Linaro Ltd.
 */

#include <linux/atomic.h>
#include <linux/bitfield.h>
#include <linux/bitmap.h>
#include <linux/bits.h>
#include <linux/cleanup.h>
#include <linux/container_of.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/firmware/samsung/exynos-acpm-protocol.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/ktime.h>
#include <linux/mailbox/exynos-message.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/math.h>
#include <linux/minmax.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/sched/clock.h>
#include <linux/seq_file.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#include "exynos-acpm.h"
#include "exynos-acpm-dvfs.h"
#include "exynos-acpm-pmic.h"

#define ACPM_PROTOCOL_SEQNUM		GENMASK(21, 16)

#define ACPM_POLL_TIMEOUT_US		(100 * USEC_PER_MSEC)
#define ACPM_TX_TIMEOUT_US		500000

#define ACPM_GS101_INITDATA_BASE	0xa000

/*
 * Printing of the APM firmware-log entries while the ring is drained:
 * 0 = none (default), 1 = entries with log level 0 (stock's policy with
 * debug-log-level 0), 2 = every entry (stock's debug-log-level 1).
 */
#ifndef MODULE
#undef MODULE_PARAM_PREFIX
#define MODULE_PARAM_PREFIX "acpm."
#endif
static unsigned int fw_log;
module_param(fw_log, uint, 0644);
MODULE_PARM_DESC(fw_log, "print APM firmware log entries: 0 none, 1 level 0 only, 2 all");

/*
 * APM firmware-log entry header (Exynos8895 stock,
 * S:drivers/soc/samsung/acpm/acpm_ipc.c:94-102, acpm_ipc.h:74-76): id [31:29],
 * level bit 28, timestamp index [27:23], APM systick count [22:0].
 */
#define ACPM_LOG_ID			GENMASK(31, 29)
#define ACPM_LOG_LEVEL			BIT(28)
#define ACPM_LOG_TIME_INDEX		GENMASK(27, 23)
#define ACPM_LOG_COUNT			GENMASK(22, 0)
#define ACPM_SYSTICK_NS			(1000 / 26)	/* S:drivers/soc/samsung/acpm/acpm_ipc.h:95 */
#define ACPM_MAX_TIMESTAMPS		32

/*
 * Channel memory types of the firmware's channel table (Exynos8895 stock:
 * S:drivers/soc/samsung/acpm/fw_header/framework.h:205-207 "enum
 * shard_buffer_type { TYPE_QUEUE = 1, TYPE_BUFFER, ...").
 * A TYPE_BUFFER channel is one message buffer the firmware rewrites; it has
 * no queue indices (S:drivers/soc/samsung/acpm/acpm_ipc.c:314-322).
 */
#define ACPM_CHAN_TYPE_QUEUE		1
#define ACPM_CHAN_TYPE_BUFFER		2

/*
 * Dynamic plugin attach/detach command word (Exynos8895 stock,
 * S:include/soc/samsung/acpm_ipc_ctrl.h:17-20), cmd[0] = DP_A or DP_D | id << 26.
 */
#define ACPM_PROTOCOL_ID		GENMASK(28, 26)
#define ACPM_PROTOCOL_DP_A		BIT(25)
#define ACPM_PROTOCOL_DP_D		BIT(24)

#define ACPM_MAX_PLUGINS		16
#define ACPM_PLUGIN_NAME_LEN		50	/* S:drivers/soc/samsung/acpm/acpm.c:98,140-141 */
#define ACPM_FVMAP_SIZE			SZ_8K	/* S:drivers/soc/samsung/cal-if/fvmap.c:15 */
#define ACPM_FVMAP_MAX_DOMAINS		16

/**
 * struct acpm_plugin_shmem - firmware plugin descriptor in SRAM.
 *
 * Exynos8895 stock struct plugin, CONFIG_EXYNOS_ACPM layout
 * (S:drivers/soc/samsung/acpm/fw_header/common.h:86-89,102-123); 44 bytes,
 * the same on the Cortex-M3 (AAPCS) and arm64.
 */
struct acpm_plugin_shmem {
	u32 id;
	u32 base_addr;
	u32 acpm_ops;
	u32 plugin_ops;
	u32 extern_func_mask;
	u32 timer_period;
	u32 timer_multiplier;
	u8 is_attached;
	u8 pad0[3];
	u32 size;
	u8 stay_attached;
	u8 pad1[3];
	u32 fw_name;
};
static_assert(sizeof(struct acpm_plugin_shmem) == 44);

/*
 * struct fvmap_header of the DVFS plugin's frequency/voltage map, one per ACPM
 * DVFS domain, 24 bytes each (S:drivers/soc/samsung/cal-if/fvmap.h:5-20):
 * u8 num_of_lv at +1, u8 num_of_pll at +3, u16 o_members at +18 (an array of
 * u16 offsets of struct pll_header, whose first word is the PLL address,
 * :22-31), u16 o_ratevolt at +20 (struct rate_volt {u32 rate; u32 volt;},
 * :33-40). All offsets are relative to the start of the map.
 */
#define ACPM_FVMAP_HDR_SIZE		24
#define ACPM_FVMAP_HDR_NUM_LV		1
#define ACPM_FVMAP_HDR_NUM_PLL		3
#define ACPM_FVMAP_HDR_O_MEMBERS	18
#define ACPM_FVMAP_HDR_O_RATEVOLT	20

/**
 * struct acpm_shmem - shared memory configuration information.
 * @reserved:	unused fields.
 * @chans:	offset to array of struct acpm_chan_shmem.
 * @reserved1:	unused fields.
 * @num_chans:	number of channels.
 * @reserved2:	unused fields.
 * @ktime_index: offset of the word holding the current timestamp index.
 * @log_buf_rear: offset of the firmware-log ring rear index (written by AP).
 * @log_buf_front: offset of the firmware-log ring front index.
 * @log_data:	offset of the firmware-log entries.
 * @log_entry_size: size of one log entry in bytes.
 * @log_entry_len: number of log entries in the ring.
 *
 * Words 7-17 are only read on SoCs with a firmware-log drain (Exynos8895
 * stock struct acpm_framework, S:drivers/soc/samsung/acpm/fw_header/framework.h:52-62).
 */
struct acpm_shmem {
	u32 reserved[2];
	u32 chans;
	u32 reserved1[3];
	u32 num_chans;
	u32 reserved2[5];
	u32 ktime_index;
	u32 log_buf_rear;
	u32 log_buf_front;
	u32 log_data;
	u32 log_entry_size;
	u32 log_entry_len;
};

/**
 * struct acpm_chan_shmem - descriptor of a shared memory channel.
 *
 * @id:			channel ID.
 * @field:		the channel's bit in the mailbox status register.
 * @owner:		firmware plugin that owns the channel.
 * @type:		ACPM_CHAN_TYPE_QUEUE or ACPM_CHAN_TYPE_BUFFER.
 * @rx_rear:		rear pointer of APM RX queue (TX for AP).
 * @rx_front:		front pointer of APM RX queue (TX for AP).
 * @rx_base:		base address of APM RX queue (TX for AP).
 * @reserved1:		unused fields.
 * @tx_rear:		rear pointer of APM TX queue (RX for AP).
 * @tx_front:		front pointer of APM TX queue (RX for AP).
 * @tx_base:		base address of APM TX queue (RX for AP).
 * @qlen:		queue length. Applies to both TX/RX queues.
 * @mlen:		message length. Applies to both TX/RX queues.
 * @reserved2:		unused fields.
 * @poll_completion:	true when the channel works on polling.
 */
struct acpm_chan_shmem {
	u32 id;
	u32 field;
	u32 owner;
	u32 type;
	u32 rx_rear;
	u32 rx_front;
	u32 rx_base;
	u32 reserved1[3];
	u32 tx_rear;
	u32 tx_front;
	u32 tx_base;
	u32 qlen;
	u32 mlen;
	u32 reserved2[2];
	u32 poll_completion;
};

/**
 * struct acpm_queue - exynos acpm queue.
 *
 * @rear:	rear address of the queue.
 * @front:	front address of the queue.
 * @base:	base address of the queue.
 */
struct acpm_queue {
	void __iomem *rear;
	void __iomem *front;
	void __iomem *base;
};

/**
 * struct acpm_rx_data - RX queue data.
 *
 * @cmd:	pointer to where the data shall be saved.
 * @n_cmd:	number of 32-bit commands.
 * @response:	true if the client expects the RX data.
 */
struct acpm_rx_data {
	u32 *cmd;
	size_t n_cmd;
	bool response;
};

#define ACPM_SEQNUM_MAX    64

/**
 * struct acpm_chan_stats - per-channel counters, read without the channel locks.
 * @xfer:	transfers started on the channel.
 * @err:	transfers that returned an error.
 * @notify:	unsolicited messages handed to the notifier chain.
 * @late:	RX entries that answered nothing and are not notifications.
 * @ack:	answers to fire-and-forget messages.
 * @noack:	fire-and-forget sequence numbers reused without an answer.
 * @stale:	RX entries found queued before the channel was first used.
 */
struct acpm_chan_stats {
	atomic_t xfer;
	atomic_t err;
	atomic_t notify;
	atomic_t late;
	atomic_t ack;
	atomic_t noack;
	atomic_t stale;
};

/**
 * struct acpm_chan - driver internal representation of a channel.
 * @cl:		mailbox client.
 * @chan:	mailbox channel.
 * @acpm:	pointer to driver private data.
 * @tx:		TX queue. The enqueue is done by the host.
 *			- front index is written by the host.
 *			- rear index is written by the firmware.
 *
 * @rx:		RX queue. The enqueue is done by the firmware.
 *			- front index is written by the firmware.
 *			- rear index is written by the host.
 * @tx_lock:	protects TX queue.
 * @rx_lock:	protects RX queue.
 * @qlen:	queue length. Applies to both TX/RX queues.
 * @mlen:	message length. Applies to both TX/RX queues.
 * @seqnum:	sequence number of the last message enqueued on TX queue.
 * @id:		channel ID.
 * @poll_completion:	indicates if the transfer needs to be polled for
 *			completion or interrupt mode is used.
 * @field:	the channel's bit in the mailbox status register (firmware table).
 * @type:	ACPM_CHAN_TYPE_QUEUE or ACPM_CHAN_TYPE_BUFFER.
 * @rx_work:	drains an interrupt-mode channel after its mailbox interrupt.
 * @stats:	counters.
 * @bitmap_seqnum: bitmap that tracks the messages on the TX/RX queues.
 * @ff_seqnum:	sequence numbers of fire-and-forget messages on interrupt-mode
 *		channels (not waited for, so not in @bitmap_seqnum).
 * @rx_data:	internal buffer used to drain the RX queue.
 */
struct acpm_chan {
	struct mbox_client cl;
	struct mbox_chan *chan;
	struct acpm_info *acpm;
	struct acpm_queue tx;
	struct acpm_queue rx;
	struct mutex tx_lock;
	struct mutex rx_lock;

	unsigned int qlen;
	unsigned int mlen;
	u8 seqnum;
	u8 id;
	bool poll_completion;
	u32 field;
	u32 type;
	struct work_struct rx_work;
	struct acpm_chan_stats stats;

	DECLARE_BITMAP(bitmap_seqnum, ACPM_SEQNUM_MAX - 1);
	DECLARE_BITMAP(ff_seqnum, ACPM_SEQNUM_MAX - 1);
	struct acpm_rx_data rx_data[ACPM_SEQNUM_MAX];
};

/**
 * struct acpm_info - driver's private data.
 * @shmem:	pointer to the SRAM configuration data.
 * @sram_base:	base address of SRAM.
 * @chans:	pointer to the ACPM channel parameters retrieved from SRAM.
 * @dev:	pointer to the exynos-acpm device.
 * @handle:	instance of acpm_handle to send to clients.
 * @num_chans:	number of channels available for this controller.
 * @match:	per-SoC data and transport policy.
 * @nh:		notifier chain for unsolicited firmware messages.
 * @sram_size:	size of the SRAM window.
 * @nplugins:	number of firmware plugins (plugin_chan SoCs).
 * @dvfs_plugin_id: id of the DVFS plugin, -1 if none was found.
 * @dvfs_attached: its is_attached flag at probe.
 * @dvfs_stay:	its stay_attached flag at probe.
 * @dvfs_dpa:	result of its attach: "ok", "fail", "sent", "skip", "nofw" or "none".
 * @dpd_plugin_id: id of the plugin detached at probe, -1 if none.
 * @dpd_res:	result of that detach: "ok", "fail" or "none".
 * @fvmap_off:	SRAM offset of the DVFS plugin's fvmap, 0 if none.
 * @fvmap_domains: number of fvmap headers read.
 * @fvmap_lv:	number of levels per DVFS domain.
 * @fvmap_pll_pa: PLL address of member 0 per DVFS domain, 0 if none.
 * @fwlog_ok:	the firmware-log ring layout is valid and drained.
 * @ts_idx:	SRAM word holding the current timestamp index.
 * @fwlog_rear:	SRAM word holding the log ring rear index.
 * @fwlog_front: SRAM word holding the log ring front index.
 * @fwlog_data:	first log entry.
 * @fwlog_es:	log entry size in bytes.
 * @fwlog_el:	number of log entries.
 * @ts_lock:	serializes timestamp index updates (trylock, as stock).
 * @fwlog_lock:	serializes ring drains (trylock).
 * @ts:		sched_clock() per timestamp index.
 * @fwlog_entries: log entries drained.
 * @fwlog_bad:	drains stopped on a corrupt ring index.
 * @fwlog_work:	periodic drain.
 */
struct acpm_info {
	struct acpm_shmem __iomem *shmem;
	void __iomem *sram_base;
	struct acpm_chan *chans;
	struct device *dev;
	struct acpm_handle handle;
	u32 num_chans;
	const struct acpm_match_data *match;
	struct blocking_notifier_head nh;
	resource_size_t sram_size;
	u32 nplugins;
	int dvfs_plugin_id;
	u8 dvfs_attached;
	u8 dvfs_stay;
	const char *dvfs_dpa;
	int dpd_plugin_id;
	const char *dpd_res;
	u32 fvmap_off;
	u32 fvmap_domains;
	u8 fvmap_lv[ACPM_FVMAP_MAX_DOMAINS];
	u32 fvmap_pll_pa[ACPM_FVMAP_MAX_DOMAINS];
	bool fwlog_ok;
	void __iomem *ts_idx;
	void __iomem *fwlog_rear;
	void __iomem *fwlog_front;
	void __iomem *fwlog_data;
	u32 fwlog_es;
	u32 fwlog_el;
	spinlock_t ts_lock;
	spinlock_t fwlog_lock;
	u64 ts[ACPM_MAX_TIMESTAMPS];
	atomic_t fwlog_entries;
	atomic_t fwlog_bad;
	struct delayed_work fwlog_work;
};

/**
 * struct acpm_match_data - of_device_id data.
 * @initdata_base:	offset in SRAM where the channels configuration resides.
 * @acpm_clk_dev_name:	base name for the ACPM clocks device that we're registering.
 * @rx_front_check:	before a TX, also wait for the RX front to differ from the
 *			next TX front (gs101 policy).
 * @cmd_timestamp:	the PMIC/DVFS commands carry a millisecond timestamp in
 *			cmd[3] (gs101); otherwise cmd[3] is 0.
 * @poll_timeout_us:	how long a polled response is waited for.
 * @irq_rx:		serve the firmware's interrupt-mode channels (ap_poll = 0)
 *			from the mailbox RX interrupt; also drop RX entries queued
 *			before the first use of a channel, and never fail on an
 *			RX entry that answers no request.
 * @notify_chan:	channel whose unsolicited messages are firmware
 *			notifications (logged), -1 for none.
 * @plugin_chan:	channel of the dynamic plugin commands (attach/detach at
 *			probe), -1 for none.
 * @fvmap_names:	names of the DVFS domains whose fvmap is logged at probe,
 *			indexed by domain; NULL entries are read but not logged.
 * @fvmap_domains:	number of DVFS domains in the fvmap.
 * @fwlog_period_ms:	period of the firmware-log drain, 0 = no drain and no
 *			timestamp index updates.
 * @fwlog_first_ms:	delay of the first periodic drain after probe.
 * @num_timestamps:	modulus of the timestamp index (at most 32).
 */
struct acpm_match_data {
	loff_t initdata_base;
	const char *acpm_clk_dev_name;
	bool rx_front_check;
	bool cmd_timestamp;
	u32 poll_timeout_us;
	bool irq_rx;
	int notify_chan;
	int plugin_chan;
	const char * const *fvmap_names;
	u32 fvmap_domains;
	u32 fwlog_period_ms;
	u32 fwlog_first_ms;
	u32 num_timestamps;
};

#define client_to_acpm_chan(c) container_of(c, struct acpm_chan, cl)
#define handle_to_acpm_info(h) container_of(h, struct acpm_info, handle)

/**
 * acpm_ts_bump() - advance the firmware's timestamp index.
 * @acpm:	driver data.
 *
 * Stock timestamp_write() (S:drivers/soc/samsung/acpm/acpm_ipc.c:53-70): the
 * firmware stamps each log entry with this index, the kernel keeps the
 * matching sched_clock(). Skipped when another CPU is updating it, as stock.
 */
static void acpm_ts_bump(struct acpm_info *acpm)
{
	u32 i;

	if (!acpm->fwlog_ok || !spin_trylock(&acpm->ts_lock))
		return;

	i = readl(acpm->ts_idx) + 1;
	if (i >= acpm->match->num_timestamps)
		i = 0;
	acpm->ts[i] = sched_clock();
	writel(i, acpm->ts_idx);

	spin_unlock(&acpm->ts_lock);
}

/**
 * acpm_fwlog_drain() - drain the APM firmware-log ring.
 * @acpm:	driver data.
 *
 * Stock acpm_log_print() (S:drivers/soc/samsung/acpm/acpm_ipc.c:72-135):
 * entries from rear to front, rear advanced and written after each entry and
 * front re-read (:121-127), then the timestamp index (:130). Printing follows
 * the fw_log parameter (default none). At most one ring length per call; a
 * corrupt index stops the drain without writing.
 */
static void acpm_fwlog_drain(struct acpm_info *acpm)
{
	u32 front, rear, n, hdr, es = acpm->fwlog_es, el = acpm->fwlog_el;
	char str[64 - 8 + 1];
	void __iomem *e;
	u64 t;

	if (!acpm->fwlog_ok || !spin_trylock(&acpm->fwlog_lock))
		return;

	front = readl(acpm->fwlog_front);
	rear = readl(acpm->fwlog_rear);
	for (n = 0; rear != front && n < el; n++) {
		if (rear >= el || front >= el) {
			atomic_inc(&acpm->fwlog_bad);
			break;
		}
		e = acpm->fwlog_data + es * rear;
		hdr = readl(e);
		if (fw_log >= 2 || (fw_log == 1 && !(hdr & ACPM_LOG_LEVEL))) {
			memcpy_fromio(str, e + 4, es - 8);
			str[es - 8] = '\0';
			t = acpm->ts[FIELD_GET(ACPM_LOG_TIME_INDEX, hdr) %
				     acpm->match->num_timestamps] +
			    (u64)FIELD_GET(ACPM_LOG_COUNT, hdr) * ACPM_SYSTICK_NS;
			dev_info(acpm->dev, "[ACPM_FW][%llu] id:%lu, %s, %x\n", t,
				 FIELD_GET(ACPM_LOG_ID, hdr), str,
				 readl(e + es - 4));
		}
		rear = (rear + 1 == el) ? 0 : rear + 1;
		writel(rear, acpm->fwlog_rear);
		atomic_inc(&acpm->fwlog_entries);
		front = readl(acpm->fwlog_front);
	}

	spin_unlock(&acpm->fwlog_lock);

	acpm_ts_bump(acpm);
}

static void acpm_fwlog_work(struct work_struct *work)
{
	struct acpm_info *acpm = container_of(to_delayed_work(work),
					      struct acpm_info, fwlog_work);

	acpm_fwlog_drain(acpm);
	queue_delayed_work(system_freezable_wq, &acpm->fwlog_work,
			   msecs_to_jiffies(acpm->match->fwlog_period_ms));
}

static void acpm_fwlog_cancel(void *data)
{
	struct acpm_info *acpm = data;

	cancel_delayed_work_sync(&acpm->fwlog_work);
}

/**
 * acpm_fwlog_init() - locate and check the firmware-log ring (fwlog SoCs).
 * @acpm:	driver data.
 *
 * Stock log_buffer_init() (S:drivers/soc/samsung/acpm/acpm_ipc.c:580-636)
 * resolves the same six framework words (:596-604) and starts the periodic
 * drain on a freezable workqueue, first after 10 s (:746-752).
 */
static void acpm_fwlog_init(struct acpm_info *acpm)
{
	struct acpm_shmem __iomem *shmem = acpm->shmem;
	resource_size_t sz = acpm->sram_size;
	u32 ti, lr, lf, ld, es, el;

	spin_lock_init(&acpm->ts_lock);
	spin_lock_init(&acpm->fwlog_lock);
	INIT_DELAYED_WORK(&acpm->fwlog_work, acpm_fwlog_work);

	ti = readl(&shmem->ktime_index);
	lr = readl(&shmem->log_buf_rear);
	lf = readl(&shmem->log_buf_front);
	ld = readl(&shmem->log_data);
	es = readl(&shmem->log_entry_size);
	el = readl(&shmem->log_entry_len);

	if (ti > sz - 4 || lr > sz - 4 || lf > sz - 4 || ld >= sz ||
	    es < 8 || es > 64 || es % 4 || !el ||
	    (u64)es * el > sz - ld ||
	    acpm->match->num_timestamps > ACPM_MAX_TIMESTAMPS ||
	    !acpm->match->num_timestamps) {
		dev_err(acpm->dev, "acpm: fwlog layout invalid\n");
		return;
	}

	acpm->ts_idx = acpm->sram_base + ti;
	acpm->fwlog_rear = acpm->sram_base + lr;
	acpm->fwlog_front = acpm->sram_base + lf;
	acpm->fwlog_data = acpm->sram_base + ld;
	acpm->fwlog_es = es;
	acpm->fwlog_el = el;

	dev_info(acpm->dev, "acpm: fwlog ktime_index@0x%x ring front@0x%x rear@0x%x data@0x%x size=%u len=%u\n",
		 ti, lf, lr, ld, es, el);

	if (devm_add_action_or_reset(acpm->dev, acpm_fwlog_cancel, acpm))
		return;

	acpm->fwlog_ok = true;
	queue_delayed_work(system_freezable_wq, &acpm->fwlog_work,
			   msecs_to_jiffies(acpm->match->fwlog_first_ms));
}

/**
 * acpm_notify() - hand an unsolicited firmware message to the notifier chain.
 * @achan:	ACPM channel info.
 * @cmd:	the message, four 32-bit words.
 *
 * Called with the channel's rx_lock held, in process context.
 */
static void acpm_notify(struct acpm_chan *achan, u32 *cmd)
{
	struct acpm_info *acpm = achan->acpm;

	atomic_inc(&achan->stats.notify);
	if (achan->id == acpm->match->notify_chan)
		dev_info_ratelimited(acpm->dev, "acpm: ch%u notify MIF req %u kHz\n",
				     achan->id, cmd[1]);
	blocking_notifier_call_chain(&acpm->nh, achan->id, cmd);
}

/**
 * acpm_rx_entry() - classify one RX queue entry (irq_rx SoCs).
 * @achan:	ACPM channel info.
 * @addr:	address of the entry in the RX queue.
 * @xfer:	transfer being waited for, or NULL.
 * @tx_seqnum:	sequence number of @xfer.
 *
 * A pending response (its number is set in bitmap_seqnum) is copied to @xfer
 * or saved for its waiter; the answer to a fire-and-forget message (number
 * set in ff_seqnum) is counted and dropped; anything else is a notification
 * on the notification channel and a late entry elsewhere. Never an error.
 *
 * Return: true if the entry is the response to @xfer.
 */
static bool acpm_rx_entry(struct acpm_chan *achan, const void __iomem *addr,
			  const struct acpm_xfer *xfer, u32 tx_seqnum)
{
	u32 rx_seqnum = FIELD_GET(ACPM_PROTOCOL_SEQNUM, readl(addr));
	struct acpm_rx_data *rx_data;
	u32 cmd[4] = {};

	if (rx_seqnum && test_bit(rx_seqnum - 1, achan->bitmap_seqnum)) {
		rx_data = &achan->rx_data[rx_seqnum - 1];
		if (!rx_data->response) {
			clear_bit(rx_seqnum - 1, achan->bitmap_seqnum);
			return false;
		}
		if (xfer && rx_seqnum == tx_seqnum) {
			__ioread32_copy(xfer->rxd, addr, xfer->rxlen / 4);
			clear_bit(rx_seqnum - 1, achan->bitmap_seqnum);
			return true;
		}
		/* Save the whole message; its waiter copies its own rxlen. */
		__ioread32_copy(rx_data->cmd, addr, rx_data->n_cmd);
		return false;
	}

	if (rx_seqnum && test_and_clear_bit(rx_seqnum - 1, achan->ff_seqnum)) {
		atomic_inc(&achan->stats.ack);
		return false;
	}

	if (achan->id == achan->acpm->match->notify_chan) {
		__ioread32_copy(cmd, addr, min_t(u32, achan->mlen, sizeof(cmd)) / 4);
		acpm_notify(achan, cmd);
	} else {
		atomic_inc(&achan->stats.late);
	}

	return false;
}

/**
 * acpm_get_saved_rx() - get the response if it was already saved.
 * @achan:	ACPM channel info.
 * @xfer:	reference to the transfer to get response for.
 * @tx_seqnum:	xfer TX sequence number.
 */
static void acpm_get_saved_rx(struct acpm_chan *achan,
			      const struct acpm_xfer *xfer, u32 tx_seqnum)
{
	const struct acpm_rx_data *rx_data = &achan->rx_data[tx_seqnum - 1];
	u32 rx_seqnum;

	if (!rx_data->response)
		return;

	rx_seqnum = FIELD_GET(ACPM_PROTOCOL_SEQNUM, rx_data->cmd[0]);

	if (rx_seqnum == tx_seqnum) {
		memcpy(xfer->rxd, rx_data->cmd, xfer->rxlen);
		clear_bit(rx_seqnum - 1, achan->bitmap_seqnum);
	}
}

/**
 * acpm_get_rx() - get response from RX queue.
 * @achan:	ACPM channel info.
 * @xfer:	reference to the transfer to get response for.
 *
 * Return: 0 on success, -errno otherwise.
 */
static int acpm_get_rx(struct acpm_chan *achan, const struct acpm_xfer *xfer)
{
	u32 rx_front, rx_seqnum, tx_seqnum, seqnum;
	const void __iomem *base, *addr;
	struct acpm_rx_data *rx_data;
	u32 i, val, mlen;
	bool rx_set = false;

	guard(mutex)(&achan->rx_lock);

	rx_front = readl(achan->rx.front);
	i = readl(achan->rx.rear);

	tx_seqnum = FIELD_GET(ACPM_PROTOCOL_SEQNUM, xfer->txd[0]);

	if (i == rx_front) {
		acpm_get_saved_rx(achan, xfer, tx_seqnum);
		return 0;
	}

	base = achan->rx.base;
	mlen = achan->mlen;

	/* Drain RX queue. */
	do {
		addr = base + mlen * i;

		if (achan->acpm->match->irq_rx) {
			if (acpm_rx_entry(achan, addr, xfer, tx_seqnum))
				rx_set = true;
			i = (i + 1) % achan->qlen;
			continue;
		}

		/* Read RX seqnum. */
		val = readl(addr);

		rx_seqnum = FIELD_GET(ACPM_PROTOCOL_SEQNUM, val);
		if (!rx_seqnum)
			return -EIO;
		/*
		 * mssg seqnum starts with value 1, whereas the driver considers
		 * the first mssg at index 0.
		 */
		seqnum = rx_seqnum - 1;
		rx_data = &achan->rx_data[seqnum];

		if (rx_data->response) {
			if (rx_seqnum == tx_seqnum) {
				__ioread32_copy(xfer->rxd, addr,
						xfer->rxlen / 4);
				rx_set = true;
				clear_bit(seqnum, achan->bitmap_seqnum);
			} else {
				/*
				 * The RX data corresponds to another request.
				 * Save the data to drain the queue, but don't
				 * clear yet the bitmap. It will be cleared
				 * after the response is copied to the request.
				 */
				__ioread32_copy(rx_data->cmd, addr,
						xfer->rxlen / 4);
			}
		} else {
			clear_bit(seqnum, achan->bitmap_seqnum);
		}

		i = (i + 1) % achan->qlen;
	} while (i != rx_front);

	/* We saved all responses, mark RX empty. */
	writel(rx_front, achan->rx.rear);

	/*
	 * If the response was not in this iteration of the queue, check if the
	 * RX data was previously saved.
	 */
	if (!rx_set)
		acpm_get_saved_rx(achan, xfer, tx_seqnum);

	return 0;
}

/**
 * acpm_dequeue_by_polling() - RX dequeue by polling.
 * @achan:	ACPM channel info.
 * @xfer:	reference to the transfer being waited for.
 *
 * Return: 0 on success, -errno otherwise.
 */
static int acpm_dequeue_by_polling(struct acpm_chan *achan,
				   const struct acpm_xfer *xfer)
{
	struct device *dev = achan->acpm->dev;
	ktime_t timeout;
	u32 seqnum;
	int ret;

	seqnum = FIELD_GET(ACPM_PROTOCOL_SEQNUM, xfer->txd[0]);

	timeout = ktime_add_us(ktime_get(), achan->acpm->match->poll_timeout_us);
	do {
		ret = acpm_get_rx(achan, xfer);
		if (ret)
			return ret;

		if (!test_bit(seqnum - 1, achan->bitmap_seqnum))
			return 0;

		/* Determined experimentally. */
		udelay(20);
	} while (ktime_before(ktime_get(), timeout));

	dev_err(dev, "Timeout! ch:%u s:%u bitmap:%lx.\n",
		achan->id, seqnum, achan->bitmap_seqnum[0]);

	return -ETIME;
}

/**
 * acpm_wait_for_queue_slots() - wait for queue slots.
 *
 * @achan:		ACPM channel info.
 * @next_tx_front:	next front index of the TX queue.
 *
 * Return: 0 on success, -errno otherwise.
 */
static int acpm_wait_for_queue_slots(struct acpm_chan *achan, u32 next_tx_front)
{
	u32 val, ret;

	/*
	 * Wait for RX front to keep up with TX front. Make sure there's at
	 * least one element between them.
	 */
	if (achan->acpm->match->rx_front_check) {
		ret = readl_poll_timeout(achan->rx.front, val,
					 next_tx_front != val, 0,
					 ACPM_TX_TIMEOUT_US);
		if (ret) {
			dev_err(achan->acpm->dev, "RX front can not keep up with TX front.\n");
			return ret;
		}
	}

	ret = readl_poll_timeout(achan->tx.rear, val, next_tx_front != val, 0,
				 ACPM_TX_TIMEOUT_US);
	if (ret)
		dev_err(achan->acpm->dev, "TX queue is full.\n");

	return ret;
}

/**
 * acpm_xfer_is_ff() - is @xfer a fire-and-forget message?
 * @achan:	ACPM channel info.
 * @xfer:	the transfer.
 *
 * On irq_rx SoCs a message on an interrupt-mode channel that expects no data
 * is not waited for, as with stock (it only waits on polling channels,
 * S:drivers/soc/samsung/acpm/acpm_ipc.c:529).
 */
static bool acpm_xfer_is_ff(const struct acpm_chan *achan,
			    const struct acpm_xfer *xfer)
{
	return achan->acpm->match->irq_rx && !achan->poll_completion &&
	       !xfer->rxd;
}

/**
 * acpm_prepare_xfer() - prepare a transfer before writing the message to the
 * TX queue.
 * @achan:	ACPM channel info.
 * @xfer:	reference to the transfer being prepared.
 */
static void acpm_prepare_xfer(struct acpm_chan *achan,
			      const struct acpm_xfer *xfer)
{
	struct acpm_rx_data *rx_data;
	u32 *txd = (u32 *)xfer->txd;
	bool ff = acpm_xfer_is_ff(achan, xfer);

	/* Prevent chan->seqnum from being re-used */
	do {
		if (++achan->seqnum == ACPM_SEQNUM_MAX)
			achan->seqnum = 1;
	} while (test_bit(achan->seqnum - 1, achan->bitmap_seqnum));

	/*
	 * A fire-and-forget number is not waited for, so it never blocks the
	 * search above; reusing one whose answer never came is counted.
	 */
	if (test_and_clear_bit(achan->seqnum - 1, achan->ff_seqnum))
		atomic_inc(&achan->stats.noack);

	txd[0] |= FIELD_PREP(ACPM_PROTOCOL_SEQNUM, achan->seqnum);

	/* Clear data for upcoming responses */
	rx_data = &achan->rx_data[achan->seqnum - 1];
	memset(rx_data->cmd, 0, sizeof(*rx_data->cmd) * rx_data->n_cmd);
	if (xfer->rxd)
		rx_data->response = true;

	/* Flag the index based on seqnum. (seqnum: 1~63, bitmap: 0~62) */
	if (ff)
		set_bit(achan->seqnum - 1, achan->ff_seqnum);
	else
		set_bit(achan->seqnum - 1, achan->bitmap_seqnum);
}

/**
 * acpm_wait_for_message_response - an helper to group all possible ways of
 * waiting for a synchronous message response.
 *
 * @achan:	ACPM channel info.
 * @xfer:	reference to the transfer being waited for.
 *
 * Return: 0 on success, -errno otherwise.
 */
static int acpm_wait_for_message_response(struct acpm_chan *achan,
					  const struct acpm_xfer *xfer)
{
	/* Just polling mode supported for now. */
	return acpm_dequeue_by_polling(achan, xfer);
}

/**
 * acpm_cmd_timestamp() - value of the cmd[3] word of a PMIC/DVFS command.
 * @handle:	pointer to the acpm handle.
 *
 * Return: the current time in ms if the SoC's firmware takes a timestamp,
 * 0 otherwise.
 */
u32 acpm_cmd_timestamp(struct acpm_handle *handle)
{
	struct acpm_info *acpm = handle_to_acpm_info(handle);

	return acpm->match->cmd_timestamp ? ktime_to_ms(ktime_get()) : 0;
}

static int __acpm_do_xfer(struct acpm_info *acpm, struct acpm_chan *achan,
			  const struct acpm_xfer *xfer)
{
	struct exynos_mbox_msg msg;
	u32 idx, tx_front;
	int ret;

	if (!xfer->txd || xfer->txlen > achan->mlen || xfer->rxlen > achan->mlen)
		return -EINVAL;

	if (!achan->poll_completion && !acpm->match->irq_rx) {
		dev_err(achan->acpm->dev, "Interrupt mode not supported\n");
		return -EOPNOTSUPP;
	}

	msg.chan_id = xfer->acpm_chan_id;
	msg.chan_type = EXYNOS_MBOX_CHAN_TYPE_DOORBELL;

	scoped_guard(mutex, &achan->tx_lock) {
		tx_front = readl(achan->tx.front);
		idx = (tx_front + 1) % achan->qlen;

		ret = acpm_wait_for_queue_slots(achan, idx);
		if (ret)
			return ret;

		acpm_prepare_xfer(achan, xfer);

		/* Write TX command. */
		__iowrite32_copy(achan->tx.base + achan->mlen * tx_front,
				 xfer->txd, xfer->txlen / 4);

		/* Advance TX front. */
		writel(idx, achan->tx.front);

		/* stock: front, timestamp, doorbell (S:.../acpm_ipc.c:522-526) */
		acpm_ts_bump(acpm);

		ret = mbox_send_message(achan->chan, (void *)&msg);
		if (ret < 0)
			return ret;

		mbox_client_txdone(achan->chan, 0);
	}

	/* Fire-and-forget on an interrupt-mode channel (irq_rx): no wait. */
	if (acpm_xfer_is_ff(achan, xfer))
		return 0;

	ret = acpm_wait_for_message_response(achan, xfer);

	/* stock drains the firmware log after a polled response (S:.../acpm_ipc.c:574) */
	if (!ret && achan->poll_completion)
		acpm_fwlog_drain(acpm);

	return ret;
}

/**
 * acpm_do_xfer() - do one transfer.
 * @handle:	pointer to the acpm handle.
 * @xfer:	transfer to initiate and wait for response.
 *
 * Return: 0 on success, -errno otherwise.
 */
int acpm_do_xfer(struct acpm_handle *handle, const struct acpm_xfer *xfer)
{
	struct acpm_info *acpm = handle_to_acpm_info(handle);
	struct acpm_chan *achan;
	int ret;

	if (xfer->acpm_chan_id >= acpm->num_chans)
		return -EINVAL;

	achan = &acpm->chans[xfer->acpm_chan_id];

	ret = __acpm_do_xfer(acpm, achan, xfer);
	atomic_inc(&achan->stats.xfer);
	if (ret)
		atomic_inc(&achan->stats.err);

	return ret;
}

/**
 * acpm_chan_shmem_get_params() - get channel parameters and addresses of the
 * TX/RX queues.
 * @achan:	ACPM channel info.
 * @chan_shmem:	__iomem pointer to a channel described in shared memory.
 */
static void acpm_chan_shmem_get_params(struct acpm_chan *achan,
				struct acpm_chan_shmem __iomem *chan_shmem)
{
	void __iomem *base = achan->acpm->sram_base;
	struct acpm_queue *rx = &achan->rx;
	struct acpm_queue *tx = &achan->tx;

	achan->mlen = readl(&chan_shmem->mlen);
	achan->poll_completion = readl(&chan_shmem->poll_completion);
	achan->id = readl(&chan_shmem->id);
	achan->qlen = readl(&chan_shmem->qlen);
	achan->field = readl(&chan_shmem->field);
	achan->type = readl(&chan_shmem->type);

	tx->base = base + readl(&chan_shmem->rx_base);
	tx->rear = base + readl(&chan_shmem->rx_rear);
	tx->front = base + readl(&chan_shmem->rx_front);

	rx->base = base + readl(&chan_shmem->tx_base);
	rx->rear = base + readl(&chan_shmem->tx_rear);
	rx->front = base + readl(&chan_shmem->tx_front);

	dev_vdbg(achan->acpm->dev, "ID = %d poll = %d, mlen = %d, qlen = %d\n",
		 achan->id, achan->poll_completion, achan->mlen, achan->qlen);
}

/**
 * acpm_achan_alloc_cmds() - allocate buffers for retrieving data from the ACPM
 * firmware.
 * @achan:	ACPM channel info.
 *
 * Return: 0 on success, -errno otherwise.
 */
static int acpm_achan_alloc_cmds(struct acpm_chan *achan)
{
	struct device *dev = achan->acpm->dev;
	struct acpm_rx_data *rx_data;
	size_t cmd_size, n_cmd;
	int i;

	if (achan->mlen == 0)
		return 0;

	cmd_size = sizeof(*(achan->rx_data[0].cmd));
	n_cmd = DIV_ROUND_UP_ULL(achan->mlen, cmd_size);

	for (i = 0; i < ACPM_SEQNUM_MAX; i++) {
		rx_data = &achan->rx_data[i];
		rx_data->n_cmd = n_cmd;
		rx_data->cmd = devm_kcalloc(dev, n_cmd, cmd_size, GFP_KERNEL);
		if (!rx_data->cmd)
			return -ENOMEM;
	}

	return 0;
}

/**
 * acpm_free_mbox_chans() - free mailbox channels.
 * @acpm:	pointer to driver data.
 */
static void acpm_free_mbox_chans(struct acpm_info *acpm)
{
	int i;

	for (i = 0; i < acpm->num_chans; i++)
		if (!IS_ERR_OR_NULL(acpm->chans[i].chan)) {
			mbox_free_channel(acpm->chans[i].chan);
			cancel_work_sync(&acpm->chans[i].rx_work);
		}
}

/**
 * acpm_rx_work() - drain an interrupt-mode channel (irq_rx SoCs).
 * @work:	the channel's rx_work.
 *
 * Mirrors stock's dequeue_policy() (S:drivers/soc/samsung/acpm/acpm_ipc.c:305-350):
 * a TYPE_BUFFER channel is one message read in place, without indices
 * (:314-322); a queue is drained entry by entry, rear written after each
 * entry and front re-read (:325-346).
 */
static void acpm_rx_work(struct work_struct *work)
{
	struct acpm_chan *achan = container_of(work, struct acpm_chan, rx_work);
	u32 cmd[4] = {};
	u32 front, rear, n;

	guard(mutex)(&achan->rx_lock);

	if (achan->type == ACPM_CHAN_TYPE_BUFFER) {
		__ioread32_copy(cmd, achan->rx.base,
				min_t(u32, achan->mlen, sizeof(cmd)) / 4);
		acpm_notify(achan, cmd);
		return;
	}

	if (!achan->qlen)
		return;

	front = readl(achan->rx.front);
	rear = readl(achan->rx.rear);
	for (n = 0; rear != front && n < achan->qlen; n++) {
		if (front >= achan->qlen || rear >= achan->qlen)
			break;
		acpm_rx_entry(achan, achan->rx.base + achan->mlen * rear,
			      NULL, 0);
		rear = (rear + 1) % achan->qlen;
		writel(rear, achan->rx.rear);
		front = readl(achan->rx.front);
	}

	/* stock dequeue_policy() ends its queue path with acpm_log_print() (:348) */
	acpm_fwlog_drain(achan->acpm);
}

/* Mailbox RX callback (atomic, A:include/linux/mailbox_client.h:24). */
static void acpm_rx_callback(struct mbox_client *cl, void *mssg)
{
	queue_work(system_highpri_wq, &client_to_acpm_chan(cl)->rx_work);
}

/**
 * acpm_chan_init_irq_rx() - irq_rx part of the channel setup.
 * @achan:	ACPM channel info.
 * @i:		channel index.
 *
 * Logs the firmware's channel parameters and drops RX entries that were
 * queued before Linux (S-Boot talks to the PMIC and the DVFS plugin itself);
 * they answer no Linux request and would otherwise be counted as late.
 * Runs before the channel is requested, so before any TX and before its RX
 * interrupt is unmasked.
 */
static void acpm_chan_init_irq_rx(struct acpm_chan *achan, int i)
{
	u32 tx_front = readl(achan->tx.front), tx_rear = readl(achan->tx.rear);
	u32 rx_front = readl(achan->rx.front), rx_rear = readl(achan->rx.rear);
	u32 stale = 0;

	if (achan->type == ACPM_CHAN_TYPE_QUEUE && achan->qlen &&
	    rx_front < achan->qlen && rx_rear < achan->qlen)
		stale = (rx_front - rx_rear + achan->qlen) % achan->qlen;

	dev_info(achan->acpm->dev,
		 "acpm: ch%d id=%u field=0x%x poll=%u type=%u qlen=%u mlen=%u tx=%u/%u rx=%u/%u stale=%u\n",
		 i, achan->id, achan->field, achan->poll_completion, achan->type,
		 achan->qlen, achan->mlen, tx_front, tx_rear, rx_front, rx_rear,
		 stale);

	if (stale) {
		writel(rx_front, achan->rx.rear);
		atomic_set(&achan->stats.stale, stale);
	}
}

/**
 * acpm_channels_init() - initialize channels based on the configuration data in
 * the shared memory.
 * @acpm:	pointer to driver data.
 *
 * Return: 0 on success, -errno otherwise.
 */
static int acpm_channels_init(struct acpm_info *acpm)
{
	struct acpm_shmem __iomem *shmem = acpm->shmem;
	struct acpm_chan_shmem __iomem *chans_shmem;
	struct device *dev = acpm->dev;
	int i, ret;

	acpm->num_chans = readl(&shmem->num_chans);
	acpm->chans = devm_kcalloc(dev, acpm->num_chans, sizeof(*acpm->chans),
				   GFP_KERNEL);
	if (!acpm->chans)
		return -ENOMEM;

	chans_shmem = acpm->sram_base + readl(&shmem->chans);

	for (i = 0; i < acpm->num_chans; i++) {
		struct acpm_chan_shmem __iomem *chan_shmem = &chans_shmem[i];
		struct acpm_chan *achan = &acpm->chans[i];
		struct mbox_client *cl = &achan->cl;

		achan->acpm = acpm;

		acpm_chan_shmem_get_params(achan, chan_shmem);

		ret = acpm_achan_alloc_cmds(achan);
		if (ret)
			return ret;

		mutex_init(&achan->rx_lock);
		mutex_init(&achan->tx_lock);
		INIT_WORK(&achan->rx_work, acpm_rx_work);

		cl->dev = dev;

		if (acpm->match->irq_rx) {
			acpm_chan_init_irq_rx(achan, i);
			if (!achan->poll_completion) {
				if (achan->id != i)
					dev_err(dev, "acpm: ch%d id=%u != index: RX irq disabled for it\n",
						i, achan->id);
				else
					cl->rx_callback = acpm_rx_callback;
			}
		}

		achan->chan = mbox_request_channel(cl, 0);
		if (IS_ERR(achan->chan)) {
			acpm_free_mbox_chans(acpm);
			return PTR_ERR(achan->chan);
		}
	}

	return 0;
}

/**
 * acpm_plugin_cmd() - send one dynamic plugin command (attach or detach).
 * @acpm:	driver data.
 * @id:		plugin id.
 * @op:		ACPM_PROTOCOL_DP_A or ACPM_PROTOCOL_DP_D.
 *
 * Stock handle_dynamic_plugin() (S:drivers/soc/samsung/acpm/acpm.c:36-65):
 * cmd[0] = op | id << 26, response expected; stock checks that the id in the
 * answer's cmd[0] is the plugin's (:286-289, :303). On an interrupt-mode
 * channel stock does not wait (S:drivers/soc/samsung/acpm/acpm_ipc.c:529).
 *
 * Return: "ok", "fail" or "sent".
 */
static const char *acpm_plugin_cmd(struct acpm_info *acpm, u32 id, u32 op)
{
	struct acpm_chan *achan = &acpm->chans[acpm->match->plugin_chan];
	u32 cmd[4] = { op | FIELD_PREP(ACPM_PROTOCOL_ID, id), 0, 0, 0 };
	struct acpm_xfer xfer = {
		.txd = cmd,
		.txlen = sizeof(cmd),
		.acpm_chan_id = acpm->match->plugin_chan,
	};
	const char *what = op == ACPM_PROTOCOL_DP_A ? "DP_A" : "DP_D";
	int ret;

	if (achan->poll_completion) {
		xfer.rxd = cmd;
		xfer.rxlen = sizeof(cmd);
	}

	ret = acpm_do_xfer(&acpm->handle, &xfer);
	if (!ret && !achan->poll_completion) {
		dev_info(acpm->dev, "acpm: plugin %u %s sent (irq channel)\n", id, what);
		return "sent";
	}
	if (!ret && FIELD_GET(ACPM_PROTOCOL_ID, cmd[0]) != id)
		ret = -EBADMSG;
	if (ret) {
		dev_err(acpm->dev, "acpm: plugin %u %s failed (%d)\n", id, what, ret);
		return "fail";
	}

	dev_info(acpm->dev, "acpm: plugin %u %s ok\n", id, what);
	return "ok";
}

/**
 * acpm_fvmap_read() - read the DVFS plugin's frequency/voltage map (read-only).
 * @acpm:	driver data.
 * @off:	SRAM offset of the map: plugin base (bit 0 cleared) + plugin size
 *		(S:drivers/soc/samsung/acpm/acpm.c:117,151-152).
 *
 * Keeps, per DVFS domain, the level count and the address of the first PLL
 * member, which stock uses to relocate its PLL pointer
 * (S:drivers/soc/samsung/cal-if/fvmap.c:236-251), and logs the levels and
 * PLL addresses of the domains named in the match data. Stock copies the map
 * out of SRAM (:178-270) and writes it only for boot-argument voltage
 * margins (:225-227), which S-Boot does not pass; nothing is written here.
 */
static void acpm_fvmap_read(struct acpm_info *acpm, u32 off)
{
	void __iomem *map = acpm->sram_base + off;
	u32 avail = min_t(u32, ACPM_FVMAP_SIZE, acpm->sram_size - off);
	u32 i, j, lv, npll, o_rv, o_mem, poff, rate, volt, pa;
	const char *name;
	void __iomem *hdr;

	if (off >= acpm->sram_size ||
	    acpm->match->fvmap_domains > ACPM_FVMAP_MAX_DOMAINS ||
	    acpm->match->fvmap_domains * ACPM_FVMAP_HDR_SIZE > avail) {
		dev_err(acpm->dev, "acpm: fvmap at 0x%x out of SRAM\n", off);
		return;
	}

	acpm->fvmap_off = off;
	acpm->fvmap_domains = acpm->match->fvmap_domains;

	for (i = 0; i < acpm->fvmap_domains; i++) {
		name = acpm->match->fvmap_names ? acpm->match->fvmap_names[i] : NULL;
		hdr = map + ACPM_FVMAP_HDR_SIZE * i;
		lv = readb(hdr + ACPM_FVMAP_HDR_NUM_LV);
		npll = readb(hdr + ACPM_FVMAP_HDR_NUM_PLL);
		o_rv = readw(hdr + ACPM_FVMAP_HDR_O_RATEVOLT);
		o_mem = readw(hdr + ACPM_FVMAP_HDR_O_MEMBERS);
		acpm->fvmap_lv[i] = lv;

		if (name)
			dev_info(acpm->dev, "acpm: fvmap %s lv=%u npll=%u\n",
				 name, lv, npll);

		if (o_rv + 8 * lv <= avail) {
			for (j = 0; name && j < lv; j++) {
				rate = readl(map + o_rv + 8 * j);
				volt = readl(map + o_rv + 8 * j + 4);
				dev_info(acpm->dev, "acpm: fvmap %s L%u %u kHz %u uV\n",
					 name, j, rate, volt);
			}
		} else if (name) {
			dev_err(acpm->dev, "acpm: fvmap %s levels out of range\n", name);
		}

		if (o_mem + 2 * npll > avail) {
			if (name && npll)
				dev_err(acpm->dev, "acpm: fvmap %s members out of range\n", name);
			continue;
		}
		for (j = 0; j < npll; j++) {
			poff = readw(map + o_mem + 2 * j);
			if (poff + 4 > avail)
				continue;
			pa = readl(map + poff);
			if (!j)
				acpm->fvmap_pll_pa[i] = pa;
			if (name)
				dev_info(acpm->dev, "acpm: fvmap %s pll%u addr=0x%08x\n",
					 name, j, pa);
		}
	}
}

/**
 * acpm_get_fvmap_pll_pa() - PLL address the DVFS plugin uses for a domain.
 * @handle:	ACPM handle.
 * @idx:	DVFS domain index.
 * @pa:		where the physical address is stored.
 *
 * Return: 0 on success, -ENODATA if no DVFS plugin map was read or the domain
 * has no PLL member.
 */
int acpm_get_fvmap_pll_pa(struct acpm_handle *handle, unsigned int idx, u32 *pa)
{
	struct acpm_info *acpm = handle_to_acpm_info(handle);

	if (!acpm->fvmap_off || idx >= acpm->fvmap_domains ||
	    !acpm->fvmap_pll_pa[idx])
		return -ENODATA;

	*pa = acpm->fvmap_pll_pa[idx];
	return 0;
}
EXPORT_SYMBOL_GPL(acpm_get_fvmap_pll_pa);

/**
 * acpm_plugin_load() - copy a dynamic plugin's image into APM SRAM.
 * @acpm:	driver data.
 * @id:		plugin id.
 * @base:	the plugin's base_addr (its entry point; bit 0 is the Thumb bit).
 * @size:	the plugin's size.
 * @name:	its fw_name, read from SRAM.
 *
 * Stock plugins_init() copies the image named by fw_name to SRAM at
 * base_addr & ~1 (S:drivers/soc/samsung/acpm/acpm.c:111, :117, :143) with
 * firmware_update() -> request_firmware() + memcpy (:72-91, :67-70) before it
 * sends DP_A (:146). base_addr is the entry point the APM framework calls on
 * attach (S:drivers/soc/samsung/acpm/fw_header/common.h:95 "(entrypoint)",
 * :125 pfn_plugin_init). The image is not in the APM firmware that S-Boot
 * loads; stock builds it into the kernel (S:firmware/Makefile:109-115 with
 * CONFIG_FIRMWARE_IN_KERNEL=y, IK:1154), where request_firmware() finds it
 * first (S:drivers/base/firmware_class.c:1054; here
 * A:drivers/base/firmware_loader/main.c:824 and CONFIG_EXTRA_FIRMWARE).
 * Like stock, only SRAM bounds the copy; a copy longer than the plugin's size
 * is logged, since the fvmap starts at base + size (:151-152).
 *
 * Return: 0, or -errno and then no DP_A may be sent: the APM would run
 * whatever the region holds and stop answering every channel.
 */
static int acpm_plugin_load(struct acpm_info *acpm, u32 id, u32 base, u32 size,
			    const char *name)
{
	const struct firmware *fw;
	u32 off = base & ~1U;
	int ret;

	ret = firmware_request_nowarn(&fw, name, acpm->dev);
	if (ret) {
		dev_err(acpm->dev, "acpm: plugin %u image %s not found (%d): DP_A not sent\n",
			id, name, ret);
		return ret;
	}

	if (!fw->size || fw->size % 4 || off >= acpm->sram_size ||
	    fw->size > acpm->sram_size - off) {
		dev_err(acpm->dev, "acpm: plugin %u image %s (%zu bytes) does not fit SRAM at 0x%x: DP_A not sent\n",
			id, name, fw->size, off);
		ret = -EFBIG;
		goto out;
	}
	if (fw->size > size)
		dev_err(acpm->dev, "acpm: plugin %u image %s (%zu bytes) exceeds its size 0x%x\n",
			id, name, fw->size, size);

	__iowrite32_copy(acpm->sram_base + off, fw->data, fw->size / 4);
	dev_info(acpm->dev, "acpm: plugin %u image %s loaded: %zu bytes at 0x%x\n",
		 id, name, fw->size, off);
out:
	release_firmware(fw);
	return ret;
}

/**
 * acpm_plugins_init() - attach/detach the firmware plugins as stock does.
 * @acpm:	driver data.
 *
 * Stock plugins_init() (S:drivers/soc/samsung/acpm/acpm.c:93-157), run before
 * any DVFS request (stock: fs_initcall_sync, :380-390; here: before the
 * clocks device is registered). In table order: a plugin with is_attached 0
 * and stay_attached 1 gets DP_A (a missing fw_name or base_addr ends the loop,
 * :112-121) after its image is copied into SRAM (acpm_plugin_load(); without
 * the image no DP_A is sent and a "dvfs" plugin reports "nofw"), one with 1/0
 * gets DP_D (:146). The plugin whose fw_name contains "dvfs" has its fvmap
 * read (:151-152).
 *
 * Return: 0, or -EINVAL if the plugin table is not inside SRAM (then the
 * caller registers no DVFS clocks).
 */
static int acpm_plugins_init(struct acpm_info *acpm)
{
	struct acpm_shmem __iomem *shmem = acpm->shmem;
	struct acpm_plugin_shmem __iomem *pl;
	char name[ACPM_PLUGIN_NAME_LEN];
	u32 off, n, i, j, id, base, size, fw, np;
	u8 att, stay;

	acpm->dvfs_plugin_id = -1;
	acpm->dvfs_dpa = "none";
	acpm->dpd_plugin_id = -1;
	acpm->dpd_res = "none";

	off = readl(&shmem->reserved[0]);	/* framework word 0: plugins */
	n = readl(&shmem->reserved[1]);		/* word 1: num_plugins */
	dev_info(acpm->dev, "initdata@0x%llx chans=%u plugins=%u\n",
		 (unsigned long long)acpm->match->initdata_base, acpm->num_chans, n);

	if (n > ACPM_MAX_PLUGINS || off >= acpm->sram_size ||
	    n * sizeof(*pl) > acpm->sram_size - off) {
		dev_err(acpm->dev, "acpm: plugin table 0x%x x %u out of SRAM\n", off, n);
		return -EINVAL;
	}
	acpm->nplugins = n;

	for (i = 0; i < n; i++) {
		pl = acpm->sram_base + off + i * sizeof(*pl);
		id = readl(&pl->id);
		base = readl(&pl->base_addr);
		size = readl(&pl->size);
		att = readb(&pl->is_attached);
		stay = readb(&pl->stay_attached);
		fw = readl(&pl->fw_name);

		name[0] = '\0';
		if (fw && fw < acpm->sram_size) {
			np = min_t(u32, sizeof(name) - 1, acpm->sram_size - fw);
			for (j = 0; j < np; j++) {
				name[j] = readb(acpm->sram_base + fw + j);
				if (!name[j])
					break;
			}
			name[j < np ? j : np] = '\0';
		}

		dev_info(acpm->dev, "acpm: plugin[%u] id=%u base=0x%x size=0x%x attached=%u stay=%u fw=%s\n",
			 i, id, base, size, att, stay, name);

		if (strstr(name, "dvfs")) {
			acpm->dvfs_plugin_id = id;
			acpm->dvfs_attached = att;
			acpm->dvfs_stay = stay;
		}

		if (!att && stay) {
			if (!fw || !base) {
				dev_err(acpm->dev, "acpm: plugin %u DP_A skipped: fw_name/base_addr missing\n",
					id);
				if (strstr(name, "dvfs"))
					acpm->dvfs_dpa = "skip";
				break;
			}
			if (acpm_plugin_load(acpm, id, base, size, name)) {
				if (strstr(name, "dvfs"))
					acpm->dvfs_dpa = "nofw";
			} else if (strstr(name, "dvfs")) {
				acpm->dvfs_dpa = acpm_plugin_cmd(acpm, id, ACPM_PROTOCOL_DP_A);
			} else {
				acpm_plugin_cmd(acpm, id, ACPM_PROTOCOL_DP_A);
			}
		} else if (att && !stay) {
			acpm->dpd_plugin_id = id;
			acpm->dpd_res = acpm_plugin_cmd(acpm, id, ACPM_PROTOCOL_DP_D);
		}

		if (strstr(name, "dvfs") && base)
			acpm_fvmap_read(acpm, (base & ~1U) + size);
	}

	return 0;
}

/**
 * acpm_chan_stats_show() - debugfs acpm/chan_stats.
 * @sf:		seq_file; sf->private is the ACPM device.
 * @unused:	unused.
 *
 * Return: 0.
 */
static int acpm_chan_stats_show(struct seq_file *sf, void *unused)
{
	struct acpm_info *acpm = dev_get_drvdata(sf->private);
	int i;

	seq_puts(sf, "ch id field poll type qlen mlen xfer err notify late ack noack stale\n");
	for (i = 0; i < acpm->num_chans; i++) {
		struct acpm_chan *c = &acpm->chans[i];

		seq_printf(sf, "%d %u 0x%x %u %u %u %u %d %d %d %d %d %d %d\n",
			   i, c->id, c->field, c->poll_completion, c->type,
			   c->qlen, c->mlen, atomic_read(&c->stats.xfer),
			   atomic_read(&c->stats.err),
			   atomic_read(&c->stats.notify),
			   atomic_read(&c->stats.late),
			   atomic_read(&c->stats.ack),
			   atomic_read(&c->stats.noack),
			   atomic_read(&c->stats.stale));
	}

	return 0;
}

static void acpm_debugfs_remove(void *data)
{
	debugfs_remove_recursive(data);
}

static void acpm_debugfs_init(struct acpm_info *acpm)
{
	struct dentry *dir = debugfs_create_dir("acpm", NULL);

	if (IS_ERR(dir))
		return;
	if (devm_add_action_or_reset(acpm->dev, acpm_debugfs_remove, dir))
		return;
	debugfs_create_devm_seqfile(acpm->dev, "chan_stats", dir,
				    acpm_chan_stats_show);
}

/**
 * acpm_setup_ops() - setup the operations structures.
 * @acpm:	pointer to the driver data.
 */
static void acpm_setup_ops(struct acpm_info *acpm)
{
	struct acpm_dvfs_ops *dvfs_ops = &acpm->handle.ops.dvfs_ops;
	struct acpm_pmic_ops *pmic_ops = &acpm->handle.ops.pmic_ops;

	dvfs_ops->set_rate = acpm_dvfs_set_rate;
	dvfs_ops->get_rate = acpm_dvfs_get_rate;
	dvfs_ops->set_cold_temp = acpm_dvfs_set_cold_temp;
	dvfs_ops->sleep_release = acpm_dvfs_sleep_release;

	pmic_ops->read_reg = acpm_pmic_read_reg;
	pmic_ops->bulk_read = acpm_pmic_bulk_read;
	pmic_ops->write_reg = acpm_pmic_write_reg;
	pmic_ops->bulk_write = acpm_pmic_bulk_write;
	pmic_ops->update_reg = acpm_pmic_update_reg;
}

static void acpm_clk_pdev_unregister(void *data)
{
	platform_device_unregister(data);
}

static int acpm_probe(struct platform_device *pdev)
{
	const struct acpm_match_data *match_data;
	struct platform_device *acpm_clk_pdev;
	struct device *dev = &pdev->dev;
	struct device_node *shmem;
	struct acpm_info *acpm;
	resource_size_t size;
	struct resource res;
	int ret;

	acpm = devm_kzalloc(dev, sizeof(*acpm), GFP_KERNEL);
	if (!acpm)
		return -ENOMEM;

	shmem = of_parse_phandle(dev->of_node, "shmem", 0);
	ret = of_address_to_resource(shmem, 0, &res);
	of_node_put(shmem);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to get shared memory.\n");

	size = resource_size(&res);
	acpm->sram_size = size;
	acpm->sram_base = devm_ioremap(dev, res.start, size);
	if (!acpm->sram_base)
		return dev_err_probe(dev, -ENOMEM,
				     "Failed to ioremap shared memory.\n");

	match_data = of_device_get_match_data(dev);
	if (!match_data)
		return dev_err_probe(dev, -EINVAL,
				     "Failed to get match data.\n");

	acpm->shmem = acpm->sram_base + match_data->initdata_base;
	acpm->dev = dev;
	acpm->match = match_data;
	BLOCKING_INIT_NOTIFIER_HEAD(&acpm->nh);

	ret = acpm_channels_init(acpm);
	if (ret)
		return ret;

	acpm_setup_ops(acpm);

	platform_set_drvdata(pdev, acpm);

	acpm_debugfs_init(acpm);

	if (match_data->fwlog_period_ms)
		acpm_fwlog_init(acpm);

	/* Plugins first: no DVFS request may precede the DVFS plugin attach. */
	if (match_data->plugin_chan >= 0 &&
	    (match_data->plugin_chan >= acpm->num_chans ||
	     acpm_plugins_init(acpm)))
		return devm_of_platform_populate(dev);

	acpm_clk_pdev = platform_device_register_data(dev,
						match_data->acpm_clk_dev_name,
						PLATFORM_DEVID_NONE, NULL, 0);
	if (IS_ERR(acpm_clk_pdev))
		return dev_err_probe(dev, PTR_ERR(acpm_clk_pdev),
				     "Failed to register ACPM clocks device.\n");

	ret = devm_add_action_or_reset(dev, acpm_clk_pdev_unregister,
				       acpm_clk_pdev);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to add devm action.\n");

	return devm_of_platform_populate(dev);
}

/**
 * acpm_handle_put() - release the handle acquired by acpm_get_by_phandle.
 * @handle:	Handle acquired by acpm_get_by_phandle.
 */
static void acpm_handle_put(struct acpm_handle *handle)
{
	struct acpm_info *acpm = handle_to_acpm_info(handle);
	struct device *dev = acpm->dev;

	module_put(dev->driver->owner);
	/* Drop reference taken with of_find_device_by_node(). */
	put_device(dev);
}

/**
 * devm_acpm_release() - devres release method.
 * @dev: pointer to device.
 * @res: pointer to resource.
 */
static void devm_acpm_release(struct device *dev, void *res)
{
	acpm_handle_put(*(struct acpm_handle **)res);
}

/**
 * acpm_get_by_node() - get the ACPM handle using node pointer.
 * @dev:	device pointer requesting ACPM handle.
 * @np:		ACPM device tree node.
 *
 * Return: pointer to handle on success, ERR_PTR(-errno) otherwise.
 *
 * Note: handle CANNOT be pointer to const
 */
static struct acpm_handle *acpm_get_by_node(struct device *dev,
					    struct device_node *np)
{
	struct platform_device *pdev;
	struct device_link *link;
	struct acpm_info *acpm;

	pdev = of_find_device_by_node(np);
	if (!pdev)
		return ERR_PTR(-EPROBE_DEFER);

	acpm = platform_get_drvdata(pdev);
	if (!acpm) {
		platform_device_put(pdev);
		return ERR_PTR(-EPROBE_DEFER);
	}

	if (!try_module_get(pdev->dev.driver->owner)) {
		platform_device_put(pdev);
		return ERR_PTR(-EPROBE_DEFER);
	}

	link = device_link_add(dev, &pdev->dev, DL_FLAG_AUTOREMOVE_SUPPLIER);
	if (!link) {
		dev_err(&pdev->dev,
			"Failed to create device link to consumer %s.\n",
			dev_name(dev));
		platform_device_put(pdev);
		module_put(pdev->dev.driver->owner);
		return ERR_PTR(-EINVAL);
	}

	return &acpm->handle;
}

/**
 * devm_acpm_get_by_node() - managed get handle using node pointer.
 * @dev: device pointer requesting ACPM handle.
 * @np:  ACPM device tree node.
 *
 * Return: pointer to handle on success, ERR_PTR(-errno) otherwise.
 */
struct acpm_handle *devm_acpm_get_by_node(struct device *dev,
					  struct device_node *np)
{
	struct acpm_handle **ptr, *handle;

	ptr = devres_alloc(devm_acpm_release, sizeof(*ptr), GFP_KERNEL);
	if (!ptr)
		return ERR_PTR(-ENOMEM);

	handle = acpm_get_by_node(dev, np);
	if (!IS_ERR(handle)) {
		*ptr = handle;
		devres_add(dev, ptr);
	} else {
		devres_free(ptr);
	}

	return handle;
}
EXPORT_SYMBOL_GPL(devm_acpm_get_by_node);

struct acpm_nb_devres {
	struct acpm_info *acpm;
	struct notifier_block *nb;
};

static void devm_acpm_notifier_release(struct device *dev, void *res)
{
	struct acpm_nb_devres *dr = res;

	blocking_notifier_chain_unregister(&dr->acpm->nh, dr->nb);
}

/**
 * devm_acpm_register_notifier() - managed registration for unsolicited
 * firmware messages.
 * @dev:	device that owns the registration.
 * @handle:	ACPM handle.
 * @nb:		notifier; called in process context with the channel id as
 *		action and a pointer to the four message words as data.
 *
 * Return: 0 on success, -errno otherwise.
 */
int devm_acpm_register_notifier(struct device *dev, struct acpm_handle *handle,
				struct notifier_block *nb)
{
	struct acpm_info *acpm = handle_to_acpm_info(handle);
	struct acpm_nb_devres *dr;
	int ret;

	dr = devres_alloc(devm_acpm_notifier_release, sizeof(*dr), GFP_KERNEL);
	if (!dr)
		return -ENOMEM;

	ret = blocking_notifier_chain_register(&acpm->nh, nb);
	if (ret) {
		devres_free(dr);
		return ret;
	}

	dr->acpm = acpm;
	dr->nb = nb;
	devres_add(dev, dr);

	return 0;
}
EXPORT_SYMBOL_GPL(devm_acpm_register_notifier);

static const struct acpm_match_data acpm_gs101 = {
	.initdata_base = ACPM_GS101_INITDATA_BASE,
	.acpm_clk_dev_name = "gs101-acpm-clk",
	.rx_front_check = true,
	.cmd_timestamp = true,
	.poll_timeout_us = ACPM_POLL_TIMEOUT_US,
	.irq_rx = false,
	.notify_chan = -1,
	.plugin_chan = -1,
};

/* fvmap domains logged at probe: index = stock GET_IDX(ACPM_DVFS_*) (L2 pass line P6) */
static const char * const exynos8895_fvmap_names[] = {
	[2] = "cpucl0",		/* S:include/dt-bindings/clock/exynos8895.h:535 ACPM_DVFS_CPUCL0 (0x0B040002) */
	[3] = "cpucl1",		/* :536 ACPM_DVFS_CPUCL1 (0x0B040003) */
	[9] = NULL,
};

static const struct acpm_match_data acpm_exynos8895 = {
	/* S:arch/arm64/boot/dts/exynos/exynos8895.dtsi:2422 "initdata-base = <0x2850>;" */
	.initdata_base = 0x2850,
	.acpm_clk_dev_name = "exynos8895-acpm-clk",
	/* S:drivers/soc/samsung/acpm/acpm_ipc.c:480-496: TX rear check only */
	.rx_front_check = false,
	/* S:drivers/soc/samsung/acpm/acpm_mfd.c:23, S:drivers/soc/samsung/cal-if/acpm_dvfs.c:30 */
	.cmd_timestamp = false,
	/* S:drivers/soc/samsung/acpm/acpm_ipc.h:94 IPC_TIMEOUT 150 ms, acpm_ipc.c:538 5 retries */
	.poll_timeout_us = 6 * 150 * USEC_PER_MSEC,
	/* S:drivers/soc/samsung/acpm/acpm_ipc.c:352-383 */
	.irq_rx = true,
	/* S:arch/arm64/boot/dts/exynos/exynos8895.dtsi:2432 acpm_dvfs "acpm-ipc-channel = <5>;" */
	.notify_chan = 5,
	/* S:arch/arm64/boot/dts/exynos/exynos8895.dtsi:2412 acpm "acpm-ipc-channel = <4>;" */
	.plugin_chan = 4,
	.fvmap_names = exynos8895_fvmap_names,
	/* ten ACPM DVFS domains, S:drivers/soc/samsung/cal-if/exynos8895/acpm_dvfs_exynos8895.h:1-12 */
	.fvmap_domains = 10,
	/* S:arch/arm64/boot/dts/exynos/exynos8895.dtsi:2425 "logging-period = <500>;" */
	.fwlog_period_ms = 500,
	/* S:drivers/soc/samsung/acpm/acpm_ipc.c:750-751 "msecs_to_jiffies(10000)" */
	.fwlog_first_ms = 10000,
	/* S:arch/arm64/boot/dts/exynos/exynos8895.dtsi:2423 "num-timestamps = <32>;" */
	.num_timestamps = 32,
};

static const struct of_device_id acpm_match[] = {
	{
		.compatible = "google,gs101-acpm-ipc",
		.data = &acpm_gs101,
	},
	{
		.compatible = "samsung,exynos8895-acpm-ipc",
		.data = &acpm_exynos8895,
	},
	{},
};
MODULE_DEVICE_TABLE(of, acpm_match);

static struct platform_driver acpm_driver = {
	.probe	= acpm_probe,
	.driver	= {
		.name = "exynos-acpm-protocol",
		.of_match_table	= acpm_match,
	},
};
module_platform_driver(acpm_driver);

MODULE_AUTHOR("Tudor Ambarus <tudor.ambarus@linaro.org>");
MODULE_DESCRIPTION("Samsung Exynos ACPM mailbox protocol driver");
MODULE_LICENSE("GPL");
