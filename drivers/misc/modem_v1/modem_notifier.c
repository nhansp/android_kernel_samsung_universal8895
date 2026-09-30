/*
 * Copyright (C) 2015 Samsung Electronics.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/stdarg.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/modem_notifier.h>

#include "modem_prj.h"
#include "modem_utils.h"

static ATOMIC_NOTIFIER_HEAD(modem_event_notifier);

int register_modem_event_notifier(struct notifier_block *nb)
{
	if (!nb)
		return -ENOENT;

	return atomic_notifier_chain_register(&modem_event_notifier, nb);
}
/*
 * L6C 2.6: exported for the L6E ABOX VSS path. Callbacks run in atomic context (hard IRQ,
 * softirq with IRQs off, or process context) and must not sleep. Stock used a raw chain and
 * never unregistered (S:drivers/misc/modem_v1/modem_notifier.c); the atomic chain calls under
 * RCU from any context and makes unregister safe against a running call (synchronize_rcu),
 * which the L6E ABOX remove() needs.
 */
EXPORT_SYMBOL_GPL(register_modem_event_notifier);

int unregister_modem_event_notifier(struct notifier_block *nb)
{
	if (!nb)
		return -ENOENT;

	return atomic_notifier_chain_unregister(&modem_event_notifier, nb);
}
EXPORT_SYMBOL_GPL(unregister_modem_event_notifier);

void modem_notify_event(enum modem_event evt)
{
	mif_err("event notify (%d) ++\n", evt);
	atomic_notifier_call_chain(&modem_event_notifier, evt, NULL);
	mif_err("event notify (%d) --\n", evt);
}
