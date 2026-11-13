// SPDX-License-Identifier: GPL-2.0
/*
 * Broadcom BCM43xx Bluetooth Low Power Mode (LPM) and power control driver
 * for Samsung Exynos8895 (greatlte).
 *
 * Stock-equivalent rewrite of S:drivers/bluetooth/bcm43xx.c (spec L6D 5.7).
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeup.h>
#include <linux/rfkill.h>
#include <linux/serial_core.h>
#include <linux/serial_s3c.h>
#include <linux/slab.h>

#define BT_POWER_ON_SETTLE_MS	100		/* S:drivers/bluetooth/bcm43xx.c:101 */
#define BT_ENTER_LPM_DELAY	ktime_set(1, 0)	/* 1 s, S:drivers/bluetooth/bcm43xx.c:242 */
#define BT_BTWAKE_RELAX_MS	500		/* wake_lock_timeout(HZ/2), S:drivers/bluetooth/bcm43xx.c:162 */
#define BT_HOSTWAKE_RELAX_MS	1000		/* wake_lock_timeout(HZ), S:drivers/bluetooth/bcm43xx.c:197 */

/*
 * 1: low power mode as stock, BT_WAKE follows UART transmit and drops 1 s after the last byte.
 * 0: BT_WAKE stays high while powered, so the controller never sleeps and its UART keeps
 * waking the SoC out of system sleep.
 */
static int bt_lpm_mode = 1;
module_param_named(lpm, bt_lpm_mode, int, 0644);

struct bt_lpm {
	struct device *dev;
	struct gpio_desc *en;
	struct gpio_desc *wake;
	struct gpio_desc *hw;
	int irq;
	struct rfkill *rfk;
	struct hrtimer timer;
	struct wakeup_source *ws_bt;
	struct wakeup_source *ws_host;
	bool host_wake;
	bool dev_wake;
	bool powered;
};

static struct bt_lpm *bt_lpm_g;			/* single instance (stock used file-scope globals, S:drivers/bluetooth/bcm43xx.c:52-78); wake_peer gets
						 * only the uart_port, so it reaches the instance through this pointer */

static int bt_set_block(void *data, bool blocked)	/* rfkill_ops.set_block, A:include/linux/rfkill.h; stock S:drivers/bluetooth/bcm43xx.c:82-117 */
{
	struct bt_lpm *bl = data;

	if (!blocked) {
		if (bl->powered)
			return 0;	/* (run 6) idempotent: sysfs `state` writes call set_block even when the state is
					 * unchanged (A:net/rfkill/core.c:360); stock would double irq_set_irq_wake */
		enable_irq_wake(bl->irq);	/* stock irq_set_irq_wake(irq, 1) S:drivers/bluetooth/bcm43xx.c:89 */
		if (!bt_lpm_mode)
			gpiod_set_value(bl->wake, 1);
		gpiod_set_value(bl->en, 1);
		msleep(BT_POWER_ON_SETTLE_MS);	/* S:drivers/bluetooth/bcm43xx.c:98-101 */
		WRITE_ONCE(bl->powered, true);
		dev_info(bl->dev, "power on\n");	/* L6D 8.3 S8 */
	} else {
		if (!bl->powered) {
			gpiod_set_value(bl->en, 0);
			return 0;	/* idempotent like the on-branch */
		}
		WRITE_ONCE(bl->powered, false);	/* (final) FIRST, so bt_wake_peer stops arming the timer */
		disable_irq_wake(bl->irq);					/* S:drivers/bluetooth/bcm43xx.c:105-110 */
		gpiod_set_value(bl->en, 0);					/* S:drivers/bluetooth/bcm43xx.c:114 */
		hrtimer_cancel(&bl->timer);
		gpiod_set_value(bl->wake, 0);
		bl->dev_wake = false;
		/* (final, R3-07) stock never cancelled the timer here (S:drivers/bluetooth/bcm43xx.c:103-115), so enter_lpm (:154-165) still turned the
		 * BT_bt_wake lock into a HZ/2 timeout. Cancelling it without a relax leaked ws_bt (suspend blocked) whenever
		 * BT was switched off < 1 s after the last TX - the normal shutdown order (HCI traffic, then rfkill state=0). */
		__pm_relax(bl->ws_bt);
		bl->host_wake = false;
		__pm_relax(bl->ws_host);	/* HOST_WAKE is undefined once BT_EN is low; the host-wake IRQ stays
						 * requested while off, as stock, so a later edge is handled by
						 * the thread from the re-read level */
		dev_info(bl->dev, "power off\n");	/* L6D 8.3 S8 */
	}
	return 0;
}

static const struct rfkill_ops bt_rfkill_ops = {
	.set_block = bt_set_block,
};

static void bt_wake_peer(struct uart_port *port)	/* atomic, under the port lock; bcm_bt_lpm_exit_lpm_locked S:drivers/bluetooth/bcm43xx.c:167-179 */
{
	struct bt_lpm *bl = READ_ONCE(bt_lpm_g);

	if (!bl || !bt_lpm_mode || !READ_ONCE(bl->powered))
		return;
	/* (final) a caller that passed this test just before bt_set_block(blocked) cleared `powered` can still re-arm the
	 * timer after the off path's hrtimer_cancel/__pm_relax; the timer then fires 1 s later and bt_enter_lpm turns ws_bt
	 * into a 500 ms timeout and drives BT_WAKE low, so that race self-heals (no lock with the rfkill path needed) */
	hrtimer_try_to_cancel(&bl->timer);
	__pm_stay_awake(bl->ws_bt);
	gpiod_set_value(bl->wake, 1);
	bl->dev_wake = true;
	hrtimer_start(&bl->timer, BT_ENTER_LPM_DELAY, HRTIMER_MODE_REL);
}

static enum hrtimer_restart bt_enter_lpm(struct hrtimer *t)	/* enter_lpm S:drivers/bluetooth/bcm43xx.c:154-165 */
{
	struct bt_lpm *bl = container_of(t, struct bt_lpm, timer);

	gpiod_set_value(bl->wake, 0);
	bl->dev_wake = false;
	__pm_wakeup_event(bl->ws_bt, BT_BTWAKE_RELAX_MS);
	return HRTIMER_NORESTART;
}

static irqreturn_t bt_host_wake_thread(int irq, void *data)	/* host_wake_isr + update_host_wake_locked S:drivers/bluetooth/bcm43xx.c:181-234 */
{
	struct bt_lpm *bl = data;
	bool v = gpiod_get_value(bl->hw);	/* level read as GPIO: the Exynos EINT irqchip has no irq_get_irqchip_state (RL-11) */

	if (v == bl->host_wake)
		return IRQ_HANDLED;
	bl->host_wake = v;
	if (v)
		__pm_stay_awake(bl->ws_host);
	else
		__pm_wakeup_event(bl->ws_host, BT_HOSTWAKE_RELAX_MS);
	return IRQ_HANDLED;
}

static void bt_lpm_ws_unregister(void *ws)
{
	wakeup_source_unregister(ws);
}

static int bcm43xx_bt_lpm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *un;
	struct platform_device *uart;
	struct bt_lpm *bl;
	int ret;

	bl = devm_kzalloc(dev, sizeof(*bl), GFP_KERNEL);
	if (!bl)
		return -ENOMEM;
	bl->dev = dev;

	un = of_parse_phandle(np, "samsung,uart", 0);
	uart = un ? of_find_device_by_node(un) : NULL;
	of_node_put(un);
	if (!uart)
		return -EPROBE_DEFER;	/* (final, R3-15a) no platform device yet for serial_1 (or it is disabled):
					 * defer instead of dereferencing NULL; 8.4 row "BT driver deferred forever" */

	ret = s3c24xx_serial_set_wake_peer(&uart->dev, bt_wake_peer);	/* S:drivers/bluetooth/bcm43xx.c:252; FIRST, so -EPROBE_DEFER
									 * (samsung_tty not bound yet) returns before anything
									 * non-devm exists; the hook is inert until bt_lpm_g is set at the end */
	if (ret) {
		put_device(&uart->dev);
		return ret;
	}

	bl->en = devm_gpiod_get(dev, "bt-en", GPIOD_OUT_LOW);		/* FDT:11977, S:drivers/bluetooth/bcm43xx.c:274-286, 321 */
	if (IS_ERR(bl->en)) {
		ret = PTR_ERR(bl->en);
		goto err_peer;
	}

	bl->wake = devm_gpiod_get(dev, "bt-wake", GPIOD_OUT_LOW);	/* FDT:11977, S:drivers/bluetooth/bcm43xx.c:288-301, 320 */
	if (IS_ERR(bl->wake)) {
		ret = PTR_ERR(bl->wake);
		goto err_peer;
	}

	bl->hw = devm_gpiod_get(dev, "bt-host-wake", GPIOD_IN);		/* FDT:11977, S:drivers/bluetooth/bcm43xx.c:303-317, 319; FIRST
									 * (sets CON = input), then the IRQ request below switches CON
									 * to EINT (exynos_irq_request_resources :225-233); never call
									 * gpiod_direction_input(hw) afterwards */
	if (IS_ERR(bl->hw)) {
		ret = PTR_ERR(bl->hw);
		goto err_peer;
	}

	hrtimer_setup(&bl->timer, bt_enter_lpm, CLOCK_MONOTONIC, HRTIMER_MODE_REL);	/* A:include/linux/hrtimer.h:231, S:drivers/bluetooth/bcm43xx.c:240-243 */

	/*
	 * The wakeup sources are released by devm actions added before the IRQ is requested, so devm frees
	 * the host-wake IRQ (whose thread uses ws_host) before the sources on every failure path.
	 */
	bl->ws_host = wakeup_source_register(dev, "BT_host_wake");	/* S:drivers/bluetooth/bcm43xx.c:247-248 */
	if (!bl->ws_host) {
		ret = -ENOMEM;
		goto err_peer;
	}
	ret = devm_add_action_or_reset(dev, bt_lpm_ws_unregister, bl->ws_host);
	if (ret)
		goto err_peer;

	bl->ws_bt = wakeup_source_register(dev, "BT_bt_wake");		/* S:drivers/bluetooth/bcm43xx.c:249-250 */
	if (!bl->ws_bt) {
		ret = -ENOMEM;
		goto err_peer;
	}
	ret = devm_add_action_or_reset(dev, bt_lpm_ws_unregister, bl->ws_bt);
	if (ret)
		goto err_peer;

	bl->irq = platform_get_irq(pdev, 0);
	if (bl->irq < 0) {
		ret = bl->irq;
		goto err_peer;
	}

	ret = devm_request_threaded_irq(dev, bl->irq, NULL, bt_host_wake_thread,
					IRQF_ONESHOT, "bt_host_wake", bl);	/* S:drivers/bluetooth/bcm43xx.c:254-256 name */
	if (ret)
		goto err_peer;

	bl->rfk = rfkill_alloc("bcm43xx Bluetooth", dev, RFKILL_TYPE_BLUETOOTH,
			       &bt_rfkill_ops, bl);	/* S:drivers/bluetooth/bcm43xx.c:323-325, A:include/linux/rfkill.h:79 */
	if (!bl->rfk) {
		ret = -ENOMEM;
		goto err_peer;
	}

	rfkill_init_sw_state(bl->rfk, 0);	/* S:drivers/bluetooth/bcm43xx.c:335 */
	ret = rfkill_register(bl->rfk);		/* S:drivers/bluetooth/bcm43xx.c:337 */
	if (ret) {
		rfkill_destroy(bl->rfk);
		goto err_peer;
	}

	rfkill_set_sw_state(bl->rfk, true);	/* S:drivers/bluetooth/bcm43xx.c:348; persistent rfkill, no sync scheduled */

	WRITE_ONCE(bt_lpm_g, bl);

	put_device(&uart->dev);	/* (final, R3-15a) drop the of_find_device_by_node reference on every path, success included */

	dev_info(dev, "ready: bt_en gpj1-7 bt_wake gpj1-4 host_wake irq %d, lpm %d\n",
		 bl->irq, bt_lpm_mode);		/* L6D 8.3 S7 */
	return 0;

err_peer:
	s3c24xx_serial_set_wake_peer(&uart->dev, NULL);
	put_device(&uart->dev);
	return ret;
}

static const struct of_device_id bcm43xx_bt_lpm_of_match[] = {
	{ .compatible = "samsung,bcm43xx" },	/* FDT:11976 */
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, bcm43xx_bt_lpm_of_match);

static struct platform_driver bcm43xx_bt_lpm_driver = {
	.probe = bcm43xx_bt_lpm_probe,
	.driver = {
		.name = "bcm43xx-bt-lpm",
		.of_match_table = bcm43xx_bt_lpm_of_match,
		.suppress_bind_attrs = true,	/* built in and never unbound: no .remove */
	},
};
builtin_platform_driver(bcm43xx_bt_lpm_driver);

MODULE_DESCRIPTION("Broadcom BCM43xx Bluetooth LPM Driver");
MODULE_LICENSE("GPL");
