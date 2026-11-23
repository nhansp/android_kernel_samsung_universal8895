/*
 * Copyright (C) 2010 Samsung Electronics.
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

#include <linux/init.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/gpio/consumer.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/mcu_ipc.h>
#include <linux/shm_ipc.h>
#include <linux/modem_notifier.h>
#include <soc/samsung/exynos8895-smc.h>
#include <soc/samsung/pmu-cp.h>

#include "modem_prj.h"
#include "modem_utils.h"
#include "link_device_memory.h"

/* The panel id the bootloader passes, which the vendor panel driver reported */
static int mif_lcdtype;

static int __init mif_lcdtype_setup(char *arg)
{
	get_option(&arg, &mif_lcdtype);
	return 0;
}
early_param("lcdtype", mif_lcdtype_setup);

#define MIF_INIT_TIMEOUT	(15 * HZ)

static inline int change_cp_pmu_manual_reset(void) {return 0; }

static struct modem_ctl *g_mc;

static irqreturn_t cp_wdt_handler(int irq, void *arg)
{
	struct modem_ctl *mc = (struct modem_ctl *)arg;
	struct io_device *iod;
	enum modem_state new_state;

	mif_disable_irq(&mc->irq_cp_wdt);
	mif_err("%s: ERR! CP_WDOG occurred\n", mc->name);

	if (mc->phone_state == STATE_ONLINE)
		modem_notify_event(MODEM_EVENT_WATCHDOG);

	/* Disable debug Snapshot */
	mif_set_snapshot(false);

	exynos_clear_cp_reset();
	new_state = STATE_CRASH_WATCHDOG;

	mif_err("new_state = %s\n", cp_state_str(new_state));

	list_for_each_entry(iod, &mc->modem_state_notify_list, list) {
		if (iod && atomic_read(&iod->opened) > 0)
			iod->modem_state_changed(iod, new_state);
	}

	return IRQ_HANDLED;
}

static irqreturn_t cp_fail_handler(int irq, void *arg)
{
	struct modem_ctl *mc = (struct modem_ctl *)arg;
	struct io_device *iod;
	enum modem_state new_state;

	mif_disable_irq(&mc->irq_cp_fail);
	mif_err("%s: ERR! CP_FAIL occurred\n", mc->name);

	exynos_cp_active_clear();
	new_state = STATE_CRASH_RESET;

	mif_err("new_state = %s\n", cp_state_str(new_state));

	list_for_each_entry(iod, &mc->modem_state_notify_list, list) {
		if (iod && atomic_read(&iod->opened) > 0)
			iod->modem_state_changed(iod, new_state);
	}

	return IRQ_HANDLED;
}

static void cp_active_handler(void *arg)
{
	struct modem_ctl *mc = (struct modem_ctl *)arg;
	struct io_device *iod;
	int cp_on = exynos_get_cp_power_status();
	int cp_active = mbox_extract_value(MCU_CP, mc->mbx_cp_status,
					mc->sbi_lte_active_mask, mc->sbi_lte_active_pos);
	enum modem_state old_state = mc->phone_state;
	enum modem_state new_state = mc->phone_state;

	mif_err("old_state:%s cp_on:%d cp_active:%d\n",
		cp_state_str(old_state), cp_on, cp_active);

	if (!cp_active) {
		if (cp_on > 0) {
			new_state = STATE_OFFLINE;
			complete_all(&mc->off_cmpl);
		} else {
			mif_err("don't care!!!\n");
		}
	}

	if (old_state != new_state) {
		mif_err("new_state = %s\n", cp_state_str(new_state));

		if (old_state == STATE_ONLINE)
			modem_notify_event(MODEM_EVENT_EXIT);

		list_for_each_entry(iod, &mc->modem_state_notify_list, list) {
			if (iod && atomic_read(&iod->opened) > 0)
				iod->modem_state_changed(iod, new_state);
		}
	}
}

static int get_system_rev(struct device *dev)
{
	int value, cnt, gpio_cnt;
	unsigned int hw_rev = 0;
	struct gpio_desc *gpio_hw_rev;

	gpio_cnt = gpiod_count(dev, NULL);
	if (gpio_cnt < 0) {
		mif_err("failed to get gpio_count from DT(%d)\n", gpio_cnt);
		return gpio_cnt;
	}

	for (cnt = 0; cnt < gpio_cnt; cnt++) {
		gpio_hw_rev = devm_gpiod_get_index(dev, NULL, cnt, GPIOD_IN);
		if (IS_ERR(gpio_hw_rev)) {
			mif_err("gpio_hw_rev%d: Invalied gpio\n", cnt);
			return -EINVAL;
		}

		value = gpiod_get_value_cansleep(gpio_hw_rev);
		hw_rev |= (value & 0x1) << cnt;
		devm_gpiod_put(dev, gpio_hw_rev);
	}

	return hw_rev;
}

#ifdef CONFIG_GPIO_DS_DETECT
static int get_ds_detect(struct device_node *np)
{
	unsigned gpio_ds_det;

	gpio_ds_det = of_get_named_gpio(np, "mif,gpio_ds_det", 0);
	if (!gpio_is_valid(gpio_ds_det)) {
		mif_err("gpio_ds_det: Invalid gpio\n");
		return 0;
	}

	return gpio_get_value(gpio_ds_det);
}
#else
static int ds_detect = 1;
module_param(ds_detect, int, S_IRUGO | S_IWUSR | S_IWGRP);
MODULE_PARM_DESC(ds_detect, "Dual SIM detect");

static int get_ds_detect(struct device_node *np)
{
	mif_info("Dual SIM detect = %d\n", ds_detect);
	return ds_detect - 1;
}
#endif

static int init_mailbox_regs(struct modem_ctl *mc)
{
	struct platform_device *pdev = to_platform_device(mc->dev);
	struct device_node *np = pdev->dev.of_node;
	unsigned int mbx_ap_status;
	unsigned int sbi_ds_det_mask, sbi_ds_det_pos;
	unsigned int sbi_sys_rev_mask, sbi_sys_rev_pos;
	int sys_rev, ds_det, i;
#ifdef CONFIG_CP_RAM_LOGGING
	unsigned int sbi_ext_backtrace_mask, sbi_ext_backtrace_pos;
#endif
	unsigned int sbi_device_type_mask, sbi_device_type_pos;
	unsigned int value = 0;

	for (i = 0; i < MAX_MBOX_NUM; i++)
		mbox_set_value(MCU_CP, i, 0);

	if (np) {
		mif_dt_read_u32(np, "mbx_ap2cp_united_status", mbx_ap_status);
		mif_dt_read_u32(np, "sbi_sys_rev_mask", sbi_sys_rev_mask);
		mif_dt_read_u32(np, "sbi_sys_rev_pos", sbi_sys_rev_pos);
		mif_dt_read_u32(np, "sbi_ds_det_mask", sbi_ds_det_mask);
		mif_dt_read_u32(np, "sbi_ds_det_pos", sbi_ds_det_pos);

		mif_dt_read_u32(np, "sbi_device_type_mask", sbi_device_type_mask);
		mif_dt_read_u32(np, "sbi_device_type_pos", sbi_device_type_pos);

		/* [3:0] display type (lcdtype [19:16]), [4] display connected */
		value = (mif_lcdtype >= 0) ? ((1 << 4) | ((mif_lcdtype >> 16) & 0xf)) : 0;

		mbox_update_value(MCU_CP, mbx_ap_status, value,
			sbi_device_type_mask, sbi_device_type_pos);

		mif_info("dsp_type:0x%x, conn:0x%x, get_val: 0x%x\n",
			mif_lcdtype, mif_lcdtype >= 0,
			mbox_get_value(MCU_CP, mbx_ap_status));

		sys_rev = get_system_rev(&pdev->dev);
		ds_det = get_ds_detect(np);
		if (sys_rev < 0 || ds_det < 0) {
			mif_info("sys_rev:%d ds_det:%d\n", sys_rev, ds_det);
			return -EINVAL;
		}

		mbox_update_value(MCU_CP, mbx_ap_status, sys_rev,
			sbi_sys_rev_mask, sbi_sys_rev_pos);
		mbox_update_value(MCU_CP, mbx_ap_status, ds_det,
			sbi_ds_det_mask, sbi_ds_det_pos);

		mif_info("sys_rev:%d, ds_det:%u (0x%x)\n",
				sys_rev, ds_det, mbox_get_value(MCU_CP, mbx_ap_status));

#ifdef CONFIG_CP_RAM_LOGGING
		mif_dt_read_u32(np, "sbi_ext_backtrace_mask",
				sbi_ext_backtrace_mask);
		mif_dt_read_u32(np, "sbi_ext_backtrace_pos",
				sbi_ext_backtrace_pos);

		mbox_update_value(MCU_CP, mbx_ap_status, shm_get_cplog_flag(),
			sbi_ext_backtrace_mask, sbi_ext_backtrace_pos);
#endif
	} else {
		mif_info("non-DT project, can't set system_rev\n");
	}

	return 0;
}

static int ss310ap_on(struct modem_ctl *mc)
{
	int ret;
	int cp_active = mbox_extract_value(MCU_CP, mc->mbx_cp_status,
						mc->sbi_lte_active_mask, mc->sbi_lte_active_pos);
	int cp_status = mbox_extract_value(MCU_CP, mc->mbx_cp_status,
						mc->sbi_cp_status_mask, mc->sbi_cp_status_pos);

	mif_err("+++\n");
	mif_err("cp_active:%d cp_status:%d\n", cp_active, cp_status);

#ifndef CONFIG_CP_SECURE_BOOT
	exynos_cp_init();
#endif

	/* Enable debug Snapshot */
	mif_set_snapshot(true);

	mc->phone_state = STATE_OFFLINE;

	if (init_mailbox_regs(mc))
		mif_err("Failed to initialize mbox regs\n");

	mbox_update_value(MCU_CP, mc->mbx_ap_status, 1,
			mc->sbi_pda_active_mask, mc->sbi_pda_active_pos);

	if (exynos_get_cp_power_status() > 0) {
		mif_err("CP aleady Power on, Just start!\n");
		exynos_cp_release();
	} else {
		exynos_set_cp_power_onoff(CP_POWER_ON);
	}

	msleep(300);
	ret = change_cp_pmu_manual_reset();
	mif_err("change_mr_reset -> %d\n", ret);

	mif_info("---\n");
	return 0;
}

static int ss310ap_off(struct modem_ctl *mc)
{
	mif_err("+++\n");

	mbox_set_interrupt(MCU_CP, mc->int_cp_wakeup);
	msleep(5);
	exynos_set_cp_power_onoff(CP_POWER_OFF);

	mif_err("---\n");
	return 0;
}

static int ss310ap_shutdown(struct modem_ctl *mc)
{
	struct io_device *iod;
	unsigned long timeout = msecs_to_jiffies(3000);
	unsigned long remain;

	mif_err("+++\n");

	if (mc->phone_state == STATE_OFFLINE
		|| exynos_get_cp_power_status() <= 0)
		goto exit;

	reinit_completion(&mc->off_cmpl);
	remain = wait_for_completion_timeout(&mc->off_cmpl, timeout);
	if (remain == 0) {
		mif_err("T-I-M-E-O-U-T\n");
		mc->phone_state = STATE_OFFLINE;
		list_for_each_entry(iod, &mc->modem_state_notify_list, list) {
			if (iod && atomic_read(&iod->opened) > 0)
				iod->modem_state_changed(iod, STATE_OFFLINE);
		}
	}

exit:
	mbox_set_interrupt(MCU_CP, mc->int_cp_wakeup);
	msleep(5);
	exynos_set_cp_power_onoff(CP_POWER_OFF);
	mif_err("---\n");
	return 0;
}

static int ss310ap_reset(struct modem_ctl *mc)
{
	void __iomem *base = shm_get_ipc_region();
	mif_err("+++\n");

	/* mc->phone_state = STATE_OFFLINE; */
	if (mc->phone_state == STATE_OFFLINE)
		return 0;

	/* FIXME: For CP debug */
	if (base) {
		if (*(unsigned int *)(base + 0xF80) == 0xDEB)
			return 0;
	}

	if (mc->phone_state == STATE_ONLINE)
		modem_notify_event(MODEM_EVENT_RESET);

	/* Change phone state to OFFLINE */
	mc->phone_state = STATE_OFFLINE;

	if (exynos_get_cp_power_status() > 0) {
		mif_err("CP aleady Power on, try reset\n");
		mbox_set_interrupt(MCU_CP, mc->int_cp_wakeup);
		msleep(5);
		exynos_cp_reset();
		mbox_sw_reset(MCU_CP);
	}

	mif_err("---\n");
	return 0;
}

static int ss310ap_boot_on(struct modem_ctl *mc)
{
	struct link_device *ld = get_current_link(mc->iod);
	struct io_device *iod;
	int cnt = 100;

	mif_info("+++\n");

	if (ld->boot_on)
		ld->boot_on(ld, mc->bootd);

	list_for_each_entry(iod, &mc->modem_state_notify_list, list) {
		if (iod && atomic_read(&iod->opened) > 0)
			iod->modem_state_changed(iod, STATE_BOOTING);
	}

	while (mbox_extract_value(MCU_CP, mc->mbx_cp_status,
				mc->sbi_cp_status_mask, mc->sbi_cp_status_pos) == 0) {
		if (--cnt > 0)
			usleep_range(10000, 20000);
		else
			return -EACCES;
	}

	mif_disable_irq(&mc->irq_cp_wdt);
	mif_enable_irq(&mc->irq_cp_fail);

	mif_info("cp_status=%u\n",
			mbox_extract_value(MCU_CP, mc->mbx_cp_status,
					mc->sbi_cp_status_mask, mc->sbi_cp_status_pos));

	mif_info("---\n");
	return 0;
}

static int ss310ap_boot_off(struct modem_ctl *mc)
{
	struct io_device *iod;
	unsigned long remain;
	int err = 0;
	mif_info("+++\n");

	reinit_completion(&mc->init_cmpl);
	remain = wait_for_completion_timeout(&mc->init_cmpl, MIF_INIT_TIMEOUT);
	if (remain == 0) {
		mif_err("T-I-M-E-O-U-T\n");
		err = -EAGAIN;
		goto exit;
	}

	mif_enable_irq(&mc->irq_cp_wdt);

	list_for_each_entry(iod, &mc->modem_state_notify_list, list) {
		if (iod && atomic_read(&iod->opened) > 0)
			iod->modem_state_changed(iod, STATE_ONLINE);
	}

	mif_info("---\n");

exit:
	mif_disable_irq(&mc->irq_cp_fail);
	return err;
}

static int ss310ap_boot_done(struct modem_ctl *mc)
{
	mif_info("+++\n");
	mif_info("---\n");
	return 0;
}

static int ss310ap_force_crash_exit(struct modem_ctl *mc)
{
	struct link_device *ld = get_current_link(mc->bootd);
	mif_err("+++\n");

	/* Make DUMP start */
	ld->force_dump(ld, mc->bootd);

	mif_err("---\n");
	return 0;
}

int ss310ap_force_crash_exit_ext(void)
{
	if (g_mc) {
		mif_err("Make forced crash exit\n");
		ss310ap_force_crash_exit(g_mc);
	}

	return 0;
}
EXPORT_SYMBOL(ss310ap_force_crash_exit_ext);

int ss310ap_send_panic_noti_ext(void)
{
	struct modem_data *modem;

	if (g_mc) {
		modem = g_mc->mdm_data;
		if (modem->mld) {
			mif_err("Send CMD_KERNEL_PANIC message to CP\n");
			send_ipc_irq(modem->mld, cmd2int(CMD_KERNEL_PANIC));
		}
	}

	return 0;
}
EXPORT_SYMBOL(ss310ap_send_panic_noti_ext);

static int ss310ap_dump_start(struct modem_ctl *mc)
{
	int err;
	struct link_device *ld = get_current_link(mc->bootd);
	mif_err("+++\n");

	if (!ld->dump_start) {
		mif_err("ERR! %s->dump_start not exist\n", ld->name);
		return -EFAULT;
	}

	/* Change phone state to CRASH_EXIT */
	mc->phone_state = STATE_CRASH_EXIT;

	err = ld->dump_start(ld, mc->bootd);
	if (err)
		return err;

	exynos_cp_release();

	mbox_update_value(MCU_CP, mc->mbx_ap_status, 1,
			mc->sbi_ap_status_mask, mc->sbi_ap_status_pos);

	mif_err("---\n");
	return err;
}

static void ss310ap_modem_boot_confirm(struct modem_ctl *mc)
{
	mbox_update_value(MCU_CP,mc->mbx_ap_status, 1,
			mc->sbi_ap_status_mask, mc->sbi_ap_status_pos);
	mif_info("ap_status=%u\n",
			mbox_extract_value(MCU_CP, mc->mbx_ap_status,
					mc->sbi_ap_status_mask, mc->sbi_ap_status_pos));
}

static void ss310ap_get_ops(struct modem_ctl *mc)
{
	mc->ops.modem_on = ss310ap_on;
	mc->ops.modem_off = ss310ap_off;
	mc->ops.modem_shutdown = ss310ap_shutdown;
	mc->ops.modem_reset = ss310ap_reset;
	mc->ops.modem_boot_on = ss310ap_boot_on;
	mc->ops.modem_boot_off = ss310ap_boot_off;
	mc->ops.modem_boot_done = ss310ap_boot_done;
	mc->ops.modem_force_crash_exit = ss310ap_force_crash_exit;
	mc->ops.modem_dump_start = ss310ap_dump_start;
	mc->ops.modem_boot_confirm = ss310ap_modem_boot_confirm;
}

static void ss310ap_get_pdata(struct modem_ctl *mc, struct modem_data *modem)
{
	struct modem_mbox *mbx = modem->mbx;

	mc->int_pda_active = mbx->int_ap2cp_active;

	mc->int_cp_wakeup = mbx->int_ap2cp_wakeup;

	mc->irq_phone_active = mbx->irq_cp2ap_active;

	mc->mbx_ap_status = mbx->mbx_ap2cp_status;
	mc->mbx_cp_status = mbx->mbx_cp2ap_status;

	mc->mbx_perf_req = mbx->mbx_cp2ap_perf_req;
	mc->irq_perf_req = mbx->irq_cp2ap_perf_req;

	mc->int_uart_noti = mbx->int_ap2cp_uart_noti;

	mc->sbi_lte_active_mask = mbx->sbi_lte_active_mask;
	mc->sbi_lte_active_pos = mbx->sbi_lte_active_pos;
	mc->sbi_cp_status_mask = mbx->sbi_cp_status_mask;
	mc->sbi_cp_status_pos = mbx->sbi_cp_status_pos;

	mc->sbi_pda_active_mask = mbx->sbi_pda_active_mask;
	mc->sbi_pda_active_pos = mbx->sbi_pda_active_pos;
	mc->sbi_ap_status_mask = mbx->sbi_ap_status_mask;
	mc->sbi_ap_status_pos = mbx->sbi_ap_status_pos;

	mc->sbi_uart_noti_mask = mbx->sbi_uart_noti_mask;
	mc->sbi_uart_noti_pos = mbx->sbi_uart_noti_pos;

	mc->sbi_crash_type_mask = mbx->sbi_crash_type_mask;
	mc->sbi_crash_type_pos = mbx->sbi_crash_type_pos;
}

int ss310ap_init_modemctl_device(struct modem_ctl *mc, struct modem_data *pdata)
{
	struct platform_device *pdev = to_platform_device(mc->dev);
	struct device_node *np = pdev->dev.of_node;
	int ret = 0;
	unsigned int irq_num;
	unsigned long flags = IRQF_NO_SUSPEND | IRQF_NO_THREAD;
	struct gpio_desc *cp_rst_n;
	struct regmap *pmu;

	mif_err("+++\n");

	pmu = syscon_regmap_lookup_by_phandle(np, "samsung,pmu-syscon");
	if (IS_ERR(pmu)) {
		mif_err("samsung,pmu-syscon: %ld\n", PTR_ERR(pmu));
		return PTR_ERR(pmu);
	}
	exynos_pmu_cp_set_regmap(pmu);

	g_mc = mc;

	ss310ap_get_ops(mc);
	ss310ap_get_pdata(mc, pdata);
	dev_set_drvdata(mc->dev, mc);

	/*
	** Register CP_FAIL interrupt handler
	*/
	ret = platform_get_irq(pdev, 0);
	if (ret < 0)
		return ret;
	irq_num = ret;
	mif_init_irq(&mc->irq_cp_fail, irq_num, "cp_fail", flags);

	ret = mif_request_irq(&mc->irq_cp_fail, cp_fail_handler, mc);
	if (ret)	{
		mif_err("Failed to request_irq with(%d)", ret);
		return ret;
	}

	/* CP_FAIL interrupt must be enabled only during CP booting */
	mc->irq_cp_fail.active = true;
	mif_disable_irq(&mc->irq_cp_fail);

	/*
	** Register CP_WDT interrupt handler
	*/
	ret = platform_get_irq(pdev, 1);
	if (ret < 0)
		return ret;
	irq_num = ret;
	mif_init_irq(&mc->irq_cp_wdt, irq_num, "cp_wdt", flags);

	ret = mif_request_irq(&mc->irq_cp_wdt, cp_wdt_handler, mc);
	if (ret) {
		mif_err("Failed to request_irq with(%d)", ret);
		return ret;
	}

	/* CP_WDT interrupt must be enabled only after CP booting */
	mc->irq_cp_wdt.active = true;
	mif_disable_irq(&mc->irq_cp_wdt);

	init_completion(&mc->init_cmpl);
	init_completion(&mc->off_cmpl);

	/*
	** Register LTE_ACTIVE MBOX interrupt handler
	*/
	ret = mbox_request_irq(MCU_CP, mc->irq_phone_active, cp_active_handler, mc);
	if (ret) {
		mif_err("Failed to mbox_request_irq %u with(%d)",
				mc->irq_phone_active, ret);
		return ret;
	}

	/*
	** Get/set CP_RST_N
	*/
	cp_rst_n = devm_gpiod_get_optional(&pdev->dev, "modem_ctrl,gpio_cp_rst_n", GPIOD_OUT_HIGH);
	if (IS_ERR(cp_rst_n)) {
		mif_err("fail req gpio %s:%ld\n", "CP_RST_N", PTR_ERR(cp_rst_n));
		return -ENODEV;
	}

	mif_err("---\n");
	return 0;
}
