// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *	      http://www.samsung.com/
 *
 * EXYNOS - Logging message from Secure World
 * Author: Junho Choi <junhosj.choi@samsung.com>
 */

#include <linux/err.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqreturn.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/debugfs.h>
#include <soc/samsung/exynos8895-smc.h>

#include <soc/samsung/exynos-seclog.h>

/*
 * The monitor sets up one MIN_LOG_BUF_LEN ring per CPU of the vendor kernel,
 * built with NR_CPUS = 8; a larger NR_CPUS would walk past the region.
 */
#define SECLOG_NR_CPUS				8

#define SECLOG_PHYS_TO_VIRT(addr)		((unsigned long)ldata.virt_addr	\
						+ ((unsigned long)(addr)	\
						- ldata.phys_addr))

static struct seclog_data ldata;
static struct seclog_ctx slog_ctx;
static struct sec_log_info *sec_log[SECLOG_NR_CPUS];

static bool exynos_seclog_pa_ok(unsigned long pa)
{
	return pa >= ldata.phys_addr && pa - ldata.phys_addr < ldata.size;
}

static void exynos_seclog_worker(struct work_struct *work)
{
	struct log_header_info *v_log_h = NULL;
	char *v_log = NULL;
	unsigned long v_log_addr = 0;
	unsigned long slot_start, slot_end;
	unsigned int cpu = 0;
	unsigned int log_len;

	pr_debug("%s: Start seclog_worker\n", __func__);

	/* Print log message in a message buffer */
	for (cpu = 0; cpu < SECLOG_NR_CPUS; cpu++) {
		slot_start = (unsigned long)sec_log[cpu];
		slot_end = slot_start + MIN_LOG_BUF_LEN;

		/* nothing pending: the ring addresses are not needed */
		if (READ_ONCE(sec_log[cpu]->log_read_cnt) ==
		    READ_ONCE(sec_log[cpu]->log_write_cnt))
			continue;
		if (!exynos_seclog_pa_ok(READ_ONCE(sec_log[cpu]->start_log_addr)))
			goto bad_entry;
		v_log_addr = SECLOG_PHYS_TO_VIRT(READ_ONCE(sec_log[cpu]->start_log_addr));

		while (READ_ONCE(sec_log[cpu]->log_read_cnt) !=
		       READ_ONCE(sec_log[cpu]->log_write_cnt)) {
			/* Check the log message is reached the end of log buffer */
			if (READ_ONCE(sec_log[cpu]->log_return_cnt) &&
			    READ_ONCE(sec_log[cpu]->log_read_cnt) ==
			    READ_ONCE(sec_log[cpu]->log_return_cnt)) {
				sec_log[cpu]->log_return_cnt = 0;
				if (!exynos_seclog_pa_ok(READ_ONCE(sec_log[cpu]->initial_log_addr)))
					goto bad_entry;
				v_log_addr = READ_ONCE(sec_log[cpu]->initial_log_addr);
				v_log_addr = SECLOG_PHYS_TO_VIRT(v_log_addr);
			}

			pr_debug("[SECLOG_DEBUG C%d] read_cnt[%d]\n",
				 cpu, sec_log[cpu]->log_read_cnt);
			pr_debug("[SECLOG_DEBUG C%d] write_cnt[%d]\n",
				 cpu, sec_log[cpu]->log_write_cnt);
			pr_debug("[SECLOG_DEBUG C%d] return_cnt[%d]\n",
				 cpu, sec_log[cpu]->log_return_cnt);
			pr_debug("[SECLOG_DEBUG C%d] v_log_addr[%#lx]\n",
				 cpu, v_log_addr);
			pr_debug("[SECLOG_DEBUG C%d] p_log_addr[%#lx]\n",
				 cpu,
				 v_log_addr
				 - (unsigned long)ldata.virt_addr
				 + ldata.phys_addr);

			if (v_log_addr < slot_start ||
			    v_log_addr + sizeof(struct log_header_info) > slot_end)
				goto bad_entry;

			v_log_h = (struct log_header_info *)v_log_addr;
			v_log = (char *)v_log_addr + sizeof(struct log_header_info);
			log_len = READ_ONCE(v_log_h->log_len);

			if (log_len > slot_end - (unsigned long)v_log)
				goto bad_entry;

			pr_debug("[SECLOG_DEBUG C%d] v_log_h[%#lx]\n",
				 cpu, (unsigned long)v_log_h);
			pr_debug("[SECLOG_DEBUG C%d] v_log[%#lx]\n",
				 cpu, (unsigned long)v_log);
			pr_debug("[SECLOG_DEBUG C%d] log_len = %d\n",
				 cpu, log_len);

			pr_info("[SECLOG C%u] [%06u.%06u] %.*s",
				cpu,
				v_log_h->tv_sec,
				v_log_h->tv_usec,
				(int)log_len, v_log);

			v_log_addr += (sizeof(struct log_header_info) + log_len);
			CHECK_AND_ALIGN_4BYTES(v_log_addr);

			WRITE_ONCE(sec_log[cpu]->log_read_cnt,
				   READ_ONCE(sec_log[cpu]->log_read_cnt) + 1);
		}
		continue;

bad_entry:
		pr_err("seclog: C%u bad entry, resync\n", cpu);
		WRITE_ONCE(sec_log[cpu]->log_read_cnt,
			   READ_ONCE(sec_log[cpu]->log_write_cnt));
		WRITE_ONCE(sec_log[cpu]->log_return_cnt, 0);
	}
}

static irqreturn_t exynos_seclog_irq_handler(int irq, void *dev_id)
{
	unsigned int cpu = 0;

	if (slog_ctx.enabled) {
		schedule_work(&slog_ctx.work);
	} else {
		for (cpu = 0; cpu < SECLOG_NR_CPUS; cpu++) {
			WRITE_ONCE(sec_log[cpu]->log_read_cnt,
				   READ_ONCE(sec_log[cpu]->log_write_cnt));
			WRITE_ONCE(sec_log[cpu]->log_return_cnt, 0);
		}
	}

	pr_debug("ISR for Secure log is implemented!\n");

	return IRQ_HANDLED;
}

static void exynos_seclog_ldfw_errors(struct device *dev, int err)
{
	int i, err_ldfw;

	for (i = 0; i < LDFW_MAX_NUM; i++) {
		err_ldfw = (err >> (BITLEN_LDFW_ERROR * i)) & MASK_LDFW_ERROR;
		if (!err_ldfw)
			continue;

		switch (err_ldfw) {
		case ERROR_NOT_SUPPORT_LDFW_SEC_LOG:
			dev_err(dev, "[ERROR] LDFW[%d] doesn't support Secure log\n", i);
			break;
		case ERROR_LDFW_ALREADY_INITIALIZED:
			dev_err(dev, "[ERROR] LDFW[%d] already initialized Secure log\n", i);
			break;
		case ERROR_NOT_SUPPORT_LDFW_ERR_VALUE:
			dev_err(dev, "[ERROR] LDFW[%d] returns unsupported error value\n", i);
			break;
		default:
			dev_err(dev, "[ERROR] LDFW[%d] Unknown error value from LDFW [err = %#x]\n",
				i, err_ldfw);
			break;
		}
	}
}

static int exynos_seclog_probe(struct platform_device *pdev)
{
	irq_hw_number_t hwirq = 0;
	struct resource res;
	void *va;
	int irq, err, i;

	/* The vendor driver took the region with RESERVEDMEM_OF_DECLARE */
	err = of_reserved_mem_region_to_resource(pdev->dev.of_node, 0, &res);
	if (err) {
		dev_err(&pdev->dev, "Fail to get seclog_mem (%d)\n", err);
		return err;
	}
	if (resource_size(&res) < SECLOG_NR_CPUS * MIN_LOG_BUF_LEN) {
		dev_err(&pdev->dev, "seclog_mem too small (%llu bytes)\n",
			(unsigned long long)resource_size(&res));
		return -EINVAL;
	}
	va = devm_memremap(&pdev->dev, res.start, resource_size(&res), MEMREMAP_WC);
	if (IS_ERR(va)) {
		dev_err(&pdev->dev, "Fail to translate message buffer\n");
		return PTR_ERR(va);
	}
	ldata.phys_addr = res.start;
	ldata.size = resource_size(&res);
	ldata.virt_addr = va;

	for (i = 0; i < SECLOG_NR_CPUS; i++) {
		sec_log[i] = (struct sec_log_info *)((unsigned long)ldata.virt_addr
								+ (MIN_LOG_BUF_LEN * i));
		dev_dbg(&pdev->dev,
			"sec_log[C%d]: %#lx\n",
			i, (unsigned long)sec_log[i]);
	}

	INIT_WORK(&slog_ctx.work, exynos_seclog_worker);
	slog_ctx.enabled = true;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	slog_ctx.irq = irq;

	hwirq = irqd_to_hwirq(irq_get_irq_data(slog_ctx.irq));

	dev_dbg(&pdev->dev,
		"hwirq for seclog (%ld)\n",
		hwirq);

	err = devm_request_irq(&pdev->dev, slog_ctx.irq,
			       exynos_seclog_irq_handler,
			       IRQF_TRIGGER_RISING, "seclog", NULL);
	if (err) {
		dev_err(&pdev->dev,
			"Fail to request IRQ handler. err(%d) irq(%d)\n",
			err, slog_ctx.irq);
		return err;
	}

	slog_ctx.debug_dir = debugfs_create_dir("seclog", NULL);
	debugfs_create_bool("seclog_debug", 0600, slog_ctx.debug_dir,
			    &slog_ctx.enabled);

	dev_dbg(&pdev->dev,
		"SMC arguments(%#x, %#lx, %#lx, %ld)\n",
		SMC_CMD_SEC_LOG_INFO, ldata.phys_addr, ldata.size, hwirq);

	/* the monitor returns a 32-bit error code */
	err = (int)exynos8895_smc(SMC_CMD_SEC_LOG_INFO, res.start, resource_size(&res), hwirq);
	if (err) {
		switch (err) {
		case ERROR_INVALID_LOG_LEN:
			dev_err(&pdev->dev,
				"[ERROR] Invalid log length [Message buffer length = %#lx]\n",
				ldata.size);
			break;
		case ERROR_INVALID_LOG_ADDR:
			dev_err(&pdev->dev,
				"[ERROR] Invalid log address [Message buffer address = %#lx]\n",
				ldata.phys_addr);
			break;
		case ERROR_INVALID_INTR_NUM:
			dev_err(&pdev->dev,
				"[ERROR] Invalid interrupt number [Interrupt number = %ld]\n",
				hwirq);
			break;
		case ERROR_ALREADY_INITIALIZED:
			dev_err(&pdev->dev, "[ERROR] Already initialized\n");
			break;
		case (int)SMC_CMD_SEC_LOG_INFO:
			dev_err(&pdev->dev, "[ERROR] EL3 Monitor doesn't support Secure log\n");
			break;
		default:
			if ((err & MASK_LDFW_MAGIC) == LDFW_MAGIC) {
				exynos_seclog_ldfw_errors(&pdev->dev, err);

				/* not fatal: the monitor took the buffer */
				goto detect_ldfw_err;
			} else {
				dev_err(&pdev->dev,
					"[ERROR] Unknown error value [err = %#x]\n",
					err);
				break;
			}
		}

		dev_err(&pdev->dev, "Fail to initialize Secure log\n");

		/* no queued work may run after the devm unmap */
		devm_free_irq(&pdev->dev, slog_ctx.irq, NULL);
		cancel_work_sync(&slog_ctx.work);
		debugfs_remove(slog_ctx.debug_dir);

		return -EINVAL;
	}

detect_ldfw_err:
	dev_info(&pdev->dev,
		 "Message buffer address[%#lx], Message buffer size[%#lx]\n",
		 ldata.phys_addr, ldata.size);
	dev_info(&pdev->dev, "Exynos Secure Log driver probe done!\n");

	return 0;
}

static const struct of_device_id exynos_seclog_of_match_table[] = {
	{ .compatible = "samsung,exynos-seclog", },
	{ },
};

static struct platform_driver exynos_seclog_driver = {
	.probe = exynos_seclog_probe,
	.driver = {
		.name = "exynos-seclog",
		.of_match_table = exynos_seclog_of_match_table,
		/* no unbind: the monitor keeps the buffer */
		.suppress_bind_attrs = true,
	}
};
builtin_platform_driver(exynos_seclog_driver);
