// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung ABOX adaptation driver
 *
 * Copyright (c) 2016 Samsung Electronics Co. Ltd.
 */
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <sound/soc.h>
#include <sound/samsung/abox.h>
#include <sound/samsung/abox_dsm.h>
#include "abox.h"

#define DSM_ERAP_TIMEOUT_MS		130
#define DSM_PARAM_COUNT			185
#define DSM_WRITE_CB			0xDAAD
#define DSM_FLAG_WRITE_RDC_CAL_ONLY	0xca00ca00
#define DSM_IDX_RDC_AT_ROOMTEMP		6
#define DSM_IDX_COLDTEMP		8
#define DSM_IDX_WRITE_FLAG		63

static struct device *dsm_abox_dev;
static DECLARE_WAIT_QUEUE_HEAD(dsm_wq_write);
static bool dsm_write_done;
static DEFINE_MUTEX(dsm_lock);

/*
 * The parameters other than the write flag and the calibration stay zero, as
 * in the first write the vendor Maxim DSM driver made after boot.
 */
int abox_dsm_set_rdc_temp(int rdc, int temp)
{
	ABOX_IPC_MSG msg = { 0 };
	struct IPC_ERAP_MSG *erap = &msg.msg.erap;
	unsigned int *param = &erap->param.raw.params[3];
	int ret;

	if (!dsm_abox_dev)
		return -ENODEV;
	mutex_lock(&dsm_lock);
	msg.ipcid = IPC_ERAP;
	erap->msgtype = REALTIME_EXTRA;
	erap->param.raw.params[0] = 1;
	erap->param.raw.params[1] = 0;
	erap->param.raw.params[2] = DSM_PARAM_COUNT;
	param[DSM_IDX_WRITE_FLAG] = DSM_FLAG_WRITE_RDC_CAL_ONLY;
	param[DSM_IDX_RDC_AT_ROOMTEMP] = rdc << 8;
	param[DSM_IDX_COLDTEMP] = temp << 19;
	WRITE_ONCE(dsm_write_done, false);
	ret = abox_start_ipc_transaction(dsm_abox_dev, IPC_ERAP, &msg,
					 sizeof(msg), 1, 1);
	if (ret < 0)
		pr_err("%s: abox_start_ipc_transaction is failed\n", __func__);
	else if (!wait_event_interruptible_timeout(dsm_wq_write,
						   READ_ONCE(dsm_write_done),
						   msecs_to_jiffies(DSM_ERAP_TIMEOUT_MS)))
		pr_err("%s: wait_event timeout\n", __func__);
	else
		dev_dbg(dsm_abox_dev, "%s: rdc 0x%08x temp %d applied\n",
			__func__, rdc, temp);
	mutex_unlock(&dsm_lock);
	return ret < 0 ? ret : 0;
}
EXPORT_SYMBOL_GPL(abox_dsm_set_rdc_temp);

static irqreturn_t abox_adaptation_irq_handler(int ipc_id, void *dev_id,
					       ABOX_IPC_MSG *msg)
{
	struct IPC_ERAP_MSG *erap = &msg->msg.erap;

	if (ipc_id != IPC_ERAP || erap->msgtype != REALTIME_EXTRA)
		return IRQ_NONE;
	if (erap->param.raw.params[0] == DSM_WRITE_CB) {
		WRITE_ONCE(dsm_write_done, true);
		wake_up_interruptible(&dsm_wq_write);
	}
	return IRQ_HANDLED;
}

static const struct snd_soc_component_driver abox_adaptation_cmpnt = {
	.name = "samsung-abox-adaptation",
};

static int samsung_abox_adaptation_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret;

	if (!dev->parent || !dev_get_drvdata(dev->parent))
		return -EPROBE_DEFER;
	ret = abox_register_irq_handler(dev->parent, IPC_ERAP,
					abox_adaptation_irq_handler, pdev);
	if (ret < 0)
		return ret;
	dsm_abox_dev = dev->parent;
	return devm_snd_soc_register_component(dev, &abox_adaptation_cmpnt,
					       NULL, 0);
}

static const struct of_device_id samsung_abox_adaptation_match[] = {
	{ .compatible = "samsung,exynos8895-abox-adaptation" },
	{},
};
MODULE_DEVICE_TABLE(of, samsung_abox_adaptation_match);

static struct platform_driver samsung_abox_adaptation_driver = {
	.probe  = samsung_abox_adaptation_probe,
	.driver = {
		.name = "samsung-abox-adaptation",
		.of_match_table = samsung_abox_adaptation_match,
	},
};

module_platform_driver(samsung_abox_adaptation_driver);

MODULE_AUTHOR("Gyeongtaek Lee, <gt82.lee@samsung.com>");
MODULE_DESCRIPTION("Samsung ASoC A-Box Adaptation Driver");
MODULE_ALIAS("platform:samsung-abox-adaptation");
MODULE_LICENSE("GPL");
