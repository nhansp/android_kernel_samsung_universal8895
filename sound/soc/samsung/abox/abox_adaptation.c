// SPDX-License-Identifier: GPL-2.0-only
/*
 * sound/soc/samsung/abox/abox_adaptation.c - Samsung ABOX adaptation driver
 *
 * Copyright (c) 2016 Samsung Electronics Co. Ltd.
 *
 * Transplanted from S:sound/soc/samsung/abox/abox_adaptation.c (Samsung
 * N950NKSU5DZE2 OSRC 4.4.111), spec L6E 5.7.6: the empty platform component
 * and the DSM calibration write of the ERAP bridge.
 */
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <sound/soc.h>
#include <sound/samsung/abox.h>
#include <sound/samsung/abox_dsm.h>
#include "abox.h"

#define DSM_ERAP_TIMEOUT_MS		130		/* TIMEOUT_MS, S:.../abox_adaptation.c:20 */
#define DSM_PARAM_COUNT			185		/* PARAM_DSM_5_0_MAX, S:include/sound/maxim_dsm.h:44 */
#define DSM_WRITE_CB			0xDAAD		/* PARAM_DSM_5_0_ABOX_WRITE_CB, :45 */
#define DSM_FLAG_WRITE_RDC_CAL_ONLY	0xca00ca00	/* FLAG_WRITE_RDC_CAL_ONLY, :33 */
#define DSM_IDX_RDC_AT_ROOMTEMP		6		/* DSM_API_SETGET_RDC_AT_ROOMTEMP, :55 */
#define DSM_IDX_COLDTEMP		8		/* DSM_API_SETGET_COLDTEMP, :56 */
#define DSM_IDX_WRITE_FLAG		63		/* DSM_API_SETGET_WRITE_FLAG, :62 */

static struct device *dsm_abox_dev;		/* = pdev->dev.parent (the ABOX device); stock phandle "abox" :202-212 */
static DECLARE_WAIT_QUEUE_HEAD(dsm_wq_write);	/* stock wq_write :33 */
static bool dsm_write_done;			/* stock abox_ipc_irq_write_avail :37 */
static DEFINE_MUTEX(dsm_lock);			/* new: serialises two amps' pushes; one amp on this board */

/* = stock maxdsm_set_rdc_temp() TYPE_C (S:sound/soc/codecs/maxim_dsm.c:1839-1843) + maxdsm_write_all() (:880-881)
 *   + maxim_dsm_write(&param[0], 0, 185) (abox_adaptation.c:77-110), with the kzalloc'ed stock array (maxim_dsm.c:2134-2136)
 *   represented by the zeroed message: byte-identical to stock's first push after boot (F-59). */
int abox_dsm_set_rdc_temp(int rdc, int temp)
{
	ABOX_IPC_MSG msg = { 0 };
	struct IPC_ERAP_MSG *erap = &msg.msg.erap;
	unsigned int *param = &erap->param.raw.params[3];	/* params[188] (S:include/sound/samsung/abox_ipc.h:156-157) */
	int ret;

	if (!dsm_abox_dev)
		return -ENODEV;
	mutex_lock(&dsm_lock);
	msg.ipcid = IPC_ERAP;
	erap->msgtype = REALTIME_EXTRA;
	erap->param.raw.params[0] = 1;			/* write, stock :85 */
	erap->param.raw.params[1] = 0;			/* offset, :86 */
	erap->param.raw.params[2] = DSM_PARAM_COUNT;	/* size, :87 */
	param[DSM_IDX_WRITE_FLAG] = DSM_FLAG_WRITE_RDC_CAL_ONLY;
	param[DSM_IDX_RDC_AT_ROOMTEMP] = rdc << 8;
	param[DSM_IDX_COLDTEMP] = temp << 19;
	WRITE_ONCE(dsm_write_done, false);
	ret = abox_start_ipc_transaction(dsm_abox_dev, IPC_ERAP, &msg, sizeof(msg), 1, 1);	/* stock :98 */
	if (ret < 0)
		pr_err("%s: abox_start_ipc_transaction is failed\n", __func__);		/* :100 */
	else if (!wait_event_interruptible_timeout(dsm_wq_write, READ_ONCE(dsm_write_done),
						   msecs_to_jiffies(DSM_ERAP_TIMEOUT_MS)))
		pr_err("%s: wait_event timeout\n", __func__);				/* :106-107 */
	else
		pr_info("%s: rdc 0x%08x temp %d applied\n", __func__, rdc, temp);	/* new (8.3) */
	mutex_unlock(&dsm_lock);
	return ret < 0 ? ret : 0;
}
EXPORT_SYMBOL_GPL(abox_dsm_set_rdc_temp);

/* stock abox_adaptation_irq_handler() (:112-172), write-completion branch only (:157-165) */
static irqreturn_t abox_adaptation_irq_handler(int ipc_id, void *dev_id, ABOX_IPC_MSG *msg)
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

static const struct snd_soc_component_driver abox_adaptation_cmpnt = { .name = "samsung-abox-adaptation" };

static int samsung_abox_adaptation_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret;

	if (!dev->parent || !dev_get_drvdata(dev->parent))
		return -EPROBE_DEFER;				/* stock :202-210 deferred on the "abox" phandle */
	ret = abox_register_irq_handler(dev->parent, IPC_ERAP, abox_adaptation_irq_handler, pdev);	/* :214-215 */
	if (ret < 0)
		return ret;
	dsm_abox_dev = dev->parent;
	dev_info(dev, "%s\n", __func__);			/* stock :217 */
	return devm_snd_soc_register_component(dev, &abox_adaptation_cmpnt, NULL, 0);
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
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(samsung_abox_adaptation_match),
	},
};

module_platform_driver(samsung_abox_adaptation_driver);

MODULE_AUTHOR("Gyeongtaek Lee, <gt82.lee@samsung.com>");
MODULE_DESCRIPTION("Samsung ASoC A-Box Adaptation Driver");
MODULE_ALIAS("platform:samsung-abox-adaptation");
MODULE_LICENSE("GPL");
