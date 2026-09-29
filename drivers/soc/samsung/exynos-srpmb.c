// SPDX-License-Identifier: GPL-2.0-only
/*
 * Exynos secure RPMB proxy (greatlte, spec L6T 5.9 "6T-R").
 *
 * The secure world (SecDrv / KEYMAN drivers of the Kinibi TEE) writes an RPMB
 * request into a shared buffer and raises SPI 69; this driver forwards it as
 * SECURITY PROTOCOL OUT/IN commands to the UFS RPMB well-known LU and writes the
 * result back. It never issues an RPMB access of its own.
 * Stock: S:drivers/scsi/scsi_srpmb.c with srpmb_scsi_ioctl() of
 * S:drivers/scsi/scsi_ioctl.c:140-383 inlined; stock defects fixed as L6T D-15.
 */

#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeup.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>
#include <scsi/scsi.h>
#include <scsi/scsi_device.h>
#include <scsi/scsi_eh.h>
#include <scsi/scsi_proto.h>
#include <ufs/ufs.h>
#include <ufs/ufshcd.h>
#include <soc/samsung/exynos8895-smc.h>

/* S:drivers/scsi/scsi_srpmb.h:16-45 verbatim (misspelling COUTNER kept so the diff to stock stays readable) */
#define GET_WRITE_COUNTER			1
#define WRITE_DATA				2
#define READ_DATA				3
#define RESULT_READ_REQ				0x0005
#define RPMB_PACKET_SIZE			512
#define WRITE_COUTNER_DATA_LEN_ERROR		0x601
#define WRITE_COUTNER_SECURITY_OUT_ERROR	0x602
#define WRITE_COUTNER_SECURITY_IN_ERROR		0x603
#define WRITE_DATA_LEN_ERROR			0x604
#define WRITE_DATA_SECURITY_OUT_ERROR		0x605
#define WRITE_DATA_RESULT_SECURITY_OUT_ERROR	0x606
#define WRITE_DATA_SECURITY_IN_ERROR		0x607
#define READ_LEN_ERROR				0x608
#define READ_DATA_SECURITY_OUT_ERROR		0x609
#define READ_DATA_SECURITY_IN_ERROR		0x60A
#define PASS_STATUS				0xBABA
#define RPMB_BUF_MAX_SIZE			(32 * 1024)
/* written into req->cmd as stock did (the SWd shares the struct): S:include/scsi/scsi_ioctl.h:10-11 */
#define SCSI_IOCTL_SECURITY_PROTOCOL_IN		7
#define SCSI_IOCTL_SECURITY_PROTOCOL_OUT	8
#define SECU_PROT_UFS				0xEC	/* S:include/scsi/scsi_proto.h:88 */
#define SECU_PROT_SPEC_CERT_DATA		0x0001	/* S:include/scsi/scsi_proto.h:89 */
#define SRPMB_RETRIES				5	/* NORMAL_RETRIES, S:drivers/scsi/scsi_ioctl.c:29 */

/*
 * = ufshcd_upiu_wlun_to_scsi_wlun(UFS_UPIU_RPMB_WLUN), which is static in
 * A:drivers/ufs/core/ufshcd.c:2999-3002 (UFS_UPIU_RPMB_WLUN 0xC4 A:include/ufs/ufs.h:67,
 * UFS_UPIU_WLUN_ID :46, SCSI_W_LUN_BASE 0xc100 A:include/scsi/scsi.h:57)
 */
#define SRPMB_RPMB_SCSI_WLUN	((UFS_UPIU_RPMB_WLUN & ~UFS_UPIU_WLUN_ID) | SCSI_W_LUN_BASE)	/* 0xc144 */

struct rpmb_req {			/* Rpmb_Req, S:include/scsi/scsi_ioctl.h:50-58; shared with the SWd */
	u32 cmd;
	u32 status_flag;		/* stock "volatile u32"; written with srpmb_set_status() */
	u32 type;
	u32 data_len;
	u32 inlen;
	u32 outlen;
	u8 rpmb_data[];
};
static_assert(sizeof(struct rpmb_req) == 24);

struct rpmb_packet {			/* S:drivers/scsi/scsi_srpmb.h:58-68 */
	u16 request;
	u16 result;
	u16 count;
	u16 address;
	u32 write_counter;
	u8 nonce[16];
	u8 data[256];
	u8 Key_MAC[32];
	u8 stuff[196];
};
static_assert(sizeof(struct rpmb_packet) == RPMB_PACKET_SIZE);

struct srpmb_ctx {
	struct device *dev;
	struct platform_device *ufs_pdev;
	struct scsi_device *sdev;	/* W-LU, referenced once found (srpmb_get_sdev) */
	struct rpmb_req *req;		/* WSM */
	dma_addr_t pa;
	struct work_struct work;
	struct workqueue_struct *wq;
	struct notifier_block pm_nb;
	struct wakeup_source *ws;
};

/*
 * L6T D-16: the UFS host sets its drvdata only in ufshcd_init()
 * (A:drivers/ufs/core/ufshcd.c:10974) and adds the W-LUs from its async scan
 * (ufshcd_scsi_add_wlus(), :8450-8466), so both are resolved here, in the
 * (ordered) worker, until they exist. scsi_device_lookup() takes a reference
 * (A:drivers/scsi/scsi.c:996-1008) that is kept: there is no unbind.
 */
static struct scsi_device *srpmb_get_sdev(struct srpmb_ctx *ctx)
{
	struct ufs_hba *hba;

	if (ctx->sdev)
		return ctx->sdev;
	hba = platform_get_drvdata(ctx->ufs_pdev);	/* NULL until ufshcd_init() ran */
	if (!hba)
		return NULL;
	ctx->sdev = scsi_device_lookup(hba->host, 0, 0, SRPMB_RPMB_SCSI_WLUN);
	return ctx->sdev;				/* NULL until the async scan added the W-LU */
}

/*
 * One SECURITY PROTOCOL IN/OUT (S:drivers/scsi/scsi_ioctl.c:140-198, 333-383).
 * CDB length: 6.18 derives it from the opcode (COMMAND_SIZE(0xA2/0xB5) = 12,
 * A:drivers/scsi/scsi_lib.c:324), stock's explicit 12. A pending unit attention
 * (stock cleared it once per reset with SCSI_UFS_REQUEST_SENSE, :356-358) is
 * retried through the failure table (pattern of A:drivers/scsi/sd.c:1944-1960).
 */
static int srpmb_sec_cmd(struct device *dev, struct scsi_device *sdev, bool in, void *buf, u32 len)
{
	u8 cdb[12] = { in ? SECURITY_PROTOCOL_IN : SECURITY_PROTOCOL_OUT, SECU_PROT_UFS };	/* 0xEC */
	struct scsi_sense_hdr sshdr = {};	/* filled only once the command ran: A:drivers/scsi/scsi_lib.c:354-356 */
	struct scsi_failure failure_defs[] = {
		{ .sense = UNIT_ATTENTION, .asc = SCMD_FAILURE_ASC_ANY, .ascq = SCMD_FAILURE_ASCQ_ANY,
		  .allowed = 5, .result = SAM_STAT_CHECK_CONDITION },
		{}
	};
	struct scsi_failures failures = { .failure_definitions = failure_defs };
	const struct scsi_exec_args args = { .sshdr = &sshdr, .failures = &failures };
	void *bounce;
	int ret;

	if (!scsi_block_when_processing_errors(sdev))	/* stock S:drivers/scsi/scsi_ioctl.c:353-354 */
		return -ENODEV;
	bounce = kzalloc(len, GFP_KERNEL);	/* stock bounce, S:drivers/scsi/scsi_ioctl.c:152-168 */
	if (!bounce)
		return -ENOMEM;
	put_unaligned_be16(SECU_PROT_SPEC_CERT_DATA, &cdb[2]);	/* 0x0001 */
	put_unaligned_be32(len, &cdb[6]);
	if (!in)
		memcpy(bounce, buf, len);
	ret = scsi_execute_cmd(sdev, cdb, in ? REQ_OP_DRV_IN : REQ_OP_DRV_OUT, bounce, len,
			       START_STOP_TIMEOUT, SRPMB_RETRIES, &args);
	if (!ret && in)
		memcpy(buf, bounce, len);
	if (ret)
		dev_err(dev, "srpmb: %s len %u result 0x%x sense %x/%x/%x\n", in ? "IN" : "OUT", len, ret,
			sshdr.sense_key, sshdr.asc, sshdr.ascq);
	kfree(bounce);
	return ret ? -EIO : 0;
}

static void swap_packet(u8 *p, u8 *d)	/* S:drivers/scsi/scsi_srpmb.c:49-54 */
{
	int i;

	for (i = 0; i < RPMB_PACKET_SIZE; i++)
		d[i] = p[RPMB_PACKET_SIZE - 1 - i];
}

static void srpmb_set_status(struct rpmb_req *req, u32 status)
{
	wmb();				/* frame data before the status the SWd waits for */
	WRITE_ONCE(req->status_flag, status);
}

/* stock: req->cmd/inlen/outlen + srpmb_scsi_ioctl(), S:drivers/scsi/scsi_ioctl.c:333-383 */
static int srpmb_xfer(struct srpmb_ctx *ctx, bool in, u32 len)
{
	struct rpmb_req *req = ctx->req;
	struct scsi_device *sdev = srpmb_get_sdev(ctx);

	if (!sdev) {
		dev_err(ctx->dev, "srpmb: no RPMB W-LU\n");
		return -ENXIO;		/* stock "sdev empty", S:drivers/scsi/scsi_ioctl.c:341-344 */
	}
	if (in) {
		req->cmd = SCSI_IOCTL_SECURITY_PROTOCOL_IN;
		req->inlen = len;
	} else {
		req->cmd = SCSI_IOCTL_SECURITY_PROTOCOL_OUT;
		req->outlen = len;
	}
	return srpmb_sec_cmd(ctx->dev, sdev, in, req->rpmb_data, len);
}

static void srpmb_worker(struct work_struct *work)	/* S:drivers/scsi/scsi_srpmb.c:56-205 */
{
	struct srpmb_ctx *ctx = container_of(work, struct srpmb_ctx, work);
	struct rpmb_req *req = ctx->req;
	u32 type = READ_ONCE(req->type), data_len = READ_ONCE(req->data_len);
	struct rpmb_packet packet;
	int ret;

	__pm_stay_awake(ctx->ws);			/* stock wake_lock(), :81 */
	switch (type) {
	case GET_WRITE_COUNTER:				/* :84-113 */
		if (data_len != RPMB_PACKET_SIZE) {
			srpmb_set_status(req, WRITE_COUTNER_DATA_LEN_ERROR);
			dev_err(ctx->dev, "data len is invalid\n");
			break;
		}
		ret = srpmb_xfer(ctx, false, RPMB_PACKET_SIZE);
		if (ret < 0) {
			srpmb_set_status(req, WRITE_COUTNER_SECURITY_OUT_ERROR);
			dev_err(ctx->dev, "ioctl read_counter error: %x\n", ret);
			break;
		}
		memset(req->rpmb_data, 0x0, data_len);
		ret = srpmb_xfer(ctx, true, data_len);
		if (ret < 0) {
			srpmb_set_status(req, WRITE_COUTNER_SECURITY_IN_ERROR);
			dev_err(ctx->dev, "ioctl error : %x\n", ret);
			break;
		}
		srpmb_set_status(req, PASS_STATUS);
		break;
	case WRITE_DATA:				/* :114-167 */
		if (data_len < RPMB_PACKET_SIZE || data_len > RPMB_PACKET_SIZE * 64) {
			srpmb_set_status(req, WRITE_DATA_LEN_ERROR);
			dev_err(ctx->dev, "data len is invalid\n");
			break;
		}
		ret = srpmb_xfer(ctx, false, data_len);
		if (ret < 0) {
			srpmb_set_status(req, WRITE_DATA_SECURITY_OUT_ERROR);
			dev_err(ctx->dev, "ioctl write data error: %x\n", ret);
			break;
		}
		memset(req->rpmb_data, 0x0, data_len);
		memset(&packet, 0x0, RPMB_PACKET_SIZE);
		packet.request = RESULT_READ_REQ;
		swap_packet((u8 *)&packet, req->rpmb_data);
		ret = srpmb_xfer(ctx, false, RPMB_PACKET_SIZE);
		if (ret < 0) {
			srpmb_set_status(req, WRITE_DATA_RESULT_SECURITY_OUT_ERROR);
			dev_err(ctx->dev, "ioctl write_data result error: %x\n", ret);
			break;
		}
		memset(req->rpmb_data, 0x0, data_len);
		ret = srpmb_xfer(ctx, true, RPMB_PACKET_SIZE);
		if (ret < 0) {
			srpmb_set_status(req, WRITE_DATA_SECURITY_IN_ERROR);
			dev_err(ctx->dev, "ioctl write_data result error: %x\n", ret);
			break;
		}
		swap_packet(req->rpmb_data, (u8 *)&packet);
		if (packet.result == 0) {
			srpmb_set_status(req, PASS_STATUS);
		} else {
			srpmb_set_status(req, packet.result);
			dev_err(ctx->dev, "packet result error: %x\n", packet.result);
		}
		break;
	case READ_DATA:					/* :168-199 */
		if (data_len < RPMB_PACKET_SIZE || data_len > RPMB_PACKET_SIZE * 64) {
			srpmb_set_status(req, READ_LEN_ERROR);
			dev_err(ctx->dev, "data len is invalid\n");
			break;
		}
		ret = srpmb_xfer(ctx, false, RPMB_PACKET_SIZE);
		if (ret < 0) {
			srpmb_set_status(req, READ_DATA_SECURITY_OUT_ERROR);
			dev_err(ctx->dev, "ioctl read data error: %x\n", ret);
			break;
		}
		memset(req->rpmb_data, 0x0, data_len);
		ret = srpmb_xfer(ctx, true, data_len);
		if (ret < 0) {
			srpmb_set_status(req, READ_DATA_SECURITY_IN_ERROR);
			dev_err(ctx->dev, "ioctl result read data error : %x\n", ret);
			break;
		}
		srpmb_set_status(req, PASS_STATUS);
		break;
	default:
		dev_err(ctx->dev, "invalid requset type : %x\n", type);	/* :201, status left as is (stock) */
	}
	__pm_relax(ctx->ws);				/* stock wake_unlock(), :204 */
}

static irqreturn_t srpmb_irq_handler(int irq, void *arg)	/* :226-233 */
{
	struct srpmb_ctx *ctx = arg;

	queue_work(ctx->wq, &ctx->work);
	return IRQ_HANDLED;
}

static int srpmb_pm_notifier(struct notifier_block *nb, unsigned long event, void *unused)	/* :207-224 */
{
	struct srpmb_ctx *ctx = container_of(nb, struct srpmb_ctx, pm_nb);

	if (event == PM_SUSPEND_PREPARE)
		flush_workqueue(ctx->wq);
	return NOTIFY_DONE;
}

static int srpmb_probe(struct platform_device *pdev)	/* stock init_wsm(), :235-324 */
{
	struct device *dev = &pdev->dev;
	struct device_node *np_ufs;
	struct srpmb_ctx *ctx;
	irq_hw_number_t hwirq;
	int irq, ret;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	ctx->dev = dev;

	/* new in 6.18: the UFS host whose RPMB W-LU serves the requests (L6T 4, D-16) */
	np_ufs = of_parse_phandle(dev->of_node, "samsung,ufs", 0);
	if (!np_ufs)
		return dev_err_probe(dev, -EINVAL, "srpmb: no samsung,ufs\n");
	ctx->ufs_pdev = of_find_device_by_node(np_ufs);	/* takes a device reference */
	of_node_put(np_ufs);
	if (!ctx->ufs_pdev)
		return -EPROBE_DEFER;

	/* sizeof(Rpmb_Req) + RPMB_BUF_MAX_SIZE, :249-251 */
	ctx->req = dmam_alloc_coherent(dev, sizeof(struct rpmb_req) + RPMB_BUF_MAX_SIZE, &ctx->pa, GFP_KERNEL);
	if (!ctx->req) {
		dev_err(dev, "wsm dma alloc failed\n");		/* :304 */
		ret = -ENOMEM;
		goto put_ufs;
	}
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		ret = irq;
		goto put_ufs;
	}
	hwirq = irqd_to_hwirq(irq_get_irq_data(irq));	/* = 101 (SPI 69), :261-268 */
	/*
	 * stock alloc_workqueue(WQ_MEM_RECLAIM | WQ_UNBOUND, 1) (:272-273) was ordered
	 * implicitly on 4.4; the single shared request buffer needs it ordered (F46)
	 */
	ctx->wq = alloc_ordered_workqueue("srpmb_wq", WQ_MEM_RECLAIM);
	if (!ctx->wq) {
		ret = -ENOMEM;
		goto put_ufs;
	}
	INIT_WORK(&ctx->work, srpmb_worker);		/* before the IRQ (F46; stock :301 after the SMC) */
	ctx->ws = wakeup_source_register(dev, "srpmb");	/* NULL is tolerated by __pm_stay_awake/__pm_relax */
	ret = request_irq(irq, srpmb_irq_handler, IRQF_TRIGGER_RISING, "ufs-srpmb", ctx);	/* :280-281 */
	if (ret) {
		dev_err(dev, "request irq failed: %x\n", ret);	/* :283 */
		goto destroy_wq;
	}
	ctx->pm_nb.notifier_call = srpmb_pm_notifier;
	ret = register_pm_notifier(&ctx->pm_nb);	/* :287-292 */
	if (ret)
		goto free_irq;
	ret = (int)exynos8895_smc(SMC_SRPMB_WSM, ctx->pa, hwirq, 0);	/* :294; stock exynos_smc() returns int */
	if (ret) {
		dev_err(dev, "wsm smc init failed: %x\n", ret);	/* :296 */
		ret = -EIO;
		goto unreg_pm;
	}
	platform_set_drvdata(pdev, ctx);
	dev_info(dev, "srpmb: wsm %pad hwirq %lu\n", &ctx->pa, hwirq);	/* T6-6 */
	return 0;

unreg_pm:
	unregister_pm_notifier(&ctx->pm_nb);
free_irq:
	free_irq(irq, ctx);
destroy_wq:
	wakeup_source_unregister(ctx->ws);
	destroy_workqueue(ctx->wq);
put_ufs:
	put_device(&ctx->ufs_pdev->dev);
	return ret;
}

static const struct of_device_id srpmb_of_match[] = {
	{ .compatible = "samsung,ufs-srpmb" },		/* S:drivers/scsi/scsi_srpmb.c:26 */
	{ }
};

static struct platform_driver srpmb_driver = {
	.probe = srpmb_probe,
	.driver = {
		.name = "exynos-ufs-srpmb",			/* S:drivers/scsi/scsi_srpmb.c:341 */
		.of_match_table = srpmb_of_match,
		.suppress_bind_attrs = true,		/* no unbind: the SWd keeps the WSM address for ever */
	},
};
builtin_platform_driver(srpmb_driver);
