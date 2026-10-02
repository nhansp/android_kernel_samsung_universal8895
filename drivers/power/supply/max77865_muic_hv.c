// SPDX-License-Identifier: GPL-2.0
/*
 * MAX77865 MUIC high-voltage (AFC/QC) module (Samsung Galaxy Note 8, greatlte).
 * Port of Samsung's drivers/muic/universal/muic_hv_max77865.c (with its
 * muic_hv_max77865.h and muic_hv.h) to the auxiliary device that the MAX77865
 * MUIC driver creates.
 *
 * Keeps the stock checklist tables and transitions minus every
 * CONFIG_MUIC_HV_12V row (not set in stock's defconfig) and minus the factory
 * branch, register bits only from stock's muic_hv_max77865.h, and the stock exit
 * conditions (AFC_CHARGER_WA_PING 5, MPing miss 2000 ms, QC settle 300 ms,
 * handshake RX bytes <= 16, stop at 0).
 * Changed behaviour: TA_TO_PREPARE records "5V ready" and notifies the
 * charger; 9 V starts only through max77865_hv_set_voltage(9000) from the charger;
 * the QC branch runs only with CONFIG_CHARGER_MAX77865_QC.
 *
 * Lock order: hv->lock may nest over the MUIC lock (hv -> MUIC
 * only): set_voltage() and the IRQ work call the cc_rp/water/chgdet accessors
 * with hv->lock held. The reverse never happens: the MUIC base posts its
 * notifier outside its lock and its hv_irq entry (called with the MUIC lock
 * held) only records the interrupt and queues work.
 * Register sets are disjoint (HV/CONTROL2_BC here,
 * CONTROL1/COM/CCDET in the base), with regmap-level atomicity per access.
 * The notifier to the charger carries no lock.
 */

#include <linux/auxiliary_bus.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kstrtox.h>
#include <linux/mfd/max77865-private.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/string.h>
#include <linux/workqueue.h>
#include <linux/usb/max77865-muic.h>
#include <linux/power/max77865_muic_hv.h>

/* AFC IRQ demux (stock drivers/muic/universal/muic_task_max77865.c) */
#define HV_IRQ_VDNMON	1
#define HV_IRQ_MRXRDY	2
#define HV_IRQ_VBADC	3
#define HV_IRQ_MPNACK	4
#define HV_IRQ_DONTCARE	0xf

/* BC CONTROL2 DPDNMAN states (stock muic_hv_max77865.h) */
#define HV_DPDNVDEN_DISABLE	0x00
#define HV_DPDNVDEN_ENABLE	(0x1 << 4)	/* BC_CONTROL2_DPDNMAN_SHIFT */
#define HV_DPDNVDEN_DONTCARE	0xff

/* STATUS2_BC DNVDATREF states */
#define HV_VDNMON_LOW		0x00
#define HV_VDNMON_HIGH		(0x1 << 0)	/* BC_STATUS2_DNVDATREF_SHIFT */
#define HV_VDNMON_DONTCARE	0xff

/* GP STATUS VBADC windows (HVH vbadc_t) */
#define HV_VBADC_VBDET		0x00
#define HV_VBADC_4V_5V		0x01
#define HV_VBADC_5V_6V		0x02
#define HV_VBADC_6V_7V		0x03
#define HV_VBADC_7V_8V		0x04
#define HV_VBADC_8V_9V		0x05
#define HV_VBADC_9V_10V		0x06
#define HV_VBADC_10V_11V	0x07
#define HV_VBADC_11V_12V	0x08
#define HV_VBADC_12V_13V	0x09
#define HV_VBADC_13V		0x0a
#define HV_VBADC_QC_5V		0xeb
#define HV_VBADC_QC_9V		0xec
#define HV_VBADC_AFC_5V		0xfa
#define HV_VBADC_AFC_9V		0xfb
#define HV_VBADC_AFC_ERR_V	0xfc
#define HV_VBADC_AFC_ERR_V_NOT_0 0xfd
#define HV_VBADC_ANY		0xfe
#define HV_VBADC_DONTCARE	0xff

/* HVTXBYTE codes (stock muic_hv.h) */
#define HV_TX_5V		0x08	/* MUIC_HV_5V */
#define HV_TX_9V		0x46	/* MUIC_HV_9V */

/* attached devices, stock numbers (Samsung's include/linux/muic/muic.h) */
enum hv_attached_dev {
	HV_DEV_NONE		= 0,	/* ATTACHED_DEV_NONE_MUIC */
	HV_DEV_TA		= 4,	/* ATTACHED_DEV_TA_MUIC */
	HV_DEV_AFC_PREPARE	= 33,
	HV_DEV_AFC_PREPARE_DUPLI = 34,
	HV_DEV_AFC_5V		= 35,
	HV_DEV_AFC_5V_DUPLI	= 36,
	HV_DEV_AFC_9V		= 37,
	HV_DEV_AFC_9V_DUPLI	= 38,
	HV_DEV_AFC_ERR_V	= 41,
	HV_DEV_AFC_ERR_V_DUPLI	= 42,
	HV_DEV_QC_PREPARE	= 43,
	HV_DEV_QC_5V		= 44,
	HV_DEV_QC_9V		= 46,
	HV_DEV_NUM		= 64,	/* above ATTACHED_DEV_NUM */
};

/* CCIC Rp values (Samsung's include/linux/ccic/s2mm005.h) */
#define HV_RP_56K		1	/* Rp_56K */
#define HV_RP_ABNORMAL		4	/* Rp_Abnormal */

/* afc_disable boot value (Samsung's drivers/muic/muic-core.c, muic.h) */
#define HV_AFC_DISABLE_VAL	0x31	/* CH_MODE_AFC_DISABLE_VAL, char '1' */

/* stock exit conditions */
#define HV_AFC_WA_PING		5	/* AFC_CHARGER_WA_PING */
#define HV_MPING_MISS_MS	2000	/* MPING_MISS_WA_TIME */
#define HV_QC_SETTLE_MS		300
#define HV_HVRX_MAX		16	/* HVRXBYTE_MAX */
#define HV_INIT_DETECT_MS	30
#define HV_CHGDET_RERUN_MS	80

/* QC support of this board (muic,qc-hv = [09] = HV_SUPPORT_QC_9V) */
#define HV_QC_9V		9

enum hv_func {
	FUNC_TA_TO_PREPARE = 0,
	FUNC_PREPARE_TO_PREPARE_DUPLI,
	FUNC_PREPARE_TO_AFC_5V,
	FUNC_PREPARE_TO_QC_PREPARE,
	FUNC_PREPARE_DUPLI_TO_PREPARE_DUPLI,
	FUNC_PREPARE_DUPLI_TO_AFC_5V,
	FUNC_PREPARE_DUPLI_TO_AFC_ERR_V,
	FUNC_PREPARE_DUPLI_TO_AFC_9V,
	FUNC_PREPARE_DUPLI_TO_QC_PREPARE,
	FUNC_AFC_5V_TO_AFC_5V_DUPLI,
	FUNC_AFC_5V_TO_AFC_ERR_V,
	FUNC_AFC_5V_TO_AFC_9V,
	FUNC_AFC_5V_TO_QC_PREPARE,
	FUNC_AFC_5V_DUPLI_TO_AFC_5V_DUPLI,
	FUNC_AFC_5V_DUPLI_TO_AFC_ERR_V,
	FUNC_AFC_5V_DUPLI_TO_AFC_9V,
	FUNC_AFC_5V_DUPLI_TO_QC_PREPARE,
	FUNC_AFC_ERR_V_TO_AFC_ERR_V_DUPLI,
	FUNC_AFC_ERR_V_TO_AFC_5V,
	FUNC_AFC_ERR_V_TO_AFC_9V,
	FUNC_AFC_ERR_V_TO_QC_PREPARE,
	FUNC_AFC_ERR_V_DUPLI_TO_AFC_ERR_V_DUPLI,
	FUNC_AFC_ERR_V_DUPLI_TO_AFC_5V,
	FUNC_AFC_ERR_V_DUPLI_TO_AFC_9V,
	FUNC_AFC_ERR_V_DUPLI_TO_QC_PREPARE,
	FUNC_AFC_9V_TO_AFC_9V_DUPLI,
	FUNC_AFC_9V_TO_AFC_ERR_V,
	FUNC_AFC_9V_TO_AFC_5V,
	FUNC_AFC_9V_TO_QC_PREPARE,
	FUNC_AFC_9V_DUPLI_TO_AFC_ERR_V,
	FUNC_AFC_9V_DUPLI_TO_AFC_5V,
	FUNC_AFC_9V_DUPLI_TO_AFC_9V_DUPLI,
	FUNC_AFC_9V_DUPLI_TO_QC_PREPARE,
	FUNC_QC_PREPARE_TO_QC_5V,
	FUNC_QC_PREPARE_TO_QC_9V,
	FUNC_QC_5V_TO_QC_9V,
	FUNC_QC_9V_TO_QC_5V,
};

/* one checklist row (stock muic_afc_data_t) */
struct hv_step {
	int new_dev;
	const char *name;
	int irq;
	u8 dpdnvden;
	u8 vbadc;
	u8 vdnmon;
	int func;
	struct hv_step *next;
};

/* checklist minus every CONFIG_MUIC_HV_12V row */
static struct hv_step hv_ta_to_prepare = {
	.new_dev	= HV_DEV_AFC_PREPARE,
	.name		= "AFC charger Prepare",
	.irq		= HV_IRQ_VDNMON,
	.dpdnvden	= HV_DPDNVDEN_ENABLE,
	.vbadc		= HV_VBADC_DONTCARE,
	.vdnmon		= HV_VDNMON_LOW,
	.func		= FUNC_TA_TO_PREPARE,
	.next		= &hv_ta_to_prepare,
};

static struct hv_step hv_prepare_to_qc_prepare = {
	.new_dev	= HV_DEV_QC_PREPARE,
	.name		= "QC charger Prepare",
	.irq		= HV_IRQ_MPNACK,
	.dpdnvden	= HV_DPDNVDEN_DONTCARE,
	.vbadc		= HV_VBADC_DONTCARE,
	.vdnmon		= HV_VDNMON_DONTCARE,
	.func		= FUNC_PREPARE_TO_QC_PREPARE,
	.next		= &hv_prepare_to_qc_prepare,
};

static struct hv_step hv_prepare_to_afc_5v = {
	.new_dev	= HV_DEV_AFC_5V,
	.name		= "AFC charger 5V",
	.irq		= HV_IRQ_VBADC,
	.dpdnvden	= HV_DPDNVDEN_DISABLE,
	.vbadc		= HV_VBADC_AFC_5V,
	.vdnmon		= HV_VDNMON_DONTCARE,
	.func		= FUNC_PREPARE_TO_AFC_5V,
	.next		= &hv_prepare_to_qc_prepare,
};

static struct hv_step hv_prepare_to_prepare_dupli = {
	.new_dev	= HV_DEV_AFC_PREPARE_DUPLI,
	.name		= "AFC charger prepare (mrxrdy)",
	.irq		= HV_IRQ_MRXRDY,
	.dpdnvden	= HV_DPDNVDEN_DISABLE,
	.vbadc		= HV_VBADC_DONTCARE,
	.vdnmon		= HV_VDNMON_DONTCARE,
	.func		= FUNC_PREPARE_TO_PREPARE_DUPLI,
	.next		= &hv_prepare_to_afc_5v,
};

static struct hv_step hv_prepare_dupli_to_qc_prepare = {
	.new_dev	= HV_DEV_QC_PREPARE,
	.name		= "QC charger Prepare",
	.irq		= HV_IRQ_MPNACK,
	.dpdnvden	= HV_DPDNVDEN_DONTCARE,
	.vbadc		= HV_VBADC_DONTCARE,
	.vdnmon		= HV_VDNMON_DONTCARE,
	.func		= FUNC_PREPARE_DUPLI_TO_QC_PREPARE,
	.next		= &hv_prepare_dupli_to_qc_prepare,
};

static struct hv_step hv_prepare_dupli_to_prepare_dupli = {
	.new_dev	= HV_DEV_AFC_PREPARE_DUPLI,
	.name		= "AFC charger prepare (mrxrdy)",
	.irq		= HV_IRQ_MRXRDY,
	.dpdnvden	= HV_DPDNVDEN_DISABLE,
	.vbadc		= HV_VBADC_DONTCARE,
	.vdnmon		= HV_VDNMON_DONTCARE,
	.func		= FUNC_PREPARE_DUPLI_TO_PREPARE_DUPLI,
	.next		= &hv_prepare_dupli_to_qc_prepare,
};

static struct hv_step hv_prepare_dupli_to_afc_9v = {
	.new_dev	= HV_DEV_AFC_9V,
	.name		= "AFC charger 9V",
	.irq		= HV_IRQ_VBADC,
	.dpdnvden	= HV_DPDNVDEN_DISABLE,
	.vbadc		= HV_VBADC_AFC_9V,
	.vdnmon		= HV_VDNMON_DONTCARE,
	.func		= FUNC_PREPARE_DUPLI_TO_AFC_9V,
	.next		= &hv_prepare_dupli_to_prepare_dupli,
};

static struct hv_step hv_prepare_dupli_to_afc_err_v = {
	.new_dev	= HV_DEV_AFC_ERR_V,
	.name		= "AFC charger ERR V",
	.irq		= HV_IRQ_VBADC,
	.dpdnvden	= HV_DPDNVDEN_DISABLE,
	.vbadc		= HV_VBADC_AFC_ERR_V,
	.vdnmon		= HV_VDNMON_DONTCARE,
	.func		= FUNC_PREPARE_DUPLI_TO_AFC_ERR_V,
	.next		= &hv_prepare_dupli_to_afc_9v,
};

static struct hv_step hv_prepare_dupli_to_afc_5v = {
	.new_dev	= HV_DEV_AFC_5V,
	.name		= "AFC charger 5V",
	.irq		= HV_IRQ_VBADC,
	.dpdnvden	= HV_DPDNVDEN_DISABLE,
	.vbadc		= HV_VBADC_AFC_5V,
	.vdnmon		= HV_VDNMON_DONTCARE,
	.func		= FUNC_PREPARE_DUPLI_TO_AFC_5V,
	.next		= &hv_prepare_dupli_to_afc_err_v,
};

static struct hv_step hv_afc_5v_to_qc_prepare = {
	.new_dev	= HV_DEV_QC_PREPARE,
	.name		= "QC charger Prepare",
	.irq		= HV_IRQ_MPNACK,
	.dpdnvden	= HV_DPDNVDEN_DONTCARE,
	.vbadc		= HV_VBADC_DONTCARE,
	.vdnmon		= HV_VDNMON_DONTCARE,
	.func		= FUNC_AFC_5V_TO_QC_PREPARE,
	.next		= &hv_afc_5v_to_qc_prepare,
};

static struct hv_step hv_afc_5v_to_afc_9v = {
	.new_dev	= HV_DEV_AFC_9V,
	.name		= "AFC charger 9V",
	.irq		= HV_IRQ_VBADC,
	.dpdnvden	= HV_DPDNVDEN_DISABLE,
	.vbadc		= HV_VBADC_AFC_9V,
	.vdnmon		= HV_VDNMON_DONTCARE,
	.func		= FUNC_AFC_5V_TO_AFC_9V,
	.next		= &hv_afc_5v_to_qc_prepare,
};

static struct hv_step hv_afc_5v_to_afc_err_v = {
	.new_dev	= HV_DEV_AFC_ERR_V,
	.name		= "AFC charger ERR V",
	.irq		= HV_IRQ_VBADC,
	.dpdnvden	= HV_DPDNVDEN_DISABLE,
	.vbadc		= HV_VBADC_AFC_ERR_V,
	.vdnmon		= HV_VDNMON_DONTCARE,
	.func		= FUNC_AFC_5V_TO_AFC_ERR_V,
	.next		= &hv_afc_5v_to_afc_9v,
};

static struct hv_step hv_afc_5v_to_afc_5v_dupli = {
	.new_dev	= HV_DEV_AFC_5V_DUPLI,
	.name		= "AFC charger 5V (mrxrdy)",
	.irq		= HV_IRQ_MRXRDY,
	.dpdnvden	= HV_DPDNVDEN_DISABLE,
	.vbadc		= HV_VBADC_AFC_5V,
	.vdnmon		= HV_VDNMON_DONTCARE,
	.func		= FUNC_AFC_5V_TO_AFC_5V_DUPLI,
	.next		= &hv_afc_5v_to_afc_err_v,
};

static struct hv_step hv_afc_5v_dupli_to_qc_prepare = {
	.new_dev	= HV_DEV_QC_PREPARE,
	.name		= "QC charger Prepare",
	.irq		= HV_IRQ_MPNACK,
	.dpdnvden	= HV_DPDNVDEN_DONTCARE,
	.vbadc		= HV_VBADC_DONTCARE,
	.vdnmon		= HV_VDNMON_DONTCARE,
	.func		= FUNC_AFC_5V_DUPLI_TO_QC_PREPARE,
	.next		= &hv_afc_5v_dupli_to_qc_prepare,
};

static struct hv_step hv_afc_5v_dupli_to_afc_5v_dupli = {
	.new_dev	= HV_DEV_AFC_5V_DUPLI,
	.name		= "AFC charger 5V (mrxrdy)",
	.irq		= HV_IRQ_MRXRDY,
	.dpdnvden	= HV_DPDNVDEN_DISABLE,
	.vbadc		= HV_VBADC_AFC_5V,
	.vdnmon		= HV_VDNMON_DONTCARE,
	.func		= FUNC_AFC_5V_DUPLI_TO_AFC_5V_DUPLI,
	.next		= &hv_afc_5v_dupli_to_qc_prepare,
};

static struct hv_step hv_afc_5v_dupli_to_afc_9v = {
	.new_dev	= HV_DEV_AFC_9V,
	.name		= "AFC charger 9V",
	.irq		= HV_IRQ_VBADC,
	.dpdnvden	= HV_DPDNVDEN_DISABLE,
	.vbadc		= HV_VBADC_AFC_9V,
	.vdnmon		= HV_VDNMON_DONTCARE,
	.func		= FUNC_AFC_5V_DUPLI_TO_AFC_9V,
	.next		= &hv_afc_5v_dupli_to_afc_5v_dupli,
};

static struct hv_step hv_afc_5v_dupli_to_afc_err_v = {
	.new_dev	= HV_DEV_AFC_ERR_V,
	.name		= "AFC charger ERR V",
	.irq		= HV_IRQ_VBADC,
	.dpdnvden	= HV_DPDNVDEN_DISABLE,
	.vbadc		= HV_VBADC_AFC_ERR_V,
	.vdnmon		= HV_VDNMON_DONTCARE,
	.func		= FUNC_AFC_5V_DUPLI_TO_AFC_ERR_V,
	.next		= &hv_afc_5v_dupli_to_afc_9v,
};

static struct hv_step hv_afc_err_v_to_qc_prepare = {
	.new_dev	= HV_DEV_QC_PREPARE,
	.name		= "QC charger Prepare",
	.irq		= HV_IRQ_MPNACK,
	.dpdnvden	= HV_DPDNVDEN_DONTCARE,
	.vbadc		= HV_VBADC_DONTCARE,
	.vdnmon		= HV_VDNMON_DONTCARE,
	.func		= FUNC_AFC_ERR_V_TO_QC_PREPARE,
	.next		= &hv_afc_err_v_to_qc_prepare,
};

static struct hv_step hv_afc_err_v_to_afc_5v = {
	.new_dev	= HV_DEV_AFC_5V,
	.name		= "AFC charger 5V",
	.irq		= HV_IRQ_VBADC,
	.dpdnvden	= HV_DPDNVDEN_DISABLE,
	.vbadc		= HV_VBADC_AFC_5V,
	.vdnmon		= HV_VDNMON_DONTCARE,
	.func		= FUNC_AFC_ERR_V_TO_AFC_5V,
	.next		= &hv_afc_err_v_to_qc_prepare,
};

static struct hv_step hv_afc_err_v_to_afc_9v = {
	.new_dev	= HV_DEV_AFC_9V,
	.name		= "AFC charger 9V",
	.irq		= HV_IRQ_VBADC,
	.dpdnvden	= HV_DPDNVDEN_DISABLE,
	.vbadc		= HV_VBADC_AFC_9V,
	.vdnmon		= HV_VDNMON_DONTCARE,
	.func		= FUNC_AFC_ERR_V_TO_AFC_9V,
	.next		= &hv_afc_err_v_to_afc_5v,
};

static struct hv_step hv_afc_err_v_to_afc_err_v_dupli = {
	.new_dev	= HV_DEV_AFC_ERR_V_DUPLI,
	.name		= "AFC charger ERR V (mrxrdy)",
	.irq		= HV_IRQ_MRXRDY,
	.dpdnvden	= HV_DPDNVDEN_DISABLE,
	.vbadc		= HV_VBADC_AFC_ERR_V,
	.vdnmon		= HV_VDNMON_DONTCARE,
	.func		= FUNC_AFC_ERR_V_TO_AFC_ERR_V_DUPLI,
	.next		= &hv_afc_err_v_to_afc_9v,
};

static struct hv_step hv_afc_err_v_dupli_to_qc_prepare = {
	.new_dev	= HV_DEV_QC_PREPARE,
	.name		= "QC charger Prepare",
	.irq		= HV_IRQ_MPNACK,
	.dpdnvden	= HV_DPDNVDEN_DONTCARE,
	.vbadc		= HV_VBADC_DONTCARE,
	.vdnmon		= HV_VDNMON_DONTCARE,
	.func		= FUNC_AFC_ERR_V_DUPLI_TO_QC_PREPARE,
	.next		= &hv_afc_err_v_dupli_to_qc_prepare,
};

static struct hv_step hv_afc_err_v_dupli_to_afc_5v = {
	.new_dev	= HV_DEV_AFC_5V,
	.name		= "AFC charger 5V",
	.irq		= HV_IRQ_VBADC,
	.dpdnvden	= HV_DPDNVDEN_DISABLE,
	.vbadc		= HV_VBADC_AFC_5V,
	.vdnmon		= HV_VDNMON_DONTCARE,
	.func		= FUNC_AFC_ERR_V_DUPLI_TO_AFC_5V,
	.next		= &hv_afc_err_v_dupli_to_qc_prepare,
};

static struct hv_step hv_afc_err_v_dupli_to_afc_9v = {
	.new_dev	= HV_DEV_AFC_9V,
	.name		= "AFC charger 9V",
	.irq		= HV_IRQ_VBADC,
	.dpdnvden	= HV_DPDNVDEN_DISABLE,
	.vbadc		= HV_VBADC_AFC_9V,
	.vdnmon		= HV_VDNMON_DONTCARE,
	.func		= FUNC_AFC_ERR_V_DUPLI_TO_AFC_9V,
	.next		= &hv_afc_err_v_dupli_to_afc_5v,
};

static struct hv_step hv_afc_err_v_dupli_to_afc_err_v_dupli = {
	.new_dev	= HV_DEV_AFC_ERR_V_DUPLI,
	.name		= "AFC charger ERR V (mrxrdy)",
	.irq		= HV_IRQ_MRXRDY,
	.dpdnvden	= HV_DPDNVDEN_DISABLE,
	.vbadc		= HV_VBADC_AFC_ERR_V,
	.vdnmon		= HV_VDNMON_DONTCARE,
	.func		= FUNC_AFC_ERR_V_DUPLI_TO_AFC_ERR_V_DUPLI,
	.next		= &hv_afc_err_v_dupli_to_afc_9v,
};

static struct hv_step hv_afc_9v_to_qc_prepare = {
	.new_dev	= HV_DEV_QC_PREPARE,
	.name		= "QC charger Prepare",
	.irq		= HV_IRQ_MPNACK,
	.dpdnvden	= HV_DPDNVDEN_DONTCARE,
	.vbadc		= HV_VBADC_DONTCARE,
	.vdnmon		= HV_VDNMON_DONTCARE,
	.func		= FUNC_AFC_9V_TO_QC_PREPARE,
	.next		= &hv_afc_9v_to_qc_prepare,
};

static struct hv_step hv_afc_9v_to_afc_err_v = {
	.new_dev	= HV_DEV_AFC_ERR_V,
	.name		= "AFC charger ERR V",
	.irq		= HV_IRQ_VBADC,
	.dpdnvden	= HV_DPDNVDEN_DISABLE,
	.vbadc		= HV_VBADC_AFC_ERR_V,
	.vdnmon		= HV_VDNMON_DONTCARE,
	.func		= FUNC_AFC_9V_TO_AFC_ERR_V,
	.next		= &hv_afc_9v_to_qc_prepare,
};

static struct hv_step hv_afc_9v_to_afc_5v = {
	.new_dev	= HV_DEV_AFC_5V,
	.name		= "AFC charger 5V",
	.irq		= HV_IRQ_VBADC,
	.dpdnvden	= HV_DPDNVDEN_DISABLE,
	.vbadc		= HV_VBADC_AFC_5V,
	.vdnmon		= HV_VDNMON_DONTCARE,
	.func		= FUNC_AFC_9V_TO_AFC_5V,
	.next		= &hv_afc_9v_to_afc_err_v,
};

static struct hv_step hv_afc_9v_to_afc_9v_dupli = {
	.new_dev	= HV_DEV_AFC_9V_DUPLI,
	.name		= "AFC charger 9V (mrxrdy)",
	.irq		= HV_IRQ_MRXRDY,
	.dpdnvden	= HV_DPDNVDEN_DISABLE,
	.vbadc		= HV_VBADC_AFC_9V,
	.vdnmon		= HV_VDNMON_DONTCARE,
	.func		= FUNC_AFC_9V_TO_AFC_9V_DUPLI,
	.next		= &hv_afc_9v_to_afc_5v,
};

static struct hv_step hv_afc_9v_dupli_to_qc_prepare = {
	.new_dev	= HV_DEV_QC_PREPARE,
	.name		= "QC charger Prepare",
	.irq		= HV_IRQ_MPNACK,
	.dpdnvden	= HV_DPDNVDEN_DONTCARE,
	.vbadc		= HV_VBADC_DONTCARE,
	.vdnmon		= HV_VDNMON_DONTCARE,
	.func		= FUNC_AFC_9V_DUPLI_TO_QC_PREPARE,
	.next		= &hv_afc_9v_dupli_to_qc_prepare,
};

static struct hv_step hv_afc_9v_dupli_to_afc_err_v = {
	.new_dev	= HV_DEV_AFC_ERR_V,
	.name		= "AFC charger ERR V",
	.irq		= HV_IRQ_VBADC,
	.dpdnvden	= HV_DPDNVDEN_DISABLE,
	.vbadc		= HV_VBADC_AFC_ERR_V,
	.vdnmon		= HV_VDNMON_DONTCARE,
	.func		= FUNC_AFC_9V_DUPLI_TO_AFC_ERR_V,
	.next		= &hv_afc_9v_dupli_to_qc_prepare,
};

static struct hv_step hv_afc_9v_dupli_to_afc_5v = {
	.new_dev	= HV_DEV_AFC_5V,
	.name		= "AFC charger 5V",
	.irq		= HV_IRQ_VBADC,
	.dpdnvden	= HV_DPDNVDEN_DISABLE,
	.vbadc		= HV_VBADC_AFC_5V,
	.vdnmon		= HV_VDNMON_DONTCARE,
	.func		= FUNC_AFC_9V_DUPLI_TO_AFC_5V,
	.next		= &hv_afc_9v_dupli_to_afc_err_v,
};

static struct hv_step hv_afc_9v_dupli_to_afc_9v_dupli = {
	.new_dev	= HV_DEV_AFC_9V_DUPLI,
	.name		= "AFC charger 9V (mrxrdy)",
	.irq		= HV_IRQ_MRXRDY,
	.dpdnvden	= HV_DPDNVDEN_DISABLE,
	.vbadc		= HV_VBADC_AFC_9V,
	.vdnmon		= HV_VDNMON_DONTCARE,
	.func		= FUNC_AFC_9V_DUPLI_TO_AFC_9V_DUPLI,
	.next		= &hv_afc_9v_dupli_to_afc_5v,
};

static struct hv_step hv_qc_prepare_to_qc_9v = {
	.new_dev	= HV_DEV_QC_9V,
	.name		= "QC charger 9V",
	.irq		= HV_IRQ_VBADC,
	.dpdnvden	= HV_DPDNVDEN_ENABLE,
	.vbadc		= HV_VBADC_QC_9V,
	.vdnmon		= HV_VDNMON_DONTCARE,
	.func		= FUNC_QC_PREPARE_TO_QC_9V,
	.next		= &hv_qc_prepare_to_qc_9v,
};

static struct hv_step hv_qc_5v_to_qc_9v = {
	.new_dev	= HV_DEV_QC_9V,
	.name		= "QC charger 9V",
	.irq		= HV_IRQ_VBADC,
	.dpdnvden	= HV_DPDNVDEN_ENABLE,
	.vbadc		= HV_VBADC_QC_9V,
	.vdnmon		= HV_VDNMON_DONTCARE,
	.func		= FUNC_QC_5V_TO_QC_9V,
	.next		= &hv_qc_5v_to_qc_9v,
};

static struct hv_step hv_qc_9v_to_qc_5v = {
	.new_dev	= HV_DEV_QC_5V,
	.name		= "QC charger 5V",
	.irq		= HV_IRQ_VBADC,
	.dpdnvden	= HV_DPDNVDEN_ENABLE,
	.vbadc		= HV_VBADC_QC_5V,
	.vdnmon		= HV_VDNMON_DONTCARE,
	.func		= FUNC_QC_9V_TO_QC_5V,
	.next		= &hv_qc_9v_to_qc_5v,
};

static struct hv_step *hv_checklist[HV_DEV_NUM] = {
	[HV_DEV_TA]			= &hv_ta_to_prepare,
	[HV_DEV_AFC_PREPARE]		= &hv_prepare_to_prepare_dupli,
	[HV_DEV_AFC_PREPARE_DUPLI]	= &hv_prepare_dupli_to_afc_5v,
	[HV_DEV_AFC_5V]			= &hv_afc_5v_to_afc_5v_dupli,
	[HV_DEV_AFC_5V_DUPLI]		= &hv_afc_5v_dupli_to_afc_err_v,
	[HV_DEV_AFC_ERR_V]		= &hv_afc_err_v_to_afc_err_v_dupli,
	[HV_DEV_AFC_ERR_V_DUPLI]	= &hv_afc_err_v_dupli_to_afc_err_v_dupli,
	[HV_DEV_AFC_9V]			= &hv_afc_9v_to_afc_9v_dupli,
	[HV_DEV_AFC_9V_DUPLI]		= &hv_afc_9v_dupli_to_afc_9v_dupli,
	/* stock's qc_prepare_to_qc_5v is defined but never linked:
	 * the QC_PREPARE head is qc_prepare_to_qc_9v. Dropped: with
	 * CONFIG_WERROR an unreferenced table is a build error. */
	[HV_DEV_QC_PREPARE]		= &hv_qc_prepare_to_qc_9v,
	[HV_DEV_QC_5V]			= &hv_qc_5v_to_qc_9v,
	[HV_DEV_QC_9V]			= &hv_qc_9v_to_qc_5v,
};

struct max77865_hv {
	struct device *dev;
	struct max77865_muic *muic;
	struct regmap *regmap;
	struct mutex lock;		/* this module's state and register sequences */
	spinlock_t irq_lock;		/* guards pending_irq (IRQ entry has no mutex) */
	unsigned int pending_irq;	/* bit per HV_IRQ_* (1..4) */
	struct workqueue_struct *wq;	/* own ordered workqueue */
	struct work_struct irq_work;	/* deferred hv_irq (MUIC lock was held) */
	struct delayed_work qc_vb_work;	/* QC VBUS settle 300 ms */
	struct delayed_work mping_miss_work; /* MPing miss 2000 ms */
	struct work_struct init_work;	/* detect after the charger is ready */
	struct notifier_block muic_nb;	/* TA attach/detach from the MUIC base */

	int attached_dev;
	u8 tx_data;			/* HVTXBYTE to use */
	u8 qc_hv;			/* supported QC voltage */
	bool afc_disable;		/* charging_mode boot value */
	bool is_afc_muic_ready;
	bool is_afc_handshaking;
	bool is_afc_muic_prepare;
	bool is_charger_ready;
	bool is_qc_vb_settle;
	u8 is_boot_dpdnvden;
	int afc_count;
	u8 bccontrol2;
	u8 hvcontrol[2];
	u8 status1, status2, status3;
	int last_mv;			/* last notified voltage in mV */
};

static struct max77865_hv *hv_singleton;	/* one MUIC per board */

/*
 * The charger registers its notifier here, not on the device: the chain is
 * static so registration succeeds even when the HV module has not bound yet
 * (probe order against the charger is not fixed) or never binds (no MUIC HV
 * DT properties). Events only flow after bind; last_mv reads 0 before that.
 */
static BLOCKING_NOTIFIER_HEAD(hv_chg_nh);

int max77865_hv_last_mv(void)
{
	struct max77865_hv *hv = READ_ONCE(hv_singleton);

	if (!hv)
		return 0;
	/* single int: plain read is safe against a concurrent notifier update */
	return READ_ONCE(hv->last_mv);
}
EXPORT_SYMBOL_GPL(max77865_hv_last_mv);

static void hv_notify(struct max77865_hv *hv, int event, int mv)
{
	int nmv = mv;

	WRITE_ONCE(hv->last_mv, mv);
	blocking_notifier_call_chain(&hv_chg_nh, event, &nmv);
}

int max77865_hv_register_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&hv_chg_nh, nb);
}
EXPORT_SYMBOL_GPL(max77865_hv_register_notifier);

int max77865_hv_unregister_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&hv_chg_nh, nb);
}
EXPORT_SYMBOL_GPL(max77865_hv_unregister_notifier);

/*
 * Register read/write with the stock read-write-read debug pattern kept as
 * dev_dbg only. All callers hold hv->lock.
 */
static int hv_write(struct max77865_hv *hv, u8 reg, u8 val)
{
	unsigned int before, after;
	int ret;

	regmap_read(hv->regmap, reg, &before);
	ret = regmap_write(hv->regmap, reg, val);
	regmap_read(hv->regmap, reg, &after);
	dev_dbg(hv->dev, "reg[0x%02x] = [0x%02x] + [0x%02x] -> [0x%02x]\n",
		reg, before & 0xff, val, after & 0xff);
	return ret;
}

static int hv_update(struct max77865_hv *hv, u8 reg, u8 val, u8 mask)
{
	unsigned int before, after = 0;
	unsigned int new_val;
	int ret = 0;

	if (regmap_read(hv->regmap, reg, &before))
		return -EIO;
	new_val = (val & mask) | (before & ~mask);
	if ((before ^ new_val) & 0xff) {
		ret = hv_write(hv, reg, new_val & 0xff);
		if (ret)
			return ret;
	}
	regmap_read(hv->regmap, reg, &after);
	dev_dbg(hv->dev, "REG(0x%02x): [0x%02x]+[0x%02x:0x%02x]=[0x%02x]\n",
		reg, before & 0xff, val, mask, after & 0xff);
	return after & 0xff;
}

/* reset the HVCONTROL block on detach */
static void hv_reset_regs(struct max77865_hv *hv)
{
	hv_write(hv, MAX77865_MUIC_REG_CONTROL2_BC, 0x00);
	hv_write(hv, MAX77865_MUIC_REG_HVCONTROL1, 0x00);
	hv_write(hv, MAX77865_MUIC_REG_HVCONTROL2, 0x00);
}

static void hv_set_afc_ready(struct max77865_hv *hv, bool value)
{
	dev_info(hv->dev, "afc_muic_ready[%d->%d]\n", hv->is_afc_muic_ready, value);
	hv->is_afc_muic_ready = value;
}

/* ADC one-shot vs always on */
static void hv_adcmode(struct max77865_hv *hv, bool always_on)
{
	int ret;

	if (always_on)
		ret = hv_update(hv, MAX77865_MUIC_REG_HVCONTROL1,
				MAX77865_HVCONTROL1_VBUSADCEN,
				MAX77865_HVCONTROL1_VBUSADCEN);
	else
		ret = hv_update(hv, MAX77865_MUIC_REG_HVCONTROL1, 0,
				MAX77865_HVCONTROL1_VBUSADCEN);
	if (ret < 0)
		dev_err(hv->dev, "cannot switch adcmode(%d)\n", ret);
}

/* ping start after prepare */
static void hv_set_afc_after_prepare(struct max77865_hv *hv)
{
	dev_info(hv->dev, "HV charger is detected\n");

	/* HVCONTROL2 = DP06EN */
	hv_write(hv, MAX77865_MUIC_REG_HVCONTROL2, MAX77865_HVCONTROL2_DP06EN);
	/* HVCONTROL1 = VBUSADCEN */
	hv_write(hv, MAX77865_MUIC_REG_HVCONTROL1, MAX77865_HVCONTROL1_VBUSADCEN);
	hv_write(hv, MAX77865_MUIC_REG_CONTROL2_BC, 0x0f);

	hv_write(hv, MAX77865_MUIC_REG_HVTXBYTE, hv->tx_data);

	/* MPINGEN|MTXEN|MPING|DP06EN|HVDIGEN = 0x5B */
	hv_write(hv, MAX77865_MUIC_REG_HVCONTROL2, 0x5b);
}

/* AFC handshake on MRXRDY (no 12 V) */
static void hv_handshaking(struct max77865_hv *hv)
{
	unsigned int hvtxbyte = 0;
	u8 hvrxbyte[HV_HVRX_MAX];
	u8 select = 0;
	int i, ret;

	memset(hvrxbyte, 0, sizeof(hvrxbyte));

	ret = hv_write(hv, MAX77865_MUIC_REG_HVCONTROL2, 0x13);
	if (ret)
		dev_err(hv->dev, "cannot write hvcontrol2(%d)\n", ret);

	regmap_read(hv->regmap, MAX77865_MUIC_REG_HVTXBYTE, &hvtxbyte);

	for (i = 0; i < HV_HVRX_MAX; i++) {
		unsigned int v = 0;

		regmap_read(hv->regmap, MAX77865_MUIC_REG_HVRXBYTE1 + i, &v);
		hvrxbyte[i] = v & 0xff;
		if (hvrxbyte[i] == 0x47)
			hvrxbyte[i] = 0x46;
		if (!hvrxbyte[i])
			break;
	}

	dev_info(hv->dev, "HVTXBYTE: %02x\n", hvtxbyte & 0xff);
	dev_info(hv->dev, "HVRXBYTE => %*ph\n", i, hvrxbyte);

	if (hvrxbyte[0] != (hvtxbyte & 0xff)) {
		for (i = 0; i < HV_HVRX_MAX && hvrxbyte[i]; i++) {
			if ((hvrxbyte[i] & 0xf0) == 0x40 &&
			    (hvtxbyte & 0xff) > select)
				select = hvrxbyte[i];
		}
		if (select)
			hv_write(hv, MAX77865_MUIC_REG_HVTXBYTE, select);
	}
}

/* ping continue/stop */
static void hv_ping(struct max77865_hv *hv, bool cont)
{
	int ret;

	dev_info(hv->dev, "control ping[%d, %c]\n", hv->afc_count, cont ? 'T' : 'F');
	ret = hv_write(hv, MAX77865_MUIC_REG_HVCONTROL2, cont ? 0x5b : 0x03);
	if (ret)
		dev_err(hv->dev, "cannot write HVCONTROL2(%d)\n", ret);
}

/* QC charger voltages; only 9 V and 5 V kept */
static void hv_qc_charger(struct max77865_hv *hv)
{
	unsigned int bcstatus2 = 0, gpstatus = 0;
	int ret1 = 0, ret2 = 0;

	regmap_read(hv->regmap, MAX77865_MUIC_REG_STATUS2_BC, &bcstatus2);
	regmap_read(hv->regmap, MAX77865_MUIC_REG_STATUS_GP, &gpstatus);
	dev_info(hv->dev, "BC_STATUS2:0x%02x GP_STATUS:0x%02x qc_hv:%x\n",
		 bcstatus2 & 0xff, gpstatus & 0xff, hv->qc_hv);

	switch (hv->qc_hv) {
	case HV_QC_9V:	/* HV_SUPPORT_QC_9V */
		ret1 = hv_write(hv, MAX77865_MUIC_REG_HVCONTROL1, 0x01);
		ret2 = hv_write(hv, MAX77865_MUIC_REG_CONTROL2_BC, 0x19);
		break;
	default:
		dev_err(hv->dev, "not support QC Charger\n");
		break;
	}

	if (ret1 + ret2)
		dev_err(hv->dev, "cannot write HVCONTROL1 or BCCONTROL2(%d)\n",
			ret1 + ret2);
}

/* VBUS settle check 300 ms after the QC prepare */
static void hv_after_qc_prepare(struct max77865_hv *hv)
{
	hv->is_qc_vb_settle = false;
	queue_delayed_work(hv->wq, &hv->qc_vb_work, msecs_to_jiffies(HV_QC_SETTLE_MS));
}

/* stock connect_start() */
static void hv_connect_start(struct max77865_hv *hv)
{
	dev_info(hv->dev, "connect_start\n");

	hv->attached_dev = HV_DEV_AFC_PREPARE;
	hv_adcmode(hv, true);
	hv_set_afc_after_prepare(hv);
	hv->afc_count = 0;
	hv->is_afc_handshaking = false;
	/* MPing miss workaround after 2000 ms */
	queue_delayed_work(hv->wq, &hv->mping_miss_work,
			   msecs_to_jiffies(HV_MPING_MISS_MS));
	hv_notify(hv, MAX77865_HV_EV_PREPARE, 5000);
}

/* TA attach preparation */
static void hv_prepare_afc_charger(struct max77865_hv *hv)
{
	int ret;

	hv_adcmode(hv, false);

	/* CONTROL2_BC DPDRV := 1 (mask 0x0C shift 2, Samsung's max77865-muic.h) */
	ret = hv_update(hv, MAX77865_MUIC_REG_CONTROL2_BC, 0x1 << 2, 0x3 << 2);
	if (ret < 0)
		goto err;
	/* DNMONEN := 1 (bit 5) */
	ret = hv_update(hv, MAX77865_MUIC_REG_CONTROL2_BC, BIT(5), BIT(5));
	if (ret < 0)
		goto err;
	/* DPDNMAN := 1 (bit 4) */
	ret = hv_update(hv, MAX77865_MUIC_REG_CONTROL2_BC, BIT(4), BIT(4));
	if (ret < 0)
		goto err;

	hv_set_afc_ready(hv, true);
	return;
err:
	dev_err(hv->dev, "fail to write muic reg(%d)\n", ret);
}

/* chgdet re-run for the voltage-0 path */
static void hv_chgdet_ready(struct max77865_hv *hv)
{
	unsigned int before, after;
	u8 val;

	regmap_read(hv->regmap, MAX77865_MUIC_REG_CONTROL2_BC, &before);
	val = (0x0 << 2) | (1 << 4);
	hv_write(hv, MAX77865_MUIC_REG_CONTROL2_BC, val);
	regmap_read(hv->regmap, MAX77865_MUIC_REG_CONTROL2_BC, &after);
	dev_info(hv->dev, "BCCTL2:[0x%02x]->[0x%02x]\n", before & 0xff, after & 0xff);

	regmap_read(hv->regmap, MAX77865_MUIC_REG_HVCONTROL2, &before);
	hv_write(hv, MAX77865_MUIC_REG_HVCONTROL2, 0x00);
	regmap_read(hv->regmap, MAX77865_MUIC_REG_HVCONTROL2, &after);
	dev_info(hv->dev, "HVCTL2:[0x%02x]->[0x%02x]\n", before & 0xff, after & 0xff);

	msleep(HV_CHGDET_RERUN_MS);
}

/* runtime voltage change */
static void hv_change_afc_voltage(struct max77865_hv *hv, int tx_data)
{
	unsigned int cur = 0;

	dev_info(hv->dev, "change afc voltage(%x)\n", tx_data);
	regmap_read(hv->regmap, MAX77865_MUIC_REG_HVTXBYTE, &cur);
	if ((cur & 0xff) == tx_data) {
		dev_info(hv->dev, "same to current voltage %x\n", tx_data);
		return;
	}
	hv->afc_count = 0;
	hv_write(hv, MAX77865_MUIC_REG_HVTXBYTE, tx_data);

	if (hv->attached_dev == HV_DEV_QC_5V || hv->attached_dev == HV_DEV_QC_9V) {
		hv_write(hv, MAX77865_MUIC_REG_HVCONTROL1, 0x01);
		if (tx_data == HV_TX_5V)
			hv_write(hv, MAX77865_MUIC_REG_CONTROL2_BC, 0x14);
		else if (tx_data == HV_TX_9V)
			hv_write(hv, MAX77865_MUIC_REG_CONTROL2_BC, 0x19);
	} else {
		hv_adcmode(hv, true);
		hv_ping(hv, true);
	}
}

static bool hv_is_hv_dev(struct max77865_hv *hv)
{
	switch (hv->attached_dev) {
	case HV_DEV_AFC_PREPARE:
	case HV_DEV_AFC_PREPARE_DUPLI:
	case HV_DEV_AFC_5V:
	case HV_DEV_AFC_5V_DUPLI:
	case HV_DEV_AFC_9V:
	case HV_DEV_AFC_9V_DUPLI:
	case HV_DEV_AFC_ERR_V:
	case HV_DEV_AFC_ERR_V_DUPLI:
	case HV_DEV_QC_PREPARE:
	case HV_DEV_QC_5V:
	case HV_DEV_QC_9V:
		return true;
	default:
		return false;
	}
}

/*
 * Charger grant entry points (stock muic_afc_set_voltage()): 9000 runs
 * connect_start() with MUIC_HV_9V only on an HV cable
 * with Rp_56K and no afc_disable/water; 5000 steps back; 0 re-runs chgdet.
 */
void max77865_hv_set_voltage(int mv)
{
	struct max77865_hv *hv = READ_ONCE(hv_singleton);
	unsigned int rp;
	bool water;

	if (!hv)
		return;

	mutex_lock(&hv->lock);
	if (!hv_is_hv_dev(hv)) {
		dev_info(hv->dev, "It's NOT HV cable type\n");
		goto out;
	}
	rp = max77865_muic_cc_rp(hv->muic);
	water = max77865_muic_water(hv->muic);
	if (mv == 9000) {
		if (hv->attached_dev == HV_DEV_AFC_9V ||
		    hv->attached_dev == HV_DEV_AFC_9V_DUPLI ||
		    hv->attached_dev == HV_DEV_QC_9V) {
			goto out;	/* already at 9 V */
		}
		if (rp != HV_RP_56K || hv->afc_disable || water) {
			dev_info(hv->dev, "afc 5V request (%s)\n",
				 hv->afc_disable ? "afc-disable" :
				 water ? "water" : "rp-not-56k");
			goto out;
		}
		hv->tx_data = HV_TX_9V;
		hv_connect_start(hv);
	} else if (mv == 5000) {
		if (hv->attached_dev == HV_DEV_AFC_9V ||
		    hv->attached_dev == HV_DEV_AFC_9V_DUPLI ||
		    hv->attached_dev == HV_DEV_QC_9V ||
		    hv->attached_dev == HV_DEV_AFC_ERR_V ||
		    hv->attached_dev == HV_DEV_AFC_ERR_V_DUPLI)
			hv_change_afc_voltage(hv, HV_TX_5V);
	} else if (mv == 0) {
		dev_info(hv->dev, "TSUB too hot. Chgdet Re-run.\n");
		hv_chgdet_ready(hv);
		max77865_muic_run_chgdet(hv->muic);
	} else {
		dev_warn(hv->dev, "invalid value %d\n", mv);
	}
out:
	mutex_unlock(&hv->lock);
}
EXPORT_SYMBOL_GPL(max77865_hv_set_voltage);

/* charger-ready gate (stock max77865_hv_muic_charger_init()) */
void max77865_hv_charger_ready(void)
{
	struct max77865_hv *hv = READ_ONCE(hv_singleton);

	if (!hv)
		return;
	mutex_lock(&hv->lock);
	if (hv->is_charger_ready) {
		dev_info(hv->dev, "charger is already ready\n");
		mutex_unlock(&hv->lock);
		return;
	}
	hv->is_charger_ready = true;
	queue_work(hv->wq, &hv->init_work);
	mutex_unlock(&hv->lock);
}
EXPORT_SYMBOL_GPL(max77865_hv_charger_ready);

static int hv_handle_attach(struct max77865_hv *hv, const struct hv_step *step)
{
	int new_dev = step->new_dev;
	bool noti = true;
	int ret = 0;
	u8 hvrx0[HV_HVRX_MAX];
	int i;

	if ((hv->hvcontrol[1] & 0x8))
		hv->afc_count = 0;

	if (!hv->is_charger_ready) {
		if (new_dev == HV_DEV_AFC_PREPARE) {
			hv->is_afc_muic_prepare = true;
			dev_info(hv->dev, "is_charger_ready[F], is_afc_muic_prepare[T]\n");
			return 0;
		}
		dev_info(hv->dev, "is_charger_ready[F], just return\n");
		return 0;
	}

	switch (step->func) {
	case FUNC_TA_TO_PREPARE:
		/*
		 * Unlike stock, no connect_start() here. Record "5V
		 * ready" as the stock AFC-5V notification; 9 V starts only through
		 * max77865_hv_set_voltage(9000) from the charger.
		 */
		dev_info(hv->dev, "First check PREPARE! AFC 5V noti.\n");
		new_dev = HV_DEV_AFC_5V;
		noti = true;
		break;
	case FUNC_PREPARE_TO_PREPARE_DUPLI:
		cancel_delayed_work(&hv->mping_miss_work);
		hv->afc_count++;
		hv_handshaking(hv);
		hv->is_afc_handshaking = true;
		if (hv->afc_count > HV_AFC_WA_PING) {
			hv_ping(hv, false);
		} else {
			hv_ping(hv, true);
			noti = false;
		}
		break;
	case FUNC_PREPARE_TO_AFC_5V:
		noti = false;
		break;
	case FUNC_PREPARE_TO_QC_PREPARE:
		cancel_delayed_work(&hv->mping_miss_work);
		if (!IS_ENABLED(CONFIG_CHARGER_MAX77865_QC)) {
			noti = false;
			break;
		}
		ret = hv_write(hv, MAX77865_MUIC_REG_HVCONTROL2, 0x03);
		if (ret)
			dev_err(hv->dev, "cannot write HVCONTROL2(%d)\n", ret);
		hv_qc_charger(hv);
		hv_after_qc_prepare(hv);
		break;
	case FUNC_PREPARE_DUPLI_TO_PREPARE_DUPLI:
		hv->afc_count++;
		if (!hv->is_afc_handshaking) {
			hv_handshaking(hv);
			hv->is_afc_handshaking = true;
		}
		if (hv->afc_count > HV_AFC_WA_PING) {
			hv_ping(hv, false);
		} else {
			hv_ping(hv, true);
			noti = false;
		}
		break;
	case FUNC_PREPARE_DUPLI_TO_AFC_5V:
		noti = false;
		break;
	case FUNC_PREPARE_DUPLI_TO_AFC_ERR_V:
		if (hv->afc_count > HV_AFC_WA_PING) {
			hv_ping(hv, false);
		} else {
			hv_ping(hv, true);
			noti = false;
		}
		break;
	case FUNC_PREPARE_DUPLI_TO_AFC_9V:
		hv_ping(hv, false);
		hv_adcmode(hv, false);
		break;
	case FUNC_PREPARE_DUPLI_TO_QC_PREPARE:
		if (!IS_ENABLED(CONFIG_CHARGER_MAX77865_QC)) {
			noti = false;
			break;
		}
		hv_qc_charger(hv);
		hv_after_qc_prepare(hv);
		break;
	case FUNC_AFC_5V_TO_AFC_5V_DUPLI:
		cancel_delayed_work(&hv->mping_miss_work);
		hv->afc_count++;
		if (!hv->is_afc_handshaking) {
			hv_handshaking(hv);
			hv->is_afc_handshaking = true;
		}
		if (hv->afc_count > HV_AFC_WA_PING) {
			hv_ping(hv, false);
			hv_adcmode(hv, true);
		} else {
			hv_ping(hv, true);
			noti = false;
		}
		break;
	case FUNC_AFC_5V_TO_AFC_ERR_V:
		cancel_delayed_work(&hv->mping_miss_work);
		if (hv->afc_count <= HV_AFC_WA_PING)
			noti = false;
		break;
	case FUNC_AFC_5V_TO_AFC_9V:
		cancel_delayed_work(&hv->mping_miss_work);
		hv_ping(hv, false);
		hv_adcmode(hv, false);
		break;
	case FUNC_AFC_5V_TO_QC_PREPARE:
		cancel_delayed_work(&hv->mping_miss_work);
		if (!IS_ENABLED(CONFIG_CHARGER_MAX77865_QC)) {
			noti = false;
			break;
		}
		hv_qc_charger(hv);
		hv_after_qc_prepare(hv);
		break;
	case FUNC_AFC_5V_DUPLI_TO_AFC_5V_DUPLI:
		hv->afc_count++;
		if (!hv->is_afc_handshaking) {
			hv_handshaking(hv);
			hv->is_afc_handshaking = true;
		}
		if (hv->afc_count > HV_AFC_WA_PING) {
			hv_ping(hv, false);
			hv_adcmode(hv, true);
		} else {
			hv_ping(hv, true);
			noti = false;
		}
		break;
	case FUNC_AFC_5V_DUPLI_TO_AFC_ERR_V:
		if (hv->afc_count > HV_AFC_WA_PING) {
			hv_ping(hv, false);
		} else {
			hv_ping(hv, true);
			noti = false;
		}
		break;
	case FUNC_AFC_5V_DUPLI_TO_AFC_9V:
		hv_ping(hv, false);
		hv_adcmode(hv, false);
		break;
	case FUNC_AFC_5V_DUPLI_TO_QC_PREPARE:
		if (!IS_ENABLED(CONFIG_CHARGER_MAX77865_QC)) {
			noti = false;
			break;
		}
		hv_qc_charger(hv);
		hv_after_qc_prepare(hv);
		break;
	case FUNC_AFC_ERR_V_TO_AFC_ERR_V_DUPLI:
		hv->afc_count++;
		if (hv->afc_count > HV_AFC_WA_PING) {
			hv_ping(hv, false);
			hv_adcmode(hv, true);
		} else {
			hv_ping(hv, true);
			noti = false;
		}
		break;
	case FUNC_AFC_ERR_V_TO_AFC_5V:
		hv_ping(hv, false);
		hv_adcmode(hv, false);
		break;
	case FUNC_AFC_ERR_V_TO_AFC_9V:
		hv_ping(hv, false);
		hv_adcmode(hv, false);
		break;
	case FUNC_AFC_ERR_V_TO_QC_PREPARE:
		if (!IS_ENABLED(CONFIG_CHARGER_MAX77865_QC)) {
			noti = false;
			break;
		}
		hv_qc_charger(hv);
		hv_after_qc_prepare(hv);
		break;
	case FUNC_AFC_ERR_V_DUPLI_TO_AFC_ERR_V_DUPLI:
		hv->afc_count++;
		if (hv->afc_count > HV_AFC_WA_PING) {
			hv_ping(hv, false);
			hv_adcmode(hv, true);
		} else {
			hv_ping(hv, true);
			noti = false;
		}
		break;
	case FUNC_AFC_ERR_V_DUPLI_TO_AFC_5V:
		hv_ping(hv, false);
		hv_adcmode(hv, false);
		break;
	case FUNC_AFC_ERR_V_DUPLI_TO_AFC_9V:
		hv_ping(hv, false);
		hv_adcmode(hv, false);
		break;
	case FUNC_AFC_ERR_V_DUPLI_TO_QC_PREPARE:
		if (!IS_ENABLED(CONFIG_CHARGER_MAX77865_QC)) {
			noti = false;
			break;
		}
		hv_qc_charger(hv);
		hv_after_qc_prepare(hv);
		break;
	case FUNC_AFC_9V_TO_AFC_9V_DUPLI:
		hv->afc_count++;
		if (hv->afc_count > HV_AFC_WA_PING) {
			hv_ping(hv, false);
			hv_adcmode(hv, true);
		} else {
			hv_ping(hv, true);
			noti = false;
		}
		break;
	case FUNC_AFC_9V_TO_AFC_ERR_V:
		if (hv->afc_count > HV_AFC_WA_PING) {
			hv_ping(hv, false);
		} else {
			hv_ping(hv, true);
			noti = false;
		}
		break;
	case FUNC_AFC_9V_TO_AFC_5V:
		hv_ping(hv, false);
		hv_adcmode(hv, false);
		break;
	case FUNC_AFC_9V_TO_QC_PREPARE:
		if (!IS_ENABLED(CONFIG_CHARGER_MAX77865_QC)) {
			noti = false;
			break;
		}
		hv_qc_charger(hv);
		hv_after_qc_prepare(hv);
		break;
	case FUNC_AFC_9V_DUPLI_TO_AFC_ERR_V:
		if (hv->afc_count > HV_AFC_WA_PING) {
			hv_ping(hv, false);
		} else {
			hv_ping(hv, true);
			noti = false;
		}
		break;
	case FUNC_AFC_9V_DUPLI_TO_AFC_5V:
		hv_ping(hv, false);
		hv_adcmode(hv, false);
		break;
	case FUNC_AFC_9V_DUPLI_TO_AFC_9V_DUPLI:
		hv->afc_count++;
		if (hv->afc_count > HV_AFC_WA_PING) {
			hv_ping(hv, false);
			hv_adcmode(hv, true);
		} else {
			hv_ping(hv, true);
			noti = false;
		}
		break;
	case FUNC_AFC_9V_DUPLI_TO_QC_PREPARE:
		if (!IS_ENABLED(CONFIG_CHARGER_MAX77865_QC)) {
			noti = false;
			break;
		}
		hv_qc_charger(hv);
		hv_after_qc_prepare(hv);
		break;
	case FUNC_QC_PREPARE_TO_QC_5V:
		if (hv->is_qc_vb_settle)
			hv_adcmode(hv, false);
		else
			noti = false;
		break;
	case FUNC_QC_PREPARE_TO_QC_9V:
		hv->is_qc_vb_settle = true;
		hv_adcmode(hv, false);
		break;
	case FUNC_QC_5V_TO_QC_9V:
		hv->is_qc_vb_settle = true;
		hv_adcmode(hv, false);
		break;
	case FUNC_QC_9V_TO_QC_5V:
		hv_adcmode(hv, false);
		break;
	default:
		dev_warn(hv->dev, "undefined hv function num(%d)\n", step->func);
		return -ESRCH;
	}

	if (hv->attached_dev == new_dev)
		noti = false;
	else if (new_dev == HV_DEV_AFC_PREPARE_DUPLI ||
		 new_dev == HV_DEV_AFC_5V_DUPLI ||
		 new_dev == HV_DEV_AFC_9V_DUPLI ||
		 new_dev == HV_DEV_AFC_ERR_V_DUPLI)
		noti = false;

	hv->attached_dev = new_dev;

	if (noti) {
		int event = MAX77865_HV_EV_NONE, mv = 0;

		switch (new_dev) {
		case HV_DEV_AFC_PREPARE:
		case HV_DEV_AFC_PREPARE_DUPLI:
			event = MAX77865_HV_EV_PREPARE;
			mv = 5000;
			break;
		case HV_DEV_AFC_5V:
		case HV_DEV_AFC_5V_DUPLI:
		case HV_DEV_QC_PREPARE:
			event = MAX77865_HV_EV_5V;
			mv = 5000;
			dev_info(hv->dev, "afc 5V request (detect)\n");
			break;
		case HV_DEV_AFC_9V:
		case HV_DEV_AFC_9V_DUPLI:
		case HV_DEV_QC_9V:
			event = MAX77865_HV_EV_9V;
			mv = 9000;
			{
				unsigned int tx = 0;

				regmap_read(hv->regmap, MAX77865_MUIC_REG_HVTXBYTE, &tx);
				for (i = 0; i < HV_HVRX_MAX; i++) {
					unsigned int v = 0;

					regmap_read(hv->regmap,
						    MAX77865_MUIC_REG_HVRXBYTE1 + i, &v);
					hvrx0[i] = v & 0xff;
					if (!hvrx0[i])
						break;
				}
				if (new_dev == HV_DEV_QC_9V)
					dev_info(hv->dev, "qc 9V\n");
				else
					dev_info(hv->dev, "afc 9V (hvrx %*ph)\n", i, hvrx0);
			}
			break;
		case HV_DEV_AFC_ERR_V:
		case HV_DEV_AFC_ERR_V_DUPLI:
			event = MAX77865_HV_EV_ERR_V;
			mv = 9000;
			break;
		default:
			break;
		}
		hv_notify(hv, event, mv);
	}
	return ret;
}

static bool hv_match_irq(const struct hv_step *step, int irq)
{
	if (step->irq == irq)
		return true;
	if (step->irq == HV_IRQ_DONTCARE)
		return true;
	return false;
}

static bool hv_match_dpdn(const struct hv_step *step, u8 dpdnvden)
{
	if (step->dpdnvden == dpdnvden)
		return true;
	if (step->dpdnvden == HV_DPDNVDEN_DONTCARE)
		return true;
	return false;
}

/* VBADC window match (no 12 V) */
static bool hv_match_vbadc(const struct hv_step *step, u8 vbadc)
{
	if (step->vbadc == vbadc)
		return true;
	if (step->vbadc == HV_VBADC_AFC_5V) {
		switch (vbadc) {
		case HV_VBADC_4V_5V:
		case HV_VBADC_5V_6V:
		case HV_VBADC_6V_7V:
			return true;
		default:
			break;
		}
	}
	if (step->vbadc == HV_VBADC_AFC_9V) {
		switch (vbadc) {
		case HV_VBADC_7V_8V:
		case HV_VBADC_8V_9V:
		case HV_VBADC_9V_10V:
			return true;
		default:
			break;
		}
	}
	if (step->vbadc == HV_VBADC_AFC_ERR_V_NOT_0) {
		switch (vbadc) {
		case HV_VBADC_10V_11V:
		case HV_VBADC_11V_12V:
		case HV_VBADC_12V_13V:
		case HV_VBADC_13V:
			return true;
		default:
			break;
		}
	}
	if (step->vbadc == HV_VBADC_AFC_ERR_V) {
		switch (vbadc) {
		case HV_VBADC_VBDET:
		case HV_VBADC_10V_11V:
		case HV_VBADC_11V_12V:
		case HV_VBADC_12V_13V:
		case HV_VBADC_13V:
			return true;
		default:
			break;
		}
	}
	if (step->vbadc == HV_VBADC_QC_5V) {
		switch (vbadc) {
		case HV_VBADC_4V_5V:
		case HV_VBADC_5V_6V:
		case HV_VBADC_6V_7V:
			return true;
		default:
			break;
		}
	}
	if (step->vbadc == HV_VBADC_QC_9V) {
		switch (vbadc) {
		case HV_VBADC_6V_7V:
		case HV_VBADC_7V_8V:
		case HV_VBADC_8V_9V:
		case HV_VBADC_9V_10V:
			return true;
		default:
			break;
		}
	}
	if (step->vbadc == HV_VBADC_ANY) {
		switch (vbadc) {
		case HV_VBADC_4V_5V:
		case HV_VBADC_5V_6V:
		case HV_VBADC_6V_7V:
		case HV_VBADC_7V_8V:
		case HV_VBADC_8V_9V:
		case HV_VBADC_9V_10V:
		case HV_VBADC_10V_11V:
		case HV_VBADC_11V_12V:
		case HV_VBADC_12V_13V:
		case HV_VBADC_13V:
			return true;
		default:
			break;
		}
	}
	if (step->vbadc == HV_VBADC_DONTCARE)
		return true;
	return false;
}

static bool hv_match_vdnmon(const struct hv_step *step, u8 vdnmon)
{
	if (step->vdnmon == vdnmon)
		return true;
	if (step->vdnmon == HV_VDNMON_DONTCARE)
		return true;
	return false;
}

/* TA type check */
static bool hv_check_ta(struct max77865_hv *hv)
{
	u8 uid, vbvolt, chgdetrun, chgtyp;

	uid = hv->status3 & MAX77865_GP_STATUS_UID;
	vbvolt = hv->status1 & MAX77865_BC_STATUS1_VBVOLT;
	chgdetrun = hv->status1 & MAX77865_BC_STATUS1_CHGDETRUN;
	chgtyp = hv->status1 & MAX77865_BC_STATUS1_CHGTYP;

	if (uid != 0xc0) {	/* UID_OPEN */
		hv_set_afc_ready(hv, false);
		return false;
	}
	if (!vbvolt || chgdetrun || chgtyp != MAX77865_CHGTYP_DEDICATED_CHARGER) {
		hv_set_afc_ready(hv, false);
		if (hv->attached_dev != HV_DEV_NONE) {
			hv->attached_dev = HV_DEV_NONE;
			hv_notify(hv, MAX77865_HV_EV_NONE, 0);
		}
		return false;
	}
	return true;
}

/* stock max77865_hv_muic_detect_dev() */
static void hv_detect_dev(struct max77865_hv *hv, int irq)
{
	const struct hv_step *step;
	unsigned int s0 = 0, s1 = 0, s2 = 0, bc = 0, h0 = 0, h1 = 0;
	u8 vdnmon, dpdnvden, vbadc;
	int i, ret;
	bool last = false;

	if (hv->attached_dev < 0 || hv->attached_dev >= HV_DEV_NUM ||
	    !hv_checklist[hv->attached_dev]) {
		dev_info(hv->dev, "non AFC Charger, just return!\n");
		return;
	}
	step = hv_checklist[hv->attached_dev];
	dev_info(hv->dev, "detect irq(%d), attached_dev(%d)\n", irq, hv->attached_dev);

	if (regmap_read(hv->regmap, MAX77865_MUIC_REG_STATUS1_BC, &s0) ||
	    regmap_read(hv->regmap, MAX77865_MUIC_REG_STATUS2_BC, &s1) ||
	    regmap_read(hv->regmap, MAX77865_MUIC_REG_STATUS_GP, &s2)) {
		dev_err(hv->dev, "fail to read muic status\n");
		return;
	}
	dev_info(hv->dev, "BC STATUS1:0x%02x, 2:0x%02x, GP STATUS:0x%02x\n",
		 s0 & 0xff, s1 & 0xff, s2 & 0xff);
	hv->status1 = s0 & 0xff;
	hv->status2 = s1 & 0xff;
	hv->status3 = s2 & 0xff;

	if (!hv_check_ta(hv)) {
		dev_err(hv->dev, "device type is not TA!\n");
		return;
	}

	vdnmon = hv->status2 & MAX77865_BC_STATUS2_DNVDATREF;
	vbadc = hv->status3 & MAX77865_GP_STATUS_VBADC;

	if (regmap_read(hv->regmap, MAX77865_MUIC_REG_CONTROL2_BC, &bc) ||
	    regmap_read(hv->regmap, MAX77865_MUIC_REG_HVCONTROL1, &h0) ||
	    regmap_read(hv->regmap, MAX77865_MUIC_REG_HVCONTROL2, &h1)) {
		dev_err(hv->dev, "fail to read muic regs\n");
		return;
	}
	dev_info(hv->dev, "BCCONTROL2: 0x%02x, HVCONTROL1:0x%02x, 2:0x%02x\n",
		 bc & 0xff, h0 & 0xff, h1 & 0xff);
	hv->bccontrol2 = bc & 0xff;
	hv->hvcontrol[0] = h0 & 0xff;
	hv->hvcontrol[1] = h1 & 0xff;
	dpdnvden = hv->bccontrol2 & MAX77865_BC_CONTROL2_DPDNMAN;
	dev_info(hv->dev, "vdnmon:0x%x vbadc:0x%x dpdnvden:0x%x\n",
		 vdnmon, vbadc, dpdnvden);

	for (i = 0; i < HV_DEV_NUM; i++, step = step->next) {
		if (step->next == step)
			last = true;
		if (!hv_match_irq(step, irq))
			goto next;
		if (!hv_match_dpdn(step, dpdnvden))
			goto next;
		if (!hv_match_vbadc(step, vbadc))
			goto next;
		if (!hv_match_vdnmon(step, vdnmon))
			goto next;
		dev_info(hv->dev, "checklist match at i(%d), %s(%d)\n",
			 i, step->name, step->new_dev);
		dev_info(hv->dev, "AFC ATTACHED %d->%d\n", hv->attached_dev,
			 step->new_dev);
		ret = hv_handle_attach(hv, step);
		if (ret)
			dev_err(hv->dev, "cannot handle attach(%d)\n", ret);
		return;
next:
		if (last) {
			dev_info(hv->dev, "not found new_dev in checklist\n");
			break;
		}
	}
	dev_info(hv->dev, "AFC MAINTAIN (%d)\n", hv->attached_dev);
}

/*
 * Deferred interrupt work: the hv_irq entry runs with the MUIC lock held, so it
 * cannot take hv->lock (which nests over the MUIC lock in set_voltage paths) nor
 * call back into the MUIC base. The work runs without the MUIC lock.
 */
static void hv_irq_work_fn(struct work_struct *work)
{
	struct max77865_hv *hv = container_of(work, struct max77865_hv, irq_work);
	unsigned int mask;
	int irq;

	for (;;) {
		spin_lock(&hv->irq_lock);
		mask = hv->pending_irq;
		hv->pending_irq = 0;
		spin_unlock(&hv->irq_lock);
		if (!mask)
			return;
		/* stock priority */
		if (mask & BIT(HV_IRQ_VDNMON))
			irq = HV_IRQ_VDNMON;
		else if (mask & BIT(HV_IRQ_MRXRDY))
			irq = HV_IRQ_MRXRDY;
		else if (mask & BIT(HV_IRQ_MPNACK))
			irq = HV_IRQ_MPNACK;
		else
			irq = HV_IRQ_VBADC;

		mutex_lock(&hv->lock);
		if (hv->afc_disable || max77865_muic_water(hv->muic)) {
			mutex_unlock(&hv->lock);
			continue;	/* as stock */
		}
		if (!hv->is_afc_muic_ready) {
			mutex_unlock(&hv->lock);
			continue;	/* as stock */
		}
		if (!hv->is_charger_ready && irq != HV_IRQ_VDNMON) {
			mutex_unlock(&hv->lock);
			continue;
		}
		/* Rp_Abnormal drops HV by re-running charger detection (Samsung's muic_ccic.c) */
		if (max77865_muic_cc_rp(hv->muic) == HV_RP_ABNORMAL && hv_is_hv_dev(hv)) {
			dev_info(hv->dev, "CC or SBU short. Chgdet Re-run.\n");
			hv_chgdet_ready(hv);
			mutex_unlock(&hv->lock);
			max77865_muic_run_chgdet(hv->muic);
			continue;
		}
		hv_detect_dev(hv, irq);
		mutex_unlock(&hv->lock);
	}
}

/*
 * MUIC interrupt demux entry: the base calls this from its IRQ
 * thread with the MUIC lock held, with the three interrupt bytes it read (stock
 * muic_task_max77865.c demux: VDNMON = INT_BC bit 5, MRXRDY = INT_FC
 * bit 7, MPNACK = INT_FC bit 3, VBADC = INT_GP bit 2). Only records; the work
 * above does the rest.
 */
static void hv_irq_cb(void *ctx, u8 int_bc, u8 int_fc, u8 int_gp)
{
	struct max77865_hv *hv = ctx;
	int irq = -1;

	if (int_bc & MAX77865_HV_INT_BC_VDNMON)
		irq = HV_IRQ_VDNMON;
	else if (int_fc & MAX77865_HV_INT_FC_MRXRDY)
		irq = HV_IRQ_MRXRDY;
	else if (int_fc & MAX77865_HV_INT_FC_MPNACK)
		irq = HV_IRQ_MPNACK;
	else if (int_gp & MAX77865_HV_INT_GP_VBADC)
		irq = HV_IRQ_VBADC;
	else
		return;

	spin_lock(&hv->irq_lock);
	hv->pending_irq |= BIT(irq);
	spin_unlock(&hv->irq_lock);
	queue_work(hv->wq, &hv->irq_work);
}

/* MUIC attached-device changes: TA attach prepares AFC, detach resets */
static int hv_muic_notify(struct notifier_block *nb, unsigned long action, void *data)
{
	struct max77865_hv *hv = container_of(nb, struct max77865_hv, muic_nb);
	int dev;

	if (action != MAX77865_MUIC_EV_ATTACHED_DEV || !data)
		return NOTIFY_DONE;
	dev = *(int *)data;

	mutex_lock(&hv->lock);
	if (dev == MAX77865_MUIC_ATTACHED_TA) {
		hv->attached_dev = HV_DEV_TA;
		hv->is_afc_muic_prepare = true;
		hv_prepare_afc_charger(hv);
	} else if (dev == MAX77865_MUIC_ATTACHED_NONE) {
		hv_set_afc_ready(hv, false);
		hv->is_afc_muic_prepare = false;
		hv_reset_regs(hv);	/* stock detach */
		cancel_delayed_work(&hv->qc_vb_work);
		cancel_delayed_work(&hv->mping_miss_work);
		hv->attached_dev = HV_DEV_NONE;
		mutex_unlock(&hv->lock);
		hv_notify(hv, MAX77865_HV_EV_NONE, 0);
		return NOTIFY_OK;
	}
	mutex_unlock(&hv->lock);
	return NOTIFY_OK;
}

/* QC VBUS settle work */
static void hv_qc_vb_work(struct work_struct *work)
{
	struct max77865_hv *hv = container_of(to_delayed_work(work),
					      struct max77865_hv, qc_vb_work);
	unsigned int gpstatus = 0;
	u8 vbadc;

	mutex_lock(&hv->lock);
	if (hv->is_qc_vb_settle)
		goto out;
	if (regmap_read(hv->regmap, MAX77865_MUIC_REG_STATUS_GP, &gpstatus))
		goto out;
	vbadc = (gpstatus & MAX77865_GP_STATUS_VBADC);
	if (vbadc == HV_VBADC_4V_5V || vbadc == HV_VBADC_5V_6V) {
		hv->is_qc_vb_settle = true;
		hv_detect_dev(hv, HV_IRQ_VBADC);
	}
out:
	mutex_unlock(&hv->lock);
}

/* MPing miss workaround */
static void hv_mping_miss_work(struct work_struct *work)
{
	struct max77865_hv *hv = container_of(to_delayed_work(work),
					      struct max77865_hv, mping_miss_work);

	mutex_lock(&hv->lock);
	if (hv->attached_dev != HV_DEV_AFC_PREPARE &&
	    hv->attached_dev != HV_DEV_AFC_5V)
		goto out;
	/* virtual MPNACK */
	hv_detect_dev(hv, HV_IRQ_MPNACK);
out:
	mutex_unlock(&hv->lock);
}

/* detect after the charger init */
static void hv_init_work_fn(struct work_struct *work)
{
	struct max77865_hv *hv = container_of(work, struct max77865_hv, init_work);
	unsigned int bcstatus2 = 0, gpstatus = 0;

	mutex_lock(&hv->lock);
	if (regmap_read(hv->regmap, MAX77865_MUIC_REG_STATUS2_BC, &bcstatus2) ||
	    regmap_read(hv->regmap, MAX77865_MUIC_REG_STATUS_GP, &gpstatus)) {
		dev_err(hv->dev, "fail to read muic regs\n");
		goto out;
	}
	dev_info(hv->dev, "BC STATUS2:0x%02x, GP STATUS:0x%02x\n",
		 bcstatus2 & 0xff, gpstatus & 0xff);
	if (hv->is_afc_muic_ready) {
		if (hv->is_afc_muic_prepare)
			hv_detect_dev(hv, HV_IRQ_VDNMON);
		else
			hv_detect_dev(hv, -1);
	}
out:
	mutex_unlock(&hv->lock);
}

/* initial VDNMON check */
static void hv_init_detect(struct max77865_hv *hv)
{
	unsigned int bcstatus2 = 0;
	u8 vdnmon;

	if (hv->is_boot_dpdnvden == HV_DPDNVDEN_ENABLE) {
		dev_info(hv->dev, "dpdnvden already ENABLE\n");
	} else if (hv->is_boot_dpdnvden == HV_DPDNVDEN_DISABLE) {
		msleep(HV_INIT_DETECT_MS);
		dev_info(hv->dev, "dpdnvden == DISABLE, 30ms delay\n");
	} else {
		dev_err(hv->dev, "dpdnvden is not correct(0x%x)!\n",
			hv->is_boot_dpdnvden);
		return;
	}

	if (regmap_read(hv->regmap, MAX77865_MUIC_REG_STATUS2_BC, &bcstatus2)) {
		dev_err(hv->dev, "fail to read STATUS2_BC\n");
		return;
	}
	vdnmon = (bcstatus2 & MAX77865_BC_STATUS2_DNVDATREF);
	if (vdnmon == HV_VDNMON_LOW)
		hv_detect_dev(hv, HV_IRQ_VDNMON);
	else
		dev_info(hv->dev, "vdnmon != LOW(0x%x)\n", vdnmon);
}

/* afc_disable from the charging_mode boot parameter (Samsung's muic-core.c, muic.h) */
static bool hv_boot_afc_disabled(void)
{
	static const char key[] = "charging_mode=";
	char *p = saved_command_line;
	int val = 0;

	while ((p = strstr(p, key))) {
		if (p == saved_command_line || p[-1] == ' ') {
			p += sizeof(key) - 1;
			if (kstrtoint(p, 0, &val))
				val = 0;
			break;
		}
		p += sizeof(key) - 1;
	}
	return ((val & 0xff00) >> 8) == HV_AFC_DISABLE_VAL;
}

static void hv_teardown(void *data)
{
	struct max77865_hv *hv = data;

	max77865_muic_set_hv_irq(hv->muic, NULL, NULL);
	max77865_muic_unregister_notifier(hv->muic, &hv->muic_nb);
	cancel_work_sync(&hv->init_work);
	cancel_work_sync(&hv->irq_work);
	cancel_delayed_work_sync(&hv->qc_vb_work);
	cancel_delayed_work_sync(&hv->mping_miss_work);
}

static void hv_wq_destroy(void *data)
{
	destroy_workqueue(data);
}

static void hv_singleton_clear(void *data)
{
	if (READ_ONCE(hv_singleton) == data)
		WRITE_ONCE(hv_singleton, NULL);
}

static int hv_probe(struct auxiliary_device *adev, const struct auxiliary_device_id *id)
{
	struct device *dev = &adev->dev;
	struct device_node *np = dev->parent ? dev->parent->of_node : NULL;
	struct max77865_hv *hv;
	struct max77865_muic *muic;
	unsigned int bccontrol2 = 0;
	u8 afc_tx, qc_hv;
	int ret;

	muic = max77865_muic_from_aux(adev);
	if (!muic)
		return -ENODEV;

	hv = devm_kzalloc(dev, sizeof(*hv), GFP_KERNEL);
	if (!hv)
		return -ENOMEM;

	hv->dev = dev;
	hv->muic = muic;
	hv->regmap = max77865_muic_regmap(muic);
	if (!hv->regmap)
		return -ENODEV;

	/* DT gate on the MUIC node (stock drivers/muic/universal/muic_dt.c) */
	if (!np) {
		dev_info(dev, "no MUIC node, AFC off\n");
		return -ENODEV;
	}
	ret = of_property_read_u8(np, "maxim,qc-hv", &qc_hv);
	if (ret) {
		dev_info(dev, "no maxim,qc-hv, AFC off\n");
		return -ENODEV;
	}
	ret = of_property_read_u8(np, "maxim,afc-initial-tx", &afc_tx);
	if (ret) {
		dev_info(dev, "no maxim,afc-initial-tx, AFC off\n");
		return -ENODEV;
	}
	hv->qc_hv = qc_hv;
	hv->tx_data = afc_tx;
	dev_info(dev, "qc-hv:0x%02x afc-tx:0x%02x\n", hv->qc_hv, hv->tx_data);

	mutex_init(&hv->lock);
	spin_lock_init(&hv->irq_lock);
	hv->attached_dev = HV_DEV_NONE;
	hv->afc_disable = hv_boot_afc_disabled();
	if (hv->afc_disable)
		dev_info(dev, "AFC mode disabled\n");
	else
		dev_info(dev, "AFC mode enabled\n");

	hv->wq = alloc_ordered_workqueue("max77865_hv", 0);
	if (!hv->wq)
		return -ENOMEM;
	ret = devm_add_action_or_reset(dev, hv_wq_destroy, hv->wq);
	if (ret)
		return ret;

	INIT_WORK(&hv->irq_work, hv_irq_work_fn);
	INIT_DELAYED_WORK(&hv->qc_vb_work, hv_qc_vb_work);
	INIT_DELAYED_WORK(&hv->mping_miss_work, hv_mping_miss_work);
	INIT_WORK(&hv->init_work, hv_init_work_fn);
	hv->muic_nb.notifier_call = hv_muic_notify;

	auxiliary_set_drvdata(adev, hv);
	WRITE_ONCE(hv_singleton, hv);
	ret = devm_add_action_or_reset(dev, hv_singleton_clear, hv);
	if (ret)
		return ret;

	/* locks/works/notifier exist before the IRQ callback is set */
	ret = devm_add_action_or_reset(dev, hv_teardown, hv);
	if (ret)
		return ret;
	ret = max77865_muic_register_notifier(muic, &hv->muic_nb);
	if (ret)
		return dev_err_probe(dev, ret, "cannot register MUIC notifier\n");
	ret = max77865_muic_set_hv_irq(muic, hv_irq_cb, hv);
	if (ret)
		return dev_err_probe(dev, ret, "cannot set hv irq\n");

	/* boot DPDNVDEN state */
	if (regmap_read(hv->regmap, MAX77865_MUIC_REG_CONTROL2_BC, &bccontrol2))
		hv->is_boot_dpdnvden = HV_DPDNVDEN_DONTCARE;
	else
		hv->is_boot_dpdnvden = bccontrol2 & MAX77865_BC_CONTROL2_DPDNMAN;

	hv_set_afc_ready(hv, false);
	hv->afc_count = 0;

	/* current TA state, if any */
	if (max77865_muic_attached_dev(muic) == MAX77865_MUIC_ATTACHED_TA) {
		hv->attached_dev = HV_DEV_TA;
		hv->is_afc_muic_prepare = true;
		hv_prepare_afc_charger(hv);
	}
	dev_info(dev, "is_afc_muic_ready=%d\n", hv->is_afc_muic_ready);
	if (hv->is_afc_muic_ready)
		hv_init_detect(hv);
	return 0;
}

static void hv_remove(struct auxiliary_device *adev)
{
	/* torn down by the devm actions */
}

static const struct auxiliary_device_id hv_id_table[] = {
	{ .name = "max77865-muic.hv", },
	{ }
};
MODULE_DEVICE_TABLE(auxiliary, hv_id_table);

static struct auxiliary_driver hv_driver = {
	.name = "hv",
	.id_table = hv_id_table,
	.probe = hv_probe,
	.remove = hv_remove,
};
module_auxiliary_driver(hv_driver);

MODULE_DESCRIPTION("Maxim MAX77865 MUIC AFC/QC high-voltage module");
MODULE_LICENSE("GPL");
