/* NXP PCF50633 Power Management Unit (PMU) driver
 *
 * (C) 2006-2008 by Openmoko, Inc.
 * Author: Harald Welte <laforge@openmoko.org>
 * 		Balaji Rao <balajirrao@openmoko.org>
 * All rights reserved.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/irqdomain.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/export.h>
#include <linux/slab.h>
#include <linux/regmap.h>
#include <linux/err.h>

#include <linux/mfd/pcf50633/core.h>

/* Two MBCS registers used during cold start */
#define PCF50633_REG_MBCS1		0x4b
#define PCF50633_REG_MBCS2		0x4c
#define PCF50633_MBCS1_USBPRES 		0x01
#define PCF50633_MBCS1_ADAPTPRES	0x01

static void pcf50633_irq_lock(struct irq_data *data)
{
	struct pcf50633 *pcf = irq_data_get_irq_chip_data(data);

	mutex_lock(&pcf->irq_lock);
}

static void pcf50633_irq_sync_unlock(struct irq_data *data)
{
	struct pcf50633 *pcf = irq_data_get_irq_chip_data(data);
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(pcf->mask_regs); ++i) {
		if (pcf->mask_regs[i] == pcf->mask_regs_cur[i])
			continue;

		regmap_update_bits(pcf->regmap, PCF50633_REG_INT1M + i, 
		    pcf->mask_regs[i] ^ pcf->mask_regs_cur[i],
		    pcf->mask_regs[i]);
		pcf->mask_regs_cur[i] = pcf->mask_regs[i];
	}

	mutex_unlock(&pcf->irq_lock);
}

static void pcf50633_irq_mask(struct irq_data *data)
{
	struct pcf50633 *pcf = irq_data_get_irq_chip_data(data);
	int irq = data->irq;
	u8 bit;
	int idx;

	idx = irq >> 3;
	bit = 1 << (irq & 0x07);

	pcf->mask_regs[idx] |= bit;
}

static void pcf50633_irq_unmask(struct irq_data *data)
{
	struct pcf50633 *pcf = irq_data_get_irq_chip_data(data);
	int irq = data->irq;
	u8 bit;
	int idx;

	idx = irq >> 3;
	bit = 1 << (irq & 0x07);

	pcf->mask_regs[idx] &= ~bit;
}

static struct irq_chip pcf50633_irq_chip = {
	.name = "pcf50633-irq",
	.irq_mask = pcf50633_irq_mask,
	.irq_unmask = pcf50633_irq_unmask,
	.irq_bus_lock = pcf50633_irq_lock,
	.irq_bus_sync_unlock = pcf50633_irq_sync_unlock,
};


#define DECLARE_IRQ(_irq, _regno) \
	[PCF50633_IRQ_ ## _irq] = { \
		.reg_offset = _regno-1, \
		.mask = PCF50633_INT ## _regno ## _ ## _irq, \
	}

static struct regmap_irq pcf50633_irqs[] = {
	DECLARE_IRQ(ADPINS, 1),
	DECLARE_IRQ(ADPREM, 1),
	DECLARE_IRQ(USBINS, 1),
	DECLARE_IRQ(USBREM, 1),
	DECLARE_IRQ(ALARM, 1),
	DECLARE_IRQ(SECOND, 1),
	DECLARE_IRQ(ONKEYR, 2),
	DECLARE_IRQ(ONKEYR, 2),
	DECLARE_IRQ(EXTON1R, 2),
	DECLARE_IRQ(EXTON1F, 2),
	DECLARE_IRQ(EXTON2R, 2),
	DECLARE_IRQ(EXTON2F, 2),
	DECLARE_IRQ(EXTON3R, 2),
	DECLARE_IRQ(EXTON3F, 2),
	DECLARE_IRQ(BATFULL, 3),
	DECLARE_IRQ(CHGHALT, 3),
	DECLARE_IRQ(THLIMON, 3),
	DECLARE_IRQ(THLIMOFF, 3),
	DECLARE_IRQ(USBLIMON, 3),
	DECLARE_IRQ(USBLIMOFF, 3),
	DECLARE_IRQ(ADCRDY, 3),
	DECLARE_IRQ(ONKEY1S, 3),
	DECLARE_IRQ(LOWSYS, 4),
	DECLARE_IRQ(LOWBAT, 4),
	DECLARE_IRQ(HIGHTMP, 4),
	DECLARE_IRQ(AUTOPWRFAIL, 4),
	DECLARE_IRQ(DWN1PWRFAIL, 4),
	DECLARE_IRQ(DWN2PWRFAIL, 4),
	DECLARE_IRQ(LEDPWRFAIL,	 4),
	DECLARE_IRQ(LEDOVP, 4),
	DECLARE_IRQ(LDO1PWRFAIL, 5),
	DECLARE_IRQ(LDO1PWRFAIL, 5),
	DECLARE_IRQ(LDO1PWRFAIL, 5),
	DECLARE_IRQ(LDO1PWRFAIL, 5),
	DECLARE_IRQ(LDO1PWRFAIL, 5),
	DECLARE_IRQ(LDO1PWRFAIL, 5),
	DECLARE_IRQ(HCLDOPWRFAIL, 5),
	DECLARE_IRQ(HCLDOOVL, 5),
	
};

static struct regmap_irq_chip pcf50633_regmap_irq_chip = {
	.name = "pcf50633-irq",
	.status_base = PCF50633_REG_INT1,
	.mask_base = PCF50633_REG_INT1M,
	.ack_base = PCF50633_REG_INT1,
	.num_regs = PCF50633_NUM_INT_REGS,
	.irqs = pcf50633_irqs,
	.num_irqs = ARRAY_SIZE(pcf50633_irqs),
};


/* Maximum amount of time ONKEY is held before emergency action is taken */
#define PCF50633_ONKEY1S_TIMEOUT 8

static irqreturn_t pcf50633_irq(int irq, void *data)
{
	struct pcf50633 *pcf = data;
	int ret, i, j;
	u8 pcf_int[5], chgstat;

	/* Read the 5 INT regs in one transaction */
	ret = pcf50633_read_block(pcf, PCF50633_REG_INT1,
						ARRAY_SIZE(pcf_int), pcf_int);
	if (ret != ARRAY_SIZE(pcf_int)) {
		dev_err(pcf->dev, "Error reading INT registers\n");

		/*
		 * If this doesn't ACK the interrupt to the chip, we'll be
		 * called once again as we're level triggered.
		 */
		goto out;
	}
	dev_info(pcf->dev, "irq: INT registers %#.2x %#.2x %#.2x %#.2x %#.2x\n",
	         pcf_int[0], pcf_int[1], pcf_int[2], pcf_int[3], pcf_int[4]);

	/* defeat 8s death from lowsys on A5 */
	pcf50633_reg_write(pcf, PCF50633_REG_OOCSHDWN,  0x04);

	/* We immediately read the usb and adapter status. We thus make sure
	 * only of USBINS/USBREM IRQ handlers are called */
	if (pcf_int[0] & (PCF50633_INT1_USBINS | PCF50633_INT1_USBREM)) {
		chgstat = pcf50633_reg_read(pcf, PCF50633_REG_MBCS2);
		if (chgstat & (0x3 << 4))
			pcf_int[0] &= ~PCF50633_INT1_USBREM;
		else
			pcf_int[0] &= ~PCF50633_INT1_USBINS;
	}

	/* Make sure only one of ADPINS or ADPREM is set */
	if (pcf_int[0] & (PCF50633_INT1_ADPINS | PCF50633_INT1_ADPREM)) {
		chgstat = pcf50633_reg_read(pcf, PCF50633_REG_MBCS2);
		if (chgstat & (0x3 << 4))
			pcf_int[0] &= ~PCF50633_INT1_ADPREM;
		else
			pcf_int[0] &= ~PCF50633_INT1_ADPINS;
	}

	dev_dbg(pcf->dev, "INT1=0x%02x INT2=0x%02x INT3=0x%02x "
			"INT4=0x%02x INT5=0x%02x\n", pcf_int[0],
			pcf_int[1], pcf_int[2], pcf_int[3], pcf_int[4]);

	/* Some revisions of the chip don't have a 8s standby mode on
	 * ONKEY1S press. We try to manually do it in such cases. */
	if ((pcf_int[0] & PCF50633_INT1_SECOND) && pcf->onkey1s_held) {
		dev_info(pcf->dev, "ONKEY1S held for %d secs\n",
							pcf->onkey1s_held);
		if (pcf->onkey1s_held++ == pcf->pdata->force_shutdown_timeout)
				pcf50633_reg_set_bit_mask(pcf, PCF50633_REG_OOCSHDWN,  0x01, 0x01);
	}

	if (pcf_int[2] & PCF50633_INT3_ONKEY1S) {
		dev_info(pcf->dev, "ONKEY1S held\n");
		pcf->onkey1s_held = 1 ;

		/* Unmask IRQ_SECOND */
		pcf50633_reg_clear_bits(pcf, PCF50633_REG_INT1M,
						PCF50633_INT1_SECOND);

		/* Unmask IRQ_ONKEYR */
		pcf50633_reg_clear_bits(pcf, PCF50633_REG_INT2M,
						PCF50633_INT2_ONKEYR);
	}

	if ((pcf_int[1] & PCF50633_INT2_ONKEYR) && pcf->onkey1s_held) {
		pcf->onkey1s_held = 0;

		/* Mask SECOND and ONKEYR interrupts */
		if (pcf->mask_regs[0] & PCF50633_INT1_SECOND)
			pcf50633_reg_set_bit_mask(pcf,
					PCF50633_REG_INT1M,
					PCF50633_INT1_SECOND,
					PCF50633_INT1_SECOND);

		if (pcf->mask_regs[1] & PCF50633_INT2_ONKEYR)
			pcf50633_reg_set_bit_mask(pcf,
					PCF50633_REG_INT2M,
					PCF50633_INT2_ONKEYR,
					PCF50633_INT2_ONKEYR);
	}

	/* Have we just resumed ? */
	if (pcf->is_suspended) {
		pcf->is_suspended = 0;

		/* Set the resume reason filtering out non resumers */
		for (i = 0; i < ARRAY_SIZE(pcf_int); i++)
			pcf->resume_reason[i] = pcf_int[i] &
						pcf->pdata->resumers[i];

		/* Make sure we don't pass on any ONKEY events to
		 * userspace now */
		pcf_int[1] &= ~(PCF50633_INT2_ONKEYR | PCF50633_INT2_ONKEYF);
	}

	for (i = 0; i < ARRAY_SIZE(pcf_int); i++) {
		/* Unset masked interrupts */
		pcf_int[i] &= ~pcf->mask_regs[i];

		for (j = 0; j < 8 ; j++) {
			if (pcf_int[i] & (1 << j))
				handle_nested_irq(pcf->irqs[i*8 + j]);
		}
	}

out:
	return IRQ_HANDLED;
}

#ifdef CONFIG_PM

int pcf50633_irq_suspend(struct pcf50633 *pcf)
{
	int ret;
	int i;
	u8 res[5];


	/* Make sure our interrupt handlers are not called
	 * henceforth */
	disable_irq(pcf->irq);

	/*	Save the masks	*
	 *	ret = pcf50633_read_block(pcf, PCF50633_REG_INT1M,
	 *		ARRAY_SIZE(pcf->suspend_irq_masks),
	 *				pcf->suspend_irq_masks);
	 *	if (ret < 0) {
	 *		dev_err(pcf->dev, "error saving irq masks\n");
	 *		goto out;
	 *	}
	 */

	/* Write wakeup irq masks */
	for (i = 0; i < ARRAY_SIZE(res); i++)
		res[i] = ~pcf->pdata->resumers[i];

	ret = pcf50633_write_block(pcf, PCF50633_REG_INT1M,
					ARRAY_SIZE(res), &res[0]);
	if (ret < 0) {
		dev_err(pcf->dev, "error writing wakeup irq masks\n");
		goto out;
	}

	pcf->is_suspended = 1;

out:
	return ret;
}

int pcf50633_irq_resume(struct pcf50633 *pcf)
{
	int ret;

	/* Write the saved mask registers */
	ret = pcf50633_write_block(pcf, PCF50633_REG_INT1M,
				ARRAY_SIZE(pcf->mask_regs),
					pcf->mask_regs);
	if (ret < 0)
		dev_err(pcf->dev, "Error restoring saved suspend masks\n");

	enable_irq(pcf->irq);

	return ret;
}

#endif

static int pcf50633_irq_map(struct irq_domain *h, unsigned int virq, irq_hw_number_t hw)
{
	struct pcf50633 * pcf = h->host_data;
	dev_info(pcf->dev, "irq map %lu -> %u\n", hw, virq);

	irq_set_chip_data(virq, h->host_data);
	irq_set_chip(virq, &pcf50633_irq_chip);
	__irq_set_handler(virq, handle_simple_irq, 0, NULL);
	irq_set_irq_type(virq, IRQ_TYPE_NONE);

	return 0;
}

static struct irq_domain_ops pcf50633_irq_ops = {
	.map = pcf50633_irq_map,
};

int pcf50633_irq_init(struct pcf50633 *pcf, int irq)
{
	struct regmap_irq_chip_data *chip_data = NULL;
	int ret;
	int i;

	/* mask all interrupts */
	memset(pcf->mask_regs, 0xff, 5);
	ret = regmap_raw_write(pcf->regmap, PCF50633_REG_INT1M, pcf->mask_regs, ARRAY_SIZE(pcf->mask_regs));
	if (ret != 0)
		return ret;
	
	/*irq_base = irq_alloc_descs(-1, 0, PCF50633_NUM_IRQ, 0);*/
	pcf->irqd = irq_domain_add_linear(pcf->dev->of_node,
	                       PCF50633_NUM_IRQ, &pcf50633_irq_ops, pcf);
	if (IS_ERR(pcf->irqd)) {
		dev_err(pcf->dev, "Failed to allocate irq domain: %ld\n", PTR_ERR(pcf->irqd));
		return PTR_ERR(pcf->irqd);
	}
	dev_info(pcf->dev, "Allocated irq domain\n");

	mutex_init(&pcf->irq_lock);
	pcf->irq = irq;
	pcf->irq_base = pcf->pdata->irq_base ?: -1;

	/* for (i = irq_base; i < irq_base + PCF50633_NUM_IRQ; ++i) {
		irq_set_chip_data(i, pcf);
		irq_set_nested_thread(i, 1);
		
		//irq_set_chip_and_handler(i, &pcf50633_irq_chip, handle_simple_irq);
		irq_set_chip(i, &pcf50633_irq_chip);
		__irq_set_handler(i, handle_simple_irq, 0, NULL);
		
		irq_modify_status(i, IRQ_NOREQUEST, IRQ_NOPROBE);
	}*/
	for (i = pcf->irq_base; i < pcf->irq_base + PCF50633_NUM_IRQ; ++i) {
		int irq = pcf->irqs[i] = irq_create_mapping(pcf->irqd, i);
		if (i <= 0) {
			dev_err(pcf->dev, "Unable to allocate pcf mapped irq %d\n", i);
			kfree(pcf->irqd);
			return -ENODEV;
		}
		
		irq_set_chip_data(irq, pcf);
		irq_set_nested_thread(irq, 1);
		irq_set_chip(irq, &pcf50633_irq_chip);
		__irq_set_handler(irq, handle_simple_irq, 0, NULL);
	}

	/*ret = regmap_add_irq_chip(pcf->regmap, pcf->irq,
	             IRQF_TRIGGER_LOW | IRQF_ONESHOT,
	             pcf->irq_base, &pcf50633_regmap_irq_chip, &chip_data);
	if (ret < 0) {
		dev_err(pcf->dev, "Failed to add irq chip: %d\n", ret);
		goto err_irq_free_descs;
	}

	pcf->irq_base = regmap_irq_chip_get_base(chip_data);*/

	ret = request_threaded_irq(irq, NULL, pcf50633_irq,
					IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					"pcf50633", pcf);
	if (ret) {
		dev_err(pcf->dev, "Failed to request IRQ %d\n", ret);
		goto err_irq_free_descs;
	}

	if (enable_irq_wake(irq) < 0)
		dev_err(pcf->dev, "IRQ %u cannot be enabled as wake-up source"
			"in this hardware revision", irq);


	return 0;

err_irq_free_descs:
	kfree(pcf->irqd);
	//irq_free_descs(pcf->irq_base, PCF50633_NUM_IRQ);

	return ret;
}

void pcf50633_irq_free(struct pcf50633 *pcf)
{
	kfree(pcf->irqd);
	//regmap_del_irq_chip(pcf->irq, irq_get_irq_data(pcf->irq)->chip_data);
	//free_irq(pcf->irq, pcf);
	//irq_free_descs(pcf->irq_base, PCF50633_NUM_IRQ);
}
