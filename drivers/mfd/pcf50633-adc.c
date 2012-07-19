/* NXP PCF50633 ADC Driver
 *
 * (C) 2006-2008 by Openmoko, Inc.
 * Author: Balaji Rao <balajirrao@openmoko.org>
 * All rights reserved.
 *
 * Broken down from monstrous PCF50633 driver mainly by
 * Harald Welte, Andy Green and Werner Almesberger
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  NOTE: This driver does not yet support subtractive ADC mode, which means
 *  you can do only one measurement per read request.
 */

#include <linux/mfd/pcf50633/core.h>
#include <linux/mfd/pcf50633/adc.h>

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>

struct pcf50633_adc_sync_request {
	int result;
	struct completion completion;
};

static void adc_setup(struct pcf50633 *pcf, struct pcf50633_adc_channel *c)
{
	/* kill ratiometric, but enable ACCSW biasing */
	dev_dbg(pcf->adc->dev, "adc_setup %#.2x %#.2x %#.2x\n",
	        c->c1flags | PCF50633_ADCC1_ADCSTART,
	        c->c2flags, c->c3flags);
	pcf50633_reg_write(pcf, PCF50633_REG_ADCC2, c->c2flags);
	pcf50633_reg_write(pcf, PCF50633_REG_ADCC3, c->c3flags);

	/* start ADC conversion on selected channel */
	pcf50633_reg_write(pcf, PCF50633_REG_ADCC1, (c->c1flags & 0x7f) | 0x01);
	//                   (channel & PCF50633_ADCC1_ADCMUX_MASK) | avg |
	//                   PCF50633_ADCC1_ADCSTART | PCF50633_ADCC1_RES_10BIT);
}

static void trigger_next_adc_job_if_any(struct pcf50633_adc *adc)
{
	int head;

	head = adc->queue_head;
	if (!adc->queue[head])
		return;

	adc_setup(child_to_pcf50633(adc), adc->channels + adc->queue[head]->channel);
}

static int
adc_enqueue_request(struct pcf50633_adc *adc, struct pcf50633_adc_request *req)
{
	int head, tail;

	mutex_lock(&adc->queue_mutex);

	head = adc->queue_head;
	tail = adc->queue_tail;

	if (adc->queue[tail]) {
		mutex_unlock(&adc->queue_mutex);
		dev_err(adc->dev, "ADC queue is full, dropping request\n");
		return -EBUSY;
	}

	adc->queue[tail] = req;
	if (head == tail)
		trigger_next_adc_job_if_any(adc);
	adc->queue_tail = (tail + 1) & (PCF50633_MAX_ADC_FIFO_DEPTH - 1);

	mutex_unlock(&adc->queue_mutex);

	return 0;
}

static void pcf50633_adc_sync_read_callback(struct pcf50633 *pcf, void *param,
	int result)
{
	struct pcf50633_adc_sync_request *req = param;

	req->result = result;
	complete(&req->completion);
}

int pcf50633_adc_sync_read(struct pcf50633 *pcf, int channel)
{
	struct pcf50633_adc_sync_request req;
	int ret;

	init_completion(&req.completion);

	ret = pcf50633_adc_async_read(pcf, channel,
		                          pcf50633_adc_sync_read_callback, &req);
	if (ret)
		return ret;

	wait_for_completion(&req.completion);

	return req.result;
}
EXPORT_SYMBOL_GPL(pcf50633_adc_sync_read);

int pcf50633_adc_async_read(struct pcf50633 *pcf, int channel,
			     void (*callback)(struct pcf50633 *, void *, int),
			     void *callback_param)
{
	struct pcf50633_adc_request *req;

	if (!(pcf->adc->channel_mask & (1 << channel)))
		return -EINVAL;

	/* req is freed when the result is ready, in interrupt handler */
	req = kmalloc(sizeof(*req), GFP_KERNEL);
	if (!req)
		return -ENOMEM;

	req->channel = channel;
	req->callback = callback;
	req->callback_param = callback_param;

	return adc_enqueue_request(pcf->adc, req);
}
EXPORT_SYMBOL_GPL(pcf50633_adc_async_read);

static int adc_result(struct pcf50633_adc *adc)
{
	struct pcf50633 *pcf = child_to_pcf50633(adc);
	u8 adcs1, adcs3;
	u16 result;

	adcs1 = pcf50633_reg_read(pcf, PCF50633_REG_ADCS1);
	adcs3 = pcf50633_reg_read(pcf, PCF50633_REG_ADCS3);
	result = (adcs1 << 2) | (adcs3 & PCF50633_ADCS3_ADCDAT1L_MASK);

	dev_dbg(adc->dev, "adc result = %d wrt %d\n", result, (adcs3 & 0x70) >> 4);

	return result;
}

static ssize_t pcf50633_adc_attr_show(struct device *dev,
                         struct device_attribute *attr, char *buf);

static DEVICE_ATTR(adcin1, S_IRUGO, pcf50633_adc_attr_show, NULL);
static DEVICE_ATTR(adcin2, S_IRUGO, pcf50633_adc_attr_show, NULL);
static DEVICE_ATTR(battemp, S_IRUGO, pcf50633_adc_attr_show, NULL);
static DEVICE_ATTR(batsns, S_IRUGO, pcf50633_adc_attr_show, NULL);

static ssize_t pcf50633_adc_attr_show(struct device *dev,
                         struct device_attribute *attr, char *buf)
{
	struct pcf50633_adc *adc = dev_get_drvdata(dev);
	int i, maxval, result, whole, frac;

	if (attr == &dev_attr_adcin1)
		i = 0;
	else if (attr == &dev_attr_adcin2)
		i = 1;
	else if (attr == &dev_attr_battemp)
		i = 2;
	else if (attr == &dev_attr_batsns)
		i = 3;
	else
		return -EINVAL;
	
	maxval = (adc->channels[i].c1flags & PCF50633_ADCC1_RES_8BIT) ? 255 : 1023;

	result = pcf50633_adc_sync_read(child_to_pcf50633(adc), i);
	if (result < 0)
		return result;
	
	if (adc->channels[i].m) {
		result = result * adc->channels[i].m + adc->channels[i].b;
		whole =  result / maxval;
		frac = result * 1000 / maxval - (whole * 1000);
	} else {
		whole = 0;
		frac = result * 1000 / maxval;
	}
	return snprintf(buf, PAGE_SIZE, "%.1d.%.3d\n", whole, frac);
}

static umode_t pcf50633_adc_sysfs_attr_is_visible(struct kobject * kobj,
                           struct attribute * attr, int index)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct pcf50633_adc * adc = dev_get_drvdata(dev);
	int i;
	
	if (attr == &dev_attr_adcin1.attr)
		i = 0;
	else if (attr == &dev_attr_adcin2.attr)
		i = 1;
	else if (attr == &dev_attr_battemp.attr)
		i = 2;
	else if (attr == &dev_attr_batsns.attr)
		i = 3;
	else
		return 0;

	if (adc->channel_mask & (1 << i))
		return attr->mode ?: 0444;

	return 0;
}

static struct attribute_group adc_attr_group = {
	.name		= NULL,      /* put in device directory */
	.is_visible	= pcf50633_adc_sysfs_attr_is_visible,
	.attrs		= (struct attribute *[]) {
		&dev_attr_adcin1.attr,
		&dev_attr_adcin2.attr,
		&dev_attr_battemp.attr,
		&dev_attr_batsns.attr,
		NULL,
	},
};

static irqreturn_t pcf50633_adc_irq(int irq, void *data)
{
	struct pcf50633_adc *adc = data;
	struct pcf50633 *pcf = child_to_pcf50633(adc);
	struct pcf50633_adc_request *req;
	int head, res;

	mutex_lock(&adc->queue_mutex);
	head = adc->queue_head;

	req = adc->queue[head];
	if (WARN_ON(!req)) {
		dev_err(adc->dev, "pcf50633-adc irq: ADC queue empty!\n");
		mutex_unlock(&adc->queue_mutex);
		return IRQ_HANDLED;
	}
	adc->queue[head] = NULL;
	adc->queue_head = (head + 1) &
				      (PCF50633_MAX_ADC_FIFO_DEPTH - 1);

	res = adc_result(adc);
	trigger_next_adc_job_if_any(adc);

	mutex_unlock(&adc->queue_mutex);

	req->callback(pcf, req->callback_param, res);
	kfree(req);

	return IRQ_HANDLED;
}

static int __devinit pcf50633_adc_probe(struct platform_device *pdev)
{
	struct pcf50633_adc *adc;
	struct pcf50633_adc_platform_data *pdata;
	int ret;

	adc = kzalloc(sizeof(*adc), GFP_KERNEL);
	if (!adc)
		return -ENOMEM;

	adc->dev = &pdev->dev;
	platform_set_drvdata(pdev, adc);

	pdata = pdev->dev.platform_data;
	if (pdata) {
		adc->channel_mask = pdata->channel_mask;
		memcpy(adc->channels, pdata->channels,
		       sizeof(struct pcf50633_adc_channel) * 4);
	}

	adc->irq = platform_get_irq(pdev, 0);
	if (adc->irq <= 0) {
		ret = adc->irq;
		dev_err(&pdev->dev, "Failed to get irq: %d\n", ret);
		goto err_free;
	}
	
	ret = request_threaded_irq(adc->irq, NULL, pcf50633_adc_irq, 0,
	                           dev_name(&pdev->dev), adc);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request irq: %d\n", ret);
		goto err_free;
	} else {
		dev_dbg(&pdev->dev, "Acquired irq %u\n", adc->irq) ;
	}

	mutex_init(&adc->queue_mutex);
	child_to_pcf50633(adc)->adc = adc;

	ret = sysfs_create_group(&pdev->dev.kobj, &adc_attr_group);
	if (ret)
		dev_err(&pdev->dev, "failed to create sysfs entries\n");

	return 0;

err_free:
	kfree(adc);
	return ret;
}

static int __devexit pcf50633_adc_remove(struct platform_device *pdev)
{
	struct pcf50633_adc *adc = platform_get_drvdata(pdev);
	int i, head;

	sysfs_remove_group(&pdev->dev.kobj, &adc_attr_group);
	free_irq(adc->irq, adc);

	mutex_lock(&adc->queue_mutex);
	head = adc->queue_head;

	if (WARN_ON(adc->queue[head]))
		dev_err(pdev->dev.parent,
			"adc driver removed with request pending\n");

	for (i = 0; i < PCF50633_MAX_ADC_FIFO_DEPTH; i++)
		kfree(adc->queue[i]);

	mutex_unlock(&adc->queue_mutex);
	
	child_to_pcf50633(adc)->adc = NULL;
	platform_set_drvdata(pdev, NULL);

	kfree(adc);

	return 0;
}

static struct platform_driver pcf50633_adc_driver = {
	.driver = {
		.name = "pcf50633-adc",
	},
	.probe = pcf50633_adc_probe,
	.remove = __devexit_p(pcf50633_adc_remove),
};

module_platform_driver(pcf50633_adc_driver);

MODULE_AUTHOR("Balaji Rao <balajirrao@openmoko.org>");
MODULE_DESCRIPTION("PCF50633 adc driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pcf50633-adc");

