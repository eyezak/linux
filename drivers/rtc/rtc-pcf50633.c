/* NXP PCF50633 RTC Driver
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
 */

#include <linux/mfd/pcf50633/core.h>

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/err.h>

enum pcf50633_time_indexes {
	PCF50633_TI_SEC,
	PCF50633_TI_MIN,
	PCF50633_TI_HOUR,
	PCF50633_TI_WKDAY,
	PCF50633_TI_DAY,
	PCF50633_TI_MONTH,
	PCF50633_TI_YEAR,
	PCF50633_TI_EXTENT /* always last */
};

struct pcf50633_time {
	u_int8_t time[PCF50633_TI_EXTENT];
};

struct pcf50633_rtc {
	struct device *dev;
	struct rtc_device *rtc_dev;

	unsigned int irq_alarm;
	unsigned int irq_second;
	int alarm_enabled:1;
	int alarm_pending:1;
};

static void pcf2rtc_time(struct rtc_time *rtc, struct pcf50633_time *pcf)
{
	rtc->tm_sec = bcd2bin(pcf->time[PCF50633_TI_SEC]);
	rtc->tm_min = bcd2bin(pcf->time[PCF50633_TI_MIN]);
	rtc->tm_hour = bcd2bin(pcf->time[PCF50633_TI_HOUR]);
	rtc->tm_wday = bcd2bin(pcf->time[PCF50633_TI_WKDAY]);
	rtc->tm_mday = bcd2bin(pcf->time[PCF50633_TI_DAY]);
	rtc->tm_mon = bcd2bin(pcf->time[PCF50633_TI_MONTH]) - 1;
	rtc->tm_year = bcd2bin(pcf->time[PCF50633_TI_YEAR]) + 100;
}

static void rtc2pcf_time(struct pcf50633_time *pcf, struct rtc_time *rtc)
{
	pcf->time[PCF50633_TI_SEC] = bin2bcd(rtc->tm_sec);
	pcf->time[PCF50633_TI_MIN] = bin2bcd(rtc->tm_min);
	pcf->time[PCF50633_TI_HOUR] = bin2bcd(rtc->tm_hour);
	pcf->time[PCF50633_TI_WKDAY] = bin2bcd(rtc->tm_wday);
	pcf->time[PCF50633_TI_DAY] = bin2bcd(rtc->tm_mday);
	pcf->time[PCF50633_TI_MONTH] = bin2bcd(rtc->tm_mon + 1);
	pcf->time[PCF50633_TI_YEAR] = bin2bcd(rtc->tm_year % 100);
}

static int
pcf50633_rtc_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	struct pcf50633_rtc *rtc = dev_get_drvdata(dev);

	if (enabled && !rtc->alarm_enabled)
		enable_irq(rtc->irq_alarm);
	else if (!enabled && rtc->alarm_enabled)
		disable_irq(rtc->irq_alarm);

	rtc->alarm_enabled = enabled;
	dev_info(dev, "alarm irq %s\n", enabled ? "enabled" : "disabled");

	return 0;
}

static int pcf50633_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct pcf50633_rtc *rtc;
	struct pcf50633_time pcf_tm;
	int ret;

	rtc = dev_get_drvdata(dev);

	ret = pcf50633_read_block(child_to_pcf50633(rtc), PCF50633_REG_RTCSC,
					    PCF50633_TI_EXTENT,
					    &pcf_tm.time[0]);
	if (ret != 0) {
		dev_err(dev, "Failed to read time\n");
		return -EIO;
	}

	pcf2rtc_time(tm, &pcf_tm);

	dev_dbg(dev, "get time %.2u.%.2u.%.4u %.2u:%.2u:%.2u\n",
		tm->tm_mday, tm->tm_mon, tm->tm_year + 1900,
		tm->tm_hour, tm->tm_min, tm->tm_sec);

	return rtc_valid_tm(tm);
}

static int pcf50633_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct pcf50633_rtc *rtc;
	struct pcf50633_time pcf_tm;
	int ret = 0;

	rtc = dev_get_drvdata(dev);

	rtc2pcf_time(&pcf_tm, tm);

	dev_dbg(dev, "set time %.2u.%.2u.%.4u %.2u:%.2u:%.2u\n",
		tm->tm_mday, tm->tm_mon, tm->tm_year + 1900,
		tm->tm_hour, tm->tm_min, tm->tm_sec);


	//alarm_masked = pcf50633_irq_mask_get(child_to_pcf50633(rtc), PCF50633_IRQ_ALARM);
	if (rtc->alarm_enabled)
		disable_irq(rtc->irq_alarm);

	/* Returns 0 on success */
	ret = pcf50633_write_block(child_to_pcf50633(rtc), PCF50633_REG_RTCSC,
					     PCF50633_TI_EXTENT,
					     &pcf_tm.time[0]);

	if (rtc->alarm_enabled)
		enable_irq(rtc->irq_alarm);

	return ret;
}

static int pcf50633_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct pcf50633_rtc *rtc;
	struct pcf50633_time pcf_tm;
	int ret = 0;

	rtc = dev_get_drvdata(dev);

	alrm->enabled = rtc->alarm_enabled;
	alrm->pending = rtc->alarm_pending;

	ret = pcf50633_read_block(child_to_pcf50633(rtc), PCF50633_REG_RTCSCA,
				PCF50633_TI_EXTENT, &pcf_tm.time[0]);
	if (ret != 0) {
		dev_err(dev, "Failed to read time\n");
		return -EIO;
	}

	pcf2rtc_time(&alrm->time, &pcf_tm);

	ret = rtc_valid_tm(&alrm->time);
	if (ret != 0) {
		dev_err(dev, "Read invalid time\n");
		return -EIO;
	}
	return 0;
}

static int pcf50633_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct pcf50633_rtc *rtc;
	struct pcf50633_time pcf_tm;
	int ret = 0;

	rtc = dev_get_drvdata(dev);

	rtc2pcf_time(&pcf_tm, &alrm->time);

	/* do like mktime does and ignore tm_wday */
	pcf_tm.time[PCF50633_TI_WKDAY] = 7;

	dev_dbg(dev, "set alarm %.2u.%.2u.%.4u %.2u:%.2u:%.2u\n",
		alrm->time.tm_mday, alrm->time.tm_mon, alrm->time.tm_year + 1900,
		alrm->time.tm_hour, alrm->time.tm_min, alrm->time.tm_sec);

	/* disable alarm interrupt */
	if (rtc->alarm_enabled)
		disable_irq(rtc->irq_alarm);

	/* Returns 0 on success */
	ret = pcf50633_write_block(child_to_pcf50633(rtc), PCF50633_REG_RTCSCA,
				PCF50633_TI_EXTENT, &pcf_tm.time[0]);
	if (!alrm->enabled)
		rtc->alarm_pending = 0;

	if (!rtc->alarm_enabled || alrm->enabled)
		enable_irq(rtc->irq_alarm);
	rtc->alarm_enabled = alrm->enabled;

	return ret;
}

static int rtc_open(struct device *dev)
{
	//struct pcf50633_rtc * rtc = dev_get_drvdata(dev);
	//enable_irq(rtc->irq_second);

	dev_dbg(dev, "OPEN\n");
	return 0;
}

static void rtc_release(struct device *dev)
{
	//struct pcf50633_rtc * rtc = dev_get_drvdata(dev);
	//disable_irq(rtc->irq_second);	

	dev_dbg(dev, "RELEASE\n");
}

static int rtc_ioctl(struct device *dev, unsigned int cmd, unsigned long arg)
{
	dev_dbg(dev, "IOCTL  %u: %lu\n", cmd, arg);
	return -ENOIOCTLCMD;
}

static int rtc_proc(struct device *dev, struct seq_file *file)
{
	dev_dbg(dev, "PROC\n");
	return 0;
}

static struct rtc_class_ops pcf50633_rtc_ops = {
	.open			= rtc_open,
	.ioctl          = rtc_ioctl,
	.release		= rtc_release,
	.proc			= rtc_proc,
	
	.read_time		= pcf50633_rtc_read_time,
	.set_time		= pcf50633_rtc_set_time,
	.read_alarm		= pcf50633_rtc_read_alarm,
	.set_alarm		= pcf50633_rtc_set_alarm,
	.alarm_irq_enable	= pcf50633_rtc_alarm_irq_enable,
};

static irqreturn_t pcf50633_rtc_irq(int irq, void *data)
{
	struct pcf50633_rtc *rtc = data;
	dev_dbg(rtc->dev, "IRQ %d: %s\n", irq, (irq == rtc->irq_alarm) ? "alarm" : "second");

	if (irq == rtc->irq_alarm) {
		rtc_update_irq(rtc->rtc_dev, 1, RTC_AF | RTC_IRQF);
		rtc->alarm_pending = 1;
	} else if (irq == rtc->irq_second) {
		rtc_update_irq(rtc->rtc_dev, 1, RTC_PF | RTC_IRQF);
	} else
		return IRQ_NONE;

	return IRQ_HANDLED;
}

static int __devinit pcf50633_rtc_probe(struct platform_device *pdev)
{
	int ret;
	struct pcf50633_rtc *rtc;

	rtc = kzalloc(sizeof(*rtc), GFP_KERNEL);
	if (!rtc)
		return -ENOMEM;

	rtc->irq_second = platform_get_irq_byname(pdev, "SECOND");
	if (rtc->irq_second <= 0) {
		ret = rtc->irq_second ?: -EINVAL;
		dev_err(&pdev->dev, "Failed to get second irq: %d\n", ret);
		goto err_free;
	}

	rtc->irq_alarm = platform_get_irq_byname(pdev, "ALARM");
	if (rtc->irq_alarm <= 0) {
		ret = rtc->irq_alarm ?: -EINVAL;
		dev_err(&pdev->dev, "Failed to get alarm irq: %d\n", ret);
		goto err_free;
	}


	rtc->dev = &pdev->dev;
	platform_set_drvdata(pdev, rtc);
	rtc->rtc_dev = rtc_device_register("pcf50633-rtc", &pdev->dev,
				&pcf50633_rtc_ops, THIS_MODULE);

	if (IS_ERR(rtc->rtc_dev)) {
		ret =  PTR_ERR(rtc->rtc_dev);
		goto err_free;
	}

	//disable_irq(rtc->irq_alarm);
	rtc->alarm_enabled = 1;
	ret = request_threaded_irq(rtc->irq_alarm, NULL, 
                         pcf50633_rtc_irq, 0, "pcf50633-rtc:alarm", rtc);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request alarm irq: %d\n", ret);
		goto err_free;
	}

	/*ret = request_threaded_irq(rtc->irq_second, NULL, 
                         pcf50633_rtc_irq, 0, "pcf50633-rtc:second", rtc);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request second irq: %d\n", ret);
		goto err_free_irq_alarm;
	}
	disable_irq(rtc->irq_second);*/
	
	return 0;

/*err_free_irq_alarm:
	free_irq(rtc->irq_alarm, rtc); */
err_free:
	kfree(rtc);
	return ret;
}

static int __devexit pcf50633_rtc_remove(struct platform_device *pdev)
{
	struct pcf50633_rtc *rtc;

	rtc = platform_get_drvdata(pdev);

	free_irq(rtc->irq_alarm, rtc);
	//free_irq(rtc->irq_second, rtc);

	rtc_device_unregister(rtc->rtc_dev);
	kfree(rtc);

	return 0;
}

static struct platform_driver pcf50633_rtc_driver = {
	.driver = {
		.name = "pcf50633-rtc",
	},
	.probe = pcf50633_rtc_probe,
	.remove = __devexit_p(pcf50633_rtc_remove),
};

module_platform_driver(pcf50633_rtc_driver);

MODULE_DESCRIPTION("PCF50633 RTC driver");
MODULE_AUTHOR("Balaji Rao <balajirrao@openmoko.org>");
MODULE_LICENSE("GPL");

