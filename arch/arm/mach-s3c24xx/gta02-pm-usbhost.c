/*
 * USBHOST Management code for the Openmoko Freerunner GSM Phone
 *
 * (C) 2007 by Openmoko Inc.
 * Author: Harald Welte <laforge@openmoko.org>
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/console.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/regulator/consumer.h>

#include <asm/io.h>
#include <asm/mach-types.h>
#include <mach/gpio.h>
#include <mach/gta02.h>
#include <mach/hardware.h>
#include <mach/regs-gpio.h>
#include <mach/regs-gpioj.h>

static inline struct regulator *dev_to_regulator(struct device *dev)
{
	return dev_get_drvdata(dev);
}

static ssize_t usbhost_read(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct regulator * regltr = dev_to_regulator(dev);

	if (!strcmp(attr->attr.name, "power_on")) {
		if (regulator_is_enabled(regltr))
			goto out_1;
	}

	return strlcpy(buf, "0\n", 3);
out_1:
	return strlcpy(buf, "1\n", 3);
}

static void usbhost_on_off(struct device *dev, int on)
{
	struct regulator * regltr = dev_to_regulator(dev);
	
	on = !!on;

	if (on == regulator_is_enabled(regltr))
		return;

	if (!on) {
		regulator_disable(regltr);
		return;
	}

	regulator_enable(regltr);
}

static ssize_t usbhost_write(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	unsigned long on = simple_strtoul(buf, NULL, 10);

	if (!strcmp(attr->attr.name, "power_on")) {
		usbhost_on_off(dev, on);

		return count;
	}

	return count;
}

static DEVICE_ATTR(power_on, 0644, usbhost_read, usbhost_write);

#ifdef CONFIG_PM

static int gta02_usbhost_suspend(struct device *dev)
{
	return 0;
}

static int gta02_usbhost_suspend_late(struct device *dev)
{
	return 0;
}

static int gta02_usbhost_resume(struct device *dev)
{
	return 0;
}

static struct dev_pm_ops gta02_usbhost_pm_ops = {
	.suspend	= gta02_usbhost_suspend,
	.suspend_noirq	= gta02_usbhost_suspend_late,
	.resume		= gta02_usbhost_resume,
};

#define GTA02_USBHOST_PM_OPS (&gta02_usbhost_pm_ops)

#else
#define GTA02_USBHOST_PM_OPS NULL
#endif /* CONFIG_PM */

/* switching of USB pads */
static ssize_t show_usb_mode(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	if (__raw_readl(S3C24XX_MISCCR) & S3C2410_MISCCR_USBHOST)
		return sprintf(buf, "host\n");

	return sprintf(buf, "device\n");
}

static ssize_t set_usb_mode(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	if (!strncmp(buf, "host", 4)) {
		printk("s3c2410: changing usb to host\n");
		s3c2410_modify_misccr(S3C2410_MISCCR_USBHOST,
				      S3C2410_MISCCR_USBHOST);
		/* FIXME:
		 * - call machine-specific disable-pullup function i
		 * - enable +Vbus (if hardware supports it)
		 */
		
		usbhost_on_off(dev, 0);
	} else if (!strncmp(buf, "device", 6)) {
		printk("s3c2410: changing usb to device\n");
		s3c2410_modify_misccr(S3C2410_MISCCR_USBHOST, 0);
		
		usbhost_on_off(dev, 1);
	} else {
		dev_err(dev, "usb_mode: unknown mode\n");
		return -EINVAL;
	}

	return count;
}

static DEVICE_ATTR(usb_mode, S_IRUGO | S_IWUSR, show_usb_mode, set_usb_mode);

static struct attribute_group gta02_usbhost_attr_group = {
	.name	= NULL,
	.attrs	= (struct attribute *[]) {
		&dev_attr_power_on.attr,
		&dev_attr_usb_mode.attr,
		NULL
	},
};

static int __devinit gta02_usbhost_probe(struct platform_device *pdev)
{
	struct regulator *regltr;
	int ret;

	regltr = regulator_get_exclusive(&pdev->dev, "USBHOST");

	if (IS_ERR(regltr)) {
		ret = PTR_ERR(regltr);
		dev_err(&pdev->dev, "Failed to get regulator: %d\n", ret);
		return ret;
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &gta02_usbhost_attr_group);
	if (ret) {
		dev_err(&pdev->dev, "Failed to create sysfs entries: %d\n", ret);
		return ret;
	}

	dev_set_drvdata(&pdev->dev, regltr);
	return 0;
}

static int gta02_usbhost_remove(struct platform_device *pdev)
{
	struct regulator *regltr = dev_to_regulator(&pdev->dev);

	usbhost_on_off(&pdev->dev, 0);

	sysfs_remove_group(&pdev->dev.kobj, &gta02_usbhost_attr_group);
	dev_set_drvdata(&pdev->dev, NULL);
	regulator_put(regltr);

	return 0;
}

static struct platform_driver gta02_usbhost_driver = {
	.probe		= gta02_usbhost_probe,
	.remove		= gta02_usbhost_remove,
	.driver		= {
		.name	= "gta02-pm-usbhost",
		.pm	= GTA02_USBHOST_PM_OPS,
	},
};

static int __devinit gta02_usbhost_init(void)
{
	return platform_driver_register(&gta02_usbhost_driver);
}
module_init(gta02_usbhost_init);

static void gta02_usbhost_exit(void)
{
	platform_driver_unregister(&gta02_usbhost_driver);
}
module_exit(gta02_usbhost_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Harald Welte <laforge@openmoko.org>");
MODULE_DESCRIPTION("Openmoko Freerunner USBHOST Power Management");
