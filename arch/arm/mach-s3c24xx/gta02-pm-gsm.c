/*
 * GSM Management code for the Openmoko Freerunner GSM Phone
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

#define DEBUG

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/console.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>

#include <asm/mach-types.h>
#include <mach/gpio.h>
#include <mach/gta02.h>
#include <mach/hardware.h>
#include <mach/regs-gpio.h>
#include <mach/regs-gpioj.h>

static int gta02_gsm_interrupts;

extern void s3c24xx_serial_console_set_silence(int);

struct gta02pm_priv {
	int gpio_ndl_gsm;
	struct console *con;
	struct regulator *regulator;
};

static struct gta02pm_priv gta02_gsm;

static struct console *find_s3c24xx_console(struct device * dev)
{
	struct console *con;

	console_lock();

	for_each_console(con) {
		dev_printk(KERN_INFO, dev, "found console %s\n", con->name);
		if (!strcmp(con->name, "ttySAC"))
			break;
	}

	console_unlock();

	return con;
}

static ssize_t gsm_read(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	if (!strcmp(attr->attr.name, "power_on")) {
		if (regulator_is_enabled(gta02_gsm.regulator))
			goto out_1;
	} else if (!strcmp(attr->attr.name, "download")) {
		if (!gpio_get_value(GTA02_GPIO_nDL_GSM))
			goto out_1;
	} else if (!strcmp(attr->attr.name, "flowcontrolled")) {
		if (s3c_gpio_getcfg(S3C2410_GPH(1)) == S3C2410_GPIO_OUTPUT)
			goto out_1;
	}

	return strlcpy(buf, "0\n", 3);
out_1:
	return strlcpy(buf, "1\n", 3);
}

static void gsm_reset(void)
{
	gpio_set_value(GTA02_GPIO_MODEM_ON, 1);
	msleep(500);
	gpio_set_value(GTA02_GPIO_MODEM_ON, 0);
}

static void gsm_on_off(struct device *dev, bool on)
{
	if (on == regulator_is_enabled(gta02_gsm.regulator))
		return;

	if (!on) {
		s3c_gpio_cfgpin(S3C2410_GPH(1), S3C2410_GPIO_INPUT);
		s3c_gpio_cfgpin(S3C2410_GPH(2), S3C2410_GPIO_INPUT);

		regulator_disable(gta02_gsm.regulator);

		return;
	}

	/* allow UART to talk to GSM side now we will power it */
	s3c_gpio_cfgpin(S3C2410_GPH(1), S3C2410_GPH1_nRTS0);
	s3c_gpio_cfgpin(S3C2410_GPH(2), S3C2410_GPH2_TXD0);

	regulator_enable(gta02_gsm.regulator);

	msleep(100);

	gsm_reset();
#ifdef CONFIG_GTA02_GSM_OLD
	/*
	 * workaround for calypso firmware moko10 and earlier,
	 * without this it will leave IRQ line high after
	 * booting
	 */
	s3c2410_gpio_setpin(S3C2410_GPH(1), 1);
	s3c_gpio_cfgpin(S3C2410_GPH(1), S3C2410_GPIO_OUTPUT);
	msleep(1000);
	s3c_gpio_cfgpin(S3C2410_GPH(1), S3C2410_GPH1_nRTS0);
#endif

}

static ssize_t gsm_write(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	int ret;
	unsigned long on;

	ret = strict_strtoul(buf, 10, &on);
	if (ret)
		return ret;

	if (!strcmp(attr->attr.name, "power_on")) {
		gsm_on_off(dev, on);

		return count;
	}

	if (!strcmp(attr->attr.name, "download")) {
		/*
		 * the keyboard / buttons driver requests and enables
		 * the JACK_INSERT IRQ.  We have to take care about
		 * not enabling and disabling the IRQ when it was
		 * already in that state or we get "unblanaced IRQ"
		 * kernel warnings and stack dumps.  So we use the
		 * copy of the ndl_gsm state to figure out if we should
		 * enable or disable the jack interrupt
		 */
		if (on) {
			if (gta02_gsm.gpio_ndl_gsm)
				disable_irq(gpio_to_irq(
						   GTA02_GPIO_JACK_INSERT));
		} else {
			if (!gta02_gsm.gpio_ndl_gsm)
				enable_irq(gpio_to_irq(
						   GTA02_GPIO_JACK_INSERT));
		}

		gta02_gsm.gpio_ndl_gsm = !on;
		gpio_set_value(GTA02_GPIO_nDL_GSM, !on);

		return count;
	}

	if (!strcmp(attr->attr.name, "flowcontrolled")) {
		if (on) {
			gta02_gsm_interrupts = 0;
			gpio_set_value(S3C2410_GPH(1), 1);
			s3c_gpio_cfgpin(S3C2410_GPH(1), S3C2410_GPIO_OUTPUT);
		} else
			s3c_gpio_cfgpin(S3C2410_GPH(1), S3C2410_GPH1_nRTS0);
	}
	
	if (!strcmp(attr->attr.name, "reset")) {
		if (on)
			gsm_reset();
	}

	return count;
}

static DEVICE_ATTR(power_on, 0644, gsm_read, gsm_write);
static DEVICE_ATTR(reset, 0644, gsm_read, gsm_write);
static DEVICE_ATTR(download, 0644, gsm_read, gsm_write);
static DEVICE_ATTR(flowcontrolled, 0644, gsm_read, gsm_write);

#ifdef CONFIG_PM

static int gta02_gsm_suspend(struct device *dev)
{
	/* GPIO state is saved/restored by S3C2410 core GPIO driver, so we
	 * don't need to do much here. */


	/* If flowcontrol asserted, abort if GSM already interrupted */
	if (s3c_gpio_getcfg(S3C2410_GPH(1)) == S3C2410_GPIO_OUTPUT) {
		if (gta02_gsm_interrupts)
			goto busy;
	}

	/* disable DL GSM to prevent jack_insert becoming 'floating' */
	gpio_set_value(GTA02_GPIO_nDL_GSM, 1);

	if (device_may_wakeup(dev))
		enable_irq_wake(GTA02_IRQ_MODEM);

	return 0;

busy:
	return -EBUSY;
}

static int gta02_gsm_suspend_late(struct device *dev)
{
	/* Last chance: abort if GSM already interrupted */
	if (s3c_gpio_getcfg(S3C2410_GPH(1)) == S3C2410_GPIO_OUTPUT) {
		if (gta02_gsm_interrupts)
			return -EBUSY;
	}
	return 0;
}

static int gta02_gsm_resume(struct device *dev)
{
	if (device_may_wakeup(dev))
		disable_irq_wake(GTA02_IRQ_MODEM);

	/* GPIO state is saved/restored by S3C2410 core GPIO driver, so we
	 * don't need to do much here. */

	gpio_set_value(GTA02_GPIO_nDL_GSM, gta02_gsm.gpio_ndl_gsm);

	return 0;
}

static const struct dev_pm_ops gta02_gsm_pm_ops = {
	.suspend	= gta02_gsm_suspend,
	.suspend_noirq	= gta02_gsm_suspend_late,
	.resume		= gta02_gsm_resume,
};

#define GTA02_GSM_PM_OPS (&gta02_gsm_pm_ops)

#else
#define GTA02_GSM_PM_OPS NULL
#endif /* CONFIG_PM */

static struct attribute *gta02_gsm_sysfs_entries[] = {
	&dev_attr_power_on.attr,
	&dev_attr_reset.attr,
	&dev_attr_download.attr,
	&dev_attr_flowcontrolled.attr,
	NULL
};

static struct attribute_group gta02_gsm_attr_group = {
	.name	= NULL,
	.attrs	= gta02_gsm_sysfs_entries,
};

static irqreturn_t gta02_gsm_irq(int irq, void *devid)
{
	gta02_gsm_interrupts++;
	return IRQ_HANDLED;
}

static int __devinit gta02_gsm_probe(struct platform_device *pdev)
{
	int ret;

	gta02_gsm.regulator = regulator_get_exclusive(&pdev->dev, "GSM");

	if (IS_ERR(gta02_gsm.regulator)) {
		ret = PTR_ERR(gta02_gsm.regulator);
		dev_err(&pdev->dev, "Failed to get regulator: %d\n", ret);
		return ret;
	}

	ret = request_irq(GTA02_IRQ_MODEM, gta02_gsm_irq,
			IRQF_DISABLED | IRQF_TRIGGER_RISING, "modem", NULL);

	if (ret) {
		regulator_put(gta02_gsm.regulator);
		dev_err(&pdev->dev, "Failed to get modem irq: %d\n", ret);
		goto err_regulator_put;
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &gta02_gsm_attr_group);
	if (ret) {
		dev_err(&pdev->dev, "Failed to create sysfs entries: %d\n",
			ret);
		goto err_free_irq;
	}

	device_init_wakeup(&pdev->dev, 1);

	/* note that download initially disabled, and enforce that */
	gta02_gsm.gpio_ndl_gsm = 1;
	gpio_request(GTA02_GPIO_nDL_GSM, "GSM nDL");
	gpio_direction_output(GTA02_GPIO_nDL_GSM, 1);
	gpio_request(GTA02_GPIO_MODEM_ON, "GSM modem enable");
	gpio_direction_output(GTA02_GPIO_MODEM_ON, 0);

	/* GSM is to be initially off (at boot, or if this module inserted) */
	gsm_on_off(&pdev->dev, 0);

	return 0;
err_free_irq:
	free_irq(GTA02_IRQ_MODEM, NULL);
err_regulator_put:
	regulator_put(gta02_gsm.regulator);

	return ret;
}

static int __devexit gta02_gsm_remove(struct platform_device *pdev)
{
	gsm_on_off(&pdev->dev, 0);

	sysfs_remove_group(&pdev->dev.kobj, &gta02_gsm_attr_group);
	free_irq(GTA02_IRQ_MODEM, NULL);
	regulator_put(gta02_gsm.regulator);

	return 0;
}

static struct platform_driver gta02_gsm_driver = {
	.probe		= gta02_gsm_probe,
	.remove		= __devexit_p(gta02_gsm_remove),
	.driver		= {
		.name	= "gta02-pm-gsm",
		.pm	= GTA02_GSM_PM_OPS,
	},
};

static int __init gta02_gsm_init(void)
{
	return platform_driver_register(&gta02_gsm_driver);
}
module_init(gta02_gsm_init);

static void __exit gta02_gsm_exit(void)
{
	platform_driver_unregister(&gta02_gsm_driver);
}
module_exit(gta02_gsm_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Harald Welte <laforge@openmoko.org>");
MODULE_DESCRIPTION("Openmoko Freerunner GSM Power Management");
