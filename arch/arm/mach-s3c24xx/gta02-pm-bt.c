/*
 * Bluetooth PM code for the Openmoko Freerunner GSM Phone
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/rfkill.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/err.h>
#include <linux/regulator/consumer.h>

#include <asm/mach-types.h>
#include <mach/gpio-fns.h>
#include <mach/gta02.h>
#include <mach/hardware.h>

#define DRVMSG "Openmoko Freerunner Bluetooth Power Management"

struct gta02_pm_bt_data {
	struct regulator *regulator;
#ifdef CONFIG_RFKILL
	struct rfkill *rfkill;
#endif /* CONFIG_RFKILL */
	int pre_resume_state;
};

static ssize_t bt_read(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	int ret = 0;
	if (!strcmp(attr->attr.name, "power_on")) {
		if (gpio_get_value(GTA02_GPIO_BT_EN))
			ret = 1;
	} else if (!strcmp(attr->attr.name, "reset")) {
		if (gpio_get_value(GTA02_GPIO_BT_EN) == 0)
			ret = 1;
	}

	if (!ret)
		return strlcpy(buf, "0\n", 3);
	else
		return strlcpy(buf, "1\n", 3);
}

static void __gta02_pm_bt_toggle_radio(struct device *dev, unsigned int on)
{
	struct gta02_pm_bt_data *bt_data = dev_get_drvdata(dev);

	dev_info(dev, "__gta02_pm_bt_toggle_radio %d\n", on);

	bt_data = dev_get_drvdata(dev);

	gpio_set_value(GTA02_GPIO_BT_EN, !on);

	if (on) {
		if (!regulator_is_enabled(bt_data->regulator))
			regulator_enable(bt_data->regulator);
	} else {
		if (regulator_is_enabled(bt_data->regulator))
			regulator_disable(bt_data->regulator);
	}

	gpio_set_value(GTA02_GPIO_BT_EN, on);
}


#ifdef CONFIG_RFKILL
static int bt_rfkill_set_block(void *data, bool blocked)
{
	struct device *dev = data;

	__gta02_pm_bt_toggle_radio(dev, !blocked);

	return 0;
}

static const struct rfkill_ops gta02_bt_rfkill_ops = {
	.set_block = bt_rfkill_set_block,
};
#endif /* CONFIG_RFKILL */


static ssize_t bt_write(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	int ret;
	unsigned long on;

	ret = strict_strtoul(buf, 10, &on);
	if (ret)
		return ret;

	if (!strcmp(attr->attr.name, "power_on")) {
#ifdef CONFIG_RFKILL
		struct gta02_pm_bt_data *bt_data;
		
		bt_data = dev_get_drvdata(dev);
		rfkill_set_sw_state(bt_data->rfkill, on ? 1 : 0);
#endif

		__gta02_pm_bt_toggle_radio(dev, on);
	} else if (!strcmp(attr->attr.name, "reset")) {
		/* reset is low-active, so we need to invert */
		gpio_set_value(GTA02_GPIO_BT_EN, on ? 0 : 1);
	}

	return count;
}

static DEVICE_ATTR(power_on, 0644, bt_read, bt_write);
static DEVICE_ATTR(reset, 0644, bt_read, bt_write);

#ifdef CONFIG_PM
static int gta02_bt_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct gta02_pm_bt_data *bt_data = dev_get_drvdata(&pdev->dev);

	dev_dbg(&pdev->dev, DRVMSG ": suspending\n");

	bt_data->pre_resume_state = gpio_get_value(GTA02_GPIO_BT_EN);
	__gta02_pm_bt_toggle_radio(&pdev->dev, 0);

	return 0;
}

static int gta02_bt_resume(struct platform_device *pdev)
{
	struct gta02_pm_bt_data *bt_data = dev_get_drvdata(&pdev->dev);
	dev_dbg(&pdev->dev, DRVMSG ": resuming\n");

	__gta02_pm_bt_toggle_radio(&pdev->dev, bt_data->pre_resume_state);
	return 0;
}
#else
#define gta02_bt_suspend	NULL
#define gta02_bt_resume		NULL
#endif

static struct attribute *gta02_bt_sysfs_entries[] = {
	&dev_attr_power_on.attr,
	&dev_attr_reset.attr,
	NULL
};

static struct attribute_group gta02_bt_attr_group = {
	.name	= NULL,
	.attrs	= gta02_bt_sysfs_entries,
};

static int __devinit gta02_bt_probe(struct platform_device *pdev)
{
#ifdef CONFIG_RFKILL
	struct rfkill *rfkill;
#endif /* CONFIG_RFKILL */
	int ret;
	struct regulator *regulator;
	struct gta02_pm_bt_data *bt_data;

	dev_info(&pdev->dev, "starting\n");

	bt_data = kzalloc(sizeof(*bt_data), GFP_KERNEL);
	dev_set_drvdata(&pdev->dev, bt_data);

	regulator = regulator_get(&pdev->dev, "BT_3V2");
	if (IS_ERR(regulator)) {
		dev_err(&pdev->dev, "Failed to get regulator BT_3V2 (%ld)\n",
		                   PTR_ERR(regulator));
		ret = -ENODEV;
		goto end;
	}

	bt_data->regulator = regulator;

	/* this tests the true physical state of the regulator... */
	if (regulator_is_enabled(regulator)) {
		/*
		 * but these only operate on the logical state of the
		 * regulator... so we need to logicaly "adopt" it on
		 * to turn it off
		 */
		regulator_enable(regulator);
		regulator_disable(regulator);
	}

	/* we pull reset to low to make sure that the chip doesn't
	 * drain power through the reset line */
	gpio_request(GTA02_GPIO_BT_EN, "Bluetooth enable");
	gpio_direction_output(GTA02_GPIO_BT_EN, 0);

#ifdef CONFIG_RFKILL
	rfkill = rfkill_alloc(pdev->name, &pdev->dev, RFKILL_TYPE_BLUETOOTH,
				&gta02_bt_rfkill_ops, &pdev->dev);

	if (!rfkill) {
		dev_err(&pdev->dev, "Failed to allocate rfkill\n");	
		ret = -ENOMEM;
		goto end_regltr;
	}

	rfkill_init_sw_state(rfkill, 0);

	ret = rfkill_register(rfkill);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register rfkill\n");
		goto end_rfkill;
	}

	bt_data->rfkill = rfkill;
#endif /* CONFIG_RFKILL */

	return sysfs_create_group(&pdev->dev.kobj, &gta02_bt_attr_group);

#ifdef CONFIG_RFKILL
end_rfkill:
	rfkill_destroy(rfkill);
#endif
end_regltr:
	regulator_put(bt_data->regulator);
end:
	kfree(bt_data);
	return ret;
}

static int __devexit gta02_bt_remove(struct platform_device *pdev)
{
	struct gta02_pm_bt_data *bt_data = dev_get_drvdata(&pdev->dev);
	struct regulator *regulator;

	sysfs_remove_group(&pdev->dev.kobj, &gta02_bt_attr_group);

#ifdef CONFIG_RFKILL
	if (bt_data->rfkill)
		rfkill_destroy(bt_data->rfkill);
#endif /* CONFIG_RFKILL */

	if (!bt_data || !bt_data->regulator)
		return 0;

	regulator = bt_data->regulator;

	/* Make sure regulator is disabled before calling regulator_put */
	if (regulator_is_enabled(regulator))
		regulator_disable(regulator);

	regulator_put(regulator);

	kfree(bt_data);

	return 0;
}

static struct platform_driver gta02_bt_driver = {
	.probe		= gta02_bt_probe,
	.remove		= __devexit_p(gta02_bt_remove),
	.suspend	= gta02_bt_suspend,
	.resume		= gta02_bt_resume,
	.driver		= {
		.name		= "gta02-pm-bt",
	},
};

static int __init gta02_bt_init(void)
{
	return platform_driver_register(&gta02_bt_driver);
}
module_init(gta02_bt_init);

static void __exit gta02_bt_exit(void)
{
	platform_driver_unregister(&gta02_bt_driver);
}
module_exit(gta02_bt_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Harald Welte <laforge@openmoko.org>");
MODULE_DESCRIPTION(DRVMSG);
