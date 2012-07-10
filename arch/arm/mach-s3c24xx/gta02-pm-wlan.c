/*
 * GTA02 WLAN power management
 *
 * (C) 2008, 2009 by Openmoko Inc.
 * Author: Andy Green <andy@openmoko.com>
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
#include <linux/mutex.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>

#include <mach/gta02.h>
#include <mach/regs-gpio.h>
#include <mach/regs-gpioj.h>
#include <plat/gpio-fns.h>

#include <linux/delay.h>
#include <linux/rfkill.h>


/* ----- Module hardware reset ("power") ----------------------------------- */


void gta02_wlan_reset(int assert_reset)
{
	if (assert_reset) {
		s3c2410_gpio_setpin(GTA02_GPIO_nWLAN_RESET, 0);
		msleep(200); /* probably excessive but we don't have specs */
	} else {
		s3c2410_gpio_setpin(GTA02_GPIO_nWLAN_RESET, 1);
	}
}

#ifdef CONFIG_PM
static int gta02_wlan_suspend(struct platform_device *pdev, pm_message_t state)
{
	dev_dbg(&pdev->dev, "suspending\n");

	return 0;
}

static int gta02_wlan_resume(struct platform_device *pdev)
{
	dev_dbg(&pdev->dev, "resuming\n");

	return 0;
}
#else
#define gta02_wlan_suspend	NULL
#define gta02_wlan_resume		NULL
#endif


/* ----- rfkill ------------------------------------------------------------ */
#ifdef CONFIG_RFKILL
/*
 * S3C MCI handles suspend/resume through device removal/insertion. In order to
 * preserve rfkill state, as required in clause 7 of section 3.1 in rfkill.txt,
 * we therefore need to maintain rfkill state outside the driver.
 *
 * This platform driver is as good a place as any other.
 */

static int (*gta02_wlan_rfkill_cb)(void *user, int on);
static void *gta02_wlan_rfkill_user;
static DEFINE_MUTEX(gta02_wlan_rfkill_lock);
static int gta02_wlan_rfkill_on;


/*
 * gta02_wlan_query_rfkill_lock is used to obtain the rfkill state before the
 * driver is ready to process rfkill callbacks. To prevent the state from
 * changing until the driver has completed its initialization, we grab and hold
 * the rfkill lock.
 *
 * A call to gta02_wlan_query_rfkill_lock must be followed by either
 * - a call to gta02_wlan_set_rfkill_cb, to complete the setup, or
 * - a call to gta02_wlan_query_rfkill_unlock to abort the setup process.
 */

int gta02_wlan_query_rfkill_lock(void)
{
	mutex_lock(&gta02_wlan_rfkill_lock);
	return gta02_wlan_rfkill_on;
}
EXPORT_SYMBOL_GPL(gta02_wlan_query_rfkill_lock);

void gta02_wlan_query_rfkill_unlock(void)
{
	mutex_unlock(&gta02_wlan_rfkill_lock);
}
EXPORT_SYMBOL_GPL(gta02_wlan_query_rfkill_unlock);


void gta02_wlan_set_rfkill_cb(int (*cb)(void *user, int on), void *user)
{
	BUG_ON(!mutex_is_locked(&gta02_wlan_rfkill_lock));
	BUG_ON(gta02_wlan_rfkill_cb);
	gta02_wlan_rfkill_cb = cb;
	gta02_wlan_rfkill_user = user;
	mutex_unlock(&gta02_wlan_rfkill_lock);
}
EXPORT_SYMBOL_GPL(gta02_wlan_set_rfkill_cb);

void gta02_wlan_clear_rfkill_cb(void)
{
	mutex_lock(&gta02_wlan_rfkill_lock);
	BUG_ON(!gta02_wlan_rfkill_cb);
	gta02_wlan_rfkill_cb = NULL;
	mutex_unlock(&gta02_wlan_rfkill_lock);
}
EXPORT_SYMBOL_GPL(gta02_wlan_clear_rfkill_cb);

static int gta02_wlan_toggle_radio(void *data, enum rfkill_state state)
{
	struct device *dev = data;
	int on = state == RFKILL_STATE_UNBLOCKED;
	int res = 0;

	dev_dbg(dev, "gta02_wlan_toggle_radio: state %d (%p)\n",
	    state, gta02_wlan_rfkill_cb);
	mutex_lock(&gta02_wlan_rfkill_lock);
	if (gta02_wlan_rfkill_cb)
		res = gta02_wlan_rfkill_cb(gta02_wlan_rfkill_user, on);
	if (!res)
		gta02_wlan_rfkill_on = on;
	mutex_unlock(&gta02_wlan_rfkill_lock);
	return res;
}
#endif /* CONFIG RFKILL */

/* SYSFS */
static unsigned int keepact = 0, shutdown = 0;
#define output_status(buf, var) strlcpy(buf, var ? "1\n" : "0\n", 3)

static ssize_t sysfs_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	if (strcmp(attr->attr.name, "shutdown") == 0)
		return output_status(buf, shutdown);
	else if (strcmp(attr->attr.name, "keepact") == 0)
		return output_status(buf, keepact);
	else
		return 0;
}

static ssize_t sysfs_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long on;
	

	ret = strict_strtoul(buf, 10, &on);
	if (ret)
		return ret;

	if (strcmp(attr->attr.name, "shutdown") == 0) {
		shutdown = on;
		
		s3c2410_gpio_setpin(GTA02_GPIO_SHUTDOWN, on ? 1 : 0);
	} else if (strcmp(attr->attr.name, "keepact") == 0) {
		keepact = on;
		
		gpio_set_value(GTA02_GPIO_KEEPACT, on ? 1 : 0);
	}

	return count;
}

static DEVICE_ATTR(shutdown, 0644, sysfs_read, sysfs_write);
static DEVICE_ATTR(keepact, 0644, sysfs_read, sysfs_write);

static struct attribute *gta02_wlan_sysfs_entries[] = {
	&dev_attr_shutdown.attr,
	&dev_attr_keepact.attr,
	NULL
};

static struct attribute_group gta02_wlan_attr_group = {
	.name	= NULL,
	.attrs	= gta02_wlan_sysfs_entries,
};

/* ----- Initialization/removal -------------------------------------------- */

static int __devinit gta02_wlan_probe(struct platform_device *pdev)
{
	/* default-on for now */
	const int default_state = 0;
#ifdef CONFIG_RFKILL
	struct rfkill *rfkill;
	int error;
#endif

	if (!machine_is_neo1973_gta02())
		return -EINVAL;

	dev_info(&pdev->dev, "starting\n");

	WARN_ON(gpio_request(GTA02_GPIO_KEEPACT, "gta02-pmu-keepact"));
	gpio_direction_output(GTA02_GPIO_KEEPACT, keepact = 1);

	s3c2410_gpio_cfgpin(GTA02_GPIO_nWLAN_RESET, S3C2410_GPIO_OUTPUT);
	gta02_wlan_reset(1);
	if (default_state)
		gta02_wlan_reset(0);

#ifdef CONFIG_RFKILL
	rfkill = rfkill_allocate(&pdev->dev, RFKILL_TYPE_WLAN);
	rfkill->name = "ar6000";
	rfkill->data = &pdev->dev;
	rfkill->state = default_state ? RFKILL_STATE_ON : RFKILL_STATE_OFF;
	/*
	 * If the WLAN driver somehow managed to get activated before we're
	 * ready, the driver is now in an unknown state, which isn't something
	 * we're prepared to handle. This can't happen, so just fail hard.
	 */
	BUG_ON(gta02_wlan_rfkill_cb);
	gta02_wlan_rfkill_on = default_state;
	rfkill->toggle_radio = gta02_wlan_toggle_radio;

	error = rfkill_register(rfkill);
	if (error) {
		rfkill_free(rfkill);
		return error;
	}

	dev_set_drvdata(&pdev->dev, rfkill);
#endif

	return sysfs_create_group(&pdev->dev.kobj, &gta02_wlan_attr_group);
}

static int __devexit gta02_wlan_remove(struct platform_device *pdev)
{
#ifdef CONFIG_RFKILL
	struct rfkill *rfkill = dev_get_drvdata(&pdev->dev);

	rfkill_unregister(rfkill);
	rfkill_free(rfkill);
#endif
	gpio_free(GTA02_GPIO_KEEPACT);

	sysfs_remove_group(&pdev->dev.kobj, &gta02_wlan_attr_group);

	return 0;
}

static struct platform_driver gta02_wlan_driver = {
	.probe		= gta02_wlan_probe,
	.remove		= __devexit_p(gta02_wlan_remove),
	.suspend	= gta02_wlan_suspend,
	.resume		= gta02_wlan_resume,
	.driver		= {
		.name		= "gta02-pm-wlan",
	},
};

static int __devinit gta02_wlan_init(void)
{
	return platform_driver_register(&gta02_wlan_driver);
}

static void gta02_wlan_exit(void)
{
	platform_driver_unregister(&gta02_wlan_driver);
}

module_init(gta02_wlan_init);
module_exit(gta02_wlan_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andy Green <andy@openmoko.com>");
MODULE_DESCRIPTION("Openmoko GTA02 WLAN power management");
