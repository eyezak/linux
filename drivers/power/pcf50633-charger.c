/* NXP PCF50633 Main Battery Charger Driver
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/interrupt.h>

#include <linux/mfd/pcf50633/core.h>
#include <linux/mfd/pcf50633/mbc.h>
#include <linux/mfd/pcf50633/adc.h>
#include <linux/mfd/pcf50633/gpio.h>


#define REG_SELECTED_MSG(reg, match) \
	((reg == match) ? "[%s] " : "%s ")
#define REG_BIT_MSG(reg, match) \
	((reg & match) ? "%s " : "")
#define REG_BITS_MSG(reg, mask, match) \
	(((reg & mask) == match) ? "%s " : "")

static inline struct pcf50633_mbc * __to_mbc(struct device * dev)
{
	return dev_get_drvdata(dev);
}

static inline struct pcf50633 * __to_pcf(struct pcf50633_mbc * mbc)
{
	return dev_to_pcf50633(mbc->dev->parent);
}

int pcf50633_mbc_usb_curlim_set(struct pcf50633_mbc *mbc, int ma)
{
	struct pcf50633 *pcf = __to_pcf(mbc);
	int ret = 0;
	u8 bits;
	int charging_start = 1;
	u8 mbcs2, chgmod;
	unsigned int mbcc5;

	if (ma > pcf->pdata->charger_reference_current_ma)
		ma = pcf->pdata->charger_reference_current_ma;
	else if (ma < 0)
		ma = 0;

	if (ma >= 1000) {
		bits = PCF50633_MBCC7_USB_1000mA;
//		ma = 1000;
	} else if (ma >= 500) {
		bits = PCF50633_MBCC7_USB_500mA;
//		ma = 500;
	} else if (ma >= 100) {
		bits = PCF50633_MBCC7_USB_100mA;
//		ma = 100;
	} else {
		bits = PCF50633_MBCC7_USB_SUSPEND;
		charging_start = 0;
//		ma = 0;
	}

	ret = pcf50633_reg_set_bit_mask(pcf, PCF50633_REG_MBCC7,
					PCF50633_MBCC7_USB_MASK, bits);
	if (ret)
		dev_err(mbc->dev, "error setting usb curlim to %d mA\n", ma);
	else
		dev_info(mbc->dev, "usb curlim to %d mA\n", ma);

	/*
	 * We limit the charging current to be the USB current limit.
	 * The reason is that on pcf50633, when it enters PMU Standby mode,
	 * which it does when the device goes "off", the USB current limit
	 * reverts to the variant default.  In at least one common case, that
	 * default is 500mA.  By setting the charging current to be the same
	 * as the USB limit we set here before PMU standby, we enforce it only
	 * using the correct amount of current even when the USB current limit
	 * gets reset to the wrong thing
	 */

	if (pcf->pdata->charger_reference_current_ma) {
		mbcc5 = (ma * 255) / pcf->pdata->charger_reference_current_ma;
		if (mbcc5 > 255)
			mbcc5 = 255;
		pcf50633_reg_write(pcf, PCF50633_REG_MBCC5, mbcc5);
	}

	mbcs2 = pcf50633_reg_read(pcf, PCF50633_REG_MBCS2);
	chgmod = (mbcs2 & PCF50633_MBCS2_MBC_MASK);

	/* If chgmod == BATFULL, setting chgena has no effect.
	 * Datasheet says we need to set resume instead but when autoresume is
	 * used resume doesn't work. Clear and set chgena instead.
	 */
	if (chgmod != PCF50633_MBCS2_MBC_BAT_FULL)
		pcf50633_reg_set_bit_mask(pcf, PCF50633_REG_MBCC1,
				PCF50633_MBCC1_CHGENA, PCF50633_MBCC1_CHGENA);
	else {
		pcf50633_reg_clear_bits(pcf, PCF50633_REG_MBCC1,
				PCF50633_MBCC1_CHGENA);
		pcf50633_reg_set_bit_mask(pcf, PCF50633_REG_MBCC1,
				PCF50633_MBCC1_CHGENA, PCF50633_MBCC1_CHGENA);
	}

	power_supply_changed(&mbc->usb);

	return ret;
}
EXPORT_SYMBOL_GPL(pcf50633_mbc_usb_curlim_set);

int pcf50633_mbc_get_status(struct pcf50633_mbc *mbc)
{
	int status = 0;
	u8 chgmod;

	chgmod = pcf50633_reg_read(__to_pcf(mbc), PCF50633_REG_MBCS2)
		& PCF50633_MBCS2_MBC_MASK;

	if (mbc->usb_online)
		status |= PCF50633_MBC_USB_ONLINE;
	if (chgmod == PCF50633_MBCS2_MBC_USB_PRE ||
	    chgmod == PCF50633_MBCS2_MBC_USB_PRE_WAIT ||
	    chgmod == PCF50633_MBCS2_MBC_USB_FAST ||
	    chgmod == PCF50633_MBCS2_MBC_USB_FAST_WAIT)
		status |= PCF50633_MBC_USB_ACTIVE;
	if (mbc->adapter_online)
		status |= PCF50633_MBC_ADAPTER_ONLINE;
	if (chgmod == PCF50633_MBCS2_MBC_ADP_PRE ||
	    chgmod == PCF50633_MBCS2_MBC_ADP_PRE_WAIT ||
	    chgmod == PCF50633_MBCS2_MBC_ADP_FAST ||
	    chgmod == PCF50633_MBCS2_MBC_ADP_FAST_WAIT)
		status |= PCF50633_MBC_ADAPTER_ACTIVE;

	return status;
}
EXPORT_SYMBOL_GPL(pcf50633_mbc_get_status);

int pcf50633_mbc_get_usb_online_status(struct pcf50633_mbc *mbc)
{
	return mbc->usb_online;
}
EXPORT_SYMBOL_GPL(pcf50633_mbc_get_usb_online_status);


static ssize_t _show_chglim(struct device *dev,
                            struct device_attribute *attr,
                            char *buf, unsigned int reg)
{
	struct pcf50633_mbc *mbc = __to_mbc(dev);
	struct pcf50633 *pcf = __to_pcf(mbc);
	u8 mbcreg;
	unsigned int ma;

	mbcreg = pcf50633_reg_read(pcf, reg);
	if (!pcf->pdata->charger_reference_current_ma)
		return -ENODEV;

	ma = (pcf->pdata->charger_reference_current_ma * mbcreg) / 255;

	return sprintf(buf, "%u\n", ma);
}

static ssize_t _set_chglim(struct device *dev,
                           struct device_attribute *attr,
                           const char *buf, size_t count,
                           unsigned int reg)
{
	struct pcf50633_mbc *mbc = __to_mbc(dev);
	struct pcf50633 *pcf = __to_pcf(mbc);
	unsigned long ma;
	unsigned int mbcreg;
	int ret;

	if (!pcf->pdata->charger_reference_current_ma)
		return -ENODEV;

	ret = strict_strtoul(buf, 10, &ma);
	if (ret)
		return -EINVAL;

	mbcreg = (ma * 255) / pcf->pdata->charger_reference_current_ma;
	if (mbcreg > 255)
		mbcreg = 255;
	pcf50633_reg_write(pcf, reg, mbcreg);

	return count;
}

static ssize_t show_status(struct device *dev,
                           struct device_attribute *attr, char *buf)
{
	struct pcf50633_mbc *mbc = __to_mbc(dev);
	struct pcf50633 *pcf = __to_pcf(mbc);
	char * bufhead = buf;
	u16 mbcc;

	mbcc = pcf50633_reg_read(pcf, PCF50633_REG_MBCS1) |
	         (pcf50633_reg_read(pcf, PCF50633_REG_MBCS2) << 8);
	
	buf += sprintf(buf, REG_BIT_MSG(mbcc, 0x01), "usbpres");
	buf += sprintf(buf, REG_BIT_MSG(mbcc, 0x02), "usbok");
	buf += sprintf(buf, REG_BITS_MSG(mbcc, 0x30, 0x0), "tbat_ok");
	buf += sprintf(buf, REG_BITS_MSG(mbcc, 0x30, 0x1), "tbat_above");
	buf += sprintf(buf, REG_BITS_MSG(mbcc, 0x30, 0x2), "tbat_below");
	buf += sprintf(buf, REG_BITS_MSG(mbcc, 0x30, 0x3), "tbat_undef");
	buf += sprintf(buf, REG_BIT_MSG(mbcc, 0x40), "prewdtexp");
	buf += sprintf(buf, REG_BIT_MSG(mbcc, 0x50), "wdtexp");
	
	buf += sprintf(buf, REG_BITS_MSG(mbcc, 0xF00, 0x0), "play");
	buf += sprintf(buf, REG_BITS_MSG(mbcc, 0xF00, 0x1), "prechg");
	buf += sprintf(buf, REG_BITS_MSG(mbcc, 0xF00, 0x2), "prechg_wait");
	buf += sprintf(buf, REG_BITS_MSG(mbcc, 0xF00, 0x3), "fastchg");
	buf += sprintf(buf, REG_BITS_MSG(mbcc, 0xF00, 0x4), "fastchg_wait");
	buf += sprintf(buf, REG_BITS_MSG(mbcc, 0xF00, 0x5), "suspend");
	buf += sprintf(buf, REG_BITS_MSG(mbcc, 0xF00, 0xA), "bat_full");
	buf += sprintf(buf, REG_BITS_MSG(mbcc, 0xF00, 0xB), "halt");

	buf += sprintf(buf, REG_BITS_MSG(mbcc, 0x3000, 0x0), "no_charger");
	buf += sprintf(buf, REG_BITS_MSG(mbcc, 0x3000, 0x1), "usb_charger");
	buf += sprintf(buf, REG_BIT_MSG(mbcc, 0x4000), "chg_res_auto");

	*(buf-1) = '\n';
	return (buf - bufhead);
}
static DEVICE_ATTR(status, S_IRUGO , show_status, NULL);

static ssize_t show_chgmode(struct device *dev,
                           struct device_attribute *attr, char *buf)
{
	struct pcf50633_mbc *mbc = __to_mbc(dev);
	char * bufhead = buf;
	u8 mbcreg;

	mbcreg = pcf50633_reg_read(__to_pcf(mbc), PCF50633_REG_MBCC7);	
	mbcreg = (mbcreg & PCF50633_MBCC7_USB_MASK);
	
	buf += sprintf(buf, REG_SELECTED_MSG(mbcreg, 0x00), "100mA");
	buf += sprintf(buf, REG_SELECTED_MSG(mbcreg, 0x01), "500mA");
	buf += sprintf(buf, REG_SELECTED_MSG(mbcreg, 0x02), "1000mA");
	buf += sprintf(buf, REG_SELECTED_MSG(mbcreg, 0x03), "off");

	*(buf-1) = '\n';
	return (buf - bufhead);
}

static ssize_t set_chgmode(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct pcf50633_mbc *mbc = dev_get_drvdata(dev);
	unsigned long ma;
	
	if (strict_strtoul(buf, 10, &ma)) {
		if (strcmp(buf, "off") == 0 || strcmp(buf, "off\n") == 0)
			ma = 0;
		else
			return -EINVAL;
	}
	if (ma!=1000 && ma != 500 && ma != 100 && ma != 0)
		return -EINVAL;

	dev_info(dev, "set_chgmode: %lu\n", ma);

	pcf50633_mbc_usb_curlim_set(mbc, ma);

	return count;
}

static DEVICE_ATTR(charge_mode, S_IRUGO | S_IWUSR, show_chgmode, set_chgmode);

/* Fast-Charge Current Limit */
static ssize_t show_fastchglim(struct device *dev,
                            struct device_attribute *attr,
                            char *buf)
{
	return _show_chglim(dev, attr, buf, PCF50633_REG_MBCC5);
}
static ssize_t set_fastchglim(struct device *dev,
                           struct device_attribute *attr,
                           const char *buf, size_t count)
{
	return _set_chglim(dev, attr, buf, count, PCF50633_REG_MBCC5);
}
static DEVICE_ATTR(fastchg_curlim, S_IRUGO | S_IWUSR, show_fastchglim, set_fastchglim);

/* Pre-Charge Current Limit */
static ssize_t show_prechglim(struct device *dev,
                            struct device_attribute *attr,
                            char *buf)
{
	return _show_chglim(dev, attr, buf, PCF50633_REG_MBCC3);
}
static ssize_t set_prechglim(struct device *dev,
                           struct device_attribute *attr,
                           const char *buf, size_t count)
{
	return _set_chglim(dev, attr, buf, count, PCF50633_REG_MBCC3);
}
static DEVICE_ATTR(prechg_curlim, S_IRUGO | S_IWUSR, show_prechglim, set_prechglim);

static struct attribute *pcf50633_mbc_sysfs_entries[] = {
	&dev_attr_status.attr,
	&dev_attr_charge_mode.attr,
	&dev_attr_fastchg_curlim.attr,
	&dev_attr_prechg_curlim.attr,
	NULL,
};

static struct attribute_group mbc_attr_group = {
	.name	= NULL,			/* put in device directory */
	.attrs	= pcf50633_mbc_sysfs_entries,
};

static irqreturn_t pcf50633_mbc_irq_handler(int irq, void *data)
{
	struct pcf50633_mbc *mbc = data;

	/* USB */
	if (irq == mbc->irqs[PCF50633_IRQ_USBINS]) {
		mbc->usb_online = 1;
	} else if (irq == mbc->irqs[PCF50633_IRQ_USBREM]) {
		mbc->usb_online = 0;
		pcf50633_mbc_usb_curlim_set(mbc, 0);
	}

	/* Adapter */
	if (irq == mbc->irqs[PCF50633_IRQ_ADPINS])
		mbc->adapter_online = 1;
	else if (irq == mbc->irqs[PCF50633_IRQ_ADPREM])
		mbc->adapter_online = 0;

	power_supply_changed(&mbc->ac);
	power_supply_changed(&mbc->usb);
	power_supply_changed(&mbc->adapter);

	if (irq == mbc->irqs[PCF50633_IRQ_USBINS]) {
		schedule_delayed_work(&mbc->work,
		                      __to_pcf(mbc)->pdata->charger_timeout);
	} else if  (irq == mbc->irqs[PCF50633_IRQ_USBREM]) {
		cancel_delayed_work_sync(&mbc->work);
		mbc->usb_curlim = 0;
	}
	
	return IRQ_HANDLED;
}

static int adapter_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	struct pcf50633_mbc *mbc = container_of(psy,
				struct pcf50633_mbc, adapter);
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval =  mbc->adapter_online;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int usb_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	struct pcf50633_mbc *mbc = container_of(psy, struct pcf50633_mbc, usb);
	int ret = 0;
	u8 usblim = pcf50633_reg_read(__to_pcf(mbc), PCF50633_REG_MBCC7) &
						PCF50633_MBCC7_USB_MASK;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = mbc->usb_online &&
				(usblim <= PCF50633_MBCC7_USB_500mA);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int ac_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	struct pcf50633_mbc *mbc = container_of(psy, struct pcf50633_mbc, ac);
	int ret = 0;
	u8 usblim = pcf50633_reg_read(__to_pcf(mbc), PCF50633_REG_MBCC7) &
						PCF50633_MBCC7_USB_MASK;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = mbc->usb_online &&
				(usblim == PCF50633_MBCC7_USB_1000mA);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static void pcf50633_mbc_curlim_from_adc(struct pcf50633 *pcf, 
                                         void *data, int res)
{
	struct pcf50633_mbc * mbc = (struct pcf50633_mbc *) mbc;
	int  ma;
	
	dev_info(mbc->dev, "curlim: adc returned %d\n", res);

	/* Interpret charger type */
	if (res < ((ADC_NOM_CHG_DETECT_USB + ADC_NOM_CHG_DETECT_1A) / 2)) {
		/*
		 * Sanity - stop GPO driving out now that we have a 1A charger
		 * GPO controls USB Host power generation on GTA02
		 */
		/* TODO: FIX GPIO
		pcf50633_gpio_set(pcf, PCF50633_GPO, 0); */
		
		ma = pcf->pdata->charger_reference_current_ma;
	} else
		/* If the PCF50633 ADC is disabled we fallback to a
		 * 100mA limit for safety. */
		ma = 100;

	pcf50633_mbc_usb_curlim_set(mbc, ma);
}

static void pcf50633_mbc_curlim_worker(struct work_struct *work)
{
	struct delayed_work * dwork;
	struct pcf50633_mbc * mbc;
	struct pcf50633 * pcf;
	
	dwork = to_delayed_work(work);
	mbc = container_of(dwork, struct pcf50633_mbc, work);
	pcf = __to_pcf(mbc);

	if (mbc->usb_curlim) {
		pcf50633_mbc_usb_curlim_set(mbc, mbc->usb_curlim);
		return;
	}

	if (pcf->adc) {
		pcf50633_adc_async_read(pcf,
					PCF50633_ADCC1_MUX_ADCIN1,
					PCF50633_ADCC1_AVERAGE_16,
					pcf50633_mbc_curlim_from_adc,
					mbc);
	} else {
		/* If the PCF50633 ADC is disabled we fallback to a
		 * 100mA limit for safety. */
		pcf50633_mbc_usb_curlim_set(mbc, 100);
	}
}

static void pcf50633_mbc_vbus_draw(struct pcf50633_mbc *mbc, int ma)
{
	struct pcf50633 *pcf  = __to_pcf(mbc);

	dev_info(mbc->dev, "vbus draw %d mA\n", ma);
	mbc->usb_curlim = ma;
	schedule_delayed_work(&mbc->work, pcf->pdata->charger_timeout);
	return;
}

static enum power_supply_property power_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static const char *pcf50633_mbc_irq_names[PCF50633_MBC_NUM_IRQS] = {
       "ADPINS",
       "ADPREM",
       "USBINS",
       "USBREM",
       "BATFULL",
       "CHGHALT",
       "THLIMON",
       "THLIMOFF",
       "USBLIMON",
       "USBLIMOFF",
       "LOWSYS",
       "LOWBAT",
};

static int __devinit pcf50633_mbc_probe(struct platform_device *pdev)
{
	struct pcf50633_mbc *mbc;
	struct pcf50633 *pcf;
	int ret;
	int i;
	u8 mbcs1;

	mbc = kzalloc(sizeof(*mbc), GFP_KERNEL);
	if (!mbc)
		return -ENOMEM;

	platform_set_drvdata(pdev, mbc);
	mbc->dev = &pdev->dev;
	mbc->get_status = pcf50633_mbc_get_status;
	mbc->usb_curlim_set = pcf50633_mbc_usb_curlim_set;
	mbc->vbus_draw = pcf50633_mbc_vbus_draw;
	INIT_DELAYED_WORK(&mbc->work, pcf50633_mbc_curlim_worker);

	pcf = __to_pcf(mbc);
	pcf->mbc = mbc;
	/* Create power supplies */
	mbc->adapter.name		= "adapter";
	mbc->adapter.type		= POWER_SUPPLY_TYPE_MAINS;
	mbc->adapter.properties		= power_props;
	mbc->adapter.num_properties	= ARRAY_SIZE(power_props);
	mbc->adapter.get_property	= &adapter_get_property;
	mbc->adapter.supplied_to	= pcf->pdata->batteries;
	mbc->adapter.num_supplicants	= pcf->pdata->num_batteries;

	mbc->usb.name			= "usb";
	mbc->usb.type			= POWER_SUPPLY_TYPE_USB;
	mbc->usb.properties		= power_props;
	mbc->usb.num_properties		= ARRAY_SIZE(power_props);
	mbc->usb.get_property		= usb_get_property;
	mbc->usb.supplied_to		= pcf->pdata->batteries;
	mbc->usb.num_supplicants	= pcf->pdata->num_batteries;

	mbc->ac.name			= "ac";
	mbc->ac.type			= POWER_SUPPLY_TYPE_MAINS;
	mbc->ac.properties		= power_props;
	mbc->ac.num_properties		= ARRAY_SIZE(power_props);
	mbc->ac.get_property		= ac_get_property;
	mbc->ac.supplied_to		= pcf->pdata->batteries;
	mbc->ac.num_supplicants		= pcf->pdata->num_batteries;

	ret = power_supply_register(&pdev->dev, &mbc->adapter);
	if (ret) {
		dev_err(&pdev->dev, "failed to register adapter\n");
		goto err_ps_adapter;
		return ret;
	}

	ret = power_supply_register(&pdev->dev, &mbc->usb);
	if (ret) {
		dev_err(&pdev->dev, "failed to register usb\n");
		goto err_ps_usb;
		return ret;
	}

	ret = power_supply_register(&pdev->dev, &mbc->ac);
	if (ret) {
		dev_err(&pdev->dev, "failed to register ac\n");
		goto err_ps_ac;
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &mbc_attr_group);
	if (ret)
		dev_err(&pdev->dev, "failed to create sysfs entries\n");

	mbcs1 = pcf50633_reg_read(pcf, PCF50633_REG_MBCS1);
	dev_info(&pdev->dev, "MBCS1: %d %d %d%d %d %d %d %d\n",
	         (mbcs1 & 0x0F) >> 7,
	         (mbcs1 & 0x0D) >> 6,
	         (mbcs1 & 0x0C) >> 5,
	         (mbcs1 & 0x0A) >> 4,
	         (mbcs1 & 0x08) >> 3,
	         (mbcs1 & 0x04) >> 2,
	         (mbcs1 & 0x02) >> 1,
	          mbcs1 & 0x01);
	if (mbcs1 & PCF50633_MBCS1_USBPRES) {
		/* simulate vbus_draw */
		mbc->usb_curlim = 500;
		
		pcf50633_mbc_irq_handler(mbc->irqs[PCF50633_MBC_IRQ_USBINS], mbc);
	}
	if (mbcs1 & PCF50633_MBCS1_ADAPTPRES)
		pcf50633_mbc_irq_handler(mbc->irqs[PCF50633_MBC_IRQ_ADPINS], mbc);

	/* Set up IRQ handlers */
	for (i = 0; i < PCF50633_MBC_NUM_IRQS; ++i) {
		ret = request_threaded_irq(mbc->irqs[i], NULL,
						pcf50633_mbc_irq_handler, 0,
						pcf50633_mbc_irq_names[i], mbc);
		if (ret) {
			dev_err(&pdev->dev, "Failed to request %s irq: %d\n",
									pcf50633_mbc_irq_names[i], ret);
			goto err_free_irq;
		}
	}

	return 0;

err_free_irq:
	for (--i; i >= 0; --i)
			free_irq(mbc->irqs[i], mbc);
err_ps_ac:
	power_supply_unregister(&mbc->usb);
err_ps_usb:
	power_supply_unregister(&mbc->adapter);
err_ps_adapter:
	kfree(mbc);
	
	return ret;
}

static int __devexit pcf50633_mbc_remove(struct platform_device *pdev)
{
	struct pcf50633_mbc *mbc = platform_get_drvdata(pdev);
	int i;

	cancel_delayed_work_sync(&mbc->work);
	// del_timer(&mbc->work.timer);

	/* Remove IRQ handlers */
	for (i = PCF50633_MBC_NUM_IRQS - 1; i >= 0; --i)
		free_irq(mbc->irqs[i], mbc);

	sysfs_remove_group(&pdev->dev.kobj, &mbc_attr_group);
	power_supply_unregister(&mbc->usb);
	power_supply_unregister(&mbc->adapter);
	power_supply_unregister(&mbc->ac);

	kfree(mbc);

	return 0;
}

static struct platform_driver pcf50633_mbc_driver = {
	.driver = {
		.name = "pcf50633-mbc",
	},
	.probe = pcf50633_mbc_probe,
	.remove = __devexit_p(pcf50633_mbc_remove),
};

module_platform_driver(pcf50633_mbc_driver);

MODULE_AUTHOR("Balaji Rao <balajirrao@openmoko.org>");
MODULE_DESCRIPTION("PCF50633 mbc driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pcf50633-mbc");
