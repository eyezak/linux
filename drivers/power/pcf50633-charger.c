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

#include <linux/mfd/pcf50633/core.h>
#include <linux/mfd/pcf50633/mbc.h>
#include <linux/mfd/pcf50633/adc.h>
#include <linux/mfd/pcf50633/gpio.h>

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

enum pcf50633_reg_mbcc1 {
	PCF50633_MBCC1_CHGENA			= 0x01,	/* Charger enable */
	PCF50633_MBCC1_AUTOSTOP			= 0x02,
	PCF50633_MBCC1_AUTORES			= 0x04, /* automatic resume */
	PCF50633_MBCC1_RESUME			= 0x08, /* explicit resume cmd */
	PCF50633_MBCC1_RESTART			= 0x10, /* restart charging */
	PCF50633_MBCC1_PREWDTIME_60M	= 0x20,	/* max. precharging time */
	PCF50633_MBCC1_WDTIME_1H		= 0x00,
	PCF50633_MBCC1_WDTIME_2H		= 0x40,
	PCF50633_MBCC1_WDTIME_4H		= 0x80,
	PCF50633_MBCC1_WDTIME_6H		= 0xc0,
};
#define PCF50633_MBCC1_WDTIME_MASK	  0xc0

enum pcf50633_reg_mbcc2 {
	PCF50633_MBCC2_VBATCOND_2V7		= 0x00,
	PCF50633_MBCC2_VBATCOND_2V85	= 0x01,
	PCF50633_MBCC2_VBATCOND_3V0		= 0x02,
	PCF50633_MBCC2_VBATCOND_3V15	= 0x03,
	PCF50633_MBCC2_VMAX_4V			= 0x00,
	PCF50633_MBCC2_VMAX_4V20		= 0x28,
	PCF50633_MBCC2_VRESDEBTIME_64S	= 0x80,	/* debounce time (32/64sec) */
};

enum pcf50633_reg_mbcc7 {
	PCF50633_MBCC7_USB_100mA		= 0x00,
	PCF50633_MBCC7_USB_500mA		= 0x01,
	PCF50633_MBCC7_USB_1000mA		= 0x02,
	PCF50633_MBCC7_USB_SUSPEND		= 0x03,
	PCF50633_MBCC7_BATTEMP_EN		= 0x04,
	PCF50633_MBCC7_BATSYSIMAX_1A6	= 0x00,
	PCF50633_MBCC7_BATSYSIMAX_1A8	= 0x40,
	PCF50633_MBCC7_BATSYSIMAX_2A0	= 0x80,
	PCF50633_MBCC7_BATSYSIMAX_2A2	= 0xc0,
};
#define PCF50633_MBCC7_USB_MASK 0x03

enum pcf50633_reg_mbcc8 {
	PCF50633_MBCC8_USBENASUS		= 0x10,
};

enum pcf50633_reg_mbcs1 {
	PCF50633_MBCS1_USBPRES			= 0x01,
	PCF50633_MBCS1_USBOK			= 0x02,
	PCF50633_MBCS1_ADAPTPRES		= 0x04,
	PCF50633_MBCS1_ADAPTOK			= 0x08,
	PCF50633_MBCS1_TBAT_OK			= 0x00,
	PCF50633_MBCS1_TBAT_ABOVE		= 0x10,
	PCF50633_MBCS1_TBAT_BELOW		= 0x20,
	PCF50633_MBCS1_TBAT_UNDEF		= 0x30,
	PCF50633_MBCS1_PREWDTEXP		= 0x40,
	PCF50633_MBCS1_WDTEXP			= 0x80,
};

enum pcf50633_reg_mbcs2_mbcmod {
	PCF50633_MBCS2_MBC_PLAY			= 0x00,
	PCF50633_MBCS2_MBC_USB_PRE		= 0x01,
	PCF50633_MBCS2_MBC_USB_PRE_WAIT	= 0x02,
	PCF50633_MBCS2_MBC_USB_FAST		= 0x03,
	PCF50633_MBCS2_MBC_USB_FAST_WAIT = 0x04,
	PCF50633_MBCS2_MBC_USB_SUSPEND	= 0x05,
	PCF50633_MBCS2_MBC_ADP_PRE		= 0x06,
	PCF50633_MBCS2_MBC_ADP_PRE_WAIT	= 0x07,
	PCF50633_MBCS2_MBC_ADP_FAST		= 0x08,
	PCF50633_MBCS2_MBC_ADP_FAST_WAIT = 0x09,
	PCF50633_MBCS2_MBC_BAT_FULL		= 0x0a,
	PCF50633_MBCS2_MBC_HALT			= 0x0b,
};
#define PCF50633_MBCS2_MBC_MASK		0x0f
enum pcf50633_reg_mbcs2_chgstat {
	PCF50633_MBCS2_CHGS_NONE		= 0x00,
	PCF50633_MBCS2_CHGS_ADAPTER		= 0x10,
	PCF50633_MBCS2_CHGS_USB			= 0x20,
	PCF50633_MBCS2_CHGS_BOTH		= 0x30,
};
#define PCF50633_MBCS2_RESSTAT_AUTO	0x40

enum pcf50633_reg_mbcs3 {
	PCF50633_MBCS3_USBLIM_PLAY	= 0x01,
	PCF50633_MBCS3_USBLIM_CGH	= 0x02,
	PCF50633_MBCS3_TLIM_PLAY	= 0x04,
	PCF50633_MBCS3_TLIM_CHG		= 0x08,
	PCF50633_MBCS3_ILIM			= 0x10,	/* 1: Ibat > Icutoff */
	PCF50633_MBCS3_VLIM			= 0x20,	/* 1: Vbat == Vmax */
	PCF50633_MBCS3_VBATSTAT		= 0x40,	/* 1: Vbat > Vbatcond */
	PCF50633_MBCS3_VRES			= 0x80, /* 1: Vbat > Vth(RES) */
};

#define PCF50633_MBCC2_VBATCOND_MASK	  0x03
#define PCF50633_MBCC2_VMAX_MASK	  0x3c

enum pcf50633_mbc_irqs {
	PCF50633_MBC_IRQ_ADPINS,
	PCF50633_MBC_IRQ_ADPREM,
	PCF50633_MBC_IRQ_USBINS,
	PCF50633_MBC_IRQ_USBREM,
	PCF50633_MBC_IRQ_BATFULL,
	PCF50633_MBC_IRQ_CHGHALT,
	PCF50633_MBC_IRQ_THLIMON,
	PCF50633_MBC_IRQ_THLIMOFF,
	PCF50633_MBC_IRQ_USBLIMON,
	PCF50633_MBC_IRQ_USBLIMOFF,
	PCF50633_MBC_IRQ_LOWSYS,
	PCF50633_MBC_IRQ_LOWBAT,
	PCF50633_MBC_NUM_IRQS,
};

static int pcf50633_mbc_irq_map[] = {
	[PCF50633_IRQ_ADPINS] = PCF50633_MBC_IRQ_ADPINS,
	[PCF50633_IRQ_ADPREM] = PCF50633_MBC_IRQ_ADPREM,
	[PCF50633_IRQ_USBINS] = PCF50633_MBC_IRQ_USBINS,
	[PCF50633_IRQ_USBREM] = PCF50633_MBC_IRQ_USBREM,
	[PCF50633_IRQ_BATFULL] = PCF50633_MBC_IRQ_BATFULL,
	[PCF50633_IRQ_CHGHALT] = PCF50633_MBC_IRQ_CHGHALT,
	[PCF50633_IRQ_THLIMON] = PCF50633_MBC_IRQ_THLIMON,
	[PCF50633_IRQ_THLIMOFF] = PCF50633_MBC_IRQ_THLIMOFF,
	[PCF50633_IRQ_USBLIMON] = PCF50633_MBC_IRQ_USBLIMON,
	[PCF50633_IRQ_USBLIMOFF] = PCF50633_MBC_IRQ_USBLIMOFF,
	[PCF50633_IRQ_LOWSYS] = PCF50633_MBC_IRQ_LOWSYS,
	[PCF50633_IRQ_LOWBAT] = PCF50633_MBC_IRQ_LOWBAT,
};

static const char *pcf50633_mbc_irq_final_names[PCF50633_MBC_NUM_IRQS] = {
       "pcf50633-mbc:adpins",
       "pcf50633-mbc:adprem",
       "pcf50633-mbc:usbins",
       "pcf50633-mbc:usbrem",
       "pcf50633-mbc:batfull",
       "pcf50633-mbc:chghalt",
       "pcf50633-mbc:thlimon",
       "pcf50633-mbc:thlimoff",
       "pcf50633-mbc:usblimon",
       "pcf50633-mbc:usblimoff",
       "pcf50633-mbc:lowsys",
       "pcf50633-mbc:lowbat",
};

struct pcf50633_mbc {
	struct pcf50633_mbc_ops ops;

	struct device *dev;

	int adapter_connected:1;
	int usb_curlim;
	
	int irqs[PCF50633_MBC_NUM_IRQS];

	struct power_supply usb;
	struct power_supply adapter;

	struct delayed_work work;
};


static inline struct pcf50633_mbc_platform_data * __to_pdata(struct pcf50633_mbc *mbc)
{
	return mbc->dev->platform_data;
}

static inline struct pcf50633_mbc * __to_mbc(struct device * dev)
{
	return dev_get_drvdata(dev);
}

static int pcf50633_mbc_usb_curlim_set(struct pcf50633_mbc *mbc, int ma)
{
	struct pcf50633 *pcf = child_to_pcf50633(mbc);
	int ret = 0;
	u8 bits, mbcs2, chgmod;
	int charging_start = 1;
	unsigned int mbcc5;

	if (ma > 1000)
		ma = 1000;
	else if (ma < 0)
		ma = 0;

	if (ma >= 1000) {
		bits = PCF50633_MBCC7_USB_1000mA;
		ma = 1000;
	} else if (ma >= 500) {
		bits = PCF50633_MBCC7_USB_500mA;
		ma = 500;
	} else if (ma >= 100) {
		bits = PCF50633_MBCC7_USB_100mA;
		ma = 100;
	} else {
		bits = PCF50633_MBCC7_USB_SUSPEND;
		charging_start = 0;
		ma = 0;
	}

	ret = pcf50633_reg_set_bit_mask(pcf, PCF50633_REG_MBCC7,
					PCF50633_MBCC7_USB_MASK, bits);
	if (ret) {
		dev_err(mbc->dev, "error (%d) setting usb curlim to %d mA\n", ret, ma);
		return ret;
	} else
		dev_info(mbc->dev, "usb curlim set to %d mA\n", ma);

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

	mbcc5 = (ma * 255) / 1000;
	if (mbcc5 > 255)
		mbcc5 = 255;
	pcf50633_reg_write(pcf, PCF50633_REG_MBCC5, mbcc5);

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

static int pcf50633_mbc_get_status(struct pcf50633_mbc *mbc)
{
	int status = 0;
	u8 mbcs2, mbcmod, chgstat;

	mbcs2 = pcf50633_reg_read(child_to_pcf50633(mbc), PCF50633_REG_MBCS2);
	mbcmod = mbcs2 & PCF50633_MBCS2_MBC_MASK;
	chgstat = mbcs2 & 0x30;

	if (chgstat & PCF50633_MBCS2_CHGS_USB)
		status |= PCF50633_MBC_USB_ONLINE;
	if (chgstat & PCF50633_MBCS2_CHGS_ADAPTER)
		status |= PCF50633_MBC_ADAPTER_ONLINE;

	if (mbcmod == PCF50633_MBCS2_MBC_USB_PRE ||
	    mbcmod == PCF50633_MBCS2_MBC_USB_PRE_WAIT ||
	    mbcmod == PCF50633_MBCS2_MBC_USB_FAST ||
	    mbcmod == PCF50633_MBCS2_MBC_USB_FAST_WAIT)
		status |= PCF50633_MBC_USB_ACTIVE;
	else if (mbcmod == PCF50633_MBCS2_MBC_ADP_PRE ||
	    mbcmod == PCF50633_MBCS2_MBC_ADP_PRE_WAIT ||
	    mbcmod == PCF50633_MBCS2_MBC_ADP_FAST ||
	    mbcmod == PCF50633_MBCS2_MBC_ADP_FAST_WAIT)
		mbcmod |= PCF50633_MBC_ADAPTER_ACTIVE;

	return status;
}

static ssize_t _show_chglim(struct device *dev,
                            struct device_attribute *attr,
                            char *buf, unsigned int reg)
{
	struct pcf50633_mbc *mbc = __to_mbc(dev);
	struct pcf50633 *pcf = child_to_pcf50633(mbc);
	u8 mbcreg;
	unsigned int ma;

	mbcreg = pcf50633_reg_read(pcf, reg);
	ma = (1000 * mbcreg) / 255;
	
	return sprintf(buf, "%u\n", ma);
}

static ssize_t show_status(struct device *dev,
                           struct device_attribute *attr, char *buf)
{
	struct pcf50633_mbc *mbc = __to_mbc(dev);
	struct pcf50633 *pcf = child_to_pcf50633(mbc);
	char * status;
	u8 mbcs2;

	mbcs2 = pcf50633_reg_read(pcf, PCF50633_REG_MBCS2);
	
	switch (mbcs2 & PCF50633_MBCS2_MBC_MASK) {
	case PCF50633_MBCS2_MBC_PLAY:
		status = "Play Only";
		break;
	case PCF50633_MBCS2_MBC_USB_PRE:
		status = "USB Precharge";
		break;
	case PCF50633_MBCS2_MBC_USB_PRE_WAIT:
		status = "USB Precharge Wait";
		break;
	case PCF50633_MBCS2_MBC_USB_FAST:
		status = "USB Fast Charge";
		break;
	case PCF50633_MBCS2_MBC_USB_FAST_WAIT:
		status = "USB Fast Charge Wait";
		break;
	case PCF50633_MBCS2_MBC_USB_SUSPEND:
		status = "USB Suspended";
		break;
	case PCF50633_MBCS2_MBC_ADP_PRE:
		status = "Adapter Precharge";
		break;
	case PCF50633_MBCS2_MBC_ADP_PRE_WAIT:
		status = "Adapter Precharge Wait";
		break;
	case PCF50633_MBCS2_MBC_ADP_FAST:
		status = "Adapter Fast Charge";
		break;
	case PCF50633_MBCS2_MBC_ADP_FAST_WAIT:
		status = "Adapter Fast Charge Wait";
		break;
	case PCF50633_MBCS2_MBC_BAT_FULL:
		status = "Battery Full";
		break;
	case PCF50633_MBCS2_MBC_HALT:
		status = "Halt";
		break;
	default:
		return -EINVAL;
	}
	
	return snprintf(buf, PAGE_SIZE, "%s\n", status);
}
static DEVICE_ATTR(status, S_IRUGO , show_status, NULL);

static ssize_t show_usb_current(struct device *dev,
                           struct device_attribute *attr, char *buf)
{
	struct pcf50633_mbc *mbc = __to_mbc(dev);
	int ma;
	u8 mbcreg;

	mbcreg = pcf50633_reg_read(child_to_pcf50633(mbc), PCF50633_REG_MBCC7);	
	
	switch (mbcreg & PCF50633_MBCC7_USB_MASK) {
	case PCF50633_MBCC7_USB_100mA:
		ma = 100;
		break;
	case PCF50633_MBCC7_USB_500mA:
		ma = 500;
		break;
	case PCF50633_MBCC7_USB_1000mA:
		ma = 1000;
		break;
	case PCF50633_MBCC7_USB_SUSPEND:
		ma = 0;
		break;
	default:
		return -EINVAL;
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", ma);
}

static ssize_t set_usb_current(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct pcf50633_mbc *mbc = dev_get_drvdata(dev);
	unsigned long ma;
	
	if (strict_strtoul(buf, 10, &ma))
		return -EINVAL;
	
	switch (ma) {
		case 1000:
		case 500:
		case 100:
		case 0:
			break;
		default:
			return -EINVAL;
	}

	dev_info(dev, "set_chgmode: %lu\n", ma);
	pcf50633_mbc_usb_curlim_set(mbc, ma);

	return count;
}

static DEVICE_ATTR(usb_current, S_IRUGO | S_IWUSR, show_usb_current, set_usb_current);

/* Fast-Charge Current Limit */
static ssize_t show_fastcharge_current_limit(struct device *dev,
                            struct device_attribute *attr,
                            char *buf)
{
	return _show_chglim(dev, attr, buf, PCF50633_REG_MBCC5);
}
static DEVICE_ATTR(fastcharge_current_limit, S_IRUGO, show_fastcharge_current_limit, NULL);

/* Pre-Charge Current Limit */
static ssize_t show_precharge_current_limit(struct device *dev,
                            struct device_attribute *attr,
                            char *buf)
{
	return _show_chglim(dev, attr, buf, PCF50633_REG_MBCC3);
}
static DEVICE_ATTR(precharge_current_limit, S_IRUGO, show_precharge_current_limit, NULL);

static struct attribute *pcf50633_mbc_sysfs_entries[] = {
	&dev_attr_status.attr,
	&dev_attr_usb_current.attr,
	&dev_attr_fastcharge_current_limit.attr,
	&dev_attr_precharge_current_limit.attr,
	NULL,
};

static struct attribute_group mbc_attr_group = {
	.name	= NULL,			/* put in device directory */
	.attrs	= pcf50633_mbc_sysfs_entries,
};

static irqreturn_t pcf50633_mbc_irq_handler(int irq, void *data)
{
	struct pcf50633_mbc *mbc = data;
	int pcfirq, mbcirq;
	
	pcfirq = pcf50633_hwirq_to_irq(mbc->dev->parent, irq);
	mbcirq = pcf50633_mbc_irq_map[pcfirq];

	switch (pcfirq) {
	case PCF50633_IRQ_USBREM:
		mbc->usb_curlim = 0;
		pcf50633_mbc_usb_curlim_set(mbc, 0);
		cancel_delayed_work_sync(&mbc->work);
		
		power_supply_changed(&mbc->usb);
		break;
	
	case PCF50633_IRQ_USBINS:
		schedule_delayed_work(&mbc->work, __to_pdata(mbc)->delay);
	
	case PCF50633_MBC_IRQ_CHGHALT:
	case PCF50633_MBC_IRQ_BATFULL:
		power_supply_changed(&mbc->usb);
		break;
	
	case PCF50633_MBC_IRQ_ADPINS:
	case PCF50633_MBC_IRQ_ADPREM:
		if (mbc->adapter_connected)
			power_supply_changed(&mbc->adapter);
		break;

	default:
		break;
	}
	
	dev_dbg(mbc->dev, "IRQ %d: %s\n", irq,
	        pcf50633_mbc_irq_final_names[mbcirq]);
	
	return IRQ_HANDLED;
}

static int adapter_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	struct pcf50633_mbc * mbc = container_of(psy, struct pcf50633_mbc, adapter);
	u8 reg;
	
	
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		reg = pcf50633_reg_read(child_to_pcf50633(mbc), PCF50633_REG_MBCS2);
		val->intval = (reg & PCF50633_MBCS2_CHGS_ADAPTER) ? 1 : 0;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		reg = pcf50633_reg_read(child_to_pcf50633(mbc), PCF50633_REG_MBCS2);
		reg &= PCF50633_MBCS2_MBC_MASK;

		if (reg == PCF50633_MBCS2_MBC_ADP_PRE || reg == PCF50633_MBCS2_MBC_ADP_PRE_WAIT)
			val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
		else if (reg == PCF50633_MBCS2_MBC_ADP_FAST || reg == PCF50633_MBCS2_MBC_ADP_FAST_WAIT)
			val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
		else
			val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int usb_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	struct pcf50633_mbc * mbc = container_of(psy, struct pcf50633_mbc, usb);
	u8 reg;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		reg = pcf50633_reg_read(child_to_pcf50633(mbc), PCF50633_REG_MBCS2);
		val->intval = (reg & PCF50633_MBCS2_CHGS_USB) ? 1 : 0;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		reg = pcf50633_reg_read(child_to_pcf50633(mbc), PCF50633_REG_MBCC7);	

		switch (reg & PCF50633_MBCC7_USB_MASK) {
		case PCF50633_MBCC7_USB_100mA:
			val->intval = 100000;
			break;
		case PCF50633_MBCC7_USB_500mA:
			val->intval = 500000;
			break;
		case PCF50633_MBCC7_USB_1000mA:
			val->intval = 1000000;
			break;
		case PCF50633_MBCC7_USB_SUSPEND:
			val->intval = 0;
			break;
		default:
			return -EINVAL;
		}
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = 1000000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		reg = pcf50633_reg_read(child_to_pcf50633(mbc), PCF50633_REG_MBCS2);
		reg &= PCF50633_MBCS2_MBC_MASK;

		if (reg == PCF50633_MBCS2_MBC_USB_PRE || reg == PCF50633_MBCS2_MBC_USB_PRE_WAIT)
			val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
		else if (reg == PCF50633_MBCS2_MBC_USB_FAST || reg == PCF50633_MBCS2_MBC_USB_FAST_WAIT)
			val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
		else
			val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int usb_set_property(struct power_supply *psy,
                            enum power_supply_property psp,
                            const union power_supply_propval *val)
{
	struct pcf50633_mbc * mbc = container_of(psy, struct pcf50633_mbc, usb);
	
	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		switch (val->intval) {
			case 1000000:
			case 500000:
			case 100000:
			case 0:
				break;
			default:
				return -EINVAL;
		}
		
		return pcf50633_mbc_usb_curlim_set(mbc, val->intval / 1000);
	default:
		return -EINVAL;
	}
	return 0;
}

static int usb_property_is_writeable(struct power_supply *psy, enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		return 1;
	default:
		return 0;
	}
}

static void pcf50633_mbc_curlim_from_adc(struct pcf50633 *pcf, 
                                         void *data, int res)
{
	struct pcf50633_mbc * mbc = (struct pcf50633_mbc *) data;
	int  ma;
	
	dev_info(mbc->dev, "curlim: adc returned %#.2x: %d %% of reference\n", 
	                    res, res * 100 / 47);

	/* Interpret charger type */
	if (res < ((ADC_NOM_CHG_DETECT_USB + ADC_NOM_CHG_DETECT_1A) / 2)) {
		/*
		 * Sanity - stop GPO driving out now that we have a 1A charger
		 * GPO controls USB Host power generation on GTA02
		 *
		 * TODO: FIX GPIO pcf50633_gpio_set(pcf, PCF50633_GPO, 0); */
		
		
		ma = __to_pdata(mbc)->reference_current_ma;
	} else
		/* If the PCF50633 ADC is disabled we fallback to a
		 * 100mA limit for safety. */
		ma = 100;

	pcf50633_mbc_usb_curlim_set(mbc, ma);
}

static void pcf50633_mbc_curlim_worker(struct work_struct *work)
{
	struct delayed_work * dwork;
	struct pcf50633 * pcf;
	struct pcf50633_mbc * mbc;
	
	dwork = to_delayed_work(work);
	mbc = container_of(dwork, struct pcf50633_mbc, work);
	pcf = child_to_pcf50633(mbc);

	if (mbc->usb_curlim) {
		pcf50633_mbc_usb_curlim_set(mbc, mbc->usb_curlim);
		return;
	}

	/*if (pcf->adc) {
		if (pcf50633_adc_async_read(pcf,
					__to_pdata(mbc)->adc_channel,
					pcf50633_mbc_curlim_from_adc,
					mbc) == 0)
			return;
	}*/

	/* If the PCF50633 ADC is disabled we fallback to a
	 * 100mA limit for safety. */
	pcf50633_mbc_usb_curlim_set(mbc, 100);
}

static void pcf50633_mbc_vbus_draw(struct pcf50633_mbc *mbc, int ma)
{
	dev_info(mbc->dev, "vbus draw %d mA\n", ma);

	mbc->usb_curlim = ma;
	schedule_delayed_work(&mbc->work, __to_pdata(mbc)->delay);
}

static enum power_supply_property adapter_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
};

static enum power_supply_property usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CURRENT_MAX,
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

typedef void (*ops_vbus_draw_fn)(struct pcf50633_mbc_ops *, int);
typedef void (*ops_get_fn)(struct pcf50633_mbc_ops *);

static int __devinit pcf50633_mbc_probe(struct platform_device *pdev)
{
	struct pcf50633_mbc *mbc;
	struct pcf50633 *pcf;
	struct pcf50633_mbc_platform_data *pdata = pdev->dev.platform_data;
	int ret;
	int i;
	u8 mbcs1;

	mbc = kzalloc(sizeof(*mbc), GFP_KERNEL);
	if (!mbc)
		return -ENOMEM;
	platform_set_drvdata(pdev, mbc);
	mbc->dev = &pdev->dev;

	mbc->ops.get_status = (ops_get_fn) &pcf50633_mbc_get_status;
	//mbc->ops.usb_curlim_set = pcf50633_mbc_usb_curlim_set;
	mbc->ops.vbus_draw = (ops_vbus_draw_fn) &pcf50633_mbc_vbus_draw;
	INIT_DELAYED_WORK(&mbc->work, pcf50633_mbc_curlim_worker);

	pcf = child_to_pcf50633(mbc);
	pcf->mbc = mbc;

	/* Set up IRQ handlers */
	for (i = 0; i < PCF50633_MBC_NUM_IRQS; ++i) {
		mbc->irqs[i] = platform_get_irq_byname(pdev, pcf50633_mbc_irq_names[i]);
		ret = request_threaded_irq(mbc->irqs[i], NULL,
						pcf50633_mbc_irq_handler, 0,
						pcf50633_mbc_irq_final_names[i], mbc);
		if (ret) {
			dev_err(&pdev->dev, "Failed to request %s (%d) irq: %d\n",
									pcf50633_mbc_irq_names[i], 
									mbc->irqs[i], ret);
			goto err_free_irq;
		}
	}
	dev_dbg(mbc->dev, "Allocated irqs from %d to %d\n", 
	                   mbc->irqs[0], mbc->irqs[PCF50633_MBC_NUM_IRQS-1]);

	/* Create power supplies */
	mbc->adapter.name			= "adapter";
	mbc->adapter.type			= POWER_SUPPLY_TYPE_MAINS;
	mbc->adapter.properties		= adapter_props;
	mbc->adapter.num_properties	= ARRAY_SIZE(adapter_props);
	mbc->adapter.get_property	= &adapter_get_property;
	mbc->adapter.supplied_to	= pdata->batteries;
	mbc->adapter.num_supplicants	= pdata->num_batteries;

	mbc->usb.name				= "usb";
	mbc->usb.type				= POWER_SUPPLY_TYPE_USB;
	mbc->usb.properties			= usb_props;
	mbc->usb.num_properties		= ARRAY_SIZE(usb_props);
	mbc->usb.get_property		= usb_get_property;
	mbc->usb.set_property		= usb_set_property;
	mbc->usb.property_is_writeable	= usb_property_is_writeable;
	mbc->usb.supplied_to		= pdata->batteries;
	mbc->usb.num_supplicants	= pdata->num_batteries;
	
	ret = power_supply_register(&pdev->dev, &mbc->usb);
	if (ret) {
		dev_err(&pdev->dev, "failed to register usb\n");
		goto err_ps_usb;
	}

	if (pdata->adapter_connected) {
		ret = power_supply_register(&pdev->dev, &mbc->adapter);
		if (ret) {
			dev_err(&pdev->dev, "failed to register adapter\n");
			goto err_ps_adapter;
		}
		mbc->adapter_connected = 1;
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
		mbc->usb_curlim = 500;
		
		pcf50633_mbc_irq_handler(mbc->irqs[PCF50633_MBC_IRQ_USBINS], mbc);
	}
	if (mbcs1 & PCF50633_MBCS1_ADAPTPRES)
		pcf50633_mbc_irq_handler(mbc->irqs[PCF50633_MBC_IRQ_ADPINS], mbc);

	return 0;

err_ps_adapter:
	power_supply_unregister(&mbc->usb);
err_ps_usb:
err_free_irq:
	for (--i; i >= 0; --i)
			free_irq(mbc->irqs[i], mbc);
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
	if (mbc->adapter_connected)
		power_supply_unregister(&mbc->adapter);

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
