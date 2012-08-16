/*

 * core.h  -- Core driver for NXP PCF50633
 *
 * (C) 2006-2008 by Openmoko, Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __LINUX_MFD_PCF50633_CORE_H
#define __LINUX_MFD_PCF50633_CORE_H

#include <linux/i2c.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/power_supply.h>

#include <linux/mfd/pcf50633/platform.h>
#include <linux/mfd/pcf50633/backlight.h>

struct pcf50633;

enum {
	/* Chip IRQs */
	/* regmap_irq_chip uses legacy irq domain ?? */
	PCF50633_IRQ_ADPINS,
	PCF50633_IRQ_ADPREM,
	PCF50633_IRQ_USBINS,
	PCF50633_IRQ_USBREM,
	PCF50633_IRQ_RESERVED1,
	PCF50633_IRQ_RESERVED2,
	PCF50633_IRQ_ALARM,
	PCF50633_IRQ_SECOND,
	PCF50633_IRQ_ONKEYR,
	PCF50633_IRQ_ONKEYF,
	PCF50633_IRQ_EXTON1R,
	PCF50633_IRQ_EXTON1F,
	PCF50633_IRQ_EXTON2R,
	PCF50633_IRQ_EXTON2F,
	PCF50633_IRQ_EXTON3R,
	PCF50633_IRQ_EXTON3F,
	PCF50633_IRQ_BATFULL,
	PCF50633_IRQ_CHGHALT,
	PCF50633_IRQ_THLIMON,
	PCF50633_IRQ_THLIMOFF,
	PCF50633_IRQ_USBLIMON,
	PCF50633_IRQ_USBLIMOFF,
	PCF50633_IRQ_ADCRDY,
	PCF50633_IRQ_ONKEY1S,
	PCF50633_IRQ_LOWSYS,
	PCF50633_IRQ_LOWBAT,
	PCF50633_IRQ_HIGHTMP,
	PCF50633_IRQ_AUTOPWRFAIL,
	PCF50633_IRQ_DWN1PWRFAIL,
	PCF50633_IRQ_DWN2PWRFAIL,
	PCF50633_IRQ_LEDPWRFAIL,
	PCF50633_IRQ_LEDOVP,
	PCF50633_IRQ_LDO1PWRFAIL,
	PCF50633_IRQ_LDO2PWRFAIL,
	PCF50633_IRQ_LDO3PWRFAIL,
	PCF50633_IRQ_LDO4PWRFAIL,
	PCF50633_IRQ_LDO5PWRFAIL,
	PCF50633_IRQ_LDO6PWRFAIL,
	PCF50633_IRQ_HCLDOPWRFAIL,
	PCF50633_IRQ_HCLDOOVL,

	/* Always last */
	PCF50633_NUM_IRQ,
};

struct pcf50633 {
	struct pcf50633_ops ops;

	struct device *dev;
	struct regmap *regmap;
	struct regmap_irq_chip_data *irq_data;

	struct pcf50633_platform_data *pdata;
	
	struct pcf50633_mbc *mbc;
	struct pcf50633_adc *adc;
	struct pcf50633_bl *bl;
	struct gpio_chip *gpio;

	struct mutex lock;

	unsigned int irq;
	unsigned int irq_base;
	u8 resume_reason[5];
	int onkey1s_held:1;
	int is_suspended:1;
};


struct pcf50633 *dev_to_pcf50633(struct device *dev);
#define child_to_pcf50633(_c) dev_to_pcf50633(_c->dev->parent)

int pcf50633_hwirq_to_irq(struct device *dev, int irq);

int pcf50633_read_block(struct pcf50633 *, unsigned int reg,
					int nr_regs, u8 *data);
int pcf50633_write_block(struct pcf50633 *pcf, unsigned int reg,
					int nr_regs, u8 *data);
u8 pcf50633_reg_read(struct pcf50633 *, unsigned int reg);
int pcf50633_reg_write(struct pcf50633 *pcf, unsigned int reg, unsigned int val);

int pcf50633_reg_set_bit_mask(struct pcf50633 *pcf, unsigned int reg, unsigned int mask, unsigned int val);
int pcf50633_reg_clear_bits(struct pcf50633 *pcf, unsigned int reg, unsigned int bits);


#endif
