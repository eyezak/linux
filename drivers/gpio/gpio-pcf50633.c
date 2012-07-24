/* NXP PCF50633 GPIO Driver
 *
 * (C) 2006-2008 by Openmoko, Inc.
 * Author: Balaji Rao <balajirrao@openmoko.org>
 * Copyright 2010, Lars-Peter Clausen <lars@metafoo.de>
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
#include <linux/platform_device.h>

#include <linux/mfd/pcf50633/core.h>
#include <linux/mfd/pcf50633/gpio.h>
#include <linux/mfd/pcf50633/pmic.h>
#include <linux/gpio.h>

#define PCF50633_REG_GPIOCTL	0x13
#define PCF50633_REG_GPIOCFG(x) (0x14 + (x))

static const u8 pcf50633_regulator_registers[PCF50633_NUM_REGULATORS] = {
	[PCF50633_REGULATOR_AUTO]	= PCF50633_REG_AUTOOUT,
	[PCF50633_REGULATOR_DOWN1]	= PCF50633_REG_DOWN1OUT,
	[PCF50633_REGULATOR_DOWN2]	= PCF50633_REG_DOWN2OUT,
	[PCF50633_REGULATOR_MEMLDO]	= PCF50633_REG_MEMLDOOUT,
	[PCF50633_REGULATOR_LDO1]	= PCF50633_REG_LDO1OUT,
	[PCF50633_REGULATOR_LDO2]	= PCF50633_REG_LDO2OUT,
	[PCF50633_REGULATOR_LDO3]	= PCF50633_REG_LDO3OUT,
	[PCF50633_REGULATOR_LDO4]	= PCF50633_REG_LDO4OUT,
	[PCF50633_REGULATOR_LDO5]	= PCF50633_REG_LDO5OUT,
	[PCF50633_REGULATOR_LDO6]	= PCF50633_REG_LDO6OUT,
	[PCF50633_REGULATOR_HCLDO]	= PCF50633_REG_HCLDOOUT,
};

static inline struct pcf50633 *gpio_chip_to_pcf50633(struct gpio_chip *chip)
{
	struct pcf50633 *pcf = dev_to_pcf50633(chip->dev->parent);
	return pcf;
}

static void pcf50633_gpio_set_value(struct gpio_chip *chip, unsigned gpio, int value)
{
	struct pcf50633 *pcf = gpio_chip_to_pcf50633(chip);
	u8 reg;

	reg = PCF50633_REG_GPIOCFG(gpio);

	pcf50633_reg_set_bit_mask(pcf, reg, 0x07, value ? 0x7 : 0x0);
}

static int pcf50633_gpio_get_value(struct gpio_chip *chip, unsigned gpio)
{
	struct pcf50633 *pcf = gpio_chip_to_pcf50633(chip);
	return pcf50633_reg_read(pcf, PCF50633_REG_GPIOCFG(gpio)) >> 3;
}


static int pcf50633_gpio_direction_output(struct gpio_chip *chip, unsigned gpio,
                                          int value)
{
	struct pcf50633 *pcf = gpio_chip_to_pcf50633(chip);
	int ret;

	ret = pcf50633_gpio_set_config(pcf, pcf->pdata->gpio_base + gpio,
	                               PCF50633_GPIO_CONFIG_OUTPUT);
	if (!ret)
	    pcf50633_gpio_set_value(chip, gpio, value);

	return ret;
}

static int pcf50633_gpio_direction_input(struct gpio_chip *chip, unsigned gpio)
{
	return -ENOSYS;
}

int pcf50633_gpio_set_config(struct pcf50633 *pcf, unsigned gpio,
                              enum pcf50633_gpio_config config)
{
	u8 reg;
	u8 direction;
	int ret;

	gpio -= pcf->pdata->gpio_base;

	if (gpio < 3) {
	    direction = (config == PCF50633_GPIO_CONFIG_INPUT) ? (1 << gpio) : 0;
	    ret = pcf50633_reg_set_bit_mask(pcf, PCF50633_REG_GPIOCTL, (1 << gpio),
					    direction);
	    if (ret) {
			return ret;
		}
	} else if (gpio > 3 || config == PCF50633_GPIO_CONFIG_INPUT) {
	    return -EINVAL;
	}

	if (config != PCF50633_GPIO_CONFIG_INPUT) {
	    reg = PCF50633_REG_GPIOCFG(gpio);
	    ret = pcf50633_reg_set_bit_mask(pcf, reg, 0x0f, config);
	}

	return ret;
}
EXPORT_SYMBOL_GPL(pcf50633_gpio_set_config);

int pcf50633_gpio_power_supply_set(struct pcf50633 *pcf,
					int gpio, int regulator, int on)
{
	u8 reg, val, mask;

	gpio -= pcf->pdata->gpio_base;

	/* the *ENA register is always one after the *OUT register */
	reg = pcf50633_regulator_registers[regulator] + 1;

	val = !!on << (gpio - PCF50633_GPIO1);
	mask = 1 << (gpio - PCF50633_GPIO1);

	return pcf50633_reg_set_bit_mask(pcf, reg, mask, val);
}
EXPORT_SYMBOL_GPL(pcf50633_gpio_power_supply_set);

u8 pcf50633_gpio_get(struct pcf50633 *pcf, int gpio)
{
	struct gpio_chip * chip = platform_get_drvdata(pcf->gpio_pdev);
	return chip->get(chip, gpio);
}
EXPORT_SYMBOL_GPL(pcf50633_gpio_get);

int pcf50633_gpio_set(struct pcf50633 *pcf, int gpio, u8 val)
{
	struct gpio_chip * chip = platform_get_drvdata(pcf->gpio_pdev);
	chip->set(chip, gpio, val);
	return 0;
}
EXPORT_SYMBOL_GPL(pcf50633_gpio_set);


static int __devinit pcf50633_gpio_probe(struct platform_device *pdev)
{
	struct pcf50633 *pcf = dev_to_pcf50633(pdev->dev.parent);
	struct gpio_chip * pcf_gpio;

	pcf_gpio = kzalloc(sizeof(struct gpio_chip), GFP_KERNEL);

	if (!pcf_gpio)
		return -ENOMEM;

	pcf_gpio->direction_input = pcf50633_gpio_direction_input;
	pcf_gpio->direction_output = pcf50633_gpio_direction_output;
	pcf_gpio->get = pcf50633_gpio_get_value;
	pcf_gpio->set = pcf50633_gpio_set_value;

	pcf_gpio->base = pcf->pdata->gpio_base;
	pcf_gpio->ngpio = 4;
	pcf_gpio->label = dev_name(pcf->dev);
	pcf_gpio->can_sleep = 1;
	pcf_gpio->owner = THIS_MODULE;
	pcf_gpio->dev = &pdev->dev;

	platform_set_drvdata(pdev, pcf_gpio);

	dev_info(&pdev->dev, "gpio chip %d to %d\n", pcf->pdata->gpio_base,
	                     pcf_gpio->base + pcf_gpio->ngpio - 1);
	return gpiochip_add(pcf_gpio);
}

static int __devexit pcf50633_gpio_remove(struct platform_device *pdev)
{
	struct gpio_chip *pcf_gpio = platform_get_drvdata(pdev);
	int ret;

	ret = gpiochip_remove(pcf_gpio);
	if (ret) {
		dev_err(&pdev->dev, "failed to remove gpio %d\n",
		        pcf_gpio->base);
		return ret;
	}

	platform_set_drvdata(pdev, NULL);
	kfree(pcf_gpio);

	return 0;
}

static struct platform_driver pcf50633_gpio_driver = {
	.probe = pcf50633_gpio_probe,
	.remove = __devexit_p(pcf50633_gpio_remove),
	.driver = {
		.name = "pcf50633-gpio",
		.owner = THIS_MODULE,
	},
};

int __init pcf50633_gpio_init(void)
{
	return platform_driver_register(&pcf50633_gpio_driver);
}
module_init(pcf50633_gpio_init);

void __exit pcf50633_gpio_exit(void)
{
	platform_driver_unregister(&pcf50633_gpio_driver);
}
module_exit(pcf50633_gpio_exit);

MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_DESCRIPTION("GPIO driver for the PCF50633");
MODULE_LICENSE("GPL");
