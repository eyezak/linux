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

#include <linux/mfd/pcf50633/core.h>
#include <linux/mfd/pcf50633/gpio.h>
#include <linux/mfd/pcf50633/pmic.h>

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>

#include <linux/gpio.h>

#define PCF50633_REG_GPIOCTL	0x13
#define PCF50633_REG_GPIOCFG(x) (0x14 + (x))

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

	if (gpio > 3)
	    return -EINVAL;

    ret = pcf50633_reg_set_bit_mask(pcf, PCF50633_REG_GPIOCTL, (1 << gpio), 0);
    if (ret)
		return ret;

    ret = pcf50633_reg_set_bit_mask(pcf, PCF50633_REG_GPIOCFG(gpio),
                                    0x0f, PCF50633_GPIO_CONFIG_OUTPUT);
	if (!ret)
	    pcf50633_gpio_set_value(chip, gpio, value);

	return ret;
}

static int pcf50633_gpio_direction_input(struct gpio_chip *chip, unsigned gpio)
{
	return -ENOSYS;
}

u8 pcf50633_gpio_get(struct pcf50633 *pcf, int gpio)
{
	struct gpio_chip * chip = pcf->gpio;
	return chip->get(chip, gpio);
}
EXPORT_SYMBOL_GPL(pcf50633_gpio_get);

int pcf50633_gpio_set(struct pcf50633 *pcf, int gpio, u8 val)
{
	struct gpio_chip * chip = pcf->gpio;
	chip->set(chip, gpio, val);
	return 0;
}
EXPORT_SYMBOL_GPL(pcf50633_gpio_set);

static int __devinit pcf50633_gpio_probe(struct platform_device *pdev)
{
	struct gpio_chip * pcf_gpio;

	pcf_gpio = kzalloc(sizeof(struct gpio_chip), GFP_KERNEL);

	if (!pcf_gpio)
		return -ENOMEM;
	
	pcf_gpio->base = *((int *)pdev->dev.platform_data);
	if (pcf_gpio->base <= 0) {
		kfree(pcf_gpio);
		return -EINVAL;
	}

	pcf_gpio->direction_input = pcf50633_gpio_direction_input;
	pcf_gpio->direction_output = pcf50633_gpio_direction_output;
	pcf_gpio->get = pcf50633_gpio_get_value;
	pcf_gpio->set = pcf50633_gpio_set_value;

	pcf_gpio->ngpio = 4;
	pcf_gpio->label = dev_name(&pdev->dev);
	pcf_gpio->can_sleep = 1;
	pcf_gpio->owner = THIS_MODULE;
	pcf_gpio->dev = &pdev->dev;

	platform_set_drvdata(pdev, pcf_gpio);
	child_to_pcf50633(pcf_gpio)->gpio = pcf_gpio;

	dev_info(&pdev->dev, "gpio chip %d to %d\n", pcf_gpio->base,
	                     pcf_gpio->base + pcf_gpio->ngpio - 1);
	return gpiochip_add(pcf_gpio);
}

static int __devexit pcf50633_gpio_remove(struct platform_device *pdev)
{
	struct gpio_chip *pcf_gpio = platform_get_drvdata(pdev);
	int ret;

	child_to_pcf50633(pcf_gpio)->gpio = NULL;
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
