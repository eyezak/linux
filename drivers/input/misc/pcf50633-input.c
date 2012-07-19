/* NXP PCF50633 Input Driver
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
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/slab.h>


#define PCF50633_OOCSTAT_ONKEY	0x01
#define PCF50633_REG_OOCSTAT	0x12
#define PCF50633_REG_OOCMODE	0x10

struct pcf50633_input {
	struct device *dev;
	struct input_dev *input_dev;
	int irq_pressed;
	int irq_released;
	int irq_held;
	unsigned long seconds;
};

static inline int force_shutdown_timeout(struct pcf50633_input *input)
{
	return *((int*) input->dev->platform_data);
}

static const char * irq_text(struct pcf50633_input *input, int irq)
{
	if (irq == input->irq_pressed)
		return "pressed";
	if (irq == input->irq_released)
		return "released";
	if (irq == input->irq_held)
		return "held";
	return "";
}
static irqreturn_t pcf50633_input_irq(int irq, void *data)
{
	struct pcf50633_input *input;
	int onkey_released;

	input = data;
	dev_dbg(&input->input_dev->dev, "button %s\n", irq_text(input, irq));

	/* We report only one event depending on the key press status */
	onkey_released = pcf50633_reg_read(child_to_pcf50633(input), PCF50633_REG_OOCSTAT)
						& PCF50633_OOCSTAT_ONKEY;

	if (irq == input->irq_pressed && !onkey_released) {
		input->seconds = get_seconds();
		input_report_key(input->input_dev, KEY_POWER, 1);
	}
	else if (irq == input->irq_released && onkey_released) {
		input_report_key(input->input_dev, KEY_POWER, 0);
		dev_dbg(&input->input_dev->dev, "seconds diff = %lu\n", get_seconds() - input->seconds);
	
		if (get_seconds() - input->seconds >= force_shutdown_timeout(input))
			pcf50633_reg_set_bit_mask(child_to_pcf50633(input),
			     PCF50633_REG_OOCSHDWN, 0x01, 0x01);
		else
			input->seconds = ULONG_MAX;
	}

	input_sync(input->input_dev);

	return IRQ_HANDLED;

}

static int __devinit pcf50633_input_probe(struct platform_device *pdev)
{
	struct pcf50633_input *input;
	struct input_dev *input_dev;
	int ret;

	input = kzalloc(sizeof(*input), GFP_KERNEL);
	if (!input)
		return -ENOMEM;
	
	input->irq_released = platform_get_irq_byname(pdev, "ONKEYR");
	if (input->irq_released <= 0) {
		dev_err(&pdev->dev, "Failed to get released irq: %d\n",
		                     input->irq_released);
		ret = input->irq_released ?: -EINVAL;
		goto err_alloc;
	}

	input->irq_pressed = platform_get_irq_byname(pdev, "ONKEYF");
	if (input->irq_pressed <= 0) {
		dev_err(&pdev->dev, "Failed to get pressed irq: %d\n",
		                     input->irq_pressed);
		ret = input->irq_pressed ?: -EINVAL;
		goto err_alloc;
	}

	input->irq_held = platform_get_irq_byname(pdev, "ONKEY1S");
	if (input->irq_held <= 0) {
		dev_err(&pdev->dev, "Failed to get pressed irq: %d\n",
		                     input->irq_pressed);
		ret = input->irq_pressed ?: -EINVAL;
		goto err_alloc;
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		ret = -ENOMEM;
		goto err_alloc;
	}

	platform_set_drvdata(pdev, input);
	input->dev = &pdev->dev;
	input->input_dev = input_dev;
	input->seconds = ULONG_MAX;

	input_dev->name = "PCF50633 PMU events";
	input_dev->id.bustype = BUS_I2C;
	input_dev->evbit[0] = BIT(EV_KEY) | BIT(EV_PWR);
	set_bit(KEY_POWER, input_dev->keybit);

	ret = input_register_device(input_dev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register input device: %d\n", ret);
		input_free_device(input_dev);
		goto err_input_register;
	}
	
	ret = request_threaded_irq(input->irq_released, NULL, pcf50633_input_irq, 0,
	                           "pcf550633-input:released", input);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request released irq: %d\n", ret);
		goto err_reqirq_rel;
	}
	dev_dbg(&pdev->dev, "ONKEYR irq is %d\n", input->irq_released);

	ret = request_threaded_irq(input->irq_pressed, NULL, pcf50633_input_irq, 0,
	                           "pcf550633-input:pressed", input);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request pressed irq: %d\n", ret);
		goto err_reqirq_press;
	}
	dev_dbg(&pdev->dev, "ONKEYF irq is %d\n", input->irq_pressed);

	ret = request_threaded_irq(input->irq_held, NULL, pcf50633_input_irq, 0,
	                           "pcf550633-input:held", input);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request held irq: %d\n", ret);
		goto err_reqirq_held;
	}
	dev_dbg(&pdev->dev, "ONKEY1S irq is %d\n", input->irq_held);

	return 0;

err_reqirq_held:
	free_irq(input->irq_pressed, input);
err_reqirq_press:
	free_irq(input->irq_released, input);
err_reqirq_rel:
	input_unregister_device(input->input_dev);
err_input_register:
	input_free_device(input->input_dev);
err_alloc:
	kfree(input);
	
	return ret;
}

static int __devexit pcf50633_input_remove(struct platform_device *pdev)
{
	struct pcf50633_input *input  = platform_get_drvdata(pdev);

	free_irq(input->irq_released, input);
	free_irq(input->irq_pressed, input);
	free_irq(input->irq_held, input);

	input_unregister_device(input->input_dev);
	input_free_device(input->input_dev);
	kfree(input);

	return 0;
}

static struct platform_driver pcf50633_input_driver = {
	.driver = {
		.name = "pcf50633-input",
	},
	.probe = pcf50633_input_probe,
	.remove = __devexit_p(pcf50633_input_remove),
};
module_platform_driver(pcf50633_input_driver);

MODULE_AUTHOR("Balaji Rao <balajirrao@openmoko.org>");
MODULE_DESCRIPTION("PCF50633 input driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pcf50633-input");
