/*
 *  Copyright (C) 2009-2010, Lars-Peter Clausen <lars@metafoo.de>
 *      PCF50633 backlight device driver
 *
 *  This program is free software; you can redistribute	 it and/or modify it
 *  under  the terms of	 the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the	License, or (at your
 *  option) any later version.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/mfd/pcf50633/core.h>
#include <linux/mfd/pcf50633/backlight.h>

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>

#include <linux/backlight.h>
#include <linux/fb.h>

struct pcf50633_bl {
	struct pcf50633_bl_ops ops;

	struct device *dev;
	struct backlight_device *bl;

	unsigned int brightness;
	unsigned int brightness_limit;
};

static inline struct pcf50633 * __to_pcf(struct pcf50633_bl *bl)
{
	return dev_to_pcf50633(bl->dev->parent);
}

/*
 * pcf50633_bl_set_brightness_limit
 *
 * Update the brightness limit for the pc50633 backlight. The actual brightness
 * will not go above the limit. This is useful to limit power drain for example
 * on low battery.
 *
 * @dev: Pointer to a pcf50633 device
 * @limit: The brightness limit. Valid values are 0-63
 */
int pcf50633_bl_set_brightness_limit(struct pcf50633 *pcf, unsigned int limit)
{
	struct pcf50633_bl *pcf_bl = pcf->bl;

	if (!pcf_bl)
		return -ENODEV;

	pcf_bl->brightness_limit = limit & 0x3f;
	backlight_update_status(pcf_bl->bl);

    return 0;
}

static int pcf50633_bl_update_status(struct backlight_device *bl)
{
	struct pcf50633_bl *pcf_bl = bl_get_data(bl);
	struct pcf50633 *pcf = __to_pcf(pcf_bl);
	unsigned int enable;

	if (bl->props.state & (BL_CORE_SUSPENDED | BL_CORE_FBBLANK) ||
		bl->props.power != FB_BLANK_UNBLANK)
		enable = 0;
	else
		enable = 1;
	
	if (bl->props.brightness > pcf_bl->brightness_limit)
		bl->props.brightness = pcf_bl->brightness_limit;

	if (enable)
		pcf50633_reg_write(pcf, PCF50633_REG_LEDOUT, bl->props.brightness);
	pcf50633_reg_write(pcf, PCF50633_REG_LEDENA, enable);
	if (!enable)
		pcf50633_reg_write(pcf, PCF50633_REG_LEDOUT, bl->props.brightness);

	dev_dbg(pcf_bl->dev, "update status %s at %d%%\n",
	        (enable) ? "enabled" : "disabled",
	        bl->props.brightness * 100 / pcf_bl->brightness_limit);

	return 0;
}

static int pcf50633_bl_ops_set_power(struct pcf50633_bl_ops *ops, unsigned int power)
{
	struct pcf50633_bl *pcf_bl = container_of(ops, struct pcf50633_bl, ops);

	pcf_bl->bl->props.power = power;
	return pcf50633_bl_update_status(pcf_bl->bl);
}

static int pcf50633_bl_get_brightness(struct backlight_device *bl)
{
	struct pcf50633_bl *pcf_bl = bl_get_data(bl);
	return pcf_bl->brightness;
}

static const struct backlight_ops pcf50633_bl_ops = {
	.get_brightness = pcf50633_bl_get_brightness,
	.update_status	= pcf50633_bl_update_status,
	.options	= BL_CORE_SUSPENDRESUME,
};

static int __devinit pcf50633_bl_probe(struct platform_device *pdev)
{
	struct pcf50633_bl_platform_data *pdata = pdev->dev.platform_data;
	struct pcf50633_bl *pcf_bl;
	struct pcf50633 *pcf;
	struct backlight_properties props;

	pcf_bl = devm_kzalloc(&pdev->dev, sizeof(*pcf_bl), GFP_KERNEL);
	if (!pcf_bl)
		return -ENOMEM;

	pcf_bl->dev = &pdev->dev;
	pcf = __to_pcf(pcf_bl);
	pcf->bl = pcf_bl;
	pcf_bl->brightness_limit = pdata->default_brightness_limit;
	pcf_bl->ops.set_power = pcf50633_bl_ops_set_power;

	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = 0x3f;
	props.brightness = pdata->default_brightness;
	pcf_bl->bl = backlight_device_register(pdev->name, &pdev->dev, pcf_bl,
						&pcf50633_bl_ops, &props);

	if (IS_ERR(pcf_bl->bl)) {
		pcf->bl = NULL;
		return PTR_ERR(pcf_bl->bl);
	}

	platform_set_drvdata(pdev, pcf_bl);
	
	/*	disable output and set brightness, ramp_time	*/
	pcf50633_reg_write(pcf, PCF50633_REG_LEDENA, 0);
	pcf50633_reg_write(pcf, PCF50633_REG_LEDOUT, pcf_bl->bl->props.brightness);
	pcf50633_reg_write(pcf, PCF50633_REG_LEDDIM, pdata->ramp_time);

	//pcf_bl->bl->props.power = FB_BLANK_UNBLANK;
	//backlight_update_status(pcf_bl->bl);

	return 0;
}

static int __devexit pcf50633_bl_remove(struct platform_device *pdev)
{
	struct pcf50633_bl *pcf_bl = platform_get_drvdata(pdev);

	pcf50633_reg_write(child_to_pcf50633(pcf_bl), PCF50633_REG_LEDENA, 0);

	__to_pcf(pcf_bl)->bl = NULL;
	backlight_device_unregister(pcf_bl->bl);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver pcf50633_bl_driver = {
	.probe =	pcf50633_bl_probe,
	.remove =	__devexit_p(pcf50633_bl_remove),
	.driver = {
		.name = "pcf50633-backlight",
	},
};

module_platform_driver(pcf50633_bl_driver);

MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_DESCRIPTION("PCF50633 backlight driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pcf50633-backlight");
