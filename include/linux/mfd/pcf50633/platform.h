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

#ifndef __LINUX_MFD_PCF50633_PLATFORM_H
#define __LINUX_MFD_PCF50633_PLATFORM_H

#include <linux/mfd/pcf50633/regs.h>


#define PCF50633_NUM_REGULATORS	11

#define PCF50633_GPIO1		0
#define PCF50633_GPIO2		1
#define PCF50633_GPIO3		2
#define PCF50633_GPO		3

/*
* @default_brightness: Backlight brightness is initialized to this value
*
* Brightness to be used after the driver has been probed.
* Valid range 0-63.
*
* @default_brightness_limit: The actual brightness is limited by this value
*
* Brightness limit to be used after the driver has been probed. This is useful
* when it is not known how much power is available for the backlight during
* probe.
* Valid range 0-63. Can be changed later with pcf50633_bl_set_brightness_limit.
*
* @ramp_time: Display ramp time when changing brightness
*
* When changing the backlights brightness the change is not instant, instead
* it fades smooth from one state to another. This value specifies how long
* the fade should take. The lower the value the higher the fade time.
* Valid range 0-255
*/
struct pcf50633_bl_platform_data {
	unsigned int	default_brightness;
	unsigned int	default_brightness_limit;
	uint8_t		ramp_time;
};

struct pcf50633_mbc_platform_data {
	char **batteries;
	int num_batteries;
	
	int reference_current_ma;
	int delay;
	int adc_channel:2;
	int adapter_connected:1;
};

struct pcf50633_adc_channel {
	u8 c1flags;
	u8 c2flags;
	u8 c3flags;
	int m;
	int b;
};
struct pcf50633_adc_platform_data {
	u8 channel_mask;
	struct pcf50633_adc_channel channels[4];
};

struct pcf50633_platform_data {
	/*
	 * Should be set accordingly to the reference resistor used, see
	 * I_{ch(ref)} charger reference current in the pcf50633 User
	 * Manual.
	 */
	struct regulator_init_data reg_init_data[PCF50633_NUM_REGULATORS];

	struct pcf50633_bl_platform_data backlight_data;
	
	struct pcf50633_mbc_platform_data mbc_data;
	
	struct pcf50633_adc_platform_data adc_data;

	int force_shutdown_timeout;		/* set to -1 to disable */
	int gpio_base;
	int irq_base;
	u8 resumers[5];
};

struct pcf50633_mbc_ops {
	void (*vbus_draw)(struct pcf50633_mbc_ops *ops, int ma);
	void (*get_status)(struct pcf50633_mbc_ops *ops);
};

struct pcf50633_ops {
	void (* shutdown)(struct pcf50633_ops * ops);
};

struct pcf50633_bl_ops {
	int (* set_power)(struct pcf50633_bl_ops *, unsigned int);
	int (* set_brightness_limit)(struct pcf50633_bl_ops *, unsigned int);
};


inline struct pcf50633_ops * dev_to_pcf50633_ops(struct device * dev)
{
	return dev_get_drvdata(dev);
}

inline struct pcf50633_mbc_ops * dev_to_pcf50633_mbc_ops(struct device * dev)
{
	return dev_get_drvdata(dev);
}

inline struct pcf50633_bl_ops * dev_to_pcf50633_bl_ops(struct device * dev)
{
	return dev_get_drvdata(dev);
}


#endif
