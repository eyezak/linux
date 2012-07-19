/*
 * adc.h  -- Driver for NXP PCF50633 ADC
 *
 * (C) 2006-2008 by Openmoko, Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __LINUX_MFD_PCF50633_ADC_H
#define __LINUX_MFD_PCF50633_ADC_H

#include <linux/mfd/pcf50633/core.h>
#include <linux/platform_device.h>


#define PCF50633_MAX_ADC_FIFO_DEPTH 8

enum {
	PCF50633_ADC_CHANNEL_IN1,
	PCF50633_ADC_CHANNEL_IN3,
	PCF50633_ADC_CHANNEL_BATSNS,
	PCF50633_ADC_CHANNEL_BATTEMP,
};


#define ADC_NOM_CHG_DETECT_1A  6
#define ADC_NOM_CHG_DETECT_USB 43

struct pcf50633_adc_request {
	void (*callback)(struct pcf50633 *, void *, int);
	void *callback_param;

	int channel;
};


struct pcf50633_adc {
	struct device *dev;

	/* Private stuff */
	struct mutex queue_mutex;
	struct pcf50633_adc_channel channels[4];
	struct pcf50633_adc_request *queue[PCF50633_MAX_ADC_FIFO_DEPTH];

	u8 channel_mask;	
	int queue_head;
	int queue_tail;
	int irq;
};


extern int
pcf50633_adc_async_read(struct pcf50633 *pcf, int channel,
		void (*callback)(struct pcf50633 *, void *, int),
		void *callback_param);
extern int
pcf50633_adc_sync_read(struct pcf50633 *pcf, int channel);

#endif /* __LINUX_PCF50633_ADC_H */
