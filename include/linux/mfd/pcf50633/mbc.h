/*
 * mbc.h  -- Driver for NXP PCF50633 Main Battery Charger
 *
 * (C) 2006-2008 by Openmoko, Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __LINUX_MFD_PCF50633_MBC_H
#define __LINUX_MFD_PCF50633_MBC_H

#include <linux/mfd/pcf50633/core.h>
#include <linux/mfd/pcf50633/platform.h>
#include <linux/platform_device.h>

/* Charger status */
#define PCF50633_MBC_USB_ONLINE		0x01
#define PCF50633_MBC_USB_ACTIVE		0x02
#define PCF50633_MBC_ADAPTER_ONLINE	0x04
#define PCF50633_MBC_ADAPTER_ACTIVE	0x08


#endif

