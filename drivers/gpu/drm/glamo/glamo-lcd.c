/*
 * SMedia Glamo 336x/337x display
 *
 * Copyright (C) 2008-2009 Thomas White <taw@bitwiz.org.uk>
 *
 * Based on glamo-fb.c (C) 2007-2008 by Openmoko, Inc.
 * Author: Harald Welte <laforge@openmoko.org>
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *
 *
 * Based on intel_display.c and intel_crt.c from drivers/gpu/drm/i915
 *  to which the following licence applies:
 *
 * Copyright © 2006-2007 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * Authors:
 *	Eric Anholt <eric@anholt.net>
 *
 */

#include <drm/drmP.h>
#include <drm/glamo_drm.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_crtc.h>
#include <linux/glamofb.h>
#include <linux/jbt6k74.h>

#include <linux/mfd/glamo-core.h>
#include "glamo-driver.h"
#include "glamo-kms-fb.h"


int reg_read_lcd(struct glamodrm_handle *gdrm, u_int16_t reg)
{
	int i = 0;

	for (i = 0; i != 2; i++)
		nop();

	return ioread16(gdrm->lcd_base + reg);
}


void reg_write_lcd(struct glamodrm_handle *gdrm,
                          u_int16_t reg, u_int16_t val)
{
	int i = 0;

	for (i = 0; i != 2; i++)
		nop();

	iowrite16(val, gdrm->lcd_base + reg);
}


void reg_set_bit_mask_lcd(struct glamodrm_handle *gdrm,
                                 u_int16_t reg, u_int16_t mask,
                                 u_int16_t val)
{
	u_int16_t tmp;

	val &= mask;

	tmp = reg_read_lcd(gdrm, reg);
	tmp &= ~mask;
	tmp |= val;
	reg_write_lcd(gdrm, reg, tmp);
}


/* Note that this has nothing at all to do with the engine command queue
 * in glamo-cmdq.c */
static inline int glamo_lcd_cmdq_empty(struct glamodrm_handle *gdrm)
{
	/* DGCMdQempty -- 1 == command queue is empty */
	return reg_read_lcd(gdrm, GLAMO_REG_LCD_STATUS1) & (1 << 15);
}


/* call holding gfb->lock_cmd  when locking, until you unlock */
int glamo_lcd_cmd_mode(struct glamodrm_handle *gdrm, int on)
{
	int timeout;
	if (on == gdrm->lcd_cmd_mode)
		return 0;

	DRM_DEBUG("glamofb_cmd_mode ( %d => %d )\n", on, gdrm->lcd_cmd_mode);
	if (on) {

		timeout = 20000;
		while ((!glamo_lcd_cmdq_empty(gdrm)) && (timeout--))
			/* yield() */;
		if (timeout < 0) {
			DRM_ERROR("LCD command queue failed to empty\n");
			return -EIO;
		}
		DRM_DEBUG("cmd queue empty in %d\n", 2000000 - timeout);

		/* display the entire frame then switch to command */
		reg_write_lcd(gdrm, GLAMO_REG_LCD_COMMAND1,
			  GLAMO_LCD_CMD_TYPE_DISP |
			  GLAMO_LCD_CMD_DATA_FIRE_VSYNC);

		/* wait until lcd idle */
		timeout = 2000000;
		while ((!(reg_read_lcd(gdrm, GLAMO_REG_LCD_STATUS2) & (1 << 12)))
		          && (timeout--))
			udelay(1);			
		if (timeout < 0) {
			DRM_ERROR("LCD failed to reach idle\n");
			return -EIO;
		}
		DRM_DEBUG("lcd status idle in %d\n", 2000000 - timeout);

		//mdelay(100);

	} else {
		/* RGB interface needs vsync/hsync */
		int mode;
		mode = reg_read_lcd(gdrm, GLAMO_REG_LCD_MODE3);
		if ( mode & GLAMO_LCD_MODE3_RGB)
			reg_write_lcd(gdrm, GLAMO_REG_LCD_COMMAND1,
				  GLAMO_LCD_CMD_TYPE_DISP |
				  GLAMO_LCD_CMD_DATA_DISP_SYNC);

		reg_write_lcd(gdrm, GLAMO_REG_LCD_COMMAND1,
			  GLAMO_LCD_CMD_TYPE_DISP |
			  GLAMO_LCD_CMD_DATA_DISP_FIRE);
	}

	gdrm->lcd_cmd_mode = on;
	return 0;
}


static struct glamo_script lcd_init_script[] = {
	{ GLAMO_REG_LCD_MODE1, 0x0020 },
	/* no display rotation, no hardware cursor, no dither, no gamma,
	 * no retrace flip, vsync low-active, hsync low active,
	 * no TVCLK, no partial display, hw dest color from fb,
	 * no partial display mode, LCD1, software flip,  */
	{ GLAMO_REG_LCD_MODE2, 0x9020 },
	  /* video flip, no ptr, no ptr, dhclk off,
	   * normal mode,  no cpuif,
	   * res, serial msb first, single fb, no fr ctrl,
	   * cpu if bits all zero, no crc
	   * 0000 0000 0010  0000 */
	{ GLAMO_REG_LCD_MODE3, 0x0b40 },
	  /* src data rgb565, res, 18bit rgb666
	   * 000 01 011 0100 0000 */
	{ GLAMO_REG_LCD_POLARITY, 0x440c },
	  /* DE high active, no cpu/lcd if, cs0 force low, a0 low active,
	   * np cpu if, 9bit serial data, sclk rising edge latch data
	   * 01 00 0 100 0 000 01 0 0 */
	/* The following values assume 640*480@16bpp */
	/* FIXME: fb0 has not yet been allocated! */
	{ GLAMO_REG_LCD_A_BASE1, PAGE_SIZE }, /* display A base address 15:0 */
	{ GLAMO_REG_LCD_A_BASE2, 0x0000 }, /* display A base address 22:16 */
	{ GLAMO_REG_LCD_B_BASE1, 0x6000 }, /* display B base address 15:0 */
	{ GLAMO_REG_LCD_B_BASE2, 0x0009 }, /* display B base address 22:16 */
	{ GLAMO_REG_LCD_CURSOR_BASE1, 0xC000 }, /* cursor base address 15:0 */
	{ GLAMO_REG_LCD_CURSOR_BASE2, 0x0012 }, /* cursor base address 22:16 */
	{ GLAMO_REG_LCD_COMMAND2, 0x0000 }, /* display page A */
};

static int glamo_lcd_run_script(struct glamodrm_handle *gdrm,
                                struct glamo_script *script, int len)
{
	int i;

	for (i = 0; i < len; i++) {
		struct glamo_script *line = &script[i];

		if (line->reg == 0xffff)
			return 0;
		else if (line->reg == 0xfffe)
			msleep(line->val);
		else
			reg_write_lcd(gdrm, script[i].reg, script[i].val);
	}

	return 0;
}


void glamo_lcd_power(struct glamodrm_handle *gdrm, int mode)
{
	
	DRM_DEBUG("dpms %s\n", ((char*[]) {"on", "standby", "suspend", "off"})[mode]);

	if ( mode == DRM_MODE_DPMS_ON) {
		glamo_engine_enable(gdrm->glamo_core, GLAMO_ENGINE_LCD);
		if (gdrm->glamo_core->pdata->fb_data->mode_change)
		    gdrm->glamo_core->pdata->fb_data->mode_change(NULL);
		//jbt6k74_setpower(JBT_POWER_MODE_NORMAL);
		//msleep(500);
	} else {
		//jbt6k74_setpower(JBT_POWER_MODE_OFF);
		if (gdrm->glamo_core->pdata->fb_data->mode_change)
		    gdrm->glamo_core->pdata->fb_data->mode_change(NULL);
		glamo_engine_suspend(gdrm->glamo_core, GLAMO_ENGINE_LCD);
	}
}


void glamo_lcd_init(struct glamodrm_handle *gdrm)
{
	/* Initial setup of the LCD controller */
	glamo_engine_enable(gdrm->glamo_core, GLAMO_ENGINE_LCD);
	glamo_engine_reset(gdrm->glamo_core, GLAMO_ENGINE_LCD);

	glamo_lcd_run_script(gdrm, lcd_init_script, ARRAY_SIZE(lcd_init_script));
	glamo_engine_suspend(gdrm->glamo_core, GLAMO_ENGINE_LCD);
}
