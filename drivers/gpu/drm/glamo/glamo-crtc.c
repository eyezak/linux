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
#include "glamo-drm-private.h"
#include "glamo-regs.h"
#include "glamo-kms-fb.h"
#include "glamo-display.h"

#define GLAMO_LCD_WIDTH_MASK 0x03FF
#define GLAMO_LCD_HEIGHT_MASK 0x03FF
#define GLAMO_LCD_PITCH_MASK 0x07FE
#define GLAMO_LCD_HV_TOTAL_MASK 0x03FF
#define GLAMO_LCD_HV_RETR_START_MASK 0x03FF
#define GLAMO_LCD_HV_RETR_END_MASK 0x03FF
#define GLAMO_LCD_HV_RETR_DISP_START_MASK 0x03FF
#define GLAMO_LCD_HV_RETR_DISP_END_MASK 0x03FF

extern int glamo_lcd_cmd_mode(struct glamodrm_handle *gdrm, int on);
extern void reg_write_lcd(struct glamodrm_handle *gdrm,
                          u_int16_t reg, u_int16_t val);
extern void reg_set_bit_mask_lcd(struct glamodrm_handle *gdrm,
                                 u_int16_t reg, u_int16_t mask,
                                 u_int16_t val);

/* This is not the right place to switch power on/off, because the helper
 * stuff ends up calling this before/after setting the mode.  We can't
 * set modes with the display off (although backlight off would be OK) */
static void glamo_crtc_dpms(struct drm_crtc *crtc, int mode)
{
	/* do nothing */
	DRM_DEBUG("mode = %s\n", ((char*[]) {"on", "standby", "suspend", "off"})[mode]);
	
	//glamo_lcd_power(to_glamo_crtc(crtc)->gdrm, mode);
}


static void glamo_crtc_prepare(struct drm_crtc *crtc)
{
	DRM_DEBUG("\n");
	glamo_lcd_power(to_glamo_crtc(crtc)->gdrm, DRM_MODE_DPMS_ON);
}


static void glamo_crtc_commit(struct drm_crtc *crtc)
{
	DRM_DEBUG("\n");
	glamo_lcd_power(to_glamo_crtc(crtc)->gdrm, DRM_MODE_DPMS_OFF);
}


static int glamo_crtc_cursor_set(struct drm_crtc *crtc,
                                 struct drm_file *file_priv,
                                 uint32_t handle,
                                 uint32_t width, uint32_t height)
{
	DRM_DEBUG("cursor = %u (%ux%u)\n", handle, width, height);
	return 0;
}


static int glamo_crtc_cursor_move(struct drm_crtc *crtc, int x, int y)
{
	DRM_DEBUG("( %d , %d )\n", x, y);
	return 0;
}


static void glamo_crtc_gamma_set(struct drm_crtc *crtc, u16 *r, u16 *g,
                                 u16 *b, uint32_t start, uint32_t size)
{
	DRM_DEBUG("[%u,%u,%u] @%u +%u\n", *r, *g, *b, start, size);
}

static bool glamo_crtc_mode_fixup(struct drm_crtc *crtc,
                                  struct drm_display_mode *mode,
                                  struct drm_display_mode *adjusted_mode)
{
	DRM_DEBUG("%p => %p\n", mode, adjusted_mode);
	return true;
}


static int glamo_crtc_mode_set_base(struct drm_crtc *crtc, int x, int y,
                                    struct drm_framebuffer *old_fb)
{
	struct glamodrm_handle *gdrm;
	struct glamo_crtc *gcrtc;
	struct glamo_fbdev *glamo_fbdev;
	struct drm_glamo_gem_object *gobj;
	u32 addr;
	u16 addr_low, addr_high;

	DRM_DEBUG("Setting base\n");

	if (!crtc->fb) {
		DRM_DEBUG("No FB bound\n");
		return -EINVAL;
	}

	/* Dig out our handle */
	gcrtc = to_glamo_crtc(crtc);
	gdrm = gcrtc->gdrm;	/* Here it is! */

	glamo_fbdev = to_glamo_fbdev(gcrtc->fb_helper);
	gobj = glamo_fbdev->obj->driver_private;

	addr = GLAMO_OFFSET_FB + gobj->block->start;
	addr_low = addr & 0xffff;
	addr_high = ((addr >> 16) & 0x7f) | 0x4000;

	glamo_lcd_cmd_mode(gdrm, 1);
	reg_write_lcd(gdrm, GLAMO_REG_LCD_A_BASE1, addr_low);
	reg_write_lcd(gdrm, GLAMO_REG_LCD_A_BASE2, addr_high);
	glamo_lcd_cmd_mode(gdrm, 0);

	return 0;
}


int glamo_crtc_mode_set(struct drm_crtc *crtc,
                               struct drm_display_mode *mode,
                               struct drm_display_mode *adjusted_mode,
                               int x, int y,
                               struct drm_framebuffer *old_fb)
{
	struct glamodrm_handle *gdrm;
	struct glamo_crtc *gcrtc;
	struct glamo_fb_platform_data *fb_info;
	int retr_start, retr_end, disp_start, disp_end;
	int rot;
	DRM_DEBUG("Setting mode\n");

	/* Dig out our handle */
	gcrtc = to_glamo_crtc(crtc);
	gdrm = gcrtc->gdrm;	/* Here it is! */
	
	/* Dig out the record which will tell us about the hardware */
	fb_info = gdrm->glamo_core->pdata->fb_data;

	/* Rotate? */
	if ( (mode->hdisplay == 640) && (mode->vdisplay == 480) ) {
		rot = GLAMO_LCD_ROT_MODE_90;
	} else if ( (mode->hdisplay == 480) && (mode->vdisplay == 640) ) {
		rot = GLAMO_LCD_ROT_MODE_0;
	} else if ( (mode->hdisplay == 320) && (mode->vdisplay == 240) ) {
		rot = GLAMO_LCD_ROT_MODE_90;
	} else if ( (mode->hdisplay == 240) && (mode->vdisplay == 320) ) {
		rot = GLAMO_LCD_ROT_MODE_0;
	} else {
		printk(KERN_WARNING "[glamo-drm] Couldn't choose rotation.\n");
		rot = GLAMO_LCD_ROT_MODE_0;
	}

	glamo_lcd_cmd_mode(gdrm, 1);

	/* Set dimensions */
	if ( rot == GLAMO_LCD_ROT_MODE_0 ) {

		glamo_engine_reclock(gdrm->glamo_core, GLAMO_ENGINE_LCD,
		                     mode->clock);

		reg_set_bit_mask_lcd(gdrm, GLAMO_REG_LCD_WIDTH,
			             GLAMO_LCD_WIDTH_MASK, mode->hdisplay);
		reg_set_bit_mask_lcd(gdrm, GLAMO_REG_LCD_HEIGHT,
			             GLAMO_LCD_HEIGHT_MASK, mode->vdisplay);
		reg_set_bit_mask_lcd(gdrm, GLAMO_REG_LCD_PITCH,
			             GLAMO_LCD_PITCH_MASK, mode->hdisplay*2);

		/* Set rotation */
		reg_set_bit_mask_lcd(gdrm, GLAMO_REG_LCD_WIDTH,
				     GLAMO_LCD_ROT_MODE_MASK, rot);
		reg_set_bit_mask_lcd(gdrm, GLAMO_REG_LCD_MODE1,
				     GLAMO_LCD_MODE1_ROTATE_EN,
				     (rot != GLAMO_LCD_ROT_MODE_0) ?
				       GLAMO_LCD_MODE1_ROTATE_EN : 0);

		/* Convert "X modeline timings" into "Glamo timings" */
		retr_start = 0;
		retr_end = retr_start + mode->hsync_end - mode->hsync_start;
		disp_start = mode->htotal - mode->hsync_start;
		disp_end = disp_start + mode->hdisplay;

		reg_set_bit_mask_lcd(gdrm, GLAMO_REG_LCD_HORIZ_TOTAL,
				     GLAMO_LCD_HV_TOTAL_MASK, mode->htotal);
		reg_set_bit_mask_lcd(gdrm, GLAMO_REG_LCD_HORIZ_RETR_START,
				     GLAMO_LCD_HV_RETR_START_MASK, retr_start);
		reg_set_bit_mask_lcd(gdrm, GLAMO_REG_LCD_HORIZ_RETR_END,
				     GLAMO_LCD_HV_RETR_END_MASK, retr_end);
		reg_set_bit_mask_lcd(gdrm, GLAMO_REG_LCD_HORIZ_DISP_START,
				     GLAMO_LCD_HV_RETR_DISP_START_MASK, disp_start);
		reg_set_bit_mask_lcd(gdrm, GLAMO_REG_LCD_HORIZ_DISP_END,
				     GLAMO_LCD_HV_RETR_DISP_END_MASK, disp_end);

		/* The same in the vertical direction */
		retr_start = 0;
		retr_end = retr_start + mode->vsync_end - mode->vsync_start;
		disp_start = mode->vtotal - mode->vsync_start;
		disp_end = disp_start + mode->vdisplay;
		reg_set_bit_mask_lcd(gdrm, GLAMO_REG_LCD_VERT_TOTAL,
				     GLAMO_LCD_HV_TOTAL_MASK, mode->vtotal);
		reg_set_bit_mask_lcd(gdrm, GLAMO_REG_LCD_VERT_RETR_START,
				     GLAMO_LCD_HV_RETR_START_MASK, retr_start);
		reg_set_bit_mask_lcd(gdrm, GLAMO_REG_LCD_VERT_RETR_END,
				     GLAMO_LCD_HV_RETR_END_MASK, retr_end);
		reg_set_bit_mask_lcd(gdrm, GLAMO_REG_LCD_VERT_DISP_START,
				     GLAMO_LCD_HV_RETR_DISP_START_MASK,
				     disp_start);
		reg_set_bit_mask_lcd(gdrm, GLAMO_REG_LCD_VERT_DISP_END,
				     GLAMO_LCD_HV_RETR_DISP_END_MASK, disp_end);

	} else {

		glamo_engine_reclock(gdrm->glamo_core, GLAMO_ENGINE_LCD,
		                     mode->clock/2);

		reg_set_bit_mask_lcd(gdrm, GLAMO_REG_LCD_WIDTH,
			             GLAMO_LCD_WIDTH_MASK, mode->vdisplay);
		reg_set_bit_mask_lcd(gdrm, GLAMO_REG_LCD_HEIGHT,
			             GLAMO_LCD_HEIGHT_MASK, mode->hdisplay);
		reg_set_bit_mask_lcd(gdrm, GLAMO_REG_LCD_PITCH,
			             GLAMO_LCD_PITCH_MASK, mode->hdisplay*2);

		/* Set rotation */
		reg_set_bit_mask_lcd(gdrm, GLAMO_REG_LCD_WIDTH,
				     GLAMO_LCD_ROT_MODE_MASK, rot);
		reg_set_bit_mask_lcd(gdrm, GLAMO_REG_LCD_MODE1,
				     GLAMO_LCD_MODE1_ROTATE_EN,
				     (rot != GLAMO_LCD_ROT_MODE_0) ?
				       GLAMO_LCD_MODE1_ROTATE_EN : 0);

		/* Apply "vertical" numbers to the horizontal registers */
		retr_start = 0;
		retr_end = retr_start + mode->vsync_end - mode->vsync_start;
		disp_start = mode->vtotal - mode->vsync_start;
		disp_end = disp_start + mode->vdisplay;

		reg_set_bit_mask_lcd(gdrm, GLAMO_REG_LCD_HORIZ_TOTAL,
				     GLAMO_LCD_HV_TOTAL_MASK, mode->vtotal);
		reg_set_bit_mask_lcd(gdrm, GLAMO_REG_LCD_HORIZ_RETR_START,
				     GLAMO_LCD_HV_RETR_START_MASK, retr_start);
		reg_set_bit_mask_lcd(gdrm, GLAMO_REG_LCD_HORIZ_RETR_END,
				     GLAMO_LCD_HV_RETR_END_MASK, retr_end);
		reg_set_bit_mask_lcd(gdrm, GLAMO_REG_LCD_HORIZ_DISP_START,
				     GLAMO_LCD_HV_RETR_DISP_START_MASK,
				     disp_start);
		reg_set_bit_mask_lcd(gdrm, GLAMO_REG_LCD_HORIZ_DISP_END,
				     GLAMO_LCD_HV_RETR_DISP_END_MASK, disp_end);

		/* Apply "horizontal" numbers to the vertical registers */
		retr_start = 0;
		retr_end = retr_start + mode->hsync_end - mode->hsync_start;
		disp_start = mode->htotal - mode->hsync_start;
		disp_end = disp_start + mode->hdisplay;
		reg_set_bit_mask_lcd(gdrm, GLAMO_REG_LCD_VERT_TOTAL,
				     GLAMO_LCD_HV_TOTAL_MASK, mode->htotal);
		reg_set_bit_mask_lcd(gdrm, GLAMO_REG_LCD_VERT_RETR_START,
				     GLAMO_LCD_HV_RETR_START_MASK, retr_start);
		reg_set_bit_mask_lcd(gdrm, GLAMO_REG_LCD_VERT_RETR_END,
				     GLAMO_LCD_HV_RETR_END_MASK, retr_end);
		reg_set_bit_mask_lcd(gdrm, GLAMO_REG_LCD_VERT_DISP_START,
				     GLAMO_LCD_HV_RETR_DISP_START_MASK,
				     disp_start);
		reg_set_bit_mask_lcd(gdrm, GLAMO_REG_LCD_VERT_DISP_END,
				     GLAMO_LCD_HV_RETR_DISP_END_MASK, disp_end);

	}

	gdrm->saved_clock = mode->clock;

	glamo_crtc_mode_set_base(crtc, 0, 0, old_fb);
	
	glamo_lcd_cmd_mode(gdrm, 0);

	/*if ( mode->hdisplay == 240 ) {
		jbt6k74_finish_resolutionchange(JBT_RESOLUTION_QVGA);
	} else {
		jbt6k74_finish_resolutionchange(JBT_RESOLUTION_VGA);
	}*/
	if (fb_info)
	    if (fb_info->mode_change)
        	fb_info->mode_change(mode);

	gcrtc->current_mode = *mode;
	gcrtc->current_mode_set = 1;
	gcrtc->current_fb = old_fb;
	gcrtc->mode_set.fb = old_fb;

	return 0;
}

static void glamo_crtc_destroy(struct drm_crtc *crtc)
{
	struct glamo_crtc *glamo_crtc = to_glamo_crtc(crtc);
	drm_crtc_cleanup(crtc);
	kfree(glamo_crtc);
}


/* CRTC functions */
static const struct drm_crtc_funcs glamo_crtc_funcs = {
	.cursor_set = glamo_crtc_cursor_set,
	.cursor_move = glamo_crtc_cursor_move,
	.gamma_set = glamo_crtc_gamma_set,
	.set_config = drm_crtc_helper_set_config,
	.destroy = glamo_crtc_destroy,
};


/* CRTC helper functions */
static const struct drm_crtc_helper_funcs glamo_crtc_helper_funcs = {
	.dpms = glamo_crtc_dpms,
	.mode_fixup = glamo_crtc_mode_fixup,
	.mode_set = glamo_crtc_mode_set,
	.mode_set_base = glamo_crtc_mode_set_base,
	.prepare = glamo_crtc_prepare,
	.commit = glamo_crtc_commit,
};

int glamo_crtc_resume(struct glamodrm_handle *gdrm)
{
	struct glamo_crtc *gcrtc = to_glamo_crtc(gdrm->crtc);

	if ( gcrtc->current_mode_set ) {
		glamo_crtc_mode_set(gdrm->crtc, &gcrtc->current_mode,
		                    &gcrtc->current_mode, 0, 0,
		                    gcrtc->current_fb);
	}
	return 0;
}

int glamo_crtc_init(struct drm_device *dev)
{
	struct glamo_crtc *glamo_crtc;
	struct glamodrm_handle *gdrm = dev->dev_private;
	
	DRM_DEBUG("\n");
	/* Initialise our CRTC object.
	 * Only one connector per CRTC.  We know this: it's kind of soldered. */
	glamo_crtc = kzalloc(sizeof(struct glamo_crtc)
	                   + sizeof(struct drm_connector *)*2, GFP_KERNEL);
	if (!glamo_crtc)
		return -ENOMEM;
	
	glamo_crtc->pixel_clock_on = 1;
	glamo_crtc->blank_mode = DRM_MODE_DPMS_OFF;
	
	drm_crtc_init(dev, &glamo_crtc->base, &glamo_crtc_funcs);
	drm_crtc_helper_add(&glamo_crtc->base, &glamo_crtc_helper_funcs);

	glamo_crtc->mode_set.crtc = &glamo_crtc->base;
	glamo_crtc->mode_set.connectors = glamo_crtc->connectors;
	glamo_crtc->mode_set.num_connectors = 0;
	glamo_crtc->mode_set.mode = &glamo_crtc->mode_set.crtc->mode;
	
	gdrm->crtc = &glamo_crtc->base;
	glamo_crtc->gdrm = gdrm;
	
	return 0;
}
