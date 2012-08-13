/*
 * SMedia Glamo 336x/337x KMS Framebuffer
 *
 * Copyright (C) 2009 Thomas White <taw@bitwiz.org.uk>
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
 */
 
#include <linux/fb.h>
 
#include <drm/drmP.h>
#include <drm/drm_fb_helper.h>
#include <drm/glamo_drm.h>

#include "glamo-driver.h"
#include "glamo-buffer.h"

 
static int glamo_fb_setcolreg(unsigned regno, unsigned red, unsigned green,
			unsigned blue, unsigned transp,
			struct fb_info *info)
{
	struct drm_fb_helper *fb_helper = info->par;

	if (!fb_helper->fb) {
		DRM_DEBUG("modeset: fb = %p | %p, [%u|%u]\n",
	          fb_helper->fb, fb_helper->saved_fb, 
	          fb_helper->crtc_count, fb_helper->connector_count);
	    return -ENODEV;
	}

	if (regno > 255)
		return 1;

	if (regno < 16) {
		switch (fb_helper->fb->depth) {
		case 15:
			fb_helper->pseudo_palette[regno] = ((red & 0xf800) >> 1) |
				((green & 0xf800) >>  6) |
				((blue & 0xf800) >> 11);
			break;
		case 16:
			fb_helper->pseudo_palette[regno] = (red & 0xf800) |
				((green & 0xfc00) >>  5) |
				((blue  & 0xf800) >> 11);
			break;
		case 24:
		case 32:
			fb_helper->pseudo_palette[regno] = ((red & 0xff00) << 8) |
				(green & 0xff00) |
				((blue  & 0xff00) >> 8);
			break;
		}
	}
	
	return 0;
}

static int glamo_fb_check_var(struct fb_var_screeninfo *var,
			struct fb_info *info)
{
	struct drm_fb_helper *fb_helper = info->par;
	struct drm_framebuffer *fb = fb_helper->fb;
	int depth;

	DRM_DEBUG("\n");
	/* Need to resize the fb object !!! */
	if (var->xres > fb->width || var->yres > fb->height) {
		DRM_ERROR("Cannot resize framebuffer object (%dx%d > %dx%d)\n",
		          var->xres,var->yres,fb->width,fb->height);
		DRM_ERROR("Need resizing code.\n");
		return -EINVAL;
	}

	switch (var->bits_per_pixel) {
	case 16:
		depth = (var->green.length == 6) ? 16 : 15;
		break;
	case 32:
		depth = (var->transp.length > 0) ? 32 : 24;
		break;
	default:
		depth = var->bits_per_pixel;
		break;
	}

	switch (depth) {
	case 16:
		var->red.offset = 11;
		var->green.offset = 5;
		var->blue.offset = 0;
		var->red.length = 5;
		var->green.length = 6;
		var->blue.length = 5;
		var->transp.length = 0;
		var->transp.offset = 0;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/* this will let fbcon do the mode init */
/* FIXME: take mode config lock? */
/* this will let fbcon do the mode init */
static int glamo_fb_helper_set_par(struct fb_info *info)
{
	struct drm_fb_helper *fb_helper = info->par;
	struct drm_device *dev = fb_helper->dev;
	struct drm_crtc *crtc;
	struct drm_mode_set *modeset;
	int ret;
	int i;

	DRM_DEBUG("\n");
	if (info->var.pixclock != 0) {
		DRM_ERROR("PIXEL CLOCK SET\n");
		return -EINVAL;
	}

	mutex_lock(&dev->mode_config.mutex);
	for (i = 0; i < fb_helper->crtc_count; i++) {
	
		modeset = &fb_helper->crtc_info[i].mode_set;
		crtc = modeset->crtc;
		/*DRM_DEBUG("modeset: fb = %p, crtc = %p, mode = %p, <%u,%u>, %p, [%u], dpms = %p\n",
		          modeset->fb, modeset->crtc, modeset->mode,
		          modeset->x, modeset->y, modeset->connectors, modeset->num_connectors,
		          modeset->connectors[0]->funcs->dpms);*/
		
		crtc->fb = modeset->fb;
		ret = crtc->funcs->set_config(&fb_helper->crtc_info[i].mode_set);
		if (ret) {
			mutex_unlock(&dev->mode_config.mutex);
			return ret;
		}
	}
	mutex_unlock(&dev->mode_config.mutex);

	if (fb_helper->delayed_hotplug) {
		fb_helper->delayed_hotplug = false;
		drm_fb_helper_hotplug_event(fb_helper);
	}
	return 0;
}

static int glamo_fb_pan_display(struct fb_var_screeninfo *var,
				struct fb_info *info)
{
	struct drm_fb_helper *fb_helper = info->par;
	struct drm_device *dev = fb_helper->dev;
	struct drm_mode_set *modeset;
	struct drm_crtc *crtc;
	int ret = 0;
	int i;

	mutex_lock(&dev->mode_config.mutex);
	for (i = 0; i < fb_helper->crtc_count; i++) {
		crtc = fb_helper->crtc_info[i].mode_set.crtc;

		modeset = &fb_helper->crtc_info[i].mode_set;

		modeset->x = var->xoffset;
		modeset->y = var->yoffset;

		if (modeset->num_connectors) {
			ret = crtc->funcs->set_config(modeset);
			if (!ret) {
				info->var.xoffset = var->xoffset;
				info->var.yoffset = var->yoffset;
			}
		}
	}
	mutex_unlock(&dev->mode_config.mutex);

	return ret;
}

static int glamo_fb_blank(int blank, struct fb_info *info)
{
	struct drm_fb_helper *fb_helper = info->par;
	struct drm_device *dev = fb_helper->dev;
	struct glamodrm_handle *gdrm = dev->dev_private;
	DRM_DEBUG("blank = %d\n", blank);
	
	switch (blank) {
	case FB_BLANK_UNBLANK:
		glamo_lcd_power(gdrm, DRM_MODE_DPMS_ON);
		break;
	case FB_BLANK_NORMAL:
		glamo_lcd_power(gdrm, DRM_MODE_DPMS_STANDBY);
		break;
	case FB_BLANK_HSYNC_SUSPEND:
		glamo_lcd_power(gdrm, DRM_MODE_DPMS_SUSPEND);
		break;
	case FB_BLANK_VSYNC_SUSPEND:
		glamo_lcd_power(gdrm, DRM_MODE_DPMS_SUSPEND);
		break;
	case FB_BLANK_POWERDOWN:
		glamo_lcd_power(gdrm, DRM_MODE_DPMS_OFF);
		break;
	}
	return 0;
}

static struct fb_ops glamo_fb_ops = {
	.owner = THIS_MODULE,
	.fb_check_var = glamo_fb_check_var,
	.fb_set_par = glamo_fb_helper_set_par,
	.fb_setcolreg = glamo_fb_setcolreg,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
	.fb_pan_display = glamo_fb_pan_display,
	.fb_blank = glamo_fb_blank,
};

/* called with drm_fb_helper->struct_mutex held */
struct fb_info * glamo_fb_init(struct drm_fb_helper *fb_helper, struct drm_fb_helper_surface_size *sizes, unsigned int start, unsigned int size)
{
	struct fb_info *info;
	
	info = framebuffer_alloc(0, fb_helper->dev->dev);
	if (!info) {
		DRM_ERROR("FAILED to allocate framebuffer\n");
		return ERR_PTR(-ENOMEM);
	}
    
    info->par = fb_helper;
	info->pseudo_palette = fb_helper->pseudo_palette;
    info->flags = FBINFO_DEFAULT;
    info->fbops = &glamo_fb_ops;
        
    strcpy(info->fix.id, "glamodrmfb"); // MODULE_NAME
    
    drm_fb_helper_fill_fix(info, fb_helper->fb->pitches[0],
                                 fb_helper->fb->depth);
    drm_fb_helper_fill_var(info, fb_helper, sizes->fb_width,
                                 sizes->fb_height);
    
    info->fix.accel = FB_ACCEL_GLAMO;
    
	info->pixmap.size = 64*1024;
	info->pixmap.buf_align = 8;
	info->pixmap.access_align = 32;
	info->pixmap.flags = FB_PIXMAP_SYSTEM;
	info->pixmap.scan_align = 1;
    
	info->screen_base = ioremap(start + GLAMO_OFFSET_FB,
	                            GLAMO_FRAMEBUFFER_ALLOCATION);
	if (!info->screen_base) {
		DRM_ERROR("Couldn't map framebuffer!\n");
		goto free;
	}
	info->screen_size = size;
	info->fix.smem_start = (unsigned long)start;
	info->fix.smem_len = size;
	
	return info;

free:
	framebuffer_release(info);
	return ERR_PTR(-ENOSPC);
}

