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


#include <drm/drmP.h>
#include <drm/glamo_drm.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_crtc.h>

#include <linux/mfd/glamo-core.h>
#include <linux/glamofb.h>
#include "glamo-drm-private.h"
#include "glamo-display.h"
#include "glamo-buffer.h"
#include "glamo-regs.h"

extern int reg_read_lcd(struct glamodrm_handle *gdrm, u_int16_t reg);


struct glamofb_par {
	struct drm_device *dev;
	struct drm_display_mode *our_mode;
	struct glamo_framebuffer *glamo_fb;
	int crtc_count;
	/* crtc currently bound to this */
	uint32_t crtc_ids[2];
};


static int glamofb_setcolreg(unsigned regno, unsigned red, unsigned green,
			unsigned blue, unsigned transp,
			struct fb_info *info)
{
	struct glamofb_par *par = info->par;
	struct drm_device *dev = par->dev;
	struct drm_crtc *crtc;
	int i;

	list_for_each_entry(crtc, &dev->mode_config.crtc_list, head) {
		struct glamo_crtc *glamo_crtc = to_glamo_crtc(crtc);
		struct drm_mode_set *modeset = &glamo_crtc->mode_set;
		struct drm_framebuffer *fb = glamo_crtc->fb_helper->fb; //modeset->fb;
		
		if (!fb) {
			DRM_DEBUG("modeset: fb = %p | %p, crtc = %p, mode = %p, <%u,%u>, %p, [%u]\n",
		          modeset->fb, crtc->fb, modeset->crtc, modeset->mode,
		          modeset->x, modeset->y, modeset->connectors, modeset->num_connectors);
		    return -ENODEV;
		}

		for (i = 0; i < par->crtc_count; i++)
			if (crtc->base.id == par->crtc_ids[i])
				break;

		if (i == par->crtc_count)
			continue;


		if (regno > 255)
			return 1;

		if (regno < 16) {
			switch (fb->depth) {
			case 15:
				glamo_crtc->fb_helper->pseudo_palette[regno] = ((red & 0xf800) >> 1) |
					((green & 0xf800) >>  6) |
					((blue & 0xf800) >> 11);
				break;
			case 16:
				glamo_crtc->fb_helper->pseudo_palette[regno] = (red & 0xf800) |
					((green & 0xfc00) >>  5) |
					((blue  & 0xf800) >> 11);
				break;
			case 24:
			case 32:
				glamo_crtc->fb_helper->pseudo_palette[regno] = ((red & 0xff00) << 8) |
					(green & 0xff00) |
					((blue  & 0xff00) >> 8);
				break;
			}
		}
	}
	return 0;
}

static int glamofb_check_var(struct fb_var_screeninfo *var,
			struct fb_info *info)
{
	struct glamofb_par *par = info->par;
	struct glamo_framebuffer *glamo_fb = par->glamo_fb;
	struct drm_framebuffer *fb = &glamo_fb->fb;
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
static int glamofb_set_par(struct fb_info *info)
{
	struct glamofb_par *par = info->par;
	struct drm_device *dev = par->dev;
	struct fb_var_screeninfo *var = &info->var;
	struct drm_fb_helper *fb_helper = &par->glamo_fb->base;
	struct drm_crtc *crtc;
	int ret, i, found = 0;

	DRM_DEBUG("%d %d\n", var->xres, var->pixclock);

	if (var->pixclock != 0) {
		DRM_ERROR("Warning: userspace gave me a pixel clock value (%i)"
		          "- I'm ignoring it.\n", var->pixclock);
	}

	list_for_each_entry(crtc, &dev->mode_config.crtc_list, head) {
		struct glamo_crtc *glamo_crtc = to_glamo_crtc(crtc);

		for (i = 0; i < par->crtc_count; i++) {
/*			if (crtc->base.id == par->crtc_ids[i])
				break;

		if (i == par->crtc_count)
			continue;*/

		if (crtc->fb == glamo_crtc->mode_set.fb) {
			mutex_lock(&dev->mode_config.mutex);
			crtc->fb = fb_helper->fb;
			ret = crtc->funcs->set_config(&glamo_crtc->mode_set);
			mutex_unlock(&dev->mode_config.mutex);
			if (ret)
				return ret;
			found = 1;
		}
		}
	}

	if (!found)	
		return 0;

	DRM_DEBUG("fb found\n");
	if (fb_helper->delayed_hotplug) {
		fb_helper->delayed_hotplug = false;
		drm_fb_helper_hotplug_event(fb_helper);
	}
	return 0;
}

/* this will let fbcon do the mode init */
int glamofb_helper_set_par(struct fb_info *info)
{
	struct glamofb_par *par = info->par;
	struct drm_device *dev = par->dev;
	struct drm_fb_helper *fb_helper = &par->glamo_fb->base;
	struct drm_crtc *crtc;
	struct drm_mode_set *modeset;
	int ret;
	int i;

	DRM_DEBUG("%d: %p %p\n", info->var.pixclock, dev, &dev->mode_config);
	if (info->var.pixclock != 0) {
		DRM_ERROR("PIXEL CLOCK SET\n");
		return -EINVAL;
	}

	mutex_lock(&dev->mode_config.mutex);
	for (i = 0; i < fb_helper->crtc_count; i++) {
	
		modeset = &fb_helper->crtc_info[i].mode_set;
		crtc = modeset->crtc;
		DRM_DEBUG("modeset: fb = %p, crtc = %p, mode = %p, <%u,%u>, %p, [%u], dpms = %p\n",
		          modeset->fb, modeset->crtc, modeset->mode,
		          modeset->x, modeset->y, modeset->connectors, modeset->num_connectors,
		          modeset->connectors[0]->funcs->dpms);
		
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

static int glamofb_pan_display(struct fb_var_screeninfo *var,
				struct fb_info *info)
{
	struct glamofb_par *par = info->par;
	struct drm_device *dev = par->dev;
	struct drm_mode_set *modeset;
	struct drm_crtc *crtc;
	struct glamo_crtc *glamo_crtc;
	int ret = 0;
	int i;

	list_for_each_entry(crtc, &dev->mode_config.crtc_list, head) {
		for (i = 0; i < par->crtc_count; i++)
			if (crtc->base.id == par->crtc_ids[i])
				break;

		if (i == par->crtc_count)
			continue;

		glamo_crtc = to_glamo_crtc(crtc);
		modeset = &glamo_crtc->mode_set;

		modeset->x = var->xoffset;
		modeset->y = var->yoffset;

		if (modeset->num_connectors) {
			mutex_lock(&dev->mode_config.mutex);
			ret = crtc->funcs->set_config(modeset);
			mutex_unlock(&dev->mode_config.mutex);
			if (!ret) {
				info->var.xoffset = var->xoffset;
				info->var.yoffset = var->yoffset;
			}
		}
	}

	return ret;
}

static void glamofb_on(struct fb_info *info)
{
	struct glamofb_par *par = info->par;
	struct drm_device *dev = par->dev;
	struct glamodrm_handle *gdrm = dev->dev_private;

	glamo_lcd_power(gdrm, 1);
}

static void glamofb_off(struct fb_info *info, int dpms_mode)
{
	struct glamofb_par *par = info->par;
	struct drm_device *dev = par->dev;
	struct glamodrm_handle *gdrm = dev->dev_private;

	glamo_lcd_power(gdrm, 0);
}

static int glamofb_blank(int blank, struct fb_info *info)
{
	switch (blank) {
	case FB_BLANK_UNBLANK:
		glamofb_on(info);
		break;
	case FB_BLANK_NORMAL:
		glamofb_off(info, DRM_MODE_DPMS_STANDBY);
		break;
	case FB_BLANK_HSYNC_SUSPEND:
		glamofb_off(info, DRM_MODE_DPMS_STANDBY);
		break;
	case FB_BLANK_VSYNC_SUSPEND:
		glamofb_off(info, DRM_MODE_DPMS_SUSPEND);
		break;
	case FB_BLANK_POWERDOWN:
		glamofb_off(info, DRM_MODE_DPMS_OFF);
		break;
	}
	return 0;
}

static struct fb_ops glamofb_ops = {
	.owner = THIS_MODULE,
	.fb_check_var = glamofb_check_var,
	.fb_set_par = glamofb_helper_set_par, // drm_fb_helper_set_par,
	.fb_setcolreg = glamofb_setcolreg,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
	.fb_pan_display = glamofb_pan_display,
	.fb_blank = glamofb_blank,
};


#define RESSIZE(ressource) (((ressource)->end - (ressource)->start)+1)

void glamo_kmsfb_suspend(struct glamodrm_handle *gdrm)
{
	fb_set_suspend(gdrm->fb, 1);
}


void glamo_kmsfb_resume(struct glamodrm_handle *gdrm)
{
	fb_set_suspend(gdrm->fb, 0);
}

static int glamo_fbdev_probe(struct drm_fb_helper *helper,
							 struct drm_fb_helper_surface_size *sizes)
{
	struct glamodrm_handle *gdrm = helper->dev->dev_private;
	struct glamo_framebuffer * gfb = container_of(helper, struct glamo_framebuffer, base);
	struct glamofb_par *par;
	
	struct drm_framebuffer *fb = &gdrm->gfb->fb;
	struct drm_mode_fb_cmd2 mode_cmd = {0};
	struct drm_glamo_gem_object *gobj;
	struct fb_info *info = NULL;
	
	u32 cols_g;
	int size, ret;
	
	DRM_DEBUG("%dx%d@%d (%dx%d)", sizes->surface_width,
	         sizes->surface_height, sizes->surface_bpp, sizes->fb_width, sizes->fb_height);
	
	if (helper->fb)
		return 0;
	
	//	TODO: implement glamo_lcd_get_format and use it here
    cols_g = reg_read_lcd(gdrm, GLAMO_REG_LCD_MODE3) & 0xc000;
    switch ( cols_g ) {
    case GLAMO_LCD_SRC_RGB565 :
            mode_cmd.pixel_format = DRM_FORMAT_RGB565;
            break;
    case GLAMO_LCD_SRC_ARGB1555 :
            mode_cmd.pixel_format = DRM_FORMAT_ARGB1555;
            break;
    case GLAMO_LCD_SRC_ARGB4444 :
            mode_cmd.pixel_format = DRM_FORMAT_ARGB4444;
            break;
    default :
            printk(KERN_WARNING "Unrecognised LCD colour mode\n");
            mode_cmd.pixel_format = DRM_FORMAT_RGB565; /* Take a guess */
            break;
    }
    
	mode_cmd.width = sizes->surface_width;
	mode_cmd.height = sizes->surface_height;

	mode_cmd.pitches[0] = ALIGN(mode_cmd.width * ((16 + 1) / 8), 64);

	size = mode_cmd.pitches[0] * mode_cmd.height;
	size = ALIGN(size, PAGE_SIZE);
	if ( size > GLAMO_FRAMEBUFFER_ALLOCATION ) {
		printk(KERN_ERR "[glamo-drm] Not enough memory for fb\n");
		ret = -ENOMEM;
		goto out;
	}
    
    // fb->obj = allocate here ??
	gfb->obj = glamo_gem_object_alloc(helper->dev, GLAMO_FRAMEBUFFER_ALLOCATION, 2);
	if (!gfb->obj) {
		printk(KERN_ERR "[glamo-drm] Failed to allocate framebuffer\n");
		ret = -ENOMEM;
		goto out;
	}
	gobj = gfb->obj->driver_private;
	
	ret = glamo_framebuffer_create(helper->dev, &mode_cmd, &fb, gfb->obj);
	if (ret) {
		DRM_ERROR("failed to allocate fb.\n");
		goto out_unref;
	}
    
    mutex_lock(&helper->dev->struct_mutex);
	info = framebuffer_alloc(sizeof(struct glamofb_par), helper->dev->dev);
	if (!info) {
		ret = -ENOMEM;
		goto out_unlock;
	}
    par = info->par;
	par->crtc_ids[0] = to_glamo_crtc(gdrm->crtc)->base.base.id;
	par->crtc_count = 1;
	par->dev = helper->dev;
	par->glamo_fb = gfb;

	
    helper->fb = fb;
    helper->fbdev = info;
    gdrm->crtc->fb = fb;
    info->pseudo_palette = helper->pseudo_palette;
    
    //info->par = helper;
    info->flags = FBINFO_DEFAULT;
    info->fbops = &glamofb_ops;
        
    strcpy(info->fix.id, "glamodrmfb"); // MODULE_NAME
    
    drm_fb_helper_fill_fix(info, fb->pitches[0], fb->depth);
    drm_fb_helper_fill_var(info, helper, sizes->fb_width, sizes->fb_height);
    
    info->fix.accel = FB_ACCEL_GLAMO;
    
	info->pixmap.size = 64*1024;
	info->pixmap.buf_align = 8;
	info->pixmap.access_align = 32;
	info->pixmap.flags = FB_PIXMAP_SYSTEM;
	info->pixmap.scan_align = 1;
    
    
	info->screen_base = ioremap(gdrm->vram->start + gobj->block->start + GLAMO_OFFSET_FB,
	                            GLAMO_FRAMEBUFFER_ALLOCATION);
	if (!info->screen_base) {
		printk(KERN_ERR "[glamo-drm] Couldn't map framebuffer!\n");
		ret = -ENOSPC;
		goto out_unlock;
	}
	info->fix.smem_start = (unsigned long)gdrm->vram->start + gobj->block->start;
	info->fix.smem_len = size;
	info->screen_size = size;
    
    mutex_unlock(&helper->dev->struct_mutex);
	printk(KERN_INFO "[glamo-drm] Allocated %dx%d fb: bo %p\n",
	       fb->width, fb->height, gfb->obj);
    
    return 1;

out_unlock:
	mutex_unlock(&helper->dev->struct_mutex);
out_unref:
	//drm_gem_object_unreference(gfb->obj);
out:	
	return ret;
}

static void glamo_fb_gamma_set(struct drm_crtc *crtc, u16 r, u16 g, u16 b, int regno)
{
	printk("[glamo-drm] fbdev: set gamma\n");
}

static void glamo_fb_gamma_get(struct drm_crtc *crtc, u16 *r, u16 *g, u16 *b, int regno)
{
	printk("[glamo-drm] fbdev: get gamma\n");
}

static struct drm_fb_helper_funcs glamo_fb_helper_funcs = {
	.gamma_set = glamo_fb_gamma_set,
	.gamma_get = glamo_fb_gamma_get,
	.fb_probe = glamo_fbdev_probe,
};


int glamo_fbdev_init(struct drm_device *dev)
{
	int ret;
	struct glamo_crtc *glamo_crtc;
	struct glamo_framebuffer * glamo_fb;
	//struct glamofb_par *par;
	struct glamodrm_handle *gdrm = dev->dev_private;
	
	DRM_DEBUG("\n");
	
	glamo_fb = kzalloc(sizeof(*glamo_fb), GFP_KERNEL);
	if (!glamo_fb)
		return -ENOMEM;
	gdrm->gfb = glamo_fb;
	
	glamo_crtc = to_glamo_crtc(gdrm->crtc);
	glamo_crtc->fb_helper = &glamo_fb->base;
	
	
	//par = glamo_fb->base.fbdev->par;
	//par->crtc_ids[0] = glamo_crtc->base.base.id;
	//par->crtc_count = 1;
	
	glamo_fb->base.funcs = &glamo_fb_helper_funcs;

	ret = drm_fb_helper_init(dev, &glamo_fb->base, 1, 1);
	if (ret)
		return ret;
	
	drm_fb_helper_single_add_all_connectors(&glamo_fb->base);
	drm_fb_helper_initial_config(&glamo_fb->base, 16);

	glamo_crtc->mode_set.fb = &glamo_fb->fb;
	
	return 0;
}
