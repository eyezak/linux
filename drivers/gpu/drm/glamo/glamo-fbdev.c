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


static int glamo_fbinfo_setcolreg(unsigned regno, unsigned red, unsigned green,
			unsigned blue, unsigned transp,
			struct fb_info *info)
{
	struct drm_fb_helper *helper = info->par;
	struct drm_device *dev = helper->dev;
	struct drm_crtc *crtc;

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

		/*for (i = 0; i < par->crtc_count; i++)
			if (crtc->base.id == par->crtc_ids[i])
				break;

		if (i == par->crtc_count)
			continue;*/


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

static int glamo_fbinfo_check_var(struct fb_var_screeninfo *var,
			struct fb_info *info)
{
	struct drm_fb_helper *helper = info->par;
	struct drm_framebuffer *fb = helper->fb;
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
static int glamo_fbinfo_helper_set_par(struct fb_info *info)
{
	struct drm_fb_helper *helper = info->par;
	struct drm_device *dev = helper->dev;
	struct drm_fb_helper *fb_helper = helper;
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

static int glamo_fbinfo_pan_display(struct fb_var_screeninfo *var,
				struct fb_info *info)
{
	struct drm_fb_helper *fb_helper = info->par;
	struct drm_device *dev = fb_helper->dev;
	struct drm_mode_set *modeset;
	struct drm_crtc *crtc;
	//struct glamo_crtc *glamo_crtc;
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

/*	list_for_each_entry(crtc, &dev->mode_config.crtc_list, head) {
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
	}*/

	return ret;
}

static int glamo_fbinfo_blank(int blank, struct fb_info *info)
{
	struct drm_fb_helper *helper = info->par;
	struct drm_device *dev = helper->dev;
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

static struct fb_ops glamofb_ops = {
	.owner = THIS_MODULE,
	.fb_check_var = glamo_fbinfo_check_var,
	.fb_set_par = glamo_fbinfo_helper_set_par,
	.fb_setcolreg = glamo_fbinfo_setcolreg,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
	.fb_pan_display = glamo_fbinfo_pan_display,
	.fb_blank = glamo_fbinfo_blank,
};


#define RESSIZE(ressource) (((ressource)->end - (ressource)->start)+1)

static int glamo_fbdev_probe(struct drm_fb_helper *helper,
							 struct drm_fb_helper_surface_size *sizes)
{
	struct glamodrm_handle *gdrm = helper->dev->dev_private;
	struct glamo_fbdev * glamo_fbdev = container_of(helper, struct glamo_fbdev, base);
	//struct glamofb_par *par;
	struct drm_framebuffer *fb;
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
            DRM_ERROR("Unrecognised LCD colour mode\n");
            mode_cmd.pixel_format = DRM_FORMAT_RGB565; /* Take a guess */
            break;
    }
    
	mode_cmd.width = sizes->surface_width;
	mode_cmd.height = sizes->surface_height;

	mode_cmd.pitches[0] = ALIGN(mode_cmd.width * ((16 + 1) / 8), 64);

	size = mode_cmd.pitches[0] * mode_cmd.height;
	size = ALIGN(size, PAGE_SIZE);
	if ( size > GLAMO_FRAMEBUFFER_ALLOCATION ) {
		DRM_ERROR("Not enough memory for fb\n");
		ret = -ENOMEM;
		goto out;
	}
    
    // fb->obj = allocate here ??
	glamo_fbdev->obj = glamo_gem_object_alloc(helper->dev, GLAMO_FRAMEBUFFER_ALLOCATION, 2);
	if (!glamo_fbdev->obj) {
		DRM_ERROR("Failed to allocate framebuffer gem object\n");
		ret = -ENOMEM;
		goto out;
	}
	gobj = glamo_fbdev->obj->driver_private;
	
	fb = glamo_framebuffer_create(helper->dev, &mode_cmd, glamo_fbdev->obj);
	if (IS_ERR(fb)) {
		ret = PTR_ERR(fb);
		DRM_ERROR("failed to allocate fb (%d).\n", ret);
		goto out_unref;
	}
    
    mutex_lock(&helper->dev->struct_mutex);
	info = framebuffer_alloc(0, helper->dev->dev);
	if (!info) {
		ret = -ENOMEM;
		DRM_ERROR("FAILED to allocate framebuffer\n");
		goto out_unlock;
	}
	DRM_DEBUG("fb=%p, dev=%p\n", fb, helper->dev);
	
    helper->fb = fb;
    helper->fbdev = info;
    
    info->par = helper;
	info->pseudo_palette = helper->pseudo_palette;
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
		DRM_ERROR("Couldn't map framebuffer!\n");
		ret = -ENOSPC;
		goto out_unlock;
	}
	info->screen_size = size;
	info->fix.smem_start = (unsigned long)gdrm->vram->start + gobj->block->start;
	info->fix.smem_len = size;
	
    mutex_unlock(&helper->dev->struct_mutex);
	DRM_DEBUG("Allocated %dx%d fb: bo %p\n",
	       fb->width, fb->height, glamo_fbdev->obj);
    
    return 1;

out_unlock:
	mutex_unlock(&helper->dev->struct_mutex);
out_unref:
	//drm_gem_object_unreference(glamo_fbdev->obj);
out:	
	return ret;
}

static void glamo_fbdev_gamma_set(struct drm_crtc *crtc, u16 r, u16 g, u16 b, int regno)
{
	printk("[glamo-drm] fbdev: set gamma\n");
}

static void glamo_fbdev_gamma_get(struct drm_crtc *crtc, u16 *r, u16 *g, u16 *b, int regno)
{
	printk("[glamo-drm] fbdev: get gamma\n");
}

static struct drm_fb_helper_funcs glamo_fb_helper_funcs = {
	.gamma_set = glamo_fbdev_gamma_set,
	.gamma_get = glamo_fbdev_gamma_get,
	.fb_probe = glamo_fbdev_probe,
};


int glamo_fbdev_init(struct drm_device *dev)
{
	int ret;
	struct glamo_crtc *glamo_crtc;
	struct glamo_fbdev * glamo_fbdev;
	//struct glamofb_par *par;
	struct glamodrm_handle *gdrm = dev->dev_private;
	
	DRM_DEBUG("\n");
	
	glamo_fbdev = kzalloc(sizeof(*glamo_fbdev), GFP_KERNEL);
	if (!glamo_fbdev)
		return -ENOMEM;
	//gdrm->gfb = glamo_fb;
	
	glamo_crtc = to_glamo_crtc(gdrm->crtc);
	glamo_crtc->fb_helper = &glamo_fbdev->base;
	
	//par = glamo_fb->base.fbdev->par;
	//par->crtc_ids[0] = glamo_crtc->base.base.id;
	//par->crtc_count = 1;
	
	glamo_fbdev->base.funcs = &glamo_fb_helper_funcs;

	ret = drm_fb_helper_init(dev, &glamo_fbdev->base, 1, 1);
	if (ret)
		return ret;
	
	drm_fb_helper_single_add_all_connectors(&glamo_fbdev->base);
	drm_fb_helper_initial_config(&glamo_fbdev->base, 16);
	
	return 0;
}

void glamo_fbdev_free(struct drm_device *dev)
{
	struct glamodrm_handle *gdrm = dev->dev_private;
	struct glamo_crtc *crtc = to_glamo_crtc(gdrm->crtc);
	struct fb_info *fb_info = crtc->fb_helper->fbdev;
	struct drm_framebuffer *fb = crtc->fb_helper->fb;
	struct glamo_fbdev *fbdev = to_glamo_fbdev(crtc->fb_helper);
	
	if (fb_info) {
		unregister_framebuffer(fb_info);
		framebuffer_release(fb_info);
	}
	
	drm_fb_helper_fini(crtc->fb_helper);
	if (fb)
		fb->funcs->destroy(fb);
	
	drm_gem_object_unreference_unlocked(fbdev->obj);
	kfree(fbdev);
	
	crtc->fb_helper = NULL;
}
