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
extern struct fb_info * glamo_fb_init(struct drm_fb_helper *fb_helper, struct drm_fb_helper_surface_size *sizes, unsigned int start, unsigned int size);

#define RESSIZE(ressource) (((ressource)->end - (ressource)->start)+1)

static int glamo_fbdev_probe(struct drm_fb_helper *fb_helper,
							 struct drm_fb_helper_surface_size *sizes)
{
	struct glamodrm_handle *gdrm = fb_helper->dev->dev_private;
	struct glamo_fbdev * glamo_fbdev = container_of(fb_helper, struct glamo_fbdev, base);
	//struct glamofb_par *par;
	struct drm_framebuffer *fb;
	struct drm_mode_fb_cmd2 mode_cmd = {0};
	struct drm_glamo_gem_object *gobj;
	struct fb_info *info = NULL;
	
	u32 cols_g;
	int size, ret;
	
	DRM_DEBUG("%dx%d@%d (%dx%d)", sizes->surface_width,
	         sizes->surface_height, sizes->surface_bpp, sizes->fb_width, sizes->fb_height);
	
	if (fb_helper->fb)
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
	glamo_fbdev->obj = glamo_gem_object_alloc(fb_helper->dev, GLAMO_FRAMEBUFFER_ALLOCATION, 2);
	if (!glamo_fbdev->obj) {
		DRM_ERROR("Failed to allocate framebuffer gem object\n");
		ret = -ENOMEM;
		goto out;
	}
	gobj = glamo_fbdev->obj->driver_private;
	
	fb = glamo_framebuffer_create(fb_helper->dev, &mode_cmd, glamo_fbdev->obj);
	if (IS_ERR(fb)) {
		ret = PTR_ERR(fb);
		DRM_ERROR("failed to allocate fb (%d).\n", ret);
		goto out_unref;
	}
    fb_helper->fb = fb;
    
    mutex_lock(&fb_helper->dev->struct_mutex);
	info = glamo_fb_init(fb_helper, sizes, 
	                     gdrm->vram->start + gobj->block->start, size);
	if (IS_ERR(info)) {
		ret = PTR_ERR(info);
		goto out_unlock;
	}
	fb_helper->fbdev = info;
    mutex_unlock(&fb_helper->dev->struct_mutex);
    
	DRM_DEBUG("Allocated %dx%d fb: bo %p\n",
	       fb->width, fb->height, glamo_fbdev->obj);
    
    return 1;

out_unlock:
	mutex_unlock(&fb_helper->dev->struct_mutex);
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
	struct glamo_fbdev * glamo_fbdev;
	struct glamodrm_handle *gdrm = dev->dev_private;
	
	DRM_DEBUG("\n");
	
	glamo_fbdev = kzalloc(sizeof(*glamo_fbdev), GFP_KERNEL);
	if (!glamo_fbdev)
		return -ENOMEM;
	
	gdrm->fb_helper = &glamo_fbdev->base;

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
	struct fb_info *fb_info = gdrm->fb_helper->fbdev;
	struct drm_framebuffer *fb = gdrm->fb_helper->fb;
	struct glamo_fbdev *fbdev = to_glamo_fbdev(gdrm->fb_helper);
	
	if (fb_info) {
		unregister_framebuffer(fb_info);
		framebuffer_release(fb_info);
	}
	
	drm_fb_helper_fini(gdrm->fb_helper);
	if (fb)
		fb->funcs->destroy(fb);
	
	drm_gem_object_unreference_unlocked(fbdev->obj);
	kfree(fbdev);
	
	gdrm->fb_helper = NULL;
}
