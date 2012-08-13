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
#include "glamo-driver.h"
#include "glamo-buffer.h"

static void glamo_framebuffer_destroy(struct drm_framebuffer *fb)
{
	DRM_DEBUG("\n");
	
	drm_framebuffer_cleanup(fb);

	kfree(fb);
}

static int glamo_framebuffer_create_handle(struct drm_framebuffer *fb,
						struct drm_file *file_priv,
						unsigned int *handle)
{
	struct glamodrm_handle *gdrm = fb->dev->dev_private;
	struct glamo_fbdev *fbdev = to_glamo_fbdev(gdrm->fb_helper);

	DRM_DEBUG("\n");
	return drm_gem_handle_create(file_priv, fbdev->obj, handle);
}


static const struct drm_framebuffer_funcs glamo_fb_funcs = {
	.destroy = glamo_framebuffer_destroy,
	.create_handle = glamo_framebuffer_create_handle,
};


struct drm_framebuffer * glamo_framebuffer_create(struct drm_device *dev,
			     struct drm_mode_fb_cmd2 *mode_cmd,
			     struct drm_gem_object *obj)
{
	struct drm_framebuffer *fb;
	int ret;

	DRM_DEBUG("%dx%d %#x\n", mode_cmd->width, mode_cmd->height, mode_cmd->pixel_format);
	
	fb = kzalloc(sizeof(*fb), GFP_KERNEL);
	if (!fb) {
		DRM_ERROR("could not allocate fb\n");
		return ERR_PTR(-ENOMEM);
	}

	ret = drm_framebuffer_init(dev, fb, &glamo_fb_funcs);
	if (ret) {
        drm_gem_object_unreference_unlocked(obj);
        
        DRM_ERROR("framebuffer init failed %d\n", ret);
        return ERR_PTR(ret);
	}

	drm_helper_mode_fill_fb_struct(fb, mode_cmd);

    DRM_DEBUG("%p\n", fb);
	return fb;
}

