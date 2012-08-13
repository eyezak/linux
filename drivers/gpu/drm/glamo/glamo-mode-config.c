/*
 * SMedia Glamo 336x/337x display
 *
 * Copyright (C) 2012 Isaac Gordezky <eye.zak.devel@gmail.com>
 *
 * Based on glamo-display.c (C) 2009-2010 by Thomas White
 * Author: Thomas White <taw@bitwiz.org.uk>
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


static struct drm_framebuffer *
glamo_user_framebuffer_create(struct drm_device *dev,
			      struct drm_file *filp,
			      struct drm_mode_fb_cmd2 *mode_cmd)
{
	struct drm_gem_object *obj;
	struct drm_framebuffer *fb;
	DRM_DEBUG("\n");

    obj = drm_gem_object_lookup(dev, filp, mode_cmd->handles[0]);
	if (!obj) {
		DRM_ERROR("unable to find handle object %u\n", mode_cmd->handles[0]);
		return ERR_PTR(ENOENT);
	}

	fb = glamo_framebuffer_create(dev, mode_cmd, obj);
	if (IS_ERR(fb)) {
		drm_gem_object_unreference(obj);
		return fb;
	}

	return fb;
}


void glamo_fbchanged(struct drm_device *dev)
{
    /* stub TODO: fixme */
    DRM_DEBUG("TODO: stub\n");
}


/* Framebuffer mode functions */
static const struct drm_mode_config_funcs glamo_mode_funcs = {
	.fb_create = glamo_user_framebuffer_create,
	.output_poll_changed = glamo_fbchanged,
};

void glamo_mode_config_init(struct drm_device *dev)
{
	DRM_DEBUG("\n");
	drm_mode_config_init(dev);
	
	dev->mode_config.min_width = 240;
	dev->mode_config.min_height = 240;
	dev->mode_config.max_width = 640;
	dev->mode_config.max_height = 640;
	dev->mode_config.funcs = &glamo_mode_funcs;
}

