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

static void glamo_connector_dpms(struct drm_connector *connector, int mode)
{
	struct glamodrm_handle *gdrm = connector->dev->dev_private;
	int old_dpms;
	
	DRM_DEBUG("%d => %d\n", connector->dpms, mode);
	
	old_dpms = connector->dpms;
	
	/* from off to on, do from crtc to connector */
	if (mode < old_dpms) {
		drm_helper_connector_dpms(connector, mode);
	}

	if (mode != old_dpms)
		glamo_lcd_power(gdrm, mode);
	
	if (mode > old_dpms) {
		drm_helper_connector_dpms(connector, mode);
	}
}

static enum drm_connector_status
glamo_connector_detect(struct drm_connector *connector, bool force)
{
	/* One hopes it hasn't been de-soldered... */
	return connector_status_connected;
}

struct drm_encoder *
glamo_connector_best_encoder(struct drm_connector *connector)
{
/*	struct glamo_output *glamo_output = to_glamo_output(connector);
	return &glamo_output->enc;*/
	
	struct drm_mode_config *mode_config = &connector->dev->mode_config;
	struct drm_encoder *encoder;
	
	list_for_each_entry(encoder, &mode_config->encoder_list, head)
		return encoder;
	
	return NULL;
}

static int glamo_connector_get_modes(struct drm_connector *connector)
{
	struct glamo_fb_platform_data *fb_info;
	struct glamodrm_handle *gdrm = connector->dev->dev_private;
	int i;

	/* Dig out the record which will tell us about the hardware */
	fb_info = gdrm->glamo_core->pdata->fb_data;

	for ( i=0; i<fb_info->num_modes; i++ ) {

		struct drm_display_mode *mode;

		mode = drm_mode_create(connector->dev);
		if ( !mode ) continue;

		mode->type = DRM_MODE_TYPE_DEFAULT | DRM_MODE_TYPE_PREFERRED;

		/* Convert framebuffer timings into KMS timings.
		 * First:  ps -> kHz */
		mode->clock = 1000000000UL / fb_info->modes[i].pixclock;
		mode->clock *= 1000; /* then kHz -> Hz */
		mode->hdisplay = fb_info->modes[i].xres;
		mode->hsync_start = fb_info->modes[i].right_margin
		                     + mode->hdisplay;
		mode->hsync_end = mode->hsync_start
		                     + fb_info->modes[i].hsync_len;
		mode->htotal = mode->hsync_end + fb_info->modes[i].left_margin;
		mode->hskew = 0;

		mode->vdisplay = fb_info->modes[i].yres;
		mode->vsync_start = fb_info->modes[i].lower_margin
		                     + mode->vdisplay;
		mode->vsync_end = mode->vsync_start
		                   + fb_info->modes[i].vsync_len;
		mode->vtotal = mode->vsync_end + fb_info->modes[i].upper_margin;
		mode->vscan = 0;

		/* Physical size */
		mode->width_mm = fb_info->width;
		mode->height_mm = fb_info->height;

		drm_mode_set_name(mode);
		drm_mode_probed_add(connector, mode);

	}

	return fb_info->num_modes;
}


static int glamo_connector_set_property(struct drm_connector *connector,
				  struct drm_property *property,
				  uint64_t value)
{
	return 0;
}


static int glamo_connector_mode_valid(struct drm_connector *connector,
                                      struct drm_display_mode *mode)
{
	if (mode->flags & DRM_MODE_FLAG_DBLSCAN)
		return MODE_NO_DBLESCAN;

	return MODE_OK;
}


static void glamo_connector_destroy(struct drm_connector *connector)
{
	drm_sysfs_connector_remove(connector);
	drm_connector_cleanup(connector);
	kfree(connector);
}

/* Connector functions */
static const struct drm_connector_funcs glamo_connector_funcs = {
	.dpms = glamo_connector_dpms,
	.detect = glamo_connector_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = glamo_connector_destroy,
	.set_property = glamo_connector_set_property,
};


/* Connector helper functions */
static const struct drm_connector_helper_funcs glamo_connector_helper_funcs = {
	.mode_valid = glamo_connector_mode_valid,
	.get_modes = glamo_connector_get_modes,
	.best_encoder = glamo_connector_best_encoder,
};

struct drm_connector * glamo_connector_init(struct drm_device *dev)
{
	struct drm_connector *connector;
	DRM_DEBUG("\n");
	
	connector = kzalloc(sizeof(struct drm_connector), GFP_KERNEL);
	if (!connector)
		return ERR_PTR(-ENOMEM);
	
	drm_connector_init(dev, connector, &glamo_connector_funcs,
	                   DRM_MODE_CONNECTOR_LVDS);
	drm_connector_helper_add(connector, &glamo_connector_helper_funcs);
	
	connector->interlace_allowed = 0;
	connector->doublescan_allowed = 0;
	connector->dpms = DRM_MODE_DPMS_OFF;
	
	drm_sysfs_connector_add(connector);
	
	return connector;
}

