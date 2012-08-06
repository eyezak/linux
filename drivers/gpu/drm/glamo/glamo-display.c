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

extern void glamo_lcd_init(struct glamodrm_handle *gdrm);
extern int glamo_crtc_init(struct drm_device *dev);
extern int glamo_crtc_resume(struct glamodrm_handle *gdrm);
extern int glamo_encoder_init(struct drm_device *dev, struct drm_encoder *enc);
extern int glamo_connector_init(struct drm_device *dev,
                         struct drm_connector * connector);
extern int glamo_crtc_mode_set(struct drm_crtc *crtc,
                               struct drm_display_mode *mode,
                               struct drm_display_mode *adjusted_mode,
                               int x, int y,
                               struct drm_framebuffer *old_fb);
extern int glamo_run_lcd_script(struct glamodrm_handle *gdrm,
                                enum glamo_script_index sc);
extern int reg_read_lcd(struct glamodrm_handle *gdrm, u_int16_t reg);


struct glamofb_par {
	struct drm_device *dev;
	struct drm_display_mode *our_mode;
	struct glamo_framebuffer *glamo_fb;
	int crtc_count;
	/* crtc currently bound to this */
	uint32_t crtc_ids[2];
};


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


static struct drm_mode_set kernelfb_mode;


/* Restore's the kernel's fbcon mode, used for panic path */
void glamo_display_restore(void)
{
	drm_crtc_helper_set_config(&kernelfb_mode);
}


/*static int glamo_display_panic(struct notifier_block *n, unsigned long ununsed,
                               void *panic_str)
{
	DRM_ERROR("panic occurred, switching back to text console\n");

	glamo_display_restore();
	return 0;
}

static struct notifier_block paniced = {
	.notifier_call = glamo_display_panic,
};*/

int glamo_modeset_init(struct drm_device *dev)
{
	int ret;
	struct glamo_output *glamo_output;
	struct glamo_crtc *glamo_crtc;
	struct glamodrm_handle *gdrm = dev->dev_private;

	DRM_DEBUG("\n");
	drm_mode_config_init(dev);

	glamo_output = kzalloc(sizeof(struct glamo_output), GFP_KERNEL);
	if (!glamo_output)
		return -ENOMEM;
	glamo_output->gdrm = gdrm;

	ret = glamo_encoder_init(dev, &glamo_output->enc);
	if (ret)
		return ret;
	ret = glamo_connector_init(dev, &glamo_output->base);
	if (ret)
		return ret;
	
	ret = glamo_crtc_init(dev);
	if (ret)
		return ret;
	glamo_crtc = to_glamo_crtc(gdrm->crtc);
	glamo_crtc->mode_set.connectors[0] = &glamo_output->base;
	glamo_crtc->mode_set.num_connectors = 1;

	dev->mode_config.min_width = 240;
	dev->mode_config.min_height = 240;
	dev->mode_config.max_width = 640;
	dev->mode_config.max_height = 640;
	dev->mode_config.funcs = &glamo_mode_funcs;

	drm_mode_connector_attach_encoder(&glamo_output->base,
	                                  &glamo_output->enc);
	
	return 0;
}

void glamo_display_suspend(struct glamodrm_handle *gdrm)
{
	/* do nothing */
}


void glamo_display_resume(struct glamodrm_handle *gdrm)
{
	/*glamo_engine_enable(gdrm->glamo_core, GLAMO_ENGINE_LCD);
	glamo_engine_reset(gdrm->glamo_core, GLAMO_ENGINE_LCD);
	glamo_run_lcd_script(gdrm, GLAMO_SCRIPT_LCD_INIT);*/

	glamo_crtc_resume(gdrm);
}
