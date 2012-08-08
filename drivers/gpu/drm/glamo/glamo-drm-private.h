/* Smedia Glamo 336x/337x DRM private bits
 *
 * Copyright (C) 2008-2009 Thomas White <taw@bitwiz.org.uk>
 * Copyright (C) 2009 Andreas Pokorny <andreas.pokorny@gmail.com>
 * Based on xf86-video-glamo
 * Copyright  2007 OpenMoko, Inc.
 * Copyright © 2009 Lars-Peter Clausen <lars@metafoo.de>
 *
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
 */

#ifndef __GLAMO_DRMPRIV_H
#define __GLAMO_DRMPRIV_H


#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/semaphore.h>
#include <linux/spinlock.h>
#include <linux/wait.h>

#include <drm/drm_fb_helper.h>
#include <drm/drm_crtc_helper.h>

#include <linux/mfd/glamo-core.h>


/* Memory to allocate for the framebuffer.
 * The rest is reserved for the DRM memory manager */
#define GLAMO_FRAMEBUFFER_ALLOCATION (2*480*640)


struct glamodrm_handle {

	/* This device */
	struct drm_device *dev;

	/* The parent device handle */
	struct glamo_core *glamo_core;

	/* Framebuffer handle for the console (i.e. /dev/fb0), now use helper */
	struct drm_fb_helper *fb_helper;

	/* Command queue registers */
	struct resource *reg;
	char __iomem *reg_base;

	/* VRAM region */
	struct resource *vram;

	/* Command queue region */
	char __iomem *cmdq_base;
	unsigned long cmdq_offs;

	/* LCD controller registers */
	struct resource *lcd_regs;
	char __iomem *lcd_base;

	/* 2D engine registers and IRQ */
	struct resource *twod_regs;
	char __iomem *twod_base;
	unsigned int twod_irq;

	/* Memory management */
	struct drm_mm *mmgr;

	/* semaphore against concurrent ioctl */
	struct semaphore add_to_ring;

	int lcd_cmd_mode;

	/* Fencing */
	atomic_t curr_seq;              /* The last used stamp number */
	struct list_head fence_list;    /* List of active fences */
	rwlock_t fence_list_lock;       /* Lock to protect fence_list */
	wait_queue_head_t fence_queue;  /* Waitqueue */
	struct tasklet_struct fence_tl; /* Tasklet for fence IRQ */

	/* A scratch block */
	struct drm_mm_node *scratch;
};


/* Private data.  This is where we keep our memory management bits */
struct drm_glamo_gem_object {
	struct drm_gem_object *obj;	/* The GEM object this refers to */
	struct drm_mm_node *block;	/* Block handle for drm_mm */
	uint64_t mmap_offset;
};

struct glamo_fbdev {
	struct drm_fb_helper base;
	struct drm_gem_object *obj;
};


/* Colour mode for KMS framebuffer */
enum {
	GLAMO_FB_RGB565,
	GLAMO_FB_ARGB1555,
	GLAMO_FB_ARGB4444
};

#define to_glamo_fbdev(x) container_of(x, struct glamo_fbdev, base)


#endif /* __GLAMO_DRMPRIV_H */
