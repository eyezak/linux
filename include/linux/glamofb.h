#ifndef _LINUX_GLAMOFB_H
#define _LINUX_GLAMOFB_H

#ifdef __KERNEL__

#include <linux/fb.h>
struct drm_display_mode;

struct glamo_fb_platform_data {
    int width, height;

    void (* mode_change)(struct drm_display_mode *mode);

    int num_modes;
    struct fb_videomode *modes;
};

#endif

#define GLAMOFB_ENGINE_ENABLE _IOW('F', 0x1, __u32)
#define GLAMOFB_ENGINE_DISABLE _IOW('F', 0x2, __u32)
#define GLAMOFB_ENGINE_RESET _IOW('F', 0x3, __u32)

#endif
