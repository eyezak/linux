#ifndef __LINUX_HDQ_H__
#define __LINUX_HDQ_H__

#include <linux/device.h>

#define HDQ_SAMPLE_PERIOD_US	10

/* platform data */

struct fiq_hdq_ops {
	int (* read)(struct device *dev, unsigned int address);
};

struct fiq_hdq_platform_data {
	int timer_id;
	unsigned int timer_irq;
	unsigned int gpio;
	unsigned int gpio_output;
	unsigned int gpio_input;
	struct device *parent;

	void (*gpio_dir_out)(unsigned int irq, unsigned int);
	void (*gpio_dir_in)(unsigned int irq, unsigned int);
	void (*gpio_set)(unsigned int irq, unsigned int val);
	unsigned int (*gpio_get)(unsigned int irq);

	int (*set_fiq)(unsigned int irq, bool on);
	void (*kick_fiq)(unsigned int irq);
	void (*handle_fiq)(unsigned long irq, int disable);
};

#endif
