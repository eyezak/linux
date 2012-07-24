/* Smedia Glamo 3362 driver
 *
 * (C) 2007 by Openmoko, Inc.
 * Author: Harald Welte <laforge@openmoko.org>
 * (C) 2009, Lars-Peter Clausen <lars@metafoo.de>
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/kernel_stat.h>
#include <linux/spinlock.h>
#include <linux/mfd/core.h>
#include <linux/mfd/glamo.h>
#include <linux/mfd/glamo-regs.h>
#include <linux/mfd/glamo-core.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>

#include <linux/pm.h>

#define GLAMO_MEM_REFRESH_COUNT 0x100

#define GLAMO_NR_IRQS 9

#define GLAMO_IRQ_HOSTBUS	0
#define GLAMO_IRQ_JPEG		1
#define GLAMO_IRQ_MPEG		2
#define GLAMO_IRQ_MPROC1	3
#define GLAMO_IRQ_MPROC0	4
#define GLAMO_IRQ_CMDQUEUE	5
#define GLAMO_IRQ_2D		6
#define GLAMO_IRQ_MMC		7
#define GLAMO_IRQ_RISC		8

/*
 * Glamo internal settings
 *
 * We run the memory interface from the faster PLLB on 2.6.28 kernels and
 * above.  Couple of GTA02 users report trouble with memory bus when they
 * upgraded from 2.6.24.  So this parameter allows reversion to 2.6.24
 * scheme if their Glamo chip needs it.
 *
 * you can override the faster default on kernel commandline using
 *
 *   glamo3362.slow_memory=1
 *
 * for example
 */

static int slow_memory;
module_param(slow_memory, int, 0644);

struct reg_range {
	int start;
	int count;
	char *name;
	unsigned dump:1;
};

static const struct reg_range reg_range[] = {
	{ 0x0000, 0x76,		"General",	1 },
	{ 0x0200, 0x18,		"Host Bus",	1 },
	{ 0x0300, 0x38,		"Memory",	1 },
/*	{ 0x0400, 0x100,	"Sensor",	0 }, */
/*	{ 0x0500, 0x300,	"ISP",		0 }, */
	{ 0x0800, 0x400,	"JPEG",		0 },
/*	{ 0x0c00, 0xcc,		"MPEG",		0 }, */
	{ 0x1100, 0xb2,		"LCD 1",	0 },
	{ 0x1200, 0x64,		"LCD 2",	0 },
	{ 0x1400, 0x42,		"MMC",		0 },
/*	{ 0x1500, 0x080,	"MPU 0",	0 },
	{ 0x1580, 0x080,	"MPU 1",	0 },
	{ 0x1600, 0x080,	"Cmd Queue",	0 },
	{ 0x1680, 0x080,	"RISC CPU",	0 },*/
	{ 0x1700, 0x400,	"2D Unit",	0 },
/*	{ 0x1b00, 0x900,	"3D Unit",	0 }, */
};

void _set_irq_flags(unsigned int irq, unsigned int iflags)
{
	unsigned long clr = 0, set = IRQ_NOREQUEST | IRQ_NOPROBE | IRQ_NOAUTOEN;

	if (irq >= nr_irqs) {
		printk(KERN_ERR "Trying to set irq flags for IRQ%d\n", irq);
		return;
	}

	if (iflags & IRQF_VALID)
		clr |= IRQ_NOREQUEST;
	if (iflags & IRQF_PROBE)
		clr |= IRQ_NOPROBE;
	if (!(iflags & IRQF_NOAUTOEN))
		clr |= IRQ_NOAUTOEN;
	/* Order is clear bits in "clr" then set bits in "set" */
	irq_modify_status(irq, clr, set & ~clr);
}

static inline void __reg_write(struct glamo_core *glamo,
				uint16_t reg, uint16_t val)
{
	writew(val, glamo->base + reg);
}

void glamo_reg_write(struct glamo_core *glamo,
				uint16_t reg, uint16_t val)
{
	spin_lock(&glamo->lock);
	__reg_write(glamo, reg, val);
	spin_unlock(&glamo->lock);
}
EXPORT_SYMBOL_GPL(glamo_reg_write);


static inline uint16_t __reg_read(struct glamo_core *glamo,
					uint16_t reg)
{
	return readw(glamo->base + reg);
}

uint16_t glamo_reg_read(struct glamo_core *glamo, uint16_t reg)
{
	uint16_t val;
	spin_lock(&glamo->lock);
	val = __reg_read(glamo, reg);
	spin_unlock(&glamo->lock);

	return val;
}
EXPORT_SYMBOL_GPL(glamo_reg_read);

static void __reg_set_bit_mask(struct glamo_core *glamo,
				uint16_t reg, uint16_t mask,
				uint16_t val)
{
	uint16_t tmp;

	val &= mask;

	tmp = __reg_read(glamo, reg);
	tmp &= ~mask;
	tmp |= val;
	__reg_write(glamo, reg, tmp);
}

static void reg_set_bit_mask(struct glamo_core *glamo,
				uint16_t reg, uint16_t mask,
				uint16_t val)
{
	spin_lock(&glamo->lock);
	__reg_set_bit_mask(glamo, reg, mask, val);
	spin_unlock(&glamo->lock);
}



static void reg_checkandset_bit_mask(struct glamo_core *glamo,
				uint16_t reg, uint16_t mask,
				uint16_t val, uint16_t check)
{
	spin_lock(&glamo->lock);
	if ((__reg_read(glamo, reg) & mask) == check)
	    __reg_set_bit_mask(glamo, reg, mask, val);
	spin_unlock(&glamo->lock);
}


static inline void __reg_set_bit(struct glamo_core *glamo,
				 uint16_t reg, uint16_t bit)
{
	uint16_t tmp;
	tmp = __reg_read(glamo, reg);
	tmp |= bit;
	__reg_write(glamo, reg, tmp);
}

void glamo_pixclock_slow (struct glamo_core *glamo) 
{
	
	int x, lastx = 0;
	int timeout = 1000000;
	int threshold = 5;
	int fa;

 	int evcnt = 0;

	for (fa = 0; fa < timeout; fa++) {
		x = glamo_reg_read(glamo, 0x1100 + GLAMO_REG_LCD_STATUS1) & 0x3ff;


		if (x == lastx) {
			evcnt++;
			if (evcnt == threshold)
				break;
		} else {
			evcnt = 0;
			lastx = x;
		}		
	}
	if (fa == timeout) {
		printk (KERN_WARNING "Glamo: Error waiting for stable x position.\n");
	}

	/* then, make glamo slower */
	/* it's not a problems if in rare case we do not slow down glamo properly
	   as all we'll get in that case is singe jittered value */
	
	glamo->slowed_divider = glamo_reg_read (glamo, 0x36) & 0xFF;
	reg_set_bit_mask (glamo, 0x36, 0xFF, 0xFF);

}

void glamo_pixclock_fast (struct glamo_core *glamo) 
{
	reg_checkandset_bit_mask (glamo, 0x36, 0xFF, glamo->slowed_divider, 0xFF);
}
EXPORT_SYMBOL_GPL(glamo_pixclock_fast);
EXPORT_SYMBOL_GPL(glamo_pixclock_slow);

static inline void __reg_clear_bit(struct glamo_core *glamo,
					uint16_t reg, uint16_t bit)
{
	uint16_t tmp;
	tmp = __reg_read(glamo, reg);
	tmp &= ~bit;
	__reg_write(glamo, reg, tmp);
}

/***********************************************************************
 * resources of sibling devices
 ***********************************************************************/

static struct resource glamo_fb_resources[] = {
	{
		.name	= "glamo-fb-regs",
		.start	= GLAMO_REGOFS_LCD,
		.end	= GLAMO_REGOFS_MMC - 1,
		.flags	= IORESOURCE_MEM,
	}, {
		.name	= "glamo-fb-mem",
		.start	= GLAMO_OFFSET_FB,
		.end	= GLAMO_OFFSET_FB + GLAMO_FB_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource glamo_mmc_resources[] = {
	{
		.name	= "glamo-mmc-regs",
		.start	= GLAMO_REGOFS_MMC,
		.end	= GLAMO_REGOFS_MPROC0 - 1,
		.flags	= IORESOURCE_MEM
	}, {
		.name	= "glamo-mmc-mem",
		.start	= GLAMO_OFFSET_FB + GLAMO_FB_SIZE,
		.end	= GLAMO_OFFSET_FB + GLAMO_FB_SIZE +
				  GLAMO_MMC_BUFFER_SIZE - 1,
		.flags	= IORESOURCE_MEM
	}, {
		.start	= GLAMO_IRQ_MMC,
		.end	= GLAMO_IRQ_MMC,
		.flags	= IORESOURCE_IRQ,
	},
};

enum glamo_cells {
	GLAMO_CELL_FB,
	GLAMO_CELL_MMC,
	GLAMO_CELL_GPIO,
};

static struct mfd_cell glamo_cells[] = {
	[GLAMO_CELL_FB] = {
		.name = "glamo-fb",
		.num_resources = ARRAY_SIZE(glamo_fb_resources),
		.resources = glamo_fb_resources,
	},
	[GLAMO_CELL_MMC] = {
		.name = "glamo-mci",
		.num_resources = ARRAY_SIZE(glamo_mmc_resources),
		.resources = glamo_mmc_resources,
	},
	[GLAMO_CELL_GPIO] = {
		.name = "glamo-gpio",
	},
};

/***********************************************************************
 * IRQ demultiplexer
 ***********************************************************************/
static inline unsigned int glamo_irq_bit(struct glamo_core *glamo,
	struct irq_data *data)
{
	return BIT(data->irq - glamo->irq_base);
}

static void glamo_ack_irq(struct irq_data *data)
{
	struct glamo_core *glamo = irq_data_get_irq_chip_data(data);
	/* clear interrupt source */
	__reg_write(glamo, GLAMO_REG_IRQ_CLEAR, glamo_irq_bit(glamo, data));
}

static void glamo_mask_irq(struct irq_data *data)
{
	struct glamo_core *glamo = irq_data_get_irq_chip_data(data);

	/* clear bit in enable register */
	__reg_clear_bit(glamo, GLAMO_REG_IRQ_ENABLE, glamo_irq_bit(glamo, data));
}

static void glamo_unmask_irq(struct irq_data *data)
{
	struct glamo_core *glamo = irq_data_get_irq_chip_data(data);

	/* set bit in enable register */
	__reg_set_bit(glamo, GLAMO_REG_IRQ_ENABLE, glamo_irq_bit(glamo, data));
}

static struct irq_chip glamo_irq_chip = {
	.name		= "glamo",
	.irq_ack	= glamo_ack_irq,
	.irq_mask	= glamo_mask_irq,
	.irq_unmask	= glamo_unmask_irq,
};

static irqreturn_t glamo_irq_handler(int irq, void *devid)
{
	struct glamo_core *glamo = devid;
	uint16_t irqstatus;
	int i;

	irqstatus = __reg_read(glamo, GLAMO_REG_IRQ_STATUS);
	for (i = 0; i < 9; ++i) {
		if (irqstatus & BIT(i))
			generic_handle_irq(glamo->irq_base + i);
	}

	return IRQ_HANDLED;
}

struct glamo_engine_reg_set {
	uint16_t reg;
	uint16_t mask_suspended;
	uint16_t mask_enabled;
};

/*
debugfs
*/

#ifdef CONFIG_DEBUG_FS
static ssize_t debugfs_regs_write(struct file *file,
				  const char __user *user_buf,
				  size_t count, loff_t *ppos)
{
	struct glamo_core *glamo = ((struct seq_file *)file->private_data)->private;
	char buf[14];
	unsigned int reg;
	unsigned int val;
	int buf_size;

	buf_size = min(count, sizeof(buf) - 1);
	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;
	if (sscanf(buf, "%x %x", &reg, &val) != 2)
		return -EFAULT;

	dev_info(&glamo->pdev->dev, "reg %#02x <-- %#04x\n", reg, val);

	glamo_reg_write(glamo, reg, val);

	return count;
}

static int glamo_show_regs(struct seq_file *s, void *pos)
{
	struct glamo_core *glamo = s->private;
	const struct reg_range *rr = reg_range;
	int i, n;

	spin_lock(&glamo->lock);
	for (i = 0; i < ARRAY_SIZE(reg_range); ++i, ++rr) {
		if (!rr->dump)
			continue;
		seq_printf(s, "\n%s\n", rr->name);
		for (n = rr->start; n < rr->start + rr->count; n += 2) {
			if ((n & 15) == 0)
				seq_printf(s, "\n%04X:  ", n);
			seq_printf(s, "%04x ", __reg_read(glamo, n));
		}
		seq_printf(s, "\n");
	}
	spin_unlock(&glamo->lock);

	return 0;
}

static int debugfs_open_file(struct inode *inode, struct file *file)
{
	return single_open(file, glamo_show_regs, inode->i_private);
}

static const struct file_operations debugfs_regs_ops = {
	.open		= debugfs_open_file,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.write		= debugfs_regs_write,
	.release	= single_release,
};

static void glamo_init_debugfs(struct glamo_core *glamo)
{
	glamo->debugfs_dir = debugfs_create_dir("glamo3362", NULL);
	if (glamo->debugfs_dir)
		debugfs_create_file("regs", S_IRUGO | S_IWUSR,
				    glamo->debugfs_dir,
				    glamo, &debugfs_regs_ops);
	else
		dev_warn(&glamo->pdev->dev, "Failed to set up debugfs.\n");
}

static void glamo_exit_debugfs(struct glamo_core *glamo)
{
	if (glamo->debugfs_dir)
		debugfs_remove_recursive(glamo->debugfs_dir);
}
#else
static void glamo_init_debugfs(struct glamo_core *glamo)
{
}

static void glamo_exit_debugfs(struct glamo_core *glamo)
{
}
#endif

struct glamo_engine_desc {
	const char *name;
	uint16_t hostbus;
	const struct glamo_engine_reg_set *regs;
	size_t num_regs;
};

static const struct glamo_engine_reg_set glamo_lcd_regs[] = {
	{ GLAMO_REG_CLOCK_LCD,
	GLAMO_CLOCK_LCD_EN_M5CLK |
	GLAMO_CLOCK_LCD_DG_M5CLK |
	GLAMO_CLOCK_LCD_EN_DMCLK,

	GLAMO_CLOCK_LCD_EN_DHCLK |
	GLAMO_CLOCK_LCD_EN_DCLK
	},
	{ GLAMO_REG_CLOCK_GEN5_1,
	GLAMO_CLOCK_GEN51_EN_DIV_DMCLK,

	GLAMO_CLOCK_GEN51_EN_DIV_DHCLK |
	GLAMO_CLOCK_GEN51_EN_DIV_DCLK
	}
};

static const struct glamo_engine_reg_set glamo_mmc_regs[] = {
	{ GLAMO_REG_CLOCK_MMC,
	GLAMO_CLOCK_MMC_EN_M9CLK |
	GLAMO_CLOCK_MMC_DG_M9CLK,

	GLAMO_CLOCK_MMC_EN_TCLK |
	GLAMO_CLOCK_MMC_DG_TCLK
	},
	{ GLAMO_REG_CLOCK_GEN5_1,
	0,
	GLAMO_CLOCK_GEN51_EN_DIV_TCLK
	}
};

static const struct glamo_engine_reg_set glamo_2d_regs[] = {
	{ GLAMO_REG_CLOCK_2D,
	GLAMO_CLOCK_2D_EN_M7CLK |
	GLAMO_CLOCK_2D_DG_M7CLK,

	GLAMO_CLOCK_2D_EN_GCLK |
	GLAMO_CLOCK_2D_DG_GCLK
	},
	{ GLAMO_REG_CLOCK_GEN5_1,
	0,
	GLAMO_CLOCK_GEN51_EN_DIV_GCLK,
	}
};

static const struct glamo_engine_reg_set glamo_cmdq_regs[] = {
	{ GLAMO_REG_CLOCK_2D,
	GLAMO_CLOCK_2D_EN_M6CLK,
	0
	},
};

#define GLAMO_ENGINE(xname, xhostbus, xregs) { \
	.name = xname, \
	.hostbus = xhostbus, \
	.num_regs = ARRAY_SIZE(xregs), \
	.regs = xregs, \
}

static const struct glamo_engine_desc glamo_engines[] = {
	[GLAMO_ENGINE_LCD] = GLAMO_ENGINE("LCD", GLAMO_HOSTBUS2_MMIO_EN_LCD,
					glamo_lcd_regs),
	[GLAMO_ENGINE_MMC] = GLAMO_ENGINE("MMC", GLAMO_HOSTBUS2_MMIO_EN_MMC,
					glamo_mmc_regs),
	[GLAMO_ENGINE_2D] = GLAMO_ENGINE("2D", GLAMO_HOSTBUS2_MMIO_EN_2D,
					glamo_2d_regs),
	[GLAMO_ENGINE_CMDQ] = GLAMO_ENGINE("CMDQ", GLAMO_HOSTBUS2_MMIO_EN_CQ,
					glamo_cmdq_regs),
};

static inline const char *glamo_engine_name(enum glamo_engine engine)
{
	return glamo_engines[engine].name;
}

/***********************************************************************
 * 'engine' support
 ***********************************************************************/

int __glamo_engine_enable(struct glamo_core *glamo, enum glamo_engine engine)
{
	int i;
	const struct glamo_engine_desc *engine_desc = &glamo_engines[engine];
	const struct glamo_engine_reg_set *reg;

	switch (engine) {
	case GLAMO_ENGINE_LCD:
	case GLAMO_ENGINE_MMC:
	case GLAMO_ENGINE_2D:
	case GLAMO_ENGINE_CMDQ:
		break;
	default:
		return -EINVAL;
	}

	reg = engine_desc->regs;

	__reg_set_bit(glamo, GLAMO_REG_HOSTBUS(2),
			engine_desc->hostbus);
	for (i = engine_desc->num_regs; i; --i, ++reg)
		__reg_set_bit(glamo, reg->reg,
				reg->mask_suspended | reg->mask_enabled);

	return 0;
}

int glamo_engine_enable(struct glamo_core *glamo, enum glamo_engine engine)
{
	int ret = 0;

	spin_lock(&glamo->lock);

	if (glamo->engine_state[engine] != GLAMO_ENGINE_ENABLED) {
		ret = __glamo_engine_enable(glamo, engine);
		if (!ret)
			glamo->engine_state[engine] = GLAMO_ENGINE_ENABLED;
	}

	spin_unlock(&glamo->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(glamo_engine_enable);

int __glamo_engine_disable(struct glamo_core *glamo, enum glamo_engine engine)
{
	int i;
	const struct glamo_engine_desc *engine_desc = &glamo_engines[engine];
	const struct glamo_engine_reg_set *reg;

	switch (engine) {
	case GLAMO_ENGINE_LCD:
	case GLAMO_ENGINE_MMC:
	case GLAMO_ENGINE_2D:
	case GLAMO_ENGINE_CMDQ:
		break;
	default:
		return -EINVAL;
	}

	reg = engine_desc->regs;

	__reg_clear_bit(glamo, GLAMO_REG_HOSTBUS(2),
			engine_desc->hostbus);
	for (i = engine_desc->num_regs; i; --i, ++reg)
		__reg_clear_bit(glamo, reg->reg,
				reg->mask_suspended | reg->mask_enabled);

	return 0;
}
int glamo_engine_disable(struct glamo_core *glamo, enum glamo_engine engine)
{
	int ret = 0;

	spin_lock(&glamo->lock);

	if (glamo->engine_state[engine] != GLAMO_ENGINE_DISABLED) {
		ret = __glamo_engine_disable(glamo, engine);
		if (!ret)
			glamo->engine_state[engine] = GLAMO_ENGINE_DISABLED;
	}

	spin_unlock(&glamo->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(glamo_engine_disable);

int __glamo_engine_suspend(struct glamo_core *glamo, enum glamo_engine engine)
{
	int i;
	const struct glamo_engine_desc *engine_desc = &glamo_engines[engine];
	const struct glamo_engine_reg_set *reg;

	switch (engine) {
	case GLAMO_ENGINE_LCD:
	case GLAMO_ENGINE_MMC:
	case GLAMO_ENGINE_2D:
	case GLAMO_ENGINE_CMDQ:
		break;
	default:
		return -EINVAL;
	}

	reg = engine_desc->regs;

	__reg_set_bit(glamo, GLAMO_REG_HOSTBUS(2),
			engine_desc->hostbus);
	for (i = engine_desc->num_regs; i; --i, ++reg) {
		__reg_set_bit(glamo, reg->reg, reg->mask_suspended);
		__reg_clear_bit(glamo, reg->reg, reg->mask_enabled);
	}

	return 0;
}

int glamo_engine_suspend(struct glamo_core *glamo, enum glamo_engine engine)
{
	int ret = 0;

	spin_lock(&glamo->lock);

	if (glamo->engine_state[engine] != GLAMO_ENGINE_SUSPENDED) {
		ret = __glamo_engine_suspend(glamo, engine);
		if (!ret)
			glamo->engine_state[engine] = GLAMO_ENGINE_SUSPENDED;
	}

	spin_unlock(&glamo->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(glamo_engine_suspend);

static const struct glamo_script reset_regs[] = {
	[GLAMO_ENGINE_LCD] = {
		GLAMO_REG_CLOCK_LCD, GLAMO_CLOCK_LCD_RESET
	},
	[GLAMO_ENGINE_MMC] = {
		GLAMO_REG_CLOCK_MMC, GLAMO_CLOCK_MMC_RESET
	},
	[GLAMO_ENGINE_CMDQ] = {
		GLAMO_REG_CLOCK_2D, GLAMO_CLOCK_2D_CQ_RESET
	},
	[GLAMO_ENGINE_2D] = {
		GLAMO_REG_CLOCK_2D, GLAMO_CLOCK_2D_RESET
	},
	[GLAMO_ENGINE_JPEG] = {
		GLAMO_REG_CLOCK_JPEG, GLAMO_CLOCK_JPEG_RESET
	},
};

void glamo_engine_reset(struct glamo_core *glamo, enum glamo_engine engine)
{
	uint16_t reg = reset_regs[engine].reg;
	uint16_t val = reset_regs[engine].val;

	if (engine >= ARRAY_SIZE(reset_regs)) {
		dev_warn(&glamo->pdev->dev, "unknown engine %u ", engine);
		return;
	}

	spin_lock(&glamo->lock);
	__reg_set_bit(glamo, reg, val);
	__reg_clear_bit(glamo, reg, val);
	spin_unlock(&glamo->lock);
}
EXPORT_SYMBOL_GPL(glamo_engine_reset);

int glamo_pll_rate(struct glamo_core *glamo,
			  enum glamo_pll pll)
{
	uint16_t reg;
	unsigned int osci = glamo->pdata->osci_clock_rate;

	switch (pll) {
	case GLAMO_PLL1:
		reg = __reg_read(glamo, GLAMO_REG_PLL_GEN1);
		break;
	case GLAMO_PLL2:
		reg = __reg_read(glamo, GLAMO_REG_PLL_GEN3);
		break;
	default:
		return -EINVAL;
	}

	return (int)osci * (int)reg;
}
EXPORT_SYMBOL_GPL(glamo_pll_rate);

int glamo_engine_reclock(struct glamo_core *glamo,
			 enum glamo_engine engine,
			 int hz)
{
	int pll;
	uint16_t reg, mask, div;

	if (!hz)
		return -EINVAL;

	switch (engine) {
	case GLAMO_ENGINE_LCD:
		pll = GLAMO_PLL1;
		reg = GLAMO_REG_CLOCK_GEN7;
		mask = 0xff;
		break;
	case GLAMO_ENGINE_MMC:
		pll = GLAMO_PLL1;
		reg = GLAMO_REG_CLOCK_GEN8;
		mask = 0xff;
		break;
	default:
		dev_warn(&glamo->pdev->dev,
			 "reclock of engine 0x%x not supported\n", engine);
		return -EINVAL;
		break;
	}

	pll = glamo_pll_rate(glamo, pll);

	div = pll / hz;

	if (div != 0 && pll / div <= hz)
		--div;

	if (div > mask)
		div = mask;

	dev_dbg(&glamo->pdev->dev,
			"PLL %d, kHZ %d, div %d\n", pll, hz / 1000, div);

	reg_set_bit_mask(glamo, reg, mask, div);
	mdelay(5); /* wait some time to stabilize */

	return pll / (div + 1);
}
EXPORT_SYMBOL_GPL(glamo_engine_reclock);

/***********************************************************************
 * script support
 ***********************************************************************/

#define GLAMO_SCRIPT_END	0xffff
#define GLAMO_SCRIPT_WAIT	0xfffe
#define GLAMO_SCRIPT_LOCK_PLL	0xfffd

/*
 * couple of people reported artefacts with 2.6.28 changes, this
 * allows reversion to 2.6.24 settings
*/
static const uint16_t reg_0x200[] = {
	0xe03, /* 0 waits on Async BB R & W, Use PLL 2 for mem bus */
	0xef0, /* 3 waits on Async BB R & W, Use PLL 1 for mem bus */
	0xea0, /* 2 waits on Async BB R & W, Use PLL 1 for mem bus */
	0xe50, /* 1 waits on Async BB R & W, Use PLL 1 for mem bus */
	0xe00, /* 0 waits on Async BB R & W, Use PLL 1 for mem bus */
	0xef3, /* 3 waits on Async BB R & W, Use PLL 2 for mem bus */
	0xea3, /* 2 waits on Async BB R & W, Use PLL 2 for mem bus */
	0xe53, /* 1 waits on Async BB R & W, Use PLL 2 for mem bus */
};

static int glamo_run_script(struct glamo_core *glamo,
				const struct glamo_script *script, int len,
				int may_sleep)
{
	int i;
	uint16_t status;
	const struct glamo_script *line = script;

	for (i = 0; i < len; ++i, ++line) {
		switch (line->reg) {
		case GLAMO_SCRIPT_END:
			return 0;
		case GLAMO_SCRIPT_WAIT:
			if (may_sleep)
				msleep(line->val);
			else
				mdelay(line->val * 4);
			break;
		case GLAMO_SCRIPT_LOCK_PLL:
			/* spin until PLLs lock */
			do {
				status = __reg_read(glamo, GLAMO_REG_PLL_GEN5);
			} while ((status & 3) != 3);
			break;
		case 0x200:
			__reg_write(glamo, line->reg,
					reg_0x200[slow_memory & 0x7]);
			break;
		default:
			__reg_write(glamo, line->reg, line->val);
			break;
		}
	}

	return 0;
}

static const struct glamo_script glamo_init_script[] = {
	{ GLAMO_REG_CLOCK_HOST,		0x1000 },
	{ GLAMO_SCRIPT_WAIT,		     2 },
	{ GLAMO_REG_CLOCK_MEMORY,	0x1000 },
	{ GLAMO_REG_CLOCK_MEMORY,	0x2000 },
	{ GLAMO_REG_CLOCK_LCD,		0x1000 },
	{ GLAMO_REG_CLOCK_MMC,		0x1000 },
	{ GLAMO_REG_CLOCK_ISP,		0x1000 },
	{ GLAMO_REG_CLOCK_ISP,		0x3000 },
	{ GLAMO_REG_CLOCK_JPEG,		0x1000 },
	{ GLAMO_REG_CLOCK_3D,		0x1000 },
	{ GLAMO_REG_CLOCK_3D,		0x3000 },
	{ GLAMO_REG_CLOCK_2D,		0x1000 },
	{ GLAMO_REG_CLOCK_2D,		0x3000 },
	{ GLAMO_REG_CLOCK_RISC1,	0x1000 },
	{ GLAMO_REG_CLOCK_MPEG,		0x1000 },
	{ GLAMO_REG_CLOCK_MPEG,		0x3000 },
	{ GLAMO_REG_CLOCK_MPROC,	0x1000 /*0x100f*/ },
	{ GLAMO_SCRIPT_WAIT,		     2 },
	{ GLAMO_REG_CLOCK_HOST,		0x0000 },
	{ GLAMO_REG_CLOCK_MEMORY,	0x0000 },
	{ GLAMO_REG_CLOCK_LCD,		0x0000 },
	{ GLAMO_REG_CLOCK_MMC,		0x0000 },
	{ GLAMO_REG_PLL_GEN1,		0x05db },	/* 48MHz */
	{ GLAMO_REG_PLL_GEN3,		0x0aba },	/* 90MHz */
	{ GLAMO_SCRIPT_LOCK_PLL, 0 },
	/*
	 * b9 of this register MUST be zero to get any interrupts on INT#
	 * the other set bits enable all the engine interrupt sources
	 */
	{ GLAMO_REG_IRQ_ENABLE,		0x0100 },
	{ GLAMO_REG_CLOCK_GEN6,		0x2000 },
	{ GLAMO_REG_CLOCK_GEN7,		0x0101 },
	{ GLAMO_REG_CLOCK_GEN8,		0x0100 },
	{ GLAMO_REG_CLOCK_HOST,		0x000d },
	/*
	 * b7..b4 = 0 = no wait states on read or write
	 * b0 = 1 select PLL2 for Host interface, b1 = enable it
	 */
	{ GLAMO_REG_HOSTBUS(1),		0x0e03 /* this is replaced by script parser */ },
	{ GLAMO_REG_HOSTBUS(2),		0x07ff }, /* TODO: Disable all */
	{ GLAMO_REG_HOSTBUS(10),	0x0000 },
	{ GLAMO_REG_HOSTBUS(11),	0x4000 },
	{ GLAMO_REG_HOSTBUS(12),	0xf00e },

	/* S-Media recommended "set tiling mode to 512 mode for memory access
	 * more efficiency when 640x480" */
	{ GLAMO_REG_MEM_TYPE,		0x0c74 }, /* 8MB, 16 word pg wr+rd */
	{ GLAMO_REG_MEM_GEN,		0xafaf }, /* 63 grants min + max */

	{ GLAMO_REG_MEM_TIMING1,	0x0108 },
	{ GLAMO_REG_MEM_TIMING2,	0x0010 }, /* Taa = 3 MCLK */
	{ GLAMO_REG_MEM_TIMING3,	0x0000 },
	{ GLAMO_REG_MEM_TIMING4,	0x0000 }, /* CE1# delay fall/rise */
	{ GLAMO_REG_MEM_TIMING5,	0x0000 }, /* UB# LB# */
	{ GLAMO_REG_MEM_TIMING6,	0x0000 }, /* OE# */
	{ GLAMO_REG_MEM_TIMING7,	0x0000 }, /* WE# */
	{ GLAMO_REG_MEM_TIMING8,	0x1002 }, /* MCLK delay, was 0x1000 */
	{ GLAMO_REG_MEM_TIMING9,	0x6006 },
	{ GLAMO_REG_MEM_TIMING10,	0x00ff },
	{ GLAMO_REG_MEM_TIMING11,	0x0001 },
	{ GLAMO_REG_MEM_POWER1,		0x0020 },
	{ GLAMO_REG_MEM_POWER2,		0x0000 },
	{ GLAMO_REG_MEM_DRAM1,		0x0000 },
	{ GLAMO_SCRIPT_WAIT,		     1 },
	{ GLAMO_REG_MEM_DRAM1,		0xc100 },
	{ GLAMO_SCRIPT_WAIT,		     1 },
	{ GLAMO_REG_MEM_DRAM1,		0xe100 },
	{ GLAMO_REG_MEM_DRAM2,		0x01d6 },
	{ GLAMO_REG_CLOCK_MEMORY,	0x000b },
};

/* Find out if we can support this version of the Glamo chip */
static int __devinit glamo_supported(struct glamo_core *glamo)
{
	uint16_t dev_id, rev_id;

	dev_id = __reg_read(glamo, GLAMO_REG_DEVICE_ID);
	rev_id = __reg_read(glamo, GLAMO_REG_REVISION_ID);

	switch (dev_id) {
	case 0x3650:
		switch (rev_id) {
		case GLAMO_CORE_REV_A2:
			break;
		case GLAMO_CORE_REV_A0:
		case GLAMO_CORE_REV_A1:
		case GLAMO_CORE_REV_A3:
			dev_warn(&glamo->pdev->dev, "untested core revision "
				 "%04x, your mileage may vary\n", rev_id);
			break;
		default:
			dev_warn(&glamo->pdev->dev, "unknown glamo revision "
				 "%04x, your mileage may vary\n", rev_id);
		}
		break;
	default:
		dev_err(&glamo->pdev->dev, "unsupported Glamo device %04x\n",
			dev_id);
		return 0;
	}

	dev_dbg(&glamo->pdev->dev, "Detected Glamo core %04x Revision %04x "
		 "(%uHz CPU / %uHz Memory)\n", dev_id, rev_id,
		 glamo_pll_rate(glamo, GLAMO_PLL1),
		 glamo_pll_rate(glamo, GLAMO_PLL2));

	return 1;
}

static int __devinit glamo_probe(struct platform_device *pdev)
{
	int ret = 0, n, irq, irq_base;
	struct glamo_core *glamo;
	struct resource *mem;

	glamo = kmalloc(GFP_KERNEL, sizeof(*glamo));
	if (!glamo)
		return -ENOMEM;

	for (n = 0; n < __NUM_GLAMO_ENGINES; n++)
		glamo->engine_state[n] = GLAMO_ENGINE_DISABLED;

	spin_lock_init(&glamo->lock);

	glamo->pdev = pdev;
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	glamo->irq = platform_get_irq(pdev, 0);
	glamo->pdata = pdev->dev.platform_data;
	glamo->slowed_divider = 0xFF;

	if (glamo->irq < 0) {
		ret = glamo->irq;
		dev_err(&pdev->dev, "Failed to get platform irq: %d\n", ret);
		goto err_free;
	}

	if (!mem) {
		dev_err(&pdev->dev, "Failed to get platform memory\n");
		ret = -ENOENT;
		goto err_free;
	}

	if (!glamo->pdata) {
		dev_err(&pdev->dev, "Missing platform data\n");
		ret = -ENOENT;
		goto err_free;
	}

	irq_base = irq_alloc_descs(glamo->pdata->irq_base, 0, GLAMO_NR_IRQS, 0);
	if (irq_base < 0) {
		dev_err(&pdev->dev, "Failed to allocate irqs: %d\n", irq_base);
		goto err_free;
	}
	glamo->irq_base = irq_base;
	dev_notice(&pdev->dev, "Allocated irqs from %d to %d\n", glamo->irq_base,
	           glamo->irq_base + GLAMO_NR_IRQS - 1);


	/* only request the generic, hostbus and memory controller registers */
	glamo->mem = request_mem_region(mem->start, GLAMO_REGOFS_VIDCAP,
					pdev->name);

	if (!glamo->mem) {
		dev_err(&pdev->dev, "Failed to request io memory region\n");
		ret = -ENOENT;
		goto err_irq_free_descs;
	}

	glamo->base = ioremap(glamo->mem->start, resource_size(glamo->mem)+0x1100);
	if (!glamo->base) {
		dev_err(&pdev->dev, "Failed to ioremap() memory region\n");
		goto err_release_mem_region;
	}

	/* confirm it isn't insane version */
	if (!glamo_supported(glamo)) {
		dev_err(&pdev->dev,
			"This version of the Glamo is not supported\n");
		goto err_iounmap;
	}

	platform_set_drvdata(pdev, glamo);

	/* debugfs */
	glamo_init_debugfs(glamo);

	/* init the chip with canned register set */
	glamo_run_script(glamo, glamo_init_script,
			 ARRAY_SIZE(glamo_init_script), 1);

	/*
	 * finally set the mfd interrupts up
	 */
	for (irq = irq_base; irq < irq_base + GLAMO_NR_IRQS; ++irq) {
		irq_set_chip_data(irq, glamo);
		irq_set_chip(irq, &glamo_irq_chip);
		__irq_set_handler(irq, handle_level_irq, 0, NULL);

#ifdef CONFIG_ARM
		_set_irq_flags(irq, IRQF_VALID);
#else
		irq_set_noprobe(irq);
#endif
	}

	ret = request_any_context_irq(glamo->irq, glamo_irq_handler,
				IRQF_TRIGGER_FALLING, "glamo3362:irqchip", glamo);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request irq: %d\n", ret);
		goto err_free_irqs;
	}

	//glamo_cells[GLAMO_CELL_FB].platform_data = glamo->pdata->fb_data;
	//glamo_cells[GLAMO_CELL_FB].pdata_size = sizeof(glamo->pdata->fb_data);
	glamo_cells[GLAMO_CELL_MMC].platform_data = glamo->pdata->mmc_data;
	glamo_cells[GLAMO_CELL_MMC].pdata_size = sizeof(glamo->pdata->mmc_data);
	glamo_cells[GLAMO_CELL_GPIO].platform_data = glamo->pdata->gpio_data;
	glamo_cells[GLAMO_CELL_GPIO].pdata_size = sizeof(glamo->pdata->gpio_data);
	
	ret = mfd_add_devices(&pdev->dev, pdev->id, (struct mfd_cell *)glamo_cells,
				ARRAY_SIZE(glamo_cells), mem, glamo->irq_base);

	if (ret) {
		dev_err(&pdev->dev, "Failed to add child devices: %d\n", ret);
		goto err_free_irq;
	}

	dev_info(&glamo->pdev->dev, "Glamo core PLL1: %uHz, PLL2: %uHz\n",
		 glamo_pll_rate(glamo, GLAMO_PLL1),
		 glamo_pll_rate(glamo, GLAMO_PLL2));

	return 0;

err_free_irq:
	free_irq(glamo->irq, glamo);
err_free_irqs:
	for (irq = irq_base; irq < irq_base + GLAMO_NR_IRQS; ++irq) {
		irq_set_chip(irq, NULL);
#ifdef CONFIG_ARM
		_set_irq_flags(irq, 0);
#else
		irq_set_probe(irq);
#endif
		irq_set_chip_data(irq, NULL);
	}
err_iounmap:
	iounmap(glamo->base);
err_release_mem_region:
	release_mem_region(glamo->mem->start, resource_size(glamo->mem));
err_irq_free_descs:
	irq_free_descs(irq_base, GLAMO_NR_IRQS);
err_free:
	platform_set_drvdata(pdev, NULL);
	kfree(glamo);

	return ret;
}

static int __devexit glamo_remove(struct platform_device *pdev)
{
	struct glamo_core *glamo = platform_get_drvdata(pdev);
	int irq;
	int irq_base = glamo->irq_base;

	glamo_exit_debugfs(glamo);

	mfd_remove_devices(&pdev->dev);

	free_irq(glamo->irq, glamo);

	for (irq = irq_base; irq < irq_base + GLAMO_NR_IRQS; ++irq) {
#ifdef CONFIG_ARM
		_set_irq_flags(irq, 0);
#else
		irq_set_noprobe();
#endif
		irq_set_chip(irq, NULL);
		irq_set_chip_data(irq, NULL);
	}

	platform_set_drvdata(pdev, NULL);
	iounmap(glamo->base);
	release_mem_region(glamo->mem->start, resource_size(glamo->mem));
	irq_free_descs(irq_base, GLAMO_NR_IRQS);
	kfree(glamo);

	return 0;
}

#ifdef CONFIG_PM
#if 0
static struct glamo_script glamo_resume_script[] = {

	{ GLAMO_REG_PLL_GEN1,		0x05db },	/* 48MHz */
	{ GLAMO_REG_PLL_GEN3,		0x0aba },	/* 90MHz */
	{ GLAMO_REG_DFT_GEN6, 1 },
		{ 0xfffe, 100 },
		{ 0xfffd, 0 },
	{ 0x200,	0x0e03 },

	/*
	 * b9 of this register MUST be zero to get any interrupts on INT#
	 * the other set bits enable all the engine interrupt sources
	 */
	{ GLAMO_REG_IRQ_ENABLE,		0x01ff },
	{ GLAMO_REG_CLOCK_HOST,		0x0018 },
	{ GLAMO_REG_CLOCK_GEN5_1, 0x18b1 },

	{ GLAMO_REG_MEM_DRAM1,		0x0000 },
		{ 0xfffe, 1 },
	{ GLAMO_REG_MEM_DRAM1,		0xc100 },
		{ 0xfffe, 1 },
	{ GLAMO_REG_MEM_DRAM1,		0xe100 },
	{ GLAMO_REG_MEM_DRAM2,		0x01d6 },
	{ GLAMO_REG_CLOCK_MEMORY,	0x000b },
};
#endif

#if 0
static void glamo_power(struct glamo_core *glamo)
{
	unsigned long flags;

	spin_lock_irqsave(&glamo->lock, flags);

	/*
Power management
static const REG_VALUE_MASK_TYPE reg_powerOn[] =
{
	{ REG_GEN_DFT6,	    REG_BIT_ALL,    REG_DATA(1u << 0)		},
	{ REG_GEN_PLL3,	    0u,		    REG_DATA(1u << 13)		},
	{ REG_GEN_MEM_CLK,  REG_BIT_ALL,    REG_BIT_EN_MOCACLK		},
	{ REG_MEM_DRAM2,    0u,		    REG_BIT_EN_DEEP_POWER_DOWN	},
	{ REG_MEM_DRAM1,    0u,		    REG_BIT_SELF_REFRESH	}
};

static const REG_VALUE_MASK_TYPE reg_powerStandby[] =
{
	{ REG_MEM_DRAM1,    REG_BIT_ALL,    REG_BIT_SELF_REFRESH    },
	{ REG_GEN_MEM_CLK,  0u,		    REG_BIT_EN_MOCACLK	    },
	{ REG_GEN_PLL3,	    REG_BIT_ALL,    REG_DATA(1u << 13)	    },
	{ REG_GEN_DFT5,	    REG_BIT_ALL,    REG_DATA(1u << 0)	    }
};

static const REG_VALUE_MASK_TYPE reg_powerSuspend[] =
{
	{ REG_MEM_DRAM2,    REG_BIT_ALL,    REG_BIT_EN_DEEP_POWER_DOWN  },
	{ REG_GEN_MEM_CLK,  0u,		    REG_BIT_EN_MOCACLK		},
	{ REG_GEN_PLL3,	    REG_BIT_ALL,    REG_DATA(1u << 13)		},
	{ REG_GEN_DFT5,	    REG_BIT_ALL,    REG_DATA(1u << 0)		}
};
*/
	switch (new_state) {
	case GLAMO_POWER_ON:

		/*
		 * glamo state on resume is nondeterministic in some
		 * fundamental way, it has also been observed that the
		 * Glamo reset pin can get asserted by, eg, touching it with
		 * a scope probe.  So the only answer is to roll with it and
		 * force an external reset on the Glamo during resume.
		 */


		break;

	case GLAMO_POWER_SUSPEND:

		break;
	}
	spin_unlock_irqrestore(&glamo->lock, flags);
}
#endif

static int glamo_suspend(struct device *dev)
{
	struct glamo_core *glamo = dev_get_drvdata(dev);
	int n;

	spin_lock(&glamo->lock);

	glamo->saved_irq_mask = __reg_read(glamo, GLAMO_REG_IRQ_ENABLE);

	/* nuke interrupts */
	__reg_write(glamo, GLAMO_REG_IRQ_ENABLE, 0x200);

	/* take down each engine before we kill mem and pll */
	for (n = 0; n < __NUM_GLAMO_ENGINES; n++) {
		if (glamo->engine_state[n] != GLAMO_ENGINE_DISABLED)
			__glamo_engine_disable(glamo, n);
	}

	/* enable self-refresh */

	__reg_write(glamo, GLAMO_REG_MEM_DRAM1,
				GLAMO_MEM_DRAM1_EN_DRAM_REFRESH |
				GLAMO_MEM_DRAM1_EN_GATE_CKE |
				GLAMO_MEM_DRAM1_SELF_REFRESH |
				GLAMO_MEM_REFRESH_COUNT);
	__reg_write(glamo, GLAMO_REG_MEM_DRAM1,
				GLAMO_MEM_DRAM1_EN_MODEREG_SET |
				GLAMO_MEM_DRAM1_EN_DRAM_REFRESH |
				GLAMO_MEM_DRAM1_EN_GATE_CKE |
				GLAMO_MEM_DRAM1_SELF_REFRESH |
				GLAMO_MEM_REFRESH_COUNT);

	/* force RAM into deep powerdown */
	__reg_write(glamo, GLAMO_REG_MEM_DRAM2,
				GLAMO_MEM_DRAM2_DEEP_PWRDOWN |
				(7 << 6) | /* tRC */
				(1 << 4) | /* tRP */
				(1 << 2) | /* tRCD */
				2); /* CAS latency */

	/* disable clocks to memory */
	__reg_write(glamo, GLAMO_REG_CLOCK_MEMORY, 0);

	/* all dividers from OSCI */
	__reg_set_bit_mask(glamo, GLAMO_REG_CLOCK_GEN5_1, 0x400, 0x400);

	/* PLL2 into bypass */
	__reg_set_bit_mask(glamo, GLAMO_REG_PLL_GEN3, 1 << 12, 1 << 12);

	__reg_write(glamo, GLAMO_BASIC_MMC_EN_TCLK_DLYA1, 0x0e00);

	/* kill PLLS 1 then 2 */
	__reg_write(glamo, GLAMO_REG_DFT_GEN5, 0x0001);
	__reg_set_bit_mask(glamo, GLAMO_REG_PLL_GEN3, 1 << 13, 1 << 13);

	spin_unlock(&glamo->lock);

	return 0;
}

static int glamo_resume(struct device *dev)
{
	struct glamo_core *glamo = dev_get_drvdata(dev);
	int n;

	(glamo->pdata->glamo_external_reset)(0);
	udelay(10);
	(glamo->pdata->glamo_external_reset)(1);
	mdelay(5);

	spin_lock(&glamo->lock);

	glamo_run_script(glamo, glamo_init_script,
			 ARRAY_SIZE(glamo_init_script), 0);


	for (n = 0; n < __NUM_GLAMO_ENGINES; n++) {
		switch (glamo->engine_state[n]) {
		case GLAMO_ENGINE_SUSPENDED:
			__glamo_engine_suspend(glamo, n);
			break;
		case GLAMO_ENGINE_ENABLED:
			__glamo_engine_enable(glamo, n);
			break;
		default:
			break;
		}
	}

	__reg_write(glamo, GLAMO_REG_IRQ_ENABLE, glamo->saved_irq_mask);

	spin_unlock(&glamo->lock);

	return 0;
}

static const struct dev_pm_ops glamo_pm_ops = {
	.suspend    = glamo_suspend,
	.resume     = glamo_resume,
	.poweroff   = glamo_suspend,
	.restore    = glamo_resume,
};

#define GLAMO_PM_OPS (&glamo_pm_ops)

#else
#define GLAMO_PM_OPS NULL
#endif

static struct platform_driver glamo_driver = {
	.probe		= glamo_probe,
	.remove		= __devexit_p(glamo_remove),
	.driver		= {
		.name	= "glamo3362",
		.owner	= THIS_MODULE,
		.pm	= GLAMO_PM_OPS,
	},
};

static int __init glamo_init(void)
{
	return platform_driver_register(&glamo_driver);
}
module_init(glamo_init);

static void __exit glamo_exit(void)
{
	platform_driver_unregister(&glamo_driver);
}
module_exit(glamo_exit);

MODULE_AUTHOR("Harald Welte <laforge@openmoko.org>");
MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_DESCRIPTION("Smedia Glamo 3362 core/resource driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:glamo3362");
