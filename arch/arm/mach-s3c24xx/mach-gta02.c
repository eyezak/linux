/*
 * linux/arch/arm/mach-s3c2442/mach-gta02.c
 *
 * S3C2442 Machine Support for Openmoko GTA02 / FreeRunner.
 *
 * Copyright (C) 2006-2009 by Openmoko, Inc.
 * Authors: Harald Welte <laforge@openmoko.org>
 *          Andy Green <andy@openmoko.org>
 *          Werner Almesberger <werner@openmoko.org>
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

#define DH_PRESENT

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/usb.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/serial_core.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_gpio.h>
#include <linux/spi/spi_bitbang.h>

#include <linux/mmc/host.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>
#include <linux/io.h>

#include <linux/i2c.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/userspace-consumer.h>
#include <linux/leds_pwm.h>

#include <linux/mfd/pcf50633/platform.h>
#include <linux/mfd/pcf50633/pmic.h>

#include <linux/power/bq27x00_battery.h>

#ifdef DH_PRESENT
#include <linux/jbt6k74.h>
#include <linux/glamofb.h>
#include <linux/mfd/glamo.h>
#else
#include <linux/fb.h>
#endif

#include <linux/input.h>
#include <linux/gpio_keys.h>

#include <linux/lis3lv02d.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <asm/irq.h>
#include <asm/mach-types.h>

#include <mach/regs-irq.h>
#include <mach/regs-gpio.h>
#include <mach/regs-gpioj.h>
#include <mach/regs-mem.h>
#include <mach/fb.h>

#include <mach/hardware.h>
#include <mach/gta02.h>

#ifdef DH_PRESENT
#include <linux/power/fiq-hdq.h>
#endif

#include <plat/usb-control.h>
#include <plat/regs-serial.h>
#include <plat/nand.h>
#include <plat/devs.h>
#include <plat/cpu.h>
#include <plat/pm.h>
#include <plat/udc.h>
#include <plat/gpio-cfg.h>
#include <plat/iic.h>
#include <plat/ts.h>
#include <plat/fiq.h>

#include "common.h"

#define GTA02_CHARGER_CONFIGURE_TIMEOUT ((3000 * HZ) / 1000)

#define GTA02_PCF50633_IRQ_BASE	(IRQ_S3C2440_AC97 + 1)
#define GTA02_GLAMO_IRQ_BASE	(GTA02_PCF50633_IRQ_BASE + PCF50633_NUM_IRQS)


static struct pcf50633_ops *gta02_pcf_ops;
static struct pcf50633_mbc_ops *gta02_pcf_mbc_ops;
static struct pcf50633_bl_ops *gta02_pcf_bl_ops;
static struct device *gta02_fiq_hdq_dev;

static struct pcf50633_platform_data gta02_pcf_pdata;

/*	UARTs  */
#define UCON (S3C2410_UCON_DEFAULT | S3C2443_UCON_RXERR_IRQEN)
#define ULCON (S3C2410_LCON_CS8 | S3C2410_LCON_PNONE | S3C2410_LCON_STOPB)
#define UFCON (S3C2410_UFCON_RXTRIG8 | S3C2410_UFCON_FIFOMODE)

static struct s3c2410_uartcfg gta02_uartcfgs[] = {
	[0] = {
		.hwport		= 0,
		.flags		= 0,
		.ucon		= UCON,
		.ulcon		= ULCON,
		.ufcon		= UFCON,
	},
	[1] = {
		.hwport		= 1,
		.flags		= 0,
		.ucon		= UCON,
		.ulcon		= ULCON,
		.ufcon		= UFCON,
	},
	[2] = {
		.hwport		= 2,
		.flags		= 0,
		.ucon		= UCON,
		.ulcon		= ULCON,
		.ufcon		= UFCON,
	},
};

/*	Delta Bluetooth DFBM-CS320  */
static struct platform_device gta02_dfbmcs320_device = {
	.name = "dfbmcs320",
};

/*	I2C Bus  */
static struct i2c_board_info gta02_i2c_devs[] __initdata = {
	{
		I2C_BOARD_INFO("pcf50633", 0x73),
		.irq = GTA02_IRQ_PCF50633,
		.platform_data = &gta02_pcf_pdata,
	},
	{
		I2C_BOARD_INFO("wm8753", 0x1a),
	},
};

/*	Resume Reason  */
struct platform_device gta02_resume_reason_device = {
	.name 		= "neo1973-resume",
	.num_resources	= 0,
	//.dev.release = (void (*)(struct device *)) kfree,
};

/*	NOR Flash  */
#define GTA02_FLASH_BASE	0x18000000 /* GCS3 */
#define GTA02_FLASH_SIZE	0x200000 /* 2MBytes */

static struct physmap_flash_data gta02_nor_flash_data = {
	.width		= 2,
};

static struct resource gta02_nor_flash_resource = {
	.start		= GTA02_FLASH_BASE,
	.end		= GTA02_FLASH_BASE + GTA02_FLASH_SIZE - 1,
	.flags		= IORESOURCE_MEM,
};

static struct platform_device gta02_nor_flash = {
	.name				= "physmap-flash",
	.id					= 0,
	.dev.platform_data	= &gta02_nor_flash_data,
	.resource			= &gta02_nor_flash_resource,
	.num_resources		= 1,
};

static struct nand_ecclayout gta02_nand_layout = {
	.eccbytes = 8,
};

/*	NAND Flash  */
static struct s3c2410_nand_set gta02_nand_sets[] = {
	[0] = {
		/*
		 * This name is also hard-coded in the boot loaders, so
		 * changing it would would require all users to upgrade
		 * their boot loaders, some of which are stored in a NOR
		 * that is considered to be immutable.
		 */
		.name		= "neo1973-nand",
		.nr_chips	= 1,
		.flash_bbt	= 1,
//		.eec_layout = &gta02_nand_layout;
	},
};

/*
 * Choose a set of timings derived from S3C@2442B MCP54
 * data sheet (K5D2G13ACM-D075 MCP Memory).
 */
static struct s3c2410_platform_nand __initdata gta02_nand_info = {
	.tacls		= 0,
	.twrph0		= 25,
	.twrph1		= 15,
	.nr_sets	= ARRAY_SIZE(gta02_nand_sets),
	.sets		= gta02_nand_sets,
};


/* USB: udc */
static void gta02_udc_vbus_draw(unsigned int ma)
{
	printk("mach-gta02: vbus draw %u mA\n", ma);
	if (!gta02_pcf_mbc_ops)
		return;

	gta02_pcf_mbc_ops->vbus_draw(gta02_pcf_mbc_ops, ma);
}
static void gta02_udc_command(enum s3c2410_udc_cmd_e cmd)
{
	int value;
	
	switch (cmd) {
	case S3C2410_UDC_P_ENABLE:
		value = 1;
		break;
	case S3C2410_UDC_P_DISABLE:
		value = 0;
		break;
	default:
		return;
	};
	
	s3c2410_gpio_setpin(GTA02_GPIO_USB_PULLUP, value);
}

/* Get PMU to set USB current limit accordingly. */
static struct s3c2410_udc_mach_info gta02_udc_cfg __initdata = {
	.vbus_draw	= gta02_udc_vbus_draw,
	//.pullup_pin = GTA02_GPIO_USB_PULLUP,
	.udc_command = gta02_udc_command
};

/* USB: hcd  */
static struct s3c2410_hcd_info gta02_usb_info __initdata = {
	.port[0]	= {
		.flags	= S3C_HCDFLG_USED,
	},
	.port[1]	= {
		.flags	= 0,
	},
};

/* Touchscreen */
static struct s3c2410_ts_mach_info gta02_ts_info = {
	.delay			= 10000, /* src has 1000 */
	.presc			= 0xff, /* slow as we can go */
	.oversampling_shift	= 0, /* upstream has 2 we have 0 */
};

/* Buttons */
static struct gpio_keys_button gta02_buttons[] = {
	{
		.gpio = GTA02_GPIO_AUX_KEY,
		.code = KEY_PHONE,
		.desc = "Aux",
		.type = EV_KEY,
		.debounce_interval = 100,
	},
	{
		.gpio = GTA02_GPIO_HOLD_KEY,
		.code = KEY_PAUSE,
		.desc = "Hold",
		.type = EV_KEY,
		.debounce_interval = 100,
	},
};

static struct gpio_keys_platform_data gta02_buttons_pdata = {
	.buttons = gta02_buttons,
	.nbuttons = ARRAY_SIZE(gta02_buttons),
};

static struct platform_device gta02_buttons_device = {
	.name = "gpio-keys",
	.id = -1,
	.dev.platform_data = &gta02_buttons_pdata,
};

/*
    static inline int gta02_pwm_to_gpio(unsigned pwm_id)
    {
	    return S3C2410_GPB(pwm_id);
    }

    static int gta02_pwm_led_init(struct device *dev, struct led_pwm *led)
    {
	    int ret;
	    int gpio = gta02_pwm_to_gpio(led->pwm_id);

	    ret = gpio_request(gpio, dev_name(dev));
	    if (ret)
		    return ret;

	    gpio_direction_output(gpio, 0);

	    return 0;
    }

    static enum led_brightness gta02_pwm_led_notify(struct device *dev,
	    struct led_pwm *led, enum led_brightness brightness)
    {
	    int gpio = gta02_pwm_to_gpio(led->pwm_id);

	    if (brightness == led->max_brightness || brightness == 0) {
		    s3c2410_gpio_cfgpin(gpio, S3C2410_GPIO_OUTPUT);
		    gpio_set_value(gpio, brightness ? 1 : 0);

		    brightness = 0;
	    } else {
		    s3c2410_gpio_cfgpin(gpio, S3C2410_GPIO_SFN2);
	    }

	    return brightness;
    }

    static void gta02_pwm_led_exit(struct device *dev, struct led_pwm *led)
    {
	    gpio_free(gta02_pwm_to_gpio(led->pwm_id));
    }
*/

/* LEDs */
/* GPIO LEDs */
static struct gpio_led gta02_gpio_leds[] = {
	{
		.name   = "gta02-aux:red",
		.default_trigger = "nand-disk",
		.gpio   = GTA02_GPIO_AUX_LED,
	},
    {
        .name   = "gta02-power:orange",
        .default_trigger = "mmc0",
        .gpio  = GTA02_GPIO_PWR_LED1,
    }, {
        .name   = "gta02-power:blue",
        .default_trigger = "heartbeat",
        .gpio  = GTA02_GPIO_PWR_LED2,
    }, {
        .name   = "gta02::vibrator",
        .default_trigger = "none",
        .gpio  = GTA02_GPIO_VIBRATOR_ON,
    },
};

static struct gpio_led_platform_data gta02_gpio_leds_pdata = {
	.leds = gta02_gpio_leds,
	.num_leds = ARRAY_SIZE(gta02_gpio_leds),
};

static struct platform_device gta02_leds_device = {
	.name = "leds-gpio",
	.id   = -1,
	.dev.platform_data = &gta02_gpio_leds_pdata,
};

/* JBT6K74 display controller */
#ifdef __JBT6K74_H__
const static struct jbt6k74_platform_data jbt6k74_pdata = {
	.gpio_reset = GTA02_GPIO_GLAMO(4),
};
#else
const static struct {} jbt6k74_pdata;
#endif

/* SPI: Accelerometers attached to SPI of s3c244x */
static struct lis3lv02d_platform_data gta02_gsensor0_pdata = {
	.click_flags    = LIS3_CLICK_SINGLE_X | LIS3_CLICK_SINGLE_Y | LIS3_CLICK_SINGLE_Z,
	.irq_cfg        = LIS3_IRQ1_CLICK | LIS3_IRQ2_DISABLE,
	.wakeup_flags   = LIS3_WAKEUP_X_LO | LIS3_WAKEUP_X_HI |
					  LIS3_WAKEUP_Y_LO | LIS3_WAKEUP_Y_HI |
					  LIS3_WAKEUP_Z_LO | LIS3_WAKEUP_Z_HI,
	.wakeup_thresh  = 40,
	.click_thresh_x = 40,
	.click_thresh_y = 40,
	.click_thresh_z = 40,
	.id				= 0,
};

static struct lis3lv02d_platform_data gta02_gsensor1_pdata = {
	.click_flags    = LIS3_CLICK_SINGLE_X | LIS3_CLICK_SINGLE_Y | LIS3_CLICK_SINGLE_Z,
	.irq_cfg        = LIS3_IRQ1_CLICK | LIS3_IRQ2_DISABLE,
	.wakeup_flags   = LIS3_WAKEUP_X_LO | LIS3_WAKEUP_X_HI |
					  LIS3_WAKEUP_Y_LO | LIS3_WAKEUP_Y_HI |
					  LIS3_WAKEUP_Z_LO | LIS3_WAKEUP_Z_HI,
	.wakeup_thresh  = 40,
	.click_thresh_x = 40,
	.click_thresh_y = 40,
	.click_thresh_z = 40,
	.id				= 1,
};


/*	SPI BUS	*/
static struct spi_board_info gta02_spi_board_info[] = {
	{
		.modalias	= "jbt6k74",
		.platform_data	= &jbt6k74_pdata,
		.controller_data = (void *)GTA02_GPIO_GLAMO(12),
		/* irq */
		.max_speed_hz	= 100 * 1000,
		.bus_num	= 2,
		.chip_select	= 0
	},
	{
		.modalias			= "lis3lv02d_spi",
		.max_speed_hz		= 1000000,
		.bus_num			= 3,
		.chip_select		= 0,
		.controller_data	= (void *) S3C2410_GPD(12),
		.platform_data		= &gta02_gsensor0_pdata,
		.irq				= GTA02_IRQ_GSENSOR_1,
	},
/*	{
		.modalias			= "lis3lv02d_spi",
		.max_speed_hz		= 1000000,
		.bus_num			= 3,
		.chip_select		= 1,
		.controller_data	= (void *) S3C2410_GPD(13),
		.platform_data		= &gta02_gsensor1_pdata,
		.irq				= GTA02_IRQ_GSENSOR_2,
	},*/
};

/*	PM-BT  */
static struct platform_device gta02_pm_bt_dev = {
	.name = "gta02-pm-bt",
	.id = -1,
};
/*	PM-GPS  */
static struct platform_device gta02_pm_gps_dev = {
	.name = "gta02-pm-gps",
	.id = -1,
};
/*	PM-WLAN  */
static struct platform_device gta02_pm_wlan_dev = {
	.name = "gta02-pm-wlan",
	.id = -1,
};

/*	PM-USBHOST  */
static struct platform_device gta02_pm_usbhost_dev = {
	.name = "gta02-pm-usbhost",
	.id = -1,
};
static struct regulator_consumer_supply usbhost_consumers[] = {
	REGULATOR_SUPPLY("USBHOST", "gta02-pm-usbhost"),
};
static struct regulator_init_data usbhost_supply_init_data = {
	.constraints = {
		.name = "USBHOST",
		.min_uV = 3700000,
		.max_uV = 3700000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = ARRAY_SIZE(usbhost_consumers),
	.consumer_supplies = usbhost_consumers,
};
static struct fixed_voltage_config usbhost_supply_config = {
	.supply_name = "USBHOST",
	.microvolts = 3700000,
	.gpio = GTA02_GPIO_PCF(PCF50633_GPO),
	.enable_high = 1,
	.init_data = &usbhost_supply_init_data,
};
static struct platform_device gta02_usbhost_supply_device = {
	.name = "reg-fixed-voltage",
	.id = 1,
	.dev.platform_data = &usbhost_supply_config,
};

/*	PM-GSM  */
static struct platform_device gta02_pm_gsm_dev = {
	.name = "gta02-pm-gsm",
	.id = -1,
};
static struct regulator_consumer_supply gsm_consumers[] = {
	REGULATOR_SUPPLY("GSM", "gta02-pm-gsm"),
};
static struct regulator_init_data gsm_supply_init_data = {
	.constraints = {
		.name = "GSM",
		.min_uV = 3700000,
		.max_uV = 3700000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = ARRAY_SIZE(gsm_consumers),
	.consumer_supplies = gsm_consumers,
};
static struct fixed_voltage_config gsm_supply_config = {
	.supply_name = "GSM",
	.microvolts = 3700000,
	.gpio = GTA02_GPIO_PCF(PCF50633_GPIO2),
	.enable_high = 1,
	.init_data = &gsm_supply_init_data,
};
static struct platform_device gta02_gsm_supply_device = {
	.name = "reg-fixed-voltage",
	.id = 0,
	.dev.platform_data = &gsm_supply_config,
};

/*	GLAMO-FB  */
static struct fb_videomode gta02_glamo_modes[] = {
	{
		.name = "480x640",
		.xres = 480,
		.yres = 640,
		.pixclock	= 40816,
		.left_margin	= 8,
		.right_margin	= 16,
		.upper_margin	= 2,
		.lower_margin	= 16,
		.hsync_len	= 8,
		.vsync_len	= 2,
		.vmode = FB_VMODE_NONINTERLACED,
	}, {
		.name = "240x320",
		.xres = 240,
		.yres = 320,
		.pixclock	= 40816,
		.left_margin	= 8,
		.right_margin	= 16,
		.upper_margin	= 2,
		.lower_margin	= 16,
		.hsync_len	= 8,
		.vsync_len	= 2,
		.vmode = FB_VMODE_NONINTERLACED,
	}
};
#ifdef _LINUX_GLAMOFB_H
static struct glamo_fb_platform_data gta02_glamo_fb_pdata = {
	.width  = 43,
	.height = 58,

	.num_modes = ARRAY_SIZE(gta02_glamo_modes),
	.modes = gta02_glamo_modes,
};
#else
static struct {} gta02_glamo_fb_pdata;
#endif

/*	GLAMO-MMC  */
/*
 * we crank down SD Card clock dynamically when GPS is powered
 */
static int gta02_glamo_mci_use_slow(void)
{
	/*	gta02_pm_gps_is_on();	*/
	return 0;
}

#ifdef __GLAMO_MFD_H
static struct glamo_mmc_platform_data gta02_glamo_mmc_pdata = {
	.glamo_mmc_use_slow = gta02_glamo_mci_use_slow,
};
#else
static struct {} gta02_glamo_mmc_pdata;
#endif

/*	GLAMO-GPIO  */
#ifdef __GLAMO_MFD_H
static struct glamo_gpio_platform_data gta02_glamo_gpio_pdata = {
	.base = GTA02_GPIO_GLAMO_BASE,
};
#else
static struct {} gta02_glamo_gpio_pdata;
#endif

/*	SPI-GPIO for glamo  */
struct spi_gpio_platform_data gta02_glamo_spigpio_pdata = {
	.sck = GTA02_GPIO_GLAMO(10),
	.mosi = GTA02_GPIO_GLAMO(11),
	.miso = GTA02_GPIO_GLAMO(5),
	.num_chipselect = 1,
};
static struct platform_device gta02_glamo_spigpio_dev = {
	.name = "spi_gpio",
	.id   = 2,
	.dev.platform_data = &gta02_glamo_spigpio_pdata,
//	.dev.release = kfree,
};

/*	GLAMO	*/
static void gta02_glamo_external_reset(int level)
{
	s3c2410_gpio_setpin(GTA02_GPIO_3D_RESET, level);
	s3c2410_gpio_cfgpin(GTA02_GPIO_3D_RESET, S3C2410_GPIO_OUTPUT);
}
static struct resource gta02_glamo_resources[] = {
	[0] = {
		.start	= S3C2410_CS1,
		.end	= S3C2410_CS1 + 0x1000000 - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= GTA02_IRQ_3D,
		.end	= GTA02_IRQ_3D,
		.flags	= IORESOURCE_IRQ,
	},
	[2] = {
		.start	= GTA02_GLAMO_IRQ_BASE,
		.end	= GTA02_GLAMO_IRQ_BASE,
		.flags	= IORESOURCE_IRQ,
	},
};
#ifdef __GLAMO_MFD_H
static struct glamo_platform_data gta02_glamo_pdata = {
	.fb_data    = &gta02_glamo_fb_pdata,
	.mmc_data   = &gta02_glamo_mmc_pdata,
	.gpio_data  = &gta02_glamo_gpio_pdata,

	.osci_clock_rate = 32768,
	.gpio_3dreset = GTA02_GPIO_3D_RESET,
	//.irq_base = GTA02_GLAMO_IRQ_BASE,

	.glamo_external_reset = gta02_glamo_external_reset,
};
#else
static struct {} gta02_glamo_pdata;
#endif
static struct platform_device gta02_glamo_dev = {
	.name		= "glamo3362",
	.id			= 0,
	.num_resources	= ARRAY_SIZE(gta02_glamo_resources),
	.resource	= gta02_glamo_resources,
	.dev.platform_data	= &gta02_glamo_pdata,
};

#define GTA02_SPI1_CS(cs)	(S3C2410_GPD(12) + (cs))
#define GTA02_SPI1_CLK		S3C2410_GPG(7)
#define GTA02_SPI1_MOSI		S3C2410_GPG(6)
#define GTA02_SPI1_MISO		S3C2410_GPG(5)

static struct spi_gpio_platform_data gta02_s3c24xx_spi_gpio1cfg = {
	.sck		= S3C2410_GPG(7),
	.mosi		= S3C2410_GPG(6),
	.miso		= S3C2410_GPG(5),
	.num_chipselect = 2,
};

static struct platform_device gta02_s3c24xx_spi_gpio1 = {
	.name		= "spi_gpio",
	.id		= 3,
	.dev.platform_data = &gta02_s3c24xx_spi_gpio1cfg,
};


static struct regulator_bulk_data gta02_gsensor_consumer_supply = {
	.supply		= "GSENSOR_3V3",
};

static struct regulator_userspace_consumer_data gta02_gsensor_consumer_data = {
	.name		= "lis3lv02d_spi",
	.num_supplies	= 1,
	.supplies	= &gta02_gsensor_consumer_supply,
};

static struct platform_device gta02_gsensor_userspace_consumer = {
	.name		= "reg-userspace-consumer",
	.id		= 0,
	.dev.platform_data = &gta02_gsensor_consumer_data,
};


/* HDQ */

static void gta02_hdq_fiq_handler(unsigned long irq, int disable)
{
	unsigned long intmask;
	unsigned long ackmask = (1 << (irq - FIQ_START));
	
	/* Are we interested in keeping the FIQ source alive ? */	
	if (disable) {
		/* Disable irq */
		intmask = __raw_readl(S3C2410_INTMSK);
		intmask |= ackmask;
		__raw_writel(intmask, S3C2410_INTMSK);
	}

	__raw_writel(ackmask, S3C2410_SRCPND);
}

static void gta02_hdq_fiq_kick(unsigned int irq)
{
	unsigned long flags;
	unsigned long intmask;
	/* we have to take care about FIQ because this modification is
	 * non-atomic, FIQ could come in after the read and before the
	 * writeback and its changes to the register would be lost
	 * (platform INTMSK mod code is taken care of already)
	 */
	local_save_flags(flags);
	local_fiq_disable();

	/* allow FIQs to resume */
	intmask = __raw_readl(S3C2410_INTMSK);
	intmask &= ~(1 << (irq - FIQ_START));
	__raw_writel(intmask, S3C2410_INTMSK);

	local_irq_restore(flags);

}

#ifdef __LINUX_HDQ_H__
struct fiq_hdq_platform_data gta02_hdq_platform_data = {
	.timer_id = 2,
	.timer_irq = IRQ_TIMER0 + 2,
	.gpio = GTA02v5_GPIO_HDQ,
	.gpio_output = S3C2410_GPIO_OUTPUT,
	.gpio_input = S3C2410_GPIO_INPUT,

    .gpio_dir_out = s3c2410_gpio_cfgpin,
    .gpio_dir_in = s3c2410_gpio_cfgpin,
    .gpio_set = s3c2410_gpio_setpin,
    .gpio_get = s3c2410_gpio_getpin,

	.set_fiq = s3c24xx_set_fiq,
	.kick_fiq = gta02_hdq_fiq_kick,
	.handle_fiq = gta02_hdq_fiq_handler,
};
#else
static struct {} gta02_hdq_platform_data;
#endif

struct platform_device gta02_hdq_device = {
	.name = "fiq-hdq",
	.id = -1,
	.dev = {
		.parent =  &s3c_device_timer[2].dev,
		.platform_data = &gta02_hdq_platform_data
	}
};

static int gta02_hdq_read(struct device *dev, unsigned int addr)
{
	struct fiq_hdq_ops *ops;
	
	if (gta02_fiq_hdq_dev == 0)
		return 0; // ENOENT ENXIO EPERM EFAULT EBUSY ENODEV
	
	ops = dev_get_drvdata(gta02_fiq_hdq_dev);
	if (ops == 0)
		return 0;
	
	return ops->read(gta02_fiq_hdq_dev, addr);
}

/* BQ27000 Battery */
#ifdef __LINUX_BQ27X00_BATTERY_H__
static struct bq27000_platform_data bq27000_pdata = {
        .name = "battery",
        .read = gta02_hdq_read,
};
#else
static struct {} bq27000_pdata;
#endif

static struct platform_device bq27000_battery_device = {
        .name = "bq27000-battery",
        .dev.platform_data = &bq27000_pdata,
};

/*	PCF50633  */

/*
 * On GTA02 the 1A charger features a 48K resistor to 0V on the ID pin.
 * We use this to recognize that we can pull 1A from the USB socket.
 *
 * These constants are the measured pcf50633 ADC levels with the 1A
 * charger / 48K resistor, and with no pulldown resistor.
 */

#define ADC_NOM_CHG_DETECT_1A 6
#define ADC_NOM_CHG_DETECT_USB 43

/*
static int gta02_get_charger_online_status(void)
{
	struct pcf50633_mbc_ops *mbc = gta02_pcf_mbc_ops;

	return mbc->get_status(mbc) & PCF50633_MBC_USB_ONLINE;
}

static int gta02_get_charger_active_status(void)
{
	struct pcf50633_mbc_ops *mbc = gta02_pcf_mbc_ops;

	return mbc->get_status(mbc) & PCF50633_MBC_USB_ACTIVE;
}
*/

static void gta02_restart(char mode, const char *cmd)
{
	/*  ensure backlight goes off before we restart  */
	if (gta02_pcf_bl_ops && gta02_pcf_bl_ops->set_power)
		gta02_pcf_bl_ops->set_power(gta02_pcf_bl_ops, 4);

	s3c244x_restart(mode, cmd);
}

static void gta02_poweroff(void)
{	
	if (gta02_pcf_ops && gta02_pcf_ops->shutdown)
		gta02_pcf_ops->shutdown(gta02_pcf_ops);
	
	gta02_restart('h', NULL);
}

static struct regulator_consumer_supply auto_consumers[] = {
	REGULATOR_SUPPLY("vdd", "s3c24xx-adc"),
};
static struct regulator_consumer_supply hcldo_consumers[] = {
	REGULATOR_SUPPLY("SD_3V3", "glamo-mci.0"),
};
static struct regulator_consumer_supply gsensor_consumers[] = {
	REGULATOR_SUPPLY("GSENSOR_3V3", "reg-userspace-consumer.0"),
};
static struct regulator_consumer_supply audio_codec_consumers[] = {
	REGULATOR_SUPPLY("CODEC_3V3", "wm8753"),
};
static struct regulator_consumer_supply bluetooth_consumers[] = {
	REGULATOR_SUPPLY("BT_3V2", "gta02-pm-bt"),
};
static struct regulator_consumer_supply gps_consumers[] = {
	REGULATOR_SUPPLY("RF_3V", "gta02-pm-gps"),
};
static struct regulator_consumer_supply lcm_consumers[] = {
	REGULATOR_SUPPLY("VDC", "spi2.0"),
	REGULATOR_SUPPLY("VDDIO", "spi2.0"),
};

static char *gta02_batteries[] = {
	"battery",
};

static struct pcf50633_platform_data gta02_pcf_pdata = {
	.backlight_data = {
		.default_brightness = 0x2f,
		.default_brightness_limit = 0x3f,
		.ramp_time = 5,
	},

	.mbc_data = {
		.batteries = gta02_batteries,
		.num_batteries = ARRAY_SIZE(gta02_batteries),

		.reference_current_ma = 1000,
		.delay = GTA02_CHARGER_CONFIGURE_TIMEOUT,
		.adc_channel = 0,
	},
	
	.adc_data = {
		.channel_mask = 0x9,
		.channels = {
			[0] = {
				.c1flags = PCF50633_ADCC1_MUX_ADCIN1 | PCF50633_ADCC1_AVERAGE_16, 
				.c3flags = PCF50633_ADCC3_ACCSW_EN, /* enable biasing */
			},
			[3] = {
				.c1flags = PCF50633_ADCC1_MUX_BATSNS_RES | PCF50633_ADCC1_AVERAGE_16,
				.m = 6,
			},
		}
	},
	
	.force_shutdown_timeout = 8,

	.gpio_base = GTA02_GPIO_PCF_BASE,
	.irq_base = GTA02_PCF50633_IRQ_BASE,

	.resumers = {
		[0] =	0x0, PCF50633_INT1_USBINS |
			PCF50633_INT1_USBREM |
			PCF50633_INT1_ALARM,
		[1] =	PCF50633_INT2_ONKEYF,
		[2] =	PCF50633_INT3_ONKEY1S,
		[3] =	PCF50633_INT4_LOWSYS |
			PCF50633_INT4_LOWBAT |
			PCF50633_INT4_HIGHTMP,
	},


	.reg_init_data = {
		[PCF50633_REGULATOR_AUTO] = {
			.constraints = {
			    .name = "auto",
				.min_uV = 3300000,
				.max_uV = 3300000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.always_on = 1,
				.apply_uV = 1,
			},
			.num_consumer_supplies = ARRAY_SIZE(auto_consumers),
			.consumer_supplies = auto_consumers,
		},
		[PCF50633_REGULATOR_DOWN1] = {
			.constraints = {
    			.name = "core",
				.min_uV = 1300000,
				.max_uV = 1600000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
						REGULATOR_CHANGE_STATUS,
				.always_on = 0,
				.apply_uV = 1,
			},
		},
		[PCF50633_REGULATOR_DOWN2] = {
			.constraints = {
			    .name = "io",
				.min_uV = 1800000,
				.max_uV = 1800000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.apply_uV = 1,
				.always_on = 1,
			},
		},
		[PCF50633_REGULATOR_HCLDO] = {
			.constraints = {
			    .name = "glamo-mci",
				.min_uV = 2000000,
				.max_uV = 3300000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
						REGULATOR_CHANGE_STATUS,
			},
			.num_consumer_supplies = ARRAY_SIZE(hcldo_consumers),
			.consumer_supplies = hcldo_consumers,
		},
		[PCF50633_REGULATOR_LDO1] = {
			.constraints = {
			    .name = "gsensor",
				.min_uV = 3300000,
				.max_uV = 3300000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.valid_ops_mask = REGULATOR_CHANGE_STATUS,
				.apply_uV = 1,
			},
			.num_consumer_supplies = ARRAY_SIZE(gsensor_consumers),
			.consumer_supplies = gsensor_consumers,
		},
		[PCF50633_REGULATOR_LDO2] = {
			.constraints = {
    			.name = "audio-codec",
				.min_uV = 3300000,
				.max_uV = 3300000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.apply_uV = 1,
			},
			.num_consumer_supplies = ARRAY_SIZE(audio_codec_consumers),
			.consumer_supplies = audio_codec_consumers,
		},
		[PCF50633_REGULATOR_LDO3] = {
			.constraints = {
			    .name = "expansion",
				.min_uV = 3000000,
				.max_uV = 3000000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.apply_uV = 1,
			},
		},
		[PCF50633_REGULATOR_LDO4] = {
			.constraints = {
			    .name = "bluetooth",
				.min_uV = 3200000,
				.max_uV = 3200000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.valid_ops_mask = REGULATOR_CHANGE_STATUS,
				.apply_uV = 1,
			},
			.num_consumer_supplies = ARRAY_SIZE(bluetooth_consumers),
			.consumer_supplies = bluetooth_consumers,
		},
		[PCF50633_REGULATOR_LDO5] = {
			.constraints = {
			    .name = "gps",
				.min_uV = 3000000,
				.max_uV = 3000000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.valid_ops_mask = REGULATOR_CHANGE_STATUS,
				.apply_uV = 1,
			},
			.num_consumer_supplies = ARRAY_SIZE(gps_consumers),
			.consumer_supplies = gps_consumers,
		},
		[PCF50633_REGULATOR_LDO6] = {
			.constraints = {
			    .name = "lcm",
				.min_uV = 3000000,
				.max_uV = 3000000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			},
			.num_consumer_supplies = ARRAY_SIZE(lcm_consumers),
			.consumer_supplies = lcm_consumers,
		},
		[PCF50633_REGULATOR_MEMLDO] = {
			.constraints = {
			    .name = "memldo",
				.min_uV = 1800000,
				.max_uV = 1800000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
			},
		},
	},
};

/*  BUS Device Registration  */
static int gta02_add_child_device(struct device *parent,
				  struct platform_device *child)
{
	if (parent)
		child->dev.parent = parent;
	return platform_device_register(child);
}

static int gta02_child_device_registered(struct device *dev)
{
	const char *devname = dev_name(dev);
	const char *busname = dev->bus->name;
	const char *drvname = dev->driver->name;

	if (strcmp(devname, "i2c") == 0)
		devname = to_i2c_client(dev)->name;
	printk("mach-gta02: %s device %s registered with %s\n", busname, devname, drvname);
	
	if (strcmp(devname, "glamo-gpio.0") == 0) {
		gta02_add_child_device(dev, &gta02_glamo_spigpio_dev);
	} else if (strcmp(devname, "0-0073") == 0) {
		/*  PMU	These guys DO need to be children of PMU. */
		gta02_pcf_ops = dev_to_pcf50633_ops(dev);
		gta02_add_child_device(dev, &bq27000_battery_device);
		gta02_add_child_device(dev, &gta02_resume_reason_device);
	} else if (strcmp(devname, "pcf50633-mbc") == 0) {
		gta02_pcf_mbc_ops = dev_get_drvdata(dev);
	} else if (strcmp(devname, "pcf50633-backlight") == 0) {
		gta02_pcf_bl_ops = dev_get_drvdata(dev);
	} else if (strcmp(devname, "pcf50633-gpio") == 0) {
		gta02_add_child_device(dev, &gta02_gsm_supply_device);
		gta02_add_child_device(dev, &gta02_usbhost_supply_device);
	} else if (strcmp(devname, "pcf50633-regulator.0") == 0) {
		gta02_add_child_device(NULL, &s3c_device_adc);
		gta02_add_child_device(NULL, &s3c_device_ts);
	} else if (strcmp(devname, "reg-fixed-voltage.0") == 0) {
		gta02_add_child_device(NULL, &gta02_pm_gsm_dev);
	} else if (strcmp(devname, "pcf50633-regulator.3") == 0) {
		gta02_add_child_device(NULL, &gta02_gsensor_userspace_consumer);
	} else if (strcmp(devname, "pcf50633-regulator.6") == 0) {
		gta02_add_child_device(NULL, &gta02_pm_bt_dev);
	} else if (strcmp(devname, "pcf50633-regulator.7") == 0) {
		gta02_add_child_device(NULL, &gta02_pm_gps_dev);
	} else if (strcmp(devname, "reg-fixed-voltage.1") == 0) {
		gta02_add_child_device(NULL, &gta02_pm_usbhost_dev);
	} else if (strcmp(devname, "fiq-hdq") == 0) {
		gta02_fiq_hdq_dev = dev;
	} else if (strcmp(devname, "spi2.0") == 0) {
		//pcf50633_bl_set_brightness_limit(gta02_pcf_ops, 0x3f);
		//pcf50633_bl_set_power(gta02_pcf_ops, 0);
		if (gta02_pcf_bl_ops)
			gta02_pcf_bl_ops->set_power(gta02_pcf_bl_ops, 0);
	} else {
		return 0;
	}
	
	return 0;
}

static int gta02_child_device_unregister(struct device *dev)
{
	const char *devname = dev_name(dev);

	if (strcmp(devname, "glamo-gpio.0") == 0) {
		platform_device_unregister(&gta02_glamo_spigpio_dev);
	} else if (strcmp(devname, "0-0073") == 0) {
		gta02_pcf_ops = NULL;
		platform_device_unregister(&bq27000_battery_device);
		platform_device_unregister(&gta02_resume_reason_device);
	} else if (strcmp(devname, "pcf50633-mbc") == 0) {
		gta02_pcf_mbc_ops = NULL;
	} else if (strcmp(devname, "pcf50633-backlight") == 0) {
		gta02_pcf_bl_ops = NULL;
	} else if (strcmp(devname, "pcf50633-gpio") == 0) {
		platform_device_unregister(&gta02_gsm_supply_device);
		platform_device_unregister(&gta02_usbhost_supply_device);
	} else if (strcmp(devname, "pcf50633-regulator.0") == 0) {
		platform_device_unregister(&s3c_device_ts);
		platform_device_unregister(&s3c_device_adc);
	} else if (strcmp(devname, "reg-fixed-voltage.0") == 0) {
		platform_device_unregister(&gta02_pm_gsm_dev);
	} else if (strcmp(devname, "pcf50633-regulator.3") == 0) {
		platform_device_unregister(&gta02_gsensor_userspace_consumer);
	} else if (strcmp(devname, "pcf50633-regulator.6") == 0) {
		platform_device_unregister(&gta02_pm_bt_dev);
	} else if (strcmp(devname, "pcf50633-regulator.7") == 0) {
		platform_device_unregister(&gta02_pm_gps_dev);
	} else if (strcmp(devname, "reg-fixed-voltage.1") == 0) {
		platform_device_unregister(&gta02_pm_usbhost_dev);
	} else if (strcmp(devname, "fiq-hdq") == 0) {
		gta02_fiq_hdq_dev = NULL;
	} else if (strcmp(devname, "spi2.0") == 0) {
		//pcf50633_bl_set_power(gta02_pcf_ops, 4);
		if (gta02_pcf_bl_ops)
			gta02_pcf_bl_ops->set_power(gta02_pcf_bl_ops, 4);
	} else {
		return 0;
	}
	//printk("mach-gta02: device %s unregistered\n", devname);
	return 0;
}

static int gta02_device_registered(struct notifier_block *block,
                unsigned long action, void *data)
{
	struct device *dev = data;

	if (action == BUS_NOTIFY_BOUND_DRIVER)
		return gta02_child_device_registered(dev);
	else if (action == BUS_NOTIFY_UNBIND_DRIVER)
		return gta02_child_device_unregister(dev);
	else
		return 0;
}
static struct notifier_block gta02_device_register_notifier = {
	.notifier_call = gta02_device_registered,
	.priority = 0,
};

/* These are the guys that don't need to be children of PMU. */
static struct platform_device *gta02_devices[] __initdata = {
	&s3c_device_ohci,
	&s3c_device_wdt,
	&s3c_device_i2c0,
	&s3c_device_iis,
	&samsung_asoc_dma,
	&s3c_device_usbgadget,
	&s3c_device_nand,
	&s3c_device_sdi,
//	&s3c_device_adc,
//	&s3c_device_ts,
	&s3c_device_timer[0],
	&s3c_device_timer[1],
	&s3c_device_timer[2],
	&s3c_device_timer[3],

	//&s3c24xx_pwm_device,	/* used to remember suspend/resume states for some gpios ?? */
	&gta02_glamo_dev,

	&gta02_s3c24xx_spi_gpio1,
	
	&gta02_nor_flash,
	&gta02_leds_device,
	&gta02_buttons_device,
	&gta02_dfbmcs320_device,
//	&gta02_pm_bt_dev,
	&gta02_pm_wlan_dev,
//	&gta02_gsensor_userspace_consumer,
	&gta02_hdq_device,
};

/*
 * This gets called frequently when we paniced.
 */
static long gta02_panic_blink(int state)
{
	long delay = 0;
	char led;

	led = (state) ? 1 : 0;
	gpio_direction_output(GTA02_GPIO_AUX_LED, led);

	return delay;
}


static void __init gta02_machine_init(void)
{
	/* Set the panic callback to turn AUX LED on or off. */
	panic_blink = gta02_panic_blink;
	pm_power_off = gta02_poweroff;

	bus_register_notifier(&platform_bus_type, &gta02_device_register_notifier);
	bus_register_notifier(&spi_bus_type, &gta02_device_register_notifier);
	bus_register_notifier(&i2c_bus_type, &gta02_device_register_notifier);
	//bus_register_notifier(&usb_bus_type, &gta02_device_register_notifier);

	s3c_pm_init();

	s3c24xx_udc_set_platdata(&gta02_udc_cfg);
	s3c24xx_ts_set_platdata(&gta02_ts_info);
	s3c_ohci_set_platdata(&gta02_usb_info);
	s3c_nand_set_platdata(&gta02_nand_info);
	s3c_i2c0_set_platdata(NULL);
	
	platform_add_devices(gta02_devices, ARRAY_SIZE(gta02_devices));
    regulator_has_full_constraints();
    
	i2c_register_board_info(0, gta02_i2c_devs,
	                        ARRAY_SIZE(gta02_i2c_devs));
	spi_register_board_info(gta02_spi_board_info,
                            ARRAY_SIZE(gta02_spi_board_info));

	//platform_add_devices(gta02_devices, ARRAY_SIZE(gta02_devices));
}

static void __init gta02_init_irq(void)
{
	s3c24xx_init_irq();

/*	irq_reserve_irqs(1, 15);
	irq_reserve_irqs(68, 2);*/
};

static struct map_desc gta02_iodesc[] __initdata = {
	{
		.virtual	= 0xe0000000,
		.pfn		= __phys_to_pfn(S3C2410_CS3 + 0x01000000),
		.length		= SZ_1M,
		.type		= MT_DEVICE
	},
};

static void __init gta02_map_io(void)
{
	s3c24xx_init_io(gta02_iodesc, ARRAY_SIZE(gta02_iodesc));
	s3c24xx_init_clocks(12000000);
	s3c24xx_init_uarts(gta02_uartcfgs, ARRAY_SIZE(gta02_uartcfgs));
}

MACHINE_START(NEO1973_GTA02, "GTA02")
	/* Maintainer: Nelson Castillo <arhuaco@freaks-unidos.net> */
	.atag_offset	= 0x100,
	.map_io			= gta02_map_io,
	.init_irq		= gta02_init_irq,
//	.nr_irqs		= (IRQ_S3C2440_AC97+1),
	.init_machine	= gta02_machine_init,
	.timer			= &s3c24xx_timer,
	.restart		= gta02_restart,
MACHINE_END
