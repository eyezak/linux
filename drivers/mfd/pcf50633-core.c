/* NXP PCF50633 Power Management Unit (PMU) driver
 *
 * (C) 2006-2008 by Openmoko, Inc.
 * Author: Harald Welte <laforge@openmoko.org>
 * 	   Balaji Rao <balajirrao@openmoko.org>
 * All rights reserved.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/mfd/core.h>
#include <linux/mfd/pcf50633/core.h>
#include <linux/mfd/pcf50633/adc.h>
#include <linux/mfd/pcf50633/mbc.h>
#include <linux/mfd/pcf50633/pmic.h>

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/regmap.h>
#include <linux/err.h>


struct pcf50633 *dev_to_pcf50633(struct device *dev)
{
	return dev_get_drvdata(dev);
}
EXPORT_SYMBOL_GPL(dev_to_pcf50633);

int pcf50633_hwirq_to_irq(struct device *dev, int irq)
{
	struct pcf50633 * pcf = dev_get_drvdata(dev);
	return irq - pcf->irq_base;
}
EXPORT_SYMBOL_GPL(pcf50633_hwirq_to_irq);

/* Read a block of upto 32 regs  */
int pcf50633_read_block(struct pcf50633 *pcf, unsigned int reg,
					int nr_regs, u8 *data)
{
	return regmap_raw_read(pcf->regmap, reg, data, nr_regs);
}
EXPORT_SYMBOL_GPL(pcf50633_read_block);

/* Write a block of upto 32 regs  */
int pcf50633_write_block(struct pcf50633 *pcf , unsigned int reg,
					int nr_regs, u8 *data)
{
	return regmap_raw_write(pcf->regmap, reg, data, nr_regs);
}
EXPORT_SYMBOL_GPL(pcf50633_write_block);

u8 pcf50633_reg_read(struct pcf50633 *pcf, unsigned int reg)
{
	unsigned int val;

	if (regmap_read(pcf->regmap, reg, &val) < 0)
		return 0x0;

	return val;
}
EXPORT_SYMBOL_GPL(pcf50633_reg_read);

int pcf50633_reg_write(struct pcf50633 *pcf, unsigned int reg, unsigned int val)
{
	return regmap_write(pcf->regmap, reg, val);
}
EXPORT_SYMBOL_GPL(pcf50633_reg_write);

int pcf50633_reg_set_bit_mask(struct pcf50633 *pcf, unsigned int reg, unsigned int mask, unsigned int val)
{
	return regmap_update_bits(pcf->regmap, reg, mask, val);
}
EXPORT_SYMBOL_GPL(pcf50633_reg_set_bit_mask);

int pcf50633_reg_clear_bits(struct pcf50633 *pcf, unsigned int reg, unsigned int val)
{
	return regmap_update_bits(pcf->regmap, reg, val, 0);
}
EXPORT_SYMBOL_GPL(pcf50633_reg_clear_bits);

/* sysfs attributes */
static ssize_t show_dump_regs(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct pcf50633 *pcf = dev_to_pcf50633(dev);
	u8 dump[16];
	int n, n1, idx = 0;
	char *buf1 = buf;
	static u8 address_no_read[] = { /* must be ascending */
		PCF50633_REG_INT1,
		PCF50633_REG_INT2,
		PCF50633_REG_INT3,
		PCF50633_REG_INT4,
		PCF50633_REG_INT5,
		0 /* terminator */
	};

	for (n = 0; n < 256; n += sizeof(dump)) {
		for (n1 = 0; n1 < sizeof(dump); n1++)
			if (n == address_no_read[idx]) {
				idx++;
				dump[n1] = 0x00;
			} else
				dump[n1] = pcf50633_reg_read(pcf, n + n1);

		hex_dump_to_buffer(dump, sizeof(dump), 16, 1, buf1, 128, 0);
		buf1 += strlen(buf1);
		*buf1++ = '\n';
		*buf1 = '\0';
	}

	return buf1 - buf;
}
static DEVICE_ATTR(dump_regs, 0400, show_dump_regs, NULL);

static ssize_t show_resume_reason(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct pcf50633 *pcf = dev_to_pcf50633(dev);
	int n;

	n = sprintf(buf, "%02x%02x%02x%02x%02x\n",
				pcf->resume_reason[0],
				pcf->resume_reason[1],
				pcf->resume_reason[2],
				pcf->resume_reason[3],
				pcf->resume_reason[4]);

	return n;
}
static DEVICE_ATTR(resume_reason, 0400, show_resume_reason, NULL);

static struct attribute *pcf_sysfs_entries[] = {
	&dev_attr_dump_regs.attr,
	&dev_attr_resume_reason.attr,
	NULL,
};

static struct attribute_group pcf_attr_group = {
	.name	= NULL,			/* put in device directory */
	.attrs	= pcf_sysfs_entries,
};


#ifdef CONFIG_PM_SLEEP
static int pcf50633_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pcf50633 *pcf = i2c_get_clientdata(client);
	int i;
	u8 res[5];

	/* Make sure our interrupt handlers are not called
	 * henceforth */
	disable_irq(pcf->irq);
	
	regcache_cache_only(pcf->regmap, true);
	regcache_mark_dirty(pcf->regmap);

	/* Write wakeup irq masks */
	for (i = 0; i < ARRAY_SIZE(res); i++)
		res[i] = ~pcf->pdata->resumers[i];

	i = pcf50633_write_block(pcf, PCF50633_REG_INT1M,
					ARRAY_SIZE(res), &res[0]);
	if (i < 0)
		dev_err(pcf->dev, "error %d writing wakeup irq masks\n", i);

	return 0;
}

static int pcf50633_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pcf50633 *pcf = i2c_get_clientdata(client);
	int ret;

	regcache_cache_only(pcf->regmap, false);
	ret = regcache_sync(pcf->regmap);
	if (ret != 0) 
		dev_err(pcf->dev, "Failed to restore register map: %d\n", ret);

	enable_irq(pcf->irq);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(pcf50633_pm, pcf50633_suspend, pcf50633_resume);

static bool pcf50633_reg_readable(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case PCF50633_REG_RESERVED1:
	case PCF50633_REG_RESERVED2:
	case PCF50633_REG_RESERVED3:
	case PCF50633_REG_RESERVED4 + 1:
	case PCF50633_REG_RESERVED4 + 2:
	case PCF50633_REG_RESERVED4 + 3:
	case PCF50633_REG_RESERVED4 + 4:
	case PCF50633_REG_RESERVED4 + 5:
	case PCF50633_REG_RESERVED4 + 6:
	case PCF50633_REG_RESERVED4 + 7:
	case PCF50633_REG_RESERVED4 + 8:
	case PCF50633_REG_RESERVED4 + 9:
	case PCF50633_REG_RESERVED4 + 10:
	case PCF50633_REG_RESERVED4 + 11:
	case PCF50633_REG_RESERVED4 + 12:
	case PCF50633_REG_RESERVED4 + 13:
	case PCF50633_REG_RESERVED4 + 14:
	case PCF50633_REG_RESERVED4 + 15:
	case PCF50633_REG_RESERVED4 + 16:
	case PCF50633_REG_RESERVED4 + 17:
	case PCF50633_REG_RESERVED4 + 18:
	case PCF50633_REG_RESERVED4 + 19:
		return false;
	default:
		return true;
	}
}

static bool pcf50633_reg_writeable(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case PCF50633_REG_INT1:
	case PCF50633_REG_INT2:
	case PCF50633_REG_INT3:
	case PCF50633_REG_INT4:
	case PCF50633_REG_INT5:
	case PCF50633_REG_VERSION:
	case PCF50633_REG_VARIANT:
	case PCF50633_REG_OOCSTAT:
	case PCF50633_REG_RESERVED1:
	case PCF50633_REG_DCDCSTAT:
	case PCF50633_REG_LDOSTAT:
	case PCF50633_REG_MBCS1:
	case PCF50633_REG_MBCS2:
	case PCF50633_REG_MBCS3:
	case PCF50633_REG_ALMDATA:
	case PCF50633_REG_RESERVED2:
	case PCF50633_REG_ADCS1:
	case PCF50633_REG_ADCS2:
	case PCF50633_REG_ADCS3:
	case PCF50633_REG_RESERVED3:
	case PCF50633_REG_RESERVED4 + 1:
	case PCF50633_REG_RESERVED4 + 2:
	case PCF50633_REG_RESERVED4 + 3:
	case PCF50633_REG_RESERVED4 + 4:
	case PCF50633_REG_RESERVED4 + 5:
	case PCF50633_REG_RESERVED4 + 6:
	case PCF50633_REG_RESERVED4 + 7:
	case PCF50633_REG_RESERVED4 + 8:
	case PCF50633_REG_RESERVED4 + 9:
	case PCF50633_REG_RESERVED4 + 10:
	case PCF50633_REG_RESERVED4 + 11:
	case PCF50633_REG_RESERVED4 + 12:
	case PCF50633_REG_RESERVED4 + 13:
	case PCF50633_REG_RESERVED4 + 14:
	case PCF50633_REG_RESERVED4 + 15:
	case PCF50633_REG_RESERVED4 + 16:
	case PCF50633_REG_RESERVED4 + 17:
	case PCF50633_REG_RESERVED4 + 18:
	case PCF50633_REG_RESERVED4 + 19:
		return false;
	default:
		return true;
	}
}

static bool pcf50633_reg_precious(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case PCF50633_REG_INT1:
	case PCF50633_REG_INT2:
	case PCF50633_REG_INT3:
	case PCF50633_REG_INT4:
	case PCF50633_REG_INT5:
		return true;
	default:
		return false;
	}
}

static struct regmap_config pcf50633_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	
	.cache_type = REGCACHE_RBTREE,
	//REGCACHE_COMPRESSED,
	
	.max_register = PCF50633_REG_DCDCPFM,
	.readable_reg = pcf50633_reg_readable,
	.writeable_reg = pcf50633_reg_writeable,
	.precious_reg = pcf50633_reg_precious,
};

#define DECLARE_IRQ(_irq, _regno) \
	[PCF50633_IRQ_ ## _irq] = { \
		.reg_offset = _regno-1, \
		.mask = PCF50633_INT ## _regno ## _ ## _irq, \
	}

static struct regmap_irq pcf50633_irqs[] = {
	//[0] = { },
	DECLARE_IRQ(ADPINS, 1),
	DECLARE_IRQ(ADPREM, 1),
	DECLARE_IRQ(USBINS, 1),
	DECLARE_IRQ(USBREM, 1),
	DECLARE_IRQ(ALARM, 1),
	DECLARE_IRQ(SECOND, 1),
	DECLARE_IRQ(ONKEYR, 2),
	DECLARE_IRQ(ONKEYF, 2),
	DECLARE_IRQ(EXTON1R, 2),
	DECLARE_IRQ(EXTON1F, 2),
	DECLARE_IRQ(EXTON2R, 2),
	DECLARE_IRQ(EXTON2F, 2),
	DECLARE_IRQ(EXTON3R, 2),
	DECLARE_IRQ(EXTON3F, 2),
	DECLARE_IRQ(BATFULL, 3),
	DECLARE_IRQ(CHGHALT, 3),
	DECLARE_IRQ(THLIMON, 3),
	DECLARE_IRQ(THLIMOFF, 3),
	DECLARE_IRQ(USBLIMON, 3),
	DECLARE_IRQ(USBLIMOFF, 3),
	DECLARE_IRQ(ADCRDY, 3),
	DECLARE_IRQ(ONKEY1S, 3),
	DECLARE_IRQ(LOWSYS, 4),
	DECLARE_IRQ(LOWBAT, 4),
	DECLARE_IRQ(HIGHTMP, 4),
	DECLARE_IRQ(AUTOPWRFAIL, 4),
	DECLARE_IRQ(DWN1PWRFAIL, 4),
	DECLARE_IRQ(DWN2PWRFAIL, 4),
	DECLARE_IRQ(LEDPWRFAIL,	 4),
	DECLARE_IRQ(LEDOVP, 4),
	DECLARE_IRQ(LDO1PWRFAIL, 5),
	DECLARE_IRQ(LDO1PWRFAIL, 5),
	DECLARE_IRQ(LDO1PWRFAIL, 5),
	DECLARE_IRQ(LDO1PWRFAIL, 5),
	DECLARE_IRQ(LDO1PWRFAIL, 5),
	DECLARE_IRQ(LDO1PWRFAIL, 5),
	DECLARE_IRQ(HCLDOPWRFAIL, 5),
	DECLARE_IRQ(HCLDOOVL, 5),
	
};

static struct regmap_irq_chip pcf50633_regmap_irq_chip = {
	.name = "pcf50633-irq",
	.status_base = PCF50633_REG_INT1,
	.mask_base = PCF50633_REG_INT1M,
	.num_regs = PCF50633_NUM_INT_REGS,
	.irqs = pcf50633_irqs,
	.num_irqs = ARRAY_SIZE(pcf50633_irqs),
};


#define PCF50633_CELL(_name) \
	{ \
		.name = _name, \
		.id = -1, \
	} \

#define PCF50633_CELL_RESOURCES(_name, _resources) \
	{ \
		.name = _name, \
		.num_resources = ARRAY_SIZE(_resources), \
		.resources = _resources, \
		.id = -1, \
	} \

#define PCF50633_CELL_ID(_name, _id) \
	{ \
		.name = _name, \
		.id = _id, \
	} \

#define PCF50633_IRQ_RESOURCE(_id) \
	{ \
		.start = PCF50633_IRQ_ ## _id, \
		.end = PCF50633_IRQ_ ## _id, \
		.flags = IORESOURCE_IRQ, \
		.name = #_id, \
	} \

static struct resource pcf50633_adc_resources[] = {
	PCF50633_IRQ_RESOURCE(ADCRDY),
};

static struct resource pcf50633_input_resources[] = {
	PCF50633_IRQ_RESOURCE(ONKEYR),
	PCF50633_IRQ_RESOURCE(ONKEYF),
	PCF50633_IRQ_RESOURCE(ONKEY1S),
};

static struct resource pcf50633_rtc_resources[] = {
	PCF50633_IRQ_RESOURCE(SECOND),
	PCF50633_IRQ_RESOURCE(ALARM),
};

static struct resource pcf50633_mbc_resources[] = {
	PCF50633_IRQ_RESOURCE(ADPINS),
	PCF50633_IRQ_RESOURCE(ADPREM),
	PCF50633_IRQ_RESOURCE(USBINS),
	PCF50633_IRQ_RESOURCE(USBREM),
	PCF50633_IRQ_RESOURCE(BATFULL),
	PCF50633_IRQ_RESOURCE(CHGHALT),
	PCF50633_IRQ_RESOURCE(THLIMON),
	PCF50633_IRQ_RESOURCE(THLIMOFF),
	PCF50633_IRQ_RESOURCE(USBLIMON),
	PCF50633_IRQ_RESOURCE(USBLIMOFF),
	PCF50633_IRQ_RESOURCE(LOWSYS),
	PCF50633_IRQ_RESOURCE(LOWBAT),
};

static struct mfd_cell pcf50633_cells[] = {
	PCF50633_CELL_RESOURCES("pcf50633-input", pcf50633_input_resources),
	PCF50633_CELL_RESOURCES("pcf50633-rtc", pcf50633_rtc_resources),
	PCF50633_CELL_RESOURCES("pcf50633-mbc", pcf50633_mbc_resources),
	PCF50633_CELL_RESOURCES("pcf50633-adc", pcf50633_adc_resources),
	PCF50633_CELL("pcf50633-backlight"),
	PCF50633_CELL("gpio-pcf50633"),
	PCF50633_CELL_ID("pcf50633-regulator", 0),
	PCF50633_CELL_ID("pcf50633-regulator", 1),
	PCF50633_CELL_ID("pcf50633-regulator", 2),
	PCF50633_CELL_ID("pcf50633-regulator", 3),
	PCF50633_CELL_ID("pcf50633-regulator", 4),
	PCF50633_CELL_ID("pcf50633-regulator", 5),
	PCF50633_CELL_ID("pcf50633-regulator", 6),
	PCF50633_CELL_ID("pcf50633-regulator", 7),
	PCF50633_CELL_ID("pcf50633-regulator", 8),
	PCF50633_CELL_ID("pcf50633-regulator", 9),
	PCF50633_CELL_ID("pcf50633-regulator", 10),
};

static int __devinit pcf50633_probe(struct i2c_client *client,
				const struct i2c_device_id *ids)
{
	struct pcf50633 *pcf;
	struct pcf50633_platform_data *pdata = client->dev.platform_data;
	int ret;
	unsigned int version, variant;

	if (!client->irq) {
		dev_err(&client->dev, "Missing IRQ\n");
		return -ENOENT;
	}
	
	pcf = kzalloc(sizeof(struct pcf50633), GFP_KERNEL);
	if (!pcf)
		return -ENOMEM;
	pcf->pdata = pdata;

	mutex_init(&pcf->lock);

	pcf->regmap = regmap_init_i2c(client, &pcf50633_regmap_config);
	if (IS_ERR(pcf->regmap)) {
		ret = PTR_ERR(pcf->regmap);
		dev_err(pcf->dev, "Failed to allocate register map: %d\n", ret);
		goto err_free;
	}

	i2c_set_clientdata(client, pcf);
	pcf->dev = &client->dev;

	regmap_read(pcf->regmap, PCF50633_REG_VERSION, &version);
	regmap_read(pcf->regmap, PCF50633_REG_VARIANT, &variant);
	if (version < 0 || variant < 0) {
		dev_err(pcf->dev, "Unable to probe pcf50633\n");
		ret = -ENODEV;
		goto err_regmap;
	}

	dev_info(pcf->dev, "Probed device version %d variant %d\n",
							version, variant);

	pcf->irq = client->irq;
	pcf->irq_base = pcf->pdata->irq_base ?: -1;
	ret = regmap_add_irq_chip(pcf->regmap, pcf->irq,
	             IRQF_TRIGGER_LOW | IRQF_ONESHOT,
	             -1, &pcf50633_regmap_irq_chip,
	             &pcf->irq_data);
	if (ret < 0) {
		dev_err(pcf->dev, "Failed to add irq chip: %d\n", ret);
		goto err_regmap;
	}
	pcf->irq_base = regmap_irq_chip_get_base(pcf->irq_data);

	if (enable_irq_wake(pcf->irq) < 0)
		dev_err(pcf->dev, "IRQ %u cannot be enabled as wake-up source"
			"in this hardware revision", pcf->irq);	
	if (ret)
		goto err_irq;

	pcf50633_cells[0].platform_data = &pdata->force_shutdown_timeout;
	pcf50633_cells[0].pdata_size = sizeof(pdata->force_shutdown_timeout);
	pcf50633_cells[2].platform_data = &pdata->mbc_data;
	pcf50633_cells[2].pdata_size = sizeof(pdata->mbc_data);
	pcf50633_cells[3].platform_data = &pdata->adc_data;
	pcf50633_cells[3].pdata_size = sizeof(pdata->adc_data);
	pcf50633_cells[4].platform_data = &pdata->backlight_data;
	pcf50633_cells[4].pdata_size = sizeof(pdata->backlight_data);
	pcf50633_cells[5].platform_data = &pdata->gpio_base;
	pcf50633_cells[5].pdata_size = sizeof(pdata->gpio_base);

	ret = mfd_add_devices(pcf->dev, 0, pcf50633_cells,
			ARRAY_SIZE(pcf50633_cells), NULL, pcf->irq_base);
	if (ret) {
		dev_err(pcf->dev, "Failed to add mfd cells.\n");
		goto err_irq;
	}

	ret = sysfs_create_group(&client->dev.kobj, &pcf_attr_group);
	if (ret)
		dev_err(pcf->dev, "error creating sysfs entries\n");

	return 0;

err_irq:
	regmap_del_irq_chip(pcf->irq, pcf->irq_data);
err_regmap:
	regmap_exit(pcf->regmap);
err_free:
	kfree(pcf);

	return ret;
}

static int __devexit pcf50633_remove(struct i2c_client *client)
{
	struct pcf50633 *pcf = i2c_get_clientdata(client);

	sysfs_remove_group(&client->dev.kobj, &pcf_attr_group);

	mfd_remove_devices(pcf->dev);

	regmap_del_irq_chip(pcf->irq, pcf->irq_data);
	regmap_exit(pcf->regmap);
	
	kfree(pcf);

	return 0;
}

static const struct i2c_device_id pcf50633_id_table[] = {
	{"pcf50633", 0x73},
	{/* end of list */}
};
MODULE_DEVICE_TABLE(i2c, pcf50633_id_table);

static struct i2c_driver pcf50633_driver = {
	.driver = {
		.name	= "pcf50633",
		.pm	= &pcf50633_pm,
	},
	.id_table = pcf50633_id_table,
	.probe = pcf50633_probe,
	.remove = __devexit_p(pcf50633_remove),
};

static int __init pcf50633_init(void)
{
	return i2c_add_driver(&pcf50633_driver);
}

static void __exit pcf50633_exit(void)
{
	i2c_del_driver(&pcf50633_driver);
}

MODULE_DESCRIPTION("I2C chip driver for NXP PCF50633 PMU");
MODULE_AUTHOR("Harald Welte <laforge@openmoko.org>");
MODULE_LICENSE("GPL");

subsys_initcall(pcf50633_init);
module_exit(pcf50633_exit);
