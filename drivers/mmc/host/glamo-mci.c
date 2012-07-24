/*
 *  linux/drivers/mmc/host/glamo-mmc.c - Glamo MMC driver
 *
 *  Copyright (C) 2007 Openmoko, Inc,  Andy Green <andy@openmoko.com>
 *  Copyright (C) 2009, Lars-Peter Clausen <lars@metafoo.de>
 *  Based on S3C MMC driver that was:
 *  Copyright (C) 2004-2006 maintech GmbH, Thomas Kleffel <tk@maintech.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/host.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/crc7.h>
#include <linux/scatterlist.h>
#include <linux/io.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/mfd/glamo.h>
#include <linux/mfd/glamo-core.h>
#include <linux/mfd/glamo-regs.h>

struct glamo_mci_host {
	struct glamo_mmc_platform_data *pdata;
	struct platform_device	*pdev;
	struct glamo_core	*core;
	struct mmc_host		*mmc;
	struct resource		*mmio_mem;
	struct resource		*data_mem;
	void __iomem		*mmio_base;
	uint16_t __iomem	*data_base;

	unsigned int irq;

	struct regulator *regulator;
	struct mmc_request *mrq;

	unsigned int clk_rate;

	unsigned short vdd;
	char power_mode;

	unsigned long transfer_start;
	unsigned long request_start;

	unsigned char request_counter;
};

static void glamo_mci_send_request(struct mmc_host *mmc,
				   struct mmc_request *mrq);
static void glamo_mci_send_command(struct glamo_mci_host *host,
				   struct mmc_command *cmd);

/*
 * Max SD clock rate
 *
 * held at /(3 + 1) due to concerns of 100R recommended series resistor
 * allows 16MHz @ 4-bit --> 8MBytes/sec raw
 *
 * you can override this on kernel commandline using
 *
 *   glamo_mci.sd_max_clk=10000000
 *
 * for example
 */

static int sd_max_clk = 17000000;
module_param(sd_max_clk, int, 0644);

/*
 * Slow SD clock rate
 *
 * you can override this on kernel commandline using
 *
 *   glamo_mci.sd_slow_ratio=8
 *
 * for example
 *
 * platform callback is used to decide effective clock rate, if not
 * defined then max is used, if defined and returns nonzero, rate is
 * divided by this factor
 */

static int sd_slow_ratio = 8;
module_param(sd_slow_ratio, int, 0644);

/*
 * Post-power SD clock rate
 *
 * you can override this on kernel commandline using
 *
 *   glamo_mci.sd_post_power_clock=1000000
 *
 * for example
 *
 * After changing power to card, clock is held at this rate until first bulk
 * transfer completes
 */

static int sd_post_power_clock = 1000000;
module_param(sd_post_power_clock, int, 0644);


static inline void glamomci_reg_write(struct glamo_mci_host *glamo,
		uint16_t reg, uint16_t val)
{
	writew(val, glamo->mmio_base + reg);
}

static inline uint16_t glamomci_reg_read(struct glamo_mci_host *glamo,
		uint16_t reg)
{
	return readw(glamo->mmio_base + reg);
}

static void glamomci_reg_set_bit_mask(struct glamo_mci_host *glamo,
		uint16_t reg, uint16_t mask, uint16_t val)
{
	uint16_t tmp;

	val &= mask;

	tmp = glamomci_reg_read(glamo, reg);
	tmp &= ~mask;
	tmp |= val;
	glamomci_reg_write(glamo, reg, tmp);
}

static void glamo_mci_reset(struct glamo_mci_host *host)
{
	glamo_engine_reset(host->core, GLAMO_ENGINE_MMC);

	glamomci_reg_write(host, GLAMO_REG_MMC_WDATADS1,
			(uint16_t)(host->data_mem->start));
	glamomci_reg_write(host, GLAMO_REG_MMC_WDATADS2,
			(uint16_t)(host->data_mem->start >> 16));

	glamomci_reg_write(host, GLAMO_REG_MMC_RDATADS1,
			(uint16_t)(host->data_mem->start));
	glamomci_reg_write(host, GLAMO_REG_MMC_RDATADS2,
			(uint16_t)(host->data_mem->start >> 16));

}

static int glamo_mci_clock_disable(struct mmc_host *mmc)
{
	struct glamo_mci_host *host = mmc_priv(mmc);
	glamo_engine_suspend(host->core, GLAMO_ENGINE_MMC);
	msleep(1000 / 16);
	
	return 0;
}

static int glamo_mci_clock_enable(struct mmc_host *mmc)
{
	struct glamo_mci_host *host = mmc_priv(mmc);
	glamo_engine_enable(host->core, GLAMO_ENGINE_MMC);
	return 0;
}

static void __iomem *glamo_mci_get_data_addr(struct glamo_mci_host *host,
	struct mmc_data *data)
{
	void __iomem *addr = host->data_base;

	if (data->host_cookie & 1)
		addr += resource_size(host->data_mem) / 2;

	return addr;
}

static void do_pio_read(struct glamo_mci_host *host, struct mmc_data *data)
{
	void __iomem *from_ptr = glamo_mci_get_data_addr(host, data);
	struct sg_mapping_iter miter;

	dev_dbg(&host->pdev->dev, "pio_read():\n");

	sg_miter_start(&miter, data->sg, data->sg_len, SG_MITER_TO_SG);

	while (sg_miter_next(&miter)) {
		memcpy(miter.addr, from_ptr, miter.length);
		from_ptr += miter.length;
	}

	sg_miter_stop(&miter);

	dev_dbg(&host->pdev->dev, "pio_read(): "
			"complete (no more data)\n");
}

static void do_pio_write(struct glamo_mci_host *host, struct mmc_data *data)
{
	void __iomem *to_ptr = glamo_mci_get_data_addr(host, data);
	struct sg_mapping_iter miter;

	dev_dbg(&host->pdev->dev, "pio_write():\n");
	sg_miter_start(&miter, data->sg, data->sg_len, SG_MITER_FROM_SG);

	while (sg_miter_next(&miter)) {
		memcpy(to_ptr, miter.addr, miter.length);
		to_ptr += miter.length;

		data->bytes_xfered += miter.length;
	}
	sg_miter_stop(&miter);

	dev_dbg(&host->pdev->dev, "pio_write(): complete\n");
}

static int glamo_mci_set_card_clock(struct glamo_mci_host *host, int freq)
{
	int real_rate = 0;

	if (freq)
		real_rate = glamo_engine_reclock(host->core, GLAMO_ENGINE_MMC,
						 freq);

	return real_rate;
}

static int glamo_mci_wait_idle(struct glamo_mci_host *host,
			       unsigned long timeout)
{
	uint16_t status;

	do {
		status = glamomci_reg_read(host, GLAMO_REG_MMC_RB_STAT1);
	} while (!(status & GLAMO_STAT1_MMC_IDLE) &&
		  time_is_after_jiffies(timeout));

	if (time_is_before_eq_jiffies(timeout)) {
		glamo_mci_reset(host);
		return -ETIMEDOUT;
	}

	return 0;
}

static void glamo_mci_request_done(struct glamo_mci_host *host,
				   struct mmc_request *mrq)
{
	mmc_request_done(host->mmc, mrq);
}

static irqreturn_t glamo_mci_irq(int irq, void *data)
{
	struct glamo_mci_host *host = data;
	struct mmc_request *mrq;
	struct mmc_command *cmd;
	uint16_t status;

	if (!host->mrq || !host->mrq->cmd)
		return IRQ_HANDLED;

	mrq = host->mrq;
	cmd = mrq->cmd;

	status = glamomci_reg_read(host, GLAMO_REG_MMC_RB_STAT1);
	dev_dbg(&host->pdev->dev, "status = 0x%04x\n", status);

	/* we ignore a data timeout report if we are also told the data came */
	if (status & GLAMO_STAT1_MMC_RB_DRDY)
		status &= ~GLAMO_STAT1_MMC_DTOUT;

	if (status & (GLAMO_STAT1_MMC_RTOUT | GLAMO_STAT1_MMC_DTOUT))
		cmd->error = -ETIMEDOUT;
	else if (status & (GLAMO_STAT1_MMC_BWERR | GLAMO_STAT1_MMC_BRERR))
		cmd->error = -EILSEQ;

	if (cmd->error) {
		dev_info(&host->pdev->dev, "Error after cmd: 0x%x\n", status);
		goto done;
	}

	/* issue STOP if we have been given one to use */
	if (mrq->stop)
		glamo_mci_send_command(host, mrq->stop);

	if (mrq->data && (mrq->data->flags & MMC_DATA_READ)) {
		mrq->data->bytes_xfered = mrq->data->blocks * mrq->data->blksz;
		if (!mrq->data->host_cookie)
			do_pio_read(host, mrq->data);
	}

	if (mrq->stop)
		mrq->stop->error = glamo_mci_wait_idle(host, jiffies + HZ);

done:
	host->mrq = NULL;
	glamo_mci_request_done(host, cmd->mrq);

	return IRQ_HANDLED;
}

static void glamo_mci_send_command(struct glamo_mci_host *host,
		struct mmc_command *cmd)
{
	uint8_t u8a[6];
	uint16_t fire = 0;
	unsigned int timeout = 1000000;
	uint16_t *reg_resp = (uint16_t *)(host->mmio_base + GLAMO_REG_MMC_CMD_RSP1);
	uint16_t status;
	int triggers_int = 1;

	/* if we can't do it, reject as busy */
	if (!(glamomci_reg_read(host, GLAMO_REG_MMC_RB_STAT1) &
		 GLAMO_STAT1_MMC_IDLE)) {
		cmd->error = -EBUSY;
		return;
	}

	/* create an array in wire order for CRC computation */
	u8a[0] = 0x40 | (cmd->opcode & 0x3f);
	u8a[1] = (uint8_t)(cmd->arg >> 24);
	u8a[2] = (uint8_t)(cmd->arg >> 16);
	u8a[3] = (uint8_t)(cmd->arg >> 8);
	u8a[4] = (uint8_t)cmd->arg;
	u8a[5] = (crc7(0, u8a, 5) << 1) | 0x01;

	/* issue the wire-order array including CRC in register order */
	glamomci_reg_write(host, GLAMO_REG_MMC_CMD_REG1, ((u8a[4] << 8) | u8a[5]));
	glamomci_reg_write(host, GLAMO_REG_MMC_CMD_REG2, ((u8a[2] << 8) | u8a[3]));
	glamomci_reg_write(host, GLAMO_REG_MMC_CMD_REG3, ((u8a[0] << 8) | u8a[1]));

	/* command index toggle */
	fire |= (host->request_counter & 1) << 12;

	/* set type of command */
	switch (mmc_cmd_type(cmd)) {
	case MMC_CMD_BC:
		fire |= GLAMO_FIRE_MMC_CMDT_BNR;
		break;
	case MMC_CMD_BCR:
		fire |= GLAMO_FIRE_MMC_CMDT_BR;
		break;
	case MMC_CMD_AC:
		fire |= GLAMO_FIRE_MMC_CMDT_AND;
		break;
	case MMC_CMD_ADTC:
		fire |= GLAMO_FIRE_MMC_CMDT_AD;
		break;
	}
	/*
	 * if it expects a response, set the type expected
	 *
	 * R1, Length  : 48bit, Normal response
	 * R1b, Length : 48bit, same R1, but added card busy status
	 * R2, Length  : 136bit (really 128 bits with CRC snipped)
	 * R3, Length  : 48bit (OCR register value)
	 * R4, Length  : 48bit, SDIO_OP_CONDITION, Reverse SDIO Card
	 * R5, Length  : 48bit, IO_RW_DIRECTION, Reverse SDIO Card
	 * R6, Length  : 48bit (RCA register)
	 * R7, Length  : 48bit (interface condition, VHS(voltage supplied),
	 *                     check pattern, CRC7)
	 */
	switch (mmc_resp_type(cmd)) {
	case MMC_RSP_R1: /* same index as R6 and R7 */
		fire |= GLAMO_FIRE_MMC_RSPT_R1;
		break;
	case MMC_RSP_R1B:
		fire |= GLAMO_FIRE_MMC_RSPT_R1b;
		break;
	case MMC_RSP_R2:
		fire |= GLAMO_FIRE_MMC_RSPT_R2;
		break;
	case MMC_RSP_R3:
		fire |= GLAMO_FIRE_MMC_RSPT_R3;
		break;
	/* R4 and R5 supported by chip not defined in linux/mmc/core.h (sdio) */
	}
	/*
	 * From the command index, set up the command class in the host ctrllr
	 *
	 * missing guys present on chip but couldn't figure out how to use yet:
	 *     0x0 "stream read"
	 *     0x9 "cancel running command"
	 */
	switch (cmd->opcode) {
	case MMC_READ_SINGLE_BLOCK:
		fire |= GLAMO_FIRE_MMC_CC_SBR; /* single block read */
		break;
	case MMC_SWITCH: /* 64 byte payload */
	case SD_APP_SD_STATUS:
	case SD_APP_SEND_SCR:
	case MMC_READ_MULTIPLE_BLOCK:
		/* we will get an interrupt off this */
		if (!cmd->mrq->stop) {
			/* multiblock no stop */
			fire |= GLAMO_FIRE_MMC_CC_MBRNS;
		} else {
			/* multiblock with stop */
			fire |= GLAMO_FIRE_MMC_CC_MBRS;
		}
		break;
	case MMC_WRITE_BLOCK:
		fire |= GLAMO_FIRE_MMC_CC_SBW; /* single block write */
		break;
	case MMC_WRITE_MULTIPLE_BLOCK:
		if (cmd->mrq->stop) {
			/* multiblock with stop */
			fire |= GLAMO_FIRE_MMC_CC_MBWS;
		} else {
			/* multiblock NO stop-- 'RESERVED'? */
			fire |= GLAMO_FIRE_MMC_CC_MBWNS;
		}
		break;
	case MMC_STOP_TRANSMISSION:
		fire |= GLAMO_FIRE_MMC_CC_STOP; /* STOP */
		triggers_int = 0;
		break;
	default:
		fire |= GLAMO_FIRE_MMC_CC_BASIC; /* "basic command" */
		triggers_int = 0;
		break;
	}

	if (cmd->data)
		host->mrq = cmd->mrq;

	/* always largest timeout */
	glamomci_reg_write(host, GLAMO_REG_MMC_TIMEOUT, 0xfff);

	/* Generate interrupt on txfer */
	glamomci_reg_set_bit_mask(host, GLAMO_REG_MMC_BASIC, 0xff36,
			0x0800 |
			GLAMO_BASIC_MMC_NO_CLK_RD_WAIT |
			GLAMO_BASIC_MMC_EN_COMPL_INT |
			GLAMO_BASIC_MMC_EN_DATA_PUPS |
			GLAMO_BASIC_MMC_EN_CMD_PUP);

	/* send the command out on the wire */
	/* dev_info(&host->pdev->dev, "Using FIRE %04X\n", fire); */
	glamomci_reg_write(host, GLAMO_REG_MMC_CMD_FIRE, fire);

	/* we are deselecting card?  because it isn't going to ack then... */
	if ((cmd->opcode == 7) && (cmd->arg == 0))
		return;

	/*
	 * we must spin until response is ready or timed out
	 * -- we don't get interrupts unless there is a bulk rx
	 */
	do
		status = glamomci_reg_read(host, GLAMO_REG_MMC_RB_STAT1);
	while (((((status >> 15) & 1) != (host->request_counter & 1)) ||
		(!(status & (GLAMO_STAT1_MMC_RB_RRDY |
			     GLAMO_STAT1_MMC_RTOUT |
			     GLAMO_STAT1_MMC_DTOUT |
			     GLAMO_STAT1_MMC_BWERR |
			     GLAMO_STAT1_MMC_BRERR)))) && (timeout--));

	if ((status & (GLAMO_STAT1_MMC_RTOUT | GLAMO_STAT1_MMC_DTOUT)) ||
	    (timeout == 0)) {
		cmd->error = -ETIMEDOUT;
	} else if (status & (GLAMO_STAT1_MMC_BWERR | GLAMO_STAT1_MMC_BRERR)) {
		cmd->error = -EILSEQ;
	}

	if (cmd->flags & MMC_RSP_PRESENT) {
		if (cmd->flags & MMC_RSP_136) {
			cmd->resp[3] = readw(&reg_resp[0]) |
						   (readw(&reg_resp[1]) << 16);
			cmd->resp[2] = readw(&reg_resp[2]) |
						   (readw(&reg_resp[3]) << 16);
			cmd->resp[1] = readw(&reg_resp[4]) |
						   (readw(&reg_resp[5]) << 16);
			cmd->resp[0] = readw(&reg_resp[6]) |
						   (readw(&reg_resp[7]) << 16);
		} else {
			cmd->resp[0] = (readw(&reg_resp[0]) >> 8) |
					   (readw(&reg_resp[1]) << 8) |
					   (readw(&reg_resp[2]) << 24);
		}
	}
}

static int glamo_mci_prepare_pio(struct glamo_mci_host *host,
		struct mmc_data *data)
{
	unsigned long addr = host->data_mem->start;

	if (data->host_cookie & 1)
		addr += resource_size(host->data_mem) / 2;

	/* set up the block info */
	glamomci_reg_write(host, GLAMO_REG_MMC_DATBLKLEN, data->blksz);
	glamomci_reg_write(host, GLAMO_REG_MMC_DATBLKCNT, data->blocks);

	if (data->flags & MMC_DATA_WRITE) {
		glamomci_reg_write(host, GLAMO_REG_MMC_WDATADS1, addr);
		glamomci_reg_write(host, GLAMO_REG_MMC_WDATADS2, addr >> 16);
	} else {
		glamomci_reg_write(host, GLAMO_REG_MMC_RDATADS1, addr);
		glamomci_reg_write(host, GLAMO_REG_MMC_RDATADS2, addr >> 16);
	}

	if ((data->flags & MMC_DATA_WRITE) && !data->host_cookie)
		do_pio_write(host, data);

	dev_dbg(&host->pdev->dev, "(blksz=%d, count=%d)\n",
				   data->blksz, data->blocks);
	return 0;
}

static void glamo_mci_send_request(struct mmc_host *mmc,
		struct mmc_request *mrq)
{
	struct glamo_mci_host *host = mmc_priv(mmc);
	struct mmc_command *cmd = mrq->cmd;

	host->request_counter++;
	host->request_start = jiffies;

	if (cmd->data) {
		if (glamo_mci_prepare_pio(host, cmd->data)) {
			cmd->error = -EIO;
			cmd->data->error = -EIO;
			goto done;
		}
	}

	dev_dbg(&host->pdev->dev, "cmd 0x%x, "
		 "arg 0x%x data=%p mrq->stop=%p flags 0x%x\n",
		 cmd->opcode, cmd->arg, cmd->data, cmd->mrq->stop,
		 cmd->flags);

	glamo_mci_send_command(host, cmd);

	/*
	 * if we don't have bulk data to take care of, we're done
	 */
	if (!cmd->data || cmd->error)
		goto done;

	/*
	 * Otherwise can can use the interrupt as async completion --
	 * if there is read data coming, or we wait for write data to complete,
	 * exit without mmc_request_done() as the payload interrupt
	 * will service it
	 */
	dev_dbg(&host->pdev->dev, "Waiting for payload data\n");
	return;
done:
	if (!cmd->error)
		cmd->error = glamo_mci_wait_idle(host, jiffies + 2 * HZ);
	glamo_mci_request_done(host, mrq);
}

static void glamo_mci_set_power_mode(struct glamo_mci_host *host,
				     unsigned char power_mode)
{
	int ret;

	if (power_mode == host->power_mode)
		return;

	switch (power_mode) {
	case MMC_POWER_UP:
		if (host->power_mode == MMC_POWER_OFF) {
			ret = regulator_enable(host->regulator);
			if (ret)
				dev_err(&host->pdev->dev,
					"Failed to enable regulator: %d\n",
					ret);
		}
		break;
	case MMC_POWER_ON:
		break;
	case MMC_POWER_OFF:
	default:
		ret = regulator_disable(host->regulator);
		if (ret)
			dev_warn(&host->pdev->dev,
				"Failed to disable regulator: %d\n",
				ret);
		break;
	}
	host->power_mode = power_mode;
}

static void glamo_mci_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct glamo_mci_host *host = mmc_priv(mmc);
	int bus_width = 0;
	int rate;
	int sd_drive;
	int ret;

	//mmc_host_enable(mmc); mmc_try_claim_host
	if (__mmc_claim_host(mmc, NULL) != 0) {
		dev_err(&host->pdev->dev, "Failed to claim mmc host\n");
		return;
	}

	/* Set power */
	glamo_mci_set_power_mode(host, ios->power_mode);

	if (host->vdd != ios->vdd) {
		ret = mmc_regulator_set_ocr(mmc, host->regulator, ios->vdd);
		if (ret)
			dev_err(&host->pdev->dev,
				"Failed to set regulator voltage: %d\n", ret);
		else
			host->vdd = ios->vdd;
	}

	rate = glamo_mci_set_card_clock(host, ios->clock);

	if ((ios->power_mode == MMC_POWER_ON) ||
		(ios->power_mode == MMC_POWER_UP)) {
		dev_info(&host->pdev->dev,
			"powered (vdd = %hu) clk: %dkHz div=%hu (req: %ukHz). "
			"Bus width=%d\n", ios->vdd,
			rate / 1000, 0,
			ios->clock / 1000, (int)ios->bus_width);
	} else {
		dev_info(&host->pdev->dev, "glamo_mci_set_ios: power down.\n");
	}

	/* set bus width */
	if (ios->bus_width == MMC_BUS_WIDTH_4)
		bus_width = GLAMO_BASIC_MMC_EN_4BIT_DATA;

	sd_drive = (rate * 4) / host->clk_rate;
	if (sd_drive > 3)
		sd_drive = 3;

	glamomci_reg_set_bit_mask(host, GLAMO_REG_MMC_BASIC,
				GLAMO_BASIC_MMC_EN_4BIT_DATA | 0xc0,
						   bus_width | sd_drive << 6);

	mmc_release_host(mmc);
	/*if (host->power_mode == MMC_POWER_OFF)
		mmc_host_disable(host->mmc);
	else
		mmc_host_lazy_disable(host->mmc);*/
}

static void glamo_mci_pre_request(struct mmc_host *mmc,
		struct mmc_request *mrq, bool is_first_req)
{
	struct glamo_mci_host *host = mmc_priv(mmc);

	mrq->data->host_cookie = (host->request_counter & 1) | 2;

	/* if write, prep the write into the shared RAM before the command */
	if (mrq->data->flags & MMC_DATA_WRITE)
		do_pio_write(host, mrq->data);
}

static void glamo_mci_post_request(struct mmc_host *mmc,
		struct mmc_request *mrq, int err)
{
	struct glamo_mci_host *host = mmc_priv(mmc);

	if (!mrq->data->host_cookie)
		return;

	if (err)
		return;

	if (mrq->data->flags & MMC_DATA_READ)
		do_pio_read(host, mrq->data);

	mrq->data->host_cookie = 0;
}

static struct mmc_host_ops glamo_mci_ops = {
	.enable		= glamo_mci_clock_enable,
	.disable	= glamo_mci_clock_disable,
	.request	= glamo_mci_send_request,
/*	
	.post_req	= glamo_mci_post_request,
	.pre_req	= glamo_mci_pre_request,
*/
	.set_ios	= glamo_mci_set_ios,
};

static int __devinit glamo_mci_probe(struct platform_device *pdev)
{
	struct mmc_host *mmc;
	struct glamo_mci_host *host;
	struct glamo_core *core = dev_get_drvdata(pdev->dev.parent);
	int ret;

	dev_info(&pdev->dev, "glamo_mci driver (C)2007 Openmoko, Inc\n");

	mmc = mmc_alloc_host(sizeof(struct glamo_mci_host), &pdev->dev);
	if (!mmc) {
		ret = -ENOMEM;
		goto probe_out;
	}

	host = mmc_priv(mmc);
	host->mmc = mmc;
	host->pdev = pdev;
	if (core->pdata)
		host->pdata = pdev->dev.platform_data;
	host->power_mode = MMC_POWER_OFF;
	host->core = core;

	host->irq = platform_get_irq(pdev, 0);

	host->regulator = regulator_get(&pdev->dev, "SD_3V3");
	if (IS_ERR(host->regulator)) {
		dev_err(&pdev->dev, "Cannot proceed without regulator.\n");
		ret = PTR_ERR(host->regulator);
		goto probe_free_host;
	}

	host->mmio_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!host->mmio_mem) {
		dev_err(&pdev->dev,
			"failed to get io memory region resouce.\n");
		ret = -ENOENT;
		goto probe_regulator_put;
	}

	host->mmio_mem = request_mem_region(host->mmio_mem->start,
					    resource_size(host->mmio_mem),
					    pdev->name);

	if (!host->mmio_mem) {
		dev_err(&pdev->dev, "failed to request io memory region.\n");
		ret = -ENOENT;
		goto probe_regulator_put;
	}

	host->mmio_base = ioremap(host->mmio_mem->start,
				  resource_size(host->mmio_mem));
	if (!host->mmio_base) {
		dev_err(&pdev->dev, "failed to ioremap() io memory region.\n");
		ret = -EINVAL;
		goto probe_free_mem_region_mmio;
	}


	/* Get ahold of our data buffer we use for data in and out on MMC */
	host->data_mem = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!host->data_mem) {
		dev_err(&pdev->dev,
			"failed to get io memory region resource.\n");
		ret = -ENOENT;
		goto probe_iounmap_mmio;
	}

	host->data_mem = request_mem_region(host->data_mem->start,
					    resource_size(host->data_mem),
					    pdev->name);

	if (!host->data_mem) {
		dev_err(&pdev->dev, "failed to request io memory region.\n");
		ret = -ENOENT;
		goto probe_iounmap_mmio;
	}
	host->data_base = ioremap(host->data_mem->start,
				  resource_size(host->data_mem));

	if (host->data_base == 0) {
		dev_err(&pdev->dev, "failed to ioremap() io memory region.\n");
		ret = -EINVAL;
		goto probe_free_mem_region_data;
	}

	ret = request_threaded_irq(host->irq, NULL, glamo_mci_irq, IRQF_SHARED,
				   pdev->name, host);
	if (ret) {
		dev_err(&pdev->dev, "failed to register irq.\n");
		goto probe_iounmap_data;
	}


	host->vdd = 0;
	host->clk_rate = glamo_pll_rate(host->core, GLAMO_PLL1);

	/* explain our host controller capabilities */
	mmc->ops	= &glamo_mci_ops;
	mmc->ocr_avail	= mmc_regulator_get_ocrmask(host->regulator);
	mmc->caps	= MMC_CAP_4_BIT_DATA |
			    MMC_CAP_MMC_HIGHSPEED |
			    MMC_CAP_SD_HIGHSPEED;

	if (host->pdata->nonremovable)
		mmc->caps |= MMC_CAP_NONREMOVABLE;

	mmc->f_min	= host->clk_rate / 256;
	mmc->f_max	= sd_max_clk;

	mmc->max_blk_count = (1 << 16) - 1; /* GLAMO_REG_MMC_RB_BLKCNT */
	mmc->max_blk_size  = (1 << 12) - 1; /* GLAMO_REG_MMC_RB_BLKLEN */
	mmc->max_req_size  = resource_size(host->data_mem) / 2;
	mmc->max_seg_size  = mmc->max_req_size;
	mmc->max_segs = 128;

	if (mmc->ocr_avail < 0) {
		dev_warn(&pdev->dev,
			"Failed to get ocr list for regulator: %d.\n",
			mmc->ocr_avail);
		mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34;
	}

	platform_set_drvdata(pdev, mmc);

	//mmc_set_disable_delay(mmc, 1000 / 16);

	//mmc_host_enable(mmc);
	__mmc_claim_host(mmc, NULL);
	if (ret != 0) {
		dev_err(&host->pdev->dev, "Failed to claim mmc host\n");
		goto probe_iounmap_data;
	}
	glamo_mci_reset(host);

	ret = mmc_add_host(mmc);
	if (ret) {
		dev_err(&pdev->dev, "failed to add mmc host.\n");
		goto probe_mmc_host_disable;
	}

	mmc_release_host(mmc);

	return 0;

probe_mmc_host_disable:
	mmc_release_host(mmc);
	free_irq(host->irq, host);
probe_iounmap_data:
	iounmap(host->data_base);
probe_free_mem_region_data:
	release_mem_region(host->data_mem->start,
			   resource_size(host->data_mem));
probe_iounmap_mmio:
	iounmap(host->mmio_base);
probe_free_mem_region_mmio:
	release_mem_region(host->mmio_mem->start,
			   resource_size(host->mmio_mem));
probe_regulator_put:
	regulator_put(host->regulator);
probe_free_host:
	mmc_free_host(mmc);
probe_out:
	return ret;
}

static int __devexit glamo_mci_remove(struct platform_device *pdev)
{
	struct mmc_host	*mmc = platform_get_drvdata(pdev);
	struct glamo_mci_host *host = mmc_priv(mmc);

	//mmc_host_enable(mmc);
	if (__mmc_claim_host(mmc, NULL) == 0) {
		mmc_remove_host(mmc);
		mmc_release_host(mmc);
	}
	//mmc_host_disable(mmc);

	synchronize_irq(host->irq);
	free_irq(host->irq, host);

	iounmap(host->mmio_base);
	iounmap(host->data_base);
	release_mem_region(host->mmio_mem->start,
				resource_size(host->mmio_mem));
	release_mem_region(host->data_mem->start,
				resource_size(host->data_mem));

	regulator_put(host->regulator);

	mmc_free_host(mmc);

	return 0;
}


#ifdef CONFIG_PM

static int glamo_mci_suspend(struct device *dev)
{
	struct mmc_host *mmc = dev_get_drvdata(dev);
	struct glamo_mci_host *host = mmc_priv(mmc);
	int ret;

	disable_irq(host->irq);

	ret = __mmc_claim_host(mmc, NULL);
	if (ret == 0) {
		ret = mmc_suspend_host(mmc);
		mmc_release_host(mmc);
	}

	return ret;
}

static int glamo_mci_resume(struct device *dev)
{
	struct mmc_host *mmc = dev_get_drvdata(dev);
	struct glamo_mci_host *host = mmc_priv(mmc);
	int ret;

	ret = __mmc_claim_host(mmc, NULL);
	if (ret != 0)
		return ret;

	glamo_mci_reset(host);
	mdelay(10);

	enable_irq(host->irq);

	ret = mmc_resume_host(host->mmc);

	mmc_release_host(mmc);

	return ret;
}

static SIMPLE_DEV_PM_OPS(glamo_mci_pm_ops, glamo_mci_suspend, glamo_mci_resume);
#define GLAMO_MCI_PM_OPS (&glamo_mci_pm_ops)

#else /* CONFIG_PM */
#define GLAMO_MCI_PM_OPS NULL
#endif /* CONFIG_PM */


static struct platform_driver glamo_mci_driver = {
	.probe  = glamo_mci_probe,
	.remove = __devexit_p(glamo_mci_remove),
	.driver = {
		.name	= "glamo-mci",
		.owner	= THIS_MODULE,
		.pm	= GLAMO_MCI_PM_OPS,
	},
};

static int __init glamo_mci_init(void)
{
	platform_driver_register(&glamo_mci_driver);
	return 0;
}
module_init(glamo_mci_init);

static void __exit glamo_mci_exit(void)
{
	platform_driver_unregister(&glamo_mci_driver);
}
module_exit(glamo_mci_exit);

MODULE_DESCRIPTION("Glamo MMC/SD Card Interface driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andy Green <andy@openmoko.com>");
MODULE_ALIAS("platform:glamo-mci");
