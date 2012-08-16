/*
 * HDQ generic GPIO bitbang driver using FIQ
 *
 * (C) 2006-2007 by Openmoko, Inc.
 * Author: Andy Green <andy@openmoko.com>
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/slab.h>

#include <asm/fiq.h>
#include <linux/power/fiq-hdq.h>


#define HDQ_READ 0

enum hdq_bitbang_states {
	HDQB_IDLE = 0,
	HDQB_TX_BREAK,
	HDQB_TX_BREAK_RECOVERY,
	HDQB_ADS_CALC,
	HDQB_ADS_LOW,
	HDQB_ADS_HIGH,
	HDQB_WAIT_RX,
	HDQB_DATA_RX_LOW,
	HDQB_DATA_RX_HIGH,
	HDQB_WAIT_TX,
};

struct hdq_priv {
	struct fiq_hdq_ops ops;
	struct mutex lock; /* if you want to use hdq, you have to take lock */
	u8 hdq_probed; /* nonzero after HDQ driver probed */
	u8 hdq_ads; /* b7..b6 = register address, b0 = r/w */
	u8 hdq_tx_data; /* data to tx for write action */
	u8 hdq_rx_data; /* data received in read action */
	u8 hdq_request_ctr; /* incremented by "user" to request a transfer */
	u8 hdq_transaction_ctr; /* incremented after each transfer */
	u8 hdq_error; /* 0 = no error */
	u8 hdq_ctr;
	u8 hdq_ctr2;
	u8 hdq_bit;
	u8 hdq_shifter;
	u8 hdq_tx_data_done;
	
	u8 fiq_claimed;
	enum hdq_bitbang_states hdq_state;
	int reported_error;
	
	struct fiq_handler fiq_handler;

	struct pwm_device *fiq_timer;
	struct fiq_hdq_platform_data *pdata;
	
	/*  we need to know that this is allocated by kmalloc  */
	unsigned int timer_irq;
	void (*handle_fiq)(unsigned long irq, int disable);
};


/* -------- FIQ handler in C --------- */
#define FIQ_C_ISR_STACK_SIZE 	256

static void __attribute__((naked)) __jump_to_isr(void)
{
	asm __volatile__ ("mov pc, r8");
}


static void __attribute__((naked)) __actual_isr(void)
{
	register void (*isr)(unsigned long) asm ("r9");
	register unsigned long data asm ("r10");
	
	asm __volatile__ (
		"stmdb	sp!, {r0-r12, lr};"
		"mov     fp, sp;"
	);
	
	isr(data);

	asm __volatile__ (
		"ldmia	sp!, {r0-r12, lr};"
		"subs	pc, lr, #4;"
	);

	return;
}

void set_fiq_c_handler(void (*isr)(unsigned long), unsigned long data)
{
	static void * handler = NULL;
	struct pt_regs regs;
	
	/* TODO: Properly handle this */
	if (!isr || data == 0) {
		kfree(handler);
		handler = NULL;
		set_fiq_handler(NULL, 0);
		return;
	}
	
	handler = kmalloc(4, GFP_KERNEL);
	memcpy(handler, __jump_to_isr, 4);

	memset(&regs, 0, sizeof(regs));
	regs.ARM_r8 = (unsigned long) __actual_isr;
	regs.ARM_r9 = (unsigned long) isr;
	regs.ARM_r10 = (unsigned long) data;
	regs.ARM_sp = 0xffff001c + FIQ_C_ISR_STACK_SIZE;

	set_fiq_handler(handler, 4);

	set_fiq_regs(&regs);
}
/* -------- FIQ handler in C ---------*/


static void hdq_bad(struct hdq_priv *hdq)
{
	if (!hdq->reported_error)
		printk(KERN_ERR "HDQ error: %d\n", hdq->hdq_error);
	hdq->reported_error = 1;
}

static void hdq_good(struct hdq_priv *hdq)
{
	if (hdq->reported_error)
		printk(KERN_INFO "HDQ responds again\n");
	hdq->reported_error = 0;
}

/**
 * hdq_fiqop - FIQ core code callback
 * @pw: Data registered with the handler
 * @release: Whether this is a release or a return.
 *
 * Called by the FIQ code when another module wants to use the FIQ, so
 * return whether we are currently using this or not and then update our
 * internal state.
 */
static int hdq_fiqop(void *pw, int release)
{
	struct hdq_priv *hdq = pw;
	int ret = 0;

	if (release) {
		if (hdq->fiq_claimed)
			ret = -EBUSY;
		   
		/* note, we do not need to unroute the FIQ, as the FIQ
		* vector code de-routes it to signal the end of transfer */
		hdq->fiq_claimed = 0;
	} else {
		hdq->fiq_claimed = 1;
	}

	return ret;
}

static void hdq_fiq_handler(unsigned long data)
{
	struct hdq_priv *hdq = (struct hdq_priv*) data;
	u8 disable;
	
	if (!hdq->hdq_probed) {
		disable = 1;
		goto done;
	}

	switch (hdq->hdq_state) {
	case HDQB_IDLE:
		if (hdq->hdq_request_ctr == hdq->hdq_transaction_ctr)
			break;
		hdq->hdq_ctr = 250 / HDQ_SAMPLE_PERIOD_US;
		hdq->pdata->gpio_set(hdq->pdata->gpio, 0);
		hdq->pdata->gpio_dir_out(hdq->pdata->gpio, hdq->pdata->gpio_output);
		hdq->hdq_tx_data_done = 0;
		hdq->hdq_state = HDQB_TX_BREAK;
		break;

	case HDQB_TX_BREAK: /* issue low for > 190us */
		if (--hdq->hdq_ctr == 0) {
			hdq->hdq_ctr = 60 / HDQ_SAMPLE_PERIOD_US;
			hdq->hdq_state = HDQB_TX_BREAK_RECOVERY;
			hdq->pdata->gpio_set(hdq->pdata->gpio, 1);
		}
		break;

	case HDQB_TX_BREAK_RECOVERY: /* issue low for > 40us */
		if (--hdq->hdq_ctr)
			break;
		hdq->hdq_shifter = hdq->hdq_ads;
		hdq->hdq_bit = 8; /* 8 bits of ads / rw */
		hdq->hdq_tx_data_done = 0; /* doing ads */
		/* fallthru on last one */
	case HDQB_ADS_CALC:
		if (hdq->hdq_shifter & 1)
			hdq->hdq_ctr = 50 / HDQ_SAMPLE_PERIOD_US;
		else
			hdq->hdq_ctr = 120 / HDQ_SAMPLE_PERIOD_US;
		/* carefully precompute the other phase length */
		hdq->hdq_ctr2 = (210 - (hdq->hdq_ctr *
				HDQ_SAMPLE_PERIOD_US)) / HDQ_SAMPLE_PERIOD_US;
		hdq->hdq_state = HDQB_ADS_LOW;
		hdq->hdq_shifter >>= 1;
		hdq->hdq_bit--;
		hdq->pdata->gpio_set(hdq->pdata->gpio, 0);
		break;

	case HDQB_ADS_LOW:
		if (--hdq->hdq_ctr)
			break;
		hdq->pdata->gpio_set(hdq->pdata->gpio, 1);
		hdq->hdq_state = HDQB_ADS_HIGH;
		break;

	case HDQB_ADS_HIGH:
		if (--hdq->hdq_ctr2 > 1) /* account for HDQB_ADS_CALC */
			break;
		if (hdq->hdq_bit) { /* more bits to do */
			hdq->hdq_state = HDQB_ADS_CALC;
			break;
		}
		/* no more bits, wait until hdq->hdq_ctr2 exhausted */
		if (hdq->hdq_ctr2)
			break;
		/* ok no more bits and very last state */
		hdq->hdq_ctr = 60 / HDQ_SAMPLE_PERIOD_US;
		/* FIXME 0 = read */
		if (hdq->hdq_ads & 0x80) { /* write the byte out */
			 /* set delay before payload */
			hdq->hdq_ctr = 300 / HDQ_SAMPLE_PERIOD_US;
			/* already high, no need to write */
			hdq->hdq_state = HDQB_WAIT_TX;
			break;
		}
		/* read the next byte */
		hdq->hdq_bit = 8; /* 8 bits of data */
		hdq->hdq_ctr = 2500 / HDQ_SAMPLE_PERIOD_US;
		hdq->hdq_state = HDQB_WAIT_RX;
		hdq->pdata->gpio_dir_in(hdq->pdata->gpio, hdq->pdata->gpio_input);
		break;

	case HDQB_WAIT_TX: /* issue low for > 40us */
		if (--hdq->hdq_ctr)
			break;
		if (!hdq->hdq_tx_data_done) { /* was that the data sent? */
			hdq->hdq_tx_data_done++;
			hdq->hdq_shifter = hdq->hdq_tx_data;
			hdq->hdq_bit = 8; /* 8 bits of data */
			hdq->hdq_state = HDQB_ADS_CALC; /* start sending */
			break;
		}
		hdq->hdq_error = 0;
		hdq->hdq_transaction_ctr = hdq->hdq_request_ctr;
		hdq->hdq_state = HDQB_IDLE; /* all tx is done */
		/* idle in input mode, it's pulled up by 10K */
		hdq->pdata->gpio_dir_in(hdq->pdata->gpio, hdq->pdata->gpio_input);
		break;

	case HDQB_WAIT_RX: /* wait for battery to talk to us */
		if (hdq->pdata->gpio_get(hdq->pdata->gpio) == 0) {
			/* it talks to us! */
			hdq->hdq_ctr2 = 1;
			hdq->hdq_bit = 8; /* 8 bits of data */
			/* timeout */
			hdq->hdq_ctr = 500 / HDQ_SAMPLE_PERIOD_US;
			hdq->hdq_state = HDQB_DATA_RX_LOW;
			break;
		}
		if (--hdq->hdq_ctr == 0) { /* timed out, error */
			hdq->hdq_error = 1;
			hdq->hdq_transaction_ctr = hdq->hdq_request_ctr;
			hdq->hdq_state = HDQB_IDLE; /* abort */
		}
		break;

	/*
	 * HDQ basically works by measuring the low time of the bit cell
	 * 32-50us --> '1', 80 - 145us --> '0'
	 */

	case HDQB_DATA_RX_LOW:
		if (hdq->pdata->gpio_get(hdq->pdata->gpio)) {
			hdq->hdq_rx_data >>= 1;
			if (hdq->hdq_ctr2 <= (65 / HDQ_SAMPLE_PERIOD_US))
				hdq->hdq_rx_data |= 0x80;

			if (--hdq->hdq_bit == 0) {
				hdq->hdq_error = 0;
				hdq->hdq_transaction_ctr =
						hdq->hdq_request_ctr;

				hdq->hdq_state = HDQB_IDLE;
			} else
				hdq->hdq_state = HDQB_DATA_RX_HIGH;
			/* timeout */
			hdq->hdq_ctr = 1000 / HDQ_SAMPLE_PERIOD_US;
			hdq->hdq_ctr2 = 1;
			break;
		}
		hdq->hdq_ctr2++;
		if (--hdq->hdq_ctr)
			break;
		 /* timed out, error */
		hdq->hdq_error = 2;
		hdq->hdq_transaction_ctr = hdq->hdq_request_ctr;
		hdq->hdq_state = HDQB_IDLE; /* abort */
		break;

	case HDQB_DATA_RX_HIGH:
		if (!hdq->pdata->gpio_get(hdq->pdata->gpio)) {
			/* it talks to us! */
			hdq->hdq_ctr2 = 1;
			/* timeout */
			hdq->hdq_ctr = 400 / HDQ_SAMPLE_PERIOD_US;
			hdq->hdq_state = HDQB_DATA_RX_LOW;
			break;
		}
		if (--hdq->hdq_ctr)
			break;
		/* timed out, error */
		hdq->hdq_error = 3;
		hdq->hdq_transaction_ctr = hdq->hdq_request_ctr;

		/* we're in input mode already */
		hdq->hdq_state = HDQB_IDLE; /* abort */
		break;
	}
	
	disable = (hdq->hdq_state == HDQB_IDLE);
done:
	/* Are we interested in keeping the FIQ source alive ? */
	hdq->pdata->handle_fiq(hdq->timer_irq, disable);
	return;
}

static int fiq_busy(struct hdq_priv *hdq)
{
	int request;
	int transact;

	request = (volatile u8)hdq->hdq_request_ctr;
	transact = (volatile u8)hdq->hdq_transaction_ctr;
	return (request != transact);
}

int fiq_hdq_read(struct device *dev, unsigned int address)
{
	int count_sleeps = 5;
	int ret = -ETIME;
	struct hdq_priv *hdq = platform_get_drvdata(to_platform_device(dev));
	
	if (!hdq->hdq_probed)
		return -EINVAL;
	mutex_lock(&hdq->lock);
	
	ret = claim_fiq(&hdq->fiq_handler);
	if (ret) {
		printk(KERN_ERR "HDQ error: unable to claim FIQ\n");
		goto unlock;
	}
	hdq->fiq_claimed = 1;
	set_fiq_c_handler(hdq_fiq_handler, (unsigned long) hdq);
	hdq->pdata->set_fiq(hdq->timer_irq, true);
	pwm_enable(hdq->fiq_timer);
	
	hdq->hdq_error = 0;
	hdq->hdq_ads = address | HDQ_READ;
	hdq->hdq_request_ctr++;
	hdq->pdata->kick_fiq(hdq->timer_irq);
	
	/*
	 * FIQ takes care of it while we block our calling process
	 * But we're not spinning -- other processes run normally while
	 * we wait for the result
	 */
	while (count_sleeps--) {
		msleep(10); /* valid transaction always completes in < 10ms */

		if (fiq_busy(hdq))
			continue;

		if (hdq->hdq_error) {
			hdq_bad(hdq);
			goto done; /* didn't see a response in good time */
		}
		hdq_good(hdq);

		ret = hdq->hdq_rx_data;
		goto done;
	}

done:
	hdq->pdata->set_fiq(hdq->timer_irq, false);
	set_fiq_c_handler(NULL, 0);
	pwm_disable(hdq->fiq_timer);
	release_fiq(&hdq->fiq_handler);
	hdq->fiq_claimed = 0;
	//gta02_hdq_fiq_release(&hdq_priv);
unlock:
	mutex_unlock(&hdq->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(fiq_hdq_read);

/* sysfs */

static ssize_t hdq_sysfs_dump(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	int n;
	int v;
	u8 u8a[128]; /* whole address space for HDQ */
	char *end = buf;
	struct hdq_priv *hdq = platform_get_drvdata(to_platform_device(dev));

	if (!hdq->hdq_probed)
		return -EINVAL;

	/* the dump does not take care about 16 bit regs, because at this
	 * bus level we don't know about the chip details
	 */
	for (n = 0; n < sizeof(u8a); n++) {
		v = fiq_hdq_read(dev, n);
		if (v < 0)
			goto bail;
		u8a[n] = v;
	}

	for (n = 0; n < sizeof(u8a); n += 16) {
		hex_dump_to_buffer(u8a + n, sizeof(u8a), 16, 1, end, 4096, 0);
		end += strlen(end);
		*end++ = '\n';
		*end = '\0';
	}
	return end - buf;

bail:
	return sprintf(buf, "ERROR %d\n", v);
}


static DEVICE_ATTR(dump, 0400, hdq_sysfs_dump, NULL);

#ifdef CONFIG_PM
static int hdq_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct hdq_priv *hdq = platform_get_drvdata(pdev);

	/* after 18s of this, the battery monitor will also go to sleep */
	hdq->pdata->gpio_dir_in(hdq->pdata->gpio, hdq->pdata->gpio_input);
	
	if (hdq->fiq_claimed) {
		hdq->pdata->set_fiq(hdq->timer_irq, false);
		set_fiq_c_handler(NULL, 0);
		pwm_disable(hdq->fiq_timer);
		release_fiq(&hdq->fiq_handler);
		hdq->fiq_claimed = 0;
		hdq->hdq_state = HDQB_IDLE;
		hdq->hdq_transaction_ctr = hdq->hdq_request_ctr;
		hdq->hdq_error = 0;
	}
	
	return 0;
}

static int hdq_resume(struct platform_device *pdev)
{
	struct hdq_priv *hdq = platform_get_drvdata(pdev);

	hdq->pdata->gpio_set(hdq->pdata->gpio, 1);
	hdq->pdata->gpio_dir_out(hdq->pdata->gpio, hdq->pdata->gpio_output);

	return 0;
}
#endif

static int __devinit hdq_probe(struct platform_device *pdev)
{
	struct fiq_hdq_platform_data *pdata = pdev->dev.platform_data;
	struct hdq_priv *hdq;
	int ret;

	if (!pdata) {
		dev_err(&pdev->dev, "no platform data supplied\n");
		return -EINVAL;
	}

	hdq = kzalloc(sizeof(struct hdq_priv), GFP_KERNEL);
	if (!hdq) {
		return -ENOMEM;
	}

	mutex_init(&hdq->lock);

	/* set our HDQ comms pin from the platform data */
	hdq->pdata = pdata;
	hdq->ops.read = fiq_hdq_read;
	
	hdq->handle_fiq = pdata->handle_fiq;
	hdq->fiq_handler.dev_id = hdq;
	hdq->fiq_handler.name = dev_name(&pdev->dev);
	hdq->fiq_handler.fiq_op = &hdq_fiqop;
	hdq->timer_irq = pdata->timer_irq;
	hdq->fiq_claimed = 0;
	platform_set_drvdata(pdev, hdq);

	hdq->pdata->gpio_set(hdq->pdata->gpio, 1);
	hdq->pdata->gpio_dir_out(hdq->pdata->gpio, hdq->pdata->gpio_output);
	
	hdq->fiq_timer = pwm_request(pdata->timer_id, "fiq timer");
	if (IS_ERR(hdq->fiq_timer)) {
		ret = PTR_ERR(hdq->fiq_timer);
		dev_err(&pdev->dev, "Could not request fiq timer: %d\n", ret);
		return ret;
	}

	ret = pwm_config(hdq->fiq_timer, HDQ_SAMPLE_PERIOD_US * 1000,
					HDQ_SAMPLE_PERIOD_US * 1000);
	if (ret) {
		dev_err(&pdev->dev, "Could not configure fiq timer: %d\n", ret);
		pwm_free(hdq->fiq_timer);
		return ret;
	}
	ret = device_create_file(&pdev->dev, &dev_attr_dump);
	if (ret)
		return ret;
	
	hdq->hdq_probed = 1; /* we are ready to do stuff now */

	hdq->pdata = pdata;

	return 0;
}

static int __devexit hdq_remove(struct platform_device *pdev)
{
	struct hdq_priv *hdq = platform_get_drvdata(pdev);

	if (hdq->fiq_claimed) {
		hdq->pdata->set_fiq(hdq->timer_irq, false);
		set_fiq_c_handler(NULL, 0);
		pwm_disable(hdq->fiq_timer);
		release_fiq(&hdq->fiq_handler);
	}

	pwm_free(hdq->fiq_timer);
	device_remove_file(&pdev->dev, &dev_attr_dump);
	kfree(hdq);
	return 0;
}

static struct platform_driver hdq_driver = {
	.probe		= hdq_probe,
	.remove		= __devexit_p(hdq_remove),
#ifdef CONFIG_PM
	.suspend	= hdq_suspend,
	.resume		= hdq_resume,
#endif
	.driver		= {
		.name	= "fiq-hdq",
		.owner  = THIS_MODULE,
	},
};

/*
 * Module stuff
 */

static int __init hdq_init(void)
{
	return platform_driver_register(&hdq_driver);
}
module_init(hdq_init);

static void __exit hdq_exit(void)
{
	platform_driver_unregister(&hdq_driver);
}
module_exit(hdq_exit);

MODULE_AUTHOR("Isaac Gordezky <eye.zak@gmail.com>");
MODULE_DESCRIPTION("FIQ-based HDQ driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:fiq-hdq");
