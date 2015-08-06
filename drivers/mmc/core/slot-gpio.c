/*
 * Generic GPIO card-detect helper
 *
 * Copyright (C) 2011, Guennadi Liakhovetski <g.liakhovetski@gmx.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/mmc/host.h>
#include <linux/mmc/slot-gpio.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/ratelimit.h>
#include <linux/delay.h>
#include <linux/irq.h>

struct workqueue_struct *enable_detection_workqueue = NULL;	
unsigned int disable_auto_sd = 0;

struct mmc_gpio {
	int ro_gpio;
	int cd_gpio;
	char *ro_label;
	bool status;
	char cd_label[0]; 
};


static int __init nsb_setup(char *this_opt){

	if (!this_opt || !*this_opt)
		return 1;

	if(*this_opt == '1')
		disable_auto_sd = 1;

	return 0;
}

__setup("NSB=", nsb_setup);

void mmc_enable_detection(struct work_struct *work)
{
	struct mmc_host *host = container_of(work, struct mmc_host, enable_detect.work);

	enable_irq(host->slot.cd_irq);
	spin_lock(&host->lock_cd_pin);
	host->cd_pin_depth--;
	spin_unlock(&host->lock_cd_pin);
	printk("%s %s leave\n", mmc_hostname(host), __func__);
}
EXPORT_SYMBOL(mmc_enable_detection);

int mmc_gpio_get_status(struct mmc_host *host)
{
	int ret = -ENOSYS;
	struct mmc_gpio *ctx = host->slot.handler_priv;

	if (!ctx || !gpio_is_valid(ctx->cd_gpio))
		goto out;

	ret = !gpio_get_value_cansleep(ctx->cd_gpio) ^
		!!(host->caps2 & MMC_CAP2_CD_ACTIVE_HIGH);
out:
	return ret;
}
EXPORT_SYMBOL(mmc_gpio_get_status);

int mmc_gpio_send_uevent(struct mmc_host *host)
{
	char *envp[2];
	char state_string[16];
	int status;

	status = mmc_gpio_get_status(host);
	if (unlikely(status < 0))
		goto out;

	snprintf(state_string, sizeof(state_string), "SWITCH_STATE=%d", status);
	envp[0] = state_string;
	envp[1] = NULL;
	kobject_uevent_env(&host->class_dev.kobj, KOBJ_ADD, envp);

out:
	return status;
}

static irqreturn_t mmc_gpio_cd_irqt(int irq, void *dev_id)
{
	
	struct mmc_host *host = dev_id;
	struct mmc_gpio *ctx = host->slot.handler_priv;
	int status;

	spin_lock(&host->lock_cd_pin);
	if (host->cd_pin_depth == 0) {
		if (enable_detection_workqueue) {
			disable_irq_nosync(host->slot.cd_irq);
			queue_delayed_work(enable_detection_workqueue, &host->enable_detect, msecs_to_jiffies(50));
			host->cd_pin_depth++;
		} else
			pr_err("%s detection workqueue is null\n", mmc_hostname(host));
	}
	spin_unlock(&host->lock_cd_pin);

	if (!host->ops)
		goto out;

	if (host->ops->card_event)
		host->ops->card_event(host);

	status = mmc_gpio_get_status(host);
	if (unlikely(status < 0))
		goto out;

	if (status ^ ctx->status) {
		pr_info("%s: slot status change detected (%d -> %d), GPIO_ACTIVE_%s\n",
				mmc_hostname(host), ctx->status, status,
				(host->caps2 & MMC_CAP2_CD_ACTIVE_HIGH) ?
				"HIGH" : "LOW");
		irq_set_irq_type(irq, (status == 0 ?
					IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING));
		ctx->status = status;
		
		host->caps |= host->caps_uhs;
		
		host->removed_cnt = 0;
		host->crc_count = 0;
		
		if (disable_auto_sd)
			goto out;
		
		mmc_detect_change(host, msecs_to_jiffies(host->extended_debounce + 200));
		mmc_gpio_send_uevent(host);
	}
out:

	return IRQ_HANDLED;
}

static int mmc_gpio_alloc(struct mmc_host *host)
{
	size_t len = strlen(dev_name(host->parent)) + 4;
	struct mmc_gpio *ctx;

	mutex_lock(&host->slot.lock);

	ctx = host->slot.handler_priv;
	if (!ctx) {
		ctx = devm_kzalloc(&host->class_dev, sizeof(*ctx) + 2 * len,
				   GFP_KERNEL);
		if (ctx) {
			ctx->ro_label = ctx->cd_label + len;
			snprintf(ctx->cd_label, len, "%s cd", dev_name(host->parent));
			snprintf(ctx->ro_label, len, "%s ro", dev_name(host->parent));
			ctx->cd_gpio = -EINVAL;
			ctx->ro_gpio = -EINVAL;
			host->slot.handler_priv = ctx;
		}
	}

	mutex_unlock(&host->slot.lock);

	return ctx ? 0 : -ENOMEM;
}

int mmc_gpio_get_ro(struct mmc_host *host)
{
	struct mmc_gpio *ctx = host->slot.handler_priv;

	if (!ctx || !gpio_is_valid(ctx->ro_gpio))
		return -ENOSYS;

	return !gpio_get_value_cansleep(ctx->ro_gpio) ^
		!!(host->caps2 & MMC_CAP2_RO_ACTIVE_HIGH);
}
EXPORT_SYMBOL(mmc_gpio_get_ro);

int mmc_gpio_get_cd(struct mmc_host *host)
{
	struct mmc_gpio *ctx = host->slot.handler_priv;

	if (!ctx || !gpio_is_valid(ctx->cd_gpio))
		return -ENOSYS;

	return !gpio_get_value_cansleep(ctx->cd_gpio) ^
		!!(host->caps2 & MMC_CAP2_CD_ACTIVE_HIGH);
}
EXPORT_SYMBOL(mmc_gpio_get_cd);

int mmc_gpio_request_ro(struct mmc_host *host, unsigned int gpio)
{
	struct mmc_gpio *ctx;
	int ret;

	if (!gpio_is_valid(gpio))
		return -EINVAL;

	ret = mmc_gpio_alloc(host);
	if (ret < 0)
		return ret;

	ctx = host->slot.handler_priv;

	ret = devm_gpio_request_one(&host->class_dev, gpio, GPIOF_DIR_IN,
				    ctx->ro_label);
	if (ret < 0)
		return ret;

	ctx->ro_gpio = gpio;

	return 0;
}
EXPORT_SYMBOL(mmc_gpio_request_ro);

int mmc_gpio_request_cd(struct mmc_host *host, unsigned int gpio)
{
	struct mmc_gpio *ctx;
	int irq = gpio_to_irq(gpio);
	int ret;
	int irq_trigger_type;

	ret = mmc_gpio_alloc(host);
	if (ret < 0)
		return ret;

	host->cd_pin_depth = 0;
	spin_lock_init(&host->lock_cd_pin);

	ctx = host->slot.handler_priv;

	ret = devm_gpio_request_one(&host->class_dev, gpio, GPIOF_DIR_IN,
				    ctx->cd_label);
	if (ret < 0) {
		return ret;
	}

	if (irq >= 0 && host->caps & MMC_CAP_NEEDS_POLL)
		irq = -EINVAL;

	ctx->cd_gpio = gpio;
	host->slot.cd_irq = irq;

	ret = mmc_gpio_get_status(host);
	if (ret < 0)
		return ret;

	irq_trigger_type = (ret == 0 ? IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING);
	ctx->status = ret;

	if (irq >= 0) {
		ret = devm_request_threaded_irq(&host->class_dev, irq,
			NULL, mmc_gpio_cd_irqt,
			irq_trigger_type | IRQF_ONESHOT,
			ctx->cd_label, host);
		if (ret < 0)
			irq = ret;
	}

	if (irq < 0)
		host->caps |= MMC_CAP_NEEDS_POLL;

	return 0;
}
EXPORT_SYMBOL(mmc_gpio_request_cd);

void mmc_gpio_free_ro(struct mmc_host *host)
{
	struct mmc_gpio *ctx = host->slot.handler_priv;
	int gpio;

	if (!ctx || !gpio_is_valid(ctx->ro_gpio))
		return;

	gpio = ctx->ro_gpio;
	ctx->ro_gpio = -EINVAL;

	devm_gpio_free(&host->class_dev, gpio);
}
EXPORT_SYMBOL(mmc_gpio_free_ro);

void mmc_gpio_free_cd(struct mmc_host *host)
{
	struct mmc_gpio *ctx = host->slot.handler_priv;
	int gpio;

	if (!ctx || !gpio_is_valid(ctx->cd_gpio))
		return;

	if (host->slot.cd_irq >= 0) {
		devm_free_irq(&host->class_dev, host->slot.cd_irq, host);
		host->slot.cd_irq = -EINVAL;
	}

	gpio = ctx->cd_gpio;
	ctx->cd_gpio = -EINVAL;

	devm_gpio_free(&host->class_dev, gpio);
}
EXPORT_SYMBOL(mmc_gpio_free_cd);
