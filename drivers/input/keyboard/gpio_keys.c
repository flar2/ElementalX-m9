/*
 * Driver for keys on GPIO lines capable of generating interrupts.
 *
 * Copyright 2005 Phil Blundell
 * Copyright 2010, 2011 David Jander <david@protonic.nl>
 * Copyright (C) 2014 HTC Corporation.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/spinlock.h>
#include <linux/pinctrl/consumer.h>

enum {
	DEBOUNCE_WAIT_IRQ,	
	DEBOUNCE_UNSTABLE_IRQ,	
	DEBOUNCE_UNKNOWN_STATE,
	DEBOUNCE_CHECKING_STATE,
	DEBOUNCE_DONE,
};

struct gpio_button_data {
	const struct gpio_keys_button *button;
	struct input_dev *input;
	struct timer_list timer;
	struct work_struct work;
	unsigned int timer_debounce;	
	unsigned int irq;
	spinlock_t lock;
	bool disabled;
	bool key_pressed;
	unsigned char bouncing_flag;
	bool prev_gpio_level;
};

struct gpio_keys_drvdata {
	const struct gpio_keys_platform_data *pdata;
	struct pinctrl *key_pinctrl;
	struct input_dev *input;
	struct mutex disable_lock;
	struct mutex attr_operation_lock;
	unsigned char wakeup_bitmask;
	unsigned char set_wakeup;
	struct gpio_button_data data[0];
};

static unsigned int vol_up_irq;
static unsigned int vol_down_irq;
#ifdef CONFIG_POWER_KEY_EID
struct gpio_button_data *gb_data;
#endif

static ssize_t vol_wakeup_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	unsigned char bitmask = 0;
	struct gpio_keys_drvdata *ddata = dev_get_drvdata(dev);
	bitmask = simple_strtoull(buf, NULL, 10);

	mutex_lock(&ddata->attr_operation_lock);
	if (bitmask) {
		if (bitmask == 127)
			ddata->wakeup_bitmask &= bitmask;
		else if (bitmask > 128)
			ddata->wakeup_bitmask &= bitmask;
		else
			ddata->wakeup_bitmask |= bitmask;
	}

	if (ddata->wakeup_bitmask && (!ddata->set_wakeup)) {
		enable_irq_wake(vol_up_irq);
		enable_irq_wake(vol_down_irq);
		ddata->set_wakeup = 1;
		KEY_LOGI("%s: change to wake up function(%d, %d)\n",
					__func__, vol_up_irq, vol_down_irq);
	} else if ((!ddata->wakeup_bitmask) && ddata->set_wakeup){
		disable_irq_wake(vol_up_irq);
		disable_irq_wake(vol_down_irq);
		ddata->set_wakeup = 0;
		KEY_LOGI("%s: change to non-wake up function(%d, %d)\n",
					__func__, vol_up_irq, vol_down_irq);
	}
	mutex_unlock(&ddata->attr_operation_lock);
	return count;
}

static ssize_t vol_wakeup_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct gpio_keys_drvdata *ddata = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%x\n", ddata->wakeup_bitmask);
}

static DEVICE_ATTR(vol_wakeup, S_IWUSR | S_IWGRP | S_IRUGO,
			vol_wakeup_show, vol_wakeup_store);


static inline int get_n_events_by_type(int type)
{
	BUG_ON(type != EV_SW && type != EV_KEY);

	return (type == EV_KEY) ? KEY_CNT : SW_CNT;
}

static void gpio_keys_disable_button(struct gpio_button_data *bdata)
{
	if (!bdata->disabled) {
		disable_irq(bdata->irq);
		if (bdata->timer_debounce)
			del_timer_sync(&bdata->timer);

		bdata->disabled = true;
	}
}

static void gpio_keys_enable_button(struct gpio_button_data *bdata)
{
	if (bdata->disabled) {
		enable_irq(bdata->irq);
		bdata->disabled = false;
	}
}

/**
 * gpio_keys_attr_show_helper() - fill in stringified bitmap of buttons
 * @ddata: pointer to drvdata
 * @buf: buffer where stringified bitmap is written
 * @type: button type (%EV_KEY, %EV_SW)
 * @only_disabled: does caller want only those buttons that are
 *                 currently disabled or all buttons that can be
 *                 disabled
 *
 * This function writes buttons that can be disabled to @buf. If
 * @only_disabled is true, then @buf contains only those buttons
 * that are currently disabled. Returns 0 on success or negative
 * errno on failure.
 */
static ssize_t gpio_keys_attr_show_helper(struct gpio_keys_drvdata *ddata,
					  char *buf, unsigned int type,
					  bool only_disabled)
{
	int n_events = get_n_events_by_type(type);
	unsigned long *bits;
	ssize_t ret;
	int i;

	bits = kcalloc(BITS_TO_LONGS(n_events), sizeof(*bits), GFP_KERNEL);
	if (!bits)
		return -ENOMEM;

	for (i = 0; i < ddata->pdata->nbuttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];

		if (bdata->button->type != type)
			continue;

		if (only_disabled && !bdata->disabled)
			continue;

		__set_bit(bdata->button->code, bits);
	}

	ret = bitmap_scnlistprintf(buf, PAGE_SIZE - 2, bits, n_events);
	buf[ret++] = '\n';
	buf[ret] = '\0';

	kfree(bits);

	return ret;
}

static ssize_t gpio_keys_attr_store_helper(struct gpio_keys_drvdata *ddata,
					   const char *buf, unsigned int type)
{
	int n_events = get_n_events_by_type(type);
	unsigned long *bits;
	ssize_t error;
	int i;

	bits = kcalloc(BITS_TO_LONGS(n_events), sizeof(*bits), GFP_KERNEL);
	if (!bits)
		return -ENOMEM;

	error = bitmap_parselist(buf, bits, n_events);
	if (error)
		goto out;

	
	for (i = 0; i < ddata->pdata->nbuttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];

		if (bdata->button->type != type)
			continue;

		if (test_bit(bdata->button->code, bits) &&
		    !bdata->button->can_disable) {
			error = -EINVAL;
			goto out;
		}
	}

	mutex_lock(&ddata->disable_lock);

	for (i = 0; i < ddata->pdata->nbuttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];

		if (bdata->button->type != type)
			continue;

		if (test_bit(bdata->button->code, bits))
			gpio_keys_disable_button(bdata);
		else
			gpio_keys_enable_button(bdata);
	}

	mutex_unlock(&ddata->disable_lock);

out:
	kfree(bits);
	return error;
}

#define ATTR_SHOW_FN(name, type, only_disabled)				\
static ssize_t gpio_keys_show_##name(struct device *dev,		\
				     struct device_attribute *attr,	\
				     char *buf)				\
{									\
	struct platform_device *pdev = to_platform_device(dev);		\
	struct gpio_keys_drvdata *ddata = platform_get_drvdata(pdev);	\
									\
	return gpio_keys_attr_show_helper(ddata, buf,			\
					  type, only_disabled);		\
}

ATTR_SHOW_FN(keys, EV_KEY, false);
ATTR_SHOW_FN(switches, EV_SW, false);
ATTR_SHOW_FN(disabled_keys, EV_KEY, true);
ATTR_SHOW_FN(disabled_switches, EV_SW, true);

static DEVICE_ATTR(keys, S_IRUGO, gpio_keys_show_keys, NULL);
static DEVICE_ATTR(switches, S_IRUGO, gpio_keys_show_switches, NULL);

#define ATTR_STORE_FN(name, type)					\
static ssize_t gpio_keys_store_##name(struct device *dev,		\
				      struct device_attribute *attr,	\
				      const char *buf,			\
				      size_t count)			\
{									\
	struct platform_device *pdev = to_platform_device(dev);		\
	struct gpio_keys_drvdata *ddata = platform_get_drvdata(pdev);	\
	ssize_t error;							\
									\
	error = gpio_keys_attr_store_helper(ddata, buf, type);		\
	if (error)							\
		return error;						\
									\
	return count;							\
}

ATTR_STORE_FN(disabled_keys, EV_KEY);
ATTR_STORE_FN(disabled_switches, EV_SW);

static DEVICE_ATTR(disabled_keys, S_IWUSR | S_IRUGO,
		   gpio_keys_show_disabled_keys,
		   gpio_keys_store_disabled_keys);
static DEVICE_ATTR(disabled_switches, S_IWUSR | S_IRUGO,
		   gpio_keys_show_disabled_switches,
		   gpio_keys_store_disabled_switches);

static struct attribute *gpio_keys_attrs[] = {
	&dev_attr_keys.attr,
	&dev_attr_switches.attr,
	&dev_attr_disabled_keys.attr,
	&dev_attr_disabled_switches.attr,
	&dev_attr_vol_wakeup.attr,
	NULL,
};

static struct attribute_group gpio_keys_attr_group = {
	.attrs = gpio_keys_attrs,
};

#include <linux/sched.h>
static int debug_key_bits = 0x0;
static DEFINE_SPINLOCK(debug_key_lock);
static void debug_combine_key(unsigned int code, int value)
{
	unsigned long flags;
	int bits;

	value = !!value;
	code -= 0x72;
	if (code > 2)
		return;

	spin_lock_irqsave(&debug_key_lock, flags);
	bits = debug_key_bits =
		(debug_key_bits & ~(1 << code)) | value << code;
	spin_unlock_irqrestore(&debug_key_lock, flags);

	if (bits == 0b111)
		show_state_filter(TASK_UNINTERRUPTIBLE);
}

static void gpio_keys_gpio_report_event(struct gpio_button_data *bdata)
{
	const struct gpio_keys_button *button = bdata->button;
	struct input_dev *input = bdata->input;
	unsigned int type = button->type ?: EV_KEY;
	int state = (gpio_get_value(button->gpio) ? 1 : 0) ^ button->active_low;

	if (type == EV_ABS) {
		if (state) {
			pr_info("[KEY] %s: key %x-%x, (%d) changed to %d\n",
				__func__, type, button->code, button->gpio, button->value);

			debug_combine_key(button->code, button->value);
			input_event(input, type, button->code, button->value);
		}
	} else {
		pr_info("[KEY] %s: key %x-%x, (%d) changed to %d\n",
			__func__, type, button->code, button->gpio, !!state);

		debug_combine_key(button->code, !!state);
		input_event(input, type, button->code, !!state);
	}
	input_sync(input);
}

static void gpio_keys_gpio_work_func(struct work_struct *work)
{
	struct gpio_button_data *bdata =
		container_of(work, struct gpio_button_data, work);
	const struct gpio_keys_button *button = bdata->button;
	unsigned int type = button->type ?: EV_KEY;
	unsigned long irqflags;
	bool temp_gpio_level;

	spin_lock_irqsave(&bdata->lock, irqflags);
	if (bdata->button->debounce_interval) {
		temp_gpio_level = gpio_get_value(button->gpio);

		switch (bdata->bouncing_flag) {
		case DEBOUNCE_UNSTABLE_IRQ:
		case DEBOUNCE_UNKNOWN_STATE:
			bdata->bouncing_flag = DEBOUNCE_CHECKING_STATE;
			bdata->prev_gpio_level = temp_gpio_level;
			pr_debug("[KEY] %s: key %x-%x, "
				"(%d) debounce from begining %d.\n",
				__func__, type, bdata->button->code,
				bdata->button->gpio, bdata->prev_gpio_level);
		break;
		case DEBOUNCE_CHECKING_STATE:
			if (bdata->prev_gpio_level == temp_gpio_level) {
				bdata->bouncing_flag = DEBOUNCE_DONE;
				pr_debug("[KEY] %s: key %x-%x, "
					"(%d) debounce done %d.\n",
					__func__, type, bdata->button->code,
					bdata->button->gpio, temp_gpio_level);
			} else {
				bdata->prev_gpio_level = temp_gpio_level;
				pr_debug("[KEY] %s: key %x-%x, "
					"(%d) debounce checking %d.\n",
					__func__, type, bdata->button->code,
					bdata->button->gpio, temp_gpio_level);
			}
		break;
		default:
			bdata->bouncing_flag = DEBOUNCE_UNKNOWN_STATE;
			KEY_LOGE("%s: key %x-%x, "
				"(%d) default debounce mode.\n",
				__func__, type, bdata->button->code,
				bdata->button->gpio);
		break;
		}
	}

	if ((bdata->button->debounce_interval) &&
			(bdata->bouncing_flag != DEBOUNCE_DONE)) {
		mod_timer(&bdata->timer, jiffies
			+ msecs_to_jiffies(bdata->button->debounce_interval));
	} else {
		bdata->bouncing_flag = DEBOUNCE_WAIT_IRQ;
		gpio_keys_gpio_report_event(bdata);
		if (bdata->button->wakeup)
			pm_relax(bdata->input->dev.parent);
	}

	spin_unlock_irqrestore(&bdata->lock, irqflags);
}

static void gpio_keys_gpio_timer(unsigned long _data)
{
	struct gpio_button_data *bdata = (struct gpio_button_data *)_data;

	schedule_work(&bdata->work);
}

#ifdef CONFIG_POWER_KEY_EID
void power_key_resume_handler(int irq)
{
	struct gpio_button_data *bdata = gb_data;
	unsigned long irqflags;
	unsigned int type = bdata->button->type ?: EV_KEY;
	int state = (gpio_get_value_cansleep(bdata->button->gpio) ? 1 : 0) ^ bdata->button->active_low;
	pr_info("[KEY] %s, irq=%d, gpio=%d, state=%d\n", __func__, irq, bdata->button->gpio, state);

	BUG_ON(irq != bdata->irq);

	spin_lock_irqsave(&bdata->lock, irqflags);
	if (bdata->bouncing_flag == DEBOUNCE_WAIT_IRQ) {
		bdata->bouncing_flag = DEBOUNCE_UNKNOWN_STATE;
		if (bdata->timer_debounce)
			mod_timer(&bdata->timer,
				jiffies + msecs_to_jiffies(bdata->timer_debounce));
		else
			schedule_work(&bdata->work);

		pr_debug("[KEY] %s: key %x-%x, (%d) start debounce\n",
			__func__, type, bdata->button->code,
			bdata->button->gpio);
	} else {
		bdata->bouncing_flag = DEBOUNCE_UNSTABLE_IRQ;
		KEY_LOGI("%s: key %x-%x, (%d) update debounce mode\n",
			__func__, type, bdata->button->code,
			bdata->button->gpio);
	}

	spin_unlock_irqrestore(&bdata->lock, irqflags);
}
#endif

static irqreturn_t gpio_keys_gpio_isr(int irq, void *dev_id)
{
	struct gpio_button_data *bdata = dev_id;
	unsigned long irqflags;
	unsigned int type = bdata->button->type ?: EV_KEY;
	int state = (gpio_get_value_cansleep(bdata->button->gpio) ? 1 : 0) ^ bdata->button->active_low;
	pr_info("[KEY] %s, irq=%d, gpio=%d, state=%d\n", __func__, irq, bdata->button->gpio, state);

	BUG_ON(irq != bdata->irq);

	if (bdata->button->wakeup)
		pm_stay_awake(bdata->input->dev.parent);

	spin_lock_irqsave(&bdata->lock, irqflags);
	if (bdata->bouncing_flag == DEBOUNCE_WAIT_IRQ) {
		bdata->bouncing_flag = DEBOUNCE_UNKNOWN_STATE;
		if (bdata->timer_debounce)
			mod_timer(&bdata->timer,
				jiffies + msecs_to_jiffies(bdata->timer_debounce));
		else
			schedule_work(&bdata->work);

		pr_debug("[KEY] %s: key %x-%x, (%d) start debounce\n",
			__func__, type, bdata->button->code,
			bdata->button->gpio);
	} else {
		bdata->bouncing_flag = DEBOUNCE_UNSTABLE_IRQ;
		KEY_LOGI("%s: key %x-%x, (%d) update debounce mode\n",
			__func__, type, bdata->button->code,
			bdata->button->gpio);
	}

	spin_unlock_irqrestore(&bdata->lock, irqflags);

	return IRQ_HANDLED;
}

static void gpio_keys_irq_timer(unsigned long _data)
{
	struct gpio_button_data *bdata = (struct gpio_button_data *)_data;
	struct input_dev *input = bdata->input;
	const struct gpio_keys_button *button = bdata->button;
	unsigned long flags;
	int state = 1;

	spin_lock_irqsave(&bdata->lock, flags);
	if (bdata->key_pressed) {
		pr_info("[KEY] %s: key %x-%x, (%d) changed to %d\n",
			__func__, EV_KEY, bdata->button->code, bdata->button->gpio, 0);

		if (button->gpio && gpio_is_valid(button->gpio))
			state = gpio_get_value_cansleep(button->gpio);
		if (state == 0) {
			mod_timer(&bdata->timer, jiffies +
					msecs_to_jiffies(bdata->timer_debounce));
		} else {
			input_event(input, EV_KEY, bdata->button->code, 0);
			input_sync(input);
			bdata->key_pressed = false;
		}
	}
	spin_unlock_irqrestore(&bdata->lock, flags);
}

static irqreturn_t gpio_keys_irq_isr(int irq, void *dev_id)
{
	struct gpio_button_data *bdata = dev_id;
	const struct gpio_keys_button *button = bdata->button;
	struct input_dev *input = bdata->input;
	unsigned long flags;

	pr_info("[KEY] %s, irq=%d, gpio=%d\n", __func__, irq, button->gpio);
	BUG_ON(irq != bdata->irq);

	spin_lock_irqsave(&bdata->lock, flags);

	if (!bdata->key_pressed) {
		if (bdata->button->wakeup)
			pm_wakeup_event(bdata->input->dev.parent, 0);

		pr_info("[KEY] %s: key %x-%x, (%d) changed to %d\n",
			__func__, EV_KEY, button->code, button->gpio, 1);
		input_event(input, EV_KEY, button->code, 1);
		input_sync(input);

		if (!bdata->timer_debounce) {
			pr_info("[KEY] %s: key %x-%x, (%d) changed to %d\n",
				__func__, EV_KEY, button->code, button->gpio, 0);
			input_event(input, EV_KEY, button->code, 0);
			input_sync(input);
			goto out;
		}

		bdata->key_pressed = true;
	}

	if (bdata->timer_debounce)
		mod_timer(&bdata->timer,
			jiffies + msecs_to_jiffies(bdata->timer_debounce));
out:
	spin_unlock_irqrestore(&bdata->lock, flags);
	return IRQ_HANDLED;
}

static int gpio_keys_setup_key(struct platform_device *pdev,
				struct input_dev *input,
				struct gpio_button_data *bdata,
				const struct gpio_keys_button *button)
{
	const char *desc = button->desc ? button->desc : "gpio_keys";
	struct device *dev = &pdev->dev;
	irq_handler_t isr;
	unsigned long irqflags;
	int irq, error;

	bdata->input = input;
	bdata->button = button;
	spin_lock_init(&bdata->lock);

	if (gpio_is_valid(button->gpio) && (button->irq <= 0)) {
		error = gpio_request_one(button->gpio, GPIOF_IN, desc);
		if (error < 0) {
			dev_err(dev, "Failed to request GPIO %d, error %d\n",
				button->gpio, error);
			return error;
		}

		if (button->debounce_interval) {
			error = gpio_set_debounce(button->gpio,
					button->debounce_interval * 1000);
			
			if (error < 0)
				bdata->timer_debounce =
						button->debounce_interval;
			pr_info("[KEY] %s, error=%d, debounce(%d, %d)\n",
				__func__, error, bdata->timer_debounce,
				button->debounce_interval);
		}

		irq = gpio_to_irq(button->gpio);
		if (irq < 0) {
			error = irq;
			dev_err(dev,
				"Unable to get irq number for GPIO %d, error %d\n",
				button->gpio, error);
			goto fail;
		}
		bdata->irq = irq;

		INIT_WORK(&bdata->work, gpio_keys_gpio_work_func);
		setup_timer(&bdata->timer,
			    gpio_keys_gpio_timer, (unsigned long)bdata);

		isr = gpio_keys_gpio_isr;
		irqflags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING;

	} else {
		if (button->irq <= 0) {
			dev_err(dev, "No IRQ specified\n");
			return -EINVAL;
		}
		bdata->irq = button->irq;

		if (button->type && button->type != EV_KEY) {
			dev_err(dev, "Only EV_KEY allowed for IRQ buttons.\n");
			return -EINVAL;
		}

		if (button->gpio && gpio_is_valid(button->gpio)) {
			error = gpio_request_one(button->gpio, GPIOF_IN, desc);
			if (error < 0) {
				dev_err(dev, "Failed to request GPIO %d, error %d\n",
					button->gpio, error);
				return error;
			}
		}

		bdata->timer_debounce = button->debounce_interval;
		setup_timer(&bdata->timer,
			    gpio_keys_irq_timer, (unsigned long)bdata);

		isr = gpio_keys_irq_isr;
		irqflags = 0;
		if (button->wakeup)
			irqflags |= IRQF_EARLY_RESUME;
	}

	bdata->bouncing_flag = DEBOUNCE_WAIT_IRQ;

	input_set_capability(input, button->type ?: EV_KEY, button->code);

	if (!button->can_disable)
		irqflags |= IRQF_SHARED;

	error = request_any_context_irq(bdata->irq, isr, irqflags, desc, bdata);
	if (error < 0) {
		dev_err(dev, "Unable to claim irq %d; error %d\n",
			bdata->irq, error);
		goto fail;
	}

	if(bdata->button->code == KEY_VOLUMEUP)
		vol_up_irq = bdata->irq;
	else if(bdata->button->code == KEY_VOLUMEDOWN)
		vol_down_irq = bdata->irq;
#ifdef CONFIG_POWER_KEY_EID
	else if(bdata->button->code == KEY_POWER)
		gb_data = bdata;
#endif

	KEY_LOGI("keycode = %d, gpio = %d, irq = %d",
		bdata->button->code, bdata->button->gpio, bdata->irq);

	return 0;

fail:
	if (gpio_is_valid(button->gpio))
		gpio_free(button->gpio);

	return error;
}

static void gpio_keys_report_state(struct gpio_keys_drvdata *ddata)
{
	struct input_dev *input = ddata->input;
	int i;

	for (i = 0; i < ddata->pdata->nbuttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];
		if (gpio_is_valid(bdata->button->gpio))
			gpio_keys_gpio_report_event(bdata);
	}
	input_sync(input);
}

static int gpio_keys_pinctrl_configure(struct gpio_keys_drvdata *ddata,
							bool active)
{
	struct pinctrl_state *set_state;
	int retval;

	if (active) {
		set_state =
			pinctrl_lookup_state(ddata->key_pinctrl,
						"tlmm_gpio_key_active");
		if (IS_ERR(set_state)) {
			dev_err(&ddata->input->dev,
				"cannot get ts pinctrl active state\n");
			return PTR_ERR(set_state);
		}
	} else {
		set_state =
			pinctrl_lookup_state(ddata->key_pinctrl,
						"tlmm_gpio_key_suspend");
		if (IS_ERR(set_state)) {
			dev_err(&ddata->input->dev,
				"cannot get gpiokey pinctrl sleep state\n");
			return PTR_ERR(set_state);
		}
	}
	retval = pinctrl_select_state(ddata->key_pinctrl, set_state);
	if (retval) {
		dev_err(&ddata->input->dev,
				"cannot set ts pinctrl active state\n");
		return retval;
	}

	return 0;
}

static int gpio_keys_open(struct input_dev *input)
{
	struct gpio_keys_drvdata *ddata = input_get_drvdata(input);
	const struct gpio_keys_platform_data *pdata = ddata->pdata;
	int error;

	if (pdata->enable) {
		error = pdata->enable(input->dev.parent);
		if (error)
			return error;
	}

	
	gpio_keys_report_state(ddata);

	return 0;
}

static void gpio_keys_close(struct input_dev *input)
{
	struct gpio_keys_drvdata *ddata = input_get_drvdata(input);
	const struct gpio_keys_platform_data *pdata = ddata->pdata;

	if (pdata->disable)
		pdata->disable(input->dev.parent);
}


#ifdef CONFIG_OF
static struct gpio_keys_platform_data *
gpio_keys_get_devtree_pdata(struct device *dev)
{
	struct device_node *node, *pp;
	struct gpio_keys_platform_data *pdata;
	struct gpio_keys_button *button;
	int error;
	int nbuttons;
	int i;

	node = dev->of_node;
	if (!node) {
		error = -ENODEV;
		goto err_out;
	}

	nbuttons = of_get_child_count(node);
	if (nbuttons == 0) {
		error = -ENODEV;
		goto err_out;
	}

	pdata = kzalloc(sizeof(*pdata) + nbuttons * (sizeof *button),
			GFP_KERNEL);
	if (!pdata) {
		error = -ENOMEM;
		goto err_out;
	}

	pdata->buttons = (struct gpio_keys_button *)(pdata + 1);
	pdata->nbuttons = nbuttons;

	pdata->rep = !!of_get_property(node, "autorepeat", NULL);
	pdata->name = of_get_property(node, "input-name", NULL);

	i = 0;
	for_each_child_of_node(node, pp) {
		int gpio = -1;
		unsigned int irq = 0;
		enum of_gpio_flags flags;
		bool gpio_props = false;
		bool irq_prop = false;

		if (of_find_property(pp, "gpios", NULL))
			gpio_props = true;

		irq = irq_of_parse_and_map(pp, 0);
		if (irq > 0)
			irq_prop = true;

		if (!gpio_props && !irq_prop) {
			dev_warn(dev, "Found button without gpios/irq\n");
			pdata->nbuttons--;
			continue;
		}

		if (!gpio_props)
			goto gpio_get;

		gpio = of_get_gpio_flags(pp, 0, &flags);
		if (gpio < 0) {
			error = gpio;
			if (error != -EPROBE_DEFER)
				dev_err(dev,
					"Failed to get gpio flags, error: %d\n",
					error);
			goto err_free_pdata;
		}

gpio_get:
		button = &pdata->buttons[i++];

		button->gpio = gpio;
		button->irq = irq;
		button->active_low = flags & OF_GPIO_ACTIVE_LOW;

		if (of_property_read_u32(pp, "linux,code", &button->code)) {
			dev_err(dev, "Button without keycode: 0x%x\n",
				button->gpio);
			error = -EINVAL;
			goto err_free_pdata;
		}

		button->desc = of_get_property(pp, "label", NULL);

		if (of_property_read_u32(pp, "linux,input-type", &button->type))
			button->type = EV_KEY;

		button->wakeup = !!of_get_property(pp, "gpio-key,wakeup", NULL);

		if (of_property_read_u32(pp, "debounce-interval",
					 &button->debounce_interval))
			button->debounce_interval = 5;
	}

	if (pdata->nbuttons == 0) {
		error = -EINVAL;
		goto err_free_pdata;
	}

	return pdata;

err_free_pdata:
	kfree(pdata);
err_out:
	return ERR_PTR(error);
}

static struct of_device_id gpio_keys_of_match[] = {
	{ .compatible = "gpio-keys", },
	{ },
};
MODULE_DEVICE_TABLE(of, gpio_keys_of_match);

#else

static inline struct gpio_keys_platform_data *
gpio_keys_get_devtree_pdata(struct device *dev)
{
	return ERR_PTR(-ENODEV);
}

#endif

static void gpio_remove_key(struct gpio_button_data *bdata)
{
	free_irq(bdata->irq, bdata);
	if (bdata->timer_debounce)
		del_timer_sync(&bdata->timer);
	cancel_work_sync(&bdata->work);
	if (gpio_is_valid(bdata->button->gpio))
		gpio_free(bdata->button->gpio);
}

static int gpio_keys_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct gpio_keys_platform_data *pdata = dev_get_platdata(dev);
	struct gpio_keys_drvdata *ddata;
	struct input_dev *input;
	int i = 0, error;
	int wakeup = 0;
	struct pinctrl_state *set_state;

	if (!pdata) {
		pdata = gpio_keys_get_devtree_pdata(dev);
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);
	}

	ddata = kzalloc(sizeof(struct gpio_keys_drvdata) +
			pdata->nbuttons * sizeof(struct gpio_button_data),
			GFP_KERNEL);
	input = input_allocate_device();
	if (!ddata || !input) {
		dev_err(dev, "failed to allocate state\n");
		error = -ENOMEM;
		goto fail1;
	}

	ddata->pdata = pdata;
	ddata->input = input;
	mutex_init(&ddata->disable_lock);
	mutex_init(&ddata->attr_operation_lock);

	platform_set_drvdata(pdev, ddata);
	input_set_drvdata(input, ddata);

	input->name = pdata->name ? : pdev->name;
	input->phys = "gpio-keys/input0";
	input->dev.parent = &pdev->dev;
	input->open = gpio_keys_open;
	input->close = gpio_keys_close;

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;

	
	if (pdata->rep)
		__set_bit(EV_REP, input->evbit);

	
	ddata->key_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(ddata->key_pinctrl)) {
		if (PTR_ERR(ddata->key_pinctrl) == -EPROBE_DEFER)
			return -EPROBE_DEFER;

		pr_debug("Target does not use pinctrl\n");
		ddata->key_pinctrl = NULL;
	}

	if (ddata->key_pinctrl) {
		error = gpio_keys_pinctrl_configure(ddata, true);
		if (error) {
			dev_err(dev, "cannot set ts pinctrl active state\n");
			goto fail2;
		}
	}

	for (i = 0; i < pdata->nbuttons; i++) {
		const struct gpio_keys_button *button = &pdata->buttons[i];
		struct gpio_button_data *bdata = &ddata->data[i];

		error = gpio_keys_setup_key(pdev, input, bdata, button);
		if (error)
			goto err_pinctrl;

		if (button->wakeup)
			wakeup = 1;
	}

	error = sysfs_create_group(&pdev->dev.kobj, &gpio_keys_attr_group);
	if (error) {
		dev_err(dev, "Unable to export keys/switches, error: %d\n",
			error);
		goto err_pinctrl;
	}

	
	error = sysfs_create_link(NULL, &pdev->dev.kobj, "keyboard");
	if (error) {
		KEY_LOGE("KEY_ERR: %s: subsystem_register failed\n", __func__);
		error = -ENOMEM;
		goto fail3;
	}

	ddata->wakeup_bitmask = 0;
	ddata->set_wakeup = 0;

	error = input_register_device(input);
	if (error) {
		dev_err(dev, "Unable to register input device, error: %d\n",
			error);
		goto fail4;
	}

	device_init_wakeup(&pdev->dev, wakeup);

	return 0;

 fail4:
	sysfs_remove_link(NULL, "keyboard");
 fail3:
	sysfs_remove_group(&pdev->dev.kobj, &gpio_keys_attr_group);
 err_pinctrl:
	if (ddata->key_pinctrl) {
		set_state =
		pinctrl_lookup_state(ddata->key_pinctrl,
						"tlmm_gpio_key_suspend");
		if (IS_ERR(set_state))
			dev_err(dev, "cannot get gpiokey pinctrl sleep state\n");
		else
			pinctrl_select_state(ddata->key_pinctrl, set_state);
	}

	while (--i >= 0)
		gpio_remove_key(&ddata->data[i]);
 fail2:
	platform_set_drvdata(pdev, NULL);
 fail1:
	input_free_device(input);
	kfree(ddata);
	
	if (!dev_get_platdata(&pdev->dev))
		kfree(pdata);

	return error;
}

static int gpio_keys_remove(struct platform_device *pdev)
{
	struct gpio_keys_drvdata *ddata = platform_get_drvdata(pdev);
	struct input_dev *input = ddata->input;
	int i;

	sysfs_remove_link(NULL, "keyboard");

	sysfs_remove_group(&pdev->dev.kobj, &gpio_keys_attr_group);

	device_init_wakeup(&pdev->dev, 0);

	for (i = 0; i < ddata->pdata->nbuttons; i++)
		gpio_remove_key(&ddata->data[i]);

	input_unregister_device(input);

	
	if (!dev_get_platdata(&pdev->dev))
		kfree(ddata->pdata);

	kfree(ddata);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int gpio_keys_suspend(struct device *dev)
{
	struct gpio_keys_drvdata *ddata = dev_get_drvdata(dev);
	struct input_dev *input = ddata->input;
	int i, ret;

	if (ddata->key_pinctrl) {
		ret = gpio_keys_pinctrl_configure(ddata, false);
		if (ret) {
			dev_err(dev, "failed to put the pin in suspend state\n");
			return ret;
		}
	}

	if (device_may_wakeup(dev)) {
		for (i = 0; i < ddata->pdata->nbuttons; i++) {
			struct gpio_button_data *bdata = &ddata->data[i];
			if (bdata->button->wakeup)
				enable_irq_wake(bdata->irq);
		}
	} else {
		mutex_lock(&input->mutex);
		if (input->users)
			gpio_keys_close(input);
		mutex_unlock(&input->mutex);
	}

	return 0;
}

static int gpio_keys_resume(struct device *dev)
{
	struct gpio_keys_drvdata *ddata = dev_get_drvdata(dev);
	struct input_dev *input = ddata->input;
	int error = 0;
	int i;

	if (ddata->key_pinctrl) {
		error = gpio_keys_pinctrl_configure(ddata, true);
		if (error) {
			dev_err(dev, "failed to put the pin in resume state\n");
			return error;
		}
	}

	if (device_may_wakeup(dev)) {
		for (i = 0; i < ddata->pdata->nbuttons; i++) {
			struct gpio_button_data *bdata = &ddata->data[i];
			if (bdata->button->wakeup)
				disable_irq_wake(bdata->irq);
		}
	} else {
		mutex_lock(&input->mutex);
		if (input->users)
			error = gpio_keys_open(input);
		mutex_unlock(&input->mutex);
	}

	if (error)
		return error;

	gpio_keys_report_state(ddata);
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(gpio_keys_pm_ops, gpio_keys_suspend, gpio_keys_resume);

static struct platform_driver gpio_keys_device_driver = {
	.probe		= gpio_keys_probe,
	.remove		= gpio_keys_remove,
	.driver		= {
		.name	= "gpio-keys",
		.owner	= THIS_MODULE,
		.pm	= &gpio_keys_pm_ops,
		.of_match_table = of_match_ptr(gpio_keys_of_match),
	}
};

static int __init gpio_keys_init(void)
{
	return platform_driver_register(&gpio_keys_device_driver);
}

static void __exit gpio_keys_exit(void)
{
	platform_driver_unregister(&gpio_keys_device_driver);
}

late_initcall(gpio_keys_init);
module_exit(gpio_keys_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Phil Blundell <pb@handhelds.org>");
MODULE_DESCRIPTION("Keyboard driver for GPIOs");
MODULE_ALIAS("platform:gpio-keys");
