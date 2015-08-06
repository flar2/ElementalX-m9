/*
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009-2014 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/rfkill.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/fs.h>

extern void bt_export_bd_address(void);
extern void msm_hs_uart_gpio_config_ext(int on);
extern void bluesleep_set_bt_pwr_state(int on);

extern bool get_gpio4_state(void);
extern int lock_wlan_seci_gpio(void);

#define BT_REG_ON    114
#define BT_WAKE_HOST 108
#define BT_WAKE_DEV  112

#define BT_UART_RTSz  48
#define BT_UART_CTSz  47
#define BT_UART_RX    46
#define BT_UART_TX    45

static struct rfkill *bt_rfk;
static const char bt_name[] = "bcm4356";

static int gpio_bt_reg_on;

struct pinctrl *bt_pinctrl;
struct pinctrl_state *bt_wake_host_set_state_on;
struct pinctrl_state *bt_wake_host_set_state_off;

#define FM_ANTON_NODE  "fmanton"
static void fm_ant_node_init(void);
static void fm_ant_node_exit(void);
static int fm_ant_open(struct inode *inode, struct file *file);
static int fm_ant_release(struct inode *inode, struct file *file);
static ssize_t fm_ant_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos);
static bool is_fm_ant_on = false;
extern int fm_ant_power_fullseg(int on);

static void htc_config_bt_on(void)
{
	int rc = 0;
	printk(KERN_INFO "[BT]== R ON ==\n");

	if (gpio_bt_reg_on < 0) {
		printk(KERN_INFO "[BT]bt_reg_on:%d !!\n", gpio_bt_reg_on);
	}

	
	
	

	
	msm_hs_uart_gpio_config_ext(1);

	
	rc = pinctrl_select_state(bt_pinctrl, bt_wake_host_set_state_on);
	if (rc) printk("[BT] cannot set BT pinctrl gpio state on\n");

	
	rc = gpio_direction_output(gpio_bt_reg_on, 0);
	if (rc) printk(KERN_INFO "[BT]set REG_ON 0 fail! %d\n", rc);
	mdelay(5);
	rc = gpio_direction_output(gpio_bt_reg_on, 1);
	if (rc) printk(KERN_INFO "[BT]set REG_ON 1 fail! %d\n", rc);
	mdelay(5);

	
	bluesleep_set_bt_pwr_state(1);
}

static void htc_config_bt_off(void)
{
	int rc = 0;

	if (gpio_bt_reg_on < 0) {
		printk(KERN_INFO "[BT]bt_reg_on:%d !!\n", gpio_bt_reg_on);
	}

	
	bluesleep_set_bt_pwr_state(0);

	
	rc = gpio_direction_output(gpio_bt_reg_on, 0);
	if (rc) printk(KERN_INFO "[BT]set REG_ON 0 fail! %d\n", rc);

	
	rc = pinctrl_select_state(bt_pinctrl, bt_wake_host_set_state_off);
	if (rc) printk("[BT] cannot set BT pinctrl gpio state off\n");

	mdelay(2);

	
	msm_hs_uart_gpio_config_ext(0);

	printk(KERN_INFO "[BT]== R OFF ==\n");
}

static int bluetooth_set_power(void *data, bool blocked)
{
	if (!blocked) {
		if (get_gpio4_state() == false)
			lock_wlan_seci_gpio();
		htc_config_bt_on();
	} else
		htc_config_bt_off();

	return 0;
}

static struct rfkill_ops htc_rfkill_ops = {
	.set_block = bluetooth_set_power,
};

static int htc_rfkill_probe(struct platform_device *pdev)
{
	int rc = 0;
	bool default_state = true;  
	struct pinctrl_state *set_state;

	printk(KERN_INFO "[BT]== rfkill_probe ==\n");

	bt_export_bd_address();
	fm_ant_node_init();

	
	if (pdev->dev.of_node) {
		gpio_bt_reg_on = of_get_named_gpio(pdev->dev.of_node,
							"brcm,bt-regon-gpio", 0);
		if (gpio_bt_reg_on < 0) {
			printk("[BT]bt-regon-gpio not provided in device tree !!!");
			
		} else {
			printk("[BT]bt-regon-gpio: %d", gpio_bt_reg_on);
		}
	}

	
	bt_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(bt_pinctrl)) {
		if (PTR_ERR(bt_pinctrl) == -EPROBE_DEFER) {
			printk("[BT] bt_pinctrl EPROBE_DEFER !!");
			return -EPROBE_DEFER;
		}
	}

	if (bt_pinctrl) {
		printk("[BT] Init GPIO pins\n");
		set_state = pinctrl_lookup_state(bt_pinctrl, "bt_wake_host_gpio_on");
		if (IS_ERR(set_state)) {
			printk("[BT] cannot get BT pinctrl state bt_wake_host_gpio_on\n");
			
		} else
			bt_wake_host_set_state_on = set_state;

		set_state = pinctrl_lookup_state(bt_pinctrl, "bt_wake_host_gpio_off");
		if (IS_ERR(set_state)) {
			printk("[BT] cannot get BT pinctrl state bt_wake_host_gpio_off\n");
			
		} else
			bt_wake_host_set_state_off = set_state;

	}

	
	
	

	bluetooth_set_power(NULL, default_state);

	bt_rfk = rfkill_alloc(bt_name, &pdev->dev, RFKILL_TYPE_BLUETOOTH,
				&htc_rfkill_ops, NULL);
	if (!bt_rfk) {
		rc = -ENOMEM;
		goto err_rfkill_alloc;
	}

	rfkill_set_states(bt_rfk, default_state, false);

	

	rc = rfkill_register(bt_rfk);
	if (rc)
		goto err_rfkill_reg;

	return 0;

err_rfkill_reg:
	rfkill_destroy(bt_rfk);
err_rfkill_alloc:
	return rc;
}

static int htc_rfkill_remove(struct platform_device *dev)
{
	rfkill_unregister(bt_rfk);
	rfkill_destroy(bt_rfk);
	fm_ant_node_exit();
	return 0;
}

static const struct of_device_id htc_rfkill_match_table[] = {
	{ .compatible = "htc,htc_rfkill", },
	{},
};

static struct platform_driver htc_rfkill_driver = {
	.probe = htc_rfkill_probe,
	.remove = htc_rfkill_remove,
	.driver = {
		.name = "htc_rfkill",
		.owner = THIS_MODULE,
		.of_match_table = htc_rfkill_match_table,
	},
};

static int __init htc_rfkill_init(void)
{
	return platform_driver_register(&htc_rfkill_driver);
}

static void __exit htc_rfkill_exit(void)
{
	platform_driver_unregister(&htc_rfkill_driver);
}

static int fm_ant_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int fm_ant_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t fm_ant_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	char input;

	if (count < 1)
		return -EINVAL;

	if (copy_from_user(&input, buffer, 1)) {
		return -EFAULT;
	}

	printk(KERN_INFO "[BT]fm_ant_write:%c\n", input);

	if (input == '0') {
		is_fm_ant_on= false;
		fm_ant_power_fullseg(0);
	} else {
		is_fm_ant_on = true;
		fm_ant_power_fullseg(1);
	}

	return count;
}

static const struct file_operations fm_ant_fops = {
	.owner   = THIS_MODULE,
	.open    = fm_ant_open,
	.release = fm_ant_release,
	.write   = fm_ant_write,
};

static struct miscdevice fm_ant_misc = {
	.name  = FM_ANTON_NODE,
	.minor = MISC_DYNAMIC_MINOR,
	.fops  = &fm_ant_fops,
};

static void fm_ant_node_init(void)
{
	int ret;
	printk(KERN_INFO "[BT]fm_ant_node_init\n");
	ret = misc_register(&fm_ant_misc);
	if (ret != 0) {
		printk(KERN_INFO "[BT]Error: failed to register Misc driver,  ret = %d\n", ret);
	}
}

static void fm_ant_node_exit(void)
{
	printk(KERN_INFO "[BT]fm_ant_node_exit\n");
	misc_deregister(&fm_ant_misc);
}

module_init(htc_rfkill_init);
module_exit(htc_rfkill_exit);
MODULE_DESCRIPTION("htc rfkill");
MODULE_AUTHOR("htc_ssdbt");
MODULE_LICENSE("GPL");
