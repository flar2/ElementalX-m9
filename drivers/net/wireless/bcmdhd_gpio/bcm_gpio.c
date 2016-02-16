/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/export.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/pm.h>
#include <linux/pm_wakeup.h>
#include <linux/sched.h>
#include <linux/pm_qos.h>
#include <linux/esoc_client.h>
#include <linux/pinctrl/consumer.h>
#include <soc/qcom/subsystem_restart.h>
#include <soc/qcom/subsystem_notif.h>
#include <soc/qcom/ramdump.h>
#include <linux/msm-bus.h>
#include <linux/msm-bus-board.h>
#ifdef CONFIG_PCI_MSM
#include <linux/msm_pcie.h>
#else
#include <mach/msm_pcie.h>
#endif

#include "bcm_dev_reg.h"

#define WLAN_VREG_NAME		"vdd-wlan"
#define WLAN_VREG_IO_NAME	"vdd-wlan-io"
#define WLAN_VREG_XTAL_NAME	"vdd-wlan-xtal"
#define WLAN_SWREG_NAME		"wlan-soc-swreg"
#define WLAN_EN_GPIO_NAME	"wlan-en-gpio"
#define WLAN_IRQ_GPIO_NAME	"wlan-irq-gpio"
#define WLAN_BOOT_GPIO_NAME "wlan-boot-gpio"

#define SOC_SWREG_VOLT_MAX	1200000
#define SOC_SWREG_VOLT_MIN	1200000
#define WLAN_VREG_IO_MAX	1800000
#define WLAN_VREG_IO_MIN	1800000
#define WLAN_VREG_XTAL_MAX	1800000
#define WLAN_VREG_XTAL_MIN	1800000

#define WLAN_ENABLE_DELAY	10000
#define BCM_PINCTRL_STATE_ACTIVE "default"

#define BCM_ENUMERATE 1
#ifdef BCM_ENUMERATE
#undef BCM_ENUMERATE
#endif

struct wlan_vreg_info {
	struct regulator *wlan_reg;
	struct regulator *soc_swreg;
	struct regulator *wlan_reg_io;
	struct regulator *wlan_reg_xtal;
	bool state;
};

struct wlan_vreg_info vreg_info;
struct wlan_gpio_info gpio_wl_en;
struct wlan_gpio_info gpio_irq;
struct wlan_gpio_info gpio_seci_in;
struct mutex seci_lock;

struct platform_device *gdev;

static int bcm_wlan_gpio_init(struct wlan_gpio_info *info)
{
	int ret = 0;

	ret = gpio_request(info->num, info->name);

	if (ret) {
		pr_err("can't get gpio %s ret %d\n", info->name, ret);
		goto err_gpio_req;
	}

    if ( !strcmp(info->name, WLAN_EN_GPIO_NAME) ) {
		pr_err("%s: gpio %d -> direction_output\n", __func__, info->num);
		ret = gpio_direction_output(info->num, info->init);

		if (ret) {
			pr_err("can't set gpio direction %s ret %d\n", info->name, ret);
			goto err_gpio_dir;
		}
    } else if (!strcmp(info->name, WLAN_IRQ_GPIO_NAME)) {
        pr_err("%s: gpio %d -> direction_input\n", __func__, info->num);
        ret = gpio_direction_input(info->num);

        if (ret) {
            pr_err("can't set gpio direction %s ret %d\n", info->name, ret);
            goto err_gpio_dir;
        }
    } else if (!strcmp(info->name, WLAN_BOOT_GPIO_NAME)) {
        pr_err("%s: free gpio %d \n", __func__, info->num);
        info->prop = false;
        gpio_free(info->num);
    }

	info->state = info->init;

	return ret;

err_gpio_dir:
	gpio_free(info->num);

err_gpio_req:

	return ret;
}

static int bcm_pinctrl_init(struct platform_device *pdev)
{
	int ret;
    struct pinctrl *pinctrl;
    struct pinctrl_state *set_state;

    pinctrl = devm_pinctrl_get(&pdev->dev);
    if (IS_ERR_OR_NULL(pinctrl)) {
        pr_err("%s: Failed to get pinctrl!\n", __func__);
        return PTR_ERR(pinctrl);
    }

    set_state = pinctrl_lookup_state(pinctrl, "default");
    if (IS_ERR_OR_NULL(set_state)) {
        pr_err("%s: Can not get active pin state!\n", __func__);
        return PTR_ERR(set_state);
    }

    ret = pinctrl_select_state(pinctrl, set_state);

	return ret;
}

static int wlan_seci_gpio_pin_init(struct platform_device *pdev, char* type)
{
    int ret = 0;
    struct pinctrl *pinctrl;
    struct pinctrl_state *set_state;

    pinctrl = devm_pinctrl_get(&pdev->dev);
    if (IS_ERR_OR_NULL(pinctrl)) {
        pr_err("%s: Failed to get pinctrl!\n", __func__);
        return PTR_ERR(pinctrl);
    }

    set_state = pinctrl_lookup_state(pinctrl, type);
    if (IS_ERR_OR_NULL(set_state)) {
        pr_err("%s: Can not get active pin state!\n", __func__);
        return PTR_ERR(set_state);
    }

    ret = pinctrl_select_state(pinctrl, set_state);

    return ret;
}

int wlan_seci_gpio_init(struct wlan_gpio_info *info, int state)
{
    int ret = 0;
	int time = 0;

    pr_err(" %s: info->num = %d, state =%d \n", __func__, info->num, state);

	mutex_lock(&seci_lock);

	if (state == 1 && info->prop == false) {

        ret = gpio_request(info->num, info->name);
        if (ret) {
            pr_err("%s: Can't get GPIO %s, ret = %d\n",
                __func__, WLAN_BOOT_GPIO_NAME, ret);
            goto out;
        }

        wlan_seci_gpio_pin_init(gdev, "seci_in_low");

        ret = gpio_direction_output(info->num, 0);
        if (ret) {
            pr_err("%s: Can't set GPIO %s direction, ret = %d\n",
                __func__, WLAN_BOOT_GPIO_NAME, ret);
            gpio_free(info->num);
            goto out;
        }

		while (gpio_get_value(info->num) !=  0 && (time < 40)) {
			usleep(10000);
			time++;
			pr_err("%s: waiting time: %d \n", __func__, time);
		}
		if (time >= 40) {
			pr_err("%s: bcm_gpio_set failed\n", __func__);
			gpio_free(info->num);
			mutex_unlock(&seci_lock);
			return -1;
		}
        info->prop = true;

    } else if (state == 0) {
        wlan_seci_gpio_pin_init(gdev, "seci_in_default");
        info->prop = false;
        gpio_free(info->num);
    }
	pr_err(" %s: info->prop= %d \n",__func__, info->prop);

out:
	mutex_unlock(&seci_lock);
    return ret;
}

static int bcm_wlan_get_resources(struct platform_device *pdev, struct wlan_gpio_info *gpio_info)
{
	int ret = 0;

	if (!of_find_property((&pdev->dev)->of_node, gpio_info->name, NULL)) {
		gpio_info->prop = false;
		goto end;
	}

	gpio_info->prop = true;
	ret = of_get_named_gpio((&pdev->dev)->of_node,
				gpio_info->name, 0);

	if (ret >= 0) {
		gpio_info->num = ret;
		ret = 0;
	} else {
		if (ret == -EPROBE_DEFER)
			pr_debug("get WLAN_EN GPIO probe defer\n");
		else
			pr_err("can't get gpio %s ret %d",
			       gpio_info->name, ret);
		goto err_get_gpio;
	}

	ret = bcm_wlan_gpio_init(gpio_info);

	if (ret) {
		pr_err("gpio init failed\n");
		goto err_gpio_init;
	}

end:
	return ret;

err_gpio_init:
err_get_gpio:
	vreg_info.state = VREG_OFF;

	return ret;
}

int bcm_gpio_set(struct wlan_gpio_info *info, bool state)
{
    int time = 0;
	if (!info->prop) {
		printk("of_find_property() failed for %s", info->name);
		return -1;
	}

	if (info->state == state) {
		printk("Already %s gpio is %s\n",
			 info->name, state ? "high" : "low");
		return -1;
	}

	gpio_set_value(info->num, state);

    while (gpio_get_value(info->num) != state && (time < 40)) {
        usleep(50000);
        time++;
    }
	info->state = state;
    if (time >= 40) {
        printk("bcm_gpio_set failed : %s\n", state ? "enabled" : "disabled");
    }
	printk("====== %s gpio is now %s, retry %d time\n", info->name, info->state ? "enabled" : "disabled", time);

	return 0;
}

static void bcm_wlan_release_resources(void)
{
	gpio_free(gpio_wl_en.num);
	gpio_free(gpio_irq.num);
	gpio_wl_en.state = WLAN_EN_LOW;
	gpio_wl_en.prop = false;
	vreg_info.state = VREG_OFF;
}

static int bcm_gpio_probe(struct platform_device *pdev)
{
	int ret = 0;
#ifdef BCM_ENUMERATE
	u32 rc_num;
	struct device *dev = &pdev->dev;
#endif

	printk("%s@%d \n", __func__, __LINE__);
    gdev = pdev;

	
	gpio_wl_en.name = WLAN_EN_GPIO_NAME;
	gpio_wl_en.num = 0;
	gpio_wl_en.state = WLAN_EN_LOW;
	gpio_wl_en.init = WLAN_EN_LOW;
	gpio_wl_en.prop = false;

	
	gpio_irq.name = WLAN_IRQ_GPIO_NAME;
	gpio_irq.num = 0;
	gpio_irq.state = WLAN_EN_LOW;
	gpio_irq.init = WLAN_EN_LOW;
	gpio_irq.prop = false;

    
    gpio_seci_in.name = WLAN_BOOT_GPIO_NAME;
    gpio_seci_in.num = 0;
    gpio_seci_in.state = WLAN_EN_LOW;
    gpio_seci_in.init = WLAN_EN_LOW;
    gpio_seci_in.prop = false;

	vreg_info.wlan_reg = NULL;
	vreg_info.state = VREG_OFF;

	mutex_init(&seci_lock);

	ret = bcm_wlan_get_resources(pdev, &gpio_wl_en);
	if (ret)
		goto err_get_wlan_res;

	ret = bcm_wlan_get_resources(pdev, &gpio_irq);
	if (ret)
		goto err_get_wlan_res;

    ret = bcm_wlan_get_resources(pdev, &gpio_seci_in);
    if (ret)
        goto err_get_wlan_res;

    ret = bcm_pinctrl_init(pdev);
    if (ret) {
        pr_err("%s: pinctrl init failed!\n", __func__);
        goto err_get_wlan_res;
    }

#ifdef BCM_ENUMERATE
	bcm_gpio_set(&gpio_wl_en, WLAN_EN_HIGH);
	usleep(WLAN_ENABLE_DELAY);
#endif

#ifdef BCM_ENUMERATE
	
	ret = of_property_read_u32(dev->of_node, "htc,wlan-rc-num", &rc_num);
	if (ret) {
		pr_err("%s: Failed to find PCIe RC number!\n", __func__);
		goto err_get_rc;
	}

	ret = msm_pcie_enumerate(rc_num);
	if (ret) {
		pr_err("%s: Failed to enable PCIe RC%x!\n", __func__, rc_num);
		goto err_pcie_enumerate;
	}
#endif

	return 0;

err_get_wlan_res:
#ifdef BCM_ENUMERATE
err_get_rc:
err_pcie_enumerate:
#endif
	pr_err("%s@%d err! ret %d\n", __func__, __LINE__, ret);
	return ret;
}

static int bcm_gpio_remove(struct platform_device *pdev)
{
	bcm_gpio_set(&gpio_wl_en, WLAN_EN_LOW);
	bcm_wlan_release_resources();

	return 0;
}

static const struct of_device_id htc_wlan_dt_match[] = {
	{.compatible = "wlan_bcm_gpio"},
	{}
};

MODULE_DEVICE_TABLE(of, htc_wlan_dt_match);

static struct platform_driver bcm_gpio_driver = {
	.probe  = bcm_gpio_probe,
	.remove = bcm_gpio_remove,
	.driver = {
		.name = "bcm_gpio",
		.owner = THIS_MODULE,
		.of_match_table = htc_wlan_dt_match,
	},
};

static int __init bcm_gpio_init(void)
{
	
	hima_wifi_init();

	return platform_driver_register(&bcm_gpio_driver);
}

static void __exit bcm_gpio_exit(void)
{
	platform_driver_unregister(&bcm_gpio_driver);
}

module_init(bcm_gpio_init);
module_exit(bcm_gpio_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION(DEVICE "Init BCM GPIO");
