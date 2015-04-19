/*
 * Copyright (C) 2014 HTC, Inc.
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

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_device.h>

#include <htc_power_debug/htc_railways.h>

struct htc_railways_info_base htc_railways_base;
htc_railways_info railways_info[HTC_RAILWAY_INFO_MAX];

char *htc_railways_rails[HTC_RAILWAY_INFO_MAX] = {
	"MX",
	"CX",
	"GFX",
	"EBI",
	"APC0",
	"APC1"
};

void htc_railways_dump_info(void)
{
	void __iomem *reg;
	int i;

	if(htc_railways_base.init) {
		reg = htc_railways_base.reg_base;

		pr_debug("%s: reg_base: %p\n", __func__, reg);
		pr_info("%s: [%6s] %12s | %12s | %12s \n", __func__, "RAILS", "turbo init", "turbo min", "turbo max");
		for (i = 0; i < HTC_RAILWAY_INFO_MAX; i++) {
			if(i < HTC_RAILWAY_INFO_RPM_MAX) {
				railways_info[i].volt_init = htc_railways_read_long_register(reg, i, offsetof(htc_railways_info, volt_init));
				railways_info[i].volt_min = htc_railways_read_long_register(reg, i, offsetof(htc_railways_info, volt_min));
				railways_info[i].volt_max = htc_railways_read_long_register(reg, i, offsetof(htc_railways_info, volt_max));
			} else if(i < HTC_RAILWAY_INFO_APC_MAX) {
				railways_info[i].volt_init = apc_railways_info[i - HTC_RAILWAY_INFO_APC0].volt_init;
				railways_info[i].volt_min = apc_railways_info[i - HTC_RAILWAY_INFO_APC0].volt_min;
				railways_info[i].volt_max = apc_railways_info[i - HTC_RAILWAY_INFO_APC0].volt_max;
			}
			pr_info("%s: [%6s] %12u | %12u | %12u \n", __func__,
						htc_railways_rails[i],
						railways_info[i].volt_init,
						railways_info[i].volt_min,
						railways_info[i].volt_max);
		}
	}
	return;
}

static int htc_railways_probe(struct platform_device *pdev)
{
	struct htc_railways_platform_data *pdata = NULL;
	struct resource *res = NULL;
	struct device_node *node = NULL;
	struct dentry *dent = NULL;

	if(!pdev)
		return -EINVAL;

	pdata = kzalloc(sizeof(struct htc_railways_platform_data), GFP_KERNEL);

	if(!pdata)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(!res)
		return -EINVAL;

	pr_debug("%s: try to initial htc_railway.\n", __func__);
	pr_debug("%s: [DEBUG] res: %p\n", __func__, res);

	pdata->phys_addr_base = res->start;
	pdata->phys_size = resource_size(res);

	node = pdev->dev.of_node;

	if(!node) {
		pr_err("%s: cannot find htc,railways-info\n", __func__);
		kfree(pdata);
		return -EINVAL;
	}

	pr_debug("%s: try to create debug fs.\n", __func__);
	dent = debugfs_create_file("htc_railways", S_IRUGO, NULL, pdata, &htc_railways_fops);
	if(!dent) {
		pr_err("%s: ERROR debugfs_create_file failed.\n", __func__);
		kfree(pdata);
		return -ENOMEM;
	}

	if(!htc_railways_base.init) {
		htc_railways_base.reg_base = ioremap_nocache(pdata->phys_addr_base, pdata->phys_size);
		htc_railways_base.init = 1;

		pr_debug("%s: phys: %0LX, size: %u\n", __func__, pdata->phys_addr_base, pdata->phys_size);
	}

	platform_set_drvdata(pdev, dent);
    pr_debug("%s: htc_railways driver init done.\n", __func__);

	htc_railways_dump_info();
	return 0;
}

static int htc_railways_remove(struct platform_device *pdev)
{
	struct dentry *dent;

	dent = platform_get_drvdata(pdev);
	debugfs_remove(dent);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct of_device_id htc_railways_table[] =
{
	{ .compatible = "htc,railways-info" },
	{},
};

static struct platform_driver htc_railways_driver =
{
	.driver = {
		.name = "htc_railways_info",
		.of_match_table = htc_railways_table,
		.owner = THIS_MODULE,
	},
	.probe 	= htc_railways_probe,
	.remove	= htc_railways_remove,
};

static int __init htc_railways_init(void)
{
	int ret;

	ret = platform_driver_register(&htc_railways_driver);

	return ret;
}
EXPORT_SYMBOL(htc_railways_init);

static void __exit htc_railways_exit(void)
{
	platform_driver_unregister(&htc_railways_driver);
}

module_init(htc_railways_init);
module_exit(htc_railways_exit);

MODULE_AUTHOR("Keric Kuo <keric_kuo@htc.com>");
MODULE_DESCRIPTION("HTC Railways debug driver");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:htc_railways_info");
