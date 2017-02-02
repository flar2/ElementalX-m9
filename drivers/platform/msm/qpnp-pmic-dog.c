/* Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/spmi.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>

#define QPNP_PON_PMIC_WD_RESET_S1_TIMER(base)	(base + 0x54)
#define QPNP_PON_PMIC_WD_RESET_S2_TIMER(base)	(base + 0x55)
#define QPNP_PON_WD_RST_S2_CTL(base)		(base + 0x56)
#define QPNP_PON_WD_RST_S2_CTL2(base)		(base + 0x57)
#define QPNP_PON_PMIC_WD_RESET_PET(base)	(base + 0x58)

#define QPNP_PON_WD_EN				BIT(7)

#define QPNP_S1_TIMER_MASK			(0x7F)
#define QPNP_S2_TIMER_MASK			(0x7F)
#define QPNP_WD_RST_S2_CTRL_MASK		(0xF)

struct qpnp_pmic_dog {
	struct spmi_device *spmi;
	u16 base;

	u32 reset_type;
	u32 bark_time;
	u32 bite_time;

	struct delayed_work wd_disable_work;
	u32 disable_work_start_time;
};

static struct qpnp_pmic_dog pmic_dog_dev;

static int qpnp_pmic_dog_masked_write(struct qpnp_pmic_dog *dev, u16 addr, u8 mask, u8 val)
{
	int rc;
	u8 reg;

	rc = spmi_ext_register_readl(dev->spmi->ctrl, dev->spmi->sid,
							addr, &reg, 1);
	if (rc) {
		dev_err(&dev->spmi->dev,
			"Unable to read from addr=%hx, rc(%d)\n",
			addr, rc);
		return rc;
	}

	reg &= ~mask;
	reg |= val & mask;
	rc = spmi_ext_register_writel(dev->spmi->ctrl, dev->spmi->sid,
							addr, &reg, 1);
	if (rc)
		dev_err(&dev->spmi->dev,
			"Unable to write to addr=%hx, rc(%d)\n", addr, rc);

	return rc;
}

int qpnp_pmic_dog_wd_config(bool enable)
{
	struct qpnp_pmic_dog *dev = &pmic_dog_dev;
	int rc = 0;

	if (!dev)
		return -EPROBE_DEFER;

	if (enable) {
		rc = qpnp_pmic_dog_masked_write(dev, QPNP_PON_PMIC_WD_RESET_S1_TIMER(dev->base), QPNP_S1_TIMER_MASK, dev->bark_time);
		if (rc)
			return rc;

		rc = qpnp_pmic_dog_masked_write(dev, QPNP_PON_PMIC_WD_RESET_S2_TIMER(dev->base), QPNP_S2_TIMER_MASK, dev->bite_time);
		if (rc)
			return rc;

		rc = qpnp_pmic_dog_masked_write(dev, QPNP_PON_WD_RST_S2_CTL(dev->base), QPNP_WD_RST_S2_CTRL_MASK, dev->reset_type);
		if (rc)
			return rc;
	}

	rc = qpnp_pmic_dog_masked_write(dev, QPNP_PON_WD_RST_S2_CTL2(dev->base), QPNP_PON_WD_EN, enable ? QPNP_PON_WD_EN : 0);
	if (rc)
		return rc;

	dev_info(&dev->spmi->dev, "set pmic dog enable = %d\n", enable);

	return rc;
}

static void wd_disable_work_func(struct work_struct *work)
{
	qpnp_pmic_dog_wd_config(false);

	return;
}

static int qpnp_pmic_dog_probe(struct spmi_device *spmi)
{
	static int is_init = 0;

	int ret;
	u32 val;
	struct qpnp_pmic_dog *dev = &pmic_dog_dev;
	struct device_node *pp = spmi->dev.of_node;
	struct resource *resource;

	if (is_init) {
		dev_err(&spmi->dev, "it is initialized before.\n");
		return 0;
	}

	dev->spmi = spmi;

	resource = spmi_get_resource(spmi, NULL, IORESOURCE_MEM, 0);
	if (!resource) {
		dev_err(&spmi->dev, "Unable to get spmi base address\n");
		return -ENXIO;
	}
	dev->base = resource->start;

	ret = of_property_read_u32(pp, "qcom,pmic-wd-reset-type", &val);
	if (ret) {
		dev_err(&spmi->dev, "wd reset type not specified\n");
		return ret;
	}
	dev->reset_type = val;

	ret = of_property_read_u32(pp, "qcom,pmic-wd-bark-time", &val);
	if (ret) {
		dev_err(&spmi->dev, "bark time not specified\n");
		return ret;
	}
	dev->bark_time = val;

	ret = of_property_read_u32(pp, "qcom,pmic-wd-bite-time", &val);
	if (ret) {
		dev_err(&spmi->dev, "bite time not specified\n");
		return ret;
	}
	dev->bite_time = val;

	ret = of_property_read_u32(pp, "qcom,pmic-wd-disable-start", &val);
	if (ret) {
		dev_err(&spmi->dev, "wd disable start not specified\n");
		return ret;
	}
	dev->disable_work_start_time = val;

	dev_info(&spmi->dev, "reset type = %d, bark time = %d, bite time = %d, disable after %d\n",
			dev->reset_type, dev->bark_time, dev->bite_time, dev->disable_work_start_time);

	ret = qpnp_pmic_dog_wd_config(true);
	if (ret) {
		dev_err(&spmi->dev, "enable pmic dog failed with ret %d\n", ret);
		return ret;
	}

	if (dev->disable_work_start_time) {
		INIT_DELAYED_WORK(&dev->wd_disable_work, wd_disable_work_func);
		schedule_delayed_work(&dev->wd_disable_work, msecs_to_jiffies(dev->disable_work_start_time * 1000));
	}

	dev_info(&spmi->dev, "driver init is done.\n");

	is_init = 1;

	return ret;
}

static int qpnp_pmic_dog_remove(struct spmi_device *spmi)
{
	struct qpnp_pmic_dog *dev = &pmic_dog_dev;

	cancel_delayed_work_sync(&dev->wd_disable_work);

	return 0;
}

static struct of_device_id spmi_match_table[] = {
	{ .compatible = "qcom,qpnp-pmic-dog", },
	{}
};

static struct spmi_driver qpnp_pmic_dog_driver = {
	.driver		= {
		.name	= "qcom,qpnp-pmic-dog",
		.of_match_table = spmi_match_table,
	},
	.probe		= qpnp_pmic_dog_probe,
	.remove		= qpnp_pmic_dog_remove,
};

static int __init qpnp_pmic_dog_init(void)
{
	return spmi_driver_register(&qpnp_pmic_dog_driver);
}
postcore_initcall(qpnp_pmic_dog_init);

static void __exit qpnp_pmic_dog_exit(void)
{
	return spmi_driver_unregister(&qpnp_pmic_dog_driver);
}
module_exit(qpnp_pmic_dog_exit);

MODULE_DESCRIPTION("QPNP PMIC DOG driver");
MODULE_LICENSE("GPL v2");
