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
 *
 */

#define pr_fmt(fmt) "%s:%d " fmt, __func__, __LINE__

#include <linux/module.h>
#include <linux/htc_flashlight.h>
#include "msm_led_flash.h"

#define FLASH_NAME "camera-led-flash"

#undef CDBG
#define CDBG(fmt, args...) pr_debug(fmt, ##args)

extern int32_t msm_led_torch_create_classdev(
				struct platform_device *pdev, void *data);

static enum flash_type flashtype;
static struct msm_led_flash_ctrl_t fctrl;

#if 1 
static struct kobject *led_status_obj; 
#endif 

static int32_t msm_led_trigger_get_subdev_id(struct msm_led_flash_ctrl_t *fctrl,
	void *arg)
{
	uint32_t *subdev_id = (uint32_t *)arg;
	if (!subdev_id) {
		pr_err("%s:%d failed\n", __func__, __LINE__);
		return -EINVAL;
	}
	*subdev_id = fctrl->pdev->id;
	CDBG("%s:%d subdev_id %d\n", __func__, __LINE__, *subdev_id);
	return 0;
}

static int32_t msm_led_trigger_config(struct msm_led_flash_ctrl_t *fctrl,
	void *data)
{
	int rc = 0;
	struct msm_camera_led_cfg_t *cfg = (struct msm_camera_led_cfg_t *)data;
	
	#if 0
	
	uint32_t i;
	uint32_t curr_l, max_curr_l;
	
	#endif
	
	CDBG("called led_state %d\n", cfg->cfgtype);

	if (!fctrl) {
		pr_err("failed\n");
		return -EINVAL;
	}

	switch (cfg->cfgtype) {
	case MSM_CAMERA_LED_OFF:
		
		#if 1
		if(htc_flash_main && htc_torch_main)
		{
			htc_flash_main(0, 0);
			htc_torch_main(0, 0);
		}
		else
			pr_err("[CAM][FL] MSM_CAMERA_LED_OFF, flashlight control is NULL\n");
		#else
		
		
		for (i = 0; i < fctrl->flash_num_sources; i++)
			if (fctrl->flash_trigger[i])
				led_trigger_event(fctrl->flash_trigger[i], 0);
		
		for (i = 0; i < fctrl->torch_num_sources; i++)
			if (fctrl->torch_trigger[i])
				led_trigger_event(fctrl->torch_trigger[i], 0);
		
		#endif
		
		break;

	case MSM_CAMERA_LED_LOW:
		
		#if 1
		if(htc_torch_main)
		{
			htc_torch_main(50, 50);
		}
		else
			pr_err("[CAM][FL] MSM_CAMERA_LED_LOW, flashlight control is NULL\n");
		#else
		
		for (i = 0; i < fctrl->torch_num_sources; i++)
			if (fctrl->torch_trigger[i]) {
				max_curr_l = fctrl->torch_max_current[i];
				if (cfg->torch_current[i] >= 0 &&
					cfg->torch_current[i] < max_curr_l) {
					curr_l = cfg->torch_current[i];
				} else {
					curr_l = fctrl->torch_op_current[i];
					pr_debug("LED torch %d clamped %d\n",
						i, curr_l);
				}
				led_trigger_event(fctrl->torch_trigger[i],
						curr_l);
			}
		
		#endif
		
		break;

	case MSM_CAMERA_LED_HIGH:
		
		#if 1
		if(htc_flash_main)
		{
			pr_info("[CAM][FL] MSM_CAMERA_LED_HIGH, called linear flashlight current value (%d, %d)\n", cfg->flash_current[0], cfg->flash_current[1]);
			htc_flash_main((int)cfg->flash_current[0], (int)cfg->flash_current[1]);
		}
		else
			pr_err("[CAM][FL] MSM_CAMERA_LED_HIGH, flashlight control is NULL\n");
		#else
		
		
		for (i = 0; i < fctrl->torch_num_sources; i++)
			if (fctrl->torch_trigger[i])
				led_trigger_event(fctrl->torch_trigger[i], 0);

		for (i = 0; i < fctrl->flash_num_sources; i++)
			if (fctrl->flash_trigger[i]) {
				max_curr_l = fctrl->flash_max_current[i];
				if (cfg->flash_current[i] >= 0 &&
					cfg->flash_current[i] < max_curr_l) {
					curr_l = cfg->flash_current[i];
				} else {
					curr_l = fctrl->flash_op_current[i];
					pr_debug("LED flash %d clamped %d\n",
						i, curr_l);
				}
				led_trigger_event(fctrl->flash_trigger[i],
					curr_l);
			}
		
		#endif
		
		break;

	case MSM_CAMERA_LED_INIT:
	case MSM_CAMERA_LED_RELEASE:
		
		#if 1
		if(htc_flash_main && htc_torch_main)
		{
			htc_flash_main(0, 0);
			htc_torch_main(0, 0);
		}
		else
			pr_err("[CAM][FL] MSM_CAMERA_LED_INIT/RELEASE, flashlight control is NULL\n");
		#else
		
		
		for (i = 0; i < fctrl->flash_num_sources; i++)
			if (fctrl->flash_trigger[i])
				led_trigger_event(fctrl->flash_trigger[i], 0);
		
		for (i = 0; i < fctrl->torch_num_sources; i++)
			if (fctrl->torch_trigger[i])
				led_trigger_event(fctrl->torch_trigger[i], 0);
		
		#endif
		
		break;

	default:
		rc = -EFAULT;
		break;
	}
	CDBG("flash_set_led_state: return %d\n", rc);
	return rc;
}

static const struct of_device_id msm_led_trigger_dt_match[] = {
	{.compatible = "qcom,camera-led-flash"},
	{}
};

MODULE_DEVICE_TABLE(of, msm_led_trigger_dt_match);

static struct platform_driver msm_led_trigger_driver = {
	.driver = {
		.name = FLASH_NAME,
		.owner = THIS_MODULE,
		.of_match_table = msm_led_trigger_dt_match,
	},
};

static int32_t msm_led_trigger_probe(struct platform_device *pdev)
{
	int32_t rc = 0, rc_1 = 0, i = 0;
	struct device_node *of_node = pdev->dev.of_node;
	struct device_node *flash_src_node = NULL;
	uint32_t count = 0;
	struct led_trigger *temp = NULL;

	CDBG("called\n");

	if (!of_node) {
		pr_err("of_node NULL\n");
		return -EINVAL;
	}

	fctrl.pdev = pdev;
	fctrl.flash_num_sources = 0;
	fctrl.torch_num_sources = 0;

	rc = of_property_read_u32(of_node, "cell-index", &pdev->id);
	if (rc < 0) {
		pr_err("failed\n");
		return -EINVAL;
	}
	CDBG("pdev id %d\n", pdev->id);

	rc = of_property_read_u32(of_node,
			"qcom,flash-type", &flashtype);
	if (rc < 0) {
		pr_err("flash-type: read failed\n");
		return -EINVAL;
	}

	
	if (of_get_property(of_node, "qcom,flash-source", &count)) {
		count /= sizeof(uint32_t);
		CDBG("qcom,flash-source count %d\n", count);
		if (count > MAX_LED_TRIGGERS) {
			pr_err("invalid count qcom,flash-source %d\n", count);
			return -EINVAL;
		}
		fctrl.flash_num_sources = count;
		for (i = 0; i < fctrl.flash_num_sources; i++) {
			flash_src_node = of_parse_phandle(of_node,
				"qcom,flash-source", i);
			if (!flash_src_node) {
				pr_err("flash_src_node %d NULL\n", i);
				continue;
			}

			rc = of_property_read_string(flash_src_node,
				"linux,default-trigger",
				&fctrl.flash_trigger_name[i]);

			rc_1 = of_property_read_string(flash_src_node,
				"qcom,default-led-trigger",
				&fctrl.flash_trigger_name[i]);
			if ((rc < 0) && (rc_1 < 0)) {
				pr_err("default-trigger: read failed\n");
				of_node_put(flash_src_node);
				continue;
			}

			CDBG("default trigger %s\n",
				fctrl.flash_trigger_name[i]);

			if (flashtype == GPIO_FLASH) {
				
				fctrl.flash_op_current[i] = LED_FULL;
			} else {
				rc = of_property_read_u32(flash_src_node,
					"qcom,current",
					&fctrl.flash_op_current[i]);
				rc_1 = of_property_read_u32(flash_src_node,
					"qcom,max-current",
					&fctrl.flash_max_current[i]);
				if ((rc < 0) || (rc_1 < 0)) {
					pr_err("current: read failed\n");
					of_node_put(flash_src_node);
					continue;
				}
			}

			of_node_put(flash_src_node);

			CDBG("max_current[%d] %d\n",
				i, fctrl.flash_op_current[i]);

			led_trigger_register_simple(fctrl.flash_trigger_name[i],
				&fctrl.flash_trigger[i]);

			if (flashtype == GPIO_FLASH)
				if (fctrl.flash_trigger[i])
					temp = fctrl.flash_trigger[i];
		}

	}
	
	if (of_get_property(of_node, "qcom,torch-source", &count)) {
		count /= sizeof(uint32_t);
		CDBG("qcom,torch-source count %d\n", count);
		if (count > MAX_LED_TRIGGERS) {
			pr_err("invalid count qcom,torch-source %d\n", count);
			return -EINVAL;
		}
		fctrl.torch_num_sources = count;

		for (i = 0; i < fctrl.torch_num_sources; i++) {
			flash_src_node = of_parse_phandle(of_node,
				"qcom,torch-source", i);
			if (!flash_src_node) {
				pr_err("torch_src_node %d NULL\n", i);
				continue;
			}

			rc = of_property_read_string(flash_src_node,
				"linux,default-trigger",
				&fctrl.torch_trigger_name[i]);

			rc_1 = of_property_read_string(flash_src_node,
				"qcom,default-led-trigger",
				&fctrl.torch_trigger_name[i]);
			if ((rc < 0) && (rc_1 < 0)) {
				pr_err("default-trigger: read failed\n");
				of_node_put(flash_src_node);
				continue;
			}

			CDBG("default trigger %s\n",
				fctrl.torch_trigger_name[i]);

			if (flashtype == GPIO_FLASH) {
				
				fctrl.torch_op_current[i] = LED_HALF;
			} else {
				rc = of_property_read_u32(flash_src_node,
					"qcom,current",
					&fctrl.torch_op_current[i]);
				rc_1 = of_property_read_u32(flash_src_node,
					"qcom,max-current",
					&fctrl.torch_max_current[i]);
				if ((rc < 0) || (rc_1 < 0)) {
					pr_err("current: read failed\n");
					of_node_put(flash_src_node);
					continue;
				}
			}

			of_node_put(flash_src_node);

			CDBG("torch max_current[%d] %d\n",
				i, fctrl.torch_op_current[i]);

			led_trigger_register_simple(fctrl.torch_trigger_name[i],
				&fctrl.torch_trigger[i]);

			if (flashtype == GPIO_FLASH)
				if (temp && !fctrl.torch_trigger[i])
					fctrl.torch_trigger[i] = temp;
		}
	}

	rc = msm_led_flash_create_v4lsubdev(pdev, &fctrl);
	if (!rc)
		msm_led_torch_create_classdev(pdev, &fctrl);

	return rc;
}

#if 1 

static uint32_t led_ril_status_value;
static uint32_t led_wimax_status_value;
static uint32_t led_hotspot_status_value;
static uint16_t led_low_temp_limit = 5;
static uint16_t led_low_cap_limit = 14;
static uint16_t led_low_cap_limit_dual = 14;

static ssize_t led_ril_status_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t length;
	length = sprintf(buf, "%d\n", led_ril_status_value);
	return length;
}

static ssize_t led_ril_status_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	uint32_t tmp = 0;

	if (buf[1] == '\n')
		tmp = buf[0] - 0x30;

	led_ril_status_value = tmp;
	pr_info("[CAM][FL] led_ril_status_value = %d\n", led_ril_status_value);
	return count;
}

static ssize_t led_wimax_status_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t length;
	length = sprintf(buf, "%d\n", led_wimax_status_value);
	return length;
}

static ssize_t led_wimax_status_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	uint32_t tmp = 0;

	if (buf[1] == '\n')
		tmp = buf[0] - 0x30;

	led_wimax_status_value = tmp;
	pr_info("[CAM][FL] led_wimax_status_value = %d\n", led_wimax_status_value);
	return count;
}

static ssize_t led_hotspot_status_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t length;
	length = sprintf(buf, "%d\n", led_hotspot_status_value);
	return length;
}

static ssize_t led_hotspot_status_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	uint32_t tmp = 0;

	tmp = buf[0] - 0x30; 

	led_hotspot_status_value = tmp;
	pr_info("[CAM][FL] led_hotspot_status_value = %d\n", led_hotspot_status_value);
	return count;
}

static ssize_t low_temp_limit_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t length;
	length = sprintf(buf, "%d\n", led_low_temp_limit);
	return length;
}

static ssize_t low_cap_limit_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t length;
	length = sprintf(buf, "%d\n", led_low_cap_limit);
	return length;
}

static ssize_t low_cap_limit_dual_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t length;
	length = sprintf(buf, "%d\n", led_low_cap_limit_dual);
	return length;
}

static DEVICE_ATTR(led_ril_status, 0644,
	led_ril_status_get,
	led_ril_status_set);

static DEVICE_ATTR(led_wimax_status, 0644,
	led_wimax_status_get,
	led_wimax_status_set);

static DEVICE_ATTR(led_hotspot_status, 0644,
	led_hotspot_status_get,
	led_hotspot_status_set);

static DEVICE_ATTR(low_temp_limit, 0444,
	low_temp_limit_get,
	NULL);

static DEVICE_ATTR(low_cap_limit, 0444,
	low_cap_limit_get,
	NULL);

static DEVICE_ATTR(low_cap_limit_dual, 0444,
	low_cap_limit_dual_get,
	NULL);

static int __init msm_led_trigger_sysfs_init(void)
{
	int ret = 0;

	pr_info("[CAM][FL] %s:%d\n", __func__, __LINE__);

	led_status_obj = kobject_create_and_add("camera_led_status", NULL);
	if (led_status_obj == NULL) {
		pr_info("[CAM][FL] msm_camera_led: subsystem_register failed\n");
		ret = -ENOMEM;
		goto error;
	}

	ret = sysfs_create_file(led_status_obj,
		&dev_attr_led_ril_status.attr);
	if (ret) {
		pr_err("[CAM][FL] msm_camera_led: sysfs_create_file dev_attr_led_ril_status failed\n");
		ret = -EFAULT;
		goto error;
	}
	ret = sysfs_create_file(led_status_obj,
		&dev_attr_led_wimax_status.attr);
	if (ret) {
		pr_err("[CAM][FL] msm_camera_led: sysfs_create_file dev_attr_led_wimax_status failed\n");
		ret = -EFAULT;
		goto error;
	}
	ret = sysfs_create_file(led_status_obj,
		&dev_attr_led_hotspot_status.attr);
	if (ret) {
		pr_err("[CAM][FL] msm_camera_led: sysfs_create_file dev_attr_led_hotspot_status failed\n");
		ret = -EFAULT;
		goto error;
	}
	ret = sysfs_create_file(led_status_obj,
		&dev_attr_low_temp_limit.attr);
	if (ret) {
		pr_err("[CAM][FL] msm_camera_led: sysfs_create_file dev_attr_low_temp_limit failed\n");
		ret = -EFAULT;
		goto error;
	}
	ret = sysfs_create_file(led_status_obj,
		&dev_attr_low_cap_limit.attr);
	if (ret) {
		pr_err("[CAM][FL] msm_camera_led: sysfs_create_file dev_attr_low_cap_limit failed\n");
		ret = -EFAULT;
		goto error;
	}
	ret = sysfs_create_file(led_status_obj,
		&dev_attr_low_cap_limit_dual.attr);
	if (ret) {
		pr_err("[CAM][FL] msm_camera_led: sysfs_create_file dev_attr_low_cap_limit_dual failed\n");
		ret = -EFAULT;
		goto error;
	}

	pr_info("[CAM][FL] %s:%d ret %d\n", __func__, __LINE__, ret);
	return ret;

error:
	kobject_del(led_status_obj);
	return ret;

}
#endif 

static int __init msm_led_trigger_add_driver(void)
{
    #if 0 

	CDBG("called\n");
	return platform_driver_probe(&msm_led_trigger_driver,
		msm_led_trigger_probe);

    #else

	int32_t rc = 0;
	pr_info("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&msm_led_trigger_driver,
		msm_led_trigger_probe);
	if (!rc) {
		rc = msm_led_trigger_sysfs_init();
	}
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	return rc;

    #endif 
}

static struct msm_flash_fn_t msm_led_trigger_func_tbl = {
	.flash_get_subdev_id = msm_led_trigger_get_subdev_id,
	.flash_led_config = msm_led_trigger_config,
};

static struct msm_led_flash_ctrl_t fctrl = {
	.func_tbl = &msm_led_trigger_func_tbl,
};

module_init(msm_led_trigger_add_driver);
MODULE_DESCRIPTION("LED TRIGGER FLASH");
MODULE_LICENSE("GPL v2");
