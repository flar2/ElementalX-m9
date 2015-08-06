/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
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
#include "msm_sensor.h"
#include <linux/async.h>
#include <linux/htc_flags.h>

#define OV2722_SENSOR_NAME "ov2722-subcam"
DEFINE_MSM_MUTEX(ov2722_mut);

#define OTP_SIZE 10

static struct msm_sensor_ctrl_t ov2722_s_ctrl;
extern int g_subcam_SOF;
int g_subcam_no_ack = 0;
static struct msm_camera_i2c_reg_array restart_reg_array[] = {
  {0x0103, 0x01},
  {0x3718, 0x10},
  {0x3702, 0x24},
  {0x373a, 0x60},
  {0x3715, 0x01},
  {0x3703, 0x2e},
  {0x3705, 0x2b},
  {0x3730, 0x30},
  {0x3704, 0x62},
  {0x3f06, 0x3a},
  {0x371c, 0x00},
  {0x371d, 0xc4},
  {0x371e, 0x01},
  {0x371f, 0x28},
  {0x3708, 0x61},
  {0x3709, 0x12},

  {0x3800, 0x00},
  {0x3801, 0x08},
  {0x3802, 0x00},
  {0x3803, 0x02},
  {0x3804, 0x07},
  {0x3805, 0x9b},
  {0x3806, 0x04},
  {0x3807, 0x45},
  {0x3808, 0x07},
  {0x3809, 0x80},
  {0x380a, 0x04},
  {0x380b, 0x38},
  {0x380c, 0x08},
  {0x380d, 0x5c},
  {0x380e, 0x04},
  {0x380f, 0x60},
  {0x3810, 0x00},
  {0x3811, 0x09},
  {0x3812, 0x00},
  {0x3813, 0x06},

  {0x3820, 0x80},
  {0x3821, 0x06},
  {0x3814, 0x11},
  {0x3815, 0x11},
  {0x3612, 0x4b},
  {0x3618, 0x04},

  {0x3a08, 0x01},
  {0x3a09, 0x50},
  {0x3a0a, 0x01},
  {0x3a0b, 0x18},
  {0x3a0d, 0x03},
  {0x3a0e, 0x03},
  {0x4520, 0x00},
  {0x4837, 0x16},
  {0x3000, 0xff},
  {0x3001, 0xff},
  {0x3002, 0xf0},
  {0x3600, 0x08},
  {0x3621, 0xc0},
  {0x3632, 0x53},
  {0x3633, 0x63},
  {0x3634, 0x24},
  {0x3f01, 0x0c},
  {0x5001, 0xc1},
  {0x3614, 0xf0},
  {0x3630, 0x2d},
  {0x370b, 0x62},
  {0x3706, 0x61},
  {0x4000, 0x02},
  {0x4002, 0xc5},
  {0x4005, 0x08},
  {0x404f, 0x84},
  {0x4051, 0x00},
  {0x5000, 0xff},
  {0x3a18, 0x00},
  {0x3a19, 0x80},
  {0x3503, 0x07},

  {0x4521, 0x00},
  {0x5183, 0xb0},
  {0x5184, 0xb0},
  {0x5185, 0xb0},
  {0x370c, 0x0c},

  {0x3035, 0x00},
  {0x3036, 0x1e},
  {0x3037, 0xa1},
  {0x303e, 0x19},
  {0x3038, 0x06},
  {0x3018, 0x04},
  {0x3000, 0x00},
  {0x3001, 0x00},
  {0x3002, 0x00},
  {0x3a0f, 0x40},
  {0x3a10, 0x38},
  {0x3a1b, 0x48},
  {0x3a1e, 0x30},
  {0x3a11, 0x90},
  {0x3a1f, 0x10},

  {0x3011, 0x22},
  
  {0x3800, 0x00},
  {0x3801, 0x08},
  {0x3802, 0x00},
  {0x3803, 0x02},
  {0x3804, 0x07},
  {0x3805, 0x9b},
  {0x3806, 0x04},
  {0x3807, 0x45},
  {0x3808, 0x07},
  {0x3809, 0x88},
  {0x380a, 0x04},
  {0x380b, 0x40},
  {0x380c, 0x08},
  {0x380d, 0x5c},
  {0x380e, 0x04},
  {0x380f, 0x70},

  {0x3810, 0x00},
  {0x3811, 0x05},
  {0x3812, 0x00},
  {0x3813, 0x02},

  {0x3820, 0x86},
  {0x3821, 0x00},
  {0x3811, 0x06},
  {0x3813, 0x03},


  {0x5000, 0xcd}, 
  {0x3503, 0x07}, 

  {0x4009, 0x40},  
  

  {0x0100, 0x01},
  {0x301a, 0xf0},
};

static struct msm_camera_i2c_reg_array stop_reg_array[] = {
{0x0100, 0x00},
};


static  struct msm_camera_i2c_reg_setting restart_settings = {
  .reg_setting = restart_reg_array,
  .size = ARRAY_SIZE(restart_reg_array),
  .addr_type = MSM_CAMERA_I2C_WORD_ADDR,
  .data_type = MSM_CAMERA_I2C_BYTE_DATA,
  .delay = 0,
};

static  struct msm_camera_i2c_reg_setting stop_settings = {
  .reg_setting = stop_reg_array,
  .size = ARRAY_SIZE(stop_reg_array),
  .addr_type = MSM_CAMERA_I2C_WORD_ADDR,
  .data_type = MSM_CAMERA_I2C_BYTE_DATA,
  .delay = 0,
};


struct msm_sensor_power_setting ov2722_sub_power_setting[] = {

	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 1,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 1,
		.delay = 6,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 2,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 0,
		.delay = 1,
	},

};

struct msm_sensor_power_setting ov2722_sub_power_setting_XC[] = {

	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 1,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 1,
		.delay = 6,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 2,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 0,
		.delay = 1,
	},

};

struct msm_sensor_power_setting ov2722_sub_power_down_setting[] = {
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 1,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val =  GPIO_OUT_LOW,
		.delay = 2,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val =0,
		.delay = 6,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 1,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
};

static void ov2722_restart(struct msm_sensor_ctrl_t *s_ctrl)
{
	long rc = 0;
	pr_info("[CAM]-sub, %s: +\n", __func__);
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_table(s_ctrl->sensor_i2c_client, &stop_settings);
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_table(s_ctrl->sensor_i2c_client, &restart_settings);
	pr_info("[CAM]-sub, %s: -\n", __func__);
}

static int ov2722_check_SOF(struct msm_sensor_ctrl_t *s_ctrl)
{
	int i = 0, j = 0;
	int got_sof = 0;
	pr_info("[CAM]-sub, %s: +\n", __func__);
	for(i = 0; i < 3 ; i++)
	{
		for(j = 0; j < 10; j++)
		{
			if(g_subcam_SOF == 0)
			{
				mdelay(30);
			}
			else
			{
				got_sof = 1;
				break;
			}
		}
		if(got_sof == 1)
		{
			break;
		}
		else
		{
			ov2722_restart(s_ctrl);
		}
	}
	if(got_sof == 1)
	{
		pr_info("[CAM]-sub, %s: got sof ok (%d,%d) -\n", __func__, i, j);
		return 0;
	}
	else
	{
		g_subcam_no_ack = 1;
		pr_info("[CAM]-sub, %s: got sof fail g_subcam_no_ack = 1 -\n", __func__);
		return -EFAULT;;
	}
}

static int ov2722_read_fuseid(struct sensorb_cfg_data *cdata,
	struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	int i;
	uint16_t read_data = 0;
	uint16_t id_addr[OTP_SIZE] = {0x3d05, 0x3d06, 0x3d07, 0x3d08, 0x3d09, 0x3d0a, 0x3d0b, 0x3d0c, 0x3d0d, 0x3d0e};
	static uint16_t id_data[OTP_SIZE] = {0};
	static int first=true;

	pr_info("%s called\n", __func__);
	if (!first) {
		cdata->cfg.fuse.fuse_id_word1 = 0;
		cdata->cfg.fuse.fuse_id_word2 = id_data[4] << 8 | id_data[5];
		cdata->cfg.fuse.fuse_id_word3 = id_data[6] << 8 | id_data[7];
		cdata->cfg.fuse.fuse_id_word4 = id_data[8] << 8 | id_data[9];

		pr_info("ov2722-subcam OTP: catched fuse->fuse_id : 0x%x 0x%x 0x%x 0x%x\n",
			cdata->cfg.fuse.fuse_id_word1,
			cdata->cfg.fuse.fuse_id_word2,
			cdata->cfg.fuse.fuse_id_word3,
			cdata->cfg.fuse.fuse_id_word4);
		return 0;
	}
	first = false;

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x0100, 1, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
		pr_err("%s: i2c_write failed\n", __func__);
		return rc;
	}
	
	for (i = 0x3d00; i <= 0x3d1f; i++) {
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, i, 0, MSM_CAMERA_I2C_BYTE_DATA);
		if (rc < 0) {
			pr_err("%s: i2c_write 0x%x failed\n", __func__, i);
			return rc;
		}
	}
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d81, 1, MSM_CAMERA_I2C_BYTE_DATA);
    msleep(10);

	for (i = 0; i < OTP_SIZE; i++) {
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, id_addr[i], &read_data, MSM_CAMERA_I2C_BYTE_DATA);
		if (rc < 0) {
			pr_err("%s: i2c_read 0x%x failed\n", __func__, i);
			return rc;
		}
		id_data[i] = read_data & 0xff;
	}

	if(cdata != NULL)
	{
		cdata->cfg.fuse.fuse_id_word1 = id_data[4];
		cdata->cfg.fuse.fuse_id_word2 = id_data[5];
		cdata->cfg.fuse.fuse_id_word3 = id_data[6];
		cdata->cfg.fuse.fuse_id_word4 = id_data[7];
		pr_info("ov2722-subcam OTP: fuse->fuse_id : 0x%x 0x%x 0x%x 0x%x\n",
			cdata->cfg.fuse.fuse_id_word1,
			cdata->cfg.fuse.fuse_id_word2,
			cdata->cfg.fuse.fuse_id_word3,
			cdata->cfg.fuse.fuse_id_word4);
	}

	return rc;
}

static int ov2722_read_fuseid32(struct sensorb_cfg_data32 *cdata,
	struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	int i;
	uint16_t read_data = 0;
	uint16_t id_addr[OTP_SIZE] = {0x3d05, 0x3d06, 0x3d07, 0x3d08, 0x3d09, 0x3d0a, 0x3d0b, 0x3d0c, 0x3d0d, 0x3d0e};
	static uint16_t id_data[OTP_SIZE] = {0};
	static int first=true;

	pr_info("%s called\n", __func__);
	if (!first) {
		cdata->cfg.fuse.fuse_id_word1 = 0;
		cdata->cfg.fuse.fuse_id_word2 = id_data[4] << 8 | id_data[5];
		cdata->cfg.fuse.fuse_id_word3 = id_data[6] << 8 | id_data[7];
		cdata->cfg.fuse.fuse_id_word4 = id_data[8] << 8 | id_data[9];

		pr_info("ov2722-subcam OTP: catched fuse->fuse_id : 0x%x 0x%x 0x%x 0x%x\n",
			cdata->cfg.fuse.fuse_id_word1,
			cdata->cfg.fuse.fuse_id_word2,
			cdata->cfg.fuse.fuse_id_word3,
			cdata->cfg.fuse.fuse_id_word4);
		return 0;
	}
	first = false;

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x0100, 1, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
		pr_err("%s: i2c_write failed\n", __func__);
		return rc;
	}
	
	for (i = 0x3d00; i <= 0x3d1f; i++) {
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, i, 0, MSM_CAMERA_I2C_BYTE_DATA);
		if (rc < 0) {
			pr_err("%s: i2c_write 0x%x failed\n", __func__, i);
			return rc;
		}
	}
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d81, 1, MSM_CAMERA_I2C_BYTE_DATA);
    msleep(10);

	for (i = 0; i < OTP_SIZE; i++) {
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, id_addr[i], &read_data, MSM_CAMERA_I2C_BYTE_DATA);
		if (rc < 0) {
			pr_err("%s: i2c_read 0x%x failed\n", __func__, i);
			return rc;
		}
		id_data[i] = read_data & 0xff;
	}

	if(cdata != NULL)
	{
		cdata->cfg.fuse.fuse_id_word1 = id_data[4];
		cdata->cfg.fuse.fuse_id_word2 = id_data[5];
		cdata->cfg.fuse.fuse_id_word3 = id_data[6];
		cdata->cfg.fuse.fuse_id_word4 = id_data[7];
		pr_info("ov2722-subcam OTP: fuse->fuse_id : 0x%x 0x%x 0x%x 0x%x\n",
			cdata->cfg.fuse.fuse_id_word1,
			cdata->cfg.fuse.fuse_id_word2,
			cdata->cfg.fuse.fuse_id_word3,
			cdata->cfg.fuse.fuse_id_word4);
	}

	return rc;

}

static struct v4l2_subdev_info ov2722_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static const struct i2c_device_id ov2722_i2c_id[] = {
	{OV2722_SENSOR_NAME, (kernel_ulong_t)&ov2722_s_ctrl},
	{ }
};

static int32_t msm_ov2722_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &ov2722_s_ctrl);
}

static struct i2c_driver ov2722_i2c_driver = {
	.id_table = ov2722_i2c_id,
	.probe  = msm_ov2722_i2c_probe,
	.driver = {
		.name = OV2722_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client ov2722_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id ov2722_dt_match[] = {
	{.compatible = "subcam,ov2722", .data = &ov2722_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, ov2722_dt_match);

static struct platform_driver ov2722_platform_driver = {
	.driver = {
		.name = "subcam,ov2722",
		.owner = THIS_MODULE,
		.of_match_table = ov2722_dt_match,
	},
};

static const char *ov2722Vendor = "Omnivision";
static const char *ov2722NAME = "ov2722";
static const char *ov2722Size = "2.1M - subcam";

static ssize_t sensor_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	sprintf(buf, "%s %s %s\n", ov2722Vendor, ov2722NAME, ov2722Size);
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(sensor, 0444, sensor_vendor_show, NULL);

static struct kobject *android_ov2722;

static int ov2722_sysfs_init(void)
{
	int ret ;
	pr_info("ov2722:kobject creat and add\n");
	android_ov2722 = kobject_create_and_add("android_camera3", NULL);
	if (android_ov2722 == NULL) {
		pr_info("ov2722_sysfs_init: subsystem_register " \
		"failed\n");
		ret = -ENOMEM;
		return ret ;
	}
	pr_info("ov2722:sysfs_create_file\n");
	ret = sysfs_create_file(android_ov2722, &dev_attr_sensor.attr);
	if (ret) {
		pr_info("ov2722_sysfs_init: sysfs_create_file " \
		"failed\n");
		kobject_del(android_ov2722);
	}

	return 0 ;
}

static int32_t ov2722_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	match = of_match_device(ov2722_dt_match, &pdev->dev);
	if (match)
	
	{
		if(msm_sensor_get_boardinfo(pdev->dev.of_node))
		{
			ov2722_s_ctrl.power_setting_array.power_setting = ov2722_sub_power_setting_XC;
			ov2722_s_ctrl.power_setting_array.size = ARRAY_SIZE(ov2722_sub_power_setting_XC);
		}
	
		rc = msm_sensor_platform_probe(pdev, match->data);
	
	}
	
	else {
		pr_err("%s:%d match is null\n", __func__, __LINE__);
		rc = -EINVAL;
	}
	pr_info("%s %s\n", __func__, dev_name(&pdev->dev));
	return rc;
}

static void __init ov2722_init_module_async(void *unused, async_cookie_t cookie)
{
	int32_t rc = 0;
    async_synchronize_cookie(cookie);
	pr_info("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&ov2722_platform_driver,
		ov2722_platform_probe);
	if (!rc) {
		ov2722_sysfs_init();
		return ;
	}
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
}

static int __init ov2722_init_module(void)
{
    async_schedule(ov2722_init_module_async, NULL);
    return 0;
}

static void __exit ov2722_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (ov2722_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&ov2722_s_ctrl);
		platform_driver_unregister(&ov2722_platform_driver);
	} else
		i2c_del_driver(&ov2722_i2c_driver);
	return;
}
int32_t ov2722_sub_sensor_power_up(struct msm_sensor_ctrl_t *s_ctrl)
{
    int32_t status;
    pr_info("%s: +\n", __func__);
    s_ctrl->power_setting_array.power_setting = ov2722_sub_power_setting;
    s_ctrl->power_setting_array.size = ARRAY_SIZE(ov2722_sub_power_setting);
    status = msm_sensor_power_up(s_ctrl);
    pr_info("%s: -\n", __func__);
    return status;
}
int32_t ov2722_sub_sensor_power_down(struct msm_sensor_ctrl_t *s_ctrl)
{
    int32_t status;
    
   
    
    pr_info("%s: +\n", __func__);
    s_ctrl->power_setting_array.power_setting = ov2722_sub_power_down_setting;
    s_ctrl->power_setting_array.size = ARRAY_SIZE(ov2722_sub_power_down_setting);
    g_subcam_no_ack = 0;
    status = msm_sensor_power_down(s_ctrl);
    pr_info("%s: -\n", __func__);
    return status;
}

int32_t ov2722_sub_sensor_config(struct msm_sensor_ctrl_t *s_ctrl,
	void __user *argp)
{
	struct sensorb_cfg_data *cdata = (struct sensorb_cfg_data *)argp;
	long rc = 0;

	switch (cdata->cfgtype) {
	case CFG_WRITE_I2C_ARRAY:
        {
            struct msm_camera_i2c_reg_setting conf_array;
            struct msm_camera_i2c_reg_array *reg_setting = NULL;
            struct msm_camera_i2c_reg_array *reg_setting_temp = NULL;
            int start_stream = 0;
            mutex_lock(s_ctrl->msm_sensor_mutex);

            if (s_ctrl->sensor_state != MSM_SENSOR_POWER_UP) {
                pr_err("[CAM]-sub, %s:%d failed: invalid state %d\n", __func__,
				__LINE__, s_ctrl->sensor_state);
                rc = -EFAULT;
                break;
            }

            if (copy_from_user(&conf_array,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_reg_setting))) {
			pr_err("[CAM]-sub, %s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
            }

            if (!conf_array.size) {
			pr_err("[CAM]-sub, %s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
            }

            reg_setting = kzalloc(conf_array.size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);

            if (!reg_setting) {
			pr_err("[CAM]-sub, %s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
            }

            if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
			conf_array.size *
			sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("[CAM]-sub, %s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
            }

            conf_array.reg_setting = reg_setting;
            reg_setting_temp = reg_setting;

            if (reg_setting_temp->reg_addr == 0x0100 && reg_setting_temp->reg_data == 0x01)
            {
                g_subcam_SOF = 0;
                start_stream = 1;
                pr_info("[CAM]-sub, %s:start_stream = 1\n", __func__);
            }
            if (reg_setting_temp->reg_addr == 0x0100 && reg_setting_temp->reg_data == 0x00)
            {
                g_subcam_no_ack = 0;
            }

			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_table(
				s_ctrl->sensor_i2c_client, &conf_array);

            kfree(reg_setting);

            if (start_stream == 1)
            {
                if (!strcmp(htc_get_bootmode(), "factory2"))
                    pr_info("[CAM]-sub, %s: MFG skip ov2722_check_SOF\n", __func__);
                else
                    rc = ov2722_check_SOF(s_ctrl);
            }
            mutex_unlock(s_ctrl->msm_sensor_mutex);
	    }
		break;
	default:
		rc = msm_sensor_config(s_ctrl, argp);
		break;
	}
	return rc;
}

int32_t ov2722_sub_sensor_config32(struct msm_sensor_ctrl_t *s_ctrl,
	void __user *argp)
{
	struct sensorb_cfg_data32 *cdata = (struct sensorb_cfg_data32 *)argp;
	long rc = 0;

	switch (cdata->cfgtype) {
	case CFG_WRITE_I2C_ARRAY:
	{
		struct msm_camera_i2c_reg_setting32 conf_array32;
		struct msm_camera_i2c_reg_setting conf_array;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;
		struct msm_camera_i2c_reg_array *reg_setting_temp = NULL;
		int start_stream = 0;
		mutex_lock(s_ctrl->msm_sensor_mutex);

		if (s_ctrl->sensor_state != MSM_SENSOR_POWER_UP) {
			pr_err("[CAM]-sub, %s:%d failed: invalid state %d\n", __func__,
				__LINE__, s_ctrl->sensor_state);
				rc = -EFAULT;
			break;
		}

		if (copy_from_user(&conf_array32,
			(void *)compat_ptr(cdata->cfg.setting),
			sizeof(struct msm_camera_i2c_reg_setting32))) {
			pr_err("[CAM]-sub, %s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		conf_array.addr_type = conf_array32.addr_type;
		conf_array.data_type = conf_array32.data_type;
		conf_array.delay = conf_array32.delay;
		conf_array.size = conf_array32.size;
		conf_array.reg_setting = compat_ptr(conf_array32.reg_setting);

		if (!conf_array.size) {
			pr_err("[CAM]-sub, %s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = kzalloc(conf_array.size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!reg_setting) {
			pr_err("[CAM]-sub, %s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting,
			(void *)(conf_array.reg_setting),
			conf_array.size *
			sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("[CAM]-sub, %s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		conf_array.reg_setting = reg_setting;
		reg_setting_temp = reg_setting;

		if (reg_setting_temp->reg_addr == 0x0100 && reg_setting_temp->reg_data == 0x01)
		{
			g_subcam_SOF = 0;
			start_stream = 1;
			pr_info("[CAM]-sub, %s:start_stream = 1\n", __func__);
		}
		if (reg_setting_temp->reg_addr == 0x0100 && reg_setting_temp->reg_data == 0x00)
		{
			g_subcam_no_ack = 0;
		}

		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_table(s_ctrl->sensor_i2c_client,
			&conf_array);
		kfree(reg_setting);

		if (start_stream == 1)
		{
			if (!strcmp(htc_get_bootmode(), "factory2"))
				pr_info("[CAM]-sub, %s: MFG skip ov2722_check_SOF\n", __func__);
			else
				rc = ov2722_check_SOF(s_ctrl);
		}
		mutex_unlock(s_ctrl->msm_sensor_mutex);
		break;
	}
	default:
		rc = msm_sensor_config32(s_ctrl, argp);
		break;
	}
	return rc;
}

int32_t ov2722_sensor_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	int32_t rc1 = 0;
	static int first = 0;
	rc = msm_sensor_match_id(s_ctrl);
	if(rc == 0) {
		if(first == 0) {
			pr_info("%s read_fuseid\n",__func__);
            #ifdef CONFIG_COMPAT
			rc1 = ov2722_read_fuseid32(NULL, s_ctrl);
            #else
            rc1 = ov2722_read_fuseid(NULL, s_ctrl);
            #endif
            first = 1;
		}
	}
	return rc;
}



static struct msm_sensor_fn_t ov2722_sensor_func_tbl = {
	
	#if 0
	.sensor_config = msm_sensor_config,
	.sensor_config32 = msm_sensor_config32,
	#else
	
	.sensor_config = ov2722_sub_sensor_config,
	.sensor_config32 = ov2722_sub_sensor_config32,
	#endif
	
	.sensor_power_up = ov2722_sub_sensor_power_up,
	.sensor_power_down = ov2722_sub_sensor_power_down,
	.sensor_match_id = msm_sensor_match_id,
	.sensor_match_id = ov2722_sensor_match_id,
	.sensor_i2c_read_fuseid = ov2722_read_fuseid,
	.sensor_i2c_read_fuseid32 = ov2722_read_fuseid32,
};

static struct msm_sensor_ctrl_t ov2722_s_ctrl = {
	.sensor_i2c_client = &ov2722_sensor_i2c_client,
	.power_setting_array.power_setting = ov2722_sub_power_setting,
	.power_setting_array.size = ARRAY_SIZE(ov2722_sub_power_setting),
	.msm_sensor_mutex = &ov2722_mut,
	.sensor_v4l2_subdev_info = ov2722_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(ov2722_subdev_info),
	.func_tbl = &ov2722_sensor_func_tbl,
};

module_init(ov2722_init_module);
module_exit(ov2722_exit_module);
MODULE_DESCRIPTION("ov2722-subcam");
MODULE_LICENSE("GPL v2");
