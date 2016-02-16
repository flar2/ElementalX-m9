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
#define ov4688_SENSOR_NAME "ov4688_front"
#define PLATFORM_DRIVER_NAME "msm_camera_ov4688_fornt"
#define ov4688_obj ov4688_front_##obj

DEFINE_MSM_MUTEX(ov4688_mut);

static struct msm_sensor_ctrl_t ov4688_s_ctrl;

static struct msm_sensor_power_setting ov4688_power_setting[] = {
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 1,
		.delay = 3,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 1,
		.delay = 3,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 1,
		.delay = 3,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 0,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 5,
	},

};

static struct msm_sensor_power_setting ov4688_power_setting_XC[] = {
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 1,
		.delay = 3,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 1,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
		.delay = 3,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 1,
		.delay = 3,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 0,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 5,
	},

};

static struct v4l2_subdev_info ov4688_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static const struct i2c_device_id ov4688_i2c_id[] = {
	{ov4688_SENSOR_NAME, (kernel_ulong_t)&ov4688_s_ctrl},
	{ }
};

static int32_t msm_ov4688_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &ov4688_s_ctrl);
}

static struct i2c_driver ov4688_i2c_driver = {
	.id_table = ov4688_i2c_id,
	.probe  = msm_ov4688_i2c_probe,
	.driver = {
		.name = ov4688_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client ov4688_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id ov4688_dt_match[] = {
	{.compatible = "htc,ov4688_front", .data = &ov4688_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, ov4688_dt_match);

static struct platform_driver ov4688_platform_driver = {
	.driver = {
		.name = "htc,ov4688_front",
		.owner = THIS_MODULE,
		.of_match_table = ov4688_dt_match,
	},
};

static const char *ov4688Vendor = "OmniVision";
static const char *ov4688NAME = "ov4688_front";
static const char *ov4688Size = "4.0M";

static ssize_t sensor_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	sprintf(buf, "%s %s %s\n", ov4688Vendor, ov4688NAME, ov4688Size);
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(sensor, 0444, sensor_vendor_show, NULL);

static struct kobject *android_ov4688;

static int ov4688_sysfs_init(void)
{
	int ret ;
	pr_info("ov4688_front:kobject creat and add\n");
	android_ov4688 = kobject_create_and_add("android_camera2", NULL);
	if (android_ov4688 == NULL) {
		pr_info("ov4688_front_sysfs_init: subsystem_register " \
		"failed\n");
		ret = -ENOMEM;
		return ret ;
	}
	pr_info("ov4688_front:sysfs_create_file\n");
	ret = sysfs_create_file(android_ov4688, &dev_attr_sensor.attr);
	if (ret) {
		pr_info("ov4688_sysfs_init: sysfs_create_file " \
		"failed\n");
		kobject_del(android_ov4688);
	}

	return 0 ;
}

static int32_t ov4688_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	match = of_match_device(ov4688_dt_match, &pdev->dev);
	if (match)
	
	{
		if(msm_sensor_get_boardinfo(pdev->dev.of_node))
		{
			ov4688_s_ctrl.power_setting_array.power_setting = ov4688_power_setting_XC;
			ov4688_s_ctrl.power_setting_array.size = ARRAY_SIZE(ov4688_power_setting_XC);
		}
	
		rc = msm_sensor_platform_probe(pdev, match->data);
	
	}
	
	else {
		pr_err("%s:%d match is null\n", __func__, __LINE__);
		rc = -EINVAL;
	}
	return rc;
}

static void __init ov4688_init_module_async(void *unused, async_cookie_t cookie)
{
	int32_t rc = 0;
	async_synchronize_cookie(cookie);
	pr_info("%s_front:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&ov4688_platform_driver,
		ov4688_platform_probe);
	if (!rc) {
		ov4688_sysfs_init();
		return;
	}
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	i2c_add_driver(&ov4688_i2c_driver);
}

static int __init ov4688_init_module(void)
{
	async_schedule(ov4688_init_module_async, NULL);
	return 0;
}

static void __exit ov4688_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (ov4688_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&ov4688_s_ctrl);
		platform_driver_unregister(&ov4688_platform_driver);
	} else
		i2c_del_driver(&ov4688_i2c_driver);
	return;
}

#if 1
static int ov4688_read_fuseid(struct sensorb_cfg_data *cdata,
	struct msm_sensor_ctrl_t *s_ctrl)
{
    #define OV4688_LITEON_OTP_SIZE 0x12

    const short addr[3][OV4688_LITEON_OTP_SIZE] = {
        
        {0x126,0x127,0x128,0x129,0x12a,0x110,0x111,0x112,0x12b,0x12c,0x11e,0x11f,0x120,0x121,0x122,0x123,0x124,0x125}, 
        {0x144,0x145,0x146,0x147,0x148,0x12e,0x12f,0x130,0x149,0x14a,0x13c,0x13d,0x13e,0x13f,0x140,0x141,0x142,0x143}, 
        {0x162,0x163,0x164,0x165,0x166,0x14c,0x14d,0x14e,0x167,0x168,0x15a,0x15b,0x15c,0x15d,0x15e,0x15f,0x160,0x161}, 
    };
    static uint8_t otp[OV4688_LITEON_OTP_SIZE];
	static int first= true;
	uint16_t read_data = 0;

    int32_t i,j;
    int32_t rc = 0;
    const int32_t offset = 0x7000;
    static int32_t valid_layer=-1;
    uint16_t addr_start=0x7000;
    uint16_t addr_end=0x71ff;

	if (first) {
	    first = false;

        

        if (rc < 0)
            pr_info("%s: i2c_write recommend settings fail\n", __func__);

        rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x0100, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
        if (rc < 0)
            pr_info("%s: i2c_write b 0x0100 fail\n", __func__);

        rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d84, 0x40, MSM_CAMERA_I2C_BYTE_DATA);
        if (rc < 0)
            pr_info("%s: i2c_write b 0x3d84 fail\n", __func__);

        rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d88, addr_start, MSM_CAMERA_I2C_WORD_DATA);
        if (rc < 0)
            pr_info("%s: i2c_write w 0x3d88 fail\n", __func__);

        rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d8a, addr_end, MSM_CAMERA_I2C_WORD_DATA);
        if (rc < 0)
            pr_info("%s: i2c_write w 0x3d8a fail\n", __func__);

        rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d81, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
        if (rc < 0)
            pr_info("%s: i2c_write b 0x3d81 fail\n", __func__);

        msleep(10);

        
        for (j=2; j>=0; j--)
        {
            for (i=0; i<OV4688_LITEON_OTP_SIZE; ++i)
            {
                rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, addr[j][i]+offset, &read_data, MSM_CAMERA_I2C_BYTE_DATA);
                if (rc < 0)
                {
                    pr_err("%s: i2c_read 0x%x failed\n", __func__, addr[j][i]);
                    return rc;
                }
                else
                {
                    otp[i]= read_data;
                    if (read_data)
                        valid_layer = j;
                }
            }
            if (valid_layer != -1)
            {
                pr_info("[CAM]%s: valid_layer of module info:%d \n", __func__, valid_layer);
                break;
            }
        }

        rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x0100, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
        if (rc < 0)
            pr_info("%s: i2c_write b 0x0100 fail\n", __func__);
    }

    if(cdata != NULL)
    {
    cdata->cfg.fuse.fuse_id_word1 = 0;
    cdata->cfg.fuse.fuse_id_word2 = otp[5];
    cdata->cfg.fuse.fuse_id_word3 = otp[6];
    cdata->cfg.fuse.fuse_id_word4 = otp[7];

    pr_info("%s: OTP Module vendor = 0x%x\n",               __func__,  otp[0]);
    pr_info("%s: OTP LENS = 0x%x\n",                        __func__,  otp[1]);
    pr_info("%s: OTP Sensor Version = 0x%x\n",              __func__,  otp[2]);
    pr_info("%s: OTP Driver IC Vendor & Version = 0x%x\n",  __func__,  otp[3]);
    pr_info("%s: OTP Actuator vender ID & Version = 0x%x\n",__func__,  otp[4]);

    pr_info("%s: OTP fuse 0 = 0x%x\n", __func__,  cdata->cfg.fuse.fuse_id_word1);
    pr_info("%s: OTP fuse 1 = 0x%x\n", __func__,  cdata->cfg.fuse.fuse_id_word2);
    pr_info("%s: OTP fuse 2 = 0x%x\n", __func__,  cdata->cfg.fuse.fuse_id_word3);
    pr_info("%s: OTP fuse 3 = 0x%x\n", __func__,  cdata->cfg.fuse.fuse_id_word4);
	}
	else
	{
	    pr_info("%s: OTP Module vendor = 0x%x\n",               __func__,  otp[0]);
	    pr_info("%s: OTP LENS = 0x%x\n",                        __func__,  otp[1]);
	    pr_info("%s: OTP Sensor Version = 0x%x\n",              __func__,  otp[2]);
	    pr_info("%s: OTP Driver IC Vendor & Version = 0x%x\n",  __func__,  otp[3]);
	    pr_info("%s: OTP Actuator vender ID & Version = 0x%x\n",__func__,  otp[4]);
	}
	return rc;
}

static int ov4688_read_fuseid32(struct sensorb_cfg_data32 *cdata,
	struct msm_sensor_ctrl_t *s_ctrl)
{
    #define OV4688_LITEON_OTP_SIZE 0x12

    const short addr[3][OV4688_LITEON_OTP_SIZE] = {
        
        {0x126,0x127,0x128,0x129,0x12a,0x110,0x111,0x112,0x12b,0x12c,0x11e,0x11f,0x120,0x121,0x122,0x123,0x124,0x125}, 
        {0x144,0x145,0x146,0x147,0x148,0x12e,0x12f,0x130,0x149,0x14a,0x13c,0x13d,0x13e,0x13f,0x140,0x141,0x142,0x143}, 
        {0x162,0x163,0x164,0x165,0x166,0x14c,0x14d,0x14e,0x167,0x168,0x15a,0x15b,0x15c,0x15d,0x15e,0x15f,0x160,0x161}, 
    };
    static uint8_t otp[OV4688_LITEON_OTP_SIZE];
	static int first= true;
	uint16_t read_data = 0;

    int32_t i,j;
    int32_t rc = 0;
    const int32_t offset = 0x7000;
    static int32_t valid_layer=-1;
    uint16_t addr_start=0x7000;
    uint16_t addr_end=0x71ff;

	if (first) {
	    first = false;

        

        if (rc < 0)
            pr_info("%s: i2c_write recommend settings fail\n", __func__);

        rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x0100, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
        if (rc < 0)
            pr_info("%s: i2c_write b 0x0100 fail\n", __func__);

        rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d88, addr_start, MSM_CAMERA_I2C_WORD_DATA);
        if (rc < 0)
            pr_info("%s: i2c_write w 0x3d88 fail\n", __func__);

        rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d8a, addr_end, MSM_CAMERA_I2C_WORD_DATA);
        if (rc < 0)
            pr_info("%s: i2c_write w 0x3d8a fail\n", __func__);

        rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d85, 0x06, MSM_CAMERA_I2C_BYTE_DATA);
        if (rc < 0)
            pr_info("%s: i2c_write b 0x3d85 fail\n", __func__);

        rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d8c, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
        if (rc < 0)
            pr_info("%s: i2c_write b 0x3d8c fail\n", __func__);

        msleep(10);

        
        for (j=2; j>=0; j--)
        {
            for (i=0; i<OV4688_LITEON_OTP_SIZE; ++i)
            {
                rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, addr[j][i]+offset, &read_data, MSM_CAMERA_I2C_BYTE_DATA);
                if (rc < 0)
                {
                    pr_err("%s: i2c_read 0x%x failed\n", __func__, addr[j][i]);
                    return rc;
                }
                else
                {
                    otp[i]= read_data;
                    if (read_data)
                        valid_layer = j;
                }
            }
            if (valid_layer != -1)
            {
                pr_info("[CAM]%s: valid_layer of module info:%d \n", __func__, valid_layer);
                break;
            }
        }

        rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x0100, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
        if (rc < 0)
            pr_info("%s: i2c_write b 0x0100 fail\n", __func__);
    }

    if(cdata != NULL)
    {
    cdata->cfg.fuse.fuse_id_word1 = 0;
    cdata->cfg.fuse.fuse_id_word2 = otp[5];
    cdata->cfg.fuse.fuse_id_word3 = otp[6];
    cdata->cfg.fuse.fuse_id_word4 = otp[7];

    pr_info("%s: OTP Module vendor = 0x%x\n",               __func__,  otp[0]);
    pr_info("%s: OTP LENS = 0x%x\n",                        __func__,  otp[1]);
    pr_info("%s: OTP Sensor Version = 0x%x\n",              __func__,  otp[2]);
    pr_info("%s: OTP Driver IC Vendor & Version = 0x%x\n",  __func__,  otp[3]);
    pr_info("%s: OTP Actuator vender ID & Version = 0x%x\n",__func__,  otp[4]);

    pr_info("%s: OTP fuse 0 = 0x%x\n", __func__,  cdata->cfg.fuse.fuse_id_word1);
    pr_info("%s: OTP fuse 1 = 0x%x\n", __func__,  cdata->cfg.fuse.fuse_id_word2);
    pr_info("%s: OTP fuse 2 = 0x%x\n", __func__,  cdata->cfg.fuse.fuse_id_word3);
    pr_info("%s: OTP fuse 3 = 0x%x\n", __func__,  cdata->cfg.fuse.fuse_id_word4);
	}
	else
	{
	    pr_info("%s: OTP Module vendor = 0x%x\n",               __func__,  otp[0]);
	    pr_info("%s: OTP LENS = 0x%x\n",                        __func__,  otp[1]);
	    pr_info("%s: OTP Sensor Version = 0x%x\n",              __func__,  otp[2]);
	    pr_info("%s: OTP Driver IC Vendor & Version = 0x%x\n",  __func__,  otp[3]);
	    pr_info("%s: OTP Actuator vender ID & Version = 0x%x\n",__func__,  otp[4]);
	}
	return rc;
}


int32_t ov4688_sensor_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	int32_t rc1 = 0;
	static int first = 0;
	rc = msm_sensor_match_id(s_ctrl);
	if(rc == 0) {
		if(first == 0) {
			pr_info("%s read_fuseid\n",__func__);
            #ifdef CONFIG_COMPAT
			rc1 = ov4688_read_fuseid32(NULL, s_ctrl);
            #else
            rc1 = ov4688_read_fuseid(NULL, s_ctrl);
            #endif
            first = 1;
		}
	}
	return rc;
}
#endif

static struct msm_sensor_fn_t ov4688_sensor_func_tbl = {
	.sensor_config = msm_sensor_config,
	.sensor_config32 = msm_sensor_config32,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
    
	.sensor_match_id = ov4688_sensor_match_id,
	.sensor_i2c_read_fuseid = ov4688_read_fuseid,
	.sensor_i2c_read_fuseid32 = ov4688_read_fuseid32,
	
};

static struct msm_sensor_ctrl_t ov4688_s_ctrl = {
	.sensor_i2c_client = &ov4688_sensor_i2c_client,
	.power_setting_array.power_setting = ov4688_power_setting,
	.power_setting_array.size = ARRAY_SIZE(ov4688_power_setting),
    .msm_sensor_mutex = &ov4688_mut,
	.sensor_v4l2_subdev_info = ov4688_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(ov4688_subdev_info),
	.func_tbl = &ov4688_sensor_func_tbl,
};

module_init(ov4688_init_module);
module_exit(ov4688_exit_module);
MODULE_DESCRIPTION("ov4688_front");
MODULE_LICENSE("GPL v2");
