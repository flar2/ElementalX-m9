/* Copyright (c) 2012-2016, The Linux Foundation. All rights reserved.
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
#define OV8858_SENSOR_NAME "ov8858_front"
#define PLATFORM_DRIVER_NAME "msm_camera_ov8858_fornt"
#define ov8858_obj ov8858_front_##obj

DEFINE_MSM_MUTEX(ov8858_mut);

static struct msm_sensor_ctrl_t ov8858_s_ctrl;

static struct msm_sensor_power_setting ov8858_power_setting[] = {
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 1,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 1,
		.delay = 1,
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
		.delay = 3,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 0,
		.delay = 3,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 5,
	},
};

static struct v4l2_subdev_info ov8858_subdev_info[] = {
	{
		.code = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt = 1,
		.order = 0,
	},
};

static const struct i2c_device_id ov8858_i2c_id[] = {
	{OV8858_SENSOR_NAME, (kernel_ulong_t)&ov8858_s_ctrl},
	{ }
};

static int32_t msm_ov8858_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &ov8858_s_ctrl);
}

static struct i2c_driver ov8858_i2c_driver = {
	.id_table = ov8858_i2c_id,
	.probe  = msm_ov8858_i2c_probe,
	.driver = {
		.name = OV8858_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client ov8858_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id ov8858_dt_match[] = {
	{.compatible = "htc,ov8858_front", .data = &ov8858_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, ov8858_dt_match);
static struct platform_driver ov8858_platform_driver = {
	.driver = {
		.name = "htc,ov8858_front",
		.owner = THIS_MODULE,
		.of_match_table = ov8858_dt_match,
	},
};

static const char *ov8858Vendor = "OmniVision";
static const char *ov8858NAME = "ov8858_front";
static const char *ov8858Size = "8.0M";

static ssize_t sensor_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	sprintf(buf, "%s %s %s\n", ov8858Vendor, ov8858NAME, ov8858Size);
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(sensor, 0444, sensor_vendor_show, NULL);

static struct kobject *android_ov8858;

static int ov8858_sysfs_init(void)
{
	int ret ;
	pr_info("ov8858_front:kobject creat and add\n");
	android_ov8858 = kobject_create_and_add("android_camera2", NULL);
	if (android_ov8858 == NULL) {
		pr_info("ov8858_front_sysfs_init: subsystem_register " \
		"failed\n");
		ret = -ENOMEM;
		return ret ;
	}
	pr_info("ov8858_front:sysfs_create_file\n");
	ret = sysfs_create_file(android_ov8858, &dev_attr_sensor.attr);
	if (ret) {
		pr_info("ov8858_sysfs_init: sysfs_create_file " \
		"failed\n");
		kobject_del(android_ov8858);
	}

	return 0 ;
}

static int32_t ov8858_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	match = of_match_device(ov8858_dt_match, &pdev->dev);
	if (match)
	{
		rc = msm_sensor_platform_probe(pdev, match->data);
	}
	else {
		pr_err("%s:%d match is null\n", __func__, __LINE__);
		rc = -EINVAL;
	}
	return rc;
}

static void __init ov8858_init_module_async(void *unused, async_cookie_t cookie)
{
	int32_t rc = 0;
	async_synchronize_cookie(cookie);
	pr_info("%s_front:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&ov8858_platform_driver,
		ov8858_platform_probe);
	if (!rc) {
		ov8858_sysfs_init();
		return;
	}
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	i2c_add_driver(&ov8858_i2c_driver);
}

static int __init ov8858_init_module(void)
{
	async_schedule(ov8858_init_module_async, NULL);
	return 0;
}

static void __exit ov8858_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (ov8858_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&ov8858_s_ctrl);
		platform_driver_unregister(&ov8858_platform_driver);
	} else
		i2c_del_driver(&ov8858_i2c_driver);
	return;
}

/*HTC_START*/
#if 1
static int ov8858_read_fuseid(struct sensorb_cfg_data *cdata,
	struct msm_sensor_ctrl_t *s_ctrl)
{
	return 0;
}

static int ov8858_read_fuseid32(struct sensorb_cfg_data32 *cdata,
	struct msm_sensor_ctrl_t *s_ctrl)
{
	#define OV8858_LITEON_OTP_SIZE 0x10

	const short addr[3][OV8858_LITEON_OTP_SIZE] = {
    //   0      1      2      3      4      5      6      7      8      9      A      B      C      D      E      F
        {0x7010,0x7011,0x7012,0x7013,0x7014,0x7015,0x7016,0x7017,0x7018,0x7019,0x701A,0x701B,0x701C,0x701D,0x701E,0x701F}, // layer 1
        {0x7020,0x7021,0x7022,0x7023,0x7024,0x7025,0x7026,0x7027,0x7028,0x7029,0x702A,0x702B,0x702C,0x702D,0x702E,0x702F}, // layer 2
        {0x7030,0x7031,0x7032,0x7033,0x7034,0x7035,0x7036,0x7037,0x7038,0x7039,0x703A,0x703B,0x703C,0x703D,0x703E,0x703F}, // layer 3
	};
	static uint8_t otp[OV8858_LITEON_OTP_SIZE];
	static int first= true;
	uint16_t read_data = 0;

	int32_t i,j;
	int32_t rc = 0;
	const int32_t offset = 0x00;
	static int32_t valid_layer = -1;
	uint16_t addr_start = 0x7000;
	uint16_t addr_end   = 0x73ff;

	pr_info("%s called\n", __func__);
//HTC_START , move read OTP to sensor probe
	if (first) {
//HTC_END
		first = false;

		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x0100, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
		if (rc < 0)
			pr_info("%s: i2c_write b 0x0100 fail\n", __func__);

		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x5002, 0x05, MSM_CAMERA_I2C_BYTE_DATA);
		if (rc < 0)
			pr_info("%s: i2c_write b 0x5002 fail\n", __func__);

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

		// start from layer 2
		for (j=2; j>=0; j--) {
			for (i=0; i<OV8858_LITEON_OTP_SIZE; i++) {
				rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, addr[j][i]+offset, &read_data, MSM_CAMERA_I2C_BYTE_DATA);
				if (rc < 0){
					pr_err("%s: i2c_read 0x%x failed\n", __func__, addr[j][i]);
                    return rc;
				}
				otp[i] = read_data & 0xff;
				if (read_data)
					valid_layer = j;
			}
			if (valid_layer!=-1)
				break;
		}
		pr_info("%s: OTP valid layer = %d\n", __func__, valid_layer);

		ov8858_s_ctrl.driver_ic = otp[3];

		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x0100, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
		if (rc < 0)
			pr_info("%s: i2c_write b 0x0100 fail\n", __func__);

		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x5002, 0x07, MSM_CAMERA_I2C_BYTE_DATA);
		if (rc < 0)
			pr_info("%s: i2c_write b 0x5002 fail\n", __func__);
	}

	// fuseid
//HTC_START , move read OTP to sensor probe
	if (cdata != NULL) {
//HTC_END
		cdata->cfg.fuse.fuse_id_word1 = 0x00;
		cdata->cfg.fuse.fuse_id_word2 = otp[5];
		cdata->cfg.fuse.fuse_id_word3 = otp[6];
		cdata->cfg.fuse.fuse_id_word4 = otp[7];

		pr_info("%s: OTP Module vendor = 0x%x\n",                __func__, otp[0]);
		pr_info("%s: OTP LENS = 0x%x\n",                         __func__, otp[1]);
		pr_info("%s: OTP Sensor Version = 0x%x\n",               __func__, otp[2]);
		pr_info("%s: OTP Driver IC Vendor & Version = 0x%x\n",   __func__, otp[3]);
		pr_info("%s: OTP Actuator vender ID & Version = 0x%x\n", __func__, otp[4]);

		pr_info("OV8858: fuse->fuse_id : 0x%x 0x%x 0x%x 0x%x\n",
			cdata->cfg.fuse.fuse_id_word1,
			cdata->cfg.fuse.fuse_id_word2,
			cdata->cfg.fuse.fuse_id_word3,
			cdata->cfg.fuse.fuse_id_word4);

#if 0   // Fix Focus for ov8858 front cam
		cdata->af_value.VCM_VENDOR = otp[0];
		cdata->af_value.MODULE_ID_AB = cdata->cfg.fuse.fuse_id_word2;
		cdata->af_value.VCM_VENDOR_ID_VERSION = otp[4];
		cdata->af_value.AF_INF_MSB = otp[8];
		cdata->af_value.AF_INF_LSB = otp[9];
		cdata->af_value.AF_MACRO_MSB = otp[10];
		cdata->af_value.AF_MACRO_LSB = otp[11];
		pr_info("%s: OTP Infinity position code (MSByte) = 0x%x\n", __func__, cdata->af_value.AF_INF_MSB);
		pr_info("%s: OTP Infinity position code (LSByte) = 0x%x\n", __func__, cdata->af_value.AF_INF_LSB);
		pr_info("%s: OTP Macro position code (MSByte) = 0x%x\n",    __func__, cdata->af_value.AF_MACRO_MSB);
		pr_info("%s: OTP Macro position code (LSByte) = 0x%x\n",    __func__, cdata->af_value.AF_MACRO_LSB);
		strlcpy(cdata->af_value.ACT_NAME, "ti201_act", sizeof("ti201_act"));
		pr_info("%s: OTP Actuator Name = %s\n", __func__, cdata->af_value.ACT_NAME);
#endif
	}
	else {
		pr_info("%s: OTP Module vendor = 0x%x\n",                __func__, otp[0]);
		pr_info("%s: OTP LENS = 0x%x\n",                         __func__, otp[1]);
		pr_info("%s: OTP Sensor Version = 0x%x\n",               __func__, otp[2]);
		pr_info("%s: OTP Driver IC Vendor & Version = 0x%x\n",   __func__, otp[3]);
		pr_info("%s: OTP Actuator vender ID & Version = 0x%x\n", __func__, otp[4]);
	}
	return rc;
}

//HTC_START , move read OTP to sensor probe
int32_t ov8858_sensor_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	int32_t rc1 = 0;
	static int first = 0;
	rc = msm_sensor_match_id(s_ctrl);

	if(rc == 0) {
		if(first == 0) {
			pr_info("%s read_fuseid\n",__func__);
            #ifdef CONFIG_COMPAT
			rc1 = ov8858_read_fuseid32(NULL, s_ctrl);
            #else
            rc1 = ov8858_read_fuseid(NULL, s_ctrl);
            #endif
            first = 1;
		}
	}
	return rc;
}
//HTC_END
#endif

static struct msm_sensor_fn_t ov8858_sensor_func_tbl = {
	.sensor_config = msm_sensor_config,
	.sensor_config32 = msm_sensor_config32,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
    //HTC_START , move read OTP to sensor probe
	.sensor_match_id = ov8858_sensor_match_id,
	.sensor_i2c_read_fuseid = ov8858_read_fuseid,
	.sensor_i2c_read_fuseid32 = ov8858_read_fuseid32,
	//HTC_END
};
/*HTC_END*/

static struct msm_sensor_ctrl_t ov8858_s_ctrl = {
	.sensor_i2c_client = &ov8858_sensor_i2c_client,
	.power_setting_array.power_setting = ov8858_power_setting,
	.power_setting_array.size = ARRAY_SIZE(ov8858_power_setting),
    .msm_sensor_mutex = &ov8858_mut,
	.sensor_v4l2_subdev_info = ov8858_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(ov8858_subdev_info),
	.func_tbl = &ov8858_sensor_func_tbl,
};

module_init(ov8858_init_module);
module_exit(ov8858_exit_module);
MODULE_DESCRIPTION("ov8858_front");
MODULE_LICENSE("GPL v2");
