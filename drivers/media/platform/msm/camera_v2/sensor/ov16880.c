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
#include <linux/vmalloc.h>
#include "msm_cci.h"
/*HTC_START*/
#ifdef CONFIG_OIS_CALIBRATION
#include "lc898123F40_htc.h"
#endif
/*HTC_END*/
#define ov16880_SENSOR_NAME "ov16880"
#define PLATFORM_DRIVER_NAME "msm_camera_ov16880"
#define ov16880_obj ov16880_##obj
#define OV16880_PDAF_SIZE 1068
#define OV16880_SECTOR_UNIT 64*4
#define OV16880_PDAF_SECTOR 5

DEFINE_MSM_MUTEX(ov16880_mut);

static struct msm_sensor_ctrl_t ov16880_s_ctrl;

static struct msm_sensor_power_setting ov16880_power_setting[] = {
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_V_CUSTOM1,
		.config_val = 1,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VAF,
		.config_val = 1,
		.delay = 1,
	},
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

static struct v4l2_subdev_info ov16880_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static const struct i2c_device_id ov16880_i2c_id[] = {
	{ov16880_SENSOR_NAME, (kernel_ulong_t)&ov16880_s_ctrl},
	{ }
};

static int32_t msm_ov16880_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &ov16880_s_ctrl);
}

static struct i2c_driver ov16880_i2c_driver = {
	.id_table = ov16880_i2c_id,
	.probe  = msm_ov16880_i2c_probe,
	.driver = {
		.name = ov16880_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client ov16880_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id ov16880_dt_match[] = {
	{.compatible = "htc,ov16880", .data = &ov16880_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, ov16880_dt_match);

static struct platform_driver ov16880_platform_driver = {
	.driver = {
		.name = "htc,ov16880",
		.owner = THIS_MODULE,
		.of_match_table = ov16880_dt_match,
	},
};

static const char *ov16880Vendor = "OmniVision";
static const char *ov16880NAME = "ov16880";
static const char *ov16880Size = "16.0M";

static ssize_t sensor_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	sprintf(buf, "%s %s %s\n", ov16880Vendor, ov16880NAME, ov16880Size);
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(sensor, 0444, sensor_vendor_show, NULL);

static struct kobject *android_ov16880;

static int ov16880_sysfs_init(void)
{
	int ret ;
	pr_info("ov16880:kobject creat and add\n");
	android_ov16880 = kobject_create_and_add("android_camera", NULL);
	if (android_ov16880 == NULL) {
		pr_info("ov16880_sysfs_init: subsystem_register " \
		"failed\n");
		ret = -ENOMEM;
		return ret ;
	}
	pr_info("ov16880:sysfs_create_file\n");
	ret = sysfs_create_file(android_ov16880, &dev_attr_sensor.attr);
	if (ret) {
		pr_info("ov16880_sysfs_init: sysfs_create_file " \
		"failed\n");
		kobject_del(android_ov16880);
	}

	return 0 ;
}

static int32_t ov16880_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	match = of_match_device(ov16880_dt_match, &pdev->dev);
	if (match)
		rc = msm_sensor_platform_probe(pdev, match->data);
	else {
		pr_err("%s:%d match is null\n", __func__, __LINE__);
		rc = -EINVAL;
	}
	return rc;
}

static void __init ov16880_init_module_async(void *unused, async_cookie_t cookie)
{
	int32_t rc = 0;
	async_synchronize_cookie(cookie);
	pr_info("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&ov16880_platform_driver,
		ov16880_platform_probe);
	if (!rc) {
		ov16880_sysfs_init();
		return;
	}
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	i2c_add_driver(&ov16880_i2c_driver);
}

static int __init ov16880_init_module(void)
{
	async_schedule(ov16880_init_module_async, NULL);
	return 0;
}

static void __exit ov16880_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (ov16880_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&ov16880_s_ctrl);
		platform_driver_unregister(&ov16880_platform_driver);
	} else
		i2c_del_driver(&ov16880_i2c_driver);
	return;
}

/*HTC_START*/
#if 1
#define EEPROM_COMPONENT_I2C_ADDR_WRITE 0x7D

static int ov16880_read_fuseid(struct sensorb_cfg_data *cdata,
	struct msm_sensor_ctrl_t *s_ctrl)
{
	return 0;
}

static int ov16880_read_fuseid32(struct sensorb_cfg_data32 *cdata,
	struct msm_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;
	int32_t i = 0;
	//int32_t retry_cnt = 0;
	static int first = true;
	uint8_t *otp_sector = NULL;
	static uint8_t otp[24] = {0};
	uint16_t address = 0;
	uint16_t cci_client_sid_backup;
	uint16_t read_data = 0;

	if (first)
	{
		first = false;
		/* Read alpha value for BLC */
		msleep(10);
		//rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x5000, 0x9a, MSM_CAMERA_I2C_BYTE_DATA);
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, 0x798F, &read_data, MSM_CAMERA_I2C_BYTE_DATA);
		otp[19] = read_data;
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, 0x7991, &read_data, MSM_CAMERA_I2C_BYTE_DATA);
		otp[20] = read_data - 0x12;
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, 0x7993, &read_data, MSM_CAMERA_I2C_BYTE_DATA);
		otp[21] = read_data - 0xe;
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, 0x7995, &read_data, MSM_CAMERA_I2C_BYTE_DATA);
		otp[22] = read_data;
		//rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x5000, 0x9b, MSM_CAMERA_I2C_BYTE_DATA);

		otp_sector = vzalloc(OV16880_SECTOR_UNIT );
		if(otp_sector == NULL)
		{
			pr_err("[OTP]%s: allocate buffer failed!!!\n", __func__);
			return -ENOMEM;
		}
		else
		{
			/* Bcakup the I2C slave address */
			cci_client_sid_backup = s_ctrl->sensor_i2c_client->cci_client->sid;
			/* Replace the I2C slave address with OIS component */
			s_ctrl->sensor_i2c_client->cci_client->sid = EEPROM_COMPONENT_I2C_ADDR_WRITE >> 1;
			/* Start from 0x1C40, read 5 sectors (0x1C40~0x1D7F) */
			htc_ext_FlashSectorRead(s_ctrl, otp_sector, 0x1C00, 1);
			/* Restore the I2C slave address */
			s_ctrl->sensor_i2c_client->cci_client->sid = cci_client_sid_backup;
			/* Check data availability */
			if( (otp_sector[3] & 0xff) == 0xff)
			{
				pr_err("[OTP]%s: Not QCT sensor!!!!! Skip OTP data\n", __func__);
				if(otp_sector)
					vfree(otp_sector);
				return -EINVAL;
			}
			else
			{
				/* Copy to otp buffer */
				for(i = 0; i < 19; i++)
				{
					if(i == 15)        address += 48;   //skip reserved area, total 48byte
					otp[i] = otp_sector[address+3] & 0xff;
					address+=4;
				}
			}
			if(otp_sector)
				vfree(otp_sector);
		}
	}
//HTC_START , move read OTP to sensor probe
    if(cdata != NULL)
    {
//HTC_END
    cdata->cfg.fuse.fuse_id_word1 = otp[11];
    cdata->cfg.fuse.fuse_id_word2 = otp[12];
    cdata->cfg.fuse.fuse_id_word3 = otp[13];
    cdata->cfg.fuse.fuse_id_word4 = otp[14];

    cdata->af_value.VCM_VENDOR = otp[0];
    cdata->af_value.VCM_VENDOR_ID_VERSION = otp[7];
    cdata->af_value.AF_INF_MSB = otp[15];
    cdata->af_value.AF_INF_LSB = otp[16];
    cdata->af_value.AF_MACRO_MSB = otp[17];
    cdata->af_value.AF_MACRO_LSB = otp[18];

    cdata->alpha.Alpha_Gb = otp[19];
    cdata->alpha.Alpha_B = otp[20];
    cdata->alpha.Alpha_R = otp[21];
    cdata->alpha.Alpha_Gr = otp[22];

    pr_info("%s: OTP Module vendor = 0x%x\n", __func__,  otp[0]);
    pr_info("%s: OTP LENS Vendor & Version = 0x%x, 0x%x\n", __func__,  otp[1], otp[2]);
    pr_info("%s: OTP Sensor Vendor & Version = 0x%x, 0x%x\n", __func__,  otp[3], otp[4]);
    pr_info("%s: OTP Driver IC Vendor & Version = 0x%x, 0x%x\n",  __func__,  otp[5], otp[6]);
    pr_info("%s: OTP Actuator vender ID & Version = 0x%x, 0x%x\n",__func__,  otp[7], otp[8]);

    pr_info("%s: OTP fuse 0 = 0x%x\n", __func__,  cdata->cfg.fuse.fuse_id_word1);
    pr_info("%s: OTP fuse 1 = 0x%x\n", __func__,  cdata->cfg.fuse.fuse_id_word2);
    pr_info("%s: OTP fuse 2 = 0x%x\n", __func__,  cdata->cfg.fuse.fuse_id_word3);
    pr_info("%s: OTP fuse 3 = 0x%x\n", __func__,  cdata->cfg.fuse.fuse_id_word4);

    pr_info("%s: OTP Infinity position code (MSByte) = 0x%x\n", __func__,  cdata->af_value.AF_INF_MSB);
    pr_info("%s: OTP Infinity position code (LSByte) = 0x%x\n", __func__,  cdata->af_value.AF_INF_LSB);
    pr_info("%s: OTP Macro position code (MSByte) = 0x%x\n",    __func__,  cdata->af_value.AF_MACRO_MSB);
    pr_info("%s: OTP Macro position code (LSByte) = 0x%x\n",    __func__,  cdata->af_value.AF_MACRO_LSB);

    pr_info("%s: OTP alpha value (Alpha_Gb) = 0x%x\n", __func__,  cdata->alpha.Alpha_Gb);
    pr_info("%s: OTP alpha value (Alpha_B) = 0x%x\n", __func__,  cdata->alpha.Alpha_B);
    pr_info("%s: OTP alpha value (Alpha_R) = 0x%x\n", __func__,  cdata->alpha.Alpha_R);
    pr_info("%s: OTP alpha value (Alpha_Gr) = 0x%x\n", __func__,  cdata->alpha.Alpha_Gr);

    strlcpy(cdata->af_value.ACT_NAME, "lc898123F40", sizeof("lc898123F40"));
    pr_info("%s: OTP Actuator Name = %s\n",__func__, cdata->af_value.ACT_NAME);
//HTC_START , move read OTP to sensor probe
    }
    else
    {
        pr_info("%s: OTP Module vendor = 0x%x\n", __func__,  otp[0]);
        pr_info("%s: OTP LENS Vendor & Version = 0x%x, 0x%x\n", __func__,  otp[1], otp[2]);
        pr_info("%s: OTP Sensor Vendor & Version = 0x%x, 0x%x\n", __func__,  otp[3], otp[4]);
        pr_info("%s: OTP Driver IC Vendor & Version = 0x%x, 0x%x\n",  __func__,  otp[5], otp[6]);
        pr_info("%s: OTP Actuator vender ID & Version = 0x%x, 0x%x\n",__func__,  otp[7], otp[8]);
    }
//HTC_END
    return rc;
}

//HTC_START , move read OTP to sensor probe
int32_t ov16880_sensor_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	rc = msm_sensor_match_id(s_ctrl);
	return rc;
}
//HTC_END

//HTC_START, get pdaf data
static int ov16880_get_pdaf_size(struct sensorb_cfg_data *cdata,
	struct msm_sensor_ctrl_t *s_ctrl)
{
	return 0;
}

static int ov16880_get_pdaf_size32(struct sensorb_cfg_data32 *cdata,
	struct msm_sensor_ctrl_t *s_ctrl)
{
	if(cdata != NULL)
	{
		if(msm_sensor_get_pdaf_flag(s_ctrl->pdev->dev.of_node))
		{
			cdata->is_pdaf_supported = 1;
			cdata->pdaf_size = OV16880_PDAF_SIZE;
		}
		else
		{
			cdata->is_pdaf_supported = 0;
			cdata->pdaf_size = 0;
		}
	}
	return 0;
}

static int ov16880_read_pdaf_data(struct sensorb_cfg_data *cdata,
	struct msm_sensor_ctrl_t *s_ctrl)
{
	return 0;
}

static int ov16880_read_pdaf_data32(struct sensorb_cfg_data32 *cdata,
	struct msm_sensor_ctrl_t *s_ctrl)
{
#ifdef CONFIG_OIS_CALIBRATION
	int rc = -EINVAL;
	uint8_t *ptr_src = NULL;
	uint8_t *ptr_dest = NULL;
	uint16_t cci_client_sid_backup;

	if(cdata != NULL)
	{
		ptr_src = vzalloc(OV16880_SECTOR_UNIT * OV16880_PDAF_SECTOR);
		if(ptr_src == NULL)
		{
			pr_err("[CAM_PDAF]%s: allocate buffer failed!!!\n", __func__);
			return -ENOMEM;
		}
		else
		{
			ptr_dest = (uint8_t *) compat_ptr(cdata->pdaf_buffer);
			/* Bcakup the I2C slave address */
			cci_client_sid_backup = s_ctrl->sensor_i2c_client->cci_client->sid;
			/* Replace the I2C slave address with OIS component */
			s_ctrl->sensor_i2c_client->cci_client->sid = EEPROM_COMPONENT_I2C_ADDR_WRITE >> 1;
			/* Start from 0x1C40, read 5 sectors (0x1C40~0x1D7F) */
			htc_ext_FlashSectorRead(s_ctrl, ptr_src, 0x1C40, 5);
			/* Restore the I2C slave address */
			s_ctrl->sensor_i2c_client->cci_client->sid = cci_client_sid_backup;
			/* Check if PDAF flag is available */
			if(ptr_src[3] != 0x01)
			{
				pr_err("[CAM_PDAF]%s: No pdaf calibration data\n", __func__);
				rc = -EFAULT;
			}
			else
			{
				/* Copy to user space buffer */
				rc = copy_to_user(ptr_dest, ptr_src, cdata->pdaf_size);
			}
			if(ptr_src)
				vfree(ptr_src);
		}
	}
	return rc;
#else
	return 0;
#endif
}
#endif
//HTC_END

static struct msm_sensor_fn_t ov16880_sensor_func_tbl = {
	.sensor_config = msm_sensor_config,
	.sensor_config32 = msm_sensor_config32,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
    //HTC_START , move read OTP to sensor probe
	.sensor_match_id = ov16880_sensor_match_id,
	.sensor_i2c_read_fuseid = ov16880_read_fuseid,
	.sensor_i2c_read_fuseid32 = ov16880_read_fuseid32,
	//HTC_END
    //HTC_START , get pdaf data
	.sensor_i2c_get_pdaf_size = ov16880_get_pdaf_size,
	.sensor_i2c_get_pdaf_size32 = ov16880_get_pdaf_size32,
	.sensor_i2c_read_pdaf_data = ov16880_read_pdaf_data,
	.sensor_i2c_read_pdaf_data32 = ov16880_read_pdaf_data32,
	//HTC_END
};
/*HTC_END*/

static struct msm_sensor_ctrl_t ov16880_s_ctrl = {
	.sensor_i2c_client = &ov16880_sensor_i2c_client,
	.power_setting_array.power_setting = ov16880_power_setting,
	.power_setting_array.size = ARRAY_SIZE(ov16880_power_setting),
    .msm_sensor_mutex = &ov16880_mut,
	.sensor_v4l2_subdev_info = ov16880_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(ov16880_subdev_info),
	.func_tbl = &ov16880_sensor_func_tbl,
};

module_init(ov16880_init_module);
module_exit(ov16880_exit_module);
MODULE_DESCRIPTION("ov16880");
MODULE_LICENSE("GPL v2");
