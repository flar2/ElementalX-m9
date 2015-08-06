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
#define t4ka7_SENSOR_NAME "t4ka7"
DEFINE_MSM_MUTEX(t4ka7_mut);

static struct msm_sensor_ctrl_t t4ka7_s_ctrl;
static uint32_t Module_Type = 0x00;    

#define DUAL_CAL_OTP_SIZE 2048    
int32_t t4ka7_read_otp_memory(uint8_t *otpPtr, struct msm_sensor_ctrl_t *s_ctrl);

static struct msm_sensor_power_setting t4ka7_power_setting[] = {
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VDIG,
		.config_val = GPIO_OUT_HIGH,
		.delay = 5,
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
		.delay = 1,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};

static struct v4l2_subdev_info t4ka7_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static const struct i2c_device_id t4ka7_i2c_id[] = {
	{t4ka7_SENSOR_NAME, (kernel_ulong_t)&t4ka7_s_ctrl},
	{ }
};

static int32_t msm_t4ka7_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &t4ka7_s_ctrl);
}

static struct i2c_driver t4ka7_i2c_driver = {
	.id_table = t4ka7_i2c_id,
	.probe  = msm_t4ka7_i2c_probe,
	.driver = {
		.name = t4ka7_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client t4ka7_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id t4ka7_dt_match[] = {
	{.compatible = "htc,t4ka7", .data = &t4ka7_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, t4ka7_dt_match);

static struct platform_driver t4ka7_platform_driver = {
	.driver = {
		.name = "htc,t4ka7",
		.owner = THIS_MODULE,
		.of_match_table = t4ka7_dt_match,
	},
};

static const char *t4ka7Vendor = "Toshiba";
static const char *t4ka7NAME = "t4ka7";
static const char *t4ka7Size = "20M";

static ssize_t sensor_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	if(Module_Type == 0x1)
		sprintf(buf, "%s %s %s - Altek\n", t4ka7Vendor, t4ka7NAME, t4ka7Size);
	else
		sprintf(buf, "%s %s %s\n", t4ka7Vendor, t4ka7NAME, t4ka7Size);
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(sensor, 0444, sensor_vendor_show, NULL);

static struct kobject *android_t4ka7;

static int t4ka7_sysfs_init(void)
{
	int ret ;
	pr_info("t4ka7:kobject creat and add\n");
	android_t4ka7 = kobject_create_and_add("android_camera", NULL);
	if (android_t4ka7 == NULL) {
		pr_info("t4ka7_sysfs_init: subsystem_register " \
		"failed\n");
		ret = -ENOMEM;
		return ret ;
	}
	pr_info("t4ka7:sysfs_create_file\n");
	ret = sysfs_create_file(android_t4ka7, &dev_attr_sensor.attr);
	if (ret) {
		pr_info("t4ka7_sysfs_init: sysfs_create_file " \
		"failed\n");
		kobject_del(android_t4ka7);
	}

	return 0 ;
}

static int t4ka7_read_fuseid(struct sensorb_cfg_data *cdata,
	struct msm_sensor_ctrl_t *s_ctrl)
{
    int32_t rc = 0;
    int32_t i = 0, page = 0;
    uint16_t read_data = 0;
    static int first= true;
    static uint8_t otp[23];
    int valid_layer = -1;
    
    static int read_otp = true;
    uint8_t *path= "/data/OTPData.dat";
    struct file* f;
    static uint8_t otp_mem[DUAL_CAL_OTP_SIZE];
    

    if (first)
    {
        first = false;
        rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x2A00, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
        if (rc < 0)
            pr_info("[CAM]%s: i2c_write w 0x2A00 fail\n", __func__);

        for(page = 2; page >= 0; page--)   
        {
            rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x2A02, page, MSM_CAMERA_I2C_BYTE_DATA);
            if (rc < 0)
                pr_info("[CAM]%s: i2c_write w 0x2A02 fail\n", __func__);

            msleep(10);
            for(i = 0; i < 13; i++)
            {
                rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, 0x2A04 + i, &read_data, MSM_CAMERA_I2C_BYTE_DATA);
                if (rc < 0)
                    pr_err("[CAM]%s: i2c_read 0x%x failed\n", __func__, (0x2A04 + i));
                else
                {
                    otp[i] = read_data;
                    if(read_data)
                        valid_layer = page;
                    read_data = 0;
                }
            }

            if(valid_layer != -1)
            {
                pr_info("[CAM]%s: valid_layer of module info:%d \n", __func__,valid_layer);
                break;
            }
		}

        valid_layer = -1;
        for(page = 5; page >= 3; page--)   
        {
            rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x2A02, page, MSM_CAMERA_I2C_BYTE_DATA);
            if (rc < 0)
                pr_info("[CAM]%s: i2c_write w 0x2A02 fail\n", __func__);

            msleep(10);
            for(i = 0; i < 10; i++)
            {
                rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, 0x2A04 + i, &read_data, MSM_CAMERA_I2C_BYTE_DATA);
                if (rc < 0)
                    pr_err("[CAM]%s: i2c_read 0x%x failed\n", __func__, (0x2A04 + i));
                else
                {
                    otp[13+i] = read_data;
                    if(read_data)
                        valid_layer = page;
                    read_data = 0;
                }
            }

            if(valid_layer != -1)
            {
                pr_info("[CAM]%s: valid_layer of VCM info:%d \n", __func__,valid_layer);
                break;
            }
		}

        for(page = 10; page >= 8; page--)   
        {
            rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x2A02, page, MSM_CAMERA_I2C_BYTE_DATA);
            if (rc < 0)
                pr_info("[CAM]%s: i2c_write w 0x2A02 fail\n", __func__);

            msleep(10);
            
            rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, 0x2A32, &read_data, MSM_CAMERA_I2C_BYTE_DATA);
            if (rc < 0)
                pr_err("[CAM]%s: i2c_read 0x2A32 failed\n", __func__);
            else
            {
                if(read_data)
                {
                    Module_Type = 0x01;
                    pr_info("[CAM]%s: valid Altek info\n", __func__);
                    break;
                }
                read_data = 0;
            }
        }

        
        pr_info("%s: read OTP for dual cam calibration\n", __func__);
        t4ka7_read_otp_memory(otp_mem, s_ctrl);
        if (rc<0) {
            pr_err("%s: t4ka7_read_otp_memory failed %d\n", __func__, rc);
            return rc;
        }
        

        rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x2A00, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
        if (rc < 0)
            pr_info("[CAM]%s: i2c_write w 0x2A00 fail\n", __func__);
    }

    if(cdata != NULL)
    {
        
        if (read_otp)
        {
            f = msm_fopen (path, O_CREAT|O_RDWR|O_TRUNC, 0666);
            if (f) {
                msm_fwrite (f, 0, otp_mem, DUAL_CAL_OTP_SIZE);
                msm_fclose (f);
                pr_info ("%s: dump OTP memory successfully\n", __func__);
            } else {
                pr_err ("%s: fail to open file to write OTP memory\n", __func__);
            }
            read_otp = false;
        }
        
    cdata->af_value.VCM_VENDOR = otp[0];
    cdata->af_value.VCM_VENDOR_ID_VERSION = otp[4];
    cdata->af_value.VCM_BIAS = otp[13];
    cdata->af_value.VCM_OFFSET = otp[14];
    cdata->af_value.VCM_BOTTOM_MECH_MSB = otp[15];
    cdata->af_value.VCM_BOTTOM_MECH_LSB = otp[16];
    cdata->af_value.AF_INF_MSB = otp[17];
    cdata->af_value.AF_INF_LSB = otp[18];
    cdata->af_value.AF_MACRO_MSB = otp[19];
    cdata->af_value.AF_MACRO_LSB = otp[20];
    cdata->af_value.VCM_TOP_MECH_MSB = otp[21];
    cdata->af_value.VCM_TOP_MECH_LSB = otp[22];

    cdata->cfg.fuse.fuse_id_word1 = otp[6] << 8 | otp[5];
    cdata->cfg.fuse.fuse_id_word2 = otp[8] << 8 | otp[7];
    cdata->cfg.fuse.fuse_id_word3 = otp[10] << 8 | otp[9];
    cdata->cfg.fuse.fuse_id_word4 = otp[12] << 8 | otp[11];

    cdata->module_type = Module_Type;

    pr_info("%s: OTP Module vendor = 0x%x\n",               __func__,  otp[0]);
    pr_info("%s: OTP LENS = 0x%x\n",                        __func__,  otp[1]);
    pr_info("%s: OTP Sensor Version = 0x%x\n",              __func__,  otp[2]);
    pr_info("%s: OTP Driver IC Vendor & Version = 0x%x\n",  __func__,  otp[3]);
    pr_info("%s: OTP Actuator vender ID & Version = 0x%x\n",__func__,  otp[4]);

    pr_info("%s: OTP fuse 0 = 0x%x\n", __func__,  cdata->cfg.fuse.fuse_id_word1);
    pr_info("%s: OTP fuse 1 = 0x%x\n", __func__,  cdata->cfg.fuse.fuse_id_word2);
    pr_info("%s: OTP fuse 2 = 0x%x\n", __func__,  cdata->cfg.fuse.fuse_id_word3);
    pr_info("%s: OTP fuse 3 = 0x%x\n", __func__,  cdata->cfg.fuse.fuse_id_word4);

    pr_info("%s: OTP BAIS Calibration data = 0x%x\n",           __func__,  cdata->af_value.VCM_BIAS);
    pr_info("%s: OTP OFFSET Calibration data = 0x%x\n",         __func__,  cdata->af_value.VCM_OFFSET);
    pr_info("%s: OTP VCM bottom mech. Limit (MSByte) = 0x%x\n", __func__,  cdata->af_value.VCM_BOTTOM_MECH_MSB);
    pr_info("%s: OTP VCM bottom mech. Limit (LSByte) = 0x%x\n", __func__,  cdata->af_value.VCM_BOTTOM_MECH_LSB);
    pr_info("%s: OTP Infinity position code (MSByte) = 0x%x\n", __func__,  cdata->af_value.AF_INF_MSB);
    pr_info("%s: OTP Infinity position code (LSByte) = 0x%x\n", __func__,  cdata->af_value.AF_INF_LSB);
    pr_info("%s: OTP Macro position code (MSByte) = 0x%x\n",    __func__,  cdata->af_value.AF_MACRO_MSB);
    pr_info("%s: OTP Macro position code (LSByte) = 0x%x\n",    __func__,  cdata->af_value.AF_MACRO_LSB);
    pr_info("%s: OTP VCM top mech. Limit (MSByte) = 0x%x\n",    __func__,  cdata->af_value.VCM_TOP_MECH_MSB);
    pr_info("%s: OTP VCM top mech. Limit (LSByte) = 0x%x\n",    __func__,  cdata->af_value.VCM_TOP_MECH_LSB);

    pr_info("%s: OTP Module type = 0x%x\n", __func__, cdata->module_type);

    strlcpy(cdata->af_value.ACT_NAME, "lc898212", sizeof("lc898212"));
    pr_info("%s: OTP Actuator Name = %s\n",__func__, cdata->af_value.ACT_NAME);
	}
	else
	{
	    pr_info("%s: OTP Module vendor = 0x%x\n",               __func__,  otp[0]);
	    pr_info("%s: OTP LENS = 0x%x\n",                        __func__,  otp[1]);
	    pr_info("%s: OTP Sensor Version = 0x%x\n",              __func__,  otp[2]);
	    pr_info("%s: OTP Driver IC Vendor & Version = 0x%x\n",  __func__,  otp[3]);
	    pr_info("%s: OTP Actuator vender ID & Version = 0x%x\n",__func__,  otp[4]);
	    pr_info("%s: OTP Module type = 0x%x\n",__func__,  Module_Type);
	}
    return rc;

}

static int t4ka7_read_fuseid32(struct sensorb_cfg_data32 *cdata,
	struct msm_sensor_ctrl_t *s_ctrl)
{
    int32_t rc = 0;
    int32_t i = 0, page = 0;
    uint16_t read_data = 0;
    static int first= true;
    static uint8_t otp[23];
    int valid_layer = -1;
    
    static int read_otp = true;
    uint8_t *path= "/data/OTPData.dat";
    struct file* f;
    static uint8_t otp_mem[DUAL_CAL_OTP_SIZE];
    

    if (first)
    {
        first = false;
        rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x2A00, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
        if (rc < 0)
            pr_info("[CAM]%s: i2c_write w 0x2A00 fail\n", __func__);

        for(page = 2; page >= 0; page--)   
        {
            rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x2A02, page, MSM_CAMERA_I2C_BYTE_DATA);
            if (rc < 0)
                pr_info("[CAM]%s: i2c_write w 0x2A02 fail\n", __func__);

            msleep(10);
            for(i = 0; i < 13; i++)
            {
                rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, 0x2A04 + i, &read_data, MSM_CAMERA_I2C_BYTE_DATA);
                if (rc < 0)
                    pr_err("[CAM]%s: i2c_read 0x%x failed\n", __func__, (0x2A04 + i));
                else
                {
                    otp[i] = read_data;
                    if(read_data)
                        valid_layer = page;
                    read_data = 0;
                }
            }

            if(valid_layer != -1)
            {
                pr_info("[CAM]%s: valid_layer of module info:%d \n", __func__,valid_layer);
                break;
            }
		}

        valid_layer = -1;
        for(page = 5; page >= 3; page--)   
        {
            rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x2A02, page, MSM_CAMERA_I2C_BYTE_DATA);
            if (rc < 0)
                pr_info("[CAM]%s: i2c_write w 0x2A02 fail\n", __func__);

            msleep(10);
            for(i = 0; i < 10; i++)
            {
                rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, 0x2A04 + i, &read_data, MSM_CAMERA_I2C_BYTE_DATA);
                if (rc < 0)
                    pr_err("[CAM]%s: i2c_read 0x%x failed\n", __func__, (0x2A04 + i));
                else
                {
                    otp[13+i] = read_data;
                    if(read_data)
                        valid_layer = page;
                    read_data = 0;
                }
            }

            if(valid_layer != -1)
            {
                pr_info("[CAM]%s: valid_layer of VCM info:%d \n", __func__,valid_layer);
                break;
            }
		}

        for(page = 10; page >= 8; page--)   
        {
            rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x2A02, page, MSM_CAMERA_I2C_BYTE_DATA);
            if (rc < 0)
                pr_info("[CAM]%s: i2c_write w 0x2A02 fail\n", __func__);

            msleep(10);
            
            rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, 0x2A32, &read_data, MSM_CAMERA_I2C_BYTE_DATA);
            if (rc < 0)
                pr_err("[CAM]%s: i2c_read 0x2A32 failed\n", __func__);
            else
            {
                if(read_data)
                {
                    Module_Type = 0x01;
                    pr_info("[CAM]%s: valid Altek info\n", __func__);
                    break;
                }
                read_data = 0;
            }
        }

        
        pr_info("%s: read OTP for dual cam calibration\n", __func__);
        t4ka7_read_otp_memory(otp_mem, s_ctrl);
        if (rc<0) {
            pr_err("%s: t4ka7_read_otp_memory failed %d\n", __func__, rc);
            return rc;
        }
        

        rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x2A00, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
        if (rc < 0)
            pr_info("[CAM]%s: i2c_write w 0x2A00 fail\n", __func__);
    }

    if(cdata != NULL)
    {
        
        if (read_otp)
        {
            f = msm_fopen (path, O_CREAT|O_RDWR|O_TRUNC, 0666);
            if (f) {
                msm_fwrite (f, 0, otp_mem, DUAL_CAL_OTP_SIZE);
                msm_fclose (f);
                pr_info ("%s: dump OTP memory successfully\n", __func__);
            } else {
                pr_err ("%s: fail to open file to write OTP memory\n", __func__);
            }
            read_otp = false;
        }
        
    cdata->af_value.VCM_VENDOR = otp[0];
    cdata->af_value.VCM_VENDOR_ID_VERSION = otp[4];
    cdata->af_value.VCM_BIAS = otp[13];
    cdata->af_value.VCM_OFFSET = otp[14];
    cdata->af_value.VCM_BOTTOM_MECH_MSB = otp[15];
    cdata->af_value.VCM_BOTTOM_MECH_LSB = otp[16];
    cdata->af_value.AF_INF_MSB = otp[17];
    cdata->af_value.AF_INF_LSB = otp[18];
    cdata->af_value.AF_MACRO_MSB = otp[19];
    cdata->af_value.AF_MACRO_LSB = otp[20];
    cdata->af_value.VCM_TOP_MECH_MSB = otp[21];
    cdata->af_value.VCM_TOP_MECH_LSB = otp[22];

    cdata->cfg.fuse.fuse_id_word1 = otp[6] << 8 | otp[5];
    cdata->cfg.fuse.fuse_id_word2 = otp[8] << 8 | otp[7];
    cdata->cfg.fuse.fuse_id_word3 = otp[10] << 8 | otp[9];
    cdata->cfg.fuse.fuse_id_word4 = otp[12] << 8 | otp[11];

    cdata->module_type = Module_Type;

    pr_info("%s: OTP Module vendor = 0x%x\n",               __func__,  otp[0]);
    pr_info("%s: OTP LENS = 0x%x\n",                        __func__,  otp[1]);
    pr_info("%s: OTP Sensor Version = 0x%x\n",              __func__,  otp[2]);
    pr_info("%s: OTP Driver IC Vendor & Version = 0x%x\n",  __func__,  otp[3]);
    pr_info("%s: OTP Actuator vender ID & Version = 0x%x\n",__func__,  otp[4]);

    pr_info("%s: OTP fuse 0 = 0x%x\n", __func__,  cdata->cfg.fuse.fuse_id_word1);
    pr_info("%s: OTP fuse 1 = 0x%x\n", __func__,  cdata->cfg.fuse.fuse_id_word2);
    pr_info("%s: OTP fuse 2 = 0x%x\n", __func__,  cdata->cfg.fuse.fuse_id_word3);
    pr_info("%s: OTP fuse 3 = 0x%x\n", __func__,  cdata->cfg.fuse.fuse_id_word4);

    pr_info("%s: OTP BAIS Calibration data = 0x%x\n",           __func__,  cdata->af_value.VCM_BIAS);
    pr_info("%s: OTP OFFSET Calibration data = 0x%x\n",         __func__,  cdata->af_value.VCM_OFFSET);
    pr_info("%s: OTP VCM bottom mech. Limit (MSByte) = 0x%x\n", __func__,  cdata->af_value.VCM_BOTTOM_MECH_MSB);
    pr_info("%s: OTP VCM bottom mech. Limit (LSByte) = 0x%x\n", __func__,  cdata->af_value.VCM_BOTTOM_MECH_LSB);
    pr_info("%s: OTP Infinity position code (MSByte) = 0x%x\n", __func__,  cdata->af_value.AF_INF_MSB);
    pr_info("%s: OTP Infinity position code (LSByte) = 0x%x\n", __func__,  cdata->af_value.AF_INF_LSB);
    pr_info("%s: OTP Macro position code (MSByte) = 0x%x\n",    __func__,  cdata->af_value.AF_MACRO_MSB);
    pr_info("%s: OTP Macro position code (LSByte) = 0x%x\n",    __func__,  cdata->af_value.AF_MACRO_LSB);
    pr_info("%s: OTP VCM top mech. Limit (MSByte) = 0x%x\n",    __func__,  cdata->af_value.VCM_TOP_MECH_MSB);
    pr_info("%s: OTP VCM top mech. Limit (LSByte) = 0x%x\n",    __func__,  cdata->af_value.VCM_TOP_MECH_LSB);

    pr_info("%s: OTP Module type = 0x%x\n", __func__, cdata->module_type);

    strlcpy(cdata->af_value.ACT_NAME, "lc898212", sizeof("lc898212"));
    pr_info("%s: OTP Actuator Name = %s\n",__func__, cdata->af_value.ACT_NAME);
	}
	else
	{
	    pr_info("%s: OTP Module vendor = 0x%x\n",               __func__,  otp[0]);
	    pr_info("%s: OTP LENS = 0x%x\n",                        __func__,  otp[1]);
	    pr_info("%s: OTP Sensor Version = 0x%x\n",              __func__,  otp[2]);
	    pr_info("%s: OTP Driver IC Vendor & Version = 0x%x\n",  __func__,  otp[3]);
	    pr_info("%s: OTP Actuator vender ID & Version = 0x%x\n",__func__,  otp[4]);
	    pr_info("%s: OTP Module type = 0x%x\n",__func__,  Module_Type);
	}
    return rc;

}

int32_t t4ka7_read_otp_memory(uint8_t *otpPtr, struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	uint16_t read_data = 0;
	int page = 0, i = 0, j = 0;
	short OTP_addr = 0x2A04;

	
	for (page = 0; page < 32; page++)
	{
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x2A02, page, MSM_CAMERA_I2C_BYTE_DATA);
		if (rc < 0)
			pr_info("%s: i2c_write w 0x2A02 fail\n", __func__);

		for (i = 0; i < 64; i++) {
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
					s_ctrl->sensor_i2c_client,
					OTP_addr,
					&read_data,
					MSM_CAMERA_I2C_BYTE_DATA);
			if (rc < 0){
				pr_err("%s: i2c_read 0x%x failed\n", __func__, OTP_addr);
				return rc;
			}
			otpPtr[j] = read_data;
			OTP_addr += 0x1;
			j++;
		}
		OTP_addr = 0x2A04;
	}
	pr_info("%s: read OTP memory done\n", __func__);
	return rc;
}

static int t4ka7_read_emmc32(struct sensorb_cfg_data32 *cdata)
{
	int ret = 0;
	unsigned char *ptr;

	ptr = get_cam_emmc_cal(&ret);
	if(ret)
	{
#if 0
		if(cdata==NULL || copy_to_user((void *)compat_ptr(cdata->cfg.setting), ptr, ret))
		{
			pr_err("[CAM]%s: get camera emmc data error!\n", __func__);
			ret = -1;
		}
		else
			pr_info("[CAM]%s: get camera emmc data successfully!\n", __func__);
#endif
		pr_info("[CAM]%s: get camera emmc data successfully!\n", __func__);
	}
	else
	{
		pr_info("[CAM]%s: get_cam_emmc_cal = NULL\n", __func__);
		ret = -1;
	}

	return ret;
}

static int t4ka7_read_emmc(struct sensorb_cfg_data *cdata)
{
	int ret = 0;
	unsigned char *ptr;

	ptr = get_cam_emmc_cal(&ret);
	if(ret)
	{
#if 0
		if(cdata==NULL || copy_to_user((void *)cdata->cfg.setting, ptr, ret))
		{
			pr_err("[CAM]%s: Can't copy emmc data to user\n", __func__);
			ret = -1;
		}
		else
			pr_info("[CAM]%s: get camera emmc data successfully!\n", __func__);
#endif
		pr_info("[CAM]%s: get camera emmc data successfully!\n", __func__);
	}
	else
	{
		pr_info("[CAM]%s: get_cam_emmc_cal = NULL\n", __func__);
		ret = -1;
	}

	return ret;
}


static int32_t t4ka7_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	match = of_match_device(t4ka7_dt_match, &pdev->dev);
	if (match)
		rc = msm_sensor_platform_probe(pdev, match->data);
	else {
		pr_err("%s:%d match is null\n", __func__, __LINE__);
		rc = -EINVAL;
	}
	return rc;
}

static void __init t4ka7_init_module_async(void *unused, async_cookie_t cookie)
{
	int32_t rc = 0;
	async_synchronize_cookie(cookie);
	pr_info("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&t4ka7_platform_driver,
		t4ka7_platform_probe);
	if (!rc) {
		t4ka7_sysfs_init();
		return;
	}
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	i2c_add_driver(&t4ka7_i2c_driver);
}

static int __init t4ka7_init_module(void)
{
	async_schedule(t4ka7_init_module_async, NULL);
	return 0;
}

static void __exit t4ka7_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (t4ka7_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&t4ka7_s_ctrl);
		platform_driver_unregister(&t4ka7_platform_driver);
	} else
		i2c_del_driver(&t4ka7_i2c_driver);
	return;
}

int32_t t4ka7_sensor_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = -22;
	int32_t rc1 = 0;
	static int first = 0;
	uint16_t chipid;
	rc = msm_sensor_match_id(s_ctrl);
	chipid = 0;

	if(rc != 0)
	{
		
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
			s_ctrl->sensor_i2c_client, 0x0,
			&chipid, MSM_CAMERA_I2C_WORD_DATA);
		if (rc < 0) {
			pr_err("%s: read t4ka7 id failed\n", __func__);
			return rc;
		}
		if (chipid != 0x0000) {
			pr_err("%s: Not to probe t4ka7 sensor\n", __func__);
			return -ENODEV;
		}
		
	}

	
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
	    s_ctrl->sensor_i2c_client, 0x0016,
	    &chipid, MSM_CAMERA_I2C_WORD_DATA);
	if (rc < 0) {
	    pr_err("%s: read imx220 id failed\n", __func__);
	    return rc;
	}
	if (chipid == 0x0220) {
	    pr_err("%s: Not to probe imx220 sensor\n", __func__);
	    return -ENODEV;
	}
	

	if(first == 0)
	{
	    pr_info("%s read_fuseid\n",__func__);
		#ifdef CONFIG_COMPAT
	    rc1 = t4ka7_read_fuseid32(NULL, s_ctrl);
	    #else
	    rc1 = t4ka7_read_fuseid(NULL, s_ctrl);
	    #endif
	    first = 1;
	}
	return rc;
}

static struct msm_sensor_fn_t t4ka7_sensor_func_tbl = {
	.sensor_config = msm_sensor_config,
	.sensor_config32 = msm_sensor_config32,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = t4ka7_sensor_match_id,
	.sensor_i2c_read_fuseid = t4ka7_read_fuseid,
	.sensor_i2c_read_fuseid32 = t4ka7_read_fuseid32,
	.sensor_i2c_read_emmc = t4ka7_read_emmc,
	.sensor_i2c_read_emmc32 = t4ka7_read_emmc32,
};

static struct msm_sensor_ctrl_t t4ka7_s_ctrl = {
	.sensor_i2c_client = &t4ka7_sensor_i2c_client,
	.power_setting_array.power_setting = t4ka7_power_setting,
	.power_setting_array.size = ARRAY_SIZE(t4ka7_power_setting),
	.msm_sensor_mutex = &t4ka7_mut,
	.sensor_v4l2_subdev_info = t4ka7_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(t4ka7_subdev_info),
	.func_tbl = &t4ka7_sensor_func_tbl
};

module_init(t4ka7_init_module);
module_exit(t4ka7_exit_module);
MODULE_DESCRIPTION("t4ka7");
MODULE_LICENSE("GPL v2");
