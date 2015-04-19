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
#define VD6869_SENSOR_NAME "vd6869"
DEFINE_MSM_MUTEX(vd6869_mut);
#define OTP_WAIT_TIMEOUT 200
#define OTP_BUFFER_OFFSET 0x33FA
#define OTP_STATUS_REG 0x3302
static int vd6869_shut_down_otp(struct msm_sensor_ctrl_t *s_ctrl,uint16_t addr, uint16_t data);
static int vd6869_init_otp(struct msm_sensor_ctrl_t *s_ctrl);

static struct msm_sensor_ctrl_t vd6869_s_ctrl;

static struct msm_sensor_power_setting vd6869_power_setting[] = {
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 1,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 1,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 1,
		.delay = 5,
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

static struct v4l2_subdev_info vd6869_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static const struct i2c_device_id vd6869_i2c_id[] = {
	{VD6869_SENSOR_NAME, (kernel_ulong_t)&vd6869_s_ctrl},
	{ }
};

static int32_t msm_vd6869_i2c_probe(struct i2c_client *client,
       const struct i2c_device_id *id)
{
       return msm_sensor_i2c_probe(client, id, &vd6869_s_ctrl);
}

static struct i2c_driver vd6869_i2c_driver = {
	.id_table = vd6869_i2c_id,
	.probe  = msm_vd6869_i2c_probe,
	.driver = {
		.name = VD6869_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client vd6869_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id vd6869_dt_match[] = {
	{.compatible = "htc,vd6869", .data = &vd6869_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, vd6869_dt_match);

static struct platform_driver vd6869_platform_driver = {
	.driver = {
		.name = "htc,vd6869",
		.owner = THIS_MODULE,
		.of_match_table = vd6869_dt_match,
	},
};

#define VD6869_VER_UNKNOWN 0xFF

struct vd6869_ver_map {
	uint8_t val;
	char *str;
};

static struct vd6869_ver_map vd6869_ver_tab[] = {  
	{ 0x09, "(0.9)"},	
	{ 0x0A, "(0.9e)"},	
	{ 0x10, "(1.0)"},	
	{ VD6869_VER_UNKNOWN, "(unknown)"}  
};

static uint8_t vd6869_ver = VD6869_VER_UNKNOWN;
static uint8_t vd6869_year_mon = 0;
static uint8_t vd6869_date = 0;

static const char *vd6869Vendor = "st";
static const char *vd6869NAME = "vd6869";
static const char *vd6869Size = "cinesensor";

static ssize_t sensor_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	uint8_t i = 0;
	uint8_t len = ARRAY_SIZE(vd6869_ver_tab);
	char vd6869NAME_ver[32];
	uint16_t year = 0;
	uint8_t month = 0;
	uint8_t date = 0;
	pr_info("%s called\n", __func__);

	memset(vd6869NAME_ver, 0, sizeof(vd6869NAME_ver));
	for (i = 0; i < len; i++) {
		if (vd6869_ver == vd6869_ver_tab[i].val)
			break;
	}
	if (i < len)  
		snprintf(vd6869NAME_ver, sizeof(vd6869NAME_ver), "%s%s",
			vd6869NAME, vd6869_ver_tab[i].str);
	else  
		snprintf(vd6869NAME_ver, sizeof(vd6869NAME_ver), "%s%s-%02X",
			vd6869NAME, vd6869_ver_tab[len - 1].str, vd6869_ver);
	pr_info("%s: version(%d) : %s\n", __func__, vd6869_ver, vd6869NAME_ver);

	year  = ((vd6869_year_mon & 0xf0)>> 4);
	month = (vd6869_year_mon & 0x0f);
	date  = ((vd6869_date & 0xf8) >> 3);

	if((year == 0)&&(month == 0)&&(date == 0))
		pr_err("%s: Invalid OTP date\n", __func__);
	else
		year += 2000; 

	snprintf(buf, PAGE_SIZE, "%s %s %s %04d-%02d-%02d \n", vd6869Vendor, vd6869NAME_ver, vd6869Size, year, month, date);
	  
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(sensor, 0444, sensor_vendor_show, NULL);

static struct kobject *android_vd6869;

static int vd6869_sysfs_init(void)
{
	int ret ;
	pr_info("%s: vd6869:kobject creat and add\n", __func__);

	android_vd6869 = kobject_create_and_add("android_camera2", NULL);
	if (android_vd6869 == NULL) {
		pr_info("vd6869_sysfs_init: subsystem_register " \
		"failed\n");
		ret = -ENOMEM;
		return ret ;
	}
	pr_info("vd6869:sysfs_create_file\n");
	ret = sysfs_create_file(android_vd6869, &dev_attr_sensor.attr);
	if (ret) {
		pr_info("vd6869_sysfs_init: sysfs_create_file " \
		"failed\n");
		kobject_del(android_vd6869);
	}

	return 0 ;
}

static int vd6869_read_fuseid(struct sensorb_cfg_data *cdata,
	struct msm_sensor_ctrl_t *s_ctrl)
{
	#define VD6869_OTP_SIZE 6
    int32_t rc = 0;
    int32_t i = 0, j = 2;
    uint16_t read_data = 0;
    static int first= true;
    const int32_t offset = OTP_BUFFER_OFFSET;
    static uint8_t otp[VD6869_OTP_SIZE];
    int valid_layer = -1;

    const short addr[3][VD6869_OTP_SIZE] = {
        {0x3C8,0x3C9,0x3CA,0x3A0,0x3A1,0x3A2}, 
        {0x3D8,0x3D9,0x3DA,0x380,0x381,0x382}, 
        {0x3B8,0x3B9,0x3BA,0x388,0x389,0x38A}, 
    };

    if (first)
    {
        first = false;
        rc = vd6869_init_otp(s_ctrl);
        if (rc<0)
        {
            pr_info("[CAM]%s: i2c_write w 0x2A02 fail\n", __func__);
            return rc;
        }

		for (j=2; j>=0; j--)
		{
			for (i=0; i<VD6869_OTP_SIZE; ++i)
			{
				rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
						s_ctrl->sensor_i2c_client,
						addr[j][i]+offset,
						&read_data,
						MSM_CAMERA_I2C_BYTE_DATA);
				if (rc < 0){
					pr_err("%s: i2c_read 0x%x failed\n", __func__, addr[j][i]);
					return rc;
				}
                else
                {
                    otp[i] = read_data;
                    if(read_data)
                        valid_layer = j;
                }
			}
            if(valid_layer != -1)
            {
                pr_info("[CAM]%s: valid_layer of module info:%d \n", __func__, valid_layer);
                break;
            }
		}
    }

    if(cdata != NULL)
    {
    cdata->cfg.fuse.fuse_id_word1 = 0;
    cdata->cfg.fuse.fuse_id_word2 = otp[3];
    cdata->cfg.fuse.fuse_id_word3 = otp[4];
    cdata->cfg.fuse.fuse_id_word4 = otp[5];

    pr_info("%s: OTP Module vendor = 0x%x\n",               __func__,  otp[0]);
    pr_info("%s: OTP LENS = 0x%x\n",                        __func__,  otp[1]);
    pr_info("%s: OTP Sensor Version = 0x%x\n",              __func__,  otp[2]);

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
	}
    return rc;

}

static int vd6869_read_fuseid32(struct sensorb_cfg_data32 *cdata,
	struct msm_sensor_ctrl_t *s_ctrl)
{
	#define VD6869_OTP_SIZE 6
    int32_t rc = 0;
    int32_t i = 0, j = 2;
    uint16_t read_data = 0;
    static int first= true;
    const int32_t offset = OTP_BUFFER_OFFSET;
    static uint8_t otp[VD6869_OTP_SIZE];
    int valid_layer = -1;

    const short addr[3][VD6869_OTP_SIZE] = {
        {0x3C8,0x3C9,0x3CA,0x3A0,0x3A1,0x3A2}, 
        {0x3D8,0x3D9,0x3DA,0x380,0x381,0x382}, 
        {0x3B8,0x3B9,0x3BA,0x388,0x389,0x38A}, 
    };

    if (first)
    {
        first = false;
        rc = vd6869_init_otp(s_ctrl);
        if (rc<0)
        {
            pr_info("[CAM]%s: i2c_write w 0x2A02 fail\n", __func__);
            return rc;
        }

		for (j=2; j>=0; j--)
		{
			for (i=0; i<VD6869_OTP_SIZE; ++i)
			{
				rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
						s_ctrl->sensor_i2c_client,
						addr[j][i]+offset,
						&read_data,
						MSM_CAMERA_I2C_BYTE_DATA);
				if (rc < 0){
					pr_err("%s: i2c_read 0x%x failed\n", __func__, addr[j][i]);
					return rc;
				}
                else
                {
                    otp[i] = read_data;
                    if(read_data)
                        valid_layer = j;
                }
			}
            if(valid_layer != -1)
            {
                pr_info("[CAM]%s: valid_layer of module info:%d \n", __func__, valid_layer);
                break;
            }
		}
    }

    if(cdata != NULL)
    {
    cdata->cfg.fuse.fuse_id_word1 = 0;
    cdata->cfg.fuse.fuse_id_word2 = otp[3];
    cdata->cfg.fuse.fuse_id_word3 = otp[4];
    cdata->cfg.fuse.fuse_id_word4 = otp[5];

    pr_info("%s: OTP Module vendor = 0x%x\n",               __func__,  otp[0]);
    pr_info("%s: OTP LENS = 0x%x\n",                        __func__,  otp[1]);
    pr_info("%s: OTP Sensor Version = 0x%x\n",              __func__,  otp[2]);

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
	}
    return rc;
}

static struct msm_camera_i2c_reg_array otp_reg_settings[] = {
    {0x44c0, 0x01},
    {0x4500, 0x01},
    {0x44e4, 0x00},
    {0x4524, 0x00},
    {0x4584, 0x01},
    {0x44ec, 0x01},
    {0x44ed, 0x80},
    {0x44f0, 0x04},
    {0x44f1, 0xb0},
    {0x452c, 0x01},
    {0x452d, 0x80},
    {0x4530, 0x04},
    {0x4531, 0xb0},
	{0x3305, 0x00},
    {0x3303, 0x01},
	{0x3304, 0x00},
    {0x3301, 0x02},
};

static  struct msm_camera_i2c_reg_setting otp_settings = {
  .reg_setting = otp_reg_settings,
  .size = ARRAY_SIZE(otp_reg_settings),
  .addr_type = MSM_CAMERA_I2C_WORD_ADDR,
  .data_type = MSM_CAMERA_I2C_BYTE_DATA,
  .delay = 0,
};

static int vd6869_init_otp(struct msm_sensor_ctrl_t *s_ctrl){
	int i,rc = 0;
	uint16_t read_data = 0;

	
	for(i = 0 ;i < OTP_WAIT_TIMEOUT; i++) {
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_table(
				s_ctrl->sensor_i2c_client,
				&otp_settings);

		if(rc < 0)
			pr_err("%s write otp init table error.... retry", __func__);
		else{
			pr_info("%s OTP table init done",__func__);
			break;
		}
		mdelay(1);
	}

	
	for(i = 0 ;i < OTP_WAIT_TIMEOUT; i++){
		
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
				s_ctrl->sensor_i2c_client,
				OTP_STATUS_REG,
				&read_data,
				MSM_CAMERA_I2C_BYTE_DATA);

		if(rc < 0){
			pr_err("%s read OTP status error",__func__);
		} else if(read_data == 0x00){
			rc = vd6869_shut_down_otp(s_ctrl,0x4584,0x00);
			if(rc < 0)
				return rc;
			rc = vd6869_shut_down_otp(s_ctrl,0x44c0,0x00);
			if(rc < 0)
				return rc;
			rc = vd6869_shut_down_otp(s_ctrl,0x4500,0x00);
			if(rc < 0)
				return rc;
			break;
		}
		mdelay(1);
	}
	return rc;
}

static int vd6869_shut_down_otp(struct msm_sensor_ctrl_t *s_ctrl,uint16_t addr, uint16_t data){
	int rc=0,i;
	for(i = 0; i < OTP_WAIT_TIMEOUT;i++){
		
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
				s_ctrl->sensor_i2c_client,
				addr,
				data,
				MSM_CAMERA_I2C_BYTE_DATA);

		if(rc < 0){
			pr_err("%s shut down OTP error 0x%x:0x%x\n",__func__,addr,data);
		}else{
			pr_info("%s shut down OTP success 0x%x:0x%x\n",__func__,addr,data);
			return rc;
		}
		mdelay(1);
	}
	pr_err("%s shut down time out 0x%x",__func__,addr);
	return rc;
}

static int32_t vd6869_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	match = of_match_device(vd6869_dt_match, &pdev->dev);
	if (match)
		rc = msm_sensor_platform_probe(pdev, match->data);
	else {
		pr_err("%s:%d match is null\n", __func__, __LINE__);
		rc = -EINVAL;
	}
	return rc;
}

static void __init vd6869_init_module_async(void *unused, async_cookie_t cookie)
{
	int32_t rc = 0;
	async_synchronize_cookie(cookie);
	pr_info("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&vd6869_platform_driver,
		vd6869_platform_probe);
	if (!rc) {
		vd6869_sysfs_init();
		return;
	}
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	i2c_add_driver(&vd6869_i2c_driver);
}

static int __init vd6869_init_module(void)
{
	async_schedule(vd6869_init_module_async, NULL);
	return 0;
}

static void __exit vd6869_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (vd6869_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&vd6869_s_ctrl);
		platform_driver_unregister(&vd6869_platform_driver);
	} else
		i2c_del_driver(&vd6869_i2c_driver);
	return;
}

int32_t vd6869_sensor_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = -22;
	int32_t rc1 = 0;
	static int first = 0;
	rc = msm_sensor_match_id(s_ctrl);
	if(rc == 0)
	{
	    if(first == 0)
	    {
	        pr_info("%s read_fuseid\n",__func__);
			#ifdef CONFIG_COMPAT
	        rc1 = vd6869_read_fuseid32(NULL, s_ctrl);
	        #else
	        rc1 = vd6869_read_fuseid(NULL, s_ctrl);
	        #endif
	        first = 1;
	    }
	}
	return rc;
}

static struct msm_sensor_fn_t vd6869_sensor_func_tbl = {
	.sensor_config = msm_sensor_config,
	.sensor_config32 = msm_sensor_config32,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = vd6869_sensor_match_id,
	.sensor_i2c_read_fuseid = vd6869_read_fuseid,
	.sensor_i2c_read_fuseid32 = vd6869_read_fuseid32,
};

static struct msm_sensor_ctrl_t vd6869_s_ctrl = {
	.sensor_i2c_client = &vd6869_sensor_i2c_client,
	.power_setting_array.power_setting = vd6869_power_setting,
	.power_setting_array.size = ARRAY_SIZE(vd6869_power_setting),
	.msm_sensor_mutex = &vd6869_mut,
	.sensor_v4l2_subdev_info = vd6869_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(vd6869_subdev_info),
	.func_tbl = &vd6869_sensor_func_tbl, 
};

module_init(vd6869_init_module);
module_exit(vd6869_exit_module);
MODULE_DESCRIPTION("vd6869");
MODULE_LICENSE("GPL v2");
