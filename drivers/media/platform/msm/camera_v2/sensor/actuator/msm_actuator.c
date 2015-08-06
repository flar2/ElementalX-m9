/* Copyright (c) 2011-2014, The Linux Foundation. All rights reserved.
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

#define pr_fmt(fmt) "%s:%d " fmt, __func__, __LINE__

#include <linux/module.h>
#include "msm_sd.h"
#include "msm_actuator.h"
#include "msm_cci.h"

DEFINE_MSM_MUTEX(msm_actuator_mutex);

#undef CDBG
#define CDBG(fmt, args...) pr_debug(fmt, ##args)

#define PARK_LENS_LONG_STEP 7
#define PARK_LENS_MID_STEP 5
#define PARK_LENS_SMALL_STEP 3

static struct v4l2_file_operations msm_actuator_v4l2_subdev_fops;
static int32_t msm_actuator_power_up(struct msm_actuator_ctrl_t *a_ctrl);
static int32_t msm_actuator_power_down(struct msm_actuator_ctrl_t *a_ctrl);

static struct msm_actuator msm_vcm_actuator_table;
static struct msm_actuator msm_piezo_actuator_table;
static struct msm_actuator msm_hvcm_actuator_table;

static struct i2c_driver msm_actuator_i2c_driver;
static struct msm_actuator *actuators[] = {
	&msm_vcm_actuator_table,
	&msm_piezo_actuator_table,
	&msm_hvcm_actuator_table,
};


#define DEFAULT_BIAS 0x63     
#define DEFAULT_OFFSET 0x57   
#define DEFAULT_INFINITY 0x7000
#define DEFAULT_MACRO -0x7000

static struct msm_camera_i2c_reg_array lc898212_settings_1[] = {
{0x80, 0x34},
{0x81, 0x20},
{0x84, 0xE0},
{0x87, 0x05},
{0xA4, 0x24},
{0x8B, 0x80},
{0x3A, 0x00},
{0x3B, 0x00},
{0x04, 0x00},
{0x05, 0x00},
{0x02, 0x00},
{0x03, 0x00},
{0x18, 0x00},
{0x19, 0x00},
{0x28, 0x80},
{0x29, 0x80},
{0x83, 0x2C},
{0x84, 0xE3},
{0x97, 0x00},
{0x98, 0x42},
{0x99, 0x00},
{0x9A, 0x00},
};

static struct msm_camera_i2c_reg_array lc898212_settings_2_MTM[] = {
{0x88, 0x68},
{0x92, 0x00},
{0xA0, 0x01},
{0x7A, 0x68},
{0x7B, 0x00},
{0x7E, 0x78},
{0x7F, 0x00},
{0x7C, 0x01},
{0x7D, 0x00},
{0x93, 0xC0},
{0x86, 0x60},

{0x40, 0x80},
{0x41, 0x10},
{0x42, 0x75},
{0x43, 0x70},
{0x44, 0x8B},
{0x45, 0x50},
{0x46, 0x6A},
{0x47, 0x10},
{0x48, 0x5A},
{0x49, 0x90},
{0x76, 0x0C},
{0x77, 0x50},
{0x4A, 0x20},
{0x4B, 0x30},
{0x50, 0x04},
{0x51, 0xF0},
{0x52, 0x76},
{0x53, 0x10},
{0x54, 0x14},
{0x55, 0x50},
{0x56, 0x00},
{0x57, 0x00},
{0x58, 0x7F},
{0x59, 0xF0},
{0x4C, 0x32},
{0x4D, 0xF0},
{0x78, 0x20},
{0x79, 0x00},
{0x4E, 0x7F},
{0x4F, 0xF0},
{0x6E, 0x00},
{0x6F, 0x00},
{0x72, 0x18},
{0x73, 0xE0},
{0x74, 0x4E},
{0x75, 0x30},
{0x30, 0x00},
{0x31, 0x00},
{0x5A, 0x06},
{0x5B, 0x80},
{0x5C, 0x72},
{0x5D, 0xF0},
{0x5E, 0x7F},
{0x5F, 0x70},
{0x60, 0x7E},
{0x61, 0xD0},
{0x62, 0x7F},
{0x63, 0xF0},
{0x64, 0x00},
{0x65, 0x00},
{0x66, 0x00},
{0x67, 0x00},
{0x68, 0x51},
{0x69, 0x30},
{0x6A, 0x72},
{0x6B, 0xF0},
{0x70, 0x00},
{0x71, 0x00},
{0x6C, 0x80},
{0x6D, 0x10},

{0x76, 0x0c},
{0x77, 0x50},
{0x78, 0x20},
{0x79, 0x00},
{0x30, 0x00},
{0x31, 0x00},
};

static struct msm_camera_i2c_reg_array lc898212_settings_2_TDK[] = {
{0x88, 0x70},
{0x92, 0x00},
{0xA0, 0x02},
{0x7A, 0x68},
{0x7B, 0x00},
{0x7E, 0x78},
{0x7F, 0x00},
{0x7C, 0x01},
{0x7D, 0x00},
{0x93, 0xC0},
{0x86, 0x60},

{0x40, 0x40},
{0x41, 0x30},
{0x42, 0x71},
{0x43, 0x50},
{0x44, 0x8F},
{0x45, 0x90},
{0x46, 0x61},
{0x47, 0xB0},
{0x48, 0x7F},
{0x49, 0xF0},
{0x76, 0x0C},
{0x77, 0x50},
{0x4A, 0x39},
{0x4B, 0x30},
{0x50, 0x04},
{0x51, 0xF0},
{0x52, 0x76},
{0x53, 0x10},
{0x54, 0x20},
{0x55, 0x30},
{0x56, 0x00},
{0x57, 0x00},
{0x58, 0x7F},
{0x59, 0xF0},
{0x4C, 0x40},
{0x4D, 0x30},
{0x78, 0x40},
{0x79, 0x00},
{0x4E, 0x80},
{0x4F, 0x10},
{0x6E, 0x00},
{0x6F, 0x00},
{0x72, 0x18},
{0x73, 0xE0},
{0x74, 0x4E},
{0x75, 0x30},
{0x30, 0x00},
{0x31, 0x00},
{0x5A, 0x06},
{0x5B, 0x80},
{0x5C, 0x72},
{0x5D, 0xF0},
{0x5E, 0x7F},
{0x5F, 0x70},
{0x60, 0x7E},
{0x61, 0xD0},
{0x62, 0x7F},
{0x63, 0xF0},
{0x64, 0x00},
{0x65, 0x00},
{0x66, 0x00},
{0x67, 0x00},
{0x68, 0x51},
{0x69, 0x30},
{0x6A, 0x72},
{0x6B, 0xF0},
{0x70, 0x00},
{0x71, 0x00},
{0x6C, 0x80},
{0x6D, 0x10},

{0x76, 0x0c},
{0x77, 0x50},
{0x78, 0x40},
{0x79, 0x00},
{0x30, 0x00},
{0x31, 0x00},
};

static struct msm_camera_i2c_reg_array lc898212_settings_3[] = {
{0x3A, 0x00},
{0x3B, 0x00}, 
{0x04, 0x00},
{0x05, 0x00}, 
{0x02, 0x00},
{0x03, 0x00}, 
{0x85, 0xC0}, 
};
static struct msm_camera_i2c_reg_array lc898212_settings_4[] = {
{0x5A, 0x08},
{0x5B, 0x00},
{0x83, 0xac},
};

static int16_t af_actuator_lc898212_dec2hex(int input_dec)
{
	int16_t hex_min = LC898212_HEX_MIN;
	int16_t hex_max = LC898212_HEX_MAX;
	int dec_max_Pos = LC898212_DEC_MAX;
	int16_t hex_output_position = 0;

	if (dec_max_Pos == 0) {
		pr_err("dec_max_Pos = 0!");
		return 0;
	}
	hex_output_position = hex_max - (dec_max_Pos - input_dec) * (hex_max - hex_min) / dec_max_Pos;

	CDBG("input_dec:%d. hex_output_position:0x%x (%d)", input_dec, (uint16_t)hex_output_position, hex_output_position);
	return hex_output_position;
}

static int32_t lc898212_wrapper_i2c_write(struct msm_actuator_ctrl_t *a_ctrl,
    int16_t next_lens_position, int8_t dir, uint16_t wait_time)
{
    int32_t rc = 0;
    int16_t hex_next_lens_pos = af_actuator_lc898212_dec2hex(next_lens_position);

    
    if (a_ctrl->enable_focus_step_log)
        pr_info("next_lens_position:%d -> 0x%x (%d)\n", next_lens_position, (uint16_t)hex_next_lens_pos, hex_next_lens_pos);

    CDBG("%s addr_type: %d, sid 0x%x (0x%x), \n", __func__, a_ctrl->i2c_client.addr_type, a_ctrl->i2c_client.cci_client->sid, (a_ctrl->i2c_client.cci_client->sid<<1));

    rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_write(&a_ctrl->i2c_client,
            0xa1,
            hex_next_lens_pos,
            MSM_CAMERA_I2C_WORD_DATA);
    if (rc < 0) {
        pr_err("%s 0xa1 i2c write failed (%d)\n", __func__, rc);
        return rc;
    }

    switch (a_ctrl->af_OTP_info.VCM_Vendor_Id_Version)
    {
        case 0x61:
            rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_write(&a_ctrl->i2c_client,
                0x16,
                (dir > 0) ? 0x180 : 0xfe80,
                MSM_CAMERA_I2C_WORD_DATA);
            break;
        case 0x71:
        case 0x72:
            rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_write(&a_ctrl->i2c_client,
                0x16,
                (dir > 0) ? 0xfe80 : 0x180,
                MSM_CAMERA_I2C_WORD_DATA);
            break;
        default:
            rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_write(&a_ctrl->i2c_client,
                0x16,
                (dir > 0) ? 0x180 : 0xfe80,
                MSM_CAMERA_I2C_WORD_DATA);
            break;
    }

    if (rc < 0) {
        pr_err("%s 0x16 i2c write failed (%d)\n", __func__, rc);
        return rc;
    }

    rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_write(&a_ctrl->i2c_client,
         0x8a,
         0xd,
         MSM_CAMERA_I2C_BYTE_DATA);
    if (rc < 0) {
         pr_err("%s 0x8a i2c write failed (%d)\n", __func__, rc);
         return rc;
    }

    usleep(wait_time);
    return rc;
}

static int32_t lc898212_act_init_focus(struct msm_actuator_ctrl_t *a_ctrl,
    uint16_t size, enum msm_actuator_data_type type,
    struct reg_settings_t *settings)
{
    int32_t rc = 0;
    uint16_t data=0;
    uint16_t step=0;
    uint8_t bias = DEFAULT_BIAS;
    uint8_t offset = DEFAULT_OFFSET;
    
    uint16_t infinity = 0x0000;
    struct msm_camera_i2c_reg_setting lc898212_settings = {
        .addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
        .data_type = MSM_CAMERA_I2C_BYTE_DATA,
        .delay = 0,
        
    };

    if (a_ctrl->af_OTP_info.VCM_OTP_Read) {
        bias = a_ctrl->af_OTP_info.VCM_Bias;
        offset = a_ctrl->af_OTP_info.VCM_Offset;
        infinity = a_ctrl->af_OTP_info.VCM_Infinity;
    }

    lc898212_settings.reg_setting = lc898212_settings_1;
    lc898212_settings.size = ARRAY_SIZE(lc898212_settings_1);
    rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_write_table(&a_ctrl->i2c_client, &lc898212_settings);
    if (rc < 0) {
        pr_err("%s: lc898212_settings_1 i2c write failed (%d)\n", __func__, rc);
        return rc;
    }
    mdelay(1);
    CDBG("%s: lc898212_settings_1 i2c write done (%d)\n", __func__, rc);

    switch (a_ctrl->af_OTP_info.VCM_Vendor_Id_Version)
    {
        case 0x61:
            lc898212_settings.reg_setting = lc898212_settings_2_MTM;
            lc898212_settings.size = ARRAY_SIZE(lc898212_settings_2_MTM);
            break;
        case 0x71:
        case 0x72:
            lc898212_settings.reg_setting = lc898212_settings_2_TDK;
            lc898212_settings.size = ARRAY_SIZE(lc898212_settings_2_TDK);
            break;
        default:
            lc898212_settings.reg_setting = lc898212_settings_2_MTM;
            lc898212_settings.size = ARRAY_SIZE(lc898212_settings_2_MTM);
            break;
    }
    rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_write_table(&a_ctrl->i2c_client, &lc898212_settings);


    if (rc < 0) {
        pr_err("%s: lc898212_settings_2 i2c write failed (%d)\n", __func__, rc);
        return rc;
    }

    rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_write(&a_ctrl->i2c_client, 0x28, offset, MSM_CAMERA_I2C_BYTE_DATA); 
    if (rc < 0) {
        pr_err("%s 0x28 i2c write failed (%d)\n", __func__, rc);
        return rc;
    }
    rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_write(&a_ctrl->i2c_client, 0x29, bias, MSM_CAMERA_I2C_BYTE_DATA); 
    if (rc < 0) {
        pr_err("%s 0x29 i2c write failed (%d)\n", __func__, rc);
        return rc;
    }

    lc898212_settings.reg_setting = lc898212_settings_3;
    lc898212_settings.size = ARRAY_SIZE(lc898212_settings_3);
    rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_write_table(&a_ctrl->i2c_client, &lc898212_settings);
    if (rc < 0) {
        pr_err("%s: lc898212_settings_3 i2c write failed (%d)\n", __func__, rc);
        return rc;
    }
    mdelay(1);
    pr_err("%s: lc898212_settings_3 i2c write done (%d)\n", __func__, rc);

    
    rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_read(&a_ctrl->i2c_client, 0x3c, &data, MSM_CAMERA_I2C_WORD_DATA);
    if (rc < 0) {
        pr_err("%s: i2c read failed (%d)\n", __func__, rc);
        return rc;
    }
    else
    {
        pr_err("%s: read current position1 = 0x%x\n", __func__, data);
	}
    
    rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_write(&a_ctrl->i2c_client, 0x4, data, MSM_CAMERA_I2C_WORD_DATA);
    if (rc < 0) {
        pr_err("%s: 0x87 i2c write failed (%d)\n", __func__, rc);
        return rc;
    }

    rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_write(&a_ctrl->i2c_client, 0x18, data, MSM_CAMERA_I2C_WORD_DATA);
    if (rc < 0) {
        pr_err("%s: 0x18 i2c read failed (%d)\n", __func__, rc);
        return rc;
    }

    rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_write(&a_ctrl->i2c_client, 0x87, 0x85, MSM_CAMERA_I2C_BYTE_DATA);
    if (rc < 0) {
        pr_err("%s: 0x87 i2c write failed (%d)\n", __func__, rc);
        return rc;
    }

    lc898212_settings.reg_setting = lc898212_settings_4;
    lc898212_settings.size = ARRAY_SIZE(lc898212_settings_4);
    rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_write_table(&a_ctrl->i2c_client, &lc898212_settings);
    if (rc < 0) {
        pr_err("%s: lc898212_settings_4 i2c write failed (%d)\n", __func__, rc);
        return rc;
    }
    pr_err("%s: lc898212_settings_4 i2c write done (%d)\n", __func__, rc);

    
    switch (a_ctrl->af_OTP_info.VCM_Vendor_Id_Version)
    {
        case 0x61:
            rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_write(&a_ctrl->i2c_client, 0xA0, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
            break;
        case 0x71:
        case 0x72:
            rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_write(&a_ctrl->i2c_client, 0xA0, 0x02, MSM_CAMERA_I2C_BYTE_DATA);
            break;
        default:
            rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_write(&a_ctrl->i2c_client, 0xA0, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
            break;
    }
    if (rc < 0) {
        pr_err("%s: 0xA0 i2c write failed (%d)\n", __func__, rc);
        return rc;
    }

    
    rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_read(&a_ctrl->i2c_client, 0x3c, &data, MSM_CAMERA_I2C_WORD_DATA);
    if (rc < 0) {
        pr_err("%s: i2c read failed (%d)\n", __func__, rc);
        return rc;
    }
    else
    {
        pr_err("%s: read current position2 = 0x%x\n", __func__, data);
	}

    rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_write(&a_ctrl->i2c_client, 0x18, data, MSM_CAMERA_I2C_WORD_DATA);
    if (rc < 0) {
        pr_err("%s: 0x18 i2c read failed (%d)\n", __func__, rc);
        return rc;
    }

    rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_write(&a_ctrl->i2c_client, 0xa1, infinity, MSM_CAMERA_I2C_WORD_DATA);
    if (rc < 0) {
        pr_err("%s 0xa1 i2c write failed (%d)\n", __func__, rc);
        return rc;
    }

    switch (a_ctrl->af_OTP_info.VCM_Vendor_Id_Version)
    {
        case 0x61:
            step = (signed short)infinity > (signed short)data ? 0x0180 : 0xfe80;
            break;
        case 0x71:
        case 0x72:
            step = (signed short)infinity > (signed short)data ? 0xfe80 : 0x0180;
            break;
        default:
            step = (signed short)infinity > (signed short)data ? 0x0180 : 0xfe80;
            break;
    }
    pr_err("%s: step = 0x%x, infinity = 0x%x\n", __func__, step, infinity);
    rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_write(&a_ctrl->i2c_client, 0x16, step, MSM_CAMERA_I2C_WORD_DATA);
    if (rc < 0) {
        pr_err("%s 0x16 i2c write failed (%d)\n", __func__, rc);
        return rc;
    }

    rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_write(&a_ctrl->i2c_client, 0x8a, 0xd, MSM_CAMERA_I2C_BYTE_DATA);
    if (rc < 0) {
        pr_err("%s 0x8a i2c write failed (%d)\n", __func__, rc);
        return rc;
    }


    return rc;

}

static int32_t msm_actuator_set_af_value(struct msm_actuator_ctrl_t *a_ctrl, af_value_t af_value)
{
	#if 1 
	a_ctrl->af_OTP_info.VCM_OTP_Read = true;
	a_ctrl->af_OTP_info.VCM_Infinity = (af_value.AF_INF_MSB<<8) | af_value.AF_INF_LSB;
	a_ctrl->af_OTP_info.VCM_Bias = af_value.VCM_BIAS;
	a_ctrl->af_OTP_info.VCM_Offset = af_value.VCM_OFFSET;
	a_ctrl->af_OTP_info.VCM_Vendor_Id_Version = af_value.VCM_VENDOR_ID_VERSION;
	if(!strcmp(af_value.ACT_NAME, "lc898212"))
	{
		a_ctrl->act_i2c_select = WRITE_MULTI_TABLE;
	}
	else
	{
		a_ctrl->act_i2c_select = WRITE_TABLE_W_MICRODELAY;
	}

	pr_info("%s: VCM_Infinity = 0x%x (%d)\n",       __func__, a_ctrl->af_OTP_info.VCM_Infinity,     (int16_t)a_ctrl->af_OTP_info.VCM_Infinity);
	pr_info("%s: VCM_Bias = 0x%x (%d)\n",           __func__, a_ctrl->af_OTP_info.VCM_Bias,         (int16_t)a_ctrl->af_OTP_info.VCM_Bias);
	pr_info("%s: VCM_Offset = 0x%x (%d)\n",         __func__, a_ctrl->af_OTP_info.VCM_Offset,       (int16_t)a_ctrl->af_OTP_info.VCM_Offset);
	pr_info("%s: VCM_Vendor_Id_Version = 0x%x (%d)\n", __func__, a_ctrl->af_OTP_info.VCM_Vendor_Id_Version, a_ctrl->af_OTP_info.VCM_Vendor_Id_Version);

	return 0;
	#else
	uint8_t i;
	int32_t rc = 0;

	strlcpy(a_ctrl->af_OTP_info.act_name, af_value.ACT_NAME, sizeof(a_ctrl->af_OTP_info.act_name));

	for(i=0; i< (sizeof(act_func)/sizeof(act_func_t)); i++) {
		if (!strcmp(a_ctrl->af_OTP_info.act_name, act_func[i].act_name)) {
			rc = act_func[i].set_af_value(a_ctrl, af_value);
		}
	}
	return rc;
    #endif
}

void msm_actuator_dump_step_table(struct msm_actuator_ctrl_t *a_ctrl, int total_step)
{
	int i = 0;
	int remainder = total_step % 10;
	for (i = 0; i < total_step; i += 10) {
		if (a_ctrl->act_i2c_select == WRITE_MULTI_TABLE) {
			if (i + 10 < total_step) {
				pr_info("[%d]:%d(%d), [%d]:%d(%d), [%d]:%d(%d), [%d]:%d(%d), [%d]:%d(%d),[%d]:%d(%d)," \
					" [%d]:%d(%d), [%d]:%d(%d), [%d]:%d(%d), [%d]:%d(%d)\n",
					i, a_ctrl->step_position_table[i], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i]),
					i+1, a_ctrl->step_position_table[i+1], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i+1]),
					i+2, a_ctrl->step_position_table[i+2], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i+2]),
					i+3, a_ctrl->step_position_table[i+3], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i+3]),
					i+4, a_ctrl->step_position_table[i+4], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i+4]),
					i+5, a_ctrl->step_position_table[i+5], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i+5]),
					i+6, a_ctrl->step_position_table[i+6], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i+6]),
					i+7, a_ctrl->step_position_table[i+7], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i+7]),
					i+8, a_ctrl->step_position_table[i+8], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i+8]),
					i+9, a_ctrl->step_position_table[i+9], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i+9]));
			}
			else if (remainder == 9) {
				pr_info("[%d]:%d(%d), [%d]:%d(%d), [%d]:%d(%d), [%d]:%d(%d), [%d]:%d(%d),[%d]:%d(%d)," \
					" [%d]:%d(%d), [%d]:%d(%d), [%d]:%d(%d)\n",
					i, a_ctrl->step_position_table[i], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i]),
					i+1, a_ctrl->step_position_table[i+1], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i+1]),
					i+2, a_ctrl->step_position_table[i+2], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i+2]),
					i+3, a_ctrl->step_position_table[i+3], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i+3]),
					i+4, a_ctrl->step_position_table[i+4], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i+4]),
					i+5, a_ctrl->step_position_table[i+5], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i+5]),
					i+6, a_ctrl->step_position_table[i+6], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i+6]),
					i+7, a_ctrl->step_position_table[i+7], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i+7]),
					i+8, a_ctrl->step_position_table[i+8], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i+8]));
			}
			else if (remainder == 8) {
				pr_info("[%d]:%d(%d), [%d]:%d(%d), [%d]:%d(%d), [%d]:%d(%d), [%d]:%d(%d),[%d]:%d(%d)," \
					" [%d]:%d(%d), [%d]:%d(%d)\n",
					i, a_ctrl->step_position_table[i], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i]),
					i+1, a_ctrl->step_position_table[i+1], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i+1]),
					i+2, a_ctrl->step_position_table[i+2], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i+2]),
					i+3, a_ctrl->step_position_table[i+3], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i+3]),
					i+4, a_ctrl->step_position_table[i+4], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i+4]),
					i+5, a_ctrl->step_position_table[i+5], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i+5]),
					i+6, a_ctrl->step_position_table[i+6], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i+6]),
					i+7, a_ctrl->step_position_table[i+7], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i+7]));
			}
			else if (remainder == 7) {
				pr_info("[%d]:%d(%d), [%d]:%d(%d), [%d]:%d(%d), [%d]:%d(%d), [%d]:%d(%d),[%d]:%d(%d)," \
					" [%d]:%d(%d)\n",
					i, a_ctrl->step_position_table[i], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i]),
					i+1, a_ctrl->step_position_table[i+1], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i+1]),
					i+2, a_ctrl->step_position_table[i+2], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i+2]),
					i+3, a_ctrl->step_position_table[i+3], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i+3]),
					i+4, a_ctrl->step_position_table[i+4], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i+4]),
					i+5, a_ctrl->step_position_table[i+5], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i+5]),
					i+6, a_ctrl->step_position_table[i+6], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i+6]));
			}
			else if (remainder == 6) {
				pr_info("[%d]:%d(%d), [%d]:%d(%d), [%d]:%d(%d), [%d]:%d(%d), [%d]:%d(%d),[%d]:%d(%d)\n",
					i, a_ctrl->step_position_table[i], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i]),
					i+1, a_ctrl->step_position_table[i+1], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i+1]),
					i+2, a_ctrl->step_position_table[i+2], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i+2]),
					i+3, a_ctrl->step_position_table[i+3], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i+3]),
					i+4, a_ctrl->step_position_table[i+4], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i+4]),
					i+5, a_ctrl->step_position_table[i+5], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i+5]));
			}
			else if (remainder == 5) {
				pr_info("[%d]:%d(%d), [%d]:%d(%d), [%d]:%d(%d), [%d]:%d(%d), [%d]:%d(%d)\n",
					i, a_ctrl->step_position_table[i], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i]),
					i+1, a_ctrl->step_position_table[i+1], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i+1]),
					i+2, a_ctrl->step_position_table[i+2], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i+2]),
					i+3, a_ctrl->step_position_table[i+3], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i+3]),
					i+4, a_ctrl->step_position_table[i+4], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i+4]));
			}
			else if (remainder == 4) {
				pr_info("[%d]:%d(%d), [%d]:%d(%d), [%d]:%d(%d), [%d]:%d(%d)\n",
					i, a_ctrl->step_position_table[i], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i]),
					i+1, a_ctrl->step_position_table[i+1], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i+1]),
					i+2, a_ctrl->step_position_table[i+2], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i+2]),
					i+3, a_ctrl->step_position_table[i+3], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i+3]));
			}
			else if (remainder == 3) {
				pr_info("[%d]:%d(%d), [%d]:%d(%d), [%d]:%d(%d)\n",
					i, a_ctrl->step_position_table[i], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i]),
					i+1, a_ctrl->step_position_table[i+1], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i+1]),
					i+2, a_ctrl->step_position_table[i+2], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i+2]));
			}
			else if (remainder == 2) {
				pr_info("[%d]:%d(%d), [%d]:%d(%d)\n",
					i, a_ctrl->step_position_table[i], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i]),
					i+1, a_ctrl->step_position_table[i+1], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i+1]));
			}
			else if (remainder == 1) {
				pr_info("[%d]:%d(%d)\n", i, a_ctrl->step_position_table[i], af_actuator_lc898212_dec2hex(a_ctrl->step_position_table[i]));
			}
		}
		else {
			if (i + 10 < total_step) {
				pr_info("[%d]:%d, [%d]:%d, [%d]:%d, [%d]:%d, [%d]:%d,[%d]:%d," \
					" [%d]:%d, [%d]:%d, [%d]:%d, [%d]:%d\n",
					i, a_ctrl->step_position_table[i], i+1, a_ctrl->step_position_table[i+1],
					i+2, a_ctrl->step_position_table[i+2], i+3, a_ctrl->step_position_table[i+3],
					i+4, a_ctrl->step_position_table[i+4], i+5, a_ctrl->step_position_table[i+5],
					i+6, a_ctrl->step_position_table[i+6], i+7, a_ctrl->step_position_table[i+7],
					i+8, a_ctrl->step_position_table[i+8], i+9, a_ctrl->step_position_table[i+9]);
			}
			else if (remainder == 9) {
				pr_info("[%d]:%d, [%d]:%d, [%d]:%d, [%d]:%d, [%d]:%d,[%d]:%d," \
					" [%d]:%d, [%d]:%d, [%d]:%d\n",
					i, a_ctrl->step_position_table[i], i+1, a_ctrl->step_position_table[i+1],
					i+2, a_ctrl->step_position_table[i+2], i+3, a_ctrl->step_position_table[i+3],
					i+4, a_ctrl->step_position_table[i+4], i+5, a_ctrl->step_position_table[i+5],
					i+6, a_ctrl->step_position_table[i+6], i+7, a_ctrl->step_position_table[i+7],
					i+8, a_ctrl->step_position_table[i+8]);
			}
			else if (remainder == 8) {
				pr_info("[%d]:%d, [%d]:%d, [%d]:%d, [%d]:%d, [%d]:%d,[%d]:%d," \
					" [%d]:%d, [%d]:%d\n",
					i, a_ctrl->step_position_table[i], i+1, a_ctrl->step_position_table[i+1],
					i+2, a_ctrl->step_position_table[i+2], i+3, a_ctrl->step_position_table[i+3],
					i+4, a_ctrl->step_position_table[i+4], i+5, a_ctrl->step_position_table[i+5],
					i+6, a_ctrl->step_position_table[i+6], i+7, a_ctrl->step_position_table[i+7]);
			}
			else if (remainder == 7) {
				pr_info("[%d]:%d, [%d]:%d, [%d]:%d, [%d]:%d, [%d]:%d,[%d]:%d," \
					" [%d]:%d\n",
					i, a_ctrl->step_position_table[i], i+1, a_ctrl->step_position_table[i+1],
					i+2, a_ctrl->step_position_table[i+2], i+3, a_ctrl->step_position_table[i+3],
					i+4, a_ctrl->step_position_table[i+4], i+5, a_ctrl->step_position_table[i+5],
					i+6, a_ctrl->step_position_table[i+6]);
			}
			else if (remainder == 6) {
				pr_info("[%d]:%d, [%d]:%d, [%d]:%d, [%d]:%d, [%d]:%d,[%d]:%d\n",
					i, a_ctrl->step_position_table[i], i+1, a_ctrl->step_position_table[i+1],
					i+2, a_ctrl->step_position_table[i+2], i+3, a_ctrl->step_position_table[i+3],
					i+4, a_ctrl->step_position_table[i+4], i+5, a_ctrl->step_position_table[i+5]);
			}
			else if (remainder == 5) {
				pr_info("[%d]:%d, [%d]:%d, [%d]:%d, [%d]:%d, [%d]:%d\n",
					i, a_ctrl->step_position_table[i], i+1, a_ctrl->step_position_table[i+1],
					i+2, a_ctrl->step_position_table[i+2], i+3, a_ctrl->step_position_table[i+3],
					i+4, a_ctrl->step_position_table[i+4]);
			}
			else if (remainder == 4) {
				pr_info("[%d]:%d, [%d]:%d, [%d]:%d, [%d]:%d\n",
					i, a_ctrl->step_position_table[i], i+1, a_ctrl->step_position_table[i+1],
					i+2, a_ctrl->step_position_table[i+2], i+3, a_ctrl->step_position_table[i+3]);
			}
			else if (remainder == 3) {
				pr_info("[%d]:%d, [%d]:%d, [%d]:%d\n",
					i, a_ctrl->step_position_table[i], i+1, a_ctrl->step_position_table[i+1],
					i+2, a_ctrl->step_position_table[i+2]);
			}
			else if (remainder == 2) {
				pr_info("[%d]:%d, [%d]:%d\n",
					i, a_ctrl->step_position_table[i], i+1, a_ctrl->step_position_table[i+1]);
			}
			else if (remainder == 1) {
				pr_info("[%d]:%d\n", i, a_ctrl->step_position_table[i]);
			}
		}
	}
}

static int32_t msm_actuator_piezo_set_default_focus(
	struct msm_actuator_ctrl_t *a_ctrl,
	struct msm_actuator_move_params_t *move_params)
{
	int32_t rc = 0;
	struct msm_camera_i2c_reg_setting reg_setting;
	CDBG("Enter\n");

	if (a_ctrl->curr_step_pos != 0) {
		a_ctrl->i2c_tbl_index = 0;
		a_ctrl->func_tbl->actuator_parse_i2c_params(a_ctrl,
			a_ctrl->initial_code, 0, 0);
		a_ctrl->func_tbl->actuator_parse_i2c_params(a_ctrl,
			a_ctrl->initial_code, 0, 0);
		reg_setting.reg_setting = a_ctrl->i2c_reg_tbl;
		reg_setting.data_type = a_ctrl->i2c_data_type;
		reg_setting.size = a_ctrl->i2c_tbl_index;
		rc = a_ctrl->i2c_client.i2c_func_tbl->
			i2c_write_table_w_microdelay(
			&a_ctrl->i2c_client, &reg_setting);
		if (rc < 0) {
			pr_err("%s: i2c write error:%d\n",
				__func__, rc);
			return rc;
		}
		a_ctrl->i2c_tbl_index = 0;
		a_ctrl->curr_step_pos = 0;
	}
	CDBG("Exit\n");
	return rc;
}

static void msm_actuator_parse_i2c_params(struct msm_actuator_ctrl_t *a_ctrl,
	int16_t next_lens_position, uint32_t hw_params, uint16_t delay)
{
	struct msm_actuator_reg_params_t *write_arr = a_ctrl->reg_tbl;
	uint32_t hw_dword = hw_params;
	uint16_t i2c_byte1 = 0, i2c_byte2 = 0;
	uint16_t value = 0;
	uint32_t size = a_ctrl->reg_tbl_size, i = 0;
	struct msm_camera_i2c_reg_array *i2c_tbl = a_ctrl->i2c_reg_tbl;
	CDBG("Enter\n");
	for (i = 0; i < size; i++) {
		if ((a_ctrl->total_steps + 1) < (a_ctrl->i2c_tbl_index)) {
			break;
		}
		if (write_arr[i].reg_write_type == MSM_ACTUATOR_WRITE_DAC) {
			value = (next_lens_position <<
				write_arr[i].data_shift) |
				((hw_dword & write_arr[i].hw_mask) >>
				write_arr[i].hw_shift);

			if (write_arr[i].reg_addr != 0xFFFF) {
				i2c_byte1 = write_arr[i].reg_addr;
				i2c_byte2 = value;
				if (size != (i+1)) {
					i2c_byte2 = value & 0xFF;
					CDBG("byte1:0x%x, byte2:0x%x\n",
						i2c_byte1, i2c_byte2);
					i2c_tbl[a_ctrl->i2c_tbl_index].
						reg_addr = i2c_byte1;
					i2c_tbl[a_ctrl->i2c_tbl_index].
						reg_data = i2c_byte2;
					i2c_tbl[a_ctrl->i2c_tbl_index].
						delay = 0;
					a_ctrl->i2c_tbl_index++;
					i++;
					i2c_byte1 = write_arr[i].reg_addr;
					i2c_byte2 = (value & 0xFF00) >> 8;
				}
			} else {
				i2c_byte1 = (value & 0xFF00) >> 8;
				i2c_byte2 = value & 0xFF;
			}
		} else {
			i2c_byte1 = write_arr[i].reg_addr;
			i2c_byte2 = (hw_dword & write_arr[i].hw_mask) >>
				write_arr[i].hw_shift;
		}
		CDBG("i2c_byte1:0x%x, i2c_byte2:0x%x\n", i2c_byte1, i2c_byte2);
		i2c_tbl[a_ctrl->i2c_tbl_index].reg_addr = i2c_byte1;
		i2c_tbl[a_ctrl->i2c_tbl_index].reg_data = i2c_byte2;
		i2c_tbl[a_ctrl->i2c_tbl_index].delay = delay;
		a_ctrl->i2c_tbl_index++;
	}
	CDBG("Exit\n");
}

static int32_t msm_actuator_init_focus(struct msm_actuator_ctrl_t *a_ctrl,
	uint16_t size, struct reg_settings_t *settings)
{
	int32_t rc = -EFAULT;
	int32_t i = 0;
	enum msm_camera_i2c_reg_addr_type save_addr_type;
	CDBG("Enter\n");

	save_addr_type = a_ctrl->i2c_client.addr_type;
	for (i = 0; i < size; i++) {

		switch (settings[i].addr_type) {
		case MSM_ACTUATOR_BYTE_ADDR:
			a_ctrl->i2c_client.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
			break;
		case MSM_ACTUATOR_WORD_ADDR:
			a_ctrl->i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
			break;
		default:
			pr_err("Unsupport addr type: %d\n",
				settings[i].addr_type);
			break;
		}

		switch (settings[i].i2c_operation) {
		case MSM_ACT_WRITE:
			rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_write(
				&a_ctrl->i2c_client,
				settings[i].reg_addr,
				settings[i].reg_data,
				settings[i].data_type);
			break;
		case MSM_ACT_POLL:
			rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_poll(
				&a_ctrl->i2c_client,
				settings[i].reg_addr,
				settings[i].reg_data,
				settings[i].data_type);
			break;
		default:
			pr_err("Unsupport i2c_operation: %d\n",
				settings[i].i2c_operation);
			break;

		if (0 != settings[i].delay)
			msleep(settings[i].delay);

		if (rc < 0)
			break;
		}
	}

	a_ctrl->curr_step_pos = 0;
	/*
	 * Recover register addr_type after the init
	 * settings are written.
	 */
	a_ctrl->i2c_client.addr_type = save_addr_type;
	CDBG("Exit\n");
	return rc;
}

static void msm_actuator_write_focus(
	struct msm_actuator_ctrl_t *a_ctrl,
	uint16_t curr_lens_pos,
	struct damping_params_t *damping_params,
	int8_t sign_direction,
	int16_t code_boundary)
{
	int16_t next_lens_pos = 0;
	uint16_t damping_code_step = 0;
	uint16_t wait_time = 0;
	CDBG("Enter\n");

	damping_code_step = damping_params->damping_step;
	wait_time = damping_params->damping_delay;
	CDBG("damping_code_step:%d wait_time:%d code_boundary:%d curr_lens_pos:%d sign_direction:%d\n",
		damping_code_step, wait_time, code_boundary, curr_lens_pos, sign_direction);

	
	for (next_lens_pos =
		curr_lens_pos + (sign_direction * damping_code_step);
		(sign_direction * next_lens_pos) <=
			(sign_direction * code_boundary);
		next_lens_pos =
			(next_lens_pos +
				(sign_direction * damping_code_step))) {
		
		if(a_ctrl->act_i2c_select == WRITE_MULTI_TABLE)
			lc898212_wrapper_i2c_write(a_ctrl, next_lens_pos, sign_direction, 0);
		else
		
			a_ctrl->func_tbl->actuator_parse_i2c_params(a_ctrl,
				next_lens_pos, damping_params->hw_params, wait_time);
		curr_lens_pos = next_lens_pos;
	}
	if (curr_lens_pos != code_boundary) {
            
            if(a_ctrl->act_i2c_select == WRITE_MULTI_TABLE)
                lc898212_wrapper_i2c_write(a_ctrl, code_boundary, sign_direction, wait_time);
            else
            
		a_ctrl->func_tbl->actuator_parse_i2c_params(a_ctrl,
			code_boundary, damping_params->hw_params, wait_time);
	}
	CDBG("Exit\n");
}

static int32_t msm_actuator_piezo_move_focus(
	struct msm_actuator_ctrl_t *a_ctrl,
	struct msm_actuator_move_params_t *move_params)
{
	int32_t dest_step_position = move_params->dest_step_pos;
	struct damping_params_t ringing_params_kernel;
	int32_t rc = 0;
	int32_t num_steps = move_params->num_steps;
	struct msm_camera_i2c_reg_setting reg_setting;
	CDBG("Enter\n");

	if (copy_from_user(&ringing_params_kernel,
		&(move_params->ringing_params[0]),
		sizeof(struct damping_params_t))) {
		pr_err("copy_from_user failed\n");
		return -EFAULT;
	}

	if (num_steps <= 0 || num_steps > MAX_NUMBER_OF_STEPS) {
		pr_err("num_steps out of range = %d\n",
			num_steps);
		return -EFAULT;
	}

	a_ctrl->i2c_tbl_index = 0;
	a_ctrl->func_tbl->actuator_parse_i2c_params(a_ctrl,
		(num_steps *
		a_ctrl->region_params[0].code_per_step),
		ringing_params_kernel.hw_params, 0);

	reg_setting.reg_setting = a_ctrl->i2c_reg_tbl;
	reg_setting.data_type = a_ctrl->i2c_data_type;
	reg_setting.size = a_ctrl->i2c_tbl_index;
	rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_write_table_w_microdelay(
		&a_ctrl->i2c_client, &reg_setting);
	if (rc < 0) {
		pr_err("i2c write error:%d\n", rc);
		return rc;
	}
	a_ctrl->i2c_tbl_index = 0;
	a_ctrl->curr_step_pos = dest_step_position;
	CDBG("Exit\n");
	return rc;
}

static int32_t msm_actuator_move_focus(
	struct msm_actuator_ctrl_t *a_ctrl,
	struct msm_actuator_move_params_t *move_params)
{
	int32_t rc = 0;
	struct damping_params_t ringing_params_kernel;
	int8_t sign_dir = move_params->sign_dir;
	uint16_t step_boundary = 0;
	uint16_t target_step_pos = 0;
	uint16_t target_lens_pos = 0;
	int16_t dest_step_pos = move_params->dest_step_pos;
	uint16_t curr_lens_pos = 0;
	int dir = move_params->dir;
	int32_t num_steps = move_params->num_steps;
	struct msm_camera_i2c_reg_setting reg_setting;

	if (copy_from_user(&ringing_params_kernel,
		&(move_params->ringing_params[a_ctrl->curr_region_index]),
		sizeof(struct damping_params_t))) {
		pr_err("copy_from_user failed\n");
		return -EFAULT;
	}


	CDBG("called, dir %d, num_steps %d\n", dir, num_steps);

	if (dest_step_pos == a_ctrl->curr_step_pos)
		return rc;

	if ((sign_dir > MSM_ACTUATOR_MOVE_SIGNED_NEAR) ||
		(sign_dir < MSM_ACTUATOR_MOVE_SIGNED_FAR)) {
		pr_err("Invalid sign_dir = %d\n", sign_dir);
		return -EFAULT;
	}
	if ((dir > MOVE_FAR) || (dir < MOVE_NEAR)) {
		pr_err("Invalid direction = %d\n", dir);
		return -EFAULT;
	}
	if (dest_step_pos > a_ctrl->total_steps) {
		pr_err("Step pos greater than total steps = %d\n",
		dest_step_pos);
		return -EFAULT;
	}
	curr_lens_pos = a_ctrl->step_position_table[a_ctrl->curr_step_pos];
	a_ctrl->i2c_tbl_index = 0;
	CDBG("curr_step_pos =%d dest_step_pos =%d curr_lens_pos=%d\n",
		a_ctrl->curr_step_pos, dest_step_pos, curr_lens_pos);

	while (a_ctrl->curr_step_pos != dest_step_pos) {
		step_boundary =
			a_ctrl->region_params[a_ctrl->curr_region_index].
			step_bound[dir];
		if ((dest_step_pos * sign_dir) <=
			(step_boundary * sign_dir)) {

			target_step_pos = dest_step_pos;
			target_lens_pos =
				a_ctrl->step_position_table[target_step_pos];
			a_ctrl->func_tbl->actuator_write_focus(a_ctrl,
					curr_lens_pos,
					&ringing_params_kernel,
					sign_dir,
					target_lens_pos);
			curr_lens_pos = target_lens_pos;

		} else {
			target_step_pos = step_boundary;
			target_lens_pos =
				a_ctrl->step_position_table[target_step_pos];
			a_ctrl->func_tbl->actuator_write_focus(a_ctrl,
					curr_lens_pos,
					&ringing_params_kernel,
					sign_dir,
					target_lens_pos);
			curr_lens_pos = target_lens_pos;

			a_ctrl->curr_region_index += sign_dir;
		}
		a_ctrl->curr_step_pos = target_step_pos;
	}

	if(a_ctrl->act_i2c_select == WRITE_MULTI_TABLE)
	{}
	else
	{
	move_params->curr_lens_pos = curr_lens_pos;
	reg_setting.reg_setting = a_ctrl->i2c_reg_tbl;
	reg_setting.data_type = a_ctrl->i2c_data_type;
	reg_setting.size = a_ctrl->i2c_tbl_index;
	rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_write_table_w_microdelay(
		&a_ctrl->i2c_client, &reg_setting);
	}
	if (rc < 0) {
		pr_err("i2c write error:%d\n", rc);
		return rc;
	}
	a_ctrl->i2c_tbl_index = 0;
	CDBG("Exit\n");

	return rc;
}


static int32_t msm_actuator_park_lens(struct msm_actuator_ctrl_t *a_ctrl)
{
	int32_t rc = 0;
	uint16_t next_lens_pos = 0;
	struct msm_camera_i2c_reg_setting reg_setting;

	a_ctrl->i2c_tbl_index = 0;
	if ((a_ctrl->curr_step_pos > a_ctrl->total_steps) ||
		(!a_ctrl->park_lens.max_step) ||
		(!a_ctrl->step_position_table) ||
		(!a_ctrl->i2c_reg_tbl) ||
		(!a_ctrl->func_tbl) ||
		(!a_ctrl->func_tbl->actuator_parse_i2c_params)) {
		pr_err("%s:%d Failed to park lens.\n",
			__func__, __LINE__);
		return -EFAULT;
	}

	if (a_ctrl->park_lens.max_step > a_ctrl->max_code_size)
		a_ctrl->park_lens.max_step = a_ctrl->max_code_size;

	next_lens_pos = a_ctrl->step_position_table[a_ctrl->curr_step_pos];
	while (next_lens_pos) {
		
		if (next_lens_pos > (a_ctrl->park_lens.max_step *
			PARK_LENS_LONG_STEP)) {
			next_lens_pos = next_lens_pos -
				(a_ctrl->park_lens.max_step *
				PARK_LENS_LONG_STEP);
		} else if (next_lens_pos > (a_ctrl->park_lens.max_step *
			PARK_LENS_MID_STEP)) {
			next_lens_pos = next_lens_pos -
				(a_ctrl->park_lens.max_step *
				PARK_LENS_MID_STEP);
		} else if (next_lens_pos > (a_ctrl->park_lens.max_step *
			PARK_LENS_SMALL_STEP)) {
			next_lens_pos = next_lens_pos -
				(a_ctrl->park_lens.max_step *
				PARK_LENS_SMALL_STEP);
		} else {
			next_lens_pos = (next_lens_pos >
				a_ctrl->park_lens.max_step) ?
				(next_lens_pos - a_ctrl->park_lens.
				max_step) : 0;
		}
		a_ctrl->func_tbl->actuator_parse_i2c_params(a_ctrl,
			next_lens_pos, a_ctrl->park_lens.hw_params,
			a_ctrl->park_lens.damping_delay);

		reg_setting.reg_setting = a_ctrl->i2c_reg_tbl;
		reg_setting.size = a_ctrl->i2c_tbl_index;
		reg_setting.data_type = a_ctrl->i2c_data_type;

		rc = a_ctrl->i2c_client.i2c_func_tbl->
			i2c_write_table_w_microdelay(
			&a_ctrl->i2c_client, &reg_setting);
		if (rc < 0) {
			pr_err("%s Failed I2C write Line %d\n",
				__func__, __LINE__);
			return rc;
		}
		a_ctrl->i2c_tbl_index = 0;
		
		usleep_range(10000, 12000);
	}

	return 0;
}

static int32_t msm_actuator_init_step_table(struct msm_actuator_ctrl_t *a_ctrl,
	struct msm_actuator_set_info_t *set_info)
{
	int16_t code_per_step = 0;
	int16_t cur_code = 0;
	int16_t step_index = 0, region_index = 0;
	uint16_t step_boundary = 0;
	uint32_t max_code_size = 1;
	uint16_t data_size = set_info->actuator_params.data_size;
	CDBG("Enter\n");

	for (; data_size > 0; data_size--)
		max_code_size *= 2;

	a_ctrl->max_code_size = max_code_size;
	kfree(a_ctrl->step_position_table);
	a_ctrl->step_position_table = NULL;

	if (set_info->af_tuning_params.total_steps
		>  MAX_ACTUATOR_AF_TOTAL_STEPS) {
		pr_err("Max actuator totalsteps exceeded = %d\n",
		set_info->af_tuning_params.total_steps);
		return -EFAULT;
	}
	
	a_ctrl->step_position_table =
		kmalloc(sizeof(uint16_t) *
		(set_info->af_tuning_params.total_steps + 1), GFP_KERNEL);

	if (a_ctrl->step_position_table == NULL)
		return -ENOMEM;

	cur_code = set_info->af_tuning_params.initial_code;
	a_ctrl->step_position_table[step_index++] = cur_code;
	for (region_index = 0;
		region_index < a_ctrl->region_size;
		region_index++) {
		code_per_step =
			a_ctrl->region_params[region_index].code_per_step;
		step_boundary =
			a_ctrl->region_params[region_index].
			step_bound[MOVE_NEAR];
		for (; step_index <= step_boundary;
			step_index++) {

			cur_code += code_per_step;
			if (cur_code < max_code_size)
				a_ctrl->step_position_table[step_index] =
					cur_code;
			else {
				for (; step_index <
					set_info->af_tuning_params.total_steps;
					step_index++)
					a_ctrl->
						step_position_table[
						step_index] =
						max_code_size;
			}

		}
	}

	
	
	if (a_ctrl->enable_focus_step_log)
		msm_actuator_dump_step_table(a_ctrl, set_info->af_tuning_params.total_steps);
	

	CDBG("Exit\n");
	return 0;
}

static int32_t msm_actuator_set_default_focus(
	struct msm_actuator_ctrl_t *a_ctrl,
	struct msm_actuator_move_params_t *move_params)
{
	int32_t rc = 0;
	CDBG("Enter\n");

	if (a_ctrl->curr_step_pos != 0)
		rc = a_ctrl->func_tbl->actuator_move_focus(a_ctrl, move_params);
	CDBG("Exit\n");
	return rc;
}

static int32_t msm_actuator_vreg_control(struct msm_actuator_ctrl_t *a_ctrl,
							int config)
{
	int rc = 0, i, cnt;
	struct msm_actuator_vreg *vreg_cfg;

	vreg_cfg = &a_ctrl->vreg_cfg;
	cnt = vreg_cfg->num_vreg;
	if (!cnt)
		return 0;

	if (cnt >= MSM_ACTUATOT_MAX_VREGS) {
		pr_err("%s failed %d cnt %d\n", __func__, __LINE__, cnt);
		return -EINVAL;
	}

	for (i = 0; i < cnt; i++) {
		if (a_ctrl->act_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
			rc = msm_camera_config_single_vreg(&(a_ctrl->pdev->dev),
				&vreg_cfg->cam_vreg[i],
				(struct regulator **)&vreg_cfg->data[i],
				config);
		} else if (a_ctrl->act_device_type ==
			MSM_CAMERA_I2C_DEVICE) {
			rc = msm_camera_config_single_vreg(
				&(a_ctrl->i2c_client.client->dev),
				&vreg_cfg->cam_vreg[i],
				(struct regulator **)&vreg_cfg->data[i],
				config);
		}
	}
	return rc;
}

static int32_t msm_actuator_power_down(struct msm_actuator_ctrl_t *a_ctrl)
{
	int32_t rc = 0;
	CDBG("Enter\n");
	if (a_ctrl->actuator_state != ACTUATOR_POWER_DOWN) {

		if (a_ctrl->func_tbl && a_ctrl->func_tbl->actuator_park_lens) {
			rc = a_ctrl->func_tbl->actuator_park_lens(a_ctrl);
			if (rc < 0)
				pr_err("%s:%d Lens park failed.\n",
					__func__, __LINE__);
		}

		rc = msm_actuator_vreg_control(a_ctrl, 0);
		if (rc < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			return rc;
		}

		kfree(a_ctrl->step_position_table);
		a_ctrl->step_position_table = NULL;
		kfree(a_ctrl->i2c_reg_tbl);
		a_ctrl->i2c_reg_tbl = NULL;
		a_ctrl->i2c_tbl_index = 0;
		a_ctrl->actuator_state = ACTUATOR_POWER_DOWN;
	}
	CDBG("Exit\n");
	return rc;
}

static int32_t msm_actuator_set_position(
	struct msm_actuator_ctrl_t *a_ctrl,
	struct msm_actuator_set_position_t *set_pos)
{
	int32_t rc = 0;
	int32_t index;
	uint16_t next_lens_position;
	uint16_t delay;
	uint32_t hw_params = 0;
	struct msm_camera_i2c_reg_setting reg_setting;
	CDBG("%s Enter %d\n", __func__, __LINE__);
	if (set_pos->number_of_steps <= 0 ||
		set_pos->number_of_steps > MAX_NUMBER_OF_STEPS) {
		pr_err("num_steps out of range = %d\n",
			set_pos->number_of_steps);
		return -EFAULT;
	}

	
	if (a_ctrl->act_i2c_select == WRITE_MULTI_TABLE) {
		for (index = 0; index < set_pos->number_of_steps; index++) {
			uint16_t cur_position = 0;
			int8_t sign_dir = 0;
			int16_t hex_next_lens_pos = 0;

			next_lens_position = set_pos->pos[index];
			hex_next_lens_pos = af_actuator_lc898212_dec2hex(next_lens_position);
			
			rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_read(&a_ctrl->i2c_client, 0x3c, &cur_position, MSM_CAMERA_I2C_WORD_DATA);
			if (rc < 0) {
				pr_err("%s: i2c read failed (%d)\n", __func__, rc);
				return rc;
			}
			sign_dir = (signed short)hex_next_lens_pos > (signed short)cur_position ? 1 : -1;
			pr_info("%s: index%d cur position:0x%x dir = %d, hex_next_lens_pos = 0x%x, delay:%d\n", __func__, index,
				cur_position, sign_dir, (uint16_t)hex_next_lens_pos, set_pos->delay[index]);

			lc898212_wrapper_i2c_write(a_ctrl, next_lens_position, sign_dir, set_pos->delay[index]);
		}
	}
	
	else {
		a_ctrl->i2c_tbl_index = 0;
		for (index = 0; index < set_pos->number_of_steps; index++) {
			next_lens_position = set_pos->pos[index];
			delay = set_pos->delay[index];
			a_ctrl->func_tbl->actuator_parse_i2c_params(a_ctrl,
			next_lens_position, hw_params, delay);

			reg_setting.reg_setting = a_ctrl->i2c_reg_tbl;
			reg_setting.size = a_ctrl->i2c_tbl_index;
			reg_setting.data_type = a_ctrl->i2c_data_type;

			rc = a_ctrl->i2c_client.i2c_func_tbl->
				i2c_write_table_w_microdelay(
				&a_ctrl->i2c_client, &reg_setting);
			if (rc < 0) {
				pr_err("%s Failed I2C write Line %d\n",
					__func__, __LINE__);
				return rc;
			}
			a_ctrl->i2c_tbl_index = 0;
		}
	}
	CDBG("%s exit %d\n", __func__, __LINE__);
	return rc;
}

static int32_t msm_actuator_set_param(struct msm_actuator_ctrl_t *a_ctrl,
	struct msm_actuator_set_info_t *set_info) {
	struct reg_settings_t *init_settings = NULL;
	int32_t rc = -EFAULT;
	uint16_t i = 0;
	struct msm_camera_cci_client *cci_client = NULL;
	CDBG("Enter\n");

	for (i = 0; i < ARRAY_SIZE(actuators); i++) {
		if (set_info->actuator_params.act_type ==
			actuators[i]->act_type) {
			a_ctrl->func_tbl = &actuators[i]->func_tbl;
			rc = 0;
		}
	}

	if (rc < 0) {
		pr_err("Actuator function table not found\n");
		return rc;
	}
	if (set_info->af_tuning_params.total_steps
		>  MAX_ACTUATOR_AF_TOTAL_STEPS) {
		pr_err("Max actuator totalsteps exceeded = %d\n",
		set_info->af_tuning_params.total_steps);
		return -EFAULT;
	}
	if (set_info->af_tuning_params.region_size
		> MAX_ACTUATOR_REGION) {
		pr_err("MAX_ACTUATOR_REGION is exceeded.\n");
		return -EFAULT;
	}

	a_ctrl->region_size = set_info->af_tuning_params.region_size;
	a_ctrl->pwd_step = set_info->af_tuning_params.pwd_step;
	a_ctrl->total_steps = set_info->af_tuning_params.total_steps;

	if (copy_from_user(&a_ctrl->region_params,
		(void *)set_info->af_tuning_params.region_params,
		a_ctrl->region_size * sizeof(struct region_params_t)))
		return -EFAULT;

	if (a_ctrl->act_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		cci_client = a_ctrl->i2c_client.cci_client;
		cci_client->sid =
			set_info->actuator_params.i2c_addr >> 1;
		cci_client->retries = 3;
		cci_client->id_map = 0;
		cci_client->cci_i2c_master = a_ctrl->cci_master;
	} else {
		a_ctrl->i2c_client.client->addr =
			set_info->actuator_params.i2c_addr;
	}

	a_ctrl->i2c_data_type = set_info->actuator_params.i2c_data_type;
	a_ctrl->i2c_client.addr_type = set_info->actuator_params.i2c_addr_type;
	if (set_info->actuator_params.reg_tbl_size <=
		MAX_ACTUATOR_REG_TBL_SIZE) {
		a_ctrl->reg_tbl_size = set_info->actuator_params.reg_tbl_size;
	} else {
		a_ctrl->reg_tbl_size = 0;
		pr_err("MAX_ACTUATOR_REG_TBL_SIZE is exceeded.\n");
		return -EFAULT;
	}

	kfree(a_ctrl->i2c_reg_tbl);
	a_ctrl->i2c_reg_tbl = NULL;
	a_ctrl->i2c_reg_tbl =
		kmalloc(sizeof(struct msm_camera_i2c_reg_array) *
		(set_info->af_tuning_params.total_steps + 1), GFP_KERNEL);
	if (!a_ctrl->i2c_reg_tbl) {
		pr_err("kmalloc fail\n");
		return -ENOMEM;
	}

	if (copy_from_user(&a_ctrl->reg_tbl,
		(void *)set_info->actuator_params.reg_tbl_params,
		a_ctrl->reg_tbl_size *
		sizeof(struct msm_actuator_reg_params_t))) {
		kfree(a_ctrl->i2c_reg_tbl);
		a_ctrl->i2c_reg_tbl = NULL;
		return -EFAULT;
	}

	
	a_ctrl->enable_focus_step_log = set_info->enable_focus_step_log;
	if(a_ctrl->act_i2c_select == WRITE_MULTI_TABLE)
	{
        rc = lc898212_act_init_focus(a_ctrl,
				set_info->actuator_params.init_setting_size,
				a_ctrl->i2c_data_type,
				init_settings);
	}
	else
	{
	
	if (set_info->actuator_params.init_setting_size &&
		set_info->actuator_params.init_setting_size
		<= MAX_ACTUATOR_INIT_SET) {
		if (a_ctrl->func_tbl->actuator_init_focus) {
			init_settings = kmalloc(sizeof(struct reg_settings_t) *
				(set_info->actuator_params.init_setting_size),
				GFP_KERNEL);
			if (init_settings == NULL) {
				kfree(a_ctrl->i2c_reg_tbl);
				a_ctrl->i2c_reg_tbl = NULL;
				pr_err("Error allocating memory for init_settings\n");
				return -EFAULT;
			}
			if (copy_from_user(init_settings,
				(void *)set_info->actuator_params.init_settings,
				set_info->actuator_params.init_setting_size *
				sizeof(struct reg_settings_t))) {
				kfree(init_settings);
				kfree(a_ctrl->i2c_reg_tbl);
				a_ctrl->i2c_reg_tbl = NULL;
				pr_err("Error copying init_settings\n");
				return -EFAULT;
			}
			rc = a_ctrl->func_tbl->actuator_init_focus(a_ctrl,
				set_info->actuator_params.init_setting_size,
				init_settings);
			kfree(init_settings);
			if (rc < 0) {
				kfree(a_ctrl->i2c_reg_tbl);
				a_ctrl->i2c_reg_tbl = NULL;
				pr_err("Error actuator_init_focus\n");
				return -EFAULT;
			}
		}
	}
	
	}
	

	
	a_ctrl->park_lens = set_info->actuator_params.park_lens;
	a_ctrl->initial_code = set_info->af_tuning_params.initial_code;
	if (a_ctrl->func_tbl->actuator_init_step_table)
		rc = a_ctrl->func_tbl->
			actuator_init_step_table(a_ctrl, set_info);

	a_ctrl->curr_step_pos = 0;
	a_ctrl->curr_region_index = 0;
	CDBG("Exit\n");

	return rc;
}

static int msm_actuator_init(struct msm_actuator_ctrl_t *a_ctrl)
{
	int rc = 0;
	CDBG("Enter\n");
	if (!a_ctrl) {
		pr_err("failed\n");
		return -EINVAL;
	}
	if (a_ctrl->act_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_util(
			&a_ctrl->i2c_client, MSM_CCI_INIT);
		if (rc < 0)
			pr_err("cci_init failed\n");
	}
	CDBG("Exit\n");
	return rc;
}

static int32_t msm_actuator_config(struct msm_actuator_ctrl_t *a_ctrl,
	void __user *argp)
{
	struct msm_actuator_cfg_data *cdata =
		(struct msm_actuator_cfg_data *)argp;
	int32_t rc = 0;
	mutex_lock(a_ctrl->actuator_mutex);
	CDBG("Enter\n");
	CDBG("%s type %d\n", __func__, cdata->cfgtype);
	switch (cdata->cfgtype) {
	case CFG_SET_ACTUATOR_AF_VALUE:
		rc = msm_actuator_set_af_value(a_ctrl, (af_value_t)cdata->cfg.af_value);
		if (rc < 0) {
			pr_err("%s set af value failed %d\n", __func__, rc);
		}
		break;
	case CFG_ACTUATOR_INIT:
		rc = msm_actuator_init(a_ctrl);
		if (rc < 0)
			pr_err("msm_actuator_init failed %d\n", rc);
		break;
	case CFG_GET_ACTUATOR_INFO:
		cdata->is_af_supported = 1;
		cdata->cfg.cam_name = a_ctrl->cam_name;
		break;

	case CFG_SET_ACTUATOR_INFO:
		rc = msm_actuator_set_param(a_ctrl, &cdata->cfg.set_info);
		if (rc < 0)
			pr_err("init table failed %d\n", rc);
		break;

	case CFG_SET_DEFAULT_FOCUS:
		rc = a_ctrl->func_tbl->actuator_set_default_focus(a_ctrl,
			&cdata->cfg.move);
		if (rc < 0)
			pr_err("move focus failed %d\n", rc);
		break;

	case CFG_MOVE_FOCUS:
		rc = a_ctrl->func_tbl->actuator_move_focus(a_ctrl,
			&cdata->cfg.move);
		if (rc < 0)
			pr_err("move focus failed %d\n", rc);
		break;
	case CFG_ACTUATOR_POWERDOWN:
		rc = msm_actuator_power_down(a_ctrl);
		if (rc < 0)
			pr_err("msm_actuator_power_down failed %d\n", rc);
		break;

	case CFG_SET_POSITION:
		rc = a_ctrl->func_tbl->actuator_set_position(a_ctrl,
			&cdata->cfg.setpos);
		if (rc < 0)
			pr_err("actuator_set_position failed %d\n", rc);
		break;

	case CFG_ACTUATOR_POWERUP:
		rc = msm_actuator_power_up(a_ctrl);
		if (rc < 0)
			pr_err("Failed actuator power up%d\n", rc);
		break;

	default:
		break;
	}
	mutex_unlock(a_ctrl->actuator_mutex);
	CDBG("Exit\n");
	return rc;
}

static int32_t msm_actuator_get_subdev_id(struct msm_actuator_ctrl_t *a_ctrl,
	void *arg)
{
	uint32_t *subdev_id = (uint32_t *)arg;
	CDBG("Enter\n");
	if (!subdev_id) {
		pr_err("failed\n");
		return -EINVAL;
	}
	if (a_ctrl->act_device_type == MSM_CAMERA_PLATFORM_DEVICE)
		*subdev_id = a_ctrl->pdev->id;
	else
		*subdev_id = a_ctrl->subdev_id;

	CDBG("subdev_id %d\n", *subdev_id);
	CDBG("Exit\n");
	return 0;
}

static struct msm_camera_i2c_fn_t msm_sensor_cci_func_tbl = {
	.i2c_read = msm_camera_cci_i2c_read,
	.i2c_read_seq = msm_camera_cci_i2c_read_seq,
	.i2c_write = msm_camera_cci_i2c_write,
	.i2c_write_table = msm_camera_cci_i2c_write_table,
	.i2c_write_seq_table = msm_camera_cci_i2c_write_seq_table,
	.i2c_write_table_w_microdelay =
		msm_camera_cci_i2c_write_table_w_microdelay,
	.i2c_util = msm_sensor_cci_i2c_util,
	.i2c_poll =  msm_camera_cci_i2c_poll,
};

static struct msm_camera_i2c_fn_t msm_sensor_qup_func_tbl = {
	.i2c_read = msm_camera_qup_i2c_read,
	.i2c_read_seq = msm_camera_qup_i2c_read_seq,
	.i2c_write = msm_camera_qup_i2c_write,
	.i2c_write_table = msm_camera_qup_i2c_write_table,
	.i2c_write_seq_table = msm_camera_qup_i2c_write_seq_table,
	.i2c_write_table_w_microdelay =
		msm_camera_qup_i2c_write_table_w_microdelay,
	.i2c_poll = msm_camera_qup_i2c_poll,
};

static int msm_actuator_close(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh) {
	int rc = 0;
	struct msm_actuator_ctrl_t *a_ctrl =  v4l2_get_subdevdata(sd);
	CDBG("Enter\n");
	if (!a_ctrl || !a_ctrl->i2c_client.i2c_func_tbl) {
		
		pr_err("failed\n");
		return -EINVAL;
	}
	if (a_ctrl->act_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_util(
			&a_ctrl->i2c_client, MSM_CCI_RELEASE);
		if (rc < 0)
			pr_err("cci_init failed\n");
	}
	kfree(a_ctrl->i2c_reg_tbl);
	a_ctrl->i2c_reg_tbl = NULL;

	CDBG("Exit\n");
	return rc;
}

static const struct v4l2_subdev_internal_ops msm_actuator_internal_ops = {
	.close = msm_actuator_close,
};

static long msm_actuator_subdev_ioctl(struct v4l2_subdev *sd,
			unsigned int cmd, void *arg)
{
	struct msm_actuator_ctrl_t *a_ctrl = v4l2_get_subdevdata(sd);
	void __user *argp = (void __user *)arg;
	CDBG("Enter\n");
	CDBG("%s:%d a_ctrl %p argp %p\n", __func__, __LINE__, a_ctrl, argp);
	switch (cmd) {
	case VIDIOC_MSM_SENSOR_GET_SUBDEV_ID:
		return msm_actuator_get_subdev_id(a_ctrl, argp);
	case VIDIOC_MSM_ACTUATOR_CFG:
		return msm_actuator_config(a_ctrl, argp);
	case MSM_SD_SHUTDOWN:
		if (!a_ctrl->i2c_client.i2c_func_tbl) {
			pr_err("a_ctrl->i2c_client.i2c_func_tbl NULL\n");
			return -EINVAL;
		} else {
			return msm_actuator_close(sd, NULL);
		}
	default:
		return -ENOIOCTLCMD;
	}
}

#ifdef CONFIG_COMPAT
static long msm_actuator_subdev_do_ioctl(
	struct file *file, unsigned int cmd, void *arg)
{
	struct video_device *vdev = video_devdata(file);
	struct v4l2_subdev *sd = vdev_to_v4l2_subdev(vdev);
	struct msm_actuator_cfg_data32 *u32 =
		(struct msm_actuator_cfg_data32 *)arg;
	struct msm_actuator_cfg_data actuator_data;
	void *parg = arg;
	long rc;

	switch (cmd) {
	case VIDIOC_MSM_ACTUATOR_CFG32:
		cmd = VIDIOC_MSM_ACTUATOR_CFG;
		switch (u32->cfgtype) {
		case CFG_SET_ACTUATOR_INFO:
			actuator_data.cfgtype = u32->cfgtype;
			actuator_data.is_af_supported = u32->is_af_supported;
			actuator_data.cfg.set_info.actuator_params.act_type =
				u32->cfg.set_info.actuator_params.act_type;

			actuator_data.cfg.set_info.actuator_params
				.reg_tbl_size =
				u32->cfg.set_info.actuator_params.reg_tbl_size;

			actuator_data.cfg.set_info.actuator_params.data_size =
				u32->cfg.set_info.actuator_params.data_size;

			actuator_data.cfg.set_info.actuator_params
				.init_setting_size =
				u32->cfg.set_info.actuator_params
				.init_setting_size;

			actuator_data.cfg.set_info.actuator_params.i2c_addr =
				u32->cfg.set_info.actuator_params.i2c_addr;

			actuator_data.cfg.set_info.actuator_params
				.i2c_addr_type =
				u32->cfg.set_info.actuator_params.i2c_addr_type;

			actuator_data.cfg.set_info.actuator_params
				.i2c_data_type =
				u32->cfg.set_info.actuator_params.i2c_data_type;

			actuator_data.cfg.set_info.actuator_params
				.reg_tbl_params =
				compat_ptr(
				u32->cfg.set_info.actuator_params
				.reg_tbl_params);

			actuator_data.cfg.set_info.actuator_params
				.init_settings =
				compat_ptr(
				u32->cfg.set_info.actuator_params
				.init_settings);

			actuator_data.cfg.set_info.af_tuning_params
				.initial_code =
				u32->cfg.set_info.af_tuning_params.initial_code;

			actuator_data.cfg.set_info.af_tuning_params.pwd_step =
				u32->cfg.set_info.af_tuning_params.pwd_step;

			actuator_data.cfg.set_info.af_tuning_params
				.region_size =
				u32->cfg.set_info.af_tuning_params.region_size;

			actuator_data.cfg.set_info.af_tuning_params
				.total_steps =
				u32->cfg.set_info.af_tuning_params.total_steps;

			actuator_data.cfg.set_info.af_tuning_params
				.region_params = compat_ptr(
				u32->cfg.set_info.af_tuning_params
				.region_params);

			actuator_data.cfg.set_info.actuator_params.park_lens =
				u32->cfg.set_info.actuator_params.park_lens;

			
			actuator_data.cfg.set_info.enable_focus_step_log =
				u32->cfg.set_info.enable_focus_step_log;
			

			parg = &actuator_data;
			break;
		case CFG_SET_DEFAULT_FOCUS:
		case CFG_MOVE_FOCUS:
			actuator_data.cfgtype = u32->cfgtype;
			actuator_data.is_af_supported = u32->is_af_supported;
			actuator_data.cfg.move.dir = u32->cfg.move.dir;

			actuator_data.cfg.move.sign_dir =
				u32->cfg.move.sign_dir;

			actuator_data.cfg.move.dest_step_pos =
				u32->cfg.move.dest_step_pos;

			actuator_data.cfg.move.num_steps =
				u32->cfg.move.num_steps;

			actuator_data.cfg.move.curr_lens_pos =
				u32->cfg.move.curr_lens_pos;

			actuator_data.cfg.move.ringing_params =
				compat_ptr(u32->cfg.move.ringing_params);
			parg = &actuator_data;
			break;
		case CFG_SET_POSITION:
			actuator_data.cfgtype = u32->cfgtype;
			actuator_data.is_af_supported = u32->is_af_supported;
			memcpy(&actuator_data.cfg.setpos, &(u32->cfg.setpos),
				sizeof(struct msm_actuator_set_position_t));
			break;
		
		case CFG_SET_ACTUATOR_AF_VALUE:
			actuator_data.cfgtype = u32->cfgtype;
			actuator_data.cfg.af_value.AF_INF_MSB = u32->cfg.af_value.AF_INF_MSB;
			actuator_data.cfg.af_value.AF_INF_LSB = u32->cfg.af_value.AF_INF_LSB;
			actuator_data.cfg.af_value.VCM_BIAS = u32->cfg.af_value.VCM_BIAS;
			actuator_data.cfg.af_value.VCM_OFFSET = u32->cfg.af_value.VCM_OFFSET;
			actuator_data.cfg.af_value.VCM_VENDOR_ID_VERSION = u32->cfg.af_value.VCM_VENDOR_ID_VERSION;
			strlcpy(actuator_data.cfg.af_value.ACT_NAME, u32->cfg.af_value.ACT_NAME, sizeof(u32->cfg.af_value.ACT_NAME));
			parg = &actuator_data;
			break;
		
		default:
			actuator_data.cfgtype = u32->cfgtype;
			parg = &actuator_data;
			break;
		}
	}

	rc = msm_actuator_subdev_ioctl(sd, cmd, parg);

	switch (cmd) {

	case VIDIOC_MSM_ACTUATOR_CFG:

		switch (u32->cfgtype) {

		case CFG_SET_DEFAULT_FOCUS:
		case CFG_MOVE_FOCUS:
			u32->cfg.move.curr_lens_pos =
				actuator_data.cfg.move.curr_lens_pos;
			break;
		default:
			break;
		}
	}

	return rc;
}

static long msm_actuator_subdev_fops_ioctl(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	return video_usercopy(file, cmd, arg, msm_actuator_subdev_do_ioctl);
}
#endif

static int32_t msm_actuator_power_up(struct msm_actuator_ctrl_t *a_ctrl)
{
	int rc = 0;
	CDBG("%s called\n", __func__);

	rc = msm_actuator_vreg_control(a_ctrl, 1);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return rc;
	}

	a_ctrl->actuator_state = ACTUATOR_POWER_UP;

	CDBG("Exit\n");
	return rc;
}

static int32_t msm_actuator_power(struct v4l2_subdev *sd, int on)
{
	int rc = 0;
	struct msm_actuator_ctrl_t *a_ctrl = v4l2_get_subdevdata(sd);
	CDBG("Enter\n");
	mutex_lock(a_ctrl->actuator_mutex);
	if (on)
		rc = msm_actuator_power_up(a_ctrl);
	else
		rc = msm_actuator_power_down(a_ctrl);
	mutex_unlock(a_ctrl->actuator_mutex);
	CDBG("Exit\n");
	return rc;
}

static struct v4l2_subdev_core_ops msm_actuator_subdev_core_ops = {
	.ioctl = msm_actuator_subdev_ioctl,
	.s_power = msm_actuator_power,
};

static struct v4l2_subdev_ops msm_actuator_subdev_ops = {
	.core = &msm_actuator_subdev_core_ops,
};

static const struct i2c_device_id msm_actuator_i2c_id[] = {
	{"qcom,actuator", (kernel_ulong_t)NULL},
	{ }
};

static int32_t msm_actuator_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	struct msm_actuator_ctrl_t *act_ctrl_t = NULL;
	struct msm_actuator_vreg *vreg_cfg = NULL;
	CDBG("Enter\n");

	if (client == NULL) {
		pr_err("msm_actuator_i2c_probe: client is null\n");
		return -EINVAL;
	}

	act_ctrl_t = kzalloc(sizeof(struct msm_actuator_ctrl_t),
		GFP_KERNEL);
	if (!act_ctrl_t) {
		pr_err("%s:%d failed no memory\n", __func__, __LINE__);
		return -ENOMEM;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("i2c_check_functionality failed\n");
		goto probe_failure;
	}

	CDBG("client = 0x%p\n",  client);

	rc = of_property_read_u32(client->dev.of_node, "cell-index",
		&act_ctrl_t->subdev_id);
	CDBG("cell-index %d, rc %d\n", act_ctrl_t->subdev_id, rc);
	if (rc < 0) {
		pr_err("failed rc %d\n", rc);
		goto probe_failure;
	}

	if (of_find_property(client->dev.of_node,
		"qcom,cam-vreg-name", NULL)) {
		vreg_cfg = &act_ctrl_t->vreg_cfg;
		rc = msm_camera_get_dt_vreg_data(client->dev.of_node,
			&vreg_cfg->cam_vreg, &vreg_cfg->num_vreg);
		if (rc < 0) {
			pr_err("failed rc %d\n", rc);
			goto probe_failure;
		}
	}

	act_ctrl_t->i2c_driver = &msm_actuator_i2c_driver;
	act_ctrl_t->i2c_client.client = client;
	act_ctrl_t->curr_step_pos = 0,
	act_ctrl_t->curr_region_index = 0,
	act_ctrl_t->actuator_state = ACTUATOR_POWER_DOWN;
	
	act_ctrl_t->act_device_type = MSM_CAMERA_I2C_DEVICE;
	act_ctrl_t->i2c_client.i2c_func_tbl = &msm_sensor_qup_func_tbl;
	act_ctrl_t->act_v4l2_subdev_ops = &msm_actuator_subdev_ops;
	act_ctrl_t->actuator_mutex = &msm_actuator_mutex;
	act_ctrl_t->cam_name = act_ctrl_t->subdev_id;
	CDBG("act_ctrl_t->cam_name: %d", act_ctrl_t->cam_name);
	
	snprintf(act_ctrl_t->msm_sd.sd.name, sizeof(act_ctrl_t->msm_sd.sd.name),
		"%s", act_ctrl_t->i2c_driver->driver.name);

	
	v4l2_i2c_subdev_init(&act_ctrl_t->msm_sd.sd,
		act_ctrl_t->i2c_client.client,
		act_ctrl_t->act_v4l2_subdev_ops);
	v4l2_set_subdevdata(&act_ctrl_t->msm_sd.sd, act_ctrl_t);
	act_ctrl_t->msm_sd.sd.internal_ops = &msm_actuator_internal_ops;
	act_ctrl_t->msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	media_entity_init(&act_ctrl_t->msm_sd.sd.entity, 0, NULL, 0);
	act_ctrl_t->msm_sd.sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	act_ctrl_t->msm_sd.sd.entity.group_id = MSM_CAMERA_SUBDEV_ACTUATOR;
	act_ctrl_t->msm_sd.close_seq = MSM_SD_CLOSE_2ND_CATEGORY | 0x2;
	msm_sd_register(&act_ctrl_t->msm_sd);
	msm_actuator_v4l2_subdev_fops = v4l2_subdev_fops;
#ifdef CONFIG_COMPAT
	msm_actuator_v4l2_subdev_fops.compat_ioctl32 =
		msm_actuator_subdev_fops_ioctl;
#endif
	act_ctrl_t->msm_sd.sd.devnode->fops =
		&msm_actuator_v4l2_subdev_fops;

	pr_info("msm_actuator_i2c_probe: succeeded\n");
	CDBG("Exit\n");

	return 0;

probe_failure:
	kfree(act_ctrl_t);
	return rc;
}

static int32_t msm_actuator_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	struct msm_camera_cci_client *cci_client = NULL;
	struct msm_actuator_ctrl_t *msm_actuator_t = NULL;
	struct msm_actuator_vreg *vreg_cfg;
	CDBG("Enter\n");

	if (!pdev->dev.of_node) {
		pr_err("of_node NULL\n");
		return -EINVAL;
	}

	msm_actuator_t = kzalloc(sizeof(struct msm_actuator_ctrl_t),
		GFP_KERNEL);
	if (!msm_actuator_t) {
		pr_err("%s:%d failed no memory\n", __func__, __LINE__);
		return -ENOMEM;
	}
	rc = of_property_read_u32((&pdev->dev)->of_node, "cell-index",
		&pdev->id);
	CDBG("cell-index %d, rc %d\n", pdev->id, rc);
	if (rc < 0) {
		kfree(msm_actuator_t);
		pr_err("failed rc %d\n", rc);
		return rc;
	}

	rc = of_property_read_u32((&pdev->dev)->of_node, "qcom,cci-master",
		&msm_actuator_t->cci_master);
	CDBG("qcom,cci-master %d, rc %d\n", msm_actuator_t->cci_master, rc);
	if (rc < 0 || msm_actuator_t->cci_master >= MASTER_MAX) {
		kfree(msm_actuator_t);
		pr_err("failed rc %d\n", rc);
		return rc;
	}

	if (of_find_property((&pdev->dev)->of_node,
			"qcom,cam-vreg-name", NULL)) {
		vreg_cfg = &msm_actuator_t->vreg_cfg;
		rc = msm_camera_get_dt_vreg_data((&pdev->dev)->of_node,
			&vreg_cfg->cam_vreg, &vreg_cfg->num_vreg);
		if (rc < 0) {
			kfree(msm_actuator_t);
			pr_err("failed rc %d\n", rc);
			return rc;
		}
	}

	msm_actuator_t->act_v4l2_subdev_ops = &msm_actuator_subdev_ops;
	msm_actuator_t->actuator_mutex = &msm_actuator_mutex;
	msm_actuator_t->cam_name = pdev->id;

	
	msm_actuator_t->pdev = pdev;
	
	msm_actuator_t->act_device_type = MSM_CAMERA_PLATFORM_DEVICE;
	msm_actuator_t->i2c_client.i2c_func_tbl = &msm_sensor_cci_func_tbl;
	msm_actuator_t->i2c_client.cci_client = kzalloc(sizeof(
		struct msm_camera_cci_client), GFP_KERNEL);
	if (!msm_actuator_t->i2c_client.cci_client) {
		kfree(msm_actuator_t->vreg_cfg.cam_vreg);
		kfree(msm_actuator_t);
		pr_err("failed no memory\n");
		return -ENOMEM;
	}

	cci_client = msm_actuator_t->i2c_client.cci_client;
	cci_client->cci_subdev = msm_cci_get_subdev();
	cci_client->cci_i2c_master = msm_actuator_t->cci_master;
	v4l2_subdev_init(&msm_actuator_t->msm_sd.sd,
		msm_actuator_t->act_v4l2_subdev_ops);
	v4l2_set_subdevdata(&msm_actuator_t->msm_sd.sd, msm_actuator_t);
	msm_actuator_t->msm_sd.sd.internal_ops = &msm_actuator_internal_ops;
	msm_actuator_t->msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(msm_actuator_t->msm_sd.sd.name,
		ARRAY_SIZE(msm_actuator_t->msm_sd.sd.name), "msm_actuator");
	media_entity_init(&msm_actuator_t->msm_sd.sd.entity, 0, NULL, 0);
	msm_actuator_t->msm_sd.sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	msm_actuator_t->msm_sd.sd.entity.group_id = MSM_CAMERA_SUBDEV_ACTUATOR;
	msm_actuator_t->msm_sd.close_seq = MSM_SD_CLOSE_2ND_CATEGORY | 0x2;
	msm_sd_register(&msm_actuator_t->msm_sd);
	msm_actuator_t->actuator_state = ACTUATOR_POWER_DOWN;
	msm_actuator_v4l2_subdev_fops = v4l2_subdev_fops;
#ifdef CONFIG_COMPAT
	msm_actuator_v4l2_subdev_fops.compat_ioctl32 =
		msm_actuator_subdev_fops_ioctl;
#endif
	msm_actuator_t->msm_sd.sd.devnode->fops =
		&msm_actuator_v4l2_subdev_fops;

	CDBG("Exit\n");
	return rc;
}

static const struct of_device_id msm_actuator_i2c_dt_match[] = {
	{.compatible = "qcom,actuator"},
	{}
};

MODULE_DEVICE_TABLE(of, msm_actuator_i2c_dt_match);

static struct i2c_driver msm_actuator_i2c_driver = {
	.id_table = msm_actuator_i2c_id,
	.probe  = msm_actuator_i2c_probe,
	.remove = __exit_p(msm_actuator_i2c_remove),
	.driver = {
		.name = "qcom,actuator",
		.owner = THIS_MODULE,
		.of_match_table = msm_actuator_i2c_dt_match,
	},
};

static const struct of_device_id msm_actuator_dt_match[] = {
	{.compatible = "qcom,actuator", .data = NULL},
	{}
};

MODULE_DEVICE_TABLE(of, msm_actuator_dt_match);

static struct platform_driver msm_actuator_platform_driver = {
	.driver = {
		.name = "qcom,actuator",
		.owner = THIS_MODULE,
		.of_match_table = msm_actuator_dt_match,
	},
};

static int __init msm_actuator_init_module(void)
{
	int32_t rc = 0;
	CDBG("Enter\n");
	rc = platform_driver_probe(&msm_actuator_platform_driver,
		msm_actuator_platform_probe);
	if (!rc)
		return rc;

	CDBG("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&msm_actuator_i2c_driver);
}

static struct msm_actuator msm_vcm_actuator_table = {
	.act_type = ACTUATOR_VCM,
	.func_tbl = {
		.actuator_init_step_table = msm_actuator_init_step_table,
		.actuator_move_focus = msm_actuator_move_focus,
		.actuator_write_focus = msm_actuator_write_focus,
		.actuator_set_default_focus = msm_actuator_set_default_focus,
		.actuator_init_focus = msm_actuator_init_focus,
		.actuator_parse_i2c_params = msm_actuator_parse_i2c_params,
		.actuator_set_position = msm_actuator_set_position,
		.actuator_park_lens = msm_actuator_park_lens,
	},
};

static struct msm_actuator msm_piezo_actuator_table = {
	.act_type = ACTUATOR_PIEZO,
	.func_tbl = {
		.actuator_init_step_table = NULL,
		.actuator_move_focus = msm_actuator_piezo_move_focus,
		.actuator_write_focus = NULL,
		.actuator_set_default_focus =
			msm_actuator_piezo_set_default_focus,
		.actuator_init_focus = msm_actuator_init_focus,
		.actuator_parse_i2c_params = msm_actuator_parse_i2c_params,
		.actuator_park_lens = NULL,
	},
};

static struct msm_actuator msm_hvcm_actuator_table = {
	.act_type = ACTUATOR_HVCM,
	.func_tbl = {
		.actuator_init_step_table = msm_actuator_init_step_table,
		.actuator_move_focus = msm_actuator_move_focus,
		.actuator_write_focus = msm_actuator_write_focus,
		.actuator_set_default_focus = msm_actuator_set_default_focus,
		.actuator_init_focus = msm_actuator_init_focus,
		.actuator_parse_i2c_params = msm_actuator_parse_i2c_params,
		.actuator_set_position = msm_actuator_set_position,
		.actuator_park_lens = msm_actuator_park_lens,
	},
};

module_init(msm_actuator_init_module);
MODULE_DESCRIPTION("MSM ACTUATOR");
MODULE_LICENSE("GPL v2");
