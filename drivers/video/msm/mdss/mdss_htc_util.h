/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
 * Copyright (C) 2007 Google Incorporated
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
#ifndef HTC_UTIL_H
#define HTC_UTIL_H

#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/io.h>

#include "mdss_fb.h"
#include "mdss_mdp.h"

#define MAX_MODE_NAME_SIZE 32
/* bl_cali will receive parameter from 1 to 20000
 * this mean can support scale rate from 0.0001 to 2
 */
#define CALIBRATION_DATA_PATH "/calibration_data"
#define DISP_FLASH_DATA "disp_flash"
#define DISP_FLASH_DATA_SIZE 64
#define LIGHT_CALI_OFFSET 36
#define LIGHT_CALI_SIZE 8
#define LIGHT_RATIO_INDEX 0
#define LIGHT_R_INDEX 1
#define LIGHT_G_INDEX 2
#define LIGHT_B_INDEX 3
#define RGB_CALI_DEF 255
#define RGB_CHECK(x)	(x>0 && x<256)
#define RGB_CALIBRATION(ori,comp)	((long)(ori*comp/RGB_CALI_DEF))
#define BL_CALI_DEF  10000
#define BL_CALI_MAX  20000
#define BRI_GAIN_CHECK(x)	(x>0 && x<=20000)
#define BACKLIGHT_CALI(ori,comp) ((unsigned int)(ori*comp/BL_CALI_DEF))

enum {
	CABC_INDEX = 0,
	HUE_INDEX,
	PP_PCC_INDEX,
	SRE_INDEX,
	LIM_BRI_INDEX,
	BL_CALI_ENABLE_INDEX,
	RGB_CALI_ENABLE_INDEX,
};

struct attribute_status {
       char *title;
       u32 req_value;
       u32 cur_value;
       u32 def_value;
};

struct mdss_dspp_pcc_config {
	u32 reg_offset;
	u32 val;
};

struct mdss_dspp_pcc_mode {
	char mode_name[MAX_MODE_NAME_SIZE];
	u32 pcc_enable;
	struct mdss_dspp_pcc_config *dspp_pcc_config;
	int dspp_pcc_config_cnt;
};

struct cali_gain {
	uint16_t BKL;
	uint16_t R;
	uint16_t G;
	uint16_t B;
};

void htc_register_camera_bkl(int level, int level_dua);
void htc_register_attrs(struct kobject *led_kobj, struct msm_fb_data_type *mfd);
void htc_set_cabc(struct msm_fb_data_type *mfd, bool skip_check);
bool htc_set_sre(struct msm_fb_data_type *mfd);
void htc_set_limit_brightness(struct msm_fb_data_type *mfd);
void htc_update_bl_cali_data(struct msm_fb_data_type *mfd);
void htc_update_rgb_cali_data(struct msm_fb_data_type *mfd, struct mdp_pcc_cfg_data *config);
void htc_reset_status(void);
void htc_dimming_on(struct msm_fb_data_type *mfd);
void htc_dimming_off(void);
void htc_debugfs_init(struct msm_fb_data_type *mfd);
void htc_set_pp_pa(struct mdss_mdp_ctl *ctl);
void htc_set_pp_pcc(struct mdss_mdp_ctl *ctl);
void htc_panel_info(const char *panel);

int htc_backlight_transfer_bl_brightness(int val, struct msm_fb_data_type *mfd, bool brightness_to_bl);
int htc_backlight_bl_to_nits(int val, struct mdss_panel_info *panel_info);
int htc_backlight_nits_to_bl(int val, struct mdss_panel_info *panel_info);
#endif /* MDSS_FB_H */
