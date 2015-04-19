/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
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

#ifndef __MDSS_HDMI_MHL_H__
#define __MDSS_HDMI_MHL_H__

#include <linux/platform_device.h>

struct msm_hdmi_mhl_ops {
	u8 (*tmds_enabled)(struct platform_device *pdev);
	int (*set_mhl_max_pclk)(struct platform_device *pdev, u32 max_val);
	int (*set_upstream_hpd)(struct platform_device *pdev, uint8_t on);
	int (*set_hev_vic)(struct platform_device *pdev, uint8_t write_burst_vic);
};

int msm_hdmi_register_mhl(struct platform_device *pdev,
			  struct msm_hdmi_mhl_ops *ops, void *data);

#ifdef CONFIG_HTC_MHL_DETECTION_8620
struct t_mhl_disconnect_notifier{
        struct list_head mhl_notifier_link;
        const char *name;
        void (*func)(void);
};
int mhl_disconnect_register_notifier(struct t_mhl_disconnect_notifier *);
static LIST_HEAD(g_lh_mhl_disconnect_notifier_list);
#endif

#endif 
