/*
 * Copyright (C) 2014 HTC, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __HTC_RAILWAYS_H__
#define __HTC_RAILWAYS_H__

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>


struct htc_railways_platform_data {
	phys_addr_t phys_addr_base;
	u32 phys_size;
};

struct htc_railways_info_base {
	void __iomem *reg_base;
	u8 init;
};

struct htc_railways_private_data {
	u32 read_idx;
	u32 len;
	char buf[512];
	struct htc_railways_platform_data *platform_data;
};

typedef enum {
    HTC_RAILWAY_INFO_MX     = 0,
    HTC_RAILWAY_INFO_CX     = 1,
    HTC_RAILWAY_INFO_GFX    = 2,
    HTC_RAILWAY_INFO_EBI    = 3,
    HTC_RAILWAY_INFO_RPM_MAX = 4,
    HTC_RAILWAY_INFO_APC0 = HTC_RAILWAY_INFO_RPM_MAX,
    HTC_RAILWAY_INFO_APC1 = 5,
    HTC_RAILWAY_INFO_APC_MAX = 6,
    HTC_RAILWAY_INFO_MAX = HTC_RAILWAY_INFO_APC_MAX
} htc_railways_info_rails;

typedef enum {
       HTC_RAILWAYS_APC0 = 0,
       HTC_RAILWAYS_APC1 = 1,
       HTC_RAILWAYS_MAX
} htc_apc_rails;

extern char *htc_railways_rails[HTC_RAILWAY_INFO_MAX];

typedef struct {
	u32 volt_init;
	u32 volt_min;
	u32 volt_max;
} htc_railways_info;

extern const struct file_operations htc_railways_fops;
extern struct htc_railways_info_base htc_railways_base;
extern htc_railways_info railways_info[HTC_RAILWAY_INFO_MAX];
extern htc_railways_info apc_railways_info[HTC_RAILWAYS_MAX];

static inline u32 htc_railways_read_long_register(void __iomem *regbase, int index, int offset)
{
    return readl_relaxed(regbase + offset + index * sizeof(htc_railways_info));
}

#endif
