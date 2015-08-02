/* Copyright (c) 2012-2015, The Linux Foundation. All rights reserved.
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

#ifndef QPNP_PON_H
#define QPNP_PON_H

#include <linux/errno.h>

enum pon_trigger_source {
	PON_SMPL = 1,
	PON_RTC,
	PON_DC_CHG,
	PON_USB_CHG,
	PON_PON1,
	PON_CBLPWR_N,
	PON_KPDPWR_N,
};

enum pon_power_off_type {
	PON_POWER_OFF_WARM_RESET	= 0x01,
	PON_POWER_OFF_SHUTDOWN		= 0x04,
	PON_POWER_OFF_HARD_RESET	= 0x07,
	PON_POWER_OFF_DVDD_HARD_RESET	= 0x08,
};

enum pon_type {
	PON_KPDPWR,
	PON_RESIN,
	PON_CBLPWR,
	PON_KPDPWR_RESIN,
};

enum pon_restart_reason {
	PON_RESTART_REASON_UNKNOWN	= 0x00,
	PON_RESTART_REASON_RECOVERY	= 0x01,
	PON_RESTART_REASON_BOOTLOADER	= 0x02,
	PON_RESTART_REASON_RTC		= 0x03,
};

#ifdef CONFIG_QPNP_POWER_ON
int qpnp_pon_system_pwr_off(enum pon_power_off_type type);
int qpnp_pon_is_warm_reset(void);
int qpnp_pon_trigger_config(enum pon_trigger_source pon_src, bool enable);
int qpnp_pon_wd_config(bool enable);

#ifdef CONFIG_KPDPWR_S2_DVDD_RESET
int qpnp_config_reset_enable(int pon_type, int en);
int qpnp_get_reset_en(int pon_type);
#endif 
int qpnp_pon_set_restart_reason(enum pon_restart_reason reason);
bool qpnp_pon_check_hard_reset_stored(void);

#else
static int qpnp_pon_system_pwr_off(enum pon_power_off_type type)
{
	return -ENODEV;
}
static inline int qpnp_pon_is_warm_reset(void) { return -ENODEV; }
static inline int qpnp_pon_trigger_config(enum pon_trigger_source pon_src,
							bool enable)
{
	return -ENODEV;
}
int qpnp_pon_wd_config(bool enable)
{
	return -ENODEV;
}

#ifdef CONFIG_KPDPWR_S2_DVDD_RESET
static inline int qpnp_config_reset_enable(int pon_type, int en)
{
	return -ENODEV;
}
static inline int qpnp_get_reset_en(int pon_type)
{
	return -ENODEV;
}
#endif 

static inline int qpnp_pon_set_restart_reason(enum pon_restart_reason reason)
{
	return -ENODEV;
}
static inline bool qpnp_pon_check_hard_reset_stored(void)
{
	return false;
}
#endif

#endif
