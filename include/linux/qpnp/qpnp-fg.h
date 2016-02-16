/* Copyright (c) 2011-2012, Code Aurora Forum. All rights reserved.
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

#ifndef __QPNP_FG_H
#define __QPNP_FG_H

#include <linux/errno.h>
#include <linux/power_supply.h>
#ifdef CONFIG_HTC_BATT_8960
#include <linux/power/htc_charger.h>
#endif

#define FALSE       0
#define TRUE        1

#ifdef CONFIG_QPNP_FG
#ifdef CONFIG_HTC_BATT_8960

extern bool flag_keep_charge_on;
extern bool flag_pa_fake_batt_temp;

#define SYS_BATT_STS_ISMISSING_VAL			0x0
#define SYS_BATT_STS_CHARGING_CV_VAL		0x1
#define SYS_BATT_STS_CHARGING_CC_VAL		0x2
#define SYS_BATT_STS_SYSTEM_FULL_VAL		0x3
#define SYS_BATT_STS_DISCHARGING_VAL		0x4
#define SYS_BATT_STS_AUTOMATIC_RECHARGE_VAL	0x5
#define SYS_BATT_STS_SUPPLEMENTAL_VAL		0x6
#define SYS_BATT_STS_EMPTY_VAL				0x7

int pmi8994_fg_get_system_status(int *result);
int pmi8994_fg_get_batt_current(int *result);
int pmi8994_fg_get_batt_soc(int *result);
int pmi8994_fg_get_batt_temperature(int *result);
int pmi8994_fg_get_batt_id(int *result);
int pmi8994_fg_get_batt_capacity(int *result);
int pmi8994_fg_get_batt_id_ohm(int *result);
int pmi8994_charger_get_attr_text(char *buf, int size);
int pmi8994_fg_store_battery_gauge_data_emmc(void);
int pmi8994_fg_get_battery_ui_soc(void);
int pmi8994_fg_store_battery_ui_soc(int soc_ui);
int pmi8994_fg_check_consistent(void);

#endif
#else 
#ifdef CONFIG_HTC_BATT_8960


static inline int pmi8994_fg_get_batt_current(int *result)
{
	return -ENXIO;
}
static inline int pmi8994_fg_get_batt_soc(int *result)
{
	return -ENXIO;
}
static inline int pmi8994_fg_get_batt_temperature(int *result)
{
	return -ENXIO;
}
static inline int pmi8994_fg_get_batt_id(int *result)
{
        return -ENXIO;
}
static inline int pmi8994_fg_get_batt_capacity(int *result)
{
        return -ENXIO;
}
static inline int pmi8994_fg_get_batt_id_ohm(int *result)
{
        return -ENXIO;
}
static inline int pmi8994_charger_get_attr_text(char *buf, int size)
{
        return -ENXIO;
}
static inline int pmi8994_fg_store_battery_gauge_data_emmc(void)
{
	return -ENXIO;
}
static inline int pmi8994_fg_get_battery_ui_soc(void)
{
	return -ENXIO;
}
static inline int pmi8994_fg_store_battery_ui_soc(int soc_ui)
{
	return -ENXIO;
}
static inline int pmi8994_fg_check_consistent(void)
{
	return -ENXIO;
}

#endif 
#endif 
#endif 

