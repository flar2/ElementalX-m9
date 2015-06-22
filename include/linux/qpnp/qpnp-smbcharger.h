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

#ifndef __QPNP_CHARGER_H
#define __QPNP_CHARGER_H

#include <linux/errno.h>
#include <linux/power_supply.h>
#ifdef CONFIG_HTC_BATT_8960
#include <linux/power/htc_charger.h>
#endif

#define FALSE       0
#define TRUE        1

#define RESTRICT_NONE   0
#define RESTRICT_SOFT   1
#define RESTRICT_HEAVY  2

#if 0
#define CHARGER_STORE_MAGIC_NUM		0xDDAACC00
#define BATTERY_HBOOT_MAGIC_NUM		0xDDAACC11
#define CHARGER_STORE_MAGIC_OFFSET		1056	
#define CHARGER_STORE_PRE_DELTA_VDDMAX_OFFSET	1080	

struct ext_usb_chg_pm8941 {
	const char	*name;
	void		*ctx;
	int		(*start_charging) (void *ctx);
	int		(*stop_charging) (void *ctx);
	bool		(*is_trickle) (void *ctx);
	struct htc_charger	*ichg;
};
#endif

#ifdef CONFIG_QPNP_SMBCHARGER
#ifdef CONFIG_HTC_BATT_8960

#if 0
int pm8941_is_pwr_src_plugged_in(void);
int pm8941_get_batt_temperature(int *result);





int pm8941_get_battery_status(void);
int pm8941_get_batt_present(void);
int pm8941_get_charging_source(int *result);
int pm8941_get_charging_enabled(int *result);

int pm8941_get_chg_usb_iusbmax(void);
int pm8941_get_chg_curr_settled(void);
int pm8941_get_chg_vinmin(void);
int pm8941_get_input_voltage_regulation(void);
int pm8941_set_chg_curr_settled(int val);
int pm8941_set_chg_vin_min(int val);
int pm8941_set_chg_iusbmax(int val);

int pm8941_set_hsml_target_ma(int target_ma);
int pm8941_is_batt_full(int *result);
int pm8941_is_batt_full_eoc_stop(int *result);
int pm8941_charger_get_attr_text(char *buf, int size);
int pm8941_gauge_get_attr_text(char *buf, int size);
int pm8941_fake_chg_gone_irq_handler(void);
int pm8941_fake_usbin_valid_irq_handler(void);
int pm8941_fake_coarse_det_usb_irq_handler(void);
#ifdef CONFIG_DUTY_CYCLE_LIMIT
int pm8941_limit_charge_enable(int chg_limit_reason,
			 int chg_limit_timer_sub_mask,
			 int limit_charge_timer_ma);
#else
int pm8941_limit_charge_enable(bool enable);
#endif
int pm8941_limit_input_current(bool enable, int reason);
int pm8941_is_chg_safety_timer_timeout(int *result);
int pm8941_get_usb_temperature(int *result);
int pm8941_store_battery_charger_data_emmc(void);
int pm8941_usb_overheat_otg_mode_check(void);
#endif

int pmi8994_gauge_get_attr_text(char *buf, int size);
int pmi8994_is_batt_temp_fault_disable_chg(int *result);
int pmi8994_is_batt_temperature_fault(int *result);
int pmi8994_get_batt_voltage(int *result);
int pmi8994_dump_all(void);
int pmi8994_get_charge_type(void);
int pmi8994_calc_max_flash_current(void);
int pmi8994_set_ftm_charge_enable_type(enum htc_ftm_power_source_type ftm_src);
int pmi8994_set_safety_timer_disable(int disable);
int pmi8994_is_charger_error_handle(void);
int pmi8994_usbin_mode_charge(void);
int pmi8994_set_pwrsrc_and_charger_enable(enum htc_power_source_type src,
			bool chg_enable, bool pwrsrc_enable);
int pmi8994_fake_src_detect_irq_handler(void);
int pmi8994_check_cable_status(void);
int pmi8994_charger_enable(bool enable);
int pmi8994_pwrsrc_enable(bool enable);
int pmi8994_is_charger_ovp(int* result);
#ifdef CONFIG_DUTY_CYCLE_LIMIT
int pmi8994_limit_charge_enable(int chg_limit_reason,
                                int chg_limit_timer_sub_mask,
                                int limit_charge_timer_ma);
#else
int pmi8994_limit_charge_enable(bool enable, int reason, int restrict);
#endif
int pmi8994_is_batt_full(int *result);
int pmi8994_is_batt_full_eoc_stop(int *result);
int pmi8994_get_cable_type_by_usb_detect(int *result);
int pmi8994_prepare_suspend(void);
int pmi8994_complete_resume(void);
int pm8994_set_hsml_target_ma(int target_ma);
int pmi8994_limit_input_current(bool enable, int reason);

#endif
#else 
#ifdef CONFIG_HTC_BATT_8960

#if 0
static inline int pm8941_is_pwr_src_plugged_in(void)
{
	return -ENXIO;
}
static inline int pm8941_get_batt_temperature(int *result)
{
	return -ENXIO;
}




static inline int pm8941_is_charger_ovp(int* result)
{
	return -ENXIO;
}

static inline int pm8941_charger_enable(bool enable)
{
	return -ENXIO;
}
static inline int pm8941_pwrsrc_enable(bool enable)
{
	return -ENXIO;
}

static inline int pm8941_get_battery_status(void)
{
	return -ENXIO;
}
static inline int pm8941_get_batt_present(void)
{
	return -ENXIO;
}
static inline int pm8941_get_charging_source(int *result)
{
	return -ENXIO;
}
static inline int pm8941_get_charging_enabled(int *result)
{
	return -ENXIO;
}

static inline int pm8941_get_chg_usb_iusbmax(void)
{
	return -ENXIO;
}
static inline int pm8941_get_chg_curr_settled(void)
{
	return -ENXIO;
}
static inline int pm8941_get_chg_vinmin(void)
{
	return -ENXIO;
}
static inline int pm8941_get_input_voltage_regulation(void)
{
	return -ENXIO;
}
static inline int pm8941_set_chg_curr_settled(int val)
{
	return -ENXIO;
}
static inline int pm8941_set_chg_vin_min(int val)
{
	return -ENXIO;
}
static inline int pm8941_set_chg_iusbmax(int val)
{
	return -ENXIO
}
static inline int pm8941_set_hsml_target_ma(int target_ma)
{
	return -ENXIO;
}
static inline int pm8941_is_batt_full(int *result)
{
	return -ENXIO;
}
static inline int pm8941_is_batt_full_eoc_stop(int *result)
{
	return -ENXIO;
}
static inline int pm8941_charger_get_attr_text(char *buf, int size)
{
	return -ENXIO;
}
static inline int pm8941_gauge_get_attr_text(char *buf, int size)
{
	return -ENXIO;
}
static inline int pm8941_fake_chg_gone_irq_handler(void)
{
	return -ENXIO;
}
static inline int pm8941_fake_usbin_valid_irq_handler(void)
{
	return -ENXIO;
}
static inline int pm8941_fake_coarse_det_usb_irq_handler(void)
{
	return -ENXIO;
}
static inline int pm8941_is_chg_safety_timer_timeout(int *result)
{
	return -ENXIO;
}
#ifdef CONFIG_DUTY_CYCLE_LIMIT
static inline int pm8941_limit_charge_enable(int chg_limit_reason,
			 int chg_limit_timer_sub_mask,
			 int limit_charge_timer_ma)
#else
static inline int pm8941_limit_charge_enable(bool enable)
{
	return -ENXIO;
}
#endif
static int pm8941_limit_input_current(bool enable, int reason);
{
	return -ENXIO;
}
static inline int pm8941_get_usb_temperature(int *result)
{
	return -ENXIO;
}
static inline int pm8941_store_battery_charger_data_emmc(void)
{
	return -ENXIO;
}
static inline int pm8941_usb_overheat_otg_mode_check(void)
{
	return -ENXIO;
}
#endif
static inline int pmi8994_gauge_get_attr_text(char *buf, int size)
{
        return -ENXIO;
}
static inline int pmi8994_is_batt_temp_fault_disable_chg(int *result)
{
	return -ENXIO;
}

static inline int pmi8994_is_batt_temperature_fault(int *result)
{
	return -ENXIO;
}

static inline int pmi8994_get_batt_voltage(int *result)
{
	return -ENXIO;
}

static inline int pmi8994_dump_all(void)
{
	return -ENXIO;
}

static inline int pmi8994_get_charge_type(void)
{
	return -ENXIO;
}

static inline int pmi8994_calc_max_flash_current(void)
{
	return -ENXIO;
}

static inline int pmi8994_fake_src_detect_irq_handler(void)
{
	return -ENXIO;
}

static inline int pmi8994_check_cable_status(void)
{
	return -ENXIO;
}

static inline int pmi8994_set_ftm_charge_enable_type(enum htc_ftm_power_source_type ftm_src)
{
	return -ENXIO;
}

static inline int pmi8994_set_safety_timer_disable(int disable)
{
	return -ENXIO;
}

static inline int pmi8994_is_charger_error_handle(void)
{
	return -ENXIO;
}

static inline int pmi8994_usbin_mode_charge(void)
{
	return -ENXIO;
}

static inline int pmi8994_set_pwrsrc_and_charger_enable(enum htc_power_source_type src,
			bool chg_enable, bool pwrsrc_enable)
{
	return -ENXIO;
}

static inline int pmi8994_charger_enable(bool enable)
{
	return -ENXIO;
}

static inline int pmi8994_pwrsrc_enable(bool enable)
{
	return -ENXIO;
}

static inline int pmi8994_is_charger_ovp(int *result)
{
	return -ENXIO;
}

#ifdef CONFIG_DUTY_CYCLE_LIMIT
static inline int pmi8994_limit_charge_enable(int chg_limit_reason,
                                              int chg_limit_timer_sub_mask,
                                              int limit_charge_timer_ma)
{
	return -ENXIO;
}
#else
static inline int pmi8994_limit_charge_enable(bool enable, int reason, int restrict)
{
	return -ENXIO;
}
#endif

static inline int pmi8994_is_batt_full(int *result)
{
	return -ENXIO;
}

static inline int pmi8994_is_batt_full_eoc_stop(int *result)
{
	return -ENXIO;
}

static inline int pmi8994_get_cable_type_by_usb_detect(int *result)
{
        return -ENXIO;
}

static inline int pmi8994_prepare_suspend(void)
{
        return -ENXIO;
}

static inline int pmi8994_complete_resume(void)
{
        return -ENXIO;
}
static inline int pmi8994_limit_input_current(bool enable, int reason)
{
        return -ENXIO;
}

static inline int pm8994_set_hsml_target_ma(int target_ma)
{
		return -ENXIO;
}

#endif 
#endif 
#endif 

