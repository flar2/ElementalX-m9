/* Copyright (c) 2014 The Linux Foundation. All rights reserved.
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
#define pr_fmt(fmt) "SMBCHG: %s: " fmt, __func__

#include <linux/spmi.h>
#include <linux/spinlock.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/power_supply.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/bitops.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/spmi.h>
#include <linux/printk.h>
#include <linux/ratelimit.h>
#include <linux/rtc.h>

#include <asm-generic/errno-base.h>
#include <linux/wakelock.h>
#include <linux/qpnp/qpnp-adc.h>

#ifdef CONFIG_HTC_BATT_8960
#include <linux/of_fdt.h>
#include <linux/libfdt_env.h>
#include <linux/libfdt.h>
#include <linux/htc_flags.h>
#include <linux/power/htc_gauge.h>
#include <linux/qpnp/qpnp-smbcharger.h>
#include <linux/qpnp/qpnp-fg.h>
#include <linux/power/htc_battery_common.h>
#endif

/* Mask/Bit helpers */
#define _SMB_MASK(BITS, POS) \
	((unsigned char)(((1 << (BITS)) - 1) << (POS)))
#define SMB_MASK(LEFT_BIT_POS, RIGHT_BIT_POS) \
		_SMB_MASK((LEFT_BIT_POS) - (RIGHT_BIT_POS) + 1, \
				(RIGHT_BIT_POS))

#define MIN(a,b)                   (((a)<(b))?(a):(b))

static inline int ABS(int x) { return x >= 0 ? x : -x; }

/* Config registers */
struct smbchg_regulator {
	struct regulator_desc	rdesc;
	struct regulator_dev	*rdev;
};

struct parallel_usb_cfg {
	struct power_supply		*psy;
	int				min_current_thr_ma;
	int				current_max_ma;
	bool				avail;
};

struct smbchg_chip {
	struct device			*dev;
	struct spmi_device		*spmi;
	struct qpnp_vadc_chip		*vadc_dev;

	/* peripheral register address bases */
	u16				chgr_base;
	u16				bat_if_base;
	u16				usb_chgpth_base;
	u16				dc_chgpth_base;
	u16				otg_base;
	u16				misc_base;

	int				fake_battery_soc;
	u8				revision[4];

	/* configuration parameters */
	int				iterm_ma;
	int				usb_max_current_ma;
	int				dc_max_current_ma;
	int				usb_target_current_ma;
	int				dc_target_current_ma;
	int				target_fastchg_current_ma;
	int				fastchg_current_ma;
	int				vfloat_mv;
	int				resume_delta_mv;
	int				safety_time;
	int				prechg_safety_time;
	int				bmd_pin_src;
	int				usbin_chgr_cfg;
	bool				iterm_disabled;
	bool				bmd_algo_disabled;
	bool				soft_vfloat_comp_disabled;
	bool				chg_enabled;
	bool				low_icl_wa_on;
	bool				low_volt_dcin;
	bool				vbat_above_headroom;
	struct parallel_usb_cfg		parallel;

	/* flash current prediction */
	int				rpara_uohm;
	int				rslow_uohm;

	/* vfloat adjustment */
	int max_vbat_sample;
	int n_vbat_samples;
	int use_vfloat_adjustments;

	/* status variables */
	int				usb_suspended;
	int				dc_suspended;
	int				wake_reasons;
	int				previous_soc;
	bool				usb_online;
	bool				dc_present;
	bool				usb_present;
	bool				batt_present;
	bool				chg_done_batt_full;
	int					otg_retries;
	ktime_t				otg_enable_time;
	bool				aicl_deglitch_on;
	bool				otg_pulse_skip_dis;
	bool				aicl_deglitch_short;
	bool				sw_esr_pulse_en;
	bool				very_weak_charger;
	const char			*battery_type;

	/* jeita and temperature */
	bool				batt_hot;
	bool				batt_cold;
	bool				batt_warm;
	bool				batt_cool;
	unsigned int			thermal_levels;
	unsigned int			therm_lvl_sel;
	unsigned int			*thermal_mitigation;

	/* irqs */
	int				batt_hot_irq;
	int				batt_warm_irq;
	int				batt_cool_irq;
	int				batt_cold_irq;
	int				batt_missing_irq;
	int				vbat_low_irq;
	int				chg_hot_irq;
	int				chg_term_irq;
	int				taper_irq;
	int				recharge_irq;
	int				fastchg_irq;
	int				safety_timeout_irq;
	int				power_ok_irq;
	int				dcin_uv_irq;
	int				usbin_uv_irq;
	int				usbin_ov_irq;
	int				src_detect_irq;
	int				otg_fail_irq;
	int				otg_oc_irq;
	int				aicl_done_irq;
/*++ 2014/10/07 USB Team, PCN00014 ++*/
	/*int				usbid_change_irq;*/
/*-- 2014/10/07 USB Team, PCN00014 --*/
/* Remove chg_inhibit_irq because of PMI8994 known issue */
#if 0
	int				chg_inhibit_irq;
#endif
	int				chg_error_irq;
	/* aicl deglitch workaround */
	unsigned long	first_aicl_seconds;
	int 			aicl_irq_count;
	struct mutex			usb_status_lock;

	/* psy */
	struct power_supply		*usb_psy;
	struct power_supply		batt_psy;
	struct power_supply		dc_psy;
	struct power_supply		*bms_psy;
	int				dc_psy_type;
	const char			*bms_psy_name;
	const char			*battery_psy_name;
	bool				psy_registered;

	struct smbchg_regulator		otg_vreg;
	struct delayed_work		usb_set_online_work;
	spinlock_t			sec_access_lock;
	struct mutex			current_change_lock;
	struct mutex			usb_set_online_lock;
	struct mutex			usb_en_lock;
	struct mutex			dc_en_lock;
	struct mutex			pm_lock;
	struct work_struct		notify_fg_work;
	struct work_struct		batt_soc_work;
	struct work_struct		usb_aicl_limit_current;
	struct delayed_work		usb_limit_max_current;
	struct delayed_work		vfloat_adjust_work;
	struct delayed_work		hvdcp_det_work;

	/* htc add */
	int                             warm_ibatmax;
	int 				warm_bat_decidegc;
	int                             cool_bat_decidegc;
	int                             hot_bat_decidegc;
	int                             cold_bat_decidegc;
	int								vfloat_cmp_mv;
	int							eoc_ibat_thre_ma;
	int							high_fastchg_cur_ma;
	int							adjust_fastchg_cur_thr_mv;
	int							qc20_usb_max_current_ma;
	int 						kddi_limit1_temp;
	int 						kddi_limit1_current;
	int 						kddi_limit1_current_qc20;
	int 						kddi_limit2_temp;
	int 						kddi_limit2_current;
	int 						kddi_limit2_current_qc20;
	spinlock_t					vbus_lock;
	struct wake_lock 			vbus_wlock;
	struct wake_lock			eoc_worker_wlock;
	struct delayed_work 		vbus_detect_work;
	struct delayed_work			eoc_work;
	struct delayed_work		usb_type_polling_work;
	struct delayed_work			retry_aicl_work;
	struct delayed_work			hvdcp_5to9V_work;
	struct workqueue_struct 	*cable_detect_wq;
	enum htc_ftm_power_source_type	ftm_src;
	enum power_supply_type 		pre_usb_supply_type;
	enum htc_power_source_type  power_source_type;
};
static struct smbchg_chip *the_chip;

/* global variables */
/* 1.Disable temp protect. 2.Skip safety timer. */
bool flag_keep_charge_on;
/* Fake cable type to AC for ATS testing. */
bool flag_force_ac_chg;
/* Fake battery temperature not over 68 degree for PA testing. */
bool flag_pa_fake_batt_temp;
/* Disable safety timer */
bool flag_disable_safety_timer;
/* Disable battery temperature hot/cold protection*/
bool flag_disable_temp_protection;
/* is battery fully charged with charging stopped */
static bool flag_ats_limit_chg;

/* static global variables declaration */
static unsigned int chg_limit_current;
static int smbchg_debug_mask;
static int eoc_count;
static int eoc_count_by_curr;
static bool is_batt_full = false;
static bool is_batt_full_eoc_stop = false;
static int is_device_XA_or_XB;
static int is_safety_timer_disable;
static unsigned long dischg_plus_sr_time;
static unsigned long dischg_last_jiffies;
static unsigned long dischg_suspend_ms;
static bool is_limit_IUSB = false;
static bool gs_is_limit_1A = false;
static bool is_over_fastchg_thr_once = false;
static bool is_qc20_done_flag = false;
static int svbat_mv;
static int retry_aicl_cnt;
static int pre_usb_max_current_ma;
static int pre_current_ma;
static bool gs_is_bad_aicl_result = false;
static unsigned int gs_aspd_rerun_count = 0;
static int pwrsrc_disabled; /* USBIN side */
static bool wa_offmode_avoid_weak_chg_out_rapidly = false;

//usb_type
union power_supply_propval g_usbtype = {0,};

/* static functions declaration */
static DEFINE_SPINLOCK(set_current_lock);

/* definition */
#define PMI8994_CHG_I_MIN_MA          300
#define PMI8994_CHG_I_MIN_MA_L1       2000
#define PMI8994_CHG_I_MIN_MA_L2       300
#define EOC_CHECK_PERIOD_MS           20000
#define CONSECUTIVE_COUNT             3
#define CLEAR_FULL_STATE_BY_LEVEL_THR 97
#define DT_ROOT_VALUE 0
#define OCP_RELEASE_TIME_UPPER_BOUND_MS	180000
// Use for ATS 6 40000000
#define ATS_IBAT_LIMIT			1000

//0x1010
#define CHG_INHIBIT_BIT       BIT(1)
#define P2F_CHG_THRESHOLD_BIT BIT(4)
#define BAT_TAPER_MODE_BIT    BIT(6)
#define BAT_TCC_REACHED_BIT   BIT(7)

/* USB current limit*/
#define USB_MA_0       (0)
#define USB_MA_2       (2)
#define USB_MA_100     (100)
#define USB_MA_150     (150)
#define USB_MA_500     (500)
#define USB_MA_900     (900)
#define USB_MA_1000     (1000)
#define USB_MA_1100    (1100)
#define USB_MA_1200    (1200)
#define USB_MA_1400 (1400)
#define USB_MA_1450 (1450)
#define USB_MA_1500	(1500)
#define USB_MA_1600	(1600)

/* HVDCP related setting */
#define CFG_SYSMIN 0xF4
#define HVDCP_CHANGE_PERIOD_MS 10000

/* Chg error bit related setting */
#define VOLT_ALLOW_RESET_CHG_ERROR_BIT 4000
#define CHG_SFT_RT_STS 0x08

enum print_reason {
	PR_REGISTER = BIT(0),
	PR_INTERRUPT = BIT(1),
	PR_STATUS = BIT(2),
	PR_DUMP = BIT(3),
	PR_PM = BIT(4),
	PR_MISC	= BIT(5),
};
enum wake_reason {
	PM_PARALLEL_CHECK = BIT(0),
	PM_REASON_VFLOAT_ADJUST = BIT(1),
	PM_ESR_PULSE = BIT(2),
};
enum qpnp_tm_state_smbchg {
	ADC_TM_WARM_STATE_SMBCHG = 0,
	ADC_TM_COOL_STATE_SMBCHG,
	ADC_TM_HOT_STATE_SMBCHG,
	ADC_TM_COLD_STATE_SMBCHG,
	ADC_TM_STATE_NUM_SMBCHG,
};
enum device_model {
	DEVICE_MODEL_INVALID = 0,
	DEVICE_MODEL_XA,
	DEVICE_MODEL_XB,
	DEVICE_MODEL_XC_OR_ABOVE,
};

#define HYSTERISIS_DECIDEGC 20

module_param_named(
	debug_mask, smbchg_debug_mask, int, S_IRUSR | S_IWUSR
);

static int
pmi8994_get_usbin_voltage_now(struct smbchg_chip *chip);
static irqreturn_t
cable_detection_vbus_irq_handler(void);

static void smbchg_rerun_aicl(struct smbchg_chip *chip);
static int rerun_apsd(struct smbchg_chip *chip);


#define pr_smb(reason, fmt, ...)				\
	do {							\
		if (smbchg_debug_mask & (reason))		\
			pr_warn(fmt, ##__VA_ARGS__);		\
		else						\
			pr_debug(fmt, ##__VA_ARGS__);		\
	} while (0)

#define pr_smb_rt(reason, fmt, ...)					\
	do {								\
		if (smbchg_debug_mask & (reason))			\
			pr_info_ratelimited(fmt, ##__VA_ARGS__);	\
		else							\
			pr_debug_ratelimited(fmt, ##__VA_ARGS__);	\
	} while (0)

static int smbchg_read(struct smbchg_chip *chip, u8 *val,
			u16 addr, int count)
{
	int rc = 0;
	struct spmi_device *spmi = chip->spmi;

	if (addr == 0) {
		dev_err(chip->dev, "addr cannot be zero addr=0x%02x sid=0x%02x rc=%d\n",
			addr, spmi->sid, rc);
		return -EINVAL;
	}

	rc = spmi_ext_register_readl(spmi->ctrl, spmi->sid, addr, val, count);
	if (rc) {
		dev_err(chip->dev, "spmi read failed addr=0x%02x sid=0x%02x rc=%d\n",
				addr, spmi->sid, rc);
		return rc;
	}
	return 0;
}

/*
 * Writes an arbitrary number of bytes to a specified register
 *
 * Do not use this function for register writes if possible. Instead use the
 * smbchg_masked_write function.
 *
 * The sec_access_lock must be held for all register writes and this function
 * does not do that. If this function is used, please hold the spinlock or
 * random secure access writes may fail.
 */
static int smbchg_write(struct smbchg_chip *chip, u8 *val,
			u16 addr, int count)
{
	int rc = 0;
	struct spmi_device *spmi = chip->spmi;

	if (addr == 0) {
		dev_err(chip->dev, "addr cannot be zero addr=0x%02x sid=0x%02x rc=%d\n",
			addr, spmi->sid, rc);
		return -EINVAL;
	}

	rc = spmi_ext_register_writel(spmi->ctrl, spmi->sid, addr, val, count);
	if (rc) {
		dev_err(chip->dev, "write failed addr=0x%02x sid=0x%02x rc=%d\n",
			addr, spmi->sid, rc);
		return rc;
	}

	return 0;
}

/*
 * Writes a register to the specified by the base and limited by the bit mask
 *
 * Do not use this function for register writes if possible. Instead use the
 * smbchg_masked_write function.
 *
 * The sec_access_lock must be held for all register writes and this function
 * does not do that. If this function is used, please hold the spinlock or
 * random secure access writes may fail.
 */
static int smbchg_masked_write_raw(struct smbchg_chip *chip, u16 base, u8 mask,
									u8 val)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, base, 1);
	if (rc) {
		dev_err(chip->dev, "spmi read failed: addr=%03X, rc=%d\n",
				base, rc);
		return rc;
	}

	reg &= ~mask;
	reg |= val & mask;

	pr_smb(PR_REGISTER, "addr = 0x%x writing 0x%x\n", base, reg);

	rc = smbchg_write(chip, &reg, base, 1);
	if (rc) {
		dev_err(chip->dev, "spmi write failed: addr=%03X, rc=%d\n",
				base, rc);
		return rc;
	}

	return 0;
}

/*
 * Writes a register to the specified by the base and limited by the bit mask
 *
 * This function holds a spin lock to ensure secure access register writes goes
 * through. If the secure access unlock register is armed, any old register
 * write can unarm the secure access unlock, causing the next write to fail.
 *
 * Note: do not use this for sec_access registers. Instead use the function
 * below: smbchg_sec_masked_write
 */
static int smbchg_masked_write(struct smbchg_chip *chip, u16 base, u8 mask,
								u8 val)
{
	unsigned long flags;
	int rc;

	spin_lock_irqsave(&chip->sec_access_lock, flags);
	rc = smbchg_masked_write_raw(chip, base, mask, val);
	spin_unlock_irqrestore(&chip->sec_access_lock, flags);

	return rc;
}

/*
 * Unlocks sec access and writes to the register specified.
 *
 * This function holds a spin lock to exclude other register writes while
 * the two writes are taking place.
 */
#define SEC_ACCESS_OFFSET	0xD0
#define SEC_ACCESS_VALUE	0xA5
#define PERIPHERAL_MASK		0xFF
static int smbchg_sec_masked_write(struct smbchg_chip *chip, u16 base, u8 mask,
									u8 val)
{
	unsigned long flags;
	int rc;
	u16 peripheral_base = base & (~PERIPHERAL_MASK);

	spin_lock_irqsave(&chip->sec_access_lock, flags);

	rc = smbchg_masked_write_raw(chip, peripheral_base + SEC_ACCESS_OFFSET,
				SEC_ACCESS_VALUE, SEC_ACCESS_VALUE);
	if (rc) {
		dev_err(chip->dev, "Unable to unlock sec_access: %d", rc);
		goto out;
	}

	rc = smbchg_masked_write_raw(chip, base, mask, val);

out:
	spin_unlock_irqrestore(&chip->sec_access_lock, flags);
	return rc;
}

static void smbchg_stay_awake(struct smbchg_chip *chip, int reason)
{
	int reasons;

	mutex_lock(&chip->pm_lock);
	reasons = chip->wake_reasons | reason;
	if (reasons != 0 && chip->wake_reasons == 0) {
		pr_smb(PR_PM, "staying awake: 0x%02x (bit %d)\n",
				reasons, reason);
		pm_stay_awake(chip->dev);
	}
	chip->wake_reasons = reasons;
	mutex_unlock(&chip->pm_lock);
}

static void smbchg_relax(struct smbchg_chip *chip, int reason)
{
	int reasons;

	mutex_lock(&chip->pm_lock);
	reasons = chip->wake_reasons & (~reason);
	if (reasons == 0 && chip->wake_reasons != 0) {
		pr_smb(PR_PM, "relaxing: 0x%02x (bit %d)\n",
				reasons, reason);
		pm_relax(chip->dev);
	}
	chip->wake_reasons = reasons;
	mutex_unlock(&chip->pm_lock);
};

#define RID_STS				0xB
#define RID_MASK			0xF
/*++ 2014/10/07 USB Team, PCN00014 ++*/
/*
static bool is_otg_present(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + RID_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev,
				"Couldn't read usb rid status rc = %d\n", rc);
		return false;
	}

	pr_smb(PR_STATUS, "RID_STS = %02x\n", reg);

	return (reg & RID_MASK) == 0;
}
*/
/*-- 2014/10/07 USB Team, PCN00014 --*/

#define USBIN_9V			BIT(5)
#define USBIN_UNREG			BIT(4)
#define USBIN_LV			BIT(3)
#define DCIN_9V				BIT(2)
#define DCIN_UNREG			BIT(1)
#define DCIN_LV				BIT(0)
#define INPUT_STS			0x0D
static bool is_dc_present(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + INPUT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb status rc = %d\n", rc);
		return false;
	}

	return !!(reg & (DCIN_9V | DCIN_UNREG | DCIN_LV));
}

#define RT_STS			0x10
#define USBIN_UV_BIT		BIT(0)
#define USBIN_OV_BIT		BIT(1)
#define USBIN_SRC_DET_BIT	BIT(2)
static bool is_usb_present(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb rt status rc = %d\n", rc);
		return false;
	}
	if (!(reg & USBIN_SRC_DET_BIT) || (reg & USBIN_OV_BIT))
		return false;

	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + INPUT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb status rc = %d\n", rc);
		return false;
	}

	return !!(reg & (USBIN_9V | USBIN_UNREG | USBIN_LV));
}

static bool get_prop_usb_ov_status(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb rt status rc = %d\n", rc);
		return 0;
	}
	pr_smb(PR_STATUS, "reg=%x\n", reg);
	if (reg & USBIN_OV_BIT) {
		return 1;
	} else {
		return 0;
	}
}

static char *usb_type_str[] = {
	"ACA_DOCK",	/* bit 0 */
	"ACA_C",	/* bit 1 */
	"ACA_B",	/* bit 2 */
	"ACA_A",	/* bit 3 */
	"SDP",		/* bit 4 */
	"OTHER",	/* bit 5 */
	"DCP",		/* bit 6 */
	"CDP",		/* bit 7 */
	"NONE",		/* bit 8 error case */
};

#define BITS_PER_REG	8
/* helper to return the string of USB type */
static char *get_usb_type_name(u8 type_reg)
{
	unsigned long type = type_reg;

	return usb_type_str[find_first_bit(&type, BITS_PER_REG)];
}

static enum power_supply_type usb_type_enum[] = {
	POWER_SUPPLY_TYPE_USB_ACA,	/* bit 0 */
	POWER_SUPPLY_TYPE_USB_ACA,	/* bit 1 */
	POWER_SUPPLY_TYPE_USB_ACA,	/* bit 2 */
	POWER_SUPPLY_TYPE_USB_ACA,	/* bit 3 */
	POWER_SUPPLY_TYPE_USB,		/* bit 4 */
	POWER_SUPPLY_TYPE_USB_DCP,	/* bit 5 */
	POWER_SUPPLY_TYPE_USB_DCP,	/* bit 6 */
	POWER_SUPPLY_TYPE_USB_CDP,	/* bit 7 */
	POWER_SUPPLY_TYPE_UNKNOWN,	/* bit 8 error case, report UNKNOWN */
};

/* helper to return enum power_supply_type of USB type */
static enum power_supply_type get_usb_supply_type(u8 type_reg)
{
	unsigned long type = type_reg;

	return usb_type_enum[find_first_bit(&type, BITS_PER_REG)];
}

#if 0//FIXME
static enum power_supply_property smbchg_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL,
};
#endif

#define CHGR_STS			0x0E
#define DONE_STS_BIT			BIT(5)
#define BATT_LESS_THAN_2V		BIT(4)
#define CHG_HOLD_OFF_BIT		BIT(3)
#define CHG_TYPE_MASK			SMB_MASK(2, 1)
#define CHG_TYPE_SHIFT			1
#define BATT_NOT_CHG_VAL		0x0
#define BATT_PRE_CHG_VAL		0x1
#define BATT_FAST_CHG_VAL		0x2
#define BATT_TAPER_CHG_VAL		0x3
#define CHG_EN_BIT			BIT(0)
static int get_prop_batt_status(struct smbchg_chip *chip)
{
	int rc, status = POWER_SUPPLY_STATUS_DISCHARGING;
	u8 reg = 0, chg_type;

	if (chip->chg_done_batt_full)
		return POWER_SUPPLY_STATUS_FULL;

	rc = smbchg_read(chip, &reg, chip->chgr_base + CHGR_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Unable to read CHGR_STS rc = %d\n", rc);
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}

	if (reg & CHG_HOLD_OFF_BIT) {
		/*
		 * when chg hold off happens the battery is
		 * not charging
		 */
		status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		goto out;
	}

	chg_type = (reg & CHG_TYPE_MASK) >> CHG_TYPE_SHIFT;

	if (chg_type == BATT_NOT_CHG_VAL)
		status = POWER_SUPPLY_STATUS_DISCHARGING;
	else
		status = POWER_SUPPLY_STATUS_CHARGING;
out:
	pr_smb_rt(PR_STATUS, "CHGR_STS = 0x%02x\n", reg);
	return status;
}

#define BAT_PRES_STATUS			0x08
#define BAT_PRES_BIT			BIT(7)
static int get_prop_batt_present(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->bat_if_base + BAT_PRES_STATUS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Unable to read CHGR_STS rc = %d\n", rc);
		return 0;
	}

	return !!(reg & BAT_PRES_BIT);
}

static int get_prop_charge_type(struct smbchg_chip *chip)
{
	int rc;
	u8 reg, chg_type;

	rc = smbchg_read(chip, &reg, chip->chgr_base + CHGR_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Unable to read CHGR_STS rc = %d\n", rc);
		return 0;
	}

	chg_type = (reg & CHG_TYPE_MASK) >> CHG_TYPE_SHIFT;
	if (chg_type == BATT_NOT_CHG_VAL)
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	else if (chg_type == BATT_FAST_CHG_VAL)
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	else if (chg_type == BATT_PRE_CHG_VAL)
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
	else if (chg_type == BATT_TAPER_CHG_VAL)
		return POWER_SUPPLY_CHARGE_TYPE_TAPER;

	return POWER_SUPPLY_CHARGE_TYPE_NONE;
}

static int set_property_on_fg(struct smbchg_chip *chip,
		enum power_supply_property prop, int val)
{
	int rc;
	union power_supply_propval ret = {0, };

	if (!chip->bms_psy && chip->bms_psy_name)
		chip->bms_psy =
			power_supply_get_by_name((char *)chip->bms_psy_name);
	if (!chip->bms_psy) {
		pr_smb(PR_STATUS, "no bms psy found\n");
		return -EINVAL;
	}

	ret.intval = val;
	rc = chip->bms_psy->set_property(chip->bms_psy, prop, &ret);
	if (rc)
		pr_smb(PR_STATUS,
			"bms psy does not allow updating prop %d rc = %d\n",
			prop, rc);

	return rc;
}

static int get_property_from_fg(struct smbchg_chip *chip,
		enum power_supply_property prop, int *val)
{
	int rc;
	union power_supply_propval ret = {0, };

	if (!chip->bms_psy && chip->bms_psy_name)
		chip->bms_psy =
			power_supply_get_by_name((char *)chip->bms_psy_name);
	if (!chip->bms_psy) {
		pr_smb(PR_STATUS, "no bms psy found\n");
		return -EINVAL;
	}

	rc = chip->bms_psy->get_property(chip->bms_psy, prop, &ret);
	if (rc) {
		pr_smb(PR_STATUS,
			"bms psy doesn't support reading prop %d rc = %d\n",
			prop, rc);
		return rc;
	}

	*val = ret.intval;
	return rc;
}

#if 0//FIXME
#define DEFAULT_BATT_CAPACITY	50
static int get_prop_batt_capacity(struct smbchg_chip *chip)
{
	union power_supply_propval ret = {0, };

	if (chip->fake_battery_soc >= 0)
		return chip->fake_battery_soc;
	if (!chip->bms_psy && chip->bms_psy_name)
		chip->bms_psy =
			power_supply_get_by_name((char *)chip->bms_psy_name);
	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_CAPACITY, &ret);
		return ret.intval;
	}

	return DEFAULT_BATT_CAPACITY;
}
#endif

static int get_prop_batt_health(struct smbchg_chip *chip)
{
	if (chip->batt_hot)
		return POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (chip->batt_cold)
		return POWER_SUPPLY_HEALTH_COLD;
	else if (chip->batt_warm)
		return POWER_SUPPLY_HEALTH_WARM;
	else if (chip->batt_cool)
		return POWER_SUPPLY_HEALTH_COOL;
	else
		return POWER_SUPPLY_HEALTH_GOOD;
}

int usb_current_table[] = {
	300,
	400,
	450,
	475,
	500,
	550,
	600,
	650,
	700,
	900,
	950,
	1000,
	1100,
	1200,
	1400,
	1450,
	1500,
	1600,
	1800,
	1850,
	1880,
	1910,
	1930,
	1950,
	1970,
	2000,
	2050,
	2100,
	2300,
	2400,
	2500,
	3000
};

int dc_current_table[] = {
	300,
	400,
	450,
	475,
	500,
	550,
	600,
	650,
	700,
	900,
	950,
	1000,
	1100,
	1200,
	1400,
	1450,
	1500,
	1600,
	1800,
	1850,
	1880,
	1910,
	1930,
	1950,
	1970,
	2000,
};

static int calc_thermal_limited_current(struct smbchg_chip *chip,
						int current_ma)
{
	int therm_ma;

	if (chip->therm_lvl_sel > 0
			&& chip->therm_lvl_sel < (chip->thermal_levels - 1)) {
		/*
		 * consider thermal limit only when it is active and not at
		 * the highest level
		 */
		therm_ma = (int)chip->thermal_mitigation[chip->therm_lvl_sel];
		if (therm_ma < current_ma) {
			pr_smb(PR_STATUS,
				"Limiting current due to thermal: %d mA",
				therm_ma);
			return therm_ma;
		}
	}

	return current_ma;
}

#define CMD_IL			0x40
#define USBIN_SUSPEND_BIT	BIT(4)
#define CURRENT_100_MA		100
#define CURRENT_150_MA		150
#define CURRENT_500_MA		500
#define CURRENT_900_MA		900
#define SUSPEND_CURRENT_MA	2
static int smbchg_usb_suspend(struct smbchg_chip *chip, bool suspend)
{
	int rc;

	rc = smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL,
			USBIN_SUSPEND_BIT, suspend ? USBIN_SUSPEND_BIT : 0);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set usb suspend rc = %d\n", rc);
	return rc;
}

#define DCIN_SUSPEND_BIT	BIT(3)
static int smbchg_dc_suspend(struct smbchg_chip *chip, bool suspend)
{
	int rc = 0;

	rc = smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL,
			DCIN_SUSPEND_BIT, suspend ? DCIN_SUSPEND_BIT : 0);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set dc suspend rc = %d\n", rc);
	return rc;
}

#define IL_CFG			0xF2
#define DCIN_INPUT_MASK	SMB_MASK(4, 0)
static int smbchg_set_dc_current_max(struct smbchg_chip *chip, int current_ma)
{
	int i;
	u8 dc_cur_val;

	for (i = ARRAY_SIZE(dc_current_table) - 1; i >= 0; i--) {
		if (current_ma >= dc_current_table[i])
			break;
	}

	if (i < 0) {
		dev_err(chip->dev, "Cannot find %dma current_table\n",
				current_ma);
		return -EINVAL;
	}

	chip->dc_max_current_ma = dc_current_table[i];
	dc_cur_val = i & DCIN_INPUT_MASK;

	pr_smb(PR_STATUS, "dc current set to %d mA\n",
			chip->dc_max_current_ma);
	return smbchg_sec_masked_write(chip, chip->dc_chgpth_base + IL_CFG,
				DCIN_INPUT_MASK, dc_cur_val);
}

enum enable_reason {
	/* userspace has suspended charging altogether */
	REASON_USER = BIT(0),
	/*
	 * this specific path has been suspended through the power supply
	 * framework
	 */
	REASON_POWER_SUPPLY = BIT(1),
	/*
	 * the usb driver has suspended this path by setting a current limit
	 * of < 2MA
	 */
	REASON_USB = BIT(2),
	/*
	 * when a wireless charger comes online,
	 * the dc path is suspended for a second
	 */
	REASON_WIRELESS = BIT(3),
	/*
	 * the thermal daemon can suspend a charge path when the system
	 * temperature levels rise
	 */
	REASON_THERMAL = BIT(4),
};

static struct power_supply *get_parallel_psy(struct smbchg_chip *chip)
{
	if (chip->parallel.psy)
		return chip->parallel.psy;
	chip->parallel.psy = power_supply_get_by_name("usb-parallel");
	if (!chip->parallel.psy)
		pr_smb(PR_STATUS, "parallel charger not found\n");
	return chip->parallel.psy;
}

#define ICL_STS_1_REG			0x7
#define ICL_STS_2_REG			0x9
#define ICL_STS_MASK			0x1F
#define AICL_STS_BIT			BIT(5)
#define AICL_SUSP_BIT			BIT(6)
#define USBIN_SUSPEND_STS_BIT		BIT(3)
#define USBIN_ACTIVE_PWR_SRC_BIT	BIT(1)
static void smbchg_parallel_usb_determine_current(struct smbchg_chip *chip)
{
	struct power_supply *parallel_psy;
	int current_limit_ma, parallel_cl_ma, total_current_ma;
	int new_parallel_cl_ma, rc;
	u8 reg;

	parallel_psy = get_parallel_psy(chip);
	if (!parallel_psy)
		return;

	rc = smbchg_read(chip, &reg,
			chip->usb_chgpth_base + ICL_STS_2_REG, 1);
	if (rc) {
		dev_err(chip->dev, "Could not read usb icl sts 2: %d\n", rc);
		return;
	}

	/*
	 * If the usbin is suspended or not the active power src, then this
	 * was triggered from DCIN AICL. Retrun silently if this is the case.
	 */
	if (!!(reg & USBIN_SUSPEND_STS_BIT) ||
				!(reg & USBIN_ACTIVE_PWR_SRC_BIT)) {
		pr_smb(PR_STATUS, "USB not active power source: %02x\n", reg);
		return;
	}

	rc = smbchg_read(chip, &reg,
			chip->usb_chgpth_base + ICL_STS_1_REG, 1);
	if (rc) {
		dev_err(chip->dev, "Could not read usb icl sts 1: %d\n", rc);
		return;
	}

	reg &= ICL_STS_MASK;
	if (reg >= ARRAY_SIZE(usb_current_table)) {
		pr_warn("invalid AICL value: %02x\n", reg);
		return;
	}

	parallel_cl_ma = chip->parallel.current_max_ma;
	if (parallel_cl_ma <= SUSPEND_CURRENT_MA)
		parallel_cl_ma = 0;

	current_limit_ma = usb_current_table[reg];
	total_current_ma = current_limit_ma + parallel_cl_ma;

	/*
	 * if the total available current is less than the minimum threshold
	 * to enable parallel charging, set the current limit to 0 to disable
	 * the parallel charge path.
	 *
	 * otherwise, set the parallel charge path's input current limit (ICL)
	 * to the total current / 2
	 */
	if (total_current_ma <= chip->parallel.min_current_thr_ma)
		new_parallel_cl_ma = 0;
	else
		new_parallel_cl_ma = total_current_ma / 2;

	if (new_parallel_cl_ma == 0)
		new_parallel_cl_ma = SUSPEND_CURRENT_MA;
	if (new_parallel_cl_ma < chip->parallel.current_max_ma
			|| chip->parallel.current_max_ma <= SUSPEND_CURRENT_MA)
		chip->parallel.current_max_ma = new_parallel_cl_ma;
	pr_smb(PR_STATUS, "ICL at %d. Setting Parallel ICL at %d\n",
			current_limit_ma, chip->parallel.current_max_ma);

	mutex_lock(&chip->usb_en_lock);
	if (!chip->usb_suspended)
		power_supply_set_current_limit(parallel_psy,
					chip->parallel.current_max_ma * 1000);
	mutex_unlock(&chip->usb_en_lock);
}

static void smbchg_parallel_usb_en(struct smbchg_chip *chip, bool enable)
{
	struct power_supply *parallel_psy;

	parallel_psy = get_parallel_psy(chip);
	if (!parallel_psy)
		return;

	power_supply_set_current_limit(parallel_psy,
			enable ? chip->parallel.current_max_ma * 1000
			: (SUSPEND_CURRENT_MA * 1000));
	pr_smb(PR_STATUS, "parallel charger %s\n",
			enable ? "unsuspended" : "suspended");
}

static void smbchg_usb_update_online_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct smbchg_chip *chip = container_of(dwork,
				struct smbchg_chip,
				usb_set_online_work);
	union power_supply_propval prop = {0, };
	bool user_enabled = (chip->usb_suspended & REASON_USER) == 0;
	int online;
	int rc;

	rc = chip->usb_psy->get_property(chip->usb_psy,
				POWER_SUPPLY_PROP_HEALTH, &prop);
	if (rc < 0) {
		pr_err("could not read USB health, rc=%d\n", rc);
		return;
	}

	online = user_enabled && chip->usb_present
		&& prop.intval != POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;

	pr_smb(PR_STATUS, "user_enabled=%x, usb_suspend=%d, online=%d\n",
			user_enabled, chip->usb_suspended, online);
	mutex_lock(&chip->usb_set_online_lock);
	if (chip->usb_online != online) {
		power_supply_set_online(chip->usb_psy, online);
		chip->usb_online = online;
	}
	mutex_unlock(&chip->usb_set_online_lock);
}

static int smbchg_usb_en(struct smbchg_chip *chip, bool enable,
		enum enable_reason reason)
{
	int rc = 0, suspended;

	pr_smb(PR_STATUS, "usb %s, susp = %02x, en? = %d, reason = %02x\n",
			chip->usb_suspended == 0 ? "enabled"
			: "suspended", chip->usb_suspended, enable, reason);
	mutex_lock(&chip->usb_en_lock);
	if (!enable)
		suspended = chip->usb_suspended | reason;
	else
		suspended = chip->usb_suspended & (~reason);

	/* avoid unnecessary spmi interactions if nothing changed */
	if (!!suspended == !!chip->usb_suspended)
		goto out;

	if (chip->parallel.avail)
		smbchg_parallel_usb_en(chip, suspended == 0);

	rc = smbchg_usb_suspend(chip, suspended != 0);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't set usb suspend: %d rc = %d\n",
			suspended, rc);
		goto out;
	}

	pr_smb(PR_STATUS, "usb charging %s, suspended = %02x\n",
			suspended == 0 ? "enabled"
			: "suspended", suspended);
out:
	chip->usb_suspended = suspended;
	mutex_unlock(&chip->usb_en_lock);
	return rc;
}

static int smbchg_dc_en(struct smbchg_chip *chip, bool enable,
		enum enable_reason reason)
{
	int rc = 0, suspended;

	pr_smb(PR_STATUS, "dc %s, susp = %02x, en? = %d, reason = %02x\n",
			chip->dc_suspended == 0 ? "enabled"
			: "suspended", chip->dc_suspended, enable, reason);
	mutex_lock(&chip->dc_en_lock);
	if (!enable)
		suspended = chip->dc_suspended | reason;
	else
		suspended = chip->dc_suspended & ~reason;

	/* avoid unnecessary spmi interactions if nothing changed */
	if (!!suspended == !!chip->dc_suspended)
		goto out;

	rc = smbchg_dc_suspend(chip, suspended != 0);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't set dc suspend: %d rc = %d\n",
			suspended, rc);
		goto out;
	}

	if (chip->psy_registered)
		power_supply_changed(&chip->dc_psy);
	pr_smb(PR_STATUS, "dc charging %s, suspended = %02x\n",
			suspended == 0 ? "enabled"
			: "suspended", suspended);
out:
	chip->dc_suspended = suspended;
	mutex_unlock(&chip->dc_en_lock);
	return rc;
}

#define CHGPTH_CFG		0xF4
#define CFG_USB_2_3_SEL_BIT	BIT(7)
#define CFG_USB_2		0
#define CFG_USB_3		BIT(7)
#define USBIN_INPUT_MASK	SMB_MASK(4, 0)
#define USBIN_MODE_CHG_BIT	BIT(0)
#define USBIN_LIMITED_MODE	0
#define USBIN_HC_MODE		BIT(0)
#define USB51_MODE_BIT		BIT(1)
#define USB51_100MA		0
#define USB51_500MA		BIT(1)
static int smbchg_set_high_usb_chg_current(struct smbchg_chip *chip,
							int current_ma)
{
	int i, rc;
	u8 usb_cur_val;

	for (i = ARRAY_SIZE(usb_current_table) - 1; i >= 0; i--) {
		if (current_ma >= usb_current_table[i])
			break;
	}
	if (i < 0) {
		dev_err(chip->dev,
			"Cannot find %dma current_table using %d\n",
			current_ma, CURRENT_150_MA);

		rc = smbchg_sec_masked_write(chip,
					chip->usb_chgpth_base + CHGPTH_CFG,
					CFG_USB_2_3_SEL_BIT, CFG_USB_3);
		rc |= smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL,
					USBIN_MODE_CHG_BIT | USB51_MODE_BIT,
					USBIN_LIMITED_MODE | USB51_100MA);
		if (rc < 0)
			dev_err(chip->dev, "Couldn't set %dmA rc=%d\n",
					CURRENT_150_MA, rc);
		else
			chip->usb_max_current_ma = 150;
		return rc;
	}

	usb_cur_val = i & USBIN_INPUT_MASK;
	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + IL_CFG,
				USBIN_INPUT_MASK, usb_cur_val);
	if (rc < 0) {
		dev_err(chip->dev, "cannot write to config c rc = %d\n", rc);
		return rc;
	}

	rc = smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL,
				USBIN_MODE_CHG_BIT, USBIN_HC_MODE);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't write cfg 5 rc = %d\n", rc);
	chip->usb_max_current_ma = usb_current_table[i];
	return rc;
}

/* if APSD results are used
 *	if SDP is detected it will look at 500mA setting
 *		if set it will draw 500mA
 *		if unset it will draw 100mA
 *	if CDP/DCP it will look at 0x0C setting
 *		i.e. values in 0x41[1, 0] does not matter
 */
static int smbchg_set_usb_current_max(struct smbchg_chip *chip,
							int current_ma)
{
	int rc;
	static bool first_time = true;

	if (!chip->batt_present) {
		pr_info_ratelimited("Ignoring usb current->%d, battery is absent\n",
				current_ma);
		return 0;
	}

	if(is_limit_IUSB && pre_usb_max_current_ma == USB_MA_1000
		&& current_ma > USB_MA_1000){
		pr_smb(PR_STATUS, "Charger is bad, force limit current %d -> 1000 mA\n",current_ma);
		current_ma = USB_MA_1000;
	}

	/* not to check for the first time */
	if ((first_time == false) &&
			(pre_current_ma == current_ma)) {
		/* Skip if the setting is the same */
		return 0;
	}
	first_time = false;
	pre_current_ma = current_ma;

	pr_smb(PR_STATUS, "USB current_ma = %d\n", current_ma);

	if (current_ma == SUSPEND_CURRENT_MA) {
		/* suspend the usb if current set to 2mA */
		rc = smbchg_usb_en(chip, false, REASON_USB);
		chip->usb_max_current_ma = 0;
		goto out;
	} else {
		rc = smbchg_usb_en(chip, true, REASON_USB);
	}

	if (chip->low_icl_wa_on) {
		chip->usb_max_current_ma = current_ma;
		pr_smb(PR_STATUS,
			"low_icl_wa on, ignoring the usb current setting\n");
		goto out;
	}
	if (current_ma < CURRENT_150_MA) {
		/* force 100mA */
		rc = smbchg_sec_masked_write(chip,
					chip->usb_chgpth_base + CHGPTH_CFG,
					CFG_USB_2_3_SEL_BIT, CFG_USB_2);
		rc |= smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL,
					USBIN_MODE_CHG_BIT | USB51_MODE_BIT,
					USBIN_LIMITED_MODE | USB51_100MA);
		chip->usb_max_current_ma = 100;
		goto out;
	}
	/* specific current values */
	if (current_ma == CURRENT_150_MA) {
		rc = smbchg_sec_masked_write(chip,
					chip->usb_chgpth_base + CHGPTH_CFG,
					CFG_USB_2_3_SEL_BIT, CFG_USB_3);
		rc |= smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL,
					USBIN_MODE_CHG_BIT | USB51_MODE_BIT,
					USBIN_LIMITED_MODE | USB51_100MA);
		chip->usb_max_current_ma = 150;
		goto out;
	}
	if (current_ma == CURRENT_500_MA) {
		rc = smbchg_sec_masked_write(chip,
					chip->usb_chgpth_base + CHGPTH_CFG,
					CFG_USB_2_3_SEL_BIT, CFG_USB_2);
		rc |= smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL,
					USBIN_MODE_CHG_BIT | USB51_MODE_BIT,
					USBIN_LIMITED_MODE | USB51_500MA);
		chip->usb_max_current_ma = 500;
		goto out;
	}
	if (current_ma == CURRENT_900_MA) {
		rc = smbchg_sec_masked_write(chip,
					chip->usb_chgpth_base + CHGPTH_CFG,
					CFG_USB_2_3_SEL_BIT, CFG_USB_3);
		rc |= smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL,
					USBIN_MODE_CHG_BIT | USB51_MODE_BIT,
					USBIN_LIMITED_MODE | USB51_500MA);
		chip->usb_max_current_ma = 900;
		goto out;
	}

	rc = smbchg_set_high_usb_chg_current(chip, current_ma);
out:
	pr_smb(PR_STATUS, "usb current set to %d mA\n",
			chip->usb_max_current_ma);
	if (rc < 0)
		dev_err(chip->dev,
			"Couldn't set %dmA rc = %d\n", current_ma, rc);

	if(chip->usb_max_current_ma > pre_usb_max_current_ma) {
		if (delayed_work_pending(&chip->retry_aicl_work))
			cancel_delayed_work_sync(&chip->retry_aicl_work);
		pr_smb(PR_STATUS, "Trigger re-do AICL when usb current goes higher, USB_MAX changed from %d mA -> %d mA\n",
				pre_usb_max_current_ma, current_ma);
		/*Re-Run AICL */
		smbchg_rerun_aicl(chip);
	}
	pre_usb_max_current_ma = chip->usb_max_current_ma;

	return rc;
}

#define FCC_CFG			0xF2
#define FCC_500MA_VAL		0x4
#define FCC_MASK		SMB_MASK(4, 0)
static int smbchg_set_fastchg_current_raw(struct smbchg_chip *chip,
							int current_ma)
{
	int i, rc;
	u8 cur_val;

	/* the fcc enumerations are the same as the usb currents */
	for (i = ARRAY_SIZE(usb_current_table) - 1; i >= 0; i--) {
		if (current_ma >= usb_current_table[i])
			break;
	}
	if (i < 0) {
		dev_err(chip->dev,
			"Cannot find %dma current_table using %d\n",
			current_ma, CURRENT_500_MA);

		rc = smbchg_sec_masked_write(chip, chip->chgr_base + FCC_CFG,
					FCC_MASK,
					FCC_500MA_VAL);
		if (rc < 0)
			dev_err(chip->dev, "Couldn't set %dmA rc=%d\n",
					CURRENT_500_MA, rc);
		else
			chip->fastchg_current_ma = 500;
		return rc;
	}

	cur_val = i & FCC_MASK;
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + FCC_CFG,
				FCC_MASK, cur_val);
	if (rc < 0) {
		dev_err(chip->dev, "cannot write to fcc cfg rc = %d\n", rc);
		return rc;
	}

#if (defined(CONFIG_HTC_BATT_8960))
	pr_smb(PR_STATUS, "fastcharge current set to %d, cur_ma=%d\n",
			usb_current_table[i], current_ma);
#else
	chip->fastchg_current_ma = usb_current_table[i];
	pr_smb(PR_STATUS, "fastcharge current set to %d\n",
			chip->fastchg_current_ma);
#endif
	return rc;
}

static int smbchg_set_fastchg_current(struct smbchg_chip *chip,
							int current_ma)
{
	int rc;

	if (chip->sw_esr_pulse_en)
		current_ma = 300;
	rc = smbchg_set_fastchg_current_raw(chip, current_ma);
	return rc;
}

static int smbchg_parallel_usb_charging_en(struct smbchg_chip *chip, bool en)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);
	union power_supply_propval pval = {0, };

	if (!parallel_psy)
		return 0;

	pval.intval = en;
	return parallel_psy->set_property(parallel_psy,
		POWER_SUPPLY_PROP_CHARGING_ENABLED, &pval);
}

static int smbchg_sw_esr_pulse_en(struct smbchg_chip *chip, bool en)
{
	int rc;

	chip->sw_esr_pulse_en = en;
	rc = smbchg_set_fastchg_current_raw(chip,
			chip->target_fastchg_current_ma);
	if (rc)
		return rc;
	rc = smbchg_parallel_usb_charging_en(chip, !en);
	return rc;
}

static void smbchg_set_appropriate_battery_current(struct smbchg_chip *chip)
{
	/* fastchg_current_ma means IBAT_MAX */
	unsigned int chg_current = chip->fastchg_current_ma;
	unsigned long flags;

	spin_lock_irqsave(&set_current_lock, flags);

	if (chg_limit_current != 0)
		chg_current = min(chg_current, chg_limit_current);

	pr_info("setting fastchg current (IBAT_MAX): %dmA\n", chg_current);
	smbchg_set_fastchg_current(chip, chg_current);
	spin_unlock_irqrestore(&set_current_lock, flags);
}

static int smbchg_get_aicl_level_ma(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg,
			chip->usb_chgpth_base + ICL_STS_1_REG, 1);
	if (rc) {
		dev_err(chip->dev, "Could not read usb icl sts 1: %d\n", rc);
		return 0;
	}
	reg &= ICL_STS_MASK;
	if (reg >= ARRAY_SIZE(usb_current_table)) {
		pr_warn("invalid AICL value: %02x\n", reg);
		return 0;
	}
	return usb_current_table[reg];
}

#define REVISION1_REG			0x0
#define DIG_MINOR			0
#define DIG_MAJOR			1
#define ANA_MINOR			2
#define ANA_MAJOR			3
static int smbchg_low_icl_wa_check(struct smbchg_chip *chip)
{
	int rc = 0;
	bool enable = (get_prop_batt_status(chip)
		!= POWER_SUPPLY_STATUS_CHARGING);

	/* only execute workaround if the charger is version 1.x */
	if (chip->revision[DIG_MAJOR] > 1)
		return 0;

	mutex_lock(&chip->current_change_lock);
	pr_smb(PR_STATUS, "low icl %s -> %s\n",
			chip->low_icl_wa_on ? "on" : "off",
			enable ? "on" : "off");
	if (enable == chip->low_icl_wa_on)
		goto out;

	chip->low_icl_wa_on = enable;
	if (enable) {
		rc = smbchg_sec_masked_write(chip,
					chip->usb_chgpth_base + CHGPTH_CFG,
					CFG_USB_2_3_SEL_BIT, CFG_USB_2);
		rc |= smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL,
					USBIN_MODE_CHG_BIT | USB51_MODE_BIT,
					USBIN_LIMITED_MODE | USB51_100MA);
		if (rc)
			dev_err(chip->dev,
				"could not set low current limit: %d\n", rc);
	} else {
		rc = smbchg_set_usb_current_max(chip, chip->usb_max_current_ma);
	}
out:
	mutex_unlock(&chip->current_change_lock);
	return rc;
}

/*
 * set the dc charge path's maximum allowed current draw
 * that may be limited by the system's thermal level
 */
static int smbchg_set_thermal_limited_dc_current_max(struct smbchg_chip *chip,
							int current_ma)
{
	current_ma = calc_thermal_limited_current(chip, current_ma);
	return smbchg_set_dc_current_max(chip, current_ma);
}

/*
 * set the usb charge path's maximum allowed current draw
 * that may be limited by the system's thermal level
 */
static int smbchg_set_thermal_limited_usb_current_max(struct smbchg_chip *chip,
							int current_ma)
{
	current_ma = calc_thermal_limited_current(chip, current_ma);
	return smbchg_set_usb_current_max(chip, current_ma);
}

enum skip_reason {
	REASON_OTG_ENABLED		= BIT(0),
	REASON_FLASH_ENABLED	= BIT(1)
};

#define CMD_CHG_REG	0x42
#define OTG_TRIM6              0xF6
#define TR_ENB_SKIP_BIT                BIT(2)
#define OTG_EN                 BIT(0)
static int smbchg_otg_pulse_skip_disable(struct smbchg_chip *chip,
				enum skip_reason reason, bool disable)
{
	int rc;
	bool disabled;

	disabled = !!chip->otg_pulse_skip_dis;
	pr_smb(PR_STATUS, "%s pulse skip, reason %d\n",
			disable ? "disabling" : "enabling", reason);
	if (disable)
		chip->otg_pulse_skip_dis |= reason;
	else
		chip->otg_pulse_skip_dis &= ~reason;
	if (disabled == !!chip->otg_pulse_skip_dis)
		return 0;
	disabled = !!chip->otg_pulse_skip_dis;

	rc = smbchg_sec_masked_write(chip, chip->otg_base + OTG_TRIM6,
			TR_ENB_SKIP_BIT, disabled ? TR_ENB_SKIP_BIT : 0);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't %s otg pulse skip rc = %d\n",
			disabled ? "disable" : "enable", rc);
		return rc;
	}
	pr_smb(PR_STATUS, "%s pulse skip\n", disabled ? "disabled" : "enabled");
	return 0;
}

int smbchg_set_otg_pulse_skip(int val)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	return smbchg_otg_pulse_skip_disable(the_chip,
			REASON_FLASH_ENABLED, val);
}

int smbchg_get_otg_pulse_skip(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	return the_chip->otg_pulse_skip_dis;
}

#define DC_AICL_CFG					0xF3
#define AICL_RERUN_MASK			(BIT(5) | BIT(4))
#define AICL_RERUN_ON			(BIT(5) | BIT(4))
#define AICL_RERUN_OFF			0
#define USB_AICL_CFG				0xF3
#define MISC_TRIM_OPT_15_8			0xF5
static int smbchg_hw_aicl_rerun_en(struct smbchg_chip *chip, bool en)
{
	int rc = 0;

	if (chip->very_weak_charger && en) {
		pr_smb(PR_STATUS, "Very weak charger, not enabling reruns");
	} else {
		rc = smbchg_sec_masked_write(chip,
			chip->misc_base + MISC_TRIM_OPT_15_8,
			AICL_RERUN_MASK, en ? AICL_RERUN_ON : AICL_RERUN_OFF);
		if (rc) {
			pr_err("Couldn't write to MISC_TRIM_OPTIONS_15_8 rc=%d\n",
					rc);
			return rc;
		}
		pr_smb(PR_STATUS, "HW AICL Reruns %s\n", en ? "on" : "off");
	}
	return rc;
}

#define DC_AICL_CFG			0xF3
#define USB_AICL_DEGLITCH_MASK		(BIT(5) | BIT(4) | BIT(3))
#define USB_AICL_DEGLITCH_SHORT		(BIT(5) | BIT(4) | BIT(3))
#define USB_AICL_DEGLITCH_LONG		0
#define DC_AICL_DEGLITCH_MASK		(BIT(5) | BIT(4) | BIT(3))
#define DC_AICL_DEGLITCH_SHORT		(BIT(5) | BIT(4) | BIT(3))
#define DC_AICL_DEGLITCH_LONG		0
#define AICL_EN_BIT				BIT(2)
static void smbchg_aicl_deglitch_wa_en(struct smbchg_chip *chip, bool en)
{
	int rc;
	pr_smb(PR_STATUS, "AICL deglitch enable:%d\n",en);

	if (en && !chip->aicl_deglitch_short) {
		rc = smbchg_sec_masked_write(chip,
			chip->usb_chgpth_base + USB_AICL_CFG,
			USB_AICL_DEGLITCH_MASK, USB_AICL_DEGLITCH_SHORT);
		if (rc) {
			pr_err("Couldn't write to USB_AICL_CFG rc=%d\n", rc);
			return;
		}
		rc = smbchg_sec_masked_write(chip,
			chip->dc_chgpth_base + DC_AICL_CFG,
			DC_AICL_DEGLITCH_MASK, DC_AICL_DEGLITCH_SHORT);
		if (rc) {
			pr_err("Couldn't write to DC_AICL_CFG rc=%d\n", rc);
			return;
		}
		if(!chip->very_weak_charger){
			rc = smbchg_hw_aicl_rerun_en(chip, true);
			if (rc) {
				pr_err("Couldn't enable AICL rerun rc= %d\n",
						rc);
				return;
			}
		}
		pr_smb(PR_STATUS, "AICL deglitch set to short\n");
	} else if (!en && chip->aicl_deglitch_short) {
		rc = smbchg_sec_masked_write(chip,
			chip->usb_chgpth_base + USB_AICL_CFG,
			USB_AICL_DEGLITCH_MASK, USB_AICL_DEGLITCH_LONG);
		if (rc) {
			pr_err("Couldn't write to USB_AICL_CFG rc=%d\n", rc);
			return;
		}
		rc = smbchg_sec_masked_write(chip,
			chip->dc_chgpth_base + DC_AICL_CFG,
			DC_AICL_DEGLITCH_MASK, DC_AICL_DEGLITCH_LONG);
		if (rc) {
			pr_err("Couldn't write to DC_AICL_CFG rc=%d\n", rc);
			return;
		}
		/*No need to disable AICL rerun*/
		/*To avoid suspend all the instances of AICL disable shoould be removed*/
		//smbchg_hw_aicl_rerun_en(chip, false);
		pr_smb(PR_STATUS, "AICL deglitch set to normal\n");
	}
	chip->aicl_deglitch_short = en;
}

#define USBIN_HVDCP_STS         0x0C
#define USBIN_HVDCP_SEL_BIT     BIT(4)
#define USBIN_HVDCP_SEL_9V_BIT  BIT(1)
void smbchg_aicl_deglitch_wa_check(void)
{
	union power_supply_propval prop = {0,};
	int rc;
	u8 reg;
	bool low_volt_chgr = true;
	pr_smb(PR_STATUS, "aicl_deglitch_wa called\n");

	if (!the_chip) {
		pr_smb(PR_STATUS,
				"smbcharger not initialized, do nothing.\n");
		return;
	}

	if (!is_usb_present(the_chip) && !is_dc_present(the_chip)) {
		pr_smb(PR_STATUS, "Charger removed\n");
		smbchg_aicl_deglitch_wa_en(the_chip, false);
		return;
	}

	if (!the_chip->bms_psy)
		return;

	if (is_usb_present(the_chip)) {
		rc = smbchg_read(the_chip, &reg,
				the_chip->usb_chgpth_base + USBIN_HVDCP_STS, 1);
		if (rc < 0) {
			pr_err("Couldn't read hvdcp status rc = %d\n", rc);
			return;
		}
		if (reg & USBIN_HVDCP_SEL_BIT)
			low_volt_chgr = false;
	} else if (is_dc_present(the_chip)) {
		if (the_chip->dc_psy_type == POWER_SUPPLY_TYPE_WIPOWER)
			low_volt_chgr = false;
		else
			low_volt_chgr = the_chip->low_volt_dcin;
	}

	if (!low_volt_chgr) {
		pr_smb(PR_STATUS, "High volt charger! Don't set deglitch\n");
		smbchg_aicl_deglitch_wa_en(the_chip, false);
		return;
	}

	/* It is possible that battery voltage went high above threshold
	 * when the charger is inserted and can go low because of system
	 * load. We shouldn't be reconfiguring AICL deglitch when this
	 * happens as it will lead to oscillation again which is being
	 * fixed here. Do it once when the battery voltage crosses the
	 * threshold (e.g. 4.2 V) and clear it only when the charger
	 * is removed.
	 */
	if (!the_chip->vbat_above_headroom) {
		rc = the_chip->bms_psy->get_property(the_chip->bms_psy,
				POWER_SUPPLY_PROP_VOLTAGE_MIN, &prop);
		if (rc < 0) {
			pr_err("could not read voltage_min, rc=%d\n", rc);
			return;
		}
		the_chip->vbat_above_headroom = !prop.intval;
	}
	smbchg_aicl_deglitch_wa_en(the_chip, the_chip->vbat_above_headroom);
}

/* WA: remove vfloat trim feature
static void smbchg_vfloat_adjust_check(struct smbchg_chip *chip)
{
	if (!chip->use_vfloat_adjustments)
		return;

	smbchg_stay_awake(chip, PM_REASON_VFLOAT_ADJUST);
	pr_smb(PR_STATUS, "Starting vfloat adjustments\n");
	schedule_delayed_work(&chip->vfloat_adjust_work, 0);
}
*/

#if 0//FIXME
static int smbchg_system_temp_level_set(struct smbchg_chip *chip,
								int lvl_sel)
{
	int rc = 0;
	int prev_therm_lvl;

	if (!chip->thermal_mitigation) {
		dev_err(chip->dev, "Thermal mitigation not supported\n");
		return -EINVAL;
	}

	if (lvl_sel < 0) {
		dev_err(chip->dev, "Unsupported level selected %d\n", lvl_sel);
		return -EINVAL;
	}

	if (lvl_sel >= chip->thermal_levels) {
		dev_err(chip->dev, "Unsupported level selected %d forcing %d\n",
				lvl_sel, chip->thermal_levels - 1);
		lvl_sel = chip->thermal_levels - 1;
	}

	if (lvl_sel == chip->therm_lvl_sel)
		return 0;

	mutex_lock(&chip->current_change_lock);
	prev_therm_lvl = chip->therm_lvl_sel;
	chip->therm_lvl_sel = lvl_sel;
	if (chip->therm_lvl_sel == (chip->thermal_levels - 1)) {
		/*
		 * Disable charging if highest value selected by
		 * setting the DC and USB path in suspend
		 */
		rc = smbchg_dc_en(chip, false, REASON_THERMAL);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set dc suspend rc %d\n", rc);
			goto out;
		}
		rc = smbchg_usb_en(chip, false, REASON_THERMAL);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set usb suspend rc %d\n", rc);
			goto out;
		}
		goto out;
	}

	rc = smbchg_set_thermal_limited_usb_current_max(chip,
					chip->usb_target_current_ma);
	rc = smbchg_set_thermal_limited_dc_current_max(chip,
					chip->dc_target_current_ma);

	if (prev_therm_lvl == chip->thermal_levels - 1) {
		/*
		 * If previously highest value was selected charging must have
		 * been disabed. Enable charging by taking the DC and USB path
		 * out of suspend.
		 */
		rc = smbchg_dc_en(chip, true, REASON_THERMAL);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set dc suspend rc %d\n", rc);
			goto out;
		}
		rc = smbchg_usb_en(chip, true, REASON_THERMAL);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set usb suspend rc %d\n", rc);
			goto out;
		}
	}
out:
	mutex_unlock(&chip->current_change_lock);
	return rc;
}

static int smbchg_battery_set_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       const union power_supply_propval *val)
{
	struct smbchg_chip *chip = container_of(psy,
				struct smbchg_chip, batt_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		smbchg_usb_en(chip, val->intval, REASON_USER);
		smbchg_dc_en(chip, val->intval, REASON_USER);
		chip->chg_enabled = val->intval;
		schedule_work(&chip->usb_set_online_work);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		chip->fake_battery_soc = val->intval;
		power_supply_changed(&chip->batt_psy);
		break;
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		smbchg_system_temp_level_set(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_FLASH_ACTIVE:
		rc = smbchg_otg_pulse_skip_disable(chip,
				REASON_FLASH_ENABLED, val->intval);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int smbchg_battery_is_writeable(struct power_supply *psy,
				       enum power_supply_property prop)
{
	int rc;

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		rc = 1;
		break;
	default:
		rc = 0;
		break;
	}
	return rc;
}

static int smbchg_battery_get_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       union power_supply_propval *val)
{
	struct smbchg_chip *chip = container_of(psy,
				struct smbchg_chip, batt_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = get_prop_batt_status(chip);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = get_prop_batt_present(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = chip->chg_enabled;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = get_prop_charge_type(chip);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = get_prop_batt_capacity(chip);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = get_prop_batt_health(chip);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = chip->fastchg_current_ma * 1000;
		break;
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		val->intval = chip->therm_lvl_sel;
		break;
	case POWER_SUPPLY_PROP_FLASH_ACTIVE:
		val->intval = chip->otg_pulse_skip_dis;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static enum power_supply_property smbchg_dc_properties[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
};

static int smbchg_dc_set_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       const union power_supply_propval *val)
{
	struct smbchg_chip *chip = container_of(psy,
				struct smbchg_chip, dc_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		return smbchg_dc_en(chip, val->intval, REASON_POWER_SUPPLY);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int smbchg_dc_get_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       union power_supply_propval *val)
{
	struct smbchg_chip *chip = container_of(psy,
				struct smbchg_chip, dc_psy);
	bool user_enabled = (chip->dc_suspended & REASON_USER) == 0;

	switch (prop) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = is_dc_present(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = chip->dc_suspended == 0;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		/* return if dc is charging the battery */
		val->intval = user_enabled && chip->dc_present;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int smbchg_dc_is_writeable(struct power_supply *psy,
				       enum power_supply_property prop)
{
	int rc;

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		rc = 1;
		break;
	default:
		rc = 0;
		break;
	}
	return rc;
}

static void smbchg_external_power_changed(struct power_supply *psy)
{
	struct smbchg_chip *chip = container_of(psy,
				struct smbchg_chip, batt_psy);
	union power_supply_propval prop = {0,};
	int rc, current_limit = 0;

	if (chip->bms_psy_name)
		chip->bms_psy =
			power_supply_get_by_name((char *)chip->bms_psy_name);

	rc = chip->usb_psy->get_property(chip->usb_psy,
				POWER_SUPPLY_PROP_CHARGING_ENABLED, &prop);
	if (rc < 0)
		pr_smb(PR_STATUS, "could not read USB charge_en, rc=%d\n",
				rc);
	else
		smbchg_usb_en(chip, prop.intval, REASON_POWER_SUPPLY);

	rc = chip->usb_psy->get_property(chip->usb_psy,
				POWER_SUPPLY_PROP_CURRENT_MAX, &prop);
	if (rc < 0)
		dev_err(chip->dev,
			"could not read USB current_max property, rc=%d\n", rc);
	else
		current_limit = prop.intval / 1000;
	pr_smb(PR_STATUS, "current_limit = %d\n", current_limit);

	mutex_lock(&chip->current_change_lock);
	chip->usb_target_current_ma = current_limit;
	rc = smbchg_set_thermal_limited_usb_current_max(chip, current_limit);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set usb current rc = %d\n", rc);
	mutex_unlock(&chip->current_change_lock);

	power_supply_changed(&chip->batt_psy);
}
#endif

#define FV_STS_REG			0xC
#define AICL_INPUT_STS_BIT		BIT(6)
static bool smbchg_is_input_current_limited(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->chgr_base + FV_STS_REG, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read FV_STS rc=%d\n", rc);
		return false;
	}

	return !!(reg & AICL_INPUT_STS_BIT);
}

#define SW_ESR_PULSE_MS			1500
static void smbchg_cc_esr_wa_check(struct smbchg_chip *chip)
{
	int rc, esr_count;

	if (!is_usb_present(chip) && !is_dc_present(chip)) {
		pr_smb(PR_STATUS, "No inputs present, skipping\n");
		return;
	}

	if (get_prop_charge_type(chip) != POWER_SUPPLY_CHARGE_TYPE_FAST) {
		pr_smb(PR_STATUS, "Not in fast charge, skipping\n");
		return;
	}

	if (!smbchg_is_input_current_limited(chip)) {
		pr_smb(PR_STATUS, "Not input current limited, skipping\n");
		return;
	}

	set_property_on_fg(chip, POWER_SUPPLY_PROP_UPDATE_NOW, 1);
	rc = get_property_from_fg(chip,
			POWER_SUPPLY_PROP_ESR_COUNT, &esr_count);
	if (rc) {
		pr_smb(PR_STATUS,
			"could not read ESR counter rc = %d\n", rc);
		return;
	}

	/*
	 * The esr_count is counting down the number of fuel gauge cycles
	 * before a ESR pulse is needed.
	 *
	 * After a successful ESR pulse, this count is reset to some
	 * high number like 28. If this reaches 0, then the fuel gauge
	 * hardware should force a ESR pulse.
	 *
	 * However, if the device is in constant current charge mode while
	 * being input current limited, the ESR pulse will not affect the
	 * battery current, so the measurement will fail.
	 *
	 * As a failsafe, force a manual ESR pulse if this value is read as
	 * 0.
	 */
	if (esr_count != 0) {
		pr_smb(PR_STATUS, "ESR count is not zero, skipping\n");
		return;
	}

	pr_smb(PR_STATUS, "Lowering charge current for ESR pulse\n");
	smbchg_stay_awake(chip, PM_ESR_PULSE);
	smbchg_sw_esr_pulse_en(chip, true);
	msleep(SW_ESR_PULSE_MS);
	pr_smb(PR_STATUS, "Raising charge current for ESR pulse\n");
	smbchg_relax(chip, PM_ESR_PULSE);
	smbchg_sw_esr_pulse_en(chip, false);
}

static void smbchg_soc_changed(struct smbchg_chip *chip)
{
	smbchg_cc_esr_wa_check(chip);
}

#define UCONV			1000000LL
#define VIN_FLASH_UV		5500000LL
#define FLASH_V_THRESHOLD	3000000LL
#define BUCK_EFFICIENCY		800LL
static int smbchg_calc_max_flash_current(struct smbchg_chip *chip)
{
	union power_supply_propval ret = {0, };
	int ocv_uv, ibat_ua, esr_uohm, rbatt_uohm, rc;
	int64_t ibat_flash_ua, total_flash_ua, total_flash_power_fw;

	if (!chip->bms_psy && chip->bms_psy_name)
		chip->bms_psy =
			power_supply_get_by_name((char *)chip->bms_psy_name);
	/* if bms psy is not found, return 0 uA (no flash available) */
	if (!chip->bms_psy) {
		pr_smb(PR_STATUS, "no bms psy found\n");
		return 0;
	}

	rc = chip->bms_psy->get_property(chip->bms_psy,
			POWER_SUPPLY_PROP_VOLTAGE_OCV, &ret);
	if (rc) {
		pr_smb(PR_STATUS, "bms psy does not support OCV\n");
		return 0;
	}
	ocv_uv = ret.intval;

	rc = chip->bms_psy->get_property(chip->bms_psy,
			POWER_SUPPLY_PROP_CURRENT_NOW, &ret);
	if (rc) {
		pr_smb(PR_STATUS, "bms psy does not support current_now\n");
		return 0;
	}
	ibat_ua = ret.intval;

	rc = chip->bms_psy->get_property(chip->bms_psy,
			POWER_SUPPLY_PROP_RESISTANCE, &ret);
	if (rc) {
		pr_smb(PR_STATUS, "bms psy does not support resistance\n");
		return 0;
	}
	esr_uohm = ret.intval;

	rbatt_uohm = esr_uohm + chip->rpara_uohm + chip->rslow_uohm;
	ibat_flash_ua = (div_s64((ocv_uv - FLASH_V_THRESHOLD) * UCONV,
			rbatt_uohm)) - ibat_ua;
	total_flash_power_fw = FLASH_V_THRESHOLD * ibat_flash_ua
			* BUCK_EFFICIENCY;
	total_flash_ua = div64_s64(total_flash_power_fw, VIN_FLASH_UV * 1000LL);
	pr_smb(PR_STATUS,
		"ibat_flash=%lld\n, ocv=%d, ibat=%d, rbatt=%d t_flash=%lld\n",
		ibat_flash_ua, ocv_uv, ibat_ua, rbatt_uohm, total_flash_ua);
	return (int)total_flash_ua;
}

int pmi8994_calc_max_flash_current(void)
{
	if(!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	return smbchg_calc_max_flash_current(the_chip);
}

static void smbchg_check_and_notify_fg_soc(struct smbchg_chip *chip)
{
	bool charger_present, charger_suspended;

	charger_present = is_usb_present(chip) | is_dc_present(chip);
	charger_suspended = chip->usb_suspended & chip->dc_suspended;

	if (charger_present && !charger_suspended
		&& get_prop_batt_status(chip)
			== POWER_SUPPLY_STATUS_CHARGING) {
		pr_smb(PR_STATUS, "Adjusting battery soc in FG\n");
		set_property_on_fg(chip, POWER_SUPPLY_PROP_STATUS,
								POWER_SUPPLY_PROP_CHARGE_FULL);
	}
}

static int smbchg_get_prop_voltage_now(struct smbchg_chip *chip)
{
	union power_supply_propval ret = {0, };

	if (!chip->bms_psy && chip->bms_psy_name)
		chip->bms_psy =
			power_supply_get_by_name((char *)chip->bms_psy_name);
	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_VOLTAGE_NOW, &ret);
		return ret.intval;
	} else {
		pr_smb(PR_STATUS, "bms_psy doesn't exist!\n");
	}

	return 0;
}

static int smbchg_get_prop_current_now(struct smbchg_chip *chip)
{
	union power_supply_propval ret = {0, };

	if (!chip->bms_psy && chip->bms_psy_name)
		chip->bms_psy =
			power_supply_get_by_name((char *)chip->bms_psy_name);
	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_CURRENT_NOW, &ret);
		return ret.intval;
	} else {
		pr_smb(PR_STATUS, "bms_psy doesn't exist!\n");
	}
	return 0;
}

static int smbchg_get_prop_capacity_now(struct smbchg_chip *chip)
{
	union power_supply_propval ret = {0, };
	static int fg_notified;

	if (!chip->bms_psy && chip->bms_psy_name)
		chip->bms_psy =
			power_supply_get_by_name((char *)chip->bms_psy_name);
	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_CAPACITY, &ret);
		if ((ret.intval == 100) && (!fg_notified) && (get_prop_charge_type(chip)
							== POWER_SUPPLY_CHARGE_TYPE_TAPER
						|| get_prop_batt_status(chip)
							== POWER_SUPPLY_STATUS_FULL)) {
			fg_notified = 1;
			smbchg_check_and_notify_fg_soc(chip);
		} else
			fg_notified = 0;

		return ret.intval;
	} else {
		pr_smb(PR_STATUS, "bms_psy doesn't exist!\n");
	}
	return 0;
}

static int smbchg_get_prop_batt_temp_now(struct smbchg_chip *chip)
{
	union power_supply_propval ret = {0, };

	if (!chip->bms_psy && chip->bms_psy_name)
		chip->bms_psy =
			power_supply_get_by_name((char *)chip->bms_psy_name);
	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_TEMP, &ret);
		if ((ret.intval >= 650) &&
				(flag_keep_charge_on || flag_pa_fake_batt_temp))
			ret.intval = 650;
		return ret.intval;
	} else {
		pr_smb(PR_STATUS, "bms_psy doesn't exist!\n");
	}
	return 0;
}

#define VFLOAT_CFG_REG			0xF4
#define VFLOAT_CMP_CFG_REG			0xF5
#define MIN_FLOAT_MV			3600
#define MAX_FLOAT_MV			4500
#define VFLOAT_MASK			SMB_MASK(5, 0)

#define MID_RANGE_FLOAT_MV_MIN		3600
#define MID_RANGE_FLOAT_MIN_VAL		0x05
#define MID_RANGE_FLOAT_STEP_MV		20

#define HIGH_RANGE_FLOAT_MIN_MV		4340
#define HIGH_RANGE_FLOAT_MIN_VAL	0x2A
#define HIGH_RANGE_FLOAT_STEP_MV	10

#define VHIGH_RANGE_FLOAT_MIN_MV	4360
#define VHIGH_RANGE_FLOAT_MIN_VAL	0x2C
#define VHIGH_RANGE_FLOAT_STEP_MV	20
static int smbchg_float_voltage_set(struct smbchg_chip *chip, int vfloat_mv)
{
	u8 temp;

	if ((vfloat_mv < MIN_FLOAT_MV) || (vfloat_mv > MAX_FLOAT_MV)) {
		dev_err(chip->dev, "bad float voltage mv =%d asked to set\n",
					vfloat_mv);
		return -EINVAL;
	}

	if (vfloat_mv <= HIGH_RANGE_FLOAT_MIN_MV) {
		/* mid range */
		temp = MID_RANGE_FLOAT_MIN_VAL
			+ (vfloat_mv - MID_RANGE_FLOAT_MV_MIN)
				/ MID_RANGE_FLOAT_STEP_MV;
	} else if (vfloat_mv <= VHIGH_RANGE_FLOAT_MIN_MV) {
		/* high range */
		temp = HIGH_RANGE_FLOAT_MIN_VAL
			+ (vfloat_mv - HIGH_RANGE_FLOAT_MIN_MV)
				/ HIGH_RANGE_FLOAT_STEP_MV;
	} else {
		/* very high range */
		temp = VHIGH_RANGE_FLOAT_MIN_VAL
			+ (vfloat_mv - VHIGH_RANGE_FLOAT_MIN_MV)
				/ VHIGH_RANGE_FLOAT_STEP_MV;
	}

	return smbchg_sec_masked_write(chip, chip->chgr_base + VFLOAT_CFG_REG,
			VFLOAT_MASK, temp);
}

static int smbchg_float_voltage_cmp_set(struct smbchg_chip *chip, int vfloat_cmp_mv)
{
	u8 temp;

	if ((vfloat_cmp_mv < MIN_FLOAT_MV) || (vfloat_cmp_mv > MAX_FLOAT_MV)) {
		pr_err("bad float comp voltage mv=%d asked to set\n",
					vfloat_cmp_mv);
		return -EINVAL;
	}

	if (vfloat_cmp_mv > chip->vfloat_mv) {
		pr_err("bad float_comp_mv=%d > float_mv=%d, skip setting\n",
                                        vfloat_cmp_mv, chip->vfloat_mv);
		return 0;
	}

	/* vfloat_comp_mv must be smaller than chip->vfloat_mv */
	if (vfloat_cmp_mv <= HIGH_RANGE_FLOAT_MIN_MV
			&& chip->vfloat_mv >= VHIGH_RANGE_FLOAT_MIN_MV) {
		temp = ((HIGH_RANGE_FLOAT_MIN_MV - vfloat_cmp_mv)
			+ (chip->vfloat_mv - VHIGH_RANGE_FLOAT_MIN_MV))
				/ MID_RANGE_FLOAT_STEP_MV
			+ (VHIGH_RANGE_FLOAT_MIN_MV - HIGH_RANGE_FLOAT_MIN_MV)
				/ HIGH_RANGE_FLOAT_STEP_MV;
	} else if (vfloat_cmp_mv >= HIGH_RANGE_FLOAT_MIN_MV
			&& chip->vfloat_mv <= VHIGH_RANGE_FLOAT_MIN_MV) {
		temp = (chip->vfloat_mv - vfloat_cmp_mv)
				/ HIGH_RANGE_FLOAT_STEP_MV;
	} else if (vfloat_cmp_mv <= HIGH_RANGE_FLOAT_MIN_MV
			&& chip->vfloat_mv > HIGH_RANGE_FLOAT_MIN_MV) {
		temp = (HIGH_RANGE_FLOAT_MIN_MV - vfloat_cmp_mv)
				/ MID_RANGE_FLOAT_STEP_MV
			+ (chip->vfloat_mv - HIGH_RANGE_FLOAT_MIN_MV)
				/ HIGH_RANGE_FLOAT_STEP_MV;
	} else if (vfloat_cmp_mv <= VHIGH_RANGE_FLOAT_MIN_MV
			&& chip->vfloat_mv > VHIGH_RANGE_FLOAT_MIN_MV) {
		temp = (VHIGH_RANGE_FLOAT_MIN_MV - vfloat_cmp_mv)
				/ HIGH_RANGE_FLOAT_STEP_MV
			+ (chip->vfloat_mv - VHIGH_RANGE_FLOAT_MIN_MV)
				/ VHIGH_RANGE_FLOAT_STEP_MV;
	} else {
		temp = (chip->vfloat_mv - vfloat_cmp_mv)
				/ VHIGH_RANGE_FLOAT_STEP_MV;
	}

	return smbchg_sec_masked_write(chip, chip->chgr_base + VFLOAT_CMP_CFG_REG,
			VFLOAT_MASK, temp);
}

#define FCC_CMP_CFG			0xF3
#define FCC_CMP			SMB_MASK(1, 0)
#define FCC_COMP_250MA			0x0
#define FCC_COMP_700MA			0x1
#define FCC_COMP_900MA			0x2
#define FCC_COMP_1200MA			0x3
static int smbchg_warm_ibatmax_set(struct smbchg_chip *chip, int warm_ibatmax)
{
	u8 reg;

	if (warm_ibatmax >= 1200)
		reg = FCC_COMP_1200MA;
	else if (warm_ibatmax >= 900)
		reg = FCC_COMP_900MA;
	else if (warm_ibatmax >= 700)
		reg = FCC_COMP_700MA;
	else
		reg = FCC_COMP_250MA;

	return smbchg_sec_masked_write(chip,
			chip->chgr_base + FCC_CMP_CFG,
			FCC_CMP, reg);
}

static int smbchg_otg_regulator_enable(struct regulator_dev *rdev)
{
	int rc = 0;
	struct smbchg_chip *chip = rdev_get_drvdata(rdev);

	chip->otg_retries = 0;
	smbchg_otg_pulse_skip_disable(chip, REASON_OTG_ENABLED, true);
	/* sleep to make sure the pulse skip is actually disabled */
	msleep(20);
	rc = smbchg_masked_write(chip, chip->bat_if_base + CMD_CHG_REG,
			OTG_EN, OTG_EN);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't enable OTG mode rc=%d\n", rc);
	else
		chip->otg_enable_time = ktime_get();
	pr_smb(PR_STATUS, "Enabling OTG Boost\n");
	return rc;
}

static int smbchg_otg_regulator_disable(struct regulator_dev *rdev)
{
	int rc = 0;
	struct smbchg_chip *chip = rdev_get_drvdata(rdev);

	rc = smbchg_masked_write(chip, chip->bat_if_base + CMD_CHG_REG,
			OTG_EN, 0);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't disable OTG mode rc=%d\n", rc);
	smbchg_otg_pulse_skip_disable(chip, REASON_OTG_ENABLED, false);
	pr_smb(PR_STATUS, "Disabling OTG Boost\n");
	return rc;
}

static int smbchg_otg_regulator_is_enable(struct regulator_dev *rdev)
{
	int rc = 0;
	u8 reg = 0;
	struct smbchg_chip *chip = rdev_get_drvdata(rdev);

	rc = smbchg_read(chip, &reg, chip->bat_if_base + CMD_CHG_REG, 1);
	if (rc < 0) {
		dev_err(chip->dev,
				"Couldn't read OTG enable bit rc=%d\n", rc);
		return rc;
	}

	return (reg & OTG_EN) ? 1 : 0;
}

struct regulator_ops smbchg_otg_reg_ops = {
	.enable		= smbchg_otg_regulator_enable,
	.disable	= smbchg_otg_regulator_disable,
	.is_enabled	= smbchg_otg_regulator_is_enable,
};

static int smbchg_regulator_init(struct smbchg_chip *chip)
{
	int rc = 0;
	struct regulator_init_data *init_data;
	struct regulator_config cfg = {};

	init_data = of_get_regulator_init_data(chip->dev, chip->dev->of_node);
	if (!init_data) {
		dev_err(chip->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	if (init_data->constraints.name) {
		chip->otg_vreg.rdesc.owner = THIS_MODULE;
		chip->otg_vreg.rdesc.type = REGULATOR_VOLTAGE;
		chip->otg_vreg.rdesc.ops = &smbchg_otg_reg_ops;
		chip->otg_vreg.rdesc.name = init_data->constraints.name;

		cfg.dev = chip->dev;
		cfg.init_data = init_data;
		cfg.driver_data = chip;
		cfg.of_node = chip->dev->of_node;

		init_data->constraints.valid_ops_mask
			|= REGULATOR_CHANGE_STATUS;

		chip->otg_vreg.rdev = regulator_register(
						&chip->otg_vreg.rdesc, &cfg);
		if (IS_ERR(chip->otg_vreg.rdev)) {
			rc = PTR_ERR(chip->otg_vreg.rdev);
			chip->otg_vreg.rdev = NULL;
			if (rc != -EPROBE_DEFER)
				dev_err(chip->dev,
					"OTG reg failed, rc=%d\n", rc);
		}
	}

	return rc;
}

static void smbchg_regulator_deinit(struct smbchg_chip *chip)
{
	if (chip->otg_vreg.rdev)
		regulator_unregister(chip->otg_vreg.rdev);
}

static void smbchg_load_jeita_setting(struct smbchg_chip *chip)
{
	union power_supply_propval ret = {0, };

	if (!chip->bms_psy && chip->bms_psy_name)
		chip->bms_psy =
				power_supply_get_by_name((char *)chip->bms_psy_name);
	if (!chip->bms_psy) {
		pr_smb(PR_STATUS, "bms_psy doesn't exist!\n");
		return;
	}

	chip->bms_psy->get_property(chip->bms_psy,
			POWER_SUPPLY_PROP_WARM_TEMP, &ret);
	chip->warm_bat_decidegc = ret.intval;

	chip->bms_psy->get_property(chip->bms_psy,
			POWER_SUPPLY_PROP_COOL_TEMP, &ret);
	chip->cool_bat_decidegc = ret.intval;

	chip->bms_psy->get_property(chip->bms_psy,
			POWER_SUPPLY_PROP_HOT_TEMP, &ret);
	chip->hot_bat_decidegc = ret.intval;

	chip->bms_psy->get_property(chip->bms_psy,
			POWER_SUPPLY_PROP_COLD_TEMP, &ret);
	chip->cold_bat_decidegc = ret.intval;

	pr_smb(PR_STATUS, "cool_bat_dec=%d, warm_bat_dec=%d, "
					"cold_bat_dec=%d, hot_bat_dec=%d\n",
			chip->cool_bat_decidegc, chip->warm_bat_decidegc,
			chip->cold_bat_decidegc, chip->hot_bat_decidegc);

}

static void smbchg_adc_notification(enum qpnp_tm_state_smbchg state, void *ctx)
{
	union power_supply_propval ret = {0, };
	struct smbchg_chip *chip = ctx;
	int temp;

	if (state >= ADC_TM_STATE_NUM_SMBCHG) {
		pr_smb(PR_STATUS, "invalid notification %d\n", state);
		return;
	}

	/* Load JEITA threshold if it wasn't loaded yet */
	if (!chip->warm_bat_decidegc)
		smbchg_load_jeita_setting(chip);

	temp = smbchg_get_prop_batt_temp_now(chip);

	if (!chip->bms_psy) {
		pr_smb(PR_STATUS, "bms_psy doesn't exist!\n");
	} else {
		if (state == ADC_TM_HOT_STATE_SMBCHG) {
			if (chip->batt_hot) {
				/* warm to hot */
				ret.intval = chip->hot_bat_decidegc - HYSTERISIS_DECIDEGC;
				chip->bms_psy->set_property(chip->bms_psy,
						POWER_SUPPLY_PROP_HOT_TEMP, &ret);

			} else {
				/* hot to warm */
				ret.intval = chip->hot_bat_decidegc;
				chip->bms_psy->set_property(chip->bms_psy,
						POWER_SUPPLY_PROP_HOT_TEMP, &ret);
			}
		} else if (state == ADC_TM_WARM_STATE_SMBCHG) {
			if (chip->batt_warm) {
				/* normal to warm */
				ret.intval = chip->warm_bat_decidegc - HYSTERISIS_DECIDEGC;
				chip->bms_psy->set_property(chip->bms_psy,
						POWER_SUPPLY_PROP_WARM_TEMP, &ret);
			} else {
				/* warm to normal */
				ret.intval = chip->warm_bat_decidegc;
				chip->bms_psy->set_property(chip->bms_psy,
						POWER_SUPPLY_PROP_WARM_TEMP, &ret);
			}
		} else if (state == ADC_TM_COOL_STATE_SMBCHG) {
			if (chip->batt_cool) {
				/* normal to cool */
				ret.intval = chip->cool_bat_decidegc + HYSTERISIS_DECIDEGC;
				chip->bms_psy->set_property(chip->bms_psy,
						POWER_SUPPLY_PROP_COOL_TEMP, &ret);

			} else {
				/* cool to normal */
				ret.intval = chip->cool_bat_decidegc;
				chip->bms_psy->set_property(chip->bms_psy,
						POWER_SUPPLY_PROP_COOL_TEMP, &ret);
			}
		} else if (state == ADC_TM_COLD_STATE_SMBCHG) {
			if (chip->batt_cold) {
				/* cool to cold */
				ret.intval = chip->cold_bat_decidegc + HYSTERISIS_DECIDEGC;
				chip->bms_psy->set_property(chip->bms_psy,
						POWER_SUPPLY_PROP_COLD_TEMP, &ret);

			} else {
				/* cold to cool */
				ret.intval = chip->cold_bat_decidegc;
				chip->bms_psy->set_property(chip->bms_psy,
						POWER_SUPPLY_PROP_COLD_TEMP, &ret);
			}
		}
	}
	pr_smb(PR_STATUS, "ret.intval=%d, batt_warm=%d, batt_cool=%d, batt_hot=%d, "
					"bat_cold=%d\n",
					ret.intval, chip->batt_warm, chip->batt_cool, chip->batt_hot,
					chip->batt_cold);
	htc_gauge_event_notify(HTC_GAUGE_EVENT_TEMP_ZONE_CHANGE);

}

static void smbchg_notify_fg_chg_status(struct smbchg_chip *chip)
{
       if (get_prop_batt_status(chip) == POWER_SUPPLY_STATUS_CHARGING) {
               pr_smb(PR_STATUS, "notifying FG on charging\n");
               set_property_on_fg(chip, POWER_SUPPLY_PROP_STATUS,
                               POWER_SUPPLY_STATUS_CHARGING);
       } else if (get_prop_batt_status(chip) ==
                       POWER_SUPPLY_STATUS_DISCHARGING) {
               pr_smb(PR_STATUS, "notifying FG on discharging\n");
               set_property_on_fg(chip, POWER_SUPPLY_PROP_STATUS,
                               POWER_SUPPLY_STATUS_DISCHARGING);
       }
}

static void smbchg_notify_fg_work(struct work_struct *work)
{
       struct smbchg_chip *chip = container_of(work,
                               struct smbchg_chip,
                               notify_fg_work);

       smbchg_notify_fg_chg_status(chip);
}

static int vf_adjust_low_threshold = 5;
module_param(vf_adjust_low_threshold, int, 0644);

static int vf_adjust_high_threshold = 7;
module_param(vf_adjust_high_threshold, int, 0644);

static int vf_adjust_n_samples = 10;
module_param(vf_adjust_n_samples, int, 0644);

static int vf_adjust_max_delta_mv = 40;
module_param(vf_adjust_max_delta_mv, int, 0644);

static int vf_adjust_trim_steps_per_adjust = 1;
module_param(vf_adjust_trim_steps_per_adjust, int, 0644);

#define CENTER_TRIM_CODE		7
#define MAX_LIN_CODE			14
#define MAX_TRIM_CODE			15
#define SCALE_SHIFT			4
#define VF_TRIM_OFFSET_MASK		SMB_MASK(3, 0)
#define VF_STEP_SIZE_MV			10
#define SCALE_LSB_MV			17
static int smbchg_trim_add_steps(int prev_trim, int delta_steps)
{
	int scale_steps;
	int linear_offset, linear_scale;
	int offset_code = prev_trim & VF_TRIM_OFFSET_MASK;
	int scale_code = (prev_trim & ~VF_TRIM_OFFSET_MASK) >> SCALE_SHIFT;

	if (abs(delta_steps) > 1) {
		pr_smb(PR_STATUS,
			"Cant trim multiple steps delta_steps = %d\n",
			delta_steps);
		return prev_trim;
	}
	if (offset_code <= CENTER_TRIM_CODE)
		linear_offset = offset_code + CENTER_TRIM_CODE;
	else if (offset_code > CENTER_TRIM_CODE)
		linear_offset = MAX_TRIM_CODE - offset_code;

	if (scale_code <= CENTER_TRIM_CODE)
		linear_scale = scale_code + CENTER_TRIM_CODE;
	else if (scale_code > CENTER_TRIM_CODE)
		linear_scale = scale_code - (CENTER_TRIM_CODE + 1);

	/* check if we can accomodate delta steps with just the offset */
	if (linear_offset + delta_steps >= 0
			&& linear_offset + delta_steps <= MAX_LIN_CODE) {
		linear_offset += delta_steps;

		if (linear_offset > CENTER_TRIM_CODE)
			offset_code = linear_offset - CENTER_TRIM_CODE;
		else
			offset_code = MAX_TRIM_CODE - linear_offset;

		return (prev_trim & ~VF_TRIM_OFFSET_MASK) | offset_code;
	}

	/* changing offset cannot satisfy delta steps, change the scale bits */
	scale_steps = delta_steps > 0 ? 1 : -1;

	if (linear_scale + scale_steps < 0
			|| linear_scale + scale_steps > MAX_LIN_CODE) {
		pr_smb(PR_STATUS,
			"Cant trim scale_steps = %d delta_steps = %d\n",
			scale_steps, delta_steps);
		return prev_trim;
	}

	linear_scale += scale_steps;

	if (linear_scale > CENTER_TRIM_CODE)
		scale_code = linear_scale - CENTER_TRIM_CODE;
	else
		scale_code = linear_scale + (CENTER_TRIM_CODE + 1);
	prev_trim = (prev_trim & VF_TRIM_OFFSET_MASK)
		| scale_code << SCALE_SHIFT;

	/*
	 * now that we have changed scale which is a 17mV jump, change the
	 * offset bits (10mV) too so the effective change is just 7mV
	 */
	delta_steps = -1 * delta_steps;

	linear_offset = clamp(linear_offset + delta_steps, 0, MAX_LIN_CODE);
	if (linear_offset > CENTER_TRIM_CODE)
		offset_code = linear_offset - CENTER_TRIM_CODE;
	else
		offset_code = MAX_TRIM_CODE - linear_offset;

	return (prev_trim & ~VF_TRIM_OFFSET_MASK) | offset_code;
}

#define TRIM_14		0xFE
#define VF_TRIM_MASK	0xFF
static int smbchg_adjust_vfloat_mv_trim(struct smbchg_chip *chip,
						int delta_mv)
{
	int sign, delta_steps, rc = 0;
	u8 prev_trim, new_trim;
	int i;

	sign = delta_mv > 0 ? 1 : -1;
	delta_steps = (delta_mv + sign * VF_STEP_SIZE_MV / 2)
			/ VF_STEP_SIZE_MV;

	rc = smbchg_read(chip, &prev_trim, chip->misc_base + TRIM_14, 1);
	if (rc) {
		dev_err(chip->dev, "Unable to read trim 14: %d\n", rc);
		return rc;
	}

	for (i = 1; i <= abs(delta_steps)
			&& i <= vf_adjust_trim_steps_per_adjust; i++) {
		new_trim = (u8)smbchg_trim_add_steps(prev_trim,
				delta_steps > 0 ? 1 : -1);
		if (new_trim == prev_trim) {
			pr_smb(PR_STATUS,
				"VFloat trim unchanged from %02x\n", prev_trim);
			/* treat no trim change as an error */
			return -EINVAL;
		}

		rc = smbchg_sec_masked_write(chip, chip->misc_base + TRIM_14,
				VF_TRIM_MASK, new_trim);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't change vfloat trim rc=%d\n", rc);
		}
		pr_smb(PR_STATUS,
			"VFlt trim %02x to %02x, delta steps: %d\n",
			prev_trim, new_trim, delta_steps);
		prev_trim = new_trim;
	}

	return rc;
}

#define VFLOAT_RESAMPLE_DELAY_MS	10000
static void smbchg_vfloat_adjust_work(struct work_struct *work)
{
	struct smbchg_chip *chip = container_of(work,
				struct smbchg_chip,
				vfloat_adjust_work.work);
	int vbat_uv, vbat_mv, ibat_ua, rc, delta_vfloat_mv;
	bool taper, enable;
	u8 reg = 0;

	taper = (get_prop_charge_type(chip)
		== POWER_SUPPLY_CHARGE_TYPE_TAPER);
	enable = taper && (chip->parallel.current_max_ma == 0);

	/* Read out vfloat trim */
	smbchg_read(chip, &reg, chip->misc_base + TRIM_14, 1);
	pr_smb(PR_STATUS,
			"vfloat trim value 0x16FE = 0x%02x\n", reg);

	if (!enable) {
		pr_smb(PR_STATUS,
			"Stopping vfloat adj taper=%d parallel_ma = %d\n",
			taper, chip->parallel.current_max_ma);
		goto stop;
	}

	set_property_on_fg(chip, POWER_SUPPLY_PROP_UPDATE_NOW, 1);
	rc = get_property_from_fg(chip,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, &vbat_uv);
	if (rc) {
		pr_smb(PR_STATUS,
			"bms psy does not support voltage rc = %d\n", rc);
		goto stop;
	}
	vbat_mv = vbat_uv / 1000;

	if ((vbat_mv - chip->vfloat_mv) < -1 * vf_adjust_max_delta_mv) {
		pr_smb(PR_STATUS, "Skip vbat out of range: %d\n", vbat_mv);
		goto reschedule;
	}

	rc = get_property_from_fg(chip,
			POWER_SUPPLY_PROP_CURRENT_NOW, &ibat_ua);
	if (rc) {
		pr_smb(PR_STATUS,
			"bms psy does not support current_now rc = %d\n", rc);
		goto stop;
	}

	if (ibat_ua / 1000 > -chip->iterm_ma) {
		pr_smb(PR_STATUS, "Skip ibat too high: %d\n", ibat_ua);
		goto reschedule;
	}

	pr_smb(PR_STATUS, "sample number = %d vbat_mv = %d ibat_ua = %d\n",
		chip->n_vbat_samples,
		vbat_mv,
		ibat_ua);

	chip->max_vbat_sample = max(chip->max_vbat_sample, vbat_mv);
	chip->n_vbat_samples += 1;
	if (chip->n_vbat_samples < vf_adjust_n_samples) {
		pr_smb(PR_STATUS, "Skip %d samples; max = %d\n",
			chip->n_vbat_samples, chip->max_vbat_sample);
		goto reschedule;
	}
	/* if max vbat > target vfloat, delta_vfloat_mv could be negative */
	delta_vfloat_mv = chip->vfloat_mv - chip->max_vbat_sample;
	pr_smb(PR_STATUS, "delta_vfloat_mv = %d, samples = %d, mvbat = %d\n",
		delta_vfloat_mv, chip->n_vbat_samples, chip->max_vbat_sample);
	/*
	 * enough valid samples has been collected, adjust trim codes
	 * based on maximum of collected vbat samples if necessary
	 */
	if (delta_vfloat_mv > vf_adjust_high_threshold
			|| delta_vfloat_mv < -1 * vf_adjust_low_threshold) {
		rc = smbchg_adjust_vfloat_mv_trim(chip, delta_vfloat_mv);
		if (rc) {
			pr_smb(PR_STATUS,
				"Stopping vfloat adj after trim adj rc = %d\n",
				 rc);
			goto stop;
		}
		chip->max_vbat_sample = 0;
		chip->n_vbat_samples = 0;
		goto reschedule;
	}

stop:
	chip->max_vbat_sample = 0;
	chip->n_vbat_samples = 0;
	smbchg_relax(chip, PM_REASON_VFLOAT_ADJUST);
	return;

reschedule:
	schedule_delayed_work(&chip->vfloat_adjust_work,
			msecs_to_jiffies(VFLOAT_RESAMPLE_DELAY_MS));
	return;
}

static int smbchg_charging_status_change(struct smbchg_chip *chip)
{
	smbchg_low_icl_wa_check(chip);
	/* WA: remove vfloat trim feature */
	//smbchg_vfloat_adjust_check(chip);
	return 0;
}

#define HOT_BAT_HARD_BIT	BIT(0)
#define HOT_BAT_SOFT_BIT	BIT(1)
#define COLD_BAT_HARD_BIT	BIT(2)
#define COLD_BAT_SOFT_BIT	BIT(3)
#define BAT_OV_BIT		BIT(4)
#define BAT_LOW_BIT		BIT(5)
#define BAT_MISSING_BIT		BIT(6)
#define BAT_TERM_MISSING_BIT	BIT(7)
static irqreturn_t batt_hot_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->bat_if_base + RT_STS, 1);
	chip->batt_hot = !!(reg & HOT_BAT_HARD_BIT);
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x, batt_hot=%d\n", reg,
				chip->batt_hot);
	smbchg_charging_status_change(chip);
	if (irq != 0)
		smbchg_adc_notification(ADC_TM_HOT_STATE_SMBCHG, chip);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	return IRQ_HANDLED;
}

static irqreturn_t batt_cold_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->bat_if_base + RT_STS, 1);
	chip->batt_cold = !!(reg & COLD_BAT_HARD_BIT);
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x, batt_cold=%d\n", reg,
				chip->batt_cold);
	smbchg_charging_status_change(chip);
	if (irq != 0)
		smbchg_adc_notification(ADC_TM_COLD_STATE_SMBCHG, chip);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	return IRQ_HANDLED;
}

static irqreturn_t batt_warm_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->bat_if_base + RT_STS, 1);
	chip->batt_warm = !!(reg & HOT_BAT_SOFT_BIT);
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x, batt_warm=%d\n", reg,
				chip->batt_warm);
	if (irq != 0)
		smbchg_adc_notification(ADC_TM_WARM_STATE_SMBCHG, chip);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	return IRQ_HANDLED;
}

static irqreturn_t batt_cool_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->bat_if_base + RT_STS, 1);
	chip->batt_cool = !!(reg & COLD_BAT_SOFT_BIT);
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x, batt_cool=%d\n", reg,
				chip->batt_cool);
	if (irq != 0)
		smbchg_adc_notification(ADC_TM_COOL_STATE_SMBCHG, chip);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	return IRQ_HANDLED;
}

static irqreturn_t batt_pres_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->bat_if_base + RT_STS, 1);
	chip->batt_present = !(reg & BAT_MISSING_BIT);
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);
	return IRQ_HANDLED;
}

static irqreturn_t vbat_low_handler(int irq, void *_chip)
{
	pr_warn_ratelimited("vbat low\n");
	return IRQ_HANDLED;
}

static irqreturn_t chg_error_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;

	pr_smb(PR_INTERRUPT, "chg-error triggered\n");
	smbchg_charging_status_change(chip);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);

	return IRQ_HANDLED;
}

static irqreturn_t fastchg_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;

#if (defined(CONFIG_HTC_BATT_8960))
	pr_err("triggered\n");
#else
	pr_smb(PR_INTERRUPT, "p2f triggered\n");
#endif
	smbchg_charging_status_change(chip);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);

	wake_lock(&chip->eoc_worker_wlock);
	schedule_delayed_work(&chip->eoc_work,
		msecs_to_jiffies(EOC_CHECK_PERIOD_MS));

	return IRQ_HANDLED;
}

static void smbchg_adjust_batt_soc_work(struct work_struct *work)
{
	struct smbchg_chip *chip = container_of(work,
				struct smbchg_chip,
				batt_soc_work);

	if (smbchg_get_prop_capacity_now(chip) == 100)
		smbchg_check_and_notify_fg_soc(chip);
}

static void smbchg_usb_limit_max_current_work(struct work_struct *work)
{
	if(!the_chip)
	{
		pr_err("called before init\n");
		return;
	}

	mutex_lock(&the_chip->current_change_lock);
	pr_smb(PR_STATUS, "USB max current changed to %d\n",
		USB_MA_1500);
	smbchg_set_thermal_limited_usb_current_max(the_chip,
		USB_MA_1500);
	mutex_unlock(&the_chip->current_change_lock);
}

static void smbchg_usb_limit_current_WA_work(struct work_struct *work)
{
	int rc = 0;
	if(!the_chip) {
		pr_err("called before init\n");
		return;
	}
	/* Disable AICL */
	smbchg_sec_masked_write(the_chip, the_chip->usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, 0);
	pr_smb(PR_INTERRUPT, "AICL disabled\n");

	mutex_lock(&the_chip->current_change_lock);
	the_chip->usb_target_current_ma = USB_MA_1000;
	rc = smbchg_set_thermal_limited_usb_current_max(the_chip, USB_MA_1000);
	mutex_unlock(&the_chip->current_change_lock);
	if (rc < 0)
		pr_err("Couldn't set usb current rc = %d\n", rc);
	else
		pr_smb(PR_INTERRUPT, "USB max current changed to 1A\n");


	/* Add a delay so that AICL successfully clears */
	msleep(50);
	smbchg_sec_masked_write(the_chip, the_chip->usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, AICL_EN_BIT);
	pr_smb(PR_INTERRUPT, "AICL re-enabled\n");
}

static irqreturn_t chg_hot_handler(int irq, void *_chip)
{
	pr_warn_ratelimited("chg hot\n");
	return IRQ_HANDLED;
}


static irqreturn_t chg_term_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->chgr_base + RT_STS, 1);

	if(reg & BAT_TCC_REACHED_BIT) {
		is_batt_full_eoc_stop = true;
		htc_gauge_event_notify(HTC_GAUGE_EVENT_EOC_STOP_CHG);
	}

	chip->chg_done_batt_full = !!(reg & BAT_TCC_REACHED_BIT);
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	if (reg & BAT_TCC_REACHED_BIT)
		schedule_work(&chip->batt_soc_work);
	smbchg_charging_status_change(chip);
	return IRQ_HANDLED;
}

static irqreturn_t taper_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->chgr_base + RT_STS, 1);
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
	smbchg_charging_status_change(chip);
	return IRQ_HANDLED;
}

static irqreturn_t recharge_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->chgr_base + RT_STS, 1);
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);
	return IRQ_HANDLED;
}

static irqreturn_t safety_timeout_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->misc_base + RT_STS, 1);
	pr_warn_ratelimited("safety timeout rt_stat = 0x%02x\n", reg);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);
	return IRQ_HANDLED;
}

int pmi8994_prepare_suspend(void)
{
	struct timespec xtime;
	unsigned long cur_jiffies;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	xtime = CURRENT_TIME;
	cur_jiffies = jiffies;
	dischg_plus_sr_time += (cur_jiffies -
			dischg_last_jiffies) * MSEC_PER_SEC / HZ;
	dischg_last_jiffies = cur_jiffies;
	dischg_suspend_ms = xtime.tv_sec * MSEC_PER_SEC +
					xtime.tv_nsec / NSEC_PER_MSEC;

	pr_debug("total dischg time=%lu ms before suspend.\n",
			dischg_plus_sr_time);

	return 0;
}

int pmi8994_complete_resume(void)
{
	unsigned long resume_ms;
	unsigned long sr_time_period_ms;
	struct timespec xtime;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	xtime = CURRENT_TIME;
	dischg_last_jiffies = jiffies;
	resume_ms = xtime.tv_sec * MSEC_PER_SEC + xtime.tv_nsec / NSEC_PER_MSEC;
	sr_time_period_ms = resume_ms - dischg_suspend_ms;
	dischg_plus_sr_time += sr_time_period_ms;

	pr_debug("sr_time_period=%lu ms; total dischg time=%lu ms "
			"after suspend.\n", sr_time_period_ms, dischg_plus_sr_time);

	return 0;
}

/*The max charging current is set as stated in section 3.3.11 of HTC mode specification v0.6.1. */
int pm8994_set_hsml_target_ma(int target_ma)
{
	int rc = 1;
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	pr_smb(PR_STATUS, "USB max charging current target_ma = %d\n", target_ma);

	/* Set max current only when src is USB type */
	if(target_ma != 0 &&
		the_chip->power_source_type == HTC_PWR_SOURCE_TYPE_USB ){
		mutex_lock(&the_chip->current_change_lock);
		the_chip->usb_target_current_ma = target_ma;
		rc = smbchg_set_thermal_limited_usb_current_max(the_chip, target_ma);
		if (rc < 0)
			pr_err("Couldn't set usb current rc = %d\n", rc);
		mutex_unlock(&the_chip->current_change_lock);
	}
	return rc;
}

static int is_HVDCP_9V_done(void *_chip);
bool pmi8994_is_HVDCP_9V_done(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	return is_HVDCP_9V_done(the_chip);
}

#define POWER_OK_BIT   BIT(0)
#define VBUS_VALID_LOWER_BOUND              4150000
static int count_same_dischg = 0;

/**
 * power_ok_handler() - called when the switcher turns on or turns off
 * @chip: pointer to smbchg_chip
 * @rt_stat: the status bit indicating switcher turning on or off
 */
static irqreturn_t power_ok_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;
	int power_ok;
	long diff;
	unsigned long dischg_time_ms;
	unsigned long cur_jiffies;
	static int first = 1;
	static int prev_power_ok = -1;
	static unsigned long prev_dischg_time_ms = 0;
#ifdef CONFIG_HTC_BATT_8960
	ktime_t now = ktime_get();
#endif

	smbchg_read(chip, &reg, chip->misc_base + RT_STS, 1);

	if (reg & POWER_OK_BIT)
		power_ok = 1;
	else
		power_ok = 0;

	cur_jiffies = jiffies;

#ifdef CONFIG_HTC_BATT_8960
	/* WA_OFFMODE_LOOPING */
	if (!strncmp(htc_get_bootmode(),"offmode_charging", strlen("offmode_charging"))
			&& power_ok == 0
			&& !wa_offmode_avoid_weak_chg_out_rapidly
			&& ktime_to_ms(now) <= 10000) {
		wa_offmode_avoid_weak_chg_out_rapidly = true;
		pr_smb(PR_INTERRUPT, "WA_OFFMODE_LOOPING: wa_offmode_avoid_weak_chg_out_rapidly=%d\n",
				wa_offmode_avoid_weak_chg_out_rapidly);
	}
#endif

	if ((power_ok == 0) && (prev_power_ok == 1)) {
		//charging --> discharging
		dischg_last_jiffies = cur_jiffies;
		dischg_plus_sr_time = 0;
		dischg_time_ms = 0;
		first = 0;
		pr_smb(PR_INTERRUPT, "triggered: 0x%02x.\n", reg);
	} else if ((power_ok == 1) && (prev_power_ok == 0) && (first == 0)){
		//discharging --> charging
		dischg_time_ms = dischg_plus_sr_time +
		((cur_jiffies - dischg_last_jiffies) * MSEC_PER_SEC / HZ);

		//if two dichg time is very close (less than 1 second)
		//And it continuously added to 3, set IUSB_MAX as 1A.
		diff = dischg_time_ms - prev_dischg_time_ms;

		pr_smb(PR_INTERRUPT, "triggered: 0x%02x. "
				"prev_dischg_time_ms(%lu) dischg_time_ms(%lu) "
				"diff(%ld) count_same_dischg(%d)\n",
				reg,
				prev_dischg_time_ms,
				dischg_time_ms,
				diff,
				count_same_dischg);

		prev_dischg_time_ms = dischg_time_ms;

		if (ABS(diff) < 1200 &&
				dischg_time_ms <= OCP_RELEASE_TIME_UPPER_BOUND_MS) {
			if(++count_same_dischg == 3) {
				is_limit_IUSB = true;
				count_same_dischg = 0;
				pr_smb(PR_INTERRUPT, "Start to limit USB current\n");
				/* Immediately limit the usb current */
				schedule_work(&chip->usb_aicl_limit_current);
#ifdef CONFIG_HTC_BATT_8960
				wa_offmode_avoid_weak_chg_out_rapidly = false;
#endif
			}
		} else {
			is_limit_IUSB = false;
			count_same_dischg = 0;
#ifdef CONFIG_HTC_BATT_8960
			wa_offmode_avoid_weak_chg_out_rapidly = false;
#endif
		}
	} else
		pr_smb(PR_INTERRUPT, "triggered: 0x%02x. prev_power_ok(%d) "
				"power_ok(%d) \n", reg, prev_power_ok, power_ok);

	prev_power_ok = power_ok;

	return IRQ_HANDLED;
}

/**
 * dcin_uv_handler() - called when the dc voltage crosses the uv threshold
 * @chip: pointer to smbchg_chip
 * @rt_stat: the status bit indicating whether dc voltage is uv
 */
#define DCIN_UNSUSPEND_DELAY_MS		1000
static irqreturn_t dcin_uv_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	bool dc_present = is_dc_present(chip);

	pr_smb(PR_STATUS, "chip->dc_present = %d dc_present = %d\n",
			chip->dc_present, dc_present);

	if (chip->dc_present != dc_present) {
		/* dc changed */
		chip->dc_present = dc_present;
		if (chip->psy_registered)
			power_supply_changed(&chip->dc_psy);
		schedule_work(&chip->notify_fg_work);
		smbchg_aicl_deglitch_wa_check();
		chip->vbat_above_headroom = false;
	}

	return IRQ_HANDLED;
}

#define HVDCP_NOTIFY_MS		2500
static void smbchg_hvdcp_det_work(struct work_struct *work)
{
	struct smbchg_chip *chip = container_of(work,
							struct smbchg_chip,
							hvdcp_det_work.work);

	if (is_HVDCP_9V_done(chip)) {

		pr_smb(PR_MISC, "setting usb psy type = %d\n",
							POWER_SUPPLY_TYPE_USB_HVDCP);
		power_supply_set_supply_type(chip->usb_psy,
						POWER_SUPPLY_TYPE_USB_HVDCP);
		if (chip->psy_registered)
			power_supply_changed(&chip->batt_psy);
		smbchg_aicl_deglitch_wa_check();

		htc_charger_event_notify(HTC_CHARGER_EVENT_SRC_HVDCP);
	}
}

static void handle_usb_removal(struct smbchg_chip *chip)
{
	struct power_supply *parallel_psy;

	smbchg_aicl_deglitch_wa_check();

	/* cancel/wait for hvdcp pending work if any */
	cancel_delayed_work_sync(&chip->hvdcp_det_work);

	if (chip->usb_psy) {
		pr_smb(PR_STATUS, "setting usb psy type = %d\n",
				POWER_SUPPLY_TYPE_UNKNOWN);
		pr_smb(PR_STATUS, "setting usb psy present = %d\n",
				chip->usb_present);
#if !(defined(CONFIG_HTC_BATT_8960))
		power_supply_set_supply_type(chip->usb_psy,
				POWER_SUPPLY_TYPE_UNKNOWN);
#endif
		power_supply_set_present(chip->usb_psy, chip->usb_present);
		if (delayed_work_pending(&chip->usb_set_online_work))
				 cancel_delayed_work_sync(&chip->usb_set_online_work);
		schedule_delayed_work(&chip->usb_set_online_work, msecs_to_jiffies(100));
	}
	if (chip->parallel.avail) {
		parallel_psy = get_parallel_psy(chip);
		if (parallel_psy) {
			power_supply_set_present(parallel_psy, false);
			chip->parallel.current_max_ma = SUSPEND_CURRENT_MA;
			disable_irq_wake(chip->aicl_done_irq);
		}
	}
	chip->vbat_above_headroom = false;
}

static bool is_src_detect_high(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb rt status rc = %d\n", rc);
		return false;
	}
	return reg &= USBIN_SRC_DET_BIT;
}

#define IDEV_STS	0x8
#if !(defined(CONFIG_HTC_BATT_8960))
static void handle_usb_insertion(struct smbchg_chip *chip)
{
	struct power_supply *parallel_psy;
	u8 reg = 0;
	int rc;
	char *usb_type_name = "null";
	enum power_supply_type usb_supply_type;

	/* usb inserted */
	rc = smbchg_read(chip, &reg, chip->misc_base + IDEV_STS, 1);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't read status 5 rc = %d\n", rc);
	usb_type_name = get_usb_type_name(reg);
	usb_supply_type = get_usb_supply_type(reg);
	pr_smb(PR_STATUS, "inserted %s, usb psy type = %d stat_5 = 0x%02x\n",
			usb_type_name, usb_supply_type, reg);
	if (chip->usb_psy) {
		pr_smb(PR_STATUS, "setting usb psy type = %d\n",
				usb_supply_type);
		power_supply_set_supply_type(chip->usb_psy, usb_supply_type);
		pr_smb(PR_STATUS, "setting usb psy present = %d\n",
				chip->usb_present);
		power_supply_set_present(chip->usb_psy, chip->usb_present);
		if (delayed_work_pending(&chip->usb_set_online_work))
				 cancel_delayed_work_sync(&chip->usb_set_online_work);
		schedule_delayed_work(&chip->usb_set_online_work, msecs_to_jiffies(100));
	}

	if (usb_supply_type == POWER_SUPPLY_TYPE_USB_DCP)
		schedule_delayed_work(&chip->hvdcp_det_work,
					msecs_to_jiffies(HVDCP_NOTIFY_MS));

	if (chip->parallel.avail) {
		chip->parallel.current_max_ma = SUSPEND_CURRENT_MA;
		parallel_psy = get_parallel_psy(chip);
		if (parallel_psy) {
			power_supply_set_present(parallel_psy, true);
			enable_irq_wake(chip->aicl_done_irq);
		}
	}
}
#endif

void update_usb_status(struct smbchg_chip *chip, bool usb_present, bool force)
{
	mutex_lock(&chip->usb_status_lock);
#if (defined(CONFIG_HTC_BATT_8960))
	cable_detection_vbus_irq_handler();
	schedule_work(&chip->notify_fg_work);
#else
	if (force) {
		chip->usb_present = usb_present;
		chip->usb_present ? handle_usb_insertion(chip)
			: handle_usb_removal(chip);
		goto unlock;
	}
	if (!chip->usb_present && usb_present) {
		chip->usb_present = usb_present;
		handle_usb_insertion(chip);
	} else if (chip->usb_present && !usb_present) {
		chip->usb_present = usb_present;
		handle_usb_removal(chip);
	}
unlock:
#endif
	mutex_unlock(&chip->usb_status_lock);
}

int pmi8994_set_ftm_charge_enable_type(enum htc_ftm_power_source_type ftm_src)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	if (the_chip->ftm_src != ftm_src) {
		pr_info("%s(%d -> %d)\n", __func__, the_chip->ftm_src, ftm_src);
		the_chip->ftm_src = ftm_src;
	}

	return 0;
}

static u32 htc_fake_charger_for_ftm(enum htc_power_source_type src)
{
	unsigned int new_src = src;

	if((src <= HTC_PWR_SOURCE_TYPE_9VAC) && (src != HTC_PWR_SOURCE_TYPE_BATT)) {
		if (the_chip->ftm_src == HTC_FTM_PWR_SOURCE_TYPE_USB)
			new_src = HTC_PWR_SOURCE_TYPE_USB;
		else if (the_chip->ftm_src == HTC_FTM_PWR_SOURCE_TYPE_AC)
			new_src = HTC_PWR_SOURCE_TYPE_AC;

		if (src != new_src)
			pr_info("%s(%d -> %d)\n", __func__, src , new_src);
	}

	return new_src;
}

static u32 htc_fake_charger_for_testing(enum htc_power_source_type src)
{
	/* set charger to 1A AC  by default */
	enum htc_power_source_type new_src = HTC_PWR_SOURCE_TYPE_AC;

	if((src > HTC_PWR_SOURCE_TYPE_9VAC) || (src == HTC_PWR_SOURCE_TYPE_BATT))
		return src;

	pr_info("%s(%d -> %d)\n", __func__, src , new_src);
	return new_src;
}

static DEFINE_MUTEX(cable_notify_sem);
static void send_cable_connect_notify(int cable_type)
{
	static struct t_cable_status_notifier *notifier;

	mutex_lock(&cable_notify_sem);

	list_for_each_entry(notifier,
		&g_lh_calbe_detect_notifier_list,
		cable_notifier_link) {
			if (notifier->func != NULL) {
				pr_smb(PR_STATUS, "Send to: %s, type %d\n",
						notifier->name, cable_type);
				/* Notify other drivers about connect type. */
				notifier->func(cable_type);
			}
		}
	mutex_unlock(&cable_notify_sem);

	wake_unlock(&the_chip->vbus_wlock);
}

int cable_detect_register_notifier(struct t_cable_status_notifier *notifier)
{
	if (!notifier || !notifier->name || !notifier->func)
		return -EINVAL;

	mutex_lock(&cable_notify_sem);
	list_add(&notifier->cable_notifier_link,
		&g_lh_calbe_detect_notifier_list);
#if 0//FIXME
	if (the_cable_info.notify_init == 1)
		notifier->func(cable_get_connect_type());
#endif
	mutex_unlock(&cable_notify_sem);
	return 0;
}

/*
 * return value
 * CONNECT_TYPE_MHL_UNKNOWN : MHL
 * CONNECT_TYPE_NOTIFY : wait USB controller online
 * CONNECT_TYPE_UNKNOWN : unknown charger
 * CONNECT_TYPE_USB : SDP charger
 */
extern int usb_get_connect_type(void);
static void usb_type_polling(struct work_struct *work)
{
	int rc;
	union power_supply_propval prop = {0,};

	the_chip->usb_psy->get_property(the_chip->usb_psy, POWER_SUPPLY_PROP_PRESENT, &prop);
	pr_info("USB present: %d\n", prop.intval);
	if (prop.intval == 0) {
		pr_info("USB cable out\n");
		return;
	}

	rc = usb_get_connect_type();
	switch(rc){
		case CONNECT_TYPE_NOTIFY :
			pr_info("USB type is under checking...\n");
			schedule_delayed_work(&the_chip->usb_type_polling_work, msecs_to_jiffies(2000));
			break;
		case CONNECT_TYPE_UNKNOWN :
		case CONNECT_TYPE_MHL_UNKNOWN :
			pr_info("USB type is unknown or MHL...send notify...%d\n", rc);
			send_cable_connect_notify(CONNECT_TYPE_UNKNOWN);
			break;
		default:
			pr_info("USB type: %d\n", rc);
			break;
	}
}

void usb_status_notifier_func(int cable_type);
static void check_cable_type(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct smbchg_chip *chip = container_of(dwork,
				struct smbchg_chip,
				vbus_detect_work);
	u8 reg = 0;
	int rc, connect_type, vbus_uv;
	char *usb_type_name = "null";
	enum power_supply_type usb_supply_type;
	bool usb_present = is_usb_present(chip);

	/* Read cable type from pmi8994 */
	rc = smbchg_read(chip, &reg, chip->misc_base + IDEV_STS, 1);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't read status 5 rc = %d\n", rc);
	usb_type_name = get_usb_type_name(reg);
	usb_supply_type = get_usb_supply_type(reg);
	vbus_uv = pmi8994_get_usbin_voltage_now(chip);

	pr_smb(PR_STATUS, "inserted %s, usb_type=%d, pre_usb_type=%d, reg=0x%02x, "
					"vbus=%d, pre_usb_present=%d, usb_present=%d\n",
			usb_type_name, usb_supply_type, chip->pre_usb_supply_type,
			reg, vbus_uv, chip->usb_present, usb_present);
	/* Continue previous usb type if usb is present but result of APSD is none. */
	if (!reg && usb_present)
		usb_supply_type = chip->pre_usb_supply_type;
	else
		chip->pre_usb_supply_type = usb_supply_type;

	power_supply_set_supply_type(chip->usb_psy, usb_supply_type);

	if (!usb_present) {
		connect_type = CONNECT_TYPE_NONE;
	} else {
		switch (usb_supply_type) {
		case POWER_SUPPLY_TYPE_UNKNOWN :
			connect_type = CONNECT_TYPE_UNKNOWN;
			break;
		case POWER_SUPPLY_TYPE_USB_DCP :
			connect_type = CONNECT_TYPE_AC;
			break;
		case POWER_SUPPLY_TYPE_USB_CDP :
		default :
			connect_type = CONNECT_TYPE_USB;
			break;
		}
	}
	if((usb_get_connect_type() > CONNECT_TYPE_MHL_UNKNOWN)
		&& (usb_get_connect_type() < CONNECT_TYPE_MHL_100MA)){ /* Cable is not MHL! */
		usb_status_notifier_func(connect_type);
		send_cable_connect_notify(connect_type);
	}
}

/**
 * usbin_uv_handler() - this is called when USB charger is removed
 * @chip: pointer to smbchg_chip chip
 * @rt_stat: the status bit indicating chg insertion/removal
 */
static irqreturn_t usbin_uv_handler(int irq, void *_chip)
{
	int rc;
	u8 reg;
	struct smbchg_chip *chip = _chip;
	bool usb_present = is_usb_present(chip);
	int aicl_level = smbchg_get_aicl_level_ma(chip);

	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb rt status rc = %d\n", rc);
		goto out;
	}

	pr_smb(PR_STATUS, "chip->usb_present = %d usb_present = %d "
			"aicl = %d rt_sts = 0x%02x\n",
			chip->usb_present, usb_present, aicl_level, reg);

	if ((reg & USBIN_UV_BIT) && (reg & USBIN_SRC_DET_BIT)
			&& chip->aicl_deglitch_short
			&& aicl_level == usb_current_table[0]) {
		pr_smb(PR_STATUS, "Very weak charger detected\n");
		chip->very_weak_charger = true;
		/* No need to disable AICL rerun */
		/* This will not allow WA to avoid suspend by fixing current to 150mA*/
		//smbchg_hw_aicl_rerun_en(chip, false);
		rc = power_supply_set_health_state(chip->usb_psy,
				POWER_SUPPLY_HEALTH_UNSPEC_FAILURE);
		if (rc)
			pr_err("Couldn't set health on usb psy rc:%d\n", rc);
		if (delayed_work_pending(&chip->usb_set_online_work))
			cancel_delayed_work_sync(&chip->usb_set_online_work);
		schedule_delayed_work(&chip->usb_set_online_work, 0);
	}

	if (reg & USBIN_UV_BIT) {
		if (!usb_present) {
			update_usb_status(chip, 0, false);
			chip->aicl_irq_count = 0;
		}
	}
#if (defined(CONFIG_HTC_BATT_8960))
	if (!(reg & USBIN_UV_BIT) && (reg & USBIN_SRC_DET_BIT)
			&& !chip->usb_present && usb_present) {
		/* USB inserted */
		cable_detection_vbus_irq_handler();
		schedule_work(&chip->notify_fg_work);
	}
#endif

	pr_smb(PR_STATUS, "triggered: 0x1310=%02x, pre_usb_present=%d,"
					" usb_present=%d\n",
			reg, chip->usb_present, usb_present);
out:
	return IRQ_HANDLED;
}

static irqreturn_t cable_detection_vbus_irq_handler(void)
{
	unsigned long flags;
	unsigned long delay_time = HZ/20*7;

	spin_lock_irqsave(&the_chip->vbus_lock, flags);

#if 0//FIXME
	__cancel_delayed_work(&the_chip->vbus_detect_work);
#endif

	/* WA_OFFMODE_LOOPING:
	 *   Delay 2 seconds to notify cable out event
	 *   to avoid the looping offmode charging issue.
	 * Symptom : The VBUS of the specific 1A adaptor
	 *   will drop during 1.5A AICL and trigger usbin_uv
	 *   to run power off flow. After VBUS recovered,
	 *   it would trigger device power on to entry
	 *   offmode charging mode. Then repeat the symptom.
	 */
	if (wa_offmode_avoid_weak_chg_out_rapidly) {
		cancel_delayed_work(&the_chip->vbus_detect_work);

		if (!is_usb_present(the_chip)) {
			delay_time = msecs_to_jiffies(2000);
			pr_smb(PR_STATUS, "WA_OFFMODE_LOOPING: delay 2 seconds"
					" to notify cable out event\n");
		}
	}

	queue_delayed_work(the_chip->cable_detect_wq,
			&the_chip->vbus_detect_work, delay_time);

	spin_unlock_irqrestore(&the_chip->vbus_lock, flags);

	wake_lock_timeout(&the_chip->vbus_wlock, HZ/2);

	pr_smb(PR_STATUS, "end\n");

	return IRQ_HANDLED;
}

/**
 * usbin_ov_handler() - this is called when USB charger is over voltage
 * @chip: pointer to smbchg_chip chip
 * @rt_stat: the status bit indicating chg insertion/removal
 */
static irqreturn_t usbin_ov_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	int ov = get_prop_usb_ov_status(chip);

	pr_smb(PR_STATUS, "triggered: ov=%d\n", ov);

	htc_charger_event_notify(HTC_CHARGER_EVENT_OVP);
	return IRQ_HANDLED;
}

/**
 * src_detect_handler() - this is called on rising edge when USB charger type
 *		 is detected and on falling edge when USB voltage falls
 *		 below the coarse detect voltage(1V), use it for
 *		 handling USB charger insertion and CDP or SDP removal
 * @chip: pointer to smbchg_chip
 * @rt_stat: the status bit indicating chg insertion/removal
 */
static irqreturn_t src_detect_handler(int irq, void *_chip)
{
	int rc;
	struct smbchg_chip *chip = _chip;
	bool usb_present = is_usb_present(chip);
	bool src_detect = is_src_detect_high(chip);

#if (defined(CONFIG_HTC_BATT_8960))
	pr_warn( "triggered:pre_usb_present=%d, usb_present=%d, src_detect=%d\n",
			chip->usb_present, usb_present, src_detect);
	/* ATS WA: Disable AICL for testing. */
	if (flag_force_ac_chg && usb_present) {
		smbchg_sec_masked_write(the_chip,
			the_chip->usb_chgpth_base + USB_AICL_CFG, AICL_EN_BIT, 0);
		pr_smb(PR_STATUS, "ATS: disable AICL\n");
	}
	if (!usb_present) {
		chip->very_weak_charger = false;
		chip->aicl_irq_count = 0;
	}
	update_usb_status(chip, usb_present, false);

	rc = power_supply_set_health_state(chip->usb_psy,
			chip->very_weak_charger
			? POWER_SUPPLY_HEALTH_UNSPEC_FAILURE
			: POWER_SUPPLY_HEALTH_GOOD);
	if (rc)
		pr_smb(PR_STATUS,
			"usb psy does not allow updating prop %d rc = %d\n",
			POWER_SUPPLY_HEALTH_GOOD, rc);
#else
	pr_smb(PR_STATUS, "chip->usb_present = %d usb_present = %d\n",
			chip->usb_present, usb_present);

	if (usb_present)
		update_usb_status(chip, usb_present, false);
#endif

	return IRQ_HANDLED;
}

/**
 * otg_oc_handler() - called when the usb otg goes over current
 */
#define NUM_OTG_RETRIES			5
#define OTG_OC_RETRY_DELAY_US		50000
static irqreturn_t otg_oc_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	s64 elapsed_us = ktime_us_delta(ktime_get(), chip->otg_enable_time);

	if (elapsed_us > OTG_OC_RETRY_DELAY_US)
		chip->otg_retries = 0;

	pr_smb(PR_INTERRUPT, "triggered\n");
	/*
	 * Due to a HW bug in the PMI8994 charger, the current inrush that
	 * occurs when connecting certain OTG devices can cause the OTG
	 * overcurrent protection to trip.
	 *
	 * The work around is to try reenabling the OTG when getting an
	 * overcurrent interrupt once.
	 */
	if (chip->otg_retries < NUM_OTG_RETRIES) {
		chip->otg_retries += 1;
		pr_smb(PR_STATUS,
			"Retrying OTG enable. Try #%d, elapsed_us %lld\n",
						chip->otg_retries, elapsed_us);
		smbchg_masked_write(chip, chip->bat_if_base + CMD_CHG_REG,
							OTG_EN, 0);
		msleep(20);
		smbchg_masked_write(chip, chip->bat_if_base + CMD_CHG_REG,
							OTG_EN, OTG_EN);
		chip->otg_enable_time = ktime_get();
	}
	return IRQ_HANDLED;
}

/**
 * otg_fail_handler() - called when the usb otg fails
 * (when vbat < OTG UVLO threshold)
 */
static irqreturn_t otg_fail_handler(int irq, void *_chip)
{
	pr_smb(PR_INTERRUPT, "triggered\n");
	return IRQ_HANDLED;
}

static int is_HVDCP_9V_done(void *_chip)
{
	struct smbchg_chip *chip = _chip;
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + USBIN_HVDCP_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb status rc = %d\n", rc);
		return FALSE;
	}

	if ((reg & USBIN_HVDCP_SEL_BIT) && (reg & USBIN_HVDCP_SEL_9V_BIT))
	{
		return TRUE; // you get HVDCP
	}
	return FALSE;
}

static int get_current_time(unsigned long *now_tm_sec)
{
	struct rtc_time tm;
	struct rtc_device *rtc;
	int rc;

	rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	if (rtc == NULL) {
		pr_err("%s: unable to open rtc device (%s)\n",
			__FILE__, CONFIG_RTC_HCTOSYS_DEVICE);
		return -EINVAL;
	}

	rc = rtc_read_time(rtc, &tm);
	if (rc) {
		pr_err("Error reading rtc device (%s) : %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}

	rc = rtc_valid_tm(&tm);
	if (rc) {
		pr_err("Invalid RTC time (%s): %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}
	rtc_tm_to_time(&tm, now_tm_sec);

close_time:
	rtc_class_close(rtc);
	return rc;
}

#define AICL_IRQ_LIMIT_SECONDS	60
#define AICL_IRQ_LIMIT_COUNT	25
#define RETRY_AICL_TOTAL	2
#define RETRY_AICL_INTERVAL_MS	180000
#define INTERVAL_1_MINUTE_MS	60000

static void increment_aicl_count(struct smbchg_chip *chip)
{
	int aicl_result = smbchg_get_aicl_level_ma(chip);
	enum power_supply_type usb_supply_type;
	bool bad_charger = false;
	int rc = 0;
	u8 reg = 0, usb_type_reg = 0;
	long elapsed_seconds;
	unsigned long now_seconds;

	rc = smbchg_read(chip, &usb_type_reg, chip->misc_base + IDEV_STS, 1);
	usb_supply_type = get_usb_supply_type(usb_type_reg);

	smbchg_read(chip, &reg, chip->usb_chgpth_base + ICL_STS_1_REG, 1);
	pr_smb(PR_INTERRUPT, "aicl count c:%d dgltch:%d first:%ld, "
			"0x1307:0x%02x, usb_type:%d, aicl_result(%d)\n",
			chip->aicl_irq_count, chip->aicl_deglitch_short,
			chip->first_aicl_seconds, reg, usb_supply_type, aicl_result);

	/* Disable the AICL rerun once a certain limit of AICL done *
	 * interrupts are received when the AICL deglitch timer is  *
	 * configured for short interval                            */
	if (chip->aicl_deglitch_short) {
		if (!chip->aicl_irq_count)
			get_current_time(&chip->first_aicl_seconds);
		get_current_time(&now_seconds);
		elapsed_seconds = now_seconds
				- chip->first_aicl_seconds;

		if (elapsed_seconds > AICL_IRQ_LIMIT_SECONDS) {
			pr_smb(PR_INTERRUPT,
				"resetting: elp:%ld first:%ld now:%ld c=%d\n",
				elapsed_seconds, chip->first_aicl_seconds,
				now_seconds, chip->aicl_irq_count);
			chip->aicl_irq_count = 1;
			get_current_time(&chip->first_aicl_seconds);
			return;
		}
		chip->aicl_irq_count++;

		if (chip->aicl_irq_count > AICL_IRQ_LIMIT_COUNT) {
			pr_smb(PR_INTERRUPT, "elpsd:%ld first:%ld now:%ld c=%d\n",
								elapsed_seconds, chip->first_aicl_seconds,
								now_seconds, chip->aicl_irq_count);
			/* Set current to 150 mA to avoid suspend */
			rc = smbchg_usb_en(chip, true, REASON_USB);
			rc = smbchg_sec_masked_write(chip,
					chip->usb_chgpth_base + CHGPTH_CFG,
					CFG_USB_2_3_SEL_BIT, CFG_USB_3);
			rc |= smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL,
				USBIN_MODE_CHG_BIT | USB51_MODE_BIT,
				USBIN_LIMITED_MODE | USB51_100MA);
			/* Change the stored state of USB max current */
			chip->usb_max_current_ma = USB_MA_150;
			pre_current_ma = pre_usb_max_current_ma = USB_MA_150;

			pr_smb(PR_INTERRUPT, "Bad cable, set current ma: %d\n", USB_MA_150);

			/* Set the USB max current limit to 1500mA after 1 minute */
			if (delayed_work_pending(&chip->usb_limit_max_current))
				cancel_delayed_work(&chip->usb_limit_max_current);
			schedule_delayed_work(&chip->usb_limit_max_current,
				msecs_to_jiffies(INTERVAL_1_MINUTE_MS));

			chip->aicl_irq_count = 0;

			/*Set the bad AICL flag*/
			gs_is_bad_aicl_result = true;
		} else if ((get_prop_charge_type(chip) ==
				POWER_SUPPLY_CHARGE_TYPE_FAST) &&
					(reg & AICL_SUSP_BIT)) {
			bad_charger = true;
		}
		if (bad_charger) {
			rc = power_supply_set_health_state(chip->usb_psy,
					POWER_SUPPLY_HEALTH_UNSPEC_FAILURE);
			if (rc)
				pr_err("Couldn't set health on usb psy rc:%d\n",
					rc);
			if (delayed_work_pending(&chip->usb_set_online_work))
				 cancel_delayed_work_sync(&chip->usb_set_online_work);
			schedule_delayed_work(&chip->usb_set_online_work, 0);
		}
	}

	/* limit IUSB_MAX as 1000mA */
	if ((reg & AICL_STS_BIT) && (is_qc20_done_flag == false) &&
			(usb_supply_type == POWER_SUPPLY_TYPE_USB_DCP)) {
		/* a. If AICL is 1.1~1.3A */
		if ((((aicl_result == USB_MA_1100) || (aicl_result == USB_MA_1200)) ||
		/* b. If similar OCP met 3 times */
				((is_limit_IUSB == true) && (aicl_result > USB_MA_1000)))) {
			gs_is_limit_1A = true;
			/* Reset cnt for retry AICL function */
			retry_aicl_cnt = 0;
		} else if (aicl_result <= USB_MA_500 &&
				retry_aicl_cnt < RETRY_AICL_TOTAL) {
			if (delayed_work_pending(&chip->retry_aicl_work))
				cancel_delayed_work_sync(&chip->retry_aicl_work);
			pr_smb(PR_STATUS, "Trigger re-do AICL in 3 minutes, cnt=%d\n",
									retry_aicl_cnt);
			schedule_delayed_work(&chip->retry_aicl_work,
								msecs_to_jiffies(RETRY_AICL_INTERVAL_MS));
		}
	}
}

/**
 * aicl_done_handler() - called when the usb AICL algorithm is finished
 *			and a current is set.
 */
static irqreturn_t aicl_done_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	bool usb_present = is_usb_present(chip);
	int aicl_level = smbchg_get_aicl_level_ma(chip);

	pr_smb(PR_INTERRUPT, "triggered, aicl: %d\n", aicl_level);

	increment_aicl_count(chip);

	if (chip->parallel.avail && usb_present)
		smbchg_parallel_usb_determine_current(chip);
	return IRQ_HANDLED;
}

static void smbchg_rerun_aicl(struct smbchg_chip *chip)
{
	smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, 0);
	/* Add a delay so that AICL successfully clears */
	msleep(50);
	smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, AICL_EN_BIT);
}

static void
retry_aicl_worker(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct smbchg_chip *chip = container_of(dwork,
				struct smbchg_chip, retry_aicl_work);
	smbchg_rerun_aicl(chip);
	retry_aicl_cnt++;
}

static void
hvdcp_5to9V_worker(struct work_struct *work)
{
	int rc = 0;
	struct delayed_work *dwork = to_delayed_work(work);
	struct smbchg_chip *chip = container_of(dwork,
			struct smbchg_chip, hvdcp_5to9V_work);

	pr_smb(PR_STATUS, "Set HVDCP 5V to 9V "
			"by setting 0x13F4<5:4>\n");

	rc = smbchg_sec_masked_write(chip,
			chip->usb_chgpth_base + CFG_SYSMIN, 0x30, 0x10);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set 0x13F4 = %d\n",
				rc);
	}
}

#define USB_CMD_APSD	0x41
#define APSD_RERUN		BIT(0)
#define USBIN_ADAPTER_9V		0x3
#define USBIN_ADAPTER_5V_9V_UNREG	0x5
#define USBIN_CHGR_MASK			SMB_MASK(2, 0)
#define USBIN_CHGR_CFG			0xF1
static int rerun_apsd(struct smbchg_chip *chip)
{
	int rc;
	u8 reg = 0;
	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + USBIN_CHGR_CFG, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb allowance rc=%d\n", rc);
		return rc;
	}
	pr_smb(PR_STATUS, "Change to USBIN_ADAPTER_ALLOWANCE_9V [0x%X]reg = 0x%X, Retry_count:%d\n",
		chip->usb_chgpth_base + USBIN_CHGR_CFG, reg, gs_aspd_rerun_count);
	rc = smbchg_sec_masked_write(chip,
		chip->usb_chgpth_base + USBIN_CHGR_CFG,
		USBIN_CHGR_MASK, USBIN_ADAPTER_9V);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't write usb allowance to 9V rc=%d\n", rc);
		return rc;
	}

	msleep(100);

	rc = smbchg_sec_masked_write(chip,
		chip->usb_chgpth_base + USBIN_CHGR_CFG,
		USBIN_CHGR_MASK, USBIN_ADAPTER_5V_9V_UNREG);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't write usb allowance to 9V rc=%d\n", rc);
		return rc;
	}
	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + USBIN_CHGR_CFG, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb allowance rc=%d\n", rc);
		return rc;
	}
	pr_smb(PR_STATUS, "Recover to USBIN_ADAPTER_ALLOWANCE_5V_TO_9V[0x%X]reg = 0x%X\n", chip->usb_chgpth_base + USBIN_CHGR_CFG, reg);

	return rc;
}


/*++ 2014/10/07 USB Team, PCN00014 ++*/
#if 0
/**
 * usbid_change_handler() - called when the usb RID changes.
 * This is used mostly for detecting OTG
 */
static irqreturn_t usbid_change_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	bool otg_present;

	pr_smb(PR_INTERRUPT, "triggered\n");

	/*
	 * After the falling edge of the usbid change interrupt occurs,
	 * there may still be some time before the ADC conversion for USB RID
	 * finishes in the fuel gauge. In the worst case, this could be up to
	 * 15 ms.
	 *
	 * Sleep for 20 ms (minimum msleep time) to wait for the conversion to
	 * finish and the USB RID status register to be updated before trying
	 * to detect OTG insertions.
	 */
	msleep(20);
	otg_present = is_otg_present(chip);
	if (chip->usb_psy)
		power_supply_set_usb_otg(chip->usb_psy, otg_present ? 1 : 0);
	if (otg_present)
		pr_smb(PR_STATUS, "OTG detected\n");

	return IRQ_HANDLED;
}
#endif
/*-- 2014/10/07 USB Team, PCN00014 --*/

/* Remove chg_inhibit_irq because of PMI8994 known issue */
#if 0
static irqreturn_t chg_inhibit_handler(int irq, void *_chip)
{
	/*
	 * charger is inserted when the battery voltage is high
	 * so h/w won't start charging just yet. Treat this as
	 * battery full
	 */
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->chgr_base + RT_STS, 1);
	chip->chg_done_batt_full = !!(reg & CHG_INHIBIT_BIT);
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	return IRQ_HANDLED;
}
#endif

static void
smbchg_adjust_fastchg_current(int vbat_mv)
{
	if (vbat_mv <= the_chip->adjust_fastchg_cur_thr_mv &&
			the_chip->fastchg_current_ma != the_chip->high_fastchg_cur_ma &&
			is_over_fastchg_thr_once == false) {
		the_chip->fastchg_current_ma = the_chip->high_fastchg_cur_ma;
		smbchg_set_appropriate_battery_current(the_chip);
	} else if (vbat_mv > the_chip->adjust_fastchg_cur_thr_mv &&
			the_chip->fastchg_current_ma != the_chip->target_fastchg_current_ma) {
		is_over_fastchg_thr_once = true;
		the_chip->fastchg_current_ma = the_chip->target_fastchg_current_ma;
		smbchg_set_appropriate_battery_current(the_chip);
	}
}


static void
smbchg_eoc_work(struct work_struct *work)
{
	int rc, sys_sts, soc = 0;
	u8 chgr_sts = 0, int_rt_sts = 0, chg_type;
	struct delayed_work *dwork = to_delayed_work(work);
	struct smbchg_chip *chip = container_of(dwork,
				struct smbchg_chip, eoc_work);
	int vbat_mv, ibat_ma;
	u8 reg_rt_sts = 0;
	int vbus_uv = 0;
	wake_lock(&chip->eoc_worker_wlock);

	rc = smbchg_read(chip, &chgr_sts, chip->chgr_base + CHGR_STS, 1);
	if (rc) {
		dev_err(chip->dev, "failed to read chgr_sts (0x100E) rc=%d\n", rc);
		return;
	}

	rc = smbchg_read(chip, &int_rt_sts, chip->chgr_base + RT_STS, 1);
	if (rc) {
		dev_err(chip->dev, "failed to read int_rt_sts (0x1010) rc=%d\n", rc);
		return;
	}

	chg_type = (chgr_sts & CHG_TYPE_MASK) >> CHG_TYPE_SHIFT;
	vbat_mv = smbchg_get_prop_voltage_now(chip)/1000;
	ibat_ma = smbchg_get_prop_current_now(chip)/1000;
	rc = pmi8994_fg_get_system_status(&sys_sts);

	if (rc)
		pr_err("failed to read sys_sts (0x4107) rc=%d\n", rc);

	pr_info("vbat_mv=%d, ibat_ma=%d, 0x100E:0x%x, 0x1010:0x%x, chg_type:%d,"
		" is_qc20_done=%d, eoc_count=%d\n", vbat_mv, ibat_ma, chgr_sts, int_rt_sts,
		chg_type, is_qc20_done_flag, eoc_count);

	/* check present of usbin or dcin */
	if (!is_usb_present(chip) && !is_dc_present(chip)) {
		pr_smb(PR_STATUS, "no charger connected, stopping\n");
		is_batt_full = false;
		is_batt_full_eoc_stop = false;
		goto stop_eoc;
	}

	/* Adjust IBAT_MAX based on specification of different battery. */
	if (chip->adjust_fastchg_cur_thr_mv != -EINVAL &&
			chip->high_fastchg_cur_ma != -EINVAL)
		smbchg_adjust_fastchg_current(vbat_mv);

	/* check QC2.0 and limit its current*/
	if (is_HVDCP_9V_done(chip) && (is_qc20_done_flag == false)) {
		is_qc20_done_flag = true;
		mutex_lock(&chip->current_change_lock);
		chip->usb_target_current_ma = chip->qc20_usb_max_current_ma;
		rc = smbchg_set_thermal_limited_usb_current_max(chip,
				chip->qc20_usb_max_current_ma);
		mutex_unlock(&chip->current_change_lock);
		if (rc < 0)
			pr_err("Couldn't set usb current rc = %d\n", rc);
		else
			pr_smb(PR_STATUS, "QC2.0 max current changed to %d mA\n",
					chip->qc20_usb_max_current_ma);
	}

	/* limit aicl result to 1000 mA */
	if (gs_is_limit_1A == true) {
		gs_is_limit_1A = false;
		mutex_lock(&chip->current_change_lock);
		chip->usb_target_current_ma = USB_MA_1000;
		rc = smbchg_set_thermal_limited_usb_current_max(chip, USB_MA_1000);
		mutex_unlock(&chip->current_change_lock);
		if (rc < 0)
			pr_err("Couldn't set usb current rc = %d\n", rc);
		else
			pr_smb(PR_INTERRUPT, "USB max current changed to 1A\n");
	}

	/* In fast charging, it would execute eoc_work */
	if ((int_rt_sts & P2F_CHG_THRESHOLD_BIT) == 0) {
		pr_info("not charging by eoc stop charging\n");
		goto stop_eoc;
	} else {
		/* to judge full and eoc flags */
		if (chg_type != BATT_TAPER_CHG_VAL) {
			pr_debug("Not in CV\n");
			eoc_count_by_curr = eoc_count = 0;
		} else if (ibat_ma > 0) {
			/* battery is discharging */
			pr_smb(PR_DUMP, "Charging but system demand increased\n");
			eoc_count_by_curr = eoc_count = 0;
		} else if ((ibat_ma * -1) > chip->eoc_ibat_thre_ma) {
			/* charging current >= eoc_ibat_thre_ma criterion */
			pr_smb(PR_DUMP, "Not at EOC, battery current too high\n");
			eoc_count_by_curr = eoc_count = 0;
		} else if ((ibat_ma * -1) > chip->iterm_ma) {
			/* iterm_ma < charging current */
			pr_smb(PR_DUMP, "start count for HTC battery full condition\n");
			eoc_count_by_curr = 0;
			eoc_count++;
			if (eoc_count == CONSECUTIVE_COUNT) {
				/* battery is full by 0.1C condition */
				is_batt_full = true;
				htc_gauge_event_notify(HTC_GAUGE_EVENT_EOC);
			}
		} else {
			eoc_count++;
			/* charging current < iterm_ma */
			if (eoc_count_by_curr == CONSECUTIVE_COUNT) {
				/* battery is truely full */
				pr_info("End of Charging\n");
				is_batt_full_eoc_stop = true;
				goto stop_eoc;
			} else {
				if (eoc_count == CONSECUTIVE_COUNT && !is_batt_full) {
					is_batt_full = true;
					htc_gauge_event_notify(HTC_GAUGE_EVENT_EOC);
				}
				eoc_count_by_curr += 1;
				pr_info("eoc_count_by_curr = %d, eoc_count=%d\n",
					eoc_count_by_curr, eoc_count);
			}
		}
	}

	/* Check if overloading is happened after full charging.
	If CLEAR_FULL_STATE_BY_LEVEL_THR is set too high(99),
	you might meet soc can't report higher soc due to
	start_percent is -22 even if overloading status has been removed. */
	if (is_batt_full) {
		soc = smbchg_get_prop_capacity_now(chip);
		if (soc < CLEAR_FULL_STATE_BY_LEVEL_THR) {
			/* We add one more condition to check overloading */
			if (chip->vfloat_mv && // vfloat_mv means max_voltage_mv
				(vbat_mv > (chip->vfloat_mv - 100))) {
				pr_info("Not satisfy overloading battery voltage"
					" critiria (%dmV < %dmV).\n", vbat_mv,
					(chip->vfloat_mv - 100));
			} else {
				is_batt_full = false;
				eoc_count = eoc_count_by_curr = 0;
				pr_info("%s: Clear is_batt_full & eoc_count due to"
					" Overloading happened, soc=%d\n",
					__func__, soc);
				htc_gauge_event_notify(HTC_GAUGE_EVENT_EOC);
			}
		}
	}

	schedule_delayed_work(&chip->eoc_work,
		msecs_to_jiffies(EOC_CHECK_PERIOD_MS));

	return;

stop_eoc:
	eoc_count_by_curr = eoc_count = 0;

	/*Check if need to rerun APSD*/
	/*If yes, wait for batt_worker to retry APSD rerun 3 times*/
	smbchg_read(chip, &reg_rt_sts, chip->misc_base + RT_STS, 1);
	vbus_uv = pmi8994_get_usbin_voltage_now(the_chip);
	if ((!(reg_rt_sts & POWER_OK_BIT)) &&
		(vbus_uv > VBUS_VALID_LOWER_BOUND) &&
		is_HVDCP_9V_done(chip) &&
		(!pwrsrc_disabled) &&
		gs_aspd_rerun_count <= 3){
		pr_info("Wait for APSD rerun: 0x1610:0x%02x is_HVDCP_9V:%d Vbus:%d Count:%d pwrsrc_disabled:%d\n",
			reg_rt_sts, is_HVDCP_9V_done(chip), vbus_uv,gs_aspd_rerun_count,pwrsrc_disabled);
		schedule_delayed_work(&chip->eoc_work,
			msecs_to_jiffies(EOC_CHECK_PERIOD_MS));
		return;
	}
	wake_unlock(&chip->eoc_worker_wlock);
}

int pmi8994_is_batt_temperature_fault(int *result)
{
	if(!the_chip) {
		pr_smb(PR_STATUS, "called before init\n");
		return -EINVAL;
	}
	*result = get_prop_batt_health(the_chip);

	if (*result != POWER_SUPPLY_HEALTH_GOOD &&
			*result != POWER_SUPPLY_HEALTH_COOL &&
			*result != POWER_SUPPLY_HEALTH_WARM)
		*result = 1;
	else
		*result = 0;
	return 0;
}

#define BATT_CHG_WARM_THR_MV	4000
int pmi8994_is_batt_temp_fault_disable_chg(int *result)
{
	int batt_temp_status, vbat_mv, is_vbatt_over_vddmax;
	int is_cold = 0, is_hot = 0, is_warm = 0;
	int warm_thr_mv;

	if (!the_chip) {
		 pr_smb(PR_STATUS, "called before init\n");
		return -EINVAL;
	}

	batt_temp_status = get_prop_batt_health(the_chip);

	vbat_mv = smbchg_get_prop_voltage_now(the_chip) / 1000;

	if (batt_temp_status == POWER_SUPPLY_HEALTH_OVERHEAT)
		is_hot = 1;
	if (batt_temp_status == POWER_SUPPLY_HEALTH_WARM)
		is_warm = 1;
	if (batt_temp_status == POWER_SUPPLY_HEALTH_COLD)
		is_cold = 1;

	if (the_chip->vfloat_cmp_mv &&
			the_chip->vfloat_cmp_mv != -EINVAL)
		warm_thr_mv = the_chip->vfloat_cmp_mv;
	else
		warm_thr_mv = BATT_CHG_WARM_THR_MV;

	if(vbat_mv >= warm_thr_mv)
		is_vbatt_over_vddmax = true;
	else
		is_vbatt_over_vddmax = false;

	pr_smb(PR_STATUS, "is_cold=%d, is_hot=%d, is_warm=%d, "
			"is_vbatt_over_vddmax=%d\n",
			is_cold, is_hot, is_warm, is_vbatt_over_vddmax);
#if 1//FIXME
	if ((is_cold || is_hot || (is_warm && is_vbatt_over_vddmax)) &&
			!flag_keep_charge_on && !flag_disable_temp_protection)
#else
	if ((is_cold || is_hot || (is_warm && is_vbatt_over_vddmax)) &&
			!flag_keep_charge_on && !flag_pa_recharge)
#endif
		*result = 1;
	else
		*result = 0;

	return 0;
}

#define EN_BAT_CHG			BIT(1)
#define CMD_CHG			0x42
static int
smbchg_charge_en(struct smbchg_chip *chip, int enable)
{
	return smbchg_masked_write(chip, chip->bat_if_base + CMD_CHG,
			EN_BAT_CHG, enable ? 0 : EN_BAT_CHG);
}

static int
smbchg_force_run_on_batt(struct smbchg_chip *chip, int disable)
{
#if 0//FIXME

	/* Don't force on battery if battery is not present */
	if (!qpnp_chg_is_batt_present(chip))
		return 0;
#endif
	/* This bit forces the charger to run off of the battery rather
	 * than a connected charger */
	return smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL,
			USBIN_SUSPEND_BIT, disable ? USBIN_SUSPEND_BIT : 0);
}

/* htc: to protect BAT_FET sw enable/disable contorl cs */
static DEFINE_SPINLOCK(charger_lock);
static int batt_charging_disabled; /* BAT FET side */
/* sw disabled batt fet control mask (reason) */
#define BATT_CHG_DISABLED_BIT_EOC	(1)
#define BATT_CHG_DISABLED_BIT_KDRV	(1<<1)
#define BATT_CHG_DISABLED_BIT_USR1	(1<<2)
#define BATT_CHG_DISABLED_BIT_USR2	(1<<3)
#define BATT_CHG_DISABLED_BIT_BAT	(1<<4)

static int smbchg_disable_auto_enable(struct smbchg_chip *chip,
		int disable, int reason)
{
	int rc;
	unsigned long flags;

	spin_lock_irqsave(&charger_lock, flags);
	if (disable)
		batt_charging_disabled |= reason;	/* set bit */
	else
		batt_charging_disabled &= ~reason;	/* clr bit */
	rc = smbchg_charge_en(the_chip, !batt_charging_disabled);
	if (rc)
		pr_err("Failed rc=%d\n", rc);
	spin_unlock_irqrestore(&charger_lock, flags);

	return rc;
}

static DEFINE_SPINLOCK(pwrsrc_lock);
/* sw disabled vbus power source control mask (reason) */
#define PWRSRC_DISABLED_BIT_KDRV	(1)
#define PWRSRC_DISABLED_BIT_USER	(1<<1)
#define PWRSRC_DISABLED_BIT_AICL		(1<<2)
#define PWRSRC_DISABLED_BIT_FTM		(1<<3)



static int smbchg_disable_pwrsrc(struct smbchg_chip *chip,
		int disable, int reason)
{
	int rc;
	unsigned long flags;

	spin_lock_irqsave(&pwrsrc_lock, flags);
	if (disable)
		pwrsrc_disabled |= reason;	/* set bit */
	else
		pwrsrc_disabled &= ~reason;	/* clr bit */
	rc = smbchg_force_run_on_batt(chip, pwrsrc_disabled);
	if (rc)
		pr_err("Failed rc=%d\n", rc);
	spin_unlock_irqrestore(&pwrsrc_lock, flags);

	return rc;
}

int pmi8994_charger_enable(bool enable)
{
	int rc = 0;

	if (the_chip) {
		enable = !!enable;
		rc = smbchg_disable_auto_enable(the_chip, !enable, BATT_CHG_DISABLED_BIT_KDRV);
	} else {
		pr_err("called before init\n");
		rc = -EINVAL;
	}

	return rc;
}

/* htc charger interface + */
int pmi8994_pwrsrc_enable(bool enable)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	/* if disable pwrsrc by user,
	 * clean count_same_dischg to skip the WA of charger OCP detection
	 */
	if (!enable)
		count_same_dischg = 0;

	return smbchg_disable_pwrsrc(the_chip, !enable, PWRSRC_DISABLED_BIT_KDRV);
}

static void handle_usb_present_change(struct smbchg_chip *chip,
				int usb_present)
{
	int soc = 0;
	if (chip->usb_present ^ usb_present) {
		chip->usb_present = usb_present;

		/* Update usb_present and usb_online for power supply	  */
		/* to notify usb driver to do enumeration 				*/
		power_supply_set_present(chip->usb_psy, chip->usb_present);
		if (delayed_work_pending(&chip->usb_set_online_work))
				 cancel_delayed_work_sync(&chip->usb_set_online_work);
		schedule_delayed_work(&chip->usb_set_online_work, msecs_to_jiffies(100));

		if (!usb_present) {
			is_batt_full = false;
			is_batt_full_eoc_stop = false;
			is_qc20_done_flag = false;
			is_over_fastchg_thr_once = false;
			gs_is_limit_1A = false;
			/* Cancel retry_aicl_worker */
			retry_aicl_cnt = 0;
			gs_is_bad_aicl_result = false;
			if (delayed_work_pending(&chip->retry_aicl_work))
				cancel_delayed_work_sync(&chip->retry_aicl_work);
			/* Cancel smbchg_usb_limit_max_current_work */
			if (delayed_work_pending(&chip->usb_limit_max_current))
				cancel_delayed_work_sync(&chip->usb_limit_max_current);

			smbchg_aicl_deglitch_wa_check();
			chip->vbat_above_headroom = false;

			if (flag_force_ac_chg) {
				/* ATS WA: Re-enable AICL */
				smbchg_sec_masked_write(chip,
					chip->usb_chgpth_base + USB_AICL_CFG,
					AICL_EN_BIT, AICL_EN_BIT);
				pr_smb(PR_STATUS, "ATS: Re-enable AICL\n");
			}
		} else {
			wake_lock(&chip->eoc_worker_wlock);
			schedule_delayed_work(&chip->eoc_work,
				msecs_to_jiffies(EOC_CHECK_PERIOD_MS));

			smbchg_aicl_deglitch_wa_check();

			soc = smbchg_get_prop_capacity_now(the_chip);
			if (chip->previous_soc != soc) {
				chip->previous_soc = soc;
				smbchg_soc_changed(the_chip);
			}
		}
	}
}

static enum device_model check_device_model(void)
{
	const char *machine_name;
	enum device_model model = DEVICE_MODEL_INVALID;

	if(initial_boot_params) {
		machine_name = fdt_getprop(initial_boot_params, DT_ROOT_VALUE, "model", NULL);
		if (machine_name) {
			pr_info("Machine Name: %s (checked in battery driver)\n", machine_name);
			if (strstr(machine_name, "XA"))
				model = DEVICE_MODEL_XA;
			else if (strstr(machine_name, "XB"))
				model = DEVICE_MODEL_XB;
			else
				model = DEVICE_MODEL_XC_OR_ABOVE;
		}
		else
			pr_info("Machine Name not found (checked in battery driver)\n");
	}
	else
		pr_err("initial_boot_params is NULL\n");

	return model;
}

int pmi8994_set_pwrsrc_and_charger_enable(enum htc_power_source_type src,
		bool chg_enable, bool pwrsrc_enable)
{
	int mA = 0, rc = 0;
	enum power_supply_type usb_supply_type;
	u8 usb_type_reg = 0;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	smbchg_read(the_chip, &usb_type_reg, the_chip->misc_base + IDEV_STS, 1);
	usb_supply_type = get_usb_supply_type(usb_type_reg);

	/*Clear the bad AICL flag when ever power source change*/
	gs_is_bad_aicl_result = false;

	pr_smb(PR_STATUS, "src=%d, chg_enable=%d, pwrsrc_enable=%d, ftm_src=%d, "
					"usb_type=%d\n",
				src, chg_enable, pwrsrc_enable, the_chip->ftm_src,
				usb_supply_type);

	if (flag_force_ac_chg)
		src = htc_fake_charger_for_testing(src);

	if (the_chip->ftm_src == HTC_FTM_PWR_SOURCE_TYPE_USB ||
			the_chip->ftm_src == HTC_FTM_PWR_SOURCE_TYPE_AC)
		src = htc_fake_charger_for_ftm(src);

	if (src < HTC_PWR_SOURCE_TYPE_MHL_UNKNOWN) {
		/* STEP0: set pwrsrc enable/disable */
		smbchg_disable_pwrsrc(the_chip, !pwrsrc_enable, PWRSRC_DISABLED_BIT_KDRV);

		/* STEP1: Set BATFET */
		smbchg_disable_auto_enable(the_chip,
				!chg_enable, BATT_CHG_DISABLED_BIT_KDRV);
	}

	/* STEP2: set vbus draw */
	switch (src) {
	case HTC_PWR_SOURCE_TYPE_BATT:
		mA = USB_MA_2;
		break;
	case HTC_PWR_SOURCE_TYPE_WIRELESS:
		mA = USB_MA_500;
		break;
	case HTC_PWR_SOURCE_TYPE_DETECTING:
	case HTC_PWR_SOURCE_TYPE_UNKNOWN_USB:
	case HTC_PWR_SOURCE_TYPE_USB:
		if (the_chip->ftm_src == HTC_FTM_PWR_SOURCE_TYPE_USB)
			mA = USB_MA_500;
		else if (usb_supply_type == POWER_SUPPLY_TYPE_USB_CDP)
			mA = USB_MA_1000;
		else
			mA = USB_MA_500;
		break;
	case HTC_PWR_SOURCE_TYPE_AC:
	case HTC_PWR_SOURCE_TYPE_9VAC:
	case HTC_PWR_SOURCE_TYPE_MHL_AC:
		if (the_chip->ftm_src == HTC_FTM_PWR_SOURCE_TYPE_AC)
			mA = USB_MA_1000;
		else if (flag_force_ac_chg)
			mA = USB_MA_1000;
		else
			mA = USB_MA_1500;
		break;
	case HTC_PWR_SOURCE_TYPE_MHL_UNKNOWN:
		mA = USB_MA_0;
		break;
	case HTC_PWR_SOURCE_TYPE_MHL_100MA:
		mA = USB_MA_100;
		break;
	case HTC_PWR_SOURCE_TYPE_MHL_500MA:
		mA = USB_MA_500;
		break;
	case HTC_PWR_SOURCE_TYPE_MHL_900MA:
		mA = USB_MA_900;
		break;
	case HTC_PWR_SOURCE_TYPE_MHL_1500MA:
		mA = USB_MA_1500;
		break;
	case HTC_PWR_SOURCE_TYPE_MHL_2000MA:
		mA = USB_MA_1500;
		break;
	default:
		mA = USB_MA_2;
		break;
	}

	mutex_lock(&the_chip->current_change_lock);
	the_chip->power_source_type = src;
	the_chip->usb_target_current_ma = mA;
	rc = smbchg_set_thermal_limited_usb_current_max(the_chip, mA);
	if (rc < 0)
		pr_err("Couldn't set usb current rc = %d\n", rc);
	mutex_unlock(&the_chip->current_change_lock);

	if (delayed_work_pending(&the_chip->hvdcp_det_work))
		cancel_delayed_work_sync(&the_chip->hvdcp_det_work);

	if (usb_supply_type == POWER_SUPPLY_TYPE_USB_DCP)
		schedule_delayed_work(&the_chip->hvdcp_det_work,
					msecs_to_jiffies(HVDCP_NOTIFY_MS));

	if (HTC_PWR_SOURCE_TYPE_BATT == src)
		handle_usb_present_change(the_chip, 0);
	else if (HTC_PWR_SOURCE_TYPE_MHL_UNKNOWN <= src) {
		if (is_usb_present(the_chip)) {
			handle_usb_present_change(the_chip, 1);
		} else {
			handle_usb_present_change(the_chip, 0);
		}
	} else
		handle_usb_present_change(the_chip, 1);

	if (the_chip->ftm_src == HTC_FTM_PWR_SOURCE_TYPE_NONE_STOP)
		smbchg_disable_pwrsrc(the_chip, true, PWRSRC_DISABLED_BIT_FTM);
	else
		smbchg_disable_pwrsrc(the_chip, false, PWRSRC_DISABLED_BIT_FTM);

	return 0;
}

static int
pmi8994_get_usbin_voltage_now(struct smbchg_chip *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;

	if (IS_ERR_OR_NULL(chip->vadc_dev)) {
		chip->vadc_dev = qpnp_get_vadc(chip->dev, "usbin");
		if (IS_ERR(chip->vadc_dev))
			return PTR_ERR(chip->vadc_dev);
	}

	rc = qpnp_vadc_read(chip->vadc_dev, USBIN, &results);
	if (rc) {
		pr_err("Unable to read usbin rc=%d\n", rc);
		return 0;
	} else {
		return results.physical;
	}
}

static int determine_initial_status(struct smbchg_chip *chip)
{
	/*
	 * It is okay to read the interrupt status here since
	 * interrupts aren't requested. reading interrupt status
	 * clears the interrupt so be careful to read interrupt
	 * status only in interrupt handling code
	 */

	batt_pres_handler(0, chip);
	batt_hot_handler(0, chip);
	batt_warm_handler(0, chip);
	batt_cool_handler(0, chip);
	batt_cold_handler(0, chip);
	chg_term_handler(0, chip);
/*++ 2014/10/07 USB Team, PCN00014 ++*/
	/*usbid_change_handler(0, chip);*/
/*-- 2014/10/07 USB Team, PCN00014 --*/

#if !(defined(CONFIG_HTC_BATT_8960))
	chip->usb_present = is_usb_present(chip);
	chip->dc_present = is_dc_present(chip);

	if (chip->usb_present)
		handle_usb_insertion(chip);
	else
		handle_usb_removal(chip);
#endif

	return 0;
}

static int prechg_time[] = {
	24,
	48,
	96,
	192,
};
static int chg_time[] = {
	192,
	384,
	768,
	1536,
};

#define USBIN_CHGR_CFG			0xF1
#define USBIN_CHGR_MASK			SMB_MASK(2, 0)

enum bpd_type {
	BPD_TYPE_BAT_NONE,
	BPD_TYPE_BAT_ID,
	BPD_TYPE_BAT_THM,
	BPD_TYPE_BAT_THM_BAT_ID,
	BPD_TYPE_DEFAULT,
};

static const char * const bpd_label[] = {
	[BPD_TYPE_BAT_NONE]		= "bpd_none",
	[BPD_TYPE_BAT_ID]		= "bpd_id",
	[BPD_TYPE_BAT_THM]		= "bpd_thm",
	[BPD_TYPE_BAT_THM_BAT_ID]	= "bpd_thm_id",
};

static inline int get_bpd(const char *name)
{
	int i = 0;
	for (i = 0; i < ARRAY_SIZE(bpd_label); i++) {
		if (strcmp(bpd_label[i], name) == 0)
			return i;
	}
	return -EINVAL;
}

#define CHGR_CFG1			0xFB
#define RECHG_THRESHOLD_SRC_BIT		BIT(1)
#define TERM_I_SRC_BIT			BIT(2)
#define TERM_SRC_FG    			BIT(2)
#define CHGR_CFG2			0xFC
#define CHG_INHIB_CFG_REG		0xF7
#define CHG_INHIBIT_50MV_VAL		0x00
#define CHG_INHIBIT_100MV_VAL		0x01
#define CHG_INHIBIT_200MV_VAL		0x02
#define CHG_INHIBIT_300MV_VAL		0x03
#define CHG_INHIBIT_MASK		0x03
#define USE_REGISTER_FOR_CURRENT	BIT(2)
#define CHG_EN_SRC_BIT			BIT(7)
#define CHG_EN_COMMAND_BIT		BIT(6)
#define P2F_CHG_TRAN			BIT(5)
#define I_TERM_BIT			BIT(3)
#define AUTO_RECHG_BIT			BIT(2)
#define CHARGER_INHIBIT_BIT		BIT(0)
#define CFG_TCC_REG			0xF9
#define CHG_ITERM_50MA			0x1
#define CHG_ITERM_100MA			0x2
#define CHG_ITERM_150MA			0x3
#define CHG_ITERM_200MA			0x4
#define CHG_ITERM_250MA			0x5
#define CHG_ITERM_300MA			0x0
#define CHG_ITERM_500MA			0x6
#define CHG_ITERM_600MA			0x7
#define CHG_ITERM_MASK			SMB_MASK(2, 0)
#define USBIN_SUSPEND_SRC_BIT		BIT(6)
#define USB51_COMMAND_POL		BIT(2)
#define USB51AC_CTRL			BIT(1)
#define SFT_CFG				0xFD
#define TR_8OR32B			0xFE
#define BUCK_8_16_FREQ_BIT		BIT(0)
#define SFT_EN_MASK			SMB_MASK(5, 4)
#define SFT_TO_MASK			SMB_MASK(3, 2)
#define PRECHG_SFT_TO_MASK		SMB_MASK(1, 0)
#define SFT_TIMER_DISABLE_BIT		BIT(5)
#define PRECHG_SFT_TIMER_DISABLE_BIT	BIT(4)
#define SAFETY_TIME_MINUTES_SHIFT	2
#define BM_CFG				0xF3
#define BATT_MISSING_ALGO_BIT		BIT(2)
#define BMD_PIN_SRC_MASK		SMB_MASK(1, 0)
#define PIN_SRC_SHIFT			0
#define CHGR_CFG			0xFF
#define RCHG_LVL_BIT			BIT(0)
#define CFG_AFVC			0xF5
#define VFLOAT_COMP_ENABLE_MASK		SMB_MASK(2, 0)
#define TR_RID_REG			0xFA
#define FG_INPUT_FET_DELAY_BIT		BIT(3)
#define TRIM_OPTIONS_7_0		0xF6
#define INPUT_MISSING_POLLER_EN_BIT	BIT(3)
#define CCMP_CFG                             0xFA
#define TEMP_MONITOR_DISABLE_BIT             0
#define JEITA_TEMP_HARD_LIMIT_DISABLE_BIT    BIT(5)
#define HOT_COLD_SL_COMP_BITS                0
#define HARD_TEMP_MASK                       SMB_MASK(6, 5)
#define SOFT_TEMP_MASK                       SMB_MASK(3, 0)
#define POWER_OK_BIT                        BIT(0)
#define VBUS_ALLOW_CHANGE_MAX               2000000
#define USBIN_CHGR_CFG                      0xF1
#define CHGR_ERROR_BIT		BIT(0)
#define AICL_WL_SEL_CFG			0xF5
#define AICL_WL_SEL_MASK		SMB_MASK(1, 0)
#define AICL_WL_SEL_45S		0

int pmi8994_is_power_ok_check(void)
{
	u8 reg = 0;
	int prev_vbus_uv, vbus_uv, i, diff;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	/* spend 100ms to filter out unstable vbus
	 * if vbus is vibrating, return 0 (no need to error handle).
	 */
	for(i = 0; i < 3; i++) {
		vbus_uv = pmi8994_get_usbin_voltage_now(the_chip);
		if (i > 0) {
			diff = vbus_uv - prev_vbus_uv;
			if (ABS(diff) >= VBUS_ALLOW_CHANGE_MAX) {
				pr_info("%s: vbus is vibrating. prev_vbus_uv(%d) "
					"vbus_uv(%d)\n", __func__, prev_vbus_uv, vbus_uv);
				return 0;
			}
		}
		prev_vbus_uv = vbus_uv;
		msleep(50);
	}

	smbchg_read(the_chip, &reg, the_chip->misc_base + RT_STS, 1);

	/* no need to error handle when
	 * 1. power_ok is 1 and vbus is high
	 * 2. power_ok is 0 and vbus is low
	 */
	pr_smb(PR_STATUS, "0x1610: 0x%02x, vbus=%d\n",
			reg, vbus_uv);
	if (((reg & POWER_OK_BIT) && (vbus_uv > VBUS_VALID_LOWER_BOUND)) ||
			(((reg & POWER_OK_BIT) == 0) && (vbus_uv <= VBUS_VALID_LOWER_BOUND)))
		return 0;
	else
		return 1;
}

int pmi8994_is_batt_over_charged_check(void)
{
	u8 chgr_rt_sts, bat_if_rt_sts;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	/* battery voltage is really over float voltage. */
	if (svbat_mv > the_chip->vfloat_mv)
		return 0;

	smbchg_read(the_chip, &chgr_rt_sts, the_chip->chgr_base + RT_STS, 1);
	smbchg_read(the_chip, &bat_if_rt_sts, the_chip->bat_if_base + RT_STS, 1);

	/*[WA] When both chg_err bit and bat_ov bit are 1,                                  *
	 *  but battery voltage isn't really over charged.                                         *
	 * Force stop charging and enable charging for trying to resume charging.*/
	if ((chgr_rt_sts & CHGR_ERROR_BIT) && (bat_if_rt_sts & BAT_OV_BIT)) {
		pr_smb(PR_STATUS, "batt_ov: 0x1010=0x%02x, 0x1210=0x%02x\n",
				chgr_rt_sts, bat_if_rt_sts);
		return 1;
	} else
		return 0;
}

int pmi8994_is_charger_error_handle(void)
{
	int rc = -1;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	if (pmi8994_is_power_ok_check() == 1){
		/*reurn APSD only when charging enabled*/
		/*retry only 3 times */
		if(!pwrsrc_disabled && gs_aspd_rerun_count++ < 3){
			/* re-run APSD */
			rc = rerun_apsd(the_chip);
			if (rc) {
				pr_err("APSD rerun failed\n");
				return rc;
			}
		}
		return 1;
	}
	else if (pmi8994_is_batt_over_charged_check() == 1){
		gs_aspd_rerun_count = 0;
		return 1;
	}
	else {
		gs_aspd_rerun_count = 0;
		return 0;
	}
}

#define ICL_OVERRIDE_BIT	BIT(2)
#define USBIN_MODE_CHG_BIT  BIT(0)

int pmi8994_usbin_mode_charge(void)
{
	int rc;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	/* 0x1340 Bit2 set to 1 & Bit0 set to 1 */
	rc = smbchg_masked_write(the_chip, the_chip->usb_chgpth_base + CMD_IL,
			ICL_OVERRIDE_BIT, ICL_OVERRIDE_BIT);
	if (rc < 0) {
		dev_err(the_chip->dev, "Couldn't set 0x1340 ICL override bit rc=%d\n",
				rc);
		return rc;
	}

	rc = smbchg_masked_write(the_chip, the_chip->usb_chgpth_base + CMD_IL,
			USBIN_MODE_CHG_BIT, USBIN_MODE_CHG_BIT);
	if (rc < 0) {
		dev_err(the_chip->dev, "Couldn't set 0x1340 usbin mode charge bit rc=%d\n",
				rc);
		return rc;
	}

	return 0;
}

int pmi8994_reset_chg_en_when_chg_error(void)
{
	int rc;
	u8 chgr_rt_sts;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	/* battery voltage only allows to be lower than 4.0V. */
	if (svbat_mv >= VOLT_ALLOW_RESET_CHG_ERROR_BIT)
		return 0;

	smbchg_read(the_chip, &chgr_rt_sts, the_chip->chgr_base + RT_STS, 1);

	/*[WA] When chg_err bit is 1,
	 * Force 0x10FC CHG_EN_COMMAND bit changed from 1 to 0 to 1
	 * for trying to resume charging.
	 * Safety timer expired not allow to reset chg error bit
	 */
	if (chgr_rt_sts & CHGR_ERROR_BIT && ((chgr_rt_sts & CHG_SFT_RT_STS) == 0)) {
		rc = smbchg_sec_masked_write(the_chip, the_chip->chgr_base + CHGR_CFG2,
				CHG_EN_COMMAND_BIT, 0);
		if (rc < 0) {
			dev_err(the_chip->dev, "Couldn't set 0x10FC CHG_EN_COMMAND bit rc=%d\n",
					rc);
			return rc;
		}

		rc = smbchg_sec_masked_write(the_chip, the_chip->chgr_base + CHGR_CFG2,
				CHG_EN_COMMAND_BIT, CHG_EN_COMMAND_BIT);
		if (rc < 0) {
			dev_err(the_chip->dev, "Couldn't set 0x10FC CHG_EN_COMMAND bit rc=%d\n",
					rc);
			return rc;
		}
	}
	pr_smb(PR_STATUS, "chgr_rt_sts: 0x1010=0x%02x\n", chgr_rt_sts);

	return 0;
}

int pmi8994_set_aicl_deglitch_wa_check(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	/*check aicl deglitch to set it when Vbat >= 4.2V*/
	if(!the_chip->aicl_deglitch_short)
		smbchg_aicl_deglitch_wa_check();

	return 0;
}

int pmi8994_set_safety_timer_disable(int disable)
{
	int rc, i;
	u8 reg;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	if (is_safety_timer_disable != disable) {
		pr_info("%s(%d -> %d)\n", __func__, is_safety_timer_disable, disable);
		is_safety_timer_disable = disable;

		/* set the safety time voltage */
		if (the_chip->safety_time != -EINVAL) {
			reg = (the_chip->safety_time > 0 ? 0 : SFT_TIMER_DISABLE_BIT) |
				(the_chip->prechg_safety_time > 0
				? 0 : PRECHG_SFT_TIMER_DISABLE_BIT);

			/* debug flag 6 4 set, disable safety timer & pre-charge timer */
			if (flag_keep_charge_on | is_safety_timer_disable
					| flag_disable_safety_timer)
				reg = SFT_TIMER_DISABLE_BIT;

			for (i = 0; i < ARRAY_SIZE(chg_time); i++) {
				if (the_chip->safety_time <= chg_time[i]) {
					reg |= i << SAFETY_TIME_MINUTES_SHIFT;
					break;
				}
			}
			for (i = 0; i < ARRAY_SIZE(prechg_time); i++) {
				if (the_chip->prechg_safety_time <= prechg_time[i]) {
					reg |= i;
					break;
				}
			}

			rc = smbchg_sec_masked_write(the_chip,
					the_chip->chgr_base + SFT_CFG,
					SFT_EN_MASK | SFT_TO_MASK |
					(the_chip->prechg_safety_time > 0
					? PRECHG_SFT_TO_MASK : 0), reg);
			if (rc < 0) {
				dev_err(the_chip->dev,
					"Couldn't set safety timer rc = %d\n",
					rc);
				return rc;
			}
		}
	}

	return 0;
}

static int smbchg_hw_init(struct smbchg_chip *chip)
{
	int rc, i;
	u8 reg, mask;

	rc = smbchg_read(chip, chip->revision,
			chip->misc_base + REVISION1_REG, 4);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read revision rc=%d\n",
				rc);
		return rc;
	}
	pr_smb(PR_STATUS, "Charger Revision DIG: %d.%d; ANA: %d.%d\n",
			chip->revision[DIG_MAJOR], chip->revision[DIG_MINOR],
			chip->revision[ANA_MAJOR], chip->revision[ANA_MINOR]);

	rc = smbchg_sec_masked_write(chip,
			chip->dc_chgpth_base + AICL_WL_SEL_CFG,
			AICL_WL_SEL_MASK, AICL_WL_SEL_45S);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set AICL rerun timer rc=%d\n",
				rc);
		return rc;
	}

	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + TR_RID_REG,
			FG_INPUT_FET_DELAY_BIT, FG_INPUT_FET_DELAY_BIT);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't disable fg input fet delay rc=%d\n",
				rc);
		return rc;
	}

#ifdef CONFIG_HTC_BATT_8960
	rc = smbchg_sec_masked_write(chip, chip->misc_base + TRIM_OPTIONS_7_0,
				INPUT_MISSING_POLLER_EN_BIT, INPUT_MISSING_POLLER_EN_BIT);
#else
	rc = smbchg_sec_masked_write(chip, chip->misc_base + TRIM_OPTIONS_7_0,
			INPUT_MISSING_POLLER_EN_BIT, 0);
#endif

	if (rc < 0) {
		dev_err(chip->dev, "Couldn't disable input missing poller rc=%d\n",
				rc);
		return rc;
	}

	/*
	 * force using current from the register i.e. ignore auto
	 * power source detect (APSD) mA ratings
	 */
	reg = USE_REGISTER_FOR_CURRENT;

	/* FIXME: Per QCT's comment, use result of AICL for current. */
	rc = smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL,
			USE_REGISTER_FOR_CURRENT, USE_REGISTER_FOR_CURRENT);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set input limit cmd rc=%d\n", rc);
		return rc;
	}

	/*
	 * set chg en by cmd register, set chg en by writing bit 1,
	 * enable auto pre to fast, enable current termination, enable
	 * auto recharge, disable chg inhibition
	 */
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + CHGR_CFG2,
			CHG_EN_SRC_BIT | CHG_EN_COMMAND_BIT | P2F_CHG_TRAN
			| I_TERM_BIT | AUTO_RECHG_BIT | CHARGER_INHIBIT_BIT,
			CHG_EN_COMMAND_BIT
			| (chip->iterm_disabled ? I_TERM_BIT : 0));
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set chgr_cfg2 rc=%d\n", rc);
		return rc;
	}

	/*
	 * use the analog sensors instead of the fuelgauge adcs so that
	 * tcc detection works without trimmed parts
	 */
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + CHGR_CFG1,
			TERM_I_SRC_BIT | RECHG_THRESHOLD_SRC_BIT, TERM_SRC_FG);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set chgr_cfg2 rc=%d\n", rc);
		return rc;
	}

	/*
	 * control USB suspend via command bits and set correct 100/500mA
	 * polarity on the usb current
	 */
	/* FIXME: Per QCT's comment, force charge even batt_id is unknown.
	     Need confirm whether remove this setting in the future with HW. */
	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + CHGPTH_CFG,
		USBIN_SUSPEND_SRC_BIT | USB51_COMMAND_POL | USB51AC_CTRL,
		0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set usb_chgpth cfg rc=%d\n", rc);
		return rc;
	}

	if (chip->usbin_chgr_cfg != -EINVAL) {
		rc = smbchg_sec_masked_write(chip,
				chip->usb_chgpth_base + USBIN_CHGR_CFG,
				USBIN_CHGR_MASK, chip->usbin_chgr_cfg);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't set usb_chgpth cfg rc=%d\n", rc);
			return rc;
		}
	}

	/* set the float voltage */
	if (chip->vfloat_mv != -EINVAL) {
		rc = smbchg_float_voltage_set(chip, chip->vfloat_mv);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set float voltage rc = %d\n", rc);
			return rc;
		}
	}

	/* set the float voltage cmp */
	if (chip->vfloat_cmp_mv != -EINVAL) {
		rc = smbchg_float_voltage_cmp_set(chip, chip->vfloat_cmp_mv);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set float voltage cmp rc = %d\n", rc);
			return rc;
		}
	}

	/* set iusb max current */
	if (chip->usb_max_current_ma > 0) {
		rc = smbchg_set_high_usb_chg_current(chip, chip->usb_max_current_ma);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set iusb max current rc = %d\n", rc);
			return rc;
		}
		pr_smb(PR_STATUS, "set iusb max current to %d\n",
								chip->usb_max_current_ma);
	}

	/* set ibatmax current for warm batt_temp */
	if (chip->warm_ibatmax > 0) {
		rc = smbchg_warm_ibatmax_set(chip, chip->warm_ibatmax);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set ibatmax current for warm batt_temp rc = %d\n", rc);
			return rc;
		}
	}

	/* set iterm */
	if (chip->iterm_ma != -EINVAL) {
		if (chip->iterm_disabled) {
			dev_err(chip->dev, "Error: Both iterm_disabled and iterm_ma set\n");
			return -EINVAL;
		} else {
			if (chip->iterm_ma <= 50)
				reg = CHG_ITERM_50MA;
			else if (chip->iterm_ma <= 100)
				reg = CHG_ITERM_100MA;
			else if (chip->iterm_ma <= 150)
				reg = CHG_ITERM_150MA;
			else if (chip->iterm_ma <= 200)
				reg = CHG_ITERM_200MA;
			else if (chip->iterm_ma <= 250)
				reg = CHG_ITERM_250MA;
			else if (chip->iterm_ma <= 300)
				reg = CHG_ITERM_300MA;
			else if (chip->iterm_ma <= 500)
				reg = CHG_ITERM_500MA;
			else
				reg = CHG_ITERM_600MA;

			rc = smbchg_sec_masked_write(chip,
					chip->chgr_base + CFG_TCC_REG,
					CHG_ITERM_MASK, reg);
			if (rc) {
				dev_err(chip->dev,
					"Couldn't set iterm rc = %d\n", rc);
				return rc;
			}
			pr_smb(PR_STATUS, "set tcc (%d) to 0x%02x\n",
					chip->iterm_ma, reg);
		}
	}

	/* set the safety time voltage */
	if (chip->safety_time != -EINVAL) {
		reg = (chip->safety_time > 0 ? 0 : SFT_TIMER_DISABLE_BIT) |
			(chip->prechg_safety_time > 0
			? 0 : PRECHG_SFT_TIMER_DISABLE_BIT);

		/* debug flag 6 4 set, disable safety timer & pre-charge timer */
		if (flag_keep_charge_on | flag_disable_safety_timer)
			reg = SFT_TIMER_DISABLE_BIT;

		for (i = 0; i < ARRAY_SIZE(chg_time); i++) {
			if (chip->safety_time <= chg_time[i]) {
				reg |= i << SAFETY_TIME_MINUTES_SHIFT;
				break;
			}
		}
		for (i = 0; i < ARRAY_SIZE(prechg_time); i++) {
			if (chip->prechg_safety_time <= prechg_time[i]) {
				reg |= i;
				break;
			}
		}

		rc = smbchg_sec_masked_write(chip,
				chip->chgr_base + SFT_CFG,
				SFT_EN_MASK | SFT_TO_MASK |
				(chip->prechg_safety_time > 0
				? PRECHG_SFT_TO_MASK : 0), reg);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set safety timer rc = %d\n",
				rc);
			return rc;
		}
	}

	/* make the buck switch faster to prevent some vbus oscillation */
	rc = smbchg_sec_masked_write(chip,
			chip->usb_chgpth_base + TR_8OR32B,
			BUCK_8_16_FREQ_BIT, 0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set buck frequency rc = %d\n", rc);
		return rc;
	}

	/* battery missing detection */
	mask =  BATT_MISSING_ALGO_BIT;
	reg = chip->bmd_algo_disabled ? BATT_MISSING_ALGO_BIT : 0;
	if (chip->bmd_pin_src < BPD_TYPE_DEFAULT) {
		mask |= BMD_PIN_SRC_MASK;
		reg |= chip->bmd_pin_src << PIN_SRC_SHIFT;
	}
	rc = smbchg_sec_masked_write(chip,
			chip->bat_if_base + BM_CFG, mask, reg);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set batt_missing config = %d\n",
									rc);
		return rc;
	}

	smbchg_charging_status_change(chip);

	/*
	 * The charger needs 20 milliseconds to go into battery supplementary
	 * mode. Sleep here until we are sure it takes into effect.
	 */
	msleep(20);
	smbchg_usb_en(chip, chip->chg_enabled, REASON_USER);
	smbchg_dc_en(chip, chip->chg_enabled, REASON_USER);
	/* resume threshold */
	if (chip->resume_delta_mv != -EINVAL) {
		rc = smbchg_sec_masked_write(chip,
				chip->chgr_base + CHGR_CFG,
				RCHG_LVL_BIT, (chip->resume_delta_mv < 200)
				? 0 : RCHG_LVL_BIT);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't set recharge rc = %d\n",
					rc);
			return rc;
		}
	}

  /* debug flag 6 4 set, disable hw soft/hard temp monitor */
	if (flag_keep_charge_on | flag_disable_temp_protection) {
		reg = TEMP_MONITOR_DISABLE_BIT |
			JEITA_TEMP_HARD_LIMIT_DISABLE_BIT | HOT_COLD_SL_COMP_BITS;
		rc = smbchg_sec_masked_write(chip, chip->chgr_base + CCMP_CFG,
			HARD_TEMP_MASK | SOFT_TEMP_MASK, reg);

		if (rc < 0) {
			dev_err(chip->dev, "Couldn't disable hw soft/hard temp monitor = %d\n",
				rc);
			return rc;
		}
	}

	if ((chip->target_fastchg_current_ma > ATS_IBAT_LIMIT) && (flag_ats_limit_chg))
		chip->target_fastchg_current_ma = ATS_IBAT_LIMIT;
	/* Set FCC (Fast-Charging Current), it actually means IBAT_MAX */
	rc = smbchg_set_fastchg_current(chip, chip->target_fastchg_current_ma);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set fastchg current = %d\n", rc);
		return rc;
	}
	chip->fastchg_current_ma = chip->target_fastchg_current_ma;

	/* DC path current settings */
	if (chip->dc_psy_type != -EINVAL) {
		rc = smbchg_set_thermal_limited_dc_current_max(chip,
						chip->dc_target_current_ma);
		if (rc < 0) {
			dev_err(chip->dev, "can't set dc current: %d\n", rc);
			return rc;
		}
	}


	/*
	 * on some devices the battery is powered via external sources which
	 * could raise its voltage above the float voltage. smbchargers go
	 * in to reverse boost in such a situation and the workaround is to
	 * disable float voltage compensation (note that the battery will appear
	 * hot/cold when powered via external source).
	 */
	if (chip->soft_vfloat_comp_disabled) {
		rc = smbchg_sec_masked_write(chip, chip->chgr_base + CFG_AFVC,
				VFLOAT_COMP_ENABLE_MASK, 0);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't disable soft vfloat rc = %d\n",
					rc);
			return rc;
		}
	}


	return rc;
}

static struct of_device_id smbchg_match_table[] = {
	{
		.compatible	= "qcom,qpnp-smbcharger",
		.data		= (void *)ARRAY_SIZE(usb_current_table),
	},
	{ },
};

#define DC_MA_MIN 300
#define DC_MA_MAX 2000
#define OF_PROP_READ(chip, prop, dt_property, retval, optional)		\
do {									\
	if (retval)							\
		break;							\
	if (optional)							\
		prop = -EINVAL;						\
									\
	retval = of_property_read_u32(chip->spmi->dev.of_node,		\
					"qcom," dt_property	,	\
					&prop);				\
									\
	if ((retval == -EINVAL) && optional)				\
		retval = 0;						\
	else if (retval)						\
		dev_err(chip->dev, "Error reading " #dt_property	\
				" property rc = %d\n", rc);		\
} while (0)

static int smb_parse_dt(struct smbchg_chip *chip)
{
	int rc = 0;
	struct device_node *node = chip->dev->of_node;
	const char *dc_psy_type, *bpd;

	if (!node) {
		dev_err(chip->dev, "device tree info. missing\n");
		return -EINVAL;
	}

	/* read optional u32 properties */
	OF_PROP_READ(chip, chip->iterm_ma, "iterm-ma", rc, 1);
	OF_PROP_READ(chip, chip->eoc_ibat_thre_ma, "eoc-ibat-thre-ma", rc, 1);
	OF_PROP_READ(chip, chip->target_fastchg_current_ma,
			"fastchg-current-ma", rc, 1);
	OF_PROP_READ(chip, chip->high_fastchg_cur_ma,
			"high-fastchg-current-ma", rc, 1);
	OF_PROP_READ(chip, chip->adjust_fastchg_cur_thr_mv,
			"adjust-fastchg-thr-mv", rc, 1);
	OF_PROP_READ(chip, chip->vfloat_mv, "float-voltage-mv", rc, 1);
	OF_PROP_READ(chip, chip->vfloat_cmp_mv, "float-voltage-cmp-mv", rc, 1);
	OF_PROP_READ(chip, chip->usb_max_current_ma, "iusb-max-ma", rc, 1);
	OF_PROP_READ(chip, chip->safety_time, "charging-timeout-mins", rc, 1);
	OF_PROP_READ(chip, chip->rpara_uohm, "rparasitic-uohm", rc, 1);
	OF_PROP_READ(chip, chip->prechg_safety_time, "precharging-timeout-mins",
			rc, 1);
	OF_PROP_READ(chip, chip->warm_ibatmax, "warm-ibatmax", rc, 1);
	OF_PROP_READ(chip, chip->kddi_limit1_temp, "kddi-limit1-temp", rc, 1);
	OF_PROP_READ(chip, chip->kddi_limit1_current, "kddi-limit1-current", rc, 1);
	OF_PROP_READ(chip, chip->kddi_limit1_current_qc20, "kddi-limit1-current-qc20", rc, 1);
	OF_PROP_READ(chip, chip->kddi_limit2_temp, "kddi-limit2-temp", rc, 1);
	OF_PROP_READ(chip, chip->kddi_limit2_current, "kddi-limit2-current", rc, 1);
	OF_PROP_READ(chip, chip->kddi_limit2_current_qc20, "kddi-limit2-current-qc20", rc, 1);
	OF_PROP_READ(chip, chip->qc20_usb_max_current_ma, "qc20-iusb-max-ma", rc, 1);
	if (chip->qc20_usb_max_current_ma == -EINVAL) {
		// device tree has no this item, so set IUSB as 1000mA
		dev_err(chip->dev, "qc20-iusb-max-ma missing\n");
		chip->qc20_usb_max_current_ma = USB_MA_1000;
	}

	if (chip->usb_max_current_ma != -EINVAL) {
		pre_usb_max_current_ma = chip->usb_max_current_ma;
	}

	if (chip->safety_time != -EINVAL &&
		(chip->safety_time > chg_time[ARRAY_SIZE(chg_time) - 1])) {
		dev_err(chip->dev, "Bad charging-timeout-mins %d\n",
						chip->safety_time);
		return -EINVAL;
	}

	if (chip->prechg_safety_time != -EINVAL &&
		(chip->prechg_safety_time >
		 prechg_time[ARRAY_SIZE(prechg_time) - 1])) {
		dev_err(chip->dev, "Bad precharging-timeout-mins %d\n",
						chip->prechg_safety_time);
		return -EINVAL;
	}
	OF_PROP_READ(chip, chip->resume_delta_mv, "resume-delta-mv", rc, 1);
	OF_PROP_READ(chip, chip->parallel.min_current_thr_ma,
			"parallel-usb-min-current-ma", rc, 1);
	/* Not support this feature yet. */
#if !(defined(CONFIG_HTC_BATT_8960))
	if (chip->parallel.min_current_thr_ma != -EINVAL)
		chip->parallel.avail = true;
#else
	chip->parallel.avail = false;
#endif

	/* read boolean configuration properties */
	chip->use_vfloat_adjustments = of_property_read_bool(node,
						"qcom,autoadjust-vfloat");
	chip->bmd_algo_disabled = of_property_read_bool(node,
						"qcom,bmd-algo-disabled");
	chip->iterm_disabled = of_property_read_bool(node,
						"qcom,iterm-disabled");
	chip->soft_vfloat_comp_disabled = of_property_read_bool(node,
					"qcom,soft-vfloat-comp-disabled");
	chip->chg_enabled = !(of_property_read_bool(node,
						"qcom,charging-disabled"));
	chip->low_volt_dcin = of_property_read_bool(node,
					"qcom,low-volt-dcin");

	/* parse the battery missing detection pin source */
	rc = of_property_read_string(chip->spmi->dev.of_node,
		"qcom,bmd-pin-src", &bpd);
	if (rc) {
		/* Select BAT_THM as default BPD scheme */
		chip->bmd_pin_src = BPD_TYPE_DEFAULT;
		rc = 0;
	} else {
		chip->bmd_pin_src = get_bpd(bpd);
		if (chip->bmd_pin_src < 0) {
			dev_err(chip->dev,
				"failed to determine bpd schema %d\n", rc);
			return rc;
		}
	}
	dev_err(chip->dev,
					"bmd_pin_src=%d, rc=%d\n", chip->bmd_pin_src, rc);

	/* parse the dc power supply configuration */
	rc = of_property_read_string(node, "qcom,dc-psy-type", &dc_psy_type);
	if (rc) {
		chip->dc_psy_type = -EINVAL;
		rc = 0;
	} else {
		if (strcmp(dc_psy_type, "Mains") == 0)
			chip->dc_psy_type = POWER_SUPPLY_TYPE_MAINS;
		else if (strcmp(dc_psy_type, "Wireless") == 0)
			chip->dc_psy_type = POWER_SUPPLY_TYPE_WIRELESS;
		else if (strcmp(dc_psy_type, "Wipower") == 0)
			chip->dc_psy_type = POWER_SUPPLY_TYPE_WIPOWER;
	}
	if (chip->dc_psy_type != -EINVAL) {
		OF_PROP_READ(chip, chip->dc_target_current_ma,
				"dc-psy-ma", rc, 0);
		if (rc)
			return rc;
		if (chip->dc_target_current_ma < DC_MA_MIN
				|| chip->dc_target_current_ma > DC_MA_MAX) {
			dev_err(chip->dev, "Bad dc mA %d\n",
					chip->dc_target_current_ma);
			return -EINVAL;
		}
	}

	OF_PROP_READ(chip, chip->usbin_chgr_cfg, "usbin-chgr-cfg", rc, 0);

	/* read the bms power supply name */
	rc = of_property_read_string(node, "qcom,bms-psy-name",
						&chip->bms_psy_name);
	if (rc)
		chip->bms_psy_name = NULL;

	/* read the bms power supply name */
	rc = of_property_read_string(node, "qcom,battery-psy-name",
						&chip->battery_psy_name);
	if (rc)
		chip->battery_psy_name = "battery";

	if (of_find_property(node, "qcom,thermal-mitigation",
					&chip->thermal_levels)) {
		chip->thermal_mitigation = devm_kzalloc(chip->dev,
			chip->thermal_levels,
			GFP_KERNEL);

		if (chip->thermal_mitigation == NULL) {
			dev_err(chip->dev, "thermal mitigation kzalloc() failed.\n");
			return -ENOMEM;
		}

		chip->thermal_levels /= sizeof(int);
		rc = of_property_read_u32_array(node,
				"qcom,thermal-mitigation",
				chip->thermal_mitigation, chip->thermal_levels);
		if (rc) {
			dev_err(chip->dev,
				"Couldn't read threm limits rc = %d\n", rc);
			return rc;
		}
	}

	return 0;
}

#define SUBTYPE_REG			0x5
#define SMBCHG_CHGR_SUBTYPE		0x1
#define SMBCHG_OTG_SUBTYPE		0x8
#define SMBCHG_BAT_IF_SUBTYPE		0x3
#define SMBCHG_USB_CHGPTH_SUBTYPE	0x4
#define SMBCHG_DC_CHGPTH_SUBTYPE	0x5
#define SMBCHG_MISC_SUBTYPE		0x7
#define REQUEST_IRQ(chip, resource, irq_num, irq_name, irq_handler, flags, rc)\
do {									\
	irq_num = spmi_get_irq_byname(chip->spmi,			\
					resource, irq_name);		\
	if (irq_num < 0) {						\
		dev_err(chip->dev, "Unable to get " irq_name " irq\n");	\
		return -ENXIO;						\
	}								\
	rc = devm_request_threaded_irq(chip->dev,			\
			irq_num, NULL, irq_handler, flags, irq_name,	\
			chip);						\
	if (rc < 0) {							\
		dev_err(chip->dev, "Unable to request " irq_name " irq: %d\n",\
				rc);					\
		return -ENXIO;						\
	}								\
} while (0)

static int smbchg_request_irqs(struct smbchg_chip *chip)
{
	int rc = 0;
	struct resource *resource;
	struct spmi_resource *spmi_resource;
	u8 subtype;
	struct spmi_device *spmi = chip->spmi;
	unsigned long flags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING
							| IRQF_ONESHOT;

	spmi_for_each_container_dev(spmi_resource, chip->spmi) {
		if (!spmi_resource) {
				dev_err(chip->dev, "spmi resource absent\n");
			return rc;
		}

		resource = spmi_get_resource(spmi, spmi_resource,
						IORESOURCE_MEM, 0);
		if (!(resource && resource->start)) {
			dev_err(chip->dev, "node %s IO resource absent!\n",
				spmi->dev.of_node->full_name);
			return rc;
		}

		rc = smbchg_read(chip, &subtype,
				resource->start + SUBTYPE_REG, 1);
		if (rc) {
			dev_err(chip->dev, "Peripheral subtype read failed rc=%d\n",
					rc);
			return rc;
		}

		switch (subtype) {
		case SMBCHG_CHGR_SUBTYPE:
			REQUEST_IRQ(chip, spmi_resource, chip->chg_error_irq,
				"chg-error", chg_error_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->taper_irq,
				"chg-taper-thr", taper_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->chg_term_irq,
				"chg-tcc-thr", chg_term_handler, flags, rc);
/* Remove chg_inhibit_irq because of PMI8994 known issue */
#if 0
			REQUEST_IRQ(chip, spmi_resource, chip->chg_inhibit_irq,
				"chg-inhibit", chg_inhibit_handler, flags, rc);
#endif
			REQUEST_IRQ(chip, spmi_resource, chip->recharge_irq,
				"chg-rechg-thr", recharge_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->fastchg_irq,
				"chg-p2f-thr", fastchg_handler, flags, rc);
			enable_irq_wake(chip->chg_term_irq);
			enable_irq_wake(chip->chg_error_irq);
			enable_irq_wake(chip->fastchg_irq);
			break;
		case SMBCHG_BAT_IF_SUBTYPE:
			REQUEST_IRQ(chip, spmi_resource, chip->batt_hot_irq,
				"batt-hot", batt_hot_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->batt_warm_irq,
				"batt-warm", batt_warm_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->batt_cool_irq,
				"batt-cool", batt_cool_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->batt_cold_irq,
				"batt-cold", batt_cold_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->batt_missing_irq,
				"batt-missing", batt_pres_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->vbat_low_irq,
				"batt-low", vbat_low_handler, flags, rc);
			enable_irq_wake(chip->batt_hot_irq);
			enable_irq_wake(chip->batt_warm_irq);
			enable_irq_wake(chip->batt_cool_irq);
			enable_irq_wake(chip->batt_cold_irq);
			enable_irq_wake(chip->batt_missing_irq);
			enable_irq_wake(chip->vbat_low_irq);
			break;
		case SMBCHG_USB_CHGPTH_SUBTYPE:
			REQUEST_IRQ(chip, spmi_resource, chip->usbin_uv_irq,
				"usbin-uv", usbin_uv_handler,
				(IRQF_TRIGGER_RISING | IRQF_ONESHOT), rc);
			REQUEST_IRQ(chip, spmi_resource, chip->usbin_ov_irq,
				"usbin-ov", usbin_ov_handler,
				flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->src_detect_irq,
				"usbin-src-det",
				src_detect_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->otg_fail_irq,
				"otg-fail", otg_fail_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->otg_oc_irq,
				"otg-oc", otg_oc_handler,
				(IRQF_TRIGGER_RISING | IRQF_ONESHOT), rc);
			REQUEST_IRQ(chip, spmi_resource, chip->aicl_done_irq,
				"aicl-done",
				aicl_done_handler, flags, rc);
/*++ 2014/10/07 USB Team, PCN00014 ++*/
			/*
			REQUEST_IRQ(chip, spmi_resource,
				chip->usbid_change_irq, "usbid-change",
				usbid_change_handler,
				(IRQF_TRIGGER_FALLING | IRQF_ONESHOT), rc);
			*/
/*-- 2014/10/07 USB Team, PCN00014 --*/
			enable_irq_wake(chip->usbin_uv_irq);
			enable_irq_wake(chip->usbin_ov_irq);
			enable_irq_wake(chip->src_detect_irq);
			enable_irq_wake(chip->otg_fail_irq);
			enable_irq_wake(chip->otg_oc_irq);
/*++ 2014/10/07 USB Team, PCN00014 ++*/
			/*enable_irq_wake(chip->usbid_change_irq);*/
/*-- 2014/10/07 USB Team, PCN00014 --*/
			break;
		case SMBCHG_DC_CHGPTH_SUBTYPE:
			REQUEST_IRQ(chip, spmi_resource, chip->dcin_uv_irq,
				"dcin-uv", dcin_uv_handler, flags, rc);
			enable_irq_wake(chip->dcin_uv_irq);
			break;
		case SMBCHG_MISC_SUBTYPE:
			REQUEST_IRQ(chip, spmi_resource, chip->power_ok_irq,
				"power-ok", power_ok_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->chg_hot_irq,
				"temp-shutdown", chg_hot_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource,
				chip->safety_timeout_irq,
				"safety-timeout",
				safety_timeout_handler, flags, rc);
			enable_irq_wake(chip->chg_hot_irq);
			enable_irq_wake(chip->safety_timeout_irq);
			break;
		case SMBCHG_OTG_SUBTYPE:
			break;
		}
	}

	return rc;
}

#define REQUIRE_BASE(chip, base, rc)					\
do {									\
	if (!rc && !chip->base) {					\
		dev_err(chip->dev, "Missing " #base "\n");		\
		rc = -EINVAL;						\
	}								\
} while (0)

static int smbchg_parse_peripherals(struct smbchg_chip *chip)
{
	int rc = 0;
	struct resource *resource;
	struct spmi_resource *spmi_resource;
	u8 subtype;
	struct spmi_device *spmi = chip->spmi;

	spmi_for_each_container_dev(spmi_resource, chip->spmi) {
		if (!spmi_resource) {
				dev_err(chip->dev, "spmi resource absent\n");
			return rc;
		}

		resource = spmi_get_resource(spmi, spmi_resource,
						IORESOURCE_MEM, 0);
		if (!(resource && resource->start)) {
			dev_err(chip->dev, "node %s IO resource absent!\n",
				spmi->dev.of_node->full_name);
			return rc;
		}

		rc = smbchg_read(chip, &subtype,
				resource->start + SUBTYPE_REG, 1);
		if (rc) {
			dev_err(chip->dev, "Peripheral subtype read failed rc=%d\n",
					rc);
			return rc;
		}

		switch (subtype) {
		case SMBCHG_CHGR_SUBTYPE:
			chip->chgr_base = resource->start;
			break;
		case SMBCHG_BAT_IF_SUBTYPE:
			chip->bat_if_base = resource->start;
			break;
		case SMBCHG_USB_CHGPTH_SUBTYPE:
			chip->usb_chgpth_base = resource->start;
			break;
		case SMBCHG_DC_CHGPTH_SUBTYPE:
			chip->dc_chgpth_base = resource->start;
			break;
		case SMBCHG_MISC_SUBTYPE:
			chip->misc_base = resource->start;
			break;
		case SMBCHG_OTG_SUBTYPE:
			chip->otg_base = resource->start;
			break;
		}
	}

	REQUIRE_BASE(chip, chgr_base, rc);
	REQUIRE_BASE(chip, bat_if_base, rc);
	REQUIRE_BASE(chip, usb_chgpth_base, rc);
	REQUIRE_BASE(chip, dc_chgpth_base, rc);
	REQUIRE_BASE(chip, misc_base, rc);

	return rc;
}

static inline void dump_reg(struct smbchg_chip *chip, u16 addr,
		const char *name)
{
	u8 reg;

	smbchg_read(chip, &reg, addr, 1);
	pr_smb(PR_DUMP, "%s - %04X = %02X\n", name, addr, reg);
}

/* dumps useful registers for debug */
static void dump_regs(struct smbchg_chip *chip)
{
	u16 addr;

	/* charger peripheral */
	for (addr = 0xB; addr <= 0x10; addr++)
		dump_reg(chip, chip->chgr_base + addr, "CHGR Status");
	for (addr = 0xF0; addr <= 0xFF; addr++)
		dump_reg(chip, chip->chgr_base + addr, "CHGR Config");
	/* battery interface peripheral */
	dump_reg(chip, chip->bat_if_base + RT_STS, "BAT_IF Status");
	dump_reg(chip, chip->bat_if_base + CMD_CHG_REG, "BAT_IF Command");
	for (addr = 0xF0; addr <= 0xFB; addr++)
		dump_reg(chip, chip->bat_if_base + addr, "BAT_IF Config");
	/* usb charge path peripheral */
	for (addr = 0x7; addr <= 0x10; addr++)
		dump_reg(chip, chip->usb_chgpth_base + addr, "USB Status");
	dump_reg(chip, chip->usb_chgpth_base + CMD_IL, "USB Command");
	for (addr = 0xF0; addr <= 0xF5; addr++)
		dump_reg(chip, chip->usb_chgpth_base + addr, "USB Config");
	/* dc charge path peripheral */
	dump_reg(chip, chip->dc_chgpth_base + RT_STS, "DC Status");
	for (addr = 0xF0; addr <= 0xF6; addr++)
		dump_reg(chip, chip->dc_chgpth_base + addr, "DC Config");
	/* misc peripheral */
	dump_reg(chip, chip->misc_base + IDEV_STS, "MISC Status");
	dump_reg(chip, chip->misc_base + RT_STS, "MISC Status");
	for (addr = 0xF0; addr <= 0xF3; addr++)
		dump_reg(chip, chip->misc_base + addr, "MISC CFG");
}

static void dump_all(int more)
{
	int ibat_ma, soc, batt_temp, vbus_uv;
	int health, present, charger_type, usb_online, dc_online;
	u8 chgr_sts = 0, chgr_rt_sts = 0, bat_if_rt_sts = 0;
	u8 chgpth_rt_sts = 0, iusb_reg = 0, aicl_reg = 0;
	u8 chgr_cfg2, bat_if_cmd, chgpth_cmd, chgpth_cfg;
	u8 chgpth_icl_sts_2 = 0, chgpth_input_sts = 0, chgpth_aicl_cfg = 0;
	u8 misc_idev_sts = 0, misc_rt_sts = 0;
	int ohm_val = 0, aicl_result;

	vbus_uv = pmi8994_get_usbin_voltage_now(the_chip);
	svbat_mv = smbchg_get_prop_voltage_now(the_chip)/1000;
	ibat_ma = smbchg_get_prop_current_now(the_chip)/1000;
	soc = smbchg_get_prop_capacity_now(the_chip);
	batt_temp = smbchg_get_prop_batt_temp_now(the_chip);
	health = get_prop_batt_health(the_chip);
	present = get_prop_batt_present(the_chip);
	charger_type = get_prop_charge_type(the_chip);
	smbchg_read(the_chip, &chgr_sts, the_chip->chgr_base + CHGR_STS, 1);
	smbchg_read(the_chip, &chgr_rt_sts, the_chip->chgr_base + RT_STS, 1);
	smbchg_read(the_chip, &chgr_cfg2, the_chip->chgr_base + CHGR_CFG2, 1);
	smbchg_read(the_chip, &bat_if_rt_sts, the_chip->bat_if_base + RT_STS, 1);
	smbchg_read(the_chip, &bat_if_cmd, the_chip->bat_if_base + CMD_CHG, 1);
	smbchg_read(the_chip, &chgpth_rt_sts,
						the_chip->usb_chgpth_base + RT_STS, 1);
	smbchg_read(the_chip, &chgpth_cmd,
						the_chip->usb_chgpth_base + CMD_IL, 1);
	smbchg_read(the_chip, &chgpth_aicl_cfg,
						the_chip->usb_chgpth_base + USB_AICL_CFG, 1);
	smbchg_read(the_chip, &chgpth_cfg,
						the_chip->usb_chgpth_base + CHGPTH_CFG, 1);
	smbchg_read(the_chip, &chgpth_icl_sts_2,
						the_chip->usb_chgpth_base + ICL_STS_2_REG, 1);
	smbchg_read(the_chip, &chgpth_input_sts,
						the_chip->usb_chgpth_base + INPUT_STS, 1);
	smbchg_read(the_chip, &misc_idev_sts, the_chip->misc_base + IDEV_STS, 1);
	smbchg_read(the_chip, &misc_rt_sts, the_chip->misc_base + RT_STS, 1);
	usb_online = is_usb_present(the_chip);
	dc_online = is_dc_present(the_chip);

	smbchg_read(the_chip, &iusb_reg, the_chip->usb_chgpth_base + IL_CFG, 1);
	smbchg_read(the_chip, &aicl_reg,
								the_chip->usb_chgpth_base + ICL_STS_1_REG, 1);
	pmi8994_fg_get_batt_id_ohm(&ohm_val);
	aicl_result = smbchg_get_aicl_level_ma(the_chip);
	the_chip->usb_psy->get_property(the_chip->usb_psy, POWER_SUPPLY_PROP_TYPE, &g_usbtype);

	printk(KERN_ERR "[BATT][CHG] V=%d mV,I=%d mA,T=%d C,id=%d ohm,SoC=%d,vbus=%d,"
			"H=%d,P=%d,CHG=%d,0x100E=%02x,0x1010=%02x,0x10FC=%02x,"
			"0x1210=%02x,0x1242=%02x,0x1309=%02x,0x130D=%02x,0x1310=%02x,0x1340=%02x,0x13F3=%02x,"
			"0x13F4=%02x,0x1608=%02x,0x1610=%02x,USB=%d,DC=%d,IUSB_MAX=%02x,AICL=%d,is_full=%d,"
			"is_limit_IUSB=%d,flag=%d%d%d%d%d,"
			"usb_type=%d\n",
			svbat_mv, ibat_ma, batt_temp, ohm_val, soc, vbus_uv, health, present, charger_type,
			chgr_sts, chgr_rt_sts, chgr_cfg2, bat_if_rt_sts, bat_if_cmd,
			chgpth_icl_sts_2, chgpth_input_sts, chgpth_rt_sts, chgpth_cmd, chgpth_aicl_cfg,
			chgpth_cfg, misc_idev_sts, misc_rt_sts, usb_online,
			dc_online, iusb_reg, aicl_result, is_batt_full, is_limit_IUSB, flag_force_ac_chg,
			flag_pa_fake_batt_temp, flag_keep_charge_on, flag_disable_safety_timer,
			flag_disable_temp_protection,
			g_usbtype.intval);

	if (smbchg_debug_mask & PR_DUMP)
		dump_regs(the_chip);
}

inline int pmi8994_dump_all(void)
{
	if(!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	dump_all(0);

	return 0;
}

int pmi8994_is_batt_full(int *result)
{
    int soc = 0;
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	/* Work around to fix EOC work queue stops and not rescheduled
	*  due to bad cable.
	*/
	if(is_batt_full){
		soc = smbchg_get_prop_capacity_now(the_chip);
		/* Check if soc less than thresold and EOC work queue not pending
		 * if yes then clear the is full flag
		 */
		if( soc <= CLEAR_FULL_STATE_BY_LEVEL_THR &&
		(!delayed_work_pending(&the_chip->eoc_work))){
			is_batt_full = false;
			pr_info("EOC not scheduled,clear full flag (is_batt_full = %d)\n", is_batt_full);
		}
	}

	*result = is_batt_full;
	return 0;
}

int pmi8994_is_batt_full_eoc_stop(int *result)
{
	int rc;
	u8 chgr_sts = 0, int_rt_sts = 0, chg_type = 0;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	*result = is_batt_full_eoc_stop;

	/* Work around to fix EOC work queue stops and not rescheduled
	*  Check related chg registers, and rescheduled again.
	*/
	if (!delayed_work_pending(&the_chip->eoc_work)) {
		rc = smbchg_read(the_chip, &chgr_sts, the_chip->chgr_base + CHGR_STS, 1);
		if (rc) {
			dev_err(the_chip->dev, "failed to read chgr_sts (0x100E) rc=%d\n", rc);
			return -ENXIO;
		}
		rc = smbchg_read(the_chip, &int_rt_sts, the_chip->chgr_base + RT_STS, 1);
		if (rc) {
			dev_err(the_chip->dev, "failed to read int_rt_sts (0x1010) rc=%d\n", rc);
			return -ENXIO;
		}
		chg_type = (chgr_sts & CHG_TYPE_MASK) >> CHG_TYPE_SHIFT;
		if ((int_rt_sts & P2F_CHG_THRESHOLD_BIT) && //in charging
			((int_rt_sts & BAT_TCC_REACHED_BIT) == 0) && //no eoc
			(chg_type != 0)) { //charging status is not no-charging.
			wake_lock(&the_chip->eoc_worker_wlock);
			schedule_delayed_work(&the_chip->eoc_work,
				msecs_to_jiffies(EOC_CHECK_PERIOD_MS));
			pr_info("eoc_work rescheduled from batt_worker\n");
		}
	}
	return 0;
}

int pmi8994_charger_get_attr_text(char *buf, int size)
{
	int len = 0;
	u16 addr;
	u8 reg;
	int vbat_mv, ibat_ma, soc, batt_temp, vbus_uv;
	int health, present, charger_type, usb_online, dc_online;
	u8 chgr_sts = 0, chgr_rt_sts = 0, bat_if_rt_sts = 0;
	u8 chgpth_rt_sts = 0, iusb_reg = 0, aicl_reg = 0;

        if(!the_chip) {
                pr_err("called before init\n");
                return -EINVAL;
        }

	vbat_mv = smbchg_get_prop_voltage_now(the_chip)/1000;
	ibat_ma = smbchg_get_prop_current_now(the_chip)/1000;
	soc = smbchg_get_prop_capacity_now(the_chip);
	vbus_uv = pmi8994_get_usbin_voltage_now(the_chip);
	batt_temp = smbchg_get_prop_batt_temp_now(the_chip);
	health = get_prop_batt_health(the_chip);
	present = get_prop_batt_present(the_chip);
	charger_type = get_prop_charge_type(the_chip);
	usb_online = is_usb_present(the_chip);
	dc_online = is_dc_present(the_chip);

	smbchg_read(the_chip, &chgr_sts, the_chip->chgr_base + CHGR_STS, 1);
	smbchg_read(the_chip, &chgr_rt_sts, the_chip->chgr_base + RT_STS, 1);
	smbchg_read(the_chip, &bat_if_rt_sts, the_chip->bat_if_base + RT_STS, 1);
	smbchg_read(the_chip, &chgpth_rt_sts,the_chip->usb_chgpth_base + RT_STS, 1);

	smbchg_read(the_chip, &iusb_reg, the_chip->usb_chgpth_base + IL_CFG, 1);
	smbchg_read(the_chip, &aicl_reg, the_chip->usb_chgpth_base + ICL_STS_1_REG, 1);

	the_chip->usb_psy->get_property(the_chip->usb_psy, POWER_SUPPLY_PROP_TYPE, &g_usbtype);

	len += scnprintf(buf + len, size -len,
			"USB_TYPE: %d;\n"
			"SOC(%%): %d;\n"
			"VBAT(mV): %d;\n"
			"IBAT(mA): %d;\n"
			"VBUS(uV): %d;\n"
			"BATT_TEMP: %d;\n"
			"HEALTH: %d;\n"
			"BATT_PRESENT(bool): %d;\n"
			"CHARGE_TYPE: %d;\n"
			"USB_ONLINE: %d;\n"
			"DC_ONLINE: %d;\n"
			"CHGR_STS: 0x%02x;\n"
			"CHGR_RT_STS: 0x%02x;\n"
			"IF_RT_STS: 0x%02x;\n"
			"USB_RT_STS: 0x%02x;\n"
			"USB_IL_CFG: 0x%02x;\n"
			"USB_ICL_STS_1_REG: 0x%02x;\n",
			g_usbtype.intval, soc, vbat_mv, ibat_ma, vbus_uv, batt_temp, health, present,
			charger_type, usb_online, dc_online,
			chgr_sts, chgr_rt_sts, bat_if_rt_sts, chgpth_rt_sts,
			iusb_reg, aicl_reg);

	/* charger peripheral */
	for (addr = 0xB; addr <= 0x10; addr++){
		smbchg_read(the_chip, &reg, the_chip->chgr_base + addr, 1); //"CHGR Status"
		len += scnprintf(buf + len, size -len, "CHGR_STS_%02x: 0x%02x\n", addr, reg);
	}
	for (addr = 0xF0; addr <= 0xFF; addr++){
		smbchg_read(the_chip, &reg, the_chip->chgr_base + addr, 1); //"CHGR Config"
		len += scnprintf(buf + len, size -len, "CHGR_CF_%02x: 0x%02x\n", addr, reg);
	}
	/* battery interface peripheral */
	smbchg_read(the_chip, &reg, the_chip->bat_if_base + RT_STS, 1);
	len += scnprintf(buf + len, size -len, "BAT_IF_STS: 0x%02x\n", reg);
	smbchg_read(the_chip, &reg, the_chip->bat_if_base + CMD_CHG_REG, 1);
	len += scnprintf(buf + len, size -len, "BAT_IF_Command: 0x%02x\n", reg);

	for (addr = 0xF0; addr <= 0xFB; addr++){
		smbchg_read(the_chip, &reg, the_chip->chgr_base + addr, 1); //"BAT_IF Config"
		len += scnprintf(buf + len, size -len, "BAT_IF_CF_%02x: 0x%02x\n", addr, reg);
	}
	/* usb charge path peripheral */
	for (addr = 0x7; addr <= 0x10; addr++){
		smbchg_read(the_chip, &reg, the_chip->usb_chgpth_base + addr, 1); //"USB Status"
		len += scnprintf(buf + len, size -len, "USB_STS_%02x: 0x%02x\n", addr, reg);
	}
	smbchg_read(the_chip, &reg, the_chip->usb_chgpth_base + CMD_IL, 1);
	len += scnprintf(buf + len, size -len, "USB_Command: 0x%02x\n", reg);
	for (addr = 0xF0; addr <= 0xF5; addr++){
		smbchg_read(the_chip, &reg, the_chip->usb_chgpth_base + addr, 1); //"USB Config"
		len += scnprintf(buf + len, size -len, "USB_CF_%02x: 0x%02x\n", addr, reg);
	}
	/* dc charge path peripheral */
	smbchg_read(the_chip, &reg, the_chip->dc_chgpth_base + RT_STS, 1);
	len += scnprintf(buf + len, size -len, "DC_Status: 0x%02x\n", reg);
	for (addr = 0xF0; addr <= 0xF6; addr++){
		smbchg_read(the_chip, &reg, the_chip->dc_chgpth_base + addr, 1); //"DC Config"
		len += scnprintf(buf + len, size -len, "DC_CF_%02x: 0x%02x\n", addr, reg);
	}
	/* misc peripheral */
	smbchg_read(the_chip, &reg, the_chip->misc_base + IDEV_STS, 1);
	len += scnprintf(buf + len, size -len, "MISC_STS: 0x%02x\n", reg);
	smbchg_read(the_chip, &reg, the_chip->misc_base + RT_STS, 1);
	len += scnprintf(buf + len, size -len, "MISC_RT_STS: 0x%02x\n", reg);
	for (addr = 0xF0; addr <= 0xF3; addr++){
		smbchg_read(the_chip, &reg, the_chip->misc_base + addr, 1); //"MISC Config"
		len += scnprintf(buf + len, size -len, "MISC_CF_%02x: 0x%02x\n", addr, reg);
	}
	return len;
}

int pmi8994_is_quick_charger_used(bool *result)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	*result = is_HVDCP_9V_done(the_chip);

	return 0;
}

int pmi8994_is_bad_cable_used(int *result)
{
	int soc = 0,batt_temp = 0,aicl_result = 0;
	bool is_temp_fault = false;
	*result = 0;
	if(!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	is_temp_fault = (the_chip->batt_hot || the_chip->batt_cold);

	if(the_chip->power_source_type != 2 || is_temp_fault){
		*result = 0;
		pr_info("chg_src:%d, is_bad_aicl_result:%d, temp_fault:%d,charging_disabled:%d,result:%d\n",
			the_chip->power_source_type,gs_is_bad_aicl_result,is_temp_fault,batt_charging_disabled,*result);
		return 0;
	}

	soc = smbchg_get_prop_capacity_now(the_chip);
	batt_temp = smbchg_get_prop_batt_temp_now(the_chip);
	aicl_result = smbchg_get_aicl_level_ma(the_chip);

	if(	soc != 100 &&
		the_chip->power_source_type == 2 &&
		gs_is_bad_aicl_result &&
		!batt_charging_disabled ){

	   *result = 1;
	}
	pr_info("chg_src:%d, SOC:%d, is_bad_aicl_result:%d, charging_disabled:%d, Temp:%d AICL:%d, result:%d\n",
			the_chip->power_source_type,soc,gs_is_bad_aicl_result,batt_charging_disabled,batt_temp,aicl_result,*result);

	return 0;
}

int pmi8994_get_batt_voltage(int *result)
{
	if(!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	*result = smbchg_get_prop_voltage_now(the_chip)/1000;

	return 0;
}

int pmi8994_get_charge_type(void)
{
	if(!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	return get_prop_charge_type(the_chip);
}

int pmi8994_fake_src_detect_irq_handler(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	/* issue a cable detection to force update */
	cable_detection_vbus_irq_handler();

	return 0;
}

int pmi8994_check_cable_status(void)
{
	int usb_present;
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	/* check cable status */
	usb_present = (int)is_usb_present(the_chip);
	return usb_present;
}

int pmi8994_is_charger_ovp(int* result)
{
	int ov = 0, uv = 0;
	int rc;
	u8 reg;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	rc = smbchg_read(the_chip, &reg, the_chip->usb_chgpth_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(the_chip->dev, "Couldn't read usb rt status rc = %d\n", rc);
		return 0;
	}

	if (reg & USBIN_UV_BIT) {
		uv = 1;
	} else if (reg & USBIN_OV_BIT) {
		ov = 1;
	}

	*result = ov;
	if (uv || ov)
		pr_smb(PR_STATUS, "ov=%d, uv=%d, reg=0X%2x\n", ov, uv, reg);
	return 0;
}

/* To limit USB curent when navigating. */
#define NAVI_LIMITED_CURRENT_UP                 700
#define NAVI_LIMITED_CURRENT_DOWN               1000
#define NAVI_LIMITED_CURRENT_UP_QC20            500
#define NAVI_LIMITED_CURRENT_DOWN_QC20          700

#define NAVI_UPPER_BOUND                        30
#define NAVI_LOWER_BOUND                        30
#define NAVI_LIMIT_TEMP                         390

#ifdef CONFIG_DUTY_CYCLE_LIMIT
int pmi8994_limit_charge_enable(int chg_limit_reason, int chg_limit_timer_sub_mask, int limit_charge_timer_ma)
{
	pr_info("chg_limit_reason=%d, chg_limit_timer_sub_mask=%d, limit_charge_timer_ma=%d\n",
			chg_limit_reason, chg_limit_timer_sub_mask, limit_charge_timer_ma);

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	/* NOTE: limit_charge_timer_ma works if it is not 0 and timer_enable is not 0 */
	if (limit_charge_timer_ma != 0 && !!(chg_limit_reason & chg_limit_timer_sub_mask))
		chg_limit_current = limit_charge_timer_ma;
	else {
		if (!!chg_limit_reason)
			chg_limit_current = PMI8994_CHG_I_MIN_MA;
		else
			chg_limit_current = 0;
	}

	pr_info("%s:chg_limit_current = %d\n", __func__, chg_limit_current);
	smbchg_set_appropriate_battery_current(the_chip);
	return 0;
}
#else
int pmi8994_limit_charge_enable(bool enable, int reason, int restrict)
{
	int batt_temp = 0;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	batt_temp = smbchg_get_prop_batt_temp_now(the_chip);
	pr_info("limit_charge=%d, reason=0x%x, restrict=%d, batt_temp=%d\n", enable, reason, restrict, batt_temp);


	/* talk */
	if (enable && (reason & HTC_BATT_CHG_LIMIT_BIT_TALK))
		chg_limit_current = PMI8994_CHG_I_MIN_MA;
	/* net talk */
	else if (enable && (reason & HTC_BATT_CHG_LIMIT_BIT_NET_TALK))
		chg_limit_current = PMI8994_CHG_I_MIN_MA;
	/* thermal*/
	else if (enable && (reason & HTC_BATT_CHG_LIMIT_BIT_THRML) && (restrict == RESTRICT_NONE))
		chg_limit_current = 0;
	else if (enable && (reason & HTC_BATT_CHG_LIMIT_BIT_THRML) && (restrict == RESTRICT_SOFT))
		chg_limit_current = PMI8994_CHG_I_MIN_MA_L1;
	else if (enable && (reason & HTC_BATT_CHG_LIMIT_BIT_THRML) && (restrict == RESTRICT_HEAVY))
		chg_limit_current = PMI8994_CHG_I_MIN_MA_L2;
	/* normal */
	else
		chg_limit_current = 0;

	smbchg_set_appropriate_battery_current(the_chip);
	return 0;
}
#endif

int pmi8994_get_cable_type_by_usb_detect(int *result)
{
	int rc;

	rc = usb_get_connect_type();
	pr_info("USB type: %d\n", rc);
	*result = rc;
	return 0;
}

int pmi8994_limit_input_current(bool enable, int reason)
{
	int batt_temp = 0;
	int limit1_temp = 0, limit1_current = 0;
	int limit2_temp = 0, limit2_current = 0;
	int limit_thermal_current = 0;
	int limit_navi_current = 0;
	int limit_current = 0;
	int orig_current_ma = 0;
	int soc = 0;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	soc = smbchg_get_prop_capacity_now(the_chip);
	batt_temp = smbchg_get_prop_batt_temp_now(the_chip);
	limit1_temp = the_chip->kddi_limit1_temp;
	limit2_temp = the_chip->kddi_limit2_temp;

	/* using QC2.0 adepter has different setting.*/
	if (is_qc20_done_flag == true) {
		limit1_current = the_chip->kddi_limit1_current_qc20;
		limit2_current = the_chip->kddi_limit2_current_qc20;
	} else {
		limit1_current = the_chip->kddi_limit1_current;
		limit2_current = the_chip->kddi_limit2_current;
	}

	orig_current_ma = the_chip->usb_target_current_ma;

	if (enable) {
		/* check NAVI first */
		if ((reason & HTC_BATT_CHG_LIMIT_BIT_NAVI) &&
				(soc > NAVI_UPPER_BOUND) &&
				(batt_temp > NAVI_LIMIT_TEMP)) {
			// Navi zone1
			if (is_qc20_done_flag == true)
				limit_navi_current = NAVI_LIMITED_CURRENT_UP_QC20;
			else
				limit_navi_current = NAVI_LIMITED_CURRENT_UP;
		} else if ((reason & HTC_BATT_CHG_LIMIT_BIT_NAVI) &&
				(soc <= NAVI_LOWER_BOUND) &&
				(batt_temp > NAVI_LIMIT_TEMP)) {
			// Navi zone2
			if (is_qc20_done_flag == true)
				limit_navi_current = NAVI_LIMITED_CURRENT_DOWN_QC20;
			else
				limit_navi_current = NAVI_LIMITED_CURRENT_DOWN;
		} else {
			// do nothing
		}

		/* check KDDI thermal */
		if ((reason & HTC_BATT_CHG_LIMIT_BIT_KDDI) &&
			(limit1_temp > 0) && (limit2_temp > 0) &&
			(limit1_temp < limit2_temp) &&
			(limit1_current > 0) && (limit2_current > 0))
		{
			if ((batt_temp > limit1_temp) && (batt_temp <= limit2_temp)) {
				// zone1 (ex. 39 < T <= 45, set IUSB_MAX as 1500mA)
				limit_thermal_current = limit1_current;
			} else if (batt_temp > limit2_temp) {
				// zone2 (ex. T > 45, set IUSB_MAX as 300mA)
				limit_thermal_current = limit2_current;
			} else if (batt_temp <= limit1_temp) {
				// zone3 (ex. T <= 39, set IUSB_MAX as original setting)
				limit_thermal_current = orig_current_ma;
			}
			else {
				// do nothing
			}
		}

		pr_info("soc:%d, batt_temp:%d, L1_temp:%d, L2_temp:%d, "
				"limit_navi_current:%d, limit_thermal_current:%d, "
				"orig_current_ma:%d\n", soc, batt_temp, limit1_temp,
				limit2_temp, limit_navi_current, limit_thermal_current,
				orig_current_ma);
		/* if 6 4 or 6 10000000 set, not to limit usb current */
		if (!flag_keep_charge_on && !flag_disable_temp_protection) {
			mutex_lock(&the_chip->current_change_lock);
			/* let limit_current not to be higher than original setting */
			if (limit_navi_current != 0 && limit_thermal_current != 0) {
				limit_current = MIN(MIN(limit_navi_current, limit_thermal_current), orig_current_ma);
				smbchg_set_thermal_limited_usb_current_max(
						the_chip, limit_current);
			} else if (limit_navi_current != 0) {
				limit_current = MIN(limit_navi_current, orig_current_ma);
				smbchg_set_thermal_limited_usb_current_max(
						the_chip, limit_current);
			} else if (limit_thermal_current != 0) {
				limit_current = MIN(limit_thermal_current, orig_current_ma);
				smbchg_set_thermal_limited_usb_current_max(
						the_chip, limit_current);
			} else {
				// do nothing
			}
			mutex_unlock(&the_chip->current_change_lock);
		}
	}
	else { // disable, force to set back original current
		mutex_lock(&the_chip->current_change_lock);
		smbchg_set_thermal_limited_usb_current_max(
				the_chip, orig_current_ma);
		mutex_unlock(&the_chip->current_change_lock);
	}

	return 0;
}

int pmi8994_get_usb_type(int *result)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	the_chip->usb_psy->get_property(the_chip->usb_psy, POWER_SUPPLY_PROP_TYPE, &g_usbtype);

	*result = g_usbtype.intval;
	return 0;
}

int pmi8994_get_vbus(int *result)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	*result = pmi8994_get_usbin_voltage_now(the_chip);
	return 0;
}

int pmi8994_get_max_iusb(int *result)
{
	u8 iusb_reg = 0;
	int rc = 0;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	rc = smbchg_read(the_chip, &iusb_reg, the_chip->usb_chgpth_base + IL_CFG, 1);

	if (rc) {
		dev_err(the_chip->dev, "cannot write to config c rc = %d\n", rc);
		return -EINVAL;
	}

	iusb_reg &= ICL_STS_MASK;
	if (iusb_reg >= ARRAY_SIZE(usb_current_table)) {
		pr_warn("invalid AICL value: %02x\n", iusb_reg);
		return -EINVAL;
	}

	*result = usb_current_table[iusb_reg];
	return 0;
}

int pmi8994_get_AICL(int *result)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	*result = smbchg_get_aicl_level_ma(the_chip);
	return 0;
}

static int smbchg_probe(struct spmi_device *spmi)
{
	int rc;
	struct smbchg_chip *chip;
	struct power_supply *usb_psy;
	enum device_model model;

	smbchg_debug_mask = 0x6;

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		pr_smb(PR_STATUS, "USB supply not found, deferring probe\n");
		return -EPROBE_DEFER;
	}

	chip = devm_kzalloc(&spmi->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&spmi->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	INIT_DELAYED_WORK(&chip->usb_set_online_work, smbchg_usb_update_online_work);
	chip->spmi = spmi;
	chip->dev = &spmi->dev;
	chip->usb_psy = usb_psy;
	chip->fake_battery_soc = -EINVAL;
	chip->usb_online = 0;
	dev_set_drvdata(&spmi->dev, chip);

	spin_lock_init(&chip->sec_access_lock);
	mutex_init(&chip->pm_lock);
	mutex_init(&chip->usb_status_lock);
	mutex_init(&chip->current_change_lock);
	mutex_init(&chip->usb_set_online_lock);
	mutex_init(&chip->usb_en_lock);
	mutex_init(&chip->dc_en_lock);

	if (!chip->bms_psy && chip->bms_psy_name)
		chip->bms_psy =
			power_supply_get_by_name((char *)chip->bms_psy_name);
	if (chip->bms_psy)
		smbchg_load_jeita_setting(chip);

	/* htc add */
	wake_lock_init(&chip->vbus_wlock,
			WAKE_LOCK_SUSPEND, "vbus_wlock");
	spin_lock_init(&chip->vbus_lock);
	wake_lock_init(&chip->eoc_worker_wlock, WAKE_LOCK_SUSPEND,
							"pmi8994_eoc_worker_lock");

	INIT_WORK(&chip->notify_fg_work, smbchg_notify_fg_work);
	INIT_WORK(&chip->batt_soc_work, smbchg_adjust_batt_soc_work);
	INIT_DELAYED_WORK(&chip->vbus_detect_work, check_cable_type);
	INIT_DELAYED_WORK(&chip->eoc_work, smbchg_eoc_work);
	INIT_DELAYED_WORK(&chip->usb_type_polling_work, usb_type_polling);
	INIT_DELAYED_WORK(&chip->retry_aicl_work, retry_aicl_worker);
	INIT_DELAYED_WORK(&chip->hvdcp_5to9V_work, hvdcp_5to9V_worker);
	INIT_DELAYED_WORK(&chip->vfloat_adjust_work, smbchg_vfloat_adjust_work);
	INIT_DELAYED_WORK(&chip->hvdcp_det_work, smbchg_hvdcp_det_work);
	chip->cable_detect_wq = create_singlethread_workqueue("cable_detect");
	INIT_WORK(&chip->usb_aicl_limit_current, smbchg_usb_limit_current_WA_work);
	INIT_DELAYED_WORK(&chip->usb_limit_max_current, smbchg_usb_limit_max_current_work);

	rc = smbchg_parse_peripherals(chip);
	if (rc) {
		dev_err(chip->dev, "Error parsing DT peripherals: %d\n", rc);
		return rc;
	}
	rc = smb_parse_dt(chip);
	if (rc < 0) {
		dev_err(&spmi->dev, "Unable to parse DT nodes: %d\n", rc);
		return rc;
	}

	rc = smbchg_regulator_init(chip);
	if (rc) {
		dev_err(&spmi->dev,
			"Couldn't initialize regulator rc=%d\n", rc);
		return rc;
	}

	rc = smbchg_hw_init(chip);
	if (rc < 0) {
		dev_err(&spmi->dev,
			"Unable to intialize hardware rc = %d\n", rc);
		goto free_regulator;
	}

	rc = determine_initial_status(chip);
	if (rc < 0) {
		dev_err(&spmi->dev,
			"Unable to determine init status rc = %d\n", rc);
		goto free_regulator;
	}

	/* Check device is XA, XB or XC above */
	model = check_device_model();
	if ((model == DEVICE_MODEL_XA) || (model == DEVICE_MODEL_XB))
		is_device_XA_or_XB = 1;
	else
		is_device_XA_or_XB = 0;

	if (model == DEVICE_MODEL_INVALID)
		dev_err(&spmi->dev,
			"Unable to get device model rc = %d\n", rc);

	/* 8994 WA, For QC2.0 power off not-charging issue
	 * After booting up for 10s, set HVDCP back to 9V.
	 */
	schedule_delayed_work(&chip->hvdcp_5to9V_work,
		msecs_to_jiffies(HVDCP_CHANGE_PERIOD_MS));

	chip->previous_soc = -EINVAL;
#if 0//FIXME
	chip->batt_psy.name		= chip->battery_psy_name;
	chip->batt_psy.type		= POWER_SUPPLY_TYPE_BATTERY;
	chip->batt_psy.get_property	= smbchg_battery_get_property;
	chip->batt_psy.set_property	= smbchg_battery_set_property;
	chip->batt_psy.properties	= smbchg_battery_properties;
	chip->batt_psy.num_properties	= ARRAY_SIZE(smbchg_battery_properties);
	chip->batt_psy.external_power_changed = smbchg_external_power_changed;
	chip->batt_psy.property_is_writeable = smbchg_battery_is_writeable;

	rc = power_supply_register(chip->dev, &chip->batt_psy);
	if (rc < 0) {
		dev_err(&spmi->dev,
			"Unable to register batt_psy rc = %d\n", rc);
		goto free_regulator;
	}

	if (chip->dc_psy_type != -EINVAL) {
		chip->dc_psy.name		= "dc";
		chip->dc_psy.type		= chip->dc_psy_type;
		chip->dc_psy.get_property	= smbchg_dc_get_property;
		chip->dc_psy.set_property	= smbchg_dc_set_property;
		chip->dc_psy.property_is_writeable = smbchg_dc_is_writeable;
		chip->dc_psy.properties		= smbchg_dc_properties;
		chip->dc_psy.num_properties = ARRAY_SIZE(smbchg_dc_properties);
		rc = power_supply_register(chip->dev, &chip->dc_psy);
		if (rc < 0) {
			dev_err(&spmi->dev,
				"Unable to register dc_psy rc = %d\n", rc);
			goto unregister_batt_psy;
		}
	}
	chip->psy_registered = true;
#else
	chip->psy_registered = false;
#endif

	the_chip = chip;

	rc = smbchg_request_irqs(chip);
	if (rc < 0) {
		dev_err(&spmi->dev, "Unable to request irqs rc = %d\n", rc);
		goto unregister_dc_psy;
	}

	power_supply_set_present(chip->usb_psy, chip->usb_present);
	htc_charger_event_notify(HTC_CHARGER_EVENT_READY);

	dump_regs(chip);
	dev_info(chip->dev, "SMBCHG successfully probed batt=%d dc = %d usb = %d\n",
			get_prop_batt_present(chip),
			chip->dc_present, chip->usb_present);
	return 0;

unregister_dc_psy:
	power_supply_unregister(&chip->dc_psy);
#if 0//FIXME
unregister_batt_psy:
	power_supply_unregister(&chip->batt_psy);
#endif
free_regulator:
	smbchg_regulator_deinit(chip);
	handle_usb_removal(chip);
	return rc;
}

static int smbchg_remove(struct spmi_device *spmi)
{
	struct smbchg_chip *chip = dev_get_drvdata(&spmi->dev);

	if (chip->dc_psy_type != -EINVAL)
		power_supply_unregister(&chip->dc_psy);

	power_supply_unregister(&chip->batt_psy);
	smbchg_regulator_deinit(chip);

	/* htc add */
	cancel_delayed_work_sync(&chip->eoc_work);
	return 0;
}

static const struct dev_pm_ops smbchg_pm_ops = {
};

MODULE_DEVICE_TABLE(spmi, smbchg_id);

static struct spmi_driver smbchg_driver = {
	.driver		= {
		.name		= "qpnp-smbcharger",
		.owner		= THIS_MODULE,
		.of_match_table	= smbchg_match_table,
		.pm		= &smbchg_pm_ops,
	},
	.probe		= smbchg_probe,
	.remove		= smbchg_remove,
};

static int __init smbchg_init(void)
{
	flag_keep_charge_on =
		(get_kernel_flag() & KERNEL_FLAG_KEEP_CHARG_ON) ? 1 : 0;
	flag_force_ac_chg =
		(get_kernel_flag() & KERNEL_FLAG_ENABLE_FAST_CHARGE) ? 1 : 0;
	flag_pa_fake_batt_temp =
		(get_kernel_flag() & KERNEL_FLAG_FOR_PA_TEST) ? 1 : 0;
	flag_disable_safety_timer =
		(get_kernel_flag() & KERNEL_FLAG_DISABLE_SAFETY_TIMER) ? 1 : 0;
	flag_disable_temp_protection =
		(get_kernel_flag() & KERNEL_FLAG_DISABLE_TBATT_PROTECT) ? 1 : 0;
	flag_ats_limit_chg =
		(get_kernel_flag() & KERNEL_FLAG_ATS_LIMIT_CHARGE) ? 1 : 0;

	return spmi_driver_register(&smbchg_driver);
}

static void __exit smbchg_exit(void)
{
	return spmi_driver_unregister(&smbchg_driver);
}

module_init(smbchg_init);
module_exit(smbchg_exit);

MODULE_DESCRIPTION("QPNP SMB Charger");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:qpnp-smbcharger");
