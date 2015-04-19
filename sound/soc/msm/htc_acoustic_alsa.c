/* arch/arm/mach-msm/htc_acoustic_alsa.c
 *
 * Copyright (C) 2012 HTC Corporation
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

#include <linux/device.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/mm.h>
#include <linux/gfp.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <sound/htc_acoustic_alsa.h>
#include <linux/htc_headset_mgr.h>

struct device_info {
	unsigned pcb_id;
	unsigned sku_id;
};

#define D(fmt, args...) printk(KERN_INFO "[AUD] htc-acoustic: "fmt, ##args)
#define E(fmt, args...) printk(KERN_ERR "[AUD] htc-acoustic: "fmt, ##args)

static DEFINE_MUTEX(api_lock);
static struct acoustic_ops default_acoustic_ops;
static struct acoustic_ops *the_ops = &default_acoustic_ops;
static struct switch_dev sdev_beats;
static struct switch_dev sdev_dq;
static struct switch_dev sdev_listen_notification;

static DEFINE_MUTEX(hs_amp_lock);
static struct amp_register_s hs_amp = {NULL, NULL};

static DEFINE_MUTEX(spk_amp_lock);
static struct amp_register_s spk_amp[SPK_AMP_MAX] = {{NULL}};

static struct amp_power_ops default_amp_power_ops = {NULL};
static struct amp_power_ops *the_amp_power_ops = &default_amp_power_ops;

static struct wake_lock htc_acoustic_wakelock;
static struct wake_lock htc_acoustic_wakelock_timeout;
static struct wake_lock htc_acoustic_dummy_wakelock;
static struct hs_notify_t hs_plug_nt[HS_N_MAX] = {{0,NULL,NULL}};
static DEFINE_MUTEX(hs_nt_lock);

static int hs_amp_open(struct inode *inode, struct file *file);
static int hs_amp_release(struct inode *inode, struct file *file);
static long hs_amp_ioctl(struct file *file, unsigned int cmd, unsigned long arg);

static enum AUD_FTM_BTPCM_MODE aud_ftm_btpcm_mode = AUD_FTM_BTPCM_MODE_PCM;
static int aud_ftm_btpcm_sim_state = 0;
static DEFINE_MUTEX(aud_ftm_lock);

static int aud_ftm_open(struct inode *inode, struct file *file);
static int aud_ftm_release(struct inode *inode, struct file *file);
static ssize_t aud_ftm_read(struct file *file, char __user *string, size_t size, loff_t *offset);
static ssize_t aud_ftm_write(struct file *file, const char __user *string, size_t size, loff_t *offset);
static long aud_ftm_ioctl(struct file *file, unsigned int cmd, unsigned long arg);

void aud_ftm_func_register(struct aud_ftm_btpcm_func_t *funcs);


static struct aud_ftm_btpcm_func_t aud_ftm_btpcm_func = {
	.init = 0,
	.sim = 0,
	.gpio_config = NULL,
	.gpio_read = NULL,
};

static struct file_operations hs_def_fops = {
	.owner = THIS_MODULE,
	.open = hs_amp_open,
	.release = hs_amp_release,
	.unlocked_ioctl = hs_amp_ioctl,
};

static struct miscdevice hs_amp_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "aud_hs_amp",
	.fops = &hs_def_fops,
};

static struct file_operations aud_ftm_fops = {
	.owner = THIS_MODULE,
	.open = aud_ftm_open,
	.release = aud_ftm_release,
	.read = aud_ftm_read,
	.write = aud_ftm_write,
	.unlocked_ioctl = aud_ftm_ioctl,
};

static struct miscdevice aud_ftm_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "aud_ftm",
	.fops = &aud_ftm_fops,
};

struct hw_component HTC_AUD_HW_LIST[AUD_HW_NUM] = {
	{
		.name = "TPA2051",
		.id = HTC_AUDIO_TPA2051,
	},
	{
		.name = "TPA2026",
		.id = HTC_AUDIO_TPA2026,
	},
	{
		.name = "TPA2028",
		.id = HTC_AUDIO_TPA2028,
	},
	{
		.name = "A1028",
		.id = HTC_AUDIO_A1028,
	},
	{
		.name = "TPA6185",
		.id = HTC_AUDIO_TPA6185,
	},
	{
		.name = "RT5501",
		.id = HTC_AUDIO_RT5501,
	},
	{
		.name = "TFA9887",
		.id = HTC_AUDIO_TFA9887,
	},
	{
		.name = "TFA9887L",
		.id = HTC_AUDIO_TFA9887L,
	},
};
EXPORT_SYMBOL(HTC_AUD_HW_LIST);

#if 0
extern unsigned int system_rev;
#endif
void htc_acoustic_register_spk_amp(enum SPK_AMP_TYPE type,int (*aud_spk_amp_f)(int, int), struct file_operations* ops)
{
	mutex_lock(&spk_amp_lock);

	if(type < SPK_AMP_MAX) {
		spk_amp[type].aud_amp_f = aud_spk_amp_f;
		spk_amp[type].fops = ops;
	}

	mutex_unlock(&spk_amp_lock);
}

int htc_acoustic_spk_amp_ctrl(enum SPK_AMP_TYPE type,int on, int dsp)
{
	int ret = 0;
	mutex_lock(&spk_amp_lock);

	if(type < SPK_AMP_MAX) {
		if(spk_amp[type].aud_amp_f)
			ret = spk_amp[type].aud_amp_f(on,dsp);
	}

	mutex_unlock(&spk_amp_lock);

	return ret;
}

int htc_acoustic_hs_amp_ctrl(int on, int dsp)
{
	int ret = 0;

	mutex_lock(&hs_amp_lock);
	if(hs_amp.aud_amp_f)
		ret = hs_amp.aud_amp_f(on,dsp);
	mutex_unlock(&hs_amp_lock);
	return ret;
}

void htc_acoustic_register_hs_amp(int (*aud_hs_amp_f)(int, int), struct file_operations* ops)
{
	mutex_lock(&hs_amp_lock);
	hs_amp.aud_amp_f = aud_hs_amp_f;
	hs_amp.fops = ops;
	mutex_unlock(&hs_amp_lock);
}

void htc_acoustic_register_hs_notify(enum HS_NOTIFY_TYPE type, struct hs_notify_t *notify)
{
	if(notify == NULL)
		return;

	mutex_lock(&hs_nt_lock);
	if(hs_plug_nt[type].used) {
		pr_err("%s: hs notification %d is reigstered\n",__func__,(int)type);
	} else {
		hs_plug_nt[type].private_data = notify->private_data;
		hs_plug_nt[type].callback_f = notify->callback_f;
		hs_plug_nt[type].used = 1;
	}
	mutex_unlock(&hs_nt_lock);
}

void htc_acoustic_register_ops(struct acoustic_ops *ops)
{
        D("acoustic_register_ops \n");
	mutex_lock(&api_lock);
	the_ops = ops;
	mutex_unlock(&api_lock);
}

int htc_acoustic_query_feature(enum HTC_FEATURE feature)
{
	int ret = -1;
	mutex_lock(&api_lock);
	switch(feature) {
		case HTC_Q6_EFFECT:
			if(the_ops && the_ops->get_q6_effect)
				ret = the_ops->get_q6_effect();
			break;
		case HTC_AUD_24BIT:
			if(the_ops && the_ops->enable_24b_audio)
				ret = the_ops->enable_24b_audio();
			break;
		default:
			break;

	};
	mutex_unlock(&api_lock);
	return ret;
}

static int acoustic_open(struct inode *inode, struct file *file)
{
	D("open\n");
	return 0;
}

static int acoustic_release(struct inode *inode, struct file *file)
{
	D("release\n");
	return 0;
}

static long
acoustic_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int rc = 0;
	unsigned char *buf = NULL;
	unsigned int us32_size = 0;
	int s32_value = 0;
	void __user *argp = (void __user *)arg;

	if (_IOC_TYPE(cmd) != ACOUSTIC_IOCTL_MAGIC)
		return -ENOTTY;


	us32_size = _IOC_SIZE(cmd);

	buf = kzalloc(us32_size, GFP_KERNEL);

	if (buf == NULL) {
		E("%s %d: allocate kernel buffer failed.\n", __func__, __LINE__);
		return -EFAULT;
	}

	if (_IOC_DIR(cmd) & _IOC_WRITE) {
		rc = copy_from_user(buf, argp, us32_size);
		if (rc) {
			E("%s %d: copy_from_user fail.\n", __func__, __LINE__);
			rc = -EFAULT;
		}
		else {
			D("%s %d: copy_from_user ok. size=%#x\n", __func__, __LINE__, us32_size);
		}
	}

	mutex_lock(&api_lock);
	switch (cmd) {
	case ACOUSTIC_SET_Q6_EFFECT: {
		if(sizeof(s32_value) <= us32_size) {
			memcpy((void*)&s32_value, (void*)buf, sizeof(s32_value));
			D("%s %d: ACOUSTIC_SET_Q6_EFFECT %#x\n", __func__, __LINE__, s32_value);
			if (s32_value < -1 || s32_value > 1) {
				E("unsupported Q6 mode: %d\n", s32_value);
				rc = -EINVAL;
				break;
			}
			if (the_ops->set_q6_effect) {
				the_ops->set_q6_effect(s32_value);
			}
		}
		else {
			E("%s %d: ACOUSTIC_SET_Q6_EFFECT error.\n", __func__, __LINE__);
			rc = -EINVAL;
		}
		break;
	}
	case ACOUSTIC_GET_HTC_REVISION:
		if (the_ops->get_htc_revision) {
			s32_value = the_ops->get_htc_revision();
		}
		else {
			s32_value = 1;
		}
		if(sizeof(s32_value) <= us32_size) {
			memcpy((void*)buf, (void*)&s32_value, sizeof(s32_value));
			D("%s %d: ACOUSTIC_GET_HTC_REVISION %#x\n", __func__, __LINE__, s32_value);
		}
		else {
			E("%s %d: ACOUSTIC_GET_HTC_REVISION error.\n", __func__, __LINE__);
			rc = -EINVAL;
		}
		break;
	case ACOUSTIC_GET_HW_COMPONENT:
		if (the_ops->get_hw_component) {
			s32_value = the_ops->get_hw_component();
		}
		else {
			s32_value = 0;
		}
		if(sizeof(s32_value) <= us32_size) {
			memcpy((void*)buf, (void*)&s32_value, sizeof(s32_value));
			D("%s %d: ACOUSTIC_GET_HW_COMPONENT %#x\n", __func__, __LINE__, s32_value);
		}
		else {
			E("%s %d: ACOUSTIC_GET_HW_COMPONENT error.\n", __func__, __LINE__);
			rc = -EINVAL;
		}
		break;
	case ACOUSTIC_GET_DMIC_INFO:
		if (the_ops->enable_digital_mic) {
			s32_value = the_ops->enable_digital_mic();
		}
		else {
			s32_value = 0;
		}
		if(sizeof(s32_value) <= us32_size) {
			memcpy((void*)buf, (void*)&s32_value, sizeof(s32_value));
			D("%s %d: ACOUSTIC_GET_DMIC_INFO %#x\n", __func__, __LINE__, s32_value);
		}
		else {
			E("%s %d: ACOUSTIC_GET_DMIC_INFO error.\n", __func__, __LINE__);
			rc = -EINVAL;
		}
		break;
	case ACOUSTIC_GET_MID: {
		char *mid = NULL;
		if (the_ops->get_mid) {
			mid = the_ops->get_mid();
			if(strlen(mid) <= us32_size) {
				strncpy((char*)buf, (char*)mid, strlen(mid));
				D("%s %d: ACOUSTIC_GET_MID %s\n", __func__, __LINE__, mid);
			}
			else {
				E("%s %d: ACOUSTIC_GET_MID error.\n", __func__, __LINE__);
				rc = -EINVAL;
			}
		}
		else {
			if(strlen("Generic") <= us32_size ) {
				strncpy((char*)buf, (char*)"Generic", strlen("Generic"));
				D("%s %d: ACOUSTIC_GET_MID %s\n", __func__, __LINE__, "Generic");
			}
			else {
				E("%s %d: ACOUSTIC_GET_MID error.\n", __func__, __LINE__);
				rc = -EINVAL;
			}
		}
		break;
	}
	case ACOUSTIC_UPDATE_BEATS_STATUS: {
		if(sizeof(s32_value) <= us32_size) {
			memcpy((void*)&s32_value, (void*)buf, sizeof(s32_value));
			D("%s %d: ACOUSTIC_UPDATE_BEATS_STATUS %#x\n", __func__, __LINE__, s32_value);
			if (s32_value < -1 || s32_value > 1) {
				rc = -EINVAL;
				break;
			}
			sdev_beats.state = -1;
			switch_set_state(&sdev_beats, s32_value);
		}
		else {
			E("%s %d: ACOUSTIC_UPDATE_BEATS_STATUS error.\n", __func__, __LINE__);
			rc = -EINVAL;
		}
		break;
	}
	case ACOUSTIC_UPDATE_DQ_STATUS: {
		if(sizeof(s32_value) <= us32_size) {
			memcpy((void*)&s32_value, (void*)buf, sizeof(s32_value));
			D("%s %d: ACOUSTIC_UPDATE_DQ_STATUS %#x\n", __func__, __LINE__, s32_value);
			if (s32_value < -1 || s32_value > 1) {
				rc = -EINVAL;
				break;
			}
			sdev_dq.state = -1;
			switch_set_state(&sdev_dq, s32_value);
		}
		else {
			E("%s %d: ACOUSTIC_UPDATE_DQ_STATUS error.\n", __func__, __LINE__);
			rc = -EINVAL;
		}
		break;
	}
	case ACOUSTIC_CONTROL_WAKELOCK: {
		if(sizeof(s32_value) <= us32_size) {
			memcpy((void*)&s32_value, (void*)buf, sizeof(s32_value));
			D("%s %d: ACOUSTIC_CONTROL_WAKELOCK %#x\n", __func__, __LINE__, s32_value);
			if (s32_value < -1 || s32_value > 1) {
				rc = -EINVAL;
				break;
			}
			if (s32_value == 1) {
				wake_lock_timeout(&htc_acoustic_wakelock, 60*HZ);
			}
			else {
				wake_lock_timeout(&htc_acoustic_wakelock_timeout, 1*HZ);
				wake_unlock(&htc_acoustic_wakelock);
			}
		}
		else {
			E("%s %d: ACOUSTIC_CONTROL_WAKELOCK error.\n", __func__, __LINE__);
			rc = -EINVAL;
		}
		break;
	}
	case ACOUSTIC_DUMMY_WAKELOCK: {
		wake_lock_timeout(&htc_acoustic_dummy_wakelock, 5*HZ);
		break;
	}
	case ACOUSTIC_UPDATE_LISTEN_NOTIFICATION: {
		if(sizeof(s32_value) <= us32_size) {
			memcpy((void*)&s32_value, (void*)buf, sizeof(s32_value));
			D("%s %d: ACOUSTIC_UPDATE_LISTEN_NOTIFICATION %#x\n", __func__, __LINE__, s32_value);
			if (s32_value < -1 || s32_value > 1) {
				rc = -EINVAL;
				break;
			}
			sdev_listen_notification.state = -1;
			switch_set_state(&sdev_listen_notification, s32_value);
		}
		else {
			E("%s %d: ACOUSTIC_UPDATE_LISTEN_NOTIFICATION error.\n", __func__, __LINE__);
			rc = -EINVAL;
		}
		break;
	}
	case ACOUSTIC_RAMDUMP:
#if 0
		pr_err("trigger ramdump by user space\n");
		if (copy_from_user(&mode, (void *)arg, sizeof(mode))) {
			rc = -EFAULT;
			break;
		}

		if (mode >= 4100 && mode <= 4800) {
			dump_stack();
			pr_err("msgid = %d\n", mode);
			BUG();
		}
#endif
		break;
	case ACOUSTIC_GET_HW_REVISION: {
#if 0
			struct device_info info;
			info.pcb_id = system_rev;
			info.sku_id = 0;
			if(sizeof(struct device_info) <= us32_size) {
				memcpy((void*)buf, (void*)&info, sizeof(struct device_info));
				D("%s %d: ACOUSTIC_GET_HW_REVISION pcb_id=%#x, sku_id=%#x\n", __func__, __LINE__, info.pcb_id, info.sku_id);
			}
			else {
				E("%s %d: ACOUSTIC_GET_HW_REVISION error.\n", __func__, __LINE__);
				rc = -EINVAL;
			}
#endif
		break;
	}
	case ACOUSTIC_AMP_CTRL: {
		struct amp_ctrl ampctrl;
		if(sizeof(struct amp_ctrl) <= us32_size) {
			memcpy((void*)&ampctrl, (void*)buf, sizeof(struct amp_ctrl));
			D("%s %d: ACOUSTIC_AMP_CTRL type=%#x\n", __func__, __LINE__, ampctrl.type);
			if(ampctrl.type == AMP_HEADPONE) {
				mutex_lock(&hs_amp_lock);
				if(hs_amp.fops && hs_amp.fops->unlocked_ioctl) {
					hs_amp.fops->unlocked_ioctl(file, cmd, arg);
				}
				mutex_unlock(&hs_amp_lock);
			} else if (ampctrl.type == AMP_SPEAKER) {
#if 0
				mutex_lock(&spk_amp_lock);
				for(i=0; i<SPK_AMP_MAX; i++) {
					if(spk_amp[i].fops && spk_amp[i].fops->unlocked_ioctl) {
						spk_amp[i].fops->unlocked_ioctl(file, cmd, arg);
					}
				}
				mutex_unlock(&spk_amp_lock);
#else
                E("%s %d: ACOUSTIC_AMP_CTRL error. Please use climax tool instead!!!\n", __func__, __LINE__);
#endif
			}
			
			kfree(buf);
			mutex_unlock(&api_lock);
			return 0;
			
		}
		else {
			E("%s %d: ACOUSTIC_AMP_CTRL error.\n", __func__, __LINE__);
			rc = -EINVAL;
		}
		break;
	}
	case ACOUSTIC_KILL_PID: {
		struct pid *pid_struct = NULL;
		if(sizeof(s32_value) <= us32_size) {
			memcpy((void*)&s32_value, (void*)buf, sizeof(s32_value));
			D("%s %d: ACOUSTIC_KILL_PID %#x\n", __func__, __LINE__, s32_value);
			if (s32_value <= 0) {
				rc = -EINVAL;
				break;
			}
			pid_struct = find_get_pid(s32_value);
			if (pid_struct) {
				kill_pid(pid_struct, SIGKILL, 1);
				D("kill pid: %d", s32_value);
			}
		}
		else {
			E("%s %d: ACOUSTIC_KILL_PID error.\n", __func__, __LINE__);
			rc = -EINVAL;
		}
		break;
	}
	default:
		rc= -EINVAL;
		break;
	}

	if(0 == rc) {
		if (_IOC_DIR(cmd) & _IOC_READ) {
			rc = copy_to_user(argp, buf, us32_size);
			if (rc) {
				E("%s %d: copy_to_user fail.\n", __func__, __LINE__);
				rc = -EFAULT;
			}
			else {
				D("%s %d: copy_to_user ok. size=%#x\n", __func__, __LINE__, us32_size);
			}
		}
	}

	kfree(buf);
	mutex_unlock(&api_lock);
	return rc;
}

static ssize_t beats_print_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "Beats\n");
}

static ssize_t dq_print_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "DQ\n");
}

static ssize_t listen_notification_print_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "Listen_notification\n");
}

static int hs_amp_open(struct inode *inode, struct file *file)
{
	int ret = 0;
	const struct file_operations *old_fops = NULL;

	mutex_lock(&hs_amp_lock);

	if(hs_amp.fops) {
		old_fops = file->f_op;

		file->f_op = fops_get(hs_amp.fops);

		if (file->f_op == NULL) {
			file->f_op = old_fops;
			ret = -ENODEV;
		}
	}
	mutex_unlock(&hs_amp_lock);

	if(ret >= 0) {

		if (file->f_op->open) {
			ret = file->f_op->open(inode, file);
			if (ret) {
				fops_put(file->f_op);
				if(old_fops)
					file->f_op = fops_get(old_fops);
				return ret;
			}
		}

		if(old_fops)
			fops_put(old_fops);
	}

	return ret;
}

static int hs_amp_release(struct inode *inode, struct file *file)
{
	return 0;
}

static long hs_amp_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	return 0;
}

void htc_amp_power_register_ops(struct amp_power_ops *ops)
{
	D("%s", __func__);
	the_amp_power_ops = ops;
}

void htc_amp_power_enable(bool enable)
{
	D("%s", __func__);
	if(the_amp_power_ops && the_amp_power_ops->set_amp_power_enable)
		the_amp_power_ops->set_amp_power_enable(enable);
}

static int htc_acoustic_hsnotify(int on)
{
	int i = 0;
	mutex_lock(&hs_nt_lock);
	for(i=0; i<HS_N_MAX; i++) {
		if(hs_plug_nt[i].used && hs_plug_nt[i].callback_f)
			hs_plug_nt[i].callback_f(hs_plug_nt[i].private_data, on);
	}
	mutex_unlock(&hs_nt_lock);

	return 0;
}

static int aud_ftm_sim_gpio_config(void *user_data, enum AUD_FTM_BTPCM_MODE mode)
{
	return 0;
}

static int aud_ftm_sim_gpio_read(void *user_data, int *state)
{
	int i;
	if (state == NULL)
		return -EINVAL;

	for (i = 0 ; i < 4 ; i++)
		state[i] = ((aud_ftm_btpcm_sim_state >> i) & 0x1);

	return 0;
}

static struct aud_ftm_btpcm_func_t aud_ftm_btpcm_func_sim = {
	.gpio_config = aud_ftm_sim_gpio_config,
	.gpio_read = aud_ftm_sim_gpio_read,
	.user_data = NULL,
};

static int aud_ftm_open(struct inode *inode, struct file *file)
{
#if AUD_FTM_DEBUG
	D("%s\n", __func__);
#endif
	return 0;
}

static int aud_ftm_release(struct inode *inode, struct file *file)
{
#if AUD_FTM_DEBUG
	D("%s\n", __func__);
#endif
	return 0;
}

static ssize_t process_ftm_read(char *gpio_values)
{
	int pin_state[4] = {0};
	ssize_t ret = 0;

	if (gpio_values == NULL)
		return -EINVAL;

	if (aud_ftm_btpcm_func.init == 0 && aud_ftm_btpcm_func.sim == 0) {
		E("%s: Not using BRCM BT chip.\n", __func__);
		return -EIO;
	}

	if (aud_ftm_btpcm_mode != AUD_FTM_BTPCM_MODE_GPIO) {
		E("%s state not GPIO\n", __func__);
		return -EPERM;
	}

	if (aud_ftm_btpcm_func.gpio_read) {
		ret = aud_ftm_btpcm_func.gpio_read(aud_ftm_btpcm_func.user_data, pin_state);
		if (ret < 0) {
			E("%s gpio_read failed (%zd)\n", __func__, ret);
			return ret;
		}

		*gpio_values = (pin_state[0] & 1) +		
				((pin_state[1] & 1) << 1) +	
				((pin_state[2] & 1) << 2) +	
				((pin_state[3] & 1) << 3);	
	}

	return ret;
}

static ssize_t process_ftm_config(enum AUD_FTM_BTPCM_MODE mode, int sim_value)
{
	if (mode >= AUD_FTM_BTPCM_MODE_CNT || mode < 0)
		return -EINVAL;

#if AUD_FTM_DEBUG
	D("%s mode %d, sim_value %d\n", __func__, mode, sim_value);
#endif

	if (mode == AUD_FTM_BTPCM_MODE_GPIO) {
		if (aud_ftm_btpcm_func.init == 0 && aud_ftm_btpcm_func.sim == 0) {
			E("%s: Not using BRCM BT chip.\n", __func__);
			mutex_unlock(&aud_ftm_lock);
			return -EIO;
		}

		
		if (aud_ftm_btpcm_func.gpio_config) {
			aud_ftm_btpcm_func.gpio_config(aud_ftm_btpcm_func.user_data, AUD_FTM_BTPCM_MODE_GPIO);
			aud_ftm_btpcm_mode = AUD_FTM_BTPCM_MODE_GPIO;
			D("%s BT PCM pins configured as GPIO input.\n", __func__);
		} else {
			E("%s gpio_config not registered.\n", __func__);
		}
	} else if (mode == AUD_FTM_BTPCM_MODE_PCM) {
		if (aud_ftm_btpcm_func.init == 0 && aud_ftm_btpcm_func.sim == 0) {
			E("%s: Not using BRCM BT chip.\n", __func__);
			mutex_unlock(&aud_ftm_lock);
			return -EIO;
		}

		
		if (aud_ftm_btpcm_func.gpio_config) {
			aud_ftm_btpcm_func.gpio_config(aud_ftm_btpcm_func.user_data, AUD_FTM_BTPCM_MODE_PCM);
			aud_ftm_btpcm_mode = AUD_FTM_BTPCM_MODE_PCM;
			D("%s BT PCM pins configured as PCM.\n", __func__);
		} else {
			E("%s gpio_config not registered.\n", __func__);
		}
	} else if (mode == AUD_FTM_BTPCM_MODE_SIM) {
		
		if (sim_value >= 0x0 && sim_value <= 0xf)
			aud_ftm_btpcm_sim_state = sim_value;
		else
			aud_ftm_btpcm_sim_state = 0xf;

		D("%s BT PCM simulation: %x\n", __func__, aud_ftm_btpcm_sim_state);
		aud_ftm_func_register(&aud_ftm_btpcm_func_sim);
		aud_ftm_btpcm_func.sim = 1;
	}
	return 0;
}

static ssize_t aud_ftm_read(struct file *file, char __user *string, size_t size, loff_t *offset)
{
	ssize_t ret = 0;
	char gpio_values = 0;
	char state[8] = {0};
#if AUD_FTM_DEBUG
	D("%s enter\n", __func__);
#endif

	if (size < 2) {
		E("%s Buffer length not enough. (At least 2 bytes)\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&aud_ftm_lock);
	ret = process_ftm_read(&gpio_values);
	if (ret == 0) {
		sprintf(state, "%x", gpio_values);
		ret = copy_to_user(string, state, 1);
	}
	mutex_unlock(&aud_ftm_lock);

#if AUD_FTM_DEBUG
	D("%s leave\n", __func__);
#endif
	return ret;
}

static ssize_t aud_ftm_write(struct file *file, const char __user *string, size_t size, loff_t *offset)
{
	char cmd_string[16] = {0};
	ssize_t ret = 0;
#if AUD_FTM_DEBUG
	D("%s enter\n", __func__);
#endif

	if (size < 0 || size >= 16 || string == NULL)
		return -EINVAL;

	mutex_lock(&aud_ftm_lock);
	ret = copy_from_user(cmd_string, string, size);
	if (ret < 0) {
		E("%s Failed to copy cmd string\n", __func__);
		mutex_unlock(&aud_ftm_lock);
		return ret;
	}
	if (!strncmp(ACOUSTIC_FTM_STRING_BTPCM_GPIO, cmd_string, strlen(ACOUSTIC_FTM_STRING_BTPCM_GPIO))) {
		ret = process_ftm_config(AUD_FTM_BTPCM_MODE_GPIO, (-1));
	} else if (!strncmp(ACOUSTIC_FTM_STRING_BTPCM_PCM, cmd_string, strlen(ACOUSTIC_FTM_STRING_BTPCM_PCM))) {
		ret = process_ftm_config(AUD_FTM_BTPCM_MODE_PCM, (-1));
	} else if (!strncmp(ACOUSTIC_FTM_STRING_BTPCM_SIM, cmd_string, strlen(ACOUSTIC_FTM_STRING_BTPCM_SIM))) {
		int sim_value = -1;
		if (size > strlen(ACOUSTIC_FTM_STRING_BTPCM_SIM)) {
			char state = cmd_string[strlen(ACOUSTIC_FTM_STRING_BTPCM_SIM)];
			if (state >= '0' && state <= '9')
				sim_value = state - '0';
			else if (state >= 'a' && state <= 'f')
				sim_value = state - 'a' + 0xa;
			else if (state >= 'A' && state <= 'F')
				sim_value = state - 'A' + 0xa;
		}
		ret = process_ftm_config(AUD_FTM_BTPCM_MODE_SIM, sim_value);
	}
	mutex_unlock(&aud_ftm_lock);

#if AUD_FTM_DEBUG
	D("%s leave\n", __func__);
#endif
	return (ret ? ret : size);
}

static long aud_ftm_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret = 0;
	char gpio_values = 0;
#if AUD_FTM_DEBUG
	D("%s enter\n", __func__);
#endif
	mutex_lock(&aud_ftm_lock);

	switch (cmd) {
	case ACOUSTIC_FTM_BTPCM_SET_GPIO:
		ret = process_ftm_config(AUD_FTM_BTPCM_MODE_GPIO, (-1));
		D("%s ACOUSTIC_FTM_BTPCM_SET_GPIO\n", __func__);
		break;

	case ACOUSTIC_FTM_BTPCM_SET_PCM:
		ret = process_ftm_config(AUD_FTM_BTPCM_MODE_PCM, (-1));
		D("%s ACOUSTIC_FTM_BTPCM_SET_PCM\n", __func__);
		break;

	case ACOUSTIC_FTM_BTPCM_READ:
		if ((void *)arg == NULL) {
			E("%s ACOUSTIC_FTM_BTPCM_READ: beed buffer to read gpio values\n", __func__);
			mutex_unlock(&aud_ftm_lock);
			return -EINVAL;
		}

		ret = process_ftm_read(&gpio_values);
		if (ret == 0) {
			ret = copy_to_user((void *)arg, &gpio_values, sizeof(gpio_values));
		}

		D("%s ACOUSTIC_FTM_BTPCM_READ: gpio_values %x\n", __func__, gpio_values);
		break;

	default:
		D("%s Unknown command %d\n", __func__, cmd);
		break;
	}

	mutex_unlock(&aud_ftm_lock);
#if AUD_FTM_DEBUG
	D("%s leave\n", __func__);
#endif
	return ret;
}

void aud_ftm_func_register(struct aud_ftm_btpcm_func_t *funcs)
{
	if (funcs->gpio_config == NULL || funcs->gpio_read == NULL)
		return;

	aud_ftm_btpcm_func.gpio_config = funcs->gpio_config;
	aud_ftm_btpcm_func.gpio_read = funcs->gpio_read;
	aud_ftm_btpcm_func.user_data = funcs->user_data;
	aud_ftm_btpcm_func.init = 1;
}
EXPORT_SYMBOL(aud_ftm_func_register);

static struct file_operations acoustic_fops = {
	.owner = THIS_MODULE,
	.open = acoustic_open,
	.release = acoustic_release,
	.unlocked_ioctl = acoustic_ioctl,
	.compat_ioctl = acoustic_ioctl,
};

static struct miscdevice acoustic_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "htc-acoustic",
	.fops = &acoustic_fops,
};

static int __init acoustic_init(void)
{
	int ret = 0;
	struct headset_notifier notifier;

	ret = misc_register(&acoustic_misc);
	wake_lock_init(&htc_acoustic_wakelock, WAKE_LOCK_SUSPEND, "htc_acoustic");
	wake_lock_init(&htc_acoustic_wakelock_timeout, WAKE_LOCK_SUSPEND, "htc_acoustic_timeout");
	wake_lock_init(&htc_acoustic_dummy_wakelock, WAKE_LOCK_SUSPEND, "htc_acoustic_dummy");

	if (ret < 0) {
		pr_err("failed to register misc device!\n");
		return ret;
	}

	ret = misc_register(&hs_amp_misc);
	if (ret < 0) {
		pr_err("failed to register aud hs amp device!\n");
		return ret;
	}

	ret = misc_register(&aud_ftm_misc);
	if (ret < 0) {
		pr_err("failed to register aud ftm device!\n");
		return ret;
	}

	sdev_beats.name = "Beats";
	sdev_beats.print_name = beats_print_name;

	ret = switch_dev_register(&sdev_beats);
	if (ret < 0) {
		pr_err("failed to register beats switch device!\n");
		return ret;
	}

	sdev_dq.name = "DQ";
	sdev_dq.print_name = dq_print_name;

	ret = switch_dev_register(&sdev_dq);
	if (ret < 0) {
		pr_err("failed to register DQ switch device!\n");
		return ret;
	}

	sdev_listen_notification.name = "Listen_notification";
	sdev_listen_notification.print_name = listen_notification_print_name;

	ret = switch_dev_register(&sdev_listen_notification);
	if (ret < 0) {
		pr_err("failed to register listen_notification switch device!\n");
		return ret;
	}

	notifier.id = HEADSET_REG_HS_INSERT;
	notifier.func = htc_acoustic_hsnotify;
	headset_notifier_register(&notifier);

	return 0;
}

static void __exit acoustic_exit(void)
{
	misc_deregister(&acoustic_misc);
}

module_init(acoustic_init);
module_exit(acoustic_exit);
