/*
 * Copyright (C) 2011 HTC, Inc.
 * Author: Dyson Lee <Dyson@intel.com>
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

static int manual_serialno_flag = 0;
static char mfg_whiteline_serialno[] = "000000000000";
static unsigned int usb_project_pid;
static unsigned int vzw_unmount_cdrom_enable;

void htc_setprop(char *func)
{
	int	call_us_ret = -1;

	char *envp[] = {
		"HOME=/",
		"PATH=/sbin:/system/sbin:/system/bin:/system/xbin",
		NULL,
	};
	char *exec_path[1] = {"/system/bin/setprop"};
	char *argv_setprop[] = { exec_path[0], "sys.usb.config", func, NULL,};
	call_us_ret = call_usermodehelper(exec_path[0],
				argv_setprop, envp, UMH_WAIT_PROC);
}

void android_set_serialno(char *serialno)
{
	strings_dev[STRING_SERIAL_IDX].s = serialno;
}

void init_mfg_serialno(void)
{
	if(!strcmp(htc_get_bootmode(),"factory2") || !strcmp(htc_get_bootmode(),"ftm"))
		android_set_serialno(mfg_whiteline_serialno);
	return;
}
static void setup_usb_denied(int htc_mode)
{
        if (htc_mode)
                _android_dev->autobot_mode = 1;
        else
                _android_dev->autobot_mode = 0;
}

static int usb_autobot_mode(void)
{
        if (_android_dev->autobot_mode)
                return 1;
        else
                return 0;
}

void android_switch_htc_mode(void)
{
	htc_usb_enable_function("adb,mass_storage,serial,projector", 1);
}

int mfg_check_white_accessory(int accessory_type)
{
	static int previous_type = 0;
	int reset_flag = 0;
	struct android_dev *dev;
	struct usb_composite_dev *cdev;

	if (manual_serialno_flag || (strcmp(htc_get_bootmode(),"factory2") && strcmp(htc_get_bootmode(),"ftm")))
		return 0;

	if (!_android_dev)
		return -EAGAIN;

	dev = _android_dev;
	cdev = dev->cdev;

	USB_INFO("%s : accessory type %d , previous type %d, connected %d\n", __func__, accessory_type, previous_type, dev->connected);
	switch (accessory_type)
	{
		case DOCK_STATE_CAR:
			mfg_whiteline_serialno[11] = '3';
			android_set_serialno(mfg_whiteline_serialno);
			if (previous_type != accessory_type) {
				previous_type = accessory_type;
				reset_flag = 1;
			}
			break;
		case DOCK_STATE_USB_HEADSET:
			mfg_whiteline_serialno[11] = '4';
			android_set_serialno(mfg_whiteline_serialno);
			if (previous_type != accessory_type) {
				previous_type = accessory_type;
				reset_flag = 1;
			}
			break;
		case DOCK_STATE_DMB:
			mfg_whiteline_serialno[11] = '5';
			android_set_serialno(mfg_whiteline_serialno);
			if (previous_type != accessory_type) {
				previous_type = accessory_type;
				reset_flag = 1;
			}
			break;
		default:
			mfg_whiteline_serialno[11] = '0';
			android_set_serialno(mfg_whiteline_serialno);
			if (previous_type != 0) {
				previous_type = 0;
				reset_flag = 1;
			}
			break;
	}
	if (reset_flag) {
		if (dev->connected) {
			schedule_delayed_work(&cdev->request_reset,REQUEST_RESET_DELAYED);
		}
	}
	return 0;
}

int htc_usb_enable_function(char *name, int ebl)
{
	struct android_dev *dev = _android_dev;
	struct usb_composite_dev *cdev = dev->cdev;
	char state_buf[60];
	char name_buf[60];
	char *function[3];
	if (!strcmp(name, "ffs"))
		snprintf(state_buf, sizeof(state_buf), "SWITCH_STATE=%s", "adb");
	else
		snprintf(state_buf, sizeof(state_buf), "SWITCH_STATE=%s", name);
	function[0] = state_buf;
	function[2] = NULL;

	if (ebl) {
		snprintf(name_buf, sizeof(name_buf),
				"SWITCH_NAME=%s", "function_switch_on");
		function[1] = name_buf;
		kobject_uevent_env(&cdev->sw_function_switch_on.dev->kobj, KOBJ_CHANGE,
				function);
	} else {
		snprintf(name_buf, sizeof(name_buf),
				"SWITCH_NAME=%s", "function_switch_off");
		function[1] = name_buf;
		kobject_uevent_env(&cdev->sw_function_switch_off.dev->kobj, KOBJ_CHANGE,
				function);
	}
	return 0;

}

static int usb_disable;

const char * add_usb_radio_debug_function(const char *buff) {
	if (!strcmp(buff, "mtp,adb,mass_storage")) 
		return "mtp,adb,mass_storage,diag,modem,rmnet";
	else if (!strcmp(buff, "mtp,adb,mass_storage,acm")) 
		return "mtp,adb,mass_storage,diag,modem,rmnet";
	else if (!strcmp(buff, "mtp,mass_storage")) 
		return "mtp,mass_storage,diag,modem,rmnet";
	else if (!strcmp(buff, "mtp,mass_storage,acm")) 
		return "mtp,mass_storage,diag,modem,rmnet";
	else if (!strcmp(buff, "ffs,acm")) 
		return "adb,diag,modem,acm";
	USB_INFO("switch to radio debug function:%s\n", buff);
	return buff;
}

void check_usb_vid_pid(struct usb_composite_dev *cdev) {
	switch(cdev->desc.idProduct) {
		case 0x0f90:
		case 0x0f63:
			cdev->desc.idVendor = 0x0bb4;
			cdev->desc.idProduct = 0x0fa2;
			break;
		case 0x0f25:
		case 0x0f26:
		case 0x0f91:
		case 0x0f64:
			cdev->desc.idVendor = 0x0bb4;
			cdev->desc.idProduct = 0x0fa3;
			break;
		case 0x0f15:
			cdev->desc.idVendor = 0x0bb4;
			cdev->desc.idProduct = 0x0f17;
		default:
			break;
	}
	return;
}
void check_usb_project_pid(struct usb_composite_dev *cdev) {
	if (cdev->desc.idProduct == 0x0f90 && usb_project_pid != 0x0000) {
		cdev->desc.idVendor = 0x0bb4;
		cdev->desc.idProduct = usb_project_pid;
	}
	return;
}

static int __init get_usb_project_pid(char *str)
{
	int ret = kstrtouint(str, 0, &usb_project_pid);
	pr_info("androidusb.pid %d: %08x from %26s\r\n",
			ret, usb_project_pid, str);
	return ret;
} early_param("androidusb.pid", get_usb_project_pid);

static int __init get_vzw_unmount_cdrom_enable(char *str)
{
	int ret = kstrtouint(str, 0, &vzw_unmount_cdrom_enable);
	pr_info("vzw_unmount_cdrom_enable %d: %08x from %26s\r\n",
			ret, vzw_unmount_cdrom_enable, str);
	return ret;
} early_param("vzw_unmount_cdrom_enable", get_vzw_unmount_cdrom_enable);

static int __init get_disk_mode_enable(char *str)
{
	int ret = kstrtouint(str, 0, &disk_mode_enable);
	pr_info("disk_mode_enable %d: %08x from %26s\r\n",
			ret, disk_mode_enable, str);
	return ret;
} early_param("disk_mode_enable", get_disk_mode_enable);

static ssize_t iSerial_show(struct device *dev, struct device_attribute *attr,
	char *buf);
static ssize_t store_dummy_usb_serial_number(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t size)
{
	struct android_dev *android_dev = _android_dev;
	struct usb_composite_dev *cdev = android_dev->cdev;
	int loop_i;

	if (size >= sizeof(serial_string)) {
		USB_INFO("%s(): input size > %zu\n",
			__func__, sizeof(serial_string));
		return -EINVAL;
	}

	for (loop_i = 0; loop_i < size; loop_i++) {
		if (buf[loop_i] >= 0x30 && buf[loop_i] <= 0x39) 
			continue;
		else if (buf[loop_i] >= 0x41 && buf[loop_i] <= 0x5A) 
			continue;
		if (buf[loop_i] == 0x0A) 
			continue;
		else {
			USB_INFO("%s(): get invaild char (0x%2.2X)\n",
					__func__, buf[loop_i]);
			return -EINVAL;
		}
	}
	strlcpy(serial_string, buf, sizeof(serial_string));
	strim(serial_string);
	android_set_serialno(serial_string);
	if (android_dev->connected)
		schedule_delayed_work(&cdev->request_reset,REQUEST_RESET_DELAYED);
	cdev->do_serial_number_change = true;
	manual_serialno_flag = 1;
	return size;
}
static const char *os_to_string(int os_type)
{
	switch (os_type) {
	case OS_NOT_YET:	return "OS_NOT_YET";
	case OS_MAC:		return "OS_MAC";
	case OS_LINUX:		return "OS_LINUX";
	case OS_WINDOWS:	return "OS_WINDOWS";
	default:		return "UNKNOWN";
	}
}
static ssize_t show_os_type(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned length;

	length = sprintf(buf, "%d\n", os_type);
	USB_INFO("%s: %s, %s", __func__, os_to_string(os_type), buf);
	return length;
}

static int usb_ats = 0;
static ssize_t store_ats(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf, "%d ", &usb_ats);
	return count;
}

static ssize_t show_ats(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned length;

	length = sprintf(buf, "%d\n", (get_debug_flag() & 0x100) || usb_ats);
	USB_INFO("%s: %s\n", __func__, buf);
	return length;
}

static ssize_t show_usb_disable_setting(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned length;

	length = sprintf(buf, "%d\n", usb_disable);
	return length;
}

void htc_dwc3_disable_usb(int disable_usb);
static ssize_t store_usb_disable_setting(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int disable_usb_function;

	sscanf(buf, "%d ", &disable_usb_function);
	USB_INFO("USB_disable set %d\n", disable_usb_function);
	usb_disable = disable_usb_function;
	htc_dwc3_disable_usb(disable_usb_function);
	return count;
}

static ssize_t show_usb_ac_cable_status(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	unsigned length;
	length = sprintf(buf, "%d",usb_get_connect_type());
	return length;
}

static ssize_t show_usb_cable_connect(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned length;
	struct android_dev *and_dev = _android_dev;

	length = sprintf(buf, "%d",(and_dev->connected == 1) && !usb_disable ? 1 : 0);
	return length;
}

static ssize_t show_is_usb_denied(struct device *dev,
                struct device_attribute *attr, char *buf)
{
        unsigned length;
        int deny = 0;

        if (usb_autobot_mode()) {
                deny = 1;
        }

        length = sprintf(buf, "%d\n", deny);
        USB_INFO("%s: %s\n", __func__, buf);
        return length;
}

static int cdrom_unmount;
static ssize_t show_cdrom_unmount(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	unsigned length;

	length = sprintf(buf, "%d\n", cdrom_unmount);
	return length;
}

static ssize_t store_cdrom_unmount(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct usb_composite_dev *cdev = _android_dev->cdev;
	int value;
	sscanf(buf, "%d", &value);
	cdrom_unmount = value;

	if (value == 1 && vzw_unmount_cdrom_enable) {
		printk(KERN_INFO "[USB] Trigger unmount uevent after 30 seconds\n");
		cancel_delayed_work(&cdev->cdusbcmd_vzw_unmount_work);
		cdev->unmount_cdrom_mask = 1 << 3 | 1 << 4;
		schedule_delayed_work(&cdev->cdusbcmd_vzw_unmount_work, 30 * HZ);
	}
	return count;
}

static ssize_t store_usb_modem_enable_setting(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int usb_modem_enable;

	sscanf(buf, "%d ", &usb_modem_enable);
	USB_INFO("modem: enable %d\n", usb_modem_enable);
	htc_usb_enable_function("modem", usb_modem_enable?1:0);
	return count;
}

static DEVICE_ATTR(usb_ac_cable_status, 0444, show_usb_ac_cable_status, NULL);
static DEVICE_ATTR(dummy_usb_serial_number, 0644, iSerial_show, store_dummy_usb_serial_number);
static DEVICE_ATTR(os_type, 0444, show_os_type, NULL);
static DEVICE_ATTR(ats, 0664, show_ats, store_ats);
static DEVICE_ATTR(usb_disable, 0664,show_usb_disable_setting, store_usb_disable_setting);
static DEVICE_ATTR(usb_cable_connect, 0444, show_usb_cable_connect, NULL);
static DEVICE_ATTR(usb_denied, 0444, show_is_usb_denied, NULL);
static DEVICE_ATTR(cdrom_unmount, 0644, show_cdrom_unmount, store_cdrom_unmount);
static DEVICE_ATTR(usb_modem_enable, 0660,NULL, store_usb_modem_enable_setting);
static __maybe_unused struct attribute *android_htc_usb_attributes[] = {
	&dev_attr_dummy_usb_serial_number.attr,
	&dev_attr_os_type.attr,
	&dev_attr_ats.attr,
	&dev_attr_usb_disable.attr,
	&dev_attr_usb_ac_cable_status.attr,
	&dev_attr_usb_cable_connect.attr,
	&dev_attr_usb_denied.attr,
	&dev_attr_cdrom_unmount.attr,
	&dev_attr_usb_modem_enable.attr,
	NULL
};

static  __maybe_unused const struct attribute_group android_usb_attr_group = {
	.attrs = android_htc_usb_attributes,
};

static void setup_vendor_info(struct android_dev *dev)
{
	if (sysfs_create_group(&dev->pdev->dev.kobj, &android_usb_attr_group))
		pr_err("%s: fail to create sysfs\n", __func__);
	
	if (sysfs_create_link(&platform_bus.kobj, &dev->pdev->dev.kobj, "android_usb"))
		pr_err("%s: fail to link android_usb to /sys/devices/platform/\n", __func__);
}
