/*
 * Synaptics DSX touchscreen driver
 *
 * Copyright (C) 2012 Synaptics Incorporated
 *
 * Copyright (C) 2012 Alexandra Chin <alexandra.chin@tw.synaptics.com>
 * Copyright (C) 2012 Scott Lin <scott.lin@tw.synaptics.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/firmware.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include <linux/input/synaptics_dsx_htc.h>
#include "synaptics_dsx_core.h"
#ifdef CONFIG_TOUCHSCREEN_TOUCH_FW_UPDATE
#include <linux/input/touch_fw_update.h>
#endif

#define FW_IMAGE_NAME "synaptics.img"

#define DO_STARTUP_FW_UPDATE

#define FORCE_UPDATE false
#define DO_LOCKDOWN false

#define MAX_IMAGE_NAME_LEN 256
#define MAX_FIRMWARE_ID_LEN 10

#define LOCKDOWN_OFFSET 0xb0
#define IMAGE_AREA_OFFSET 0x100

#define BOOTLOADER_ID_OFFSET 0
#define BLOCK_NUMBER_OFFSET 0

#define V5_PROPERTIES_OFFSET 2
#define V5_BLOCK_SIZE_OFFSET 3
#define V5_BLOCK_COUNT_OFFSET 5
#define V5_BLOCK_DATA_OFFSET 2

#define V6_PROPERTIES_OFFSET 1
#define V6_BLOCK_SIZE_OFFSET 2
#define V6_BLOCK_COUNT_OFFSET 3
#define V6_BLOCK_DATA_OFFSET 1
#define V6_FLASH_COMMAND_OFFSET 2
#define V6_FLASH_STATUS_OFFSET 3

#define LOCKDOWN_BLOCK_COUNT 5

#define REG_MAP (1 << 0)
#define UNLOCKED (1 << 1)
#define HAS_CONFIG_ID (1 << 2)
#define HAS_PERM_CONFIG (1 << 3)
#define HAS_BL_CONFIG (1 << 4)
#define HAS_DISP_CONFIG (1 << 5)
#define HAS_CTRL1 (1 << 6)

#define UI_CONFIG_AREA 0x00
#define PERM_CONFIG_AREA 0x01
#define BL_CONFIG_AREA 0x02
#define DISP_CONFIG_AREA 0x03

#define CMD_WRITE_FW_BLOCK 0x2
#define CMD_ERASE_ALL 0x3
#define CMD_WRITE_LOCKDOWN_BLOCK 0x4
#define CMD_READ_CONFIG_BLOCK 0x5
#define CMD_WRITE_CONFIG_BLOCK 0x6
#define CMD_ERASE_CONFIG 0x7
#define CMD_READ_TW_PIN 0x8
#define CMD_ERASE_BL_CONFIG 0x9
#define CMD_ERASE_DISP_CONFIG 0xa
#define CMD_ENABLE_FLASH_PROG 0xf

#define SLEEP_MODE_NORMAL (0x00)
#define SLEEP_MODE_SENSOR_SLEEP (0x01)
#define SLEEP_MODE_RESERVED0 (0x02)
#define SLEEP_MODE_RESERVED1 (0x03)

#define ENABLE_WAIT_MS (1 * 1000)
#define WRITE_WAIT_MS (3 * 1000)
#define ERASE_WAIT_MS (5 * 1000)

#define MIN_SLEEP_TIME_US 50
#define MAX_SLEEP_TIME_US 100

#define ENTER_FLASH_PROG_WAIT_MS 20

#ifdef CONFIG_TOUCHSCREEN_TOUCH_FW_UPDATE
#define TOUCH_VENDOR "SYNAPTICS"
#define FW_FLASH_TIMEOUT 120
struct touch_fwu_notifier synaptics_tp_notifier;
static void synaptics_fwu_progress(int percentage);
#endif

extern char *htc_get_bootmode(void);
static int fwu_do_reflash(void);
static int fwu_do_write_config(void);

static ssize_t fwu_sysfs_show_image(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count);

static ssize_t fwu_sysfs_store_image(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count);

static ssize_t fwu_sysfs_do_reflash_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t fwu_sysfs_write_config_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t fwu_sysfs_read_config_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t fwu_sysfs_config_area_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t fwu_sysfs_image_name_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t fwu_sysfs_image_size_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t fwu_sysfs_block_size_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t fwu_sysfs_firmware_block_count_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t fwu_sysfs_configuration_block_count_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t fwu_sysfs_perm_config_block_count_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t fwu_sysfs_bl_config_block_count_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t fwu_sysfs_disp_config_block_count_show(struct device *dev,
		struct device_attribute *attr, char *buf);

enum bl_version {
	V5 = 5,
	V6 = 6,
};

enum flash_area {
	NONE,
	UI_FIRMWARE,
	CONFIG_AREA,
};

enum update_mode {
	NORMAL = 1,
	FORCE = 2,
	LOCKDOWN = 8,
};

struct image_header {
	
	unsigned char checksum[4];
	unsigned char reserved_04;
	unsigned char reserved_05;
	unsigned char options_firmware_id:1;
	unsigned char options_bootloader:1;
	unsigned char options_reserved:6;
	unsigned char bootloader_version;
	unsigned char firmware_size[4];
	unsigned char config_size[4];
	
	unsigned char product_id[SYNAPTICS_RMI4_PRODUCT_ID_SIZE];
	unsigned char package_id[2];
	unsigned char package_id_revision[2];
	unsigned char product_info[SYNAPTICS_RMI4_PRODUCT_INFO_SIZE];
	
	unsigned char bootloader_addr[4];
	unsigned char bootloader_size[4];
	unsigned char ui_addr[4];
	unsigned char ui_size[4];
	
	unsigned char ds_id[16];
	
	unsigned char disp_config_addr[4];
	unsigned char disp_config_size[4];
	unsigned char reserved_48_4f[8];
	
	unsigned char firmware_id[4];
};

struct image_header_data {
	bool contains_firmware_id;
	bool contains_bootloader;
	bool contains_disp_config;
	unsigned int firmware_id;
	unsigned int checksum;
	unsigned int firmware_size;
	unsigned int config_size;
	unsigned int bootloader_size;
	unsigned int disp_config_offset;
	unsigned int disp_config_size;
	unsigned char bootloader_version;
	unsigned char product_id[SYNAPTICS_RMI4_PRODUCT_ID_SIZE + 1];
	unsigned char product_info[SYNAPTICS_RMI4_PRODUCT_INFO_SIZE];
};

struct pdt_properties {
	union {
		struct {
			unsigned char reserved_1:6;
			unsigned char has_bsr:1;
			unsigned char reserved_2:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f01_device_status {
	union {
		struct {
			unsigned char status_code:4;
			unsigned char reserved:2;
			unsigned char flash_prog:1;
			unsigned char unconfigured:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f01_device_control {
	union {
		struct {
			unsigned char sleep_mode:2;
			unsigned char nosleep:1;
			unsigned char reserved:2;
			unsigned char charger_connected:1;
			unsigned char report_rate:1;
			unsigned char configured:1;
		} __packed;
		unsigned char data[1];
	};
};

struct synaptics_rmi4_fwu_handle {
	enum bl_version bl_version;
	bool initialized;
	bool program_enabled;
	bool has_perm_config;
	bool has_bl_config;
	bool has_disp_config;
	bool force_update;
	bool in_flash_prog_mode;
	bool do_lockdown;
	unsigned int data_pos;
	unsigned int image_size;
	unsigned char *image_name;
	unsigned char *ext_data_source;
	unsigned char *read_config_buf;
	unsigned char intr_mask;
	unsigned char command;
	unsigned char bootloader_id[2];
	unsigned char flash_properties;
	unsigned char flash_status;
	unsigned char productinfo1;
	unsigned char productinfo2;
	unsigned char properties_off;
	unsigned char blk_size_off;
	unsigned char blk_count_off;
	unsigned char blk_data_off;
	unsigned char flash_cmd_off;
	unsigned char flash_status_off;
	unsigned short block_size;
	unsigned short fw_block_count;
	unsigned short config_block_count;
	unsigned short lockdown_block_count;
	unsigned short perm_config_block_count;
	unsigned short bl_config_block_count;
	unsigned short disp_config_block_count;
	unsigned short config_size;
	unsigned short config_area;
	char product_id[SYNAPTICS_RMI4_PRODUCT_ID_SIZE + 1];
	const unsigned char *firmware_data;
	const unsigned char *config_data;
	const unsigned char *disp_config_data;
	const unsigned char *lockdown_data;
	struct workqueue_struct *fwu_workqueue;
	struct work_struct fwu_work;
	struct synaptics_rmi4_fn_desc f34_fd;
	struct synaptics_rmi4_data *rmi4_data;
	struct wake_lock fwu_wake_lock;
};

static struct bin_attribute dev_attr_data = {
	.attr = {
		.name = "data",
		.mode = (S_IRUGO | S_IWUSR),
	},
	.size = 0,
	.read = fwu_sysfs_show_image,
	.write = fwu_sysfs_store_image,
};

static struct device_attribute attrs[] = {
	__ATTR(doreflash, S_IWUSR,
			synaptics_rmi4_show_error,
			fwu_sysfs_do_reflash_store),
	__ATTR(writeconfig, S_IWUSR,
			synaptics_rmi4_show_error,
			fwu_sysfs_write_config_store),
	__ATTR(readconfig, S_IWUSR,
			synaptics_rmi4_show_error,
			fwu_sysfs_read_config_store),
	__ATTR(configarea, S_IWUSR,
			synaptics_rmi4_show_error,
			fwu_sysfs_config_area_store),
	__ATTR(imagename, S_IWUSR,
			synaptics_rmi4_show_error,
			fwu_sysfs_image_name_store),
	__ATTR(imagesize, S_IWUSR,
			synaptics_rmi4_show_error,
			fwu_sysfs_image_size_store),
	__ATTR(blocksize, S_IRUGO,
			fwu_sysfs_block_size_show,
			synaptics_rmi4_store_error),
	__ATTR(fwblockcount, S_IRUGO,
			fwu_sysfs_firmware_block_count_show,
			synaptics_rmi4_store_error),
	__ATTR(configblockcount, S_IRUGO,
			fwu_sysfs_configuration_block_count_show,
			synaptics_rmi4_store_error),
	__ATTR(permconfigblockcount, S_IRUGO,
			fwu_sysfs_perm_config_block_count_show,
			synaptics_rmi4_store_error),
	__ATTR(blconfigblockcount, S_IRUGO,
			fwu_sysfs_bl_config_block_count_show,
			synaptics_rmi4_store_error),
	__ATTR(dispconfigblockcount, S_IRUGO,
			fwu_sysfs_disp_config_block_count_show,
			synaptics_rmi4_store_error),
};

static struct synaptics_rmi4_fwu_handle *fwu;

DECLARE_COMPLETION(fwu_remove_complete);

static uint32_t syn_crc(uint16_t *data, uint16_t len)
{
	uint32_t sum1, sum2;
	sum1 = sum2 = 0xFFFF;
	if (data) {
		while (len--) {
			sum1 += *data++;
			sum2 += sum1;
			sum1 = (sum1 & 0xFFFF) + (sum1 >> 16);
			sum2 = (sum2 & 0xFFFF) + (sum2 >> 16);
		}
	} else {
		pr_err("%s: data incorrect", __func__);
		return (0xFFFF | 0xFFFF << 16);
	}
	return sum1 | (sum2 << 16);
}

static unsigned int le_to_uint(const unsigned char *ptr)
{
	return (unsigned int)ptr[0] +
			(unsigned int)ptr[1] * 0x100 +
			(unsigned int)ptr[2] * 0x10000 +
			(unsigned int)ptr[3] * 0x1000000;
}

static unsigned int be_to_uint(const unsigned char *ptr)
{
	return (unsigned int)ptr[3] +
			(unsigned int)ptr[2] * 0x100 +
			(unsigned int)ptr[1] * 0x10000 +
			(unsigned int)ptr[0] * 0x1000000;
}

static void parse_header(struct image_header_data *header,
		const unsigned char *fw_image)
{
	struct image_header *data = (struct image_header *)fw_image;

	header->checksum = le_to_uint(data->checksum);

	header->bootloader_version = data->bootloader_version;

	header->firmware_size = le_to_uint(data->firmware_size);

	header->config_size = le_to_uint(data->config_size);

	memcpy(header->product_id, data->product_id, sizeof(data->product_id));
	header->product_id[sizeof(data->product_id)] = 0;

	memcpy(header->product_info, data->product_info,
			sizeof(data->product_info));

	header->contains_firmware_id = data->options_firmware_id;
	if (header->contains_firmware_id)
		header->firmware_id = le_to_uint(data->firmware_id);

	header->contains_bootloader = data->options_bootloader;
	if (header->contains_bootloader)
		header->bootloader_size = le_to_uint(data->bootloader_size);

	if ((header->bootloader_version == V5) && header->contains_bootloader) {
		header->contains_disp_config = true;
		header->disp_config_offset = le_to_uint(data->disp_config_addr);
		header->disp_config_size = le_to_uint(data->disp_config_size);
	} else {
		header->contains_disp_config = false;
	}

	if ((header->product_info[0] & 0x01) == 0x01) {
		fwu->do_lockdown = true;
		pr_info("%s: do_lockdown = %d\n", __func__, fwu->do_lockdown);
	}
	pr_info("%s: product_info = %d\n", __func__, header->product_info[0]);

	return;
}

static int fwu_read_f01_device_status(struct f01_device_status *status)
{
	int retval;
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->f01_data_base_addr,
			status->data,
			sizeof(status->data));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read F01 device status\n",
				__func__);
		return retval;
	}

	return 0;
}

static int fwu_read_f34_queries(void)
{
	int retval;
	unsigned char count;
	unsigned char buf[10];
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			fwu->f34_fd.query_base_addr + BOOTLOADER_ID_OFFSET,
			fwu->bootloader_id,
			sizeof(fwu->bootloader_id));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read bootloader ID\n",
				__func__);
		return retval;
	}

	if (fwu->bootloader_id[1] == '5') {
		fwu->bl_version = V5;
	} else if (fwu->bootloader_id[1] == '6') {
		fwu->bl_version = V6;
	} else {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Unrecognized bootloader version\n",
				__func__);
		return -EINVAL;
	}

	if (fwu->bl_version == V5) {
		fwu->properties_off = V5_PROPERTIES_OFFSET;
		fwu->blk_size_off = V5_BLOCK_SIZE_OFFSET;
		fwu->blk_count_off = V5_BLOCK_COUNT_OFFSET;
		fwu->blk_data_off = V5_BLOCK_DATA_OFFSET;
	} else if (fwu->bl_version == V6) {
		fwu->properties_off = V6_PROPERTIES_OFFSET;
		fwu->blk_size_off = V6_BLOCK_SIZE_OFFSET;
		fwu->blk_count_off = V6_BLOCK_COUNT_OFFSET;
		fwu->blk_data_off = V6_BLOCK_DATA_OFFSET;
	}

	retval = synaptics_rmi4_reg_read(rmi4_data,
			fwu->f34_fd.query_base_addr + fwu->properties_off,
			&fwu->flash_properties,
			sizeof(fwu->flash_properties));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read flash properties\n",
				__func__);
		return retval;
	}

	count = 4;

	if (fwu->flash_properties & HAS_PERM_CONFIG) {
		fwu->has_perm_config = 1;
		count += 2;
	}

	if (fwu->flash_properties & HAS_BL_CONFIG) {
		fwu->has_bl_config = 1;
		count += 2;
	}

	if (fwu->flash_properties & HAS_DISP_CONFIG) {
		fwu->has_disp_config = 1;
		count += 2;
	}

	retval = synaptics_rmi4_reg_read(rmi4_data,
			fwu->f34_fd.query_base_addr + fwu->blk_size_off,
			buf,
			2);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read block size info\n",
				__func__);
		return retval;
	}

	batohs(&fwu->block_size, &(buf[0]));

	if (fwu->bl_version == V5) {
		fwu->flash_cmd_off = fwu->blk_data_off + fwu->block_size;
		fwu->flash_status_off = fwu->flash_cmd_off;
	} else if (fwu->bl_version == V6) {
		fwu->flash_cmd_off = V6_FLASH_COMMAND_OFFSET;
		fwu->flash_status_off = V6_FLASH_STATUS_OFFSET;
	}

	retval = synaptics_rmi4_reg_read(rmi4_data,
			fwu->f34_fd.query_base_addr + fwu->blk_count_off,
			buf,
			count);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read block count info\n",
				__func__);
		return retval;
	}

	batohs(&fwu->fw_block_count, &(buf[0]));
	batohs(&fwu->config_block_count, &(buf[2]));

	count = 4;

	if (fwu->has_perm_config) {
		batohs(&fwu->perm_config_block_count, &(buf[count]));
		count += 2;
	}

	if (fwu->has_bl_config) {
		batohs(&fwu->bl_config_block_count, &(buf[count]));
		count += 2;
	}

	if (fwu->has_disp_config)
		batohs(&fwu->disp_config_block_count, &(buf[count]));

	return 0;
}

static int fwu_read_f34_flash_status(void)
{
	int retval;
	unsigned char status;
	unsigned char command;
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			fwu->f34_fd.data_base_addr + fwu->flash_status_off,
			&status,
			sizeof(status));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read flash status\n",
				__func__);
		return retval;
	}

	fwu->program_enabled = status >> 7;

	if (fwu->bl_version == V5)
		fwu->flash_status = (status >> 4) & MASK_3BIT;
	else if (fwu->bl_version == V6)
		fwu->flash_status = status & MASK_3BIT;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			fwu->f34_fd.data_base_addr + fwu->flash_cmd_off,
			&command,
			sizeof(command));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read flash command\n",
				__func__);
		return retval;
	}

	fwu->command = command & MASK_4BIT;

	return 0;
}

static int fwu_write_f34_command(unsigned char cmd)
{
	int retval;
	unsigned char command = cmd & MASK_4BIT;
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;

	fwu->command = cmd;

	retval = synaptics_rmi4_reg_write(rmi4_data,
			fwu->f34_fd.data_base_addr + fwu->flash_cmd_off,
			&command,
			sizeof(command));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to write command 0x%02x\n",
				__func__, command);
		return retval;
	}

	return 0;
}

static int fwu_wait_for_idle(int timeout_ms)
{
	int count = 0;
	int timeout_count = ((timeout_ms * 1000) / MAX_SLEEP_TIME_US) + 1;
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;

	do {
		usleep_range(MIN_SLEEP_TIME_US, MAX_SLEEP_TIME_US);

		count++;
		if (count == timeout_count)
			fwu_read_f34_flash_status();

		if ((fwu->command == 0x00) && (fwu->flash_status == 0x00))
			return 0;
	} while (count < timeout_count);

	dev_err(rmi4_data->pdev->dev.parent,
			"%s: Timed out waiting for idle status, command: %X, status: %X\n",
			__func__, fwu->command, fwu->flash_status);

	return -ETIMEDOUT;
}

static enum flash_area fwu_go_nogo(struct image_header_data *header)
{
	int retval;
	enum flash_area flash_area = NONE;
	unsigned char index = 0;
	unsigned char config_id[4];
	unsigned int device_config_id;
	unsigned int image_config_id;
	unsigned int device_fw_id;
	unsigned long image_fw_id;
	char *strptr;
	char *firmware_id;
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;

	if (fwu->force_update) {
		flash_area = UI_FIRMWARE;
		goto exit;
	}

	
	if (fwu->in_flash_prog_mode) {
		flash_area = UI_FIRMWARE;
		goto exit;
	}

	
	device_fw_id = rmi4_data->firmware_id;
	dev_info(rmi4_data->pdev->dev.parent,
			"%s: Device firmware ID = %d\n",
			__func__, device_fw_id);

	
	if (header->contains_firmware_id) {
		image_fw_id = header->firmware_id;
	} else {
		strptr = strstr(fwu->image_name, "PR");
		if (!strptr) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: No valid PR number (PRxxxxxxx) "
					"found in image file name (%s)\n",
					__func__, fwu->image_name);
			flash_area = NONE;
			goto exit;
		}

		strptr += 2;
		firmware_id = kzalloc(MAX_FIRMWARE_ID_LEN, GFP_KERNEL);
		while (strptr[index] >= '0' && strptr[index] <= '9') {
			firmware_id[index] = strptr[index];
			index++;
		}

		retval = sstrtoul(firmware_id, 10, &image_fw_id);
		kfree(firmware_id);
		if (retval) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to obtain image firmware ID\n",
					__func__);
			flash_area = NONE;
			goto exit;
		}
	}
	dev_info(rmi4_data->pdev->dev.parent,
			"%s: Image firmware ID = %d\n",
			__func__, (unsigned int)image_fw_id);

	if (image_fw_id != device_fw_id) {
		flash_area = UI_FIRMWARE;
		goto exit;
	} else if (image_fw_id == device_fw_id) {
		dev_info(rmi4_data->pdev->dev.parent,
				"%s: Image firmware ID is the same as device firmware ID\n",
				__func__);
		flash_area = NONE;
		goto exit;
	}

	
	retval = synaptics_rmi4_reg_read(rmi4_data,
				fwu->f34_fd.ctrl_base_addr,
				config_id,
				sizeof(config_id));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read device config ID\n",
				__func__);
		flash_area = NONE;
		goto exit;
	}
	device_config_id = be_to_uint(config_id);
	dev_info(rmi4_data->pdev->dev.parent,
			"%s: Device config ID = 0x%02x 0x%02x 0x%02x 0x%02x\n",
			__func__,
			config_id[0],
			config_id[1],
			config_id[2],
			config_id[3]);

	
	image_config_id = be_to_uint(fwu->config_data);
	dev_info(rmi4_data->pdev->dev.parent,
			"%s: Image config ID = 0x%02x 0x%02x 0x%02x 0x%02x\n",
			__func__,
			fwu->config_data[0],
			fwu->config_data[1],
			fwu->config_data[2],
			fwu->config_data[3]);


	flash_area = NONE;

exit:
	if (flash_area == NONE) {
		dev_info(rmi4_data->pdev->dev.parent,
				"%s: No need to do reflash\n",
				__func__);
	} else {
		dev_info(rmi4_data->pdev->dev.parent,
				"%s: Updating %s\n",
				__func__,
				flash_area == UI_FIRMWARE ?
				"UI firmware" :
				"config only");
	}

	return flash_area;
}

static int fwu_scan_pdt(void)
{
	int retval;
	unsigned char ii;
	unsigned char intr_count = 0;
	unsigned char intr_off;
	unsigned char intr_src;
	unsigned short addr;
	bool f01found = false;
	bool f34found = false;
	struct synaptics_rmi4_fn_desc rmi_fd;
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;

	for (addr = PDT_START; addr > PDT_END; addr -= PDT_ENTRY_SIZE) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				addr,
				(unsigned char *)&rmi_fd,
				sizeof(rmi_fd));
		if (retval < 0)
			return retval;

		if (rmi_fd.fn_number) {
			dev_info(rmi4_data->pdev->dev.parent,
					"%s: Found F%02x\n",
					__func__, rmi_fd.fn_number);
			switch (rmi_fd.fn_number) {
			case SYNAPTICS_RMI4_F01:
				f01found = true;

				rmi4_data->f01_query_base_addr =
						rmi_fd.query_base_addr;
				rmi4_data->f01_ctrl_base_addr =
						rmi_fd.ctrl_base_addr;
				rmi4_data->f01_data_base_addr =
						rmi_fd.data_base_addr;
				rmi4_data->f01_cmd_base_addr =
						rmi_fd.cmd_base_addr;
				break;
			case SYNAPTICS_RMI4_F34:
				f34found = true;
				fwu->f34_fd.query_base_addr =
						rmi_fd.query_base_addr;
				fwu->f34_fd.ctrl_base_addr =
						rmi_fd.ctrl_base_addr;
				fwu->f34_fd.data_base_addr =
						rmi_fd.data_base_addr;

				fwu->intr_mask = 0;
				intr_src = rmi_fd.intr_src_count;
				intr_off = intr_count % 8;
				for (ii = intr_off;
						ii < ((intr_src & MASK_3BIT) +
						intr_off);
						ii++) {
					fwu->intr_mask |= 1 << ii;
				}
				break;
			}
		} else {
			break;
		}

		intr_count += (rmi_fd.intr_src_count & MASK_3BIT);
	}

	if (!f01found || !f34found) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to find both F01 and F34\n",
				__func__);
		return -EINVAL;
	}

	return 0;
}

static int fwu_write_blocks(unsigned char *block_ptr, unsigned short block_cnt,
		unsigned char command)
{
	int retval;
	unsigned char block_offset[] = {0, 0};
	unsigned short block_num;
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;

	block_offset[1] |= (fwu->config_area << 5);

	retval = synaptics_rmi4_reg_write(rmi4_data,
			fwu->f34_fd.data_base_addr + BLOCK_NUMBER_OFFSET,
			block_offset,
			sizeof(block_offset));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to write to block number registers\n",
				__func__);
		return retval;
	}

	for (block_num = 0; block_num < block_cnt; block_num++) {
		retval = synaptics_rmi4_reg_write(rmi4_data,
				fwu->f34_fd.data_base_addr + fwu->blk_data_off,
				block_ptr,
				fwu->block_size);
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to write block data (block %d)\n",
					__func__, block_num);
			return retval;
		}

		retval = fwu_write_f34_command(command);
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to write command for block %d\n",
					__func__, block_num);
			return retval;
		}

		retval = fwu_wait_for_idle(WRITE_WAIT_MS);
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to wait for idle status (block %d)\n",
					__func__, block_num);
			return retval;
		}

		block_ptr += fwu->block_size;
	}

	return 0;
}

static int fwu_write_firmware(void)
{
	return fwu_write_blocks((unsigned char *)fwu->firmware_data,
		fwu->fw_block_count, CMD_WRITE_FW_BLOCK);
}

static int fwu_write_configuration(void)
{
	return fwu_write_blocks((unsigned char *)fwu->config_data,
		fwu->config_block_count, CMD_WRITE_CONFIG_BLOCK);
}

static int fwu_write_disp_configuration(void)
{
	fwu->config_area = DISP_CONFIG_AREA;
	fwu->config_data = fwu->disp_config_data;
	fwu->config_block_count = fwu->disp_config_block_count;

	return fwu_do_write_config();
}

static int fwu_write_lockdown(void)
{
	return fwu_write_blocks((unsigned char *)fwu->lockdown_data,
		fwu->lockdown_block_count, CMD_WRITE_LOCKDOWN_BLOCK);
}

static int fwu_get_tw_vendor(void)
{
	int retval;
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;
	uint16_t tw_pin_mask = rmi4_data->hw_if->board_data->tw_pin_mask;
	uint8_t data[6];

	memcpy(&data, &tw_pin_mask, sizeof(tw_pin_mask));
	data[2] = data[0];
	data[3] = data[1];
	retval = synaptics_rmi4_reg_write(rmi4_data,
			fwu->f34_fd.data_base_addr + 1,
			data,
			4);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to write tw vendor pin\n",
				__func__);
		return -EINVAL;
	}
	retval = fwu_write_f34_command(CMD_READ_TW_PIN);
	if (retval < 0)
		return retval;

	retval = fwu_wait_for_idle(ENABLE_WAIT_MS);
	if (retval < 0)
		return retval;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			fwu->f34_fd.data_base_addr + 1,
			data,
			sizeof(data));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read tw vendor pin\n",
				__func__);
		return -EINVAL;
	}
	rmi4_data->tw_vendor_pin = (data[5] << 8) | data[4];
	dev_info(rmi4_data->pdev->dev.parent, " %s: tw_vendor_pin = %x\n", __func__,
						rmi4_data->tw_vendor_pin);

	return 0;
}

static int crc_comparison(uint32_t config_crc)
{
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;
	uint8_t data[17];
	int retval;
	uint32_t flash_crc;

	data[0] = fwu->config_block_count-1;
	data[1] = 0x00;
	retval = synaptics_rmi4_reg_write(rmi4_data,
			fwu->f34_fd.data_base_addr,
			data,
			2);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to write crc addr\n",
				__func__);
		return retval;
	}

	retval = fwu_write_f34_command(CMD_READ_CONFIG_BLOCK);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to write read config command\n",
				__func__);
		return retval;
	}

	retval = fwu_wait_for_idle(WRITE_WAIT_MS);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to wait for idle status\n",
				__func__);
		return retval;
	}

	retval = synaptics_rmi4_reg_read(rmi4_data,
			fwu->f34_fd.data_base_addr + 1,
			data,
			sizeof(data));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read crc data\n",
				__func__);
		return retval;
	}

	memcpy(&flash_crc, &data[12], 4);
	dev_info(rmi4_data->pdev->dev.parent, " %s: config_crc = %X, flash_crc = %X\n",
					__func__, config_crc, flash_crc);

	if (flash_crc == config_crc)
		return 0;
	else
		return 1;
}

static int fwu_write_bootloader_id(void)
{
	int retval;
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;

	retval = synaptics_rmi4_reg_write(rmi4_data,
			fwu->f34_fd.data_base_addr + fwu->blk_data_off,
			fwu->bootloader_id,
			sizeof(fwu->bootloader_id));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to write bootloader ID\n",
				__func__);
		return retval;
	}

	return 0;
}

static int fwu_enter_flash_prog(void)
{
	int retval;
	struct f01_device_status f01_device_status;
	struct f01_device_control f01_device_control;
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;

	retval = fwu_write_bootloader_id();
	if (retval < 0)
		return retval;

	retval = fwu_write_f34_command(CMD_ENABLE_FLASH_PROG);
	if (retval < 0)
		return retval;

	retval = fwu_wait_for_idle(ENABLE_WAIT_MS);
	if (retval < 0)
		return retval;

	if (!fwu->program_enabled) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Program enabled bit not set\n",
				__func__);
		return -EINVAL;
	}

	if (rmi4_data->hw_if->bl_hw_init) {
		retval = rmi4_data->hw_if->bl_hw_init(rmi4_data);
		if (retval < 0)
			return retval;
	}

	retval = fwu_scan_pdt();
	if (retval < 0)
		return retval;

	retval = fwu_read_f01_device_status(&f01_device_status);
	if (retval < 0)
		return retval;

	if (!f01_device_status.flash_prog) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Not in flash prog mode\n",
				__func__);
		return -EINVAL;
	}

	retval = fwu_read_f34_queries();
	if (retval < 0)
		return retval;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			f01_device_control.data,
			sizeof(f01_device_control.data));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read F01 device control\n",
				__func__);
		return retval;
	}

	f01_device_control.nosleep = true;
	f01_device_control.sleep_mode = SLEEP_MODE_NORMAL;

	retval = synaptics_rmi4_reg_write(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			f01_device_control.data,
			sizeof(f01_device_control.data));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to write F01 device control\n",
				__func__);
		return retval;
	}

	msleep(ENTER_FLASH_PROG_WAIT_MS);

	return retval;
}

static int fwu_do_reflash(void)
{
	int retval;
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;
	dev_info(rmi4_data->pdev->dev.parent, " %s\n", __func__);

	retval = fwu_write_bootloader_id();
	if (retval < 0)
		return retval;

	dev_info(rmi4_data->pdev->dev.parent,
			"%s: Bootloader ID written\n",
			__func__);

	retval = fwu_write_f34_command(CMD_ERASE_ALL);
	if (retval < 0)
		return retval;

	dev_info(rmi4_data->pdev->dev.parent,
			"%s: Erase all command written\n",
			__func__);

	retval = fwu_wait_for_idle(ERASE_WAIT_MS);
	if (retval < 0)
		return retval;

	dev_info(rmi4_data->pdev->dev.parent,
			"%s: Idle status detected\n",
			__func__);

	if (fwu->firmware_data) {
		retval = fwu_write_firmware();
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to write firmware\n",
					__func__);
			return retval;
		}
		pr_notice("%s: Firmware programmed\n", __func__);
	}

	if (fwu->config_data) {
		retval = fwu_write_configuration();
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to write configuration\n",
					__func__);
			return retval;
		}
		pr_notice("%s: Configuration programmed\n", __func__);
	}

	if (fwu->disp_config_data) {
		retval = fwu_write_disp_configuration();
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to write disp configuration\n",
					__func__);
			return retval;
		}
		pr_notice("%s: Display configuration programmed\n", __func__);
	}

	dev_info(rmi4_data->pdev->dev.parent, " %s\n", __func__);
	return retval;
}

static int fwu_do_write_config(void)
{
	int retval;
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;
	dev_info(rmi4_data->pdev->dev.parent, " %s\n", __func__);

	if (fwu->config_area == PERM_CONFIG_AREA) {
		fwu->config_block_count = fwu->perm_config_block_count;
		goto write_config;
	}

	retval = fwu_write_bootloader_id();
	if (retval < 0)
		return retval;

	dev_dbg(rmi4_data->pdev->dev.parent,
			"%s: Bootloader ID written\n",
			__func__);

	switch (fwu->config_area) {
	case UI_CONFIG_AREA:
		retval = fwu_write_f34_command(CMD_ERASE_CONFIG);
		break;
	case BL_CONFIG_AREA:
		retval = fwu_write_f34_command(CMD_ERASE_BL_CONFIG);
		fwu->config_block_count = fwu->bl_config_block_count;
		break;
	case DISP_CONFIG_AREA:
		retval = fwu_write_f34_command(CMD_ERASE_DISP_CONFIG);
		fwu->config_block_count = fwu->disp_config_block_count;
		break;
	}
	if (retval < 0)
		return retval;

	dev_dbg(rmi4_data->pdev->dev.parent,
			"%s: Erase command written\n",
			__func__);

	retval = fwu_wait_for_idle(ERASE_WAIT_MS);
	if (retval < 0)
		return retval;

	dev_dbg(rmi4_data->pdev->dev.parent,
			"%s: Idle status detected\n",
			__func__);

write_config:
	retval = fwu_write_configuration();
	if (retval < 0)
		return retval;

	pr_notice("%s: Config written\n", __func__);

	return retval;
}

static int fwu_start_write_config(void)
{
	int retval;
	unsigned short block_count;
	struct image_header_data header;
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;

	switch (fwu->config_area) {
	case UI_CONFIG_AREA:
		block_count = fwu->config_block_count;
		break;
	case PERM_CONFIG_AREA:
		if (!fwu->has_perm_config)
			return -EINVAL;
		block_count = fwu->perm_config_block_count;
		break;
	case BL_CONFIG_AREA:
		if (!fwu->has_bl_config)
			return -EINVAL;
		block_count = fwu->bl_config_block_count;
		break;
	case DISP_CONFIG_AREA:
		if (!fwu->has_disp_config)
			return -EINVAL;
		block_count = fwu->disp_config_block_count;
		break;
	default:
		return -EINVAL;
	}

	if (fwu->ext_data_source)
		fwu->config_data = fwu->ext_data_source;
	else
		return -EINVAL;

	fwu->config_size = fwu->block_size * block_count;

	
	if ((fwu->config_area == UI_CONFIG_AREA) &&
			(fwu->config_size != fwu->image_size)) {
		parse_header(&header, fwu->ext_data_source);

		if (header.config_size) {
			fwu->config_data = fwu->ext_data_source +
					IMAGE_AREA_OFFSET +
					header.firmware_size;
			if (header.contains_bootloader)
				fwu->config_data += header.bootloader_size;
		} else {
			return -EINVAL;
		}
	}

	pr_notice("%s: Start of write config process\n", __func__);

	retval = fwu_enter_flash_prog();
	if (retval < 0)
		goto exit;

	retval = fwu_do_write_config();
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to write config\n",
				__func__);
	}

exit:
	rmi4_data->reset_device(rmi4_data);

	pr_notice("%s: End of write config process\n", __func__);

	return retval;
}

static int fwu_do_read_config(void)
{
	int retval;
	unsigned char block_offset[] = {0, 0};
	unsigned short block_num;
	unsigned short block_count;
	unsigned short index = 0;
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;

	retval = fwu_enter_flash_prog();
	if (retval < 0)
		goto exit;

	dev_dbg(rmi4_data->pdev->dev.parent,
			"%s: Entered flash prog mode\n",
			__func__);

	switch (fwu->config_area) {
	case UI_CONFIG_AREA:
		block_count = fwu->config_block_count;
		break;
	case PERM_CONFIG_AREA:
		if (!fwu->has_perm_config) {
			retval = -EINVAL;
			goto exit;
		}
		block_count = fwu->perm_config_block_count;
		break;
	case BL_CONFIG_AREA:
		if (!fwu->has_bl_config) {
			retval = -EINVAL;
			goto exit;
		}
		block_count = fwu->bl_config_block_count;
		break;
	case DISP_CONFIG_AREA:
		if (!fwu->has_disp_config) {
			retval = -EINVAL;
			goto exit;
		}
		block_count = fwu->disp_config_block_count;
		break;
	default:
		retval = -EINVAL;
		goto exit;
	}

	fwu->config_size = fwu->block_size * block_count;

	kfree(fwu->read_config_buf);
	fwu->read_config_buf = kzalloc(fwu->config_size, GFP_KERNEL);

	block_offset[1] |= (fwu->config_area << 5);

	retval = synaptics_rmi4_reg_write(rmi4_data,
			fwu->f34_fd.data_base_addr + BLOCK_NUMBER_OFFSET,
			block_offset,
			sizeof(block_offset));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to write to block number registers\n",
				__func__);
		goto exit;
	}

	for (block_num = 0; block_num < block_count; block_num++) {
		retval = fwu_write_f34_command(CMD_READ_CONFIG_BLOCK);
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to write read config command\n",
					__func__);
			goto exit;
		}

		retval = fwu_wait_for_idle(WRITE_WAIT_MS);
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to wait for idle status\n",
					__func__);
			goto exit;
		}

		retval = synaptics_rmi4_reg_read(rmi4_data,
				fwu->f34_fd.data_base_addr + fwu->blk_data_off,
				&fwu->read_config_buf[index],
				fwu->block_size);
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to read block data (block %d)\n",
					__func__, block_num);
			goto exit;
		}

		index += fwu->block_size;
	}

exit:
	rmi4_data->reset_device(rmi4_data);

	return retval;
}

static int fwu_do_lockdown(void)
{
	int retval;
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;

	retval = fwu_enter_flash_prog();
	if (retval < 0)
		return retval;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			fwu->f34_fd.query_base_addr + fwu->properties_off,
			&fwu->flash_properties,
			sizeof(fwu->flash_properties));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read flash properties\n",
				__func__);
		return retval;
	}

	if ((fwu->flash_properties & UNLOCKED) == 0) {
		dev_info(rmi4_data->pdev->dev.parent,
				"%s: Device already locked down\n",
				__func__);
		return retval;
	}

	retval = fwu_write_lockdown();
	if (retval < 0)
		return retval;

	pr_notice("%s: Lockdown programmed\n", __func__);

	return retval;
}

static int fwu_start_reflash(void)
{
	int retval = 0;
	enum flash_area flash_area;
	struct image_header_data header;
	struct f01_device_status f01_device_status;
	const unsigned char *fw_image;
	const struct firmware *fw_entry = NULL;
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;
	dev_info(rmi4_data->pdev->dev.parent, " %s\n", __func__);

	if (rmi4_data->sensor_sleep) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Sensor sleeping\n",
				__func__);
		return -ENODEV;
	}

	rmi4_data->stay_awake = true;

	pr_notice("%s: Start of reflash process\n", __func__);

	if (fwu->ext_data_source) {
		fw_image = fwu->ext_data_source;
	} else {
		strncpy(fwu->image_name, FW_IMAGE_NAME, MAX_IMAGE_NAME_LEN);
		dev_dbg(rmi4_data->pdev->dev.parent,
				"%s: Requesting firmware image %s\n",
				__func__, fwu->image_name);

		retval = request_firmware(&fw_entry, fwu->image_name,
				rmi4_data->pdev->dev.parent);
		if (retval != 0) {
			dev_info(rmi4_data->pdev->dev.parent,
					"%s: Firmware image %s not available\n",
					__func__, fwu->image_name);
			retval = -EINVAL;
			goto exit;
		}

		dev_dbg(rmi4_data->pdev->dev.parent,
				"%s: Firmware image size = %zu\n",
				__func__, fw_entry->size);

		fw_image = fw_entry->data;
	}

	parse_header(&header, fw_image);

	if (fwu->bl_version != header.bootloader_version) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Bootloader version mismatch\n",
				__func__);
		retval = -EINVAL;
		goto exit;
	}

	retval = fwu_read_f01_device_status(&f01_device_status);
	if (retval < 0)
		goto exit;

	if (f01_device_status.flash_prog) {
		dev_info(rmi4_data->pdev->dev.parent,
				"%s: In flash prog mode\n",
				__func__);
		fwu->in_flash_prog_mode = true;
	} else {
		fwu->in_flash_prog_mode = false;
	}

	if (fwu->do_lockdown) {
		switch (fwu->bl_version) {
		case V5:
		case V6:
			fwu->lockdown_data = fw_image + LOCKDOWN_OFFSET;
			fwu->lockdown_block_count = LOCKDOWN_BLOCK_COUNT;
			retval = fwu_do_lockdown();
			if (retval < 0) {
				dev_err(rmi4_data->pdev->dev.parent,
						"%s: Failed to do lockdown\n",
						__func__);
			}
		default:
			break;
		}
	}

	if (header.firmware_size)
		fwu->firmware_data = fw_image + IMAGE_AREA_OFFSET;
	else
		fwu->firmware_data = NULL;

	if (header.config_size)
		fwu->config_data = fw_image + IMAGE_AREA_OFFSET +
				header.firmware_size;
	else
		fwu->config_data = NULL;

	if (header.contains_bootloader) {
		if (header.firmware_size)
			fwu->firmware_data += header.bootloader_size;
		if (header.config_size)
			fwu->config_data += header.bootloader_size;
	}

	if (header.contains_disp_config)
		fwu->disp_config_data = fw_image + header.disp_config_offset;
	else
		fwu->disp_config_data = NULL;

	flash_area = fwu_go_nogo(&header);

	if (flash_area != NONE) {
		retval = fwu_enter_flash_prog();
		if (retval < 0)
			goto exit;
	}

	switch (flash_area) {
	case UI_FIRMWARE:
		retval = fwu_do_reflash();
		break;
	case CONFIG_AREA:
		retval = fwu_do_write_config();
		break;
	case NONE:
		retval = 1;
	default:
		goto exit;
	}

	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to do reflash\n",
				__func__);
	}

exit:
	rmi4_data->reset_device(rmi4_data);

	if (fw_entry)
		release_firmware(fw_entry);

	pr_notice("%s: End of reflash process\n", __func__);

	rmi4_data->stay_awake = false;

	return retval;
}

int synaptics_fw_updater(unsigned char *fw_data)
{
	int retval;
	pr_info("%s\n", __func__);

	if (!fwu)
		return -ENODEV;

	if (!fwu->initialized)
		return -ENODEV;

	fwu->ext_data_source = fw_data;
	fwu->config_area = UI_CONFIG_AREA;

	retval = fwu_start_reflash();

	return retval;
}
EXPORT_SYMBOL(synaptics_fw_updater);

int synaptics_config_updater(struct synaptics_dsx_board_data *bdata)
{
	int retval, i;
	struct synaptics_rmi4_config *cfg_table = bdata->config_table;
	struct synaptics_rmi4_data *rmi4_data;
	unsigned int device_fw_id;
	uint8_t eng_id = bdata->eng_id;
	uint16_t tw_pin_mask = bdata->tw_pin_mask;
	int config_num = bdata->config_num;
	uint32_t crc_checksum;
	unsigned char config_id[4];
	unsigned int device_config_id;
	unsigned int image_config_id;
	uint8_t *config_data;
	int config_size = fwu->config_block_count * fwu->block_size;
	pr_info("%s\n", __func__);

	if (!fwu)
		return -ENODEV;

	if (!fwu->initialized)
		return -ENODEV;

	rmi4_data = fwu->rmi4_data;
	device_fw_id = rmi4_data->firmware_id;

	rmi4_data->stay_awake = true;
	
	retval = synaptics_rmi4_reg_read(rmi4_data,
				fwu->f34_fd.ctrl_base_addr,
				config_id,
				sizeof(config_id));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read device config ID\n",
				__func__);
		goto exit;
	}
	device_config_id = be_to_uint(config_id);
	dev_info(rmi4_data->pdev->dev.parent,
			"%s: Device config ID = 0x%02x 0x%02x 0x%02x 0x%02x\n",
			__func__,
			config_id[0],
			config_id[1],
			config_id[2],
			config_id[3]);

	fwu->config_area = UI_CONFIG_AREA;

	pr_notice("%s: Start of write config process\n", __func__);

	retval = fwu_enter_flash_prog();
	if (retval < 0)
		goto exit;

	if (tw_pin_mask) {
		retval = fwu_get_tw_vendor();
		if (retval < 0)
			goto exit;
	}

	i = 0;
	while (cfg_table[i].pr_number > device_fw_id) {
		i++;
		if (i == config_num) {
			dev_info(rmi4_data->pdev->dev.parent, " %s: no config data\n", __func__);
			goto exit;
		}
	}

	while (cfg_table[i].eng_id != eng_id) {
		i++;
		if (i == config_num) {
			dev_info(rmi4_data->pdev->dev.parent, " %s: eng_id does not match\n", __func__);
			goto exit;
		}
	}

	if (tw_pin_mask) {
		while ((cfg_table[i].sensor_id > 0)
			&& (cfg_table[i].sensor_id != (rmi4_data->tw_vendor_pin | SENSOR_ID_CHECKING_EN))) {
			i++;
			if (i == config_num) {
				dev_info(rmi4_data->pdev->dev.parent, " %s: no config data\n", __func__);
				goto exit;
			}
		}
	}

	if (cfg_table[i].cover_setting_size != 0) {
		memcpy(rmi4_data->cover_setting, cfg_table[i].cover_setting, sizeof(rmi4_data->cover_setting));
		memcpy(rmi4_data->uncover_setting, cfg_table[i].uncover_setting, sizeof(rmi4_data->uncover_setting));
		memcpy(rmi4_data->glove_mode_setting, cfg_table[i].glove_mode_setting, sizeof(rmi4_data->glove_mode_setting));
		rmi4_data->cover_setting_size = cfg_table[i].cover_setting_size;
	}

	config_data = cfg_table[i].config;
	
	image_config_id = be_to_uint(config_data);
	dev_info(rmi4_data->pdev->dev.parent,
			"%s: Image config ID = 0x%02x 0x%02x 0x%02x 0x%02x\n",
			__func__,
			config_data[0],
			config_data[1],
			config_data[2],
			config_data[3]);

	crc_checksum = syn_crc((uint16_t *)config_data, (config_size)/2-2);
	if (crc_checksum == 0xFFFFFFFF) {
		dev_err(rmi4_data->pdev->dev.parent,
			"%s: crc_checksum Error", __func__);
		return -1;
	}
	memcpy(&config_data[(config_size) - 4], &crc_checksum, 4);
	dev_info(rmi4_data->pdev->dev.parent, " %s: crc_cksum = %X\n",
					__func__, crc_checksum);
	if (image_config_id == device_config_id) {
		retval = crc_comparison(crc_checksum);
		if (retval < 0) {
			dev_info(rmi4_data->pdev->dev.parent, " %s: CRC comparison fail!\n",
					__func__);
			goto exit;
		} else if (retval == 0) {
			dev_info(rmi4_data->pdev->dev.parent, " %s: No need to update\n",
					__func__);
			goto exit;
		}
	}
	fwu->config_data = config_data;

	retval = fwu_do_write_config();
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to write config\n",
				__func__);
	}

exit:
	rmi4_data->reset_device(rmi4_data);

	rmi4_data->stay_awake = false;
	pr_notice("%s: End of write config process\n", __func__);

	dev_info(rmi4_data->pdev->dev.parent, " %s end\n", __func__);
	return retval;
}
EXPORT_SYMBOL(synaptics_config_updater);

#ifdef DO_STARTUP_FW_UPDATE
static void fwu_startup_fw_update_work(struct work_struct *work)
{
	struct synaptics_dsx_board_data *bdata = fwu->rmi4_data->hw_if->board_data;
	wake_lock(&fwu->fwu_wake_lock);

	if (bdata->update_feature & SYNAPTICS_RMI4_UPDATE_IMAGE)
		synaptics_fw_updater(NULL);

	if (bdata->update_feature & SYNAPTICS_RMI4_UPDATE_CONFIG)
		synaptics_config_updater(bdata);

	wake_unlock(&fwu->fwu_wake_lock);

	return;
}
#endif

static ssize_t fwu_sysfs_show_image(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count)
{
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;

	if (count < fwu->config_size) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Not enough space (%zu bytes) in buffer\n",
				__func__, count);
		return -EINVAL;
	}

	memcpy(buf, fwu->read_config_buf, fwu->config_size);

	return fwu->config_size;
}

static ssize_t fwu_sysfs_store_image(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count)
{
	memcpy((void *)(&fwu->ext_data_source[fwu->data_pos]),
			(const void *)buf,
			count);

	fwu->data_pos += count;

	return count;
}

static ssize_t fwu_sysfs_do_reflash_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned int input;
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;

	fwu->do_lockdown = DO_LOCKDOWN;

	if (sscanf(buf, "%u", &input) != 1) {
		retval = -EINVAL;
		goto exit;
	}

	if (input & LOCKDOWN) {
		fwu->do_lockdown = true;
		input &= ~LOCKDOWN;
	}

	if ((input != NORMAL) && (input != FORCE)) {
		retval = -EINVAL;
		goto exit;
	}

	if (input == FORCE)
		fwu->force_update = true;

	retval = synaptics_fw_updater(fwu->ext_data_source);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to do reflash\n",
				__func__);
		goto exit;
	}

	retval = count;

exit:
	kfree(fwu->ext_data_source);
	fwu->ext_data_source = NULL;
	fwu->force_update = FORCE_UPDATE;
	fwu->do_lockdown = DO_LOCKDOWN;
	return retval;
}

static ssize_t fwu_sysfs_write_config_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned int input;
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;

	if (sscanf(buf, "%u", &input) != 1) {
		retval = -EINVAL;
		goto exit;
	}

	if (input != 1) {
		retval = -EINVAL;
		goto exit;
	}

	retval = fwu_start_write_config();
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to write config\n",
				__func__);
		goto exit;
	}

	retval = count;

exit:
	kfree(fwu->ext_data_source);
	fwu->ext_data_source = NULL;
	return retval;
}

static ssize_t fwu_sysfs_read_config_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned int input;
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	if (input != 1)
		return -EINVAL;

	retval = fwu_do_read_config();
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read config\n",
				__func__);
		return retval;
	}

	return count;
}

static ssize_t fwu_sysfs_config_area_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned long config_area;

	retval = sstrtoul(buf, 10, &config_area);
	if (retval)
		return retval;

	fwu->config_area = config_area;

	return count;
}

static ssize_t fwu_sysfs_image_name_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	memcpy(fwu->image_name, buf, count);

	return count;
}

static ssize_t fwu_sysfs_image_size_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned long size;
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;

	retval = sstrtoul(buf, 10, &size);
	if (retval)
		return retval;

	fwu->image_size = size;
	fwu->data_pos = 0;

	kfree(fwu->ext_data_source);
	fwu->ext_data_source = kzalloc(fwu->image_size, GFP_KERNEL);
	if (!fwu->ext_data_source) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to alloc mem for image data\n",
				__func__);
		return -ENOMEM;
	}

	return count;
}

static ssize_t fwu_sysfs_block_size_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", fwu->block_size);
}

static ssize_t fwu_sysfs_firmware_block_count_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", fwu->fw_block_count);
}

static ssize_t fwu_sysfs_configuration_block_count_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", fwu->config_block_count);
}

static ssize_t fwu_sysfs_perm_config_block_count_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", fwu->perm_config_block_count);
}

static ssize_t fwu_sysfs_bl_config_block_count_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", fwu->bl_config_block_count);
}

static ssize_t fwu_sysfs_disp_config_block_count_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", fwu->disp_config_block_count);
}

static void synaptics_rmi4_fwu_attn(struct synaptics_rmi4_data *rmi4_data,
		unsigned char intr_mask)
{
	if (!fwu)
		return;

	if (fwu->intr_mask & intr_mask)
		fwu_read_f34_flash_status();

	return;
}

#ifdef CONFIG_TOUCHSCREEN_TOUCH_FW_UPDATE
static void synaptics_fwu_progress(int percentage)
{
	touch_fw_update_progress(percentage);
}

static int check_img_version(const struct firmware *fw, int *tagLen, unsigned int version)
{
	char tag[40], fw_ver[40];
	int i = 0;

	if (fw->data[0] == 'T' && fw->data[1] == 'P') {
		while ((tag[i] = fw->data[i]) != '\n')
			i++;
		tag[i] = '\0';
		*tagLen = i+1;
		pr_info("%s: tag=%s\n", __func__, tag);
		snprintf(fw_ver, 40, "%d", version);
		pr_info("%s: fw_ver=%s\n", __func__, fw_ver);
		if (version==0) {
			pr_info("%s: Need Update\n", __func__);
			return 1;
		}
		if (strstr(tag, fw_ver) != NULL) {
			pr_info("%s: Update Bypass\n", __func__);
			return 0; 
		}
	}

	pr_info("%s: Need Update\n", __func__);
	return 1;
}

int synaptics_touch_fw_update(struct firmware *fw)
{
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;
	unsigned char *fw_source;
	int tagLen = 0, ret = 0;

	synaptics_fwu_progress(0);
	if (check_img_version(fw, &tagLen, rmi4_data->firmware_id)) {
		fw_source = kzalloc(fw->size-tagLen, GFP_KERNEL);
		memcpy(fw_source, fw->data+tagLen, fw->size-tagLen);
		fwu->do_lockdown = DO_LOCKDOWN;
		synaptics_fwu_progress(50);
		ret = synaptics_fw_updater(fw_source);
		kfree(fw_source);
		fwu->do_lockdown = DO_LOCKDOWN;
		if (ret < 0) {
			pr_err("firmware download failed\n");
			return -1;
		} else if (ret == 1) {
			pr_info("firmware download bypass\n");
		}
	} else {
		ret = 1;
	}

	synaptics_fwu_progress(100);
	return ret;
}

int register_synaptics_fw_update(void)
{
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;
	synaptics_tp_notifier.fwupdate = synaptics_touch_fw_update;
	synaptics_tp_notifier.flash_timeout = FW_FLASH_TIMEOUT;
	snprintf(synaptics_tp_notifier.fw_vendor, sizeof(synaptics_tp_notifier.fw_vendor), "%s", TOUCH_VENDOR);
	snprintf(synaptics_tp_notifier.fw_ver, sizeof(synaptics_tp_notifier.fw_ver), "%d", rmi4_data->firmware_id);
	return register_fw_update(&synaptics_tp_notifier);
}
#endif

static int synaptics_rmi4_fwu_init(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char attr_count;
	struct pdt_properties pdt_props;
	dev_info(rmi4_data->pdev->dev.parent, " %s\n", __func__);

	fwu = kzalloc(sizeof(*fwu), GFP_KERNEL);
	if (!fwu) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to alloc mem for fwu\n",
				__func__);
		retval = -ENOMEM;
		goto exit;
	}

	fwu->image_name = kzalloc(MAX_IMAGE_NAME_LEN, GFP_KERNEL);
	if (!fwu->image_name) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to alloc mem for image name\n",
				__func__);
		retval = -ENOMEM;
		goto exit_free_fwu;
	}

	fwu->rmi4_data = rmi4_data;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			PDT_PROPS,
			pdt_props.data,
			sizeof(pdt_props.data));
	if (retval < 0) {
		dev_dbg(rmi4_data->pdev->dev.parent,
				"%s: Failed to read PDT properties, assuming 0x00\n",
				__func__);
	} else if (pdt_props.has_bsr) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Reflash for LTS not currently supported\n",
				__func__);
		retval = -ENODEV;
		goto exit_free_mem;
	}

	retval = fwu_scan_pdt();
	if (retval < 0)
		goto exit_free_mem;

	fwu->productinfo1 = rmi4_data->rmi4_mod_info.product_info[0];
	fwu->productinfo2 = rmi4_data->rmi4_mod_info.product_info[1];
	memcpy(fwu->product_id, rmi4_data->rmi4_mod_info.product_id_string,
			SYNAPTICS_RMI4_PRODUCT_ID_SIZE);
	fwu->product_id[SYNAPTICS_RMI4_PRODUCT_ID_SIZE] = 0;

	dev_dbg(rmi4_data->pdev->dev.parent,
			"%s: F01 product info: 0x%04x 0x%04x\n",
			__func__, fwu->productinfo1, fwu->productinfo2);
	dev_dbg(rmi4_data->pdev->dev.parent,
			"%s: F01 product ID: %s\n",
			__func__, fwu->product_id);

	retval = fwu_read_f34_queries();
	if (retval < 0)
		goto exit_free_mem;

	fwu->force_update = FORCE_UPDATE;
	fwu->do_lockdown = DO_LOCKDOWN;
	fwu->initialized = true;

	retval = sysfs_create_bin_file(&rmi4_data->input_dev->dev.kobj,
			&dev_attr_data);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to create sysfs bin file\n",
				__func__);
		goto exit_free_mem;
	}

	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		retval = sysfs_create_file(&rmi4_data->input_dev->dev.kobj,
				&attrs[attr_count].attr);
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to create sysfs attributes\n",
					__func__);
			retval = -ENODEV;
			goto exit_remove_attrs;
		}
	}

#ifdef DO_STARTUP_FW_UPDATE
	wake_lock_init(&fwu->fwu_wake_lock, WAKE_LOCK_SUSPEND, "fwu_wake_lock");
	fwu->fwu_workqueue = create_singlethread_workqueue("fwu_workqueue");
	INIT_WORK(&fwu->fwu_work, fwu_startup_fw_update_work);
	queue_work(fwu->fwu_workqueue,
			&fwu->fwu_work);
#endif
#ifdef CONFIG_TOUCHSCREEN_TOUCH_FW_UPDATE
	register_synaptics_fw_update();
#endif

	return 0;

exit_remove_attrs:
	for (attr_count--; attr_count >= 0; attr_count--) {
		sysfs_remove_file(&rmi4_data->input_dev->dev.kobj,
				&attrs[attr_count].attr);
	}

	sysfs_remove_bin_file(&rmi4_data->input_dev->dev.kobj, &dev_attr_data);

exit_free_mem:
	kfree(fwu->image_name);

exit_free_fwu:
	kfree(fwu);
	fwu = NULL;

exit:
	return retval;
}

static void synaptics_rmi4_fwu_remove(struct synaptics_rmi4_data *rmi4_data)
{
	unsigned char attr_count;

	if (!fwu)
		goto exit;

#ifdef CONFIG_TOUCHSCREEN_TOUCH_FW_UPDATE
	unregister_fw_update();
#endif

#ifdef DO_STARTUP_FW_UPDATE
	cancel_work_sync(&fwu->fwu_work);
	flush_workqueue(fwu->fwu_workqueue);
	destroy_workqueue(fwu->fwu_workqueue);
	wake_lock_destroy(&fwu->fwu_wake_lock);
#endif

	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		sysfs_remove_file(&rmi4_data->input_dev->dev.kobj,
				&attrs[attr_count].attr);
	}

	sysfs_remove_bin_file(&rmi4_data->input_dev->dev.kobj, &dev_attr_data);

	kfree(fwu->read_config_buf);
	kfree(fwu->image_name);
	kfree(fwu);
	fwu = NULL;

exit:
	complete(&fwu_remove_complete);

	return;
}

static struct synaptics_rmi4_exp_fn fwu_module = {
	.fn_type = RMI_FW_UPDATER,
	.init = synaptics_rmi4_fwu_init,
	.remove = synaptics_rmi4_fwu_remove,
	.reset = NULL,
	.reinit = NULL,
	.early_suspend = NULL,
	.suspend = NULL,
	.resume = NULL,
	.late_resume = NULL,
	.attn = synaptics_rmi4_fwu_attn,
};

static int __init rmi4_fw_update_module_init(void)
{
	if ((strcmp(htc_get_bootmode(), "offmode_charging") == 0)
		|| (strcmp(htc_get_bootmode(), "recovery") == 0)) {
		pr_info("%s: %s mode\n", __func__, htc_get_bootmode());
		return 0;
	}

	pr_info("%s\n", __func__);
	synaptics_rmi4_new_function(&fwu_module, true);

	return 0;
}

static void __exit rmi4_fw_update_module_exit(void)
{
	if ((strcmp(htc_get_bootmode(), "offmode_charging") == 0)
		|| (strcmp(htc_get_bootmode(), "recovery") == 0)) {
		return;
	}

	synaptics_rmi4_new_function(&fwu_module, false);

	wait_for_completion(&fwu_remove_complete);

	return;
}

module_init(rmi4_fw_update_module_init);
module_exit(rmi4_fw_update_module_exit);

MODULE_AUTHOR("Synaptics, Inc.");
MODULE_DESCRIPTION("Synaptics DSX FW Update Module");
MODULE_LICENSE("GPL v2");
