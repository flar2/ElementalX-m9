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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef _SYNAPTICS_DSX_RMI4_H_
#define _SYNAPTICS_DSX_RMI4_H_

#define SYNAPTICS_DS4 (1 << 0)
#define SYNAPTICS_DS5 (1 << 1)
#define SYNAPTICS_DSX_DRIVER_PRODUCT (SYNAPTICS_DS4 | SYNAPTICS_DS5)
#define SYNAPTICS_DSX_DRIVER_VERSION 0x2002

#include <linux/version.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#ifdef CONFIG_FB
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 38))
#define KERNEL_ABOVE_2_6_38
#endif

#ifdef KERNEL_ABOVE_2_6_38
#define sstrtoul(...) kstrtoul(__VA_ARGS__)
#else
#define sstrtoul(...) strict_strtoul(__VA_ARGS__)
#endif

#define dev_fm(fmt) "[TP]:" fmt
#define dev_err_fm(fmt) "[TP]:[ERR]:" fmt

#ifdef pr_info
#undef pr_info
#endif
#define pr_info(fmt, ...) printk(KERN_INFO dev_fm(fmt), ##__VA_ARGS__)

#ifdef pr_notice
#undef pr_notice
#endif
#define pr_notice(fmt, ...) printk(KERN_INFO dev_fm(fmt), ##__VA_ARGS__)

#ifdef pr_err
#undef pr_err
#endif
#define pr_err(fmt, ...) printk(KERN_ERR dev_err_fm(fmt), ##__VA_ARGS__)
#ifdef dev_info
#undef dev_info
#endif
#define dev_info(dev, fmt, ...)  do { if (dev) \
			printk(KERN_INFO dev_fm(fmt), ##__VA_ARGS__); \
			} while (0)

#ifdef dev_err
#undef dev_err
#endif
#define dev_err(dev, fmt, ...)  do { if (dev) \
			printk(KERN_ERR dev_err_fm(fmt), ##__VA_ARGS__); \
			} while (0)

#define PDT_PROPS (0X00EF)
#define PDT_START (0x00E9)
#define PDT_END (0x00D0)
#define PDT_ENTRY_SIZE (0x0006)
#define PAGES_TO_SERVICE (10)
#define PAGE_SELECT_LEN (2)
#define ADDRESS_WORD_LEN (2)

#define SYNAPTICS_RMI4_F01 (0x01)
#define SYNAPTICS_RMI4_F11 (0x11)
#define SYNAPTICS_RMI4_F12 (0x12)
#define SYNAPTICS_RMI4_F1A (0x1a)
#define SYNAPTICS_RMI4_F34 (0x34)
#define SYNAPTICS_RMI4_F51 (0x51)
#define SYNAPTICS_RMI4_F54 (0x54)
#define SYNAPTICS_RMI4_F55 (0x55)
#define SYNAPTICS_RMI4_FDB (0xdb)

#define SYNAPTICS_RMI4_PRODUCT_INFO_SIZE 2
#define SYNAPTICS_RMI4_PRODUCT_ID_SIZE 10
#define SYNAPTICS_RMI4_CHIP_ID_SIZE 2
#define SYNAPTICS_RMI4_BUILD_ID_SIZE 3

#define F12_FINGERS_TO_SUPPORT 10
#define F12_NO_OBJECT_STATUS 0x00
#define F12_FINGER_STATUS 0x01
#define F12_STYLUS_STATUS 0x02
#define F12_PALM_STATUS 0x03
#define F12_HOVERING_FINGER_STATUS 0x05
#define F12_GLOVED_FINGER_STATUS 0x06

#define MAX_NUMBER_OF_BUTTONS 4
#define MAX_INTR_REGISTERS 4

#define MASK_16BIT 0xFFFF
#define MASK_8BIT 0xFF
#define MASK_7BIT 0x7F
#define MASK_6BIT 0x3F
#define MASK_5BIT 0x1F
#define MASK_4BIT 0x0F
#define MASK_3BIT 0x07
#define MASK_2BIT 0x03
#define MASK_1BIT 0x01

#define SENSOR_ID_CHECKING_EN	1 << 16

#define SYNAPTICS_RMI4_UPDATE_NONE	0
#define SYNAPTICS_RMI4_UPDATE_IMAGE	1
#define SYNAPTICS_RMI4_UPDATE_CONFIG	2
#define SYNAPTICS_RMI4_UPDATE_BOTH	3

enum exp_fn {
	RMI_DEV = 0,
	RMI_FW_UPDATER,
	RMI_TEST_REPORTING,
	RMI_PROXIMITY,
	RMI_ACTIVE_PEN,
	RMI_DEBUG,
	RMI_LAST,
};

struct synaptics_rmi4_fn_desc {
	unsigned char query_base_addr;
	unsigned char cmd_base_addr;
	unsigned char ctrl_base_addr;
	unsigned char data_base_addr;
	unsigned char intr_src_count;
	unsigned char fn_number;
};

struct synaptics_rmi4_fn_full_addr {
	unsigned short query_base;
	unsigned short cmd_base;
	unsigned short ctrl_base;
	unsigned short data_base;
};

struct synaptics_rmi4_f11_extra_data {
	unsigned char data38_offset;
};

struct synaptics_rmi4_f12_extra_data {
	unsigned char data1_offset;
	unsigned char data4_offset;
	unsigned char data15_offset;
	unsigned char data15_size;
	unsigned char data15_data[(F12_FINGERS_TO_SUPPORT + 7) / 8];
	unsigned char ctrl9_offset;
	unsigned char ctrl10_offset;
	unsigned char ctrl11_offset;
	unsigned char ctrl15_offset;
	unsigned char ctrl20_offset;
	unsigned char ctrl23_offset;
};

struct synaptics_rmi4_fn {
	unsigned char fn_number;
	unsigned char num_of_data_sources;
	unsigned char num_of_data_points;
	unsigned char size_of_data_register_block;
	unsigned char intr_reg_num;
	unsigned char intr_mask;
	struct synaptics_rmi4_fn_full_addr full_addr;
	struct list_head link;
	int data_size;
	void *data;
	void *extra;
};

struct synaptics_rmi4_device_info {
	unsigned int version_major;
	unsigned int version_minor;
	unsigned char manufacturer_id;
	unsigned char product_props;
	unsigned char product_info[SYNAPTICS_RMI4_PRODUCT_INFO_SIZE];
	unsigned char product_id_string[SYNAPTICS_RMI4_PRODUCT_ID_SIZE + 1];
	unsigned char package_id[SYNAPTICS_RMI4_CHIP_ID_SIZE];
	unsigned char build_id[SYNAPTICS_RMI4_BUILD_ID_SIZE];
	struct list_head support_fn_list;
};

struct synaptics_rmi4_noise_state {
	uint16_t im;
	uint16_t cidim;
	uint16_t freq;
	uint8_t ns;
};

struct synaptics_rmi4_report_points {
	uint8_t state;
	int x;
	int y;
	int z;
	int w;
};

struct synaptics_rmi4_data {
	struct platform_device *pdev;
	struct input_dev *input_dev;
	struct input_dev *sr_input_dev;
	const struct synaptics_dsx_hw_interface *hw_if;
	struct synaptics_rmi4_device_info rmi4_mod_info;
	struct regulator *pwr_reg;
	struct regulator *bus_reg;
	struct mutex rmi4_reset_mutex;
	struct mutex rmi4_report_mutex;
	struct mutex rmi4_io_ctrl_mutex;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
#ifdef CONFIG_FB
	struct notifier_block fb_notifier;
#endif
	unsigned char current_page;
	unsigned char button_0d_enabled;
	unsigned char full_pm_cycle;
	unsigned char num_of_tx;
	unsigned char num_of_rx;
	unsigned char num_of_fingers;
	unsigned char max_touch_width;
	unsigned char report_enable;
	unsigned char no_sleep_setting;
	unsigned char intr_mask[MAX_INTR_REGISTERS];
	unsigned char *button_txrx_mapping;
	unsigned short num_of_intr_regs;
	unsigned short f01_query_base_addr;
	unsigned short f01_cmd_base_addr;
	unsigned short f01_ctrl_base_addr;
	unsigned short f01_data_base_addr;
	unsigned short f34_query_base_addr;
	unsigned short f34_cmd_base_addr;
	unsigned short f34_ctrl_base_addr;
	unsigned short f34_data_base_addr;
	unsigned short f54_query_base_addr;
	unsigned short f54_cmd_base_addr;
	unsigned short f54_ctrl_base_addr;
	unsigned short f54_data_base_addr;
	unsigned short f51_query_base_addr;
	unsigned short f51_cmd_base_addr;
	unsigned short f51_ctrl_base_addr;
	unsigned short f51_data_base_addr;
	unsigned int firmware_id;
	unsigned int chip_id;
	int irq;
	int sensor_max_x;
	int sensor_max_y;
	int sensor_max_z;
	bool flash_prog_mode;
	bool irq_enabled;
	bool fingers_on_2d;
	bool suspend;
	bool sensor_sleep;
	bool stay_awake;
	unsigned char support_htc_event;
	int (*irq_enable)(struct synaptics_rmi4_data *rmi4_data, bool enable);
	bool f11_wakeup_gesture;
	bool f12_wakeup_gesture;
	bool enable_wakeup_gesture;
	int (*reset_device)(struct synaptics_rmi4_data *rmi4_data);
	int tw_vendor_pin;
	int face_down;
	struct pinctrl *ts_pinctrl;
	struct pinctrl_state *gpio_state_active;
	struct pinctrl_state *gpio_state_suspend;
	int width_factor;
	int height_factor;
	struct synaptics_rmi4_report_points report_points[10];
	uint32_t config_version;
	uint16_t  tw_vendor;
	unsigned char  diag_command;
	atomic_t data_ready;
	int16_t *report_data;
	uint8_t *temp_report_data;
	uint8_t i2c_to_mcu;
	int cont_splash_enable;	
	uint8_t glove_enable;
	uint8_t glove_status;
	uint8_t glove_setting;
	uint8_t cover_mode;
	uint8_t cover_setting_size;
	uint8_t cover_setting[10];
	uint8_t uncover_setting[10];
	uint8_t glove_mode_setting[10];
	struct synaptics_rmi4_noise_state noise_state;
	uint8_t hall_block_touch_event;
};

struct synaptics_dsx_bus_access {
	unsigned char type;
	int (*read)(struct synaptics_rmi4_data *rmi4_data, unsigned short addr,
		unsigned char *data, unsigned short length);
	int (*write)(struct synaptics_rmi4_data *rmi4_data, unsigned short addr,
		unsigned char *data, unsigned short length);
};

struct synaptics_dsx_hw_interface {
	struct synaptics_dsx_board_data *board_data;
	const struct synaptics_dsx_bus_access *bus_access;
	int (*bl_hw_init)(struct synaptics_rmi4_data *rmi4_data);
	int (*ui_hw_init)(struct synaptics_rmi4_data *rmi4_data);
};

struct synaptics_rmi4_exp_fn {
	enum exp_fn fn_type;
	int (*init)(struct synaptics_rmi4_data *rmi4_data);
	void (*remove)(struct synaptics_rmi4_data *rmi4_data);
	void (*reset)(struct synaptics_rmi4_data *rmi4_data);
	void (*reinit)(struct synaptics_rmi4_data *rmi4_data);
	void (*early_suspend)(struct synaptics_rmi4_data *rmi4_data);
	void (*suspend)(struct synaptics_rmi4_data *rmi4_data);
	void (*resume)(struct synaptics_rmi4_data *rmi4_data);
	void (*late_resume)(struct synaptics_rmi4_data *rmi4_data);
	void (*attn)(struct synaptics_rmi4_data *rmi4_data,
			unsigned char intr_mask);
};

int synaptics_rmi4_bus_init(void);

void synaptics_rmi4_bus_exit(void);

void synaptics_rmi4_new_function(struct synaptics_rmi4_exp_fn *exp_fn_module,
		bool insert);

int synaptics_fw_updater(unsigned char *fw_data);
int synaptics_config_updater(struct synaptics_dsx_board_data *bdata);

static inline int synaptics_rmi4_reg_read(
		struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr,
		unsigned char *data,
		unsigned short len)
{
	return rmi4_data->hw_if->bus_access->read(rmi4_data, addr, data, len);
}

static inline int synaptics_rmi4_reg_write(
		struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr,
		unsigned char *data,
		unsigned short len)
{
	return rmi4_data->hw_if->bus_access->write(rmi4_data, addr, data, len);
}

static inline ssize_t synaptics_rmi4_show_error(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	dev_warn(dev, "%s Attempted to read from write-only attribute %s\n",
			__func__, attr->attr.name);
	return -EPERM;
}

static inline ssize_t synaptics_rmi4_store_error(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	dev_warn(dev, "%s Attempted to write to read-only attribute %s\n",
			__func__, attr->attr.name);
	return -EPERM;
}

static inline void batohs(unsigned short *dest, unsigned char *src)
{
	*dest = src[1] * 0x100 + src[0];
}

static inline void hstoba(unsigned char *dest, unsigned short src)
{
	dest[0] = src % 0x100;
	dest[1] = src / 0x100;
}

#endif
