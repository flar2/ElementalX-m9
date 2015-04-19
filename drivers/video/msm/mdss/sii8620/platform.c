/*
 * SiI8620 Linux Driver
 *
 * Copyright (C) 2013 Silicon Image, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 * This program is distributed AS-IS WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; INCLUDING without the implied warranty
 * of MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE or NON-INFRINGEMENT.
 * See the GNU General Public License for more details at
 * http://www.gnu.org/licenses/gpl-2.0.html.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>

#include <linux/types.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/semaphore.h>
#include <linux/cdev.h>
#include <linux/spi/spi.h>
#include <linux/regulator/consumer.h>
#include <linux/async.h>

#include "si_fw_macros.h"
#include "si_infoframe.h"
#include "si_edid.h"
#include "si_mhl_defs.h"
#include "si_mhl2_edid_3d_api.h"
#include "si_mhl_tx_hw_drv_api.h"
#ifdef MEDIA_DATA_TUNNEL_SUPPORT
#include <linux/input.h>
#include "si_mdt_inputdev.h"
#endif
#include "mhl_linux_tx.h"
#include "mhl_supp.h"
#include "platform.h"
#include "si_mhl_callback_api.h"
#include "si_8620_drv.h"
#include "si_8620_regs.h"
#include "../mdss_hdmi_mhl.h"



#define GPIO_EXP_ADDR				0x40

#define RESET_PULSE_WIDTH			1	

#define GPIO_EXP_INPUT_REGS_OFFSET		0x80
#define GPIO_EXP_OUTPUT_REGS_OFFSET		0x88
#define GPIO_EXP_POL_INVERT_REGS_OFFSET		0x90
#define GPIO_EXP_IO_CONFIG_REGS_OFFSET		0x98
#define GPIO_EXP_INTR_MASK_REGS_OFFSET		0xA0

#define GPIO_EXP_BANK_2_OUTPUT_DEFAULT		0xFF
#define GPIO_EXP_BANK_2_3D			(0x01 << 0)
#define GPIO_EXP_BANK_2_PKD_PXL			(0x01 << 1)
#define GPIO_EXP_BANK_2_HDCP_ON			(0x01 << 2)
#define GPIO_EXP_BANK_2_N_TCODE			(0x01 << 3)
#define GPIO_EXP_BANK_2_LED_USB_MODE		(0x01 << 4)
#define GPIO_EXP_BANK_2_SPR_LED2		(0x01 << 5)
#define GPIO_EXP_BANK_2_SPR_LED3		(0x01 << 6)
#define GPIO_EXP_BANK_2_SPR_LED4		(0x01 << 7)

#define GPIO_EXP_BANK_3_OUTPUT_DEFAULT		0x67

#define GPIO_EXP_BANK_3_MHL_TX_RST_B		(0x01 << 0)
#define GPIO_EXP_BANK_3_FW_WAKE_A		(0x01 << 1)
#define GPIO_EXP_BANK_3_CHG_DET			(0x01 << 2)
#define GPIO_EXP_BANK_3_XO3_SINK_VBUS_SENSE	(0x01 << 3)
#define GPIO_EXP_BANK_3_12V_PS_SENSE		(0x01 << 4)
#define GPIO_EXP_BANK_3_EEPROM_WR_EN		(0x01 << 5)
#define GPIO_EXP_BANK_3_TX2MHLRX_PWR_A		(0x01 << 6)
#define GPIO_EXP_BANK_3_M2U_VBUS_CTRL_A		(0x01 << 7)

#define GPIO_EXP_BANK_4_OUTPUT_DEFAULT		0xF0
#define GPIO_EXP_BANK_4_DSW9			(0x01 << 0)
#define GPIO_EXP_BANK_4_DSW10			(0x01 << 1)
#define GPIO_EXP_BANK_4_DSW11			(0x01 << 2)
#define GPIO_EXP_BANK_4_DSW12			(0x01 << 3)
#define GPIO_EXP_BANK_4_USB_SW_CTRL0		(0x01 << 4)
#define GPIO_EXP_BANK_4_USB_SW_CTRL1		(0x01 << 5)
#define GPIO_EXP_BANK_4_LED15_AMBER		(0x01 << 6)
#define GPIO_EXP_BANK_4_LED15_GREEN		(0x01 << 7)

#define GET_FROM_MODULE_PARAM			-1
#define GPIO_ON_EXPANDER			-2

#define REG_PCA_950x_PORT_0_INPUT		0x00
#define REG_PCA_950x_PORT_1_INPUT		0x01
#define REG_PCA_950x_PORT_2_INPUT		0x02
#define REG_PCA_950x_PORT_3_INPUT		0x03
#define REG_PCA_950x_PORT_4_INPUT		0x04

#define REG_PCA_950x_PORT_0_OUTPUT		0x08
#define REG_PCA_950x_PORT_1_OUTPUT		0x09
#define REG_PCA_950x_PORT_2_OUTPUT		0x0A
#define REG_PCA_950x_PORT_3_OUTPUT		0x0B
#define REG_PCA_950x_PORT_4_OUTPUT		0x0C

u8 gpio_exp_bank2_output;
u8 gpio_exp_bank3_output;
u8 gpio_exp_bank4_output;

static char *buildTime = "Built " __DATE__ "-" __TIME__;
static char *buildVersion = "1.01." BUILD_NUM_STRING;

struct semaphore platform_lock;
static uint32_t platform_flags;
bool probe_fail = false;

static struct spi_device *spi_dev;
#define SPI_BUS_NUM		1
#define	SPI_CHIP_SEL		0
#define SPI_TRANSFER_MODE	SPI_MODE_0
#define SPI_BUS_SPEED		1000000

enum si_spi_opcodes {
	spi_op_disable = 0x04,
	spi_op_enable = 0x06,
	spi_op_reg_read = 0x60,
	spi_op_reg_write = 0x61,
	spi_op_emsc_read = 0x80,
	spi_op_emsc_write = 0x81,
	spi_op_slow_cbus_read = 0x90,
	spi_op_slow_cbus_write = 0x91
};

#define MAX_SPI_PAYLOAD_SIZE		256
#define MAX_SPI_CMD_SIZE		3
#define EMSC_WRITE_SPI_CMD_SIZE		1
#define EMSC_READ_SPI_CMD_SIZE		1
#define MAX_SPI_DUMMY_XFER_BYTES	20
#define MAX_SPI_XFER_BUFFER_SIZE	(MAX_SPI_CMD_SIZE + \
		MAX_SPI_DUMMY_XFER_BYTES + MAX_SPI_PAYLOAD_SIZE)
#define MAX_SPI_EMSC_BLOCK_SIZE (MAX_SPI_CMD_SIZE + MAX_SPI_PAYLOAD_SIZE)

#define MAX_I2C_PAYLOAD_SIZE		256
#define MAX_I2C_CMD_SIZE		0

#define MAX_I2C_EMSC_BLOCK_SIZE (MAX_I2C_CMD_SIZE + MAX_I2C_PAYLOAD_SIZE)

struct spi_xfer_mem {
	u8 *tx_buf;
	u8 *rx_buf;
	uint8_t *block_tx_buffers;
	struct spi_transfer spi_xfer[2];
	struct spi_message spi_cmd;
} spi_mem;

struct i2c_xfer_mem {
	uint8_t *block_tx_buffers;
} i2c_mem;

static struct i2c_adapter *i2c_bus_adapter;

struct i2c_dev_info {
	uint8_t dev_addr;
	struct i2c_client *client;
};

#define I2C_DEV_INFO(addr) \
	{.dev_addr = addr >> 1, .client = NULL}

static struct i2c_dev_info device_addresses[] = {
	I2C_DEV_INFO(SA_TX_PAGE_0),
	I2C_DEV_INFO(SA_TX_PAGE_1),
	I2C_DEV_INFO(SA_TX_PAGE_2),
	I2C_DEV_INFO(SA_TX_PAGE_3),
	I2C_DEV_INFO(SA_TX_PAGE_6),
	I2C_DEV_INFO(SA_TX_CBUS),
	I2C_DEV_INFO(GPIO_EXP_ADDR),
};

int debug_level;
bool debug_reg_dump;
bool input_dev_rap = 1;
bool input_dev_rcp = 1;
bool input_dev_ucp = 1;
bool input_dev_rbp = 1;
int hdcp_content_type;
bool use_spi = 0;
int crystal_khz = 19200; 
int use_heartbeat;

bool wait_for_user_intr;
int tmds_link_speed;
bool continue_to_ecbus = 1;
#ifdef FORCE_OCBUS_FOR_ECTS
bool force_ocbus_for_ects;
#endif
int gpio_index = 138;

module_param(debug_reg_dump, bool, S_IRUGO);
module_param(debug_level, int, S_IRUGO);

module_param(input_dev_rap, bool, S_IRUGO);
module_param(input_dev_rcp, bool, S_IRUGO);
module_param(input_dev_ucp, bool, S_IRUGO);
module_param(input_dev_rbp, bool, S_IRUGO);
module_param(hdcp_content_type, int, S_IRUGO);
module_param(use_spi, bool, S_IRUGO);
module_param(crystal_khz, int, S_IRUGO);
module_param(use_heartbeat, int, S_IRUGO);
module_param(wait_for_user_intr, bool, S_IRUGO);
module_param(tmds_link_speed, int, S_IRUGO);
module_param(continue_to_ecbus, bool, S_IRUGO);
#ifdef	FORCE_OCBUS_FOR_ECTS
module_param(force_ocbus_for_ects, bool, S_IRUGO);
#endif

module_param_named(debug_msgs, debug_level, int, S_IRUGO);

struct platform_signals_list platform_signals[] = {
	{.name = "TX_HW_RESET",
	 .gpio_number = GET_FROM_MODULE_PARAM,
	 .param = NULL},
	{.name = "TX_FW_WAKE",
	 .gpio_number = GET_FROM_MODULE_PARAM,
	 .param = NULL},
	{.name = "CHG_DET",
	 .gpio_number = GET_FROM_MODULE_PARAM,
	 .param = NULL},
	{.name = "XO3_SINK_VBUS_SENSE",
	 .gpio_number = GET_FROM_MODULE_PARAM,
	 .param = NULL},
	{.name = "TWELVE_VOLT_PS_SENSE",
	 .gpio_number = GET_FROM_MODULE_PARAM,
	 .param = NULL},
	{.name = "EEPROM_WR_EN",
	 .gpio_number = GET_FROM_MODULE_PARAM,
	 .param = NULL},
	{.name = "TX2MHLRX_PWR",
	 .gpio_number = GET_FROM_MODULE_PARAM,
	 .param = NULL},
	{.name = "M2U_VBUS_CTRL",
	 .gpio_number = GET_FROM_MODULE_PARAM,
	 .param = NULL},
	{.name = "LED_3D",
	 .gpio_number = GET_FROM_MODULE_PARAM,
	 .param = NULL},
	{.name = "LED_PACKED_PIXEL",
	 .gpio_number = GET_FROM_MODULE_PARAM,
	 .param = NULL},
	{.name = "LED_HDCP",
	 .gpio_number = GET_FROM_MODULE_PARAM,
	 .param = NULL},
	{.name = "LED_USB_MODE",
	 .gpio_number = GET_FROM_MODULE_PARAM,
	 .param = NULL},
	{.name = "LED_SPARE_2",
	 .gpio_number = GET_FROM_MODULE_PARAM,
	 .param = NULL},
	{.name = "LED_SPARE_3",
	 .gpio_number = GET_FROM_MODULE_PARAM,
	 .param = NULL},
	{.name = "LED_SPARE_4",
	 .gpio_number = GET_FROM_MODULE_PARAM,
	 .param = NULL},
	{.name = "X02_USB_SW_CTRL",
	 .gpio_number = GET_FROM_MODULE_PARAM,
	 .param = NULL},
	{.name = "X02_USB_SW_CTRL0",
	 .gpio_number = GET_FROM_MODULE_PARAM,
	 .param = NULL},
	{.name = "X02_USB_SW_CTRL1",
	 .gpio_number = GET_FROM_MODULE_PARAM,
	 .param = NULL},
	{.name = "X02_LED15_AMBER",
	 .gpio_number = GET_FROM_MODULE_PARAM,
	 .param = NULL},
	{.name = "X02_LED15_GREEN",
	 .gpio_number = GET_FROM_MODULE_PARAM,
	 .param = NULL},
	{.name = "6031_SEL1",
	 .gpio_number = GET_FROM_MODULE_PARAM,
	 .param = NULL},
	{.name = "6031_SEL2",
	 .gpio_number = GET_FROM_MODULE_PARAM,
	 .param = NULL}
};

static inline int platform_read_i2c_block(struct i2c_adapter *i2c_bus, u8 page,
					  u8 offset, u16 count, u8 *values)
{
	struct i2c_msg msg[2];

	msg[0].flags = 0;
	msg[0].addr = page >> 1;
	msg[0].buf = &offset;
	msg[0].len = 1;

	msg[1].flags = I2C_M_RD;
	msg[1].addr = page >> 1;
	msg[1].buf = values;
	msg[1].len = count;

	return i2c_transfer(i2c_bus_adapter, msg, 2);
}

static inline int platform_write_i2c_block(struct i2c_adapter *i2c_bus, u8 page,
					   u8 offset, u16 count, u8 *values)
{
	struct i2c_msg msg;
	u8 *buffer;
	int ret;

	buffer = kmalloc(count + 1, GFP_KERNEL);
	if (!buffer) {
		printk(KERN_ERR "%s:%d buffer allocation failed\n",
			__func__, __LINE__);
		return -ENOMEM;
	}

	buffer[0] = offset;
	memmove(&buffer[1], values, count);

	msg.flags = 0;
	msg.addr = page >> 1;
	msg.buf = buffer;
	msg.len = count + 1;

	ret = i2c_transfer(i2c_bus, &msg, 1);

	kfree(buffer);

	if (ret != 1) {
		printk(KERN_ERR "%s:%d I2c write failed 0x%02x:0x%02x\n",
			__func__, __LINE__, page, offset);
		ret = -EIO;
	} else {
		ret = 0;
	}

	return ret;
}

static int platform_read_i2c_reg(struct i2c_adapter *bus_adapter_i2c, u8 page,
				 u8 offset)
{
	int ret;
	u8 byte_read;
	ret =
	    platform_read_i2c_block(bus_adapter_i2c, page, offset, 1,
				    &byte_read);
	MHL_TX_DBG_INFO("\tGI2C_R %2x:%2x = %2x\n", page, offset, ret);
	if (ret != 2) {
		printk(KERN_ERR "%s:%d I2c read failed, 0x%02x:0x%02x\n",
			__func__, __LINE__, page, offset);
		ret = -EIO;
	} else {
		ret = 0;
	}
	return ret ? ret : byte_read;
}

static int platform_write_i2c_reg(struct i2c_adapter *bus_adapter_i2c, u8 page,
				  u8 offset, u8 value)
{
	MHL_TX_DBG_INFO("\tGI2C_W %2x:%2x <- %2x\n", page, offset, value);
	return platform_write_i2c_block(bus_adapter_i2c, page, offset, 1,
					&value);
}

uint32_t platform_get_flags(void)
{
	return platform_flags;
}

int get_config(void *dev_context, int config_idx)
{
	int pin_state = 0;

	if (config_idx < ARRAY_SIZE(platform_signals)) {
		switch (platform_signals[config_idx].gpio_number) {
		case GET_FROM_MODULE_PARAM:
			pin_state = *(platform_signals[config_idx].param);
			break;
		case GPIO_ON_EXPANDER:
			pin_state =
			    (platform_read_i2c_reg
			     (i2c_bus_adapter,
			      platform_signals[config_idx].gpio_reg_PCA950x.
			      slave_addr,
			      platform_signals[config_idx].gpio_reg_PCA950x.
			      offset)
			     & platform_signals[config_idx].
			     gpio_mask_PCA950x) ? 1 : 0;
			break;
		default:
			pin_state =
			    gpio_get_value(platform_signals[config_idx].
					   gpio_number);
			break;
		}
	}
	return pin_state;
}

void set_pin_impl(void *dev_context, int pin_idx, int value,
		  const char *function_name, int line_num)
{
	uint8_t bank_value;

	if (pin_idx < ARRAY_SIZE(platform_signals)) {

		MHL_TX_DBG_INFO("set_pin(%s,%d)\n",
				platform_signals[pin_idx].name, value);
		switch (platform_signals[pin_idx].gpio_number) {
		case GET_FROM_MODULE_PARAM:
			break;
		case GPIO_ON_EXPANDER:
			bank_value =
			    *(platform_signals[pin_idx].gpio_bank_value);
			if (value)
				bank_value |=
				    platform_signals[pin_idx].gpio_mask_PCA950x;
			else
				bank_value &=
				    ~platform_signals[pin_idx].
				    gpio_mask_PCA950x;

			*(platform_signals[pin_idx].gpio_bank_value) =
			    bank_value;
			platform_write_i2c_reg(i2c_bus_adapter,
					       platform_signals[pin_idx].
					       gpio_reg_PCA950x.slave_addr,
					       platform_signals[pin_idx].
					       gpio_reg_PCA950x.offset,
					       bank_value);
			break;
		default:
			gpio_set_value(platform_signals[pin_idx].gpio_number,
				       value);
			break;
		}
	}
}

void mhl_tx_vbus_control(enum vbus_power_state power_state)
{
	struct device *parent_dev;
	struct mhl_dev_context *dev_context;

	if (use_spi)
		parent_dev = &spi_dev->dev;
	else
		parent_dev = &device_addresses[0].client->dev;

	dev_context = dev_get_drvdata(parent_dev);

	switch (power_state) {
	case VBUS_OFF:
		set_pin(dev_context, M2U_VBUS_CTRL, 0);
		set_pin(dev_context, TX2MHLRX_PWR, 1);
		break;

	case VBUS_ON:
		set_pin(dev_context, TX2MHLRX_PWR, 0);
		set_pin(dev_context, M2U_VBUS_CTRL, 1);
		break;

	default:
		dev_err(dev_context->mhl_dev,
			"%s: Invalid power state %d received!\n",
			__func__, power_state);
		break;
	}
}

int si_device_dbg_i2c_reg_xfer(void *dev_context, u8 page, u8 offset,
			       u16 count, bool rw_flag, u8 *buffer)
{
	u16 address = (page << 8) | offset;

	if (rw_flag == DEBUG_I2C_WRITE)
		return mhl_tx_write_reg_block(dev_context, address, count,
					      buffer);
	else
		return mhl_tx_read_reg_block(dev_context, address, count,
					     buffer);
}

#define MAX_DEBUG_MSG_SIZE	1024

#if defined(DEBUG)

char *find_file_name(const char *path_spec)
{
	char *pc;

	for (pc = (char *)&path_spec[strlen(path_spec)];
		pc != path_spec; --pc) {
		if ('\\' == *pc) {
			++pc;
			break;
		}
		if ('/' == *pc) {
			++pc;
			break;
		}
	}
	return pc;
}

void print_formatted_debug_msg(char *file_spec, const char *func_name,
			       int line_num, char *fmt, ...)
{
	uint8_t *msg = NULL;
	uint8_t *msg_offset;
	char *file_spec_sep = NULL;
	int remaining_msg_len = MAX_DEBUG_MSG_SIZE;
	int len;
	va_list ap;

	if (fmt == NULL)
		return;

	msg = kmalloc(remaining_msg_len, GFP_KERNEL);
	if (msg == NULL)
		return;

	msg_offset = msg;

	len = scnprintf(msg_offset, remaining_msg_len, "mhl: ");
	msg_offset += len;
	remaining_msg_len -= len;

	
	if (file_spec != NULL)
		file_spec = find_file_name(file_spec);

	if (file_spec != NULL) {
		if (func_name != NULL)
			file_spec_sep = "->";
		else if (line_num != -1)
			file_spec_sep = ":";
	}

	if (file_spec) {
		len = scnprintf(msg_offset, remaining_msg_len, "%s", file_spec);
		msg_offset += len;
		remaining_msg_len -= len;
	}

	if (file_spec_sep) {
		len =
		    scnprintf(msg_offset, remaining_msg_len, "%s",
			      file_spec_sep);
		msg_offset += len;
		remaining_msg_len -= len;
	}

	if (func_name) {
		len = scnprintf(msg_offset, remaining_msg_len, "%s", func_name);
		msg_offset += len;
		remaining_msg_len -= len;
	}

	if (line_num != -1) {
		if ((file_spec != NULL) || (func_name != NULL))
			len =
			    scnprintf(msg_offset, remaining_msg_len, ":%d ",
				      line_num);
		else
			len =
			    scnprintf(msg_offset, remaining_msg_len, "%d ",
				      line_num);

		msg_offset += len;
		remaining_msg_len -= len;
	}

	va_start(ap, fmt);
	len = vscnprintf(msg_offset, remaining_msg_len, fmt, ap);
	va_end(ap);

	printk(msg);

	kfree(msg);
}

void dump_transfer(enum tx_interface_types if_type,
		   u8 page, u8 offset, u16 count, u8 *values, bool write)
{
	if (debug_reg_dump != 0) {
		int buf_size = 64;
		u16 idx;
		int buf_offset;
		char *buf;
		char *if_type_msg;

		switch (if_type) {
		case TX_INTERFACE_TYPE_I2C:
			if_type_msg = "I2C";
			break;
		case TX_INTERFACE_TYPE_SPI:
			if_type_msg = "SPI";
			break;
		default:
			return;
		};

		if (count > 1) {
			
			buf_size += count * 3;
			
			buf_size += ((count / 16) + 1) * 8;
		}

		buf = kmalloc(buf_size, GFP_KERNEL);
		if (!buf)
			return;

		if (count == 1) {

			scnprintf(buf, buf_size, "   %s %02X.%02X %s %02X\n",
				  if_type_msg,
				  page, offset, write ? "W" : "R", values[0]);
		} else {
			idx = 0;
			buf_offset =
			    scnprintf(buf, buf_size, "%s %02X.%02X %s(%d)",
				      if_type_msg, page, offset,
				      write ? "W" : "R", count);

			for (idx = 0; idx < count; idx++) {
				if (0 == (idx & 0x0F))
					buf_offset +=
					    scnprintf(&buf[buf_offset],
						      buf_size - buf_offset,
						      "\n%04X: ", idx);

				buf_offset += scnprintf(&buf[buf_offset],
							buf_size - buf_offset,
							"%02X ", values[idx]);
			}
			buf_offset +=
			    scnprintf(&buf[buf_offset], buf_size - buf_offset,
				      "\n");
		}

		print_formatted_debug_msg(NULL, NULL, -1, buf);
		kfree(buf);
	}
}
#endif 

static struct mhl_drv_info drv_info = {
	.drv_context_size = sizeof(struct drv_hw_context),
	.mhl_device_initialize = si_mhl_tx_chip_initialize,
	.mhl_device_isr = si_mhl_tx_drv_device_isr,
	.mhl_device_dbg_i2c_reg_xfer = si_device_dbg_i2c_reg_xfer,
	.mhl_device_get_aksv = si_mhl_tx_drv_get_aksv
};

static void toggle_reset_n(void)
{
	MHL_TX_DBG_INFO("Toggle MHL_RST_B/RESET_N pin. Resets 8620 only.\n");
	gpio_set_value(drv_info.reset_pin, 0);

	
	msleep(RESET_PULSE_WIDTH);

	gpio_set_value(drv_info.reset_pin, 1);

}

void platform_mhl_tx_hw_reset(uint32_t reset_period, uint32_t reset_delay)
{
	
	toggle_reset_n();

	if (reset_delay)
		msleep(reset_delay);

	if (use_spi) {
		u8 cmd = spi_op_enable;
		spi_write(spi_dev, &cmd, 1);
	}
}

int is_interrupt_asserted(void)
{
	return (gpio_get_value(drv_info.intr_pin) ? 0 : 1);
}

int mhl_tx_write_reg_block_i2c(void *drv_context, u8 page, u8 offset,
			       u16 count, u8 *values)
{
	DUMP_I2C_TRANSFER(page, offset, count, values, true);

	return platform_write_i2c_block(i2c_bus_adapter, page, offset, count,
					values);
}

int mhl_tx_write_reg_i2c(void *drv_context, u8 page, u8 offset, u8 value)
{
	return mhl_tx_write_reg_block_i2c(drv_context, page, offset, 1, &value);
}

int mhl_tx_read_reg_block_i2c(void *drv_context, u8 page, u8 offset,
			      u16 count, u8 *values)
{
	int ret;

	if (count == 0) {
		MHL_TX_DBG_ERR("Tried to read 0 bytes\n");
		return -EINVAL;
	}

	ret =
	    platform_read_i2c_block(i2c_bus_adapter, page, offset, count,
				    values);
	if (ret != 2) {
		MHL_TX_DBG_ERR("I2c read failed, 0x%02x:0x%02x\n", page,
			       offset);
		ret = -EIO;
	} else {
		ret = 0;
		DUMP_I2C_TRANSFER(page, offset, count, values, false);
	}

	return ret;
}

int mhl_tx_read_reg_i2c(void *drv_context, u8 page, u8 offset)
{
	u8 byte_read;
	int status;

	status = mhl_tx_read_reg_block_i2c(drv_context, page, offset,
					   1, &byte_read);

	return status ? status : byte_read;
}

static int i2c_addr_to_spi_cmd(void *drv_context, bool write, u8 *page,
			       u8 *opcode, u8 *dummy_bytes)
{
	if (write) {
		*opcode = spi_op_reg_write;
		*dummy_bytes = 0;
	} else {
		*opcode = spi_op_reg_read;
		*dummy_bytes = 5;
	}

	switch (*page) {
	case SA_TX_PAGE_0:
		*page = 0;
		break;
	case SA_TX_PAGE_1:
		*page = 1;
		break;
	case SA_TX_PAGE_2:
		*page = 2;
		break;
	case SA_TX_PAGE_3:
		*page = 3;
		break;
	case SA_TX_PAGE_4:
		*page = 4;
		break;
	case SA_TX_CBUS:
		*page = 5;
		break;
	case SA_TX_PAGE_6:
		*page = 6;
		break;
	case SA_TX_PAGE_7:
		*page = 7;
		break;
	case SA_TX_PAGE_8:
		*page = 8;
		break;
	default:
		MHL_TX_DBG_ERR("Called with unknown page 0x%02x\n", *page);
		return -EINVAL;
	}
	return 0;
}

inline uint8_t reg_page(uint16_t address)
{
	return (uint8_t)((address >> 8) & 0x00FF);
}

inline uint8_t reg_offset(uint16_t address)
{
	return (uint8_t)(address & 0x00FF);
}

static int mhl_tx_write_reg_block_spi(void *drv_context, u8 page, u8 offset,
				      u16 count, u8 *values)
{
	u8 opcode;
	u8 dummy_bytes;
	u16 length = count + 3;
	int ret;

	DUMP_SPI_TRANSFER(page, offset, count, values, true);

	ret = i2c_addr_to_spi_cmd(drv_context, true, &page, &opcode,
				  &dummy_bytes);
	if (ret != 0)
		return ret;

	length = 3 + count + dummy_bytes;

	if (length > MAX_SPI_XFER_BUFFER_SIZE) {
		MHL_TX_DBG_ERR("Transfer count (%d) is too large!\n", count);
		return -EINVAL;
	}

	spi_mem.tx_buf[0] = opcode;
	spi_mem.tx_buf[1] = page;
	spi_mem.tx_buf[2] = offset;
	if (dummy_bytes)
		memset(&spi_mem.tx_buf[3], 0, dummy_bytes);

	memmove(&spi_mem.tx_buf[dummy_bytes + 3], values, count);

	ret = spi_write(spi_dev, spi_mem.tx_buf, length);

	if (ret != 0) {
		MHL_TX_DBG_ERR("SPI write block failed, "
			       "page: 0x%02x, register: 0x%02x\n",
			       page, offset);
		ret = -EIO;
	} else {
		ret = 0;
	}

	return ret;
}

static int mhl_tx_write_reg_spi(void *drv_context, u8 page, u8 offset, u8 value)
{
	return mhl_tx_write_reg_block_spi(drv_context, page, offset, 1, &value);
}

static int mhl_tx_read_reg_block_spi(void *drv_context, u8 page, u8 offset,
				     u16 count, u8 *values)
{
	u8 page_num = page;
	u8 opcode;
	u8 dummy_bytes;
	u16 length;
	int ret;

	if (count > MAX_SPI_PAYLOAD_SIZE) {
		MHL_TX_DBG_ERR("Requested transfer size is too large\n");
		return -EINVAL;
	}

	ret = i2c_addr_to_spi_cmd(drv_context, false, &page_num, &opcode,
				  &dummy_bytes);
	if (ret != 0)
		return ret;

	if ((reg_page(REG_DDC_DATA) == page) &&
	    (reg_offset(REG_DDC_DATA) == offset)) {
		dummy_bytes = 11;
		opcode = 0x62;
	}

	length = 3 + count + dummy_bytes;
	if (length > MAX_SPI_XFER_BUFFER_SIZE) {
		MHL_TX_DBG_ERR("Requested transfer size is too large\n");
		return -EINVAL;
	}

	spi_message_init(&spi_mem.spi_cmd);
	memset(&spi_mem.spi_xfer, 0, sizeof(spi_mem.spi_xfer));
	spi_mem.tx_buf[0] = opcode;
	spi_mem.tx_buf[1] = page_num;
	spi_mem.tx_buf[2] = offset;

	spi_mem.spi_xfer[0].tx_buf = spi_mem.tx_buf;
	spi_mem.spi_xfer[0].len = 3 + dummy_bytes;
	spi_message_add_tail(&spi_mem.spi_xfer[0], &spi_mem.spi_cmd);

	spi_mem.spi_xfer[1].rx_buf = spi_mem.rx_buf;
	spi_mem.spi_xfer[1].len = count;
	spi_mem.spi_xfer[1].cs_change = 1;
	spi_message_add_tail(&spi_mem.spi_xfer[1], &spi_mem.spi_cmd);

	ret = spi_sync(spi_dev, &spi_mem.spi_cmd);

	if (ret != 0) {
		MHL_TX_DBG_ERR("SPI read block failed, "
			       "page: 0x%02x, register: 0x%02x\n",
			       page, offset);
	} else {
		memcpy(values, spi_mem.rx_buf, count);
		DUMP_SPI_TRANSFER(page, offset, count, values, false);
	}

	return ret;
}

int mhl_tx_read_reg_block_spi_emsc(void *drv_context, u16 count, u8 * values)
{
	u8 dummy_bytes = 1;
	u16 length;
	int ret;

	if (count > MAX_SPI_PAYLOAD_SIZE) {
		MHL_TX_DBG_ERR("Requested transfer size is too large\n");
		return -EINVAL;
	}

	length = EMSC_READ_SPI_CMD_SIZE + dummy_bytes + count;
	if (length > MAX_SPI_XFER_BUFFER_SIZE) {
		MHL_TX_DBG_ERR("Requested transfer size is too large\n");
		return -EINVAL;
	}

	spi_message_init(&spi_mem.spi_cmd);
	memset(&spi_mem.spi_xfer, 0, sizeof(spi_mem.spi_xfer));
	spi_mem.tx_buf[0] = spi_op_emsc_read;

	spi_mem.spi_xfer[0].tx_buf = spi_mem.tx_buf;
	spi_mem.spi_xfer[0].len = EMSC_READ_SPI_CMD_SIZE + dummy_bytes;
	spi_message_add_tail(&spi_mem.spi_xfer[0], &spi_mem.spi_cmd);

	spi_mem.spi_xfer[1].rx_buf = spi_mem.rx_buf;
	spi_mem.spi_xfer[1].len = count;
	spi_mem.spi_xfer[1].cs_change = 1;
	spi_message_add_tail(&spi_mem.spi_xfer[1], &spi_mem.spi_cmd);

	ret = spi_sync(spi_dev, &spi_mem.spi_cmd);

	if (ret != 0) {
		MHL_TX_DBG_ERR("SPI eMSC read block failed ");
	} else {
		memcpy(values, spi_mem.rx_buf, count);
		
	}

	return ret;
}

static int mhl_tx_read_reg_spi(void *drv_context, u8 page, u8 offset)
{
	u8 byte_read;
	int status;

	status = mhl_tx_read_reg_block_spi(drv_context, page, offset, 1,
					   &byte_read);

	return status ? status : byte_read;
}

int mhl_tx_write_reg_block(void *drv_context, u16 address, u16 count,
	u8 *values)
{
	u8 page = (u8)(address >> 8);
	u8 offset = (u8)address;

	if (use_spi)
		return mhl_tx_write_reg_block_spi(drv_context, page, offset,
						  count, values);
	else
		return mhl_tx_write_reg_block_i2c(drv_context, page, offset,
						  count, values);
}

void si_mhl_tx_platform_get_block_buffer_info(struct block_buffer_info_t
					      *block_buffer_info)
{
	if (use_spi) {
		block_buffer_info->buffer = spi_mem.block_tx_buffers;
		block_buffer_info->req_size = MAX_SPI_EMSC_BLOCK_SIZE;
		block_buffer_info->payload_offset = EMSC_WRITE_SPI_CMD_SIZE;
	} else {
		block_buffer_info->buffer = i2c_mem.block_tx_buffers;
		block_buffer_info->req_size = MAX_I2C_EMSC_BLOCK_SIZE;
		block_buffer_info->payload_offset = MAX_I2C_CMD_SIZE;
	}
}

int mhl_tx_write_block_spi_emsc(void *drv_context, struct block_req *req)
{
	u16 length;
	int ret;


	
	length = EMSC_WRITE_SPI_CMD_SIZE + req->count;

	if (length > MAX_SPI_EMSC_BLOCK_SIZE) {
		MHL_TX_DBG_ERR("Transfer count (%d) is too large!\n",
			       req->count);
		return -EINVAL;
	}

	req->platform_header[0] = spi_op_emsc_write;

	ret = spi_write(spi_dev, req->platform_header, length);

	if (ret != 0) {
		MHL_TX_DBG_ERR("SPI write block failed\n");
		ret = -EIO;
	} else {
		ret = 0;
	}

	return ret;
}

int mhl_tx_write_reg(void *drv_context, u16 address, u8 value)
{
	u8 page = (u8)(address >> 8);
	u8 offset = (u8)address;

	if (use_spi)
		return mhl_tx_write_reg_spi(drv_context, page, offset, value);
	else
		return mhl_tx_write_reg_i2c(drv_context, page, offset, value);
}

int mhl_tx_read_reg_block(void *drv_context, u16 address, u16 count, u8 *values)
{
	u8 page = (u8)(address >> 8);
	u8 offset = (u8)address;

	if (use_spi)
		return mhl_tx_read_reg_block_spi(drv_context, page, offset,
						 count, values);
	else
		return mhl_tx_read_reg_block_i2c(drv_context, page, offset,
						 count, values);
}

int mhl_tx_read_reg(void *drv_context, u16 address)
{
	u8 page = (u8)(address >> 8);
	u8 offset = (u8)address;

	if (use_spi)
		return mhl_tx_read_reg_spi(drv_context, page, offset);
	else
		return mhl_tx_read_reg_i2c(drv_context, page, offset);
}

int mhl_tx_modify_reg(void *drv_context, u16 address, u8 mask, u8 value)
{
	int reg_value;
	int write_status;

	reg_value = mhl_tx_read_reg(drv_context, address);
	if (reg_value < 0)
		return reg_value;

	reg_value &= ~mask;
	reg_value |= mask & value;

	write_status = mhl_tx_write_reg(drv_context, address, reg_value);

	if (write_status < 0)
		return write_status;
	else
		return reg_value;
}

enum hpd_control_mode platform_get_hpd_control_mode(void)
{
	return HPD_CTRL_OPEN_DRAIN;
}

static int si_8620_add_i2c(struct i2c_client *client)
{
	struct i2c_adapter *adapter = client->adapter;
	int idx;

	
	i2c_bus_adapter = adapter;
	if (i2c_bus_adapter == NULL) {
		pr_err("%s() failed to get i2c adapter\n", __func__);
		return ENODEV;
	}

	for (idx = 0; idx < ARRAY_SIZE(device_addresses); idx++) {
		if(idx == 0)
			device_addresses[idx].client = client;
		else {
			device_addresses[idx].client = i2c_new_dummy(i2c_bus_adapter,
				device_addresses[idx].dev_addr);
			if (device_addresses[idx].client == NULL){
				return ENODEV;
			}
		}
	}

	return 0;
}

static int mhl_get_power_dt_data(struct device *dev)
{
	int rc = 0;

	drv_info.avcc_1_vreg = devm_regulator_get(dev, "avcc_1v");
	if (IS_ERR(drv_info.avcc_1_vreg)) {
		pr_err("%s: could not get avcc_1v reg, rc=%ld\n",
			__func__, PTR_ERR(drv_info.avcc_1_vreg));
		return PTR_ERR(drv_info.avcc_1_vreg);
	}
	rc = regulator_set_voltage(drv_info.avcc_1_vreg, 1000000,
		1000000);
	if (rc) {
		pr_err("%s: set voltage failed on avcc_1v vreg, rc=%d\n",
			__func__, rc);
		return rc;
	}

	return 0;
}

static int mhl_power_on(void)
{
	int rc = 0;

	if (!IS_ERR(drv_info.avcc_1_vreg)) {
		rc = regulator_enable(drv_info.avcc_1_vreg);
		if (rc) {
			pr_err("%s: Failed to enable avcc_1_vreg regulator.\n",
				__func__);
			return rc;
		}
	}
	msleep(10);

	return rc;
}

static int mhl_get_gpio_dt_data(struct device *dev)
{
	struct device_node *of_node = dev->of_node;

	drv_info.dpdn_pin = of_get_named_gpio(of_node, "mhl-dpdn-gpio", 0);
	if (!gpio_is_valid(drv_info.dpdn_pin)) {
		pr_err("%s: Can't get mhl-dpdn-gpio\n", __func__);
		return -EINVAL;
	} else {
		platform_signals[SWITCH_SEL1].gpio_number = drv_info.dpdn_pin;
		gpio_request_one(drv_info.dpdn_pin, GPIOF_OUT_INIT_LOW, NULL);
		gpio_direction_output(drv_info.dpdn_pin, 0);
		gpio_set_value_cansleep(drv_info.dpdn_pin, 0);
	}

	drv_info.dpdn_pin_ = of_get_named_gpio(of_node, "mhl-dpdn-gpio_", 0);
	if (!gpio_is_valid(drv_info.dpdn_pin_)) {
		pr_err("%s: Can't get mhl-dpdn-gpio_\n", __func__);
		return -EINVAL;
	} else {
		platform_signals[SWITCH_SEL2].gpio_number = drv_info.dpdn_pin_;
		gpio_request_one(drv_info.dpdn_pin_, GPIOF_OUT_INIT_LOW, NULL);
		gpio_direction_output(drv_info.dpdn_pin_, 0);
		gpio_set_value_cansleep(drv_info.dpdn_pin_, 0);
	}

	drv_info.reset_pin = of_get_named_gpio(of_node, "mhl-rst-gpio", 0);
	if (!gpio_is_valid(drv_info.reset_pin)) {
		pr_err("%s: Can't get mhl-rst-gpio\n", __func__);
		return -EINVAL;
	} else {
		pr_info("[MHL]rst_gpio[%d]=%d,PwrOn Pull_High", drv_info.reset_pin,
			gpio_get_value_cansleep(drv_info.reset_pin));
		gpio_request_one(drv_info.reset_pin, GPIOF_OUT_INIT_HIGH, NULL);
		gpio_direction_output(drv_info.reset_pin, 1);
		gpio_set_value_cansleep(drv_info.reset_pin, 1);
		msleep(100); 
	}

	drv_info.intr_pin = of_get_named_gpio(of_node, "mhl-intr-gpio", 0);
	if (!gpio_is_valid(drv_info.intr_pin)) {
		pr_err("%s: Can't get mhl-intr-gpio\n", __func__);
		return -EINVAL;
	} else {
		gpio_request_one(drv_info.intr_pin, GPIOF_IN, NULL);
		gpio_direction_input(drv_info.intr_pin);
	}

	drv_info.fw_wake_pin = of_get_named_gpio(of_node, "mhl-fw-wake-gpio", 0);
	if (!gpio_is_valid(drv_info.fw_wake_pin)) {
		pr_err("%s: Can't get mhl-fw-wake-gpio\n", __func__);
		return -EINVAL;
	} else {
		platform_signals[TX_FW_WAKE].gpio_number = drv_info.fw_wake_pin;
		gpio_request_one(drv_info.fw_wake_pin, GPIOF_OUT_INIT_LOW, NULL);
		gpio_direction_output(drv_info.fw_wake_pin, 0);
	}

	return 0;
}

static int mhl_get_dt_data(struct device *dev)
{
	int rc = 0;
	struct device_node *of_node = NULL;
	struct device_node *hdmi_tx_node = NULL;

	if (!dev) {
		pr_err("%s: invalid input\n", __func__);
		return -EINVAL;
	}

	of_node = dev->of_node;
	if (!of_node) {
		pr_err("%s: invalid of_node\n", __func__);
		return -EINVAL;
	}

	rc = mhl_get_power_dt_data(dev);
	if (rc < 0) {
		pr_err("%s: mhl_get_power_dt_data fail\n", __func__);
		return -EINVAL;
	}

	rc = mhl_power_on();
	if (rc < 0) {
		pr_err("%s: mhl_power_on fail\n", __func__);
		return -EINVAL;
	}

	rc = mhl_get_gpio_dt_data(dev);
	if (rc < 0) {
		pr_err("%s: mhl_get_gpio_dt_data fail\n", __func__);
		return -EINVAL;
	}

	
	hdmi_tx_node = of_parse_phandle(of_node, "qcom,hdmi-tx-map", 0);
	if (!hdmi_tx_node) {
		pr_err("%s: can't find hdmi phandle\n", __func__);
		return -EINVAL;
	}

	drv_info.hdmi_pdev = of_find_device_by_node(hdmi_tx_node);
	if (!drv_info.hdmi_pdev) {
		pr_err("%s: can't find the device by node\n", __func__);
		return -EINVAL;
	}
	pr_debug("%s: hdmi_pdev [0x%lx] to pdata->pdev\n",
		__func__, (unsigned long int)drv_info.hdmi_pdev);

	return 0;
} 

void si_wakeup_mhl(void)
{
	pr_info("%s\n", __func__);
	gpio_set_value_cansleep(drv_info.dpdn_pin, 1);
	gpio_set_value_cansleep(drv_info.dpdn_pin_, 1);
}

void enable_hdmi(int enable)
{
	if (drv_info.hdmi_pdev && drv_info.hdmi_mhl_ops) {
		drv_info.hdmi_mhl_ops->set_upstream_hpd(
		drv_info.hdmi_pdev, enable);
		gpio_set_value_cansleep(drv_info.dpdn_pin, enable);
		gpio_set_value_cansleep(drv_info.dpdn_pin_, enable);
	}
}

static int si_8620_mhl_tx_i2c_probe(struct i2c_client *client,
					      const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	int ret;

	if (!adapter ||
		!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		pr_info("[ERROR] %s() i2c no function\n", __func__);
		ret = -EIO;
		goto done;
	}
	i2c_mem.block_tx_buffers = kmalloc(MAX_I2C_EMSC_BLOCK_SIZE *
		NUM_BLOCK_QUEUE_REQUESTS, GFP_KERNEL);
	if (NULL == i2c_mem.block_tx_buffers) {
		ret = -ENOMEM;
		goto done;
	}

	ret = si_8620_add_i2c(client);
	if(ret < 0) {
		pr_err("si_8240_8558_add_i2c failed, error code %d\n", ret);
		return ret;
	}

	ret = mhl_get_dt_data(&client->dev);
	if(ret < 0) {
		pr_err("mhl_get_dt_data failed, error code %d\n", ret);
		return ret;
	}

	drv_info.hdmi_mhl_ops = devm_kzalloc(&client->dev,
		sizeof(struct msm_hdmi_mhl_ops),
		GFP_KERNEL);

	if (!drv_info.hdmi_mhl_ops) {
		pr_err("%s: alloc hdmi mhl ops failed\n", __func__);
		return -ENOMEM;
	}

	if (drv_info.hdmi_pdev) {
		ret = msm_hdmi_register_mhl(drv_info.hdmi_pdev,
			drv_info.hdmi_mhl_ops, NULL);
		if (ret) {
			pr_err("%s: register with hdmi failed\n", __func__);
			return -EPROBE_DEFER;
		}
	}

	ret = drv_info.hdmi_mhl_ops->set_mhl_max_pclk(
		drv_info.hdmi_pdev, MAX_MHL_PCLK);
	if (ret) {
		pr_err("%s: can't set max mhl pclk\n", __func__);
		return -EPERM;
	}

	ret = mhl_tx_init(&drv_info, client);
	if (ret) {
		pr_err("%s(): mhl_tx_init failed, error code %d\n",
			__func__, ret);
	}
done:
	if (ret != 0)
		probe_fail = true;
	return ret;
}

static int si_8620_mhl_tx_remove(struct i2c_client *client)
{
	if (!use_spi)
		kfree(i2c_mem.block_tx_buffers);

	return 0;
}

int si_8620_pm_suspend(struct device *dev)
{
	int status;

	status = down_interruptible(&platform_lock);
	if (status)
		goto done;

	status = mhl_handle_power_change_request(dev, false);
	
	
	

	up(&platform_lock);
done:
	return status;
}

int si_8620_pm_resume(struct device *dev)
{
	int status;

	status = down_interruptible(&platform_lock);
	if (status)
		goto done;

	
	
	
	status = mhl_handle_power_change_request(dev, true);

	up(&platform_lock);
done:
	return status;
}

int si_8620_power_control(bool power_up)
{
	struct device *dev = NULL;
	int status;

	if (use_spi)
		dev = &spi_dev->dev;
	else
		dev = &device_addresses[0].client->dev;

	if (power_up)
		status = si_8620_pm_resume(dev);
	else
		status = si_8620_pm_suspend(dev);
	return status;
}
EXPORT_SYMBOL_GPL(si_8620_power_control);

int si_8620_get_hpd_status(int *hpd_status)
{
	struct device *dev = NULL;
	int status = 0;
	struct mhl_dev_context *dev_context;

	if (use_spi)
		dev = &spi_dev->dev;
	else
		dev = &device_addresses[0].client->dev;

	dev_context = dev_get_drvdata(dev);

	if (down_interruptible(&dev_context->isr_lock)) {
		MHL_TX_DBG_ERR("%scouldn't acquire mutex%s\n",
			ANSI_ESC_RED_TEXT, ANSI_ESC_RESET_TEXT);
		return  -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR("%sshutting down%s\n",
			ANSI_ESC_YELLOW_TEXT, ANSI_ESC_RESET_TEXT);
		status = -ENODEV;
	} else {
		*hpd_status = si_mhl_tx_drv_get_hpd_status(dev_context);
		MHL_TX_DBG_INFO("HPD status: %d\n",
			*hpd_status);
	}
	up(&dev_context->isr_lock);
	return status;
}
EXPORT_SYMBOL_GPL(si_8620_get_hpd_status);

int si_8620_get_hdcp2_status(uint32_t *hdcp2_status)
{
	struct device *dev = NULL;
	int status = 0;
	struct mhl_dev_context *dev_context;

	if (use_spi)
		dev = &spi_dev->dev;
	else
		dev = &device_addresses[0].client->dev;

	dev_context = dev_get_drvdata(dev);

	if (down_interruptible(&dev_context->isr_lock)) {
		MHL_TX_DBG_ERR("%scouldn't acquire mutex%s\n",
			ANSI_ESC_RED_TEXT, ANSI_ESC_RESET_TEXT);
		return  -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR("%sshutting down%s\n",
			ANSI_ESC_YELLOW_TEXT, ANSI_ESC_RESET_TEXT);
		status = -ENODEV;
	} else {
		*hdcp2_status = si_mhl_tx_drv_get_hdcp2_status(dev_context);
		MHL_TX_DBG_INFO("HDCP2 status: 0x%8x\n",
			*hdcp2_status);
	}
	up(&dev_context->isr_lock);
	return status;
}
EXPORT_SYMBOL_GPL(si_8620_get_hdcp2_status);

static const struct i2c_device_id si_8620_mhl_tx_id[] = {
	{MHL_DEVICE_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, si_8620_mhl_tx_id);

static const struct dev_pm_ops si_8620_tx_pm_ops = {
	.runtime_suspend = si_8620_pm_suspend,
	.runtime_resume = si_8620_pm_resume,
};

static struct of_device_id mhl_match_table[] = {
	{.compatible = COMPATIBLE_NAME,},
	{ },
};

static struct i2c_driver si_8620_mhl_tx_i2c_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = MHL_DRIVER_NAME,
		   .of_match_table = mhl_match_table,
		   .pm = &si_8620_tx_pm_ops,
		   },
	.id_table = si_8620_mhl_tx_id,
	.probe = si_8620_mhl_tx_i2c_probe,
	.remove = si_8620_mhl_tx_remove,
	.command = NULL,
};

static int si_8620_mhl_tx_spi_probe(struct spi_device *spi)
{
	int ret;

	pr_info("%s(), spi = %p\n", __func__, spi);
	spi->bits_per_word = 8;
	spi_dev = spi;

	spi_mem.tx_buf = kmalloc(MAX_SPI_XFER_BUFFER_SIZE, GFP_KERNEL);
	spi_mem.rx_buf = kmalloc(MAX_SPI_XFER_BUFFER_SIZE, GFP_KERNEL);
	spi_mem.block_tx_buffers = kmalloc(MAX_SPI_EMSC_BLOCK_SIZE *
		NUM_BLOCK_QUEUE_REQUESTS, GFP_KERNEL);
	if (!spi_mem.tx_buf || !spi_mem.rx_buf || !spi_mem.block_tx_buffers) {
		ret = -ENOMEM;
		goto failed;
	}
	
	if (ret) {
		pr_err("%s(): mhl_tx_init failed, error code %d\n",
		       __func__, ret);
		goto failed;
	}
	goto done;

failed:
	kfree(spi_mem.tx_buf);
	spi_mem.tx_buf = NULL;

	kfree(spi_mem.rx_buf);
	spi_mem.rx_buf = NULL;

	kfree(spi_mem.block_tx_buffers);
	spi_mem.block_tx_buffers = NULL;

	probe_fail = true;

done:
	return ret;
}

static int si_8620_mhl_spi_remove(struct spi_device *spi_dev)
{
	pr_info("%s() called\n", __func__);

	kfree(spi_mem.tx_buf);
	kfree(spi_mem.rx_buf);
	kfree(spi_mem.block_tx_buffers);
	return 0;
}

static struct spi_driver si_86x0_mhl_tx_spi_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = MHL_DRIVER_NAME,
		   .pm = &si_8620_tx_pm_ops,
		   },
	.probe = si_8620_mhl_tx_spi_probe,
	.remove = si_8620_mhl_spi_remove,
};

static struct spi_board_info __initdata si_86x0_spi_board_info = {
	.modalias = MHL_DRIVER_NAME,
	.max_speed_hz = SPI_BUS_SPEED,
	.bus_num = SPI_BUS_NUM,
	.chip_select = SPI_CHIP_SEL,
	.mode = SPI_TRANSFER_MODE,
	.irq = -1
};

static int __init add_spi_device_to_bus(void)
{
	struct spi_master *spi_master;
	int status = 0;

	pr_info("add_spi_device_to_bus called\n");

	spi_master = spi_busnum_to_master(SPI_BUS_NUM);
	if (spi_master == NULL) {
		pr_err("spi_busnum_to_master(%d) returned NULL\n", SPI_BUS_NUM);
		return -1;
	}

	spi_dev = spi_new_device(spi_master, &si_86x0_spi_board_info);
	if (spi_dev == NULL || probe_fail) {
		pr_err("spi_new_device() failed\n");
		if (spi_dev)
			spi_unregister_device(spi_dev);
		status = -1;
		goto exit;
	}

exit:
	put_device(&spi_master->dev);
	return status;
}

static int __init spi_init(void)
{
	int status;

	status = spi_register_driver(&si_86x0_mhl_tx_spi_driver);
	if (status < 0) {
		pr_err("[ERROR] %s():%d failed !\n", __func__, __LINE__);
		goto exit;
	}

	status = add_spi_device_to_bus();
	if (status < 0) {
		pr_err("Failed to add MHL transmitter as SPI device\n");
		spi_unregister_driver(&si_86x0_mhl_tx_spi_driver);
	}

exit:
	if (probe_fail)
		status = -ENODEV;

	return status;
}

static int __init i2c_init(void)
{
	int ret = -EFAULT;

	ret = i2c_add_driver(&si_8620_mhl_tx_i2c_driver);
	if (ret < 0 || probe_fail) {
		if (ret == 0)
			i2c_del_driver(&si_8620_mhl_tx_i2c_driver);
		pr_err
		    ("[ERROR] %s():%d failed !\n\nCHECK POWER AND CONNECTION "
		    "TO CP8620 Starter Kit.\n\n", __func__, __LINE__);
	}

	MHL_TX_DBG_INFO("returning %d\n", ret);
	return ret;
}

static int __init si_8620_driver_init(bool use_spi)
{
	if (use_spi)
		return spi_init();
	else
		return i2c_init();
}

static void __init si_8620_init_async(void *unused, async_cookie_t cookie)
{
	si_8620_driver_init(use_spi);
}
static int __init si_8620_init(void)
{
	pr_info("mhl: Starting SiI8620 Driver v%s ++\n",
		buildVersion);
	pr_info("mhl: %s\n", buildTime);

	sema_init(&platform_lock, 1);

	platform_flags &= ~PLATFORM_FLAG_HEARTBEAT_MASK;
	switch (use_heartbeat) {
	case 0:
		
		break;
	case 1:
		platform_flags |= PLATFORM_VALUE_ISSUE_HEARTBEAT;
		break;
	case 2:
		platform_flags |= PLATFORM_VALUE_DISCONN_HEARTBEAT;
		break;
	default:
		MHL_TX_DBG_ERR("%sinvalid use_heartbeat parameter%s\n",
			       ANSI_ESC_RED_TEXT, ANSI_ESC_RESET_TEXT);
	}

	if (tmds_link_speed == 15)
		platform_flags |= PLATFORM_FLAG_1_5GBPS;
	else if (tmds_link_speed == 3)
		platform_flags |= PLATFORM_FLAG_3GBPS;
	else if (tmds_link_speed == 6)
		platform_flags |= PLATFORM_FLAG_6GBPS;

	async_schedule(si_8620_init_async, NULL);

	return 0;
}

static void __exit si_8620_exit(void)
{
	int idx;

	pr_info("si_8620_exit called\n");

	si_8620_power_control(false);

	if (use_spi) {
		mhl_tx_remove(&spi_dev->dev);
		spi_unregister_driver(&si_86x0_mhl_tx_spi_driver);
		spi_unregister_device(spi_dev);
	} else {
		mhl_tx_remove(&device_addresses[0].client->dev);
		MHL_TX_DBG_INFO("client removed\n");
		i2c_del_driver(&si_8620_mhl_tx_i2c_driver);
		MHL_TX_DBG_INFO("i2c driver deleted from context\n");

		for (idx = 0; idx < ARRAY_SIZE(device_addresses); idx++) {
			MHL_TX_DBG_INFO("\n");
			if (device_addresses[idx].client != NULL) {
				MHL_TX_DBG_INFO("unregistering device:%p\n",
						device_addresses[idx].client);
				i2c_unregister_device(device_addresses[idx].
						      client);
			}
		}
	}
	MHL_TX_DBG_ERR("driver unloaded.\n");
}

static int debug_level_stack[15];
static unsigned int debug_level_stack_ptr;

void push_debug_level(int new_verbosity)
{
	if (debug_level_stack_ptr < ARRAY_SIZE(debug_level_stack)) {
		
		debug_level_stack[debug_level_stack_ptr++] = debug_level;
		debug_level = new_verbosity;
	} else {
		MHL_TX_DBG_ERR("%sdebug_level_stack overflowed%s\n",
			       ANSI_ESC_RED_TEXT, ANSI_ESC_RESET_TEXT);
	}
}

void pop_debug_level(void)
{
	if (debug_level_stack_ptr > 0) {
		if (debug_level_stack_ptr > ARRAY_SIZE(debug_level_stack)) {
			MHL_TX_DBG_ERR("%sdebug_level_stack overflowed%s\n",
				       ANSI_ESC_RED_TEXT, ANSI_ESC_RESET_TEXT);
		} else {
			debug_level =
			    debug_level_stack[--debug_level_stack_ptr];
		}
	}
}

int si_8620_register_callbacks(struct si_mhl_callback_api_t *p_callbacks)
{
	struct device *dev = NULL;
	int status = 0;
	struct mhl_dev_context *dev_context;
	struct drv_hw_context *hw_context;

	if (use_spi)
		dev = &spi_dev->dev;
	else
		dev = &device_addresses[0].client->dev;

	dev_context = dev_get_drvdata(dev);
	hw_context = (struct drv_hw_context *)&dev_context->drv_context;

	if (down_interruptible(&dev_context->isr_lock))
		return -ERESTARTSYS;

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		status = -ENODEV;
	} else {
		if (NULL != p_callbacks)
			hw_context->callbacks = *p_callbacks;
	}

	up(&dev_context->isr_lock);
	return status;
}
EXPORT_SYMBOL(si_8620_register_callbacks);

int si_8620_info_frame_change(enum hpd_high_callback_status mode_parm,
	union avif_or_cea_861_dtd_u *p_avif_or_dtd,
	size_t avif_or_dtd_max_length, union vsif_mhl3_or_hdmi_u *p_vsif,
	size_t vsif_max_length)
{
	struct device *dev = NULL;
	int status;
	struct mhl_dev_context *dev_context;
	struct drv_hw_context *hw_context;

	if (use_spi)
		dev = &spi_dev->dev;
	else
		dev = &device_addresses[0].client->dev;

	dev_context = dev_get_drvdata(dev);
	hw_context = (struct drv_hw_context *)&dev_context->drv_context;

	if (down_interruptible(&dev_context->isr_lock))
		return -ERESTARTSYS;

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		status = -ENODEV;
	} else {
		size_t xfer_size;

		memset(&hw_context->vsif_mhl3_or_hdmi_from_callback, 0,
			sizeof(hw_context->vsif_mhl3_or_hdmi_from_callback));
		memset(&hw_context->avif_or_dtd_from_callback, 0,
			sizeof(hw_context->avif_or_dtd_from_callback));

		if (sizeof(hw_context->vsif_mhl3_or_hdmi_from_callback) <
			vsif_max_length) {
			xfer_size = sizeof(
				hw_context->vsif_mhl3_or_hdmi_from_callback);
		} else {
			xfer_size = vsif_max_length;
		}
		memcpy(&hw_context->vsif_mhl3_or_hdmi_from_callback, p_vsif,
			xfer_size);

		if (sizeof(hw_context->avif_or_dtd_from_callback) <
			avif_or_dtd_max_length) {
			xfer_size = sizeof(
				hw_context->avif_or_dtd_from_callback);
		} else {
			xfer_size = avif_or_dtd_max_length;
		}
		memcpy(&hw_context->avif_or_dtd_from_callback, p_avif_or_dtd,
			xfer_size);

		status = si_mhl_tx_drv_set_display_mode(dev_context, mode_parm);
	}

	up(&dev_context->isr_lock);
	return status;
}
EXPORT_SYMBOL(si_8620_info_frame_change);

module_init(si_8620_init);
module_exit(si_8620_exit);

MODULE_DESCRIPTION("Silicon Image MHL Transmitter driver");
MODULE_AUTHOR("Silicon Image <http://www.siliconimage.com>");
MODULE_LICENSE("GPL");
