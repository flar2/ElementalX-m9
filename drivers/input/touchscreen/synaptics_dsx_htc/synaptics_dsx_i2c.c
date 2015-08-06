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
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/types.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/input/synaptics_dsx_htc.h>
#include "synaptics_dsx_core.h"

#define SYN_I2C_RETRY_TIMES 10

#ifdef CONFIG_OF
static uint32_t parse_eng_id(uint32_t eng_id_mask){
	uint32_t eng_id = 0;
	uint8_t temp[4] = {0};
	struct device_node *devnode = of_find_node_by_path("/chosen/mfg");

	if (devnode) {
		if (of_property_read_u8_array(devnode, "skuid.engineer_id",
					temp, sizeof(uint32_t)/sizeof(uint8_t)))
			pr_err(" %s, Failed to get property: engineer_id", __func__);
	} else {
		pr_err(" %s, Failed to find device node", __func__);
		return 0;
	}

	eng_id = temp[3] << 24 | temp[2] << 16 | temp[1] << 8 | temp[0];
	pr_debug(" %s, eng_id_mask = 0x%8X, temp = 0x%2X%2X%2X%2X, eng_id = 0x%X",
			__func__, eng_id_mask, temp[0], temp[1], temp[2], temp[3], eng_id);

	eng_id &= eng_id_mask;
	pr_info(" %s, eng_id = 0x%X", __func__, eng_id);
	return eng_id;
}

static int parse_config(struct device *dev, struct synaptics_dsx_board_data *bdata)
{
	struct synaptics_rmi4_config *cfg_table;
	struct device_node *node, *pp = NULL;
	struct property *prop;
	uint8_t cnt = 0, i = 0;
	u32 data = 0;
	int len = 0, size = 0;

	pr_info(" %s\n", __func__);
	node = dev->of_node;
	if (node == NULL) {
		pr_err(" %s, can't find device_node", __func__);
		return -ENODEV;
	}

	while ((pp = of_get_next_child(node, pp)))
		cnt++;

	if (!cnt)
		return -ENODEV;

	cfg_table = kzalloc(cnt * (sizeof *cfg_table), GFP_KERNEL);
	if (!cfg_table)
		return -ENOMEM;

	pp = NULL;
	while ((pp = of_get_next_child(node, pp))) {
		if (of_property_read_u32(pp, "sensor_id", &data) == 0)
			cfg_table[i].sensor_id = (data | SENSOR_ID_CHECKING_EN);

		if (of_property_read_u32(pp, "pr_number", &data) == 0)
			cfg_table[i].pr_number = data;

		if (of_property_read_u32(pp, "eng_id", &data) == 0)
			cfg_table[i].eng_id = data;
		else
			cfg_table[i].eng_id = 0;

		if (bdata->support_cover) {
			prop = of_find_property(pp, "cover_setting", &size);
			if (prop) {
				memcpy(cfg_table[i].cover_setting, prop->value, size);
				cfg_table[i].cover_setting_size = size;
			} else
				cfg_table[i].cover_setting_size = 0;
			prop = of_find_property(pp, "uncover_setting", &size);
			if (prop) {
				memcpy(cfg_table[i].uncover_setting, prop->value, size);
			}

			if (bdata->support_glove) {
				prop = of_find_property(pp, "glove_setting", &size);
				if (prop) {
					memcpy(cfg_table[i].glove_mode_setting, prop->value, size);
				}
			}
		}

		prop = of_find_property(pp, "config", &len);
		if (!prop) {
			pr_err(" %s:Looking up %s property in node %s failed",
				__func__, "config", pp->full_name);
			return -ENODEV;
		} else if (!len) {
			pr_err(" %s:Invalid length of configuration data\n",
				__func__);
			return -EINVAL;
		}

		cfg_table[i].length = len;
		memcpy(cfg_table[i].config, prop->value, cfg_table[i].length);
		i++;
	}

	bdata->config_num = cnt;
	bdata->config_table = cfg_table;

	return 0;
}

static int parse_dt(struct device *dev, struct synaptics_dsx_board_data *bdata)
{
	int retval, coords_size;
	u32 value;
	const char *name;
	struct property *prop;
	struct device_node *np = dev->of_node;
	uint32_t coords[4] = {0};

	pr_info("%s\n", __func__);
	bdata->irq_gpio = of_get_named_gpio_flags(np,
			"synaptics,irq-gpio", 0, NULL);

	pr_info("%s: irq_gpio = %d\n", __func__, bdata->irq_gpio);

	retval = of_property_read_u32(np, "synaptics,irq-flags", &value);
	if (retval < 0)
		return retval;
	else
		bdata->irq_flags = value;

	retval = of_property_read_string(np, "synaptics,pwr-reg-name", &name);
	if (retval == -EINVAL)
		bdata->pwr_reg_name = NULL;
	else if (retval < 0)
		return retval;
	else
		bdata->pwr_reg_name = name;

	retval = of_property_read_string(np, "synaptics,bus-reg-name", &name);
	if (retval == -EINVAL)
		bdata->bus_reg_name = NULL;
	else if (retval < 0)
		return retval;
	else
		bdata->bus_reg_name = name;

	if (of_property_read_bool(np, "synaptics,power-gpio")) {
		bdata->power_gpio = of_get_named_gpio_flags(np,
				"synaptics,power-gpio", 0, NULL);
		retval = of_property_read_u32(np, "synaptics,power-on-state",
				&value);
		if (retval < 0)
			return retval;
		else
			bdata->power_on_state = value;
	} else {
		bdata->power_gpio = -1;
	}

	if (of_property_read_bool(np, "synaptics,power-delay-ms")) {
		retval = of_property_read_u32(np, "synaptics,power-delay-ms",
				&value);
		if (retval < 0)
			return retval;
		else
			bdata->power_delay_ms = value;
	} else {
		bdata->power_delay_ms = 0;
	}

	if (of_property_read_bool(np, "synaptics,reset-gpio")) {
		bdata->reset_gpio = of_get_named_gpio_flags(np,
				"synaptics,reset-gpio", 0, NULL);
		retval = of_property_read_u32(np, "synaptics,reset-on-state",
				&value);
		if (retval < 0)
			return retval;
		else
			bdata->reset_on_state = value;
		retval = of_property_read_u32(np, "synaptics,reset-active-ms",
				&value);
		if (retval < 0)
			return retval;
		else
			bdata->reset_active_ms = value;
	} else {
		bdata->reset_gpio = -1;
	}

	pr_info("%s: reset_gpio = %d\n", __func__, bdata->reset_gpio);

	if (of_property_read_bool(np, "synaptics,reset-delay-ms")) {
		retval = of_property_read_u32(np, "synaptics,reset-delay-ms",
				&value);
		if (retval < 0)
			return retval;
		else
			bdata->reset_delay_ms = value;
	} else {
		bdata->reset_delay_ms = 0;
	}

	if (of_property_read_bool(np, "synaptics,switch-gpio")) {
		bdata->switch_gpio = of_get_named_gpio_flags(np,
				"synaptics,switch-gpio", 0, NULL);
	} else {
		bdata->switch_gpio = -1;
	}
	pr_info("%s: switch_gpio = %d\n", __func__, bdata->switch_gpio);

	bdata->swap_axes = of_property_read_bool(np, "synaptics,swap-axes");

	bdata->x_flip = of_property_read_bool(np, "synaptics,x-flip");

	bdata->y_flip = of_property_read_bool(np, "synaptics,y-flip");

	prop = of_find_property(np, "synaptics,cap-button-map", NULL);
	if (prop && prop->length) {
		bdata->cap_button_map->map = devm_kzalloc(dev,
				prop->length,
				GFP_KERNEL);
		if (!bdata->cap_button_map->map)
			return -ENOMEM;
		bdata->cap_button_map->nbuttons = prop->length / sizeof(u32);
		retval = of_property_read_u32_array(np,
				"synaptics,cap-button-map",
				bdata->cap_button_map->map,
				bdata->cap_button_map->nbuttons);
		if (retval < 0) {
			bdata->cap_button_map->nbuttons = 0;
			bdata->cap_button_map->map = NULL;
		}
	} else {
		bdata->cap_button_map->nbuttons = 0;
		bdata->cap_button_map->map = NULL;
	}

	if (of_property_read_bool(np, "synaptics,tw-pin-mask")) {
		retval = of_property_read_u32(np, "synaptics,tw-pin-mask",
				&value);
		if (retval < 0)
			return retval;
		else
			bdata->tw_pin_mask = value;
	} else {
		bdata->tw_pin_mask = 0;
	}

	if (of_property_read_bool(np, "synaptics,update-feature")) {
		retval = of_property_read_u32(np, "synaptics,update-feature",
				&value);
		if (retval < 0)
			return retval;
		else
			bdata->update_feature = value;
	} else {
		bdata->update_feature = 0;
	}

	if (of_property_read_bool(np, "synaptics,support-glove")) {
		retval = of_property_read_u32(np, "synaptics,support-glove",
				&value);
		if (retval < 0)
			return retval;
		else
			bdata->support_glove = value;
	} else {
		bdata->support_glove = 0;
	}

	if (of_property_read_bool(np, "synaptics,support-cover")) {
		retval = of_property_read_u32(np, "synaptics,support-cover",
				&value);
		if (retval < 0)
			return retval;
		else
			bdata->support_cover = value;
	} else {
		bdata->support_cover = 0;
	}

	if (of_property_read_bool(np, "synaptics,hall-block-time")) {
		retval = of_property_read_u32(np, "synaptics,hall-block-time",
				&value);
		if (retval < 0)
			return retval;
		else
			bdata->hall_block_touch_time = value;
	} else {
		bdata->hall_block_touch_time = 0;
	}

	prop = of_find_property(np, "synaptics,display-coords", NULL);
	if (prop) {
		coords_size = prop->length / sizeof(u32);
		if (coords_size != 4)
			pr_info("%s:Invalid display coords size %d", __func__, coords_size);

		retval = of_property_read_u32_array(np, "synaptics,display-coords", coords, coords_size);
		if (retval && (retval != -EINVAL)) {
			pr_err("%s:Fail to read display-coords %d\n", __func__, retval);
			return retval;
		}
		bdata->display_width  = coords[1];
		bdata->display_height = coords[3];
		
	}

	
	if (of_property_read_u32(np, "htc,eng_id", &bdata->eng_id) == 0) {
		pr_info("(INIT) eng_id = %d", bdata->eng_id);
	} else if (of_property_read_u32(np, "htc,eng_id_mask", &bdata->eng_id_mask) == 0) {
		bdata->eng_id = parse_eng_id(bdata->eng_id_mask);
		pr_info("(INIT) parse eng_id = %d", bdata->eng_id);
	} else {
		bdata->eng_id = 0;
	}

	parse_config(dev, bdata);

	return 0;
}
#endif

static int synaptics_rmi4_i2c_set_page(struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr)
{
	int retval;
	unsigned char retry;
	unsigned char buf[PAGE_SELECT_LEN];
	unsigned char page;
	struct i2c_client *i2c = to_i2c_client(rmi4_data->pdev->dev.parent);

	page = ((addr >> 8) & MASK_8BIT);
	if (page != rmi4_data->current_page) {
		buf[0] = MASK_8BIT;
		buf[1] = page;
		for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
			retval = i2c_master_send(i2c, buf, PAGE_SELECT_LEN);
			if (retval != PAGE_SELECT_LEN) {
				dev_dbg(rmi4_data->pdev->dev.parent,
						"%s: I2C retry %d\n",
						__func__, retry + 1);
				msleep(20);
			} else {
				rmi4_data->current_page = page;
				break;
			}
		}
	} else {
		retval = PAGE_SELECT_LEN;
	}

	return retval;
}

static int synaptics_rmi4_i2c_read(struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr, unsigned char *data, unsigned short length)
{
	int retval;
	unsigned char retry;
	unsigned char buf;
	struct i2c_client *i2c = to_i2c_client(rmi4_data->pdev->dev.parent);
	struct i2c_msg msg[] = {
		{
			.addr = i2c->addr,
			.flags = 0,
			.len = 1,
			.buf = &buf,
		},
		{
			.addr = i2c->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = data,
		},
	};

	buf = addr & MASK_8BIT;

	mutex_lock(&rmi4_data->rmi4_io_ctrl_mutex);

	retval = synaptics_rmi4_i2c_set_page(rmi4_data, addr);
	if (retval != PAGE_SELECT_LEN) {
		retval = -EIO;
		goto exit;
	}

	for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(i2c->adapter, msg, 2) == 2) {
			retval = length;
			break;
		}
		dev_dbg(rmi4_data->pdev->dev.parent,
				"%s: I2C retry %d\n",
				__func__, retry + 1);
		msleep(20);
	}

	if (retry == SYN_I2C_RETRY_TIMES) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: I2C read over retry limit\n",
				__func__);
		retval = -EIO;
	}

exit:
	mutex_unlock(&rmi4_data->rmi4_io_ctrl_mutex);

	return retval;
}

static int synaptics_rmi4_i2c_write(struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr, unsigned char *data, unsigned short length)
{
	int retval;
	unsigned char retry;
	unsigned char buf[length + 1];
	struct i2c_client *i2c = to_i2c_client(rmi4_data->pdev->dev.parent);
	struct i2c_msg msg[] = {
		{
			.addr = i2c->addr,
			.flags = 0,
			.len = length + 1,
			.buf = buf,
		}
	};

	mutex_lock(&rmi4_data->rmi4_io_ctrl_mutex);

	retval = synaptics_rmi4_i2c_set_page(rmi4_data, addr);
	if (retval != PAGE_SELECT_LEN) {
		retval = -EIO;
		goto exit;
	}

	buf[0] = addr & MASK_8BIT;
	memcpy(&buf[1], &data[0], length);

	for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(i2c->adapter, msg, 1) == 1) {
			retval = length;
			break;
		}
		dev_dbg(rmi4_data->pdev->dev.parent,
				"%s: I2C retry %d\n",
				__func__, retry + 1);
		msleep(20);
	}

	if (retry == SYN_I2C_RETRY_TIMES) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: I2C write over retry limit\n",
				__func__);
		retval = -EIO;
	}

exit:
	mutex_unlock(&rmi4_data->rmi4_io_ctrl_mutex);

	return retval;
}

static struct synaptics_dsx_bus_access bus_access = {
	.type = BUS_I2C,
	.read = synaptics_rmi4_i2c_read,
	.write = synaptics_rmi4_i2c_write,
};

static struct synaptics_dsx_hw_interface hw_if;

static struct platform_device *synaptics_dsx_i2c_device;

static void synaptics_rmi4_i2c_dev_release(struct device *dev)
{
	kfree(synaptics_dsx_i2c_device);

	return;
}

static int synaptics_rmi4_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *dev_id)
{
	int retval;

	pr_info("%s\n", __func__);
	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev,
				"%s: SMBus byte data commands not supported by host\n",
				__func__);
		return -EIO;
	}

	synaptics_dsx_i2c_device = kzalloc(
			sizeof(struct platform_device),
			GFP_KERNEL);
	if (!synaptics_dsx_i2c_device) {
		dev_err(&client->dev,
				"%s: Failed to allocate memory for synaptics_dsx_i2c_device\n",
				__func__);
		return -ENOMEM;
	}

#ifdef CONFIG_OF
	if (client->dev.of_node) {
		hw_if.board_data = devm_kzalloc(&client->dev,
				sizeof(struct synaptics_dsx_board_data),
				GFP_KERNEL);
		if (!hw_if.board_data) {
			dev_err(&client->dev,
					"%s: Failed to allocate memory for board data\n",
					__func__);
			return -ENOMEM;
		}
		hw_if.board_data->cap_button_map = devm_kzalloc(&client->dev,
				sizeof(struct synaptics_dsx_cap_button_map),
				GFP_KERNEL);
		if (!hw_if.board_data->cap_button_map) {
			dev_err(&client->dev,
					"%s: Failed to allocate memory for button map\n",
					__func__);
			return -ENOMEM;
		}
		parse_dt(&client->dev, hw_if.board_data);
	}
#else
	hw_if.board_data = client->dev.platform_data;
#endif
	hw_if.bus_access = &bus_access;

	synaptics_dsx_i2c_device->name = PLATFORM_DRIVER_NAME;
	synaptics_dsx_i2c_device->id = 0;
	synaptics_dsx_i2c_device->num_resources = 0;
	synaptics_dsx_i2c_device->dev.parent = &client->dev;
	synaptics_dsx_i2c_device->dev.platform_data = &hw_if;
	synaptics_dsx_i2c_device->dev.release = synaptics_rmi4_i2c_dev_release;

	retval = platform_device_register(synaptics_dsx_i2c_device);
	if (retval) {
		dev_err(&client->dev,
				"%s: Failed to register platform device\n",
				__func__);
		return -ENODEV;
	}

	return 0;
}

static int synaptics_rmi4_i2c_remove(struct i2c_client *client)
{
	platform_device_unregister(synaptics_dsx_i2c_device);

	return 0;
}

static const struct i2c_device_id synaptics_rmi4_id_table[] = {
	{I2C_DRIVER_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, synaptics_rmi4_id_table);

#ifdef CONFIG_OF
static struct of_device_id synaptics_rmi4_of_match_table[] = {
	{
		.compatible = "synaptics,dsx",
	},
	{},
};
MODULE_DEVICE_TABLE(of, synaptics_rmi4_of_match_table);
#else
#define synaptics_rmi4_of_match_table NULL
#endif

static struct i2c_driver synaptics_rmi4_i2c_driver = {
	.driver = {
		.name = I2C_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = synaptics_rmi4_of_match_table,
	},
	.probe = synaptics_rmi4_i2c_probe,
	.remove = synaptics_rmi4_i2c_remove,
	.id_table = synaptics_rmi4_id_table,
};

int synaptics_rmi4_bus_init(void)
{
	return i2c_add_driver(&synaptics_rmi4_i2c_driver);
}
EXPORT_SYMBOL(synaptics_rmi4_bus_init);

void synaptics_rmi4_bus_exit(void)
{
	i2c_del_driver(&synaptics_rmi4_i2c_driver);

	return;
}
EXPORT_SYMBOL(synaptics_rmi4_bus_exit);

MODULE_AUTHOR("Synaptics, Inc.");
MODULE_DESCRIPTION("Synaptics DSX I2C Bus Support Module");
MODULE_LICENSE("GPL v2");
