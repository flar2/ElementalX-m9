/*
 * Copyright (C) 2014 HTC, Inc.
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

#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/slab.h>

#include <htc_power_debug/htc_railways.h>

extern void htc_railways_dump_info(void);
extern struct htc_railways_info_base htc_railways_base;

static inline int htc_railways_append_data_to_buf(char *buf, htc_railways_info *data, int buflength, int idx)
{
	return snprintf(buf, buflength,
		"[%6s] %12u | %12u | %12u\n", htc_railways_rails[idx], data->volt_init, data->volt_min, data->volt_max);
}

static inline int htc_railways_copy_info(struct htc_railways_private_data *prvdata)
{
	htc_railways_info data;
	int i = 0, length = 0;

	length += snprintf(prvdata->buf, sizeof(prvdata->buf),
		"[%6s] %12s | %12s | %12s\n", "RAILS", "turbo init", "turbo min", "turbo max");

	for(i = 0; i < HTC_RAILWAY_INFO_MAX; i++) {
		data.volt_init = railways_info[i].volt_init;
		data.volt_max = railways_info[i].volt_max;
		data.volt_min = railways_info[i].volt_min;

		length += htc_railways_append_data_to_buf(prvdata->buf + length, &data, sizeof(prvdata->buf) - length, i);
		prvdata->read_idx++;
	}
	return length;
}

static ssize_t htc_railways_file_read(struct file *file, char __user *bufu, size_t count, loff_t *ppos)
{
	struct htc_railways_private_data *prvdata;

	prvdata = file->private_data;
	if(!prvdata)
		return -EINVAL;

	if(!bufu || count == 0)
		return -EINVAL;

	if((*ppos >= prvdata->len) && (prvdata->read_idx < HTC_RAILWAY_INFO_MAX)) {
		prvdata->len = htc_railways_copy_info(prvdata);
		*ppos = 0;
	}

	return simple_read_from_buffer(bufu, count, ppos, prvdata->buf, prvdata->len);
}

static int htc_railways_file_open(struct inode *inode, struct file *file)
{
	struct htc_railways_private_data *prvdata;
	struct htc_railways_platform_data *pdata;

	htc_railways_dump_info();
	pdata = inode->i_private;

	file->private_data = kmalloc(sizeof(struct htc_railways_private_data), GFP_KERNEL);
	if(!file->private_data)
		return -ENOMEM;

	prvdata = file->private_data;
	prvdata->read_idx = prvdata->len = 0;
	prvdata->platform_data = pdata;

	return 0;
}

static int htc_railways_file_close(struct inode *inode, struct file *file)
{
	kfree(file->private_data);
	return 0;
}

const struct file_operations htc_railways_fops =
{
	.owner	= THIS_MODULE,
	.open	= htc_railways_file_open,
	.read	= htc_railways_file_read,
	.release= htc_railways_file_close,
	.llseek	= no_llseek,
};
