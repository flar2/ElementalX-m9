/*
 * Copyright (C) 2013 HTC, Inc.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <asm/uaccess.h>
#include <soc/qcom/rpm-smd.h>
#include <soc/qcom/rpm_htc_cmd.h>

#include "rpm_htc_cmd_shared.h"

#define THIS_MODULE_NAME	"rpm_htc_cmd"

int htc_rpm_cmd_trigger_rpm_fatal(void)
{
	int rc = 0;
	uint32_t dummy = 0;

	struct msm_rpm_kvp kvp[] = {
		{
			.key = dummy,
			.data = (void *)&dummy,
			.length = sizeof(uint32_t),
		},
	};

	pr_info("RPM CMD: trigger rpm fatal.\n");

	rc = msm_rpm_send_message_noirq(MSM_RPM_CTX_ACTIVE_SET, RPM_HTC_CMD_REQ_TYPE, RHCF_RPM_FATAL, kvp, ARRAY_SIZE(kvp));

	WARN(rc < 0, "RPM CMD hold vdd dig failed.\n");
	return rc;
}

int htc_rpm_cmd_hold_vdd_dig(uint32_t enable, uint32_t corner)
{
	int rc = 0;

	struct msm_rpm_kvp kvp[] = {
		{
			.key = (VDD_DIG_HOLD_PARA_ENABLE),
			.data = (void *)&enable,
			.length = sizeof(uint32_t),
		},
		{
			.key = (VDD_DIG_HOLD_PARA_CORNER),
			.data = (void *)&corner,
			.length = sizeof(uint32_t),
		},
	};

	pr_info("RPM CMD: vote vdd dig enable = %d, corner = %d\n", enable, corner);

	rc = msm_rpm_send_message_noirq(MSM_RPM_CTX_ACTIVE_SET, RPM_HTC_CMD_REQ_TYPE, RHCF_VDD_DIG_HOLD, kvp, ARRAY_SIZE(kvp));

	WARN(rc < 0, "RPM CMD hold vdd dig failed.\n");

	return rc;
}

int htc_rpm_cmd_vote_vdd_dig(uint32_t corner)
{
	int enable = (corner > RAILWAY_NO_REQUEST) ? true : false;
	return htc_rpm_cmd_hold_vdd_dig(enable, corner);
}

#ifdef CONFIG_DEBUG_FS
#define MAX_WRITE_BUFF_CHAR (8)
static ssize_t trigger_rpm_fatal_write(struct file *file,
                                         const char __user *user_buf,
                                         size_t count, loff_t *ppos)
{
        char buf[MAX_WRITE_BUFF_CHAR];
        struct seq_file *seqf;
        unsigned long value;
        int buf_size, ret;

	if (count > MAX_WRITE_BUFF_CHAR)
		return -EINVAL;

        memset(buf, 0, sizeof(buf));
        buf_size = min(count, sizeof(buf) - 1);

        if (copy_from_user(buf, user_buf, buf_size))
                return -EFAULT;

        ret = strict_strtoul(buf, 10, &value);
        if (ret < 0)
                return ret;

	ret = htc_rpm_cmd_trigger_rpm_fatal();

	seqf = file->private_data;
	if (ret < 0)
		seq_printf(seqf, "Request vdd dig corner %lu failed.\n", value);

        *ppos += count;
        return count;
}

static int trigger_rpm_fatal_show(struct seq_file *s, void *unused)
{
	int size = 0;
	size += seq_printf(s, "Usage: echo 1 > trigger_rpm_fatal\n");
	return 0;
}

static int trigger_rpm_fatal_open(struct inode *inode, struct file *file)
{
	return single_open(file, trigger_rpm_fatal_show, inode->i_private);
}

static const struct file_operations trigger_rpm_fatal_fops = {
	.open           = trigger_rpm_fatal_open,
	.read           = seq_read,
	.write          = trigger_rpm_fatal_write,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static ssize_t hold_vdd_dig_write(struct file *file,
                                         const char __user *user_buf,
                                         size_t count, loff_t *ppos)
{
        char buf[MAX_WRITE_BUFF_CHAR];
        struct seq_file *seqf;
        unsigned long value;
        int buf_size, ret;

	if (count > MAX_WRITE_BUFF_CHAR)
		return -EINVAL;

        memset(buf, 0, sizeof(buf));
        buf_size = min(count, sizeof(buf) - 1);

        if (copy_from_user(buf, user_buf, buf_size))
                return -EFAULT;

        ret = strict_strtoul(buf, 10, &value);
        if (ret < 0)
                return ret;

	if (value)
		ret = htc_rpm_cmd_hold_vdd_dig(true, value);
	else
		ret = htc_rpm_cmd_hold_vdd_dig(false, RAILWAY_NO_REQUEST);

	seqf = file->private_data;
	if (ret < 0)
		seq_printf(seqf, "Request vdd dig corner %lu failed.\n", value);

        *ppos += count;
        return count;
}

static int hold_vdd_dig_show(struct seq_file *s, void *unused)
{
	int size = 0;
	size += seq_printf(s, "Usage: echo <corner> > hold_vdd_dig\n");
	size += seq_printf(s, "       valid coners: %d, %d, %d, %d\n",
			RAILWAY_NO_REQUEST, RAILWAY_SVS_SOC,
			RAILWAY_SVS_NOMINAL, RAILWAY_SVS_SUPER_TURBO);
	return 0;
}

static int hold_vdd_dig_open(struct inode *inode, struct file *file)
{
	return single_open(file, hold_vdd_dig_show, inode->i_private);
}

static const struct file_operations hold_vdd_dig_fops = {
	.open           = hold_vdd_dig_open,
	.read           = seq_read,
	.write          = hold_vdd_dig_write,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static struct dentry *base_dir;
static int __init rpm_htc_cmd_debugfs_init(void)
{
	base_dir = debugfs_create_dir(THIS_MODULE_NAME, NULL);
	if (!base_dir) {
		pr_err("%s: create debugfs dir failed\n", THIS_MODULE_NAME);
		return -EIO;
	}

	debugfs_create_file("hold_vdd_dig", S_IWUSR|S_IRUGO, base_dir, NULL, &hold_vdd_dig_fops);
	debugfs_create_file("trigger_rpm_fatal", S_IWUSR|S_IRUGO, base_dir, NULL, &trigger_rpm_fatal_fops);
	return 0;
}

module_init(rpm_htc_cmd_debugfs_init);
#endif
