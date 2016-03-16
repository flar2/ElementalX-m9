/* Copyright (c) 2011-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/debugfs.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/msm_ion.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/of_device.h>
#include <soc/qcom/scm.h>
#include <soc/qcom/qseecomi.h>


#ifdef HTC_TZ_LOG
#undef HTC_TZ_LOG
#endif


#define QSEE_LOG_BUF_SIZE 0x8000


#define TZBSP_DIAG_MAJOR_VERSION_LEGACY	2
#define TZBSP_MAX_CPU_COUNT 0x08
#define TZBSP_DIAG_NUM_OF_VMID 16
#define TZBSP_DIAG_VMID_DESC_LEN 7
#define TZBSP_DIAG_INT_NUM  32
#define TZBSP_MAX_INT_DESC 16
struct tzdbg_vmid_t {
	uint8_t vmid; 
	uint8_t desc[TZBSP_DIAG_VMID_DESC_LEN];	
};
struct tzdbg_boot_info_t {
	uint32_t wb_entry_cnt;	
	uint32_t wb_exit_cnt;	
	uint32_t pc_entry_cnt;	
	uint32_t pc_exit_cnt;	
	uint32_t warm_jmp_addr;	
	uint32_t spare;	
};
struct tzdbg_reset_info_t {
	uint32_t reset_type;	
	uint32_t reset_cnt;	
};
struct tzdbg_int_t {
	uint16_t int_info;
	uint8_t avail;
	uint8_t spare;
	uint32_t int_num;
	uint8_t int_desc[TZBSP_MAX_INT_DESC];
	uint64_t int_count[TZBSP_MAX_CPU_COUNT]; 
};

struct tzdbg_log_pos_t {
	uint16_t wrap;
	uint16_t offset;
};

struct tzdbg_log_t {
	struct tzdbg_log_pos_t	log_pos;
	
	uint8_t					log_buf[];
};

struct tzdbg_t {
	uint32_t magic_num;
	uint32_t version;
	uint32_t cpu_count;
	uint32_t vmid_info_off;
	uint32_t boot_info_off;
	uint32_t reset_info_off;
	uint32_t int_info_off;
	uint32_t ring_off;
	uint32_t ring_len;
	struct tzdbg_vmid_t vmid_info[TZBSP_DIAG_NUM_OF_VMID];
	struct tzdbg_boot_info_t  boot_info[TZBSP_MAX_CPU_COUNT];
	struct tzdbg_reset_info_t reset_info[TZBSP_MAX_CPU_COUNT];
	uint32_t num_interrupts;
	struct tzdbg_int_t  int_info[TZBSP_DIAG_INT_NUM];
	struct tzdbg_log_t ring_buffer;	
};

enum tzdbg_stats_type {
	TZDBG_BOOT = 0,
	TZDBG_RESET,
	TZDBG_INTERRUPT,
	TZDBG_VMID,
	TZDBG_GENERAL,
	TZDBG_LOG,
	TZDBG_QSEE_LOG,
#ifdef CONFIG_HTC_TZ_LOG
	TZDBG_HTCLOG,
#endif
	TZDBG_STATS_MAX
};

struct tzdbg_stat {
	char *name;
	char *data;
};

struct tzdbg {
	void __iomem *virt_iobase;
	struct tzdbg_t *diag_buf;
	char *disp_buf;
	int debug_tz[TZDBG_STATS_MAX];
	struct tzdbg_stat stat[TZDBG_STATS_MAX];
};

static struct tzdbg tzdbg = {

	.stat[TZDBG_BOOT].name = "boot",
	.stat[TZDBG_RESET].name = "reset",
	.stat[TZDBG_INTERRUPT].name = "interrupt",
	.stat[TZDBG_VMID].name = "vmid",
	.stat[TZDBG_GENERAL].name = "general",
	.stat[TZDBG_LOG].name = "log",
	.stat[TZDBG_QSEE_LOG].name = "qsee_log",
#ifdef CONFIG_HTC_TZ_LOG
	.stat[TZDBG_HTCLOG].name = "htclog",
#endif
};

#ifdef CONFIG_HTC_TZ_LOG
#ifndef MSM_TZLOG_PHYS
#define MSM_TZLOG_PHYS		0x05FD0000
#endif

#ifndef MSM_TZLOG_SIZE
#define MSM_TZLOG_SIZE		(64 * 1024)
#endif

#define TZ_SCM_LOG_PHYS     MSM_TZLOG_PHYS
#define TZ_SCM_LOG_SIZE     MSM_TZLOG_SIZE

#define INT_SIZE		4

struct htc_tzlog_dev {
	char *buffer;
	char *tmp_buf;
	uint32_t *pw_cursor;
	uint32_t *pr_cursor;
	uint32_t tz_scm_log_phys;
	uint32_t tz_scm_log_size;
};

static struct htc_tzlog_dev *htc_tzlog;
#endif
static struct tzdbg_log_t *g_qsee_log;
static uint32_t debug_rw_buf_size;


static int _disp_tz_general_stats(void)
{
	int len = 0;

	len += snprintf(tzdbg.disp_buf + len, debug_rw_buf_size - 1,
			"   Version        : 0x%x\n"
			"   Magic Number   : 0x%x\n"
			"   Number of CPU  : %d\n",
			tzdbg.diag_buf->version,
			tzdbg.diag_buf->magic_num,
			tzdbg.diag_buf->cpu_count);
	tzdbg.stat[TZDBG_GENERAL].data = tzdbg.disp_buf;
	return len;
}

static int _disp_tz_vmid_stats(void)
{
	int i, num_vmid;
	int len = 0;
	struct tzdbg_vmid_t *ptr;

	ptr = (struct tzdbg_vmid_t *)((unsigned char *)tzdbg.diag_buf +
					tzdbg.diag_buf->vmid_info_off);
	num_vmid = ((tzdbg.diag_buf->boot_info_off -
				tzdbg.diag_buf->vmid_info_off)/
					(sizeof(struct tzdbg_vmid_t)));

	for (i = 0; i < num_vmid; i++) {
		if (ptr->vmid < 0xFF) {
			len += snprintf(tzdbg.disp_buf + len,
				(debug_rw_buf_size - 1) - len,
				"   0x%x        %s\n",
				(uint32_t)ptr->vmid, (uint8_t *)ptr->desc);
		}
		if (len > (debug_rw_buf_size - 1)) {
			pr_warn("%s: Cannot fit all info into the buffer\n",
								__func__);
			break;
		}
		ptr++;
	}

	tzdbg.stat[TZDBG_VMID].data = tzdbg.disp_buf;
	return len;
}

static int _disp_tz_boot_stats(void)
{
	int i;
	int len = 0;
	struct tzdbg_boot_info_t *ptr;

	ptr = (struct tzdbg_boot_info_t *)((unsigned char *)tzdbg.diag_buf +
					tzdbg.diag_buf->boot_info_off);

	for (i = 0; i < tzdbg.diag_buf->cpu_count; i++) {
		len += snprintf(tzdbg.disp_buf + len,
				(debug_rw_buf_size - 1) - len,
				"  CPU #: %d\n"
				"     Warmboot jump address     : 0x%x\n"
				"     Warmboot entry CPU counter: 0x%x\n"
				"     Warmboot exit CPU counter : 0x%x\n"
				"     Power Collapse entry CPU counter: 0x%x\n"
				"     Power Collapse exit CPU counter : 0x%x\n",
				i, ptr->warm_jmp_addr, ptr->wb_entry_cnt,
				ptr->wb_exit_cnt, ptr->pc_entry_cnt,
				ptr->pc_exit_cnt);

		if (len > (debug_rw_buf_size - 1)) {
			pr_warn("%s: Cannot fit all info into the buffer\n",
								__func__);
			break;
		}
		ptr++;
	}
	tzdbg.stat[TZDBG_BOOT].data = tzdbg.disp_buf;
	return len;
}

static int _disp_tz_reset_stats(void)
{
	int i;
	int len = 0;
	struct tzdbg_reset_info_t *ptr;

	ptr = (struct tzdbg_reset_info_t *)((unsigned char *)tzdbg.diag_buf +
					tzdbg.diag_buf->reset_info_off);

	for (i = 0; i < tzdbg.diag_buf->cpu_count; i++) {
		len += snprintf(tzdbg.disp_buf + len,
				(debug_rw_buf_size - 1) - len,
				"  CPU #: %d\n"
				"     Reset Type (reason)       : 0x%x\n"
				"     Reset counter             : 0x%x\n",
				i, ptr->reset_type, ptr->reset_cnt);

		if (len > (debug_rw_buf_size - 1)) {
			pr_warn("%s: Cannot fit all info into the buffer\n",
								__func__);
			break;
		}

		ptr++;
	}
	tzdbg.stat[TZDBG_RESET].data = tzdbg.disp_buf;
	return len;
}

static int _disp_tz_interrupt_stats(void)
{
	int i, j, int_info_size;
	int len = 0;
	int *num_int;
	unsigned char *ptr;
	struct tzdbg_int_t *tzdbg_ptr;

	num_int = (uint32_t *)((unsigned char *)tzdbg.diag_buf +
			(tzdbg.diag_buf->int_info_off - sizeof(uint32_t)));
	ptr = ((unsigned char *)tzdbg.diag_buf +
					tzdbg.diag_buf->int_info_off);
	int_info_size = ((tzdbg.diag_buf->ring_off -
				tzdbg.diag_buf->int_info_off)/(*num_int));

	for (i = 0; i < (*num_int); i++) {
		tzdbg_ptr = (struct tzdbg_int_t *)ptr;
		len += snprintf(tzdbg.disp_buf + len,
				(debug_rw_buf_size - 1) - len,
				"     Interrupt Number          : 0x%x\n"
				"     Type of Interrupt         : 0x%x\n"
				"     Description of interrupt  : %s\n",
				tzdbg_ptr->int_num,
				(uint32_t)tzdbg_ptr->int_info,
				(uint8_t *)tzdbg_ptr->int_desc);
		for (j = 0; j < tzdbg.diag_buf->cpu_count; j++) {
			len += snprintf(tzdbg.disp_buf + len,
				(debug_rw_buf_size - 1) - len,
				"     int_count on CPU # %d      : %u\n",
				(uint32_t)j,
				(uint32_t)tzdbg_ptr->int_count[j]);
		}
		len += snprintf(tzdbg.disp_buf + len, debug_rw_buf_size - 1,
									"\n");

		if (len > (debug_rw_buf_size - 1)) {
			pr_warn("%s: Cannot fit all info into the buffer\n",
								__func__);
			break;
		}

		ptr += int_info_size;
	}
	tzdbg.stat[TZDBG_INTERRUPT].data = tzdbg.disp_buf;
	return len;
}

static int _disp_tz_log_stats_legacy(void)
{
	int len = 0;
	unsigned char *ptr;

	ptr = (unsigned char *)tzdbg.diag_buf +
					tzdbg.diag_buf->ring_off;
	len += snprintf(tzdbg.disp_buf, (debug_rw_buf_size - 1) - len,
							"%s\n", ptr);

	tzdbg.stat[TZDBG_LOG].data = tzdbg.disp_buf;
	return len;
}

static int _disp_log_stats(struct tzdbg_log_t *log,
			struct tzdbg_log_pos_t *log_start, uint32_t log_len,
			size_t count, uint32_t buf_idx)
{
	uint32_t wrap_start;
	uint32_t wrap_end;
	uint32_t wrap_cnt;
	int max_len;
	int len = 0;
	int i = 0;

	wrap_start = log_start->wrap;
	wrap_end = log->log_pos.wrap;

	
	if (wrap_end >= wrap_start) {
		wrap_cnt = wrap_end - wrap_start;
	} else {
		
		wrap_cnt = 2;
	}

	if (wrap_cnt > 1) {
		
		
		log_start->wrap = log->log_pos.wrap - 1;
		log_start->offset = (log->log_pos.offset + 1) % log_len;
	} else if ((wrap_cnt == 1) &&
		(log->log_pos.offset > log_start->offset)) {
		/* end position has overwritten start */
		log_start->offset = (log->log_pos.offset + 1) % log_len;
	}

	while (log_start->offset == log->log_pos.offset) {
		unsigned long t = msleep_interruptible(50);
		if (t != 0) {
			
			return 0;
		}

		if (buf_idx == TZDBG_LOG)
			memcpy_fromio((void *)tzdbg.diag_buf, tzdbg.virt_iobase,
						debug_rw_buf_size);

	}

	max_len = (count > debug_rw_buf_size) ? debug_rw_buf_size : count;

	while ((log_start->offset != log->log_pos.offset) && (len < max_len)) {
		tzdbg.disp_buf[i++] = log->log_buf[log_start->offset];
		log_start->offset = (log_start->offset + 1) % log_len;
		if (0 == log_start->offset)
			++log_start->wrap;
		++len;
	}

	tzdbg.stat[buf_idx].data = tzdbg.disp_buf;
	return len;
}

static int _disp_tz_log_stats(size_t count)
{
	static struct tzdbg_log_pos_t log_start = {0};
	struct tzdbg_log_t *log_ptr;
	log_ptr = (struct tzdbg_log_t *)((unsigned char *)tzdbg.diag_buf +
				tzdbg.diag_buf->ring_off -
				offsetof(struct tzdbg_log_t, log_buf));

	return _disp_log_stats(log_ptr, &log_start,
				tzdbg.diag_buf->ring_len, count, TZDBG_LOG);
}

#ifdef CONFIG_HTC_TZ_LOG
static int _disp_tz_htc_log_stats(char __user *ubuf, size_t count, loff_t *offp)
{
	char *buf = htc_tzlog->buffer;
	uint32_t *pw_cursor = htc_tzlog->pw_cursor;
	uint32_t *pr_cursor = htc_tzlog->pr_cursor;
	uint32_t tz_scm_log_size = htc_tzlog->tz_scm_log_size;
	uint32_t r_cursor, w_cursor;
	int ret;

	if (buf != 0 && (count <= MSM_TZLOG_SIZE)) {
		
		r_cursor = readl_relaxed(pr_cursor);
		w_cursor = readl_relaxed(pw_cursor);

		if (r_cursor < w_cursor) {
			if ((w_cursor - r_cursor) > count) {
				memcpy_fromio(htc_tzlog->tmp_buf, buf + r_cursor, count);
				ret = copy_to_user(ubuf, htc_tzlog->tmp_buf, count);
				if (ret == count)
					return -EFAULT;

				writel_relaxed(r_cursor + count, pr_cursor);
				return count;
			} else {
				memcpy_fromio(htc_tzlog->tmp_buf, buf + r_cursor, (w_cursor - r_cursor));
				ret = copy_to_user(ubuf, htc_tzlog->tmp_buf, (w_cursor - r_cursor));
				if (ret == (w_cursor - r_cursor))
					return -EFAULT;

				writel_relaxed(w_cursor, pr_cursor);
				return (w_cursor - r_cursor);
			}
		}

		if (r_cursor > w_cursor) {
			uint32_t buf_end = tz_scm_log_size - 2*INT_SIZE - 1;
			uint32_t left_len = buf_end - r_cursor;

			if (left_len > count) {
				memcpy_fromio(htc_tzlog->tmp_buf, buf + r_cursor, count);
				ret = copy_to_user(ubuf, htc_tzlog->tmp_buf, count);
				if (ret == count)
					return -EFAULT;

				writel_relaxed(r_cursor + count, pr_cursor);
				return count;
			} else {
				memcpy_fromio(htc_tzlog->tmp_buf, buf + r_cursor, left_len);
				ret = copy_to_user(ubuf, htc_tzlog->tmp_buf, left_len);
				if (ret == left_len)
					return -EFAULT;

				writel_relaxed(0, pr_cursor);
				return left_len;
			}
		}

		if (r_cursor == w_cursor) {
			pr_info("No New Trust Zone log\n");
			return 0;
		}
	}

	return 0;
}
#endif

static int _disp_qsee_log_stats(size_t count)
{
	static struct tzdbg_log_pos_t log_start = {0};

	return _disp_log_stats(g_qsee_log, &log_start,
			QSEE_LOG_BUF_SIZE - sizeof(struct tzdbg_log_pos_t),
			count, TZDBG_QSEE_LOG);
}

static ssize_t tzdbgfs_read(struct file *file, char __user *buf,
	size_t count, loff_t *offp)
{
	int len = 0;
	int *tz_id =  file->private_data;

	memcpy_fromio((void *)tzdbg.diag_buf, tzdbg.virt_iobase,
						debug_rw_buf_size);
	switch (*tz_id) {
	case TZDBG_BOOT:
		len = _disp_tz_boot_stats();
		break;
	case TZDBG_RESET:
		len = _disp_tz_reset_stats();
		break;
	case TZDBG_INTERRUPT:
		len = _disp_tz_interrupt_stats();
		break;
	case TZDBG_GENERAL:
		len = _disp_tz_general_stats();
		break;
	case TZDBG_VMID:
		len = _disp_tz_vmid_stats();
		break;
	case TZDBG_LOG:
		if (TZBSP_DIAG_MAJOR_VERSION_LEGACY <
				(tzdbg.diag_buf->version >> 16)) {
			len = _disp_tz_log_stats(count);
			*offp = 0;
		} else {
			len = _disp_tz_log_stats_legacy();
		}
		break;
	case TZDBG_QSEE_LOG:
		len = _disp_qsee_log_stats(count);
		*offp = 0;
		break;
#ifdef CONFIG_HTC_TZ_LOG
	case TZDBG_HTCLOG:
		return _disp_tz_htc_log_stats(buf, count, offp);
#endif
	default:
		break;
	}

	if (len > count)
		len = count;

	return simple_read_from_buffer(buf, len, offp,
				tzdbg.stat[(*tz_id)].data, len);
}

static int tzdbgfs_open(struct inode *inode, struct file *pfile)
{
	pfile->private_data = inode->i_private;
	return 0;
}

const struct file_operations tzdbg_fops = {
	.owner   = THIS_MODULE,
	.read    = tzdbgfs_read,
	.open    = tzdbgfs_open,
};

static struct ion_client  *g_ion_clnt;
static struct ion_handle *g_ihandle;

static void tzdbg_register_qsee_log_buf(void)
{
	
	struct qseecom_reg_log_buf_ireq req;

	
	struct qseecom_command_scm_resp resp = {};
	ion_phys_addr_t pa = 0;
	size_t len;
	int ret = 0;

	
	g_ion_clnt = msm_ion_client_create("qsee_log");
	if (g_ion_clnt == NULL) {
		pr_err("%s: Ion client cannot be created\n", __func__);
		return;
	}

	g_ihandle = ion_alloc(g_ion_clnt, QSEE_LOG_BUF_SIZE,
			4096, ION_HEAP(ION_QSECOM_HEAP_ID), 0);
	if (IS_ERR_OR_NULL(g_ihandle)) {
		pr_err("%s: Ion client could not retrieve the handle\n",
			__func__);
		goto err1;
	}

	ret = ion_phys(g_ion_clnt, g_ihandle, &pa, &len);
	if (ret) {
		pr_err("%s: Ion conversion to physical address failed\n",
			__func__);
		goto err2;
	}

	req.qsee_cmd_id = QSEOS_REGISTER_LOG_BUF_COMMAND;
	req.phy_addr = (uint32_t)pa;
	req.len = len;

	if (!is_scm_armv8()) {
		
		ret = scm_call(SCM_SVC_TZSCHEDULER, 1,  &req, sizeof(req),
			&resp, sizeof(resp));
	} else {
		struct scm_desc desc = {0};
		desc.args[0] = (uint32_t)pa;
		desc.args[1] = len;
		desc.arginfo = 0x22;
		ret = scm_call2(SCM_QSEEOS_FNID(1, 6), &desc);
		resp.result = desc.ret[0];
	}

	if (ret) {
		pr_err("%s: scm_call to register log buffer failed\n",
			__func__);
		goto err2;
	}

	if (resp.result != QSEOS_RESULT_SUCCESS) {
		pr_err(
		"%s: scm_call to register log buf failed, resp result =%d\n",
		__func__, resp.result);
		goto err2;
	}

	g_qsee_log =
		(struct tzdbg_log_t *)ion_map_kernel(g_ion_clnt, g_ihandle);

	if (IS_ERR(g_qsee_log)) {
		pr_err("%s: Couldn't map ion buffer to kernel\n",
			__func__);
		goto err2;
	}

	g_qsee_log->log_pos.wrap = g_qsee_log->log_pos.offset = 0;
	return;

err2:
	ion_free(g_ion_clnt, g_ihandle);
	g_ihandle = NULL;
err1:
	ion_client_destroy(g_ion_clnt);
	g_ion_clnt = NULL;
}

static int  tzdbgfs_init(struct platform_device *pdev)
{
	int rc = 0;
	int i;
	struct dentry           *dent_dir;
	struct dentry           *dent;

	dent_dir = debugfs_create_dir("tzdbg", NULL);
	if (dent_dir == NULL) {
		dev_err(&pdev->dev, "tzdbg debugfs_create_dir failed\n");
		return -ENOMEM;
	}

	for (i = 0; i < TZDBG_STATS_MAX; i++) {
		tzdbg.debug_tz[i] = i;
		dent = debugfs_create_file(tzdbg.stat[i].name,
				S_IRUGO, dent_dir,
				&tzdbg.debug_tz[i], &tzdbg_fops);
		if (dent == NULL) {
			dev_err(&pdev->dev, "TZ debugfs_create_file failed\n");
			rc = -ENOMEM;
			goto err;
		}
	}
	tzdbg.disp_buf = kzalloc(debug_rw_buf_size, GFP_KERNEL);
	if (tzdbg.disp_buf == NULL) {
		pr_err("%s: Can't Allocate memory for tzdbg.disp_buf\n",
			__func__);

		goto err;
	}
	platform_set_drvdata(pdev, dent_dir);
	return 0;
err:
	debugfs_remove_recursive(dent_dir);

	return rc;
}

static void tzdbgfs_exit(struct platform_device *pdev)
{
	struct dentry           *dent_dir;

	kzfree(tzdbg.disp_buf);
	dent_dir = platform_get_drvdata(pdev);
	debugfs_remove_recursive(dent_dir);
	if (g_ion_clnt != NULL) {
		if (!IS_ERR_OR_NULL(g_ihandle)) {
			ion_unmap_kernel(g_ion_clnt, g_ihandle);
			ion_free(g_ion_clnt, g_ihandle);
		}
	ion_client_destroy(g_ion_clnt);
}
}

static int tz_log_probe(struct platform_device *pdev)
{
	struct resource *resource;
	void __iomem *virt_iobase;
	phys_addr_t tzdiag_phy_iobase;
	uint32_t *ptr = NULL;
#ifdef CONFIG_HTC_TZ_LOG
	struct device_node *node = pdev->dev.of_node;
	uint32_t tz_scm_log_phys, tz_scm_log_size;
#endif

	resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!resource) {
		dev_err(&pdev->dev,
				"%s: ERROR Missing MEM resource\n", __func__);
		return -ENXIO;
	};

	debug_rw_buf_size = resource->end - resource->start + 1;

	virt_iobase = devm_ioremap_nocache(&pdev->dev, resource->start,
				debug_rw_buf_size);
	if (!virt_iobase) {
		dev_err(&pdev->dev,
			"%s: ERROR could not ioremap: start=%pr, len=%u\n",
			__func__, &resource->start,
			(unsigned int)(debug_rw_buf_size));
		return -ENXIO;
	}
	tzdiag_phy_iobase = readl_relaxed(virt_iobase);

	tzdbg.virt_iobase = devm_ioremap_nocache(&pdev->dev,
				tzdiag_phy_iobase, debug_rw_buf_size);

	if (!tzdbg.virt_iobase) {
		dev_err(&pdev->dev,
			"%s: ERROR could not ioremap: start=%pr, len=%u\n",
			__func__, &tzdiag_phy_iobase,
			debug_rw_buf_size);
		return -ENXIO;
	}

	ptr = kzalloc(debug_rw_buf_size, GFP_KERNEL);
	if (ptr == NULL) {
		pr_err("%s: Can't Allocate memory: ptr\n",
			__func__);
		return -ENXIO;
	}

	tzdbg.diag_buf = (struct tzdbg_t *)ptr;

#ifdef CONFIG_HTC_TZ_LOG
	htc_tzlog = kzalloc(sizeof(struct htc_tzlog_dev), GFP_KERNEL);
	if (!htc_tzlog) {
		pr_err("%s: Can't Allocate memory: scm_dev\n", __func__);
		return -ENOMEM;
	}

	htc_tzlog->tmp_buf = kzalloc(MSM_TZLOG_SIZE, GFP_KERNEL);
	if (!htc_tzlog->tmp_buf) {
		pr_err("%s: Can't Allocate memory: scm_dev\n", __func__);
		kfree(htc_tzlog);
		return -ENOMEM;
	}

	if (of_property_read_u32(node, "htc,tz_scm_log_phys", &tz_scm_log_phys))
		tz_scm_log_phys = TZ_SCM_LOG_PHYS;
	if (of_property_read_u32(node, "htc,tz_scm_log_size", &tz_scm_log_size))
		tz_scm_log_size = TZ_SCM_LOG_SIZE;

	htc_tzlog->tz_scm_log_phys = tz_scm_log_phys;
	htc_tzlog->tz_scm_log_size = tz_scm_log_size;

	htc_tzlog->buffer = devm_ioremap_nocache(&pdev->dev,
		tz_scm_log_phys, tz_scm_log_size);
	if (htc_tzlog->buffer == NULL) {
		pr_err("%s: ioremap fail...\n", __func__);
		kfree(htc_tzlog->tmp_buf);
		kfree(htc_tzlog);
		return -EFAULT;
	}

	pr_info("[TZLOG] buffer address:%x size:%x\n",
		tz_scm_log_phys, tz_scm_log_size);

	htc_tzlog->pr_cursor = (uint32_t *)(htc_tzlog->buffer +	tz_scm_log_size - 2*4);
	htc_tzlog->pw_cursor = (uint32_t *)(htc_tzlog->buffer +	tz_scm_log_size - 4);

	memset_io(htc_tzlog->buffer, 0, tz_scm_log_size);
	secure_log_operation(tz_scm_log_phys, tz_scm_log_size, 0 , 0 , 0);
#endif

	if (tzdbgfs_init(pdev))
		goto err;

	tzdbg_register_qsee_log_buf();
	return 0;
err:
	kfree(tzdbg.diag_buf);
	return -ENXIO;
}


static int tz_log_remove(struct platform_device *pdev)
{
	kzfree(tzdbg.diag_buf);
	tzdbgfs_exit(pdev);

	return 0;
}

static struct of_device_id tzlog_match[] = {
	{	.compatible = "qcom,tz-log",
	},
	{}
};

static struct platform_driver tz_log_driver = {
	.probe		= tz_log_probe,
	.remove		= tz_log_remove,
	.driver		= {
		.name = "tz_log",
		.owner = THIS_MODULE,
		.of_match_table = tzlog_match,
	},
};

static int __init tz_log_init(void)
{
	return platform_driver_register(&tz_log_driver);
}

static void __exit tz_log_exit(void)
{
	platform_driver_unregister(&tz_log_driver);
}

module_init(tz_log_init);
module_exit(tz_log_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("TZ Log driver");
MODULE_VERSION("1.1");
MODULE_ALIAS("platform:tz_log");
