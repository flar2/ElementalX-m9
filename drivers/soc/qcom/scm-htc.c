/* Copyright (c) 2010-2013, The Linux Foundation. All rights reserved.
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

#include <linux/slab.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/delay.h>

#include <asm/cacheflush.h>
#include <asm/compiler.h>

#include <soc/qcom/scm.h>

static int security_level;
static int security_wp_magic;
static int security_wp_type;

int secure_get_security_level(uint32_t *level)
{
	int ret;
	struct scm_desc desc = {0};

	if (!is_scm_armv8())
		return -EINVAL;

	desc.arginfo = SCM_ARGS(0);
	ret = scm_call2(SCM_OEM_FNID(SCM_SVC_HTC, TZ_HTC_SVC_GET_SECURITY_LEVEL), &desc);
	*level = desc.ret[0];

	pr_info("TZ_HTC_SVC_GET_SECURITY_LEVEL ret = %d, %d\n", ret, *level);
	return ret;
}
EXPORT_SYMBOL(secure_get_security_level);

int secure_get_msm_serial(uint32_t *serial)
{
	int ret;
	struct scm_desc desc = {0};

	if (!is_scm_armv8())
		return -EINVAL;

	desc.arginfo = SCM_ARGS(0);
	ret = scm_call2(SCM_OEM_FNID(SCM_SVC_HTC, TZ_HTC_SVC_GET_SECURITY_LEVEL), &desc);
	*serial = desc.ret[1];

	pr_debug("TZ_HTC_SVC_GET_SECURITY_LEVEL ret = %d, %d\n", ret, *serial);
	return ret;
}
EXPORT_SYMBOL(secure_get_msm_serial);

int secure_wp_magic_write(uint32_t magic, uint32_t type)
{
	int ret;
	struct scm_desc desc = {0};

	if (!is_scm_armv8())
		return -EINVAL;

	desc.arginfo = SCM_ARGS(2);
	desc.args[0] = magic;
	desc.args[1] = type;

	ret = scm_call2(SCM_OEM_FNID(SCM_SVC_HTC, TZ_HTC_SVC_WP_MAGIC_WRITE), &desc);
	pr_info("TZ_HTC_SVC_WP_MAGIC_WRITE ret = %d\n", ret);

	return ret;
}
EXPORT_SYMBOL(secure_wp_magic_write);

int secure_wp_magic_read(uint32_t *magic, uint32_t *type)
{
	int ret;
	struct scm_desc desc = {0};

	if (!is_scm_armv8())
		return -EINVAL;

	desc.arginfo = SCM_ARGS(0);
	ret = scm_call2(SCM_OEM_FNID(SCM_SVC_HTC, TZ_HTC_SVC_WP_MAGIC_READ), &desc);

	*magic = desc.ret[0];
	*type = desc.ret[1];
	pr_info("TZ_HTC_SVC_WP_MAGIC_READ ret = %d\n", ret);

	return ret;
}
EXPORT_SYMBOL(secure_wp_magic_read);

int secure_log_operation(unsigned int address, unsigned int size,
        unsigned int buf_addr, unsigned buf_len, int revert)
{
	int ret;
	struct scm_desc desc = {0};

	if (!is_scm_armv8())
        return -EINVAL;

	desc.args[0] = address;
	desc.args[1] = size;
	desc.arginfo = SCM_ARGS(2);
	ret = scm_call2(SCM_OEM_FNID(SCM_SVC_HTC, TZ_HTC_SVC_LOG_OPERATOR), &desc);
	pr_info("TZ_HTC_SVC_LOG_OPERATOR ret = %d\n", ret);
	return ret;
}

static int level_set_func(const char *val, struct kernel_param *kp)
{
	int ret;

	pr_info("%s started(%d)...\n", __func__, (int)strlen(val));
	ret = param_set_int(val, kp);
	pr_info("%s finished(%d): %d...\n", __func__, ret, security_level);

	return ret;
}

static int level_get_func(char *val, struct kernel_param *kp)
{
	int ret;

	ret = secure_get_security_level(&security_level);
	pr_info("%s: %d, %d(%x)...\n", __func__, ret, security_level, security_level);
	ret = param_get_int(val, kp);
	pr_info("%s: %d, %d(%x)...\n", __func__, ret, security_level, security_level);

    return ret;
}

module_param_call(security_level, level_set_func, level_get_func, &security_level, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);


static int wp_magic_set_func(const char *val, struct kernel_param *kp)
{
	int ret;

	param_set_int(val, kp);
	pr_info("%s: magic: %d(%x) type: %d(%x)\n", __func__,
			security_wp_magic, security_wp_magic, security_wp_type, security_wp_type);
	ret = secure_wp_magic_write(security_wp_magic, security_wp_type);

	return ret;
}

static int wp_magic_get_func(char *val, struct kernel_param *kp)
{
	int ret;

	security_wp_magic = 0;
	security_wp_type = 0;
	ret = secure_wp_magic_read(&security_wp_magic, &security_wp_type);
	if (ret)
		pr_info("%s: secure_wp_magic_read error\n", __func__);
	else
		pr_info("%s: magic: %d(%x) type: %d(%x)\n", __func__,
			security_wp_magic, security_wp_magic, security_wp_type, security_wp_type);
	ret = param_get_int(val, kp);

	return ret;
}

module_param_call(security_wp_magic, wp_magic_set_func, wp_magic_get_func, &security_wp_magic, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
module_param_call(security_wp_type, wp_magic_set_func, wp_magic_get_func, &security_wp_type, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);

