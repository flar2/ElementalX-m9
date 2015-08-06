/* Copyright (c) 2010-2015, The Linux Foundation. All rights reserved.
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

#define SCM_ENOMEM		-5
#define SCM_EOPNOTSUPP		-4
#define SCM_EINVAL_ADDR		-3
#define SCM_EINVAL_ARG		-2
#define SCM_ERROR		-1
#define SCM_INTERRUPTED		1
#define SCM_EBUSY		-55
#define SCM_V2_EBUSY		-12

static DEFINE_MUTEX(scm_lock);

#define SCM_EBUSY_WAIT_MS 30
#define SCM_EBUSY_MAX_RETRY 20
#define SCM_EBUSY_MAX_RETRY_RESTORE_CFG 60

#define RESTORE_SEC_CFG    2

#define IOMMU_SEC_ID	18
#define IOMMU_SPARE_NUM	0

#define N_EXT_SCM_ARGS 7
#define FIRST_EXT_ARG_IDX 3
#define SMC_ATOMIC_SYSCALL 31
#define N_REGISTER_ARGS (MAX_SCM_ARGS - N_EXT_SCM_ARGS + 1)
#define SMC64_MASK 0x40000000
#define SMC_ATOMIC_MASK 0x80000000
#define IS_CALL_AVAIL_CMD 1

#define SCM_BUF_LEN(__cmd_size, __resp_size)	\
	(sizeof(struct scm_command) + sizeof(struct scm_response) + \
		__cmd_size + __resp_size)
struct scm_command {
	u32	len;
	u32	buf_offset;
	u32	resp_hdr_offset;
	u32	id;
	u32	buf[0];
};

struct scm_response {
	u32	len;
	u32	buf_offset;
	u32	is_complete;
};

#ifdef CONFIG_ARM64

#define R0_STR "x0"
#define R1_STR "x1"
#define R2_STR "x2"
#define R3_STR "x3"
#define R4_STR "x4"
#define R5_STR "x5"

#define outer_inv_range(x, y)
#define outer_flush_range(x, y)

#define __cpuc_flush_dcache_area __flush_dcache_area

#else

#define R0_STR "r0"
#define R1_STR "r1"
#define R2_STR "r2"
#define R3_STR "r3"
#define R4_STR "r4"
#define R5_STR "r5"

#endif

static inline struct scm_response *scm_command_to_response(
		const struct scm_command *cmd)
{
	return (void *)cmd + cmd->resp_hdr_offset;
}

static inline void *scm_get_command_buffer(const struct scm_command *cmd)
{
	return (void *)cmd->buf;
}

static inline void *scm_get_response_buffer(const struct scm_response *rsp)
{
	return (void *)rsp + rsp->buf_offset;
}

static int scm_remap_error(int err)
{
	switch (err) {
	case SCM_ERROR:
		return -EIO;
	case SCM_EINVAL_ADDR:
	case SCM_EINVAL_ARG:
		return -EINVAL;
	case SCM_EOPNOTSUPP:
		return -EOPNOTSUPP;
	case SCM_ENOMEM:
		return -ENOMEM;
	case SCM_EBUSY:
		return SCM_EBUSY;
	case SCM_V2_EBUSY:
		return SCM_V2_EBUSY;
	}
	return -EINVAL;
}

static u32 smc(u32 cmd_addr)
{
	int context_id;
	register u32 r0 asm("r0") = 1;
	register u32 r1 asm("r1") = (uintptr_t)&context_id;
	register u32 r2 asm("r2") = cmd_addr;
	do {
		asm volatile(
			__asmeq("%0", R0_STR)
			__asmeq("%1", R0_STR)
			__asmeq("%2", R1_STR)
			__asmeq("%3", R2_STR)
#ifdef REQUIRES_SEC
			".arch_extension sec\n"
#endif
			"smc	#0\n"
			: "=r" (r0)
			: "r" (r0), "r" (r1), "r" (r2)
			: "r3");
	} while (r0 == SCM_INTERRUPTED);

	return r0;
}

static int __scm_call(const struct scm_command *cmd)
{
	int ret;
	u32 cmd_addr = virt_to_phys(cmd);

	__cpuc_flush_dcache_area((void *)cmd, cmd->len);
	outer_flush_range(cmd_addr, cmd_addr + cmd->len);

	ret = smc(cmd_addr);
	if (ret < 0) {
		if (ret != SCM_EBUSY)
			pr_err("scm_call failed with error code %d\n", ret);
		ret = scm_remap_error(ret);
	}
	return ret;
}

#ifndef CONFIG_ARM64
static void scm_inv_range(unsigned long start, unsigned long end)
{
	u32 cacheline_size, ctr;

	asm volatile("mrc p15, 0, %0, c0, c0, 1" : "=r" (ctr));
	cacheline_size = 4 << ((ctr >> 16) & 0xf);

	start = round_down(start, cacheline_size);
	end = round_up(end, cacheline_size);
	outer_inv_range(start, end);
	while (start < end) {
		asm ("mcr p15, 0, %0, c7, c6, 1" : : "r" (start)
		     : "memory");
		start += cacheline_size;
	}
	dsb();
	isb();
}
#else

static void scm_inv_range(unsigned long start, unsigned long end)
{
	dmac_inv_range((void *)start, (void *)end);
}
#endif


static int scm_call_common(u32 svc_id, u32 cmd_id, const void *cmd_buf,
				size_t cmd_len, void *resp_buf, size_t resp_len,
				struct scm_command *scm_buf,
				size_t scm_buf_length)
{
	int ret;
	struct scm_response *rsp;
	unsigned long start, end;

	scm_buf->len = scm_buf_length;
	scm_buf->buf_offset = offsetof(struct scm_command, buf);
	scm_buf->resp_hdr_offset = scm_buf->buf_offset + cmd_len;
	scm_buf->id = (svc_id << 10) | cmd_id;

	if (cmd_buf)
		memcpy(scm_get_command_buffer(scm_buf), cmd_buf, cmd_len);

	mutex_lock(&scm_lock);
	ret = __scm_call(scm_buf);
	mutex_unlock(&scm_lock);
	if (ret)
		return ret;

	rsp = scm_command_to_response(scm_buf);
	start = (unsigned long)rsp;

	do {
		scm_inv_range(start, start + sizeof(*rsp));
	} while (!rsp->is_complete);

	end = (unsigned long)scm_get_response_buffer(rsp) + resp_len;
	scm_inv_range(start, end);

	if (resp_buf)
		memcpy(resp_buf, scm_get_response_buffer(rsp), resp_len);

	return ret;
}

static int _scm_call_retry(u32 svc_id, u32 cmd_id, const void *cmd_buf,
				size_t cmd_len, void *resp_buf, size_t resp_len,
				struct scm_command *cmd,
				size_t len)
{
	int ret, retry_count = 0;

	do {
		ret = scm_call_common(svc_id, cmd_id, cmd_buf, cmd_len,
					resp_buf, resp_len, cmd, len);
		if (ret == SCM_EBUSY)
			msleep(SCM_EBUSY_WAIT_MS);
	} while (ret == SCM_EBUSY && (retry_count++ < SCM_EBUSY_MAX_RETRY));

	if (ret == SCM_EBUSY)
		pr_err("scm: secure world busy (rc = SCM_EBUSY)\n");

	return ret;
}

int scm_call_noalloc(u32 svc_id, u32 cmd_id, const void *cmd_buf,
		size_t cmd_len, void *resp_buf, size_t resp_len,
		void *scm_buf, size_t scm_buf_len)
{
	int ret;
	size_t len = SCM_BUF_LEN(cmd_len, resp_len);

	if (cmd_len > scm_buf_len || resp_len > scm_buf_len ||
	    len > scm_buf_len)
		return -EINVAL;

	if (!IS_ALIGNED((unsigned long)scm_buf, PAGE_SIZE))
		return -EINVAL;

	memset(scm_buf, 0, scm_buf_len);

	ret = scm_call_common(svc_id, cmd_id, cmd_buf, cmd_len, resp_buf,
				resp_len, scm_buf, len);
	return ret;

}

#ifdef CONFIG_ARM64

static int __scm_call_armv8_64(u64 x0, u64 x1, u64 x2, u64 x3, u64 x4, u64 x5,
				u64 *ret1, u64 *ret2, u64 *ret3)
{
	register u64 r0 asm("r0") = x0;
	register u64 r1 asm("r1") = x1;
	register u64 r2 asm("r2") = x2;
	register u64 r3 asm("r3") = x3;
	register u64 r4 asm("r4") = x4;
	register u64 r5 asm("r5") = x5;

	do {
		asm volatile(
			__asmeq("%0", R0_STR)
			__asmeq("%1", R1_STR)
			__asmeq("%2", R2_STR)
			__asmeq("%3", R3_STR)
			__asmeq("%4", R0_STR)
			__asmeq("%5", R1_STR)
			__asmeq("%6", R2_STR)
			__asmeq("%7", R3_STR)
			__asmeq("%8", R4_STR)
			__asmeq("%9", R5_STR)
#ifdef REQUIRES_SEC
			".arch_extension sec\n"
#endif
			"smc	#0\n"
			: "=r" (r0), "=r" (r1), "=r" (r2), "=r" (r3)
			: "r" (r0), "r" (r1), "r" (r2), "r" (r3), "r" (r4),
			  "r" (r5)
			: "x6", "x7", "x8", "x9", "x10", "x11", "x12", "x13",
			  "x14", "x15", "x16", "x17");
	} while (r0 == SCM_INTERRUPTED);

	if (ret1)
		*ret1 = r1;
	if (ret2)
		*ret2 = r2;
	if (ret3)
		*ret3 = r3;

	return r0;
}

static int __scm_call_armv8_32(u32 w0, u32 w1, u32 w2, u32 w3, u32 w4, u32 w5,
				u64 *ret1, u64 *ret2, u64 *ret3)
{
	register u32 r0 asm("r0") = w0;
	register u32 r1 asm("r1") = w1;
	register u32 r2 asm("r2") = w2;
	register u32 r3 asm("r3") = w3;
	register u32 r4 asm("r4") = w4;
	register u32 r5 asm("r5") = w5;

	do {
		asm volatile(
			__asmeq("%0", R0_STR)
			__asmeq("%1", R1_STR)
			__asmeq("%2", R2_STR)
			__asmeq("%3", R3_STR)
			__asmeq("%4", R0_STR)
			__asmeq("%5", R1_STR)
			__asmeq("%6", R2_STR)
			__asmeq("%7", R3_STR)
			__asmeq("%8", R4_STR)
			__asmeq("%9", R5_STR)
#ifdef REQUIRES_SEC
			".arch_extension sec\n"
#endif
			"smc	#0\n"
			: "=r" (r0), "=r" (r1), "=r" (r2), "=r" (r3)
			: "r" (r0), "r" (r1), "r" (r2), "r" (r3), "r" (r4),
			  "r" (r5)
			: "x6", "x7", "x8", "x9", "x10", "x11", "x12", "x13",
			"x14", "x15", "x16", "x17");

	} while (r0 == SCM_INTERRUPTED);

	if (ret1)
		*ret1 = r1;
	if (ret2)
		*ret2 = r2;
	if (ret3)
		*ret3 = r3;

	return r0;
}

#else

static int __scm_call_armv8_32(u32 w0, u32 w1, u32 w2, u32 w3, u32 w4, u32 w5,
				u64 *ret1, u64 *ret2, u64 *ret3)
{
	register u32 r0 asm("r0") = w0;
	register u32 r1 asm("r1") = w1;
	register u32 r2 asm("r2") = w2;
	register u32 r3 asm("r3") = w3;
	register u32 r4 asm("r4") = w4;
	register u32 r5 asm("r5") = w5;

	do {
		asm volatile(
			__asmeq("%0", R0_STR)
			__asmeq("%1", R1_STR)
			__asmeq("%2", R2_STR)
			__asmeq("%3", R3_STR)
			__asmeq("%4", R0_STR)
			__asmeq("%5", R1_STR)
			__asmeq("%6", R2_STR)
			__asmeq("%7", R3_STR)
			__asmeq("%8", R4_STR)
			__asmeq("%9", R5_STR)
#ifdef REQUIRES_SEC
			".arch_extension sec\n"
#endif
			"smc	#0\n"
			: "=r" (r0), "=r" (r1), "=r" (r2), "=r" (r3)
			: "r" (r0), "r" (r1), "r" (r2), "r" (r3), "r" (r4),
			 "r" (r5));

	} while (r0 == SCM_INTERRUPTED);

	if (ret1)
		*ret1 = r1;
	if (ret2)
		*ret2 = r2;
	if (ret3)
		*ret3 = r3;

	return r0;
}

static int __scm_call_armv8_64(u64 x0, u64 x1, u64 x2, u64 x3, u64 x4, u64 x5,
				u64 *ret1, u64 *ret2, u64 *ret3)
{
	return 0;
}
#endif

struct scm_extra_arg {
	union {
		u32 args32[N_EXT_SCM_ARGS];
		u64 args64[N_EXT_SCM_ARGS];
	};
};

static enum scm_interface_version {
	SCM_UNKNOWN,
	SCM_LEGACY,
	SCM_ARMV8_32,
	SCM_ARMV8_64,
} scm_version = SCM_UNKNOWN;

static u32 scm_version_mask;

bool is_scm_armv8(void)
{
	int ret;
	u64 ret1, x0;

	if (likely(scm_version != SCM_UNKNOWN))
		return (scm_version == SCM_ARMV8_32) ||
			(scm_version == SCM_ARMV8_64);

	
	scm_version = SCM_ARMV8_64;
	ret1 = 0;
	x0 = SCM_SIP_FNID(SCM_SVC_INFO, IS_CALL_AVAIL_CMD) | SMC_ATOMIC_MASK;
	ret = __scm_call_armv8_64(x0 | SMC64_MASK, SCM_ARGS(1), x0, 0, 0, 0,
				  &ret1, NULL, NULL);
	if (ret || !ret1) {
		
		ret1 = 0;
		ret = __scm_call_armv8_32(x0, SCM_ARGS(1), x0, 0, 0, 0,
					  &ret1, NULL, NULL);
		if (ret || !ret1)
			scm_version = SCM_LEGACY;
		else
			scm_version = SCM_ARMV8_32;
	} else
		scm_version_mask = SMC64_MASK;

	pr_debug("scm_call: scm version is %x, mask is %x\n", scm_version,
		  scm_version_mask);

	return (scm_version == SCM_ARMV8_32) ||
			(scm_version == SCM_ARMV8_64);
}
EXPORT_SYMBOL(is_scm_armv8);

static int allocate_extra_arg_buffer(struct scm_desc *desc, gfp_t flags)
{
	int i, j;
	struct scm_extra_arg *argbuf;
	int arglen = desc->arginfo & 0xf;
	size_t argbuflen = PAGE_ALIGN(sizeof(struct scm_extra_arg));

	desc->x5 = desc->args[FIRST_EXT_ARG_IDX];

	if (likely(arglen <= N_REGISTER_ARGS)) {
		desc->extra_arg_buf = NULL;
		return 0;
	}

	argbuf = kzalloc(argbuflen, flags);
	if (!argbuf) {
		pr_err("scm_call: failed to alloc mem for extended argument buffer\n");
		return -ENOMEM;
	}

	desc->extra_arg_buf = argbuf;

	j = FIRST_EXT_ARG_IDX;
	if (scm_version == SCM_ARMV8_64)
		for (i = 0; i < N_EXT_SCM_ARGS; i++)
			argbuf->args64[i] = desc->args[j++];
	else
		for (i = 0; i < N_EXT_SCM_ARGS; i++)
			argbuf->args32[i] = desc->args[j++];
	desc->x5 = virt_to_phys(argbuf);
	__cpuc_flush_dcache_area(argbuf, argbuflen);
	outer_flush_range(virt_to_phys(argbuf),
			  virt_to_phys(argbuf) + argbuflen);

	return 0;
}

static inline bool is_restore_cfg(u32 fn_id, struct scm_desc *desc)
{
	return fn_id == SCM_SIP_FNID(SCM_SVC_MP, RESTORE_SEC_CFG) &&
		desc->args[0] == IOMMU_SEC_ID &&
		desc->args[1] == IOMMU_SPARE_NUM;
}

int scm_call2(u32 fn_id, struct scm_desc *desc)
{
	int arglen = desc->arginfo & 0xf;
	int ret, retry_count = 0;
	int retry_count_max = is_restore_cfg(fn_id, desc)?
		SCM_EBUSY_MAX_RETRY_RESTORE_CFG:SCM_EBUSY_MAX_RETRY;
	u64 x0;

	ret = allocate_extra_arg_buffer(desc, GFP_KERNEL);
	if (ret)
		return ret;

	x0 = fn_id | scm_version_mask;

	do {
		mutex_lock(&scm_lock);

		desc->ret[0] = desc->ret[1] = desc->ret[2] = 0;

		pr_debug("scm_call: func id %#llx, args: %#x, %#llx, %#llx, %#llx, %#llx\n",
			x0, desc->arginfo, desc->args[0], desc->args[1],
			desc->args[2], desc->x5);

		if (scm_version == SCM_ARMV8_64)
			ret = __scm_call_armv8_64(x0, desc->arginfo,
						  desc->args[0], desc->args[1],
						  desc->args[2], desc->x5,
						  &desc->ret[0], &desc->ret[1],
						  &desc->ret[2]);
		else
			ret = __scm_call_armv8_32(x0, desc->arginfo,
						  desc->args[0], desc->args[1],
						  desc->args[2], desc->x5,
						  &desc->ret[0], &desc->ret[1],
						  &desc->ret[2]);
		mutex_unlock(&scm_lock);

		if (ret == SCM_V2_EBUSY)
			msleep(SCM_EBUSY_WAIT_MS);
	}  while (ret == SCM_V2_EBUSY && (retry_count++ < retry_count_max));

	if (ret < 0)
		pr_err("scm_call failed: func id %#llx, arginfo: %#x, args: %#llx, %#llx, %#llx, %#llx, ret: %d, syscall returns: %#llx, %#llx, %#llx\n",
			x0, desc->arginfo, desc->args[0], desc->args[1],
			desc->args[2], desc->x5, ret, desc->ret[0],
			desc->ret[1], desc->ret[2]);

	if (arglen > N_REGISTER_ARGS)
		kfree(desc->extra_arg_buf);
	if (ret < 0)
		return scm_remap_error(ret);
	return 0;
}
EXPORT_SYMBOL(scm_call2);

int scm_call2_atomic(u32 fn_id, struct scm_desc *desc)
{
	int arglen = desc->arginfo & 0xf;
	int ret;
	u64 x0;

	ret = allocate_extra_arg_buffer(desc, GFP_ATOMIC);
	if (ret)
		return ret;

	x0 = fn_id | BIT(SMC_ATOMIC_SYSCALL) | scm_version_mask;

	pr_debug("scm_call: func id %#llx, args: %#x, %#llx, %#llx, %#llx, %#llx\n",
		x0, desc->arginfo, desc->args[0], desc->args[1],
		desc->args[2], desc->x5);

	if (scm_version == SCM_ARMV8_64)
		ret = __scm_call_armv8_64(x0, desc->arginfo, desc->args[0],
					  desc->args[1], desc->args[2],
					  desc->x5, &desc->ret[0],
					  &desc->ret[1], &desc->ret[2]);
	else
		ret = __scm_call_armv8_32(x0, desc->arginfo, desc->args[0],
					  desc->args[1], desc->args[2],
					  desc->x5, &desc->ret[0],
					  &desc->ret[1], &desc->ret[2]);
	if (ret < 0)
		pr_err("scm_call failed: func id %#llx, arginfo: %#x, args: %#llx, %#llx, %#llx, %#llx, ret: %d, syscall returns: %#llx, %#llx, %#llx\n",
			x0, desc->arginfo, desc->args[0], desc->args[1],
			desc->args[2], desc->x5, ret, desc->ret[0],
			desc->ret[1], desc->ret[2]);

	if (arglen > N_REGISTER_ARGS)
		kfree(desc->extra_arg_buf);
	if (ret < 0)
		return scm_remap_error(ret);
	return ret;
}

int scm_call(u32 svc_id, u32 cmd_id, const void *cmd_buf, size_t cmd_len,
		void *resp_buf, size_t resp_len)
{
	struct scm_command *cmd;
	int ret;
	size_t len = SCM_BUF_LEN(cmd_len, resp_len);

	if (cmd_len > len || resp_len > len)
		return -EINVAL;

	cmd = kzalloc(PAGE_ALIGN(len), GFP_KERNEL);
	if (!cmd)
		return -ENOMEM;

	ret = scm_call_common(svc_id, cmd_id, cmd_buf, cmd_len, resp_buf,
				resp_len, cmd, len);
	if (unlikely(ret == SCM_EBUSY))
		ret = _scm_call_retry(svc_id, cmd_id, cmd_buf, cmd_len,
				      resp_buf, resp_len, cmd, PAGE_ALIGN(len));
	kfree(cmd);
	return ret;
}
EXPORT_SYMBOL(scm_call);

#define SCM_CLASS_REGISTER	(0x2 << 8)
#define SCM_MASK_IRQS		BIT(5)
#define SCM_ATOMIC(svc, cmd, n) (((((svc) << 10)|((cmd) & 0x3ff)) << 12) | \
				SCM_CLASS_REGISTER | \
				SCM_MASK_IRQS | \
				(n & 0xf))

s32 scm_call_atomic1(u32 svc, u32 cmd, u32 arg1)
{
	int context_id;
	register u32 r0 asm("r0") = SCM_ATOMIC(svc, cmd, 1);
	register u32 r1 asm("r1") = (uintptr_t)&context_id;
	register u32 r2 asm("r2") = arg1;

	asm volatile(
		__asmeq("%0", R0_STR)
		__asmeq("%1", R0_STR)
		__asmeq("%2", R1_STR)
		__asmeq("%3", R2_STR)
#ifdef REQUIRES_SEC
			".arch_extension sec\n"
#endif
		"smc	#0\n"
		: "=r" (r0)
		: "r" (r0), "r" (r1), "r" (r2)
		: "r3");
	return r0;
}
EXPORT_SYMBOL(scm_call_atomic1);

s32 scm_call_atomic1_1(u32 svc, u32 cmd, u32 arg1, u32 *ret1)
{
	int context_id;
	register u32 r0 asm("r0") = SCM_ATOMIC(svc, cmd, 1);
	register u32 r1 asm("r1") = (uintptr_t)&context_id;
	register u32 r2 asm("r2") = arg1;

	asm volatile(
		__asmeq("%0", R0_STR)
		__asmeq("%1", R1_STR)
		__asmeq("%2", R0_STR)
		__asmeq("%3", R1_STR)
		__asmeq("%4", R2_STR)
#ifdef REQUIRES_SEC
			".arch_extension sec\n"
#endif
		"smc	#0\n"
		: "=r" (r0), "=r" (r1)
		: "r" (r0), "r" (r1), "r" (r2)
		: "r3");
	if (ret1)
		*ret1 = r1;
	return r0;
}
EXPORT_SYMBOL(scm_call_atomic1_1);

s32 scm_call_atomic2(u32 svc, u32 cmd, u32 arg1, u32 arg2)
{
	int context_id;
	register u32 r0 asm("r0") = SCM_ATOMIC(svc, cmd, 2);
	register u32 r1 asm("r1") = (uintptr_t)&context_id;
	register u32 r2 asm("r2") = arg1;
	register u32 r3 asm("r3") = arg2;

	asm volatile(
		__asmeq("%0", R0_STR)
		__asmeq("%1", R0_STR)
		__asmeq("%2", R1_STR)
		__asmeq("%3", R2_STR)
		__asmeq("%4", R3_STR)
#ifdef REQUIRES_SEC
			".arch_extension sec\n"
#endif
		"smc	#0\n"
		: "=r" (r0)
		: "r" (r0), "r" (r1), "r" (r2), "r" (r3));
	return r0;
}
EXPORT_SYMBOL(scm_call_atomic2);

s32 scm_call_atomic3(u32 svc, u32 cmd, u32 arg1, u32 arg2, u32 arg3)
{
	int context_id;
	register u32 r0 asm("r0") = SCM_ATOMIC(svc, cmd, 3);
	register u32 r1 asm("r1") = (uintptr_t)&context_id;
	register u32 r2 asm("r2") = arg1;
	register u32 r3 asm("r3") = arg2;
	register u32 r4 asm("r4") = arg3;

	asm volatile(
		__asmeq("%0", R0_STR)
		__asmeq("%1", R0_STR)
		__asmeq("%2", R1_STR)
		__asmeq("%3", R2_STR)
		__asmeq("%4", R3_STR)
		__asmeq("%5", R4_STR)
#ifdef REQUIRES_SEC
			".arch_extension sec\n"
#endif
		"smc	#0\n"
		: "=r" (r0)
		: "r" (r0), "r" (r1), "r" (r2), "r" (r3), "r" (r4));
	return r0;
}
EXPORT_SYMBOL(scm_call_atomic3);

s32 scm_call_atomic4_3(u32 svc, u32 cmd, u32 arg1, u32 arg2,
		u32 arg3, u32 arg4, u32 *ret1, u32 *ret2)
{
	int ret;
	int context_id;
	register u32 r0 asm("r0") = SCM_ATOMIC(svc, cmd, 4);
	register u32 r1 asm("r1") = (uintptr_t)&context_id;
	register u32 r2 asm("r2") = arg1;
	register u32 r3 asm("r3") = arg2;
	register u32 r4 asm("r4") = arg3;
	register u32 r5 asm("r5") = arg4;

	asm volatile(
		__asmeq("%0", R0_STR)
		__asmeq("%1", R1_STR)
		__asmeq("%2", R2_STR)
		__asmeq("%3", R0_STR)
		__asmeq("%4", R1_STR)
		__asmeq("%5", R2_STR)
		__asmeq("%6", R3_STR)
#ifdef REQUIRES_SEC
			".arch_extension sec\n"
#endif
		"smc	#0\n"
		: "=r" (r0), "=r" (r1), "=r" (r2)
		: "r" (r0), "r" (r1), "r" (r2), "r" (r3), "r" (r4), "r" (r5));
	ret = r0;
	if (ret1)
		*ret1 = r1;
	if (ret2)
		*ret2 = r2;
	return r0;
}
EXPORT_SYMBOL(scm_call_atomic4_3);

s32 scm_call_atomic5_3(u32 svc, u32 cmd, u32 arg1, u32 arg2,
	u32 arg3, u32 arg4, u32 arg5, u32 *ret1, u32 *ret2, u32 *ret3)
{
	int ret;
	int context_id;
	register u32 r0 asm("r0") = SCM_ATOMIC(svc, cmd, 5);
	register u32 r1 asm("r1") = (uintptr_t)&context_id;
	register u32 r2 asm("r2") = arg1;
	register u32 r3 asm("r3") = arg2;
	register u32 r4 asm("r4") = arg3;
	register u32 r5 asm("r5") = arg4;
	register u32 r6 asm("r6") = arg5;

	asm volatile(
		__asmeq("%0", R0_STR)
		__asmeq("%1", R1_STR)
		__asmeq("%2", R2_STR)
		__asmeq("%3", R3_STR)
		__asmeq("%4", R0_STR)
		__asmeq("%5", R1_STR)
		__asmeq("%6", R2_STR)
		__asmeq("%7", R3_STR)
#ifdef REQUIRES_SEC
			".arch_extension sec\n"
#endif
		"smc	#0\n"
		: "=r" (r0), "=r" (r1), "=r" (r2), "=r" (r3)
		: "r" (r0), "r" (r1), "r" (r2), "r" (r3), "r" (r4), "r" (r5),
		 "r" (r6));
	ret = r0;

	if (ret1)
		*ret1 = r1;
	if (ret2)
		*ret2 = r2;
	if (ret3)
		*ret3 = r3;
	return r0;
}
EXPORT_SYMBOL(scm_call_atomic5_3);

u32 scm_get_version(void)
{
	int context_id;
	static u32 version = -1;
	register u32 r0 asm("r0");
	register u32 r1 asm("r1");

	if (version != -1)
		return version;

	mutex_lock(&scm_lock);

	r0 = 0x1 << 8;
	r1 = (uintptr_t)&context_id;
	do {
		asm volatile(
			__asmeq("%0", R0_STR)
			__asmeq("%1", R1_STR)
			__asmeq("%2", R0_STR)
			__asmeq("%3", R1_STR)
#ifdef REQUIRES_SEC
			".arch_extension sec\n"
#endif
			"smc	#0\n"
			: "=r" (r0), "=r" (r1)
			: "r" (r0), "r" (r1)
			: "r2", "r3");
	} while (r0 == SCM_INTERRUPTED);

	version = r1;
	mutex_unlock(&scm_lock);

	return version;
}
EXPORT_SYMBOL(scm_get_version);

#define SCM_IO_READ	0x1
#define SCM_IO_WRITE	0x2

u32 scm_io_read(phys_addr_t address)
{
	if (!is_scm_armv8()) {
		return scm_call_atomic1(SCM_SVC_IO, SCM_IO_READ, address);
	} else {
		struct scm_desc desc = {
			.args[0] = address,
			.arginfo = SCM_ARGS(1),
		};
		scm_call2_atomic(SCM_SIP_FNID(SCM_SVC_IO, SCM_IO_READ), &desc);
		return desc.ret[0];
	}
}
EXPORT_SYMBOL(scm_io_read);

int scm_io_write(phys_addr_t address, u32 val)
{
	int ret;

	if (!is_scm_armv8()) {
		ret = scm_call_atomic2(SCM_SVC_IO, SCM_IO_WRITE, address, val);
	} else {
		struct scm_desc desc = {
			.args[0] = address,
			.args[1] = val,
			.arginfo = SCM_ARGS(2),
		};
		ret = scm_call2_atomic(SCM_SIP_FNID(SCM_SVC_IO, SCM_IO_WRITE),
				       &desc);
	}
	return ret;
}
EXPORT_SYMBOL(scm_io_write);

int scm_is_call_available(u32 svc_id, u32 cmd_id)
{
	int ret;
	struct scm_desc desc = {0};

	if (!is_scm_armv8()) {
		u32 ret_val = 0;
		u32 svc_cmd = (svc_id << 10) | cmd_id;

		ret = scm_call(SCM_SVC_INFO, IS_CALL_AVAIL_CMD, &svc_cmd,
			sizeof(svc_cmd), &ret_val, sizeof(ret_val));
		if (ret)
			return ret;

		return ret_val;
	}
	desc.arginfo = SCM_ARGS(1);
	desc.args[0] = SCM_SIP_FNID(svc_id, cmd_id);
	ret = scm_call2(SCM_SIP_FNID(SCM_SVC_INFO, IS_CALL_AVAIL_CMD), &desc);
	if (ret)
		return ret;

	return desc.ret[0];
}
EXPORT_SYMBOL(scm_is_call_available);

#define GET_FEAT_VERSION_CMD	3
int scm_get_feat_version(u32 feat)
{
	struct scm_desc desc = {0};
	int ret;

	if (!is_scm_armv8()) {
		if (scm_is_call_available(SCM_SVC_INFO, GET_FEAT_VERSION_CMD)) {
			u32 version;
			if (!scm_call(SCM_SVC_INFO, GET_FEAT_VERSION_CMD, &feat,
				      sizeof(feat), &version, sizeof(version)))
				return version;
		}
		return 0;
	}

	ret = scm_is_call_available(SCM_SVC_INFO, GET_FEAT_VERSION_CMD);
	if (ret <= 0)
		return 0;

	desc.args[0] = feat;
	desc.arginfo = SCM_ARGS(1);
	ret = scm_call2(SCM_SIP_FNID(SCM_SVC_INFO, GET_FEAT_VERSION_CMD),
			&desc);
	if (!ret)
		return desc.ret[0];

	return 0;
}
EXPORT_SYMBOL(scm_get_feat_version);

int scm_restore_sec_cfg(u32 device_id, u32 spare, int *scm_ret)
{
	struct scm_desc desc = {0};
	int ret;
	struct restore_sec_cfg {
		u32 device_id;
		u32 spare;
	} cfg;

	cfg.device_id = device_id;
	cfg.spare = spare;

	if (IS_ERR_OR_NULL(scm_ret))
		return -EINVAL;

	if (!is_scm_armv8())
		return scm_call(SCM_SVC_MP, RESTORE_SEC_CFG, &cfg, sizeof(cfg),
				scm_ret, sizeof(*scm_ret));

	desc.args[0] = device_id;
	desc.args[1] = spare;
	desc.arginfo = SCM_ARGS(2);

	ret = scm_call2(SCM_SIP_FNID(SCM_SVC_MP, RESTORE_SEC_CFG), &desc);
	if (ret)
		return ret;

	*scm_ret = desc.ret[0];
	return 0;
}
EXPORT_SYMBOL(scm_restore_sec_cfg);
