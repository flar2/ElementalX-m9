/* Copyright (c) 2002,2007-2014, The Linux Foundation. All rights reserved.
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
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/log2.h>
#include <linux/time.h>
#include <linux/delay.h>

#include "kgsl.h"
#include "kgsl_sharedmem.h"
#include "kgsl_cffdump.h"
#include "kgsl_trace.h"
#include "kgsl_pwrctrl.h"

#include "adreno.h"
#include "adreno_pm4types.h"
#include "adreno_ringbuffer.h"

#include "a3xx_reg.h"

#define GSL_RB_NOP_SIZEDWORDS				2

#define RB_HOSTPTR(_rb, _pos) \
	((unsigned int *) ((_rb)->buffer_desc.hostptr + \
		((_pos) * sizeof(unsigned int))))

#define RB_GPUADDR(_rb, _pos) \
	((_rb)->buffer_desc.gpuaddr + ((_pos) * sizeof(unsigned int)))

static void _cff_write_ringbuffer(struct adreno_ringbuffer *rb)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(rb->device);
	struct kgsl_device *device = &adreno_dev->dev;
	unsigned int gpuaddr;
	unsigned int *hostptr;
	size_t size;

	if (device->cff_dump_enable == 0)
		return;

	BUG_ON(rb->wptr < rb->last_wptr);

	size = (rb->wptr - rb->last_wptr) * sizeof(unsigned int);

	hostptr = RB_HOSTPTR(rb, rb->last_wptr);
	gpuaddr = RB_GPUADDR(rb, rb->last_wptr);

	kgsl_cffdump_memcpy(device, gpuaddr, hostptr, size);
}

void adreno_ringbuffer_submit(struct adreno_ringbuffer *rb,
		struct adreno_submit_time *time)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(rb->device);
	struct adreno_gpudev *gpudev = ADRENO_GPU_DEVICE(adreno_dev);
	BUG_ON(rb->wptr == 0);

	kgsl_pwrscale_busy(rb->device);

	
	_cff_write_ringbuffer(rb);


	if (time != NULL) {
		unsigned long flags;
		local_irq_save(flags);

		if (gpudev->alwayson_counter_read != NULL)
			time->ticks = gpudev->alwayson_counter_read(adreno_dev);
		else
			time->ticks = 0;

		
		time->ktime = local_clock();

		
		getnstimeofday(&time->utime);

		local_irq_restore(flags);
	}

	
	mb();

	adreno_writereg(adreno_dev, ADRENO_REG_CP_RB_WPTR, rb->wptr);
}

static int
adreno_ringbuffer_waitspace(struct adreno_ringbuffer *rb,
				unsigned int numcmds, int wptr_ahead)
{
	int nopcount;
	unsigned int freecmds;
	unsigned int *cmds;
	unsigned int gpuaddr;
	unsigned long wait_time;
	unsigned long wait_timeout = msecs_to_jiffies(ADRENO_IDLE_TIMEOUT);
	unsigned long wait_time_part;
	unsigned int rptr;

	
	if (wptr_ahead) {
		
		nopcount = KGSL_RB_DWORDS - rb->wptr - 1;

		cmds = RB_HOSTPTR(rb, rb->wptr);
		gpuaddr = RB_GPUADDR(rb, rb->wptr);

		*cmds = cp_nop_packet(nopcount);
		kgsl_cffdump_write(rb->device, gpuaddr, *cmds);

		do {
			rptr = adreno_get_rptr(rb);
		} while (!rptr);

		rb->wptr = 0;
	}

	wait_time = jiffies + wait_timeout;
	wait_time_part = jiffies + msecs_to_jiffies(KGSL_TIMEOUT_PART);
	
	while (1) {
		rptr = adreno_get_rptr(rb);

		freecmds = rptr - rb->wptr;

		if (freecmds == 0 || freecmds > numcmds)
			break;

		if (time_after(jiffies, wait_time)) {
			KGSL_DRV_ERR(rb->device,
			"Timed out while waiting for freespace in ringbuffer "
			"rptr: 0x%x, wptr: 0x%x\n", rptr, rb->wptr);
			return -ETIMEDOUT;
		}

	}
	return 0;
}

unsigned int *adreno_ringbuffer_allocspace(struct adreno_ringbuffer *rb,
					unsigned int numcmds)
{
	unsigned int *ptr = NULL;
	int ret = 0;
	unsigned int rptr;
	BUG_ON(numcmds >= KGSL_RB_DWORDS);

	rptr = adreno_get_rptr(rb);
	
	if (rb->wptr >= rptr) {
		
		
		if ((rb->wptr + numcmds) > (KGSL_RB_DWORDS -
				GSL_RB_NOP_SIZEDWORDS))
			ret = adreno_ringbuffer_waitspace(rb, numcmds, 1);
	} else {
		
		if ((rb->wptr + numcmds) >= rptr)
			ret = adreno_ringbuffer_waitspace(rb, numcmds, 0);
		
		
		if (!ret && (rb->wptr + numcmds) > (KGSL_RB_DWORDS -
				GSL_RB_NOP_SIZEDWORDS))
			ret = adreno_ringbuffer_waitspace(rb, numcmds, 1);
	}

	if (!ret) {
		rb->last_wptr = rb->wptr;

		ptr = (unsigned int *)rb->buffer_desc.hostptr + rb->wptr;
		rb->wptr += numcmds;
	} else
		ptr = ERR_PTR(ret);

	return ptr;
}

#define FW_LOOP_COUNT	100	
#define FW_LOOP_PERIOD	50	

static int _load_firmware(struct kgsl_device *device, const char *fwfile,
			  void **data, int *len)
{
	const struct firmware *fw = NULL;
	int ret;
	int i = FW_LOOP_COUNT;

	while (i--) {
		ret = request_firmware(&fw, fwfile, device->dev);
		if (ret)
			usleep_range(FW_LOOP_PERIOD * 1000, FW_LOOP_PERIOD * 1000 + 500);
		else
			break;
	}

	if (ret) {
		KGSL_DRV_ERR(device, "request_firmware(%s) failed for %d times: %d\n",
			fwfile, FW_LOOP_COUNT, ret);
		return ret;
	}

	*data = kmalloc(fw->size, GFP_KERNEL);

	if (*data) {
		memcpy(*data, fw->data, fw->size);
		*len = fw->size;
	}

	release_firmware(fw);
	return (*data != NULL) ? 0 : -ENOMEM;
}

void adreno_ringbuffer_read_pm4_ucode(struct kgsl_device *device)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);

	if (adreno_dev->pm4_fw == NULL) {
		int len;
		void *ptr;

		int ret = _load_firmware(device,
			adreno_dev->gpucore->pm4fw_name, &ptr, &len);

		if (ret)
			goto err;

		
		if (len % ((sizeof(uint32_t) * 3)) != sizeof(uint32_t)) {
			KGSL_DRV_ERR(device, "Bad pm4 microcode size: %d\n",
				len);
			kfree(ptr);
			goto err;
		}

		adreno_dev->pm4_fw_size = len / sizeof(uint32_t);
		adreno_dev->pm4_fw = ptr;
		adreno_dev->pm4_fw_version = adreno_dev->pm4_fw[1];
	}

	return;

err:
	KGSL_DRV_FATAL(device, "Failed to read pm4 microcode %s\n",
		adreno_dev->gpucore->pm4fw_name);
}

static inline int adreno_ringbuffer_load_pm4_ucode(struct kgsl_device *device,
			unsigned int start, unsigned int end, unsigned int addr)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	int i;

	adreno_writereg(adreno_dev, ADRENO_REG_CP_ME_RAM_WADDR, addr);
	for (i = start; i < end; i++)
		adreno_writereg(adreno_dev, ADRENO_REG_CP_ME_RAM_DATA,
					adreno_dev->pm4_fw[i]);

	return 0;
}

void adreno_ringbuffer_read_pfp_ucode(struct kgsl_device *device)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);

	if (adreno_dev->pfp_fw == NULL) {
		int len;
		void *ptr;

		int ret = _load_firmware(device,
			adreno_dev->gpucore->pfpfw_name, &ptr, &len);
		if (ret)
			goto err;

		
		if (len % sizeof(uint32_t) != 0) {
			KGSL_DRV_ERR(device, "Bad PFP microcode size: %d\n",
				len);
			kfree(ptr);
			goto err;
		}

		adreno_dev->pfp_fw_size = len / sizeof(uint32_t);
		adreno_dev->pfp_fw = ptr;
		adreno_dev->pfp_fw_version = adreno_dev->pfp_fw[5];
	}

	return;

err:
	KGSL_DRV_FATAL(device, "Failed to read pfp microcode %s\n",
		adreno_dev->gpucore->pfpfw_name);
}

static inline int adreno_ringbuffer_load_pfp_ucode(struct kgsl_device *device,
			unsigned int start, unsigned int end, unsigned int addr)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	int i;

	adreno_writereg(adreno_dev, ADRENO_REG_CP_PFP_UCODE_ADDR, addr);
	for (i = start; i < end; i++)
		adreno_writereg(adreno_dev, ADRENO_REG_CP_PFP_UCODE_DATA,
						adreno_dev->pfp_fw[i]);

	return 0;
}

static int _ringbuffer_bootstrap_ucode(struct adreno_ringbuffer *rb,
					unsigned int load_jt)
{
	unsigned int *cmds, bootstrap_size, rb_size;
	int i = 0;
	int ret;
	struct kgsl_device *device = rb->device;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	unsigned int pm4_size, pm4_idx, pm4_addr, pfp_size, pfp_idx, pfp_addr;

	
	if (load_jt) {
		pm4_idx = adreno_dev->gpucore->pm4_jt_idx;
		pm4_addr = adreno_dev->gpucore->pm4_jt_addr;
		pfp_idx = adreno_dev->gpucore->pfp_jt_idx;
		pfp_addr = adreno_dev->gpucore->pfp_jt_addr;
	} else {
		
		pm4_idx = 1;
		pm4_addr = 0;
		pfp_idx = 1;
		pfp_addr = 0;
	}

	pm4_size = (adreno_dev->pm4_fw_size - pm4_idx);
	pfp_size = (adreno_dev->pfp_fw_size - pfp_idx);

	bootstrap_size = (pm4_size + pfp_size + 5);


	if (adreno_is_a4xx(adreno_dev)) {
		adreno_writereg(adreno_dev, ADRENO_REG_CP_PFP_UCODE_ADDR,
			0x400);
		adreno_writereg(adreno_dev, ADRENO_REG_CP_PFP_UCODE_DATA,
			 0x6f0009);

		rb_size = bootstrap_size + 6;
	} else {
		adreno_writereg(adreno_dev, ADRENO_REG_CP_PFP_UCODE_ADDR,
			0x200);
		adreno_writereg(adreno_dev, ADRENO_REG_CP_PFP_UCODE_DATA,
			 0x6f0005);
		rb_size = bootstrap_size;
	}

	
	adreno_writereg(adreno_dev, ADRENO_REG_CP_ME_CNTL, 0);

	cmds = adreno_ringbuffer_allocspace(rb, rb_size);
	if (IS_ERR(cmds))
		return PTR_ERR(cmds);
	if (cmds == NULL)
		return -ENOSPC;

	
	*cmds++ = cp_type3_packet(CP_BOOTSTRAP_UCODE, (bootstrap_size - 1));
	*cmds++ = pfp_size;
	*cmds++ = pfp_addr;
	*cmds++ = pm4_size;
	*cmds++ = pm4_addr;

	if (adreno_is_a4xx(adreno_dev)) {
		for (i = pm4_idx; i < adreno_dev->pm4_fw_size; i++)
			*cmds++ = adreno_dev->pm4_fw[i];
		for (i = pfp_idx; i < adreno_dev->pfp_fw_size; i++)
			*cmds++ = adreno_dev->pfp_fw[i];

		*cmds++ = cp_type3_packet(CP_REG_RMW, 3);
		*cmds++ = 0x20000000 + A4XX_CP_RB_WPTR;
		*cmds++ = 0xffffffff;
		*cmds++ = 0x00000002;
		*cmds++ = cp_type3_packet(CP_INTERRUPT, 1);
		*cmds++ = 0;

		rb->wptr = rb->wptr - 2;
		adreno_ringbuffer_submit(rb, NULL);
		rb->wptr = rb->wptr + 2;
	} else {
		for (i = pfp_idx; i < adreno_dev->pfp_fw_size; i++)
			*cmds++ = adreno_dev->pfp_fw[i];
		for (i = pm4_idx; i < adreno_dev->pm4_fw_size; i++)
			*cmds++ = adreno_dev->pm4_fw[i];
		adreno_ringbuffer_submit(rb, NULL);
	}

	
	ret = adreno_spin_idle(device);

	
	if (adreno_is_a430(adreno_dev))
		kgsl_regwrite(device, A4XX_CP_DEBUG,
					A4XX_CP_DEBUG_DEFAULT & ~(1 << 14));

	return ret;
}

static void _ringbuffer_setup_common(struct adreno_ringbuffer *rb)
{
	struct kgsl_device *device = rb->device;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct adreno_ringbuffer *rb_temp;
	int i;

	FOR_EACH_RINGBUFFER(adreno_dev, rb_temp, i) {
		kgsl_sharedmem_set(rb_temp->device,
			&(rb_temp->buffer_desc), 0,
			0xAA, KGSL_RB_SIZE);
		rb_temp->wptr = 0;
		rb_temp->rptr = 0;
		adreno_iommu_set_pt_generate_rb_cmds(rb_temp,
					device->mmu.defaultpagetable);
	}


	adreno_writereg(adreno_dev, ADRENO_REG_CP_RB_CNTL,
		(ilog2(KGSL_RB_DWORDS >> 1) & 0x3F) |
		(1 << 27));

	adreno_writereg(adreno_dev, ADRENO_REG_CP_RB_BASE,
					rb->buffer_desc.gpuaddr);

	
	if (adreno_is_a305(adreno_dev) || adreno_is_a305c(adreno_dev) ||
		adreno_is_a306(adreno_dev) || adreno_is_a320(adreno_dev) ||
		adreno_is_a304(adreno_dev))
		kgsl_regwrite(device, A3XX_CP_QUEUE_THRESHOLDS, 0x000E0602);
	else if (adreno_is_a330(adreno_dev) || adreno_is_a305b(adreno_dev) ||
			adreno_is_a310(adreno_dev))
		kgsl_regwrite(device, A3XX_CP_QUEUE_THRESHOLDS, 0x003E2008);
}

static int _ringbuffer_start_common(struct adreno_ringbuffer *rb)
{
	int status;
	struct kgsl_device *device = rb->device;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct adreno_gpudev *gpudev = ADRENO_GPU_DEVICE(adreno_dev);

	
	status = gpudev->rb_init(adreno_dev, rb);
	if (status)
		return status;

	
	return adreno_spin_idle(device);
}

int adreno_ringbuffer_warm_start(struct adreno_device *adreno_dev)
{
	int status;
	struct adreno_ringbuffer *rb = ADRENO_CURRENT_RINGBUFFER(adreno_dev);
	struct kgsl_device *device = rb->device;

	_ringbuffer_setup_common(rb);

	
	if (adreno_bootstrap_ucode(adreno_dev)) {
		status = _ringbuffer_bootstrap_ucode(rb, 1);
		if (status != 0)
			return status;

	} else {
		
		status = adreno_ringbuffer_load_pm4_ucode(device,
			adreno_dev->gpucore->pm4_jt_idx,
			adreno_dev->pm4_fw_size,
			adreno_dev->gpucore->pm4_jt_addr);
		if (status != 0)
			return status;

		
		status = adreno_ringbuffer_load_pfp_ucode(device,
			adreno_dev->gpucore->pfp_jt_idx,
			adreno_dev->pfp_fw_size,
			adreno_dev->gpucore->pfp_jt_addr);
		if (status != 0)
			return status;

		
		adreno_writereg(adreno_dev, ADRENO_REG_CP_ME_CNTL, 0);
	}

	return _ringbuffer_start_common(rb);
}

int adreno_ringbuffer_cold_start(struct adreno_device *adreno_dev)
{
	int status;
	struct adreno_ringbuffer *rb = ADRENO_CURRENT_RINGBUFFER(adreno_dev);


	_ringbuffer_setup_common(rb);

	
	if (adreno_bootstrap_ucode(adreno_dev)) {


		status = adreno_ringbuffer_load_pm4_ucode(rb->device, 1,
			adreno_dev->gpucore->pm4_bstrp_size+1, 0);
		if (status != 0)
			return status;

		status = adreno_ringbuffer_load_pfp_ucode(rb->device, 1,
			adreno_dev->gpucore->pfp_bstrp_size+1, 0);
		if (status != 0)
			return status;

		
		status = _ringbuffer_bootstrap_ucode(rb, 0);
		if (status != 0)
			return status;

	} else {
		
		status = adreno_ringbuffer_load_pm4_ucode(rb->device, 1,
					adreno_dev->pm4_fw_size, 0);
		if (status != 0)
			return status;

		
		status = adreno_ringbuffer_load_pfp_ucode(rb->device, 1,
					adreno_dev->pfp_fw_size, 0);
		if (status != 0)
			return status;

		
		adreno_writereg(adreno_dev, ADRENO_REG_CP_ME_CNTL, 0);
	}

	return _ringbuffer_start_common(rb);
}

void adreno_ringbuffer_stop(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = &adreno_dev->dev;
	struct adreno_ringbuffer *rb;
	int i;
	FOR_EACH_RINGBUFFER(adreno_dev, rb, i)
		kgsl_cancel_events(device, &(rb->events));
}

static int _adreno_ringbuffer_init(struct adreno_device *adreno_dev,
				struct adreno_ringbuffer *rb, int id)
{
	int ret;
	char name[64];

	rb->device = &adreno_dev->dev;
	rb->id = id;

	snprintf(name, sizeof(name), "rb_events-%d", id);
	kgsl_add_event_group(&rb->events, NULL, name,
		adreno_rb_readtimestamp, rb);
	rb->timestamp = 0;
	init_waitqueue_head(&rb->ts_expire_waitq);

	ret = kgsl_allocate_global(&adreno_dev->dev, &rb->pagetable_desc,
		PAGE_SIZE, 0, KGSL_MEMDESC_PRIVILEGED);
	if (ret)
		return ret;

	ret = kgsl_allocate_global(&adreno_dev->dev, &rb->buffer_desc,
			KGSL_RB_SIZE, KGSL_MEMFLAGS_GPUREADONLY, 0);
	return ret;
}

int adreno_ringbuffer_init(struct kgsl_device *device)
{
	int status = 0;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct adreno_gpudev *gpudev = ADRENO_GPU_DEVICE(adreno_dev);
	struct adreno_ringbuffer *rb;
	int i;

	adreno_dev->num_ringbuffers = gpudev->num_prio_levels;
	FOR_EACH_RINGBUFFER(adreno_dev, rb, i) {
		status = _adreno_ringbuffer_init(adreno_dev, rb, i);
		if (status)
			break;
	}
	if (status)
		adreno_ringbuffer_close(adreno_dev);
	else
		adreno_dev->cur_rb = &(adreno_dev->ringbuffers[0]);

	return status;
}

static void _adreno_ringbuffer_close(struct adreno_ringbuffer *rb)
{
	if (rb->pagetable_desc.hostptr)
		kgsl_free_global(&rb->pagetable_desc);

	memset(&rb->pt_update_desc, 0, sizeof(struct kgsl_memdesc));

	if (rb->buffer_desc.hostptr)
		kgsl_free_global(&rb->buffer_desc);
	kgsl_del_event_group(&rb->events);
	memset(rb, 0, sizeof(struct adreno_ringbuffer));
}

void adreno_ringbuffer_close(struct adreno_device *adreno_dev)
{
	struct adreno_ringbuffer *rb;
	int i;

	kfree(adreno_dev->pfp_fw);
	kfree(adreno_dev->pm4_fw);

	adreno_dev->pfp_fw = NULL;
	adreno_dev->pm4_fw = NULL;

	FOR_EACH_RINGBUFFER(adreno_dev, rb, i)
		_adreno_ringbuffer_close(rb);
}

static int
adreno_ringbuffer_addcmds(struct adreno_ringbuffer *rb,
				unsigned int flags, unsigned int *cmds,
				int sizedwords, uint32_t timestamp,
				struct adreno_submit_time *time)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(rb->device);
	unsigned int *ringcmds;
	unsigned int total_sizedwords = sizedwords;
	unsigned int i;
	unsigned int context_id = 0;
	unsigned int gpuaddr = rb->device->memstore.gpuaddr;
	bool profile_ready;
	struct adreno_context *drawctxt = rb->drawctxt_active;
	bool secured_ctxt = false;

	if (drawctxt != NULL && kgsl_context_detached(&drawctxt->base) &&
		!(flags & KGSL_CMD_FLAGS_INTERNAL_ISSUE))
		return -EINVAL;

	rb->timestamp++;

	
	if (!drawctxt || (flags & KGSL_CMD_FLAGS_INTERNAL_ISSUE))
		timestamp = rb->timestamp;
	else
		context_id = drawctxt->base.id;

	if (drawctxt) {
		drawctxt->internal_timestamp = rb->timestamp;
		if (drawctxt->base.flags & KGSL_CONTEXT_SECURE)
			secured_ctxt = true;
	}

	profile_ready = drawctxt &&
		adreno_profile_assignments_ready(&adreno_dev->profile) &&
		!(flags & KGSL_CMD_FLAGS_INTERNAL_ISSUE);

	total_sizedwords += flags & KGSL_CMD_FLAGS_PMODE ? 4 : 0;
	
	total_sizedwords += 2;
	
	total_sizedwords += (flags & KGSL_CMD_FLAGS_INTERNAL_ISSUE) ? 2 : 0;

	total_sizedwords += (secured_ctxt) ? 26 : 0;

	
	total_sizedwords +=
		(drawctxt || (flags & KGSL_CMD_FLAGS_INTERNAL_ISSUE)) ?  2 : 0;

	
	if (adreno_is_a3xx(adreno_dev))
		total_sizedwords += 3;

	
	if (adreno_is_a4xx(adreno_dev) || adreno_is_a3xx(adreno_dev))
		total_sizedwords += 4;

	total_sizedwords += 3; 
	total_sizedwords += 4; 

	if (drawctxt && !(flags & KGSL_CMD_FLAGS_INTERNAL_ISSUE)) {
		total_sizedwords += 3; 
	}

	if (flags & KGSL_CMD_FLAGS_WFI)
		total_sizedwords += 2; 

	if (profile_ready)
		total_sizedwords += 6;   

	
	if (flags & KGSL_CMD_FLAGS_PWRON_FIXUP)
		total_sizedwords += 9;

	ringcmds = adreno_ringbuffer_allocspace(rb, total_sizedwords);
	if (IS_ERR(ringcmds))
		return PTR_ERR(ringcmds);
	if (ringcmds == NULL)
		return -ENOSPC;

	*ringcmds++ = cp_nop_packet(1);
	*ringcmds++ = KGSL_CMD_IDENTIFIER;

	if (flags & KGSL_CMD_FLAGS_INTERNAL_ISSUE) {
		*ringcmds++ = cp_nop_packet(1);
		*ringcmds++ = KGSL_CMD_INTERNAL_IDENTIFIER;
	}

	if (flags & KGSL_CMD_FLAGS_PWRON_FIXUP) {
		
		*ringcmds++ = cp_type3_packet(CP_SET_PROTECTED_MODE, 1);
		*ringcmds++ = 0;

		*ringcmds++ = cp_nop_packet(1);
		*ringcmds++ = KGSL_PWRON_FIXUP_IDENTIFIER;
		*ringcmds++ = CP_HDR_INDIRECT_BUFFER_PFE;
		*ringcmds++ = adreno_dev->pwron_fixup.gpuaddr;
		*ringcmds++ = adreno_dev->pwron_fixup_dwords;

		
		*ringcmds++ = cp_type3_packet(CP_SET_PROTECTED_MODE, 1);
		*ringcmds++ = 1;
	}

	
	if (profile_ready)
		adreno_profile_preib_processing(adreno_dev, drawctxt,
				&flags, &ringcmds);

	
	*ringcmds++ = cp_type3_packet(CP_MEM_WRITE, 2);
	if (drawctxt && !(flags & KGSL_CMD_FLAGS_INTERNAL_ISSUE))
		*ringcmds++ = gpuaddr +
			KGSL_MEMSTORE_OFFSET(context_id, soptimestamp);
	else
		*ringcmds++ = gpuaddr +
			KGSL_MEMSTORE_RB_OFFSET(rb, soptimestamp);
	*ringcmds++ = timestamp;

	if (secured_ctxt) {
		*ringcmds++ = cp_type3_packet(CP_WAIT_FOR_IDLE, 1);
		*ringcmds++ = 0x00000000;
		*ringcmds++ = cp_type3_packet(CP_WAIT_FOR_ME, 1);
		*ringcmds++ = 0x00000000;
		*ringcmds++ = cp_type3_packet(CP_SET_PROTECTED_MODE, 1);
		*ringcmds++ = 0;
		*ringcmds++ = cp_type3_packet(CP_WIDE_REG_WRITE, 2);
		*ringcmds++ = A4XX_RBBM_SECVID_TRUST_CONTROL;
		*ringcmds++ = 1;
		*ringcmds++ = cp_type3_packet(CP_SET_PROTECTED_MODE, 1);
		*ringcmds++ = 1;
		
		*ringcmds++ = cp_type3_packet(CP_WAIT_FOR_ME, 1);
		*ringcmds++ = 0x00000000;
	}

	if (flags & KGSL_CMD_FLAGS_PMODE) {
		
		*ringcmds++ = cp_type3_packet(CP_SET_PROTECTED_MODE, 1);
		*ringcmds++ = 0;
	}

	for (i = 0; i < sizedwords; i++)
		*ringcmds++ = cmds[i];

	if (flags & KGSL_CMD_FLAGS_PMODE) {
		
		*ringcmds++ = cp_type3_packet(CP_SET_PROTECTED_MODE, 1);
		*ringcmds++ = 1;
	}


	*ringcmds++ = cp_type3_packet(CP_EVENT_WRITE, 1);
	*ringcmds++ = 0x07; 
	*ringcmds++ = cp_type3_packet(CP_WAIT_FOR_IDLE, 1);
	*ringcmds++ = 0x00;

	if (profile_ready)
		adreno_profile_postib_processing(adreno_dev, &flags, &ringcmds);

	*ringcmds++ = cp_type3_packet(CP_EVENT_WRITE, 3);
	*ringcmds++ = CACHE_FLUSH_TS;

	if (drawctxt && !(flags & KGSL_CMD_FLAGS_INTERNAL_ISSUE)) {
		*ringcmds++ = gpuaddr + KGSL_MEMSTORE_OFFSET(context_id,
							eoptimestamp);
		*ringcmds++ = timestamp;
		*ringcmds++ = cp_type3_packet(CP_MEM_WRITE, 2);
		*ringcmds++ = gpuaddr +
			KGSL_MEMSTORE_RB_OFFSET(rb, eoptimestamp);
		*ringcmds++ = rb->timestamp;
	} else {
		*ringcmds++ = gpuaddr + KGSL_MEMSTORE_RB_OFFSET(rb,
							eoptimestamp);
		*ringcmds++ = timestamp;
	}

	if (drawctxt || (flags & KGSL_CMD_FLAGS_INTERNAL_ISSUE)) {
		*ringcmds++ = cp_type3_packet(CP_INTERRUPT, 1);
		*ringcmds++ = CP_INTERRUPT_RB;
	}

	if (adreno_is_a3xx(adreno_dev)) {
		
		*ringcmds++ = cp_type3_packet(CP_SET_CONSTANT, 2);
		*ringcmds++ =
			(0x4<<16) | (A3XX_HLSQ_CL_KERNEL_GROUP_X_REG - 0x2000);
		*ringcmds++ = 0;
	}

	if (flags & KGSL_CMD_FLAGS_WFI) {
		*ringcmds++ = cp_type3_packet(CP_WAIT_FOR_IDLE, 1);
		*ringcmds++ = 0x00000000;
	}

	if (secured_ctxt) {
		*ringcmds++ = cp_type3_packet(CP_WAIT_FOR_IDLE, 1);
		*ringcmds++ = 0x00000000;
		*ringcmds++ = cp_type3_packet(CP_WAIT_FOR_ME, 1);
		*ringcmds++ = 0x00000000;
		*ringcmds++ = cp_type3_packet(CP_SET_PROTECTED_MODE, 1);
		*ringcmds++ = 0;
		*ringcmds++ = cp_type3_packet(CP_WIDE_REG_WRITE, 2);
		*ringcmds++ = A4XX_RBBM_SECVID_TRUST_CONTROL;
		*ringcmds++ = 0;
		*ringcmds++ = cp_type3_packet(CP_SET_PROTECTED_MODE, 1);
		*ringcmds++ = 1;
		*ringcmds++ = cp_type3_packet(CP_WAIT_FOR_ME, 1);
		*ringcmds++ = 0x00000000;
	}

	adreno_ringbuffer_submit(rb, time);

	return 0;
}

int
adreno_ringbuffer_issuecmds(struct adreno_ringbuffer *rb,
				unsigned int flags,
				unsigned int *cmds,
				int sizedwords)
{
	flags |= KGSL_CMD_FLAGS_INTERNAL_ISSUE;

	return adreno_ringbuffer_addcmds(rb, flags, cmds,
		sizedwords, 0, NULL);
}

static bool _parse_ibs(struct kgsl_device_private *dev_priv, uint gpuaddr,
			   int sizedwords);

static bool
_handle_type3(struct kgsl_device_private *dev_priv, uint *hostaddr)
{
	unsigned int opcode = cp_type3_opcode(*hostaddr);
	switch (opcode) {
	case CP_INDIRECT_BUFFER_PFD:
	case CP_INDIRECT_BUFFER_PFE:
	case CP_COND_INDIRECT_BUFFER_PFE:
	case CP_COND_INDIRECT_BUFFER_PFD:
		return _parse_ibs(dev_priv, hostaddr[1], hostaddr[2]);
	case CP_NOP:
	case CP_WAIT_FOR_IDLE:
	case CP_WAIT_REG_MEM:
	case CP_WAIT_REG_EQ:
	case CP_WAT_REG_GTE:
	case CP_WAIT_UNTIL_READ:
	case CP_WAIT_IB_PFD_COMPLETE:
	case CP_REG_RMW:
	case CP_REG_TO_MEM:
	case CP_MEM_WRITE:
	case CP_MEM_WRITE_CNTR:
	case CP_COND_EXEC:
	case CP_COND_WRITE:
	case CP_EVENT_WRITE:
	case CP_EVENT_WRITE_SHD:
	case CP_EVENT_WRITE_CFL:
	case CP_EVENT_WRITE_ZPD:
	case CP_DRAW_INDX:
	case CP_DRAW_INDX_2:
	case CP_DRAW_INDX_BIN:
	case CP_DRAW_INDX_2_BIN:
	case CP_VIZ_QUERY:
	case CP_SET_STATE:
	case CP_SET_CONSTANT:
	case CP_IM_LOAD:
	case CP_IM_LOAD_IMMEDIATE:
	case CP_LOAD_CONSTANT_CONTEXT:
	case CP_INVALIDATE_STATE:
	case CP_SET_SHADER_BASES:
	case CP_SET_BIN_MASK:
	case CP_SET_BIN_SELECT:
	case CP_SET_BIN_BASE_OFFSET:
	case CP_SET_BIN_DATA:
	case CP_CONTEXT_UPDATE:
	case CP_INTERRUPT:
	case CP_IM_STORE:
	case CP_LOAD_STATE:
		break;
	
	case CP_ME_INIT:
	case CP_SET_PROTECTED_MODE:
	default:
		KGSL_CMD_ERR(dev_priv->device, "bad CP opcode %0x\n", opcode);
		return false;
		break;
	}

	return true;
}

static bool
_handle_type0(struct kgsl_device_private *dev_priv, uint *hostaddr)
{
	unsigned int reg = type0_pkt_offset(*hostaddr);
	unsigned int cnt = type0_pkt_size(*hostaddr);
	if (reg < 0x0192 || (reg + cnt) >= 0x8000) {
		KGSL_CMD_ERR(dev_priv->device, "bad type0 reg: 0x%0x cnt: %d\n",
			     reg, cnt);
		return false;
	}
	return true;
}

static bool _parse_ibs(struct kgsl_device_private *dev_priv,
			   uint gpuaddr, int sizedwords)
{
	static uint level; 
	bool ret = false;
	uint *hostaddr, *hoststart;
	int dwords_left = sizedwords; 
	struct kgsl_mem_entry *entry;

	entry = kgsl_sharedmem_find_region(dev_priv->process_priv,
					   gpuaddr, sizedwords * sizeof(uint));
	if (entry == NULL) {
		KGSL_CMD_ERR(dev_priv->device,
			     "no mapping for gpuaddr: 0x%08x\n", gpuaddr);
		return false;
	}

	hostaddr = kgsl_gpuaddr_to_vaddr(&entry->memdesc, gpuaddr);
	if (hostaddr == NULL) {
		KGSL_CMD_ERR(dev_priv->device,
			     "no mapping for gpuaddr: 0x%08x\n", gpuaddr);
		return false;
	}

	hoststart = hostaddr;

	level++;

	KGSL_CMD_INFO(dev_priv->device, "ib: gpuaddr:0x%08x, wc:%d, hptr:%p\n",
		gpuaddr, sizedwords, hostaddr);

	mb();
	while (dwords_left > 0) {
		bool cur_ret = true;
		int count = 0; 

		switch (*hostaddr >> 30) {
		case 0x0: 
			count = (*hostaddr >> 16)+2;
			cur_ret = _handle_type0(dev_priv, hostaddr);
			break;
		case 0x1: 
			count = 2;
			break;
		case 0x3: 
			count = ((*hostaddr >> 16) & 0x3fff) + 2;
			cur_ret = _handle_type3(dev_priv, hostaddr);
			break;
		default:
			KGSL_CMD_ERR(dev_priv->device, "unexpected type: "
				"type:%d, word:0x%08x @ 0x%p, gpu:0x%08x\n",
				*hostaddr >> 30, *hostaddr, hostaddr,
				gpuaddr+4*(sizedwords-dwords_left));
			cur_ret = false;
			count = dwords_left;
			break;
		}

		if (!cur_ret) {
			KGSL_CMD_ERR(dev_priv->device,
				"bad sub-type: #:%d/%d, v:0x%08x"
				" @ 0x%p[gb:0x%08x], level:%d\n",
				sizedwords-dwords_left, sizedwords, *hostaddr,
				hostaddr, gpuaddr+4*(sizedwords-dwords_left),
				level);

			if (ADRENO_DEVICE(dev_priv->device)->ib_check_level
				>= 2)
				print_hex_dump(KERN_ERR,
					level == 1 ? "IB1:" : "IB2:",
					DUMP_PREFIX_OFFSET, 32, 4, hoststart,
					sizedwords*4, 0);
			goto done;
		}

		
		dwords_left -= count;
		hostaddr += count;
		if (dwords_left < 0) {
			KGSL_CMD_ERR(dev_priv->device,
				"bad count: c:%d, #:%d/%d, "
				"v:0x%08x @ 0x%p[gb:0x%08x], level:%d\n",
				count, sizedwords-(dwords_left+count),
				sizedwords, *(hostaddr-count), hostaddr-count,
				gpuaddr+4*(sizedwords-(dwords_left+count)),
				level);
			if (ADRENO_DEVICE(dev_priv->device)->ib_check_level
				>= 2)
				print_hex_dump(KERN_ERR,
					level == 1 ? "IB1:" : "IB2:",
					DUMP_PREFIX_OFFSET, 32, 4, hoststart,
					sizedwords*4, 0);
			goto done;
		}
	}

	ret = true;
done:
	if (!ret)
		KGSL_DRV_ERR(dev_priv->device,
			"parsing failed: gpuaddr:0x%08x, "
			"host:0x%p, wc:%d\n", gpuaddr, hoststart, sizedwords);

	level--;

	return ret;
}

static inline bool _ringbuffer_verify_ib(struct kgsl_device_private *dev_priv,
		struct kgsl_memobj_node *ib)
{
	struct kgsl_device *device = dev_priv->device;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);

	
	if (ib->sizedwords == 0 || ib->sizedwords > 0xFFFFF) {
		KGSL_DRV_ERR(device, "Invalid IB size 0x%zX\n",
				ib->sizedwords);
		return false;
	}

	if (unlikely(adreno_dev->ib_check_level >= 1) &&
		!_parse_ibs(dev_priv, ib->gpuaddr, ib->sizedwords)) {
		KGSL_DRV_ERR(device, "Could not verify the IBs\n");
		return false;
	}

	return true;
}

int
adreno_ringbuffer_issueibcmds(struct kgsl_device_private *dev_priv,
				struct kgsl_context *context,
				struct kgsl_cmdbatch *cmdbatch,
				uint32_t *timestamp)
{
	struct kgsl_device *device = dev_priv->device;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct adreno_context *drawctxt = ADRENO_CONTEXT(context);
	struct kgsl_memobj_node *ib;
	int ret;

	if (kgsl_context_invalid(context))
		return -EDEADLK;

	
	list_for_each_entry(ib, &cmdbatch->cmdlist, node)
		if (!_ringbuffer_verify_ib(dev_priv, ib))
			return -EINVAL;

	
	wait_for_completion(&device->cmdbatch_gate);

	if (!(cmdbatch->flags & KGSL_CMDBATCH_MARKER)
		&& !(cmdbatch->flags & KGSL_CMDBATCH_SYNC))
		device->flags &= ~KGSL_FLAG_WAKE_ON_TOUCH;

	
	ret = adreno_dispatcher_queue_cmd(adreno_dev, drawctxt, cmdbatch,
		timestamp);

	if (!ret && test_and_clear_bit(ADRENO_CONTEXT_FAULT, &context->priv))
		ret = -EPROTO;

	return ret;
}

void adreno_ringbuffer_set_constraint(struct kgsl_device *device,
			struct kgsl_cmdbatch *cmdbatch)
{
	struct kgsl_context *context = cmdbatch->context;
	if (context->pwr_constraint.type &&
		((context->flags & KGSL_CONTEXT_PWR_CONSTRAINT) ||
			(cmdbatch->flags & KGSL_CONTEXT_PWR_CONSTRAINT)))
		kgsl_pwrctrl_set_constraint(device, &context->pwr_constraint,
						context->id);
}

static inline int _get_alwayson_counter(struct adreno_device *adreno_dev,
		unsigned int *cmds, unsigned int gpuaddr)
{
	unsigned int *p = cmds;

	*p++ = cp_type3_packet(CP_REG_TO_MEM, 2);
	*p++ = adreno_getreg(adreno_dev, ADRENO_REG_RBBM_ALWAYSON_COUNTER_LO) |
		(1 << 30) | (2 << 18);
	*p++ = gpuaddr;

	return (unsigned int)(p - cmds);
}

int adreno_ringbuffer_submitcmd(struct adreno_device *adreno_dev,
		struct kgsl_cmdbatch *cmdbatch, struct adreno_submit_time *time)
{
	struct kgsl_device *device = &adreno_dev->dev;
	struct kgsl_memobj_node *ib;
	unsigned int numibs = 0;
	unsigned int *link;
	unsigned int *cmds;
	struct kgsl_context *context;
	struct adreno_context *drawctxt;
	bool use_preamble = true;
	bool cmdbatch_user_profiling = false;
	bool cmdbatch_kernel_profiling = false;
	int flags = KGSL_CMD_FLAGS_NONE;
	int ret;
	struct adreno_ringbuffer *rb;
	struct kgsl_cmdbatch_profiling_buffer *profile_buffer = NULL;
	unsigned int dwords = 0;
	struct adreno_submit_time local;

	struct kgsl_mem_entry *entry = cmdbatch->profiling_buf_entry;
	if (entry)
		profile_buffer = kgsl_gpuaddr_to_vaddr(&entry->memdesc,
					cmdbatch->profiling_buffer_gpuaddr);

	context = cmdbatch->context;
	drawctxt = ADRENO_CONTEXT(context);

	
	list_for_each_entry(ib, &cmdbatch->cmdlist, node)
		numibs++;

	rb = drawctxt->rb;

	
	adreno_profile_process_results(adreno_dev);

	if (test_bit(ADRENO_CONTEXT_SKIP_CMD, &drawctxt->base.priv) &&
		(!test_bit(CMDBATCH_FLAG_SKIP, &cmdbatch->priv))) {

		set_bit(KGSL_FT_SKIPCMD, &cmdbatch->fault_recovery);
		cmdbatch->fault_policy = drawctxt->fault_policy;
		set_bit(CMDBATCH_FLAG_FORCE_PREAMBLE, &cmdbatch->priv);

		
		adreno_fault_skipcmd_detached(device, drawctxt, cmdbatch);

		
		clear_bit(ADRENO_CONTEXT_SKIP_CMD, &drawctxt->base.priv);
		drawctxt->fault_policy = 0;
	}


	if ((drawctxt->base.flags & KGSL_CONTEXT_PREAMBLE) &&
		!test_bit(CMDBATCH_FLAG_FORCE_PREAMBLE, &cmdbatch->priv) &&
		(rb->drawctxt_active == drawctxt))
		use_preamble = false;

	if (test_bit(CMDBATCH_FLAG_SKIP, &cmdbatch->priv)) {
		use_preamble = false;
		numibs = 0;
	}

	
	dwords = 5;

	
	dwords += (numibs * 3);

	if (cmdbatch->flags & KGSL_CMDBATCH_PROFILING &&
		adreno_is_a4xx(adreno_dev) && profile_buffer) {
		cmdbatch_user_profiling = true;
		dwords += 6;


		if (time == NULL)
			time = &local;
	}

	if (test_bit(CMDBATCH_FLAG_PROFILE, &cmdbatch->priv)) {
		cmdbatch_kernel_profiling = true;
		dwords += 6;
	}

	link = kzalloc(sizeof(unsigned int) *  dwords, GFP_KERNEL);
	if (!link) {
		ret = -ENOMEM;
		goto done;
	}

	cmds = link;

	*cmds++ = cp_nop_packet(1);
	*cmds++ = KGSL_START_OF_IB_IDENTIFIER;

	if (cmdbatch_kernel_profiling) {
		cmds += _get_alwayson_counter(adreno_dev, cmds,
			adreno_dev->cmdbatch_profile_buffer.gpuaddr +
			ADRENO_CMDBATCH_PROFILE_OFFSET(cmdbatch->profile_index,
				started));
	}

	if (cmdbatch_user_profiling) {
		cmds += _get_alwayson_counter(adreno_dev, cmds,
			cmdbatch->profiling_buffer_gpuaddr +
			offsetof(struct kgsl_cmdbatch_profiling_buffer,
			gpu_ticks_submitted));
	}

	if (numibs) {
		list_for_each_entry(ib, &cmdbatch->cmdlist, node) {
			
			if ((ib->priv & MEMOBJ_PREAMBLE) &&
					(use_preamble == false))
				*cmds++ = cp_nop_packet(3);

			if (ib->priv & MEMOBJ_SKIP)
				*cmds++ = cp_nop_packet(2);
			else
				*cmds++ = CP_HDR_INDIRECT_BUFFER_PFE;

			*cmds++ = ib->gpuaddr;
			*cmds++ = ib->sizedwords;
		}
	}

	if (cmdbatch_kernel_profiling) {
		cmds += _get_alwayson_counter(adreno_dev, cmds,
			adreno_dev->cmdbatch_profile_buffer.gpuaddr +
			ADRENO_CMDBATCH_PROFILE_OFFSET(cmdbatch->profile_index,
				retired));
	}

	if (cmdbatch_user_profiling) {
		cmds += _get_alwayson_counter(adreno_dev, cmds,
			cmdbatch->profiling_buffer_gpuaddr +
			offsetof(struct kgsl_cmdbatch_profiling_buffer,
			gpu_ticks_retired));
	}

	*cmds++ = cp_nop_packet(1);
	*cmds++ = KGSL_END_OF_IB_IDENTIFIER;

	ret = adreno_drawctxt_switch(adreno_dev, rb, drawctxt, cmdbatch->flags);

	if (ret)
		goto done;

	if (test_bit(CMDBATCH_FLAG_WFI, &cmdbatch->priv))
		flags = KGSL_CMD_FLAGS_WFI;


	if (test_and_clear_bit(ADRENO_DEVICE_PWRON, &adreno_dev->priv) &&
		test_bit(ADRENO_DEVICE_PWRON_FIXUP, &adreno_dev->priv))
		flags |= KGSL_CMD_FLAGS_PWRON_FIXUP;

	
	adreno_ringbuffer_set_constraint(device, cmdbatch);

	
	kgsl_cffdump_capture_ib_desc(device, context, cmdbatch);


	ret = adreno_ringbuffer_addcmds(rb, flags,
					&link[0], (cmds - link),
					cmdbatch->timestamp, time);

	
	if (cmdbatch_user_profiling) {
		profile_buffer->wall_clock_s = time->utime.tv_sec;
		profile_buffer->wall_clock_ns = time->utime.tv_nsec;
		profile_buffer->gpu_ticks_queued = time->ticks;
	}

	
	if (entry)
		kgsl_memdesc_unmap(&entry->memdesc);

	kgsl_cffdump_regpoll(device,
		adreno_getreg(adreno_dev, ADRENO_REG_RBBM_STATUS) << 2,
		0x00000000, 0x80000000);
done:
	trace_kgsl_issueibcmds(device, context->id, cmdbatch,
			numibs, cmdbatch->timestamp,
			cmdbatch->flags, ret, drawctxt->type);

	kfree(link);
	return ret;
}

static void adreno_ringbuffer_mmu_clk_disable_event(struct kgsl_device *device,
			struct kgsl_event_group *group, void *data, int type)
{
	struct adreno_ringbuffer_mmu_disable_clk_param *param = data;
	kgsl_mmu_disable_clk(&device->mmu, param->unit);
	
	kfree(param);
}

void
adreno_ringbuffer_mmu_disable_clk_on_ts(struct kgsl_device *device,
			struct adreno_ringbuffer *rb, unsigned int timestamp,
			int unit)
{
	struct adreno_ringbuffer_mmu_disable_clk_param *param;

	param = kmalloc(sizeof(*param), GFP_KERNEL);
	if (!param)
		return;

	param->rb = rb;
	param->unit = unit;
	param->ts = timestamp;

	if (kgsl_add_event(device, &(rb->events),
		param->ts, adreno_ringbuffer_mmu_clk_disable_event, param)) {
		KGSL_DRV_ERR(device,
			"Failed to add IOMMU disable clk event\n");
		kfree(param);
	}
}

static void adreno_ringbuffer_wait_callback(struct kgsl_device *device,
		struct kgsl_event_group *group,
		void *priv, int result)
{
	struct adreno_ringbuffer *rb = group->priv;
	wake_up_all(&rb->ts_expire_waitq);
}

int adreno_ringbuffer_waittimestamp(struct adreno_ringbuffer *rb,
					unsigned int timestamp,
					unsigned int msecs)
{
	struct kgsl_device *device = rb->device;
	int ret;
	unsigned long wait_time;

	
	BUG_ON(0 == msecs);

	ret = kgsl_add_event(device, &rb->events, timestamp,
		adreno_ringbuffer_wait_callback, NULL);
	if (ret)
		return ret;

	mutex_unlock(&device->mutex);

	wait_time = msecs_to_jiffies(msecs);
	if (0 == wait_event_timeout(rb->ts_expire_waitq,
		!kgsl_event_pending(device, &rb->events, timestamp,
				adreno_ringbuffer_wait_callback, NULL),
		wait_time))
		ret  = -ETIMEDOUT;

	mutex_lock(&device->mutex);
	if (!ret && !adreno_ringbuffer_check_timestamp(rb,
		timestamp, KGSL_TIMESTAMP_RETIRED)) {
		ret = -EAGAIN;
	}

	return ret;
}

