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
#include "adreno.h"
#include "kgsl_sharedmem.h"

static unsigned int _adreno_mmu_set_pt_update_condition(
			struct adreno_ringbuffer *rb,
			unsigned int *cmds, unsigned int ptname)
{
	struct kgsl_device *device = rb->device;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	unsigned int *cmds_orig = cmds;
	*cmds++ = cp_type3_packet(CP_MEM_WRITE, 2);
	*cmds++ = rb->pagetable_desc.gpuaddr +
		offsetof(struct adreno_ringbuffer_pagetable_info,
		switch_pt_enable);
	*cmds++ = 1;
	*cmds++ = cp_type3_packet(CP_WAIT_MEM_WRITES, 1);
	*cmds++ = 0;
	*cmds++ = cp_type3_packet(CP_WAIT_FOR_ME, 1);
	*cmds++ = 0;
	if (ADRENO_FEATURE(adreno_dev, ADRENO_HAS_REG_TO_REG_CMDS)) {
		
		*cmds++ = cp_type3_packet(CP_MEM_TO_REG, 2);
		*cmds++ = adreno_getreg(adreno_dev,
					ADRENO_REG_CP_SCRATCH_REG7);
		*cmds++ = adreno_dev->ringbuffers[0].pagetable_desc.gpuaddr +
			offsetof(struct adreno_ringbuffer_pagetable_info,
				current_global_ptname);
		
		*cmds++ = cp_type3_packet(CP_MEM_TO_REG, 2);
		*cmds++ = adreno_getreg(adreno_dev,
					ADRENO_REG_CP_SCRATCH_REG6);
		*cmds++ = rb->pagetable_desc.gpuaddr +
			offsetof(struct adreno_ringbuffer_pagetable_info,
				incoming_ptname);
		*cmds++ = cp_type3_packet(CP_COND_REG_EXEC, 3);
		*cmds++ = (2 << 28) | adreno_getreg(adreno_dev,
					ADRENO_REG_CP_SCRATCH_REG6);
		*cmds++ = adreno_getreg(adreno_dev,
					ADRENO_REG_CP_SCRATCH_REG7);
		*cmds++ = 7;
		*cmds++ = cp_type3_packet(CP_MEM_WRITE, 2);
		*cmds++ = rb->pagetable_desc.gpuaddr +
			offsetof(struct adreno_ringbuffer_pagetable_info,
			switch_pt_enable);
		*cmds++ = 0;
	} else {
		*cmds++ = cp_type3_packet(CP_COND_WRITE, 6);
		
		*cmds++ = (1 << 8) | (1 << 4) | 3;
		*cmds++ = adreno_dev->ringbuffers[0].pagetable_desc.gpuaddr +
			offsetof(struct adreno_ringbuffer_pagetable_info,
				current_global_ptname);
		*cmds++ = ptname;
		*cmds++ = 0xFFFFFFFF;
		*cmds++ = rb->pagetable_desc.gpuaddr +
			offsetof(struct adreno_ringbuffer_pagetable_info,
			switch_pt_enable);
		*cmds++ = 0;
	}
	*cmds++ = cp_type3_packet(CP_WAIT_MEM_WRITES, 1);
	*cmds++ = 0;
	*cmds++ = cp_type3_packet(CP_WAIT_FOR_ME, 1);
	*cmds++ = 0;

	return cmds - cmds_orig;
}

static unsigned int _adreno_iommu_pt_update_pid_to_mem(
				struct adreno_ringbuffer *rb,
				unsigned int *cmds, int ptname)
{
	struct kgsl_device *device = rb->device;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	unsigned int *cmds_orig = cmds;

	if (ADRENO_FEATURE(adreno_dev, ADRENO_HAS_REG_TO_REG_CMDS)) {
		
		*cmds++ = cp_type3_packet(CP_MEM_TO_REG, 2);
		*cmds++ = adreno_getreg(adreno_dev,
			ADRENO_REG_CP_SCRATCH_REG6);
		*cmds++ = rb->pagetable_desc.gpuaddr +
			offsetof(struct adreno_ringbuffer_pagetable_info,
				incoming_ptname);
		
		*cmds++ = cp_type3_packet(CP_REG_TO_MEM, 2);
		*cmds++ = adreno_getreg(adreno_dev,
			ADRENO_REG_CP_SCRATCH_REG6);
		*cmds++ = rb->pagetable_desc.gpuaddr +
			offsetof(struct adreno_ringbuffer_pagetable_info,
				current_rb_ptname);
		*cmds++ = cp_type3_packet(CP_REG_TO_MEM, 2);
		*cmds++ = adreno_getreg(adreno_dev,
			ADRENO_REG_CP_SCRATCH_REG6);
		*cmds++ = adreno_dev->ringbuffers[0].pagetable_desc.gpuaddr +
			offsetof(struct adreno_ringbuffer_pagetable_info,
				current_global_ptname);
	} else {
		*cmds++ = cp_type3_packet(CP_MEM_WRITE, 2);
		*cmds++ = rb->pagetable_desc.gpuaddr +
			offsetof(struct adreno_ringbuffer_pagetable_info,
				current_rb_ptname);
		*cmds++ = ptname;

		*cmds++ = cp_type3_packet(CP_MEM_WRITE, 2);
		*cmds++ = adreno_dev->ringbuffers[0].pagetable_desc.gpuaddr +
			offsetof(struct adreno_ringbuffer_pagetable_info,
				current_global_ptname);
		*cmds++ = ptname;
	}
	
	*cmds++ = cp_type3_packet(CP_MEM_WRITE, 2);
	*cmds++ = rb->pagetable_desc.gpuaddr +
		offsetof(struct adreno_ringbuffer_pagetable_info,
		switch_pt_enable);
	*cmds++ = 0;
	*cmds++ = cp_type3_packet(CP_WAIT_MEM_WRITES, 1);
	*cmds++ = 0;
	*cmds++ = cp_type3_packet(CP_WAIT_FOR_ME, 1);
	*cmds++ = 0;

	return cmds - cmds_orig;
}

static unsigned int _adreno_iommu_set_pt_v0(struct kgsl_device *device,
					unsigned int *cmds_orig,
					phys_addr_t pt_val,
					int num_iommu_units)
{
	phys_addr_t reg_pt_val;
	unsigned int *cmds = cmds_orig;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	int i;

	cmds += adreno_add_bank_change_cmds(cmds,
				KGSL_IOMMU_CONTEXT_USER,
				device->mmu.setstate_memory.gpuaddr +
				KGSL_IOMMU_SETSTATE_NOP_OFFSET);

	cmds += adreno_add_idle_cmds(adreno_dev, cmds);

	
	cmds += kgsl_mmu_sync_lock(&device->mmu, cmds);

	for (i = 0; i < num_iommu_units; i++) {
		reg_pt_val = kgsl_mmu_get_default_ttbr0(&device->mmu,
					i, KGSL_IOMMU_CONTEXT_USER);
		reg_pt_val &= ~KGSL_IOMMU_CTX_TTBR0_ADDR_MASK;
		reg_pt_val |= (pt_val & KGSL_IOMMU_CTX_TTBR0_ADDR_MASK);
		*cmds++ = cp_type3_packet(CP_MEM_WRITE, 2);
		*cmds++ = kgsl_mmu_get_reg_gpuaddr(&device->mmu, i,
			KGSL_IOMMU_CONTEXT_USER, KGSL_IOMMU_CTX_TTBR0);
		*cmds++ = reg_pt_val;
		*cmds++ = cp_type3_packet(CP_WAIT_FOR_IDLE, 1);
		*cmds++ = 0x00000000;

		cmds += adreno_add_read_cmds(cmds,
			kgsl_mmu_get_reg_gpuaddr(&device->mmu, i,
				KGSL_IOMMU_CONTEXT_USER, KGSL_IOMMU_CTX_TTBR0),
				reg_pt_val,
				device->mmu.setstate_memory.gpuaddr +
				KGSL_IOMMU_SETSTATE_NOP_OFFSET);
	}
	for (i = 0; i < num_iommu_units; i++) {
		reg_pt_val = (pt_val + kgsl_mmu_get_default_ttbr0(
					&device->mmu,
					i, KGSL_IOMMU_CONTEXT_USER));
		reg_pt_val &= ~KGSL_IOMMU_CTX_TTBR0_ADDR_MASK;
		reg_pt_val |= (pt_val & KGSL_IOMMU_CTX_TTBR0_ADDR_MASK);
		*cmds++ = cp_type3_packet(CP_MEM_WRITE, 2);
		*cmds++ = kgsl_mmu_get_reg_gpuaddr(&device->mmu, i,
			KGSL_IOMMU_CONTEXT_USER,
			KGSL_IOMMU_CTX_TLBIALL);
		*cmds++ = 1;

		cmds += __adreno_add_idle_indirect_cmds(cmds,
			device->mmu.setstate_memory.gpuaddr +
			KGSL_IOMMU_SETSTATE_NOP_OFFSET);

		cmds += adreno_add_read_cmds(cmds,
			kgsl_mmu_get_reg_gpuaddr(&device->mmu, i,
				KGSL_IOMMU_CONTEXT_USER,
				KGSL_IOMMU_CTX_TTBR0),
				reg_pt_val,
				device->mmu.setstate_memory.gpuaddr +
				KGSL_IOMMU_SETSTATE_NOP_OFFSET);
	}

	
	cmds += kgsl_mmu_sync_unlock(&device->mmu, cmds);

	cmds += adreno_add_bank_change_cmds(cmds,
		KGSL_IOMMU_CONTEXT_PRIV,
		device->mmu.setstate_memory.gpuaddr +
		KGSL_IOMMU_SETSTATE_NOP_OFFSET);

	cmds += adreno_add_idle_cmds(adreno_dev, cmds);

	return cmds - cmds_orig;
}

static unsigned int _adreno_iommu_set_pt_v1(struct adreno_ringbuffer *rb,
					unsigned int *cmds_orig,
					phys_addr_t pt_val,
					unsigned int ptname,
					int num_iommu_units)
{
	struct kgsl_device *device = rb->device;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	uint64_t ttbr0_val = 0;
	unsigned int reg_pt_val;
	unsigned int *cmds = cmds_orig;
	unsigned int *cond_exec_ptr;
	int i;
	unsigned int ttbr0, tlbiall, tlbstatus, tlbsync, mmu_ctrl;

	cmds += adreno_add_idle_cmds(adreno_dev, cmds);

	
	cmds += _adreno_mmu_set_pt_update_condition(rb, cmds, ptname);
	*cmds++ = cp_type3_packet(CP_COND_EXEC, 4);
	*cmds++ = rb->pagetable_desc.gpuaddr +
		offsetof(struct adreno_ringbuffer_pagetable_info,
			switch_pt_enable);
	*cmds++ = rb->pagetable_desc.gpuaddr +
		offsetof(struct adreno_ringbuffer_pagetable_info,
			switch_pt_enable);
	*cmds++ = 1;
	
	cond_exec_ptr = cmds;
	cmds++;
	for (i = 0; i < num_iommu_units; i++) {
		if (ADRENO_FEATURE(adreno_dev, ADRENO_HAS_REG_TO_REG_CMDS)) {
			int count = 1;

			if (KGSL_IOMMU_CTX_TTBR0_ADDR_MASK &
				0xFFFFFFFF00000000ULL)
				count = 2;
			
			*cmds++ = cp_type3_packet(CP_MEM_TO_REG, 2);
			*cmds++ = count << 16 | adreno_getreg(adreno_dev,
					ADRENO_REG_CP_SCRATCH_REG6);
			*cmds++ = rb->pagetable_desc.gpuaddr +
				offsetof(
				struct adreno_ringbuffer_pagetable_info,
				ttbr0) + i * sizeof(uint64_t);
			*cmds++ = cp_type3_packet(CP_WAIT_FOR_IDLE, 1);
			*cmds++ = 0;
			*cmds++ = cp_type3_packet(CP_REG_TO_SCRATCH, 1);
			*cmds++ = (count << 24) | (6 << 16) |
				adreno_getreg(adreno_dev,
						ADRENO_REG_CP_SCRATCH_REG6);
		} else {
			ttbr0_val = kgsl_mmu_get_default_ttbr0(&device->mmu,
				i, KGSL_IOMMU_CONTEXT_USER);
			ttbr0_val &= ~KGSL_IOMMU_CTX_TTBR0_ADDR_MASK;
			ttbr0_val |= (pt_val & KGSL_IOMMU_CTX_TTBR0_ADDR_MASK);
		}
		mmu_ctrl = kgsl_mmu_get_reg_ahbaddr(
			&device->mmu, i,
			KGSL_IOMMU_CONTEXT_USER,
			KGSL_IOMMU_IMPLDEF_MICRO_MMU_CTRL) >> 2;

		ttbr0 = kgsl_mmu_get_reg_ahbaddr(&device->mmu, i,
					KGSL_IOMMU_CONTEXT_USER,
					KGSL_IOMMU_CTX_TTBR0) >> 2;

		if (kgsl_mmu_hw_halt_supported(&device->mmu, i)) {
			*cmds++ = cp_type3_packet(CP_WAIT_FOR_IDLE, 1);
			*cmds++ = 0;
			if (adreno_is_a4xx(adreno_dev))
				cmds += adreno_wait_reg_mem(cmds,
				adreno_getreg(adreno_dev,
					ADRENO_REG_CP_WFI_PEND_CTR),
					1, 0xFFFFFFFF, 0xF);
			else
				cmds += adreno_wait_reg_eq(cmds,
				adreno_getreg(adreno_dev,
					ADRENO_REG_CP_WFI_PEND_CTR),
					1, 0xFFFFFFFF, 0xF);

			
			*cmds++ = cp_type3_packet(CP_REG_RMW, 3);
			*cmds++ = mmu_ctrl;
			
			*cmds++ =
				 ~(KGSL_IOMMU_IMPLDEF_MICRO_MMU_CTRL_HALT);
				
			*cmds++ =
				   KGSL_IOMMU_IMPLDEF_MICRO_MMU_CTRL_HALT;
			
			if (adreno_is_a4xx(adreno_dev))
				cmds += adreno_wait_reg_mem(cmds,
					mmu_ctrl,
					KGSL_IOMMU_IMPLDEF_MICRO_MMU_CTRL_IDLE,
					KGSL_IOMMU_IMPLDEF_MICRO_MMU_CTRL_IDLE,
					0xF);
			else
				cmds += adreno_wait_reg_eq(cmds,
					mmu_ctrl,
					KGSL_IOMMU_IMPLDEF_MICRO_MMU_CTRL_IDLE,
					KGSL_IOMMU_IMPLDEF_MICRO_MMU_CTRL_IDLE,
					0xF);
		}
		if (ADRENO_FEATURE(adreno_dev, ADRENO_HAS_REG_TO_REG_CMDS)) {
			
			*cmds++ = cp_type3_packet(CP_SCRATCH_TO_REG, 1);
			if (KGSL_IOMMU_CTX_TTBR0_ADDR_MASK &
				0xFFFFFFFF00000000ULL)
				*cmds++ = (2 << 24) | (6 << 16) | ttbr0;
			else
				*cmds++ = (1 << 24) | (6 << 16) | ttbr0;
		} else {
			if (KGSL_IOMMU_CTX_TTBR0_ADDR_MASK &
				0xFFFFFFFF00000000ULL) {
				reg_pt_val = (unsigned int)ttbr0_val &
						0xFFFFFFFF;
				*cmds++ = cp_type0_packet(ttbr0, 1);
				*cmds++ = reg_pt_val;
				reg_pt_val = (unsigned int)
				((ttbr0_val & 0xFFFFFFFF00000000ULL) >> 32);
				*cmds++ = cp_type0_packet(ttbr0 + 1, 1);
				*cmds++ = reg_pt_val;
			} else {
				reg_pt_val = ttbr0_val;
				*cmds++ = cp_type0_packet(ttbr0, 1);
				*cmds++ = reg_pt_val;
			}
		}
		if (kgsl_mmu_hw_halt_supported(&device->mmu, i)) {
			
			*cmds++ = cp_type3_packet(CP_REG_RMW, 3);
			*cmds++ = mmu_ctrl;
			
			*cmds++ =
			   ~(KGSL_IOMMU_IMPLDEF_MICRO_MMU_CTRL_HALT);
			
			*cmds++ = 0;
			
			*cmds++ = cp_type3_packet(CP_WAIT_FOR_ME, 1);
			*cmds++ = 0;
		}
		tlbiall = kgsl_mmu_get_reg_ahbaddr(&device->mmu, i,
					KGSL_IOMMU_CONTEXT_USER,
					KGSL_IOMMU_CTX_TLBIALL) >> 2;
		*cmds++ = cp_type0_packet(tlbiall, 1);
		*cmds++ = 1;

		tlbsync = kgsl_mmu_get_reg_ahbaddr(&device->mmu, i,
					KGSL_IOMMU_CONTEXT_USER,
					KGSL_IOMMU_CTX_TLBSYNC) >> 2;
		*cmds++ = cp_type0_packet(tlbsync, 1);
		*cmds++ = 0;

		tlbstatus = kgsl_mmu_get_reg_ahbaddr(&device->mmu, i,
				KGSL_IOMMU_CONTEXT_USER,
				KGSL_IOMMU_CTX_TLBSTATUS) >> 2;
		if (adreno_is_a4xx(adreno_dev))
			cmds += adreno_wait_reg_mem(cmds, tlbstatus, 0,
				KGSL_IOMMU_CTX_TLBSTATUS_SACTIVE, 0xF);
		else
			cmds += adreno_wait_reg_eq(cmds, tlbstatus, 0,
				KGSL_IOMMU_CTX_TLBSTATUS_SACTIVE, 0xF);
		
		*cmds++ = cp_type3_packet(CP_WAIT_FOR_ME, 1);
		*cmds++ = 0;
	}
	
	*cond_exec_ptr = (cmds - cond_exec_ptr - 1);
	cmds += adreno_add_idle_cmds(adreno_dev, cmds);
	cmds += _adreno_iommu_pt_update_pid_to_mem(rb, cmds, ptname);

	return cmds - cmds_orig;
}


static unsigned int _adreno_iommu_set_pt_v2_a3xx(struct kgsl_device *device,
					unsigned int *cmds_orig,
					phys_addr_t pt_val,
					int num_iommu_units)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	uint64_t ttbr0_val;
	unsigned int reg_pt_val;
	unsigned int *cmds = cmds_orig;
	int i;
	unsigned int ttbr0, tlbiall, tlbstatus, tlbsync;

	cmds += adreno_add_idle_cmds(adreno_dev, cmds);

	for (i = 0; i < num_iommu_units; i++) {
		ttbr0_val = kgsl_mmu_get_default_ttbr0(&device->mmu,
				i, KGSL_IOMMU_CONTEXT_USER);
		ttbr0_val &= ~KGSL_IOMMU_CTX_TTBR0_ADDR_MASK;
		ttbr0_val |= (pt_val & KGSL_IOMMU_CTX_TTBR0_ADDR_MASK);
		ttbr0 = kgsl_mmu_get_reg_ahbaddr(&device->mmu, i,
					KGSL_IOMMU_CONTEXT_USER,
					KGSL_IOMMU_CTX_TTBR0) >> 2;

		cmds += adreno_wait_reg_eq(cmds,
			adreno_getreg(adreno_dev,
				ADRENO_REG_CP_WFI_PEND_CTR),
				1, 0xFFFFFFFF, 0xF);

		
		*cmds++ = cp_type3_packet(CP_REG_RMW, 3);
		*cmds++ = A3XX_VBIF_DDR_OUTPUT_RECOVERABLE_HALT_CTRL0;
		
		*cmds++ = ~(VBIF_RECOVERABLE_HALT_CTRL);
		
		*cmds++ = 0x1;

		
		cmds += adreno_wait_reg_eq(cmds,
			A3XX_VBIF_DDR_OUTPUT_RECOVERABLE_HALT_CTRL1,
				1, 0xFFFFFFFF, 0xF);

		
		if (KGSL_IOMMU_CTX_TTBR0_ADDR_MASK &
			0xFFFFFFFF00000000ULL) {
			reg_pt_val = (unsigned int)ttbr0_val &
					0xFFFFFFFF;
			*cmds++ = cp_type3_packet(CP_REG_WR_NO_CTXT, 2);
			*cmds++ = ttbr0;
			*cmds++ = reg_pt_val;
			reg_pt_val = (unsigned int)
			((ttbr0_val & 0xFFFFFFFF00000000ULL) >> 32);
			*cmds++ = cp_type3_packet(CP_REG_WR_NO_CTXT, 2);
			*cmds++ = ttbr0 + 1;
			*cmds++ = reg_pt_val;
		} else {
			reg_pt_val = ttbr0_val;
			*cmds++ = cp_type3_packet(CP_REG_WR_NO_CTXT, 2);
			*cmds++ = ttbr0;
			*cmds++ = reg_pt_val;
		}

		
		*cmds++ = cp_type3_packet(CP_REG_RMW, 3);
		*cmds++ = A3XX_VBIF_DDR_OUTPUT_RECOVERABLE_HALT_CTRL0;
		
		*cmds++ = ~(VBIF_RECOVERABLE_HALT_CTRL);
		
		*cmds++ = 0;

		
		*cmds++ = cp_type3_packet(CP_WAIT_FOR_ME, 1);
		*cmds++ = 0;

		tlbiall = kgsl_mmu_get_reg_ahbaddr(&device->mmu, i,
					KGSL_IOMMU_CONTEXT_USER,
					KGSL_IOMMU_CTX_TLBIALL) >> 2;
		*cmds++ = cp_type3_packet(CP_REG_WR_NO_CTXT, 2);
		*cmds++ = tlbiall;
		*cmds++ = 1;

		tlbsync = kgsl_mmu_get_reg_ahbaddr(&device->mmu, i,
					KGSL_IOMMU_CONTEXT_USER,
					KGSL_IOMMU_CTX_TLBSYNC) >> 2;
		*cmds++ = cp_type3_packet(CP_REG_WR_NO_CTXT, 2);
		*cmds++ = tlbsync;
		*cmds++ = 0;

		tlbstatus = kgsl_mmu_get_reg_ahbaddr(&device->mmu, i,
				KGSL_IOMMU_CONTEXT_USER,
					KGSL_IOMMU_CTX_TLBSTATUS) >> 2;
		cmds += adreno_wait_reg_eq(cmds, tlbstatus, 0,
				KGSL_IOMMU_CTX_TLBSTATUS_SACTIVE, 0xF);
			
		*cmds++ = cp_type3_packet(CP_WAIT_FOR_ME, 1);
		*cmds++ = 0;
	}

	cmds += adreno_add_idle_cmds(adreno_dev, cmds);

	return cmds - cmds_orig;
}

static unsigned int _adreno_iommu_set_pt_v2_a4xx(struct kgsl_device *device,
					unsigned int *cmds_orig,
					phys_addr_t pt_val,
					int num_iommu_units)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	uint64_t ttbr0_val;
	unsigned int reg_pt_val;
	unsigned int *cmds = cmds_orig;
	int i;
	unsigned int ttbr0, tlbiall, tlbstatus, tlbsync;

	cmds += adreno_add_idle_cmds(adreno_dev, cmds);

	for (i = 0; i < num_iommu_units; i++) {
		ttbr0_val = kgsl_mmu_get_default_ttbr0(&device->mmu,
				i, KGSL_IOMMU_CONTEXT_USER);
		ttbr0_val &= ~KGSL_IOMMU_CTX_TTBR0_ADDR_MASK;
		ttbr0_val |= (pt_val & KGSL_IOMMU_CTX_TTBR0_ADDR_MASK);
		ttbr0 = kgsl_mmu_get_reg_ahbaddr(&device->mmu, i,
					KGSL_IOMMU_CONTEXT_USER,
					KGSL_IOMMU_CTX_TTBR0) >> 2;

		cmds += adreno_wait_reg_mem(cmds,
				adreno_getreg(adreno_dev,
					ADRENO_REG_CP_WFI_PEND_CTR),
					1, 0xFFFFFFFF, 0xF);

		
		*cmds++ = cp_type3_packet(CP_REG_RMW, 3);
		*cmds++ = A3XX_VBIF_DDR_OUTPUT_RECOVERABLE_HALT_CTRL0;
		
		*cmds++ = ~(VBIF_RECOVERABLE_HALT_CTRL);
		
		*cmds++ = 0x1;

		
		cmds += adreno_wait_reg_mem(cmds,
			A3XX_VBIF_DDR_OUTPUT_RECOVERABLE_HALT_CTRL1,
				1, 0xFFFFFFFF, 0xF);

		
		if (sizeof(phys_addr_t) > sizeof(unsigned int)) {

			reg_pt_val = ttbr0_val & 0xFFFFFFFF;
			*cmds++ = cp_type3_packet(CP_WIDE_REG_WRITE, 2);
			*cmds++ = ttbr0;
			*cmds++ = reg_pt_val;

			reg_pt_val = (unsigned int)((ttbr0_val &
				0xFFFFFFFF00000000ULL) >> 32);
			*cmds++ = cp_type3_packet(CP_WIDE_REG_WRITE, 2);
			*cmds++ = ttbr0+1;
			*cmds++ = reg_pt_val;
		} else {
			reg_pt_val = ttbr0_val;
			*cmds++ = cp_type3_packet(CP_WIDE_REG_WRITE, 2);
			*cmds++ = ttbr0;
			*cmds++ = reg_pt_val;
		}

		
		*cmds++ = cp_type3_packet(CP_REG_RMW, 3);
		*cmds++ = A3XX_VBIF_DDR_OUTPUT_RECOVERABLE_HALT_CTRL0;
		
		*cmds++ = ~(VBIF_RECOVERABLE_HALT_CTRL);
		
		*cmds++ = 0;

		
		*cmds++ = cp_type3_packet(CP_WAIT_FOR_ME, 1);
		*cmds++ = 0;

		tlbiall = kgsl_mmu_get_reg_ahbaddr(&device->mmu, i,
					KGSL_IOMMU_CONTEXT_USER,
					KGSL_IOMMU_CTX_TLBIALL) >> 2;

		*cmds++ = cp_type3_packet(CP_WIDE_REG_WRITE, 2);
		*cmds++ = tlbiall;
		*cmds++ = 1;

		tlbsync = kgsl_mmu_get_reg_ahbaddr(&device->mmu, i,
					KGSL_IOMMU_CONTEXT_USER,
					KGSL_IOMMU_CTX_TLBSYNC) >> 2;

		*cmds++ = cp_type3_packet(CP_WIDE_REG_WRITE, 2);
		*cmds++ = tlbsync;
		*cmds++ = 0;

		tlbstatus = kgsl_mmu_get_reg_ahbaddr(&device->mmu, i,
				KGSL_IOMMU_CONTEXT_USER,
				KGSL_IOMMU_CTX_TLBSTATUS) >> 2;
		cmds += adreno_wait_reg_mem(cmds, tlbstatus, 0,
				KGSL_IOMMU_CTX_TLBSTATUS_SACTIVE, 0xF);
		
		*cmds++ = cp_type3_packet(CP_WAIT_FOR_ME, 1);
		*cmds++ = 0;
	}

	cmds += adreno_add_idle_cmds(adreno_dev, cmds);

	return cmds - cmds_orig;
}

static unsigned int adreno_iommu_set_pt_generate_cmds(
					struct adreno_ringbuffer *rb,
					unsigned int *cmds,
					struct kgsl_pagetable *pt)
{
	struct kgsl_device *device = rb->device;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	phys_addr_t pt_val;
	int num_iommu_units;
	unsigned int *cmds_orig = cmds;

	
	if (test_bit(ADRENO_DEVICE_FAULT, &adreno_dev->priv))
		return 0;

	num_iommu_units = kgsl_mmu_get_num_iommu_units(&device->mmu);

	pt_val = kgsl_mmu_get_pt_base_addr(&device->mmu, pt);

	cmds += __adreno_add_idle_indirect_cmds(cmds,
		device->mmu.setstate_memory.gpuaddr +
		KGSL_IOMMU_SETSTATE_NOP_OFFSET);

	if (kgsl_msm_supports_iommu_v2())
		if (adreno_is_a4xx(adreno_dev))
			cmds += _adreno_iommu_set_pt_v2_a4xx(device, cmds,
					pt_val, num_iommu_units);
		else
			cmds += _adreno_iommu_set_pt_v2_a3xx(device, cmds,
					pt_val, num_iommu_units);
	else if (msm_soc_version_supports_iommu_v0())
		cmds += _adreno_iommu_set_pt_v0(device, cmds, pt_val,
						num_iommu_units);
	else
		cmds += _adreno_iommu_set_pt_v1(rb, cmds, pt_val,
						pt->name,
						num_iommu_units);

	
	*cmds++ = cp_type3_packet(CP_INVALIDATE_STATE, 1);
	*cmds++ = 0x7fff;

	return cmds - cmds_orig;
}

static unsigned int adreno_iommu_set_pt_ib(struct adreno_ringbuffer *rb,
				unsigned int *cmds,
				struct kgsl_pagetable *pt)
{
	struct kgsl_device *device = rb->device;
	unsigned int *cmds_orig = cmds;
	phys_addr_t pt_val;
	int i;
	uint64_t ttbr0_val;
	int num_iommu_units = kgsl_mmu_get_num_iommu_units(&device->mmu);

	pt_val = kgsl_mmu_get_pt_base_addr(&(rb->device->mmu), pt);

	
	*cmds++ = cp_type3_packet(CP_MEM_WRITE, 2);
	*cmds++ = rb->pagetable_desc.gpuaddr +
		offsetof(struct adreno_ringbuffer_pagetable_info,
			incoming_ptname);
	*cmds++ = pt->name;

	
	for (i = 0; i < num_iommu_units; i++) {
		ttbr0_val = kgsl_mmu_get_default_ttbr0(&device->mmu,
				i, KGSL_IOMMU_CONTEXT_USER);
		ttbr0_val &= ~KGSL_IOMMU_CTX_TTBR0_ADDR_MASK;
		ttbr0_val |= (pt_val & KGSL_IOMMU_CTX_TTBR0_ADDR_MASK);

		if (KGSL_IOMMU_CTX_TTBR0_ADDR_MASK & 0xFFFFFFFF00000000ULL) {
			*cmds++ = cp_type3_packet(CP_MEM_WRITE, 2);
			*cmds++ = rb->pagetable_desc.gpuaddr +
				offsetof(
				struct adreno_ringbuffer_pagetable_info,
				ttbr0) + i * sizeof(uint64_t);
			*cmds++ = ttbr0_val & 0xFFFFFFFF;
			*cmds++ = cp_type3_packet(CP_MEM_WRITE, 2);
			*cmds++ = rb->pagetable_desc.gpuaddr +
				offsetof(
				struct adreno_ringbuffer_pagetable_info,
				ttbr0) + i * sizeof(uint64_t) +
				sizeof(unsigned int);
			*cmds++ = ttbr0_val >> 32;
		} else {
			*cmds++ = cp_type3_packet(CP_MEM_WRITE, 2);
			*cmds++ = rb->pagetable_desc.gpuaddr +
				offsetof(
				struct adreno_ringbuffer_pagetable_info,
				ttbr0) + i * sizeof(uint64_t);
			*cmds = ttbr0_val & 0xFFFFFFFF;
		}
	}
	*cmds++ = cp_type3_packet(CP_WAIT_MEM_WRITES, 1);
	*cmds++ = 0;
	*cmds++ = cp_type3_packet(CP_WAIT_FOR_ME, 1);
	*cmds++ = 0;
	*cmds++ = CP_HDR_INDIRECT_BUFFER_PFE;
	*cmds++ = rb->pt_update_desc.gpuaddr;
	*cmds++ = rb->pt_update_desc.size / sizeof(unsigned int);

	return cmds - cmds_orig;
}

int _set_pagetable_gpu(struct adreno_ringbuffer *rb,
			struct kgsl_pagetable *new_pt)
{
	unsigned int *link = NULL, *cmds;
	struct kgsl_device *device = rb->device;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	int result;

	link = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (link == NULL) {
		result = -ENOMEM;
		goto done;
	}

	cmds = link;

	kgsl_mmu_enable_clk(&device->mmu, KGSL_IOMMU_MAX_UNITS);

	
	if (adreno_is_a4xx(adreno_dev))
		cmds += adreno_set_apriv(adreno_dev, cmds, 1);

	if (ADRENO_FEATURE(adreno_dev, ADRENO_HAS_REG_TO_REG_CMDS))
		cmds += adreno_iommu_set_pt_ib(rb, cmds, new_pt);
	else
		cmds += adreno_iommu_set_pt_generate_cmds(rb, cmds, new_pt);

	if (adreno_is_a4xx(adreno_dev))
		cmds += adreno_set_apriv(adreno_dev, cmds, 0);

	if ((unsigned int) (cmds - link) > (PAGE_SIZE / sizeof(unsigned int))) {
		KGSL_DRV_ERR(device, "Temp command buffer overflow\n");
		BUG();
	}
	result = adreno_ringbuffer_issuecmds(rb,
			KGSL_CMD_FLAGS_PMODE, link,
			(unsigned int)(cmds - link));

	if (result)
		kgsl_mmu_disable_clk(&device->mmu, KGSL_IOMMU_MAX_UNITS);
	else
		adreno_ringbuffer_mmu_disable_clk_on_ts(device, rb,
						rb->timestamp,
						KGSL_IOMMU_MAX_UNITS);

done:
	kfree(link);
	return result;
}

int adreno_iommu_set_pt(struct adreno_ringbuffer *rb,
			struct kgsl_pagetable *new_pt)
{
	struct kgsl_device *device = rb->device;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct kgsl_pagetable *cur_pt = device->mmu.defaultpagetable;
	int result = 0;
	int cpu_path = 0;

	if (rb->drawctxt_active)
		cur_pt = rb->drawctxt_active->base.proc_priv->pagetable;

	if (new_pt == cur_pt)
		return 0;

	if (adreno_dev->cur_rb == rb) {
		if (adreno_use_cpu_path(adreno_dev))
			cpu_path = 1;
	} else if ((rb->wptr == rb->rptr &&
			new_pt == device->mmu.defaultpagetable)) {
		cpu_path = 1;
	}
	if (cpu_path) {
		if (rb == adreno_dev->cur_rb) {
			result = kgsl_mmu_set_pt(&device->mmu, new_pt);
			if (result)
				return result;
			
			kgsl_sharedmem_writel(device,
			&adreno_dev->ringbuffers[0].pagetable_desc,
				offsetof(
					struct adreno_ringbuffer_pagetable_info,
					current_global_ptname),
				new_pt->name);
		}
		kgsl_sharedmem_writel(device, &rb->pagetable_desc,
			offsetof(struct adreno_ringbuffer_pagetable_info,
				current_rb_ptname),
			new_pt->name);
	} else {
		result = _set_pagetable_gpu(rb, new_pt);
	}
	return result;
}
void adreno_iommu_set_pt_generate_rb_cmds(struct adreno_ringbuffer *rb,
						struct kgsl_pagetable *pt)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(rb->device);

	if (!ADRENO_FEATURE(adreno_dev, ADRENO_HAS_REG_TO_REG_CMDS) ||
		rb->pt_update_desc.hostptr)
		return;

	rb->pt_update_desc.hostptr = rb->pagetable_desc.hostptr +
			sizeof(struct adreno_ringbuffer_pagetable_info);
	rb->pt_update_desc.size =
		adreno_iommu_set_pt_generate_cmds(rb,
				rb->pt_update_desc.hostptr, pt) *
				sizeof(unsigned int);
	rb->pt_update_desc.gpuaddr = rb->pagetable_desc.gpuaddr +
			sizeof(struct adreno_ringbuffer_pagetable_info);
}
