/*
 * Idle processing for ARMv8-based Qualcomm SoCs.
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2007-2009, 2011-2014 The Linux Foundation. All rights reserved.
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

#include <linux/linkage.h>
#include <linux/threads.h>
#include <asm/assembler.h>

#ifdef CONFIG_HTC_DEBUG_FOOTPRINT
#include <htc_mnemosyne/htc_footprint.h>
#endif

ENTRY(msm_pm_boot_entry)

#ifdef CONFIG_HTC_DEBUG_FOOTPRINT
	MPIDR2INDEX	x0, x4
	VIRT2PHYS	x4, mnemosyne_phys, msm_pm_boot_entry, x2
	ldr		x4, [x4]					/* Keep mnemosyne_phys in x4 */

	cmp	x4, #0
	beq	1f							/* if base is NULL, skip it. */

	ldr	x2, =mnemosyne_kernel_footprint_cpu			/* Get the offset of an element */
	add	x2, x4, x2						/* Get the address of an element */
	add	x2, x2, x0, LSL #MNEMOSYNE_ELEMENT_SIZE_BIT_SHIFT	/* calculate data offset with cpu index first. */

	mov	x3, #2							/* write footprint 2 */
	strb	w3, [x2]
	dsb	sy							/* ensure data are written. */
1:
#endif

	mrs	x0, mpidr_el1
	and     x0, x0, #0xffff              /* what CPU/cluster affinity */

	adr	x3, align
	ldr	x1, [x3]
	sub	x3, x1, x3
	ldr	x1, =msm_pc_debug_counters_phys /* phys addr for IMEM reg */
	sub	x1, x1, x3			/* translate virt to phys */
	ldr	x1,[x1]
	cmp	x1, #0
	beq	skip_pc_debug3
	and	x2, x0, #0xff00
	lsr	x2, x0, #0x8
	add	x1, x1, x2, lsl #7	/* get offset for this cluster */
	and	x2, x0, #0xff
	add	x1, x1, x2, lsl #5	/* get offset for this cpu */
	add	x1, x1, #8		/* warmboot entry counter */
	ldr	x2, [x1]
	add	x2, x2, #1
	str	x2, [x1]
skip_pc_debug3:
	ldr     x1, =msm_pm_boot_vector
	sub	x1, x1, x3		/* translate virt to phys */
	lsr	x2, x0, #0x8		/* Get cluster id and clear cpu id */
	add	x1, x1, x2, lsl #0x5	/* Get offset for this cluster */
	and	x0, x0, #0x03		/* Get cpu id and clear cluster id */
	add	x1, x1, x0, lsl #3	/* Get offset for this cpu */

#ifdef CONFIG_HTC_DEBUG_FOOTPRINT
	MPIDR2INDEX	x0, x4
	VIRT2PHYS	x4, mnemosyne_phys, msm_pm_boot_entry, x2
	ldr		x4, [x4]					/* Keep mnemosyne_phys in x4 */

	cmp	x4, #0
	beq	1f							/* if base is NULL, skip it. */

	ldr	x2, =mnemosyne_kernel_footprint_cpu			/* Get the offset of an element */
	add	x2, x4, x2						/* Get the address of an element */
	add	x2, x2, x0, LSL #MNEMOSYNE_ELEMENT_SIZE_BIT_SHIFT	/* calculate data offset with cpu index first. */

	mov	x3, #3							/* write footprint 3 */
	strb	w3, [x2]
	dsb	sy							/* ensure data are written. */

	ldr	x2, =mnemosyne_cpu_reset_vector_address			/* Get the offset of an element */
	add	x2, x4, x2						/* Get the address of an element */
	add	x2, x2, x0, LSL #MNEMOSYNE_ELEMENT_SIZE_BIT_SHIFT	/* calculate data offset with cpu index first. */

	str	x1, [x2]
	dsb	sy							/* ensure data are written. */

	ldr	x2, =mnemosyne_cpu_reset_vector_address_value		/* Get the offset of an element */
	add	x2, x4, x2						/* Get the address of an element */
	add	x2, x2, x0, LSL #MNEMOSYNE_ELEMENT_SIZE_BIT_SHIFT	/* calculate data offset with cpu index first. */

	ldr	x3, [x1]
	str	x3, [x2]
	dsb	sy							/* ensure data are written. */
1:
#endif

	ldr 	x1, [x1]
	ret	x1                 /* jump                           */
ENDPROC(msm_pm_boot_entry)

	__ALIGN
align:  .quad .
	.data
	.globl msm_pm_boot_vector
msm_pm_boot_vector:
	.space  8 * 4 * 4

	.globl msm_pc_debug_counters_phys
msm_pc_debug_counters_phys:
	.long 0x0
