/*
 * Based on arch/arm/include/asm/mmu_context.h
 *
 * Copyright (C) 1996 Russell King.
 * Copyright (C) 2012 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef __ASM_MMU_CONTEXT_H
#define __ASM_MMU_CONTEXT_H

#include <linux/compiler.h>
#include <linux/sched.h>

#include <asm/cacheflush.h>
#include <asm/proc-fns.h>
#include <asm-generic/mm_hooks.h>
#include <asm/cputype.h>
#include <asm/pgtable.h>

#define MAX_ASID_BITS	16

#if defined(CONFIG_HTC_DEBUG_RTB)
#include <linux/htc_debug_tools.h>
#include <linux/msm_rtb.h>
#endif

extern unsigned int cpu_last_asid;

void __init_new_context(struct task_struct *tsk, struct mm_struct *mm);
void __new_context(struct mm_struct *mm);

#ifdef CONFIG_PID_IN_CONTEXTIDR
static inline void contextidr_thread_switch(struct task_struct *next)
{
	asm(
	"	msr	contextidr_el1, %0\n"
	"	isb"
	:
	: "r" (task_pid_nr(next)));
#if defined(CONFIG_HTC_DEBUG_RTB)
	uncached_logk_pc(LOGK_CTXID, (void *)(uintptr_t)htc_debug_get_sched_clock_ms(), (void *)(uintptr_t)(task_pid_nr(next)));
#endif
}
#else
static inline void contextidr_thread_switch(struct task_struct *next)
{
}
#endif

static inline void cpu_set_reserved_ttbr0(void)
{
	unsigned long ttbr = page_to_phys(empty_zero_page);

	asm(
	"	msr	ttbr0_el1, %0			// set TTBR0\n"
	"	isb"
	:
	: "r" (ttbr));
}

static inline void switch_new_context(struct mm_struct *mm)
{
	unsigned long flags;

	__new_context(mm);

	local_irq_save(flags);
	cpu_switch_mm(mm->pgd, mm);
	local_irq_restore(flags);
}

static inline void check_and_switch_context(struct mm_struct *mm,
					    struct task_struct *tsk)
{
	cpu_set_reserved_ttbr0();

	if (!((mm->context.id ^ cpu_last_asid) >> MAX_ASID_BITS))
		cpu_switch_mm(mm->pgd, mm);
	else if (irqs_disabled())
		set_ti_thread_flag(task_thread_info(tsk), TIF_SWITCH_MM);
	else
		switch_new_context(mm);
}

#define init_new_context(tsk,mm)	(__init_new_context(tsk,mm),0)
#define destroy_context(mm)		do { } while(0)

#define finish_arch_post_lock_switch \
	finish_arch_post_lock_switch
static inline void finish_arch_post_lock_switch(void)
{
	if (test_and_clear_thread_flag(TIF_SWITCH_MM)) {
		struct mm_struct *mm = current->mm;
		unsigned long flags;

		__new_context(mm);

		local_irq_save(flags);
		cpu_switch_mm(mm->pgd, mm);
		local_irq_restore(flags);
	}
}

static inline void
enter_lazy_tlb(struct mm_struct *mm, struct task_struct *tsk)
{
}

static inline void
switch_mm(struct mm_struct *prev, struct mm_struct *next,
	  struct task_struct *tsk)
{
	unsigned int cpu = smp_processor_id();

	if (next == &init_mm) {
		cpu_set_reserved_ttbr0();
		return;
	}

	if (!cpumask_test_and_set_cpu(cpu, mm_cpumask(next)) || prev != next)
		check_and_switch_context(next, tsk);
}

#define deactivate_mm(tsk,mm)	do { } while (0)
#define activate_mm(prev,next)	switch_mm(prev, next, NULL)

#endif
