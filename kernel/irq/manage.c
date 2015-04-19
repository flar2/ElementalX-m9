/*
 * linux/kernel/irq/manage.c
 *
 * Copyright (C) 1992, 1998-2006 Linus Torvalds, Ingo Molnar
 * Copyright (C) 2005-2006 Thomas Gleixner
 *
 * This file contains driver APIs to the irq subsystem.
 */

#define pr_fmt(fmt) "genirq: " fmt

#include <linux/irq.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/random.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/sched/rt.h>
#include <linux/task_work.h>

#include "internals.h"

#ifdef CONFIG_IRQ_FORCED_THREADING
__read_mostly bool force_irqthreads;

static int __init setup_forced_irqthreads(char *arg)
{
	force_irqthreads = true;
	return 0;
}
early_param("threadirqs", setup_forced_irqthreads);
#endif

void synchronize_irq(unsigned int irq)
{
	struct irq_desc *desc = irq_to_desc(irq);
	bool inprogress;

	if (!desc)
		return;

	do {
		unsigned long flags;

		while (irqd_irq_inprogress(&desc->irq_data))
			cpu_relax();

		
		raw_spin_lock_irqsave(&desc->lock, flags);
		inprogress = irqd_irq_inprogress(&desc->irq_data);
		raw_spin_unlock_irqrestore(&desc->lock, flags);

		
	} while (inprogress);

	wait_event(desc->wait_for_threads, !atomic_read(&desc->threads_active));
}
EXPORT_SYMBOL(synchronize_irq);

#ifdef CONFIG_SMP
cpumask_var_t irq_default_affinity;

int irq_can_set_affinity(unsigned int irq)
{
	struct irq_desc *desc = irq_to_desc(irq);

	if (!desc || !irqd_can_balance(&desc->irq_data) ||
	    !desc->irq_data.chip || !desc->irq_data.chip->irq_set_affinity)
		return 0;

	return 1;
}

void irq_set_thread_affinity(struct irq_desc *desc)
{
	struct irqaction *action = desc->action;

	while (action) {
		if (action->thread)
			set_bit(IRQTF_AFFINITY, &action->thread_flags);
		action = action->next;
	}
}

#ifdef CONFIG_GENERIC_PENDING_IRQ
static inline bool irq_can_move_pcntxt(struct irq_data *data)
{
	return irqd_can_move_in_process_context(data);
}
static inline bool irq_move_pending(struct irq_data *data)
{
	return irqd_is_setaffinity_pending(data);
}
static inline void
irq_copy_pending(struct irq_desc *desc, const struct cpumask *mask)
{
	cpumask_copy(desc->pending_mask, mask);
}
static inline void
irq_get_pending(struct cpumask *mask, struct irq_desc *desc)
{
	cpumask_copy(mask, desc->pending_mask);
}
#else
static inline bool irq_can_move_pcntxt(struct irq_data *data) { return true; }
static inline bool irq_move_pending(struct irq_data *data) { return false; }
static inline void
irq_copy_pending(struct irq_desc *desc, const struct cpumask *mask) { }
static inline void
irq_get_pending(struct cpumask *mask, struct irq_desc *desc) { }
#endif

int irq_do_set_affinity(struct irq_data *data, const struct cpumask *mask,
			bool force)
{
	struct irq_desc *desc = irq_data_to_desc(data);
	struct irq_chip *chip = irq_data_get_irq_chip(data);
	int ret;

	ret = chip->irq_set_affinity(data, mask, force);
	switch (ret) {
	case IRQ_SET_MASK_OK:
		cpumask_copy(data->affinity, mask);
	case IRQ_SET_MASK_OK_NOCOPY:
		irq_set_thread_affinity(desc);
		ret = 0;
	}

	return ret;
}

int irq_set_affinity_locked(struct irq_data *data, const struct cpumask *mask,
			    bool force)
{
	struct irq_chip *chip = irq_data_get_irq_chip(data);
	struct irq_desc *desc = irq_data_to_desc(data);
	int ret = 0;

	if (!chip || !chip->irq_set_affinity)
		return -EINVAL;

	if (irq_can_move_pcntxt(data)) {
		ret = irq_do_set_affinity(data, mask, force);
	} else {
		irqd_set_move_pending(data);
		irq_copy_pending(desc, mask);
	}

	if (!list_empty(&desc->affinity_notify))
		schedule_work(&desc->affinity_work);

	irqd_set(data, IRQD_AFFINITY_SET);

	return ret;
}

int __irq_set_affinity(unsigned int irq, const struct cpumask *mask, bool force)
{
	struct irq_desc *desc = irq_to_desc(irq);
	unsigned long flags;
	int ret;

	if (!desc)
		return -EINVAL;

	raw_spin_lock_irqsave(&desc->lock, flags);
	ret = irq_set_affinity_locked(irq_desc_get_irq_data(desc), mask, force);
	raw_spin_unlock_irqrestore(&desc->lock, flags);
	return ret;
}
EXPORT_SYMBOL(irq_set_affinity);

int irq_set_affinity_hint(unsigned int irq, const struct cpumask *m)
{
	unsigned long flags;
	struct irq_desc *desc = irq_get_desc_lock(irq, &flags, IRQ_GET_DESC_CHECK_GLOBAL);

	if (!desc)
		return -EINVAL;
	desc->affinity_hint = m;
	irq_put_desc_unlock(desc, flags);
	return 0;
}
EXPORT_SYMBOL_GPL(irq_set_affinity_hint);

void irq_affinity_notify(struct work_struct *work)
{
	struct irq_desc *desc =
			container_of(work, struct irq_desc, affinity_work);
	cpumask_var_t cpumask;
	unsigned long flags;
	struct irq_affinity_notify *notify;

	if (!desc || !alloc_cpumask_var(&cpumask, GFP_KERNEL))
		return;

	raw_spin_lock_irqsave(&desc->lock, flags);
	if (irq_move_pending(&desc->irq_data))
		irq_get_pending(cpumask, desc);
	else
		cpumask_copy(cpumask, desc->irq_data.affinity);
	raw_spin_unlock_irqrestore(&desc->lock, flags);

	list_for_each_entry(notify, &desc->affinity_notify, list) {
		if (!kref_get_unless_zero(&notify->kref))
			continue;
		notify->notify(notify, cpumask);
		kref_put(&notify->kref, notify->release);
	}

	free_cpumask_var(cpumask);
}

int
irq_set_affinity_notifier(unsigned int irq, struct irq_affinity_notify *notify)
{
	struct irq_desc *desc = irq_to_desc(irq);
	unsigned long flags;

	if (!desc)
		return -EINVAL;

	if (!notify) {
		WARN("%s called with NULL notifier - use irq_release_affinity_notifier function instead.\n",
				__func__);
		return -EINVAL;
	}

	notify->irq = irq;
	kref_init(&notify->kref);
	INIT_LIST_HEAD(&notify->list);
	raw_spin_lock_irqsave(&desc->lock, flags);
	list_add(&notify->list, &desc->affinity_notify);
	raw_spin_unlock_irqrestore(&desc->lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(irq_set_affinity_notifier);

int irq_release_affinity_notifier(struct irq_affinity_notify *notify)
{
	struct irq_desc *desc;
	unsigned long flags;

	if (!notify)
		return -EINVAL;

	desc = irq_to_desc(notify->irq);
	raw_spin_lock_irqsave(&desc->lock, flags);
	list_del(&notify->list);
	raw_spin_unlock_irqrestore(&desc->lock, flags);
	kref_put(&notify->kref, notify->release);

	return 0;
}
EXPORT_SYMBOL(irq_release_affinity_notifier);

#ifndef CONFIG_AUTO_IRQ_AFFINITY
static int
setup_affinity(unsigned int irq, struct irq_desc *desc, struct cpumask *mask)
{
	struct cpumask *set = irq_default_affinity;
	int node = desc->irq_data.node;

	
	if (!irq_can_set_affinity(irq))
		return 0;

	if (irqd_has_set(&desc->irq_data, IRQD_AFFINITY_SET)) {
		if (cpumask_intersects(desc->irq_data.affinity,
				       cpu_online_mask))
			set = desc->irq_data.affinity;
		else
			irqd_clear(&desc->irq_data, IRQD_AFFINITY_SET);
	}

	cpumask_and(mask, cpu_online_mask, set);
	if (node != NUMA_NO_NODE) {
		const struct cpumask *nodemask = cpumask_of_node(node);

		
		if (cpumask_intersects(mask, nodemask))
			cpumask_and(mask, mask, nodemask);
	}
	irq_do_set_affinity(&desc->irq_data, mask, false);
	return 0;
}
#else
static inline int
setup_affinity(unsigned int irq, struct irq_desc *d, struct cpumask *mask)
{
	return irq_select_affinity(irq);
}
#endif

int irq_select_affinity_usr(unsigned int irq, struct cpumask *mask)
{
	struct irq_desc *desc = irq_to_desc(irq);
	unsigned long flags;
	int ret;

	raw_spin_lock_irqsave(&desc->lock, flags);
	ret = setup_affinity(irq, desc, mask);
	raw_spin_unlock_irqrestore(&desc->lock, flags);
	return ret;
}

#else
static inline int
setup_affinity(unsigned int irq, struct irq_desc *desc, struct cpumask *mask)
{
	return 0;
}
#endif

void __disable_irq(struct irq_desc *desc, unsigned int irq, bool suspend)
{
	if (suspend) {
		if (!desc->action || (desc->action->flags & IRQF_NO_SUSPEND))
			return;
		desc->istate |= IRQS_SUSPENDED;
	}

	if (!desc->depth++)
		irq_disable(desc);
}

static int __disable_irq_nosync(unsigned int irq)
{
	unsigned long flags;
	struct irq_desc *desc = irq_get_desc_buslock(irq, &flags, IRQ_GET_DESC_CHECK_GLOBAL);

	if (!desc)
		return -EINVAL;
	__disable_irq(desc, irq, false);
	irq_put_desc_busunlock(desc, flags);
	return 0;
}

void disable_irq_nosync(unsigned int irq)
{
	__disable_irq_nosync(irq);
}
EXPORT_SYMBOL(disable_irq_nosync);

void disable_irq(unsigned int irq)
{
	if (!__disable_irq_nosync(irq))
		synchronize_irq(irq);
}
EXPORT_SYMBOL(disable_irq);

void __enable_irq(struct irq_desc *desc, unsigned int irq, bool resume)
{
	if (resume) {
		if (!(desc->istate & IRQS_SUSPENDED)) {
			if (!desc->action)
				return;
			if (!(desc->action->flags & IRQF_FORCE_RESUME))
				return;
			
			desc->depth++;
		}
		desc->istate &= ~IRQS_SUSPENDED;
	}

	switch (desc->depth) {
	case 0:
 err_out:
		WARN(1, KERN_WARNING "Unbalanced enable for IRQ %d, desc->istate=0x%x, desc->depth=%d\n", irq, desc->istate, desc->depth);
		break;
	case 1: {
		if (desc->istate & IRQS_SUSPENDED)
			goto err_out;
		
		irq_settings_set_noprobe(desc);
		irq_enable(desc);
		check_irq_resend(desc, irq);
		
	}
	default:
		desc->depth--;
	}
}

void enable_irq(unsigned int irq)
{
	unsigned long flags;
	struct irq_desc *desc = irq_get_desc_buslock(irq, &flags, IRQ_GET_DESC_CHECK_GLOBAL);

	if (!desc)
		return;
	if (WARN(!desc->irq_data.chip,
		 KERN_ERR "enable_irq before setup/request_irq: irq %u\n", irq))
		goto out;

	__enable_irq(desc, irq, false);
out:
	irq_put_desc_busunlock(desc, flags);
}
EXPORT_SYMBOL(enable_irq);

static int set_irq_wake_real(unsigned int irq, unsigned int on)
{
	struct irq_desc *desc = irq_to_desc(irq);
	int ret = -ENXIO;

	if (irq_desc_get_chip(desc)->flags &  IRQCHIP_SKIP_SET_WAKE)
		return 0;

	if (desc->irq_data.chip->irq_set_wake)
		ret = desc->irq_data.chip->irq_set_wake(&desc->irq_data, on);

	return ret;
}

int irq_set_irq_wake(unsigned int irq, unsigned int on)
{
	unsigned long flags;
	struct irq_desc *desc = irq_get_desc_buslock(irq, &flags, IRQ_GET_DESC_CHECK_GLOBAL);
	int ret = 0;

	if (!desc)
		return -EINVAL;

	if (on) {
		if (desc->wake_depth++ == 0) {
			ret = set_irq_wake_real(irq, on);
			if (ret)
				desc->wake_depth = 0;
			else
				irqd_set(&desc->irq_data, IRQD_WAKEUP_STATE);
		}
	} else {
		if (desc->wake_depth == 0) {
			WARN(1, "Unbalanced IRQ %d wake disable\n", irq);
		} else if (--desc->wake_depth == 0) {
			ret = set_irq_wake_real(irq, on);
			if (ret)
				desc->wake_depth = 1;
			else
				irqd_clear(&desc->irq_data, IRQD_WAKEUP_STATE);
		}
	}
	irq_put_desc_busunlock(desc, flags);
	return ret;
}
EXPORT_SYMBOL(irq_set_irq_wake);

int irq_read_line(unsigned int irq)
{
	struct irq_desc *desc = irq_to_desc(irq);
	int val;

	if (!desc || !desc->irq_data.chip->irq_read_line)
		return -EINVAL;

	chip_bus_lock(desc);
	raw_spin_lock(&desc->lock);
	val = desc->irq_data.chip->irq_read_line(&desc->irq_data);
	raw_spin_unlock(&desc->lock);
	chip_bus_sync_unlock(desc);
	return val;
}
EXPORT_SYMBOL_GPL(irq_read_line);

int can_request_irq(unsigned int irq, unsigned long irqflags)
{
	unsigned long flags;
	struct irq_desc *desc = irq_get_desc_lock(irq, &flags, 0);
	int canrequest = 0;

	if (!desc)
		return 0;

	if (irq_settings_can_request(desc)) {
		if (!desc->action ||
		    irqflags & desc->action->flags & IRQF_SHARED)
			canrequest = 1;
	}
	irq_put_desc_unlock(desc, flags);
	return canrequest;
}

int __irq_set_trigger(struct irq_desc *desc, unsigned int irq,
		      unsigned long flags)
{
	struct irq_chip *chip = desc->irq_data.chip;
	int ret, unmask = 0;

	if (!chip || !chip->irq_set_type) {
		pr_debug("No set_type function for IRQ %d (%s)\n", irq,
			 chip ? (chip->name ? : "unknown") : "unknown");
		return 0;
	}

	flags &= IRQ_TYPE_SENSE_MASK;

	if (chip->flags & IRQCHIP_SET_TYPE_MASKED) {
		if (!irqd_irq_masked(&desc->irq_data))
			mask_irq(desc);
		if (!irqd_irq_disabled(&desc->irq_data))
			unmask = 1;
	}

	
	ret = chip->irq_set_type(&desc->irq_data, flags);

	switch (ret) {
	case IRQ_SET_MASK_OK:
		irqd_clear(&desc->irq_data, IRQD_TRIGGER_MASK);
		irqd_set(&desc->irq_data, flags);

	case IRQ_SET_MASK_OK_NOCOPY:
		flags = irqd_get_trigger_type(&desc->irq_data);
		irq_settings_set_trigger_mask(desc, flags);
		irqd_clear(&desc->irq_data, IRQD_LEVEL);
		irq_settings_clr_level(desc);
		if (flags & IRQ_TYPE_LEVEL_MASK) {
			irq_settings_set_level(desc);
			irqd_set(&desc->irq_data, IRQD_LEVEL);
		}

		ret = 0;
		break;
	default:
		pr_err("Setting trigger mode %lu for irq %u failed (%pF)\n",
		       flags, irq, chip->irq_set_type);
	}
	if (unmask)
		unmask_irq(desc);
	return ret;
}

#ifdef CONFIG_HARDIRQS_SW_RESEND
int irq_set_parent(int irq, int parent_irq)
{
	unsigned long flags;
	struct irq_desc *desc = irq_get_desc_lock(irq, &flags, 0);

	if (!desc)
		return -EINVAL;

	desc->parent_irq = parent_irq;

	irq_put_desc_unlock(desc, flags);
	return 0;
}
#endif

static irqreturn_t irq_default_primary_handler(int irq, void *dev_id)
{
	return IRQ_WAKE_THREAD;
}

static irqreturn_t irq_nested_primary_handler(int irq, void *dev_id)
{
	WARN(1, "Primary handler called for nested irq %d\n", irq);
	return IRQ_NONE;
}

static int irq_wait_for_interrupt(struct irqaction *action)
{
	set_current_state(TASK_INTERRUPTIBLE);

	while (!kthread_should_stop()) {

		if (test_and_clear_bit(IRQTF_RUNTHREAD,
				       &action->thread_flags)) {
			__set_current_state(TASK_RUNNING);
			return 0;
		}
		schedule();
		set_current_state(TASK_INTERRUPTIBLE);
	}
	__set_current_state(TASK_RUNNING);
	return -1;
}

static void irq_finalize_oneshot(struct irq_desc *desc,
				 struct irqaction *action)
{
	if (!(desc->istate & IRQS_ONESHOT))
		return;
again:
	chip_bus_lock(desc);
	raw_spin_lock_irq(&desc->lock);

	if (unlikely(irqd_irq_inprogress(&desc->irq_data))) {
		raw_spin_unlock_irq(&desc->lock);
		chip_bus_sync_unlock(desc);
		cpu_relax();
		goto again;
	}

	if (test_bit(IRQTF_RUNTHREAD, &action->thread_flags))
		goto out_unlock;

	desc->threads_oneshot &= ~action->thread_mask;

	if (!desc->threads_oneshot && !irqd_irq_disabled(&desc->irq_data) &&
	    irqd_irq_masked(&desc->irq_data))
		unmask_irq(desc);

out_unlock:
	raw_spin_unlock_irq(&desc->lock);
	chip_bus_sync_unlock(desc);
}

#ifdef CONFIG_SMP
static void
irq_thread_check_affinity(struct irq_desc *desc, struct irqaction *action)
{
	cpumask_var_t mask;
	bool valid = true;

	if (!test_and_clear_bit(IRQTF_AFFINITY, &action->thread_flags))
		return;

	if (!alloc_cpumask_var(&mask, GFP_KERNEL)) {
		set_bit(IRQTF_AFFINITY, &action->thread_flags);
		return;
	}

	raw_spin_lock_irq(&desc->lock);
	if (desc->irq_data.affinity)
		cpumask_copy(mask, desc->irq_data.affinity);
	else
		valid = false;
	raw_spin_unlock_irq(&desc->lock);

	if (valid)
		set_cpus_allowed_ptr(current, mask);
	free_cpumask_var(mask);
}
#else
static inline void
irq_thread_check_affinity(struct irq_desc *desc, struct irqaction *action) { }
#endif

static irqreturn_t
irq_forced_thread_fn(struct irq_desc *desc, struct irqaction *action)
{
	irqreturn_t ret;

	local_bh_disable();
	ret = action->thread_fn(action->irq, action->dev_id);
	irq_finalize_oneshot(desc, action);
	local_bh_enable();
	return ret;
}

static irqreturn_t irq_thread_fn(struct irq_desc *desc,
		struct irqaction *action)
{
	irqreturn_t ret;

	ret = action->thread_fn(action->irq, action->dev_id);
	irq_finalize_oneshot(desc, action);
	return ret;
}

static void wake_threads_waitq(struct irq_desc *desc)
{
	if (atomic_dec_and_test(&desc->threads_active))
		wake_up(&desc->wait_for_threads);
}

static void irq_thread_dtor(struct callback_head *unused)
{
	struct task_struct *tsk = current;
	struct irq_desc *desc;
	struct irqaction *action;

	if (WARN_ON_ONCE(!(current->flags & PF_EXITING)))
		return;

	action = kthread_data(tsk);

	pr_err("exiting task \"%s\" (%d) is an active IRQ thread (irq %d)\n",
	       tsk->comm, tsk->pid, action->irq);


	desc = irq_to_desc(action->irq);
	if (test_and_clear_bit(IRQTF_RUNTHREAD, &action->thread_flags))
		wake_threads_waitq(desc);

	
	irq_finalize_oneshot(desc, action);
}

static int irq_thread(void *data)
{
	struct callback_head on_exit_work;
	static const struct sched_param param = {
		.sched_priority = MAX_USER_RT_PRIO/2,
	};
	struct irqaction *action = data;
	struct irq_desc *desc = irq_to_desc(action->irq);
	irqreturn_t (*handler_fn)(struct irq_desc *desc,
			struct irqaction *action);

	if (force_irqthreads && test_bit(IRQTF_FORCED_THREAD,
					&action->thread_flags))
		handler_fn = irq_forced_thread_fn;
	else
		handler_fn = irq_thread_fn;

	sched_setscheduler(current, SCHED_FIFO, &param);

	init_task_work(&on_exit_work, irq_thread_dtor);
	task_work_add(current, &on_exit_work, false);

	irq_thread_check_affinity(desc, action);

	while (!irq_wait_for_interrupt(action)) {
		irqreturn_t action_ret;

		irq_thread_check_affinity(desc, action);

		action_ret = handler_fn(desc, action);
		if (action_ret == IRQ_HANDLED)
			atomic_inc(&desc->threads_handled);

		wake_threads_waitq(desc);
	}

	task_work_cancel(current, irq_thread_dtor);
	return 0;
}

static void irq_setup_forced_threading(struct irqaction *new)
{
	if (!force_irqthreads)
		return;
	if (new->flags & (IRQF_NO_THREAD | IRQF_PERCPU | IRQF_ONESHOT))
		return;

	new->flags |= IRQF_ONESHOT;

	if (!new->thread_fn) {
		set_bit(IRQTF_FORCED_THREAD, &new->thread_flags);
		new->thread_fn = new->handler;
		new->handler = irq_default_primary_handler;
	}
}

static int
__setup_irq(unsigned int irq, struct irq_desc *desc, struct irqaction *new)
{
	struct irqaction *old, **old_ptr;
	unsigned long flags, thread_mask = 0;
	int ret, nested, shared = 0;
	cpumask_var_t mask;

	if (!desc)
		return -EINVAL;

	if (desc->irq_data.chip == &no_irq_chip)
		return -ENOSYS;
	if (!try_module_get(desc->owner))
		return -ENODEV;

	nested = irq_settings_is_nested_thread(desc);
	if (nested) {
		if (!new->thread_fn) {
			ret = -EINVAL;
			goto out_mput;
		}
		new->handler = irq_nested_primary_handler;
	} else {
		if (irq_settings_can_thread(desc))
			irq_setup_forced_threading(new);
	}

	if (new->thread_fn && !nested) {
		struct task_struct *t;

		t = kthread_create(irq_thread, new, "irq/%d-%s", irq,
				   new->name);
		if (IS_ERR(t)) {
			ret = PTR_ERR(t);
			goto out_mput;
		}
		get_task_struct(t);
		new->thread = t;
		set_bit(IRQTF_AFFINITY, &new->thread_flags);
	}

	if (!alloc_cpumask_var(&mask, GFP_KERNEL)) {
		ret = -ENOMEM;
		goto out_thread;
	}

	/*
	 * Drivers are often written to work w/o knowledge about the
	 * underlying irq chip implementation, so a request for a
	 * threaded irq without a primary hard irq context handler
	 * requires the ONESHOT flag to be set. Some irq chips like
	 * MSI based interrupts are per se one shot safe. Check the
	 * chip flags, so we can avoid the unmask dance at the end of
	 * the threaded handler for those.
	 */
	if (desc->irq_data.chip->flags & IRQCHIP_ONESHOT_SAFE)
		new->flags &= ~IRQF_ONESHOT;

	raw_spin_lock_irqsave(&desc->lock, flags);
	old_ptr = &desc->action;
	old = *old_ptr;
	if (old) {
		if (!((old->flags & new->flags) & IRQF_SHARED) ||
		    ((old->flags ^ new->flags) & IRQF_TRIGGER_MASK) ||
		    ((old->flags ^ new->flags) & IRQF_ONESHOT))
			goto mismatch;

		
		if ((old->flags & IRQF_PERCPU) !=
		    (new->flags & IRQF_PERCPU))
			goto mismatch;

		
		do {
			thread_mask |= old->thread_mask;
			old_ptr = &old->next;
			old = *old_ptr;
		} while (old);
		shared = 1;
	}

	if (new->flags & IRQF_ONESHOT) {
		if (thread_mask == ~0UL) {
			ret = -EBUSY;
			goto out_mask;
		}
		new->thread_mask = 1 << ffz(thread_mask);

	} else if (new->handler == irq_default_primary_handler &&
		   !(desc->irq_data.chip->flags & IRQCHIP_ONESHOT_SAFE)) {
		pr_err("Threaded irq requested with handler=NULL and !ONESHOT for irq %d\n",
		       irq);
		ret = -EINVAL;
		goto out_mask;
	}

	if (!shared) {
		init_waitqueue_head(&desc->wait_for_threads);

		
		if (new->flags & IRQF_TRIGGER_MASK) {
			ret = __irq_set_trigger(desc, irq,
					new->flags & IRQF_TRIGGER_MASK);

			if (ret)
				goto out_mask;
		}

		desc->istate &= ~(IRQS_AUTODETECT | IRQS_SPURIOUS_DISABLED | \
				  IRQS_ONESHOT | IRQS_WAITING);
		irqd_clear(&desc->irq_data, IRQD_IRQ_INPROGRESS);

		if (new->flags & IRQF_PERCPU) {
			irqd_set(&desc->irq_data, IRQD_PER_CPU);
			irq_settings_set_per_cpu(desc);
		}

		if (new->flags & IRQF_ONESHOT)
			desc->istate |= IRQS_ONESHOT;

		if (irq_settings_can_autoenable(desc))
			irq_startup(desc, true);
		else
			
			desc->depth = 1;

		
		if (new->flags & IRQF_NOBALANCING) {
			irq_settings_set_no_balancing(desc);
			irqd_set(&desc->irq_data, IRQD_NO_BALANCING);
		}

		
		setup_affinity(irq, desc, mask);

	} else if (new->flags & IRQF_TRIGGER_MASK) {
		unsigned int nmsk = new->flags & IRQF_TRIGGER_MASK;
		unsigned int omsk = irq_settings_get_trigger_mask(desc);

		if (nmsk != omsk)
			
			pr_warning("irq %d uses trigger mode %u; requested %u\n",
				   irq, nmsk, omsk);
	}

	new->irq = irq;
	*old_ptr = new;

	
	desc->irq_count = 0;
	desc->irqs_unhandled = 0;

	if (shared && (desc->istate & IRQS_SPURIOUS_DISABLED)) {
		desc->istate &= ~IRQS_SPURIOUS_DISABLED;
		__enable_irq(desc, irq, false);
	}

	raw_spin_unlock_irqrestore(&desc->lock, flags);

	if (new->thread)
		wake_up_process(new->thread);

	register_irq_proc(irq, desc);
	new->dir = NULL;
	register_handler_proc(irq, new);
	free_cpumask_var(mask);

	return 0;

mismatch:
	if (!(new->flags & IRQF_PROBE_SHARED)) {
		pr_err("Flags mismatch irq %d. %08x (%s) vs. %08x (%s)\n",
		       irq, new->flags, new->name, old->flags, old->name);
#ifdef CONFIG_DEBUG_SHIRQ
		dump_stack();
#endif
	}
	ret = -EBUSY;

out_mask:
	raw_spin_unlock_irqrestore(&desc->lock, flags);
	free_cpumask_var(mask);

out_thread:
	if (new->thread) {
		struct task_struct *t = new->thread;

		new->thread = NULL;
		kthread_stop(t);
		put_task_struct(t);
	}
out_mput:
	module_put(desc->owner);
	return ret;
}

int setup_irq(unsigned int irq, struct irqaction *act)
{
	int retval;
	struct irq_desc *desc = irq_to_desc(irq);

	if (WARN_ON(irq_settings_is_per_cpu_devid(desc)))
		return -EINVAL;
	chip_bus_lock(desc);
	retval = __setup_irq(irq, desc, act);
	chip_bus_sync_unlock(desc);

	return retval;
}
EXPORT_SYMBOL_GPL(setup_irq);

static struct irqaction *__free_irq(unsigned int irq, void *dev_id)
{
	struct irq_desc *desc = irq_to_desc(irq);
	struct irqaction *action, **action_ptr;
	unsigned long flags;

	WARN(in_interrupt(), "Trying to free IRQ %d from IRQ context!\n", irq);

	if (!desc)
		return NULL;

	raw_spin_lock_irqsave(&desc->lock, flags);

	action_ptr = &desc->action;
	for (;;) {
		action = *action_ptr;

		if (!action) {
			WARN(1, "Trying to free already-free IRQ %d\n", irq);
			raw_spin_unlock_irqrestore(&desc->lock, flags);

			return NULL;
		}

		if (action->dev_id == dev_id)
			break;
		action_ptr = &action->next;
	}

	
	*action_ptr = action->next;

	
	if (!desc->action) {
		irq_shutdown(desc);

		
		if (desc->irq_data.chip->irq_mask)
			desc->irq_data.chip->irq_mask(&desc->irq_data);
		else if (desc->irq_data.chip->irq_mask_ack)
			desc->irq_data.chip->irq_mask_ack(&desc->irq_data);
	}

#ifdef CONFIG_SMP
	
	if (WARN_ON_ONCE(desc->affinity_hint))
		desc->affinity_hint = NULL;
#endif

	raw_spin_unlock_irqrestore(&desc->lock, flags);

	unregister_handler_proc(irq, action);

	
	synchronize_irq(irq);

#ifdef CONFIG_DEBUG_SHIRQ
	if (action->flags & IRQF_SHARED) {
		local_irq_save(flags);
		action->handler(irq, dev_id);
		local_irq_restore(flags);
	}
#endif

	if (action->thread) {
		kthread_stop(action->thread);
		put_task_struct(action->thread);
	}

	module_put(desc->owner);
	return action;
}

void remove_irq(unsigned int irq, struct irqaction *act)
{
	struct irq_desc *desc = irq_to_desc(irq);

	if (desc && !WARN_ON(irq_settings_is_per_cpu_devid(desc)))
	    __free_irq(irq, act->dev_id);
}
EXPORT_SYMBOL_GPL(remove_irq);

void free_irq(unsigned int irq, void *dev_id)
{
	struct irq_desc *desc = irq_to_desc(irq);
#ifdef CONFIG_SMP
	struct irq_affinity_notify *notify;
#endif
	if (!desc || WARN_ON(irq_settings_is_per_cpu_devid(desc)))
		return;

#ifdef CONFIG_SMP
	WARN_ON(!list_empty(&desc->affinity_notify));

	list_for_each_entry(notify, &desc->affinity_notify, list)
		kref_put(&notify->kref, notify->release);
#endif

	chip_bus_lock(desc);
	kfree(__free_irq(irq, dev_id));
	chip_bus_sync_unlock(desc);
}
EXPORT_SYMBOL(free_irq);

int request_threaded_irq(unsigned int irq, irq_handler_t handler,
			 irq_handler_t thread_fn, unsigned long irqflags,
			 const char *devname, void *dev_id)
{
	struct irqaction *action;
	struct irq_desc *desc;
	int retval;

	if ((irqflags & IRQF_SHARED) && !dev_id)
		return -EINVAL;

	desc = irq_to_desc(irq);
	if (!desc)
		return -EINVAL;

	if (!irq_settings_can_request(desc) ||
	    WARN_ON(irq_settings_is_per_cpu_devid(desc)))
		return -EINVAL;

	if (!handler) {
		if (!thread_fn)
			return -EINVAL;
		handler = irq_default_primary_handler;
	}

	action = kzalloc(sizeof(struct irqaction), GFP_KERNEL);
	if (!action)
		return -ENOMEM;

	action->handler = handler;
	action->thread_fn = thread_fn;
	action->flags = irqflags;
	action->name = devname;
	action->dev_id = dev_id;

	chip_bus_lock(desc);
	retval = __setup_irq(irq, desc, action);
	chip_bus_sync_unlock(desc);

	if (retval)
		kfree(action);

#ifdef CONFIG_DEBUG_SHIRQ_FIXME
	if (!retval && (irqflags & IRQF_SHARED)) {
		unsigned long flags;

		disable_irq(irq);
		local_irq_save(flags);

		handler(irq, dev_id);

		local_irq_restore(flags);
		enable_irq(irq);
	}
#endif
	return retval;
}
EXPORT_SYMBOL(request_threaded_irq);

int request_any_context_irq(unsigned int irq, irq_handler_t handler,
			    unsigned long flags, const char *name, void *dev_id)
{
	struct irq_desc *desc = irq_to_desc(irq);
	int ret;

	if (!desc)
		return -EINVAL;

	if (irq_settings_is_nested_thread(desc)) {
		ret = request_threaded_irq(irq, NULL, handler,
					   flags, name, dev_id);
		return !ret ? IRQC_IS_NESTED : ret;
	}

	ret = request_irq(irq, handler, flags, name, dev_id);
	return !ret ? IRQC_IS_HARDIRQ : ret;
}
EXPORT_SYMBOL_GPL(request_any_context_irq);

void irq_set_pending(unsigned int irq)
{
	struct irq_desc *desc = irq_to_desc(irq);
	unsigned long flags;

	if (desc) {
		raw_spin_lock_irqsave(&desc->lock, flags);
		desc->istate |= IRQS_PENDING;
		raw_spin_unlock_irqrestore(&desc->lock, flags);
	}
}
EXPORT_SYMBOL_GPL(irq_set_pending);

void enable_percpu_irq(unsigned int irq, unsigned int type)
{
	unsigned int cpu = smp_processor_id();
	unsigned long flags;
	struct irq_desc *desc = irq_get_desc_lock(irq, &flags, IRQ_GET_DESC_CHECK_PERCPU);

	if (!desc)
		return;

	type &= IRQ_TYPE_SENSE_MASK;
	if (type != IRQ_TYPE_NONE) {
		int ret;

		ret = __irq_set_trigger(desc, irq, type);

		if (ret) {
			WARN(1, "failed to set type for IRQ%d\n", irq);
			goto out;
		}
	}

	irq_percpu_enable(desc, cpu);
out:
	irq_put_desc_unlock(desc, flags);
}
EXPORT_SYMBOL_GPL(enable_percpu_irq);

void disable_percpu_irq(unsigned int irq)
{
	unsigned int cpu = smp_processor_id();
	unsigned long flags;
	struct irq_desc *desc = irq_get_desc_lock(irq, &flags, IRQ_GET_DESC_CHECK_PERCPU);

	if (!desc)
		return;

	irq_percpu_disable(desc, cpu);
	irq_put_desc_unlock(desc, flags);
}
EXPORT_SYMBOL_GPL(disable_percpu_irq);

static struct irqaction *__free_percpu_irq(unsigned int irq, void __percpu *dev_id)
{
	struct irq_desc *desc = irq_to_desc(irq);
	struct irqaction *action;
	unsigned long flags;

	WARN(in_interrupt(), "Trying to free IRQ %d from IRQ context!\n", irq);

	if (!desc)
		return NULL;

	raw_spin_lock_irqsave(&desc->lock, flags);

	action = desc->action;
	if (!action || action->percpu_dev_id != dev_id) {
		WARN(1, "Trying to free already-free IRQ %d\n", irq);
		goto bad;
	}

	if (!cpumask_empty(desc->percpu_enabled)) {
		WARN(1, "percpu IRQ %d still enabled on CPU%d!\n",
		     irq, cpumask_first(desc->percpu_enabled));
		goto bad;
	}

	
	desc->action = NULL;

	raw_spin_unlock_irqrestore(&desc->lock, flags);

	unregister_handler_proc(irq, action);

	module_put(desc->owner);
	return action;

bad:
	raw_spin_unlock_irqrestore(&desc->lock, flags);
	return NULL;
}

void remove_percpu_irq(unsigned int irq, struct irqaction *act)
{
	struct irq_desc *desc = irq_to_desc(irq);

	if (desc && irq_settings_is_per_cpu_devid(desc))
	    __free_percpu_irq(irq, act->percpu_dev_id);
}

void free_percpu_irq(unsigned int irq, void __percpu *dev_id)
{
	struct irq_desc *desc = irq_to_desc(irq);

	if (!desc || !irq_settings_is_per_cpu_devid(desc))
		return;

	chip_bus_lock(desc);
	kfree(__free_percpu_irq(irq, dev_id));
	chip_bus_sync_unlock(desc);
}

int setup_percpu_irq(unsigned int irq, struct irqaction *act)
{
	struct irq_desc *desc = irq_to_desc(irq);
	int retval;

	if (!desc || !irq_settings_is_per_cpu_devid(desc))
		return -EINVAL;
	chip_bus_lock(desc);
	retval = __setup_irq(irq, desc, act);
	chip_bus_sync_unlock(desc);

	return retval;
}

int request_percpu_irq(unsigned int irq, irq_handler_t handler,
		       const char *devname, void __percpu *dev_id)
{
	struct irqaction *action;
	struct irq_desc *desc;
	int retval;

	if (!dev_id)
		return -EINVAL;

	desc = irq_to_desc(irq);
	if (!desc || !irq_settings_can_request(desc) ||
	    !irq_settings_is_per_cpu_devid(desc))
		return -EINVAL;

	action = kzalloc(sizeof(struct irqaction), GFP_KERNEL);
	if (!action)
		return -ENOMEM;

	action->handler = handler;
	action->flags = IRQF_PERCPU | IRQF_NO_SUSPEND;
	action->name = devname;
	action->percpu_dev_id = dev_id;

	chip_bus_lock(desc);
	retval = __setup_irq(irq, desc, action);
	chip_bus_sync_unlock(desc);

	if (retval)
		kfree(action);

	return retval;
}
