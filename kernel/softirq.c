/*
 *	linux/kernel/softirq.c
 *
 *	Copyright (C) 1992 Linus Torvalds
 *
 *	Distribute under GPLv2.
 *
 *	Rewritten. Old one was good in 2.2, but in 2.3 it was immoral. --ANK (990903)
 */

#include <linux/export.h>
#include <linux/kernel_stat.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/notifier.h>
#include <linux/percpu.h>
#include <linux/cpu.h>
#include <linux/freezer.h>
#include <linux/kthread.h>
#include <linux/rcupdate.h>
#include <linux/ftrace.h>
#include <linux/smp.h>
#include <linux/smpboot.h>
#include <linux/tick.h>

#define CREATE_TRACE_POINTS
#include <trace/events/irq.h>

#include <asm/irq.h>
#include <linux/msm_rtb.h>

#ifndef __ARCH_IRQ_STAT
irq_cpustat_t irq_stat[NR_CPUS] ____cacheline_aligned;
EXPORT_SYMBOL(irq_stat);
#endif

static struct softirq_action softirq_vec[NR_SOFTIRQS] __cacheline_aligned_in_smp;

DEFINE_PER_CPU(struct task_struct *, ksoftirqd);

char *softirq_to_name[NR_SOFTIRQS] = {
	"HI", "TIMER", "NET_TX", "NET_RX", "BLOCK", "BLOCK_IOPOLL",
	"TASKLET", "SCHED", "HRTIMER", "RCU"
};

static void wakeup_softirqd(void)
{
	
	struct task_struct *tsk = __this_cpu_read(ksoftirqd);

	if (tsk && tsk->state != TASK_RUNNING)
		wake_up_process(tsk);
}


#ifdef CONFIG_TRACE_IRQFLAGS
static void __local_bh_disable(unsigned long ip, unsigned int cnt)
{
	unsigned long flags;

	WARN_ON_ONCE(in_irq());

	raw_local_irq_save(flags);
	preempt_count() += cnt;
	if (softirq_count() == cnt)
		trace_softirqs_off(ip);
	raw_local_irq_restore(flags);

	if (preempt_count() == cnt)
		trace_preempt_off(CALLER_ADDR0, get_parent_ip(CALLER_ADDR1));
}
#else 
static inline void __local_bh_disable(unsigned long ip, unsigned int cnt)
{
	add_preempt_count(cnt);
	barrier();
}
#endif 

void local_bh_disable(void)
{
	__local_bh_disable((unsigned long)__builtin_return_address(0),
				SOFTIRQ_DISABLE_OFFSET);
}

EXPORT_SYMBOL(local_bh_disable);

static void __local_bh_enable(unsigned int cnt)
{
	WARN_ON_ONCE(in_irq());
	WARN_ON_ONCE(!irqs_disabled());

	if (softirq_count() == cnt)
		trace_softirqs_on((unsigned long)__builtin_return_address(0));
	sub_preempt_count(cnt);
}

void _local_bh_enable(void)
{
	__local_bh_enable(SOFTIRQ_DISABLE_OFFSET);
}

EXPORT_SYMBOL(_local_bh_enable);

static inline void _local_bh_enable_ip(unsigned long ip)
{
	WARN_ON_ONCE(in_irq() || irqs_disabled());
#ifdef CONFIG_TRACE_IRQFLAGS
	local_irq_disable();
#endif
	if (softirq_count() == SOFTIRQ_DISABLE_OFFSET)
		trace_softirqs_on(ip);
	sub_preempt_count(SOFTIRQ_DISABLE_OFFSET - 1);

	if (unlikely(!in_interrupt() && local_softirq_pending()))
		do_softirq();

	dec_preempt_count();
#ifdef CONFIG_TRACE_IRQFLAGS
	local_irq_enable();
#endif
	preempt_check_resched();
}

void local_bh_enable(void)
{
	_local_bh_enable_ip((unsigned long)__builtin_return_address(0));
}
EXPORT_SYMBOL(local_bh_enable);

void local_bh_enable_ip(unsigned long ip)
{
	_local_bh_enable_ip(ip);
}
EXPORT_SYMBOL(local_bh_enable_ip);

#define MAX_SOFTIRQ_TIME  msecs_to_jiffies(2)
#define MAX_SOFTIRQ_RESTART 10

asmlinkage void __do_softirq(void)
{
	struct softirq_action *h;
	__u32 pending;
	unsigned long end = jiffies + MAX_SOFTIRQ_TIME;
	int cpu;
	unsigned long old_flags = current->flags;
	int max_restart = MAX_SOFTIRQ_RESTART;

	current->flags &= ~PF_MEMALLOC;

	pending = local_softirq_pending();
	account_irq_enter_time(current);

	__local_bh_disable((unsigned long)__builtin_return_address(0),
				SOFTIRQ_OFFSET);
	lockdep_softirq_enter();

	cpu = smp_processor_id();
restart:
	
	set_softirq_pending(0);

	local_irq_enable();

	h = softirq_vec;

	do {
		if (pending & 1) {
			unsigned int vec_nr = h - softirq_vec;
			int prev_count = preempt_count();

			kstat_incr_softirqs_this_cpu(vec_nr);

			trace_softirq_entry(vec_nr);
			uncached_logk(LOGK_SOFTIRQ, (void *)(h->action));
			h->action(h);
			trace_softirq_exit(vec_nr);
			if (unlikely(prev_count != preempt_count())) {
				printk(KERN_ERR "huh, entered softirq %u %s %p"
				       "with preempt_count %08x,"
				       " exited with %08x?\n", vec_nr,
				       softirq_to_name[vec_nr], h->action,
				       prev_count, preempt_count());
				preempt_count() = prev_count;
			}

			rcu_bh_qs(cpu);
		}
		h++;
		pending >>= 1;
	} while (pending);

	local_irq_disable();

	pending = local_softirq_pending();
	if (pending) {
		if (time_before(jiffies, end) && !need_resched() &&
		    --max_restart)
			goto restart;

		wakeup_softirqd();
	}

	lockdep_softirq_exit();

	account_irq_exit_time(current);
	__local_bh_enable(SOFTIRQ_OFFSET);
	tsk_restore_flags(current, old_flags, PF_MEMALLOC);
}

#ifndef __ARCH_HAS_DO_SOFTIRQ

asmlinkage void do_softirq(void)
{
	__u32 pending;
	unsigned long flags;

	if (in_interrupt())
		return;

	local_irq_save(flags);

	pending = local_softirq_pending();

	if (pending)
		__do_softirq();

	local_irq_restore(flags);
}

#endif

void irq_enter(void)
{
	rcu_irq_enter();
	if (is_idle_task(current) && !in_interrupt()) {
		local_bh_disable();
		tick_check_idle();
		_local_bh_enable();
	}

	__irq_enter();
}

static inline void invoke_softirq(void)
{
	if (!force_irqthreads) {
		do_softirq();
	} else {
		wakeup_softirqd();
	}
}

static inline void tick_irq_exit(void)
{
#ifdef CONFIG_NO_HZ_COMMON
	int cpu = smp_processor_id();

	
	if ((idle_cpu(cpu) && !need_resched()) || tick_nohz_full_cpu(cpu)) {
		if (!in_interrupt())
			tick_nohz_irq_exit();
	}
#endif
}

void irq_exit(void)
{
#ifndef __ARCH_IRQ_EXIT_IRQS_DISABLED
	local_irq_disable();
#else
	WARN_ON_ONCE(!irqs_disabled());
#endif

	account_irq_exit_time(current);
	trace_hardirq_exit();
	sub_preempt_count(HARDIRQ_OFFSET);
	if (!in_interrupt() && local_softirq_pending())
		invoke_softirq();

	tick_irq_exit();
	rcu_irq_exit();
}

inline void raise_softirq_irqoff(unsigned int nr)
{
	__raise_softirq_irqoff(nr);

	if (!in_interrupt())
		wakeup_softirqd();
}

void raise_softirq(unsigned int nr)
{
	unsigned long flags;

	local_irq_save(flags);
	raise_softirq_irqoff(nr);
	local_irq_restore(flags);
}

void __raise_softirq_irqoff(unsigned int nr)
{
	trace_softirq_raise(nr);
	or_softirq_pending(1UL << nr);
}

void open_softirq(int nr, void (*action)(struct softirq_action *))
{
	softirq_vec[nr].action = action;
}

struct tasklet_head
{
	struct tasklet_struct *head;
	struct tasklet_struct **tail;
};

static DEFINE_PER_CPU(struct tasklet_head, tasklet_vec);
static DEFINE_PER_CPU(struct tasklet_head, tasklet_hi_vec);

void __tasklet_schedule(struct tasklet_struct *t)
{
	unsigned long flags;

	local_irq_save(flags);
	t->next = NULL;
	*__this_cpu_read(tasklet_vec.tail) = t;
	__this_cpu_write(tasklet_vec.tail, &(t->next));
	raise_softirq_irqoff(TASKLET_SOFTIRQ);
	local_irq_restore(flags);
}

EXPORT_SYMBOL(__tasklet_schedule);

void __tasklet_hi_schedule(struct tasklet_struct *t)
{
	unsigned long flags;

	local_irq_save(flags);
	t->next = NULL;
	*__this_cpu_read(tasklet_hi_vec.tail) = t;
	__this_cpu_write(tasklet_hi_vec.tail,  &(t->next));
	raise_softirq_irqoff(HI_SOFTIRQ);
	local_irq_restore(flags);
}

EXPORT_SYMBOL(__tasklet_hi_schedule);

void __tasklet_hi_schedule_first(struct tasklet_struct *t)
{
	BUG_ON(!irqs_disabled());

	t->next = __this_cpu_read(tasklet_hi_vec.head);
	__this_cpu_write(tasklet_hi_vec.head, t);
	__raise_softirq_irqoff(HI_SOFTIRQ);
}

EXPORT_SYMBOL(__tasklet_hi_schedule_first);

static void tasklet_action(struct softirq_action *a)
{
	struct tasklet_struct *list;

	local_irq_disable();
	list = __this_cpu_read(tasklet_vec.head);
	__this_cpu_write(tasklet_vec.head, NULL);
	__this_cpu_write(tasklet_vec.tail, &__get_cpu_var(tasklet_vec).head);
	local_irq_enable();

	while (list) {
		struct tasklet_struct *t = list;

		list = list->next;

		if (tasklet_trylock(t)) {
			if (!atomic_read(&t->count)) {
				if (!test_and_clear_bit(TASKLET_STATE_SCHED, &t->state))
					BUG();
				t->func(t->data);
				tasklet_unlock(t);
				continue;
			}
			tasklet_unlock(t);
		}

		local_irq_disable();
		t->next = NULL;
		*__this_cpu_read(tasklet_vec.tail) = t;
		__this_cpu_write(tasklet_vec.tail, &(t->next));
		__raise_softirq_irqoff(TASKLET_SOFTIRQ);
		local_irq_enable();
	}
}

static void tasklet_hi_action(struct softirq_action *a)
{
	struct tasklet_struct *list;

	local_irq_disable();
	list = __this_cpu_read(tasklet_hi_vec.head);
	__this_cpu_write(tasklet_hi_vec.head, NULL);
	__this_cpu_write(tasklet_hi_vec.tail, &__get_cpu_var(tasklet_hi_vec).head);
	local_irq_enable();

	while (list) {
		struct tasklet_struct *t = list;

		list = list->next;

		if (tasklet_trylock(t)) {
			if (!atomic_read(&t->count)) {
				if (!test_and_clear_bit(TASKLET_STATE_SCHED, &t->state))
					BUG();
				t->func(t->data);
				tasklet_unlock(t);
				continue;
			}
			tasklet_unlock(t);
		}

		local_irq_disable();
		t->next = NULL;
		*__this_cpu_read(tasklet_hi_vec.tail) = t;
		__this_cpu_write(tasklet_hi_vec.tail, &(t->next));
		__raise_softirq_irqoff(HI_SOFTIRQ);
		local_irq_enable();
	}
}


void tasklet_init(struct tasklet_struct *t,
		  void (*func)(unsigned long), unsigned long data)
{
	t->next = NULL;
	t->state = 0;
	atomic_set(&t->count, 0);
	t->func = func;
	t->data = data;
}

EXPORT_SYMBOL(tasklet_init);

void tasklet_kill(struct tasklet_struct *t)
{
	if (in_interrupt())
		printk("Attempt to kill tasklet from interrupt\n");

	while (test_and_set_bit(TASKLET_STATE_SCHED, &t->state)) {
		do {
			yield();
		} while (test_bit(TASKLET_STATE_SCHED, &t->state));
	}
	tasklet_unlock_wait(t);
	clear_bit(TASKLET_STATE_SCHED, &t->state);
}

EXPORT_SYMBOL(tasklet_kill);


static enum hrtimer_restart __hrtimer_tasklet_trampoline(struct hrtimer *timer)
{
	struct tasklet_hrtimer *ttimer =
		container_of(timer, struct tasklet_hrtimer, timer);

	tasklet_hi_schedule(&ttimer->tasklet);
	return HRTIMER_NORESTART;
}

static void __tasklet_hrtimer_trampoline(unsigned long data)
{
	struct tasklet_hrtimer *ttimer = (void *)data;
	enum hrtimer_restart restart;

	restart = ttimer->function(&ttimer->timer);
	if (restart != HRTIMER_NORESTART)
		hrtimer_restart(&ttimer->timer);
}

void tasklet_hrtimer_init(struct tasklet_hrtimer *ttimer,
			  enum hrtimer_restart (*function)(struct hrtimer *),
			  clockid_t which_clock, enum hrtimer_mode mode)
{
	hrtimer_init(&ttimer->timer, which_clock, mode);
	ttimer->timer.function = __hrtimer_tasklet_trampoline;
	tasklet_init(&ttimer->tasklet, __tasklet_hrtimer_trampoline,
		     (unsigned long)ttimer);
	ttimer->function = function;
}
EXPORT_SYMBOL_GPL(tasklet_hrtimer_init);

void __init softirq_init(void)
{
	int cpu;

	for_each_possible_cpu(cpu) {
		per_cpu(tasklet_vec, cpu).tail =
			&per_cpu(tasklet_vec, cpu).head;
		per_cpu(tasklet_hi_vec, cpu).tail =
			&per_cpu(tasklet_hi_vec, cpu).head;
	}

	open_softirq(TASKLET_SOFTIRQ, tasklet_action);
	open_softirq(HI_SOFTIRQ, tasklet_hi_action);
}

static int ksoftirqd_should_run(unsigned int cpu)
{
	return local_softirq_pending();
}

static void run_ksoftirqd(unsigned int cpu)
{
	local_irq_disable();
	if (local_softirq_pending()) {
		__do_softirq();
		local_irq_enable();
		cond_resched();

		preempt_disable();
		rcu_note_context_switch(cpu);
		preempt_enable();

		return;
	}
	local_irq_enable();
}

#ifdef CONFIG_HOTPLUG_CPU
void tasklet_kill_immediate(struct tasklet_struct *t, unsigned int cpu)
{
	struct tasklet_struct **i;

	BUG_ON(cpu_online(cpu));
	BUG_ON(test_bit(TASKLET_STATE_RUN, &t->state));

	if (!test_bit(TASKLET_STATE_SCHED, &t->state))
		return;

	
	for (i = &per_cpu(tasklet_vec, cpu).head; *i; i = &(*i)->next) {
		if (*i == t) {
			*i = t->next;
			
			if (*i == NULL)
				per_cpu(tasklet_vec, cpu).tail = i;
			return;
		}
	}
	BUG();
}

static void takeover_tasklets(unsigned int cpu)
{
	
	local_irq_disable();

	
	if (&per_cpu(tasklet_vec, cpu).head != per_cpu(tasklet_vec, cpu).tail) {
		*__this_cpu_read(tasklet_vec.tail) = per_cpu(tasklet_vec, cpu).head;
		this_cpu_write(tasklet_vec.tail, per_cpu(tasklet_vec, cpu).tail);
		per_cpu(tasklet_vec, cpu).head = NULL;
		per_cpu(tasklet_vec, cpu).tail = &per_cpu(tasklet_vec, cpu).head;
	}
	raise_softirq_irqoff(TASKLET_SOFTIRQ);

	if (&per_cpu(tasklet_hi_vec, cpu).head != per_cpu(tasklet_hi_vec, cpu).tail) {
		*__this_cpu_read(tasklet_hi_vec.tail) = per_cpu(tasklet_hi_vec, cpu).head;
		__this_cpu_write(tasklet_hi_vec.tail, per_cpu(tasklet_hi_vec, cpu).tail);
		per_cpu(tasklet_hi_vec, cpu).head = NULL;
		per_cpu(tasklet_hi_vec, cpu).tail = &per_cpu(tasklet_hi_vec, cpu).head;
	}
	raise_softirq_irqoff(HI_SOFTIRQ);

	local_irq_enable();
}
#endif 

static int __cpuinit cpu_callback(struct notifier_block *nfb,
				  unsigned long action,
				  void *hcpu)
{
	switch (action) {
#ifdef CONFIG_HOTPLUG_CPU
	case CPU_DEAD:
	case CPU_DEAD_FROZEN:
		takeover_tasklets((unsigned long)hcpu);
		break;
#endif 
	}
	return NOTIFY_OK;
}

static struct notifier_block __cpuinitdata cpu_nfb = {
	.notifier_call = cpu_callback
};

static struct smp_hotplug_thread softirq_threads = {
	.store			= &ksoftirqd,
	.thread_should_run	= ksoftirqd_should_run,
	.thread_fn		= run_ksoftirqd,
	.thread_comm		= "ksoftirqd/%u",
};

static __init int spawn_ksoftirqd(void)
{
	register_cpu_notifier(&cpu_nfb);

	BUG_ON(smpboot_register_percpu_thread(&softirq_threads));

	return 0;
}
early_initcall(spawn_ksoftirqd);


int __init __weak early_irq_init(void)
{
	return 0;
}

int __init __weak arch_probe_nr_irqs(void)
{
	return NR_IRQS_LEGACY;
}

int __init __weak arch_early_irq_init(void)
{
	return 0;
}
