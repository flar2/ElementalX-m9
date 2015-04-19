/*
 * Completely Fair Scheduling (CFS) Class (SCHED_NORMAL/SCHED_BATCH)
 *
 *  Copyright (C) 2007 Red Hat, Inc., Ingo Molnar <mingo@redhat.com>
 *
 *  Interactivity improvements by Mike Galbraith
 *  (C) 2007 Mike Galbraith <efault@gmx.de>
 *
 *  Various enhancements by Dmitry Adamushko.
 *  (C) 2007 Dmitry Adamushko <dmitry.adamushko@gmail.com>
 *
 *  Group scheduling enhancements by Srivatsa Vaddagiri
 *  Copyright IBM Corporation, 2007
 *  Author: Srivatsa Vaddagiri <vatsa@linux.vnet.ibm.com>
 *
 *  Scaled math optimizations by Thomas Gleixner
 *  Copyright (C) 2007, Thomas Gleixner <tglx@linutronix.de>
 *
 *  Adaptive scheduling granularity, math enhancements by Peter Zijlstra
 *  Copyright (C) 2007 Red Hat, Inc., Peter Zijlstra <pzijlstr@redhat.com>
 */

#include <linux/latencytop.h>
#include <linux/sched.h>
#include <linux/cpumask.h>
#include <linux/slab.h>
#include <linux/profile.h>
#include <linux/interrupt.h>
#include <linux/mempolicy.h>
#include <linux/migrate.h>
#include <linux/task_work.h>

#include <trace/events/sched.h>

#include "sched.h"

unsigned int sysctl_sched_latency = 6000000ULL;
unsigned int normalized_sysctl_sched_latency = 6000000ULL;

enum sched_tunable_scaling sysctl_sched_tunable_scaling
	= SCHED_TUNABLESCALING_LOG;

unsigned int sysctl_sched_min_granularity = 750000ULL;
unsigned int normalized_sysctl_sched_min_granularity = 750000ULL;

static unsigned int sched_nr_latency = 8;

unsigned int sysctl_sched_child_runs_first __read_mostly;

unsigned int __read_mostly sysctl_sched_wake_to_idle;

unsigned int sysctl_sched_wakeup_granularity = 1000000UL;
unsigned int normalized_sysctl_sched_wakeup_granularity = 1000000UL;

const_debug unsigned int sysctl_sched_migration_cost = 500000UL;

unsigned int __read_mostly sysctl_sched_shares_window = 10000000UL;

#ifdef CONFIG_CFS_BANDWIDTH
unsigned int sysctl_sched_cfs_bandwidth_slice = 5000UL;
#endif

static int get_update_sysctl_factor(void)
{
	unsigned int cpus = min_t(int, num_online_cpus(), 8);
	unsigned int factor;

	switch (sysctl_sched_tunable_scaling) {
	case SCHED_TUNABLESCALING_NONE:
		factor = 1;
		break;
	case SCHED_TUNABLESCALING_LINEAR:
		factor = cpus;
		break;
	case SCHED_TUNABLESCALING_LOG:
	default:
		factor = 1 + ilog2(cpus);
		break;
	}

	return factor;
}

static void update_sysctl(void)
{
	unsigned int factor = get_update_sysctl_factor();

#define SET_SYSCTL(name) \
	(sysctl_##name = (factor) * normalized_sysctl_##name)
	SET_SYSCTL(sched_min_granularity);
	SET_SYSCTL(sched_latency);
	SET_SYSCTL(sched_wakeup_granularity);
#undef SET_SYSCTL
}

void sched_init_granularity(void)
{
	update_sysctl();
}

#if BITS_PER_LONG == 32
# define WMULT_CONST	(~0UL)
#else
# define WMULT_CONST	(1UL << 32)
#endif

#define WMULT_SHIFT	32

#define SRR(x, y) (((x) + (1UL << ((y) - 1))) >> (y))

static unsigned long
calc_delta_mine(unsigned long delta_exec, unsigned long weight,
		struct load_weight *lw)
{
	u64 tmp;

	if (likely(weight > (1UL << SCHED_LOAD_RESOLUTION)))
		tmp = (u64)delta_exec * scale_load_down(weight);
	else
		tmp = (u64)delta_exec;

	if (!lw->inv_weight) {
		unsigned long w = scale_load_down(lw->weight);

		if (BITS_PER_LONG > 32 && unlikely(w >= WMULT_CONST))
			lw->inv_weight = 1;
		else if (unlikely(!w))
			lw->inv_weight = WMULT_CONST;
		else
			lw->inv_weight = WMULT_CONST / w;
	}

	if (unlikely(tmp > WMULT_CONST))
		tmp = SRR(SRR(tmp, WMULT_SHIFT/2) * lw->inv_weight,
			WMULT_SHIFT/2);
	else
		tmp = SRR(tmp * lw->inv_weight, WMULT_SHIFT);

	return (unsigned long)min(tmp, (u64)(unsigned long)LONG_MAX);
}

#ifdef CONFIG_SMP
static int active_load_balance_cpu_stop(void *data);
#endif

const struct sched_class fair_sched_class;


#ifdef CONFIG_FAIR_GROUP_SCHED

static inline struct rq *rq_of(struct cfs_rq *cfs_rq)
{
	return cfs_rq->rq;
}

#define entity_is_task(se)	(!se->my_q)

static inline struct task_struct *task_of(struct sched_entity *se)
{
#ifdef CONFIG_SCHED_DEBUG
	WARN_ON_ONCE(!entity_is_task(se));
#endif
	return container_of(se, struct task_struct, se);
}

#define for_each_sched_entity(se) \
		for (; se; se = se->parent)

static inline struct cfs_rq *task_cfs_rq(struct task_struct *p)
{
	return p->se.cfs_rq;
}

static inline struct cfs_rq *cfs_rq_of(struct sched_entity *se)
{
	return se->cfs_rq;
}

static inline struct cfs_rq *group_cfs_rq(struct sched_entity *grp)
{
	return grp->my_q;
}

static void update_cfs_rq_blocked_load(struct cfs_rq *cfs_rq,
				       int force_update);

static inline void list_add_leaf_cfs_rq(struct cfs_rq *cfs_rq)
{
	if (!cfs_rq->on_list) {
		if (cfs_rq->tg->parent &&
		    cfs_rq->tg->parent->cfs_rq[cpu_of(rq_of(cfs_rq))]->on_list) {
			list_add_rcu(&cfs_rq->leaf_cfs_rq_list,
				&rq_of(cfs_rq)->leaf_cfs_rq_list);
		} else {
			list_add_tail_rcu(&cfs_rq->leaf_cfs_rq_list,
				&rq_of(cfs_rq)->leaf_cfs_rq_list);
		}

		cfs_rq->on_list = 1;
		
		update_cfs_rq_blocked_load(cfs_rq, 0);
	}
}

static inline void list_del_leaf_cfs_rq(struct cfs_rq *cfs_rq)
{
	if (cfs_rq->on_list) {
		list_del_rcu(&cfs_rq->leaf_cfs_rq_list);
		cfs_rq->on_list = 0;
	}
}

#define for_each_leaf_cfs_rq(rq, cfs_rq) \
	list_for_each_entry_rcu(cfs_rq, &rq->leaf_cfs_rq_list, leaf_cfs_rq_list)

static inline int
is_same_group(struct sched_entity *se, struct sched_entity *pse)
{
	if (se->cfs_rq == pse->cfs_rq)
		return 1;

	return 0;
}

static inline struct sched_entity *parent_entity(struct sched_entity *se)
{
	return se->parent;
}

static inline int depth_se(struct sched_entity *se)
{
	int depth = 0;

	for_each_sched_entity(se)
		depth++;

	return depth;
}

static void
find_matching_se(struct sched_entity **se, struct sched_entity **pse)
{
	int se_depth, pse_depth;


	
	se_depth = depth_se(*se);
	pse_depth = depth_se(*pse);

	while (se_depth > pse_depth) {
		se_depth--;
		*se = parent_entity(*se);
	}

	while (pse_depth > se_depth) {
		pse_depth--;
		*pse = parent_entity(*pse);
	}

	while (!is_same_group(*se, *pse)) {
		*se = parent_entity(*se);
		*pse = parent_entity(*pse);
	}
}

#else	

static inline struct task_struct *task_of(struct sched_entity *se)
{
	return container_of(se, struct task_struct, se);
}

static inline struct rq *rq_of(struct cfs_rq *cfs_rq)
{
	return container_of(cfs_rq, struct rq, cfs);
}

#define entity_is_task(se)	1

#define for_each_sched_entity(se) \
		for (; se; se = NULL)

static inline struct cfs_rq *task_cfs_rq(struct task_struct *p)
{
	return &task_rq(p)->cfs;
}

static inline struct cfs_rq *cfs_rq_of(struct sched_entity *se)
{
	struct task_struct *p = task_of(se);
	struct rq *rq = task_rq(p);

	return &rq->cfs;
}

static inline struct cfs_rq *group_cfs_rq(struct sched_entity *grp)
{
	return NULL;
}

static inline void list_add_leaf_cfs_rq(struct cfs_rq *cfs_rq)
{
}

static inline void list_del_leaf_cfs_rq(struct cfs_rq *cfs_rq)
{
}

#define for_each_leaf_cfs_rq(rq, cfs_rq) \
		for (cfs_rq = &rq->cfs; cfs_rq; cfs_rq = NULL)

static inline int
is_same_group(struct sched_entity *se, struct sched_entity *pse)
{
	return 1;
}

static inline struct sched_entity *parent_entity(struct sched_entity *se)
{
	return NULL;
}

static inline void
find_matching_se(struct sched_entity **se, struct sched_entity **pse)
{
}

#endif	

static __always_inline
void account_cfs_rq_runtime(struct cfs_rq *cfs_rq, unsigned long delta_exec);


static inline u64 max_vruntime(u64 max_vruntime, u64 vruntime)
{
	s64 delta = (s64)(vruntime - max_vruntime);
	if (delta > 0)
		max_vruntime = vruntime;

	return max_vruntime;
}

static inline u64 min_vruntime(u64 min_vruntime, u64 vruntime)
{
	s64 delta = (s64)(vruntime - min_vruntime);
	if (delta < 0)
		min_vruntime = vruntime;

	return min_vruntime;
}

static inline int entity_before(struct sched_entity *a,
				struct sched_entity *b)
{
	return (s64)(a->vruntime - b->vruntime) < 0;
}

static void update_min_vruntime(struct cfs_rq *cfs_rq)
{
	u64 vruntime = cfs_rq->min_vruntime;

	if (cfs_rq->curr)
		vruntime = cfs_rq->curr->vruntime;

	if (cfs_rq->rb_leftmost) {
		struct sched_entity *se = rb_entry(cfs_rq->rb_leftmost,
						   struct sched_entity,
						   run_node);

		if (!cfs_rq->curr)
			vruntime = se->vruntime;
		else
			vruntime = min_vruntime(vruntime, se->vruntime);
	}

	
	cfs_rq->min_vruntime = max_vruntime(cfs_rq->min_vruntime, vruntime);
#ifndef CONFIG_64BIT
	smp_wmb();
	cfs_rq->min_vruntime_copy = cfs_rq->min_vruntime;
#endif
}

static void __enqueue_entity(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	struct rb_node **link = &cfs_rq->tasks_timeline.rb_node;
	struct rb_node *parent = NULL;
	struct sched_entity *entry;
	int leftmost = 1;

	while (*link) {
		parent = *link;
		entry = rb_entry(parent, struct sched_entity, run_node);
		if (entity_before(se, entry)) {
			link = &parent->rb_left;
		} else {
			link = &parent->rb_right;
			leftmost = 0;
		}
	}

	if (leftmost)
		cfs_rq->rb_leftmost = &se->run_node;

	rb_link_node(&se->run_node, parent, link);
	rb_insert_color(&se->run_node, &cfs_rq->tasks_timeline);
}

static void __dequeue_entity(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	if (cfs_rq->rb_leftmost == &se->run_node) {
		struct rb_node *next_node;

		next_node = rb_next(&se->run_node);
		cfs_rq->rb_leftmost = next_node;
	}

	rb_erase(&se->run_node, &cfs_rq->tasks_timeline);
}

struct sched_entity *__pick_first_entity(struct cfs_rq *cfs_rq)
{
	struct rb_node *left = cfs_rq->rb_leftmost;

	if (!left)
		return NULL;

	return rb_entry(left, struct sched_entity, run_node);
}

static struct sched_entity *__pick_next_entity(struct sched_entity *se)
{
	struct rb_node *next = rb_next(&se->run_node);

	if (!next)
		return NULL;

	return rb_entry(next, struct sched_entity, run_node);
}

#ifdef CONFIG_SCHED_DEBUG
struct sched_entity *__pick_last_entity(struct cfs_rq *cfs_rq)
{
	struct rb_node *last = rb_last(&cfs_rq->tasks_timeline);

	if (!last)
		return NULL;

	return rb_entry(last, struct sched_entity, run_node);
}


int sched_proc_update_handler(struct ctl_table *table, int write,
		void __user *buffer, size_t *lenp,
		loff_t *ppos)
{
	int ret = proc_dointvec_minmax(table, write, buffer, lenp, ppos);
	int factor = get_update_sysctl_factor();

	if (ret || !write)
		return ret;

	sched_nr_latency = DIV_ROUND_UP(sysctl_sched_latency,
					sysctl_sched_min_granularity);

#define WRT_SYSCTL(name) \
	(normalized_sysctl_##name = sysctl_##name / (factor))
	WRT_SYSCTL(sched_min_granularity);
	WRT_SYSCTL(sched_latency);
	WRT_SYSCTL(sched_wakeup_granularity);
#undef WRT_SYSCTL

	return 0;
}
#endif

static inline unsigned long
calc_delta_fair(unsigned long delta, struct sched_entity *se)
{
	if (unlikely(se->load.weight != NICE_0_LOAD))
		delta = calc_delta_mine(delta, NICE_0_LOAD, &se->load);

	return delta;
}

static u64 __sched_period(unsigned long nr_running)
{
	u64 period = sysctl_sched_latency;
	unsigned long nr_latency = sched_nr_latency;

	if (unlikely(nr_running > nr_latency)) {
		period = sysctl_sched_min_granularity;
		period *= nr_running;
	}

	return period;
}

static u64 sched_slice(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	u64 slice = __sched_period(cfs_rq->nr_running + !se->on_rq);

	for_each_sched_entity(se) {
		struct load_weight *load;
		struct load_weight lw;

		cfs_rq = cfs_rq_of(se);
		load = &cfs_rq->load;

		if (unlikely(!se->on_rq)) {
			lw = cfs_rq->load;

			update_load_add(&lw, se->load.weight);
			load = &lw;
		}
		slice = calc_delta_mine(slice, se->load.weight, load);
	}
	return slice;
}

static u64 sched_vslice(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	return calc_delta_fair(sched_slice(cfs_rq, se), se);
}

static inline void
__update_curr(struct cfs_rq *cfs_rq, struct sched_entity *curr,
	      unsigned long delta_exec)
{
	unsigned long delta_exec_weighted;

	schedstat_set(curr->statistics.exec_max,
		      max((u64)delta_exec, curr->statistics.exec_max));

	curr->sum_exec_runtime += delta_exec;
	schedstat_add(cfs_rq, exec_clock, delta_exec);
	delta_exec_weighted = calc_delta_fair(delta_exec, curr);

	curr->vruntime += delta_exec_weighted;
	update_min_vruntime(cfs_rq);
}

static void update_curr(struct cfs_rq *cfs_rq)
{
	struct sched_entity *curr = cfs_rq->curr;
	u64 now = rq_of(cfs_rq)->clock_task;
	unsigned long delta_exec;

	if (unlikely(!curr))
		return;

	delta_exec = (unsigned long)(now - curr->exec_start);
	if (!delta_exec)
		return;

	__update_curr(cfs_rq, curr, delta_exec);
	curr->exec_start = now;

	if (entity_is_task(curr)) {
		struct task_struct *curtask = task_of(curr);

		trace_sched_stat_runtime(curtask, delta_exec, curr->vruntime);
		cpuacct_charge(curtask, delta_exec);
		account_group_exec_runtime(curtask, delta_exec);
	}

	account_cfs_rq_runtime(cfs_rq, delta_exec);
}

static inline void
update_stats_wait_start(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	schedstat_set(se->statistics.wait_start, rq_of(cfs_rq)->clock);
}

static void update_stats_enqueue(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	if (se != cfs_rq->curr)
		update_stats_wait_start(cfs_rq, se);
}

static void
update_stats_wait_end(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	schedstat_set(se->statistics.wait_max, max(se->statistics.wait_max,
			rq_of(cfs_rq)->clock - se->statistics.wait_start));
	schedstat_set(se->statistics.wait_count, se->statistics.wait_count + 1);
	schedstat_set(se->statistics.wait_sum, se->statistics.wait_sum +
			rq_of(cfs_rq)->clock - se->statistics.wait_start);
#ifdef CONFIG_SCHEDSTATS
	if (entity_is_task(se)) {
		trace_sched_stat_wait(task_of(se),
			rq_of(cfs_rq)->clock - se->statistics.wait_start);
	}
#endif
	schedstat_set(se->statistics.wait_start, 0);
}

static inline void
update_stats_dequeue(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	if (se != cfs_rq->curr)
		update_stats_wait_end(cfs_rq, se);
}

static inline void
update_stats_curr_start(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	se->exec_start = rq_of(cfs_rq)->clock_task;
}


#ifdef CONFIG_NUMA_BALANCING
unsigned int sysctl_numa_balancing_scan_period_min = 100;
unsigned int sysctl_numa_balancing_scan_period_max = 100*50;
unsigned int sysctl_numa_balancing_scan_period_reset = 100*600;

unsigned int sysctl_numa_balancing_scan_size = 256;

unsigned int sysctl_numa_balancing_scan_delay = 1000;

static void task_numa_placement(struct task_struct *p)
{
	int seq;

	if (!p->mm)	
		return;
	seq = ACCESS_ONCE(p->mm->numa_scan_seq);
	if (p->numa_scan_seq == seq)
		return;
	p->numa_scan_seq = seq;

	
}

void task_numa_fault(int node, int pages, bool migrated)
{
	struct task_struct *p = current;

	if (!sched_feat_numa(NUMA))
		return;

	

        if (!migrated)
		p->numa_scan_period = min(sysctl_numa_balancing_scan_period_max,
			p->numa_scan_period + jiffies_to_msecs(10));

	task_numa_placement(p);
}

static void reset_ptenuma_scan(struct task_struct *p)
{
	ACCESS_ONCE(p->mm->numa_scan_seq)++;
	p->mm->numa_scan_offset = 0;
}

void task_numa_work(struct callback_head *work)
{
	unsigned long migrate, next_scan, now = jiffies;
	struct task_struct *p = current;
	struct mm_struct *mm = p->mm;
	struct vm_area_struct *vma;
	unsigned long start, end;
	long pages;

	WARN_ON_ONCE(p != container_of(work, struct task_struct, numa_work));

	work->next = work; 
	if (p->flags & PF_EXITING)
		return;

	if (mm->first_nid == NUMA_PTE_SCAN_INIT)
		mm->first_nid = numa_node_id();
	if (mm->first_nid != NUMA_PTE_SCAN_ACTIVE) {
		
		if (numa_node_id() == mm->first_nid &&
		    !sched_feat_numa(NUMA_FORCE))
			return;

		mm->first_nid = NUMA_PTE_SCAN_ACTIVE;
	}

	migrate = mm->numa_next_reset;
	if (time_after(now, migrate)) {
		p->numa_scan_period = sysctl_numa_balancing_scan_period_min;
		next_scan = now + msecs_to_jiffies(sysctl_numa_balancing_scan_period_reset);
		xchg(&mm->numa_next_reset, next_scan);
	}

	migrate = mm->numa_next_scan;
	if (time_before(now, migrate))
		return;

	if (p->numa_scan_period == 0)
		p->numa_scan_period = sysctl_numa_balancing_scan_period_min;

	next_scan = now + msecs_to_jiffies(p->numa_scan_period);
	if (cmpxchg(&mm->numa_next_scan, migrate, next_scan) != migrate)
		return;

	if (migrate_ratelimited(numa_node_id()))
		return;

	start = mm->numa_scan_offset;
	pages = sysctl_numa_balancing_scan_size;
	pages <<= 20 - PAGE_SHIFT; 
	if (!pages)
		return;

	down_read(&mm->mmap_sem);
	vma = find_vma(mm, start);
	if (!vma) {
		reset_ptenuma_scan(p);
		start = 0;
		vma = mm->mmap;
	}
	for (; vma; vma = vma->vm_next) {
		if (!vma_migratable(vma))
			continue;

		
		if (vma->vm_end - vma->vm_start < HPAGE_SIZE)
			continue;

		if (!(vma->vm_flags & (VM_READ | VM_EXEC | VM_WRITE)))
			continue;

		do {
			start = max(start, vma->vm_start);
			end = ALIGN(start + (pages << PAGE_SHIFT), HPAGE_SIZE);
			end = min(end, vma->vm_end);
			pages -= change_prot_numa(vma, start, end);

			start = end;
			if (pages <= 0)
				goto out;
		} while (end != vma->vm_end);
	}

out:
	if (vma)
		mm->numa_scan_offset = start;
	else
		reset_ptenuma_scan(p);
	up_read(&mm->mmap_sem);
}

void task_tick_numa(struct rq *rq, struct task_struct *curr)
{
	struct callback_head *work = &curr->numa_work;
	u64 period, now;

	if (!curr->mm || (curr->flags & PF_EXITING) || work->next != work)
		return;

	now = curr->se.sum_exec_runtime;
	period = (u64)curr->numa_scan_period * NSEC_PER_MSEC;

	if (now - curr->node_stamp > period) {
		if (!curr->node_stamp)
			curr->numa_scan_period = sysctl_numa_balancing_scan_period_min;
		curr->node_stamp = now;

		if (!time_before(jiffies, curr->mm->numa_next_scan)) {
			init_task_work(work, task_numa_work); 
			task_work_add(curr, work, true);
		}
	}
}
#else
static void task_tick_numa(struct rq *rq, struct task_struct *curr)
{
}
#endif 

static void
account_entity_enqueue(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	update_load_add(&cfs_rq->load, se->load.weight);
	if (!parent_entity(se))
		update_load_add(&rq_of(cfs_rq)->load, se->load.weight);
#ifdef CONFIG_SMP
	if (entity_is_task(se))
		list_add(&se->group_node, &rq_of(cfs_rq)->cfs_tasks);
#endif
	cfs_rq->nr_running++;
}

static void
account_entity_dequeue(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	update_load_sub(&cfs_rq->load, se->load.weight);
	if (!parent_entity(se))
		update_load_sub(&rq_of(cfs_rq)->load, se->load.weight);
	if (entity_is_task(se))
		list_del_init(&se->group_node);
	cfs_rq->nr_running--;
}

#ifdef CONFIG_FAIR_GROUP_SCHED
# ifdef CONFIG_SMP
static inline long calc_tg_weight(struct task_group *tg, struct cfs_rq *cfs_rq)
{
	long tg_weight;

	tg_weight = atomic64_read(&tg->load_avg);
	tg_weight -= cfs_rq->tg_load_contrib;
	tg_weight += cfs_rq->load.weight;

	return tg_weight;
}

static long calc_cfs_shares(struct cfs_rq *cfs_rq, struct task_group *tg)
{
	long tg_weight, load, shares;

	tg_weight = calc_tg_weight(tg, cfs_rq);
	load = cfs_rq->load.weight;

	shares = (tg->shares * load);
	if (tg_weight)
		shares /= tg_weight;

	if (shares < MIN_SHARES)
		shares = MIN_SHARES;
	if (shares > tg->shares)
		shares = tg->shares;

	return shares;
}
# else 
static inline long calc_cfs_shares(struct cfs_rq *cfs_rq, struct task_group *tg)
{
	return tg->shares;
}
# endif 
static void reweight_entity(struct cfs_rq *cfs_rq, struct sched_entity *se,
			    unsigned long weight)
{
	if (se->on_rq) {
		
		if (cfs_rq->curr == se)
			update_curr(cfs_rq);
		account_entity_dequeue(cfs_rq, se);
	}

	update_load_set(&se->load, weight);

	if (se->on_rq)
		account_entity_enqueue(cfs_rq, se);
}

static inline int throttled_hierarchy(struct cfs_rq *cfs_rq);

static void update_cfs_shares(struct cfs_rq *cfs_rq)
{
	struct task_group *tg;
	struct sched_entity *se;
	long shares;

	tg = cfs_rq->tg;
	se = tg->se[cpu_of(rq_of(cfs_rq))];
	if (!se || throttled_hierarchy(cfs_rq))
		return;
#ifndef CONFIG_SMP
	if (likely(se->load.weight == tg->shares))
		return;
#endif
	shares = calc_cfs_shares(cfs_rq, tg);

	reweight_entity(cfs_rq_of(se), se, shares);
}
#else 
static inline void update_cfs_shares(struct cfs_rq *cfs_rq)
{
}
#endif 

#if defined(CONFIG_SMP) && defined(CONFIG_FAIR_GROUP_SCHED)
#define LOAD_AVG_PERIOD 32
#define LOAD_AVG_MAX 47742 
#define LOAD_AVG_MAX_N 345 

static const u32 runnable_avg_yN_inv[] = {
	0xffffffff, 0xfa83b2da, 0xf5257d14, 0xefe4b99a, 0xeac0c6e6, 0xe5b906e6,
	0xe0ccdeeb, 0xdbfbb796, 0xd744fcc9, 0xd2a81d91, 0xce248c14, 0xc9b9bd85,
	0xc5672a10, 0xc12c4cc9, 0xbd08a39e, 0xb8fbaf46, 0xb504f333, 0xb123f581,
	0xad583ee9, 0xa9a15ab4, 0xa5fed6a9, 0xa2704302, 0x9ef5325f, 0x9b8d39b9,
	0x9837f050, 0x94f4efa8, 0x91c3d373, 0x8ea4398a, 0x8b95c1e3, 0x88980e80,
	0x85aac367, 0x82cd8698,
};

static const u32 runnable_avg_yN_sum[] = {
	    0, 1002, 1982, 2941, 3880, 4798, 5697, 6576, 7437, 8279, 9103,
	 9909,10698,11470,12226,12966,13690,14398,15091,15769,16433,17082,
	17718,18340,18949,19545,20128,20698,21256,21802,22336,22859,23371,
};

static __always_inline u64 decay_load(u64 val, u64 n)
{
	unsigned int local_n;

	if (!n)
		return val;
	else if (unlikely(n > LOAD_AVG_PERIOD * 63))
		return 0;

	
	local_n = n;

	if (unlikely(local_n >= LOAD_AVG_PERIOD)) {
		val >>= local_n / LOAD_AVG_PERIOD;
		local_n %= LOAD_AVG_PERIOD;
	}

	val *= runnable_avg_yN_inv[local_n];
	
	return val >> 32;
}

static u32 __compute_runnable_contrib(u64 n)
{
	u32 contrib = 0;

	if (likely(n <= LOAD_AVG_PERIOD))
		return runnable_avg_yN_sum[n];
	else if (unlikely(n >= LOAD_AVG_MAX_N))
		return LOAD_AVG_MAX;

	
	do {
		contrib /= 2; 
		contrib += runnable_avg_yN_sum[LOAD_AVG_PERIOD];

		n -= LOAD_AVG_PERIOD;
	} while (n > LOAD_AVG_PERIOD);

	contrib = decay_load(contrib, n);
	return contrib + runnable_avg_yN_sum[n];
}

static void add_to_scaled_stat(int cpu, struct sched_avg *sa, u64 delta);
static inline void decay_scaled_stat(struct sched_avg *sa, u64 periods);

#ifdef CONFIG_SCHED_HMP

unsigned int __read_mostly sched_init_task_load_pelt;
unsigned int __read_mostly sched_init_task_load_windows;
unsigned int __read_mostly sysctl_sched_init_task_load_pct = 15;

unsigned int __read_mostly sysctl_sched_min_runtime = 0; 
u64 __read_mostly sched_min_runtime = 0; 

static inline unsigned int task_load(struct task_struct *p)
{
	if (sched_use_pelt)
		return p->se.avg.runnable_avg_sum_scaled;

	return p->ravg.demand;
}

unsigned int max_task_load(void)
{
	if (sched_use_pelt)
		return LOAD_AVG_MAX;

	return sched_ravg_window;
}

unsigned int __read_mostly sched_enable_hmp = 0;

unsigned int __read_mostly sysctl_sched_spill_nr_run = 10;

unsigned int __read_mostly sched_enable_power_aware = 0;

unsigned int __read_mostly sysctl_sched_powerband_limit_pct = 20;

unsigned int __read_mostly sched_spill_load;
unsigned int __read_mostly sysctl_sched_spill_load_pct = 100;

unsigned int __read_mostly sched_small_task;
unsigned int __read_mostly sysctl_sched_small_task_pct = 10;

#ifdef CONFIG_SCHED_FREQ_INPUT
unsigned int __read_mostly sysctl_sched_heavy_task_pct;
unsigned int __read_mostly sched_heavy_task;
#endif

unsigned int __read_mostly sched_upmigrate;
unsigned int __read_mostly sysctl_sched_upmigrate_pct = 80;

unsigned int __read_mostly sched_downmigrate;
unsigned int __read_mostly sysctl_sched_downmigrate_pct = 60;

int __read_mostly sysctl_sched_upmigrate_min_nice = 15;

unsigned int sysctl_sched_boost;

static inline int available_cpu_capacity(int cpu)
{
	struct rq *rq = cpu_rq(cpu);

	return rq->capacity;
}

void set_hmp_defaults(void)
{
	sched_spill_load =
		pct_to_real(sysctl_sched_spill_load_pct);

	sched_small_task =
		pct_to_real(sysctl_sched_small_task_pct);

	sched_upmigrate =
		pct_to_real(sysctl_sched_upmigrate_pct);

	sched_downmigrate =
		pct_to_real(sysctl_sched_downmigrate_pct);

#ifdef CONFIG_SCHED_FREQ_INPUT
	sched_heavy_task =
		pct_to_real(sysctl_sched_heavy_task_pct);
#endif

	sched_init_task_load_pelt =
		div64_u64((u64)sysctl_sched_init_task_load_pct *
			  (u64)LOAD_AVG_MAX, 100);

	sched_init_task_load_windows =
		div64_u64((u64)sysctl_sched_init_task_load_pct *
			  (u64)sched_ravg_window, 100);
}

u32 sched_get_init_task_load(struct task_struct *p)
{
	return p->init_load_pct;
}

int sched_set_init_task_load(struct task_struct *p, int init_load_pct)
{
	if (init_load_pct < 0 || init_load_pct > 100)
		return -EINVAL;

	p->init_load_pct = init_load_pct;

	return 0;
}

int sched_set_cpu_prefer_idle(int cpu, int prefer_idle)
{
	struct rq *rq = cpu_rq(cpu);

	rq->prefer_idle = !!prefer_idle;

	return 0;
}

int sched_get_cpu_prefer_idle(int cpu)
{
	struct rq *rq = cpu_rq(cpu);

	return rq->prefer_idle;
}

int sched_set_cpu_mostly_idle_load(int cpu, int mostly_idle_pct)
{
	struct rq *rq = cpu_rq(cpu);

	if (mostly_idle_pct < 0 || mostly_idle_pct > 100)
		return -EINVAL;

	rq->mostly_idle_load = pct_to_real(mostly_idle_pct);

	return 0;
}

int sched_set_cpu_mostly_idle_freq(int cpu, unsigned int mostly_idle_freq)
{
	struct rq *rq = cpu_rq(cpu);

	if (mostly_idle_freq > rq->max_possible_freq)
		return -EINVAL;

	rq->mostly_idle_freq = mostly_idle_freq;

	return 0;
}

unsigned int sched_get_cpu_mostly_idle_freq(int cpu)
{
	struct rq *rq = cpu_rq(cpu);

	return rq->mostly_idle_freq;
}

int sched_get_cpu_mostly_idle_load(int cpu)
{
	struct rq *rq = cpu_rq(cpu);
	int mostly_idle_pct;

	mostly_idle_pct = real_to_pct(rq->mostly_idle_load);

	return mostly_idle_pct;
}

int sched_set_cpu_mostly_idle_nr_run(int cpu, int nr_run)
{
	struct rq *rq = cpu_rq(cpu);

	rq->mostly_idle_nr_run = nr_run;

	return 0;
}

int sched_get_cpu_mostly_idle_nr_run(int cpu)
{
	struct rq *rq = cpu_rq(cpu);

	return rq->mostly_idle_nr_run;
}

int sched_set_cpu_budget(int cpu, int budget)
{
	struct rq *rq = cpu_rq(cpu);

	rq->budget = budget;

	return 0;
}

int sched_get_cpu_budget(int cpu)
{
	struct rq *rq = cpu_rq(cpu);

	return rq->budget;
}

u64 scale_load_to_cpu(u64 task_load, int cpu)
{
	struct rq *rq = cpu_rq(cpu);

	task_load *= (u64)rq->load_scale_factor;
	task_load /= 1024;

	return task_load;
}

static inline int is_big_task(struct task_struct *p)
{
	u64 load = task_load(p);
	int nice = TASK_NICE(p);

	
	if (nice > sysctl_sched_upmigrate_min_nice)
		return 0;

	load = scale_load_to_cpu(load, task_cpu(p));

	return load > sched_upmigrate;
}

static inline int is_small_task(struct task_struct *p)
{
	u64 load = task_load(p);
	load *= (u64)max_load_scale_factor;
	load /= 1024;
	return load < sched_small_task;
}

static inline u64 cpu_load(int cpu)
{
	struct rq *rq = cpu_rq(cpu);

	return scale_load_to_cpu(rq->cumulative_runnable_avg, cpu);
}

static inline u64 cpu_load_sync(int cpu, int sync)
{
	struct rq *rq = cpu_rq(cpu);
	u64 load;

	load = rq->cumulative_runnable_avg;

	if (sync && cpu == smp_processor_id()) {
		if (load > rq->curr->ravg.demand)
			load -= rq->curr->ravg.demand;
		else
			load = 0;
	}

	return scale_load_to_cpu(load, cpu);
}

static int
spill_threshold_crossed(struct task_struct *p, struct rq *rq, int cpu,
			int sync)
{
	u64 total_load = cpu_load_sync(cpu, sync) +
		scale_load_to_cpu(task_load(p), cpu);

	if (total_load > sched_spill_load ||
	    (rq->nr_running + 1) > sysctl_sched_spill_nr_run)
		return 1;

	return 0;
}

int mostly_idle_cpu(int cpu)
{
	struct rq *rq = cpu_rq(cpu);

	return cpu_load(cpu) <= rq->mostly_idle_load
		&& rq->nr_running <= rq->mostly_idle_nr_run
		&& !sched_cpu_high_irqload(cpu);
}

static int mostly_idle_cpu_sync(int cpu, int sync)
{
	struct rq *rq = cpu_rq(cpu);
	u64 load = cpu_load_sync(cpu, sync);
	int nr_running;

	nr_running = rq->nr_running;

	if (sync && cpu == smp_processor_id())
		nr_running--;

	return load <= rq->mostly_idle_load &&
		nr_running <= rq->mostly_idle_nr_run &&
		!sched_cpu_high_irqload(cpu);
}

static int boost_refcount;
static DEFINE_SPINLOCK(boost_lock);
static DEFINE_MUTEX(boost_mutex);

static void boost_kick_cpus(void)
{
	int i;

	for_each_online_cpu(i) {
		if (cpu_rq(i)->capacity != max_capacity)
			boost_kick(i);
	}
}

int sched_boost(void)
{
	return boost_refcount > 0;
}

int sched_set_boost(int enable)
{
	unsigned long flags;
	int ret = 0;
	int old_refcount;

	if (!sched_enable_hmp)
		return -EINVAL;

	spin_lock_irqsave(&boost_lock, flags);

	old_refcount = boost_refcount;

	if (enable == 1) {
		boost_refcount++;
	} else if (!enable) {
		if (boost_refcount >= 1)
			boost_refcount--;
		else
			ret = -EINVAL;
	} else {
		ret = -EINVAL;
	}

	if (!old_refcount && boost_refcount)
		boost_kick_cpus();

	trace_sched_set_boost(boost_refcount);
	spin_unlock_irqrestore(&boost_lock, flags);

	return ret;
}

int sched_boost_handler(struct ctl_table *table, int write,
		void __user *buffer, size_t *lenp,
		loff_t *ppos)
{
	int ret;

	mutex_lock(&boost_mutex);
	if (!write)
		sysctl_sched_boost = sched_boost();

	ret = proc_dointvec(table, write, buffer, lenp, ppos);
	if (ret || !write)
		goto done;

	ret = (sysctl_sched_boost <= 1) ?
		sched_set_boost(sysctl_sched_boost) : -EINVAL;

done:
	mutex_unlock(&boost_mutex);
	return ret;
}

int over_schedule_budget(int cpu)
{
	struct rq *rq = cpu_rq(cpu);

	if (rq->budget == 0)
		return 1;

	if (rq->budget == 100)
		return 0;

	return (rq->load_avg > rq->budget)? 1 : 0;
}

static int task_will_fit(struct task_struct *p, int cpu)
{
	u64 load;
	int prev_cpu = task_cpu(p);
	struct rq *prev_rq = cpu_rq(prev_cpu);
	struct rq *rq = cpu_rq(cpu);
	int upmigrate = sched_upmigrate;
	int nice = TASK_NICE(p);

	if (rq->capacity == max_capacity)
		return 1;

	if (sched_boost()) {
		if (rq->capacity > prev_rq->capacity)
			return 1;
	} else {
		
		if (nice > sysctl_sched_upmigrate_min_nice)
			return 1;

		load = scale_load_to_cpu(task_load(p), cpu);

		if (prev_rq->capacity > rq->capacity)
			upmigrate = sched_downmigrate;

		if (load < upmigrate)
			return 1;
	}

	return 0;
}

static int eligible_cpu(struct task_struct *p, int cpu, int sync)
{
	struct rq *rq = cpu_rq(cpu);

	if (mostly_idle_cpu_sync(cpu, sync))
		return 1;

	if (rq->capacity != max_capacity)
		return !spill_threshold_crossed(p, rq, cpu, sync);

	return 0;
}

struct cpu_pwr_stats __weak *get_cpu_pwr_stats(void)
{
	return NULL;
}

int power_delta_exceeded(unsigned int cpu_cost, unsigned int base_cost)
{
	int delta, cost_limit;

	if (!base_cost || cpu_cost == base_cost)
		return 0;

	delta = cpu_cost - base_cost;
	cost_limit = div64_u64((u64)sysctl_sched_powerband_limit_pct *
						(u64)base_cost, 100);
	return abs(delta) > cost_limit;
}

unsigned int power_cost_at_freq(int cpu, unsigned int freq)
{
	int i = 0;
	struct cpu_pwr_stats *per_cpu_info = get_cpu_pwr_stats();
	struct cpu_pstate_pwr *costs;

	if (!per_cpu_info || !per_cpu_info[cpu].ptable ||
	    !sched_enable_power_aware)
		return cpu_rq(cpu)->max_possible_capacity;

	if (!freq)
		freq = min_max_freq;

	costs = per_cpu_info[cpu].ptable;

	while (costs[i].freq != 0) {
		if (costs[i].freq >= freq ||
		    costs[i+1].freq == 0)
			return costs[i].power;
		i++;
	}
	BUG();
}

static unsigned int power_cost(struct task_struct *p, int cpu)
{
	u64 demand;
	unsigned int task_freq;
	unsigned int cur_freq = cpu_rq(cpu)->cur_freq;

	if (!sched_enable_power_aware)
		return cpu_rq(cpu)->max_possible_capacity;

	
	demand = scale_load_to_cpu(task_load(p), cpu) * 100;
	demand = div64_u64(demand, max_task_load());

	task_freq = demand * cpu_rq(cpu)->max_possible_freq;
	task_freq /= 100; 

	task_freq = max(cur_freq, task_freq);

	return power_cost_at_freq(cpu, task_freq);
}

static int best_small_task_cpu(struct task_struct *p, int sync)
{
	int best_busy_cpu = -1, best_fallback_cpu = -1;
	int best_mi_cpu = -1;
	int min_cost_cpu = -1, min_cstate_cpu = -1;
	int min_cstate = INT_MAX;
	int min_fallback_cpu_cost = INT_MAX;
	int min_cost = INT_MAX;
	int i, cstate, cpu_cost;
	u64 load, min_busy_load = ULLONG_MAX;
	int cost_list[nr_cpu_ids];
	int prev_cpu = task_cpu(p);
	struct cpumask search_cpus;

	cpumask_and(&search_cpus,  tsk_cpus_allowed(p), cpu_online_mask);

	if (cpumask_empty(&search_cpus))
		return prev_cpu;

	for_each_cpu(i, &search_cpus) {

		trace_sched_cpu_load(cpu_rq(i), idle_cpu(i),
				     mostly_idle_cpu_sync(i, sync),
				     sched_irqload(i), power_cost(p, i),
				     cpu_temp(i));

		cpu_cost = power_cost(p, i);
		if (cpu_cost < min_cost ||
		    (cpu_cost == min_cost && i == prev_cpu)) {
			min_cost = cpu_cost;
			min_cost_cpu = i;
		}

		cost_list[i] = cpu_cost;
	}

	if (!cpu_rq(min_cost_cpu)->cstate &&
	    mostly_idle_cpu_sync(min_cost_cpu, sync) &&
	    min_cost_cpu == prev_cpu)
		return min_cost_cpu;

	for_each_cpu(i, &search_cpus) {
		struct rq *rq = cpu_rq(i);
		cstate = rq->cstate;

		if (rq->budget == 0)
			continue;

		if (over_schedule_budget(i))
			continue;

		if (power_delta_exceeded(cost_list[i], min_cost)) {
			if (cost_list[i] < min_fallback_cpu_cost ||
			    (cost_list[i] == min_fallback_cpu_cost &&
			     i == prev_cpu)) {
				best_fallback_cpu = i;
				min_fallback_cpu_cost = cost_list[i];
			}
			continue;
		}

		if (idle_cpu(i) && cstate && !sched_cpu_high_irqload(i)) {
			if (cstate < min_cstate ||
			    (cstate == min_cstate && i == prev_cpu)) {
				min_cstate_cpu = i;
				min_cstate = cstate;
			}
			continue;
		}

		if (mostly_idle_cpu_sync(i, sync)) {
			if (best_mi_cpu == -1 || i == prev_cpu)
				best_mi_cpu = i;
			continue;
		}

		load = cpu_load_sync(i, sync);
		if (!spill_threshold_crossed(p, rq, i, sync)) {
			if (load < min_busy_load ||
			    (load == min_busy_load && i == prev_cpu)) {
				min_busy_load = load;
				best_busy_cpu = i;
			}
		}
	}

	if (best_mi_cpu != -1)
		return best_mi_cpu;

	if (min_cstate_cpu != -1)
		return min_cstate_cpu;

	if (best_busy_cpu != -1)
		return best_busy_cpu;

	return best_fallback_cpu;
}

#define MOVE_TO_BIG_CPU			1
#define MOVE_TO_LITTLE_CPU		2
#define MOVE_TO_POWER_EFFICIENT_CPU	3
#define MOVE_TO_BUDGET_CAPABLE_CPU	4

static int skip_cpu(struct task_struct *p, int cpu, int reason)
{
	struct rq *rq = cpu_rq(cpu);
	struct rq *task_rq = task_rq(p);
	int skip = 0;

	if (!reason)
		return 0;

	if (is_reserved(cpu))
		return 1;

	switch (reason) {
	case MOVE_TO_BIG_CPU:
		skip = (rq->capacity <= task_rq->capacity);
		break;

	case MOVE_TO_LITTLE_CPU:
		skip = (rq->capacity >= task_rq->capacity);
		break;

	case MOVE_TO_POWER_EFFICIENT_CPU:
		skip = rq->capacity < task_rq->capacity  ||
			power_cost(p, cpu) >  power_cost(p,  task_cpu(p));
		break;
	case MOVE_TO_BUDGET_CAPABLE_CPU:
		skip = 0;
		break;
	default:
		skip = (cpu == task_cpu(p));
		break;
	}

	return skip;
}

static int select_packing_target(struct task_struct *p, int best_cpu)
{
	struct rq *rq = cpu_rq(best_cpu);
	struct cpumask search_cpus;
	int i;
	int min_cost = INT_MAX;
	int target = best_cpu;

	if (rq->cur_freq >= rq->mostly_idle_freq)
		return best_cpu;

	
	if (rq->max_freq <= rq->mostly_idle_freq)
		return best_cpu;

	cpumask_and(&search_cpus, tsk_cpus_allowed(p), cpu_online_mask);
	cpumask_and(&search_cpus, &search_cpus, &rq->freq_domain_cpumask);

	
	for_each_cpu(i, &search_cpus) {
		int cost = power_cost(p, i);

		if (cost < min_cost && !sched_cpu_high_irqload(i)) {
			target = i;
			min_cost = cost;
		}
	}

	return target;
}

static inline int wake_to_idle(struct task_struct *p)
{
	return (current->flags & PF_WAKE_UP_IDLE) ||
			 (p->flags & PF_WAKE_UP_IDLE);
}

static int select_best_cpu(struct task_struct *p, int target, int reason,
			   int sync)
{
	int i, best_cpu = -1, fallback_idle_cpu = -1, min_cstate_cpu = -1;
	int prev_cpu = task_cpu(p);
	int cpu_cost, min_cost = INT_MAX;
	int min_idle_cost = INT_MAX, min_busy_cost = INT_MAX;
	u64 load, min_load = ULLONG_MAX, min_fallback_load = ULLONG_MAX;
	int small_task = is_small_task(p);
	int boost = sched_boost();
	int cstate, min_cstate = INT_MAX;
	int prefer_idle = -1;
	int curr_cpu = smp_processor_id();
	int prefer_idle_override = 0;
	int fallback_minload_cpu = -1;
	u64 min_load_b = ULLONG_MAX;

	if (reason) {
		prefer_idle = 1;
		prefer_idle_override = 1;
	}

	if (wake_to_idle(p)) {
		prefer_idle = 1;
		prefer_idle_override = 1;
		small_task = 0;
	}

	if (small_task && !boost) {
		best_cpu = best_small_task_cpu(p, sync);
		prefer_idle = 0;	
		goto done;
	}

	
	for_each_cpu_and(i, tsk_cpus_allowed(p), cpu_online_mask) {

		trace_sched_cpu_load(cpu_rq(i), idle_cpu(i),
				     mostly_idle_cpu_sync(i, sync),
				     sched_irqload(i), power_cost(p, i),
				     cpu_temp(i));

		if (cpu_rq(i)->budget == 0)
			continue;

		if (!boost && over_schedule_budget(i))
			continue;

		if (skip_cpu(p, i, reason))
			continue;

		load = cpu_load_sync(i, sync);
		if (load < min_load_b) {
			min_load_b = load;
			fallback_minload_cpu = i;
		}

		if (!task_will_fit(p, i)) {
			if (mostly_idle_cpu_sync(i, sync)) {
				load = cpu_load_sync(i, sync);
				if (load < min_fallback_load ||
				    (load == min_fallback_load &&
				     i == prev_cpu)) {
					min_fallback_load = load;
					fallback_idle_cpu = i;
				}
			}
			continue;
		}

		
		if (prefer_idle == -1)
			prefer_idle = cpu_rq(i)->prefer_idle;

		if (!eligible_cpu(p, i, sync))
			continue;


		load = cpu_load_sync(i, sync);
		cpu_cost = power_cost(p, i);
		cstate = cpu_rq(i)->cstate;

		if (power_delta_exceeded(cpu_cost, min_cost)) {
			if (cpu_cost > min_cost)
				continue;

			min_cost = cpu_cost;
			min_load = ULLONG_MAX;
			min_cstate = INT_MAX;
			min_cstate_cpu = -1;
			best_cpu = -1;
			if (!prefer_idle_override)
				prefer_idle = cpu_rq(i)->prefer_idle;
		}

		if (idle_cpu(i) || (sync && i == curr_cpu && prefer_idle &&
				    cpu_rq(i)->nr_running == 1)) {
			if (cstate > min_cstate)
				continue;

			if (cstate < min_cstate) {
				min_idle_cost = cpu_cost;
				min_cstate = cstate;
				min_cstate_cpu = i;
				continue;
			}

			if (cpu_cost < min_idle_cost ||
			    (cpu_cost == min_idle_cost && i == prev_cpu)) {
				min_idle_cost = cpu_cost;
				min_cstate_cpu = i;
			}

			continue;
		}

		if (load > min_load)
			continue;

		if (load < min_load) {
			min_load = load;
			min_busy_cost = cpu_cost;
			best_cpu = i;
			continue;
		}

		if (cpu_cost < min_busy_cost ||
		    (cpu_cost == min_busy_cost && i == prev_cpu)) {
			min_busy_cost = cpu_cost;
			best_cpu = i;
		}
	}

	if (min_cstate_cpu >= 0 && (prefer_idle > 0 ||
		!(best_cpu >= 0 && mostly_idle_cpu_sync(best_cpu, sync))))
		best_cpu = min_cstate_cpu;
done:
	if (best_cpu < 0) {
		if (unlikely(fallback_idle_cpu < 0))
			best_cpu = prev_cpu;
		else
			best_cpu = fallback_idle_cpu;
	}

	if (cpu_rq(best_cpu)->mostly_idle_freq && !prefer_idle_override)
		best_cpu = select_packing_target(p, best_cpu);

	if (!boost && over_schedule_budget(best_cpu) && fallback_minload_cpu >= 0)
		best_cpu = fallback_minload_cpu;

	trace_sched_task_load(p, small_task, boost, reason, sync, prefer_idle);

	return best_cpu;
}

void inc_nr_big_small_task(struct rq *rq, struct task_struct *p)
{
	if (!sched_enable_hmp || sched_disable_window_stats)
		return;

	if (is_big_task(p))
		rq->nr_big_tasks++;
	else if (is_small_task(p))
		rq->nr_small_tasks++;
}

void dec_nr_big_small_task(struct rq *rq, struct task_struct *p)
{
	if (!sched_enable_hmp || sched_disable_window_stats)
		return;

	if (is_big_task(p))
		rq->nr_big_tasks--;
	else if (is_small_task(p))
		rq->nr_small_tasks--;

	BUG_ON(rq->nr_big_tasks < 0 || rq->nr_small_tasks < 0);
}

void fixup_nr_big_small_task(int cpu)
{
	struct rq *rq = cpu_rq(cpu);
	struct task_struct *p;

	rq->nr_big_tasks = 0;
	rq->nr_small_tasks = 0;
	list_for_each_entry(p, &rq->cfs_tasks, se.group_node)
		inc_nr_big_small_task(rq, p);
}

void pre_big_small_task_count_change(const struct cpumask *cpus)
{
	int i;

	local_irq_disable();

	for_each_cpu(i, cpus)
		raw_spin_lock(&cpu_rq(i)->lock);
}

void post_big_small_task_count_change(const struct cpumask *cpus)
{
	int i;

	
	for_each_cpu(i, cpus)
		fixup_nr_big_small_task(i);

	for_each_cpu(i, cpus)
		raw_spin_unlock(&cpu_rq(i)->lock);

	local_irq_enable();
}

DEFINE_MUTEX(policy_mutex);

static inline int invalid_value(unsigned int *data)
{
	int val = *data;

	if (data == &sysctl_sched_ravg_hist_size)
		return (val < 2 || val > RAVG_HIST_SIZE_MAX);

	if (data == &sysctl_sched_window_stats_policy)
		return (val >= WINDOW_STATS_INVALID_POLICY);

	return 0;
}

int sched_window_update_handler(struct ctl_table *table, int write,
		void __user *buffer, size_t *lenp,
		loff_t *ppos)
{
	int ret;
	unsigned int *data = (unsigned int *)table->data;
	unsigned int old_val;

	if (!sched_enable_hmp)
		return -EINVAL;

	mutex_lock(&policy_mutex);

	old_val = *data;

	ret = proc_dointvec(table, write, buffer, lenp, ppos);
	if (ret || !write || (write && (old_val == *data)))
		goto done;

	if (invalid_value(data)) {
		*data = old_val;
		ret = -EINVAL;
		goto done;
	}

	reset_all_window_stats(0, 0);

done:
	mutex_unlock(&policy_mutex);

	return ret;
}

int sched_hmp_proc_update_handler(struct ctl_table *table, int write,
		void __user *buffer, size_t *lenp,
		loff_t *ppos)
{
	int ret;
	unsigned int *data = (unsigned int *)table->data;
	unsigned int old_val = *data;

	ret = proc_dointvec_minmax(table, write, buffer, lenp, ppos);
	if (ret || !write || !sched_enable_hmp)
		return ret;

	if (data == &sysctl_sched_min_runtime) {
		sched_min_runtime = ((u64) sysctl_sched_min_runtime) * 1000;
		return 0;
	}
	if ((sysctl_sched_downmigrate_pct > sysctl_sched_upmigrate_pct) ||
				*data > 100) {
		*data = old_val;
		return -EINVAL;
	}

	if ((*data != old_val) &&
		(data == &sysctl_sched_upmigrate_pct ||
		data == &sysctl_sched_small_task_pct)) {
			get_online_cpus();
			pre_big_small_task_count_change(cpu_online_mask);
	}

	set_hmp_defaults();

	if ((*data != old_val) &&
		(data == &sysctl_sched_upmigrate_pct ||
		data == &sysctl_sched_small_task_pct)) {
			post_big_small_task_count_change(cpu_online_mask);
			put_online_cpus();
	}

	return 0;
}


static inline void reset_balance_interval(int cpu)
{
	struct sched_domain *sd;

	if (cpu >= nr_cpu_ids)
		return;

	rcu_read_lock();
	for_each_domain(cpu, sd)
		sd->balance_interval = 0;
	rcu_read_unlock();
}

static inline int find_new_hmp_ilb(int call_cpu, int type)
{
	int i;
	int best_cpu = nr_cpu_ids;
	struct sched_domain *sd;
	int min_cost = INT_MAX, cost;
	struct rq *src_rq = cpu_rq(call_cpu);
	struct rq *dst_rq;

	rcu_read_lock();

	
	for_each_domain(call_cpu, sd) {
		for_each_cpu(i, sched_domain_span(sd)) {
			dst_rq = cpu_rq(i);
			if (!idle_cpu(i) || (type == NOHZ_KICK_RESTRICT
				  && dst_rq->capacity > src_rq->capacity))
				continue;

			cost = power_cost_at_freq(i, min_max_freq);
			if (cost < min_cost) {
				best_cpu = i;
				min_cost = cost;
			}
		}

		if (best_cpu < nr_cpu_ids)
			break;
	}

	rcu_read_unlock();

	reset_balance_interval(best_cpu);

	return best_cpu;
}

static int lower_power_cpu_available(struct task_struct *p, int cpu)
{
	int i;
	int lowest_power_cpu = task_cpu(p);
	int lowest_power = power_cost(p, task_cpu(p));

	
	for_each_cpu_and(i, tsk_cpus_allowed(p), cpu_online_mask) {
		if (idle_cpu(i) && task_will_fit(p, i)) {
			int idle_power_cost = power_cost(p, i);
			if (idle_power_cost < lowest_power) {
				lowest_power_cpu = i;
				lowest_power = idle_power_cost;
			}
		}
	}

	return (lowest_power_cpu != task_cpu(p));
}

static inline int is_cpu_throttling_imminent(int cpu);
static inline int is_task_migration_throttled(struct task_struct *p);

static inline int migration_needed(struct rq *rq, struct task_struct *p)
{
	int nice = TASK_NICE(p);

	if (!sched_enable_hmp || p->state != TASK_RUNNING)
		return 0;

	if (over_schedule_budget(cpu_of(rq)))
		return MOVE_TO_BUDGET_CAPABLE_CPU;

	if (sched_boost()) {
		if (rq->capacity != max_capacity)
			return MOVE_TO_BIG_CPU;

		return 0;
	}

	if (is_small_task(p))
		return 0;

	
	if (nice > sysctl_sched_upmigrate_min_nice &&
		rq->capacity > min_capacity)
			return MOVE_TO_LITTLE_CPU;

	if (!task_will_fit(p, cpu_of(rq)))
		return MOVE_TO_BIG_CPU;

	if (sched_enable_power_aware &&
	    !is_task_migration_throttled(p) &&
	    is_cpu_throttling_imminent(cpu_of(rq)) &&
	    lower_power_cpu_available(p, cpu_of(rq)))
		return MOVE_TO_POWER_EFFICIENT_CPU;

	return 0;
}

static DEFINE_RAW_SPINLOCK(migration_lock);

static inline int
kick_active_balance(struct rq *rq, struct task_struct *p, int new_cpu)
{
	unsigned long flags;
	int rc = 0;

	
	raw_spin_lock_irqsave(&rq->lock, flags);
	if (!rq->active_balance) {
		rq->active_balance = 1;
		rq->push_cpu = new_cpu;
		get_task_struct(p);
		rq->push_task = p;
		rc = 1;
	}
	raw_spin_unlock_irqrestore(&rq->lock, flags);

	return rc;
}

void check_for_migration(struct rq *rq, struct task_struct *p)
{
	int cpu = cpu_of(rq), new_cpu;
	int active_balance = 0, reason;

	reason = migration_needed(rq, p);
	if (!reason)
		return;

	raw_spin_lock(&migration_lock);
	new_cpu = select_best_cpu(p, cpu, reason, 0);

	if (new_cpu != cpu) {
		active_balance = kick_active_balance(rq, p, new_cpu);
		if (active_balance)
			mark_reserved(new_cpu);
	}

	raw_spin_unlock(&migration_lock);

	if (active_balance)
		stop_one_cpu_nowait(cpu, active_load_balance_cpu_stop, rq,
					&rq->active_balance_work);
}

static inline int nr_big_tasks(struct rq *rq)
{
	return rq->nr_big_tasks;
}

static inline int is_cpu_throttling_imminent(int cpu)
{
	int throttling = 0;
	struct cpu_pwr_stats *per_cpu_info = get_cpu_pwr_stats();

	if (per_cpu_info)
		throttling = per_cpu_info[cpu].throttling;
	return throttling;
}

static inline int is_task_migration_throttled(struct task_struct *p)
{
	u64 delta = sched_clock() - p->run_start;

	return delta < sched_min_runtime;
}

unsigned int cpu_temp(int cpu)
{
	struct cpu_pwr_stats *per_cpu_info = get_cpu_pwr_stats();
	if (per_cpu_info)
		return per_cpu_info[cpu].temp;
	else
		return 0;
}


#else	

#define sched_enable_power_aware 0

static inline int task_will_fit(struct task_struct *p, int cpu)
{
	return 1;
}

static inline int select_best_cpu(struct task_struct *p, int target,
				  int reason, int sync)
{
	return 0;
}

static inline int find_new_hmp_ilb(int call_cpu, int type)
{
	return 0;
}

static inline int power_cost(struct task_struct *p, int cpu)
{
	return SCHED_POWER_SCALE;
}

static inline int
spill_threshold_crossed(struct task_struct *p, struct rq *rq, int cpu, int sync)
{
	return 0;
}

static inline int mostly_idle_cpu(int cpu)
{
	return 0;
}

static inline int sched_boost(void)
{
	return 0;
}

static inline int is_small_task(struct task_struct *p)
{
	return 0;
}

static inline int is_big_task(struct task_struct *p)
{
	return 0;
}

static inline int nr_big_tasks(struct rq *rq)
{
	return 0;
}

static inline int is_cpu_throttling_imminent(int cpu)
{
	return 0;
}

static inline int is_task_migration_throttled(struct task_struct *p)
{
	return 0;
}

unsigned int cpu_temp(int cpu)
{
	return 0;
}


#endif	

#ifdef CONFIG_SCHED_HMP

void init_new_task_load(struct task_struct *p)
{
	int i;
	u32 init_load_windows = sched_init_task_load_windows;
	u32 init_load_pelt = sched_init_task_load_pelt;
	u32 init_load_pct = current->init_load_pct;

	
	memset(&p->ravg, 0, sizeof(struct ravg));
	p->se.avg.decay_count	= 0;

	if (init_load_pct) {
		init_load_pelt = div64_u64((u64)init_load_pct *
			  (u64)LOAD_AVG_MAX, 100);
		init_load_windows = div64_u64((u64)init_load_pct *
			  (u64)sched_ravg_window, 100);
	}

	p->ravg.demand = init_load_windows;
	for (i = 0; i < RAVG_HIST_SIZE_MAX; ++i)
		p->ravg.sum_history[i] = init_load_windows;

	p->se.avg.runnable_avg_period =
		init_load_pelt ? LOAD_AVG_MAX : 0;
	p->se.avg.runnable_avg_sum = init_load_pelt;
	p->se.avg.runnable_avg_sum_scaled = init_load_pelt;
}

#else 

#if defined(CONFIG_SMP) && defined(CONFIG_FAIR_GROUP_SCHED)

void init_new_task_load(struct task_struct *p)
{
	p->se.avg.decay_count = 0;
	p->se.avg.runnable_avg_period = 0;
	p->se.avg.runnable_avg_sum = 0;
}

#else	

void init_new_task_load(struct task_struct *p)
{
}

#endif	

#endif 

static __always_inline int __update_entity_runnable_avg(int cpu, u64 now,
							struct sched_avg *sa,
							int runnable)
{
	u64 delta, periods;
	u32 runnable_contrib;
	int delta_w, decayed = 0;

	delta = now - sa->last_runnable_update;
	if ((s64)delta < 0) {
		sa->last_runnable_update = now;
		return 0;
	}

	delta >>= 10;
	if (!delta)
		return 0;
	sa->last_runnable_update = now;

	
	delta_w = sa->runnable_avg_period % 1024;
	if (delta + delta_w >= 1024) {
		
		decayed = 1;

		delta_w = 1024 - delta_w;
		if (runnable) {
			sa->runnable_avg_sum += delta_w;
			add_to_scaled_stat(cpu, sa, delta_w);
		}
		sa->runnable_avg_period += delta_w;

		delta -= delta_w;

		
		periods = delta / 1024;
		delta %= 1024;

		sa->runnable_avg_sum = decay_load(sa->runnable_avg_sum,
						  periods + 1);
		decay_scaled_stat(sa, periods + 1);
		sa->runnable_avg_period = decay_load(sa->runnable_avg_period,
						     periods + 1);

		
		runnable_contrib = __compute_runnable_contrib(periods);
		if (runnable) {
			sa->runnable_avg_sum += runnable_contrib;
			add_to_scaled_stat(cpu, sa, runnable_contrib);
		}
		sa->runnable_avg_period += runnable_contrib;
	}

	
	if (runnable) {
		sa->runnable_avg_sum += delta;
		add_to_scaled_stat(cpu, sa, delta);
	}
	sa->runnable_avg_period += delta;

	return decayed;
}

static inline u64 __synchronize_entity_decay(struct sched_entity *se)
{
	struct cfs_rq *cfs_rq = cfs_rq_of(se);
	u64 decays = atomic64_read(&cfs_rq->decay_counter);

	decays -= se->avg.decay_count;
	if (!decays) {
		se->avg.decay_count = 0;
		return 0;
	}

	se->avg.load_avg_contrib = decay_load(se->avg.load_avg_contrib, decays);
	se->avg.decay_count = 0;

	return decays;
}

#ifdef CONFIG_FAIR_GROUP_SCHED
static inline void __update_cfs_rq_tg_load_contrib(struct cfs_rq *cfs_rq,
						 int force_update)
{
	struct task_group *tg = cfs_rq->tg;
	s64 tg_contrib;

	tg_contrib = cfs_rq->runnable_load_avg + cfs_rq->blocked_load_avg;
	tg_contrib -= cfs_rq->tg_load_contrib;

	if (force_update || abs64(tg_contrib) > cfs_rq->tg_load_contrib / 8) {
		atomic64_add(tg_contrib, &tg->load_avg);
		cfs_rq->tg_load_contrib += tg_contrib;
	}
}

static inline void __update_tg_runnable_avg(struct sched_avg *sa,
						  struct cfs_rq *cfs_rq)
{
	struct task_group *tg = cfs_rq->tg;
	long contrib;

	
	contrib = div_u64(sa->runnable_avg_sum << NICE_0_SHIFT,
			  sa->runnable_avg_period + 1);
	contrib -= cfs_rq->tg_runnable_contrib;

	if (abs(contrib) > cfs_rq->tg_runnable_contrib / 64) {
		atomic_add(contrib, &tg->runnable_avg);
		cfs_rq->tg_runnable_contrib += contrib;
	}
}

static inline void __update_group_entity_contrib(struct sched_entity *se)
{
	struct cfs_rq *cfs_rq = group_cfs_rq(se);
	struct task_group *tg = cfs_rq->tg;
	int runnable_avg;

	u64 contrib;

	contrib = cfs_rq->tg_load_contrib * tg->shares;
	se->avg.load_avg_contrib = div64_u64(contrib,
					     atomic64_read(&tg->load_avg) + 1);

	runnable_avg = atomic_read(&tg->runnable_avg);
	if (runnable_avg < NICE_0_LOAD) {
		se->avg.load_avg_contrib *= runnable_avg;
		se->avg.load_avg_contrib >>= NICE_0_SHIFT;
	}
}
#else
static inline void __update_cfs_rq_tg_load_contrib(struct cfs_rq *cfs_rq,
						 int force_update) {}
static inline void __update_tg_runnable_avg(struct sched_avg *sa,
						  struct cfs_rq *cfs_rq) {}
static inline void __update_group_entity_contrib(struct sched_entity *se) {}
#endif

static inline void __update_task_entity_contrib(struct sched_entity *se)
{
	u32 contrib;

	
	contrib = se->avg.runnable_avg_sum * scale_load_down(se->load.weight);
	contrib /= (se->avg.runnable_avg_period + 1);
	se->avg.load_avg_contrib = scale_load(contrib);
}

static long __update_entity_load_avg_contrib(struct sched_entity *se)
{
	long old_contrib = se->avg.load_avg_contrib;

	if (entity_is_task(se)) {
		__update_task_entity_contrib(se);
	} else {
		__update_tg_runnable_avg(&se->avg, group_cfs_rq(se));
		__update_group_entity_contrib(se);
	}

	return se->avg.load_avg_contrib - old_contrib;
}

static inline void subtract_blocked_load_contrib(struct cfs_rq *cfs_rq,
						 long load_contrib)
{
	if (likely(load_contrib < cfs_rq->blocked_load_avg))
		cfs_rq->blocked_load_avg -= load_contrib;
	else
		cfs_rq->blocked_load_avg = 0;
}

static inline u64 cfs_rq_clock_task(struct cfs_rq *cfs_rq);

static inline void update_entity_load_avg(struct sched_entity *se,
					  int update_cfs_rq)
{
	struct cfs_rq *cfs_rq = cfs_rq_of(se);
	long contrib_delta;
	u64 now;
	int cpu = cpu_of(rq_of(cfs_rq));
	int decayed;

	if (entity_is_task(se)) {
		now = cfs_rq_clock_task(cfs_rq);
		if (se->on_rq) {
			dec_cumulative_runnable_avg(rq_of(cfs_rq), task_of(se));
			dec_nr_big_small_task(rq_of(cfs_rq), task_of(se));
		}
	} else
		now = cfs_rq_clock_task(group_cfs_rq(se));

	decayed = __update_entity_runnable_avg(cpu, now, &se->avg, se->on_rq);
	if (entity_is_task(se) && se->on_rq) {
		inc_cumulative_runnable_avg(rq_of(cfs_rq), task_of(se));
		inc_nr_big_small_task(rq_of(cfs_rq), task_of(se));
	}

	if (!decayed)
		return;

	contrib_delta = __update_entity_load_avg_contrib(se);

	if (!update_cfs_rq)
		return;

	if (se->on_rq)
		cfs_rq->runnable_load_avg += contrib_delta;
	else
		subtract_blocked_load_contrib(cfs_rq, -contrib_delta);
}

static void update_cfs_rq_blocked_load(struct cfs_rq *cfs_rq, int force_update)
{
	u64 now = cfs_rq_clock_task(cfs_rq) >> 20;
	u64 decays;

	decays = now - cfs_rq->last_decay;
	if (!decays && !force_update)
		return;

	if (atomic64_read(&cfs_rq->removed_load)) {
		u64 removed_load = atomic64_xchg(&cfs_rq->removed_load, 0);
		subtract_blocked_load_contrib(cfs_rq, removed_load);
	}

	if (decays) {
		cfs_rq->blocked_load_avg = decay_load(cfs_rq->blocked_load_avg,
						      decays);
		atomic64_add(decays, &cfs_rq->decay_counter);
		cfs_rq->last_decay = now;
	}

	__update_cfs_rq_tg_load_contrib(cfs_rq, force_update);
}

static inline void update_rq_runnable_avg(struct rq *rq, int runnable)
{
	__update_entity_runnable_avg(cpu_of(rq), rq->clock_task,
						 &rq->avg, runnable);
	__update_tg_runnable_avg(&rq->avg, &rq->cfs);
}

static inline void enqueue_entity_load_avg(struct cfs_rq *cfs_rq,
						  struct sched_entity *se,
						  int wakeup)
{
	if (unlikely(se->avg.decay_count <= 0)) {
		se->avg.last_runnable_update = rq_of(cfs_rq)->clock_task;
		if (se->avg.decay_count) {
			se->avg.last_runnable_update -= (-se->avg.decay_count)
							<< 20;
			update_entity_load_avg(se, 0);
			
			se->avg.decay_count = 0;
		}
		wakeup = 0;
	} else {
		__synchronize_entity_decay(se);
	}

	
	if (wakeup) {
		subtract_blocked_load_contrib(cfs_rq, se->avg.load_avg_contrib);
		update_entity_load_avg(se, 0);
	}

	cfs_rq->runnable_load_avg += se->avg.load_avg_contrib;
	
	update_cfs_rq_blocked_load(cfs_rq, !wakeup);
}

static inline void dequeue_entity_load_avg(struct cfs_rq *cfs_rq,
						  struct sched_entity *se,
						  int sleep)
{
	update_entity_load_avg(se, 1);
	
	update_cfs_rq_blocked_load(cfs_rq, !sleep);

	cfs_rq->runnable_load_avg -= se->avg.load_avg_contrib;
	if (sleep) {
		cfs_rq->blocked_load_avg += se->avg.load_avg_contrib;
		se->avg.decay_count = atomic64_read(&cfs_rq->decay_counter);
	} 
}

void idle_enter_fair(struct rq *this_rq)
{
	update_rq_runnable_avg(this_rq, 1);
}

void idle_exit_fair(struct rq *this_rq)
{
	update_rq_runnable_avg(this_rq, 0);
}

#else
static inline void update_entity_load_avg(struct sched_entity *se,
					  int update_cfs_rq) {}
static inline void update_rq_runnable_avg(struct rq *rq, int runnable) {}
static inline void enqueue_entity_load_avg(struct cfs_rq *cfs_rq,
					   struct sched_entity *se,
					   int wakeup) {}
static inline void dequeue_entity_load_avg(struct cfs_rq *cfs_rq,
					   struct sched_entity *se,
					   int sleep) {}
static inline void update_cfs_rq_blocked_load(struct cfs_rq *cfs_rq,
					      int force_update) {}
#endif

#ifdef CONFIG_SCHED_HMP

unsigned int pct_task_load(struct task_struct *p)
{
	unsigned int load;

	load = div64_u64((u64)task_load(p) * 100, (u64)max_task_load());

	return load;
}

static inline void
add_to_scaled_stat(int cpu, struct sched_avg *sa, u64 delta)
{
	struct rq *rq = cpu_rq(cpu);
	int cur_freq = rq->cur_freq, max_freq = rq->max_freq;
	int cpu_max_possible_freq = rq->max_possible_freq;
	u64 scaled_delta;
	int sf;

	if (!sched_enable_hmp)
		return;

	if (unlikely(cur_freq > max_possible_freq ||
		     (cur_freq == max_freq &&
		      max_freq < cpu_max_possible_freq)))
		cur_freq = max_possible_freq;

	scaled_delta = div64_u64(delta * cur_freq, max_possible_freq);
	sf = (rq->efficiency * 1024) / max_possible_efficiency;
	scaled_delta *= sf;
	scaled_delta >>= 10;
	sa->runnable_avg_sum_scaled += scaled_delta;
}

static inline void decay_scaled_stat(struct sched_avg *sa, u64 periods)
{
	if (!sched_enable_hmp)
		return;

	sa->runnable_avg_sum_scaled =
		decay_load(sa->runnable_avg_sum_scaled,
			   periods);
}

#else  

static inline void
add_to_scaled_stat(int cpu, struct sched_avg *sa, u64 delta)
{
}

static inline void decay_scaled_stat(struct sched_avg *sa, u64 periods)
{
}

#endif 

static void enqueue_sleeper(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
#ifdef CONFIG_SCHEDSTATS
	struct task_struct *tsk = NULL;

	if (entity_is_task(se))
		tsk = task_of(se);

	if (se->statistics.sleep_start) {
		u64 delta = rq_of(cfs_rq)->clock - se->statistics.sleep_start;

		if ((s64)delta < 0)
			delta = 0;

		if (unlikely(delta > se->statistics.sleep_max))
			se->statistics.sleep_max = delta;

		se->statistics.sleep_start = 0;
		se->statistics.sum_sleep_runtime += delta;

		if (tsk) {
			account_scheduler_latency(tsk, delta >> 10, 1);
			trace_sched_stat_sleep(tsk, delta);
		}
	}
	if (se->statistics.block_start) {
		u64 delta = rq_of(cfs_rq)->clock - se->statistics.block_start;

		if ((s64)delta < 0)
			delta = 0;

		if (unlikely(delta > se->statistics.block_max))
			se->statistics.block_max = delta;

		se->statistics.block_start = 0;
		se->statistics.sum_sleep_runtime += delta;

		if (tsk) {
			if (tsk->in_iowait) {
				se->statistics.iowait_sum += delta;
				se->statistics.iowait_count++;
				trace_sched_stat_iowait(tsk, delta);
			}

			trace_sched_stat_blocked(tsk, delta);

			if (unlikely(prof_on == SLEEP_PROFILING)) {
				profile_hits(SLEEP_PROFILING,
						(void *)get_wchan(tsk),
						delta >> 20);
			}
			account_scheduler_latency(tsk, delta >> 10, 0);
		}
	}
#endif
}

static void check_spread(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
#ifdef CONFIG_SCHED_DEBUG
	s64 d = se->vruntime - cfs_rq->min_vruntime;

	if (d < 0)
		d = -d;

	if (d > 3*sysctl_sched_latency)
		schedstat_inc(cfs_rq, nr_spread_over);
#endif
}

static void
place_entity(struct cfs_rq *cfs_rq, struct sched_entity *se, int initial)
{
	u64 vruntime = cfs_rq->min_vruntime;

	if (initial && sched_feat(START_DEBIT))
		vruntime += sched_vslice(cfs_rq, se);

	
	if (!initial) {
		unsigned long thresh = sysctl_sched_latency;

		if (sched_feat(GENTLE_FAIR_SLEEPERS))
			thresh >>= 1;

		vruntime -= thresh;
	}

	
	se->vruntime = max_vruntime(se->vruntime, vruntime);
}

static void check_enqueue_throttle(struct cfs_rq *cfs_rq);

static void
enqueue_entity(struct cfs_rq *cfs_rq, struct sched_entity *se, int flags)
{
	if (!(flags & ENQUEUE_WAKEUP) || (flags & ENQUEUE_WAKING))
		se->vruntime += cfs_rq->min_vruntime;

	update_curr(cfs_rq);
	enqueue_entity_load_avg(cfs_rq, se, flags & ENQUEUE_WAKEUP);
	account_entity_enqueue(cfs_rq, se);
	update_cfs_shares(cfs_rq);

	if (flags & ENQUEUE_WAKEUP) {
		place_entity(cfs_rq, se, 0);
		enqueue_sleeper(cfs_rq, se);
	}

	update_stats_enqueue(cfs_rq, se);
	check_spread(cfs_rq, se);
	if (se != cfs_rq->curr)
		__enqueue_entity(cfs_rq, se);
	se->on_rq = 1;

	if (cfs_rq->nr_running == 1) {
		list_add_leaf_cfs_rq(cfs_rq);
		check_enqueue_throttle(cfs_rq);
	}
}

static void __clear_buddies_last(struct sched_entity *se)
{
	for_each_sched_entity(se) {
		struct cfs_rq *cfs_rq = cfs_rq_of(se);
		if (cfs_rq->last == se)
			cfs_rq->last = NULL;
		else
			break;
	}
}

static void __clear_buddies_next(struct sched_entity *se)
{
	for_each_sched_entity(se) {
		struct cfs_rq *cfs_rq = cfs_rq_of(se);
		if (cfs_rq->next == se)
			cfs_rq->next = NULL;
		else
			break;
	}
}

static void __clear_buddies_skip(struct sched_entity *se)
{
	for_each_sched_entity(se) {
		struct cfs_rq *cfs_rq = cfs_rq_of(se);
		if (cfs_rq->skip == se)
			cfs_rq->skip = NULL;
		else
			break;
	}
}

static void clear_buddies(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	if (cfs_rq->last == se)
		__clear_buddies_last(se);

	if (cfs_rq->next == se)
		__clear_buddies_next(se);

	if (cfs_rq->skip == se)
		__clear_buddies_skip(se);
}

static __always_inline void return_cfs_rq_runtime(struct cfs_rq *cfs_rq);

static void
dequeue_entity(struct cfs_rq *cfs_rq, struct sched_entity *se, int flags)
{
	update_curr(cfs_rq);
	dequeue_entity_load_avg(cfs_rq, se, flags & DEQUEUE_SLEEP);

	update_stats_dequeue(cfs_rq, se);
	if (flags & DEQUEUE_SLEEP) {
#ifdef CONFIG_SCHEDSTATS
		if (entity_is_task(se)) {
			struct task_struct *tsk = task_of(se);

			if (tsk->state & TASK_INTERRUPTIBLE)
				se->statistics.sleep_start = rq_of(cfs_rq)->clock;
			if (tsk->state & TASK_UNINTERRUPTIBLE)
				se->statistics.block_start = rq_of(cfs_rq)->clock;
		}
#endif
	}

	clear_buddies(cfs_rq, se);

	if (se != cfs_rq->curr)
		__dequeue_entity(cfs_rq, se);
	se->on_rq = 0;
	account_entity_dequeue(cfs_rq, se);

	if (!(flags & DEQUEUE_SLEEP))
		se->vruntime -= cfs_rq->min_vruntime;

	
	return_cfs_rq_runtime(cfs_rq);

	update_min_vruntime(cfs_rq);
	update_cfs_shares(cfs_rq);
}

static void
check_preempt_tick(struct cfs_rq *cfs_rq, struct sched_entity *curr)
{
	unsigned long ideal_runtime, delta_exec;
	struct sched_entity *se;
	s64 delta;

	ideal_runtime = sched_slice(cfs_rq, curr);
	delta_exec = curr->sum_exec_runtime - curr->prev_sum_exec_runtime;
	if (delta_exec > ideal_runtime) {
		resched_task(rq_of(cfs_rq)->curr);
		clear_buddies(cfs_rq, curr);
		return;
	}

	if (delta_exec < sysctl_sched_min_granularity)
		return;

	se = __pick_first_entity(cfs_rq);
	delta = curr->vruntime - se->vruntime;

	if (delta < 0)
		return;

	if (delta > ideal_runtime)
		resched_task(rq_of(cfs_rq)->curr);
}

static void
set_next_entity(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	
	if (se->on_rq) {
		update_stats_wait_end(cfs_rq, se);
		__dequeue_entity(cfs_rq, se);
	}

	update_stats_curr_start(cfs_rq, se);
	cfs_rq->curr = se;
#ifdef CONFIG_SCHEDSTATS
	if (rq_of(cfs_rq)->load.weight >= 2*se->load.weight) {
		se->statistics.slice_max = max(se->statistics.slice_max,
			se->sum_exec_runtime - se->prev_sum_exec_runtime);
	}
#endif
	se->prev_sum_exec_runtime = se->sum_exec_runtime;
}

static int
wakeup_preempt_entity(struct sched_entity *curr, struct sched_entity *se);

static struct sched_entity *pick_next_entity(struct cfs_rq *cfs_rq)
{
	struct sched_entity *se = __pick_first_entity(cfs_rq);
	struct sched_entity *left = se;

	if (cfs_rq->skip == se) {
		struct sched_entity *second = __pick_next_entity(se);
		if (second && wakeup_preempt_entity(second, left) < 1)
			se = second;
	}

	if (cfs_rq->last && wakeup_preempt_entity(cfs_rq->last, left) < 1)
		se = cfs_rq->last;

	if (cfs_rq->next && wakeup_preempt_entity(cfs_rq->next, left) < 1)
		se = cfs_rq->next;

	clear_buddies(cfs_rq, se);

	return se;
}

static void check_cfs_rq_runtime(struct cfs_rq *cfs_rq);

static void put_prev_entity(struct cfs_rq *cfs_rq, struct sched_entity *prev)
{
	if (prev->on_rq)
		update_curr(cfs_rq);

	
	check_cfs_rq_runtime(cfs_rq);

	check_spread(cfs_rq, prev);
	if (prev->on_rq) {
		update_stats_wait_start(cfs_rq, prev);
		
		__enqueue_entity(cfs_rq, prev);
		
		update_entity_load_avg(prev, 1);
	}
	cfs_rq->curr = NULL;
}

static void
entity_tick(struct cfs_rq *cfs_rq, struct sched_entity *curr, int queued)
{
	update_curr(cfs_rq);

	update_entity_load_avg(curr, 1);
	update_cfs_rq_blocked_load(cfs_rq, 1);
	update_cfs_shares(cfs_rq);

#ifdef CONFIG_SCHED_HRTICK
	if (queued) {
		resched_task(rq_of(cfs_rq)->curr);
		return;
	}
	if (!sched_feat(DOUBLE_TICK) &&
			hrtimer_active(&rq_of(cfs_rq)->hrtick_timer))
		return;
#endif

	if (cfs_rq->nr_running > 1)
		check_preempt_tick(cfs_rq, curr);
}



#ifdef CONFIG_CFS_BANDWIDTH

#ifdef HAVE_JUMP_LABEL
static struct static_key __cfs_bandwidth_used;

static inline bool cfs_bandwidth_used(void)
{
	return static_key_false(&__cfs_bandwidth_used);
}

void cfs_bandwidth_usage_inc(void)
{
	static_key_slow_inc(&__cfs_bandwidth_used);
}

void cfs_bandwidth_usage_dec(void)
{
	static_key_slow_dec(&__cfs_bandwidth_used);
}
#else 
static bool cfs_bandwidth_used(void)
{
	return true;
}

void cfs_bandwidth_usage_inc(void) {}
void cfs_bandwidth_usage_dec(void) {}
#endif 

static inline u64 default_cfs_period(void)
{
	return 100000000ULL;
}

static inline u64 sched_cfs_bandwidth_slice(void)
{
	return (u64)sysctl_sched_cfs_bandwidth_slice * NSEC_PER_USEC;
}

void __refill_cfs_bandwidth_runtime(struct cfs_bandwidth *cfs_b)
{
	u64 now;

	if (cfs_b->quota == RUNTIME_INF)
		return;

	now = sched_clock_cpu(smp_processor_id());
	cfs_b->runtime = cfs_b->quota;
	cfs_b->runtime_expires = now + ktime_to_ns(cfs_b->period);
}

static inline struct cfs_bandwidth *tg_cfs_bandwidth(struct task_group *tg)
{
	return &tg->cfs_bandwidth;
}

static inline u64 cfs_rq_clock_task(struct cfs_rq *cfs_rq)
{
	if (unlikely(cfs_rq->throttle_count))
		return cfs_rq->throttled_clock_task;

	return rq_of(cfs_rq)->clock_task - cfs_rq->throttled_clock_task_time;
}

static int assign_cfs_rq_runtime(struct cfs_rq *cfs_rq)
{
	struct task_group *tg = cfs_rq->tg;
	struct cfs_bandwidth *cfs_b = tg_cfs_bandwidth(tg);
	u64 amount = 0, min_amount, expires;

	
	min_amount = sched_cfs_bandwidth_slice() - cfs_rq->runtime_remaining;

	raw_spin_lock(&cfs_b->lock);
	if (cfs_b->quota == RUNTIME_INF)
		amount = min_amount;
	else {
		if (!cfs_b->timer_active) {
			__refill_cfs_bandwidth_runtime(cfs_b);
			__start_cfs_bandwidth(cfs_b);
		}

		if (cfs_b->runtime > 0) {
			amount = min(cfs_b->runtime, min_amount);
			cfs_b->runtime -= amount;
			cfs_b->idle = 0;
		}
	}
	expires = cfs_b->runtime_expires;
	raw_spin_unlock(&cfs_b->lock);

	cfs_rq->runtime_remaining += amount;
	if ((s64)(expires - cfs_rq->runtime_expires) > 0)
		cfs_rq->runtime_expires = expires;

	return cfs_rq->runtime_remaining > 0;
}

static void expire_cfs_rq_runtime(struct cfs_rq *cfs_rq)
{
	struct cfs_bandwidth *cfs_b = tg_cfs_bandwidth(cfs_rq->tg);
	struct rq *rq = rq_of(cfs_rq);

	
	if (likely((s64)(rq->clock - cfs_rq->runtime_expires) < 0))
		return;

	if (cfs_rq->runtime_remaining < 0)
		return;


	if ((s64)(cfs_rq->runtime_expires - cfs_b->runtime_expires) >= 0) {
		
		cfs_rq->runtime_expires += TICK_NSEC;
	} else {
		
		cfs_rq->runtime_remaining = 0;
	}
}

static void __account_cfs_rq_runtime(struct cfs_rq *cfs_rq,
				     unsigned long delta_exec)
{
	
	cfs_rq->runtime_remaining -= delta_exec;
	expire_cfs_rq_runtime(cfs_rq);

	if (likely(cfs_rq->runtime_remaining > 0))
		return;

	if (!assign_cfs_rq_runtime(cfs_rq) && likely(cfs_rq->curr))
		resched_task(rq_of(cfs_rq)->curr);
}

static __always_inline
void account_cfs_rq_runtime(struct cfs_rq *cfs_rq, unsigned long delta_exec)
{
	if (!cfs_bandwidth_used() || !cfs_rq->runtime_enabled)
		return;

	__account_cfs_rq_runtime(cfs_rq, delta_exec);
}

static inline int cfs_rq_throttled(struct cfs_rq *cfs_rq)
{
	return cfs_bandwidth_used() && cfs_rq->throttled;
}

static inline int throttled_hierarchy(struct cfs_rq *cfs_rq)
{
	return cfs_bandwidth_used() && cfs_rq->throttle_count;
}

static inline int throttled_lb_pair(struct task_group *tg,
				    int src_cpu, int dest_cpu)
{
	struct cfs_rq *src_cfs_rq, *dest_cfs_rq;

	src_cfs_rq = tg->cfs_rq[src_cpu];
	dest_cfs_rq = tg->cfs_rq[dest_cpu];

	return throttled_hierarchy(src_cfs_rq) ||
	       throttled_hierarchy(dest_cfs_rq);
}

static int tg_unthrottle_up(struct task_group *tg, void *data)
{
	struct rq *rq = data;
	struct cfs_rq *cfs_rq = tg->cfs_rq[cpu_of(rq)];

	cfs_rq->throttle_count--;
#ifdef CONFIG_SMP
	if (!cfs_rq->throttle_count) {
		
		cfs_rq->throttled_clock_task_time += rq->clock_task -
					     cfs_rq->throttled_clock_task;
	}
#endif

	return 0;
}

static int tg_throttle_down(struct task_group *tg, void *data)
{
	struct rq *rq = data;
	struct cfs_rq *cfs_rq = tg->cfs_rq[cpu_of(rq)];

	
	if (!cfs_rq->throttle_count)
		cfs_rq->throttled_clock_task = rq->clock_task;
	cfs_rq->throttle_count++;

	return 0;
}

static void throttle_cfs_rq(struct cfs_rq *cfs_rq)
{
	struct rq *rq = rq_of(cfs_rq);
	struct cfs_bandwidth *cfs_b = tg_cfs_bandwidth(cfs_rq->tg);
	struct sched_entity *se;
	long task_delta, dequeue = 1;

	se = cfs_rq->tg->se[cpu_of(rq_of(cfs_rq))];

	
	rcu_read_lock();
	walk_tg_tree_from(cfs_rq->tg, tg_throttle_down, tg_nop, (void *)rq);
	rcu_read_unlock();

	task_delta = cfs_rq->h_nr_running;
	for_each_sched_entity(se) {
		struct cfs_rq *qcfs_rq = cfs_rq_of(se);
		
		if (!se->on_rq)
			break;

		if (dequeue)
			dequeue_entity(qcfs_rq, se, DEQUEUE_SLEEP);
		qcfs_rq->h_nr_running -= task_delta;

		if (qcfs_rq->load.weight)
			dequeue = 0;
	}

	if (!se)
		rq->nr_running -= task_delta;

	cfs_rq->throttled = 1;
	cfs_rq->throttled_clock = rq->clock;
	raw_spin_lock(&cfs_b->lock);
	list_add_tail_rcu(&cfs_rq->throttled_list, &cfs_b->throttled_cfs_rq);
	if (!cfs_b->timer_active)
		__start_cfs_bandwidth(cfs_b);
	raw_spin_unlock(&cfs_b->lock);
}

void unthrottle_cfs_rq(struct cfs_rq *cfs_rq)
{
	struct rq *rq = rq_of(cfs_rq);
	struct cfs_bandwidth *cfs_b = tg_cfs_bandwidth(cfs_rq->tg);
	struct sched_entity *se;
	int enqueue = 1;
	long task_delta;

	se = cfs_rq->tg->se[cpu_of(rq_of(cfs_rq))];

	cfs_rq->throttled = 0;
	raw_spin_lock(&cfs_b->lock);
	cfs_b->throttled_time += rq->clock - cfs_rq->throttled_clock;
	list_del_rcu(&cfs_rq->throttled_list);
	raw_spin_unlock(&cfs_b->lock);

	update_rq_clock(rq);
	
	walk_tg_tree_from(cfs_rq->tg, tg_nop, tg_unthrottle_up, (void *)rq);

	if (!cfs_rq->load.weight)
		return;

	task_delta = cfs_rq->h_nr_running;
	for_each_sched_entity(se) {
		if (se->on_rq)
			enqueue = 0;

		cfs_rq = cfs_rq_of(se);
		if (enqueue)
			enqueue_entity(cfs_rq, se, ENQUEUE_WAKEUP);
		cfs_rq->h_nr_running += task_delta;

		if (cfs_rq_throttled(cfs_rq))
			break;
	}

	if (!se)
		rq->nr_running += task_delta;

	
	if (rq->curr == rq->idle && rq->cfs.nr_running)
		resched_task(rq->curr);
}

static u64 distribute_cfs_runtime(struct cfs_bandwidth *cfs_b,
		u64 remaining, u64 expires)
{
	struct cfs_rq *cfs_rq;
	u64 runtime = remaining;

	rcu_read_lock();
	list_for_each_entry_rcu(cfs_rq, &cfs_b->throttled_cfs_rq,
				throttled_list) {
		struct rq *rq = rq_of(cfs_rq);

		raw_spin_lock(&rq->lock);
		if (!cfs_rq_throttled(cfs_rq))
			goto next;

		runtime = -cfs_rq->runtime_remaining + 1;
		if (runtime > remaining)
			runtime = remaining;
		remaining -= runtime;

		cfs_rq->runtime_remaining += runtime;
		cfs_rq->runtime_expires = expires;

		
		if (cfs_rq->runtime_remaining > 0)
			unthrottle_cfs_rq(cfs_rq);

next:
		raw_spin_unlock(&rq->lock);

		if (!remaining)
			break;
	}
	rcu_read_unlock();

	return remaining;
}

static int do_sched_cfs_period_timer(struct cfs_bandwidth *cfs_b, int overrun)
{
	u64 runtime, runtime_expires;
	int idle = 1, throttled;

	raw_spin_lock(&cfs_b->lock);
	
	if (cfs_b->quota == RUNTIME_INF)
		goto out_unlock;

	throttled = !list_empty(&cfs_b->throttled_cfs_rq);
	
	idle = cfs_b->idle && !throttled;
	cfs_b->nr_periods += overrun;

	
	if (idle)
		goto out_unlock;

	cfs_b->timer_active = 1;

	__refill_cfs_bandwidth_runtime(cfs_b);

	if (!throttled) {
		
		cfs_b->idle = 1;
		goto out_unlock;
	}

	
	cfs_b->nr_throttled += overrun;

	runtime = cfs_b->runtime;
	runtime_expires = cfs_b->runtime_expires;
	cfs_b->runtime = 0;

	while (throttled && runtime > 0) {
		raw_spin_unlock(&cfs_b->lock);
		
		runtime = distribute_cfs_runtime(cfs_b, runtime,
						 runtime_expires);
		raw_spin_lock(&cfs_b->lock);

		throttled = !list_empty(&cfs_b->throttled_cfs_rq);
	}

	
	cfs_b->runtime = runtime;
	cfs_b->idle = 0;
out_unlock:
	if (idle)
		cfs_b->timer_active = 0;
	raw_spin_unlock(&cfs_b->lock);

	return idle;
}

static const u64 min_cfs_rq_runtime = 1 * NSEC_PER_MSEC;
static const u64 min_bandwidth_expiration = 2 * NSEC_PER_MSEC;
static const u64 cfs_bandwidth_slack_period = 5 * NSEC_PER_MSEC;

static int runtime_refresh_within(struct cfs_bandwidth *cfs_b, u64 min_expire)
{
	struct hrtimer *refresh_timer = &cfs_b->period_timer;
	u64 remaining;

	
	if (hrtimer_callback_running(refresh_timer))
		return 1;

	
	remaining = ktime_to_ns(hrtimer_expires_remaining(refresh_timer));
	if (remaining < min_expire)
		return 1;

	return 0;
}

static void start_cfs_slack_bandwidth(struct cfs_bandwidth *cfs_b)
{
	u64 min_left = cfs_bandwidth_slack_period + min_bandwidth_expiration;

	
	if (runtime_refresh_within(cfs_b, min_left))
		return;

	start_bandwidth_timer(&cfs_b->slack_timer,
				ns_to_ktime(cfs_bandwidth_slack_period));
}

static void __return_cfs_rq_runtime(struct cfs_rq *cfs_rq)
{
	struct cfs_bandwidth *cfs_b = tg_cfs_bandwidth(cfs_rq->tg);
	s64 slack_runtime = cfs_rq->runtime_remaining - min_cfs_rq_runtime;

	if (slack_runtime <= 0)
		return;

	raw_spin_lock(&cfs_b->lock);
	if (cfs_b->quota != RUNTIME_INF &&
	    cfs_rq->runtime_expires == cfs_b->runtime_expires) {
		cfs_b->runtime += slack_runtime;

		
		if (cfs_b->runtime > sched_cfs_bandwidth_slice() &&
		    !list_empty(&cfs_b->throttled_cfs_rq))
			start_cfs_slack_bandwidth(cfs_b);
	}
	raw_spin_unlock(&cfs_b->lock);

	
	cfs_rq->runtime_remaining -= slack_runtime;
}

static __always_inline void return_cfs_rq_runtime(struct cfs_rq *cfs_rq)
{
	if (!cfs_bandwidth_used())
		return;

	if (!cfs_rq->runtime_enabled || cfs_rq->nr_running)
		return;

	__return_cfs_rq_runtime(cfs_rq);
}

static void do_sched_cfs_slack_timer(struct cfs_bandwidth *cfs_b)
{
	u64 runtime = 0, slice = sched_cfs_bandwidth_slice();
	u64 expires;

	
	raw_spin_lock(&cfs_b->lock);
	if (runtime_refresh_within(cfs_b, min_bandwidth_expiration)) {
		raw_spin_unlock(&cfs_b->lock);
		return;
	}

	if (cfs_b->quota != RUNTIME_INF && cfs_b->runtime > slice) {
		runtime = cfs_b->runtime;
		cfs_b->runtime = 0;
	}
	expires = cfs_b->runtime_expires;
	raw_spin_unlock(&cfs_b->lock);

	if (!runtime)
		return;

	runtime = distribute_cfs_runtime(cfs_b, runtime, expires);

	raw_spin_lock(&cfs_b->lock);
	if (expires == cfs_b->runtime_expires)
		cfs_b->runtime = runtime;
	raw_spin_unlock(&cfs_b->lock);
}

static void check_enqueue_throttle(struct cfs_rq *cfs_rq)
{
	if (!cfs_bandwidth_used())
		return;

	
	if (!cfs_rq->runtime_enabled || cfs_rq->curr)
		return;

	
	if (cfs_rq_throttled(cfs_rq))
		return;

	
	account_cfs_rq_runtime(cfs_rq, 0);
	if (cfs_rq->runtime_remaining <= 0)
		throttle_cfs_rq(cfs_rq);
}

static void check_cfs_rq_runtime(struct cfs_rq *cfs_rq)
{
	if (!cfs_bandwidth_used())
		return;

	if (likely(!cfs_rq->runtime_enabled || cfs_rq->runtime_remaining > 0))
		return;

	if (cfs_rq_throttled(cfs_rq))
		return;

	throttle_cfs_rq(cfs_rq);
}

static inline u64 default_cfs_period(void);
static int do_sched_cfs_period_timer(struct cfs_bandwidth *cfs_b, int overrun);
static void do_sched_cfs_slack_timer(struct cfs_bandwidth *cfs_b);

static enum hrtimer_restart sched_cfs_slack_timer(struct hrtimer *timer)
{
	struct cfs_bandwidth *cfs_b =
		container_of(timer, struct cfs_bandwidth, slack_timer);
	do_sched_cfs_slack_timer(cfs_b);

	return HRTIMER_NORESTART;
}

static enum hrtimer_restart sched_cfs_period_timer(struct hrtimer *timer)
{
	struct cfs_bandwidth *cfs_b =
		container_of(timer, struct cfs_bandwidth, period_timer);
	ktime_t now;
	int overrun;
	int idle = 0;

	for (;;) {
		now = hrtimer_cb_get_time(timer);
		overrun = hrtimer_forward(timer, now, cfs_b->period);

		if (!overrun)
			break;

		idle = do_sched_cfs_period_timer(cfs_b, overrun);
	}

	return idle ? HRTIMER_NORESTART : HRTIMER_RESTART;
}

void init_cfs_bandwidth(struct cfs_bandwidth *cfs_b)
{
	raw_spin_lock_init(&cfs_b->lock);
	cfs_b->runtime = 0;
	cfs_b->quota = RUNTIME_INF;
	cfs_b->period = ns_to_ktime(default_cfs_period());

	INIT_LIST_HEAD(&cfs_b->throttled_cfs_rq);
	hrtimer_init(&cfs_b->period_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	cfs_b->period_timer.function = sched_cfs_period_timer;
	hrtimer_init(&cfs_b->slack_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	cfs_b->slack_timer.function = sched_cfs_slack_timer;
}

static void init_cfs_rq_runtime(struct cfs_rq *cfs_rq)
{
	cfs_rq->runtime_enabled = 0;
	INIT_LIST_HEAD(&cfs_rq->throttled_list);
}

void __start_cfs_bandwidth(struct cfs_bandwidth *cfs_b)
{
	while (unlikely(hrtimer_active(&cfs_b->period_timer)) &&
	       hrtimer_try_to_cancel(&cfs_b->period_timer) < 0) {
		
		raw_spin_unlock(&cfs_b->lock);
		cpu_relax();
		raw_spin_lock(&cfs_b->lock);
		
		if (cfs_b->timer_active)
			return;
	}

	cfs_b->timer_active = 1;
	start_bandwidth_timer(&cfs_b->period_timer, cfs_b->period);
}

static void destroy_cfs_bandwidth(struct cfs_bandwidth *cfs_b)
{
	hrtimer_cancel(&cfs_b->period_timer);
	hrtimer_cancel(&cfs_b->slack_timer);
}

static void __maybe_unused unthrottle_offline_cfs_rqs(struct rq *rq)
{
	struct cfs_rq *cfs_rq;

	for_each_leaf_cfs_rq(rq, cfs_rq) {
		struct cfs_bandwidth *cfs_b = tg_cfs_bandwidth(cfs_rq->tg);

		if (!cfs_rq->runtime_enabled)
			continue;

		cfs_rq->runtime_remaining = cfs_b->quota;
		if (cfs_rq_throttled(cfs_rq))
			unthrottle_cfs_rq(cfs_rq);
	}
}

#else 
static inline u64 cfs_rq_clock_task(struct cfs_rq *cfs_rq)
{
	return rq_of(cfs_rq)->clock_task;
}

static void account_cfs_rq_runtime(struct cfs_rq *cfs_rq,
				     unsigned long delta_exec) {}
static void check_cfs_rq_runtime(struct cfs_rq *cfs_rq) {}
static void check_enqueue_throttle(struct cfs_rq *cfs_rq) {}
static __always_inline void return_cfs_rq_runtime(struct cfs_rq *cfs_rq) {}

static inline int cfs_rq_throttled(struct cfs_rq *cfs_rq)
{
	return 0;
}

static inline int throttled_hierarchy(struct cfs_rq *cfs_rq)
{
	return 0;
}

static inline int throttled_lb_pair(struct task_group *tg,
				    int src_cpu, int dest_cpu)
{
	return 0;
}

void init_cfs_bandwidth(struct cfs_bandwidth *cfs_b) {}

#ifdef CONFIG_FAIR_GROUP_SCHED
static void init_cfs_rq_runtime(struct cfs_rq *cfs_rq) {}
#endif

static inline struct cfs_bandwidth *tg_cfs_bandwidth(struct task_group *tg)
{
	return NULL;
}
static inline void destroy_cfs_bandwidth(struct cfs_bandwidth *cfs_b) {}
static inline void unthrottle_offline_cfs_rqs(struct rq *rq) {}

#endif 


#ifdef CONFIG_SCHED_HRTICK
static void hrtick_start_fair(struct rq *rq, struct task_struct *p)
{
	struct sched_entity *se = &p->se;
	struct cfs_rq *cfs_rq = cfs_rq_of(se);

	WARN_ON(task_rq(p) != rq);

	if (rq->cfs.h_nr_running > 1) {
		u64 slice = sched_slice(cfs_rq, se);
		u64 ran = se->sum_exec_runtime - se->prev_sum_exec_runtime;
		s64 delta = slice - ran;

		if (delta < 0) {
			if (rq->curr == p)
				resched_task(p);
			return;
		}

		if (rq->curr != p)
			delta = max_t(s64, 10000LL, delta);

		hrtick_start(rq, delta);
	}
}

static void hrtick_update(struct rq *rq)
{
	struct task_struct *curr = rq->curr;

	if (!hrtick_enabled(rq) || curr->sched_class != &fair_sched_class)
		return;

	hrtick_start_fair(rq, curr);
}
#else 
static inline void
hrtick_start_fair(struct rq *rq, struct task_struct *p)
{
}

static inline void hrtick_update(struct rq *rq)
{
}
#endif

static void
enqueue_task_fair(struct rq *rq, struct task_struct *p, int flags)
{
	struct cfs_rq *cfs_rq;
	struct sched_entity *se = &p->se;

	for_each_sched_entity(se) {
		if (se->on_rq)
			break;
		cfs_rq = cfs_rq_of(se);
		enqueue_entity(cfs_rq, se, flags);

		if (cfs_rq_throttled(cfs_rq))
			break;
		cfs_rq->h_nr_running++;

		flags = ENQUEUE_WAKEUP;
	}

	for_each_sched_entity(se) {
		cfs_rq = cfs_rq_of(se);
		cfs_rq->h_nr_running++;

		if (cfs_rq_throttled(cfs_rq))
			break;

		update_cfs_shares(cfs_rq);
		update_entity_load_avg(se, 1);
	}

	if (!se) {
		update_rq_runnable_avg(rq, rq->nr_running);
		inc_nr_running(rq);
		inc_nr_big_small_task(rq, p);
	}
	hrtick_update(rq);
}

static void set_next_buddy(struct sched_entity *se);

static void dequeue_task_fair(struct rq *rq, struct task_struct *p, int flags)
{
	struct cfs_rq *cfs_rq;
	struct sched_entity *se = &p->se;
	int task_sleep = flags & DEQUEUE_SLEEP;

	for_each_sched_entity(se) {
		cfs_rq = cfs_rq_of(se);
		dequeue_entity(cfs_rq, se, flags);

		if (cfs_rq_throttled(cfs_rq))
			break;
		cfs_rq->h_nr_running--;

		
		if (cfs_rq->load.weight) {
			if (task_sleep && parent_entity(se))
				set_next_buddy(parent_entity(se));

			
			se = parent_entity(se);
			break;
		}
		flags |= DEQUEUE_SLEEP;
	}

	for_each_sched_entity(se) {
		cfs_rq = cfs_rq_of(se);
		cfs_rq->h_nr_running--;

		if (cfs_rq_throttled(cfs_rq))
			break;

		update_cfs_shares(cfs_rq);
		update_entity_load_avg(se, 1);
	}

	if (!se) {
		dec_nr_running(rq);
		update_rq_runnable_avg(rq, 1);
		dec_nr_big_small_task(rq, p);
	}
	hrtick_update(rq);
}

#ifdef CONFIG_SMP
static unsigned long weighted_cpuload(const int cpu)
{
	return cpu_rq(cpu)->load.weight;
}

static unsigned long source_load(int cpu, int type)
{
	struct rq *rq = cpu_rq(cpu);
	unsigned long total = weighted_cpuload(cpu);

	if (type == 0 || !sched_feat(LB_BIAS))
		return total;

	return min(rq->cpu_load[type-1], total);
}

static unsigned long target_load(int cpu, int type)
{
	struct rq *rq = cpu_rq(cpu);
	unsigned long total = weighted_cpuload(cpu);

	if (type == 0 || !sched_feat(LB_BIAS))
		return total;

	return max(rq->cpu_load[type-1], total);
}

static unsigned long power_of(int cpu)
{
	return cpu_rq(cpu)->cpu_power;
}

static unsigned long cpu_avg_load_per_task(int cpu)
{
	struct rq *rq = cpu_rq(cpu);
	unsigned long nr_running = ACCESS_ONCE(rq->nr_running);

	if (nr_running)
		return rq->load.weight / nr_running;

	return 0;
}


static void task_waking_fair(struct task_struct *p)
{
	struct sched_entity *se = &p->se;
	struct cfs_rq *cfs_rq = cfs_rq_of(se);
	u64 min_vruntime;

#ifndef CONFIG_64BIT
	u64 min_vruntime_copy;

	do {
		min_vruntime_copy = cfs_rq->min_vruntime_copy;
		smp_rmb();
		min_vruntime = cfs_rq->min_vruntime;
	} while (min_vruntime != min_vruntime_copy);
#else
	min_vruntime = cfs_rq->min_vruntime;
#endif

	se->vruntime -= min_vruntime;
}

#ifdef CONFIG_FAIR_GROUP_SCHED
static long effective_load(struct task_group *tg, int cpu, long wl, long wg)
{
	struct sched_entity *se = tg->se[cpu];

	if (!tg->parent)	
		return wl;

	for_each_sched_entity(se) {
		long w, W;

		tg = se->my_q->tg;

		W = wg + calc_tg_weight(tg, se->my_q);

		w = se->my_q->load.weight + wl;

		if (W > 0 && w < W)
			wl = (w * tg->shares) / W;
		else
			wl = tg->shares;

		if (wl < MIN_SHARES)
			wl = MIN_SHARES;

		wl -= se->load.weight;

		wg = 0;
	}

	return wl;
}
#else

static inline unsigned long effective_load(struct task_group *tg, int cpu,
		unsigned long wl, unsigned long wg)
{
	return wl;
}

#endif

static int wake_affine(struct sched_domain *sd, struct task_struct *p, int sync)
{
	s64 this_load, load;
	int idx, this_cpu, prev_cpu;
	unsigned long tl_per_task;
	struct task_group *tg;
	unsigned long weight;
	int balanced;

	idx	  = sd->wake_idx;
	this_cpu  = smp_processor_id();
	prev_cpu  = task_cpu(p);
	load	  = source_load(prev_cpu, idx);
	this_load = target_load(this_cpu, idx);

	if (sync) {
		tg = task_group(current);
		weight = current->se.load.weight;

		this_load += effective_load(tg, this_cpu, -weight, -weight);
		load += effective_load(tg, prev_cpu, 0, -weight);
	}

	tg = task_group(p);
	weight = p->se.load.weight;

	if (this_load > 0) {
		s64 this_eff_load, prev_eff_load;

		this_eff_load = 100;
		this_eff_load *= power_of(prev_cpu);
		this_eff_load *= this_load +
			effective_load(tg, this_cpu, weight, weight);

		prev_eff_load = 100 + (sd->imbalance_pct - 100) / 2;
		prev_eff_load *= power_of(this_cpu);
		prev_eff_load *= load + effective_load(tg, prev_cpu, 0, weight);

		balanced = this_eff_load <= prev_eff_load;
	} else
		balanced = true;

	if (sync && balanced)
		return 1;

	schedstat_inc(p, se.statistics.nr_wakeups_affine_attempts);
	tl_per_task = cpu_avg_load_per_task(this_cpu);

	if (balanced ||
	    (this_load <= load &&
	     this_load + target_load(prev_cpu, idx) <= tl_per_task)) {
		schedstat_inc(sd, ttwu_move_affine);
		schedstat_inc(p, se.statistics.nr_wakeups_affine);

		return 1;
	}
	return 0;
}

static struct sched_group *
find_idlest_group(struct sched_domain *sd, struct task_struct *p,
		  int this_cpu, int load_idx)
{
	struct sched_group *idlest = NULL, *group = sd->groups;
	unsigned long min_load = ULONG_MAX, this_load = 0;
	int imbalance = 100 + (sd->imbalance_pct-100)/2;

	do {
		unsigned long load, avg_load;
		int local_group;
		int i;

		
		if (!cpumask_intersects(sched_group_cpus(group),
					tsk_cpus_allowed(p)))
			continue;

		local_group = cpumask_test_cpu(this_cpu,
					       sched_group_cpus(group));

		
		avg_load = 0;

		for_each_cpu(i, sched_group_cpus(group)) {
			
			if (local_group)
				load = source_load(i, load_idx);
			else
				load = target_load(i, load_idx);

			avg_load += load;
		}

		
		avg_load = (avg_load * SCHED_POWER_SCALE) / group->sgp->power;

		if (local_group) {
			this_load = avg_load;
		} else if (avg_load < min_load) {
			min_load = avg_load;
			idlest = group;
		}
	} while (group = group->next, group != sd->groups);

	if (!idlest || 100*this_load < imbalance*min_load)
		return NULL;
	return idlest;
}

static int
find_idlest_cpu(struct sched_group *group, struct task_struct *p, int this_cpu)
{
	unsigned long load, min_load = ULONG_MAX;
	int idlest = -1;
	int i;

	
	for_each_cpu_and(i, sched_group_cpus(group), tsk_cpus_allowed(p)) {
		load = weighted_cpuload(i);

		if (load < min_load || (load == min_load && i == this_cpu)) {
			min_load = load;
			idlest = i;
		}
	}

	return idlest;
}

static int select_idle_sibling(struct task_struct *p, int target)
{
	struct sched_domain *sd;
	struct sched_group *sg;
	int i = task_cpu(p);

	if (idle_cpu(target))
		return target;

	if (i != target && cpus_share_cache(i, target) && idle_cpu(i))
		return i;

	if (!sysctl_sched_wake_to_idle &&
	    !(current->flags & PF_WAKE_UP_IDLE) &&
	    !(p->flags & PF_WAKE_UP_IDLE))
		return target;

	sd = rcu_dereference(per_cpu(sd_llc, target));
	for_each_lower_domain(sd) {
		sg = sd->groups;
		do {
			if (!cpumask_intersects(sched_group_cpus(sg),
						tsk_cpus_allowed(p)))
				goto next;

			for_each_cpu(i, sched_group_cpus(sg)) {
				if (i == target || !idle_cpu(i))
					goto next;
			}

			target = cpumask_first_and(sched_group_cpus(sg),
					tsk_cpus_allowed(p));
			goto done;
next:
			sg = sg->next;
		} while (sg != sd->groups);
	}
done:
	return target;
}

static int
select_task_rq_fair(struct task_struct *p, int sd_flag, int wake_flags)
{
	struct sched_domain *tmp, *affine_sd = NULL, *sd = NULL;
	int cpu = smp_processor_id();
	int prev_cpu = task_cpu(p);
	int new_cpu = cpu;
	int want_affine = 0;
	int sync = wake_flags & WF_SYNC;

	if (p->nr_cpus_allowed == 1)
		return prev_cpu;

	if (sched_enable_hmp)
		return select_best_cpu(p, prev_cpu, 0, sync);

	if (sd_flag & SD_BALANCE_WAKE) {
		if (cpumask_test_cpu(cpu, tsk_cpus_allowed(p)))
			want_affine = 1;
		new_cpu = prev_cpu;
	}

	rcu_read_lock();
	for_each_domain(cpu, tmp) {
		if (!(tmp->flags & SD_LOAD_BALANCE))
			continue;

		if (want_affine && (tmp->flags & SD_WAKE_AFFINE) &&
		    cpumask_test_cpu(prev_cpu, sched_domain_span(tmp))) {
			affine_sd = tmp;
			break;
		}

		if (tmp->flags & sd_flag)
			sd = tmp;
	}

	if (affine_sd) {
		if (cpu != prev_cpu && wake_affine(affine_sd, p, sync))
			prev_cpu = cpu;

		new_cpu = select_idle_sibling(p, prev_cpu);
		goto unlock;
	}

	while (sd) {
		int load_idx = sd->forkexec_idx;
		struct sched_group *group;
		int weight;

		if (!(sd->flags & sd_flag)) {
			sd = sd->child;
			continue;
		}

		if (sd_flag & SD_BALANCE_WAKE)
			load_idx = sd->wake_idx;

		group = find_idlest_group(sd, p, cpu, load_idx);
		if (!group) {
			sd = sd->child;
			continue;
		}

		new_cpu = find_idlest_cpu(group, p, cpu);
		if (new_cpu == -1 || new_cpu == cpu) {
			
			sd = sd->child;
			continue;
		}

		
		cpu = new_cpu;
		weight = sd->span_weight;
		sd = NULL;
		for_each_domain(cpu, tmp) {
			if (weight <= tmp->span_weight)
				break;
			if (tmp->flags & sd_flag)
				sd = tmp;
		}
		
	}
unlock:
	rcu_read_unlock();

	return new_cpu;
}

#ifdef CONFIG_FAIR_GROUP_SCHED
static void
migrate_task_rq_fair(struct task_struct *p, int next_cpu)
{
	struct sched_entity *se = &p->se;
	struct cfs_rq *cfs_rq = cfs_rq_of(se);

	if (se->avg.decay_count) {
		se->avg.decay_count = -__synchronize_entity_decay(se);
		atomic64_add(se->avg.load_avg_contrib, &cfs_rq->removed_load);
	}
}
#endif
#endif 

static unsigned long
wakeup_gran(struct sched_entity *curr, struct sched_entity *se)
{
	unsigned long gran = sysctl_sched_wakeup_granularity;

	return calc_delta_fair(gran, se);
}

static int
wakeup_preempt_entity(struct sched_entity *curr, struct sched_entity *se)
{
	s64 gran, vdiff = curr->vruntime - se->vruntime;

	if (vdiff <= 0)
		return -1;

	gran = wakeup_gran(curr, se);
	if (vdiff > gran)
		return 1;

	return 0;
}

static void set_last_buddy(struct sched_entity *se)
{
	if (entity_is_task(se) && unlikely(task_of(se)->policy == SCHED_IDLE))
		return;

	for_each_sched_entity(se)
		cfs_rq_of(se)->last = se;
}

static void set_next_buddy(struct sched_entity *se)
{
	if (entity_is_task(se) && unlikely(task_of(se)->policy == SCHED_IDLE))
		return;

	for_each_sched_entity(se)
		cfs_rq_of(se)->next = se;
}

static void set_skip_buddy(struct sched_entity *se)
{
	for_each_sched_entity(se)
		cfs_rq_of(se)->skip = se;
}

static void check_preempt_wakeup(struct rq *rq, struct task_struct *p, int wake_flags)
{
	struct task_struct *curr = rq->curr;
	struct sched_entity *se = &curr->se, *pse = &p->se;
	struct cfs_rq *cfs_rq = task_cfs_rq(curr);
	int scale = cfs_rq->nr_running >= sched_nr_latency;
	int next_buddy_marked = 0;

	if (unlikely(se == pse))
		return;

	if (unlikely(throttled_hierarchy(cfs_rq_of(pse))))
		return;

	if (sched_feat(NEXT_BUDDY) && scale && !(wake_flags & WF_FORK)) {
		set_next_buddy(pse);
		next_buddy_marked = 1;
	}

	if (test_tsk_need_resched(curr))
		return;

	
	if (unlikely(curr->policy == SCHED_IDLE) &&
	    likely(p->policy != SCHED_IDLE))
		goto preempt;

	if (unlikely(p->policy != SCHED_NORMAL) || !sched_feat(WAKEUP_PREEMPTION))
		return;

	find_matching_se(&se, &pse);
	update_curr(cfs_rq_of(se));
	BUG_ON(!pse);
	if (wakeup_preempt_entity(se, pse) == 1) {
		if (!next_buddy_marked)
			set_next_buddy(pse);
		goto preempt;
	}

	return;

preempt:
	resched_task(curr);
	if (unlikely(!se->on_rq || curr == rq->idle))
		return;

	if (sched_feat(LAST_BUDDY) && scale && entity_is_task(se))
		set_last_buddy(se);
}

static struct task_struct *pick_next_task_fair(struct rq *rq)
{
	struct task_struct *p;
	struct cfs_rq *cfs_rq = &rq->cfs;
	struct sched_entity *se;

	if (!cfs_rq->nr_running)
		return NULL;

	do {
		se = pick_next_entity(cfs_rq);
		set_next_entity(cfs_rq, se);
		cfs_rq = group_cfs_rq(se);
	} while (cfs_rq);

	p = task_of(se);
	if (hrtick_enabled(rq))
		hrtick_start_fair(rq, p);

	return p;
}

static void put_prev_task_fair(struct rq *rq, struct task_struct *prev)
{
	struct sched_entity *se = &prev->se;
	struct cfs_rq *cfs_rq;

	for_each_sched_entity(se) {
		cfs_rq = cfs_rq_of(se);
		put_prev_entity(cfs_rq, se);
	}
}

static void yield_task_fair(struct rq *rq)
{
	struct task_struct *curr = rq->curr;
	struct cfs_rq *cfs_rq = task_cfs_rq(curr);
	struct sched_entity *se = &curr->se;

	if (unlikely(rq->nr_running == 1))
		return;

	clear_buddies(cfs_rq, se);

	if (curr->policy != SCHED_BATCH) {
		update_rq_clock(rq);
		update_curr(cfs_rq);
		 rq->skip_clock_update = 1;
	}

	set_skip_buddy(se);
}

static bool yield_to_task_fair(struct rq *rq, struct task_struct *p, bool preempt)
{
	struct sched_entity *se = &p->se;

	
	if (!se->on_rq || throttled_hierarchy(cfs_rq_of(se)))
		return false;

	
	set_next_buddy(se);

	yield_task_fair(rq);

	return true;
}

#ifdef CONFIG_SMP
 

static unsigned long __read_mostly max_load_balance_interval = HZ/10;

#define LBF_ALL_PINNED	0x01
#define LBF_NEED_BREAK	0x02
#define LBF_SOME_PINNED 0x04
#define LBF_IGNORE_SMALL_TASKS 0x08
#define LBF_PWR_ACTIVE_BALANCE 0x10
#define LBF_SCHED_BOOST 0x20

struct lb_env {
	struct sched_domain	*sd;

	struct rq		*src_rq;
	int			src_cpu;

	int			dst_cpu;
	struct rq		*dst_rq;

	struct cpumask		*dst_grpmask;
	int			new_dst_cpu;
	enum cpu_idle_type	idle;
	long			imbalance;
	
	struct cpumask		*cpus;
	unsigned int		busiest_grp_capacity;
	unsigned int		busiest_nr_running;

	unsigned int		flags;

	unsigned int		loop;
	unsigned int		loop_break;
	unsigned int		loop_max;
};

static DEFINE_PER_CPU(bool, dbs_boost_needed);
static DEFINE_PER_CPU(int, dbs_boost_load_moved);

static void move_task(struct task_struct *p, struct lb_env *env)
{
	deactivate_task(env->src_rq, p, 0);
	set_task_cpu(p, env->dst_cpu);
	activate_task(env->dst_rq, p, 0);
	check_preempt_curr(env->dst_rq, p, 0);
	if (task_notify_on_migrate(p))
		per_cpu(dbs_boost_needed, env->dst_cpu) = true;
}

static int
task_hot(struct task_struct *p, u64 now, struct sched_domain *sd)
{
	s64 delta;

	if (p->sched_class != &fair_sched_class)
		return 0;

	if (unlikely(p->policy == SCHED_IDLE))
		return 0;

	if (sched_feat(CACHE_HOT_BUDDY) && this_rq()->nr_running &&
			(&p->se == cfs_rq_of(&p->se)->next ||
			 &p->se == cfs_rq_of(&p->se)->last))
		return 1;

	if (sysctl_sched_migration_cost == -1)
		return 1;
	if (sysctl_sched_migration_cost == 0)
		return 0;

	delta = now - p->se.exec_start;

	return delta < (s64)sysctl_sched_migration_cost;
}

static
int can_migrate_task(struct task_struct *p, struct lb_env *env)
{
	int tsk_cache_hot = 0;


	if (over_schedule_budget(env->dst_cpu))
		return 0;

	if (throttled_lb_pair(task_group(p), env->src_cpu, env->dst_cpu))
		return 0;

	if (nr_big_tasks(env->src_rq) &&
			capacity(env->dst_rq) > capacity(env->src_rq) &&
			!is_big_task(p))
		return 0;

	if (env->flags & LBF_IGNORE_SMALL_TASKS && is_small_task(p))
		return 0;

	if (!task_will_fit(p, env->dst_cpu) &&
			env->busiest_nr_running <= env->busiest_grp_capacity)
		return 0;

	if (!cpumask_test_cpu(env->dst_cpu, tsk_cpus_allowed(p))) {
		int cpu;

		schedstat_inc(p, se.statistics.nr_failed_migrations_affine);

		if (!env->dst_grpmask || (env->flags & LBF_SOME_PINNED))
			return 0;

		
		for_each_cpu_and(cpu, env->dst_grpmask, env->cpus) {
			if (cpumask_test_cpu(cpu, tsk_cpus_allowed(p))) {
				env->flags |= LBF_SOME_PINNED;
				env->new_dst_cpu = cpu;
				break;
			}
		}

		return 0;
	}

	
	env->flags &= ~LBF_ALL_PINNED;

	if (task_running(env->src_rq, p)) {
		schedstat_inc(p, se.statistics.nr_failed_migrations_running);
		return 0;
	}


	tsk_cache_hot = task_hot(p, env->src_rq->clock_task, env->sd);
	if (!tsk_cache_hot ||
		env->sd->nr_balance_failed > env->sd->cache_nice_tries) {

		if (tsk_cache_hot) {
			schedstat_inc(env->sd, lb_hot_gained[env->idle]);
			schedstat_inc(p, se.statistics.nr_forced_migrations);
		}

		return 1;
	}

	schedstat_inc(p, se.statistics.nr_failed_migrations_hot);
	return 0;
}

static int move_one_task(struct lb_env *env)
{
	struct task_struct *p, *n;

	list_for_each_entry_safe(p, n, &env->src_rq->cfs_tasks, se.group_node) {
		if (!can_migrate_task(p, env))
			continue;
		move_task(p, env);
		schedstat_inc(env->sd, lb_gained[env->idle]);
		per_cpu(dbs_boost_load_moved, env->dst_cpu) += pct_task_load(p);

		return 1;
	}
	return 0;
}

static unsigned long task_h_load(struct task_struct *p);

static const unsigned int sched_nr_migrate_break = 32;

static int move_tasks(struct lb_env *env)
{
	struct list_head *tasks = &env->src_rq->cfs_tasks;
	struct task_struct *p;
	unsigned long load;
	int pulled = 0;
	int orig_loop = env->loop;

	if (env->imbalance <= 0)
		return 0;

	if (capacity(env->dst_rq) > capacity(env->src_rq))
		env->flags |= LBF_IGNORE_SMALL_TASKS;

redo:
	while (!list_empty(tasks)) {
		p = list_first_entry(tasks, struct task_struct, se.group_node);

		env->loop++;
		
		if (env->loop > env->loop_max)
			break;

		
		if (env->loop > env->loop_break) {
			env->loop_break += sched_nr_migrate_break;
			env->flags |= LBF_NEED_BREAK;
			break;
		}

		if (!can_migrate_task(p, env))
			goto next;

		load = task_h_load(p);

		if (sched_feat(LB_MIN) && load < 16 && !env->sd->nr_balance_failed)
			goto next;

		if ((load / 2) > env->imbalance)
			goto next;

		move_task(p, env);
		pulled++;
		env->imbalance -= load;
		per_cpu(dbs_boost_load_moved, env->dst_cpu) += pct_task_load(p);

#ifdef CONFIG_PREEMPT
		if (env->idle == CPU_NEWLY_IDLE)
			break;
#endif

		if (env->imbalance <= 0)
			break;

		continue;
next:
		list_move_tail(&p->se.group_node, tasks);
	}

	if (env->flags & LBF_IGNORE_SMALL_TASKS && !pulled) {
		tasks = &env->src_rq->cfs_tasks;
		env->flags &= ~LBF_IGNORE_SMALL_TASKS;
		env->loop = orig_loop;
		goto redo;
	}

	schedstat_add(env->sd, lb_gained[env->idle], pulled);

	return pulled;
}

#ifdef CONFIG_FAIR_GROUP_SCHED
static void __update_blocked_averages_cpu(struct task_group *tg, int cpu)
{
	struct sched_entity *se = tg->se[cpu];
	struct cfs_rq *cfs_rq = tg->cfs_rq[cpu];

	
	if (throttled_hierarchy(cfs_rq))
		return;

	update_cfs_rq_blocked_load(cfs_rq, 1);

	if (se) {
		update_entity_load_avg(se, 1);
		if (!se->avg.runnable_avg_sum && !cfs_rq->nr_running)
			list_del_leaf_cfs_rq(cfs_rq);
	} else {
		struct rq *rq = rq_of(cfs_rq);
		update_rq_runnable_avg(rq, rq->nr_running);
	}
}

static void update_blocked_averages(int cpu)
{
	struct rq *rq = cpu_rq(cpu);
	struct cfs_rq *cfs_rq;
	unsigned long flags;

	raw_spin_lock_irqsave(&rq->lock, flags);
	update_rq_clock(rq);
	for_each_leaf_cfs_rq(rq, cfs_rq) {
		__update_blocked_averages_cpu(cfs_rq->tg, rq->cpu);
	}

	raw_spin_unlock_irqrestore(&rq->lock, flags);
}

static int tg_load_down(struct task_group *tg, void *data)
{
	unsigned long load;
	long cpu = (long)data;

	if (!tg->parent) {
		load = cpu_rq(cpu)->load.weight;
	} else {
		load = tg->parent->cfs_rq[cpu]->h_load;
		load *= tg->se[cpu]->load.weight;
		load /= tg->parent->cfs_rq[cpu]->load.weight + 1;
	}

	tg->cfs_rq[cpu]->h_load = load;

	return 0;
}

static void update_h_load(long cpu)
{
	struct rq *rq = cpu_rq(cpu);
	unsigned long now = jiffies;

	if (rq->h_load_throttle == now)
		return;

	rq->h_load_throttle = now;

	rcu_read_lock();
	walk_tg_tree(tg_load_down, tg_nop, (void *)cpu);
	rcu_read_unlock();
}

static unsigned long task_h_load(struct task_struct *p)
{
	struct cfs_rq *cfs_rq = task_cfs_rq(p);
	unsigned long load;

	load = p->se.load.weight;
	load = div_u64(load * cfs_rq->h_load, cfs_rq->load.weight + 1);

	return load;
}
#else
static inline void update_blocked_averages(int cpu)
{
}

static inline void update_h_load(long cpu)
{
}

static unsigned long task_h_load(struct task_struct *p)
{
	return p->se.load.weight;
}
#endif

struct sd_lb_stats {
	struct sched_group *busiest; 
	struct sched_group *this;  
	unsigned long total_load;  
	unsigned long total_pwr;   
	unsigned long avg_load;	   

	
	unsigned long this_load;
	unsigned long this_load_per_task;
	unsigned long this_nr_running;
	unsigned long this_has_capacity;
	unsigned long this_group_capacity;
	unsigned int  this_idle_cpus;

	
	unsigned int  busiest_idle_cpus;
	unsigned long max_load;
	unsigned long busiest_load_per_task;
	unsigned long busiest_nr_running;
#ifdef CONFIG_SCHED_HMP
	unsigned long busiest_nr_small_tasks;
	unsigned long busiest_nr_big_tasks;
	u64 busiest_scaled_load;
#endif
	unsigned long busiest_group_capacity;
	unsigned long busiest_has_capacity;
	unsigned int  busiest_group_weight;

	int group_imb; 
};

struct sg_lb_stats {
	unsigned long avg_load; 
	unsigned long group_load; 
	unsigned long sum_nr_running; 
#ifdef CONFIG_SCHED_HMP
	unsigned long sum_nr_big_tasks, sum_nr_small_tasks;
	u64 group_cpu_load; 
#endif
	unsigned long sum_weighted_load; 
	unsigned long group_capacity;
	unsigned long idle_cpus;
	unsigned long group_weight;
	int group_imb; 
	int group_has_capacity; 
};

#ifdef CONFIG_SCHED_HMP

static int
bail_inter_cluster_balance(struct lb_env *env, struct sd_lb_stats *sds)
{
	int nr_cpus;

	if (group_rq_capacity(sds->this) <= group_rq_capacity(sds->busiest))
		return 0;

	if (sds->busiest_nr_big_tasks)
		return 0;

	nr_cpus = cpumask_weight(sched_group_cpus(sds->busiest));

	if ((sds->busiest_scaled_load < nr_cpus * sched_spill_load) &&
		(sds->busiest_nr_running <
			nr_cpus * sysctl_sched_spill_nr_run)) {
			return 1;
	}

	return 0;
}

#else	

static inline int
bail_inter_cluster_balance(struct lb_env *env, struct sd_lb_stats *sds)
{
	return 0;
}

#endif	

static inline int get_sd_load_idx(struct sched_domain *sd,
					enum cpu_idle_type idle)
{
	int load_idx;

	switch (idle) {
	case CPU_NOT_IDLE:
		load_idx = sd->busy_idx;
		break;

	case CPU_NEWLY_IDLE:
		load_idx = sd->newidle_idx;
		break;
	default:
		load_idx = sd->idle_idx;
		break;
	}

	return load_idx;
}

static unsigned long default_scale_freq_power(struct sched_domain *sd, int cpu)
{
	return SCHED_POWER_SCALE;
}

unsigned long __weak arch_scale_freq_power(struct sched_domain *sd, int cpu)
{
	return default_scale_freq_power(sd, cpu);
}

static unsigned long default_scale_smt_power(struct sched_domain *sd, int cpu)
{
	unsigned long weight = sd->span_weight;
	unsigned long smt_gain = sd->smt_gain;

	smt_gain /= weight;

	return smt_gain;
}

unsigned long __weak arch_scale_smt_power(struct sched_domain *sd, int cpu)
{
	return default_scale_smt_power(sd, cpu);
}

static unsigned long scale_rt_power(int cpu)
{
	struct rq *rq = cpu_rq(cpu);
	u64 total, available, age_stamp, avg;

	age_stamp = ACCESS_ONCE(rq->age_stamp);
	avg = ACCESS_ONCE(rq->rt_avg);

	total = sched_avg_period() + (rq->clock - age_stamp);

	if (unlikely(total < avg)) {
		
		available = 0;
	} else {
		available = total - avg;
	}

	if (unlikely((s64)total < SCHED_POWER_SCALE))
		total = SCHED_POWER_SCALE;

	total >>= SCHED_POWER_SHIFT;

	return div_u64(available, total);
}

static void update_cpu_power(struct sched_domain *sd, int cpu)
{
	unsigned long weight = sd->span_weight;
	unsigned long power = SCHED_POWER_SCALE;
	struct sched_group *sdg = sd->groups;

	if ((sd->flags & SD_SHARE_CPUPOWER) && weight > 1) {
		if (sched_feat(ARCH_POWER))
			power *= arch_scale_smt_power(sd, cpu);
		else
			power *= default_scale_smt_power(sd, cpu);

		power >>= SCHED_POWER_SHIFT;
	}

	sdg->sgp->power_orig = power;

	if (sched_feat(ARCH_POWER))
		power *= arch_scale_freq_power(sd, cpu);
	else
		power *= default_scale_freq_power(sd, cpu);

	power >>= SCHED_POWER_SHIFT;

	power *= scale_rt_power(cpu);
	power >>= SCHED_POWER_SHIFT;

	if (!power)
		power = 1;

	cpu_rq(cpu)->cpu_power = power;
	sdg->sgp->power = power;
}

void update_group_power(struct sched_domain *sd, int cpu)
{
	struct sched_domain *child = sd->child;
	struct sched_group *group, *sdg = sd->groups;
	unsigned long power;
	unsigned long interval;

	interval = msecs_to_jiffies(sd->balance_interval);
	interval = clamp(interval, 1UL, max_load_balance_interval);
	sdg->sgp->next_update = jiffies + interval;

	if (!child) {
		update_cpu_power(sd, cpu);
		return;
	}

	power = 0;

	if (child->flags & SD_OVERLAP) {

		for_each_cpu(cpu, sched_group_cpus(sdg))
			power += power_of(cpu);
	} else  {
 

		group = child->groups;
		do {
			power += group->sgp->power;
			group = group->next;
		} while (group != child->groups);
	}

	sdg->sgp->power_orig = sdg->sgp->power = power;
}

static inline int
fix_small_capacity(struct sched_domain *sd, struct sched_group *group)
{
	if (!(sd->flags & SD_SHARE_CPUPOWER))
		return 0;

	if (group->sgp->power * 32 > group->sgp->power_orig * 29)
		return 1;

	return 0;
}

static inline void update_sg_lb_stats(struct lb_env *env,
			struct sched_group *group, int load_idx,
			int local_group, int *balance, struct sg_lb_stats *sgs)
{
	unsigned long nr_running, max_nr_running, min_nr_running;
	unsigned long load, max_cpu_load, min_cpu_load;
	unsigned int balance_cpu = -1, first_idle_cpu = 0;
	unsigned long avg_load_per_task = 0;
	int i;

	if (local_group)
		balance_cpu = group_balance_cpu(group);

	
	max_cpu_load = 0;
	min_cpu_load = ~0UL;
	max_nr_running = 0;
	min_nr_running = ~0UL;

	for_each_cpu_and(i, sched_group_cpus(group), env->cpus) {
		struct rq *rq = cpu_rq(i);

		trace_sched_cpu_load(cpu_rq(i), idle_cpu(i),
				     mostly_idle_cpu(i),
				     sched_irqload(i),
				     power_cost_at_freq(i, 0),
				     cpu_temp(i));
		nr_running = rq->nr_running;

		
		if (local_group) {
			if (idle_cpu(i) && !first_idle_cpu &&
					cpumask_test_cpu(i, sched_group_mask(group))) {
				first_idle_cpu = 1;
				balance_cpu = i;
			}

			load = target_load(i, load_idx);
		} else {
			load = source_load(i, load_idx);
			if (load > max_cpu_load)
				max_cpu_load = load;
			if (min_cpu_load > load)
				min_cpu_load = load;

			if (nr_running > max_nr_running)
				max_nr_running = nr_running;
			if (min_nr_running > nr_running)
				min_nr_running = nr_running;
		}

		sgs->group_load += load;
		sgs->sum_nr_running += nr_running;
#ifdef CONFIG_SCHED_HMP
		sgs->sum_nr_big_tasks += rq->nr_big_tasks;
		sgs->sum_nr_small_tasks += rq->nr_small_tasks;
		sgs->group_cpu_load += cpu_load(i);
#endif
		sgs->sum_weighted_load += weighted_cpuload(i);
		if (idle_cpu(i))
			sgs->idle_cpus++;
	}

	if (local_group) {
		if (env->idle != CPU_NEWLY_IDLE) {
			if (balance_cpu != env->dst_cpu) {
				*balance = 0;
				return;
			}
			update_group_power(env->sd, env->dst_cpu);
		} else if (time_after_eq(jiffies, group->sgp->next_update))
			update_group_power(env->sd, env->dst_cpu);
	}

	
	sgs->avg_load = (sgs->group_load*SCHED_POWER_SCALE) / group->sgp->power;

	if (sgs->sum_nr_running)
		avg_load_per_task = sgs->sum_weighted_load / sgs->sum_nr_running;

	if ((max_cpu_load - min_cpu_load) >= avg_load_per_task &&
	    (max_nr_running - min_nr_running) > 1)
		sgs->group_imb = 1;

	sgs->group_capacity = DIV_ROUND_CLOSEST(group->sgp->power,
						SCHED_POWER_SCALE);
	if (!sgs->group_capacity)
		sgs->group_capacity = fix_small_capacity(env->sd, group);
	sgs->group_weight = group->group_weight;

	if (sgs->group_capacity > sgs->sum_nr_running)
		sgs->group_has_capacity = 1;
}

static bool update_sd_pick_busiest(struct lb_env *env,
				   struct sd_lb_stats *sds,
				   struct sched_group *sg,
				   struct sg_lb_stats *sgs)
{
	int cpu;

	if (sched_boost() && !sds->busiest && sgs->sum_nr_running &&
		(env->idle != CPU_NOT_IDLE) && (capacity(env->dst_rq) >
		group_rq_capacity(sg))) {
		env->flags |= LBF_SCHED_BOOST;
		return true;
	}

	if (sgs->avg_load <= sds->max_load)
		return false;

	if (sgs->sum_nr_running > sgs->group_capacity) {
		env->flags &= ~LBF_PWR_ACTIVE_BALANCE;
		return true;
	}

	if (sgs->group_imb) {
		env->flags &= ~LBF_PWR_ACTIVE_BALANCE;
		return true;
	}

	cpu = cpumask_first(sched_group_cpus(sg));
	if (!sds->busiest && (capacity(env->dst_rq) == group_rq_capacity(sg)) &&
	    sgs->sum_nr_running && (env->idle != CPU_NOT_IDLE) &&
	    power_cost_at_freq(env->dst_cpu, 0) <
	    power_cost_at_freq(cpu, 0) &&
	    !is_task_migration_throttled(cpu_rq(cpu)->curr) &&
	    is_cpu_throttling_imminent(cpu)) {
		env->flags |= LBF_PWR_ACTIVE_BALANCE;
		return true;
	}

	if ((env->sd->flags & SD_ASYM_PACKING) && sgs->sum_nr_running &&
	    env->dst_cpu < group_first_cpu(sg)) {
		if (!sds->busiest)
			return true;

		if (group_first_cpu(sds->busiest) > group_first_cpu(sg))
			return true;
	}

	return false;
}

static inline void update_sd_lb_stats(struct lb_env *env,
					int *balance, struct sd_lb_stats *sds)
{
	struct sched_domain *child = env->sd->child;
	struct sched_group *sg = env->sd->groups;
	struct sg_lb_stats sgs;
	int load_idx, prefer_sibling = 0;

	if (child && child->flags & SD_PREFER_SIBLING)
		prefer_sibling = 1;

	load_idx = get_sd_load_idx(env->sd, env->idle);

	do {
		int local_group;

		local_group = cpumask_test_cpu(env->dst_cpu, sched_group_cpus(sg));
		memset(&sgs, 0, sizeof(sgs));
		update_sg_lb_stats(env, sg, load_idx, local_group, balance, &sgs);

		if (local_group && !(*balance))
			return;

		sds->total_load += sgs.group_load;
		sds->total_pwr += sg->sgp->power;

		if (prefer_sibling && !local_group && sds->this_has_capacity)
			sgs.group_capacity = min(sgs.group_capacity, 1UL);

		if (local_group) {
			sds->this_load = sgs.avg_load;
			sds->this = sg;
			sds->this_nr_running = sgs.sum_nr_running;
			sds->this_load_per_task = sgs.sum_weighted_load;
			sds->this_has_capacity = sgs.group_has_capacity;
			sds->this_idle_cpus = sgs.idle_cpus;
			sds->this_group_capacity = sgs.group_capacity;
		} else if (update_sd_pick_busiest(env, sds, sg, &sgs)) {
			sds->max_load = sgs.avg_load;
			sds->busiest = sg;
			env->busiest_nr_running = sds->busiest_nr_running
							= sgs.sum_nr_running;
			sds->busiest_idle_cpus = sgs.idle_cpus;
			env->busiest_grp_capacity = sds->busiest_group_capacity
							= sgs.group_capacity;
			sds->busiest_load_per_task = sgs.sum_weighted_load;
			sds->busiest_has_capacity = sgs.group_has_capacity;
			sds->busiest_group_weight = sgs.group_weight;
			sds->group_imb = sgs.group_imb;
#ifdef CONFIG_SCHED_HMP
			sds->busiest_nr_small_tasks = sgs.sum_nr_small_tasks;
			sds->busiest_nr_big_tasks = sgs.sum_nr_big_tasks;
			sds->busiest_scaled_load = sgs.group_cpu_load;
#endif
		}

		sg = sg->next;
	} while (sg != env->sd->groups);
}

static int check_asym_packing(struct lb_env *env, struct sd_lb_stats *sds)
{
	int busiest_cpu;

	if (!(env->sd->flags & SD_ASYM_PACKING))
		return 0;

	if (!sds->busiest)
		return 0;

	busiest_cpu = group_first_cpu(sds->busiest);
	if (env->dst_cpu > busiest_cpu)
		return 0;

	env->imbalance = DIV_ROUND_CLOSEST(
		sds->max_load * sds->busiest->sgp->power, SCHED_POWER_SCALE);

	return 1;
}

static inline
void fix_small_imbalance(struct lb_env *env, struct sd_lb_stats *sds)
{
	unsigned long tmp, pwr_now = 0, pwr_move = 0;
	unsigned int imbn = 2;
	unsigned long scaled_busy_load_per_task;

	if (sds->this_nr_running) {
		sds->this_load_per_task /= sds->this_nr_running;
		if (sds->busiest_load_per_task >
				sds->this_load_per_task)
			imbn = 1;
	} else {
		sds->this_load_per_task =
			cpu_avg_load_per_task(env->dst_cpu);
	}

	scaled_busy_load_per_task = sds->busiest_load_per_task
					 * SCHED_POWER_SCALE;
	scaled_busy_load_per_task /= sds->busiest->sgp->power;

	if (sds->max_load - sds->this_load + scaled_busy_load_per_task >=
			(scaled_busy_load_per_task * imbn)) {
		env->imbalance = sds->busiest_load_per_task;
		return;
	}


	pwr_now += sds->busiest->sgp->power *
			min(sds->busiest_load_per_task, sds->max_load);
	pwr_now += sds->this->sgp->power *
			min(sds->this_load_per_task, sds->this_load);
	pwr_now /= SCHED_POWER_SCALE;

	
	tmp = (sds->busiest_load_per_task * SCHED_POWER_SCALE) /
		sds->busiest->sgp->power;
	if (sds->max_load > tmp)
		pwr_move += sds->busiest->sgp->power *
			min(sds->busiest_load_per_task, sds->max_load - tmp);

	
	if (sds->max_load * sds->busiest->sgp->power <
		sds->busiest_load_per_task * SCHED_POWER_SCALE)
		tmp = (sds->max_load * sds->busiest->sgp->power) /
			sds->this->sgp->power;
	else
		tmp = (sds->busiest_load_per_task * SCHED_POWER_SCALE) /
			sds->this->sgp->power;
	pwr_move += sds->this->sgp->power *
			min(sds->this_load_per_task, sds->this_load + tmp);
	pwr_move /= SCHED_POWER_SCALE;

	
	if (pwr_move > pwr_now)
		env->imbalance = sds->busiest_load_per_task;
}

static inline void calculate_imbalance(struct lb_env *env, struct sd_lb_stats *sds)
{
	unsigned long max_pull, load_above_capacity = ~0UL;

	sds->busiest_load_per_task /= sds->busiest_nr_running;
	if (sds->group_imb) {
		sds->busiest_load_per_task =
			min(sds->busiest_load_per_task, sds->avg_load);
	}

	if (sds->max_load < sds->avg_load) {
		env->imbalance = 0;
		return fix_small_imbalance(env, sds);
	}

	if (!sds->group_imb) {
		load_above_capacity = (sds->busiest_nr_running -
						sds->busiest_group_capacity);

		load_above_capacity *= (SCHED_LOAD_SCALE * SCHED_POWER_SCALE);

		load_above_capacity /= sds->busiest->sgp->power;
	}

	max_pull = min(sds->max_load - sds->avg_load, load_above_capacity);

	
	env->imbalance = min(max_pull * sds->busiest->sgp->power,
		(sds->avg_load - sds->this_load) * sds->this->sgp->power)
			/ SCHED_POWER_SCALE;

	if (env->imbalance < sds->busiest_load_per_task)
		return fix_small_imbalance(env, sds);

}


static struct sched_group *
find_busiest_group(struct lb_env *env, int *balance)
{
	struct sd_lb_stats sds;

	memset(&sds, 0, sizeof(sds));

	update_sd_lb_stats(env, balance, &sds);

	if (!(*balance))
		goto ret;

	if ((env->idle == CPU_IDLE || env->idle == CPU_NEWLY_IDLE) &&
	    check_asym_packing(env, &sds))
		return sds.busiest;

	
	if (!sds.busiest || sds.busiest_nr_running == 0)
		goto out_balanced;

	if (sched_boost() && (capacity(env->dst_rq) >
				group_rq_capacity(sds.busiest)))
		goto force_balance;

	if (bail_inter_cluster_balance(env, &sds))
		goto out_balanced;

	sds.avg_load = (SCHED_POWER_SCALE * sds.total_load) / sds.total_pwr;

	if (sds.group_imb)
		goto force_balance;

	
	if (env->idle == CPU_NEWLY_IDLE && sds.this_has_capacity &&
			!sds.busiest_has_capacity)
		goto force_balance;

	if (sds.this_load >= sds.max_load)
		goto out_balanced;

	if (sds.this_load >= sds.avg_load)
		goto out_balanced;

	if (env->idle == CPU_IDLE) {
		if ((sds.this_idle_cpus <= sds.busiest_idle_cpus + 1) &&
		    sds.busiest_nr_running <= sds.busiest_group_weight)
			goto out_balanced;
	} else {
		if (100 * sds.max_load <= env->sd->imbalance_pct * sds.this_load)
			goto out_balanced;
	}

force_balance:
	
	calculate_imbalance(env, &sds);
	return sds.busiest;

out_balanced:
ret:
	env->imbalance = 0;
	return NULL;
}

#ifdef CONFIG_SCHED_HMP
static struct rq *find_busiest_queue_hmp(struct lb_env *env,
				     struct sched_group *group)
{
	struct rq *busiest = NULL, *rq;
	u64 max_runnable_avg = 0;
	int i;

	for_each_cpu(i, sched_group_cpus(group)) {
		if (!cpumask_test_cpu(i, env->cpus))
			continue;

		rq = cpu_rq(i);

		if (rq->cumulative_runnable_avg > max_runnable_avg) {
			max_runnable_avg = rq->cumulative_runnable_avg;
			busiest = rq;
		}
	}

	return busiest;
}
#else
static inline struct rq *find_busiest_queue_hmp(struct lb_env *env,
				     struct sched_group *group)
{
	return NULL;
}
#endif

static struct rq *find_busiest_queue(struct lb_env *env,
				     struct sched_group *group)
{
	struct rq *busiest = NULL, *rq;
	unsigned long max_load = 0;
	int i;

	if (sched_enable_hmp)
		return find_busiest_queue_hmp(env, group);

	for_each_cpu(i, sched_group_cpus(group)) {
		unsigned long power = power_of(i);
		unsigned long capacity = DIV_ROUND_CLOSEST(power,
							SCHED_POWER_SCALE);
		unsigned long wl;

		if (!capacity)
			capacity = fix_small_capacity(env->sd, group);

		if (!cpumask_test_cpu(i, env->cpus))
			continue;

		rq = cpu_rq(i);
		wl = weighted_cpuload(i);

		if (capacity && rq->nr_running == 1 && wl > env->imbalance)
			continue;

		wl = (wl * SCHED_POWER_SCALE) / power;

		if (wl > max_load) {
			max_load = wl;
			busiest = rq;
		}
	}

	return busiest;
}

#define MAX_PINNED_INTERVAL	16

DEFINE_PER_CPU(cpumask_var_t, load_balance_mask);

static int need_active_balance(struct lb_env *env)
{
	struct sched_domain *sd = env->sd;

	if (env->flags & (LBF_PWR_ACTIVE_BALANCE | LBF_SCHED_BOOST))
		return 1;

	if (env->idle == CPU_NEWLY_IDLE) {

		if ((sd->flags & SD_ASYM_PACKING) && env->src_cpu > env->dst_cpu)
			return 1;
	}

	return unlikely(sd->nr_balance_failed > sd->cache_nice_tries+2);
}

static int load_balance(int this_cpu, struct rq *this_rq,
			struct sched_domain *sd, enum cpu_idle_type idle,
			int *balance)
{
	int ld_moved, cur_ld_moved, active_balance = 0;
	struct sched_group *group;
	struct rq *busiest = NULL;
	unsigned long flags;
	struct cpumask *cpus = __get_cpu_var(load_balance_mask);

	struct lb_env env = {
		.sd			= sd,
		.dst_cpu		= this_cpu,
		.dst_rq			= this_rq,
		.dst_grpmask    	= sched_group_cpus(sd->groups),
		.idle			= idle,
		.busiest_nr_running 	= 0,
		.busiest_grp_capacity 	= 0,
		.loop_break		= sched_nr_migrate_break,
		.cpus			= cpus,
		.flags			= 0,
		.loop			= 0,
	};

	if (idle == CPU_NEWLY_IDLE)
		env.dst_grpmask = NULL;

	cpumask_copy(cpus, cpu_active_mask);

	per_cpu(dbs_boost_load_moved, this_cpu) = 0;
	schedstat_inc(sd, lb_count[idle]);

redo:
	group = find_busiest_group(&env, balance);

	if (*balance == 0)
		goto out_balanced;

	if (!group) {
		schedstat_inc(sd, lb_nobusyg[idle]);
		goto out_balanced;
	}

	busiest = find_busiest_queue(&env, group);
	if (!busiest) {
		schedstat_inc(sd, lb_nobusyq[idle]);
		goto out_balanced;
	}

	BUG_ON(busiest == env.dst_rq);

	schedstat_add(sd, lb_imbalance[idle], env.imbalance);

	ld_moved = 0;
	if (busiest->nr_running > 1) {
		env.flags |= LBF_ALL_PINNED;
		env.src_cpu   = busiest->cpu;
		env.src_rq    = busiest;
		env.loop_max  = min(sysctl_sched_nr_migrate, busiest->nr_running);

		update_h_load(env.src_cpu);
more_balance:
		local_irq_save(flags);
		double_rq_lock(env.dst_rq, busiest);

		cur_ld_moved = move_tasks(&env);
		ld_moved += cur_ld_moved;
		double_rq_unlock(env.dst_rq, busiest);
		local_irq_restore(flags);

		if (cur_ld_moved && env.dst_cpu != smp_processor_id())
			resched_cpu(env.dst_cpu);

		if (env.flags & LBF_NEED_BREAK) {
			env.flags &= ~LBF_NEED_BREAK;
			goto more_balance;
		}

		if ((env.flags & LBF_SOME_PINNED) && env.imbalance > 0) {

			env.dst_rq	 = cpu_rq(env.new_dst_cpu);
			env.dst_cpu	 = env.new_dst_cpu;
			env.flags	&= ~LBF_SOME_PINNED;
			env.loop	 = 0;
			env.loop_break	 = sched_nr_migrate_break;

			
			cpumask_clear_cpu(env.dst_cpu, env.cpus);

			goto more_balance;
		}

		
		if (unlikely(env.flags & LBF_ALL_PINNED)) {
			cpumask_clear_cpu(cpu_of(busiest), cpus);
			if (!cpumask_empty(cpus)) {
				env.loop = 0;
				env.loop_break = sched_nr_migrate_break;
				goto redo;
			}
			goto out_balanced;
		}
	}

	if (!ld_moved) {
		if (!(env.flags & (LBF_PWR_ACTIVE_BALANCE | LBF_SCHED_BOOST)))
			schedstat_inc(sd, lb_failed[idle]);

		if (idle != CPU_NEWLY_IDLE &&
		    !(env.flags & (LBF_PWR_ACTIVE_BALANCE | LBF_SCHED_BOOST)) &&
		    !over_schedule_budget(env.dst_cpu))
			sd->nr_balance_failed++;

		if (need_active_balance(&env)) {
			raw_spin_lock_irqsave(&busiest->lock, flags);

			if (!cpumask_test_cpu(this_cpu,
					tsk_cpus_allowed(busiest->curr))) {
				raw_spin_unlock_irqrestore(&busiest->lock,
							    flags);
				env.flags |= LBF_ALL_PINNED;
				goto out_one_pinned;
			}

			if (!busiest->active_balance) {
				busiest->active_balance = 1;
				busiest->push_cpu = this_cpu;
				active_balance = 1;
			}
			raw_spin_unlock_irqrestore(&busiest->lock, flags);

			if (active_balance) {
				stop_one_cpu_nowait(cpu_of(busiest),
					active_load_balance_cpu_stop, busiest,
					&busiest->active_balance_work);
				ld_moved++;
			}

			sd->nr_balance_failed = sd->cache_nice_tries+1;
		}
	} else {
		sd->nr_balance_failed = 0;
		if (per_cpu(dbs_boost_needed, this_cpu)) {
			struct migration_notify_data mnd;

			mnd.src_cpu = cpu_of(busiest);
			mnd.dest_cpu = this_cpu;
			mnd.load = per_cpu(dbs_boost_load_moved, this_cpu);
			if (mnd.load > 100)
				mnd.load = 100;
			atomic_notifier_call_chain(&migration_notifier_head,
						   0, (void *)&mnd);
			per_cpu(dbs_boost_needed, this_cpu) = false;
			per_cpu(dbs_boost_load_moved, this_cpu) = 0;

		}

		
		if (!same_freq_domain(this_cpu, cpu_of(busiest))) {
			check_for_freq_change(this_rq);
			check_for_freq_change(busiest);
		}
	}
	if (likely(!active_balance)) {
		
		sd->balance_interval = sd->min_interval;
	} else {
		if (sd->balance_interval < sd->max_interval)
			sd->balance_interval *= 2;
	}

	goto out;

out_balanced:
	schedstat_inc(sd, lb_balanced[idle]);

	sd->nr_balance_failed = 0;

out_one_pinned:
	
	if (((env.flags & LBF_ALL_PINNED) &&
			sd->balance_interval < MAX_PINNED_INTERVAL) ||
			(sd->balance_interval < sd->max_interval))
		sd->balance_interval *= 2;

	ld_moved = 0;
out:
	trace_sched_load_balance(this_cpu, idle, *balance,
				 group ? group->cpumask[0] : 0,
				 busiest ? busiest->nr_running : 0,
				 env.imbalance, env.flags, ld_moved,
				 sd->balance_interval);
	return ld_moved;
}

void idle_balance(int this_cpu, struct rq *this_rq)
{
	struct sched_domain *sd;
	int pulled_task = 0;
	unsigned long next_balance = jiffies + HZ;
	int i, cost;
	int min_power = INT_MAX;
	int balance_cpu = -1;
	struct rq *balance_rq = NULL;

	this_rq->idle_stamp = this_rq->clock;

	if (this_rq->avg_idle < sysctl_sched_migration_cost)
		return;

	rcu_read_lock();
	sd = rcu_dereference_check_sched_domain(this_rq->sd);
	if (sd && sched_enable_power_aware) {
		for_each_cpu(i, sched_domain_span(sd)) {
			if (i == this_cpu || idle_cpu(i)) {
				cost = power_cost_at_freq(i, 0);
				if (cost < min_power) {
					min_power = cost;
					balance_cpu = i;
				}
			}
		}
		BUG_ON(balance_cpu == -1);

	} else {
		balance_cpu = this_cpu;
	}
	rcu_read_unlock();

	if (over_schedule_budget(balance_cpu)) {
		if (!over_schedule_budget(this_cpu))
			balance_cpu = this_cpu;
		else
			return;
	}

	balance_rq = cpu_rq(balance_cpu);

	raw_spin_unlock(&this_rq->lock);

	update_blocked_averages(balance_cpu);
	rcu_read_lock();
	for_each_domain(balance_cpu, sd) {
		unsigned long interval;
		int balance = 1;

		if (!(sd->flags & SD_LOAD_BALANCE))
			continue;

		if (sd->flags & SD_BALANCE_NEWIDLE) {
			
			pulled_task = load_balance(balance_cpu, balance_rq,
					sd, CPU_NEWLY_IDLE, &balance);
		}

		interval = msecs_to_jiffies(sd->balance_interval);
		if (time_after(next_balance, sd->last_balance + interval))
			next_balance = sd->last_balance + interval;
		if (pulled_task) {
			balance_rq->idle_stamp = 0;
			break;
		}
	}
	rcu_read_unlock();

	raw_spin_lock(&this_rq->lock);

	if (balance_cpu == this_cpu &&
	    (!pulled_task || time_after(jiffies, this_rq->next_balance))) {
		this_rq->next_balance = next_balance;
	}
}

static int active_load_balance_cpu_stop(void *data)
{
	struct rq *busiest_rq = data;
	int busiest_cpu = cpu_of(busiest_rq);
	int target_cpu = busiest_rq->push_cpu;
	struct rq *target_rq = cpu_rq(target_cpu);
	struct sched_domain *sd = NULL;
	struct task_struct *push_task;
	struct lb_env env = {
		.sd			= sd,
		.dst_cpu		= target_cpu,
		.dst_rq			= target_rq,
		.src_cpu		= busiest_rq->cpu,
		.src_rq			= busiest_rq,
		.idle			= CPU_IDLE,
		.busiest_nr_running 	= 0,
		.busiest_grp_capacity 	= 0,
		.flags			= 0,
		.loop			= 0,
	};
	bool moved = false;

	raw_spin_lock_irq(&busiest_rq->lock);

	per_cpu(dbs_boost_load_moved, target_cpu) = 0;

	
	if (unlikely(busiest_cpu != smp_processor_id() ||
		     !busiest_rq->active_balance))
		goto out_unlock;

	
	if (busiest_rq->nr_running <= 1)
		goto out_unlock;

	BUG_ON(busiest_rq == target_rq);

	
	double_lock_balance(busiest_rq, target_rq);

	push_task = busiest_rq->push_task;
	target_cpu = busiest_rq->push_cpu;
	if (push_task) {
		if (push_task->on_rq && push_task->state == TASK_RUNNING &&
		    task_cpu(push_task) == busiest_cpu &&
		    cpu_online(target_cpu)) {
			move_task(push_task, &env);
			moved = true;
		}
		goto out_unlock_balance;
	}

	
	rcu_read_lock();
	for_each_domain(target_cpu, sd) {
		if ((sd->flags & SD_LOAD_BALANCE) &&
		    cpumask_test_cpu(busiest_cpu, sched_domain_span(sd)))
				break;
	}

	if (likely(sd)) {
		env.sd = sd;
		schedstat_inc(sd, alb_count);

		if (move_one_task(&env)) {
			schedstat_inc(sd, alb_pushed);
			moved = true;
		} else {
			schedstat_inc(sd, alb_failed);
		}
	}
	rcu_read_unlock();
out_unlock_balance:
	double_unlock_balance(busiest_rq, target_rq);
out_unlock:
	busiest_rq->active_balance = 0;
	push_task = busiest_rq->push_task;
	target_cpu = busiest_rq->push_cpu;
	if (push_task) {
		put_task_struct(push_task);
		clear_reserved(target_cpu);
		busiest_rq->push_task = NULL;
	}
	raw_spin_unlock_irq(&busiest_rq->lock);

	if (moved && !same_freq_domain(busiest_cpu, target_cpu)) {
		check_for_freq_change(busiest_rq);
		check_for_freq_change(target_rq);
	}

	if (per_cpu(dbs_boost_needed, target_cpu)) {
		struct migration_notify_data mnd;

		mnd.src_cpu = cpu_of(busiest_rq);
		mnd.dest_cpu = target_cpu;
		mnd.load = per_cpu(dbs_boost_load_moved, target_cpu);
		if (mnd.load > 100)
			mnd.load = 100;
		atomic_notifier_call_chain(&migration_notifier_head,
					   0, (void *)&mnd);

		per_cpu(dbs_boost_needed, target_cpu) = false;
		per_cpu(dbs_boost_load_moved, target_cpu) = 0;
	}
	return 0;
}

#ifdef CONFIG_NO_HZ_COMMON
static struct {
	cpumask_var_t idle_cpus_mask;
	atomic_t nr_cpus;
	unsigned long next_balance;     
} nohz ____cacheline_aligned;

static inline int find_new_ilb(int call_cpu, int type)
{
	int ilb;

	if (sched_enable_hmp)
		return find_new_hmp_ilb(call_cpu, type);

	ilb = cpumask_first(nohz.idle_cpus_mask);

	if (ilb < nr_cpu_ids && idle_cpu(ilb))
		return ilb;

	return nr_cpu_ids;
}

static void nohz_balancer_kick(int cpu, int type)
{
	int ilb_cpu;

	nohz.next_balance++;

	ilb_cpu = find_new_ilb(cpu, type);

	if (ilb_cpu >= nr_cpu_ids)
		return;

	if (test_and_set_bit(NOHZ_BALANCE_KICK, nohz_flags(ilb_cpu)))
		return;
	smp_send_reschedule(ilb_cpu);
	return;
}

static inline void nohz_balance_exit_idle(int cpu)
{
	if (unlikely(test_bit(NOHZ_TICK_STOPPED, nohz_flags(cpu)))) {
		cpumask_clear_cpu(cpu, nohz.idle_cpus_mask);
		atomic_dec(&nohz.nr_cpus);
		clear_bit(NOHZ_TICK_STOPPED, nohz_flags(cpu));
	}
}

static inline void set_cpu_sd_state_busy(void)
{
	struct sched_domain *sd;
	int cpu = smp_processor_id();

	rcu_read_lock();
	sd = rcu_dereference_check_sched_domain(cpu_rq(cpu)->sd);

	if (!sd || !sd->nohz_idle)
		goto unlock;
	sd->nohz_idle = 0;

	for (; sd; sd = sd->parent)
		atomic_inc(&sd->groups->sgp->nr_busy_cpus);
unlock:
	rcu_read_unlock();
}

void set_cpu_sd_state_idle(void)
{
	struct sched_domain *sd;
	int cpu = smp_processor_id();

	rcu_read_lock();
	sd = rcu_dereference_check_sched_domain(cpu_rq(cpu)->sd);

	if (!sd || sd->nohz_idle)
		goto unlock;
	sd->nohz_idle = 1;

	for (; sd; sd = sd->parent)
		atomic_dec(&sd->groups->sgp->nr_busy_cpus);
unlock:
	rcu_read_unlock();
}

void nohz_balance_enter_idle(int cpu)
{
	if (!cpu_active(cpu))
		return;

	if (test_bit(NOHZ_TICK_STOPPED, nohz_flags(cpu)))
		return;

	cpumask_set_cpu(cpu, nohz.idle_cpus_mask);
	atomic_inc(&nohz.nr_cpus);
	set_bit(NOHZ_TICK_STOPPED, nohz_flags(cpu));
}

static int __cpuinit sched_ilb_notifier(struct notifier_block *nfb,
					unsigned long action, void *hcpu)
{
	switch (action & ~CPU_TASKS_FROZEN) {
	case CPU_DYING:
		nohz_balance_exit_idle(smp_processor_id());
		return NOTIFY_OK;
	default:
		return NOTIFY_DONE;
	}
}
#endif

static DEFINE_SPINLOCK(balancing);

void update_max_interval(void)
{
	max_load_balance_interval = HZ*num_online_cpus()/10;
}

static void rebalance_domains(int cpu, enum cpu_idle_type idle)
{
	int balance = 1;
	struct rq *rq = cpu_rq(cpu);
	unsigned long interval;
	struct sched_domain *sd;
	
	unsigned long next_balance = jiffies + 60*HZ;
	int update_next_balance = 0;
	int need_serialize;

	update_blocked_averages(cpu);

	rcu_read_lock();
	for_each_domain(cpu, sd) {
		if (!(sd->flags & SD_LOAD_BALANCE))
			continue;

		interval = sd->balance_interval;
		if (idle != CPU_IDLE)
			interval *= sd->busy_factor;

		
		interval = msecs_to_jiffies(interval);
		interval = clamp(interval, 1UL, max_load_balance_interval);

		need_serialize = sd->flags & SD_SERIALIZE;

		if (need_serialize) {
			if (!spin_trylock(&balancing))
				goto out;
		}

		if (time_after_eq(jiffies, sd->last_balance + interval)) {
			if (load_balance(cpu, rq, sd, idle, &balance)) {
				idle = idle_cpu(cpu) ? CPU_IDLE : CPU_NOT_IDLE;
			}
			sd->last_balance = jiffies;
		}
		if (need_serialize)
			spin_unlock(&balancing);
out:
		if (time_after(next_balance, sd->last_balance + interval)) {
			next_balance = sd->last_balance + interval;
			update_next_balance = 1;
		}

		if (!balance)
			break;
	}
	rcu_read_unlock();

	if (likely(update_next_balance))
		rq->next_balance = next_balance;
}

#ifdef CONFIG_NO_HZ_COMMON

static int select_lowest_power_cpu(struct cpumask *cpus)
{
	int i, cost;
	int lowest_power_cpu = -1;
	int lowest_power = INT_MAX;

	if (sched_enable_power_aware) {
		for_each_cpu(i, cpus) {
			cost = power_cost_at_freq(i, 0);
			if (cost < lowest_power) {
				lowest_power_cpu = i;
				lowest_power = cost;
			}
		}
		BUG_ON(lowest_power_cpu == -1);
		return lowest_power_cpu;
	} else {
		return cpumask_first(cpus);
	}
}

static void nohz_idle_balance(int this_cpu, enum cpu_idle_type idle)
{
	struct rq *this_rq = cpu_rq(this_cpu);
	struct rq *rq;
	int balance_cpu;
	struct cpumask cpus_to_balance;

	if (idle != CPU_IDLE ||
	    !test_bit(NOHZ_BALANCE_KICK, nohz_flags(this_cpu)))
		goto end;

	cpumask_copy(&cpus_to_balance, nohz.idle_cpus_mask);

	while (!cpumask_empty(&cpus_to_balance)) {
		balance_cpu = select_lowest_power_cpu(&cpus_to_balance);

		cpumask_clear_cpu(balance_cpu, &cpus_to_balance);
		if (over_schedule_budget(balance_cpu))
			continue;

		if (balance_cpu == this_cpu || !idle_cpu(balance_cpu))
			continue;

		if (need_resched())
			break;

		rq = cpu_rq(balance_cpu);

		raw_spin_lock_irq(&rq->lock);
		update_rq_clock(rq);
		update_idle_cpu_load(rq);
		raw_spin_unlock_irq(&rq->lock);

		rebalance_domains(balance_cpu, CPU_IDLE);

		if (time_after(this_rq->next_balance, rq->next_balance))
			this_rq->next_balance = rq->next_balance;
	}
	nohz.next_balance = this_rq->next_balance;
end:
	clear_bit(NOHZ_BALANCE_KICK, nohz_flags(this_cpu));
}

#ifdef CONFIG_SCHED_HMP
static inline int _nohz_kick_needed_hmp(struct rq *rq, int cpu, int *type)
{
	struct sched_domain *sd;
	int i;

	if (rq->mostly_idle_freq && rq->cur_freq < rq->mostly_idle_freq
		 && rq->max_freq > rq->mostly_idle_freq)
			return 0;

	if (rq->nr_running >= 2 && (rq->nr_running - rq->nr_small_tasks >= 2 ||
	     rq->nr_running > rq->mostly_idle_nr_run ||
		cpu_load(cpu) > rq->mostly_idle_load)) {

		if (rq->capacity == max_capacity)
			return 1;

		rcu_read_lock();
		sd = rcu_dereference_check_sched_domain(rq->sd);
		if (!sd) {
			rcu_read_unlock();
			return 0;
		}

		for_each_cpu(i, sched_domain_span(sd)) {
			if (cpu_load(i) < sched_spill_load) {
				*type = NOHZ_KICK_RESTRICT;
				break;
			}
		}
		rcu_read_unlock();
		return 1;
	}

	return 0;
}
#else
static inline int _nohz_kick_needed_hmp(struct rq *rq, int cpu, int *type)
{
	return 0;
}
#endif

static inline int _nohz_kick_needed(struct rq *rq, int cpu, int *type)
{
	unsigned long now = jiffies;

	if (sched_enable_hmp)
		return _nohz_kick_needed_hmp(rq, cpu, type);

	if (likely(!atomic_read(&nohz.nr_cpus)))
		return 0;

	if (time_before(now, nohz.next_balance))
		return 0;

	return (rq->nr_running >= 2);
}

static inline int nohz_kick_needed(struct rq *rq, int cpu, int *type)
{
	struct sched_domain *sd;

	if (unlikely(idle_cpu(cpu)))
		return 0;

	set_cpu_sd_state_busy();
	nohz_balance_exit_idle(cpu);

	if (_nohz_kick_needed(rq, cpu, type))
		goto need_kick;

	rcu_read_lock();
	for_each_domain(cpu, sd) {
		struct sched_group *sg = sd->groups;
		struct sched_group_power *sgp = sg->sgp;
		int nr_busy = atomic_read(&sgp->nr_busy_cpus);

#ifndef CONFIG_SCHED_HMP
		if (sd->flags & SD_SHARE_PKG_RESOURCES && nr_busy > 1)
			goto need_kick_unlock;
#endif

		if (sd->flags & SD_ASYM_PACKING && nr_busy != sg->group_weight
		    && (cpumask_first_and(nohz.idle_cpus_mask,
					  sched_domain_span(sd)) < cpu))
			goto need_kick_unlock;

		if (!(sd->flags & (SD_SHARE_PKG_RESOURCES | SD_ASYM_PACKING)))
			break;
	}
	rcu_read_unlock();
	return 0;

need_kick_unlock:
	rcu_read_unlock();
need_kick:
	return 1;
}
#else
static void nohz_idle_balance(int this_cpu, enum cpu_idle_type idle) { }
#endif

static void run_rebalance_domains(struct softirq_action *h)
{
	int this_cpu = smp_processor_id();
	struct rq *this_rq = cpu_rq(this_cpu);
	enum cpu_idle_type idle = this_rq->idle_balance ?
						CPU_IDLE : CPU_NOT_IDLE;

	rebalance_domains(this_cpu, idle);

	nohz_idle_balance(this_cpu, idle);
}

static inline int on_null_domain(int cpu)
{
	return !rcu_dereference_sched(cpu_rq(cpu)->sd);
}

void trigger_load_balance(struct rq *rq, int cpu)
{
	int type = NOHZ_KICK_ANY;

	if (over_schedule_budget(cpu)) {
		return;
	}

	
	if (time_after_eq(jiffies, rq->next_balance) &&
	    likely(!on_null_domain(cpu)))
		raise_softirq(SCHED_SOFTIRQ);
#ifdef CONFIG_NO_HZ_COMMON
	if (nohz_kick_needed(rq, cpu, &type) && likely(!on_null_domain(cpu)))
		nohz_balancer_kick(cpu, type);
#endif
}

static void rq_online_fair(struct rq *rq)
{
	update_sysctl();
}

static void rq_offline_fair(struct rq *rq)
{
	update_sysctl();

	
	unthrottle_offline_cfs_rqs(rq);
}

#endif 

static void task_tick_fair(struct rq *rq, struct task_struct *curr, int queued)
{
	struct cfs_rq *cfs_rq;
	struct sched_entity *se = &curr->se;

	for_each_sched_entity(se) {
		cfs_rq = cfs_rq_of(se);
		entity_tick(cfs_rq, se, queued);
	}

	if (sched_feat_numa(NUMA))
		task_tick_numa(rq, curr);

	update_rq_runnable_avg(rq, 1);
}

static void task_fork_fair(struct task_struct *p)
{
	struct cfs_rq *cfs_rq;
	struct sched_entity *se = &p->se, *curr;
	int this_cpu = smp_processor_id();
	struct rq *rq = this_rq();
	unsigned long flags;

	raw_spin_lock_irqsave(&rq->lock, flags);

	update_rq_clock(rq);

	cfs_rq = task_cfs_rq(current);
	curr = cfs_rq->curr;

	rcu_read_lock();
	__set_task_cpu(p, this_cpu);
	rcu_read_unlock();

	update_curr(cfs_rq);

	if (curr)
		se->vruntime = curr->vruntime;
	place_entity(cfs_rq, se, 1);

	if (sysctl_sched_child_runs_first && curr && entity_before(curr, se)) {
		swap(curr->vruntime, se->vruntime);
		resched_task(rq->curr);
	}

	se->vruntime -= cfs_rq->min_vruntime;

	raw_spin_unlock_irqrestore(&rq->lock, flags);
}

static void
prio_changed_fair(struct rq *rq, struct task_struct *p, int oldprio)
{
	if (!p->se.on_rq)
		return;

	if (rq->curr == p) {
		if (p->prio > oldprio)
			resched_task(rq->curr);
	} else
		check_preempt_curr(rq, p, 0);
}

static void switched_from_fair(struct rq *rq, struct task_struct *p)
{
	struct sched_entity *se = &p->se;
	struct cfs_rq *cfs_rq = cfs_rq_of(se);

	if (!p->on_rq && p->state != TASK_RUNNING) {
		place_entity(cfs_rq, se, 0);
		se->vruntime -= cfs_rq->min_vruntime;
	}

#if defined(CONFIG_FAIR_GROUP_SCHED) && defined(CONFIG_SMP)
	if (p->se.avg.decay_count) {
		struct cfs_rq *cfs_rq = cfs_rq_of(&p->se);
		__synchronize_entity_decay(&p->se);
		subtract_blocked_load_contrib(cfs_rq,
				p->se.avg.load_avg_contrib);
	}
#endif
}

static void switched_to_fair(struct rq *rq, struct task_struct *p)
{
	if (!p->se.on_rq)
		return;

	if (rq->curr == p)
		resched_task(rq->curr);
	else
		check_preempt_curr(rq, p, 0);
}

static void set_curr_task_fair(struct rq *rq)
{
	struct sched_entity *se = &rq->curr->se;

	for_each_sched_entity(se) {
		struct cfs_rq *cfs_rq = cfs_rq_of(se);

		set_next_entity(cfs_rq, se);
		
		account_cfs_rq_runtime(cfs_rq, 0);
	}
}

void init_cfs_rq(struct cfs_rq *cfs_rq)
{
	cfs_rq->tasks_timeline = RB_ROOT;
	cfs_rq->min_vruntime = (u64)(-(1LL << 20));
#ifndef CONFIG_64BIT
	cfs_rq->min_vruntime_copy = cfs_rq->min_vruntime;
#endif
#if defined(CONFIG_FAIR_GROUP_SCHED) && defined(CONFIG_SMP)
	atomic64_set(&cfs_rq->decay_counter, 1);
	atomic64_set(&cfs_rq->removed_load, 0);
#endif
}

#ifdef CONFIG_FAIR_GROUP_SCHED
static void task_move_group_fair(struct task_struct *p, int on_rq)
{
	struct cfs_rq *cfs_rq;
	if (!on_rq && (!p->se.sum_exec_runtime || p->state == TASK_WAKING))
		on_rq = 1;

	if (!on_rq)
		p->se.vruntime -= cfs_rq_of(&p->se)->min_vruntime;
	set_task_rq(p, task_cpu(p));
	if (!on_rq) {
		cfs_rq = cfs_rq_of(&p->se);
		p->se.vruntime += cfs_rq->min_vruntime;
#ifdef CONFIG_SMP
		p->se.avg.decay_count = atomic64_read(&cfs_rq->decay_counter);
		cfs_rq->blocked_load_avg += p->se.avg.load_avg_contrib;
#endif
	}
}

void free_fair_sched_group(struct task_group *tg)
{
	int i;

	destroy_cfs_bandwidth(tg_cfs_bandwidth(tg));

	for_each_possible_cpu(i) {
		if (tg->cfs_rq)
			kfree(tg->cfs_rq[i]);
		if (tg->se)
			kfree(tg->se[i]);
	}

	kfree(tg->cfs_rq);
	kfree(tg->se);
}

int alloc_fair_sched_group(struct task_group *tg, struct task_group *parent)
{
	struct cfs_rq *cfs_rq;
	struct sched_entity *se;
	int i;

	tg->cfs_rq = kzalloc(sizeof(cfs_rq) * nr_cpu_ids, GFP_KERNEL);
	if (!tg->cfs_rq)
		goto err;
	tg->se = kzalloc(sizeof(se) * nr_cpu_ids, GFP_KERNEL);
	if (!tg->se)
		goto err;

	tg->shares = NICE_0_LOAD;

	init_cfs_bandwidth(tg_cfs_bandwidth(tg));

	for_each_possible_cpu(i) {
		cfs_rq = kzalloc_node(sizeof(struct cfs_rq),
				      GFP_KERNEL, cpu_to_node(i));
		if (!cfs_rq)
			goto err;

		se = kzalloc_node(sizeof(struct sched_entity),
				  GFP_KERNEL, cpu_to_node(i));
		if (!se)
			goto err_free_rq;

		init_cfs_rq(cfs_rq);
		init_tg_cfs_entry(tg, cfs_rq, se, i, parent->se[i]);
	}

	return 1;

err_free_rq:
	kfree(cfs_rq);
err:
	return 0;
}

void unregister_fair_sched_group(struct task_group *tg, int cpu)
{
	struct rq *rq = cpu_rq(cpu);
	unsigned long flags;

	if (!tg->cfs_rq[cpu]->on_list)
		return;

	raw_spin_lock_irqsave(&rq->lock, flags);
	list_del_leaf_cfs_rq(tg->cfs_rq[cpu]);
	raw_spin_unlock_irqrestore(&rq->lock, flags);
}

void init_tg_cfs_entry(struct task_group *tg, struct cfs_rq *cfs_rq,
			struct sched_entity *se, int cpu,
			struct sched_entity *parent)
{
	struct rq *rq = cpu_rq(cpu);

	cfs_rq->tg = tg;
	cfs_rq->rq = rq;
	init_cfs_rq_runtime(cfs_rq);

	tg->cfs_rq[cpu] = cfs_rq;
	tg->se[cpu] = se;

	
	if (!se)
		return;

	if (!parent)
		se->cfs_rq = &rq->cfs;
	else
		se->cfs_rq = parent->my_q;

	se->my_q = cfs_rq;
	
	update_load_set(&se->load, NICE_0_LOAD);
	se->parent = parent;
}

static DEFINE_MUTEX(shares_mutex);

int sched_group_set_shares(struct task_group *tg, unsigned long shares)
{
	int i;
	unsigned long flags;

	if (!tg->se[0])
		return -EINVAL;

	shares = clamp(shares, scale_load(MIN_SHARES), scale_load(MAX_SHARES));

	mutex_lock(&shares_mutex);
	if (tg->shares == shares)
		goto done;

	tg->shares = shares;
	for_each_possible_cpu(i) {
		struct rq *rq = cpu_rq(i);
		struct sched_entity *se;

		se = tg->se[i];
		
		raw_spin_lock_irqsave(&rq->lock, flags);
		for_each_sched_entity(se)
			update_cfs_shares(group_cfs_rq(se));
		raw_spin_unlock_irqrestore(&rq->lock, flags);
	}

done:
	mutex_unlock(&shares_mutex);
	return 0;
}
#else 

void free_fair_sched_group(struct task_group *tg) { }

int alloc_fair_sched_group(struct task_group *tg, struct task_group *parent)
{
	return 1;
}

void unregister_fair_sched_group(struct task_group *tg, int cpu) { }

#endif 


static unsigned int get_rr_interval_fair(struct rq *rq, struct task_struct *task)
{
	struct sched_entity *se = &task->se;
	unsigned int rr_interval = 0;

	if (rq->cfs.load.weight)
		rr_interval = NS_TO_JIFFIES(sched_slice(cfs_rq_of(se), se));

	return rr_interval;
}

const struct sched_class fair_sched_class = {
	.next			= &idle_sched_class,
	.enqueue_task		= enqueue_task_fair,
	.dequeue_task		= dequeue_task_fair,
	.yield_task		= yield_task_fair,
	.yield_to_task		= yield_to_task_fair,

	.check_preempt_curr	= check_preempt_wakeup,

	.pick_next_task		= pick_next_task_fair,
	.put_prev_task		= put_prev_task_fair,

#ifdef CONFIG_SMP
	.select_task_rq		= select_task_rq_fair,
#ifdef CONFIG_FAIR_GROUP_SCHED
	.migrate_task_rq	= migrate_task_rq_fair,
#endif
	.rq_online		= rq_online_fair,
	.rq_offline		= rq_offline_fair,

	.task_waking		= task_waking_fair,
#endif

	.set_curr_task          = set_curr_task_fair,
	.task_tick		= task_tick_fair,
	.task_fork		= task_fork_fair,

	.prio_changed		= prio_changed_fair,
	.switched_from		= switched_from_fair,
	.switched_to		= switched_to_fair,

	.get_rr_interval	= get_rr_interval_fair,

#ifdef CONFIG_FAIR_GROUP_SCHED
	.task_move_group	= task_move_group_fair,
#endif
};

#ifdef CONFIG_SCHED_DEBUG
void print_cfs_stats(struct seq_file *m, int cpu)
{
	struct cfs_rq *cfs_rq;

	rcu_read_lock();
	for_each_leaf_cfs_rq(cpu_rq(cpu), cfs_rq)
		print_cfs_rq(m, cpu, cfs_rq);
	rcu_read_unlock();
}
#endif

__init void init_sched_fair_class(void)
{
#ifdef CONFIG_SMP
	open_softirq(SCHED_SOFTIRQ, run_rebalance_domains);

#ifdef CONFIG_NO_HZ_COMMON
	nohz.next_balance = jiffies;
	zalloc_cpumask_var(&nohz.idle_cpus_mask, GFP_NOWAIT);
	cpu_notifier(sched_ilb_notifier, 0);
#endif
#endif 

}
