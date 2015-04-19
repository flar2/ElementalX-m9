
#include <linux/sched.h>
#include <linux/sched/sysctl.h>
#include <linux/sched/rt.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/stop_machine.h>
#include <linux/tick.h>

#include "cpupri.h"
#include "cpuacct.h"

extern __read_mostly int scheduler_running;

#define NICE_TO_PRIO(nice)	(MAX_RT_PRIO + (nice) + 20)
#define PRIO_TO_NICE(prio)	((prio) - MAX_RT_PRIO - 20)
#define TASK_NICE(p)		PRIO_TO_NICE((p)->static_prio)

#define USER_PRIO(p)		((p)-MAX_RT_PRIO)
#define TASK_USER_PRIO(p)	USER_PRIO((p)->static_prio)
#define MAX_USER_PRIO		(USER_PRIO(MAX_PRIO))

#define NS_TO_JIFFIES(TIME)	((unsigned long)(TIME) / (NSEC_PER_SEC / HZ))

#if 0 
# define SCHED_LOAD_RESOLUTION	10
# define scale_load(w)		((w) << SCHED_LOAD_RESOLUTION)
# define scale_load_down(w)	((w) >> SCHED_LOAD_RESOLUTION)
#else
# define SCHED_LOAD_RESOLUTION	0
# define scale_load(w)		(w)
# define scale_load_down(w)	(w)
#endif

#define SCHED_LOAD_SHIFT	(10 + SCHED_LOAD_RESOLUTION)
#define SCHED_LOAD_SCALE	(1L << SCHED_LOAD_SHIFT)

#define NICE_0_LOAD		SCHED_LOAD_SCALE
#define NICE_0_SHIFT		SCHED_LOAD_SHIFT

#define SCHED_LOAD_WINDOW_SIZE 10


#define RUNTIME_INF	((u64)~0ULL)

static inline int fair_policy(int policy)
{
	return policy == SCHED_NORMAL || policy == SCHED_BATCH;
}

static inline int rt_policy(int policy)
{
	return policy == SCHED_FIFO || policy == SCHED_RR;
}

static inline int task_has_rt_policy(struct task_struct *p)
{
	return rt_policy(p->policy);
}

struct rt_prio_array {
	DECLARE_BITMAP(bitmap, MAX_RT_PRIO+1); 
	struct list_head queue[MAX_RT_PRIO];
};

struct rt_bandwidth {
	
	raw_spinlock_t		rt_runtime_lock;
	ktime_t			rt_period;
	u64			rt_runtime;
	struct hrtimer		rt_period_timer;
};

extern struct mutex sched_domains_mutex;

#ifdef CONFIG_CGROUP_SCHED

#include <linux/cgroup.h>

struct cfs_rq;
struct rt_rq;

extern struct list_head task_groups;

struct cfs_bandwidth {
#ifdef CONFIG_CFS_BANDWIDTH
	raw_spinlock_t lock;
	ktime_t period;
	u64 quota, runtime;
	s64 hierarchal_quota;
	u64 runtime_expires;

	int idle, timer_active;
	struct hrtimer period_timer, slack_timer;
	struct list_head throttled_cfs_rq;

	
	int nr_periods, nr_throttled;
	u64 throttled_time;
#endif
};

struct task_group {
	struct cgroup_subsys_state css;

	bool notify_on_migrate;

#ifdef CONFIG_FAIR_GROUP_SCHED
	
	struct sched_entity **se;
	
	struct cfs_rq **cfs_rq;
	unsigned long shares;

	atomic_t load_weight;
	atomic64_t load_avg;
	atomic_t runnable_avg;
#endif

#ifdef CONFIG_RT_GROUP_SCHED
	struct sched_rt_entity **rt_se;
	struct rt_rq **rt_rq;

	struct rt_bandwidth rt_bandwidth;
#endif

	struct rcu_head rcu;
	struct list_head list;

	struct task_group *parent;
	struct list_head siblings;
	struct list_head children;

#ifdef CONFIG_SCHED_AUTOGROUP
	struct autogroup *autogroup;
#endif

	struct cfs_bandwidth cfs_bandwidth;
};

#ifdef CONFIG_FAIR_GROUP_SCHED
#define ROOT_TASK_GROUP_LOAD	NICE_0_LOAD

#define MIN_SHARES	(1UL <<  1)
#define MAX_SHARES	(1UL << 18)
#endif

typedef int (*tg_visitor)(struct task_group *, void *);

extern int walk_tg_tree_from(struct task_group *from,
			     tg_visitor down, tg_visitor up, void *data);

static inline int walk_tg_tree(tg_visitor down, tg_visitor up, void *data)
{
	return walk_tg_tree_from(&root_task_group, down, up, data);
}

extern int tg_nop(struct task_group *tg, void *data);

extern void free_fair_sched_group(struct task_group *tg);
extern int alloc_fair_sched_group(struct task_group *tg, struct task_group *parent);
extern void unregister_fair_sched_group(struct task_group *tg, int cpu);
extern void init_tg_cfs_entry(struct task_group *tg, struct cfs_rq *cfs_rq,
			struct sched_entity *se, int cpu,
			struct sched_entity *parent);
extern void init_cfs_bandwidth(struct cfs_bandwidth *cfs_b);
extern int sched_group_set_shares(struct task_group *tg, unsigned long shares);

extern void __refill_cfs_bandwidth_runtime(struct cfs_bandwidth *cfs_b);
extern void __start_cfs_bandwidth(struct cfs_bandwidth *cfs_b);
extern void unthrottle_cfs_rq(struct cfs_rq *cfs_rq);

extern void free_rt_sched_group(struct task_group *tg);
extern int alloc_rt_sched_group(struct task_group *tg, struct task_group *parent);
extern void init_tg_rt_entry(struct task_group *tg, struct rt_rq *rt_rq,
		struct sched_rt_entity *rt_se, int cpu,
		struct sched_rt_entity *parent);

extern struct task_group *sched_create_group(struct task_group *parent);
extern void sched_online_group(struct task_group *tg,
			       struct task_group *parent);
extern void sched_destroy_group(struct task_group *tg);
extern void sched_offline_group(struct task_group *tg);

extern void sched_move_task(struct task_struct *tsk);

#ifdef CONFIG_FAIR_GROUP_SCHED
extern int sched_group_set_shares(struct task_group *tg, unsigned long shares);
#endif

#else 

struct cfs_bandwidth { };

#endif	

struct cfs_rq {
	struct load_weight load;
	unsigned int nr_running, h_nr_running;

	u64 exec_clock;
	u64 min_vruntime;
#ifndef CONFIG_64BIT
	u64 min_vruntime_copy;
#endif

	struct rb_root tasks_timeline;
	struct rb_node *rb_leftmost;

	struct sched_entity *curr, *next, *last, *skip;

#ifdef	CONFIG_SCHED_DEBUG
	unsigned int nr_spread_over;
#endif

#ifdef CONFIG_SMP
#ifdef CONFIG_FAIR_GROUP_SCHED
	u64 runnable_load_avg, blocked_load_avg;
	atomic64_t decay_counter, removed_load;
	u64 last_decay;
#endif 
#ifdef CONFIG_FAIR_GROUP_SCHED
	u32 tg_runnable_contrib;
	u64 tg_load_contrib;
#endif 

	unsigned long h_load;
#endif 

#ifdef CONFIG_FAIR_GROUP_SCHED
	struct rq *rq;	

	int on_list;
	struct list_head leaf_cfs_rq_list;
	struct task_group *tg;	

#ifdef CONFIG_CFS_BANDWIDTH
	int runtime_enabled;
	u64 runtime_expires;
	s64 runtime_remaining;

	u64 throttled_clock, throttled_clock_task;
	u64 throttled_clock_task_time;
	int throttled, throttle_count;
	struct list_head throttled_list;
#endif 
#endif 
};

static inline int rt_bandwidth_enabled(void)
{
	return sysctl_sched_rt_runtime >= 0;
}

struct rt_rq {
	struct rt_prio_array active;
	unsigned int rt_nr_running;
#if defined CONFIG_SMP || defined CONFIG_RT_GROUP_SCHED
	struct {
		int curr; 
#ifdef CONFIG_SMP
		int next; 
#endif
	} highest_prio;
#endif
#ifdef CONFIG_SMP
	unsigned long rt_nr_migratory;
	unsigned long rt_nr_total;
	int overloaded;
	struct plist_head pushable_tasks;
#endif
	int rt_throttled;
	u64 rt_time;
	u64 rt_runtime;
	
	raw_spinlock_t rt_runtime_lock;

#ifdef CONFIG_RT_GROUP_SCHED
	unsigned long rt_nr_boosted;

	struct rq *rq;
	struct list_head leaf_rt_rq_list;
	struct task_group *tg;
#endif
};

#ifdef CONFIG_SMP

struct root_domain {
	atomic_t refcount;
	atomic_t rto_count;
	struct rcu_head rcu;
	cpumask_var_t span;
	cpumask_var_t online;

	cpumask_var_t rto_mask;
	struct cpupri cpupri;
};

extern struct root_domain def_root_domain;

#endif 

struct rq {
	
	raw_spinlock_t lock;

	unsigned int nr_running;
	#define CPU_LOAD_IDX_MAX 5
	unsigned long cpu_load[CPU_LOAD_IDX_MAX];
	unsigned long last_load_update_tick;
#ifdef CONFIG_NO_HZ_COMMON
	u64 nohz_stamp;
	unsigned long nohz_flags;
#endif
#ifdef CONFIG_NO_HZ_FULL
	unsigned long last_sched_tick;
#endif
	int skip_clock_update;

	
	struct load_weight load;
	unsigned long nr_load_updates;
	u64 nr_switches;

	struct cfs_rq cfs;
	struct rt_rq rt;

#ifdef CONFIG_FAIR_GROUP_SCHED
	
	struct list_head leaf_cfs_rq_list;
#ifdef CONFIG_SMP
	unsigned long h_load_throttle;
#endif 
#endif 

#ifdef CONFIG_RT_GROUP_SCHED
	struct list_head leaf_rt_rq_list;
#endif

	unsigned long nr_uninterruptible;

	struct task_struct *curr, *idle, *stop;
	unsigned long next_balance;
	struct mm_struct *prev_mm;

	u64 clock;
	u64 clock_task;

	atomic_t nr_iowait;

#ifdef CONFIG_SMP
	struct root_domain *rd;
	struct sched_domain *sd;

	unsigned long cpu_power;

	unsigned char idle_balance;
	
	int post_schedule;
	int active_balance;
	int push_cpu;
	struct task_struct *push_task;
	struct cpu_stop_work active_balance_work;
	
	int cpu;
	int online;

	struct list_head cfs_tasks;

	u64 rt_avg;
	u64 age_stamp;
	u64 idle_stamp;
	u64 avg_idle;
	int cstate, wakeup_latency, wakeup_energy;
#endif

#ifdef CONFIG_SCHED_HMP
	unsigned int cur_freq, max_freq, min_freq, max_possible_freq;
	struct cpumask freq_domain_cpumask;

	u64 cumulative_runnable_avg;
	int efficiency; 
	int load_scale_factor;
	int capacity;
	int max_possible_capacity;
	u64 window_start;
	int prefer_idle;
	u32 mostly_idle_load;
	int mostly_idle_nr_run;
	int mostly_idle_freq;

	u64 cur_irqload;
	u64 avg_irqload;
	u64 irqload_ts;

#ifdef CONFIG_SCHED_FREQ_INPUT
	unsigned int old_busy_time;
#endif
#endif

#ifdef CONFIG_SCHED_FREQ_INPUT
	u64 curr_runnable_sum;
	u64 prev_runnable_sum;
#endif

	u64 load_history[SCHED_LOAD_WINDOW_SIZE];
	int load_avg;
	int budget;
	int load_history_index;
	u64 load_last_update_timestamp;

#ifdef CONFIG_SCHED_HMP
	int nr_small_tasks, nr_big_tasks;
	unsigned long hmp_flags;
#endif

#ifdef CONFIG_IRQ_TIME_ACCOUNTING
	u64 prev_irq_time;
#endif
#ifdef CONFIG_PARAVIRT
	u64 prev_steal_time;
#endif
#ifdef CONFIG_PARAVIRT_TIME_ACCOUNTING
	u64 prev_steal_time_rq;
#endif

	
	unsigned long calc_load_update;
	long calc_load_active;

#ifdef CONFIG_SCHED_HRTICK
#ifdef CONFIG_SMP
	int hrtick_csd_pending;
	struct call_single_data hrtick_csd;
#endif
	struct hrtimer hrtick_timer;
#endif

#ifdef CONFIG_SCHEDSTATS
	
	struct sched_info rq_sched_info;
	unsigned long long rq_cpu_time;
	

	
	unsigned int yld_count;

	
	unsigned int sched_count;
	unsigned int sched_goidle;

	
	unsigned int ttwu_count;
	unsigned int ttwu_local;
#endif

#ifdef CONFIG_SMP
	struct llist_head wake_list;
#endif

	struct sched_avg avg;
};

static inline int cpu_of(struct rq *rq)
{
#ifdef CONFIG_SMP
	return rq->cpu;
#else
	return 0;
#endif
}

DECLARE_PER_CPU(struct rq, runqueues);

#define cpu_rq(cpu)		(&per_cpu(runqueues, (cpu)))
#define this_rq()		(&__get_cpu_var(runqueues))
#define task_rq(p)		cpu_rq(task_cpu(p))
#define cpu_curr(cpu)		(cpu_rq(cpu)->curr)
#define raw_rq()		(&__raw_get_cpu_var(runqueues))

#ifdef CONFIG_SMP

#define rcu_dereference_check_sched_domain(p) \
	rcu_dereference_check((p), \
			      lockdep_is_held(&sched_domains_mutex))

#define for_each_domain(cpu, __sd) \
	for (__sd = rcu_dereference_check_sched_domain(cpu_rq(cpu)->sd); \
			__sd; __sd = __sd->parent)

#define for_each_lower_domain(sd) for (; sd; sd = sd->child)

static inline struct sched_domain *highest_flag_domain(int cpu, int flag)
{
	struct sched_domain *sd, *hsd = NULL;

	for_each_domain(cpu, sd) {
		if (!(sd->flags & flag))
			break;
		hsd = sd;
	}

	return hsd;
}

DECLARE_PER_CPU(struct sched_domain *, sd_llc);
DECLARE_PER_CPU(int, sd_llc_id);

struct sched_group_power {
	atomic_t ref;
	unsigned int power, power_orig;
	unsigned long next_update;
	atomic_t nr_busy_cpus;

	unsigned long cpumask[0]; 
};

struct sched_group {
	struct sched_group *next;	
	atomic_t ref;

	unsigned int group_weight;
	struct sched_group_power *sgp;

	unsigned long cpumask[0];
};

static inline struct cpumask *sched_group_cpus(struct sched_group *sg)
{
	return to_cpumask(sg->cpumask);
}

static inline struct cpumask *sched_group_mask(struct sched_group *sg)
{
	return to_cpumask(sg->sgp->cpumask);
}

static inline unsigned int group_first_cpu(struct sched_group *group)
{
	return cpumask_first(sched_group_cpus(group));
}

extern int group_balance_cpu(struct sched_group *sg);

#define group_rq_capacity(group) capacity(cpu_rq(group_first_cpu(group)))

#endif 

#include "stats.h"
#include "auto_group.h"

extern void init_new_task_load(struct task_struct *p);

#ifdef CONFIG_SCHED_HMP

#define WINDOW_STATS_RECENT		0
#define WINDOW_STATS_MAX		1
#define WINDOW_STATS_MAX_RECENT_AVG	2
#define WINDOW_STATS_AVG		3
#define WINDOW_STATS_INVALID_POLICY	4

extern struct mutex policy_mutex;
extern unsigned int sched_ravg_window;
extern unsigned int sched_use_pelt;
extern unsigned int sched_disable_window_stats;
extern unsigned int max_possible_freq;
extern unsigned int min_max_freq;
extern unsigned int pct_task_load(struct task_struct *p);
extern unsigned int max_possible_efficiency;
extern unsigned int min_possible_efficiency;
extern unsigned int max_capacity;
extern unsigned int min_capacity;
extern unsigned int max_load_scale_factor;
extern unsigned long capacity_scale_cpu_efficiency(int cpu);
extern unsigned long capacity_scale_cpu_freq(int cpu);
extern unsigned int sched_mostly_idle_load;
extern unsigned int sched_small_task;
extern unsigned int sched_upmigrate;
extern unsigned int sched_downmigrate;
extern unsigned int sched_init_task_load_pelt;
extern unsigned int sched_init_task_load_windows;
extern unsigned int sched_heavy_task;

extern void fixup_nr_big_small_task(int cpu);
u64 scale_load_to_cpu(u64 load, int cpu);
unsigned int max_task_load(void);
extern void sched_account_irqtime(int cpu, struct task_struct *curr,
				 u64 delta, u64 wallclock);
unsigned int cpu_temp(int cpu);

static inline int capacity(struct rq *rq)
{
	return rq->capacity;
}

static inline void
inc_cumulative_runnable_avg(struct rq *rq, struct task_struct *p)
{
	if (sched_use_pelt)
		rq->cumulative_runnable_avg +=
				p->se.avg.runnable_avg_sum_scaled;
	else if (!sched_disable_window_stats)
		rq->cumulative_runnable_avg += p->ravg.demand;
}

static inline void
dec_cumulative_runnable_avg(struct rq *rq, struct task_struct *p)
{
	if (sched_use_pelt)
		rq->cumulative_runnable_avg -=
				p->se.avg.runnable_avg_sum_scaled;
	else if (!sched_disable_window_stats)
		rq->cumulative_runnable_avg -= p->ravg.demand;
	BUG_ON((s64)rq->cumulative_runnable_avg < 0);
}

#define pct_to_real(tunable)	\
		(div64_u64((u64)tunable * (u64)max_task_load(), 100))

#define real_to_pct(tunable)	\
		(div64_u64((u64)tunable * (u64)100, (u64)max_task_load()))

#define SCHED_HIGH_IRQ_TIMEOUT 3
static inline u64 sched_irqload(int cpu)
{
	struct rq *rq = cpu_rq(cpu);
	s64 delta;

	delta = get_jiffies_64() - rq->irqload_ts;

	if (delta < SCHED_HIGH_IRQ_TIMEOUT)
		return rq->avg_irqload;
	else
		return 0;
}

static inline int sched_cpu_high_irqload(int cpu)
{
	return sched_irqload(cpu) >= sysctl_sched_cpu_high_irqload;
}

#else	

static inline int pct_task_load(struct task_struct *p) { return 0; }

static inline int capacity(struct rq *rq)
{
	return SCHED_LOAD_SCALE;
}

static inline void
inc_cumulative_runnable_avg(struct rq *rq, struct task_struct *p)
{
}

static inline void
dec_cumulative_runnable_avg(struct rq *rq, struct task_struct *p)
{
}

static inline unsigned long capacity_scale_cpu_efficiency(int cpu)
{
	return SCHED_LOAD_SCALE;
}

static inline unsigned long capacity_scale_cpu_freq(int cpu)
{
	return SCHED_LOAD_SCALE;
}

static inline void sched_account_irqtime(int cpu, struct task_struct *curr,
				 u64 delta, u64 wallclock)
{
}

static inline int sched_cpu_high_irqload(int cpu) { return 0; }

#endif	

#ifdef CONFIG_SCHED_FREQ_INPUT
extern void check_for_freq_change(struct rq *rq);

static inline int same_freq_domain(int src_cpu, int dst_cpu)
{
	struct rq *rq = cpu_rq(src_cpu);

	if (src_cpu == dst_cpu)
		return 1;

	return cpumask_test_cpu(dst_cpu, &rq->freq_domain_cpumask);
}

#else	

#define sched_migration_fixup	0

static inline void check_for_freq_change(struct rq *rq) { }

static inline int same_freq_domain(int src_cpu, int dst_cpu)
{
	return 1;
}

#endif	

#ifdef CONFIG_SCHED_HMP

#define	BOOST_KICK	0
#define	CPU_RESERVED	1

static inline int is_reserved(int cpu)
{
	struct rq *rq = cpu_rq(cpu);

	return test_bit(CPU_RESERVED, &rq->hmp_flags);
}

static inline int mark_reserved(int cpu)
{
	struct rq *rq = cpu_rq(cpu);

	
	return test_and_set_bit(CPU_RESERVED, &rq->hmp_flags);
}

static inline void clear_reserved(int cpu)
{
	struct rq *rq = cpu_rq(cpu);

	clear_bit(CPU_RESERVED, &rq->hmp_flags);
}

extern unsigned int sched_enable_hmp;
extern unsigned int sched_enable_power_aware;

int mostly_idle_cpu(int cpu);
extern void check_for_migration(struct rq *rq, struct task_struct *p);
extern void pre_big_small_task_count_change(const struct cpumask *cpus);
extern void post_big_small_task_count_change(const struct cpumask *cpus);
extern void inc_nr_big_small_task(struct rq *rq, struct task_struct *p);
extern void dec_nr_big_small_task(struct rq *rq, struct task_struct *p);
extern void set_hmp_defaults(void);
extern unsigned int power_cost_at_freq(int cpu, unsigned int freq);
extern void reset_all_window_stats(u64 window_start, unsigned int window_size);
extern void boost_kick(int cpu);
extern int sched_boost(void);

#else 

#define sched_enable_hmp 0
#define sched_freq_legacy_mode 1

static inline void check_for_migration(struct rq *rq, struct task_struct *p) { }
static inline void pre_big_small_task_count_change(void) { }
static inline void post_big_small_task_count_change(void) { }
static inline void set_hmp_defaults(void) { }

static inline void inc_nr_big_small_task(struct rq *rq, struct task_struct *p)
{
}

static inline void dec_nr_big_small_task(struct rq *rq, struct task_struct *p)
{
}

static inline void clear_reserved(int cpu) { }

#define power_cost_at_freq(...) 0

#define trace_sched_cpu_load(...)

#endif 

#ifdef CONFIG_CGROUP_SCHED

static inline struct task_group *task_group(struct task_struct *p)
{
	return p->sched_task_group;
}

static inline bool task_notify_on_migrate(struct task_struct *p)
{
	return task_group(p)->notify_on_migrate;
}

static inline void set_task_rq(struct task_struct *p, unsigned int cpu)
{
#if defined(CONFIG_FAIR_GROUP_SCHED) || defined(CONFIG_RT_GROUP_SCHED)
	struct task_group *tg = task_group(p);
#endif

#ifdef CONFIG_FAIR_GROUP_SCHED
	p->se.cfs_rq = tg->cfs_rq[cpu];
	p->se.parent = tg->se[cpu];
#endif

#ifdef CONFIG_RT_GROUP_SCHED
	p->rt.rt_rq  = tg->rt_rq[cpu];
	p->rt.parent = tg->rt_se[cpu];
#endif
}

#else 

static inline void set_task_rq(struct task_struct *p, unsigned int cpu) { }
static inline struct task_group *task_group(struct task_struct *p)
{
	return NULL;
}
static inline bool task_notify_on_migrate(struct task_struct *p)
{
	return false;
}
#endif 

static inline void __set_task_cpu(struct task_struct *p, unsigned int cpu)
{
	set_task_rq(p, cpu);
#ifdef CONFIG_SMP
	smp_wmb();
	task_thread_info(p)->cpu = cpu;
#endif
}

#ifdef CONFIG_SCHED_DEBUG
# include <linux/static_key.h>
# define const_debug __read_mostly
#else
# define const_debug const
#endif

extern const_debug unsigned int sysctl_sched_features;

#define SCHED_FEAT(name, enabled)	\
	__SCHED_FEAT_##name ,

enum {
#include "features.h"
	__SCHED_FEAT_NR,
};

#undef SCHED_FEAT

#if defined(CONFIG_SCHED_DEBUG) && defined(HAVE_JUMP_LABEL)
static __always_inline bool static_branch__true(struct static_key *key)
{
	return static_key_true(key); 
}

static __always_inline bool static_branch__false(struct static_key *key)
{
	return static_key_false(key); 
}

#define SCHED_FEAT(name, enabled)					\
static __always_inline bool static_branch_##name(struct static_key *key) \
{									\
	return static_branch__##enabled(key);				\
}

#include "features.h"

#undef SCHED_FEAT

extern struct static_key sched_feat_keys[__SCHED_FEAT_NR];
#define sched_feat(x) (static_branch_##x(&sched_feat_keys[__SCHED_FEAT_##x]))
#else 
#define sched_feat(x) (sysctl_sched_features & (1UL << __SCHED_FEAT_##x))
#endif 

#ifdef CONFIG_NUMA_BALANCING
#define sched_feat_numa(x) sched_feat(x)
#ifdef CONFIG_SCHED_DEBUG
#define numabalancing_enabled sched_feat_numa(NUMA)
#else
extern bool numabalancing_enabled;
#endif 
#else
#define sched_feat_numa(x) (0)
#define numabalancing_enabled (0)
#endif 

static inline u64 global_rt_period(void)
{
	return (u64)sysctl_sched_rt_period * NSEC_PER_USEC;
}

static inline u64 global_rt_runtime(void)
{
	if (sysctl_sched_rt_runtime < 0)
		return RUNTIME_INF;

	return (u64)sysctl_sched_rt_runtime * NSEC_PER_USEC;
}



static inline int task_current(struct rq *rq, struct task_struct *p)
{
	return rq->curr == p;
}

static inline int task_running(struct rq *rq, struct task_struct *p)
{
#ifdef CONFIG_SMP
	return p->on_cpu;
#else
	return task_current(rq, p);
#endif
}


#ifndef prepare_arch_switch
# define prepare_arch_switch(next)	do { } while (0)
#endif
#ifndef finish_arch_switch
# define finish_arch_switch(prev)	do { } while (0)
#endif
#ifndef finish_arch_post_lock_switch
# define finish_arch_post_lock_switch()	do { } while (0)
#endif

#ifndef __ARCH_WANT_UNLOCKED_CTXSW
static inline void prepare_lock_switch(struct rq *rq, struct task_struct *next)
{
#ifdef CONFIG_SMP
	next->on_cpu = 1;
#endif
}

static inline void finish_lock_switch(struct rq *rq, struct task_struct *prev)
{
#ifdef CONFIG_SMP
	smp_wmb();
	prev->on_cpu = 0;
#endif
#ifdef CONFIG_DEBUG_SPINLOCK
	
	rq->lock.owner = current;
#endif
	spin_acquire(&rq->lock.dep_map, 0, 0, _THIS_IP_);

	raw_spin_unlock_irq(&rq->lock);
}

#else 
static inline void prepare_lock_switch(struct rq *rq, struct task_struct *next)
{
#ifdef CONFIG_SMP
	next->on_cpu = 1;
#endif
	raw_spin_unlock(&rq->lock);
}

static inline void finish_lock_switch(struct rq *rq, struct task_struct *prev)
{
#ifdef CONFIG_SMP
	smp_wmb();
	prev->on_cpu = 0;
#endif
	local_irq_enable();
}
#endif 

#define WF_SYNC		0x01		
#define WF_FORK		0x02		
#define WF_MIGRATED	0x4		

static inline void update_load_add(struct load_weight *lw, unsigned long inc)
{
	lw->weight += inc;
	lw->inv_weight = 0;
}

static inline void update_load_sub(struct load_weight *lw, unsigned long dec)
{
	lw->weight -= dec;
	lw->inv_weight = 0;
}

static inline void update_load_set(struct load_weight *lw, unsigned long w)
{
	lw->weight = w;
	lw->inv_weight = 0;
}


#define WEIGHT_IDLEPRIO                3
#define WMULT_IDLEPRIO         1431655765

static const int prio_to_weight[40] = {
      88761,     71755,     56483,     46273,     36291,
      29154,     23254,     18705,     14949,     11916,
       9548,      7620,      6100,      4904,      3906,
       3121,      2501,      1991,      1586,      1277,
       1024,       820,       655,       526,       423,
        335,       272,       215,       172,       137,
        110,        87,        70,        56,        45,
         36,        29,        23,        18,        15,
};

static const u32 prio_to_wmult[40] = {
      48388,     59856,     76040,     92818,    118348,
     147320,    184698,    229616,    287308,    360437,
     449829,    563644,    704093,    875809,   1099582,
    1376151,   1717300,   2157191,   2708050,   3363326,
    4194304,   5237765,   6557202,   8165337,  10153587,
   12820798,  15790321,  19976592,  24970740,  31350126,
   39045157,  49367440,  61356676,  76695844,  95443717,
  119304647, 148102320, 186737708, 238609294, 286331153,
};

#define ENQUEUE_WAKEUP		1
#define ENQUEUE_HEAD		2
#ifdef CONFIG_SMP
#define ENQUEUE_WAKING		4	
#else
#define ENQUEUE_WAKING		0
#endif

#define DEQUEUE_SLEEP		1

struct sched_class {
	const struct sched_class *next;

	void (*enqueue_task) (struct rq *rq, struct task_struct *p, int flags);
	void (*dequeue_task) (struct rq *rq, struct task_struct *p, int flags);
	void (*yield_task) (struct rq *rq);
	bool (*yield_to_task) (struct rq *rq, struct task_struct *p, bool preempt);

	void (*check_preempt_curr) (struct rq *rq, struct task_struct *p, int flags);

	struct task_struct * (*pick_next_task) (struct rq *rq);
	void (*put_prev_task) (struct rq *rq, struct task_struct *p);

#ifdef CONFIG_SMP
	int  (*select_task_rq)(struct task_struct *p, int sd_flag, int flags);
	void (*migrate_task_rq)(struct task_struct *p, int next_cpu);

	void (*pre_schedule) (struct rq *this_rq, struct task_struct *task);
	void (*post_schedule) (struct rq *this_rq);
	void (*task_waking) (struct task_struct *task);
	void (*task_woken) (struct rq *this_rq, struct task_struct *task);

	void (*set_cpus_allowed)(struct task_struct *p,
				 const struct cpumask *newmask);

	void (*rq_online)(struct rq *rq);
	void (*rq_offline)(struct rq *rq);
#endif

	void (*set_curr_task) (struct rq *rq);
	void (*task_tick) (struct rq *rq, struct task_struct *p, int queued);
	void (*task_fork) (struct task_struct *p);

	void (*switched_from) (struct rq *this_rq, struct task_struct *task);
	void (*switched_to) (struct rq *this_rq, struct task_struct *task);
	void (*prio_changed) (struct rq *this_rq, struct task_struct *task,
			     int oldprio);

	unsigned int (*get_rr_interval) (struct rq *rq,
					 struct task_struct *task);

#ifdef CONFIG_FAIR_GROUP_SCHED
	void (*task_move_group) (struct task_struct *p, int on_rq);
#endif
};

#define sched_class_highest (&stop_sched_class)
#define for_each_class(class) \
   for (class = sched_class_highest; class; class = class->next)

extern const struct sched_class stop_sched_class;
extern const struct sched_class rt_sched_class;
extern const struct sched_class fair_sched_class;
extern const struct sched_class idle_sched_class;


#ifdef CONFIG_SMP

extern void update_group_power(struct sched_domain *sd, int cpu);

extern void trigger_load_balance(struct rq *rq, int cpu);
extern void idle_balance(int this_cpu, struct rq *this_rq);

#if defined(CONFIG_FAIR_GROUP_SCHED)
extern void idle_enter_fair(struct rq *this_rq);
extern void idle_exit_fair(struct rq *this_rq);
#else
static inline void idle_enter_fair(struct rq *this_rq) {}
static inline void idle_exit_fair(struct rq *this_rq) {}
#endif

#else	

static inline void idle_balance(int cpu, struct rq *rq)
{
}

#endif

#ifdef CONFIG_SYSRQ_SCHED_DEBUG
extern void sysrq_sched_debug_show(void);
#endif
extern void sched_init_granularity(void);
extern void update_max_interval(void);
extern void init_sched_rt_class(void);
extern void init_sched_fair_class(void);

extern void resched_task(struct task_struct *p);
extern void resched_cpu(int cpu);

extern struct rt_bandwidth def_rt_bandwidth;
extern void init_rt_bandwidth(struct rt_bandwidth *rt_b, u64 period, u64 runtime);

extern void update_idle_cpu_load(struct rq *this_rq);

#ifdef CONFIG_PARAVIRT
static inline u64 steal_ticks(u64 steal)
{
	if (unlikely(steal > NSEC_PER_SEC))
		return div_u64(steal, TICK_NSEC);

	return __iter_div_u64_rem(steal, TICK_NSEC, &steal);
}
#endif

static inline void inc_nr_running(struct rq *rq)
{
	sched_update_nr_prod(cpu_of(rq), rq->nr_running, true);
	rq->nr_running++;

#ifdef CONFIG_NO_HZ_FULL
	if (rq->nr_running == 2) {
		if (tick_nohz_full_cpu(rq->cpu)) {
			
			smp_wmb();
			smp_send_reschedule(rq->cpu);
		}
       }
#endif
}

static inline void dec_nr_running(struct rq *rq)
{
	sched_update_nr_prod(cpu_of(rq), rq->nr_running, false);
	rq->nr_running--;
}

static inline void rq_last_tick_reset(struct rq *rq)
{
#ifdef CONFIG_NO_HZ_FULL
	rq->last_sched_tick = jiffies;
#endif
}

extern void update_rq_clock(struct rq *rq);

extern void activate_task(struct rq *rq, struct task_struct *p, int flags);
extern void deactivate_task(struct rq *rq, struct task_struct *p, int flags);

extern void check_preempt_curr(struct rq *rq, struct task_struct *p, int flags);

extern const_debug unsigned int sysctl_sched_time_avg;
extern const_debug unsigned int sysctl_sched_nr_migrate;
extern const_debug unsigned int sysctl_sched_migration_cost;

static inline u64 sched_avg_period(void)
{
	return (u64)sysctl_sched_time_avg * NSEC_PER_MSEC / 2;
}

#ifdef CONFIG_SCHED_HRTICK

static inline int hrtick_enabled(struct rq *rq)
{
	if (!sched_feat(HRTICK))
		return 0;
	if (!cpu_active(cpu_of(rq)))
		return 0;
	return hrtimer_is_hres_active(&rq->hrtick_timer);
}

void hrtick_start(struct rq *rq, u64 delay);

#else

static inline int hrtick_enabled(struct rq *rq)
{
	return 0;
}

#endif 

#ifdef CONFIG_SMP
extern void sched_avg_update(struct rq *rq);
static inline void sched_rt_avg_update(struct rq *rq, u64 rt_delta)
{
	rq->rt_avg += rt_delta;
	sched_avg_update(rq);
}
#else
static inline void sched_rt_avg_update(struct rq *rq, u64 rt_delta) { }
static inline void sched_avg_update(struct rq *rq) { }
#endif

extern void start_bandwidth_timer(struct hrtimer *period_timer, ktime_t period);

#ifdef CONFIG_SMP
#ifdef CONFIG_PREEMPT

static inline void double_rq_lock(struct rq *rq1, struct rq *rq2);

static inline int _double_lock_balance(struct rq *this_rq, struct rq *busiest)
	__releases(this_rq->lock)
	__acquires(busiest->lock)
	__acquires(this_rq->lock)
{
	raw_spin_unlock(&this_rq->lock);
	double_rq_lock(this_rq, busiest);

	return 1;
}

#else
static inline int _double_lock_balance(struct rq *this_rq, struct rq *busiest)
	__releases(this_rq->lock)
	__acquires(busiest->lock)
	__acquires(this_rq->lock)
{
	int ret = 0;

	if (unlikely(!raw_spin_trylock(&busiest->lock))) {
		if (busiest < this_rq) {
			raw_spin_unlock(&this_rq->lock);
			raw_spin_lock(&busiest->lock);
			raw_spin_lock_nested(&this_rq->lock,
					      SINGLE_DEPTH_NESTING);
			ret = 1;
		} else
			raw_spin_lock_nested(&busiest->lock,
					      SINGLE_DEPTH_NESTING);
	}
	return ret;
}

#endif 

static inline int double_lock_balance(struct rq *this_rq, struct rq *busiest)
{
	if (unlikely(!irqs_disabled())) {
		
		raw_spin_unlock(&this_rq->lock);
		BUG_ON(1);
	}

	return _double_lock_balance(this_rq, busiest);
}

static inline void double_unlock_balance(struct rq *this_rq, struct rq *busiest)
	__releases(busiest->lock)
{
	raw_spin_unlock(&busiest->lock);
	lock_set_subclass(&this_rq->lock.dep_map, 0, _RET_IP_);
}

static inline void double_rq_lock(struct rq *rq1, struct rq *rq2)
	__acquires(rq1->lock)
	__acquires(rq2->lock)
{
	BUG_ON(!irqs_disabled());
	if (rq1 == rq2) {
		raw_spin_lock(&rq1->lock);
		__acquire(rq2->lock);	
	} else {
		if (rq1 < rq2) {
			raw_spin_lock(&rq1->lock);
			raw_spin_lock_nested(&rq2->lock, SINGLE_DEPTH_NESTING);
		} else {
			raw_spin_lock(&rq2->lock);
			raw_spin_lock_nested(&rq1->lock, SINGLE_DEPTH_NESTING);
		}
	}
}

static inline void double_rq_unlock(struct rq *rq1, struct rq *rq2)
	__releases(rq1->lock)
	__releases(rq2->lock)
{
	raw_spin_unlock(&rq1->lock);
	if (rq1 != rq2)
		raw_spin_unlock(&rq2->lock);
	else
		__release(rq2->lock);
}

#else 

static inline void double_rq_lock(struct rq *rq1, struct rq *rq2)
	__acquires(rq1->lock)
	__acquires(rq2->lock)
{
	BUG_ON(!irqs_disabled());
	BUG_ON(rq1 != rq2);
	raw_spin_lock(&rq1->lock);
	__acquire(rq2->lock);	
}

static inline void double_rq_unlock(struct rq *rq1, struct rq *rq2)
	__releases(rq1->lock)
	__releases(rq2->lock)
{
	BUG_ON(rq1 != rq2);
	raw_spin_unlock(&rq1->lock);
	__release(rq2->lock);
}

#endif

extern struct sched_entity *__pick_first_entity(struct cfs_rq *cfs_rq);
extern struct sched_entity *__pick_last_entity(struct cfs_rq *cfs_rq);
extern void print_cfs_stats(struct seq_file *m, int cpu);
extern void print_rt_stats(struct seq_file *m, int cpu);

extern void init_cfs_rq(struct cfs_rq *cfs_rq);
extern void init_rt_rq(struct rt_rq *rt_rq, struct rq *rq);

extern void cfs_bandwidth_usage_inc(void);
extern void cfs_bandwidth_usage_dec(void);

#ifdef CONFIG_NO_HZ_COMMON
enum rq_nohz_flag_bits {
	NOHZ_TICK_STOPPED,
	NOHZ_BALANCE_KICK,
};

#define nohz_flags(cpu)	(&cpu_rq(cpu)->nohz_flags)
#endif

#define NOHZ_KICK_ANY 0
#define NOHZ_KICK_RESTRICT 1

#ifdef CONFIG_IRQ_TIME_ACCOUNTING

DECLARE_PER_CPU(u64, cpu_hardirq_time);
DECLARE_PER_CPU(u64, cpu_softirq_time);

#ifndef CONFIG_64BIT
DECLARE_PER_CPU(seqcount_t, irq_time_seq);

static inline void irq_time_write_begin(void)
{
	__this_cpu_inc(irq_time_seq.sequence);
	smp_wmb();
}

static inline void irq_time_write_end(void)
{
	smp_wmb();
	__this_cpu_inc(irq_time_seq.sequence);
}

static inline u64 irq_time_read(int cpu)
{
	u64 irq_time;
	unsigned seq;

	do {
		seq = read_seqcount_begin(&per_cpu(irq_time_seq, cpu));
		irq_time = per_cpu(cpu_softirq_time, cpu) +
			   per_cpu(cpu_hardirq_time, cpu);
	} while (read_seqcount_retry(&per_cpu(irq_time_seq, cpu), seq));

	return irq_time;
}
#else 
static inline void irq_time_write_begin(void)
{
}

static inline void irq_time_write_end(void)
{
}

static inline u64 irq_time_read(int cpu)
{
	return per_cpu(cpu_softirq_time, cpu) + per_cpu(cpu_hardirq_time, cpu);
}
#endif 
#endif 
