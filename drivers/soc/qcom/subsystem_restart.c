/* Copyright (c) 2011-2015, The Linux Foundation. All rights reserved.
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

#define pr_fmt(fmt) "subsys-restart: %s(): " fmt, __func__

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/list.h>
#include <linux/io.h>
#include <linux/kthread.h>
#include <linux/time.h>
#include <linux/wakelock.h>
#include <linux/suspend.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/idr.h>
#include <linux/debugfs.h>
#include <linux/interrupt.h>
#include <linux/of_gpio.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <soc/qcom/subsystem_restart.h>
#include <soc/qcom/subsystem_notif.h>
#include <soc/qcom/socinfo.h>
#include <soc/qcom/sysmon.h>

#include <asm/current.h>

#if defined(CONFIG_HTC_DEBUG_RIL_PCN0009_SSR_DUMP_TASK)
#include <linux/sched/rt.h>
#include <linux/htc_flags.h>
#endif

#define DISABLE_SSR 0x9889deed
static uint disable_restart_work;
module_param(disable_restart_work, uint, S_IRUGO | S_IWUSR);

static int enable_debug;
module_param(enable_debug, int, S_IRUGO | S_IWUSR);

#if defined(CONFIG_HTC_DEBUG_RIL_PCN0009_SSR_DUMP_TASK)
static uint disable_ssr_check_work = 0;
module_param(disable_ssr_check_work, uint, S_IRUGO | S_IWUSR);
#endif

enum p_subsys_state {
	SUBSYS_NORMAL,
	SUBSYS_CRASHED,
	SUBSYS_RESTARTING,
};

enum subsys_state {
	SUBSYS_OFFLINE,
	SUBSYS_ONLINE,
};

static const char * const subsys_states[] = {
	[SUBSYS_OFFLINE] = "OFFLINE",
	[SUBSYS_ONLINE] = "ONLINE",
};

static const char * const restart_levels[] = {
	[RESET_SOC] = "SYSTEM",
	[RESET_SUBSYS_COUPLED] = "RELATED",
};

#if defined(CONFIG_HTC_FEATURES_SSR)
static const char * const enable_ramdumps[] = {
	[DISABLE_RAMDUMP] = "DISABLE",
	[ENABLE_RAMDUMP] = "ENABLE",
};
#endif

#if defined(CONFIG_HTC_DEBUG_SSR)

#define SUBSYS_NAME_MAX_LENGTH 40
#define RD_BUF_SIZE			  256
#define MODEM_ERRMSG_LIST_LEN 10

struct msm_msr_info {
	int valid;
	struct timespec msr_time;
	char modem_errmsg[RD_BUF_SIZE];
};
int msm_msr_index = 0;
static struct msm_msr_info msr_info_list[MODEM_ERRMSG_LIST_LEN];

static ssize_t subsystem_restart_reason_nonblock_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{

	int i = 0;
	char tmp[RD_BUF_SIZE+30];

	for( i=0; i<MODEM_ERRMSG_LIST_LEN; i++ ) {
		if( msr_info_list[i].valid != 0 ) {
			
			snprintf(tmp, RD_BUF_SIZE+30, "%ld-%s|\n\r", msr_info_list[i].msr_time.tv_sec, msr_info_list[i].modem_errmsg);
			strcat(buf, tmp);
			memset(tmp, 0, RD_BUF_SIZE+30);
		}
		msr_info_list[i].valid = 0;
		memset(msr_info_list[i].modem_errmsg, 0, RD_BUF_SIZE);
	}
	strcat(buf, "\n\r\0");

	return strlen(buf);
}

void subsystem_restart_reason_nonblock_init(void)
{
	int i = 0;
	msm_msr_index = 0;
	for( i=0; i<MODEM_ERRMSG_LIST_LEN; i++ ) {
		msr_info_list[i].valid = 0;
		memset(msr_info_list[i].modem_errmsg, 0, RD_BUF_SIZE);
	}
}

#define subsystem_restart_ro_attr(_name) \
	static struct kobj_attribute _name##_attr = {  \
		.attr   = {                             \
			.name = __stringify(_name),     \
			.mode = 0444,                   \
		},                                      \
		.show   = _name##_show,                 \
		.store  = NULL,         \
	}


subsystem_restart_ro_attr(subsystem_restart_reason_nonblock);


static struct attribute *g[] = {
	&subsystem_restart_reason_nonblock_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = g,
};

#endif

#if defined(CONFIG_HTC_DEBUG_RIL_PCN0009_SSR_DUMP_TASK)
#define SSR_CHECK_TIMEOUT 5000
#define SSR_CHECK_COUNT 20
static void subsystem_restart_check_func(struct work_struct *work);
static inline void dump_busy_task(void);
static inline void dump_disk_sleep_task(void);
#endif

struct subsys_tracking {
	enum p_subsys_state p_state;
	spinlock_t s_lock;
	enum subsys_state state;
	struct mutex lock;
};

struct subsys_soc_restart_order {
	struct device_node **device_ptrs;
	int count;

	struct subsys_tracking track;
	struct subsys_device **subsys_ptrs;
	struct list_head list;
};

struct restart_log {
	struct timeval time;
	struct subsys_device *dev;
	struct list_head list;
};

struct subsys_device {
	struct subsys_desc *desc;
	struct work_struct work;
	struct wakeup_source ssr_wlock;
#if defined(CONFIG_HTC_DEBUG_RIL_PCN0009_SSR_DUMP_TASK)
	struct delayed_work ssr_check_wq;
#endif
	char wlname[64];
	struct work_struct device_restart_work;
	struct subsys_tracking track;

	void *notify;
	struct device dev;
	struct module *owner;
	int count;
	int id;
	int restart_level;
#if defined(CONFIG_HTC_FEATURES_SSR)
	int enable_ramdump;
#endif
#if defined(CONFIG_HTC_DEBUG_SSR)
#define HTC_DEBUG_SSR_REASON_LEN 80
	char restart_reason[HTC_DEBUG_SSR_REASON_LEN];
#endif
	int crash_count;
	struct subsys_soc_restart_order *restart_order;
#ifdef CONFIG_DEBUG_FS
	struct dentry *dentry;
#endif
	bool do_ramdump_on_put;
	struct cdev char_dev;
	dev_t dev_no;
	struct completion err_ready;
	bool crashed;
	int notif_state;
	struct list_head list;
};

#if defined(CONFIG_HTC_DEBUG_RIL_PCN0009_SSR_DUMP_TASK)
static inline void dump_busy_task(void)
{
	struct task_struct *g, *p;
	struct timespec ts;
	
	ts = ktime_to_timespec(ktime_get());
	printk("Scan busy tasks =======START ========== ktime_get = %d.%d \n", (int)ts.tv_sec, (int)ts.tv_nsec);
	do_each_thread(g, p) {
		if(p->state == TASK_RUNNING)
		{
			if(rt_task(p))
				printk("RT task: %-15.15s ", p->comm);
			else
				printk("%-15.15s ", p->comm);
			printk("%5d %6d 0x%08lx \n", task_pid_nr(p), task_pid_nr(rcu_dereference(p->real_parent)), (unsigned long)task_thread_info(p)->flags);
			show_stack(p, NULL);
		}
	} while_each_thread(g, p);
	printk("Scan busy tasks ======END=========== \n");
	printk(" Current task is %-15.15s \n", current->comm);
	show_stack(current, NULL);
}

static inline void dump_disk_sleep_task(void)
{
	struct task_struct *g, *p;
	struct timespec ts;
	
	ts = ktime_to_timespec(ktime_get());
	printk("Scan disk sleep tasks =======START ========== ktime_get = %d.%d \n", (int)ts.tv_sec, (int)ts.tv_nsec);
	do_each_thread(g, p) {
		if(p->state == TASK_UNINTERRUPTIBLE)
		{
			if(rt_task(p))
				printk("RT task: %-15.15s ", p->comm);
			else
				printk("%-15.15s ", p->comm);
			printk("%5d %6d 0x%08lx \n", task_pid_nr(p), task_pid_nr(rcu_dereference(p->real_parent)), (unsigned long)task_thread_info(p)->flags);
			show_stack(p, NULL);
		}
	} while_each_thread(g, p);
	printk("Scan disk sleep tasks ======END=========== \n");
	printk(" Current task is %-15.15s \n", current->comm);
	show_stack(current, NULL);
}

static void subsystem_restart_check_func(struct work_struct *work)
{
	struct delayed_work* ssr_check_wq = to_delayed_work(work);

	struct subsys_device* dev = container_of(ssr_check_wq,
						struct subsys_device, ssr_check_wq);

	struct subsys_soc_restart_order *order = dev->restart_order;
	struct subsys_desc *desc = dev->desc;
	struct subsys_tracking *track;
	int dump_busy_task_count = 0;

	if (strcmp(desc->name, "modem")) {
		printk("[<%p>][%s]: Not modem SSR, ssr device name[%s]\n", current, __func__, desc->name);
	}

	printk("[<%p>][%s]: Check Busy task start.\n", current, __func__);

	if (order) {
		track = &order->track;
	} else {
		track = &dev->track;
	}

	for (dump_busy_task_count = 0; dump_busy_task_count < SSR_CHECK_COUNT; dump_busy_task_count++) {
		printk("[<%p>][%s]: dump_busy_task_count=[%d], track->p_state=[%d]\n", current, __func__, dump_busy_task_count, track->p_state);
		if ( track->p_state == SUBSYS_NORMAL ) {
			break;
		}
		dump_busy_task();
		dump_disk_sleep_task();
		msleep(1000);
	}

	if ( track->p_state != SUBSYS_NORMAL ) {
		printk("[<%p>][%s]: %s SSR timeout over 25 secs, track->p_state=[%d].", current, __func__, desc->name, track->p_state);
		if ( get_radio_flag() & BIT(3) ) {
			panic("[<%p>][%s]: %s SSR timeout over 25 secs.", current, __func__, desc->name);
		}
	}

	printk("[<%p>][%s]: Check Busy task end.\n", current, __func__);

}
#endif

static struct subsys_device *to_subsys(struct device *d)
{
	return container_of(d, struct subsys_device, dev);
}

static struct subsys_tracking *subsys_get_track(struct subsys_device *subsys)
{
	struct subsys_soc_restart_order *order = subsys->restart_order;

	if (order)
		return &order->track;
	else
		return &subsys->track;
}

static ssize_t name_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", to_subsys(dev)->desc->name);
}

static ssize_t state_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	enum subsys_state state = to_subsys(dev)->track.state;
	return snprintf(buf, PAGE_SIZE, "%s\n", subsys_states[state]);
}

static ssize_t crash_count_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", to_subsys(dev)->crash_count);
}

static ssize_t
restart_level_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int level = to_subsys(dev)->restart_level;
	return snprintf(buf, PAGE_SIZE, "%s\n", restart_levels[level]);
}

static ssize_t restart_level_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct subsys_device *subsys = to_subsys(dev);
	int i;
	const char *p;

	p = memchr(buf, '\n', count);
	if (p)
		count = p - buf;

	for (i = 0; i < ARRAY_SIZE(restart_levels); i++)
		if (!strncasecmp(buf, restart_levels[i], count)) {
			subsys->restart_level = i;
			return count;
		}
	return -EPERM;
}

#if defined(CONFIG_HTC_FEATURES_SSR)
static ssize_t enable_ramdump_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int enable_ramdump = to_subsys(dev)->enable_ramdump;
	return snprintf(buf, PAGE_SIZE, "%s\n", enable_ramdumps[enable_ramdump]);
}

static ssize_t enable_ramdump_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct subsys_device *subsys = to_subsys(dev);
	int i;
	const char *p;

	p = memchr(buf, '\n', count);
	if (p)
		count = p - buf;

	for (i = 0; i < ARRAY_SIZE(enable_ramdumps); i++)
		if (!strncasecmp(buf, enable_ramdumps[i], count)) {
			subsys->enable_ramdump = i;
			return count;
		}

		return -EPERM;
}

void subsys_set_enable_ramdump(struct subsys_device *dev, int enable)
{
	dev->enable_ramdump = enable;
}
EXPORT_SYMBOL(subsys_set_enable_ramdump);

void subsys_set_restart_level(struct subsys_device *dev, int level)
{
	dev->restart_level = level;
}
EXPORT_SYMBOL(subsys_set_restart_level);
#endif

static ssize_t system_debug_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct subsys_device *subsys = to_subsys(dev);
	char p[6] = "set";

	if (!subsys->desc->system_debug)
		strlcpy(p, "reset", sizeof(p));

	return snprintf(buf, PAGE_SIZE, "%s\n", p);
}

static ssize_t system_debug_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct subsys_device *subsys = to_subsys(dev);

	if (!strncasecmp(buf, "set", count))
		subsys->desc->system_debug = true;
	else if (!strncasecmp(buf, "reset", count))
		subsys->desc->system_debug = false;
	else
		return -EPERM;
	return count;
}

static ssize_t
firmware_name_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", to_subsys(dev)->desc->fw_name);
}

static ssize_t firmware_name_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct subsys_device *subsys = to_subsys(dev);
	struct subsys_tracking *track = subsys_get_track(subsys);
	const char *p;

	p = memchr(buf, '\n', count);
	if (p)
		count = p - buf;
	if (!count)
		return -EPERM;

	pr_info("Changing subsys fw_name to %s\n", buf);
	mutex_lock(&track->lock);
	strlcpy(subsys->desc->fw_name, buf, count + 1);
	mutex_unlock(&track->lock);
	return count;
}

int subsys_get_restart_level(struct subsys_device *dev)
{
	return dev->restart_level;
}
EXPORT_SYMBOL(subsys_get_restart_level);

#if defined(CONFIG_HTC_DEBUG_SSR)
void subsys_set_restart_reason(struct subsys_device *dev, const char* reason)
{
	if (!dev || !reason)
		return;
	snprintf(dev->restart_reason, sizeof(dev->restart_reason) - 1, "%s", reason);
}
EXPORT_SYMBOL(subsys_set_restart_reason);
#endif 

static ssize_t crashed_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
    bool crashed = to_subsys(dev)->crashed;
    if (crashed)
	    return snprintf(buf, PAGE_SIZE, "%s\n", "TRUE");
    else
        return snprintf(buf, PAGE_SIZE, "%s\n", "FALSE");
}

static ssize_t crashed_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct subsys_device *subsys = to_subsys(dev);
	const char *p;

	p = memchr(buf, '\n', count);
	if (p)
		count = p - buf;

	if (!strncasecmp(buf, "FALSE", count)) {
			subsys->crashed = false;
			return count;
	}
	return -EPERM;
}

static void subsys_set_state(struct subsys_device *subsys,
			     enum subsys_state state)
{
	unsigned long flags;

	spin_lock_irqsave(&subsys->track.s_lock, flags);
	if (subsys->track.state != state) {
		subsys->track.state = state;
		spin_unlock_irqrestore(&subsys->track.s_lock, flags);
		sysfs_notify(&subsys->dev.kobj, NULL, "state");
		return;
	}
	spin_unlock_irqrestore(&subsys->track.s_lock, flags);
}

void subsys_default_online(struct subsys_device *dev)
{
	subsys_set_state(dev, SUBSYS_ONLINE);
}
EXPORT_SYMBOL(subsys_default_online);

static struct device_attribute subsys_attrs[] = {
	__ATTR_RO(name),
	__ATTR_RO(state),
	__ATTR_RO(crash_count),
#if defined(CONFIG_HTC_FEATURES_SSR)
	__ATTR(enable_ramdump, 0644, enable_ramdump_show, enable_ramdump_store),
#endif
	__ATTR(crashed, 0644, crashed_show, crashed_store),
	__ATTR(restart_level, 0644, restart_level_show, restart_level_store),
	__ATTR(system_debug, 0644, system_debug_show, system_debug_store),
	__ATTR(firmware_name, 0644, firmware_name_show, firmware_name_store),
	__ATTR_NULL,
};

static struct bus_type subsys_bus_type = {
	.name		= "msm_subsys",
	.dev_attrs	= subsys_attrs,
};

static DEFINE_IDA(subsys_ida);

#if !defined(CONFIG_HTC_FEATURES_SSR)
static int enable_ramdumps;
module_param(enable_ramdumps, int, S_IRUGO | S_IWUSR);
#endif

struct workqueue_struct *ssr_wq;
static struct class *char_class;

static LIST_HEAD(restart_log_list);
static LIST_HEAD(subsys_list);
static LIST_HEAD(ssr_order_list);
static DEFINE_MUTEX(soc_order_reg_lock);
static DEFINE_MUTEX(restart_log_mutex);
static DEFINE_MUTEX(subsys_list_lock);
static DEFINE_MUTEX(char_device_lock);
static DEFINE_MUTEX(ssr_order_mutex);

static struct subsys_soc_restart_order *
update_restart_order(struct subsys_device *dev)
{
	int i;
	struct subsys_soc_restart_order *order;
	struct device_node *device = dev->desc->dev->of_node;

	mutex_lock(&soc_order_reg_lock);
	list_for_each_entry(order, &ssr_order_list, list) {
		for (i = 0; i < order->count; i++) {
			if (order->device_ptrs[i] == device) {
				order->subsys_ptrs[i] = dev;
				goto found;
			}
		}
	}
	order = NULL;
found:
	mutex_unlock(&soc_order_reg_lock);

	return order;
}

static int max_restarts;
module_param(max_restarts, int, 0644);

static long max_history_time = 3600;
module_param(max_history_time, long, 0644);

static void do_epoch_check(struct subsys_device *dev)
{
	int n = 0;
	struct timeval *time_first = NULL, *curr_time;
	struct restart_log *r_log, *temp;
	static int max_restarts_check;
	static long max_history_time_check;

	mutex_lock(&restart_log_mutex);

	max_restarts_check = max_restarts;
	max_history_time_check = max_history_time;

	
	if (!max_restarts_check)
		goto out;

	r_log = kmalloc(sizeof(struct restart_log), GFP_KERNEL);
	if (!r_log)
		goto out;
	r_log->dev = dev;
	do_gettimeofday(&r_log->time);
	curr_time = &r_log->time;
	INIT_LIST_HEAD(&r_log->list);

	list_add_tail(&r_log->list, &restart_log_list);

	list_for_each_entry_safe(r_log, temp, &restart_log_list, list) {

		if ((curr_time->tv_sec - r_log->time.tv_sec) >
				max_history_time_check) {

			pr_debug("Deleted node with restart_time = %ld\n",
					r_log->time.tv_sec);
			list_del(&r_log->list);
			kfree(r_log);
			continue;
		}
		if (!n) {
			time_first = &r_log->time;
			pr_debug("Time_first: %ld\n", time_first->tv_sec);
		}
		n++;
		pr_debug("Restart_time: %ld\n", r_log->time.tv_sec);
	}

	if (time_first && n >= max_restarts_check) {
		if ((curr_time->tv_sec - time_first->tv_sec) <
				max_history_time_check)
			panic("Subsystems have crashed %d times in less than "
				"%ld seconds!", max_restarts_check,
				max_history_time_check);
	}

out:
	mutex_unlock(&restart_log_mutex);
}

#if !defined(CONFIG_HTC_FEATURES_SSR)
static int is_ramdump_enabled(struct subsys_device *dev)
{
	if (dev->desc->ramdump_disable_gpio)
		return !dev->desc->ramdump_disable;

	return enable_ramdumps;
}
#endif

static void send_sysmon_notif(struct subsys_device *dev)
{
	struct subsys_device *subsys;

	mutex_lock(&subsys_list_lock);
	list_for_each_entry(subsys, &subsys_list, list)
		if ((subsys->notif_state > 0) && (subsys != dev))
			sysmon_send_event(dev->desc, subsys->desc,
						subsys->notif_state);
	mutex_unlock(&subsys_list_lock);
}

static void for_each_subsys_device(struct subsys_device **list, unsigned count,
		void *data, void (*fn)(struct subsys_device *, void *))
{
	while (count--) {
		struct subsys_device *dev = *list++;
		if (!dev)
			continue;
		fn(dev, data);
	}
}

static void notify_each_subsys_device(struct subsys_device **list,
		unsigned count,
		enum subsys_notif_type notif, void *data)
{
	struct subsys_device *subsys;

	while (count--) {
		struct subsys_device *dev = *list++;
		struct notif_data notif_data;
		struct platform_device *pdev;

		if (!dev)
			continue;

		pdev = container_of(dev->desc->dev, struct platform_device,
									dev);
		dev->notif_state = notif;

		mutex_lock(&subsys_list_lock);
		list_for_each_entry(subsys, &subsys_list, list)
			if (dev != subsys &&
				subsys->track.state == SUBSYS_ONLINE)
				sysmon_send_event(subsys->desc, dev->desc,
								notif);
		mutex_unlock(&subsys_list_lock);

		if (notif == SUBSYS_AFTER_POWERUP &&
				dev->track.state == SUBSYS_ONLINE)
			send_sysmon_notif(dev);

		notif_data.crashed = subsys_get_crash_status(dev);
#if !defined(CONFIG_HTC_FEATURES_SSR)
		notif_data.enable_ramdump = is_ramdump_enabled(dev);
#endif
		notif_data.no_auth = dev->desc->no_auth;
		notif_data.pdev = pdev;

		subsys_notif_queue_notification(dev->notify, notif,
								&notif_data);
	}
}

static void enable_all_irqs(struct subsys_device *dev)
{
	if (dev->desc->err_ready_irq)
		enable_irq(dev->desc->err_ready_irq);
	if (dev->desc->wdog_bite_irq && dev->desc->wdog_bite_handler) {
		enable_irq(dev->desc->wdog_bite_irq);
		irq_set_irq_wake(dev->desc->wdog_bite_irq, 1);
	}
	if (dev->desc->err_fatal_irq && dev->desc->err_fatal_handler)
		enable_irq(dev->desc->err_fatal_irq);
	if (dev->desc->stop_ack_irq && dev->desc->stop_ack_handler)
		enable_irq(dev->desc->stop_ack_irq);
}

static void disable_all_irqs(struct subsys_device *dev)
{
	if (dev->desc->err_ready_irq)
		disable_irq(dev->desc->err_ready_irq);
	if (dev->desc->wdog_bite_irq && dev->desc->wdog_bite_handler) {
		disable_irq(dev->desc->wdog_bite_irq);
		irq_set_irq_wake(dev->desc->wdog_bite_irq, 0);
	}
	if (dev->desc->err_fatal_irq && dev->desc->err_fatal_handler)
		disable_irq(dev->desc->err_fatal_irq);
	if (dev->desc->stop_ack_irq && dev->desc->stop_ack_handler)
		disable_irq(dev->desc->stop_ack_irq);
}

static int wait_for_err_ready(struct subsys_device *subsys)
{
	int ret;

	if (!subsys->desc->err_ready_irq || enable_debug == 1)
		return 0;

	ret = wait_for_completion_timeout(&subsys->err_ready,
					  msecs_to_jiffies(10000));
	if (!ret) {
		pr_err("[%s]: Error ready timed out\n", subsys->desc->name);
		return -ETIMEDOUT;
	}

	return 0;
}

static void subsystem_shutdown(struct subsys_device *dev, void *data)
{
	const char *name = dev->desc->name;

	pr_info("[%p]: Shutting down %s\n", current, name);
	if (dev->desc->shutdown(dev->desc, true) < 0)
		panic("subsys-restart: [%p]: Failed to shutdown %s!",
			current, name);
	dev->crash_count++;
	subsys_set_state(dev, SUBSYS_OFFLINE);
	disable_all_irqs(dev);
}

static void subsystem_ramdump(struct subsys_device *dev, void *data)
{
	const char *name = dev->desc->name;

	if (dev->desc->ramdump)
#if defined(CONFIG_HTC_FEATURES_SSR)
		if (dev->desc->ramdump(dev->enable_ramdump, dev->desc) < 0)
#else
		if (dev->desc->ramdump(is_ramdump_enabled(dev), dev->desc) < 0)
#endif
			pr_warn("%s[%p]: Ramdump failed.\n", name, current);
	dev->do_ramdump_on_put = false;
}

static void subsystem_free_memory(struct subsys_device *dev, void *data)
{
	if (dev->desc->free_memory)
		dev->desc->free_memory(dev->desc);
}

static void subsystem_powerup(struct subsys_device *dev, void *data)
{
	const char *name = dev->desc->name;
	int ret;

	pr_info("[%p]: Powering up %s\n", current, name);
	init_completion(&dev->err_ready);

	if (dev->desc->powerup(dev->desc) < 0) {
		notify_each_subsys_device(&dev, 1, SUBSYS_POWERUP_FAILURE,
								NULL);
		panic("[%p]: Powerup error: %s!", current, name);
	}
	enable_all_irqs(dev);

	ret = wait_for_err_ready(dev);
	if (ret) {
		notify_each_subsys_device(&dev, 1, SUBSYS_POWERUP_FAILURE,
								NULL);
		panic("[%p]: Timed out waiting for error ready: %s!",
			current, name);
	}
	subsys_set_state(dev, SUBSYS_ONLINE);
	subsys_set_crash_status(dev, false);
}

static int __find_subsys(struct device *dev, void *data)
{
	struct subsys_device *subsys = to_subsys(dev);
	return !strcmp(subsys->desc->name, data);
}

static struct subsys_device *find_subsys(const char *str)
{
	struct device *dev;

	if (!str)
		return NULL;

	dev = bus_find_device(&subsys_bus_type, NULL, (void *)str,
			__find_subsys);
	return dev ? to_subsys(dev) : NULL;
}

static int subsys_start(struct subsys_device *subsys)
{
	int ret;

	notify_each_subsys_device(&subsys, 1, SUBSYS_BEFORE_POWERUP,
								NULL);

	init_completion(&subsys->err_ready);
	ret = subsys->desc->powerup(subsys->desc);
	if (ret) {
		notify_each_subsys_device(&subsys, 1, SUBSYS_POWERUP_FAILURE,
									NULL);
		return ret;
	}
	enable_all_irqs(subsys);

	if (subsys->desc->is_not_loadable) {
		subsys_set_state(subsys, SUBSYS_ONLINE);
		return 0;
	}

	ret = wait_for_err_ready(subsys);
	if (ret) {
		notify_each_subsys_device(&subsys, 1, SUBSYS_POWERUP_FAILURE,
									NULL);
		subsys->desc->shutdown(subsys->desc, false);
		disable_all_irqs(subsys);
		return ret;
	} else {
		subsys_set_state(subsys, SUBSYS_ONLINE);
	}

	notify_each_subsys_device(&subsys, 1, SUBSYS_AFTER_POWERUP,
								NULL);
	return ret;
}

static void subsys_stop(struct subsys_device *subsys)
{
	const char *name = subsys->desc->name;

	subsys->desc->sysmon_shutdown_ret = sysmon_send_shutdown(subsys->desc);
	if (subsys->desc->sysmon_shutdown_ret)
		pr_debug("Graceful shutdown failed for %s\n", name);

	notify_each_subsys_device(&subsys, 1, SUBSYS_BEFORE_SHUTDOWN, NULL);
	subsys->desc->shutdown(subsys->desc, false);
	subsys_set_state(subsys, SUBSYS_OFFLINE);
	disable_all_irqs(subsys);
	notify_each_subsys_device(&subsys, 1, SUBSYS_AFTER_SHUTDOWN, NULL);
}

void *__subsystem_get(const char *name, const char *fw_name)
{
	struct subsys_device *subsys;
	struct subsys_device *subsys_d;
	int ret;
	void *retval;
	struct subsys_tracking *track;

	if (!name)
		return NULL;

	subsys = retval = find_subsys(name);
	if (!subsys)
		return ERR_PTR(-ENODEV);
	if (!try_module_get(subsys->owner)) {
		retval = ERR_PTR(-ENODEV);
		goto err_module;
	}

	subsys_d = subsystem_get(subsys->desc->depends_on);
	if (IS_ERR(subsys_d)) {
		retval = subsys_d;
		goto err_depends;
	}

	track = subsys_get_track(subsys);
	mutex_lock(&track->lock);
	if (!subsys->count) {
		if (fw_name) {
			pr_info("Changing subsys fw_name to %s\n", fw_name);
			strlcpy(subsys->desc->fw_name, fw_name,
				sizeof(subsys->desc->fw_name));
		}
		ret = subsys_start(subsys);
		if (ret) {
			retval = ERR_PTR(ret);
			goto err_start;
		}
	}
	subsys->count++;
	mutex_unlock(&track->lock);
	return retval;
err_start:
	mutex_unlock(&track->lock);
	subsystem_put(subsys_d);
err_depends:
	module_put(subsys->owner);
err_module:
	put_device(&subsys->dev);
	return retval;
}

void *subsystem_get(const char *name)
{
	return __subsystem_get(name, NULL);
}
EXPORT_SYMBOL(subsystem_get);

void *subsystem_get_with_fwname(const char *name, const char *fw_name)
{
	return __subsystem_get(name, fw_name);
}
EXPORT_SYMBOL(subsystem_get_with_fwname);

void subsystem_put(void *subsystem)
{
	struct subsys_device *subsys_d, *subsys = subsystem;
	struct subsys_tracking *track;

	if (IS_ERR_OR_NULL(subsys))
		return;

	track = subsys_get_track(subsys);
	mutex_lock(&track->lock);
	if (WARN(!subsys->count, "%s: %s: Reference count mismatch\n",
			subsys->desc->name, __func__))
		goto err_out;
	if (!--subsys->count) {
		subsys_stop(subsys);
		if (subsys->do_ramdump_on_put)
			subsystem_ramdump(subsys, NULL);
		subsystem_free_memory(subsys, NULL);
	}
	mutex_unlock(&track->lock);

	subsys_d = find_subsys(subsys->desc->depends_on);
	if (subsys_d) {
		subsystem_put(subsys_d);
		put_device(&subsys_d->dev);
	}
	module_put(subsys->owner);
	put_device(&subsys->dev);
	return;
err_out:
	mutex_unlock(&track->lock);
}
EXPORT_SYMBOL(subsystem_put);

static void subsystem_restart_wq_func(struct work_struct *work)
{
	struct subsys_device *dev = container_of(work,
						struct subsys_device, work);
	struct subsys_device **list;
	struct subsys_desc *desc = dev->desc;
	struct subsys_soc_restart_order *order = dev->restart_order;
	struct subsys_tracking *track;
	unsigned count;
	unsigned long flags;

	if (order) {
		list = order->subsys_ptrs;
		count = order->count;
		track = &order->track;
	} else {
		list = &dev;
		count = 1;
		track = &dev->track;
	}

	mutex_lock(&track->lock);
	do_epoch_check(dev);

	mutex_lock(&soc_order_reg_lock);

	pr_debug("[%p]: Starting restart sequence for %s\n", current,
			desc->name);

#if defined(CONFIG_HTC_DEBUG_RIL_PCN0009_SSR_DUMP_TASK)
	if ( disable_ssr_check_work == 0 &&
		!strcmp(desc->name, "modem")) {
		printk("[<%p>][%s]: schedule ssr check work.\n", current, __func__);
		schedule_delayed_work(&dev->ssr_check_wq, msecs_to_jiffies(SSR_CHECK_TIMEOUT));
	}
#endif

	notify_each_subsys_device(list, count, SUBSYS_BEFORE_SHUTDOWN, NULL);
	for_each_subsys_device(list, count, NULL, subsystem_shutdown);
	notify_each_subsys_device(list, count, SUBSYS_AFTER_SHUTDOWN, NULL);

#if defined(CONFIG_HTC_FEATURES_SSR)
	notify_each_subsys_device(list, count, SUBSYS_RAMDUMP_NOTIFICATION,&(dev->enable_ramdump));
#else
	notify_each_subsys_device(list, count, SUBSYS_RAMDUMP_NOTIFICATION,
									NULL);
#endif

	spin_lock_irqsave(&track->s_lock, flags);
	track->p_state = SUBSYS_RESTARTING;
	spin_unlock_irqrestore(&track->s_lock, flags);

	
	for_each_subsys_device(list, count, NULL, subsystem_ramdump);

	for_each_subsys_device(list, count, NULL, subsystem_free_memory);

	notify_each_subsys_device(list, count, SUBSYS_BEFORE_POWERUP, NULL);
	for_each_subsys_device(list, count, NULL, subsystem_powerup);
	notify_each_subsys_device(list, count, SUBSYS_AFTER_POWERUP, NULL);

#if defined(CONFIG_HTC_DEBUG_RIL_PCN0009_SSR_DUMP_TASK)
	if ( disable_ssr_check_work == 0 &&
		!strcmp(desc->name, "modem")) {
		printk("[<%p>][%s]: cancel ssr check work.\n", current, __func__);
		cancel_delayed_work(&dev->ssr_check_wq);
	}
#endif

	pr_info("[%p]: Restart sequence for %s completed.\n",
			current, desc->name);

	mutex_unlock(&soc_order_reg_lock);
	mutex_unlock(&track->lock);

	spin_lock_irqsave(&track->s_lock, flags);
	track->p_state = SUBSYS_NORMAL;
	__pm_relax(&dev->ssr_wlock);
	spin_unlock_irqrestore(&track->s_lock, flags);
}

static void __subsystem_restart_dev(struct subsys_device *dev)
{
	struct subsys_desc *desc = dev->desc;
	const char *name = dev->desc->name;
	struct subsys_tracking *track;
	unsigned long flags;

#if defined(CONFIG_HTC_DEBUG_SSR)
		
		if (!strncmp(name, "modem",
					SUBSYS_NAME_MAX_LENGTH)) {
		msr_info_list[msm_msr_index].valid = 1;
		msr_info_list[msm_msr_index].msr_time = current_kernel_time();
		snprintf(msr_info_list[msm_msr_index].modem_errmsg, RD_BUF_SIZE, "%s", dev->restart_reason);
		if(++msm_msr_index >= MODEM_ERRMSG_LIST_LEN)
		msm_msr_index = 0;
			}
	   
#endif

#if defined(CONFIG_HTC_FEATURES_SSR)
	pr_info("Restarting %s [level=%s]!\n", desc->name, restart_levels[dev->restart_level]);
#else
	pr_debug("Restarting %s [level=%s]!\n", desc->name,
			restart_levels[dev->restart_level]);
#endif

	track = subsys_get_track(dev);
	spin_lock_irqsave(&track->s_lock, flags);
	if (track->p_state != SUBSYS_CRASHED &&
					dev->track.state == SUBSYS_ONLINE) {
		if (track->p_state != SUBSYS_RESTARTING) {
			track->p_state = SUBSYS_CRASHED;
			__pm_stay_awake(&dev->ssr_wlock);
			queue_work(ssr_wq, &dev->work);
		} else {
			panic("Subsystem %s crashed during SSR!", name);
		}
	} else
		WARN(dev->track.state == SUBSYS_OFFLINE,
			"SSR aborted: %s subsystem not online\n", name);
	spin_unlock_irqrestore(&track->s_lock, flags);
}

static void device_restart_work_hdlr(struct work_struct *work)
{
	struct subsys_device *dev = container_of(work, struct subsys_device,
							device_restart_work);

	notify_each_subsys_device(&dev, 1, SUBSYS_SOC_RESET, NULL);
#if defined(CONFIG_HTC_DEBUG_SSR)
	panic("SSR: %s crashed. %s", dev->desc->name, dev->restart_reason);
#else
	panic("subsys-restart: Resetting the SoC - %s crashed.",
							dev->desc->name);
#endif
}

int subsystem_restart_dev(struct subsys_device *dev)
{
	const char *name;

	if (!get_device(&dev->dev))
		return -ENODEV;

	if (!try_module_get(dev->owner)) {
		put_device(&dev->dev);
		return -ENODEV;
	}

	name = dev->desc->name;

	if (system_state == SYSTEM_RESTART
		|| system_state == SYSTEM_POWER_OFF) {
		pr_err("%s crashed during a system poweroff/shutdown.\n", name);
		return -EBUSY;
	}

	pr_info("Restart sequence requested for %s, restart_level = %s.\n",
		name, restart_levels[dev->restart_level]);

	if (WARN(disable_restart_work == DISABLE_SSR,
		"subsys-restart: Ignoring restart request for %s.\n", name)) {
		return 0;
	}

	switch (dev->restart_level) {

	case RESET_SUBSYS_COUPLED:
		__subsystem_restart_dev(dev);
		break;
	case RESET_SOC:
		__pm_stay_awake(&dev->ssr_wlock);
		schedule_work(&dev->device_restart_work);
		return 0;
	default:
		panic("subsys-restart: Unknown restart level!\n");
		break;
	}
	module_put(dev->owner);
	put_device(&dev->dev);

	return 0;
}
EXPORT_SYMBOL(subsystem_restart_dev);

int subsystem_restart(const char *name)
{
	int ret;
	struct subsys_device *dev = find_subsys(name);

	if (!dev)
		return -ENODEV;

	ret = subsystem_restart_dev(dev);
	put_device(&dev->dev);
	return ret;
}
EXPORT_SYMBOL(subsystem_restart);

int subsystem_crashed(const char *name)
{
	struct subsys_device *dev = find_subsys(name);
	struct subsys_tracking *track;

	if (!dev)
		return -ENODEV;

	if (!get_device(&dev->dev))
		return -ENODEV;

	track = subsys_get_track(dev);

	mutex_lock(&track->lock);
	dev->do_ramdump_on_put = true;
	mutex_unlock(&track->lock);

	put_device(&dev->dev);
	return 0;
}
EXPORT_SYMBOL(subsystem_crashed);

void subsys_set_crash_status(struct subsys_device *dev, bool crashed)
{
	dev->crashed = crashed;
}

bool subsys_get_crash_status(struct subsys_device *dev)
{
	return dev->crashed;
}

static struct subsys_device *desc_to_subsys(struct device *d)
{
	struct subsys_device *device, *subsys_dev = 0;

	mutex_lock(&subsys_list_lock);
	list_for_each_entry(device, &subsys_list, list)
		if (device->desc->dev == d)
			subsys_dev = device;
	mutex_unlock(&subsys_list_lock);
	return subsys_dev;
}

void notify_proxy_vote(struct device *device)
{
	struct subsys_device *dev = desc_to_subsys(device);

	if (dev)
		notify_each_subsys_device(&dev, 1, SUBSYS_PROXY_VOTE, NULL);
}

void notify_proxy_unvote(struct device *device)
{
	struct subsys_device *dev = desc_to_subsys(device);

	if (dev)
		notify_each_subsys_device(&dev, 1, SUBSYS_PROXY_UNVOTE, NULL);
}

#ifdef CONFIG_DEBUG_FS
static ssize_t subsys_debugfs_read(struct file *filp, char __user *ubuf,
		size_t cnt, loff_t *ppos)
{
	int r;
	char buf[40];
	struct subsys_device *subsys = filp->private_data;

	r = snprintf(buf, sizeof(buf), "%d\n", subsys->count);
	return simple_read_from_buffer(ubuf, cnt, ppos, buf, r);
}

static ssize_t subsys_debugfs_write(struct file *filp,
		const char __user *ubuf, size_t cnt, loff_t *ppos)
{
	struct subsys_device *subsys = filp->private_data;
	char buf[10];
	char *cmp;

	cnt = min(cnt, sizeof(buf) - 1);
	if (copy_from_user(&buf, ubuf, cnt))
		return -EFAULT;
	buf[cnt] = '\0';
	cmp = strstrip(buf);

	if (!strcmp(cmp, "restart")) {
		if (subsystem_restart_dev(subsys))
			return -EIO;
	} else if (!strcmp(cmp, "get")) {
		if (subsystem_get(subsys->desc->name))
			return -EIO;
	} else if (!strcmp(cmp, "put")) {
		subsystem_put(subsys);
	} else {
		return -EINVAL;
	}

	return cnt;
}

static const struct file_operations subsys_debugfs_fops = {
	.open	= simple_open,
	.read	= subsys_debugfs_read,
	.write	= subsys_debugfs_write,
};

static struct dentry *subsys_base_dir;

static int __init subsys_debugfs_init(void)
{
	subsys_base_dir = debugfs_create_dir("msm_subsys", NULL);
	return !subsys_base_dir ? -ENOMEM : 0;
}

static void subsys_debugfs_exit(void)
{
	debugfs_remove_recursive(subsys_base_dir);
}

static int subsys_debugfs_add(struct subsys_device *subsys)
{
	if (!subsys_base_dir)
		return -ENOMEM;

	subsys->dentry = debugfs_create_file(subsys->desc->name,
				S_IRUGO | S_IWUSR, subsys_base_dir,
				subsys, &subsys_debugfs_fops);
	return !subsys->dentry ? -ENOMEM : 0;
}

static void subsys_debugfs_remove(struct subsys_device *subsys)
{
	debugfs_remove(subsys->dentry);
}
#else
static int __init subsys_debugfs_init(void) { return 0; };
static void subsys_debugfs_exit(void) { }
static int subsys_debugfs_add(struct subsys_device *subsys) { return 0; }
static void subsys_debugfs_remove(struct subsys_device *subsys) { }
#endif

static int subsys_device_open(struct inode *inode, struct file *file)
{
	struct subsys_device *device, *subsys_dev = 0;
	void *retval;

	mutex_lock(&subsys_list_lock);
	list_for_each_entry(device, &subsys_list, list)
		if (MINOR(device->dev_no) == iminor(inode))
			subsys_dev = device;
	mutex_unlock(&subsys_list_lock);

	if (!subsys_dev)
		return -EINVAL;

	retval = subsystem_get_with_fwname(subsys_dev->desc->name,
					subsys_dev->desc->fw_name);
	if (IS_ERR(retval))
		return PTR_ERR(retval);

	return 0;
}

static int subsys_device_close(struct inode *inode, struct file *file)
{
	struct subsys_device *device, *subsys_dev = 0;

	mutex_lock(&subsys_list_lock);
	list_for_each_entry(device, &subsys_list, list)
		if (MINOR(device->dev_no) == iminor(inode))
			subsys_dev = device;
	mutex_unlock(&subsys_list_lock);

	if (!subsys_dev)
		return -EINVAL;

	subsystem_put(subsys_dev);
	return 0;
}

static const struct file_operations subsys_device_fops = {
		.owner = THIS_MODULE,
		.open = subsys_device_open,
		.release = subsys_device_close,
};

static void subsys_device_release(struct device *dev)
{
	struct subsys_device *subsys = to_subsys(dev);

	wakeup_source_trash(&subsys->ssr_wlock);
	mutex_destroy(&subsys->track.lock);
	ida_simple_remove(&subsys_ida, subsys->id);
	kfree(subsys);
}
static irqreturn_t subsys_err_ready_intr_handler(int irq, void *subsys)
{
	struct subsys_device *subsys_dev = subsys;
	dev_info(subsys_dev->desc->dev,
		"Subsystem error monitoring/handling services are up\n");

	if (subsys_dev->desc->is_not_loadable)
		return IRQ_HANDLED;

	complete(&subsys_dev->err_ready);
	return IRQ_HANDLED;
}

static int subsys_char_device_add(struct subsys_device *subsys_dev)
{
	int ret = 0;
	static int major, minor;
	dev_t dev_no;

	mutex_lock(&char_device_lock);
	if (!major) {
		ret = alloc_chrdev_region(&dev_no, 0, 4, "subsys");
		if (ret < 0) {
			pr_err("Failed to alloc subsys_dev region, err %d\n",
									ret);
			goto fail;
		}
		major = MAJOR(dev_no);
		minor = MINOR(dev_no);
	} else
		dev_no = MKDEV(major, minor);

	if (!device_create(char_class, subsys_dev->desc->dev, dev_no,
			NULL, "subsys_%s", subsys_dev->desc->name)) {
		pr_err("Failed to create subsys_%s device\n",
						subsys_dev->desc->name);
		goto fail_unregister_cdev_region;
	}

	cdev_init(&subsys_dev->char_dev, &subsys_device_fops);
	subsys_dev->char_dev.owner = THIS_MODULE;
	ret = cdev_add(&subsys_dev->char_dev, dev_no, 1);
	if (ret < 0)
		goto fail_destroy_device;

	subsys_dev->dev_no = dev_no;
	minor++;
	mutex_unlock(&char_device_lock);

	return 0;

fail_destroy_device:
	device_destroy(char_class, dev_no);
fail_unregister_cdev_region:
	unregister_chrdev_region(dev_no, 1);
fail:
	mutex_unlock(&char_device_lock);
	return ret;
}

static void subsys_char_device_remove(struct subsys_device *subsys_dev)
{
	cdev_del(&subsys_dev->char_dev);
	device_destroy(char_class, subsys_dev->dev_no);
	unregister_chrdev_region(subsys_dev->dev_no, 1);
}

static void subsys_remove_restart_order(struct device_node *device)
{
	struct subsys_soc_restart_order *order;
	int i;

	mutex_lock(&ssr_order_mutex);
	list_for_each_entry(order, &ssr_order_list, list)
		for (i = 0; i < order->count; i++)
			if (order->device_ptrs[i] == device)
				order->subsys_ptrs[i] = NULL;
	mutex_unlock(&ssr_order_mutex);
}

static struct subsys_soc_restart_order *ssr_parse_restart_orders(struct
							subsys_desc * desc)
{
	int i, j, count, num = 0;
	struct subsys_soc_restart_order *order, *tmp;
	struct device *dev = desc->dev;
	struct device_node *ssr_node;
	uint32_t len;

	if (!of_get_property(dev->of_node, "qcom,restart-group", &len))
		return NULL;

	count = len/sizeof(uint32_t);

	order = devm_kzalloc(dev, sizeof(*order), GFP_KERNEL);
	if (!order)
		return ERR_PTR(-ENOMEM);

	order->subsys_ptrs = devm_kzalloc(dev,
				count * sizeof(struct subsys_device *),
				GFP_KERNEL);
	if (!order->subsys_ptrs)
		return ERR_PTR(-ENOMEM);

	order->device_ptrs = devm_kzalloc(dev,
				count * sizeof(struct device_node *),
				GFP_KERNEL);
	if (!order->device_ptrs)
		return ERR_PTR(-ENOMEM);

	for (i = 0; i < count; i++) {
		ssr_node = of_parse_phandle(dev->of_node,
						"qcom,restart-group", i);
		if (!ssr_node)
			return ERR_PTR(-ENXIO);
		of_node_put(ssr_node);
		pr_info("%s device has been added to %s's restart group\n",
						ssr_node->name, desc->name);
		order->device_ptrs[i] = ssr_node;
	}

	mutex_lock(&ssr_order_mutex);
	list_for_each_entry(tmp, &ssr_order_list, list) {
		for (i = 0; i < count; i++) {
			for (j = 0; j < count; j++) {
				if (order->device_ptrs[j] !=
					tmp->device_ptrs[i])
					continue;
				else
					num++;
			}
		}

		if (num == count && tmp->count == count)
			goto err;
		else if (num) {
			tmp = ERR_PTR(-EINVAL);
			goto err;
		}
	}

	order->count = count;
	mutex_init(&order->track.lock);
	spin_lock_init(&order->track.s_lock);

	INIT_LIST_HEAD(&order->list);
	list_add_tail(&order->list, &ssr_order_list);
	mutex_unlock(&ssr_order_mutex);

	return order;
err:
	mutex_unlock(&ssr_order_mutex);
	return tmp;
}

static int __get_gpio(struct subsys_desc *desc, const char *prop,
		int *gpio)
{
	struct device_node *dnode = desc->dev->of_node;
	int ret = -ENOENT;

	if (of_find_property(dnode, prop, NULL)) {
		*gpio = of_get_named_gpio(dnode, prop, 0);
		ret = *gpio < 0 ? *gpio : 0;
	}

	return ret;
}

static int __get_irq(struct subsys_desc *desc, const char *prop,
		unsigned int *irq, int *gpio)
{
	int ret, gpiol, irql;

	ret = __get_gpio(desc, prop, &gpiol);
	if (ret)
		return ret;

	irql = gpio_to_irq(gpiol);

	if (irql == -ENOENT)
		irql = -ENXIO;

	if (irql < 0) {
		pr_err("[%s]: Error getting IRQ \"%s\"\n", desc->name,
				prop);
		return irql;
	} else {
		if (gpio)
			*gpio = gpiol;
		*irq = irql;
	}

	return 0;
}

static int subsys_parse_devicetree(struct subsys_desc *desc)
{
	struct subsys_soc_restart_order *order;
	int ret;

	struct platform_device *pdev = container_of(desc->dev,
					struct platform_device, dev);

	ret = __get_irq(desc, "qcom,gpio-err-fatal", &desc->err_fatal_irq,
							&desc->err_fatal_gpio);
	if (ret && ret != -ENOENT)
		return ret;

	ret = __get_irq(desc, "qcom,gpio-err-ready", &desc->err_ready_irq,
							NULL);
	if (ret && ret != -ENOENT)
		return ret;

	ret = __get_irq(desc, "qcom,gpio-stop-ack", &desc->stop_ack_irq, NULL);
	if (ret && ret != -ENOENT)
		return ret;

	ret = __get_gpio(desc, "qcom,gpio-force-stop", &desc->force_stop_gpio);
	if (ret && ret != -ENOENT)
		return ret;

	ret = __get_gpio(desc, "qcom,gpio-ramdump-disable",
			&desc->ramdump_disable_gpio);
	if (ret && ret != -ENOENT)
		return ret;

	ret = platform_get_irq(pdev, 0);
	if (ret > 0)
		desc->wdog_bite_irq = ret;

	order = ssr_parse_restart_orders(desc);
	if (IS_ERR(order)) {
		pr_err("Could not initialize SSR restart order, err = %ld\n",
							PTR_ERR(order));
		return PTR_ERR(order);
	}

	return 0;
}

static int subsys_setup_irqs(struct subsys_device *subsys)
{
	struct subsys_desc *desc = subsys->desc;
	int ret;

	if (desc->err_fatal_irq && desc->err_fatal_handler) {
		ret = devm_request_irq(desc->dev, desc->err_fatal_irq,
				desc->err_fatal_handler,
				IRQF_TRIGGER_RISING, desc->name, desc);
		if (ret < 0) {
			dev_err(desc->dev, "[%s]: Unable to register error fatal IRQ handler!: %d\n",
				desc->name, ret);
			return ret;
		}
		disable_irq(desc->err_fatal_irq);
	}

	if (desc->stop_ack_irq && desc->stop_ack_handler) {
		ret = devm_request_irq(desc->dev, desc->stop_ack_irq,
			desc->stop_ack_handler,
			IRQF_TRIGGER_RISING, desc->name, desc);
		if (ret < 0) {
			dev_err(desc->dev, "[%s]: Unable to register stop ack handler!: %d\n",
				desc->name, ret);
			return ret;
		}
		disable_irq(desc->stop_ack_irq);
	}

	if (desc->wdog_bite_irq && desc->wdog_bite_handler) {
		ret = devm_request_irq(desc->dev, desc->wdog_bite_irq,
			desc->wdog_bite_handler,
			IRQF_TRIGGER_RISING, desc->name, desc);
		if (ret < 0) {
			dev_err(desc->dev, "[%s]: Unable to register wdog bite handler!: %d\n",
				desc->name, ret);
			return ret;
		}
		disable_irq(desc->wdog_bite_irq);
	}

	if (desc->err_ready_irq) {
		ret = devm_request_irq(desc->dev,
					desc->err_ready_irq,
					subsys_err_ready_intr_handler,
					IRQF_TRIGGER_RISING,
					"error_ready_interrupt", subsys);
		if (ret < 0) {
			dev_err(desc->dev,
				"[%s]: Unable to register err ready handler\n",
				desc->name);
			return ret;
		}
		disable_irq(desc->err_ready_irq);
	}

	return 0;
}

static void subsys_free_irqs(struct subsys_device *subsys)
{
	struct subsys_desc *desc = subsys->desc;

	if (desc->err_fatal_irq && desc->err_fatal_handler)
		devm_free_irq(desc->dev, desc->err_fatal_irq, desc);
	if (desc->stop_ack_irq && desc->stop_ack_handler)
		devm_free_irq(desc->dev, desc->stop_ack_irq, desc);
	if (desc->wdog_bite_irq && desc->wdog_bite_handler)
		devm_free_irq(desc->dev, desc->wdog_bite_irq, desc);
	if (desc->err_ready_irq)
		devm_free_irq(desc->dev, desc->err_ready_irq, subsys);
}

struct subsys_device *subsys_register(struct subsys_desc *desc)
{
	struct subsys_device *subsys;
	struct device_node *ofnode = desc->dev->of_node;
	int ret;

	subsys = kzalloc(sizeof(*subsys), GFP_KERNEL);
	if (!subsys)
		return ERR_PTR(-ENOMEM);

	subsys->desc = desc;
	subsys->owner = desc->owner;
	subsys->dev.parent = desc->dev;
	subsys->dev.bus = &subsys_bus_type;
	subsys->dev.release = subsys_device_release;
	subsys->notif_state = -1;
	subsys->desc->sysmon_pid = -1;
	strlcpy(subsys->desc->fw_name, desc->name,
			sizeof(subsys->desc->fw_name));

	subsys->notify = subsys_notif_add_subsys(desc->name);

#if defined(CONFIG_HTC_DEBUG_SSR)
	memset(subsys->restart_reason, 0, sizeof(subsys->restart_reason));
#endif

	snprintf(subsys->wlname, sizeof(subsys->wlname), "ssr(%s)", desc->name);
	wakeup_source_init(&subsys->ssr_wlock, subsys->wlname);
	INIT_WORK(&subsys->work, subsystem_restart_wq_func);
	INIT_WORK(&subsys->device_restart_work, device_restart_work_hdlr);
#if defined(CONFIG_HTC_DEBUG_RIL_PCN0009_SSR_DUMP_TASK)
	INIT_DELAYED_WORK(&subsys->ssr_check_wq, subsystem_restart_check_func);
#endif
	spin_lock_init(&subsys->track.s_lock);

	subsys->id = ida_simple_get(&subsys_ida, 0, 0, GFP_KERNEL);
	if (subsys->id < 0) {
		ret = subsys->id;
		goto err_ida;
	}
	dev_set_name(&subsys->dev, "subsys%d", subsys->id);

	mutex_init(&subsys->track.lock);

	ret = subsys_debugfs_add(subsys);
	if (ret)
		goto err_debugfs;

	ret = device_register(&subsys->dev);
	if (ret) {
		device_unregister(&subsys->dev);
		goto err_register;
	}

	ret = subsys_char_device_add(subsys);
	if (ret) {
		put_device(&subsys->dev);
		goto err_register;
	}

	if (ofnode) {
		ret = subsys_parse_devicetree(desc);
		if (ret)
			goto err_register;

		subsys->restart_order = update_restart_order(subsys);

		ret = subsys_setup_irqs(subsys);
		if (ret < 0)
			goto err_setup_irqs;

		if (of_property_read_u32(ofnode, "qcom,ssctl-instance-id",
					&desc->ssctl_instance_id))
			pr_debug("Reading instance-id for %s failed\n",
								desc->name);

		if (of_property_read_u32(ofnode, "qcom,sysmon-id",
					&subsys->desc->sysmon_pid))
			pr_debug("Reading sysmon-id for %s failed\n",
								desc->name);
	}

	ret = sysmon_notifier_register(desc);
	if (ret < 0)
		goto err_sysmon_notifier;

	mutex_lock(&subsys_list_lock);
	INIT_LIST_HEAD(&subsys->list);
	list_add_tail(&subsys->list, &subsys_list);
	mutex_unlock(&subsys_list_lock);

	return subsys;
err_sysmon_notifier:
	if (ofnode)
		subsys_free_irqs(subsys);
err_setup_irqs:
	if (ofnode)
		subsys_remove_restart_order(ofnode);
err_register:
	subsys_debugfs_remove(subsys);
err_debugfs:
	mutex_destroy(&subsys->track.lock);
	ida_simple_remove(&subsys_ida, subsys->id);
err_ida:
	wakeup_source_trash(&subsys->ssr_wlock);
	kfree(subsys);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL(subsys_register);

void subsys_unregister(struct subsys_device *subsys)
{
	struct subsys_device *subsys_dev, *tmp;
	struct device_node *device = subsys->desc->dev->of_node;

	if (IS_ERR_OR_NULL(subsys))
		return;

	if (get_device(&subsys->dev)) {
		mutex_lock(&subsys_list_lock);
		list_for_each_entry_safe(subsys_dev, tmp, &subsys_list, list)
			if (subsys_dev == subsys)
				list_del(&subsys->list);
		mutex_unlock(&subsys_list_lock);

		if (device) {
			subsys_free_irqs(subsys);
			subsys_remove_restart_order(device);
		}
		mutex_lock(&subsys->track.lock);
		WARN_ON(subsys->count);
		device_unregister(&subsys->dev);
		mutex_unlock(&subsys->track.lock);
		subsys_debugfs_remove(subsys);
		subsys_char_device_remove(subsys);
		sysmon_notifier_unregister(subsys->desc);
		put_device(&subsys->dev);
	}
}
EXPORT_SYMBOL(subsys_unregister);

static int subsys_panic(struct device *dev, void *data)
{
	struct subsys_device *subsys = to_subsys(dev);

	if (subsys->desc->crash_shutdown)
		subsys->desc->crash_shutdown(subsys->desc);
	return 0;
}

static int ssr_panic_handler(struct notifier_block *this,
				unsigned long event, void *ptr)
{
	bus_for_each_dev(&subsys_bus_type, NULL, NULL, subsys_panic);
	return NOTIFY_DONE;
}

static struct notifier_block panic_nb = {
	.notifier_call  = ssr_panic_handler,
};

static int __init subsys_restart_init(void)
{
	int ret;
#if defined(CONFIG_HTC_DEBUG_SSR)
		struct kobject *properties_kobj;
		
		subsystem_restart_reason_nonblock_init();
		properties_kobj = kobject_create_and_add("subsystem_restart_properties", NULL);
		if (properties_kobj) {
			ret = sysfs_create_group(properties_kobj, &attr_group);
			if (ret) {
				pr_err("subsys_restart_init: sysfs_create_group failed\n");
				return ret;
			}
		}
		
#endif
	ssr_wq = alloc_workqueue("ssr_wq", WQ_CPU_INTENSIVE, 0);
	BUG_ON(!ssr_wq);

	ret = bus_register(&subsys_bus_type);
	if (ret)
		goto err_bus;
	ret = subsys_debugfs_init();
	if (ret)
		goto err_debugfs;

	char_class = class_create(THIS_MODULE, "subsys");
	if (IS_ERR(char_class)) {
		ret = -ENOMEM;
		pr_err("Failed to create subsys_dev class\n");
		goto err_class;
	}

	ret = atomic_notifier_chain_register(&panic_notifier_list,
			&panic_nb);
	if (ret)
		goto err_soc;

	return 0;

err_soc:
	class_destroy(char_class);
err_class:
	subsys_debugfs_exit();
err_debugfs:
	bus_unregister(&subsys_bus_type);
err_bus:
	destroy_workqueue(ssr_wq);
	return ret;
}
arch_initcall(subsys_restart_init);

MODULE_DESCRIPTION("Subsystem Restart Driver");
MODULE_LICENSE("GPL v2");
