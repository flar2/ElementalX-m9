/*
 * Alarmtimer interface
 *
 * This interface provides a timer which is similarto hrtimers,
 * but triggers a RTC alarm if the box is suspend.
 *
 * This interface is influenced by the Android RTC Alarm timer
 * interface.
 *
 * Copyright (C) 2010 IBM Corperation
 *
 * Author: John Stultz <john.stultz@linaro.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/time.h>
#include <linux/hrtimer.h>
#include <linux/timerqueue.h>
#include <linux/rtc.h>
#include <linux/alarmtimer.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/posix-timers.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
#include <linux/module.h>

#define OFFALARM_SIZE    (10)
#define OFFALARM_SNOOZE_SIZE    (10)

static int offalarm_size = OFFALARM_SIZE;
static int offalarm_snooze_size = OFFALARM_SNOOZE_SIZE;

static int offalarm[OFFALARM_SIZE];
static int offalarm_snooze[OFFALARM_SNOOZE_SIZE];
module_param_array_named(offalarm, offalarm, uint, &offalarm_size, S_IRUGO | S_IWUSR);
module_param_array_named(offalarm_snooze, offalarm_snooze, uint, &offalarm_snooze_size, S_IRUGO | S_IWUSR);

#define ALARM_DELTA 120

static struct alarm_base {
	spinlock_t		lock;
	struct timerqueue_head	timerqueue;
	ktime_t			(*gettime)(void);
	clockid_t		base_clockid;
} alarm_bases[ALARM_NUMTYPE];

static ktime_t freezer_delta;
static DEFINE_SPINLOCK(freezer_delta_lock);

static struct wakeup_source *ws;

#ifdef CONFIG_RTC_CLASS
static struct rtc_timer		rtctimer;
static struct rtc_device	*rtcdev;
static DEFINE_SPINLOCK(rtcdev_lock);
static unsigned long power_on_alarm;
static struct mutex power_on_alarm_lock;

void set_power_on_alarm(long secs, bool enable)
{
	int rc;
	struct timespec wall_time;
	long rtc_secs, alarm_time, alarm_delta;
	struct rtc_time rtc_time;
	struct rtc_wkalrm alarm;
	pr_info("set_power_on_alarm: secs = %ld enable = %d\n",secs, enable);

	rc = mutex_lock_interruptible(&power_on_alarm_lock);
	if (rc != 0)
		return;

	if (enable) {
			power_on_alarm = secs;
	} else {
		if (power_on_alarm == secs)
			power_on_alarm = 0;
		else
			goto exit;
	}

	if (!power_on_alarm)
		goto disable_alarm;

	rtc_read_time(rtcdev, &rtc_time);
	getnstimeofday(&wall_time);
	rtc_tm_to_time(&rtc_time, &rtc_secs);
	alarm_delta = wall_time.tv_sec - rtc_secs;
	alarm_time = power_on_alarm - alarm_delta;
	pr_info("set_power_on_alarm: alarm_time = %ld\n",alarm_time);

	if ((alarm_time - ALARM_DELTA) > rtc_secs)
		alarm_time -= ALARM_DELTA;
	else
		goto disable_alarm;

	rtc_time_to_tm(alarm_time, &alarm.time);
	alarm.enabled = 1;
	rc = rtc_set_alarm(rtcdev, &alarm);
	if (rc)
		goto disable_alarm;

	mutex_unlock(&power_on_alarm_lock);
	return;

disable_alarm:
	power_on_alarm = 0;
	rtc_alarm_irq_enable(rtcdev, 0);
exit:
	mutex_unlock(&power_on_alarm_lock);
}

static void alarmtimer_triggered_func(void *p)
{
	struct rtc_device *rtc = rtcdev;
	if (!(rtc->irq_data & RTC_AF))
		return;
	__pm_wakeup_event(ws, 2 * MSEC_PER_SEC);
}

static struct rtc_task alarmtimer_rtc_task = {
	.func = alarmtimer_triggered_func
};


static int find_offmode_alarm(void)
{
        struct timespec rtc_now;
        int i;
        int nearest_alarm = 0;
        int nearest_alarm_snooze = 0;

        getnstimeofday(&rtc_now);
		pr_info("find_offmode_alarm: offalarm_size=%d  rtc_now.tv_sec= %ld\n", offalarm_size, rtc_now.tv_sec);
        for (i = 0; i < offalarm_size; i++) {
			pr_info("find_offmode_alarm: offalarm[%d]=%d \n", i,offalarm[i]);
                if (offalarm[i] > rtc_now.tv_sec) {
                        if (nearest_alarm == 0)
                                nearest_alarm = offalarm[i];
                        else if (offalarm[i] < nearest_alarm)
                                nearest_alarm = offalarm[i];
                }
        }
        for (i = 0; i < offalarm_snooze_size; i++) {
                if (offalarm_snooze[i] > rtc_now.tv_sec) {
                        if (nearest_alarm_snooze == 0)
                                nearest_alarm_snooze = offalarm_snooze[i];
                        else if (offalarm_snooze[i] < nearest_alarm_snooze)
                                nearest_alarm_snooze = offalarm_snooze[i];
                }
        }
        if((nearest_alarm == 0 || nearest_alarm_snooze <= nearest_alarm) && nearest_alarm_snooze != 0)
                return nearest_alarm_snooze;
        else
                return nearest_alarm;
}

static void alarm_shutdown(struct platform_device *pdev)
{
        int offmode_alarm;
        struct rtc_wkalrm rtc_alarm;

        offmode_alarm = find_offmode_alarm();
		pr_info("alarm_shutdown: offmode_alarm = %d \n",offmode_alarm);
        if (offmode_alarm > 0) {
                rtc_time_to_tm(offmode_alarm, &rtc_alarm.time);
				pr_info("alarm_shutdown: h::m:s == %d::%d::%d, d/m/y = %d/%d/%d\r\n",
	rtc_alarm.time.tm_hour, rtc_alarm.time.tm_min, rtc_alarm.time.tm_sec,
	rtc_alarm.time.tm_mday, rtc_alarm.time.tm_mon, rtc_alarm.time.tm_year);
                rtc_alarm.enabled = 1;
                rtc_set_alarm(rtcdev, &rtc_alarm);
        }
}

struct rtc_device *alarmtimer_get_rtcdev(void)
{
	unsigned long flags;
	struct rtc_device *ret = NULL;

	spin_lock_irqsave(&rtcdev_lock, flags);
	ret = rtcdev;
	spin_unlock_irqrestore(&rtcdev_lock, flags);

	return ret;
}

static int alarmtimer_rtc_add_device(struct device *dev,
				struct class_interface *class_intf)
{
	unsigned long flags;
	int err = 0;
	struct rtc_device *rtc = to_rtc_device(dev);
	if (rtcdev)
		return -EBUSY;
	if (!rtc->ops->set_alarm)
		return -1;

	spin_lock_irqsave(&rtcdev_lock, flags);
	if (!rtcdev) {
		err = rtc_irq_register(rtc, &alarmtimer_rtc_task);
		if (err)
			goto rtc_irq_reg_err;
		rtcdev = rtc;
		
		get_device(dev);
	}

rtc_irq_reg_err:
	spin_unlock_irqrestore(&rtcdev_lock, flags);
	return err;

}

static void alarmtimer_rtc_remove_device(struct device *dev,
				struct class_interface *class_intf)
{
	if (rtcdev && dev == &rtcdev->dev) {
		rtc_irq_unregister(rtcdev, &alarmtimer_rtc_task);
		rtcdev = NULL;
	}
}

static inline void alarmtimer_rtc_timer_init(void)
{
	mutex_init(&power_on_alarm_lock);

	rtc_timer_init(&rtctimer, NULL, NULL);
}

static struct class_interface alarmtimer_rtc_interface = {
	.add_dev = &alarmtimer_rtc_add_device,
	.remove_dev = &alarmtimer_rtc_remove_device,
};

static int alarmtimer_rtc_interface_setup(void)
{
	alarmtimer_rtc_interface.class = rtc_class;
	return class_interface_register(&alarmtimer_rtc_interface);
}
static void alarmtimer_rtc_interface_remove(void)
{
	class_interface_unregister(&alarmtimer_rtc_interface);
}
#else
struct rtc_device *alarmtimer_get_rtcdev(void)
{
	return NULL;
}
#define rtcdev (NULL)
static inline int alarmtimer_rtc_interface_setup(void) { return 0; }
static inline void alarmtimer_rtc_interface_remove(void) { }
static inline void alarmtimer_rtc_timer_init(void) { }
void set_power_on_alarm(long secs, bool enable) { }
#endif

static void alarmtimer_enqueue(struct alarm_base *base, struct alarm *alarm)
{
	if (alarm->state & ALARMTIMER_STATE_ENQUEUED)
		timerqueue_del(&base->timerqueue, &alarm->node);

	timerqueue_add(&base->timerqueue, &alarm->node);
	alarm->state |= ALARMTIMER_STATE_ENQUEUED;
}

static void alarmtimer_dequeue(struct alarm_base *base, struct alarm *alarm)
{
	if (!(alarm->state & ALARMTIMER_STATE_ENQUEUED))
		return;

	timerqueue_del(&base->timerqueue, &alarm->node);
	alarm->state &= ~ALARMTIMER_STATE_ENQUEUED;
}


static enum hrtimer_restart alarmtimer_fired(struct hrtimer *timer)
{
	struct alarm *alarm = container_of(timer, struct alarm, timer);
	struct alarm_base *base = &alarm_bases[alarm->type];
	unsigned long flags;
	int ret = HRTIMER_NORESTART;
	int restart = ALARMTIMER_NORESTART;

	spin_lock_irqsave(&base->lock, flags);
	alarmtimer_dequeue(base, alarm);
	spin_unlock_irqrestore(&base->lock, flags);

	if (alarm->function)
		restart = alarm->function(alarm, base->gettime());

	spin_lock_irqsave(&base->lock, flags);
	if (restart != ALARMTIMER_NORESTART) {
		hrtimer_set_expires(&alarm->timer, alarm->node.expires);
		alarmtimer_enqueue(base, alarm);
		ret = HRTIMER_RESTART;
	}
	spin_unlock_irqrestore(&base->lock, flags);

	return ret;

}

ktime_t alarm_expires_remaining(const struct alarm *alarm)
{
	struct alarm_base *base = &alarm_bases[alarm->type];
	return ktime_sub(alarm->node.expires, base->gettime());
}

#ifdef CONFIG_RTC_CLASS
static int alarmtimer_suspend(struct device *dev)
{
	struct rtc_time tm;
	ktime_t min, now;
	unsigned long flags;
	struct rtc_device *rtc;
	int i;
	int ret;

	spin_lock_irqsave(&freezer_delta_lock, flags);
	min = freezer_delta;
	freezer_delta = ktime_set(0, 0);
	spin_unlock_irqrestore(&freezer_delta_lock, flags);

	rtc = alarmtimer_get_rtcdev();
	
	if (!rtc)
		return 0;

	
	for (i = 0; i < ALARM_NUMTYPE; i++) {
		struct alarm_base *base = &alarm_bases[i];
		struct timerqueue_node *next;
		ktime_t delta;

		spin_lock_irqsave(&base->lock, flags);
		next = timerqueue_getnext(&base->timerqueue);
		spin_unlock_irqrestore(&base->lock, flags);
		if (!next)
			continue;
		delta = ktime_sub(next->expires, base->gettime());
		if (!min.tv64 || (delta.tv64 < min.tv64))
			min = delta;
	}
	if (min.tv64 == 0)
		return 0;

	if (ktime_to_ns(min) < 2 * NSEC_PER_SEC) {
		__pm_wakeup_event(ws, 2 * MSEC_PER_SEC);
		return -EBUSY;
	}

	
	rtc_timer_cancel(rtc, &rtctimer);
	rtc_read_time(rtc, &tm);
	now = rtc_tm_to_ktime(tm);
	now = ktime_add(now, min);

	
	ret = rtc_timer_start(rtc, &rtctimer, now, ktime_set(0, 0));
	if (ret < 0)
		__pm_wakeup_event(ws, MSEC_PER_SEC);
	return ret;
}
static int alarmtimer_resume(struct device *dev)
{
	struct rtc_device *rtc;

	rtc = alarmtimer_get_rtcdev();
	
	if (!rtc)
		return 0;
	rtc_timer_cancel(rtc, &rtctimer);

	set_power_on_alarm(power_on_alarm , 1);
	return 0;
}
#else
static int alarmtimer_suspend(struct device *dev)
{
	return 0;
}

static int alarmtimer_resume(struct device *dev)
{
	return 0;
}
#endif

static void alarmtimer_freezerset(ktime_t absexp, enum alarmtimer_type type)
{
	ktime_t delta;
	unsigned long flags;
	struct alarm_base *base = &alarm_bases[type];

	delta = ktime_sub(absexp, base->gettime());

	spin_lock_irqsave(&freezer_delta_lock, flags);
	if (!freezer_delta.tv64 || (delta.tv64 < freezer_delta.tv64))
		freezer_delta = delta;
	spin_unlock_irqrestore(&freezer_delta_lock, flags);
}


void alarm_init(struct alarm *alarm, enum alarmtimer_type type,
		enum alarmtimer_restart (*function)(struct alarm *, ktime_t))
{
	timerqueue_init(&alarm->node);
	hrtimer_init(&alarm->timer, alarm_bases[type].base_clockid,
			HRTIMER_MODE_ABS);
	alarm->timer.function = alarmtimer_fired;
	alarm->function = function;
	alarm->type = type;
	alarm->state = ALARMTIMER_STATE_INACTIVE;
}

int alarm_start(struct alarm *alarm, ktime_t start)
{
	struct alarm_base *base = &alarm_bases[alarm->type];
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&base->lock, flags);
	alarm->node.expires = start;
	alarmtimer_enqueue(base, alarm);
	ret = hrtimer_start(&alarm->timer, alarm->node.expires,
				HRTIMER_MODE_ABS);
	spin_unlock_irqrestore(&base->lock, flags);
	return ret;
}

int alarm_start_relative(struct alarm *alarm, ktime_t start)
{
	struct alarm_base *base = &alarm_bases[alarm->type];

	start = ktime_add(start, base->gettime());
	return alarm_start(alarm, start);
}

void alarm_restart(struct alarm *alarm)
{
	struct alarm_base *base = &alarm_bases[alarm->type];
	unsigned long flags;

	spin_lock_irqsave(&base->lock, flags);
	hrtimer_set_expires(&alarm->timer, alarm->node.expires);
	hrtimer_restart(&alarm->timer);
	alarmtimer_enqueue(base, alarm);
	spin_unlock_irqrestore(&base->lock, flags);
}

int alarm_try_to_cancel(struct alarm *alarm)
{
	struct alarm_base *base = &alarm_bases[alarm->type];
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&base->lock, flags);
	ret = hrtimer_try_to_cancel(&alarm->timer);
	if (ret >= 0)
		alarmtimer_dequeue(base, alarm);
	spin_unlock_irqrestore(&base->lock, flags);
	return ret;
}


int alarm_cancel(struct alarm *alarm)
{
	for (;;) {
		int ret = alarm_try_to_cancel(alarm);
		if (ret >= 0)
			return ret;
		cpu_relax();
	}
}


u64 alarm_forward(struct alarm *alarm, ktime_t now, ktime_t interval)
{
	u64 overrun = 1;
	ktime_t delta;

	delta = ktime_sub(now, alarm->node.expires);

	if (delta.tv64 < 0)
		return 0;

	if (unlikely(delta.tv64 >= interval.tv64)) {
		s64 incr = ktime_to_ns(interval);

		overrun = ktime_divns(delta, incr);

		alarm->node.expires = ktime_add_ns(alarm->node.expires,
							incr*overrun);

		if (alarm->node.expires.tv64 > now.tv64)
			return overrun;
		overrun++;
	}

	alarm->node.expires = ktime_add(alarm->node.expires, interval);
	return overrun;
}

u64 alarm_forward_now(struct alarm *alarm, ktime_t interval)
{
	struct alarm_base *base = &alarm_bases[alarm->type];

	return alarm_forward(alarm, base->gettime(), interval);
}



static enum alarmtimer_type clock2alarm(clockid_t clockid)
{
	if (clockid == CLOCK_REALTIME_ALARM)
		return ALARM_REALTIME;
	if (clockid == CLOCK_BOOTTIME_ALARM)
		return ALARM_BOOTTIME;
	return -1;
}

static enum alarmtimer_restart alarm_handle_timer(struct alarm *alarm,
							ktime_t now)
{
	struct k_itimer *ptr = container_of(alarm, struct k_itimer,
						it.alarm.alarmtimer);
	if (posix_timer_event(ptr, 0) != 0)
		ptr->it_overrun++;

	
	if (ptr->it.alarm.interval.tv64) {
		ptr->it_overrun += alarm_forward(alarm, now,
						ptr->it.alarm.interval);
		return ALARMTIMER_RESTART;
	}
	return ALARMTIMER_NORESTART;
}

static int alarm_clock_getres(const clockid_t which_clock, struct timespec *tp)
{
	clockid_t baseid = alarm_bases[clock2alarm(which_clock)].base_clockid;

	if (!alarmtimer_get_rtcdev())
		return -EINVAL;

	return hrtimer_get_res(baseid, tp);
}

static int alarm_clock_get(clockid_t which_clock, struct timespec *tp)
{
	struct alarm_base *base = &alarm_bases[clock2alarm(which_clock)];

	if (!alarmtimer_get_rtcdev())
		return -EINVAL;

	*tp = ktime_to_timespec(base->gettime());
	return 0;
}

static int alarm_timer_create(struct k_itimer *new_timer)
{
	enum  alarmtimer_type type;
	struct alarm_base *base;

	if (!alarmtimer_get_rtcdev())
		return -ENOTSUPP;

	if (!capable(CAP_WAKE_ALARM))
		return -EPERM;

	type = clock2alarm(new_timer->it_clock);
	base = &alarm_bases[type];
	alarm_init(&new_timer->it.alarm.alarmtimer, type, alarm_handle_timer);
	return 0;
}

static void alarm_timer_get(struct k_itimer *timr,
				struct itimerspec *cur_setting)
{
	memset(cur_setting, 0, sizeof(struct itimerspec));

	cur_setting->it_interval =
			ktime_to_timespec(timr->it.alarm.interval);
	cur_setting->it_value =
		ktime_to_timespec(timr->it.alarm.alarmtimer.node.expires);
	return;
}

static int alarm_timer_del(struct k_itimer *timr)
{
	if (!rtcdev)
		return -ENOTSUPP;

	if (alarm_try_to_cancel(&timr->it.alarm.alarmtimer) < 0)
		return TIMER_RETRY;

	return 0;
}

static int alarm_timer_set(struct k_itimer *timr, int flags,
				struct itimerspec *new_setting,
				struct itimerspec *old_setting)
{
	if (!rtcdev)
		return -ENOTSUPP;

	if (old_setting)
		alarm_timer_get(timr, old_setting);

	
	if (alarm_try_to_cancel(&timr->it.alarm.alarmtimer) < 0)
		return TIMER_RETRY;

	
	timr->it.alarm.interval = timespec_to_ktime(new_setting->it_interval);
	alarm_start(&timr->it.alarm.alarmtimer,
			timespec_to_ktime(new_setting->it_value));
	return 0;
}

static enum alarmtimer_restart alarmtimer_nsleep_wakeup(struct alarm *alarm,
								ktime_t now)
{
	struct task_struct *task = (struct task_struct *)alarm->data;

	alarm->data = NULL;
	if (task)
		wake_up_process(task);
	return ALARMTIMER_NORESTART;
}

static int alarmtimer_do_nsleep(struct alarm *alarm, ktime_t absexp)
{
	alarm->data = (void *)current;
	do {
		set_current_state(TASK_INTERRUPTIBLE);
		alarm_start(alarm, absexp);
		if (likely(alarm->data))
			schedule();

		alarm_cancel(alarm);
	} while (alarm->data && !signal_pending(current));

	__set_current_state(TASK_RUNNING);

	return (alarm->data == NULL);
}


static int update_rmtp(ktime_t exp, enum  alarmtimer_type type,
			struct timespec __user *rmtp)
{
	struct timespec rmt;
	ktime_t rem;

	rem = ktime_sub(exp, alarm_bases[type].gettime());

	if (rem.tv64 <= 0)
		return 0;
	rmt = ktime_to_timespec(rem);

	if (copy_to_user(rmtp, &rmt, sizeof(*rmtp)))
		return -EFAULT;

	return 1;

}

static long __sched alarm_timer_nsleep_restart(struct restart_block *restart)
{
	enum  alarmtimer_type type = restart->nanosleep.clockid;
	ktime_t exp;
	struct timespec __user  *rmtp;
	struct alarm alarm;
	int ret = 0;

	exp.tv64 = restart->nanosleep.expires;
	alarm_init(&alarm, type, alarmtimer_nsleep_wakeup);

	if (alarmtimer_do_nsleep(&alarm, exp))
		goto out;

	if (freezing(current))
		alarmtimer_freezerset(exp, type);

	rmtp = restart->nanosleep.rmtp;
	if (rmtp) {
		ret = update_rmtp(exp, type, rmtp);
		if (ret <= 0)
			goto out;
	}


	
	ret = -ERESTART_RESTARTBLOCK;
out:
	return ret;
}

static int alarm_timer_nsleep(const clockid_t which_clock, int flags,
		     struct timespec *tsreq, struct timespec __user *rmtp)
{
	enum  alarmtimer_type type = clock2alarm(which_clock);
	struct alarm alarm;
	ktime_t exp;
	int ret = 0;
	struct restart_block *restart;

	if (!alarmtimer_get_rtcdev())
		return -ENOTSUPP;

	if (!capable(CAP_WAKE_ALARM))
		return -EPERM;

	alarm_init(&alarm, type, alarmtimer_nsleep_wakeup);

	exp = timespec_to_ktime(*tsreq);
	
	if (flags != TIMER_ABSTIME) {
		ktime_t now = alarm_bases[type].gettime();
		exp = ktime_add(now, exp);
	}

	if (alarmtimer_do_nsleep(&alarm, exp))
		goto out;

	if (freezing(current))
		alarmtimer_freezerset(exp, type);

	
	if (flags == TIMER_ABSTIME) {
		ret = -ERESTARTNOHAND;
		goto out;
	}

	if (rmtp) {
		ret = update_rmtp(exp, type, rmtp);
		if (ret <= 0)
			goto out;
	}

	restart = &current_thread_info()->restart_block;
	restart->fn = alarm_timer_nsleep_restart;
	restart->nanosleep.clockid = type;
	restart->nanosleep.expires = exp.tv64;
	restart->nanosleep.rmtp = rmtp;
	ret = -ERESTART_RESTARTBLOCK;

out:
	return ret;
}


static const struct dev_pm_ops alarmtimer_pm_ops = {
	.suspend = alarmtimer_suspend,
	.resume = alarmtimer_resume,
};

static struct platform_driver alarmtimer_driver = {
	.shutdown = alarm_shutdown,
	.driver = {
		.name = "alarmtimer",
		.pm = &alarmtimer_pm_ops,
	}
};

static int __init alarmtimer_init(void)
{
	struct platform_device *pdev;
	int error = 0;
	int i;
	struct k_clock alarm_clock = {
		.clock_getres	= alarm_clock_getres,
		.clock_get	= alarm_clock_get,
		.timer_create	= alarm_timer_create,
		.timer_set	= alarm_timer_set,
		.timer_del	= alarm_timer_del,
		.timer_get	= alarm_timer_get,
		.nsleep		= alarm_timer_nsleep,
	};

	alarmtimer_rtc_timer_init();

	posix_timers_register_clock(CLOCK_REALTIME_ALARM, &alarm_clock);
	posix_timers_register_clock(CLOCK_BOOTTIME_ALARM, &alarm_clock);

	
	alarm_bases[ALARM_REALTIME].base_clockid = CLOCK_REALTIME;
	alarm_bases[ALARM_REALTIME].gettime = &ktime_get_real;
	alarm_bases[ALARM_BOOTTIME].base_clockid = CLOCK_BOOTTIME;
	alarm_bases[ALARM_BOOTTIME].gettime = &ktime_get_boottime;
	for (i = 0; i < ALARM_NUMTYPE; i++) {
		timerqueue_init_head(&alarm_bases[i].timerqueue);
		spin_lock_init(&alarm_bases[i].lock);
	}

	error = alarmtimer_rtc_interface_setup();
	if (error)
		return error;

	error = platform_driver_register(&alarmtimer_driver);
	if (error)
		goto out_if;

	pdev = platform_device_register_simple("alarmtimer", -1, NULL, 0);
	if (IS_ERR(pdev)) {
		error = PTR_ERR(pdev);
		goto out_drv;
	}
	ws = wakeup_source_register("alarmtimer");
	return 0;

out_drv:
	platform_driver_unregister(&alarmtimer_driver);
out_if:
	alarmtimer_rtc_interface_remove();
	return error;
}
device_initcall(alarmtimer_init);
