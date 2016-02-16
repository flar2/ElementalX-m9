/*
 * kernel/power/suspend.c - Suspend to RAM and standby functionality.
 *
 * Copyright (c) 2003 Patrick Mochel
 * Copyright (c) 2003 Open Source Development Lab
 * Copyright (c) 2009 Rafael J. Wysocki <rjw@sisk.pl>, Novell Inc.
 *
 * This file is released under the GPLv2.
 */

#include <linux/string.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/cpu.h>
#include <linux/syscalls.h>
#include <linux/gfp.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/export.h>
#include <linux/suspend.h>
#include <linux/syscore_ops.h>
#include <linux/ftrace.h>
#include <linux/rtc.h>
#include <trace/events/power.h>
#include <linux/wakeup_reason.h>

#include "power.h"
#include <soc/qcom/smsm.h>

struct pm_sleep_state pm_states[PM_SUSPEND_MAX] = {
	[PM_SUSPEND_FREEZE] = { .label = "freeze", .state = PM_SUSPEND_FREEZE },
	[PM_SUSPEND_STANDBY] = { .label = "standby", },
	[PM_SUSPEND_MEM] = { .label = "mem", },
};

static const struct platform_suspend_ops *suspend_ops;

static bool need_suspend_ops(suspend_state_t state)
{
	return !!(state > PM_SUSPEND_FREEZE);
}

static DECLARE_WAIT_QUEUE_HEAD(suspend_freeze_wait_head);
static bool suspend_freeze_wake;

static void freeze_begin(void)
{
	suspend_freeze_wake = false;
}

static void freeze_enter(void)
{
	wait_event(suspend_freeze_wait_head, suspend_freeze_wake);
}

void freeze_wake(void)
{
	suspend_freeze_wake = true;
	wake_up(&suspend_freeze_wait_head);
}
EXPORT_SYMBOL_GPL(freeze_wake);

static bool valid_state(suspend_state_t state)
{
	return suspend_ops && suspend_ops->valid && suspend_ops->valid(state);
}

void suspend_set_ops(const struct platform_suspend_ops *ops)
{
	suspend_state_t i;

	lock_system_sleep();

	suspend_ops = ops;
	for (i = PM_SUSPEND_STANDBY; i <= PM_SUSPEND_MEM; i++)
		pm_states[i].state = valid_state(i) ? i : 0;

	unlock_system_sleep();
}
EXPORT_SYMBOL_GPL(suspend_set_ops);

int suspend_valid_only_mem(suspend_state_t state)
{
	return state == PM_SUSPEND_MEM;
}
EXPORT_SYMBOL_GPL(suspend_valid_only_mem);

static int suspend_test(int level)
{
#ifdef CONFIG_PM_DEBUG
	if (pm_test_level == level) {
		printk(KERN_INFO "suspend debug: Waiting for 5 seconds.\n");
		mdelay(5000);
		return 1;
	}
#endif 
	return 0;
}

static int suspend_prepare(suspend_state_t state)
{
	int error;

	if (need_suspend_ops(state) && (!suspend_ops || !suspend_ops->enter))
		return -EPERM;

	pm_prepare_console();

	error = pm_notifier_call_chain(PM_SUSPEND_PREPARE);
	if (error)
		goto Finish;

	error = suspend_freeze_processes();
	if (!error)
		return 0;
	log_suspend_abort_reason("One or more tasks refusing to freeze");
	suspend_stats.failed_freeze++;
	dpm_save_failed_step(SUSPEND_FREEZE);
 Finish:
	pm_notifier_call_chain(PM_POST_SUSPEND);
	pm_restore_console();
	return error;
}

void __attribute__ ((weak)) arch_suspend_disable_irqs(void)
{
	local_irq_disable();
}

void __attribute__ ((weak)) arch_suspend_enable_irqs(void)
{
	local_irq_enable();
}

static int suspend_enter(suspend_state_t state, bool *wakeup)
{
	char suspend_abort[MAX_SUSPEND_ABORT_LEN];
	int error, last_dev;

	if (need_suspend_ops(state) && suspend_ops->prepare) {
		error = suspend_ops->prepare();
		if (error)
			goto Platform_finish;
	}

	error = dpm_suspend_end(PMSG_SUSPEND);
	if (error) {
		last_dev = suspend_stats.last_failed_dev + REC_FAILED_NUM - 1;
		last_dev %= REC_FAILED_NUM;
		printk(KERN_ERR "PM: Some devices failed to power down\n");
		log_suspend_abort_reason("%s device failed to power down",
			suspend_stats.failed_devs[last_dev]);
		goto Platform_finish;
	}

	if (need_suspend_ops(state) && suspend_ops->prepare_late) {
		error = suspend_ops->prepare_late();
		if (error)
			goto Platform_wake;
	}

	if (suspend_test(TEST_PLATFORM))
		goto Platform_wake;

	if (state == PM_SUSPEND_FREEZE) {
		freeze_enter();
		goto Platform_wake;
	}

	error = disable_nonboot_cpus();
	if (error || suspend_test(TEST_CPUS)) {
		log_suspend_abort_reason("Disabling non-boot cpus failed");
		goto Enable_cpus;
	}

	arch_suspend_disable_irqs();
	BUG_ON(!irqs_disabled());

	error = syscore_suspend();
	if (!error) {
		*wakeup = pm_wakeup_pending();
		if (!(suspend_test(TEST_CORE) || *wakeup)) {
			error = suspend_ops->enter(state);
			events_check_enabled = false;
		} else if (*wakeup) {
			pm_get_active_wakeup_sources(suspend_abort,
				MAX_SUSPEND_ABORT_LEN);
			log_suspend_abort_reason(suspend_abort);
			error = -EBUSY;
		}
		syscore_resume();
	}

	arch_suspend_enable_irqs();
	BUG_ON(irqs_disabled());

 Enable_cpus:
	enable_nonboot_cpus();

 Platform_wake:
	if (need_suspend_ops(state) && suspend_ops->wake)
		suspend_ops->wake();

	dpm_resume_start(PMSG_RESUME);

 Platform_finish:
	if (need_suspend_ops(state) && suspend_ops->finish)
		suspend_ops->finish();

	return error;
}

int suspend_devices_and_enter(suspend_state_t state)
{
	int error;
	bool wakeup = false;

	if (need_suspend_ops(state) && !suspend_ops)
		return -ENOSYS;

	trace_machine_suspend(state);
	if (need_suspend_ops(state) && suspend_ops->begin) {
		error = suspend_ops->begin(state);
		if (error)
			goto Close;
	}
	if (!suspend_console_deferred)
		suspend_console();
	ftrace_stop();
	suspend_test_start();
	error = dpm_suspend_start(PMSG_SUSPEND);
	if (error) {
		printk(KERN_ERR "PM: Some devices failed to suspend\n");
		log_suspend_abort_reason("Some devices failed to suspend");
		goto Recover_platform;
	}
	suspend_test_finish("suspend devices");
	if (suspend_test(TEST_DEVICES))
		goto Recover_platform;

	do {
		error = suspend_enter(state, &wakeup);
	} while (!error && !wakeup && need_suspend_ops(state)
		&& suspend_ops->suspend_again && suspend_ops->suspend_again());

 Resume_devices:
	suspend_test_start();
	dpm_resume_end(PMSG_RESUME);
	suspend_test_finish("resume devices");
	ftrace_start();
	if (!suspend_console_deferred)
		resume_console();
 Close:
	if (need_suspend_ops(state) && suspend_ops->end)
		suspend_ops->end();
	trace_machine_suspend(PWR_EVENT_EXIT);
	return error;

 Recover_platform:
	if (need_suspend_ops(state) && suspend_ops->recover)
		suspend_ops->recover();
	goto Resume_devices;
}

static void suspend_finish(void)
{
	suspend_thaw_processes();
	pm_notifier_call_chain(PM_POST_SUSPEND);
	pm_restore_console();
}

static int enter_state(suspend_state_t state)
{
	int error;

	if (state == PM_SUSPEND_FREEZE) {
#ifdef CONFIG_PM_DEBUG
		if (pm_test_level != TEST_NONE && pm_test_level <= TEST_CPUS) {
			pr_warning("PM: Unsupported test mode for freeze state,"
				   "please choose none/freezer/devices/platform.\n");
			return -EAGAIN;
		}
#endif
	} else if (!valid_state(state)) {
		return -EINVAL;
	}
	if (!mutex_trylock(&pm_mutex))
		return -EBUSY;

	if (state == PM_SUSPEND_FREEZE)
		freeze_begin();

	suspend_sys_sync_queue();
	pr_debug("PM: Preparing system for %s sleep\n", pm_states[state].label);
	error = suspend_prepare(state);
	if (error)
		goto Unlock;

	if (suspend_test(TEST_FREEZER))
		goto Finish;

	pr_debug("PM: Entering %s sleep\n", pm_states[state].label);
	pm_restrict_gfp_mask();
	error = suspend_devices_and_enter(state);
	pm_restore_gfp_mask();

 Finish:
	pr_debug("PM: Finishing wakeup.\n");
	suspend_finish();
 Unlock:
	mutex_unlock(&pm_mutex);
	return error;
}

static void pm_suspend_marker(char *annotation)
{
	struct timespec ts;
	struct rtc_time tm;

	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec, &tm);
	pr_info("PM: suspend %s %d-%02d-%02d %02d:%02d:%02d.%09lu UTC\n",
		annotation, tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
		tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);
}

int pm_suspend(suspend_state_t state)
{
	int error;

	if (state <= PM_SUSPEND_ON || state >= PM_SUSPEND_MAX)
		return -EINVAL;

	smsm_change_state(SMSM_APPS_STATE, HTC_SMSM_APPS_RESUME, HTC_SMSM_APPS_SUSPEND);
	printk(KERN_DEBUG "Enable garbage filter\n");
	pm_suspend_marker("entry");
	error = enter_state(state);
	if (error) {
		suspend_stats.fail++;
		dpm_save_failed_errno(error);
	} else {
		suspend_stats.success++;
	}
	pm_suspend_marker("exit");
	smsm_change_state(SMSM_APPS_STATE, HTC_SMSM_APPS_SUSPEND, HTC_SMSM_APPS_RESUME);
	printk(KERN_DEBUG "Disable garbage filter\n");
	return error;
}
EXPORT_SYMBOL(pm_suspend);
