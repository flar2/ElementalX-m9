/* Copyright (c) 2010-2015, The Linux Foundation. All rights reserved.
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

#include <linux/export.h>
#include <linux/interrupt.h>
#include <asm/page.h>
#include <linux/pm_runtime.h>
#include <linux/msm-bus.h>
#include <linux/msm-bus-board.h>
#include <linux/ktime.h>
#include <linux/delay.h>
#include <linux/msm_adreno_devfreq.h>
#include <linux/of_device.h>

#include "kgsl.h"
#include "kgsl_pwrscale.h"
#include "kgsl_device.h"
#include "kgsl_trace.h"
#include <soc/qcom/devfreq_devbw.h>

#define KGSL_PWRFLAGS_POWER_ON 0
#define KGSL_PWRFLAGS_CLK_ON   1
#define KGSL_PWRFLAGS_AXI_ON   2
#define KGSL_PWRFLAGS_IRQ_ON   3
#define KGSL_PWRFLAGS_WAKEUP_PWRLEVEL  24

#define UPDATE_BUSY_VAL		1000000

#define INIT_UDELAY		200
#define MAX_UDELAY		2000

#define TH_HZ			20

#define KGSL_MAX_BUSLEVELS	20

#define DEFAULT_BUS_P 25
#define DEFAULT_BUS_DIV (100 / DEFAULT_BUS_P)

struct clk_pair {
	const char *name;
	uint map;
};

static struct clk_pair clks[KGSL_MAX_CLKS] = {
	{
		.name = "src_clk",
		.map = KGSL_CLK_SRC,
	},
	{
		.name = "core_clk",
		.map = KGSL_CLK_CORE,
	},
	{
		.name = "iface_clk",
		.map = KGSL_CLK_IFACE,
	},
	{
		.name = "mem_clk",
		.map = KGSL_CLK_MEM,
	},
	{
		.name = "mem_iface_clk",
		.map = KGSL_CLK_MEM_IFACE,
	},
	{
		.name = "alt_mem_iface_clk",
		.map = KGSL_CLK_ALT_MEM_IFACE,
	},
	{
		.name = "rbbmtimer_clk",
		.map = KGSL_CLK_RBBMTIMER,
	},
	{
		.name = "gtcu_clk",
		.map = KGSL_CLK_GFX_GTCU,
	},
	{
		.name = "gtbu_clk",
		.map = KGSL_CLK_GFX_GTBU,
	},
};

static unsigned int ib_votes[KGSL_MAX_BUSLEVELS];
static int last_vote_buslevel;
static int max_vote_buslevel;

static void kgsl_pwrctrl_clk(struct kgsl_device *device, int state,
					int requested_state);
static void kgsl_pwrctrl_axi(struct kgsl_device *device, int state);
static int kgsl_pwrctrl_pwrrail(struct kgsl_device *device, int state);
static void kgsl_pwrctrl_set_state(struct kgsl_device *device,
				unsigned int state);
static void kgsl_pwrctrl_request_state(struct kgsl_device *device,
				unsigned int state);

static void _record_pwrevent(struct kgsl_device *device,
			ktime_t t, int event) {
	struct kgsl_pwrscale *psc = &device->pwrscale;
	struct kgsl_pwr_history *history = &psc->history[event];
	int i = history->index;
	if (history->events == NULL)
		return;
	history->events[i].duration = ktime_us_delta(t,
					history->events[i].start);
	i = (i + 1) % history->size;
	history->index = i;
	history->events[i].start = t;
	switch (event) {
	case KGSL_PWREVENT_STATE:
		history->events[i].data = device->state;
		break;
	case KGSL_PWREVENT_GPU_FREQ:
		history->events[i].data = device->pwrctrl.active_pwrlevel;
		break;
	case KGSL_PWREVENT_BUS_FREQ:
		history->events[i].data = last_vote_buslevel;
		break;
	default:
		break;
	}
}

static unsigned int kgsl_get_bw(void)
{
	return ib_votes[last_vote_buslevel];
}

static void _ab_buslevel_update(struct kgsl_pwrctrl *pwr,
				unsigned long *ab)
{
	unsigned int ib = ib_votes[last_vote_buslevel];
	unsigned int max_bw = ib_votes[max_vote_buslevel];
	if (!ab)
		return;
	if (ib == 0)
		*ab = 0;
	else if (!pwr->bus_percent_ab)
		*ab = DEFAULT_BUS_P * ib / 100;
	else
		*ab = (pwr->bus_percent_ab * max_bw) / 100;

	if (*ab > ib)
		*ab = ib;
}

static unsigned int _adjust_pwrlevel(struct kgsl_pwrctrl *pwr, int level,
					struct kgsl_pwr_constraint *pwrc,
					int popp)
{
	unsigned int max_pwrlevel = max_t(unsigned int, pwr->thermal_pwrlevel,
		pwr->max_pwrlevel);
	unsigned int min_pwrlevel = max_t(unsigned int, pwr->thermal_pwrlevel,
		pwr->min_pwrlevel);

	switch (pwrc->type) {
	case KGSL_CONSTRAINT_PWRLEVEL: {
		switch (pwrc->sub_type) {
		case KGSL_CONSTRAINT_PWR_MAX:
			return max_pwrlevel;
			break;
		case KGSL_CONSTRAINT_PWR_MIN:
			return min_pwrlevel;
			break;
		default:
			break;
		}
	}
	break;
	}

	if (popp && (max_pwrlevel < pwr->active_pwrlevel))
		max_pwrlevel = pwr->active_pwrlevel;

	if (level < max_pwrlevel)
		return max_pwrlevel;
	if (level > min_pwrlevel)
		return min_pwrlevel;

	return level;
}

void kgsl_pwrctrl_buslevel_update(struct kgsl_device *device,
			bool on)
{
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	int cur = pwr->pwrlevels[pwr->active_pwrlevel].bus_freq;
	int buslevel = 0;
	unsigned long ab;

	
	if (on && !(test_bit(KGSL_PWRFLAGS_AXI_ON, &pwr->power_flags)))
		return;
	if (on) {
		buslevel = min_t(int, pwr->pwrlevels[0].bus_max,
				cur + pwr->bus_mod);
		buslevel = max_t(int, buslevel, 1);
	} else {
		
		pwr->bus_mod = 0;
	}
	trace_kgsl_buslevel(device, pwr->active_pwrlevel, buslevel);
	last_vote_buslevel = buslevel;

	
	_ab_buslevel_update(pwr, &ab);

	if (pwr->ocmem_pcl)
		msm_bus_scale_client_update_request(pwr->ocmem_pcl,
			on ? pwr->active_pwrlevel : pwr->num_pwrlevels - 1);

	
	if (pwr->pcl)
		msm_bus_scale_client_update_request(pwr->pcl, buslevel);

	
	devfreq_vbif_update_bw(ib_votes[last_vote_buslevel], ab);
}
EXPORT_SYMBOL(kgsl_pwrctrl_buslevel_update);

void kgsl_pwrctrl_pwrlevel_change_settings(struct kgsl_device *device,
						bool post, bool wake_up)
{
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	unsigned int old = pwr->previous_pwrlevel;
	unsigned int new = pwr->active_pwrlevel;

	if (device->state != KGSL_STATE_ACTIVE)
		return;
	if (old == new)
		return;
	if (!device->ftbl->pwrlevel_change_settings)
		return;

	if (((old == 0) && !post) || ((old == 0) && wake_up)) {
		
		device->ftbl->pwrlevel_change_settings(device, 1);
	} else if (((new == 0) && post) || ((new == 0) && wake_up)) {
		
		device->ftbl->pwrlevel_change_settings(device, 0);
	}
}

void kgsl_pwrctrl_set_thermal_cycle(struct kgsl_pwrctrl *pwr,
						unsigned int new_level)
{
	if (new_level != pwr->thermal_pwrlevel)
		return;
	if (pwr->thermal_pwrlevel == pwr->sysfs_pwr_limit->level) {
		
		if (pwr->thermal_cycle == CYCLE_ENABLE) {
			pwr->thermal_cycle = CYCLE_ACTIVE;
			mod_timer(&pwr->thermal_timer, jiffies +
					(TH_HZ - pwr->thermal_timeout));
			pwr->thermal_highlow = 1;
		}
	} else {
		
		if (pwr->thermal_cycle == CYCLE_ACTIVE) {
			pwr->thermal_cycle = CYCLE_ENABLE;
			del_timer_sync(&pwr->thermal_timer);
		}
	}
}

void kgsl_pwrctrl_pwrlevel_change(struct kgsl_device *device,
				unsigned int new_level)
{
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	struct kgsl_pwrlevel *pwrlevel;
	unsigned int old_level = pwr->active_pwrlevel;

	
	if ((pwr->constraint.type != KGSL_CONSTRAINT_NONE) &&
		(time_after(jiffies, pwr->constraint.expires))) {
		
		trace_kgsl_constraint(device, pwr->constraint.type,
						old_level, 0);
		
		pwr->constraint.expires = 0;
		pwr->constraint.type = KGSL_CONSTRAINT_NONE;
	}

	new_level = _adjust_pwrlevel(pwr, new_level, &pwr->constraint,
					device->pwrscale.popp_level);

	if (new_level == 0 && test_bit(KGSL_PWRFLAGS_WAKEUP_PWRLEVEL,
		&device->pwrctrl.ctrl_flags))
		new_level = 1;

	kgsl_pwrctrl_set_thermal_cycle(pwr, new_level);

	if (new_level == old_level)
		return;

	pwr->active_pwrlevel = new_level;
	pwr->previous_pwrlevel = old_level;

	if (pwr->bus_mod < 0 || new_level < old_level) {
		pwr->bus_mod = 0;
		pwr->bus_percent_ab = 0;
	}
	kgsl_pwrctrl_buslevel_update(device, true);

	pwrlevel = &pwr->pwrlevels[pwr->active_pwrlevel];
	
	kgsl_pwrctrl_pwrlevel_change_settings(device, 0, 0);
	clk_set_rate(pwr->grp_clks[0], pwrlevel->gpu_freq);
	trace_kgsl_pwrlevel(device, pwr->active_pwrlevel,
			pwrlevel->gpu_freq);
	
	kgsl_pwrctrl_pwrlevel_change_settings(device, 1, 0);

	
	device->pwrscale.freq_change_time = ktime_to_ms(ktime_get());
}
EXPORT_SYMBOL(kgsl_pwrctrl_pwrlevel_change);

void kgsl_pwrctrl_set_constraint(struct kgsl_device *device,
			struct kgsl_pwr_constraint *pwrc, uint32_t id)
{
	unsigned int constraint;
	struct kgsl_pwr_constraint *pwrc_old;

	if (device == NULL || pwrc == NULL)
		return;
	constraint = _adjust_pwrlevel(&device->pwrctrl,
				device->pwrctrl.active_pwrlevel, pwrc, 0);
	pwrc_old = &device->pwrctrl.constraint;

	if ((pwrc_old->type == KGSL_CONSTRAINT_NONE) ||
		(constraint < pwrc_old->hint.pwrlevel.level)) {
		pwrc_old->type = pwrc->type;
		pwrc_old->sub_type = pwrc->sub_type;
		pwrc_old->hint.pwrlevel.level = constraint;
		pwrc_old->owner_id = id;
		pwrc_old->expires = jiffies + device->pwrctrl.interval_timeout;
		kgsl_pwrctrl_pwrlevel_change(device, constraint);
		
		trace_kgsl_constraint(device, pwrc_old->type, constraint, 1);
	} else if ((pwrc_old->type == pwrc->type) &&
		(pwrc_old->hint.pwrlevel.level == constraint)) {
			pwrc_old->owner_id = id;
			pwrc_old->expires = jiffies +
					device->pwrctrl.interval_timeout;
	}
}
EXPORT_SYMBOL(kgsl_pwrctrl_set_constraint);

static ssize_t kgsl_pwrctrl_thermal_pwrlevel_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	struct kgsl_pwrctrl *pwr;
	int ret;
	unsigned int level = 0;

	if (device == NULL)
		return 0;

	pwr = &device->pwrctrl;

	ret = kgsl_sysfs_store(buf, &level);

	if (ret)
		return ret;

	mutex_lock(&device->mutex);

	if (level > pwr->num_pwrlevels - 2)
		level = pwr->num_pwrlevels - 2;

	pwr->thermal_pwrlevel = level;

	
	kgsl_pwrctrl_pwrlevel_change(device, pwr->active_pwrlevel);
	mutex_unlock(&device->mutex);

	return count;
}

static ssize_t kgsl_pwrctrl_thermal_pwrlevel_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{

	struct kgsl_device *device = kgsl_device_from_dev(dev);
	struct kgsl_pwrctrl *pwr;
	if (device == NULL)
		return 0;
	pwr = &device->pwrctrl;
	return snprintf(buf, PAGE_SIZE, "%d\n", pwr->thermal_pwrlevel);
}

static ssize_t kgsl_pwrctrl_max_pwrlevel_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	struct kgsl_pwrctrl *pwr;
	int ret;
	unsigned int level = 0;

	if (device == NULL)
		return 0;

	pwr = &device->pwrctrl;

	ret = kgsl_sysfs_store(buf, &level);
	if (ret)
		return ret;

	mutex_lock(&device->mutex);

	
	if (level > pwr->min_pwrlevel)
		level = pwr->min_pwrlevel;

	pwr->max_pwrlevel = level;

	
	kgsl_pwrctrl_pwrlevel_change(device, pwr->active_pwrlevel);
	mutex_unlock(&device->mutex);

	return count;
}

static ssize_t kgsl_pwrctrl_max_pwrlevel_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{

	struct kgsl_device *device = kgsl_device_from_dev(dev);
	struct kgsl_pwrctrl *pwr;
	if (device == NULL)
		return 0;
	pwr = &device->pwrctrl;
	return snprintf(buf, PAGE_SIZE, "%u\n", pwr->max_pwrlevel);
}

static ssize_t kgsl_pwrctrl_min_pwrlevel_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{	struct kgsl_device *device = kgsl_device_from_dev(dev);
	struct kgsl_pwrctrl *pwr;
	int ret;
	unsigned int level = 0;

	if (device == NULL)
		return 0;

	pwr = &device->pwrctrl;

	ret = kgsl_sysfs_store(buf, &level);
	if (ret)
		return ret;

	mutex_lock(&device->mutex);
	if (level > pwr->num_pwrlevels - 2)
		level = pwr->num_pwrlevels - 2;

	
	if (level < pwr->max_pwrlevel)
		level = pwr->max_pwrlevel;

	pwr->min_pwrlevel = level;

	
	kgsl_pwrctrl_pwrlevel_change(device, pwr->active_pwrlevel);

	mutex_unlock(&device->mutex);

	return count;
}

static ssize_t kgsl_pwrctrl_min_pwrlevel_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	struct kgsl_pwrctrl *pwr;
	if (device == NULL)
		return 0;
	pwr = &device->pwrctrl;
	return snprintf(buf, PAGE_SIZE, "%u\n", pwr->min_pwrlevel);
}

static ssize_t kgsl_pwrctrl_num_pwrlevels_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{

	struct kgsl_device *device = kgsl_device_from_dev(dev);
	struct kgsl_pwrctrl *pwr;
	if (device == NULL)
		return 0;
	pwr = &device->pwrctrl;
	return snprintf(buf, PAGE_SIZE, "%d\n", pwr->num_pwrlevels - 1);
}


static int _get_nearest_pwrlevel(struct kgsl_pwrctrl *pwr, unsigned int clock)
{
	int i;

	for (i = pwr->num_pwrlevels - 1; i >= 0; i--) {
		if (abs(pwr->pwrlevels[i].gpu_freq - clock) < 5000000)
			return i;
	}

	return -ERANGE;
}

static ssize_t kgsl_pwrctrl_max_gpuclk_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	struct kgsl_pwrctrl *pwr;
	unsigned int val = 0;
	int level, ret;

	if (device == NULL)
		return 0;

	pwr = &device->pwrctrl;

	ret = kgsl_sysfs_store(buf, &val);
	if (ret)
		return ret;

	mutex_lock(&device->mutex);
	level = _get_nearest_pwrlevel(pwr, val);
	
	if (level < 0) {
		unsigned int hfreq, diff, udiff, i;
		if ((val < pwr->pwrlevels[pwr->num_pwrlevels - 1].gpu_freq) ||
			(val > pwr->pwrlevels[0].gpu_freq))
			goto err;

		
		for (i = 0; i < pwr->num_pwrlevels - 1; i++) {
			if ((pwr->pwrlevels[i].gpu_freq > val) &&
				(pwr->pwrlevels[i + 1].gpu_freq < val)) {
				level = i;
				break;
			}
		}
		if (i == pwr->num_pwrlevels - 1)
			goto err;
		hfreq = pwr->pwrlevels[i].gpu_freq;
		diff =  hfreq - pwr->pwrlevels[i + 1].gpu_freq;
		udiff = hfreq - val;
		pwr->thermal_timeout = (udiff * TH_HZ) / diff;
		pwr->thermal_cycle = CYCLE_ENABLE;
	} else {
		pwr->thermal_cycle = CYCLE_DISABLE;
		del_timer_sync(&pwr->thermal_timer);
	}
	mutex_unlock(&device->mutex);

	if (pwr->sysfs_pwr_limit)
		kgsl_pwr_limits_set_freq(pwr->sysfs_pwr_limit,
					pwr->pwrlevels[level].gpu_freq);
	return count;

err:
	mutex_unlock(&device->mutex);
	return count;
}

static ssize_t kgsl_pwrctrl_max_gpuclk_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{

	struct kgsl_device *device = kgsl_device_from_dev(dev);
	struct kgsl_pwrctrl *pwr;
	unsigned int freq;
	if (device == NULL)
		return 0;
	pwr = &device->pwrctrl;
	freq = pwr->pwrlevels[pwr->thermal_pwrlevel].gpu_freq;
	
	if (pwr->thermal_cycle) {
		unsigned int hfreq = freq;
		unsigned int lfreq = pwr->pwrlevels[pwr->
				thermal_pwrlevel + 1].gpu_freq;
		freq = pwr->thermal_timeout * (lfreq / TH_HZ) +
			(TH_HZ - pwr->thermal_timeout) * (hfreq / TH_HZ);
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", freq);
}

static ssize_t kgsl_pwrctrl_gpuclk_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	struct kgsl_pwrctrl *pwr;
	unsigned int val = 0;
	int ret, level;

	if (device == NULL)
		return 0;

	pwr = &device->pwrctrl;

	ret = kgsl_sysfs_store(buf, &val);
	if (ret)
		return ret;

	mutex_lock(&device->mutex);
	level = _get_nearest_pwrlevel(pwr, val);
	if (level >= 0)
		kgsl_pwrctrl_pwrlevel_change(device, (unsigned int) level);

	mutex_unlock(&device->mutex);
	return count;
}

static ssize_t kgsl_pwrctrl_gpuclk_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	struct kgsl_pwrctrl *pwr;
	if (device == NULL)
		return 0;
	pwr = &device->pwrctrl;
	return snprintf(buf, PAGE_SIZE, "%ld\n", kgsl_pwrctrl_active_freq(pwr));
}

static ssize_t kgsl_pwrctrl_idle_timer_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	unsigned int val = 0;
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	int ret;

	if (device == NULL)
		return 0;

	ret = kgsl_sysfs_store(buf, &val);
	if (ret)
		return ret;


	if (val > jiffies_to_usecs(MAX_JIFFY_OFFSET))
		return -EINVAL;

	mutex_lock(&device->mutex);

	
	device->pwrctrl.interval_timeout = msecs_to_jiffies(val);

	mutex_unlock(&device->mutex);

	return count;
}

static ssize_t kgsl_pwrctrl_idle_timer_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	if (device == NULL)
		return 0;
	
	return snprintf(buf, PAGE_SIZE, "%u\n",
		jiffies_to_msecs(device->pwrctrl.interval_timeout));
}

static ssize_t kgsl_pwrctrl_pmqos_active_latency_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	unsigned int val = 0;
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	int ret;

	if (device == NULL)
		return 0;

	ret = kgsl_sysfs_store(buf, &val);
	if (ret)
		return ret;

	mutex_lock(&device->mutex);
	device->pwrctrl.pm_qos_active_latency = val;
	mutex_unlock(&device->mutex);

	return count;
}

static ssize_t kgsl_pwrctrl_pmqos_active_latency_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	if (device == NULL)
		return 0;
	return snprintf(buf, PAGE_SIZE, "%d\n",
		device->pwrctrl.pm_qos_active_latency);
}

static ssize_t kgsl_pwrctrl_gpubusy_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int ret;
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	struct kgsl_clk_stats *stats;

	if (device == NULL)
		return 0;
	stats = &device->pwrctrl.clk_stats;
	ret = snprintf(buf, PAGE_SIZE, "%7d %7d\n",
			stats->busy_old, stats->total_old);
	if (!test_bit(KGSL_PWRFLAGS_AXI_ON, &device->pwrctrl.power_flags)) {
		stats->busy_old = 0;
		stats->total_old = 0;
	}
	return ret;
}

static ssize_t kgsl_pwrctrl_gpu_available_frequencies_show(
					struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	struct kgsl_pwrctrl *pwr;
	int index, num_chars = 0;

	if (device == NULL)
		return 0;
	pwr = &device->pwrctrl;
	for (index = 0; index < pwr->num_pwrlevels - 1; index++)
		num_chars += snprintf(buf + num_chars, PAGE_SIZE, "%d ",
		pwr->pwrlevels[index].gpu_freq);
	buf[num_chars++] = '\n';
	return num_chars;
}

static ssize_t kgsl_pwrctrl_reset_count_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	if (device == NULL)
		return 0;
	return snprintf(buf, PAGE_SIZE, "%d\n", device->reset_counter);
}

static void __force_on(struct kgsl_device *device, int flag, int on)
{
	if (on) {
		switch (flag) {
		case KGSL_PWRFLAGS_CLK_ON:
			kgsl_pwrctrl_clk(device, KGSL_PWRFLAGS_ON,
				KGSL_STATE_ACTIVE);
			break;
		case KGSL_PWRFLAGS_AXI_ON:
			kgsl_pwrctrl_axi(device, KGSL_PWRFLAGS_ON);
			break;
		case KGSL_PWRFLAGS_POWER_ON:
			kgsl_pwrctrl_pwrrail(device, KGSL_PWRFLAGS_ON);
			break;
		}
		set_bit(flag, &device->pwrctrl.ctrl_flags);
	} else {
		clear_bit(flag, &device->pwrctrl.ctrl_flags);
	}
}

static ssize_t __force_on_show(struct device *dev,
					struct device_attribute *attr,
					char *buf, int flag)
{
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	if (device == NULL)
		return 0;
	return snprintf(buf, PAGE_SIZE, "%d\n",
		test_bit(flag, &device->pwrctrl.ctrl_flags));
}

static ssize_t __force_on_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count,
					int flag)
{
	unsigned int val = 0;
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	int ret;

	if (device == NULL)
		return 0;

	ret = kgsl_sysfs_store(buf, &val);
	if (ret)
		return ret;

	mutex_lock(&device->mutex);
	__force_on(device, flag, val);
	mutex_unlock(&device->mutex);

	return count;
}

static ssize_t kgsl_pwrctrl_force_clk_on_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	return __force_on_show(dev, attr, buf, KGSL_PWRFLAGS_CLK_ON);
}

static ssize_t kgsl_pwrctrl_force_clk_on_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	return __force_on_store(dev, attr, buf, count, KGSL_PWRFLAGS_CLK_ON);
}

static ssize_t kgsl_pwrctrl_force_bus_on_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	return __force_on_show(dev, attr, buf, KGSL_PWRFLAGS_AXI_ON);
}

static ssize_t kgsl_pwrctrl_force_bus_on_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	return __force_on_store(dev, attr, buf, count, KGSL_PWRFLAGS_AXI_ON);
}

static ssize_t kgsl_pwrctrl_force_rail_on_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	return __force_on_show(dev, attr, buf, KGSL_PWRFLAGS_POWER_ON);
}

static ssize_t kgsl_pwrctrl_force_rail_on_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	return __force_on_store(dev, attr, buf, count, KGSL_PWRFLAGS_POWER_ON);
}

static ssize_t kgsl_pwrctrl_bus_split_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	if (device == NULL)
		return 0;
	return snprintf(buf, PAGE_SIZE, "%d\n",
		device->pwrctrl.bus_control);
}

static ssize_t kgsl_pwrctrl_bus_split_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	unsigned int val = 0;
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	int ret;

	if (device == NULL)
		return 0;

	ret = kgsl_sysfs_store(buf, &val);
	if (ret)
		return ret;

	mutex_lock(&device->mutex);
	device->pwrctrl.bus_control = val ? true : false;
	mutex_unlock(&device->mutex);

	return count;
}

static ssize_t kgsl_pwrctrl_default_pwrlevel_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	if (device == NULL)
		return 0;
	return snprintf(buf, PAGE_SIZE, "%d\n",
		device->pwrctrl.default_pwrlevel);
}

static ssize_t kgsl_pwrctrl_default_pwrlevel_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	struct kgsl_pwrctrl *pwr;
	struct kgsl_pwrscale *pwrscale;
	int ret;
	unsigned int level = 0;

	if (device == NULL)
		return 0;

	pwr = &device->pwrctrl;
	pwrscale = &device->pwrscale;

	ret = kgsl_sysfs_store(buf, &level);
	if (ret)
		return ret;

	if (level > pwr->num_pwrlevels - 2)
		goto done;

	mutex_lock(&device->mutex);
	pwr->default_pwrlevel = level;
	pwrscale->gpu_profile.profile.initial_freq
			= pwr->pwrlevels[level].gpu_freq;

	mutex_unlock(&device->mutex);
done:
	return count;
}


static ssize_t kgsl_popp_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	unsigned int val = 0;
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	int ret;

	if (device == NULL)
		return 0;

	ret = kgsl_sysfs_store(buf, &val);
	if (ret)
		return ret;

	mutex_lock(&device->mutex);
	if (val)
		set_bit(POPP_ON, &device->pwrscale.popp_state);
	else
		clear_bit(POPP_ON, &device->pwrscale.popp_state);
	mutex_unlock(&device->mutex);

	return count;
}

static ssize_t kgsl_popp_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	if (device == NULL)
		return 0;
	return snprintf(buf, PAGE_SIZE, "%d\n",
		test_bit(POPP_ON, &device->pwrscale.popp_state));
}

static DEVICE_ATTR(gpuclk, 0644, kgsl_pwrctrl_gpuclk_show,
	kgsl_pwrctrl_gpuclk_store);
static DEVICE_ATTR(max_gpuclk, 0644, kgsl_pwrctrl_max_gpuclk_show,
	kgsl_pwrctrl_max_gpuclk_store);
static DEVICE_ATTR(idle_timer, 0644, kgsl_pwrctrl_idle_timer_show,
	kgsl_pwrctrl_idle_timer_store);
static DEVICE_ATTR(gpubusy, 0444, kgsl_pwrctrl_gpubusy_show,
	NULL);
static DEVICE_ATTR(gpu_available_frequencies, 0444,
	kgsl_pwrctrl_gpu_available_frequencies_show,
	NULL);
static DEVICE_ATTR(max_pwrlevel, 0644,
	kgsl_pwrctrl_max_pwrlevel_show,
	kgsl_pwrctrl_max_pwrlevel_store);
static DEVICE_ATTR(min_pwrlevel, 0644,
	kgsl_pwrctrl_min_pwrlevel_show,
	kgsl_pwrctrl_min_pwrlevel_store);
static DEVICE_ATTR(thermal_pwrlevel, 0644,
	kgsl_pwrctrl_thermal_pwrlevel_show,
	kgsl_pwrctrl_thermal_pwrlevel_store);
static DEVICE_ATTR(num_pwrlevels, 0444,
	kgsl_pwrctrl_num_pwrlevels_show,
	NULL);
static DEVICE_ATTR(pmqos_active_latency, 0644,
	kgsl_pwrctrl_pmqos_active_latency_show,
	kgsl_pwrctrl_pmqos_active_latency_store);
static DEVICE_ATTR(reset_count, 0444,
	kgsl_pwrctrl_reset_count_show,
	NULL);
static DEVICE_ATTR(force_clk_on, 0644,
	kgsl_pwrctrl_force_clk_on_show,
	kgsl_pwrctrl_force_clk_on_store);
static DEVICE_ATTR(force_bus_on, 0644,
	kgsl_pwrctrl_force_bus_on_show,
	kgsl_pwrctrl_force_bus_on_store);
static DEVICE_ATTR(force_rail_on, 0644,
	kgsl_pwrctrl_force_rail_on_show,
	kgsl_pwrctrl_force_rail_on_store);
static DEVICE_ATTR(bus_split, 0644,
	kgsl_pwrctrl_bus_split_show,
	kgsl_pwrctrl_bus_split_store);
static DEVICE_ATTR(default_pwrlevel, 0644,
	kgsl_pwrctrl_default_pwrlevel_show,
	kgsl_pwrctrl_default_pwrlevel_store);
static DEVICE_ATTR(popp, 0644, kgsl_popp_show, kgsl_popp_store);

static const struct device_attribute *pwrctrl_attr_list[] = {
	&dev_attr_gpuclk,
	&dev_attr_max_gpuclk,
	&dev_attr_idle_timer,
	&dev_attr_gpubusy,
	&dev_attr_gpu_available_frequencies,
	&dev_attr_max_pwrlevel,
	&dev_attr_min_pwrlevel,
	&dev_attr_thermal_pwrlevel,
	&dev_attr_num_pwrlevels,
	&dev_attr_pmqos_active_latency,
	&dev_attr_reset_count,
	&dev_attr_force_clk_on,
	&dev_attr_force_bus_on,
	&dev_attr_force_rail_on,
	&dev_attr_bus_split,
	&dev_attr_default_pwrlevel,
	&dev_attr_popp,
	NULL
};

int kgsl_pwrctrl_init_sysfs(struct kgsl_device *device)
{
	return kgsl_create_device_sysfs_files(device->dev, pwrctrl_attr_list);
}

void kgsl_pwrctrl_uninit_sysfs(struct kgsl_device *device)
{
	kgsl_remove_device_sysfs_files(device->dev, pwrctrl_attr_list);
}

void kgsl_pwrctrl_busy_time(struct kgsl_device *device, u64 time, u64 busy)
{
	struct kgsl_clk_stats *stats = &device->pwrctrl.clk_stats;
	stats->total += time;
	stats->busy += busy;

	if (stats->total < UPDATE_BUSY_VAL)
		return;

	
	stats->total_old = stats->total;
	stats->busy_old = stats->busy;
	stats->total = 0;
	stats->busy = 0;

	trace_kgsl_gpubusy(device, stats->busy_old, stats->total_old);
}
EXPORT_SYMBOL(kgsl_pwrctrl_busy_time);

void kgsl_pwrctrl_clk(struct kgsl_device *device, int state,
					  int requested_state)
{
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	int i = 0;

	if (test_bit(KGSL_PWRFLAGS_CLK_ON, &pwr->ctrl_flags))
		return;

	if (state == KGSL_PWRFLAGS_OFF) {
		if (test_and_clear_bit(KGSL_PWRFLAGS_CLK_ON,
			&pwr->power_flags)) {
			trace_kgsl_clk(device, state);
			for (i = KGSL_MAX_CLKS - 1; i > 0; i--)
				if (pwr->grp_clks[i])
					clk_disable(pwr->grp_clks[i]);
			
			if ((pwr->pwrlevels[0].gpu_freq > 0) &&
				(requested_state != KGSL_STATE_NAP)) {
				for (i = KGSL_MAX_CLKS - 1; i > 0; i--)
					if (pwr->grp_clks[i])
						clk_unprepare(pwr->grp_clks[i]);
				clk_set_rate(pwr->grp_clks[0],
					pwr->pwrlevels[pwr->num_pwrlevels - 1].
					gpu_freq);
			}
		} else if (requested_state == KGSL_STATE_SLEEP) {
			
			for (i = KGSL_MAX_CLKS - 1; i > 0; i--)
				if (pwr->grp_clks[i])
					clk_unprepare(pwr->grp_clks[i]);
			if ((pwr->pwrlevels[0].gpu_freq > 0))
				clk_set_rate(pwr->grp_clks[0],
					pwr->pwrlevels[pwr->num_pwrlevels - 1].
					gpu_freq);
		}
	} else if (state == KGSL_PWRFLAGS_ON) {
		if (!test_and_set_bit(KGSL_PWRFLAGS_CLK_ON,
			&pwr->power_flags)) {
			trace_kgsl_clk(device, state);
			
			if (device->state != KGSL_STATE_NAP) {
				if (pwr->pwrlevels[0].gpu_freq > 0)
					clk_set_rate(pwr->grp_clks[0],
						pwr->pwrlevels
						[pwr->active_pwrlevel].
						gpu_freq);
				for (i = KGSL_MAX_CLKS - 1; i > 0; i--)
					if (pwr->grp_clks[i])
						clk_prepare(pwr->grp_clks[i]);
			}
			for (i = KGSL_MAX_CLKS - 1; i > 0; i--)
				if (pwr->grp_clks[i])
					clk_enable(pwr->grp_clks[i]);
		}
	}
}

static void kgsl_pwrctrl_axi(struct kgsl_device *device, int state)
{
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;

	if (test_bit(KGSL_PWRFLAGS_AXI_ON, &pwr->ctrl_flags))
		return;

	if (state == KGSL_PWRFLAGS_OFF) {
		if (test_and_clear_bit(KGSL_PWRFLAGS_AXI_ON,
			&pwr->power_flags)) {
			trace_kgsl_bus(device, state);
			kgsl_pwrctrl_buslevel_update(device, false);

			if (pwr->devbw)
				devfreq_suspend_devbw(pwr->devbw);
		}
	} else if (state == KGSL_PWRFLAGS_ON) {
		if (!test_and_set_bit(KGSL_PWRFLAGS_AXI_ON,
			&pwr->power_flags)) {
			trace_kgsl_bus(device, state);
			kgsl_pwrctrl_buslevel_update(device, true);

			if (pwr->devbw)
				devfreq_resume_devbw(pwr->devbw);
		}
	}
}

static int kgsl_pwrctrl_pwrrail(struct kgsl_device *device, int state)
{
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	int status_gpu = 0;
	int status_cx = 0;

	if (test_bit(KGSL_PWRFLAGS_POWER_ON, &pwr->ctrl_flags))
		return 0;

	if (state == KGSL_PWRFLAGS_OFF) {
		if (test_and_clear_bit(KGSL_PWRFLAGS_POWER_ON,
			&pwr->power_flags)) {
			trace_kgsl_rail(device, state);
			if (pwr->gpu_cx)
				regulator_disable(pwr->gpu_cx);
			if (pwr->gpu_reg)
				regulator_disable(pwr->gpu_reg);
		}
	} else if (state == KGSL_PWRFLAGS_ON) {
		if (!test_and_set_bit(KGSL_PWRFLAGS_POWER_ON,
			&pwr->power_flags)) {
			if (pwr->gpu_reg) {
				status_gpu = regulator_enable(pwr->gpu_reg);
				if (status_gpu)
					KGSL_DRV_ERR(device,
							"core regulator_enable "
							"failed: %d\n",
							status_gpu);
			}
			if (pwr->gpu_cx) {
				status_cx = regulator_enable(pwr->gpu_cx);
				if (status_cx)
					KGSL_DRV_ERR(device,
							"cx regulator_enable "
							"failed: %d\n",
							status_cx);
			}
			if (status_gpu || status_cx) {
				if (pwr->gpu_reg && !status_gpu)
					regulator_disable(pwr->gpu_reg);
				if (pwr->gpu_cx && !status_cx)
					regulator_disable(pwr->gpu_cx);
				clear_bit(KGSL_PWRFLAGS_POWER_ON,
					&pwr->power_flags);
			} else
				trace_kgsl_rail(device, state);
		}
	}

	return status_gpu || status_cx;
}

static void kgsl_pwrctrl_irq(struct kgsl_device *device, int state)
{
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;

	if (state == KGSL_PWRFLAGS_ON) {
		if (!test_and_set_bit(KGSL_PWRFLAGS_IRQ_ON,
			&pwr->power_flags)) {
			trace_kgsl_irq(device, state);
			enable_irq(pwr->interrupt_num);
		}
	} else if (state == KGSL_PWRFLAGS_OFF) {
		if (test_and_clear_bit(KGSL_PWRFLAGS_IRQ_ON,
			&pwr->power_flags)) {
			trace_kgsl_irq(device, state);
			if (in_interrupt())
				disable_irq_nosync(pwr->interrupt_num);
			else
				disable_irq(pwr->interrupt_num);
		}
	}
}

static void kgsl_thermal_cycle(struct work_struct *work)
{
	struct kgsl_pwrctrl *pwr = container_of(work, struct kgsl_pwrctrl,
						thermal_cycle_ws);
	struct kgsl_device *device = container_of(pwr, struct kgsl_device,
							pwrctrl);

	if (device == NULL)
		return;

	mutex_lock(&device->mutex);
	if (pwr->thermal_cycle == CYCLE_ACTIVE) {
		if (pwr->thermal_highlow)
			kgsl_pwrctrl_pwrlevel_change(device,
					pwr->thermal_pwrlevel);
		else
			kgsl_pwrctrl_pwrlevel_change(device,
					pwr->thermal_pwrlevel + 1);
	}
	mutex_unlock(&device->mutex);
}

void kgsl_thermal_timer(unsigned long data)
{
	struct kgsl_device *device = (struct kgsl_device *) data;

	
	if (device->pwrctrl.thermal_highlow) {
		mod_timer(&device->pwrctrl.thermal_timer,
					jiffies +
					device->pwrctrl.thermal_timeout);
		device->pwrctrl.thermal_highlow = 0;
	} else {
		mod_timer(&device->pwrctrl.thermal_timer,
					jiffies + (TH_HZ -
					device->pwrctrl.thermal_timeout));
		device->pwrctrl.thermal_highlow = 1;
	}
	
	queue_work(device->work_queue, &device->pwrctrl.thermal_cycle_ws);
}

int kgsl_pwrctrl_init(struct kgsl_device *device)
{
	int i, k, m, set_bus = 1, n = 0, result = 0;
	unsigned int freq_i, rbbmtimer_freq;
	struct clk *clk;
	struct platform_device *pdev = device->pdev;
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	struct kgsl_device_platform_data *pdata = dev_get_platdata(&pdev->dev);
	struct device_node *ocmem_bus_node;
	struct msm_bus_scale_pdata *ocmem_scale_table = NULL;
	struct device_node *gpubw_dev_node;
	struct platform_device *p2dev;

	
	for (i = 0; i < KGSL_MAX_CLKS; i++) {
		if (pdata->clk_map & clks[i].map) {
			clk = clk_get(&pdev->dev, clks[i].name);
			if (IS_ERR(clk))
				goto clk_err;
			pwr->grp_clks[i] = clk;
		}
	}
	
	if (pwr->grp_clks[0] == NULL)
		pwr->grp_clks[0] = pwr->grp_clks[1];

	if (pdata->num_levels > KGSL_MAX_PWRLEVELS ||
	    pdata->num_levels < 1) {
		KGSL_PWR_ERR(device, "invalid power level count: %d\n",
					 pdata->num_levels);
		result = -EINVAL;
		goto done;
	}
	pwr->num_pwrlevels = pdata->num_levels;

	

	pwr->max_pwrlevel = 0;
	pwr->min_pwrlevel = pdata->num_levels - 2;
	pwr->thermal_pwrlevel = 0;

	pwr->active_pwrlevel = pdata->init_level;
	pwr->default_pwrlevel = pdata->init_level;
	pwr->init_pwrlevel = pdata->init_level;
	pwr->wakeup_maxpwrlevel = 0;
	for (i = 0; i < pdata->num_levels; i++) {
		pwr->pwrlevels[i].gpu_freq =
		(pdata->pwrlevel[i].gpu_freq > 0) ?
		clk_round_rate(pwr->grp_clks[0],
					   pdata->pwrlevel[i].
					   gpu_freq) : 0;
		pwr->pwrlevels[i].bus_freq =
			pdata->pwrlevel[i].bus_freq;
		pwr->pwrlevels[i].bus_min =
			pdata->pwrlevel[i].bus_min;
		pwr->pwrlevels[i].bus_max =
			pdata->pwrlevel[i].bus_max;
		if (pwr->pwrlevels[i].bus_min != pwr->pwrlevels[i].bus_max)
			set_bus = 0;
	}
	
	if (pwr->pwrlevels[0].gpu_freq > 0) {
		clk_set_rate(pwr->grp_clks[0], pwr->
				pwrlevels[pwr->num_pwrlevels - 1].gpu_freq);
		rbbmtimer_freq = clk_round_rate(pwr->grp_clks[6],
						KGSL_RBBMTIMER_CLK_FREQ);
		clk_set_rate(pwr->grp_clks[6], rbbmtimer_freq);
	}

	pwr->gpu_reg = regulator_get(&pdev->dev, "vdd");
	if (IS_ERR(pwr->gpu_reg))
		pwr->gpu_reg = NULL;

	if (pwr->gpu_reg) {
		pwr->gpu_cx = regulator_get(&pdev->dev, "vddcx");
		if (IS_ERR(pwr->gpu_cx))
			pwr->gpu_cx = NULL;
	} else
		pwr->gpu_cx = NULL;

	pwr->power_flags = 0;

	pwr->interval_timeout = pdata->idle_timeout;
	pwr->strtstp_sleepwake = pdata->strtstp_sleepwake;

	if (kgsl_property_read_u32(device, "qcom,pm-qos-active-latency",
		&pwr->pm_qos_active_latency))
		pwr->pm_qos_active_latency = 501;

	if (kgsl_property_read_u32(device, "qcom,pm-qos-wakeup-latency",
		&pwr->pm_qos_wakeup_latency))
		pwr->pm_qos_wakeup_latency = 101;

	pm_runtime_enable(&pdev->dev);

	ocmem_bus_node = of_find_node_by_name(
				device->pdev->dev.of_node,
				"qcom,ocmem-bus-client");
	
	if (ocmem_bus_node) {
		ocmem_scale_table = msm_bus_pdata_from_node
				(device->pdev, ocmem_bus_node);
		if (ocmem_scale_table)
			pwr->ocmem_pcl = msm_bus_scale_register_client
					(ocmem_scale_table);

		if (!pwr->ocmem_pcl) {
			KGSL_PWR_ERR(device,
				"msm_bus_scale_register_client failed: id %d table %p",
				device->id, ocmem_scale_table);
			result = -EINVAL;
			goto done;
		}
	}

	
	pwr->bus_control = pdata->bus_control;

	
	gpubw_dev_node = of_parse_phandle(pdev->dev.of_node,
					"qcom,gpubw-dev", 0);
	if (gpubw_dev_node) {
		p2dev = of_find_device_by_node(gpubw_dev_node);
		if (p2dev)
			pwr->devbw = &p2dev->dev;
	} else {
		pwr->pcl = msm_bus_scale_register_client
				(pdata->bus_scale_table);
		if (!pwr->pcl) {
			KGSL_PWR_ERR(device,
				"msm_bus_scale_register_client failed: id %d table %p",
				device->id, pdata->bus_scale_table);
			result = -EINVAL;
			goto done;
		}
	}

	freq_i = pwr->min_pwrlevel;
	if (set_bus == 1)
		pwr->pwrlevels[freq_i].bus_min = 1;

	for (i = 0; i < pdata->bus_scale_table->num_usecases; i++) {
		struct msm_bus_paths *usecase =
				&pdata->bus_scale_table->usecase[i];
		struct msm_bus_vectors *vector = &usecase->vectors[0];
		if (vector->dst == MSM_BUS_SLAVE_EBI_CH0 &&
				vector->ib != 0) {

			if (i < KGSL_MAX_BUSLEVELS) {
				
				ib_votes[i] =
					DIV_ROUND_UP_ULL(vector->ib, 1048576)
					- 1;
				if (ib_votes[i] > ib_votes[max_vote_buslevel])
					max_vote_buslevel = i;
			}

			for (k = 0; k < n; k++)
				if (vector->ib == pwr->bus_ib[k]) {
					static uint64_t last_ib = 0xFFFFFFFF;
					if (set_bus == 0)
						break;
					if (vector->ib <= last_ib) {
						pwr->pwrlevels[freq_i--].
							bus_max = i - 1;
						pwr->pwrlevels[freq_i].
							bus_min = i;
					}
					last_ib = vector->ib;
					break;
				}
			
			if (k == n) {
				pwr->bus_ib[k] = vector->ib;
				n++;
				
				for (m = 0; m < pwr->num_pwrlevels - 1; m++) {
					if (pdata->bus_scale_table->
						usecase[pwr->pwrlevels[m].
						bus_freq].vectors[0].ib
						== vector->ib)
						pwr->bus_index[m] = k;
				}
			}
		}
	}
	if (set_bus == 1)
		pwr->pwrlevels[0].bus_max = i - 1;

	INIT_WORK(&pwr->thermal_cycle_ws, kgsl_thermal_cycle);
	setup_timer(&pwr->thermal_timer, kgsl_thermal_timer,
			(unsigned long) device);

	INIT_LIST_HEAD(&pwr->limits);
	spin_lock_init(&pwr->limits_lock);
	pwr->sysfs_pwr_limit = kgsl_pwr_limits_add(KGSL_DEVICE_3D0);

	devfreq_vbif_register_callback(kgsl_get_bw);

	return result;

clk_err:
	result = PTR_ERR(clk);
	KGSL_PWR_ERR(device, "clk_get(%s) failed: %d\n",
				 clks[i].name, result);

done:
	return result;
}

void kgsl_pwrctrl_close(struct kgsl_device *device)
{
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	int i;

	KGSL_PWR_INFO(device, "close device %d\n", device->id);

	pm_runtime_disable(&device->pdev->dev);

	if (pwr->pcl)
		msm_bus_scale_unregister_client(pwr->pcl);

	pwr->pcl = 0;

	if (pwr->ocmem_pcl)
		msm_bus_scale_unregister_client(pwr->ocmem_pcl);

	pwr->ocmem_pcl = 0;

	if (pwr->gpu_reg) {
		regulator_put(pwr->gpu_reg);
		pwr->gpu_reg = NULL;
	}

	if (pwr->gpu_cx) {
		regulator_put(pwr->gpu_cx);
		pwr->gpu_cx = NULL;
	}

	for (i = 1; i < KGSL_MAX_CLKS; i++)
		if (pwr->grp_clks[i]) {
			clk_put(pwr->grp_clks[i]);
			pwr->grp_clks[i] = NULL;
		}

	pwr->grp_clks[0] = NULL;
	pwr->power_flags = 0;

	if (!IS_ERR_OR_NULL(pwr->sysfs_pwr_limit)) {
		list_del(&pwr->sysfs_pwr_limit->node);
		kfree(pwr->sysfs_pwr_limit);
		pwr->sysfs_pwr_limit = NULL;
	}
}

void kgsl_idle_check(struct work_struct *work)
{
	struct kgsl_device *device = container_of(work, struct kgsl_device,
							idle_check_ws);
	WARN_ON(device == NULL);
	if (device == NULL)
		return;

	mutex_lock(&device->mutex);

	if (device->state == KGSL_STATE_ACTIVE
		   || device->state ==  KGSL_STATE_NAP) {

		if (!atomic_read(&device->active_cnt))
			kgsl_pwrctrl_change_state(device,
					device->requested_state);

		kgsl_pwrctrl_request_state(device, KGSL_STATE_NONE);
		if (device->state == KGSL_STATE_ACTIVE)
			mod_timer(&device->idle_timer,
					jiffies +
					device->pwrctrl.interval_timeout);
	}

	kgsl_pwrscale_update(device);
	mutex_unlock(&device->mutex);
}
EXPORT_SYMBOL(kgsl_idle_check);

void kgsl_timer(unsigned long data)
{
	struct kgsl_device *device = (struct kgsl_device *) data;

	KGSL_PWR_INFO(device, "idle timer expired device %d\n", device->id);
	if (device->requested_state != KGSL_STATE_SUSPEND) {
		if (device->pwrctrl.strtstp_sleepwake)
			kgsl_pwrctrl_request_state(device, KGSL_STATE_SLUMBER);
		else
			kgsl_pwrctrl_request_state(device, KGSL_STATE_SLEEP);
		
		queue_work(device->work_queue, &device->idle_check_ws);
	}
}

static bool kgsl_pwrctrl_isenabled(struct kgsl_device *device)
{
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	return ((test_bit(KGSL_PWRFLAGS_CLK_ON, &pwr->power_flags) != 0) &&
		(test_bit(KGSL_PWRFLAGS_AXI_ON, &pwr->power_flags) != 0));
}

/**
 * kgsl_pre_hwaccess - Enforce preconditions for touching registers
 * @device: The device
 *
 * This function ensures that the correct lock is held and that the GPU
 * clock is on immediately before a register is read or written. Note
 * that this function does not check active_cnt because the registers
 * must be accessed during device start and stop, when the active_cnt
 * may legitimately be 0.
 */
void kgsl_pre_hwaccess(struct kgsl_device *device)
{
	
	BUG_ON(!mutex_is_locked(&device->mutex));
	
	BUG_ON(!kgsl_pwrctrl_isenabled(device));
}
EXPORT_SYMBOL(kgsl_pre_hwaccess);

static int kgsl_pwrctrl_enable(struct kgsl_device *device)
{
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	int level, status;

	if (pwr->wakeup_maxpwrlevel) {
		level = pwr->max_pwrlevel;
		pwr->wakeup_maxpwrlevel = 0;
	} else if (kgsl_popp_check(device)) {
		level = pwr->active_pwrlevel;
	} else {
		level = pwr->default_pwrlevel;
	}

	kgsl_pwrctrl_pwrlevel_change(device, level);

	
	status = kgsl_pwrctrl_pwrrail(device, KGSL_PWRFLAGS_ON);
	if (status)
		return status;
	kgsl_pwrctrl_clk(device, KGSL_PWRFLAGS_ON, KGSL_STATE_ACTIVE);
	kgsl_pwrctrl_axi(device, KGSL_PWRFLAGS_ON);
	device->ftbl->regulator_enable(device);
	return status;
}

static void kgsl_pwrctrl_disable(struct kgsl_device *device)
{
	
	device->ftbl->regulator_disable(device);
	kgsl_pwrctrl_axi(device, KGSL_PWRFLAGS_OFF);
	kgsl_pwrctrl_clk(device, KGSL_PWRFLAGS_OFF, KGSL_STATE_SLEEP);
	kgsl_pwrctrl_pwrrail(device, KGSL_PWRFLAGS_OFF);
}

static int _init(struct kgsl_device *device)
{
	int status = 0;
	switch (device->state) {
	case KGSL_STATE_NAP:
	case KGSL_STATE_SLEEP:
		
		status = kgsl_pwrctrl_enable(device);
	case KGSL_STATE_ACTIVE:
		kgsl_pwrctrl_irq(device, KGSL_PWRFLAGS_OFF);
		del_timer_sync(&device->idle_timer);
		device->ftbl->stop(device);
		
	case KGSL_STATE_AWARE:
		kgsl_pwrctrl_disable(device);
		
	case KGSL_STATE_SLUMBER:
	case KGSL_STATE_NONE:
		kgsl_pwrctrl_set_state(device, KGSL_STATE_INIT);
	}

	return status;
}

static int _wake(struct kgsl_device *device)
{
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	int status = 0;

	switch (device->state) {
	case KGSL_STATE_SUSPEND:
		complete_all(&device->hwaccess_gate);
		
		device->ftbl->resume(device);
		
	case KGSL_STATE_SLUMBER:
		status = device->ftbl->start(device,
				device->pwrctrl.superfast);
		device->pwrctrl.superfast = false;

		if (status) {
			kgsl_pwrctrl_request_state(device, KGSL_STATE_NONE);
			KGSL_DRV_ERR(device, "start failed %d\n", status);
			
			dump_stack();
			break;
		}
		
	case KGSL_STATE_SLEEP:
		kgsl_pwrctrl_axi(device, KGSL_PWRFLAGS_ON);
		kgsl_pwrscale_wake(device);
		kgsl_pwrctrl_irq(device, KGSL_PWRFLAGS_ON);
		
	case KGSL_STATE_NAP:
		
		kgsl_pwrctrl_clk(device, KGSL_PWRFLAGS_ON, KGSL_STATE_ACTIVE);

		kgsl_pwrctrl_set_state(device, KGSL_STATE_ACTIVE);

		
		kgsl_pwrctrl_pwrlevel_change_settings(device, 1, 1);
		
		pwr->previous_pwrlevel = pwr->active_pwrlevel;
		mod_timer(&device->idle_timer, jiffies +
				device->pwrctrl.interval_timeout);
		break;
	case KGSL_STATE_AWARE:
		
		kgsl_pwrctrl_set_state(device, KGSL_STATE_ACTIVE);
		kgsl_pwrctrl_irq(device, KGSL_PWRFLAGS_ON);
		mod_timer(&device->idle_timer, jiffies +
				device->pwrctrl.interval_timeout);
		break;
	default:
		KGSL_PWR_WARN(device, "unhandled state %s\n",
				kgsl_pwrstate_to_str(device->state));
		kgsl_pwrctrl_request_state(device, KGSL_STATE_NONE);
		status = -EINVAL;
		break;
	}
	return status;
}

static int
_aware(struct kgsl_device *device)
{
	int status = 0;
	switch (device->state) {
	case KGSL_STATE_INIT:
		status = kgsl_pwrctrl_enable(device);
		break;
	
	case KGSL_STATE_NAP:
	case KGSL_STATE_SLEEP:
		status = _wake(device);
	case KGSL_STATE_ACTIVE:
		kgsl_pwrctrl_irq(device, KGSL_PWRFLAGS_OFF);
		del_timer_sync(&device->idle_timer);
		break;
	case KGSL_STATE_SLUMBER:
		set_bit(KGSL_PWRFLAGS_WAKEUP_PWRLEVEL,
				&device->pwrctrl.ctrl_flags);
		status = kgsl_pwrctrl_enable(device);
		clear_bit(KGSL_PWRFLAGS_WAKEUP_PWRLEVEL,
				&device->pwrctrl.ctrl_flags);
		break;
	default:
		status = -EINVAL;
	}
	if (status)
		kgsl_pwrctrl_request_state(device, KGSL_STATE_NONE);
	else
		kgsl_pwrctrl_set_state(device, KGSL_STATE_AWARE);
	return status;
}

static int
_nap(struct kgsl_device *device)
{
	switch (device->state) {
	case KGSL_STATE_ACTIVE:
		if (!device->ftbl->isidle(device)) {
			kgsl_pwrctrl_request_state(device, KGSL_STATE_NONE);
			return -EBUSY;
		}

		kgsl_pwrscale_update_stats(device);

		kgsl_pwrctrl_clk(device, KGSL_PWRFLAGS_OFF, KGSL_STATE_NAP);
		kgsl_pwrctrl_set_state(device, KGSL_STATE_NAP);
	case KGSL_STATE_SLEEP:
	case KGSL_STATE_SLUMBER:
		break;
	case KGSL_STATE_AWARE:
		KGSL_PWR_WARN(device,
			"transition AWARE -> NAP is not permitted\n");
	default:
		kgsl_pwrctrl_request_state(device, KGSL_STATE_NONE);
		break;
	}
	return 0;
}

static int
_sleep(struct kgsl_device *device)
{
	switch (device->state) {
	case KGSL_STATE_ACTIVE:
		if (!device->ftbl->isidle(device)) {
			kgsl_pwrctrl_request_state(device, KGSL_STATE_NONE);
			return -EBUSY;
		}
		
	case KGSL_STATE_NAP:
		kgsl_pwrctrl_irq(device, KGSL_PWRFLAGS_OFF);
		kgsl_pwrctrl_axi(device, KGSL_PWRFLAGS_OFF);
		kgsl_pwrscale_sleep(device);
		kgsl_pwrctrl_clk(device, KGSL_PWRFLAGS_OFF, KGSL_STATE_SLEEP);
		kgsl_pwrctrl_set_state(device, KGSL_STATE_SLEEP);
		pm_qos_update_request(&device->pwrctrl.pm_qos_req_dma,
					PM_QOS_DEFAULT_VALUE);
		break;
	case KGSL_STATE_SLUMBER:
		break;
	case KGSL_STATE_AWARE:
		KGSL_PWR_WARN(device,
			"transition AWARE -> SLEEP is not permitted\n");
	default:
		kgsl_pwrctrl_request_state(device, KGSL_STATE_NONE);
		break;
	}

	return 0;
}

static int
_slumber(struct kgsl_device *device)
{
	int status = 0;
	switch (device->state) {
	case KGSL_STATE_ACTIVE:
		if (!device->ftbl->isidle(device)) {
			kgsl_pwrctrl_request_state(device, KGSL_STATE_NONE);
			return -EBUSY;
		}
		
	case KGSL_STATE_NAP:
	case KGSL_STATE_SLEEP:
		del_timer_sync(&device->idle_timer);
		if (device->pwrctrl.thermal_cycle == CYCLE_ACTIVE) {
			device->pwrctrl.thermal_cycle = CYCLE_ENABLE;
			del_timer_sync(&device->pwrctrl.thermal_timer);
		}
		kgsl_pwrctrl_irq(device, KGSL_PWRFLAGS_OFF);
		
		status = kgsl_pwrctrl_enable(device);
		device->ftbl->suspend_context(device);
		device->ftbl->stop(device);
		kgsl_pwrctrl_disable(device);
		kgsl_pwrscale_sleep(device);
		kgsl_pwrctrl_irq(device, KGSL_PWRFLAGS_OFF);
		kgsl_pwrctrl_set_state(device, KGSL_STATE_SLUMBER);
		pm_qos_update_request(&device->pwrctrl.pm_qos_req_dma,
						PM_QOS_DEFAULT_VALUE);
		break;
	case KGSL_STATE_SUSPEND:
		complete_all(&device->hwaccess_gate);
		device->ftbl->resume(device);
		kgsl_pwrctrl_set_state(device, KGSL_STATE_SLUMBER);
		break;
	case KGSL_STATE_AWARE:
		kgsl_pwrctrl_disable(device);
		kgsl_pwrctrl_set_state(device, KGSL_STATE_SLUMBER);
		break;
	default:
		kgsl_pwrctrl_request_state(device, KGSL_STATE_NONE);
		break;

	}
	return status;
}

int _suspend(struct kgsl_device *device)
{
	int ret = 0;

	if ((KGSL_STATE_NONE == device->state) ||
			(KGSL_STATE_INIT == device->state))
		return ret;

	
	device->ftbl->drain(device);
	
	ret = kgsl_active_count_wait(device, 0);
	if (ret) {
		KGSL_DRV_ERR(device, "Failed to get kgsl active count.\n");
		goto err;
	}

	ret = device->ftbl->idle(device);
	if (ret) {
		KGSL_DRV_ERR(device, "Failed to set device idle.\n");
		goto err;
	}

	ret = _slumber(device);
	if (ret) {
		KGSL_DRV_ERR(device, "Failed to put device in slumber mode.\n");
		goto err;
	}

	kgsl_pwrctrl_set_state(device, KGSL_STATE_SUSPEND);
	return ret;

err:
	device->ftbl->resume(device);
	KGSL_PWR_ERR(device, "device failed to SUSPEND %d\n", ret);
	return ret;
}

int kgsl_pwrctrl_change_state(struct kgsl_device *device, int state)
{
	int status = 0;
	if (device->state == state)
		return status;
	kgsl_pwrctrl_request_state(device, state);

	
	switch (state) {
	case KGSL_STATE_INIT:
		status = _init(device);
		break;
	case KGSL_STATE_AWARE:
		status = _aware(device);
		break;
	case KGSL_STATE_ACTIVE:
		status = _wake(device);
		break;
	case KGSL_STATE_NAP:
		status = _nap(device);
		break;
	case KGSL_STATE_SLEEP:
		status = _sleep(device);
		break;
	case KGSL_STATE_SLUMBER:
		status = _slumber(device);
		break;
	case KGSL_STATE_SUSPEND:
		status = _suspend(device);
		break;
	default:
		KGSL_PWR_INFO(device, "bad state request 0x%x\n", state);
		kgsl_pwrctrl_request_state(device, KGSL_STATE_NONE);
		status = -EINVAL;
		break;
	}

	
	if (!status) {
		ktime_t t = ktime_get();
		_record_pwrevent(device, t, KGSL_PWREVENT_STATE);
	}
	return status;
}
EXPORT_SYMBOL(kgsl_pwrctrl_change_state);

static void kgsl_pwrctrl_set_state(struct kgsl_device *device,
				unsigned int state)
{
	trace_kgsl_pwr_set_state(device, state);
	device->state = state;
	device->requested_state = KGSL_STATE_NONE;
}

static void kgsl_pwrctrl_request_state(struct kgsl_device *device,
				unsigned int state)
{
	if (state != KGSL_STATE_NONE && state != device->requested_state)
		trace_kgsl_pwr_request_state(device, state);
	device->requested_state = state;
}

const char *kgsl_pwrstate_to_str(unsigned int state)
{
	switch (state) {
	case KGSL_STATE_NONE:
		return "NONE";
	case KGSL_STATE_INIT:
		return "INIT";
	case KGSL_STATE_AWARE:
		return "AWARE";
	case KGSL_STATE_ACTIVE:
		return "ACTIVE";
	case KGSL_STATE_NAP:
		return "NAP";
	case KGSL_STATE_SLEEP:
		return "SLEEP";
	case KGSL_STATE_SUSPEND:
		return "SUSPEND";
	case KGSL_STATE_SLUMBER:
		return "SLUMBER";
	default:
		break;
	}
	return "UNKNOWN";
}
EXPORT_SYMBOL(kgsl_pwrstate_to_str);


int kgsl_active_count_get(struct kgsl_device *device)
{
	int ret = 0;
	BUG_ON(!mutex_is_locked(&device->mutex));

	if ((atomic_read(&device->active_cnt) == 0) &&
		(device->state != KGSL_STATE_ACTIVE)) {
		mutex_unlock(&device->mutex);
		wait_for_completion(&device->hwaccess_gate);
		mutex_lock(&device->mutex);
		device->pwrctrl.superfast = true;
		ret = kgsl_pwrctrl_change_state(device, KGSL_STATE_ACTIVE);
	}
	if (ret == 0)
		atomic_inc(&device->active_cnt);
	trace_kgsl_active_count(device,
		(unsigned long) __builtin_return_address(0));
	return ret;
}
EXPORT_SYMBOL(kgsl_active_count_get);

void kgsl_active_count_put(struct kgsl_device *device)
{
	BUG_ON(!mutex_is_locked(&device->mutex));
	BUG_ON(atomic_read(&device->active_cnt) == 0);

	if (atomic_dec_and_test(&device->active_cnt)) {
		if (device->state == KGSL_STATE_ACTIVE &&
			device->requested_state == KGSL_STATE_NONE) {
			kgsl_pwrctrl_request_state(device, KGSL_STATE_NAP);
			queue_work(device->work_queue, &device->idle_check_ws);
		}

		mod_timer(&device->idle_timer,
			jiffies + device->pwrctrl.interval_timeout);
	}

	trace_kgsl_active_count(device,
		(unsigned long) __builtin_return_address(0));

	wake_up(&device->active_cnt_wq);
}
EXPORT_SYMBOL(kgsl_active_count_put);

static int _check_active_count(struct kgsl_device *device, int count)
{
	
	return atomic_read(&device->active_cnt) > count ? 0 : 1;
}

int kgsl_active_count_wait(struct kgsl_device *device, int count)
{
	int result = 0;
	long wait_jiffies = HZ;

	BUG_ON(!mutex_is_locked(&device->mutex));

	while (atomic_read(&device->active_cnt) > count) {
		long ret;
		mutex_unlock(&device->mutex);
		ret = wait_event_timeout(device->active_cnt_wq,
			_check_active_count(device, count), wait_jiffies);
		mutex_lock(&device->mutex);
		result = ret == 0 ? -ETIMEDOUT : 0;
		if (!result)
			wait_jiffies = ret;
		else
			break;
	}

	return result;
}
EXPORT_SYMBOL(kgsl_active_count_wait);

static void _update_limits(struct kgsl_pwr_limit *limit, unsigned int reason,
							unsigned int level)
{
	struct kgsl_device *device = limit->device;
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	struct kgsl_pwr_limit *temp_limit;
	unsigned int max_level = 0;

	spin_lock(&pwr->limits_lock);
	switch (reason) {
	case KGSL_PWR_ADD_LIMIT:
			list_add(&limit->node, &pwr->limits);
			break;
	case KGSL_PWR_DEL_LIMIT:
			list_del(&limit->node);
			if (list_empty(&pwr->limits))
				goto done;
			break;
	case KGSL_PWR_SET_LIMIT:
			limit->level = level;
			break;
	}

	list_for_each_entry(temp_limit, &pwr->limits, node) {
		max_level = max_t(unsigned int, max_level, temp_limit->level);
	}

done:
	spin_unlock(&pwr->limits_lock);

	mutex_lock(&device->mutex);
	pwr->thermal_pwrlevel = max_level;
	kgsl_pwrctrl_pwrlevel_change(device, pwr->active_pwrlevel);
	mutex_unlock(&device->mutex);
}

void *kgsl_pwr_limits_add(enum kgsl_deviceid id)
{
	struct kgsl_device *device = kgsl_get_device(id);
	struct kgsl_pwr_limit *limit;

	if (IS_ERR_OR_NULL(device))
		return NULL;

	limit = kzalloc(sizeof(struct kgsl_pwr_limit),
						GFP_KERNEL);
	if (limit == NULL)
		return ERR_PTR(-ENOMEM);
	limit->device = device;

	_update_limits(limit, KGSL_PWR_ADD_LIMIT, 0);
	return limit;
}
EXPORT_SYMBOL(kgsl_pwr_limits_add);

void kgsl_pwr_limits_del(void *limit_ptr)
{
	struct kgsl_pwr_limit *limit = limit_ptr;
	if (IS_ERR(limit))
		return;

	_update_limits(limit, KGSL_PWR_DEL_LIMIT, 0);
	kfree(limit);
}
EXPORT_SYMBOL(kgsl_pwr_limits_del);

int kgsl_pwr_limits_set_freq(void *limit_ptr, unsigned int freq)
{
	struct kgsl_pwrctrl *pwr;
	struct kgsl_pwr_limit *limit = limit_ptr;
	int level;

	if (IS_ERR(limit))
		return -EINVAL;

	pwr = &limit->device->pwrctrl;
	level = _get_nearest_pwrlevel(pwr, freq);
	if (level < 0)
		return -EINVAL;
	_update_limits(limit, KGSL_PWR_SET_LIMIT, level);
	return 0;
}
EXPORT_SYMBOL(kgsl_pwr_limits_set_freq);

void kgsl_pwr_limits_set_default(void *limit_ptr)
{
	struct kgsl_pwr_limit *limit = limit_ptr;

	if (IS_ERR(limit))
		return;

	_update_limits(limit, KGSL_PWR_SET_LIMIT, 0);
}
EXPORT_SYMBOL(kgsl_pwr_limits_set_default);

unsigned int kgsl_pwr_limits_get_freq(enum kgsl_deviceid id)
{
	struct kgsl_device *device = kgsl_get_device(id);
	struct kgsl_pwrctrl *pwr;
	unsigned int freq;

	if (IS_ERR_OR_NULL(device))
		return 0;
	pwr = &device->pwrctrl;
	mutex_lock(&device->mutex);
	freq = pwr->pwrlevels[pwr->thermal_pwrlevel].gpu_freq;
	mutex_unlock(&device->mutex);

	return freq;
}
EXPORT_SYMBOL(kgsl_pwr_limits_get_freq);
