/* Copyright (c) 2013-2015, The Linux Foundation. All rights reserved.
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

#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/jiffies.h>
#include <linux/err.h>

#include "kgsl.h"
#include "kgsl_cffdump.h"
#include "kgsl_sharedmem.h"
#include "adreno.h"
#include "adreno_ringbuffer.h"
#include "adreno_trace.h"
#include "kgsl_sharedmem.h"
#include "kgsl_htc.h"

#define CMDQUEUE_NEXT(_i, _s) (((_i) + 1) % (_s))

static unsigned int _context_cmdqueue_size = 50;

static unsigned int _context_queue_wait = 10000;

static unsigned int _context_cmdbatch_burst = 5;

static unsigned int _fault_throttle_time = 3000;
static unsigned int _fault_throttle_burst = 3;

static unsigned int _dispatcher_q_inflight_hi = 15;

static unsigned int _dispatcher_q_inflight_lo = 4;

static unsigned int _cmdbatch_timeout = 2000;

static unsigned int _fault_timer_interval = 200;

static int adreno_dispatch_process_cmdqueue(struct adreno_device *adreno_dev,
				struct adreno_dispatcher_cmdqueue *dispatch_q,
				int long_ib_detect);

static void _track_context(struct adreno_dispatcher_cmdqueue *cmdqueue,
		unsigned int id)
{
	struct adreno_context_list *list = cmdqueue->active_contexts;
	int oldest = -1, empty = -1;
	unsigned long age = 0;
	int i, count = 0;
	bool updated = false;

	for (i = 0; i < ACTIVE_CONTEXT_LIST_MAX; i++) {

		
		if (list[i].id == id) {
			list[i].jiffies = jiffies + msecs_to_jiffies(500);
			updated = true;
			count++;
			continue;
		}

		
		if ((list[i].id == 0) ||
			time_after(jiffies, list[i].jiffies)) {
			empty = i;
			continue;
		}

		count++;

		
		if (oldest == -1 || time_before(list[i].jiffies, age)) {
			age = list[i].jiffies;
			oldest = i;
		}
	}

	if (updated == false) {
		int pos = (empty != -1) ? empty : oldest;

		list[pos].jiffies = jiffies + msecs_to_jiffies(500);
		list[pos].id = id;
		count++;
	}

	cmdqueue->active_context_count = count;
}


static inline int
_cmdqueue_inflight(struct adreno_dispatcher_cmdqueue *cmdqueue)
{
	return (cmdqueue->active_context_count > 1)
		? _dispatcher_q_inflight_lo : _dispatcher_q_inflight_hi;
}

static void fault_detect_read(struct kgsl_device *device)
{
	int i;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);

	for (i = 0; i < adreno_dev->num_ringbuffers; i++) {
		struct adreno_ringbuffer *rb = &(adreno_dev->ringbuffers[i]);
		adreno_rb_readtimestamp(device, rb,
			KGSL_TIMESTAMP_RETIRED, &(rb->fault_detect_ts));
	}

	for (i = 0; i < adreno_ft_regs_num; i++) {
		if (adreno_ft_regs[i] == 0)
			continue;
		kgsl_regread(device, adreno_ft_regs[i], &adreno_ft_regs_val[i]);
	}
}

static inline bool _isidle(struct kgsl_device *device)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	unsigned int i;
	unsigned int reg_rbbm_status;

	if (!kgsl_state_is_awake(device))
		goto ret;

	adreno_readreg(adreno_dev, ADRENO_REG_RBBM_STATUS,
			&reg_rbbm_status);

	if (reg_rbbm_status & ADRENO_RBBM_STATUS_BUSY_MASK)
		return false;
ret:
	for (i = 0; i < adreno_ft_regs_num; i++)
		adreno_ft_regs_val[i] = 0;
	return true;
}

static int fault_detect_read_compare(struct kgsl_device *device)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct adreno_ringbuffer *rb = ADRENO_CURRENT_RINGBUFFER(adreno_dev);
	int i, ret = 0;
	unsigned int ts;

	
	if (_isidle(device) == true)
		ret = 1;

	for (i = 0; i < adreno_ft_regs_num; i++) {
		unsigned int val;

		if (adreno_ft_regs[i] == 0)
			continue;
		kgsl_regread(device, adreno_ft_regs[i], &val);
		if (val != adreno_ft_regs_val[i])
			ret = 1;
		adreno_ft_regs_val[i] = val;
	}

	if (!adreno_rb_readtimestamp(device, adreno_dev->cur_rb,
				KGSL_TIMESTAMP_RETIRED, &ts)) {
		if (ts != rb->fault_detect_ts)
			ret = 1;

		rb->fault_detect_ts = ts;
	}

	return ret;
}

static void _retire_marker(struct kgsl_cmdbatch *cmdbatch)
{
	struct kgsl_context *context = cmdbatch->context;
	struct kgsl_device *device = context->device;

	kgsl_sharedmem_writel(device, &device->memstore,
		KGSL_MEMSTORE_OFFSET(context->id, soptimestamp),
		cmdbatch->timestamp);

	kgsl_sharedmem_writel(device, &device->memstore,
		KGSL_MEMSTORE_OFFSET(context->id, eoptimestamp),
		cmdbatch->timestamp);


	
	kgsl_process_event_group(device, &context->events);

	trace_adreno_cmdbatch_retired(cmdbatch, -1, 0, 0);
	kgsl_cmdbatch_destroy(cmdbatch);
}

static int _check_context_queue(struct adreno_context *drawctxt)
{
	int ret;

	spin_lock(&drawctxt->lock);


	if (kgsl_context_invalid(&drawctxt->base))
		ret = 1;
	else
		ret = drawctxt->queued < _context_cmdqueue_size ? 1 : 0;

	spin_unlock(&drawctxt->lock);

	return ret;
}

static bool _marker_expired(struct kgsl_cmdbatch *cmdbatch)
{
	return (cmdbatch->flags & KGSL_CMDBATCH_MARKER) &&
		kgsl_check_timestamp(cmdbatch->device, cmdbatch->context,
			cmdbatch->marker_timestamp);
}

static inline void _pop_cmdbatch(struct adreno_context *drawctxt)
{
	drawctxt->cmdqueue_head = CMDQUEUE_NEXT(drawctxt->cmdqueue_head,
		ADRENO_CONTEXT_CMDQUEUE_SIZE);
	drawctxt->queued--;
}

static struct kgsl_cmdbatch *_get_cmdbatch(struct adreno_context *drawctxt)
{
	struct kgsl_cmdbatch *cmdbatch = NULL;
	bool pending = false;

	if (drawctxt->cmdqueue_head == drawctxt->cmdqueue_tail)
		return NULL;

	cmdbatch = drawctxt->cmdqueue[drawctxt->cmdqueue_head];

	
	if (cmdbatch->flags & KGSL_CMDBATCH_MARKER) {
		if (_marker_expired(cmdbatch)) {
			_pop_cmdbatch(drawctxt);
			_retire_marker(cmdbatch);

			
			return _get_cmdbatch(drawctxt);
		}


		if (!test_bit(CMDBATCH_FLAG_SKIP, &cmdbatch->priv))
			pending = true;
	}

	spin_lock_bh(&cmdbatch->lock);
	if (!list_empty(&cmdbatch->synclist))
		pending = true;
	spin_unlock_bh(&cmdbatch->lock);

	if (pending) {
		if (!cmdbatch->timeout_jiffies) {
			cmdbatch->timeout_jiffies = jiffies + 5 * HZ;
			mod_timer(&cmdbatch->timer, cmdbatch->timeout_jiffies);
		}

		return ERR_PTR(-EAGAIN);
	}

	del_timer_sync(&cmdbatch->timer);

	_pop_cmdbatch(drawctxt);
	return cmdbatch;
}

static struct kgsl_cmdbatch *adreno_dispatcher_get_cmdbatch(
		struct adreno_context *drawctxt)
{
	struct kgsl_cmdbatch *cmdbatch;

	spin_lock(&drawctxt->lock);
	cmdbatch = _get_cmdbatch(drawctxt);
	spin_unlock(&drawctxt->lock);

	return cmdbatch;
}

static inline int adreno_dispatcher_requeue_cmdbatch(
		struct adreno_context *drawctxt, struct kgsl_cmdbatch *cmdbatch)
{
	unsigned int prev;
	spin_lock(&drawctxt->lock);

	if (kgsl_context_detached(&drawctxt->base) ||
		kgsl_context_invalid(&drawctxt->base)) {
		spin_unlock(&drawctxt->lock);
		
		kgsl_cmdbatch_destroy(cmdbatch);
		return -EINVAL;
	}

	prev = drawctxt->cmdqueue_head == 0 ?
		(ADRENO_CONTEXT_CMDQUEUE_SIZE - 1) :
		(drawctxt->cmdqueue_head - 1);


	BUG_ON(prev == drawctxt->cmdqueue_tail);

	drawctxt->cmdqueue[prev] = cmdbatch;
	drawctxt->queued++;

	
	drawctxt->cmdqueue_head = prev;
	spin_unlock(&drawctxt->lock);
	return 0;
}

static void  dispatcher_queue_context(struct adreno_device *adreno_dev,
		struct adreno_context *drawctxt)
{
	struct adreno_dispatcher *dispatcher = &adreno_dev->dispatcher;

	
	if (kgsl_context_detached(&drawctxt->base))
		return;

	spin_lock(&dispatcher->plist_lock);

	if (plist_node_empty(&drawctxt->pending)) {
		
		if (_kgsl_context_get(&drawctxt->base)) {
			trace_dispatch_queue_context(drawctxt);
			plist_add(&drawctxt->pending, &dispatcher->pending);
		}
	}

	spin_unlock(&dispatcher->plist_lock);
}

static int sendcmd(struct adreno_device *adreno_dev,
	struct kgsl_cmdbatch *cmdbatch)
{
	struct kgsl_device *device = &adreno_dev->dev;
	struct adreno_dispatcher *dispatcher = &adreno_dev->dispatcher;
	struct adreno_dispatcher_cmdqueue *dispatch_q =
				ADRENO_CMDBATCH_DISPATCH_CMDQUEUE(cmdbatch);
	struct adreno_submit_time time;
	uint64_t secs = 0;
	unsigned long nsecs = 0;
	int ret;

	mutex_lock(&device->mutex);
	if (adreno_gpu_halt(adreno_dev) != 0) {
		mutex_unlock(&device->mutex);
		return -EBUSY;
	}

	dispatcher->inflight++;
	dispatch_q->inflight++;

	if (dispatcher->inflight == 1 &&
			!test_bit(ADRENO_DISPATCHER_POWER, &dispatcher->priv)) {
		
		ret = kgsl_active_count_get(device);
		if (ret) {
			dispatcher->inflight--;
			dispatch_q->inflight--;
			mutex_unlock(&device->mutex);
			return ret;
		}

		set_bit(ADRENO_DISPATCHER_POWER, &dispatcher->priv);
	}

	if (test_bit(ADRENO_DEVICE_CMDBATCH_PROFILE, &adreno_dev->priv)) {
		set_bit(CMDBATCH_FLAG_PROFILE, &cmdbatch->priv);
		cmdbatch->profile_index = adreno_dev->cmdbatch_profile_index;
		adreno_dev->cmdbatch_profile_index =
			(adreno_dev->cmdbatch_profile_index + 1) %
			ADRENO_CMDBATCH_PROFILE_COUNT;
	}

	ret = adreno_ringbuffer_submitcmd(adreno_dev, cmdbatch, &time);


	if (dispatcher->inflight == 1) {
		if (ret == 0) {
			fault_detect_read(device);

			if (!test_and_set_bit(ADRENO_DISPATCHER_ACTIVE,
				&dispatcher->priv))
				INIT_COMPLETION(dispatcher->idle_gate);
		} else {
			kgsl_active_count_put(device);
			clear_bit(ADRENO_DISPATCHER_POWER, &dispatcher->priv);
		}
	}

	mutex_unlock(&device->mutex);

	if (ret) {
		dispatcher->inflight--;
		dispatch_q->inflight--;
		KGSL_DRV_ERR(device,
			"Unable to submit command to the ringbuffer %d\n", ret);
		return ret;
	}

	secs = time.ktime;
	nsecs = do_div(secs, 1000000000);

	trace_adreno_cmdbatch_submitted(cmdbatch, (int) dispatcher->inflight,
		time.ticks, (unsigned long) secs, nsecs / 1000);

	cmdbatch->submit_ticks = time.ticks;

	dispatch_q->cmd_q[dispatch_q->tail] = cmdbatch;
	dispatch_q->tail = (dispatch_q->tail + 1) %
		ADRENO_DISPATCH_CMDQUEUE_SIZE;

	if (dispatcher->inflight == 1) {
		if (&(adreno_dev->cur_rb->dispatch_q) == dispatch_q) {
			cmdbatch->expires = jiffies +
				msecs_to_jiffies(_cmdbatch_timeout);
			mod_timer(&dispatcher->timer, cmdbatch->expires);
		}

		
		if (adreno_dev->fast_hang_detect)
			mod_timer(&dispatcher->fault_timer,
				jiffies +
				msecs_to_jiffies(_fault_timer_interval));
	}

	return 0;
}

static int dispatcher_context_sendcmds(struct adreno_device *adreno_dev,
		struct adreno_context *drawctxt)
{
	struct adreno_dispatcher_cmdqueue *dispatch_q =
					&(drawctxt->rb->dispatch_q);
	int count = 0;
	int ret = 0;
	int inflight = _cmdqueue_inflight(dispatch_q);
	unsigned int timestamp;

	if (dispatch_q->inflight >= inflight)
		return -EBUSY;

	while ((count < _context_cmdbatch_burst) &&
		(dispatch_q->inflight < inflight)) {
		struct kgsl_cmdbatch *cmdbatch;

		if (adreno_gpu_fault(adreno_dev) != 0)
			break;

		cmdbatch = adreno_dispatcher_get_cmdbatch(drawctxt);


		if (IS_ERR_OR_NULL(cmdbatch)) {
			if (IS_ERR(cmdbatch))
				ret = PTR_ERR(cmdbatch);
			break;
		}


		if (cmdbatch->flags & KGSL_CMDBATCH_SYNC) {
			kgsl_cmdbatch_destroy(cmdbatch);
			continue;
		}

		timestamp = cmdbatch->timestamp;

		ret = sendcmd(adreno_dev, cmdbatch);

		if (ret) {
			if (adreno_dispatcher_requeue_cmdbatch(drawctxt,
				cmdbatch))
				ret = -EINVAL;
			break;
		}

		drawctxt->submitted_timestamp = timestamp;

		count++;
	}


	if (_check_context_queue(drawctxt))
		wake_up_all(&drawctxt->wq);

	if (!ret)
		ret = count;

	
	return ret;
}

static int _adreno_dispatcher_issuecmds(struct adreno_device *adreno_dev)
{
	struct adreno_dispatcher *dispatcher = &adreno_dev->dispatcher;
	struct adreno_context *drawctxt, *next;
	struct plist_head requeue, busy_list;
	int ret;

	
	if (adreno_gpu_fault(adreno_dev) != 0)
			return 0;

	plist_head_init(&requeue);
	plist_head_init(&busy_list);

	
	while (1) {

		
		if (adreno_gpu_fault(adreno_dev) != 0)
			break;

		if (0 != adreno_gpu_halt(adreno_dev))
			break;

		spin_lock(&dispatcher->plist_lock);

		if (plist_head_empty(&dispatcher->pending)) {
			spin_unlock(&dispatcher->plist_lock);
			break;
		}

		
		drawctxt = plist_first_entry(&dispatcher->pending,
			struct adreno_context, pending);

		plist_del(&drawctxt->pending, &dispatcher->pending);

		spin_unlock(&dispatcher->plist_lock);

		if (kgsl_context_detached(&drawctxt->base) ||
			kgsl_context_invalid(&drawctxt->base)) {
			kgsl_context_put(&drawctxt->base);
			continue;
		}

		ret = dispatcher_context_sendcmds(adreno_dev, drawctxt);

		if (ret != 0) {
			spin_lock(&dispatcher->plist_lock);


			if (!plist_node_empty(&drawctxt->pending)) {
				plist_del(&drawctxt->pending,
						&dispatcher->pending);
				kgsl_context_put(&drawctxt->base);
			}

			if (ret == -EBUSY)
				
				plist_add(&drawctxt->pending, &busy_list);
			else
				plist_add(&drawctxt->pending, &requeue);

			spin_unlock(&dispatcher->plist_lock);
		} else {

			kgsl_context_put(&drawctxt->base);
		}
	}

	spin_lock(&dispatcher->plist_lock);

	
	plist_for_each_entry_safe(drawctxt, next, &busy_list, pending) {
		plist_del(&drawctxt->pending, &busy_list);
		plist_add(&drawctxt->pending, &dispatcher->pending);
	}

	
	plist_for_each_entry_safe(drawctxt, next, &requeue, pending) {
		plist_del(&drawctxt->pending, &requeue);
		plist_add(&drawctxt->pending, &dispatcher->pending);
	}

	spin_unlock(&dispatcher->plist_lock);

	return 0;
}

static int adreno_dispatcher_issuecmds(struct adreno_device *adreno_dev)
{
	struct adreno_dispatcher *dispatcher = &adreno_dev->dispatcher;
	int ret;

	
	if (!mutex_trylock(&dispatcher->mutex)) {
		adreno_dispatcher_schedule(&adreno_dev->dev);
		return 0;
	}

	ret = _adreno_dispatcher_issuecmds(adreno_dev);
	mutex_unlock(&dispatcher->mutex);

	return ret;
}

static int get_timestamp(struct adreno_context *drawctxt,
		struct kgsl_cmdbatch *cmdbatch, unsigned int *timestamp)
{
	
	if (cmdbatch->flags & KGSL_CMDBATCH_SYNC) {
		*timestamp = 0;
		return 0;
	}

	if (drawctxt->base.flags & KGSL_CONTEXT_USER_GENERATED_TS) {
		if (timestamp_cmp(drawctxt->timestamp, *timestamp) >= 0)
			return -ERANGE;

		drawctxt->timestamp = *timestamp;
	} else
		drawctxt->timestamp++;

	*timestamp = drawctxt->timestamp;
	return 0;
}

int adreno_dispatcher_queue_cmd(struct adreno_device *adreno_dev,
		struct adreno_context *drawctxt, struct kgsl_cmdbatch *cmdbatch,
		uint32_t *timestamp)
{
	struct adreno_dispatcher_cmdqueue *dispatch_q =
				ADRENO_CMDBATCH_DISPATCH_CMDQUEUE(cmdbatch);
	int ret;

	spin_lock(&drawctxt->lock);

	if (kgsl_context_detached(&drawctxt->base)) {
		spin_unlock(&drawctxt->lock);
		return -EINVAL;
	}


	if (test_and_clear_bit(ADRENO_CONTEXT_FORCE_PREAMBLE,
				&drawctxt->base.priv))
		set_bit(CMDBATCH_FLAG_FORCE_PREAMBLE, &cmdbatch->priv);


	if ((drawctxt->base.flags & KGSL_CONTEXT_CTX_SWITCH) ||
		(cmdbatch->flags & KGSL_CMDBATCH_CTX_SWITCH))
		set_bit(CMDBATCH_FLAG_FORCE_PREAMBLE, &cmdbatch->priv);

	
	if (drawctxt->base.flags & KGSL_CONTEXT_IFH_NOP)
		set_bit(CMDBATCH_FLAG_SKIP, &cmdbatch->priv);


	if (test_bit(ADRENO_CONTEXT_SKIP_EOF, &drawctxt->base.priv)) {
		set_bit(CMDBATCH_FLAG_SKIP, &cmdbatch->priv);


		if (cmdbatch->flags & KGSL_CMDBATCH_END_OF_FRAME) {
			clear_bit(ADRENO_CONTEXT_SKIP_EOF,
				  &drawctxt->base.priv);

			set_bit(ADRENO_CONTEXT_FORCE_PREAMBLE,
				&drawctxt->base.priv);
		}
	}

	

	while (drawctxt->queued >= _context_cmdqueue_size) {
		trace_adreno_drawctxt_sleep(drawctxt);
		spin_unlock(&drawctxt->lock);

		ret = wait_event_interruptible_timeout(drawctxt->wq,
			_check_context_queue(drawctxt),
			msecs_to_jiffies(_context_queue_wait));

		spin_lock(&drawctxt->lock);
		trace_adreno_drawctxt_wake(drawctxt);

		if (ret <= 0) {
			spin_unlock(&drawctxt->lock);
			return (ret == 0) ? -ETIMEDOUT : (int) ret;
		}
	}

	if (kgsl_context_invalid(&drawctxt->base)) {
		spin_unlock(&drawctxt->lock);
		return -EDEADLK;
	}
	if (kgsl_context_detached(&drawctxt->base)) {
		spin_unlock(&drawctxt->lock);
		return -EINVAL;
	}

	ret = get_timestamp(drawctxt, cmdbatch, timestamp);
	if (ret) {
		spin_unlock(&drawctxt->lock);
		return ret;
	}

	cmdbatch->timestamp = *timestamp;

	if (cmdbatch->flags & KGSL_CMDBATCH_MARKER) {


		if (!drawctxt->queued && kgsl_check_timestamp(cmdbatch->device,
			cmdbatch->context, drawctxt->queued_timestamp)) {
			trace_adreno_cmdbatch_queued(cmdbatch,
				drawctxt->queued);

			_retire_marker(cmdbatch);
			spin_unlock(&drawctxt->lock);
			return 0;
		}


		cmdbatch->marker_timestamp = drawctxt->queued_timestamp;
	}

	
	if (!(cmdbatch->flags & KGSL_CONTEXT_SYNC))
		drawctxt->queued_timestamp = *timestamp;


	if (drawctxt->base.flags & KGSL_CONTEXT_NO_FAULT_TOLERANCE)
		set_bit(KGSL_FT_DISABLE, &cmdbatch->fault_policy);
	else
		cmdbatch->fault_policy = adreno_dev->ft_policy;

	
	drawctxt->cmdqueue[drawctxt->cmdqueue_tail] = cmdbatch;
	drawctxt->cmdqueue_tail = (drawctxt->cmdqueue_tail + 1) %
		ADRENO_CONTEXT_CMDQUEUE_SIZE;


	if (!(cmdbatch->flags & KGSL_CMDBATCH_MARKER)) {
		unsigned int i = drawctxt->cmdqueue_head;

		while (i != drawctxt->cmdqueue_tail) {
			if (drawctxt->cmdqueue[i]->flags & KGSL_CMDBATCH_MARKER)
				set_bit(CMDBATCH_FLAG_SKIP,
					&drawctxt->cmdqueue[i]->priv);

			i = CMDQUEUE_NEXT(i, ADRENO_CONTEXT_CMDQUEUE_SIZE);
		}
	}

	drawctxt->queued++;
	trace_adreno_cmdbatch_queued(cmdbatch, drawctxt->queued);

	_track_context(dispatch_q, drawctxt->base.id);

	spin_unlock(&drawctxt->lock);

	
	dispatcher_queue_context(adreno_dev, drawctxt);


	if (dispatch_q->inflight < _context_cmdbatch_burst)
		adreno_dispatcher_issuecmds(adreno_dev);

	return 0;
}

static int _mark_context(int id, void *ptr, void *data)
{
	unsigned int guilty = *((unsigned int *) data);
	struct kgsl_context *context = ptr;


	if (guilty == 0 || guilty == context->id)
		context->reset_status =
			KGSL_CTX_STAT_GUILTY_CONTEXT_RESET_EXT;
	else if (context->reset_status !=
		KGSL_CTX_STAT_GUILTY_CONTEXT_RESET_EXT)
		context->reset_status =
			KGSL_CTX_STAT_INNOCENT_CONTEXT_RESET_EXT;

	return 0;
}

static void mark_guilty_context(struct kgsl_device *device, unsigned int id)
{
	

	read_lock(&device->context_lock);
	idr_for_each(&device->context_idr, _mark_context, &id);
	read_unlock(&device->context_lock);
}

static void cmdbatch_skip_ib(struct kgsl_cmdbatch *cmdbatch,
				unsigned int base)
{
	struct kgsl_memobj_node *ib;

	list_for_each_entry(ib, &cmdbatch->cmdlist, node) {
		if (ib->gpuaddr == base) {
			ib->priv |= MEMOBJ_SKIP;
			if (base)
				return;
		}
	}
}

static void cmdbatch_skip_cmd(struct kgsl_cmdbatch *cmdbatch,
	struct kgsl_cmdbatch **replay, int count)
{
	struct adreno_context *drawctxt = ADRENO_CONTEXT(cmdbatch->context);
	int i;

	for (i = 1; i < count; i++) {
		if (replay[i]->context->id == cmdbatch->context->id) {
			replay[i]->fault_policy = replay[0]->fault_policy;
			set_bit(CMDBATCH_FLAG_FORCE_PREAMBLE, &replay[i]->priv);
			set_bit(KGSL_FT_SKIPCMD, &replay[i]->fault_recovery);
			break;
		}
	}

	if ((i == count) && drawctxt) {
		set_bit(ADRENO_CONTEXT_SKIP_CMD, &drawctxt->base.priv);
		drawctxt->fault_policy = replay[0]->fault_policy;
	}

	
	set_bit(CMDBATCH_FLAG_SKIP, &cmdbatch->priv);
	cmdbatch->fault_recovery = 0;
}

static void cmdbatch_skip_frame(struct kgsl_cmdbatch *cmdbatch,
	struct kgsl_cmdbatch **replay, int count)
{
	struct adreno_context *drawctxt = ADRENO_CONTEXT(cmdbatch->context);
	int skip = 1;
	int i;

	for (i = 0; i < count; i++) {


		if (replay[i]->context->id != cmdbatch->context->id)
			continue;


		if (skip) {
			set_bit(CMDBATCH_FLAG_SKIP, &replay[i]->priv);

			if (replay[i]->flags & KGSL_CMDBATCH_END_OF_FRAME)
				skip = 0;
		} else {
			set_bit(CMDBATCH_FLAG_FORCE_PREAMBLE, &replay[i]->priv);
			return;
		}
	}


	if (skip && drawctxt)
		set_bit(ADRENO_CONTEXT_SKIP_EOF, &drawctxt->base.priv);


	if (!skip && drawctxt)
		set_bit(ADRENO_CONTEXT_FORCE_PREAMBLE, &drawctxt->base.priv);
}

static void remove_invalidated_cmdbatches(struct kgsl_device *device,
		struct kgsl_cmdbatch **replay, int count)
{
	int i;

	for (i = 0; i < count; i++) {
		struct kgsl_cmdbatch *cmd = replay[i];
		if (cmd == NULL)
			continue;

		if (kgsl_context_detached(cmd->context) ||
			kgsl_context_invalid(cmd->context)) {
			replay[i] = NULL;

			mutex_lock(&device->mutex);
			kgsl_cancel_events_timestamp(device,
				&cmd->context->events, cmd->timestamp);
			mutex_unlock(&device->mutex);

			kgsl_cmdbatch_destroy(cmd);
		}
	}
}

static char _pidname[TASK_COMM_LEN];

static inline const char *_kgsl_context_comm(struct kgsl_context *context)
{
	if (context && context->proc_priv)
		strlcpy(_pidname, context->proc_priv->comm, sizeof(_pidname));
	else
		snprintf(_pidname, TASK_COMM_LEN, "unknown");

	return _pidname;
}

#define pr_fault(_d, _c, fmt, args...) \
		dev_err((_d)->dev, "%s[%d]: " fmt, \
		_kgsl_context_comm((_c)->context), \
		(_c)->context->proc_priv->pid, ##args)


static void adreno_fault_header(struct kgsl_device *device,
	struct kgsl_cmdbatch *cmdbatch)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	unsigned int status, base, rptr, wptr, ib1base, ib2base, ib1sz, ib2sz;

	kgsl_regread(device,
			adreno_getreg(adreno_dev, ADRENO_REG_RBBM_STATUS),
			&status);
	kgsl_regread(device,
		adreno_getreg(adreno_dev, ADRENO_REG_CP_RB_BASE),
		&base);
	kgsl_regread(device,
		adreno_getreg(adreno_dev, ADRENO_REG_CP_RB_RPTR),
		&rptr);
	kgsl_regread(device,
		adreno_getreg(adreno_dev, ADRENO_REG_CP_RB_WPTR),
		&wptr);
	kgsl_regread(device,
		adreno_getreg(adreno_dev, ADRENO_REG_CP_IB1_BASE),
		&ib1base);
	kgsl_regread(device,
		adreno_getreg(adreno_dev, ADRENO_REG_CP_IB1_BUFSZ),
		&ib1sz);
	kgsl_regread(device,
		adreno_getreg(adreno_dev, ADRENO_REG_CP_IB2_BASE),
		&ib2base);
	kgsl_regread(device,
		adreno_getreg(adreno_dev, ADRENO_REG_CP_IB2_BUFSZ),
		&ib2sz);

	trace_adreno_gpu_fault(cmdbatch->context->id, cmdbatch->timestamp,
		status, rptr, wptr, ib1base, ib1sz, ib2base, ib2sz);

	pr_fault(device, cmdbatch,
		"gpu fault ctx %d ts %d status %8.8X rb %4.4x/%4.4x ib1 %8.8x/%4.4x ib2 %8.8x/%4.4x\n",
		cmdbatch->context->id, cmdbatch->timestamp, status,
		rptr, wptr, ib1base, ib1sz, ib2base, ib2sz);
}

void adreno_fault_skipcmd_detached(struct kgsl_device *device,
				 struct adreno_context *drawctxt,
				 struct kgsl_cmdbatch *cmdbatch)
{
	if (test_bit(ADRENO_CONTEXT_SKIP_CMD, &drawctxt->base.priv) &&
			kgsl_context_detached(&drawctxt->base)) {
		pr_fault(device, cmdbatch, "gpu %s ctx %d\n",
			 "detached", cmdbatch->context->id);
		clear_bit(ADRENO_CONTEXT_SKIP_CMD, &drawctxt->base.priv);
	}
}

void process_cmdbatch_fault(struct kgsl_device *device,
		struct kgsl_cmdbatch **replay, int count,
		unsigned int base,
		int fault)
{
	struct kgsl_cmdbatch *cmdbatch = replay[0];
	int i;
	char *state = "failed";

	if (test_bit(KGSL_FT_THROTTLE, &cmdbatch->fault_policy)) {
		if (time_after(jiffies, (cmdbatch->context->fault_time
				+ msecs_to_jiffies(_fault_throttle_time)))) {
			cmdbatch->context->fault_time = jiffies;
			cmdbatch->context->fault_count = 1;
		} else {
			cmdbatch->context->fault_count++;
			if (cmdbatch->context->fault_count >
					_fault_throttle_burst) {
				set_bit(KGSL_FT_DISABLE,
						&cmdbatch->fault_policy);
				pr_fault(device, cmdbatch,
					 "gpu fault threshold exceeded %d faults in %d msecs\n",
					 _fault_throttle_burst,
					 _fault_throttle_time);
			}
		}
	}


	if (test_bit(KGSL_FT_DISABLE, &cmdbatch->fault_policy) ||
		test_bit(KGSL_FT_TEMP_DISABLE, &cmdbatch->fault_policy)) {
		state = "skipped";
		bitmap_zero(&cmdbatch->fault_policy, BITS_PER_LONG);
	}

	
	if (kgsl_context_detached(cmdbatch->context)) {
		state = "detached";
		bitmap_zero(&cmdbatch->fault_policy, BITS_PER_LONG);
	}


	set_bit(KGSL_FT_SKIP_PMDUMP, &cmdbatch->fault_policy);


	if (fault & ADRENO_HARD_FAULT)
		clear_bit(KGSL_FT_REPLAY, &(cmdbatch->fault_policy));


	if (fault & ADRENO_TIMEOUT_FAULT)
		bitmap_zero(&cmdbatch->fault_policy, BITS_PER_LONG);


	if (test_bit(KGSL_CONTEXT_PRIV_PAGEFAULT,
		     &cmdbatch->context->priv)) {
		
		clear_bit(KGSL_FT_REPLAY, &cmdbatch->fault_policy);
		clear_bit(KGSL_CONTEXT_PRIV_PAGEFAULT,
			  &cmdbatch->context->priv);
	}


	
	if (test_and_clear_bit(KGSL_FT_REPLAY, &cmdbatch->fault_policy)) {
		trace_adreno_cmdbatch_recovery(cmdbatch, BIT(KGSL_FT_REPLAY));
		set_bit(KGSL_FT_REPLAY, &cmdbatch->fault_recovery);
		return;
	}


	if (test_and_clear_bit(KGSL_FT_SKIPIB, &cmdbatch->fault_policy)) {
		trace_adreno_cmdbatch_recovery(cmdbatch, BIT(KGSL_FT_SKIPIB));
		set_bit(KGSL_FT_SKIPIB, &cmdbatch->fault_recovery);

		for (i = 0; i < count; i++) {
			if (replay[i] != NULL &&
				replay[i]->context->id == cmdbatch->context->id)
				cmdbatch_skip_ib(replay[i], base);
		}

		return;
	}

	
	if (test_and_clear_bit(KGSL_FT_SKIPCMD, &cmdbatch->fault_policy)) {
		trace_adreno_cmdbatch_recovery(cmdbatch, BIT(KGSL_FT_SKIPCMD));

		
		cmdbatch_skip_cmd(cmdbatch, replay, count);

		return;
	}

	if (test_and_clear_bit(KGSL_FT_SKIPFRAME, &cmdbatch->fault_policy)) {
		trace_adreno_cmdbatch_recovery(cmdbatch,
			BIT(KGSL_FT_SKIPFRAME));
		set_bit(KGSL_FT_SKIPFRAME, &cmdbatch->fault_recovery);

		cmdbatch_skip_frame(cmdbatch, replay, count);
		return;
	}

	

	pr_fault(device, cmdbatch, "gpu %s ctx %d ts %d\n",
		state, cmdbatch->context->id, cmdbatch->timestamp);

	
	mark_guilty_context(device, cmdbatch->context->id);

	
	adreno_drawctxt_invalidate(device, cmdbatch->context);
}

static void recover_dispatch_q(struct kgsl_device *device,
		struct adreno_dispatcher_cmdqueue *dispatch_q,
		int fault,
		unsigned int base)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct kgsl_cmdbatch **replay = NULL;
	unsigned int ptr;
	int first = 0;
	int count = 0;
	int i;

	
	replay = kzalloc(sizeof(*replay) * dispatch_q->inflight, GFP_KERNEL);

	if (replay == NULL) {
		unsigned int ptr = dispatch_q->head;

		
		while (ptr != dispatch_q->tail) {
			struct kgsl_context *context =
				dispatch_q->cmd_q[ptr]->context;

			mark_guilty_context(device, context->id);
			adreno_drawctxt_invalidate(device, context);
			kgsl_cmdbatch_destroy(dispatch_q->cmd_q[ptr]);

			ptr = CMDQUEUE_NEXT(ptr, ADRENO_DISPATCH_CMDQUEUE_SIZE);
		}


		count = 0;
		goto replay;
	}

	
	ptr = dispatch_q->head;

	while (ptr != dispatch_q->tail) {
		replay[count++] = dispatch_q->cmd_q[ptr];
		ptr = CMDQUEUE_NEXT(ptr, ADRENO_DISPATCH_CMDQUEUE_SIZE);
	}

	if (fault && count)
		process_cmdbatch_fault(device, replay,
					count, base, fault);
replay:
	dispatch_q->inflight = 0;
	dispatch_q->head = dispatch_q->tail = 0;
	
	remove_invalidated_cmdbatches(device, replay, count);

	
	for (i = 0; i < count; i++) {

		int ret;

		if (replay[i] == NULL)
			continue;


		if (first == 0) {
			set_bit(CMDBATCH_FLAG_FORCE_PREAMBLE, &replay[i]->priv);
			first = 1;
		}


		set_bit(CMDBATCH_FLAG_WFI, &replay[i]->priv);

		ret = sendcmd(adreno_dev, replay[i]);


		if (ret) {
			pr_fault(device, replay[i],
				"gpu reset failed ctx %d ts %d\n",
				replay[i]->context->id, replay[i]->timestamp);

			
			mark_guilty_context(device, replay[i]->context->id);

			adreno_drawctxt_invalidate(device, replay[i]->context);
			remove_invalidated_cmdbatches(device, &replay[i],
				count - i);
		}
	}

	
	clear_bit(ADRENO_DEVICE_FAULT, &adreno_dev->priv);

	kfree(replay);
}

static int dispatcher_do_fault(struct kgsl_device *device)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct adreno_dispatcher *dispatcher = &adreno_dev->dispatcher;
	struct adreno_dispatcher_cmdqueue *dispatch_q = NULL, *dispatch_q_temp;
	struct adreno_ringbuffer *rb;
	unsigned int reg, base;
	struct kgsl_cmdbatch *cmdbatch = NULL;
	int ret, i;
	int fault;
	int halt;
	int keepfault, fault_pid = 0;

	fault = atomic_xchg(&dispatcher->fault, 0);
	keepfault = fault;
	if (fault == 0)
		return 0;
	if (dispatcher->inflight == 0) {
		KGSL_DRV_WARN(device,
		"dispatcher_do_fault with 0 inflight commands\n");
		mutex_lock(&device->mutex);
		if (device->state == KGSL_STATE_AWARE)
			ret = kgsl_pwrctrl_change_state(device,
				KGSL_STATE_ACTIVE);
		else
			ret = 0;
		mutex_unlock(&device->mutex);
		return ret;
	}

	
	del_timer_sync(&dispatcher->timer);
	del_timer_sync(&dispatcher->fault_timer);

	mutex_lock(&device->mutex);

	
	kgsl_cffdump_hang(device);

	adreno_readreg(adreno_dev, ADRENO_REG_CP_RB_BASE, &base);


	if (fault & ADRENO_TIMEOUT_FAULT) {
		adreno_readreg(adreno_dev, ADRENO_REG_CP_ME_CNTL, &reg);
		reg |= (1 << 27) | (1 << 28);
		adreno_writereg(adreno_dev, ADRENO_REG_CP_ME_CNTL, reg);
	}
	FOR_EACH_RINGBUFFER(adreno_dev, rb, i) {
		adreno_dispatch_process_cmdqueue(adreno_dev,
			&(rb->dispatch_q), 0);
		
		if (base == rb->buffer_desc.gpuaddr)
			dispatch_q = &(rb->dispatch_q);
	}

	if (dispatch_q && (dispatch_q->tail != dispatch_q->head)) {
		cmdbatch = dispatch_q->cmd_q[dispatch_q->head];
		fault_pid = cmdbatch->context->proc_priv->pid;
		trace_adreno_cmdbatch_fault(cmdbatch, fault);
	}

	
	kgsl_mmu_set_pagefault(&device->mmu);

	adreno_readreg(adreno_dev, ADRENO_REG_CP_IB1_BASE, &base);


	if (cmdbatch &&
		!test_bit(KGSL_FT_SKIP_PMDUMP, &cmdbatch->fault_policy)) {
		adreno_fault_header(device, cmdbatch);
		kgsl_device_snapshot(device, cmdbatch->context);
	}

	
	dispatcher->inflight = 0;

	
	halt = adreno_gpu_halt(adreno_dev);
	adreno_clear_gpu_halt(adreno_dev);
	ret = adreno_reset(device);
	mutex_unlock(&device->mutex);
	
	atomic_set(&dispatcher->fault, 0);

	
	BUG_ON(ret);

	
	if (dispatch_q)
		recover_dispatch_q(device, dispatch_q, fault, base);
	FOR_EACH_RINGBUFFER(adreno_dev, rb, i) {
		dispatch_q_temp = &(rb->dispatch_q);
		if (dispatch_q_temp != dispatch_q)
			recover_dispatch_q(device, dispatch_q_temp, 0, base);
	}

	atomic_add(halt, &adreno_dev->halt);

	adreno_fault_panic(device, fault_pid, keepfault);

	return 1;
}

static inline int cmdbatch_consumed(struct kgsl_cmdbatch *cmdbatch,
		unsigned int consumed, unsigned int retired)
{
	return ((timestamp_cmp(cmdbatch->timestamp, consumed) >= 0) &&
		(timestamp_cmp(retired, cmdbatch->timestamp) < 0));
}

static void _print_recovery(struct kgsl_device *device,
		struct kgsl_cmdbatch *cmdbatch)
{
	static struct {
		unsigned int mask;
		const char *str;
	} flags[] = { ADRENO_FT_TYPES };

	int i, nr = find_first_bit(&cmdbatch->fault_recovery, BITS_PER_LONG);
	char *result = "unknown";

	for (i = 0; i < ARRAY_SIZE(flags); i++) {
		if (flags[i].mask == BIT(nr)) {
			result = (char *) flags[i].str;
			break;
		}
	}

	pr_fault(device, cmdbatch,
		"gpu %s ctx %d ts %d policy %lX\n",
		result, cmdbatch->context->id, cmdbatch->timestamp,
		cmdbatch->fault_recovery);
}

static void cmdbatch_profile_ticks(struct adreno_device *adreno_dev,
	struct kgsl_cmdbatch *cmdbatch, uint64_t *start, uint64_t *retire)
{
	void *ptr = adreno_dev->cmdbatch_profile_buffer.hostptr;
	struct adreno_cmdbatch_profile_entry *entry;

	entry = (struct adreno_cmdbatch_profile_entry *)
		(ptr + (cmdbatch->profile_index * sizeof(*entry)));

	rmb();
	*start = entry->started;
	*retire = entry->retired;
}

static int adreno_dispatch_process_cmdqueue(struct adreno_device *adreno_dev,
				struct adreno_dispatcher_cmdqueue *dispatch_q,
				int long_ib_detect)
{
	struct kgsl_device *device = &(adreno_dev->dev);
	struct adreno_dispatcher *dispatcher = &(adreno_dev->dispatcher);
	uint64_t start_ticks = 0, retire_ticks = 0;

	struct adreno_dispatcher_cmdqueue *active_q =
			&(adreno_dev->cur_rb->dispatch_q);
	int count = 0;

	while (dispatch_q->head != dispatch_q->tail) {
		struct kgsl_cmdbatch *cmdbatch =
			dispatch_q->cmd_q[dispatch_q->head];
		struct adreno_context *drawctxt;
		BUG_ON(cmdbatch == NULL);

		drawctxt = ADRENO_CONTEXT(cmdbatch->context);


		if (kgsl_check_timestamp(device, cmdbatch->context,
			cmdbatch->timestamp)) {


			if (cmdbatch->fault_recovery != 0) {
				
				set_bit(ADRENO_CONTEXT_FAULT,
					&cmdbatch->context->priv);

				_print_recovery(device, cmdbatch);
			}

			
			dispatcher->inflight--;
			dispatch_q->inflight--;


			if (test_bit(CMDBATCH_FLAG_PROFILE, &cmdbatch->priv))
				cmdbatch_profile_ticks(adreno_dev, cmdbatch,
					&start_ticks, &retire_ticks);

			trace_adreno_cmdbatch_retired(cmdbatch,
				(int) dispatcher->inflight, start_ticks,
				retire_ticks);

			
			drawctxt->submit_retire_ticks[drawctxt->ticks_index] =
				retire_ticks - cmdbatch->submit_ticks;

			drawctxt->ticks_index = (drawctxt->ticks_index + 1)
				% SUBMIT_RETIRE_TICKS_SIZE;

			
			dispatch_q->cmd_q[dispatch_q->head] = NULL;

			
			dispatch_q->head = CMDQUEUE_NEXT(dispatch_q->head,
				ADRENO_DISPATCH_CMDQUEUE_SIZE);

			
			kgsl_cmdbatch_destroy(cmdbatch);

			

			if (dispatch_q->inflight > 0 &&
				dispatch_q == active_q) {
				cmdbatch =
					dispatch_q->cmd_q[dispatch_q->head];
				cmdbatch->expires = jiffies +
					msecs_to_jiffies(_cmdbatch_timeout);
			}

			count++;
			continue;
		}

		if (!long_ib_detect ||
			drawctxt->base.flags & KGSL_CONTEXT_NO_FAULT_TOLERANCE
			|| dispatch_q != active_q)
			break;


		if (time_is_after_jiffies(cmdbatch->expires))
			break;

		

		pr_fault(device, cmdbatch,
			"gpu timeout ctx %d ts %d\n",
			cmdbatch->context->id, cmdbatch->timestamp);

		adreno_set_gpu_fault(adreno_dev, ADRENO_TIMEOUT_FAULT);
		break;
	}
	return count;
}

static void adreno_dispatcher_work(struct work_struct *work)
{
	struct adreno_dispatcher *dispatcher =
		container_of(work, struct adreno_dispatcher, work);
	struct adreno_device *adreno_dev =
		container_of(dispatcher, struct adreno_device, dispatcher);
	struct kgsl_device *device = &adreno_dev->dev;
	int count;

	mutex_lock(&dispatcher->mutex);

	
	count = adreno_dispatch_process_cmdqueue(adreno_dev,
		&(adreno_dev->cur_rb->dispatch_q),
		adreno_dev->long_ib_detect);

	
	if (dispatcher_do_fault(device))
		goto done;

	if (dispatcher->inflight == 0 && count)
		queue_work(device->work_queue, &device->event_work);

	
	_adreno_dispatcher_issuecmds(adreno_dev);

done:
	
	if (dispatcher->inflight) {
		struct kgsl_cmdbatch *cmdbatch =
			adreno_dev->cur_rb->dispatch_q.cmd_q[
				adreno_dev->cur_rb->dispatch_q.head];
		if (cmdbatch)
			
			mod_timer(&dispatcher->timer, cmdbatch->expires);

		
		mutex_lock(&device->mutex);
		kgsl_pwrscale_update(device);
		mod_timer(&device->idle_timer, jiffies +
				device->pwrctrl.interval_timeout);
		mutex_unlock(&device->mutex);
	} else {
		
		mutex_lock(&device->mutex);

		if (test_and_clear_bit(ADRENO_DISPATCHER_ACTIVE,
			&dispatcher->priv))
			complete_all(&dispatcher->idle_gate);

		del_timer_sync(&dispatcher->fault_timer);

		if (test_bit(ADRENO_DISPATCHER_POWER, &dispatcher->priv)) {
			kgsl_active_count_put(device);
			clear_bit(ADRENO_DISPATCHER_POWER, &dispatcher->priv);
		}

		mutex_unlock(&device->mutex);
	}

	mutex_unlock(&dispatcher->mutex);
}

void adreno_dispatcher_schedule(struct kgsl_device *device)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct adreno_dispatcher *dispatcher = &adreno_dev->dispatcher;

	queue_work(device->work_queue, &dispatcher->work);
}

void adreno_dispatcher_queue_context(struct kgsl_device *device,
	struct adreno_context *drawctxt)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);

	dispatcher_queue_context(adreno_dev, drawctxt);
	adreno_dispatcher_schedule(device);
}


static void adreno_dispatcher_fault_timer(unsigned long data)
{
	struct adreno_device *adreno_dev = (struct adreno_device *) data;
	struct kgsl_device *device = &adreno_dev->dev;
	struct adreno_dispatcher *dispatcher = &adreno_dev->dispatcher;

	
	if (adreno_dev->fast_hang_detect == 0)
		return;

	if (adreno_gpu_fault(adreno_dev)) {
		adreno_dispatcher_schedule(device);
		return;
	}


	if (!fault_detect_read_compare(device)) {
		adreno_set_gpu_fault(adreno_dev, ADRENO_SOFT_FAULT);
		adreno_dispatcher_schedule(device);
	} else {
		mod_timer(&dispatcher->fault_timer,
			jiffies + msecs_to_jiffies(_fault_timer_interval));
	}
}

static void adreno_dispatcher_timer(unsigned long data)
{
	struct adreno_device *adreno_dev = (struct adreno_device *) data;
	struct kgsl_device *device = &adreno_dev->dev;

	adreno_dispatcher_schedule(device);
}
void adreno_dispatcher_irq_fault(struct kgsl_device *device)
{
	adreno_set_gpu_fault(ADRENO_DEVICE(device), ADRENO_HARD_FAULT);
	adreno_dispatcher_schedule(device);
}

void adreno_dispatcher_start(struct kgsl_device *device)
{
	complete_all(&device->cmdbatch_gate);

	
	adreno_dispatcher_schedule(device);
}

void adreno_dispatcher_stop(struct adreno_device *adreno_dev)
{
	struct adreno_dispatcher *dispatcher = &adreno_dev->dispatcher;

	del_timer_sync(&dispatcher->timer);
	del_timer_sync(&dispatcher->fault_timer);
}

void adreno_dispatcher_close(struct adreno_device *adreno_dev)
{
	struct adreno_dispatcher *dispatcher = &adreno_dev->dispatcher;
	int i;
	struct adreno_ringbuffer *rb;

	mutex_lock(&dispatcher->mutex);
	del_timer_sync(&dispatcher->timer);
	del_timer_sync(&dispatcher->fault_timer);

	FOR_EACH_RINGBUFFER(adreno_dev, rb, i) {
		struct adreno_dispatcher_cmdqueue *dispatch_q =
			&(rb->dispatch_q);
		while (dispatch_q->head != dispatch_q->tail) {
			kgsl_cmdbatch_destroy(
				dispatch_q->cmd_q[dispatch_q->head]);
			dispatch_q->head = (dispatch_q->head + 1)
				% ADRENO_DISPATCH_CMDQUEUE_SIZE;
		}
	}

	mutex_unlock(&dispatcher->mutex);

	kobject_put(&dispatcher->kobj);
}

struct dispatcher_attribute {
	struct attribute attr;
	ssize_t (*show)(struct adreno_dispatcher *,
			struct dispatcher_attribute *, char *);
	ssize_t (*store)(struct adreno_dispatcher *,
			struct dispatcher_attribute *, const char *buf,
			size_t count);
	unsigned int max;
	unsigned int *value;
};

#define DISPATCHER_UINT_ATTR(_name, _mode, _max, _value) \
	struct dispatcher_attribute dispatcher_attr_##_name =  { \
		.attr = { .name = __stringify(_name), .mode = _mode }, \
		.show = _show_uint, \
		.store = _store_uint, \
		.max = _max, \
		.value = &(_value), \
	}

#define to_dispatcher_attr(_a) \
	container_of((_a), struct dispatcher_attribute, attr)
#define to_dispatcher(k) container_of(k, struct adreno_dispatcher, kobj)

static ssize_t _store_uint(struct adreno_dispatcher *dispatcher,
		struct dispatcher_attribute *attr,
		const char *buf, size_t size)
{
	unsigned int val = 0;
	int ret;

	ret = kgsl_sysfs_store(buf, &val);
	if (ret)
		return ret;

	if (!val || (attr->max && (val > attr->max)))
		return -EINVAL;

	*((unsigned int *) attr->value) = val;
	return size;
}

static ssize_t _show_uint(struct adreno_dispatcher *dispatcher,
		struct dispatcher_attribute *attr,
		char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n",
		*((unsigned int *) attr->value));
}

static DISPATCHER_UINT_ATTR(inflight, 0644, ADRENO_DISPATCH_CMDQUEUE_SIZE,
	_dispatcher_q_inflight_hi);

static DISPATCHER_UINT_ATTR(inflight_low_latency, 0644,
	ADRENO_DISPATCH_CMDQUEUE_SIZE, _dispatcher_q_inflight_lo);
static DISPATCHER_UINT_ATTR(context_cmdqueue_size, 0644,
	ADRENO_CONTEXT_CMDQUEUE_SIZE - 1, _context_cmdqueue_size);
static DISPATCHER_UINT_ATTR(context_burst_count, 0644, 0,
	_context_cmdbatch_burst);
static DISPATCHER_UINT_ATTR(cmdbatch_timeout, 0644, 0, _cmdbatch_timeout);
static DISPATCHER_UINT_ATTR(context_queue_wait, 0644, 0, _context_queue_wait);
static DISPATCHER_UINT_ATTR(fault_detect_interval, 0644, 0,
	_fault_timer_interval);
static DISPATCHER_UINT_ATTR(fault_throttle_time, 0644, 0,
	_fault_throttle_time);
static DISPATCHER_UINT_ATTR(fault_throttle_burst, 0644, 0,
	_fault_throttle_burst);

static struct attribute *dispatcher_attrs[] = {
	&dispatcher_attr_inflight.attr,
	&dispatcher_attr_inflight_low_latency.attr,
	&dispatcher_attr_context_cmdqueue_size.attr,
	&dispatcher_attr_context_burst_count.attr,
	&dispatcher_attr_cmdbatch_timeout.attr,
	&dispatcher_attr_context_queue_wait.attr,
	&dispatcher_attr_fault_detect_interval.attr,
	&dispatcher_attr_fault_throttle_time.attr,
	&dispatcher_attr_fault_throttle_burst.attr,
	NULL,
};

static ssize_t dispatcher_sysfs_show(struct kobject *kobj,
				   struct attribute *attr, char *buf)
{
	struct adreno_dispatcher *dispatcher = to_dispatcher(kobj);
	struct dispatcher_attribute *pattr = to_dispatcher_attr(attr);
	ssize_t ret = -EIO;

	if (pattr->show)
		ret = pattr->show(dispatcher, pattr, buf);

	return ret;
}

static ssize_t dispatcher_sysfs_store(struct kobject *kobj,
				    struct attribute *attr,
				    const char *buf, size_t count)
{
	struct adreno_dispatcher *dispatcher = to_dispatcher(kobj);
	struct dispatcher_attribute *pattr = to_dispatcher_attr(attr);
	ssize_t ret = -EIO;

	if (pattr->store)
		ret = pattr->store(dispatcher, pattr, buf, count);

	return ret;
}

static void dispatcher_sysfs_release(struct kobject *kobj)
{
}

static const struct sysfs_ops dispatcher_sysfs_ops = {
	.show = dispatcher_sysfs_show,
	.store = dispatcher_sysfs_store
};

static struct kobj_type ktype_dispatcher = {
	.sysfs_ops = &dispatcher_sysfs_ops,
	.default_attrs = dispatcher_attrs,
	.release = dispatcher_sysfs_release
};

int adreno_dispatcher_init(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = &adreno_dev->dev;
	struct adreno_dispatcher *dispatcher = &adreno_dev->dispatcher;
	int ret;

	memset(dispatcher, 0, sizeof(*dispatcher));

	mutex_init(&dispatcher->mutex);

	setup_timer(&dispatcher->timer, adreno_dispatcher_timer,
		(unsigned long) adreno_dev);

	setup_timer(&dispatcher->fault_timer, adreno_dispatcher_fault_timer,
		(unsigned long) adreno_dev);

	INIT_WORK(&dispatcher->work, adreno_dispatcher_work);

	init_completion(&dispatcher->idle_gate);
	complete_all(&dispatcher->idle_gate);

	plist_head_init(&dispatcher->pending);
	spin_lock_init(&dispatcher->plist_lock);

	ret = kobject_init_and_add(&dispatcher->kobj, &ktype_dispatcher,
		&device->dev->kobj, "dispatch");

	return ret;
}

int adreno_dispatcher_idle(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = &adreno_dev->dev;
	struct adreno_dispatcher *dispatcher = &adreno_dev->dispatcher;
	int ret;

	BUG_ON(!mutex_is_locked(&device->mutex));
	if (!test_bit(ADRENO_DEVICE_STARTED, &adreno_dev->priv))
		return 0;

	if (mutex_is_locked(&dispatcher->mutex) &&
		dispatcher->mutex.owner == current)
		BUG_ON(1);

	adreno_get_gpu_halt(adreno_dev);

	mutex_unlock(&device->mutex);

	ret = wait_for_completion_timeout(&dispatcher->idle_gate,
			msecs_to_jiffies(ADRENO_IDLE_TIMEOUT));
	if (ret == 0) {
		ret = -ETIMEDOUT;
		WARN(1, "Dispatcher halt timeout ");
	} else if (ret < 0) {
		KGSL_DRV_ERR(device, "Dispatcher halt failed %d\n", ret);
	} else {
		ret = 0;
	}

	mutex_lock(&device->mutex);
	adreno_put_gpu_halt(adreno_dev);
	adreno_dispatcher_schedule(device);
	return ret;
}
