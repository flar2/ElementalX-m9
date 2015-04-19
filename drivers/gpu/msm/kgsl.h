/* Copyright (c) 2008-2014, The Linux Foundation. All rights reserved.
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
#ifndef __KGSL_H
#define __KGSL_H

#include <linux/types.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/msm_kgsl.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/cdev.h>
#include <linux/regulator/consumer.h>
#include <linux/mm.h>
#include <linux/dma-attrs.h>

#include "kgsl_htc.h"

#define KGSL_MEMSTORE_SIZE	((int)(PAGE_SIZE * 2))
#define KGSL_MEMSTORE_GLOBAL	(0)
#define KGSL_PRIORITY_MAX_RB_LEVELS 4
#define KGSL_MEMSTORE_MAX	(KGSL_MEMSTORE_SIZE / \
	sizeof(struct kgsl_devmemstore) - 1 - KGSL_PRIORITY_MAX_RB_LEVELS)

#define KGSL_TIMESTAMP_WINDOW 0x80000000

#define DRM_KGSL_GEM_CACHE_OP_TO_DEV	0x0001
#define DRM_KGSL_GEM_CACHE_OP_FROM_DEV	0x0002

#define KGSL_SVM_UPPER_BOUND (0xC0000000 - SZ_16M)

#define KGSL_SVM_LOWER_BOUND PAGE_SIZE


#define KGSL_STATS_ADD(_size, _stat, _max) \
	do { _stat += (_size); if (_stat > _max) _max = _stat; } while (0)

#define KGSL_MAX_NUMIBS 100000

struct kgsl_device;
struct kgsl_context;

struct kgsl_driver {
	struct cdev cdev;
	dev_t major;
	struct class *class;
	
	struct device virtdev;
	
	struct kobject *ptkobj;
	struct kobject *prockobj;
	struct kgsl_device *devp[KGSL_DEVICE_MAX];

	
	struct list_head process_list;
	
	struct list_head pagetable_list;
	
	spinlock_t ptlock;
	
	struct mutex process_mutex;

	
	struct mutex devlock;

	struct {
		unsigned int vmalloc;
		unsigned int vmalloc_max;
		unsigned int page_alloc;
		unsigned int page_alloc_max;
		unsigned int coherent;
		unsigned int coherent_max;
		unsigned int secure;
		unsigned int secure_max;
		unsigned int mapped;
		unsigned int mapped_max;
	} stats;
	unsigned int full_cache_threshold;
	struct kgsl_driver_htc_priv priv;
};

extern struct kgsl_driver kgsl_driver;

struct kgsl_pagetable;
struct kgsl_memdesc;
struct kgsl_cmdbatch;

struct kgsl_memdesc_ops {
	unsigned int vmflags;
	int (*vmfault)(struct kgsl_memdesc *, struct vm_area_struct *,
		       struct vm_fault *);
	void (*free)(struct kgsl_memdesc *memdesc);
	int (*map_kernel)(struct kgsl_memdesc *);
	void (*unmap_kernel)(struct kgsl_memdesc *);
};

#define KGSL_MEMDESC_GUARD_PAGE BIT(0)
#define KGSL_MEMDESC_GLOBAL BIT(1)
#define KGSL_MEMDESC_FROZEN BIT(2)
#define KGSL_MEMDESC_MAPPED BIT(3)
#define KGSL_MEMDESC_GENPOOL_ALLOC BIT(4)
#define KGSL_MEMDESC_SECURE BIT(5)
#define KGSL_MEMDESC_BITMAP_ALLOC BIT(6)
#define KGSL_MEMDESC_PRIVATE BIT(7)
#define KGSL_MEMDESC_PRIVILEGED BIT(8)
#define KGSL_MEMDESC_TZ_LOCKED BIT(9)

struct kgsl_memdesc {
	struct kgsl_pagetable *pagetable;
	void *hostptr; 
	unsigned int hostptr_count; 
	unsigned long useraddr; 
	unsigned int gpuaddr;
	phys_addr_t physaddr;
	size_t size;
	unsigned int priv; 
	struct scatterlist *sg;
	unsigned int sglen; 
	struct kgsl_memdesc_ops *ops;
	unsigned int flags; 
	struct device *dev;
	struct dma_attrs attrs;
	struct kgsl_process_private *private;
};

#if 0
#define KGSL_MEM_ENTRY_KERNEL 0
#define KGSL_MEM_ENTRY_PMEM (KGSL_USER_MEM_TYPE_PMEM + 1)
#define KGSL_MEM_ENTRY_ASHMEM (KGSL_USER_MEM_TYPE_ASHMEM + 1)
#define KGSL_MEM_ENTRY_USER (KGSL_USER_MEM_TYPE_ADDR + 1)
#define KGSL_MEM_ENTRY_ION (KGSL_USER_MEM_TYPE_ION + 1)
#define KGSL_MEM_ENTRY_MAX (KGSL_USER_MEM_TYPE_MAX + 1)
#else
enum {
	KGSL_MEM_ENTRY_KERNEL = 0,
	KGSL_MEM_ENTRY_PMEM,
	KGSL_MEM_ENTRY_ASHMEM,
	KGSL_MEM_ENTRY_USER,
	KGSL_MEM_ENTRY_ION,
	KGSL_MEM_ENTRY_PAGE_ALLOC,
	KGSL_MEM_ENTRY_PRE_ALLOC,
	KGSL_MEM_ENTRY_MAX,
};
#endif
#define KGSL_MEM_TYPES \
	{ KGSL_MEM_ENTRY_KERNEL, "gpumem" }, \
	{ KGSL_MEM_ENTRY_PMEM, "pmem" }, \
	{ KGSL_MEM_ENTRY_ASHMEM, "ashmem" }, \
	{ KGSL_MEM_ENTRY_USER, "usermem" }, \
	{ KGSL_MEM_ENTRY_ION, "ion" }

struct kgsl_mem_entry {
	struct kref refcount;
	struct kgsl_memdesc memdesc;
	void *priv_data;
	struct rb_node node;
	unsigned int id;
	struct kgsl_process_private *priv;
	int pending_free;
};

struct kgsl_device_private;
struct kgsl_event_group;

typedef void (*kgsl_event_func)(struct kgsl_device *, struct kgsl_event_group *,
		void *, int);

struct kgsl_event {
	struct kgsl_device *device;
	struct kgsl_context *context;
	unsigned int timestamp;
	kgsl_event_func func;
	void *priv;
	struct list_head node;
	unsigned int created;
	struct work_struct work;
	int result;
	struct kgsl_event_group *group;
};

typedef int (*readtimestamp_func)(struct kgsl_device *, void *,
	enum kgsl_timestamp_type, unsigned int *);

struct kgsl_event_group {
	struct kgsl_context *context;
	spinlock_t lock;
	struct list_head events;
	struct list_head group;
	unsigned int processed;
	char name[64];
	readtimestamp_func readtimestamp;
	void *priv;
};

long kgsl_ioctl_device_getproperty(struct kgsl_device_private *dev_priv,
					  unsigned int cmd, void *data);
long kgsl_ioctl_device_setproperty(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_device_waittimestamp_ctxtid(struct kgsl_device_private
				*dev_priv, unsigned int cmd, void *data);
long kgsl_ioctl_rb_issueibcmds(struct kgsl_device_private *dev_priv,
				      unsigned int cmd, void *data);
long kgsl_ioctl_submit_commands(struct kgsl_device_private *dev_priv,
				unsigned int cmd, void *data);
long kgsl_ioctl_cmdstream_readtimestamp_ctxtid(struct kgsl_device_private
					*dev_priv, unsigned int cmd,
					void *data);
long kgsl_ioctl_cmdstream_freememontimestamp_ctxtid(
						struct kgsl_device_private
						*dev_priv, unsigned int cmd,
						void *data);
long kgsl_ioctl_drawctxt_create(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_drawctxt_destroy(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_sharedmem_free(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_gpumem_free_id(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_map_user_mem(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_gpumem_sync_cache(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_gpumem_sync_cache_bulk(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_sharedmem_flush_cache(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_gpumem_alloc(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_gpumem_alloc_id(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_gpumem_get_info(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_cff_syncmem(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_cff_user_event(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_timestamp_event(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);

int kgsl_cmdbatch_add_memobj(struct kgsl_cmdbatch *cmdbatch,
			struct kgsl_ibdesc *ibdesc);

int kgsl_cmdbatch_add_sync(struct kgsl_device *device,
			struct kgsl_cmdbatch *cmdbatch,
			struct kgsl_cmd_syncpoint *sync);

void kgsl_mem_entry_destroy(struct kref *kref);

struct kgsl_mem_entry *kgsl_sharedmem_find_region(
	struct kgsl_process_private *private, unsigned int gpuaddr,
	size_t size);

void kgsl_get_memory_usage(char *str, size_t len, unsigned int memflags);

extern const struct dev_pm_ops kgsl_pm_ops;

int kgsl_suspend_driver(struct platform_device *pdev, pm_message_t state);
int kgsl_resume_driver(struct platform_device *pdev);

#ifdef CONFIG_MSM_KGSL_DRM
extern int kgsl_drm_init(struct platform_device *dev);
extern void kgsl_drm_exit(void);
#else
static inline int kgsl_drm_init(struct platform_device *dev)
{
	return 0;
}

static inline void kgsl_drm_exit(void)
{
}
#endif

static inline int kgsl_gpuaddr_in_memdesc(const struct kgsl_memdesc *memdesc,
				unsigned int gpuaddr, size_t size)
{
	
	if (!size)
		size = 1;

	
	if (size > UINT_MAX - gpuaddr)
		return 0;

	if (gpuaddr >= memdesc->gpuaddr &&
	    ((gpuaddr + size) <= (memdesc->gpuaddr + memdesc->size))) {
		return 1;
	}
	return 0;
}

static inline void *kgsl_memdesc_map(struct kgsl_memdesc *memdesc)
{
	if (memdesc->ops && memdesc->ops->map_kernel)
		memdesc->ops->map_kernel(memdesc);

	return memdesc->hostptr;
}

static inline void kgsl_memdesc_unmap(struct kgsl_memdesc *memdesc)
{
	if (memdesc->ops && memdesc->ops->unmap_kernel)
		memdesc->ops->unmap_kernel(memdesc);
}

static inline void *kgsl_gpuaddr_to_vaddr(struct kgsl_memdesc *memdesc,
					     unsigned int gpuaddr)
{
	void *hostptr = NULL;

	if ((gpuaddr >= memdesc->gpuaddr) &&
		(gpuaddr < (memdesc->gpuaddr + memdesc->size)))
		hostptr = kgsl_memdesc_map(memdesc);

	return hostptr != NULL ? hostptr + (gpuaddr - memdesc->gpuaddr) : NULL;
}

static inline int timestamp_cmp(unsigned int a, unsigned int b)
{
	
	if (a == b)
		return 0;

	
	if ((a > b) && (a - b < KGSL_TIMESTAMP_WINDOW))
		return 1;

	a += KGSL_TIMESTAMP_WINDOW;
	b += KGSL_TIMESTAMP_WINDOW;
	return ((a > b) && (a - b <= KGSL_TIMESTAMP_WINDOW)) ? 1 : -1;
}

static inline int
kgsl_mem_entry_get(struct kgsl_mem_entry *entry)
{
	return kref_get_unless_zero(&entry->refcount);
}

static inline void
kgsl_mem_entry_put(struct kgsl_mem_entry *entry)
{
	kref_put(&entry->refcount, kgsl_mem_entry_destroy);
}

static inline bool kgsl_addr_range_overlap(unsigned int gpuaddr1,
		unsigned int size1,
		unsigned int gpuaddr2, unsigned int size2)
{
	if ((size1 > (UINT_MAX - gpuaddr1)) || (size2 > (UINT_MAX - gpuaddr2)))
		return false;
	return !(((gpuaddr1 + size1) <= gpuaddr2) ||
		(gpuaddr1 >= (gpuaddr2 + size2)));
}

static inline void *kgsl_malloc(size_t size)
{
	if (size <= PAGE_SIZE)
		return kzalloc(size, GFP_KERNEL);

	return vmalloc(size);
}

static inline void kgsl_free(void *ptr)
{
	if (ptr != NULL && is_vmalloc_addr(ptr))
		return vfree(ptr);

	kfree(ptr);
}

#endif 
