#ifndef KGSL_HTC_H
#define KGSL_HTC_H

#include <linux/msm_kgsl.h>

#define KGSL_CONTEXT_CHECK_RATIO_DIVIDEND	(19)
#define KGSL_CONTEXT_CHECK_RATIO_DIVISOR	(20)
#define KGSL_CONTEXT_CHECK_THRESHOLD	\
	(KGSL_MEMSTORE_MAX * \
		KGSL_CONTEXT_CHECK_RATIO_DIVIDEND / \
		KGSL_CONTEXT_CHECK_RATIO_DIVISOR)

#define KGSL_CONTEXT_KILL_RATIO_DIVIDEND	(3)
#define KGSL_CONTEXT_KILL_RATIO_DIVISOR		(4)
#define KGSL_CONTEXT_KILL_THRESHOLD(X)	\
	(X * \
		KGSL_CONTEXT_KILL_RATIO_DIVIDEND / \
		KGSL_CONTEXT_KILL_RATIO_DIVISOR)

struct kgsl_device;
struct idr;

struct kgsl_driver_htc_priv {
	struct work_struct work;
	unsigned long next_jiffies;
};

int kgsl_driver_htc_init(struct kgsl_driver_htc_priv *priv);

int kgsl_device_htc_init(struct kgsl_device *device);

void kgsl_dump_contextpid_locked(struct idr *context_idr);

void kgsl_check_context_id_locked(struct idr *context_idr, int total);

void adreno_fault_panic(struct kgsl_device *device, unsigned int pid, int fault);

#endif
