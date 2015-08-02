/*
 *  linux/kernel/printk.c
 *
 *  Copyright (C) 1991, 1992  Linus Torvalds
 *
 * Modified to make sys_syslog() more flexible: added commands to
 * return the last 4k of kernel messages, regardless of whether
 * they've been read or not.  Added option to suppress kernel printk's
 * to the console.  Added hook for sending the console messages
 * elsewhere, in preparation for a serial line console (someday).
 * Ted Ts'o, 2/11/93.
 * Modified for sysctl support, 1/8/97, Chris Horn.
 * Fixed SMP synchronization, 08/08/99, Manfred Spraul
 *     manfred@colorfullife.com
 * Rewrote bits to get rid of console_lock
 *	01Mar01 Andrew Morton
 */

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/console.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/nmi.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/interrupt.h>			
#include <linux/delay.h>
#include <linux/smp.h>
#include <linux/security.h>
#include <linux/bootmem.h>
#include <linux/memblock.h>
#include <linux/aio.h>
#include <linux/syscalls.h>
#include <linux/kexec.h>
#include <linux/kdb.h>
#include <linux/ratelimit.h>
#include <linux/kmsg_dump.h>
#include <linux/syslog.h>
#include <linux/cpu.h>
#include <linux/notifier.h>
#include <linux/rculist.h>
#include <linux/poll.h>
#include <linux/irq_work.h>
#include <linux/utsname.h>

#include <asm/uaccess.h>
#include <linux/htc_debug_tools.h>

#define CREATE_TRACE_POINTS
#include <trace/events/printk.h>

#ifdef CONFIG_EARLY_PRINTK_DIRECT
extern void printascii(char *);
#endif

#define DEFAULT_MESSAGE_LOGLEVEL CONFIG_DEFAULT_MESSAGE_LOGLEVEL

#define MINIMUM_CONSOLE_LOGLEVEL 1 
#define DEFAULT_CONSOLE_LOGLEVEL 7 

int console_printk[4] = {
	DEFAULT_CONSOLE_LOGLEVEL,	
	DEFAULT_MESSAGE_LOGLEVEL,	
	MINIMUM_CONSOLE_LOGLEVEL,	
	DEFAULT_CONSOLE_LOGLEVEL,	
};

int oops_in_progress;
EXPORT_SYMBOL(oops_in_progress);

static DEFINE_SEMAPHORE(console_sem);
struct console *console_drivers;
EXPORT_SYMBOL_GPL(console_drivers);

#ifdef CONFIG_LOCKDEP
static struct lockdep_map console_lock_dep_map = {
	.name = "console_lock"
};
#endif

static int console_locked, console_suspended;

static struct console *exclusive_console;

struct console_cmdline
{
	char	name[8];			
	int	index;				
	char	*options;			
#ifdef CONFIG_A11Y_BRAILLE_CONSOLE
	char	*brl_options;			
#endif
};

#define MAX_CMDLINECONSOLES 8

static struct console_cmdline console_cmdline[MAX_CMDLINECONSOLES];
static int selected_console = -1;
static int preferred_console = -1;
int console_set_on_cmdline;
EXPORT_SYMBOL(console_set_on_cmdline);

static int console_may_schedule;


enum log_flags {
	LOG_NOCONS	= 1,	
	LOG_NEWLINE	= 2,	
	LOG_PREFIX	= 4,	
	LOG_CONT	= 8,	
};

struct log {
	u64 ts_nsec;		
	u16 len;		
	u16 text_len;		
	u16 dict_len;		
	u8 facility;		
	u8 flags:5;		
	u8 level:3;		
#if defined(CONFIG_LOG_BUF_MAGIC)
	u32 magic;		
#endif
	u8 logbuf_cpu_id;	
	u32 logbuf_pid;		
};

static DEFINE_RAW_SPINLOCK(logbuf_lock);

#ifdef CONFIG_PRINTK
DECLARE_WAIT_QUEUE_HEAD(log_wait);
static u64 syslog_seq;
static u32 syslog_idx;
static enum log_flags syslog_prev;
static size_t syslog_partial;

static u64 log_first_seq;
static u32 log_first_idx;

static u64 log_next_seq;
static u32 log_next_idx;
static u32 log_last_valid_idx;

static u64 console_seq;
static u32 console_idx;
static enum log_flags console_prev;

static u64 clear_seq;
static u32 clear_idx;

#define PREFIX_MAX		32
#define LOG_LINE_MAX		1024 - PREFIX_MAX

#if defined(CONFIG_HAVE_EFFICIENT_UNALIGNED_ACCESS)
#define LOG_ALIGN 4
#else
#define LOG_ALIGN __alignof__(struct log)
#endif
#define __LOG_BUF_LEN (1 << CONFIG_LOG_BUF_SHIFT)
static char __log_buf[__LOG_BUF_LEN] __aligned(LOG_ALIGN);
static char *log_buf = __log_buf;
static u32 log_buf_len = __LOG_BUF_LEN;

#if defined(CONFIG_OOPS_LOG_BUFFER)
#define __OOPS_LOG_BUF_LEN (1 << CONFIG_OOPS_LOG_BUF_SHIFT)
static char __log_oops_buf[__OOPS_LOG_BUF_LEN] __aligned(LOG_ALIGN);
static char *log_oops_buf = __log_oops_buf;
static u32 log_oops_buf_len = __OOPS_LOG_BUF_LEN;

static int log_oops_full;
static u64 log_oops_first_seq = ULLONG_MAX;
static u64 log_oops_last_seq;
static u32 log_oops_next_idx;

static u32 syslog_oops_buf_idx;

static const char log_oops_end[] = "---end of oops log buffer---";
#endif

#if defined(CONFIG_LOG_BUF_MAGIC)
static u32 __log_align __used = LOG_ALIGN;
#define LOG_MAGIC(msg) ((msg)->magic = 0x5d7aefca)
#else
#define LOG_MAGIC(msg)
#endif

static volatile unsigned int logbuf_cpu = UINT_MAX;

static char *log_text(const struct log *msg)
{
	return (char *)msg + sizeof(struct log);
}

static char *log_dict(const struct log *msg)
{
	return (char *)msg + sizeof(struct log) + msg->text_len;
}

static struct log *log_from_idx(u32 idx, bool logbuf)
{
	struct log *msg;
	char *buf;

#if defined(CONFIG_OOPS_LOG_BUFFER)
	buf = logbuf ? log_buf : log_oops_buf;
#else
	buf = log_buf;
	BUG_ON(!logbuf);
#endif
	msg = (struct log *)(buf + idx);

	if (!msg->len)
		return (struct log *)buf;
	return msg;
}

static u32 log_next(u32 idx, bool logbuf)
{
	struct log *msg;
	char *buf;

#if defined(CONFIG_OOPS_LOG_BUFFER)
	buf = logbuf ? log_buf : log_oops_buf;
#else
	buf = log_buf;
	BUG_ON(!logbuf);
#endif
	msg = (struct log *)(buf + idx);

	
	if (!msg->len) {
		msg = (struct log *)buf;
		return msg->len;
	}
	return idx + msg->len;
}

#if defined(CONFIG_OOPS_LOG_BUFFER)
void oops_printk_start(void)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&logbuf_lock, flags);
	if (log_oops_first_seq == ULLONG_MAX)
		log_oops_first_seq = log_next_seq;
	raw_spin_unlock_irqrestore(&logbuf_lock, flags);
}

static void log_oops_store(struct log *msg)
{
	u32 free;
	const int eom_len = strlen(log_oops_end);
	const size_t eom_size = sizeof(struct log) + eom_len;
	char buf[eom_size + LOG_ALIGN];
	u32 pad_len;
	u64 ts_nsec;
	int eom = 0;

	if (log_first_seq >= log_oops_first_seq && !log_oops_full) {
		free = log_oops_buf_len - log_oops_next_idx;
		pad_len = (-eom_size) & (LOG_ALIGN - 1);
		if ((free - msg->len) < (eom_size + pad_len)) {
			ts_nsec = msg->ts_nsec;
			msg = (struct log *)buf;
			memcpy(log_text(msg), log_oops_end, eom_len);
			msg->len = eom_size + pad_len;
			msg->text_len = eom_len;
			msg->dict_len = 0;
			msg->facility = 1;
			msg->level = default_message_loglevel & 7;
			msg->flags = (LOG_NEWLINE | LOG_PREFIX) & 0x1f;
			msg->ts_nsec = ts_nsec;
			eom = 1;
		}

		if (free >= msg->len) {
			memcpy(log_oops_buf + log_oops_next_idx, msg, msg->len);
			log_oops_next_idx += msg->len;
			log_oops_last_seq = log_first_seq;
			if (eom)
				log_oops_full = 1;
		} else {
			log_oops_full = 1;
		}
	}
}
#else
static void log_oops_store(struct log *msg)
{
}
#endif

static void log_store(int facility, int level,
		      enum log_flags flags, u64 ts_nsec,
		      const char *dict, u16 dict_len,
		      const char *text, u16 text_len)
{
	struct log *msg;
	u32 size, pad_len;

	
	size = sizeof(struct log) + text_len + dict_len;
	pad_len = (-size) & (LOG_ALIGN - 1);
	size += pad_len;

	while (log_first_seq < log_next_seq) {
		u32 free;

		if (log_next_idx > log_first_idx)
			free = max(log_buf_len - log_next_idx, log_first_idx);
		else
			free = log_first_idx - log_next_idx;

		if (free > size + sizeof(struct log))
			break;

		msg = (struct log *)(log_buf + log_first_idx);
		log_oops_store(msg);

		
		log_first_idx = log_next(log_first_idx, true);
		log_first_seq++;
	}

	if (log_next_idx + size + sizeof(struct log) >= log_buf_len) {
		memset(log_buf + log_next_idx, 0, sizeof(struct log));
		LOG_MAGIC((struct log *)(log_buf + log_next_idx));
		log_next_idx = 0;
	}

	
	msg = (struct log *)(log_buf + log_next_idx);
	memcpy(log_text(msg), text, text_len);
	msg->text_len = text_len;
	memcpy(log_dict(msg), dict, dict_len);
	msg->dict_len = dict_len;
	msg->facility = facility;
	msg->level = level & 7;
	msg->flags = flags & 0x1f;
	LOG_MAGIC(msg);
	if (ts_nsec > 0)
		msg->ts_nsec = ts_nsec;
	else
		msg->ts_nsec = local_clock();
	memset(log_dict(msg) + dict_len, 0, pad_len);
	msg->len = sizeof(struct log) + text_len + dict_len + pad_len;

	
	log_last_valid_idx = log_next_idx;
	log_next_idx += msg->len;
	log_next_seq++;
}

#ifdef CONFIG_SECURITY_DMESG_RESTRICT
int dmesg_restrict = 1;
#else
int dmesg_restrict;
#endif

static int syslog_action_restricted(int type)
{
	if (dmesg_restrict)
		return 1;
	return type != SYSLOG_ACTION_READ_ALL &&
	       type != SYSLOG_ACTION_SIZE_BUFFER;
}

static int check_syslog_permissions(int type, bool from_file)
{
	if (from_file && type != SYSLOG_ACTION_OPEN)
		return 0;

	if (syslog_action_restricted(type)) {
		if (capable(CAP_SYSLOG))
			return 0;
		if (capable(CAP_SYS_ADMIN)) {
			pr_warn_once("%s (%d): Attempt to access syslog with "
				     "CAP_SYS_ADMIN but no CAP_SYSLOG "
				     "(deprecated).\n",
				 current->comm, task_pid_nr(current));
			return 0;
		}
		return -EPERM;
	}
	return security_syslog(type);
}


struct devkmsg_user {
	u64 seq;
	u32 idx;
	enum log_flags prev;
	struct mutex lock;
	char buf[8192];
};

static ssize_t devkmsg_writev(struct kiocb *iocb, const struct iovec *iv,
			      unsigned long count, loff_t pos)
{
	char *buf, *line;
	int i;
	int level = default_message_loglevel;
	int facility = 1;	
	size_t len = iov_length(iv, count);
	ssize_t ret = len;

	if (len > LOG_LINE_MAX)
		return -EINVAL;
	buf = kmalloc(len+1, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	line = buf;
	for (i = 0; i < count; i++) {
		if (copy_from_user(line, iv[i].iov_base, iv[i].iov_len)) {
			ret = -EFAULT;
			goto out;
		}
		line += iv[i].iov_len;
	}

	line = buf;
	if (line[0] == '<') {
		char *endp = NULL;

		i = simple_strtoul(line+1, &endp, 10);
		if (endp && endp[0] == '>') {
			level = i & 7;
			if (i >> 3)
				facility = i >> 3;
			endp++;
			len -= endp - line;
			line = endp;
		}
	}
	line[len] = '\0';

	printk_emit(facility, level, NULL, 0, "%s", line);
out:
	kfree(buf);
	return ret;
}

#if defined(CONFIG_OOPS_LOG_BUFFER)
static bool devkmsg_seq_passed(struct devkmsg_user *user)
{
	if ((log_oops_first_seq == ULLONG_MAX && user->seq < log_first_seq) ||
	    (log_oops_first_seq != ULLONG_MAX &&
	     user->seq < log_oops_first_seq))
		return true;
	else
		return false;
}
#else
static bool devkmsg_seq_passed(struct devkmsg_user *user)
{
	return user->seq < log_first_seq;
}
#endif

static ssize_t devkmsg_read(struct file *file, char __user *buf,
			    size_t count, loff_t *ppos)
{
	struct devkmsg_user *user = file->private_data;
	struct log *msg;
	u64 ts_usec;
	size_t i;
	char cont = '-';
	size_t len;
	ssize_t ret;
	bool regular_buf = true;

	if (!user)
		return -EBADF;

	ret = mutex_lock_interruptible(&user->lock);
	if (ret)
		return ret;
	raw_spin_lock_irq(&logbuf_lock);
	while (user->seq == log_next_seq) {
		if (file->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			raw_spin_unlock_irq(&logbuf_lock);
			goto out;
		}

		raw_spin_unlock_irq(&logbuf_lock);
		ret = wait_event_interruptible(log_wait,
					       user->seq != log_next_seq);
		if (ret)
			goto out;
		raw_spin_lock_irq(&logbuf_lock);
	}

	if (devkmsg_seq_passed(user)) {
		
		user->idx = log_first_idx;
		user->seq = log_first_seq;
		ret = -EPIPE;
		raw_spin_unlock_irq(&logbuf_lock);
		goto out;
	}
#if defined(CONFIG_OOPS_LOG_BUFFER)
	else if (log_oops_first_seq != ULLONG_MAX) {
		if (user->seq <= log_oops_first_seq) {
			user->idx = 0;
			regular_buf = false;
		} else if (user->seq > log_oops_first_seq &&
			 user->seq < log_oops_last_seq) {
			regular_buf = false;
		} else if (user->seq < log_first_seq) {
			user->idx = log_first_idx;
			user->seq = log_first_seq;
		}
	}
#endif

	msg = log_from_idx(user->idx, regular_buf);
	ts_usec = msg->ts_nsec;
	do_div(ts_usec, 1000);

	if (msg->flags & LOG_CONT && !(user->prev & LOG_CONT))
		cont = 'c';
	else if ((msg->flags & LOG_CONT) ||
		 ((user->prev & LOG_CONT) && !(msg->flags & LOG_PREFIX)))
		cont = '+';

	len = sprintf(user->buf, "%u,%llu,%llu,%c;",
		      (msg->facility << 3) | msg->level,
		      user->seq, ts_usec, cont);
	user->prev = msg->flags;

	
	for (i = 0; i < msg->text_len; i++) {
		unsigned char c = log_text(msg)[i];

		if (c < ' ' || c >= 127 || c == '\\')
			len += sprintf(user->buf + len, "\\x%02x", c);
		else
			user->buf[len++] = c;
	}
	user->buf[len++] = '\n';

	if (msg->dict_len) {
		bool line = true;

		for (i = 0; i < msg->dict_len; i++) {
			unsigned char c = log_dict(msg)[i];

			if (line) {
				user->buf[len++] = ' ';
				line = false;
			}

			if (c == '\0') {
				user->buf[len++] = '\n';
				line = true;
				continue;
			}

			if (c < ' ' || c >= 127 || c == '\\') {
				len += sprintf(user->buf + len, "\\x%02x", c);
				continue;
			}

			user->buf[len++] = c;
		}
		user->buf[len++] = '\n';
	}

	user->idx = log_next(user->idx, regular_buf);
	user->seq++;
	raw_spin_unlock_irq(&logbuf_lock);

	if (len > count) {
		ret = -EINVAL;
		goto out;
	}

	if (copy_to_user(buf, user->buf, len)) {
		ret = -EFAULT;
		goto out;
	}
	ret = len;
out:
	mutex_unlock(&user->lock);
	return ret;
}

static void devkmsg_set_first(struct devkmsg_user *user)
{
#if defined(CONFIG_OOPS_LOG_BUFFER)
	if (log_oops_first_seq != ULLONG_MAX) {
		user->idx = 0;
		user->seq = log_oops_first_seq;
	} else
#endif
	{
		user->idx = log_first_idx;
		user->seq = log_first_seq;
	}
}

static loff_t devkmsg_llseek(struct file *file, loff_t offset, int whence)
{
	struct devkmsg_user *user = file->private_data;
	loff_t ret = 0;

	if (!user)
		return -EBADF;
	if (offset)
		return -ESPIPE;

	raw_spin_lock_irq(&logbuf_lock);
	switch (whence) {
	case SEEK_SET:
		
		devkmsg_set_first(user);
		break;
	case SEEK_DATA:
		user->idx = clear_idx;
		user->seq = clear_seq;
		break;
	case SEEK_END:
		
		user->idx = log_next_idx;
		user->seq = log_next_seq;
		break;
	default:
		ret = -EINVAL;
	}
	raw_spin_unlock_irq(&logbuf_lock);
	return ret;
}

static unsigned int devkmsg_poll(struct file *file, poll_table *wait)
{
	struct devkmsg_user *user = file->private_data;
	int ret = 0;

	if (!user)
		return POLLERR|POLLNVAL;

	poll_wait(file, &log_wait, wait);

	raw_spin_lock_irq(&logbuf_lock);
	if (user->seq < log_next_seq) {
		
		if (user->seq < log_first_seq)
			ret = POLLIN|POLLRDNORM|POLLERR|POLLPRI;
		else
			ret = POLLIN|POLLRDNORM;
	}
	raw_spin_unlock_irq(&logbuf_lock);

	return ret;
}

static int devkmsg_open(struct inode *inode, struct file *file)
{
	struct devkmsg_user *user;
	int err;

	
	if ((file->f_flags & O_ACCMODE) == O_WRONLY)
		return 0;

	err = check_syslog_permissions(SYSLOG_ACTION_READ_ALL,
				       SYSLOG_FROM_READER);
	if (err)
		return err;

	user = kmalloc(sizeof(struct devkmsg_user), GFP_KERNEL);
	if (!user)
		return -ENOMEM;

	mutex_init(&user->lock);

	raw_spin_lock_irq(&logbuf_lock);
	devkmsg_set_first(user);
	raw_spin_unlock_irq(&logbuf_lock);

	file->private_data = user;
	return 0;
}

static int devkmsg_release(struct inode *inode, struct file *file)
{
	struct devkmsg_user *user = file->private_data;

	if (!user)
		return 0;

	mutex_destroy(&user->lock);
	kfree(user);
	return 0;
}

const struct file_operations kmsg_fops = {
	.open = devkmsg_open,
	.read = devkmsg_read,
	.aio_write = devkmsg_writev,
	.llseek = devkmsg_llseek,
	.poll = devkmsg_poll,
	.release = devkmsg_release,
};

#ifdef CONFIG_KEXEC
void log_buf_kexec_setup(void)
{
	VMCOREINFO_SYMBOL(log_buf);
	VMCOREINFO_SYMBOL(log_buf_len);
	VMCOREINFO_SYMBOL(log_first_idx);
	VMCOREINFO_SYMBOL(log_next_idx);
	VMCOREINFO_STRUCT_SIZE(log);
	VMCOREINFO_OFFSET(log, ts_nsec);
	VMCOREINFO_OFFSET(log, len);
	VMCOREINFO_OFFSET(log, text_len);
	VMCOREINFO_OFFSET(log, dict_len);
}
#endif

static unsigned long __initdata new_log_buf_len;

static int __init log_buf_len_setup(char *str)
{
	unsigned size = memparse(str, &str);

	if (size)
		size = roundup_pow_of_two(size);
	if (size > log_buf_len)
		new_log_buf_len = size;

	return 0;
}
early_param("log_buf_len", log_buf_len_setup);

void __init setup_log_buf(int early)
{
	unsigned long flags;
	char *new_log_buf;
	int free;

	if (!new_log_buf_len)
		return;

	if (early) {
		unsigned long mem;

		mem = memblock_alloc(new_log_buf_len, PAGE_SIZE);
		if (!mem)
			return;
		new_log_buf = __va(mem);
	} else {
		new_log_buf = alloc_bootmem_nopanic(new_log_buf_len);
	}

	if (unlikely(!new_log_buf)) {
		pr_err("log_buf_len: %ld bytes not available\n",
			new_log_buf_len);
		return;
	}

	raw_spin_lock_irqsave(&logbuf_lock, flags);
	log_buf_len = new_log_buf_len;
	log_buf = new_log_buf;
	new_log_buf_len = 0;
	free = __LOG_BUF_LEN - log_next_idx;
	memcpy(log_buf, __log_buf, __LOG_BUF_LEN);
	raw_spin_unlock_irqrestore(&logbuf_lock, flags);

	pr_info("log_buf_len: %d\n", log_buf_len);
	pr_info("early log buf free: %d(%d%%)\n",
		free, (free * 100) / __LOG_BUF_LEN);
}

static bool __read_mostly ignore_loglevel;

static int __init ignore_loglevel_setup(char *str)
{
	ignore_loglevel = 1;
	printk(KERN_INFO "debug: ignoring loglevel setting.\n");

	return 0;
}

early_param("ignore_loglevel", ignore_loglevel_setup);
module_param(ignore_loglevel, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(ignore_loglevel, "ignore loglevel setting, to"
	"print all kernel messages to the console.");

#ifdef CONFIG_BOOT_PRINTK_DELAY

static int boot_delay; 
static unsigned long long loops_per_msec;	

static int __init boot_delay_setup(char *str)
{
	unsigned long lpj;

	lpj = preset_lpj ? preset_lpj : 1000000;	
	loops_per_msec = (unsigned long long)lpj / 1000 * HZ;

	get_option(&str, &boot_delay);
	if (boot_delay > 10 * 1000)
		boot_delay = 0;

	pr_debug("boot_delay: %u, preset_lpj: %ld, lpj: %lu, "
		"HZ: %d, loops_per_msec: %llu\n",
		boot_delay, preset_lpj, lpj, HZ, loops_per_msec);
	return 1;
}
__setup("boot_delay=", boot_delay_setup);

static void boot_delay_msec(int level)
{
	unsigned long long k;
	unsigned long timeout;

	if ((boot_delay == 0 || system_state != SYSTEM_BOOTING)
		|| (level >= console_loglevel && !ignore_loglevel)) {
		return;
	}

	k = (unsigned long long)loops_per_msec * boot_delay;

	timeout = jiffies + msecs_to_jiffies(boot_delay);
	while (k) {
		k--;
		cpu_relax();
		if (time_after(jiffies, timeout))
			break;
		touch_nmi_watchdog();
	}
}
#else
static inline void boot_delay_msec(int level)
{
}
#endif

#if defined(CONFIG_PRINTK_TIME)
static bool printk_time = 1;
#else
static bool printk_time;
#endif
module_param_named(time, printk_time, bool, S_IRUGO | S_IWUSR);

#if defined(CONFIG_PRINTK_CPU_ID)
static int printk_cpu_id = 1;
#else
static int printk_cpu_id = 0;
#endif
module_param_named(cpu, printk_cpu_id, int, S_IRUGO | S_IWUSR);

#if defined(CONFIG_PRINTK_PID)
static int printk_pid = 1;
#else
static int printk_pid = 0;
#endif
module_param_named(pid, printk_pid, int, S_IRUGO | S_IWUSR);

static void log_store_other(u8 cpu_id, u32 pid) {
	if (printk_cpu_id) {
		struct log *this_log;
		this_log = (struct log *)(log_buf + log_last_valid_idx);
		this_log->logbuf_cpu_id = cpu_id;
	}
	if (printk_pid) {
		struct log *this_log;
		this_log = (struct log *)(log_buf + log_last_valid_idx);
		this_log->logbuf_pid = pid;
	}
}

static size_t print_time(u64 ts, char *buf)
{
	unsigned long rem_nsec;

	if (!printk_time)
		return 0;

	rem_nsec = do_div(ts, 1000000000);

	if (!buf)
		return snprintf(NULL, 0, "[%5lu.000000] ", (unsigned long)ts);

	return sprintf(buf, "[%5lu.%06lu] ",
		       (unsigned long)ts, rem_nsec / 1000);
}

static size_t print_cpu_id(u8 cpu_id, char *buf)
{
	if (!printk_cpu_id)
		return 0;

	if (!buf)
		return snprintf(NULL, 0, "c%u ", cpu_id);

	return sprintf(buf, "c%u ", cpu_id);
}

static size_t print_pid(u32 pid, char *buf)
{
	if (!printk_pid)
		return 0;

	if (!buf)
		return snprintf(NULL, 0, "%6u ", pid);

	return sprintf(buf, "%6u ", pid);
}

static size_t print_prefix(const struct log *msg, bool syslog, char *buf)
{
	size_t len = 0;
	unsigned int prefix = (msg->facility << 3) | msg->level;

	if (syslog) {
		if (buf) {
			len += sprintf(buf, "<%u>", prefix);
		} else {
			len += 3;
			if (prefix > 999)
				len += 3;
			else if (prefix > 99)
				len += 2;
			else if (prefix > 9)
				len++;
		}
	}

	len += print_time(msg->ts_nsec, buf ? buf + len : NULL);
	len += print_cpu_id(msg->logbuf_cpu_id, buf ? buf + len : NULL);
	len += print_pid(msg->logbuf_pid, buf ? buf + len : NULL);
	return len;
}

static size_t msg_print_text(const struct log *msg, enum log_flags prev,
			     bool syslog, char *buf, size_t size)
{
	const char *text = log_text(msg);
	size_t text_size = msg->text_len;
	bool prefix = true;
	bool newline = true;
	size_t len = 0;

	if ((prev & LOG_CONT) && !(msg->flags & LOG_PREFIX))
		prefix = false;

	if (msg->flags & LOG_CONT) {
		if ((prev & LOG_CONT) && !(prev & LOG_NEWLINE))
			prefix = false;

		if (!(msg->flags & LOG_NEWLINE))
			newline = false;
	}

	do {
		const char *next = memchr(text, '\n', text_size);
		size_t text_len;

		if (next) {
			text_len = next - text;
			next++;
			text_size -= next - text;
		} else {
			text_len = text_size;
		}

		if (buf) {
			if (print_prefix(msg, syslog, NULL) +
			    text_len + 1 >= size - len)
				break;

			if (prefix)
				len += print_prefix(msg, syslog, buf + len);
			memcpy(buf + len, text, text_len);
			len += text_len;
			if (next || newline)
				buf[len++] = '\n';
		} else {
			
			if (prefix)
				len += print_prefix(msg, syslog, NULL);
			len += text_len;
			if (next || newline)
				len++;
		}

		prefix = true;
		text = next;
	} while (text);

	return len;
}

#if defined(CONFIG_OOPS_LOG_BUFFER)
static int syslog_oops_buf_print(char __user *buf, int size, char *text)
{
	struct log *msg;
	size_t n;
	size_t skip;
	int len = 0;

	raw_spin_lock_irq(&logbuf_lock);
	if (log_oops_first_seq != ULLONG_MAX &&
	    syslog_seq < log_oops_first_seq) {
		syslog_seq = log_oops_first_seq;
		syslog_oops_buf_idx = 0;
	}
	while (size > 0 && log_oops_last_seq > syslog_seq) {
		skip = syslog_partial;
		msg = log_from_idx(syslog_oops_buf_idx, false);
		n = msg_print_text(msg, syslog_prev, true, text,
				   LOG_LINE_MAX + PREFIX_MAX);
		if (n - syslog_partial <= size) {
			
			syslog_oops_buf_idx = log_next(syslog_oops_buf_idx,
						       false);
			syslog_seq++;
			syslog_prev = msg->flags;
			n -= syslog_partial;
			syslog_partial = 0;
		} else if (!len) {
			
			n = size;
			syslog_partial += n;
		} else {
			n = 0;
		}
		if (!n)
			break;

		raw_spin_unlock_irq(&logbuf_lock);
		if (copy_to_user(buf, text + skip, n)) {
			raw_spin_lock_irq(&logbuf_lock);
			if (!len)
				len = -EFAULT;
			break;
		}
		raw_spin_lock_irq(&logbuf_lock);

		len += n;
		size -= n;
		buf += n;
	}
	raw_spin_unlock_irq(&logbuf_lock);

	return len;
}

static int syslog_print_oops_buf_all(char __user *buf, int size, bool clear,
				     char *text)
{
	int len = 0;
	u32 idx = 0;
	u64 seq = clear_seq;
	enum log_flags prev = 0;
	u64 next_seq;

	if (!buf)
		return len;

	raw_spin_lock_irq(&logbuf_lock);

	seq = log_oops_first_seq;
	next_seq = log_oops_last_seq;
	while (len >= 0 && len < size && seq < next_seq) {
		struct log *msg = log_from_idx(idx, false);
		int textlen;

		textlen = msg_print_text(msg, prev, true, text,
					 LOG_LINE_MAX + PREFIX_MAX);
		if (textlen < 0) {
			len = textlen;
			break;
		}
		idx = log_next(idx, false);
		seq++;
		prev = msg->flags;

		raw_spin_unlock_irq(&logbuf_lock);
		if (copy_to_user(buf + len, text, textlen))
			len = -EFAULT;
		else
			len += textlen;
		raw_spin_lock_irq(&logbuf_lock);
	}

	raw_spin_unlock_irq(&logbuf_lock);

	return len;
}
#else
static int syslog_oops_buf_print(char __user *buf, int size, char *text)
{
	return 0;
}

static int syslog_print_oops_buf_all(char __user *buf, int size, bool clear,
				     char *text)
{
	return 0;
}
#endif

int syslog_print(char __user *buf, int size)
{
	char *text;
	struct log *msg;
	int oops_buf_len;
	int len = 0;

	text = kmalloc(LOG_LINE_MAX + PREFIX_MAX, GFP_KERNEL);
	if (!text)
		return -ENOMEM;

	oops_buf_len = syslog_oops_buf_print(buf, size, text);
	if (oops_buf_len < 0)
		return oops_buf_len;

	size -= oops_buf_len;

	while (size > 0) {
		size_t n;
		size_t skip;

		raw_spin_lock_irq(&logbuf_lock);
		if (syslog_seq < log_first_seq) {
			
			syslog_seq = log_first_seq;
			syslog_idx = log_first_idx;
			syslog_prev = 0;
			syslog_partial = 0;
		}
		if (syslog_seq == log_next_seq) {
			raw_spin_unlock_irq(&logbuf_lock);
			break;
		}

		skip = syslog_partial;
		msg = log_from_idx(syslog_idx, true);
		n = msg_print_text(msg, syslog_prev, true, text,
				   LOG_LINE_MAX + PREFIX_MAX);
		if (n - syslog_partial <= size) {
			
			syslog_idx = log_next(syslog_idx, true);
			syslog_seq++;
			syslog_prev = msg->flags;
			n -= syslog_partial;
			syslog_partial = 0;
		} else if (!len){
			
			n = size;
			syslog_partial += n;
		} else
			n = 0;
		raw_spin_unlock_irq(&logbuf_lock);

		if (!n)
			break;

		if (copy_to_user(buf + oops_buf_len, text + skip, n)) {
			if (!len)
				len = -EFAULT;
			break;
		}

		len += n;
		size -= n;
		buf += n;
	}

	kfree(text);
	if (len > 0)
		len += oops_buf_len;
	return len;
}

static int syslog_print_all(char __user *buf, int size, bool clear)
{
	char *text;
	int oops_len;
	int len = 0;

	text = kmalloc(LOG_LINE_MAX + PREFIX_MAX, GFP_KERNEL);
	if (!text)
		return -ENOMEM;

	oops_len = syslog_print_oops_buf_all(buf, size, clear, text);
	if (oops_len < 0)
		return oops_len;

	raw_spin_lock_irq(&logbuf_lock);
	if (buf) {
		u64 next_seq;
		u64 seq;
		u32 idx;
		u64 start_seq;
		u32 start_idx;
		enum log_flags prev;

		if (clear_seq < log_first_seq) {
			
			start_seq = log_first_seq;
			start_idx = log_first_idx;
		} else {
			start_seq = clear_seq;
			start_idx = clear_idx;
		}

		seq = start_seq;
		idx = start_idx;
		prev = 0;
		while (seq < log_next_seq) {
			struct log *msg = log_from_idx(idx, true);

			len += msg_print_text(msg, prev, true, NULL, 0);
			prev = msg->flags;
			idx = log_next(idx, true);
			seq++;
		}

		
		seq = start_seq;
		idx = start_idx;
		prev = 0;
		while ((len > size - oops_len) && seq < log_next_seq) {
			struct log *msg = log_from_idx(idx, true);

			len -= msg_print_text(msg, prev, true, NULL, 0);
			prev = msg->flags;
			idx = log_next(idx, true);
			seq++;
		}

		
		next_seq = log_next_seq;

		len = 0;
		prev = 0;
		while (len >= 0 && seq < next_seq) {
			struct log *msg = log_from_idx(idx, true);
			int textlen;

			textlen = msg_print_text(msg, prev, true, text,
						 LOG_LINE_MAX + PREFIX_MAX);
			if (textlen < 0) {
				len = textlen;
				break;
			}
			idx = log_next(idx, true);
			seq++;
			prev = msg->flags;

			raw_spin_unlock_irq(&logbuf_lock);
			if (copy_to_user(buf + len + oops_len, text, textlen))
				len = -EFAULT;
			else
				len += textlen;
			raw_spin_lock_irq(&logbuf_lock);

			if (seq < log_first_seq) {
				
				seq = log_first_seq;
				idx = log_first_idx;
				prev = 0;
			}
		}
	}

	if (clear) {
		clear_seq = log_next_seq;
		clear_idx = log_next_idx;
	}
	raw_spin_unlock_irq(&logbuf_lock);

	kfree(text);
	if (len > 0)
		len += oops_len;
	return len;
}

#if defined(CONFIG_HTC_DEBUG_BOOTLOADER_LOG)
static inline int insert_to_buf_ln(char __user *buf, int buf_len, char* str)
{
	int len = 0;
	len = strlen(str)+1;
	if (buf_len >= len) {
		memcpy(buf, str, len);
		buf[len-1] = '\n';
		return len;
	}
	return 0;
}
#endif

int do_syslog(int type, char __user *buf, int len, bool from_file)
{
	bool clear = false;
	static int saved_console_loglevel = -1;
	int error;
#if defined(CONFIG_HTC_DEBUG_BOOTLOADER_LOG)
	ssize_t lk_len = 0, lk_len_total = 0;
#define HB_LAST_TITLE "[HB LAST]"
#define HB_LOG_TITLE "[HB LOG]"
#endif

	error = check_syslog_permissions(type, from_file);
	if (error)
		goto out;

	error = security_syslog(type);
	if (error)
		return error;

	switch (type) {
	case SYSLOG_ACTION_CLOSE:	
		break;
	case SYSLOG_ACTION_OPEN:	
		break;
	case SYSLOG_ACTION_READ:	
		error = -EINVAL;
		if (!buf || len < 0)
			goto out;
		error = 0;
		if (!len)
			goto out;
		if (!access_ok(VERIFY_WRITE, buf, len)) {
			error = -EFAULT;
			goto out;
		}
		error = wait_event_interruptible(log_wait,
						 syslog_seq != log_next_seq);
		if (error)
			goto out;
		error = syslog_print(buf, len);
		break;
	
	case SYSLOG_ACTION_READ_CLEAR:
		clear = true;
		
	
	case SYSLOG_ACTION_READ_ALL:
		error = -EINVAL;
		if (!buf || len < 0)
			goto out;
		error = 0;
		if (!len)
			goto out;
		if (!access_ok(VERIFY_WRITE, buf, len)) {
			error = -EFAULT;
			goto out;
		}
		error = syslog_print_all(buf, len, clear);
		break;
#if defined(CONFIG_HTC_DEBUG_BOOTLOADER_LOG)
	
	case SYSLOG_ACTION_READ_ALL_APPEND_LK:
		error = -EINVAL;
		if (!buf || len < 0)
			goto out;
		error = 0;
		if (!len)
			goto out;
		if (!access_ok(VERIFY_WRITE, buf, len)) {
			error = -EFAULT;
			goto out;
		}

		lk_len = insert_to_buf_ln(buf, len, HB_LAST_TITLE);
		len -= lk_len;
		buf += lk_len;
		lk_len_total += lk_len;

		lk_len = bldr_last_log_read_once(buf, len);
		len -= lk_len;
		buf += lk_len;
		lk_len_total += lk_len;

		lk_len = insert_to_buf_ln(buf, len, HB_LOG_TITLE);
		len -= lk_len;
		buf += lk_len;
		lk_len_total += lk_len;

		lk_len = bldr_log_read_once(buf, len);
		len -= lk_len;
		buf += lk_len;
		lk_len_total += lk_len;

		error = syslog_print_all(buf, len, clear);
		error += lk_len_total;
		break;
#endif
	
	case SYSLOG_ACTION_CLEAR:
		syslog_print_all(NULL, 0, true);
		break;
	
	case SYSLOG_ACTION_CONSOLE_OFF:
		if (saved_console_loglevel == -1)
			saved_console_loglevel = console_loglevel;
		console_loglevel = minimum_console_loglevel;
		break;
	
	case SYSLOG_ACTION_CONSOLE_ON:
		if (saved_console_loglevel != -1) {
			console_loglevel = saved_console_loglevel;
			saved_console_loglevel = -1;
		}
		break;
	
	case SYSLOG_ACTION_CONSOLE_LEVEL:
		error = -EINVAL;
		if (len < 1 || len > 8)
			goto out;
		if (len < minimum_console_loglevel)
			len = minimum_console_loglevel;
		console_loglevel = len;
		
		saved_console_loglevel = -1;
		error = 0;
		break;
	
	case SYSLOG_ACTION_SIZE_UNREAD:
		raw_spin_lock_irq(&logbuf_lock);
		if (syslog_seq < log_first_seq) {
			
			syslog_seq = log_first_seq;
			syslog_idx = log_first_idx;
			syslog_prev = 0;
			syslog_partial = 0;
		}
		if (from_file) {
			error = log_next_idx - syslog_idx;
		} else {
			u64 seq = syslog_seq;
			u32 idx = syslog_idx;
			enum log_flags prev = syslog_prev;

			error = 0;
			while (seq < log_next_seq) {
				struct log *msg = log_from_idx(idx,
								      true);

				error += msg_print_text(msg, prev, true, NULL, 0);
				idx = log_next(idx, true);
				seq++;
				prev = msg->flags;
			}
			error -= syslog_partial;
		}
		raw_spin_unlock_irq(&logbuf_lock);
		break;
	
	case SYSLOG_ACTION_SIZE_BUFFER:
		error = log_buf_len;
#if defined(CONFIG_OOPS_LOG_BUFFER)
		error += log_oops_buf_len;
#endif
		break;
	default:
		error = -EINVAL;
		break;
	}
out:
	return error;
}

SYSCALL_DEFINE3(syslog, int, type, char __user *, buf, int, len)
{
	return do_syslog(type, buf, len, SYSLOG_FROM_READER);
}

static void call_console_drivers(int level, const char *text, size_t len)
{
	struct console *con;

	trace_console(text, len);

	if (level >= console_loglevel && !ignore_loglevel)
		return;
	if (!console_drivers)
		return;

	for_each_console(con) {
		if (exclusive_console && con != exclusive_console)
			continue;
		if (!(con->flags & CON_ENABLED))
			continue;
		if (!con->write)
			continue;
		if (!cpu_online(smp_processor_id()) &&
		    !(con->flags & CON_ANYTIME))
			continue;
		con->write(con, text, len);
	}
}

static void zap_locks(void)
{
	static unsigned long oops_timestamp;

	if (time_after_eq(jiffies, oops_timestamp) &&
			!time_after(jiffies, oops_timestamp + 30 * HZ))
		return;

	oops_timestamp = jiffies;

	debug_locks_off();
	
	raw_spin_lock_init(&logbuf_lock);
	
	sema_init(&console_sem, 1);
}

static int have_callable_console(void)
{
	struct console *con;

	for_each_console(con)
		if (con->flags & CON_ANYTIME)
			return 1;

	return 0;
}

static inline int can_use_console(unsigned int cpu)
{
	return cpu_online(cpu) || have_callable_console();
}

static int console_trylock_for_printk(unsigned int cpu)
	__releases(&logbuf_lock)
{
	int retval = 0, wake = 0;

	if (console_trylock()) {
		retval = 1;

		if (!can_use_console(cpu)) {
			console_locked = 0;
			wake = 1;
			retval = 0;
		}
	}
	logbuf_cpu = UINT_MAX;
	raw_spin_unlock(&logbuf_lock);
	if (wake)
		up(&console_sem);
	return retval;
}

int printk_delay_msec __read_mostly;

static inline void printk_delay(void)
{
	if (unlikely(printk_delay_msec)) {
		int m = printk_delay_msec;

		while (m--) {
			mdelay(1);
			touch_nmi_watchdog();
		}
	}
}

static struct cont {
	char buf[LOG_LINE_MAX];
	size_t len;			
	size_t cons;			/* bytes written to console */
	struct task_struct *owner;	
	u64 ts_nsec;			
	u8 level;			
	u8 facility;			
	enum log_flags flags;		
	bool flushed:1;			
	u8 cont_cpu_id;			
	u32 cont_pid;			
} cont;

static void cont_flush(enum log_flags flags)
{
	if (cont.flushed)
		return;
	if (cont.len == 0)
		return;

	if (cont.cons) {
		log_store(cont.facility, cont.level, flags | LOG_NOCONS,
			  cont.ts_nsec, NULL, 0, cont.buf, cont.len);
		log_store_other(cont.cont_cpu_id, cont.cont_pid);
		cont.flags = flags;
		cont.flushed = true;
	} else {
		log_store(cont.facility, cont.level, flags, 0,
			  NULL, 0, cont.buf, cont.len);
		log_store_other(cont.cont_cpu_id, cont.cont_pid);
		cont.len = 0;
	}
}

static bool cont_add(int facility, int level, const char *text, size_t len)
{
	if (cont.len && cont.flushed)
		return false;

	if (cont.len + len > sizeof(cont.buf)) {
		
		cont_flush(LOG_CONT);
		return false;
	}

	if (!cont.len) {
		cont.facility = facility;
		cont.level = level;
		cont.owner = current;
		cont.ts_nsec = local_clock();
		cont.flags = 0;
		cont.cons = 0;
		cont.flushed = false;
		cont.cont_cpu_id = smp_processor_id();
		cont.cont_pid = current->pid;
	}

	memcpy(cont.buf + cont.len, text, len);
	cont.len += len;

	if (cont.len > (sizeof(cont.buf) * 80) / 100)
		cont_flush(LOG_CONT);

	return true;
}

static size_t cont_print_text(char *text, size_t size)
{
	size_t textlen = 0;
	size_t len;

	if (cont.cons == 0 && (console_prev & LOG_NEWLINE)) {
		textlen += print_time(cont.ts_nsec, text);
		size -= textlen;
	}

	len = cont.len - cont.cons;
	if (len > 0) {
		if (len+1 > size)
			len = size-1;
		memcpy(text + textlen, cont.buf + cont.cons, len);
		textlen += len;
		cont.cons = cont.len;
	}

	if (cont.flushed) {
		if (cont.flags & LOG_NEWLINE)
			text[textlen++] = '\n';
		
		cont.len = 0;
	}
	return textlen;
}

asmlinkage int vprintk_emit(int facility, int level,
			    const char *dict, size_t dictlen,
			    const char *fmt, va_list args)
{
	static int recursion_bug;
	static char textbuf[LOG_LINE_MAX];
	char *text = textbuf;
	size_t text_len;
	enum log_flags lflags = 0;
	unsigned long flags;
	int this_cpu;
	int printed_len = 0;

	boot_delay_msec(level);
	printk_delay();

	
	local_irq_save(flags);
	this_cpu = smp_processor_id();

	if (unlikely(logbuf_cpu == this_cpu)) {
		if (!oops_in_progress && !lockdep_recursing(current)) {
			recursion_bug = 1;
			goto out_restore_irqs;
		}
		zap_locks();
	}

	lockdep_off();
	raw_spin_lock(&logbuf_lock);
	logbuf_cpu = this_cpu;

	if (recursion_bug) {
		static const char recursion_msg[] =
			"BUG: recent printk recursion!";

		recursion_bug = 0;
		printed_len += strlen(recursion_msg);
		
		log_store(0, 2, LOG_PREFIX|LOG_NEWLINE, 0,
			  NULL, 0, recursion_msg, printed_len);
	}

	text_len = vscnprintf(text, sizeof(textbuf), fmt, args);

	
	if (text_len && text[text_len-1] == '\n') {
		text_len--;
		lflags |= LOG_NEWLINE;
	}

	
	if (facility == 0) {
		int kern_level = printk_get_level(text);

		if (kern_level) {
			const char *end_of_header = printk_skip_level(text);
			switch (kern_level) {
			case '0' ... '7':
				if (level == -1)
					level = kern_level - '0';
			case 'd':	
				lflags |= LOG_PREFIX;
			case 'c':	
				break;
			}
			text_len -= end_of_header - text;
			text = (char *)end_of_header;
		}
	}

#ifdef CONFIG_EARLY_PRINTK_DIRECT
	printascii(text);
#endif

	if (level == -1)
		level = default_message_loglevel;

	if (dict)
		lflags |= LOG_PREFIX|LOG_NEWLINE;

	if (!(lflags & LOG_NEWLINE)) {
		if (cont.len && (lflags & LOG_PREFIX || cont.owner != current))
			cont_flush(LOG_NEWLINE);

		
		if (!cont_add(facility, level, text, text_len)) {
			log_store(facility, level, lflags | LOG_CONT, 0,
				  dict, dictlen, text, text_len);
			log_store_other(logbuf_cpu, current->pid);
		}
	} else {
		bool stored = false;

		if (cont.len && cont.owner == current) {
			if (!(lflags & LOG_PREFIX))
				stored = cont_add(facility, level, text, text_len);
			cont_flush(LOG_NEWLINE);
		}

		if (!stored) {
			log_store(facility, level, lflags, 0,
				  dict, dictlen, text, text_len);
			log_store_other(logbuf_cpu, current->pid);
		}
	}
	printed_len += text_len;

	if (console_trylock_for_printk(this_cpu))
		console_unlock();

	lockdep_on();
out_restore_irqs:
	local_irq_restore(flags);

	return printed_len;
}
EXPORT_SYMBOL(vprintk_emit);

asmlinkage int vprintk(const char *fmt, va_list args)
{
	return vprintk_emit(0, -1, NULL, 0, fmt, args);
}
EXPORT_SYMBOL(vprintk);

asmlinkage int printk_emit(int facility, int level,
			   const char *dict, size_t dictlen,
			   const char *fmt, ...)
{
	va_list args;
	int r;

	va_start(args, fmt);
	r = vprintk_emit(facility, level, dict, dictlen, fmt, args);
	va_end(args);

	return r;
}
EXPORT_SYMBOL(printk_emit);

asmlinkage int printk(const char *fmt, ...)
{
	va_list args;
	int r;

#ifdef CONFIG_KGDB_KDB
	if (unlikely(kdb_trap_printk)) {
		va_start(args, fmt);
		r = vkdb_printf(fmt, args);
		va_end(args);
		return r;
	}
#endif
	va_start(args, fmt);
	r = vprintk_emit(0, -1, NULL, 0, fmt, args);
	va_end(args);

	return r;
}
EXPORT_SYMBOL(printk);

#else 

#define LOG_LINE_MAX		0
#define PREFIX_MAX		0
#define LOG_LINE_MAX 0
static u64 syslog_seq;
static u32 syslog_idx;
static u64 console_seq;
static u32 console_idx;
static enum log_flags syslog_prev;
static u64 log_first_seq;
static u32 log_first_idx;
static u64 log_next_seq;
static enum log_flags console_prev;
static struct cont {
	size_t len;
	size_t cons;
	u8 level;
	bool flushed:1;
} cont;
static struct log *log_from_idx(u32 idx, bool logbuf) { return NULL; }
static u32 log_next(u32 idx, bool logbuf) { return 0; }
static void call_console_drivers(int level, const char *text, size_t len) {}
static size_t msg_print_text(const struct log *msg, enum log_flags prev,
			     bool syslog, char *buf, size_t size) { return 0; }
static size_t cont_print_text(char *text, size_t size) { return 0; }

#endif 

#ifdef CONFIG_EARLY_PRINTK
struct console *early_console;

void early_vprintk(const char *fmt, va_list ap)
{
	if (early_console) {
		char buf[512];
		int n = vscnprintf(buf, sizeof(buf), fmt, ap);

		early_console->write(early_console, buf, n);
	}
}

asmlinkage void early_printk(const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	early_vprintk(fmt, ap);
	va_end(ap);
}
#endif

static int __add_preferred_console(char *name, int idx, char *options,
				   char *brl_options)
{
	struct console_cmdline *c;
	int i;

	for (i = 0; i < MAX_CMDLINECONSOLES && console_cmdline[i].name[0]; i++)
		if (strcmp(console_cmdline[i].name, name) == 0 &&
			  console_cmdline[i].index == idx) {
				if (!brl_options)
					selected_console = i;
				return 0;
		}
	if (i == MAX_CMDLINECONSOLES)
		return -E2BIG;
	if (!brl_options)
		selected_console = i;
	c = &console_cmdline[i];
	strlcpy(c->name, name, sizeof(c->name));
	c->options = options;
#ifdef CONFIG_A11Y_BRAILLE_CONSOLE
	c->brl_options = brl_options;
#endif
	c->index = idx;
	return 0;
}
static int __init console_setup(char *str)
{
	char buf[sizeof(console_cmdline[0].name) + 4]; 
	char *s, *options, *brl_options = NULL;
	int idx;

#ifdef CONFIG_A11Y_BRAILLE_CONSOLE
	if (!memcmp(str, "brl,", 4)) {
		brl_options = "";
		str += 4;
	} else if (!memcmp(str, "brl=", 4)) {
		brl_options = str + 4;
		str = strchr(brl_options, ',');
		if (!str) {
			printk(KERN_ERR "need port name after brl=\n");
			return 1;
		}
		*(str++) = 0;
	}
#endif

	if (str[0] >= '0' && str[0] <= '9') {
		strcpy(buf, "ttyS");
		strncpy(buf + 4, str, sizeof(buf) - 5);
	} else {
		strncpy(buf, str, sizeof(buf) - 1);
	}
	buf[sizeof(buf) - 1] = 0;
	if ((options = strchr(str, ',')) != NULL)
		*(options++) = 0;
#ifdef __sparc__
	if (!strcmp(str, "ttya"))
		strcpy(buf, "ttyS0");
	if (!strcmp(str, "ttyb"))
		strcpy(buf, "ttyS1");
#endif
	for (s = buf; *s; s++)
		if ((*s >= '0' && *s <= '9') || *s == ',')
			break;
	idx = simple_strtoul(s, NULL, 10);
	*s = 0;

	__add_preferred_console(buf, idx, options, brl_options);
	console_set_on_cmdline = 1;
	return 1;
}
__setup("console=", console_setup);

int add_preferred_console(char *name, int idx, char *options)
{
	return __add_preferred_console(name, idx, options, NULL);
}

int update_console_cmdline(char *name, int idx, char *name_new, int idx_new, char *options)
{
	struct console_cmdline *c;
	int i;

	for (i = 0; i < MAX_CMDLINECONSOLES && console_cmdline[i].name[0]; i++)
		if (strcmp(console_cmdline[i].name, name) == 0 &&
			  console_cmdline[i].index == idx) {
				c = &console_cmdline[i];
				strlcpy(c->name, name_new, sizeof(c->name));
				c->name[sizeof(c->name) - 1] = 0;
				c->options = options;
				c->index = idx_new;
				return i;
		}
	
	return -1;
}

bool console_suspend_enabled = 1;
EXPORT_SYMBOL(console_suspend_enabled);

static int __init console_suspend_disable(char *str)
{
	console_suspend_enabled = 0;
	return 1;
}
__setup("no_console_suspend", console_suspend_disable);
module_param_named(console_suspend, console_suspend_enabled,
		bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(console_suspend, "suspend console during suspend"
	" and hibernate operations");

int suspend_console_deferred;
module_param_named(
		suspend_console_deferred, suspend_console_deferred, int, S_IRUGO | S_IWUSR | S_IWGRP
);

void suspend_console(void)
{
	if (!console_suspend_enabled)
		return;
	printk("Suspending console(s) (use no_console_suspend to debug)\n");
	console_lock();
	console_suspended = 1;
	up(&console_sem);
}

void resume_console(void)
{
	if (!console_suspend_enabled)
		return;
	down(&console_sem);
	console_suspended = 0;
	console_unlock();
}

static void __cpuinit console_flush(struct work_struct *work)
{
	console_lock();
	console_unlock();
}

static __cpuinitdata DECLARE_WORK(console_cpu_notify_work, console_flush);

static int __cpuinit console_cpu_notify(struct notifier_block *self,
	unsigned long action, void *hcpu)
{
	switch (action) {
	case CPU_DEAD:
	case CPU_DOWN_FAILED:
	case CPU_UP_CANCELED:
#ifdef CONFIG_CONSOLE_FLUSH_ON_HOTPLUG
		console_lock();
		console_unlock();
#endif
		break;
	case CPU_ONLINE:
	case CPU_DYING:
		
		if (!console_trylock())
			schedule_work(&console_cpu_notify_work);
		else
			console_unlock();
	}
	return NOTIFY_OK;
}

void console_lock(void)
{
	BUG_ON(in_interrupt());
	down(&console_sem);
	if (console_suspended)
		return;
	console_locked = 1;
	console_may_schedule = 1;
	mutex_acquire(&console_lock_dep_map, 0, 0, _RET_IP_);
}
EXPORT_SYMBOL(console_lock);

int console_trylock(void)
{
	if (down_trylock(&console_sem))
		return 0;
	if (console_suspended) {
		up(&console_sem);
		return 0;
	}
	console_locked = 1;
	console_may_schedule = 0;
	mutex_acquire(&console_lock_dep_map, 0, 1, _RET_IP_);
	return 1;
}
EXPORT_SYMBOL(console_trylock);

int is_console_locked(void)
{
	return console_locked;
}

static void console_cont_flush(char *text, size_t size)
{
	unsigned long flags;
	size_t len;

	raw_spin_lock_irqsave(&logbuf_lock, flags);

	if (!cont.len)
		goto out;

	if (console_seq < log_next_seq && !cont.cons)
		goto out;

	len = cont_print_text(text, size);
	raw_spin_unlock(&logbuf_lock);
	stop_critical_timings();
	call_console_drivers(cont.level, text, len);
	start_critical_timings();
	local_irq_restore(flags);
	return;
out:
	raw_spin_unlock_irqrestore(&logbuf_lock, flags);
}

void console_unlock(void)
{
	static char text[LOG_LINE_MAX + PREFIX_MAX];
	static u64 seen_seq;
	unsigned long flags;
	bool wake_klogd = false;
	bool retry;

	if (console_suspended) {
		up(&console_sem);
		return;
	}

	console_may_schedule = 0;

	
	console_cont_flush(text, sizeof(text));
again:
	for (;;) {
		struct log *msg;
		size_t len;
		int level;

		raw_spin_lock_irqsave(&logbuf_lock, flags);
		if (seen_seq != log_next_seq) {
			wake_klogd = true;
			seen_seq = log_next_seq;
		}

		if (console_seq < log_first_seq) {
			
			console_seq = log_first_seq;
			console_idx = log_first_idx;
			console_prev = 0;
		}
skip:
		if (console_seq == log_next_seq)
			break;

		msg = log_from_idx(console_idx, true);
		if (msg->flags & LOG_NOCONS) {
			console_idx = log_next(console_idx, true);
			console_seq++;
			msg->flags &= ~LOG_NOCONS;
			console_prev = msg->flags;
			goto skip;
		}

		level = msg->level;
		len = msg_print_text(msg, console_prev, false,
				     text, sizeof(text));
		console_idx = log_next(console_idx, true);
		console_seq++;
		console_prev = msg->flags;
		raw_spin_unlock(&logbuf_lock);

		stop_critical_timings();	
		call_console_drivers(level, text, len);
		start_critical_timings();
		local_irq_restore(flags);
	}
	console_locked = 0;
	mutex_release(&console_lock_dep_map, 1, _RET_IP_);

	
	if (unlikely(exclusive_console))
		exclusive_console = NULL;

	raw_spin_unlock(&logbuf_lock);

	up(&console_sem);

	raw_spin_lock(&logbuf_lock);
	retry = console_seq != log_next_seq;
	raw_spin_unlock_irqrestore(&logbuf_lock, flags);

	if (retry && console_trylock())
		goto again;

	if (wake_klogd)
		wake_up_klogd();
}
EXPORT_SYMBOL(console_unlock);

void __sched console_conditional_schedule(void)
{
	if (console_may_schedule)
		cond_resched();
}
EXPORT_SYMBOL(console_conditional_schedule);

void console_unblank(void)
{
	struct console *c;

	if (oops_in_progress) {
		if (down_trylock(&console_sem) != 0)
			return;
	} else
		console_lock();

	console_locked = 1;
	console_may_schedule = 0;
	for_each_console(c)
		if ((c->flags & CON_ENABLED) && c->unblank)
			c->unblank();
	console_unlock();
}

struct tty_driver *console_device(int *index)
{
	struct console *c;
	struct tty_driver *driver = NULL;

	console_lock();
	for_each_console(c) {
		if (!c->device)
			continue;
		driver = c->device(c, index);
		if (driver)
			break;
	}
	console_unlock();
	return driver;
}

void console_stop(struct console *console)
{
	console_lock();
	console->flags &= ~CON_ENABLED;
	console_unlock();
}
EXPORT_SYMBOL(console_stop);

void console_start(struct console *console)
{
	console_lock();
	console->flags |= CON_ENABLED;
	console_unlock();
}
EXPORT_SYMBOL(console_start);

static int __read_mostly keep_bootcon;

static int __init keep_bootcon_setup(char *str)
{
	keep_bootcon = 1;
	printk(KERN_INFO "debug: skip boot console de-registration.\n");

	return 0;
}

early_param("keep_bootcon", keep_bootcon_setup);

void register_console(struct console *newcon)
{
	int i;
	unsigned long flags;
	struct console *bcon = NULL;

	if (console_drivers && newcon->flags & CON_BOOT) {
		
		for_each_console(bcon) {
			if (!(bcon->flags & CON_BOOT)) {
				printk(KERN_INFO "Too late to register bootconsole %s%d\n",
					newcon->name, newcon->index);
				return;
			}
		}
	}

	if (console_drivers && console_drivers->flags & CON_BOOT)
		bcon = console_drivers;

	if (preferred_console < 0 || bcon || !console_drivers)
		preferred_console = selected_console;

	if (newcon->early_setup)
		newcon->early_setup();

	if (preferred_console < 0) {
		if (newcon->index < 0)
			newcon->index = 0;
		if (newcon->setup == NULL ||
		    newcon->setup(newcon, NULL) == 0) {
			newcon->flags |= CON_ENABLED;
			if (newcon->device) {
				newcon->flags |= CON_CONSDEV;
				preferred_console = 0;
			}
		}
	}

	for (i = 0; i < MAX_CMDLINECONSOLES && console_cmdline[i].name[0];
			i++) {
		if (strcmp(console_cmdline[i].name, newcon->name) != 0)
			continue;
		if (newcon->index >= 0 &&
		    newcon->index != console_cmdline[i].index)
			continue;
		if (newcon->index < 0)
			newcon->index = console_cmdline[i].index;
#ifdef CONFIG_A11Y_BRAILLE_CONSOLE
		if (console_cmdline[i].brl_options) {
			newcon->flags |= CON_BRL;
			braille_register_console(newcon,
					console_cmdline[i].index,
					console_cmdline[i].options,
					console_cmdline[i].brl_options);
			return;
		}
#endif
		if (newcon->setup &&
		    newcon->setup(newcon, console_cmdline[i].options) != 0)
			break;
		newcon->flags |= CON_ENABLED;
		newcon->index = console_cmdline[i].index;
		if (i == selected_console) {
			newcon->flags |= CON_CONSDEV;
			preferred_console = selected_console;
		}
		break;
	}

	if (!(newcon->flags & CON_ENABLED))
		return;

	if (bcon && ((newcon->flags & (CON_CONSDEV | CON_BOOT)) == CON_CONSDEV))
		newcon->flags &= ~CON_PRINTBUFFER;

	console_lock();
	if ((newcon->flags & CON_CONSDEV) || console_drivers == NULL) {
		newcon->next = console_drivers;
		console_drivers = newcon;
		if (newcon->next)
			newcon->next->flags &= ~CON_CONSDEV;
	} else {
		newcon->next = console_drivers->next;
		console_drivers->next = newcon;
	}
	if (newcon->flags & CON_PRINTBUFFER) {
		raw_spin_lock_irqsave(&logbuf_lock, flags);
		console_seq = syslog_seq;
		console_idx = syslog_idx;
		console_prev = syslog_prev;
		raw_spin_unlock_irqrestore(&logbuf_lock, flags);
		exclusive_console = newcon;
	}
	console_unlock();
	console_sysfs_notify();

	if (bcon &&
	    ((newcon->flags & (CON_CONSDEV | CON_BOOT)) == CON_CONSDEV) &&
	    !keep_bootcon) {
		printk(KERN_INFO "console [%s%d] enabled, bootconsole disabled\n",
			newcon->name, newcon->index);
		for_each_console(bcon)
			if (bcon->flags & CON_BOOT)
				unregister_console(bcon);
	} else {
		printk(KERN_INFO "%sconsole [%s%d] enabled\n",
			(newcon->flags & CON_BOOT) ? "boot" : "" ,
			newcon->name, newcon->index);
	}
}
EXPORT_SYMBOL(register_console);

int unregister_console(struct console *console)
{
        struct console *a, *b;
	int res = 1;

#ifdef CONFIG_A11Y_BRAILLE_CONSOLE
	if (console->flags & CON_BRL)
		return braille_unregister_console(console);
#endif

	console_lock();
	if (console_drivers == console) {
		console_drivers=console->next;
		res = 0;
	} else if (console_drivers) {
		for (a=console_drivers->next, b=console_drivers ;
		     a; b=a, a=b->next) {
			if (a == console) {
				b->next = a->next;
				res = 0;
				break;
			}
		}
	}

	if (console_drivers != NULL && console->flags & CON_CONSDEV)
		console_drivers->flags |= CON_CONSDEV;

	console_unlock();
	console_sysfs_notify();
	return res;
}
EXPORT_SYMBOL(unregister_console);

static int __init printk_late_init(void)
{
	struct console *con;

	for_each_console(con) {
		if (!keep_bootcon && con->flags & CON_BOOT) {
			printk(KERN_INFO "turn off boot console %s%d\n",
				con->name, con->index);
			unregister_console(con);
		}
	}
	hotcpu_notifier(console_cpu_notify, 0);
	return 0;
}
late_initcall(printk_late_init);

#if defined CONFIG_PRINTK
#define PRINTK_BUF_SIZE		512

#define PRINTK_PENDING_WAKEUP	0x01
#define PRINTK_PENDING_SCHED	0x02

static DEFINE_PER_CPU(int, printk_pending);
static DEFINE_PER_CPU(char [PRINTK_BUF_SIZE], printk_sched_buf);

static void wake_up_klogd_work_func(struct irq_work *irq_work)
{
	int pending = __this_cpu_xchg(printk_pending, 0);

	if (pending & PRINTK_PENDING_SCHED) {
		char *buf = __get_cpu_var(printk_sched_buf);
		printk(KERN_WARNING "[sched_delayed] %s", buf);
	}

	if (pending & PRINTK_PENDING_WAKEUP)
		wake_up_interruptible(&log_wait);
}

static DEFINE_PER_CPU(struct irq_work, wake_up_klogd_work) = {
	.func = wake_up_klogd_work_func,
	.flags = IRQ_WORK_LAZY,
};

void wake_up_klogd(void)
{
	preempt_disable();
	if (waitqueue_active(&log_wait)) {
		this_cpu_or(printk_pending, PRINTK_PENDING_WAKEUP);
		irq_work_queue(&__get_cpu_var(wake_up_klogd_work));
	}
	preempt_enable();
}

int printk_sched(const char *fmt, ...)
{
	unsigned long flags;
	va_list args;
	char *buf;
	int r;

	local_irq_save(flags);
	buf = __get_cpu_var(printk_sched_buf);

	va_start(args, fmt);
	r = vsnprintf(buf, PRINTK_BUF_SIZE, fmt, args);
	va_end(args);

	__this_cpu_or(printk_pending, PRINTK_PENDING_SCHED);
	irq_work_queue(&__get_cpu_var(wake_up_klogd_work));
	local_irq_restore(flags);

	return r;
}

DEFINE_RATELIMIT_STATE(printk_ratelimit_state, 5 * HZ, 10);

int __printk_ratelimit(const char *func)
{
	return ___ratelimit(&printk_ratelimit_state, func);
}
EXPORT_SYMBOL(__printk_ratelimit);

bool printk_timed_ratelimit(unsigned long *caller_jiffies,
			unsigned int interval_msecs)
{
	if (*caller_jiffies == 0
			|| !time_in_range(jiffies, *caller_jiffies,
					*caller_jiffies
					+ msecs_to_jiffies(interval_msecs))) {
		*caller_jiffies = jiffies;
		return true;
	}
	return false;
}
EXPORT_SYMBOL(printk_timed_ratelimit);

static DEFINE_SPINLOCK(dump_list_lock);
static LIST_HEAD(dump_list);

int kmsg_dump_register(struct kmsg_dumper *dumper)
{
	unsigned long flags;
	int err = -EBUSY;

	
	if (!dumper->dump)
		return -EINVAL;

	spin_lock_irqsave(&dump_list_lock, flags);
	
	if (!dumper->registered) {
		dumper->registered = 1;
		list_add_tail_rcu(&dumper->list, &dump_list);
		err = 0;
	}
	spin_unlock_irqrestore(&dump_list_lock, flags);

	return err;
}
EXPORT_SYMBOL_GPL(kmsg_dump_register);

int kmsg_dump_unregister(struct kmsg_dumper *dumper)
{
	unsigned long flags;
	int err = -EINVAL;

	spin_lock_irqsave(&dump_list_lock, flags);
	if (dumper->registered) {
		dumper->registered = 0;
		list_del_rcu(&dumper->list);
		err = 0;
	}
	spin_unlock_irqrestore(&dump_list_lock, flags);
	synchronize_rcu();

	return err;
}
EXPORT_SYMBOL_GPL(kmsg_dump_unregister);

static bool always_kmsg_dump;
module_param_named(always_kmsg_dump, always_kmsg_dump, bool, S_IRUGO | S_IWUSR);

void kmsg_dump(enum kmsg_dump_reason reason)
{
	struct kmsg_dumper *dumper;
	unsigned long flags;

	if ((reason > KMSG_DUMP_OOPS) && !always_kmsg_dump)
		return;

	rcu_read_lock();
	list_for_each_entry_rcu(dumper, &dump_list, list) {
		if (dumper->max_reason && reason > dumper->max_reason)
			continue;

		
		dumper->active = true;

		raw_spin_lock_irqsave(&logbuf_lock, flags);
		dumper->cur_seq = clear_seq;
		dumper->cur_idx = clear_idx;
		dumper->next_seq = log_next_seq;
		dumper->next_idx = log_next_idx;
		raw_spin_unlock_irqrestore(&logbuf_lock, flags);

		
		dumper->dump(dumper, reason);

		
		dumper->active = false;
	}
	rcu_read_unlock();
}

bool kmsg_dump_get_line_nolock(struct kmsg_dumper *dumper, bool syslog,
			       char *line, size_t size, size_t *len)
{
	struct log *msg;
	size_t l = 0;
	bool ret = false;

	if (!dumper->active)
		goto out;

	if (dumper->cur_seq < log_first_seq) {
		
		dumper->cur_seq = log_first_seq;
		dumper->cur_idx = log_first_idx;
	}

	
	if (dumper->cur_seq >= log_next_seq)
		goto out;

	msg = log_from_idx(dumper->cur_idx, true);
	l = msg_print_text(msg, 0, syslog, line, size);

	dumper->cur_idx = log_next(dumper->cur_idx, true);
	dumper->cur_seq++;
	ret = true;
out:
	if (len)
		*len = l;
	return ret;
}

bool kmsg_dump_get_line(struct kmsg_dumper *dumper, bool syslog,
			char *line, size_t size, size_t *len)
{
	unsigned long flags;
	bool ret;

	raw_spin_lock_irqsave(&logbuf_lock, flags);
	ret = kmsg_dump_get_line_nolock(dumper, syslog, line, size, len);
	raw_spin_unlock_irqrestore(&logbuf_lock, flags);

	return ret;
}
EXPORT_SYMBOL_GPL(kmsg_dump_get_line);

bool kmsg_dump_get_buffer(struct kmsg_dumper *dumper, bool syslog,
			  char *buf, size_t size, size_t *len)
{
	unsigned long flags;
	u64 seq;
	u32 idx;
	u64 next_seq;
	u32 next_idx;
	enum log_flags prev;
	size_t l = 0;
	bool ret = false;

	if (!dumper->active)
		goto out;

	raw_spin_lock_irqsave(&logbuf_lock, flags);
	if (dumper->cur_seq < log_first_seq) {
		
		dumper->cur_seq = log_first_seq;
		dumper->cur_idx = log_first_idx;
	}

	
	if (dumper->cur_seq >= dumper->next_seq) {
		raw_spin_unlock_irqrestore(&logbuf_lock, flags);
		goto out;
	}

	
	seq = dumper->cur_seq;
	idx = dumper->cur_idx;
	prev = 0;
	while (seq < dumper->next_seq) {
		struct log *msg = log_from_idx(idx, true);

		l += msg_print_text(msg, prev, true, NULL, 0);
		idx = log_next(idx, true);
		seq++;
		prev = msg->flags;
	}

	
	seq = dumper->cur_seq;
	idx = dumper->cur_idx;
	prev = 0;
	while (l > size && seq < dumper->next_seq) {
		struct log *msg = log_from_idx(idx, true);

		l -= msg_print_text(msg, prev, true, NULL, 0);
		idx = log_next(idx, true);
		seq++;
		prev = msg->flags;
	}

	
	next_seq = seq;
	next_idx = idx;

	l = 0;
	prev = 0;
	while (seq < dumper->next_seq) {
		struct log *msg = log_from_idx(idx, true);

		l += msg_print_text(msg, prev, syslog, buf + l, size - l);
		idx = log_next(idx, true);
		seq++;
		prev = msg->flags;
	}

	dumper->next_seq = next_seq;
	dumper->next_idx = next_idx;
	ret = true;
	raw_spin_unlock_irqrestore(&logbuf_lock, flags);
out:
	if (len)
		*len = l;
	return ret;
}
EXPORT_SYMBOL_GPL(kmsg_dump_get_buffer);

void kmsg_dump_rewind_nolock(struct kmsg_dumper *dumper)
{
	dumper->cur_seq = clear_seq;
	dumper->cur_idx = clear_idx;
	dumper->next_seq = log_next_seq;
	dumper->next_idx = log_next_idx;
}

void kmsg_dump_rewind(struct kmsg_dumper *dumper)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&logbuf_lock, flags);
	kmsg_dump_rewind_nolock(dumper);
	raw_spin_unlock_irqrestore(&logbuf_lock, flags);
}
EXPORT_SYMBOL_GPL(kmsg_dump_rewind);

static char dump_stack_arch_desc_str[128];

void __init dump_stack_set_arch_desc(const char *fmt, ...)
{
	va_list args;

	va_start(args, fmt);
	vsnprintf(dump_stack_arch_desc_str, sizeof(dump_stack_arch_desc_str),
		  fmt, args);
	va_end(args);
}

void dump_stack_print_info(const char *log_lvl)
{
	printk("%sCPU: %d PID: %d Comm: %.20s %s %s %.*s\n",
	       log_lvl, raw_smp_processor_id(), current->pid, current->comm,
	       print_tainted(), init_utsname()->release,
	       (int)strcspn(init_utsname()->version, " "),
	       init_utsname()->version);

	if (dump_stack_arch_desc_str[0] != '\0')
		printk("%sHardware name: %s\n",
		       log_lvl, dump_stack_arch_desc_str);

	print_worker_info(log_lvl, current);
}

void show_regs_print_info(const char *log_lvl)
{
	dump_stack_print_info(log_lvl);

	printk("%stask: %p ti: %p task.ti: %p\n",
	       log_lvl, current, current_thread_info(),
	       task_thread_info(current));
}

#endif
