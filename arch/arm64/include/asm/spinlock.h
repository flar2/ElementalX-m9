/*
 * Copyright (C) 2012 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef __ASM_SPINLOCK_H
#define __ASM_SPINLOCK_H

#include <asm/spinlock_types.h>
#include <asm/processor.h>


#define arch_spin_unlock_wait(lock) \
	do { while (arch_spin_is_locked(lock)) cpu_relax(); } while (0)

#define arch_spin_lock_flags(lock, flags) arch_spin_lock(lock)

static inline void arch_spin_lock(arch_spinlock_t *lock)
{
	unsigned int tmp;
	arch_spinlock_t lockval, newval;

	asm volatile(
	
"	prfm	pstl1strm, %3\n"
"1:	ldaxr	%w0, %3\n"
"	add	%w1, %w0, %w5\n"
"	stxr	%w2, %w1, %3\n"
"	cbnz	%w2, 1b\n"
	
"	eor	%w1, %w0, %w0, ror #16\n"
"	cbz	%w1, 3f\n"
#ifndef CONFIG_HTC_WFE_DISABLE
"	sevl\n"
"2:	wfe\n"
#else
"2:\n"
#endif
"	ldaxrh	%w2, %4\n"
"	eor	%w1, %w2, %w0, lsr #16\n"
"	cbnz	%w1, 2b\n"
	
"3:"
	: "=&r" (lockval), "=&r" (newval), "=&r" (tmp), "+Q" (*lock)
	: "Q" (lock->owner), "I" (1 << TICKET_SHIFT)
	: "memory");
}

static inline int arch_spin_trylock(arch_spinlock_t *lock)
{
	unsigned int tmp;
	arch_spinlock_t lockval;

	asm volatile(
"1:	ldaxr	%w0, %2\n"
"	eor	%w1, %w0, %w0, ror #16\n"
"	cbnz	%w1, 2f\n"
"	add	%w0, %w0, %3\n"
"	stxr	%w1, %w0, %2\n"
"	cbnz	%w1, 1b\n"
"2:"
	: "=&r" (lockval), "=&r" (tmp), "+Q" (*lock)
	: "I" (1 << TICKET_SHIFT)
	: "memory");

	return !tmp;
}

static inline void arch_spin_unlock(arch_spinlock_t *lock)
{
	asm volatile(
"	stlrh	%w1, %0\n"
#ifdef CONFIG_ARM64_SEV_IN_LOCK_UNLOCK
"	dsb sy\n"
"	sev\n"
#endif
	: "=Q" (lock->owner)
	: "r" (lock->owner + 1)
	: "memory");
}

static inline int arch_spin_value_unlocked(arch_spinlock_t lock)
{
	return lock.owner == lock.next;
}

static inline int arch_spin_is_locked(arch_spinlock_t *lock)
{
	return !arch_spin_value_unlocked(ACCESS_ONCE(*lock));
}

static inline int arch_spin_is_contended(arch_spinlock_t *lock)
{
	arch_spinlock_t lockval = ACCESS_ONCE(*lock);
	return (lockval.next - lockval.owner) > 1;
}
#define arch_spin_is_contended	arch_spin_is_contended


static inline void arch_write_lock(arch_rwlock_t *rw)
{
	unsigned int tmp;

	asm volatile(
#ifndef CONFIG_HTC_WFE_DISABLE
	"	sevl\n"
	"1:	wfe\n"
#else
	"1:\n"
#endif
	"2:	ldaxr	%w0, %1\n"
	"	cbnz	%w0, 1b\n"
	"	stxr	%w0, %w2, %1\n"
	"	cbnz	%w0, 2b\n"
	: "=&r" (tmp), "+Q" (rw->lock)
	: "r" (0x80000000)
	: "memory");
}

static inline int arch_write_trylock(arch_rwlock_t *rw)
{
	unsigned int tmp;

	asm volatile(
	"2:	ldaxr	%w0, %1\n"
	"	cbnz	%w0, 1f\n"
	"	stxr	%w0, %w2, %1\n"
	"	cbnz	%w0, 2b\n"
	"1:\n"
	: "=&r" (tmp), "+Q" (rw->lock)
	: "r" (0x80000000)
	: "memory");

	return !tmp;
}

static inline void arch_write_unlock(arch_rwlock_t *rw)
{
	asm volatile(
	"	stlr	%w1, %0\n"
#ifdef CONFIG_ARM64_SEV_IN_LOCK_UNLOCK
	"	dsb sy\n"
	"	sev\n"
#endif
	: "=Q" (rw->lock) : "r" (0) : "memory");
}

#define arch_write_can_lock(x)		((x)->lock == 0)

static inline void arch_read_lock(arch_rwlock_t *rw)
{
	unsigned int tmp, tmp2;

	asm volatile(
#ifndef CONFIG_HTC_WFE_DISABLE
	"	sevl\n"
	"1:	wfe\n"
#else
	"1:\n"
#endif
	"2:	ldaxr	%w0, %2\n"
	"	add	%w0, %w0, #1\n"
	"	tbnz	%w0, #31, 1b\n"
	"	stxr	%w1, %w0, %2\n"
	"	cbnz	%w1, 2b\n"
	: "=&r" (tmp), "=&r" (tmp2), "+Q" (rw->lock)
	:
	: "memory");
}

static inline void arch_read_unlock(arch_rwlock_t *rw)
{
	unsigned int tmp, tmp2;

	asm volatile(
	"1:	ldxr	%w0, %2\n"
	"	sub	%w0, %w0, #1\n"
	"	stlxr	%w1, %w0, %2\n"
#ifdef CONFIG_ARM64_SEV_IN_LOCK_UNLOCK
	"	dsb sy\n"
	"	sev\n"
#endif
	"	cbnz	%w1, 1b\n"
	: "=&r" (tmp), "=&r" (tmp2), "+Q" (rw->lock)
	:
	: "memory");
}

static inline int arch_read_trylock(arch_rwlock_t *rw)
{
	unsigned int tmp, tmp2 = 1;

	asm volatile(
	"2:	ldaxr	%w0, %2\n"
	"	add	%w0, %w0, #1\n"
	"	tbnz	%w0, #31, 1f\n"
	"	stxr	%w1, %w0, %2\n"
	"	cbnz	%w1, 2b\n"
	"1:\n"
	: "=&r" (tmp), "+r" (tmp2), "+Q" (rw->lock)
	:
	: "memory");

	return !tmp2;
}

#define arch_read_can_lock(x)		((x)->lock < 0x80000000)

#define arch_read_lock_flags(lock, flags) arch_read_lock(lock)
#define arch_write_lock_flags(lock, flags) arch_write_lock(lock)

#define arch_spin_relax(lock)	cpu_relax()
#define arch_read_relax(lock)	cpu_relax()
#define arch_write_relax(lock)	cpu_relax()

#endif 
