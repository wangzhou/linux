// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Queued spinlock
 *
 * (C) Copyright 2013-2015 Hewlett-Packard Development Company, L.P.
 * (C) Copyright 2013-2014,2018 Red Hat, Inc.
 * (C) Copyright 2015 Intel Corp.
 * (C) Copyright 2015 Hewlett-Packard Enterprise Development LP
 *
 * Authors: Waiman Long <longman@redhat.com>
 *          Peter Zijlstra <peterz@infradead.org>
 */

#ifndef _GEN_PV_LOCK_SLOWPATH

#include <linux/smp.h>
#include <linux/bug.h>
#include <linux/cpumask.h>
#include <linux/percpu.h>
#include <linux/hardirq.h>
#include <linux/mutex.h>
#include <linux/prefetch.h>
#include <asm/byteorder.h>
#include <asm/qspinlock.h>
#include <trace/events/lock.h>

/*
 * Include queued spinlock statistics code
 */
#include "qspinlock_stat.h"

/*
 * The basic principle of a queue-based spinlock can best be understood
 * by studying a classic queue-based spinlock implementation called the
 * MCS lock. A copy of the original MCS lock paper ("Algorithms for Scalable
 * Synchronization on Shared-Memory Multiprocessors by Mellor-Crummey and
 * Scott") is available at
 *
 * https://bugzilla.kernel.org/show_bug.cgi?id=206115
 *
 * This queued spinlock implementation is based on the MCS lock, however to
 * make it fit the 4 bytes we assume spinlock_t to be, and preserve its
 * existing API, we must modify it somehow.
 *
 * In particular; where the traditional MCS lock consists of a tail pointer
 * (8 bytes) and needs the next pointer (another 8 bytes) of its own node to
 * unlock the next pending (next->locked), we compress both these: {tail,
 * next->locked} into a single u32 value.
 *
 * Since a spinlock disables recursion of its own context and there is a limit
 * to the contexts that can nest; namely: task, softirq, hardirq, nmi. As there
 * are at most 4 nesting levels, it can be encoded by a 2-bit number. Now
 * we can encode the tail by combining the 2-bit nesting level with the cpu
 * number. With one byte for the lock value and 3 bytes for the tail, only a
 * 32-bit word is now needed. Even though we only need 1 bit for the lock,
 * we extend it to a full byte to achieve better performance for architectures
 * that support atomic byte write.
 *
 * We also change the first spinner to spin on the lock bit instead of its
 * node; whereby avoiding the need to carry a node from lock to unlock, and
 * preserving existing lock API. This also makes the unlock code simpler and
 * faster.
 *
 * N.B. The current implementation only supports architectures that allow
 *      atomic operations on smaller 8-bit and 16-bit data types.
 *
 */

#include "mcs_spinlock.h"
#define MAX_NODES	4

/*
 * On 64-bit architectures, the mcs_spinlock structure will be 16 bytes in
 * size and four of them will fit nicely in one 64-byte cacheline. For
 * pvqspinlock, however, we need more space for extra data. To accommodate
 * that, we insert two more long words to pad it up to 32 bytes. IOW, only
 * two of them can fit in a cacheline in this case. That is OK as it is rare
 * to have more than 2 levels of slowpath nesting in actual use. We don't
 * want to penalize pvqspinlocks to optimize for a rare case in native
 * qspinlocks.
 */
struct qnode {
	struct mcs_spinlock mcs;
#ifdef CONFIG_PARAVIRT_SPINLOCKS
	long reserved[2];
#endif
};

/*
 * The pending bit spinning loop count.
 * This heuristic is used to limit the number of lockword accesses
 * made by atomic_cond_read_relaxed when waiting for the lock to
 * transition out of the "== _Q_PENDING_VAL" state. We don't spin
 * indefinitely because there's no guarantee that we'll make forward
 * progress.
 */
#ifndef _Q_PENDING_LOOPS
#define _Q_PENDING_LOOPS	1
#endif

/*
 * Per-CPU queue node structures; we can never have more than 4 nested
 * contexts: task, softirq, hardirq, nmi.
 *
 * Exactly fits one 64-byte cacheline on a 64-bit architecture.
 *
 * PV doubles the storage and uses the second cacheline for PV state.
 */
static DEFINE_PER_CPU_ALIGNED(struct qnode, qnodes[MAX_NODES]);

/*
 * We must be able to distinguish between no-tail and the tail at 0:0,
 * therefore increment the cpu number by one.
 */

static inline __pure u32 encode_tail(int cpu, int idx)
{
	u32 tail;

	/* w: 17 shift够么？*/
	tail  = (cpu + 1) << _Q_TAIL_CPU_OFFSET;
	tail |= idx << _Q_TAIL_IDX_OFFSET; /* assume < 4 */

	return tail;
}

static inline __pure struct mcs_spinlock *decode_tail(u32 tail)
{
	int cpu = (tail >> _Q_TAIL_CPU_OFFSET) - 1;
	int idx = (tail &  _Q_TAIL_IDX_MASK) >> _Q_TAIL_IDX_OFFSET;

	return per_cpu_ptr(&qnodes[idx].mcs, cpu);
}

static inline __pure
struct mcs_spinlock *grab_mcs_node(struct mcs_spinlock *base, int idx)
{
	return &((struct qnode *)base + idx)->mcs;
}

#define _Q_LOCKED_PENDING_MASK (_Q_LOCKED_MASK | _Q_PENDING_MASK)

#if _Q_PENDING_BITS == 8
/**
 * clear_pending - clear the pending bit.
 * @lock: Pointer to queued spinlock structure
 *
 * *,1,* -> *,0,*
 */
static __always_inline void clear_pending(struct qspinlock *lock)
{
	WRITE_ONCE(lock->pending, 0);
}

/**
 * clear_pending_set_locked - take ownership and clear the pending bit.
 * @lock: Pointer to queued spinlock structure
 *
 * *,1,0 -> *,0,1
 *
 * Lock stealing is not allowed if this function is used.
 */
static __always_inline void clear_pending_set_locked(struct qspinlock *lock)
{
	WRITE_ONCE(lock->locked_pending, _Q_LOCKED_VAL);
}

/*
 * xchg_tail - Put in the new queue tail code word & retrieve previous one
 * @lock : Pointer to queued spinlock structure
 * @tail : The new queue tail code word
 * Return: The previous queue tail code word
 *
 * xchg(lock, tail), which heads an address dependency
 *
 * p,*,* -> n,*,* ; prev = xchg(lock, node)
 */
static __always_inline u32 xchg_tail(struct qspinlock *lock, u32 tail)
{
	/*
	 * We can use relaxed semantics since the caller ensures that the
	 * MCS node is properly initialized before updating the tail.
	 */
	return (u32)xchg_relaxed(&lock->tail,
				 tail >> _Q_TAIL_OFFSET) << _Q_TAIL_OFFSET;
}

#else /* _Q_PENDING_BITS == 8 */

/**
 * clear_pending - clear the pending bit.
 * @lock: Pointer to queued spinlock structure
 *
 * *,1,* -> *,0,*
 */
static __always_inline void clear_pending(struct qspinlock *lock)
{
	atomic_andnot(_Q_PENDING_VAL, &lock->val);
}

/**
 * clear_pending_set_locked - take ownership and clear the pending bit.
 * @lock: Pointer to queued spinlock structure
 *
 * *,1,0 -> *,0,1
 */
static __always_inline void clear_pending_set_locked(struct qspinlock *lock)
{
	atomic_add(-_Q_PENDING_VAL + _Q_LOCKED_VAL, &lock->val);
}

/**
 * xchg_tail - Put in the new queue tail code word & retrieve previous one
 * @lock : Pointer to queued spinlock structure
 * @tail : The new queue tail code word
 * Return: The previous queue tail code word
 *
 * xchg(lock, tail)
 *
 * p,*,* -> n,*,* ; prev = xchg(lock, node)
 */
static __always_inline u32 xchg_tail(struct qspinlock *lock, u32 tail)
{
	u32 old, new, val = atomic_read(&lock->val);

	for (;;) {
		new = (val & _Q_LOCKED_PENDING_MASK) | tail;
		/*
		 * We can use relaxed semantics since the caller ensures that
		 * the MCS node is properly initialized before updating the
		 * tail.
		 */
		old = atomic_cmpxchg_relaxed(&lock->val, val, new);
		if (old == val)
			break;

		val = old;
	}
	return old;
}
#endif /* _Q_PENDING_BITS == 8 */

/**
 * queued_fetch_set_pending_acquire - fetch the whole lock value and set pending
 * @lock : Pointer to queued spinlock structure
 * Return: The previous lock value
 *
 * *,*,* -> *,1,*
 */
#ifndef queued_fetch_set_pending_acquire
static __always_inline u32 queued_fetch_set_pending_acquire(struct qspinlock *lock)
{
	return atomic_fetch_or_acquire(_Q_PENDING_VAL, &lock->val);
}
#endif

/**
 * set_locked - Set the lock bit and own the lock
 * @lock: Pointer to queued spinlock structure
 *
 * *,*,0 -> *,0,1
 */
static __always_inline void set_locked(struct qspinlock *lock)
{
	WRITE_ONCE(lock->locked, _Q_LOCKED_VAL);
}


/*
 * Generate the native code for queued_spin_unlock_slowpath(); provide NOPs for
 * all the PV callbacks.
 */

static __always_inline void __pv_init_node(struct mcs_spinlock *node) { }
static __always_inline void __pv_wait_node(struct mcs_spinlock *node,
					   struct mcs_spinlock *prev) { }
static __always_inline void __pv_kick_node(struct qspinlock *lock,
					   struct mcs_spinlock *node) { }
static __always_inline u32  __pv_wait_head_or_lock(struct qspinlock *lock,
						   struct mcs_spinlock *node)
						   { return 0; }

#define pv_enabled()		false

#define pv_init_node		__pv_init_node
#define pv_wait_node		__pv_wait_node
#define pv_kick_node		__pv_kick_node
#define pv_wait_head_or_lock	__pv_wait_head_or_lock

#ifdef CONFIG_PARAVIRT_SPINLOCKS
#define queued_spin_lock_slowpath	native_queued_spin_lock_slowpath
#endif

#endif /* _GEN_PV_LOCK_SLOWPATH */

/**
 * queued_spin_lock_slowpath - acquire the queued spinlock
 * @lock: Pointer to queued spinlock structure
 * @val: Current value of the queued spinlock 32-bit word
 *
 * (queue tail, pending bit, lock value)
 *
 *              fast     :    slow                                  :    unlock
 *                       :                                          :
 * uncontended  (0,0,0) -:--> (0,0,1) ------------------------------:--> (*,*,0)
 *                       :       | ^--------.------.             /  :
 *                       :       v           \      \            |  :
 * pending               :    (0,1,1) +--> (0,1,0)   \           |  :
 *                       :       | ^--'              |           |  :
 *                       :       v                   |           |  :
 * uncontended           :    (n,x,y) +--> (n,0,0) --'           |  :
 *   queue               :       | ^--'                          |  :
 *                       :       v                               |  :
 * contended             :    (*,x,y) +--> (*,0,0) ---> (*,0,1) -'  :
 *   queue               :         ^--'                             :
 */
void __lockfunc queued_spin_lock_slowpath(struct qspinlock *lock, u32 val)
{
	struct mcs_spinlock *prev, *next, *node;
	u32 old, tail;
	int idx;

	BUILD_BUG_ON(CONFIG_NR_CPUS >= (1U << _Q_TAIL_CPU_BITS));

	/* w: 半虚拟化? */
	if (pv_enabled())
		goto pv_queue;

	/* w: 只有x86支持? */
	if (virt_spin_lock(lock))
		return;

	/*
	 * Wait for in-progress pending->locked hand-overs with a bounded
	 * number of spins so that we guarantee forward progress.
	 *
	 * 0,1,0 -> 0,0,1
	 */
	/* 第一个cpu离开，已经有一个cpu在pending时，执行这个的应该是第三个cpu */
	if (val == _Q_PENDING_VAL) {
		int cnt = _Q_PENDING_LOOPS;
		/*
		 * VAL != _Q_PENDING_VAL定义读到的值不是xxx, cnt是retry的次数。
		 * 停在这里等待第二个cpu占到锁或者第二个cpu占到然后释放锁，不过
		 * 只try一次，所以这个条件过去后，还是有可能第二个cpu依然pending。
		 */
		val = atomic_cond_read_relaxed(&lock->val,
					       (VAL != _Q_PENDING_VAL) || !cnt--);
	}

	/*
	 * If we observe any contention; queue.
	 */
	/*
	 * ~_Q_LOCKED_MASK为0xffffff00, 第三个cpu入口。(*, 1, 不关心)的情况，只要
	 * 有pending就可以进来这里。
	 */
	if (val & ~_Q_LOCKED_MASK)
		goto queue;

	/*
	 * trylock || pending
	 *
	 * 0,0,* -> 0,1,* -> 0,0,1 pending, trylock
	 */
	/*
	 * 返回val的old值。第二个排队cpu入口，(0, 0, 1)的情况。(0, 1, 0)的情况
	 * 在上面已经处理，(*, 1, 0/1)也已处理。
	 *
	 * 可能存在两个cpu抢(0, 0, 1)的情况。返回值是lock->val老值，所以可以根据
	 * 返回的val做处理，这时后一个set pending bit的cpu得到老值是(0, 1, 1)。
	 * 对于这种情况，后一个core上的lock请求要queue起来。
	 *
	 * 而且这里是只用pending bit覆盖val，所以还会出现其它的问题。比如当前 
	 * core之前拿到的是(0, 0, 1)，但是在当前core走这个流程的时候，占有锁
	 * 的core把锁去掉了，也就是val已经变(0, 0, 0)，如果是这样返回的val就是
	 * (0, 0, 0)。这种情况应该不用管。
	 *
	 * 还有一种情况，
	 */
	val = queued_fetch_set_pending_acquire(lock);

	/*
	 * If we observe contention, there is a concurrent locker.
	 *
	 * Undo and queue; our setting of PENDING might have made the
	 * n,0,0 -> 0,0,0 transition fail and it will now be waiting
	 * on @next to become !NULL.
	 *
	 * 没有理解上面的解释？上面的val & ~_Q_LOCKED_MASK不是已经拦住了(n,0,0)
	 * 的状态么?
	 */
	/* 检测到如上的(0, 1, 1) */
	if (unlikely(val & ~_Q_LOCKED_MASK)) {

		/* Undo PENDING if we set it. */
		/* 似乎这里总是走不到这个分支？*/
		if (!(val & _Q_PENDING_MASK))
			clear_pending(lock);

		goto queue;
	}

	/*
	 * We're pending, wait for the owner to go away.
	 *
	 * 0,1,1 -> *,1,0
	 *
	 * this wait loop must be a load-acquire such that we match the
	 * store-release that clears the locked bit and create lock
	 * sequentiality; this is because not all
	 * clear_pending_set_locked() implementations imply full
	 * barriers.
	 */
	/*
	 * pending等locked离开的逻辑，就是一直读等到locked是0。注意，这里有一个
	 * 问题，如下的读等待里使用了ldxr + wfe，但是unlock的地方只是一个普通store,
	 * 那么看来关键还时ldxr设exclusive这个标记，有这个标记后，只要对这个标记
	 * 做invalid就可以触发wfe的唤醒。
	 */
	if (val & _Q_LOCKED_MASK)
		smp_cond_load_acquire(&lock->locked, !VAL);

	/*
	 * take ownership and clear the pending bit.
	 *
	 * 0,1,0 -> 0,0,1
	 */
	/* pending占据了锁，locked set 1，pending bit清理掉 */
	clear_pending_set_locked(lock);
	lockevent_inc(lock_pending);
	return;

	/*
	 * End of pending bit optimistic spinning and beginning of MCS
	 * queuing.
	 */
queue:
	lockevent_inc(lock_slowpath);
pv_queue:
	node = this_cpu_ptr(&qnodes[0].mcs);
	/* 看起来每个context是混乱编号的? */
	idx = node->count++;
	tail = encode_tail(smp_processor_id(), idx);

	trace_contention_begin(lock, LCB_F_SPIN);

	/*
	 * 4 nodes are allocated based on the assumption that there will
	 * not be nested NMIs taking spinlocks. That may not be true in
	 * some architectures even though the chance of needing more than
	 * 4 nodes will still be extremely unlikely. When that happens,
	 * we fall back to spinning on the lock directly without using
	 * any MCS node. This is not the most elegant solution, but is
	 * simple enough.
	 */
	/* 怎么理解内核里只有四种上下文的spinlock? */
	if (unlikely(idx >= MAX_NODES)) {
		lockevent_inc(lock_no_node);
		while (!queued_spin_trylock(lock))
			cpu_relax();
		goto release;
	}

	node = grab_mcs_node(node, idx);

	/*
	 * Keep counts of non-zero index values:
	 */
	lockevent_cond_inc(lock_use_node2 + idx - 1, idx);

	/*
	 * Ensure that we increment the head node->count before initialising
	 * the actual node. If the compiler is kind enough to reorder these
	 * stores, then an IRQ could overwrite our assignments.
	 */
	barrier();

	node->locked = 0;
	node->next = NULL;
	pv_init_node(node);

	/*
	 * We touched a (possibly) cold cacheline in the per-cpu queue node;
	 * attempt the trylock once more in the hope someone let go while we
	 * weren't watching.
	 */
	/*
	 * 这里直接先try一把，如果pending/lock都是0，就确实轮到自己加锁了，因为
	 * 这里的排队等锁，和其它核的放锁是在不同core上，是有可能在排队流程中，
	 * 拥有锁的core已经把锁已经放掉的。
	 */
	if (queued_spin_trylock(lock))
		goto release;

	/*
	 * Ensure that the initialisation of @node is complete before we
	 * publish the updated tail via xchg_tail() and potentially link
	 * @node into the waitqueue via WRITE_ONCE(prev->next, node) below.
	 */
	smp_wmb();

	/*
	 * Publish the updated tail.
	 * We have already touched the queueing cacheline; don't bother with
	 * pending stuff.
	 *
	 * p,*,* -> n,*,*
	 */
	/* old是之前的tail */
	old = xchg_tail(lock, tail);
	next = NULL;

	/*
	 * if there was a previous node; link it and wait until reaching the
	 * head of the waitqueue.
	 */
	/* 老的tail不是0，说明当前的节点不是第一个mcs node */
	if (old & _Q_TAIL_MASK) {
		prev = decode_tail(old);

		/* Link @node into the waitqueue. */
		WRITE_ONCE(prev->next, node);

		pv_wait_node(node, prev);
		/*
		 * 如果不是mcs list的第一个节点，就在当前节点spin等待。读到0就一直读。
		 * 但是下面可以看到，这里即使等到，也会在下面locked前再次等到
		 * pending/locked都是0。
		 *
		 * 注意，这里有ldxr + wfe的core睡眠。当这个读等待被解开时，note
		 * 变成了第三个cpu，这个cpu在下面的代码上继续等待pending/locked
		 * 为0。
		 */
		arch_mcs_spin_lock_contended(&node->locked);

		/*
		 * While waiting for the MCS lock, the next pointer may have
		 * been set by another lock waiter. We optimistically load
		 * the next pointer & prefetch the cacheline for writing
		 * to reduce latency in the upcoming MCS unlock operation.
		 */
		next = READ_ONCE(node->next);
		if (next)
			/* 预取当前第4个cpu的mcs node的指针 */
			prefetchw(next);
	}

	/*
	 * 当old & _Q_TAIL_MASK是0时，就是第一个mcs node进来时，会跳过前面的分支
	 * 直接走到这里，这种情况下，第一个mcs node的cpu在这里等待pending/locked
	 * 为0。
	 *
	 * 注意，上面分支中的等待解开，也会到这里。上面分支解开的点是，当第三个
	 * cpu占有锁的时候会给后面的mcs node写1。
	 *
	 * 所以，第四个以及之后的cpu走到上面的点都会停下等待，但是，所有的解开等待
	 * 的逻辑都发生在当时的第三个和第四个cpu之间。
	 */
	/*
	 * we're at the head of the waitqueue, wait for the owner & pending to
	 * go away.
	 *
	 * *,x,y -> *,0,0
	 *
	 * this wait loop must use a load-acquire such that we match the
	 * store-release that clears the locked bit and create lock
	 * sequentiality; this is because the set_locked() function below
	 * does not imply a full barrier.
	 *
	 * The PV pv_wait_head_or_lock function, if active, will acquire
	 * the lock and return a non-zero value. So we have to skip the
	 * atomic_cond_read_acquire() call. As the next PV queue head hasn't
	 * been designated yet, there is no way for the locked value to become
	 * _Q_SLOW_VAL. So both the set_locked() and the
	 * atomic_cmpxchg_relaxed() calls will be safe.
	 *
	 * If PV isn't active, 0 will be returned instead.
	 *
	 */
	if ((val = pv_wait_head_or_lock(lock, node)))
		goto locked;

	/* 这种都是在循环等待，这里等到的条件是pending/locked都是0 */
	val = atomic_cond_read_acquire(&lock->val, !(VAL & _Q_LOCKED_PENDING_MASK));

locked:
	/*
	 * claim the lock:
	 *
	 * n,0,0 -> 0,0,1 : lock, uncontended
	 * *,*,0 -> *,*,1 : lock, contended
	 *
	 * If the queue head is the only one in the queue (lock value == tail)
	 * and nobody is pending, clear the tail code and grab the lock.
	 * Otherwise, we only need to grab the lock.
	 */

	/*
	 * In the PV case we might already have _Q_LOCKED_VAL set, because
	 * of lock stealing; therefore we must also allow:
	 *
	 * n,0,1 -> 0,0,1
	 *
	 * Note: at this point: (val & _Q_PENDING_MASK) == 0, because of the
	 *       above wait condition, therefore any concurrent setting of
	 *       PENDING will make the uncontended transition fail.
	 */
	/*
	 * 这里有两种情况, 一个是只有一个mcs节点，一个是多于一个mcs节点的情况。
	 * val是第三个cpu占有锁之前的lock->val状态，之前val中的tail已经是当前的
	 * tail，证明当前的mcs node就是mcs node链表里唯一的一个mcs node。
	 */
	if ((val & _Q_TAIL_MASK) == tail) {
		if (atomic_try_cmpxchg_relaxed(&lock->val, &val, _Q_LOCKED_VAL))
			/* 第三个cpu直接占据锁，注意这个时候pending是空着。*/
			goto release; /* No contention */
	}

	/*
	 * Either somebody is queued behind us or _Q_PENDING_VAL got set
	 * which will then detect the remaining tail and queue behind us
	 * ensuring we'll see a @next.
	 */
	/* 走到这里，当前的mcs node后还有mcs node */
	set_locked(lock);

	/*
	 * contended path; wait for next if not observed yet, release.
	 */
	/*
	 * lock->val(tail)和mcs list不是同步更新的, 所以，可能val里的tail已经更新，
	 * 这样上面已经可以检测到当前的mcs node不是唯一个，但是后面mcs node的指针
	 * 还没有挂到当前mcs node的next上。所以，如果是这样，就等到后续mcs node
	 * 挂上来。
	 *
	 * 注意，这里的条件读也是ldxr + wfe睡眠。
	 */
	if (!next)
		next = smp_cond_load_relaxed(&node->next, (VAL));

	/*
	 * next是第二个mcs, 这里写1，会解开上面的spin，不过上面的等待里用wfe睡眠了。
	 * 这里看似没有地方唤醒wfe，实际上ldxr + wfe的睡眠方式，在global monitor
	 * 上记录了睡眠core的标记，普通的store会invalid，睡眠core上的cache line，
	 * 就会唤醒wfe。
	 */
	arch_mcs_spin_unlock_contended(&next->locked);
	pv_kick_node(lock, next);

release:
	trace_contention_end(lock, 0);

	/*
	 * release the node
	 */
	__this_cpu_dec(qnodes[0].mcs.count);
}
EXPORT_SYMBOL(queued_spin_lock_slowpath);

/*
 * Generate the paravirt code for queued_spin_unlock_slowpath().
 */
#if !defined(_GEN_PV_LOCK_SLOWPATH) && defined(CONFIG_PARAVIRT_SPINLOCKS)
#define _GEN_PV_LOCK_SLOWPATH

#undef  pv_enabled
#define pv_enabled()	true

#undef pv_init_node
#undef pv_wait_node
#undef pv_kick_node
#undef pv_wait_head_or_lock

#undef  queued_spin_lock_slowpath
#define queued_spin_lock_slowpath	__pv_queued_spin_lock_slowpath

#include "qspinlock_paravirt.h"
#include "qspinlock.c"

bool nopvspin __initdata;
static __init int parse_nopvspin(char *arg)
{
	nopvspin = true;
	return 0;
}
early_param("nopvspin", parse_nopvspin);
#endif
