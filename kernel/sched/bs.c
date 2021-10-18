// SPDX-License-Identifier: GPL-2.0
/*
 * Baby Scheduler (BS) Class (SCHED_NORMAL/SCHED_BATCH)
 *
 *  Copyright (C) 2021, Hamad Al Marri <hamad.s.almarri@gmail.com>
 */
#include "sched.h"
#include "pelt.h"
#include "fair_numa.h"
#include "bs.h"

#define TIME_PERIOD 5000000000ULL

#define MIN_DEADLINE_NS	 590000ULL
/* MIN_DEADLINE_NS * 2 */
#define DEADLINE_NS	1180000ULL

const s64 prio_factor[40] = {
 /* -20 */  -1080000L, -980000L, -880000L, -780000L, -680000L,
 /* -15 */   -580000L, -480000L, -380000L, -280000L, -180000L,
 /* -10 */    -80000L,  -79000L,  -78000L,  -77000L,  -76000L,
 /*  -5 */    -75000L,  -74000L,  -73000L,  -72000L,  -71000L,
 /*   0 */         0L, 1080000L, 1090000L, 1100000L, 1110000L,
 /*   5 */   1120000L, 1130000L, 1140000L, 1150000L, 1160000L,
 /*  10 */   1170000L, 1180000L, 1190000L, 1200000L, 1210000L,
 /*  15 */   1220000L, 1230000L, 1240000L, 1250000L, 1260000L
};

#define YIELD_MARK(bsn)		((bsn)->deadline |= 0x8000000000000000ULL)
#define YIELD_UNMARK(bsn)	((bsn)->deadline &= 0x7FFFFFFFFFFFFFFFULL)

static inline void
calc_bursts(struct sched_entity *se, u64 burst, u64 now)
{
	struct bs_node *bsn = &se->bs_node;

	if ((s64) now - bsn->start_time >= TIME_PERIOD) {
		bsn->start_time = now;
		bsn->prev_bursts = bsn->bursts;
		bsn->bursts = 0ULL;
	}

	bsn->bursts += burst;
}

static inline u64
calc_deadline(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	struct task_struct *p = task_of(se);
	s64 prio_diff;
	u64 now = rq_clock(rq_of(cfs_rq));
	u64 deadline = now + DEADLINE_NS;

	if (PRIO_TO_NICE(p->prio) == 0)
		return deadline;

	prio_diff = prio_factor[PRIO_TO_NICE(p->prio) + 20];

	return deadline + prio_diff;
}

static inline bool
reached_deadline(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	u64 now = rq_clock(rq_of(cfs_rq));
	s64 delta = se->bs_node.deadline - now;

	return (delta <= 0);
}

static inline void update_mlfq(struct sched_entity *se)
{
	struct bs_node *bsn = &se->bs_node;
	u64 burst = bsn->prev_bursts;

	if (!burst)
		burst = bsn->bursts;

	if (burst < 209000000ULL)
		bsn->mlfq = MLFQ_HIGH_INTERACTIVE;
	else if (burst < 523000000ULL)
		bsn->mlfq = MLFQ_INTERACTIVE;
	else if (burst < 1140000000ULL)
		bsn->mlfq = MLFQ_NORMAL;
	else if (burst < 2400000000ULL)
		bsn->mlfq = MLFQ_LONG;
	else
		bsn->mlfq = MLFQ_BATCH;
}

static void update_curr(struct cfs_rq *cfs_rq)
{
	struct sched_entity *curr = cfs_rq->curr;
	u64 now = rq_clock_task(rq_of(cfs_rq));
	u64 delta_exec;

	if (unlikely(!curr))
		return;

	delta_exec = now - curr->exec_start;
	if (unlikely((s64)delta_exec <= 0))
		return;

	curr->exec_start = now;
	curr->sum_exec_runtime += delta_exec;

	calc_bursts(curr, delta_exec, now);
	update_mlfq(curr);
	if (reached_deadline(cfs_rq, curr))
		curr->bs_node.deadline = calc_deadline(cfs_rq, curr);
}

static void update_curr_fair(struct rq *rq)
{
	update_curr(cfs_rq_of(&rq->curr->se));
}

/**
 * Does a have earlier deadline than b?
 */
static inline bool
entity_before(struct bs_node *a, struct bs_node *b)
{
	if (a->mlfq < b->mlfq)
		return true;
	else if (a->mlfq > b->mlfq)
		return false;

	return (s64)(a->deadline - b->deadline) < 0;
}

#ifdef CONFIG_SCHED_HRTICK

#define DIFF_DL(a, b) ( (s64)((a)->bs_node.deadline - (b)->bs_node.deadline) )
#define DIFF_DL_NOW(a, now) ( (s64)((a)->bs_node.deadline - (now)) )

static struct sched_entity *
pick_next_entity(struct cfs_rq *cfs_rq, struct sched_entity *curr);

/*
 *     cs         cm         cd
 *     |  min dl  |          |
 *     |<-------->|          |
 *  n1 |    n1    |    n2    | n3
 *  @@@|@@@@@@@@@@|##########|****
 *     | c1       |          |
 *     | ^        |    c2    |
 *     |now       |    ^     |
 *     |          |   now    |
 *
 * n1 & c1: hr(cm - c1)
 * n2 & c1: hr(cd - c1)
 * n3 & c1: hr(n3 - c1)
 *
 * n1 & c2: resched_curr
 * n2 & c2: hr(cd - c2)
 * n3 & c2: hr(n3 - c2)
 */
static void hrtick_start_fair(struct rq *rq, struct task_struct *pcurr)
{
	struct sched_entity *curr = &pcurr->se;
	struct bs_node *c_bsn = &curr->bs_node;
	struct sched_entity *next;
	u64 now = rq_clock(rq);
	s64 cm, cd, c_ran;

	if (rq->cfs.h_nr_running < 2)
		return;

	next = pick_next_entity(&rq->cfs, NULL);

	if (!next)
		return;

	c_ran	= curr->sum_exec_runtime - curr->prev_sum_exec_runtime;
	cm	= MIN_DEADLINE_NS - c_ran;
	cd	= DIFF_DL_NOW(curr, now);

	// if c1 (if 590us >= current ran)
	if (cm >= 0) {
		if (entity_before(&next->bs_node, c_bsn)) {
			// if n1
			if (DIFF_DL(curr, next) - MIN_DEADLINE_NS > 0)
				hrtick_start(rq, cm);
			// if n2
			else
				hrtick_start(rq, cd);
		}
		// if n3
		else {
			hrtick_start(rq, DIFF_DL_NOW(next, now));
		}
	}
	// if c2
	else {
		if (entity_before(&next->bs_node, c_bsn)) {
			// if n1
			if (DIFF_DL(curr, next) - MIN_DEADLINE_NS > 0)
				resched_curr(rq);
			// if n2
			else
				hrtick_start(rq, cd);
		}
		// if n3
		else {
			hrtick_start(rq, DIFF_DL_NOW(next, now));
		}
	}
}

/*
 * called from enqueue/dequeue and updates the hrtick when the
 * current task is from our class.
 */
static void hrtick_update(struct rq *rq)
{
	struct task_struct *curr = rq->curr;

	if (!hrtick_enabled_fair(rq) || curr->sched_class != &fair_sched_class)
		return;

	hrtick_start_fair(rq, curr);
}
#else /* !CONFIG_SCHED_HRTICK */
static inline void
hrtick_start_fair(struct rq *rq, struct task_struct *curr)
{
}

static inline void hrtick_update(struct rq *rq)
{
}
#endif

static void __enqueue_entity(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	struct bs_node *bsn = &se->bs_node;

	bsn->next = bsn->prev = NULL;

	// if empty
	if (!cfs_rq->head) {
		cfs_rq->head	= bsn;
	}
	else {
		bsn->next	     = cfs_rq->head;
		cfs_rq->head->prev   = bsn;
		cfs_rq->head         = bsn;
	}
}

static void __dequeue_entity(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	struct bs_node *bsn = &se->bs_node;
	struct bs_node *prev, *next;

	// if only one se in rq
	if (cfs_rq->head->next == NULL) {
		cfs_rq->head = NULL;
	}
	// if it is the head
	else if (bsn == cfs_rq->head) {
		cfs_rq->head	   = cfs_rq->head->next;
		cfs_rq->head->prev = NULL;
	}
	// if in the middle
	else {
		prev = bsn->prev;
		next = bsn->next;

		prev->next = next;
		if (next)
			next->prev = prev;
	}
}

static void
enqueue_entity(struct cfs_rq *cfs_rq, struct sched_entity *se, int flags)
{
	bool curr = cfs_rq->curr == se;
	bool renorm = !(flags & ENQUEUE_WAKEUP) || (flags & ENQUEUE_MIGRATED);

	update_curr(cfs_rq);

	/*
	 * Renormalise, such that we're placed at the current
	 * moment in time, instead of some random moment in the past. Being
	 * placed in the past could significantly boost this task to the
	 * fairness detriment of existing tasks.
	 */
	if (renorm && !curr)
		se->bs_node.deadline = calc_deadline(cfs_rq, se);

	update_mlfq(se);

	account_entity_enqueue(cfs_rq, se);

	if (!curr)
		__enqueue_entity(cfs_rq, se);

	se->on_rq = 1;
}

static void
dequeue_entity(struct cfs_rq *cfs_rq, struct sched_entity *se, int flags)
{
	int task_sleep = flags & DEQUEUE_SLEEP;
	struct task_struct *p = task_of(se);

	/*
	 * Check if didn't reach its deadline, then
	 * rise its priority
	 */
	if (task_sleep && !reached_deadline(cfs_rq, se) &&
	    PRIO_TO_NICE(p->prio) > -20)
	{
		p->prio--;
	}

	update_curr(cfs_rq);

	if (se != cfs_rq->curr)
		__dequeue_entity(cfs_rq, se);

	se->on_rq = 0;
	account_entity_dequeue(cfs_rq, se);
}

static void
enqueue_task_fair(struct rq *rq, struct task_struct *p, int flags)
{
	struct sched_entity *se = &p->se;
	struct cfs_rq *cfs_rq = cfs_rq_of(se);
	int idle_h_nr_running = task_has_idle_policy(p);

	if (!se->on_rq) {
		enqueue_entity(cfs_rq, se, flags);
		cfs_rq->h_nr_running++;
		cfs_rq->idle_h_nr_running += idle_h_nr_running;
	}

	add_nr_running(rq, 1);
	hrtick_update(rq);
}

static void dequeue_task_fair(struct rq *rq, struct task_struct *p, int flags)
{
	struct sched_entity *se = &p->se;
	struct cfs_rq *cfs_rq = cfs_rq_of(se);
	int idle_h_nr_running = task_has_idle_policy(p);

	dequeue_entity(cfs_rq, se, flags);

	cfs_rq->h_nr_running--;
	cfs_rq->idle_h_nr_running -= idle_h_nr_running;

	sub_nr_running(rq, 1);
	hrtick_update(rq);
}

static void yield_task_fair(struct rq *rq)
{
	struct task_struct *curr = rq->curr;
	struct cfs_rq *cfs_rq = task_cfs_rq(curr);

	YIELD_MARK(&curr->se.bs_node);

	/*
	 * Are we the only task in the tree?
	 */
	if (unlikely(rq->nr_running == 1))
		return;

	if (curr->policy != SCHED_BATCH) {
		update_rq_clock(rq);
		/*
		 * Update run-time statistics of the 'current'.
		 */
		update_curr(cfs_rq);
		/*
		 * Tell update_rq_clock() that we've just updated,
		 * so we don't do microscopic update in schedule()
		 * and double the fastpath cost.
		 */
		rq_clock_skip_update(rq);
	}
}

static bool yield_to_task_fair(struct rq *rq, struct task_struct *p)
{
	yield_task_fair(rq);
	return true;
}

static void
set_next_entity(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	int nice = PRIO_TO_NICE(task_of(se)->prio);

	if (se->on_rq)
		__dequeue_entity(cfs_rq, se);

	se->exec_start = rq_clock_task(rq_of(cfs_rq));
	cfs_rq->curr = se;
	se->prev_sum_exec_runtime = se->sum_exec_runtime;

	if (nice >= 0 && reached_deadline(cfs_rq, se))
		se->bs_node.deadline = calc_deadline(cfs_rq, se);
}

static struct sched_entity *
pick_next_entity(struct cfs_rq *cfs_rq, struct sched_entity *curr)
{
	struct bs_node *bsn = cfs_rq->head;
	struct bs_node *next;

	if (!bsn)
		return curr;

	next = bsn->next;
	while (next) {
		if (entity_before(next, bsn))
			bsn = next;

		next = next->next;
	}

	if (curr && entity_before(&curr->bs_node, bsn))
		return curr;

	return se_of(bsn);
}

struct task_struct *
pick_next_task_fair(struct rq *rq, struct task_struct *prev, struct rq_flags *rf)
{
	struct cfs_rq *cfs_rq = &rq->cfs;
	struct sched_entity *se;
	struct task_struct *p;
	int new_tasks;

again:
	if (!sched_fair_runnable(rq))
		goto idle;

	if (prev)
		put_prev_task(rq, prev);

	se = pick_next_entity(cfs_rq, NULL);
	set_next_entity(cfs_rq, se);

	p = task_of(se);

	if (prev)
		YIELD_UNMARK(&prev->se.bs_node);

done: __maybe_unused;
#ifdef CONFIG_SMP
	/*
	 * Move the next running task to the front of
	 * the list, so our cfs_tasks list becomes MRU
	 * one.
	 */
	list_move(&p->se.group_node, &rq->cfs_tasks);
#endif

	if (hrtick_enabled_fair(rq))
		hrtick_start_fair(rq, p);

	return p;

idle:
	if (!rf)
		return NULL;

	new_tasks = newidle_balance(rq, rf);

	/*
	 * Because newidle_balance() releases (and re-acquires) rq->lock, it is
	 * possible for any higher priority task to appear. In that case we
	 * must re-start the pick_next_entity() loop.
	 */
	if (new_tasks < 0)
		return RETRY_TASK;

	if (new_tasks > 0)
		goto again;

	/*
	 * rq is about to be idle, check if we need to update the
	 * lost_idle_time of clock_pelt
	 */
	update_idle_rq_clock_pelt(rq);

	return NULL;
}

static struct task_struct *__pick_next_task_fair(struct rq *rq)
{
	return pick_next_task_fair(rq, NULL, NULL);
}

#ifdef CONFIG_SMP
static struct task_struct *pick_task_fair(struct rq *rq)
{
	struct sched_entity *se;
	struct cfs_rq *cfs_rq = &rq->cfs;
	struct sched_entity *curr = cfs_rq->curr;

	if (!cfs_rq->nr_running)
		return NULL;

	/* When we pick for a remote RQ, we'll not have done put_prev_entity() */
	if (curr) {
		if (curr->on_rq)
			update_curr(cfs_rq);
		else
			curr = NULL;
	}

	se = pick_next_entity(cfs_rq, curr);

	return task_of(se);
}
#endif

static void put_prev_entity(struct cfs_rq *cfs_rq, struct sched_entity *prev)
{
	/*
	 * If still on the runqueue then deactivate_task()
	 * was not called and update_curr() has to be done:
	 */
	if (prev->on_rq)
		update_curr(cfs_rq);

	if (prev->on_rq)
		__enqueue_entity(cfs_rq, prev);

	cfs_rq->curr = NULL;
}

static void put_prev_task_fair(struct rq *rq, struct task_struct *prev)
{
	struct sched_entity *se = &prev->se;

	put_prev_entity(cfs_rq_of(se), se);
}

static void set_next_task_fair(struct rq *rq, struct task_struct *p, bool first)
{
	struct sched_entity *se = &p->se;
	struct cfs_rq *cfs_rq = cfs_rq_of(se);

#ifdef CONFIG_SMP
	if (task_on_rq_queued(p)) {
		/*
		 * Move the next running task to the front of the list, so our
		 * cfs_tasks list becomes MRU one.
		 */
		list_move(&se->group_node, &rq->cfs_tasks);
	}
#endif

	set_next_entity(cfs_rq, se);
}

static inline bool used_burst(struct sched_entity *curr)
{
	struct bs_node *bsn = &curr->bs_node;
	s64 delta_exec = curr->sum_exec_runtime - curr->prev_sum_exec_runtime;
	u64 ideal_exec = DEADLINE_NS * ((u64) bsn->mlfq);

	if (bsn->mlfq == MLFQ_HIGH_INTERACTIVE)
		return true;

	return (s64)(delta_exec - ideal_exec) >= 0;
}

static inline void
check_preempt_tick(struct cfs_rq *cfs_rq, struct sched_entity *curr)
{
	if (used_burst(curr) && pick_next_entity(cfs_rq, curr) != curr)
		resched_curr(rq_of(cfs_rq));
}

static void
entity_tick(struct cfs_rq *cfs_rq, struct sched_entity *curr, int queued)
{
	update_curr(cfs_rq);

#ifdef CONFIG_SCHED_HRTICK
	/*
	 * queued ticks are scheduled to match the slice, so don't bother
	 * validating it and just reschedule.
	 */
	if (queued) {
		resched_curr(rq_of(cfs_rq));
		return;
	}

	/*
	 * don't let the period tick interfere with the hrtick preemption
	 */
	if (!sched_feat(DOUBLE_TICK) &&
			hrtimer_active(&rq_of(cfs_rq)->hrtick_timer))
		return;
#endif

	if (cfs_rq->nr_running > 1)
		check_preempt_tick(cfs_rq, curr);
}

#define SAME_LQ(a, b) ((a)->bs_node.mlfq == (b)->bs_node.mlfq)

static void check_preempt_wakeup(struct rq *rq, struct task_struct *p, int wake_flags)
{
	struct task_struct *curr = rq->curr;
	struct sched_entity *se = &curr->se, *wse = &p->se;

	if (unlikely(se == wse))
		return;

	if (test_tsk_need_resched(curr))
		return;

	/* Idle tasks are by definition preempted by non-idle tasks. */
	if (unlikely(task_has_idle_policy(curr)) &&
	    likely(!task_has_idle_policy(p)))
		goto preempt;

	/*
	 * Batch and idle tasks do not preempt non-idle tasks (their preemption
	 * is driven by the tick):
	 */
	if (unlikely(p->policy != SCHED_NORMAL) || !sched_feat(WAKEUP_PREEMPTION))
		return;

	/*
	 * Lower priority tasks do not preempt higher ones
	 */
	if (p->prio > curr->prio)
		return;

	update_curr(cfs_rq_of(se));

	if (SAME_LQ(se, wse) && used_burst(se))
		goto preempt;
	else if (entity_before(&wse->bs_node, &se->bs_node))
		goto preempt;

	return;

preempt:
	resched_curr(rq);
}

#ifdef CONFIG_SMP
static int
balance_fair(struct rq *rq, struct task_struct *prev, struct rq_flags *rf)
{
	if (rq->nr_running)
		return 1;

	return newidle_balance(rq, rf) != 0;
}

static int
select_task_rq_fair(struct task_struct *p, int prev_cpu, int wake_flags)
{
	struct rq *rq = cpu_rq(prev_cpu);
	unsigned int min_this = rq->nr_running;
	unsigned int min = rq->nr_running;
	int cpu, new_cpu = prev_cpu;

	for_each_online_cpu(cpu) {
		if (cpu_rq(cpu)->nr_running < min) {
			new_cpu = cpu;
			min = cpu_rq(cpu)->nr_running;
		}
	}

	if (min == min_this)
		return prev_cpu;

	return new_cpu;
}

static int
can_migrate_task(struct task_struct *p, int dst_cpu, struct rq *src_rq)
{
	if (task_running(src_rq, p))
		return 0;

	/* Disregard pcpu kthreads; they are where they need to be. */
	if (kthread_is_per_cpu(p))
		return 0;

	if (!cpumask_test_cpu(dst_cpu, p->cpus_ptr))
		return 0;

	return 1;
}

static void pull_from(struct rq *dist_rq,
		      struct rq *src_rq,
		      struct rq_flags *src_rf,
		      struct task_struct *p)
{
	struct rq_flags rf;

	// detach task
	deactivate_task(src_rq, p, DEQUEUE_NOCLOCK);
	set_task_cpu(p, cpu_of(dist_rq));

	// unlock src rq
	rq_unlock(src_rq, src_rf);

	// lock dist rq
	rq_lock(dist_rq, &rf);
	update_rq_clock(dist_rq);

	activate_task(dist_rq, p, ENQUEUE_NOCLOCK);
	check_preempt_curr(dist_rq, p, 0);

	// unlock dist rq
	rq_unlock(dist_rq, &rf);

	local_irq_restore(src_rf->flags);
}

static int move_task(struct rq *dist_rq, struct rq *src_rq,
			struct rq_flags *src_rf)
{
	struct cfs_rq *src_cfs_rq = &src_rq->cfs;
	struct task_struct *p;
	struct bs_node *bsn = src_cfs_rq->head;

	while (bsn) {
		p = task_of(se_of(bsn));
		if (can_migrate_task(p, cpu_of(dist_rq), src_rq)) {
			pull_from(dist_rq, src_rq, src_rf, p);
			return 1;
		}

		bsn = bsn->next;
	}

	/*
	 * Here we know we have not migrated any task,
	 * thus, we need to unlock and return 0
	 * Note: the pull_from does the unlocking for us.
	 */
	rq_unlock(src_rq, src_rf);
	local_irq_restore(src_rf->flags);

	return 0;
}

static int newidle_balance(struct rq *this_rq, struct rq_flags *rf)
{
	int this_cpu = this_rq->cpu;
	struct rq *src_rq;
	int src_cpu = -1, cpu;
	int pulled_task = 0;
	unsigned int max = 0;
	struct rq_flags src_rf;

	/*
	 * We must set idle_stamp _before_ calling idle_balance(), such that we
	 * measure the duration of idle_balance() as idle time.
	 */
	this_rq->idle_stamp = rq_clock(this_rq);

	/*
	 * Do not pull tasks towards !active CPUs...
	 */
	if (!cpu_active(this_cpu))
		return 0;

	rq_unpin_lock(this_rq, rf);
	raw_spin_unlock(&this_rq->__lock);

	for_each_online_cpu(cpu) {
		/*
		 * Stop searching for tasks to pull if there are
		 * now runnable tasks on this rq.
		 */
		if (this_rq->nr_running > 0)
			goto out;

		if (cpu == this_cpu)
			continue;

		src_rq = cpu_rq(cpu);

		if (src_rq->nr_running < 2)
			continue;

		if (src_rq->nr_running > max) {
			max = src_rq->nr_running;
			src_cpu = cpu;
		}
	}

	if (src_cpu != -1) {
		src_rq = cpu_rq(src_cpu);

		rq_lock_irqsave(src_rq, &src_rf);
		update_rq_clock(src_rq);

		if (src_rq->nr_running < 2) {
			rq_unlock(src_rq, &src_rf);
			local_irq_restore(src_rf.flags);
		} else {
			pulled_task = move_task(this_rq, src_rq, &src_rf);
		}
	}

out:
	raw_spin_lock(&this_rq->__lock);

	/*
	 * While browsing the domains, we released the rq lock, a task could
	 * have been enqueued in the meantime. Since we're not going idle,
	 * pretend we pulled a task.
	 */
	if (this_rq->cfs.h_nr_running && !pulled_task)
		pulled_task = 1;

	/* Is there a task of a high priority class? */
	if (this_rq->nr_running != this_rq->cfs.h_nr_running)
		pulled_task = -1;

	if (pulled_task)
		this_rq->idle_stamp = 0;

	rq_repin_lock(this_rq, rf);

	return pulled_task;
}

static inline int on_null_domain(struct rq *rq)
{
	return unlikely(!rcu_dereference_sched(rq->sd));
}

void trigger_load_balance(struct rq *this_rq)
{
	int this_cpu = cpu_of(this_rq);
	int cpu;
	unsigned int max, min;
	struct rq *max_rq, *min_rq, *c_rq;
	struct rq_flags src_rf;

	if (this_cpu != 0)
		return;

	max = min = this_rq->nr_running;
	max_rq = min_rq = this_rq;

	for_each_online_cpu(cpu) {
		c_rq = cpu_rq(cpu);

		/*
		 * Don't need to rebalance while attached to NULL domain or
		 * runqueue CPU is not active
		 */
		if (unlikely(on_null_domain(c_rq) || !cpu_active(cpu)))
			continue;

		if (c_rq->nr_running < min) {
			min = c_rq->nr_running;
			min_rq = c_rq;
		}

		if (c_rq->nr_running > max) {
			max = c_rq->nr_running;
			max_rq = c_rq;
		}
	}

	if (min_rq == max_rq || max - min < 2)
		return;

	rq_lock_irqsave(max_rq, &src_rf);
	update_rq_clock(max_rq);

	if (max_rq->nr_running < 2) {
		rq_unlock(max_rq, &src_rf);
		local_irq_restore(src_rf.flags);
		return;
	}

	move_task(min_rq, max_rq, &src_rf);
}

void update_group_capacity(struct sched_domain *sd, int cpu) {}
#endif /* CONFIG_SMP */

static void task_tick_fair(struct rq *rq, struct task_struct *curr, int queued)
{
	struct sched_entity *se = &curr->se;
	struct cfs_rq *cfs_rq = cfs_rq_of(se);

	entity_tick(cfs_rq, se, queued);

	if (static_branch_unlikely(&sched_numa_balancing))
		task_tick_numa(rq, curr);
}

static void task_fork_fair(struct task_struct *p)
{
	struct cfs_rq *cfs_rq;
	struct sched_entity *curr, *se = &p->se;
	struct rq *rq = this_rq();
	struct rq_flags rf;

	rq_lock(rq, &rf);
	update_rq_clock(rq);

	se->bs_node.start_time = rq_clock_task(rq);
	se->bs_node.prev_bursts = 0;

	cfs_rq = task_cfs_rq(current);

	se->bs_node.deadline = calc_deadline(cfs_rq, se);

	curr = cfs_rq->curr;
	if (curr)
		update_curr(cfs_rq);

	rq_unlock(rq, &rf);
}

/*
 * All the scheduling class methods:
 */
DEFINE_SCHED_CLASS(fair) = {

	.enqueue_task		= enqueue_task_fair,
	.dequeue_task		= dequeue_task_fair,
	.yield_task		= yield_task_fair,
	.yield_to_task		= yield_to_task_fair,

	.check_preempt_curr	= check_preempt_wakeup,

	.pick_next_task		= __pick_next_task_fair,
	.put_prev_task		= put_prev_task_fair,
	.set_next_task          = set_next_task_fair,

#ifdef CONFIG_SMP
	.balance		= balance_fair,
	.pick_task		= pick_task_fair,
	.select_task_rq		= select_task_rq_fair,
	.migrate_task_rq	= migrate_task_rq_fair,

	.rq_online		= rq_online_fair,
	.rq_offline		= rq_offline_fair,

	.task_dead		= task_dead_fair,
	.set_cpus_allowed	= set_cpus_allowed_common,
#endif

	.task_tick		= task_tick_fair,
	.task_fork		= task_fork_fair,

	.prio_changed		= prio_changed_fair,
	.switched_from		= switched_from_fair,
	.switched_to		= switched_to_fair,

	.get_rr_interval	= get_rr_interval_fair,

	.update_curr		= update_curr_fair,
};
