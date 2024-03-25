// SPDX-License-Identifier: GPL-2.0
/*
 * Baby Scheduler (BS) Class (SCHED_NORMAL/SCHED_BATCH)
 *
 *  Copyright (C) 2021, Hamad Al Marri <hamad.s.almarri@gmail.com>
 */
#include <linux/sched/cputime.h>
#include <linux/sched/isolation.h>
#include <linux/sched/nohz.h>
#include <linux/memory-tiers.h>
#include <linux/mempolicy.h>
#include <linux/task_work.h>

#include "sched.h"
#include "pelt.h"

unsigned int sysctl_sched_base_slice	= 7000ULL;
unsigned int bs_shared_quota		= 105000ULL; // 105us
u32 alpha				= 500U;

struct lb_env {
	struct rq		*src_rq;
	int			src_cpu;

	int			dst_cpu;
	struct rq		*dst_rq;

	enum cpu_idle_type	idle;

	struct rq_flags		*src_rf;
	unsigned int		flags;
};

struct global_candidate {
	struct rq *rq;
	struct bs_node *candidate;
	u64 est;

	// for update
	raw_spinlock_t lock;
};

#define MAX_EST 0xFFFFFFFFFFFFFFFULL

struct global_candidate global_candidate = {0, 0, MAX_EST};

#include "fair_numa.h"
#include "fair_debug.h"
#include "fair_dep_funcs.h"

static inline int clear_this_candidate(struct sched_entity *se)
{
	struct bs_node *bsn = &se->bs_node;
	struct bs_node *curr_can = READ_ONCE(global_candidate.candidate);

	if (bsn != curr_can)
		return 0;

	WRITE_ONCE(global_candidate.candidate, NULL);
	WRITE_ONCE(global_candidate.rq, NULL);
	WRITE_ONCE(global_candidate.est, MAX_EST);

	return 1;
}

static inline void clear_rq_candidate(struct cfs_rq *cfs_rq)
{
	struct rq *rq = READ_ONCE(global_candidate.rq);

	if (rq != rq_of(cfs_rq))
		return;

	WRITE_ONCE(global_candidate.candidate, NULL);
	WRITE_ONCE(global_candidate.rq, NULL);
	WRITE_ONCE(global_candidate.est, MAX_EST);
}

static inline void __update_candidate(struct cfs_rq *cfs_rq, struct bs_node *bsn)
{
	unsigned long flags;
	u64 curr_cand_est;

	curr_cand_est = READ_ONCE(global_candidate.est);

	if ((s64)(bsn->est - curr_cand_est) < 0) {
		raw_spin_lock_irqsave(&global_candidate.lock, flags);
		global_candidate.rq = rq_of(cfs_rq);
		global_candidate.candidate = bsn;
		global_candidate.est = bsn->est;
		raw_spin_unlock_irqrestore(&global_candidate.lock, flags);
	}
}

static inline bool
can_be_candidate(struct bs_node *bsn, int this_cpu)
{
	struct task_struct *p;

	if (!bsn)
		return 0;

	p = task_of(se_of(bsn));

	if (kthread_is_per_cpu(p))
		return 0;

	// just migrated
	if (p->se.avg.last_update_time == 0)
		return 0;

	if (task_on_cpu(cpu_rq(this_cpu), p))
		return 0;

	// some tasks are pinned to this cpu
	if (p->nr_cpus_allowed <= 1)
		return 0;

	if (is_migration_disabled(p))
		return 0;

	return 1;
}

static void update_candidate(struct cfs_rq *cfs_rq)
{
	struct bs_node *bsn = NULL;
	int this_cpu = cpu_of(rq_of(cfs_rq));

	if (can_be_candidate(cfs_rq->head, this_cpu))
		bsn = cfs_rq->head;
	else if (can_be_candidate(cfs_rq->q2_head, this_cpu))
		bsn = cfs_rq->q2_head;

	if (bsn)
		__update_candidate(cfs_rq, bsn);
}

static void update_curr(struct cfs_rq *cfs_rq)
{
	struct sched_entity *curr = cfs_rq->curr;
	struct task_struct *curtask = task_of(curr);
	u64 now = rq_clock_task(rq_of(cfs_rq));
	s64 delta_exec, calc;

	if (unlikely(!curr))
		return;

	delta_exec = now - curr->exec_start;
	if (unlikely(delta_exec <= 0))
		return;

	curr->exec_start = now;
	curr->sum_exec_runtime += delta_exec;

	if (schedstat_enabled()) {
		struct sched_statistics *stats;

		stats = __schedstats_from_se(curr);
		__schedstat_set(stats->exec_max,
				max(delta_exec, stats->exec_max));
	}

	calc = calc_delta_fair(delta_exec, curr);
	curr->vruntime			+= calc;
	curr->bs_node.vburst		+= calc;
	curr->bs_node.c_vrt_start	+= calc;
	curr->bs_node.r_vrt_start	+= calc;
#ifdef CONFIG_SCHED_DEBUG
	curr->bs_node.prev_vburst = curr->bs_node.vburst;
#endif
	update_deadline(cfs_rq, curr);

	cfs_rq->local_cand_est = curr->bs_node.est;

	trace_sched_stat_runtime(curtask, delta_exec);
	account_group_exec_runtime(curtask, delta_exec);
	cgroup_account_cputime(curtask, delta_exec);
	if (curtask->dl_server)
		dl_server_update(curtask->dl_server, delta_exec);
}

static void update_curr_fair(struct rq *rq)
{
	update_curr(cfs_rq_of(&rq->curr->se));
}

/**
 * Should `a` preempts `b`?
 */
static inline bool entity_before(struct bs_node *a, struct bs_node *b)
{
	return (s64)(a->est - b->est) < 0;
}

static void __enqueue_entity(struct bs_node **q, struct bs_node *bsn)
{
	struct bs_node *prev;

	if (!(*q) || entity_before(bsn, *q)) {
		bsn->next = *q;
		*q = bsn;
		return;
	}

	// insert after prev
	prev = *q;
	while (prev->next && entity_before(prev->next, bsn))
		prev = prev->next;

	bsn->next = prev->next;
	prev->next = bsn;
}

static void __dequeue_entity_from_q2(struct cfs_rq *cfs_rq, struct bs_node *bsn)
{
	struct bs_node *prev, *itr;

	itr  = cfs_rq->q2_head;
	prev = NULL;

	while (itr && itr != bsn) {
		prev = itr;
		itr = itr->next;
	}

	if (bsn == cfs_rq->q2_head)
		// if it is the head
		cfs_rq->q2_head = cfs_rq->q2_head->next;
	else
		prev->next = itr->next;
}

static void __dequeue_entity(struct cfs_rq *cfs_rq, struct bs_node *bsn)
{
	struct bs_node *prev, *itr;

	itr  = cfs_rq->head;
	prev = NULL;

	while (itr && itr != bsn) {
		prev = itr;
		itr = itr->next;
	}

	if (!itr) {
		// then it is in q2
		__dequeue_entity_from_q2(cfs_rq, bsn);
		return;
	}

	if (bsn == cfs_rq->head)
		// if it is the head
		cfs_rq->head = cfs_rq->head->next;
	else
		prev->next = itr->next;
}

static void
update_est_entity(struct sched_entity *se)
{
	struct bs_node *bsn = &se->bs_node;
	u64 vburst	= bsn->vburst;
	u64 prev_est	= bsn->est;
	u64 next_est;

	/*
	 * <alpha> * <prev burst> + (1 - <alpha>) * <prev estimated>
	 */
	next_est = (alpha * vburst) + ((1000 - alpha) * prev_est);
	next_est /= 1000;

	bsn->est = next_est;
}

static void
enqueue_entity(struct cfs_rq *cfs_rq, struct sched_entity *se, int flags)
{
	bool curr = cfs_rq->curr == se;
	bool wakeup = (flags & ENQUEUE_WAKEUP);

	update_curr(cfs_rq);
	account_entity_enqueue(cfs_rq, se);

	if (!wakeup)
		update_est_entity(se);

	/* Entity has migrated, no longer consider this task hot */
	if (flags & ENQUEUE_MIGRATED)
		se->exec_start = 0;

	if (!curr)
		__enqueue_entity(&cfs_rq->head, &se->bs_node);

	se->on_rq = 1;
}

static void
dequeue_entity(struct cfs_rq *cfs_rq, struct sched_entity *se, int flags)
{
	update_curr(cfs_rq);
	update_est_entity(se);

	if (flags & DEQUEUE_SLEEP)
		se->bs_node.vburst = 0;

	if (se != cfs_rq->curr)
		__dequeue_entity(cfs_rq, &se->bs_node);

	if (clear_this_candidate(se))
		update_candidate(cfs_rq);

	se->on_rq = 0;
	account_entity_dequeue(cfs_rq, se);
}

static void
enqueue_task_fair(struct rq *rq, struct task_struct *p, int flags)
{
	struct sched_entity *se = &p->se;
	struct cfs_rq *cfs_rq = cfs_rq_of(se);
	int idle_h_nr_running = task_has_idle_policy(p);
	int task_new = !(flags & ENQUEUE_WAKEUP);

	/*
	 * The code below (indirectly) updates schedutil which looks at
	 * the cfs_rq utilization to select a frequency.
	 * Let's add the task's estimated utilization to the cfs_rq's
	 * estimated utilization, before we update schedutil.
	 */
	util_est_enqueue(&rq->cfs, p);

	/*
	 * If in_iowait is set, the code below may not trigger any cpufreq
	 * utilization updates, so do it here explicitly with the IOWAIT flag
	 * passed.
	 */
	if (p->in_iowait)
		cpufreq_update_util(rq, SCHED_CPUFREQ_IOWAIT);

	if (!se->on_rq) {
		enqueue_entity(cfs_rq, se, flags);
		cfs_rq->h_nr_running++;
		cfs_rq->idle_h_nr_running += idle_h_nr_running;
	}

	se->bs_node.r_vrt_start = 0;

	update_candidate(cfs_rq);

	add_nr_running(rq, 1);

	if (!task_new)
		update_overutilized_status(rq);

	hrtick_update(rq);
}

static void dequeue_task_fair(struct rq *rq, struct task_struct *p, int flags)
{
	struct sched_entity *se = &p->se;
	struct cfs_rq *cfs_rq = cfs_rq_of(se);
	int task_sleep = flags & DEQUEUE_SLEEP;
	int idle_h_nr_running = task_has_idle_policy(p);

	util_est_dequeue(&rq->cfs, p);

	dequeue_entity(cfs_rq, se, flags);

	cfs_rq->h_nr_running--;
	cfs_rq->idle_h_nr_running -= idle_h_nr_running;

	sub_nr_running(rq, 1);
	util_est_update(&rq->cfs, p, task_sleep);
	hrtick_update(rq);
}

static void yield_task_fair(struct rq *rq)
{
	struct task_struct *curr = rq->curr;
	struct cfs_rq *cfs_rq = task_cfs_rq(curr);

	/*
	 * Are we the only task in the tree?
	 */
	if (unlikely(rq->nr_running == 1))
		return;

	curr->se.yielded = true;

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

static bool yield_to_task_fair(struct rq *rq, struct task_struct *p)
{
	struct sched_entity *se = &p->se;

	if (!se->on_rq)
		return false;

	yield_task_fair(rq);
	return true;
}

static __always_inline
int __entity_end_quota(struct cfs_rq *cfs_rq, struct sched_entity *curr)
{
	unsigned int n = max(cfs_rq->nr_running, 1);
	unsigned int quota;
	struct bs_node *bs = &curr->bs_node;

	quota = max(bs_shared_quota / n, sysctl_sched_base_slice);

	return (s64)(bs->r_vrt_start - (u64)quota) >= 0;
}

static int entity_end_quota(struct cfs_rq *cfs_rq, struct sched_entity *curr)
{
	unsigned int n = cfs_rq->nr_running;

	if (n <= 1)
		return 0;

	return __entity_end_quota(cfs_rq, curr);
}

static int entity_end_min_slice(struct sched_entity *curr)
{
	struct bs_node *bs = &curr->bs_node;

	return (s64)(bs->c_vrt_start - (u64)sysctl_sched_base_slice) >= 0;
}

static void check_preempt_wakeup_fair(struct rq *rq, struct task_struct *p, int wake_flags)
{
	struct cfs_rq *cfs_rq = &rq->cfs;
	struct task_struct *curr = rq->curr;
	struct sched_entity *curr_se = &curr->se, *pse = &p->se;
	int cse_is_idle, pse_is_idle;

	if (unlikely(curr_se == pse))
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

	cse_is_idle = se_is_idle(curr_se);
	pse_is_idle = se_is_idle(pse);

	/*
	 * Preempt an idle group in favor of a non-idle group (and don't preempt
	 * in the inverse case).
	 */
	if (cse_is_idle && !pse_is_idle)
		goto preempt;
	if (cse_is_idle != pse_is_idle)
		return;

	update_curr(cfs_rq_of(curr_se));

	/*
	 * - if curr_se ended quoat then preempt
	 * - if waked entity is before curr_se and
	 *   curr_se ended min slice
	 */
	if (__entity_end_quota(cfs_rq, curr_se))
		goto preempt;

	if (entity_before(&pse->bs_node, &curr_se->bs_node))
		goto preempt;

	return;

preempt:
	resched_curr(rq);
}

static void
set_next_entity(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	if (se->on_rq)
		__dequeue_entity(cfs_rq, &se->bs_node);

	se->exec_start = rq_clock_task(rq_of(cfs_rq));

	se->bs_node.c_vrt_start = 0;

	update_candidate(cfs_rq);
	cfs_rq->local_cand_est = se->bs_node.est;

	cfs_rq->curr = se;
	se->prev_sum_exec_runtime = se->sum_exec_runtime;
}

struct sched_entity *__pick_first_entity(struct cfs_rq *cfs_rq)
{
	if (!cfs_rq->head)
		return NULL;

	return se_of(cfs_rq->head);
}

static struct sched_entity *__pick_next_entity(struct cfs_rq *cfs_rq)
{
	struct bs_node *bs_curr = &cfs_rq->curr->bs_node;

	/*
	 * Here we avoid picking curr
	 * while __pick_first_entity picks the
	 * min since curr == NULL
	 */
	if (cfs_rq->head == bs_curr) {
		if (!cfs_rq->head->next)
			return NULL;

		return se_of(cfs_rq->head->next);
	}

	return se_of(cfs_rq->head);
}

static struct sched_entity* pick_next_entity(struct cfs_rq *cfs_rq, struct sched_entity *curr)
{
	if (!cfs_rq->head) {
		// need to switch to q2
		cfs_rq->head = cfs_rq->q2_head;
		cfs_rq->q2_head = NULL;
	}

	if (!cfs_rq->head)
		return NULL;

	if (!cfs_rq->curr)
		return __pick_first_entity(cfs_rq);

	return __pick_next_entity(cfs_rq);
}

struct task_struct *
pick_next_task_fair(struct rq *rq, struct task_struct *prev, struct rq_flags *rf)
{
	struct cfs_rq *cfs_rq = &rq->cfs;
	struct sched_entity *se;
	struct task_struct *p;
	int new_tasks;

	/*
	 * to cpu0, don't push any
	 * candidates to this rq
	 */
	cfs_rq->local_cand_est = 0;
	clear_rq_candidate(cfs_rq);

again:
	if (!sched_fair_runnable(rq))
		goto idle;

	if (prev)
		put_prev_task(rq, prev);

	se = pick_next_entity(cfs_rq, NULL);
	set_next_entity(cfs_rq, se);

	p = task_of(se);

done: __maybe_unused;
	if (hrtick_enabled_fair(rq))
		hrtick_start_fair(rq, p);

	update_misfit_status(p, rq);

	return p;

idle:
	cfs_rq->local_cand_est = MAX_EST;

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

	/*
	 * to cpu0, don't push any
	 * candidates to this rq
	 */
	cfs_rq->local_cand_est = 0;
	clear_rq_candidate(cfs_rq);

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

static void __enqueue_prev_entity(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	if (se->yielded || entity_end_quota(cfs_rq, se)) {
		se->yielded = false;
		se->bs_node.r_vrt_start = 0;

		__enqueue_entity(&cfs_rq->q2_head, &se->bs_node);
	} else {
		__enqueue_entity(&cfs_rq->head, &se->bs_node);
	}
}

static void put_prev_entity(struct cfs_rq *cfs_rq, struct sched_entity *prev)
{
	/*
	 * If still on the runqueue then deactivate_task()
	 * was not called and update_curr() has to be done:
	 */
	if (prev->on_rq) {
		update_curr(cfs_rq);
		__enqueue_prev_entity(cfs_rq, prev);
	}

	update_est_entity(prev);

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

	set_next_entity(cfs_rq, se);
}


static void
entity_tick(struct cfs_rq *cfs_rq, struct sched_entity *curr, int queued)
{
	struct sched_entity *se;

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

	if (cfs_rq->nr_running <= 1) {
		clear_rq_candidate(cfs_rq);
	} else {
		if (curr->yielded || entity_end_quota(cfs_rq, curr)) {
			resched_curr(rq_of(cfs_rq));
			return;
		}

		se = __pick_first_entity(cfs_rq);
		if (!se)
			return;

		 if (entity_before(&se->bs_node, &curr->bs_node) && entity_end_min_slice(curr)) {
			resched_curr(rq_of(cfs_rq));
			return;
		}
	}

	/*
	 * don't let the period tick interfere with the hrtick preemption
	 */
	if (!sched_feat(DOUBLE_TICK) &&
			hrtimer_active(&rq_of(cfs_rq)->hrtick_timer))
		return;
#endif
}

#include "balancer.h"

static void task_tick_fair(struct rq *rq, struct task_struct *curr, int queued)
{
	struct sched_entity *se = &curr->se;
	struct cfs_rq *cfs_rq = cfs_rq_of(se);

	entity_tick(cfs_rq, se, queued);

	if (static_branch_unlikely(&sched_numa_balancing))
		task_tick_numa(rq, curr);

	update_misfit_status(curr, rq);
	update_overutilized_status(task_rq(curr));
}

static void task_fork_fair(struct task_struct *p)
{
	struct cfs_rq *cfs_rq;
	struct sched_entity *curr;
	struct rq *rq = this_rq();
	struct rq_flags rf;

	rq_lock(rq, &rf);
	update_rq_clock(rq);

	cfs_rq = task_cfs_rq(current);
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

	.wakeup_preempt		= check_preempt_wakeup_fair,

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

static void reweight_entity(struct cfs_rq *cfs_rq, struct sched_entity *se,
			    unsigned long weight)
{
	bool curr = cfs_rq->curr == se;

	if (se->on_rq) {
		/* commit outstanding execution time */
		if (curr)
			update_curr(cfs_rq);

		update_load_sub(&cfs_rq->load, se->load.weight);
	}
	dequeue_load_avg(cfs_rq, se);

	update_load_set(&se->load, weight);

#ifdef CONFIG_SMP
	do {
		u32 divider = get_pelt_divider(&se->avg);

		se->avg.load_avg = div_u64(se_weight(se) * se->avg.load_sum, divider);
	} while (0);
#endif

	enqueue_load_avg(cfs_rq, se);
	if (se->on_rq)
		update_load_add(&cfs_rq->load, se->load.weight);
}

void reweight_task(struct task_struct *p, int prio)
{
	struct sched_entity *se = &p->se;
	struct cfs_rq *cfs_rq = cfs_rq_of(se);
	struct load_weight *load = &se->load;
	unsigned long weight = scale_load(sched_prio_to_weight[prio]);

	reweight_entity(cfs_rq, se, weight);
	load->inv_weight = sched_prio_to_wmult[prio];
}

/* Working cpumask for: load_balance, load_balance_newidle. */
static DEFINE_PER_CPU(cpumask_var_t, load_balance_mask);
static DEFINE_PER_CPU(cpumask_var_t, select_rq_mask);
static DEFINE_PER_CPU(cpumask_var_t, should_we_balance_tmpmask);

__init void init_sched_fair_class(void)
{
#ifdef CONFIG_SMP
	int i;

	for_each_possible_cpu(i) {
		zalloc_cpumask_var_node(&per_cpu(load_balance_mask, i), GFP_KERNEL, cpu_to_node(i));
		zalloc_cpumask_var_node(&per_cpu(select_rq_mask,    i), GFP_KERNEL, cpu_to_node(i));
		zalloc_cpumask_var_node(&per_cpu(should_we_balance_tmpmask, i),
					GFP_KERNEL, cpu_to_node(i));
	}

	open_softirq(SCHED_SOFTIRQ, run_rebalance_domains);

#ifdef CONFIG_NO_HZ_COMMON
	nohz.next_balance = jiffies;
	nohz.next_blocked = jiffies;
	zalloc_cpumask_var(&nohz.idle_cpus_mask, GFP_NOWAIT);
#endif
#endif /* SMP */

}
