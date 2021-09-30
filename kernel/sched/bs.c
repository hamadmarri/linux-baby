// SPDX-License-Identifier: GPL-2.0
/*
 * Basic Scheduler (BS) Class (SCHED_NORMAL/SCHED_BATCH)
 *
 *  Copyright (C) 2021, Hamad Al Marri <hamad.s.almarri@gmail.com>
 */
#include "sched.h"
#include "fair_numa.h"
#include "pelt.h"

/*
 * After fork, child runs first. If set to 0 (default) then
 * parent will (try to) run first.
 */
unsigned int sysctl_sched_child_runs_first __read_mostly;

const_debug unsigned int sysctl_sched_migration_cost	= 500000UL;

void __init sched_init_granularity(void) {}

#ifdef CONFIG_SMP
/* Give new sched_entity start runnable values to heavy its load in infant time */
void init_entity_runnable_average(struct sched_entity *se) {}
void post_init_entity_util_avg(struct task_struct *p) {}
void update_max_interval(void) {}
#endif /** CONFIG_SMP */

void init_cfs_rq(struct cfs_rq *cfs_rq)
{
	cfs_rq->tasks_timeline = RB_ROOT_CACHED;
#ifdef CONFIG_SMP
	raw_spin_lock_init(&cfs_rq->removed.lock);
#endif
}

__init void init_sched_fair_class(void) {}

void reweight_task(struct task_struct *p, int prio) {}

static inline struct sched_entity *se_of(struct bs_node *bsn)
{
	return container_of(bsn, struct sched_entity, bs_node);
}

#ifdef CONFIG_SCHED_SMT
DEFINE_STATIC_KEY_FALSE(sched_smt_present);
EXPORT_SYMBOL_GPL(sched_smt_present);

static inline void set_idle_cores(int cpu, int val)
{
	struct sched_domain_shared *sds;

	sds = rcu_dereference(per_cpu(sd_llc_shared, cpu));
	if (sds)
		WRITE_ONCE(sds->has_idle_cores, val);
}

static inline bool test_idle_cores(int cpu, bool def)
{
	struct sched_domain_shared *sds;

	sds = rcu_dereference(per_cpu(sd_llc_shared, cpu));
	if (sds)
		return READ_ONCE(sds->has_idle_cores);

	return def;
}

void __update_idle_core(struct rq *rq)
{
	int core = cpu_of(rq);
	int cpu;

	rcu_read_lock();
	if (test_idle_cores(core, true))
		goto unlock;

	for_each_cpu(cpu, cpu_smt_mask(core)) {
		if (cpu == core)
			continue;

		if (!available_idle_cpu(cpu))
			goto unlock;
	}

	set_idle_cores(core, 1);
unlock:
	rcu_read_unlock();
}
#endif

static void
account_entity_enqueue(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
#ifdef CONFIG_SMP
	if (entity_is_task(se)) {
		struct rq *rq = rq_of(cfs_rq);

		account_numa_enqueue(rq, task_of(se));
		list_add(&se->group_node, &rq->cfs_tasks);
	}
#endif
	cfs_rq->nr_running++;
}

static void
account_entity_dequeue(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
#ifdef CONFIG_SMP
	if (entity_is_task(se)) {
		account_numa_dequeue(rq_of(cfs_rq), task_of(se));
		list_del_init(&se->group_node);
	}
#endif
	cfs_rq->nr_running--;
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
}

static void update_curr_fair(struct rq *rq)
{
	update_curr(cfs_rq_of(&rq->curr->se));
}

static void __enqueue_entity(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	struct bs_node *bsn = &se->bs_node;
	struct bs_node *prev;

	bsn->next = bsn->prev = NULL;

	//printk("********* A __enqueue_entity");

	// if empty
	if (!cfs_rq->head) {
		//printk("********* B __enqueue_entity");
		
		cfs_rq->head	= bsn;
		cfs_rq->cursor	= bsn;
	}
	// if cursor == head
	else if (cfs_rq->cursor == cfs_rq->head) {
		//printk("********* C __enqueue_entity");
		
		bsn->next = cfs_rq->head;
		cfs_rq->head->prev   = bsn;
		cfs_rq->cursor->prev = bsn;
		cfs_rq->head         = bsn;
	}
	// if cursor != head
	else {
		//printk("********* D __enqueue_entity");
		
		BUG_ON(!cfs_rq->cursor);
		BUG_ON(!cfs_rq->cursor->prev);
		
		prev = cfs_rq->cursor->prev;

		bsn->next = cfs_rq->cursor;
		cfs_rq->cursor->prev = bsn;

		prev->next = bsn;
		bsn->prev  = prev;
	}
	
	//printk("********* E __enqueue_entity");
}

static void __dequeue_entity(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	struct bs_node *bsn = &se->bs_node;
	struct bs_node *prev, *next;

	// if only one se in rq
	if (cfs_rq->head->next == NULL) {
		cfs_rq->head	= NULL;
		cfs_rq->cursor	= NULL;
	} else if (bsn == cfs_rq->head) {
		// if it is the head
		cfs_rq->head		= cfs_rq->head->next;
		cfs_rq->head->prev	= NULL;

		if (bsn == cfs_rq->cursor)
			cfs_rq->cursor = cfs_rq->cursor->next;
	} else {
		// if in the middle
		if (bsn == cfs_rq->cursor)
			cfs_rq->cursor = cfs_rq->cursor->prev;

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

	update_curr(cfs_rq);

	account_entity_enqueue(cfs_rq, se);

	if (!curr)
		__enqueue_entity(cfs_rq, se);

	se->on_rq = 1;
}

static void
dequeue_entity(struct cfs_rq *cfs_rq, struct sched_entity *se, int flags)
{
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

static void check_preempt_wakeup(struct rq *rq, struct task_struct *p, int wake_flags)
{
	struct task_struct *curr = rq->curr;
	struct sched_entity *se = &curr->se, *pse = &p->se;

	if (unlikely(se == pse))
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

	update_curr(cfs_rq_of(se));
	return;

preempt:
	resched_curr(rq);
}

static void
set_next_entity(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	if (se->on_rq)
		__dequeue_entity(cfs_rq, se);

	se->exec_start = rq_clock_task(rq_of(cfs_rq));
	cfs_rq->curr = se;
	se->prev_sum_exec_runtime = se->sum_exec_runtime;
}

static struct sched_entity *
pick_next_entity(struct cfs_rq *cfs_rq, struct sched_entity *curr)
{
	struct bs_node *bsn;

	bsn = cfs_rq->cursor = cfs_rq->cursor->next;

	if (!bsn)
		bsn = cfs_rq->cursor = cfs_rq->head;

	return se_of(bsn);
}

struct task_struct *
pick_next_task_fair(struct rq *rq, struct task_struct *prev, struct rq_flags *rf)
{
	struct cfs_rq *cfs_rq = &rq->cfs;
	struct sched_entity *se;
	struct task_struct *p;
	//int new_tasks;

//again:
	if (!sched_fair_runnable(rq))
		goto idle;

	if (prev)
		put_prev_task(rq, prev);

	se = pick_next_entity(cfs_rq, NULL);
	set_next_entity(cfs_rq, se);

	p = task_of(se);

done: __maybe_unused;
#ifdef CONFIG_SMP
	/*
	 * Move the next running task to the front of
	 * the list, so our cfs_tasks list becomes MRU
	 * one.
	 */
	list_move(&p->se.group_node, &rq->cfs_tasks);
#endif

	return p;

idle:
	if (!rf)
		return NULL;

	//new_tasks = newidle_balance(rq, rf);

	/*
	 * Because newidle_balance() releases (and re-acquires) rq->lock, it is
	 * possible for any higher priority task to appear. In that case we
	 * must re-start the pick_next_entity() loop.
	 */
	//if (new_tasks < 0)
		return RETRY_TASK;

	//if (new_tasks > 0)
		//goto again;

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

	//if (se)
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

static void
check_preempt_tick(struct cfs_rq *cfs_rq, struct sched_entity *curr)
{
	resched_curr(rq_of(cfs_rq));
}

static void
entity_tick(struct cfs_rq *cfs_rq, struct sched_entity *curr, int queued)
{
	update_curr(cfs_rq);

	if (cfs_rq->nr_running > 1)
		check_preempt_tick(cfs_rq, curr);
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

static void migrate_task_rq_fair(struct task_struct *p, int new_cpu)
{
	update_scan_period(p, new_cpu);
}

static void rq_online_fair(struct rq *rq) {}
static void rq_offline_fair(struct rq *rq) {}
static void task_dead_fair(struct task_struct *p) {}

void trigger_load_balance(struct rq *rq) {}

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

static void
prio_changed_fair(struct rq *rq, struct task_struct *p, int oldprio) {}

static void switched_from_fair(struct rq *rq, struct task_struct *p) {}

static void switched_to_fair(struct rq *rq, struct task_struct *p)
{
	if (task_on_rq_queued(p)) {
		/*
		 * We were most likely switched from sched_rt, so
		 * kick off the schedule if running, otherwise just see
		 * if we can still preempt the current task.
		 */
		resched_curr(rq);
	}
}

static unsigned int get_rr_interval_fair(struct rq *rq, struct task_struct *task)
{
	return 0;
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
