#ifdef CONFIG_SMP
static int
balance_fair(struct rq *rq, struct task_struct *prev, struct rq_flags *rf)
{
	if (rq->nr_running)
		return 1;

	return newidle_balance(rq, rf) != 0;
}

/* Runqueue only has SCHED_IDLE tasks enqueued */
static int sched_idle_rq(struct rq *rq)
{
	return unlikely(rq->nr_running == rq->cfs.idle_h_nr_running &&
			rq->nr_running);
}

#ifdef CONFIG_SMP
static int sched_idle_cpu(int cpu)
{
	return sched_idle_rq(cpu_rq(cpu));
}
#endif

static int
select_task_rq_fair(struct task_struct *p, int prev_cpu, int wake_flags)
{
	struct rq *rq = cpu_rq(prev_cpu);
	unsigned int min_this = rq->nr_running;
	unsigned int min = rq->nr_running;
	int cpu, new_cpu = prev_cpu;

	if (sched_idle_cpu(prev_cpu))
		return prev_cpu;

	for_each_online_cpu(cpu) {
		if (!cpumask_test_cpu(cpu, p->cpus_ptr))
			continue;

		if (available_idle_cpu(cpu) || sched_idle_cpu(cpu) || cpu_rq(cpu)->nr_running <= min) {

			if (cpu_rq(cpu)->nr_running == min && !cpus_share_cache(prev_cpu, cpu))
				continue;

			new_cpu = cpu;
			min = cpu_rq(cpu)->nr_running;
		}
	}

	if (min == min_this)
		return prev_cpu;

	return new_cpu;
}

static void pull_from(struct task_struct *p, struct lb_env *env)
{
	struct rq_flags rf;

	// detach task
	deactivate_task(env->src_rq, p, DEQUEUE_NOCLOCK);
	set_task_cpu(p, env->dst_cpu);

	// unlock src rq
	rq_unlock(env->src_rq, env->src_rf);

	// lock this rq
	rq_lock(env->dst_rq, &rf);
	update_rq_clock(env->dst_rq);

	activate_task(env->dst_rq, p, ENQUEUE_NOCLOCK);
	wakeup_preempt(env->dst_rq, p, 0);

	// unlock this rq
	rq_unlock(env->dst_rq, &rf);

	local_irq_restore(env->src_rf->flags);
}

/*
 * Returns 1, if task migration degrades locality
 * Returns 0, if task migration improves locality i.e migration preferred.
 * Returns -1, if task migration is not affected by locality.
 */
static int migrate_degrades_locality(struct task_struct *p, struct rq *dst_rq, struct rq *src_rq)
{
	struct numa_group *numa_group = rcu_dereference(p->numa_group);
	unsigned long src_weight, dst_weight;
	int src_nid, dst_nid, dist;

	if (!static_branch_likely(&sched_numa_balancing))
		return -1;

	if (!p->numa_faults)
		return -1;

	src_nid = cpu_to_node(cpu_of(src_rq));
	dst_nid = cpu_to_node(cpu_of(dst_rq));

	if (src_nid == dst_nid)
		return -1;

	/* Migrating away from the preferred node is always bad. */
	if (src_nid == p->numa_preferred_nid) {
		if (src_rq->nr_running > src_rq->nr_preferred_running)
			return 1;
		else
			return -1;
	}

	/* Encourage migration to the preferred node. */
	if (dst_nid == p->numa_preferred_nid)
		return 0;

	/* Leaving a core idle is often worse than degrading locality. */
	if (sched_idle_cpu(cpu_of(dst_rq)))
		return -1;

	dist = node_distance(src_nid, dst_nid);
	if (numa_group) {
		src_weight = group_weight(p, src_nid, dist);
		dst_weight = group_weight(p, dst_nid, dist);
	} else {
		src_weight = task_weight(p, src_nid, dist);
		dst_weight = task_weight(p, dst_nid, dist);
	}

	return dst_weight < src_weight;
}

#define MIN_HOTNESS 0x7FFFFFFFFFFFFFFLL

static s64 task_hotness(struct task_struct *p, struct rq *dst_rq, struct rq *src_rq)
{
	s64 delta;

	lockdep_assert_rq_held(src_rq);

	// TODO: to remove? no need for binary decision here
	// if (p->sched_class != &fair_sched_class) // && unlikely(!task_has_idle_policy(p)))
	// 	return MIN_HOTNESS;

	// TODO: why migrating idle task?
	// if (unlikely(task_has_idle_policy(p)))
	// 	return MIN_HOTNESS;

	/* SMT siblings share cache */
	if (cpus_share_cache(cpu_of(dst_rq), cpu_of(src_rq)))
		return MIN_HOTNESS;

	/*
	 * TODO: check buddy next
	 * Buddy candidates are cache hot:
	 */
	// if (sched_feat(CACHE_HOT_BUDDY) && dst_rq->nr_running &&
	//     (&p->se == cfs_rq_of(&p->se)->next))
	// 	return 1;

	if (sysctl_sched_migration_cost == -1)
		return 0;

	if (sysctl_sched_migration_cost == 0)
		return MIN_HOTNESS;

	delta = rq_clock_task(src_rq) - p->se.exec_start;

	return delta;
}

static s64 hotness_of(struct task_struct *p, struct lb_env *env)
{
	int tsk_cache_hot;

	tsk_cache_hot = migrate_degrades_locality(p, env->dst_rq, env->src_rq);

	// 0, if task migration improves locality i.e migration preferred.
	if (tsk_cache_hot == 0)
		return MIN_HOTNESS;

	// 1, if task migration degrades locality
	if (tsk_cache_hot == 1)
		return 0;

	// -1, if task migration is not affected by locality.
	return task_hotness(p, env->dst_rq, env->src_rq);
}

static int
can_migrate_task(struct task_struct *p, struct lb_env *env)
{
	if (task_on_cpu(env->src_rq, p))
		return 0;

	/* Disregard pcpu kthreads; they are where they need to be. */
	if (kthread_is_per_cpu(p))
		return 0;

	if (!cpumask_test_cpu(env->dst_cpu, p->cpus_ptr))
		return 0;

	return 1;
}

static int move_task(struct rq *dst_rq, struct rq *src_rq,
			struct rq_flags *src_rf)
{
	struct cfs_rq *src_cfs_rq = &src_rq->cfs;
	struct task_struct *p = NULL, *tsk_itr;
	struct bs_node *bsn = src_cfs_rq->head;
	s64 tsk_coldest = 0, tsk_hotness;

	struct lb_env env = {
		.dst_cpu	= cpu_of(dst_rq),
		.dst_rq		= dst_rq,
		.src_cpu	= cpu_of(src_rq),
		.src_rq		= src_rq,
		.src_rf		= src_rf,
		.idle		= dst_rq->idle_balance ? CPU_IDLE : CPU_NOT_IDLE,
	};

	while (bsn) {
		tsk_itr = task_of(se_of(bsn));

		if (!can_migrate_task(tsk_itr, &env)) {
			bsn = bsn->next;
			continue;
		}

		tsk_hotness = hotness_of(tsk_itr, &env);

		if (!p) {
			tsk_coldest = tsk_hotness;
			p = tsk_itr;
		} else if (tsk_hotness > tsk_coldest) {
			// greater value means it is colder

			tsk_coldest = tsk_hotness;
			p = tsk_itr;
		}

		bsn = bsn->next;
	}

	if (p) {
		pull_from(p, &env);
		return 1;
	} else {
		rq_unlock(src_rq, src_rf);
		local_irq_restore(src_rf->flags);
	}

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

		if (src_rq->nr_running <= 1)
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

		if (src_rq->nr_running <= 1) {
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

static void rebalance(struct rq *this_rq)
{
	int this_cpu = cpu_of(this_rq);
	int cpu, max_cpu, min_cpu;
	unsigned int max, min;
	struct rq *max_rq, *min_rq, *c_rq;
	struct rq_flags src_rf;

	if (this_cpu != 0)
		return;

again:
	max = min = this_rq->nr_running;
	max_rq = min_rq = this_rq;

	// find max
	for_each_online_cpu(cpu) {
		c_rq = cpu_rq(cpu);

		if (unlikely(on_null_domain(c_rq) || !cpu_active(cpu)))
			continue;

		if (c_rq->nr_running > max) {
			max = c_rq->nr_running;
			max_rq = c_rq;
			max_cpu = cpu;
		}
	}

	// find min
	for_each_online_cpu(cpu) {
		c_rq = cpu_rq(cpu);

		if (max_cpu == cpu)
			continue;

		if (unlikely(on_null_domain(c_rq) || !cpu_active(cpu)))
			continue;

		if (available_idle_cpu(cpu) || sched_idle_cpu(cpu) || c_rq->nr_running <= min) {

			if (c_rq->nr_running == min && !cpus_share_cache(max_cpu, cpu))
				continue;

			min = c_rq->nr_running;
			min_rq = c_rq;
			min_cpu = cpu;
		}
	}

	if (min_rq == max_rq || max - min <= 1)
		return;

	rq_lock_irqsave(max_rq, &src_rf);
	update_rq_clock(max_rq);

	if (max_rq->nr_running <= 1) {
		rq_unlock(max_rq, &src_rf);
		local_irq_restore(src_rf.flags);
		return;
	}

	if (move_task(min_rq, max_rq, &src_rf))
		goto again;
}

void trigger_load_balance(struct rq *this_rq)
{
	int this_cpu = cpu_of(this_rq);

	if (this_cpu != 0)
		return;

	rebalance(this_rq);
}

#include "nohz.h"

void update_group_capacity(struct sched_domain *sd, int cpu) {}
#endif /* CONFIG_SMP */
