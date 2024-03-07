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

static int
can_migrate_task(struct task_struct *p, int dst_cpu, struct rq *src_rq)
{
	if (task_on_cpu(src_rq, p))
		return 0;

	/* Disregard pcpu kthreads; they are where they need to be. */
	if (kthread_is_per_cpu(p))
		return 0;

	if (!cpumask_test_cpu(dst_cpu, p->cpus_ptr))
		return 0;

	return 1;
}

static void pull_from(struct rq *this_rq,
		      struct rq *src_rq,
		      struct rq_flags *src_rf,
		      struct task_struct *p)
{
	struct rq_flags rf;

	// detach task
	deactivate_task(src_rq, p, DEQUEUE_NOCLOCK);
	set_task_cpu(p, cpu_of(this_rq));

	// unlock src rq
	rq_unlock(src_rq, src_rf);

	// lock this rq
	rq_lock(this_rq, &rf);
	update_rq_clock(this_rq);

	activate_task(this_rq, p, ENQUEUE_NOCLOCK);
	wakeup_preempt(this_rq, p, 0);

	// unlock this rq
	rq_unlock(this_rq, &rf);

	local_irq_restore(src_rf->flags);
}

static int move_task(struct rq *this_rq, struct rq *src_rq,
			struct rq_flags *src_rf)
{
	struct cfs_rq *src_cfs_rq = &src_rq->cfs;
	struct task_struct *p;
	struct bs_node *bsn = src_cfs_rq->head;
	int moved = 0;

	while (bsn) {
		p = task_of(se_of(bsn));
		if (can_migrate_task(p, cpu_of(this_rq), src_rq)) {
			pull_from(this_rq, src_rq, src_rf, p);
			moved = 1;
			break;
		}

		bsn = bsn->next;
	}

	if (!moved) {
		rq_unlock(src_rq, src_rf);
		local_irq_restore(src_rf->flags);
	}

	return moved;
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
