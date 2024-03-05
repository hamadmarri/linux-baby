
#ifdef CONFIG_NO_HZ_COMMON
static struct {
	cpumask_var_t idle_cpus_mask;
	atomic_t nr_cpus;
	int has_blocked;		/* Idle CPUS has blocked load */
	int needs_update;		/* Newly idle CPUs need their next_balance collated */
	unsigned long next_balance;     /* in jiffy units */
	unsigned long next_blocked;	/* Next update of blocked load in jiffies */
} nohz ____cacheline_aligned;

static bool update_nohz_stats(struct rq *rq)
{
	unsigned int cpu = rq->cpu;

	if (!rq->has_blocked_load)
		return false;

	if (!cpumask_test_cpu(cpu, nohz.idle_cpus_mask))
		return false;

	if (!time_after(jiffies, READ_ONCE(rq->last_blocked_load_update_tick)))
		return true;

	return rq->has_blocked_load;
}

/*
 * Internal function that runs load balance for all idle cpus. The load balance
 * can be a simple update of blocked load or a complete load balance with
 * tasks movement depending of flags.
 */
static void _nohz_idle_balance(struct rq *this_rq, unsigned int flags)
{
	/* Earliest time when we have to do rebalance again */
	bool has_blocked_load = false;
	int this_cpu = this_rq->cpu;
	int balance_cpu;
	struct rq *rq;

	SCHED_WARN_ON((flags & NOHZ_KICK_MASK) == NOHZ_BALANCE_KICK);

	/*
	 * We assume there will be no idle load after this update and clear
	 * the has_blocked flag. If a cpu enters idle in the mean time, it will
	 * set the has_blocked flag and trigger another update of idle load.
	 * Because a cpu that becomes idle, is added to idle_cpus_mask before
	 * setting the flag, we are sure to not clear the state and not
	 * check the load of an idle cpu.
	 *
	 * Same applies to idle_cpus_mask vs needs_update.
	 */
	if (flags & NOHZ_STATS_KICK)
		WRITE_ONCE(nohz.has_blocked, 0);
	if (flags & NOHZ_NEXT_KICK)
		WRITE_ONCE(nohz.needs_update, 0);

	/*
	 * Ensures that if we miss the CPU, we must see the has_blocked
	 * store from nohz_balance_enter_idle().
	 */
	smp_mb();

	/*
	 * Start with the next CPU after this_cpu so we will end with this_cpu and let a
	 * chance for other idle cpu to pull load.
	 */
	for_each_cpu_wrap(balance_cpu,  nohz.idle_cpus_mask, this_cpu+1) {
		if (!idle_cpu(balance_cpu))
			continue;

		/*
		 * If this CPU gets work to do, stop the load balancing
		 * work being done for other CPUs. Next load
		 * balancing owner will pick it up.
		 */
		if (need_resched()) {
			if (flags & NOHZ_STATS_KICK)
				has_blocked_load = true;
			if (flags & NOHZ_NEXT_KICK)
				WRITE_ONCE(nohz.needs_update, 1);
			goto abort;
		}

		rq = cpu_rq(balance_cpu);

		if (flags & NOHZ_STATS_KICK)
			has_blocked_load |= update_nohz_stats(rq);
	}

abort:
	/* There is still blocked load, enable periodic update */
	if (has_blocked_load)
		WRITE_ONCE(nohz.has_blocked, 1);
}

/*
 * Check if we need to directly run the ILB for updating blocked load before
 * entering idle state. Here we run ILB directly without issuing IPIs.
 *
 * Note that when this function is called, the tick may not yet be stopped on
 * this CPU yet. nohz.idle_cpus_mask is updated only when tick is stopped and
 * cleared on the next busy tick. In other words, nohz.idle_cpus_mask updates
 * don't align with CPUs enter/exit idle to avoid bottlenecks due to high idle
 * entry/exit rate (usec). So it is possible that _nohz_idle_balance() is
 * called from this function on (this) CPU that's not yet in the mask. That's
 * OK because the goal of nohz_run_idle_balance() is to run ILB only for
 * updating the blocked load of already idle CPUs without waking up one of
 * those idle CPUs and outside the preempt disable / irq off phase of the local
 * cpu about to enter idle, because it can take a long time.
 */
void nohz_run_idle_balance(int cpu)
{
	unsigned int flags;

	flags = atomic_fetch_andnot(NOHZ_NEWILB_KICK, nohz_flags(cpu));

	/*
	 * Update the blocked load only if no SCHED_SOFTIRQ is about to happen
	 * (ie NOHZ_STATS_KICK set) and will do the same.
	 */
	if ((flags == NOHZ_NEWILB_KICK) && !need_resched())
		_nohz_idle_balance(cpu_rq(cpu), NOHZ_STATS_KICK);
}

static void set_cpu_sd_state_busy(int cpu)
{
	struct sched_domain *sd;

	rcu_read_lock();
	sd = rcu_dereference(per_cpu(sd_llc, cpu));

	if (!sd || !sd->nohz_idle)
		goto unlock;
	sd->nohz_idle = 0;

	atomic_inc(&sd->shared->nr_busy_cpus);
unlock:
	rcu_read_unlock();
}

void nohz_balance_exit_idle(struct rq *rq)
{
	SCHED_WARN_ON(rq != this_rq());

	if (likely(!rq->nohz_tick_stopped))
		return;

	rq->nohz_tick_stopped = 0;
	cpumask_clear_cpu(rq->cpu, nohz.idle_cpus_mask);
	atomic_dec(&nohz.nr_cpus);

	set_cpu_sd_state_busy(rq->cpu);
}

static void set_cpu_sd_state_idle(int cpu)
{
	struct sched_domain *sd;

	rcu_read_lock();
	sd = rcu_dereference(per_cpu(sd_llc, cpu));

	if (!sd || sd->nohz_idle)
		goto unlock;
	sd->nohz_idle = 1;

	atomic_dec(&sd->shared->nr_busy_cpus);
unlock:
	rcu_read_unlock();
}

/*
 * This routine will record that the CPU is going idle with tick stopped.
 * This info will be used in performing idle load balancing in the future.
 */
void nohz_balance_enter_idle(int cpu)
{
	struct rq *rq = cpu_rq(cpu);

	SCHED_WARN_ON(cpu != smp_processor_id());

	/* If this CPU is going down, then nothing needs to be done: */
	if (!cpu_active(cpu))
		return;

	/* Spare idle load balancing on CPUs that don't want to be disturbed: */
	if (!housekeeping_cpu(cpu, HK_TYPE_SCHED))
		return;

	/*
	 * Can be set safely without rq->lock held
	 * If a clear happens, it will have evaluated last additions because
	 * rq->lock is held during the check and the clear
	 */
	rq->has_blocked_load = 1;

	/*
	 * The tick is still stopped but load could have been added in the
	 * meantime. We set the nohz.has_blocked flag to trig a check of the
	 * *_avg. The CPU is already part of nohz.idle_cpus_mask so the clear
	 * of nohz.has_blocked can only happen after checking the new load
	 */
	if (rq->nohz_tick_stopped)
		goto out;

	/* If we're a completely isolated CPU, we don't play: */
	if (on_null_domain(rq))
		return;

	rq->nohz_tick_stopped = 1;

	cpumask_set_cpu(cpu, nohz.idle_cpus_mask);
	atomic_inc(&nohz.nr_cpus);

	/*
	 * Ensures that if nohz_idle_balance() fails to observe our
	 * @idle_cpus_mask store, it must observe the @has_blocked
	 * and @needs_update stores.
	 */
	smp_mb__after_atomic();

	set_cpu_sd_state_idle(cpu);

	WRITE_ONCE(nohz.needs_update, 1);
out:
	/*
	 * Each time a cpu enter idle, we assume that it has blocked load and
	 * enable the periodic update of the load of idle cpus
	 */
	WRITE_ONCE(nohz.has_blocked, 1);
}
#endif /* CONFIG_NO_HZ_COMMON */
