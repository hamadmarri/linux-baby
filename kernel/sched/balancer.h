#ifdef CONFIG_SMP
static int
balance_fair(struct rq *rq, struct task_struct *prev, struct rq_flags *rf)
{
	if (rq->nr_running)
		return 1;

	return newidle_balance(rq, rf) != 0;
}

static int
wake_affine_idle(int this_cpu, int prev_cpu, int sync)
{
	/*
	 * If this_cpu is idle, it implies the wakeup is from interrupt
	 * context. Only allow the move if cache is shared. Otherwise an
	 * interrupt intensive workload could force all tasks onto one
	 * node depending on the IO topology or IRQ affinity settings.
	 *
	 * If the prev_cpu is idle and cache affine then avoid a migration.
	 * There is no guarantee that the cache hot data from an interrupt
	 * is more important than cache hot data on the prev_cpu and from
	 * a cpufreq perspective, it's better to have higher utilisation
	 * on one CPU.
	 */
	if (available_idle_cpu(this_cpu) && cpus_share_cache(this_cpu, prev_cpu))
		return available_idle_cpu(prev_cpu) ? prev_cpu : this_cpu;

	if (sync && cpu_rq(this_cpu)->nr_running == 1)
		return this_cpu;

	if (available_idle_cpu(prev_cpu))
		return prev_cpu;

	return nr_cpumask_bits;
}

static int
wake_affine(struct task_struct *p, int this_cpu, int prev_cpu, int sync)
{
	int target = nr_cpumask_bits;

	target = wake_affine_idle(this_cpu, prev_cpu, sync);

	if (target == nr_cpumask_bits)
		return prev_cpu;

	return target;
}

static int wake_wide(struct task_struct *p)
{
	unsigned int master = current->wakee_flips;
	unsigned int slave = p->wakee_flips;
	int factor = __this_cpu_read(sd_llc_size);

	if (master < slave)
		swap(master, slave);
	if (slave < factor || master < slave * factor)
		return 0;
	return 1;
}

static void record_wakee(struct task_struct *p)
{
	/*
	 * Only decay a single time; tasks that have less then 1 wakeup per
	 * jiffy will not have built up many flips.
	 */
	if (time_after(jiffies, current->wakee_flip_decay_ts + HZ)) {
		current->wakee_flips >>= 1;
		current->wakee_flip_decay_ts = jiffies;
	}

	if (current->last_wakee != p) {
		current->last_wakee = p;
		current->wakee_flips++;
	}
}

static int
select_task_rq_fair(struct task_struct *p, int prev_cpu, int wake_flags)
{
	int sync = (wake_flags & WF_SYNC) && !(current->flags & PF_EXITING);
	int cpu = smp_processor_id();
	int new_cpu = prev_cpu;
	int want_affine = 0;
	struct rq *rq = cpu_rq(prev_cpu);
	unsigned int min_prev = rq->nr_running;
	unsigned int min = rq->nr_running;
	int this_cpu = smp_processor_id();

	if (wake_flags & WF_TTWU) {
		record_wakee(p);

		if ((wake_flags & WF_CURRENT_CPU) &&
		    cpumask_test_cpu(cpu, p->cpus_ptr))
			return cpu;

		want_affine = !wake_wide(p) && cpumask_test_cpu(cpu, p->cpus_ptr);
	}

	for_each_cpu_wrap(cpu, cpu_online_mask, this_cpu) {
		if (unlikely(!cpumask_test_cpu(cpu, p->cpus_ptr)))
			continue;

		if (want_affine) {
			if (cpu != prev_cpu)
				new_cpu = wake_affine(p, cpu, prev_cpu, sync);

			return new_cpu;
		}

		if (cpu_rq(cpu)->nr_running < min) {
			new_cpu = cpu;
			min = cpu_rq(cpu)->nr_running;
		}
	}

	if (min == min_prev)
		return prev_cpu;

	return new_cpu;
}

#ifdef CONFIG_NO_HZ_COMMON
static inline bool cfs_rq_has_blocked(struct cfs_rq *cfs_rq)
{
	if (cfs_rq->avg.load_avg)
		return true;

	if (cfs_rq->avg.util_avg)
		return true;

	return false;
}

static inline bool others_have_blocked(struct rq *rq)
{
	if (READ_ONCE(rq->avg_rt.util_avg))
		return true;

	if (READ_ONCE(rq->avg_dl.util_avg))
		return true;

	if (thermal_load_avg(rq))
		return true;

#ifdef CONFIG_HAVE_SCHED_AVG_IRQ
	if (READ_ONCE(rq->avg_irq.util_avg))
		return true;
#endif

	return false;
}

static inline void update_blocked_load_tick(struct rq *rq)
{
	WRITE_ONCE(rq->last_blocked_load_update_tick, jiffies);
}

static inline void update_blocked_load_status(struct rq *rq, bool has_blocked)
{
	if (!has_blocked)
		rq->has_blocked_load = 0;
}
#else
static inline bool cfs_rq_has_blocked(struct cfs_rq *cfs_rq) { return false; }
static inline bool others_have_blocked(struct rq *rq) { return false; }
static inline void update_blocked_load_tick(struct rq *rq) {}
static inline void update_blocked_load_status(struct rq *rq, bool has_blocked) {}
#endif

static inline int
update_cfs_rq_load_avg(u64 now, struct cfs_rq *cfs_rq)
{
	unsigned long removed_load = 0, removed_util = 0, removed_runnable = 0;
	struct sched_avg *sa = &cfs_rq->avg;
	int decayed = 0;

	if (cfs_rq->removed.nr) {
		unsigned long r;
		u32 divider = get_pelt_divider(&cfs_rq->avg);

		raw_spin_lock(&cfs_rq->removed.lock);
		swap(cfs_rq->removed.util_avg, removed_util);
		swap(cfs_rq->removed.load_avg, removed_load);
		swap(cfs_rq->removed.runnable_avg, removed_runnable);
		cfs_rq->removed.nr = 0;
		raw_spin_unlock(&cfs_rq->removed.lock);

		r = removed_load;
		sub_positive(&sa->load_avg, r);
		sub_positive(&sa->load_sum, r * divider);
		/* See sa->util_sum below */
		sa->load_sum = max_t(u32, sa->load_sum, sa->load_avg * PELT_MIN_DIVIDER);

		r = removed_util;
		sub_positive(&sa->util_avg, r);
		sub_positive(&sa->util_sum, r * divider);
		/*
		 * Because of rounding, se->util_sum might ends up being +1 more than
		 * cfs->util_sum. Although this is not a problem by itself, detaching
		 * a lot of tasks with the rounding problem between 2 updates of
		 * util_avg (~1ms) can make cfs->util_sum becoming null whereas
		 * cfs_util_avg is not.
		 * Check that util_sum is still above its lower bound for the new
		 * util_avg. Given that period_contrib might have moved since the last
		 * sync, we are only sure that util_sum must be above or equal to
		 *    util_avg * minimum possible divider
		 */
		sa->util_sum = max_t(u32, sa->util_sum, sa->util_avg * PELT_MIN_DIVIDER);

		r = removed_runnable;
		sub_positive(&sa->runnable_avg, r);
		sub_positive(&sa->runnable_sum, r * divider);
		/* See sa->util_sum above */
		sa->runnable_sum = max_t(u32, sa->runnable_sum,
					      sa->runnable_avg * PELT_MIN_DIVIDER);

		decayed = 1;
	}

	decayed |= __update_load_avg_cfs_rq(now, cfs_rq);
	u64_u32_store_copy(sa->last_update_time,
			   cfs_rq->last_update_time_copy,
			   sa->last_update_time);
	return decayed;
}

static bool __update_blocked_fair(struct rq *rq, bool *done)
{
	struct cfs_rq *cfs_rq = &rq->cfs;
	bool decayed;

	decayed = update_cfs_rq_load_avg(cfs_rq_clock_pelt(cfs_rq), cfs_rq);
	if (cfs_rq_has_blocked(cfs_rq))
		*done = false;

	return decayed;
}

static bool __update_blocked_others(struct rq *rq, bool *done)
{
	const struct sched_class *curr_class;
	u64 now = rq_clock_pelt(rq);
	unsigned long thermal_pressure;
	bool decayed;

	/*
	 * update_load_avg() can call cpufreq_update_util(). Make sure that RT,
	 * DL and IRQ signals have been updated before updating CFS.
	 */
	curr_class = rq->curr->sched_class;

	thermal_pressure = arch_scale_thermal_pressure(cpu_of(rq));

	decayed = update_rt_rq_load_avg(now, rq, curr_class == &rt_sched_class) |
		  update_dl_rq_load_avg(now, rq, curr_class == &dl_sched_class) |
		  update_thermal_load_avg(rq_clock_thermal(rq), rq, thermal_pressure) |
		  update_irq_load_avg(rq, 0);

	if (others_have_blocked(rq))
		*done = false;

	return decayed;
}

static void update_blocked_averages(int cpu)
{
	bool decayed = false, done = true;
	struct rq *rq = cpu_rq(cpu);
	struct rq_flags rf;

	rq_lock_irqsave(rq, &rf);
	update_blocked_load_tick(rq);
	update_rq_clock(rq);

	decayed |= __update_blocked_others(rq, &done);
	decayed |= __update_blocked_fair(rq, &done);

	update_blocked_load_status(rq, !done);
	if (decayed)
		cpufreq_update_util(rq, 0);
	rq_unlock_irqrestore(rq, &rf);
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

#ifdef CONFIG_NUMA_BALANCING
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

#else
static inline int migrate_degrades_locality(struct task_struct *p, struct rq *dst_rq, struct rq *src_rq)
{
	return -1;
}
#endif

#define MIN_HOTNESS 0x7FFFFFFFFFFFFFFLL

static s64 task_hotness(struct task_struct *p, struct rq *dst_rq, struct rq *src_rq)
{
	s64 delta;

	lockdep_assert_rq_held(src_rq);

	if (unlikely(task_has_idle_policy(p)))
		return 0;

	/* SMT siblings share cache */
	if (cpus_share_cache(cpu_of(dst_rq), cpu_of(src_rq)))
		return MIN_HOTNESS;

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
can_migrate_task(struct task_struct *p, struct rq *dst_rq, struct rq *src_rq)
{
	/* Disregard pcpu kthreads; they are where they need to be. */
	if (kthread_is_per_cpu(p))
		return 0;

	if (!cpumask_test_cpu(cpu_of(dst_rq), p->cpus_ptr))
		return 0;

	if (task_on_cpu(src_rq, p))
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

		if (!can_migrate_task(tsk_itr, dst_rq, src_rq)) {
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

static int idle_pull_global_candidate(struct rq *dist_rq)
{
	struct rq *src_rq;
	struct task_struct *p;
	struct rq_flags rf, src_rf;
	struct bs_node *cand = READ_ONCE(global_candidate.candidate);

	if (!cand)
		return 0;

	src_rq = READ_ONCE(global_candidate.rq);
	if (!src_rq || src_rq == dist_rq)
		return 0;

	rq_lock_irqsave(src_rq, &src_rf);
	update_rq_clock(src_rq);
		raw_spin_lock(&global_candidate.lock);
			cand = global_candidate.candidate;
			if (!cand)
				goto fail_unlock;

			p = task_of(se_of(cand));
			if (task_rq(p) != src_rq ||
			    !can_migrate_task(p, dist_rq, src_rq))
				goto fail_unlock;

			global_candidate.rq = NULL;
			global_candidate.candidate = NULL;
			global_candidate.est = MAX_EST;
		raw_spin_unlock(&global_candidate.lock);

		// detach task
		deactivate_task(src_rq, p, DEQUEUE_NOCLOCK);
		set_task_cpu(p, cpu_of(dist_rq));
	// unlock src rq
	rq_unlock(src_rq, &src_rf);

	// lock dist rq
	rq_lock(dist_rq, &rf);
	update_rq_clock(dist_rq);
		activate_task(dist_rq, p, ENQUEUE_NOCLOCK);
		wakeup_preempt(dist_rq, p, 0);
	// unlock dist rq
	rq_unlock(dist_rq, &rf);

	local_irq_restore(src_rf.flags);

	// printk(KERN_INFO "idle_pull_global_candidate");

	return 1;

fail_unlock:
	raw_spin_unlock(&global_candidate.lock);
	rq_unlock(src_rq, &src_rf);
	local_irq_restore(src_rf.flags);
	return 0;
}

static void idle_balance(struct rq *this_rq)
{
	int this_cpu = this_rq->cpu;
	struct rq *src_rq;
	int src_cpu = -1, cpu;
	unsigned int max = 0;
	struct rq_flags src_rf;

	if (idle_pull_global_candidate(this_rq))
		return;

	for_each_online_cpu(cpu) {
		/*
		 * Stop searching for tasks to pull if there are
		 * now runnable tasks on this rq.
		 */
		if (this_rq->nr_running > 0)
			return;

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

	if (src_cpu == -1)
		return;

	src_rq = cpu_rq(src_cpu);

	rq_lock_irqsave(src_rq, &src_rf);
	update_rq_clock(src_rq);

	if (src_rq->nr_running < 2) {
		rq_unlock(src_rq, &src_rf);
		local_irq_restore(src_rf.flags);
	} else {
		move_task(this_rq, src_rq, &src_rf);
	}
}

static void active_pull_global_candidate(struct rq *dist_rq)
{
	struct cfs_rq *cfs_rq = &dist_rq->cfs;
	u64 cand_est = READ_ONCE(global_candidate.est);
	u64 local_est = READ_ONCE(cfs_rq->local_cand_est);
	struct rq *src_rq;
	struct task_struct *p;
	struct rq_flags rf, src_rf;
	struct bs_node *cand;

	cand = READ_ONCE(global_candidate.candidate);

	if (!cand)
		return;

	if ((s64)(local_est - cand_est) <= 0)
		return;

	src_rq = READ_ONCE(global_candidate.rq);
	if (!src_rq || src_rq == dist_rq)
		return;

	rq_lock_irqsave(src_rq, &src_rf);
	update_rq_clock(src_rq);
		raw_spin_lock(&global_candidate.lock);
			cand = global_candidate.candidate;
			cand_est = global_candidate.est;

			if (!cand)
				goto fail_unlock;

			p = task_of(se_of(cand));
			if (task_rq(p) != src_rq ||
			    !can_migrate_task(p, dist_rq, src_rq))
				goto fail_unlock;

			if ((s64)(local_est - cand_est) <= 0)
				goto fail_unlock;

			global_candidate.rq = NULL;
			global_candidate.candidate = NULL;
			global_candidate.est = MAX_EST;
		raw_spin_unlock(&global_candidate.lock);

		// detach task
		deactivate_task(src_rq, p, DEQUEUE_NOCLOCK);
		set_task_cpu(p, cpu_of(dist_rq));
	// unlock src rq
	rq_unlock(src_rq, &src_rf);

	// lock dist rq
	rq_lock(dist_rq, &rf);
	update_rq_clock(dist_rq);
		activate_task(dist_rq, p, ENQUEUE_NOCLOCK);
		wakeup_preempt(dist_rq, p, 0);
	// unlock dist rq
	rq_unlock(dist_rq, &rf);

	local_irq_restore(src_rf.flags);

	// printk(KERN_INFO "active_pull_global_candidate");
	return;

fail_unlock:
	raw_spin_unlock(&global_candidate.lock);
	rq_unlock(src_rq, &src_rf);
	local_irq_restore(src_rf.flags);
}

static void nohz_try_pull_from_candidate(void)
{
	int cpu;
	struct rq *rq;
	struct cfs_rq *cfs_rq;
#ifdef CONFIG_NO_HZ_FULL
	struct rq_flags rf;
#endif

	/* first, push to grq*/
	for_each_online_cpu(cpu) {
		rq = cpu_rq(cpu);
#ifdef CONFIG_NO_HZ_FULL
		cfs_rq = &rq->cfs;

		if (idle_cpu(cpu) || cfs_rq->nr_running > 1)
			goto out;

		rq_lock_irqsave(rq, &rf);
		update_rq_clock(rq);
		update_curr(cfs_rq);
		rq_unlock_irqrestore(rq, &rf);
out:
#endif
		if (idle_cpu(cpu) || !sched_fair_runnable(rq))
			idle_pull_global_candidate(rq);
		else
			active_pull_global_candidate(rq);
	}
}

static int newidle_balance(struct rq *this_rq, struct rq_flags *rf)
{
	int this_cpu = this_rq->cpu;
	struct rq *src_rq;
	int src_cpu = -1, cpu;
	int pulled_task = 0;
	unsigned int max = 0;
	struct rq_flags src_rf;

	update_misfit_status(NULL, this_rq);

	/*
	 * There is a task waiting to run. No need to search for one.
	 * Return 0; the task will be enqueued when switching to idle.
	 */
	if (this_rq->ttwu_pending)
		return 0;

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

	update_blocked_averages(this_cpu);

	pulled_task = idle_pull_global_candidate(this_rq);
	if (pulled_task)
		goto out;

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
	int cpu;
	unsigned int max, min;
	struct rq *max_rq, *min_rq, *c_rq;
	struct rq_flags src_rf;

	update_blocked_averages(this_rq->cpu);

again:
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

	if (min_rq == max_rq || max - min <= 1)
		return;

	rq_lock_irqsave(max_rq, &src_rf);
	update_rq_clock(max_rq);

	if (max_rq->nr_running <= 1) {
		rq_unlock(max_rq, &src_rf);
		local_irq_restore(src_rf.flags);
		return;
	}

	if(move_task(min_rq, max_rq, &src_rf))
		goto again;
}

static void nohz_balancer_kick(struct rq *rq);

void trigger_load_balance(struct rq *this_rq)
{
	int this_cpu = cpu_of(this_rq);

	if (this_cpu != 0)
		goto out;

	nohz_try_pull_from_candidate();

	rebalance(this_rq);

out:
	if (time_after_eq(jiffies, this_rq->next_balance)) {
		this_rq->next_balance = jiffies + msecs_to_jiffies(19);
		update_blocked_averages(this_rq->cpu);
	}

	nohz_balancer_kick(this_rq);
}

#include "nohz.h"

void update_group_capacity(struct sched_domain *sd, int cpu) {}
#endif /* CONFIG_SMP */
