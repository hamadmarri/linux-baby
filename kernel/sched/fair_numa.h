
#ifdef CONFIG_NUMA_BALANCING
static void account_numa_enqueue(struct rq *rq, struct task_struct *p)
{
	rq->nr_numa_running += (p->numa_preferred_nid != NUMA_NO_NODE);
	rq->nr_preferred_running += (p->numa_preferred_nid == task_node(p));
}

static void account_numa_dequeue(struct rq *rq, struct task_struct *p)
{
	rq->nr_numa_running -= (p->numa_preferred_nid != NUMA_NO_NODE);
	rq->nr_preferred_running -= (p->numa_preferred_nid == task_node(p));
}

/* For sanity's sake, never scan more PTEs than MAX_SCAN_WINDOW MB/sec. */
#define MAX_SCAN_WINDOW 2560

static unsigned int task_scan_min(struct task_struct *p)
{
	unsigned int scan_size = READ_ONCE(sysctl_numa_balancing_scan_size);
	unsigned int scan, floor;
	unsigned int windows = 1;

	if (scan_size < MAX_SCAN_WINDOW)
		windows = MAX_SCAN_WINDOW / scan_size;
	floor = 1000 / windows;

	scan = sysctl_numa_balancing_scan_period_min / task_nr_scan_windows(p);
	return max_t(unsigned int, floor, scan);
}

static inline unsigned long group_faults_priv(struct numa_group *ng)
{
	unsigned long faults = 0;
	int node;

	for_each_online_node(node) {
		faults += ng->faults[task_faults_idx(NUMA_MEM, node, 1)];
	}

	return faults;
}

static inline unsigned long group_faults_shared(struct numa_group *ng)
{
	unsigned long faults = 0;
	int node;

	for_each_online_node(node) {
		faults += ng->faults[task_faults_idx(NUMA_MEM, node, 0)];
	}

	return faults;
}

static unsigned int task_scan_start(struct task_struct *p)
{
	unsigned long smin = task_scan_min(p);
	unsigned long period = smin;
	struct numa_group *ng;

	/* Scale the maximum scan period with the amount of shared memory. */
	rcu_read_lock();
	ng = rcu_dereference(p->numa_group);
	if (ng) {
		unsigned long shared = group_faults_shared(ng);
		unsigned long private = group_faults_priv(ng);

		period *= refcount_read(&ng->refcount);
		period *= shared + 1;
		period /= private + shared + 1;
	}
	rcu_read_unlock();

	return max(smin, period);
}

static void update_scan_period(struct task_struct *p, int new_cpu)
{
	int src_nid = cpu_to_node(task_cpu(p));
	int dst_nid = cpu_to_node(new_cpu);

	if (!static_branch_likely(&sched_numa_balancing))
		return;

	if (!p->mm || !p->numa_faults || (p->flags & PF_EXITING))
		return;

	if (src_nid == dst_nid)
		return;

	/*
	 * Allow resets if faults have been trapped before one scan
	 * has completed. This is most likely due to a new task that
	 * is pulled cross-node due to wakeups or load balancing.
	 */
	if (p->numa_scan_seq) {
		/*
		 * Avoid scan adjustments if moving to the preferred
		 * node or if the task was not previously running on
		 * the preferred node.
		 */
		if (dst_nid == p->numa_preferred_nid ||
		    (p->numa_preferred_nid != NUMA_NO_NODE &&
			src_nid != p->numa_preferred_nid))
			return;
	}

	p->numa_scan_period = task_scan_start(p);
}

static void task_tick_numa(struct rq *rq, struct task_struct *curr)
{
	struct callback_head *work = &curr->numa_work;
	u64 period, now;

	/*
	 * We don't care about NUMA placement if we don't have memory.
	 */
	if ((curr->flags & (PF_EXITING | PF_KTHREAD)) || work->next != work)
		return;

	/*
	 * Using runtime rather than walltime has the dual advantage that
	 * we (mostly) drive the selection from busy threads and that the
	 * task needs to have done some actual work before we bother with
	 * NUMA placement.
	 */
	now = curr->se.sum_exec_runtime;
	period = (u64)curr->numa_scan_period * NSEC_PER_MSEC;

	if (now > curr->node_stamp + period) {
		if (!curr->node_stamp)
			curr->numa_scan_period = task_scan_start(curr);
		curr->node_stamp += period;

		if (!time_before(jiffies, curr->mm->numa_next_scan))
			task_work_add(curr, work, TWA_RESUME);
	}
}
#else
static void account_numa_enqueue(struct rq *rq, struct task_struct *p) {}
static inline void account_numa_dequeue(struct rq *rq, struct task_struct *p) {}
static inline void update_scan_period(struct task_struct *p, int new_cpu) {}
static void task_tick_numa(struct rq *rq, struct task_struct *curr) {}
#endif /** CONFIG_NUMA_BALANCING */

