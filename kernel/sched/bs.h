
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
static int newidle_balance(struct rq *this_rq, struct rq_flags *rf);
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
