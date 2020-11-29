/**
 * litmus/sched_mc_ce.c
 *
 * The Cyclic Executive (CE) scheduler used by the mixed criticality scheduling
 * algorithm.
 */

#include <asm/atomic.h>
#include <asm/uaccess.h>

#include <linux/module.h>
#include <linux/percpu.h>
#include <linux/hrtimer.h>
#include <linux/pid.h>
#include <linux/sched.h>
#include <linux/proc_fs.h>

#include <litmus/litmus.h>
#include <litmus/sched_plugin.h>
#include <litmus/rt_domain.h>
#include <litmus/rt_param.h>
#include <litmus/litmus_proc.h>
#include <litmus/sched_trace.h>
#include <litmus/jobs.h>
#include <litmus/sched_mc.h>
#include <litmus/ce_domain.h>

static struct sched_plugin mc_ce_plugin __cacheline_aligned_in_smp;

#define using_linux_plugin() (litmus == &linux_sched_plugin)

/* get a reference to struct domain for a CPU */
#define get_domain_for(cpu) (&per_cpu(domains, cpu)->domain)

#define get_pid_table(cpu) (&per_cpu(ce_pid_table, cpu))
#define get_pid_entry(cpu, idx) (&(get_pid_table(cpu)->entries[idx]))

static atomic_t start_time_set = ATOMIC_INIT(-1);
static atomic64_t start_time = ATOMIC64_INIT(0);
static struct proc_dir_entry *mc_ce_dir = NULL, *ce_file = NULL;

/*
 * Cache the budget along with the struct PID for a task so that we don't need
 * to fetch its task_struct every time we check to see what should be
 * scheduled.
 */
struct ce_pid_entry {
	struct pid *pid;
	lt_t budget;
	/* accumulated (summed) budgets, including this one */
	lt_t acc_time;
	unsigned int expected_job;
};

/*
 * Each CPU needs a mapping of level A ID (integer) to struct pid so that we
 * can get its task struct.
 */
struct ce_pid_table {
	struct ce_pid_entry entries[CONFIG_PLUGIN_MC_LEVEL_A_MAX_TASKS];
	int num_pid_entries;
	lt_t cycle_time;
};

DEFINE_PER_CPU(struct ce_pid_table, ce_pid_table);

/*
 * How we get the domain for a given CPU locally. Set with the
 * mc_ce_set_domains function. Must be done before activating plugins. Be
 * careful when using domains as a variable elsewhere in this file.
 */

DEFINE_PER_CPU(struct domain_data*, domains);

/*
 * The domains and other data used by the MC-CE plugin when it runs alone.
 */
DEFINE_PER_CPU(struct domain_data, _mc_ce_doms);
DEFINE_PER_CPU(struct ce_dom_data, _mc_ce_dom_data);
DEFINE_PER_CPU(raw_spinlock_t, _mc_ce_dom_locks);

#ifdef CONFIG_PLUGIN_MC_RELEASE_MASTER
static int interrupt_cpu;
#endif

long mc_ce_set_domains(const int n, struct domain_data *domains_in[])
{
	const int max = (NR_CPUS < n) ? NR_CPUS : n;
	struct domain_data *new_dom = NULL;
	int i, ret;
	if (!using_linux_plugin()) {
		printk(KERN_WARNING "can't set MC-CE domains when not using "
				"Linux scheduler.\n");
		ret = -EINVAL;
		goto out;
	}
	for (i = 0; i < max; ++i) {
		new_dom = domains_in[i];
		per_cpu(domains, i) = new_dom;
	}
	ret = 0;
out:
	return ret;
}

unsigned int mc_ce_get_expected_job(const int cpu, const int idx)
{
	const struct ce_pid_table *pid_table = get_pid_table(cpu);
	BUG_ON(0 > cpu);
	BUG_ON(0 > idx);
	BUG_ON(pid_table->num_pid_entries <= idx);
	return pid_table->entries[idx].expected_job;
}

/*
 * Get the offset into the cycle taking the start time into account.
 */
static inline lt_t get_cycle_offset(const lt_t when, const lt_t cycle_time)
{
	lt_t remaining = -1;
	long long st = atomic64_read(&start_time);
        long long w_s = when - st;
        
        while (remaining >= 0 && remaining <= cycle_time) {
        	remaining = w_s - cycle_time;
                w_s -= cycle_time;
        }
	lt_t offset = remaining;
	TRACE("when: %llu  cycle_time: %llu start_time: %lld  offset %llu\n",
			when, cycle_time, st, offset);
	return offset;
}

/*
 * The user land job completion call will set the RT_F_SLEEP flag and then
 * call schedule. This function is used when schedule sleeps a task.
 *
 * Do not call prepare_for_next_period on Level-A tasks!
 */
static void mc_ce_job_completion(struct domain *dom, struct task_struct *ts)
{
	const int cpu = task_cpu(ts);
	const int idx = tsk_mc_data(ts)->mc_task.lvl_a_id;
	const struct ce_pid_entry *pid_entry = get_pid_entry(cpu, idx);
	unsigned int just_finished;

	TRACE_TASK(ts, "Completed\n");

	sched_trace_task_completion(ts, 0);
	/* post-increment is important here */
	just_finished = (tsk_rt(ts)->job_params.job_no)++;

	/* Job completes in expected window: everything is normal.
	 * Job completes in an earlier window: BUG(), that's wrong.
	 * Job completes in a later window: The job is behind.
	 */
	if (just_finished < pid_entry->expected_job) {
		/* this job is already released because it's running behind */
		set_rt_flags(ts, RT_F_RUNNING);
		TRACE_TASK(ts, "appears behind: the expected job is %u but "
				"job %u just completed\n",
				pid_entry->expected_job, just_finished);
	} else if (pid_entry->expected_job < just_finished) {
		printk(KERN_CRIT "job %u completed in expected job %u which "
				"seems too early\n", just_finished,
				pid_entry->expected_job);
		BUG();
	}
}


/*
 * Return the index into the PID entries table of what to schedule next.
 * Don't call if the table is empty. Assumes the caller has the domain lock.
 * The offset parameter is the offset into the cycle.
 *
 * TODO Currently O(n) in the number of tasks on the CPU. Binary search?
 */
static int mc_ce_schedule_at(const struct domain *dom, lt_t offset)
{
	const struct ce_dom_data *ce_data = dom->data;
	struct ce_pid_table *pid_table = get_pid_table(ce_data->cpu);
	const struct ce_pid_entry *pid_entry = NULL;
	int idx;

	BUG_ON(pid_table->cycle_time < 1);
	BUG_ON(pid_table->num_pid_entries < 1);

	for (idx = 0; idx < pid_table->num_pid_entries; ++idx) {
		pid_entry = &pid_table->entries[idx];
		if (offset < pid_entry->acc_time) {
			/* found task to schedule in this window */
			break;
		}
	}
	/* can only happen if cycle_time is not right */
	BUG_ON(pid_entry->acc_time > pid_table->cycle_time);
	TRACE("schedule at returning task %d for CPU %d\n", idx, ce_data->cpu);
	return idx;
}

static struct task_struct *mc_ce_schedule(struct task_struct *prev)
{
	struct domain *dom = get_domain_for(smp_processor_id());
	struct ce_dom_data *ce_data = dom->data;
	struct task_struct *next = NULL;
	int exists, sleep, should_sched_exists, should_sched_blocked,
	    should_sched_asleep;

	raw_spin_lock(dom->lock);

	/* sanity checking */
	BUG_ON(ce_data->scheduled && ce_data->scheduled != prev);
	BUG_ON(ce_data->scheduled && !is_realtime(prev));
	BUG_ON(is_realtime(prev) && !ce_data->scheduled);

	exists = NULL != ce_data->scheduled;
	sleep = exists && RT_F_SLEEP == get_rt_flags(ce_data->scheduled);

	TRACE("exists: %d, sleep: %d\n", exists, sleep);

	if (sleep)
		mc_ce_job_completion(dom, ce_data->scheduled);

	/* these checks must go after the call to mc_ce_job_completion in case
	 * a late task needs to be scheduled again right away and its the only
	 * task on a core
	 */
	should_sched_exists = NULL != ce_data->should_schedule;
	should_sched_blocked = should_sched_exists &&
		!is_running(ce_data->should_schedule);
	should_sched_asleep = should_sched_exists &&
		RT_F_SLEEP == get_rt_flags(ce_data->should_schedule);

	TRACE("should_sched_exists: %d, should_sched_blocked: %d, "
			"should_sched_asleep: %d\n", should_sched_exists,
			should_sched_blocked, should_sched_asleep);

	if (should_sched_exists && !should_sched_blocked &&
			!should_sched_asleep) {
		/*
		 * schedule the task that should be executing in the cyclic
		 * schedule if it is not blocked and not sleeping
		 */
		next = ce_data->should_schedule;
	}
	sched_state_task_picked();
	raw_spin_unlock(dom->lock);
	return next;
}

static void mc_ce_finish_switch(struct task_struct *prev)
{
	struct domain *dom = get_domain_for(smp_processor_id());
	struct ce_dom_data *ce_data = dom->data;

	TRACE("finish switch\n");

	if (is_realtime(current) && CRIT_LEVEL_A == tsk_mc_crit(current))
		ce_data->scheduled = current;
	else
		ce_data->scheduled = NULL;
}

/*
 * Admit task called to see if this task is permitted to enter the system.
 * Here we look up the task's PID structure and save it in the proper slot on
 * the CPU this task will run on.
 */
long mc_ce_admit_task_common(struct task_struct *ts)
{
	struct domain *dom = get_domain_for(get_partition(ts));
	struct ce_dom_data *ce_data = dom->data;
	struct mc_data *mcd = tsk_mc_data(ts);
	struct pid *pid = NULL;
	long retval = -EINVAL;
	const int lvl_a_id = mcd->mc_task.lvl_a_id;
	struct ce_pid_table *pid_table = get_pid_table(ce_data->cpu);

	BUG_ON(get_partition(ts) != ce_data->cpu);

	/* check the task has migrated to the right CPU (like in sched_cedf) */
	if (task_cpu(ts) != get_partition(ts)) {
		printk(KERN_INFO "litmus: %d admitted on CPU %d but want %d ",
				ts->pid, task_cpu(ts), get_partition(ts));
		goto out;
	}

	/* only level A tasks can be CE */
	if (!mcd || CRIT_LEVEL_A != tsk_mc_crit(ts)) {
		printk(KERN_INFO "litmus: non-MC or non level A task %d\n",
				ts->pid);
		goto out;
	}

	/* try and get the task's PID structure */
	pid = get_task_pid(ts, PIDTYPE_PID);
	if (IS_ERR_OR_NULL(pid)) {
		printk(KERN_INFO "litmus: couldn't get pid struct for %d\n",
				ts->pid);
		goto out;
	}

	if (lvl_a_id >= pid_table->num_pid_entries) {
		printk(KERN_INFO "litmus: level A id greater than expected "
				"number of tasks %d for %d cpu %d\n",
				pid_table->num_pid_entries, ts->pid,
				get_partition(ts));
		goto out_put_pid;
	}
	if (pid_table->entries[lvl_a_id].pid) {
		printk(KERN_INFO "litmus: have saved pid info id: %d cpu: %d\n",
				lvl_a_id, get_partition(ts));
		goto out_put_pid;
	}
	if (get_exec_cost(ts) >= pid_table->entries[lvl_a_id].budget) {
		printk(KERN_INFO "litmus: execution cost %llu is larger than "
				"the budget %llu\n",
				get_exec_cost(ts),
				pid_table->entries[lvl_a_id].budget);
		goto out_put_pid;
	}
	pid_table->entries[lvl_a_id].pid = pid;
	retval = 0;
	/* don't call put_pid if we are successful */
	goto out;

out_put_pid:
	put_pid(pid);
out:
	return retval;
}

static long mc_ce_admit_task(struct task_struct *ts)
{
	struct domain *dom = get_domain_for(get_partition(ts));
	unsigned long flags, retval;
	raw_spin_lock_irqsave(dom->lock, flags);
	retval = mc_ce_admit_task_common(ts);
	raw_spin_unlock_irqrestore(dom->lock, flags);
	return retval;
}

/*
 * Called to set up a new real-time task (after the admit_task callback).
 * At this point the task's struct PID is already hooked up on the destination
 * CPU. The task may already be running.
 */
static void mc_ce_task_new(struct task_struct *ts, int on_rq, int running)
{
	const int cpu = task_cpu(ts);
	struct domain *dom = get_domain_for(cpu);
	struct ce_dom_data *ce_data = dom->data;
	struct ce_pid_table *pid_table = get_pid_table(cpu);
	struct pid *pid_should_be_running;
	struct ce_pid_entry *pid_entry;
	unsigned long flags;
	int idx, should_be_running;
	lt_t offset;

	raw_spin_lock_irqsave(dom->lock, flags);
	pid_entry = get_pid_entry(cpu, tsk_mc_data(ts)->mc_task.lvl_a_id);
	/* initialize some task state */
	set_rt_flags(ts, RT_F_RUNNING);

	/* have to call mc_ce_schedule_at because the task only gets a PID
	 * entry after calling admit_task */
	offset = get_cycle_offset(litmus_clock(), pid_table->cycle_time);
	idx = mc_ce_schedule_at(dom, offset);
	pid_should_be_running = get_pid_entry(cpu, idx)->pid;
	rcu_read_lock();
	should_be_running = (ts == pid_task(pid_should_be_running, PIDTYPE_PID));
	rcu_read_unlock();
	if (running) {
		/* admit task checks that the task is not on the wrong CPU */
		BUG_ON(task_cpu(ts) != get_partition(ts));
		BUG_ON(ce_data->scheduled);
		ce_data->scheduled = ts;

		if (should_be_running)
			ce_data->should_schedule = ts;
		else
			preempt_if_preemptable(ce_data->scheduled, ce_data->cpu);
	} else if (!running && should_be_running) {
		ce_data->should_schedule = ts;
		preempt_if_preemptable(ce_data->scheduled, ce_data->cpu);
	}
	raw_spin_unlock_irqrestore(dom->lock, flags);
}

/*
 * Called to re-introduce a task after blocking.
 * Can potentailly be called multiple times.
 */
static void mc_ce_task_wake_up(struct task_struct *ts)
{
	struct domain *dom = get_domain_for(get_partition(ts));
	struct ce_dom_data *ce_data = dom->data;
	unsigned long flags;

	TRACE_TASK(ts, "wake up\n");

	raw_spin_lock_irqsave(dom->lock, flags);
	if (ts == ce_data->should_schedule && ts != ce_data->scheduled)
		preempt_if_preemptable(ts, ce_data->cpu);
	raw_spin_unlock_irqrestore(dom->lock, flags);
}

/*
 * Called to notify the plugin of a blocking real-time tasks. Only called for
 * real-time tasks and before schedule is called.
 */
static void mc_ce_task_block(struct task_struct *ts)
{
	/* nothing to do because it will be taken care of in schedule */
	TRACE_TASK(ts, "blocked\n");
}

/*
 * Called when a task switches from RT mode back to normal mode.
 */
void mc_ce_task_exit_common(struct task_struct *ts)
{
	struct domain *dom = get_domain_for(get_partition(ts));
	struct ce_dom_data *ce_data = dom->data;
	unsigned long flags;
	struct pid *pid;
	const int lvl_a_id = tsk_mc_data(ts)->mc_task.lvl_a_id;
	struct ce_pid_table *pid_table = get_pid_table(ce_data->cpu);

	BUG_ON(CRIT_LEVEL_A != tsk_mc_crit(ts));
	BUG_ON(lvl_a_id >= pid_table->num_pid_entries);

	raw_spin_lock_irqsave(dom->lock, flags);
	pid = pid_table->entries[lvl_a_id].pid;
	BUG_ON(!pid);
	put_pid(pid);
	pid_table->entries[lvl_a_id].pid = NULL;
	if (ce_data->scheduled == ts)
		ce_data->scheduled = NULL;
	if (ce_data->should_schedule == ts)
		ce_data->should_schedule = NULL;
	raw_spin_unlock_irqrestore(dom->lock, flags);
}

/***********************************************************
 * Timer stuff
 **********************************************************/

/*
 * Returns the next absolute time that the timer should fire.
 */
lt_t mc_ce_timer_callback_common(struct domain *dom)
{
	/* relative and absolute times for cycles */
	lt_t now, offset_rel, cycle_start_abs, next_timer_abs;
	struct task_struct *should_schedule;
	struct ce_pid_table *pid_table;
	struct ce_pid_entry *pid_entry;
	struct ce_dom_data *ce_data;
	int idx, budget_overrun;

	ce_data = dom->data;
	pid_table = get_pid_table(ce_data->cpu);

	/* Based off of the current time, figure out the offset into the cycle
	 * and the cycle's start time, and determine what should be scheduled.
	 */
	now = litmus_clock();
	offset_rel = get_cycle_offset(now, pid_table->cycle_time);
	cycle_start_abs = now - offset_rel;
	idx = mc_ce_schedule_at(dom, offset_rel);
	pid_entry = get_pid_entry(ce_data->cpu, idx);
	next_timer_abs = cycle_start_abs + pid_entry->acc_time;

	STRACE("timer: now: %llu  offset_rel: %llu  cycle_start_abs: %llu  "
			"next_timer_abs: %llu\n", now, offset_rel,
			cycle_start_abs, next_timer_abs);

	/* get the task_struct (pid_task can accept a NULL) */
	rcu_read_lock();
	should_schedule = pid_task(pid_entry->pid, PIDTYPE_PID);
	rcu_read_unlock();
	ce_data->should_schedule = should_schedule;

	if (should_schedule && 0 == atomic_read(&start_time_set)) {
		/*
		 * If jobs are not overrunning their budgets, then this
		 * should not happen.
		 */
		pid_entry->expected_job++;
		budget_overrun = pid_entry->expected_job !=
			tsk_rt(should_schedule)->job_params.job_no;
		if (budget_overrun)
			TRACE_MC_TASK(should_schedule,
				      "timer expected job number: %u "
				      "but current job: %u\n",
				      pid_entry->expected_job,
				      tsk_rt(should_schedule)->job_params.job_no);
	}

	if (ce_data->should_schedule) {
		tsk_rt(should_schedule)->job_params.deadline =
			cycle_start_abs + pid_entry->acc_time;
		tsk_rt(should_schedule)->job_params.release =
			tsk_rt(should_schedule)->job_params.deadline -
			pid_entry->budget;
		tsk_rt(should_schedule)->job_params.exec_time = 0;
		sched_trace_task_release(should_schedule);
		set_rt_flags(ce_data->should_schedule, RT_F_RUNNING);
	}
	return next_timer_abs;
}

/*
 * What to do when a timer fires. The timer should only be armed if the number
 * of PID entries is positive.
 */
#ifdef CONFIG_MERGE_TIMERS
static void mc_ce_timer_callback(struct rt_event *e)
#else
static enum hrtimer_restart mc_ce_timer_callback(struct hrtimer *timer)
#endif
{
	struct ce_dom_data *ce_data;
	unsigned long flags;
	struct domain *dom;
	lt_t next_timer_abs;
#ifdef CONFIG_MERGE_TIMERS
	struct event_group *event_group;
	ce_data = container_of(e, struct ce_dom_data, event);
	/* use the same CPU the callbacking is executing on by passing NO_CPU */
	event_group = get_event_group_for(NO_CPU);
#else /* CONFIG_MERGE_TIMERS */
	ce_data = container_of(timer, struct ce_dom_data, timer);
#endif
	dom = get_domain_for(ce_data->cpu);

	TRACE("timer callback on CPU %d (before lock)\n", ce_data->cpu);

	raw_spin_lock_irqsave(dom->lock, flags);
	next_timer_abs = mc_ce_timer_callback_common(dom);

	/* setup an event or timer for the next release in the CE schedule */
#ifdef CONFIG_MERGE_TIMERS
	add_event(event_group, e, next_timer_abs);
#else
	hrtimer_set_expires(timer, ns_to_ktime(next_timer_abs));
#endif

	if (ce_data->scheduled != ce_data->should_schedule)
		preempt_if_preemptable(ce_data->scheduled, ce_data->cpu);

	raw_spin_unlock_irqrestore(dom->lock, flags);

#ifndef CONFIG_MERGE_TIMERS
	return HRTIMER_RESTART;
#endif
}

/*
 * Cancel timers on all CPUs. Returns 1 if any were active.
 */
static int cancel_all_timers(void)
{
	struct ce_dom_data *ce_data;
	struct domain *dom;
	int cpu, ret = 0;
#ifndef CONFIG_MERGE_TIMERS
	int cancel_res;
#endif

	TRACE("cancel all timers\n");

	for_each_online_cpu(cpu) {
		dom = get_domain_for(cpu);
		ce_data = dom->data;
		ce_data->should_schedule = NULL;
#ifdef CONFIG_MERGE_TIMERS
		cancel_event(&ce_data->event);
#else
		cancel_res = hrtimer_cancel(&ce_data->timer);
		atomic_set(&ce_data->timer_info.state,
				HRTIMER_START_ON_INACTIVE);
		ret = ret || cancel_res;
#endif
	}
	return ret;
}

/*
 * Arm all timers so that they start at the new value of start time.
 * Any CPU without CE PID entries won't have a timer armed.
 * All timers should be canceled before calling this.
 */
static void arm_all_timers(void)
{
	struct domain *dom;
	struct ce_dom_data *ce_data;
	struct ce_pid_table *pid_table;
	int cpu, idx, cpu_for_timer;
	const lt_t start = atomic64_read(&start_time);

	TRACE("arm all timers\n");

	for_each_online_cpu(cpu) {
		dom = get_domain_for(cpu);
		ce_data = dom->data;
		pid_table = get_pid_table(cpu);
		if (0 == pid_table->num_pid_entries)
			continue;
		for (idx = 0; idx < pid_table->num_pid_entries; idx++) {
			pid_table->entries[idx].expected_job = 0;
		}
#ifdef CONFIG_PLUGIN_MC_RELEASE_MASTER
		cpu_for_timer = interrupt_cpu;
#else
		cpu_for_timer = cpu;
#endif

#ifdef CONFIG_MERGE_TIMERS
		add_event(get_event_group_for(cpu_for_timer),
				&ce_data->event, start);
#else
		hrtimer_start_on(cpu_for_timer, &ce_data->timer_info,
				&ce_data->timer, ns_to_ktime(start),
				HRTIMER_MODE_ABS_PINNED);
#endif
	}
}

/*
 * There are no real releases in the CE, but the task release syscall will
 * call this. We can re-set our notion of the CE period start to make
 * the schedule look pretty.
 */
void mc_ce_release_at_common(struct task_struct *ts, lt_t start)
{
	TRACE_TASK(ts, "release at\n");
	if (atomic_inc_and_test(&start_time_set)) {
		/* in this case, we won the race */
		cancel_all_timers();
		atomic64_set(&start_time, start);
		arm_all_timers();
	} else
		atomic_dec(&start_time_set);
}

long mc_ce_activate_plugin_common(void)
{
	struct ce_dom_data *ce_data;
	struct domain *dom;
	long ret;
	int cpu;

#ifdef CONFIG_PLUGIN_MC_RELEASE_MASTER
	interrupt_cpu = atomic_read(&release_master_cpu);
	if (NO_CPU == interrupt_cpu) {
		printk(KERN_ERR "LITMUS: MC-CE needs a release master\n");
		ret = -EINVAL;
		goto out;
	}
#endif

	for_each_online_cpu(cpu) {
		dom = get_domain_for(cpu);
		ce_data = dom->data;
		ce_data->scheduled = NULL;
		ce_data->should_schedule = NULL;
	}

	atomic_set(&start_time_set, -1);
	atomic64_set(&start_time, litmus_clock());
	/* may not want to arm timers on activation, just after release */
	arm_all_timers();
	ret = 0;
out:
	return ret;
}

static long mc_ce_activate_plugin(void)
{
	struct domain_data *our_domains[NR_CPUS];
	int cpu, n = 0;
	long ret;

	for_each_online_cpu(cpu) {
		BUG_ON(NR_CPUS <= n);
		our_domains[cpu] = &per_cpu(_mc_ce_doms, cpu);
		n++;
	}
	ret = mc_ce_set_domains(n, our_domains);
	if (ret)
		goto out;
	ret = mc_ce_activate_plugin_common();
out:
	return ret;
}

static void clear_pid_entries(void)
{
	struct ce_pid_table *pid_table = NULL;
	int cpu, entry;

	for_each_online_cpu(cpu) {
		pid_table = get_pid_table(cpu);
		pid_table->num_pid_entries = 0;
		pid_table->cycle_time = 0;
		for (entry = 0; entry < CONFIG_PLUGIN_MC_LEVEL_A_MAX_TASKS;
				++entry) {
			if (NULL != pid_table->entries[entry].pid) {
				put_pid(pid_table->entries[entry].pid);
				pid_table->entries[entry].pid = NULL;
			}
			pid_table->entries[entry].budget = 0;
			pid_table->entries[entry].acc_time = 0;
			pid_table->entries[entry].expected_job = 0;
		}
	}
}

long mc_ce_deactivate_plugin_common(void)
{
	int cpu;
	cancel_all_timers();
	for_each_online_cpu(cpu) {
		per_cpu(domains, cpu) = NULL;
	}
	return 0;
}

/*	Plugin object	*/
static struct sched_plugin mc_ce_plugin __cacheline_aligned_in_smp = {
	.plugin_name		= "MC-CE",
	.admit_task		= mc_ce_admit_task,
	.task_new		= mc_ce_task_new,
	.complete_job		= complete_job,
	.release_at		= mc_ce_release_at_common,
	.task_exit		= mc_ce_task_exit_common,
	.schedule		= mc_ce_schedule,
	.finish_switch		= mc_ce_finish_switch,
	.task_wake_up		= mc_ce_task_wake_up,
	.task_block		= mc_ce_task_block,
	.activate_plugin	= mc_ce_activate_plugin,
	.deactivate_plugin	= mc_ce_deactivate_plugin_common,
};

static int setup_proc(void);
static int __init init_sched_mc_ce(void)
{
	raw_spinlock_t *ce_lock;
	struct domain_data *dom_data;
	struct domain *dom;
	int cpu, err;

	for_each_online_cpu(cpu) {
		per_cpu(domains, cpu) = NULL;
		ce_lock = &per_cpu(_mc_ce_dom_locks, cpu);
		raw_spin_lock_init(ce_lock);
		dom_data = &per_cpu(_mc_ce_doms, cpu);
		dom = &dom_data->domain;
		ce_domain_init(dom, ce_lock, NULL, NULL, NULL, NULL, NULL,
				&per_cpu(_mc_ce_dom_data, cpu), cpu,
				mc_ce_timer_callback);
	}
	clear_pid_entries();
	err = setup_proc();
	if (!err)
		err = register_sched_plugin(&mc_ce_plugin);
	return err;
}

#define BUF_SIZE PAGE_SIZE
static int write_into_proc(char *proc_buf, const int proc_size, char *fmt, ...)
{
	static char buf[BUF_SIZE];
	int n;
	va_list args;

	/* When writing to procfs, we don't care about the trailing null that
	 * is not included in the count returned by vscnprintf.
	 */
	va_start(args, fmt);
	n = vsnprintf(buf, BUF_SIZE, fmt, args);
	va_end(args);
	if (BUF_SIZE <= n || proc_size <= n) {
		/* too big for formatting buffer or proc (less null byte) */
		n = -EINVAL;
		goto out;
	}
	memcpy(proc_buf, buf, n);
out:
	return n;
}
#undef BUF_SIZE

/*
 * Writes a PID entry to the procfs.
 *
 * @page buffer to write into.
 * @count bytes available in the buffer
 */
#define PID_SPACE 15
#define TASK_INFO_BUF (PID_SPACE + TASK_COMM_LEN)
static int write_pid_entry(char *page, const int count, const int cpu,
		const int task, struct ce_pid_entry *pid_entry)
{
	static char task_info[TASK_INFO_BUF];
	struct task_struct *ts;
	int n = 0, err, ti_n;
	char *ti_b;

	if (pid_entry->pid) {
		rcu_read_lock();
		ts = pid_task(pid_entry->pid, PIDTYPE_PID);
		rcu_read_unlock();

		/* get some information about the task */
		if (ts) {
			ti_b = task_info;
			ti_n = snprintf(ti_b, PID_SPACE, "%d", ts->pid);
			if (PID_SPACE <= ti_n)
				ti_n = PID_SPACE - 1;
			ti_b += ti_n;
			*ti_b = ' '; /* nuke the null byte */
			ti_b++;
			get_task_comm(ti_b, ts);
		} else {
			strncpy(task_info, "pid_task() failed :(",
					TASK_INFO_BUF);
		}

	} else
		strncpy(task_info, "no", TASK_INFO_BUF);
	task_info[TASK_INFO_BUF - 1] = '\0'; /* just to be sure */

	err = write_into_proc(page + n, count - n, "# task: %s\n", task_info);
	if (err < 0) {
		n = -ENOSPC;
		goto out;
	}
	n += err;
	err = write_into_proc(page + n, count - n, "%d, %d, %llu\n",
			cpu, task, pid_entry->budget);
	if (err < 0) {
		n = -ENOSPC;
		goto out;
	}
	n += err;
out:
	return n;
}
#undef PID_SPACE
#undef TASK_INFO_BUF

/*
 * Called when the user-land reads from proc.
 */
static int proc_read_ce_file(char *page, char **start, off_t off, int count,
		int *eof, void *data)
{
	int n = 0, err, cpu, t;
	struct ce_pid_table *pid_table;

	if (off > 0) {
		printk(KERN_INFO "litmus: MC-CE called read with off > 0\n");
		goto out;
	}

	for_each_online_cpu(cpu) {
		pid_table = get_pid_table(cpu);
		for (t = 0; t < pid_table->num_pid_entries; ++t) {
			err = write_pid_entry(page + n, count - n,
					cpu, t, get_pid_entry(cpu, t));
			if (err < 0) {
				n = -ENOSPC;
				goto out;
			}
			n += err;
		}
	}
out:
	*eof = 1;
	return n;
}

/*
 * Skip a commented line.
 */
static int skip_comment(const char *buf, const unsigned long max)
{
	unsigned long i = 0;
	const char *c = buf;
	if (0 == max || !c || *c != '#')
		return 0;
	++c; ++i;
	for (; i < max; ++i) {
		if (*c == '\n') {
			++c; ++i;
			break;
		}
		++c;
	}
	return i;
}

/* a budget of 5 milliseconds is probably reasonable */
#define BUDGET_THRESHOLD 5000000ULL
static int setup_pid_entry(const int cpu, const int task, const lt_t budget)
{
	struct ce_pid_table *pid_table = get_pid_table(cpu);
	struct ce_pid_entry *new_entry = NULL;
	int err = 0;

	/* check the inputs */
	if (cpu < 0 || NR_CPUS <= cpu || task < 0 ||
			CONFIG_PLUGIN_MC_LEVEL_A_MAX_TASKS <= task ||
			budget < 1) {
		printk(KERN_INFO "litmus: bad cpu, task ID, or budget sent to "
				"MC-CE proc\n");
		err = -EINVAL;
		goto out;
	}
	/* check for small budgets */
	if (BUDGET_THRESHOLD > budget) {
		printk(KERN_CRIT "litmus: you gave a small budget for an "
				"MC-CE task; that might be an issue.\n");
	}
	/* check that we have space for a new entry */
	if (CONFIG_PLUGIN_MC_LEVEL_A_MAX_TASKS <= pid_table->num_pid_entries) {
		printk(KERN_INFO "litmus: too many MC-CE tasks for cpu "
				"%d\n", cpu);
		err = -EINVAL;
		goto out;
	}
	/* add the new entry */
	new_entry = get_pid_entry(cpu, pid_table->num_pid_entries);
	BUG_ON(NULL != new_entry->pid);
	new_entry->budget = budget;
	new_entry->acc_time = pid_table->cycle_time + budget;
	/* update the domain entry */
	pid_table->cycle_time += budget;
	pid_table->num_pid_entries++;
out:
	return err;
}
#undef BUDGET_THRESHOLD

/*
 * Called when the user-land writes to proc.
 *
 * Error checking is quite minimal. Format is:
 * <cpu>, <process ID>, <budget>
 */
#define PROCFS_MAX_SIZE PAGE_SIZE
static int proc_write_ce_file(struct file *file, const char __user *buffer,
		unsigned long count, void *data)
{
	static char kbuf[PROCFS_MAX_SIZE];
	char *c = kbuf, *c_skipped;
	int cpu, task, cnt = 0, chars_read, converted, err;
	lt_t budget;

	if (!using_linux_plugin()) {
		printk(KERN_INFO "litmus: can only edit MC-CE proc under Linux "
				"plugin\n");
		cnt = -EINVAL;
		goto out;
	}

	if (count > PROCFS_MAX_SIZE) {
		printk(KERN_INFO "litmus: MC-CE procfs got too many bytes "
				"from user-space.\n");
		cnt = -EINVAL;
		goto out;
	}

	if (copy_from_user(kbuf, buffer, count)) {
		printk(KERN_INFO "litmus: couldn't copy from user %s\n",
				__FUNCTION__);
		cnt = -EFAULT;
		goto out;
	}
	clear_pid_entries();
	while (cnt < count) {
		c_skipped = skip_spaces(c);
		if (c_skipped != c) {
			chars_read = c_skipped - c;
			cnt += chars_read;
			c += chars_read;
			continue;
		}
		if (*c == '#') {
			chars_read = skip_comment(c, count - cnt);
			cnt += chars_read;
			c += chars_read;
			continue;
		}
		converted = sscanf(c, "%d, %d, %llu%n", &cpu, &task, &budget,
				&chars_read);
		if (3 != converted) {
			printk(KERN_INFO "litmus: MC-CE procfs expected three "
					"arguments, but got %d.\n", converted);
			cnt = -EINVAL;
			goto out;
		}
		cnt += chars_read;
		c += chars_read;
		err = setup_pid_entry(cpu, task, budget);
		if (err) {
			cnt = -EINVAL;
			goto out;
		}
	}
out:
	return cnt;
}
#undef PROCFS_MAX_SIZE

#define CE_FILE_PROC_NAME "ce_file"
static void tear_down_proc(void)
{
	if (ce_file)
		remove_proc_entry(CE_FILE_PROC_NAME, mc_ce_dir);
	if (mc_ce_dir)
		remove_plugin_proc_dir(&mc_ce_plugin);
}

static int setup_proc(void)
{
	int err;
	err = make_plugin_proc_dir(&mc_ce_plugin, &mc_ce_dir);
	if (err) {
		printk(KERN_ERR "could not create MC-CE procfs dir.\n");
		goto out;
	}
	ce_file = create_proc_entry(CE_FILE_PROC_NAME, 0644, mc_ce_dir);
	if (!ce_file) {
		printk(KERN_ERR "could not create MC-CE procfs file.\n");
		err = -EIO;
		goto out_remove_proc;
	}
	ce_file->read_proc = proc_read_ce_file;
	ce_file->write_proc = proc_write_ce_file;
	goto out;
out_remove_proc:
	tear_down_proc();
out:
	return err;
}
#undef CE_FILE_PROC_NAME

static void clean_sched_mc_ce(void)
{
	tear_down_proc();
}

module_init(init_sched_mc_ce);
module_exit(clean_sched_mc_ce);
