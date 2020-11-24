/**
 * litmus/sched_mc.c
 *
 * Implementation of the Mixed Criticality scheduling algorithm.
 *
 * (Per Mollison, Erickson, Anderson, Baruah, Scoredos 2010)
 *
 * Absolute first: relative time spent doing different parts of release
 * and scheduling overhead needs to be measured and graphed.
 *
 * Domain locks should be more fine-grained. There is no reason to hold the
 * ready-queue lock when adding a task to the release-queue.
 *
 * The levels should be converted to linked-lists so that they are more
 * adaptable and need not be identical on all processors.
 *
 * The interaction between remove_from_all and other concurrent operations
 * should be re-examined. If a job_completion and a preemption happen
 * simultaneously, a task could be requeued, removed, then requeued again.
 *
 * Level-C tasks should be able to swap CPUs a-la GSN-EDF. They should also
 * try and swap with the last CPU they were on. This could be complicated for
 * ghost tasks.
 *
 * Locking for timer-merging could be infinitely more fine-grained. A second
 * hash could select a lock to use based on queue slot. This approach might
 * also help with add_release in rt_domains.
 *
 * It should be possible to reserve a CPU for ftdumping.
 *
 * The real_deadline business seems sloppy.
 *
 * The amount of data in the header file should be cut down. The use of the
 * header file in general needs to be re-examined.
 *
 * The plugin needs to be modified so that it doesn't freeze when it is
 * deactivated in a VM.
 *
 * The locking in check_for_preempt is not fine-grained enough.
 *
 * The size of the structures could be smaller. Debugging info might be
 * excessive as things currently stand.
 *
 * The macro can_requeue has been expanded too much. Anything beyond
 * scheduled_on is a hack!
 *
 * Domain names (rt_domain) are still clumsy.
 *
 * Should BE be moved into the kernel? This will require benchmarking.
 */

#include <linux/spinlock.h>
#include <linux/percpu.h>
#include <linux/sched.h>
#include <linux/hrtimer.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/poison.h>
#include <linux/pid.h>

#include <litmus/litmus.h>
#include <litmus/trace.h>
#include <litmus/jobs.h>
#include <litmus/sched_plugin.h>
#include <litmus/edf_common.h>
#include <litmus/sched_trace.h>
#include <litmus/domain.h>
#include <litmus/bheap.h>
#include <litmus/event_group.h>
#include <litmus/budget.h>

#include <litmus/sched_mc.h>
#include <litmus/ce_domain.h>

/**
 * struct cpu_entry - State of a CPU for the entire MC system
 * @cpu		  CPU id
 * @scheduled	  Task that is physically running
 * @linked	  Task that should be running / is logically running
 * @lock	  For serialization
 * @crit_entries  Array of CPU state per criticality level
 * @redir	  List of redirected work for this CPU.
 * @redir_lock	  Lock for @redir.
 * @event_group	  Event group for timer merging.
 */
struct cpu_entry {
	int			cpu;
	struct task_struct*	scheduled;
	struct task_struct*	will_schedule;
	struct task_struct*	linked;
	raw_spinlock_t		lock;
	struct crit_entry	crit_entries[NUM_CRIT_LEVELS];
#ifdef CONFIG_PLUGIN_MC_REDIRECT
	struct list_head	redir;
	raw_spinlock_t		redir_lock;
#endif
#ifdef CONFIG_MERGE_TIMERS
	struct event_group *event_group;
#endif
};

DEFINE_PER_CPU(struct cpu_entry, cpus);
#ifdef CONFIG_RELEASE_MASTER
static int interrupt_cpu;
#endif

#define domain_data(dom)  (container_of(dom, struct domain_data, domain))
#define is_global(dom)    (domain_data(dom)->heap)
#define is_global_task(t) (is_global(get_task_domain(t)))
#define can_use(ce) \
	((ce)->state == CS_ACTIVE || (ce->state == CS_ACTIVATE))
#define can_requeue(t)							\
	((t)->rt_param.linked_on == NO_CPU && /* Not linked anywhere */ \
	 !is_queued(t) &&	              /* Not gonna be linked */ \
	 (!is_global_task(t) || (t)->rt_param.scheduled_on == NO_CPU))
#define entry_level(e) \
	(((e)->linked) ? tsk_mc_crit((e)->linked) : NUM_CRIT_LEVELS - 1)
#define crit_cpu(ce) \
	(container_of((void*)((ce) - (ce)->level), struct cpu_entry, crit_entries))
#define get_crit_entry_for(cpu, level) (&per_cpu(cpus, cpu).crit_entries[level])
#define TRACE_ENTRY(e, fmt, args...)				\
	STRACE("P%d, linked=" TS " " fmt, e->cpu, TA(e->linked), ##args)
#define TRACE_CRIT_ENTRY(ce, fmt, args...)			\
	STRACE("%s P%d, linked=" TS " " fmt,			\
	      (ce)->domain->name, crit_cpu(ce)->cpu, TA((ce)->linked), ##args)

/*
 * Sort CPUs within a global domain's heap.
 */
static int cpu_lower_prio(struct bheap_node *a, struct bheap_node *b)
{
	struct domain *domain;
	struct crit_entry *first, *second;
	struct task_struct *first_link, *second_link;

	first  = a->value;
	second = b->value;
	first_link  = first->linked;
	second_link = second->linked;

	if (first->state == CS_REMOVED || second->state == CS_REMOVED) {
		/* Removed entries go at the back of the heap */
		return first->state  != CS_REMOVED &&
		       second->state != CS_REMOVED;
	} else if (!first_link || !second_link) {
		/* Entry with nothing scheduled is lowest priority */
		return second_link && !first_link;
	} else {
		/* Sort by deadlines of tasks */
		domain = get_task_domain(first_link);
		return domain->higher_prio(second_link, first_link);
	}
}

/*
 * Return true if the domain has a higher priority ready task. The @curr
 * task must belong to the domain.
 */
static int mc_preempt_needed(struct domain *dom, struct task_struct* curr)
{
	struct task_struct *next = dom->peek_ready(dom);
	if (!next || !curr) {
		return next && !curr;
	} else {
		BUG_ON(tsk_mc_crit(next) != tsk_mc_crit(curr));
		return get_task_domain(next)->higher_prio(next, curr);
	}
}

/*
 * Update crit entry position in a global heap. Caller must hold
 * @ce's domain lock.
 */
static inline void update_crit_position(struct crit_entry *ce)
{
	struct bheap *heap;
	if (is_global(ce->domain)) {
		heap = domain_data(ce->domain)->heap;
		BUG_ON(!heap);
		BUG_ON(!bheap_node_in_heap(ce->node));
		bheap_delete(cpu_lower_prio, heap, ce->node);
		bheap_insert(cpu_lower_prio, heap, ce->node);
	}
}

/*
 * Update crit entry position in a global heap if it has been marked
 * for update. Caller must hold @ce's domain lock.
 */
static void fix_crit_position(struct crit_entry *ce)
{
	if (is_global(ce->domain)) {
		if (CS_ACTIVATE == ce->state) {
			ce->state = CS_ACTIVE;
			update_crit_position(ce);
		} else if (CS_REMOVE == ce->state) {
			ce->state = CS_REMOVED;
			update_crit_position(ce);
		}
	}
}

/*
 * Return next CPU which should preempted or NULL if the domain has no
 * preemptable CPUs. Caller must hold the @dom lock.
 */
static inline struct crit_entry* lowest_prio_cpu(struct domain *dom)
{
	struct bheap *heap = domain_data(dom)->heap;
	struct bheap_node* hn;
	struct crit_entry *ce, *res = NULL;
	do {
		hn = bheap_peek(cpu_lower_prio, heap);
		ce = (hn) ? hn->value : NULL;
		if (ce) {
			if (ce->state == CS_ACTIVE)
				res = ce;
			else if (ce->state == CS_REMOVED)
				ce = NULL;
			else
				fix_crit_position(ce);
		}
	} while (ce && !res);
	return res;
}

/*
 * Cancel ghost timer.
 */
static inline void cancel_ghost(struct crit_entry *ce)
{
#ifdef CONFIG_MERGE_TIMERS
	cancel_event(&ce->event);
#else
	hrtimer_try_to_cancel(&ce->timer);
#endif
}

/*
 * Arm ghost timer. Will merge timers if the option is specified.
 */
static inline void arm_ghost(struct crit_entry *ce, lt_t fire)
{
#ifdef CONFIG_MERGE_TIMERS
	add_event(crit_cpu(ce)->event_group, &ce->event, fire);
#else
	__hrtimer_start_range_ns(&ce->timer,
				 ns_to_ktime(fire),
				 0 /* delta */,
				 HRTIMER_MODE_ABS_PINNED,
				 0 /* no wakeup */);
#endif
}

/*
 * Time accounting for ghost tasks.
 * Must be called before a decision is made involving the task's budget.
 */
static void update_ghost_time(struct task_struct *p)
{
	u64 clock = litmus_clock();
	u64 delta = clock - p->se.exec_start;
	BUG_ON(!is_ghost(p));
	if (unlikely ((s64)delta < 0)) {
		delta = 0;
		TRACE_MC_TASK(p, "WARNING: negative time delta\n");
	}
	if (tsk_mc_data(p)->mc_job.ghost_budget <= delta) {
		TRACE_MC_TASK(p, "Ghost job could have ended\n");
		tsk_mc_data(p)->mc_job.ghost_budget = 0;
		p->se.exec_start = clock;
	} else {
		TRACE_MC_TASK(p, "Ghost job updated, but didn't finish\n");
		tsk_mc_data(p)->mc_job.ghost_budget -= delta;
		p->se.exec_start = clock;
	}
}

/**
 * link_task_to_crit() - Logically run a task at a criticality level.
 * Caller must hold @ce's CPU lock.
 */
static void link_task_to_crit(struct crit_entry *ce,
			      struct task_struct *task)
{
	lt_t when_to_fire;

	TRACE_CRIT_ENTRY(ce, "Linking " TS "\n", TA(task));
	BUG_ON(!can_use(ce) && task);
	BUG_ON(task && tsk_rt(task)->linked_on != NO_CPU);
	BUG_ON(task && is_global(ce->domain) &&
	       !bheap_node_in_heap(ce->node));

	/* Unlink last task */
	if (ce->linked) {
		TRACE_MC_TASK(ce->linked, "Unlinking\n");
		ce->linked->rt_param.linked_on = NO_CPU;
		if (is_ghost(ce->linked)) {
			cancel_ghost(ce);
			if (tsk_mc_data(ce->linked)->mc_job.ghost_budget > 0) {
				/* Job isn't finished, so do accounting */
				update_ghost_time(ce->linked);
			}
		}
	}

	/* Actually link task */
	ce->linked = task;
	if (task) {
		task->rt_param.linked_on = crit_cpu(ce)->cpu;
		if (is_ghost(task) && CRIT_LEVEL_A != tsk_mc_crit(task)) {
			/* There is a level-A timer that will force a
			 * preemption, so we don't set this for level-A
			 * tasks. Otherwise reset the budget timer.
			 */
			task->se.exec_start = litmus_clock();
			when_to_fire = task->se.exec_start +
				tsk_mc_data(task)->mc_job.ghost_budget;
			arm_ghost(ce, when_to_fire);
		}
	}
}

static void check_for_preempt(struct domain*);

/**
 * job_arrival() - Called when a task re-enters the system.
 * Caller must hold no locks.
 */
static void job_arrival(struct task_struct *task)
{
	struct domain *dom = get_task_domain(task);

	TRACE_MC_TASK(task, "Job arriving\n");
	BUG_ON(!task);

	raw_spin_lock(dom->lock);
	if (can_requeue(task)) {
		BUG_ON(task->rt_param.linked_on != NO_CPU);
		dom->requeue(dom, task);
		check_for_preempt(dom);
	} else {
		/* If a global task is scheduled on one cpu, it CANNOT
		 * be requeued into a global domain. Another cpu might
		 * dequeue the global task before it is descheduled,
		 * causing the system to crash when the task is scheduled
		 * in two places simultaneously.
		 */
		TRACE_MC_TASK(task, "Delayed arrival of scheduled task\n");
	}
	raw_spin_unlock(dom->lock);
}

/**
 * low_prio_arrival() - If CONFIG_PLUGIN_MC_REDIRECT is enabled, will
 * redirect a lower priority job_arrival work to the interrupt_cpu.
 */
static void low_prio_arrival(struct task_struct *task)
{
	struct cpu_entry *entry;

	/* Race conditions! */
	if (!can_requeue(task)) return;

#ifdef  CONFIG_PLUGIN_MC_REDIRECT
	if (!is_global_task(task))
		goto arrive;
	if (smp_processor_id() != interrupt_cpu) {
		entry = &__get_cpu_var(cpus);
		raw_spin_lock(&entry->redir_lock);
		TRACE_MC_TASK(task, "Adding to redirect queue\n");
		list_add(&tsk_rt(task)->list, &entry->redir);
		raw_spin_unlock(&entry->redir_lock);
		litmus_reschedule(interrupt_cpu);
	} else
#endif
	{
arrive:
		job_arrival(task);
	}
}

#ifdef CONFIG_PLUGIN_MC_REDIRECT
/**
 * fix_global_levels() - Execute redirected job arrivals on this cpu.
 */
static void fix_global_levels(void)
{
	int c;
	struct cpu_entry *e;
	struct list_head *pos, *safe;
	struct task_struct *t;

	STRACE("Fixing global levels\n");
	for_each_online_cpu(c) {
		e = &per_cpu(cpus, c);
		raw_spin_lock(&e->redir_lock);
		list_for_each_safe(pos, safe, &e->redir) {
			t = list_entry(pos, struct task_struct, rt_param.list);
			BUG_ON(!t);
			TRACE_MC_TASK(t, "Dequeued redirected job\n");
			list_del_init(pos);
			job_arrival(t);
		}
		raw_spin_unlock(&e->redir_lock);
	}
}
#endif

/**
 * link_task_to_cpu() - Logically run a task on a CPU.
 * The task must first have been linked to one of the CPU's crit_entries.
 * Caller must hold the entry lock.
 */
static void link_task_to_cpu(struct cpu_entry *entry, struct task_struct *task)
{
	int i = entry_level(entry);
	struct crit_entry *ce;
	TRACE_MC_TASK(task, "Linking to P%d\n", entry->cpu);
	BUG_ON(task && tsk_rt(task)->linked_on != entry->cpu);
	BUG_ON(task && is_ghost(task));

	if (task){
		set_rt_flags(task, RT_F_RUNNING);
	}
	entry->linked = task;

	/* Higher criticality crit entries are now usable */
	for (; i < entry_level(entry) + 1; i++) {
		ce = &entry->crit_entries[i];
		if (!can_use(ce)) {
			ce->state = CS_ACTIVATE;
		}
	}
}

/**
 * preempt() - Preempt a logically running task with a higher priority one.
 * @dom	Domain from which to draw higher priority task
 * @ce	CPU criticality level to preempt
 *
 * Caller must hold the lock for @dom and @ce's CPU lock.
 */
static void preempt(struct domain *dom, struct crit_entry *ce)
{
	struct task_struct *task = dom->take_ready(dom);
	struct cpu_entry *entry = crit_cpu(ce);
	struct task_struct *old = ce->linked;

	BUG_ON(!task);
	TRACE_CRIT_ENTRY(ce, "Preempted by " TS "\n", TA(task));

	/* Per-domain preemption */
	link_task_to_crit(ce, task);
	if (old && can_requeue(old)) {
		dom->requeue(dom, old);
	}
	update_crit_position(ce);

	/* Preempt actual execution if this is a running task */
	if (!is_ghost(task)) {
		link_task_to_cpu(entry, task);
		preempt_if_preemptable(entry->scheduled, entry->cpu);
	} else if (old && old == entry->linked) {
		/* Preempted a running task with a ghost job. Null needs to be
		 * running.
		 */
		link_task_to_cpu(entry, NULL);
		preempt_if_preemptable(entry->scheduled, entry->cpu);
	}
}

/**
 * update_crit_levels() - Update criticality entries for the new cpu state.
 * This should be called after a new task has been linked to @entry.
 * The caller must hold the @entry->lock, but this method will release it.
 */
static void update_crit_levels(struct cpu_entry *entry)
{
	int i, global_preempted;
	struct crit_entry *ce;
	struct task_struct *readmit[NUM_CRIT_LEVELS];
	enum crit_level level = entry_level(entry);

	/* Remove lower priority tasks from the entry */
	for (i = level + 1; i < NUM_CRIT_LEVELS; i++) {
		ce = &entry->crit_entries[i];

		global_preempted = ce->linked &&
			/* This task is running on a cpu */
			ce->linked->rt_param.scheduled_on == entry->cpu &&
			/* But it was preempted */
			ce->linked != entry->linked &&
			/* And it is an eligible global task */
			!is_ghost(ce->linked) && is_global(ce->domain);

		/* Do not readmit global tasks which are preempted! These can't
		 * ever be re-admitted until they are descheduled for reasons
		 * explained in job_arrival.
		 */
		readmit[i] = (!global_preempted) ? ce->linked : NULL;

		ce->state = CS_REMOVE;
		if (ce->linked)
			link_task_to_crit(ce, NULL);
	}
	/* Need to unlock so we can access domains */
	raw_spin_unlock(&entry->lock);

	/* Re-admit tasks to the system */
	for (i = level + 1; i < NUM_CRIT_LEVELS; i++) {
		ce = &entry->crit_entries[i];
		if (readmit[i]) {
			low_prio_arrival(readmit[i]);
		}
	}
}

/**
 * check_for_preempt() - Causes a preemption if higher-priority tasks are ready.
 * Caller must hold domain lock.
 * Makes gigantic nasty assumption that there is 1 global criticality level,
 * and it is the last one in each list, so it doesn't call update_crit..
 */
static void check_for_preempt(struct domain *dom)
{
	int recheck = 1;
	struct cpu_entry *entry;
	struct crit_entry *ce;

	if (is_global(dom)) {
		/* Loop until we find a non-preemptable CPU */
		while ((ce = lowest_prio_cpu(dom)) && recheck) {
			entry = crit_cpu(ce);
			recheck = 1;

			/* Cache next task */
			dom->peek_ready(dom);

			raw_spin_lock(&entry->lock);
			if (!can_use(ce))
				/* CPU disabled while locking! */
				fix_crit_position(ce);
			else if (dom->preempt_needed(dom, ce->linked))
				/* Success! Check for more preemptions */
				preempt(dom, ce);
			else {
				/* Failure! */
				recheck = 0;
				TRACE_CRIT_ENTRY(ce, "Stopped global check\n");
			}
			raw_spin_unlock(&entry->lock);
		}
	} else /* Partitioned */ {
		ce = domain_data(dom)->crit_entry;
		entry = crit_cpu(ce);

		/* Cache next task */
		dom->peek_ready(dom);

		raw_spin_lock(&entry->lock);
		if (can_use(ce) && dom->preempt_needed(dom, ce->linked)) {
			preempt(dom, ce);
			update_crit_levels(entry);
		} else {
			raw_spin_unlock(&entry->lock);
		}
	}
}

/**
 * remove_from_all() - Logically remove a task from all structures.
 * Caller must hold no locks.
 */
static void remove_from_all(struct task_struct* task)
{
	int update = 0;
    	struct cpu_entry *entry;
	struct crit_entry *ce;
	struct domain *dom = get_task_domain(task);

	TRACE_MC_TASK(task, "Removing from everything\n");
	BUG_ON(!task);

	raw_spin_lock(dom->lock);

	/* Remove the task from any CPU state */
	if (task->rt_param.linked_on != NO_CPU) {
		entry = &per_cpu(cpus, task->rt_param.linked_on);
		raw_spin_lock(&entry->lock);

		/* Unlink only if task is still linked post lock */
		ce = &entry->crit_entries[tsk_mc_crit(task)];
		if (task->rt_param.linked_on != NO_CPU) {
			BUG_ON(ce->linked != task);
			link_task_to_crit(ce, NULL);
			update_crit_position(ce);
			if (!is_ghost(task) && entry->linked == task) {
				update = 1;
				link_task_to_cpu(entry, NULL);
			}
		} else {
			TRACE_MC_TASK(task, "Unlinked before we got lock!\n");
		}
		if (update)
			update_crit_levels(entry);
		else
			raw_spin_unlock(&entry->lock);
	} else {
		TRACE_MC_TASK(task, "Not linked to anything\n");
	}

	/* Ensure the task isn't returned by its domain */
	dom->remove(dom, task);

	raw_spin_unlock(dom->lock);
}

/**
 * job_completion() - Update task state and re-enter it into the system.
 * Converts tasks which have completed their execution early into ghost jobs.
 * Caller must hold no locks.
 */
static void job_completion(struct task_struct *task, int forced)
{
	int behind;
	TRACE_MC_TASK(task, "Completed\n");

	/* Logically stop the task execution */
	set_rt_flags(task, RT_F_SLEEP);
	remove_from_all(task);

	/* Level-A tasks cannot ever get behind */
	behind = tsk_mc_crit(task) != CRIT_LEVEL_A && behind_server(task);

	if (!forced && !is_ghost(task)) {
		/* Task voluntarily ceased execution. Move on to next period */
		task_release(task);
		sched_trace_task_completion(task, forced);

		/* Convert to ghost job */
		tsk_mc_data(task)->mc_job.ghost_budget = budget_remaining(task);
		tsk_mc_data(task)->mc_job.is_ghost = 1;
	}

	/* If the task has no ghost budget, convert back from ghost.
	 * If the task is behind, undo ghost conversion so that it
	 * can catch up.
	 */
	if (behind || tsk_mc_data(task)->mc_job.ghost_budget == 0) {
		TRACE_MC_TASK(task, "Not a ghost task\n");
		tsk_mc_data(task)->mc_job.is_ghost = 0;
		tsk_mc_data(task)->mc_job.ghost_budget = 0;
	}

	/* If server has run out of budget, wait until next release */
	if (budget_exhausted(task)) {
		server_release(task);
	}

	/* Requeue non-blocking tasks */
	if (is_running(task))
		job_arrival(task);
}

/**
 * mc_ghost_exhausted() - Complete logically running ghost task.
 */
#ifdef CONFIG_MERGE_TIMERS
static void mc_ghost_exhausted(struct rt_event *e)
{
	struct crit_entry *ce = container_of(e, struct crit_entry, event);
#else
static enum hrtimer_restart mc_ghost_exhausted(struct hrtimer *timer)
{
	struct crit_entry *ce = container_of(timer, struct crit_entry, timer);
#endif

	unsigned long flags;
	struct task_struct *tmp = NULL;

	local_irq_save(flags);
	TRACE("Ghost exhausted\n");
	TRACE_CRIT_ENTRY(ce, "Firing here\n");

	/* Due to race conditions, we cannot just set the linked
	 * task's budget to 0 as it may no longer be the task
	 * for which this timer was armed. Instead, update the running
	 * task time and see if this causes exhaustion.
	 */
	raw_spin_lock(&crit_cpu(ce)->lock);
	if (ce->linked && is_ghost(ce->linked)) {
		update_ghost_time(ce->linked);
		if (tsk_mc_data(ce->linked)->mc_job.ghost_budget == 0) {
			tmp = ce->linked;
		}
	}
	raw_spin_unlock(&crit_cpu(ce)->lock);

	if (tmp)
		job_completion(tmp, 0);

	local_irq_restore(flags);
#ifndef CONFIG_MERGE_TIMERS
	return HRTIMER_NORESTART;
#endif
}

/*
 * The MC-CE common timer callback code for merged and non-merged timers.
 * Returns the next time the timer should fire.
 */
static lt_t __ce_timer_function(struct ce_dom_data *ce_data)
{
	struct crit_entry *ce = get_crit_entry_for(ce_data->cpu, CRIT_LEVEL_A);
	struct domain *dom = ce->domain;
	struct task_struct *old_link = NULL;
	lt_t next_timer_abs;

	TRACE("MC level-A timer callback for CPU %d\n", ce_data->cpu);

	raw_spin_lock(dom->lock);

	raw_spin_lock(&crit_cpu(ce)->lock);
	if (ce->linked &&
	    ce->linked == ce_data->should_schedule &&
	    is_ghost(ce->linked))
	{
		old_link = ce->linked;
		tsk_mc_data(ce->linked)->mc_job.ghost_budget = 0;
		link_task_to_crit(ce, NULL);
	}
	raw_spin_unlock(&crit_cpu(ce)->lock);

	next_timer_abs = mc_ce_timer_callback_common(dom);

	/* Job completion will check for preemptions by means of calling job
	 * arrival if the task is not blocked */
	if (NULL != old_link) {
		STRACE("old_link " TS " so will call job completion\n", TA(old_link));
		raw_spin_unlock(dom->lock);
		job_completion(old_link, 0);
	} else {
		STRACE("old_link was null, so will call check for preempt\n");
		raw_spin_unlock(dom->lock);
		check_for_preempt(dom);
	}
	return next_timer_abs;
}

#ifdef CONFIG_MERGE_TIMERS
static void ce_timer_function(struct rt_event *e)
{
	struct ce_dom_data *ce_data =
		container_of(e, struct ce_dom_data, event);
	unsigned long flags;
	lt_t next_timer_abs;

	TS_LVLA_RELEASE_START;

	local_irq_save(flags);
	next_timer_abs = __ce_timer_function(ce_data);
	add_event(per_cpu(cpus, ce_data->cpu).event_group, e, next_timer_abs);
	local_irq_restore(flags);

	TS_LVLA_RELEASE_END;
}
#else /* else to CONFIG_MERGE_TIMERS */
static enum hrtimer_restart ce_timer_function(struct hrtimer *timer)
{
	struct ce_dom_data *ce_data =
		container_of(timer, struct ce_dom_data, timer);
	unsigned long flags;
	lt_t next_timer_abs;

	TS_LVLA_RELEASE_START;

	local_irq_save(flags);
	next_timer_abs = __ce_timer_function(ce_data);
	hrtimer_set_expires(timer, ns_to_ktime(next_timer_abs));
	local_irq_restore(flags);

	TS_LVLA_RELEASE_END;

	return HRTIMER_RESTART;
}
#endif /* CONFIG_MERGE_TIMERS */


/**
 * mc_release_jobs() - Add heap of tasks to the system, check for preemptions.
 */
static void mc_release_jobs(rt_domain_t* rt, struct bheap* tasks)
{
	unsigned long flags;
	struct task_struct *first = bheap_peek(rt->order, tasks)->value;
	struct domain *dom = get_task_domain(first);

	raw_spin_lock_irqsave(dom->lock, flags);
	TRACE(TS "Jobs released\n", TA(first));
	__merge_ready(rt, tasks);
	check_for_preempt(dom);
	raw_spin_unlock_irqrestore(dom->lock, flags);
}

/**
 * ms_task_new() - Setup new mixed-criticality task.
 * Assumes that there are no partitioned domains after level B.
 */
static void mc_task_new(struct task_struct *t, int on_rq, int running)
{
	unsigned long flags;
	struct cpu_entry* entry;
	enum crit_level level = tsk_mc_crit(t);

	local_irq_save(flags);
	TRACE("New mixed criticality task %d\n", t->pid);

	/* Assign domain */
	if (level < CRIT_LEVEL_C)
		entry = &per_cpu(cpus, get_partition(t));
	else
		entry = &per_cpu(cpus, task_cpu(t));
	t->rt_param._domain = entry->crit_entries[level].domain;

	/* Setup job params */
	release_at(t, litmus_clock());
	tsk_mc_data(t)->mc_job.ghost_budget = 0;
	tsk_mc_data(t)->mc_job.is_ghost = 0;
	if (running) {
		BUG_ON(entry->scheduled);
		entry->scheduled = t;
		tsk_rt(t)->scheduled_on = entry->cpu;
	} else {
		t->rt_param.scheduled_on = NO_CPU;
	}
	t->rt_param.linked_on = NO_CPU;

	job_arrival(t);

	local_irq_restore(flags);
}

/**
 * mc_task_new() - Add task back into its domain check for preemptions.
 */
static void mc_task_wake_up(struct task_struct *task)
{
	unsigned long flags;
	lt_t now = litmus_clock();
	local_irq_save(flags);

	TRACE(TS " wakes up\n", TA(task));
	if (is_tardy(task, now)) {
		/* Task missed its last release */
		release_at(task, now);
		sched_trace_task_release(task);
	}
	if (!is_ghost(task))
		job_arrival(task);

	local_irq_restore(flags);
}

/**
 * mc_task_block() - Remove task from state to prevent it being run anywhere.
 */
static void mc_task_block(struct task_struct *task)
{
	unsigned long flags;
	local_irq_save(flags);
	TRACE(TS " blocks\n", TA(task));
	remove_from_all(task);
	local_irq_restore(flags);
}

/**
 * mc_task_exit() - Remove task from the system.
 */
static void mc_task_exit(struct task_struct *task)
{
	unsigned long flags;
	local_irq_save(flags);
	BUG_ON(!is_realtime(task));
	TRACE(TS " RIP\n", TA(task));

	remove_from_all(task);
	if (tsk_rt(task)->scheduled_on != NO_CPU) {
		per_cpu(cpus, tsk_rt(task)->scheduled_on).scheduled = NULL;
		tsk_rt(task)->scheduled_on = NO_CPU;
	}

	if (CRIT_LEVEL_A == tsk_mc_crit(task))
		mc_ce_task_exit_common(task);

	local_irq_restore(flags);
}

/**
 * mc_admit_task() - Return true if the task is valid.
 * Assumes there are no partitioned levels after level B.
 */
static long mc_admit_task(struct task_struct* task)
{
	const enum crit_level crit = tsk_mc_crit(task);
	long ret;
	if (!tsk_mc_data(task))	{
		printk(KERN_WARNING "Tried to admit task with no criticality "
			"level\n");
		ret = -EINVAL;
		goto out;
	}
	if (crit < CRIT_LEVEL_C && get_partition(task) == NO_CPU) {
		printk(KERN_WARNING "Tried to admit partitioned task with no "
		       "partition\n");
		ret = -EINVAL;
		goto out;
	}
	if (crit == CRIT_LEVEL_A) {
		ret = mc_ce_admit_task_common(task);
		if (ret)
			goto out;
	}
	printk(KERN_INFO "Admitted task with criticality level %d\n",
		tsk_mc_crit(task));
	ret = 0;
out:
	return ret;
}

/**
 * mc_schedule() - Return next task which should be scheduled.
 */
static struct task_struct* mc_schedule(struct task_struct* prev)
{
	unsigned long flags;
	struct domain *dom;
	struct crit_entry *ce;
	struct cpu_entry* entry = &__get_cpu_var(cpus);
	int i, out_of_time, sleep, preempt, exists, blocks, global, lower;
	struct task_struct *dtask = NULL, *ready_task = NULL, *next = NULL;

	local_irq_save(flags);

	/* Litmus gave up because it couldn't access the stack of the CPU
	 * on which will_schedule was migrating from. Requeue it.
	 * This really only happens in VMs.
	 */
	if (entry->will_schedule && entry->will_schedule != prev) {
		entry->will_schedule->rt_param.scheduled_on = NO_CPU;
		low_prio_arrival(entry->will_schedule);
	}

	raw_spin_lock(&entry->lock);

	/* Sanity checking */
	BUG_ON(entry->scheduled && entry->scheduled != prev);
	BUG_ON(entry->scheduled && !is_realtime(prev));
	BUG_ON(is_realtime(prev) && !entry->scheduled);

	/* Determine state */
	exists      = entry->scheduled != NULL;
	blocks      = exists && !is_running(entry->scheduled);
	out_of_time = exists &&	budget_enforced(entry->scheduled) &&
				budget_exhausted(entry->scheduled);
	sleep	    = exists && get_rt_flags(entry->scheduled) == RT_F_SLEEP;
	global      = exists && is_global_task(entry->scheduled);
	preempt     = entry->scheduled != entry->linked;
	lower       = exists && preempt && entry->linked &&
		tsk_mc_crit(entry->scheduled) > tsk_mc_crit(entry->linked);

	TRACE(TS " blocks:%d out_of_time:%d sleep:%d preempt:%d\n",
	      TA(prev), blocks, out_of_time, sleep, preempt);

	if (exists)
		prev->rt_param.scheduled_on = NO_CPU;

	raw_spin_unlock(&entry->lock);


#ifdef CONFIG_PLUGIN_MC_REDIRECT
	if (smp_processor_id() == interrupt_cpu)
		fix_global_levels();
#endif

	/* If a task blocks we have no choice but to reschedule */
	if (blocks)
		remove_from_all(entry->scheduled);
	/* Any task which exhausts its budget or sleeps waiting for its next
	 * period completes unless its execution has been forcibly stopped.
	 */
	if ((out_of_time || sleep) && !blocks)/* && !preempt)*/
		job_completion(entry->scheduled, !sleep);
	/* Global scheduled tasks must wait for a deschedule before they
	 * can rejoin the global state. Rejoin them here.
	 */
	else if (global && preempt && !blocks) {
		if (lower)
			low_prio_arrival(entry->scheduled);
		else
			job_arrival(entry->scheduled);
	}

	/* Pick next task if none is linked */
	raw_spin_lock(&entry->lock);
	for (i = 0; i < NUM_CRIT_LEVELS && !entry->linked; i++) {
		ce = &entry->crit_entries[i];
		dom = ce->domain;

		/* Swap locks. We cannot acquire a domain lock while
		 * holding an entry lock or deadlocks will happen.
		 */
		raw_spin_unlock(&entry->lock);
		raw_spin_lock(dom->lock);

		/* Do domain stuff before grabbing CPU locks */
		dtask = dom->peek_ready(dom);
		fix_crit_position(ce);

		raw_spin_lock(&entry->lock);

		if (!entry->linked && !ce->linked && dtask && can_use(ce)) {
			dom->take_ready(dom);
			link_task_to_crit(ce, dtask);
			update_crit_position(ce);
			ready_task = (is_ghost(dtask)) ? NULL : dtask;

			/* Task found! */
			if (ready_task) {
				link_task_to_cpu(entry, ready_task);
				raw_spin_unlock(dom->lock);
				update_crit_levels(entry);
				raw_spin_lock(&entry->lock);
				continue;
			}
		}
		raw_spin_unlock(dom->lock);
	}

	/* Schedule next task */
	next = entry->linked;
	if (entry->linked)
		entry->linked->rt_param.scheduled_on = entry->cpu;
	entry->will_schedule = entry->linked;
	sched_state_task_picked();

	raw_spin_unlock(&entry->lock);
	local_irq_restore(flags);
	if (next) {
		TRACE_MC_TASK(next, "Picked this task\n");
	} else if (exists && !next)
		TRACE_ENTRY(entry, "Becomes idle at %llu\n", litmus_clock());
	return next;
}

void mc_finish_switch(struct task_struct *prev)
{
	struct cpu_entry* entry = &__get_cpu_var(cpus);
	entry->scheduled = is_realtime(current) ? current : NULL;
	TRACE_TASK(prev, "Switched away from to " TS "\n",
		   TA(entry->scheduled));
}

/*
 * This is the plugin's release at function, called by the release task-set
 * system call. Other places in the file use the generic LITMUS release_at(),
 * which is not this.
 */
void mc_release_at(struct task_struct *ts, lt_t start)
{
	/* hack so that we can have CE timers start at the right time */
	if (CRIT_LEVEL_A == tsk_mc_crit(ts))
		mc_ce_release_at_common(ts, start);
	else
		release_at(ts, start);
}

long mc_deactivate_plugin(void)
{
	return mc_ce_deactivate_plugin_common();
}

/* **************************************************************************
 * Initialization
 * ************************************************************************** */

/* Initialize values here so that they are allocated with the module
 * and destroyed when the module is unloaded.
 */

/* LVL-A */
DEFINE_PER_CPU(struct domain_data, _mc_crit_a);
DEFINE_PER_CPU(raw_spinlock_t, _mc_crit_a_lock);
DEFINE_PER_CPU(struct ce_dom_data, _mc_crit_a_ce_data);
/* LVL-B */
DEFINE_PER_CPU(struct domain_data, _mc_crit_b);
DEFINE_PER_CPU(rt_domain_t, _mc_crit_b_rt);
/* LVL-C */
static struct domain_data _mc_crit_c;
static rt_domain_t _mc_crit_c_rt;
struct bheap _mc_heap_c;
struct bheap_node _mc_nodes_c[NR_CPUS];

static long mc_activate_plugin(void)
{
	struct domain_data *dom_data;
	struct domain *dom;
	struct domain_data *our_domains[NR_CPUS];
	int cpu, n = 0;
	long ret;

#ifdef CONFIG_RELEASE_MASTER
	interrupt_cpu = atomic_read(&release_master_cpu);
#if defined(CONFIG_PLUGIN_MC_REDIRECT) || defined(CONFIG_PLUGIN_MC_RELEASE_MASTER)
	if (NO_CPU == interrupt_cpu) {
		printk(KERN_ERR "LITMUS-MC: need a release master\n");
		ret = -EINVAL;
		goto out;
	}
#endif
#endif

	for_each_online_cpu(cpu) {
		BUG_ON(NR_CPUS <= n);
		dom = per_cpu(cpus, cpu).crit_entries[CRIT_LEVEL_A].domain;
		dom_data = domain_data(dom);
		our_domains[cpu] = dom_data;
#if defined(CONFIG_MERGE_TIMERS) && defined(CONFIG_PLUGIN_MC_RELEASE_MASTER)
		per_cpu(cpus, cpu).event_group =
			get_event_group_for(interrupt_cpu);
#elif defined(CONFIG_MERGE_TIMERS) && !defined(CONFIG_PLUGIN_MC_RELEASE_MASTER)
		per_cpu(cpus, cpu).event_group = get_event_group_for(cpu);
#endif
		n++;
	}
	ret = mc_ce_set_domains(n, our_domains);
	if (ret)
		goto out;
	ret = mc_ce_activate_plugin_common();
out:
	return ret;
}

static struct sched_plugin mc_plugin __cacheline_aligned_in_smp = {
	.plugin_name		= "MC",
	.task_new		= mc_task_new,
	.complete_job		= complete_job,
	.task_exit		= mc_task_exit,
	.schedule		= mc_schedule,
	.task_wake_up		= mc_task_wake_up,
	.task_block		= mc_task_block,
	.admit_task		= mc_admit_task,
	.activate_plugin	= mc_activate_plugin,
	.release_at		= mc_release_at,
	.deactivate_plugin	= mc_deactivate_plugin,
	.finish_switch		= mc_finish_switch,
};

static void init_crit_entry(struct crit_entry *ce, enum crit_level level,
			    struct domain_data *dom_data,
			    struct bheap_node *node)
{
	ce->level  = level;
	ce->linked = NULL;
	ce->node   = node;
	ce->domain = &dom_data->domain;
	ce->state  = CS_ACTIVE;
#ifdef CONFIG_MERGE_TIMERS
	init_event(&ce->event, level, mc_ghost_exhausted,
		   event_list_alloc(GFP_ATOMIC));
#else
	hrtimer_init(&ce->timer, CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
	ce->timer.function = mc_ghost_exhausted;
#endif

}

static void init_local_domain(struct cpu_entry *entry, struct domain_data *dom_data,
			      enum crit_level level)
{
	dom_data->heap = NULL;
	dom_data->crit_entry = &entry->crit_entries[level];
	init_crit_entry(dom_data->crit_entry, level, dom_data, NULL);
}

static void init_global_domain(struct domain_data *dom_data, enum crit_level level,
			       struct bheap *heap, struct bheap_node *nodes)
{
	int cpu;
	struct cpu_entry *entry;
	struct crit_entry *ce;
	struct bheap_node *node;

	dom_data->crit_entry = NULL;
	dom_data->heap = heap;
	bheap_init(heap);

	for_each_online_cpu(cpu) {
		entry = &per_cpu(cpus, cpu);
		node = &nodes[cpu];
		ce = &entry->crit_entries[level];
		init_crit_entry(ce, level, dom_data, node);
		bheap_node_init(&ce->node, ce);
		bheap_insert(cpu_lower_prio, heap, node);
	}
}

static inline void init_edf_domain(struct domain *dom, rt_domain_t *rt,
				   int prio, int is_partitioned, int cpu)
{
	pd_domain_init(dom, rt, edf_ready_order, NULL,
		       mc_release_jobs, mc_preempt_needed,
		       edf_higher_prio);
#if defined(CONFIG_PLUGIN_MC_RELEASE_MASTER) && defined(CONFIG_MERGE_TIMERS)
	/* All timers are on one CPU and release-master is using the event
	 * merging interface as well. */
	BUG_ON(NO_CPU == interrupt_cpu);
	rt->event_group = get_event_group_for(interrupt_cpu);
	rt->prio = prio;
#elif defined(CONFIG_PLUGIN_MC_RELEASE_MASTER) && !defined(CONFIG_MERGE_TIMERS)
	/* Using release master, but not merging timers. */
	rt->release_master = interrupt_cpu;
#elif !defined(CONFIG_PLUGIN_MC_RELEASE_MASTER) && defined(CONFIG_MERGE_TIMERS)
	/* Merge the timers, but don't move them to the release master. */
	if (is_partitioned) {
		rt->event_group = get_event_group_for(cpu);
	} else {
		/* Global timers will be added to the event groups that code is
		 * executing on when add_event() is called.
		 */
		rt->event_group = NULL;
	}
	rt->prio = prio;
#endif
}

struct domain_data *ce_domain_for(int);
static int __init init_mc(void)
{
	int cpu;
	struct cpu_entry *entry;
	struct domain_data *dom_data;
	rt_domain_t *rt;
	raw_spinlock_t *a_dom_lock, *b_dom_lock, *c_dom_lock; /* For lock debugger */
	struct ce_dom_data *ce_data;

	for_each_online_cpu(cpu) {
		entry = &per_cpu(cpus, cpu);

		/* CPU */
		entry->cpu = cpu;
		entry->scheduled = NULL;
		entry->linked = NULL;

		raw_spin_lock_init(&entry->lock);

#ifdef CONFIG_PLUGIN_MC_REDIRECT
		raw_spin_lock_init(&entry->redir_lock);
		INIT_LIST_HEAD(&entry->redir);
#endif

		/* CRIT_LEVEL_A */
		dom_data = &per_cpu(_mc_crit_a, cpu);
		ce_data = &per_cpu(_mc_crit_a_ce_data, cpu);
		a_dom_lock = &per_cpu(_mc_crit_a_lock, cpu);
		raw_spin_lock_init(a_dom_lock);
		ce_domain_init(&dom_data->domain,
				a_dom_lock, ce_requeue, ce_peek_and_take_ready,
				ce_peek_and_take_ready, mc_preempt_needed,
				ce_higher_prio, ce_data, cpu,
				ce_timer_function);
		init_local_domain(entry, dom_data, CRIT_LEVEL_A);
		dom_data->domain.name = "LVL-A";

		/* CRIT_LEVEL_B */
		dom_data = &per_cpu(_mc_crit_b, cpu);
		rt = &per_cpu(_mc_crit_b_rt, cpu);
		init_local_domain(entry, dom_data, CRIT_LEVEL_B);
		init_edf_domain(&dom_data->domain, rt, CRIT_LEVEL_B, 1, cpu);
		b_dom_lock = dom_data->domain.lock;
		raw_spin_lock_init(b_dom_lock);
		dom_data->domain.name = "LVL-B";
	}

	/* CRIT_LEVEL_C */
	init_global_domain(&_mc_crit_c, CRIT_LEVEL_C,
			   &_mc_heap_c, _mc_nodes_c);
	init_edf_domain(&_mc_crit_c.domain, &_mc_crit_c_rt, CRIT_LEVEL_C,
			0, NO_CPU);
	c_dom_lock = _mc_crit_c.domain.lock;
	raw_spin_lock_init(c_dom_lock);
	_mc_crit_c.domain.name = "LVL-C";

	return register_sched_plugin(&mc_plugin);
}

module_init(init_mc);
