#ifndef _LINUX_SCHED_MC_H_
#define _LINUX_SCHED_MC_H_

/* criticality levels */
enum crit_level {
	/* probably don't need to assign these (paranoid) */
	CRIT_LEVEL_A = 0,
	CRIT_LEVEL_B = 1,
	CRIT_LEVEL_C = 2,
	NUM_CRIT_LEVELS = 3,
};

struct mc_task {
	enum crit_level crit;
	int lvl_a_id;
	int lvl_a_eligible;
};

struct mc_job {
	int is_ghost:1;
	lt_t ghost_budget;
};

#ifdef __KERNEL__
/*
 * These are used only in the kernel. Userspace programs like RTSpin won't see
 * them.
 */
struct mc_data {
	struct mc_task mc_task;
	struct mc_job mc_job;
};

#define tsk_mc_data(t)	 (tsk_rt(t)->mc_data)
#define tsk_mc_crit(t)	 (tsk_mc_data(t)->mc_task.crit)
#define is_ghost(t)	 (tsk_mc_data(t)->mc_job.is_ghost)

#define TS "(%s/%d:%d:%s)"
#define TA(t) (t) ? tsk_mc_data(t) ? is_ghost(t) ? "ghost" : t->comm \
						 : t->comm : "NULL", \
	      (t) ? t->pid : 1,					\
	      (t) ? t->rt_param.job_params.job_no : 1,		\
	      (t && get_task_domain(t)) ? get_task_domain(t)->name : ""
#define STRACE(fmt, args...) \
	sched_trace_log_message("%d P%d      [%s@%s:%d]: " fmt,	\
				TRACE_ARGS,  ## args)
#define TRACE_MC_TASK(t, fmt, args...)				\
	STRACE(TS " " fmt, TA(t), ##args)

/*
 * The MC-CE scheduler uses this as domain data.
 */
struct ce_dom_data {
	int cpu;
	struct task_struct *scheduled, *should_schedule;
#ifdef CONFIG_MERGE_TIMERS
	struct rt_event event;
#else
	struct hrtimer_start_on_info timer_info;
	struct hrtimer timer;
#endif
};

/**
 * enum crit_state - Logically add / remove CPUs from criticality levels.
 *
 * Global crit levels need to use a two step process to remove CPUs so
 * that the CPUs can be removed without holding domain locks.
 *
 * @CS_ACTIVE	The criticality entry can run a task
 * @CS_ACTIVATE The criticality entry can run a task, but hasn't had its
 *		position updated in a global heap. Set with ONLY CPU lock.
 * @CS_REMOVE   The criticality entry is logically removed, but hasn't had its
 *		position adjusted in a global heap. This should be set when
 *		ONLY the CPU state is locked.
 * @CS_REMOVED	The criticality entry has been removed from the crit level
 */
enum crit_state { CS_ACTIVE, CS_ACTIVATE, CS_REMOVE, CS_REMOVED };

/**
 * struct crit_entry - State of a CPU within each criticality level system.
 * @level	Criticality level of this entry
 * @linked	Logically running task, ghost or regular
 * @domain	Domain from which to draw tasks
 * @usable	False if a higher criticality task is running
 * @event	For ghost task budget enforcement (merge timers)
 * @timer	For ghost task budget enforcement (not merge timers)
 * @node	Used to sort crit_entries by preemptability in global domains
 */
struct crit_entry {
	enum crit_level		level;
	struct task_struct*	linked;
	struct domain*		domain;
	enum crit_state		state;
#ifdef CONFIG_MERGE_TIMERS
	struct rt_event		event;
#else
	struct hrtimer		timer;
#endif
	struct bheap_node*	node;
};

/**
 * struct domain_data - Wrap domains with related CPU state
 * @domain	A domain for a criticality level
 * @heap	The preemptable heap of crit entries (for global domains)
 * @crit_entry	The crit entry for this domain (for partitioned domains)
 */
struct domain_data {
	struct domain 		domain;
	struct bheap*		heap;
	struct crit_entry*	crit_entry;
};

/*
 * Functions that are used with the MC-CE plugin.
 */
long mc_ce_set_domains(const int, struct domain_data*[]);
unsigned int mc_ce_get_expected_job(const int, const int);

/*
 * These functions are (lazily) inserted into the MC plugin code so that it
 * manipulates the MC-CE state.
 */
long mc_ce_admit_task_common(struct task_struct*);
void mc_ce_task_exit_common(struct task_struct*);
lt_t mc_ce_timer_callback_common(domain_t*);
void mc_ce_release_at_common(struct task_struct*, lt_t);
long mc_ce_activate_plugin_common(void);
long mc_ce_deactivate_plugin_common(void);

#endif /* __KERNEL__ */

#endif
