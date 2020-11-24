/**
 * --Todo--
 * Naming: this should become rt_domain while the old rt_domain should be
 * changed to sd_domain (sporadic) or pd_domain (periodic).
 * task_new: need to add and use this method
 */
#ifndef _LITMUS_DOMAIN_H_
#define _LITMUS_DOMAIN_H_

struct domain;

typedef void (*requeue_t)(struct domain*, struct task_struct*);
typedef void (*remove_t)(struct domain*, struct task_struct*);
typedef struct task_struct* (*peek_ready_t)(struct domain*);
typedef struct task_struct* (*take_ready_t)(struct domain*);
typedef int (*preempt_needed_t)(struct domain*, struct task_struct*);
typedef int (*task_prio_t)(struct task_struct*, struct task_struct*);

typedef struct domain {
	raw_spinlock_t*		lock; /* for coarse serialization     	*/
	struct list_head	list; /* list membership              	*/
	void*			data; /* implementation-specific data	*/
	char*			name; /* for debugging 		      	*/

	/* add a task to the domain */
	requeue_t		requeue;
	/* prevent a task from being returned by the domain */
	remove_t		remove;
	/* return next ready task */
	peek_ready_t		peek_ready;
	/* remove and return next ready task */
	take_ready_t		take_ready;
	/* return true if the domain has a task which should preempt the
	 * task given
	 */
	preempt_needed_t	preempt_needed;
	/* for tasks within this domain, returns true if the first has
	 * has a higher priority than the second
	 */
	task_prio_t		higher_prio;
} domain_t;

void domain_init(domain_t *dom,
		 raw_spinlock_t *lock,
		 requeue_t requeue,
		 peek_ready_t peek_ready,
		 take_ready_t take_ready,
		 preempt_needed_t preempt_needed,
		 task_prio_t priority);
#endif
