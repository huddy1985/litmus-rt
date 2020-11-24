#ifndef _LITMUS_CE_DOMAIN_H
#define _LITMUS_CE_DOMAIN_H

/*
 * Functions that the MC plugin needs to call through a domain pointer.
 */
void ce_requeue(domain_t*, struct task_struct*);
struct task_struct* ce_peek_and_take_ready(domain_t*);
int ce_higher_prio(struct task_struct*, struct task_struct*);

#ifdef CONFIG_MERGE_TIMERS
typedef void (*ce_timer_callback_t)(struct rt_event*);
#else
typedef enum hrtimer_restart (*ce_timer_callback_t)(struct hrtimer*);
#endif

void ce_domain_init(domain_t*,
		raw_spinlock_t*,
		requeue_t,
		peek_ready_t,
		take_ready_t,
		preempt_needed_t,
		task_prio_t,
		struct ce_dom_data*,
		const int,
		ce_timer_callback_t);
#endif
