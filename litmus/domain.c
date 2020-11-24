#include <linux/list.h>
#include <linux/spinlock_types.h>

#include <litmus/domain.h>

void domain_init(domain_t *dom,
		 raw_spinlock_t *lock,
		 requeue_t requeue,
		 peek_ready_t peek_ready,
		 take_ready_t take_ready,
		 preempt_needed_t preempt_needed,
		 task_prio_t priority)
{
	INIT_LIST_HEAD(&dom->list);
	dom->lock = lock;
	dom->requeue = requeue;
	dom->peek_ready = peek_ready;
	dom->take_ready = take_ready;
	dom->preempt_needed = preempt_needed;
	dom->higher_prio = priority;
}
