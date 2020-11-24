#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/module.h>

#include <litmus/litmus.h>
#include <litmus/trace.h>
#include <litmus/sched_trace.h>
#include <litmus/event_group.h>

#if 1
#define VTRACE(fmt, args...)					\
sched_trace_log_message("%d P%d        [%s@%s:%d]: " fmt,	\
				TRACE_ARGS,  ## args)
#else
#define VTRACE(fmt, args...)
#endif

/*
 * Return event_queue slot for the given time.
 */
static unsigned int time2slot(lt_t time)
{
	return (unsigned int) time2quanta(time, FLOOR) % EVENT_QUEUE_SLOTS;
}

/*
 * Executes events from an event_list in priority order.
 * Events can requeue themselves when they are called.
 */
static enum hrtimer_restart on_timer(struct hrtimer *timer)
{
	int prio, num;
	unsigned long flags;
	struct event_list *el;
	struct rt_event *e;
	struct list_head *pos, events[NUM_EVENT_PRIORITIES];
	raw_spinlock_t *queue_lock;

	el = container_of(timer, struct event_list, timer);
	queue_lock = &el->group->queue_lock;

	raw_spin_lock_irqsave(queue_lock, flags);

	/* Remove event_list from hashtable so that no more events
	 * are added to it.
	 */
	VTRACE("Removing event list 0x%x\n", el);
	list_del_init(&el->queue_node);

	/* Copy over events so that the event_list can re-used when the lock
	 * is released.
	 */
	VTRACE("Emptying event list 0x%x\n", el);
	for (prio = 0; prio < NUM_EVENT_PRIORITIES; prio++) {
		list_replace_init(&el->events[prio], &events[prio]);
	}

	for (prio = 0; prio < NUM_EVENT_PRIORITIES; prio++) {
		/* Fire events. Complicated loop is used so that events
		 * in the list can be canceled (removed) while other events are
		 * executing.
		 */
		for (pos = events[prio].next, num = 0;
		     prefetch(pos->next), events[prio].next != &events[prio];
		     pos = events[prio].next, num++) {

			e = list_entry(pos, struct rt_event, events_node);
			list_del_init(pos);
			raw_spin_unlock_irqrestore(queue_lock, flags);

			VTRACE("Dequeueing event 0x%x with prio %d from 0x%x\n",
			       e, e->prio, el);
			e->function(e);

			raw_spin_lock_irqsave(queue_lock, flags);
		}
	}
	raw_spin_unlock_irqrestore(queue_lock, flags);

	VTRACE("Exhausted %d events from list 0x%x\n", num, el);

	return HRTIMER_NORESTART;
}

/*
 * Return event_list for the given event and time. If no event_list
 * is being used yet and use_event_heap is 1, will create the list
 * and return it. Otherwise it will return NULL.
 */
static struct event_list* get_event_list(struct event_group *group,
					 struct rt_event *e,
					 lt_t fire,
					 int use_event_list)
{
	struct list_head* pos;
	struct event_list *el = NULL, *tmp;
	unsigned int slot = time2slot(fire);
	int remaining = 300;

	VTRACE("Getting list for time %llu, event 0x%x\n", fire, e);

	/* Initialize pos for the case that the list is empty */
	pos = group->event_queue[slot].next;
	list_for_each(pos, &group->event_queue[slot]) {
		BUG_ON(remaining-- < 0);
		tmp = list_entry(pos, struct event_list, queue_node);
		if (lt_after_eq(fire, tmp->fire_time) &&
		    lt_before(fire, tmp->fire_time + group->res)) {
			VTRACE("Found match 0x%x at time %llu\n",
			       tmp, tmp->fire_time);
			el = tmp;
			break;
		} else if (lt_before(fire, tmp->fire_time)) {
			/* We need to insert a new node since el is
			 * already in the future
			 */
			VTRACE("Time %llu was before %llu\n",
			       fire, tmp->fire_time);
			break;
		} else {
			VTRACE("Time %llu was after %llu\n",
			       fire, tmp->fire_time + group->res);
		}
	}
	if (!el && use_event_list) {
		/* Use pre-allocated list */
		tmp = e->event_list;
		tmp->fire_time = fire;
		tmp->group = group;
		/* Add to queue */
		VTRACE("Using list 0x%x for priority %d and time %llu\n",
		       tmp, e->prio, fire);
		BUG_ON(!list_empty(&tmp->queue_node));
		list_add(&tmp->queue_node, pos->prev);
		el = tmp;
	}
	return el;
}

/*
 * Prepare a release list for a new set of events.
 */
static void reinit_event_list(struct event_group *group, struct rt_event *e)
{
	int prio, t_ret;
	struct event_list *el = e->event_list;

	VTRACE("Reinitting list 0x%x for event 0x%x\n", el, e);

	/* Cancel timer */
	t_ret = hrtimer_pull_cancel(group->cpu, &el->timer, &el->info);
	BUG_ON(t_ret == 1);
	if (t_ret == -1) {
		/* The on_timer callback is running for this list */
		VTRACE("Timer is running concurrently!\n");
	}
	/* Clear event lists */
	for (prio = 0; prio < NUM_EVENT_PRIORITIES; prio++)
		INIT_LIST_HEAD(&el->events[prio]);
}

/**
 * add_event() - Add timer to event group.
 */
void add_event(struct event_group *group, struct rt_event *e, lt_t fire)
{
	struct event_list *el;
	int in_use;

	VTRACE("Adding event 0x%x with priority %d for time %llu\n",
	       e, e->prio, fire);

	/* A NULL group means use the group of the currently executing CPU  */
	if (NULL == group)
		group = get_event_group_for(NO_CPU);
	/* Saving the group is important for cancellations */
	e->_event_group = group;

	raw_spin_lock(&group->queue_lock);
	el = get_event_list(group, e, fire, 0);
	if (!el) {
		/* Use our own, but drop lock first */
		raw_spin_unlock(&group->queue_lock);
		reinit_event_list(group, e);
		raw_spin_lock(&group->queue_lock);
		el = get_event_list(group, e, fire, 1);
	}

	/* Add event to sorted list */
	VTRACE("Inserting event 0x%x at end of event_list 0x%x\n", e, el);
	list_add(&e->events_node, &el->events[e->prio]);
	raw_spin_unlock(&group->queue_lock);

	/* Arm timer if we are the owner */
	if (el == e->event_list) {
		VTRACE("Arming timer on event 0x%x for %llu\n", e, fire);
		in_use = hrtimer_start_on(group->cpu, &el->info,
					  &el->timer, ns_to_ktime(el->fire_time),
					  HRTIMER_MODE_ABS_PINNED);
		BUG_ON(in_use);
	} else {
		VTRACE("Not my timer @%llu\n", fire);
	}
}

/**
 * cancel_event() - Remove event from the group.
 */
void cancel_event(struct rt_event *e)
{
	int prio, cancel;
	struct rt_event *swap, *entry;
	struct event_list *tmp;
	struct event_group *group;
	struct list_head *list, *pos;

	VTRACE("Canceling event 0x%x with priority %d\n", e, e->prio);
	group = e->_event_group;
	if (!group) return;

	raw_spin_lock(&group->queue_lock);

	/* Relies on the fact that an event_list's owner is ALWAYS present
	 * as one of the event_list's events.
	 */
	for (prio = 0, cancel = 0, swap = NULL;
	     prio < NUM_EVENT_PRIORITIES && !swap;
	     prio++) {

		list = &e->event_list->events[prio];
		cancel |= !list_empty(list);

		/* Find any element which is not the event_list's owner */
		list_for_each(pos, list) {
			entry = list_entry(pos, struct rt_event, events_node);
			if (entry != e) {
				swap = entry;
				break;
			}
		}
	}

	if (swap) {
		/* Give the other guy ownership of the event_list */
		VTRACE("Swapping list 0x%x with event 0x%x event list 0x%x\n",
		       e->event_list, swap, swap->event_list);
		tmp = swap->event_list;
		swap->event_list = e->event_list;
		BUG_ON(!tmp);
		e->event_list = tmp;
	} else if (cancel) {
		/* Cancel the event_list we own */
		hrtimer_pull_cancel(group->cpu,
				    &e->event_list->timer,
				    &e->event_list->info);
		list_del_init(&e->event_list->queue_node);
	}
	/* Remove ourselves from any list we may be a part of */
	list_del_init(&e->events_node);
	e->_event_group = NULL;

	raw_spin_unlock(&group->queue_lock);
}

struct kmem_cache *event_list_cache;

struct event_list* event_list_alloc(int gfp_flags)
{
	int prio;
	struct event_list *el = kmem_cache_alloc(event_list_cache, gfp_flags);
	if (el) {
		hrtimer_init(&el->timer, CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
		INIT_LIST_HEAD(&el->queue_node);
		el->timer.function = on_timer;
		hrtimer_start_on_info_init(&el->info);
		for (prio = 0; prio < NUM_EVENT_PRIORITIES; prio++)
			INIT_LIST_HEAD(&el->events[prio]);
	} else {
		VTRACE("Failed to allocate event list!\n");
		printk(KERN_CRIT "Failed to allocate event list.\n");
		BUG();
	}
	return el;
}

void init_event(struct rt_event *e, int prio, fire_event_t function,
		struct event_list *el)
{
	e->prio = prio;
	e->function = function;
	e->event_list = el;
	e->_event_group = NULL;
	INIT_LIST_HEAD(&e->events_node);
}

/**
 * init_event_group() - Prepare group for events.
 * @group	Group to prepare
 * @res		Timer resolution. Two events of @res distance will be merged
 * @cpu		Cpu on which to fire timers
 */
static void init_event_group(struct event_group *group, lt_t res, int cpu)
{
	int i;
	VTRACE("Creating group with resolution %llu on CPU %d", res, cpu);
	group->res = res;
	group->cpu = cpu;
	for (i = 0; i < EVENT_QUEUE_SLOTS; i++)
		INIT_LIST_HEAD(&group->event_queue[i]);
	raw_spin_lock_init(&group->queue_lock);
}


DEFINE_PER_CPU(struct event_group, _event_groups);

struct event_group *get_event_group_for(const int cpu)
{
	return &per_cpu(_event_groups,
			(NO_CPU == cpu) ? smp_processor_id() : cpu);
}

static int __init _init_event_groups(void)
{
	int cpu;
	printk("Initializing LITMUS^RT event groups.\n");

	for_each_online_cpu(cpu) {
		init_event_group(get_event_group_for(cpu),
				CONFIG_MERGE_TIMERS_WINDOW, cpu);
	}
	return 0;
}

module_init(_init_event_groups);
