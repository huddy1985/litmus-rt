#ifndef _LINUX_EVENT_QUEUE_H_
#define _LINUX_EVENT_QUEUE_H_

#define EVENT_QUEUE_SLOTS 127 /* prime */

#define NUM_EVENT_PRIORITIES 4 /* num crit levels really */

struct rt_event;
typedef void (*fire_event_t)(struct rt_event *e);

struct event_group {
	lt_t 			res;
	int 			cpu;
	struct list_head 	event_queue[EVENT_QUEUE_SLOTS];
	raw_spinlock_t 		queue_lock;
};

/**
 * A group of actions to fire at a given time
 */
struct event_list {
	/* Use multiple list heads so that inserts are O(1) */
	struct list_head events[NUM_EVENT_PRIORITIES];

	/* For timer firing */
	lt_t 				fire_time;
	struct hrtimer 			timer;
	struct hrtimer_start_on_info 	info;

	struct list_head    queue_node;  /* For event_queue */
	struct event_group* group; /* For callback    */
};

/**
 * A single action to fire at a time
 */
struct rt_event {
	/* Function to call on event expiration */
	fire_event_t 	 function;
	/* Priority of this event (lower is better) */
	int 		 prio;

	/* For membership in the event_list */
	struct list_head 	events_node;
	/* To avoid runtime allocation. This is NOT necessarily
	 * the event_list containing this event. This is just a
	 * pre-allocated event list which can be used for merging
	 * events.
	 */
	struct event_list* 	event_list;
	/* Pointer set by add_event() so that we can cancel this event
	 * without knowing what group it is in (don't touch it).
	 */
	struct event_group*	_event_group;
};

/**
 * add_event() - Add timer to event group.
 * @group	Group with which to merge event. If NULL, use the event
 *		group of whatever CPU currently executing on.
 * @e		Event to be fired at a specific time
 * @time	Time to fire event
 */
void add_event(struct event_group* group, struct rt_event* e, lt_t time);

/**
 * cancel_event() - Remove event from the group.
 */
void cancel_event(struct rt_event*);

/**
 * init_event() - Create an event.
 * @e		Event to create
 * @prio 	Priority of the event (lower is better)
 * @function	Function to fire when event expires
 * @el		Pre-allocated event list for timer merging
 */
void init_event(struct rt_event* e, int prio, fire_event_t function,
		struct event_list *el);

struct event_list* event_list_alloc(int);
void event_list_free(struct event_list *el);

/**
 * get_event_group_for() - Get the event group for a CPU.
 * @cpu		The CPU to get the event group for. Use NO_CPU to get the
 *		event group of the CPU that the call is executing on.
 */
struct event_group *get_event_group_for(const int cpu);

#endif
