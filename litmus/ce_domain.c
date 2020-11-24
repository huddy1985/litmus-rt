#include <linux/pid.h>
#include <linux/sched.h>
#include <linux/hrtimer.h>
#include <linux/slab.h>

#include <litmus/litmus.h>
#include <litmus/debug_trace.h>
#include <litmus/rt_param.h>
#include <litmus/domain.h>
#include <litmus/event_group.h>
#include <litmus/sched_mc.h>
#include <litmus/ce_domain.h>

/*
 * Called for:
 * task_new
 * job_completion
 * wake_up
 */
void ce_requeue(domain_t *dom, struct task_struct *ts)
{
	const struct ce_dom_data *ce_data = dom->data;
	const int idx = tsk_mc_data(ts)->mc_task.lvl_a_id;
	const unsigned int just_finished = tsk_rt(ts)->job_params.job_no;
	const unsigned int expected_job =
		mc_ce_get_expected_job(ce_data->cpu, idx);
	const int asleep = RT_F_SLEEP == get_rt_flags(ts);

	TRACE_MC_TASK(ts, "entered ce_requeue. asleep: %d  just_finished: %3u  "
			"expected_job: %3u\n",
			asleep, just_finished, expected_job);

	tsk_mc_data(ts)->mc_task.lvl_a_eligible = 1;

	/* When coming from job completion, the task will be asleep. */
	if (asleep && just_finished < expected_job) {
		TRACE_MC_TASK(ts, "appears behind\n");
	} else if (asleep && expected_job < just_finished) {
		TRACE_MC_TASK(ts, "job %u completed in expected job %u which "
				"seems too early\n", just_finished,
				expected_job);
	}
}

/*
 *
 */
void ce_remove(domain_t *dom, struct task_struct *ts)
{
	tsk_mc_data(ts)->mc_task.lvl_a_eligible = 0;
}

/*
 * ce_take_ready and ce_peek_ready
 */
struct task_struct* ce_peek_and_take_ready(domain_t *dom)
{
	const struct ce_dom_data *ce_data = dom->data;
	struct task_struct *ret = NULL, *sched = ce_data->should_schedule;
	const int exists = NULL != sched;
	const int blocked = exists && !is_running(sched);
	const int elig = exists && tsk_mc_data(sched) &&
		tsk_mc_data(sched)->mc_task.lvl_a_eligible;

	/* Return the task we should schedule if it is not blocked or sleeping. */
	if (exists && !blocked && elig)
		ret = sched;
	return ret;
}

int ce_higher_prio(struct task_struct *a, struct task_struct *b)
{
	const domain_t *dom = get_task_domain(a);
	const struct ce_dom_data *ce_data = dom->data;
	return (a != b && a == ce_data->should_schedule);
}

void ce_domain_init(domain_t *dom,
		raw_spinlock_t *lock,
		requeue_t requeue,
		peek_ready_t peek_ready,
		take_ready_t take_ready,
		preempt_needed_t preempt_needed,
		task_prio_t task_prio,
		struct ce_dom_data *dom_data,
		const int cpu,
		ce_timer_callback_t ce_timer_callback)
{
	domain_init(dom, lock, requeue, peek_ready, take_ready, preempt_needed,
			task_prio);
	dom->data = dom_data;
	dom->remove = ce_remove;
	dom_data->cpu = cpu;
#ifdef CONFIG_MERGE_TIMERS
	init_event(&dom_data->event, CRIT_LEVEL_A, ce_timer_callback,
			event_list_alloc(GFP_ATOMIC));
#else
	hrtimer_start_on_info_init(&dom_data->timer_info);
	hrtimer_init(&dom_data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
	dom_data->timer.function = ce_timer_callback;
#endif
}
