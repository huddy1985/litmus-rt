#ifndef _LITMUS_BUDGET_H_
#define _LITMUS_BUDGET_H_

/**
 * update_enforcement_timer() - Update per-processor enforcement timer for
 * the next scheduled task.
 *
 * If @t is not NULL and has a precisely enforced budget, the timer will be
 * armed to trigger a reschedule when the budget is exhausted. Otherwise,
 * the timer will be cancelled.
*/
void update_enforcement_timer(struct task_struct* t);

/* True if a task's server has progressed farther than the task
 * itself. This happens when budget enforcement has caused a task to be
 * booted off until the next period.
 */
#define behind_server(t)\
	(lt_before((t)->rt_param.job_params.real_release, get_release(t)))

/**
 * server_release() - Prepare the task server parameters for the next period.
 * The server for @t is what is actually executed from the schedulers
 * perspective.
 */
void server_release(struct task_struct *t);

/**
 * task_release() - Prepare actual task parameters for the next period.
 * The actual task parameters for @t, real_deadline and real_release, are
 * the deadline and release from the tasks perspective. We only record these
 * so that we can write them to feather trace.
 */
void task_release(struct task_struct *t);
#endif
