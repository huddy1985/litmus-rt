#ifndef _SYS_TRACE_H_
#define	_SYS_TRACE_H_

#ifdef CONFIG_SCHED_OVERHEAD_TRACE

#include <litmus/feather_trace.h>
#include <litmus/feather_buffer.h>


/*********************** TIMESTAMPS ************************/

enum task_type_marker {
	TSK_BE,
	TSK_RT,
	TSK_UNKNOWN,
	TSK_LVLA
};

struct timestamp {
	uint64_t		timestamp;
	uint32_t		seq_no;
	uint8_t			cpu;
	uint8_t			event;
	uint8_t			task_type;
};

/* tracing callbacks */
feather_callback void save_timestamp(unsigned long event);
feather_callback void save_timestamp_def(unsigned long event, unsigned long type);
feather_callback void save_timestamp_task(unsigned long event, unsigned long t_ptr);
feather_callback void save_timestamp_cpu(unsigned long event, unsigned long cpu);
feather_callback void save_task_latency(unsigned long event, unsigned long when_ptr);
feather_callback void save_timestamp_time(unsigned long event, unsigned long time_ptr);

#define TIMESTAMP(id) ft_event0(id, save_timestamp)

#define DTIMESTAMP(id, def)  ft_event1(id, save_timestamp_def, (unsigned long) def)

#define TTIMESTAMP(id, task) \
	ft_event1(id, save_timestamp_task, (unsigned long) task)

#define CTIMESTAMP(id, cpu) \
	ft_event1(id, save_timestamp_cpu, (unsigned long) cpu)

#define LTIMESTAMP(id, task) \
	ft_event1(id, save_task_latency, (unsigned long) task)

#define TIMESTAMP_TIME(id, time_ptr) \
	ft_event1(id, save_timestamp_time, (unsigned long) time_ptr)

#define TIMESTAMP_PID(id) ft_event0(id, save_timestamp_pid)

#else /* !CONFIG_SCHED_OVERHEAD_TRACE */

#define TIMESTAMP(id)        /* no tracing */

#define DTIMESTAMP(id, def)  /* no tracing */

#define TTIMESTAMP(id, task) /* no tracing */

#define CTIMESTAMP(id, cpu)  /* no tracing */

#define LTIMESTAMP(id, when_ptr) /* no tracing */

#define TIMESTAMP_TIME(id, time_ptr) /* no tracing */

#define TIMESTAMP_PID(id) /* no tracing */

#endif


/* Convention for timestamps
 * =========================
 *
 * In order to process the trace files with a common tool, we use the following
 * convention to measure execution times: The end time id of a code segment is
 * always the next number after the start time event id.
 */

#define __TS_SYSCALL_IN_START(p)	TIMESTAMP_TIME(10, p)
#define TS_SYSCALL_IN_END		TIMESTAMP_PID(11)

#define TS_SYSCALL_OUT_START		TIMESTAMP_PID(20)
#define TS_SYSCALL_OUT_END		TIMESTAMP_PID(21)

#define TS_LOCK_START			TIMESTAMP_PID(30)
#define TS_LOCK_END			TIMESTAMP_PID(31)

#define TS_LOCK_SUSPEND			TIMESTAMP_PID(38)
#define TS_LOCK_RESUME			TIMESTAMP_PID(39)

#define TS_UNLOCK_START			TIMESTAMP_PID(40)
#define TS_UNLOCK_END			TIMESTAMP_PID(41)

#define TS_SCHED_START			DTIMESTAMP(100, TSK_UNKNOWN) /* we only
								      * care
								      * about
								      * next */
#define TS_SCHED_END(t)			TTIMESTAMP(101, t)
#define TS_SCHED2_START(t) 		TTIMESTAMP(102, t)
#define TS_SCHED2_END(t)       		TTIMESTAMP(103, t)

#define TS_CXS_START(t)			TTIMESTAMP(104, t)
#define TS_CXS_END(t)			TTIMESTAMP(105, t)

#define TS_RELEASE_START		DTIMESTAMP(106, TSK_RT)
#define TS_RELEASE_END			DTIMESTAMP(107, TSK_RT)

#define TS_TICK_START(t)		TTIMESTAMP(110, t)
#define TS_TICK_END(t) 			TTIMESTAMP(111, t)

#define TS_LVLA_RELEASE_START		DTIMESTAMP(112, TSK_RT)
#define TS_LVLA_RELEASE_END		DTIMESTAMP(113, TSK_RT)

#define TS_LVLA_SCHED_START		DTIMESTAMP(114, TSK_UNKNOWN)
#define TS_LVLA_SCHED_END_ID		115
#define TS_LVLA_SCHED_END(t)		TTIMESTAMP(TS_LVLA_SCHED_END_ID, t)


#define TS_PLUGIN_SCHED_START		/* TIMESTAMP(120) */  /* currently unused */
#define TS_PLUGIN_SCHED_END		/* TIMESTAMP(121) */

#define TS_PLUGIN_TICK_START		/* TIMESTAMP(130) */
#define TS_PLUGIN_TICK_END		/* TIMESTAMP(131) */

#define TS_ENTER_NP_START		TIMESTAMP(140)
#define TS_ENTER_NP_END			TIMESTAMP(141)

#define TS_EXIT_NP_START		TIMESTAMP(150)
#define TS_EXIT_NP_END			TIMESTAMP(151)

#define TS_SEND_RESCHED_START(c)	CTIMESTAMP(190, c)
#define TS_SEND_RESCHED_END		DTIMESTAMP(191, TSK_UNKNOWN)

#define TS_RELEASE_LATENCY(when)	LTIMESTAMP(208, &(when))

#endif /* !_SYS_TRACE_H_ */
