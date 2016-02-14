#undef TRACE_SYSTEM
#define TRACE_SYSTEM sched_energy

#if !defined(_TRACE_SCHED_ENERGY_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_SCHED_ENERGY_H

#include <linux/sched.h>
#include <linux/tracepoint.h>

/*
 * Tracpoints for the energy scheduling class.
 */

/* Switching to the energy scheduling class. */
TRACE_EVENT(sched_energy_switch_from,
	TP_PROTO(int nr_energy, int nr_total, char reason),

	TP_ARGS(nr_energy, nr_total, reason),

	TP_STRUCT__entry(
		__field(int,	nr_energy)
		__field(int,	nr_total)
		__field(char,	reason)
	),

	TP_fast_assign(
		__entry->nr_energy = nr_energy;
		__entry->nr_total = nr_total;
		__entry->reason = reason;
	),

	TP_printk("energy=%d total=%d reason=%c", __entry->nr_energy, __entry->nr_total,
		__entry->reason)
);

/* Switching from the energy scheduling class. */
TRACE_EVENT(sched_energy_switch_to,
	TP_PROTO(int nr_energy, int nr_total, char reason),

	TP_ARGS(nr_energy, nr_total, reason),

	TP_STRUCT__entry(
		__field(int,	nr_energy)
		__field(int,	nr_total)
		__field(char,	reason)
	),

	TP_fast_assign(
		__entry->nr_energy = nr_energy;
		__entry->nr_total = nr_total;
		__entry->reason = reason;
	),

	TP_printk("energy=%d total=%d reason=%c", __entry->nr_energy, __entry->nr_total,
		__entry->reason)
);

/* Switching inside the energy scheduling class. */
TRACE_EVENT(sched_energy_switch_in,
	TP_PROTO(char reason),

	TP_ARGS(reason),

	TP_STRUCT__entry(
		__field(char,	reason)
	),

	TP_fast_assign(
		__entry->reason = reason;
	),

	TP_printk("reason=%c", __entry->reason)
);

/* Running the idle thread. */
TRACE_EVENT(sched_energy_run_idle,
	TP_PROTO(int nr_runnable),

	TP_ARGS(nr_runnable),

	TP_STRUCT__entry(
		__field(int,	nr_runnable)
	),

	TP_fast_assign(
		__entry->nr_runnable = nr_runnable;
	),

	TP_printk("runnable=%d", __entry->nr_runnable)
);

/* Running a normal thread. */
TRACE_EVENT(sched_energy_run_normal,
	TP_PROTO(struct task_struct* tsk, int nr_runnable),

	TP_ARGS(tsk, nr_runnable),

	TP_STRUCT__entry(
		__field(pid_t,	pid)
		__field(int,	nr_runnable)
	),

	TP_fast_assign(
		__entry->pid = tsk->pid;
		__entry->nr_runnable = nr_runnable;
	),

	TP_printk("pid=%d runnable=%d", __entry->pid, __entry->nr_runnable)
);

/* Enqueue task in the global runqueue. */
TRACE_EVENT(sched_energy_global_enqueue,
	TP_PROTO(struct task_struct* tsk, int nr_energy),

	TP_ARGS(tsk, nr_energy),

	TP_STRUCT__entry(
		__field(pid_t,	pid)
		__field(int,	nr_energy)
	),

	TP_fast_assign(
		__entry->pid = tsk->pid;
		__entry->nr_energy = nr_energy;
	),

	TP_printk("pid=%d energy=%d", __entry->pid, __entry->nr_energy)
);

/* Enqueue task in the local runqueue. */
TRACE_EVENT(sched_energy_local_enqueue,
	TP_PROTO(struct task_struct* tsk, int cpu, int nr_runnable, int nr_assigned),

	TP_ARGS(tsk, cpu, nr_runnable, nr_assigned),

	TP_STRUCT__entry(
		__field(pid_t,	pid)
		__field(int,	cpu)
		__field(int,	nr_runnable)
		__field(int,	nr_assigned)
	),

	TP_fast_assign(
		__entry->pid = tsk->pid;
		__entry->cpu = cpu;
		__entry->nr_runnable = nr_runnable;
		__entry->nr_assigned = nr_assigned;
	),

	TP_printk("pid=%d cpu=%d runnable=%d assigned=%d", __entry->pid, __entry->cpu,
		__entry->nr_runnable, __entry->nr_assigned)
);

/* Dequeue task in the global runqueue. */
TRACE_EVENT(sched_energy_global_dequeue,
	TP_PROTO(struct task_struct* tsk, int nr_energy),

	TP_ARGS(tsk, nr_energy),

	TP_STRUCT__entry(
		__field(pid_t,	pid)
		__field(int,	nr_energy)
	),

	TP_fast_assign(
		__entry->pid = tsk->pid;
		__entry->nr_energy = nr_energy;
	),

	TP_printk("pid=%d energy=%d", __entry->pid, __entry->nr_energy)
);

/* Dequeue task in the local runqueue. */
TRACE_EVENT(sched_energy_local_dequeue,
	TP_PROTO(struct task_struct* tsk, int cpu, int nr_runnable, int nr_assigned),

	TP_ARGS(tsk, cpu, nr_runnable, nr_assigned),

	TP_STRUCT__entry(
		__field(pid_t,	pid)
		__field(int,	cpu)
		__field(int,	nr_runnable)
		__field(int,	nr_assigned)
	),

	TP_fast_assign(
		__entry->pid = tsk->pid;
		__entry->cpu = cpu;
		__entry->nr_runnable = nr_runnable;
		__entry->nr_assigned = nr_assigned;
	),

	TP_printk("pid=%d cpu=%d runnable=%d assigned=%d", __entry->pid, __entry->cpu,
		__entry->nr_runnable, __entry->nr_assigned)
);

#endif /* _TRACE_SCHED_ENERGY_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
