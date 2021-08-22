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
	TP_PROTO(int nr_energy, int nr_total, struct task_struct* task, char reason),

	TP_ARGS(nr_energy, nr_total, task, reason),

	TP_STRUCT__entry(
		__field(int,	nr_energy)
		__field(int,	nr_total)
		__field(pid_t,	pid)
		__field(char,	reason)
	),

	TP_fast_assign(
		__entry->nr_energy = nr_energy;
		__entry->nr_total = nr_total;
		__entry->pid = task ? task->pid : 0;
		__entry->reason = reason;
	),

	TP_printk("energy=%d total=%d task=%d reason=%c", __entry->nr_energy, __entry->nr_total,
		__entry->pid, __entry->reason)
);

/* Switching from the energy scheduling class. */
TRACE_EVENT(sched_energy_switch_to,
	TP_PROTO(int nr_energy, int nr_total, struct task_struct* task, char reason),

	TP_ARGS(nr_energy, nr_total, task, reason),

	TP_STRUCT__entry(
		__field(int,	nr_energy)
		__field(int,	nr_total)
		__field(pid_t,	pid)
		__field(char,	reason)
	),

	TP_fast_assign(
		__entry->nr_energy = nr_energy;
		__entry->nr_total = nr_total;
		__entry->pid = task ? task->pid : 0;
		__entry->reason = reason;
	),

	TP_printk("energy=%d total=%d task=%d reason=%c", __entry->nr_energy, __entry->nr_total,
		__entry->pid, __entry->reason)
);

/* Switching inside the energy scheduling class. */
TRACE_EVENT(sched_energy_switch_in,
	TP_PROTO(struct task_struct* task1, struct task_struct* task2, int nr_tasks, char reason),

	TP_ARGS(task1, task2, nr_tasks, reason),

	TP_STRUCT__entry(
		__field(pid_t,	pid1)
		__field(pid_t,	pid2)
		__field(int,	nr_tasks)
		__field(char,	reason)
	),

	TP_fast_assign(
		__entry->pid1 = task1 ? task1->pid : 0;
		__entry->pid2 = task2 ? task2->pid : 0;
		__entry->nr_tasks = nr_tasks;
		__entry->reason = reason;
	),

	TP_printk("task1=%d task2=%d tasks=%d reason=%c", __entry->pid1, __entry->pid2,
		__entry->nr_tasks, __entry->reason)
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

/* Executed a remote reschedule request. */
TRACE_EVENT(sched_energy_resched_cpu,
	TP_PROTO(int nr_normal, int nr_runnable, int nr_assigned, int remote),

	TP_ARGS(nr_normal, nr_runnable, nr_assigned, remote),

	TP_STRUCT__entry(
		__field(int,	nr_normal)
		__field(int,	nr_runnable)
		__field(int,	nr_assigned)
		__field(int,	remote)
	),

	TP_fast_assign(
		__entry->nr_normal = nr_normal;
		__entry->nr_runnable = nr_runnable;
		__entry->nr_assigned = nr_assigned;
		__entry->remote = remote;
	),

	TP_printk("normal=%d runnable=%d assigned=%d remote=%d", __entry->nr_normal,
		__entry->nr_runnable, __entry->nr_assigned, __entry->remote)
);

/* The CPU is managed to enable time sharing with CFS. */
TRACE_EVENT(sched_energy_manage_cpu,
	TP_PROTO(int state, int new_state, int nr_normal, int nr_assigned, int remote),

	TP_ARGS(state, new_state, nr_normal, nr_assigned, remote),

	TP_STRUCT__entry(
		__field(char,	state)
		__field(char,	new_state)
		__field(int,	nr_normal)
		__field(int,	nr_assigned)
		__field(int,	remote)
	),

	TP_fast_assign(
		__entry->state = state == 0x1 ? 'B' : 'U';
		__entry->new_state = new_state == 0x1 ? 'B' : 'U';
		__entry->nr_normal = nr_normal;
		__entry->nr_assigned = nr_assigned;
		__entry->remote = remote;
	),

	TP_printk("%c->%c normal=%d assigned=%d remote=%d", __entry->state, __entry->new_state,
		__entry->nr_normal, __entry->nr_assigned, __entry->remote)
);

/* An energy scheduling task gets removed from the CPU. */
TRACE_EVENT(sched_energy_put_prev,
	TP_PROTO(struct task_struct* tsk),

	TP_ARGS(tsk),

	TP_STRUCT__entry(
		__field(pid_t,	pid)
	),

	TP_fast_assign(
		__entry->pid = tsk->pid;
	),

	TP_printk("pid=%d", __entry->pid)
);

/* An energy scheduling task gets assigned to a CPU. */
TRACE_EVENT(sched_energy_set_curr,
	TP_PROTO(struct task_struct* tsk),

	TP_ARGS(tsk),

	TP_STRUCT__entry(
		__field(pid_t,	pid)
	),

	TP_fast_assign(
		__entry->pid = tsk->pid;
	),

	TP_printk("pid=%d", __entry->pid)
);

/* The energy scheduling class's pick_next_task was called. */
TRACE_EVENT(sched_energy_missing_resched_curr_local,
	TP_PROTO(int nr_normal, int nr_runnable),

	TP_ARGS(nr_normal, nr_runnable),

	TP_STRUCT__entry(
		__field(int,	nr_normal)
		__field(int,	nr_runnable)
	),

	TP_fast_assign(
		__entry->nr_normal = nr_normal;
		__entry->nr_runnable = nr_runnable;
	),

	TP_printk("normal=%d runnable=%d", __entry->nr_normal, __entry->nr_runnable)
);

/* The energy scheduling class's pick_next_task was called. */
TRACE_EVENT(sched_energy_pick_next_task,
	TP_PROTO(struct task_struct* tsk, int nr_normal, int nr_runnable, int nr_assigned),

	TP_ARGS(tsk, nr_normal, nr_runnable, nr_assigned),

	TP_STRUCT__entry(
		__field(pid_t,	pid)
		__field(int,	nr_normal)
		__field(int,	nr_runnable)
		__field(int,	nr_assigned)
	),

	TP_fast_assign(
		__entry->pid = tsk ? tsk->pid : -1;
		__entry->nr_normal = nr_normal;
		__entry->nr_runnable = nr_runnable;
		__entry->nr_assigned = nr_assigned;
	),

	TP_printk("pid=%d normal=%d runnable=%d assigned=%d", __entry->pid,
		__entry->nr_normal, __entry->nr_runnable, __entry->nr_assigned)
);

TRACE_EVENT(sched_energy_read_rapl_counters_before,
	TP_PROTO(bool wait),

	TP_ARGS(wait),

	TP_STRUCT__entry(
		__field(bool,	wait)
	),

	TP_fast_assign(
		__entry->wait = wait;
	),

	TP_printk("wait=%i", __entry->wait)
);

TRACE_EVENT(sched_energy_read_rapl_counters_after,
	TP_PROTO(bool wait, u64 duration),

	TP_ARGS(wait, duration),

	TP_STRUCT__entry(
		__field(bool,	wait)
		__field(u64,	duration)
	),

	TP_fast_assign(
		__entry->wait = wait;
		__entry->duration = duration;
	),

	TP_printk("wait=%i duration=%llu", __entry->wait, __entry->duration)
);

TRACE_EVENT(sched_energy_power_usage,
	TP_PROTO(struct task_struct* tsk, u32 pkg, u32 dram, u32 core),

	TP_ARGS(tsk, pkg, dram, core),

	TP_STRUCT__entry(
		__field(pid_t,	pid)
		__field(u32,	pkg)
		__field(u32,	dram)
		__field(u32,	core)
	),

	TP_fast_assign(
		__entry->pid = tsk ? tsk->pid : -1;
		__entry->pkg = pkg;
		__entry->dram = dram;
		__entry->core = core;
	),

	TP_printk("pid=%d pkg=%u dram=%u core=%u", __entry->pid, __entry->pkg, __entry->dram,
		__entry->core)
);

#endif /* _TRACE_SCHED_ENERGY_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
