/* vim: set noet ts=8 sw=8 sts=8 : */

#include <linux/cpumask.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/smp.h>
#include <linux/spinlock.h>
#include <linux/kthread.h>

#include <linux/sched.h>

#include "sched.h"
#include "cpuacct.h"


/***
 * Internal constants
 ***/

/* Thread states. */
enum {
	/* Thread RQ-queued states. */
	THREAD_RQ_RUNNABLE = 0x1,
	THREAD_RQ_QUEUED = THREAD_RQ_RUNNABLE,

	/* Thread CPU-queued states. */
	THREAD_CPU_RUNNABLE = 0x2,
	THREAD_CPU_QUEUED = THREAD_CPU_RUNNABLE,

	/* Thread running states. */
	THREAD_CPU_RUNNING = 0x4
};

/* Reschedule states. */
enum {
	LOCAL_RESCHED = 0x1,
	ENERGY_RESCHED = 0x2
};

/* The default scheduling slice for one thread. --> 10ms <-- */
#define THREAD_SCHED_SLICE 10000000ULL


/***
 * Internal data structure prototypes.
 ***/

struct energy_task;
struct global_rq;


/***
 * Internal data structure definitions.
 ***/

/* The representation of a task which should be run which energy accounting
 * enabled. */
struct energy_task {
	/* Is it currently running. */
	int running;

	/* The task struct belonging to the real task. */
	struct task_struct* task;

	/* The energy domain where the task should run. */
	struct cpumask domain;

	/* All runnable threads. */
	struct list_head runnable;
	u32 nr_runnable;

	/* The link in the global runqueue. */
	struct list_head rq;

	/* Runtime statistics */
	u64 start_running;
};

/* The global runqueue for all task with their corresponding threads which
 * are managed by this scheduling class. */
struct global_rq {
	/* Lock for the global runqueue. */
	raw_spinlock_t lock;

	/* Is the scheduling class currently running. */
	int running;

	/* All energy tasks. */
	struct list_head tasks;
	u32 nr_tasks;

	/* The total number of runnable threads. */
	u32 nr_threads;

	/* Runtime statistics */
	u64 start_running;
	u64 stop_running;
};


/***
 * Internal variables.
 ***/

static struct global_rq grq;


/***
 * Internal function prototypes.
 ***/

/* Working with the global runqueue. */
static void init_grq(void);
static void lock_grq(void);
static void unlock_grq(void);

/* Working with the energy runqueues. */
static void lock_local_rq(struct rq*);
static void unlock_local_rq(struct rq*);

/* Get the real task belonging to a linux task. */
static struct task_struct* find_task(struct task_struct*);

/* Init energy tasks. */
static void init_energy_task(struct energy_task*);

/* Energy tasks and the global runqueue. */
static void enqueue_energy_task(struct energy_task*);
static void dequeue_energy_task(struct energy_task*);

/* Find an energy task belonging to a linux task. */
static struct energy_task* find_energy_task(struct task_struct*);
static struct energy_task* create_energy_task(struct task_struct*);
static void free_energy_task(struct energy_task*);

/* Threads and energy tasks. */
static bool thread_in_rq_queued(struct task_struct*);
static bool thread_in_cpu_rq_queued(struct task_struct*);
static bool thread_cpu_running(struct task_struct*);

static void enqueue_runnable(struct rq*, struct energy_task*,
		struct task_struct*);
static void dequeue_runnable(struct energy_task*, struct task_struct*);

static void enqueue_running(struct rq*, struct task_struct*);
static void dequeue_running(struct task_struct*);

/* Move threads between scheduling classes. */
static void raise_thread(struct task_struct*, struct energy_task*);
static void lower_thread(struct task_struct*, struct energy_task*,
		unsigned int, const struct sched_class*);

/* Determining the scheduling slices. */
static u64 sched_slice_class(void);
static u64 sched_slice_energy(struct energy_task*);
static u64 sched_slice_local(struct rq*);
static u64 sched_slice_other(void);

/* Should we perform a scheduling operation? */
static bool should_switch_to_energy(struct rq*);
static bool should_switch_from_energy(struct rq*);
static bool should_switch_energy(struct rq*);
static bool should_switch_local(struct rq*);

/* Should we redistribute an energy task again? */
static bool should_redistribute_energy(struct energy_task*,
		struct task_struct*);

/* Set runqueue bits to perform scheduling operations. */
static void resched_curr_energy(struct rq*);
static bool need_resched_curr_energy(struct rq*);
static void clear_resched_curr_energy(struct rq*);

static void resched_curr_local(struct rq*);
static bool need_resched_curr_local(struct rq*);
static void clear_resched_curr_local(struct rq*);

/* Update runtime statistics. */
static void update_energy_statistics(struct energy_task*);
static void update_local_statistics(struct rq*, struct task_struct*);

/* Schedule and remove energy tasks. */
static void set_energy_task(struct rq*, struct energy_task*);
static void set_local_task(struct rq*, struct task_struct*);

static void distribute_energy_task(struct rq*, struct energy_task*);
static void distribute_local_task(struct rq*, struct task_struct*);

static void redistribute_energy_task(struct rq*, struct energy_task*, bool);

static void move_local_task(struct task_struct*, unsigned int);

static void clear_energy_task(struct rq*, struct energy_task*);
static void clear_local_tasks(struct rq*);

static void put_energy_task(struct rq*, struct energy_task*);
static void put_local_task(struct rq*, struct task_struct*);

static struct energy_task* pick_next_energy_task(void);
static struct task_struct* pick_next_local_task(struct rq*);

/* Initialize the energy domain. */
static void init_energy_domain(struct cpumask*, unsigned int);

/* Idle function. */
static int idle_thread_fn(void*);


/***
 * External function prototypes
 ***/

/* Functions needed for the scheduling class implementation. */
void enqueue_task_energy(struct rq*, struct task_struct*, int);
void dequeue_task_energy(struct rq*, struct task_struct*, int);

void yield_task_energy(struct rq*);
bool yield_to_task_energy(struct rq*, struct task_struct*, bool);

void check_preempt_curr_energy(struct rq*, struct task_struct*, int);

struct task_struct* pick_next_task_energy(struct rq*, struct task_struct*);

void put_prev_task_energy(struct rq*, struct task_struct*);

void set_curr_task_energy(struct rq*);

void task_tick_energy(struct rq*, struct task_struct*, int);
void task_fork_energy(struct task_struct*);
void task_dead_energy(struct task_struct*);

void switched_from_energy(struct rq*, struct task_struct*);
void switched_to_energy(struct rq*, struct task_struct*);
void prio_changed_energy(struct rq*, struct task_struct*, int);

unsigned int get_rr_interval_energy(struct rq*, struct task_struct*);

void update_curr_energy(struct rq*);

int select_task_rq_energy(struct task_struct*, int, int, int);
void migrate_task_rq_energy(struct task_struct*, int);

void task_waking_energy(struct task_struct*);
void task_woken_energy(struct rq*, struct task_struct*);

void set_cpus_allowed_energy(struct task_struct*, const struct cpumask*);

void rq_online_energy(struct rq*);
void rq_offline_energy(struct rq*);


/***
 * Internal function definitions.
 ***/

/* Initialize the global runqueue. */
static void init_grq(void) {
	raw_spin_lock_init(&(grq.lock));

	INIT_LIST_HEAD(&(grq.tasks));
	grq.nr_tasks = 0;
	grq.nr_threads = 0;

	grq.stop_running = 0;
	grq.start_running = 0;
}

/* Lock the global runqueue. */
static void lock_grq(void) __acquires(grq.lock) {
	do_raw_spin_lock(&(grq.lock));
}

/* Unlock the global runqueue. */
static void unlock_grq(void) __releases(grq.lock) {
	do_raw_spin_unlock(&(grq.lock));
}

/* Lock the local energy rq embedded in the CPU runqueues.
 *
 * @rq:		the runqueue of the current CPU.
 */
static void lock_local_rq(struct rq* rq) __acquires(rq->en.lock) {
	do_raw_spin_lock(&(rq->en.lock));
}

/* Unlock the local energy rq embedded in the CPU runqueue.
 *
 * @rq:		the runqueue of the current CPU.
 */
static void unlock_local_rq(struct rq* rq) __releases(rq->en.lock) {
	do_raw_spin_unlock(&(rq->en.lock));
}

/* Find the real task corresponding to a task struct.
 *
 * @t:		the task struct for which the real task should be returned.
 *
 * @returns:	the real task corresponding to the given task struct.
 */
static struct task_struct* find_task(struct task_struct* t) {
	if (thread_group_leader(t)) {
		return t;
	} else {
		return t->group_leader;
	}
}

/* Initialize an energy task struct.
 *
 * @e_task:	the pointer to the energy task, which should be
 *		initialized.
 */
static void init_energy_task(struct energy_task* e_task) {
	e_task->running = 0;

	e_task->task = NULL;

	cpumask_clear(&(e_task->domain));

	INIT_LIST_HEAD(&(e_task->runnable));
	e_task->nr_runnable = 0;

	e_task->start_running = 0;
}

/* Enqueue an energy task in the global runqueue.
 *
 * Requires that the lock of the global runqueue is taken.
 *
 * @e_task:	the energy task which should be enqueued.
 */
static void enqueue_energy_task(struct energy_task* e_task) {
	list_add(&(e_task->rq), &(grq.tasks));
	grq.nr_tasks++;
}

/* Dequeue an energy task from the global runqueue.
 *
 * Requires that the lock of the global runqueue is taken.
 *
 * @e_task:	the energy task which should be dequeued.
 */
static void dequeue_energy_task(struct energy_task* e_task) {
	list_del(&(e_task->rq));
	grq.nr_tasks--;
}

/* Find the energy task struct corresponding to a linux task t.
 *
 * Requires that the lock of the global rq is taken.
 *
 * @t:		the task struct of the linux task for which the corresponding
 *		energy task struct should be returned.
 *
 * @returns:	the energy task struct corresponding to the given linux task.
 */
static struct energy_task* find_energy_task(struct task_struct* t) {
	struct task_struct* task = find_task(t);
	struct energy_task* e_task;

	list_for_each_entry(e_task, &(grq.tasks), rq) {
		if (e_task->task == task) {
			return e_task;
		}
	}

	return NULL;
}

/* Create an energy task corresponding to a linux task t.
 *
 * Requires that the lock of the global rq is taken.
 *
 * @t:		the task struct of the linux task for which the corresponding
 *		energy task should be created.
 */
static struct energy_task* create_energy_task(struct task_struct* t) {
	struct task_struct* task = find_task(t);
	struct energy_task* e_task;

	/* Create and initialize a new energy task. We are using  GFP_ATOMIC here
	 * as we are currently holding a lock. */
	e_task = kmalloc(sizeof(struct energy_task), GFP_ATOMIC);
	if (e_task == NULL) {
		return NULL;
	}

	init_energy_task(e_task);

	/* Remember the task struct for the actual task. */
	e_task->task = task;

	/* Enqueue the created task in the global runqueue. */
	enqueue_energy_task(e_task);

	return e_task;
}

/* Free an energy task again.
 *
 * Requires that the lock of the global rq is taken.
 *
 * @e_task:	the energy task which should be freed again.
 */
static void free_energy_task(struct energy_task* e_task) {
	dequeue_energy_task(e_task);
	kfree(e_task);
}

/* Check if a thread is queued in the scheduling class runqueue.
 *
 * @t:		the task struct of the thread which should be tested.
 *
 * @returns:	whether or not the thread is queued in the runqueue or not.
 */
static inline bool thread_in_rq_queued(struct task_struct* t) {
	return t->ee.state & THREAD_RQ_QUEUED;
}

/* Check if a thread is queued in the CPU runqueue.
 *
 * @t:		the task struct of the thread which should be tested.
 *
 * @returns:	whether or not the thread is queued in the CPU runqueue or not.
 */
static inline bool thread_in_cpu_rq_queued(struct task_struct* t) {
	return t->ee.state & THREAD_CPU_QUEUED;
}

/* Check if a thread is currently running on a CPU.
 *
 * @t:		the task struc of the thread which should be tested.
 *
 * @returns:	whether or not the thread is currently running on a CPU.
 */
static inline bool thread_cpu_running(struct task_struct* t) {
	return t->ee.state & THREAD_CPU_RUNNING;
}

/* Enqueue a thread into the list of running threads of the energy task.
 *
 * @rq:		the runqueue of the current CPU.
 * @e_task:	the energy task struct of the energy task to which the thread
 *		belongs.
 * @t:		the task struct of the thread which should be enqueued.
 */
static void enqueue_runnable(struct rq* rq, struct energy_task* e_task, struct task_struct* t) {
	if (thread_in_rq_queued(t)) {
		/* This thread is already enqueued in the runqueue. */
		BUG();
	}

	list_add(&(t->ee.rq), &(e_task->runnable));
	e_task->nr_runnable++;

	t->ee.state |= THREAD_RQ_RUNNABLE;

	/* Remember in the global runqueue that we have a runnable thread. */
	grq.nr_threads++;

	/* Remember in the runqueue that there is now a new runnable linux task. */
	rq->en.nr_assigned++;
	add_nr_running(rq, 1);
}

/* Enqueue a thread into the list of running threads of a CPU.
 *
 * Requires that the lock of the local runqueue is taken.
 *
 * @rq:		the runqueue of the current CPU.
 * @t:		the task struct fo the thread which should be enqueued.
 */
static void enqueue_running(struct rq* rq, struct task_struct* t) {
	if (thread_in_cpu_rq_queued(t)) {
		/* The thread is already enqueued in the runqueue. */
		BUG();
	}

	list_add(&(t->ee.cpu_rq), &(rq->en.runnable));
	rq->en.nr_runnable++;

	t->ee.state |= THREAD_CPU_RUNNABLE;
}

/* Dequeue a thread from the list of running threads of the energy task.
 *
 * @e_task:	the energy task struct of the energy task to which the thread
 *		belongs.
 * @t:		the task struct of the thread which should be dequeued.
 */
static void dequeue_runnable(struct energy_task* e_task, struct task_struct* t) {
	if (!thread_in_rq_queued(t)) {
		/* This thread is not enqueued in the runqueue. */
		BUG();
	}

	list_del(&(t->ee.rq));
	e_task->nr_runnable--;

	t->ee.state &= ~THREAD_RQ_RUNNABLE;

	/* Remember in the global runqueue that the thread is no longer runnable. */
	grq.nr_threads--;

	/* Remember in the runqueue that the thread is no longer runnable. */
	task_rq(t)->en.nr_assigned--;
	sub_nr_running(task_rq(t), 1);
}

/* Dequeue a thread from the list of running threads on a CPU runqueue.
 *
 * Requires that the local runqueue lock is taken.
 *
 * @t:		the task struct of the thread which should be dequeued.
 */
static void dequeue_running(struct task_struct* t) {
	if (!thread_in_cpu_rq_queued(t)) {
		/* This thread is not enqueued in the runqueue. */
		BUG();
	}

	list_del(&(t->ee.cpu_rq));
	task_rq(t)->en.nr_runnable--;

	t->ee.state &= ~THREAD_CPU_QUEUED;
}

/* Raise a thread from a different scheduling class into the energy scheduling class.
 *
 * @t:		the task struct of the thread which should be raised into the energy
 *		scheduling class.
 * @e_task:	the energy task to which the thread belongs.
 */
static void raise_thread(struct task_struct* t, struct energy_task* e_task) {
	int running, queued;

	if (t->sched_class == &energy_sched_class) {
		/* Nothing to do here, this thread is already part of our scheduling
		 * class. */
		return;
	}

	/* Lock the task's runqueue if necessary. */
	if (task_rq(t) != this_rq()) {
		lockdep_unpin_lock(this_rq()->lock);
		double_lock_balance(this_rq(), task_rq(t));
	}

	running = task_current(task_rq(t), t);
	queued = task_on_rq_queued(t);

	if (queued)
		t->sched_class->dequeue_task(task_rq(t), t, 0);
	if (running)
		t->sched_class->put_prev_task(task_rq(t), t);

	t->sched_class = &energy_sched_class;
	t->policy = SCHED_ENERGY;

	if (running)
		set_curr_task_energy(task_rq(t));
	if (queued)
		enqueue_runnable(task_rq(t), e_task, t);

	/* Unlock the task's runqueue again if necessary. */
	if (task_rq(t) != this_rq()) {
		double_unlock_balance(this_rq(), task_rq(t));
		lockdep_pin_lock(this_rq()->lock);
	}
}

/* Lower a thread from the energy scheduling class into a different scheduling class.
 *
 * @t:		the task struct of the thread which should be lowered into a different
 *		scheduling class.
 * @e_task:	the energy task to which the thread belongs.
 * @policy:	the policy of the new scheduling class.
 * @class:	the new scheduling class.
 */
static void lower_thread(struct task_struct* t, struct energy_task* e_task,
		unsigned int policy, const struct sched_class* class) {
	int running, queued;

	if (t->sched_class != &energy_sched_class) {
		/* Nothing to do here, this thread is not part anymore of the energy
		 * scheduling class. */
		return;
	}

	/* Lock the task's runqueue if necessary. */
	if (task_rq(t) != this_rq()) {
		lockdep_unpin_lock(this_rq()->lock);
		double_lock_balance(this_rq(), task_rq(t));
	}

	running = task_current(task_rq(t), t);
	queued = task_on_rq_queued(t);

	if (queued)
		dequeue_runnable(e_task, t);
	if (running)
		put_prev_task_energy(task_rq(t), t);

	t->sched_class = class;
	t->policy = policy;

	if (running)
		t->sched_class->set_curr_task(task_rq(t));
	if (queued)
		t->sched_class->enqueue_task(task_rq(t), t, 0);

	/* Unlock the task's runqueue again if necessary. */
	if (task_rq(t) != this_rq()) {
		double_unlock_balance(this_rq(), task_rq(t));
		lockdep_pin_lock(this_rq()->lock);
	}
}

/* Calculate the time which the energy scheduling class should run.
 *
 * @returns:	the runtime for the energy scheduling class.
 */
static inline u64 sched_slice_class(void) {
	return grq.nr_threads * THREAD_SCHED_SLICE;
}

/* Calculate the time which the current energy task should run.
 *
 * @returns:	the runtime for the energy task.
 */
static inline u64 sched_slice_energy(struct energy_task* e_task) {
	/* The energy scheduling slice is simply the class scheduling slice
	 * distributed equally between the energy tasks. */
	return e_task->nr_runnable * THREAD_SCHED_SLICE;
}

/* Calculate the time which a thread assigned to CPU should run.
 *
 * @rq:		the runqueue of the current CPU.
 *
 * @returns:	the runtime for the current thread.
 */
static inline u64 sched_slice_local(struct rq* rq) {
	/* The local scheduling slice is simply the energy scheduling slice
	 * distributed equally between the threads assigned to one CPU. */
	return rq->en.nr_runnable != 0 ?
		sched_slice_energy(rq->en.curr_e_task) / rq->en.nr_runnable :
		sched_slice_energy(rq->en.curr_e_task);
}

/* Calculate the time which other scheduling classes should run.
 *
 * @returns:	the runtime for other scheduling class.
 */
static inline u64 sched_slice_other(void) {
	return (nr_running() - grq.nr_threads) * THREAD_SCHED_SLICE;
}

/* Decide if we should switch to the energy sched class from another one.
 *
 * @rq:		the runqueue of the current CPU.
 *
 * @returns:	whether we should switch or not.
 */
static inline bool should_switch_to_energy(struct rq* rq) {
	if (grq.nr_threads == 0) {
		/* We have no threads to schedule currently. */
		return false;
	} else if (nr_running() == grq.nr_threads) {
		/* There are only threads of energy tasks in the system. */
		return true;
	} else {
		u64 now = rq_clock(rq);
		u64 not_running = now <= grq.stop_running ? 0 : now - grq.stop_running;

		return not_running > sched_slice_other();
	}
}

/* Decide if we should switch away from the energy scheduling class to another
 * one.
 *
 * @rq:		the runqueue of the current CPU.
 *
 * @returns:	whether we should switch or not.
 */
static inline bool should_switch_from_energy(struct rq* rq) {
	if (grq.nr_threads == 0) {
		/* We have no threads to schedule currently. */
		return true;
	} else if (nr_running() == grq.nr_threads) {
		/* There are only threads of energy tasks in the system. */
		return false;
	} else {
		u64 now = rq_clock(rq);
		u64 running = now <= grq.start_running ? 0 : now - grq.start_running;

		return running > sched_slice_class();
	}
}

/* Decide if we should switch to another energy task.
 *
 * Requires that the global runqueue lock is taken.
 *
 * @rq:		the runqueue of the current CPU.
 *
 * @returns:	whether we should switch or not.
 */
static inline bool should_switch_energy(struct rq* rq) {
	if (grq.nr_tasks <= 1) {
		/* We can only switch between energy tasks if there are more than
		 * one energy task in the global runqueue.*/
		return false;
	} else {
		/* Ok, we have more than one energy task. So decide based on the runtime
		 * of the energy task. */
		struct energy_task* e_task = rq->en.curr_e_task;

		u64 now = rq_clock(rq);
		u64 running = now <= e_task->start_running ? 0 : now - e_task->start_running;

		return running > sched_slice_energy(e_task);
	}
}

/* Decide if we should switch to another CPU local thread.
 *
 * Requires that the local runqueue lock is taken.
 *
 * @rq:		the runqueue of the current CPU.
 *
 * @returns:	whether we should switch or not.
 */
static inline bool should_switch_local(struct rq* rq) {
	if (rq->en.nr_runnable <= 1) {
		/* We can only switch locally if there are more than one thread assigned
		 * to this runqueue. */
		return false;
	} else {
		/* Ok, we have more than one thread assigned to this runqueue. So decide
		 * based on how long the thread was running if we should switch to another
		 * one or not. */
		struct task_struct* curr = rq->en.curr;
		u64 exec_time;

		exec_time = curr->se.sum_exec_runtime - curr->se.prev_sum_exec_runtime;

		return exec_time > sched_slice_local(rq);
	}
}

/* Decide whether we need to reassign the threads of an energy task to the available
 * CPU's.
 *
 * @e_task:	the energy task struct of the energy task which may needs to be
 *		redistributed.
 * @t:		the task struct of the thread of the energy task which changed its
 *		state.
 * @arrived:	whether or not the thread was added or removed from the runqueue.
 *
 * @retruns:	whether or not it is necessary to redistribute the enregy task.
 */
static inline bool should_redistribute_energy(struct energy_task* e_task,
		struct task_struct* t) {
	return e_task->running || thread_cpu_running(t);
}

/* Tell the given runqueue to perform an energy task rescheduling.
 *
 * @rq:		the runqueue which should perform an energy task rescheduling.
 */
static inline void resched_curr_energy(struct rq* rq) {
	if (this_rq() == rq)
		resched_curr(rq);
	else
		set_tsk_need_resched(rq->curr);

	rq->en.resched_flags |= ENERGY_RESCHED;
}

/* Check if we must perform an energy task rescheduling.
 *
 * @rq:		the runqueue of the current CPU.
 *
 * @returns:	whether or not we must reschedule the energy task.
 */
static inline bool need_resched_curr_energy(struct rq* rq) {
	return rq->en.resched_flags & ENERGY_RESCHED;
}

/* Clear the energy task reschedule flag again on the given runqueue.
 *
 * @rq:		the runqueue at which the flag should be cleared.
 */
static inline void clear_resched_curr_energy(struct rq* rq) {
	rq->en.resched_flags &= ~ENERGY_RESCHED;
}

/* Tell the given runqueue to perform a local rescheduling.
 *
 * @rq:		the runqueue which should perform a local rescheduling.
 */
static inline void resched_curr_local(struct rq* rq) {
	if (this_rq() == rq)
		resched_curr(rq);
	else
		set_tsk_need_resched(rq->curr);

	rq->en.resched_flags |= LOCAL_RESCHED;
}

/* Check if we must perform a local rescheduling.
 *
 * @rq:		the runqueue of the current CPU.
 *
 * @returns:	whether or not we must reschedule locally.
 */
static inline bool need_resched_curr_local(struct rq* rq) {
	return rq->en.resched_flags & LOCAL_RESCHED;
}

/* Clear the local reschedule flag again on the given runqueue.
 *
 * @rq:		the runqueue at which the flag should be cleared.
 */
static inline void clear_resched_curr_local(struct rq* rq) {
	rq->en.resched_flags &= ~LOCAL_RESCHED;
}

/* Update the energy statistics of an energy task.
 *
 * @e_task:	the energy task struct of the energy task.
 */
static void update_energy_statistics(struct energy_task* e_task) {
	/* TODO: Implement energy accounting. */
	return;
}

/* Update the runtime statistics of a thread of an energy task.
 *
 * @rq:		the runqueue at which the thread did run.
 * @t:		the task struct of the thread.
 */
static void update_local_statistics(struct rq* rq, struct task_struct* t) {
	u64 now = rq_clock_task(rq);
	s64 delta_exec;

	/* Calculate how long the linux task has run. */
	delta_exec = now - t->se.exec_start;
	t->se.exec_start = now;

	if (unlikely(delta_exec <= 0))
		return;

	/* Update the maximum runtime. */
	if (delta_exec > t->se.statistics.exec_max)
		schedstat_set(t->se.statistics.exec_max, delta_exec);

	/* Increase the total runtime of the linux task. */
	t->se.sum_exec_runtime += delta_exec;

	/* Update the CPU accounting. */
	cpuacct_charge(t, delta_exec);

	/* Update the runtime average of the scheduler. */
	sched_rt_avg_update(rq, delta_exec);
}

/* Update the CPU assigned to the given linux task t.
 *
 * This method also properly handles the number of runnable linux tasks on
 * each of the CPU's runqueues.
 *
 * @t:		the task struct of the linux task for which the assigned CPU
 *		should be changed.
 * @cpu:	the new CPU number.
 */
static inline void move_local_task(struct task_struct* t, unsigned int cpu) {
	if (task_cpu(t) == cpu) {
		/* Do not move from the task if it is already at the wanted
		 * CPU. */
		return;
	}

	sub_nr_running(task_rq(t), 1);
	task_rq(t)->en.nr_assigned--;

	__set_task_cpu(t, cpu);

	task_rq(t)->en.nr_assigned++;
	add_nr_running(task_rq(t), 1);
}

/* Set on the given runqueue that the given energy task is now running there.
 *
 * @rq:		the runqueue where the idle task should run.
 * @e_task:	the energy task which is currently running.
 */
static void set_energy_task(struct rq* rq, struct energy_task* e_task) {
	lock_local_rq(rq);

	rq->en.curr_e_task = e_task;

	unlock_local_rq(rq);
}

/* Set the thread to run on the current runqeueue.
 *
 * Requires that the lock of the local runqueue is taken.
 *
 * @rq:		the runqueue of the current CPU.
 * @t:		the task struct of the thread which should run next.
 */
static inline void set_local_task(struct rq* rq, struct task_struct* t) {
	t->ee.state |= THREAD_CPU_RUNNING;
	rq->en.curr = t;

	t->se.exec_start = rq_clock_task(rq);
	t->se.prev_sum_exec_runtime = t->se.sum_exec_runtime;
}

static void __distribute_energy_task(struct energy_task* e_task) {
	struct task_struct* thread;
	int cpu;

	/* Distribute all runnable threads belonging to the current energy task
	 * on the available CPUs in the energy domain. */
	list_for_each_entry(thread, &(e_task->runnable), ee.rq) {
		if (thread_in_cpu_rq_queued(thread)) {
			/* This thread is already assigned to a CPU runqueue. No
			 * need to do this again. */
			continue;
		} else {
			/* Find the CPU where the thread can run and which has the lowest
			 * load. */
			int min_load = INT_MAX;
			struct rq* best_rq = NULL;

			for_each_cpu_and(cpu, &(e_task->domain), &(thread->cpus_allowed)) {
				int load = cpu_rq(cpu)->en.nr_runnable;

				if (load < min_load) {
					min_load = load;
					best_rq = cpu_rq(cpu);
				}
			}

			if (best_rq == NULL) {
				/* We could not find any CPU suited for the thread. Assign it to
				 * the first one in our energy domain. */
				best_rq = cpu_rq(cpumask_first(&(e_task->domain)));
			}

			distribute_local_task(best_rq, thread);
		}
	}

	/* Set on all runqueues that the current energy task is running. */
	for_each_cpu(cpu, &(e_task->domain)) {
		struct rq* c_rq = cpu_rq(cpu);
		set_energy_task(c_rq, e_task);

		resched_curr_local(c_rq);
	}

}

/* Tell all CPUs belonging to the current energy domain, that a new energy
 * task is going to run and hence which threads are assigned to them.
 *
 * Requires that the global runqueue lock is taken.
 *
 * @rq:		the runqueue of the current CPU.
 * @e_task:	the energy task which is going to be distributed.
 */
static void distribute_energy_task(struct rq* rq, struct energy_task* e_task) {
	/* Mark the energy task running. */
	e_task->running = 1;
	e_task->start_running = rq_clock(rq);

	/* Copy the current energy domain. */
	cpumask_copy(&(e_task->domain), &(rq->en.domain));

	__distribute_energy_task(e_task);
}

/* Assign a linux task t belonging to the energy task e_task to a special
 * runqueue.
 *
 * @rq:		the runqueue to which the linux task should be assigned.
 * @t:		the linux task which should be assigned.
 */
static void distribute_local_task(struct rq* rq, struct task_struct* t) {
	clear_resched_curr_local(rq);

	lock_local_rq(rq);

	/* Update the CPU assigned to the local task. */
	move_local_task(t, cpu_of(rq));

	/* Enqueue in the local runqueue. */
	enqueue_running(rq, t);

	unlock_local_rq(rq);
}

/* Reevaluate the task assignment after a new thread for an energy task arrived.
 *
 * Requires that the global runqueue lock is taken.
 *
 * @rq:		the runqueue of the current CPU.
 * @e_task:	the energy task struct of the energy task which should be redistributed.
 */
static void redistribute_energy_task(struct rq* rq, struct energy_task* e_task, bool arrived) {
	if (arrived) {
		if (!grq.running) {
			/* The scheduling class is not running yet. Switch to it. */
			grq.running = 1;
			grq.start_running = rq_clock(rq);
		}

		if (!e_task->running) {
			/* The energy task is not running yet. Switch to it. */
			distribute_energy_task(rq, e_task);
		} else {
			/* The energy task is already running, so just redistribute it. */
			__distribute_energy_task(e_task);
		}
	} else {
		if (e_task->nr_runnable != 0) {
			/* The energy task still has threads to run, so just redistribute them. */
			__distribute_energy_task(e_task);
		} else {
			/* The energy task has no threads to run anymore. Remove the task. */
			put_energy_task(rq, e_task);

			if (grq.nr_tasks == 0 ) {
				/* This was the last task of the scheduling class, so we have to switch
				 * back to the other scheduling classes. */
				grq.running = 0;
				grq.stop_running = rq_clock(rq);
			}
		}
	}
}

/* Tell all CPUs belonging to the current energy domain, that the energy
 * task is not going to run any more.
 *
 * @rq:		the runqueue of the current CPU.
 * @e_task:	the energy task which is going to stop.
 */
static void clear_energy_task(struct rq* rq, struct energy_task* e_task) {
	int cpu;

	for_each_cpu(cpu, &(e_task->domain)) {
		clear_local_tasks(cpu_rq(cpu));
	}

	clear_resched_curr_energy(rq);
}

/* Clear the locally assigned linux tasks at the given runqueue rq.
 *
 * Requires that the runqueue lock is taken.
 *
 * @rq:		the runqueue at which all local linux tasks should be
 *		removed.
 */
static void clear_local_tasks(struct rq* rq) {
	lock_local_rq(rq);

	/* Clear the lits of threads assigned to this CPU. */
	while (!list_empty(&(rq->en.runnable))) {
		struct task_struct* thread = list_first_entry(&(rq->en.runnable), struct task_struct, ee.cpu_rq);
		dequeue_running(thread);
	}
	rq->en.nr_runnable = 0;

	/* Reset the pointers to the currently running task and energy task. */
	rq->en.curr = NULL;
	rq->en.curr_e_task = NULL;

	clear_resched_curr_local(rq);

	unlock_local_rq(rq);

	/* Force rescheduling on the runqueue. */
	set_tsk_need_resched(rq->curr);
}

/* Remove the energy task e_task as currently running one.
 *
 * @rq:		the runqueue of the current CPU.
 * @e_task:	the energy task which should not run any more.
 */
static void put_energy_task(struct rq* rq, struct energy_task* e_task) {
	/* Update the energy task's statistics. */
	update_energy_statistics(e_task);

	/* Tell all CPU's to stop executing the threads of the current
	 * energy task. */
	clear_energy_task(rq, e_task);

	e_task->running = 0;

	cpumask_clear(&(e_task->domain));

	/* Check if we can remove the energy task again. */
	if (e_task->nr_runnable == 0) {
		free_energy_task(e_task);
	}
}

/* Remove the linux task t as currently running one.
 *
 * @rq:		the runqueue of the current CPU.
 * @t:		the task struct of the linux task which should not run
 *		any more.
 */
static void put_local_task(struct rq* rq, struct task_struct* t) {
	update_local_statistics(rq, t);

	lock_local_rq(rq);

	t->ee.state &= ~THREAD_CPU_RUNNING;

	rq->en.curr = NULL;

	unlock_local_rq(rq);
}

/* Pick a new energy task which should run next from the global runqueue.
 *
 * Requires that the global runqueue lock is taken.
 *
 * @returns:	the energy task which should run next.
 */
static struct energy_task* pick_next_energy_task(void) {
	struct energy_task* head = list_first_entry(&(grq.tasks), struct energy_task, rq);

	clear_resched_curr_energy(this_rq());

	/* Go through the whole list by rotating it and try to find an energy task which is
	 * not running already but has runnable threads. */
	do {
		struct energy_task* next_e_task = list_first_entry(&(grq.tasks),
				struct energy_task, rq);
		list_rotate_left(&(grq.tasks));

		if ((next_e_task->running == 0) && (next_e_task->nr_runnable != 0)) {
			/* We have found our next energy task. */

			return next_e_task;
		}
	} while (head != list_first_entry(&(grq.tasks), struct energy_task, rq));

	/* We could not find any task. */
	return NULL;
}

/* Pick a new linux task which should run from the list of runnable task of the
 * given runqueue.
 *
 * @rq:		the runqueue from which a new task should be picked.
 */
static struct task_struct* pick_next_local_task(struct rq* rq) {
	struct task_struct* next;

	clear_resched_curr_local(rq);

	lock_local_rq(rq);

	if (rq->en.nr_runnable != 0) {
		/* We have available threads on the runqueue, so pick on of them. */
		next = list_first_entry(&(rq->en.runnable), struct task_struct, ee.cpu_rq);
		list_rotate_left(&(rq->en.runnable));
	} else {
		/* We have no threads to run, so run the idle task. */
		next = rq->en.idle;
	}

	/* Set that we are now executing the selected thread. */
	set_local_task(rq, next);

	unlock_local_rq(rq);

	return next;
}

/* The idle thread function. */
static int idle_thread_fn(void* unused) {
	while (!kthread_should_stop()) {
		while (!tif_need_resched())
			cpu_relax();

		schedule();
	}

	return 0;
}

/* Set and initialize the energy domain of a given CPU.
 *
 * @domain:	the pointer to the cpumask where the energy domain should be
 *		set.
 * @cpu:	the CPU for which we need to initialize the energy domain.
 */
static void init_energy_domain(struct cpumask* domain, unsigned int cpu) {
	/* TODO: Really find the energy domain of the current CPU. */
	cpumask_setall(domain);
}


/***
 * External function definitions.
 ***/

/* Add a linux task t to the runqueue.
 *
 * @rq:		the runqueue of the current CPU.
 * @t:		the task struct of the linux task which should be added.
 * @flags:
 */
void enqueue_task_energy(struct rq* rq, struct task_struct* t, int flags) {
	struct energy_task* e_task;

	lock_grq();

	e_task = find_energy_task(t);

	if (e_task == NULL) {
		/* Ok, the energy task did not exist yet, so we need to create it first,
		 * before we can continue. */
		if (!(e_task = create_energy_task(t))) {
			BUG();
		}
	}

	/* Add the thread to the list of runnable threads. */
	enqueue_runnable(rq, e_task, t);

	if (should_redistribute_energy(e_task, t)) {
		redistribute_energy_task(rq, e_task, true);
	}

	unlock_grq();
}

/* Remove a linux task t from the runqueue.
 *
 * @rq:		the runqueue of the current CPU.
 * @t:		the task struct of the linux task which should be removed.
 * @flags:
 */
void dequeue_task_energy(struct rq* rq, struct task_struct* t, int flags) {
	struct energy_task* e_task;

	lock_grq();

	e_task = find_energy_task(t);

	if (e_task == NULL) {
		/* This should not happen. */
		BUG();
	}

	/* Remove the thread from the CPU runqueue. */
	if (thread_in_cpu_rq_queued(t)) {
		lock_local_rq(task_rq(t));

		dequeue_running(t);

		unlock_local_rq(task_rq(t));
	}

	/* Remove the thread from the list of runnable threads. */
	dequeue_runnable(e_task, t);

	if (should_redistribute_energy(e_task, t)) {
		redistribute_energy_task(rq, e_task, false);
	}

	unlock_grq();
}

/* The currently running linux task wants to give up the CPU.
 *
 * @rq:		the runqueue of the current CPU.
 */
void yield_task_energy(struct rq* rq) {
	if (rq->en.nr_runnable > 2) {
		/* Yield in this scheduling class will only work if multiple
		 * threads of the same task are assigned to the same CPU. If
		 * this is the case, a local rescheduling is performed. */
		resched_curr_local(rq);
	}
}

/* The current running linux task wants to give up the CPU to another linux task t.
 *
 * @rq:		the runqueue of the current CPU.
 * @t:		the task struct of the linux task which should run next.
 * @preemt:	whether or not preempting of another linux task is allowed.
 *
 * @returns:	whether we yielded to the other task or not.
 */
bool yield_to_task_energy(struct rq* rq, struct task_struct* t, bool preemt) {
	/* TODO: Implement yield to. */
	return false;
}

/* Preempt the current linux task in favor of the linux task t.
 *
 * @rq:		the runqueue of the current CPU.
 * @t:		the task struct of the linux task which should be run in favor
 *		of the current one.
 * @flags:
 */
void check_preempt_curr_energy(struct rq* rq, struct task_struct* t, int flags) {
	/* We are doing nothing here. The currently running task is never preempted
	 * in favor of another one. */
	return;
}

/* Select a new linux task which should run instead of prev.
 *
 * @rq:		the runqueue of the current CPU.
 * @prev:	the task struct of the liunx task which should be replaced.
 *
 * @returns:	the task struct of the linux task which should run next.
 */
struct task_struct* pick_next_task_energy(struct rq* rq, struct task_struct* prev) {
	lock_grq();

	if (!grq.running) {
		if (should_switch_to_energy(rq)) {
			struct energy_task* next_e_task = pick_next_energy_task();

			grq.running = 1;
			grq.start_running = rq_clock(rq);

			distribute_energy_task(rq, next_e_task);
		}
	} else {
		struct energy_task* curr_e_task = rq->en.curr_e_task;

		if (curr_e_task == NULL) {
			if (should_switch_from_energy(rq)) {
				grq.running = 0;
				grq.stop_running = rq_clock(rq);
			} else {
				struct energy_task* next_e_task = pick_next_energy_task();

				distribute_energy_task(rq, next_e_task);
			}
		} else {
			if (should_switch_from_energy(rq)) {
				grq.running = 0;
				grq.stop_running = rq_clock(rq);

				put_energy_task(rq, curr_e_task);
			} else if (should_switch_energy(rq)) {
				struct energy_task* next_e_task = pick_next_energy_task();

				put_energy_task(rq, curr_e_task);

				distribute_energy_task(rq, next_e_task);
			}
		}
	}

	unlock_grq();

	if (need_resched_curr_local(rq)) {
		/* Tell the scheduling class of prev that it is going to be removed. */
		put_prev_task(rq, prev);

		/* Select a new thread which should run on this CPU. */
		pick_next_local_task(rq);
	}

	return rq->en.curr;
}

/* Put a currently running linux task t back into the runqueue.
 *
 * @rq:		the runqueue of the current CPU.
 * @t:		the task struct of the linux task which should give up the CPU.
 */
void put_prev_task_energy(struct rq* rq, struct task_struct* t) {
	put_local_task(rq, t);
}

/*
 * @rq:		the runqueue of the current CPU.
 */
void set_curr_task_energy(struct rq* rq) {
	struct task_struct* curr = rq->curr;

	lock_local_rq(rq);

	if (!thread_in_rq_queued(curr)) {
		enqueue_running(rq, curr);
	}
	set_local_task(rq, curr);

	unlock_local_rq(rq);
}

/* A scheduling tick happened with the linux task t running.
 *
 * @rq:		the runqueue of the current CPU.
 * @t:		the task struct of the linux task for which the scheduling tick
 *		happened.
 * @queued:	is the task still in a runqueue.
 */
void task_tick_energy(struct rq* rq, struct task_struct* t, int queued) {
	update_local_statistics(rq, t);

	lock_grq();

	if (should_switch_energy(rq) || should_switch_from_energy(rq)) {
		resched_curr(rq);
	}

	unlock_grq();

	lock_local_rq(rq);

	if (should_switch_local(rq)) {
		resched_curr_local(rq);
	}

	unlock_local_rq(rq);
}

/* The linux task t was just created by a fork.
 *
 * @t:		the task struct of the linux task which just was created.
 */
void task_fork_energy(struct task_struct* t) {
	/* We have nothing to do here currently. */
	return;
}

/* The linux task t died.
 *
 * @t:		the task struct of the linux task which just died.
 */
void task_dead_energy(struct task_struct* t) {
	/* We have nothing to do here currently. */
	return;
}

/* The scheduling class of the linux task t changed to another one.
 *
 * @rq:		the runqueue of the current CPU.
 * @t:		the linux task which was removed from our scheduling class.
 */
void switched_from_energy(struct rq* rq, struct task_struct* t) {
	struct energy_task* e_task;
	struct task_struct* thread;

	lock_grq();

	e_task = find_energy_task(t);

	if (e_task) {
		for_each_thread(e_task->task, thread) {
			lower_thread(thread, e_task, t->policy, t->sched_class);
		}

		if (e_task->running) {
			redistribute_energy_task(rq, e_task, false);
		}
	}

	unlock_grq();
}

/* The scheduling class of the linux task t changed to this one.
 *
 * @rq:		the runqueue of the current CPU.
 * @t:		the linux task which was added to our scheduling class.
 */
void switched_to_energy(struct rq* rq, struct task_struct* t) {
	struct energy_task* e_task;
	struct task_struct* thread;

	lock_grq();

	e_task = find_energy_task(t);

	if (e_task) {
		for_each_thread(e_task->task, thread) {
			raise_thread(thread, e_task);
		}

		if (e_task->running) {
			redistribute_energy_task(rq, e_task, true);
		}
	}

	unlock_grq();
}

/* The priority of the linux task t changed.
 *
 * @rq:		the runqueue of the current CPU.
 * @t:		the task struct of the linux task for which the priority was changed.
 * @old_prio:	the previous priority of the task.
 */
void prio_changed_energy(struct rq* rq, struct task_struct* t, int old_prio) {
	/* We have nothing to do here currently. */
	return;
}

/* Get the round robin interval for the linux task t.
 *
 * This information is important for the POSIX RR-Scheduler.
 *
 * @rq:		the runqueue of the current CPU.
 * @t:		the task struct of the linux task for which the RR interval should
 *		be returned.
 *
 * @returns:	the RR interval for the given linux task.
 */
unsigned int get_rr_interval_energy(struct rq* rq, struct task_struct* t) {
	return sched_slice_local(rq);
}

/* Update the runtime statistics of the currently running linux task outside
 * of a schedule tick.
 *
 * @rq:		the runqueue of the current CPU.
 */
void update_curr_energy(struct rq* rq) {
	update_local_statistics(rq, rq->curr);
}

/* Select the CPU where the linux task t should run.
 *
 * @t:		the task struct of the linux task for which a new runqueue should be
 *		selected.
 * @cpu:	the CPU where it is currently running.
 * @sd_flags:	scheduler flags such as SD_BALANCE_FORK.
 * @flags:	other flags like wake_up_flags.
 *
 * @returns:	the CPU number where the linux task t should run.
 */
int select_task_rq_energy(struct task_struct* t, int cpu, int sd_flags, int flags) {
	return cpu;
}

/* The linux task t is going to be migrated to a new CPU.
 *
 * @t:		the task struct of the linux task which is going to be migrated.
 * @new_cpu:	the new CPU where the linux task will be migrated to.
 */
void migrate_task_rq_energy(struct task_struct* t, int new_cpu) {
	/* We have nothing to do here currently. */
	return;
}

/* The linux task t is going to be woken up.
 *
 * @t:		the task struct of the linux task which is going to be woken up.
 */
void task_waking_energy(struct task_struct* t) {
	/* We have nothing to do here currently. */
	return;
}

/* The linux task t was woken up.
 *
 * @rq:		the runqueue where the linux task was woken up.
 * @t:		the task struct of the linux task which was woken up.
 */
void task_woken_energy(struct rq* rq, struct task_struct* t) {
	/* We have nothing to do here currently. */
	return;
}

/* The CPUs where the linux task t is allowed to run changed.
 *
 * @t:		the task struct of the linux task which changed its CPUs.
 * @newmask:	the new CPUs where the linux task is allowed to run.
 */
void set_cpus_allowed_energy(struct task_struct* t, const struct cpumask* newmask) {
	/* TODO: Check if we need to reschedule the corresponding energy task. */
	return;
}

/* A CPU was plugged in and became online.
 *
 * @rq:		the runqueue of the CPU which just came online.
 */
void rq_online_energy(struct rq* rq) {
	/* TODO: Check if we need can reschedule the current energy task. */
	return;
}

/* A CPU was plugged out and became offline.
 *
 * @rq:		the runqueue of the CPU which just came offline.
 */
void rq_offline_energy(struct rq* rq) {
	/* TODO: Handle this properly. */
	return;
}


/***
 * The Energy Scheduling Class.
 ***/

const struct sched_class energy_sched_class = {
	.next = &fair_sched_class,

	.enqueue_task = enqueue_task_energy,
	.dequeue_task = dequeue_task_energy,

	.yield_task = yield_task_energy,
	.yield_to_task = yield_to_task_energy,

	.check_preempt_curr = check_preempt_curr_energy,

	.pick_next_task = pick_next_task_energy,

	.put_prev_task = put_prev_task_energy,

	.set_curr_task = set_curr_task_energy,

	.task_tick = task_tick_energy,
	.task_fork = task_fork_energy,
	.task_dead = task_dead_energy,

	.switched_from = switched_from_energy,
	.switched_to = switched_to_energy,
	.prio_changed = prio_changed_energy,

	.get_rr_interval = get_rr_interval_energy,

	.update_curr = update_curr_energy,

#ifdef CONFIG_SMP
	.select_task_rq = select_task_rq_energy,
	.migrate_task_rq = migrate_task_rq_energy,

	.task_waking = task_waking_energy,
	.task_woken = task_woken_energy,

	.set_cpus_allowed = set_cpus_allowed_energy,

	.rq_online = rq_online_energy,
	.rq_offline = rq_offline_energy,
#endif
};

/***
 * Other external functions.
 ***/

/* Initialize the per core runqueues.
 *
 * @e_rq:	the energy runqueue which must be initialized.
 * @cpu:	the CPU it which this runqueue is established.
 */
void __init init_e_rq(struct e_rq* e_rq, unsigned int cpu) {
	raw_spin_lock_init(&(e_rq->lock));

	e_rq->resched_flags = 0;

	init_energy_domain(&(e_rq->domain), cpu);

	INIT_LIST_HEAD(&(e_rq->runnable));
	e_rq->nr_runnable = 0;

	e_rq->nr_assigned = 0;

	e_rq->curr = NULL;
	e_rq->curr_e_task = NULL;

	e_rq->idle = NULL;
}

int __init init_e_idle_threads(void) {
	int cpu;

	for_each_possible_cpu(cpu) {
		struct task_struct* idle_thread;
		struct rq* c_rq = cpu_rq(cpu);

		/* Create the kernel thread and move it into our scheduling
		 * class. */
		idle_thread = kthread_create(idle_thread_fn, NULL, "e_idle/%u", cpu);
		kthread_bind(idle_thread, cpu);

		idle_thread->sched_class = &energy_sched_class;
		idle_thread->policy = SCHED_ENERGY;
		idle_thread->state = TASK_RUNNING;

		c_rq->en.idle = idle_thread;
	}

	return 0;
}

late_initcall(init_e_idle_threads);

/* Initialize the energy scheduling class. */
void __init init_sched_energy_class(void) {
	init_grq();
}
