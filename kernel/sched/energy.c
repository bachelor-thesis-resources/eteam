/* vim: set noet ts=8 sw=8 sts=8 : */

#include <linux/cpumask.h>
#include <linux/list.h>
#include <linux/smp.h>
#include <linux/spinlock.h>

#include <linux/sched.h>

#include "sched.h"
#include "cpuacct.h"


/***
 * Internal data structure prototypes.
 ***/

struct energy_task;
struct global_rq;
struct redistribute_data;


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

	/* All runnable threads. */
	struct list_head runnable;
	u32 nr_runnable;

	/* All sleeping threads. */
	struct list_head sleeping;
	u32 nr_sleeping;

	/* The link in the global runqueue. */
	struct list_head rq;

	/* Runtime statistics */
	u64 total_exec_runtime;
	u64 prev_total_exec_runtime;
};

/* The global runqueue for all task with their corresponding threads which
 * are managed by this scheduling class. */
struct global_rq {
	/* Lock for the global runqueue. */
	raw_spinlock_t lock;

	/* The number of tasks currently in the list. */
	u32 nr_tasks;

	/* The number of threads currently in the list. */
	u32 nr_threads;

	/* The list of tasks for which energy should be measured. */
	struct list_head tasks;

	/* Runtime statistics */
	u64 start_running;
	u64 stop_running;
};


/* All the data needed to make redistribute a currently running energy
 * task. */
struct redistribute_data {
	/* The energy task which should be redistributed. */
	struct energy_task* e_task;

	/* The thread which changed its state. */
	struct task_struct* thread;

	/* Whether the thread just arrived, or vanished. */
	bool arrived;
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
static struct energy_task* find_or_create_energy_task(struct task_struct*);

/* Threads and energy tasks. */
static void enqueue_sleeping(struct rq*, struct energy_task*, struct task_struct*);
static void dequeue_sleeping(struct rq*, struct energy_task*, struct task_struct*);

static void enqueue_runnable(struct rq*, struct energy_task*, struct task_struct*);
static void dequeue_runnable(struct rq*, struct energy_task*, struct task_struct*);

static void enqueue_thread(struct rq*, struct energy_task*, struct task_struct*);
static void dequeue_thread(struct rq*, struct energy_task*, struct task_struct*);

/* Threads and the scheduling class. */
static void raise_thread(struct task_struct*);
static void lower_thread(struct task_struct*, const struct sched_class* to);

/* Determining the scheduling slices. */
static u64 sched_slice_class(void);
static u64 sched_slice_energy(void);
static u64 sched_slice_local(struct rq*);
static u64 sched_slice_other(void);

/* Should we perform a scheduling operation? */
static bool should_switch_to_energy(struct rq*);
static bool should_switch_from_energy(struct rq*);
static bool should_switch_energy(struct rq*);
static bool should_switch_local(struct rq*);

/* Should we redistribute an energy task again? */
static bool should_redistribute_energy(struct energy_task*, struct task_struct*);

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
static void distribute_local_task(struct rq*, struct energy_task*, struct task_struct*);
static void distribute_idle_task(struct rq*, struct energy_task*);
static void distribute_energy_task(struct rq*, struct energy_task*);

void redistribute_energy_task(void*);

static void set_local_task_cpu(struct task_struct*, unsigned int);

static void clear_energy_task(struct rq*, struct energy_task*);
static void clear_local_task(struct rq*);

static void put_energy_task(struct rq*, struct energy_task*);
static void put_local_task(struct rq*, struct task_struct*);

static struct energy_task* pick_next_energy_task(void);
static struct task_struct* pick_next_local_task(struct rq*);

/* Leader management. */
static bool is_leader(struct rq*);
static unsigned int leader_cpu(void);
static struct rq* leader_rq(void);


/***
 * Internal function definitions.
 ***/

/* Initialize the global runqueue. */
static void init_grq(void) {
	INIT_LIST_HEAD(&(grq.tasks));
	raw_spin_lock_init(&(grq.lock));
	grq.nr_tasks = 0;
	grq.nr_threads = 0;

	grq.stop_running = grq.start_running = 0;
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

	INIT_LIST_HEAD(&(e_task->runnable));
	e_task->nr_runnable = 0;

	INIT_LIST_HEAD(&(e_task->sleeping));
	e_task->nr_sleeping = 0;

	e_task->total_exec_runtime = e_task->prev_total_exec_runtime = 0;
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

/* Find the energy task struct corresponding to a task.
 *
 * Requires that the lock of the global rq is taken.
 *
 * @t:		the task struct of the real task for which the corresponding
 *		energy task struct should be returned.
 *
 * @returns:	the energy task struct corresponding to the given task.
 */
static struct energy_task* __find_energy_task(struct task_struct* task) {
	struct energy_task* e_task;

	list_for_each_entry(e_task, &(grq.tasks), rq) {
		if (e_task->task == task) {
			return e_task;
		}
	}

	return NULL;
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

	return __find_energy_task(task);
}

/* Finds or creates the energy task struct corresponding to a linux task t.
 *
 * Requires that the lock of the global rq is taken.
 *
 * @t:		the task struct of the linux task for which the corresponding
 *		energy task struct should be returned.
 *
 * @returns:	the energy task struct corresponding to the given linux task.
 */
static struct energy_task* find_or_create_energy_task(struct task_struct* t) {
	struct task_struct* task = find_task(t);
	struct energy_task* e_task = __find_energy_task(task);

	if (e_task == NULL) {
		/* There is no energy task yet. So create and initialize one. */
		e_task = (struct energy_task*)kmalloc(sizeof(struct energy_task), GFP_KERNEL);
		init_energy_task(e_task);

		/* Remember the task struct for the actual task. */
		e_task->task = task;

		/* Enqueue the created task in the global runqueue. */
		enqueue_energy_task(e_task);
	}

	return e_task;
}

/* Enqueue a thread into the list of sleeping threads of the energy task.
 *
 * @rq:		the runqueue of the current CPU.
 * @e_task:	the energy task struct of the energy task to which the thread
 *		belongs.
 * @t:		the task struct of the thread which should be enqueued.
 */
static void enqueue_sleeping(struct rq* rq, struct energy_task* e_task, struct task_struct* t) {
	if (t->ee.on_rq == 1) {
		/* This thread is already enqueued in the runqueue. */
		return;
	}

	/* Add to the list. */
	list_add(&(t->ee.threads), &(e_task->sleeping));
	e_task->nr_sleeping++;

	/* Mark the thread as added. */
	t->ee.on_rq = 1;
}

/* Enqueue a thread into the list of running threads of the energy task.
 *
 * @rq:		the runqueue of the current CPU.
 * @e_task:	the energy task struct of the energy task to which the thread
 *		belongs.
 * @t:		the task struct of the thread which should be enqueued.
 */
static void enqueue_runnable(struct rq* rq, struct energy_task* e_task, struct task_struct* t) {
	if (t->ee.on_rq == 1) {
		/* This thread is already enqueued in the runqueue. */
		return;
	}

	/* Add to the list. */
	list_add(&(t->ee.threads), &(e_task->runnable));
	e_task->nr_runnable++;

	/* Mark the thread as added. */
	t->ee.on_rq = 1;

	/* Remember in the global runqueue that we have a runnable thread. */
	grq.nr_threads++;

	/* Remember in the runqueue that there is now a new runnable linux task. */
	add_nr_running(rq, 1);
}

/* Enqueue a thread into the list of threads of the energy task.
 *
 * @rq:		the runqueue of the current CPU.
 * @e_task:	the energy task struct of the energy task to which the thread
 *		belongs.
 * @t:		the task struct of the thread which should be enqueued.
 */
static void enqueue_thread(struct rq* rq, struct energy_task* e_task, struct task_struct* t) {
	/* Use the correct function to enqueue the linux task depending on its state. */
	if (t->state & TASK_RUNNING) {
		enqueue_runnable(rq, e_task, t);
	} else if (t->state & TASK_UNINTERRUPTIBLE || t->state & TASK_INTERRUPTIBLE) {
		enqueue_sleeping(rq, e_task, t);
	}
}

/* Dequeue a thread from the list of sleeping threads of the energy task.
 *
 * @rq:		the runqueue of the current CPU.
 * @e_task:	the energy task struct of the energy task to which the thread
 *		belongs.
 * @t:		the task struct of the thread which should be dequeued.
 */
static void dequeue_sleeping(struct rq* rq, struct energy_task* e_task, struct task_struct* t) {
	if (t->ee.on_rq == 0) {
		/* This thread is not enqueued in the runqueue. */
		return;
	}

	/* Remove from the list. */
	list_del(&(t->ee.threads));
	e_task->nr_sleeping--;

	t->ee.on_rq = 0;
}

/* Dequeue a thread from the list of running threads of the energy task.
 *
 * @rq:		the runqueue of the current CPU.
 * @e_task:	the energy task struct of the energy task to which the thread
 *		belongs.
 * @t:		the task struct of the thread which should be dequeued.
 */
static void dequeue_runnable(struct rq* rq, struct energy_task* e_task, struct task_struct* t) {
	if (t->ee.on_rq == 0) {
		/* This thread is not enqueued in the runqueue. */
		return;
	}

	/* Remove from the list. */
	list_del(&(t->ee.threads));
	e_task->nr_runnable--;

	t->ee.on_rq = 0;

	/* Remember in the global runqueue that the thread is no longer runnable. */
	grq.nr_threads--;

	/* Remember in the runqueue that the thread is no longer runnable. */
	sub_nr_running(rq, 1);
}

/* Dequeue a thread from the list of threads of the energy task.
 *
 * @rq:		the runqueue of the current CPU.
 * @e_task:	the energy task struct of the energy task to which the thread
 *		belongs.
 * @t:		the task struct of the thread which should be dequeued.
 */
static void dequeue_thread(struct rq* rq, struct energy_task* e_task, struct task_struct* t) {
	/* Use the correct function to dequeue the linux task depending on its state. */
	if (t->state & TASK_RUNNING) {
		dequeue_runnable(rq, e_task, t);
	} else if (t->state & TASK_INTERRUPTIBLE || t->state & TASK_UNINTERRUPTIBLE) {
		dequeue_sleeping(rq, e_task, t);
	}
}

/* Raise the thread t from a different scheduling class to this one.
 *
 * @t:		the task struct of the thread which should be raised.
 */
static void raise_thread(struct task_struct* t) {
	struct rq* rq;
	int queued, running;

	/* First check if we really need to raise the thread. */
	if (t->sched_class == &energy_sched_class) {
		return;
	}

	/* Find and lock the runqueue of the thread. */
	rq = task_rq(t);

	if (rq != this_rq()) {
		/* We need to lock also the threads runqueue, as we are
		 * changing stuff on it. However this is not necessary if
		 * the thread runs on the current runqueue. */

		/* We use double lock balance for this, as it already handles
		 * all the nasty corner cases. */
		double_lock_balance(this_rq(), rq);
	}

	queued = task_on_rq_queued(t);
	running = task_running(rq, t);

	/* Dequeue the task if it is currently queued.*/
	if (queued) {
		t->sched_class->dequeue_task(rq, t, 0);
	}

	/* Stop the task if it is currently running. */
	if (running) {
		put_prev_task(rq, t);
	}

	/* Now raise the thread. */
	t->sched_class = &energy_sched_class;

	/* If we raise a thread, we also need to move it to the leaders runqueue.
	 * This is important, as only the leader can make energy scheduling decisions.
	 * Consequently the other CPU's should not be disturbed by this. */
	set_task_cpu(t, leader_cpu());

	if (rq != this_rq()) {
		/* We need to unlock the threads runqueue again. This is not
		 * necessary if the thread runs on the current runqueue. */
		double_unlock_balance(this_rq(), rq);
	}
}

/* Lower the thread t from the energy accounting scheduling class back
 * to the scheduling class to.
 *
 * @t:		the task struct of the thread which should be lowered.
 * @to:		the scheduling class to which the thread should be lowered.
 */
static void lower_thread(struct task_struct* t, const struct sched_class* to) {
	struct rq* rq;
	int queued;

	/* First check if we really need to lower this thread. */
	if (t->sched_class != &energy_sched_class) {
		return;
	}

	/* Find and lock the runqueue of the thread. */
	rq = task_rq(t);

	if (rq != this_rq()) {
		/* We need to lock also the threads runqueue, as we are
		 * changing stuff on it. However this is not necessary if
		 * the thread runs on the current runqueue. */

		/* We use double lock balance for this, as it already handles
		 * all the nasty corner cases. */
		double_lock_balance(this_rq(), rq);
	}

	queued = task_on_rq_queued(t);

	/* Lower the thread. */
	t->sched_class = to;

	/* Add it back into the runqueue if it is currently queued. */
	if (queued) {
		t->sched_class->enqueue_task(rq, t, 0);
		t->sched_class->check_preempt_curr(rq, t, 0);
	}

	if (rq != this_rq()) {
		/* We need to unlock the threads runqueue again. This is not
		 * necessary if the thread runs on the current runqueue. */
		double_unlock_balance(this_rq(), rq);
	}
}

static inline u64 sched_slice_class(void) {
	return grq.nr_threads * sysctl_sched_min_granularity;
}

static inline u64 sched_slice_energy(void) {
	/* The energy scheduling slice is simply the class scheduling slice
	 * distributed equally between the energy tasks. */
	return sched_slice_class() / grq.nr_tasks;
}

static inline u64 sched_slice_local(struct rq* rq) {
	/* The local scheduling slice is simply the energy scheduling slice
	 * distributed equally between the threads assigned to one CPU. */
	return sched_slice_energy() / rq->en.nr_threads;
}

static inline u64 sched_slice_other(void) {
	return nr_running() - grq.nr_threads * sysctl_sched_min_granularity;
}

/* Decide if we should switch to the energy sched class from another one.
 *
 * @rq:		the runqueue of the current CPU.
 *
 * @returns:	whether we should switch or not.
 */
static inline bool should_switch_to_energy(struct rq* rq) {
	u64 now = rq_clock(rq);
	u64 not_running = now - grq.stop_running;

	return (grq.nr_tasks > 0) && (not_running > sched_slice_other());
}

/* Decide if we should switch away from the energy sched class to another
 * one.
 *
 * @rq:		the runqueue of the current CPU.
 *
 * @returns:	whether we should switch or not.
 */
static inline bool should_switch_from_energy(struct rq* rq) {
	u64 now = rq_clock(rq);
	u64 running = now - grq.start_running;

	return (grq.nr_tasks == 0) || (running > sched_slice_class());
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
	struct energy_task* e_task;
	u64 exec_time;

	/* We can only switch between energy tasks if there are more than
	 * one energy task. */
	if (grq.nr_tasks <= 1) {
		return false;
	}

	/* Ok, we have more than one energy task. So decide based on the total
	 * runtime of the task's threads. */
	e_task = find_energy_task(rq->en.curr_task);
	exec_time = e_task->total_exec_runtime - e_task->prev_total_exec_runtime;

	return exec_time > sched_slice_energy();
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
	struct task_struct* curr;
	u64 exec_time;

	/* We can only switch locally if there are more than one thread assigned
	 * to this runqueue. */
	if (rq->en.nr_threads <= 1) {
		return false;
	}

	/* Ok, we have more than one thread assigned to this runqueue. So decide
	 * based on how long the thread was running if we should switch to another
	 * one or not. */
	curr = rq->en.curr;
	exec_time = curr->se.sum_exec_runtime - curr->se.prev_sum_exec_runtime;

	return exec_time > sched_slice_local(rq);
}

/* Decide whether we need to reassign the threads of an energy task to the available
 * CPU's.
 *
 * @e_task:	the energy task struct of the energy task which may needs to be
 *		redistributed.
 * @t:		the task struct of the thread of the energy task which changed its
 *		state.
 *
 * @retruns:	whether or not it is necessary to redistribute the enregy task.
 */
static inline bool should_redistribute_energy(struct energy_task* e_task, struct task_struct* t) {
	return e_task->running == 1;
}

/* Tell the given runqueue to perform an energy task rescheduling.
 *
 * @rq:		the runqueue which should perform an energy task rescheduling.
 */
static inline void resched_curr_energy(struct rq* rq) {
	resched_curr(rq);
	rq->en.energy_resched = 1;
}

/* Check if we must perform an energy task rescheduling.
 *
 * @rq:		the runqueue of the current CPU.
 *
 * @returns:	whether or not we must reschedule the energy task.
 */
static inline bool need_resched_curr_energy(struct rq* rq) {
	return rq->en.energy_resched == 1;
}

/* Clear the energy task reschedule flag again on the given runqueue.
 *
 * @rq:		the runqueue at which the flag should be cleared.
 */
static inline void clear_resched_curr_energy(struct rq* rq) {
	rq->en.energy_resched = 0;
}

/* Tell the given runqueue to perform a local rescheduling.
 *
 * @rq:		the runqueue which should perform a local rescheduling.
 */
static inline void resched_curr_local(struct rq* rq) {
	resched_curr(rq);
	rq->en.local_resched = 1;
}

/* Check if we must perform a local rescheduling.
 *
 * @rq:		the runqueue of the current CPU.
 *
 * @returns:	whether or not we must reschedule locally.
 */
static inline bool need_resched_curr_local(struct rq* rq) {
	return rq->en.local_resched == 1;
}

/* Clear the local reschedule flag again on the given runqueue.
 *
 * @rq:		the runqueue at which the flag should be cleared.
 */
static inline void clear_resched_curr_local(struct rq* rq) {
	rq->en.local_resched = 0;
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
static void set_local_task_cpu(struct task_struct* t, unsigned int cpu) {
	sub_nr_running(task_rq(t), 1);

	set_task_cpu(t, cpu);

	add_nr_running(task_rq(t), 1);
}

/* Assign a linux task t belonging to the energy task e_task to a special
 * runqueue.
 *
 * @rq:		the runqueue to which the linux task should be assigned.
 * @e_task:	the energy task to which the linux task belongs.
 * @t:		the linux task which should be assigned.
 */
static void distribute_local_task(struct rq* rq, struct energy_task* e_task, 
		struct task_struct* t) {
	struct e_rq* e_rq;

	lock_local_rq(rq);

	/* Update the CPU assigned to the local task. */
	set_local_task_cpu(t, cpu_of(rq));

	e_rq = &(rq->en);

	/* Add the thread to the internal list. */
	list_add(&(t->ee.cpu_rq), &(e_rq->threads));
	e_rq->nr_threads++;

	/* Remember which energy task is running here. */
	e_rq->curr_task = e_task->task;

	unlock_local_rq(rq);

	resched_curr_local(rq);
}

/* Assign the idle task of the energy sched class to the given runqueue.
 *
 * @rq:		the runqueue where the idle task should run.
 * @e_task:	the energy task which is currently running.
 */
static void distribute_idle_task(struct rq* rq, struct energy_task* e_task) {
	lock_local_rq(rq);

	rq->en.curr_task = e_task->task;

	unlock_local_rq(rq);

	resched_curr_local(rq);
}

/* Tell all CPUs belonging to the current energy domain, that a new energy
 * task is going to run and hence which threads are assigned to them.
 *
 * @rq:		the runqueue of the current CPU.
 * @e_task:	the energy task which is going to be distributed.
 */
static void distribute_energy_task(struct rq* rq, struct energy_task* e_task) {
	struct task_struct* thread;
	int cpu;

	/* Distribute all runnable threads belonging to the current energy task
	 * on the available CPUs in the energy domain. */
	list_for_each_entry(thread, &(e_task->runnable), ee.threads) {
		u32 min_load = U32_MAX;
		struct rq* best_rq = NULL;

		/* Find for each thread the best suited CPU. */
		for_each_cpu_and(cpu, &(rq->en.domain), &(thread->cpus_allowed)) {
			int load = cpu_rq(cpu)->en.nr_threads;

			if (load < min_load) {
				min_load = load;
				best_rq = cpu_rq(cpu);
			}
		}
	
		if (best_rq == NULL) {
			/* We could not find any CPU suited for the thread. Assign it
			 * to the first one in the energy domain. */
			best_rq = cpu_rq(cpumask_first(&(rq->en.domain)));
		}

		distribute_local_task(best_rq, e_task, thread);
	}

	/* Set the idle task at all the CPUs which have no local task assigned. */
	for_each_cpu(cpu, &(rq->en.domain)) {
		struct rq* c_rq = cpu_rq(cpu);

		if (c_rq->en.nr_threads == 0) {
			distribute_idle_task(c_rq, e_task);
		}
	}
}

/* Reassign all threads belonging to an energy task to CPUs after a new thread arrived
 * or an existing thread vanished.
 *
 * @data:
 */
void redistribute_energy_task(void* data) {
	struct energy_task* e_task = ((struct redistribute_data*)data)->e_task;
	struct task_struct* thread = ((struct redistribute_data*)data)->thread;
	bool arrived = ((struct redistribute_data*)data)->arrived;
	struct rq* rq = this_rq();

	if (!arrived) {
		/* The thread vanished. So check if we need to update our energy task.
		 * This may be necessary if the last runnable task vanished and we have
		 * to give up the CPU. */
		if (e_task->nr_runnable == 0) {
			resched_curr_energy(rq);
		} else {
			struct rq* t_rq = task_rq(thread);

			if (t_rq->en.nr_threads == 0) {
				distribute_idle_task(t_rq, e_task);
			}
		}
	} else {
		/* Ok a new thread arrived. So pick a CPU where the new thread should
		 * run. */
		int cpu;
		u32 min_load = U32_MAX;
		struct rq* best_rq = NULL;

		for_each_cpu_and(cpu, &(rq->en.domain), &(thread->cpus_allowed)) {
			int load = cpu_rq(cpu)->en.nr_threads;

			if (load < min_load) {
				min_load = load;
				best_rq = cpu_rq(cpu);
			}
		}
	
		if (best_rq == NULL) {
			/* We could not find any CPU suited for the thread. Assign it
			 * to the first one in the energy domain. */
			best_rq = cpu_rq(cpumask_first(&(rq->en.domain)));
		}

		distribute_local_task(best_rq, e_task, thread);
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

	for_each_cpu(cpu, &(rq->en.domain)) {
		clear_local_task(cpu_rq(cpu));
	}
}

/* Remove the currently running idle task from the CPU and perform some clean up with
 * it.
 *
 * Requires that the lock of the current e_rq is taken.
 *
 * @rq:		the runqueue of the current CPU.
 */
static inline void __put_idle_task(struct rq* rq) {
	struct task_struct* idle_task = rq->en.curr;

	/* Put the idle task back into its own scheduling class. */
	idle_task->sched_class = &idle_sched_class;

	/* Clear the flag that we are currently running the idle task. */
	rq->en.runs_idle = 0;
}

/* Tell an energy task thread t that it is going to be removed.
 *
 * Requires that the lock of the current e_rq is taken.
 *
 * @rq:		the runqueue of the thread which is going to be removed.
 * @t:		the task struct of the thread which is going to be removed.
 */
static inline void __put_local_task(struct rq* rq, struct task_struct* t) {
	update_local_statistics(rq, t);
	t->ee.running = 0;
}

/* Clear the locally assigned linux tasks at the given runqueue rq.
 *
 * @rq:		the runqueue at which all local linux tasks should be 
 *		removed.
 */
static void clear_local_task(struct rq* rq) {
	struct e_rq* e_rq = &(rq->en);

	lock_local_rq(rq);

	if (rq->en.runs_idle == 1) {
		__put_idle_task(rq);
	}
	__put_local_task(rq, e_rq->curr);

	/* Clear the lits of threads assigned to this CPU. */
	while (!list_empty(&(e_rq->threads))) {
		struct task_struct* thread = list_first_entry(&(e_rq->threads), struct task_struct, ee.cpu_rq);
		list_del(&(thread->ee.cpu_rq));

		/* Move the local task to the leader CPU. */
		set_local_task_cpu(thread, leader_cpu());
	}
	e_rq->nr_threads = 0;

	/* Reset the pointers to the currently running task and energy task. */
	e_rq->curr = NULL;
	e_rq->curr_task = NULL;

	unlock_local_rq(rq);

	/* Force rescheduling on the runqueue. */
	resched_curr(rq);
}

/* Remove the energy task e_task as currently running one.
 *
 * @rq:		the runqueue of the current CPU.
 * @e_task:	the energy task which should not run any more.
 */
static void put_energy_task(struct rq* rq, struct energy_task* e_task) {
	/* Update the energy task's statistics. */
	update_energy_statistics(e_task);

	e_task->running = 0;

	/* Tell all CPU's to stop executing the threads of the current
	 * energy task. */
	clear_energy_task(rq, e_task);
}

/* Remove the linux task t as currently running one.
 *
 * @rq:		the runqueue of the current CPU.
 * @t:		the task struct of the linux task which should not run
 *		any more.
 */
static void put_local_task(struct rq* rq, struct task_struct* t) {
	lock_local_rq(rq);

	if (rq->en.runs_idle == 1) {
		/* Remove the idle task from the current CPU. */
		__put_idle_task(rq);
	}
	/* Update the threads statistics. */
	__put_local_task(rq, t);

	/* Reset the currently running linux task. */
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

			/* Mark the energy task as selected. */
			next_e_task->running = 1;

			return next_e_task;
		}
	} while (head != list_first_entry(&(grq.tasks), struct energy_task, rq));

	/* We could not find any task. */
	return NULL;
}

/* Pick the idle task to run next on the current rq and do some other preparations.
 *
 * Requires that the lock of the local e_rq is taken.
 *
 * @rq:		the runqueue of the current CPU.
 *
 * @returns:	the prepared task struct of the idle task for this runqueue.
 */
static inline struct task_struct* __pick_idle_task(struct rq* rq) {
	struct task_struct* idle_task = rq->idle;

	/* Raise the idle task in the energy sched class. */
	idle_task->sched_class = &energy_sched_class;

	/* Remember that we are running the idle task. */
	rq->en.runs_idle = 1;

	return idle_task;
}

/* Pick a thread of an energy task which should run next on the current rq.
 *
 * Requires that the lock of the local e_rq is taken.
 *
 * @rq:		the runqueue of the current CPU.
 *
 * @returns:	the task struct of the thread which should run next on this runqueue.
 */
static inline struct task_struct* __pick_local_task(struct rq* rq) {
	struct task_struct* next_task;

	/* Pick the first thread from the list and rotate it so that the next
	 * time another thread gets selected. */
	next_task = list_first_entry(&(rq->en.threads), struct task_struct, ee.cpu_rq);
	list_rotate_left(&(rq->en.threads));

	return next_task;
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

	if (rq->en.nr_threads != 0) {
		/* We have available threads on the runqueue, so pick on of them. */
		next = __pick_local_task(rq);
	} else {
		/* We have no threads to run, so run the idle task. */
		next = __pick_idle_task(rq);
	}

	/* Set that we are now executing the selected thread. */
	next->ee.running = 1;
	rq->en.curr = next;

	/* Set the start point of the next task. And remember how long the
	 * task have been run until now. */
	next->se.exec_start = rq_clock_task(rq);
	next->se.prev_sum_exec_runtime = next->se.sum_exec_runtime;

	unlock_local_rq(rq);

	return next;
}

/* Check if the current CPU is the leader of the energy domain.
 *
 * @rq:		the runqueue of the current CPU.
 *
 * @returns:	whether the current CPU is the leader or not.
 */
static bool is_leader(struct rq* rq) {
	return rq->en.leader == cpu_of(rq);
}

/* Get the leader CPU of the current energy domain.
 *
 * @returns:	the CPU number of the leader CPU.
 */
static unsigned int leader_cpu(void) {
	return this_rq()->en.leader;
}

/* Get the runqueue of the leader CPU of the current energy domain.
 *
 * @returns:	the runqueue of the leader CPU.
 */
static struct rq* leader_rq(void) {
	return cpu_rq(leader_cpu());
}

/* Set and initialize the energy domain of a given CPU.
 *
 * @domain:	the pointer to the cpumask where the energy domain should be
 *		set.
 * @cpu:	the CPU for which we need to initialize the energy domain.
 */
static void init_energy_domain(struct cpumask* domain, unsigned int cpu) {
	/* Currently we simply use all available CPUs.
	 *
	 * TODO: Really find the energy domain of the current CPU. */
	cpumask_setall(domain);
}


/***
 * Functions required for the scheduling class implementation. 
 ***/

/* Add a linux task t to the runqueue.
 *
 * @rq:		the runqueue of the current CPU.
 * @t:		the task struct of the linux task which should be added.
 * @flags:
 */
void enqueue_task_energy(struct rq* rq, struct task_struct* t, int flags) {
	struct energy_task* e_task;

	/* The idea we are following here is to simple add the given linux task
	 * to the list of managed tasks. All the corner cases which we need to
	 * consider are handled in different functions. */

	lock_grq();

	e_task = find_or_create_energy_task(t);

	if (flags & ENQUEUE_WAKEUP) {
		/* Remove the thread from the list of sleeping threads. */
		dequeue_sleeping(rq, e_task, t);
	}

	/* Add the thread to the list of runnable threads. */
	enqueue_runnable(rq, e_task, t);

	unlock_grq();

	if (should_redistribute_energy(e_task, t)) {
		struct redistribute_data data = {
			.e_task = e_task,
			.thread = t,
			.arrived = true
		};

		smp_call_function_single(leader_cpu(), redistribute_energy_task, &data, 1);
	}
}

/* Remove a linux task t from the runqueue.
 *
 * @rq:		the runqueue of the current CPU.
 * @t:		the task struct of the linux task which should be removed.
 * @flags:
 */
void dequeue_task_energy(struct rq* rq, struct task_struct* t, int flags) {
	struct energy_task* e_task;

	/* The idea here is similar to the enqueue case. We simply remove the
	 * given linux task from the list of managed tasks and handle all corner
	 * cases like dead or moved to different class somewhere else. */

	lock_grq();

	e_task = find_energy_task(t);
	
	if (e_task == NULL) {
		/* This should not happen. */
		BUG();
	} else {
		/* Remove the thread from the list of runnable threads. */
		dequeue_runnable(rq, e_task, t);

		if (flags & DEQUEUE_SLEEP) {
			/* Add the thread to the list of sleeping threads. */
			enqueue_sleeping(rq, e_task, t);
		}
	}

	unlock_grq();

	if (should_redistribute_energy(e_task, t)) {
		struct redistribute_data data = {
			.e_task = e_task, 
			.thread = t, 
			.arrived = false
		};

		smp_call_function_single(leader_cpu(), redistribute_energy_task, &data, 1);
	}
}

/* The currently running linux task wants to give up the CPU.
 *
 * @rq:		the current runqueue of the CPU.
 */
void yield_task_energy(struct rq* rq) {
	if (rq->en.nr_threads > 2) {
		/* Yield in this scheduling class will only work if multiple
		 * threads of the same task are assigned to the same CPU. If
		 * this is the case, a local rescheduling is performed. */
		resched_curr_local(rq);
	}
}

/* The current running task wants to give up the CPU to another linux task t.
 *
 * @rq:		the current runqueue of the CPU.
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
 * @rq:		the current runqueue of the CPU.
 * @t:		the task struct of the linux task which should be run in favor
 *		of the current one.
 * @flags:
 */
void check_preempt_curr_energy(struct rq* rq, struct task_struct* t,
			       int flags) {
	/* We are doing nothing here. The currently running task is never preempted
	 * in favor of another one. */
	return;
}

/* Select a new linux task which should run instead of linux task prev.
 *
 * @rq:		the current runqueue of the CPU.
 * @prev:	the task struct of the liunx task which should be replaced.
 *
 * @returns:	the task struct of the linux task which should run next.
 */
struct task_struct* pick_next_task_energy(struct rq* rq,
					  struct task_struct* prev) {
	/* Somewhere we already decided that it is necessary to pick another task,
	 * so don't evaluate this here again. What we need to decide is, which energy
	 * task to run next or if we should exit energy accounting at all. */

	if (is_leader(rq)) {
		struct energy_task* e_task;

		/* Just the leader is allowed to decide which energy task to run next
		 * or if we should return to cfs. */

		lock_grq();
		e_task = find_energy_task(prev);

		if (e_task == NULL) {
			/* Until now no energy task was running. We have to decide
			 * whether we should run an energy task now or not. */
			if (should_switch_to_energy(rq)) {
				/* Select a new energy task. */
				struct energy_task* next_e_task = pick_next_energy_task();
				grq.start_running = rq_clock(rq);
				unlock_grq();

				/* Tell the scheduling class of prev that it is going to be
				 * removed. */
				put_prev_task(rq, prev);

				/* Distribute our decision to the other CPUs of the energy
				 * domain. */
				distribute_energy_task(rq, next_e_task);
			} else {
				/* We don't want or can't schedule an energy task. */
				unlock_grq();
			}
		} else {
			/* There is currently an energy task running. So check if we need
			 * to perform a local reschedule or a global, or if we should give
			 * up scheduling completely. */
			if (should_switch_from_energy(rq)) {
				/* We should switch away from this scheduling class. */
				grq.stop_running = rq_clock(rq);
				unlock_grq();

				/* Account energy. This will also call put_local_task on
				 * all CPUs of the energy domain. */
				put_energy_task(rq, e_task);
			} else if (need_resched_curr_energy(rq)){
				/* We must make a global reschedule. */
				struct energy_task* next_e_task = pick_next_energy_task();
				unlock_grq();

				/* Account the energy. This will also call put_local_task on
				 * all CPUs of the energy domain. */
				put_energy_task(rq, e_task);

				/* Distribute the decision to other CPUs of the energy 
				 * domain. */
				distribute_energy_task(rq, next_e_task);
			} else {
				unlock_grq();
			}
		}
	}
	
	/* Non leader can only make local reschedules. */
	if (need_resched_curr_local(rq)) {
		/* Tell the scheduling class of prev that it is going to be removed. */
		put_prev_task(rq, prev);

		/* Select a new thread which should run on this CPU. */
		pick_next_local_task(rq);
	}

	return rq->en.curr;
}

/* Put a currently running task p back into the runqueue.
 *
 * @rq:		the current runqueue of the CPU.
 * @p:		the task which should give up the CPU.
 */
void put_prev_task_energy(struct rq* rq, struct task_struct* p) {
	put_local_task(rq, p);
}

/* The runqueues current linux task was updated. Either its scheduling class
 * or some priority.
 *
 * @rq:		the current runqueue of the CPU.
 */
void set_curr_task_energy(struct rq* rq) {
	/* As it is only possible to make local scheduling decisions, we need to
	 * perform a rescheduling. */
	resched_curr(rq);
}

/* A scheduling tick happened with the linux task t running.
 *
 * @rq:		the current runqueue of the CPU.
 * @t:		the task for which the scheduling tick happened.
 * @queued:	is the task still in a runqueue.
 */
void task_tick_energy(struct rq* rq, struct task_struct* t, int queued) {

	/* So first of all we need to account the scheduling tick. */
	update_local_statistics(rq, t);

	/* So now we have to decide if it is necessary to make any changes on the current
	 * scheduling process. Energy task decisions can only be made if we are the energy
	 * domain leader. Local decisions can be done at any runqueue. */
	if (is_leader(rq)) {
		lock_grq();

		if (should_switch_energy(rq)) {
			resched_curr_energy(rq);
		}

		unlock_grq();
	}

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
	/* As we don't know the linux task yet, we do nothing here. The linux
	 * task will be added to the list anyway later in the wake_up_new_task
	 * routine. */
	return;
}

/* The linux task t died.
 *
 * @t:		the task struct of the linux task which just died.
 */
void task_dead_energy(struct task_struct* t) {
	struct energy_task* e_task;
	/* If a linux task dies, it could happen, that it was the very last one.
	 * In this case we need to remove the corresponding energy task. The thread
	 * itself was already removed from the list by the schedule routine. */
	lock_grq();

	e_task = find_energy_task(t);				/* The energy task belonging
								   to the linux task. */

	if (e_task == NULL) {
		/* This should not happen, but just be sure. Maybe somebody
		 * already removed the energy task from our list. */
	} else {
		/* Check if there are no threads left for the task. */
		if (e_task->nr_runnable == 0 && e_task->nr_sleeping == 0) {
			/* The last thread was removed from the energy task. Free the
			 * energy task. */
			dequeue_energy_task(e_task);
			kfree(e_task);
		}
	}

	unlock_grq();
}

/* The scheduling class of the linux task t changed to another one.
 *
 * @rq:		the current runqueue of the CPU.
 * @t:		the linux task which was removed from our scheduling class.
 */
void switched_from_energy(struct rq* rq, struct task_struct* t) {
	struct energy_task* e_task;

	/* One of our linux tasks got removed from our class. If this happens,
	 * we also need to remove all the other threads belonging to the same
	 * task from this scheduling class. */
	lock_grq();

	e_task = find_energy_task(t);				/* The energy task belonging 
								   to the linux task. */

	if (e_task == NULL) {
		/* This should not happen as we just removed the given linux task. */
		BUG();
	} else {
		const struct sched_class* new_class = t->sched_class;
								/* The sched class where the task 
								   should be lowered to. */

		struct task_struct* task = e_task->task;	/* The real task. */
		struct task_struct* thread;			/* The threads of the task. */

		/* Lower all the other threads belonging to the same task as the
		 * given linux task. */
		for_each_thread(task, thread) {
			if (thread == t) {
				continue;
			}

			lower_thread(thread, new_class);
			dequeue_thread(task_rq(thread), e_task, thread);
		}

		/* Also free the energy task. */
		dequeue_energy_task(e_task);
		kfree(e_task);
	}

	unlock_grq();
}

/* The scheduling class of the linux task t changed to this one.
 *
 * @rq:		the current runqueue of the CPU.
 * @t:		the linux task which was added to our scheduling class.
 */
void switched_to_energy(struct rq* rq, struct task_struct* t) {
	struct energy_task* e_task;

	/* One linux task got added to our scheduling class. Consequently we need
	 * to raise all the other linux tasks which belong to this one in our class
	 * as well. */
	lock_grq();

	e_task = find_energy_task(t);				/* The energy task belonging
								   to the linux task. */

	if (e_task == NULL) {
		/* This should not happen as we just added the given linux task. */
		BUG();
	} else {
		struct task_struct* task = e_task->task;	/* The real task. */
		struct task_struct* thread;			/* The threads of the task. */

		/* Raise all the other threads belonging to the same task as the
		 * given linux task. */
		for_each_thread(task, thread) {
			if (thread == t) {
				continue;
			}

			raise_thread(thread);
			enqueue_thread(task_rq(thread), e_task, thread);
		}
	}

	unlock_grq();
}

/* The priority of the linux task t changed.
 *
 * @rq:		the current runqueue of the CPU.
 * @t:		the task struct of the linux task for which the priority was changed.
 * @old_prio:	the previous priority of the task.
 */
void prio_changed_energy(struct rq* rq, struct task_struct* p, int old_prio) {
	/* Currently we do not care about priority changes. */
	return;
}

/* Update the runtime statistics of the currently running linux task outside
 * of a schedule tick.
 *
 * @rq:		the current runqueue of the CPU.
 */
void update_curr_energy(struct rq* rq) {
	/* Update the local statistics. */
	update_local_statistics(rq, rq->curr);
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

    .update_curr = update_curr_energy,
};

/***
 * Other external functions. 
 ***/

/* Initialize the per core runqueues.
 *
 * @e_rq:	the energy runqueue which must be initialized.
 * @cpu:	the CPU it which this runqueue is established.
 */
void init_e_rq(struct e_rq* e_rq, unsigned int cpu) {
	raw_spin_lock_init(&(e_rq->lock));

	e_rq->local_resched = e_rq->energy_resched = 0;

	init_energy_domain(&(e_rq->domain), cpu);
	e_rq->leader = cpumask_first(&(e_rq->domain));

	INIT_LIST_HEAD(&(e_rq->threads));
	e_rq->nr_threads = 0;

	e_rq->runs_idle = 0;

	e_rq->curr = e_rq->curr_task = NULL;
}

/* Initialize the energy scheduling class. */
void init_sched_energy_class(void) { 
	init_grq();
}
