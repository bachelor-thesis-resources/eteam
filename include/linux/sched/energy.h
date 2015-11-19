#ifndef _SCHED_ENERGY_H
#define _SCHED_ENERGY_H

#include <linux/sched/prio.h>

static inline int e_prio(int prio) {
	if (unlikely(prio == ENERGY_PRIO))
		return 1;
	return 0;
}

static inline int e_task(struct task_struct* p) {
	return e_prio(p->prio);
}

#endif
