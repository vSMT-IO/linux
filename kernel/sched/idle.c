// SPDX-License-Identifier: GPL-2.0-only
/*
 * Generic entry points for the idle threads and
 * implementation of the idle task scheduling class.
 *
 * (NOTE: these are not related to SCHED_IDLE batch scheduled
 *        tasks which are handled in sched/fair.c )
 */
#include "sched.h"

#include <trace/events/power.h>

/* Linker adds these: start and end of __cpuidle functions */
extern char __cpuidle_text_start[], __cpuidle_text_end[];


//wwj
extern int vsmtio_enable_all;
extern int vsmtio_wa_threshold;
DECLARE_PER_CPU(unsigned long, monitor_flag);
DECLARE_PER_CPU(unsigned long, ht_counter);
DECLARE_PER_CPU(unsigned long, ht_inst);
DECLARE_PER_CPU(unsigned long, ht_cycles);
DECLARE_PER_CPU(unsigned long, ht_stall);
DECLARE_PER_CPU(unsigned long, ht_miss);
DECLARE_PER_CPU(unsigned long, rass_timer);
DECLARE_PER_CPU(unsigned long, ltcr_timer);
DECLARE_PER_CPU(unsigned long, ltcr_round_flag);
DECLARE_PER_CPU(unsigned long, ltcr_rr_num);
DECLARE_PER_CPU(unsigned long, ltcr_ipc);
DECLARE_PER_CPU(unsigned long, ltcr_start);
DECLARE_PER_CPU(unsigned long, wa_timer);
DECLARE_PER_CPU(unsigned long, wa_start);
DECLARE_PER_CPU(unsigned long, wa_avg_rrr);
DECLARE_PER_CPU(unsigned long, wa_std_rrr);
DECLARE_PER_CPU(unsigned long, ltcr_maintain);
DECLARE_PER_CPU(unsigned long, wa_avg_rrr_log);
DECLARE_PER_CPU(unsigned long, wa_std_rrr_log);


/**
 * sched_idle_set_state - Record idle state for the current CPU.
 * @idle_state: State to record.
 */
void sched_idle_set_state(struct cpuidle_state *idle_state)
{
	idle_set_state(this_rq(), idle_state);
}

static int __read_mostly cpu_idle_force_poll;

void cpu_idle_poll_ctrl(bool enable)
{
	if (enable) {
		cpu_idle_force_poll++;
	} else {
		cpu_idle_force_poll--;
		WARN_ON_ONCE(cpu_idle_force_poll < 0);
	}
}

#ifdef CONFIG_GENERIC_IDLE_POLL_SETUP
static int __init cpu_idle_poll_setup(char *__unused)
{
	cpu_idle_force_poll = 1;

	return 1;
}
__setup("nohlt", cpu_idle_poll_setup);

static int __init cpu_idle_nopoll_setup(char *__unused)
{
	cpu_idle_force_poll = 0;

	return 1;
}
__setup("hlt", cpu_idle_nopoll_setup);
#endif

static noinline int __cpuidle cpu_idle_poll(void)
{
	rcu_idle_enter();
	trace_cpu_idle_rcuidle(0, smp_processor_id());
	local_irq_enable();
	stop_critical_timings();

	while (!tif_need_resched() &&
		(cpu_idle_force_poll || tick_check_broadcast_expired()))
		cpu_relax();
	start_critical_timings();
	trace_cpu_idle_rcuidle(PWR_EVENT_EXIT, smp_processor_id());
	rcu_idle_exit();

	return 1;
}

/* Weak implementations for optional arch specific functions */
void __weak arch_cpu_idle_prepare(void) { }
void __weak arch_cpu_idle_enter(void) { }
void __weak arch_cpu_idle_exit(void) { }
void __weak arch_cpu_idle_dead(void) { }
void __weak arch_cpu_idle(void)
{
	cpu_idle_force_poll = 1;
	local_irq_enable();
}

/**
 * default_idle_call - Default CPU idle routine.
 *
 * To use when the cpuidle framework cannot be used.
 */
void __cpuidle default_idle_call(void)
{
	if (current_clr_polling_and_test()) {
		local_irq_enable();
	} else {
		stop_critical_timings();
		arch_cpu_idle();
		start_critical_timings();
	}
}

static int call_cpuidle(struct cpuidle_driver *drv, struct cpuidle_device *dev,
		      int next_state)
{
	/*
	 * The idle task must be scheduled, it is pointless to go to idle, just
	 * update no idle residency and return.
	 */
	if (current_clr_polling_and_test()) {
		dev->last_residency = 0;
		local_irq_enable();
		return -EBUSY;
	}

	/*
	 * Enter the idle state previously returned by the governor decision.
	 * This function will block until an interrupt occurs and will take
	 * care of re-enabling the local interrupts
	 */
	return cpuidle_enter(drv, dev, next_state);
}

/**
 * cpuidle_idle_call - the main idle function
 *
 * NOTE: no locks or semaphores should be used here
 *
 * On archs that support TIF_POLLING_NRFLAG, is called with polling
 * set, and it returns with polling set.  If it ever stops polling, it
 * must clear the polling bit.
 */
static void cpuidle_idle_call(void)
{
	struct cpuidle_device *dev = cpuidle_get_device();
	struct cpuidle_driver *drv = cpuidle_get_cpu_driver(dev);
	int next_state, entered_state;

	/*
	 * Check if the idle task must be rescheduled. If it is the
	 * case, exit the function after re-enabling the local irq.
	 */
	if (need_resched()) {
		local_irq_enable();
		return;
	}

	/*
	 * The RCU framework needs to be told that we are entering an idle
	 * section, so no more rcu read side critical sections and one more
	 * step to the grace period
	 */

	if (cpuidle_not_available(drv, dev)) {
		tick_nohz_idle_stop_tick();
		rcu_idle_enter();

		default_idle_call();
		goto exit_idle;
	}

	/*
	 * Suspend-to-idle ("s2idle") is a system state in which all user space
	 * has been frozen, all I/O devices have been suspended and the only
	 * activity happens here and in iterrupts (if any).  In that case bypass
	 * the cpuidle governor and go stratight for the deepest idle state
	 * available.  Possibly also suspend the local tick and the entire
	 * timekeeping to prevent timer interrupts from kicking us out of idle
	 * until a proper wakeup interrupt happens.
	 */

	if (idle_should_enter_s2idle() || dev->use_deepest_state) {
		if (idle_should_enter_s2idle()) {
			rcu_idle_enter();

			entered_state = cpuidle_enter_s2idle(drv, dev);
			if (entered_state > 0) {
				local_irq_enable();
				goto exit_idle;
			}

			rcu_idle_exit();
		}

		tick_nohz_idle_stop_tick();
		rcu_idle_enter();

		next_state = cpuidle_find_deepest_state(drv, dev);
		call_cpuidle(drv, dev, next_state);
	} else {
		bool stop_tick = true;

		/*
		 * Ask the cpuidle framework to choose a convenient idle state.
		 */
		next_state = cpuidle_select(drv, dev, &stop_tick);

		if (stop_tick || tick_nohz_tick_stopped())
			tick_nohz_idle_stop_tick();
		else
			tick_nohz_idle_retain_tick();

		rcu_idle_enter();

		entered_state = call_cpuidle(drv, dev, next_state);
		/*
		 * Give the governor an opportunity to reflect on the outcome
		 */
		cpuidle_reflect(dev, entered_state);
	}

exit_idle:
	__current_set_polling();

	/*
	 * It is up to the idle functions to reenable local interrupts
	 */
	if (WARN_ON_ONCE(irqs_disabled()))
		local_irq_enable();

	rcu_idle_exit();
}

void do_wa(int cpu)
{
	int _cpu = cpu;
	int nr_cpu = num_possible_cpus();
	int i;
	unsigned long *ptr_wa_std_rrr_log = per_cpu_ptr(&wa_std_rrr_log, cpu);
	unsigned long *ptr_wa_avg_rrr_log = per_cpu_ptr(&wa_avg_rrr_log, cpu);
	unsigned long small_std_rrr = *ptr_wa_std_rrr_log;
	unsigned long avg_rrr = *ptr_wa_avg_rrr_log;
	int target_cpu = cpu;
	unsigned long target_std_rrr = small_std_rrr;
	unsigned long target_avg_rrr = avg_rrr;

	for_each_possible_cpu(i) {
		if (i < nr_cpu/2) {
			unsigned long *_ptr_wa_std_rrr_log = per_cpu_ptr(&wa_std_rrr_log, i);
			if (small_std_rrr > *_ptr_wa_std_rrr_log) {
				small_std_rrr = *_ptr_wa_std_rrr_log;
				unsigned long *_ptr_wa_avg_rrr_log = per_cpu_ptr(&wa_avg_rrr_log, i);
				avg_rrr = *_ptr_wa_avg_rrr_log;
				_cpu = i;
			}
		}
	}

	unsigned long rrr_diff = int_pow((avg_rrr - target_avg_rrr), 2);

	for_each_possible_cpu(i) {
		if (i < nr_cpu/2) {
			unsigned long *__ptr_wa_avg_rrr_log = per_cpu_ptr(&wa_avg_rrr_log, i);
			if (int_pow((avg_rrr - *__ptr_wa_avg_rrr_log), 2) > rrr_diff) {
				target_cpu = i;
				rrr_diff = int_pow((avg_rrr - *__ptr_wa_avg_rrr_log), 2);
				target_avg_rrr = *__ptr_wa_avg_rrr_log;
			}
		}
	}

	if (target_cpu == _cpu) {
		return ;
	} else {
		struct rq* rq = cpu_rq(_cpu);
		struct rq* _rq = cpu_rq(target_cpu);
		struct task_struct *curr = rq->curr;
		struct task_struct *_curr = _rq->curr;
		struct task_struct *t;
		struct task_struct *_t;
		unsigned long vcpu_rrr_diff = int_pow((curr->rrr - avg_rrr), 2);
		struct task_struct *src_vcpu = rq->curr;
		struct task_struct *target_vcpu = _rq->curr;
		unsigned long _vcpu_rrr_diff = int_pow((_curr->rrr - target_avg_rrr), 2);

		for_each_process_thread(curr, t) {
			if ((t->is_vcpu == 1) && (task_cpu(t) == _cpu)) {
				if (vcpu_rrr_diff > int_pow((t->rrr - avg_rrr), 2)) {
					vcpu_rrr_diff = int_pow((t->rrr - avg_rrr), 2);
					src_vcpu = t;
				}
			}
		}

		for_each_process_thread(_curr, _t) {
			if ((_t->is_vcpu == 1) && (task_cpu(t) == target_cpu)) {
				if (_vcpu_rrr_diff > int_pow((_t->rrr - target_avg_rrr), 2)) {
					_vcpu_rrr_diff = int_pow((t->rrr - target_avg_rrr), 2);
					target_vcpu = t;
				}
			}
		}

		rcu_read_lock();
		__set_task_cpu(target_vcpu, _cpu);
		rcu_read_unlock();

		rcu_read_lock();
		__set_task_cpu(src_vcpu, target_cpu);
		rcu_read_unlock();

	}

	return ;
}

void do_ca(struct task_struct *pa, struct task_struct *pb) {

	unsigned long _base;
	unsigned long _stall;
	unsigned long _miss;

	//model trained beforehand
	_base = 1396 * ((pa->sure.base * 1000) / pa->sure.inst);
	_stall = 861 * ((pa->sure.stall * 1000) / pa->sure.inst) + 277 * ((pa->sure.stall * 1000) / pa->sure.inst) * ((pb->sure.stall * 1000) / pb->sure.inst);
	_miss = 1436 * ((pa->sure.miss * 1000) / pa->sure.inst) + 910 * ((pa->sure.miss * 1000) / pa->sure.inst) * ((pb->sure.miss * 1000) / pb->sure.inst);

	pb->ca_slowdown = _base + _stall + _miss;
}

extern void move_to_cpu(const int cpu, struct task_struct *tsk);

void vsmtio_ca(int cpu)
{
	int nr_cpu = num_possible_cpus();
	int pair_cpu = cpu - nr_cpu/2;

	//pair_cpu to cpu
	struct rq *cpu_rq = cpu_rq(pair_cpu);
	struct task_struct *cpu_curr = cpu_rq->curr;
	struct task_struct *t;
	struct task_struct *_t;
	unsigned long slowdown = 0;
	int init_flag = 0;

	for_each_process_thread(cpu_curr, t) {
		if ((t->is_vcpu == 1) && (task_cpu(t) == cpu)) {
			do_ca(cpu_curr, t);
			if (init_flag == 0) {
				init_flag = 1;
				_t = t;
				slowdown = t->ca_slowdown;
			} else {
				if (slowdown > t->ca_slowdown) {
					_t = t;
					slowdown = t->ca_slowdown;
				}
			}
		}
	}

	_t->prio_log = _t->prio;
	_t->prio = 39;
	move_to_cpu(pair_cpu, _t);
	_t->is_moved = 1;
}

/*
 * Generic idle loop implementation
 *
 * Called with polling cleared.
 */
static void do_idle(void)
{
	int cpu = smp_processor_id();
	//wwj
	int i;
	int nr_cpu = num_possible_cpus();
	int wa_flag = 0;
	/*
	 * If the arch has a polling bit, we maintain an invariant:
	 *
	 * Our polling bit is clear if we're not scheduled (i.e. if rq->curr !=
	 * rq->idle). This means that, if rq->idle has the polling bit set,
	 * then setting need_resched is guaranteed to cause the CPU to
	 * reschedule.
	 */

	__current_set_polling();
	tick_nohz_idle_enter();

	while (!need_resched()) {

		if (vsmtio_enable_all) {
			for_each_possible_cpu(i) {
				if (i < nr_cpu/2) {
					unsigned long *ptr_wa_start = per_cpu_ptr(&wa_start, i);
					if (*ptr_wa_start != 1) {
						goto _vsmtio_out;
					}
					unsigned long *ptr_wa_std_rrr_log = per_cpu_ptr(&wa_std_rrr_log, i);
					if (*ptr_wa_std_rrr_log < vsmtio_wa_threshold) {
						wa_flag = 1;
					}
				}

				if (wa_flag == 0) {
					goto _vsmtio_out;
				}

				do_wa(cpu);
			}

			if (cpu >= nr_cpu/2) {
				vsmtio_ca(cpu);
			}
		}
_vsmtio_out:

		check_pgt_cache();
		rmb();

		if (cpu_is_offline(cpu)) {
			tick_nohz_idle_stop_tick_protected();
			cpuhp_report_idle_dead();
			arch_cpu_idle_dead();
		}

		local_irq_disable();
		arch_cpu_idle_enter();

		/*
		 * In poll mode we reenable interrupts and spin. Also if we
		 * detected in the wakeup from idle path that the tick
		 * broadcast device expired for us, we don't want to go deep
		 * idle as we know that the IPI is going to arrive right away.
		 */
		if (cpu_idle_force_poll || tick_check_broadcast_expired()) {
			tick_nohz_idle_restart_tick();
			cpu_idle_poll();
		} else {
			cpuidle_idle_call();
		}
		arch_cpu_idle_exit();
	}

	/*
	 * Since we fell out of the loop above, we know TIF_NEED_RESCHED must
	 * be set, propagate it into PREEMPT_NEED_RESCHED.
	 *
	 * This is required because for polling idle loops we will not have had
	 * an IPI to fold the state for us.
	 */
	preempt_set_need_resched();
	tick_nohz_idle_exit();
	__current_clr_polling();

	/*
	 * We promise to call sched_ttwu_pending() and reschedule if
	 * need_resched() is set while polling is set. That means that clearing
	 * polling needs to be visible before doing these things.
	 */
	smp_mb__after_atomic();

	sched_ttwu_pending();
	schedule_idle();

	if (unlikely(klp_patch_pending(current)))
		klp_update_patch_state(current);
}

bool cpu_in_idle(unsigned long pc)
{
	return pc >= (unsigned long)__cpuidle_text_start &&
		pc < (unsigned long)__cpuidle_text_end;
}

struct idle_timer {
	struct hrtimer timer;
	int done;
};

static enum hrtimer_restart idle_inject_timer_fn(struct hrtimer *timer)
{
	struct idle_timer *it = container_of(timer, struct idle_timer, timer);

	WRITE_ONCE(it->done, 1);
	set_tsk_need_resched(current);

	return HRTIMER_NORESTART;
}

void play_idle(unsigned long duration_ms)
{
	struct idle_timer it;

	/*
	 * Only FIFO tasks can disable the tick since they don't need the forced
	 * preemption.
	 */
	WARN_ON_ONCE(current->policy != SCHED_FIFO);
	WARN_ON_ONCE(current->nr_cpus_allowed != 1);
	WARN_ON_ONCE(!(current->flags & PF_KTHREAD));
	WARN_ON_ONCE(!(current->flags & PF_NO_SETAFFINITY));
	WARN_ON_ONCE(!duration_ms);

	rcu_sleep_check();
	preempt_disable();
	current->flags |= PF_IDLE;
	cpuidle_use_deepest_state(true);

	it.done = 0;
	hrtimer_init_on_stack(&it.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	it.timer.function = idle_inject_timer_fn;
	hrtimer_start(&it.timer, ms_to_ktime(duration_ms), HRTIMER_MODE_REL_PINNED);

	while (!READ_ONCE(it.done))
		do_idle();

	cpuidle_use_deepest_state(false);
	current->flags &= ~PF_IDLE;

	preempt_fold_need_resched();
	preempt_enable();
}
EXPORT_SYMBOL_GPL(play_idle);

void cpu_startup_entry(enum cpuhp_state state)
{

	//wwj
		int my_cpu = smp_processor_id();
		unsigned long *ptr_inst = per_cpu_ptr(&ht_inst, my_cpu);
		unsigned long *ptr_cycles = per_cpu_ptr(&ht_cycles, my_cpu);
		unsigned long *ptr_stall = per_cpu_ptr(&ht_stall, my_cpu);
		unsigned long *ptr_miss = per_cpu_ptr(&ht_miss, my_cpu);
		unsigned long *ptr_mf = per_cpu_ptr(&monitor_flag, my_cpu);
		unsigned long *ptr_counter = per_cpu_ptr(&ht_counter, my_cpu);
		*ptr_inst = 0;
		*ptr_cycles = 0;
		*ptr_stall = 0;
		*ptr_miss = 0;
		*ptr_counter = 0;
		*ptr_mf = 0;
	//if (vsmtio_enable_all) {
		unsigned long *ptr_rass_timer = per_cpu_ptr(&rass_timer, my_cpu);
		unsigned long *ptr_ltcr_timer = per_cpu_ptr(&ltcr_timer, my_cpu);
		unsigned long *ptr_ltcr_round_flag = per_cpu_ptr(&ltcr_round_flag, my_cpu);
		unsigned long *ptr_ltcr_rr_num = per_cpu_ptr(&ltcr_rr_num, my_cpu);
		unsigned long *ptr_ltcr_ipc = per_cpu_ptr(&ltcr_ipc, my_cpu);
		unsigned long *ptr_wa_timer = per_cpu_ptr(&wa_timer, my_cpu);
		unsigned long *ptr_wa_avg_rrr = per_cpu_ptr(&wa_avg_rrr, my_cpu);
		unsigned long *ptr_wa_std_rrr = per_cpu_ptr(&wa_std_rrr, my_cpu);
		unsigned long *ptr_ltcr_start = per_cpu_ptr(&ltcr_start, my_cpu);
		unsigned long *ptr_wa_start = per_cpu_ptr(&wa_start, my_cpu);
		unsigned long *ptr_ltcr_maintain = per_cpu_ptr(&ltcr_maintain, my_cpu);
		unsigned long *ptr_wa_avg_rrr_log = per_cpu_ptr(&wa_avg_rrr_log, my_cpu);
		unsigned long *ptr_wa_std_rrr_log = per_cpu_ptr(&wa_std_rrr_log, my_cpu);

		*ptr_rass_timer = 0;
		*ptr_ltcr_timer = 0;
		*ptr_ltcr_round_flag = 0;
		*ptr_ltcr_rr_num = 0;
		*ptr_ltcr_ipc = 0;
		*ptr_wa_timer = 0;
		*ptr_wa_start = 0;
		*ptr_wa_avg_rrr = 0;
		*ptr_wa_std_rrr = 0;
		*ptr_ltcr_start = 0;
		*ptr_ltcr_maintain = 0;
		*ptr_wa_avg_rrr_log = 0;
		*ptr_wa_std_rrr_log = 0;
	//}

	arch_cpu_idle_prepare();
	cpuhp_online_idle(state);
	while (1)
		do_idle();
}

/*
 * idle-task scheduling class.
 */

#ifdef CONFIG_SMP
static int
select_task_rq_idle(struct task_struct *p, int cpu, int sd_flag, int flags)
{
	return task_cpu(p); /* IDLE tasks as never migrated */
}
#endif

/*
 * Idle tasks are unconditionally rescheduled:
 */
static void check_preempt_curr_idle(struct rq *rq, struct task_struct *p, int flags)
{
	resched_curr(rq);
}

static struct task_struct *
pick_next_task_idle(struct rq *rq, struct task_struct *prev, struct rq_flags *rf)
{
	put_prev_task(rq, prev);
	update_idle_core(rq);
	schedstat_inc(rq->sched_goidle);

	return rq->idle;
}

/*
 * It is not legal to sleep in the idle task - print a warning
 * message if some code attempts to do it:
 */
static void
dequeue_task_idle(struct rq *rq, struct task_struct *p, int flags)
{
	raw_spin_unlock_irq(&rq->lock);
	printk(KERN_ERR "bad: scheduling from the idle thread!\n");
	dump_stack();
	raw_spin_lock_irq(&rq->lock);
}

static void put_prev_task_idle(struct rq *rq, struct task_struct *prev)
{
}

/*
 * scheduler tick hitting a task of our scheduling class.
 *
 * NOTE: This function can be called remotely by the tick offload that
 * goes along full dynticks. Therefore no local assumption can be made
 * and everything must be accessed through the @rq and @curr passed in
 * parameters.
 */
static void task_tick_idle(struct rq *rq, struct task_struct *curr, int queued)
{
}

static void set_curr_task_idle(struct rq *rq)
{
}

static void switched_to_idle(struct rq *rq, struct task_struct *p)
{
	BUG();
}

static void
prio_changed_idle(struct rq *rq, struct task_struct *p, int oldprio)
{
	BUG();
}

static unsigned int get_rr_interval_idle(struct rq *rq, struct task_struct *task)
{
	return 0;
}

static void update_curr_idle(struct rq *rq)
{
}

/*
 * Simple, special scheduling class for the per-CPU idle tasks:
 */
const struct sched_class idle_sched_class = {
	/* .next is NULL */
	/* no enqueue/yield_task for idle tasks */

	/* dequeue is not valid, we print a debug message there: */
	.dequeue_task		= dequeue_task_idle,

	.check_preempt_curr	= check_preempt_curr_idle,

	.pick_next_task		= pick_next_task_idle,
	.put_prev_task		= put_prev_task_idle,

#ifdef CONFIG_SMP
	.select_task_rq		= select_task_rq_idle,
	.set_cpus_allowed	= set_cpus_allowed_common,
#endif

	.set_curr_task          = set_curr_task_idle,
	.task_tick		= task_tick_idle,

	.get_rr_interval	= get_rr_interval_idle,

	.prio_changed		= prio_changed_idle,
	.switched_to		= switched_to_idle,
	.update_curr		= update_curr_idle,
};
