//! Bounded stepper diagnostic event queue and execution counters.
//!
//! Events are produced from P1/P2/P3 contexts and drained from a P2 task.
//! The queue is protected with a short interrupt-free critical section so
//! producers never touch the transport from an ISR.

use core::sync::atomic::{AtomicI32, AtomicU32, Ordering::Relaxed};

use foci_firmware::diagnostic::{
    StepperDiagnosticEvent, StepperEventReason, StepperExecutionStats, StepperPerfDiagnosticEvent,
};

use crate::state::stepper::STEPPER_CHANNEL_COUNT;

/// Number of stepper diagnostic events retained before oldest-drop overflow.
pub(crate) const STEPPER_DIAG_QUEUE_DEPTH: usize = 16;
/// Number of fatal performance snapshots retained before oldest-drop overflow.
pub(crate) const STEPPER_PERF_EVENT_QUEUE_DEPTH: usize = 4;

/// Generic trigger-domain critical section site.
pub(crate) const PERF_SITE_TRIGGER_DOMAIN: u8 = 0;
/// `queue_step` trigger-domain critical section site.
pub(crate) const PERF_SITE_QUEUE_STEP: u8 = 1;
/// TIM5 trigger-domain critical section site.
pub(crate) const PERF_SITE_TIM5: u8 = 2;

const CPU_CYCLES_PER_US: u32 = 168;
const CRIT_10US_CYCLES: u32 = 10 * CPU_CYCLES_PER_US;
const CRIT_50US_CYCLES: u32 = 50 * CPU_CYCLES_PER_US;
const CRIT_100US_CYCLES: u32 = 100 * CPU_CYCLES_PER_US;
const CRIT_1000US_CYCLES: u32 = 1000 * CPU_CYCLES_PER_US;

static mut STEPPER_DIAG_EVENTS: heapless::Deque<StepperDiagnosticEvent, STEPPER_DIAG_QUEUE_DEPTH> =
    heapless::Deque::new();
static mut STEPPER_PERF_EVENTS: heapless::Deque<
    StepperPerfDiagnosticEvent,
    STEPPER_PERF_EVENT_QUEUE_DEPTH,
> = heapless::Deque::new();

struct StepperPerfCounters {
    sample_count: AtomicU32,
    crit_count: AtomicU32,
    crit_max_cycles: AtomicU32,
    crit_max_site: AtomicU32,
    crit_over_10us: AtomicU32,
    crit_over_50us: AtomicU32,
    crit_over_100us: AtomicU32,
    crit_over_1000us: AtomicU32,
    queue_step_count: AtomicU32,
    queue_step_max_cycles: AtomicU32,
    tim5_activation_count: AtomicU32,
    tim5_irq_max_cycles: AtomicU32,
    tim5_dispatch_max_cycles: AtomicU32,
    tim5_dispatch_max_cycles_events: AtomicU32,
    tim5_events_max_per_irq: AtomicU32,
    tim5_event_count_total: AtomicU32,
    tim5_defer_count: AtomicU32,
    tim5_empty_count: AtomicU32,
    tim5_events_last_activation: AtomicU32,
    tim5_burst_cycles_per_event_max: AtomicU32,
    tim5_burst_cycles_per_event_max_cycles: AtomicU32,
    tim5_burst_cycles_per_event_max_events: AtomicU32,
    tim5_burst_cycles_per_event_floor3_max: AtomicU32,
    tim5_entry_latency_max_ticks: AtomicU32,
    tim5_pop_lateness_max_ticks: AtomicU32,
    scheduler_cycles_max: AtomicU32,
    scheduler_cycles_events_at_max: AtomicU32,
    scheduler_cycles_per_event_max: AtomicU32,
    scheduler_cycles_per_event_max_cycles: AtomicU32,
    scheduler_cycles_per_event_max_events: AtomicU32,
    scheduler_cycles_per_event_floor3_max: AtomicU32,
    scheduler_full_count: AtomicU32,
    stepper_load_lateness_max_ticks: AtomicU32,
    stepper_load_lateness_last_ticks: AtomicI32,
}

impl StepperPerfCounters {
    const fn new() -> Self {
        Self {
            sample_count: AtomicU32::new(0),
            crit_count: AtomicU32::new(0),
            crit_max_cycles: AtomicU32::new(0),
            crit_max_site: AtomicU32::new(0),
            crit_over_10us: AtomicU32::new(0),
            crit_over_50us: AtomicU32::new(0),
            crit_over_100us: AtomicU32::new(0),
            crit_over_1000us: AtomicU32::new(0),
            queue_step_count: AtomicU32::new(0),
            queue_step_max_cycles: AtomicU32::new(0),
            tim5_activation_count: AtomicU32::new(0),
            tim5_irq_max_cycles: AtomicU32::new(0),
            tim5_dispatch_max_cycles: AtomicU32::new(0),
            tim5_dispatch_max_cycles_events: AtomicU32::new(0),
            tim5_events_max_per_irq: AtomicU32::new(0),
            tim5_event_count_total: AtomicU32::new(0),
            tim5_defer_count: AtomicU32::new(0),
            tim5_empty_count: AtomicU32::new(0),
            tim5_events_last_activation: AtomicU32::new(0),
            tim5_burst_cycles_per_event_max: AtomicU32::new(0),
            tim5_burst_cycles_per_event_max_cycles: AtomicU32::new(0),
            tim5_burst_cycles_per_event_max_events: AtomicU32::new(0),
            tim5_burst_cycles_per_event_floor3_max: AtomicU32::new(0),
            tim5_entry_latency_max_ticks: AtomicU32::new(0),
            tim5_pop_lateness_max_ticks: AtomicU32::new(0),
            scheduler_cycles_max: AtomicU32::new(0),
            scheduler_cycles_events_at_max: AtomicU32::new(0),
            scheduler_cycles_per_event_max: AtomicU32::new(0),
            scheduler_cycles_per_event_max_cycles: AtomicU32::new(0),
            scheduler_cycles_per_event_max_events: AtomicU32::new(0),
            scheduler_cycles_per_event_floor3_max: AtomicU32::new(0),
            scheduler_full_count: AtomicU32::new(0),
            stepper_load_lateness_max_ticks: AtomicU32::new(0),
            stepper_load_lateness_last_ticks: AtomicI32::new(0),
        }
    }

    fn reset(&self) {
        self.sample_count.store(0, Relaxed);
        self.crit_count.store(0, Relaxed);
        self.crit_max_cycles.store(0, Relaxed);
        self.crit_max_site.store(0, Relaxed);
        self.crit_over_10us.store(0, Relaxed);
        self.crit_over_50us.store(0, Relaxed);
        self.crit_over_100us.store(0, Relaxed);
        self.crit_over_1000us.store(0, Relaxed);
        self.queue_step_count.store(0, Relaxed);
        self.queue_step_max_cycles.store(0, Relaxed);
        self.tim5_activation_count.store(0, Relaxed);
        self.tim5_irq_max_cycles.store(0, Relaxed);
        self.tim5_dispatch_max_cycles.store(0, Relaxed);
        self.tim5_dispatch_max_cycles_events.store(0, Relaxed);
        self.tim5_events_max_per_irq.store(0, Relaxed);
        self.tim5_event_count_total.store(0, Relaxed);
        self.tim5_defer_count.store(0, Relaxed);
        self.tim5_empty_count.store(0, Relaxed);
        self.tim5_events_last_activation.store(0, Relaxed);
        self.tim5_burst_cycles_per_event_max.store(0, Relaxed);
        self.tim5_burst_cycles_per_event_max_cycles
            .store(0, Relaxed);
        self.tim5_burst_cycles_per_event_max_events
            .store(0, Relaxed);
        self.tim5_burst_cycles_per_event_floor3_max
            .store(0, Relaxed);
        self.tim5_entry_latency_max_ticks.store(0, Relaxed);
        self.tim5_pop_lateness_max_ticks.store(0, Relaxed);
        self.scheduler_cycles_max.store(0, Relaxed);
        self.scheduler_cycles_events_at_max.store(0, Relaxed);
        self.scheduler_cycles_per_event_max.store(0, Relaxed);
        self.scheduler_cycles_per_event_max_cycles.store(0, Relaxed);
        self.scheduler_cycles_per_event_max_events.store(0, Relaxed);
        self.scheduler_cycles_per_event_floor3_max.store(0, Relaxed);
        self.scheduler_full_count.store(0, Relaxed);
        self.stepper_load_lateness_max_ticks.store(0, Relaxed);
        self.stepper_load_lateness_last_ticks.store(0, Relaxed);
    }
}

static STEPPER_EXECUTION_STATS: StepperExecutionStats<STEPPER_CHANNEL_COUNT> =
    StepperExecutionStats::new();
static STEPPER_PERF_COUNTERS: StepperPerfCounters = StepperPerfCounters::new();

fn record_max(max: &AtomicU32, value: u32) {
    let mut current = max.load(Relaxed);
    while value > current {
        match max.compare_exchange_weak(current, value, Relaxed, Relaxed) {
            Ok(_) => return,
            Err(next) => current = next,
        }
    }
}

fn record_max_with_event_count(
    max: &AtomicU32,
    events_at_max: &AtomicU32,
    value: u32,
    event_count: u32,
) {
    let mut current = max.load(Relaxed);
    while value > current {
        match max.compare_exchange_weak(current, value, Relaxed, Relaxed) {
            Ok(_) => {
                events_at_max.store(event_count, Relaxed);
                return;
            }
            Err(next) => current = next,
        }
    }
}

fn record_max_burst_ratio(
    max: &AtomicU32,
    cycles_at_max: &AtomicU32,
    events_at_max: &AtomicU32,
    ratio: u32,
    cycles: u32,
    event_count: u32,
) {
    let mut current = max.load(Relaxed);
    while ratio > current {
        match max.compare_exchange_weak(current, ratio, Relaxed, Relaxed) {
            Ok(_) => {
                cycles_at_max.store(cycles, Relaxed);
                events_at_max.store(event_count, Relaxed);
                return;
            }
            Err(next) => current = next,
        }
    }
}

fn record_max_with_site(max: &AtomicU32, site: &AtomicU32, value: u32, site_code: u8) {
    let mut current = max.load(Relaxed);
    while value > current {
        match max.compare_exchange_weak(current, value, Relaxed, Relaxed) {
            Ok(_) => {
                site.store(u32::from(site_code), Relaxed);
                return;
            }
            Err(next) => current = next,
        }
    }
}

/// Push one diagnostic event, dropping the oldest event if the queue is full.
pub(crate) fn push_event(event: StepperDiagnosticEvent) {
    cortex_m::interrupt::free(|_| {
        // SAFETY: all accesses to this static go through this interrupt-free
        // critical section.
        let queue = unsafe { &mut *core::ptr::addr_of_mut!(STEPPER_DIAG_EVENTS) };
        if queue.is_full() {
            let _ = queue.pop_front();
        }
        let _ = queue.push_back(event);
    });
}

/// Pop the oldest diagnostic event.
pub(crate) fn pop_event() -> Option<StepperDiagnosticEvent> {
    cortex_m::interrupt::free(|_| {
        // SAFETY: all accesses to this static go through this interrupt-free
        // critical section.
        unsafe { &mut *core::ptr::addr_of_mut!(STEPPER_DIAG_EVENTS) }.pop_front()
    })
}

/// Push one fatal performance event, dropping the oldest event if the queue is full.
pub(crate) fn push_perf_event(event: StepperPerfDiagnosticEvent) {
    cortex_m::interrupt::free(|_| {
        // SAFETY: all accesses to this static go through this interrupt-free
        // critical section.
        let queue = unsafe { &mut *core::ptr::addr_of_mut!(STEPPER_PERF_EVENTS) };
        if queue.is_full() {
            let _ = queue.pop_front();
        }
        let _ = queue.push_back(event);
    });
}

/// Pop the oldest fatal performance event.
pub(crate) fn pop_perf_event() -> Option<StepperPerfDiagnosticEvent> {
    cortex_m::interrupt::free(|_| {
        // SAFETY: all accesses to this static go through this interrupt-free
        // critical section.
        unsafe { &mut *core::ptr::addr_of_mut!(STEPPER_PERF_EVENTS) }.pop_front()
    })
}

/// Capture a fatal step-dispatch performance snapshot.
pub(crate) fn record_perf_event(reason: StepperEventReason, channel: u8, clock: u32) {
    push_perf_event(StepperPerfDiagnosticEvent {
        reason,
        clock,
        snapshot: perf_snapshot(channel, false),
    });
}

/// Record that the board armed the step timer for a new activation.
pub(crate) fn record_timer_activation(channel: u8, clock: u32) {
    STEPPER_EXECUTION_STATS.record_timer_activation(channel, clock);
}

/// Record an accepted `queue_step` segment.
pub(crate) fn record_queued_segment(channel: u8, steps: u32) {
    STEPPER_EXECUTION_STATS.record_queued_segment(channel, steps);
}

/// Record a segment loaded by the timer ISR.
pub(crate) fn record_loaded_segment(channel: u8, steps: u32) {
    STEPPER_EXECUTION_STATS.record_loaded_segment(channel, steps);
}

/// Record a segment loaded by the timer ISR with scheduling context.
pub(crate) fn record_loaded_segment_timing(
    channel: u8,
    steps: u32,
    now: u32,
    scheduled: u32,
    compare: u32,
) {
    STEPPER_EXECUTION_STATS.record_loaded_segment_timing(channel, steps, now, scheduled, compare);
    let lateness = now.wrapping_sub(scheduled) as i32;
    let perf = &STEPPER_PERF_COUNTERS;
    perf.stepper_load_lateness_last_ticks
        .store(lateness, Relaxed);
    if lateness > 0 {
        record_max(&perf.stepper_load_lateness_max_ticks, lateness as u32);
    }
}

/// Record one executed motor step.
pub(crate) fn record_executed_step(channel: u8, forward: bool, clock: u32) {
    STEPPER_EXECUTION_STATS.record_executed_step(channel, forward, clock);
}

/// Record a `queue_step` segment discarded while awaiting reset.
pub(crate) fn record_discarded_segment(channel: u8, steps: u32) {
    STEPPER_EXECUTION_STATS.record_discarded_segment(channel, steps);
}

/// Record a queue-empty step timer stop.
pub(crate) fn record_queue_empty(channel: u8) {
    STEPPER_EXECUTION_STATS.record_queue_empty(channel);
}

/// Record a missed step deadline.
pub(crate) fn record_missed_deadline(channel: u8) {
    STEPPER_EXECUTION_STATS.record_missed_deadline(channel);
}

/// Record a physical stop snapshot before the active segment is reset.
pub(crate) fn record_physical_stop(
    channel: u8,
    reason: foci_firmware::diagnostic::StepperEventReason,
    remaining_events: u32,
    queue_len: u16,
    drained_segments: u32,
    drained_steps: u32,
) {
    STEPPER_EXECUTION_STATS.record_physical_stop(
        channel,
        reason,
        remaining_events,
        queue_len,
        drained_segments,
        drained_steps,
    );
}

/// Record an accepted step-clock reset.
pub(crate) fn record_reset(channel: u8, drained_segments: u32, drained_steps: u32) {
    STEPPER_EXECUTION_STATS.record_reset(channel, drained_segments, drained_steps);
}

/// Record one trigger-domain critical-section duration.
pub(crate) fn record_critical_section(site: u8, cycles: u32) {
    let counters = &STEPPER_PERF_COUNTERS;
    counters.sample_count.fetch_add(1, Relaxed);
    counters.crit_count.fetch_add(1, Relaxed);
    record_max_with_site(
        &counters.crit_max_cycles,
        &counters.crit_max_site,
        cycles,
        site,
    );
    if cycles >= CRIT_10US_CYCLES {
        counters.crit_over_10us.fetch_add(1, Relaxed);
    }
    if cycles >= CRIT_50US_CYCLES {
        counters.crit_over_50us.fetch_add(1, Relaxed);
    }
    if cycles >= CRIT_100US_CYCLES {
        counters.crit_over_100us.fetch_add(1, Relaxed);
    }
    if cycles >= CRIT_1000US_CYCLES {
        counters.crit_over_1000us.fetch_add(1, Relaxed);
    }
    if site == PERF_SITE_QUEUE_STEP {
        counters.queue_step_count.fetch_add(1, Relaxed);
        record_max(&counters.queue_step_max_cycles, cycles);
    }
}

/// Record lateness observed at TIM5 ISR body entry before dispatch starts.
pub(crate) fn record_tim5_entry_latency(lateness_ticks: u32) {
    record_max(
        &STEPPER_PERF_COUNTERS.tim5_entry_latency_max_ticks,
        lateness_ticks,
    );
}

/// Record that TIM5 dispatch deferred after hitting the repeat guard.
pub(crate) fn record_tim5_defer() {
    STEPPER_PERF_COUNTERS.tim5_defer_count.fetch_add(1, Relaxed);
}

/// Record scheduler pop lateness measured before event callback work.
pub(crate) fn record_scheduler_pop_lateness(lateness_ticks: u32) {
    record_max(
        &STEPPER_PERF_COUNTERS.tim5_pop_lateness_max_ticks,
        lateness_ticks,
    );
}

/// Record a failed scheduler insertion caused by a full event table.
pub(crate) fn record_scheduler_full() {
    STEPPER_PERF_COUNTERS
        .scheduler_full_count
        .fetch_add(1, Relaxed);
}

/// Record TIM5-dispatch scheduler bookkeeping cycles.
pub(crate) fn record_scheduler_cycles(scheduler_cycles: u32, event_count: u32) {
    let counters = &STEPPER_PERF_COUNTERS;
    record_max_with_event_count(
        &counters.scheduler_cycles_max,
        &counters.scheduler_cycles_events_at_max,
        scheduler_cycles,
        event_count,
    );
    if let Some(cycles_per_event) = scheduler_cycles.checked_div(event_count) {
        record_max_burst_ratio(
            &counters.scheduler_cycles_per_event_max,
            &counters.scheduler_cycles_per_event_max_cycles,
            &counters.scheduler_cycles_per_event_max_events,
            cycles_per_event,
            scheduler_cycles,
            event_count,
        );
        if event_count >= 3 {
            record_max(
                &counters.scheduler_cycles_per_event_floor3_max,
                cycles_per_event,
            );
        }
    }
}

/// Record one TIM5 IRQ dispatch sample.
pub(crate) fn record_tim5_irq(irq_cycles: u32, dispatch_cycles: u32, event_count: u32) {
    let counters = &STEPPER_PERF_COUNTERS;
    counters.sample_count.fetch_add(1, Relaxed);
    counters.tim5_activation_count.fetch_add(1, Relaxed);
    counters
        .tim5_event_count_total
        .fetch_add(event_count, Relaxed);
    counters
        .tim5_events_last_activation
        .store(event_count, Relaxed);
    if let Some(cycles_per_event) = dispatch_cycles.checked_div(event_count) {
        record_max_burst_ratio(
            &counters.tim5_burst_cycles_per_event_max,
            &counters.tim5_burst_cycles_per_event_max_cycles,
            &counters.tim5_burst_cycles_per_event_max_events,
            cycles_per_event,
            dispatch_cycles,
            event_count,
        );
        if event_count >= 3 {
            record_max(
                &counters.tim5_burst_cycles_per_event_floor3_max,
                cycles_per_event,
            );
        }
    } else {
        counters.tim5_empty_count.fetch_add(1, Relaxed);
    }
    record_max(&counters.tim5_irq_max_cycles, irq_cycles);
    record_max_with_event_count(
        &counters.tim5_dispatch_max_cycles,
        &counters.tim5_dispatch_max_cycles_events,
        dispatch_cycles,
        event_count,
    );
    record_max(&counters.tim5_events_max_per_irq, event_count);
}

/// Return a no-motion snapshot of step-dispatch performance counters.
pub(crate) fn perf_snapshot(
    channel: u8,
    clear: bool,
) -> foci_firmware::diagnostic::StepperPerfSnapshot {
    let counters = &STEPPER_PERF_COUNTERS;
    let snapshot = foci_firmware::diagnostic::StepperPerfSnapshot {
        channel,
        sample_count: counters.sample_count.load(Relaxed),
        crit_count: counters.crit_count.load(Relaxed),
        crit_max_cycles: counters.crit_max_cycles.load(Relaxed),
        crit_max_site: counters.crit_max_site.load(Relaxed) as u8,
        crit_over_10us: counters.crit_over_10us.load(Relaxed),
        crit_over_50us: counters.crit_over_50us.load(Relaxed),
        crit_over_100us: counters.crit_over_100us.load(Relaxed),
        crit_over_1000us: counters.crit_over_1000us.load(Relaxed),
        queue_step_count: counters.queue_step_count.load(Relaxed),
        queue_step_max_cycles: counters.queue_step_max_cycles.load(Relaxed),
        tim5_activation_count: counters.tim5_activation_count.load(Relaxed),
        tim5_irq_max_cycles: counters.tim5_irq_max_cycles.load(Relaxed),
        tim5_dispatch_max_cycles: counters.tim5_dispatch_max_cycles.load(Relaxed),
        tim5_dispatch_max_cycles_events: counters.tim5_dispatch_max_cycles_events.load(Relaxed),
        tim5_events_max_per_irq: counters.tim5_events_max_per_irq.load(Relaxed),
        tim5_event_count_total: counters.tim5_event_count_total.load(Relaxed),
        tim5_defer_count: counters.tim5_defer_count.load(Relaxed),
        tim5_empty_count: counters.tim5_empty_count.load(Relaxed),
        tim5_events_last_activation: counters.tim5_events_last_activation.load(Relaxed),
        tim5_burst_cycles_per_event_max: counters.tim5_burst_cycles_per_event_max.load(Relaxed),
        tim5_burst_cycles_per_event_max_cycles: counters
            .tim5_burst_cycles_per_event_max_cycles
            .load(Relaxed),
        tim5_burst_cycles_per_event_max_events: counters
            .tim5_burst_cycles_per_event_max_events
            .load(Relaxed),
        tim5_burst_cycles_per_event_floor3_max: counters
            .tim5_burst_cycles_per_event_floor3_max
            .load(Relaxed),
        tim5_entry_latency_max_ticks: counters.tim5_entry_latency_max_ticks.load(Relaxed),
        tim5_pop_lateness_max_ticks: counters.tim5_pop_lateness_max_ticks.load(Relaxed),
        scheduler_cycles_max: counters.scheduler_cycles_max.load(Relaxed),
        scheduler_cycles_events_at_max: counters.scheduler_cycles_events_at_max.load(Relaxed),
        scheduler_cycles_per_event_max: counters.scheduler_cycles_per_event_max.load(Relaxed),
        scheduler_cycles_per_event_max_cycles: counters
            .scheduler_cycles_per_event_max_cycles
            .load(Relaxed),
        scheduler_cycles_per_event_max_events: counters
            .scheduler_cycles_per_event_max_events
            .load(Relaxed),
        scheduler_cycles_per_event_floor3_max: counters
            .scheduler_cycles_per_event_floor3_max
            .load(Relaxed),
        scheduler_full_count: counters.scheduler_full_count.load(Relaxed),
        stepper_load_lateness_max_ticks: counters.stepper_load_lateness_max_ticks.load(Relaxed),
        stepper_load_lateness_last_ticks: counters.stepper_load_lateness_last_ticks.load(Relaxed),
        build_trace_enabled: cfg!(feature = "trace"),
    };
    if clear {
        counters.reset();
    }
    snapshot
}

/// Return a no-motion snapshot of stepper execution counters.
pub(crate) fn snapshot(
    channel: u8,
    position: i32,
    timer_active: bool,
    queue_len: u16,
) -> Option<foci_firmware::diagnostic::StepperExecutionSnapshot> {
    STEPPER_EXECUTION_STATS.snapshot(channel, position, timer_active, queue_len)
}
