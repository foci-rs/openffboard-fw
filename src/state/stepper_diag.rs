//! Bounded stepper diagnostic event queue and execution counters.
//!
//! Events are produced from P1/P2/P3 contexts and drained from a P2 task.
//! The queue is protected with a short interrupt-free critical section so
//! producers never touch the transport from an ISR.

use core::sync::atomic::{AtomicI32, AtomicU32, Ordering::Relaxed};

use foci_firmware::diagnostic::{
    StepperDiagnosticEvent, StepperEventReason, StepperPerfDiagnosticEvent,
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

struct StepperExecutionCounters {
    queued_segments: AtomicU32,
    queued_steps: AtomicU32,
    loaded_segments: AtomicU32,
    loaded_steps: AtomicU32,
    executed_pos_steps: AtomicU32,
    executed_neg_steps: AtomicU32,
    activation_count: AtomicU32,
    last_activation_clock: AtomicU32,
    activation_loaded_segments: AtomicU32,
    activation_executed_steps: AtomicU32,
    first_load_now: AtomicU32,
    first_load_scheduled: AtomicU32,
    first_load_compare: AtomicU32,
    first_load_lead_ticks: AtomicI32,
    first_load_compare_delay_ticks: AtomicI32,
    first_step_clock: AtomicU32,
    first_step_delay_ticks: AtomicI32,
    discarded_segments: AtomicU32,
    discarded_steps: AtomicU32,
    queue_empty_count: AtomicU32,
    missed_deadline_count: AtomicU32,
    stop_count: AtomicU32,
    stop_drained_segments: AtomicU32,
    stop_drained_steps: AtomicU32,
    reset_count: AtomicU32,
    reset_drained_segments: AtomicU32,
    reset_drained_steps: AtomicU32,
    last_stop_reason: AtomicU32,
    last_stop_remaining_events: AtomicU32,
    last_stop_queue_len: AtomicU32,
    last_stop_drained_segments: AtomicU32,
    last_stop_drained_steps: AtomicU32,
}

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
    scheduler_full_count: AtomicU32,
    stepper_load_lateness_max_ticks: AtomicU32,
    stepper_load_lateness_last_ticks: AtomicI32,
}

impl StepperExecutionCounters {
    const fn new() -> Self {
        Self {
            queued_segments: AtomicU32::new(0),
            queued_steps: AtomicU32::new(0),
            loaded_segments: AtomicU32::new(0),
            loaded_steps: AtomicU32::new(0),
            executed_pos_steps: AtomicU32::new(0),
            executed_neg_steps: AtomicU32::new(0),
            activation_count: AtomicU32::new(0),
            last_activation_clock: AtomicU32::new(0),
            activation_loaded_segments: AtomicU32::new(0),
            activation_executed_steps: AtomicU32::new(0),
            first_load_now: AtomicU32::new(0),
            first_load_scheduled: AtomicU32::new(0),
            first_load_compare: AtomicU32::new(0),
            first_load_lead_ticks: AtomicI32::new(0),
            first_load_compare_delay_ticks: AtomicI32::new(0),
            first_step_clock: AtomicU32::new(0),
            first_step_delay_ticks: AtomicI32::new(0),
            discarded_segments: AtomicU32::new(0),
            discarded_steps: AtomicU32::new(0),
            queue_empty_count: AtomicU32::new(0),
            missed_deadline_count: AtomicU32::new(0),
            stop_count: AtomicU32::new(0),
            stop_drained_segments: AtomicU32::new(0),
            stop_drained_steps: AtomicU32::new(0),
            reset_count: AtomicU32::new(0),
            reset_drained_segments: AtomicU32::new(0),
            reset_drained_steps: AtomicU32::new(0),
            last_stop_reason: AtomicU32::new(0),
            last_stop_remaining_events: AtomicU32::new(0),
            last_stop_queue_len: AtomicU32::new(0),
            last_stop_drained_segments: AtomicU32::new(0),
            last_stop_drained_steps: AtomicU32::new(0),
        }
    }
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
        self.scheduler_full_count.store(0, Relaxed);
        self.stepper_load_lateness_max_ticks.store(0, Relaxed);
        self.stepper_load_lateness_last_ticks.store(0, Relaxed);
    }
}

static STEPPER_EXECUTION_COUNTERS: [StepperExecutionCounters; STEPPER_CHANNEL_COUNT] =
    [const { StepperExecutionCounters::new() }; STEPPER_CHANNEL_COUNT];

static STEPPER_PERF_COUNTERS: StepperPerfCounters = StepperPerfCounters::new();

fn counters(channel: u8) -> Option<&'static StepperExecutionCounters> {
    STEPPER_EXECUTION_COUNTERS.get(channel as usize)
}

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
    if let Some(counters) = counters(channel) {
        counters.activation_count.fetch_add(1, Relaxed);
        counters.last_activation_clock.store(clock, Relaxed);
        counters.activation_loaded_segments.store(0, Relaxed);
        counters.activation_executed_steps.store(0, Relaxed);
        counters.first_load_now.store(0, Relaxed);
        counters.first_load_scheduled.store(0, Relaxed);
        counters.first_load_compare.store(0, Relaxed);
        counters.first_load_lead_ticks.store(0, Relaxed);
        counters.first_load_compare_delay_ticks.store(0, Relaxed);
        counters.first_step_clock.store(0, Relaxed);
        counters.first_step_delay_ticks.store(0, Relaxed);
    }
}

/// Record an accepted `queue_step` segment.
pub(crate) fn record_queued_segment(channel: u8, steps: u32) {
    if let Some(counters) = counters(channel) {
        counters.queued_segments.fetch_add(1, Relaxed);
        counters.queued_steps.fetch_add(steps, Relaxed);
    }
}

/// Record a segment loaded by the timer ISR.
pub(crate) fn record_loaded_segment(channel: u8, steps: u32) {
    if let Some(counters) = counters(channel) {
        counters.loaded_segments.fetch_add(1, Relaxed);
        counters.loaded_steps.fetch_add(steps, Relaxed);
    }
}

/// Record a segment loaded by the timer ISR with scheduling context.
pub(crate) fn record_loaded_segment_timing(
    channel: u8,
    steps: u32,
    now: u32,
    scheduled: u32,
    compare: u32,
) {
    record_loaded_segment(channel, steps);
    let lateness = now.wrapping_sub(scheduled) as i32;
    let perf = &STEPPER_PERF_COUNTERS;
    perf.stepper_load_lateness_last_ticks
        .store(lateness, Relaxed);
    if lateness > 0 {
        record_max(&perf.stepper_load_lateness_max_ticks, lateness as u32);
    }
    if let Some(counters) = counters(channel)
        && counters.activation_loaded_segments.fetch_add(1, Relaxed) == 0
    {
        counters.first_load_now.store(now, Relaxed);
        counters.first_load_scheduled.store(scheduled, Relaxed);
        counters.first_load_compare.store(compare, Relaxed);
        counters
            .first_load_lead_ticks
            .store(scheduled.wrapping_sub(now) as i32, Relaxed);
        counters
            .first_load_compare_delay_ticks
            .store(compare.wrapping_sub(now) as i32, Relaxed);
    }
}

/// Record one executed motor step.
pub(crate) fn record_executed_step(channel: u8, forward: bool, clock: u32) {
    if let Some(counters) = counters(channel) {
        if forward {
            counters.executed_pos_steps.fetch_add(1, Relaxed);
        } else {
            counters.executed_neg_steps.fetch_add(1, Relaxed);
        }
        if counters.activation_executed_steps.fetch_add(1, Relaxed) == 0 {
            let activation_clock = counters.last_activation_clock.load(Relaxed);
            counters.first_step_clock.store(clock, Relaxed);
            counters
                .first_step_delay_ticks
                .store(clock.wrapping_sub(activation_clock) as i32, Relaxed);
        }
    }
}

/// Record a `queue_step` segment discarded while awaiting reset.
pub(crate) fn record_discarded_segment(channel: u8, steps: u32) {
    if let Some(counters) = counters(channel) {
        counters.discarded_segments.fetch_add(1, Relaxed);
        counters.discarded_steps.fetch_add(steps, Relaxed);
    }
}

/// Record a queue-empty step timer stop.
pub(crate) fn record_queue_empty(channel: u8) {
    if let Some(counters) = counters(channel) {
        counters.queue_empty_count.fetch_add(1, Relaxed);
    }
}

/// Record a missed step deadline.
pub(crate) fn record_missed_deadline(channel: u8) {
    if let Some(counters) = counters(channel) {
        counters.missed_deadline_count.fetch_add(1, Relaxed);
    }
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
    if let Some(counters) = counters(channel) {
        counters.stop_count.fetch_add(1, Relaxed);
        counters
            .stop_drained_segments
            .fetch_add(drained_segments, Relaxed);
        counters
            .stop_drained_steps
            .fetch_add(drained_steps, Relaxed);
        counters.last_stop_reason.store(reason as u32, Relaxed);
        counters
            .last_stop_remaining_events
            .store(remaining_events, Relaxed);
        counters
            .last_stop_queue_len
            .store(u32::from(queue_len), Relaxed);
        counters
            .last_stop_drained_segments
            .store(drained_segments, Relaxed);
        counters
            .last_stop_drained_steps
            .store(drained_steps, Relaxed);
    }
}

/// Record an accepted step-clock reset.
pub(crate) fn record_reset(channel: u8, drained_segments: u32, drained_steps: u32) {
    if let Some(counters) = counters(channel) {
        counters.reset_count.fetch_add(1, Relaxed);
        counters
            .reset_drained_segments
            .fetch_add(drained_segments, Relaxed);
        counters
            .reset_drained_steps
            .fetch_add(drained_steps, Relaxed);
    }
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
    if event_count == 0 {
        counters.tim5_empty_count.fetch_add(1, Relaxed);
    } else {
        let cycles_per_event = dispatch_cycles / event_count;
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
    counters(channel).map(
        |counters| foci_firmware::diagnostic::StepperExecutionSnapshot {
            channel,
            position,
            queued_segments: counters.queued_segments.load(Relaxed),
            queued_steps: counters.queued_steps.load(Relaxed),
            loaded_segments: counters.loaded_segments.load(Relaxed),
            loaded_steps: counters.loaded_steps.load(Relaxed),
            executed_pos_steps: counters.executed_pos_steps.load(Relaxed),
            executed_neg_steps: counters.executed_neg_steps.load(Relaxed),
            activation_count: counters.activation_count.load(Relaxed),
            last_activation_clock: counters.last_activation_clock.load(Relaxed),
            first_load_now: counters.first_load_now.load(Relaxed),
            first_load_scheduled: counters.first_load_scheduled.load(Relaxed),
            first_load_compare: counters.first_load_compare.load(Relaxed),
            first_load_lead_ticks: counters.first_load_lead_ticks.load(Relaxed),
            first_load_compare_delay_ticks: counters.first_load_compare_delay_ticks.load(Relaxed),
            first_step_clock: counters.first_step_clock.load(Relaxed),
            first_step_delay_ticks: counters.first_step_delay_ticks.load(Relaxed),
            discarded_segments: counters.discarded_segments.load(Relaxed),
            discarded_steps: counters.discarded_steps.load(Relaxed),
            queue_empty_count: counters.queue_empty_count.load(Relaxed),
            missed_deadline_count: counters.missed_deadline_count.load(Relaxed),
            stop_count: counters.stop_count.load(Relaxed),
            stop_drained_segments: counters.stop_drained_segments.load(Relaxed),
            stop_drained_steps: counters.stop_drained_steps.load(Relaxed),
            reset_count: counters.reset_count.load(Relaxed),
            reset_drained_segments: counters.reset_drained_segments.load(Relaxed),
            reset_drained_steps: counters.reset_drained_steps.load(Relaxed),
            last_stop_reason: counters.last_stop_reason.load(Relaxed) as u8,
            last_stop_remaining_events: counters.last_stop_remaining_events.load(Relaxed),
            last_stop_queue_len: counters.last_stop_queue_len.load(Relaxed) as u16,
            last_stop_drained_segments: counters.last_stop_drained_segments.load(Relaxed),
            last_stop_drained_steps: counters.last_stop_drained_steps.load(Relaxed),
            timer_active,
            queue_len,
        },
    )
}
