//! Bounded stepper diagnostic event queue and execution counters.
//!
//! Events are produced from P1/P2/P3 contexts and drained from a P2 task.
//! The queue is protected with a short interrupt-free critical section so
//! producers never touch the transport from an ISR.

use core::sync::atomic::{AtomicI32, AtomicU32, Ordering::Relaxed};

use foci_firmware::diagnostic::StepperDiagnosticEvent;

use crate::state::stepper::STEPPER_CHANNEL_COUNT;

/// Number of stepper diagnostic events retained before oldest-drop overflow.
pub(crate) const STEPPER_DIAG_QUEUE_DEPTH: usize = 16;

static mut STEPPER_DIAG_EVENTS: heapless::Deque<StepperDiagnosticEvent, STEPPER_DIAG_QUEUE_DEPTH> =
    heapless::Deque::new();

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

static STEPPER_EXECUTION_COUNTERS: [StepperExecutionCounters; STEPPER_CHANNEL_COUNT] =
    [const { StepperExecutionCounters::new() }; STEPPER_CHANNEL_COUNT];

fn counters(channel: u8) -> Option<&'static StepperExecutionCounters> {
    STEPPER_EXECUTION_COUNTERS.get(channel as usize)
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
