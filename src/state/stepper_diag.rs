//! Bounded stepper diagnostic event queue and execution counters.
//!
//! Events are produced from P1/P2/P3 contexts and drained from a P2 task.
//! The queue is protected with a short interrupt-free critical section so
//! producers never touch the transport from an ISR.

use foci_firmware::diagnostic::{
    StepperDiagnosticEvent, StepperDiagnosticQueue, StepperExecutionStats,
};

use crate::state::stepper::STEPPER_CHANNEL_COUNT;

/// Number of stepper diagnostic events retained before oldest-drop overflow.
pub(crate) const STEPPER_DIAG_QUEUE_DEPTH: usize = 16;

static mut STEPPER_DIAG_EVENTS: StepperDiagnosticQueue<
    StepperDiagnosticEvent,
    STEPPER_DIAG_QUEUE_DEPTH,
> = StepperDiagnosticQueue::new();

static STEPPER_EXECUTION_STATS: StepperExecutionStats<STEPPER_CHANNEL_COUNT> =
    StepperExecutionStats::new();

/// Push one diagnostic event, dropping the oldest event if the queue is full.
pub(crate) fn push_event(event: StepperDiagnosticEvent) {
    cortex_m::interrupt::free(|_| {
        // SAFETY: all accesses to this static go through this interrupt-free
        // critical section.
        unsafe { &mut *core::ptr::addr_of_mut!(STEPPER_DIAG_EVENTS) }.push_drop_oldest(event);
    });
}

/// Pop the oldest diagnostic event.
pub(crate) fn pop_event() -> Option<StepperDiagnosticEvent> {
    cortex_m::interrupt::free(|_| {
        // SAFETY: all accesses to this static go through this interrupt-free
        // critical section.
        unsafe { &mut *core::ptr::addr_of_mut!(STEPPER_DIAG_EVENTS) }.pop()
    })
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
}

/// Record one executed logical motor step and its emitted physical pulses.
pub(crate) fn record_executed_step(channel: u8, forward: bool, physical_pulses: u8, clock: u32) {
    STEPPER_EXECUTION_STATS.record_executed_step(channel, forward, physical_pulses, clock);
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

/// Return a no-motion snapshot of stepper execution counters.
pub(crate) fn snapshot(
    channel: u8,
    position: i32,
    timer_active: bool,
    queue_len: u16,
) -> Option<foci_firmware::diagnostic::StepperExecutionSnapshot> {
    STEPPER_EXECUTION_STATS.snapshot(channel, position, timer_active, queue_len)
}
