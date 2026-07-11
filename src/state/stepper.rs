//! Stepper-subsystem board-local state (queue, hardware stepper handles).
//! The cross-priority atomics (position, interval, direction, timer mirror)
//! have moved to `foci_firmware::state::stepper`.

use core::sync::atomic::{AtomicI32, AtomicU32, Ordering};

use foci_firmware::step_pulse::StepPulseTiming;
use foci_klipper::pulse_mapper::PulseMapperConfig;
use foci_klipper::stepper::{HardwareStepper, LogicalStepEvent, MotionScaleConfigError, StepMove};

/// OpenFFBoard TIM5 input clock in hertz.
pub(crate) const STEP_TIMER_HZ: u32 = 84_000_000;

/// OpenFFBoard's software-audited P3 handler ceiling is not a measurement.
pub(crate) const STEP_TIMING_PROVISIONAL: bool = true;

/// Four TIM5 ticks provide the 40 ns minimum high/low pulse phase.
pub(crate) const STEP_HALF_PERIOD_TICKS: u32 = 4;

/// Two half-periods provide the project-owned DIR setup interval.
pub(crate) const STEP_DIR_SETUP_TICKS: u32 = 8;

/// DIR setup plus two complete high/low pulse periods.
pub(crate) const STEP_WAVEFORM_WORST_CASE_TICKS: u32 = 24;

/// PROVISIONAL outer-dispatch repeat threshold at 84 MHz (100 microseconds).
pub(crate) const PROVISIONAL_DISPATCH_REPEAT_TICKS: u32 = 8_400;

/// PROVISIONAL exception entry/exit, status, lock, and diagnostic wrapper budget.
pub(crate) const PROVISIONAL_ISR_WRAPPER_TICKS: u32 = 512;

/// PROVISIONAL budget for the final dispatch iteration's constant-time scheduler pop.
pub(crate) const PROVISIONAL_SCHEDULER_POP_TICKS: u32 = 512;

/// PROVISIONAL fixed `process_stepper_timer` entry/exit and empty-queue budget.
pub(crate) const PROVISIONAL_STEPPER_CALLBACK_FIXED_TICKS: u32 = 512;

/// Maximum logical events emitted by one stepper callback.
///
/// One one-count segment may already be active. The audit charges all 1,024
/// slots declared by the fixed queue, rather than relying on the ring's
/// reserved-slot detail, so the callback is bounded at 1,025 events before it
/// observes empty or reaches a segment with another step and requeues itself.
pub(crate) const PROVISIONAL_MAX_LOGICAL_EVENTS_PER_CALLBACK: usize = 1_025;

/// PROVISIONAL mapper arithmetic budget per logical event.
///
/// `map_step` has two signed 64-bit Euclidean quotient/remainder paths. This
/// charges each possible software helper 256 TIM5 ticks (512 CPU cycles),
/// enough for 64 quotient bits at eight CPU cycles per bit.
pub(crate) const PROVISIONAL_MAPPER_DIVISION_TICKS: u32 = 512;

/// PROVISIONAL GPIO, loop-control, and bounded-spin overhead beyond waveform time.
///
/// The pulse loop runs only `0..=2`; the exact two-pulse plus DIR waits consume
/// 24 ticks and this term covers loop branches, counter reads, and GPIO writes.
pub(crate) const PROVISIONAL_PULSE_CONTROL_TICKS: u32 = 232;

/// PROVISIONAL mapper publication, atomics, and fixed diagnostic queue work.
pub(crate) const PROVISIONAL_EVENT_STATE_TICKS: u32 = 256;

/// PROVISIONAL fixed-capacity move-queue transition budget per event.
///
/// Heapless SPSC dequeue is constant time. Charging this on every event also
/// covers the maximum 1,024 transitions through the fixed queue conservatively.
pub(crate) const PROVISIONAL_QUEUE_TRANSITION_TICKS: u32 = 256;

/// Itemized PROVISIONAL per-event subtotal before upward rounding.
pub(crate) const PROVISIONAL_LOGICAL_EVENT_SUBTOTAL_TICKS: u32 = PROVISIONAL_MAPPER_DIVISION_TICKS
    + STEP_WAVEFORM_WORST_CASE_TICKS
    + PROVISIONAL_PULSE_CONTROL_TICKS
    + PROVISIONAL_EVENT_STATE_TICKS
    + PROVISIONAL_QUEUE_TRANSITION_TICKS;

/// PROVISIONAL rounded per-logical-event control-flow budget.
///
/// The itemized subtotal is 1,280 ticks: 512 mapper + 24 waveform + 232 pulse
/// control + 256 state publication + 256 queue transition. It rounds upward
/// to 2,048 ticks without relying on runtime measurement.
pub(crate) const PROVISIONAL_LOGICAL_EVENT_TICKS: u32 =
    PROVISIONAL_LOGICAL_EVENT_SUBTOTAL_TICKS.next_power_of_two();

/// PROVISIONAL final scheduler requeue budget.
///
/// Requeue examines at most 13 fixed scheduler slots for a free slot and at
/// most 12 linked entries for insertion. Charging 64 ticks to each of those
/// 25 candidates gives 1,600 ticks, rounded upward to 2,048.
pub(crate) const PROVISIONAL_SCHEDULER_REQUEUE_TICKS: u32 = 2_048;

/// PROVISIONAL defer, lock-exit, diagnostic, and ISR-tail budget.
pub(crate) const PROVISIONAL_POST_DISPATCH_TICKS: u32 = 1_024;

/// Itemized PROVISIONAL bounded-control-flow subtotal before upward rounding.
pub(crate) const PROVISIONAL_CONTROL_FLOW_SUBTOTAL_TICKS: u32 = PROVISIONAL_DISPATCH_REPEAT_TICKS
    + PROVISIONAL_ISR_WRAPPER_TICKS
    + PROVISIONAL_SCHEDULER_POP_TICKS
    + PROVISIONAL_STEPPER_CALLBACK_FIXED_TICKS
    + PROVISIONAL_MAX_LOGICAL_EVENTS_PER_CALLBACK as u32 * PROVISIONAL_LOGICAL_EVENT_TICKS
    + PROVISIONAL_SCHEDULER_REQUEUE_TICKS
    + PROVISIONAL_POST_DISPATCH_TICKS;

/// PROVISIONAL total P3 handler ceiling derived from bounded control flow.
///
/// Dispatch processes callbacks until 8,400 ticks have elapsed, then checks the
/// threshold only after a callback. Therefore total time is bounded by the
/// repeat window plus one maximum callback and wrapper/tail work. The itemized
/// subtotal is 2,112,208 ticks; this ceiling rounds that upward to 4,194,304.
/// It is a deliberately restrictive static audit, not a measurement. Task 7
/// must measure the disabled-output path and confirm or replace it.
pub(crate) const PROVISIONAL_TOTAL_HANDLER_WCET_TICKS: u32 =
    PROVISIONAL_CONTROL_FLOW_SUBTOTAL_TICKS.next_power_of_two();

/// PROVISIONAL admission margin for flash/bus contention and audit error.
///
/// The separate 524,288 ticks are one eighth of the rounded ceiling and keep
/// nonzero uncertainty headroom outside the control-flow subtotal.
pub(crate) const PROVISIONAL_HANDLER_MARGIN_TICKS: u32 = 524_288;

/// Physical pulses completed since the most recent mapper installation.
pub(crate) static PHYSICAL_PULSE_COUNT: AtomicU32 = AtomicU32::new(0);

/// Last mapper phase published after STEP returned low.
pub(crate) static MAPPER_PHASE: AtomicI32 = AtomicI32::new(0);

/// Depth of each hardware step/dir move queue.
pub(crate) const STEP_QUEUE_DEPTH: usize = 1024;

/// Number of hardware stepper runtime channels on OpenFFBoard.
pub(crate) const STEPPER_CHANNEL_COUNT: usize = 1;

/// Lock-free move queues for hardware step/dir.
///
/// P2 enqueues, P3 dequeues. When timer is disabled, P1/P2 may drain.
///
/// SAFETY: Access protocol enforced by RTIC priorities:
/// - Timer active: P2 enqueues only (Producer), P3 dequeues only (Consumer)
/// - Timer disabled: P1/P2 may drain (Consumer side safe, P3 quiesced)
pub(crate) static mut STEP_QUEUES: [heapless::spsc::Queue<
    foci_klipper::stepper::StepMove,
    STEP_QUEUE_DEPTH,
>; STEPPER_CHANNEL_COUNT] = [const { heapless::spsc::Queue::new() }];

/// P2-local HardwareStepper state (direction tracking, prepare_move).
/// Only accessed from usb_task (P2) command handlers.
pub(crate) static mut P2_STEPPERS: [foci_klipper::stepper::HardwareStepper; STEPPER_CHANNEL_COUNT] =
    [const { foci_klipper::stepper::HardwareStepper::new() }];

/// P3-local HardwareStepper state (active segment, step execution).
/// Only accessed from TIM5 CC1 ISR (P3).
pub(crate) static mut ISR_STEPPERS: [foci_klipper::stepper::HardwareStepper;
    STEPPER_CHANNEL_COUNT] = [const { foci_klipper::stepper::HardwareStepper::new() }];

/// Return the hardware step queue for `channel`.
///
/// # Safety
///
/// Callers must uphold OpenFFBoard's per-channel RTIC access discipline for
/// the returned queue and must not create overlapping mutable references to
/// the same channel.
#[inline]
pub(crate) unsafe fn step_queue(
    channel: u8,
) -> Option<&'static mut heapless::spsc::Queue<foci_klipper::stepper::StepMove, STEP_QUEUE_DEPTH>> {
    let idx = channel as usize;
    if idx >= STEPPER_CHANNEL_COUNT {
        return None;
    }
    let ptr = core::ptr::addr_of_mut!(STEP_QUEUES)
        .cast::<heapless::spsc::Queue<foci_klipper::stepper::StepMove, STEP_QUEUE_DEPTH>>();
    // SAFETY: idx was bounds-checked above. The caller guarantees exclusive
    // mutable access to this channel under the board's RTIC access discipline.
    Some(unsafe { &mut *ptr.add(idx) })
}

/// Return the P2-local hardware stepper state for `channel`.
///
/// # Safety
///
/// Callers must uphold OpenFFBoard's per-channel RTIC access discipline for
/// the returned stepper state and must not create overlapping mutable
/// references to the same channel.
#[inline]
pub(crate) unsafe fn p2_stepper(
    channel: u8,
) -> Option<&'static mut foci_klipper::stepper::HardwareStepper> {
    let idx = channel as usize;
    if idx >= STEPPER_CHANNEL_COUNT {
        return None;
    }
    let ptr = core::ptr::addr_of_mut!(P2_STEPPERS).cast::<foci_klipper::stepper::HardwareStepper>();
    // SAFETY: idx was bounds-checked above. The caller guarantees exclusive
    // mutable access to this channel under the board's RTIC access discipline.
    Some(unsafe { &mut *ptr.add(idx) })
}

/// Return the ISR-local hardware stepper state for `channel`.
///
/// # Safety
///
/// Callers must uphold OpenFFBoard's per-channel RTIC access discipline for
/// the returned stepper state and must not create overlapping mutable
/// references to the same channel.
#[inline]
pub(crate) unsafe fn isr_stepper(
    channel: u8,
) -> Option<&'static mut foci_klipper::stepper::HardwareStepper> {
    let idx = channel as usize;
    if idx >= STEPPER_CHANNEL_COUNT {
        return None;
    }
    let ptr =
        core::ptr::addr_of_mut!(ISR_STEPPERS).cast::<foci_klipper::stepper::HardwareStepper>();
    // SAFETY: idx was bounds-checked above. The caller guarantees exclusive
    // mutable access to this channel under the board's RTIC access discipline.
    Some(unsafe { &mut *ptr.add(idx) })
}

/// Return OpenFFBoard's immutable, PROVISIONAL logical-event timing budget.
pub(crate) fn step_pulse_timing() -> StepPulseTiming {
    let timing = StepPulseTiming::new(
        STEP_TIMER_HZ,
        PROVISIONAL_TOTAL_HANDLER_WCET_TICKS,
        PROVISIONAL_HANDLER_MARGIN_TICKS,
    )
    .expect("OpenFFBoard step timing constants are valid");
    debug_assert_eq!(
        timing.waveform_worst_case_ticks(),
        STEP_WAVEFORM_WORST_CASE_TICKS
    );
    timing
}

/// Clear any old mapper, then install a freshly centered replacement.
///
/// Failure is fail-closed: the prior mapper is cleared before configuration.
pub(crate) fn install_motion_scale_mapper(
    stepper: &mut HardwareStepper,
    config: PulseMapperConfig,
) -> Result<(), MotionScaleConfigError> {
    stepper.clear_motion_scale();
    stepper.configure_motion_scale(config)
}

/// Admit mapper installation only while every board quiescence guard holds.
pub(crate) const fn motion_scale_install_admitted(
    motor_armed: bool,
    queue_empty: bool,
    timer_active: bool,
) -> bool {
    !motor_armed && queue_empty && !timer_active
}

/// Enforce the mapper's production pulse-count contract in release builds.
pub(crate) const fn assert_physical_pulse_budget(physical_pulses: u8) {
    assert!(
        physical_pulses <= 2,
        "logical event exceeds two physical pulses"
    );
}

/// Prepare and enqueue without consuming P2 direction when the queue is full.
pub(crate) fn prepare_and_enqueue<const N: usize>(
    stepper: &mut HardwareStepper,
    queue: &mut heapless::spsc::Queue<StepMove, N>,
    move_: StepMove,
) -> Result<usize, usize> {
    if queue.is_full() {
        return Err(queue.len());
    }
    let prepared = stepper.prepare_move(move_);
    // P2 is the only producer, so a consumer can only create more space after
    // the full check. Enqueue therefore cannot fail after direction changes.
    queue.enqueue(prepared).map_err(|_| queue.len())?;
    Ok(queue.len())
}

/// Publish mapper-observable state after a complete pulse train is low.
pub(crate) fn publish_logical_event(stepper: &HardwareStepper, event: LogicalStepEvent) {
    PHYSICAL_PULSE_COUNT.fetch_add(u32::from(event.physical_pulses), Ordering::Relaxed);
    let phase = stepper
        .mapper_phase()
        .expect("logical event requires an installed mapper");
    MAPPER_PHASE.store(phase as i32, Ordering::Relaxed);
}

/// Reset mapper-observable counters at the new common origin.
pub(crate) fn reset_mapper_observables() {
    PHYSICAL_PULSE_COUNT.store(0, Ordering::Relaxed);
    MAPPER_PHASE.store(0, Ordering::Relaxed);
}

#[cfg(test)]
mod tests {
    use super::*;
    use foci_klipper::pulse_mapper::PulseMapperConfig;
    use foci_klipper::stepper::StepMove;

    #[test]
    fn openffboard_timing_uses_provisional_wcet_and_margin() {
        let timing = step_pulse_timing();
        assert!(STEP_TIMING_PROVISIONAL);
        assert_eq!(timing.half_period_ticks(), 4);
        assert_eq!(timing.dir_setup_ticks(), 8);
        assert_eq!(timing.waveform_worst_case_ticks(), 24);
        assert_eq!(timing.half_period_ticks(), STEP_HALF_PERIOD_TICKS);
        assert_eq!(timing.dir_setup_ticks(), STEP_DIR_SETUP_TICKS);
        assert_eq!(
            timing.waveform_worst_case_ticks(),
            STEP_WAVEFORM_WORST_CASE_TICKS
        );
        assert_eq!(
            timing.total_handler_wcet_ticks(),
            PROVISIONAL_TOTAL_HANDLER_WCET_TICKS
        );
        assert_eq!(timing.margin_ticks(), PROVISIONAL_HANDLER_MARGIN_TICKS);
        assert_eq!(PROVISIONAL_DISPATCH_REPEAT_TICKS, STEP_TIMER_HZ / 10_000);
        assert_eq!(PROVISIONAL_MAX_LOGICAL_EVENTS_PER_CALLBACK, 1_025);
        assert_eq!(
            PROVISIONAL_MAX_LOGICAL_EVENTS_PER_CALLBACK,
            STEP_QUEUE_DEPTH + 1
        );
        assert_eq!(PROVISIONAL_LOGICAL_EVENT_SUBTOTAL_TICKS, 1_280);
        assert_eq!(PROVISIONAL_LOGICAL_EVENT_TICKS, 2_048);
        assert_eq!(PROVISIONAL_CONTROL_FLOW_SUBTOTAL_TICKS, 2_112_208);
        assert!(PROVISIONAL_TOTAL_HANDLER_WCET_TICKS >= PROVISIONAL_CONTROL_FLOW_SUBTOTAL_TICKS);
        assert_eq!(PROVISIONAL_TOTAL_HANDLER_WCET_TICKS, 4_194_304);
        assert_eq!(PROVISIONAL_HANDLER_MARGIN_TICKS, 524_288);
    }

    #[test]
    fn failed_mapper_replacement_leaves_no_prior_mapper_usable() {
        let mut stepper = foci_klipper::stepper::HardwareStepper::new();
        stepper
            .configure_motion_scale(PulseMapperConfig::new(3_200).unwrap())
            .unwrap();
        stepper.load_move(StepMove::new(100, 1, 0));

        assert!(
            install_motion_scale_mapper(&mut stepper, PulseMapperConfig::new(4_000).unwrap())
                .is_err()
        );
        assert!(!stepper.motion_scale_configured());
    }

    #[test]
    fn mapper_install_admission_requires_every_idle_guard() {
        assert!(motion_scale_install_admitted(false, true, false));
        assert!(!motion_scale_install_admitted(true, true, false));
        assert!(!motion_scale_install_admitted(false, false, false));
        assert!(!motion_scale_install_admitted(false, true, true));
    }

    #[test]
    fn physical_pulse_budget_accepts_zero_through_two() {
        assert_physical_pulse_budget(0);
        assert_physical_pulse_budget(1);
        assert_physical_pulse_budget(2);
    }

    #[test]
    #[should_panic(expected = "logical event exceeds two physical pulses")]
    fn physical_pulse_budget_rejects_three_in_release_code() {
        assert_physical_pulse_budget(3);
    }

    #[test]
    fn queue_full_does_not_consume_pending_direction_change() {
        let mut stepper = foci_klipper::stepper::HardwareStepper::new();
        stepper.set_next_step_dir(false);
        let mut queue = heapless::spsc::Queue::<StepMove, 2>::new();
        queue.enqueue(StepMove::new(100, 1, 0)).unwrap();

        assert_eq!(
            prepare_and_enqueue(&mut stepper, &mut queue, StepMove::new(100, 1, 0)),
            Err(1)
        );
        assert!(stepper.next_move_changes_direction());
    }
}
