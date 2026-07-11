//! Stepper-subsystem board-local state (queue, hardware stepper handles).
//! The cross-priority atomics (position, interval, direction, timer mirror)
//! have moved to `foci_firmware::state::stepper`.

use core::sync::atomic::{AtomicI32, AtomicU32, Ordering};

use foci_firmware::step_pulse::StepPulseTiming;
use foci_klipper::pulse_mapper::PulseMapperConfig;
use foci_klipper::stepper::{HardwareStepper, LogicalStepEvent, MotionScaleConfigError, StepMove};

/// OpenFFBoard TIM5 input clock in hertz.
pub(crate) const STEP_TIMER_HZ: u32 = 84_000_000;

/// Four TIM5 ticks provide the 40 ns minimum high/low pulse phase.
pub(crate) const STEP_HALF_PERIOD_TICKS: u32 = 4;

/// Two half-periods provide the project-owned DIR setup interval.
pub(crate) const STEP_DIR_SETUP_TICKS: u32 = 8;

/// DIR setup plus two complete high/low pulse periods.
pub(crate) const STEP_WAVEFORM_WORST_CASE_TICKS: u32 = 24;

/// PROVISIONAL total P3 direction-change plus two-pulse handler ceiling.
///
/// The STM32F407 core runs at two CPU cycles per TIM5 tick. The release TIM5
/// symbol contains 1,080 static instructions across all mutually exclusive
/// dispatch paths. The direction-change/two-pulse audit reserves 2,048 executed
/// instruction slots, nearly two complete copies of that body, to cover loop
/// repetition and the mapper's out-of-line 64-bit division helper. Charging
/// every slot 12 CPU cycles gives 24,576 cycles, or 12,288 TIM5 ticks; rounding
/// upward to 16,384 ticks also covers the 48 CPU cycles represented by the
/// 24-tick waveform. Task 7 measures the disabled-output path and confirms or
/// replaces this deliberately conservative ceiling.
pub(crate) const PROVISIONAL_TOTAL_HANDLER_WCET_TICKS: u32 = 16_384;

/// PROVISIONAL admission margin for flash/bus contention and audit error.
///
/// The extra 2,048 TIM5 ticks are 4,096 CPU cycles, one eighth of the audited
/// ceiling, and are kept separate so Task 7 can report measured headroom.
pub(crate) const PROVISIONAL_HANDLER_MARGIN_TICKS: u32 = 2_048;

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
