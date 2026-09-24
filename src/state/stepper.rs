//! Stepper-subsystem board-local state (queue, hardware stepper handles).
//! The cross-priority atomics (position, interval, direction, timer mirror)
//! have moved to `foci_firmware::state::stepper`.

use core::sync::atomic::{AtomicI32, AtomicU32, Ordering};

use foci_firmware::step_pulse::StepPulseTiming;
use foci_firmware::stepper_runtime::{StepDeadlineStatus, classify_step_deadline};
use foci_klipper::pulse_mapper::PulseMapperConfig;
use foci_klipper::scheduler::SchedulerEventKind;
use foci_klipper::stepper::{HardwareStepper, LogicalStepEvent, MotionScaleConfigError};

/// OpenFFBoard TIM5 input clock in hertz.
pub(crate) const STEP_TIMER_HZ: u32 = 84_000_000;

/// Four TIM5 ticks provide the 40 ns minimum high/low pulse phase.
pub(crate) const STEP_HALF_PERIOD_TICKS: u32 = 4;

/// Two half-periods provide the project-owned DIR setup interval.
pub(crate) const STEP_DIR_SETUP_TICKS: u32 = 8;

/// DIR setup plus two complete high/low pulse periods.
pub(crate) const STEP_WAVEFORM_WORST_CASE_TICKS: u32 = 24;

/// Physical pulses completed since the most recent mapper installation.
pub(crate) static PHYSICAL_PULSE_COUNT: AtomicU32 = AtomicU32::new(0);

/// Last mapper phase published after STEP returned low.
pub(crate) static MAPPER_PHASE: AtomicI32 = AtomicI32::new(0);

pub(crate) use foci_klipper::step_queue::STEP_QUEUE_DEPTH;

/// Number of hardware stepper runtime channels on OpenFFBoard.
pub(crate) const STEPPER_CHANNEL_COUNT: usize = 1;

/// Lock-free step/dir move queue with explicit producer/consumer ownership.
/// Split once at RTIC `init` via `crate::main::init` -- never touched
/// through the old unsafe accessor again.
pub(crate) static STEP_QUEUE_CHANNEL: foci_klipper::step_queue::StepQueueChannel<STEP_QUEUE_DEPTH> =
    foci_klipper::step_queue::StepQueueChannel::new();

/// P2-owned producer handle, populated once by `init_step_queue`. Nothing
/// else ever touches it, so aliasing is inexpressible.
pub(crate) static mut STEP_QUEUE_PRODUCER: Option<
    foci_klipper::step_queue::Producer<'static, foci_klipper::stepper::StepMove>,
> = None;

/// Split `STEP_QUEUE_CHANNEL` once and store the producer half. Must be
/// called exactly once, from RTIC `init`, before any other stepper
/// task runs.
///
/// # Safety
///
/// Caller (RTIC `init`) guarantees this runs exactly once, before any
/// other task that touches `STEP_QUEUE_PRODUCER` or `STEP_QUEUE_CHANNEL`.
pub(crate) unsafe fn init_step_queue() {
    // SAFETY: `init` runs once, before any other task starts; nothing else
    // writes `STEP_QUEUE_PRODUCER`. `addr_of_mut!` avoids the edition-2024
    // `static_mut_refs` lint by never spelling `&mut STEP_QUEUE_PRODUCER`
    // directly, matching this file's existing `P2_STEPPERS`/`ISR_STEPPERS`
    // accessor pattern.
    unsafe {
        *core::ptr::addr_of_mut!(STEP_QUEUE_PRODUCER) = Some(STEP_QUEUE_CHANNEL.split_once());
    }
}

/// Borrow the P2-owned producer for `channel`. Returns `None` for any
/// channel other than 0 (OpenFFBoard has one physical stepper) or if
/// `init_step_queue` has not yet run.
///
/// # Safety
///
/// Callable only from P2 (the sole owner of `STEP_QUEUE_PRODUCER`).
pub(crate) unsafe fn step_queue_producer(
    channel: u8,
) -> Option<&'static mut foci_klipper::step_queue::Producer<'static, foci_klipper::stepper::StepMove>>
{
    if channel != 0 {
        return None;
    }
    // SAFETY: caller guarantees P2-only access; `init_step_queue` has run
    // by the time any P2 command handler executes. `addr_of_mut!` avoids
    // the edition-2024 `static_mut_refs` lint the same way as above.
    let producer_slot = unsafe { &mut *core::ptr::addr_of_mut!(STEP_QUEUE_PRODUCER) };
    producer_slot.as_mut()
}

/// P2-local HardwareStepper state (direction tracking, prepare_move).
/// Only accessed from usb_task (P2) command handlers.
pub(crate) static mut P2_STEPPERS: [foci_klipper::stepper::HardwareStepper; STEPPER_CHANNEL_COUNT] =
    [const { foci_klipper::stepper::HardwareStepper::new() }];

/// P3-local HardwareStepper state (active segment, step execution).
/// Only accessed from TIM5 CC1 ISR (P3).
pub(crate) static mut ISR_STEPPERS: [foci_klipper::stepper::HardwareStepper;
    STEPPER_CHANNEL_COUNT] = [const { foci_klipper::stepper::HardwareStepper::new() }];

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

/// Return OpenFFBoard's immutable physical waveform timing.
pub(crate) fn step_pulse_timing() -> StepPulseTiming {
    let timing =
        StepPulseTiming::new(STEP_TIMER_HZ).expect("OpenFFBoard step timing constants are valid");
    debug_assert_eq!(
        timing.waveform_worst_case_ticks(),
        STEP_WAVEFORM_WORST_CASE_TICKS
    );
    timing
}

/// Classify actual scheduler-pop lateness only for physical stepper events.
pub(crate) fn scheduler_step_deadline_status(
    kind: SchedulerEventKind,
    scheduled: u32,
    now: u32,
) -> Option<(usize, StepDeadlineStatus)> {
    match kind {
        SchedulerEventKind::Stepper(channel) => Some((
            channel,
            classify_step_deadline(scheduled, now, step_pulse_timing().fatal_lateness_ticks()),
        )),
        SchedulerEventKind::EndstopSample(_)
        | SchedulerEventKind::TrSyncExpire(_)
        | SchedulerEventKind::TrSyncReport(_) => None,
    }
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

/// Failure to replace the board-local mapper while fail-closed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum MotionScaleMapperReplacementError {
    /// A board quiescence guard rejected installation.
    NotAdmitted,
    /// The scheduler rejected the replacement configuration.
    InvalidState,
}

/// Clear mapper state and observables before applying admission and replacement.
pub(crate) fn replace_motion_scale_mapper(
    stepper: &mut HardwareStepper,
    config: PulseMapperConfig,
    admitted: bool,
) -> Result<(), MotionScaleMapperReplacementError> {
    stepper.clear_motion_scale();
    reset_mapper_observables();
    if !admitted {
        return Err(MotionScaleMapperReplacementError::NotAdmitted);
    }
    install_motion_scale_mapper(stepper, config)
        .map_err(|_| MotionScaleMapperReplacementError::InvalidState)
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
    fn openffboard_timing_contains_waveform_and_runtime_threshold_only() {
        let timing = step_pulse_timing();
        assert_eq!(timing.half_period_ticks(), 4);
        assert_eq!(timing.dir_setup_ticks(), 8);
        assert_eq!(timing.waveform_worst_case_ticks(), 24);
        assert_eq!(timing.fatal_lateness_ticks(), 84_000);
    }

    #[test]
    fn scheduler_deadline_classification_applies_only_to_stepper_events() {
        use foci_firmware::stepper_runtime::StepDeadlineStatus;
        use foci_klipper::scheduler::SchedulerEventKind;

        assert_eq!(
            scheduler_step_deadline_status(SchedulerEventKind::Stepper(0), 1_000, 85_001),
            Some((0, StepDeadlineStatus::Fatal { ticks: 84_001 }))
        );
        assert_eq!(
            scheduler_step_deadline_status(SchedulerEventKind::EndstopSample(0), 1_000, 85_001),
            None
        );
    }

    #[test]
    fn invalid_mapper_replacement_clears_prior_mapper_and_observables() {
        let mut stepper = foci_klipper::stepper::HardwareStepper::new();
        stepper
            .configure_motion_scale(PulseMapperConfig::new(3_200).unwrap())
            .unwrap();
        stepper.load_move(StepMove::new(100, 1, 0));
        PHYSICAL_PULSE_COUNT.store(9, Ordering::Relaxed);
        MAPPER_PHASE.store(17, Ordering::Relaxed);

        assert!(
            replace_motion_scale_mapper(
                &mut stepper,
                PulseMapperConfig::new(4_000).unwrap(),
                true,
            )
                .is_err()
        );
        assert!(!stepper.motion_scale_configured());
        assert_eq!(PHYSICAL_PULSE_COUNT.load(Ordering::Relaxed), 0);
        assert_eq!(MAPPER_PHASE.load(Ordering::Relaxed), 0);
    }

    #[test]
    fn mapper_install_admission_requires_every_idle_guard() {
        assert!(motion_scale_install_admitted(false, true, false));
        assert!(!motion_scale_install_admitted(true, true, false));
        assert!(!motion_scale_install_admitted(false, false, false));
        assert!(!motion_scale_install_admitted(false, true, true));
    }

    fn assert_busy_mapper_replacement_fail_closed(
        motor_armed: bool,
        queue_empty: bool,
        timer_active: bool,
    ) {
        let mut stepper = foci_klipper::stepper::HardwareStepper::new();
        stepper
            .configure_motion_scale(PulseMapperConfig::new(3_200).unwrap())
            .unwrap();
        PHYSICAL_PULSE_COUNT.store(9, Ordering::Relaxed);
        MAPPER_PHASE.store(17, Ordering::Relaxed);
        let admitted = motion_scale_install_admitted(motor_armed, queue_empty, timer_active);

        assert!(
            replace_motion_scale_mapper(
                &mut stepper,
                PulseMapperConfig::new(4_000).unwrap(),
                admitted,
            )
            .is_err()
        );
        assert!(!stepper.motion_scale_configured());
        assert_eq!(PHYSICAL_PULSE_COUNT.load(Ordering::Relaxed), 0);
        assert_eq!(MAPPER_PHASE.load(Ordering::Relaxed), 0);
    }

    #[test]
    fn motor_armed_mapper_replacement_is_fail_closed() {
        assert_busy_mapper_replacement_fail_closed(true, true, false);
    }

    #[test]
    fn queue_nonempty_mapper_replacement_is_fail_closed() {
        assert_busy_mapper_replacement_fail_closed(false, false, false);
    }

    #[test]
    fn timer_active_mapper_replacement_is_fail_closed() {
        assert_busy_mapper_replacement_fail_closed(false, true, true);
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
}
