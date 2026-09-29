//! Stepper-subsystem board-local state (queue, hardware stepper handles).

use core::sync::atomic::Ordering;

use foci_klipper::stepper::{HardwareStepper, LogicalStepEvent};

use super::stepper_mapper::{MAPPER_PHASE, PHYSICAL_PULSE_COUNT};
pub(crate) use super::stepper_mapper::{
    STEP_TIMER_HZ, assert_physical_pulse_budget, motion_scale_install_admitted,
    replace_motion_scale_mapper, scheduler_step_deadline_status,
};

/// Four TIM5 ticks provide the 40 ns minimum high/low pulse phase.
pub(crate) const STEP_HALF_PERIOD_TICKS: u32 = 4;

/// Two half-periods provide the project-owned DIR setup interval.
pub(crate) const STEP_DIR_SETUP_TICKS: u32 = 8;

pub(crate) use foci_klipper::step_queue::STEP_QUEUE_DEPTH;

/// Number of hardware stepper runtime channels on OpenFFBoard.
pub(crate) const STEPPER_CHANNEL_COUNT: usize = 1;

/// Lock-free step/dir move queue with explicit producer/consumer ownership.
pub(crate) static STEP_QUEUE_CHANNEL: foci_klipper::step_queue::StepQueueChannel<STEP_QUEUE_DEPTH> =
    foci_klipper::step_queue::StepQueueChannel::new();

/// P2-owned producer handle, populated once by `init_step_queue`.
pub(crate) static mut STEP_QUEUE_PRODUCER: Option<
    foci_klipper::step_queue::Producer<'static, foci_klipper::stepper::StepMove>,
> = None;

/// Split `STEP_QUEUE_CHANNEL` once and store the producer half.
///
/// # Safety
///
/// Caller (RTIC `init`) guarantees this runs exactly once, before any
/// other task that touches `STEP_QUEUE_PRODUCER` or `STEP_QUEUE_CHANNEL`.
pub(crate) unsafe fn init_step_queue() {
    // SAFETY: `init` runs once, before any other task starts; nothing else
    // writes `STEP_QUEUE_PRODUCER`.
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
/// Callable only from P2, or from P1 inside `with_trigger_domain` (the
/// commissioning stimulus): either context excludes every other producer.
pub(crate) unsafe fn step_queue_producer(
    channel: u8,
) -> Option<&'static mut foci_klipper::step_queue::Producer<'static, foci_klipper::stepper::StepMove>>
{
    if channel != 0 {
        return None;
    }
    // SAFETY: caller guarantees exclusive producer access; `init_step_queue` has run
    // by the time any P2 command handler executes. `addr_of_mut!` avoids
    // the edition-2024 `static_mut_refs` lint the same way as above.
    let producer_slot = unsafe { &mut *core::ptr::addr_of_mut!(STEP_QUEUE_PRODUCER) };
    producer_slot.as_mut()
}

/// Producer-side HardwareStepper state (direction tracking, prepare_move).
/// Accessed from usb_task (P2) command handlers, and from the P1
/// commissioning stimulus inside `with_trigger_domain`.
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

/// Publish mapper-observable state after a complete pulse train is low.
pub(crate) fn publish_logical_event(stepper: &HardwareStepper, event: LogicalStepEvent) {
    PHYSICAL_PULSE_COUNT.fetch_add(u32::from(event.physical_pulses), Ordering::Relaxed);
    let phase = stepper
        .mapper_phase()
        .expect("logical event requires an installed mapper");
    MAPPER_PHASE.store(phase as i32, Ordering::Relaxed);
}
