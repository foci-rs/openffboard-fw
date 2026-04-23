//! Stepper-subsystem state (queue, position, direction latches, step
//! clock). Task 8b populates this from `mod app` in main.rs.

/// Stepper position target, updated by usb_receive (P2) after each
/// protocol dispatch, read by tmc_control (P1) for SPI writes.
/// Lockfree: AtomicI32 avoids priority ceiling lock that would
/// block P2 protocol tasks.
pub(crate) static STEPPER_POSITION_TARGET: core::sync::atomic::AtomicI32 =
    core::sync::atomic::AtomicI32::new(0);

/// Step count (incremented by P3 ISR, read by P1/P2 for stepper_get_position).
pub(crate) static STEPPER_POSITION: core::sync::atomic::AtomicI32 =
    core::sync::atomic::AtomicI32::new(0);

/// Current step interval in TIM5 clock ticks, written by P3 after
/// every step event. Written to 0 when the step queue drains (both
/// the ISR empty-queue branch and stepper_stop). Read by tmc_control
/// at 1 kHz for velocity feedforward computation.
pub(crate) static STEP_INTERVAL: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(0);

/// Current step direction, written by P3 when direction changes.
/// true = forward, false = reverse. Read by tmc_control for
/// feedforward sign.
pub(crate) static STEP_DIRECTION: core::sync::atomic::AtomicBool =
    core::sync::atomic::AtomicBool::new(true);

/// Read-only mirror of `stepper_stop[0].timer_active` for P1.
/// Written inside TriggerDomain lock via `set_timer_active()`,
/// read by P1 for advisory purposes (PID settle timing).
/// P1 must never enter TriggerDomain.
pub(crate) static STEP_TIMER_ACTIVE_MIRROR: core::sync::atomic::AtomicBool =
    core::sync::atomic::AtomicBool::new(false);

/// Lock-free move queue for hardware step/dir.
/// P2 enqueues, P3 dequeues. When timer is disabled, P1/P2 may drain.
///
/// SAFETY: Access protocol enforced by RTIC priorities:
/// - Timer active: P2 enqueues only (Producer), P3 dequeues only (Consumer)
/// - Timer disabled: P1/P2 may drain (Consumer side safe, P3 quiesced)
pub(crate) static mut STEP_QUEUE: heapless::spsc::Queue<foci_klipper::stepper::StepMove, 1024> =
    heapless::spsc::Queue::new();

/// P2-local HardwareStepper state (direction tracking, prepare_move).
/// Only accessed from usb_task (P2) command handlers.
pub(crate) static mut P2_STEPPER: foci_klipper::stepper::HardwareStepper =
    foci_klipper::stepper::HardwareStepper::new();

/// P3-local HardwareStepper state (active segment, step execution).
/// Only accessed from TIM5 CC1 ISR (P3).
pub(crate) static mut ISR_STEPPER: foci_klipper::stepper::HardwareStepper =
    foci_klipper::stepper::HardwareStepper::new();
