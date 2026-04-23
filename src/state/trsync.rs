//! Trsync-subsystem state (report clocks, trigger reasons, timeouts).
//! Task 8b populates this from `mod app` in main.rs.

/// Homing I-suppression state machine.
/// 0=Idle, 1=SuppressPending, 2=Suppressing, 3=RestorePending
pub(crate) static HOMING_STATE: core::sync::atomic::AtomicU8 = core::sync::atomic::AtomicU8::new(0);

/// TrSync OID that fired the trigger. 0xFF = no trigger pending.
/// Written by TriggerDomain on first trigger, cleared by
/// trsync_trigger command cleanup or config_reset.
pub(crate) static HOMING_TRIGGERED: core::sync::atomic::AtomicU8 =
    core::sync::atomic::AtomicU8::new(0xFF);

/// Indicates trsync_state reports are pending delivery.
/// Set by TIM5 CC2 ISR, cleared by report_task.
pub(crate) static REPORT_PENDING: core::sync::atomic::AtomicBool =
    core::sync::atomic::AtomicBool::new(false);

/// Per-endstop pin identifier (enum value from `config_endstop`).
/// `0xFF` means no pin configured.
pub(crate) static ENDSTOP_PIN_IDS: [core::sync::atomic::AtomicU8;
    foci_klipper::state::NUM_ENDSTOPS] = [
    core::sync::atomic::AtomicU8::new(0xFF),
    core::sync::atomic::AtomicU8::new(0xFF),
    core::sync::atomic::AtomicU8::new(0xFF),
    core::sync::atomic::AtomicU8::new(0xFF),
];
