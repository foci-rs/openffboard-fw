//! Lifecycle-subsystem state (statistics, shutdown tracking, fault,
//! motor arming). Task 8b populates this from `mod app` in main.rs.

/// 64-bit uptime tracking. Updated by the stats task every 5 seconds.
/// `stats_send_time` is the CYCCNT value at last stats send.
/// `stats_send_time_high` counts 32-bit CYCCNT wraps (~25.5s at 168MHz).
pub(crate) static STATS_SEND_TIME: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(0);
pub(crate) static STATS_SEND_TIME_HIGH: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(0);

/// Shutdown active flag. Set by enter_shutdown(), cleared by clear_shutdown().
/// Checked by tmc_control to perform one-shot motor safe-state on first tick.
pub(crate) static SHUTDOWN_ACTIVE: core::sync::atomic::AtomicBool =
    core::sync::atomic::AtomicBool::new(false);

/// Set by fault EXTI, cleared by P1 fault handler after cleanup.
pub(crate) static FAULT_ACTIVE: core::sync::atomic::AtomicBool =
    core::sync::atomic::AtomicBool::new(false);
