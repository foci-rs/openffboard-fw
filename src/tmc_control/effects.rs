//! OpenFFBoard effects for shared TMC control runtime actions.

use foci_firmware::tmc_control::{BoardAction, BoardActions, ShutdownReason, StepperAction};
use foci_firmware::tmc_control::{CommissionWaitRequestDisposition, classify_wait_request};

use foci_firmware::tmc::TmcRequest;

use crate::helpers::lifecycle::read_tim5;
use crate::helpers::stepper::stepper_stop_from_p1;

/// Board signal that interrupts an absolute commissioning wait.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum CommissionWaitExit {
    /// Enter the existing shutdown safe-state path.
    Shutdown,
    /// Return to the queue-flush path.
    QueueFlush,
    /// Enter GPIO-first emergency-disable handling.
    EmergencyDisable,
    /// The host-liveness lease has expired; enter the shutdown safe-state
    /// path exactly like `Shutdown` above.
    ClockSyncExpired,
    /// An operator-requested cancel reached this checkpoint. Distinct from
    /// `Shutdown`: the MCU stays running and the run reports a graceful
    /// `Cancelled` terminal instead of requiring `FIRMWARE_RESTART`.
    Cancelled,
}

/// Classify board-owned signals checked between commissioning wait slices.
pub(crate) const fn classify_commission_wait_exit(
    shutdown: bool,
    queue_flush: bool,
    emergency_disable: bool,
    fault_active: bool,
    clock_sync_expired: bool,
    cancel_requested: bool,
) -> Option<CommissionWaitExit> {
    if emergency_disable || fault_active {
        Some(CommissionWaitExit::EmergencyDisable)
    } else if cancel_requested {
        Some(CommissionWaitExit::Cancelled)
    } else if shutdown {
        Some(CommissionWaitExit::Shutdown)
    } else if queue_flush {
        Some(CommissionWaitExit::QueueFlush)
    } else if clock_sync_expired {
        Some(CommissionWaitExit::ClockSyncExpired)
    } else {
        None
    }
}

/// Delegate queued commissioning request policy to the shared classifier.
pub(crate) fn commission_wait_request_disposition(
    request: &TmcRequest,
) -> CommissionWaitRequestDisposition {
    classify_wait_request(&request.kind)
}

/// Apply all actions emitted by one runtime call immediately.
pub(crate) fn apply_actions<F>(actions: BoardActions, mut enter_shutdown: F)
where
    F: FnMut(ShutdownReason),
{
    for action in actions {
        apply_action(action, &mut enter_shutdown);
    }
}

fn apply_action<F>(action: BoardAction, enter_shutdown: &mut F)
where
    F: FnMut(ShutdownReason),
{
    match action {
        BoardAction::Shutdown(reason) => enter_shutdown(reason),
        BoardAction::SignalEmergencyDisable => {
            foci_firmware::state::tmc::TMC_DISABLE_SIGNAL.signal(());
        }
        BoardAction::Stepper(StepperAction::None) => {}
        BoardAction::Stepper(StepperAction::StopNow) => stepper_stop_from_p1(0),
        BoardAction::Stepper(StepperAction::ArmCheck { .. }) => {
            defmt::panic!("ArmCheck requires SPI/backend state and must be handled inline")
        }
        BoardAction::Stepper(StepperAction::ApplyPostAction(_)) => {
            defmt::warn!("TMC runtime post-action requires dispatch state; handled inline")
        }
    }
}

const CLOCK_SYNC_TIMEOUT_TICKS: u32 = 5 * crate::ankyra::constants::CLOCK_FREQ;

/// Whether the host-liveness lease has expired: any channel is
/// physically enabled, and either no valid `get_clock` has ever been
/// received, or more than `CLOCK_SYNC_TIMEOUT_TICKS` have elapsed since
/// the last one. `now` must be a freshly sampled `read_tim5()` at the
/// call site — never a value captured once and reused across iterations.
pub(crate) fn lease_expired(now: u32) -> bool {
    lease_snapshot(now).expired
}

/// Diagnostic breakdown of the lease-expiry decision at `now`, for logging
/// at the point a lease actually trips -- distinguishes a genuinely stale
/// `LAST_HOST_LIVENESS_TICKS` (real stall) from an unexpected `expired`
/// verdict despite a recent value (bookkeeping bug).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct LeaseSnapshot {
    pub(crate) any_enabled: bool,
    pub(crate) has_received_valid_clock: bool,
    pub(crate) last_liveness_ticks: u32,
    pub(crate) elapsed_ticks: u32,
    pub(crate) expired: bool,
}

pub(crate) fn lease_snapshot(now: u32) -> LeaseSnapshot {
    use core::sync::atomic::Ordering::Relaxed;
    use foci_firmware::state::lifecycle::{HAS_RECEIVED_VALID_CLOCK, LAST_HOST_LIVENESS_TICKS};

    let any_enabled = crate::tmc_control::board::tmc_state()
        .enable_pin_asserted
        .load(Relaxed);
    let has_received_valid_clock = HAS_RECEIVED_VALID_CLOCK.load(Relaxed);
    let last_liveness_ticks = LAST_HOST_LIVENESS_TICKS.load(Relaxed);
    let elapsed_ticks = foci_firmware::helpers::clock::elapsed_ticks(now, last_liveness_ticks);
    let expired =
        any_enabled && (!has_received_valid_clock || elapsed_ticks > CLOCK_SYNC_TIMEOUT_TICKS);

    LeaseSnapshot {
        any_enabled,
        has_received_valid_clock,
        last_liveness_ticks,
        elapsed_ticks,
        expired,
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum LeaseCheckDisposition {
    /// The lease has expired and shutdown is not already active: enter
    /// shutdown now and skip the rest of this iteration.
    EnterClockSyncShutdown,
    /// Either shutdown is already active, or the lease has not expired:
    /// proceed with normal iteration work.
    Proceed,
}

/// Decide the disposition above. Always `Proceed` while `shutdown_active`
/// is true — checking `lease_expired` and acting on it while shutdown is
/// already active would starve the existing safe-state gate permanently,
/// since re-entering shutdown is a no-op on repeat calls but a misplaced
/// continue would still fire every iteration. Call this immediately after
/// the existing `SHUTDOWN_ACTIVE` handling block — never before it.
pub(crate) const fn classify_lease_check(
    shutdown_active: bool,
    lease_expired: bool,
) -> LeaseCheckDisposition {
    if !shutdown_active && lease_expired {
        LeaseCheckDisposition::EnterClockSyncShutdown
    } else {
        LeaseCheckDisposition::Proceed
    }
}

/// Emit a Klipper shutdown after the caller has entered MCU shutdown state.
pub(crate) fn emit_shutdown(reason: ShutdownReason) {
    let clock = read_tim5();
    match reason {
        ShutdownReason::BootSpiFailed => ::ankyra::klipper_shutdown_from!(
            &mut crate::_ankyra_config::Sender,
            "TMC4671 SPI failed at boot",
            clock,
        ),
        ShutdownReason::ChipIdMismatch => ::ankyra::klipper_shutdown_from!(
            &mut crate::_ankyra_config::Sender,
            "TMC4671 chip ID mismatch",
            clock,
        ),
        ShutdownReason::InvalidTmcChannel => ::ankyra::klipper_shutdown_from!(
            &mut crate::_ankyra_config::Sender,
            "Invalid TMC channel",
            clock,
        ),
        ShutdownReason::RequestValidationFailed => ::ankyra::klipper_shutdown_from!(
            &mut crate::_ankyra_config::Sender,
            "TMC request validation failed",
            clock,
        ),
        ShutdownReason::CurrentLimitInvalid => ::ankyra::klipper_shutdown_from!(
            &mut crate::_ankyra_config::Sender,
            "TMC current limit invalid",
            clock,
        ),
        ShutdownReason::LiveGainApplyFailed => ::ankyra::klipper_shutdown_from!(
            &mut crate::_ankyra_config::Sender,
            "Live gain application failed",
            clock,
        ),
        ShutdownReason::PendingConfigApplyFailed => ::ankyra::klipper_shutdown_from!(
            &mut crate::_ankyra_config::Sender,
            "TMC pending config apply failed",
            clock,
        ),
        ShutdownReason::AdcCurrentClipped => ::ankyra::klipper_shutdown_from!(
            &mut crate::_ankyra_config::Sender,
            "TMC4671: ADC current clipped (overcurrent)",
            clock,
        ),
        ShutdownReason::TmcFault => ::ankyra::klipper_shutdown_from!(
            &mut crate::_ankyra_config::Sender,
            "TMC4671 fault",
            clock,
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::sync::atomic::Ordering::Relaxed;
    use foci_firmware::state::lifecycle::{HAS_RECEIVED_VALID_CLOCK, LAST_HOST_LIVENESS_TICKS};
    use foci_firmware::tmc::{TmcRequest, TmcRequestKind};
    use foci_firmware::tmc_control::CommissionWaitRequestDisposition;

    /// Serializes every test in this module that touches the global
    /// liveness atomics or `enable_pin_asserted` -- neither is otherwise
    /// synchronized, and `cargo test` runs this module's tests in
    /// parallel by default.
    static TEST_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

    fn reset_lease_state() -> std::sync::MutexGuard<'static, ()> {
        let guard = TEST_LOCK.lock().expect("lease test lock is available");
        HAS_RECEIVED_VALID_CLOCK.store(false, Relaxed);
        LAST_HOST_LIVENESS_TICKS.store(0, Relaxed);
        crate::tmc_control::board::tmc_state()
            .enable_pin_asserted
            .store(false, Relaxed);
        guard
    }

    #[test]
    fn lease_expired_false_when_no_channel_enabled() {
        let _guard = reset_lease_state();
        HAS_RECEIVED_VALID_CLOCK.store(true, Relaxed);
        LAST_HOST_LIVENESS_TICKS.store(0, Relaxed);
        assert!(!lease_expired(CLOCK_SYNC_TIMEOUT_TICKS + 1));
    }

    #[test]
    fn lease_snapshot_matches_lease_expired_when_stale() {
        let _guard = reset_lease_state();
        crate::tmc_control::board::tmc_state()
            .enable_pin_asserted
            .store(true, Relaxed);
        HAS_RECEIVED_VALID_CLOCK.store(true, Relaxed);
        LAST_HOST_LIVENESS_TICKS.store(100, Relaxed);
        let now = 100 + CLOCK_SYNC_TIMEOUT_TICKS + 1;

        let snapshot = lease_snapshot(now);

        assert!(snapshot.expired);
        assert!(snapshot.any_enabled);
        assert!(snapshot.has_received_valid_clock);
        assert_eq!(snapshot.last_liveness_ticks, 100);
        assert_eq!(snapshot.elapsed_ticks, CLOCK_SYNC_TIMEOUT_TICKS + 1);
    }

    #[test]
    fn lease_snapshot_not_expired_when_enabled_and_recent() {
        let _guard = reset_lease_state();
        crate::tmc_control::board::tmc_state()
            .enable_pin_asserted
            .store(true, Relaxed);
        HAS_RECEIVED_VALID_CLOCK.store(true, Relaxed);
        LAST_HOST_LIVENESS_TICKS.store(100, Relaxed);
        let now = 100 + CLOCK_SYNC_TIMEOUT_TICKS - 1;

        let snapshot = lease_snapshot(now);

        assert!(!snapshot.expired);
        assert_eq!(snapshot.elapsed_ticks, CLOCK_SYNC_TIMEOUT_TICKS - 1);
    }

    #[test]
    fn lease_snapshot_expired_when_no_clock_ever_received() {
        let _guard = reset_lease_state();
        crate::tmc_control::board::tmc_state()
            .enable_pin_asserted
            .store(true, Relaxed);

        let snapshot = lease_snapshot(0);

        assert!(snapshot.expired);
        assert!(!snapshot.has_received_valid_clock);
    }

    #[test]
    fn lease_snapshot_not_expired_when_last_liveness_races_ahead_of_now() {
        let _guard = reset_lease_state();
        crate::tmc_control::board::tmc_state()
            .enable_pin_asserted
            .store(true, Relaxed);
        HAS_RECEIVED_VALID_CLOCK.store(true, Relaxed);
        LAST_HOST_LIVENESS_TICKS.store(1_000, Relaxed);
        let now = 999;

        let snapshot = lease_snapshot(now);

        assert!(!snapshot.expired);
        assert_eq!(snapshot.elapsed_ticks, 0);
    }

    #[test]
    fn lease_expired_false_when_enabled_and_recent() {
        let _guard = reset_lease_state();
        crate::tmc_control::board::tmc_state()
            .enable_pin_asserted
            .store(true, Relaxed);
        HAS_RECEIVED_VALID_CLOCK.store(true, Relaxed);
        LAST_HOST_LIVENESS_TICKS.store(100, Relaxed);
        assert!(!lease_expired(100 + CLOCK_SYNC_TIMEOUT_TICKS - 1));
    }

    #[test]
    fn lease_expired_true_when_enabled_and_stale() {
        let _guard = reset_lease_state();
        crate::tmc_control::board::tmc_state()
            .enable_pin_asserted
            .store(true, Relaxed);
        HAS_RECEIVED_VALID_CLOCK.store(true, Relaxed);
        LAST_HOST_LIVENESS_TICKS.store(100, Relaxed);
        assert!(lease_expired(100 + CLOCK_SYNC_TIMEOUT_TICKS + 1));
    }

    #[test]
    fn lease_expired_true_when_enabled_and_no_clock_ever_received() {
        let _guard = reset_lease_state();
        crate::tmc_control::board::tmc_state()
            .enable_pin_asserted
            .store(true, Relaxed);
        assert!(lease_expired(0));
    }

    #[test]
    fn lease_expired_does_not_self_clear_while_still_enabled_and_stale() {
        // Once shutdown is active but the pin has not yet been physically
        // lowered (safe state not yet applied), enable_pin_asserted is
        // still true and lease_expired must keep reporting true on every
        // subsequent check -- it must not self-clear just because a
        // shutdown transition already happened once. The loop's own
        // SHUTDOWN_ACTIVE gate (not this function) is what stops
        // lease_expired from being consulted again until the gate finishes.
        let _guard = reset_lease_state();
        crate::tmc_control::board::tmc_state()
            .enable_pin_asserted
            .store(true, Relaxed);
        HAS_RECEIVED_VALID_CLOCK.store(true, Relaxed);
        LAST_HOST_LIVENESS_TICKS.store(0, Relaxed);
        let now = CLOCK_SYNC_TIMEOUT_TICKS + 1;
        assert!(lease_expired(now));
        assert!(lease_expired(now));
    }

    #[test]
    fn lease_expired_transitions_true_across_an_advancing_sequence() {
        let _guard = reset_lease_state();
        crate::tmc_control::board::tmc_state()
            .enable_pin_asserted
            .store(true, Relaxed);
        HAS_RECEIVED_VALID_CLOCK.store(true, Relaxed);
        LAST_HOST_LIVENESS_TICKS.store(0, Relaxed);

        let slice_ticks = 1_000 * (crate::ankyra::constants::CLOCK_FREQ / 1_000_000);
        let mut now = 0u32;
        let mut saw_expired = false;
        for _ in 0..(CLOCK_SYNC_TIMEOUT_TICKS / slice_ticks + 2) {
            if lease_expired(now) {
                saw_expired = true;
                break;
            }
            now = now.wrapping_add(slice_ticks);
        }
        assert!(
            saw_expired,
            "lease_expired never fired across an advancing sequence"
        );
    }

    #[test]
    fn classify_lease_check_enters_shutdown_when_not_active_and_expired() {
        assert_eq!(
            classify_lease_check(false, true),
            LeaseCheckDisposition::EnterClockSyncShutdown
        );
    }

    #[test]
    fn classify_lease_check_proceeds_when_not_active_and_not_expired() {
        assert_eq!(
            classify_lease_check(false, false),
            LeaseCheckDisposition::Proceed
        );
    }

    #[test]
    fn classify_lease_check_proceeds_when_active_and_expired() {
        // Even though lease_expired is true, an already-active shutdown
        // must never re-trigger through this path.
        assert_eq!(
            classify_lease_check(true, true),
            LeaseCheckDisposition::Proceed
        );
    }

    #[test]
    fn classify_lease_check_proceeds_when_active_and_not_expired() {
        assert_eq!(
            classify_lease_check(true, false),
            LeaseCheckDisposition::Proceed
        );
    }

    #[test]
    fn classify_commission_wait_exit_reports_clock_sync_expired() {
        assert_eq!(
            classify_commission_wait_exit(false, false, false, false, true, false),
            Some(CommissionWaitExit::ClockSyncExpired)
        );
    }

    #[test]
    fn classify_commission_wait_exit_emergency_disable_takes_priority_over_clock_sync() {
        assert_eq!(
            classify_commission_wait_exit(false, false, true, false, true, false),
            Some(CommissionWaitExit::EmergencyDisable)
        );
    }

    #[test]
    fn classify_commission_wait_exit_shutdown_takes_priority_over_clock_sync() {
        assert_eq!(
            classify_commission_wait_exit(true, false, false, false, true, false),
            Some(CommissionWaitExit::Shutdown)
        );
    }

    #[test]
    fn classify_commission_wait_exit_none_when_nothing_set() {
        assert_eq!(
            classify_commission_wait_exit(false, false, false, false, false, false),
            None
        );
    }

    #[test]
    fn critical_status_and_disable_exit_commission_wait() {
        assert_eq!(
            classify_commission_wait_exit(false, false, false, true, false, false),
            Some(CommissionWaitExit::EmergencyDisable)
        );
        assert_eq!(
            classify_commission_wait_exit(false, false, true, false, false, false),
            Some(CommissionWaitExit::EmergencyDisable)
        );
    }

    #[test]
    fn commission_wait_requests_use_shared_disposition() {
        let cancel = TmcRequest::new(0, TmcRequestKind::SetEnable(false));
        let reject = TmcRequest::new(0, TmcRequestKind::Commission { oid: 2, profile: 1 });
        let drop = TmcRequest::new(0, TmcRequestKind::SetCurrent { run_ma: 800 });

        assert_eq!(
            commission_wait_request_disposition(&cancel),
            CommissionWaitRequestDisposition::Cancel
        );
        assert_eq!(
            commission_wait_request_disposition(&reject),
            CommissionWaitRequestDisposition::RejectBusy
        );
        assert_eq!(
            commission_wait_request_disposition(&drop),
            CommissionWaitRequestDisposition::DropBusy
        );
    }

    #[test]
    fn forced_ready_yield_survives_processing_past_classification_time() {
        use foci_firmware::tmc_control::{CommissionPollDriver, PollDriverAction};

        let mut driver = CommissionPollDriver::new();
        for _ in 0..8 {
            assert_eq!(
                driver.classify_ready(10_000).unwrap(),
                PollDriverAction::PollNow
            );
        }
        let action = driver.classify_ready(10_000).unwrap();
        let processing_finished_us = 20_000;

        assert!(processing_finished_us > 10_001);
        assert_eq!(action, PollDriverAction::YieldNow);
    }

    #[test]
    fn signals_arriving_during_yield_exit_before_dispatch_model() {
        let cases = [
            (true, false, false, false, false),
            (false, true, false, false, false),
            (false, false, true, false, true),
            (false, false, false, true, true),
        ];

        for (shutdown, flush, disable, fault, gpio_first) in cases {
            let exit = classify_commission_wait_exit(shutdown, flush, disable, fault, false, false);
            let dispatch = exit.is_none();
            let reply = dispatch;
            let trace = dispatch;
            let next_engine_poll = dispatch;

            assert!(exit.is_some());
            assert_eq!(
                matches!(exit, Some(CommissionWaitExit::EmergencyDisable)),
                gpio_first
            );
            assert!(!dispatch);
            assert!(!reply);
            assert!(!trace);
            assert!(!next_engine_poll);
        }
    }

    #[test]
    fn cancel_requested_exits_wait_when_nothing_else_set() {
        assert_eq!(
            classify_commission_wait_exit(false, false, false, false, false, true),
            Some(CommissionWaitExit::Cancelled)
        );
    }

    #[test]
    fn emergency_disable_takes_priority_over_cancel_requested() {
        assert_eq!(
            classify_commission_wait_exit(false, false, true, false, false, true),
            Some(CommissionWaitExit::EmergencyDisable)
        );
    }

    #[test]
    fn fault_active_takes_priority_over_cancel_requested() {
        assert_eq!(
            classify_commission_wait_exit(false, false, false, true, false, true),
            Some(CommissionWaitExit::EmergencyDisable)
        );
    }

    #[test]
    fn cancel_requested_takes_priority_over_shutdown() {
        assert_eq!(
            classify_commission_wait_exit(true, false, false, false, false, true),
            Some(CommissionWaitExit::Cancelled)
        );
    }

    #[test]
    fn cancel_requested_takes_priority_over_queue_flush() {
        assert_eq!(
            classify_commission_wait_exit(false, true, false, false, false, true),
            Some(CommissionWaitExit::Cancelled)
        );
    }

    #[test]
    fn cancel_requested_takes_priority_over_clock_sync_expired() {
        assert_eq!(
            classify_commission_wait_exit(false, false, false, false, true, true),
            Some(CommissionWaitExit::Cancelled)
        );
    }
}
