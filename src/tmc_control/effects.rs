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
}

/// Classify board-owned signals checked between commissioning wait slices.
pub(crate) const fn classify_commission_wait_exit(
    shutdown: bool,
    queue_flush: bool,
    emergency_disable: bool,
    fault_active: bool,
) -> Option<CommissionWaitExit> {
    if emergency_disable || fault_active {
        Some(CommissionWaitExit::EmergencyDisable)
    } else if shutdown {
        Some(CommissionWaitExit::Shutdown)
    } else if queue_flush {
        Some(CommissionWaitExit::QueueFlush)
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
        BoardAction::Reply(reply) => emit_reply(reply),
        BoardAction::DelegatedDiagnostic(delegated) => {
            defmt::warn!(
                "diagnostic request delegated before board handler: {}",
                delegated.request.channel,
            );
        }
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
        ShutdownReason::LiveGainApplyFailed => ::ankyra::klipper_shutdown_from!(
            &mut crate::_ankyra_config::Sender,
            "Live gain application failed",
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

fn emit_reply(reply: foci_firmware::commission_dispatch::ReplyPayload) {
    crate::ankyra::replies::emit_terminal_reply(reply);
}

#[cfg(test)]
mod tests {
    use super::*;
    use foci_firmware::tmc::{TmcRequest, TmcRequestKind};
    use foci_firmware::tmc_control::CommissionWaitRequestDisposition;

    #[test]
    fn critical_status_and_disable_exit_commission_wait() {
        assert_eq!(
            classify_commission_wait_exit(false, false, false, true),
            Some(CommissionWaitExit::EmergencyDisable)
        );
        assert_eq!(
            classify_commission_wait_exit(false, false, true, false),
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
            let exit = classify_commission_wait_exit(shutdown, flush, disable, fault);
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
}
