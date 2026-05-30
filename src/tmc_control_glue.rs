//! OpenFFBoard glue for applying shared TMC control runtime actions.

use foci_firmware::tmc_control::{
    BoardAction, BoardActions, DelegatedDiagnostic, ShutdownReason, StepperAction,
};

use crate::helpers::lifecycle::read_tim5;
use crate::helpers::stepper::stepper_stop_from_p1;

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
        BoardAction::DelegatedDiagnostic(DelegatedDiagnostic { request }) => {
            defmt::warn!(
                "diagnostic request delegated before board handler: {}",
                request.channel,
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
