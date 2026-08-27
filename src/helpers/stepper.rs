//! Stepper helpers: P1-safe physical stepper halt.
//! Promoted from `mod app` in `main.rs` by Task 8c so handlers and the
//! TIM5 scheduler ISR share a single implementation.

use crate::trigger_domain::stop_physical_stepper_unmasked;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use foci_firmware::commission_dispatch::{
    DispatchState, StepperPostAction, StepperTaskDirective, apply_stepper_post_action_state,
};
use foci_firmware::context::stepper_reconcile::StepperReconcilePending;
use foci_firmware::diagnostic::StepperEventReason;

pub(crate) static STEPPER_RECONCILE_PENDING: StepperReconcilePending<
    { crate::board_config::TMC_CHANNEL_COUNT as usize },
> = StepperReconcilePending::new();
pub(crate) static STEPPER_RECONCILE_WAKE: Signal<CriticalSectionRawMutex, ()> = Signal::new();

pub(crate) fn request_stop_reconciliation(channel: u8) {
    if STEPPER_RECONCILE_PENDING.request_stop(channel).is_err() {
        defmt::error!(
            "request_stop_reconciliation: invalid OpenFFBoard channel {}",
            channel
        );
        return;
    }
    STEPPER_RECONCILE_WAKE.signal(());
}

pub(crate) fn request_arm_reconciliation(channel: u8) {
    if STEPPER_RECONCILE_PENDING.request_arm(channel).is_err() {
        defmt::error!(
            "request_arm_reconciliation: invalid OpenFFBoard channel {}",
            channel
        );
        return;
    }
    STEPPER_RECONCILE_WAKE.signal(());
}

fn tmc_state() -> &'static foci_firmware::state::tmc::TmcChannelState {
    foci_firmware::state::tmc::tmc_channel0()
}

fn stepper_state() -> &'static foci_firmware::state::stepper::StepperChannelState {
    foci_firmware::state::stepper::stepper_channel0()
}

/// Quiesce the physical stepper from a P1 context without entering
/// `TriggerDomain`.
///
/// P1 performs the physical quiesce immediately (queue drain via
/// `stop_physical_stepper_unmasked`) then requests P2 reconciliation to
/// cancel the stepper scheduler event and update the authoritative
/// `timer_active` inside `TriggerDomain`. P1 must never enter
/// `TriggerDomain` per spec.
pub(crate) fn stepper_stop_from_p1(channel: u8) {
    if channel != 0 {
        defmt::error!(
            "stepper_stop_from_p1: invalid OpenFFBoard channel {}",
            channel
        );
        return;
    }

    tmc_state()
        .motor_armed
        .store(false, core::sync::atomic::Ordering::Relaxed);
    stop_physical_stepper_unmasked(channel as usize, StepperEventReason::P1Stop);
    crate::helpers::stepper_diag::record_stepper_event(
        StepperEventReason::P1Stop,
        channel,
        crate::helpers::lifecycle::read_tim5(),
        0,
        0,
        0,
    );
    stepper_state()
        .timer_active_mirror
        .store(false, core::sync::atomic::Ordering::Relaxed);
    request_stop_reconciliation(0);
}

/// Apply a board-owned stepper runtime transition after commissioning dispatch.
///
/// This is the single owner of firmware step-epoch zeroing, channel motor arming,
/// and arm reconciliation requests. Dispatch only decides which transition is
/// needed.
pub(crate) fn apply_stepper_post_action(action: StepperPostAction, state: &mut DispatchState<'_>) {
    match apply_stepper_post_action_state(action, state) {
        StepperTaskDirective::None => {}
        StepperTaskDirective::StopCheck => {
            stepper_stop_from_p1(0);
        }
        StepperTaskDirective::ArmCheck => {
            request_arm_reconciliation(0);
        }
    }
}
