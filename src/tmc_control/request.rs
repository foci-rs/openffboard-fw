//! OpenFFBoard TMC control request handling.
//!
//! This module owns the board-local request policy that needs SPI, the enable
//! pin, and the OpenFFBoard single-channel runtime state. The RTIC task in
//! `main.rs` keeps request waiting, shutdown locking, and control-loop timing.

use core::sync::atomic::Ordering;

use embassy_stm32::gpio::Output;
use foci_firmware::pid::{InnerLoopGains, OuterLoopGains};
use foci_firmware::state::trsync::HOMING_STATE;
use foci_firmware::tmc::{TmcRequest, TmcRequestKind};
use foci_firmware::tmc_control::helpers::{apply_live_inner_gains, apply_live_outer_gains};
use foci_firmware::tmc_control::{ShutdownReason, TmcControlRequestOutcome, TmcControlRequestTail};
use foci_tmc4671::spi::Tmc4671Spi;

use crate::app_types::TmcSpiDevice;
#[cfg(feature = "dev")]
use crate::helpers::dev::{do_test_spin, tmc_enter_openloop};
use crate::helpers::stepper::apply_stepper_post_action;
use crate::tmc_control::board::{openffboard_backend, stepper_state, tmc_state};
use crate::tmc_dispatch::handle_tmc_request;

fn next_commission_trace_seq(commission_trace_engine_seq: &mut u32) -> u32 {
    let seq = *commission_trace_engine_seq;
    *commission_trace_engine_seq = (*commission_trace_engine_seq).wrapping_add(1);
    seq
}

pub(crate) async fn handle_tmc_control_request(
    req: TmcRequest,
    tmc: &mut Tmc4671Spi<TmcSpiDevice>,
    enable_pin: &mut Output<'static>,
    runtime: &mut foci_firmware::tmc_control::TmcControlRuntime,
    commission_trace_engine_seq: &mut u32,
) -> TmcControlRequestOutcome {
    match req {
        TmcRequest { channel, .. } if channel != 0 => {
            return TmcControlRequestOutcome::shutdown(
                ShutdownReason::InvalidTmcChannel,
                TmcControlRequestTail::SkipTickTail,
            );
        }
        TmcRequest {
            kind: TmcRequestKind::SetEnable(enabled),
            ..
        } => {
            if enabled {
                if !runtime.is_calibrated() {
                    // Auto-calibrate: Klipper may send enable before the Python module's
                    // foci_calibrate arrives due to queue_digital_out timing.
                    if !foci_firmware::tmc::auto_calibrate_on_enable_allowed(0) {
                        defmt::warn!("Auto-calibrate rejected: host gate closed");
                        return TmcControlRequestOutcome::skip_tick_tail();
                    }
                    if tmc_state()
                        .commission_in_flight
                        .swap(true, Ordering::Relaxed)
                    {
                        defmt::warn!("Auto-calibrate skipped: commission in flight");
                        return TmcControlRequestOutcome::skip_tick_tail();
                    }
                    // TODO: hardcoded OID 0 — correct for single-motor OpenFFBoard but
                    // needs OID routing for multi-motor boards.
                    defmt::info!("Auto-calibrating on enable");
                    runtime.set_commission_engine(
                        foci_firmware::commission::CommissioningEngine::new_calibrate_for_tmc_with_gains_and_trace_seq(
                            0,
                            tmc_state(),
                            runtime.stored_gains(),
                            next_commission_trace_seq(commission_trace_engine_seq),
                        ),
                    );
                    // Don't mark the motor armed yet -- the CalibrateDone handler does
                    // that after encoder alignment + closed-loop entry.
                    return TmcControlRequestOutcome::skip_tick_tail();
                }
                // Existing precondition: this arm runs only after the runtime has
                // recorded calibration; otherwise the auto-calibrate branch above
                // returns before reaching this point.
                if runtime.is_motor_enabled() {
                    defmt::info!("Motor enable sync (already armed)");
                    return TmcControlRequestOutcome::skip_tick_tail();
                }
                let actions = runtime.prepare_enable_request(tmc_state());
                for action in actions {
                    if let foci_firmware::tmc_control::BoardAction::Stepper(
                        foci_firmware::tmc_control::StepperAction::ArmCheck { post_action },
                    ) = action
                    {
                        if !runtime.restore_phase_advance_now(tmc).await {
                            defmt::warn!("phase advance: failed to restore ABN offset before arm");
                            continue;
                        }
                        {
                            let mut backend = openffboard_backend(tmc, enable_pin);
                            let mut state = runtime.dispatch_state(tmc_state(), stepper_state());
                            if let Err(error) = foci_firmware::tmc_ops::arm_motor_electrical(
                                &mut backend,
                                &mut state,
                            )
                            .await
                            {
                                defmt::error!(
                                    "Motor enable rejected by sentinel verify: {}",
                                    error
                                );
                                foci_firmware::tmc_ops::disarm_motor(&mut backend, &mut state)
                                    .await;
                                continue;
                            }
                            apply_stepper_post_action(post_action, &mut state);
                        }

                        defmt::info!("Motor enabled (calibrated, step/dir armed)");
                        #[cfg(feature = "trace")]
                        crate::usb_trace::request_event(
                            foci_firmware::trace::events::EVENT_LEVEL_INFO,
                            foci_firmware::trace::events::SUBSYSTEM_TMC_CONTROL,
                            crate::usb_trace::EVENT_MOTOR_ARM,
                            [0; 4],
                            0,
                        );
                    }
                }
            } else {
                // Disable: stop stepping first, then kill motor electrically. This runtime
                // call emits only immediate stepper actions; shutdown reporting remains in
                // the tmc_control caller where app_state can be locked synchronously.
                crate::tmc_control_glue::apply_actions(runtime.prepare_disable_request(), |_| {});
                let phase_advance_restored = runtime.restore_phase_advance_now(tmc).await;
                if !phase_advance_restored {
                    defmt::warn!("phase advance: failed to restore ABN offset before disarm");
                }
                {
                    let mut backend = openffboard_backend(tmc, enable_pin);
                    let mut state = runtime.dispatch_state(tmc_state(), stepper_state());
                    foci_firmware::tmc_ops::disarm_motor(&mut backend, &mut state).await;
                }
                if !phase_advance_restored {
                    return TmcControlRequestOutcome::skip_tick_tail();
                }

                defmt::info!("TMC4671 motor disabled");
                #[cfg(feature = "trace")]
                crate::usb_trace::request_event(
                    foci_firmware::trace::events::EVENT_LEVEL_INFO,
                    foci_firmware::trace::events::SUBSYSTEM_TMC_CONTROL,
                    crate::usb_trace::EVENT_MOTOR_DISARM,
                    [0; 4],
                    0,
                );
            }
        }
        TmcRequest {
            kind: TmcRequestKind::Calibrate { oid },
            ..
        } => {
            if runtime.is_motor_enabled() {
                defmt::warn!("foci_calibrate rejected: motor already enabled");
                tmc_state()
                    .commission_in_flight
                    .store(false, Ordering::Relaxed);
                ::ankyra::klipper_reply_from!(
                    &mut crate::_ankyra_config::Sender,
                    foci_firmware::ankyra::replies::FociCalibrateResult,
                    oid: u8 = oid,
                    status: u8 = foci_firmware::calibration::CalibrationError::AlreadyEnabled.status_code(),
                    adc_i0: u16 = 0,
                    adc_i1: u16 = 0,
                    encoder_count: u32 = 0,
                );
                return TmcControlRequestOutcome::skip_tick_tail();
            }
            // Engine tick will drive ADC cal; CalibrateDone handler performs encoder
            // alignment and closed-loop entry.
            runtime.clear_calibrated();
            runtime.set_commission_engine(
                foci_firmware::commission::CommissioningEngine::new_calibrate_for_tmc_with_gains_and_trace_seq(
                    oid,
                    tmc_state(),
                    runtime.stored_gains(),
                    next_commission_trace_seq(commission_trace_engine_seq),
                ),
            );
        }
        #[cfg(feature = "dev")]
        TmcRequest {
            kind: TmcRequestKind::Init,
            ..
        } => {
            defmt::info!("TMC4671 init: starting open-loop stepper config");
            tmc_enter_openloop(tmc, enable_pin).await;
        }
        #[cfg(feature = "dev")]
        TmcRequest {
            kind: TmcRequestKind::TestSpin { revolutions },
            ..
        } => {
            do_test_spin(tmc, revolutions).await;
        }
        TmcRequest {
            kind:
                TmcRequestKind::SetPidGains {
                    flux_p,
                    flux_i,
                    torque_p,
                    torque_i,
                },
            ..
        } => {
            let gains = InnerLoopGains {
                flux_p,
                flux_i,
                torque_p,
                torque_i,
            };
            let write_status = apply_live_inner_gains(runtime, tmc, gains).await;
            if !write_status.all_ok() {
                defmt::warn!("PID gains live write failed");
            }
            defmt::info!(
                "PID gains received: flux P={} I={}, torque P={} I={}",
                flux_p,
                flux_i,
                torque_p,
                torque_i,
            );
        }
        TmcRequest {
            kind:
                TmcRequestKind::SetPositionGains {
                    position_p,
                    position_i,
                    velocity_p,
                    velocity_i,
                },
            ..
        } => {
            let gains = OuterLoopGains {
                position_p,
                position_i,
                velocity_p,
                velocity_i,
            };
            let write_status = apply_live_outer_gains(tmc_state(), tmc, gains).await;
            if !write_status.all_ok() {
                defmt::warn!("Position gains live write failed");
            }
            defmt::info!(
                "Position gains received: pos P={} I={}, vel P={} I={}",
                position_p,
                position_i,
                velocity_p,
                velocity_i,
            );
        }
        TmcRequest {
            kind: TmcRequestKind::SetVelocityFeedforward { enable, multiplier },
            ..
        } => {
            defmt::info!(
                "Velocity feedforward: {} multiplier={} (via queue)",
                enable,
                multiplier
            );
            // Feedforward flag is already set by the atomic in the command handler; this
            // arm exists for exhaustiveness.
        }
        TmcRequest {
            kind:
                TmcRequestKind::SetVelocityTransientFeedforward {
                    enable,
                    lead_time_us,
                    gain_permille,
                    max_offset,
                    rate_hz,
                },
            ..
        } => {
            defmt::info!(
                "Velocity transient feedforward: {} lead_time_us={} \
                gain_permille={} max_offset={} rate_hz={} (via queue)",
                enable,
                lead_time_us,
                gain_permille,
                max_offset,
                rate_hz
            );
            // Feedforward atomics are set by the command handler; this arm exists for
            // exhaustiveness.
        }
        TmcRequest {
            kind:
                TmcRequestKind::SetAccelFeedforward {
                    enable,
                    accel_gain_permille,
                    decel_gain_permille,
                },
            ..
        } => {
            defmt::info!(
                "Acceleration feedforward: {} accel_gain_permille={} \
                decel_gain_permille={} (via queue)",
                enable,
                accel_gain_permille,
                decel_gain_permille
            );
            // Feedforward flag is already set by the atomic in the command handler; this
            // arm exists for exhaustiveness.
        }
        TmcRequest {
            kind:
                TmcRequestKind::SetDecouplingFeedforward {
                    enable,
                    r_int,
                    l_int,
                    pole_pairs,
                    position_units_per_rev,
                    f_pwm_hz,
                    max_offset,
                },
            ..
        } => {
            defmt::info!(
                "Decoupling feedforward: {} r_int={} l_int={} pole_pairs={} \
                position_units_per_rev={} f_pwm_hz={} max_offset={} (via queue)",
                enable,
                r_int,
                l_int,
                pole_pairs,
                position_units_per_rev,
                f_pwm_hz,
                max_offset
            );
            // Feedforward atomics are set by the command handler; this arm exists for
            // exhaustiveness.
        }
        TmcRequest {
            kind:
                TmcRequestKind::SetPositionLead {
                    enable,
                    gain_permille,
                    max_counts,
                },
            ..
        } => {
            defmt::info!(
                "Position lead: {} gain_permille={} max_counts={} (via queue)",
                enable,
                gain_permille,
                max_counts
            );
            // Position-lead atomics are set by the command handler; this arm exists for
            // exhaustiveness.
        }
        TmcRequest {
            kind:
                TmcRequestKind::SetPhaseAdvance {
                    enable,
                    gain_ppm,
                    max_counts,
                    deadband,
                },
            ..
        } => {
            defmt::info!(
                "Phase advance: {} gain_ppm={} max_counts={} deadband={} \
                (via queue)",
                enable,
                gain_ppm,
                max_counts,
                deadband
            );
            // Phase-advance atomics are set by the command handler; this arm exists for
            // exhaustiveness.
        }
        TmcRequest {
            kind: TmcRequestKind::Commission { oid, profile },
            ..
        } => {
            if runtime.is_motor_enabled() {
                tmc_state()
                    .commission_in_flight
                    .store(false, Ordering::Relaxed);
                ::ankyra::klipper_reply_from!(
                    &mut crate::_ankyra_config::Sender,
                    foci_firmware::ankyra::replies::FociCommissionPhase,
                    oid: u8 = oid,
                    phase: u8 = 0,
                    status: u8 =
                        foci_firmware::autotune::CommissionError::MotorEnabled
                            .status_code(),
                );
                return TmcControlRequestOutcome::skip_tick_tail();
            }
            let current_limit = tmc_state().current_limit_reg.load(Ordering::Relaxed);
            if current_limit == 0 {
                tmc_state()
                    .commission_in_flight
                    .store(false, Ordering::Relaxed);
                ::ankyra::klipper_reply_from!(
                    &mut crate::_ankyra_config::Sender,
                    foci_firmware::ankyra::replies::FociCommissionPhase,
                    oid: u8 = oid,
                    phase: u8 = 0,
                    status: u8 =
                        foci_firmware::autotune::CommissionError::NoCurrent.status_code(),
                );
                return TmcControlRequestOutcome::skip_tick_tail();
            }
            runtime.clear_calibrated();
            runtime.set_commission_engine(
                foci_firmware::commission::CommissioningEngine::new_commission_for_tmc_with_trace_seq(
                    oid,
                    tmc_state(),
                    foci_firmware::autotune::CommissionProfile::from_u8(profile),
                    next_commission_trace_seq(commission_trace_engine_seq),
                ),
            );
        }
        TmcRequest {
            kind:
                TmcRequestKind::Tune {
                    oid,
                    profile,
                    mode,
                    inner_lambda_us,
                    theta_e_us,
                    current_ringing_count,
                    current_bandwidth_hz,
                    tau_e_us,
                    tau_e_crosscheck_us,
                    tau_residual_permille,
                    inner_warning_flags,
                },
            ..
        } => {
            // Stage 2 requires the motor to be enabled and holding in closed-loop
            // position mode (from Stage 1 or a prior calibrate). Reject if not ready.
            if !runtime.is_motor_enabled() || !runtime.is_calibrated() {
                tmc_state()
                    .commission_in_flight
                    .store(false, Ordering::Relaxed);
                ::ankyra::klipper_reply_from!(
                    &mut crate::_ankyra_config::Sender,
                    foci_firmware::ankyra::replies::FociCommissionPhase,
                    oid: u8 = oid,
                    phase: u8 = 0,
                    status: u8 =
                        foci_firmware::autotune::CommissionError::EncoderNotAligned
                            .status_code(),
                );
                return TmcControlRequestOutcome::skip_tick_tail();
            }
            let current_limit = tmc_state().current_limit_reg.load(Ordering::Relaxed);
            if current_limit == 0 {
                tmc_state()
                    .commission_in_flight
                    .store(false, Ordering::Relaxed);
                ::ankyra::klipper_reply_from!(
                    &mut crate::_ankyra_config::Sender,
                    foci_firmware::ankyra::replies::FociCommissionPhase,
                    oid: u8 = oid,
                    phase: u8 = 0,
                    status: u8 =
                        foci_firmware::autotune::CommissionError::NoCurrent.status_code(),
                );
                return TmcControlRequestOutcome::skip_tick_tail();
            }
            if !runtime.restore_phase_advance_now(tmc).await {
                defmt::warn!("phase advance: failed to restore ABN offset before tune");
                tmc_state()
                    .commission_in_flight
                    .store(false, Ordering::Relaxed);
                ::ankyra::klipper_reply_from!(
                    &mut crate::_ankyra_config::Sender,
                    foci_firmware::ankyra::replies::FociCommissionPhase,
                    oid: u8 = oid,
                    phase: u8 = 0,
                    status: u8 = foci_firmware::autotune::CommissionError::Spi.status_code(),
                );
                return TmcControlRequestOutcome::skip_tick_tail();
            }
            let entry_snapshot = {
                use core::sync::atomic::Ordering::Relaxed;
                foci_firmware::commission::EntrySnapshot {
                    velocity_p: tmc_state().velocity_p_value.load(Relaxed),
                    velocity_i: tmc_state().velocity_i_value.load(Relaxed),
                    position_p: tmc_state().position_p_value.load(Relaxed),
                    position_i: tmc_state().position_i_value.load(Relaxed),
                    velocity_limit: tmc_state().velocity_limit_cfg.load(Relaxed),
                    voltage_limit: tmc_state().voltage_limit_cfg.load(Relaxed),
                    velocity_filter_hz: tmc_state().velocity_filter_hz.load(Relaxed),
                    torque_filter_hz: tmc_state().torque_filter_hz.load(Relaxed),
                    position_filter_hz: tmc_state().position_filter_hz.load(Relaxed),
                    flux_filter_hz: tmc_state().flux_filter_hz.load(Relaxed),
                }
            };
            let outer_params = foci_firmware::commission::OuterTuneParams {
                profile: foci_firmware::autotune::CommissionProfile::from_u8(profile),
                mode: foci_firmware::autotune::OperatingMode::from_u8(mode),
                inner_lambda_us,
                theta_e_us,
                current_ringing: current_ringing_count > 1,
                current_bandwidth_hz,
                tau_e_us,
                tau_e_crosscheck_us,
                tau_residual_permille,
                inner_warning_flags,
            };
            runtime.set_commission_engine(
                foci_firmware::commission::CommissioningEngine::new_outer_for_tmc_with_trace_seq(
                    oid,
                    tmc_state(),
                    outer_params,
                    entry_snapshot,
                    next_commission_trace_seq(commission_trace_engine_seq),
                ),
            );
        }
        TmcRequest {
            kind: TmcRequestKind::SelfTest { oid },
            ..
        } => {
            if runtime.is_motor_enabled() {
                defmt::warn!("foci_selftest rejected: motor enabled");
                tmc_state()
                    .commission_in_flight
                    .store(false, Ordering::Relaxed);
                ::ankyra::klipper_reply_from!(
                    &mut crate::_ankyra_config::Sender,
                    foci_firmware::ankyra::replies::FociSelftestDone,
                    oid: u8 = oid,
                    status: u8 =
                        foci_firmware::autotune::CommissionError::MotorEnabled
                            .status_code(),
                );
                return TmcControlRequestOutcome::skip_tick_tail();
            }
            // Engine tick drives selftest phases; SelftestDone handler sends the completion
            // signal.
            runtime.set_commission_engine(
                foci_firmware::commission::CommissioningEngine::new_selftest_for_tmc_with_trace_seq(
                    oid,
                    tmc_state(),
                    next_commission_trace_seq(commission_trace_engine_seq),
                ),
            );
        }
        diagnostic_req @ TmcRequest {
            kind:
                TmcRequestKind::CurrentStepTest { .. }
                | TmcRequestKind::CurrentVectorStepTest { .. }
                | TmcRequestKind::CurrentTorqueSampleTest { .. }
                | TmcRequestKind::PositionTorqueOffsetSampleTest { .. }
                | TmcRequestKind::VoltageStepTest { .. },
            ..
        } => {
            let homing_active = HOMING_STATE.load(Ordering::Relaxed) != 0;
            let stepper_active = stepper_state().timer_active_mirror.load(Ordering::Relaxed);
            let saved_voltage_limit = tmc_state().voltage_limit_cfg.load(Ordering::Relaxed);
            match crate::tmc_diagnostics::handle_diagnostic_request(
                diagnostic_req,
                crate::tmc_diagnostics::DiagnosticContext {
                    tmc,
                    runtime: &mut *runtime,
                    homing_active,
                    stepper_active,
                    saved_voltage_limit,
                },
            )
            .await
            {
                crate::tmc_diagnostics::DiagnosticDispatch::Handled => {}
                crate::tmc_diagnostics::DiagnosticDispatch::NotDiagnostic(_req) => {
                    defmt::error!("diagnostic dispatch returned non-diagnostic request");
                }
            }
        }
        req @ TmcRequest {
            kind: TmcRequestKind::SetEncoder { .. },
            ..
        } => {
            let request_failed = handle_tmc_request(req, tmc).await.is_err();
            if request_failed {
                return TmcControlRequestOutcome::shutdown_then_refresh_velocity_scale(
                    ShutdownReason::RequestValidationFailed,
                    TmcControlRequestTail::SkipTickTail,
                );
            }
            return TmcControlRequestOutcome::refresh_velocity_scale(
                TmcControlRequestTail::SkipTickTail,
            );
        }
        other => {
            if handle_tmc_request(other, tmc).await.is_err() {
                return TmcControlRequestOutcome::shutdown(
                    ShutdownReason::RequestValidationFailed,
                    TmcControlRequestTail::SkipTickTail,
                );
            }
        }
    }

    TmcControlRequestOutcome::skip_tick_tail()
}
