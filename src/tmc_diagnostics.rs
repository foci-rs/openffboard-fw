//! OpenFFBoard-local active TMC diagnostics.
//!
//! These flows remain board-local during the first shared runtime extraction
//! because they own large reply payloads, board delays, and diagnostic-specific
//! restore paths.

use crate::app::Mono;
use foci_firmware::tmc::{TmcRequest, TmcRequestKind};
use foci_firmware::tmc_diagnostics::{
    CURRENT_STEP_STATUS_NOT_READY, CURRENT_STEP_STATUS_SPI, CurrentStepReport,
    CurrentTorqueSampleReport, DiagnosticDelay, DiagnosticOutcome, DiagnosticReadiness,
    DiagnosticReport, VoltageStepReport, current_step_encoder_delta, empty_voltage_step_report,
    pack_voltage_ext, run_current_step_request, run_current_torque_sample_request,
    run_position_torque_offset_sample_request,
};
use foci_tmc4671::spi::Tmc4671Spi;
use rtic_monotonics::{Monotonic, fugit::ExtU64};

use crate::app_types::TmcSpiDevice;

const CURRENT_STEP_POSITION_MODE: u32 = 0x8000_0003;
const CURRENT_STEP_OPEN_LOOP_VOLTAGE_MODE: u32 = 0x0000_0008;
const CURRENT_STEP_SETTLE_MS: u64 = 20;

struct MonoDiagnosticDelay;

impl DiagnosticDelay for MonoDiagnosticDelay {
    async fn delay_ms(&mut self, ms: u64) {
        Mono::delay(ms.millis()).await;
    }
}

/// Runtime state borrowed by board-local active diagnostic handlers.
pub(crate) struct DiagnosticContext<'a> {
    pub(crate) tmc: &'a mut Tmc4671Spi<TmcSpiDevice>,
    pub(crate) runtime: &'a mut foci_firmware::tmc_control::TmcControlRuntime,
    pub(crate) homing_active: bool,
    pub(crate) stepper_active: bool,
    pub(crate) saved_voltage_limit: u32,
}

/// Result of routing a TMC request through the diagnostic dispatcher.
pub(crate) enum DiagnosticDispatch {
    Handled,
    NotDiagnostic(TmcRequest),
}

fn split_signed_halves(value: u32) -> (i16, i16) {
    (
        foci_firmware::tmc::decode_signed_upper_half(value),
        foci_firmware::tmc::decode_signed_lower_half(value),
    )
}

fn current_step_reply(report: CurrentStepReport) {
    ::ankyra::klipper_reply_from!(
        &mut crate::_ankyra_config::Sender,
        foci_firmware::ankyra::replies::FociCurrentStepResult,
        oid: u8 = report.oid,
        status: u8 = report.status,
        target: i16 = report.torque_target,
        duration_ms: u16 = report.duration_ms,
        voltage_limit: u16 = report.voltage_limit,
        torque_before: i16 = report.torque_before,
        torque_during: i16 = report.torque_during,
        torque_after: i16 = report.torque_after,
        flux_during: i16 = report.flux_during,
        iq_during: i16 = report.iq_during,
        id_during: i16 = report.id_during,
        uq_limited: i16 = report.uq_limited,
        ud_limited: i16 = report.ud_limited,
        encoder_before: u32 = report.encoder_before,
        encoder_after: u32 = report.encoder_after,
        encoder_delta: i32 = report.encoder_delta,
        adc_vm_raw: u16 = report.adc_vm_raw,
    );
}

fn current_vector_step_reply(report: CurrentStepReport) {
    ::ankyra::klipper_reply_from!(
        &mut crate::_ankyra_config::Sender,
        foci_firmware::ankyra::replies::FociCurrentVectorStepResult,
        oid: u8 = report.oid,
        status: u8 = report.status,
        torque_target: i16 = report.torque_target,
        flux_target: i16 = report.flux_target,
        duration_ms: u16 = report.duration_ms,
        voltage_limit: u16 = report.voltage_limit,
        torque_before: i16 = report.torque_before,
        torque_during: i16 = report.torque_during,
        torque_after: i16 = report.torque_after,
        flux_during: i16 = report.flux_during,
        iq_during: i16 = report.iq_during,
        id_during: i16 = report.id_during,
        uq_limited: i16 = report.uq_limited,
        ud_limited: i16 = report.ud_limited,
        encoder_before: u32 = report.encoder_before,
        encoder_after: u32 = report.encoder_after,
        encoder_delta: i32 = report.encoder_delta,
        adc_vm_raw: u16 = report.adc_vm_raw,
    );
}

/// Send the appropriate reply for a current step test result.
///
/// Sends a vector reply if `vector_reply` is true, otherwise sends a scalar reply.
pub(crate) fn send_current_step_reply(vector_reply: bool, report: CurrentStepReport) {
    if vector_reply {
        current_vector_step_reply(report);
    } else {
        current_step_reply(report);
    }
}

/// Send the summary reply for a current torque sample test result.
pub(crate) fn current_torque_sample_reply(report: CurrentTorqueSampleReport) {
    ::ankyra::klipper_reply_from!(
        &mut crate::_ankyra_config::Sender,
        foci_firmware::ankyra::replies::FociCurrentTorqueSampleResult,
        oid: u8 = report.oid,
        status: u8 = report.status,
        target: i16 = report.target,
        flux_target: i16 = report.flux_target,
        sample_delay_ms: u16 = report.sample_delay_ms,
        voltage_limit: u16 = report.voltage_limit,
        torque_before: i16 = report.torque_before,
        torque_sample: i16 = report.torque_sample,
        torque_after: i16 = report.torque_after,
        flux_sample: i16 = report.flux_sample,
        iq_sample: i16 = report.iq_sample,
        id_sample: i16 = report.id_sample,
        uq_limited: i16 = report.uq_limited,
        ud_limited: i16 = report.ud_limited,
        encoder_before: u32 = report.encoder_before,
        encoder_sample: u32 = report.encoder_sample,
        encoder_after: u32 = report.encoder_after,
        encoder_delta_sample: i32 = report.encoder_delta_sample,
        encoder_delta_after: i32 = report.encoder_delta_after,
        adc_vm_raw: u16 = report.adc_vm_raw,
        pidin_target_torque: i32 = report.pidin_target_torque,
        pidin_target_flux: i32 = report.pidin_target_flux,
        pidout_target_torque: i32 = report.pidout_target_torque,
        pidout_target_flux: i32 = report.pidout_target_flux,
        pid_torque_target_monitor: i16 = report.pid_torque_target_monitor,
        status_flags: u32 = report.status_flags,
    );
}

/// Send the detail reply for a current torque sample test result.
pub(crate) fn current_torque_sample_detail_reply(report: &CurrentTorqueSampleReport) {
    ::ankyra::klipper_reply_from!(
        &mut crate::_ankyra_config::Sender,
        foci_firmware::ankyra::replies::FociCurrentTorqueSampleDetailResult,
        oid: u8 = report.oid,
        target: i16 = report.target,
        flux_target: i16 = report.flux_target,
        sample_delay_ms: u16 = report.sample_delay_ms,
        voltage_limit: u16 = report.voltage_limit,
        torque_error: i32 = report.torque_error,
        flux_error: i32 = report.flux_error,
        torque_error_sum: i32 = report.torque_error_sum,
        flux_error_sum: i32 = report.flux_error_sum,
        uq_prelimit: i16 = report.uq_prelimit,
        ud_prelimit: i16 = report.ud_prelimit,
        ff_velocity: i32 = report.ff_velocity,
        ff_torque: i16 = report.ff_torque,
    );
}

fn emit_current_torque_sample_report(report: CurrentTorqueSampleReport, emit_detail: bool) {
    if emit_detail {
        current_torque_sample_detail_reply(&report);
    }
    current_torque_sample_reply(report);
}

/// Send the reply for a voltage step test result.
pub(crate) fn voltage_step_reply(report: VoltageStepReport) {
    ::ankyra::klipper_reply_from!(
        &mut crate::_ankyra_config::Sender,
        foci_firmware::ankyra::replies::FociVoltageStepResult,
        oid: u8 = report.oid,
        status: u8 = report.status,
        uq_ext: i16 = report.uq_ext,
        ud_ext: i16 = report.ud_ext,
        sample_delay_ms: u16 = report.sample_delay_ms,
        torque_before: i16 = report.torque_before,
        torque_sample: i16 = report.torque_sample,
        torque_after: i16 = report.torque_after,
        flux_sample: i16 = report.flux_sample,
        iq_sample: i16 = report.iq_sample,
        id_sample: i16 = report.id_sample,
        uq_limited: i16 = report.uq_limited,
        ud_limited: i16 = report.ud_limited,
        uux_sample: i16 = report.uux_sample,
        uwy_sample: i16 = report.uwy_sample,
        pwm_ux_sample: i16 = report.pwm_ux_sample,
        pwm_wy_sample: i16 = report.pwm_wy_sample,
        pwm_sv_chop: u32 = report.pwm_sv_chop,
        pwm_bbm: u32 = report.pwm_bbm,
        pwm_maxcnt: u32 = report.pwm_maxcnt,
        phi_e_sample: i16 = report.phi_e_sample,
        phi_m_sample: i16 = report.phi_m_sample,
        encoder_before: u32 = report.encoder_before,
        encoder_sample: u32 = report.encoder_sample,
        encoder_after: u32 = report.encoder_after,
        encoder_delta_sample: i32 = report.encoder_delta_sample,
        encoder_delta_after: i32 = report.encoder_delta_after,
        adc_vm_raw: u16 = report.adc_vm_raw,
        status_flags: u32 = report.status_flags,
    );
}

/// Handle board-local active diagnostic requests.
pub(crate) async fn handle_diagnostic_request(
    req: TmcRequest,
    ctx: DiagnosticContext<'_>,
) -> DiagnosticDispatch {
    let tmc = ctx.tmc;
    let runtime = ctx.runtime;

    match req {
        diagnostic_req @ TmcRequest {
            kind:
                TmcRequestKind::CurrentStepTest { .. } | TmcRequestKind::CurrentVectorStepTest { .. },
            ..
        } => {
            let mut delay = MonoDiagnosticDelay;
            let outcome = run_current_step_request(
                diagnostic_req,
                foci_firmware::tmc_diagnostics::DiagnosticContext {
                    tmc,
                    runtime,
                    delay: &mut delay,
                    readiness: DiagnosticReadiness {
                        homing_active: ctx.homing_active,
                        stepper_active: ctx.stepper_active,
                        saved_voltage_limit: ctx.saved_voltage_limit,
                    },
                },
            )
            .await;
            match outcome {
                DiagnosticOutcome::Handled(DiagnosticReport::CurrentStep {
                    vector_reply,
                    report,
                }) => send_current_step_reply(vector_reply, report),
                DiagnosticOutcome::Handled(_) => {
                    defmt::error!("current-step runner returned unexpected diagnostic report");
                }
                DiagnosticOutcome::NotDiagnostic(_req) => {
                    defmt::error!("current-step runner returned non-diagnostic request");
                }
            }
        }
        diagnostic_req @ TmcRequest {
            kind: TmcRequestKind::CurrentTorqueSampleTest { .. },
            ..
        } => {
            let mut delay = MonoDiagnosticDelay;
            let outcome = run_current_torque_sample_request(
                diagnostic_req,
                foci_firmware::tmc_diagnostics::DiagnosticContext {
                    tmc,
                    runtime,
                    delay: &mut delay,
                    readiness: DiagnosticReadiness {
                        homing_active: ctx.homing_active,
                        stepper_active: ctx.stepper_active,
                        saved_voltage_limit: ctx.saved_voltage_limit,
                    },
                },
            )
            .await;
            match outcome {
                DiagnosticOutcome::Handled(DiagnosticReport::CurrentTorqueSample {
                    report,
                    emit_detail,
                }) => emit_current_torque_sample_report(report, emit_detail),
                DiagnosticOutcome::Handled(_) => {
                    defmt::error!("torque-sample runner returned unexpected diagnostic report");
                }
                DiagnosticOutcome::NotDiagnostic(_req) => {
                    defmt::error!("torque-sample runner returned non-diagnostic request");
                }
            }
        }
        diagnostic_req @ TmcRequest {
            kind: TmcRequestKind::PositionTorqueOffsetSampleTest { .. },
            ..
        } => {
            let mut delay = MonoDiagnosticDelay;
            let outcome = run_position_torque_offset_sample_request(
                diagnostic_req,
                foci_firmware::tmc_diagnostics::DiagnosticContext {
                    tmc,
                    runtime,
                    delay: &mut delay,
                    readiness: DiagnosticReadiness {
                        homing_active: ctx.homing_active,
                        stepper_active: ctx.stepper_active,
                        saved_voltage_limit: ctx.saved_voltage_limit,
                    },
                },
            )
            .await;
            match outcome {
                DiagnosticOutcome::Handled(DiagnosticReport::CurrentTorqueSample {
                    report,
                    emit_detail,
                }) => emit_current_torque_sample_report(report, emit_detail),
                DiagnosticOutcome::Handled(_) => {
                    defmt::error!("position-offset runner returned unexpected diagnostic report");
                }
                DiagnosticOutcome::NotDiagnostic(_req) => {
                    defmt::error!("position-offset runner returned non-diagnostic request");
                }
            }
        }
        TmcRequest {
            kind:
                TmcRequestKind::VoltageStepTest {
                    oid,
                    uq_ext,
                    ud_ext,
                    sample_delay_ms,
                },
            ..
        } => {
            let homing_active = ctx.homing_active;
            let stepper_active = ctx.stepper_active;
            if !runtime.is_motor_enabled()
                || !runtime.is_calibrated()
                || homing_active
                || stepper_active
            {
                voltage_step_reply(empty_voltage_step_report(
                    oid,
                    CURRENT_STEP_STATUS_NOT_READY,
                    uq_ext,
                    ud_ext,
                    sample_delay_ms,
                ));
                return DiagnosticDispatch::Handled;
            }

            let saved_mode = match tmc
                .read_register(foci_tmc4671::registers::MODE_RAMP_MODE_MOTION)
                .await
            {
                Ok(value) => value,
                Err(_) => {
                    voltage_step_reply(empty_voltage_step_report(
                        oid,
                        CURRENT_STEP_STATUS_SPI,
                        uq_ext,
                        ud_ext,
                        sample_delay_ms,
                    ));
                    return DiagnosticDispatch::Handled;
                }
            };
            if saved_mode != CURRENT_STEP_POSITION_MODE {
                voltage_step_reply(empty_voltage_step_report(
                    oid,
                    CURRENT_STEP_STATUS_NOT_READY,
                    uq_ext,
                    ud_ext,
                    sample_delay_ms,
                ));
                return DiagnosticDispatch::Handled;
            }

            let mut report = empty_voltage_step_report(oid, 0, uq_ext, ud_ext, sample_delay_ms);

            if !runtime.restore_phase_advance_now(tmc).await {
                defmt::warn!("phase advance: failed to restore ABN offset before voltage step");
                voltage_step_reply(empty_voltage_step_report(
                    oid,
                    CURRENT_STEP_STATUS_SPI,
                    uq_ext,
                    ud_ext,
                    sample_delay_ms,
                ));
                return DiagnosticDispatch::Handled;
            }

            let result: Result<(), ()> = async {
                use foci_tmc4671::registers::*;

                report.encoder_before =
                    tmc.read_register(ABN_DECODER_COUNT).await.map_err(|_| ())?;
                let encoder_counts_per_rev =
                    tmc.read_register(ABN_DECODER_PPR).await.map_err(|_| ())?;

                let before = tmc
                    .read_register(PID_TORQUE_FLUX_ACTUAL)
                    .await
                    .map_err(|_| ())?;
                report.torque_before = foci_firmware::tmc::decode_signed_upper_half(before);

                tmc.write_register(STATUS_FLAGS, 0).await.map_err(|_| ())?;
                tmc.write_register(UQ_UD_EXT, pack_voltage_ext(uq_ext, ud_ext))
                    .await
                    .map_err(|_| ())?;
                tmc.write_register(MODE_RAMP_MODE_MOTION, CURRENT_STEP_OPEN_LOOP_VOLTAGE_MODE)
                    .await
                    .map_err(|_| ())?;

                Mono::delay((sample_delay_ms as u64).millis()).await;

                report.encoder_sample =
                    tmc.read_register(ABN_DECODER_COUNT).await.map_err(|_| ())?;
                report.encoder_delta_sample = current_step_encoder_delta(
                    report.encoder_before,
                    report.encoder_sample,
                    encoder_counts_per_rev,
                );

                let actual = tmc
                    .read_register(PID_TORQUE_FLUX_ACTUAL)
                    .await
                    .map_err(|_| ())?;
                let (torque, flux) = split_signed_halves(actual);
                report.torque_sample = torque;
                report.flux_sample = flux;

                let iq_id = tmc
                    .read_sub_register(INTERIM_FOC_IQ_ID)
                    .await
                    .map_err(|_| ())?;
                let (iq, id) = split_signed_halves(iq_id);
                report.iq_sample = iq;
                report.id_sample = id;

                let uq_ud = tmc
                    .read_sub_register(INTERIM_FOC_UQ_UD_LIMITED)
                    .await
                    .map_err(|_| ())?;
                let (uq, ud) = split_signed_halves(uq_ud);
                report.uq_limited = uq;
                report.ud_limited = ud;

                // TMC4671-LA Rev 1, 02/2024, INTERIM_DATA selector 15 packs
                // FOC_UWY in bits 31:16 and FOC_UUX in bits 15:0.
                let uwy_uux = tmc
                    .read_sub_register(INTERIM_FOC_UWY_UUX)
                    .await
                    .map_err(|_| ())?;
                let (uwy, uux) = split_signed_halves(uwy_uux);
                report.uwy_sample = uwy;
                report.uux_sample = uux;

                // INTERIM_DATA selector 17 packs PWM_WY in bits 31:16 and
                // PWM_UX in bits 15:0.
                let pwm_wy_ux = tmc
                    .read_sub_register(INTERIM_PWM_WY_UX)
                    .await
                    .map_err(|_| ())?;
                let (pwm_wy, pwm_ux) = split_signed_halves(pwm_wy_ux);
                report.pwm_wy_sample = pwm_wy;
                report.pwm_ux_sample = pwm_ux;

                report.pwm_sv_chop = tmc.read_register(PWM_SV_CHOP).await.map_err(|_| ())?;
                report.pwm_bbm = tmc.read_register(PWM_BBM_H_BBM_L).await.map_err(|_| ())?;
                report.pwm_maxcnt = tmc.read_register(PWM_MAXCNT).await.map_err(|_| ())?;

                // ABN_DECODER_PHI_E_PHI_M packs PHI_E in bits 31:16 and
                // PHI_M in bits 15:0.
                let phi_e_phi_m = tmc
                    .read_register(ABN_DECODER_PHI_E_PHI_M)
                    .await
                    .map_err(|_| ())?;
                let (phi_e, phi_m) = split_signed_halves(phi_e_phi_m);
                report.phi_e_sample = phi_e;
                report.phi_m_sample = phi_m;

                // TMC4671-LA Rev 1, 02/2024, ADC_RAW_DATA 0x02 selected by
                // ADC_RAW_ADDR 0x03: selector 1 packs ADC_AGPI_A_RAW in
                // bits 31:16 and ADC_VM_RAW in bits 15:0.
                report.adc_vm_raw = tmc
                    .read_sub_register(ADC_AGPI_A_RAW_ADC_VM_RAW)
                    .await
                    .map_err(|_| ())? as u16;
                report.status_flags = tmc.read_register(STATUS_FLAGS).await.map_err(|_| ())?;

                tmc.write_register(MODE_RAMP_MODE_MOTION, 0)
                    .await
                    .map_err(|_| ())?;
                tmc.write_register(UQ_UD_EXT, 0).await.map_err(|_| ())?;
                tmc.write_register(MODE_RAMP_MODE_MOTION, saved_mode)
                    .await
                    .map_err(|_| ())?;
                Mono::delay(CURRENT_STEP_SETTLE_MS.millis()).await;

                report.encoder_after =
                    tmc.read_register(ABN_DECODER_COUNT).await.map_err(|_| ())?;
                report.encoder_delta_after = current_step_encoder_delta(
                    report.encoder_before,
                    report.encoder_after,
                    encoder_counts_per_rev,
                );

                let after = tmc
                    .read_register(PID_TORQUE_FLUX_ACTUAL)
                    .await
                    .map_err(|_| ())?;
                report.torque_after = foci_firmware::tmc::decode_signed_upper_half(after);
                Ok(())
            }
            .await;

            if result.is_err() {
                report.status = CURRENT_STEP_STATUS_SPI;
                let _ = tmc
                    .write_register(foci_tmc4671::registers::MODE_RAMP_MODE_MOTION, 0)
                    .await;
                let _ = tmc
                    .write_register(foci_tmc4671::registers::UQ_UD_EXT, 0)
                    .await;
                let _ = tmc
                    .write_register(foci_tmc4671::registers::MODE_RAMP_MODE_MOTION, saved_mode)
                    .await;
            }

            voltage_step_reply(report);
        }
        req => return DiagnosticDispatch::NotDiagnostic(req),
    }

    DiagnosticDispatch::Handled
}
