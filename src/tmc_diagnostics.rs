//! OpenFFBoard active diagnostic reply glue.
//!
//! The diagnostic state machines live in `foci-firmware`; this module keeps
//! RTIC delay wiring and OpenFFBoard reply emission local.

use crate::app::Mono;
use foci_firmware::tmc::TmcRequest;
use foci_firmware::tmc_diagnostics::{
    CurrentStepReport, CurrentTorqueSampleReport, DiagnosticDelay, DiagnosticOutcome,
    DiagnosticReadiness, DiagnosticReport, VoltageStepReport,
    handle_diagnostic_request as handle_shared_diagnostic_request,
};
use foci_tmc4671::spi::Tmc4671Spi;
use rtic_monotonics::{Monotonic, fugit::ExtU64};

use crate::app_types::TmcSpiDevice;

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

fn emit_diagnostic_report(report: DiagnosticReport) {
    match report {
        DiagnosticReport::CurrentStep {
            vector_reply,
            report,
        } => send_current_step_reply(vector_reply, report),
        DiagnosticReport::CurrentTorqueSample {
            report,
            emit_detail,
        } => emit_current_torque_sample_report(report, emit_detail),
        DiagnosticReport::VoltageStep(report) => voltage_step_reply(report),
    }
}

/// Handle board-local active diagnostic requests.
pub(crate) async fn handle_diagnostic_request(
    req: TmcRequest,
    ctx: DiagnosticContext<'_>,
) -> DiagnosticDispatch {
    let mut delay = MonoDiagnosticDelay;
    let outcome = handle_shared_diagnostic_request(
        req,
        foci_firmware::tmc_diagnostics::DiagnosticContext {
            tmc: ctx.tmc,
            runtime: ctx.runtime,
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
        DiagnosticOutcome::Handled(report) => {
            emit_diagnostic_report(report);
            DiagnosticDispatch::Handled
        }
        DiagnosticOutcome::NotDiagnostic(req) => DiagnosticDispatch::NotDiagnostic(req),
    }
}
