//! OpenFFBoard-local active TMC diagnostics.
//!
//! These flows remain board-local during the first shared runtime extraction
//! because they own large reply payloads, board delays, and diagnostic-specific
//! restore paths.

use crate::app::Mono;
use foci_firmware::tmc::{TmcRequest, TmcRequestKind};
use foci_tmc4671::spi::Tmc4671Spi;
use rtic_monotonics::{Monotonic, fugit::ExtU64};

use crate::app_types::TmcSpiDevice;

const CURRENT_STEP_TORQUE_MODE: u32 = 0x8000_0001;
const CURRENT_STEP_POSITION_MODE: u32 = 0x8000_0003;
const CURRENT_STEP_OPEN_LOOP_VOLTAGE_MODE: u32 = 0x0000_0008;
const CURRENT_STEP_SETTLE_MS: u64 = 20;
const CURRENT_STEP_STATUS_NOT_READY: u8 = 1;
const CURRENT_STEP_STATUS_SPI: u8 = 2;

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

/// Collects the readings captured during a single current step test.
pub(crate) struct CurrentStepReport {
    pub(crate) oid: u8,
    pub(crate) status: u8,
    pub(crate) torque_target: i16,
    pub(crate) flux_target: i16,
    pub(crate) duration_ms: u16,
    pub(crate) voltage_limit: u16,
    pub(crate) torque_before: i16,
    pub(crate) torque_during: i16,
    pub(crate) torque_after: i16,
    pub(crate) flux_during: i16,
    pub(crate) iq_during: i16,
    pub(crate) id_during: i16,
    pub(crate) uq_limited: i16,
    pub(crate) ud_limited: i16,
    pub(crate) encoder_before: u32,
    pub(crate) encoder_after: u32,
    pub(crate) encoder_delta: i32,
    pub(crate) adc_vm_raw: u16,
}

/// Collects the readings captured during a current torque sample test.
pub(crate) struct CurrentTorqueSampleReport {
    pub(crate) oid: u8,
    pub(crate) status: u8,
    pub(crate) target: i16,
    pub(crate) flux_target: i16,
    pub(crate) sample_delay_ms: u16,
    pub(crate) voltage_limit: u16,
    pub(crate) torque_before: i16,
    pub(crate) torque_sample: i16,
    pub(crate) torque_after: i16,
    pub(crate) flux_sample: i16,
    pub(crate) iq_sample: i16,
    pub(crate) id_sample: i16,
    pub(crate) uq_limited: i16,
    pub(crate) ud_limited: i16,
    pub(crate) encoder_before: u32,
    pub(crate) encoder_sample: u32,
    pub(crate) encoder_after: u32,
    pub(crate) encoder_delta_sample: i32,
    pub(crate) encoder_delta_after: i32,
    pub(crate) adc_vm_raw: u16,
    pub(crate) pidin_target_torque: i32,
    pub(crate) pidin_target_flux: i32,
    pub(crate) pidout_target_torque: i32,
    pub(crate) pidout_target_flux: i32,
    pub(crate) pid_torque_target_monitor: i16,
    pub(crate) torque_error: i32,
    pub(crate) flux_error: i32,
    pub(crate) torque_error_sum: i32,
    pub(crate) flux_error_sum: i32,
    pub(crate) uq_prelimit: i16,
    pub(crate) ud_prelimit: i16,
    pub(crate) ff_velocity: i32,
    pub(crate) ff_torque: i16,
    pub(crate) status_flags: u32,
}

/// Collects the readings captured during a voltage step test.
pub(crate) struct VoltageStepReport {
    pub(crate) oid: u8,
    pub(crate) status: u8,
    pub(crate) uq_ext: i16,
    pub(crate) ud_ext: i16,
    pub(crate) sample_delay_ms: u16,
    pub(crate) torque_before: i16,
    pub(crate) torque_sample: i16,
    pub(crate) torque_after: i16,
    pub(crate) flux_sample: i16,
    pub(crate) iq_sample: i16,
    pub(crate) id_sample: i16,
    pub(crate) uq_limited: i16,
    pub(crate) ud_limited: i16,
    pub(crate) uux_sample: i16,
    pub(crate) uwy_sample: i16,
    pub(crate) pwm_ux_sample: i16,
    pub(crate) pwm_wy_sample: i16,
    pub(crate) pwm_sv_chop: u32,
    pub(crate) pwm_bbm: u32,
    pub(crate) pwm_maxcnt: u32,
    pub(crate) phi_e_sample: i16,
    pub(crate) phi_m_sample: i16,
    pub(crate) encoder_before: u32,
    pub(crate) encoder_sample: u32,
    pub(crate) encoder_after: u32,
    pub(crate) encoder_delta_sample: i32,
    pub(crate) encoder_delta_after: i32,
    pub(crate) adc_vm_raw: u16,
    pub(crate) status_flags: u32,
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

/// Construct a zero-filled [`CurrentTorqueSampleReport`] with the given header fields.
pub(crate) fn empty_current_torque_sample_report(
    oid: u8,
    status: u8,
    target: i16,
    flux_target: i16,
    sample_delay_ms: u16,
    voltage_limit: u16,
) -> CurrentTorqueSampleReport {
    CurrentTorqueSampleReport {
        oid,
        status,
        target,
        flux_target,
        sample_delay_ms,
        voltage_limit,
        torque_before: 0,
        torque_sample: 0,
        torque_after: 0,
        flux_sample: 0,
        iq_sample: 0,
        id_sample: 0,
        uq_limited: 0,
        ud_limited: 0,
        encoder_before: 0,
        encoder_sample: 0,
        encoder_after: 0,
        encoder_delta_sample: 0,
        encoder_delta_after: 0,
        adc_vm_raw: 0,
        pidin_target_torque: 0,
        pidin_target_flux: 0,
        pidout_target_torque: 0,
        pidout_target_flux: 0,
        pid_torque_target_monitor: 0,
        torque_error: 0,
        flux_error: 0,
        torque_error_sum: 0,
        flux_error_sum: 0,
        uq_prelimit: 0,
        ud_prelimit: 0,
        ff_velocity: 0,
        ff_torque: 0,
        status_flags: 0,
    }
}

/// Construct a zero-filled [`VoltageStepReport`] with the given header fields.
pub(crate) fn empty_voltage_step_report(
    oid: u8,
    status: u8,
    uq_ext: i16,
    ud_ext: i16,
    sample_delay_ms: u16,
) -> VoltageStepReport {
    VoltageStepReport {
        oid,
        status,
        uq_ext,
        ud_ext,
        sample_delay_ms,
        torque_before: 0,
        torque_sample: 0,
        torque_after: 0,
        flux_sample: 0,
        iq_sample: 0,
        id_sample: 0,
        uq_limited: 0,
        ud_limited: 0,
        uux_sample: 0,
        uwy_sample: 0,
        pwm_ux_sample: 0,
        pwm_wy_sample: 0,
        pwm_sv_chop: 0,
        pwm_bbm: 0,
        pwm_maxcnt: 0,
        phi_e_sample: 0,
        phi_m_sample: 0,
        encoder_before: 0,
        encoder_sample: 0,
        encoder_after: 0,
        encoder_delta_sample: 0,
        encoder_delta_after: 0,
        adc_vm_raw: 0,
        status_flags: 0,
    }
}

/// Pack the uq and ud external voltage values into a single u32 register word.
pub(crate) fn pack_voltage_ext(uq_ext: i16, ud_ext: i16) -> u32 {
    (u32::from(uq_ext as u16) << 16) | u32::from(ud_ext as u16)
}

/// Compute the signed encoder delta between two raw position readings.
///
/// Wraps the result into the range `[-counts_per_rev/2, counts_per_rev/2]` when
/// `counts_per_rev` is positive, returning the raw difference otherwise.
pub(crate) fn current_step_encoder_delta(before: u32, after: u32, counts_per_rev: u32) -> i32 {
    let raw = (after as i32).wrapping_sub(before as i32);
    let counts_per_rev = counts_per_rev as i32;
    if counts_per_rev <= 0 {
        return raw;
    }

    let half = counts_per_rev / 2;
    if raw > half {
        raw - counts_per_rev
    } else if raw < -half {
        raw + counts_per_rev
    } else {
        raw
    }
}

/// Write a value to a TMC4671 sub-register via the address/data register pair.
async fn write_current_step_sub_register(
    tmc: &mut Tmc4671Spi<TmcSpiDevice>,
    sub: foci_tmc4671::registers::SubRegister,
    value: u32,
) -> Result<(), ()> {
    tmc.write_register(sub.addr, sub.sub_addr as u32)
        .await
        .map_err(|_| ())?;
    tmc.write_register(sub.data, value).await.map_err(|_| ())
}

/// Zero out the PID torque and flux error integrators.
pub(crate) async fn clear_current_step_integrators(
    tmc: &mut Tmc4671Spi<TmcSpiDevice>,
) -> Result<(), ()> {
    use foci_tmc4671::registers::{PID_ERROR_PID_FLUX_ERROR_SUM, PID_ERROR_PID_TORQUE_ERROR_SUM};

    write_current_step_sub_register(tmc, PID_ERROR_PID_TORQUE_ERROR_SUM, 0).await?;
    write_current_step_sub_register(tmc, PID_ERROR_PID_FLUX_ERROR_SUM, 0).await
}

/// Handle board-local active diagnostic requests.
pub(crate) async fn handle_diagnostic_request(
    req: TmcRequest,
    ctx: DiagnosticContext<'_>,
) -> DiagnosticDispatch {
    let tmc = ctx.tmc;
    let runtime = ctx.runtime;

    match req {
        TmcRequest {
            kind:
                current_step_kind @ (TmcRequestKind::CurrentStepTest { .. }
                | TmcRequestKind::CurrentVectorStepTest { .. }),
            ..
        } => {
            let (oid, torque_target, flux_target, duration_ms, voltage_limit, vector_reply) =
                match current_step_kind {
                    TmcRequestKind::CurrentStepTest {
                        oid,
                        target,
                        duration_ms,
                        voltage_limit,
                    } => (oid, target, 0, duration_ms, voltage_limit, false),
                    TmcRequestKind::CurrentVectorStepTest {
                        oid,
                        torque_target,
                        flux_target,
                        duration_ms,
                        voltage_limit,
                    } => (
                        oid,
                        torque_target,
                        flux_target,
                        duration_ms,
                        voltage_limit,
                        true,
                    ),
                    _ => unreachable!(),
                };
            let homing_active = ctx.homing_active;
            let stepper_active = ctx.stepper_active;
            if !runtime.is_motor_enabled()
                || !runtime.is_calibrated()
                || homing_active
                || stepper_active
            {
                send_current_step_reply(
                    vector_reply,
                    CurrentStepReport {
                        oid,
                        status: CURRENT_STEP_STATUS_NOT_READY,
                        torque_target,
                        flux_target,
                        duration_ms,
                        voltage_limit,
                        torque_before: 0,
                        torque_during: 0,
                        torque_after: 0,
                        flux_during: 0,
                        iq_during: 0,
                        id_during: 0,
                        uq_limited: 0,
                        ud_limited: 0,
                        encoder_before: 0,
                        encoder_after: 0,
                        encoder_delta: 0,
                        adc_vm_raw: 0,
                    },
                );
                return DiagnosticDispatch::Handled;
            }

            let saved_mode = match tmc
                .read_register(foci_tmc4671::registers::MODE_RAMP_MODE_MOTION)
                .await
            {
                Ok(value) => value,
                Err(_) => {
                    send_current_step_reply(
                        vector_reply,
                        CurrentStepReport {
                            oid,
                            status: CURRENT_STEP_STATUS_SPI,
                            torque_target,
                            flux_target,
                            duration_ms,
                            voltage_limit,
                            torque_before: 0,
                            torque_during: 0,
                            torque_after: 0,
                            flux_during: 0,
                            iq_during: 0,
                            id_during: 0,
                            uq_limited: 0,
                            ud_limited: 0,
                            encoder_before: 0,
                            encoder_after: 0,
                            encoder_delta: 0,
                            adc_vm_raw: 0,
                        },
                    );
                    return DiagnosticDispatch::Handled;
                }
            };
            let saved_voltage_limit = ctx.saved_voltage_limit;
            let position_actual = match tmc
                .read_register(foci_tmc4671::registers::PID_POSITION_ACTUAL)
                .await
            {
                Ok(value) => value,
                Err(_) => {
                    send_current_step_reply(
                        vector_reply,
                        CurrentStepReport {
                            oid,
                            status: CURRENT_STEP_STATUS_SPI,
                            torque_target,
                            flux_target,
                            duration_ms,
                            voltage_limit,
                            torque_before: 0,
                            torque_during: 0,
                            torque_after: 0,
                            flux_during: 0,
                            iq_during: 0,
                            id_during: 0,
                            uq_limited: 0,
                            ud_limited: 0,
                            encoder_before: 0,
                            encoder_after: 0,
                            encoder_delta: 0,
                            adc_vm_raw: 0,
                        },
                    );
                    return DiagnosticDispatch::Handled;
                }
            };

            let mut status = 0u8;
            let mut torque_before = 0i16;
            let mut torque_during = 0i16;
            let mut torque_after = 0i16;
            let mut flux_during = 0i16;
            let mut iq_during = 0i16;
            let mut id_during = 0i16;
            let mut uq_limited = 0i16;
            let mut ud_limited = 0i16;
            let mut encoder_before = 0u32;
            let mut encoder_after = 0u32;
            let mut encoder_delta = 0i32;
            let mut adc_vm_raw = 0u16;

            if !runtime.restore_phase_advance_now(tmc).await {
                defmt::warn!("phase advance: failed to restore ABN offset before current step");
                send_current_step_reply(
                    vector_reply,
                    CurrentStepReport {
                        oid,
                        status: CURRENT_STEP_STATUS_SPI,
                        torque_target,
                        flux_target,
                        duration_ms,
                        voltage_limit,
                        torque_before,
                        torque_during,
                        torque_after,
                        flux_during,
                        iq_during,
                        id_during,
                        uq_limited,
                        ud_limited,
                        encoder_before,
                        encoder_after,
                        encoder_delta,
                        adc_vm_raw,
                    },
                );
                return DiagnosticDispatch::Handled;
            }

            let result: Result<(), ()> = async {
                use foci_tmc4671::registers::*;

                tmc.write_register(PID_TORQUE_FLUX_TARGET, 0)
                    .await
                    .map_err(|_| ())?;
                tmc.write_register(PIDOUT_UQ_UD_LIMITS, voltage_limit as u32)
                    .await
                    .map_err(|_| ())?;
                clear_current_step_integrators(tmc).await?;
                tmc.write_register(MODE_RAMP_MODE_MOTION, CURRENT_STEP_TORQUE_MODE)
                    .await
                    .map_err(|_| ())?;

                Mono::delay(CURRENT_STEP_SETTLE_MS.millis()).await;

                encoder_before = tmc.read_register(ABN_DECODER_COUNT).await.map_err(|_| ())?;
                let encoder_counts_per_rev =
                    tmc.read_register(ABN_DECODER_PPR).await.map_err(|_| ())?;

                let before = tmc
                    .read_register(PID_TORQUE_FLUX_ACTUAL)
                    .await
                    .map_err(|_| ())?;
                torque_before = foci_firmware::tmc::decode_signed_upper_half(before);

                let target_reg =
                    foci_firmware::tmc::pack_torque_flux_target(torque_target, flux_target);
                tmc.write_register(PID_TORQUE_FLUX_TARGET, target_reg)
                    .await
                    .map_err(|_| ())?;
                Mono::delay((duration_ms as u64).millis()).await;

                let actual = tmc
                    .read_register(PID_TORQUE_FLUX_ACTUAL)
                    .await
                    .map_err(|_| ())?;
                let (torque, flux) = split_signed_halves(actual);
                torque_during = torque;
                flux_during = flux;

                let iq_id = tmc
                    .read_sub_register(INTERIM_FOC_IQ_ID)
                    .await
                    .map_err(|_| ())?;
                let (iq, id) = split_signed_halves(iq_id);
                iq_during = iq;
                id_during = id;

                let uq_ud = tmc
                    .read_sub_register(INTERIM_FOC_UQ_UD_LIMITED)
                    .await
                    .map_err(|_| ())?;
                let (uq, ud) = split_signed_halves(uq_ud);
                uq_limited = uq;
                ud_limited = ud;

                // TMC4671-LA Rev 1, 02/2024, ADC_RAW_DATA 0x02 selected by
                // ADC_RAW_ADDR 0x03: selector 1 packs ADC_AGPI_A_RAW in
                // bits 31:16 and ADC_VM_RAW in bits 15:0.
                adc_vm_raw = tmc
                    .read_sub_register(ADC_AGPI_A_RAW_ADC_VM_RAW)
                    .await
                    .map_err(|_| ())? as u16;

                tmc.write_register(PID_TORQUE_FLUX_TARGET, 0)
                    .await
                    .map_err(|_| ())?;
                Mono::delay(CURRENT_STEP_SETTLE_MS.millis()).await;

                encoder_after = tmc.read_register(ABN_DECODER_COUNT).await.map_err(|_| ())?;
                encoder_delta = current_step_encoder_delta(
                    encoder_before,
                    encoder_after,
                    encoder_counts_per_rev,
                );

                let after = tmc
                    .read_register(PID_TORQUE_FLUX_ACTUAL)
                    .await
                    .map_err(|_| ())?;
                torque_after = foci_firmware::tmc::decode_signed_upper_half(after);
                Ok(())
            }
            .await;

            if result.is_err() {
                status = CURRENT_STEP_STATUS_SPI;
            }

            let _ = tmc
                .write_register(foci_tmc4671::registers::PID_TORQUE_FLUX_TARGET, 0)
                .await;
            let _ = tmc
                .write_register(
                    foci_tmc4671::registers::PIDOUT_UQ_UD_LIMITS,
                    saved_voltage_limit,
                )
                .await;
            let _ = tmc
                .write_register(
                    foci_tmc4671::registers::PID_POSITION_TARGET,
                    position_actual,
                )
                .await;
            let restore_mode = if saved_mode == 0 {
                CURRENT_STEP_POSITION_MODE
            } else {
                saved_mode
            };
            let _ = tmc
                .write_register(foci_tmc4671::registers::MODE_RAMP_MODE_MOTION, restore_mode)
                .await;

            send_current_step_reply(
                vector_reply,
                CurrentStepReport {
                    oid,
                    status,
                    torque_target,
                    flux_target,
                    duration_ms,
                    voltage_limit,
                    torque_before,
                    torque_during,
                    torque_after,
                    flux_during,
                    iq_during,
                    id_during,
                    uq_limited,
                    ud_limited,
                    encoder_before,
                    encoder_after,
                    encoder_delta,
                    adc_vm_raw,
                },
            );
        }
        TmcRequest {
            kind:
                TmcRequestKind::CurrentTorqueSampleTest {
                    oid,
                    target,
                    flux_target,
                    sample_delay_ms,
                    voltage_limit,
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
                current_torque_sample_reply(empty_current_torque_sample_report(
                    oid,
                    CURRENT_STEP_STATUS_NOT_READY,
                    target,
                    flux_target,
                    sample_delay_ms,
                    voltage_limit,
                ));
                return DiagnosticDispatch::Handled;
            }

            let saved_mode = match tmc
                .read_register(foci_tmc4671::registers::MODE_RAMP_MODE_MOTION)
                .await
            {
                Ok(value) => value,
                Err(_) => {
                    current_torque_sample_reply(empty_current_torque_sample_report(
                        oid,
                        CURRENT_STEP_STATUS_SPI,
                        target,
                        flux_target,
                        sample_delay_ms,
                        voltage_limit,
                    ));
                    return DiagnosticDispatch::Handled;
                }
            };
            let saved_voltage_limit = ctx.saved_voltage_limit;
            let position_actual = match tmc
                .read_register(foci_tmc4671::registers::PID_POSITION_ACTUAL)
                .await
            {
                Ok(value) => value,
                Err(_) => {
                    current_torque_sample_reply(empty_current_torque_sample_report(
                        oid,
                        CURRENT_STEP_STATUS_SPI,
                        target,
                        flux_target,
                        sample_delay_ms,
                        voltage_limit,
                    ));
                    return DiagnosticDispatch::Handled;
                }
            };

            let mut status = 0u8;
            let mut torque_before = 0i16;
            let mut torque_sample = 0i16;
            let mut torque_after = 0i16;
            let mut flux_sample = 0i16;
            let mut iq_sample = 0i16;
            let mut id_sample = 0i16;
            let mut uq_limited = 0i16;
            let mut ud_limited = 0i16;
            let mut encoder_before = 0u32;
            let mut encoder_sample = 0u32;
            let mut encoder_after = 0u32;
            let mut encoder_delta_sample = 0i32;
            let mut encoder_delta_after = 0i32;
            let mut adc_vm_raw = 0u16;
            let mut pidin_target_torque = 0i32;
            let mut pidin_target_flux = 0i32;
            let mut pidout_target_torque = 0i32;
            let mut pidout_target_flux = 0i32;
            let mut pid_torque_target_monitor = 0i16;
            let mut torque_error = 0i32;
            let mut flux_error = 0i32;
            let mut torque_error_sum = 0i32;
            let mut flux_error_sum = 0i32;
            let mut uq_prelimit = 0i16;
            let mut ud_prelimit = 0i16;
            let mut ff_velocity = 0i32;
            let mut ff_torque = 0i16;
            let mut status_flags = 0u32;

            if !runtime.restore_phase_advance_now(tmc).await {
                defmt::warn!("phase advance: failed to restore ABN offset before torque sample");
                current_torque_sample_reply(empty_current_torque_sample_report(
                    oid,
                    CURRENT_STEP_STATUS_SPI,
                    target,
                    flux_target,
                    sample_delay_ms,
                    voltage_limit,
                ));
                return DiagnosticDispatch::Handled;
            }

            let result: Result<(), ()> = async {
                use foci_tmc4671::registers::*;

                tmc.write_register(PID_TORQUE_FLUX_TARGET, 0)
                    .await
                    .map_err(|_| ())?;
                tmc.write_register(PIDOUT_UQ_UD_LIMITS, voltage_limit as u32)
                    .await
                    .map_err(|_| ())?;
                clear_current_step_integrators(tmc).await?;
                tmc.write_register(STATUS_FLAGS, 0).await.map_err(|_| ())?;
                tmc.write_register(MODE_RAMP_MODE_MOTION, CURRENT_STEP_TORQUE_MODE)
                    .await
                    .map_err(|_| ())?;

                Mono::delay(CURRENT_STEP_SETTLE_MS.millis()).await;

                encoder_before = tmc.read_register(ABN_DECODER_COUNT).await.map_err(|_| ())?;
                let encoder_counts_per_rev =
                    tmc.read_register(ABN_DECODER_PPR).await.map_err(|_| ())?;

                let before = tmc
                    .read_register(PID_TORQUE_FLUX_ACTUAL)
                    .await
                    .map_err(|_| ())?;
                torque_before = foci_firmware::tmc::decode_signed_upper_half(before);

                let target_reg = foci_firmware::tmc::pack_torque_flux_target(target, flux_target);
                tmc.write_register(PID_TORQUE_FLUX_TARGET, target_reg)
                    .await
                    .map_err(|_| ())?;
                Mono::delay((sample_delay_ms as u64).millis()).await;

                pidin_target_torque = tmc
                    .read_sub_register(INTERIM_PIDIN_TARGET_TORQUE)
                    .await
                    .map_err(|_| ())? as i32;
                pidin_target_flux = tmc
                    .read_sub_register(INTERIM_PIDIN_TARGET_FLUX)
                    .await
                    .map_err(|_| ())? as i32;
                pidout_target_torque = tmc
                    .read_sub_register(INTERIM_PIDOUT_TARGET_TORQUE)
                    .await
                    .map_err(|_| ())? as i32;
                pidout_target_flux = tmc
                    .read_sub_register(INTERIM_PIDOUT_TARGET_FLUX)
                    .await
                    .map_err(|_| ())? as i32;
                let torque_target_actual = tmc
                    .read_sub_register(INTERIM_PID_TORQUE_TARGET_TORQUE_ACTUAL)
                    .await
                    .map_err(|_| ())?;
                pid_torque_target_monitor =
                    foci_firmware::tmc::decode_signed_upper_half(torque_target_actual);
                torque_error = tmc
                    .read_sub_register(PID_ERROR_PID_TORQUE_ERROR)
                    .await
                    .map_err(|_| ())? as i32;
                flux_error = tmc
                    .read_sub_register(PID_ERROR_PID_FLUX_ERROR)
                    .await
                    .map_err(|_| ())? as i32;
                torque_error_sum = tmc
                    .read_sub_register(PID_ERROR_PID_TORQUE_ERROR_SUM)
                    .await
                    .map_err(|_| ())? as i32;
                flux_error_sum = tmc
                    .read_sub_register(PID_ERROR_PID_FLUX_ERROR_SUM)
                    .await
                    .map_err(|_| ())? as i32;
                status_flags = tmc.read_register(STATUS_FLAGS).await.map_err(|_| ())?;

                encoder_sample = tmc.read_register(ABN_DECODER_COUNT).await.map_err(|_| ())?;
                encoder_delta_sample = current_step_encoder_delta(
                    encoder_before,
                    encoder_sample,
                    encoder_counts_per_rev,
                );

                let actual = tmc
                    .read_register(PID_TORQUE_FLUX_ACTUAL)
                    .await
                    .map_err(|_| ())?;
                let (torque, flux) = split_signed_halves(actual);
                torque_sample = torque;
                flux_sample = flux;

                let iq_id = tmc
                    .read_sub_register(INTERIM_FOC_IQ_ID)
                    .await
                    .map_err(|_| ())?;
                let (iq, id) = split_signed_halves(iq_id);
                iq_sample = iq;
                id_sample = id;

                let uq_ud_prelimit = tmc
                    .read_sub_register(INTERIM_FOC_UQ_UD)
                    .await
                    .map_err(|_| ())?;
                let (uq, ud) = split_signed_halves(uq_ud_prelimit);
                uq_prelimit = uq;
                ud_prelimit = ud;
                ff_velocity = tmc
                    .read_sub_register(INTERIM_FF_VELOCITY)
                    .await
                    .map_err(|_| ())? as i32;
                ff_torque = tmc
                    .read_sub_register(INTERIM_FF_TORQUE)
                    .await
                    .map_err(|_| ())? as i16;

                let uq_ud = tmc
                    .read_sub_register(INTERIM_FOC_UQ_UD_LIMITED)
                    .await
                    .map_err(|_| ())?;
                let (uq, ud) = split_signed_halves(uq_ud);
                uq_limited = uq;
                ud_limited = ud;

                // TMC4671-LA Rev 1, 02/2024, ADC_RAW_DATA 0x02 selected by
                // ADC_RAW_ADDR 0x03: selector 1 packs ADC_AGPI_A_RAW in
                // bits 31:16 and ADC_VM_RAW in bits 15:0.
                adc_vm_raw = tmc
                    .read_sub_register(ADC_AGPI_A_RAW_ADC_VM_RAW)
                    .await
                    .map_err(|_| ())? as u16;

                tmc.write_register(PID_TORQUE_FLUX_TARGET, 0)
                    .await
                    .map_err(|_| ())?;
                Mono::delay(CURRENT_STEP_SETTLE_MS.millis()).await;

                encoder_after = tmc.read_register(ABN_DECODER_COUNT).await.map_err(|_| ())?;
                encoder_delta_after = current_step_encoder_delta(
                    encoder_before,
                    encoder_after,
                    encoder_counts_per_rev,
                );

                let after = tmc
                    .read_register(PID_TORQUE_FLUX_ACTUAL)
                    .await
                    .map_err(|_| ())?;
                torque_after = foci_firmware::tmc::decode_signed_upper_half(after);
                Ok(())
            }
            .await;

            if result.is_err() {
                status = CURRENT_STEP_STATUS_SPI;
            }

            let _ = tmc
                .write_register(foci_tmc4671::registers::PID_TORQUE_FLUX_TARGET, 0)
                .await;
            let _ = tmc
                .write_register(
                    foci_tmc4671::registers::PIDOUT_UQ_UD_LIMITS,
                    saved_voltage_limit,
                )
                .await;
            let _ = tmc
                .write_register(
                    foci_tmc4671::registers::PID_POSITION_TARGET,
                    position_actual,
                )
                .await;
            let restore_mode = if saved_mode == 0 {
                CURRENT_STEP_POSITION_MODE
            } else {
                saved_mode
            };
            let _ = tmc
                .write_register(foci_tmc4671::registers::MODE_RAMP_MODE_MOTION, restore_mode)
                .await;

            let report = CurrentTorqueSampleReport {
                oid,
                status,
                target,
                flux_target,
                sample_delay_ms,
                voltage_limit,
                torque_before,
                torque_sample,
                torque_after,
                flux_sample,
                iq_sample,
                id_sample,
                uq_limited,
                ud_limited,
                encoder_before,
                encoder_sample,
                encoder_after,
                encoder_delta_sample,
                encoder_delta_after,
                adc_vm_raw,
                pidin_target_torque,
                pidin_target_flux,
                pidout_target_torque,
                pidout_target_flux,
                pid_torque_target_monitor,
                torque_error,
                flux_error,
                torque_error_sum,
                flux_error_sum,
                uq_prelimit,
                ud_prelimit,
                ff_velocity,
                ff_torque,
                status_flags,
            };
            if status == 0 {
                current_torque_sample_detail_reply(&report);
            }
            current_torque_sample_reply(report);
        }
        TmcRequest {
            kind:
                TmcRequestKind::PositionTorqueOffsetSampleTest {
                    oid,
                    target,
                    sample_delay_ms,
                    voltage_limit,
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
                current_torque_sample_reply(empty_current_torque_sample_report(
                    oid,
                    CURRENT_STEP_STATUS_NOT_READY,
                    target,
                    0,
                    sample_delay_ms,
                    voltage_limit,
                ));
                return DiagnosticDispatch::Handled;
            }

            let saved_mode = match tmc
                .read_register(foci_tmc4671::registers::MODE_RAMP_MODE_MOTION)
                .await
            {
                Ok(value) => value,
                Err(_) => {
                    current_torque_sample_reply(empty_current_torque_sample_report(
                        oid,
                        CURRENT_STEP_STATUS_SPI,
                        target,
                        0,
                        sample_delay_ms,
                        voltage_limit,
                    ));
                    return DiagnosticDispatch::Handled;
                }
            };
            if saved_mode != CURRENT_STEP_POSITION_MODE {
                current_torque_sample_reply(empty_current_torque_sample_report(
                    oid,
                    CURRENT_STEP_STATUS_NOT_READY,
                    target,
                    0,
                    sample_delay_ms,
                    voltage_limit,
                ));
                return DiagnosticDispatch::Handled;
            }

            let saved_voltage_limit = ctx.saved_voltage_limit;
            let saved_offset = match tmc
                .read_register(foci_tmc4671::registers::PID_TORQUE_FLUX_OFFSET)
                .await
            {
                Ok(value) => value,
                Err(_) => {
                    current_torque_sample_reply(empty_current_torque_sample_report(
                        oid,
                        CURRENT_STEP_STATUS_SPI,
                        target,
                        0,
                        sample_delay_ms,
                        voltage_limit,
                    ));
                    return DiagnosticDispatch::Handled;
                }
            };

            let mut report = empty_current_torque_sample_report(
                oid,
                0,
                target,
                0,
                sample_delay_ms,
                voltage_limit,
            );

            let result: Result<(), ()> = async {
                use foci_tmc4671::registers::*;

                tmc.write_register(PIDOUT_UQ_UD_LIMITS, voltage_limit as u32)
                    .await
                    .map_err(|_| ())?;
                tmc.write_register(STATUS_FLAGS, 0).await.map_err(|_| ())?;

                report.encoder_before =
                    tmc.read_register(ABN_DECODER_COUNT).await.map_err(|_| ())?;
                let encoder_counts_per_rev =
                    tmc.read_register(ABN_DECODER_PPR).await.map_err(|_| ())?;

                let before = tmc
                    .read_register(PID_TORQUE_FLUX_ACTUAL)
                    .await
                    .map_err(|_| ())?;
                report.torque_before = foci_firmware::tmc::decode_signed_upper_half(before);

                let offset_reg = u32::from(target as u16) << 16;
                tmc.write_register(PID_TORQUE_FLUX_OFFSET, offset_reg)
                    .await
                    .map_err(|_| ())?;
                Mono::delay((sample_delay_ms as u64).millis()).await;

                report.pidin_target_torque = tmc
                    .read_sub_register(INTERIM_PIDIN_TARGET_TORQUE)
                    .await
                    .map_err(|_| ())? as i32;
                report.pidin_target_flux = tmc
                    .read_sub_register(INTERIM_PIDIN_TARGET_FLUX)
                    .await
                    .map_err(|_| ())? as i32;
                report.pidout_target_torque = tmc
                    .read_sub_register(INTERIM_PIDOUT_TARGET_TORQUE)
                    .await
                    .map_err(|_| ())? as i32;
                report.pidout_target_flux = tmc
                    .read_sub_register(INTERIM_PIDOUT_TARGET_FLUX)
                    .await
                    .map_err(|_| ())? as i32;
                let torque_target_actual = tmc
                    .read_sub_register(INTERIM_PID_TORQUE_TARGET_TORQUE_ACTUAL)
                    .await
                    .map_err(|_| ())?;
                report.pid_torque_target_monitor =
                    foci_firmware::tmc::decode_signed_upper_half(torque_target_actual);
                report.torque_error = tmc
                    .read_sub_register(PID_ERROR_PID_TORQUE_ERROR)
                    .await
                    .map_err(|_| ())? as i32;
                report.flux_error = tmc
                    .read_sub_register(PID_ERROR_PID_FLUX_ERROR)
                    .await
                    .map_err(|_| ())? as i32;
                report.torque_error_sum = tmc
                    .read_sub_register(PID_ERROR_PID_TORQUE_ERROR_SUM)
                    .await
                    .map_err(|_| ())? as i32;
                report.flux_error_sum = tmc
                    .read_sub_register(PID_ERROR_PID_FLUX_ERROR_SUM)
                    .await
                    .map_err(|_| ())? as i32;
                report.status_flags = tmc.read_register(STATUS_FLAGS).await.map_err(|_| ())?;

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

                let uq_ud_prelimit = tmc
                    .read_sub_register(INTERIM_FOC_UQ_UD)
                    .await
                    .map_err(|_| ())?;
                let (uq, ud) = split_signed_halves(uq_ud_prelimit);
                report.uq_prelimit = uq;
                report.ud_prelimit = ud;
                report.ff_velocity = tmc
                    .read_sub_register(INTERIM_FF_VELOCITY)
                    .await
                    .map_err(|_| ())? as i32;
                report.ff_torque = tmc
                    .read_sub_register(INTERIM_FF_TORQUE)
                    .await
                    .map_err(|_| ())? as i16;

                let uq_ud = tmc
                    .read_sub_register(INTERIM_FOC_UQ_UD_LIMITED)
                    .await
                    .map_err(|_| ())?;
                let (uq, ud) = split_signed_halves(uq_ud);
                report.uq_limited = uq;
                report.ud_limited = ud;

                // TMC4671-LA Rev 1, 02/2024, ADC_RAW_DATA 0x02 selected by
                // ADC_RAW_ADDR 0x03: selector 1 packs ADC_AGPI_A_RAW in
                // bits 31:16 and ADC_VM_RAW in bits 15:0.
                report.adc_vm_raw = tmc
                    .read_sub_register(ADC_AGPI_A_RAW_ADC_VM_RAW)
                    .await
                    .map_err(|_| ())? as u16;

                tmc.write_register(PID_TORQUE_FLUX_OFFSET, saved_offset)
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
            }

            let _ = tmc
                .write_register(
                    foci_tmc4671::registers::PID_TORQUE_FLUX_OFFSET,
                    saved_offset,
                )
                .await;
            let _ = tmc
                .write_register(
                    foci_tmc4671::registers::PIDOUT_UQ_UD_LIMITS,
                    saved_voltage_limit,
                )
                .await;
            let _ = tmc
                .write_register(foci_tmc4671::registers::MODE_RAMP_MODE_MOTION, saved_mode)
                .await;

            if report.status == 0 {
                current_torque_sample_detail_reply(&report);
            }
            current_torque_sample_reply(report);
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
