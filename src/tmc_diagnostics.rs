//! OpenFFBoard-local active TMC diagnostics.
//!
//! These flows remain board-local during the first shared runtime extraction
//! because they own large reply payloads, board delays, and diagnostic-specific
//! restore paths.

use foci_tmc4671::spi::Tmc4671Spi;

use crate::app_types::TmcSpiDevice;

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
