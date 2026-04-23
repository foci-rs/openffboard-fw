//! FOCI handlers: `foci_calibrate`, `foci_commission`, `foci_dump_registers`,
//! `foci_selftest`, `foci_trace_fetch`, `foci_trace_info`, `foci_tune`.
//!
//! Ported from `mod app` in `main.rs` by Task 8l of the anchor → ankyra
//! migration. Each handler takes `&mut FirmwareCtx<'_>` as its context
//! and accesses the underlying Klipper `State` via `ctx.state`. Failure
//! replies for commissioning/calibration are emitted synchronously so
//! the host always sees a terminal reply even when the request cannot
//! be enqueued; progress/success replies are emitted later from the
//! `tmc_control` task.

use ankyra::prelude::klipper_command;

use crate::firmware_ctx::FirmwareCtx;
use crate::helpers::tmc::{FociTuneParams, foci_tune_inner, tmc_send};
use crate::state::tmc::{COMMISSION_IN_FLIGHT, STEP_WIDTH_VALUE};

/// Trigger TMC4671 self-test diagnostic via the commissioning engine.
///
/// Only one commissioning operation may be in flight at a time.
/// Concurrent requests are rejected.
#[klipper_command]
pub fn foci_selftest(_ctx: &mut FirmwareCtx<'_>, oid: u8) {
    use core::sync::atomic::Ordering::Relaxed;
    if COMMISSION_IN_FLIGHT.swap(true, Relaxed) {
        defmt::warn!("foci_selftest rejected: commission in flight");
        ::ankyra::klipper_reply!(
            crate::ankyra::replies::FociSelftestDone,
            oid: u8 = oid,
            status: u8 = foci_firmware::autotune::CommissionError::AlreadyRunning.status_code()
        );
        return;
    }
    if !tmc_send(foci_firmware::tmc::tmc_selftest(oid)) {
        COMMISSION_IN_FLIGHT.store(false, Relaxed);
        ::ankyra::klipper_reply!(
            crate::ankyra::replies::FociSelftestDone,
            oid: u8 = oid,
            status: u8 = foci_firmware::autotune::CommissionError::QueueFull.status_code()
        );
    }
}

/// Calibrate the TMC4671: ADC offset calibration, encoder alignment,
/// and closed-loop entry via the commissioning engine.
///
/// Must be called before motor enable. Rejected if motor is already
/// enabled or another commissioning operation is in flight.
#[klipper_command]
pub fn foci_calibrate(_ctx: &mut FirmwareCtx<'_>, oid: u8) {
    use core::sync::atomic::Ordering::Relaxed;
    // Pre-flight: STEP_WIDTH_VALUE must have been populated by a prior
    // tmc_set_encoder command. If it hasn't, fail before touching hardware
    // — otherwise calibration would succeed but the motor would silently
    // refuse to respond to step pulses. The engine's ClosedLoopEntry
    // phase also checks this via backend.step_width().
    if STEP_WIDTH_VALUE.load(Relaxed) <= 0 {
        defmt::error!(
            "foci_calibrate rejected: STEP_WIDTH not configured (host must call tmc_set_encoder first)"
        );
        ::ankyra::klipper_reply!(
            crate::ankyra::replies::FociCalibrateResult,
            oid: u8 = oid,
            status: u8 = foci_firmware::calibration::CalibrationError::ConfigFault.status_code(),
            adc_i0: u16 = 0,
            adc_i1: u16 = 0,
            encoder_count: u32 = 0
        );
        return;
    }
    if COMMISSION_IN_FLIGHT.swap(true, Relaxed) {
        defmt::warn!("foci_calibrate rejected: commission in flight");
        ::ankyra::klipper_reply!(
            crate::ankyra::replies::FociCalibrateResult,
            oid: u8 = oid,
            status: u8 = foci_firmware::calibration::CalibrationError::InternalError.status_code(),
            adc_i0: u16 = 0,
            adc_i1: u16 = 0,
            encoder_count: u32 = 0
        );
        return;
    }
    if !tmc_send(foci_firmware::tmc::tmc_calibrate(oid)) {
        COMMISSION_IN_FLIGHT.store(false, Relaxed);
        ::ankyra::klipper_reply!(
            crate::ankyra::replies::FociCalibrateResult,
            oid: u8 = oid,
            status: u8 = foci_firmware::calibration::CalibrationError::InternalError.status_code(),
            adc_i0: u16 = 0,
            adc_i1: u16 = 0,
            encoder_count: u32 = 0
        );
    }
}

/// Dump all TMC4671 registers via klipper_output protocol.
///
/// The firmware reads registers internally and streams results as
/// `foci_dump_value` replies terminated by `foci_dump_done`. Replaces
/// Python-side register reads.
#[klipper_command]
pub fn foci_dump_registers(_ctx: &mut FirmwareCtx<'_>, oid: u8) {
    if !tmc_send(foci_firmware::tmc::tmc_dump_registers(oid)) {
        defmt::warn!("foci_dump_registers dropped");
        // No reply — let the Python wait loop timeout with an
        // explicit "register dump timed out" error.
    }
}

/// Query trace buffer metadata.
#[klipper_command]
pub fn foci_trace_info(_ctx: &mut FirmwareCtx<'_>, oid: u8) {
    if !tmc_send(foci_firmware::tmc::tmc_trace_info(oid)) {
        defmt::warn!("trace_info: TMC channel full");
    }
}

/// Fetch a trace sample by offset. `generation` must match current generation.
#[klipper_command]
pub fn foci_trace_fetch(_ctx: &mut FirmwareCtx<'_>, oid: u8, offset: u16, generation: u8) {
    if !tmc_send(foci_firmware::tmc::tmc_trace_fetch(oid, offset, generation)) {
        defmt::warn!("trace_fetch: TMC channel full");
    }
}

/// Start Stage 1 commissioning (diagnostics + current tune + closed-loop entry).
#[klipper_command]
pub fn foci_commission(_ctx: &mut FirmwareCtx<'_>, oid: u8, profile: u8) {
    use core::sync::atomic::Ordering::Relaxed;
    if COMMISSION_IN_FLIGHT.swap(true, Relaxed) {
        ::ankyra::klipper_reply!(
            crate::ankyra::replies::FociCommissionPhase,
            oid: u8 = oid,
            phase: u8 = 0,
            status: u8 = foci_firmware::autotune::CommissionError::AlreadyRunning.status_code()
        );
        return;
    }
    if !tmc_send(foci_firmware::tmc::tmc_commission(oid, profile)) {
        COMMISSION_IN_FLIGHT.store(false, Relaxed);
        ::ankyra::klipper_reply!(
            crate::ankyra::replies::FociCommissionPhase,
            oid: u8 = oid,
            phase: u8 = 0,
            status: u8 = foci_firmware::autotune::CommissionError::QueueFull.status_code()
        );
    }
}

/// Start Stage 2 installed tuning (mechanical ID + velocity/position/filters).
#[klipper_command]
pub fn foci_tune(
    _ctx: &mut FirmwareCtx<'_>,
    oid: u8,
    profile: u8,
    mode: u8,
    inner_lambda: u32,
    theta_e: u32,
    current_ringing: u8,
    current_bw: u32,
) {
    foci_tune_inner(FociTuneParams {
        oid,
        profile,
        mode,
        inner_lambda,
        theta_e,
        current_ringing,
        current_bw,
    });
}
