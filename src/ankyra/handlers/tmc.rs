//! TMC4671 handlers: `tmc_init`, `tmc_read_register`, `tmc_write_register`,
//! `tmc_set_current`, `tmc_set_encoder`, `tmc_set_encoder_dir`,
//! `tmc_set_flux_filter`, `tmc_set_pid_gains`, `tmc_set_position_filter`,
//! `tmc_set_position_gains`, `tmc_set_torque_filter`,
//! `tmc_set_velocity_feedforward`, `tmc_set_velocity_filter`,
//! `tmc_set_velocity_limit`, `tmc_test_spin`.
//!
//! Ported from `mod app` in `main.rs` by Task 8k of the anchor → ankyra
//! migration. Each handler takes `&mut FirmwareCtx<'_>` as its context
//! and accesses the underlying Klipper `State` via `ctx.state`.

use ankyra::prelude::klipper_command;

use crate::firmware_ctx::FirmwareCtx;
use crate::helpers::lifecycle::{enter_shutdown, read_tim5};
use crate::helpers::tmc::tmc_send;
use foci_firmware::state::tmc::{
    FLUX_FILTER_HZ, POSITION_FILTER_HZ, STEP_WIDTH_VALUE, TORQUE_FILTER_HZ, VELOCITY_FEEDFORWARD,
    VELOCITY_FILTER_HZ, VELOCITY_LIMIT_CFG,
};

/// Number of TMC4671 motor channels on OpenFFBoard (single-motor board).
const TMC_MOTOR_COUNT: u8 = 1;

/// Trigger TMC4671 initialization sequence.
#[cfg(feature = "dev")]
#[klipper_command]
pub fn tmc_init(_ctx: &mut FirmwareCtx<'_>, _oid: u8) {
    tmc_send(foci_firmware::tmc::tmc_init());
}

/// Test motor rotation by sweeping PHI_E_EXT through N revolutions.
#[cfg(feature = "dev")]
#[klipper_command]
pub fn tmc_test_spin(_ctx: &mut FirmwareCtx<'_>, _oid: u8, revolutions: u8) {
    tmc_send(foci_firmware::tmc::tmc_test_spin(revolutions));
}

/// Write a TMC4671 register via SPI.
#[cfg(feature = "dev")]
#[klipper_command]
pub fn tmc_write_register(_ctx: &mut FirmwareCtx<'_>, _oid: u8, addr: u8, value: u32) {
    tmc_send(foci_firmware::tmc::tmc_write_register(addr, value));
}

/// Read a TMC4671 register via SPI.
///
/// The response is sent asynchronously as a tmc_register_value reply.
#[cfg(feature = "dev")]
#[klipper_command]
pub fn tmc_read_register(_ctx: &mut FirmwareCtx<'_>, oid: u8, addr: u8) {
    if !tmc_send(foci_firmware::tmc::tmc_read_register(oid, addr)) {
        defmt::warn!("tmc_read_register dropped: addr={:#04x}", addr);
        // No reply — let the Python query_command timeout naturally.
    }
}

/// Set motor run current. The firmware converts milliamps to
/// PID_TORQUE_FLUX_LIMITS register counts using the board-specific
/// current scaling factor.
#[klipper_command]
pub fn tmc_set_current(ctx: &mut FirmwareCtx<'_>, _oid: u8, run_ma: u32) {
    if !tmc_send(foci_firmware::tmc::tmc_set_current(run_ma)) && enter_shutdown(ctx.state) {
        ::ankyra::klipper_shutdown!("tmc_set_current lost (queue full)", read_tim5());
    }
}

/// Configure ABN encoder resolution and compute step width.
///
/// Uses `tmc_set_encoder_hw` to compute the step width from PPR and
/// pole pairs, then stores it in the `STEP_WIDTH_VALUE` atomic for
/// the TMC4671 STEP_WIDTH register. The TMC4671 position PID operates
/// on phi_e, not encoder counts:
///   step_width = 65536 * pole_pairs / quadrature_ppr
#[klipper_command]
pub fn tmc_set_encoder(ctx: &mut FirmwareCtx<'_>, _oid: u8, channel: u8, ppr: u32) {
    if channel >= TMC_MOTOR_COUNT {
        if enter_shutdown(ctx.state) {
            ::ankyra::klipper_shutdown!("Invalid TMC channel", read_tim5());
        }
        return;
    }
    // Use _hw variant: returns step_width for TMC4671 STEP_WIDTH register
    let (req, step_width) = foci_firmware::tmc::tmc_set_encoder_hw(ppr, 50);
    STEP_WIDTH_VALUE.store(step_width, core::sync::atomic::Ordering::Relaxed);
    defmt::info!("channel={} ppr={} step_width={}", channel, ppr, step_width);
    if !tmc_send(req) && enter_shutdown(ctx.state) {
        ::ankyra::klipper_shutdown!("tmc_set_encoder lost (queue full)", read_tim5());
    }
}

/// Set ABN encoder count direction. Inverted=1 flips the count
/// direction so the position PID has correct (negative) feedback.
/// Depends on motor/encoder wiring — not board-specific.
#[klipper_command]
pub fn tmc_set_encoder_dir(ctx: &mut FirmwareCtx<'_>, _oid: u8, channel: u8, invert: u8) {
    if channel >= TMC_MOTOR_COUNT {
        if enter_shutdown(ctx.state) {
            ::ankyra::klipper_shutdown!("Invalid TMC channel", read_tim5());
        }
        return;
    }
    if !tmc_send(foci_firmware::tmc::tmc_set_encoder_dir(invert != 0)) && enter_shutdown(ctx.state)
    {
        ::ankyra::klipper_shutdown!("tmc_set_encoder_dir lost (queue full)", read_tim5());
    }
}

/// Set inner loop PI gains from host (explicit override or saved values).
#[klipper_command]
pub fn tmc_set_pid_gains(
    ctx: &mut FirmwareCtx<'_>,
    _oid: u8,
    flux_p: u16,
    flux_i: u16,
    torque_p: u16,
    torque_i: u16,
) {
    if !tmc_send(foci_firmware::tmc::tmc_set_pid_gains(
        flux_p, flux_i, torque_p, torque_i,
    )) && enter_shutdown(ctx.state)
    {
        ::ankyra::klipper_shutdown!("tmc_set_pid_gains lost (queue full)", read_tim5());
    }
}

/// Set velocity biquad filter cutoff frequency from host config.
#[klipper_command]
pub fn tmc_set_velocity_filter(_ctx: &mut FirmwareCtx<'_>, _oid: u8, filter_hz: u16) {
    VELOCITY_FILTER_HZ.store(filter_hz, core::sync::atomic::Ordering::Relaxed);
    defmt::info!("Velocity filter: {} Hz", filter_hz);
}

/// Set torque biquad filter cutoff frequency from host config.
#[klipper_command]
pub fn tmc_set_torque_filter(_ctx: &mut FirmwareCtx<'_>, _oid: u8, filter_hz: u16) {
    TORQUE_FILTER_HZ.store(filter_hz, core::sync::atomic::Ordering::Relaxed);
    defmt::info!("Torque filter: {} Hz", filter_hz);
}

/// Set position target biquad filter cutoff frequency from host config.
#[klipper_command]
pub fn tmc_set_position_filter(_ctx: &mut FirmwareCtx<'_>, _oid: u8, filter_hz: u16) {
    POSITION_FILTER_HZ.store(filter_hz, core::sync::atomic::Ordering::Relaxed);
    defmt::info!("Position filter: {} Hz", filter_hz);
}

/// Set flux biquad filter cutoff frequency from host config.
#[klipper_command]
pub fn tmc_set_flux_filter(_ctx: &mut FirmwareCtx<'_>, _oid: u8, filter_hz: u16) {
    FLUX_FILTER_HZ.store(filter_hz, core::sync::atomic::Ordering::Relaxed);
    defmt::info!("Flux filter: {} Hz", filter_hz);
}

/// Set position/velocity PI gains from host config.
#[klipper_command]
pub fn tmc_set_position_gains(
    ctx: &mut FirmwareCtx<'_>,
    _oid: u8,
    position_p: u16,
    position_i: u16,
    velocity_p: u16,
    velocity_i: u16,
) {
    if !tmc_send(foci_firmware::tmc::tmc_set_position_gains(
        position_p, position_i, velocity_p, velocity_i,
    )) && enter_shutdown(ctx.state)
    {
        ::ankyra::klipper_shutdown!("tmc_set_position_gains lost (queue full)", read_tim5());
    }
}

/// Enable or disable velocity feedforward from host config.
#[klipper_command]
pub fn tmc_set_velocity_feedforward(_ctx: &mut FirmwareCtx<'_>, _oid: u8, enable: u8) {
    VELOCITY_FEEDFORWARD.store(enable != 0, core::sync::atomic::Ordering::Relaxed);
    defmt::info!("Velocity feedforward: {}", enable != 0);
}

/// Set PID velocity limit from host config.
/// Caps position PID output to prevent integrator-driven runaway.
/// Firmware default: 500,000. Override via printer.cfg pid_velocity_limit.
#[klipper_command]
pub fn tmc_set_velocity_limit(_ctx: &mut FirmwareCtx<'_>, _oid: u8, limit: u32) {
    VELOCITY_LIMIT_CFG.store(limit, core::sync::atomic::Ordering::Relaxed);
    defmt::info!("Velocity limit: {}", limit);
}
