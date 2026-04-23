//! Digital-output command handlers: `config_digital_out`,
//! `queue_digital_out`, `update_digital_out`.
//!
//! Ported from `mod app` in `main.rs` by Task 8f of the anchor → ankyra
//! migration. Each handler takes `&mut FirmwareCtx<'_>` as its context
//! and accesses the underlying Klipper `State` via `ctx.state`.

use ankyra::prelude::klipper_command;
use foci_firmware::tmc::TmcRequest;

use crate::firmware_ctx::FirmwareCtx;
use crate::helpers::lifecycle::{enter_shutdown, read_tim5};
use crate::helpers::tmc::tmc_send;

/// Register a digital output object for motor enable/disable.
///
/// Pin parameters are accepted for protocol compatibility but ignored;
/// the TMC4671 enable line is controlled directly.
#[klipper_command]
pub fn config_digital_out(
    ctx: &mut FirmwareCtx<'_>,
    oid: u8,
    _pin: u32,
    _value: u8,
    _default_value: u8,
    _max_duration: u32,
) {
    if let Err(e) = foci_klipper::commands::digital_out::config_digital_out(ctx.state, oid) {
        defmt::error!("config_digital_out: {}", e);
        if enter_shutdown(ctx.state) {
            ::ankyra::klipper_shutdown!("config_digital_out failed", read_tim5());
        }
        return;
    }
    defmt::info!(
        "config_digital_out: oid={} (enable control, pin ignored)",
        oid
    );
}

/// Schedule a digital output change at a given clock time.
///
/// Used by Klipper for timed motor enable/disable.
#[klipper_command]
pub fn queue_digital_out(ctx: &mut FirmwareCtx<'_>, oid: u8, clock: u32, on_ticks: u32) {
    let _ = clock;
    match foci_klipper::commands::digital_out::queue_digital_out(ctx.state, oid, on_ticks) {
        Ok(enabled) => {
            if !tmc_send(TmcRequest::SetEnable(enabled)) && enter_shutdown(ctx.state) {
                ::ankyra::klipper_shutdown!("Motor enable/disable lost (queue full)", read_tim5());
            }
        }
        Err(e) => {
            defmt::error!("queue_digital_out: {}", e);
            if enter_shutdown(ctx.state) {
                ::ankyra::klipper_shutdown!("queue_digital_out failed", read_tim5());
            }
        }
    }
}

/// Immediately update a digital output value.
#[klipper_command]
pub fn update_digital_out(ctx: &mut FirmwareCtx<'_>, oid: u8, value: u8) {
    match foci_klipper::commands::digital_out::update_digital_out(ctx.state, oid, value) {
        Ok(enabled) => {
            if !tmc_send(TmcRequest::SetEnable(enabled)) && enter_shutdown(ctx.state) {
                ::ankyra::klipper_shutdown!("Motor enable/disable lost (queue full)", read_tim5());
            }
        }
        Err(e) => {
            defmt::error!("update_digital_out: {}", e);
            if enter_shutdown(ctx.state) {
                ::ankyra::klipper_shutdown!("update_digital_out failed", read_tim5());
            }
        }
    }
}
