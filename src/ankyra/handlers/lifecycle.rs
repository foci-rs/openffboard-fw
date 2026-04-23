//! Lifecycle command handlers: status queries (`get_clock`, `get_uptime`,
//! `get_config`), recovery (`clear_shutdown`, `config_reset`, `reset`),
//! and diagnostic (`emergency_stop`, `debug_nop`, `allocate_oids`,
//! `finalize_config`).
//!
//! Ported from `mod app` in `main.rs` by Task 8e of the anchor → ankyra
//! migration. Each handler takes `&mut FirmwareCtx<'_>` as its context
//! and accesses the underlying Klipper `State` via `ctx.state`.

use ankyra::prelude::klipper_command;

use crate::firmware_ctx::FirmwareCtx;
use crate::helpers::lifecycle::{KATAPULT_BYPASS, enter_shutdown, katapult_reset, read_tim5};
use crate::state::lifecycle::{SHUTDOWN_ACTIVE, STATS_SEND_TIME, STATS_SEND_TIME_HIGH};
use crate::state::stepper::STEPPER_POSITION_TARGET;
use crate::state::tmc::{
    COMMISSION_IN_FLIGHT, CURRENT_LIMIT_REG, ENCODER_REVERSED, FEEDFORWARD_RESET, FLUX_FILTER_HZ,
    PID_ERROR_SUMS_CLEAR, POSITION_FILTER_HZ, TMC_QUEUE_FLUSH, TORQUE_FILTER_HZ,
    VELOCITY_FEEDFORWARD, VELOCITY_FILTER_HZ, VELOCITY_LIMIT_CFG,
};
use crate::state::trsync::{ENDSTOP_PIN_IDS, HOMING_STATE, HOMING_TRIGGERED, REPORT_PENDING};
use crate::trigger_domain::{
    PHYSICAL_STEPPER_INDICES, stop_physical_stepper_no_lock, with_trigger_domain,
};

#[klipper_command(in_shutdown)]
pub fn get_clock(_ctx: &mut FirmwareCtx<'_>) {
    let clock = read_tim5();
    ::ankyra::klipper_reply!(crate::ankyra::replies::Clock, clock: u32 = clock);
}

#[klipper_command(in_shutdown)]
pub fn get_uptime(_ctx: &mut FirmwareCtx<'_>) {
    use core::sync::atomic::Ordering::Relaxed;
    let cur = read_tim5();
    let high = STATS_SEND_TIME_HIGH.load(Relaxed) + u32::from(cur < STATS_SEND_TIME.load(Relaxed));
    ::ankyra::klipper_reply!(
        crate::ankyra::replies::Uptime,
        high: u32 = high,
        clock: u32 = cur,
    );
}

#[klipper_command(in_shutdown)]
pub fn get_config(ctx: &mut FirmwareCtx<'_>) {
    let (is_config, crc, is_shutdown, move_count) =
        foci_klipper::commands::lifecycle::get_config(ctx.state);
    ::ankyra::klipper_reply!(
        crate::ankyra::replies::Config,
        is_config: bool = is_config,
        crc: u32 = crc,
        is_shutdown: bool = is_shutdown,
        move_count: u16 = move_count,
    );
}

/// Clear shutdown state, allowing normal command processing to resume.
///
/// Matches Klipper C `command_clear_shutdown` (basecmd.c). If called
/// when not in shutdown, triggers a shutdown instead.
#[klipper_command(in_shutdown)]
pub fn clear_shutdown(ctx: &mut FirmwareCtx<'_>) {
    if foci_klipper::commands::lifecycle::clear_shutdown(ctx.state).is_err()
        && enter_shutdown(ctx.state)
    {
        ::ankyra::klipper_shutdown!("Shutdown cleared when not shutdown", read_tim5());
    }
    SHUTDOWN_ACTIVE.store(false, core::sync::atomic::Ordering::Relaxed);
}

/// Full config reset: clear all OIDs, steppers, and shutdown state.
///
/// Matches Klipper C `config_reset` (basecmd.c:262). Only available
/// during shutdown. The host uses this as a fallback restart mechanism
/// when hardware `reset` is not available (linux/PRU platforms).
#[klipper_command(in_shutdown)]
pub fn config_reset(ctx: &mut FirmwareCtx<'_>) {
    use core::sync::atomic::Ordering::Relaxed;
    if !ctx.state.mcu_state.is_shutdown() {
        if enter_shutdown(ctx.state) {
            ::ankyra::klipper_shutdown!("config_reset only available when shutdown", read_tim5());
        }
        return;
    }
    foci_klipper::commands::lifecycle::config_reset(ctx.state);
    // Full TriggerDomain reset
    with_trigger_domain(|td| {
        td.scheduler.clear();
        let tim5 = embassy_stm32::pac::TIM5;
        tim5.dier().modify(|w| w.set_ccie(1, false));
        for ts in td.trsync.iter_mut() {
            ts.reset();
        }
        for es in td.endstops.iter_mut() {
            es.reset();
        }
        for sig in td.stop_signals.iter_mut() {
            *sig = None;
        }
        td.trsync_by_oid.clear();
        td.endstop_by_oid.clear();
        // Stop physical steppers only
        for &idx in PHYSICAL_STEPPER_INDICES {
            stop_physical_stepper_no_lock(idx);
            td.set_timer_active(idx, false);
        }
        // Reset all stepper stop state (physical + logical)
        for ss in td.stepper_stop.iter_mut() {
            *ss = foci_klipper::trigger::StepperStopRuntime::new();
        }
    });
    // Clear derived mailbox state outside critical section
    REPORT_PENDING.store(false, Relaxed);
    HOMING_TRIGGERED.store(0xFF, Relaxed);
    HOMING_STATE.store(0, Relaxed);
    // Clear firmware-side endstop pin assignments
    for pin_id in &ENDSTOP_PIN_IDS {
        pin_id.store(0xFF, Relaxed);
    }
    // Reset board-local control and protocol state
    CURRENT_LIMIT_REG.store(0, Relaxed);
    VELOCITY_FILTER_HZ.store(0, Relaxed);
    TORQUE_FILTER_HZ.store(0, Relaxed);
    POSITION_FILTER_HZ.store(0, Relaxed);
    FLUX_FILTER_HZ.store(0, Relaxed);
    VELOCITY_FEEDFORWARD.store(false, Relaxed);
    VELOCITY_LIMIT_CFG.store(500_000, Relaxed);
    FEEDFORWARD_RESET.store(false, Relaxed);
    PID_ERROR_SUMS_CLEAR.store(false, Relaxed);
    ENCODER_REVERSED.store(false, Relaxed);
    STEPPER_POSITION_TARGET.store(0, Relaxed);
    COMMISSION_IN_FLIGHT.store(false, Relaxed);
    STATS_SEND_TIME.store(0, Relaxed);
    STATS_SEND_TIME_HIGH.store(0, Relaxed);
    // Signal tmc_control to drain stale requests from the pre-reset epoch
    TMC_QUEUE_FLUSH.store(true, Relaxed);
}

#[klipper_command]
pub fn finalize_config(ctx: &mut FirmwareCtx<'_>, crc: u32) {
    foci_klipper::commands::lifecycle::finalize_config(ctx.state, crc);
    defmt::info!("Config finalized, CRC: {:#010x}", crc);
}

#[klipper_command]
pub fn allocate_oids(_ctx: &mut FirmwareCtx<'_>, _count: u8) {
    foci_klipper::commands::lifecycle::allocate_oids(_count);
}

#[klipper_command(in_shutdown)]
pub fn emergency_stop(ctx: &mut FirmwareCtx<'_>) {
    if enter_shutdown(ctx.state) {
        ::ankyra::klipper_shutdown!("Command request", read_tim5());
    }
}

/// Reset the MCU. Matches Klipper's `command_reset` (armcm_reset.c).
///
/// Writes `CANBOOT_BYPASS` to Katapult's request signature so the
/// bootloader skips straight to the application after reset. This is
/// used by `FIRMWARE_RESTART`.
#[klipper_command(in_shutdown)]
pub fn reset(_ctx: &mut FirmwareCtx<'_>) {
    katapult_reset(KATAPULT_BYPASS);
}

#[klipper_command(in_shutdown)]
pub fn debug_nop(_ctx: &mut FirmwareCtx<'_>) {}
