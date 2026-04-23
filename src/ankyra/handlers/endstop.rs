//! Endstop handlers: `config_endstop`, `endstop_home`,
//! `endstop_query_state`.
//!
//! Ported from `mod app` in `main.rs` by Task 8h of the anchor → ankyra
//! migration. Each handler takes `&mut FirmwareCtx<'_>` as its context
//! and accesses the underlying Klipper `State` via `ctx.state`.

use ankyra::prelude::klipper_command;

use crate::firmware_ctx::FirmwareCtx;
use crate::helpers::lifecycle::{enter_shutdown, read_tim5};
use crate::helpers::trsync::{GPIO_MAX_PIN_VALUE, gpio_configure_input, gpio_read_pin};
use crate::trigger_domain::with_trigger_domain;
use foci_firmware::state::trsync::ENDSTOP_PIN_IDS;

/// Register an endstop object.
///
/// Allocates the next free Endstop slot and maps the Klipper OID to it.
/// Configures the specified GPIO pin as input with optional pull-up
/// resistor for endstop switch reading.
#[klipper_command]
pub fn config_endstop(ctx: &mut FirmwareCtx<'_>, oid: u8, pin: u8, pull_up: u8) {
    let result = with_trigger_domain(|td| {
        if td.endstop_by_oid.contains_key(&oid) {
            return Err(());
        }
        let idx = td.endstop_by_oid.len();
        if idx >= foci_klipper::state::NUM_ENDSTOPS {
            return Err(());
        }
        td.endstop_by_oid.insert(oid, idx).map_err(|_| ())?;
        Ok(idx)
    });
    let idx = match result {
        Ok(idx) => idx,
        Err(()) => {
            defmt::error!("config_endstop: oid={}", oid);
            if enter_shutdown(ctx.state) {
                ::ankyra::klipper_shutdown!("config_endstop failed", read_tim5());
            }
            return;
        }
    };

    // Configure GPIO pin as input (only for real GPIO pins, not virtual)
    if pin < GPIO_MAX_PIN_VALUE {
        gpio_configure_input(pin, pull_up != 0);
        ENDSTOP_PIN_IDS[idx].store(pin, core::sync::atomic::Ordering::Relaxed);
        defmt::info!(
            "config_endstop: oid={} idx={} pin=P{}{}{}",
            oid,
            idx,
            ['A', 'B', 'C', 'D', 'E'][(pin / 16) as usize],
            pin % 16,
            if pull_up != 0 { " (pull-up)" } else { "" }
        );
    } else {
        defmt::info!(
            "config_endstop: oid={} idx={} pin={} (virtual, no GPIO)",
            oid,
            idx,
            pin
        );
    }
}

/// Arm an endstop for homing.
///
/// Configures sampling parameters and links to a TrSync for trigger
/// notification. Schedules the first endstop sample event via the
/// trigger scheduler (TIM5 CC2).
#[klipper_command]
#[allow(clippy::too_many_arguments)]
pub fn endstop_home(
    ctx: &mut FirmwareCtx<'_>,
    oid: u8,
    clock: u32,
    sample_ticks: u32,
    sample_count: u8,
    rest_ticks: u32,
    pin_value: u8,
    trsync_oid: u8,
    trigger_reason: u8,
) {
    let result = with_trigger_domain(|td| {
        let idx = match td.endstop_by_oid.get(&oid) {
            Some(&idx) => idx,
            None => return Err(()),
        };
        // Zero sample_count is a disarm request — no trsync needed
        if sample_count != 0 && !td.trsync_by_oid.contains_key(&trsync_oid) {
            return Err(());
        }
        // Look up pin_id from config-time storage
        let pin_id = ENDSTOP_PIN_IDS[idx].load(core::sync::atomic::Ordering::Relaxed);
        // Cancel existing scheduler entry for this endstop
        td.scheduler.cancel(|k| {
            matches!(k,
                foci_klipper::scheduler::SchedulerEventKind::EndstopSample(i) if *i == idx
            )
        });
        // Configure the endstop runtime
        td.endstops[idx].start_homing(
            clock,
            sample_ticks,
            sample_count,
            rest_ticks,
            pin_value != 0,
            trsync_oid,
            trigger_reason,
            pin_id,
        );
        // Schedule first sample event if homing
        if sample_count > 0 {
            td.scheduler
                .schedule(foci_klipper::scheduler::SchedulerEvent {
                    wake_clock: clock,
                    kind: foci_klipper::scheduler::SchedulerEventKind::EndstopSample(idx),
                });
        }
        td.reprogram_cc2();
        Ok(())
    });
    if result.is_err() {
        defmt::error!("endstop_home: oid={}", oid);
        if enter_shutdown(ctx.state) {
            ::ankyra::klipper_shutdown!("endstop_home failed", read_tim5());
        }
    }
}

/// Query the current state of an endstop.
///
/// Returns homing status, next sample clock, and current pin value.
#[klipper_command]
pub fn endstop_query_state(ctx: &mut FirmwareCtx<'_>, oid: u8) {
    // Brief lock to copy state
    let query = with_trigger_domain(|td| {
        let idx = match td.endstop_by_oid.get(&oid) {
            Some(&idx) => idx,
            None => return Err(()),
        };
        let (homing, next_clock) = td.endstops[idx].query_state();
        let pin_id = ENDSTOP_PIN_IDS[idx].load(core::sync::atomic::Ordering::Relaxed);
        Ok((homing, next_clock, pin_id))
    });
    match query {
        Ok((homing, next_clock, pin_id)) => {
            let pin_state = if pin_id != 0xFF {
                gpio_read_pin(pin_id)
            } else {
                false
            };
            ::ankyra::klipper_reply!(
                crate::ankyra::replies::EndstopState,
                oid: u8 = oid,
                homing: u8 = u8::from(homing),
                next_clock: u32 = next_clock,
                pin_value: u8 = u8::from(pin_state)
            );
        }
        Err(()) => {
            defmt::error!("endstop_query_state: unknown oid={}", oid);
            if enter_shutdown(ctx.state) {
                ::ankyra::klipper_shutdown!("endstop_query_state failed", read_tim5());
            }
        }
    }
}
