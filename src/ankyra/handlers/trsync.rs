//! Trsync handlers: `config_trsync`, `trsync_set_timeout`,
//! `trsync_start`, `trsync_trigger`.
//!
//! Ported from `mod app` in `main.rs` by Task 8i of the anchor → ankyra
//! migration. Each handler takes `&mut FirmwareCtx<'_>` as its context
//! and accesses the underlying Klipper `State` via `ctx.state`.

use ankyra::prelude::klipper_command;

use crate::firmware_ctx::FirmwareCtx;
use crate::helpers::lifecycle::{enter_shutdown, read_tim5};
use crate::trigger_domain::with_trigger_domain;
use foci_firmware::state::trsync::{HOMING_STATE, HOMING_TRIGGERED};

/// Register a TrSync (trigger synchronization) object.
///
/// Allocates the next free TrSync slot and maps the Klipper OID to it.
#[klipper_command]
pub fn config_trsync(ctx: &mut FirmwareCtx<'_>, oid: u8) {
    let result = with_trigger_domain(|td| {
        if td.trsync_by_oid.contains_key(&oid) {
            return Err(());
        }
        let idx = td.trsync_by_oid.len();
        if idx >= foci_klipper::state::NUM_TRSYNC {
            return Err(());
        }
        td.trsync_by_oid.insert(oid, idx).map_err(|_| ())?;
        Ok(())
    });
    if result.is_err() {
        defmt::error!("config_trsync: oid={}", oid);
        if enter_shutdown(ctx.state) {
            ::ankyra::klipper_shutdown!("config_trsync failed", read_tim5());
        }
        return;
    }
    defmt::info!("config_trsync: oid={}", oid);
}

/// Arm a TrSync for synchronized triggering.
///
/// Configures the reporting interval and timeout reason, then transitions
/// the TrSync to the Armed state. Schedules the first report timer event.
#[klipper_command]
pub fn trsync_start(
    ctx: &mut FirmwareCtx<'_>,
    oid: u8,
    report_clock: u32,
    report_ticks: u32,
    expire_reason: u8,
) {
    let result = with_trigger_domain(|td| {
        let idx = match td.trsync_by_oid.get(&oid) {
            Some(&idx) => idx,
            None => return Err(()),
        };
        // Clear old stop signals for this trsync OID
        for slot in td.stop_signals.iter_mut() {
            if let Some(signal) = slot
                && signal.trsync_oid == oid
            {
                *slot = None;
            }
        }
        // Clear blocked_by_trigger for steppers blocked by this trsync
        for stepper in td.stepper_stop.iter_mut() {
            if stepper.blocked_by_trsync == Some(oid) {
                stepper.blocked_by_trigger = false;
                stepper.blocked_by_trsync = None;
            }
        }
        // Cancel any existing scheduler entries for this trsync
        td.scheduler.cancel(|k| {
            matches!(k,
                foci_klipper::scheduler::SchedulerEventKind::TrSyncExpire(i)
                | foci_klipper::scheduler::SchedulerEventKind::TrSyncReport(i)
                if *i == idx
            )
        });
        // Arm the runtime
        td.trsync[idx].arm(report_clock, report_ticks, expire_reason);
        // Schedule first report timer if report_ticks > 0
        if report_ticks > 0 {
            td.scheduler
                .schedule(foci_klipper::scheduler::SchedulerEvent {
                    wake_clock: report_clock,
                    kind: foci_klipper::scheduler::SchedulerEventKind::TrSyncReport(idx),
                });
        }
        td.reprogram_cc2();
        Ok(())
    });
    if result.is_err() {
        defmt::error!("trsync_start: oid={}", oid);
        if enter_shutdown(ctx.state) {
            ::ankyra::klipper_shutdown!("trsync_start failed", read_tim5());
        }
    }
}

/// Update the expiry deadline for a TrSync.
///
/// Schedules the expire timer event. The TrSync will auto-trigger with
/// the configured expire_reason if the clock reaches this value before
/// a manual trigger.
#[klipper_command]
pub fn trsync_set_timeout(ctx: &mut FirmwareCtx<'_>, oid: u8, clock: u32) {
    let result = with_trigger_domain(|td| {
        let idx = match td.trsync_by_oid.get(&oid) {
            Some(&idx) => idx,
            None => return Err(()),
        };
        if td.trsync[idx].can_trigger() {
            td.trsync[idx].set_timeout(clock);
            // Cancel any existing expire event and schedule a new one
            td.scheduler.cancel(|k| {
                matches!(k,
                    foci_klipper::scheduler::SchedulerEventKind::TrSyncExpire(i) if *i == idx
                )
            });
            td.scheduler
                .schedule(foci_klipper::scheduler::SchedulerEvent {
                    wake_clock: clock,
                    kind: foci_klipper::scheduler::SchedulerEventKind::TrSyncExpire(idx),
                });
            td.reprogram_cc2();
        }
        Ok(())
    });
    if result.is_err() {
        defmt::error!("trsync_set_timeout: oid={}", oid);
        if enter_shutdown(ctx.state) {
            ::ankyra::klipper_shutdown!("trsync_set_timeout failed", read_tim5());
        }
    }
}

/// Manually trigger a TrSync and clean up.
///
/// Mirrors Klipper C's `command_trsync_trigger`: attempts to fire the
/// trigger (no-op if already triggered), then unconditionally stops
/// report/expire timers, clears blocked_by_trigger on steppers, and
/// sends one final `trsync_state` reply with `can_trigger=0`.
#[klipper_command]
pub fn trsync_trigger(ctx: &mut FirmwareCtx<'_>, oid: u8, reason: u8) {
    let trigger_reason = with_trigger_domain(|td| {
        let idx = match td.trsync_by_oid.get(&oid) {
            Some(&idx) => idx,
            None => return Err(()),
        };
        // Attempt trigger via TriggerDomain::trigger() which dispatches
        // stop signals if this is the first trigger (no-op if already triggered)
        let _result = td.trigger(oid, reason, 0);
        let trigger_reason = td.trsync[idx].trigger_reason();

        // Cancel scheduler entries for this trsync
        td.scheduler.cancel(|k| {
            matches!(k,
                foci_klipper::scheduler::SchedulerEventKind::TrSyncExpire(i)
                | foci_klipper::scheduler::SchedulerEventKind::TrSyncReport(i)
                if *i == idx
            )
        });
        // Clear pending report to prevent stale async delivery
        td.trsync[idx].clear_pending_report();
        td.trsync[idx].cancel_timers();

        // Clear stop signals for this trsync OID
        for slot in td.stop_signals.iter_mut() {
            if let Some(signal) = slot
                && signal.trsync_oid == oid
            {
                *slot = None;
            }
        }

        // Clear blocked_by_trigger for steppers blocked by this trsync
        // (leaves need_reset true until reset_step_clock)
        for stepper in td.stepper_stop.iter_mut() {
            if stepper.blocked_by_trsync == Some(oid) {
                stepper.blocked_by_trigger = false;
                stepper.blocked_by_trsync = None;
            }
        }

        td.reprogram_cc2();
        Ok(trigger_reason)
    });

    match trigger_reason {
        Ok(trigger_reason) => {
            // Clear homing trigger sentinel and advance suppression
            // state machine. If P1 is in state 2 (Suppressing), the
            // host trsync_trigger cleanup must advance it to state 3
            // (RestorePending) so I-suppression is released even if
            // P1 hasn't polled HOMING_TRIGGERED yet.
            use core::sync::atomic::Ordering::Relaxed;
            HOMING_TRIGGERED.store(0xFF, Relaxed);
            let _ = HOMING_STATE.compare_exchange(2, 3, Relaxed, Relaxed);
            // Send final trsync_state reply (can_trigger=0, clock=0)
            ::ankyra::klipper_reply!(
                foci_firmware::ankyra::replies::TrsyncState,
                oid: u8 = oid,
                can_trigger: u8 = 0,
                trigger_reason: u8 = trigger_reason,
                clock: u32 = 0
            );
        }
        Err(()) => {
            defmt::error!("trsync_trigger: unknown oid={}", oid);
            if enter_shutdown(ctx.state) {
                ::ankyra::klipper_shutdown!("trsync_trigger failed", read_tim5());
            }
        }
    }
}
