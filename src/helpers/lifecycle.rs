//! Lifecycle helpers: Klipper clock reads, shutdown entry, and
//! Katapult bootloader reset. Promoted from `mod app` in `main.rs` by
//! Task 8c so handlers (migrating in Tasks 8e–8l) and the RTIC task
//! graph can share a single implementation.

use core::sync::atomic::Ordering::Relaxed;

use foci_klipper::state::State;

use crate::state::lifecycle::SHUTDOWN_ACTIVE;
use crate::state::tmc::TMC_DISABLE_SIGNAL;
use crate::state::trsync::{HOMING_STATE, HOMING_TRIGGERED, REPORT_PENDING};
use crate::trigger_domain::{
    PHYSICAL_STEPPER_INDICES, stop_physical_stepper_no_lock, with_trigger_domain,
};

/// Katapult bootloader vector table base (from Klipper `armcm_reset.c`).
pub(crate) const KATAPULT_BOOT_ADDRESS: u32 = 0x0800_0000;
/// Signature at `bl_vectors[reset_handler - 9]` when a Katapult
/// bootloader is resident.
pub(crate) const KATAPULT_SIGNATURE: u64 = 0x2174_6f6f_426e_6143; // "CanBoot!"
/// Request signature that keeps the MCU in the bootloader after reset
/// (used by klipper-usb's 1200-baud DTR reflash path).
#[allow(dead_code)]
pub(crate) const KATAPULT_REQUEST: u64 = 0x5984_E3FA_6CA1_589B;
/// Request signature that skips the bootloader on next boot
/// (used for `FIRMWARE_RESTART`).
pub(crate) const KATAPULT_BYPASS: u64 = 0x7b06_ec45_a9a8_243d;

/// Read the TIM5 counter (Klipper clock at 84 MHz).
pub(crate) fn read_tim5() -> u32 {
    embassy_stm32::pac::TIM5.cnt().read()
}

/// Escalate the firmware to shutdown state and quiesce the trigger
/// domain.
///
/// Returns `true` if this call actually entered shutdown (first call).
/// Returns `false` if already in shutdown. The caller should send
/// `klipper_shutdown!` only when this returns `true`, so each call
/// site provides a unique static string reason.
pub(crate) fn enter_shutdown(context: &mut State) -> bool {
    let entered = foci_klipper::commands::lifecycle::enter_shutdown(context);
    if entered {
        TMC_DISABLE_SIGNAL.signal(());
        SHUTDOWN_ACTIVE.store(true, Relaxed);
        // Quiesce TriggerDomain: cancel timers, stop steppers
        with_trigger_domain(|td| {
            td.scheduler.clear();
            let tim5 = embassy_stm32::pac::TIM5;
            tim5.dier().modify(|w| w.set_ccie(1, false));
            // Stop physical steppers
            for &idx in PHYSICAL_STEPPER_INDICES {
                stop_physical_stepper_no_lock(idx);
                td.set_timer_active(idx, false);
            }
            // Clear transient stepper state
            for ss in td.stepper_stop.iter_mut() {
                ss.need_reset = true;
                ss.blocked_by_trigger = false;
                ss.blocked_by_trsync = None;
            }
            // Clear transient trsync state (preserve OID maps).
            // Trigger each trsync to clear can_trigger per spec,
            // then cancel timers and pending reports.
            for (trsync_idx, ts) in td.trsync.iter_mut().enumerate() {
                // Find OID for this index (if configured)
                let oid = td
                    .trsync_by_oid
                    .iter()
                    .find(|&(_, &idx)| idx == trsync_idx)
                    .map(|(&oid, _)| oid)
                    .unwrap_or(0);
                ts.trigger(oid, 0, 0); // Sets can_trigger=false
                ts.cancel_timers();
                ts.clear_pending_report();
            }
            // Clear stop signals and endstop homing
            for sig in td.stop_signals.iter_mut() {
                *sig = None;
            }
            for es in td.endstops.iter_mut() {
                es.reset();
            }
        });
        REPORT_PENDING.store(false, Relaxed);
        HOMING_TRIGGERED.store(0xFF, Relaxed);
        HOMING_STATE.store(0, Relaxed);
    }
    entered
}

/// Write a Katapult request signature and perform a system reset.
///
/// `KATAPULT_REQUEST`: stay in bootloader (for reflashing via 1200-baud DTR).
/// `KATAPULT_BYPASS`: skip bootloader, boot application (for FIRMWARE_RESTART).
///
/// Matches Klipper's `canboot_reset()` in armcm_reset.c. Verifies the
/// bootloader signature before writing, no-ops if no bootloader is present.
pub(crate) fn katapult_reset(req_signature: u64) {
    // SAFETY: The bootloader vector table at KATAPULT_BOOT_ADDRESS is
    // always readable. We verify the Katapult signature before writing.
    unsafe {
        let bl_vectors = KATAPULT_BOOT_ADDRESS as *const u32;
        let reset_handler = core::ptr::read_volatile(bl_vectors.add(1));
        let boot_sig_addr = (reset_handler - 9) as *const u64;
        let req_sig_addr = core::ptr::read_volatile(bl_vectors) as *mut u64;

        // Verify alignment and signature
        if !(boot_sig_addr as usize).is_multiple_of(8)
            || core::ptr::read_volatile(boot_sig_addr) != KATAPULT_SIGNATURE
            || !(req_sig_addr as usize).is_multiple_of(8)
        {
            // No valid bootloader — just reset
            cortex_m::peripheral::SCB::sys_reset();
        }

        cortex_m::interrupt::disable();
        core::ptr::write_volatile(req_sig_addr, req_signature);
    }

    cortex_m::peripheral::SCB::sys_reset();
}
