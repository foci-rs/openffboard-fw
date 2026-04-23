//! Stepper handlers: `config_stepper`, `queue_step`, `set_next_step_dir`,
//! `reset_step_clock`, `stepper_get_position`,
//! `stepper_get_commanded_position`, `stepper_stop_on_trigger`.
//!
//! Ported from `mod app` in `main.rs` by Task 8g of the anchor → ankyra
//! migration. Each handler takes `&mut FirmwareCtx<'_>` as its context
//! and accesses the underlying Klipper `State` via `ctx.state`.

use ankyra::prelude::klipper_command;

use crate::firmware_ctx::FirmwareCtx;
use crate::helpers::lifecycle::{enter_shutdown, read_tim5};
use crate::helpers::stepper::arm_step_timer;
use crate::state::stepper::{ISR_STEPPER, P2_STEPPER, STEP_QUEUE, STEPPER_POSITION};
use crate::state::tmc::MOTOR_ARMED;
use crate::state::trsync::HOMING_STATE;
use crate::trigger_domain::with_trigger_domain;

/// Register a stepper motor object.
///
/// Pin parameters are accepted for protocol compatibility but ignored;
/// the TMC4671 handles stepping internally.
#[klipper_command]
pub fn config_stepper(
    ctx: &mut FirmwareCtx<'_>,
    oid: u8,
    _step_pin: u8,
    _dir_pin: u8,
    _invert_step: u8,
    _step_pulse_ticks: u32,
) {
    if let Err(e) = foci_klipper::commands::stepper::config_stepper(ctx.state, oid) {
        defmt::error!("config_stepper: {}", e);
        if enter_shutdown(ctx.state) {
            ::ankyra::klipper_shutdown!("config_stepper failed", read_tim5());
        }
        return;
    }
    defmt::info!("config_stepper: oid={} (pins ignored, TMC4671 mode)", oid);
}

/// Enqueue a trapezoidal move segment for a stepper.
///
/// Routes through the hardware step/dir SPSC queue. Moves are consumed
/// by the TIM5 CC1 ISR (P3) which toggles STEP/DIR GPIO pins.
#[klipper_command]
pub fn queue_step(ctx: &mut FirmwareCtx<'_>, oid: u8, interval: u32, count: u16, add: i16) {
    use foci_klipper::commands::stepper::{QueueStepResult, queue_step_hw};

    let m = foci_klipper::stepper::StepMove::new(interval, count, add);

    // Lock TriggerDomain across need_reset check, direction state,
    // enqueue, and arm — all atomic per spec.
    let result = with_trigger_domain(|td| {
        let need_reset = td.stepper_stop[0].need_reset;

        // SAFETY: P2 is the sole producer. Queue len read is safe from P2.
        let queue_was_empty = unsafe { (*core::ptr::addr_of_mut!(STEP_QUEUE)).is_empty() };

        match queue_step_hw(need_reset, count, queue_was_empty) {
            Ok(QueueStepResult::Discarded) => Ok(()),
            Err(e) => Err(e),
            Ok(_result) => {
                // Prepare move with direction state inside the lock.
                // Must happen after need_reset check to avoid advancing
                // direction bookkeeping for discarded moves.
                // SAFETY: P2_STEPPER is only accessed from P2 command handlers.
                let m = unsafe { (*core::ptr::addr_of_mut!(P2_STEPPER)).prepare_move(m) };

                // Push to SPSC queue
                // SAFETY: P2 is the sole producer.
                let ok = unsafe { (*core::ptr::addr_of_mut!(STEP_QUEUE)).enqueue(m).is_ok() };
                if !ok {
                    return Err(foci_klipper::commands::CommandError::QueueFull);
                }

                // Arm timer if the motor is ready and not already active
                if !td.timer_active(0) && MOTOR_ARMED.load(core::sync::atomic::Ordering::Relaxed) {
                    let now = read_tim5();
                    arm_step_timer(now.wrapping_add(1));
                    td.set_timer_active(0, true);
                }
                Ok(())
            }
        }
    });

    if let Err(e) = result {
        defmt::error!("queue_step: oid={} {}", oid, e);
        if enter_shutdown(ctx.state) {
            ::ankyra::klipper_shutdown!("queue_step failed", read_tim5());
        }
    }
}

/// Set the direction for subsequent steps on a stepper.
///
/// Direction is captured per-move in `queue_step` via `prepare_move()`.
#[klipper_command]
pub fn set_next_step_dir(_ctx: &mut FirmwareCtx<'_>, _oid: u8, dir: u8) {
    // SAFETY: P2_STEPPER is only accessed from P2 command handlers.
    unsafe {
        (*core::ptr::addr_of_mut!(P2_STEPPER)).set_next_step_dir(dir != 0);
    }
}

/// Reset the step clock reference for a stepper.
///
/// Only valid when the stepper is inactive (timer not armed).
/// Triggers shutdown if called while active (matches Klipper stepper.c).
#[klipper_command]
pub fn reset_step_clock(ctx: &mut FirmwareCtx<'_>, _oid: u8, clock: u32) {
    let result: Result<(), foci_klipper::commands::CommandError> = with_trigger_domain(|td| {
        foci_klipper::commands::stepper::reset_step_clock_hw(td.stepper_stop[0].timer_active)?;

        // Drain any stale items from a previous stop
        // SAFETY: Timer is not active (checked above), P3 is quiesced.
        unsafe { while (*core::ptr::addr_of_mut!(STEP_QUEUE)).dequeue().is_some() {} }

        // Set clock base on both stepper state halves.
        // SAFETY: reset_step_clock_hw accepted only because the timer is
        // inactive, so P3 is not using ISR_STEPPER. P2_STEPPER is only
        // accessed from P2 command handlers.
        unsafe {
            foci_klipper::commands::stepper::reset_stepper_clocks(
                &mut *core::ptr::addr_of_mut!(P2_STEPPER),
                &mut *core::ptr::addr_of_mut!(ISR_STEPPER),
                clock,
            );
        }

        // Only clear need_reset if not blocked by an active trigger.
        // When blocked_by_trigger is set, in-flight drip commands must
        // not re-enable stepping. The flag is cleared by the host's
        // trsync_trigger command which arrives after the host has
        // received the trsync_state notification.
        if !td.stepper_stop[0].blocked_by_trigger {
            td.stepper_stop[0].need_reset = false;
        }
        Ok(())
    });

    if let Err(e) = result {
        defmt::error!("reset_step_clock: {}", e);
        if enter_shutdown(ctx.state) {
            ::ankyra::klipper_shutdown!("Can't reset time when stepper active", read_tim5());
        }
    }
}

/// Report the current position of a stepper.
#[klipper_command]
pub fn stepper_get_position(_ctx: &mut FirmwareCtx<'_>, oid: u8) {
    let pos = STEPPER_POSITION.load(core::sync::atomic::Ordering::Relaxed);
    ::ankyra::klipper_reply!(
        crate::ankyra::replies::StepperPosition,
        oid: u8 = oid,
        pos: i32 = pos
    );
}

/// Report the commanded (target) position of a stepper.
///
/// For hardware step/dir this is equivalent to `stepper_get_position`
/// since the firmware tracks only the logical step count.
#[klipper_command]
pub fn stepper_get_commanded_position(_ctx: &mut FirmwareCtx<'_>, oid: u8) {
    let pos = STEPPER_POSITION.load(core::sync::atomic::Ordering::Relaxed);
    ::ankyra::klipper_reply!(
        crate::ankyra::replies::StepperPosition,
        oid: u8 = oid,
        pos: i32 = pos
    );
}

/// Link a stepper to a TrSync so the stepper stops when the TrSync triggers.
///
/// Multiple steppers can be linked to the same TrSync. When a trigger fires,
/// all linked steppers are stopped.
#[klipper_command]
pub fn stepper_stop_on_trigger(ctx: &mut FirmwareCtx<'_>, oid: u8, trsync_oid: u8) {
    // Resolve stepper OID to index via State (outside critical section)
    let stepper_idx = match ctx.state.steppers_by_oid.get(&oid) {
        Some(&idx) => idx,
        None => {
            defmt::error!("stepper_stop_on_trigger: unknown stepper oid={}", oid);
            if enter_shutdown(ctx.state) {
                ::ankyra::klipper_shutdown!(
                    "stepper_stop_on_trigger: unknown stepper",
                    read_tim5()
                );
            }
            return;
        }
    };

    let result = with_trigger_domain(|td| {
        // Validate trsync OID exists
        if !td.trsync_by_oid.contains_key(&trsync_oid) {
            return Err(());
        }
        // Reject if this stepper already has an active stop signal,
        // matching Klipper C where each stepper has a single stop_signal
        // slot and trsync_add_signal() shuts down if already active.
        for signal in td.stop_signals.iter().flatten() {
            if signal.stepper_idx == stepper_idx {
                return Err(());
            }
        }
        // Find empty slot and insert
        for slot in td.stop_signals.iter_mut() {
            if slot.is_none() {
                *slot = Some(foci_klipper::trigger::StopSignal {
                    trsync_oid,
                    stepper_idx,
                });
                return Ok(());
            }
        }
        Err(()) // No free slots
    });

    if result.is_err() {
        defmt::error!(
            "stepper_stop_on_trigger: oid={} trsync_oid={} failed",
            oid,
            trsync_oid,
        );
        if enter_shutdown(ctx.state) {
            ::ankyra::klipper_shutdown!("stepper_stop_on_trigger failed", read_tim5());
        }
    } else {
        // Begin homing I-suppression: transition Idle -> SuppressPending.
        // HOMING_TRIGGERED is NOT set here — it is set by
        // TriggerDomain::trigger() when the endstop/expire fires.
        use core::sync::atomic::Ordering::Relaxed;
        let _ = HOMING_STATE.compare_exchange(0, 1, Relaxed, Relaxed);
    }
}
