//! OpenFFBoard USB trace stream integration.

#![cfg(feature = "trace")]

use core::sync::atomic::Ordering;

use foci_firmware::trace_stream::FrameBuf;

use crate::state::usb_trace::{QueuedFrame, TRACE_COUNTERS, current_tx, event_queue, motion_queue};

/// Product string used by trace builds.
pub(crate) const TRACE_PRODUCT: &str = "FOCI OpenFFBoard (trace)";

fn compute_boot_id() -> u32 {
    const UID_BASE: *const u32 = 0x1FFF_7A10 as *const u32;
    // SAFETY: UID registers are always readable at this address on STM32F4.
    let uid0 = unsafe { core::ptr::read_volatile(UID_BASE) };
    // SAFETY: UID registers are always readable at this address on STM32F4.
    let uid1 = unsafe { core::ptr::read_volatile(UID_BASE.add(1)) };
    // SAFETY: UID registers are always readable at this address on STM32F4.
    let uid2 = unsafe { core::ptr::read_volatile(UID_BASE.add(2)) };
    let tim5 = crate::helpers::lifecycle::read_tim5();
    let dwt = cortex_m::peripheral::DWT::cycle_count();
    uid0 ^ uid1 ^ uid2 ^ tim5 ^ dwt
}

/// Mark the trace stream configured after USB configuration.
pub(crate) fn note_usb_configured() {
    TRACE_COUNTERS
        .boot_id
        .store(compute_boot_id(), Ordering::Relaxed);
    TRACE_COUNTERS.configured.store(true, Ordering::Relaxed);
}

/// Return true when trace producers should emit samples.
pub(crate) fn is_configured() -> bool {
    TRACE_COUNTERS.configured.load(Ordering::Relaxed) && foci_usb_stm32::is_configured()
}

/// Enqueue a motion/counter frame without blocking.
#[allow(dead_code)]
pub(crate) fn enqueue_motion(frame: FrameBuf) {
    // SAFETY: Trace producers enqueue complete frames; the USB task is the
    // only consumer. RTIC priorities prevent overlapping mutable queue use for
    // this initial single-producer path.
    unsafe {
        let queue = motion_queue();
        if queue.enqueue(QueuedFrame::new(frame)).is_err() {
            TRACE_COUNTERS.motion_drops.fetch_add(1, Ordering::Relaxed);
            TRACE_COUNTERS
                .motion_drop_gap
                .store(true, Ordering::Relaxed);
        }
        let len = queue.len() as u16;
        let mut current = TRACE_COUNTERS.high_water.load(Ordering::Relaxed);
        while len > current {
            match TRACE_COUNTERS.high_water.compare_exchange_weak(
                current,
                len,
                Ordering::Relaxed,
                Ordering::Relaxed,
            ) {
                Ok(_) => break,
                Err(next) => current = next,
            }
        }
    }
}

/// Enqueue an event frame without blocking.
#[allow(dead_code)]
pub(crate) fn enqueue_event(frame: FrameBuf) {
    // SAFETY: Trace event producers enqueue complete frames; the USB task is
    // the only consumer in this initial wiring slice.
    unsafe {
        if event_queue().enqueue(QueuedFrame::new(frame)).is_err() {
            TRACE_COUNTERS.event_drops.fetch_add(1, Ordering::Relaxed);
            TRACE_COUNTERS.event_drop_gap.store(true, Ordering::Relaxed);
        }
    }
}

/// Drain at most one 64-byte trace packet to USB.
pub(crate) fn drain_once() {
    // SAFETY: This is the only trace consumer. Producers only enqueue complete
    // `QueuedFrame` values. `current_tx` is therefore single-consumer state and
    // preserves partially sent frames across calls.
    unsafe {
        let current_tx = current_tx();
        if current_tx.is_none() {
            *current_tx = motion_queue().dequeue().or_else(|| event_queue().dequeue());
        }
        let Some(current) = current_tx.as_mut() else {
            return;
        };
        let written = {
            let bytes = current.frame.bytes();
            let offset = current.offset as usize;
            foci_usb_stm32::trace_write(&bytes[offset..])
        };
        if written > 0 {
            current.offset = current.offset.saturating_add(written as u16);
            if current.offset as usize >= current.frame.bytes().len() {
                *current_tx = None;
            }
        } else {
            TRACE_COUNTERS
                .endpoint_not_ready
                .fetch_add(1, Ordering::Relaxed);
        }
    }
}
