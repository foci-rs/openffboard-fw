//! OpenFFBoard USB trace stream integration.

#![cfg(feature = "trace")]

use core::sync::atomic::{AtomicBool, AtomicI32, AtomicU16, AtomicU32, Ordering};

use foci_firmware::trace_stream::FrameBuf;

use crate::state::usb_trace::{QueuedFrame, TRACE_COUNTERS, current_tx, event_queue, motion_queue};

/// Product string used by trace builds.
pub(crate) const TRACE_PRODUCT: &str = "FOCI OpenFFBoard (trace)";

struct TraceClockExt {
    last_low: u32,
    high: u64,
}

const TIM5_WRAP_DISAMBIGUATION_THRESHOLD: u32 = 1 << 31;
const CRITICAL_HZ: u32 = 1000;
const CONTROL_DETAIL_HZ: u32 = 500;
const STATUS_HZ: u32 = 10;

static mut TRACE_CLOCK_EXT: TraceClockExt = TraceClockExt {
    last_low: 0,
    high: 0,
};

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

fn store_split_u64(low: &AtomicU32, high: &AtomicU32, value: u64) {
    cortex_m::interrupt::free(|_| {
        low.store(value as u32, Ordering::Relaxed);
        high.store((value >> 32) as u32, Ordering::Relaxed);
    });
}

fn load_split_u64(low: &AtomicU32, high: &AtomicU32) -> u64 {
    cortex_m::interrupt::free(|_| {
        u64::from(low.load(Ordering::Relaxed)) | (u64::from(high.load(Ordering::Relaxed)) << 32)
    })
}

/// Extend the 32-bit TIM5 Klipper clock into the u64 trace timestamp domain.
///
/// This must be called from the 1 ms tick before emitting frames. It may also
/// be called from P2/P3 diagnostics. It is protected by a short critical
/// section because every trace timestamp in a session must be monotonic. The
/// half-window wrap heuristic assumes this function is called at least once
/// per TIM5 wrap; the 1 ms tick satisfies that during normal operation and USB
/// configuration is expected well before the first 51.1 s OpenFFBoard F407
/// TIM5 wrap at the 84 MHz APB1 timer clock.
pub(crate) fn trace_now_ticks() -> u64 {
    let low = crate::helpers::lifecycle::read_tim5();
    extend_tim5_low(low)
}

/// Extend a previously sampled TIM5 low word into the trace timestamp domain.
pub(crate) fn extend_tim5_low(low: u32) -> u64 {
    cortex_m::interrupt::free(|_| unsafe {
        let ext = &mut *core::ptr::addr_of_mut!(TRACE_CLOCK_EXT);
        let last_low = ext.last_low;
        if low < last_low && last_low.wrapping_sub(low) > TIM5_WRAP_DISAMBIGUATION_THRESHOLD {
            ext.high = ext.high.wrapping_add(1u64 << 32);
            ext.last_low = low;
        } else if low >= last_low {
            ext.last_low = low;
        }
        ext.high | u64::from(low)
    })
}

/// Record the trace boot timestamp after TIM5 starts.
pub(crate) fn note_trace_boot() {
    let boot = trace_now_ticks();
    store_split_u64(
        &TRACE_COUNTERS.boot_tick_low,
        &TRACE_COUNTERS.boot_tick_high,
        boot,
    );
}

fn note_trace_usb_configured_time() {
    let configured = trace_now_ticks();
    let boot = load_split_u64(
        &TRACE_COUNTERS.boot_tick_low,
        &TRACE_COUNTERS.boot_tick_high,
    );
    let ticks_per_millisecond = u64::from(crate::ankyra::constants::CLOCK_FREQ / 1000);
    let elapsed_ticks = configured.saturating_sub(boot);
    let elapsed_ms = (elapsed_ticks / ticks_per_millisecond).min(u64::from(u32::MAX));
    store_split_u64(
        &TRACE_COUNTERS.configured_tick_low,
        &TRACE_COUNTERS.configured_tick_high,
        configured,
    );
    TRACE_COUNTERS
        .pre_config_elapsed_ms
        .store(elapsed_ms as u32, Ordering::Relaxed);
    TRACE_COUNTERS.pre_config_skipped_critical.store(
        ((elapsed_ms * u64::from(CRITICAL_HZ)) / 1000).min(u64::from(u32::MAX)) as u32,
        Ordering::Relaxed,
    );
    TRACE_COUNTERS.pre_config_skipped_control_detail.store(
        ((elapsed_ms * u64::from(CONTROL_DETAIL_HZ)) / 1000).min(u64::from(u32::MAX)) as u32,
        Ordering::Relaxed,
    );
    TRACE_COUNTERS.pre_config_skipped_status.store(
        ((elapsed_ms * u64::from(STATUS_HZ)) / 1000).min(u64::from(u32::MAX)) as u32,
        Ordering::Relaxed,
    );
}

/// Mark the trace stream configured after USB configuration.
pub(crate) fn note_usb_configured() {
    note_trace_usb_configured_time();
    TRACE_COUNTERS
        .boot_id
        .store(compute_boot_id(), Ordering::Relaxed);
    TRACE_COUNTERS.configured.store(true, Ordering::Relaxed);
}

/// Trace timing diagnostics.
#[allow(dead_code)]
pub(crate) struct TraceTiming {
    pub(crate) max_step_jitter_ticks: AtomicU32,
    pub(crate) min_deadline_slack_ticks: AtomicI32,
    pub(crate) last_served_step_tick_low: AtomicU32,
    pub(crate) last_served_step_tick_high: AtomicU32,
    pub(crate) last_klipper_command_service_tick_low: AtomicU32,
    pub(crate) last_klipper_command_service_tick_high: AtomicU32,
    pub(crate) missed_or_deferred_steps: AtomicU32,
    pub(crate) klipper_rx_queue_depth: AtomicU16,
    pub(crate) spi_error_count: AtomicU32,
    pub(crate) status_interrupt_count: AtomicU32,
    pub(crate) suppressed_status_change_count: AtomicU32,
    pub(crate) pending_status_change: AtomicBool,
    pub(crate) last_status_change_emit_tick: AtomicU32,
    pub(crate) first_spi_sample_cycles: AtomicU32,
    pub(crate) max_spi_sample_cycles: AtomicU32,
    pub(crate) first_spi_duration_reported: AtomicBool,
    pub(crate) skip_next_control_detail: AtomicBool,
    pub(crate) trace_deadline_pressure_count: AtomicU32,
}

impl TraceTiming {
    /// Create zeroed trace timing counters.
    pub(crate) const fn new() -> Self {
        Self {
            max_step_jitter_ticks: AtomicU32::new(0),
            min_deadline_slack_ticks: AtomicI32::new(i32::MAX),
            last_served_step_tick_low: AtomicU32::new(0),
            last_served_step_tick_high: AtomicU32::new(0),
            last_klipper_command_service_tick_low: AtomicU32::new(0),
            last_klipper_command_service_tick_high: AtomicU32::new(0),
            missed_or_deferred_steps: AtomicU32::new(0),
            klipper_rx_queue_depth: AtomicU16::new(0),
            spi_error_count: AtomicU32::new(0),
            status_interrupt_count: AtomicU32::new(0),
            suppressed_status_change_count: AtomicU32::new(0),
            pending_status_change: AtomicBool::new(false),
            last_status_change_emit_tick: AtomicU32::new(0),
            first_spi_sample_cycles: AtomicU32::new(0),
            max_spi_sample_cycles: AtomicU32::new(0),
            first_spi_duration_reported: AtomicBool::new(false),
            skip_next_control_detail: AtomicBool::new(false),
            trace_deadline_pressure_count: AtomicU32::new(0),
        }
    }
}

pub(crate) static TRACE_TIMING: TraceTiming = TraceTiming::new();

/// Store the last served step tick.
pub(crate) fn store_last_served_step_tick(ticks: u64) {
    store_split_u64(
        &TRACE_TIMING.last_served_step_tick_low,
        &TRACE_TIMING.last_served_step_tick_high,
        ticks,
    );
}

/// Load the last served step tick.
#[allow(dead_code)]
pub(crate) fn load_last_served_step_tick() -> u64 {
    load_split_u64(
        &TRACE_TIMING.last_served_step_tick_low,
        &TRACE_TIMING.last_served_step_tick_high,
    )
}

/// Store the last Klipper command-service tick.
pub(crate) fn store_last_klipper_command_service_tick(ticks: u64) {
    store_split_u64(
        &TRACE_TIMING.last_klipper_command_service_tick_low,
        &TRACE_TIMING.last_klipper_command_service_tick_high,
        ticks,
    );
}

/// Load the last Klipper command-service tick.
#[allow(dead_code)]
pub(crate) fn load_last_klipper_command_service_tick() -> u64 {
    load_split_u64(
        &TRACE_TIMING.last_klipper_command_service_tick_low,
        &TRACE_TIMING.last_klipper_command_service_tick_high,
    )
}

/// Record a TMC SPI error for status frames.
#[allow(dead_code)]
pub(crate) fn record_spi_error() {
    TRACE_TIMING.spi_error_count.fetch_add(1, Ordering::Relaxed);
    TRACE_TIMING
        .pending_status_change
        .store(true, Ordering::Relaxed);
}

/// Record a TMC STATUS pin interrupt.
pub(crate) fn record_status_interrupt() {
    TRACE_TIMING
        .status_interrupt_count
        .fetch_add(1, Ordering::Relaxed);
    TRACE_TIMING
        .pending_status_change
        .store(true, Ordering::Relaxed);
}

/// Record measured SPI sampling duration.
#[allow(dead_code)]
pub(crate) fn record_spi_sample_cycles(cycles: u32) {
    // DWT cycles are CPU-clock cycles. This threshold is sized for the 168 MHz
    // F407 CPU clock and is intentionally not derived from CLOCK_FREQ, which
    // is the 84 MHz TIM5/Klipper clock.
    const TRACE_DETAIL_SKIP_THRESHOLD_CYCLES: u32 = 120_000;

    let mut current = TRACE_TIMING.max_spi_sample_cycles.load(Ordering::Relaxed);
    while cycles > current {
        match TRACE_TIMING.max_spi_sample_cycles.compare_exchange_weak(
            current,
            cycles,
            Ordering::Relaxed,
            Ordering::Relaxed,
        ) {
            Ok(_) => break,
            Err(next) => current = next,
        }
    }
    let _ = TRACE_TIMING.first_spi_sample_cycles.compare_exchange(
        0,
        cycles,
        Ordering::Relaxed,
        Ordering::Relaxed,
    );
    if cycles > TRACE_DETAIL_SKIP_THRESHOLD_CYCLES {
        TRACE_TIMING
            .skip_next_control_detail
            .store(true, Ordering::Relaxed);
        TRACE_TIMING
            .trace_deadline_pressure_count
            .fetch_add(1, Ordering::Relaxed);
        TRACE_TIMING
            .pending_status_change
            .store(true, Ordering::Relaxed);
    }
}

/// Record timing for a served step-timer deadline.
pub(crate) fn record_step_timing(scheduled: u32, actual: u32) {
    let jitter = actual.wrapping_sub(scheduled) as i32;
    let abs = jitter.unsigned_abs();
    let mut current = TRACE_TIMING.max_step_jitter_ticks.load(Ordering::Relaxed);
    while abs > current {
        match TRACE_TIMING.max_step_jitter_ticks.compare_exchange_weak(
            current,
            abs,
            Ordering::Relaxed,
            Ordering::Relaxed,
        ) {
            Ok(_) => break,
            Err(next) => current = next,
        }
    }
    let slack = scheduled.wrapping_sub(actual) as i32;
    let mut min = TRACE_TIMING
        .min_deadline_slack_ticks
        .load(Ordering::Relaxed);
    while slack < min {
        match TRACE_TIMING.min_deadline_slack_ticks.compare_exchange_weak(
            min,
            slack,
            Ordering::Relaxed,
            Ordering::Relaxed,
        ) {
            Ok(_) => break,
            Err(next) => min = next,
        }
    }
    store_last_served_step_tick(extend_tim5_low(actual));
}

/// Take per-window step jitter and deadline slack accumulators.
#[allow(dead_code)]
pub(crate) fn take_jitter_and_slack() -> (u32, i32) {
    let jitter = TRACE_TIMING
        .max_step_jitter_ticks
        .swap(0, Ordering::Relaxed);
    let slack = TRACE_TIMING
        .min_deadline_slack_ticks
        .swap(i32::MAX, Ordering::Relaxed);
    let normalized_slack = if slack == i32::MAX { 0 } else { slack };
    (jitter, normalized_slack)
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
