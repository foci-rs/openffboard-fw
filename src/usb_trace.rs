//! OpenFFBoard USB trace stream integration.

#![cfg(feature = "trace")]

use core::sync::atomic::{AtomicBool, AtomicI32, AtomicU8, AtomicU16, AtomicU32, Ordering};

use foci_firmware::trace_stream::{FrameBuf, METADATA_PAYLOAD_MAX};

use crate::state::usb_trace::{
    PENDING_EVENTS, PendingEvent, QueuedFrame, TRACE_COUNTERS, current_tx, event_queue,
    motion_queue,
};

/// Product string used by trace builds.
pub(crate) const TRACE_PRODUCT: &str = "FOCI OpenFFBoard (trace)";

pub(crate) const EVENT_LEVEL_INFO: u8 = 1;
#[allow(dead_code)]
pub(crate) const EVENT_LEVEL_WARN: u8 = 2;
pub(crate) const EVENT_LEVEL_ERROR: u8 = 3;

pub(crate) const SUBSYSTEM_TMC_CONTROL: u8 = 1;
#[allow(dead_code)]
pub(crate) const SUBSYSTEM_STEP_TIMER: u8 = 2;
pub(crate) const SUBSYSTEM_USB_TRACE: u8 = 3;
#[allow(dead_code)]
pub(crate) const SUBSYSTEM_KLIPPER: u8 = 4;
#[allow(dead_code)]
pub(crate) const SUBSYSTEM_HOMING_ENDSTOP: u8 = 5;
pub(crate) const SUBSYSTEM_SAFETY_SHUTDOWN: u8 = 6;
#[allow(dead_code)]
pub(crate) const SUBSYSTEM_COMMISSIONING: u8 = 7;

pub(crate) const EVENT_FIRMWARE_BOOT: u16 = 1;
pub(crate) const EVENT_USB_CONFIGURED: u16 = 2;
pub(crate) const EVENT_MOTOR_ARM: u16 = 3;
pub(crate) const EVENT_MOTOR_DISARM: u16 = 4;
#[allow(dead_code)]
pub(crate) const EVENT_HOMING_OR_ENDSTOP: u16 = 5;
#[allow(dead_code)]
pub(crate) const EVENT_GAIN_APPLIED: u16 = 6;
#[allow(dead_code)]
pub(crate) const EVENT_TRACE_QUEUE_OVERFLOW: u16 = 7;
#[allow(dead_code)]
pub(crate) const EVENT_SPI_ERROR: u16 = 8;
pub(crate) const EVENT_STATUS_INTERRUPT: u16 = 9;
#[allow(dead_code)]
pub(crate) const EVENT_SHUTDOWN: u16 = 10;
pub(crate) const EVENT_SPI_SAMPLE_DURATION: u16 = 11;

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

static TRACE_SERIAL_LEN: AtomicU8 = AtomicU8::new(0);
static mut TRACE_SERIAL: [u8; 31] = [0; 31];

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

/// Cache the USB serial string for metadata frames.
///
/// Call once during init before trace producers run. The metadata path reads
/// the cached bytes without an ownership handoff.
pub(crate) fn set_trace_serial(serial: &str) {
    let bytes = serial.as_bytes();
    let len = bytes.len().min(31);
    cortex_m::interrupt::free(|_| unsafe {
        let dst = core::ptr::addr_of_mut!(TRACE_SERIAL).cast::<u8>();
        core::ptr::copy_nonoverlapping(bytes.as_ptr(), dst, len);
        TRACE_SERIAL_LEN.store(len as u8, Ordering::Relaxed);
    });
}

fn trace_serial() -> &'static str {
    let len = TRACE_SERIAL_LEN.load(Ordering::Relaxed) as usize;
    cortex_m::interrupt::free(|_| unsafe {
        let src = core::ptr::addr_of!(TRACE_SERIAL).cast::<u8>();
        let bytes = core::slice::from_raw_parts(src, len);
        core::str::from_utf8_unchecked(bytes)
    })
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
    TRACE_COUNTERS
        .boot_id
        .store(compute_boot_id(), Ordering::Relaxed);
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
    TRACE_COUNTERS.configured.store(true, Ordering::Relaxed);
    request_event(
        EVENT_LEVEL_INFO,
        SUBSYSTEM_TMC_CONTROL,
        EVENT_FIRMWARE_BOOT,
        [0; 4],
        0,
    );
    request_event(
        EVENT_LEVEL_INFO,
        SUBSYSTEM_USB_TRACE,
        EVENT_USB_CONFIGURED,
        [0; 4],
        0,
    );
}

fn header(
    frame_type: u8,
    payload_len: usize,
    source_kind: u8,
    source_index: u8,
    flags: u8,
) -> foci_firmware::trace_stream::FrameHeader {
    foci_firmware::trace_stream::FrameHeader {
        frame_type,
        flags,
        payload_len: payload_len as u16,
        source_kind,
        source_index,
        sequence: TRACE_COUNTERS.next_sequence(),
        timestamp_ticks: trace_now_ticks(),
        boot_id: TRACE_COUNTERS.boot_id.load(Ordering::Relaxed),
    }
}

fn motion_flags() -> u8 {
    if TRACE_COUNTERS
        .motion_drop_gap
        .swap(false, Ordering::Relaxed)
    {
        foci_firmware::trace_stream::FLAG_DROP_GAP_BEFORE_THIS_FRAME
    } else {
        0
    }
}

/// Request a structured event from any RTIC context.
pub(crate) fn request_event(
    level: u8,
    subsystem_id: u8,
    event_id: u16,
    args: [i32; 4],
    arg_count: u8,
) {
    if !is_configured() {
        return;
    }
    let event = PendingEvent {
        level,
        subsystem_id,
        event_id,
        args,
        arg_count,
    };
    // O(1) critical section: push one fixed-size event or count one drop.
    cortex_m::interrupt::free(|_| unsafe {
        let queue = &mut *core::ptr::addr_of_mut!(PENDING_EVENTS);
        if queue.push_back(event).is_err() {
            TRACE_COUNTERS.event_drops.fetch_add(1, Ordering::Relaxed);
            TRACE_COUNTERS.event_drop_gap.store(true, Ordering::Relaxed);
        }
    });
}

fn enqueue_event_frame(event: PendingEvent) {
    use foci_firmware::trace_stream::*;

    let sample = EventSample {
        level: event.level,
        subsystem_id: event.subsystem_id,
        event_id: event.event_id,
        arg_count: event.arg_count,
        args: event.args,
    };
    let mut payload = [0u8; EVENT_PAYLOAD_LEN];
    encode_event_payload(&sample, &mut payload);
    let flags = if TRACE_COUNTERS.event_drop_gap.swap(false, Ordering::Relaxed) {
        FLAG_DROP_GAP_BEFORE_THIS_FRAME
    } else {
        0
    };
    let mut frame = FrameBuf::new();
    frame.write_header(header(
        FRAME_TYPE_EVENT,
        EVENT_PAYLOAD_LEN,
        SOURCE_KIND_BOARD,
        SOURCE_INDEX_NA,
        flags,
    ));
    frame
        .payload_mut(EVENT_PAYLOAD_LEN)
        .copy_from_slice(&payload);
    enqueue_event(frame);
}

/// Convert pending multi-producer event requests into event frames.
pub(crate) fn drain_pending_events() {
    loop {
        let event = cortex_m::interrupt::free(|_| unsafe {
            let queue = &mut *core::ptr::addr_of_mut!(PENDING_EVENTS);
            queue.pop_front()
        });
        let Some(event) = event else {
            break;
        };
        enqueue_event_frame(event);
    }
}

/// Emit a one-shot event with the measured trace SPI sample duration.
pub(crate) fn request_spi_duration_event_if_ready() {
    if TRACE_TIMING
        .first_spi_duration_reported
        .swap(true, Ordering::Relaxed)
    {
        return;
    }
    let cycles = TRACE_TIMING.first_spi_sample_cycles.load(Ordering::Relaxed);
    if cycles == 0 {
        TRACE_TIMING
            .first_spi_duration_reported
            .store(false, Ordering::Relaxed);
        return;
    }
    request_event(
        EVENT_LEVEL_INFO,
        SUBSYSTEM_TMC_CONTROL,
        EVENT_SPI_SAMPLE_DURATION,
        [
            cycles as i32,
            TRACE_TIMING.max_spi_sample_cycles.load(Ordering::Relaxed) as i32,
            0,
            0,
        ],
        2,
    );
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
pub(crate) fn load_last_klipper_command_service_tick() -> u64 {
    load_split_u64(
        &TRACE_TIMING.last_klipper_command_service_tick_low,
        &TRACE_TIMING.last_klipper_command_service_tick_high,
    )
}

/// Record a TMC SPI error for status frames.
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

fn push_str(out: &mut heapless::Vec<u8, METADATA_PAYLOAD_MAX>, value: &str) {
    let bytes = value.as_bytes();
    let len = bytes.len().min(31);
    let _ = out.push(len as u8);
    let _ = out.extend_from_slice(&bytes[..len]);
}

/// Emit stream metadata.
pub(crate) fn emit_metadata() {
    use core::sync::atomic::Ordering::Relaxed;
    use foci_firmware::trace_stream::*;

    if !is_configured() {
        return;
    }

    let mut payload: heapless::Vec<u8, METADATA_PAYLOAD_MAX> = heapless::Vec::new();
    let _ = payload.extend_from_slice(&[
        1, // metadata schema
        1, // trace feature revision
        1, // source count
        3, // skipped sample counter count
    ]);
    let _ = payload.extend_from_slice(&crate::ankyra::constants::CLOCK_FREQ.to_le_bytes());
    let _ = payload.extend_from_slice(&1000u32.to_le_bytes());
    let _ = payload.extend_from_slice(&500u32.to_le_bytes());
    let _ = payload.extend_from_slice(&10u32.to_le_bytes());
    let _ = payload.extend_from_slice(
        &TRACE_COUNTERS
            .pre_config_elapsed_ms
            .load(Relaxed)
            .to_le_bytes(),
    );
    let _ = payload.extend_from_slice(
        &TRACE_COUNTERS
            .pre_config_skipped_critical
            .load(Relaxed)
            .to_le_bytes(),
    );
    let _ = payload.extend_from_slice(
        &TRACE_COUNTERS
            .pre_config_skipped_control_detail
            .load(Relaxed)
            .to_le_bytes(),
    );
    let _ = payload.extend_from_slice(
        &TRACE_COUNTERS
            .pre_config_skipped_status
            .load(Relaxed)
            .to_le_bytes(),
    );
    push_str(&mut payload, "openffboard-fw");
    push_str(&mut payload, TRACE_PRODUCT);
    push_str(&mut payload, trace_serial());
    let _ = payload.extend_from_slice(&[SOURCE_KIND_FOCI, 0, 7]);
    let _ = payload.extend_from_slice(b"channel");

    let mut frame = FrameBuf::new();
    frame.write_header(header(
        FRAME_TYPE_METADATA,
        payload.len(),
        SOURCE_KIND_BOARD,
        SOURCE_INDEX_NA,
        0,
    ));
    frame.payload_mut(payload.len()).copy_from_slice(&payload);
    enqueue_motion(frame);
}

/// Enqueue a motion/counter frame without blocking.
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
        } else {
            crate::transport::USB_WAKE.signal(());
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
pub(crate) fn enqueue_event(frame: FrameBuf) {
    // SAFETY: Trace event producers enqueue complete frames; the USB task is
    // the only consumer in this initial wiring slice.
    unsafe {
        if event_queue().enqueue(QueuedFrame::new(frame)).is_err() {
            TRACE_COUNTERS.event_drops.fetch_add(1, Ordering::Relaxed);
            TRACE_COUNTERS.event_drop_gap.store(true, Ordering::Relaxed);
        } else {
            crate::transport::USB_WAKE.signal(());
        }
    }
}

async fn read_register_trace<B>(
    tmc: &mut foci_tmc4671::spi::Tmc4671Spi<B>,
    register: foci_tmc4671::registers::Register,
    sample_flags: &mut u8,
) -> u32
where
    B: embedded_hal_async::spi::SpiDevice<u8>,
{
    match tmc.read_register(register).await {
        Ok(value) => value,
        Err(_) => {
            record_spi_error();
            *sample_flags |= foci_firmware::trace_stream::SAMPLE_FLAG_SPI_ERROR;
            0
        }
    }
}

async fn read_sub_register_trace<B>(
    tmc: &mut foci_tmc4671::spi::Tmc4671Spi<B>,
    register: foci_tmc4671::registers::SubRegister,
    sample_flags: &mut u8,
) -> u32
where
    B: embedded_hal_async::spi::SpiDevice<u8>,
{
    match tmc.read_sub_register(register).await {
        Ok(value) => value,
        Err(_) => {
            record_spi_error();
            *sample_flags |= foci_firmware::trace_stream::SAMPLE_FLAG_SPI_ERROR;
            0
        }
    }
}

async fn read_register_counting_errors<B>(
    tmc: &mut foci_tmc4671::spi::Tmc4671Spi<B>,
    register: foci_tmc4671::registers::Register,
) -> u32
where
    B: embedded_hal_async::spi::SpiDevice<u8>,
{
    let mut ignored_flags = 0u8;
    read_register_trace(tmc, register, &mut ignored_flags).await
}

async fn read_sub_register_counting_errors<B>(
    tmc: &mut foci_tmc4671::spi::Tmc4671Spi<B>,
    register: foci_tmc4671::registers::SubRegister,
) -> u32
where
    B: embedded_hal_async::spi::SpiDevice<u8>,
{
    let mut ignored_flags = 0u8;
    read_sub_register_trace(tmc, register, &mut ignored_flags).await
}

/// Sample critical and control-detail trace frames from the 1 ms control tick.
pub(crate) async fn sample_tick<B>(
    tmc: &mut foci_tmc4671::spi::Tmc4671Spi<B>,
    motor_enabled: bool,
    cached_velocity_offset: i32,
    control_tick: u32,
) where
    B: embedded_hal_async::spi::SpiDevice<u8>,
{
    use core::sync::atomic::Ordering::Relaxed;
    use foci_firmware::trace_stream::*;
    use foci_tmc4671::registers::*;

    if !is_configured() {
        return;
    }

    let spi_sample_start = cortex_m::peripheral::DWT::cycle_count();
    let stepper = foci_firmware::state::stepper::stepper_channel0();
    let (jitter, slack) = take_jitter_and_slack();
    let mut sample_flags = 0u8;

    let position_actual =
        read_register_trace(tmc, PID_POSITION_ACTUAL, &mut sample_flags).await as i32;
    // PID_VELOCITY_TARGET exists in foci_tmc4671::registers at 0x66, matching
    // TMC4671-LA datasheet Rev 1, 02/2024, section 7 "PID Controller". Do not
    // use `StepperChannelState.interval` here; it is AtomicU32 timer ticks per
    // step, not a velocity.
    let pid_velocity_target =
        read_register_trace(tmc, PID_VELOCITY_TARGET, &mut sample_flags).await as i32;
    let pid_velocity_actual =
        read_register_trace(tmc, PID_VELOCITY_ACTUAL, &mut sample_flags).await as i32;
    let pid_torque_flux_target =
        read_register_trace(tmc, PID_TORQUE_FLUX_TARGET, &mut sample_flags).await;
    let pid_torque_flux_actual =
        read_register_trace(tmc, PID_TORQUE_FLUX_ACTUAL, &mut sample_flags).await;
    let foc_uq_ud = read_sub_register_trace(tmc, INTERIM_FOC_UQ_UD, &mut sample_flags).await;
    let foc_uq_ud_limited =
        read_sub_register_trace(tmc, INTERIM_FOC_UQ_UD_LIMITED, &mut sample_flags).await;
    let step_position = stepper.position.load(Relaxed);

    let sample = CriticalSample {
        step_position,
        tmc_position_actual: position_actual,
        position_error: step_position.wrapping_sub(position_actual),
        target_velocity: pid_velocity_target,
        pid_velocity_actual,
        cached_pid_velocity_offset: cached_velocity_offset,
        velocity_error: pid_velocity_target.wrapping_sub(pid_velocity_actual),
        enable_and_direction: (if motor_enabled { 0x80 } else { 0 })
            | (u8::from(stepper.direction.load(Relaxed)) & 0x01),
        sample_flags,
        timer_queue_depth: TRACE_TIMING.klipper_rx_queue_depth.load(Relaxed),
        step_timer_jitter: jitter,
        min_deadline_slack: slack,
        last_served_step_tick: load_last_served_step_tick(),
        last_klipper_command_service_tick: load_last_klipper_command_service_tick(),
        missed_or_deferred_step_counter: TRACE_TIMING.missed_or_deferred_steps.load(Relaxed),
        pid_torque_flux_target,
        pid_torque_flux_actual,
        interim_foc_uq_ud: foc_uq_ud,
        interim_foc_uq_ud_limited: foc_uq_ud_limited,
    };
    let mut payload = [0u8; CRITICAL_PAYLOAD_LEN];
    encode_critical_payload(&sample, &mut payload);
    let mut frame = FrameBuf::new();
    frame.write_header(header(
        FRAME_TYPE_CRITICAL,
        CRITICAL_PAYLOAD_LEN,
        SOURCE_KIND_FOCI,
        0,
        motion_flags(),
    ));
    frame
        .payload_mut(CRITICAL_PAYLOAD_LEN)
        .copy_from_slice(&payload);
    enqueue_motion(frame);

    let emit_control_detail = control_tick.is_multiple_of(2)
        && !TRACE_TIMING.skip_next_control_detail.swap(false, Relaxed);
    if emit_control_detail {
        let detail = ControlDetailSample {
            interim_pidout_target_torque: read_sub_register_counting_errors(
                tmc,
                INTERIM_PIDOUT_TARGET_TORQUE,
            )
            .await,
            interim_pidout_target_flux: read_sub_register_counting_errors(
                tmc,
                INTERIM_PIDOUT_TARGET_FLUX,
            )
            .await,
            interim_pidin_target_velocity: read_sub_register_counting_errors(
                tmc,
                INTERIM_PIDIN_TARGET_VELOCITY,
            )
            .await,
            interim_pidout_target_velocity: read_sub_register_counting_errors(
                tmc,
                INTERIM_PIDOUT_TARGET_VELOCITY,
            )
            .await,
            pidout_uq_ud_limits: read_register_counting_errors(tmc, PIDOUT_UQ_UD_LIMITS).await,
            pid_torque_flux_limits: read_register_counting_errors(tmc, PID_TORQUE_FLUX_LIMITS)
                .await,
            pid_velocity_limit: read_register_counting_errors(tmc, PID_VELOCITY_LIMIT).await,
        };
        let mut payload = [0u8; CONTROL_DETAIL_PAYLOAD_LEN];
        encode_control_detail_payload(&detail, &mut payload);
        let mut frame = FrameBuf::new();
        frame.write_header(header(
            FRAME_TYPE_CONTROL_DETAIL,
            CONTROL_DETAIL_PAYLOAD_LEN,
            SOURCE_KIND_FOCI,
            0,
            motion_flags(),
        ));
        frame
            .payload_mut(CONTROL_DETAIL_PAYLOAD_LEN)
            .copy_from_slice(&payload);
        enqueue_motion(frame);
    }

    let spi_sample_cycles = cortex_m::peripheral::DWT::cycle_count().wrapping_sub(spi_sample_start);
    record_spi_sample_cycles(spi_sample_cycles);
}

#[derive(Clone, Copy, PartialEq, Eq)]
struct StatusSnapshot {
    source_kind: u8,
    source_index: u8,
    sample: foci_firmware::trace_stream::StatusSample,
}

static mut LAST_STATUS: [StatusSnapshot; 4] = [StatusSnapshot {
    source_kind: 0,
    source_index: 0,
    sample: foci_firmware::trace_stream::StatusSample {
        status_flags: 0,
        status_interrupt_count: 0,
        clipping_flags: 0,
        spi_error_count: 0,
        watchdog_or_step_overrun_count: 0,
        klipper_rx_queue_depth: 0,
        usb_trace_queue_high_water: 0,
        dropped_motion_sample_count: 0,
        dropped_event_log_count: 0,
        endpoint_not_ready_count: 0,
        suppressed_status_change_count: 0,
    },
}; 4];

fn status_trigger_nonzero(sample: &foci_firmware::trace_stream::StatusSample) -> bool {
    sample.status_flags != 0
        || sample.status_interrupt_count != 0
        || sample.clipping_flags != 0
        || sample.spi_error_count != 0
        || sample.watchdog_or_step_overrun_count != 0
        || sample.dropped_motion_sample_count != 0
        || sample.dropped_event_log_count != 0
        || sample.endpoint_not_ready_count != 0
        || sample.suppressed_status_change_count != 0
}

fn status_trigger_changed(
    previous: &foci_firmware::trace_stream::StatusSample,
    next: &foci_firmware::trace_stream::StatusSample,
) -> bool {
    // Queue depth and queue high-water are intentionally omitted from the
    // change trigger so normal command traffic does not cause status spam.
    previous.status_flags != next.status_flags
        || previous.status_interrupt_count != next.status_interrupt_count
        || previous.clipping_flags != next.clipping_flags
        || previous.spi_error_count != next.spi_error_count
        || previous.watchdog_or_step_overrun_count != next.watchdog_or_step_overrun_count
        || previous.dropped_motion_sample_count != next.dropped_motion_sample_count
        || previous.dropped_event_log_count != next.dropped_event_log_count
        || previous.endpoint_not_ready_count != next.endpoint_not_ready_count
        || previous.suppressed_status_change_count != next.suppressed_status_change_count
}

fn emit_one_status(
    slot: usize,
    source_kind: u8,
    source_index: u8,
    sample: foci_firmware::trace_stream::StatusSample,
    force: bool,
) {
    use foci_firmware::trace_stream::*;

    let previous = unsafe {
        let ptr = core::ptr::addr_of!(LAST_STATUS).cast::<StatusSnapshot>();
        core::ptr::read(ptr.add(slot))
    };
    let changed = previous.source_kind != source_kind
        || previous.source_index != source_index
        || status_trigger_changed(&previous.sample, &sample);
    let should_emit = force || status_trigger_nonzero(&sample) || changed;
    if !should_emit {
        return;
    }

    let mut payload = [0u8; STATUS_PAYLOAD_LEN];
    encode_status_payload(&sample, &mut payload);
    let mut frame = FrameBuf::new();
    frame.write_header(header(
        FRAME_TYPE_STATUS,
        STATUS_PAYLOAD_LEN,
        source_kind,
        source_index,
        motion_flags(),
    ));
    frame
        .payload_mut(STATUS_PAYLOAD_LEN)
        .copy_from_slice(&payload);
    enqueue_motion(frame);
    unsafe {
        let ptr = core::ptr::addr_of_mut!(LAST_STATUS).cast::<StatusSnapshot>();
        core::ptr::write(
            ptr.add(slot),
            StatusSnapshot {
                source_kind,
                source_index,
                sample,
            },
        );
    }
}

/// Emit board, FOCI, USB, and Klipper status frames when due.
pub(crate) async fn emit_status_frames<B>(
    tmc: &mut foci_tmc4671::spi::Tmc4671Spi<B>,
    control_tick: u32,
    regular_tick: bool,
) where
    B: embedded_hal_async::spi::SpiDevice<u8>,
{
    use core::sync::atomic::Ordering::Relaxed;
    use foci_firmware::trace_stream::*;
    use foci_tmc4671::registers::STATUS_FLAGS;

    if !is_configured() {
        return;
    }

    let pending_change = TRACE_TIMING.pending_status_change.swap(false, Relaxed);
    if !regular_tick && !pending_change {
        return;
    }
    if pending_change {
        let previous = TRACE_TIMING.last_status_change_emit_tick.load(Relaxed);
        if control_tick.wrapping_sub(previous) < 10 {
            TRACE_TIMING
                .suppressed_status_change_count
                .fetch_add(1, Relaxed);
            return;
        }
        TRACE_TIMING
            .last_status_change_emit_tick
            .store(control_tick, Relaxed);
    }

    // TMC4671-LA datasheet Rev 1, 02/2024, section 5 "Safety Functions",
    // Table 30 defines STATUS_FLAGS (0x7C). It displays the actual status
    // flags; reads do not clear bits. Clearing requires a write, and the trace
    // path must never write this register. Table 30 marks bit 19 as reserved;
    // do not introduce a PLL-not-locked bit without separate silicon evidence.
    let status_flags = read_register_counting_errors(tmc, STATUS_FLAGS).await;
    const STATUS_PID_LIMIT_MASK: u32 = (1 << 0)
        | (1 << 2)
        | (1 << 3)
        | (1 << 4)
        | (1 << 6)
        | (1 << 7)
        | (1 << 8)
        | (1 << 10)
        | (1 << 11)
        | (1 << 12)
        | (1 << 14)
        | (1 << 15);
    const STATUS_IPARK_LIMIT_MASK: u32 = (1 << 16) | (1 << 17) | (1 << 18);
    const STATUS_PWM_LIMIT_MASK: u32 = (1 << 24) | (1 << 25);
    const STATUS_ADC_CLIP_MASK: u32 = (1 << 26) | (1 << 27);
    let clipping_flags = status_flags
        & (STATUS_PID_LIMIT_MASK
            | STATUS_IPARK_LIMIT_MASK
            | STATUS_PWM_LIMIT_MASK
            | STATUS_ADC_CLIP_MASK);
    let spi_errors = TRACE_TIMING.spi_error_count.load(Relaxed);
    let status_interrupts = TRACE_TIMING.status_interrupt_count.load(Relaxed);
    let missed = TRACE_TIMING.missed_or_deferred_steps.load(Relaxed);
    let trace_pressure = TRACE_TIMING.trace_deadline_pressure_count.load(Relaxed);
    let endpoint_or_trace_pressure = TRACE_COUNTERS
        .endpoint_not_ready
        .load(Relaxed)
        .saturating_add(trace_pressure);
    let suppressed = TRACE_TIMING.suppressed_status_change_count.load(Relaxed);

    // Fields that do not apply to a source kind are encoded as zero. Board and
    // FOCI frames both carry the full STATUS_FLAGS value by design: the
    // duplicate bytes are cheaper than making host parsing source-specific.
    // Trace deadline pressure is telemetry degradation, so it is reported with
    // endpoint-not-ready pressure instead of motion watchdog/step overruns.
    let board = StatusSample {
        status_flags,
        status_interrupt_count: status_interrupts,
        clipping_flags,
        spi_error_count: spi_errors,
        watchdog_or_step_overrun_count: missed,
        klipper_rx_queue_depth: TRACE_TIMING.klipper_rx_queue_depth.load(Relaxed),
        usb_trace_queue_high_water: TRACE_COUNTERS.high_water.load(Relaxed),
        dropped_motion_sample_count: TRACE_COUNTERS.motion_drops.load(Relaxed),
        dropped_event_log_count: TRACE_COUNTERS.event_drops.load(Relaxed),
        endpoint_not_ready_count: endpoint_or_trace_pressure,
        suppressed_status_change_count: suppressed,
    };
    let foci = StatusSample {
        status_flags,
        status_interrupt_count: status_interrupts,
        clipping_flags,
        spi_error_count: spi_errors,
        watchdog_or_step_overrun_count: missed,
        ..StatusSample::default()
    };
    let usb = StatusSample {
        usb_trace_queue_high_water: TRACE_COUNTERS.high_water.load(Relaxed),
        dropped_motion_sample_count: TRACE_COUNTERS.motion_drops.load(Relaxed),
        dropped_event_log_count: TRACE_COUNTERS.event_drops.load(Relaxed),
        endpoint_not_ready_count: endpoint_or_trace_pressure,
        suppressed_status_change_count: suppressed,
        ..StatusSample::default()
    };
    let klipper = StatusSample {
        klipper_rx_queue_depth: TRACE_TIMING.klipper_rx_queue_depth.load(Relaxed),
        ..StatusSample::default()
    };

    emit_one_status(0, SOURCE_KIND_BOARD, SOURCE_INDEX_NA, board, regular_tick);
    emit_one_status(1, SOURCE_KIND_FOCI, 0, foci, false);
    emit_one_status(2, SOURCE_KIND_USB, SOURCE_INDEX_NA, usb, false);
    emit_one_status(3, SOURCE_KIND_KLIPPER, SOURCE_INDEX_NA, klipper, false);
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
