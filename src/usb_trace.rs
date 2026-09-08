//! OpenFFBoard USB trace stream integration.

#![cfg(feature = "trace")]

use core::sync::atomic::{AtomicI32, AtomicU8, AtomicU32, Ordering};

use foci_firmware::trace::events::{SUBSYSTEM_USB_TRACE, TraceClockExt};
use foci_firmware::trace::pressure::{TracePressureRuntime, TracePressureStatusInputs};
use foci_firmware::trace::sampling::{
    TraceSampleInputs, TraceSamplePlan, commissioning_trace_sample_plan,
};
use foci_firmware::trace::stream::{FrameBuf, frame_sequence};
use foci_firmware::trace::transport::{self, TraceQueueSelection};

use crate::state::usb_trace::{
    MOTION_QUEUE_DEPTH, PendingEvent, QueuedFrame, TRACE_COUNTERS, current_tx, event_queue,
    motion_consumer, motion_producer, pending_events,
};

/// Product string used by trace builds.
pub(crate) const TRACE_PRODUCT: &str = "FOCI OpenFFBoard (trace)";

static TRACE_PRESSURE: TracePressureRuntime = TracePressureRuntime::new();

/// Enable the lower-rate raw current-path diagnostic frame.
const CURRENT_PATH_DIAGNOSTIC_ENABLED: bool = cfg!(feature = "trace-current-path");

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

const CRITICAL_HZ: u32 = 1000;
const CONTROL_DETAIL_HZ: u32 = 500;
const STATUS_HZ: u32 = 10;
const TRACE_CHANNEL_COUNT: usize = 1;
const STATUS_SLOT_COUNT: usize = transport::status_slot_count(TRACE_CHANNEL_COUNT);

static mut TRACE_CLOCK_EXT: TraceClockExt = TraceClockExt::new();

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
        ext.extend_low(low)
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
        foci_firmware::trace::events::EVENT_LEVEL_INFO,
        foci_firmware::trace::events::SUBSYSTEM_TMC_CONTROL,
        EVENT_FIRMWARE_BOOT,
        [0; 4],
        0,
    );
    request_event(
        foci_firmware::trace::events::EVENT_LEVEL_INFO,
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
) -> foci_firmware::trace::stream::FrameHeader {
    header_at_timestamp(
        frame_type,
        payload_len,
        source_kind,
        source_index,
        flags,
        trace_now_ticks(),
    )
}

fn header_at_timestamp(
    frame_type: u8,
    payload_len: usize,
    source_kind: u8,
    source_index: u8,
    flags: u8,
    timestamp_ticks: u64,
) -> foci_firmware::trace::stream::FrameHeader {
    foci_firmware::trace::stream::FrameHeader {
        frame_type,
        flags,
        payload_len: payload_len as u16,
        source_kind,
        source_index,
        sequence: TRACE_COUNTERS.next_sequence(),
        timestamp_ticks,
        boot_id: TRACE_COUNTERS.boot_id.load(Ordering::Relaxed),
    }
}

fn motion_flags() -> u8 {
    if TRACE_COUNTERS
        .motion_drop_gap
        .swap(false, Ordering::Relaxed)
    {
        foci_firmware::trace::stream::FLAG_DROP_GAP_BEFORE_THIS_FRAME
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
    let timestamp_ticks = trace_now_ticks();
    let event = PendingEvent {
        timestamp_ticks,
        level,
        subsystem_id,
        event_id,
        args,
        arg_count,
    };
    // O(1) critical section: push one fixed-size event or count one drop.
    let queued = cortex_m::interrupt::free(|_| unsafe {
        let queue = pending_events();
        if queue.push_back(event).is_err() {
            TRACE_COUNTERS.event_drops.fetch_add(1, Ordering::Relaxed);
            TRACE_COUNTERS.event_drop_gap.store(true, Ordering::Relaxed);
            false
        } else {
            true
        }
    });
    if queued {
        crate::transport::USB_WAKE.signal(());
    }
}

pub(crate) fn request_commission_event(
    source_index: u8,
    event: foci_firmware::trace::events::CommissionTraceEvent,
) {
    request_event(
        event.level as u8,
        foci_firmware::trace::events::SUBSYSTEM_COMMISSIONING,
        event.event_id,
        [
            i32::from(source_index),
            i32::from(foci_firmware::trace::events::engine_kind_wire_code(
                event.engine_kind,
            )),
            event.payload[0],
            event.payload[1],
        ],
        2 + event.payload_count.min(2),
    );
}

fn enqueue_event_frame(event: PendingEvent) {
    use foci_firmware::trace::stream::*;

    let sample = EventSample {
        level: event.level,
        subsystem_id: event.subsystem_id,
        event_id: event.event_id,
        arg_count: event.arg_count,
        args: event.args,
    };
    let flags = if TRACE_COUNTERS.event_drop_gap.swap(false, Ordering::Relaxed) {
        FLAG_DROP_GAP_BEFORE_THIS_FRAME
    } else {
        0
    };
    let frame = transport::build_event_frame(
        header_at_timestamp(
            FRAME_TYPE_EVENT,
            EVENT_PAYLOAD_LEN,
            SOURCE_KIND_BOARD,
            SOURCE_INDEX_NA,
            flags,
            event.timestamp_ticks,
        ),
        sample,
    );
    enqueue_event(frame);
}

/// Convert pending multi-producer event requests into event frames.
pub(crate) fn drain_pending_events() {
    loop {
        let event = cortex_m::interrupt::free(|_| unsafe {
            let queue = pending_events();
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
    if let Some(event) = TRACE_PRESSURE.take_spi_duration_event() {
        request_event(
            event.level,
            event.subsystem_id,
            event.event_id,
            event.args,
            event.arg_count,
        );
    }
}

/// Trace timing diagnostics.
pub(crate) struct TraceTiming {
    pub(crate) max_step_jitter_ticks: AtomicU32,
    pub(crate) min_deadline_slack_ticks: AtomicI32,
    pub(crate) last_served_step_tick_low: AtomicU32,
    pub(crate) last_served_step_tick_high: AtomicU32,
    pub(crate) missed_or_deferred_steps: AtomicU32,
}

impl TraceTiming {
    /// Create zeroed trace timing counters.
    pub(crate) const fn new() -> Self {
        Self {
            max_step_jitter_ticks: AtomicU32::new(0),
            min_deadline_slack_ticks: AtomicI32::new(i32::MAX),
            last_served_step_tick_low: AtomicU32::new(0),
            last_served_step_tick_high: AtomicU32::new(0),
            missed_or_deferred_steps: AtomicU32::new(0),
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
    TRACE_PRESSURE.store_last_command_service_tick(ticks);
}

/// Load the last Klipper command-service tick.
pub(crate) fn load_last_klipper_command_service_tick() -> u64 {
    TRACE_PRESSURE.last_command_service_tick()
}

/// Record the latest successful step queue depth for trace samples.
pub(crate) fn record_step_queue_depth(queue_len: usize) {
    TRACE_PRESSURE.record_queue_depth(queue_len);
}

/// Record a TMC SPI error for status frames.
pub(crate) fn record_spi_error() {
    TRACE_PRESSURE.record_spi_error();
}

/// Record a TMC STATUS pin interrupt.
pub(crate) fn record_status_interrupt() {
    TRACE_PRESSURE.record_status_interrupt();
}

/// Record measured SPI sampling duration.
pub(crate) fn record_spi_sample_cycles(cycles: u32) {
    // DWT cycles are CPU-clock cycles. This threshold is sized for the 168 MHz
    // F407 CPU clock and is intentionally not derived from CLOCK_FREQ, which
    // is the 84 MHz TIM5/Klipper clock.
    const TRACE_DETAIL_SKIP_THRESHOLD_CYCLES: u32 = 120_000;

    TRACE_PRESSURE.record_spi_sample_cycles(cycles, TRACE_DETAIL_SKIP_THRESHOLD_CYCLES);
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

/// Emit stream metadata.
pub(crate) fn emit_metadata() {
    use core::sync::atomic::Ordering::Relaxed;
    use foci_firmware::trace::stream::*;

    if !is_configured() {
        return;
    }

    let Ok(sources) = transport::canonical_sources(TRACE_CHANNEL_COUNT as u8) else {
        return;
    };
    let Ok(payload) = transport::build_metadata_payload(transport::TraceMetadataInputs {
        skipped_sample_counter_count: 3,
        clock_freq: crate::ankyra::constants::CLOCK_FREQ,
        critical_hz: CRITICAL_HZ,
        control_detail_hz: CONTROL_DETAIL_HZ,
        status_hz: STATUS_HZ,
        pre_configuration_elapsed_ms: TRACE_COUNTERS.pre_config_elapsed_ms.load(Relaxed),
        pre_configuration_skipped_critical: TRACE_COUNTERS
            .pre_config_skipped_critical
            .load(Relaxed),
        pre_configuration_skipped_control_detail: TRACE_COUNTERS
            .pre_config_skipped_control_detail
            .load(Relaxed),
        pre_configuration_skipped_status: TRACE_COUNTERS.pre_config_skipped_status.load(Relaxed),
        firmware_id: "openffboard-fw",
        product: TRACE_PRODUCT,
        serial: trace_serial(),
        sources: sources.as_slice(),
    }) else {
        return;
    };

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

/// Emit a compact runtime phase-advance diagnostic frame.
pub(crate) fn emit_phase_advance(
    phase_advance_counts: i16,
    phase_advance_flags: u8,
    gain_ppm: i32,
    max_counts: u16,
    deadband: u16,
) {
    use foci_firmware::trace::stream::*;

    if !is_configured() {
        return;
    }

    let sample = PhaseAdvanceSample {
        phase_advance_counts,
        phase_advance_flags,
        reserved: 0,
        gain_ppm,
        max_counts,
        deadband,
    };
    let mut payload = [0u8; PHASE_ADVANCE_PAYLOAD_LEN];
    encode_phase_advance_payload(&sample, &mut payload);
    let mut frame = FrameBuf::new();
    frame.write_header(header(
        FRAME_TYPE_PHASE_ADVANCE,
        PHASE_ADVANCE_PAYLOAD_LEN,
        SOURCE_KIND_FOCI,
        0,
        motion_flags(),
    ));
    frame
        .payload_mut(PHASE_ADVANCE_PAYLOAD_LEN)
        .copy_from_slice(&payload);
    enqueue_motion(frame);
}

/// Enqueue one characterization-only raw velocity observation.
pub(crate) fn request_velocity_observation(
    channel: u8,
    sample: foci_firmware::trace::stream::VelocityTraceObservation,
) {
    use foci_firmware::trace::stream::*;

    if !is_configured() {
        return;
    }

    let mut payload = [0u8; VELOCITY_OBSERVATION_PAYLOAD_LEN];
    encode_velocity_observation_payload(&sample, &mut payload);
    let mut frame = FrameBuf::new();
    frame.write_header(header(
        FRAME_TYPE_VELOCITY_OBSERVATION,
        VELOCITY_OBSERVATION_PAYLOAD_LEN,
        SOURCE_KIND_FOCI,
        channel,
        motion_flags(),
    ));
    frame
        .payload_mut(VELOCITY_OBSERVATION_PAYLOAD_LEN)
        .copy_from_slice(&payload);
    enqueue_motion(frame);
}

/// Enqueue one canonical velocity-sweep record for offline audit.
pub(crate) fn request_velocity_sweep_evidence(
    channel: u8,
    record: foci_firmware::ankyra::velocity_sweep_encoding::VelocitySweepCanonicalRecord,
) {
    use foci_firmware::trace::stream::*;

    if !is_configured() {
        return;
    }

    let payload = record.as_bytes();
    let mut frame = FrameBuf::new();
    frame.write_header(header(
        FRAME_TYPE_VELOCITY_SWEEP_EVIDENCE,
        payload.len(),
        SOURCE_KIND_FOCI,
        channel,
        motion_flags(),
    ));
    frame.payload_mut(payload.len()).copy_from_slice(payload);
    enqueue_motion(frame);
}

/// Enqueue a motion/counter frame without blocking.
pub(crate) fn enqueue_motion(frame: FrameBuf) {
    // SAFETY: TMC control is the sole motion-frame producer. The USB task owns
    // the separately split consumer endpoint.
    unsafe {
        let producer = motion_producer();
        if producer.enqueue(QueuedFrame::new(frame)).is_err() {
            TRACE_COUNTERS.motion_drops.fetch_add(1, Ordering::Relaxed);
            TRACE_COUNTERS
                .motion_drop_gap
                .store(true, Ordering::Relaxed);
        } else {
            crate::transport::USB_WAKE.signal(());
        }
        let len = producer.len() as u16;
        transport::update_high_water(&TRACE_COUNTERS.high_water, len);
    }
}

/// Enqueue an event frame without blocking.
pub(crate) fn enqueue_event(frame: FrameBuf) {
    // SAFETY: Pending events are converted, enqueued, and consumed only by the
    // priority-2 USB task.
    unsafe {
        if event_queue().enqueue(QueuedFrame::new(frame)).is_err() {
            TRACE_COUNTERS.event_drops.fetch_add(1, Ordering::Relaxed);
            TRACE_COUNTERS.event_drop_gap.store(true, Ordering::Relaxed);
        } else {
            crate::transport::USB_WAKE.signal(());
        }
    }
}

fn motion_queue_len() -> usize {
    // SAFETY: This runs in the P1 producer context.
    unsafe { motion_producer().len() }
}

fn dequeue_next_frame() -> Option<QueuedFrame> {
    // SAFETY: The USB task is the only trace consumer and the only event-queue
    // owner. The motion producer is isolated behind its split SPSC endpoint.
    unsafe {
        let motion = motion_consumer();
        let event = event_queue();
        let motion_seq = motion.peek().map(|queued| frame_sequence(&queued.frame));
        let event_seq = event.peek().map(|queued| frame_sequence(&queued.frame));

        match transport::select_next_queue(motion_seq, event_seq) {
            TraceQueueSelection::Motion => motion.dequeue(),
            TraceQueueSelection::Event => event.dequeue(),
            TraceQueueSelection::None => None,
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
            *sample_flags |= foci_firmware::trace::stream::SAMPLE_FLAG_SPI_ERROR;
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
            *sample_flags |= foci_firmware::trace::stream::SAMPLE_FLAG_SPI_ERROR;
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

/// Board-local inputs for one OpenFFBoard trace sample.
#[derive(Clone, Copy)]
pub(crate) struct TraceSampleContext {
    /// Whether the runtime currently considers the motor enabled.
    pub(crate) motor_enabled: bool,
}

/// Sample critical and control-detail trace frames from the 1 ms control tick.
pub(crate) async fn sample_tick<B>(
    tmc: &mut foci_tmc4671::spi::Tmc4671Spi<B>,
    context: TraceSampleContext,
    inputs: TraceSampleInputs,
) where
    B: embedded_hal_async::spi::SpiDevice<u8>,
{
    use core::sync::atomic::Ordering::Relaxed;
    use foci_firmware::trace::stream::*;
    use foci_tmc4671::registers::*;

    if !is_configured() {
        return;
    }

    let spi_sample_start = cortex_m::peripheral::DWT::cycle_count();
    let stepper = foci_firmware::state::stepper::stepper_channel0();
    let (jitter, slack) = take_jitter_and_slack();
    let mut sample_flags = 0u8;

    let step_position = stepper.position.load(Relaxed);
    let position_actual =
        read_register_trace(tmc, PID_POSITION_ACTUAL, &mut sample_flags).await as i32;
    let position_error =
        read_sub_register_trace(tmc, PID_ERROR_PID_POSITION_ERROR, &mut sample_flags).await as i32;
    let pid_velocity_actual =
        read_register_trace(tmc, PID_VELOCITY_ACTUAL, &mut sample_flags).await as i32;
    let velocity_error =
        read_sub_register_trace(tmc, PID_ERROR_PID_VELOCITY_ERROR, &mut sample_flags).await as i32;
    let interim_pid_torque_target_actual = read_sub_register_trace(
        tmc,
        INTERIM_PID_TORQUE_TARGET_TORQUE_ACTUAL,
        &mut sample_flags,
    )
    .await;
    let foc_uq_ud = read_sub_register_trace(tmc, INTERIM_FOC_UQ_UD, &mut sample_flags).await;
    let foc_uq_ud_limited =
        read_sub_register_trace(tmc, INTERIM_FOC_UQ_UD_LIMITED, &mut sample_flags).await;

    let sample = CriticalSample {
        step_position,
        tmc_position_actual: position_actual,
        position_error,
        target_velocity: inputs.command_velocity_reference,
        pid_velocity_actual,
        cached_pid_velocity_offset: inputs.cached_velocity_offset,
        velocity_error,
        enable_and_direction: (if context.motor_enabled { 0x80 } else { 0 })
            | (u8::from(stepper.direction.load(Relaxed)) & 0x01),
        sample_flags,
        timer_queue_depth: TRACE_PRESSURE.latest_queue_depth(),
        step_timer_jitter: jitter,
        min_deadline_slack: slack,
        last_served_step_tick: load_last_served_step_tick(),
        last_klipper_command_service_tick: load_last_klipper_command_service_tick(),
        missed_or_deferred_step_counter: TRACE_TIMING.missed_or_deferred_steps.load(Relaxed),
        interim_pidin_target_torque: 0,
        interim_pid_torque_target_actual,
        interim_foc_uq_ud: foc_uq_ud,
        interim_foc_uq_ud_limited: foc_uq_ud_limited,
        cached_accel_torque_offset: inputs.cached_accel_torque_offset,
        accel_feedforward_gain_permille: inputs.accel_feedforward_gain_permille,
        accel_feedforward_flags: inputs.accel_feedforward_flags,
        cached_velocity_transient_offset: inputs.cached_velocity_transient_offset,
        cached_velocity_clamped: inputs.cached_velocity_clamped,
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

    if current_path_due(
        CURRENT_PATH_DIAGNOSTIC_ENABLED,
        inputs.control_tick,
        motion_queue_len(),
        MOTION_QUEUE_DEPTH,
    ) {
        let mut current_path_flags = 0u8;
        let interim_foc_iq_id =
            read_sub_register_trace(tmc, INTERIM_FOC_IQ_ID, &mut current_path_flags).await;
        let interim_foc_iwy_iux =
            read_sub_register_trace(tmc, INTERIM_FOC_IWY_IUX, &mut current_path_flags).await;
        let interim_adc_i1_i0 =
            read_sub_register_trace(tmc, INTERIM_ADC_I1_I0, &mut current_path_flags).await;
        let current_path = CurrentPathSample {
            sample_flags: current_path_flags,
            interim_foc_iq_id,
            interim_foc_iwy_iux,
            interim_adc_i1_i0,
        };
        let mut payload = [0u8; CURRENT_PATH_PAYLOAD_LEN];
        encode_current_path_payload(&current_path, &mut payload);
        let mut frame = FrameBuf::new();
        frame.write_header(header(
            FRAME_TYPE_CURRENT_PATH,
            CURRENT_PATH_PAYLOAD_LEN,
            SOURCE_KIND_FOCI,
            0,
            motion_flags(),
        ));
        frame
            .payload_mut(CURRENT_PATH_PAYLOAD_LEN)
            .copy_from_slice(&payload);
        enqueue_motion(frame);
    }

    let emit_control_detail = control_detail_due(
        inputs.control_tick,
        motion_queue_len(),
        MOTION_QUEUE_DEPTH,
        TRACE_PRESSURE.take_control_detail_skip(),
    );
    if emit_control_detail {
        let state = foci_firmware::state::tmc::tmc_channel0();
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
            pidout_uq_ud_limits: state.voltage_limit_cfg.load(Relaxed),
            pid_torque_flux_limits: state.load_current_limit(Relaxed).register_value(),
            pid_velocity_limit: state.velocity_limit_cfg.load(Relaxed),
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

pub(crate) async fn sample_commission_closed_loop_activation<B, R, D>(
    tmc: &mut foci_tmc4671::spi::Tmc4671Spi<B>,
    motor_enabled: bool,
    control_tick: u32,
    remaining_budget_us: R,
    past_deadline: D,
) where
    B: embedded_hal_async::spi::SpiDevice<u8>,
    R: Fn() -> u64,
    D: Fn() -> bool,
{
    let trace_configured = is_configured();
    let past_deadline = trace_configured && past_deadline();
    let remaining_budget_us = if trace_configured && !past_deadline {
        remaining_budget_us()
    } else {
        0
    };

    match commissioning_trace_sample_plan(
        trace_configured,
        past_deadline,
        remaining_budget_us,
        control_tick,
    ) {
        TraceSamplePlan::SkipUnconfigured => {}
        TraceSamplePlan::DropForBudget => {
            TRACE_COUNTERS.motion_drops.fetch_add(1, Ordering::Relaxed);
            TRACE_COUNTERS
                .motion_drop_gap
                .store(true, Ordering::Relaxed);
        }
        TraceSamplePlan::Sample(inputs) => {
            sample_tick(tmc, TraceSampleContext { motor_enabled }, inputs).await;
        }
    }
}

static mut LAST_STATUS: [transport::StatusSnapshot; STATUS_SLOT_COUNT] =
    [transport::StatusSnapshot::new(); STATUS_SLOT_COUNT];

fn emit_one_status(
    slot: usize,
    source_kind: u8,
    source_index: u8,
    sample: foci_firmware::trace::stream::StatusSample,
    force: bool,
) {
    use foci_firmware::trace::stream::*;

    let previous = unsafe {
        let ptr = core::ptr::addr_of!(LAST_STATUS).cast::<transport::StatusSnapshot>();
        core::ptr::read(ptr.add(slot))
    };
    if !transport::should_emit_status(&previous, source_kind, source_index, &sample, force) {
        return;
    }

    let frame = transport::build_status_frame(
        header(
            FRAME_TYPE_STATUS,
            STATUS_PAYLOAD_LEN,
            source_kind,
            source_index,
            motion_flags(),
        ),
        sample,
    );
    enqueue_motion(frame);
    unsafe {
        let ptr = core::ptr::addr_of_mut!(LAST_STATUS).cast::<transport::StatusSnapshot>();
        core::ptr::write(
            ptr.add(slot),
            transport::StatusSnapshot {
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
    use foci_firmware::trace::stream::*;
    use foci_tmc4671::registers::STATUS_FLAGS;

    if !is_configured() {
        return;
    }

    if !TRACE_PRESSURE.should_emit_status(control_tick, regular_tick, true) {
        return;
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
    let counters = TRACE_PRESSURE.status_counters(TracePressureStatusInputs {
        endpoint_not_ready_count: TRACE_COUNTERS.endpoint_not_ready.load(Relaxed),
        usb_trace_queue_high_water: TRACE_COUNTERS.high_water.load(Relaxed),
        dropped_motion_sample_count: TRACE_COUNTERS.motion_drops.load(Relaxed),
        dropped_event_log_count: TRACE_COUNTERS.event_drops.load(Relaxed),
        missed_or_deferred_step_count: TRACE_TIMING.missed_or_deferred_steps.load(Relaxed),
    });

    // Fields that do not apply to a source kind are encoded as zero. Board and
    // FOCI frames both carry the full STATUS_FLAGS value by design: the
    // duplicate bytes are cheaper than making host parsing source-specific.
    let board = StatusSample {
        status_flags,
        status_interrupt_count: counters.status_interrupt_count,
        clipping_flags,
        spi_error_count: counters.spi_error_count,
        watchdog_or_step_overrun_count: counters.watchdog_or_step_overrun_count,
        klipper_rx_queue_depth: counters.latest_queue_depth,
        usb_trace_queue_high_water: counters.usb_trace_queue_high_water,
        dropped_motion_sample_count: counters.dropped_motion_sample_count,
        dropped_event_log_count: counters.dropped_event_log_count,
        endpoint_not_ready_count: counters.endpoint_not_ready_count,
        suppressed_status_change_count: counters.suppressed_status_change_count,
        trace_deadline_pressure_count: counters.trace_deadline_pressure_count,
        max_spi_sample_cycles: counters.max_spi_sample_cycles,
    };
    let foci = StatusSample {
        status_flags,
        status_interrupt_count: counters.status_interrupt_count,
        clipping_flags,
        spi_error_count: counters.spi_error_count,
        watchdog_or_step_overrun_count: counters.watchdog_or_step_overrun_count,
        ..StatusSample::default()
    };
    let usb = StatusSample {
        usb_trace_queue_high_water: counters.usb_trace_queue_high_water,
        dropped_motion_sample_count: counters.dropped_motion_sample_count,
        dropped_event_log_count: counters.dropped_event_log_count,
        endpoint_not_ready_count: counters.endpoint_not_ready_count,
        suppressed_status_change_count: counters.suppressed_status_change_count,
        ..StatusSample::default()
    };
    let klipper = StatusSample {
        klipper_rx_queue_depth: counters.latest_queue_depth,
        ..StatusSample::default()
    };

    emit_one_status(
        transport::board_status_slot(),
        SOURCE_KIND_BOARD,
        SOURCE_INDEX_NA,
        board,
        regular_tick,
    );
    emit_one_status(
        transport::usb_status_slot(),
        SOURCE_KIND_USB,
        SOURCE_INDEX_NA,
        usb,
        false,
    );
    emit_one_status(
        transport::klipper_status_slot(),
        SOURCE_KIND_KLIPPER,
        SOURCE_INDEX_NA,
        klipper,
        false,
    );
    emit_one_status(
        transport::foci_status_slot(0),
        SOURCE_KIND_FOCI,
        0,
        foci,
        false,
    );
}

/// Drain at most one 64-byte trace packet to USB.
pub(crate) fn drain_once() {
    // SAFETY: This is the only trace consumer. Producers only enqueue complete
    // `QueuedFrame` values. `current_tx` is therefore single-consumer state and
    // preserves partially sent frames across calls.
    unsafe {
        let current_tx = current_tx();
        if current_tx.is_none() {
            *current_tx = dequeue_next_frame();
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
