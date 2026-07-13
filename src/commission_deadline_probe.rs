//! OpenFFBoard attribution for short commissioning deadline releases.

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use foci_firmware::autotune::PhaseId;
use foci_firmware::selftest::CommissionDiagnostic;
use foci_firmware::tmc_control::deadline_attribution::{
    DeadlineAttributionProbe, source_overlap_cycles,
};

const CYCLES_PER_MICROSECOND: u32 = 168;
const MAX_ATTRIBUTED_WAIT_US: u64 = 250;
const MAX_ATTRIBUTED_OVERDUE_US: u64 = 1_000;
const DIAG_DEADLINE_RELEASE: u8 = 40;
const DIAG_DEADLINE_HOPS: u8 = 41;
const WINDOW_FLAG: u32 = 1 << 31;
const NO_TIMER_WAIT_FLAG: u32 = 1 << 30;
const EXPIRED_BEFORE_SLEEP_FLAG: u32 = 1 << 29;
const PRE_ARM_SOURCE_FLAG: u32 = 1 << 28;

pub(crate) const SOURCE_WAIT_REQUEST: u32 = 1 << 1;
pub(crate) const SOURCE_USB_TASK: u32 = 1 << 2;
pub(crate) const SOURCE_TIM5_IRQ: u32 = 1 << 3;
pub(crate) const SOURCE_USB_IRQ: u32 = 1 << 4;
pub(crate) const SOURCE_FAULT_IRQ: u32 = 1 << 5;
pub(crate) const SOURCE_STATS_TASK: u32 = 1 << 6;
pub(crate) const SOURCE_BLINK_TASK: u32 = 1 << 7;
pub(crate) const SOURCE_STOP_CHECK: u32 = 1 << 8;
pub(crate) const SOURCE_ARM_CHECK: u32 = 1 << 9;
pub(crate) const SOURCE_REPORT_TASK: u32 = 1 << 10;
pub(crate) const SOURCE_STEPPER_DIAG_TASK: u32 = 1 << 11;
pub(crate) const SOURCE_TIM2_IRQ: u32 = 1 << 12;

static WAIT_PROBE: DeadlineAttributionProbe = DeadlineAttributionProbe::new();
static SERVICE_PROBE: DeadlineAttributionProbe = DeadlineAttributionProbe::new();
static CLASSIFICATION_WINDOW_ACTIVE: AtomicBool = AtomicBool::new(false);
static CLASSIFICATION_SOURCE_MASK: AtomicU32 = AtomicU32::new(0);
static EVENT_ACTIVE: AtomicBool = AtomicBool::new(false);
static EVENT_DUE_CYCLE: AtomicU32 = AtomicU32::new(0);
static EVENT_REQUESTED_WAIT_US: AtomicU32 = AtomicU32::new(0);
static EVENT_TIMER_RETURN_CYCLE: AtomicU32 = AtomicU32::new(0);
static EVENT_TIMER_LATENESS_US: AtomicU32 = AtomicU32::new(0);
static EVENT_POLL_ENTRY_CYCLE: AtomicU32 = AtomicU32::new(0);
static EVENT_LAST_ELAPSED_CYCLE: AtomicU32 = AtomicU32::new(0);
static EVENT_ENGINE_RETURN_CYCLE: AtomicU32 = AtomicU32::new(0);
static EVENT_POST_DEADLINE_SOURCE_MASK: AtomicU32 = AtomicU32::new(0);
static EVENT_PRE_ARM_SOURCE_PRESENT: AtomicBool = AtomicBool::new(false);
static EVENT_EXPIRED_BEFORE_SLEEP: AtomicBool = AtomicBool::new(false);
static WORST_ENTRY_LATENESS_CYCLES: AtomicU32 = AtomicU32::new(0);
static WORST_TIMER_LATENESS_US: AtomicU32 = AtomicU32::new(0);
static WORST_RETURN_TO_ENTRY_CYCLES: AtomicU32 = AtomicU32::new(0);
static WORST_ENTRY_TO_INNER_RETURN_CYCLES: AtomicU32 = AtomicU32::new(0);
static WORST_INNER_TO_ENGINE_RETURN_CYCLES: AtomicU32 = AtomicU32::new(0);
static WORST_ENGINE_RETURN_TO_CLASSIFY_CYCLES: AtomicU32 = AtomicU32::new(0);
static WORST_REQUESTED_WAIT_US: AtomicU32 = AtomicU32::new(0);
static WORST_POST_DEADLINE_SOURCE_MASK: AtomicU32 = AtomicU32::new(0);
static WORST_FLAGS: AtomicU32 = AtomicU32::new(0);

/// Reset retained attribution at the beginning of a commissioning run.
pub(crate) fn reset() {
    WAIT_PROBE.reset();
    SERVICE_PROBE.reset();
    CLASSIFICATION_WINDOW_ACTIVE.store(false, Ordering::Release);
    CLASSIFICATION_SOURCE_MASK.store(0, Ordering::Relaxed);
    EVENT_ACTIVE.store(false, Ordering::Release);
    EVENT_TIMER_RETURN_CYCLE.store(0, Ordering::Relaxed);
    EVENT_POLL_ENTRY_CYCLE.store(0, Ordering::Relaxed);
    EVENT_LAST_ELAPSED_CYCLE.store(0, Ordering::Relaxed);
    EVENT_ENGINE_RETURN_CYCLE.store(0, Ordering::Relaxed);
    EVENT_POST_DEADLINE_SOURCE_MASK.store(0, Ordering::Relaxed);
    EVENT_PRE_ARM_SOURCE_PRESENT.store(false, Ordering::Relaxed);
    WORST_ENTRY_LATENESS_CYCLES.store(0, Ordering::Relaxed);
    WORST_TIMER_LATENESS_US.store(0, Ordering::Relaxed);
    WORST_RETURN_TO_ENTRY_CYCLES.store(0, Ordering::Relaxed);
    WORST_ENTRY_TO_INNER_RETURN_CYCLES.store(0, Ordering::Relaxed);
    WORST_INNER_TO_ENGINE_RETURN_CYCLES.store(0, Ordering::Relaxed);
    WORST_ENGINE_RETURN_TO_CLASSIFY_CYCLES.store(0, Ordering::Relaxed);
    WORST_REQUESTED_WAIT_US.store(0, Ordering::Relaxed);
    WORST_POST_DEADLINE_SOURCE_MASK.store(0, Ordering::Relaxed);
    WORST_FLAGS.store(0, Ordering::Relaxed);
}

/// Record the first engine clock read after one absolute wait.
pub(crate) fn observe_elapsed(cycle: u32) {
    if !EVENT_ACTIVE.load(Ordering::Acquire) {
        return;
    }
    if EVENT_POLL_ENTRY_CYCLE
        .compare_exchange(0, cycle, Ordering::AcqRel, Ordering::Relaxed)
        .is_ok()
    {
        WAIT_PROBE.resume(cycle);
        SERVICE_PROBE.arm(cycle, 0);
    } else {
        EVENT_LAST_ELAPSED_CYCLE.store(cycle, Ordering::Relaxed);
    }
}

fn close_event(classify_cycle: u32, terminal: bool) {
    if !EVENT_ACTIVE.swap(false, Ordering::AcqRel) {
        return;
    }
    SERVICE_PROBE.resume(classify_cycle);

    let due_cycle = EVENT_DUE_CYCLE.load(Ordering::Relaxed);
    let poll_entry_cycle = EVENT_POLL_ENTRY_CYCLE.load(Ordering::Relaxed);
    if poll_entry_cycle == 0 {
        return;
    }
    let entry_lateness_cycles = poll_entry_cycle.wrapping_sub(due_cycle);
    if entry_lateness_cycles >= (1 << 31)
        || entry_lateness_cycles <= WORST_ENTRY_LATENESS_CYCLES.load(Ordering::Relaxed)
    {
        return;
    }

    let timer_return_cycle = EVENT_TIMER_RETURN_CYCLE.load(Ordering::Relaxed);
    let inner_return_cycle = EVENT_LAST_ELAPSED_CYCLE.load(Ordering::Relaxed);
    let engine_return_cycle = EVENT_ENGINE_RETURN_CYCLE.load(Ordering::Relaxed);
    let return_to_entry_cycles = if timer_return_cycle == 0 {
        0
    } else {
        poll_entry_cycle.wrapping_sub(timer_return_cycle)
    };
    let entry_to_inner_return_cycles = if inner_return_cycle == 0 {
        0
    } else {
        inner_return_cycle.wrapping_sub(poll_entry_cycle)
    };
    let inner_to_engine_return_cycles = if inner_return_cycle == 0 || engine_return_cycle == 0 {
        0
    } else {
        engine_return_cycle.wrapping_sub(inner_return_cycle)
    };
    let engine_return_to_classify_cycles = if engine_return_cycle == 0 {
        0
    } else {
        classify_cycle.wrapping_sub(engine_return_cycle)
    };
    let mut flags = if terminal { WINDOW_FLAG } else { 0 };
    if timer_return_cycle == 0 {
        flags |= NO_TIMER_WAIT_FLAG;
    }
    if EVENT_EXPIRED_BEFORE_SLEEP.load(Ordering::Relaxed) {
        flags |= EXPIRED_BEFORE_SLEEP_FLAG;
    }
    if EVENT_PRE_ARM_SOURCE_PRESENT.load(Ordering::Relaxed) {
        flags |= PRE_ARM_SOURCE_FLAG;
    }

    WORST_TIMER_LATENESS_US.store(
        EVENT_TIMER_LATENESS_US.load(Ordering::Relaxed),
        Ordering::Relaxed,
    );
    WORST_RETURN_TO_ENTRY_CYCLES.store(return_to_entry_cycles, Ordering::Relaxed);
    WORST_ENTRY_TO_INNER_RETURN_CYCLES.store(entry_to_inner_return_cycles, Ordering::Relaxed);
    WORST_INNER_TO_ENGINE_RETURN_CYCLES.store(inner_to_engine_return_cycles, Ordering::Relaxed);
    WORST_ENGINE_RETURN_TO_CLASSIFY_CYCLES
        .store(engine_return_to_classify_cycles, Ordering::Relaxed);
    WORST_REQUESTED_WAIT_US.store(
        EVENT_REQUESTED_WAIT_US.load(Ordering::Relaxed),
        Ordering::Relaxed,
    );
    WORST_POST_DEADLINE_SOURCE_MASK.store(
        EVENT_POST_DEADLINE_SOURCE_MASK.load(Ordering::Relaxed),
        Ordering::Relaxed,
    );
    WORST_FLAGS.store(flags, Ordering::Relaxed);
    WORST_ENTRY_LATENESS_CYCLES.store(entry_lateness_cycles, Ordering::Release);
}

/// Close the event that produced a terminal commissioning result.
pub(crate) fn close_terminal_service(classify_cycle: u32) {
    close_event(classify_cycle, true);
}

/// Close the event that produced a ready-now commissioning result.
pub(crate) fn close_ready_service(classify_cycle: u32) {
    close_event(classify_cycle, false);
}

/// Arm attribution for one short absolute commissioning deadline.
pub(crate) fn arm_short_deadline(
    deadline_us: u64,
    now_us: u64,
    arm_cycle: u32,
    classification_source_mask: u32,
) {
    close_event(arm_cycle, false);

    EVENT_TIMER_RETURN_CYCLE.store(0, Ordering::Relaxed);
    EVENT_TIMER_LATENESS_US.store(0, Ordering::Relaxed);
    EVENT_POLL_ENTRY_CYCLE.store(0, Ordering::Relaxed);
    EVENT_LAST_ELAPSED_CYCLE.store(0, Ordering::Relaxed);
    EVENT_ENGINE_RETURN_CYCLE.store(0, Ordering::Relaxed);
    EVENT_POST_DEADLINE_SOURCE_MASK.store(0, Ordering::Relaxed);
    EVENT_PRE_ARM_SOURCE_PRESENT.store(false, Ordering::Relaxed);
    EVENT_EXPIRED_BEFORE_SLEEP.store(deadline_us <= now_us, Ordering::Relaxed);

    if deadline_us > now_us {
        let wait_us = deadline_us - now_us;
        if wait_us > MAX_ATTRIBUTED_WAIT_US {
            WAIT_PROBE.cancel_active();
            return;
        }
        let wait_us = wait_us as u32;
        let due_cycle = arm_cycle.wrapping_add(wait_us.saturating_mul(CYCLES_PER_MICROSECOND));
        EVENT_DUE_CYCLE.store(due_cycle, Ordering::Relaxed);
        EVENT_REQUESTED_WAIT_US.store(wait_us, Ordering::Relaxed);
        EVENT_ACTIVE.store(true, Ordering::Release);
        WAIT_PROBE.arm(due_cycle, wait_us);
        return;
    }

    let overdue_us = now_us - deadline_us;
    if overdue_us > MAX_ATTRIBUTED_OVERDUE_US {
        WAIT_PROBE.cancel_active();
        return;
    }
    let due_cycle =
        arm_cycle.wrapping_sub((overdue_us as u32).saturating_mul(CYCLES_PER_MICROSECOND));
    EVENT_DUE_CYCLE.store(due_cycle, Ordering::Relaxed);
    EVENT_REQUESTED_WAIT_US.store(0, Ordering::Relaxed);
    if classification_source_mask != 0 {
        EVENT_POST_DEADLINE_SOURCE_MASK.store(classification_source_mask, Ordering::Relaxed);
        EVENT_PRE_ARM_SOURCE_PRESENT.store(true, Ordering::Relaxed);
    }
    EVENT_ACTIVE.store(true, Ordering::Release);
    WAIT_PROBE.arm(due_cycle, 0);
    WAIT_PROBE.mark_sleep_entry(arm_cycle);
}

/// Record the monotonic and DWT timestamps immediately after `delay_until` returns.
pub(crate) fn record_timer_return(wake_us: u64, returned_us: u64, cycle: u32) {
    if !EVENT_ACTIVE.load(Ordering::Acquire) {
        return;
    }
    EVENT_TIMER_RETURN_CYCLE.store(cycle, Ordering::Relaxed);
    EVENT_TIMER_LATENESS_US.store(
        u32::try_from(returned_us.saturating_sub(wake_us)).unwrap_or(u32::MAX),
        Ordering::Relaxed,
    );
}

/// Record the board boundary immediately after the scheduled engine poll returns.
pub(crate) fn record_engine_return(cycle: u32) {
    if EVENT_ACTIVE.load(Ordering::Acquire) {
        EVENT_ENGINE_RETURN_CYCLE.store(cycle, Ordering::Relaxed);
    }
}

/// Mark the final board handoff immediately before sleeping or repolling.
pub(crate) fn mark_sleep_entry() {
    let sleep_entry_cycle = cortex_m::peripheral::DWT::cycle_count();
    WAIT_PROBE.mark_sleep_entry(sleep_entry_cycle);
    if EVENT_ACTIVE.load(Ordering::Acquire)
        && sleep_entry_cycle.wrapping_sub(EVENT_DUE_CYCLE.load(Ordering::Relaxed)) < (1 << 31)
    {
        EVENT_EXPIRED_BEFORE_SLEEP.store(true, Ordering::Relaxed);
    }
}

/// Cancel an active deadline when the wait exits without normal resumption.
pub(crate) fn cancel_active() {
    EVENT_ACTIVE.store(false, Ordering::Release);
    WAIT_PROBE.cancel_active();
    SERVICE_PROBE.cancel_active();
}

/// Begin source attribution around the board classification clock pair.
pub(crate) fn begin_classification_window() {
    CLASSIFICATION_SOURCE_MASK.store(0, Ordering::Relaxed);
    CLASSIFICATION_WINDOW_ACTIVE.store(true, Ordering::Release);
}

/// End classification attribution and return its guarded-source mask.
#[must_use]
pub(crate) fn finish_classification_window() -> u32 {
    CLASSIFICATION_WINDOW_ACTIVE.store(false, Ordering::Release);
    CLASSIFICATION_SOURCE_MASK.load(Ordering::Acquire)
}

/// Begin one attributed task or interrupt span.
#[must_use]
pub(crate) fn source_span(source_mask: u32) -> DeadlineSourceSpan {
    DeadlineSourceSpan {
        source_mask,
        entry_cycle: cortex_m::peripheral::DWT::cycle_count(),
    }
}

/// Scope guard that records a source's complete DWT span on drop.
///
/// A span that crosses the service-to-wait boundary is attributed only to the
/// probe active when the guard drops. Diagnostics therefore interpret a source
/// present in the wait mask as a possible boundary-straddling service source.
pub(crate) struct DeadlineSourceSpan {
    source_mask: u32,
    entry_cycle: u32,
}

impl Drop for DeadlineSourceSpan {
    fn drop(&mut self) {
        let exit_cycle = cortex_m::peripheral::DWT::cycle_count();
        if CLASSIFICATION_WINDOW_ACTIVE.load(Ordering::Acquire) {
            CLASSIFICATION_SOURCE_MASK.fetch_or(self.source_mask, Ordering::Relaxed);
        }
        if EVENT_ACTIVE.load(Ordering::Acquire)
            && source_overlap_cycles(
                EVENT_DUE_CYCLE.load(Ordering::Relaxed),
                self.entry_cycle,
                exit_cycle,
            ) != 0
        {
            EVENT_POST_DEADLINE_SOURCE_MASK.fetch_or(self.source_mask, Ordering::Relaxed);
        }
        WAIT_PROBE.record_source_span(self.source_mask, self.entry_cycle, exit_cycle);
        SERVICE_PROBE.record_source_span(self.source_mask, self.entry_cycle, exit_cycle);
    }
}

/// Encode the worst wait and service windows on the existing generic path.
///
/// Diagnostic 40 contains packed timer-return lateness and return-to-entry time,
/// packed requested wait and total entry lateness, and the same-event
/// post-deadline source mask plus flags. Diagnostic 41 contains
/// poll-entry-to-inner-return, inner-return-to-scheduled-engine-return, and
/// engine-return-to-classification durations. All values describe the same
/// worst entry event.
/// Bit 31 marks a terminal event, bit 30 marks a path without a timer wait, bit
/// 29 marks a deadline already expired before the wait handoff, and bit 28
/// marks a source captured while pairing the pre-arm classification clocks.
pub(crate) fn diagnostics() -> Option<[CommissionDiagnostic; 2]> {
    let entry_lateness_cycles = WORST_ENTRY_LATENESS_CYCLES.load(Ordering::Acquire);
    if entry_lateness_cycles == 0 {
        return None;
    }
    let cycles_to_us = |cycles: u32| cycles.div_ceil(CYCLES_PER_MICROSECOND);
    let pack_u16 =
        |low: u32, high: u32| low.min(u32::from(u16::MAX)) | (high.min(u32::from(u16::MAX)) << 16);
    let timer_lateness_us = WORST_TIMER_LATENESS_US.load(Ordering::Relaxed);
    let return_to_entry_us = cycles_to_us(WORST_RETURN_TO_ENTRY_CYCLES.load(Ordering::Relaxed));
    let requested_wait_us = WORST_REQUESTED_WAIT_US.load(Ordering::Relaxed);
    let entry_lateness_us = cycles_to_us(entry_lateness_cycles);
    Some([
        CommissionDiagnostic {
            phase: PhaseId::ElectricalId.to_u8(),
            code: DIAG_DEADLINE_RELEASE,
            status: 2,
            value0: pack_u16(timer_lateness_us, return_to_entry_us),
            value1: pack_u16(requested_wait_us, entry_lateness_us),
            value2: WORST_POST_DEADLINE_SOURCE_MASK.load(Ordering::Relaxed)
                | WORST_FLAGS.load(Ordering::Relaxed),
        },
        CommissionDiagnostic {
            phase: PhaseId::ElectricalId.to_u8(),
            code: DIAG_DEADLINE_HOPS,
            status: 2,
            value0: cycles_to_us(WORST_ENTRY_TO_INNER_RETURN_CYCLES.load(Ordering::Relaxed)),
            value1: cycles_to_us(WORST_INNER_TO_ENGINE_RETURN_CYCLES.load(Ordering::Relaxed)),
            value2: cycles_to_us(WORST_ENGINE_RETURN_TO_CLASSIFY_CYCLES.load(Ordering::Relaxed)),
        },
    ])
}
