//! OpenFFBoard attribution for short commissioning deadline releases.

use core::sync::atomic::{AtomicBool, Ordering};

use foci_firmware::autotune::PhaseId;
use foci_firmware::selftest::CommissionDiagnostic;
use foci_firmware::tmc_control::deadline_attribution::DeadlineAttributionProbe;

const CYCLES_PER_MICROSECOND: u32 = 168;
const MAX_ATTRIBUTED_WAIT_US: u64 = 250;
const MAX_ATTRIBUTED_OVERDUE_US: u64 = 1_000;
const DIAG_DEADLINE_RELEASE: u8 = 40;
const DIAG_DEADLINE_SOURCE: u8 = 41;
const WINDOW_FLAG: u32 = 1 << 31;

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

static WAIT_PROBE: DeadlineAttributionProbe = DeadlineAttributionProbe::new();
static SERVICE_PROBE: DeadlineAttributionProbe = DeadlineAttributionProbe::new();
static SERVICE_WORST_WAS_TERMINAL: AtomicBool = AtomicBool::new(false);

/// Reset retained attribution at the beginning of a commissioning run.
pub(crate) fn reset() {
    WAIT_PROBE.reset();
    SERVICE_PROBE.reset();
    SERVICE_WORST_WAS_TERMINAL.store(false, Ordering::Relaxed);
}

/// Complete an active wait and anchor a new service window at the paired DWT instant.
pub(crate) fn observe_elapsed(cycle: u32) {
    WAIT_PROBE.resume(cycle);
    SERVICE_PROBE.arm(cycle, 0);
}

fn close_service_window(classify_cycle: u32, terminal: bool) {
    let previous = SERVICE_PROBE.worst_snapshot().release_lateness_cycles;
    SERVICE_PROBE.resume(classify_cycle);
    let current = SERVICE_PROBE.worst_snapshot().release_lateness_cycles;
    if current > previous {
        SERVICE_WORST_WAS_TERMINAL.store(terminal, Ordering::Relaxed);
    }
}

/// Close the service window that produced a terminal commissioning result.
pub(crate) fn close_terminal_service(classify_cycle: u32) {
    close_service_window(classify_cycle, true);
}

/// Arm attribution for one short absolute commissioning deadline.
pub(crate) fn arm_short_deadline(deadline_us: u64, now_us: u64, arm_cycle: u32) {
    if deadline_us > now_us {
        let wait_us = deadline_us - now_us;
        if wait_us > MAX_ATTRIBUTED_WAIT_US {
            WAIT_PROBE.cancel_active();
            return;
        }
        close_service_window(arm_cycle, false);
        let wait_us = wait_us as u32;
        let due_cycle = arm_cycle.wrapping_add(wait_us.saturating_mul(CYCLES_PER_MICROSECOND));
        WAIT_PROBE.arm(due_cycle, wait_us);
        return;
    }

    let overdue_us = now_us - deadline_us;
    if overdue_us > MAX_ATTRIBUTED_OVERDUE_US {
        WAIT_PROBE.cancel_active();
        return;
    }
    close_service_window(arm_cycle, false);
    let due_cycle =
        arm_cycle.wrapping_sub((overdue_us as u32).saturating_mul(CYCLES_PER_MICROSECOND));
    WAIT_PROBE.arm(due_cycle, 0);
    WAIT_PROBE.mark_sleep_entry(arm_cycle);
}

/// Mark the final board handoff immediately before sleeping or repolling.
pub(crate) fn mark_sleep_entry() {
    WAIT_PROBE.mark_sleep_entry(cortex_m::peripheral::DWT::cycle_count());
}

/// Cancel an active deadline when the wait exits without normal resumption.
pub(crate) fn cancel_active() {
    WAIT_PROBE.cancel_active();
    SERVICE_PROBE.cancel_active();
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
        WAIT_PROBE.record_source_span(self.source_mask, self.entry_cycle, exit_cycle);
        SERVICE_PROBE.record_source_span(self.source_mask, self.entry_cycle, exit_cycle);
    }
}

/// Encode the worst wait and service windows on the existing generic path.
///
/// Diagnostic 40 contains requested wait, release lateness, and wait source
/// mask. Bit 31 of the mask means the deadline had expired before sleep.
/// Diagnostic 41 contains service elapsed time, attributed overlap, and service
/// source mask. Bit 31 of the mask means the retained service window ended in a
/// terminal classification.
pub(crate) fn diagnostics() -> Option<[CommissionDiagnostic; 2]> {
    let wait = WAIT_PROBE.worst_snapshot();
    let service = SERVICE_PROBE.worst_snapshot();
    if wait.release_lateness_cycles == 0 && service.release_lateness_cycles == 0 {
        return None;
    }
    let wait = wait.to_microseconds(CYCLES_PER_MICROSECOND)?;
    let service = service.to_microseconds(CYCLES_PER_MICROSECOND)?;
    let wait_flags = if wait.deadline_expired_before_sleep {
        WINDOW_FLAG
    } else {
        0
    };
    let service_flags = if SERVICE_WORST_WAS_TERMINAL.load(Ordering::Relaxed) {
        WINDOW_FLAG
    } else {
        0
    };
    Some([
        CommissionDiagnostic {
            phase: PhaseId::ElectricalId.to_u8(),
            code: DIAG_DEADLINE_RELEASE,
            status: 2,
            value0: wait.requested_wait_us,
            value1: wait.release_lateness_us,
            value2: wait.source_mask | wait_flags,
        },
        CommissionDiagnostic {
            phase: PhaseId::ElectricalId.to_u8(),
            code: DIAG_DEADLINE_SOURCE,
            status: 2,
            value0: service.release_lateness_us,
            value1: service.accumulated_overlap_us,
            value2: service.source_mask | service_flags,
        },
    ])
}
