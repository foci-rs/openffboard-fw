//! OpenFFBoard attribution for short commissioning deadline releases.

use foci_firmware::autotune::PhaseId;
use foci_firmware::selftest::CommissionDiagnostic;
use foci_firmware::tmc_control::deadline_attribution::DeadlineAttributionProbe;

const CYCLES_PER_MICROSECOND: u32 = 168;
const MAX_ATTRIBUTED_WAIT_US: u64 = 250;
const MAX_ATTRIBUTED_OVERDUE_US: u64 = 1_000;
const DIAG_DEADLINE_RELEASE: u8 = 40;
const DIAG_DEADLINE_SOURCE: u8 = 41;

pub(crate) const SOURCE_WAIT_REQUEST: u32 = 1 << 1;
pub(crate) const SOURCE_USB_TASK: u32 = 1 << 2;
pub(crate) const SOURCE_TIM5_IRQ: u32 = 1 << 3;
pub(crate) const SOURCE_USB_IRQ: u32 = 1 << 4;
pub(crate) const SOURCE_FAULT_IRQ: u32 = 1 << 5;
pub(crate) const SOURCE_STATS_TASK: u32 = 1 << 6;
pub(crate) const SOURCE_BLINK_TASK: u32 = 1 << 7;

static PROBE: DeadlineAttributionProbe = DeadlineAttributionProbe::new();

/// Reset retained attribution at the beginning of a commissioning run.
pub(crate) fn reset() {
    PROBE.reset();
}

/// Arm attribution for one short absolute commissioning deadline.
pub(crate) fn arm_short_deadline(deadline_us: u64, now_us: u64, arm_cycle: u32) {
    if deadline_us > now_us {
        let wait_us = deadline_us - now_us;
        if wait_us > MAX_ATTRIBUTED_WAIT_US {
            PROBE.cancel_active();
            return;
        }
        let wait_us = wait_us as u32;
        let due_cycle = arm_cycle.wrapping_add(wait_us.saturating_mul(CYCLES_PER_MICROSECOND));
        PROBE.arm(due_cycle, wait_us);
        return;
    }

    let overdue_us = now_us - deadline_us;
    if overdue_us > MAX_ATTRIBUTED_OVERDUE_US {
        PROBE.cancel_active();
        return;
    }
    let due_cycle =
        arm_cycle.wrapping_sub((overdue_us as u32).saturating_mul(CYCLES_PER_MICROSECOND));
    PROBE.arm(due_cycle, 0);
    PROBE.mark_sleep_entry(arm_cycle);
}

/// Mark the final board handoff immediately before sleeping or repolling.
pub(crate) fn mark_sleep_entry() {
    PROBE.mark_sleep_entry(cortex_m::peripheral::DWT::cycle_count());
}

/// Complete the active deadline when the engine observes its resumed timestamp.
pub(crate) fn resume() {
    PROBE.resume(cortex_m::peripheral::DWT::cycle_count());
}

/// Cancel an active deadline when the wait exits without normal resumption.
pub(crate) fn cancel_active() {
    PROBE.cancel_active();
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
pub(crate) struct DeadlineSourceSpan {
    source_mask: u32,
    entry_cycle: u32,
}

impl Drop for DeadlineSourceSpan {
    fn drop(&mut self) {
        PROBE.record_source_span(
            self.source_mask,
            self.entry_cycle,
            cortex_m::peripheral::DWT::cycle_count(),
        );
    }
}

/// Encode the worst retained short-deadline release on the existing generic path.
pub(crate) fn diagnostics() -> Option<[CommissionDiagnostic; 2]> {
    let retained = PROBE.worst_snapshot();
    if retained.release_lateness_cycles == 0 {
        return None;
    }
    let evidence = retained.to_microseconds(CYCLES_PER_MICROSECOND)?;
    Some([
        CommissionDiagnostic {
            phase: PhaseId::ElectricalId.to_u8(),
            code: DIAG_DEADLINE_RELEASE,
            status: 2,
            value0: evidence.requested_wait_us,
            value1: evidence.release_lateness_us,
            value2: evidence.source_mask,
        },
        CommissionDiagnostic {
            phase: PhaseId::ElectricalId.to_u8(),
            code: DIAG_DEADLINE_SOURCE,
            status: 2,
            value0: evidence.accumulated_overlap_us,
            value1: evidence.max_source_overlap_us,
            value2: u32::from(evidence.deadline_expired_before_sleep),
        },
    ])
}
