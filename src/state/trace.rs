//! Trace-buffer state (ring buffer, generation counter, capture mask).
//! Task 8b populates this from `mod app` in main.rs.

/// Trace capture buffer in CCM RAM. Initialized via `reset()` in init.
///
/// SAFETY: Only accessed from tmc_control (P1) after init. MaybeUninit
/// because CCM RAM has no default initialization; init() writes a valid
/// TraceBuffer via ptr::write before any read.
#[unsafe(link_section = ".ccmram")]
pub(crate) static mut TRACE_BUF: core::mem::MaybeUninit<foci_firmware::trace::TraceBuffer> =
    core::mem::MaybeUninit::uninit();
