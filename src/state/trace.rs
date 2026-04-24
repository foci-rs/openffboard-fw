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

/// Get a mutable reference to the trace buffer.
///
/// # Safety
///
/// Must only be called from tmc_control (P1) after init has completed.
#[inline]
pub(crate) unsafe fn trace_buf() -> &'static mut foci_firmware::trace::TraceBuffer {
    // Use addr_of_mut! to avoid creating a reference to the mutable static
    // (Rust 2024 edition compatibility). MaybeUninit<T> has the same layout as T.
    // SAFETY: caller upholds the P1-only + post-init contract.
    unsafe { &mut *core::ptr::addr_of_mut!(TRACE_BUF).cast::<foci_firmware::trace::TraceBuffer>() }
}
