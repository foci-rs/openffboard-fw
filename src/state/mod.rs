//! Board-level static state, decomposed by subsystem.

pub mod stepper;
pub mod stepper_diag;
pub mod tmc;
#[cfg(feature = "trace")]
pub(crate) mod usb_trace;
