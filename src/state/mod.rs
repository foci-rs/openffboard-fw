//! Board-level static state, decomposed by subsystem.
//!
//! Each submodule holds the `pub(crate) static` atomics and simple
//! statics for its subsystem. Task 8b populates these from `mod app`
//! in main.rs. Future boards may eventually share these via a common
//! crate.

pub mod lifecycle;
pub mod stepper;
pub mod tmc;
pub mod trace;
pub mod trsync;
