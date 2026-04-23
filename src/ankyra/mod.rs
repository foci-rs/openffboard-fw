//! Protocol-shape items and dispatch surfaces for the ankyra Klipper
//! protocol framework.
//!
//! Owns `#[klipper_reply]` / `#[klipper_output]` / `#[klipper_constant]`
//! struct declarations and (added in Task 8) `klipper_enumeration!`,
//! `ankyra_provider!`, and the per-subsystem handler modules. The
//! `ankyra_config!` invocation stays at the crate root (`main.rs`)
//! because ankyra requires it there.

pub mod enumerations;
pub mod handlers;
pub mod outputs;
pub mod provider;
pub mod replies;
