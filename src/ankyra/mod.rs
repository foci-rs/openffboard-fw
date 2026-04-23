//! Protocol-shape items and dispatch surfaces for the ankyra Klipper
//! protocol framework.
//!
//! Owns `#[klipper_reply]` / `#[klipper_output]` / `#[klipper_constant]`
//! struct declarations and (added in Task 8) `klipper_enumeration!`,
//! `ankyra_provider!`, and the per-subsystem handler modules. The
//! `ankyra_config!` invocation stays at the crate root (`main.rs`)
//! because ankyra requires it there.
//!
//! Note: `replies` was moved to `foci_firmware::ankyra::replies` in Task 6
//! and is re-exported here as a module for ankyra_provider! compatibility.

pub mod constants;
pub mod enumerations;
pub mod handlers;
pub mod provider;
// Re-export foci-firmware replies as a local module for ankyra_provider! macro compatibility.
pub mod replies {
    pub use foci_firmware::ankyra::replies::*;
}
// Re-export foci-firmware outputs as a local module for ankyra_provider! macro compatibility.
pub mod outputs {
    pub use foci_firmware::ankyra::outputs::*;
}
