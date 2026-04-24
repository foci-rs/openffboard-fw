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
// The allow(unused_imports) suppresses the dead-shim warning introduced in Tasks 21/22 once
// BOARD_PROVIDER no longer enumerates these items directly. Task 24 removes this module.
pub mod replies {
    #[allow(unused_imports)]
    pub use foci_firmware::ankyra::replies::*;
}
// Re-export foci-firmware outputs as a local module for ankyra_provider! macro compatibility.
// Task 24 removes this module.
pub mod outputs {
    #[allow(unused_imports)]
    pub use foci_firmware::ankyra::outputs::*;
}
// Re-export foci-firmware firmware-wide constants for ankyra_provider! compatibility.
// Board-specific constants (CLOCK_FREQ, MCU, RESERVE_PINS_*) are defined in the local
// constants module. Task 24 removes this module.
pub mod firmware_constants {
    #[allow(unused_imports)]
    pub use foci_firmware::ankyra::constants::*;
}
