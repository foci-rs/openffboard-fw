//! Re-export of the lifted digital-output handlers.
//!
//! The handler bodies now live in
//! `foci_firmware::ankyra::handlers::digital_out` (lifted in Task 11).
//! This module re-exports them so the board's `ankyra_provider!`
//! invocation — which requires `crate::…` paths — can continue to name
//! them. Task 24 removes this shim entirely once BOARD_PROVIDER no
//! longer references these items directly.

#[allow(unused_imports)]
pub use foci_firmware::ankyra::handlers::digital_out::*;
