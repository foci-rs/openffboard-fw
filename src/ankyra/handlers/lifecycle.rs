//! Re-export of the lifted lifecycle handlers.
//!
//! The handler bodies now live in `foci_firmware::ankyra::handlers::lifecycle`
//! (lifted in Task 10). This module re-exports them so the board's
//! `ankyra_provider!` invocation — which requires `crate::…` paths — can
//! continue to name them. Task 20 replaces this shim with an
//! `ankyra_reexport_provider!` of the upstream `FOCI_FIRMWARE_PROVIDER`.

#[allow(unused_imports)]
pub use foci_firmware::ankyra::handlers::lifecycle::*;
