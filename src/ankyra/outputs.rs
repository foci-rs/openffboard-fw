//! `#[klipper_output]` struct declarations for openffboard-fw.
//!
//! Each struct in this module is a Klipper output shape sent to the host
//! via `::ankyra::klipper_output!(Path, field: ty = expr, ...)`. Outputs
//! differ from replies only in the wire-direction marker trait
//! (`OutputPayload` vs `ReplyPayload`); the encoding and dictionary
//! entries are built identically.
//!
//! # Wire-name preservation
//!
//! Ankyra derives the protocol name from the struct identifier verbatim
//! (`protocol_name = struct_ident.to_string()`) and — in the absence of
//! an explicit `#[klipper_output(format = "...")]` override — synthesizes
//! the Klipper-style message format as
//! `"<struct_name> <field1>=%<spec1> ..."`. Struct names in this module
//! therefore match the on-the-wire protocol name exactly (including any
//! `snake_case` form the Klipper host expects). The
//! `#[allow(non_camel_case_types)]` attribute is applied per struct so
//! the lowercase identifier is accepted.

use ankyra::prelude::klipper_output;

/// `stats` output — the once-per-second watchdog/accumulator frame
/// Klipper's logs surface as `Stats <clock> mcu=<name> ...`.
///
/// Emitted by the `stats_tick` task each time the watchdog is petted.
/// The field names and order reproduce the pre-migration anchor wire
/// format `"stats count=%u sum=%u sumsq=%u"` so the Klipper host's
/// dictionary match and per-field parser continue to work unchanged.
#[klipper_output]
#[allow(non_camel_case_types)]
pub struct stats {
    /// Number of stats ticks accumulated since the last emission.
    pub count: u32,
    /// Sum of the per-tick elapsed clock deltas.
    pub sum: u32,
    /// Sum of the squared per-tick elapsed deltas (scaled by
    /// `STATS_SUMSQ_BASE`).
    pub sumsq: u32,
}
