//! `#[klipper_reply]` struct declarations for openffboard-fw.
//!
//! Each struct in this module is a reply shape that a `#[klipper_command]`
//! handler (or Task 8's background reply emitters) sends back to the
//! Klipper host via `::ankyra::klipper_reply!(Path, field: ty = expr, ...)`.
//!
//! # Wire-name preservation
//!
//! Ankyra derives the protocol name from the struct identifier verbatim
//! (`protocol_name = struct_ident.to_string()`); there is no `rename`
//! attribute. To keep wire compatibility with the pre-migration anchor
//! firmware and the Klipper host's expectations, struct names in this
//! module match the on-the-wire protocol names exactly — including the
//! `snake_case` forms Klipper uses. The `#[allow(non_camel_case_types)]`
//! attribute is applied per struct so clippy/rustc accept the lowercase
//! identifier that matches the wire name.

use ankyra::prelude::klipper_reply;

/// `trsync_state` reply — emitted by the trigger subsystem as trsync
/// snapshots are drained.
///
/// The struct identifier is intentionally snake_case so ankyra produces
/// `trsync_state` as the protocol name; see the module-level docs.
#[klipper_reply]
#[allow(non_camel_case_types)]
pub struct trsync_state {
    /// Object id of the trsync endpoint reporting the snapshot.
    pub oid: u8,
    /// Non-zero while the trsync can still fire its trigger.
    pub can_trigger: u8,
    /// Reason code captured when the trsync transitioned out of
    /// `can_trigger`.
    pub trigger_reason: u8,
    /// Clock tick at which the snapshot was taken.
    pub clock: u32,
}
