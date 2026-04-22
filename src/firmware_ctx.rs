//! Concrete context passed to every ankyra command handler in this
//! binary.
//!
//! Starts as a thin wrapper around `foci_klipper::state::State` —
//! identical scope to what anchor dispatched today. Grow this struct
//! with additional borrows/handles as individual handler ports reveal
//! the binary-scope helpers they need.

use foci_klipper::state::State;

pub struct FirmwareCtx<'a> {
    pub state: &'a mut State,
}

impl<'a> FirmwareCtx<'a> {
    pub fn new(state: &'a mut State) -> Self {
        Self { state }
    }
}

impl<'a> ankyra::ShutdownState for FirmwareCtx<'a> {
    fn is_shutdown(&self) -> bool {
        self.state.is_shutdown()
    }
}
