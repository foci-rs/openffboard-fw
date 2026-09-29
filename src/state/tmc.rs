//! TMC4671-subsystem board-local state: RTIC channel sender and channel size.

use core::cell::RefCell;

use critical_section::Mutex;
use foci_firmware::tmc::TmcRequest;
use rtic_sync::channel::Sender;

use crate::board_config;

/// Channel depth for TMC control requests.
pub(crate) const TMC_CHANNEL_SIZE: usize = 8;
const _: () = assert!(
    board_config::TMC_CHANNEL_COUNT as usize
        * foci_firmware::state::tmc::CONNECT_TIME_QUEUE_SLOTS_PER_CHANNEL
        * 2
        <= TMC_CHANNEL_SIZE
);

/// Shared sender handle for the TMC control channel. Populated in RTIC
/// `init`, used by P2 command handlers to enqueue [`TmcRequest`]s.
pub(crate) static TMC_REQUEST_SENDER: Mutex<
    RefCell<Option<Sender<'static, TmcRequest, TMC_CHANNEL_SIZE>>>,
> = Mutex::new(RefCell::new(None));
