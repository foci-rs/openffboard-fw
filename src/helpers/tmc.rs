//! TMC4671 helpers: request enqueue to the `tmc_control` task.

use foci_firmware::context::tmc::TmcRequestQueueBoard;
use foci_firmware::tmc::TmcRequest;

use crate::state::tmc::TMC_REQUEST_SENDER;

pub(crate) struct OpenFfboardTmcQueue;

impl TmcRequestQueueBoard for OpenFfboardTmcQueue {
    fn try_send_tmc_request(&self, req: TmcRequest) -> bool {
        critical_section::with(|cs| {
            if let Some(sender) = TMC_REQUEST_SENDER.borrow(cs).borrow_mut().as_mut() {
                return sender.try_send(req).is_ok();
            }
            false
        })
    }
}
