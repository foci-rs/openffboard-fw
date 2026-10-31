#![no_std]

#[cfg(test)]
extern crate std;

#[cfg(test)]
#[path = "../../src/state/stepper_mapper.rs"]
mod stepper_mapper;

#[allow(dead_code)]
#[path = "../../src/usb_trace_policy.rs"]
mod usb_trace_policy;

#[cfg(test)]
mod usb_trace_policy_tests {
    use foci_firmware::trace::sampling::TraceMode;

    use foci_firmware::context::QueueFull;
    use foci_firmware::trace::control_state::{
        AppliedRequest, ControlStateHeader, ControlStateRecord,
    };

    use crate::usb_trace_policy::{
        EmitOutcome, MAX_CONTROL_STATE_FRAMES_PER_TICK, TraceModeRequests, emit_pending,
        skipped_control_detail_for, skipped_critical_for,
    };

    #[test]
    fn pre_config_skipped_critical_counts_only_critical_mode_time() {
        assert_eq!(skipped_critical_for(2000, 0), 0);
        assert_eq!(skipped_critical_for(2000, 500), 500);
    }

    #[test]
    fn pre_config_skipped_control_detail_uses_100_hz() {
        assert_eq!(skipped_control_detail_for(2000), 200);
    }

    #[test]
    fn mode_requests_are_serviced_in_order_with_their_oid() {
        let q = TraceModeRequests::new();
        assert!(q.push(TraceMode::Critical, 3).is_ok());
        assert!(q.push(TraceMode::Diagnostic, 4).is_ok());
        assert!(!q.is_empty());
        assert_eq!(q.pop(), Some((TraceMode::Critical, 3)));
        assert_eq!(q.pop(), Some((TraceMode::Diagnostic, 4)));
        assert_eq!(q.pop(), None);
    }

    #[test]
    fn a_full_ring_rejects_the_push_and_keeps_the_queued_modes() {
        let q = TraceModeRequests::new();
        for m in [
            TraceMode::Critical,
            TraceMode::Dormant,
            TraceMode::Diagnostic,
            TraceMode::Critical,
        ] {
            assert!(q.push(m, 3).is_ok());
        }
        assert!(q.push(TraceMode::Dormant, 3).is_err());
        assert_eq!(q.pop(), Some((TraceMode::Critical, 3)));
        assert!(q.push(TraceMode::Dormant, 3).is_ok());
    }

    use core::cell::RefCell;

    type Sent = RefCell<heapless::Vec<ControlStateHeader, 8>>;

    fn accept_into(sent: &Sent) -> impl FnMut(ControlStateHeader) -> Result<(), QueueFull> + '_ {
        |h| {
            sent.borrow_mut().push(h).unwrap();
            Ok(())
        }
    }

    fn sequences(sent: &Sent) -> heapless::Vec<u32, 8> {
        sent.borrow().iter().map(|h| h.change_sequence).collect()
    }

    #[test]
    fn four_filter_requests_yield_four_frames_in_order_with_consecutive_sequences() {
        let r = ControlStateRecord::new();
        for kind in [
            AppliedRequest::VelocityFilter,
            AppliedRequest::TorqueFilter,
            AppliedRequest::PositionFilter,
            AppliedRequest::FluxFilter,
        ] {
            r.record(kind);
        }
        r.stamp(10);
        let sent = Sent::default();
        assert_eq!(emit_pending(&r, accept_into(&sent)), EmitOutcome::Queued);
        let sent = sent.borrow();
        let kinds: heapless::Vec<_, 4> = sent.iter().map(|h| h.applied_request).collect();
        assert_eq!(
            kinds.as_slice(),
            [
                AppliedRequest::VelocityFilter,
                AppliedRequest::TorqueFilter,
                AppliedRequest::PositionFilter,
                AppliedRequest::FluxFilter,
            ]
        );
        let seqs: heapless::Vec<_, 4> = sent.iter().map(|h| h.change_sequence).collect();
        assert_eq!(seqs.as_slice(), [1, 2, 3, 4]);
    }

    #[test]
    fn a_sender_that_takes_two_frames_leaves_the_rest_for_the_next_call_without_repeats() {
        let r = ControlStateRecord::new();
        for _ in 0..4 {
            r.record(AppliedRequest::RunCurrent);
        }
        r.stamp(10);
        let sent = Sent::default();
        let mut room = 2;
        assert_eq!(
            emit_pending(&r, |h| {
                if room == 0 {
                    return Err(QueueFull);
                }
                room -= 1;
                sent.borrow_mut().push(h).unwrap();
                Ok(())
            }),
            EmitOutcome::Retained
        );
        assert_eq!(sequences(&sent).as_slice(), [1, 2]);
        assert!(r.is_dirty());
        assert_eq!(emit_pending(&r, accept_into(&sent)), EmitOutcome::Queued);
        assert_eq!(sequences(&sent).as_slice(), [1, 2, 3, 4]);
        assert_eq!(
            emit_pending(&r, |_| panic!("must not emit twice")),
            EmitOutcome::Clean
        );
    }

    #[test]
    fn a_refused_entry_is_offered_again_and_accepted_exactly_once() {
        let r = ControlStateRecord::new();
        r.record(AppliedRequest::OuterGains);
        r.stamp(10);
        assert_eq!(emit_pending(&r, |_| Err(QueueFull)), EmitOutcome::Retained);
        assert!(r.is_dirty());
        r.record(AppliedRequest::RunCurrent);
        r.stamp(11);
        let sent = Sent::default();
        assert_eq!(emit_pending(&r, accept_into(&sent)), EmitOutcome::Queued);
        assert_eq!(sequences(&sent).as_slice(), [1, 2]);
        assert!(!r.is_dirty());
        assert_eq!(
            emit_pending(&r, |_| panic!("must not emit twice")),
            EmitOutcome::Clean
        );
    }

    #[test]
    fn snapshot_frame_precedes_mutator_frames_and_retries_independently() {
        let r = ControlStateRecord::new();
        r.record(AppliedRequest::OuterGains);
        r.record(AppliedRequest::RunCurrent);
        r.stamp(10);
        r.mark_snapshot();
        let sent = Sent::default();
        let mut room = 1;
        assert_eq!(
            emit_pending(&r, |h| {
                if room == 0 {
                    return Err(QueueFull);
                }
                room -= 1;
                sent.borrow_mut().push(h).unwrap();
                Ok(())
            }),
            EmitOutcome::Retained
        );
        assert_eq!(sent.borrow()[0].applied_request, AppliedRequest::None);
        assert!(!r.is_snapshot_pending());
        assert!(r.is_dirty());
        assert_eq!(emit_pending(&r, accept_into(&sent)), EmitOutcome::Queued);
        let kinds: heapless::Vec<_, 3> = sent.borrow().iter().map(|h| h.applied_request).collect();
        assert_eq!(
            kinds.as_slice(),
            [
                AppliedRequest::None,
                AppliedRequest::OuterGains,
                AppliedRequest::RunCurrent,
            ]
        );
    }

    #[test]
    fn mutator_frames_per_call_are_capped_and_the_remainder_goes_next_call() {
        let r = ControlStateRecord::new();
        for _ in 0..MAX_CONTROL_STATE_FRAMES_PER_TICK + 2 {
            r.record(AppliedRequest::VoltageLimit);
        }
        r.stamp(10);
        let sent = Sent::default();
        assert_eq!(emit_pending(&r, accept_into(&sent)), EmitOutcome::Retained);
        assert_eq!(sequences(&sent).as_slice(), [1, 2, 3, 4]);
        assert_eq!(emit_pending(&r, accept_into(&sent)), EmitOutcome::Queued);
        assert_eq!(sequences(&sent).as_slice(), [1, 2, 3, 4, 5, 6]);
    }
}
