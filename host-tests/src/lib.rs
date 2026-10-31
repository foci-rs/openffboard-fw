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
        EmitOutcome, TraceModeRequests, emit_pending, skipped_control_detail_for,
        skipped_critical_for,
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
    fn mode_requests_are_serviced_while_a_snapshot_is_owed_and_the_snapshot_coalesces() {
        let q = TraceModeRequests::new();
        let r = ControlStateRecord::new();
        q.push(TraceMode::Critical, 3).unwrap();
        q.push(TraceMode::Dormant, 4).unwrap();
        r.mark_snapshot();

        assert_eq!(q.pop(), Some((TraceMode::Critical, 3)));
        r.mark_snapshot();
        assert_eq!(q.pop(), Some((TraceMode::Dormant, 4)));
        r.mark_snapshot();
        assert!(q.is_empty());

        let mut snapshots = 0;
        assert_eq!(
            emit_pending(&r, |h| {
                assert_eq!(h.applied_request, AppliedRequest::None);
                snapshots += 1;
                Ok(())
            }),
            EmitOutcome::Queued
        );
        assert_eq!(snapshots, 1);
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

    #[test]
    fn control_state_emission_retries_until_queued_and_sends_the_latest_state() {
        let r = ControlStateRecord::new();
        r.record(AppliedRequest::OuterGains);
        r.stamp(10);
        assert_eq!(emit_pending(&r, |_| Err(QueueFull)), EmitOutcome::Retained);
        assert!(r.is_dirty());
        r.record(AppliedRequest::RunCurrent);
        r.stamp(11);
        let sent = core::cell::Cell::new(None);
        assert_eq!(
            emit_pending(&r, |h| {
                sent.set(Some(h));
                Ok(())
            }),
            EmitOutcome::Queued
        );
        assert!(!r.is_dirty());
        assert_eq!(sent.get().unwrap().change_sequence, 2);
        assert_eq!(
            emit_pending(&r, |_| panic!("must not emit twice")),
            EmitOutcome::Clean
        );
    }

    #[test]
    fn snapshot_and_mutator_frames_go_out_in_order_and_retry_independently() {
        let r = ControlStateRecord::new();
        r.record(AppliedRequest::OuterGains);
        r.stamp(10);
        r.mark_snapshot();
        let sent: core::cell::RefCell<heapless::Vec<ControlStateHeader, 2>> = Default::default();
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
        assert_eq!(
            emit_pending(&r, |h| {
                sent.borrow_mut().push(h).unwrap();
                Ok(())
            }),
            EmitOutcome::Queued
        );
        assert_eq!(sent.borrow()[1].applied_request, AppliedRequest::OuterGains);
    }
}
