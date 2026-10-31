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

    use crate::usb_trace_policy::{
        TraceModeRequests, skipped_control_detail_for, skipped_critical_for,
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
}
