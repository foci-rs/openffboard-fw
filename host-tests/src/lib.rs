#![no_std]
#![allow(dead_code)]

#[path = "../../src/state/stepper.rs"]
mod stepper;

#[cfg(test)]
mod commissioning_board_contract_tests {
    const MAIN: &str = include_str!("../../src/main.rs");
    const REQUEST: &str = include_str!("../../src/tmc_control/request.rs");
    const REPLIES: &str = include_str!("../../src/ankyra/replies.rs");
    const CONSTANTS: &str = include_str!("../../src/ankyra/constants.rs");
    const PROVIDER: &str = include_str!("../../src/ankyra/provider.rs");
    const SHARED_EVENTS: &str =
        include_str!("../../../../shared/foci-firmware/src/trace/events.rs");
    const SHARED_TYPES: &str =
        include_str!("../../../../shared/foci-firmware/src/commissioning/types.rs");
    const SHARED_INTEGRAL_REPORTING: &str = include_str!(
        "../../../../shared/foci-firmware/src/commissioning/outer/velocity/integral_reporting.rs"
    );
    const SHARED_INTEGRAL_PLAN: &str = include_str!(
        "../../../../shared/foci-firmware/src/commissioning/outer/velocity/integral_plan.rs"
    );

    const REPLY_ID_WORST_CASE_BYTES: usize = 3;
    const KLIPPER_PAYLOAD_WITH_ID_MAX: usize = 59;

    fn assert_reply_contract(method: &str, reply: &str, fields: &[&str]) {
        let start = REPLIES
            .find(method)
            .unwrap_or_else(|| panic!("missing board reply method {method}"));
        let body = &REPLIES[start..];
        assert!(body.contains(reply), "{method} does not forward {reply}");

        let mut cursor = 0;
        for field in fields {
            let offset = body[cursor..]
                .find(field)
                .unwrap_or_else(|| panic!("{method} missing field {field}"));
            cursor += offset + field.len();
        }
    }

    fn assert_wire_budget(name: &str, field_widths: &[usize]) {
        let encoded = field_widths.iter().sum::<usize>() + REPLY_ID_WORST_CASE_BYTES;
        assert!(
            encoded <= KLIPPER_PAYLOAD_WITH_ID_MAX,
            "{name} worst-case payload with id is {encoded} bytes"
        );
    }

    #[test]
    fn startup_establishes_pi_representation_before_adc_vm_capture() {
        let chip_identity = MAIN
            .find("CHIPINFO_SI_TYPE")
            .expect("startup must verify the TMC4671 identity");
        let representation = MAIN
            .find("bootstrap_pi_representation")
            .expect("startup must establish the fixed PI representation");
        let adc_vm = MAIN
            .find("capture_adc_vm_offset")
            .expect("startup must retain ADC-VM capture");

        assert!(chip_identity < representation);
        assert!(representation < adc_vm);
        assert_eq!(MAIN.matches("bootstrap_pi_representation").count(), 1);
        assert!(MAIN.contains("TMC4671 PI representation failed at boot"));
        assert!(!MAIN.contains("0x15"));
    }

    #[test]
    fn board_does_not_forward_removed_stage_b_reply_families() {
        assert!(!REPLIES.contains("foci_velocity_stage_b_reproduction_v3_"));
        for removed in [
            "structured_boundary",
            "directional_region_core",
            "joint_region",
            "stage_b_handoff_core",
            "stage_b_nomination",
            "stage_b_directional_handoff",
            "stage_b_reproduction_v4_",
        ] {
            assert!(!REPLIES.contains(removed), "retained board reply {removed}");
        }
    }

    #[test]
    fn board_publishes_the_stage_b_schema_revision() {
        assert!(CONSTANTS.contains("pub const STAGE_B_EVIDENCE_SCHEMA_REVISION: u32 = 14;"));
    }

    #[test]
    fn board_owns_and_invalidates_one_stage_c_candidate() {
        assert_eq!(MAIN.matches("StageCCandidateSlot::new()").count(), 1);
        assert_eq!(MAIN.matches("stage_c_candidate.observe_fault()").count(), 2);
        assert!(MAIN.contains("CommissionTerminalContext::new"));
        assert!(REQUEST.contains("handle_tmc_control_request_with_stages"));
        assert!(REQUEST.contains("stage_c_candidate"));
    }

    #[test]
    fn board_retains_stage_c_authority_from_the_plan_event() {
        // Nonterminal classification stays outside the terminal async dispatch
        // future. foci-firmware's `scheduled_step` size gate pins that budget,
        // so Stage-C authority is installed synchronously from the plan event
        // rather than by routing polls through the staged poll driver.
        assert!(
            MAIN.contains("poll => runtime.classify_commission_poll(poll)"),
            "nonterminal polls must stay out of the async dispatch future"
        );
        assert!(
            !MAIN.contains("drive_commissioning_poll_with_stages"),
            "the staged poll driver pulls terminal dispatch into every poll"
        );

        let arm = MAIN
            .find("CommissionEvent::VelocityIntegralPlan")
            .expect("board must handle the Stage-C plan event");
        let body = &MAIN[arm..];
        let install = body
            .find("install_authority_from_plan")
            .expect("plan event must install retained Stage-C authority");
        let next_arm = body
            .find("ScheduledCommissionTickAction")
            .unwrap_or(body.len());
        assert!(
            install < next_arm,
            "authority install must sit inside the plan-event arm"
        );
    }

    #[test]
    fn board_reaches_the_staged_request_dispatch_that_owns_stage_c_resume() {
        // Two shared-firmware fixes for the Stage-C authority landed on paths
        // this board does not execute, and shared tests passed both times. The
        // board's own call shape is what decides whether new dispatch arms --
        // the Stage-C resume among them -- are reachable here at all.
        assert!(
            REQUEST.contains("handle_tmc_control_request_with_stages"),
            "tune requests must reach the staged request dispatch"
        );
        assert!(
            REQUEST.contains("stage_c_candidate"),
            "the staged dispatch must be given the retained Stage-C slot"
        );
        // Retention is what a resume consumes. It is installed synchronously
        // from the plan event because nonterminal classification stays outside
        // the terminal async dispatch future on this board.
        assert!(
            MAIN.contains("install_authority_from_plan"),
            "the plan event must retain authority for a later resume"
        );
        assert!(
            MAIN.contains("poll => runtime.classify_commission_poll(poll)"),
            "nonterminal polls must stay out of the async dispatch future"
        );
    }

    #[test]
    fn board_forwards_the_complete_stage_c_reply_family() {
        for method in [
            "foci_commissioning_workflow_plan",
            "foci_velocity_integral_plan_core",
            "foci_velocity_integral_plan_geometry",
            "foci_velocity_integral_plan_authority",
            "foci_velocity_integral_plan_timing",
            "foci_velocity_integral_plan_travel",
            "foci_velocity_integral_plan_rung",
            "foci_velocity_integral_observation_core",
            "foci_velocity_integral_observation_rate",
            "foci_velocity_integral_observation_quality",
            "foci_velocity_integral_rung_core",
            "foci_velocity_integral_rung_component",
            "foci_velocity_integral_run_summary",
            "foci_velocity_integral_curve_interval",
            "foci_velocity_integral_drift",
            "foci_velocity_integral_stage_b_comparison",
            "foci_velocity_integral_reproduction_core",
            "foci_velocity_integral_reproduction_mask",
            "foci_velocity_integral_reproduction_interval",
            "foci_velocity_integral_reproduction_digest",
            "foci_velocity_integral_terminal_core",
            "foci_velocity_integral_terminal_identity",
            "foci_velocity_integral_terminal_timing",
            "foci_robustness_reversal_terminal",
        ] {
            assert!(REPLIES.contains(method), "missing board reply {method}");
        }
    }

    #[test]
    fn board_forwards_integral_authority_without_narrowing() {
        assert_reply_contract(
            "foci_velocity_integral_plan_authority",
            "FociVelocityIntegralPlanAuthority",
            &[
                "directional_membership: u32",
                "joint_membership: u32",
                "directional_validity: u8",
                "reduced_margin: u8",
            ],
        );
        assert_reply_contract(
            "foci_velocity_integral_run_summary",
            "FociVelocityIntegralRunSummary",
            &["opening_available_mask: u8", "bookend_available_mask: u8"],
        );
        assert_reply_contract(
            "foci_velocity_integral_stage_b_comparison",
            "FociVelocityIntegralStageBComparison",
            &["direction: u8", "available: u8"],
        );
    }

    #[test]
    fn board_publishes_the_stage_c_schema_and_geometry_constants() {
        for declaration in [
            "pub const STAGE_C_EVIDENCE_SCHEMA_REVISION: u32 = 13;",
            "pub const STAGE_C_MAX_POSITIVE_RUNGS: u32 = 30;",
            "pub const STAGE_C_OBSERVATIONS_PER_DIRECTION: u32 = 4;",
            "pub const STAGE_C_OBSERVATIONS_PER_RUNG: u32 = 8;",
        ] {
            assert!(CONSTANTS.contains(declaration), "missing {declaration}");
        }
        for constant in [
            "STAGE_B_EVIDENCE_SCHEMA_REVISION",
            "STAGE_C_EVIDENCE_SCHEMA_REVISION",
            "STAGE_C_MAX_POSITIVE_RUNGS",
            "STAGE_C_OBSERVATIONS_PER_DIRECTION",
            "STAGE_C_OBSERVATIONS_PER_RUNG",
        ] {
            assert!(PROVIDER.contains(constant), "provider omits {constant}");
        }
        assert!(SHARED_TYPES.contains("Failed = 5"));
        assert!(
            SHARED_INTEGRAL_REPORTING.contains("PLAN_PROBE_CONSTRAINED_TEST_POINT: u8 = 1 << 1")
        );
        assert!(
            SHARED_INTEGRAL_REPORTING
                .contains("TERMINAL_PROBE_CONSTRAINED_TEST_POINT: u8 = 1 << 3")
        );
        assert_reply_contract(
            "foci_commissioning_workflow_plan",
            "FociCommissioningWorkflowPlan",
            &["shape: u8"],
        );
        assert!(!REPLIES.contains("foci_combined_"));
        assert!(!REPLIES.contains("foci_velocity_combined_"));
    }

    #[test]
    fn board_forwards_rung_origin_recovery() {
        for method in [
            "foci_rung_origin_recovery_summary",
            "foci_velocity_integral_plan_recovery",
        ] {
            assert!(REPLIES.contains(method), "missing board reply {method}");
        }
        for retired in [
            "foci_rung_origin_recovery_core",
            "foci_rung_origin_recovery_position",
            "foci_rung_origin_recovery_timing",
            "foci_rung_origin_recovery_limits",
        ] {
            assert!(!REPLIES.contains(retired), "retained board reply {retired}");
        }
        assert_reply_contract(
            "foci_rung_origin_recovery_summary",
            "FociRungOriginRecoverySummary",
            &[
                "oid: u8",
                "run_sequence: u32",
                "evidence_sequence: u16",
                "stage: u8",
                "rung_index: u8",
                "p_raw: u16",
                "binding_source: u8",
                "outcome: u8",
            ],
        );
        assert!(CONSTANTS.contains("pub const STAGE_B_EVIDENCE_SCHEMA_REVISION: u32 = 14;"));
        assert!(CONSTANTS.contains("pub const STAGE_C_EVIDENCE_SCHEMA_REVISION: u32 = 13;"));
        assert!(SHARED_EVENTS.contains("EVENT_COMMISSION_RUNG_ORIGIN_RECOVERY: u16 = 143"));
        assert!(SHARED_EVENTS.contains("EVENT_COMMISSION_REST_SELECTION_DIAGNOSTIC: u16 = 151"));
        assert!(!REPLIES.contains("rest_selection"));
    }

    #[test]
    fn trace_event_ids_match_shared_recovery_and_timing_markers() {
        assert!(SHARED_EVENTS.contains("EVENT_COMMISSION_RUNG_ORIGIN_RECOVERY: u16 = 143"));
        assert!(SHARED_EVENTS.contains("EVENT_COMMISSION_OUTER_SAFETY_TIMING: u16 = 144"));
    }
}
