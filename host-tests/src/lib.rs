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
    fn board_owns_one_stage_b_candidate_and_threads_it_through_dispatch() {
        assert_eq!(MAIN.matches("StageBCandidateSlot::new()").count(), 1);
        assert!(MAIN.contains("&mut stage_b_candidate"));
        assert!(REQUEST.contains("handle_tmc_control_request_with_stages"));
    }

    #[test]
    fn board_forwards_the_complete_stage_b_reply_family() {
        for method in [
            "foci_velocity_structured_boundary",
            "foci_velocity_directional_region_core",
            "foci_velocity_directional_region_model",
            "foci_velocity_directional_region_rates",
            "foci_velocity_directional_region_boundary",
            "foci_velocity_joint_region",
            "foci_velocity_stage_b_handoff_core",
            "foci_velocity_stage_b_nomination",
            "foci_velocity_stage_b_directional_handoff",
            "foci_velocity_stage_b_reproduction_v3_core",
            "foci_velocity_stage_b_reproduction_v3_membership",
            "foci_velocity_stage_b_reproduction_v3_pooled",
            "foci_velocity_stage_b_reproduction_v3_common",
            "foci_velocity_stage_b_reproduction_v3_coverage",
            "foci_velocity_stage_b_reproduction_v3_digest",
            "foci_velocity_stage_b_terminal_core",
            "foci_velocity_stage_b_terminal_identity",
            "foci_velocity_stage_b_terminal_interval",
        ] {
            assert!(REPLIES.contains(method), "missing board reply {method}");
        }
    }

    #[test]
    fn board_publishes_the_stage_b_schema_revision() {
        assert!(CONSTANTS.contains("pub const STAGE_B_EVIDENCE_SCHEMA_REVISION: u32 = 3;"));
    }

    #[test]
    fn board_owns_and_invalidates_one_stage_c_candidate() {
        assert_eq!(MAIN.matches("StageCCandidateSlot::new()").count(), 1);
        assert_eq!(MAIN.matches("stage_c_candidate.observe_fault()").count(), 2);
        assert!(MAIN.contains("CommissionRetention::new"));
        assert!(REQUEST.contains("handle_tmc_control_request_with_stages"));
        assert!(REQUEST.contains("stage_c_candidate"));
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
        ] {
            assert!(REPLIES.contains(method), "missing board reply {method}");
        }
    }

    #[test]
    fn board_publishes_the_stage_c_schema_and_geometry_constants() {
        for declaration in [
            "pub const STAGE_C_EVIDENCE_SCHEMA_REVISION: u32 = 1;",
            "pub const STAGE_C_MAX_POSITIVE_RUNGS: u32 = 30;",
            "pub const STAGE_C_OBSERVATIONS_PER_DIRECTION: u32 = 4;",
            "pub const STAGE_C_OBSERVATIONS_PER_RUNG: u32 = 8;",
        ] {
            assert!(CONSTANTS.contains(declaration), "missing {declaration}");
        }
        for constant in [
            "STAGE_C_EVIDENCE_SCHEMA_REVISION",
            "STAGE_C_MAX_POSITIVE_RUNGS",
            "STAGE_C_OBSERVATIONS_PER_DIRECTION",
            "STAGE_C_OBSERVATIONS_PER_RUNG",
        ] {
            assert!(PROVIDER.contains(constant), "provider omits {constant}");
        }
    }

    #[test]
    fn board_forwards_exact_consensus_reply_contracts() {
        assert_reply_contract(
            "foci_velocity_rung_consensus_core",
            "FociVelocityRungConsensusCore",
            &[
                "oid: u8",
                "run_sequence: u32",
                "evidence_sequence: u16",
                "fragment: u8",
                "rung_index: u8",
                "velocity_p: u16",
                "forward_class: u8",
                "reverse_class: u8",
                "forward_collected_mask: u8",
                "reverse_collected_mask: u8",
                "forward_eligible_mask: u8",
                "reverse_eligible_mask: u8",
                "forward_included_mask: u8",
                "reverse_included_mask: u8",
                "forward_component_count: u8",
                "reverse_component_count: u8",
                "forward_operable_count: u8",
                "reverse_operable_count: u8",
                "flags: u8",
            ],
        );
        assert_reply_contract(
            "foci_velocity_rung_consensus_component",
            "FociVelocityRungConsensusComponent",
            &[
                "oid: u8",
                "run_sequence: u32",
                "evidence_sequence: u16",
                "fragment: u8",
                "rung_index: u8",
                "direction: u8",
                "component_index: u8",
                "low_q16: i32",
                "high_q16: i32",
            ],
        );
        assert_reply_contract(
            "foci_velocity_rung_consensus_pool",
            "FociVelocityRungConsensusPool",
            &[
                "oid: u8",
                "run_sequence: u32",
                "evidence_sequence: u16",
                "fragment: u8",
                "rung_index: u8",
                "direction: u8",
                "pooled_q16: i32",
                "pooled_low_q16: i32",
                "pooled_high_q16: i32",
                "variance_floor_observations: u8",
                "mean_min_mantissa: i32",
                "mean_max_mantissa: i32",
                "envelope_low_mantissa: i32",
                "envelope_high_mantissa: i32",
                "rate_shift: u8",
            ],
        );
        assert!(!REPLIES.contains("FociVelocityRungVerdictBand"));
        assert!(!REPLIES.contains("FociVelocityRungVerdictQuality"));
    }

    #[test]
    fn consensus_replies_fit_the_board_wire_budget() {
        assert_wire_budget(
            "consensus core",
            &[2, 5, 3, 2, 2, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2],
        );
        assert_wire_budget("consensus component", &[2, 5, 3, 2, 2, 2, 2, 5, 5]);
        assert_wire_budget(
            "consensus pool",
            &[2, 5, 3, 2, 2, 2, 5, 5, 5, 2, 5, 5, 5, 5, 2],
        );
    }
}
