#![no_std]
#![allow(dead_code)]

#[cfg(test)]
extern crate std;

#[path = "../../src/state/stepper.rs"]
mod stepper;

#[cfg(test)]
mod commissioning_board_contract_tests {
    const MAIN: &str = include_str!("../../src/main.rs");
    const FIRMWARE_CTX: &str = include_str!("../../src/firmware_ctx.rs");
    const BOARD: &str = include_str!("../../src/tmc_control/board.rs");
    const DRIVER: &str = include_str!("../../../../shared/foci-firmware/src/tmc_control/driver.rs");
    const TMC_DISPATCH: &str = include_str!("../../src/tmc_dispatch.rs");
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
    const SHARED_COMMISSION_REPLY_MANIFEST: &str =
        include_str!("../../../../shared/foci-firmware/src/commission_dispatch/replies.rs");
    const SHARED_COMMISSION_ERROR_TYPES: &str =
        include_str!("../../../../shared/foci-firmware/src/autotune/types.rs");
    const SHARED_CALIBRATION_TYPES: &str =
        include_str!("../../../../shared/foci-firmware/src/calibration.rs");
    const SHARED_FOCI_HANDLERS: &str =
        include_str!("../../../../shared/foci-firmware/src/ankyra/handlers/foci.rs");
    const BOARD_CARGO_TOML: &str = include_str!("../../Cargo.toml");
    const TRIGGER_DOMAIN: &str = include_str!("../../src/trigger_domain.rs");
    const BOARD_INIT: &str = include_str!("../../src/board_init.rs");
    const USB_TRACE: &str = include_str!("../../src/usb_trace.rs");
    const TMC_DIAGNOSTICS: &str = include_str!("../../src/tmc_diagnostics.rs");

    const REPLY_ID_WORST_CASE_BYTES: usize = 3;
    const KLIPPER_PAYLOAD_WITH_ID_MAX: usize = 59;

    fn extract_fn<'a>(text: &'a str, name: &str) -> &'a str {
        let marker = std::format!("fn {name}");
        let start = text
            .find(&marker)
            .unwrap_or_else(|| panic!("missing fn {name}"));
        let brace_start = text[start..]
            .find('{')
            .map(|offset| start + offset)
            .expect("unterminated function signature");
        let mut depth = 0usize;
        for (offset, ch) in text[brace_start..].char_indices() {
            match ch {
                '{' => depth += 1,
                '}' => {
                    depth -= 1;
                    if depth == 0 {
                        return &text[start..brace_start + offset + 1];
                    }
                }
                _ => {}
            }
        }
        panic!("unterminated fn {name}");
    }

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

    /// Same contract `assert_reply_contract` checks, against the shared
    /// manifest's `entry { ... }` text instead of a board's compiled
    /// REPLIES -- valid because `entry { name: <method>, ..., reply:
    /// <Type>, ..., fields: { <field>: <ty>, ... } }` contains the method
    /// name, then (searching forward) the reply type, then each field in
    /// order, the same shape `assert_reply_contract` already expects.
    fn assert_manifest_entry(method: &str, reply: &str, fields: &[&str]) {
        let start = SHARED_COMMISSION_REPLY_MANIFEST
            .find(&std::format!("name: {method}"))
            .unwrap_or_else(|| panic!("missing manifest entry {method}"));
        let after = &SHARED_COMMISSION_REPLY_MANIFEST[start..];
        // Bound the search to this one entry: many entries share common
        // field names (`oid: u8` alone appears in most of them), so without
        // an upper bound a missing/misordered field in this entry could be
        // silently satisfied by a *later* entry's matching text instead --
        // the check would pass while proving nothing about this method.
        // `after` starts at "name: <method>", i.e. strictly after this
        // entry's own opening "entry {", so the next occurrence of
        // "entry {" (if any) is the following entry's, never this one's.
        let entry_end = after.find("entry {").unwrap_or(after.len());
        let body = &after[..entry_end];
        assert!(
            body.contains(reply),
            "{method} manifest entry does not declare {reply}"
        );

        let mut cursor = 0;
        for field in fields {
            let offset = body[cursor..]
                .find(field)
                .unwrap_or_else(|| panic!("{method} manifest entry missing field {field}"));
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
    fn spi_dma_interrupt_priority_is_explicit_and_below_hard_real_time() {
        assert!(
            BOARD_INIT.contains(
                "config.dma_interrupt_priority = embassy_stm32::interrupt::Priority::P15;"
            ),
            "rcc_config() must set dma_interrupt_priority explicitly to P15, not inherit Embassy's P0 default"
        );
        assert!(
            MAIN.contains("board_init::verify_dma_interrupt_priority();"),
            "init() must call verify_dma_interrupt_priority() to check the programmed NVIC priority at boot"
        );
    }

    #[test]
    fn startup_establishes_pi_representation_before_adc_vm_capture() {
        let chip_identity = DRIVER
            .find("CHIPINFO_SI_TYPE")
            .expect("startup must verify the TMC4671 identity");
        let representation = DRIVER
            .find("bootstrap_pi_representation")
            .expect("startup must establish the fixed PI representation");
        let adc_vm = DRIVER
            .find("capture_adc_vm_offset")
            .expect("startup must retain ADC-VM capture");

        assert!(chip_identity < representation);
        assert!(representation < adc_vm);
        assert_eq!(DRIVER.matches("bootstrap_pi_representation").count(), 1);
        assert!(DRIVER.contains("PI representation failed at boot"));
        assert!(!MAIN.contains("0x15"));
        assert!(
            !MAIN.contains("bootstrap_pi_representation"),
            "bootstrap now lives in the shared driver"
        );
    }

    #[test]
    fn shutdown_drops_a_queued_homing_re_entry() {
        let shutdown = extract_fn(FIRMWARE_CTX, "enter_shutdown");
        assert!(shutdown.contains("HOMING_STATE.store(0, Relaxed)"));
        assert!(
            shutdown.contains("HOMING_REENTER_PENDING.store(false, Relaxed)"),
            "shutdown must drop a queued homing re-entry or the next restore re-arms homing"
        );
    }

    #[test]
    fn step_hold_is_gated_on_the_homing_start_clock() {
        assert_eq!(
            MAIN.matches("step_held_for_homing(stepper.next_step_clock())")
                .count(),
            1,
            "the only homing hold sits where a due step is emitted and keys off \
             that step's scheduled clock"
        );
        assert!(
            !MAIN.contains("HOMING_STATE.load(Relaxed) == 1"),
            "a hold keyed on HOMING_STATE alone stalls the previous move's \
             in-flight steps past their deadline"
        );
    }

    #[test]
    fn homing_start_notification_is_an_explicit_no_op() {
        let implementation = extract_fn(FIRMWARE_CTX, "notify_homing_start");
        assert!(
            implementation.contains("maintenance loop is always armed"),
            "the no-op must retain the OpenFFBoard-specific rationale"
        );

        let open = implementation.find('{').expect("notification body");
        let close = implementation.rfind('}').expect("notification body");
        let executable = implementation[open + 1..close]
            .lines()
            .map(str::trim)
            .filter(|line| !line.is_empty() && !line.starts_with("//"))
            .collect::<std::string::String>();
        assert_eq!(
            executable, "Ok(())",
            "OpenFFBoard must acknowledge homing without queue, signal, or event side effects"
        );
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
    fn board_publishes_the_proportional_schema_revision() {
        assert!(CONSTANTS.contains("pub const PROPORTIONAL_EVIDENCE_SCHEMA_REVISION: u32 = 14;"));
    }

    #[test]
    fn board_owns_and_invalidates_channel_indexed_integral_candidates() {
        assert!(MAIN.contains("IntegralRetentionSlots<"));
        assert!(MAIN.contains("{ TMC_CHANNEL_COUNT as usize }"));
        assert_eq!(
            MAIN.matches("integral_candidates.invalidate_all()").count(),
            2
        );
        assert!(MAIN.contains(".slot_mut(0)"));
        assert!(BOARD.contains("CommissionRetention::new"));
        assert!(REQUEST.contains("handle_tmc_control_request_with_stages"));
        assert!(REQUEST.contains(".slot_mut(req.channel)"));
    }

    #[test]
    fn board_retains_integral_authority_from_the_plan_event() {
        // Nonterminal classification stays outside the terminal async dispatch
        // future. foci-firmware's `scheduled_step` size gate pins that budget,
        // so Stage-C authority is installed synchronously from the plan event
        // rather than by routing polls through the staged poll driver.
        assert!(
            DRIVER.contains(".classify_commission_poll(poll)"),
            "nonterminal polls must stay out of the async dispatch future"
        );
        assert!(
            !MAIN.contains("drive_commissioning_poll_with_stages")
                && !DRIVER.contains("drive_commissioning_poll_with_stages"),
            "the staged poll driver pulls terminal dispatch into every poll"
        );

        let arm = DRIVER
            .find("CommissionEvent::VelocityIntegralPlan")
            .expect("driver must handle the Stage-C plan event");
        let body = &DRIVER[arm..];
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
    fn board_reaches_the_staged_request_dispatch_that_owns_integral_resume() {
        // Two shared-firmware fixes for the Stage-C authority landed on paths
        // this board does not execute, and shared tests passed both times. The
        // board's own call shape is what decides whether new dispatch arms --
        // the Stage-C resume among them -- are reachable here at all.
        assert!(
            REQUEST.contains("handle_tmc_control_request_with_stages"),
            "tune requests must reach the staged request dispatch"
        );
        assert!(
            REQUEST.contains("integral_candidate"),
            "the staged dispatch must be given the retained Stage-C slot"
        );
        // Retention is what a resume consumes. It is installed synchronously
        // from the plan event because nonterminal classification stays outside
        // the terminal async dispatch future on this board.
        assert!(
            DRIVER.contains("install_authority_from_plan"),
            "the plan event must retain authority for a later resume"
        );
        assert!(
            DRIVER.contains(".classify_commission_poll(poll)"),
            "nonterminal polls must stay out of the async dispatch future"
        );
    }

    #[test]
    fn board_polls_the_terminal_restore_and_cancels_it_on_every_exit() {
        // The terminal restore is a poll-driven machine on the runtime, not an
        // awaited chain inside this task's future. Every commissioning-loop
        // exit must latch a cancel, and the loop must drain that latch before
        // touching the channel again.
        assert!(
            !MAIN.contains("drive_commissioning_terminal_with_stages"),
            "the board must not await the whole terminal chain in one future"
        );
        assert!(DRIVER.contains("begin_commissioning_terminal"));
        assert!(DRIVER.contains("poll_commissioning_terminal"));
        assert!(DRIVER.contains("is_restore_pending"));
        assert!(
            MAIN.contains("restore_cancel_pending") && MAIN.contains("drive_restore_cancel"),
            "the loop must drain a latched restore cancel"
        );
        assert_eq!(
            MAIN.matches("request_restore_cancel").count(),
            2,
            "the two loop sites that latch a restore cancel: the commission \
             wait-exit check and the in-commissioning request cancel"
        );
    }

    #[test]
    fn board_forwards_the_complete_integral_reply_family() {
        for method in [
            "foci_commissioning_workflow_plan",
            "foci_velocity_integral_plan_core",
            "foci_velocity_integral_plan_geometry",
            "foci_velocity_integral_plan_authority",
            "foci_velocity_integral_plan_timing",
            "foci_velocity_integral_plan_travel",
            "foci_velocity_integral_plan_rung",
            "foci_velocity_integral_terminal",
            "foci_robustness_reversal_terminal",
        ] {
            assert!(
                SHARED_COMMISSION_REPLY_MANIFEST.contains(&std::format!("name: {method}")),
                "missing manifest entry {method}"
            );
        }
    }

    #[test]
    fn board_forwards_robustness_cycle_evidence_via_the_manifest() {
        assert!(
            SHARED_COMMISSION_REPLY_MANIFEST.contains("name: foci_robustness_cycle_evidence"),
            "missing manifest entry foci_robustness_cycle_evidence"
        );
    }

    #[test]
    fn board_invokes_the_shared_commission_reply_manifest() {
        assert!(
            REPLIES.contains("commission_reply_forwarding_impls"),
            "board no longer wires the shared CommissionReplySink manifest \
             (foci_firmware::__commission_reply_manifest!/\
             foci_firmware_macros::commission_reply_forwarding_impls) -- \
             a board that stops calling it silently drops every \
             manifest-driven reply"
        );
    }

    #[test]
    fn board_cargo_toml_depends_on_foci_firmware_macros() {
        assert!(
            BOARD_CARGO_TOML.contains("foci-firmware-macros"),
            "board's Cargo.toml no longer depends on foci-firmware-macros, \
             so the shared manifest invocation above cannot resolve"
        );
    }

    #[test]
    fn board_forwards_integral_authority_without_narrowing() {
        assert_manifest_entry(
            "foci_velocity_integral_plan_authority",
            "FociVelocityIntegralPlanAuthority",
            &[
                "directional_membership: u32",
                "joint_membership: u32",
                "directional_validity: u8",
                "reduced_margin: u8",
            ],
        );
        assert_manifest_entry(
            "foci_velocity_integral_terminal",
            "FociVelocityIntegralTerminal",
            &["oid: u8", "payload: &[u8]"],
        );
    }

    #[test]
    fn board_publishes_the_integral_schema_and_geometry_constants() {
        for declaration in [
            "pub const INTEGRAL_EVIDENCE_SCHEMA_REVISION: u32 = 13;",
            "pub const INTEGRAL_MAX_POSITIVE_RUNGS: u32 = 30;",
            "pub const INTEGRAL_OBSERVATIONS_PER_DIRECTION: u32 = 4;",
            "pub const INTEGRAL_OBSERVATIONS_PER_RUNG: u32 = 8;",
        ] {
            assert!(CONSTANTS.contains(declaration), "missing {declaration}");
        }
        for constant in [
            "PROPORTIONAL_EVIDENCE_SCHEMA_REVISION",
            "INTEGRAL_EVIDENCE_SCHEMA_REVISION",
            "INTEGRAL_MAX_POSITIVE_RUNGS",
            "INTEGRAL_OBSERVATIONS_PER_DIRECTION",
            "INTEGRAL_OBSERVATIONS_PER_RUNG",
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
        assert_manifest_entry(
            "foci_commissioning_workflow_plan",
            "FociCommissioningWorkflowPlan",
            &["shape: u8"],
        );
        assert!(!REPLIES.contains("foci_combined_"));
        assert!(!REPLIES.contains("foci_velocity_combined_"));
    }

    #[test]
    fn board_forwards_recovery_plan_geometry_but_not_the_retired_recovery_reply() {
        assert!(
            SHARED_COMMISSION_REPLY_MANIFEST.contains("name: foci_velocity_integral_plan_recovery"),
            "missing manifest entry foci_velocity_integral_plan_recovery"
        );
        for retired in [
            "foci_rung_origin_recovery_core",
            "foci_rung_origin_recovery_position",
            "foci_rung_origin_recovery_timing",
            "foci_rung_origin_recovery_limits",
            "foci_rung_origin_recovery_summary",
        ] {
            assert!(!REPLIES.contains(retired), "retained board reply {retired}");
        }
        assert!(CONSTANTS.contains("pub const PROPORTIONAL_EVIDENCE_SCHEMA_REVISION: u32 = 14;"));
        assert!(CONSTANTS.contains("pub const INTEGRAL_EVIDENCE_SCHEMA_REVISION: u32 = 13;"));
        assert!(SHARED_EVENTS.contains("EVENT_COMMISSION_RUNG_ORIGIN_RECOVERY: u16 = 143"));
        assert!(!REPLIES.contains("rest_selection"));
    }

    #[test]
    fn trace_event_ids_match_shared_recovery_and_timing_markers() {
        assert!(SHARED_EVENTS.contains("EVENT_COMMISSION_RUNG_ORIGIN_RECOVERY: u16 = 143"));
        assert!(SHARED_EVENTS.contains("EVENT_COMMISSION_OUTER_SAFETY_TIMING: u16 = 144"));
    }

    #[test]
    fn board_forwards_the_tune_result_provenance_fields() {
        assert_reply_contract(
            "foci_tune_result",
            "FociTuneResult",
            &[
                "oid: u8",
                "status: u8",
                "velocity_p: u16",
                "velocity_i: u16",
                "velocity_limit: u32",
                "position_p: u16",
                "position_i: u16",
                "velocity_filter_hz: u16",
                "position_filter_hz: u16",
                "flux_filter_hz: u16",
                "torque_filter_hz: u16",
                "warning_code: u8",
                "probed_velocity_mrev_s: u32",
                "d_eq_q: i32",
                "confidence_q: u16",
                "band_lower_percent: u8",
                "band_upper_percent: u8",
                "band_position_q: u16",
            ],
        );
    }

    #[test]
    fn dump_registers_emits_velocity_clamp_values_after_sub_registers() {
        let sub_registers = TMC_DISPATCH
            .find("DUMP_SUB_REGISTERS")
            .expect("dump dispatch must read the sub-register table");
        let clamp_values = TMC_DISPATCH
            .find("clamp_dump_values")
            .expect("dump dispatch must emit the velocity-clamp values");
        let done = TMC_DISPATCH
            .find("FociDumpDone")
            .expect("dump dispatch must send the done reply");

        assert!(
            sub_registers < clamp_values,
            "clamp values must be emitted after the sub-register loop"
        );
        assert!(
            clamp_values < done,
            "clamp values must be emitted before the done reply"
        );
        assert!(TMC_DISPATCH.contains("velocity_clamp_latched()"));
        assert!(TMC_DISPATCH.contains("velocity_clamp_count()"));
    }

    #[test]
    fn p1_unmasked_drain_never_reacquires_a_critical_section() {
        let unmasked_start = TRIGGER_DOMAIN
            .find("QueueAccess::Unmasked => {")
            .expect("stop_physical_stepper_inner must branch on QueueAccess::Unmasked");
        let locked_start = TRIGGER_DOMAIN
            .find("QueueAccess::LockedInPlace => {")
            .expect("the Unmasked arm must be followed by the LockedInPlace arm");
        assert!(unmasked_start < locked_start);
        let unmasked_arm = &TRIGGER_DOMAIN[unmasked_start..locked_start];

        let detach_pos = unmasked_arm
            .find("detach_consumer()")
            .expect("the unmasked arm must detach the consumer before draining");
        let dequeue_pos = unmasked_arm.find("consumer.dequeue()").expect(
            "the unmasked arm must actually drain the detached consumer, \
                     not just call detach/attach with nothing in between",
        );
        let attach_pos = unmasked_arm
            .find("attach_consumer(")
            .expect("the unmasked arm must reattach the consumer after draining");

        assert!(
            detach_pos < dequeue_pos,
            "the drain loop must run after detaching the consumer, not before"
        );
        assert!(
            dequeue_pos < attach_pos,
            "the drain loop must run before reattaching the consumer, not after"
        );
        assert!(
            !unmasked_arm.contains("interrupt::free"),
            "the unmasked arm must never reacquire a critical section directly -- \
             masking belongs only inside detach_consumer/attach_consumer, not the drain loop"
        );
    }

    /// Return the index of the `}` matching the `{` at `open_brace_pos`
    /// (which must itself be a `{` character), by counting brace depth.
    /// Comment- and string-literal-blind, like every other source-contract
    /// check in this module -- adequate for this crate's existing use of
    /// `include_str!` guards, not a real parser.
    fn find_matching_close_brace(text: &str, open_brace_pos: usize) -> usize {
        let mut depth = 0i32;
        for (offset, ch) in text[open_brace_pos..].char_indices() {
            match ch {
                '{' => depth += 1,
                '}' => {
                    depth -= 1;
                    if depth == 0 {
                        return open_brace_pos + offset;
                    }
                }
                _ => {}
            }
        }
        panic!("no matching closing brace found for the brace at byte {open_brace_pos}");
    }

    #[test]
    fn locked_arm_hands_off_instead_of_draining_inline() {
        let locked_open_marker = TRIGGER_DOMAIN
            .find("QueueAccess::LockedInPlace => {")
            .expect("stop_physical_stepper_inner must branch on QueueAccess::LockedInPlace");
        let locked_open_brace = locked_open_marker + "QueueAccess::LockedInPlace => ".len();
        assert_eq!(
            &TRIGGER_DOMAIN[locked_open_brace..locked_open_brace + 1],
            "{"
        );
        let locked_close_brace = find_matching_close_brace(TRIGGER_DOMAIN, locked_open_brace);
        let locked_arm = &TRIGGER_DOMAIN[locked_open_brace..=locked_close_brace];

        assert!(
            locked_arm.contains("detach_consumer("),
            "the LockedInPlace arm must detach the consumer, not borrow it in place"
        );
        assert!(
            !locked_arm.contains(".dequeue()"),
            "the LockedInPlace arm must never drain the queue itself -- that is exactly \
             the critical-section-duration problem this ticket fixes. Draining belongs \
             only in drain_all_populated_slots, run by trigger_drain_task with interrupts \
             enabled."
        );
        assert!(
            locked_arm.contains("store_pending_drain("),
            "the LockedInPlace arm must hand the detached consumer to trigger_drain_task"
        );
        assert!(
            !locked_arm.contains("record_physical_stop("),
            "diagnostic recording for a masked stop that was successfully handed off must \
             happen in drain_all_populated_slots (after the drain, with real segment/step \
             counts), not inside stop_physical_stepper_inner's LockedInPlace arm itself"
        );
    }

    #[test]
    fn locked_stop_still_resets_gpio_state_unconditionally() {
        // Regression guard for a defect caught in this plan's own cross-agent
        // review: an early `return` *inside* the LockedInPlace arm would skip
        // direction/STEP GPIO reset, corrupting the next segment's pulse
        // parity. This must check from the start of the match (so it covers
        // the match arms themselves, where the original defect actually
        // lived) through the end of the function -- an earlier draft of this
        // guard checked only after the match's closing brace, which does not
        // cover a `return` inside an arm and would not have caught the
        // defect it was written to guard against. A legitimate early
        // `return` exists before the match (the
        // `!PHYSICAL_STEPPER_INDICES.contains(...)` bounds check at the top
        // of the function) and must stay outside this checked region.
        let fn_open_marker = TRIGGER_DOMAIN
            .find("fn stop_physical_stepper_inner(")
            .expect("trigger_domain.rs must define stop_physical_stepper_inner");
        let fn_open_brace = TRIGGER_DOMAIN[fn_open_marker..]
            .find('{')
            .map(|offset| fn_open_marker + offset)
            .expect("stop_physical_stepper_inner must have a body");
        let fn_close_brace = find_matching_close_brace(TRIGGER_DOMAIN, fn_open_brace);
        let fn_body = &TRIGGER_DOMAIN[fn_open_brace..=fn_close_brace];

        let match_pos = fn_body
            .find("match access {")
            .expect("stop_physical_stepper_inner must branch on QueueAccess");
        let from_match_onward = &fn_body[match_pos..];

        // Word-boundary match, not a plain substring search: the region
        // legitimately contains the comment word "returns" (describing
        // detach_consumer's Option), which a bare `contains("return")`
        // would misfire on even though it is not the `return` keyword.
        let has_return_keyword = from_match_onward.match_indices("return").any(|(pos, _)| {
            let before_ok = from_match_onward[..pos]
                .chars()
                .next_back()
                .map_or(true, |c| !c.is_alphanumeric() && c != '_');
            let after_ok = from_match_onward["return".len() + pos..]
                .chars()
                .next()
                .map_or(true, |c| !c.is_alphanumeric() && c != '_');
            before_ok && after_ok
        });
        assert!(
            !has_return_keyword,
            "no `return` is allowed anywhere from the start of the QueueAccess match \
             through the end of stop_physical_stepper_inner -- this includes inside \
             either match arm, not just after the match closes -- because \
             direction-GPIO and STEP-GPIO reset must run unconditionally for every \
             stop, and an early return from an arm would skip them"
        );

        let direction_gpio_pos = TRIGGER_DOMAIN
            .find("set_odr(0, embassy_stm32::pac::gpio::vals::Odr::LOW)")
            .expect("stop_physical_stepper_inner must reset the direction GPIO to LOW");
        let step_gpio_pos = TRIGGER_DOMAIN
            .find("set_odr(11, embassy_stm32::pac::gpio::vals::Odr::LOW)")
            .expect("stop_physical_stepper_inner must reset the STEP GPIO to LOW");
        let locked_open_marker = TRIGGER_DOMAIN
            .find("QueueAccess::LockedInPlace => {")
            .expect("stop_physical_stepper_inner must branch on QueueAccess::LockedInPlace");
        let locked_open_brace = locked_open_marker + "QueueAccess::LockedInPlace => ".len();
        let locked_close_brace = find_matching_close_brace(TRIGGER_DOMAIN, locked_open_brace);

        assert!(
            locked_close_brace < direction_gpio_pos,
            "the direction GPIO reset must run after the LockedInPlace arm's \
             actual closing brace, not nested inside it"
        );
        assert!(
            locked_close_brace < step_gpio_pos,
            "the STEP GPIO reset must run after the LockedInPlace arm's actual \
             closing brace, not nested inside it"
        );
        assert!(
            direction_gpio_pos < step_gpio_pos,
            "the direction GPIO reset must run before the STEP GPIO reset, \
             matching the real code's order"
        );
    }

    #[test]
    fn drain_all_populated_slots_records_after_draining() {
        let fn_open_marker = TRIGGER_DOMAIN
            .find("pub(crate) fn drain_all_populated_slots()")
            .expect("trigger_domain.rs must define drain_all_populated_slots");
        let fn_open_brace = TRIGGER_DOMAIN[fn_open_marker..]
            .find('{')
            .map(|offset| fn_open_marker + offset)
            .expect("drain_all_populated_slots must have a body");
        let fn_close_brace = find_matching_close_brace(TRIGGER_DOMAIN, fn_open_brace);
        let fn_body = &TRIGGER_DOMAIN[fn_open_brace..=fn_close_brace];

        let dequeue_pos = fn_body
            .find(".dequeue()")
            .expect("drain_all_populated_slots must actually drain the detached consumer");
        let record_pos = fn_body
            .find("record_physical_stop(")
            .expect("drain_all_populated_slots must record the stop after draining it");
        let attach_pos = fn_body
            .find("attach_consumer(")
            .expect("drain_all_populated_slots must reattach the consumer after draining");

        assert!(
            dequeue_pos < record_pos,
            "must record the drained segment/step counts after actually draining, \
             not before (the counts would be wrong -- always zero)"
        );
        assert!(
            record_pos < attach_pos,
            "must record before reattaching, matching the existing Unmasked-arm ordering \
             (detach -> drain -> record -> reattach)"
        );
        assert!(
            !fn_body.contains("interrupt::free"),
            "drain_all_populated_slots must never hold a critical section across the \
             drain loop -- masking belongs only inside take_pending_drain/attach_consumer, \
             never across the dequeue loop"
        );
    }

    #[test]
    fn board_references_the_foci_commission_cancel_command() {
        assert!(
            SHARED_FOCI_HANDLERS.contains("pub fn foci_commission_cancel"),
            "foci_commission_cancel command handler must be defined"
        );
        assert!(
            SHARED_FOCI_HANDLERS.contains("#[klipper_command]\npub fn foci_commission_cancel"),
            "foci_commission_cancel must be marked with #[klipper_command]"
        );
    }

    #[test]
    fn commission_error_cancelled_has_status_code_74() {
        assert!(
            SHARED_COMMISSION_ERROR_TYPES.contains("Cancelled => 74"),
            "CommissionError::Cancelled must map to status code 74"
        );
    }

    #[test]
    fn calibration_error_cancelled_has_status_code_10() {
        assert!(
            SHARED_CALIBRATION_TYPES.contains("Cancelled => 10"),
            "CalibrationError::Cancelled must map to status code 10"
        );
    }

    #[test]
    fn sampler_reads_position_error_right_after_position_actual() {
        let body = USB_TRACE
            .split("pub(crate) async fn sample_tick")
            .nth(1)
            .expect("sample_tick present");
        let at = |needle: &str| {
            body.find(needle)
                .unwrap_or_else(|| panic!("missing {needle}"))
        };
        let step = at("stepper.position.load(Relaxed)");
        let actual = at("read_register_trace(tmc, PID_POSITION_ACTUAL");
        let error = at("read_sub_register_trace(tmc, PID_ERROR_PID_POSITION_ERROR");
        let velocity = at("read_register_trace(tmc, PID_VELOCITY_ACTUAL");
        assert!(step < actual && actual < error && error < velocity);
        assert!(!body.contains("step_position.wrapping_sub(position_actual)"));
    }

    #[test]
    fn board_owns_the_tmc_status_input() {
        assert!(
            BOARD_INIT.contains(
                "pub fn init_tmc_status_pin(pe8: Peri<'static, peripherals::PE8>) -> Input<'static>"
            ),
            "the STATUS input must be returned, not forgotten"
        );
        assert!(!BOARD_INIT.contains("core::mem::forget(status_pin)"));
        assert!(MAIN.contains("tmc_status: Input<'static>"));
        assert!(MAIN.contains("status_pin_asserted"));
    }

    #[test]
    fn control_loop_drives_one_unit_per_iteration_and_never_loops_over_channels() {
        let task_start = MAIN
            .find("async fn tmc_control(")
            .expect("tmc_control task");
        let task_end = MAIN[task_start..]
            .find("#[task(")
            .map_or(MAIN.len(), |o| task_start + o);
        let body = &MAIN[task_start..task_end];
        assert!(
            !body.contains("for channel in"),
            "no channel loops inside the future"
        );
        assert!(
            !body.contains("'commission_poll"),
            "the commission poll loop is gone"
        );
        assert_eq!(
            body.matches(".drive(").count(),
            1,
            "exactly one drive await site"
        );
        assert_eq!(body.matches("scheduler.pick(").count(), 1);
        assert_eq!(
            body.matches("scheduler.record(").count(),
            1,
            "finalize and drive share one record site"
        );
        assert!(body.contains("select(rx.recv(), control_sleep)"));
        assert!(body.contains("status_pin_asserted"));
    }

    #[test]
    fn maintenance_always_armed_is_explicit_before_the_wait_deadline() {
        let body = extract_fn(MAIN, "tmc_control");
        let mode = body
            .find("scheduler.set_mode(0, driver.mode(), now_us());")
            .expect("channel mode update");
        let arm = body
            .find("scheduler.set_maintenance_arm(MaintenanceArm::Armed, now_us());")
            .expect("explicit always-armed maintenance policy");
        let deadline = body
            .find(".earliest_due_us()")
            .expect("scheduler wait deadline");

        assert!(mode < arm && arm < deadline);
        assert_eq!(body.matches("scheduler.set_maintenance_arm(").count(), 1);
        assert!(!body.contains("MaintenanceArm::Disarmed"));
    }

    #[test]
    fn maintenance_always_armed_diagnostic_reports_state_and_transition_counts() {
        let body = extract_fn(MAIN, "tmc_control");

        assert!(body.contains("let arm = scheduler.maintenance_arm_stats();"));
        assert!(body.contains("maint arm={:?} a2d={} d2a={}"));
        assert!(body.contains("arm.state"));
        assert!(body.contains("arm.armed_to_disarmed"));
        assert!(body.contains("arm.disarmed_to_armed"));
        assert_eq!(body.matches("maintenance_arm_stats()").count(), 1);
    }

    #[test]
    fn fault_active_is_cleared_only_after_the_interlock_recheck() {
        let task_start = MAIN
            .find("async fn tmc_control(")
            .expect("tmc_control task");
        let body = &MAIN[task_start..];
        assert_eq!(
            body.matches("FAULT_ACTIVE.store(false").count(),
            1,
            "one emergency-disable clear shared by the fault and deadline-miss paths"
        );
        let clear = body
            .find("FAULT_ACTIVE.store(false")
            .expect("the emergency-disable clear");
        // Wide enough to span the whole containment block from the enable-pin
        // drop down to the clear, with margin, while stopping short of any
        // earlier call site that would satisfy these assertions spuriously.
        let window = &body[clear.saturating_sub(3_000)..clear];
        assert!(
            window.contains("status_pin.is_high()"),
            "the clear follows a fresh pin poll"
        );
        assert!(
            window.contains("TMC_DISABLE_SIGNAL.signaled()"),
            "the clear follows a pending-signal check"
        );
        assert!(
            window.contains("enable_pin.set_low()"),
            "the clear follows lowering the enable pin"
        );
        assert!(
            window.contains("record_stepper_event("),
            "the clear follows the STATUS diagnostic"
        );
        // The full diagnostic is recorded before the latch drops, so a fault
        // that survives containment is still reported with its STATUS word.
        let pin_poll = window
            .find("status_pin.is_high()")
            .expect("the pin re-check inside the containment block");
        let trace_event = window
            .find("SUBSYSTEM_SAFETY_SHUTDOWN")
            .expect("the safety-shutdown trace event");
        let log = window
            .find("emergency disable: STATUS=")
            .expect("the emergency-disable log");
        assert!(
            trace_event < pin_poll && log < pin_poll,
            "the trace event and log precede the interlock re-check"
        );
        assert!(
            body.contains("|| status_pin_asserted"),
            "the polled level is part of the containment predicate"
        );
        assert!(
            body.contains("enable_status_exti()"),
            "EXTI8 is enabled when bootstrap reports Ready"
        );
    }

    #[test]
    fn control_detail_reads_the_real_interim_pidout_target_flux_register() {
        assert!(
            !USB_TRACE.contains("interim_pidout_target_flux: 0,"),
            "interim_pidout_target_flux must not be a hardcoded placeholder"
        );
        let body = USB_TRACE
            .split("let detail = ControlDetailSample {")
            .nth(1)
            .expect("ControlDetailSample construction present");
        let flux = body
            .find("interim_pidout_target_flux:")
            .expect("interim_pidout_target_flux field present");
        let flux_field = &body[flux..];
        assert!(
            flux_field
                .starts_with("interim_pidout_target_flux: read_sub_register_counting_errors(")
                && flux_field.contains("INTERIM_PIDOUT_TARGET_FLUX,"),
            "interim_pidout_target_flux must read the real sub-register like its sibling fields"
        );
    }

    #[test]
    fn invalid_tmc_channel_sends_a_host_visible_diagnostic_reply() {
        assert!(
            TMC_DISPATCH.contains("FociDiagnostic"),
            "an invalid TMC channel must report a FociDiagnostic reply, not just a defmt log"
        );
        assert!(
            TMC_DISPATCH.contains("FaultClass::InvalidChannel"),
            "the diagnostic reply must use the InvalidChannel fault class"
        );
    }

    #[test]
    fn diagnostic_dispatch_uses_raw_lease_expired_not_the_classifier() {
        assert!(
            TMC_DIAGNOSTICS.contains("tmc_control::effects::lease_expired("),
            "diagnostic dispatch must wire the host-liveness check to the raw \
             lease_expired function"
        );
        assert!(
            !TMC_DIAGNOSTICS.contains("classify_lease_check")
                && !TMC_DIAGNOSTICS.contains("LeaseCheckDisposition"),
            "diagnostic dispatch must not use classify_lease_check -- its \
             shutdown-active suppression does not apply to a diagnostic \
             runner's one-shot poll"
        );
    }

    #[test]
    fn safety_bootstrap_runs_before_the_first_status_pin_read_and_before_drive() {
        let tmc_control_fn = extract_fn(MAIN, "tmc_control");
        let bootstrap_pos = tmc_control_fn
            .find("bootstrap_tmc_safety(")
            .expect("eager safety-bootstrap call must exist in tmc_control()");
        let status_read_pos = tmc_control_fn
            .find("status_pin.is_high()")
            .expect("the control loop's STATUS-pin read must exist in tmc_control()");
        let drive_call_pos = tmc_control_fn
            .find("driver.drive(unit, &mut backend, ctx_unit)")
            .expect("the drive() call must exist in tmc_control()");

        assert!(
            bootstrap_pos < status_read_pos,
            "safety bootstrap must run before the control loop's first STATUS-pin \
             read -- a stale STATUS_MASK from a prior session can otherwise trip \
             the emergency-disable branch before bootstrap ever corrects it, \
             permanently skipping the correction on every subsequent boot"
        );
        assert!(
            bootstrap_pos < drive_call_pos,
            "safety bootstrap must run before the drive() call"
        );
    }

    #[test]
    fn specific_tmc_fault_classification_is_reported_before_stepper_too_far_in_past() {
        let adc_pos = MAIN
            .find("EmergencyDisableFault::AdcCurrentClipped =>")
            .expect("ADC-clip fault arm");
        let status_read_pos = MAIN
            .find("EmergencyDisableFault::StatusReadFailed =>")
            .expect("status-read-failed fault arm");
        let deadline_pos = MAIN
            .find("ShutdownReason::StepperTooFarInPast")
            .expect("StepperTooFarInPast shutdown call");
        assert!(
            adc_pos < deadline_pos && status_read_pos < deadline_pos,
            "a specific TMC fault classification (ADC current clipped, STATUS_FLAGS read \
             failed) must be checked and reported before falling back to \
             StepperTooFarInPast -- a stepper naturally misses its own deadline the \
             instant its motor is emergency-disabled, so an unconditional preference for \
             the deadline-miss label masks the real electrical fault that caused it"
        );

        let generic_pos = MAIN
            .find("EmergencyDisableFault::Generic =>")
            .expect("Generic fault arm");
        assert!(
            generic_pos < deadline_pos,
            "StepperTooFarInPast must be reported only from within the Generic-fault \
             arm (no specific TMC fault was observed), not as an unconditional \
             override ahead of fault classification"
        );
    }
}
