//! Board-local ankyra provider aggregating all binary commands, replies,
//! outputs, and enumerations for openffboard-fw.
//!
//! Consumed by `ankyra_config!` in `main.rs`. The provider macro's list
//! entries accept bare idents and `crate::…`-prefixed paths only; generic
//! arguments are rejected by `ProviderPath::parse`, so the two
//! lifetime-parameterized reply structs (`FociTraceData<'a>`,
//! `SpiTransferResponse<'a>`) are listed by bare path — the macro resolves
//! each to the descriptor call `crate::ankyra::replies::__ankyra_descriptor_<Leaf>()`
//! emitted by `#[klipper_reply]`, which does not depend on the lifetime.
//!
//! Handler paths follow `crate::ankyra::handlers::<subsystem>::<handler>`,
//! reply paths follow `crate::ankyra::replies::<PascalName>`, the `Stats`
//! output lives at `crate::ankyra::outputs::Stats`, and the `Pins`
//! enumeration at `crate::ankyra::enumerations::Pins`.
//!
//! Dev-feature-gated handlers (`tmc_init`, `tmc_test_spin`,
//! `tmc_write_register`, `tmc_read_register`) keep their `#[cfg]` gates on
//! the handler items themselves; the provider list mirrors those gates with
//! `#[cfg_attr(...)]`-like conditional entries so the list shape matches the
//! set of descriptor fns that actually exist in the current feature set.
//! Because `ankyra_provider!` is a function-like macro whose input is a
//! single token tree, conditional entries are expressed by duplicating the
//! macro invocation under `#[cfg]` / `#[cfg(not(...))]`. This keeps the two
//! branches lock-step.

#[cfg(feature = "dev")]
ankyra::ankyra_provider! {
    name: BOARD_PROVIDER,
    commands: [
        // Lifecycle (10)
        crate::ankyra::handlers::lifecycle::get_clock,
        crate::ankyra::handlers::lifecycle::get_uptime,
        crate::ankyra::handlers::lifecycle::get_config,
        crate::ankyra::handlers::lifecycle::clear_shutdown,
        crate::ankyra::handlers::lifecycle::config_reset,
        crate::ankyra::handlers::lifecycle::finalize_config,
        crate::ankyra::handlers::lifecycle::allocate_oids,
        crate::ankyra::handlers::lifecycle::emergency_stop,
        crate::ankyra::handlers::lifecycle::reset,
        crate::ankyra::handlers::lifecycle::debug_nop,
        // Digital out (3)
        crate::ankyra::handlers::digital_out::config_digital_out,
        crate::ankyra::handlers::digital_out::queue_digital_out,
        crate::ankyra::handlers::digital_out::update_digital_out,
        // Stepper (7)
        crate::ankyra::handlers::stepper::config_stepper,
        crate::ankyra::handlers::stepper::queue_step,
        crate::ankyra::handlers::stepper::set_next_step_dir,
        crate::ankyra::handlers::stepper::reset_step_clock,
        crate::ankyra::handlers::stepper::stepper_get_position,
        crate::ankyra::handlers::stepper::stepper_get_commanded_position,
        crate::ankyra::handlers::stepper::stepper_stop_on_trigger,
        // Endstop (3)
        crate::ankyra::handlers::endstop::config_endstop,
        crate::ankyra::handlers::endstop::endstop_home,
        crate::ankyra::handlers::endstop::endstop_query_state,
        // Trsync (4)
        crate::ankyra::handlers::trsync::config_trsync,
        crate::ankyra::handlers::trsync::trsync_set_timeout,
        crate::ankyra::handlers::trsync::trsync_start,
        crate::ankyra::handlers::trsync::trsync_trigger,
        // SPI (6)
        crate::ankyra::handlers::spi::config_spi,
        crate::ankyra::handlers::spi::config_spi_shutdown,
        crate::ankyra::handlers::spi::config_spi_without_cs,
        crate::ankyra::handlers::spi::spi_send,
        crate::ankyra::handlers::spi::spi_set_bus,
        crate::ankyra::handlers::spi::spi_transfer,
        // TMC (15) — dev-gated entries follow non-dev entries
        crate::ankyra::handlers::tmc::tmc_set_current,
        crate::ankyra::handlers::tmc::tmc_set_encoder,
        crate::ankyra::handlers::tmc::tmc_set_encoder_dir,
        crate::ankyra::handlers::tmc::tmc_set_flux_filter,
        crate::ankyra::handlers::tmc::tmc_set_pid_gains,
        crate::ankyra::handlers::tmc::tmc_set_position_filter,
        crate::ankyra::handlers::tmc::tmc_set_position_gains,
        crate::ankyra::handlers::tmc::tmc_set_torque_filter,
        crate::ankyra::handlers::tmc::tmc_set_velocity_feedforward,
        crate::ankyra::handlers::tmc::tmc_set_velocity_filter,
        crate::ankyra::handlers::tmc::tmc_set_velocity_limit,
        // Dev-gated TMC (4)
        crate::ankyra::handlers::tmc::tmc_init,
        crate::ankyra::handlers::tmc::tmc_test_spin,
        crate::ankyra::handlers::tmc::tmc_write_register,
        crate::ankyra::handlers::tmc::tmc_read_register,
        // FOCI (7)
        crate::ankyra::handlers::foci::foci_calibrate,
        crate::ankyra::handlers::foci::foci_commission,
        crate::ankyra::handlers::foci::foci_dump_registers,
        crate::ankyra::handlers::foci::foci_selftest,
        crate::ankyra::handlers::foci::foci_trace_fetch,
        crate::ankyra::handlers::foci::foci_trace_info,
        crate::ankyra::handlers::foci::foci_tune,
    ],
    replies: [
        // 18 reply structs, alphabetical by PascalCase leaf.
        // Lifetime-parameterized structs are listed by bare path; the
        // descriptor fn is emitted without lifetime dependence.
        crate::ankyra::replies::Clock,
        crate::ankyra::replies::Config,
        crate::ankyra::replies::EndstopState,
        crate::ankyra::replies::FociCalibrateResult,
        crate::ankyra::replies::FociCommissionPhase,
        crate::ankyra::replies::FociCommissionResult,
        crate::ankyra::replies::FociDumpDone,
        crate::ankyra::replies::FociDumpValue,
        crate::ankyra::replies::FociSelftestDone,
        crate::ankyra::replies::FociSelftestResult,
        crate::ankyra::replies::FociTraceData,
        crate::ankyra::replies::FociTraceInfoResult,
        crate::ankyra::replies::FociTuneResult,
        crate::ankyra::replies::SpiTransferResponse,
        crate::ankyra::replies::StepperPosition,
        crate::ankyra::replies::TmcRegisterValue,
        crate::ankyra::replies::TrsyncState,
        crate::ankyra::replies::Uptime,
    ],
    outputs: [
        crate::ankyra::outputs::Stats,
    ],
    enumerations: [
        crate::ankyra::enumerations::Pins,
    ],
}

#[cfg(not(feature = "dev"))]
ankyra::ankyra_provider! {
    name: BOARD_PROVIDER,
    commands: [
        // Lifecycle (10)
        crate::ankyra::handlers::lifecycle::get_clock,
        crate::ankyra::handlers::lifecycle::get_uptime,
        crate::ankyra::handlers::lifecycle::get_config,
        crate::ankyra::handlers::lifecycle::clear_shutdown,
        crate::ankyra::handlers::lifecycle::config_reset,
        crate::ankyra::handlers::lifecycle::finalize_config,
        crate::ankyra::handlers::lifecycle::allocate_oids,
        crate::ankyra::handlers::lifecycle::emergency_stop,
        crate::ankyra::handlers::lifecycle::reset,
        crate::ankyra::handlers::lifecycle::debug_nop,
        // Digital out (3)
        crate::ankyra::handlers::digital_out::config_digital_out,
        crate::ankyra::handlers::digital_out::queue_digital_out,
        crate::ankyra::handlers::digital_out::update_digital_out,
        // Stepper (7)
        crate::ankyra::handlers::stepper::config_stepper,
        crate::ankyra::handlers::stepper::queue_step,
        crate::ankyra::handlers::stepper::set_next_step_dir,
        crate::ankyra::handlers::stepper::reset_step_clock,
        crate::ankyra::handlers::stepper::stepper_get_position,
        crate::ankyra::handlers::stepper::stepper_get_commanded_position,
        crate::ankyra::handlers::stepper::stepper_stop_on_trigger,
        // Endstop (3)
        crate::ankyra::handlers::endstop::config_endstop,
        crate::ankyra::handlers::endstop::endstop_home,
        crate::ankyra::handlers::endstop::endstop_query_state,
        // Trsync (4)
        crate::ankyra::handlers::trsync::config_trsync,
        crate::ankyra::handlers::trsync::trsync_set_timeout,
        crate::ankyra::handlers::trsync::trsync_start,
        crate::ankyra::handlers::trsync::trsync_trigger,
        // SPI (6)
        crate::ankyra::handlers::spi::config_spi,
        crate::ankyra::handlers::spi::config_spi_shutdown,
        crate::ankyra::handlers::spi::config_spi_without_cs,
        crate::ankyra::handlers::spi::spi_send,
        crate::ankyra::handlers::spi::spi_set_bus,
        crate::ankyra::handlers::spi::spi_transfer,
        // TMC (11) — dev-gated entries omitted when feature is off
        crate::ankyra::handlers::tmc::tmc_set_current,
        crate::ankyra::handlers::tmc::tmc_set_encoder,
        crate::ankyra::handlers::tmc::tmc_set_encoder_dir,
        crate::ankyra::handlers::tmc::tmc_set_flux_filter,
        crate::ankyra::handlers::tmc::tmc_set_pid_gains,
        crate::ankyra::handlers::tmc::tmc_set_position_filter,
        crate::ankyra::handlers::tmc::tmc_set_position_gains,
        crate::ankyra::handlers::tmc::tmc_set_torque_filter,
        crate::ankyra::handlers::tmc::tmc_set_velocity_feedforward,
        crate::ankyra::handlers::tmc::tmc_set_velocity_filter,
        crate::ankyra::handlers::tmc::tmc_set_velocity_limit,
        // FOCI (7)
        crate::ankyra::handlers::foci::foci_calibrate,
        crate::ankyra::handlers::foci::foci_commission,
        crate::ankyra::handlers::foci::foci_dump_registers,
        crate::ankyra::handlers::foci::foci_selftest,
        crate::ankyra::handlers::foci::foci_trace_fetch,
        crate::ankyra::handlers::foci::foci_trace_info,
        crate::ankyra::handlers::foci::foci_tune,
    ],
    replies: [
        crate::ankyra::replies::Clock,
        crate::ankyra::replies::Config,
        crate::ankyra::replies::EndstopState,
        crate::ankyra::replies::FociCalibrateResult,
        crate::ankyra::replies::FociCommissionPhase,
        crate::ankyra::replies::FociCommissionResult,
        crate::ankyra::replies::FociDumpDone,
        crate::ankyra::replies::FociDumpValue,
        crate::ankyra::replies::FociSelftestDone,
        crate::ankyra::replies::FociSelftestResult,
        crate::ankyra::replies::FociTraceData,
        crate::ankyra::replies::FociTraceInfoResult,
        crate::ankyra::replies::FociTuneResult,
        crate::ankyra::replies::SpiTransferResponse,
        crate::ankyra::replies::StepperPosition,
        crate::ankyra::replies::TmcRegisterValue,
        crate::ankyra::replies::TrsyncState,
        crate::ankyra::replies::Uptime,
    ],
    outputs: [
        crate::ankyra::outputs::Stats,
    ],
    enumerations: [
        crate::ankyra::enumerations::Pins,
    ],
}
