//! OpenFFBoard TMC control request handling.

use core::sync::atomic::Ordering;

use embassy_stm32::gpio::Output;
use foci_firmware::autotune::CommissionError;
use foci_firmware::commission_dispatch::{ReplyPayload, StepperPostAction};
use foci_firmware::commissioning::outer::velocity::integral_reproduction::IntegralRetentionSlots;
use foci_firmware::state::stepper::StepperChannelState;
use foci_firmware::state::tmc::TmcChannelState;
use foci_firmware::state::trsync::HOMING_STATE;
use foci_firmware::tmc::TmcRequest;
use foci_firmware::tmc_control::dispatch::RegisterRequestError;
use foci_firmware::tmc_control::request::handle_tmc_control_request_with_stages;
use foci_firmware::tmc_control::request::{
    MotionScaleMapperInstallError, TmcPreRequestDisposition, TmcRequestBoard,
};
use foci_firmware::tmc_control::{
    BoardActions, ShutdownReason, TmcControlRequestOutcome, TmcControlRequestTail,
    TmcControlRuntime,
};
use foci_tmc4671::spi::Tmc4671Spi;

use crate::app_types::TmcSpiDevice;
use crate::board_config::TMC_CHANNEL_COUNT;
#[cfg(feature = "dev")]
use crate::helpers::dev::{do_test_spin, tmc_enter_openloop};
use crate::helpers::stepper::apply_stepper_post_action;
use crate::state::stepper::{
    isr_stepper, motion_scale_install_admitted, replace_motion_scale_mapper,
};
use crate::tmc_control::board::{openffboard_backend, stepper_state, tmc_state};
use crate::tmc_dispatch::handle_tmc_request;
use crate::trigger_domain::with_trigger_domain;

fn next_commission_trace_seq(commission_trace_engine_seq: &mut u32) -> u32 {
    let seq = *commission_trace_engine_seq;
    *commission_trace_engine_seq = (*commission_trace_engine_seq).wrapping_add(1);
    seq
}

struct OpenFFBoardRequestBoard<'a> {
    enable_pin: &'a mut Output<'static>,
    commission_trace_engine_seq: &'a mut u32,
}

struct OpenFFBoardBusyReplySink;

impl foci_firmware::tmc_control::request::CommissionBusyReplySink for OpenFFBoardBusyReplySink {
    fn normal_tail(&self) -> TmcControlRequestTail {
        TmcControlRequestTail::SkipTickTail
    }

    fn emit_reply(&mut self, channel: u8, payload: ReplyPayload) {
        crate::ankyra::replies::emit_terminal_reply(channel, payload);
    }

    fn emit_busy_diagnostic(&mut self, request: TmcRequest) {
        crate::tmc_diagnostics::emit_commission_busy_diagnostic(request);
    }
}

impl<'a> OpenFFBoardRequestBoard<'a> {
    fn new(enable_pin: &'a mut Output<'static>, commission_trace_engine_seq: &'a mut u32) -> Self {
        Self {
            enable_pin,
            commission_trace_engine_seq,
        }
    }
}

impl TmcRequestBoard<TmcSpiDevice> for OpenFFBoardRequestBoard<'_> {
    fn channel(&self) -> u8 {
        0
    }

    fn tmc_state(&self) -> &'static TmcChannelState {
        tmc_state()
    }

    fn stepper_state(&self) -> &'static StepperChannelState {
        stepper_state()
    }

    fn commissioning_blocked(&self, _runtime: &TmcControlRuntime) -> bool {
        false
    }

    fn next_commission_trace_seq(&mut self) -> u32 {
        next_commission_trace_seq(self.commission_trace_engine_seq)
    }

    fn normal_tail(&self) -> TmcControlRequestTail {
        TmcControlRequestTail::SkipTickTail
    }

    fn default_current_filter_hz(&self) -> u16 {
        crate::board_config::TMC_CONFIG.default_current_filter_hz()
    }

    fn pwm_freq_hz(&self) -> u32 {
        crate::board_config::TMC_CONFIG.pwm_freq_hz()
    }

    fn stimulus_step_timer_hz(&self) -> u32 {
        crate::state::stepper::STEP_TIMER_HZ
    }

    fn velocity_limit_evidence_capability(&self) -> u32 {
        crate::ankyra::constants::VELOCITY_LIMIT_EVIDENCE_CAPABILITY
    }

    fn motion_scale_queue_empty(&self) -> bool {
        crate::state::stepper::STEP_QUEUE_CHANNEL.is_empty()
    }

    fn install_motion_scale_mapper(
        &mut self,
        mapper: foci_klipper::pulse_mapper::PulseMapperConfig,
    ) -> Result<(), MotionScaleMapperInstallError> {
        with_trigger_domain(|td| {
            let motor_armed = tmc_state().motor_armed.load(Ordering::Relaxed);
            let queue_empty = crate::state::stepper::STEP_QUEUE_CHANNEL.is_empty();
            let timer_active = crate::trigger_domain::timer_active(td.state(), 0);
            let admitted = motion_scale_install_admitted(motor_armed, queue_empty, timer_active);

            // SAFETY: The trigger-domain lock excludes P2/P3 while replacement
            // clears the prior mapper before applying the captured admission.
            let stepper = unsafe { isr_stepper(0).ok_or(MotionScaleMapperInstallError)? };
            replace_motion_scale_mapper(stepper, mapper, admitted)
                .map_err(|_| MotionScaleMapperInstallError)
        })
    }

    fn report_motion_scale_rejection(
        &mut self,
        oid: u8,
        rejection: foci_firmware::motion_scale::MotionScaleRejection,
    ) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociMotionScaleRejected,
            oid: u8 = oid,
            stage: u8 = foci_firmware::motion_scale::MOTION_SCALE_REJECTION_STAGE_P1,
            reason: u8 = rejection.code(),
        );
    }

    fn emit_reply(&mut self, channel: u8, payload: ReplyPayload) {
        crate::ankyra::replies::emit_terminal_reply(channel, payload);
    }

    #[cfg(feature = "trace")]
    fn record_velocity_sweep_trace_record(
        &mut self,
        record: foci_firmware::ankyra::velocity_sweep_encoding::VelocitySweepCanonicalRecord,
    ) {
        crate::usb_trace::request_velocity_sweep_evidence(0, record);
    }

    fn emit_motor_arm_event(&mut self) {
        #[cfg(feature = "trace")]
        crate::usb_trace::request_event(
            foci_firmware::trace::events::EVENT_LEVEL_INFO,
            foci_firmware::trace::events::SUBSYSTEM_TMC_CONTROL,
            crate::usb_trace::EVENT_MOTOR_ARM,
            [0; 4],
            0,
        );
    }

    fn emit_motor_disarm_event(&mut self) {
        #[cfg(feature = "trace")]
        crate::usb_trace::request_event(
            foci_firmware::trace::events::EVENT_LEVEL_INFO,
            foci_firmware::trace::events::SUBSYSTEM_TMC_CONTROL,
            crate::usb_trace::EVENT_MOTOR_DISARM,
            [0; 4],
            0,
        );
    }

    fn apply_stepper_actions(&mut self, actions: BoardActions) {
        crate::tmc_control::effects::apply_actions(actions, |_| {});
    }

    async fn arm_motor(
        &mut self,
        runtime: &mut TmcControlRuntime,
        tmc: &mut Tmc4671Spi<TmcSpiDevice>,
        post_action: StepperPostAction,
    ) -> Result<(), CommissionError> {
        let mut backend = openffboard_backend(tmc, self.enable_pin);
        let mut state = runtime.dispatch_state(tmc_state(), stepper_state());
        foci_firmware::tmc_control::lifecycle::arm_motor_electrical(&mut backend, &mut state)
            .await?;
        apply_stepper_post_action(post_action, &mut state);
        Ok(())
    }

    async fn disarm_motor(
        &mut self,
        runtime: &mut TmcControlRuntime,
        tmc: &mut Tmc4671Spi<TmcSpiDevice>,
    ) {
        let mut backend = openffboard_backend(tmc, self.enable_pin);
        let mut state = runtime.dispatch_state(tmc_state(), stepper_state());
        foci_firmware::tmc_control::lifecycle::disarm_motor(&mut backend, &mut state).await;
    }

    async fn dispatch_register_request(
        &mut self,
        req: TmcRequest,
        tmc: &mut Tmc4671Spi<TmcSpiDevice>,
    ) -> Result<(), RegisterRequestError> {
        handle_tmc_request(req, tmc).await
    }

    async fn run_diagnostic_request(
        &mut self,
        req: TmcRequest,
        runtime: &mut TmcControlRuntime,
        tmc: &mut Tmc4671Spi<TmcSpiDevice>,
    ) {
        let homing_active = HOMING_STATE.load(Ordering::Relaxed) != 0;
        let stepper_active = stepper_state().timer_active_mirror.load(Ordering::Relaxed);
        let saved_voltage_limit = tmc_state().voltage_limit_cfg.load(Ordering::Relaxed);
        match crate::tmc_diagnostics::handle_diagnostic_request(
            req,
            crate::tmc_diagnostics::DiagnosticContext {
                tmc,
                runtime,
                enable_pin: self.enable_pin,
                homing_active,
                stepper_active,
                saved_voltage_limit,
                tmc_config: &crate::board_config::TMC_CONFIG,
            },
        )
        .await
        {
            crate::tmc_diagnostics::DiagnosticDispatch::Handled => {}
            crate::tmc_diagnostics::DiagnosticDispatch::NotDiagnostic(_req) => {
                defmt::error!("diagnostic dispatch returned non-diagnostic request");
            }
        }
    }

    #[cfg(feature = "dev")]
    async fn run_dev_init(&mut self, tmc: &mut Tmc4671Spi<TmcSpiDevice>) {
        tmc_enter_openloop(tmc, self.enable_pin).await;
    }

    #[cfg(feature = "dev")]
    async fn run_dev_test_spin(&mut self, tmc: &mut Tmc4671Spi<TmcSpiDevice>, revolutions: u8) {
        do_test_spin(tmc, revolutions).await;
    }
}

pub(crate) async fn handle_tmc_control_request(
    req: TmcRequest,
    tmc: &mut Tmc4671Spi<TmcSpiDevice>,
    enable_pin: &mut Output<'static>,
    runtime: &mut foci_firmware::tmc_control::TmcControlRuntime,
    commission_trace_engine_seq: &mut u32,
    integral_candidates: &mut IntegralRetentionSlots<{ TMC_CHANNEL_COUNT as usize }>,
) -> TmcControlRequestOutcome {
    if req.channel != 0 {
        return TmcControlRequestOutcome::shutdown(
            ShutdownReason::InvalidTmcChannel,
            TmcControlRequestTail::SkipTickTail,
        );
    }

    let integral_candidate = integral_candidates
        .slot_mut(req.channel)
        .expect("validated TMC channel");
    let mut board = OpenFFBoardRequestBoard::new(enable_pin, commission_trace_engine_seq);
    handle_tmc_control_request_with_stages(
        req,
        TmcPreRequestDisposition::ContinueRequest,
        runtime,
        integral_candidate,
        tmc,
        &mut board,
    )
    .await
}

/// Emit the shared busy-only response without granting hardware capabilities.
pub(crate) fn handle_commission_wait_reject(req: TmcRequest) -> TmcControlRequestOutcome {
    foci_firmware::tmc_control::request::handle_commission_wait_reject(
        req,
        &mut OpenFFBoardBusyReplySink,
    )
}
