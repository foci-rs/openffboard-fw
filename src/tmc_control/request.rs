//! OpenFFBoard TMC control request handling.
//!
//! This module owns the OpenFFBoard adapter for the shared TMC request runner.
//! The RTIC task in `main.rs` keeps request waiting, shutdown locking, and
//! control-loop timing.

use core::sync::atomic::Ordering;

use embassy_stm32::gpio::Output;
use foci_firmware::autotune::CommissionError;
use foci_firmware::commission_dispatch::{ReplyPayload, StepperPostAction};
use foci_firmware::state::stepper::StepperChannelState;
use foci_firmware::state::tmc::TmcChannelState;
use foci_firmware::state::trsync::HOMING_STATE;
use foci_firmware::tmc::TmcRequest;
use foci_firmware::tmc_control::request::{
    MotionScaleMapperInstallError, TmcPreRequestDisposition, TmcRequestBoard,
    handle_tmc_control_request as handle_shared_request,
};
use foci_firmware::tmc_control::{
    BoardActions, ShutdownReason, TmcControlRequestOutcome, TmcControlRequestTail,
    TmcControlRuntime,
};
use foci_tmc4671::spi::Tmc4671Spi;

use crate::app_types::TmcSpiDevice;
#[cfg(feature = "dev")]
use crate::helpers::dev::{do_test_spin, tmc_enter_openloop};
use crate::helpers::stepper::apply_stepper_post_action;
use crate::state::stepper::{
    install_motion_scale_mapper, isr_stepper, reset_mapper_observables, step_queue,
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

    fn motion_scale_queue_empty(&self) -> bool {
        // SAFETY: The TMC request runs at P1 and only observes queue state.
        unsafe { step_queue(0).is_some_and(|queue| queue.is_empty()) }
    }

    fn install_motion_scale_mapper(
        &mut self,
        mapper: foci_klipper::pulse_mapper::PulseMapperConfig,
    ) -> Result<(), MotionScaleMapperInstallError> {
        with_trigger_domain(|td| {
            let idle = !tmc_state().motor_armed.load(Ordering::Relaxed)
                && unsafe { step_queue(0).is_some_and(|queue| queue.is_empty()) }
                && !td.timer_active(0);
            if !idle {
                return Err(MotionScaleMapperInstallError);
            }

            // SAFETY: The trigger-domain lock excludes P2/P3, and the checked
            // disarmed/empty/inactive state keeps the ISR quiesced afterward.
            let stepper = unsafe { isr_stepper(0).ok_or(MotionScaleMapperInstallError)? };
            install_motion_scale_mapper(stepper, mapper)
                .map_err(|_| MotionScaleMapperInstallError)?;
            reset_mapper_observables();
            Ok(())
        })
    }

    fn emit_reply(&mut self, payload: ReplyPayload) {
        crate::ankyra::replies::emit_terminal_reply(payload);
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
    ) -> Result<(), ()> {
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
) -> TmcControlRequestOutcome {
    if req.channel != 0 {
        return TmcControlRequestOutcome::shutdown(
            ShutdownReason::InvalidTmcChannel,
            TmcControlRequestTail::SkipTickTail,
        );
    }

    let mut board = OpenFFBoardRequestBoard::new(enable_pin, commission_trace_engine_seq);
    handle_shared_request(
        req,
        TmcPreRequestDisposition::ContinueRequest,
        runtime,
        tmc,
        &mut board,
    )
    .await
}
