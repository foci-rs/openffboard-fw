//! OpenFFBoard TMC control board accessors.

use core::sync::atomic::Ordering::Relaxed;

use embassy_stm32::gpio::Output;
use foci_firmware::commission_dispatch::CommissionRetention;
use foci_firmware::commissioning::outer::velocity::integral_reproduction::IntegralRetentionSlot;
use foci_firmware::shutdown::ShutdownReason;
use foci_firmware::state::trsync::HOMING_STATE;
use foci_firmware::tmc_control::TmcControlRuntime;
use foci_firmware::tmc_control::helpers::refresh_velocity_scale_from_state;
use foci_firmware::tmc_control::{
    DriveCtx, ParkReason, TmcControlConfig, TmcSafetyBootstrapFailure,
};
use foci_tmc4671::spi::Tmc4671Spi;

use crate::ankyra::constants::CLOCK_FREQ;
use crate::app_types::TmcSpiDevice;
use crate::board_config::TMC_CONFIG;
use crate::commissioning_backend::OpenFFBoardBackend;

/// `STATUS_FLAGS` bit routed to the TMC4671 STATUS pin (PE8 EXTI). Set by
/// the safety bootstrap in `STATUS_MASK`; the emergency-disable audit reads
/// it back to correlate the pin level with the flag.
pub(crate) const STATUS_CRITICAL_MASK: u32 = 1 << 26;

/// Assemble one drive-context borrow for the control loop's current tick.
pub(crate) fn drive_ctx<'a>(
    now_us: u64,
    homing_snapshot: u8,
    status_pin_asserted: bool,
    config: &'a TmcControlConfig,
    stage_c: &'a mut IntegralRetentionSlot,
) -> DriveCtx<'a> {
    DriveCtx {
        now_us,
        homing_active: HOMING_STATE.load(Relaxed) != 0,
        homing_snapshot,
        status_pin_asserted,
        status_mask: STATUS_CRITICAL_MASK,
        config,
        retention: CommissionRetention::new(stage_c),
    }
}

/// How a parked channel is reported to the host: the old bootstrap's
/// message for the first three failures, the stage-specific shutdown reason
/// for the safety bootstrap.
pub(crate) enum ParkShutdown {
    Message(&'static str),
    Reason(ShutdownReason),
}

pub(crate) fn park_shutdown(reason: ParkReason) -> ParkShutdown {
    match reason {
        ParkReason::SpiUnavailable => ParkShutdown::Message("TMC4671 SPI failed at boot"),
        ParkReason::ChipIdMismatch(_) => ParkShutdown::Message("TMC4671 chip ID mismatch"),
        ParkReason::PiRepresentation => {
            ParkShutdown::Message("TMC4671 PI representation failed at boot")
        }
        ParkReason::SafetyBootstrap(stage) => {
            ParkShutdown::Reason(TmcSafetyBootstrapFailure { stage }.shutdown_reason(0))
        }
    }
}

/// Enable the TMC4671 STATUS interrupt on PE8 once the safety bootstrap has
/// written `STATUS_MASK`. Moved verbatim from the old startup sequence.
pub(crate) fn enable_status_exti() {
    let syscfg = embassy_stm32::pac::SYSCFG;
    // EXTICR3[3:0] = 4 selects port E for EXTI line 8.
    syscfg.exticr(2).modify(|w| w.set_exti(0, 4));
    let exti = embassy_stm32::pac::EXTI;
    exti.rtsr(0).modify(|w| w.set_line(8, true));
    exti.ftsr(0).modify(|w| w.set_line(8, false));
    exti.pr(0).write(|w| w.set_line(8, true));
    exti.imr(0).modify(|w| w.set_line(8, true));
    defmt::info!(
        "TMC4671 STATUS_MASK: {:#010x}, EXTI8 enabled",
        STATUS_CRITICAL_MASK
    );
}

pub(crate) fn tmc_state() -> &'static foci_firmware::state::tmc::TmcChannelState {
    foci_firmware::state::tmc::tmc_channel0()
}

pub(crate) fn stepper_state() -> &'static foci_firmware::state::stepper::StepperChannelState {
    foci_firmware::state::stepper::stepper_channel0()
}

pub(crate) fn openffboard_backend<'a>(
    tmc: &'a mut Tmc4671Spi<TmcSpiDevice>,
    enable_pin: &'a mut Output<'static>,
) -> OpenFFBoardBackend<'a> {
    OpenFFBoardBackend::new(0, tmc, enable_pin, tmc_state())
}

pub(crate) fn refresh_velocity_scale(runtime: &mut TmcControlRuntime) {
    refresh_velocity_scale_from_state(runtime, tmc_state(), CLOCK_FREQ, TMC_CONFIG.pwm_freq_hz());
}
