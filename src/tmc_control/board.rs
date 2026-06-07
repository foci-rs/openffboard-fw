//! OpenFFBoard TMC control board accessors.

use embassy_stm32::gpio::Output;
use foci_firmware::tmc_control::TmcControlRuntime;
use foci_firmware::tmc_control::helpers::refresh_velocity_scale_from_state;
use foci_tmc4671::spi::Tmc4671Spi;

use crate::ankyra::constants::CLOCK_FREQ;
use crate::app_types::TmcSpiDevice;
use crate::board_config::TMC_CONFIG;
use crate::commissioning_backend::OpenFFBoardBackend;

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
    OpenFFBoardBackend {
        channel: 0,
        tmc,
        enable_pin,
        tmc_state: tmc_state(),
    }
}

pub(crate) fn refresh_velocity_scale(runtime: &mut TmcControlRuntime) {
    refresh_velocity_scale_from_state(runtime, tmc_state(), CLOCK_FREQ, TMC_CONFIG.pwm_freq_hz());
}
