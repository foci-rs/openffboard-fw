//! OpenFFBoard-specific TMC4671 hardware configuration constants.

use crate::ankyra::enumerations::Pins;

/// Number of board-local TMC4671 motor channels.
pub(crate) const TMC_CHANNEL_COUNT: u8 = 1;

/// OpenFFBoard v1.3 VM sense divider high-side resistor (R23).
pub(crate) const VM_DIVIDER_HIGH_OHMS: u32 = 71_500;

/// OpenFFBoard v1.3 VM sense divider low-side resistor (R24).
pub(crate) const VM_DIVIDER_LOW_OHMS: u32 = 1_500;

/// TMC4671 ADC_VM reference span in millivolts.
pub(crate) const VM_ADC_REFERENCE_MILLIVOLTS: u32 = 2_500;

/// Centered positive ADC_VM count span for the TMC4671 16-bit raw ADC.
pub(crate) const VM_ADC_CENTER_COUNTS: u32 = 32_767;

/// Return the motor channel represented by a virtual STEP pin.
pub(crate) fn step_pin_channel(pin: u8) -> Option<u8> {
    match Pins::try_from(pin) {
        Ok(Pins::STEP0) => Some(0),
        _ => None,
    }
}

/// Return the motor channel represented by a virtual DIR pin.
pub(crate) fn dir_pin_channel(pin: u8) -> Option<u8> {
    match Pins::try_from(pin) {
        Ok(Pins::DIR0) => Some(0),
        _ => None,
    }
}

/// Return the motor channel represented by a virtual ENA pin.
pub(crate) fn enable_pin_channel(pin: u32) -> Option<u8> {
    let pin = u8::try_from(pin).ok()?;
    match Pins::try_from(pin) {
        Ok(Pins::ENA0) => Some(0),
        _ => None,
    }
}

/// Board-specific TMC4671 hardware configuration for OpenFFBoard v1.3 ACS724.
///
/// PWM at 25 kHz (PWM_MAXCNT=3999, fCLK=25 MHz per datasheet 4.9.3),
/// 90ns BBM dead-time for resistance-method diagnostics,
/// symmetric DSADC MDEC derived from the PWM period.
/// The 1024-count commissioning excitation is kept unchanged for this diagnostic test
/// while staying within the ADC setup phase's conservative voltage clamp.
///
/// Current-loop P uses a physical-bandwidth formula and the 1600 Hz OpenFFBoard baseline;
/// live validation found bandwidth alone is not the holding-noise control lever.
const OPENFFBOARD_V13_PWM_MAXCNT: u16 = 3999;
const OPENFFBOARD_V13_DSADC_MDEC: u16 =
    foci_firmware::config::dsadc_mdec_from_pwm_maxcnt(OPENFFBOARD_V13_PWM_MAXCNT);

pub(crate) const TMC_CONFIG: foci_firmware::config::TmcConfig = foci_firmware::config::TmcConfig {
    pwm_maxcnt: OPENFFBOARD_V13_PWM_MAXCNT,
    tmc_clock_hz: 25_000_000,
    bbm_h: 9,
    bbm_l: 9,
    dsadc_mdec_a: OPENFFBOARD_V13_DSADC_MDEC,
    dsadc_mdec_b: OPENFFBOARD_V13_DSADC_MDEC,
    // OpenFFBoard v1.3 ACS724 routing: UX=I0, WY=I1, V=I2.
    adc_i_select: 0x1800_0100,
    excitation_voltage: 1024,
    bandwidth_hz: 1600.0,
    electrical_model_theta_e_us: 160,
    current_validation_flux_max_encoder_delta_counts: 4,
    current_validation_torque_max_encoder_delta_counts: 16,
    resistance_max_expected_current_count: 1600,
    resistance_scale_metadata_validated: true,
    // Milliamps per ADC least significant bit, derived from the sense
    // resistor value and ADC configuration.
    current_scale_ma_per_lsb: 1.155,
};

const OPENFFBOARD_V13_BBM_REGISTER: u32 = 0x0909;
const _: () = assert!(TMC_CONFIG.bbm_register() == OPENFFBOARD_V13_BBM_REGISTER);
const OPENFFBOARD_V13_DSADC_MDEC_REGISTER: u32 =
    ((OPENFFBOARD_V13_DSADC_MDEC as u32) << 16) | OPENFFBOARD_V13_DSADC_MDEC as u32;
const _: () = assert!(TMC_CONFIG.dsadc_mdec_register() == OPENFFBOARD_V13_DSADC_MDEC_REGISTER);
const OPENFFBOARD_V13_EXCITATION_VOLTAGE: u16 = 1024;
const _: () = assert!(TMC_CONFIG.excitation_voltage == OPENFFBOARD_V13_EXCITATION_VOLTAGE);

/// Per-channel feedback wiring. Every board channel is ABN-only today;
/// analog-encoder enablement is a follow-up.
pub(crate) const FEEDBACK_CONFIGS: [foci_firmware::config::FeedbackConfig;
    TMC_CHANNEL_COUNT as usize] = [foci_firmware::config::FeedbackConfig {
    phi_e_source: foci_firmware::config::FeedbackSource::Abn,
    position_source: foci_firmware::config::FeedbackSource::Abn,
}; TMC_CHANNEL_COUNT as usize];
