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
    high_rate_delay_period_us: 200,
    high_rate_delay_cycles_per_us: 168,
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

/// STM32F407 ADC instance carrying the internal temperature sensor channel.
///
/// RM0090 is not cached in this repository, so this is verified instead
/// against the real Klipper STM32 ADC driver at `src/stm32/adc.c` in the
/// `klipper` checkout (`~/Documents/Development/klipper` at the time of
/// writing): `gpio_adc_setup` (around line 84) defaults to
/// `ADC_TypeDef *adc = ADC1`, and only reassigns `adc = ADC3` when the
/// resolved channel index is `>= 19` (line ~89), which is reserved for the
/// extra PF-pin channels present on STM32F4x5/F446 parts. The internal
/// temperature-sensor pseudo-pin (`ADC_TEMPERATURE_PIN`, line 18) sits at
/// channel index 16 in the `adc_pins` table (line ~21-26) -- below that
/// threshold -- so it stays on the default `ADC1`; enabling it is done via
/// `ADC123_COMMON->CCR = ADC_CCR_TSVREFE` (line ~113), the ADC1/2/3 shared
/// common-control register, not a per-instance one. Independently
/// corroborated via web search: ST community references confirm RM0090
/// wires the STM32F40x/F41x temperature sensor only to `ADC1_IN16`, not
/// ADC2 or ADC3.
pub(crate) type InternalTempAdcInstance = embassy_stm32::peripherals::ADC1;

/// Return the `shared/foci-firmware` `ANALOG_CHANNELS` slot index for a
/// virtual analog-in pin. OpenFFBoard has only `MCU_TEMP`, at index 0.
#[allow(dead_code)] // consumed by Task 6's analog-in dispatch wiring
pub(crate) fn analog_pin_channel(pin: u8) -> Option<u8> {
    match Pins::try_from(pin) {
        Ok(Pins::MCU_TEMP) => Some(0),
        _ => None,
    }
}

/// STM32F407 internal temperature sensor factory calibration, read from
/// ST's documented system-memory addresses (RM0090 device electronic
/// signature section). `TS_CAL1` is captured at 30C, `TS_CAL2` at 110C,
/// both at 3.3V VDDA per the datasheet -- confirm this against the
/// STM32F407 datasheet's "temperature sensor characteristics" table before
/// relying on it.
const TS_CAL1_ADDR: *const u16 = 0x1FFF_7A2C as *const u16;
const TS_CAL2_ADDR: *const u16 = 0x1FFF_7A2E as *const u16;
const TS_CAL1_TEMP_C: f32 = 30.0;
const TS_CAL2_TEMP_C: f32 = 110.0;

/// Read the STM32F407's factory-programmed temperature sensor calibration.
///
/// # Safety
/// These addresses are ST's documented read-only system memory calibration
/// area, valid on every STM32F407 device -- reading them is always safe;
/// this function exists only to centralize the `unsafe` block, not because
/// the read itself is conditionally unsafe.
pub(crate) fn read_mcu_temp_calibration() -> foci_firmware::mcu_temp::FactoryCalibration {
    // SAFETY: TS_CAL1_ADDR/TS_CAL2_ADDR are ST's documented factory
    // calibration addresses in system memory, present and readable on
    // every STM32F407 device.
    let cal1 = unsafe { core::ptr::read_volatile(TS_CAL1_ADDR) };
    let cal2 = unsafe { core::ptr::read_volatile(TS_CAL2_ADDR) };
    foci_firmware::mcu_temp::FactoryCalibration {
        cal_low: (TS_CAL1_TEMP_C, cal1),
        cal_high: (TS_CAL2_TEMP_C, cal2),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn analog_pin_channel_resolves_mcu_temp_only() {
        assert_eq!(analog_pin_channel(Pins::MCU_TEMP as u8), Some(0));
        assert_eq!(analog_pin_channel(Pins::STEP0 as u8), None);
    }
}
