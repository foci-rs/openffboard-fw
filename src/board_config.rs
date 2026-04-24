//! OpenFFBoard-specific TMC4671 hardware configuration constants.

/// Current scaling for OpenFFBoard: milliamps per ADC least significant bit.
/// Derived from sense resistor value and ADC configuration.
pub(crate) const CURRENT_SCALE_MA_PER_LSB: f32 = 1.155;

/// Board-specific TMC4671 hardware configuration for OpenFFBoard v1.2.x.
///
/// PWM at 25 kHz (PWM_MAXCNT=3999, fCLK=25 MHz per datasheet 4.9.3),
/// 100ns BBM dead-time, DSADC MDEC=331 (Table 14).
///
/// Bandwidth 150 Hz determined by sweep at 8 MHz SPI (2026-04-05):
///   50 Hz  — weak holding torque (P too low with accurate L)
///  100 Hz  — good hold, inaudible, smooth moves
///  150 Hz  — strong hold, inaudible, best position stiffness
///  200 Hz  — slightly audible vibration, audible stepping during moves
/// The optimal bandwidth increased from 50 Hz to 150 Hz after fixing
/// the PWM frequency formula (datasheet: 25 kHz, not 6.25 kHz) and
/// improving L measurement accuracy via 8 MHz SPI.
pub(crate) const TMC_CONFIG: foci_firmware::config::TmcConfig = foci_firmware::config::TmcConfig {
    pwm_maxcnt: 3999,
    tmc_clock_hz: 25_000_000,
    bbm_h: 10,
    bbm_l: 10,
    dsadc_mdec: 331,
    excitation_voltage: 512,
    didt_cycles: 5000,
    bandwidth_hz: 150.0,
};
