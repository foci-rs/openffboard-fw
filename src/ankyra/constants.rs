use ankyra::klipper_constant;

use crate::board_config;

#[klipper_constant]
pub const CLOCK_FREQ: u32 = 84_000_000;

#[klipper_constant]
pub const ENVELOPE_PROPORTIONAL_NUM: u32 = 3;
const _: () = assert!(
    ENVELOPE_PROPORTIONAL_NUM
        == foci_firmware::commissioning::outer::velocity::sweep_plan::ENVELOPE_PROPORTIONAL_NUM
);

#[klipper_constant]
pub const ENVELOPE_PROPORTIONAL_DEN: u32 = 2;
const _: () = assert!(
    ENVELOPE_PROPORTIONAL_DEN
        == foci_firmware::commissioning::outer::velocity::sweep_plan::ENVELOPE_PROPORTIONAL_DEN
);

#[klipper_constant]
pub const ENVELOPE_ABSOLUTE_MARGIN_MREV_S: u32 = 2_000;
const _: () = assert!(
    ENVELOPE_ABSOLUTE_MARGIN_MREV_S
        == foci_firmware::commissioning::outer::velocity::sweep_plan::ENVELOPE_ABSOLUTE_MARGIN_MREV_S
);

#[klipper_constant]
pub const VELOCITY_LIMIT_EVIDENCE_CAPABILITY: u32 = 1;
const _: () = assert!(
    VELOCITY_LIMIT_EVIDENCE_CAPABILITY
        == foci_firmware::commissioning::outer::velocity::VELOCITY_LIMIT_EVIDENCE_CAPABILITY_LATCHED_FLAG
);

#[allow(dead_code)]
#[klipper_constant]
pub const MCU: &str = "stm32f407xx";

/// Klipper reads this via `get_constant_float`, which parses the wire string, so `u32` is
/// enough; `#[klipper_constant]` has no float type.
#[klipper_constant]
pub const ADC_MAX: u32 = 4095;

#[allow(dead_code)]
#[klipper_constant]
pub const RESERVE_PINS_USB: &str = "PA9,PA11,PA12";

#[allow(dead_code)]
#[klipper_constant]
pub const RESERVE_PINS_SPI: &str = "PA4,PA5,PA6,PA7";

#[allow(dead_code)]
#[klipper_constant]
pub const RESERVE_PINS_TMC: &str = "PE7,PE8,PE10,PE12";

#[allow(dead_code)]
#[klipper_constant]
pub const RESERVE_PINS_LED: &str = "PD7,PE0,PE1";

#[allow(dead_code)]
#[klipper_constant]
pub const RESERVE_PINS_SWD: &str = "PA13,PA14";

#[allow(dead_code)]
#[klipper_constant]
pub const RESERVE_PINS_DRV: &str = "PB0,PB1,PC4,PC5";

#[allow(dead_code)]
#[klipper_constant]
pub const RESERVE_PINS_STEP: &str = "PA0,PB11";

#[klipper_constant]
pub const FOCI_VM_DIVIDER_HIGH_OHMS: u32 = 71_500;
const _: () = assert!(FOCI_VM_DIVIDER_HIGH_OHMS == board_config::VM_DIVIDER_HIGH_OHMS);

#[klipper_constant]
pub const FOCI_VM_DIVIDER_LOW_OHMS: u32 = 1_500;
const _: () = assert!(FOCI_VM_DIVIDER_LOW_OHMS == board_config::VM_DIVIDER_LOW_OHMS);

#[klipper_constant]
pub const FOCI_VM_ADC_REFERENCE_MILLIVOLTS: u32 = 2_500;
const _: () =
    assert!(FOCI_VM_ADC_REFERENCE_MILLIVOLTS == board_config::VM_ADC_REFERENCE_MILLIVOLTS);

#[klipper_constant]
pub const FOCI_VM_ADC_CENTER_COUNTS: u32 = 32_767;
const _: () = assert!(FOCI_VM_ADC_CENTER_COUNTS == board_config::VM_ADC_CENTER_COUNTS);

#[klipper_constant]
pub const FOCI_MAX_RUN_CURRENT_MA: u32 = 20_000;
const _: () = assert!(FOCI_MAX_RUN_CURRENT_MA == board_config::TMC_CONFIG.max_run_current_ma);
