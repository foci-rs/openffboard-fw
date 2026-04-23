//! `#[klipper_constant]` declarations registered into the binary's
//! data dictionary `config` section. Consumed by the `constants: [...]`
//! list in `src/ankyra/provider.rs`.
//!
//! All constant identifiers are SCREAMING_SNAKE_CASE; ankyra preserves
//! them verbatim as wire names (per `pascal_to_snake` passthrough rule).
//!
//! Several constants below (MCU, STEPPER_STEP_BOTH_EDGE, RECEIVE_WINDOW,
//! STATS_SUMSQ_BASE, RESERVE_PINS_*) are only consumed over the wire and
//! are never referenced from Rust — the `#[allow(dead_code)]` suppresses
//! the unused-const lint without masking genuinely dead code elsewhere.

use ankyra::klipper_constant;

#[klipper_constant]
pub const CLOCK_FREQ: u32 = 84_000_000;

#[allow(dead_code)]
#[klipper_constant]
pub const MCU: &str = "stm32f407xx";

// STEPPER_STEP_BOTH_EDGE=1 matches the value Klipper C declares
// unconditionally in `src/stepper.c`, which suppresses Klipper's
// "MCU has deprecated code" startup warning. Reporting anything
// else (0 or absent) fires the warning permanently with no user-
// facing way to silence it.
//
// Safety does NOT depend on this value:
//   - The only driver module that binds to this firmware is
//     `klipper-foci`, which never calls
//     `setup_default_pulse_duration(..., step_both_edge=True)`, so
//     `_req_step_both_edge` stays False and Klipper never activates
//     toggle-mode stepping for FOCI steppers.
//   - The firmware's step ISR only implements `stepper_event_full`
//     semantics: it always doubles `count` internally and always
//     emits a STEP rising edge followed by a falling edge. Klipper
//     queue_step counts are motor-step counts in both Klipper modes,
//     so the TMC4671 sees the right number of rising edges even if
//     a future driver module were to request toggle mode.
//   - The TMC4671 itself only counts rising edges on STEP
//     (datasheet §4.2.4), so any accidental toggle-mode generation
//     by the firmware would be caught as a test failure immediately.
//
// The constant is therefore a protocol-parity advertisement, not a
// capability gate.
#[allow(dead_code)]
#[klipper_constant]
pub const STEPPER_STEP_BOTH_EDGE: u32 = 1;

#[allow(dead_code)]
#[klipper_constant]
pub const RECEIVE_WINDOW: u32 = 192;

#[allow(dead_code)]
#[klipper_constant]
pub const STATS_SUMSQ_BASE: u32 = 256;

// ---- Reserved pin constants ----
// Tells the Klipper host which pins are firmware-owned and must not be
// assigned to user-facing config sections (steppers, heaters, fans, etc.).

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

/// STEP (PB11) and DIR (PA0) GPIO pins for TMC4671 hardware step/dir.
#[allow(dead_code)]
#[klipper_constant]
pub const RESERVE_PINS_STEP: &str = "PA0,PB11";
