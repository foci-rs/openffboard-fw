//! Board-level type aliases and the Klipper pin enumeration.
//!
//! Items that are naturally `mod app`-scoped but do not need to live
//! next to the RTIC task graph are collected here so `main.rs` stays
//! focused on the RTIC wiring.

use anchor::klipper_enumeration;
use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Async as AsyncMode;
use embassy_stm32::spi::Spi;
use embedded_hal_bus::spi::ExclusiveDevice;

/// SPI bus type for TMC4671 communication.
pub type Spi1Bus = Spi<'static, AsyncMode, embassy_stm32::spi::mode::Master>;

/// Sub-microsecond busy-wait delay for SPI read pauses.
///
/// The TMC4671-LA requires a 500 ns pause after the address byte for
/// SPI reads above 2 MHz (datasheet Table 2, tSCKpause). At 168 MHz,
/// 500 ns = 84 CPU cycles — too short for async yield.
pub struct CycleDelay;

impl embedded_hal_async::delay::DelayNs for CycleDelay {
    async fn delay_ns(&mut self, ns: u32) {
        let cycles = (ns as u64 * 168 / 1000) as u32;
        cortex_m::asm::delay(cycles);
    }
}

/// TMC4671 SPI device with exclusive bus access and software CS.
pub type TmcSpiDevice = ExclusiveDevice<Spi1Bus, Output<'static>, CycleDelay>;

// Pin enumerations at crate root (outside `mod app`) so anchor_codegen
// can discover them. TMC4671 handles all pins internally — these exist
// only for Klipper config pin name resolution.
klipper_enumeration!(
    #[allow(non_camel_case_types)]
    #[klipper_enumeration(name = "pin")]
    pub enum Pins {
        Range(PA, 0, 16),
        Range(PB, 0, 16),
        Range(PC, 0, 16),
        Range(PD, 0, 16),
        Range(PE, 0, 16),
        // Virtual pins for FOCI stepper control.
        // STEP/DIR are handled internally (no GPIO toggle).
        // ENA maps to the TMC4671 enable pin (PE7 on OpenFFBoard).
        STEP0,
        DIR0,
        ENA0,
    }
);
