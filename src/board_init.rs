//! Board-bring-up helpers for OpenFFBoard.

use embassy_stm32::Peri;
use embassy_stm32::gpio::{AfType, Flex, Input, Level, Output, Pull, Speed};
use embassy_stm32::peripherals;

/// Bytes left unpainted below the live stack pointer so the paint cannot reach
/// the running frame or an exception frame stacked on top of it.
const STACK_PAINT_GUARD_BYTES: usize = 256;

/// Paint the unused span of the MSP stack and register it for later readout.
///
/// Must run before the init task builds any deeper frame: the span a later
/// frame occupies would otherwise never be painted, and the readout would
/// report that frame's depth as untouched headroom.
///
/// `_stack_end` is cortex-m-rt's top-of-statics address (`link.x` defines it as
/// `__euninit`, the end of `.uninit`, and asserts both its 4-byte alignment and
/// `_stack_start >= _stack_end`). The MSP grows down from `_stack_start` at the
/// end of RAM, so everything between the two is stack.
pub fn paint_stack() {
    unsafe extern "C" {
        static _stack_end: u32;
    }

    // SAFETY: `_stack_end` is the linker's end-of-statics address and the live
    // MSP grows down toward it, so every byte in `[_stack_end, sp)` is stack --
    // `.bss` and `.uninit` end at `_stack_end` and no output section is placed
    // above it. Dropping `STACK_PAINT_GUARD_BYTES` off the top excludes the
    // running frame, and RTIC runs `init` with interrupts masked, so nothing
    // else can be using the span while it is written. The bounds handed to
    // `register_painted_region` are exactly the span painted here, and the
    // stack stays readable for the life of the program.
    unsafe {
        let bottom = (&raw const _stack_end) as usize;
        let top = (cortex_m::register::msp::read() as usize)
            .saturating_sub(STACK_PAINT_GUARD_BYTES)
            & !0b11;
        if top <= bottom {
            return;
        }
        let len_words = (top - bottom) / size_of::<u32>();
        for index in 0..len_words {
            (bottom as *mut u32)
                .add(index)
                .write_volatile(foci_firmware::stack_watermark::STACK_PAINT_SENTINEL);
        }
        foci_firmware::stack_watermark::register_painted_region(bottom, len_words);
    }
}

/// Read the STM32 96-bit unique device ID and format as a hex string.
///
/// The UID is at 0x1FFF_7A10 on STM32F4. Returns a 24-character hex
/// string. Reads the 12 raw bytes sequentially (matching
/// Klipper/Katapult convention) and outputs high nibble first for each
/// byte.
pub fn read_stm32_uid(buf: &mut [u8; 24]) -> &str {
    const UID_BASE: *const u8 = 0x1FFF_7A10 as *const u8;
    let hex = b"0123456789ABCDEF";
    for i in 0..12 {
        // SAFETY: UID registers are always readable at this address on STM32F4
        let b = unsafe { core::ptr::read_volatile(UID_BASE.add(i)) };
        buf[i * 2] = hex[(b >> 4) as usize];
        buf[i * 2 + 1] = hex[(b & 0x0F) as usize];
    }
    // SAFETY: buf contains only ASCII hex characters
    unsafe { core::str::from_utf8_unchecked(buf) }
}

/// Build the embassy-stm32 clock configuration for 168 MHz SYSCLK,
/// 48 MHz USB: HSE 8 MHz → PLL → 168 MHz, USB uses PLL1_Q.
pub fn rcc_config() -> embassy_stm32::Config {
    use embassy_stm32::rcc::*;

    let mut config = embassy_stm32::Config::default();
    config.rcc.hse = Some(Hse {
        freq: embassy_stm32::time::Hertz(8_000_000),
        mode: HseMode::Oscillator,
    });
    config.rcc.pll_src = PllSource::HSE;
    config.rcc.pll = Some(Pll {
        prediv: PllPreDiv::DIV4,
        mul: PllMul::MUL168,
        divp: Some(PllPDiv::DIV2),
        divq: Some(PllQDiv::DIV7),
        divr: None,
    });
    config.rcc.ahb_pre = AHBPrescaler::DIV1;
    config.rcc.apb1_pre = APBPrescaler::DIV4;
    config.rcc.apb2_pre = APBPrescaler::DIV2;
    config.rcc.sys = Sysclk::PLL1_P;
    config.rcc.mux.clk48sel = embassy_stm32::pac::rcc::vals::Clk48sel::PLL1_Q;

    config.dma_interrupt_priority = SPI_DMA_INTERRUPT_PRIORITY;

    config
}

/// Configure TIM5 as the free-running 32-bit Klipper clock at 84 MHz.
///
/// TIM5 is the sole Klipper clock source and the step-timer base. CC1
/// interrupt is enabled later (first `queue_step` arms the timer);
/// unmasking TIM5 in the NVIC is similarly deferred so the UIF flag
/// set by the initial EGR.UG does not fire a spurious ISR.
pub fn init_tim5_clock() {
    let rcc = embassy_stm32::pac::RCC;
    rcc.apb1enr().modify(|w| w.set_tim5en(true));
    cortex_m::asm::delay(2);
    let tim5 = embassy_stm32::pac::TIM5;
    tim5.psc().write_value(0);
    tim5.arr().write_value(0xFFFF_FFFF);
    tim5.egr().write(|w| w.set_ug(true));
    tim5.cr1().modify(|w| w.set_cen(true));

    tim5.dier().write_value(Default::default());
    tim5.sr().write_value(Default::default());
}

/// Keep the debug/trace clock domain alive while the core sleeps.
///
/// The `#[idle]` task's `wfi()` puts the core into Sleep mode whenever no
/// interrupt is pending -- most of the time, given the 10 kHz control tick.
/// Without `DBG_SLEEP` set, an attached debugger can lose the core or stall
/// RTT reads across every sleep entry. `DBG_STOP`/`DBG_STANDBY` are left
/// unset: this firmware never sets `SLEEPDEEP`, so it never reaches Stop or
/// Standby.
pub fn enable_debug_clock_during_sleep() {
    embassy_stm32::pac::DBGMCU
        .cr()
        .modify(|w| w.set_dbg_sleep(true));
}

/// Configure PB11 (STEP) and PA0 (DIR) as push-pull outputs at very
/// high speed, initial level low.
pub fn init_stepdir_gpio() {
    use embassy_stm32::pac::gpio::vals;
    let rcc = embassy_stm32::pac::RCC;
    rcc.ahb1enr().modify(|w| w.set_gpioaen(true));
    rcc.ahb1enr().modify(|w| w.set_gpioben(true));

    let gpio_b = embassy_stm32::pac::GPIOB;
    gpio_b
        .moder()
        .modify(|w| w.set_moder(11, vals::Moder::OUTPUT));
    gpio_b
        .otyper()
        .modify(|w| w.set_ot(11, vals::Ot::PUSH_PULL));
    gpio_b
        .ospeedr()
        .modify(|w| w.set_ospeedr(11, vals::Ospeedr::VERY_HIGH_SPEED));
    gpio_b.odr().modify(|w| w.set_odr(11, vals::Odr::LOW));

    let gpio_a = embassy_stm32::pac::GPIOA;
    gpio_a
        .moder()
        .modify(|w| w.set_moder(0, vals::Moder::OUTPUT));
    gpio_a.otyper().modify(|w| w.set_ot(0, vals::Ot::PUSH_PULL));
    gpio_a
        .ospeedr()
        .modify(|w| w.set_ospeedr(0, vals::Ospeedr::VERY_HIGH_SPEED));
    gpio_a.odr().modify(|w| w.set_odr(0, vals::Odr::LOW));
}

/// Drive PE10 (brake chopper) LOW at boot so the brake resistor does
/// not fire while the STM32 boots with GPIO in analog mode.
pub fn init_brake_pin(pe10: Peri<'static, peripherals::PE10>) {
    let brake_pin = Output::new(pe10, Level::Low, Speed::Low);
    // Keep the pin configured — don't drop the handle.
    core::mem::forget(brake_pin);
}

/// Configure PE8 as a pulled-down input for the TMC4671 STATUS line and
/// return it. EXTI interrupt is enabled later in `tmc_control` after
/// `STATUS_MASK` is written to the TMC4671; the returned input is polled at
/// every control-loop wake.
pub fn init_tmc_status_pin(pe8: Peri<'static, peripherals::PE8>) -> Input<'static> {
    Input::new(pe8, Pull::Down)
}

/// Configure PA11 (USB D-) and PA12 (USB D+) for USB OTG FS.
/// AF 10 is the USB OTG FS alternate function on STM32F4.
pub fn init_usb_pins(
    pa11: Peri<'static, peripherals::PA11>,
    pa12: Peri<'static, peripherals::PA12>,
) {
    let mut dm = Flex::new(pa11);
    dm.set_as_af_unchecked(10, AfType::input(Pull::None));
    let mut dp = Flex::new(pa12);
    dp.set_as_af_unchecked(10, AfType::input(Pull::None));
    // Pins must stay configured — don't drop the Flex handles.
    core::mem::forget(dm);
    core::mem::forget(dp);
}

/// Enable the USB OTG FS peripheral clock and briefly delay so the
/// clock can stabilize before further register accesses.
pub fn enable_usb_clock() {
    embassy_stm32::pac::RCC
        .ahb2enr()
        .modify(|w| w.set_usb_otg_fsen(true));
    cortex_m::asm::delay(100);
}

/// Enable the DWT cycle counter.
///
/// CYCCNT is disabled on reset; it requires DEMCR.TRCENA and
/// DWT.CYCCNTENA to be set. Used for first-sample SPI timing
/// measurements.
pub fn enable_dwt_cyccnt() {
    // SAFETY: accessed exactly once during init. DCB and DWT are
    // Cortex-M system registers that are always safe to read/write here.
    unsafe {
        let dcb = &*cortex_m::peripheral::DCB::PTR;
        dcb.demcr.modify(|v| v | (1 << 24));
        let dwt = &*cortex_m::peripheral::DWT::PTR;
        dwt.cyccnt.write(0);
        dwt.ctrl.modify(|v| v | 1);
    }
}

/// NVIC priority of the SPI1 DMA completion interrupts: numerically above
/// (less urgent than) the RTIC P3 ceiling that [`verify_dma_interrupt_priority`] enforces.
const SPI_DMA_INTERRUPT_PRIORITY: embassy_stm32::interrupt::Priority =
    embassy_stm32::interrupt::Priority::P15;

/// Confirm the SPI1 RX/TX DMA completion interrupts (DMA2_STREAM2,
/// DMA2_STREAM3) are configured below the RTIC P3 hard-real-time ceiling
/// (TIM5/USB/STATUS -- raw NVIC priority `0xD0` on this 4-priority-bit part,
/// `Priority::P13` in Embassy's encoding).
///
/// Panics if either DMA interrupt is at or above that level: that would let
/// SPI DMA completion preempt STEP/DIR dispatch, the exact regression this
/// check exists to catch.
pub fn verify_dma_interrupt_priority() {
    use embassy_stm32::interrupt::{Interrupt, InterruptExt, Priority};

    let rx = Interrupt::DMA2_STREAM2.get_priority();
    let tx = Interrupt::DMA2_STREAM3.get_priority();
    defmt::info!(
        "SPI DMA NVIC priority: DMA2_STREAM2={} DMA2_STREAM3={}",
        rx,
        tx
    );
    assert!(
        rx > Priority::P13 && tx > Priority::P13,
        "SPI DMA interrupt priority is at or above RTIC P3 (TIM5/USB/STATUS)"
    );
}
