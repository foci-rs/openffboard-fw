#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

// PAC shim for RTIC + embassy-stm32
pub mod pac {
    pub use embassy_stm32::pac::Interrupt as interrupt;
    pub use embassy_stm32::pac::*;
    pub use embassy_stm32::Peripherals;
}

#[rtic::app(device = crate::pac, peripherals = true, dispatchers = [UART4, UART5, USART3])]
mod app {
    use core::cell::RefCell;
    use core::mem::MaybeUninit;

    use anchor::*;
    use bbqueue::BBBuffer;
    use cortex_m::peripheral::DWT;
    use critical_section::Mutex;
    use embassy_stm32::gpio::{Level, Output, Speed};
    use embassy_stm32::mode::Async as AsyncMode;
    use embassy_stm32::spi::Spi;
    use embassy_stm32::usb::Driver;
    use embassy_stm32::wdg::IndependentWatchdog;
    use embassy_stm32::{bind_interrupts, peripherals, usb};
    use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
    use embassy_sync::signal::Signal;
    use embassy_usb::class::cdc_acm::{CdcAcmClass, State as CdcState};
    use embassy_usb::{Builder, Config as UsbConfig, UsbDevice};
    use embedded_hal_bus::spi::ExclusiveDevice;
    use klipper_mcu::shutdown::McuState;
    use klipper_mcu::state::State;
    use rtic_monotonics::systick::prelude::*;
    use rtic_sync::channel::{Receiver, Sender};
    use tmc4671::spi::Tmc4671Spi;

    bind_interrupts!(struct Irqs {
        OTG_FS => usb::InterruptHandler<peripherals::USB_OTG_FS>;
        DMA2_STREAM2 => embassy_stm32::dma::InterruptHandler<peripherals::DMA2_CH2>;
        DMA2_STREAM3 => embassy_stm32::dma::InterruptHandler<peripherals::DMA2_CH3>;
    });

    systick_monotonic!(Mono, 1000);

    /// USB HAL driver type alias for use throughout this module.
    type HalUsbDriver = Driver<'static, peripherals::USB_OTG_FS>;

    /// SPI bus type for TMC4671 communication.
    type Spi1Bus = Spi<'static, AsyncMode, embassy_stm32::spi::mode::Master>;

    /// TMC4671 SPI device with exclusive bus access and software CS.
    type TmcSpiDevice = ExclusiveDevice<Spi1Bus, Output<'static>, embedded_hal_bus::spi::NoDelay>;

    /// Register access request from protocol handler to tmc_control task.
    enum TmcRequest {
        Write { addr: u8, value: u32 },
        Read { oid: u8, addr: u8 },
        SetEnable(bool),
        Init,
        TestSpin { revolutions: u8 },
        SetCurrent { run_ma: u32 },
    }

    const TMC_CHANNEL_SIZE: usize = 8;

    /// Current scaling for OpenFFBoard: milliamps per ADC least significant bit.
    /// Derived from sense resistor value and ADC configuration.
    const CURRENT_SCALE_MA_PER_LSB: f32 = 1.155;

    /// Static buffers required by the USB device stack.
    ///
    /// Held in `#[init(local = [...])]` storage so they obtain `'static` lifetime.
    struct UsbBuffers {
        config_descriptor: [u8; 256],
        bos_descriptor: [u8; 256],
        msos_descriptor: [u8; 0],
        control_buf: [u8; 64],
        cdc_state: MaybeUninit<CdcState<'static>>,
    }

    impl UsbBuffers {
        const fn new() -> Self {
            Self {
                config_descriptor: [0; 256],
                bos_descriptor: [0; 256],
                msos_descriptor: [],
                control_buf: [0; 64],
                cdc_state: MaybeUninit::uninit(),
            }
        }
    }

    // ---- bbqueue TX buffer and transport output ----

    const USB_TX_BUFFER_SIZE: usize = 512;

    static USB_TX_PRODUCER: Mutex<RefCell<Option<bbqueue::Producer<'static, USB_TX_BUFFER_SIZE>>>> =
        Mutex::new(RefCell::new(None));
    static USB_TX_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

    /// 64-bit uptime tracking. Updated by the stats task every 5 seconds.
    /// `stats_send_time` is the CYCCNT value at last stats send.
    /// `stats_send_time_high` counts 32-bit CYCCNT wraps (~25.5s at 168MHz).
    static STATS_SEND_TIME: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
    static STATS_SEND_TIME_HIGH: core::sync::atomic::AtomicU32 =
        core::sync::atomic::AtomicU32::new(0);

    /// Emergency motor disable signal. Set by enter_shutdown(), consumed
    /// by tmc_control. Uses a Signal so tmc_control wakes immediately
    /// even if blocked on an empty channel.
    static TMC_DISABLE_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

    static TMC_REQUEST_SENDER: Mutex<
        RefCell<Option<Sender<'static, TmcRequest, TMC_CHANNEL_SIZE>>>,
    > = Mutex::new(RefCell::new(None));

    pub struct BufferTransportOutput;
    pub const TRANSPORT_OUTPUT: BufferTransportOutput = BufferTransportOutput;

    impl TransportOutput for BufferTransportOutput {
        type Output = ScratchOutput;
        fn output(&self, f: impl FnOnce(&mut Self::Output)) {
            let mut scratch = ScratchOutput::new();
            f(&mut scratch);
            let output = scratch.result();

            critical_section::with(|cs| {
                if let Some(producer) = USB_TX_PRODUCER.borrow(cs).borrow_mut().as_mut() {
                    if let Ok(mut grant) = producer.grant_exact(output.len()) {
                        grant.buf().copy_from_slice(output);
                        grant.commit(output.len());
                        USB_TX_SIGNAL.signal(());
                    }
                }
            });
        }
    }

    // ---- Klipper constants ----

    #[klipper_constant]
    const CLOCK_FREQ: u32 = 168_000_000;

    #[klipper_constant]
    const MCU: &str = "stm32f407xx";

    #[klipper_constant]
    const STEPPER_STEP_BOTH_EDGE: u32 = 1;

    #[klipper_constant]
    const RECEIVE_WINDOW: u32 = 192;

    #[klipper_constant]
    const STATS_SUMSQ_BASE: u32 = 256;

    // ---- Klipper command handlers ----

    #[klipper_command(in_shutdown)]
    pub fn get_clock(_context: &mut State) {
        let clock = DWT::cycle_count();
        klipper_reply!(clock, clock: u32 = clock);
    }

    #[klipper_command(in_shutdown)]
    pub fn get_uptime(_context: &mut State) {
        use core::sync::atomic::Ordering::Relaxed;
        let cur = DWT::cycle_count();
        let high =
            STATS_SEND_TIME_HIGH.load(Relaxed) + u32::from(cur < STATS_SEND_TIME.load(Relaxed));
        klipper_reply!(uptime, high: u32 = high, clock: u32 = cur);
    }

    #[klipper_command(in_shutdown)]
    pub fn get_config(context: &mut State) {
        let is_config = context.config_crc.is_some();
        let crc = context.config_crc.unwrap_or(0);
        let is_shutdown = context.mcu_state.is_shutdown();
        klipper_reply!(
            config,
            is_config: bool = is_config,
            crc: u32 = crc,
            is_shutdown: bool = is_shutdown,
            move_count: u16 = 1024
        );
    }

    /// Clear shutdown state, allowing normal command processing to resume.
    ///
    /// Matches Klipper C `command_clear_shutdown` (basecmd.c). If called
    /// when not in shutdown, triggers a shutdown instead.
    #[klipper_command(in_shutdown)]
    pub fn clear_shutdown(context: &mut State) {
        if !context.mcu_state.is_shutdown() {
            enter_shutdown(context, "Shutdown cleared when not shutdown");
            return;
        }
        context.mcu_state = McuState::Running;
    }

    /// Full config reset: clear all OIDs, steppers, and shutdown state.
    ///
    /// Matches Klipper C `config_reset` (basecmd.c:262). Only available
    /// during shutdown. The host uses this as a fallback restart mechanism
    /// when hardware `reset` is not available (linux/PRU platforms).
    #[klipper_command(in_shutdown)]
    pub fn config_reset(context: &mut State) {
        if !context.mcu_state.is_shutdown() {
            enter_shutdown(context, "config_reset only available when shutdown");
            return;
        }
        context.reset();
    }

    #[klipper_command]
    pub fn finalize_config(context: &mut State, crc: u32) {
        context.config_crc = Some(crc);
        context.mcu_state = McuState::Running;
        defmt::info!("Config finalized, CRC: {:#010x}", crc);
    }

    #[klipper_command]
    pub fn allocate_oids(_count: u8) {
        // No-op: we use static allocation
    }

    #[klipper_command(in_shutdown)]
    pub fn emergency_stop(context: &mut State) {
        enter_shutdown(context, klipper_mcu::shutdown::reasons::COMMAND_REQUEST);
    }

    /// Transition to shutdown state: disable motor, notify host.
    fn enter_shutdown(context: &mut State, reason: &str) {
        if context.mcu_state.is_shutdown() {
            return;
        }
        context.mcu_state = McuState::Shutdown { reason: 0 };
        TMC_DISABLE_SIGNAL.signal(());
        defmt::error!("Shutdown: {}", reason);
        klipper_output!("shutdown static_string_id=%hu", 0u16);
    }

    /// Reset the MCU. Matches Klipper's `command_reset` (armcm_reset.c).
    ///
    /// Writes `CANBOOT_BYPASS` to Katapult's request signature so the
    /// bootloader skips straight to the application after reset. This is
    /// used by `FIRMWARE_RESTART`.
    #[klipper_command(in_shutdown)]
    pub fn reset() {
        katapult_reset(KATAPULT_BYPASS);
    }

    #[klipper_command(in_shutdown)]
    pub fn debug_nop() {}

    // Katapult bootloader constants (from Klipper armcm_reset.c).
    const KATAPULT_BOOT_ADDRESS: u32 = 0x0800_0000;
    const KATAPULT_SIGNATURE: u64 = 0x2174_6f6f_426e_6143; // "CanBoot!"
    const KATAPULT_REQUEST: u64 = 0x5984_E3FA_6CA1_589B; // stay in bootloader
    const KATAPULT_BYPASS: u64 = 0x7b06_ec45_a9a8_243d; // skip bootloader

    /// Write a Katapult request signature and perform a system reset.
    ///
    /// `KATAPULT_REQUEST`: stay in bootloader (for reflashing via 1200-baud DTR).
    /// `KATAPULT_BYPASS`: skip bootloader, boot application (for FIRMWARE_RESTART).
    ///
    /// Matches Klipper's `canboot_reset()` in armcm_reset.c. Verifies the
    /// bootloader signature before writing, no-ops if no bootloader is present.
    fn katapult_reset(req_signature: u64) {
        // SAFETY: The bootloader vector table at KATAPULT_BOOT_ADDRESS is
        // always readable. We verify the Katapult signature before writing.
        unsafe {
            let bl_vectors = KATAPULT_BOOT_ADDRESS as *const u32;
            let reset_handler = core::ptr::read_volatile(bl_vectors.add(1));
            let boot_sig_addr = (reset_handler - 9) as *const u64;
            let req_sig_addr = core::ptr::read_volatile(bl_vectors) as *mut u64;

            // Verify alignment and signature
            if !(boot_sig_addr as usize).is_multiple_of(8)
                || core::ptr::read_volatile(boot_sig_addr) != KATAPULT_SIGNATURE
                || !(req_sig_addr as usize).is_multiple_of(8)
            {
                // No valid bootloader — just reset
                cortex_m::peripheral::SCB::sys_reset();
            }

            cortex_m::interrupt::disable();
            core::ptr::write_volatile(req_sig_addr, req_signature);
        }

        cortex_m::peripheral::SCB::sys_reset();
    }

    // ---- Stepper commands ----

    /// Register a stepper motor object.
    ///
    /// Pin parameters are accepted for protocol compatibility but ignored;
    /// the TMC4671 handles stepping internally.
    #[klipper_command]
    pub fn config_stepper(
        context: &mut State,
        oid: u8,
        _step_pin: u8,
        _dir_pin: u8,
        _invert_step: u8,
        _step_pulse_ticks: u32,
    ) {
        let idx = context.steppers_by_oid.len();
        if idx >= klipper_mcu::state::NUM_STEPPERS {
            defmt::error!("config_stepper: no stepper slots available");
            return;
        }
        if context.steppers_by_oid.insert(oid, idx).is_err() {
            defmt::error!("config_stepper: OID map full");
            return;
        }
        defmt::info!(
            "config_stepper: oid={} idx={} (pins ignored, TMC4671 mode)",
            oid,
            idx
        );
    }

    /// Enqueue a trapezoidal move segment for a stepper.
    #[klipper_command]
    pub fn queue_step(context: &mut State, oid: u8, interval: u32, count: u16, add: i16) {
        if let Some(stepper) = context.stepper_mut(oid) {
            let m = klipper_mcu::stepper::StepMove {
                interval,
                count,
                add,
            };
            if stepper.queue_move(m).is_err() {
                defmt::error!("queue_step: move queue overflow for oid={}", oid);
            }
        }
    }

    /// Set the direction for subsequent steps on a stepper.
    ///
    /// Klipper convention: `dir == 0` means forward, `dir == 1` means backward.
    #[klipper_command]
    pub fn set_next_step_dir(context: &mut State, oid: u8, dir: u8) {
        if let Some(stepper) = context.stepper_mut(oid) {
            stepper.set_direction(dir == 0);
        }
    }

    /// Reset the step clock reference for a stepper.
    #[klipper_command]
    pub fn reset_step_clock(context: &mut State, oid: u8, clock: u32) {
        if let Some(stepper) = context.stepper_mut(oid) {
            stepper.reset_clock(clock);
        }
    }

    /// Report the current position of a stepper.
    #[klipper_command]
    pub fn stepper_get_position(context: &mut State, oid: u8) {
        let pos = context.stepper(oid).map(|s| s.position()).unwrap_or(0);
        klipper_reply!(stepper_position, oid: u8 = oid, pos: i32 = pos);
    }

    /// Report the commanded (target) position of a stepper.
    ///
    /// For TMC4671 mode this is equivalent to `stepper_get_position` since
    /// we track only the logical step count.
    #[klipper_command]
    pub fn stepper_get_commanded_position(context: &mut State, oid: u8) {
        let pos = context.stepper(oid).map(|s| s.position()).unwrap_or(0);
        klipper_reply!(stepper_position, oid: u8 = oid, pos: i32 = pos);
    }

    /// Stop a stepper on a trigger-sync event.
    ///
    /// TrSync is not implemented in Phase 1; accept the command silently.
    #[klipper_command]
    pub fn stepper_stop_on_trigger(context: &mut State, _oid: u8, _trsync_oid: u8) {
        let _ = context;
    }

    // ---- Digital output commands (motor enable/disable) ----

    /// Register a digital output object for motor enable/disable.
    ///
    /// Pin parameters are accepted for protocol compatibility but ignored;
    /// the TMC4671 enable line is controlled directly.
    #[klipper_command]
    pub fn config_digital_out(
        context: &mut State,
        oid: u8,
        _pin: u32,
        _value: u8,
        _default_value: u8,
        _max_duration: u32,
    ) {
        let idx = context.enable_by_oid.len();
        if idx < klipper_mcu::state::NUM_STEPPERS {
            let _ = context.enable_by_oid.insert(oid, idx);
        }
        defmt::info!(
            "config_digital_out: oid={} (enable control, pin ignored)",
            oid
        );
    }

    /// Schedule a digital output change at a given clock time.
    ///
    /// Used by Klipper for timed motor enable/disable.
    #[klipper_command]
    pub fn queue_digital_out(context: &mut State, oid: u8, clock: u32, on_ticks: u32) {
        let _ = clock;
        if let Some(&idx) = context.enable_by_oid.get(&oid) {
            let enabled = on_ticks != 0;
            context.steppers[idx].set_enabled(enabled);
            tmc_send(TmcRequest::SetEnable(enabled));
        }
    }

    /// Immediately update a digital output value.
    #[klipper_command]
    pub fn update_digital_out(context: &mut State, oid: u8, value: u8) {
        if let Some(&idx) = context.enable_by_oid.get(&oid) {
            let enabled = value != 0;
            context.steppers[idx].set_enabled(enabled);
            tmc_send(TmcRequest::SetEnable(enabled));
        }
    }

    // ---- Stub commands for Klipper protocol compatibility ----

    /// TrSync configuration stub (Phase 2).
    #[klipper_command]
    pub fn config_trsync(_oid: u8) {}

    /// TrSync start stub (Phase 2).
    ///
    /// Immediately reports completion with the given expire_reason so
    /// Klipper doesn't wait indefinitely for a trigger.
    #[klipper_command]
    pub fn trsync_start(
        _context: &mut State,
        oid: u8,
        report_clock: u32,
        _report_ticks: u32,
        expire_reason: u8,
    ) {
        klipper_reply!(
            trsync_state,
            oid: u8 = oid,
            can_trigger: u8 = 0,
            trigger_reason: u8 = expire_reason,
            clock: u32 = report_clock
        );
    }

    /// TrSync timeout stub (Phase 2).
    #[klipper_command]
    pub fn trsync_set_timeout(_oid: u8, _clock: u32) {}

    /// TrSync trigger stub (Phase 2).
    #[klipper_command]
    pub fn trsync_trigger(_oid: u8, _reason: u8) {}

    /// Endstop configuration stub.
    #[klipper_command]
    pub fn config_endstop(_oid: u8, _pin: u8, _pull_up: u8) {}

    /// Endstop homing stub.
    #[klipper_command]
    #[allow(clippy::too_many_arguments)]
    pub fn endstop_home(
        _oid: u8,
        _clock: u32,
        _sample_ticks: u32,
        _sample_count: u8,
        _rest_ticks: u32,
        _pin_value: u8,
        _trsync_oid: u8,
        _trigger_reason: u8,
    ) {
    }

    /// Reply with endstop state (always "not triggered").
    #[klipper_command]
    pub fn endstop_query_state(_context: &mut State, oid: u8) {
        klipper_reply!(
            endstop_state,
            oid: u8 = oid,
            homing: u8 = 0,
            next_clock: u32 = 0,
            pin_value: u8 = 0
        );
    }

    /// SPI configuration stub (TMC4671 SPI is handled internally).
    #[klipper_command]
    pub fn config_spi(_oid: u8, _pin: u32, _cs_active_high: u8) {}

    /// SPI configuration without chip-select stub.
    #[klipper_command]
    pub fn config_spi_without_cs(_oid: u8) {}

    /// SPI bus assignment stub.
    #[klipper_command]
    pub fn spi_set_bus(_oid: u8, _spi_bus: u32, _mode: u32, _rate: u32) {}

    /// SPI shutdown message configuration stub.
    #[klipper_command]
    pub fn config_spi_shutdown(_oid: u8, _spi_oid: u8, _shutdown_msg: &[u8]) {}

    /// SPI send stub (data is accepted but not forwarded).
    #[klipper_command]
    pub fn spi_send(_oid: u8, _data: &[u8]) {}

    // ---- TMC4671 register access commands ----

    /// Send a request to the tmc_control task.
    /// Send a request to the tmc_control task.
    fn tmc_send(req: TmcRequest) {
        critical_section::with(|cs| {
            if let Some(sender) = TMC_REQUEST_SENDER.borrow(cs).borrow_mut().as_mut() {
                let _ = sender.try_send(req);
            }
        });
    }

    /// Trigger TMC4671 initialization sequence.
    #[klipper_command]
    pub fn tmc_init(_context: &mut State, _oid: u8) {
        tmc_send(TmcRequest::Init);
    }

    /// Test motor rotation by sweeping PHI_E_EXT through N revolutions.
    #[klipper_command]
    pub fn tmc_test_spin(_context: &mut State, _oid: u8, revolutions: u8) {
        tmc_send(TmcRequest::TestSpin { revolutions });
    }

    /// Write a TMC4671 register via SPI.
    #[klipper_command]
    pub fn tmc_write_register(_context: &mut State, _oid: u8, addr: u8, value: u32) {
        tmc_send(TmcRequest::Write { addr, value });
    }

    /// Read a TMC4671 register via SPI.
    ///
    /// The response is sent asynchronously as a tmc_register_value reply.
    #[klipper_command]
    pub fn tmc_read_register(_context: &mut State, oid: u8, addr: u8) {
        tmc_send(TmcRequest::Read { oid, addr });
    }

    /// Set motor run current. The firmware converts milliamps to
    /// PID_TORQUE_FLUX_LIMITS register counts using the board-specific
    /// current scaling factor.
    #[klipper_command]
    pub fn tmc_set_current(_context: &mut State, _oid: u8, run_ma: u32) {
        tmc_send(TmcRequest::SetCurrent { run_ma });
    }

    /// SPI transfer stub (returns empty response).
    #[klipper_command]
    pub fn spi_transfer(_oid: u8, _data: &[u8]) {
        let empty: &[u8] = &[];
        klipper_reply!(spi_transfer_response, oid: u8 = _oid, response: &[u8] = empty);
    }

    // ---- RTIC resources ----

    #[shared]
    struct Shared {
        app_state: State,
    }

    #[local]
    struct Local {
        led: Output<'static>,
        tmc: Tmc4671Spi<TmcSpiDevice>,
        tmc_enable: Output<'static>,
        tmc_rx: Receiver<'static, TmcRequest, TMC_CHANNEL_SIZE>,
        wdg: IndependentWatchdog<'static, peripherals::IWDG>,
        usb: UsbDevice<'static, HalUsbDriver>,
        usb_sender: embassy_usb::class::cdc_acm::Sender<'static, HalUsbDriver>,
        usb_receiver: embassy_usb::class::cdc_acm::Receiver<'static, HalUsbDriver>,
        usb_control: embassy_usb::class::cdc_acm::ControlChanged<'static>,
        usb_tx_consumer: bbqueue::Consumer<'static, USB_TX_BUFFER_SIZE>,
    }

    /// Read the STM32 96-bit unique device ID and format as a hex string.
    ///
    /// The UID is at 0x1FFF_7A10 on STM32F4. Returns a 24-character hex string.
    /// Read the STM32 96-bit unique device ID and format as a hex string.
    ///
    /// Reads the 12 raw bytes sequentially (matching Klipper/Katapult convention)
    /// and outputs high nibble first for each byte.
    fn read_stm32_uid(buf: &mut [u8; 24]) -> &str {
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

    #[init(local = [
        usb_buf: UsbBuffers = UsbBuffers::new(),
        ep_out_buf: [u8; 256] = [0; 256],
        usb_tx_queue: BBBuffer<USB_TX_BUFFER_SIZE> = BBBuffer::new(),
        tmc_channel: rtic_sync::channel::Channel<TmcRequest, TMC_CHANNEL_SIZE> =
            rtic_sync::channel::Channel::new(),
        uid_str: [u8; 24] = [0; 24],
    ])]
    fn init(mut ctx: init::Context) -> (Shared, Local) {
        defmt::info!("FOCI OpenFFBoard starting");

        // Configure clock tree: HSE 8 MHz -> PLL -> 168 MHz SYSCLK, 48 MHz USB
        let mut config = embassy_stm32::Config::default();
        {
            use embassy_stm32::rcc::*;
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
        }
        let p = embassy_stm32::init(config);

        defmt::info!("SYSCLK: 168 MHz, USB: 48 MHz");

        // Enable DWT cycle counter for Klipper clock reporting
        ctx.core.DCB.enable_trace();
        ctx.core.DWT.enable_cycle_counter();

        defmt::info!("DWT CYCCNT enabled for Klipper clock @ 168 MHz");

        Mono::start(ctx.core.SYST, 168_000_000);

        // PD7: system status LED on OpenFFBoard
        let led = Output::new(p.PD7, Level::Low, Speed::Low);

        // PE7: TMC4671 ENABLE pin (active high, start disabled)
        let tmc_enable = Output::new(p.PE7, Level::Low, Speed::Low);

        // IWDG: 10-second timeout, kicked by the stats task every 5 seconds
        let mut wdg = IndependentWatchdog::new(p.IWDG, 10_000_000);
        wdg.unleash();
        defmt::info!("IWDG started: 10s timeout");

        // SPI1: TMC4671 communication at 2 MHz, Mode 3 (CPOL=1, CPHA=1)
        let mut spi_config = embassy_stm32::spi::Config::default();
        spi_config.mode = embassy_stm32::spi::Mode {
            polarity: embassy_stm32::spi::Polarity::IdleHigh,
            phase: embassy_stm32::spi::Phase::CaptureOnSecondTransition,
        };
        spi_config.frequency = embassy_stm32::time::Hertz(2_000_000);

        let spi1 = Spi::new(
            p.SPI1, p.PA5,      // SCK
            p.PA7,      // MOSI
            p.PA6,      // MISO
            p.DMA2_CH3, // TX DMA
            p.DMA2_CH2, // RX DMA
            Irqs, spi_config,
        );

        // PA4: TMC4671 chip select (active low)
        let spi_cs = Output::new(p.PA4, Level::High, Speed::VeryHigh);
        let spi_dev = ExclusiveDevice::new_no_delay(spi1, spi_cs).unwrap();
        let tmc = Tmc4671Spi::new(spi_dev);

        // TMC register request channel
        let (tmc_tx, tmc_rx) = ctx.local.tmc_channel.split();
        critical_section::with(|cs| {
            TMC_REQUEST_SENDER.borrow(cs).replace(Some(tmc_tx));
        });

        defmt::info!("SPI1 configured: 2 MHz, Mode 3, DMA");

        // USB OTG FS driver setup
        // vbus_detection = true: OpenFFBoard has VBUS wired to PA9
        let mut usb_hw_config = usb::Config::default();
        usb_hw_config.vbus_detection = true;

        let driver = Driver::new_fs(
            p.USB_OTG_FS,
            Irqs,
            p.PA12, // D+
            p.PA11, // D-
            ctx.local.ep_out_buf,
            usb_hw_config,
        );

        // USB device descriptor configuration — Klipper VID/PID
        let mut usb_config = UsbConfig::new(0x1D50, 0x614E);
        usb_config.manufacturer = Some("foci");
        usb_config.product = Some("FOCI OpenFFBoard");
        let uid = read_stm32_uid(ctx.local.uid_str);
        usb_config.serial_number = Some(uid);
        usb_config.max_power = 100;
        usb_config.max_packet_size_0 = 64;

        let usb_buf = ctx.local.usb_buf;

        let mut builder = Builder::new(
            driver,
            usb_config,
            &mut usb_buf.config_descriptor,
            &mut usb_buf.bos_descriptor,
            &mut usb_buf.msos_descriptor,
            &mut usb_buf.control_buf,
        );

        // SAFETY: cdc_state is uninit storage; write() initialises it exactly once
        // here before the reference escapes. No other code accesses this slot.
        let cdc_state = usb_buf.cdc_state.write(CdcState::new());
        let class = CdcAcmClass::new(&mut builder, cdc_state, 64);
        let usb = builder.build();
        let (usb_sender, usb_receiver, usb_control) = class.split_with_control();

        // Split bbqueue for TX output
        let (usb_tx_prod, usb_tx_consumer) = ctx.local.usb_tx_queue.try_split().unwrap();
        critical_section::with(|cs| {
            USB_TX_PRODUCER.borrow(cs).replace(Some(usb_tx_prod));
        });

        // Spawn tasks
        tmc_control::spawn().ok();
        usb_pump::spawn().ok();
        usb_receive::spawn().ok();
        usb_send::spawn().ok();
        usb_control::spawn().ok();
        blink::spawn().ok();
        stats::spawn().ok();

        defmt::info!("FOCI OpenFFBoard initialized");

        (
            Shared {
                app_state: State::new(),
            },
            Local {
                led,
                tmc,
                tmc_enable,
                tmc_rx,
                wdg,
                usb,
                usb_sender,
                usb_receiver,
                usb_control,
                usb_tx_consumer,
            },
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            rtic::export::wfi();
        }
    }

    /// TMC4671 control task. Runs at priority 1 (preempted by protocol tasks).
    ///
    /// On startup, reads the TMC4671 chip identification register to verify
    /// SPI communication. Then processes register read/write requests from
    /// the protocol handler via the TMC request channel.
    #[task(priority = 1, local = [tmc, tmc_enable, tmc_rx])]
    async fn tmc_control(ctx: tmc_control::Context) {
        let tmc = ctx.local.tmc;
        let enable_pin = ctx.local.tmc_enable;
        let rx = ctx.local.tmc_rx;

        // NOTE: auto-init disabled - motor current regulation not yet safe
        // tmc_init_open_loop(tmc, enable_pin).await;

        // Verify TMC4671 chip identity
        match tmc
            .read_sub_register(tmc4671::registers::CHIPINFO_SI_TYPE)
            .await
        {
            Ok(chip_id) => {
                if chip_id == 0x3436_3731 {
                    defmt::info!("TMC4671 chip ID verified: {:#010x}", chip_id);
                } else {
                    defmt::error!(
                        "TMC4671 chip ID mismatch: expected 0x34363731, got {:#010x}",
                        chip_id
                    );
                }
            }
            Err(_) => {
                defmt::error!("TMC4671 SPI communication failed");
            }
        }

        // Process register read/write requests from the host.
        // Use select to race between channel recv and emergency disable
        // signal, ensuring motor disable is never dropped.
        loop {
            use embassy_futures::select::{select, Either};

            match select(rx.recv(), TMC_DISABLE_SIGNAL.wait()).await {
                Either::First(Ok(req)) => match req {
                    TmcRequest::Write { addr, value } => {
                        let reg = tmc4671::registers::Register::new(addr);
                        if tmc.write_register(reg, value).await.is_err() {
                            defmt::error!("TMC4671 SPI write failed: addr={:#04x}", addr);
                        }
                    }
                    TmcRequest::Read { oid, addr } => {
                        let reg = tmc4671::registers::Register::new(addr);
                        match tmc.read_register(reg).await {
                            Ok(value) => {
                                klipper_reply!(
                                    tmc_register_value,
                                    oid: u8 = oid,
                                    addr: u8 = addr,
                                    value: u32 = value
                                );
                            }
                            Err(_) => {
                                defmt::error!("TMC4671 SPI read failed: addr={:#04x}", addr);
                            }
                        }
                    }
                    TmcRequest::SetEnable(enabled) => {
                        if enabled {
                            enable_pin.set_high();
                        } else {
                            enable_pin.set_low();
                        }
                        defmt::info!("TMC4671 enable: {}", enabled);
                    }
                    TmcRequest::Init => {
                        defmt::info!("TMC4671 init: starting open-loop stepper config");
                        tmc_init_open_loop(tmc, enable_pin).await;
                    }
                    TmcRequest::TestSpin { revolutions } => {
                        do_test_spin(tmc, revolutions).await;
                    }
                    TmcRequest::SetCurrent { run_ma } => {
                        use tmc4671::registers::*;
                        let counts = (run_ma as f32 / CURRENT_SCALE_MA_PER_LSB) as u32;
                        let limit = (counts << 16) | counts;
                        if tmc.write_register(PID_TORQUE_FLUX_LIMITS, limit).await.is_err() {
                            defmt::error!("TMC4671 SPI write failed: PID_TORQUE_FLUX_LIMITS");
                        }
                        defmt::info!("TMC4671 current: {}mA -> {} counts", run_ma, counts);
                    }
                },
                Either::First(Err(_)) => {}
                Either::Second(()) => {
                    enable_pin.set_low();
                    defmt::warn!("TMC4671 emergency disable");
                }
            }
        }
    }

    /// Sweep PHI_E_EXT through N full mechanical revolutions.
    ///
    /// With 50 pole pairs, one mechanical revolution = 50 electrical
    /// cycles. Each electrical cycle sweeps PHI_E_EXT from 0 to 65535.
    /// Uses whatever voltage/mode settings were configured by tmc_init.
    async fn do_test_spin(tmc: &mut Tmc4671Spi<TmcSpiDevice>, revolutions: u8) {
        use tmc4671::registers::*;

        let total_steps = revolutions as u32 * 50 * 256; // 256 microsteps per e-cycle
        defmt::info!(
            "Spinning {} revolutions ({} steps)",
            revolutions,
            total_steps
        );

        for step in 0..total_steps {
            let angle = (step * 256) % 65536;
            let _ = tmc.write_register(PHI_E_EXT, angle).await;
            Mono::delay(1.millis()).await;
        }

        // Return to angle 0
        let _ = tmc.write_register(PHI_E_EXT, 0).await;
        defmt::info!("Spin complete");
    }

    /// Configure TMC4671 for open-loop stepper operation with current regulation.
    ///
    /// Sequence:
    /// 1. Stop motor, configure PWM and ADC
    /// 2. Calibrate ADC offsets (motor should be disconnected or idle)
    /// 3. Set up torque mode with current limit
    /// 4. PHI_E_EXT for open-loop angle control
    ///
    /// Register values verified against TMC4671-LA datasheet rev 2.08.
    async fn tmc_init_open_loop(
        tmc: &mut Tmc4671Spi<TmcSpiDevice>,
        enable_pin: &mut Output<'static>,
    ) {
        use tmc4671::registers::*;

        // 1. SAFETY FIRST: stop motor, disable outputs, set voltage limit
        // The power-on default PIDOUT_UQ_UD_LIMITS is 32767 (full voltage).
        // We MUST set a safe limit before ANYTHING else touches the motor.
        enable_pin.set_low();
        let _ = tmc.write_register(MODE_RAMP_MODE_MOTION, 0x0000_0000).await;
        let voltage_limit: u32 = 1024;
        let _ = tmc.write_register(PIDOUT_UQ_UD_LIMITS, voltage_limit).await;
        // Verify the write took effect
        let readback = tmc.read_register(PIDOUT_UQ_UD_LIMITS).await.unwrap_or(0);
        defmt::info!(
            "PIDOUT_UQ_UD_LIMITS: wrote={}, read={}",
            voltage_limit,
            readback
        );
        // Also zero the torque target to prevent any current drive
        let _ = tmc.write_register(PID_TORQUE_FLUX_TARGET, 0).await;
        // And zero the UQ/UD external values
        let _ = tmc.write_register(UQ_UD_EXT, 0).await;

        // 2. PWM configuration (chopping disabled for ADC calibration)
        let _ = tmc.write_register(PWM_POLARITIES, 0).await;
        let _ = tmc.write_register(PWM_MAXCNT, 3999).await; // 25kHz
        let _ = tmc.write_register(PWM_BBM_H_BBM_L, (10 << 8) | 10).await;
        // PWM_SV=0 (SVPWM off per DS 4.9.7 - stepper only), CHOP=0 for ADC cal
        let _ = tmc.write_register(PWM_SV_CHOP, 0).await;

        // 3. ADC configuration
        // DSADC_MCFG: leave at power-on default (ANALOG mode, internal ADC)
        let mdec: u32 = 331;
        let _ = tmc
            .write_register(DSADC_MDEC_B_MDEC_A, (mdec << 16) | mdec)
            .await;
        // bits[29:28] ADC_I_WY_SELECT=1 (WY phase current from ADC_I1)
        // bits[27:26] ADC_I_V_SELECT=0  (V unused for stepper FOC2)
        // bits[25:24] ADC_I_UX_SELECT=0 (UX phase current from ADC_I0, default)
        // bits[15:8]  ADC_I1_SELECT=1   (I1 sourced from internal DSADC ch1)
        let _ = tmc.write_register(ADC_I_SELECT, (1 << 28) | (1 << 8)).await;

        // 4. Motor type: 2-phase stepper (type 2 per datasheet), 50 pole pairs
        let _ = tmc
            .write_register(MOTOR_TYPE_N_POLE_PAIRS, (2 << 16) | 50)
            .await;

        // 5. ADC offset calibration
        // With PWM_CHOP=0, no current flows even with motor connected.
        // This matches the Python driver's _calibrate_adc() method.
        let _ = tmc.write_register(ADC_I0_SCALE_OFFSET, 256 << 16).await;
        let _ = tmc.write_register(ADC_I1_SCALE_OFFSET, 256 << 16).await;

        // Enable to power the ADC, but CHOP=0 means no PWM switching
        enable_pin.set_high();
        Mono::delay(250.millis()).await;

        // Sample ADC values 128 times and average for offset
        let mut sum_i0: i32 = 0;
        let mut sum_i1: i32 = 0;
        const N_SAMPLES: i32 = 128;
        for _ in 0..N_SAMPLES {
            if let Ok(raw) = tmc.read_register(ADC_IWY_IUX).await {
                let i_ux = (raw & 0xFFFF) as i16 as i32;
                let i_wy = ((raw >> 16) & 0xFFFF) as i16 as i32;
                sum_i0 += i_ux;
                sum_i1 += i_wy;
            }
            Mono::delay(1.millis()).await;
        }
        // Cast via i16 to u16: negative values wrap correctly for the
        // TMC4671's unsigned offset subtraction register.
        let offset_i0 = (sum_i0 / N_SAMPLES) as i16 as u16;
        let offset_i1 = (sum_i1 / N_SAMPLES) as i16 as u16;

        let _ = tmc
            .write_register(ADC_I0_SCALE_OFFSET, (256 << 16) | offset_i0 as u32)
            .await;
        let _ = tmc
            .write_register(ADC_I1_SCALE_OFFSET, (256 << 16) | offset_i1 as u32)
            .await;

        defmt::info!(
            "ADC calibrated: I0 offset={}, I1 offset={}",
            offset_i0,
            offset_i1
        );

        // 6. Re-enable PWM chopping after calibration (SVPWM stays off for stepper)
        let _ = tmc.write_register(PWM_SV_CHOP, 7).await;

        // 7. PID gains for torque (current) control
        // Conservative Q8.8 PID gains: P = 1.577 * 256 = 404, I = 0.1 * 256 = 26.
        let pid_pi: u32 = (404 << 16) | 26;
        let _ = tmc.write_register(PID_FLUX_P_FLUX_I, pid_pi).await;
        let _ = tmc.write_register(PID_TORQUE_P_TORQUE_I, pid_pi).await;

        // 9. Current limit via PID_TORQUE_FLUX_LIMITS
        // Both torque and flux limited to 433 counts (~0.5A with 1.155 mA/LSB).
        // Flux limit must be nonzero so the flux PID can actively regulate
        // the D-axis current to zero (reject disturbances).
        let current_limit: u32 = 433;
        let _ = tmc
            .write_register(
                PID_TORQUE_FLUX_LIMITS,
                (current_limit << 16) | current_limit,
            )
            .await;

        // 10. PHI_E source: external angle register (open-loop)
        let _ = tmc.write_register(PHI_E_SELECTION, 1).await;
        let _ = tmc.write_register(PHI_E_EXT, 0).await;

        // 11. Set torque target: 200 counts (~0.23A, conservative)
        let torque_target: u32 = 200;
        let _ = tmc
            .write_register(PID_TORQUE_FLUX_TARGET, torque_target << 16)
            .await;

        // 12. Motion mode: TORQUE mode with current regulation
        // MODE_PID_TYPE=1 (bit 31), MODE_MOTION=1 (torque)
        let _ = tmc.write_register(MODE_RAMP_MODE_MOTION, 0x8000_0001).await;

        // Verify critical registers after enabling torque mode
        let v_limit_final = tmc.read_register(PIDOUT_UQ_UD_LIMITS).await.unwrap_or(0);
        let mode_final = tmc.read_register(MODE_RAMP_MODE_MOTION).await.unwrap_or(0);
        let target_final = tmc.read_register(PID_TORQUE_FLUX_TARGET).await.unwrap_or(0);
        defmt::info!(
            "TMC4671 init done: V_limit={}, mode=0x{:08X}, target=0x{:08X}",
            v_limit_final,
            mode_final,
            target_final
        );

        // Diagnostic: log PID state for 5 seconds after enabling
        for i in 0..10u32 {
            Mono::delay(500.millis()).await;
            let actual = tmc.read_register(PID_TORQUE_FLUX_ACTUAL).await.unwrap_or(0);
            let adc = tmc.read_register(ADC_IWY_IUX).await.unwrap_or(0);
            let torque_actual = ((actual >> 16) & 0xFFFF) as i16;
            let flux_actual = (actual & 0xFFFF) as i16;
            let i_ux = (adc & 0xFFFF) as i16;
            let i_wy = ((adc >> 16) & 0xFFFF) as i16;
            defmt::info!(
                "[{}] PID: torque={} flux={} | ADC: ux={} wy={}",
                i,
                torque_actual,
                flux_actual,
                i_ux,
                i_wy
            );
        }
    }

    /// Monitors CDC-ACM control line changes for bootloader entry.
    ///
    /// Katapult's flashtool requests bootloader entry by setting DTR,
    /// changing baud to 1200, then dropping DTR. We detect the DTR
    /// high-to-low transition with 1200 baud to avoid false triggers
    /// from normal Klipper serial port handling.
    #[task(priority = 2, local = [usb_control, dtr_was_set: bool = false])]
    async fn usb_control(ctx: usb_control::Context) {
        loop {
            ctx.local.usb_control.control_changed().await;
            let dtr = ctx.local.usb_control.dtr();
            if *ctx.local.dtr_was_set
                && !dtr
                && ctx.local.usb_control.line_coding().data_rate() == 1200
            {
                defmt::info!("1200 baud + DTR drop: entering Katapult bootloader");
                katapult_reset(KATAPULT_REQUEST);
            }
            *ctx.local.dtr_was_set = dtr;
        }
    }

    /// Periodic stats heartbeat sent to Klipper.
    ///
    /// Klipper requires regular stats messages to confirm the MCU is alive.
    /// Sends `stats count=%u sum=%u sumsq=%u` every 5 seconds with basic
    /// CPU usage metrics. Also kicks the IWDG watchdog.
    #[task(priority = 1, local = [wdg])]
    async fn stats(ctx: stats::Context) {
        let mut count: u32 = 0;
        let mut sum: u32 = 0;
        let mut sumsq: u32 = 0;
        let mut last_clock = DWT::cycle_count();

        loop {
            Mono::delay(5000.millis()).await;

            let now = DWT::cycle_count();
            let elapsed = now.wrapping_sub(last_clock);
            last_clock = now;

            // Track one "iteration" per stats period with elapsed ticks
            count += 1;
            sum += elapsed;
            let diff_scaled = elapsed / STATS_SUMSQ_BASE;
            sumsq = sumsq.saturating_add(diff_scaled.saturating_mul(elapsed));

            // Track 32-bit clock wraps for get_uptime high word.
            // Store time before incrementing high to avoid a race with
            // get_uptime (P2) preempting between the two updates.
            {
                use core::sync::atomic::Ordering::Relaxed;
                let prev = STATS_SEND_TIME.swap(now, Relaxed);
                if now < prev {
                    STATS_SEND_TIME_HIGH.fetch_add(1, Relaxed);
                }
            }

            defmt::info!("stats tick");
            klipper_output!("stats count=%u sum=%u sumsq=%u", count, sum, sumsq);
            ctx.local.wdg.pet();

            count = 0;
            sum = 0;
            sumsq = 0;
        }
    }

    /// Heartbeat task: toggles the system LED every 500 ms.
    #[task(priority = 1, local = [led])]
    async fn blink(ctx: blink::Context) {
        loop {
            ctx.local.led.toggle();
            Mono::delay(500.millis()).await;
        }
    }

    /// Drives the USB device state machine; must run continuously.
    #[task(priority = 2, local = [usb])]
    async fn usb_pump(ctx: usb_pump::Context) {
        ctx.local.usb.run().await;
    }

    /// Receives USB packets and feeds them to the anchor protocol handler.
    #[task(priority = 2, local = [usb_receiver], shared = [app_state])]
    async fn usb_receive(mut ctx: usb_receive::Context) {
        let receiver = ctx.local.usb_receiver;
        loop {
            receiver.wait_connection().await;
            defmt::info!("USB connected");
            loop {
                let mut rcv_buf = FifoBuffer::<192>::new();
                match receiver.read_packet(rcv_buf.receive_buffer()).await {
                    Ok(n) => {
                        rcv_buf.advance(n);
                        if !rcv_buf.is_empty() {
                            let mut wrap = SliceInputBuffer::new(rcv_buf.data());
                            ctx.shared.app_state.lock(|state| {
                                crate::KLIPPER_TRANSPORT.receive(&mut wrap, state);
                            });
                            rcv_buf.pop(rcv_buf.len() - wrap.available());
                        }
                    }
                    Err(_) => {
                        defmt::warn!("USB disconnected");
                        break;
                    }
                }
            }
        }
    }

    /// Drains the bbqueue TX buffer and sends packets over USB.
    #[task(priority = 2, local = [usb_sender, usb_tx_consumer])]
    async fn usb_send(ctx: usb_send::Context) {
        let sender = ctx.local.usb_sender;
        loop {
            sender.wait_connection().await;
            let mut more = false;
            loop {
                if !more {
                    USB_TX_SIGNAL.wait().await;
                }
                let grant = match ctx.local.usb_tx_consumer.read() {
                    Ok(grant) => grant,
                    Err(_) => {
                        more = false;
                        continue;
                    }
                };
                let len = grant.buf().len();
                let take = len.min(64);

                match sender.write_packet(&grant.buf()[..take]).await {
                    Ok(_) => {
                        grant.release(take);
                        more = len != take;
                    }
                    Err(_) => {
                        defmt::warn!("USB send error");
                        break;
                    }
                }
            }
        }
    }
}

use anchor::klipper_config_generate;
use anchor::klipper_enumeration;

// Pin enumerations at crate root (outside mod app) so anchor_codegen
// can discover them. TMC4671 handles all pins internally — these exist
// only for Klipper config pin name resolution.
klipper_enumeration!(
    #[allow(non_camel_case_types)]
    #[klipper_enumeration(name = "pin")]
    enum Pins {
        PA0,
        PA1,
        PA2,
        PA3,
        PA4,
        PA5,
        PA6,
        PA7,
        PA8,
        PA9,
        PA10,
        PA11,
        PA12,
        PA13,
        PA14,
        PA15,
        PB0,
        PB1,
        PB2,
        PB3,
        PB4,
        PB5,
        PB6,
        PB7,
        PB8,
        PB9,
        PB10,
        PB11,
        PB12,
        PB13,
        PB14,
        PB15,
        PC0,
        PC1,
        PC2,
        PC3,
        PC4,
        PC5,
        PC6,
        PC7,
        PC8,
        PC9,
        PC10,
        PC11,
        PC12,
        PC13,
        PC14,
        PC15,
        PD0,
        PD1,
        PD2,
        PD3,
        PD4,
        PD5,
        PD6,
        PD7,
        PD8,
        PD9,
        PD10,
        PD11,
        PD12,
        PD13,
        PD14,
        PD15,
        PE0,
        PE1,
        PE2,
        PE3,
        PE4,
        PE5,
        PE6,
        PE7,
        PE8,
        PE9,
        PE10,
        PE11,
        PE12,
        PE13,
        PE14,
        PE15,
    }
);

klipper_config_generate!(
    transport = crate::app::TRANSPORT_OUTPUT: crate::app::BufferTransportOutput,
    context = &'ctx mut klipper_mcu::state::State,
);
