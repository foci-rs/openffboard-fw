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
    use core::mem::MaybeUninit;
    use core::ptr::addr_of_mut;

    use anchor::*;
    use bbqueue::BBBuffer;
    use cortex_m::peripheral::DWT;
    use embassy_stm32::gpio::{Level, Output, Speed};
    use embassy_stm32::mode::Async as AsyncMode;
    use embassy_stm32::spi::Spi;
    use embassy_stm32::usb::Driver;
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
    }

    const TMC_CHANNEL_SIZE: usize = 4;

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

    static mut USB_TX_PRODUCER: MaybeUninit<bbqueue::Producer<'static, USB_TX_BUFFER_SIZE>> =
        MaybeUninit::uninit();
    static USB_TX_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

    static mut TMC_REQUEST_SENDER: MaybeUninit<Sender<'static, TmcRequest, TMC_CHANNEL_SIZE>> =
        MaybeUninit::uninit();

    pub struct BufferTransportOutput;
    pub const TRANSPORT_OUTPUT: BufferTransportOutput = BufferTransportOutput;

    impl TransportOutput for BufferTransportOutput {
        type Output = ScratchOutput;
        fn output(&self, f: impl FnOnce(&mut Self::Output)) {
            let mut scratch = ScratchOutput::new();
            f(&mut scratch);
            let output = scratch.result();

            // SAFETY: USB_TX_PRODUCER is initialised exactly once in init() before
            // any task runs. After init, only this function accesses the producer.
            let producer = unsafe {
                &mut *addr_of_mut!(USB_TX_PRODUCER)
                    .cast::<bbqueue::Producer<'static, USB_TX_BUFFER_SIZE>>()
            };
            if let Ok(mut grant) = producer.grant_exact(output.len()) {
                grant.buf().copy_from_slice(output);
                grant.commit(output.len());
                USB_TX_SIGNAL.signal(());
            }
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

    #[klipper_command]
    pub fn get_clock(_context: &mut State) {
        let clock = DWT::cycle_count();
        klipper_reply!(clock, clock: u32 = clock);
    }

    #[klipper_command]
    pub fn get_uptime(_context: &mut State) {
        let c = DWT::cycle_count() as u64;
        klipper_reply!(
            uptime,
            high: u32 = (c >> 32) as u32,
            clock: u32 = (c & 0xFFFF_FFFF) as u32
        );
    }

    #[klipper_command]
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

    #[klipper_command]
    pub fn config_reset(context: &mut State) {
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

    #[klipper_command]
    pub fn emergency_stop(context: &mut State) {
        context.mcu_state = McuState::Shutdown { reason: 0 };
        defmt::error!("Emergency stop!");
    }

    #[klipper_command]
    pub fn reset() {
        request_katapult_bootloader();
    }

    #[klipper_command]
    pub fn debug_nop() {}

    /// Request reboot into Katapult bootloader.
    ///
    /// Writes the Katapult magic signature to the RAM address pointed to by
    /// the bootloader's initial stack pointer (first word of the vector table
    /// at 0x08000000). Katapult checks this on boot and stays in bootloader
    /// mode instead of jumping to the application.
    fn request_katapult_bootloader() {
        const KATAPULT_BOOT_ADDRESS: u32 = 0x0800_0000;
        const KATAPULT_MAGIC: u64 = 0x5984_E3FA_6CA1_589B;

        cortex_m::interrupt::disable();

        // SAFETY: The bootloader vector table at KATAPULT_BOOT_ADDRESS is
        // always readable. Its first entry is the initial SP value, which
        // points to a valid RAM address. We write the magic signature there
        // so Katapult detects a bootloader request on the next reset.
        unsafe {
            let bl_vectors = KATAPULT_BOOT_ADDRESS as *const u32;
            let initial_sp = core::ptr::read_volatile(bl_vectors);
            let req_sig = initial_sp as *mut u64;
            core::ptr::write_volatile(req_sig, KATAPULT_MAGIC);
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
        if let Some(&idx) = context.enable_by_oid.get(&oid) {
            let enabled = on_ticks != 0;
            context.steppers[idx].set_enabled(enabled);
            defmt::info!(
                "queue_digital_out: oid={} clock={} enabled={}",
                oid,
                clock,
                enabled
            );
        }
    }

    /// Immediately update a digital output value.
    #[klipper_command]
    pub fn update_digital_out(context: &mut State, oid: u8, value: u8) {
        if let Some(&idx) = context.enable_by_oid.get(&oid) {
            context.steppers[idx].set_enabled(value != 0);
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

    /// Write a TMC4671 register via SPI.
    ///
    /// Queues the write request for the tmc_control task which owns
    /// the SPI bus.
    #[klipper_command]
    pub fn tmc_write_register(_context: &mut State, _oid: u8, addr: u8, value: u32) {
        // SAFETY: TMC_REQUEST_SENDER is initialised exactly once in init()
        // before any task runs. After init, only command handlers access it,
        // and they run sequentially within usb_receive.
        let sender = unsafe {
            &mut *core::ptr::addr_of_mut!(TMC_REQUEST_SENDER).cast::<Sender<
                'static,
                TmcRequest,
                TMC_CHANNEL_SIZE,
            >>()
        };
        let _ = sender.try_send(TmcRequest::Write { addr, value });
    }

    /// Read a TMC4671 register via SPI.
    ///
    /// Queues the read request for the tmc_control task. The response
    /// is sent asynchronously as a tmc_register_value reply.
    #[klipper_command]
    pub fn tmc_read_register(_context: &mut State, oid: u8, addr: u8) {
        let sender = unsafe {
            &mut *core::ptr::addr_of_mut!(TMC_REQUEST_SENDER).cast::<Sender<
                'static,
                TmcRequest,
                TMC_CHANNEL_SIZE,
            >>()
        };
        let _ = sender.try_send(TmcRequest::Read { oid, addr });
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
        tmc_rx: Receiver<'static, TmcRequest, TMC_CHANNEL_SIZE>,
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
        // SAFETY: init runs exactly once before any task is spawned.
        unsafe {
            addr_of_mut!(TMC_REQUEST_SENDER).write(MaybeUninit::new(tmc_tx));
        }

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
        // SAFETY: init runs exactly once before any task is spawned, so no concurrent access.
        unsafe {
            addr_of_mut!(USB_TX_PRODUCER).write(MaybeUninit::new(usb_tx_prod));
        }

        // Spawn tasks
        tmc_control::spawn().ok();
        usb_pump::spawn().ok();
        usb_receive::spawn().ok();
        usb_send::spawn().ok();
        usb_control::spawn().ok();
        blink::spawn().ok();

        defmt::info!("FOCI OpenFFBoard initialized");

        (
            Shared {
                app_state: State::new(),
            },
            Local {
                led,
                tmc,
                tmc_rx,
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
    #[task(priority = 1, local = [tmc, tmc_rx])]
    async fn tmc_control(ctx: tmc_control::Context) {
        let tmc = ctx.local.tmc;
        let rx = ctx.local.tmc_rx;

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

        // Process register read/write requests from the host
        loop {
            if let Ok(req) = rx.recv().await {
                match req {
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
                }
            }
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
                request_katapult_bootloader();
            }
            *ctx.local.dtr_was_set = dtr;
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
