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
    use embassy_stm32::usb::Driver;
    use embassy_stm32::{bind_interrupts, peripherals, usb};
    use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
    use embassy_sync::signal::Signal;
    use embassy_usb::class::cdc_acm::{CdcAcmClass, State as CdcState};
    use embassy_usb::{Builder, Config as UsbConfig, UsbDevice};
    use klipper_mcu::shutdown::McuState;
    use klipper_mcu::state::State;
    use rtic_monotonics::systick::prelude::*;

    bind_interrupts!(struct Irqs {
        OTG_FS => usb::InterruptHandler<peripherals::USB_OTG_FS>;
    });

    systick_monotonic!(Mono, 1000);

    /// USB HAL driver type alias for use throughout this module.
    type HalUsbDriver = Driver<'static, peripherals::USB_OTG_FS>;

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
        cortex_m::peripheral::SCB::sys_reset();
    }

    #[klipper_command]
    pub fn debug_nop() {}

    // ---- RTIC resources ----

    #[shared]
    struct Shared {
        app_state: State,
    }

    #[local]
    struct Local {
        led: Output<'static>,
        usb: UsbDevice<'static, HalUsbDriver>,
        usb_sender: embassy_usb::class::cdc_acm::Sender<'static, HalUsbDriver>,
        usb_receiver: embassy_usb::class::cdc_acm::Receiver<'static, HalUsbDriver>,
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
        let (usb_sender, usb_receiver) = class.split();

        // Split bbqueue for TX output
        let (usb_tx_prod, usb_tx_consumer) = ctx.local.usb_tx_queue.try_split().unwrap();
        // SAFETY: init runs exactly once before any task is spawned, so no concurrent access.
        unsafe {
            addr_of_mut!(USB_TX_PRODUCER).write(MaybeUninit::new(usb_tx_prod));
        }

        // Spawn tasks
        usb_pump::spawn().ok();
        usb_receive::spawn().ok();
        usb_send::spawn().ok();
        blink::spawn().ok();

        defmt::info!("FOCI OpenFFBoard initialized");

        (
            Shared {
                app_state: State::new(),
            },
            Local {
                led,
                usb,
                usb_sender,
                usb_receiver,
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
klipper_config_generate!(
    transport = crate::app::TRANSPORT_OUTPUT: crate::app::BufferTransportOutput,
    context = &'ctx mut klipper_mcu::state::State,
);
