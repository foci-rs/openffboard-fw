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

    use embassy_stm32::gpio::{Level, Output, Speed};
    use embassy_stm32::usb::Driver;
    use embassy_stm32::{bind_interrupts, peripherals, usb};
    use embassy_usb::class::cdc_acm::{CdcAcmClass, State as CdcState};
    use embassy_usb::driver::EndpointError;
    use embassy_usb::{Builder, Config as UsbConfig, UsbDevice};
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

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        led: Output<'static>,
        usb: UsbDevice<'static, HalUsbDriver>,
        usb_sender: embassy_usb::class::cdc_acm::Sender<'static, HalUsbDriver>,
        usb_receiver: embassy_usb::class::cdc_acm::Receiver<'static, HalUsbDriver>,
    }

    #[init(local = [
        usb_buf: UsbBuffers = UsbBuffers::new(),
        ep_out_buf: [u8; 256] = [0; 256],
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
        blink::spawn().ok();

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
        usb_config.serial_number = Some("TODO_UID");
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

        usb_pump::spawn().ok();
        usb_echo::spawn().ok();

        defmt::info!("FOCI OpenFFBoard initialized");

        (
            Shared {},
            Local {
                led,
                usb,
                usb_sender,
                usb_receiver,
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

    /// Echo task: loops received USB CDC-ACM bytes back to the host.
    ///
    /// This is a temporary placeholder; Task 4 will replace it with the
    /// anchor protocol handler.
    #[task(priority = 2, local = [usb_receiver, usb_sender])]
    async fn usb_echo(ctx: usb_echo::Context) {
        let receiver = ctx.local.usb_receiver;
        let sender = ctx.local.usb_sender;
        loop {
            receiver.wait_connection().await;
            defmt::info!("USB connected");
            loop {
                let mut buf = [0u8; 64];
                match receiver.read_packet(&mut buf).await {
                    Ok(n) => {
                        if let Err(EndpointError::Disabled) = sender.write_packet(&buf[..n]).await {
                            defmt::warn!("USB send error: endpoint disabled");
                            break;
                        }
                    }
                    Err(EndpointError::Disabled) => {
                        defmt::warn!("USB disconnected");
                        break;
                    }
                    Err(EndpointError::BufferOverflow) => {
                        defmt::warn!("USB RX buffer overflow");
                        break;
                    }
                }
            }
        }
    }
}
