#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

// PAC shim for RTIC + embassy-stm32
pub mod pac {
    pub use embassy_stm32::Peripherals;
    pub use embassy_stm32::pac::Interrupt as interrupt;
    pub use embassy_stm32::pac::*;
}

mod ankyra;
mod app_types;
mod board_init;
mod firmware_ctx;
mod heartbeat;
mod helpers;
mod interrupts;
mod platform_impl;
mod state;
mod transport;
mod trigger_domain;
mod usb_task;

#[rtic::app(device = crate::pac, peripherals = true, dispatchers = [UART4, UART5, USART3])]
mod app {
    use crate::ankyra::constants::CLOCK_FREQ;
    use embassy_stm32::gpio::{Level, Output, Speed};
    use embassy_stm32::spi::Spi;
    use embassy_stm32::wdg::IndependentWatchdog;
    use embassy_stm32::{bind_interrupts, peripherals};
    use embedded_hal_bus::spi::ExclusiveDevice;
    use foci_klipper::state::State;
    use foci_tmc4671::spi::Tmc4671Spi;
    use rtic_monotonics::stm32::prelude::*;
    use rtic_sync::channel::Receiver;

    use crate::app_types::{CycleDelay, TmcSpiDevice};
    use crate::helpers::lifecycle::*;
    use crate::helpers::stepper::*;
    use crate::helpers::trsync::*;
    use crate::state::stepper::*;
    use crate::state::tmc::*;
    use crate::state::trace::*;
    use crate::trigger_domain::{TRIGGER_DOMAIN, TriggerDomain, with_trigger_domain};
    use foci_firmware::state::lifecycle::*;
    use foci_firmware::state::stepper::*;
    use foci_firmware::state::tmc::*;
    use foci_firmware::state::trsync::*;

    bind_interrupts!(struct Irqs {
        DMA2_STREAM2 => embassy_stm32::dma::InterruptHandler<peripherals::DMA2_CH2>;
        DMA2_STREAM3 => embassy_stm32::dma::InterruptHandler<peripherals::DMA2_CH3>;
    });

    // TIM2-backed monotonic at 1 MHz (1 us resolution).
    //
    // Previously used SysTick at 1 kHz, but the commissioning engine's
    // tight sample loops in Phase 5 (R/L measurement) rely on sub-ms
    // deadlines. Using a TIM2-backed monotonic gives true u64 us
    // precision without the 51-second wraparound bug that affected
    // the old read_tim5()/84 path.
    //
    // The rtic-monotonics crate handles the hardware timer overflow
    // automatically via an extended u64 counter in a TimerQueueBackend.
    stm32_tim2_monotonic!(Mono, 1_000_000);

    use foci_firmware::tmc::TmcRequest;

    /// SPI budget for fast trace preset (us). DWT measured ~227us on hardware.
    const TRACE_SPI_BUDGET_FAST_US: u64 = 300;
    /// SPI budget for full trace preset (us). Estimated ~450us from fast ratio.
    const TRACE_SPI_BUDGET_FULL_US: u64 = 550;

    /// Current scaling for OpenFFBoard: milliamps per ADC least significant bit.
    /// Derived from sense resistor value and ADC configuration.
    const CURRENT_SCALE_MA_PER_LSB: f32 = 1.155;

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
    const TMC_CONFIG: foci_firmware::config::TmcConfig = foci_firmware::config::TmcConfig {
        pwm_maxcnt: 3999,
        tmc_clock_hz: 25_000_000,
        bbm_h: 10,
        bbm_l: 10,
        dsadc_mdec: 331,
        excitation_voltage: 512,
        didt_cycles: 5000,
        bandwidth_hz: 150.0,
    };

    /// CommissioningBackend implementation for OpenFFBoard.
    ///
    /// Wraps the TMC4671 SPI driver, enable pin, and timer so the
    /// board-agnostic [`foci_firmware::commission::CommissioningEngine`] can
    /// perform register I/O, pin control, and timing.
    struct OpenFFBoardBackend<'a> {
        tmc: &'a mut Tmc4671Spi<TmcSpiDevice>,
        enable_pin: &'a mut Output<'static>,
    }

    impl<'a> foci_firmware::commission::CommissioningBackend for OpenFFBoardBackend<'a> {
        type Error = foci_tmc4671::spi::SpiError<
            embedded_hal_bus::spi::DeviceError<
                embassy_stm32::spi::Error,
                core::convert::Infallible,
            >,
        >;

        async fn read_register(
            &mut self,
            reg: foci_tmc4671::registers::Register,
        ) -> Result<u32, Self::Error> {
            self.tmc.read_register(reg).await
        }

        async fn write_register(
            &mut self,
            reg: foci_tmc4671::registers::Register,
            val: u32,
        ) -> Result<(), Self::Error> {
            self.tmc.write_register(reg, val).await
        }

        fn set_enable_pin(&mut self, state: bool) {
            if state {
                self.enable_pin.set_high();
            } else {
                self.enable_pin.set_low();
            }
        }

        async fn read_encoder_count(&mut self) -> u32 {
            self.tmc
                .read_register(foci_tmc4671::registers::ABN_DECODER_COUNT)
                .await
                .unwrap_or(0)
        }

        fn elapsed_us(&self) -> u64 {
            // Microseconds since Mono start. Returned as u64 so deadline
            // comparisons in commission.rs never wrap across the 51-second
            // TIM5 boundary that previously broke the engine.
            //
            // Source: TIM2 monotonic at 1 MHz (1 us resolution) via
            // rtic-monotonics. See systick_monotonic! replacement at the
            // top of this module.
            Mono::now().duration_since_epoch().to_micros()
        }

        fn is_shutdown_requested(&self) -> bool {
            if TMC_DISABLE_SIGNAL.try_take().is_some() {
                TMC_DISABLE_SIGNAL.signal(());
                true
            } else {
                false
            }
        }

        fn current_limit(&self) -> u32 {
            CURRENT_LIMIT_REG.load(core::sync::atomic::Ordering::Relaxed)
        }

        fn pwm_maxcnt(&self) -> u16 {
            TMC_CONFIG.pwm_maxcnt
        }

        fn bbm_register(&self) -> u32 {
            TMC_CONFIG.bbm_register()
        }

        fn dsadc_mdec_register(&self) -> u32 {
            TMC_CONFIG.dsadc_mdec_register()
        }

        fn excitation_voltage(&self) -> u16 {
            TMC_CONFIG.excitation_voltage
        }

        fn tmc_config(&self) -> &foci_firmware::config::TmcConfig {
            &TMC_CONFIG
        }

        fn step_width(&self) -> u32 {
            STEP_WIDTH_VALUE
                .load(core::sync::atomic::Ordering::Relaxed)
                .max(0) as u32
        }

        fn operational_velocity_limit(&self) -> u32 {
            VELOCITY_LIMIT_CFG.load(core::sync::atomic::Ordering::Relaxed)
        }
    }

    /// Acceptable residual following error after a completed move, in ABN
    /// encoder counts. Within this window the controller is fighting encoder
    /// quantization and belt/motor compliance more than useful position error.
    const HOLD_SETTLE_DEADBAND_COUNTS: u32 = 4;
    const HOLD_SETTLE_DELAY_MS: u8 = 100;

    // TriggerDomain, PHYSICAL_STEPPER_INDICES, TRIGGER_TIMER_MIN_LEAD_TICKS,
    // TRIGGER_DOMAIN, with_trigger_domain(), and stop_physical_stepper_no_lock()
    // live in crate::trigger_domain and are re-imported below.

    /// Get a mutable reference to the trace buffer.
    ///
    /// # Safety
    ///
    /// Must only be called from tmc_control (P1) after init has completed.
    #[inline]
    unsafe fn trace_buf() -> &'static mut foci_firmware::trace::TraceBuffer {
        // Use addr_of_mut! to avoid creating a reference to the mutable static
        // (Rust 2024 edition compatibility). MaybeUninit<T> has the same layout as T.
        // SAFETY: caller upholds the P1-only + post-init contract of `trace_buf`.
        unsafe {
            &mut *core::ptr::addr_of_mut!(TRACE_BUF).cast::<foci_firmware::trace::TraceBuffer>()
        }
    }

    // ---- Klipper constants ----
    // Declared in `crate::ankyra::constants` and registered into the data
    // dictionary via the `constants: [...]` list in `crate::ankyra::provider`.

    // ---- TMC4671 register access commands ----
    // Ported to `crate::ankyra::handlers::tmc` by Task 8k.
    // FOCI commissioning/tune/trace commands ported to
    // `crate::ankyra::handlers::foci` by Task 8l.

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
    }

    #[init(local = [
        tmc_channel: rtic_sync::channel::Channel<TmcRequest, TMC_CHANNEL_SIZE> =
            rtic_sync::channel::Channel::new(),
        uid_str: [u8; 24] = [0; 24],
    ])]
    fn init(ctx: init::Context) -> (Shared, Local) {
        defmt::info!("FOCI OpenFFBoard starting");

        let p = embassy_stm32::init(crate::board_init::rcc_config());
        defmt::info!("SYSCLK: 168 MHz, USB: 48 MHz");

        crate::board_init::init_tim5_clock();
        crate::board_init::init_stepdir_gpio();
        defmt::info!("TIM5 enabled for Klipper clock @ 84 MHz, STEP/DIR GPIO configured");

        // TIM2 clock: 84 MHz (APB1 timer clock, same bus as TIM5). At the
        // 1 MHz target monotonic rate, the prescaler is 84 (integer divisor).
        // rtic-monotonics enables and configures TIM2 internally; no PAC
        // setup is required beyond passing the timer clock frequency.
        Mono::start(84_000_000);

        // PD7: system status LED on OpenFFBoard
        let led = Output::new(p.PD7, Level::Low, Speed::Low);

        // PE7: TMC4671 ENABLE pin (active high, start disabled)
        let tmc_enable = Output::new(p.PE7, Level::Low, Speed::Low);

        crate::board_init::init_brake_pin(p.PE10);
        crate::board_init::init_tmc_status_pin(p.PE8);

        // IWDG: 10-second timeout, kicked by the stats task every 5 seconds.
        let mut wdg = IndependentWatchdog::new(p.IWDG, 10_000_000);
        wdg.unleash();
        defmt::info!("IWDG started: 10s timeout");

        // SPI1: TMC4671 communication at 8 MHz, Mode 3 (CPOL=1, CPHA=1)
        let mut spi_config = embassy_stm32::spi::Config::default();
        spi_config.mode = embassy_stm32::spi::Mode {
            polarity: embassy_stm32::spi::Polarity::IdleHigh,
            phase: embassy_stm32::spi::Phase::CaptureOnSecondTransition,
        };
        spi_config.frequency = embassy_stm32::time::Hertz(8_000_000);

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
        let spi_dev = ExclusiveDevice::new(spi1, spi_cs, CycleDelay)
            .expect("ExclusiveDevice::new is infallible");
        let tmc = Tmc4671Spi::new(spi_dev);

        // TMC register request channel
        let (tmc_tx, tmc_rx) = ctx.local.tmc_channel.split();
        critical_section::with(|cs| {
            TMC_REQUEST_SENDER.borrow(cs).replace(Some(tmc_tx));
        });

        defmt::info!("SPI1 configured: 8 MHz, Mode 3, DMA");

        crate::board_init::init_usb_pins(p.PA11, p.PA12);
        crate::board_init::enable_usb_clock();

        let uid = crate::board_init::read_stm32_uid(ctx.local.uid_str);

        // SAFETY: Called exactly once, before any other klipper-usb function.
        // base_addr is the USB_OTG_FS peripheral base on STM32F407.
        unsafe {
            foci_usb_stm32::usb_init(&foci_usb_stm32::UsbConfig {
                base_addr: 0x5000_0000,
                irq_num: 67,
                trdt: 6,
                vbus_detection: false, // F407 uses NOVBUSSENS, not GOTGCTL BVAL
                double_buffer_tx: true,
                vid: 0x1D50,
                pid: 0x614E,
                manufacturer: "foci",
                product: "FOCI OpenFFBoard",
                serial: uid,
            });
        }

        crate::board_init::enable_dwt_cyccnt();

        // Initialize trace buffer in CCM RAM
        unsafe {
            // Use addr_of_mut! to avoid creating a reference to the mutable static
            let ptr =
                core::ptr::addr_of_mut!(TRACE_BUF).cast::<foci_firmware::trace::TraceBuffer>();
            // Validate address is in CCM range
            let addr = ptr as usize;
            defmt::assert!(
                (0x1000_0000..0x1001_0000).contains(&addr),
                "TRACE_BUF not in CCM RAM: {:#010x}",
                addr
            );
            defmt::info!(
                "trace: buffer at {:#010x}, size {} bytes",
                addr,
                core::mem::size_of::<foci_firmware::trace::TraceBuffer>()
            );
            core::ptr::write(ptr, {
                let mut buf = core::mem::zeroed::<foci_firmware::trace::TraceBuffer>();
                // reset() sets magic and all fields to valid initial state
                buf.reset();
                buf
            });
        }

        // Initialize TriggerDomain
        // SAFETY: init runs once before any task starts. No concurrent access.
        unsafe {
            core::ptr::addr_of_mut!(TRIGGER_DOMAIN)
                .cast::<TriggerDomain>()
                .write(TriggerDomain::new());
        }

        // Spawn tasks
        tmc_control::spawn().ok();
        usb_task::spawn().ok();
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
            },
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            rtic::export::wfi();
        }
    }

    /// Handle a TMC request that does not require main-loop state (motor_enabled, enable_pin).
    ///
    /// SetEnable, Init, and TestSpin are handled inline by the tmc_control main loop.
    /// Returns `Ok(())` on success, `Err(())` if the caller should enter shutdown.
    async fn handle_tmc_request(
        req: TmcRequest,
        tmc: &mut Tmc4671Spi<TmcSpiDevice>,
    ) -> Result<(), ()> {
        match req {
            #[cfg(feature = "dev")]
            TmcRequest::Write { addr, value } => {
                let reg = foci_tmc4671::registers::Register::new(addr);
                if tmc.write_register(reg, value).await.is_err() {
                    defmt::error!("TMC4671 SPI write failed: addr={:#04x}", addr);
                }
            }
            #[cfg(feature = "dev")]
            TmcRequest::Read { oid, addr } => {
                let reg = foci_tmc4671::registers::Register::new(addr);
                match tmc.read_register(reg).await {
                    Ok(value) => {
                        ::ankyra::klipper_reply_from!(
                            &mut crate::_ankyra_config::Sender,
                            foci_firmware::ankyra::replies::TmcRegisterValue,
                            oid: u8 = oid,
                            addr: u8 = addr,
                            value: u32 = value,
                        );
                    }
                    Err(_) => {
                        defmt::error!("TMC4671 SPI read failed: addr={:#04x}", addr);
                    }
                }
            }
            TmcRequest::SetCurrent { run_ma } => {
                use foci_tmc4671::registers::*;
                // User configures RMS current (Klipper convention). The TMC4671
                // PID limits operate on peak amplitude in the d-q frame
                // (datasheet eq. 19/21: I_phase = I_D * sin(phi_e)).
                // Convert: peak = RMS * sqrt(2).
                let peak_ma = run_ma as f32 * core::f32::consts::SQRT_2;
                let counts = (peak_ma / CURRENT_SCALE_MA_PER_LSB) as u32;
                let limit = (counts << 16) | counts;
                if tmc
                    .write_register(PID_TORQUE_FLUX_LIMITS, limit)
                    .await
                    .is_err()
                {
                    defmt::error!("TMC4671 SPI write failed: PID_TORQUE_FLUX_LIMITS");
                    return Err(());
                }
                CURRENT_LIMIT_REG.store(limit, core::sync::atomic::Ordering::Relaxed);
                defmt::info!(
                    "TMC4671 current: {}mA RMS -> {} counts (peak)",
                    run_ma,
                    counts
                );
            }
            TmcRequest::SetEncoder { ppr } => {
                use foci_tmc4671::registers::*;
                if ppr == 0 {
                    defmt::error!("Invalid encoder PPR: 0");
                    return Err(());
                }
                let quadrature_ppr = ppr.saturating_mul(4);
                if tmc
                    .write_register(ABN_DECODER_PPR, quadrature_ppr)
                    .await
                    .is_err()
                {
                    defmt::error!("TMC4671 SPI write failed: ABN_DECODER_PPR");
                    return Err(());
                }
                let step_width = 65536u64 * 50 / quadrature_ppr as u64;
                defmt::info!("TMC4671 encoder: {} PPR, step_width={}", ppr, step_width);
            }
            TmcRequest::SetEncoderDir { invert } => {
                use foci_tmc4671::registers::*;
                let mode = if invert { 1u32 << 12 } else { 0 };
                if tmc.write_register(ABN_DECODER_MODE, mode).await.is_err() {
                    defmt::error!("TMC4671 SPI write failed: ABN_DECODER_MODE");
                    return Err(());
                }
                ENCODER_REVERSED.store(invert, core::sync::atomic::Ordering::Relaxed);
                // Signal the tick loop to reset feedforward state: prev_position_target
                // has the wrong sign after a direction flip, which would produce a
                // one-tick velocity impulse in the feedforward delta computation.
                FEEDFORWARD_RESET.store(true, core::sync::atomic::Ordering::Relaxed);
                defmt::info!(
                    "TMC4671 encoder direction: {}",
                    if invert { "reversed" } else { "default" }
                );
            }
            TmcRequest::SelfTest { .. } => {
                defmt::warn!("handle_tmc_request: SelfTest handled by main loop");
            }
            TmcRequest::DumpRegisters { oid } => {
                for &addr in foci_firmware::dump::DUMP_REGISTERS {
                    // Yield between SPI reads to let USB task drain TX buffer
                    Mono::delay(1.millis()).await;

                    let reg = foci_tmc4671::registers::Register::new(addr);
                    let value = match tmc.read_register(reg).await {
                        Ok(v) => v,
                        Err(_) => {
                            defmt::error!("TMC4671 SPI read failed: addr={:#04x}", addr);
                            0
                        }
                    };
                    ::ankyra::klipper_reply_from!(
                        &mut crate::_ankyra_config::Sender,
                        foci_firmware::ankyra::replies::FociDumpValue,
                        oid: u8 = oid,
                        addr: u8 = addr,
                        value: u32 = value,
                    );
                }
                // Sub-registers (INTERIM, PID_ERROR) use synthetic addrs 0x80+
                for &(synthetic_addr, sub_reg) in foci_firmware::dump::DUMP_SUB_REGISTERS {
                    Mono::delay(1.millis()).await;

                    let value = match tmc.read_sub_register(sub_reg).await {
                        Ok(v) => v,
                        Err(_) => {
                            defmt::error!(
                                "TMC4671 sub-register read failed: addr={:#04x}",
                                synthetic_addr
                            );
                            0
                        }
                    };
                    ::ankyra::klipper_reply_from!(
                        &mut crate::_ankyra_config::Sender,
                        foci_firmware::ankyra::replies::FociDumpValue,
                        oid: u8 = oid,
                        addr: u8 = synthetic_addr,
                        value: u32 = value,
                    );
                }
                Mono::delay(1.millis()).await;
                ::ankyra::klipper_reply_from!(
                    &mut crate::_ankyra_config::Sender,
                    foci_firmware::ankyra::replies::FociDumpDone,
                    oid: u8 = oid,
                );
            }
            TmcRequest::SetPidGains { .. } => {
                defmt::warn!("handle_tmc_request: SetPidGains handled by main loop");
            }
            TmcRequest::SetEnable(_) => {
                defmt::warn!("handle_tmc_request: SetEnable handled by main loop");
            }
            TmcRequest::Calibrate { .. } => {
                defmt::warn!("handle_tmc_request: Calibrate handled by main loop");
            }
            TmcRequest::SetVelocityFilter { .. } => {}
            TmcRequest::SetTorqueFilter { .. } => {}
            TmcRequest::SetPositionFilter { .. } => {}
            TmcRequest::SetFluxFilter { .. } => {}
            TmcRequest::SetPositionGains { .. } => {
                defmt::warn!("handle_tmc_request: SetPositionGains handled by main loop");
            }
            TmcRequest::SetVelocityFeedforward { .. } => {}
            // During active commissioning, trace requests queue until the
            // engine reaches a terminal state (Continue/PhaseChanged both
            // `continue` before rx.recv()). By that point stop() has run,
            // so FetchResult::Busy is protocol-correct but not observable
            // during commissioning. The Python FOCI_TRACE command runs
            // after commissioning completes, so this is not a functional gap.
            TmcRequest::TraceInfo { oid } => {
                let info = unsafe { trace_buf() }.info(oid);
                ::ankyra::klipper_reply_from!(
                    &mut crate::_ankyra_config::Sender,
                    foci_firmware::ankyra::replies::FociTraceInfoResult,
                    oid: u8 = oid,
                    state: u8 = info.state,
                    generation: u8 = info.generation,
                    preset: u8 = info.preset,
                    count: u16 = info.count,
                    phase: u8 = info.phase_id,
                    sample_period_us: u16 = info.sample_period_us,
                    dropped: u16 = info.dropped,
                );
            }
            TmcRequest::TraceFetch {
                oid,
                offset,
                generation,
            } => {
                let buf = unsafe { trace_buf() };
                match buf.fetch(oid, offset, generation) {
                    foci_firmware::trace::FetchResult::Ok(ring_idx) => {
                        let data = if buf.preset() == foci_firmware::trace::PRESET_FULL {
                            buf.sample_bytes_full(ring_idx)
                        } else {
                            buf.sample_bytes_fast(ring_idx)
                        };
                        ::ankyra::klipper_reply_from!(
                            &mut crate::_ankyra_config::Sender,
                            foci_firmware::ankyra::replies::FociTraceData,
                            oid: u8 = oid,
                            offset: u16 = offset,
                            status: u8 = 0,
                            data: &[u8] = data,
                        );
                    }
                    foci_firmware::trace::FetchResult::Busy => {
                        ::ankyra::klipper_reply_from!(
                            &mut crate::_ankyra_config::Sender,
                            foci_firmware::ankyra::replies::FociTraceData,
                            oid: u8 = oid,
                            offset: u16 = offset,
                            status: u8 = 1,
                            data: &[u8] = &[],
                        );
                    }
                    foci_firmware::trace::FetchResult::Invalid => {
                        ::ankyra::klipper_reply_from!(
                            &mut crate::_ankyra_config::Sender,
                            foci_firmware::ankyra::replies::FociTraceData,
                            oid: u8 = oid,
                            offset: u16 = offset,
                            status: u8 = 2,
                            data: &[u8] = &[],
                        );
                    }
                }
            }
            TmcRequest::Commission { .. } => {
                defmt::warn!("handle_tmc_request: Commission handled by main loop");
            }
            TmcRequest::Tune { .. } => {
                defmt::warn!("handle_tmc_request: Tune handled by main loop");
            }
            #[cfg(feature = "dev")]
            TmcRequest::Init => {
                defmt::warn!("handle_tmc_request: Init handled by main loop");
            }
            #[cfg(feature = "dev")]
            TmcRequest::TestSpin { .. } => {
                defmt::warn!("handle_tmc_request: TestSpin handled by main loop");
            }
        }
        Ok(())
    }

    /// Read trace registers and store a sample. Called from tmc_control tick.
    /// `tick_deadline` is the absolute time of the next tick.
    async fn trace_sample_tick(
        tmc: &mut Tmc4671Spi<TmcSpiDevice>,
        tick_deadline: <Mono as rtic_monotonics::Monotonic>::Instant,
    ) {
        use foci_tmc4671::registers::*;

        let buf = unsafe { trace_buf() };
        if !buf.is_capturing() {
            return;
        }

        let preset = buf.preset();
        let engine_tick = buf.engine_tick();

        // Full preset decimation: skip odd ticks (500 Hz effective)
        if preset == foci_firmware::trace::PRESET_FULL && engine_tick % 2 != 0 {
            buf.tick_no_sample(false); // intentional decimation, not a drop
            return;
        }

        // Pre-read deadline check
        let budget_us = if preset == foci_firmware::trace::PRESET_FULL {
            TRACE_SPI_BUDGET_FULL_US
        } else {
            TRACE_SPI_BUDGET_FAST_US
        };
        let now = Mono::now();
        if tick_deadline
            .checked_duration_since(now)
            .is_none_or(|d| d.to_micros() < budget_us)
        {
            buf.tick_no_sample(true); // budget drop
            return;
        }

        // First sample: measure actual SPI duration via DWT CYCCNT
        let is_first_sample = engine_tick == 0;
        let cyccnt_before = if is_first_sample {
            cortex_m::peripheral::DWT::cycle_count()
        } else {
            0
        };

        // Read fast-preset registers. Any SPI failure discards the whole
        // sample — storing zeros would be indistinguishable from real data.
        macro_rules! read_or_drop {
            ($expr:expr) => {
                match $expr.await {
                    Ok(v) => v,
                    Err(_) => {
                        buf.tick_no_sample(true);
                        return;
                    }
                }
            };
        }

        let position_target = read_or_drop!(tmc.read_register(PID_POSITION_TARGET)) as i32;
        let position_actual = read_or_drop!(tmc.read_register(PID_POSITION_ACTUAL)) as i32;
        let torque_flux_actual = read_or_drop!(tmc.read_register(PID_TORQUE_FLUX_ACTUAL));
        let abn_count = read_or_drop!(tmc.read_register(ABN_DECODER_COUNT)) as i32;
        let status_flags = read_or_drop!(tmc.read_register(STATUS_FLAGS));
        let velocity_actual =
            read_or_drop!(tmc.read_sub_register(INTERIM_PIDOUT_TARGET_VELOCITY)) as i32;

        if preset == foci_firmware::trace::PRESET_FULL {
            // Read additional full-preset registers
            let torque_flux_target = read_or_drop!(tmc.read_register(PID_TORQUE_FLUX_TARGET));
            let velocity_offset = read_or_drop!(tmc.read_register(PID_VELOCITY_OFFSET)) as i32;
            let error_sum_position =
                read_or_drop!(tmc.read_sub_register(PID_ERROR_PID_POSITION_ERROR_SUM)) as i32;
            let error_sum_velocity =
                read_or_drop!(tmc.read_sub_register(PID_ERROR_PID_VELOCITY_ERROR_SUM)) as i32;
            let error_sum_torque =
                read_or_drop!(tmc.read_sub_register(PID_ERROR_PID_TORQUE_ERROR_SUM)) as i32;

            // First sample: log measured SPI duration
            if is_first_sample {
                let elapsed_cycles =
                    cortex_m::peripheral::DWT::cycle_count().wrapping_sub(cyccnt_before);
                // 168 MHz -> cycles / 168 = microseconds
                let elapsed_us = elapsed_cycles / 168;
                defmt::info!(
                    "trace: full SPI read {} us ({} cycles)",
                    elapsed_us,
                    elapsed_cycles
                );
                if elapsed_us as u64 > TRACE_SPI_BUDGET_FULL_US {
                    defmt::warn!(
                        "trace: full SPI read {} us exceeds budget {} us",
                        elapsed_us,
                        TRACE_SPI_BUDGET_FULL_US
                    );
                }
            }

            // Post-read deadline check
            if Mono::now() > tick_deadline {
                buf.tick_no_sample(true);
                defmt::warn!("trace: tick overrun, full sample discarded");
                return;
            }

            buf.push_full(foci_firmware::trace::TraceSampleFull {
                fast: foci_firmware::trace::TraceSampleFast {
                    engine_tick: 0, // set by push
                    phase_id: 0,    // set by push
                    flags: 0,       // set by push
                    position_target,
                    position_actual,
                    torque_flux_actual,
                    velocity_actual,
                    status_flags,
                    abn_count,
                },
                torque_flux_target,
                velocity_offset,
                error_sum_position,
                error_sum_velocity,
                error_sum_torque,
            });
        } else {
            // First sample: log measured SPI duration
            if is_first_sample {
                let elapsed_cycles =
                    cortex_m::peripheral::DWT::cycle_count().wrapping_sub(cyccnt_before);
                let elapsed_us = elapsed_cycles / 168;
                defmt::info!(
                    "trace: fast SPI read {} us ({} cycles)",
                    elapsed_us,
                    elapsed_cycles
                );
                if elapsed_us as u64 > TRACE_SPI_BUDGET_FAST_US {
                    defmt::warn!(
                        "trace: fast SPI read {} us exceeds budget {} us",
                        elapsed_us,
                        TRACE_SPI_BUDGET_FAST_US
                    );
                }
            }

            // Post-read deadline check
            if Mono::now() > tick_deadline {
                buf.tick_no_sample(true);
                defmt::warn!("trace: tick overrun, fast sample discarded");
                return;
            }

            buf.push_fast(foci_firmware::trace::TraceSampleFast {
                engine_tick: 0,
                phase_id: 0,
                flags: 0,
                position_target,
                position_actual,
                torque_flux_actual,
                velocity_actual,
                status_flags,
                abn_count,
            });
        }
    }

    /// TMC4671 control task. Runs at priority 1 (preempted by protocol tasks).
    ///
    /// On startup, reads the TMC4671 chip identification register to verify
    /// SPI communication. Then processes register read/write requests from
    /// the protocol handler via the TMC request channel.
    ///
    /// The 1ms loop also advances the emulated stepper, converting queued
    /// step commands into position target updates. This must run on a timer
    /// rather than only on USB receive — Klipper sends queue_step commands
    /// ahead of time and expects the MCU to process them at the scheduled
    /// clock values, even when no USB traffic is arriving.
    #[task(priority = 1, shared = [app_state], local = [tmc, tmc_enable, tmc_rx])]
    async fn tmc_control(mut ctx: tmc_control::Context) {
        let tmc = ctx.local.tmc;
        let enable_pin = ctx.local.tmc_enable;
        let rx = ctx.local.tmc_rx;

        // NOTE: auto-init disabled - motor current regulation not yet safe
        // tmc_init_open_loop(tmc, enable_pin).await;

        // Verify TMC4671 chip identity — shutdown if SPI fails or wrong chip
        match tmc
            .read_sub_register(foci_tmc4671::registers::CHIPINFO_SI_TYPE)
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
                    ctx.shared.app_state.lock(|state| {
                        if enter_shutdown(state) {
                            ::ankyra::klipper_shutdown_from!(
                                &mut crate::_ankyra_config::Sender,
                                "TMC4671 chip ID mismatch",
                                read_tim5(),
                            );
                        }
                    });
                    return;
                }
            }
            Err(_) => {
                defmt::error!("TMC4671 SPI communication failed");
                ctx.shared.app_state.lock(|state| {
                    if enter_shutdown(state) {
                        ::ankyra::klipper_shutdown_from!(
                            &mut crate::_ankyra_config::Sender,
                            "TMC4671 SPI failed at boot",
                            read_tim5(),
                        );
                    }
                });
                return;
            }
        }

        // Disable TMC4671 brake chopper unconditionally.
        //
        // Write both ADC_VM_LIMIT_HIGH and ADC_VM_LIMIT_LOW to u16::MAX
        // (0xFFFFFFFF). Since ADC_VM_RAW is a 16-bit value that can
        // never exceed 0xFFFF, the comparison `ADC_VM > LIMIT_HIGH` is
        // never true and the BRAKE pin never activates.
        //
        // WARNING: Do NOT write 0/0 to disable the brake, despite the
        // TMC4671 rev 2.08 datasheet section 4.7.12 claim that "BRAKE can
        // be deactivated by setting both registers to Zero". Bench testing
        // on 2026-04-09 proved that writing 0/0 causes the brake to fire
        // continuously — consistent with the comparison `ADC_VM > 0` being
        // always true for any positive bus voltage. The "deactivation"
        // language in the datasheet is either wrong or refers to stopping
        // an already-firing brake (not the default state). The safe
        // "disabled" pattern is 0xFFFFFFFF, not 0.
        //
        // The prior auto-configuration (sample VM at boot, set HIGH at +15%)
        // had two fundamental issues:
        //
        //   1. Threshold too tight: 15% above nominal bus voltage leaves
        //      insufficient margin for regen transients and PSU ripple. At
        //      24V nominal, HIGH was ~26.65V — easily crossed during any
        //      brief motor spin, causing continuous brake firing.
        //   2. Voltage divider saturates at ~43V: the OpenFFBoard v1.2.4
        //      voltage sense can only measure up to ~43V, so boards running
        //      at 36V/48V/65V bus have no usable ADC_VM reading and can't
        //      be protected by this mechanism at all.
        //
        // Proper overvoltage protection requires the Layer 2 brake (STM32
        // ADC monitoring + PE10 GPIO control) with its own voltage sense
        // path that spans the full 12-65V operating range. That is tracked
        // in docs/specs/2026-03-29-foci-design.md (Future Ideas).
        //
        // For now, rely on the PSU's own overvoltage protection and the
        // bulk capacitors to absorb typical regen events. PE10 remains
        // held LOW at boot (line ~1650) to keep the Layer 2 brake FET off.
        {
            use foci_tmc4671::registers::*;
            let _ = tmc.write_register(ADC_VM_LIMITS, 0xFFFF_FFFF).await;
        }

        // Configure STATUS_MASK: route critical fault flags to the STATUS
        // output pin (PE8 EXTI). Only PLL lock loss and ADC current clipping
        // warrant immediate hardware interrupt — other flags are polled.
        {
            use foci_tmc4671::registers::*;
            const STATUS_NOT_PLL_LOCKED: u32 = 1 << 19;
            const STATUS_ADC_I_CLIPPED: u32 = 1 << 26;
            let mask = STATUS_NOT_PLL_LOCKED | STATUS_ADC_I_CLIPPED;
            let _ = tmc.write_register(STATUS_MASK, mask).await;
            // Clear pending STATUS flags from power-on, then configure
            // PE8 EXTI for rising edge interrupt on STATUS assertion.
            let _ = tmc.read_register(STATUS_FLAGS).await;
            let _ = tmc.write_register(STATUS_FLAGS, 0).await;
            let syscfg = embassy_stm32::pac::SYSCFG;
            // EXTICR3[3:0] = 4 selects port E for EXTI line 8.
            syscfg.exticr(2).modify(|w| w.set_exti(0, 4));
            let exti = embassy_stm32::pac::EXTI;
            exti.rtsr(0).modify(|w| w.set_line(8, true));
            exti.ftsr(0).modify(|w| w.set_line(8, false));
            exti.pr(0).write(|w| w.set_line(8, true)); // Clear pending
            exti.imr(0).modify(|w| w.set_line(8, true)); // Unmask
            defmt::info!("TMC4671 STATUS_MASK: {:#010x}, EXTI8 enabled", mask);
        }

        // Track whether the motor is enabled. Only set to true after the full
        // enable sequence (ADC cal, encoder alignment, closed-loop entry) completes.
        let mut motor_enabled = false;

        // Track whether calibration has been completed. Motor enable is
        // rejected unless calibrated. Reset on disable or shutdown.
        let mut calibrated = false;

        // Counter for 1s STATUS_FLAGS poll (1000 * 1ms ticks).
        let mut status_poll_counter: u16 = 0;

        // Stored PID gains from host (explicit or R/L-derived).
        // None = use defaults during calibration.
        let mut stored_gains: Option<foci_firmware::pid::InnerLoopGains> = None;

        // Stored position/velocity PID gains from host.
        // Active commissioning engine. Set when a Commission/Tune
        // request arrives, cleared when the engine finishes or fails.
        let mut commission_engine: Option<foci_firmware::commission::CommissioningEngine> = None;

        // Whether the shutdown safe-state has already been applied this
        // shutdown cycle. Reset when shutdown clears.
        let mut shutdown_handled = false;

        // Velocity feedforward scale factor, computed when encoder PPR arrives.
        // velocity_tmc = velocity_scale / step_interval
        let mut velocity_scale: u32 = 0;

        // Track whether feedforward was active on the previous tick, so we
        // can zero PID_VELOCITY_OFFSET on the deactivation edge.
        let mut feedforward_was_active = false;
        let mut pid_error_clear_countdown_ms: u8 = 0;

        // Main loop: wait for channel message or 1ms timeout.
        // On message: handle request. On timeout: write position target.
        // Position is read from STEPPER_POSITION_TARGET atomic (updated
        // by usb_receive at P2) — no shared resource lock needed.
        loop {
            use embassy_futures::select::{Either, select};

            // On shutdown, perform one-shot motor safe-state then idle until
            // shutdown clears. This catches USB disconnect / host crash where
            // no further commands arrive but the TMC4671 remains energized.
            if SHUTDOWN_ACTIVE.load(core::sync::atomic::Ordering::Relaxed) {
                if !shutdown_handled {
                    commission_engine.take();
                    foci_firmware::selftest::restore_safe_state(tmc, enable_pin).await;
                    motor_enabled = false;
                    calibrated = false;
                    COMMISSION_IN_FLIGHT.store(false, core::sync::atomic::Ordering::Relaxed);
                    stepper_stop_from_p1();
                    MOTOR_ARMED.store(false, core::sync::atomic::Ordering::Relaxed);
                    shutdown_handled = true;
                }
                Mono::delay(1.millis()).await;
                continue;
            }
            shutdown_handled = false;

            // Drain stale requests after config_reset
            if TMC_QUEUE_FLUSH.swap(false, core::sync::atomic::Ordering::Relaxed) {
                while rx.try_recv().is_ok() {}
                motor_enabled = false;
                calibrated = false;
                stored_gains = None;
                // Clear any in-flight commissioning engine and restore safe
                // hardware state (zero targets, disable outputs) so the motor
                // is not left energized in whatever mode the engine last set.
                if commission_engine.take().is_some() {
                    foci_firmware::selftest::restore_safe_state(tmc, enable_pin).await;
                }
                COMMISSION_IN_FLIGHT.store(false, core::sync::atomic::Ordering::Relaxed);
                // Clear commissioned outer configuration so the next closed-
                // loop entry uses defaults, not stale commissioned values.
                use core::sync::atomic::Ordering::Relaxed as R;
                VELOCITY_LIMIT_CFG.store(500_000, R);
                POSITION_P_VALUE.store(640, R);
                POSITION_I_VALUE.store(0, R);
                VELOCITY_P_VALUE.store(1152, R);
                VELOCITY_I_VALUE.store(0, R);
                VELOCITY_FILTER_HZ.store(0, R);
                POSITION_FILTER_HZ.store(0, R);
                FLUX_FILTER_HZ.store(0, R);
                TORQUE_FILTER_HZ.store(0, R);
                // Reset hardware stepper state
                MOTOR_ARMED.store(false, core::sync::atomic::Ordering::Relaxed);
                stepper_stop_from_p1();
                STEPPER_POSITION.store(0, core::sync::atomic::Ordering::Relaxed);
                FAULT_ACTIVE.store(false, core::sync::atomic::Ordering::Relaxed);
                defmt::info!("TMC request queue flushed after config_reset");
            }

            // Tick the commissioning engine if one is active.
            // Each step() does at most 1ms of work, then returns.
            if let Some(engine) = commission_engine.as_mut() {
                // Capture deadline before engine.step() so SPI time consumed
                // by the engine reduces the remaining trace budget.
                let engine_tick_deadline =
                    Mono::now() + fugit::Duration::<u64, 1, 1_000_000>::millis(1);
                let mut backend = OpenFFBoardBackend { tmc, enable_pin };
                let result = engine.step(&mut backend).await;
                match result {
                    foci_firmware::commission::StepResult::Continue => {
                        // Trace capture runs here — the main-loop trace call
                        // is unreachable due to 'continue'.
                        trace_sample_tick(tmc, engine_tick_deadline).await;
                        Mono::delay(1.millis()).await;
                        continue;
                    }
                    foci_firmware::commission::StepResult::PhaseChanged(phase_id) => {
                        let oid = engine.oid();
                        // Start fast trace on ClosedLoopEntry for Stage 1
                        // (Commission only, not Calibrate — spec excludes
                        // Calibrate-on-enable and foci_calibrate CLE paths).
                        if phase_id == foci_firmware::autotune::PhaseId::ClosedLoopEntry
                            && !engine.is_calibrate()
                            && !unsafe { trace_buf() }.is_capturing()
                        {
                            unsafe {
                                trace_buf().start(oid, foci_firmware::trace::PRESET_FAST);
                            }
                        }
                        unsafe { trace_buf().set_phase(phase_id.to_u8()) };
                        ::ankyra::klipper_reply_from!(
                            &mut crate::_ankyra_config::Sender,
                            foci_firmware::ankyra::replies::FociCommissionPhase,
                            oid: u8 = oid,
                            phase: u8 = phase_id.to_u8(),
                            status: u8 = 0,
                        );
                        Mono::delay(1.millis()).await;
                        continue;
                    }
                    foci_firmware::commission::StepResult::InnerFailed(error) => {
                        unsafe { trace_buf().stop() };
                        let oid = engine.oid();
                        let is_calibrate = engine.is_calibrate();
                        let is_selftest = engine.is_selftest();
                        let selftest_results = engine.take_selftest_results();
                        defmt::error!("InnerFailed: calibrate={} error={}", is_calibrate, error,);
                        commission_engine = None;
                        COMMISSION_IN_FLIGHT.store(false, core::sync::atomic::Ordering::Relaxed);
                        foci_firmware::selftest::restore_safe_state(tmc, enable_pin).await;
                        if is_calibrate {
                            // Send raw CommissionError status code — the
                            // old CalibrationError mapping was lossy (4
                            // errors mapped to one code). Python handles
                            // the full CommissionError code range (1-17).
                            ::ankyra::klipper_reply_from!(
                                &mut crate::_ankyra_config::Sender,
                                foci_firmware::ankyra::replies::FociCalibrateResult,
                                oid: u8 = oid,
                                status: u8 = error.status_code(),
                                adc_i0: u16 = 0,
                                adc_i1: u16 = 0,
                                encoder_count: u32 = 0,
                            );
                        } else if is_selftest {
                            // Emit any collected stage results, then the
                            // terminal done with the error code.
                            for (stage, v) in selftest_results.iter() {
                                ::ankyra::klipper_reply_from!(
                                    &mut crate::_ankyra_config::Sender,
                                    foci_firmware::ankyra::replies::FociSelftestResult,
                                    oid: u8 = oid,
                                    stage: u8 = stage,
                                    status: u8 = v.status,
                                    value: u32 = v.value,
                                );
                            }
                            ::ankyra::klipper_reply_from!(
                                &mut crate::_ankyra_config::Sender,
                                foci_firmware::ankyra::replies::FociSelftestDone,
                                oid: u8 = oid,
                                status: u8 = error.status_code(),
                            );
                        } else {
                            ::ankyra::klipper_reply_from!(
                                &mut crate::_ankyra_config::Sender,
                                foci_firmware::ankyra::replies::FociCommissionPhase,
                                oid: u8 = oid,
                                phase: u8 = 0,
                                status: u8 = error.status_code(),
                            );
                        }
                    }
                    foci_firmware::commission::StepResult::OuterDone(outer) => {
                        unsafe { trace_buf().stop() };
                        let oid = engine.oid();
                        commission_engine = None;

                        // Persist outer gains to atomics for closed-loop entry.
                        use core::sync::atomic::Ordering::Relaxed as R;
                        POSITION_P_VALUE.store(outer.position_p, R);
                        POSITION_I_VALUE.store(outer.position_i, R);
                        VELOCITY_P_VALUE.store(outer.velocity_p, R);
                        VELOCITY_I_VALUE.store(outer.velocity_i, R);
                        VELOCITY_LIMIT_CFG.store(outer.velocity_limit, R);
                        VELOCITY_FILTER_HZ.store(outer.velocity_filter_hz, R);
                        POSITION_FILTER_HZ.store(outer.position_filter_hz, R);
                        FLUX_FILTER_HZ.store(outer.flux_filter_hz, R);
                        TORQUE_FILTER_HZ.store(outer.torque_filter_hz, R);

                        // Restore TMC4671 to operational state. The outer
                        // tuning phases leave the chip in stopped/torque
                        // mode with safety-envelope limits. We must write
                        // the tuned gains, operational limits, filters,
                        // and re-enter position mode before declaring done.
                        let restore = async {
                            use foci_tmc4671::registers::*;

                            // Tuned velocity + position gains.
                            let velocity_reg =
                                (outer.velocity_p as u32) << 16 | outer.velocity_i as u32;
                            tmc.write_register(PID_VELOCITY_P_VELOCITY_I, velocity_reg)
                                .await
                                .map_err(|_| ())?;
                            let position_reg =
                                (outer.position_p as u32) << 16 | outer.position_i as u32;
                            tmc.write_register(PID_POSITION_P_POSITION_I, position_reg)
                                .await
                                .map_err(|_| ())?;

                            // Operational limits (not safety caps).
                            tmc.write_register(PID_VELOCITY_LIMIT, outer.velocity_limit)
                                .await
                                .map_err(|_| ())?;
                            let current_limit = CURRENT_LIMIT_REG.load(R);
                            tmc.write_register(PID_TORQUE_FLUX_LIMITS, current_limit)
                                .await
                                .map_err(|_| ())?;
                            tmc.write_register(PIDOUT_UQ_UD_LIMITS, 1024)
                                .await
                                .map_err(|_| ())?;

                            // Clear PID error sums before mode switch.
                            tmc.write_register(PID_POSITION_ACTUAL, 0)
                                .await
                                .map_err(|_| ())?;
                            let target =
                                tmc.read_register(ABN_DECODER_COUNT).await.map_err(|_| ())?;
                            let target_scaled =
                                target.wrapping_mul(STEP_WIDTH_VALUE.load(R) as u32);
                            tmc.write_register(PID_POSITION_TARGET, target_scaled)
                                .await
                                .map_err(|_| ())?;

                            // Enter position mode (advanced PID cascade).
                            tmc.write_register(MODE_RAMP_MODE_MOTION, 0x8000_0003)
                                .await
                                .map_err(|_| ())?;

                            Ok::<(), ()>(())
                        };
                        if restore.await.is_err() {
                            defmt::error!("OuterDone: position-mode restore SPI failed");
                            foci_firmware::selftest::restore_safe_state(tmc, enable_pin).await;
                            COMMISSION_IN_FLIGHT.store(false, R);
                            motor_enabled = false;
                            calibrated = false;
                            ::ankyra::klipper_reply_from!(
                                &mut crate::_ankyra_config::Sender,
                                foci_firmware::ankyra::replies::FociTuneResult,
                                oid: u8 = oid,
                                status: u8 = foci_firmware::autotune::CommissionError::Spi
                                    .status_code(),
                                velocity_p: u16 = 0,
                                velocity_i: u16 = 0,
                                velocity_limit: u32 = 0,
                                position_p: u16 = 0,
                                position_i: u16 = 0,
                                velocity_filter_hz: u16 = 0,
                                position_filter_hz: u16 = 0,
                                flux_filter_hz: u16 = 0,
                                torque_filter_hz: u16 = 0,
                                j_eff: u32 = 0,
                                b_eff: u32 = 0,
                                warning_code: u8 = 0,
                                mech_torque_step: u16 = 0,
                                mech_fwd_accel: u32 = 0,
                                mech_rev_accel: u32 = 0,
                                mech_fwd_peak_velocity: u32 = 0,
                                mech_rev_peak_velocity: u32 = 0,
                                mech_fwd_travel: u32 = 0,
                                mech_rev_travel: u32 = 0,
                                synth_velocity_p: u16 = 0,
                                synth_velocity_i: u16 = 0,
                                synth_position_p: u16 = 0,
                                synth_position_i: u16 = 0,
                            );
                            continue;
                        }

                        // Re-apply biquad filters from stored atomics.
                        apply_stored_filters(tmc).await;

                        enable_pin.set_high();
                        calibrated = true;
                        motor_enabled = true;
                        MOTOR_ARMED.store(true, R);
                        COMMISSION_IN_FLIGHT.store(false, R);

                        let status = match outer.status {
                            foci_firmware::autotune::CommissionStatus::Accepted => 0u8,
                            foci_firmware::autotune::CommissionStatus::AcceptedWithWarnings => 1,
                            foci_firmware::autotune::CommissionStatus::InnerOnly => 2,
                            foci_firmware::autotune::CommissionStatus::Failed(e) => e.status_code(),
                        };
                        ::ankyra::klipper_reply_from!(
                            &mut crate::_ankyra_config::Sender,
                            foci_firmware::ankyra::replies::FociTuneResult,
                            oid: u8 = oid,
                            status: u8 = status,
                            velocity_p: u16 = outer.velocity_p,
                            velocity_i: u16 = outer.velocity_i,
                            velocity_limit: u32 = outer.velocity_limit,
                            position_p: u16 = outer.position_p,
                            position_i: u16 = outer.position_i,
                            velocity_filter_hz: u16 = outer.velocity_filter_hz,
                            position_filter_hz: u16 = outer.position_filter_hz,
                            flux_filter_hz: u16 = outer.flux_filter_hz,
                            torque_filter_hz: u16 = outer.torque_filter_hz,
                            j_eff: u32 = outer.j_eff,
                            b_eff: u32 = outer.b_eff,
                            warning_code: u8 = outer.warning_code,
                            mech_torque_step: u16 = outer.mech_torque_step,
                            mech_fwd_accel: u32 = outer.mech_fwd_accel,
                            mech_rev_accel: u32 = outer.mech_rev_accel,
                            mech_fwd_peak_velocity: u32 = outer.mech_fwd_peak_velocity,
                            mech_rev_peak_velocity: u32 = outer.mech_rev_peak_velocity,
                            mech_fwd_travel: u32 = outer.mech_fwd_travel,
                            mech_rev_travel: u32 = outer.mech_rev_travel,
                            synth_velocity_p: u16 = outer.synth_velocity_p,
                            synth_velocity_i: u16 = outer.synth_velocity_i,
                            synth_position_p: u16 = outer.synth_position_p,
                            synth_position_i: u16 = outer.synth_position_i,
                        );
                    }
                    foci_firmware::commission::StepResult::OuterFailed(error) => {
                        unsafe { trace_buf().stop() };
                        let oid = engine.oid();
                        let snapshot = engine.entry_snapshot();
                        commission_engine = None;
                        // Keep COMMISSION_IN_FLIGHT set until the restore
                        // check and terminal reply are complete, preventing
                        // a new foci_tune from racing the restore window.

                        // Classify the original error: Spi, Shutdown, and
                        // SafetyEnvelopeViolation are safety faults that
                        // must hard-disable immediately. All other errors
                        // are soft failures eligible for entry-gain restore.
                        use foci_firmware::autotune::CommissionError as CE;
                        let is_safety_fault =
                            matches!(error, CE::Spi | CE::Shutdown | CE::SafetyEnvelopeViolation);

                        let restore_failure: Option<CE> = if is_safety_fault {
                            Some(error)
                        } else if let Some(snapshot) = snapshot {
                            let mut restore_backend = OpenFFBoardBackend { tmc, enable_pin };
                            match foci_firmware::commissioning::inner::restore_entry_gains(
                                &snapshot,
                                &mut restore_backend,
                            )
                            .await
                            {
                                Ok(()) => {
                                    // Stability check with cooperative delay.
                                    Mono::delay(10.millis()).await;
                                    let pos1 = tmc
                                        .read_register(foci_tmc4671::registers::PID_POSITION_ACTUAL)
                                        .await;
                                    Mono::delay(10.millis()).await;
                                    let pos2 = tmc
                                        .read_register(foci_tmc4671::registers::PID_POSITION_ACTUAL)
                                        .await;
                                    match (pos1, pos2) {
                                        (Ok(p1), Ok(p2)) => {
                                            let drift =
                                                (p2 as i32).wrapping_sub(p1 as i32).unsigned_abs();
                                            if drift > 2 {
                                                defmt::error!(
                                                    "restore: unstable: p1={}, p2={}, drift={}",
                                                    p1,
                                                    p2,
                                                    drift
                                                );
                                                Some(CE::CurrentValidationFailed)
                                            } else {
                                                None
                                            }
                                        }
                                        _ => {
                                            defmt::error!("restore: stability read SPI failed");
                                            Some(CE::Spi)
                                        }
                                    }
                                }
                                Err(restore_err) => {
                                    defmt::error!("Stage 2 restore failed: {}", restore_err);
                                    Some(restore_err)
                                }
                            }
                        } else {
                            Some(CE::Spi)
                        };

                        let reply_status = if let Some(fault) = restore_failure {
                            // Safety fault: hard disable, clear all state.
                            // Report the actual restore failure to the host.
                            let _ = tmc
                                .write_register(foci_tmc4671::registers::MODE_RAMP_MODE_MOTION, 0)
                                .await;
                            foci_firmware::selftest::restore_safe_state(tmc, enable_pin).await;
                            calibrated = false;
                            motor_enabled = false;
                            MOTOR_ARMED.store(false, core::sync::atomic::Ordering::Relaxed);
                            stepper_stop_from_p1();
                            defmt::error!("Stage 2 safety fault ({}), motor disabled", fault);
                            fault.status_code()
                        } else {
                            // Soft failure: motor holding with entry gains.
                            enable_pin.set_high();
                            calibrated = true;
                            motor_enabled = true;
                            MOTOR_ARMED.store(true, core::sync::atomic::Ordering::Relaxed);
                            defmt::warn!("Stage 2 soft failure ({}), entry gains restored", error);
                            error.status_code()
                        };

                        COMMISSION_IN_FLIGHT.store(false, core::sync::atomic::Ordering::Relaxed);
                        ::ankyra::klipper_reply_from!(
                            &mut crate::_ankyra_config::Sender,
                            foci_firmware::ankyra::replies::FociCommissionPhase,
                            oid: u8 = oid,
                            phase: u8 = 0,
                            status: u8 = reply_status,
                        );
                    }
                    foci_firmware::commission::StepResult::CalibrateDone(cal_result) => {
                        let oid = engine.oid();
                        commission_engine = None;

                        // The engine's ClosedLoopEntry phase entered position
                        // mode with conservative defaults. Re-apply the
                        // saved gain set from atomics (host-configured or
                        // outer-commissioned values). SPI failures abort
                        // and restore safe state -- reporting success with
                        // wrong gains active is worse than failing.
                        //
                        // NOTE: the stability check ran with conservative
                        // defaults, not the saved gains. A post-restore
                        // stability re-check is not implemented (the old
                        // tmc_enter_closedloop didn't have one either).
                        let gain_restore = async {
                            use core::sync::atomic::Ordering::Relaxed;
                            use foci_tmc4671::registers::*;

                            // Current-loop gains from prior commissioning.
                            if let Some(gains) = stored_gains {
                                let (flux_reg, torque_reg) = gains.to_registers();
                                tmc.write_register(PID_FLUX_P_FLUX_I, flux_reg)
                                    .await
                                    .map_err(|_| ())?;
                                tmc.write_register(PID_TORQUE_P_TORQUE_I, torque_reg)
                                    .await
                                    .map_err(|_| ())?;
                            }

                            // Velocity gains.
                            let velocity_p = VELOCITY_P_VALUE.load(Relaxed);
                            let velocity_i = VELOCITY_I_VALUE.load(Relaxed);
                            let velocity_reg = (velocity_p as u32) << 16 | velocity_i as u32;
                            tmc.write_register(PID_VELOCITY_P_VELOCITY_I, velocity_reg)
                                .await
                                .map_err(|_| ())?;

                            // Position gains.
                            let position_p = POSITION_P_VALUE.load(Relaxed);
                            let position_i = POSITION_I_VALUE.load(Relaxed);
                            let position_reg = (position_p as u32) << 16 | position_i as u32;
                            tmc.write_register(PID_POSITION_P_POSITION_I, position_reg)
                                .await
                                .map_err(|_| ())?;

                            // Velocity limit.
                            let velocity_limit = VELOCITY_LIMIT_CFG.load(Relaxed);
                            tmc.write_register(PID_VELOCITY_LIMIT, velocity_limit)
                                .await
                                .map_err(|_| ())?;

                            // Current limit (TMC4671 resets on mode change).
                            let current_limit = CURRENT_LIMIT_REG.load(Relaxed);
                            tmc.write_register(PID_TORQUE_FLUX_LIMITS, current_limit)
                                .await
                                .map_err(|_| ())?;

                            Ok::<(), ()>(())
                        };
                        if gain_restore.await.is_err() {
                            defmt::error!("CalibrateDone: saved gain restore SPI failed");
                            foci_firmware::selftest::restore_safe_state(tmc, enable_pin).await;
                            COMMISSION_IN_FLIGHT
                                .store(false, core::sync::atomic::Ordering::Relaxed);
                            ::ankyra::klipper_reply_from!(
                                &mut crate::_ankyra_config::Sender,
                                foci_firmware::ankyra::replies::FociCalibrateResult,
                                oid: u8 = oid,
                                status: u8 = foci_firmware::calibration::CalibrationError::SpiError.status_code(),
                                adc_i0: u16 = 0,
                                adc_i1: u16 = 0,
                                encoder_count: u32 = 0,
                            );
                            continue;
                        }

                        // Re-apply biquad filters from stored atomics.
                        // The engine's ClosedLoopEntry disabled all filters
                        // (no access to filter Hz atomics). Filters are
                        // best-effort -- failure does not abort calibration.
                        apply_stored_filters(tmc).await;

                        enable_pin.set_high();
                        calibrated = true;
                        motor_enabled = true;
                        MOTOR_ARMED.store(true, core::sync::atomic::Ordering::Relaxed);
                        arm_check::spawn().ok();
                        COMMISSION_IN_FLIGHT.store(false, core::sync::atomic::Ordering::Relaxed);
                        defmt::info!("foci_calibrate complete: oid={}", oid);
                        ::ankyra::klipper_reply_from!(
                            &mut crate::_ankyra_config::Sender,
                            foci_firmware::ankyra::replies::FociCalibrateResult,
                            oid: u8 = oid,
                            status: u8 = 0,
                            adc_i0: u16 = cal_result.adc_i0,
                            adc_i1: u16 = cal_result.adc_i1,
                            encoder_count: u32 = cal_result.encoder_count,
                        );
                    }
                    foci_firmware::commission::StepResult::SelftestDone => {
                        let oid = engine.oid();
                        let results = engine.take_selftest_results();
                        commission_engine = None;
                        COMMISSION_IN_FLIGHT.store(false, core::sync::atomic::Ordering::Relaxed);
                        // Engine already restored safe state internally.
                        // Emit one result message per populated stage, then the
                        // terminal done message.
                        for (stage, stage_value) in results.iter() {
                            ::ankyra::klipper_reply_from!(
                                &mut crate::_ankyra_config::Sender,
                                foci_firmware::ankyra::replies::FociSelftestResult,
                                oid: u8 = oid,
                                stage: u8 = stage,
                                status: u8 = stage_value.status,
                                value: u32 = stage_value.value,
                            );
                        }
                        ::ankyra::klipper_reply_from!(
                            &mut crate::_ankyra_config::Sender,
                            foci_firmware::ankyra::replies::FociSelftestDone,
                            oid: u8 = oid,
                            status: u8 = 0,
                        );
                    }
                    foci_firmware::commission::StepResult::CommissionDone(commission_result) => {
                        unsafe { trace_buf().stop() };
                        let oid = engine.oid();
                        commission_engine = None;

                        // The engine's ClosedLoopEntry phase already
                        // configured PID gains, limits, filters, and
                        // entered position mode. Re-apply current limit.
                        let current_limit =
                            CURRENT_LIMIT_REG.load(core::sync::atomic::Ordering::Relaxed);
                        let _ = tmc
                            .write_register(
                                foci_tmc4671::registers::PID_TORQUE_FLUX_LIMITS,
                                current_limit,
                            )
                            .await;

                        // Persist inner gains so future calibrate uses them.
                        let inner = &commission_result.inner;
                        stored_gains = Some(foci_firmware::pid::InnerLoopGains {
                            flux_p: inner.flux_p,
                            flux_i: inner.flux_i,
                            torque_p: inner.torque_p,
                            torque_i: inner.torque_i,
                        });

                        // Sync all outer-gain atomics to match the
                        // fallback values the engine wrote to hardware.
                        // CalibrateDone reads these on the next re-enable.
                        POSITION_P_VALUE.store(
                            commission_result.fallback_position_p,
                            core::sync::atomic::Ordering::Relaxed,
                        );
                        POSITION_I_VALUE.store(
                            commission_result.fallback_position_i,
                            core::sync::atomic::Ordering::Relaxed,
                        );
                        VELOCITY_P_VALUE.store(
                            commission_result.fallback_velocity_p,
                            core::sync::atomic::Ordering::Relaxed,
                        );
                        VELOCITY_I_VALUE.store(
                            commission_result.fallback_velocity_i,
                            core::sync::atomic::Ordering::Relaxed,
                        );
                        VELOCITY_LIMIT_CFG.store(
                            commission_result.fallback_velocity_limit,
                            core::sync::atomic::Ordering::Relaxed,
                        );

                        enable_pin.set_high();
                        calibrated = true;
                        motor_enabled = true;
                        MOTOR_ARMED.store(true, core::sync::atomic::Ordering::Relaxed);
                        arm_check::spawn().ok();
                        COMMISSION_IN_FLIGHT.store(false, core::sync::atomic::Ordering::Relaxed);

                        let status = match inner.status {
                            foci_firmware::autotune::CommissionStatus::Accepted => 0u8,
                            foci_firmware::autotune::CommissionStatus::AcceptedWithWarnings => 1,
                            foci_firmware::autotune::CommissionStatus::InnerOnly => 2,
                            foci_firmware::autotune::CommissionStatus::Failed(e) => e.status_code(),
                        };
                        defmt::info!("foci_commission complete: oid={} status={}", oid, status);
                        ::ankyra::klipper_reply_from!(
                            &mut crate::_ankyra_config::Sender,
                            foci_firmware::ankyra::replies::FociCommissionResult,
                            oid: u8 = oid,
                            status: u8 = status,
                            r_mohm: u32 = inner.r_mohm,
                            l_uh: u32 = inner.l_uh,
                            flux_p: u16 = inner.flux_p,
                            flux_i: u16 = inner.flux_i,
                            torque_p: u16 = inner.torque_p,
                            torque_i: u16 = inner.torque_i,
                            lambda_us: u32 = inner.lambda_us,
                            theta_e_us: u32 = inner.theta_e_us,
                            ringing_count: u8 = inner.ringing_count,
                            bandwidth_hz: u32 = inner.bandwidth_hz,
                            fallback_velocity_p: u16 = commission_result.fallback_velocity_p,
                            fallback_velocity_i: u16 = commission_result.fallback_velocity_i,
                            fallback_position_p: u16 = commission_result.fallback_position_p,
                            fallback_position_i: u16 = commission_result.fallback_position_i,
                            fallback_velocity_limit: u32 = commission_result.fallback_velocity_limit,
                        );
                    }
                }
                // For Done/Failed cases, fall through to normal processing
            }

            match select(rx.recv(), Mono::delay(1.millis())).await {
                // Channel message received
                Either::First(Ok(req)) => match req {
                    TmcRequest::SetEnable(enabled) => {
                        if enabled {
                            if !calibrated {
                                // Auto-calibrate: Klipper may send enable before
                                // the Python module's foci_calibrate arrives due
                                // to queue_digital_out timing. Launch the
                                // commissioning engine to run ADC calibration
                                // asynchronously; the CalibrateDone handler will
                                // complete encoder alignment and closed-loop entry.
                                if COMMISSION_IN_FLIGHT
                                    .swap(true, core::sync::atomic::Ordering::Relaxed)
                                {
                                    defmt::warn!("Auto-calibrate skipped: commission in flight");
                                    continue;
                                }
                                // TODO: hardcoded OID 0 — correct for single-motor
                                // OpenFFBoard but needs OID routing for multi-motor boards.
                                defmt::info!("Auto-calibrating on enable");
                                commission_engine = Some(
                                    foci_firmware::commission::CommissioningEngine::new_calibrate(
                                        0,
                                    ),
                                );
                                // Don't set motor_enabled/MOTOR_ARMED yet --
                                // the CalibrateDone handler does that after
                                // encoder alignment + closed-loop entry.
                                continue;
                            }
                            motor_enabled = true;
                            enable_pin.set_high();

                            // Zero firmware position counter
                            STEPPER_POSITION.store(0, core::sync::atomic::Ordering::Relaxed);

                            // Arm stepper: allow TIM5 CC1 ISR to fire
                            MOTOR_ARMED.store(true, core::sync::atomic::Ordering::Relaxed);

                            // If queue already has moves (burst-drain scenario),
                            // arm timer to start processing them
                            arm_check::spawn().ok();

                            defmt::info!("Motor enabled (calibrated, step/dir armed)");
                        } else {
                            // Disable: stop stepping, kill motor
                            MOTOR_ARMED.store(false, core::sync::atomic::Ordering::Relaxed);
                            stepper_stop_from_p1();

                            motor_enabled = false;
                            calibrated = false;
                            foci_firmware::selftest::restore_safe_state(tmc, enable_pin).await;

                            // Zero position counter
                            STEPPER_POSITION.store(0, core::sync::atomic::Ordering::Relaxed);

                            defmt::info!("TMC4671 motor disabled");
                        }
                    }
                    TmcRequest::Calibrate { oid } => {
                        if motor_enabled {
                            defmt::warn!("foci_calibrate rejected: motor already enabled");
                            COMMISSION_IN_FLIGHT
                                .store(false, core::sync::atomic::Ordering::Relaxed);
                            ::ankyra::klipper_reply_from!(
                                &mut crate::_ankyra_config::Sender,
                                foci_firmware::ankyra::replies::FociCalibrateResult,
                                oid: u8 = oid,
                                status: u8 = foci_firmware::calibration::CalibrationError::AlreadyEnabled.status_code(),
                                adc_i0: u16 = 0,
                                adc_i1: u16 = 0,
                                encoder_count: u32 = 0,
                            );
                            continue;
                        }
                        // Engine tick will drive ADC cal; CalibrateDone handler
                        // performs encoder alignment and closed-loop entry.
                        calibrated = false;
                        commission_engine = Some(
                            foci_firmware::commission::CommissioningEngine::new_calibrate(oid),
                        );
                    }
                    #[cfg(feature = "dev")]
                    TmcRequest::Init => {
                        defmt::info!("TMC4671 init: starting open-loop stepper config");
                        tmc_enter_openloop(tmc, enable_pin).await;
                    }
                    #[cfg(feature = "dev")]
                    TmcRequest::TestSpin { revolutions } => {
                        do_test_spin(tmc, revolutions).await;
                    }
                    TmcRequest::SetPidGains {
                        flux_p,
                        flux_i,
                        torque_p,
                        torque_i,
                    } => {
                        stored_gains = Some(foci_firmware::pid::InnerLoopGains {
                            flux_p,
                            flux_i,
                            torque_p,
                            torque_i,
                        });
                        defmt::info!(
                            "PID gains received: flux P={} I={}, torque P={} I={}",
                            flux_p,
                            flux_i,
                            torque_p,
                            torque_i,
                        );
                    }
                    TmcRequest::SetPositionGains {
                        position_p,
                        position_i,
                        velocity_p,
                        velocity_i,
                    } => {
                        POSITION_P_VALUE.store(position_p, core::sync::atomic::Ordering::Relaxed);
                        POSITION_I_VALUE.store(position_i, core::sync::atomic::Ordering::Relaxed);
                        VELOCITY_P_VALUE.store(velocity_p, core::sync::atomic::Ordering::Relaxed);
                        VELOCITY_I_VALUE.store(velocity_i, core::sync::atomic::Ordering::Relaxed);
                        defmt::info!(
                            "Position gains received: pos P={} I={}, vel P={} I={}",
                            position_p,
                            position_i,
                            velocity_p,
                            velocity_i,
                        );
                    }
                    TmcRequest::SetVelocityFeedforward { enable } => {
                        defmt::info!("Velocity feedforward: {} (via queue)", enable);
                        // Feedforward flag is already set by the atomic in the
                        // command handler; this arm exists for exhaustiveness.
                    }
                    TmcRequest::Commission { oid, profile } => {
                        if motor_enabled {
                            COMMISSION_IN_FLIGHT
                                .store(false, core::sync::atomic::Ordering::Relaxed);
                            ::ankyra::klipper_reply_from!(
                                &mut crate::_ankyra_config::Sender,
                                foci_firmware::ankyra::replies::FociCommissionPhase,
                                oid: u8 = oid,
                                phase: u8 = 0,
                                status: u8 =
                                    foci_firmware::autotune::CommissionError::MotorEnabled
                                        .status_code(),
                            );
                            continue;
                        }
                        let current_limit =
                            CURRENT_LIMIT_REG.load(core::sync::atomic::Ordering::Relaxed);
                        if current_limit == 0 {
                            COMMISSION_IN_FLIGHT
                                .store(false, core::sync::atomic::Ordering::Relaxed);
                            ::ankyra::klipper_reply_from!(
                                &mut crate::_ankyra_config::Sender,
                                foci_firmware::ankyra::replies::FociCommissionPhase,
                                oid: u8 = oid,
                                phase: u8 = 0,
                                status: u8 =
                                    foci_firmware::autotune::CommissionError::NoCurrent
                                        .status_code(),
                            );
                            continue;
                        }
                        calibrated = false;
                        commission_engine = Some(
                            foci_firmware::commission::CommissioningEngine::new_commission(
                                oid,
                                foci_firmware::autotune::CommissionProfile::from_u8(profile),
                            ),
                        );
                        // Trace start deferred to ClosedLoopEntry phase transition
                        // (see PhaseChanged handler). Spec scopes Stage 1 trace to
                        // ClosedLoopEntry only — starting here would add SPI reads
                        // throughout inner diagnostics/electrical ID phases.
                    }
                    TmcRequest::Tune {
                        oid,
                        profile,
                        mode,
                        inner_lambda_us,
                        theta_e_us,
                        current_ringing_count,
                        current_bandwidth_hz,
                    } => {
                        // Stage 2 requires the motor to be enabled and
                        // holding in closed-loop position mode (from Stage 1
                        // or a prior calibrate). Reject if not ready.
                        if !motor_enabled || !calibrated {
                            COMMISSION_IN_FLIGHT
                                .store(false, core::sync::atomic::Ordering::Relaxed);
                            ::ankyra::klipper_reply_from!(
                                &mut crate::_ankyra_config::Sender,
                                foci_firmware::ankyra::replies::FociCommissionPhase,
                                oid: u8 = oid,
                                phase: u8 = 0,
                                status: u8 =
                                    foci_firmware::autotune::CommissionError::EncoderNotAligned
                                        .status_code(),
                            );
                            continue;
                        }
                        let current_limit =
                            CURRENT_LIMIT_REG.load(core::sync::atomic::Ordering::Relaxed);
                        if current_limit == 0 {
                            COMMISSION_IN_FLIGHT
                                .store(false, core::sync::atomic::Ordering::Relaxed);
                            ::ankyra::klipper_reply_from!(
                                &mut crate::_ankyra_config::Sender,
                                foci_firmware::ankyra::replies::FociCommissionPhase,
                                oid: u8 = oid,
                                phase: u8 = 0,
                                status: u8 =
                                    foci_firmware::autotune::CommissionError::NoCurrent
                                        .status_code(),
                            );
                            continue;
                        }
                        let entry_snapshot = {
                            use core::sync::atomic::Ordering::Relaxed;
                            foci_firmware::commission::EntrySnapshot {
                                velocity_p: VELOCITY_P_VALUE.load(Relaxed),
                                velocity_i: VELOCITY_I_VALUE.load(Relaxed),
                                position_p: POSITION_P_VALUE.load(Relaxed),
                                position_i: POSITION_I_VALUE.load(Relaxed),
                                velocity_limit: VELOCITY_LIMIT_CFG.load(Relaxed),
                                voltage_limit: 16000,
                                velocity_filter_hz: VELOCITY_FILTER_HZ.load(Relaxed),
                                torque_filter_hz: TORQUE_FILTER_HZ.load(Relaxed),
                                position_filter_hz: POSITION_FILTER_HZ.load(Relaxed),
                                flux_filter_hz: FLUX_FILTER_HZ.load(Relaxed),
                            }
                        };
                        let outer_params = foci_firmware::commission::OuterTuneParams {
                            profile: foci_firmware::autotune::CommissionProfile::from_u8(profile),
                            mode: foci_firmware::autotune::OperatingMode::from_u8(mode),
                            inner_lambda_us,
                            theta_e_us,
                            current_ringing: current_ringing_count > 1,
                            current_bandwidth_hz,
                        };
                        commission_engine =
                            Some(foci_firmware::commission::CommissioningEngine::new_outer(
                                oid,
                                outer_params,
                                entry_snapshot,
                            ));
                        unsafe {
                            trace_buf().start(oid, foci_firmware::trace::PRESET_FULL);
                            trace_buf()
                                .set_phase(foci_firmware::autotune::PhaseId::MechanicalId.to_u8());
                        }
                    }
                    TmcRequest::SelfTest { oid } => {
                        if motor_enabled {
                            defmt::warn!("foci_selftest rejected: motor enabled");
                            COMMISSION_IN_FLIGHT
                                .store(false, core::sync::atomic::Ordering::Relaxed);
                            ::ankyra::klipper_reply_from!(
                                &mut crate::_ankyra_config::Sender,
                                foci_firmware::ankyra::replies::FociSelftestDone,
                                oid: u8 = oid,
                                status: u8 =
                                    foci_firmware::autotune::CommissionError::MotorEnabled
                                        .status_code(),
                            );
                            continue;
                        }
                        // Engine tick drives selftest phases; SelftestDone
                        // handler sends the completion signal.
                        commission_engine =
                            Some(foci_firmware::commission::CommissioningEngine::new_selftest(oid));
                    }
                    req @ TmcRequest::SetEncoder { .. } => {
                        if handle_tmc_request(req, tmc).await.is_err() {
                            ctx.shared.app_state.lock(|state| {
                                if enter_shutdown(state) {
                                    ::ankyra::klipper_shutdown_from!(
                                        &mut crate::_ankyra_config::Sender,
                                        "TMC request validation failed",
                                        read_tim5(),
                                    );
                                }
                            });
                        }
                        // Compute feedforward scale factor from step_width and PWM config
                        velocity_scale = foci_firmware::tmc::compute_velocity_scale_factor(
                            STEP_WIDTH_VALUE
                                .load(core::sync::atomic::Ordering::Relaxed)
                                .unsigned_abs(),
                            CLOCK_FREQ,
                            TMC_CONFIG.pwm_freq_hz(),
                        );
                    }
                    other => {
                        if handle_tmc_request(other, tmc).await.is_err() {
                            ctx.shared.app_state.lock(|state| {
                                if enter_shutdown(state) {
                                    ::ankyra::klipper_shutdown_from!(
                                        &mut crate::_ankyra_config::Sender,
                                        "TMC request validation failed",
                                        read_tim5(),
                                    );
                                }
                            });
                        }
                    }
                },
                Either::First(Err(_)) => {}
                // 1ms timeout — write position target via SPI
                Either::Second(()) => {}
            }

            // Capture tick start for deadline computation. Must be after
            // select() resolves — placing it before would include the 1ms
            // idle wait, making deadline checks ineffective.
            let tick_start = Mono::now();

            // Check emergency disable (non-blocking). Triggered by EXTI
            // STATUS interrupt or enter_shutdown. Disables motor and reports
            // the fault to Klipper so the host knows the MCU is down.
            if TMC_DISABLE_SIGNAL.try_take().is_some() {
                // Stop hardware stepper immediately
                MOTOR_ARMED.store(false, core::sync::atomic::Ordering::Relaxed);
                stepper_stop_from_p1();
                FAULT_ACTIVE.store(false, core::sync::atomic::Ordering::Relaxed);

                motor_enabled = false;
                calibrated = false;
                enable_pin.set_low();
                // Read STATUS_FLAGS to identify the fault cause.
                let flags = tmc
                    .read_register(foci_tmc4671::registers::STATUS_FLAGS)
                    .await
                    .unwrap_or(0);
                defmt::error!("TMC4671 emergency disable: STATUS={:#010x}", flags);
                // Report shutdown to Klipper with a specific reason.
                const NOT_PLL_LOCKED: u32 = 1 << 19;
                const ADC_I_CLIPPED: u32 = 1 << 26;
                ctx.shared.app_state.lock(|state| {
                    if flags & NOT_PLL_LOCKED != 0 {
                        if enter_shutdown(state) {
                            ::ankyra::klipper_shutdown_from!(
                                &mut crate::_ankyra_config::Sender,
                                "TMC4671: PLL not locked (clock lost)",
                                read_tim5(),
                            );
                        }
                    } else if flags & ADC_I_CLIPPED != 0 {
                        if enter_shutdown(state) {
                            ::ankyra::klipper_shutdown_from!(
                                &mut crate::_ankyra_config::Sender,
                                "TMC4671: ADC current clipped (overcurrent)",
                                read_tim5(),
                            );
                        }
                    } else if enter_shutdown(state) {
                        ::ankyra::klipper_shutdown_from!(
                            &mut crate::_ankyra_config::Sender,
                            "TMC4671 fault",
                            read_tim5(),
                        );
                    }
                });
            }

            // Hardware step/dir: position target is driven by TIM5 CC1 ISR
            // toggling STEP GPIO → TMC4671 hardware step counter. No SPI
            // position target write needed. The 1ms tick loop only handles
            // STATUS_FLAGS polling, TrSync, endstop, and watchdog.

            // Velocity feedforward: compute offset from live step interval
            // and write to PID_VELOCITY_OFFSET. Gated on:
            // - motor_enabled (closedloop active, safe to write 0x67)
            // - velocity_scale != 0 (encoder configured)
            // - VELOCITY_FEEDFORWARD enabled (host config)
            // - commission_engine is None (not commissioning)
            // - homing not active
            let ff_active = motor_enabled
                && velocity_scale != 0
                && VELOCITY_FEEDFORWARD.load(core::sync::atomic::Ordering::Relaxed)
                && commission_engine.is_none();

            if ff_active {
                let homing_active = HOMING_STATE.load(core::sync::atomic::Ordering::Relaxed) != 0;
                let reset = FEEDFORWARD_RESET.swap(false, core::sync::atomic::Ordering::Relaxed);

                let offset = if homing_active || reset {
                    0i32
                } else {
                    let interval = STEP_INTERVAL.load(core::sync::atomic::Ordering::Relaxed);
                    let forward = STEP_DIRECTION.load(core::sync::atomic::Ordering::Relaxed);
                    foci_firmware::tmc::compute_velocity_offset(velocity_scale, interval, forward)
                };

                let _ = tmc
                    .write_register(foci_tmc4671::registers::PID_VELOCITY_OFFSET, offset as u32)
                    .await;
            } else if feedforward_was_active {
                // Feedforward just deactivated — clear stale offset
                let _ = tmc
                    .write_register(foci_tmc4671::registers::PID_VELOCITY_OFFSET, 0)
                    .await;
            }
            feedforward_was_active = ff_active;

            // Trace capture: read registers and store sample.
            // Deadline = tick start + 1ms. Budget check inside ensures
            // trace SPI reads don't overrun the tick period.
            let tick_deadline = tick_start + fugit::Duration::<u64, 1, 1_000_000>::millis(1);
            trace_sample_tick(tmc, tick_deadline).await;

            if PID_ERROR_SUMS_CLEAR.swap(false, core::sync::atomic::Ordering::Relaxed) {
                pid_error_clear_countdown_ms = HOLD_SETTLE_DELAY_MS;
            }
            let timer_active = STEP_TIMER_ACTIVE_MIRROR.load(core::sync::atomic::Ordering::Relaxed);
            if motor_enabled
                && commission_engine.is_none()
                && !timer_active
                && pid_error_clear_countdown_ms > 0
            {
                pid_error_clear_countdown_ms -= 1;
                if pid_error_clear_countdown_ms == 0 {
                    if settle_hold_and_clear_pid_errors(tmc).await.is_err() {
                        defmt::warn!("TMC4671 PID settle/clear failed");
                    } else {
                        defmt::info!("TMC4671 PID settled and cleared after motion");
                    }
                }
            }

            // TrSync and endstop polling have been replaced by the TIM5 CC2
            // trigger scheduler. See step_timer_irq CC2 dispatch above.

            // Homing I-suppression state machine.
            //
            // During homing the motor hits a physical constraint (endstop or
            // bed). The position I integrator accumulates error, causing
            // overshoot when the motor releases. Fix: zero position I gain
            // while homing, restore + clear integrator when TrSync resolves.
            {
                use core::sync::atomic::Ordering::Relaxed;
                let homing = HOMING_STATE.load(Relaxed);
                match homing {
                    1 => {
                        // SuppressPending: write position P with I=0
                        let pos_p = POSITION_P_VALUE.load(Relaxed);
                        let reg = (pos_p as u32) << 16; // P in upper half, I=0
                        let _ = tmc
                            .write_register(foci_tmc4671::registers::PID_POSITION_P_POSITION_I, reg)
                            .await;
                        HOMING_STATE.store(2, Relaxed);
                        defmt::info!("homing: position I suppressed");
                    }
                    2 => {
                        // Suppressing: check if a trigger has fired via the
                        // HOMING_TRIGGERED sentinel (set by TriggerDomain on
                        // first trigger, 0xFF = no trigger pending).
                        // P1 never enters TriggerDomain per spec.
                        // If host's trsync_trigger runs before P1 polls,
                        // trsync_trigger advances HOMING_STATE 2→3 directly.
                        if HOMING_TRIGGERED.load(Relaxed) != 0xFF {
                            HOMING_STATE.store(3, Relaxed);
                        }
                    }
                    3 => {
                        // RestorePending: restore position I and clear integrator
                        let pos_p = POSITION_P_VALUE.load(Relaxed);
                        let pos_i = POSITION_I_VALUE.load(Relaxed);
                        let reg = ((pos_p as u32) << 16) | pos_i as u32;
                        let _ = tmc
                            .write_register(foci_tmc4671::registers::PID_POSITION_P_POSITION_I, reg)
                            .await;
                        // Clear position error integrator
                        let _ = tmc
                            .write_sub_register(
                                foci_tmc4671::registers::PID_ERROR_PID_POSITION_ERROR_SUM,
                                0,
                            )
                            .await;
                        HOMING_STATE.store(0, Relaxed);
                        defmt::info!("homing: position I restored, integrator cleared");
                    }
                    _ => {} // Idle, nothing to do
                }
            }

            // Poll STATUS_FLAGS every ~1 second for warning-level flags.
            // Critical faults (PLL lost, ADC clipped) are handled by EXTI.
            status_poll_counter += 1;
            if status_poll_counter >= 1000 {
                status_poll_counter = 0;
                if let Ok(flags) = tmc
                    .read_register(foci_tmc4671::registers::STATUS_FLAGS)
                    .await
                {
                    // Warning flags: PID output limits (position, velocity, current)
                    const PID_LIMIT_MASK: u32 = 0x0000_FFFF; // bits 0-15
                    const PWM_LIMIT_MASK: u32 = (1 << 24) | (1 << 25); // pwm_min, pwm_max
                    const FAULT_MASK: u32 = (1 << 19) | (1 << 26); // not_PLL_locked, adc_i_clipped

                    if flags & FAULT_MASK != 0 {
                        defmt::error!("TMC4671 STATUS fault: {:#010x}", flags);
                        TMC_DISABLE_SIGNAL.signal(());
                    } else if motor_enabled && flags & (PID_LIMIT_MASK | PWM_LIMIT_MASK) != 0 {
                        defmt::warn!("TMC4671 STATUS: {:#010x}", flags);
                    }
                }
            }
        }
    }

    /// Sweep PHI_E_EXT through N full mechanical revolutions.
    ///
    /// With 50 pole pairs, one mechanical revolution = 50 electrical
    /// cycles. Each electrical cycle sweeps PHI_E_EXT from 0 to 65535.
    /// Uses whatever voltage/mode settings were configured by tmc_init.
    #[cfg(feature = "dev")]
    async fn do_test_spin(tmc: &mut Tmc4671Spi<TmcSpiDevice>, revolutions: u8) {
        use foci_tmc4671::registers::*;

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

    /// Perform TMC4671 hardware safety shutdown, PWM/ADC configuration, and ADC
    /// offset calibration.
    ///
    /// Steps:
    /// 1. Safety shutdown: enable_pin low, MODE=0, voltage limit=1024, zero targets
    /// 2. PWM config: polarities=0, PWM frequency and BBM from TMC_CONFIG, CHOP=0 for calibration
    /// Configure TMC4671 for open-loop stepper operation with current regulation.
    ///
    /// Performs hardware init, ADC offset calibration, and configures torque
    /// mode with PHI_E_EXT for open-loop angle control. Self-contained dev
    /// helper; production ADC calibration uses the commissioning engine's
    /// cooperative AdcCalibrate phase.
    ///
    /// Register values verified against TMC4671-LA datasheet rev 2.08.
    #[cfg(feature = "dev")]
    async fn tmc_enter_openloop(
        tmc: &mut Tmc4671Spi<TmcSpiDevice>,
        enable_pin: &mut Output<'static>,
    ) {
        use foci_tmc4671::registers::*;

        // --- ADC calibration (inlined from former tmc_adc_calibrate) ---

        // 1. SAFETY FIRST: stop motor, disable outputs, set voltage limit
        enable_pin.set_low();
        let _ = tmc.write_register(MODE_RAMP_MODE_MOTION, 0).await;
        let _ = tmc.write_register(PIDOUT_UQ_UD_LIMITS, 1024).await;
        let _ = tmc.write_register(PID_TORQUE_FLUX_TARGET, 0).await;
        let _ = tmc.write_register(UQ_UD_EXT, 0).await;

        // 2. PWM configuration (chopping disabled for ADC calibration)
        let _ = tmc.write_register(PWM_POLARITIES, 0).await;
        let _ = tmc
            .write_register(PWM_MAXCNT, TMC_CONFIG.pwm_maxcnt as u32)
            .await;
        let _ = tmc
            .write_register(PWM_BBM_H_BBM_L, TMC_CONFIG.bbm_register())
            .await;
        if tmc.write_register(PWM_SV_CHOP, 0).await.is_err() {
            defmt::error!("open-loop init: PWM_SV_CHOP write failed");
            return;
        }

        // 3. ADC configuration
        let _ = tmc
            .write_register(DSADC_MDEC_B_MDEC_A, TMC_CONFIG.dsadc_mdec_register())
            .await;
        let _ = tmc.write_register(ADC_I_SELECT, (1 << 28) | (1 << 8)).await;

        // 4. Motor type: 2-phase stepper (type 2), 50 pole pairs
        let _ = tmc
            .write_register(MOTOR_TYPE_N_POLE_PAIRS, (2 << 16) | 50)
            .await;

        // 4b. Velocity meter: PWM-synchronized for advanced PI stability
        let _ = tmc.write_register(VELOCITY_SELECTION, (1 << 8) | 0).await;

        // 5. ADC offset calibration (PWM_CHOP=0, no current flows)
        let _ = tmc.write_register(ADC_I0_SCALE_OFFSET, 256 << 16).await;
        let _ = tmc.write_register(ADC_I1_SCALE_OFFSET, 256 << 16).await;
        enable_pin.set_high();
        Mono::delay(250_u64.millis()).await;

        let mut sum_i0: i32 = 0;
        let mut sum_i1: i32 = 0;
        const N_SAMPLES: i32 = 128;
        for _ in 0..N_SAMPLES {
            if let Ok(raw) = tmc.read_register(ADC_IWY_IUX).await {
                sum_i0 += (raw & 0xFFFF) as i16 as i32;
                sum_i1 += ((raw >> 16) & 0xFFFF) as i16 as i32;
            }
            Mono::delay(1_u64.millis()).await;
        }
        let offset_i0 = (sum_i0 / N_SAMPLES) as i16 as u16;
        let offset_i1 = (sum_i1 / N_SAMPLES) as i16 as u16;

        if !(30000..=36000).contains(&offset_i0) || !(30000..=36000).contains(&offset_i1) {
            defmt::error!(
                "ADC offset out of range: I0={}, I1={} (expected 30000..=36000)",
                offset_i0,
                offset_i1
            );
            return;
        }

        let _ = tmc
            .write_register(ADC_I0_SCALE_OFFSET, (256 << 16) | offset_i0 as u32)
            .await;
        let _ = tmc
            .write_register(ADC_I1_SCALE_OFFSET, (256 << 16) | offset_i1 as u32)
            .await;
        defmt::info!("ADC calibrated: I0={}, I1={}", offset_i0, offset_i1);

        // 6. Re-enable PWM chopping
        let _ = tmc.write_register(PWM_SV_CHOP, 7).await;

        // --- Open-loop configuration ---

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

        // 11. Set flux (D-axis) target for alignment/holding at PHI_E=0
        // CRITICAL: Flux target is bits[15:0], torque target is bits[31:16].
        // Setting torque (Q-axis) instead of flux (D-axis) causes the rotor to
        // align 90 electrical degrees off, producing reversed commutation.
        let flux_target: i16 = 200; // ~0.23A, conservative
        let target_reg: u32 = PidTorqueFluxTarget::new()
            .with_pid_flux_target(flux_target)
            .with_pid_torque_target(0)
            .into();
        let _ = tmc.write_register(PID_TORQUE_FLUX_TARGET, target_reg).await;

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

    /// Clear TMC4671 PID error accumulators.
    ///
    /// The current-loop sums can saturate during commissioning and step
    /// transients, then keep STATUS_FLAGS asserted even after the commanded
    /// position has returned to zero. Clearing all four sums at lifecycle
    /// boundaries removes stale integral state without changing configured
    /// gains or the active position target.
    async fn clear_pid_error_sums(tmc: &mut Tmc4671Spi<TmcSpiDevice>) -> Result<(), ()> {
        use foci_tmc4671::registers::*;

        tmc.write_sub_register(PID_ERROR_PID_TORQUE_ERROR_SUM, 0)
            .await
            .map_err(|_| ())?;
        tmc.write_sub_register(PID_ERROR_PID_FLUX_ERROR_SUM, 0)
            .await
            .map_err(|_| ())?;
        tmc.write_sub_register(PID_ERROR_PID_VELOCITY_ERROR_SUM, 0)
            .await
            .map_err(|_| ())?;
        tmc.write_sub_register(PID_ERROR_PID_POSITION_ERROR_SUM, 0)
            .await
            .map_err(|_| ())?;
        Ok(())
    }

    async fn settle_hold_and_clear_pid_errors(
        tmc: &mut Tmc4671Spi<TmcSpiDevice>,
    ) -> Result<(), ()> {
        use core::sync::atomic::Ordering::Relaxed;
        use foci_tmc4671::registers::*;

        let target = tmc
            .read_register(PID_POSITION_TARGET)
            .await
            .map_err(|_| ())?;
        let actual = tmc
            .read_register(PID_POSITION_ACTUAL)
            .await
            .map_err(|_| ())?;
        let step_width = STEP_WIDTH_VALUE.load(Relaxed).unsigned_abs().max(1);
        let deadband = step_width.saturating_mul(HOLD_SETTLE_DEADBAND_COUNTS);
        let error = (actual as i32).wrapping_sub(target as i32);
        if error.unsigned_abs() <= deadband {
            // With ABN feedback active, PID_POSITION_ACTUAL is driven by
            // encoder phase. Accept tiny settled error by moving the internal
            // TMC hold target to the measured actual position; firmware and
            // Klipper logical step counts remain unchanged.
            tmc.write_register(PID_POSITION_TARGET, actual)
                .await
                .map_err(|_| ())?;
        }

        clear_pid_error_sums(tmc).await?;
        tmc.write_register(STATUS_FLAGS, 0).await.map_err(|_| ())?;
        Ok(())
    }

    /// Re-apply biquad filters from stored atomics.
    ///
    /// Called after CalibrateDone to restore filters that the engine's
    /// ClosedLoopEntry phase disabled (it has no access to the filter
    /// Hz atomics). Best-effort: warns on failure, does not abort.
    async fn apply_stored_filters(tmc: &mut Tmc4671Spi<TmcSpiDevice>) {
        use core::sync::atomic::Ordering::Relaxed;
        use foci_tmc4671::registers::*;

        let pwm_maxcnt = TMC_CONFIG.pwm_maxcnt;

        // Velocity biquad.
        let filter_hz = VELOCITY_FILTER_HZ.load(Relaxed);
        if filter_hz > 0
            && let Some(coeffs) = foci_firmware::biquad::butterworth_lowpass(filter_hz, pwm_maxcnt)
        {
            let _ = tmc
                .write_sub_register(CONFIG_BIQUAD_V_A_1, coeffs.a_1 as u32)
                .await;
            let _ = tmc
                .write_sub_register(CONFIG_BIQUAD_V_A_2, coeffs.a_2 as u32)
                .await;
            let _ = tmc
                .write_sub_register(CONFIG_BIQUAD_V_B_0, coeffs.b_0 as u32)
                .await;
            let _ = tmc
                .write_sub_register(CONFIG_BIQUAD_V_B_1, coeffs.b_1 as u32)
                .await;
            let _ = tmc
                .write_sub_register(CONFIG_BIQUAD_V_B_2, coeffs.b_2 as u32)
                .await;
            let _ = tmc.write_sub_register(CONFIG_BIQUAD_V_ENABLE, 1).await;
        }

        // Torque biquad.
        let filter_hz = TORQUE_FILTER_HZ.load(Relaxed);
        if filter_hz > 0
            && let Some(coeffs) = foci_firmware::biquad::butterworth_lowpass(filter_hz, pwm_maxcnt)
        {
            let _ = tmc
                .write_sub_register(CONFIG_BIQUAD_T_A_1, coeffs.a_1 as u32)
                .await;
            let _ = tmc
                .write_sub_register(CONFIG_BIQUAD_T_A_2, coeffs.a_2 as u32)
                .await;
            let _ = tmc
                .write_sub_register(CONFIG_BIQUAD_T_B_0, coeffs.b_0 as u32)
                .await;
            let _ = tmc
                .write_sub_register(CONFIG_BIQUAD_T_B_1, coeffs.b_1 as u32)
                .await;
            let _ = tmc
                .write_sub_register(CONFIG_BIQUAD_T_B_2, coeffs.b_2 as u32)
                .await;
            let _ = tmc.write_sub_register(CONFIG_BIQUAD_T_ENABLE, 1).await;
        }

        // Position biquad.
        let filter_hz = POSITION_FILTER_HZ.load(Relaxed);
        if filter_hz > 0
            && let Some(coeffs) = foci_firmware::biquad::butterworth_lowpass(filter_hz, pwm_maxcnt)
        {
            let _ = tmc
                .write_sub_register(CONFIG_BIQUAD_X_A_1, coeffs.a_1 as u32)
                .await;
            let _ = tmc
                .write_sub_register(CONFIG_BIQUAD_X_A_2, coeffs.a_2 as u32)
                .await;
            let _ = tmc
                .write_sub_register(CONFIG_BIQUAD_X_B_0, coeffs.b_0 as u32)
                .await;
            let _ = tmc
                .write_sub_register(CONFIG_BIQUAD_X_B_1, coeffs.b_1 as u32)
                .await;
            let _ = tmc
                .write_sub_register(CONFIG_BIQUAD_X_B_2, coeffs.b_2 as u32)
                .await;
            let _ = tmc.write_sub_register(CONFIG_BIQUAD_X_ENABLE, 1).await;
        }

        // Flux biquad.
        let filter_hz = FLUX_FILTER_HZ.load(Relaxed);
        if filter_hz > 0
            && let Some(coeffs) = foci_firmware::biquad::butterworth_lowpass(filter_hz, pwm_maxcnt)
        {
            let _ = tmc
                .write_sub_register(CONFIG_BIQUAD_F_A_1, coeffs.a_1 as u32)
                .await;
            let _ = tmc
                .write_sub_register(CONFIG_BIQUAD_F_A_2, coeffs.a_2 as u32)
                .await;
            let _ = tmc
                .write_sub_register(CONFIG_BIQUAD_F_B_0, coeffs.b_0 as u32)
                .await;
            let _ = tmc
                .write_sub_register(CONFIG_BIQUAD_F_B_1, coeffs.b_1 as u32)
                .await;
            let _ = tmc
                .write_sub_register(CONFIG_BIQUAD_F_B_2, coeffs.b_2 as u32)
                .await;
            let _ = tmc.write_sub_register(CONFIG_BIQUAD_F_ENABLE, 1).await;
        }
    }

    /// P2 software task: clear TriggerDomain timer_active after P1 stop.
    ///
    /// Spawned by `stepper_stop_from_p1()`. P1 performs the physical
    /// quiesce (CC1 disable, queue drain) immediately, then this task
    /// updates the authoritative flag so `reset_step_clock` and
    /// `arm_check` see the correct state.
    #[task(priority = 2)]
    async fn stop_check(_ctx: stop_check::Context) {
        with_trigger_domain(|td| {
            td.set_timer_active(0, false);
        });
    }

    /// P2 software task: arm TIM5 CC1 if motor is ready and queue has moves.
    ///
    /// Spawned by P1 after calibration/enable completion. P1 must never
    /// enter TriggerDomain — this task crosses the boundary at P2.
    #[task(priority = 2)]
    async fn arm_check(_ctx: arm_check::Context) {
        with_trigger_domain(|td| {
            if !MOTOR_ARMED.load(core::sync::atomic::Ordering::Relaxed) || td.timer_active(0) {
                return;
            }

            // SAFETY: P2 is the sole producer while the timer is inactive.
            let queue_has_moves = unsafe { !(*core::ptr::addr_of_mut!(STEP_QUEUE)).is_empty() };
            if queue_has_moves {
                arm_step_timer(read_tim5().wrapping_add(1));
                td.set_timer_active(0, true);
            }
        });
    }

    /// TIM5 CC1 step timer interrupt.
    ///
    /// Dequeues moves from SPSC consumer, toggles STEP GPIO, schedules
    /// next compare. Same priority as USB OTG and fault EXTI (P3).
    #[task(binds = TIM5, priority = 3)]
    fn step_timer_irq(_ctx: step_timer_irq::Context) {
        use core::sync::atomic::Ordering::Relaxed;
        use foci_klipper::stepper::StepEventResult;

        let tim5 = embassy_stm32::pac::TIM5;

        // --- CC2: trigger scheduler (process before CC1) ---
        if tim5.sr().read().ccif(1) && tim5.dier().read().ccie(1) {
            tim5.sr().modify(|w| w.set_ccif(1, false));
            let mut need_report = false;
            with_trigger_domain(|td| {
                use foci_klipper::scheduler::SchedulerEventKind;
                let now = read_tim5();
                let due_indices = td.scheduler.find_due(now);
                // Collect (kind, wake_clock) pairs before mutating scheduler
                let mut due_events: heapless::Vec<(SchedulerEventKind, u32), 12> =
                    heapless::Vec::new();
                for &slot_idx in &due_indices {
                    if let Some(event) = td.scheduler.get(slot_idx) {
                        let _ = due_events.push((event.kind, event.wake_clock));
                    }
                    td.scheduler.remove(slot_idx);
                }

                for &(kind, wake_clock) in &due_events {
                    match kind {
                        SchedulerEventKind::EndstopSample(endstop_idx) => {
                            use foci_klipper::endstop::{EndstopPhase, EndstopSampleResult};
                            let pin_id = ENDSTOP_PIN_IDS[endstop_idx].load(Relaxed);
                            let read_pin = || -> bool {
                                if pin_id != 0xFF {
                                    gpio_read_pin(pin_id)
                                } else {
                                    false
                                }
                            };

                            let phase = td.endstops[endstop_idx].phase();
                            let result = match phase {
                                EndstopPhase::Scanning => {
                                    td.endstops[endstop_idx].scan(read_pin(), wake_clock)
                                }
                                EndstopPhase::Oversampling => {
                                    td.endstops[endstop_idx].oversample(read_pin(), wake_clock)
                                }
                            };

                            match result {
                                EndstopSampleResult::EnterOversamplingNow => {
                                    // Second GPIO read (C double-read)
                                    match td.endstops[endstop_idx]
                                        .oversample(read_pin(), wake_clock)
                                    {
                                        EndstopSampleResult::Triggered { trsync_oid, reason } => {
                                            td.trigger(trsync_oid, reason, now);
                                            need_report = true;
                                        }
                                        EndstopSampleResult::Reschedule { next_clock } => {
                                            td.scheduler.schedule(
                                                foci_klipper::scheduler::SchedulerEvent {
                                                    wake_clock: next_clock,
                                                    kind: SchedulerEventKind::EndstopSample(
                                                        endstop_idx,
                                                    ),
                                                },
                                            );
                                        }
                                        _ => {}
                                    }
                                }
                                EndstopSampleResult::Triggered { trsync_oid, reason } => {
                                    td.trigger(trsync_oid, reason, now);
                                    need_report = true;
                                }
                                EndstopSampleResult::Reschedule { next_clock } => {
                                    td.scheduler.schedule(
                                        foci_klipper::scheduler::SchedulerEvent {
                                            wake_clock: next_clock,
                                            kind: SchedulerEventKind::EndstopSample(endstop_idx),
                                        },
                                    );
                                }
                                EndstopSampleResult::Inactive => {}
                            }
                        }
                        SchedulerEventKind::TrSyncExpire(trsync_idx) => {
                            // The scheduler event already confirms the deadline
                            // has passed. Call td.trigger() directly — do NOT call
                            // check_timeout() which internally triggers the
                            // TrSyncRuntime, causing td.trigger() to see
                            // AlreadyTriggered and skip stop signal dispatch.
                            let oid = td
                                .trsync_by_oid
                                .iter()
                                .find(|&(_, &idx)| idx == trsync_idx)
                                .map(|(&oid, _)| oid);
                            if let Some(oid) = oid {
                                let expire_reason = td.trsync[trsync_idx].expire_reason();
                                td.trigger(oid, expire_reason, now);
                                need_report = true;
                            }
                        }
                        SchedulerEventKind::TrSyncReport(trsync_idx) => {
                            let oid = td
                                .trsync_by_oid
                                .iter()
                                .find(|&(_, &idx)| idx == trsync_idx)
                                .map(|(&oid, _)| oid);
                            if let Some(oid) = oid
                                && td.trsync[trsync_idx].can_trigger()
                            {
                                td.trsync[trsync_idx].set_liveness_report(oid, now);
                                need_report = true;
                                // Advance report_clock by report_ticks (stable cadence)
                                let report_ticks = td.trsync[trsync_idx].report_ticks();
                                if report_ticks > 0 {
                                    let next_report = td.trsync[trsync_idx]
                                        .report_clock()
                                        .wrapping_add(report_ticks);
                                    td.trsync[trsync_idx].set_report_clock(next_report);
                                    td.scheduler.schedule(
                                        foci_klipper::scheduler::SchedulerEvent {
                                            wake_clock: next_report,
                                            kind: SchedulerEventKind::TrSyncReport(trsync_idx),
                                        },
                                    );
                                }
                                // After trigger, no more liveness reports — don't reschedule
                            }
                        }
                    }
                }
                td.reprogram_cc2();
            });
            if need_report {
                REPORT_PENDING.store(true, Relaxed);
                report_task::spawn().ok();
            }
        }

        // --- CC1: step timer (existing logic) ---
        // Skip CC1 if the flag is not set, or if CC2 trigger processing
        // disabled CC1 (stepper stop) — prevents one extra step after trigger.
        if !tim5.sr().read().ccif(0) || !tim5.dier().read().ccie(0) {
            return;
        }
        // Clear CC1 interrupt flag
        tim5.sr().modify(|w| w.set_ccif(0, false));

        // Check gates
        if FAULT_ACTIVE.load(Relaxed) {
            return;
        }
        if !MOTOR_ARMED.load(Relaxed) {
            return;
        }

        // SAFETY: ISR_STEPPER is only accessed from this P3 ISR.
        // P3 tasks cannot preempt each other on Cortex-M.
        let stepper = unsafe { &mut *core::ptr::addr_of_mut!(ISR_STEPPER) };

        let gpio_b = embassy_stm32::pac::GPIOB;
        let gpio_a = embassy_stm32::pac::GPIOA;

        loop {
            if !stepper.has_active_segment() {
                // Need a new move from the queue
                // SAFETY: P3 is the sole consumer of the queue.
                // P3 ISRs cannot preempt each other on Cortex-M.
                match unsafe { (*core::ptr::addr_of_mut!(STEP_QUEUE)).dequeue() } {
                    Some(m) => {
                        let dir_changed = stepper.load_move(m);
                        STEP_INTERVAL.store(stepper.current_interval(), Relaxed);
                        let dir_val = if stepper.dir_state() {
                            embassy_stm32::pac::gpio::vals::Odr::HIGH
                        } else {
                            embassy_stm32::pac::gpio::vals::Odr::LOW
                        };
                        gpio_a.odr().modify(|w| w.set_odr(0, dir_val));
                        STEP_DIRECTION.store(stepper.dir_state(), Relaxed);
                        if dir_changed {
                            FEEDFORWARD_RESET.store(true, Relaxed);
                        }
                    }
                    None => {
                        // Queue empty: disarm timer, go idle
                        tim5.dier().modify(|w| w.set_ccie(0, false));
                        STEP_INTERVAL.store(0, Relaxed);
                        PID_ERROR_SUMS_CLEAR.store(true, Relaxed);
                        with_trigger_domain(|td| {
                            td.set_timer_active(0, false);
                        });
                        return;
                    }
                }

                // Check deadline for the first step of this segment
                let now = tim5.cnt().read();
                let scheduled = stepper.next_step_clock();
                match stepper.check_deadline(scheduled, now, CLOCK_FREQ) {
                    Ok(compare) => {
                        let immediate = compare != scheduled;
                        if immediate {
                            stepper.rebase_next_step_clock(compare);
                        }
                        tim5.ccr(0).write_value(compare);
                        if !immediate {
                            return;
                        }
                    }
                    Err(_) => {
                        tim5.dier().modify(|w| w.set_ccie(0, false));
                        with_trigger_domain(|td| {
                            td.set_timer_active(0, false);
                        });
                        TMC_DISABLE_SIGNAL.signal(());
                        defmt::error!("Stepper too far in past");
                        return;
                    }
                }
            } else {
                // Active segment: execute one timer event (step or unstep).
                //
                // Full-mode scheduling (STEPPER_STEP_BOTH_EDGE=0): each motor
                // step is a pair of events. Event 1 drives STEP HIGH (step
                // phase, TMC4671 sees a rising edge and advances its counter);
                // event 2 drives STEP LOW (unstep phase, no-op for the
                // TMC4671). STEPPER_POSITION only advances on the step phase
                // so the firmware counter matches the TMC4671's view of
                // position. Matches Klipper C `stepper_event_full`.
                {
                    use embassy_stm32::pac::gpio::vals::Idr;
                    let is_high = gpio_b.idr().read().idr(11) == Idr::HIGH;
                    gpio_b.bsrr().write(|w| {
                        if is_high {
                            w.set_br(11, true);
                        } else {
                            w.set_bs(11, true);
                        }
                    });
                }

                match stepper.step_event() {
                    StepEventResult::ScheduleNext { clock, phase } => {
                        // Rising edge (LOW → HIGH) → TMC4671 advances, so the
                        // firmware counter advances too. Falling edge (HIGH →
                        // LOW) is the second half of the pulse and does not
                        // change the driver's position.
                        use foci_klipper::stepper::StepPhase;
                        let is_step = matches!(phase, StepPhase::Step);
                        if is_step {
                            let dir = if stepper.dir_state() { 1i32 } else { -1i32 };
                            STEPPER_POSITION.fetch_add(dir, Relaxed);
                        }
                        STEP_INTERVAL.store(stepper.current_interval(), Relaxed);
                        let now = tim5.cnt().read();
                        match stepper.check_deadline(clock, now, CLOCK_FREQ) {
                            Ok(compare) => {
                                let immediate = compare != clock;
                                // Only rebase the cadence anchor when the
                                // scheduled event IS the next rising edge
                                // (Unstep phase returns `next_step_clock`).
                                // For Step phase `clock` is the pulse-close
                                // time, only STEP_PULSE_TICKS past the
                                // rising edge — it is almost always in the
                                // past by the time we read TIM5 here, so
                                // rebasing would drag every subsequent
                                // rising edge forward by the pulse-close
                                // latency and stretch the step interval.
                                // Leaving the anchor untouched lets the
                                // following Unstep event schedule the next
                                // rising edge at `original_step_clock +
                                // interval` as intended.
                                if immediate && !is_step {
                                    stepper.rebase_next_step_clock(compare);
                                }
                                tim5.ccr(0).write_value(compare);
                                if !immediate {
                                    return;
                                }
                                continue;
                            }
                            Err(_) => {
                                tim5.dier().modify(|w| w.set_ccie(0, false));
                                with_trigger_domain(|td| {
                                    td.set_timer_active(0, false);
                                });
                                TMC_DISABLE_SIGNAL.signal(());
                                defmt::error!("Stepper too far in past");
                                return;
                            }
                        }
                    }
                    StepEventResult::NeedMove => {
                        // Final unstep of the segment: the GPIO toggle above
                        // already drove STEP LOW, leaving the pin in the
                        // quiescent state before we dequeue the next move.
                        continue;
                    }
                }
            }
        }
    }

    /// USB OTG FS interrupt handler. Masks interrupt sources and wakes usb_task.
    ///
    /// Priority 3: must be above P2 protocol tasks so the ISR always runs
    /// immediately. The handler only masks and signals — no FIFO access.
    #[task(binds = OTG_FS, priority = 3)]
    fn otg_fs_irq(_ctx: otg_fs_irq::Context) {
        crate::interrupts::otg_fs_irq();
    }

    /// TMC4671 STATUS pin interrupt (PE8, EXTI line 8).
    ///
    /// Fires when any STATUS_MASK-enabled flag is asserted (not_PLL_locked,
    /// adc_i_clipped). Signals immediate motor disable; tmc_control reads
    /// STATUS_FLAGS on next tick to log the cause.
    #[task(binds = EXTI9_5, priority = 3)]
    fn tmc_status_irq(_ctx: tmc_status_irq::Context) {
        crate::interrupts::tmc_status_irq(&FAULT_ACTIVE, &TMC_DISABLE_SIGNAL);
    }

    /// Single USB task replacing usb_pump, usb_receive, usb_send, usb_control.
    ///
    /// Handles EP0 setup, bulk RX, protocol dispatch, and bulk TX in one pass.
    /// No scheduling gap between receiving commands and sending acks.
    /// Katapult 1200-baud DTR detection is handled by klipper-usb internally.
    #[task(priority = 2, shared = [app_state])]
    async fn usb_task(mut ctx: usb_task::Context) {
        loop {
            crate::usb_task::pump(|f| ctx.shared.app_state.lock(f), &REPORT_PENDING).await;
        }
    }

    /// Send pending trsync_state reports outside the TriggerDomain lock.
    ///
    /// Spawned by the TIM5 CC2 ISR when any trsync has a pending report.
    /// Drains all pending snapshots in one brief critical section, then
    /// sends replies at P2 without holding the lock.
    #[task(priority = 2)]
    async fn report_task(_ctx: report_task::Context) {
        crate::heartbeat::emit_trigger_reports(
            |snaps| {
                with_trigger_domain(|td| {
                    for ts in td.trsync.iter_mut() {
                        if let Some(snap) = ts.take_report() {
                            let _ = snaps.push(snap);
                        }
                    }
                });
            },
            &REPORT_PENDING,
        );
    }

    /// Periodic stats heartbeat sent to Klipper.
    ///
    /// Klipper requires regular stats messages to confirm the MCU is alive.
    /// Sends `stats count=%u sum=%u sumsq=%u` every 5 seconds with basic
    /// CPU usage metrics. Also kicks the IWDG watchdog.
    #[task(priority = 1, local = [wdg])]
    async fn stats(ctx: stats::Context) {
        let mut accumulator = crate::heartbeat::StatsAccumulator::new(read_tim5());
        loop {
            Mono::delay(5000.millis()).await;
            crate::heartbeat::stats_tick(
                &mut accumulator,
                read_tim5(),
                &STATS_SEND_TIME,
                &STATS_SEND_TIME_HIGH,
                ctx.local.wdg,
            );
        }
    }

    /// Heartbeat task: toggles the system LED every 500 ms.
    #[task(priority = 1, local = [led])]
    async fn blink(ctx: blink::Context) {
        loop {
            crate::heartbeat::toggle_led(ctx.local.led);
            Mono::delay(500.millis()).await;
        }
    }
}

::ankyra::ankyra_config! {
    transport = crate::transport::TRANSPORT_OUTPUT: crate::transport::BufferTransportOutput,
    context = &'ctx mut crate::firmware_ctx::FirmwareCtx<'ctx>,
    providers = [crate::ankyra::provider::BOARD_PROVIDER],
    static_strings = [
        "Can't reset time when stepper active",
        "Command request",
        "Invalid TMC channel",
        "Motor enable/disable lost (queue full)",
        "Shutdown cleared when not shutdown",
        "TMC request validation failed",
        "TMC4671 SPI failed at boot",
        "TMC4671 chip ID mismatch",
        "TMC4671 fault",
        "TMC4671: ADC current clipped (overcurrent)",
        "TMC4671: PLL not locked (clock lost)",
        "config_digital_out failed",
        "config_endstop failed",
        "config_reset only available when shutdown",
        "config_stepper failed",
        "config_trsync failed",
        "endstop_home failed",
        "endstop_query_state failed",
        "queue_digital_out failed",
        "queue_step failed",
        "stepper_stop_on_trigger failed",
        "stepper_stop_on_trigger: unknown stepper",
        "tmc_set_current lost (queue full)",
        "tmc_set_encoder lost (queue full)",
        "tmc_set_encoder_dir lost (queue full)",
        "tmc_set_pid_gains lost (queue full)",
        "tmc_set_position_gains lost (queue full)",
        "trsync_set_timeout failed",
        "trsync_start failed",
        "trsync_trigger failed",
        "update_digital_out failed",
    ],
    app = "foci",
    version = env!("CARGO_PKG_VERSION"),
    build_versions = "",
    license = "MIT OR Apache-2.0",
}
