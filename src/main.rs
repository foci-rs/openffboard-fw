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
mod board_config;
mod board_init;
mod commissioning_backend;
mod firmware_ctx;
mod heartbeat;
mod helpers;
mod interrupts;
mod platform_impl;
mod state;
mod tmc_dispatch;
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
    use rtic_monotonics::fugit::ExtU64;
    use rtic_monotonics::stm32::prelude::*;
    use rtic_sync::channel::Receiver;

    use crate::app_types::{CycleDelay, TmcSpiDevice};
    use crate::helpers::lifecycle::*;
    use crate::helpers::stepper::*;
    use crate::helpers::stepper_diag::*;
    use crate::helpers::trsync::*;
    use crate::state::stepper::*;
    use crate::state::tmc::*;
    use crate::state::trace::*;
    use crate::trigger_domain::{
        TRIGGER_DOMAIN, TRIGGER_TIMER_MIN_LEAD_TICKS, TriggerDomain, with_trigger_domain,
    };
    use foci_firmware::diagnostic::StepperEventReason;
    use foci_firmware::state::lifecycle::*;
    use foci_firmware::state::tmc::{TMC_DISABLE_SIGNAL, TMC_QUEUE_FLUSH};
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

    use foci_firmware::tmc::{TmcRequest, TmcRequestKind};

    use crate::board_config::TMC_CONFIG;
    use crate::commissioning_backend::OpenFFBoardBackend;

    const HOLD_SETTLE_DELAY_MS: u8 = 100;

    fn tmc_state() -> &'static foci_firmware::state::tmc::TmcChannelState {
        foci_firmware::state::tmc::tmc_channel0()
    }

    fn stepper_state() -> &'static foci_firmware::state::stepper::StepperChannelState {
        foci_firmware::state::stepper::stepper_channel0()
    }

    fn dispatch_state<'a>(
        calibrated: &'a mut bool,
        motor_enabled: &'a mut bool,
        stored_gains: &'a mut Option<foci_firmware::pid::InnerLoopGains>,
    ) -> foci_firmware::commission_dispatch::DispatchState<'a> {
        foci_firmware::commission_dispatch::DispatchState::new(
            tmc_state(),
            stepper_state(),
            calibrated,
            motor_enabled,
            stored_gains,
        )
    }

    fn openffboard_backend<'a>(
        tmc: &'a mut Tmc4671Spi<TmcSpiDevice>,
        enable_pin: &'a mut Output<'static>,
    ) -> OpenFFBoardBackend<'a> {
        OpenFFBoardBackend {
            tmc,
            enable_pin,
            tmc_state: tmc_state(),
        }
    }

    // TriggerDomain, PHYSICAL_STEPPER_INDICES, TRIGGER_TIMER_MIN_LEAD_TICKS,
    // TRIGGER_DOMAIN, with_trigger_domain(), and stop_physical_stepper_no_lock()
    // live in crate::trigger_domain and are re-imported below.

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

        // Initialize trace buffers in CCM RAM
        unsafe {
            // Use addr_of_mut! to avoid creating a reference to the mutable static
            let ptr =
                core::ptr::addr_of_mut!(TRACE_BUFS).cast::<foci_firmware::trace::TraceBuffer>();
            // Validate address is in CCM range
            let addr = ptr as usize;
            defmt::assert!(
                (0x1000_0000..0x1001_0000).contains(&addr),
                "TRACE_BUFS not in CCM RAM: {:#010x}",
                addr
            );
            defmt::info!(
                "trace: buffers at {:#010x}, size {} bytes",
                addr,
                core::mem::size_of::<foci_firmware::trace::TraceBuffer>()
                    * crate::board_config::TMC_CHANNEL_COUNT as usize
            );
            for channel in 0..crate::board_config::TMC_CHANNEL_COUNT as usize {
                core::ptr::write(ptr.add(channel), {
                    let mut buf = core::mem::zeroed::<foci_firmware::trace::TraceBuffer>();
                    // reset() sets magic and all fields to valid initial state
                    buf.reset();
                    buf
                });
            }
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

    use crate::ankyra::replies::{emit_phase_reply, emit_terminal_reply};
    #[cfg(feature = "dev")]
    use crate::helpers::dev::{do_test_spin, tmc_enter_openloop};
    use crate::tmc_dispatch::handle_tmc_request;

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
                        if crate::firmware_ctx::enter_shutdown(state) {
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
                    if crate::firmware_ctx::enter_shutdown(state) {
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
        // Position is read from stepper_state().position_target atomic (updated
        // by usb_receive at P2) — no shared resource lock needed.
        loop {
            use embassy_futures::select::{Either, select};

            // On shutdown, perform one-shot motor safe-state then idle until
            // shutdown clears. This catches USB disconnect / host crash where
            // no further commands arrive but the TMC4671 remains energized.
            if SHUTDOWN_ACTIVE.load(core::sync::atomic::Ordering::Relaxed) {
                if !shutdown_handled {
                    commission_engine.take();
                    {
                        let mut backend = openffboard_backend(tmc, enable_pin);
                        let mut state =
                            dispatch_state(&mut calibrated, &mut motor_enabled, &mut stored_gains);
                        // disarm_motor clears tmc_state().motor_armed, motor_enabled, calibrated,
                        // and runs restore_safe_state_via_backend (best-effort SPI writes).
                        foci_firmware::tmc_ops::disarm_motor(&mut backend, &mut state).await;
                    }
                    tmc_state()
                        .commission_in_flight
                        .store(false, core::sync::atomic::Ordering::Relaxed);
                    stepper_stop_from_p1(0);
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
                    let mut backend = openffboard_backend(tmc, enable_pin);
                    let mut state =
                        dispatch_state(&mut calibrated, &mut motor_enabled, &mut stored_gains);
                    foci_firmware::tmc_ops::disarm_motor(&mut backend, &mut state).await;
                }
                tmc_state()
                    .commission_in_flight
                    .store(false, core::sync::atomic::Ordering::Relaxed);
                // Clear commissioned outer configuration so the next closed-
                // loop entry uses defaults, not stale commissioned values.
                use core::sync::atomic::Ordering::Relaxed as R;
                tmc_state().velocity_limit_cfg.store(500_000, R);
                tmc_state().position_p_value.store(640, R);
                tmc_state().position_i_value.store(0, R);
                tmc_state().velocity_p_value.store(1152, R);
                tmc_state().velocity_i_value.store(0, R);
                tmc_state().velocity_filter_hz.store(0, R);
                tmc_state().position_filter_hz.store(0, R);
                tmc_state().flux_filter_hz.store(0, R);
                tmc_state().torque_filter_hz.store(0, R);
                // Reset hardware stepper state.
                stepper_stop_from_p1(0);
                stepper_state()
                    .position
                    .store(0, core::sync::atomic::Ordering::Relaxed);
                FAULT_ACTIVE.store(false, core::sync::atomic::Ordering::Relaxed);
                defmt::info!("TMC request queue flushed after config_reset");
            }

            // Tick the commissioning engine if one is active.
            // Each step() does at most 1ms of work, then returns.
            if commission_engine.is_some() {
                // Capture deadline before engine.step() so SPI time consumed
                // by the engine reduces the remaining trace budget.
                let engine_tick_deadline =
                    Mono::now() + fugit::Duration::<u64, 1, 1_000_000>::millis(1);

                // Sequential borrows: engine.step() consumes the backend
                // first, then drive_commission_result re-borrows it for
                // post-processing. Two scoped constructions keep the
                // borrows non-overlapping.
                let result = {
                    let mut backend = openffboard_backend(tmc, enable_pin);
                    commission_engine.as_mut().unwrap().step(&mut backend).await
                };

                let outcome = {
                    let mut backend = openffboard_backend(tmc, enable_pin);
                    let mut state =
                        dispatch_state(&mut calibrated, &mut motor_enabled, &mut stored_gains);
                    foci_firmware::commission_dispatch::drive_commission_result(
                        result,
                        &mut commission_engine,
                        &mut backend,
                        &mut state,
                        TMC_CONFIG.pwm_maxcnt,
                    )
                    .await
                };

                use foci_firmware::commission_dispatch::CommissionOutcome;
                match outcome {
                    CommissionOutcome::Continue => {
                        // Trace capture during Continue tick — engine.step's
                        // SPI eats into the 1ms trace budget.
                        foci_firmware::trace::sample_tick(
                            tmc,
                            unsafe { trace_buf0() },
                            || {
                                engine_tick_deadline
                                    .checked_duration_since(Mono::now())
                                    .map(|d| d.to_micros())
                            },
                            || Mono::now() > engine_tick_deadline,
                            168,
                        )
                        .await;
                        Mono::delay(1.millis()).await;
                        continue;
                    }
                    CommissionOutcome::PhaseChanged {
                        oid,
                        phase_id,
                        engine_kind,
                    } => {
                        use foci_firmware::commissioning::types::EngineKind;
                        // Start fast trace on ClosedLoopEntry for Stage 1
                        // commission only (not Calibrate, not Selftest, not
                        // OuterTune).
                        if phase_id == foci_firmware::autotune::PhaseId::ClosedLoopEntry
                            && engine_kind == EngineKind::Commission
                            && !unsafe { trace_buf0() }.is_capturing()
                        {
                            unsafe {
                                trace_buf0().start(oid, foci_firmware::trace::PRESET_FAST);
                            }
                        }
                        unsafe { trace_buf0().set_phase(phase_id.to_u8()) };
                        emit_phase_reply(oid, phase_id);
                        Mono::delay(1.millis()).await;
                        continue;
                    }
                    CommissionOutcome::Terminal { reply, actions } => {
                        if actions.stop_trace {
                            unsafe { trace_buf0().stop() };
                        }
                        {
                            let mut state = dispatch_state(
                                &mut calibrated,
                                &mut motor_enabled,
                                &mut stored_gains,
                            );
                            apply_stepper_post_action(actions.stepper, &mut state);
                        }
                        emit_terminal_reply(reply);
                    }
                }
                // For Terminal cases, fall through to normal channel-receive processing.
            }

            match select(rx.recv(), Mono::delay(1.millis())).await {
                // Channel message received
                Either::First(Ok(req)) => match req {
                    TmcRequest { channel, .. } if channel != 0 => {
                        ctx.shared.app_state.lock(|state| {
                            if crate::firmware_ctx::enter_shutdown(state) {
                                ::ankyra::klipper_shutdown_from!(
                                    &mut crate::_ankyra_config::Sender,
                                    "Invalid TMC channel",
                                    read_tim5(),
                                );
                            }
                        });
                        continue;
                    }
                    TmcRequest {
                        kind: TmcRequestKind::SetEnable(enabled),
                        ..
                    } => {
                        if enabled {
                            if !calibrated {
                                // Auto-calibrate: Klipper may send enable before
                                // the Python module's foci_calibrate arrives due
                                // to queue_digital_out timing. Launch the
                                // commissioning engine to run ADC calibration
                                // asynchronously; the CalibrateDone handler will
                                // complete encoder alignment and closed-loop entry.
                                if !foci_firmware::tmc::auto_calibrate_on_enable_allowed(0) {
                                    defmt::warn!("Auto-calibrate rejected: host gate closed");
                                    continue;
                                }
                                if tmc_state()
                                    .commission_in_flight
                                    .swap(true, core::sync::atomic::Ordering::Relaxed)
                                {
                                    defmt::warn!("Auto-calibrate skipped: commission in flight");
                                    continue;
                                }
                                // TODO: hardcoded OID 0 — correct for single-motor
                                // OpenFFBoard but needs OID routing for multi-motor boards.
                                defmt::info!("Auto-calibrating on enable");
                                commission_engine = Some(
                                    foci_firmware::commission::CommissioningEngine::new_calibrate_for_tmc(
                                        0,
                                        tmc_state(),
                                    ),
                                );
                                // Don't set motor_enabled/tmc_state().motor_armed yet --
                                // the CalibrateDone handler does that after
                                // encoder alignment + closed-loop entry.
                                continue;
                            }
                            // Existing precondition: this arm runs only when
                            // calibrated == true (the !calibrated branch above
                            // continues to the auto-calibrate path).
                            if motor_enabled {
                                defmt::info!("Motor enable sync (already armed)");
                                continue;
                            }
                            {
                                let mut backend = openffboard_backend(tmc, enable_pin);
                                let mut state = dispatch_state(
                                    &mut calibrated,
                                    &mut motor_enabled,
                                    &mut stored_gains,
                                );
                                foci_firmware::tmc_ops::arm_motor_electrical(
                                    &mut backend,
                                    &mut state,
                                );
                                apply_stepper_post_action(
                                    foci_firmware::commission_dispatch::StepperPostAction::ArmPreservePosition,
                                    &mut state,
                                );
                            }

                            defmt::info!("Motor enabled (calibrated, step/dir armed)");
                        } else {
                            // Disable: stop stepping first, then kill motor electrically.
                            stepper_stop_from_p1(0);
                            {
                                let mut backend = openffboard_backend(tmc, enable_pin);
                                let mut state = dispatch_state(
                                    &mut calibrated,
                                    &mut motor_enabled,
                                    &mut stored_gains,
                                );
                                foci_firmware::tmc_ops::disarm_motor(&mut backend, &mut state)
                                    .await;
                            }

                            defmt::info!("TMC4671 motor disabled");
                        }
                    }
                    TmcRequest {
                        kind: TmcRequestKind::Calibrate { oid },
                        ..
                    } => {
                        if motor_enabled {
                            defmt::warn!("foci_calibrate rejected: motor already enabled");
                            tmc_state()
                                .commission_in_flight
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
                            foci_firmware::commission::CommissioningEngine::new_calibrate_for_tmc(
                                oid,
                                tmc_state(),
                            ),
                        );
                    }
                    #[cfg(feature = "dev")]
                    TmcRequest {
                        kind: TmcRequestKind::Init,
                        ..
                    } => {
                        defmt::info!("TMC4671 init: starting open-loop stepper config");
                        tmc_enter_openloop(tmc, enable_pin).await;
                    }
                    #[cfg(feature = "dev")]
                    TmcRequest {
                        kind: TmcRequestKind::TestSpin { revolutions },
                        ..
                    } => {
                        do_test_spin(tmc, revolutions).await;
                    }
                    TmcRequest {
                        kind:
                            TmcRequestKind::SetPidGains {
                                flux_p,
                                flux_i,
                                torque_p,
                                torque_i,
                            },
                        ..
                    } => {
                        let gains = foci_firmware::pid::InnerLoopGains {
                            flux_p,
                            flux_i,
                            torque_p,
                            torque_i,
                        };
                        stored_gains = Some(gains);
                        let (flux_reg, torque_reg) =
                            foci_firmware::tmc_ops::pack_inner_gain_registers(gains);
                        let flux_ok = tmc
                            .write_register(foci_tmc4671::registers::PID_FLUX_P_FLUX_I, flux_reg)
                            .await
                            .is_ok();
                        let torque_ok = tmc
                            .write_register(
                                foci_tmc4671::registers::PID_TORQUE_P_TORQUE_I,
                                torque_reg,
                            )
                            .await
                            .is_ok();
                        if !flux_ok || !torque_ok {
                            defmt::warn!("PID gains live write failed");
                        }
                        defmt::info!(
                            "PID gains received: flux P={} I={}, torque P={} I={}",
                            flux_p,
                            flux_i,
                            torque_p,
                            torque_i,
                        );
                    }
                    TmcRequest {
                        kind:
                            TmcRequestKind::SetPositionGains {
                                position_p,
                                position_i,
                                velocity_p,
                                velocity_i,
                            },
                        ..
                    } => {
                        tmc_state()
                            .position_p_value
                            .store(position_p, core::sync::atomic::Ordering::Relaxed);
                        tmc_state()
                            .position_i_value
                            .store(position_i, core::sync::atomic::Ordering::Relaxed);
                        tmc_state()
                            .velocity_p_value
                            .store(velocity_p, core::sync::atomic::Ordering::Relaxed);
                        tmc_state()
                            .velocity_i_value
                            .store(velocity_i, core::sync::atomic::Ordering::Relaxed);
                        let (velocity_reg, position_reg) =
                            foci_firmware::tmc_ops::pack_outer_gain_registers(
                                position_p, position_i, velocity_p, velocity_i,
                            );
                        let velocity_ok = tmc
                            .write_register(
                                foci_tmc4671::registers::PID_VELOCITY_P_VELOCITY_I,
                                velocity_reg,
                            )
                            .await
                            .is_ok();
                        let position_ok = tmc
                            .write_register(
                                foci_tmc4671::registers::PID_POSITION_P_POSITION_I,
                                position_reg,
                            )
                            .await
                            .is_ok();
                        if !velocity_ok || !position_ok {
                            defmt::warn!("Position gains live write failed");
                        }
                        defmt::info!(
                            "Position gains received: pos P={} I={}, vel P={} I={}",
                            position_p,
                            position_i,
                            velocity_p,
                            velocity_i,
                        );
                    }
                    TmcRequest {
                        kind: TmcRequestKind::SetVelocityFeedforward { enable, multiplier },
                        ..
                    } => {
                        defmt::info!(
                            "Velocity feedforward: {} multiplier={} (via queue)",
                            enable,
                            multiplier
                        );
                        // Feedforward flag is already set by the atomic in the
                        // command handler; this arm exists for exhaustiveness.
                    }
                    TmcRequest {
                        kind: TmcRequestKind::Commission { oid, profile },
                        ..
                    } => {
                        if motor_enabled {
                            tmc_state()
                                .commission_in_flight
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
                        let current_limit = tmc_state()
                            .current_limit_reg
                            .load(core::sync::atomic::Ordering::Relaxed);
                        if current_limit == 0 {
                            tmc_state()
                                .commission_in_flight
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
                            foci_firmware::commission::CommissioningEngine::new_commission_for_tmc(
                                oid,
                                tmc_state(),
                                foci_firmware::autotune::CommissionProfile::from_u8(profile),
                            ),
                        );
                        // Trace start deferred to ClosedLoopEntry phase transition
                        // (see PhaseChanged handler). Spec scopes Stage 1 trace to
                        // ClosedLoopEntry only — starting here would add SPI reads
                        // throughout inner diagnostics/electrical ID phases.
                    }
                    TmcRequest {
                        kind:
                            TmcRequestKind::Tune {
                                oid,
                                profile,
                                mode,
                                inner_lambda_us,
                                theta_e_us,
                                current_ringing_count,
                                current_bandwidth_hz,
                                tau_e_us,
                                tau_e_crosscheck_us,
                                tau_residual_permille,
                                inner_warning_flags,
                            },
                        ..
                    } => {
                        // Stage 2 requires the motor to be enabled and
                        // holding in closed-loop position mode (from Stage 1
                        // or a prior calibrate). Reject if not ready.
                        if !motor_enabled || !calibrated {
                            tmc_state()
                                .commission_in_flight
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
                        let current_limit = tmc_state()
                            .current_limit_reg
                            .load(core::sync::atomic::Ordering::Relaxed);
                        if current_limit == 0 {
                            tmc_state()
                                .commission_in_flight
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
                                velocity_p: tmc_state().velocity_p_value.load(Relaxed),
                                velocity_i: tmc_state().velocity_i_value.load(Relaxed),
                                position_p: tmc_state().position_p_value.load(Relaxed),
                                position_i: tmc_state().position_i_value.load(Relaxed),
                                velocity_limit: tmc_state().velocity_limit_cfg.load(Relaxed),
                                voltage_limit: 16000,
                                velocity_filter_hz: tmc_state().velocity_filter_hz.load(Relaxed),
                                torque_filter_hz: tmc_state().torque_filter_hz.load(Relaxed),
                                position_filter_hz: tmc_state().position_filter_hz.load(Relaxed),
                                flux_filter_hz: tmc_state().flux_filter_hz.load(Relaxed),
                            }
                        };
                        let outer_params = foci_firmware::commission::OuterTuneParams {
                            profile: foci_firmware::autotune::CommissionProfile::from_u8(profile),
                            mode: foci_firmware::autotune::OperatingMode::from_u8(mode),
                            inner_lambda_us,
                            theta_e_us,
                            current_ringing: current_ringing_count > 1,
                            current_bandwidth_hz,
                            tau_e_us,
                            tau_e_crosscheck_us,
                            tau_residual_permille,
                            inner_warning_flags,
                        };
                        commission_engine = Some(
                            foci_firmware::commission::CommissioningEngine::new_outer_for_tmc(
                                oid,
                                tmc_state(),
                                outer_params,
                                entry_snapshot,
                            ),
                        );
                        unsafe {
                            trace_buf0().start(oid, foci_firmware::trace::PRESET_FULL);
                            trace_buf0()
                                .set_phase(foci_firmware::autotune::PhaseId::MechanicalId.to_u8());
                        }
                    }
                    TmcRequest {
                        kind: TmcRequestKind::SelfTest { oid },
                        ..
                    } => {
                        if motor_enabled {
                            defmt::warn!("foci_selftest rejected: motor enabled");
                            tmc_state()
                                .commission_in_flight
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
                        commission_engine = Some(
                            foci_firmware::commission::CommissioningEngine::new_selftest_for_tmc(
                                oid,
                                tmc_state(),
                            ),
                        );
                    }
                    req @ TmcRequest {
                        kind: TmcRequestKind::SetEncoder { .. },
                        ..
                    } => {
                        if handle_tmc_request(req, tmc).await.is_err() {
                            ctx.shared.app_state.lock(|state| {
                                if crate::firmware_ctx::enter_shutdown(state) {
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
                            tmc_state()
                                .step_width_value
                                .load(core::sync::atomic::Ordering::Relaxed)
                                .unsigned_abs(),
                            CLOCK_FREQ,
                            TMC_CONFIG.pwm_freq_hz(),
                        );
                    }
                    other => {
                        if handle_tmc_request(other, tmc).await.is_err() {
                            ctx.shared.app_state.lock(|state| {
                                if crate::firmware_ctx::enter_shutdown(state) {
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
                stepper_stop_from_p1(0);
                FAULT_ACTIVE.store(false, core::sync::atomic::Ordering::Relaxed);

                motor_enabled = false;
                calibrated = false;
                enable_pin.set_low();
                // Read STATUS_FLAGS to identify the fault cause.
                let flags = tmc
                    .read_register(foci_tmc4671::registers::STATUS_FLAGS)
                    .await
                    .unwrap_or(0);
                record_stepper_event(
                    StepperEventReason::TmcDisableSignal,
                    0,
                    read_tim5(),
                    UNKNOWN_QUEUE_LEN,
                    flags,
                    0,
                );
                defmt::error!("TMC4671 emergency disable: STATUS={:#010x}", flags);
                // Report shutdown to Klipper with a specific reason.
                const NOT_PLL_LOCKED: u32 = 1 << 19;
                const ADC_I_CLIPPED: u32 = 1 << 26;
                ctx.shared.app_state.lock(|state| {
                    if flags & NOT_PLL_LOCKED != 0 {
                        if crate::firmware_ctx::enter_shutdown(state) {
                            ::ankyra::klipper_shutdown_from!(
                                &mut crate::_ankyra_config::Sender,
                                "TMC4671: PLL not locked (clock lost)",
                                read_tim5(),
                            );
                        }
                    } else if flags & ADC_I_CLIPPED != 0 {
                        if crate::firmware_ctx::enter_shutdown(state) {
                            ::ankyra::klipper_shutdown_from!(
                                &mut crate::_ankyra_config::Sender,
                                "TMC4671: ADC current clipped (overcurrent)",
                                read_tim5(),
                            );
                        }
                    } else if crate::firmware_ctx::enter_shutdown(state) {
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
            // - tmc_state().velocity_feedforward enabled (host config)
            // - commission_engine is None (not commissioning)
            // - homing not active
            let ff_active = motor_enabled
                && velocity_scale != 0
                && tmc_state()
                    .velocity_feedforward
                    .load(core::sync::atomic::Ordering::Relaxed)
                && commission_engine.is_none();

            if ff_active {
                let homing_active = HOMING_STATE.load(core::sync::atomic::Ordering::Relaxed) != 0;
                let reset = tmc_state()
                    .feedforward_reset
                    .swap(false, core::sync::atomic::Ordering::Relaxed);

                let offset = if homing_active || reset {
                    0i32
                } else {
                    let interval = stepper_state()
                        .interval
                        .load(core::sync::atomic::Ordering::Relaxed);
                    let forward = stepper_state()
                        .direction
                        .load(core::sync::atomic::Ordering::Relaxed);
                    let multiplier = tmc_state()
                        .velocity_feedforward_multiplier
                        .load(core::sync::atomic::Ordering::Relaxed);
                    foci_firmware::tmc::compute_velocity_offset_scaled(
                        velocity_scale,
                        interval,
                        forward,
                        multiplier,
                    )
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
            foci_firmware::trace::sample_tick(
                tmc,
                unsafe { trace_buf0() },
                || {
                    tick_deadline
                        .checked_duration_since(Mono::now())
                        .map(|d| d.to_micros())
                },
                || Mono::now() > tick_deadline,
                168,
            )
            .await;

            if tmc_state()
                .pid_error_sums_clear
                .swap(false, core::sync::atomic::Ordering::Relaxed)
            {
                pid_error_clear_countdown_ms = HOLD_SETTLE_DELAY_MS;
            }
            let timer_active = stepper_state()
                .timer_active_mirror
                .load(core::sync::atomic::Ordering::Relaxed);
            if motor_enabled
                && commission_engine.is_none()
                && !timer_active
                && pid_error_clear_countdown_ms > 0
            {
                pid_error_clear_countdown_ms -= 1;
                if pid_error_clear_countdown_ms == 0 {
                    if foci_firmware::tmc_ops::settle_hold_and_clear_pid_errors(tmc, tmc_state())
                        .await
                        .is_err()
                    {
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
                        let pos_p = tmc_state().position_p_value.load(Relaxed);
                        let reg = (pos_p as u32) << 16; // P in upper half, I=0
                        let _ = tmc
                            .write_register(foci_tmc4671::registers::PID_POSITION_P_POSITION_I, reg)
                            .await;
                        HOMING_STATE.store(2, Relaxed);
                        defmt::info!("homing: position I suppressed");
                    }
                    // Suppressing: advance to RestorePending once a trigger
                    // has fired via the HOMING_TRIGGERED sentinel (set by
                    // TriggerDomain on first trigger, 0xFF = no trigger
                    // pending). P1 never enters TriggerDomain per spec. If
                    // host's trsync_trigger runs before P1 polls,
                    // trsync_trigger advances HOMING_STATE 2->3 directly.
                    2 if HOMING_TRIGGERED.load(Relaxed) != 0xFF => {
                        HOMING_STATE.store(3, Relaxed);
                    }
                    3 => {
                        // RestorePending: restore position I and clear integrator
                        let pos_p = tmc_state().position_p_value.load(Relaxed);
                        let pos_i = tmc_state().position_i_value.load(Relaxed);
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

    /// P2 software task: clear TriggerDomain timer_active after P1 stop.
    ///
    /// Spawned by `stepper_stop_from_p1()`. P1 performs the physical
    /// quiesce immediately, then this task cancels the stale stepper timer
    /// and updates the authoritative flag so `reset_step_clock` and
    /// `arm_check` see the correct state.
    #[task(priority = 2)]
    async fn stop_check(_ctx: stop_check::Context) {
        with_trigger_domain(|td| {
            td.scheduler.cancel(|kind| {
                matches!(
                    kind,
                    foci_klipper::scheduler::SchedulerEventKind::Stepper(0)
                )
            });
            td.set_timer_active(0, false);
            td.reprogram_timer();
        });
    }

    /// P2 software task: arm TIM5 scheduler if motor is ready and queue has moves.
    ///
    /// Spawned by P1 after calibration/enable completion. P1 must never
    /// enter TriggerDomain — this task crosses the boundary at P2.
    #[task(priority = 2)]
    async fn arm_check(_ctx: arm_check::Context) {
        with_trigger_domain(|td| {
            if !tmc_state()
                .motor_armed
                .load(core::sync::atomic::Ordering::Relaxed)
                || td.timer_active(0)
            {
                return;
            }

            // SAFETY: P2 is the sole producer while the timer is inactive.
            let queue_has_moves = unsafe {
                !step_queue(0)
                    .expect("OpenFFBoard channel 0 exists")
                    .is_empty()
            };
            if queue_has_moves {
                let now = read_tim5();
                crate::state::stepper_diag::record_timer_activation(0, now);
                td.scheduler.cancel(|kind| {
                    matches!(
                        kind,
                        foci_klipper::scheduler::SchedulerEventKind::Stepper(0)
                    )
                });
                td.scheduler
                    .schedule(foci_klipper::scheduler::SchedulerEvent {
                        wake_clock: now.wrapping_add(1),
                        kind: foci_klipper::scheduler::SchedulerEventKind::Stepper(0),
                    });
                td.set_timer_active(0, true);
                td.reprogram_timer();
            }
        });
    }

    /// Maximum continuous TIM5 scheduler dispatch before deferring.
    ///
    /// Matches Klipper's repeat guard shape: process bursts in one IRQ, but
    /// avoid monopolizing P3 if a pathological stream stays immediately due.
    const TIM5_SCHEDULER_REPEAT_TICKS: u32 = CLOCK_FREQ / 10_000;

    fn schedule_stepper_timer(td: &mut TriggerDomain, stepper_idx: usize, clock: u32) {
        td.scheduler
            .schedule(foci_klipper::scheduler::SchedulerEvent {
                wake_clock: clock,
                kind: foci_klipper::scheduler::SchedulerEventKind::Stepper(stepper_idx),
            });
    }

    fn process_stepper_timer(td: &mut TriggerDomain, stepper_idx: usize) {
        use core::sync::atomic::Ordering::Relaxed;
        use foci_klipper::stepper::{StepEventResult, StepPhase};

        if stepper_idx != 0 {
            return;
        }
        if FAULT_ACTIVE.load(Relaxed) || !tmc_state().motor_armed.load(Relaxed) {
            td.set_timer_active(0, false);
            stepper_state().interval.store(0, Relaxed);
            tmc_state().pid_error_sums_clear.store(true, Relaxed);
            return;
        }

        // SAFETY: ISR stepper state is only accessed from this P3 ISR or from
        // stop paths that first gate/cancel the stepper timer.
        let stepper = unsafe { isr_stepper(0).expect("OpenFFBoard channel 0 exists") };

        let gpio_b = embassy_stm32::pac::GPIOB;
        let gpio_a = embassy_stm32::pac::GPIOA;

        loop {
            if !stepper.has_active_segment() {
                // Need a new move from the queue.
                // SAFETY: P3 is the sole consumer while motor_armed is true.
                let loaded_steps = match unsafe {
                    step_queue(0)
                        .expect("OpenFFBoard channel 0 exists")
                        .dequeue()
                } {
                    Some(m) => {
                        let loaded_steps = u32::from(m.count);
                        let dir_changed = stepper.load_move(m);
                        stepper_state()
                            .interval
                            .store(stepper.current_interval(), Relaxed);
                        let dir_val = if stepper.dir_state() {
                            embassy_stm32::pac::gpio::vals::Odr::HIGH
                        } else {
                            embassy_stm32::pac::gpio::vals::Odr::LOW
                        };
                        gpio_a.odr().modify(|w| w.set_odr(0, dir_val));
                        stepper_state()
                            .direction
                            .store(stepper.dir_state(), Relaxed);
                        if dir_changed {
                            tmc_state().feedforward_reset.store(true, Relaxed);
                        }
                        loaded_steps
                    }
                    None => {
                        stepper.mark_idle();
                        crate::state::stepper_diag::record_queue_empty(0);
                        if HOMING_STATE.load(Relaxed) != 0 {
                            record_stepper_event(
                                StepperEventReason::QueueEmpty,
                                0,
                                read_tim5(),
                                0,
                                0,
                                0,
                            );
                        }
                        stepper_state().interval.store(0, Relaxed);
                        tmc_state().pid_error_sums_clear.store(true, Relaxed);
                        td.set_timer_active(0, false);
                        return;
                    }
                };

                let now = read_tim5();
                let scheduled = stepper.next_step_clock();
                match stepper.check_deadline(scheduled, now, CLOCK_FREQ) {
                    Ok(compare) => {
                        crate::state::stepper_diag::record_loaded_segment_timing(
                            0,
                            loaded_steps,
                            now,
                            scheduled,
                            compare,
                        );
                        schedule_stepper_timer(td, 0, compare);
                        return;
                    }
                    Err(_) => {
                        crate::state::stepper_diag::record_loaded_segment(0, loaded_steps);
                        crate::state::stepper_diag::record_missed_deadline(0);
                        record_stepper_event(
                            StepperEventReason::MissedDeadlineLoad,
                            0,
                            now,
                            UNKNOWN_QUEUE_LEN,
                            scheduled,
                            now,
                        );
                        td.set_timer_active(0, false);
                        TMC_DISABLE_SIGNAL.signal(());
                        defmt::error!("Stepper too far in past");
                        return;
                    }
                }
            }

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
                    let is_step = matches!(phase, StepPhase::Step);
                    if is_step {
                        let dir = if stepper.dir_state() { 1i32 } else { -1i32 };
                        stepper_state().position.fetch_add(dir, Relaxed);
                        crate::state::stepper_diag::record_executed_step(
                            0,
                            stepper.dir_state(),
                            read_tim5(),
                        );
                    }
                    stepper_state()
                        .interval
                        .store(stepper.current_interval(), Relaxed);
                    let now = read_tim5();
                    match stepper.check_deadline(clock, now, CLOCK_FREQ) {
                        Ok(compare) => {
                            schedule_stepper_timer(td, 0, compare);
                            return;
                        }
                        Err(_) => {
                            crate::state::stepper_diag::record_missed_deadline(0);
                            record_stepper_event(
                                StepperEventReason::MissedDeadlineStep,
                                0,
                                now,
                                UNKNOWN_QUEUE_LEN,
                                clock,
                                now,
                            );
                            td.set_timer_active(0, false);
                            TMC_DISABLE_SIGNAL.signal(());
                            defmt::error!("Stepper too far in past");
                            return;
                        }
                    }
                }
                StepEventResult::NeedMove => {
                    // Final unstep already drove STEP low; load the next segment
                    // in the same scheduler callback if one is queued.
                }
            }
        }
    }

    fn process_trigger_timer(
        td: &mut TriggerDomain,
        kind: foci_klipper::scheduler::SchedulerEventKind,
        wake_clock: u32,
        need_report: &mut bool,
    ) {
        use core::sync::atomic::Ordering::Relaxed;
        use foci_klipper::scheduler::SchedulerEventKind;

        match kind {
            SchedulerEventKind::Stepper(stepper_idx) => process_stepper_timer(td, stepper_idx),
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
                    EndstopPhase::Scanning => td.endstops[endstop_idx].scan(read_pin(), wake_clock),
                    EndstopPhase::Oversampling => {
                        td.endstops[endstop_idx].oversample(read_pin(), wake_clock)
                    }
                };

                match result {
                    EndstopSampleResult::EnterOversamplingNow => {
                        match td.endstops[endstop_idx].oversample(read_pin(), wake_clock) {
                            EndstopSampleResult::Triggered { trsync_oid, reason } => {
                                td.trigger(trsync_oid, reason, read_tim5());
                                *need_report = true;
                            }
                            EndstopSampleResult::Reschedule { next_clock } => {
                                td.scheduler
                                    .schedule(foci_klipper::scheduler::SchedulerEvent {
                                        wake_clock: next_clock,
                                        kind: SchedulerEventKind::EndstopSample(endstop_idx),
                                    });
                            }
                            _ => {}
                        }
                    }
                    EndstopSampleResult::Triggered { trsync_oid, reason } => {
                        td.trigger(trsync_oid, reason, read_tim5());
                        *need_report = true;
                    }
                    EndstopSampleResult::Reschedule { next_clock } => {
                        td.scheduler
                            .schedule(foci_klipper::scheduler::SchedulerEvent {
                                wake_clock: next_clock,
                                kind: SchedulerEventKind::EndstopSample(endstop_idx),
                            });
                    }
                    EndstopSampleResult::Inactive => {}
                }
            }
            SchedulerEventKind::TrSyncExpire(trsync_idx) => {
                let oid = td
                    .trsync_by_oid
                    .iter()
                    .find(|&(_, &idx)| idx == trsync_idx)
                    .map(|(&oid, _)| oid);
                if let Some(oid) = oid {
                    let expire_reason = td.trsync[trsync_idx].expire_reason();
                    td.trigger(oid, expire_reason, read_tim5());
                    *need_report = true;
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
                    let now = read_tim5();
                    td.trsync[trsync_idx].set_liveness_report(oid, now);
                    *need_report = true;
                    let report_ticks = td.trsync[trsync_idx].report_ticks();
                    if report_ticks > 0 {
                        let next_report = td.trsync[trsync_idx]
                            .report_clock()
                            .wrapping_add(report_ticks);
                        td.trsync[trsync_idx].set_report_clock(next_report);
                        td.scheduler
                            .schedule(foci_klipper::scheduler::SchedulerEvent {
                                wake_clock: next_report,
                                kind: SchedulerEventKind::TrSyncReport(trsync_idx),
                            });
                    }
                }
            }
        }
    }

    fn dispatch_tim5_scheduler(td: &mut TriggerDomain, need_report: &mut bool) {
        let dispatch_start = read_tim5();
        loop {
            let now = read_tim5();
            if let Some(event) = td.scheduler.pop_due(now) {
                process_trigger_timer(td, event.kind, event.wake_clock, need_report);
                let elapsed = read_tim5().wrapping_sub(dispatch_start) as i32;
                if elapsed >= TIM5_SCHEDULER_REPEAT_TICKS as i32 {
                    td.reprogram_timer();
                    return;
                }
                continue;
            }

            let Some(next_clock) = td.scheduler.earliest_deadline() else {
                td.disarm_timer();
                return;
            };
            let now = read_tim5();
            let delta = next_clock.wrapping_sub(now) as i32;
            if delta <= 0 {
                continue;
            }
            if delta <= TRIGGER_TIMER_MIN_LEAD_TICKS as i32 {
                while (next_clock.wrapping_sub(read_tim5()) as i32) > 0 {}
                continue;
            }
            td.arm_timer_at(next_clock);
            return;
        }
    }

    /// TIM5 unified Klipper scheduler interrupt.
    ///
    /// Dispatches stepper, endstop, and trsync timers from one ordered software
    /// queue on the TIM5 clock. Same priority as USB OTG and fault EXTI (P3).
    #[task(binds = TIM5, priority = 3)]
    fn step_timer_irq(_ctx: step_timer_irq::Context) {
        use core::sync::atomic::Ordering::Relaxed;

        let tim5 = embassy_stm32::pac::TIM5;
        let sr = tim5.sr().read();
        let dier = tim5.dier().read();
        let cc1_pending = sr.ccif(0) && dier.ccie(0);
        let cc2_pending = sr.ccif(1) && dier.ccie(1);
        if !cc1_pending && !cc2_pending {
            return;
        }
        tim5.sr().modify(|w| {
            w.set_ccif(0, false);
            w.set_ccif(1, false);
        });

        let mut need_report = false;
        with_trigger_domain(|td| {
            dispatch_tim5_scheduler(td, &mut need_report);
        });
        if need_report {
            REPORT_PENDING.store(true, Relaxed);
            report_task::spawn().ok();
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

    /// Send pending stepper diagnostic events outside ISR/P1 contexts.
    #[task(priority = 2)]
    async fn stepper_diag_task(_ctx: stepper_diag_task::Context) {
        crate::heartbeat::emit_stepper_diagnostics();
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

foci_firmware::foci_ankyra_config! {
    transport = crate::transport::TRANSPORT_OUTPUT: crate::transport::BufferTransportOutput,
    context = &'ctx mut crate::firmware_ctx::FirmwareCtx<'ctx>,
    providers = [
        foci_firmware::ankyra::provider::FOCI_FIRMWARE_PROVIDER,
        crate::ankyra::provider::BOARD_PROVIDER,
    ],
    board_static_strings = [],
    app = "foci",
    version = env!("CARGO_PKG_VERSION"),
    build_versions = "",
    license = "MIT OR Apache-2.0",
}
