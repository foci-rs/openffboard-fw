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
mod tmc_control;
mod tmc_diagnostics;
mod tmc_dispatch;
mod transport;
mod trigger_domain;
mod usb_task;
#[cfg(feature = "trace")]
mod usb_trace;

#[rtic::app(device = crate::pac, peripherals = true, dispatchers = [UART4, UART5, USART3])]
mod app {
    use core::sync::atomic::Ordering::Relaxed;

    use crate::ankyra::constants::CLOCK_FREQ;
    use embassy_stm32::gpio::{Level, Output, Speed};
    use embassy_stm32::spi::Spi;
    use embassy_stm32::wdg::IndependentWatchdog;
    use embassy_stm32::{bind_interrupts, peripherals};
    use embedded_hal_bus::spi::ExclusiveDevice;
    use foci_klipper::state::State;
    use foci_tmc4671::spi::Tmc4671Spi;
    use rtic_monotonics::fugit::{self, ExtU64};
    use rtic_monotonics::stm32::prelude::*;
    use rtic_sync::channel::Receiver;

    use crate::app_types::{CycleDelay, TmcSpiDevice};
    use crate::helpers::lifecycle::*;
    use crate::helpers::stepper::*;
    use crate::helpers::stepper_diag::*;
    use crate::helpers::trsync::*;
    use crate::state::stepper::*;
    use crate::state::tmc::*;
    use crate::trigger_domain::{
        TRIGGER_DOMAIN, TRIGGER_TIMER_MIN_LEAD_TICKS, TriggerDomain, with_trigger_domain,
        with_trigger_domain_site,
    };
    use foci_firmware::context::stepper::{
        StepperArmCheckBoard, StepperStopCheckBoard, StepperTimerStateBoard,
    };
    use foci_firmware::context::stepper_reconcile::drain_stepper_reconciliation;
    use foci_firmware::diagnostic::StepperEventReason;
    use foci_firmware::state::lifecycle::*;
    use foci_firmware::state::stepper::{NO_STEPPER_CHANNEL, STEPPER_DEADLINE_MISS_CHANNEL};
    use foci_firmware::state::tmc::{TMC_DISABLE_SIGNAL, TMC_QUEUE_FLUSH};
    use foci_firmware::state::trsync::*;
    use foci_firmware::stepper_runtime::StepDeadlineStatus;

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

    use crate::board_config::{CURRENT_SCALE_MA_PER_LSB, TMC_CONFIG};
    use crate::tmc_control::board::{
        openffboard_backend, refresh_velocity_scale, stepper_state, tmc_state,
    };
    use crate::tmc_control::request::{handle_commission_wait_reject, handle_tmc_control_request};
    use foci_firmware::tmc_control::TmcControlRequestTail;

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
        #[cfg(feature = "trace")]
        crate::usb_trace::note_trace_boot();
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
        let tmc = Tmc4671Spi::new(spi_dev)
            .with_retry_policy(foci_tmc4671::spi::RetryPolicy::production());

        // TMC register request channel
        let (tmc_tx, tmc_rx) = ctx.local.tmc_channel.split();
        critical_section::with(|cs| {
            TMC_REQUEST_SENDER.borrow(cs).replace(Some(tmc_tx));
        });

        defmt::info!("SPI1 configured: 8 MHz, Mode 3, DMA");

        crate::board_init::init_usb_pins(p.PA11, p.PA12);
        crate::board_init::enable_usb_clock();

        let uid = crate::board_init::read_stm32_uid(ctx.local.uid_str);
        #[cfg(feature = "trace")]
        crate::usb_trace::set_trace_serial(uid);

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
                product: {
                    #[cfg(feature = "trace")]
                    {
                        crate::usb_trace::TRACE_PRODUCT
                    }
                    #[cfg(not(feature = "trace"))]
                    {
                        "FOCI OpenFFBoard"
                    }
                },
                serial: uid,
            });
        }

        crate::board_init::enable_dwt_cyccnt();

        // Initialize TriggerDomain
        // SAFETY: init runs once before any task starts. No concurrent access.
        unsafe {
            core::ptr::addr_of_mut!(TRIGGER_DOMAIN)
                .cast::<TriggerDomain>()
                .write(TriggerDomain::new());
        }
        // Spawn tasks
        stepper_reconcile::spawn().expect("stepper reconciliation worker must spawn exactly once");
        tmc_control::spawn().ok();
        usb_task::spawn().ok();
        blink::spawn().ok();
        stats::spawn().ok();
        watchdog::spawn().ok();

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

        match foci_firmware::adc_vm::capture_adc_vm_offset(tmc).await {
            Ok(capture) => {
                if let Some(tmc_state) = foci_firmware::state::tmc::tmc_channel(0) {
                    tmc_state.store_adc_vm_offset(capture.offset_raw, capture.sample_count);
                }
                defmt::info!(
                    "ADC_VM offset cached: raw={} samples={}",
                    capture.offset_raw,
                    capture.sample_count
                );
            }
            Err(_) => {
                defmt::warn!("ADC_VM offset capture failed; host connect will reject VM decode");
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
        //   2. ADC_VM scaling is board-specific. OpenFFBoard v1.3 now has a
        //      schematic-derived VM model for dump/gain-synthesis evidence,
        //      but that does not make the TMC brake threshold path a complete
        //      protection design. Do not re-enable this comparator without a
        //      dedicated brake-threshold validation pass.
        //
        // Proper overvoltage protection requires the Layer 2 brake (STM32
        // ADC monitoring + PE10 GPIO control) with its own voltage sense
        // path that spans the full 12-65V operating range. That is tracked
        // in docs/specs/2026-03-29-foci-design.md (Future Ideas).
        //
        // For now, rely on the PSU's own overvoltage protection and the
        // bulk capacitors to absorb typical regen events. PE10 remains
        // held LOW at boot (line ~1650) to keep the Layer 2 brake FET off.
        // Configure STATUS_MASK to route ADC current clipping to the STATUS
        // output pin (PE8 EXTI). Other flags remain polled.
        const STATUS_ADC_I_CLIPPED: u32 = 1 << 26;
        let bootstrap_failure = {
            let mut backend = openffboard_backend(tmc, enable_pin);
            foci_firmware::tmc_control::bootstrap_tmc_safety(&mut backend, STATUS_ADC_I_CLIPPED)
                .await
                .err()
        };

        if let Some(failure) = bootstrap_failure {
            let reason = failure.shutdown_reason(0);
            defmt::error!(
                "TMC4671 channel 0 safety bootstrap failed: {:?}",
                failure.stage
            );
            loop {
                if SHUTDOWN_ACTIVE.load(core::sync::atomic::Ordering::Relaxed) {
                    Mono::delay(1.millis()).await;
                    continue;
                }
                let _request = rx.recv().await;
                if SHUTDOWN_ACTIVE.load(core::sync::atomic::Ordering::Relaxed) {
                    continue;
                }
                ctx.shared.app_state.lock(|state| {
                    if crate::firmware_ctx::enter_shutdown(state) {
                        foci_firmware::dispatch_shutdown!(
                            reason,
                            read_tim5(),
                            &mut crate::_ankyra_config::Sender
                        );
                    }
                });
            }
        }

        // Safety bootstrap succeeded. Clear any pending EXTI state and enable
        // the TMC4671 STATUS interrupt before entering normal control.
        {
            let syscfg = embassy_stm32::pac::SYSCFG;
            // EXTICR3[3:0] = 4 selects port E for EXTI line 8.
            syscfg.exticr(2).modify(|w| w.set_exti(0, 4));
            let exti = embassy_stm32::pac::EXTI;
            exti.rtsr(0).modify(|w| w.set_line(8, true));
            exti.ftsr(0).modify(|w| w.set_line(8, false));
            exti.pr(0).write(|w| w.set_line(8, true)); // Clear pending
            exti.imr(0).modify(|w| w.set_line(8, true)); // Unmask
            defmt::info!(
                "TMC4671 STATUS_MASK: {:#010x}, EXTI8 enabled",
                STATUS_ADC_I_CLIPPED
            );
        }

        // Shared runtime for board-agnostic control state and timing. This
        // RTIC task keeps hardware ownership, request waiting, diagnostics,
        // and USB trace transport board-local.
        let mut runtime = foci_firmware::tmc_control::TmcControlRuntime::new_for_channel(0);
        let signals = foci_firmware::tmc_control::TmcControlSignals::new(
            &SHUTDOWN_ACTIVE,
            &TMC_QUEUE_FLUSH,
            &FAULT_ACTIVE,
            &TMC_DISABLE_SIGNAL,
        );

        // Board constants bundled once and passed to runtime commissioning tick.
        let tmc_control_config = foci_firmware::tmc_control::TmcControlConfig::new(
            CLOCK_FREQ,
            TMC_CONFIG.pwm_freq_hz(),
            CURRENT_SCALE_MA_PER_LSB,
            TMC_CONFIG,
        );

        let mut next_control_tick = Mono::now() + 1.millis();
        let mut commission_poll_driver = foci_firmware::tmc_control::CommissionPollDriver::new();
        let mut commission_tick_max_cycles = 0u32;
        let mut commission_measurement_active = false;
        // Commission trace sequence counter. Incremented each time a new engine
        // is installed so trace frames can be correlated per commissioning run.
        let mut commission_trace_engine_seq: u32 = 0;
        #[cfg(feature = "trace")]
        let mut commission_trace_phase: Option<(
            foci_firmware::commissioning::types::EngineKind,
            foci_firmware::autotune::PhaseId,
        )> = None;

        // Whether the shutdown safe-state completed this shutdown cycle.
        // If phase-advance restore fails, keep retrying before idling.
        let mut shutdown_safe_state_applied = false;

        // Main loop: wait for channel message or 1ms timeout.
        // On message: handle request. On timeout: write position target.
        // Position is read from stepper_state().position_target atomic (updated
        // by usb_receive at P2) — no shared resource lock needed.
        'tmc_loop: loop {
            use embassy_futures::select::{Either, select};

            // On shutdown, perform one-shot motor safe-state then idle until
            // shutdown clears. This catches USB disconnect / host crash where
            // no further commands arrive but the TMC4671 remains energized.
            if SHUTDOWN_ACTIVE.load(core::sync::atomic::Ordering::Relaxed) {
                commission_measurement_active = false;
                if !shutdown_safe_state_applied {
                    #[cfg(feature = "trace")]
                    {
                        commission_trace_phase = None;
                    }
                    let shutdown_safe_state_complete = {
                        let mut backend = openffboard_backend(tmc, enable_pin);
                        foci_firmware::tmc_control::lifecycle::apply_shutdown_safe_state(
                            &mut runtime,
                            &mut backend,
                            tmc_state(),
                            stepper_state(),
                        )
                        .await
                    };
                    tmc_state()
                        .commission_in_flight
                        .store(false, core::sync::atomic::Ordering::Relaxed);
                    stepper_stop_from_p1(0);
                    shutdown_safe_state_applied = shutdown_safe_state_complete;
                }
                Mono::delay(1.millis()).await;
                continue;
            }
            shutdown_safe_state_applied = false;

            // Drain stale requests after config_reset
            if TMC_QUEUE_FLUSH.load(core::sync::atomic::Ordering::Relaxed) {
                commission_measurement_active = false;
                while rx.try_recv().is_ok() {}
                let commission_was_active = runtime.is_commission_active();
                crate::tmc_control::effects::apply_actions(
                    runtime.poll_queue_flush(&signals, tmc_state(), stepper_state()),
                    |reason| {
                        ctx.shared.app_state.lock(|state| {
                            if crate::firmware_ctx::enter_shutdown(state) {
                                crate::tmc_control::effects::emit_shutdown(reason);
                            }
                        });
                    },
                );
                // Clear any in-flight commissioning engine and restore safe
                // hardware state (zero targets, disable outputs) so the motor
                // is not left energized in whatever mode the engine last set.
                if commission_was_active {
                    #[cfg(feature = "trace")]
                    {
                        commission_trace_phase = None;
                    }
                    let mut backend = openffboard_backend(tmc, enable_pin);
                    let mut state = runtime.dispatch_state(tmc_state(), stepper_state());
                    foci_firmware::tmc_control::lifecycle::disarm_motor(&mut backend, &mut state)
                        .await;
                }
                tmc_state()
                    .commission_in_flight
                    .store(false, core::sync::atomic::Ordering::Relaxed);
                defmt::info!("TMC request queue flushed after config_reset");
            }

            // Process emergency-disable signals before commissioning can poll
            // again. Absolute commissioning waits return here after lowering
            // GPIO, so no engine or SPI work precedes fault containment.
            if TMC_DISABLE_SIGNAL.try_take().is_some()
                || FAULT_ACTIVE.load(core::sync::atomic::Ordering::Relaxed)
            {
                commission_measurement_active = false;
                let deadline_channel =
                    STEPPER_DEADLINE_MISS_CHANNEL.swap(NO_STEPPER_CHANNEL, Relaxed);
                let stop_channel = if deadline_channel == NO_STEPPER_CHANNEL {
                    0
                } else {
                    deadline_channel
                };
                stepper_stop_from_p1(stop_channel);
                FAULT_ACTIVE.store(false, core::sync::atomic::Ordering::Relaxed);

                {
                    let s = runtime.dispatch_state(tmc_state(), stepper_state());
                    *s.motor_enabled = false;
                    *s.calibrated = false;
                }
                enable_pin.set_low();
                if deadline_channel != NO_STEPPER_CHANNEL {
                    let clock = read_tim5();
                    ctx.shared.app_state.lock(|state| {
                        if crate::firmware_ctx::enter_shutdown(state) {
                            foci_firmware::dispatch_shutdown!(
                                foci_firmware::shutdown::ShutdownReason::StepperTooFarInPast,
                                clock,
                                &mut crate::_ankyra_config::Sender
                            );
                        }
                    });
                    continue;
                }
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
                #[cfg(feature = "trace")]
                crate::usb_trace::request_event(
                    foci_firmware::trace::events::EVENT_LEVEL_ERROR,
                    foci_firmware::trace::events::SUBSYSTEM_SAFETY_SHUTDOWN,
                    crate::usb_trace::EVENT_STATUS_INTERRUPT,
                    [flags as i32, 0, 0, 0],
                    1,
                );
                defmt::error!("TMC4671 emergency disable: STATUS={:#010x}", flags);
                const ADC_I_CLIPPED: u32 = 1 << 26;
                ctx.shared.app_state.lock(|state| {
                    if flags & ADC_I_CLIPPED != 0 {
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
                continue;
            }

            // Tick the commissioning engine through the shared absolute scheduler.
            if runtime.is_commission_active() {
                if !commission_measurement_active {
                    commission_poll_driver =
                        foci_firmware::tmc_control::CommissionPollDriver::new();
                    commission_tick_max_cycles = 0;
                    commission_measurement_active = true;
                }
                'commission_poll: loop {
                    let exit = crate::tmc_control::effects::classify_commission_wait_exit(
                        SHUTDOWN_ACTIVE.load(core::sync::atomic::Ordering::Relaxed),
                        TMC_QUEUE_FLUSH.load(core::sync::atomic::Ordering::Relaxed),
                        TMC_DISABLE_SIGNAL.signaled(),
                        FAULT_ACTIVE.load(core::sync::atomic::Ordering::Relaxed),
                    );
                    if let Some(exit) = exit {
                        if matches!(
                            exit,
                            crate::tmc_control::effects::CommissionWaitExit::EmergencyDisable
                        ) {
                            enable_pin.set_low();
                        }
                        continue 'tmc_loop;
                    }
                    // Capture deadline before the tick so SPI time consumed by the
                    // engine reduces the remaining trace budget.
                    #[cfg(feature = "trace")]
                    let engine_tick_deadline =
                        Mono::now() + fugit::Duration::<u64, 1, 1_000_000>::millis(1);

                    let engine_kind = runtime
                        .pending_commission_kind()
                        .expect("active commissioning engine has a kind");
                    let tick_start = cortex_m::peripheral::DWT::cycle_count();
                    let mut poll = {
                        let mut backend = openffboard_backend(tmc, enable_pin);
                        runtime
                            .step_commissioning_scheduled(&mut backend)
                            .await
                            .expect("active commissioning engine produces a poll")
                    };
                    let now_us = Mono::now().duration_since_epoch().to_micros();
                    let driver_action = match commission_poll_driver.classify(&poll, now_us) {
                        Ok(action) => action,
                        Err(error) => {
                            poll =
                                foci_firmware::commissioning::schedule::CommissionPoll::Terminal {
                                    result: if matches!(
                                        engine_kind,
                                        foci_firmware::commissioning::types::EngineKind::OuterTune
                                    ) {
                                        foci_firmware::commissioning::schedule::CommissionTerminal::OuterFailed(
                                    error,
                                )
                                    } else {
                                        foci_firmware::commissioning::schedule::CommissionTerminal::InnerFailed(
                                    error,
                                )
                                    },
                                };
                            foci_firmware::tmc_control::PollDriverAction::Terminal
                        }
                    };
                    if matches!(
                        driver_action,
                        foci_firmware::tmc_control::PollDriverAction::YieldNow
                    ) {
                        Mono::delay(foci_firmware::tmc_control::FORCED_READY_YIELD_US.micros())
                            .await;
                        let exit = crate::tmc_control::effects::classify_commission_wait_exit(
                            SHUTDOWN_ACTIVE.load(core::sync::atomic::Ordering::Relaxed),
                            TMC_QUEUE_FLUSH.load(core::sync::atomic::Ordering::Relaxed),
                            TMC_DISABLE_SIGNAL.signaled(),
                            FAULT_ACTIVE.load(core::sync::atomic::Ordering::Relaxed),
                        );
                        if let Some(exit) = exit {
                            if matches!(
                                exit,
                                crate::tmc_control::effects::CommissionWaitExit::EmergencyDisable
                            ) {
                                enable_pin.set_low();
                            }
                            continue 'tmc_loop;
                        }
                    }
                    let action =
                        {
                            let mut backend = openffboard_backend(tmc, enable_pin);
                            let action = match poll {
                            foci_firmware::commissioning::schedule::CommissionPoll::Terminal {
                                result,
                            } => runtime
                                .drive_commissioning_terminal(
                                    result,
                                    &mut backend,
                                    tmc_state(),
                                    stepper_state(),
                                    &tmc_control_config,
                                )
                                .await,
                            poll => runtime.classify_commission_poll(poll),
                        };
                            action.expect("active commissioning poll produces an action")
                        };
                    let tick_cycles =
                        cortex_m::peripheral::DWT::cycle_count().wrapping_sub(tick_start);
                    commission_tick_max_cycles = commission_tick_max_cycles.max(tick_cycles);
                    #[cfg(feature = "trace")]
                    crate::usb_trace::drain_pending_events();

                    use foci_firmware::tmc_control::ScheduledCommissionTickAction;
                    match action {
                        ScheduledCommissionTickAction::ReadyNow { event }
                        | ScheduledCommissionTickAction::At { event, .. } => {
                            if let Some(event) = event {
                                let foci_firmware::commissioning::schedule::CommissionEvent::PhaseChanged {
                                oid,
                                phase_id,
                                engine_kind,
                            } = event;
                                #[cfg(not(feature = "trace"))]
                                let _ = engine_kind;
                                #[cfg(feature = "trace")]
                                {
                                    commission_trace_phase = Some((engine_kind, phase_id));
                                }
                                emit_phase_reply(oid, phase_id);
                            }
                            #[cfg(feature = "trace")]
                            if commission_trace_phase
                                == Some((
                                    foci_firmware::commissioning::types::EngineKind::Commission,
                                    foci_firmware::autotune::PhaseId::ClosedLoopEntry,
                                ))
                            {
                                let trace_tick = runtime.advance_trace_sample_tick();
                                crate::usb_trace::sample_commission_closed_loop_entry(
                                    tmc,
                                    runtime.is_motor_enabled(),
                                    trace_tick,
                                    || {
                                        engine_tick_deadline
                                            .checked_duration_since(Mono::now())
                                            .map(|d| d.to_micros())
                                            .unwrap_or(0)
                                    },
                                    || Mono::now() > engine_tick_deadline,
                                )
                                .await;
                            }

                            match driver_action {
                                foci_firmware::tmc_control::PollDriverAction::PollNow => {
                                    continue 'tmc_loop;
                                }
                                foci_firmware::tmc_control::PollDriverAction::YieldNow => {
                                    continue 'tmc_loop;
                                }
                                foci_firmware::tmc_control::PollDriverAction::WaitUntil(
                                    deadline_us,
                                ) => {
                                    loop {
                                        let exit =
                                        crate::tmc_control::effects::classify_commission_wait_exit(
                                            SHUTDOWN_ACTIVE
                                                .load(core::sync::atomic::Ordering::Relaxed),
                                            TMC_QUEUE_FLUSH
                                                .load(core::sync::atomic::Ordering::Relaxed),
                                            TMC_DISABLE_SIGNAL.signaled(),
                                            FAULT_ACTIVE
                                                .load(core::sync::atomic::Ordering::Relaxed),
                                        );
                                        if let Some(exit) = exit {
                                            if matches!(
                                            exit,
                                            crate::tmc_control::effects::CommissionWaitExit::EmergencyDisable
                                        ) {
                                            enable_pin.set_low();
                                        }
                                            continue 'tmc_loop;
                                        }

                                        if let Ok(req) = rx.try_recv() {
                                            use foci_firmware::tmc_control::CommissionWaitRequestDisposition;
                                            match crate::tmc_control::effects::commission_wait_request_disposition(&req) {
                                            CommissionWaitRequestDisposition::Cancel => {
                                                enable_pin.set_low();
                                                runtime.take_commission_engine();
                                                tmc_state().commission_in_flight.store(
                                                    false,
                                                    core::sync::atomic::Ordering::Relaxed,
                                                );
                                                let outcome = handle_tmc_control_request(
                                                    req,
                                                    tmc,
                                                    enable_pin,
                                                    &mut runtime,
                                                    &mut commission_trace_engine_seq,
                                                )
                                                .await;
                                                if let Some(reason) = outcome.shutdown {
                                                    ctx.shared.app_state.lock(|state| {
                                                        if crate::firmware_ctx::enter_shutdown(state) {
                                                            crate::tmc_control::effects::emit_shutdown(reason);
                                                        }
                                                    });
                                                }
                                                if outcome.should_refresh_velocity_scale {
                                                    refresh_velocity_scale(&mut runtime);
                                                }
                                                commission_measurement_active = false;
                                                continue 'tmc_loop;
                                            }
                                            CommissionWaitRequestDisposition::RejectBusy => {
                                                let outcome = handle_commission_wait_reject(req);
                                                if let Some(reason) = outcome.shutdown {
                                                    ctx.shared.app_state.lock(|state| {
                                                        if crate::firmware_ctx::enter_shutdown(state) {
                                                            crate::tmc_control::effects::emit_shutdown(reason);
                                                        }
                                                    });
                                                }
                                                if outcome.should_refresh_velocity_scale {
                                                    refresh_velocity_scale(&mut runtime);
                                                }
                                            }
                                            CommissionWaitRequestDisposition::DropBusy => {}
                                        }
                                        }

                                        let now_us = Mono::now().duration_since_epoch().to_micros();
                                        match foci_firmware::tmc_control::classify_absolute_wait(
                                            now_us,
                                            deadline_us,
                                        ) {
                                            Ok(
                                                foci_firmware::tmc_control::AbsoluteWaitAction::PollNow,
                                            ) => {
                                                break;
                                            }
                                            Ok(
                                                foci_firmware::tmc_control::AbsoluteWaitAction::SleepUntil(
                                                    wake_us,
                                                ),
                                            ) => {
                                                let wake = fugit::Instant::<u64, 1, 1_000_000>::from_ticks(
                                                    wake_us,
                                                );
                                                Mono::delay_until(wake).await;
                                            }
                                            Err(_) => {
                                                enable_pin.set_low();
                                                TMC_DISABLE_SIGNAL.signal(());
                                                continue 'tmc_loop;
                                            }
                                        }
                                    }
                                    continue 'commission_poll;
                                }
                                foci_firmware::tmc_control::PollDriverAction::Terminal => {
                                    unreachable!("nonterminal poll classified as terminal")
                                }
                            }
                        }
                        ScheduledCommissionTickAction::Terminal { reply, actions } => {
                            #[cfg(feature = "trace")]
                            {
                                commission_trace_phase = None;
                            }
                            {
                                let mut state =
                                    runtime.dispatch_state(tmc_state(), stepper_state());
                                apply_stepper_post_action(actions.stepper, &mut state);
                            }
                            emit_terminal_reply(reply);
                            defmt::info!(
                                "commissioning terminal max tick cycles: {}",
                                commission_tick_max_cycles,
                            );
                            commission_measurement_active = false;
                            break 'commission_poll;
                        }
                    }
                }
                // Terminal cases fall through to normal channel processing.
            }

            let control_period_us = runtime.control_period_us(
                foci_firmware::tmc_control::TmcControlStateRefs::new(tmc_state(), stepper_state()),
            );
            let control_sleep = Mono::delay_until(next_control_tick);

            let selected = select(rx.recv(), control_sleep).await;

            match selected {
                // Channel message received
                Either::First(Ok(req)) => {
                    let outcome = handle_tmc_control_request(
                        req,
                        tmc,
                        enable_pin,
                        &mut runtime,
                        &mut commission_trace_engine_seq,
                    )
                    .await;

                    if let Some(reason) = outcome.shutdown {
                        ctx.shared.app_state.lock(|state| {
                            if crate::firmware_ctx::enter_shutdown(state) {
                                crate::tmc_control::effects::emit_shutdown(reason);
                            }
                        });
                    }

                    if outcome.should_refresh_velocity_scale {
                        refresh_velocity_scale(&mut runtime);
                    }

                    match outcome.tail {
                        TmcControlRequestTail::FallThrough => {}
                        TmcControlRequestTail::SkipTickTail => continue,
                    }
                }
                Either::First(Err(_)) => continue,
                // Control timeout - run fast FF or the decimated 1 kHz maintenance tick.
                Either::Second(()) => {}
            }
            next_control_tick = Mono::now() + u64::from(control_period_us).micros();
            let slow_control_tick = runtime.update_slow_control(control_period_us);

            // Hardware step/dir: position target is driven by TIM5 CC1 ISR
            // toggling STEP GPIO → TMC4671 hardware step counter. No SPI
            // position target write needed. The 1ms tick loop only handles
            // STATUS_FLAGS polling, TrSync, endstop, and watchdog.

            let homing_active = HOMING_STATE.load(core::sync::atomic::Ordering::Relaxed) != 0;
            let reset = tmc_state()
                .feedforward_reset
                .swap(false, core::sync::atomic::Ordering::Relaxed);

            let refs =
                foci_firmware::tmc_control::TmcControlStateRefs::new(tmc_state(), stepper_state());
            let command_velocity_reference =
                runtime.command_velocity_reference(refs, homing_active, reset);
            if runtime
                .apply_feedforward_tick(
                    tmc,
                    refs,
                    &tmc_control_config,
                    homing_active,
                    reset,
                    slow_control_tick,
                )
                .await
                .is_err()
            {
                defmt::warn!("TMC feedforward tick failed");
            }

            if !slow_control_tick {
                continue;
            }

            runtime
                .apply_phase_and_position_tick(
                    tmc,
                    refs,
                    command_velocity_reference,
                    homing_active,
                    reset,
                )
                .await;

            #[cfg(feature = "trace")]
            {
                let trace_tick = runtime.advance_trace_tick();
                let inputs = foci_firmware::trace::sampling::TraceSampleInputs::from_runtime_trace(
                    runtime.trace_state(),
                    trace_tick,
                );
                crate::usb_trace::sample_tick(
                    tmc,
                    crate::usb_trace::TraceSampleContext {
                        motor_enabled: runtime.is_motor_enabled(),
                    },
                    inputs,
                )
                .await;
                if trace_tick.phase_advance_due {
                    crate::usb_trace::emit_phase_advance(
                        trace_tick.phase_advance_counts,
                        trace_tick.phase_advance_flags,
                        tmc_state()
                            .phase_advance_gain_ppm
                            .load(core::sync::atomic::Ordering::Relaxed),
                        tmc_state()
                            .phase_advance_max_counts
                            .load(core::sync::atomic::Ordering::Relaxed),
                        tmc_state()
                            .phase_advance_deadband
                            .load(core::sync::atomic::Ordering::Relaxed),
                    );
                }
                crate::usb_trace::emit_status_frames(
                    tmc,
                    trace_tick.control_tick,
                    trace_tick.status_detail_due,
                )
                .await;
                crate::usb_trace::request_spi_duration_event_if_ready();
                crate::usb_trace::drain_pending_events();
                if trace_tick.metadata_due {
                    crate::usb_trace::emit_metadata();
                }
            }

            // TrSync and endstop polling have been replaced by the TIM5 CC2
            // trigger scheduler. See step_timer_irq CC2 dispatch above.

            crate::tmc_control::effects::apply_actions(
                runtime
                    .apply_slow_maintenance_tick(
                        tmc,
                        refs,
                        &tmc_control_config,
                        foci_firmware::tmc_control::TmcControlHomingRefs::new(
                            &HOMING_STATE,
                            &HOMING_TRIGGERED,
                        ),
                    )
                    .await,
                |reason| {
                    ctx.shared.app_state.lock(|state| {
                        if crate::firmware_ctx::enter_shutdown(state) {
                            crate::tmc_control::effects::emit_shutdown(reason);
                        }
                    });
                },
            );
        }
    }

    impl StepperStopCheckBoard for crate::firmware_ctx::OpenFfboardStepperBoard {
        fn stop_check_index(&self, channel: u8) -> Option<usize> {
            (channel == 0).then_some(0)
        }

        fn clear_step_timer(
            &mut self,
            state: &mut foci_firmware::context::trsync::TriggerDomainState,
            stepper_idx: usize,
            _channel: u8,
        ) {
            state.scheduler.cancel(|kind| {
                matches!(
                    kind,
                    foci_klipper::scheduler::SchedulerEventKind::Stepper(0)
                )
            });
            self.set_timer_active(state, stepper_idx, false);
            crate::trigger_domain::reprogram_timer(state);
        }
    }

    impl StepperArmCheckBoard for crate::firmware_ctx::OpenFfboardStepperBoard {
        fn arm_check_index(&self, channel: u8) -> Option<usize> {
            (channel == 0).then_some(0)
        }

        fn motor_armed(&self, _channel: u8) -> bool {
            tmc_state()
                .motor_armed
                .load(core::sync::atomic::Ordering::Relaxed)
        }

        fn queue_has_moves(&self, channel: u8) -> bool {
            // SAFETY: P2 is the sole producer while the timer is inactive.
            unsafe {
                step_queue(channel)
                    .map(|queue| !queue.is_empty())
                    .unwrap_or(false)
            }
        }

        fn arm_check_timer(
            &mut self,
            state: &mut foci_firmware::context::trsync::TriggerDomainState,
            stepper_idx: usize,
            channel: u8,
        ) {
            let now = read_tim5();
            crate::state::stepper_diag::record_timer_activation(channel, now);
            state.scheduler.cancel(|kind| {
                matches!(
                    kind,
                    foci_klipper::scheduler::SchedulerEventKind::Stepper(0)
                )
            });
            insert_trigger_event_unmeasured(
                state,
                foci_klipper::scheduler::SchedulerEvent {
                    wake_clock: now.wrapping_add(1),
                    kind: foci_klipper::scheduler::SchedulerEventKind::Stepper(0),
                },
            );
            self.set_timer_active(state, stepper_idx, true);
            crate::trigger_domain::reprogram_timer(state);
        }
    }

    /// P2 software task: reconcile pending stepper stop and arm state.
    #[task(priority = 2)]
    async fn stepper_reconcile(_ctx: stepper_reconcile::Context) {
        loop {
            STEPPER_RECONCILE_WAKE.wait().await;
            loop {
                let pending = STEPPER_RECONCILE_PENDING.take_pending();
                if pending.is_empty() {
                    break;
                }
                with_trigger_domain(|td| {
                    let mut board = crate::firmware_ctx::OpenFfboardStepperBoard;
                    drain_stepper_reconciliation(td.state_mut(), &mut board, pending);
                });
            }
        }
    }

    /// Maximum continuous TIM5 scheduler dispatch before deferring.
    ///
    /// Matches Klipper's repeat guard shape: process bursts in one IRQ, but
    /// avoid monopolizing P3 if a pathological stream stays immediately due.
    const TIM5_SCHEDULER_REPEAT_TICKS: u32 = CLOCK_FREQ / 10_000;
    /// Short defer after a saturated TIM5 dispatch burst.
    ///
    /// Klipper C uses a 5 us defer when timer dispatch keeps main tasks busy.
    /// On OpenFFBoard TIM5 runs at 84 MHz, so this is 420 timer ticks.
    const TIM5_SCHEDULER_DEFER_TICKS: u32 = CLOCK_FREQ / 200_000;

    #[inline(always)]
    fn add_scheduler_cycles(accumulator: &mut u32, start: u32) {
        *accumulator = (*accumulator)
            .wrapping_add(cortex_m::peripheral::DWT::cycle_count().wrapping_sub(start));
    }

    fn record_schedule_result(result: foci_klipper::scheduler::ScheduleResult) -> bool {
        match result {
            foci_klipper::scheduler::ScheduleResult::Inserted { new_earliest } => new_earliest,
            foci_klipper::scheduler::ScheduleResult::Full => {
                crate::state::stepper_diag::record_scheduler_full();
                false
            }
        }
    }

    fn insert_trigger_event_unmeasured(
        state: &mut foci_firmware::context::trsync::TriggerDomainState,
        event: foci_klipper::scheduler::SchedulerEvent,
    ) -> bool {
        record_schedule_result(state.scheduler.try_schedule(event))
    }

    fn insert_trigger_event(
        td: &mut TriggerDomain,
        event: foci_klipper::scheduler::SchedulerEvent,
        scheduler_cycles: &mut u32,
    ) -> bool {
        let start = cortex_m::peripheral::DWT::cycle_count();
        let result = td.scheduler.try_schedule(event);
        add_scheduler_cycles(scheduler_cycles, start);
        record_schedule_result(result)
    }

    fn arm_stepper_timer_event(
        td: &mut TriggerDomain,
        stepper_idx: usize,
        clock: u32,
        scheduler_cycles: &mut u32,
        slot: Option<foci_klipper::scheduler::ScheduledSlot>,
    ) {
        if let Some(slot) = slot {
            let start = cortex_m::peripheral::DWT::cycle_count();
            let result = td.scheduler.requeue_slot(slot, clock);
            add_scheduler_cycles(scheduler_cycles, start);
            if matches!(result, foci_klipper::scheduler::ScheduleResult::Full) {
                crate::state::stepper_diag::record_scheduler_full();
            }
        } else {
            insert_trigger_event(
                td,
                foci_klipper::scheduler::SchedulerEvent {
                    wake_clock: clock,
                    kind: foci_klipper::scheduler::SchedulerEventKind::Stepper(stepper_idx),
                },
                scheduler_cycles,
            );
        }
    }

    #[inline(always)]
    fn spin_tim5_ticks(start: u32, ticks: u32) {
        while read_tim5().wrapping_sub(start) < ticks {
            core::hint::spin_loop();
        }
    }

    /// Emit one bounded logical event as a complete, non-preemptible pulse train.
    ///
    /// This P3 hot path performs no SPI, logging, allocation, scheduler work,
    /// or yield. The mapper guarantees zero, one, or two physical pulses.
    #[inline(always)]
    fn emit_logical_step_event(event: foci_klipper::stepper::LogicalStepEvent) {
        let gpio_a = embassy_stm32::pac::GPIOA;
        let gpio_b = embassy_stm32::pac::GPIOB;

        gpio_b.bsrr().write(|w| w.set_br(11, true));
        if event.dir_setup_required {
            gpio_a.bsrr().write(|w| {
                if event.direction {
                    w.set_bs(0, true);
                } else {
                    w.set_br(0, true);
                }
            });
            let setup_start = read_tim5();
            spin_tim5_ticks(setup_start, crate::state::stepper::STEP_DIR_SETUP_TICKS);
        }

        crate::state::stepper::assert_physical_pulse_budget(event.physical_pulses);
        for _ in 0..event.physical_pulses {
            gpio_b.bsrr().write(|w| w.set_bs(11, true));
            let high_start = read_tim5();
            spin_tim5_ticks(high_start, crate::state::stepper::STEP_HALF_PERIOD_TICKS);
            gpio_b.bsrr().write(|w| w.set_br(11, true));
            let low_start = read_tim5();
            spin_tim5_ticks(low_start, crate::state::stepper::STEP_HALF_PERIOD_TICKS);
        }
    }

    fn process_stepper_timer(
        td: &mut TriggerDomain,
        stepper_idx: usize,
        scheduler_cycles: &mut u32,
        slot: Option<foci_klipper::scheduler::ScheduledSlot>,
    ) {
        use core::sync::atomic::Ordering::Relaxed;
        use foci_klipper::stepper::StepEventResult;

        if stepper_idx != 0 {
            return;
        }
        let mut stepper_slot = slot;
        let gpio_b = embassy_stm32::pac::GPIOB;
        gpio_b.bsrr().write(|w| w.set_br(11, true));
        if FAULT_ACTIVE.load(Relaxed) || !tmc_state().motor_armed.load(Relaxed) {
            td.set_timer_active(0, false);
            stepper_state().interval.store(0, Relaxed);
            stepper_state().interval_add.store(0, Relaxed);
            tmc_state().pid_error_sums_clear.store(true, Relaxed);
            return;
        }
        if HOMING_STATE.load(Relaxed) == 1 {
            // P1 has not yet performed homing preparation. Defer the first
            // step so it can restore phase advance and suppress position I.
            arm_stepper_timer_event(
                td,
                0,
                read_tim5().wrapping_add(CLOCK_FREQ / 1000),
                scheduler_cycles,
                stepper_slot.take(),
            );
            td.set_timer_active(0, true);
            td.reprogram_timer();
            return;
        }

        // SAFETY: ISR stepper state is only accessed from this P3 ISR or from
        // stop paths that first gate/cancel the stepper timer.
        let stepper = unsafe { isr_stepper(0).expect("OpenFFBoard channel 0 exists") };

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
                        let interval_add = i32::from(m.add);
                        let old_direction = stepper.dir_state();
                        stepper.load_move(m);
                        let dir_changed = old_direction != stepper.dir_state();
                        stepper_state()
                            .interval
                            .store(stepper.current_interval(), Relaxed);
                        stepper_state().interval_add.store(interval_add, Relaxed);
                        let dir_val = if stepper.dir_state() {
                            embassy_stm32::pac::gpio::vals::Odr::HIGH
                        } else {
                            embassy_stm32::pac::gpio::vals::Odr::LOW
                        };
                        gpio_a.odr().modify(|w| w.set_odr(0, dir_val));
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
                        stepper_state().interval_add.store(0, Relaxed);
                        tmc_state().pid_error_sums_clear.store(true, Relaxed);
                        td.set_timer_active(0, false);
                        return;
                    }
                };

                let now = read_tim5();
                let scheduled = stepper.next_step_clock();
                #[cfg(feature = "trace")]
                crate::usb_trace::record_step_timing(scheduled, now);
                match stepper.check_deadline(scheduled, now, CLOCK_FREQ) {
                    Ok(compare) => {
                        crate::state::stepper_diag::record_loaded_segment_timing(
                            0,
                            loaded_steps,
                            now,
                            scheduled,
                            compare,
                        );
                        arm_stepper_timer_event(
                            td,
                            0,
                            compare,
                            scheduler_cycles,
                            stepper_slot.take(),
                        );
                        return;
                    }
                    Err(_) => {
                        crate::state::stepper_diag::record_loaded_segment(0, loaded_steps);
                        crate::state::stepper_diag::record_missed_deadline(0);
                        #[cfg(feature = "trace")]
                        crate::usb_trace::TRACE_TIMING
                            .missed_or_deferred_steps
                            .fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                        record_stepper_event(
                            StepperEventReason::MissedDeadlineLoad,
                            0,
                            now,
                            UNKNOWN_QUEUE_LEN,
                            scheduled,
                            now,
                        );
                        td.set_timer_active(0, false);
                        STEPPER_DEADLINE_MISS_CHANNEL.store(0, Relaxed);
                        TMC_DISABLE_SIGNAL.signal(());
                        return;
                    }
                }
            }

            let result = stepper.step_event();
            let event = result.event();
            emit_logical_step_event(event);

            // STEP is low before mapper state, pulse counts, or the logical
            // execution become observable outside this P3 train.
            let now = read_tim5();
            crate::state::stepper::publish_logical_event(stepper, event);
            let logical_delta = if event.direction { 1 } else { -1 };
            stepper_state().position.fetch_add(logical_delta, Relaxed);
            stepper_state().direction.store(event.direction, Relaxed);
            stepper_state()
                .interval
                .store(stepper.current_interval(), Relaxed);
            crate::state::stepper_diag::record_executed_step(
                0,
                event.direction,
                event.physical_pulses,
                now,
            );

            match result {
                StepEventResult::ScheduleNext { clock, .. } => {
                    #[cfg(feature = "trace")]
                    crate::usb_trace::record_step_timing(clock, now);
                    arm_stepper_timer_event(td, 0, clock, scheduler_cycles, stepper_slot.take());
                    return;
                }
                StepEventResult::NeedMove { .. } => {
                    // The final logical event is fully emitted and published;
                    // load a contiguous queued move only after STEP is low.
                }
            }
        }
    }

    fn process_trigger_timer(
        td: &mut TriggerDomain,
        kind: foci_klipper::scheduler::SchedulerEventKind,
        wake_clock: u32,
        need_report: &mut bool,
        scheduler_cycles: &mut u32,
        stepper_slot: Option<foci_klipper::scheduler::ScheduledSlot>,
    ) {
        use core::sync::atomic::Ordering::Relaxed;
        use foci_klipper::scheduler::SchedulerEventKind;

        match kind {
            SchedulerEventKind::Stepper(stepper_idx) => {
                process_stepper_timer(td, stepper_idx, scheduler_cycles, stepper_slot);
            }
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
                                insert_trigger_event(
                                    td,
                                    foci_klipper::scheduler::SchedulerEvent {
                                        wake_clock: next_clock,
                                        kind: SchedulerEventKind::EndstopSample(endstop_idx),
                                    },
                                    scheduler_cycles,
                                );
                            }
                            _ => {}
                        }
                    }
                    EndstopSampleResult::Triggered { trsync_oid, reason } => {
                        td.trigger(trsync_oid, reason, read_tim5());
                        *need_report = true;
                    }
                    EndstopSampleResult::Reschedule { next_clock } => {
                        insert_trigger_event(
                            td,
                            foci_klipper::scheduler::SchedulerEvent {
                                wake_clock: next_clock,
                                kind: SchedulerEventKind::EndstopSample(endstop_idx),
                            },
                            scheduler_cycles,
                        );
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
                        insert_trigger_event(
                            td,
                            foci_klipper::scheduler::SchedulerEvent {
                                wake_clock: next_report,
                                kind: SchedulerEventKind::TrSyncReport(trsync_idx),
                            },
                            scheduler_cycles,
                        );
                    }
                }
            }
        }
    }

    fn defer_tim5_scheduler(td: &mut TriggerDomain) {
        if td.scheduler.is_empty() {
            td.disarm_timer();
            return;
        }
        #[cfg(feature = "trace")]
        crate::usb_trace::TRACE_TIMING
            .missed_or_deferred_steps
            .fetch_add(1, core::sync::atomic::Ordering::Relaxed);
        crate::state::stepper_diag::record_tim5_defer();
        td.arm_timer_at(read_tim5().wrapping_add(TIM5_SCHEDULER_DEFER_TICKS));
    }

    fn dispatch_tim5_scheduler(
        td: &mut TriggerDomain,
        need_report: &mut bool,
        scheduler_cycles: &mut u32,
    ) -> u32 {
        let dispatch_start = read_tim5();
        let mut processed_events = 0;
        loop {
            let now = read_tim5();
            let start = cortex_m::peripheral::DWT::cycle_count();
            let slot = td.scheduler.pop_due_slot(now);
            add_scheduler_cycles(scheduler_cycles, start);
            if let Some(slot) = slot {
                let event = slot.event();
                let stepper_slot = match event.kind {
                    foci_klipper::scheduler::SchedulerEventKind::Stepper(_) => Some(slot),
                    _ => None,
                };
                // Record before the callback so dispatch work is not folded
                // into scheduler-pop lateness.
                let lateness = now.wrapping_sub(event.wake_clock) as i32;
                if lateness > 0 {
                    crate::state::stepper_diag::record_scheduler_pop_lateness(lateness as u32);
                }
                processed_events += 1;
                if let Some((channel, StepDeadlineStatus::Fatal { .. })) =
                    crate::state::stepper::scheduler_step_deadline_status(
                        event.kind,
                        event.wake_clock,
                        now,
                    )
                {
                    let channel = channel as u8;
                    crate::state::stepper_diag::record_missed_deadline(channel);
                    record_stepper_event(
                        StepperEventReason::MissedDeadlineStep,
                        channel,
                        now,
                        UNKNOWN_QUEUE_LEN,
                        event.wake_clock,
                        now,
                    );
                    STEPPER_DEADLINE_MISS_CHANNEL.store(channel, Relaxed);
                    td.disarm_timer();
                    td.set_timer_active(channel as usize, false);
                    TMC_DISABLE_SIGNAL.signal(());
                    return processed_events;
                }
                process_trigger_timer(
                    td,
                    event.kind,
                    event.wake_clock,
                    need_report,
                    scheduler_cycles,
                    stepper_slot,
                );
                let elapsed = read_tim5().wrapping_sub(dispatch_start) as i32;
                if elapsed >= TIM5_SCHEDULER_REPEAT_TICKS as i32 {
                    defer_tim5_scheduler(td);
                    return processed_events;
                }
                continue;
            }

            let start = cortex_m::peripheral::DWT::cycle_count();
            let next_clock = td.scheduler.earliest_deadline();
            add_scheduler_cycles(scheduler_cycles, start);
            let Some(next_clock) = next_clock else {
                td.disarm_timer();
                return processed_events;
            };
            let now = read_tim5();
            let compare = foci_klipper::scheduler::timer_arm_clock(
                now,
                next_clock,
                TRIGGER_TIMER_MIN_LEAD_TICKS,
            );
            td.arm_timer_at(compare);
            return processed_events;
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
        let entry_now = tim5.cnt().read();
        let mut entry_lateness = 0i32;
        if cc1_pending {
            let ccr = tim5.ccr(0).read();
            let delta = entry_now.wrapping_sub(ccr) as i32;
            if delta > entry_lateness {
                entry_lateness = delta;
            }
        }
        if cc2_pending {
            let ccr = tim5.ccr(1).read();
            let delta = entry_now.wrapping_sub(ccr) as i32;
            if delta > entry_lateness {
                entry_lateness = delta;
            }
        }
        if entry_lateness > 0 {
            crate::state::stepper_diag::record_tim5_entry_latency(entry_lateness as u32);
        }
        let irq_start = cortex_m::peripheral::DWT::cycle_count();
        tim5.sr().modify(|w| {
            w.set_ccif(0, false);
            w.set_ccif(1, false);
        });

        let mut need_report = false;
        let mut dispatch_cycles = 0;
        let mut processed_events = 0;
        let mut scheduler_cycles = 0;
        with_trigger_domain_site(crate::state::stepper_diag::PERF_SITE_TIM5, |td| {
            let dispatch_start = cortex_m::peripheral::DWT::cycle_count();
            processed_events = dispatch_tim5_scheduler(td, &mut need_report, &mut scheduler_cycles);
            dispatch_cycles = cortex_m::peripheral::DWT::cycle_count().wrapping_sub(dispatch_start);
        });
        let irq_cycles = cortex_m::peripheral::DWT::cycle_count().wrapping_sub(irq_start);
        crate::state::stepper_diag::record_tim5_irq(irq_cycles, dispatch_cycles, processed_events);
        crate::state::stepper_diag::record_scheduler_cycles(scheduler_cycles, processed_events);
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
    /// Fires when any STATUS_MASK-enabled flag is asserted. Currently this is
    /// `adc_i_clipped`; tmc_control reads STATUS_FLAGS on the next tick to log
    /// the cause.
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
    /// CPU usage metrics.
    #[task(priority = 1)]
    async fn stats(_ctx: stats::Context) {
        let mut accumulator = crate::heartbeat::StatsAccumulator::new(read_tim5());
        loop {
            Mono::delay(5000.millis()).await;
            crate::heartbeat::stats_tick(
                &mut accumulator,
                read_tim5(),
                &STATS_SEND_TIME,
                &STATS_SEND_TIME_HIGH,
            );
        }
    }

    /// Service the independent watchdog above sustained priority-1 work.
    #[task(priority = 2, local = [wdg])]
    async fn watchdog(ctx: watchdog::Context) {
        loop {
            Mono::delay(crate::heartbeat::WATCHDOG_INTERVAL_MS.millis()).await;
            crate::heartbeat::watchdog_tick(ctx.local.wdg);
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
