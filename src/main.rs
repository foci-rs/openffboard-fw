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

mod commission;

#[rtic::app(device = crate::pac, peripherals = true, dispatchers = [UART4, UART5, USART3])]
mod app {
    use core::cell::RefCell;

    use anchor::*;
    // CycleDelay uses cortex_m::asm::delay() (CPU busy-wait, no DWT needed)
    use critical_section::Mutex;
    use embassy_stm32::gpio::{Level, Output, Speed};
    use embassy_stm32::mode::Async as AsyncMode;
    use embassy_stm32::spi::Spi;
    use embassy_stm32::wdg::IndependentWatchdog;
    use embassy_stm32::{bind_interrupts, peripherals};
    use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
    use embassy_sync::signal::Signal;
    use embedded_hal_bus::spi::ExclusiveDevice;
    use klipper_mcu::state::State;
    use rtic_monotonics::stm32::prelude::*;
    use rtic_sync::channel::{Receiver, Sender};
    use tmc4671::spi::Tmc4671Spi;

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

    /// SPI bus type for TMC4671 communication.
    type Spi1Bus = Spi<'static, AsyncMode, embassy_stm32::spi::mode::Master>;

    /// Sub-microsecond busy-wait delay for SPI read pauses.
    ///
    /// The TMC4671-LA requires a 500ns pause after the address byte for
    /// SPI reads above 2 MHz (datasheet Table 2, tSCKpause). At 168 MHz,
    /// 500ns = 84 CPU cycles — too short for async yield.
    struct CycleDelay;

    impl embedded_hal_async::delay::DelayNs for CycleDelay {
        async fn delay_ns(&mut self, ns: u32) {
            let cycles = (ns as u64 * 168 / 1000) as u32;
            cortex_m::asm::delay(cycles);
        }
    }

    /// TMC4671 SPI device with exclusive bus access and software CS.
    type TmcSpiDevice = ExclusiveDevice<Spi1Bus, Output<'static>, CycleDelay>;

    use foci_core::tmc::TmcRequest;

    const TMC_CHANNEL_SIZE: usize = 8;

    /// SPI budget for fast trace preset (us). DWT measured ~227us on hardware.
    const TRACE_SPI_BUDGET_FAST_US: u64 = 300;
    /// SPI budget for full trace preset (us). Estimated ~450us from fast ratio.
    const TRACE_SPI_BUDGET_FULL_US: u64 = 550;

    /// Number of TMC4671 motor channels on OpenFFBoard (single-motor board).
    const TMC_MOTOR_COUNT: u8 = 1;

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
    const TMC_CONFIG: foci_core::config::TmcConfig = foci_core::config::TmcConfig {
        pwm_maxcnt: 3999,
        tmc_clock_hz: 25_000_000,
        bbm_h: 10,
        bbm_l: 10,
        dsadc_mdec: 331,
        excitation_voltage: 512,
        didt_cycles: 5000,
        bandwidth_hz: 150.0,
    };

    /// Errors that can occur during the TMC4671 calibration sequence.
    #[derive(Debug, defmt::Format)]
    enum CalibrationError {
        /// SPI communication or register verification failed.
        SpiError,
        /// Motor already enabled (calibration not needed).
        AlreadyEnabled,
        /// Internal firmware error (queue full, channel not initialized, etc.).
        InternalError,
        /// Required configuration (e.g., `tmc_set_encoder`) has not been
        /// issued by the host before calibration. Most commonly this means
        /// STEP_WIDTH has not been computed, so the TMC4671 hardware step
        /// input would not advance the position target — the motor would
        /// calibrate successfully but silently not respond to step pulses.
        ConfigFault,
    }

    impl CalibrationError {
        /// Map to a numeric status code for `foci_calibrate_result`.
        fn status_code(&self) -> u8 {
            match self {
                Self::SpiError => 1,
                // Code 4 was PidFault (removed with tmc_enter_closedloop).
                Self::AlreadyEnabled => 5,
                Self::InternalError => 6,
                Self::ConfigFault => 7,
            }
        }
    }

    /// CommissioningBackend implementation for OpenFFBoard.
    ///
    /// Wraps the TMC4671 SPI driver, enable pin, and timer so the
    /// board-agnostic [`crate::commission::CommissioningEngine`] can
    /// perform register I/O, pin control, and timing.
    struct OpenFFBoardBackend<'a> {
        tmc: &'a mut Tmc4671Spi<TmcSpiDevice>,
        enable_pin: &'a mut Output<'static>,
    }

    impl<'a> crate::commission::CommissioningBackend for OpenFFBoardBackend<'a> {
        type Error = tmc4671::spi::SpiError<
            embedded_hal_bus::spi::DeviceError<
                embassy_stm32::spi::Error,
                core::convert::Infallible,
            >,
        >;

        async fn read_register(
            &mut self,
            reg: tmc4671::registers::Register,
        ) -> Result<u32, Self::Error> {
            self.tmc.read_register(reg).await
        }

        async fn write_register(
            &mut self,
            reg: tmc4671::registers::Register,
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
                .read_register(tmc4671::registers::ABN_DECODER_COUNT)
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

        fn tmc_config(&self) -> &foci_core::config::TmcConfig {
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

    /// USB wake signal. The OTG_FS ISR signals this to wake usb_task.
    static USB_WAKE: Signal<CriticalSectionRawMutex, ()> = Signal::new();

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

    /// Stepper position target, updated by usb_receive (P2) after each
    /// protocol dispatch, read by tmc_control (P1) for SPI writes.
    /// Lockfree: AtomicI32 avoids priority ceiling lock that would
    /// block P2 protocol tasks.
    static STEPPER_POSITION_TARGET: core::sync::atomic::AtomicI32 =
        core::sync::atomic::AtomicI32::new(0);

    /// Current limit for PID_TORQUE_FLUX_LIMITS (0x5E).
    /// Datasheet: single u16 field at bits[15:0]; upper 16 bits ignored by hardware.
    /// Set by tmc_set_current, re-applied during calibration to ensure
    /// the limit is correct regardless of TMC4671 register state.
    static CURRENT_LIMIT_REG: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

    /// Velocity biquad filter cutoff frequency in Hz (set by tmc_set_velocity_filter).
    /// 0 = disabled, 10..=1000 = enabled. Applied during closed-loop entry.
    static VELOCITY_FILTER_HZ: core::sync::atomic::AtomicU16 =
        core::sync::atomic::AtomicU16::new(0);

    /// Torque biquad filter cutoff frequency in Hz (set by tmc_set_torque_filter).
    /// 0 = disabled, 10..=1000 = enabled. Applied during closed-loop entry.
    static TORQUE_FILTER_HZ: core::sync::atomic::AtomicU16 = core::sync::atomic::AtomicU16::new(0);

    /// Position target biquad filter cutoff frequency in Hz (set by tmc_set_position_filter).
    /// 0 = disabled, 10..=1000 = enabled. Applied during closed-loop entry.
    static POSITION_FILTER_HZ: core::sync::atomic::AtomicU16 =
        core::sync::atomic::AtomicU16::new(0);

    /// Flux biquad filter cutoff frequency in Hz (set by tmc_set_flux_filter).
    /// 0 = disabled, 10..=1000 = enabled. Applied during closed-loop entry.
    static FLUX_FILTER_HZ: core::sync::atomic::AtomicU16 = core::sync::atomic::AtomicU16::new(0);

    /// Velocity feedforward enable flag (set by tmc_set_velocity_feedforward).
    /// When true, the 1ms tick loop computes velocity from the step
    /// interval and writes PID_VELOCITY_OFFSET.
    static VELOCITY_FEEDFORWARD: core::sync::atomic::AtomicBool =
        core::sync::atomic::AtomicBool::new(false);

    /// Signal the tick loop to zero feedforward for one tick after a
    /// direction change. Avoids a sign-flip transient in
    /// PID_VELOCITY_OFFSET. Set by handle_tmc_request when encoder
    /// direction changes, consumed by tmc_control on next iteration.
    static FEEDFORWARD_RESET: core::sync::atomic::AtomicBool =
        core::sync::atomic::AtomicBool::new(false);

    /// Signal the tick loop to clear TMC4671 PID error sums after motion.
    /// The step ISR cannot use SPI, so it sets this flag when the hardware
    /// step queue drains and P1 performs the register writes on the next tick.
    static PID_ERROR_SUMS_CLEAR: core::sync::atomic::AtomicBool =
        core::sync::atomic::AtomicBool::new(false);

    /// Configurable PID velocity limit (caps position PID output).
    /// Prevents integrator-driven velocity runaway when position I > 0.
    /// Override via tmc_set_velocity_limit command from host config.
    /// Written to PID_VELOCITY_LIMIT (0x60) during closedloop entry.
    static VELOCITY_LIMIT_CFG: core::sync::atomic::AtomicU32 =
        core::sync::atomic::AtomicU32::new(500_000);

    /// Whether the encoder direction is reversed (set by tmc_set_encoder_dir).
    /// When true, position targets are negated before writing to the TMC4671.
    static ENCODER_REVERSED: core::sync::atomic::AtomicBool =
        core::sync::atomic::AtomicBool::new(false);

    /// Guard for single-in-flight commissioning. Set at command ingress (P2),
    /// cleared after commissioning completes or fails (P1).
    static COMMISSION_IN_FLIGHT: core::sync::atomic::AtomicBool =
        core::sync::atomic::AtomicBool::new(false);

    /// Shutdown active flag. Set by enter_shutdown(), cleared by clear_shutdown().
    /// Checked by tmc_control to perform one-shot motor safe-state on first tick.
    static SHUTDOWN_ACTIVE: core::sync::atomic::AtomicBool =
        core::sync::atomic::AtomicBool::new(false);

    /// Signal tmc_control to drain all pending requests. Set by config_reset
    /// (P2), cleared by tmc_control (P1) after draining.
    static TMC_QUEUE_FLUSH: core::sync::atomic::AtomicBool =
        core::sync::atomic::AtomicBool::new(false);

    // --- Hardware step/dir state ---

    /// Step count (incremented by P3 ISR, read by P1/P2 for stepper_get_position).
    static STEPPER_POSITION: core::sync::atomic::AtomicI32 = core::sync::atomic::AtomicI32::new(0);

    /// Set by P1 after calibration, cleared on disable/fault.
    /// Gates timer arming and ISR execution.
    static MOTOR_ARMED: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

    /// Set by fault EXTI, cleared by P1 fault handler after cleanup.
    static FAULT_ACTIVE: core::sync::atomic::AtomicBool =
        core::sync::atomic::AtomicBool::new(false);

    /// STEP_WIDTH value computed by tmc_set_encoder_hw, written to TMC4671
    /// register 0x78 during closedloop entry.
    static STEP_WIDTH_VALUE: core::sync::atomic::AtomicI32 = core::sync::atomic::AtomicI32::new(0);

    /// Acceptable residual following error after a completed move, in ABN
    /// encoder counts. Within this window the controller is fighting encoder
    /// quantization and belt/motor compliance more than useful position error.
    const HOLD_SETTLE_DEADBAND_COUNTS: u32 = 4;
    const HOLD_SETTLE_DELAY_MS: u8 = 100;

    /// Current step interval in TIM5 clock ticks, written by P3 after
    /// every step event. Written to 0 when the step queue drains (both
    /// the ISR empty-queue branch and stepper_stop). Read by tmc_control
    /// at 1 kHz for velocity feedforward computation.
    static STEP_INTERVAL: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

    /// Current step direction, written by P3 when direction changes.
    /// true = forward, false = reverse. Read by tmc_control for
    /// feedforward sign.
    static STEP_DIRECTION: core::sync::atomic::AtomicBool =
        core::sync::atomic::AtomicBool::new(true);

    /// Read-only mirror of `stepper_stop[0].timer_active` for P1.
    /// Written inside TriggerDomain lock via `set_timer_active()`,
    /// read by P1 for advisory purposes (PID settle timing).
    /// P1 must never enter TriggerDomain.
    static STEP_TIMER_ACTIVE_MIRROR: core::sync::atomic::AtomicBool =
        core::sync::atomic::AtomicBool::new(false);

    /// Homing I-suppression state machine.
    /// 0=Idle, 1=SuppressPending, 2=Suppressing, 3=RestorePending
    static HOMING_STATE: core::sync::atomic::AtomicU8 = core::sync::atomic::AtomicU8::new(0);

    /// TrSync OID that fired the trigger. 0xFF = no trigger pending.
    /// Written by TriggerDomain on first trigger, cleared by
    /// trsync_trigger command cleanup or config_reset.
    static HOMING_TRIGGERED: core::sync::atomic::AtomicU8 = core::sync::atomic::AtomicU8::new(0xFF);

    /// Indicates trsync_state reports are pending delivery.
    /// Set by TIM5 CC2 ISR, cleared by report_task.
    static REPORT_PENDING: core::sync::atomic::AtomicBool =
        core::sync::atomic::AtomicBool::new(false);

    /// Position P gain for cross-priority access (Q8.8).
    static POSITION_P_VALUE: core::sync::atomic::AtomicU16 =
        core::sync::atomic::AtomicU16::new(640);

    /// Position I gain for cross-priority access (Q8.8).
    static POSITION_I_VALUE: core::sync::atomic::AtomicU16 = core::sync::atomic::AtomicU16::new(0);

    /// Velocity P gain for cross-priority access (Q8.8).
    /// Default 1152 = 4.5 in Q8.8 (conservative).
    static VELOCITY_P_VALUE: core::sync::atomic::AtomicU16 =
        core::sync::atomic::AtomicU16::new(1152);

    /// Velocity I gain for cross-priority access (Q8.8).
    static VELOCITY_I_VALUE: core::sync::atomic::AtomicU16 = core::sync::atomic::AtomicU16::new(0);

    /// Trace capture buffer in CCM RAM. Initialized via `reset()` in init.
    ///
    /// SAFETY: Only accessed from tmc_control (P1) after init. MaybeUninit
    /// because CCM RAM has no default initialization; init() writes a valid
    /// TraceBuffer via ptr::write before any read.
    #[link_section = ".ccmram"]
    static mut TRACE_BUF: core::mem::MaybeUninit<foci_core::trace::TraceBuffer> =
        core::mem::MaybeUninit::uninit();

    /// Lock-free move queue for hardware step/dir.
    /// P2 enqueues, P3 dequeues. When timer is disabled, P1/P2 may drain.
    ///
    /// SAFETY: Access protocol enforced by RTIC priorities:
    /// - Timer active: P2 enqueues only (Producer), P3 dequeues only (Consumer)
    /// - Timer disabled: P1/P2 may drain (Consumer side safe, P3 quiesced)
    static mut STEP_QUEUE: heapless::spsc::Queue<klipper_mcu::stepper::StepMove, 1024> =
        heapless::spsc::Queue::new();

    /// P2-local HardwareStepper state (direction tracking, prepare_move).
    /// Only accessed from usb_task (P2) command handlers.
    static mut P2_STEPPER: klipper_mcu::stepper::HardwareStepper =
        klipper_mcu::stepper::HardwareStepper::new();

    /// P3-local HardwareStepper state (active segment, step execution).
    /// Only accessed from TIM5 CC1 ISR (P3).
    static mut ISR_STEPPER: klipper_mcu::stepper::HardwareStepper =
        klipper_mcu::stepper::HardwareStepper::new();

    // ---- TriggerDomain: unified trsync/endstop/stepper-stop resource ----

    /// Physical stepper indices that drive real motors via TIM5 CC1.
    /// OpenFFBoard has one physical stepper (index 0).
    const PHYSICAL_STEPPER_INDICES: &[usize] = &[0];

    /// Unified resource owning all trsync, endstop, and stepper-stop state.
    ///
    /// Accessed via `with_trigger_domain()` which uses
    /// `cortex_m::interrupt::free()` for an interrupt-disabled critical
    /// section, equivalent to RTIC P3 ceiling. This is NOT an RTIC shared
    /// resource — it is a `static mut` with manual locking.
    struct TriggerDomain {
        trsync: [klipper_mcu::trigger::TrSyncRuntime; klipper_mcu::state::NUM_TRSYNC],
        trsync_by_oid: heapless::LinearMap<u8, usize, { klipper_mcu::state::NUM_TRSYNC }>,
        stop_signals: [Option<klipper_mcu::trigger::StopSignal>; 8],
        stepper_stop: [klipper_mcu::trigger::StepperStopRuntime; klipper_mcu::state::NUM_STEPPERS],
        endstops: [klipper_mcu::endstop::EndstopRuntime; klipper_mcu::state::NUM_ENDSTOPS],
        endstop_by_oid: heapless::LinearMap<u8, usize, { klipper_mcu::state::NUM_ENDSTOPS }>,
        scheduler: klipper_mcu::scheduler::TriggerScheduler,
    }

    impl TriggerDomain {
        const fn new() -> Self {
            Self {
                trsync: [const { klipper_mcu::trigger::TrSyncRuntime::new() };
                    klipper_mcu::state::NUM_TRSYNC],
                trsync_by_oid: heapless::LinearMap::new(),
                stop_signals: [None; 8],
                stepper_stop: [const { klipper_mcu::trigger::StepperStopRuntime::new() };
                    klipper_mcu::state::NUM_STEPPERS],
                endstops: [const { klipper_mcu::endstop::EndstopRuntime::new() };
                    klipper_mcu::state::NUM_ENDSTOPS],
                endstop_by_oid: heapless::LinearMap::new(),
                scheduler: klipper_mcu::scheduler::TriggerScheduler::new(),
            }
        }

        /// Set `timer_active` for a stepper, updating the P1 mirror
        /// for physical steppers.
        fn set_timer_active(&mut self, stepper_idx: usize, active: bool) {
            self.stepper_stop[stepper_idx].timer_active = active;
            // Mirror to P1-readable atomic for physical steppers
            if stepper_idx == 0 {
                STEP_TIMER_ACTIVE_MIRROR.store(active, core::sync::atomic::Ordering::Relaxed);
            }
        }

        /// Read `timer_active` for a stepper.
        fn timer_active(&self, stepper_idx: usize) -> bool {
            self.stepper_stop[stepper_idx].timer_active
        }

        /// Attempt to trigger a trsync, dispatching stop signals to all
        /// linked steppers. Returns the trigger result.
        ///
        /// Must be called inside `with_trigger_domain()`.
        fn trigger(
            &mut self,
            trsync_oid: u8,
            reason: u8,
            clock: u32,
        ) -> klipper_mcu::trigger::TriggerResult {
            let idx = match self.trsync_by_oid.get(&trsync_oid) {
                Some(&idx) => idx,
                None => {
                    return klipper_mcu::trigger::TriggerResult::UnknownOid;
                }
            };
            let result = self.trsync[idx].trigger(trsync_oid, reason, clock);
            if matches!(result, klipper_mcu::trigger::TriggerResult::First { .. }) {
                // Collect stepper indices from matching stop signals, then
                // clear the slots. Two-pass avoids borrow conflict between
                // stop_signals iteration and set_timer_active(&mut self).
                let mut stopped_indices: heapless::Vec<usize, 8> = heapless::Vec::new();
                for slot in self.stop_signals.iter_mut() {
                    if let Some(signal) = slot {
                        if signal.trsync_oid == trsync_oid {
                            let _ = stopped_indices.push(signal.stepper_idx);
                            *slot = None;
                        }
                    }
                }
                // Dispatch physical stops and update stepper state
                for &stepper_idx in &stopped_indices {
                    stop_physical_stepper_no_lock(stepper_idx);
                    self.stepper_stop[stepper_idx].need_reset = true;
                    self.stepper_stop[stepper_idx].blocked_by_trigger = true;
                    self.stepper_stop[stepper_idx].blocked_by_trsync = Some(trsync_oid);
                    self.set_timer_active(stepper_idx, false);
                }
                // Signal homing I-suppression only when a stop signal
                // actually fired — prevents unrelated trsync triggers
                // from advancing the suppression state machine.
                if !stopped_indices.is_empty() {
                    HOMING_TRIGGERED.store(trsync_oid, core::sync::atomic::Ordering::Relaxed);
                }
            }
            result
        }

        /// Reprogram TIM5 CC2 from the scheduler's earliest deadline.
        ///
        /// Called inside the TriggerDomain lock from both P2 command handlers
        /// and the P3 ISR after processing events.
        fn reprogram_cc2(&mut self) {
            let tim5 = embassy_stm32::pac::TIM5;
            if let Some(earliest) = self.scheduler.earliest_deadline() {
                let now = tim5.cnt().read();
                let delta = earliest.wrapping_sub(now) as i32;
                let clamped = if delta <= TRIGGER_TIMER_MIN_LEAD_TICKS as i32 {
                    now.wrapping_add(TRIGGER_TIMER_MIN_LEAD_TICKS)
                } else {
                    earliest
                };
                tim5.ccr(1).write_value(clamped);
                tim5.sr().modify(|w| w.set_ccif(1, false));
                tim5.dier().modify(|w| w.set_ccie(1, true));
                // Ensure NVIC is unmasked (idempotent)
                unsafe {
                    cortex_m::peripheral::NVIC::unmask(embassy_stm32::pac::Interrupt::TIM5);
                }
                self.scheduler.set_cc2_active(true);
            } else {
                tim5.dier().modify(|w| w.set_ccie(1, false));
                self.scheduler.set_cc2_active(false);
            }
        }
    }

    /// Minimum lead time for TIM5 CC2 arming (trigger scheduler).
    /// 128 ticks at 84 MHz = ~1.5 us.
    const TRIGGER_TIMER_MIN_LEAD_TICKS: u32 = 128;

    /// TriggerDomain static. Initialized in RTIC `init`, accessed via
    /// `with_trigger_domain()`.
    static mut TRIGGER_DOMAIN: core::mem::MaybeUninit<TriggerDomain> =
        core::mem::MaybeUninit::uninit();

    /// Lock TriggerDomain with interrupt-disabled critical section.
    ///
    /// Equivalent to P3 ceiling since P3 is the highest application priority.
    /// Callable from P2 command handlers and P3 ISR.
    ///
    /// Uses `addr_of_mut!` to avoid the `static_mut_refs` lint (matching the
    /// `STEP_QUEUE` access pattern in the existing codebase).
    #[inline]
    fn with_trigger_domain<F, R>(f: F) -> R
    where
        F: FnOnce(&mut TriggerDomain) -> R,
    {
        cortex_m::interrupt::free(|_| {
            // SAFETY: All access to TRIGGER_DOMAIN goes through this function.
            // interrupt::free ensures exclusive access across all priorities.
            // addr_of_mut! avoids creating a reference to the static mut.
            unsafe { f(&mut *core::ptr::addr_of_mut!(TRIGGER_DOMAIN).cast::<TriggerDomain>()) }
        })
    }

    /// Stop a physical stepper without locking TriggerDomain.
    ///
    /// Called from inside `with_trigger_domain()` during trigger dispatch.
    /// Only acts on physical stepper indices (CC1 timer, GPIO, queue).
    ///
    /// Per spec: disable CC1, clear interval, reset ISR stepper engine,
    /// drain queue, reset direction GPIO.
    fn stop_physical_stepper_no_lock(stepper_idx: usize) {
        use core::sync::atomic::Ordering::Relaxed;
        if !PHYSICAL_STEPPER_INDICES.contains(&stepper_idx) {
            return;
        }
        let tim5 = embassy_stm32::pac::TIM5;
        // 1. Disable CC1 interrupt — quiesce P3 step timer
        tim5.dier().modify(|w| w.set_ccie(0, false));
        tim5.sr().modify(|w| w.set_ccif(0, false));
        // 2. Clear step interval
        STEP_INTERVAL.store(0, Relaxed);
        PID_ERROR_SUMS_CLEAR.store(true, Relaxed);
        // 3. Reset ISR stepper engine state (clear stale partial segment)
        // SAFETY: P3 (timer ISR) is quiesced by CC1IE disable above.
        unsafe {
            (*core::ptr::addr_of_mut!(ISR_STEPPER)).reset();
        }
        // 4. Drain queue
        // SAFETY: P3 (timer ISR) is quiesced — we are the sole accessor.
        unsafe { while (*core::ptr::addr_of_mut!(STEP_QUEUE)).dequeue().is_some() {} }
        // 5. Reset direction GPIO (PA0) to known low state
        let gpio_a = embassy_stm32::pac::GPIOA;
        gpio_a
            .odr()
            .modify(|w| w.set_odr(0, embassy_stm32::pac::gpio::vals::Odr::LOW));
        STEP_DIRECTION.store(true, Relaxed); // Forward = default
    }

    static TMC_REQUEST_SENDER: Mutex<
        RefCell<Option<Sender<'static, TmcRequest, TMC_CHANNEL_SIZE>>>,
    > = Mutex::new(RefCell::new(None));

    /// Get a mutable reference to the trace buffer.
    ///
    /// # Safety
    ///
    /// Must only be called from tmc_control (P1) after init has completed.
    #[inline]
    unsafe fn trace_buf() -> &'static mut foci_core::trace::TraceBuffer {
        // Use addr_of_mut! to avoid creating a reference to the mutable static
        // (Rust 2024 edition compatibility). MaybeUninit<T> has the same layout as T.
        &mut *core::ptr::addr_of_mut!(TRACE_BUF).cast::<foci_core::trace::TraceBuffer>()
    }

    pub struct BufferTransportOutput;
    pub const TRANSPORT_OUTPUT: BufferTransportOutput = BufferTransportOutput;

    impl TransportOutput for BufferTransportOutput {
        type Output = ScratchOutput;
        fn output(&self, f: impl FnOnce(&mut Self::Output)) {
            let mut scratch = ScratchOutput::new();
            f(&mut scratch);
            let output = scratch.result();
            klipper_usb::tx_write(output);
            // Wake usb_task to drain TX buffer. Critical for data
            // generated outside usb_task (stats output from P1).
            USB_WAKE.signal(());
        }
    }

    // ---- Klipper constants ----

    #[klipper_constant]
    const CLOCK_FREQ: u32 = 84_000_000;

    #[klipper_constant]
    const MCU: &str = "stm32f407xx";

    #[klipper_constant]
    const STEPPER_STEP_BOTH_EDGE: u32 = 1;

    #[klipper_constant]
    const RECEIVE_WINDOW: u32 = 192;

    #[klipper_constant]
    const STATS_SUMSQ_BASE: u32 = 256;

    // ---- Reserved pin constants ----
    // Tells the Klipper host which pins are firmware-owned and must not be
    // assigned to user-facing config sections (steppers, heaters, fans, etc.).

    #[klipper_constant]
    const RESERVE_PINS_USB: &str = "PA9,PA11,PA12";

    #[klipper_constant]
    const RESERVE_PINS_SPI: &str = "PA4,PA5,PA6,PA7";

    #[klipper_constant]
    const RESERVE_PINS_TMC: &str = "PE7,PE8,PE10,PE12";

    #[klipper_constant]
    const RESERVE_PINS_LED: &str = "PD7,PE0,PE1";

    #[klipper_constant]
    const RESERVE_PINS_SWD: &str = "PA13,PA14";

    #[klipper_constant]
    const RESERVE_PINS_DRV: &str = "PB0,PB1,PC4,PC5";

    /// STEP (PB11) and DIR (PA0) GPIO pins for TMC4671 hardware step/dir.
    #[klipper_constant]
    const RESERVE_PINS_STEP: &str = "PA0,PB11";

    // ---- Klipper command handlers ----

    /// Read the TIM5 counter (Klipper clock at 84 MHz).
    fn read_tim5() -> u32 {
        embassy_stm32::pac::TIM5.cnt().read()
    }

    #[klipper_command(in_shutdown)]
    pub fn get_clock(_context: &mut State) {
        let clock = read_tim5();
        klipper_reply!(clock, clock: u32 = clock);
    }

    #[klipper_command(in_shutdown)]
    pub fn get_uptime(_context: &mut State) {
        use core::sync::atomic::Ordering::Relaxed;
        let cur = read_tim5();
        let high =
            STATS_SEND_TIME_HIGH.load(Relaxed) + u32::from(cur < STATS_SEND_TIME.load(Relaxed));
        klipper_reply!(uptime, high: u32 = high, clock: u32 = cur);
    }

    #[klipper_command(in_shutdown)]
    pub fn get_config(context: &mut State) {
        let (is_config, crc, is_shutdown, move_count) =
            klipper_mcu::commands::lifecycle::get_config(context);
        klipper_reply!(
            config,
            is_config: bool = is_config,
            crc: u32 = crc,
            is_shutdown: bool = is_shutdown,
            move_count: u16 = move_count
        );
    }

    /// Clear shutdown state, allowing normal command processing to resume.
    ///
    /// Matches Klipper C `command_clear_shutdown` (basecmd.c). If called
    /// when not in shutdown, triggers a shutdown instead.
    #[klipper_command(in_shutdown)]
    pub fn clear_shutdown(context: &mut State) {
        if klipper_mcu::commands::lifecycle::clear_shutdown(context).is_err()
            && enter_shutdown(context)
        {
            klipper_shutdown!("Shutdown cleared when not shutdown", read_tim5());
        }
        SHUTDOWN_ACTIVE.store(false, core::sync::atomic::Ordering::Relaxed);
    }

    /// Full config reset: clear all OIDs, steppers, and shutdown state.
    ///
    /// Matches Klipper C `config_reset` (basecmd.c:262). Only available
    /// during shutdown. The host uses this as a fallback restart mechanism
    /// when hardware `reset` is not available (linux/PRU platforms).
    #[klipper_command(in_shutdown)]
    pub fn config_reset(context: &mut State) {
        use core::sync::atomic::Ordering::Relaxed;
        if !context.mcu_state.is_shutdown() {
            if enter_shutdown(context) {
                klipper_shutdown!("config_reset only available when shutdown", read_tim5());
            }
            return;
        }
        klipper_mcu::commands::lifecycle::config_reset(context);
        // Full TriggerDomain reset
        with_trigger_domain(|td| {
            td.scheduler.clear();
            let tim5 = embassy_stm32::pac::TIM5;
            tim5.dier().modify(|w| w.set_ccie(1, false));
            for ts in td.trsync.iter_mut() {
                ts.reset();
            }
            for es in td.endstops.iter_mut() {
                es.reset();
            }
            for sig in td.stop_signals.iter_mut() {
                *sig = None;
            }
            td.trsync_by_oid.clear();
            td.endstop_by_oid.clear();
            // Stop physical steppers only
            for &idx in PHYSICAL_STEPPER_INDICES {
                stop_physical_stepper_no_lock(idx);
                td.set_timer_active(idx, false);
            }
            // Reset all stepper stop state (physical + logical)
            for ss in td.stepper_stop.iter_mut() {
                *ss = klipper_mcu::trigger::StepperStopRuntime::new();
            }
        });
        // Clear derived mailbox state outside critical section
        REPORT_PENDING.store(false, Relaxed);
        HOMING_TRIGGERED.store(0xFF, Relaxed);
        HOMING_STATE.store(0, Relaxed);
        // Clear firmware-side endstop pin assignments
        for pin_id in &ENDSTOP_PIN_IDS {
            pin_id.store(0xFF, Relaxed);
        }
        // Reset board-local control and protocol state
        CURRENT_LIMIT_REG.store(0, Relaxed);
        VELOCITY_FILTER_HZ.store(0, Relaxed);
        TORQUE_FILTER_HZ.store(0, Relaxed);
        POSITION_FILTER_HZ.store(0, Relaxed);
        FLUX_FILTER_HZ.store(0, Relaxed);
        VELOCITY_FEEDFORWARD.store(false, Relaxed);
        VELOCITY_LIMIT_CFG.store(500_000, Relaxed);
        FEEDFORWARD_RESET.store(false, Relaxed);
        PID_ERROR_SUMS_CLEAR.store(false, Relaxed);
        ENCODER_REVERSED.store(false, Relaxed);
        STEPPER_POSITION_TARGET.store(0, Relaxed);
        COMMISSION_IN_FLIGHT.store(false, Relaxed);
        STATS_SEND_TIME.store(0, Relaxed);
        STATS_SEND_TIME_HIGH.store(0, Relaxed);
        // Signal tmc_control to drain stale requests from the pre-reset epoch
        TMC_QUEUE_FLUSH.store(true, Relaxed);
    }

    #[klipper_command]
    pub fn finalize_config(context: &mut State, crc: u32) {
        klipper_mcu::commands::lifecycle::finalize_config(context, crc);
        defmt::info!("Config finalized, CRC: {:#010x}", crc);
    }

    #[klipper_command]
    pub fn allocate_oids(_count: u8) {
        klipper_mcu::commands::lifecycle::allocate_oids(_count);
    }

    #[klipper_command(in_shutdown)]
    pub fn emergency_stop(context: &mut State) {
        if enter_shutdown(context) {
            klipper_shutdown!("Command request", read_tim5());
        }
    }

    /// Transition to shutdown state: disable motor, notify host.
    ///
    /// Returns `true` if this call actually entered shutdown (first call).
    /// Returns `false` if already in shutdown. The caller should send
    /// `klipper_shutdown!` only when this returns `true`, so each call
    /// site provides a unique static string reason.
    fn enter_shutdown(context: &mut State) -> bool {
        use core::sync::atomic::Ordering::Relaxed;
        let entered = klipper_mcu::commands::lifecycle::enter_shutdown(context);
        if entered {
            TMC_DISABLE_SIGNAL.signal(());
            SHUTDOWN_ACTIVE.store(true, Relaxed);
            // Quiesce TriggerDomain: cancel timers, stop steppers
            with_trigger_domain(|td| {
                td.scheduler.clear();
                let tim5 = embassy_stm32::pac::TIM5;
                tim5.dier().modify(|w| w.set_ccie(1, false));
                // Stop physical steppers
                for &idx in PHYSICAL_STEPPER_INDICES {
                    stop_physical_stepper_no_lock(idx);
                    td.set_timer_active(idx, false);
                }
                // Clear transient stepper state
                for ss in td.stepper_stop.iter_mut() {
                    ss.need_reset = true;
                    ss.blocked_by_trigger = false;
                    ss.blocked_by_trsync = None;
                }
                // Clear transient trsync state (preserve OID maps).
                // Trigger each trsync to clear can_trigger per spec,
                // then cancel timers and pending reports.
                for (trsync_idx, ts) in td.trsync.iter_mut().enumerate() {
                    // Find OID for this index (if configured)
                    let oid = td
                        .trsync_by_oid
                        .iter()
                        .find(|(_, &idx)| idx == trsync_idx)
                        .map(|(&oid, _)| oid)
                        .unwrap_or(0);
                    ts.trigger(oid, 0, 0); // Sets can_trigger=false
                    ts.cancel_timers();
                    ts.clear_pending_report();
                }
                // Clear stop signals and endstop homing
                for sig in td.stop_signals.iter_mut() {
                    *sig = None;
                }
                for es in td.endstops.iter_mut() {
                    es.reset();
                }
            });
            REPORT_PENDING.store(false, Relaxed);
            HOMING_TRIGGERED.store(0xFF, Relaxed);
            HOMING_STATE.store(0, Relaxed);
        }
        entered
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
    #[allow(dead_code)] // Used by klipper-usb's check_reboot() path
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
        if let Err(e) = klipper_mcu::commands::stepper::config_stepper(context, oid) {
            defmt::error!("config_stepper: {}", e);
            if enter_shutdown(context) {
                klipper_shutdown!("config_stepper failed", read_tim5());
            }
            return;
        }
        defmt::info!("config_stepper: oid={} (pins ignored, TMC4671 mode)", oid);
    }

    /// Enqueue a trapezoidal move segment for a stepper.
    ///
    /// Routes through the hardware step/dir SPSC queue. Moves are consumed
    /// by the TIM5 CC1 ISR (P3) which toggles STEP/DIR GPIO pins.
    #[klipper_command]
    pub fn queue_step(context: &mut State, oid: u8, interval: u32, count: u16, add: i16) {
        use klipper_mcu::commands::stepper::{queue_step_hw, QueueStepResult};

        let m = klipper_mcu::stepper::StepMove::new(interval, count, add);

        // Lock TriggerDomain across need_reset check, direction state,
        // enqueue, and arm — all atomic per spec.
        let result = with_trigger_domain(|td| {
            let need_reset = td.stepper_stop[0].need_reset;

            // SAFETY: P2 is the sole producer. Queue len read is safe from P2.
            let queue_was_empty = unsafe { (*core::ptr::addr_of_mut!(STEP_QUEUE)).is_empty() };

            match queue_step_hw(need_reset, count, queue_was_empty) {
                Ok(QueueStepResult::Discarded) => Ok(()),
                Err(e) => Err(e),
                Ok(_result) => {
                    // Prepare move with direction state inside the lock.
                    // Must happen after need_reset check to avoid advancing
                    // direction bookkeeping for discarded moves.
                    // SAFETY: P2_STEPPER is only accessed from P2 command handlers.
                    let m = unsafe { (*core::ptr::addr_of_mut!(P2_STEPPER)).prepare_move(m) };

                    // Push to SPSC queue
                    // SAFETY: P2 is the sole producer.
                    let ok = unsafe { (*core::ptr::addr_of_mut!(STEP_QUEUE)).enqueue(m).is_ok() };
                    if !ok {
                        return Err(klipper_mcu::commands::CommandError::QueueFull);
                    }

                    // Arm timer if the motor is ready and not already active
                    if !td.timer_active(0)
                        && MOTOR_ARMED.load(core::sync::atomic::Ordering::Relaxed)
                    {
                        let now = read_tim5();
                        arm_step_timer(now.wrapping_add(1));
                        td.set_timer_active(0, true);
                    }
                    Ok(())
                }
            }
        });

        if let Err(e) = result {
            defmt::error!("queue_step: oid={} {}", oid, e);
            if enter_shutdown(context) {
                klipper_shutdown!("queue_step failed", read_tim5());
            }
        }
    }

    /// Set the direction for subsequent steps on a stepper.
    ///
    /// Direction is captured per-move in `queue_step` via `prepare_move()`.
    #[klipper_command]
    pub fn set_next_step_dir(_context: &mut State, _oid: u8, dir: u8) {
        // SAFETY: P2_STEPPER is only accessed from P2 command handlers.
        unsafe {
            (*core::ptr::addr_of_mut!(P2_STEPPER)).set_next_step_dir(dir != 0);
        }
    }

    /// Reset the step clock reference for a stepper.
    ///
    /// Only valid when the stepper is inactive (timer not armed).
    /// Triggers shutdown if called while active (matches Klipper stepper.c).
    #[klipper_command]
    pub fn reset_step_clock(context: &mut State, _oid: u8, clock: u32) {
        let result: Result<(), klipper_mcu::commands::CommandError> = with_trigger_domain(|td| {
            klipper_mcu::commands::stepper::reset_step_clock_hw(td.stepper_stop[0].timer_active)?;

            // Drain any stale items from a previous stop
            // SAFETY: Timer is not active (checked above), P3 is quiesced.
            unsafe { while (*core::ptr::addr_of_mut!(STEP_QUEUE)).dequeue().is_some() {} }

            // Set clock base on P2 stepper state
            // SAFETY: P2_STEPPER is only accessed from P2 command handlers.
            unsafe {
                (*core::ptr::addr_of_mut!(P2_STEPPER)).reset_step_clock(clock);
            }

            // Only clear need_reset if not blocked by an active trigger.
            // When blocked_by_trigger is set, in-flight drip commands must
            // not re-enable stepping. The flag is cleared by the host's
            // trsync_trigger command which arrives after the host has
            // received the trsync_state notification.
            if !td.stepper_stop[0].blocked_by_trigger {
                td.stepper_stop[0].need_reset = false;
            }
            Ok(())
        });

        if let Err(e) = result {
            defmt::error!("reset_step_clock: {}", e);
            if enter_shutdown(context) {
                klipper_shutdown!("Can't reset time when stepper active", read_tim5());
            }
        }
    }

    /// Report the current position of a stepper.
    #[klipper_command]
    pub fn stepper_get_position(_context: &mut State, oid: u8) {
        let pos = STEPPER_POSITION.load(core::sync::atomic::Ordering::Relaxed);
        klipper_reply!(stepper_position, oid: u8 = oid, pos: i32 = pos);
    }

    /// Report the commanded (target) position of a stepper.
    ///
    /// For hardware step/dir this is equivalent to `stepper_get_position`
    /// since the firmware tracks only the logical step count.
    #[klipper_command]
    pub fn stepper_get_commanded_position(_context: &mut State, oid: u8) {
        let pos = STEPPER_POSITION.load(core::sync::atomic::Ordering::Relaxed);
        klipper_reply!(stepper_position, oid: u8 = oid, pos: i32 = pos);
    }

    /// Link a stepper to a TrSync so the stepper stops when the TrSync triggers.
    ///
    /// Multiple steppers can be linked to the same TrSync. When a trigger fires,
    /// all linked steppers are stopped.
    #[klipper_command]
    pub fn stepper_stop_on_trigger(context: &mut State, oid: u8, trsync_oid: u8) {
        // Resolve stepper OID to index via State (outside critical section)
        let stepper_idx = match context.steppers_by_oid.get(&oid) {
            Some(&idx) => idx,
            None => {
                defmt::error!("stepper_stop_on_trigger: unknown stepper oid={}", oid);
                if enter_shutdown(context) {
                    klipper_shutdown!("stepper_stop_on_trigger: unknown stepper", read_tim5());
                }
                return;
            }
        };

        let result = with_trigger_domain(|td| {
            // Validate trsync OID exists
            if !td.trsync_by_oid.contains_key(&trsync_oid) {
                return Err(());
            }
            // Reject if this stepper already has an active stop signal,
            // matching Klipper C where each stepper has a single stop_signal
            // slot and trsync_add_signal() shuts down if already active.
            for signal in td.stop_signals.iter().flatten() {
                if signal.stepper_idx == stepper_idx {
                    return Err(());
                }
            }
            // Find empty slot and insert
            for slot in td.stop_signals.iter_mut() {
                if slot.is_none() {
                    *slot = Some(klipper_mcu::trigger::StopSignal {
                        trsync_oid,
                        stepper_idx,
                    });
                    return Ok(());
                }
            }
            Err(()) // No free slots
        });

        if result.is_err() {
            defmt::error!(
                "stepper_stop_on_trigger: oid={} trsync_oid={} failed",
                oid,
                trsync_oid,
            );
            if enter_shutdown(context) {
                klipper_shutdown!("stepper_stop_on_trigger failed", read_tim5());
            }
        } else {
            // Begin homing I-suppression: transition Idle -> SuppressPending.
            // HOMING_TRIGGERED is NOT set here — it is set by
            // TriggerDomain::trigger() when the endstop/expire fires.
            use core::sync::atomic::Ordering::Relaxed;
            let _ = HOMING_STATE.compare_exchange(0, 1, Relaxed, Relaxed);
        }
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
        if let Err(e) = klipper_mcu::commands::digital_out::config_digital_out(context, oid) {
            defmt::error!("config_digital_out: {}", e);
            if enter_shutdown(context) {
                klipper_shutdown!("config_digital_out failed", read_tim5());
            }
            return;
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
        match klipper_mcu::commands::digital_out::queue_digital_out(context, oid, on_ticks) {
            Ok(enabled) => {
                if !tmc_send(TmcRequest::SetEnable(enabled)) && enter_shutdown(context) {
                    klipper_shutdown!("Motor enable/disable lost (queue full)", read_tim5());
                }
            }
            Err(e) => {
                defmt::error!("queue_digital_out: {}", e);
                if enter_shutdown(context) {
                    klipper_shutdown!("queue_digital_out failed", read_tim5());
                }
            }
        }
    }

    /// Immediately update a digital output value.
    #[klipper_command]
    pub fn update_digital_out(context: &mut State, oid: u8, value: u8) {
        match klipper_mcu::commands::digital_out::update_digital_out(context, oid, value) {
            Ok(enabled) => {
                if !tmc_send(TmcRequest::SetEnable(enabled)) && enter_shutdown(context) {
                    klipper_shutdown!("Motor enable/disable lost (queue full)", read_tim5());
                }
            }
            Err(e) => {
                defmt::error!("update_digital_out: {}", e);
                if enter_shutdown(context) {
                    klipper_shutdown!("update_digital_out failed", read_tim5());
                }
            }
        }
    }

    // ---- GPIO infrastructure for endstop pin reading ----

    /// Number of GPIO ports (A..E) on the STM32F407.
    const GPIO_NUM_PORTS: u8 = 5;

    /// Pin enum value representing the first non-GPIO virtual pin.
    /// Pin enum: PA0=0..PA15=15, PB0=16..PB15=31, ..., PE0=64..PE15=79.
    const GPIO_MAX_PIN_VALUE: u8 = GPIO_NUM_PORTS * 16;

    /// Per-endstop pin identifier (enum value from `config_endstop`).
    /// `0xFF` means no pin configured.
    static ENDSTOP_PIN_IDS: [core::sync::atomic::AtomicU8; klipper_mcu::state::NUM_ENDSTOPS] = [
        core::sync::atomic::AtomicU8::new(0xFF),
        core::sync::atomic::AtomicU8::new(0xFF),
        core::sync::atomic::AtomicU8::new(0xFF),
        core::sync::atomic::AtomicU8::new(0xFF),
    ];

    /// Get the PAC GPIO port for the given port index (0=A, 1=B, ..., 4=E).
    ///
    /// Returns `None` for out-of-range port indices.
    fn gpio_port(port: u8) -> Option<embassy_stm32::pac::gpio::Gpio> {
        match port {
            0 => Some(embassy_stm32::pac::GPIOA),
            1 => Some(embassy_stm32::pac::GPIOB),
            2 => Some(embassy_stm32::pac::GPIOC),
            3 => Some(embassy_stm32::pac::GPIOD),
            4 => Some(embassy_stm32::pac::GPIOE),
            _ => None,
        }
    }

    /// Configure a GPIO pin as input with optional pull-up.
    ///
    /// Enables the GPIO port clock, sets the pin to input mode, and configures
    /// the pull-up/pull-down resistor. Uses raw PAC register access to avoid
    /// the type-erasure problem with embassy-stm32 `Input` pins.
    ///
    /// `pin_value` is the pin enum value (PA0=0, PB0=16, etc.).
    fn gpio_configure_input(pin_value: u8, pull_up: bool) {
        let port_idx = pin_value / 16;
        let pin_idx = (pin_value % 16) as usize;

        // Enable GPIO port clock on AHB1
        let rcc = embassy_stm32::pac::RCC;
        rcc.ahb1enr().modify(|w| match port_idx {
            0 => w.set_gpioaen(true),
            1 => w.set_gpioben(true),
            2 => w.set_gpiocen(true),
            3 => w.set_gpioden(true),
            4 => w.set_gpioeen(true),
            _ => {}
        });
        // Brief delay for clock to stabilize
        cortex_m::asm::delay(2);

        if let Some(gpio) = gpio_port(port_idx) {
            use embassy_stm32::pac::gpio::vals;
            // Set pin as input
            gpio.moder()
                .modify(|w| w.set_moder(pin_idx, vals::Moder::INPUT));
            // Configure pull-up or floating
            let pull = if pull_up {
                vals::Pupdr::PULL_UP
            } else {
                vals::Pupdr::FLOATING
            };
            gpio.pupdr().modify(|w| w.set_pupdr(pin_idx, pull));
        }
    }

    /// Read the current state of a GPIO pin.
    ///
    /// `pin_value` is the pin enum value (PA0=0, PB0=16, etc.).
    /// Returns `false` if the pin value is out of range.
    fn gpio_read_pin(pin_value: u8) -> bool {
        if pin_value >= GPIO_MAX_PIN_VALUE {
            return false;
        }
        let port_idx = pin_value / 16;
        let pin_idx = (pin_value % 16) as usize;

        gpio_port(port_idx).is_some_and(|gpio| {
            use embassy_stm32::pac::gpio::vals;
            gpio.idr().read().idr(pin_idx) == vals::Idr::HIGH
        })
    }

    // ---- TrSync and Endstop commands ----

    /// Register a TrSync (trigger synchronization) object.
    ///
    /// Allocates the next free TrSync slot and maps the Klipper OID to it.
    #[klipper_command]
    pub fn config_trsync(context: &mut State, oid: u8) {
        let result = with_trigger_domain(|td| {
            if td.trsync_by_oid.contains_key(&oid) {
                return Err(());
            }
            let idx = td.trsync_by_oid.len();
            if idx >= klipper_mcu::state::NUM_TRSYNC {
                return Err(());
            }
            td.trsync_by_oid.insert(oid, idx).map_err(|_| ())?;
            Ok(())
        });
        if result.is_err() {
            defmt::error!("config_trsync: oid={}", oid);
            if enter_shutdown(context) {
                klipper_shutdown!("config_trsync failed", read_tim5());
            }
            return;
        }
        defmt::info!("config_trsync: oid={}", oid);
    }

    /// Arm a TrSync for synchronized triggering.
    ///
    /// Configures the reporting interval and timeout reason, then transitions
    /// the TrSync to the Armed state. Schedules the first report timer event.
    #[klipper_command]
    pub fn trsync_start(
        context: &mut State,
        oid: u8,
        report_clock: u32,
        report_ticks: u32,
        expire_reason: u8,
    ) {
        let result = with_trigger_domain(|td| {
            let idx = match td.trsync_by_oid.get(&oid) {
                Some(&idx) => idx,
                None => return Err(()),
            };
            // Clear old stop signals for this trsync OID
            for slot in td.stop_signals.iter_mut() {
                if let Some(signal) = slot {
                    if signal.trsync_oid == oid {
                        *slot = None;
                    }
                }
            }
            // Clear blocked_by_trigger for steppers blocked by this trsync
            for stepper in td.stepper_stop.iter_mut() {
                if stepper.blocked_by_trsync == Some(oid) {
                    stepper.blocked_by_trigger = false;
                    stepper.blocked_by_trsync = None;
                }
            }
            // Cancel any existing scheduler entries for this trsync
            td.scheduler.cancel(|k| {
                matches!(k,
                    klipper_mcu::scheduler::SchedulerEventKind::TrSyncExpire(i)
                    | klipper_mcu::scheduler::SchedulerEventKind::TrSyncReport(i)
                    if *i == idx
                )
            });
            // Arm the runtime
            td.trsync[idx].arm(report_clock, report_ticks, expire_reason);
            // Schedule first report timer if report_ticks > 0
            if report_ticks > 0 {
                td.scheduler
                    .schedule(klipper_mcu::scheduler::SchedulerEvent {
                        wake_clock: report_clock,
                        kind: klipper_mcu::scheduler::SchedulerEventKind::TrSyncReport(idx),
                    });
            }
            td.reprogram_cc2();
            Ok(())
        });
        if result.is_err() {
            defmt::error!("trsync_start: oid={}", oid);
            if enter_shutdown(context) {
                klipper_shutdown!("trsync_start failed", read_tim5());
            }
        }
    }

    /// Update the expiry deadline for a TrSync.
    ///
    /// Schedules the expire timer event. The TrSync will auto-trigger with
    /// the configured expire_reason if the clock reaches this value before
    /// a manual trigger.
    #[klipper_command]
    pub fn trsync_set_timeout(context: &mut State, oid: u8, clock: u32) {
        let result = with_trigger_domain(|td| {
            let idx = match td.trsync_by_oid.get(&oid) {
                Some(&idx) => idx,
                None => return Err(()),
            };
            if td.trsync[idx].can_trigger() {
                td.trsync[idx].set_timeout(clock);
                // Cancel any existing expire event and schedule a new one
                td.scheduler.cancel(|k| {
                    matches!(k,
                        klipper_mcu::scheduler::SchedulerEventKind::TrSyncExpire(i) if *i == idx
                    )
                });
                td.scheduler
                    .schedule(klipper_mcu::scheduler::SchedulerEvent {
                        wake_clock: clock,
                        kind: klipper_mcu::scheduler::SchedulerEventKind::TrSyncExpire(idx),
                    });
                td.reprogram_cc2();
            }
            Ok(())
        });
        if result.is_err() {
            defmt::error!("trsync_set_timeout: oid={}", oid);
            if enter_shutdown(context) {
                klipper_shutdown!("trsync_set_timeout failed", read_tim5());
            }
        }
    }

    /// Manually trigger a TrSync and clean up.
    ///
    /// Mirrors Klipper C's `command_trsync_trigger`: attempts to fire the
    /// trigger (no-op if already triggered), then unconditionally stops
    /// report/expire timers, clears blocked_by_trigger on steppers, and
    /// sends one final `trsync_state` reply with `can_trigger=0`.
    #[klipper_command]
    pub fn trsync_trigger(context: &mut State, oid: u8, reason: u8) {
        let trigger_reason = with_trigger_domain(|td| {
            let idx = match td.trsync_by_oid.get(&oid) {
                Some(&idx) => idx,
                None => return Err(()),
            };
            // Attempt trigger via TriggerDomain::trigger() which dispatches
            // stop signals if this is the first trigger (no-op if already triggered)
            let _result = td.trigger(oid, reason, 0);
            let trigger_reason = td.trsync[idx].trigger_reason();

            // Cancel scheduler entries for this trsync
            td.scheduler.cancel(|k| {
                matches!(k,
                    klipper_mcu::scheduler::SchedulerEventKind::TrSyncExpire(i)
                    | klipper_mcu::scheduler::SchedulerEventKind::TrSyncReport(i)
                    if *i == idx
                )
            });
            // Clear pending report to prevent stale async delivery
            td.trsync[idx].clear_pending_report();
            td.trsync[idx].cancel_timers();

            // Clear stop signals for this trsync OID
            for slot in td.stop_signals.iter_mut() {
                if let Some(signal) = slot {
                    if signal.trsync_oid == oid {
                        *slot = None;
                    }
                }
            }

            // Clear blocked_by_trigger for steppers blocked by this trsync
            // (leaves need_reset true until reset_step_clock)
            for stepper in td.stepper_stop.iter_mut() {
                if stepper.blocked_by_trsync == Some(oid) {
                    stepper.blocked_by_trigger = false;
                    stepper.blocked_by_trsync = None;
                }
            }

            td.reprogram_cc2();
            Ok(trigger_reason)
        });

        match trigger_reason {
            Ok(trigger_reason) => {
                // Clear homing trigger sentinel and advance suppression
                // state machine. If P1 is in state 2 (Suppressing), the
                // host trsync_trigger cleanup must advance it to state 3
                // (RestorePending) so I-suppression is released even if
                // P1 hasn't polled HOMING_TRIGGERED yet.
                use core::sync::atomic::Ordering::Relaxed;
                HOMING_TRIGGERED.store(0xFF, Relaxed);
                let _ = HOMING_STATE.compare_exchange(2, 3, Relaxed, Relaxed);
                // Send final trsync_state reply (can_trigger=0, clock=0)
                klipper_reply!(
                    trsync_state,
                    oid: u8 = oid,
                    can_trigger: u8 = 0,
                    trigger_reason: u8 = trigger_reason,
                    clock: u32 = 0
                );
            }
            Err(()) => {
                defmt::error!("trsync_trigger: unknown oid={}", oid);
                if enter_shutdown(context) {
                    klipper_shutdown!("trsync_trigger failed", read_tim5());
                }
            }
        }
    }

    /// Register an endstop object.
    ///
    /// Allocates the next free Endstop slot and maps the Klipper OID to it.
    /// Configures the specified GPIO pin as input with optional pull-up
    /// resistor for endstop switch reading.
    #[klipper_command]
    pub fn config_endstop(context: &mut State, oid: u8, pin: u8, pull_up: u8) {
        let result = with_trigger_domain(|td| {
            if td.endstop_by_oid.contains_key(&oid) {
                return Err(());
            }
            let idx = td.endstop_by_oid.len();
            if idx >= klipper_mcu::state::NUM_ENDSTOPS {
                return Err(());
            }
            td.endstop_by_oid.insert(oid, idx).map_err(|_| ())?;
            Ok(idx)
        });
        let idx = match result {
            Ok(idx) => idx,
            Err(()) => {
                defmt::error!("config_endstop: oid={}", oid);
                if enter_shutdown(context) {
                    klipper_shutdown!("config_endstop failed", read_tim5());
                }
                return;
            }
        };

        // Configure GPIO pin as input (only for real GPIO pins, not virtual)
        if pin < GPIO_MAX_PIN_VALUE {
            gpio_configure_input(pin, pull_up != 0);
            ENDSTOP_PIN_IDS[idx].store(pin, core::sync::atomic::Ordering::Relaxed);
            defmt::info!(
                "config_endstop: oid={} idx={} pin=P{}{}{}",
                oid,
                idx,
                ['A', 'B', 'C', 'D', 'E'][(pin / 16) as usize],
                pin % 16,
                if pull_up != 0 { " (pull-up)" } else { "" }
            );
        } else {
            defmt::info!(
                "config_endstop: oid={} idx={} pin={} (virtual, no GPIO)",
                oid,
                idx,
                pin
            );
        }
    }

    /// Arm an endstop for homing.
    ///
    /// Configures sampling parameters and links to a TrSync for trigger
    /// notification. Schedules the first endstop sample event via the
    /// trigger scheduler (TIM5 CC2).
    #[klipper_command]
    #[allow(clippy::too_many_arguments)]
    pub fn endstop_home(
        context: &mut State,
        oid: u8,
        clock: u32,
        sample_ticks: u32,
        sample_count: u8,
        rest_ticks: u32,
        pin_value: u8,
        trsync_oid: u8,
        trigger_reason: u8,
    ) {
        let result = with_trigger_domain(|td| {
            let idx = match td.endstop_by_oid.get(&oid) {
                Some(&idx) => idx,
                None => return Err(()),
            };
            // Zero sample_count is a disarm request — no trsync needed
            if sample_count != 0 && !td.trsync_by_oid.contains_key(&trsync_oid) {
                return Err(());
            }
            // Look up pin_id from config-time storage
            let pin_id = ENDSTOP_PIN_IDS[idx].load(core::sync::atomic::Ordering::Relaxed);
            // Cancel existing scheduler entry for this endstop
            td.scheduler.cancel(|k| {
                matches!(k,
                    klipper_mcu::scheduler::SchedulerEventKind::EndstopSample(i) if *i == idx
                )
            });
            // Configure the endstop runtime
            td.endstops[idx].start_homing(
                clock,
                sample_ticks,
                sample_count,
                rest_ticks,
                pin_value != 0,
                trsync_oid,
                trigger_reason,
                pin_id,
            );
            // Schedule first sample event if homing
            if sample_count > 0 {
                td.scheduler
                    .schedule(klipper_mcu::scheduler::SchedulerEvent {
                        wake_clock: clock,
                        kind: klipper_mcu::scheduler::SchedulerEventKind::EndstopSample(idx),
                    });
            }
            td.reprogram_cc2();
            Ok(())
        });
        if result.is_err() {
            defmt::error!("endstop_home: oid={}", oid);
            if enter_shutdown(context) {
                klipper_shutdown!("endstop_home failed", read_tim5());
            }
        }
    }

    /// Query the current state of an endstop.
    ///
    /// Returns homing status, next sample clock, and current pin value.
    #[klipper_command]
    pub fn endstop_query_state(context: &mut State, oid: u8) {
        // Brief lock to copy state
        let query = with_trigger_domain(|td| {
            let idx = match td.endstop_by_oid.get(&oid) {
                Some(&idx) => idx,
                None => return Err(()),
            };
            let (homing, next_clock) = td.endstops[idx].query_state();
            let pin_id = ENDSTOP_PIN_IDS[idx].load(core::sync::atomic::Ordering::Relaxed);
            Ok((homing, next_clock, pin_id))
        });
        match query {
            Ok((homing, next_clock, pin_id)) => {
                let pin_state = if pin_id != 0xFF {
                    gpio_read_pin(pin_id)
                } else {
                    false
                };
                klipper_reply!(
                    endstop_state,
                    oid: u8 = oid,
                    homing: u8 = u8::from(homing),
                    next_clock: u32 = next_clock,
                    pin_value: u8 = u8::from(pin_state)
                );
            }
            Err(()) => {
                defmt::error!("endstop_query_state: unknown oid={}", oid);
                if enter_shutdown(context) {
                    klipper_shutdown!("endstop_query_state failed", read_tim5());
                }
            }
        }
    }

    /// SPI configuration stub (TMC4671 SPI is handled internally).
    #[klipper_command]
    pub fn config_spi(_oid: u8, _pin: u32, _cs_active_high: u8) {
        klipper_mcu::commands::spi::config_spi(_oid, _pin, _cs_active_high);
    }

    /// SPI configuration without chip-select stub.
    #[klipper_command]
    pub fn config_spi_without_cs(_oid: u8) {
        klipper_mcu::commands::spi::config_spi_without_cs(_oid);
    }

    /// SPI bus assignment stub.
    #[klipper_command]
    pub fn spi_set_bus(_oid: u8, _spi_bus: u32, _mode: u32, _rate: u32) {
        klipper_mcu::commands::spi::spi_set_bus(_oid, _spi_bus, _mode, _rate);
    }

    /// SPI shutdown message configuration stub.
    #[klipper_command]
    pub fn config_spi_shutdown(_oid: u8, _spi_oid: u8, _shutdown_msg: &[u8]) {
        klipper_mcu::commands::spi::config_spi_shutdown(_oid, _spi_oid, _shutdown_msg);
    }

    /// SPI send stub (data is accepted but not forwarded).
    #[klipper_command]
    pub fn spi_send(_oid: u8, _data: &[u8]) {
        klipper_mcu::commands::spi::spi_send(_oid, _data);
    }

    // ---- TMC4671 register access commands ----

    /// Send a request to the tmc_control task.
    ///
    /// Returns `true` if the request was enqueued, `false` if the channel
    /// was full or not yet initialized. Callers with async reply contracts
    /// must send a failure reply to the host when this returns `false`.
    fn tmc_send(req: TmcRequest) -> bool {
        critical_section::with(|cs| {
            if let Some(sender) = TMC_REQUEST_SENDER.borrow(cs).borrow_mut().as_mut() {
                if sender.try_send(req).is_err() {
                    defmt::warn!("TMC request channel full, command dropped");
                    return false;
                }
                return true;
            }
            false
        })
    }

    /// Trigger TMC4671 initialization sequence.
    #[cfg(feature = "dev")]
    #[klipper_command]
    pub fn tmc_init(_context: &mut State, _oid: u8) {
        tmc_send(foci_core::tmc::tmc_init());
    }

    /// Test motor rotation by sweeping PHI_E_EXT through N revolutions.
    #[cfg(feature = "dev")]
    #[klipper_command]
    pub fn tmc_test_spin(_context: &mut State, _oid: u8, revolutions: u8) {
        tmc_send(foci_core::tmc::tmc_test_spin(revolutions));
    }

    /// Write a TMC4671 register via SPI.
    #[cfg(feature = "dev")]
    #[klipper_command]
    pub fn tmc_write_register(_context: &mut State, _oid: u8, addr: u8, value: u32) {
        tmc_send(foci_core::tmc::tmc_write_register(addr, value));
    }

    /// Read a TMC4671 register via SPI.
    ///
    /// The response is sent asynchronously as a tmc_register_value reply.
    #[cfg(feature = "dev")]
    #[klipper_command]
    pub fn tmc_read_register(_context: &mut State, oid: u8, addr: u8) {
        if !tmc_send(foci_core::tmc::tmc_read_register(oid, addr)) {
            defmt::warn!("tmc_read_register dropped: addr={:#04x}", addr);
            // No reply — let the Python query_command timeout naturally.
        }
    }

    /// Set motor run current. The firmware converts milliamps to
    /// PID_TORQUE_FLUX_LIMITS register counts using the board-specific
    /// current scaling factor.
    #[klipper_command]
    pub fn tmc_set_current(context: &mut State, _oid: u8, run_ma: u32) {
        if !tmc_send(foci_core::tmc::tmc_set_current(run_ma)) && enter_shutdown(context) {
            klipper_shutdown!("tmc_set_current lost (queue full)", read_tim5());
        }
    }

    /// Configure ABN encoder resolution and compute step width.
    ///
    /// Uses `tmc_set_encoder_hw` to compute the step width from PPR and
    /// pole pairs, then stores it in the `STEP_WIDTH_VALUE` atomic for
    /// the TMC4671 STEP_WIDTH register. The TMC4671 position PID operates
    /// on phi_e, not encoder counts:
    ///   step_width = 65536 * pole_pairs / quadrature_ppr
    #[klipper_command]
    pub fn tmc_set_encoder(context: &mut State, _oid: u8, channel: u8, ppr: u32) {
        if channel >= TMC_MOTOR_COUNT {
            if enter_shutdown(context) {
                klipper_shutdown!("Invalid TMC channel", read_tim5());
            }
            return;
        }
        // Use _hw variant: returns step_width for TMC4671 STEP_WIDTH register
        let (req, step_width) = foci_core::tmc::tmc_set_encoder_hw(ppr, 50);
        STEP_WIDTH_VALUE.store(step_width, core::sync::atomic::Ordering::Relaxed);
        defmt::info!("channel={} ppr={} step_width={}", channel, ppr, step_width);
        if !tmc_send(req) && enter_shutdown(context) {
            klipper_shutdown!("tmc_set_encoder lost (queue full)", read_tim5());
        }
    }

    /// Set ABN encoder count direction. Inverted=1 flips the count
    /// direction so the position PID has correct (negative) feedback.
    /// Depends on motor/encoder wiring — not board-specific.
    #[klipper_command]
    pub fn tmc_set_encoder_dir(context: &mut State, _oid: u8, channel: u8, invert: u8) {
        if channel >= TMC_MOTOR_COUNT {
            if enter_shutdown(context) {
                klipper_shutdown!("Invalid TMC channel", read_tim5());
            }
            return;
        }
        if !tmc_send(foci_core::tmc::tmc_set_encoder_dir(invert != 0)) && enter_shutdown(context) {
            klipper_shutdown!("tmc_set_encoder_dir lost (queue full)", read_tim5());
        }
    }

    /// Trigger TMC4671 self-test diagnostic via the commissioning engine.
    ///
    /// Only one commissioning operation may be in flight at a time.
    /// Concurrent requests are rejected.
    #[klipper_command]
    pub fn foci_selftest(_context: &mut State, oid: u8) {
        use core::sync::atomic::Ordering::Relaxed;
        if COMMISSION_IN_FLIGHT.swap(true, Relaxed) {
            defmt::warn!("foci_selftest rejected: commission in flight");
            klipper_reply!(
                foci_commission_phase,
                oid: u8 = oid,
                phase: u8 = 0,
                status: u8 =
                    foci_core::autotune::CommissionError::AlreadyRunning.status_code()
            );
            return;
        }
        if !tmc_send(foci_core::tmc::tmc_selftest(oid)) {
            COMMISSION_IN_FLIGHT.store(false, Relaxed);
            klipper_reply!(
                foci_commission_phase,
                oid: u8 = oid,
                phase: u8 = 0,
                status: u8 =
                    foci_core::autotune::CommissionError::QueueFull.status_code()
            );
        }
    }

    /// Calibrate the TMC4671: ADC offset calibration, encoder alignment,
    /// and closed-loop entry via the commissioning engine.
    ///
    /// Must be called before motor enable. Rejected if motor is already
    /// enabled or another commissioning operation is in flight.
    #[klipper_command]
    pub fn foci_calibrate(_context: &mut State, oid: u8) {
        use core::sync::atomic::Ordering::Relaxed;
        // Pre-flight: STEP_WIDTH_VALUE must have been populated by a prior
        // tmc_set_encoder command. If it hasn't, fail before touching hardware
        // — otherwise calibration would succeed but the motor would silently
        // refuse to respond to step pulses. The engine's ClosedLoopEntry
        // phase also checks this via backend.step_width().
        if STEP_WIDTH_VALUE.load(Relaxed) <= 0 {
            defmt::error!(
                "foci_calibrate rejected: STEP_WIDTH not configured (host must call tmc_set_encoder first)"
            );
            klipper_reply!(
                foci_calibrate_result,
                oid: u8 = oid,
                status: u8 = CalibrationError::ConfigFault.status_code(),
                adc_i0: u16 = 0,
                adc_i1: u16 = 0,
                encoder_count: u32 = 0
            );
            return;
        }
        if COMMISSION_IN_FLIGHT.swap(true, Relaxed) {
            defmt::warn!("foci_calibrate rejected: commission in flight");
            klipper_reply!(
                foci_calibrate_result,
                oid: u8 = oid,
                status: u8 = CalibrationError::InternalError.status_code(),
                adc_i0: u16 = 0,
                adc_i1: u16 = 0,
                encoder_count: u32 = 0
            );
            return;
        }
        if !tmc_send(foci_core::tmc::tmc_calibrate(oid)) {
            COMMISSION_IN_FLIGHT.store(false, Relaxed);
            klipper_reply!(
                foci_calibrate_result,
                oid: u8 = oid,
                status: u8 = CalibrationError::InternalError.status_code(),
                adc_i0: u16 = 0,
                adc_i1: u16 = 0,
                encoder_count: u32 = 0
            );
        }
    }

    /// Dump all TMC4671 registers via klipper_output protocol.
    ///
    /// The firmware reads registers internally and streams results as
    /// FOCI:DUMP protocol lines. Replaces Python-side register reads.
    #[klipper_command]
    pub fn foci_dump_registers(_context: &mut State, oid: u8) {
        if !tmc_send(foci_core::tmc::tmc_dump_registers(oid)) {
            defmt::warn!("foci_dump_registers dropped");
            // No reply — let the Python wait loop timeout with an
            // explicit "register dump timed out" error.
        }
    }

    /// Query trace buffer metadata.
    #[klipper_command]
    pub fn foci_trace_info(_context: &mut State, oid: u8) {
        if !tmc_send(foci_core::tmc::tmc_trace_info(oid)) {
            defmt::warn!("trace_info: TMC channel full");
        }
    }

    /// Fetch a trace sample by offset. gen must match current generation.
    #[klipper_command]
    pub fn foci_trace_fetch(_context: &mut State, oid: u8, offset: u16, gen: u8) {
        if !tmc_send(foci_core::tmc::tmc_trace_fetch(oid, offset, gen)) {
            defmt::warn!("trace_fetch: TMC channel full");
        }
    }

    /// Set inner loop PI gains from host (explicit override or saved values).
    #[klipper_command]
    pub fn tmc_set_pid_gains(
        context: &mut State,
        _oid: u8,
        flux_p: u16,
        flux_i: u16,
        torque_p: u16,
        torque_i: u16,
    ) {
        if !tmc_send(foci_core::tmc::tmc_set_pid_gains(
            flux_p, flux_i, torque_p, torque_i,
        )) && enter_shutdown(context)
        {
            klipper_shutdown!("tmc_set_pid_gains lost (queue full)", read_tim5());
        }
    }

    /// Set velocity biquad filter cutoff frequency from host config.
    #[klipper_command]
    pub fn tmc_set_velocity_filter(_context: &mut State, _oid: u8, filter_hz: u16) {
        VELOCITY_FILTER_HZ.store(filter_hz, core::sync::atomic::Ordering::Relaxed);
        defmt::info!("Velocity filter: {} Hz", filter_hz);
    }

    /// Set torque biquad filter cutoff frequency from host config.
    #[klipper_command]
    pub fn tmc_set_torque_filter(_context: &mut State, _oid: u8, filter_hz: u16) {
        TORQUE_FILTER_HZ.store(filter_hz, core::sync::atomic::Ordering::Relaxed);
        defmt::info!("Torque filter: {} Hz", filter_hz);
    }

    /// Set position target biquad filter cutoff frequency from host config.
    #[klipper_command]
    pub fn tmc_set_position_filter(_context: &mut State, _oid: u8, filter_hz: u16) {
        POSITION_FILTER_HZ.store(filter_hz, core::sync::atomic::Ordering::Relaxed);
        defmt::info!("Position filter: {} Hz", filter_hz);
    }

    /// Set flux biquad filter cutoff frequency from host config.
    #[klipper_command]
    pub fn tmc_set_flux_filter(_context: &mut State, _oid: u8, filter_hz: u16) {
        FLUX_FILTER_HZ.store(filter_hz, core::sync::atomic::Ordering::Relaxed);
        defmt::info!("Flux filter: {} Hz", filter_hz);
    }

    /// Set position/velocity PI gains from host config.
    #[klipper_command]
    pub fn tmc_set_position_gains(
        context: &mut State,
        _oid: u8,
        position_p: u16,
        position_i: u16,
        velocity_p: u16,
        velocity_i: u16,
    ) {
        if !tmc_send(foci_core::tmc::tmc_set_position_gains(
            position_p, position_i, velocity_p, velocity_i,
        )) && enter_shutdown(context)
        {
            klipper_shutdown!("tmc_set_position_gains lost (queue full)", read_tim5());
        }
    }

    /// Enable or disable velocity feedforward from host config.
    #[klipper_command]
    pub fn tmc_set_velocity_feedforward(_context: &mut State, _oid: u8, enable: u8) {
        VELOCITY_FEEDFORWARD.store(enable != 0, core::sync::atomic::Ordering::Relaxed);
        defmt::info!("Velocity feedforward: {}", enable != 0);
    }

    /// Set PID velocity limit from host config.
    /// Caps position PID output to prevent integrator-driven runaway.
    /// Firmware default: 500,000. Override via printer.cfg pid_velocity_limit.
    #[klipper_command]
    pub fn tmc_set_velocity_limit(_context: &mut State, _oid: u8, limit: u32) {
        VELOCITY_LIMIT_CFG.store(limit, core::sync::atomic::Ordering::Relaxed);
        defmt::info!("Velocity limit: {}", limit);
    }

    /// Start Stage 1 commissioning (diagnostics + current tune + closed-loop entry).
    #[klipper_command]
    pub fn foci_commission(_context: &mut State, oid: u8, profile: u8) {
        use core::sync::atomic::Ordering::Relaxed;
        if COMMISSION_IN_FLIGHT.swap(true, Relaxed) {
            klipper_reply!(
                foci_commission_phase,
                oid: u8 = oid,
                phase: u8 = 0,
                status: u8 = foci_core::autotune::CommissionError::AlreadyRunning.status_code()
            );
            return;
        }
        if !tmc_send(foci_core::tmc::tmc_commission(oid, profile)) {
            COMMISSION_IN_FLIGHT.store(false, Relaxed);
            klipper_reply!(
                foci_commission_phase,
                oid: u8 = oid,
                phase: u8 = 0,
                status: u8 = foci_core::autotune::CommissionError::QueueFull.status_code()
            );
        }
    }

    /// Parameters for `foci_tune` grouped into a struct to satisfy
    /// clippy's `too_many_arguments` lint. The `#[klipper_command]`
    /// handler signature is protocol-bound and cannot be changed.
    struct FociTuneParams {
        oid: u8,
        profile: u8,
        mode: u8,
        inner_lambda: u32,
        theta_e: u32,
        current_ringing: u8,
        current_bw: u32,
    }

    /// Inner implementation of `foci_tune`.
    fn foci_tune_inner(params: FociTuneParams) {
        use core::sync::atomic::Ordering::Relaxed;
        let FociTuneParams {
            oid,
            profile,
            mode,
            inner_lambda,
            theta_e,
            current_ringing,
            current_bw,
        } = params;
        if COMMISSION_IN_FLIGHT.swap(true, Relaxed) {
            klipper_reply!(
                foci_commission_phase,
                oid: u8 = oid,
                phase: u8 = 0,
                status: u8 = foci_core::autotune::CommissionError::AlreadyRunning.status_code()
            );
            return;
        }
        if !tmc_send(foci_core::tmc::tmc_tune(
            oid,
            profile,
            mode,
            inner_lambda,
            theta_e,
            current_ringing,
            current_bw,
        )) {
            COMMISSION_IN_FLIGHT.store(false, Relaxed);
            klipper_reply!(
                foci_commission_phase,
                oid: u8 = oid,
                phase: u8 = 0,
                status: u8 = foci_core::autotune::CommissionError::QueueFull.status_code()
            );
        }
    }

    /// Start Stage 2 installed tuning (mechanical ID + velocity/position/filters).
    #[klipper_command]
    pub fn foci_tune(
        oid: u8,
        profile: u8,
        mode: u8,
        inner_lambda: u32,
        theta_e: u32,
        current_ringing: u8,
        current_bw: u32,
    ) {
        foci_tune_inner(FociTuneParams {
            oid,
            profile,
            mode,
            inner_lambda,
            theta_e,
            current_ringing,
            current_bw,
        });
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
    }

    /// Read the STM32 96-bit unique device ID and format as a hex string.
    ///
    /// The UID is at 0x1FFF_7A10 on STM32F4. Returns a 24-character hex string.
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
        tmc_channel: rtic_sync::channel::Channel<TmcRequest, TMC_CHANNEL_SIZE> =
            rtic_sync::channel::Channel::new(),
        uid_str: [u8; 24] = [0; 24],
    ])]
    fn init(ctx: init::Context) -> (Shared, Local) {
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

        // TIM5: free-running 32-bit counter at 84 MHz (APB1 timer clock).
        // Sole Klipper clock source and step timer base.
        {
            let rcc = embassy_stm32::pac::RCC;
            rcc.apb1enr().modify(|w| w.set_tim5en(true));
            // Two cycles for clock to stabilize
            cortex_m::asm::delay(2);
            let tim5 = embassy_stm32::pac::TIM5;
            tim5.psc().write_value(0); // No prescaler: 84 MHz
            tim5.arr().write_value(0xFFFF_FFFF); // Full 32-bit range
            tim5.egr().write(|w| w.set_ug(true)); // Force update to load PSC/ARR
            tim5.cr1().modify(|w| w.set_cen(true)); // Start counting
        }

        // TIM5 CC1: configure for step timer interrupts (not armed yet).
        // CC1 interrupt is enabled when the first queue_step arms the timer.
        // NVIC unmask is also deferred to first arm — unmasking here would
        // fire the ISR from the UIF flag set by EGR.UG above.
        {
            let tim5 = embassy_stm32::pac::TIM5;
            tim5.dier().write_value(Default::default()); // All interrupts off
            tim5.sr().write_value(Default::default()); // Clear all pending flags
        }

        // PB11: STEP GPIO (push-pull output, low initial)
        // PA0: DIR GPIO (push-pull output, low initial)
        {
            use embassy_stm32::pac::gpio::vals;
            // Enable GPIOB and GPIOA clocks
            let rcc = embassy_stm32::pac::RCC;
            rcc.ahb1enr().modify(|w| w.set_gpioaen(true));
            rcc.ahb1enr().modify(|w| w.set_gpioben(true));

            // PB11: push-pull output, very high speed
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

            // PA0: push-pull output, very high speed
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

        // PE10: Brake chopper control (active-high assumed).
        // Drive LOW immediately to prevent the brake resistor from firing
        // while the STM32 is booting. On STM32 reset, GPIO defaults to
        // analog/high-impedance which the brake FET may interpret as ON.
        {
            let _brake_pin = Output::new(p.PE10, Level::Low, Speed::Low);
            core::mem::forget(_brake_pin); // Keep pin configured, don't drop
        }

        // PE8: TMC4671 STATUS pin (input, active high).
        // Pin mode configured here; EXTI interrupt enabled later in
        // tmc_control after STATUS_MASK is written to the TMC4671.
        {
            use embassy_stm32::gpio::{Input, Pull};
            let _status_pin = Input::new(p.PE8, Pull::Down);
            core::mem::forget(_status_pin);
        }

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
        let spi_dev = ExclusiveDevice::new(spi1, spi_cs, CycleDelay).unwrap();
        let tmc = Tmc4671Spi::new(spi_dev);

        // TMC register request channel
        let (tmc_tx, tmc_rx) = ctx.local.tmc_channel.split();
        critical_section::with(|cs| {
            TMC_REQUEST_SENDER.borrow(cs).replace(Some(tmc_tx));
        });

        defmt::info!("SPI1 configured: 8 MHz, Mode 3, DMA");

        // USB OTG FS: configure D+/D- GPIO pins (PA11=D-, PA12=D+)
        // Alternate function 10 for USB OTG FS on STM32F4
        {
            use embassy_stm32::gpio::{AfType, Flex, Pull};
            let mut dm = Flex::new(p.PA11);
            dm.set_as_af_unchecked(10, AfType::input(Pull::None));
            let mut dp = Flex::new(p.PA12);
            dp.set_as_af_unchecked(10, AfType::input(Pull::None));
            // Pins must stay configured — don't drop the Flex handles
            core::mem::forget(dm);
            core::mem::forget(dp);
        }

        // Enable USB OTG FS peripheral clock
        embassy_stm32::pac::RCC
            .ahb2enr()
            .modify(|w| w.set_usb_otg_fsen(true));
        // Brief delay for clock to stabilize
        cortex_m::asm::delay(100);

        let uid = read_stm32_uid(ctx.local.uid_str);

        // SAFETY: Called exactly once, before any other klipper-usb function.
        // base_addr is the USB_OTG_FS peripheral base on STM32F407.
        unsafe {
            klipper_usb::usb_init(&klipper_usb::UsbConfig {
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

        // Enable DWT CYCCNT for first-sample SPI timing measurement.
        // CYCCNT is disabled on reset; requires DEMCR.TRCENA + DWT.CYCCNTENA.
        {
            let dcb = unsafe { &*cortex_m::peripheral::DCB::PTR };
            unsafe {
                dcb.demcr.modify(|v| v | (1 << 24)); // TRCENA
            }
            let dwt = unsafe { &*cortex_m::peripheral::DWT::PTR };
            unsafe {
                dwt.cyccnt.write(0);
                dwt.ctrl.modify(|v| v | 1); // CYCCNTENA
            }
        }

        // Initialize trace buffer in CCM RAM
        unsafe {
            // Use addr_of_mut! to avoid creating a reference to the mutable static
            let ptr = core::ptr::addr_of_mut!(TRACE_BUF).cast::<foci_core::trace::TraceBuffer>();
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
                core::mem::size_of::<foci_core::trace::TraceBuffer>()
            );
            core::ptr::write(ptr, {
                let mut buf = core::mem::zeroed::<foci_core::trace::TraceBuffer>();
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
                let reg = tmc4671::registers::Register::new(addr);
                if tmc.write_register(reg, value).await.is_err() {
                    defmt::error!("TMC4671 SPI write failed: addr={:#04x}", addr);
                }
            }
            #[cfg(feature = "dev")]
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
            TmcRequest::SetCurrent { run_ma } => {
                use tmc4671::registers::*;
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
                use tmc4671::registers::*;
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
                use tmc4671::registers::*;
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
                for &addr in foci_core::dump::DUMP_REGISTERS {
                    // Yield between SPI reads to let USB task drain TX buffer
                    Mono::delay(1.millis()).await;

                    let reg = tmc4671::registers::Register::new(addr);
                    let value = match tmc.read_register(reg).await {
                        Ok(v) => v,
                        Err(_) => {
                            defmt::error!("TMC4671 SPI read failed: addr={:#04x}", addr);
                            0
                        }
                    };
                    klipper_reply!(
                        foci_dump_value,
                        oid: u8 = oid,
                        addr: u8 = addr,
                        value: u32 = value
                    );
                }
                // Sub-registers (INTERIM, PID_ERROR) use synthetic addrs 0x80+
                for &(synthetic_addr, sub_reg) in foci_core::dump::DUMP_SUB_REGISTERS {
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
                    klipper_reply!(
                        foci_dump_value,
                        oid: u8 = oid,
                        addr: u8 = synthetic_addr,
                        value: u32 = value
                    );
                }
                Mono::delay(1.millis()).await;
                klipper_reply!(foci_dump_done, oid: u8 = oid);
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
                klipper_reply!(
                    foci_trace_info_result,
                    oid: u8 = oid,
                    state: u8 = info.state,
                    gen: u8 = info.generation,
                    preset: u8 = info.preset,
                    count: u16 = info.count,
                    phase: u8 = info.phase_id,
                    sample_period_us: u16 = info.sample_period_us,
                    dropped: u16 = info.dropped
                );
            }
            TmcRequest::TraceFetch { oid, offset, gen } => {
                let buf = unsafe { trace_buf() };
                match buf.fetch(oid, offset, gen) {
                    foci_core::trace::FetchResult::Ok(ring_idx) => {
                        let data = if buf.preset() == foci_core::trace::PRESET_FULL {
                            buf.sample_bytes_full(ring_idx)
                        } else {
                            buf.sample_bytes_fast(ring_idx)
                        };
                        klipper_reply!(
                            foci_trace_data,
                            oid: u8 = oid,
                            offset: u16 = offset,
                            status: u8 = 0,
                            data: &[u8] = data
                        );
                    }
                    foci_core::trace::FetchResult::Busy => {
                        klipper_reply!(
                            foci_trace_data,
                            oid: u8 = oid,
                            offset: u16 = offset,
                            status: u8 = 1,
                            data: &[u8] = &[]
                        );
                    }
                    foci_core::trace::FetchResult::Invalid => {
                        klipper_reply!(
                            foci_trace_data,
                            oid: u8 = oid,
                            offset: u16 = offset,
                            status: u8 = 2,
                            data: &[u8] = &[]
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
        use tmc4671::registers::*;

        let buf = unsafe { trace_buf() };
        if !buf.is_capturing() {
            return;
        }

        let preset = buf.preset();
        let engine_tick = buf.engine_tick();

        // Full preset decimation: skip odd ticks (500 Hz effective)
        if preset == foci_core::trace::PRESET_FULL && engine_tick % 2 != 0 {
            buf.tick_no_sample(false); // intentional decimation, not a drop
            return;
        }

        // Pre-read deadline check
        let budget_us = if preset == foci_core::trace::PRESET_FULL {
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

        if preset == foci_core::trace::PRESET_FULL {
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

            buf.push_full(foci_core::trace::TraceSampleFull {
                fast: foci_core::trace::TraceSampleFast {
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

            buf.push_fast(foci_core::trace::TraceSampleFast {
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
                    ctx.shared.app_state.lock(|state| {
                        if enter_shutdown(state) {
                            klipper_shutdown!("TMC4671 chip ID mismatch", read_tim5());
                        }
                    });
                    return;
                }
            }
            Err(_) => {
                defmt::error!("TMC4671 SPI communication failed");
                ctx.shared.app_state.lock(|state| {
                    if enter_shutdown(state) {
                        klipper_shutdown!("TMC4671 SPI failed at boot", read_tim5());
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
            use tmc4671::registers::*;
            let _ = tmc.write_register(ADC_VM_LIMITS, 0xFFFF_FFFF).await;
        }

        // Configure STATUS_MASK: route critical fault flags to the STATUS
        // output pin (PE8 EXTI). Only PLL lock loss and ADC current clipping
        // warrant immediate hardware interrupt — other flags are polled.
        {
            use tmc4671::registers::*;
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
        let mut stored_gains: Option<foci_core::pid::InnerLoopGains> = None;

        // Stored position/velocity PID gains from host.
        // Active commissioning engine. Set when a Commission/Tune
        // request arrives, cleared when the engine finishes or fails.
        let mut commission_engine: Option<crate::commission::CommissioningEngine> = None;

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
            use embassy_futures::select::{select, Either};

            // On shutdown, perform one-shot motor safe-state then idle until
            // shutdown clears. This catches USB disconnect / host crash where
            // no further commands arrive but the TMC4671 remains energized.
            if SHUTDOWN_ACTIVE.load(core::sync::atomic::Ordering::Relaxed) {
                if !shutdown_handled {
                    commission_engine.take();
                    foci_core::selftest::restore_safe_state(tmc, enable_pin).await;
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
                    foci_core::selftest::restore_safe_state(tmc, enable_pin).await;
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
                    crate::commission::StepResult::Continue => {
                        // Trace capture runs here — the main-loop trace call
                        // is unreachable due to 'continue'.
                        trace_sample_tick(tmc, engine_tick_deadline).await;
                        Mono::delay(1.millis()).await;
                        continue;
                    }
                    crate::commission::StepResult::PhaseChanged(phase_id) => {
                        let oid = engine.oid();
                        // Start fast trace on ClosedLoopEntry for Stage 1
                        // (Commission only, not Calibrate — spec excludes
                        // Calibrate-on-enable and foci_calibrate CLE paths).
                        if phase_id == foci_core::autotune::PhaseId::ClosedLoopEntry
                            && !engine.is_calibrate()
                            && !unsafe { trace_buf() }.is_capturing()
                        {
                            unsafe {
                                trace_buf().start(oid, foci_core::trace::PRESET_FAST);
                            }
                        }
                        unsafe { trace_buf().set_phase(phase_id.to_u8()) };
                        klipper_reply!(
                            foci_commission_phase,
                            oid: u8 = oid,
                            phase: u8 = phase_id.to_u8(),
                            status: u8 = 0
                        );
                        Mono::delay(1.millis()).await;
                        continue;
                    }
                    crate::commission::StepResult::InnerFailed(error) => {
                        unsafe { trace_buf().stop() };
                        let oid = engine.oid();
                        let is_calibrate = engine.is_calibrate();
                        defmt::error!("InnerFailed: calibrate={} error={}", is_calibrate, error,);
                        commission_engine = None;
                        COMMISSION_IN_FLIGHT.store(false, core::sync::atomic::Ordering::Relaxed);
                        foci_core::selftest::restore_safe_state(tmc, enable_pin).await;
                        if is_calibrate {
                            // Send raw CommissionError status code — the
                            // old CalibrationError mapping was lossy (4
                            // errors mapped to one code). Python handles
                            // the full CommissionError code range (1-17).
                            klipper_reply!(
                                foci_calibrate_result,
                                oid: u8 = oid,
                                status: u8 = error.status_code(),
                                adc_i0: u16 = 0,
                                adc_i1: u16 = 0,
                                encoder_count: u32 = 0
                            );
                        } else {
                            klipper_reply!(
                                foci_commission_phase,
                                oid: u8 = oid,
                                phase: u8 = 0,
                                status: u8 = error.status_code()
                            );
                        }
                    }
                    crate::commission::StepResult::OuterDone(outer) => {
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
                            use tmc4671::registers::*;

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
                            foci_core::selftest::restore_safe_state(tmc, enable_pin).await;
                            COMMISSION_IN_FLIGHT.store(false, R);
                            motor_enabled = false;
                            calibrated = false;
                            klipper_reply!(
                                foci_tune_result,
                                oid: u8 = oid,
                                status: u8 = foci_core::autotune::CommissionError::Spi
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
                                synth_position_i: u16 = 0
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
                            foci_core::autotune::CommissionStatus::Accepted => 0u8,
                            foci_core::autotune::CommissionStatus::AcceptedWithWarnings => 1,
                            foci_core::autotune::CommissionStatus::InnerOnly => 2,
                            foci_core::autotune::CommissionStatus::Failed(e) => e.status_code(),
                        };
                        klipper_reply!(
                            foci_tune_result,
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
                            synth_position_i: u16 = outer.synth_position_i
                        );
                    }
                    crate::commission::StepResult::OuterFailed(error) => {
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
                        use foci_core::autotune::CommissionError as CE;
                        let is_safety_fault =
                            matches!(error, CE::Spi | CE::Shutdown | CE::SafetyEnvelopeViolation);

                        let restore_failure: Option<CE> = if is_safety_fault {
                            Some(error)
                        } else if let Some(snapshot) = snapshot {
                            let mut restore_backend = OpenFFBoardBackend { tmc, enable_pin };
                            match crate::commission::CommissioningEngine::restore_entry_gains(
                                &snapshot,
                                &mut restore_backend,
                            )
                            .await
                            {
                                Ok(()) => {
                                    // Stability check with cooperative delay.
                                    Mono::delay(10.millis()).await;
                                    let pos1 = tmc
                                        .read_register(tmc4671::registers::PID_POSITION_ACTUAL)
                                        .await;
                                    Mono::delay(10.millis()).await;
                                    let pos2 = tmc
                                        .read_register(tmc4671::registers::PID_POSITION_ACTUAL)
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
                                .write_register(tmc4671::registers::MODE_RAMP_MODE_MOTION, 0)
                                .await;
                            foci_core::selftest::restore_safe_state(tmc, enable_pin).await;
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
                        klipper_reply!(
                            foci_commission_phase,
                            oid: u8 = oid,
                            phase: u8 = 0,
                            status: u8 = reply_status
                        );
                    }
                    crate::commission::StepResult::CalibrateDone(cal_result) => {
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
                            use tmc4671::registers::*;

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
                            foci_core::selftest::restore_safe_state(tmc, enable_pin).await;
                            COMMISSION_IN_FLIGHT
                                .store(false, core::sync::atomic::Ordering::Relaxed);
                            klipper_reply!(
                                foci_calibrate_result,
                                oid: u8 = oid,
                                status: u8 = CalibrationError::SpiError.status_code(),
                                adc_i0: u16 = 0,
                                adc_i1: u16 = 0,
                                encoder_count: u32 = 0
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
                        klipper_reply!(
                            foci_calibrate_result,
                            oid: u8 = oid,
                            status: u8 = 0,
                            adc_i0: u16 = cal_result.adc_i0,
                            adc_i1: u16 = cal_result.adc_i1,
                            encoder_count: u32 = cal_result.encoder_count
                        );
                    }
                    crate::commission::StepResult::SelftestDone => {
                        let oid = engine.oid();
                        commission_engine = None;
                        COMMISSION_IN_FLIGHT.store(false, core::sync::atomic::Ordering::Relaxed);
                        // Engine already restored safe state internally.
                        klipper_reply!(
                            foci_commission_phase,
                            oid: u8 = oid,
                            phase: u8 = 0,
                            status: u8 = 0
                        );
                    }
                    crate::commission::StepResult::CommissionDone(commission_result) => {
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
                                tmc4671::registers::PID_TORQUE_FLUX_LIMITS,
                                current_limit,
                            )
                            .await;

                        // Persist inner gains so future calibrate uses them.
                        let inner = &commission_result.inner;
                        stored_gains = Some(foci_core::pid::InnerLoopGains {
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
                            foci_core::autotune::CommissionStatus::Accepted => 0u8,
                            foci_core::autotune::CommissionStatus::AcceptedWithWarnings => 1,
                            foci_core::autotune::CommissionStatus::InnerOnly => 2,
                            foci_core::autotune::CommissionStatus::Failed(e) => e.status_code(),
                        };
                        defmt::info!("foci_commission complete: oid={} status={}", oid, status);
                        klipper_reply!(
                            foci_commission_result,
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
                            fallback_velocity_limit: u32 = commission_result.fallback_velocity_limit
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
                                commission_engine =
                                    Some(crate::commission::CommissioningEngine::new_calibrate(0));
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
                            foci_core::selftest::restore_safe_state(tmc, enable_pin).await;

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
                            klipper_reply!(
                                foci_calibrate_result,
                                oid: u8 = oid,
                                status: u8 = CalibrationError::AlreadyEnabled.status_code(),
                                adc_i0: u16 = 0,
                                adc_i1: u16 = 0,
                                encoder_count: u32 = 0
                            );
                            continue;
                        }
                        // Engine tick will drive ADC cal; CalibrateDone handler
                        // performs encoder alignment and closed-loop entry.
                        calibrated = false;
                        commission_engine =
                            Some(crate::commission::CommissioningEngine::new_calibrate(oid));
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
                        stored_gains = Some(foci_core::pid::InnerLoopGains {
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
                            klipper_reply!(
                                foci_commission_phase,
                                oid: u8 = oid,
                                phase: u8 = 0,
                                status: u8 =
                                    foci_core::autotune::CommissionError::MotorEnabled
                                        .status_code()
                            );
                            continue;
                        }
                        let current_limit =
                            CURRENT_LIMIT_REG.load(core::sync::atomic::Ordering::Relaxed);
                        if current_limit == 0 {
                            COMMISSION_IN_FLIGHT
                                .store(false, core::sync::atomic::Ordering::Relaxed);
                            klipper_reply!(
                                foci_commission_phase,
                                oid: u8 = oid,
                                phase: u8 = 0,
                                status: u8 =
                                    foci_core::autotune::CommissionError::NoCurrent
                                        .status_code()
                            );
                            continue;
                        }
                        calibrated = false;
                        commission_engine =
                            Some(crate::commission::CommissioningEngine::new_commission(
                                oid,
                                foci_core::autotune::CommissionProfile::from_u8(profile),
                            ));
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
                            klipper_reply!(
                                foci_commission_phase,
                                oid: u8 = oid,
                                phase: u8 = 0,
                                status: u8 =
                                    foci_core::autotune::CommissionError::EncoderNotAligned
                                        .status_code()
                            );
                            continue;
                        }
                        let current_limit =
                            CURRENT_LIMIT_REG.load(core::sync::atomic::Ordering::Relaxed);
                        if current_limit == 0 {
                            COMMISSION_IN_FLIGHT
                                .store(false, core::sync::atomic::Ordering::Relaxed);
                            klipper_reply!(
                                foci_commission_phase,
                                oid: u8 = oid,
                                phase: u8 = 0,
                                status: u8 =
                                    foci_core::autotune::CommissionError::NoCurrent
                                        .status_code()
                            );
                            continue;
                        }
                        let entry_snapshot = {
                            use core::sync::atomic::Ordering::Relaxed;
                            crate::commission::EntrySnapshot {
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
                        let outer_params = crate::commission::OuterTuneParams {
                            profile: foci_core::autotune::CommissionProfile::from_u8(profile),
                            mode: foci_core::autotune::OperatingMode::from_u8(mode),
                            inner_lambda_us,
                            theta_e_us,
                            current_ringing: current_ringing_count > 1,
                            current_bandwidth_hz,
                        };
                        commission_engine =
                            Some(crate::commission::CommissioningEngine::new_outer(
                                oid,
                                outer_params,
                                entry_snapshot,
                            ));
                        unsafe {
                            trace_buf().start(oid, foci_core::trace::PRESET_FULL);
                            trace_buf()
                                .set_phase(foci_core::autotune::PhaseId::MechanicalId.to_u8());
                        }
                    }
                    TmcRequest::SelfTest { oid } => {
                        if motor_enabled {
                            defmt::warn!("foci_selftest rejected: motor enabled");
                            COMMISSION_IN_FLIGHT
                                .store(false, core::sync::atomic::Ordering::Relaxed);
                            klipper_reply!(
                                foci_commission_phase,
                                oid: u8 = oid,
                                phase: u8 = 0,
                                status: u8 =
                                    foci_core::autotune::CommissionError::MotorEnabled
                                        .status_code()
                            );
                            continue;
                        }
                        // Engine tick drives selftest phases; SelftestDone
                        // handler sends the completion signal.
                        commission_engine =
                            Some(crate::commission::CommissioningEngine::new_selftest(oid));
                    }
                    req @ TmcRequest::SetEncoder { .. } => {
                        if handle_tmc_request(req, tmc).await.is_err() {
                            ctx.shared.app_state.lock(|state| {
                                if enter_shutdown(state) {
                                    klipper_shutdown!("TMC request validation failed", read_tim5());
                                }
                            });
                        }
                        // Compute feedforward scale factor from step_width and PWM config
                        velocity_scale = foci_core::tmc::compute_velocity_scale_factor(
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
                                    klipper_shutdown!("TMC request validation failed", read_tim5());
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
                    .read_register(tmc4671::registers::STATUS_FLAGS)
                    .await
                    .unwrap_or(0);
                defmt::error!("TMC4671 emergency disable: STATUS={:#010x}", flags);
                // Report shutdown to Klipper with a specific reason.
                const NOT_PLL_LOCKED: u32 = 1 << 19;
                const ADC_I_CLIPPED: u32 = 1 << 26;
                ctx.shared.app_state.lock(|state| {
                    if flags & NOT_PLL_LOCKED != 0 {
                        if enter_shutdown(state) {
                            klipper_shutdown!("TMC4671: PLL not locked (clock lost)", read_tim5());
                        }
                    } else if flags & ADC_I_CLIPPED != 0 {
                        if enter_shutdown(state) {
                            klipper_shutdown!(
                                "TMC4671: ADC current clipped (overcurrent)",
                                read_tim5()
                            );
                        }
                    } else if enter_shutdown(state) {
                        klipper_shutdown!("TMC4671 fault", read_tim5());
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
                    foci_core::tmc::compute_velocity_offset(velocity_scale, interval, forward)
                };

                let _ = tmc
                    .write_register(tmc4671::registers::PID_VELOCITY_OFFSET, offset as u32)
                    .await;
            } else if feedforward_was_active {
                // Feedforward just deactivated — clear stale offset
                let _ = tmc
                    .write_register(tmc4671::registers::PID_VELOCITY_OFFSET, 0)
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
                            .write_register(tmc4671::registers::PID_POSITION_P_POSITION_I, reg)
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
                            .write_register(tmc4671::registers::PID_POSITION_P_POSITION_I, reg)
                            .await;
                        // Clear position error integrator
                        let _ = tmc
                            .write_sub_register(
                                tmc4671::registers::PID_ERROR_PID_POSITION_ERROR_SUM,
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
                if let Ok(flags) = tmc.read_register(tmc4671::registers::STATUS_FLAGS).await {
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
        use tmc4671::registers::*;

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
        use tmc4671::registers::*;

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
        use tmc4671::registers::*;

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
        use tmc4671::registers::*;

        let pwm_maxcnt = TMC_CONFIG.pwm_maxcnt;

        // Velocity biquad.
        let filter_hz = VELOCITY_FILTER_HZ.load(Relaxed);
        if filter_hz > 0 {
            if let Some(coeffs) = foci_core::biquad::butterworth_lowpass(filter_hz, pwm_maxcnt) {
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
        }

        // Torque biquad.
        let filter_hz = TORQUE_FILTER_HZ.load(Relaxed);
        if filter_hz > 0 {
            if let Some(coeffs) = foci_core::biquad::butterworth_lowpass(filter_hz, pwm_maxcnt) {
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
        }

        // Position biquad.
        let filter_hz = POSITION_FILTER_HZ.load(Relaxed);
        if filter_hz > 0 {
            if let Some(coeffs) = foci_core::biquad::butterworth_lowpass(filter_hz, pwm_maxcnt) {
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
        }

        // Flux biquad.
        let filter_hz = FLUX_FILTER_HZ.load(Relaxed);
        if filter_hz > 0 {
            if let Some(coeffs) = foci_core::biquad::butterworth_lowpass(filter_hz, pwm_maxcnt) {
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
    }

    /// Stop hardware stepper from P1: hardware-only, no TriggerDomain lock.
    ///
    /// Performs physical quiesce (disable CC1, drain queue, reset ISR engine),
    /// updates the P1-readable mirror atomic, then spawns `stop_check` to
    /// update the authoritative `timer_active` inside TriggerDomain at P2.
    ///
    /// P1 must never enter TriggerDomain per spec.
    fn stepper_stop_from_p1() {
        stop_physical_stepper_no_lock(0);
        STEP_TIMER_ACTIVE_MIRROR.store(false, core::sync::atomic::Ordering::Relaxed);
        stop_check::spawn().ok();
    }

    /// Minimum lead time when arming TIM5 CC1 from thread context.
    ///
    /// If CC1 is programmed too close to the live CNT value, the compare can
    /// be missed before CC1IE is fully armed. TIM5 is free-running 32-bit at
    /// 84 MHz, so a missed match would not reoccur until wrap (~51 s later).
    const STEP_TIMER_ARM_MIN_LEAD_TICKS: u32 = 128;

    /// Arm the TIM5 CC1 step timer for a compare at `clock`.
    ///
    /// Ensures NVIC is unmasked, clamps too-close compares forward, then
    /// enables CC1 interrupt. Does NOT update TriggerDomain's timer_active
    /// — the caller must do that inside the critical section.
    fn arm_step_timer(clock: u32) {
        let tim5 = embassy_stm32::pac::TIM5;
        // Ensure NVIC is unmasked (idempotent after first call)
        unsafe { cortex_m::peripheral::NVIC::unmask(embassy_stm32::pac::Interrupt::TIM5) };
        let now = tim5.cnt().read();
        let min_clock = now.wrapping_add(STEP_TIMER_ARM_MIN_LEAD_TICKS);
        let clock = if (clock as i32).wrapping_sub(min_clock as i32) < 0 {
            min_clock
        } else {
            clock
        };
        tim5.ccr(0).write_value(clock);
        tim5.sr().modify(|w| w.set_ccif(0, false)); // Clear pending CC1
        tim5.dier().modify(|w| w.set_ccie(0, true)); // Enable CC1 interrupt
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
        use klipper_mcu::stepper::StepEventResult;

        let tim5 = embassy_stm32::pac::TIM5;

        // --- CC2: trigger scheduler (process before CC1) ---
        if tim5.sr().read().ccif(1) && tim5.dier().read().ccie(1) {
            tim5.sr().modify(|w| w.set_ccif(1, false));
            let mut need_report = false;
            with_trigger_domain(|td| {
                use klipper_mcu::scheduler::SchedulerEventKind;
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
                            use klipper_mcu::endstop::{EndstopPhase, EndstopSampleResult};
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
                                                klipper_mcu::scheduler::SchedulerEvent {
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
                                    td.scheduler
                                        .schedule(klipper_mcu::scheduler::SchedulerEvent {
                                            wake_clock: next_clock,
                                            kind: SchedulerEventKind::EndstopSample(endstop_idx),
                                        });
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
                                .find(|(_, &idx)| idx == trsync_idx)
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
                                .find(|(_, &idx)| idx == trsync_idx)
                                .map(|(&oid, _)| oid);
                            if let Some(oid) = oid {
                                if td.trsync[trsync_idx].can_trigger() {
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
                                            klipper_mcu::scheduler::SchedulerEvent {
                                                wake_clock: next_report,
                                                kind: SchedulerEventKind::TrSyncReport(trsync_idx),
                                            },
                                        );
                                    }
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
                // Active segment: execute one step
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

                let dir = if stepper.dir_state() { 1i32 } else { -1i32 };
                STEPPER_POSITION.fetch_add(dir, Relaxed);

                match stepper.step_event() {
                    StepEventResult::ScheduleNext { clock } => {
                        STEP_INTERVAL.store(stepper.current_interval(), Relaxed);
                        let now = tim5.cnt().read();
                        match stepper.check_deadline(clock, now, CLOCK_FREQ) {
                            Ok(compare) => {
                                let immediate = compare != clock;
                                if immediate {
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
        klipper_usb::irq_handler();
        USB_WAKE.signal(());
    }

    /// TMC4671 STATUS pin interrupt (PE8, EXTI line 8).
    ///
    /// Fires when any STATUS_MASK-enabled flag is asserted (not_PLL_locked,
    /// adc_i_clipped). Signals immediate motor disable; tmc_control reads
    /// STATUS_FLAGS on next tick to log the cause.
    #[task(binds = EXTI9_5, priority = 3)]
    fn tmc_status_irq(_ctx: tmc_status_irq::Context) {
        let exti = embassy_stm32::pac::EXTI;
        exti.pr(0).write(|w| w.set_line(8, true));

        // Immediately quiesce step generation (same P3, atomic)
        FAULT_ACTIVE.store(true, core::sync::atomic::Ordering::Relaxed);
        let tim5 = embassy_stm32::pac::TIM5;
        tim5.dier().modify(|w| w.set_ccie(0, false));

        TMC_DISABLE_SIGNAL.signal(());
        defmt::warn!("TMC4671 STATUS interrupt: fault detected, step timer disabled");
    }

    /// Single USB task replacing usb_pump, usb_receive, usb_send, usb_control.
    ///
    /// Handles EP0 setup, bulk RX, protocol dispatch, and bulk TX in one pass.
    /// No scheduling gap between receiving commands and sending acks.
    /// Katapult 1200-baud DTR detection is handled by klipper-usb internally.
    #[task(priority = 2, shared = [app_state])]
    async fn usb_task(mut ctx: usb_task::Context) {
        loop {
            USB_WAKE.wait().await;

            // Process pending USB events. Poll reads new bulk OUT data
            // and drains bulk IN. Loop while making progress — break if
            // the protocol can't consume any bytes (incomplete frame).
            while klipper_usb::poll() {
                let data = klipper_usb::rx_data();
                if data.is_empty() {
                    break;
                }
                let len = data.len().min(128);
                let mut local_buf = [0u8; 128];
                local_buf[..len].copy_from_slice(&data[..len]);

                // Feed to protocol — only consume what transport processed.
                // Partial frames at the end must stay in the RX buffer.
                let mut input = SliceInputBuffer::new(&local_buf[..len]);
                ctx.shared.app_state.lock(|state| {
                    crate::KLIPPER_TRANSPORT.receive(&mut input, state);
                });
                let consumed = len - input.available();
                klipper_usb::rx_consume(consumed);
                if consumed == 0 {
                    break; // No progress — wait for more USB data
                }
                // Yield to let report_task run when a trigger report is pending.
                // Both tasks are P2; yielding gives report_task a chance to
                // send the trsync_state reply before we process more commands.
                if REPORT_PENDING.load(core::sync::atomic::Ordering::Relaxed) {
                    break;
                }
            }
            // Final poll to flush any remaining TX
            klipper_usb::poll();
        }
    }

    /// Send pending trsync_state reports outside the TriggerDomain lock.
    ///
    /// Spawned by the TIM5 CC2 ISR when any trsync has a pending report.
    /// Drains all pending snapshots in one brief critical section, then
    /// sends replies at P2 without holding the lock.
    #[task(priority = 2)]
    async fn report_task(_ctx: report_task::Context) {
        use core::sync::atomic::Ordering::Relaxed;
        let mut snapshots: heapless::Vec<klipper_mcu::trigger::ReportSnapshot, 4> =
            heapless::Vec::new();
        with_trigger_domain(|td| {
            for ts in td.trsync.iter_mut() {
                if let Some(snap) = ts.take_report() {
                    let _ = snapshots.push(snap);
                }
            }
            REPORT_PENDING.store(false, Relaxed);
        });
        for snap in &snapshots {
            klipper_reply!(
                trsync_state,
                oid: u8 = snap.oid,
                can_trigger: u8 = snap.can_trigger,
                trigger_reason: u8 = snap.trigger_reason,
                clock: u32 = snap.clock
            );
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
        let mut last_clock = read_tim5();

        loop {
            Mono::delay(5000.millis()).await;

            let now = read_tim5();
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
        // Virtual pins for FOCI stepper control.
        // STEP/DIR are handled internally (no GPIO toggle).
        // ENA maps to the TMC4671 enable pin (PE7 on OpenFFBoard).
        STEP0,
        DIR0,
        ENA0,
    }
);

klipper_config_generate!(
    transport = crate::app::TRANSPORT_OUTPUT: crate::app::BufferTransportOutput,
    context = &'ctx mut klipper_mcu::state::State,
);
