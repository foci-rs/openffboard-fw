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
    use rtic_monotonics::systick::prelude::*;
    use rtic_sync::channel::{Receiver, Sender};
    use tmc4671::spi::Tmc4671Spi;

    bind_interrupts!(struct Irqs {
        DMA2_STREAM2 => embassy_stm32::dma::InterruptHandler<peripherals::DMA2_CH2>;
        DMA2_STREAM3 => embassy_stm32::dma::InterruptHandler<peripherals::DMA2_CH3>;
    });

    systick_monotonic!(Mono, 1_000);

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
        /// ADC offset values are out of the expected range (30000..=36000).
        AdcFault,
        /// ABN encoder not responding or count stuck at zero.
        EncoderFault,
        /// PID loop not tracking target (torque or position diverging).
        PidFault,
        /// Motor already enabled (calibration not needed).
        AlreadyEnabled,
        /// Internal firmware error (queue full, channel not initialized, etc.).
        InternalError,
    }

    impl CalibrationError {
        /// Map to a numeric status code for `foci_calibrate_response`.
        fn status_code(&self) -> u8 {
            match self {
                Self::SpiError => 1,
                Self::AdcFault => 2,
                Self::EncoderFault => 3,
                Self::PidFault => 4,
                Self::AlreadyEnabled => 5,
                Self::InternalError => 6,
            }
        }
    }

    /// Bridges selftest stage results to Klipper protocol replies.
    ///
    /// Each call to `report_stage` emits a `foci_selftest_result` reply
    /// back to the host with the stage number, status, and value.
    struct KlipperSelftestReporter {
        oid: u8,
    }

    impl foci_core::selftest::SelftestReporter for KlipperSelftestReporter {
        fn report_stage(&mut self, stage: u8, status: u8, value: u32) {
            klipper_reply!(
                foci_selftest_result,
                oid: u8 = self.oid,
                stage: u8 = stage,
                status: u8 = status,
                value: u32 = value
            );
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
    /// When true, the 1ms tick loop computes position delta and writes
    /// PID_VELOCITY_OFFSET. Applied immediately in the tick loop.
    static VELOCITY_FEEDFORWARD: core::sync::atomic::AtomicBool =
        core::sync::atomic::AtomicBool::new(false);

    /// Signal the tick loop to reset feedforward state (prev_position_target,
    /// first_tick_after_enable). Set by handle_tmc_request when encoder
    /// direction changes, consumed by the tick loop on next iteration.
    static FEEDFORWARD_RESET: core::sync::atomic::AtomicBool =
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

    /// Guard for single-in-flight selftest. Set at command ingress (P2),
    /// cleared after foci_selftest_done is emitted (P1).
    static SELFTEST_IN_FLIGHT: core::sync::atomic::AtomicBool =
        core::sync::atomic::AtomicBool::new(false);

    /// Signal tmc_control to drain all pending requests. Set by config_reset
    /// (P2), cleared by tmc_control (P1) after draining.
    static TMC_QUEUE_FLUSH: core::sync::atomic::AtomicBool =
        core::sync::atomic::AtomicBool::new(false);

    // --- Hardware step/dir state ---

    /// Step count (incremented by P3 ISR, read by P1/P2 for stepper_get_position).
    static STEPPER_POSITION: core::sync::atomic::AtomicI32 = core::sync::atomic::AtomicI32::new(0);

    /// Set by stepper_stop(), cleared by reset_step_clock. When true,
    /// queue_step discards moves. Starts false to match Klipper stepper.c
    /// (SF_NEED_RESET is NOT set by config_stepper — only by stepper_stop).
    static STEPPER_NEED_RESET: core::sync::atomic::AtomicBool =
        core::sync::atomic::AtomicBool::new(false);

    /// Set by P1 after calibration, cleared on disable/fault.
    /// Gates timer arming and ISR execution.
    static MOTOR_ARMED: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

    /// Set by fault EXTI, cleared by P1 fault handler after cleanup.
    static FAULT_ACTIVE: core::sync::atomic::AtomicBool =
        core::sync::atomic::AtomicBool::new(false);

    /// True when TIM5 CC1 is armed. Used by reset_step_clock to detect
    /// "called while active" (triggers shutdown).
    static TIMER_ACTIVE: core::sync::atomic::AtomicBool =
        core::sync::atomic::AtomicBool::new(false);

    /// STEP_WIDTH value computed by tmc_set_encoder_hw, written to TMC4671
    /// register 0x78 during closedloop entry.
    static STEP_WIDTH_VALUE: core::sync::atomic::AtomicI32 = core::sync::atomic::AtomicI32::new(0);

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
    }

    /// Full config reset: clear all OIDs, steppers, and shutdown state.
    ///
    /// Matches Klipper C `config_reset` (basecmd.c:262). Only available
    /// during shutdown. The host uses this as a fallback restart mechanism
    /// when hardware `reset` is not available (linux/PRU platforms).
    #[klipper_command(in_shutdown)]
    pub fn config_reset(context: &mut State) {
        if !context.mcu_state.is_shutdown() {
            if enter_shutdown(context) {
                klipper_shutdown!("config_reset only available when shutdown", read_tim5());
            }
            return;
        }
        klipper_mcu::commands::lifecycle::config_reset(context);
        // Clear firmware-side endstop pin assignments
        for pin_id in &ENDSTOP_PIN_IDS {
            pin_id.store(0xFF, core::sync::atomic::Ordering::Relaxed);
        }
        // Reset board-local control and protocol state
        CURRENT_LIMIT_REG.store(0, core::sync::atomic::Ordering::Relaxed);
        VELOCITY_FILTER_HZ.store(0, core::sync::atomic::Ordering::Relaxed);
        TORQUE_FILTER_HZ.store(0, core::sync::atomic::Ordering::Relaxed);
        POSITION_FILTER_HZ.store(0, core::sync::atomic::Ordering::Relaxed);
        FLUX_FILTER_HZ.store(0, core::sync::atomic::Ordering::Relaxed);
        VELOCITY_FEEDFORWARD.store(false, core::sync::atomic::Ordering::Relaxed);
        VELOCITY_LIMIT_CFG.store(500_000, core::sync::atomic::Ordering::Relaxed);
        FEEDFORWARD_RESET.store(false, core::sync::atomic::Ordering::Relaxed);
        ENCODER_REVERSED.store(false, core::sync::atomic::Ordering::Relaxed);
        STEPPER_POSITION_TARGET.store(0, core::sync::atomic::Ordering::Relaxed);
        SELFTEST_IN_FLIGHT.store(false, core::sync::atomic::Ordering::Relaxed);
        STATS_SEND_TIME.store(0, core::sync::atomic::Ordering::Relaxed);
        STATS_SEND_TIME_HIGH.store(0, core::sync::atomic::Ordering::Relaxed);
        // Signal tmc_control to drain stale requests from the pre-reset epoch
        TMC_QUEUE_FLUSH.store(true, core::sync::atomic::Ordering::Relaxed);
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
        let entered = klipper_mcu::commands::lifecycle::enter_shutdown(context);
        if entered {
            TMC_DISABLE_SIGNAL.signal(());
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
        use core::sync::atomic::Ordering::Relaxed;
        use klipper_mcu::commands::stepper::{queue_step_hw, QueueStepResult};

        let need_reset = STEPPER_NEED_RESET.load(Relaxed);

        // Check queue state before pushing (need len for first-move detection)
        // SAFETY: P2 is the sole producer. Queue len read is safe from P2.
        let queue_was_empty = unsafe { (*core::ptr::addr_of_mut!(STEP_QUEUE)).is_empty() };

        match queue_step_hw(need_reset, count, queue_was_empty) {
            Ok(QueueStepResult::Discarded) => return,
            Err(e) => {
                defmt::error!("queue_step: oid={} {}", oid, e);
                if enter_shutdown(context) {
                    klipper_shutdown!("queue_step failed", read_tim5());
                }
                return;
            }
            Ok(result) => {
                // Prepare move with direction state
                // SAFETY: P2_STEPPER is only accessed from P2 command handlers.
                let m = klipper_mcu::stepper::StepMove::new(interval, count, add);
                let m = unsafe { (*core::ptr::addr_of_mut!(P2_STEPPER)).prepare_move(m) };

                // Push to SPSC queue
                // SAFETY: P2 is the sole producer.
                let ok = unsafe { (*core::ptr::addr_of_mut!(STEP_QUEUE)).enqueue(m).is_ok() };
                if !ok {
                    defmt::error!("queue_step: queue full");
                    if enter_shutdown(context) {
                        klipper_shutdown!("queue_step: queue full", read_tim5());
                    }
                    return;
                }

                // Arm timer on first move if motor is armed.
                // Fire immediately — P3 ISR will pop the move and schedule.
                if matches!(result, QueueStepResult::QueuedFirstMove)
                    && MOTOR_ARMED.load(Relaxed)
                    && !TIMER_ACTIVE.load(Relaxed)
                {
                    let now = read_tim5();
                    arm_step_timer(now.wrapping_add(1));
                }
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
        use core::sync::atomic::Ordering::Relaxed;

        if let Err(e) =
            klipper_mcu::commands::stepper::reset_step_clock_hw(TIMER_ACTIVE.load(Relaxed))
        {
            defmt::error!("reset_step_clock: {}", e);
            if enter_shutdown(context) {
                klipper_shutdown!("Can't reset time when stepper active", read_tim5());
            }
            return;
        }

        // Drain any stale items from a previous stop
        // SAFETY: Timer is not active (checked above), P3 is quiesced.
        unsafe { while (*core::ptr::addr_of_mut!(STEP_QUEUE)).dequeue().is_some() {} }

        // Set clock base on P2 stepper state
        // SAFETY: P2_STEPPER is only accessed from P2 command handlers.
        unsafe {
            (*core::ptr::addr_of_mut!(P2_STEPPER)).reset_step_clock(clock);
        }

        STEPPER_NEED_RESET.store(false, Relaxed);
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
        if let Err(e) =
            klipper_mcu::commands::stepper::stepper_stop_on_trigger(context, oid, trsync_oid)
        {
            defmt::error!(
                "stepper_stop_on_trigger: oid={} trsync_oid={} {}",
                oid,
                trsync_oid,
                e
            );
            if enter_shutdown(context) {
                klipper_shutdown!("stepper_stop_on_trigger failed", read_tim5());
            }
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
                if !tmc_send(TmcRequest::SetEnable(enabled)) {
                    if enter_shutdown(context) {
                        klipper_shutdown!("Motor enable/disable lost (queue full)", read_tim5());
                    }
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
                if !tmc_send(TmcRequest::SetEnable(enabled)) {
                    if enter_shutdown(context) {
                        klipper_shutdown!("Motor enable/disable lost (queue full)", read_tim5());
                    }
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
        if let Err(e) = klipper_mcu::commands::trsync::config_trsync(context, oid) {
            defmt::error!("config_trsync: {}", e);
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
    /// the TrSync to the Armed state.
    #[klipper_command]
    pub fn trsync_start(
        context: &mut State,
        oid: u8,
        report_clock: u32,
        report_ticks: u32,
        expire_reason: u8,
    ) {
        if let Err(e) = klipper_mcu::commands::trsync::trsync_start(
            context,
            oid,
            report_clock,
            report_ticks,
            expire_reason,
        ) {
            defmt::error!("trsync_start: {}", e);
            if enter_shutdown(context) {
                klipper_shutdown!("trsync_start failed", read_tim5());
            }
        }
    }

    /// Update the expiry deadline for a TrSync.
    ///
    /// The TrSync will auto-trigger with the configured expire_reason if
    /// the clock reaches this value before a manual trigger.
    #[klipper_command]
    pub fn trsync_set_timeout(context: &mut State, oid: u8, clock: u32) {
        if let Err(e) = klipper_mcu::commands::trsync::trsync_set_timeout(context, oid, clock) {
            defmt::error!("trsync_set_timeout: {}", e);
            if enter_shutdown(context) {
                klipper_shutdown!("trsync_set_timeout failed", read_tim5());
            }
        }
    }

    /// Manually trigger a TrSync with the given reason code.
    ///
    /// If the trigger is accepted (TrSync was Armed), stops all steppers
    /// linked to this TrSync OID via stepper_stop_on_trigger.
    #[klipper_command]
    pub fn trsync_trigger(context: &mut State, oid: u8, reason: u8) {
        if let Err(e) = klipper_mcu::commands::trsync::trsync_trigger(context, oid, reason) {
            defmt::error!("trsync_trigger: {}", e);
            if enter_shutdown(context) {
                klipper_shutdown!("trsync_trigger failed", read_tim5());
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
        let idx = context.endstop_by_oid.len();
        if let Err(e) = klipper_mcu::commands::endstop::config_endstop(context, oid) {
            defmt::error!("config_endstop: {}", e);
            if enter_shutdown(context) {
                klipper_shutdown!("config_endstop failed", read_tim5());
            }
            return;
        }

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
    /// notification. Actual GPIO sampling runs in the tmc_control 1ms loop.
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
        if let Err(e) = klipper_mcu::commands::endstop::endstop_home(
            context,
            oid,
            clock,
            sample_ticks,
            sample_count,
            rest_ticks,
            pin_value != 0,
            trsync_oid,
            trigger_reason,
        ) {
            defmt::error!("endstop_home: {}", e);
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
        let (homing, next_clock) =
            match klipper_mcu::commands::endstop::endstop_query_state(context, oid) {
                Ok(result) => result,
                Err(e) => {
                    defmt::error!("endstop_query_state: {}", e);
                    if enter_shutdown(context) {
                        klipper_shutdown!("endstop_query_state failed", read_tim5());
                    }
                    return;
                }
            };
        // Read the live GPIO pin state for the endstop
        let pin_state = context
            .endstop_by_oid
            .get(&oid)
            .map(|&idx| {
                let pin_id = ENDSTOP_PIN_IDS[idx].load(core::sync::atomic::Ordering::Relaxed);
                if pin_id != 0xFF {
                    gpio_read_pin(pin_id)
                } else {
                    false
                }
            })
            .unwrap_or(false);
        klipper_reply!(
            endstop_state,
            oid: u8 = oid,
            homing: u8 = u8::from(homing),
            next_clock: u32 = next_clock,
            pin_value: u8 = u8::from(pin_state)
        );
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
        if !tmc_send(foci_core::tmc::tmc_set_current(run_ma)) {
            if enter_shutdown(context) {
                klipper_shutdown!("tmc_set_current lost (queue full)", read_tim5());
            }
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
        if !tmc_send(req) {
            if enter_shutdown(context) {
                klipper_shutdown!("tmc_set_encoder lost (queue full)", read_tim5());
            }
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
        if !tmc_send(foci_core::tmc::tmc_set_encoder_dir(invert != 0)) {
            if enter_shutdown(context) {
                klipper_shutdown!("tmc_set_encoder_dir lost (queue full)", read_tim5());
            }
        }
    }

    /// Trigger TMC4671 self-test diagnostic. Results stream via
    /// `foci_selftest_result` replies, completed with `foci_selftest_done`.
    ///
    /// Only one selftest may be in flight at a time. Concurrent requests
    /// are rejected with `SELFTEST_DONE_ABORTED`.
    #[klipper_command]
    pub fn foci_selftest(_context: &mut State, oid: u8) {
        use core::sync::atomic::Ordering::Relaxed;
        if SELFTEST_IN_FLIGHT.swap(true, Relaxed) {
            defmt::warn!("foci_selftest rejected: already in flight");
            klipper_reply!(
                foci_selftest_done,
                oid: u8 = oid,
                status: u8 = foci_core::selftest::SELFTEST_DONE_ABORTED
            );
            return;
        }
        if !tmc_send(foci_core::tmc::tmc_selftest(oid)) {
            SELFTEST_IN_FLIGHT.store(false, Relaxed);
            klipper_reply!(
                foci_selftest_done,
                oid: u8 = oid,
                status: u8 = foci_core::selftest::SELFTEST_DONE_ABORTED
            );
        }
    }

    /// Calibrate the TMC4671: ADC offset calibration, encoder alignment,
    /// and closed-loop entry. Sends `foci_calibrate_response` when complete.
    ///
    /// Must be called before motor enable. Rejected if motor is already enabled.
    #[klipper_command]
    pub fn foci_calibrate(_context: &mut State, oid: u8) {
        if !tmc_send(foci_core::tmc::tmc_calibrate(oid)) {
            klipper_reply!(
                foci_calibrate_response,
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
        )) {
            if enter_shutdown(context) {
                klipper_shutdown!("tmc_set_pid_gains lost (queue full)", read_tim5());
            }
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
        )) {
            if enter_shutdown(context) {
                klipper_shutdown!("tmc_set_position_gains lost (queue full)", read_tim5());
            }
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

    /// Trigger R/L measurement and PI gain computation.
    #[klipper_command]
    pub fn foci_autotune(_context: &mut State, oid: u8) {
        if !tmc_send(foci_core::tmc::tmc_autotune(oid)) {
            klipper_reply!(
                foci_autotune_result,
                oid: u8 = oid,
                status: u8 = 1,
                resistance_mohm: u32 = 0,
                inductance_uh: u32 = 0,
                flux_p: u16 = 0,
                flux_i: u16 = 0,
                torque_p: u16 = 0,
                torque_i: u16 = 0
            );
        }
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

        Mono::start(ctx.core.SYST, 168_000_000);

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
            TmcRequest::Autotune { .. } => {
                defmt::warn!("handle_tmc_request: Autotune handled by main loop");
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

        // Disable TMC4671 brake chopper until VM is measured. Both limits=0
        // disables the BRAKE output (datasheet section 4.7.12). Proper limits
        // are set later after measuring the actual bus voltage.
        {
            use tmc4671::registers::*;
            let _ = tmc.write_register(ADC_VM_LIMITS, 0).await;
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

        // Configure brake chopper thresholds from measured bus voltage.
        //
        // Sample ADC_VM_RAW at boot to measure supply voltage, then set
        // activation/deactivation thresholds as percentages of measured peak:
        //   HIGH = +15% above measured (activation)
        //   LOW  = +10% above measured (deactivation, 5% hysteresis)
        //
        // Margins follow industry servo drive practice (5-20% activation,
        // 1-5% hysteresis). References: ABB ACS880 (+15-19%), Rockwell
        // PF755 (+16%, 4% hysteresis), KEB F5 (+16%).
        //
        // Configured at boot (not during motor calibration) so protection
        // is active before Klipper connects. VM divider readings are
        // unaffected by PWM_CHOP state — no motor interaction needed.
        //
        // TMC4671 datasheet rev 2.08, section 4.7.12: BRAKE pin activates
        // per-PWM-cycle when ADC_VM > ADC_VM_LIMIT_HIGH, deactivates when
        // ADC_VM < ADC_VM_LIMIT_LOW.
        {
            use tmc4671::registers::*;

            // Ensure PWM_CHOP=0 before sampling (power-on default, but
            // explicit per project safety rules).
            let _ = tmc.write_register(PWM_SV_CHOP, 0).await;

            let mut vm_max: u16 = 0;
            for _ in 0..100_u32 {
                if let Ok(raw) = tmc.read_sub_register(ADC_AGPI_A_RAW_ADC_VM_RAW).await {
                    // ADC_VM_RAW is in bits[15:0] (datasheet section 7.2)
                    let vm = (raw & 0xFFFF) as u16;
                    if vm > vm_max {
                        vm_max = vm;
                    }
                }
                Mono::delay(1.millis()).await;
            }

            // Require a plausible reading to filter ADC noise floor.
            // Any real supply voltage produces counts well above 100.
            if vm_max > 100 {
                let high = vm_max as u32 * 115 / 100;
                let low = vm_max as u32 * 110 / 100;

                // Clamp to u16::MAX if overflow — fail-safe (max protection),
                // not fail-open (no protection).
                let high = high.min(u16::MAX as u32);
                let low = low.min(u16::MAX as u32);

                // Datasheet section 7.2, page 103: LOW = bits[15:0], HIGH = bits[31:16]
                let limits = (high << 16) | low;
                if tmc.write_register(ADC_VM_LIMITS, limits).await.is_err() {
                    defmt::warn!("Brake chopper: ADC_VM_LIMITS write failed");
                }
                defmt::info!(
                    "Brake chopper: VM peak={}, LOW={} (+10%), HIGH={} (+15%)",
                    vm_max,
                    low,
                    high,
                );
            } else {
                if tmc.write_register(ADC_VM_LIMITS, 0).await.is_err() {
                    defmt::warn!("Brake chopper: ADC_VM_LIMITS write failed");
                }
                defmt::warn!("Brake chopper disabled: no VM reading");
            }
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
        // None = use defaults during calibration.
        let mut stored_position_gains: Option<(u16, u16, u16, u16)> = None;

        // (Velocity feedforward and move boundary detection removed —
        // hardware step/dir eliminates the 1ms SPI position target write.)

        // Main loop: wait for channel message or 1ms timeout.
        // On message: handle request. On timeout: write position target.
        // Position is read from STEPPER_POSITION_TARGET atomic (updated
        // by usb_receive at P2) — no shared resource lock needed.
        loop {
            use embassy_futures::select::{select, Either};

            // Drain stale requests after config_reset
            if TMC_QUEUE_FLUSH.swap(false, core::sync::atomic::Ordering::Relaxed) {
                while rx.try_recv().is_ok() {}
                motor_enabled = false;
                calibrated = false;
                stored_gains = None;
                stored_position_gains = None;
                // Reset hardware stepper state
                MOTOR_ARMED.store(false, core::sync::atomic::Ordering::Relaxed);
                stepper_stop();
                STEPPER_POSITION.store(0, core::sync::atomic::Ordering::Relaxed);
                FAULT_ACTIVE.store(false, core::sync::atomic::Ordering::Relaxed);
                defmt::info!("TMC request queue flushed after config_reset");
            }

            match select(rx.recv(), Mono::delay(1.millis())).await {
                // Channel message received
                Either::First(Ok(req)) => match req {
                    TmcRequest::SetEnable(enabled) => {
                        if enabled {
                            if !calibrated {
                                // Auto-calibrate: Klipper may send enable before
                                // the Python module's foci_calibrate arrives due
                                // to queue_digital_out timing. Run calibration
                                // in-place instead of shutting down.
                                let current_limit =
                                    CURRENT_LIMIT_REG.load(core::sync::atomic::Ordering::Relaxed);
                                if current_limit == 0 {
                                    defmt::error!("Motor enable rejected: current limit not set");
                                    continue;
                                }
                                defmt::info!("Auto-calibrating on enable");
                                match tmc_run_calibration(
                                    tmc,
                                    enable_pin,
                                    current_limit,
                                    stored_gains,
                                    stored_position_gains,
                                )
                                .await
                                {
                                    Ok(_) => {
                                        calibrated = true;
                                    }
                                    Err(e) => {
                                        defmt::error!("Auto-calibration failed: {}", e);
                                        ctx.shared.app_state.lock(|state| {
                                            if enter_shutdown(state) {
                                                klipper_shutdown!(
                                                    "Calibration failed on enable",
                                                    read_tim5()
                                                );
                                            }
                                        });
                                        continue;
                                    }
                                }
                            }
                            motor_enabled = true;
                            enable_pin.set_high();

                            // Zero firmware position counter
                            STEPPER_POSITION.store(0, core::sync::atomic::Ordering::Relaxed);

                            // Arm stepper: allow TIM5 CC1 ISR to fire
                            MOTOR_ARMED.store(true, core::sync::atomic::Ordering::Relaxed);

                            // If queue already has moves (burst-drain scenario),
                            // arm timer to start processing them
                            let queue_has_moves =
                                unsafe { !(*core::ptr::addr_of_mut!(STEP_QUEUE)).is_empty() };
                            if queue_has_moves
                                && !TIMER_ACTIVE.load(core::sync::atomic::Ordering::Relaxed)
                            {
                                let now = read_tim5();
                                arm_step_timer(now.wrapping_add(1));
                            }

                            defmt::info!("Motor enabled (calibrated, step/dir armed)");
                        } else {
                            // Disable: stop stepping, kill motor
                            MOTOR_ARMED.store(false, core::sync::atomic::Ordering::Relaxed);
                            stepper_stop();

                            motor_enabled = false;
                            calibrated = false;
                            use tmc4671::registers::*;
                            let _ = tmc.write_register(MODE_RAMP_MODE_MOTION, 0).await;
                            let _ = tmc.write_register(PID_TORQUE_FLUX_TARGET, 0).await;
                            let _ = tmc.write_register(UQ_UD_EXT, 0).await;
                            let _ = tmc.write_register(PWM_SV_CHOP, 0).await;
                            enable_pin.set_low();

                            // Zero position counter
                            STEPPER_POSITION.store(0, core::sync::atomic::Ordering::Relaxed);

                            defmt::info!("TMC4671 motor disabled");
                        }
                    }
                    TmcRequest::Calibrate { oid } => {
                        const STATUS_OK: u8 = 0;

                        if motor_enabled {
                            defmt::warn!("foci_calibrate rejected: motor already enabled");
                            klipper_reply!(
                                foci_calibrate_response,
                                oid: u8 = oid,
                                status: u8 = CalibrationError::AlreadyEnabled.status_code(),
                                adc_i0: u16 = 0,
                                adc_i1: u16 = 0,
                                encoder_count: u32 = 0
                            );
                            continue;
                        }

                        // Pre-flight: verify TMC4671 is alive before calibration
                        let chip_id = tmc
                            .read_sub_register(tmc4671::registers::CHIPINFO_SI_TYPE)
                            .await
                            .unwrap_or(0);
                        if chip_id != tmc4671::CHIP_ID {
                            defmt::error!("foci_calibrate: chip ID mismatch: {:#010x}", chip_id);
                            klipper_reply!(
                                foci_calibrate_response,
                                oid: u8 = oid,
                                status: u8 = CalibrationError::SpiError.status_code(),
                                adc_i0: u16 = 0,
                                adc_i1: u16 = 0,
                                encoder_count: u32 = 0
                            );
                            continue;
                        }

                        // Run the three-stage calibration sequence
                        let result = tmc_run_calibration(
                            tmc,
                            enable_pin,
                            CURRENT_LIMIT_REG.load(core::sync::atomic::Ordering::Relaxed),
                            stored_gains,
                            stored_position_gains,
                        )
                        .await;

                        match result {
                            Ok((adc_i0, adc_i1, encoder_count)) => {
                                calibrated = true;
                                defmt::info!("foci_calibrate complete: oid={}", oid);
                                klipper_reply!(
                                    foci_calibrate_response,
                                    oid: u8 = oid,
                                    status: u8 = STATUS_OK,
                                    adc_i0: u16 = adc_i0,
                                    adc_i1: u16 = adc_i1,
                                    encoder_count: u32 = encoder_count
                                );
                            }
                            Err(e) => {
                                // Safety shutdown on calibration failure
                                calibrated = false;
                                use tmc4671::registers::*;
                                let _ = tmc.write_register(MODE_RAMP_MODE_MOTION, 0).await;
                                let _ = tmc.write_register(PID_TORQUE_FLUX_TARGET, 0).await;
                                let _ = tmc.write_register(PWM_SV_CHOP, 0).await;
                                enable_pin.set_low();
                                defmt::error!("foci_calibrate failed: oid={} error={}", oid, e);
                                klipper_reply!(
                                    foci_calibrate_response,
                                    oid: u8 = oid,
                                    status: u8 = e.status_code(),
                                    adc_i0: u16 = 0,
                                    adc_i1: u16 = 0,
                                    encoder_count: u32 = 0
                                );
                            }
                        }
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
                        stored_position_gains =
                            Some((position_p, position_i, velocity_p, velocity_i));
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
                    TmcRequest::Autotune { oid } => {
                        if motor_enabled {
                            klipper_reply!(
                                foci_autotune_result,
                                oid: u8 = oid,
                                status: u8 = 2,
                                resistance_mohm: u32 = 0,
                                inductance_uh: u32 = 0,
                                flux_p: u16 = 0,
                                flux_i: u16 = 0,
                                torque_p: u16 = 0,
                                torque_i: u16 = 0
                            );
                            continue;
                        }

                        let current_limit =
                            CURRENT_LIMIT_REG.load(core::sync::atomic::Ordering::Relaxed);
                        if current_limit == 0 {
                            klipper_reply!(
                                foci_autotune_result,
                                oid: u8 = oid,
                                status: u8 = 3,
                                resistance_mohm: u32 = 0,
                                inductance_uh: u32 = 0,
                                flux_p: u16 = 0,
                                flux_i: u16 = 0,
                                torque_p: u16 = 0,
                                torque_i: u16 = 0
                            );
                            continue;
                        }

                        // Autotune drives the motor — invalidate calibration
                        // state so the next enable re-runs the full sequence.
                        calibrated = false;

                        // Helper: check for hardware fault between measurement
                        // steps. The STATUS EXTI ISR queues TMC_DISABLE_SIGNAL
                        // but the main loop only polls it after the match arm
                        // returns. Check explicitly so faults during the
                        // multi-second measurement are caught promptly.
                        //
                        // try_take() consumes the signal, so re-signal after
                        // detection. The re-fired signal persists in the Signal
                        // until the main loop's emergency handler (bottom of the
                        // loop, after the select+match block) calls try_take()
                        // on the next iteration and runs shutdown reporting.
                        let fault_pending = || -> bool {
                            if TMC_DISABLE_SIGNAL.try_take().is_some() {
                                TMC_DISABLE_SIGNAL.signal(());
                                true
                            } else {
                                false
                            }
                        };

                        // Macro for autotune abort: safe state + reply + continue
                        macro_rules! autotune_abort {
                            ($status:expr, $r:expr, $l:expr) => {{
                                foci_core::selftest::restore_safe_state(tmc, enable_pin).await;
                                klipper_reply!(
                                    foci_autotune_result,
                                    oid: u8 = oid,
                                    status: u8 = $status,
                                    resistance_mohm: u32 = $r,
                                    inductance_uh: u32 = $l,
                                    flux_p: u16 = 0,
                                    flux_i: u16 = 0,
                                    torque_p: u16 = 0,
                                    torque_i: u16 = 0
                                );
                                continue;
                            }};
                        }

                        // ADC calibration must run before R/L measurement.
                        // Without it, current readings are uncalibrated.
                        if let Err(e) = tmc_adc_calibrate(tmc, enable_pin).await {
                            defmt::error!("Autotune ADC calibration failed: {}", e);
                            autotune_abort!(4, 0, 0);
                        }

                        // Check for fault after ADC calibration
                        if fault_pending() {
                            defmt::error!("Autotune aborted: hardware fault during ADC cal");
                            autotune_abort!(4, 0, 0);
                        }

                        // Measure R with PWM-rate averaging (1000 samples per coil)
                        let r_currents = match foci_core::selftest::autotune_measure_resistance(
                            tmc,
                            enable_pin,
                            &mut Mono,
                            &TMC_CONFIG,
                        )
                        .await
                        {
                            Some(r) => r,
                            None => {
                                autotune_abort!(4, 0, 0);
                            }
                        };

                        // Check for fault after R measurement
                        if fault_pending() {
                            defmt::error!("Autotune aborted: hardware fault during R measurement");
                            autotune_abort!(4, 0, 0);
                        }

                        // Measure L with square-wave di/dt.
                        // Time the measurement with DWT cycle counter so we
                        // know the actual dt per cycle (SPI overhead dominates).
                        let dwt_start = read_tim5();
                        let l_result = match foci_core::selftest::autotune_measure_inductance(
                            tmc,
                            enable_pin,
                            &mut Mono,
                            &TMC_CONFIG,
                        )
                        .await
                        {
                            Some(l) => l,
                            None => {
                                autotune_abort!(4, 0, 0);
                            }
                        };
                        let dwt_end = read_tim5();
                        // Total wall-clock cycles for both coils, minus settle/cooldown.
                        // The measurement function includes 120ms of delays (10ms pre-toggle
                        // + 50ms cooldown per coil) that would inflate dt_us by ~12%.
                        let dwt_raw = dwt_end.wrapping_sub(dwt_start);
                        const SETTLE_CYCLES: u32 = foci_core::selftest::DIDT_SETTLE_MS * 168_000;
                        let dwt_elapsed = dwt_raw.saturating_sub(SETTLE_CYCLES);

                        foci_core::selftest::restore_safe_state(tmc, enable_pin).await;

                        // Check for fault after L measurement
                        if fault_pending() {
                            defmt::error!("Autotune aborted: hardware fault during L measurement");
                            klipper_reply!(
                                foci_autotune_result,
                                oid: u8 = oid,
                                status: u8 = 4,
                                resistance_mohm: u32 = 0,
                                inductance_uh: u32 = 0,
                                flux_p: u16 = 0,
                                flux_i: u16 = 0,
                                torque_p: u16 = 0,
                                torque_i: u16 = 0
                            );
                            continue;
                        }

                        // Build internal measurement for gain computation
                        let current_avg = ((r_currents.0 as u32 + r_currents.1 as u32) / 2) as u16;

                        // Compute average dI using wall-clock timing.
                        // L_int = V * total_time / sum_delta_I (per coil)
                        // For the gain formula we need dI per unit time:
                        //   dI_per_sec = sum_delta / total_time_s
                        // But it's simpler to pass V, sum_delta, and elapsed
                        // time separately and let the gain formula handle it.
                        //
                        // For now (Phase 1), compute approximate physical L:
                        //   L = V * dt / dI (per cycle, in internal units)
                        //   L_physical = L_int * R_physical / R_int (approximate)
                        //
                        // Use DWT elapsed to get actual dt per cycle:
                        //   total_cycles_both_coils = count_a + count_b
                        //   (includes settle time, but settle is small vs measurement)
                        let total_count = l_result.count.0 as u64 + l_result.count.1 as u64;
                        let total_sum_delta = l_result.sum_delta.0 + l_result.sum_delta.1;

                        // Average dI per cycle (using actual timing)
                        // delta_i_avg = sum_delta / count (per cycle, internal units)
                        let delta_i_avg = if total_count > 0 {
                            (total_sum_delta / total_count) as u16
                        } else {
                            0u16
                        };

                        // Actual dt per cycle in microseconds:
                        // dt_us = dwt_elapsed / CPU_MHZ / total_cycles_measured
                        // But total_cycles_measured includes 2 coils' settle time.
                        // Better: dt_us = dwt_elapsed / (168 * total_count)
                        // (each count entry is one SPI write+read cycle)
                        const CPU_MHZ: u32 = 168;
                        let dt_us = if total_count > 0 {
                            dwt_elapsed as u64 / (CPU_MHZ as u64 * total_count)
                        } else {
                            0
                        };

                        // Excitation voltage used during R/L measurement (from config)
                        let excitation = TMC_CONFIG.excitation_voltage as u32;

                        // Approximate physical R/L for user display
                        let r_mohm: u32 = if current_avg > 0 {
                            excitation * 1000 / current_avg as u32
                        } else {
                            0
                        };
                        // L_uh = V * dt_us / dI (internal units cancel in ratio)
                        // Then convert to physical: multiply by R_physical/R_int
                        // Simplified: L_uh = R_mohm * dt_us / 1000
                        //   (because V/dI ~ R_int, and R_mohm/R_int gives the
                        //    physical scaling... but actually:
                        //    L_int = V * dt / dI, L_phys = L_int * scale
                        //    Since R_int = V/I and R_phys = R_mohm/1000:
                        //    scale = R_phys/R_int = R_mohm * I / (V * 1000)
                        //    L_phys = V * dt / dI * R_mohm * I / (V * 1000)
                        //           = dt * R_mohm * I / (dI * 1000)
                        //    L_uh = dt_us * R_mohm * I / (dI * 1000)
                        let l_uh: u32 = if delta_i_avg > 0 {
                            (dt_us * r_mohm as u64 * current_avg as u64
                                / (delta_i_avg as u64 * 1000)) as u32
                        } else {
                            0
                        };

                        defmt::info!(
                            "Autotune: V={} I={} dI={} dt={}us R~{}mOhm L~{}uH (DWT={})",
                            excitation,
                            current_avg,
                            delta_i_avg,
                            dt_us,
                            r_mohm,
                            l_uh,
                            dwt_elapsed,
                        );

                        // Phase 2: compute P-only gains from internal measurements
                        let internal = foci_core::pid::InternalMeasurement {
                            voltage: excitation as u16,
                            current_avg,
                            delta_i_avg,
                            dt_us: dt_us as u32,
                        };
                        let gains =
                            foci_core::pid::compute_gains_from_internal(&internal, &TMC_CONFIG);

                        // Write to TMC4671 — abort if SPI fails
                        let (flux_reg, torque_reg) = gains.to_registers();
                        if tmc
                            .write_register(tmc4671::registers::PID_FLUX_P_FLUX_I, flux_reg)
                            .await
                            .is_err()
                            || tmc
                                .write_register(
                                    tmc4671::registers::PID_TORQUE_P_TORQUE_I,
                                    torque_reg,
                                )
                                .await
                                .is_err()
                        {
                            defmt::error!("Autotune: SPI write failed for gain registers");
                            autotune_abort!(5, r_mohm, l_uh);
                        }

                        stored_gains = Some(gains);

                        defmt::info!("Autotune gains: P={} I={}", gains.flux_p, gains.flux_i,);

                        klipper_reply!(
                            foci_autotune_result,
                            oid: u8 = oid,
                            status: u8 = 0,
                            resistance_mohm: u32 = r_mohm,
                            inductance_uh: u32 = l_uh,
                            flux_p: u16 = gains.flux_p,
                            flux_i: u16 = gains.flux_i,
                            torque_p: u16 = gains.torque_p,
                            torque_i: u16 = gains.torque_i
                        );
                    }
                    TmcRequest::SelfTest { oid } => {
                        let status = if motor_enabled {
                            defmt::warn!("foci_selftest rejected: motor enabled");
                            foci_core::selftest::SELFTEST_DONE_ABORTED
                        } else {
                            defmt::info!("foci_selftest: starting");
                            let current_limit =
                                CURRENT_LIMIT_REG.load(core::sync::atomic::Ordering::Relaxed);
                            let mut reporter = KlipperSelftestReporter { oid };
                            let result = foci_core::selftest::run_selftest(
                                tmc,
                                enable_pin,
                                &mut Mono,
                                &TMC_CONFIG,
                                current_limit,
                                &mut reporter,
                            )
                            .await;
                            defmt::info!("foci_selftest: done, status={}", result.status);
                            result.status
                        };
                        klipper_reply!(
                            foci_selftest_done,
                            oid: u8 = oid,
                            status: u8 = status
                        );
                        SELFTEST_IN_FLIGHT.store(false, core::sync::atomic::Ordering::Relaxed);
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

            // Check emergency disable (non-blocking). Triggered by EXTI
            // STATUS interrupt or enter_shutdown. Disables motor and reports
            // the fault to Klipper so the host knows the MCU is down.
            if TMC_DISABLE_SIGNAL.try_take().is_some() {
                // Stop hardware stepper immediately
                MOTOR_ARMED.store(false, core::sync::atomic::Ordering::Relaxed);
                stepper_stop();
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

            // TrSync: check timeouts and send periodic status reports.
            //
            // Runs every 1ms tick. For each active TrSync:
            // - check_timeout: auto-triggers if the expiry clock has passed
            // - report_due: sends trsync_state to the host at the configured interval
            //
            // Split into two passes to satisfy the borrow checker: first collect
            // OID-index pairs from the map, then process each TrSync using direct
            // field access (split borrows on State's distinct fields).
            {
                use klipper_mcu::trsync::TrSyncState;
                let now_clock = read_tim5();
                ctx.shared.app_state.lock(|state| {
                    // Collect OID-index pairs (max NUM_TRSYNC = 4)
                    let mut pairs: [(u8, usize); klipper_mcu::state::NUM_TRSYNC] =
                        [(0, 0); klipper_mcu::state::NUM_TRSYNC];
                    let count = state.trsync_by_oid.len();
                    for (i, (&oid, &idx)) in state.trsync_by_oid.iter().enumerate() {
                        pairs[i] = (oid, idx);
                    }

                    for &(oid, idx) in &pairs[..count] {
                        // Check timeout — if expired, stop hardware stepper
                        if state.trsync[idx].check_timeout(now_clock) {
                            stepper_stop();
                        }

                        // Send periodic status report if due
                        if state.trsync[idx].report_due(now_clock) {
                            let (can_trigger, trigger_reason) = match state.trsync[idx].state() {
                                TrSyncState::Idle => (0u8, 0u8),
                                TrSyncState::Armed => (1u8, 0u8),
                                TrSyncState::Triggered(reason) => (0u8, reason),
                            };
                            klipper_reply!(
                                trsync_state,
                                oid: u8 = oid,
                                can_trigger: u8 = can_trigger,
                                trigger_reason: u8 = trigger_reason,
                                clock: u32 = now_clock
                            );
                        }
                    }
                });
            }

            // Endstop: sample configured GPIO pins when homing is active.
            //
            // For each active endstop whose sample clock has been reached, read
            // the GPIO pin state and feed it to the endstop state machine. If
            // the endstop triggers, fire the linked TrSync and stop steppers.
            {
                use klipper_mcu::endstop::SampleResult;
                let now_clock = read_tim5();
                ctx.shared.app_state.lock(|state| {
                    // Collect OID-index pairs (max NUM_ENDSTOPS = 4)
                    let mut pairs: [(u8, usize); klipper_mcu::state::NUM_ENDSTOPS] =
                        [(0, 0); klipper_mcu::state::NUM_ENDSTOPS];
                    let count = state.endstop_by_oid.len();
                    for (i, (&oid, &idx)) in state.endstop_by_oid.iter().enumerate() {
                        pairs[i] = (oid, idx);
                    }

                    for &(oid, idx) in &pairs[..count] {
                        if !state.endstops[idx].is_homing() {
                            continue;
                        }

                        // Check if it is time for the next sample
                        let next = state.endstops[idx].next_sample_clock();
                        let elapsed = now_clock.wrapping_sub(next);
                        if elapsed > 0x8000_0000 {
                            // next_sample_clock is in the future; skip
                            continue;
                        }

                        // Read GPIO pin state
                        let pin_id =
                            ENDSTOP_PIN_IDS[idx].load(core::sync::atomic::Ordering::Relaxed);
                        let pin_state = if pin_id != 0xFF {
                            gpio_read_pin(pin_id)
                        } else {
                            false
                        };

                        match state.endstops[idx].sample(pin_state, now_clock) {
                            SampleResult::Triggered { reason, trsync_oid } => {
                                // Fire TrSync trigger and stop linked steppers
                                if let Some(&ts_idx) = state.trsync_by_oid.get(&trsync_oid) {
                                    state.trsync[ts_idx].trigger(reason);
                                    stepper_stop();
                                } else {
                                    defmt::error!(
                                        "endstop trigger: unknown trsync_oid={}",
                                        trsync_oid
                                    );
                                    if enter_shutdown(state) {
                                        klipper_shutdown!(
                                            "Endstop trigger: unknown trsync OID",
                                            read_tim5()
                                        );
                                    }
                                }
                                defmt::info!(
                                    "endstop oid={} triggered: reason={} trsync_oid={}",
                                    oid,
                                    reason,
                                    trsync_oid
                                );
                            }
                            SampleResult::Continue | SampleResult::NotHoming => {}
                        }
                    }
                });
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

    /// Run the full three-stage calibration sequence: ADC calibration,
    /// encoder alignment, and closed-loop entry.
    ///
    /// Returns `(adc_i0, adc_i1, encoder_count)` on success, or
    /// `CalibrationError` on any stage failure.
    async fn tmc_run_calibration(
        tmc: &mut Tmc4671Spi<TmcSpiDevice>,
        enable_pin: &mut Output<'static>,
        current_limit_reg: u32,
        stored_gains: Option<foci_core::pid::InnerLoopGains>,
        stored_position_gains: Option<(u16, u16, u16, u16)>,
    ) -> Result<(u16, u16, u32), CalibrationError> {
        let (adc_i0, adc_i1) = tmc_adc_calibrate(tmc, enable_pin).await?;
        let encoder_count = tmc_align_encoder(tmc, current_limit_reg).await?;
        // Use default inner-loop gains for closed-loop entry (runaway
        // detection needs known-good torque). Position/velocity gains
        // are applied directly since they don't affect runaway detection.
        tmc_enter_closedloop(tmc, None, stored_position_gains).await?;

        // Now apply autotuned gains if available
        if let Some(gains) = stored_gains {
            let (flux_reg, torque_reg) = gains.to_registers();
            tmc.write_register(tmc4671::registers::PID_FLUX_P_FLUX_I, flux_reg)
                .await
                .map_err(|_| CalibrationError::SpiError)?;
            tmc.write_register(tmc4671::registers::PID_TORQUE_P_TORQUE_I, torque_reg)
                .await
                .map_err(|_| CalibrationError::SpiError)?;
        }

        // Re-apply current limit AFTER closed-loop entry. The TMC4671
        // resets PID_TORQUE_FLUX_LIMITS during mode changes, clearing the
        // torque limit field. Without this, the motor has zero torque
        // capacity in position mode.
        tmc.write_register(
            tmc4671::registers::PID_TORQUE_FLUX_LIMITS,
            current_limit_reg,
        )
        .await
        .map_err(|_| CalibrationError::SpiError)?;

        Ok((adc_i0, adc_i1, encoder_count))
    }

    /// Perform TMC4671 hardware safety shutdown, PWM/ADC configuration, and ADC
    /// offset calibration.
    ///
    /// Steps:
    /// 1. Safety shutdown: enable_pin low, MODE=0, voltage limit=1024, zero targets
    /// 2. PWM config: polarities=0, PWM frequency and BBM from TMC_CONFIG, CHOP=0 for calibration
    /// 3. ADC config: DSADC decimation from TMC_CONFIG, ADC_I_SELECT for 2-phase routing
    /// 4. Motor type: 2-phase stepper (type 2), 50 pole pairs
    /// 5. ADC offset calibration: scale=256, enable, wait 250 ms, sample 128x, write offsets
    /// 6. Re-enable PWM chopping: PWM_SV_CHOP=7
    ///
    /// Returns `(offset_i0, offset_i1)` on success. Critical register writes are
    /// verified by read-back; ADC offsets are range-checked.
    ///
    /// This function is shared by both the dev open-loop path and the production
    /// closed-loop enable sequence.
    ///
    /// Register values verified against TMC4671-LA datasheet rev 2.08.
    async fn tmc_adc_calibrate(
        tmc: &mut Tmc4671Spi<TmcSpiDevice>,
        enable_pin: &mut Output<'static>,
    ) -> Result<(u16, u16), CalibrationError> {
        use tmc4671::registers::*;

        // 1. SAFETY FIRST: stop motor, disable outputs, set voltage limit
        // The power-on default PIDOUT_UQ_UD_LIMITS is 32767 (full voltage).
        // We MUST set a safe limit before ANYTHING else touches the motor.
        enable_pin.set_low();
        tmc.write_register(MODE_RAMP_MODE_MOTION, 0x0000_0000)
            .await
            .map_err(|_| CalibrationError::SpiError)?;
        let voltage_limit: u32 = 1024;
        tmc.write_register(PIDOUT_UQ_UD_LIMITS, voltage_limit)
            .await
            .map_err(|_| CalibrationError::SpiError)?;
        // Verify voltage limit write
        let readback = tmc
            .read_register(PIDOUT_UQ_UD_LIMITS)
            .await
            .map_err(|_| CalibrationError::SpiError)?;
        if readback != voltage_limit {
            defmt::error!(
                "PIDOUT_UQ_UD_LIMITS verify failed: wrote={}, read={}",
                voltage_limit,
                readback
            );
            return Err(CalibrationError::SpiError);
        }
        // Also zero the torque target to prevent any current drive
        let _ = tmc.write_register(PID_TORQUE_FLUX_TARGET, 0).await;
        // And zero the UQ/UD external values
        let _ = tmc.write_register(UQ_UD_EXT, 0).await;

        // 2. PWM configuration (chopping disabled for ADC calibration)
        let _ = tmc.write_register(PWM_POLARITIES, 0).await;
        let _ = tmc
            .write_register(PWM_MAXCNT, TMC_CONFIG.pwm_maxcnt as u32)
            .await;
        let _ = tmc
            .write_register(PWM_BBM_H_BBM_L, TMC_CONFIG.bbm_register())
            .await;
        // PWM_SV=0 (SVPWM off per DS 4.9.7 - stepper only), CHOP=0 for ADC cal
        tmc.write_register(PWM_SV_CHOP, 0)
            .await
            .map_err(|_| CalibrationError::SpiError)?;
        // Verify MODE=0 and PWM_SV_CHOP=0 — critical for safe ADC calibration
        let mode_rb = tmc
            .read_register(MODE_RAMP_MODE_MOTION)
            .await
            .map_err(|_| CalibrationError::SpiError)?;
        let chop_rb = tmc
            .read_register(PWM_SV_CHOP)
            .await
            .map_err(|_| CalibrationError::SpiError)?;
        if mode_rb != 0 || chop_rb != 0 {
            defmt::error!(
                "ADC cal pre-check failed: MODE={:#010x}, PWM_SV_CHOP={}",
                mode_rb,
                chop_rb
            );
            return Err(CalibrationError::SpiError);
        }

        // 3. ADC configuration
        // DSADC_MCFG: leave at power-on default (ANALOG mode, internal ADC)
        let _ = tmc
            .write_register(DSADC_MDEC_B_MDEC_A, TMC_CONFIG.dsadc_mdec_register())
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

        // 4b. Velocity meter: PWM-synchronized (selection=1).
        // Required for advanced PI mode (datasheet section 4.6.11, p.51):
        // "This option is recommended for usage with the advanced PI controller
        // structure. Otherwise, the controller structure might tend to be
        // unstable due to non-matched sampling."
        // VELOCITY_SELECTION: bits[7:0]=0 (follows PHI_E_SELECTION),
        // bits[15:8]=1 (PWM-synchronized velocity meter for advanced PI).
        // Note: phi_m_abn (9) for position/velocity was tested but doesn't
        // work with SPI-written position targets — phi_m wraps at 65536
        // (one revolution) while the emulated stepper accumulates without
        // wrapping, causing position target overflow. The reference driver
        // avoids this by using hardware step/dir input instead of SPI.
        let _ = tmc.write_register(VELOCITY_SELECTION, (1 << 8) | 0).await;

        // 5. ADC offset calibration
        // With PWM_CHOP=0, no current flows even with motor connected.
        // This matches the Python driver's _calibrate_adc() method.
        let _ = tmc.write_register(ADC_I0_SCALE_OFFSET, 256 << 16).await;
        let _ = tmc.write_register(ADC_I1_SCALE_OFFSET, 256 << 16).await;

        // Enable to power the ADC, but CHOP=0 means no PWM switching
        enable_pin.set_high();
        Mono::delay(250_u32.millis()).await;

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
            Mono::delay(1_u32.millis()).await;
        }
        // Cast via i16 to u16: negative values wrap correctly for the
        // TMC4671's unsigned offset subtraction register.
        let offset_i0 = (sum_i0 / N_SAMPLES) as i16 as u16;
        let offset_i1 = (sum_i1 / N_SAMPLES) as i16 as u16;

        // Validate ADC offsets are in expected range (centered around 33000)
        if !(30000..=36000).contains(&offset_i0) || !(30000..=36000).contains(&offset_i1) {
            defmt::error!(
                "ADC offset out of range: I0={}, I1={} (expected 30000..=36000)",
                offset_i0,
                offset_i1
            );
            return Err(CalibrationError::AdcFault);
        }

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

        Ok((offset_i0, offset_i1))
    }

    /// Configure TMC4671 for open-loop stepper operation with current regulation.
    ///
    /// Calls `tmc_adc_calibrate` for hardware init and ADC offset calibration,
    /// then configures torque mode with PHI_E_EXT for open-loop angle control.
    ///
    /// Register values verified against TMC4671-LA datasheet rev 2.08.
    #[cfg(feature = "dev")]
    async fn tmc_enter_openloop(
        tmc: &mut Tmc4671Spi<TmcSpiDevice>,
        enable_pin: &mut Output<'static>,
    ) {
        use tmc4671::registers::*;

        if let Err(e) = tmc_adc_calibrate(tmc, enable_pin).await {
            defmt::error!("ADC calibration failed in open-loop init: {}", e);
            return;
        }

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

    /// Align the ABN encoder to the D-axis using open-loop voltage mode,
    /// waiting for the rotor to settle, then zeroing the encoder count.
    ///
    /// Uses UQ_UD_EXT mode (mode 8) to apply a direct D-axis voltage at
    /// PHI_E=0. This matches the TMC-API reference implementation and
    /// provides sufficient force to overcome motor detent torque regardless
    /// of impedance. PID-regulated torque mode (mode 1) was previously used
    /// but was insufficient for motors with strong detent torque.
    ///
    /// Steps:
    /// 1. Set PHI_E_SELECTION=1 (external) and PHI_E_EXT=0
    /// 2. Capture ABN_DECODER_COUNT before alignment
    /// 3. Apply D-axis voltage via UQ_UD_EXT (UD=excitation_voltage, UQ=0)
    /// 4. Enter open-loop voltage mode (MODE_MOTION=8)
    /// 5. Hold for settling (rotor aligns to nearest D-axis pole)
    /// 6. Validate encoder (movement detected or stable non-zero count)
    /// 7. Zero ABN_DECODER_COUNT and PHI_E_PHI_M_OFFSET
    /// 8. Switch PHI_E to ABN encoder (while still driving)
    /// 9. Stop drive (UQ_UD_EXT=0, MODE=0)
    ///
    /// **Post-condition:** PHI_E_SELECTION=3 (ABN encoder), motor stopped.
    /// The angle reference is locked to ABN before the drive stops, so the
    /// rotor cannot drift to a detent position and corrupt the alignment.
    /// `tmc_enter_closedloop` must NOT overwrite PHI_E_SELECTION.
    ///
    /// Returns the encoder count before zeroing on success.
    async fn tmc_align_encoder(
        tmc: &mut Tmc4671Spi<TmcSpiDevice>,
        current_limit_reg: u32,
    ) -> Result<u32, CalibrationError> {
        use tmc4671::registers::*;

        // Reject alignment if current limit is not configured.
        // With current_limit_reg=0, flux_target=0 and the rotor never aligns,
        // but the encoder stability check passes (both reads are 0), producing
        // a silently miscalibrated encoder.
        if current_limit_reg == 0 {
            defmt::error!("encoder align: current limit not set");
            return Err(CalibrationError::SpiError);
        }

        // 1. PHI_E source: external, angle = 0 (D-axis).
        // The D-axis is at phi_e=0 in the 16-bit electrical angle space.
        // With 50 pole pairs, the rotor travels at most half a pole pitch
        // (0.2mm) to reach the nearest D-axis alignment point.
        let _ = tmc.write_register(PHI_E_SELECTION, 1).await;
        let _ = tmc.write_register(PHI_E_EXT, 0).await;

        // 2. Ensure encoder has non-zero count before alignment.
        // On cold boot, ABN_DECODER_COUNT is 0. If the rotor is already
        // at a D-axis point, the alignment won't move it and the encoder
        // check fails. A brief open-loop nudge guarantees encoder counts
        // are accumulated, proving the encoder is connected.
        let count_pre_nudge = tmc
            .read_register(ABN_DECODER_COUNT)
            .await
            .map_err(|_| CalibrationError::SpiError)?;
        // NOTE: encoder nudge disabled — it moved the motor to a random
        // D-axis position that the alignment drive couldn't correct,
        // causing ~50% alignment failure rate. Encoder connectivity is
        // verified by checking for count change during the alignment
        // drive itself (the alignment moves the motor, proving encoder
        // is connected). The count_pre_nudge==0 case is handled by the
        // existing movement check below (moved || count != 0).
        if count_pre_nudge == 0 {
            // On cold boot, ABN_DECODER_COUNT is 0. If the rotor is already
            // at the D-axis, the alignment won't move it and the movement
            // check would reject count=0 as "no movement". Seed the counter
            // to 1 so the post-alignment check can detect that the alignment
            // drive changed it (it will be zeroed after alignment anyway).
            let _ = tmc.write_register(ABN_DECODER_COUNT, 1).await;
            defmt::info!("encoder count=0 at alignment start, seeded to 1");
        }

        let count_before = tmc
            .read_register(ABN_DECODER_COUNT)
            .await
            .map_err(|_| CalibrationError::SpiError)?;

        // 3. Raise voltage limit for alignment. ADC calibration sets
        // PIDOUT_UQ_UD_LIMITS to 1024 as a safety measure, but that clips
        // alignment voltages above 1024. Raise to allow the alignment
        // voltage through. Closedloop entry sets the operational limit.
        let _ = tmc.write_register(PIDOUT_UQ_UD_LIMITS, 2000).await;

        // 4. D-axis voltage for alignment: UD = 1000, UQ = 0.
        // Uses open-loop voltage mode (UQ_UD_EXT, mode 8) matching the
        // TMC-API reference implementation. The voltage must be sufficient
        // to overcome motor detent torque but not so high as to overdrive
        // the magnetic circuit. Original FOCI implementation used 1000.
        // UQ_UD_EXT layout: bits[31:16] = UQ, bits[15:0] = UD
        let _ = tmc.write_register(UQ_UD_EXT, 1000u32).await;

        // 5. Enter open-loop voltage mode (MODE_PID_TYPE=1, MODE_MOTION=8)
        let _ = tmc.write_register(MODE_RAMP_MODE_MOTION, 0x8000_0008).await;

        // 6. Hold for settling (rotor aligns to nearest D-axis pole)
        Mono::delay(500_u32.millis()).await;

        // 7. Validate encoder: stable count, detect movement or prior counts.
        let count1 = tmc
            .read_register(ABN_DECODER_COUNT)
            .await
            .map_err(|_| CalibrationError::SpiError)?;
        Mono::delay(10_u32.millis()).await;
        let count2 = tmc
            .read_register(ABN_DECODER_COUNT)
            .await
            .map_err(|_| CalibrationError::SpiError)?;

        // Check encoder presence: compare against pre-alignment count.
        // Verify the alignment produced encoder movement OR that the count
        // is stable and non-zero (prior motion accumulated).
        let moved = count1 != count_before || count2 != count_before;
        // Allow +/-2 encoder counts of jitter during settling.
        let stable = (count1 as i32 - count2 as i32).unsigned_abs() <= 2;
        if !moved && count1 == 0 {
            defmt::error!(
                "encoder align: no movement (before={}, after={},{})",
                count_before,
                count1,
                count2
            );
            let _ = tmc.write_register(UQ_UD_EXT, 0).await;
            let _ = tmc.write_register(MODE_RAMP_MODE_MOTION, 0).await;
            return Err(CalibrationError::EncoderFault);
        }
        if !stable {
            defmt::error!(
                "encoder align: encoder unstable: count1={}, count2={}",
                count1,
                count2
            );
            let _ = tmc.write_register(UQ_UD_EXT, 0).await;
            let _ = tmc.write_register(MODE_RAMP_MODE_MOTION, 0).await;
            return Err(CalibrationError::EncoderFault);
        }

        // 8. Zero encoder count and offset at the D-axis position.
        // CRITICAL: the motor is still held in voltage mode while we do this.
        let old_count = count1;
        let _ = tmc.write_register(ABN_DECODER_COUNT, 0).await;
        let _ = tmc.write_register(ABN_DECODER_PHI_E_PHI_M_OFFSET, 0).await;

        // 9. Switch PHI_E to ABN encoder WHILE STILL DRIVING.
        // The motor is actively held at the D-axis position, so the ABN-
        // derived PHI_E (count=0, offset=0 -> PHI_E=0) matches the external
        // PHI_E=0 we were using. This avoids an unpowered gap between
        // alignment and closed-loop entry where the rotor could snap to a
        // detent position and corrupt the angle reference.
        let _ = tmc.write_register(PHI_E_SELECTION, 3).await;

        // 10. Stop drive: zero voltage first, then stop mode.
        // PHI_E is now locked to ABN encoder — the angle reference survives
        // the brief coast until tmc_enter_closedloop engages position mode.
        let _ = tmc.write_register(UQ_UD_EXT, 0).await;
        let _ = tmc.write_register(MODE_RAMP_MODE_MOTION, 0x0000_0000).await;

        defmt::info!("encoder aligned: old_count={}", old_count);
        Ok(old_count)
    }

    /// Configure TMC4671 for closed-loop position mode with ABN encoder feedback.
    ///
    /// **Pre-condition:** `tmc_align_encoder` has already set PHI_E_SELECTION=3
    /// (ABN encoder) with the encoder zeroed at the D-axis. This function must
    /// NOT overwrite PHI_E_SELECTION.
    ///
    /// Steps:
    /// 1. Set flux/torque PID gains
    /// 2. Set velocity PID gains (host-configured or defaults)
    /// 3. Set position PID gains (host-configured or defaults)
    /// 4. Set PID_VELOCITY_LIMIT (anti-windup)
    /// 5. Raise PIDOUT_UQ_UD_LIMITS for operation
    /// 6. Zero PID_POSITION_ACTUAL (also zeros target)
    /// 7. Zero PID_POSITION_ERROR_SUM (integrator)
    /// 8-11. Write biquad filters if configured
    /// 12. Enter position mode: MODE_RAMP_MODE_MOTION = 0x8000_0003
    /// 13. Validate: position error is not diverging
    ///
    /// Assumes tmc_adc_calibrate and tmc_align_encoder have already been called.
    async fn tmc_enter_closedloop(
        tmc: &mut Tmc4671Spi<TmcSpiDevice>,
        stored_gains: Option<foci_core::pid::InnerLoopGains>,
        stored_position_gains: Option<(u16, u16, u16, u16)>,
    ) -> Result<(), CalibrationError> {
        use tmc4671::registers::*;

        // 1. Flux and torque PID: use stored gains or conservative defaults
        let gains = stored_gains.unwrap_or(foci_core::pid::DEFAULT_INNER_GAINS);
        let (flux_reg, torque_reg) = gains.to_registers();
        let _ = tmc.write_register(PID_FLUX_P_FLUX_I, flux_reg).await;
        let _ = tmc.write_register(PID_TORQUE_P_TORQUE_I, torque_reg).await;

        // 2. Velocity PID: use stored gains or defaults (P=4.5, I=0)
        let (vel_p, vel_i) = stored_position_gains
            .map(|(_, _, vp, vi)| (vp, vi))
            .unwrap_or((1152, 0));
        let velocity_pi: u32 = (vel_p as u32) << 16 | vel_i as u32;
        let _ = tmc
            .write_register(PID_VELOCITY_P_VELOCITY_I, velocity_pi)
            .await;

        // 3. Position PID: use stored gains or defaults (P=2.5, I=0)
        let (pos_p, pos_i) = stored_position_gains
            .map(|(pp, pi, _, _)| (pp, pi))
            .unwrap_or((640, 0));
        let position_pi: u32 = (pos_p as u32) << 16 | pos_i as u32;
        let _ = tmc
            .write_register(PID_POSITION_P_POSITION_I, position_pi)
            .await;

        // 4. Velocity limit: host-configured or unconstrained default.
        // Caps position PID output velocity to prevent integrator windup runaway.
        let vel_limit = VELOCITY_LIMIT_CFG.load(core::sync::atomic::Ordering::Relaxed);
        let _ = tmc.write_register(PID_VELOCITY_LIMIT, vel_limit).await;

        // 5. Raise voltage limit for actual operation. ADC cal sets it to
        // 1024 as a safety measure, but position PID needs headroom to drive
        // motion. 16000 tested working in Phase 1 (up to 200+ mm/s).
        let _ = tmc.write_register(PIDOUT_UQ_UD_LIMITS, 16000).await;

        // NOTE: PHI_E_SELECTION is NOT written here. tmc_align_encoder
        // already set it to 3 (ABN encoder) while the motor was held in
        // voltage mode, locking the angle reference before the drive stopped.

        // 6. Zero position: writing PID_POSITION_ACTUAL also writes
        // PID_POSITION_TARGET to the same value (per datasheet 0x6B),
        // avoiding an unwanted position error on mode entry.
        let _ = tmc.write_register(PID_POSITION_ACTUAL, 0).await;

        // Zero position integrator to prevent stale error sum from prior session.
        // Must come after PID_POSITION_ACTUAL=0 so the integrator doesn't
        // immediately re-accumulate from a nonzero position error.
        let _ = tmc
            .write_sub_register(PID_ERROR_PID_POSITION_ERROR_SUM, 0)
            .await;

        // 7. Explicit POSITION_SELECTION=0 (phi_e). Don't rely on power-on
        // default — survives fault/reset/re-calibration because closedloop
        // entry always runs.
        let _ = tmc.write_register(POSITION_SELECTION, 0).await;

        // 8. Write STEP_WIDTH for hardware step/dir input. Each STEP pulse
        // increments the TMC4671's internal position target by this value.
        let step_width = STEP_WIDTH_VALUE.load(core::sync::atomic::Ordering::Relaxed);
        if step_width > 0 {
            let _ = tmc.write_register(STEP_WIDTH, step_width as u32).await;
            defmt::info!("STEP_WIDTH(0x78) = {} written", step_width);
        } else {
            defmt::warn!("STEP_WIDTH = 0, skipping register write!");
        }

        // 8. Write velocity biquad filter if configured, disable if not.
        // If the cutoff is nonzero but out of the valid 10-1000 Hz range,
        // butterworth_lowpass returns None — explicitly disable in that case.
        let filter_hz = VELOCITY_FILTER_HZ.load(core::sync::atomic::Ordering::Relaxed);
        if filter_hz > 0 {
            if let Some(coeffs) =
                foci_core::biquad::butterworth_lowpass(filter_hz, TMC_CONFIG.pwm_maxcnt)
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
                defmt::info!("Velocity biquad: {}Hz enabled", filter_hz);
            }
        }

        // 9. Write torque biquad filter if configured
        let filter_hz = TORQUE_FILTER_HZ.load(core::sync::atomic::Ordering::Relaxed);
        if filter_hz > 0 {
            if let Some(coeffs) =
                foci_core::biquad::butterworth_lowpass(filter_hz, TMC_CONFIG.pwm_maxcnt)
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
                defmt::info!("Torque biquad: {}Hz enabled", filter_hz);
            }
        }

        // 10. Write position target biquad filter if configured
        let filter_hz = POSITION_FILTER_HZ.load(core::sync::atomic::Ordering::Relaxed);
        if filter_hz > 0 {
            if let Some(coeffs) =
                foci_core::biquad::butterworth_lowpass(filter_hz, TMC_CONFIG.pwm_maxcnt)
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
                defmt::info!("Position biquad: {}Hz enabled", filter_hz);
            }
        }

        // 11. Write flux biquad filter if configured
        let filter_hz = FLUX_FILTER_HZ.load(core::sync::atomic::Ordering::Relaxed);
        if filter_hz > 0 {
            if let Some(coeffs) =
                foci_core::biquad::butterworth_lowpass(filter_hz, TMC_CONFIG.pwm_maxcnt)
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
                defmt::info!("Flux biquad: {}Hz enabled", filter_hz);
            }
        }

        // NOTE: PID_VELOCITY_OFFSET is NOT cleared here — writing to 0x67
        // during closedloop entry caused intermittent encoder alignment failures.
        // The register is cleared during disable and restore_safe_state instead.

        // 12. Enter position mode: MODE_PID_TYPE=1 (bit 31), MODE_MOTION=3 (position)
        let _ = tmc.write_register(MODE_RAMP_MODE_MOTION, 0x8000_0003).await;

        // 13. Runaway detection (20ms). If position error exceeds 40 steps
        // and is still growing, the feedback sign is wrong — fail fast.
        // Encoder polarity errors are also caught in Stage 2 torque
        // tracking, but this is a secondary safety net for position mode.
        // Settling is NOT checked: P-only position control has steady-state
        // offset and normal PID oscillation would cause false positives.
        const RUNAWAY_THRESHOLD: u32 = 32760; // ~40 steps

        Mono::delay(10.millis()).await;
        let pos1 = tmc
            .read_register(PID_POSITION_ACTUAL)
            .await
            .map_err(|_| CalibrationError::SpiError)? as i32;
        Mono::delay(10.millis()).await;
        let pos2 = tmc
            .read_register(PID_POSITION_ACTUAL)
            .await
            .map_err(|_| CalibrationError::SpiError)? as i32;
        if pos2.unsigned_abs() > RUNAWAY_THRESHOLD && pos2.unsigned_abs() > pos1.unsigned_abs() {
            defmt::error!(
                "closed-loop: runaway detected: pos1={}, pos2={}",
                pos1,
                pos2
            );
            let _ = tmc.write_register(MODE_RAMP_MODE_MOTION, 0).await;
            return Err(CalibrationError::PidFault);
        }

        defmt::info!("TMC4671 closed-loop position mode active");
        Ok(())
    }

    /// Stop hardware stepper: disable timer, set need_reset, drain queue.
    ///
    /// Safe to call from P1 or P2. P3 is quiesced by disabling TIM5 CC1.
    fn stepper_stop() {
        use core::sync::atomic::Ordering::Relaxed;
        let tim5 = embassy_stm32::pac::TIM5;
        // Disable CC1 interrupt — quiesce P3
        tim5.dier().modify(|w| w.set_ccie(0, false));
        // Clear any pending CC1 flag
        tim5.sr().modify(|w| w.set_ccif(0, false));
        TIMER_ACTIVE.store(false, Relaxed);
        STEPPER_NEED_RESET.store(true, Relaxed);
        // Drain queue (safe: P3 is disabled by the CC1IE clear above)
        // SAFETY: P3 (timer ISR) is quiesced — we are the sole accessor.
        unsafe { while (*core::ptr::addr_of_mut!(STEP_QUEUE)).dequeue().is_some() {} }
    }

    /// Arm the TIM5 CC1 step timer for a compare at `clock`.
    ///
    /// Ensures NVIC is unmasked, sets compare value, enables CC1 interrupt.
    fn arm_step_timer(clock: u32) {
        use core::sync::atomic::Ordering::Relaxed;
        let tim5 = embassy_stm32::pac::TIM5;
        // Ensure NVIC is unmasked (idempotent after first call)
        unsafe { cortex_m::peripheral::NVIC::unmask(embassy_stm32::pac::Interrupt::TIM5) };
        tim5.ccr(0).write_value(clock);
        tim5.sr().modify(|w| w.set_ccif(0, false)); // Clear pending CC1
        tim5.dier().modify(|w| w.set_ccie(0, true)); // Enable CC1 interrupt
        TIMER_ACTIVE.store(true, Relaxed);
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
                        if dir_changed {
                            // Set DIR GPIO (PA0) before first step pulse
                            let dir_val = if stepper.dir_state() {
                                embassy_stm32::pac::gpio::vals::Odr::HIGH
                            } else {
                                embassy_stm32::pac::gpio::vals::Odr::LOW
                            };
                            gpio_a.odr().modify(|w| w.set_odr(0, dir_val));
                        }
                    }
                    None => {
                        // Queue empty: disarm timer, go idle
                        tim5.dier().modify(|w| w.set_ccie(0, false));
                        TIMER_ACTIVE.store(false, Relaxed);
                        return;
                    }
                }

                // Check deadline for the first step of this segment
                let now = tim5.cnt().read();
                match stepper.check_deadline(stepper.next_step_clock(), now, CLOCK_FREQ) {
                    Ok(compare) => {
                        tim5.ccr(0).write_value(compare);
                        if compare != now.wrapping_add(2) {
                            // Step is in the future — schedule and return
                            return;
                        }
                        // Step is now or immediately — fall through to execute
                    }
                    Err(()) => {
                        // >1ms past: shutdown
                        tim5.dier().modify(|w| w.set_ccie(0, false));
                        TIMER_ACTIVE.store(false, Relaxed);
                        // Signal shutdown via TMC_DISABLE_SIGNAL
                        TMC_DISABLE_SIGNAL.signal(());
                        defmt::error!("Stepper too far in past");
                        return;
                    }
                }
            } else {
                // Active segment: execute one step
                // Toggle STEP GPIO (PB11) via BSRR (atomic set/reset)
                {
                    use embassy_stm32::pac::gpio::vals::Idr;
                    let is_high = gpio_b.idr().read().idr(11) == Idr::HIGH;
                    gpio_b.bsrr().write(|w| {
                        if is_high {
                            w.set_br(11, true); // Reset (low)
                        } else {
                            w.set_bs(11, true); // Set (high)
                        }
                    });
                }

                // Increment position
                let dir = if stepper.dir_state() { 1i32 } else { -1i32 };
                STEPPER_POSITION.fetch_add(dir, Relaxed);

                // Advance scheduling
                match stepper.step_event() {
                    StepEventResult::ScheduleNext { clock } => {
                        let now = tim5.cnt().read();
                        match stepper.check_deadline(clock, now, CLOCK_FREQ) {
                            Ok(compare) => {
                                tim5.ccr(0).write_value(compare);
                                if compare != now.wrapping_add(2) {
                                    return; // Future step: wait for next interrupt
                                }
                                // Immediate: loop to execute next step
                                continue;
                            }
                            Err(()) => {
                                tim5.dier().modify(|w| w.set_ccie(0, false));
                                TIMER_ACTIVE.store(false, Relaxed);
                                TMC_DISABLE_SIGNAL.signal(());
                                defmt::error!("Stepper too far in past");
                                return;
                            }
                        }
                    }
                    StepEventResult::NeedMove => {
                        // Segment exhausted: loop to dequeue next
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
            }
            // Final poll to flush any remaining TX
            klipper_usb::poll();
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
