//! TMC4671-subsystem state (register shadows, calibration flags, fault
//! latches). Task 8b populates this from `mod app` in main.rs.

use core::cell::RefCell;

use critical_section::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use foci_firmware::tmc::TmcRequest;
use rtic_sync::channel::Sender;

/// Channel depth for TMC control requests.
pub(crate) const TMC_CHANNEL_SIZE: usize = 8;

/// Emergency motor disable signal. Set by enter_shutdown(), consumed
/// by tmc_control. Uses a Signal so tmc_control wakes immediately
/// even if blocked on an empty channel.
pub(crate) static TMC_DISABLE_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Current limit for PID_TORQUE_FLUX_LIMITS (0x5E).
/// Datasheet: single u16 field at bits[15:0]; upper 16 bits ignored by hardware.
/// Set by tmc_set_current, re-applied during calibration to ensure
/// the limit is correct regardless of TMC4671 register state.
pub(crate) static CURRENT_LIMIT_REG: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(0);

/// Velocity biquad filter cutoff frequency in Hz (set by tmc_set_velocity_filter).
/// 0 = disabled, 10..=1000 = enabled. Applied during closed-loop entry.
pub(crate) static VELOCITY_FILTER_HZ: core::sync::atomic::AtomicU16 =
    core::sync::atomic::AtomicU16::new(0);

/// Torque biquad filter cutoff frequency in Hz (set by tmc_set_torque_filter).
/// 0 = disabled, 10..=1000 = enabled. Applied during closed-loop entry.
pub(crate) static TORQUE_FILTER_HZ: core::sync::atomic::AtomicU16 =
    core::sync::atomic::AtomicU16::new(0);

/// Position target biquad filter cutoff frequency in Hz (set by tmc_set_position_filter).
/// 0 = disabled, 10..=1000 = enabled. Applied during closed-loop entry.
pub(crate) static POSITION_FILTER_HZ: core::sync::atomic::AtomicU16 =
    core::sync::atomic::AtomicU16::new(0);

/// Flux biquad filter cutoff frequency in Hz (set by tmc_set_flux_filter).
/// 0 = disabled, 10..=1000 = enabled. Applied during closed-loop entry.
pub(crate) static FLUX_FILTER_HZ: core::sync::atomic::AtomicU16 =
    core::sync::atomic::AtomicU16::new(0);

/// Velocity feedforward enable flag (set by tmc_set_velocity_feedforward).
/// When true, the 1ms tick loop computes velocity from the step
/// interval and writes PID_VELOCITY_OFFSET.
pub(crate) static VELOCITY_FEEDFORWARD: core::sync::atomic::AtomicBool =
    core::sync::atomic::AtomicBool::new(false);

/// Signal the tick loop to zero feedforward for one tick after a
/// direction change. Avoids a sign-flip transient in
/// PID_VELOCITY_OFFSET. Set by handle_tmc_request when encoder
/// direction changes, consumed by tmc_control on next iteration.
pub(crate) static FEEDFORWARD_RESET: core::sync::atomic::AtomicBool =
    core::sync::atomic::AtomicBool::new(false);

/// Signal the tick loop to clear TMC4671 PID error sums after motion.
/// The step ISR cannot use SPI, so it sets this flag when the hardware
/// step queue drains and P1 performs the register writes on the next tick.
pub(crate) static PID_ERROR_SUMS_CLEAR: core::sync::atomic::AtomicBool =
    core::sync::atomic::AtomicBool::new(false);

/// Configurable PID velocity limit (caps position PID output).
/// Prevents integrator-driven velocity runaway when position I > 0.
/// Override via tmc_set_velocity_limit command from host config.
/// Written to PID_VELOCITY_LIMIT (0x60) during closedloop entry.
pub(crate) static VELOCITY_LIMIT_CFG: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(500_000);

/// Whether the encoder direction is reversed (set by tmc_set_encoder_dir).
/// When true, position targets are negated before writing to the TMC4671.
pub(crate) static ENCODER_REVERSED: core::sync::atomic::AtomicBool =
    core::sync::atomic::AtomicBool::new(false);

/// Guard for single-in-flight commissioning. Set at command ingress (P2),
/// cleared after commissioning completes or fails (P1).
pub(crate) static COMMISSION_IN_FLIGHT: core::sync::atomic::AtomicBool =
    core::sync::atomic::AtomicBool::new(false);

/// Signal tmc_control to drain all pending requests. Set by config_reset
/// (P2), cleared by tmc_control (P1) after draining.
pub(crate) static TMC_QUEUE_FLUSH: core::sync::atomic::AtomicBool =
    core::sync::atomic::AtomicBool::new(false);

/// Set by P1 after calibration, cleared on disable/fault.
/// Gates timer arming and ISR execution.
pub(crate) static MOTOR_ARMED: core::sync::atomic::AtomicBool =
    core::sync::atomic::AtomicBool::new(false);

/// STEP_WIDTH value computed by tmc_set_encoder_hw, written to TMC4671
/// register 0x78 during closedloop entry.
pub(crate) static STEP_WIDTH_VALUE: core::sync::atomic::AtomicI32 =
    core::sync::atomic::AtomicI32::new(0);

/// Position P gain for cross-priority access (Q8.8).
pub(crate) static POSITION_P_VALUE: core::sync::atomic::AtomicU16 =
    core::sync::atomic::AtomicU16::new(640);

/// Position I gain for cross-priority access (Q8.8).
pub(crate) static POSITION_I_VALUE: core::sync::atomic::AtomicU16 =
    core::sync::atomic::AtomicU16::new(0);

/// Velocity P gain for cross-priority access (Q8.8).
/// Default 1152 = 4.5 in Q8.8 (conservative).
pub(crate) static VELOCITY_P_VALUE: core::sync::atomic::AtomicU16 =
    core::sync::atomic::AtomicU16::new(1152);

/// Velocity I gain for cross-priority access (Q8.8).
pub(crate) static VELOCITY_I_VALUE: core::sync::atomic::AtomicU16 =
    core::sync::atomic::AtomicU16::new(0);

/// Shared sender handle for the TMC control channel. Populated in RTIC
/// `init`, used by P2 command handlers to enqueue [`TmcRequest`]s.
pub(crate) static TMC_REQUEST_SENDER: Mutex<
    RefCell<Option<Sender<'static, TmcRequest, TMC_CHANNEL_SIZE>>>,
> = Mutex::new(RefCell::new(None));
