//! Commissioning state machine and hardware backend.
//!
//! The [`CommissioningEngine`] is a cooperative state machine invoked
//! every 1ms by the `tmc_control` task. Each `step()` call advances
//! the current phase by one chunk of work (max 1ms runtime).
//!
//! Phases 1-4 (diagnostics) are implemented here. Phases 5-7
//! (electrical ID, current tune, validation) are filled by the
//! electrical-ID and current-tune plans.

use foci_core::autotune::{
    CommissionError, CommissionProfile, CommissionStatus, CurrentGains, CurrentValidation,
    ElectricalModel, MechanicalModel, OperatingMode, PhaseId, TransientBuffer, ValidationResult,
};
use foci_core::config::TmcConfig;
use tmc4671::registers::Register;

/// Hardware abstraction for commissioning measurements.
///
/// The engine operates through this trait rather than raw SPI.
/// Each board crate provides a concrete implementation wrapping
/// its SPI device, GPIO pins, and timer.
///
/// SPI methods are async to match the DMA-based embassy-stm32
/// driver. The engine's `step()` is also async, called from the
/// async `tmc_control` task — consistent with how `selftest.rs`
/// measurement functions work.
pub trait CommissioningBackend {
    /// SPI/hardware error type.
    type Error;

    /// Read a TMC4671 register.
    async fn read_register(&mut self, reg: Register) -> Result<u32, Self::Error>;

    /// Write a TMC4671 register.
    async fn write_register(&mut self, reg: Register, val: u32) -> Result<(), Self::Error>;

    /// Set the TMC4671 enable pin state.
    fn set_enable_pin(&mut self, state: bool);

    /// Read the current ABN encoder count.
    async fn read_encoder_count(&mut self) -> u32;

    /// Microseconds elapsed since an arbitrary reference point.
    ///
    /// Used for deadline-driven sub-state timing within `step()`.
    /// Must be monotonic within a commissioning session.
    fn elapsed_us(&self) -> u32;

    /// Whether a shutdown has been requested (M112 / e-stop).
    ///
    /// Checked at the top of every `step()` call.
    fn is_shutdown_requested(&self) -> bool;

    /// Read the configured current limit register value.
    ///
    /// Returns `PID_TORQUE_FLUX_LIMITS` as set by `tmc_set_current`.
    /// Zero means no current limit configured.
    fn current_limit(&self) -> u32;

    /// PWM_MAXCNT register value for the board.
    fn pwm_maxcnt(&self) -> u16;

    /// PWM_BBM_H_BBM_L register value: `(bbm_h << 8) | bbm_l`.
    fn bbm_register(&self) -> u32;

    /// DSADC_MDEC_B_MDEC_A register value (same MDEC for both groups).
    fn dsadc_mdec_register(&self) -> u32;

    /// Test excitation voltage magnitude (PIDOUT units, bits [15:0]).
    fn excitation_voltage(&self) -> u16;

    /// Board-specific TMC4671 configuration.
    ///
    /// Needed by SIMC gain synthesis for PWM frequency.
    fn tmc_config(&self) -> &TmcConfig;

    /// TMC4671 STEP_WIDTH value (phi_e increment per step pulse).
    ///
    /// Used by position-loop tuning as the position step size.
    /// Returns 0 if the encoder has not been configured.
    fn step_width(&self) -> u32;
}

/// Result of a `step()` call.
#[derive(Debug, defmt::Format)]
pub enum StepResult {
    /// Engine is still working. Call `step()` again next tick.
    Continue,
    /// A phase transition occurred. Report to host.
    PhaseChanged(PhaseId),
    /// Inner commissioning completed successfully.
    InnerDone(InnerResult),
    /// Inner commissioning failed.
    InnerFailed(CommissionError),
    /// Outer commissioning completed successfully.
    OuterDone(OuterResult),
    /// Outer commissioning failed.
    OuterFailed(CommissionError),
}

/// Result of inner (electrical) commissioning.
#[derive(Debug, Clone, defmt::Format)]
pub struct InnerResult {
    /// Overall status.
    pub status: CommissionStatus,
    /// Resistance in milliohms (for host display/persistence).
    pub r_mohm: u32,
    /// Inductance in microhenries (for host display/persistence).
    pub l_uh: u32,
    /// Computed flux P gain (Q8.8).
    pub flux_p: u16,
    /// Computed flux I gain (Q8.8).
    pub flux_i: u16,
    /// Computed torque P gain (Q8.8).
    pub torque_p: u16,
    /// Computed torque I gain (Q8.8).
    pub torque_i: u16,
}

/// Result of outer (mechanical) commissioning.
#[derive(Debug, Clone, defmt::Format)]
pub struct OuterResult {
    /// Overall status.
    pub status: CommissionStatus,
    /// Velocity P gain (Q8.8).
    pub velocity_p: u16,
    /// Velocity I gain (Q8.8).
    pub velocity_i: u16,
    /// Velocity limit.
    pub velocity_limit: u32,
    /// Position P gain (Q8.8).
    pub position_p: u16,
    /// Position I gain (Q8.8).
    pub position_i: u16,
    /// Velocity filter cutoff Hz (0 = disabled).
    pub velocity_filter_hz: u16,
    /// Position filter cutoff Hz (0 = disabled).
    pub position_filter_hz: u16,
    /// Flux filter cutoff Hz (0 = disabled).
    pub flux_filter_hz: u16,
    /// Torque filter cutoff Hz (0 = disabled).
    pub torque_filter_hz: u16,
}

// ---------------------------------------------------------------------------
// Diagnostic sub-state constants
// ---------------------------------------------------------------------------

/// Number of ADC samples for offset calibration.
const ADC_CAL_SAMPLES: u16 = 128;

/// Settling time after enabling ADC outputs (microseconds).
const ADC_CAL_SETTLE_US: u32 = 250_000;

/// Valid ADC offset range (centered around ~33000).
const ADC_OFFSET_MIN: u16 = 30_000;
const ADC_OFFSET_MAX: u16 = 36_000;

/// Minimum ADC current to consider a coil present (raw ADC units).
const MOTOR_CURRENT_THRESHOLD: u16 = 200;

/// Settling time after applying coil test voltage (microseconds).
const COIL_SETTLE_US: u32 = 100_000;

/// Settling time after disabling coil test (microseconds).
const COIL_COOLDOWN_US: u32 = 50_000;

/// Number of PHI_E steps for encoder direction sweep.
const ENCODER_DIR_STEPS: u16 = 50;

/// Per-step dwell time for encoder direction sweep (microseconds).
const ENCODER_DIR_STEP_US: u32 = 10_000;

// ---------------------------------------------------------------------------
// Electrical identification constants (Phase 5)
// ---------------------------------------------------------------------------

/// Number of steady-state current samples for resistance measurement.
const RES_SAMPLE_COUNT: u16 = 1000;

/// Sampling interval for resistance and transient captures (microseconds).
/// Matches PWM rate (~160us at 25 kHz) for Nyquist-rate current sampling.
const ELEC_SAMPLE_PERIOD_US: u32 = 160;

/// Settling time after applying excitation voltage (microseconds).
const RES_SETTLE_US: u32 = 100_000;

/// Cooldown time after disabling a per-coil measurement (microseconds).
const ELEC_COOLDOWN_US: u32 = 50_000;

/// Brief settle before starting di/dt toggling (microseconds).
const DIDT_PRE_SETTLE_US: u32 = 10_000;

/// Maximum di/dt toggle cycles per step() call.
/// Each cycle is ~2 SPI transfers (~22us at 8 MHz SPI).
/// Budget: ~30 cycles/step ≈ 660us, leaving margin within 1ms tick.
const DIDT_CYCLES_PER_STEP: u32 = 30;

/// Minimum average |delta_I| per di/dt cycle to consider inductance valid.
/// Below this, the current change is too small to extract L reliably.
const DIDT_MIN_DELTA: u32 = 5;

/// Number of transient samples for delay identification.
/// Matches TransientBuffer capacity.
const DELAY_SAMPLE_COUNT: u16 = 256;

/// Minimum detectable current for delay onset (ADC units above baseline).
/// Must be above noise floor but sensitive enough to detect delay.
const DELAY_ONSET_THRESHOLD: u16 = 20;

/// Fraction of steady-state current for 63% rise detection (fixed-point).
/// 0.632 * 1024 ≈ 647 (Q10 format for integer math).
const RISE_63_PCT_Q10: u32 = 647;

// ---------------------------------------------------------------------------
// Current validation constants (Phase 7)
// ---------------------------------------------------------------------------

/// Step amplitude as a fraction of run_current (15% = 15/100).
const VALIDATION_STEP_PERCENT: u32 = 15;

/// Settling time after applying or removing current step (microseconds).
const VALIDATION_SETTLE_US: u32 = 50_000;

/// Lambda multiplier for retry (150% = increase lambda by 50%).
/// Applied as lambda_us = lambda_us * 3 / 2.
const RETRY_LAMBDA_MULTIPLIER_NUM: u32 = 3;
const RETRY_LAMBDA_MULTIPLIER_DEN: u32 = 2;

/// Capture direction for bidirectional validation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
enum CaptureDirection {
    Forward,
    Reverse,
}

/// Current-loop validation sub-state machine (Phase 7).
#[derive(Debug, defmt::Format)]
enum CurrentValidateState {
    /// Write PI gains to TMC4671 registers.
    WriteGains,
    /// Switch to torque mode and apply current step.
    StepSetup { direction: CaptureDirection },
    /// Wait for transient to develop before capturing.
    StepSettle {
        direction: CaptureDirection,
        deadline_us: u32,
    },
    /// Capture step response into TransientBuffer at 160us intervals.
    StepCapture {
        direction: CaptureDirection,
        next_sample_us: u32,
    },
    /// Extract metrics and classify response.
    Classify,
    /// Restore safe state before finalizing.
    SafeState { deadline_us: u32 },
    /// Emit final InnerResult.
    Finalize,
}

/// Accumulated data for current-loop validation.
#[derive(Debug, Clone, defmt::Format)]
struct CurrentValidateData {
    /// The electrical model from Phase 5.
    model: ElectricalModel,
    /// Currently applied gains.
    gains: CurrentGains,
    /// Validation attempt (0 = first, 1 = retry).
    attempt: u8,
    /// Current step amplitude (absolute, PIDOUT units).
    step_amplitude: u16,
    /// Forward capture metrics (filled after forward capture).
    forward_metrics: Option<CurrentValidation>,
    /// Reverse capture metrics (filled after reverse capture).
    reverse_metrics: Option<CurrentValidation>,
    /// Final commissioning status to report.
    final_status: CommissionStatus,
    /// PID-output saturation counter: samples where the current PID's
    /// voltage output is at or near `PIDOUT_UQ_UD_LIMITS`.
    saturation_count: u16,
    /// Configured PIDOUT voltage limit for saturation detection.
    pidout_limit: u32,
}

// ---------------------------------------------------------------------------
// Diagnostic sub-states
// ---------------------------------------------------------------------------

/// ADC calibration sub-state.
#[derive(Debug, defmt::Format)]
enum AdcCalState {
    /// Configure registers and enable outputs.
    Setup,
    /// Wait for ADC outputs to settle.
    Settle { deadline_us: u32 },
    /// Collect ADC samples one per step() call.
    Sample {
        sum_i0: i32,
        sum_i1: i32,
        count: u16,
    },
    /// Validate and program offsets.
    Finalize { sum_i0: i32, sum_i1: i32 },
}

/// Coil test sub-state (used for both coil A and coil B).
#[derive(Debug, defmt::Format)]
enum CoilTestState {
    /// Configure open-loop voltage mode for this phase.
    Setup { phase: u8 },
    /// Wait for current to settle.
    Settle { phase: u8, deadline_us: u32 },
    /// Read ADC and record result.
    Read { phase: u8 },
    /// Disable and cool down before next test.
    Cooldown { phase: u8, deadline_us: u32 },
}

/// Coil check phase state (wraps both coil tests).
#[derive(Debug, defmt::Format)]
enum CoilCheckState {
    /// Testing coil A (phase 0).
    CoilA(CoilTestState),
    /// Testing coil B (phase 1).
    CoilB(CoilTestState),
}

/// Accumulated coil test observations carried between sub-phases.
#[derive(Debug, Clone, Copy, defmt::Format)]
struct CoilObservation {
    /// Current on the expected ADC channel.
    expected: i16,
    /// Current on the other ADC channel.
    other: i16,
}

/// Encoder check sub-state.
#[derive(Debug, defmt::Format)]
enum EncoderCheckState {
    /// Record baseline encoder count (captured during coil tests).
    CheckPresence,
    /// Set up open-loop mode for direction sweep.
    DirSetup,
    /// Sweep PHI_E_EXT in steps, one per tick.
    DirSweep {
        step: u16,
        deadline_us: u32,
        count_start: i32,
    },
    /// Read final count and determine direction.
    DirFinalize { count_start: i32 },
}

/// Accumulated per-coil measurements for electrical identification.
#[derive(Debug, Clone, defmt::Format)]
struct ElectricalIdData {
    /// Excitation voltage used for all measurements (PIDOUT units).
    voltage: u32,
    /// Per-coil accumulated signed current for resistance (sum of i16 readings).
    res_current_sum: [i32; 2],
    /// Per-coil number of resistance samples collected.
    res_sample_count: [u16; 2],
    /// Per-coil accumulated |delta_I| for di/dt inductance measurement.
    ind_sum_delta: [u64; 2],
    /// Per-coil number of valid di/dt cycles.
    ind_count: [u32; 2],
    /// Per-coil elapsed microseconds during di/dt measurement.
    ind_elapsed_us: [u32; 2],
}

impl ElectricalIdData {
    fn new(voltage: u32) -> Self {
        Self {
            voltage,
            res_current_sum: [0; 2],
            res_sample_count: [0; 2],
            ind_sum_delta: [0; 2],
            ind_count: [0; 2],
            ind_elapsed_us: [0; 2],
        }
    }
}

/// Electrical identification sub-state machine (Phase 5).
#[derive(Debug, defmt::Format)]
enum ElectricalIdState {
    // --- Resistance measurement (per coil) ---
    /// Configure open-loop voltage mode for resistance measurement.
    ResSetup { coil: u8 },
    /// Wait for steady-state current.
    ResSettle { coil: u8, deadline_us: u32 },
    /// Sample PID_TORQUE_FLUX_ACTUAL at 160us intervals (multiple per step).
    ResSample { coil: u8, next_sample_us: u32 },
    /// Disable motor and cool down before next measurement.
    ResDisable { coil: u8, deadline_us: u32 },

    // --- Inductance measurement (per coil, square-wave di/dt) ---
    /// Configure for di/dt toggling.
    IndSetup { coil: u8 },
    /// Brief settle before toggling.
    IndSettle { coil: u8, deadline_us: u32 },
    /// Toggle voltage and accumulate |delta_I| (chunked per step).
    IndToggle {
        coil: u8,
        remaining_cycles: u32,
        prev_current: i16,
        start_us: u32,
    },
    /// Disable motor and cool down.
    IndDisable { coil: u8, deadline_us: u32 },

    // --- Delay identification ---
    /// Configure for delay step response capture.
    DelaySetup,
    /// Brief settle at zero voltage before step.
    DelaySettle { deadline_us: u32 },
    /// Apply voltage step and start capturing.
    DelayStep,
    /// Capture 256 samples into TransientBuffer at 160us intervals.
    DelaySample { baseline: i16, next_sample_us: u32 },
    /// Disable and restore safe state.
    DelayDisable { deadline_us: u32 },

    // --- Final ---
    /// Assemble ElectricalModel and validate.
    Validate,
}

/// Inner commissioning phase state machine.
///
/// Each variant carries its own local state so that multi-step
/// operations can be chunked across 1ms `step()` calls.
#[derive(Debug, defmt::Format)]
enum InnerPhase {
    /// Phase 1: ADC offset calibration.
    AdcCalibrate(AdcCalState),
    /// Phase 2: Coil connectivity check.
    CoilCheck {
        state: CoilCheckState,
        /// Encoder count captured before coil tests (for encoder presence).
        encoder_baseline: u32,
        /// Coil A observation (filled after coil A test).
        coil_a: Option<CoilObservation>,
    },
    /// Phase 3: Phase wiring validation.
    PhaseWiringCheck {
        coil_a: CoilObservation,
        coil_b: CoilObservation,
        /// Encoder count captured before coil tests (carried forward).
        encoder_baseline: u32,
    },
    /// Phase 4: Encoder presence and direction detection.
    EncoderCheck {
        state: EncoderCheckState,
        encoder_baseline: u32,
    },
    /// Phase 5: Electrical identification (R, L, theta_e).
    ElectricalId {
        state: ElectricalIdState,
        data: ElectricalIdData,
    },
    /// Phase 6: Current-loop SIMC gain synthesis.
    CurrentTune { model: ElectricalModel },
    /// Phase 7: Current-loop step-response validation.
    CurrentValidate {
        state: CurrentValidateState,
        data: CurrentValidateData,
    },
    /// All inner phases completed successfully.
    Done(InnerResult),
    /// A phase failed.
    Failed(CommissionError),
}

// ---------------------------------------------------------------------------
// Outer safety-envelope constants
// ---------------------------------------------------------------------------

/// Default velocity cap in encoder counts per millisecond.
/// 2 rev/s at 4096 counts/rev = 8192 counts/s = ~8 counts/ms.
const VELOCITY_CAP_COUNTS_PER_MS: i32 = 8;

/// Default position window: +/- encoder counts from start position.
/// 2 revolutions at 4096 counts/rev = 8192 counts.
const POSITION_WINDOW_COUNTS: i32 = 8192;

// ---------------------------------------------------------------------------
// Mechanical identification constants (Phase 8)
// ---------------------------------------------------------------------------

/// Torque step as a fraction of run_current (25% = 25/100).
/// Spec says 20-30%; 25% is a safe midpoint.
const MECH_ID_TORQUE_STEP_PERCENT: u32 = 25;

/// Sampling interval for encoder capture (microseconds).
/// 1 kHz = 1000us between samples.
const MECH_ID_SAMPLE_PERIOD_US: u32 = 1000;

/// Settling time after removing torque (microseconds).
/// Allow the motor to decelerate and stop before reverse capture.
const MECH_ID_SETTLE_US: u32 = 500_000;

/// Forward/reverse agreement threshold (30% = 30/100).
/// If |J_fwd - J_rev| / avg > 30%, mechanical ID fails.
const MECH_ID_SYMMETRY_THRESHOLD_PCT: u32 = 30;

/// Minimum number of acceleration samples used for inertia estimate.
/// First few samples before friction/damping dominates.
const MECH_ID_ACCEL_SAMPLES: usize = 8;

/// Minimum velocity delta to consider the motor has started moving.
/// In encoder counts per sample period.
const MECH_ID_VELOCITY_ONSET_THRESHOLD: i32 = 2;

// ---------------------------------------------------------------------------
// Mechanical identification sub-states
// ---------------------------------------------------------------------------

/// Mechanical identification sub-state machine (Phase 8).
#[derive(Debug, defmt::Format)]
enum MechanicalIdState {
    /// Record starting position, configure torque mode.
    Setup,
    /// Apply torque step and begin forward capture.
    ForwardStart,
    /// Sample encoder position into transient buffer at 1 kHz.
    ForwardCapture { next_sample_us: u32 },
    /// Return to zero torque and wait for settling.
    ForwardSettle { deadline_us: u32 },
    /// Apply reverse torque step and begin reverse capture.
    ReverseStart,
    /// Sample encoder position for reverse direction.
    ReverseCapture { next_sample_us: u32 },
    /// Return to zero torque and wait for settling.
    ReverseSettle { deadline_us: u32 },
    /// Estimate mechanical parameters from captured data.
    Estimate,
    /// Validate gates and finalize.
    Finalize,
}

/// Accumulated data for mechanical identification.
#[derive(Debug, defmt::Format)]
struct MechanicalIdData {
    /// Starting encoder position recorded at setup.
    start_position: i32,
    /// Torque step amplitude (PIDOUT units).
    torque_step: u16,
    /// Forward capture: per-sample velocity (encoder counts/ms).
    /// Stored as [initial_acceleration, peak_velocity] pairs.
    forward_initial_accel: i32,
    forward_peak_velocity: i32,
    forward_delay_samples: u16,
    /// Reverse capture: same metrics.
    reverse_initial_accel: i32,
    reverse_peak_velocity: i32,
    reverse_delay_samples: u16,
}

impl MechanicalIdData {
    fn new(start_position: i32, torque_step: u16) -> Self {
        Self {
            start_position,
            torque_step,
            forward_initial_accel: 0,
            forward_peak_velocity: 0,
            forward_delay_samples: 0,
            reverse_initial_accel: 0,
            reverse_peak_velocity: 0,
            reverse_delay_samples: 0,
        }
    }
}

/// Outer safety limits, computed once at the start of outer commissioning.
#[derive(Debug, Clone, Copy, defmt::Format)]
struct OuterSafetyLimits {
    /// Maximum absolute velocity in encoder counts per millisecond.
    velocity_cap: i32,
    /// Maximum distance from start position in encoder counts.
    position_window: i32,
    /// Starting encoder position for window enforcement.
    start_position: i32,
    /// Torque cap: the configured run_current limit.
    torque_cap: u16,
    /// Previous encoder count for velocity estimation.
    prev_encoder_count: i32,
}

impl OuterSafetyLimits {
    fn new(start_position: i32, torque_cap: u16) -> Self {
        Self {
            velocity_cap: VELOCITY_CAP_COUNTS_PER_MS,
            position_window: POSITION_WINDOW_COUNTS,
            start_position,
            torque_cap,
            prev_encoder_count: start_position,
        }
    }
}

/// Outer commissioning phase state machine.
#[derive(Debug, defmt::Format)]
#[allow(dead_code)] // Later phases (9-13) are placeholders for future plans.
enum OuterPhase {
    /// Phase 8: Mechanical identification.
    MechanicalId {
        state: MechanicalIdState,
        data: MechanicalIdData,
        safety: OuterSafetyLimits,
    },
    /// Phase 9: Velocity-loop SIMC gain synthesis.
    VelocityTune {
        model: MechanicalModel,
        safety: OuterSafetyLimits,
    },
    /// Phase 10: Velocity-loop step-response validation.
    VelocityValidate {
        attempt: u8,
        safety: OuterSafetyLimits,
    },
    /// Phase 11: Position-loop P ramp-up.
    PositionTune { safety: OuterSafetyLimits },
    /// Phase 12: Filter selection.
    FilterTune { safety: OuterSafetyLimits },
    /// Phase 13: Assemble and transmit results.
    Commit,
    /// All outer phases completed.
    Done(OuterResult),
    /// A phase failed.
    Failed(CommissionError),
}

/// Which commissioning mode is active.
#[derive(Debug, defmt::Format)]
enum EngineMode {
    Inner {
        phase: InnerPhase,
        profile: CommissionProfile,
    },
    Outer {
        phase: OuterPhase,
        profile: CommissionProfile,
        mode: OperatingMode,
    },
}

/// Cooperative commissioning state machine.
///
/// Created when a `foci_commission_inner` or `foci_commission_outer`
/// command arrives. Stored as `Option<CommissioningEngine>` in the
/// P1 `tmc_control` task. Each 1kHz tick calls `step()`.
pub struct CommissioningEngine {
    oid: u8,
    engine_mode: EngineMode,
    buf: TransientBuffer,
}

impl CommissioningEngine {
    /// Create an engine for inner (electrical) commissioning.
    pub fn new_inner(oid: u8, profile: CommissionProfile) -> Self {
        Self {
            oid,
            engine_mode: EngineMode::Inner {
                phase: InnerPhase::AdcCalibrate(AdcCalState::Setup),
                profile,
            },
            buf: TransientBuffer::new(),
        }
    }

    /// Create an engine for outer (mechanical) commissioning.
    pub fn new_outer(oid: u8, profile: CommissionProfile, mode: OperatingMode) -> Self {
        // Safety limits and data are initialized with placeholder values.
        // The MechanicalIdState::Setup sub-state reads the actual encoder
        // position and current limit from the backend on the first step().
        let safety = OuterSafetyLimits::new(0, 0);
        let data = MechanicalIdData::new(0, 0);
        Self {
            oid,
            engine_mode: EngineMode::Outer {
                phase: OuterPhase::MechanicalId {
                    state: MechanicalIdState::Setup,
                    data,
                    safety,
                },
                profile,
                mode,
            },
            buf: TransientBuffer::new(),
        }
    }

    /// The OID this engine was started for.
    pub fn oid(&self) -> u8 {
        self.oid
    }

    /// Advance the state machine by one tick (max 1ms).
    ///
    /// Returns a [`StepResult`] indicating whether to continue,
    /// report a phase transition, or finalize.
    ///
    /// The method destructures `engine_mode` to pass `phase` and
    /// `buf` separately, avoiding a double `&mut self` borrow.
    pub async fn step<B: CommissioningBackend>(&mut self, backend: &mut B) -> StepResult {
        // E-stop check at the top of every step
        if backend.is_shutdown_requested() {
            return self.abort(CommissionError::Shutdown);
        }

        match &mut self.engine_mode {
            EngineMode::Inner {
                ref mut phase,
                profile,
            } => Self::step_inner(phase, &mut self.buf, *profile, backend).await,
            EngineMode::Outer {
                ref mut phase,
                profile,
                mode,
            } => Self::step_outer(phase, &mut self.buf, *profile, *mode, backend).await,
        }
    }

    /// Abort commissioning, restoring safe state.
    fn abort(&mut self, error: CommissionError) -> StepResult {
        match &self.engine_mode {
            EngineMode::Inner { .. } => StepResult::InnerFailed(error),
            EngineMode::Outer { .. } => StepResult::OuterFailed(error),
        }
    }

    /// Step inner phase state machine.
    ///
    /// Dispatches to per-phase handlers. Each handler advances the
    /// sub-state by one chunk of work (bounded to ~1ms). Phase
    /// transitions are signalled by replacing `*phase` and returning
    /// `StepResult::PhaseChanged`.
    async fn step_inner<B: CommissioningBackend>(
        phase: &mut InnerPhase,
        buf: &mut TransientBuffer,
        profile: CommissionProfile,
        backend: &mut B,
    ) -> StepResult {
        match phase {
            InnerPhase::AdcCalibrate(_) => Self::step_adc_calibrate(phase, backend).await,
            InnerPhase::CoilCheck { .. } => Self::step_coil_check(phase, backend).await,
            InnerPhase::PhaseWiringCheck {
                coil_a,
                coil_b,
                encoder_baseline,
            } => {
                let a = *coil_a;
                let b = *coil_b;
                let baseline = *encoder_baseline;
                Self::step_phase_wiring(phase, a, b, baseline)
            }
            InnerPhase::EncoderCheck { .. } => Self::step_encoder_check(phase, backend).await,
            InnerPhase::ElectricalId { .. } => Self::step_elec_id(phase, buf, backend).await,
            InnerPhase::CurrentTune { .. } => {
                Self::step_current_tune(phase, profile, backend).await
            }
            InnerPhase::CurrentValidate { .. } => {
                Self::step_current_validate(phase, buf, profile, backend).await
            }
            InnerPhase::Done(result) => StepResult::InnerDone(result.clone()),
            InnerPhase::Failed(err) => StepResult::InnerFailed(*err),
        }
    }

    // -------------------------------------------------------------------
    // Phase 1: ADC Calibration
    // -------------------------------------------------------------------

    /// Advance ADC calibration sub-state.
    ///
    /// Ported from `selftest_adc_calibrate`: configures TMC4671 for
    /// zero-current ADC sampling (PWM_CHOP=0), collects 128 samples
    /// one per step() call, validates offsets, programs them back.
    async fn step_adc_calibrate<B: CommissioningBackend>(
        phase: &mut InnerPhase,
        backend: &mut B,
    ) -> StepResult {
        use tmc4671::registers::*;

        let cal_state = match phase {
            InnerPhase::AdcCalibrate(cal_state) => cal_state,
            _ => return StepResult::Continue,
        };

        match cal_state {
            AdcCalState::Setup => {
                // Stop motor, disable outputs, set voltage limit.
                backend.set_enable_pin(false);
                if backend
                    .write_register(MODE_RAMP_MODE_MOTION, 0)
                    .await
                    .is_err()
                    || backend
                        .write_register(PIDOUT_UQ_UD_LIMITS, 1024)
                        .await
                        .is_err()
                {
                    return Self::fail_inner(phase, CommissionError::Spi);
                }

                // Verify voltage limit write.
                match backend.read_register(PIDOUT_UQ_UD_LIMITS).await {
                    Ok(1024) => {}
                    _ => return Self::fail_inner(phase, CommissionError::Spi),
                }

                let _ = backend.write_register(PID_TORQUE_FLUX_TARGET, 0).await;
                let _ = backend.write_register(UQ_UD_EXT, 0).await;

                // PWM config: chopping disabled for calibration.
                let _ = backend.write_register(PWM_POLARITIES, 0).await;
                let _ = backend
                    .write_register(PWM_MAXCNT, backend.pwm_maxcnt() as u32)
                    .await;
                let _ = backend
                    .write_register(PWM_BBM_H_BBM_L, backend.bbm_register())
                    .await;
                if backend.write_register(PWM_SV_CHOP, 0).await.is_err() {
                    return Self::fail_inner(phase, CommissionError::Spi);
                }

                // Verify MODE=0 and PWM_SV_CHOP=0.
                let mode_ok = backend
                    .read_register(MODE_RAMP_MODE_MOTION)
                    .await
                    .is_ok_and(|v| v == 0);
                let chop_ok = backend
                    .read_register(PWM_SV_CHOP)
                    .await
                    .is_ok_and(|v| v == 0);
                if !mode_ok || !chop_ok {
                    return Self::fail_inner(phase, CommissionError::Spi);
                }

                // ADC configuration.
                let _ = backend
                    .write_register(DSADC_MDEC_B_MDEC_A, backend.dsadc_mdec_register())
                    .await;
                let _ = backend
                    .write_register(ADC_I_SELECT, (1 << 28) | (1 << 8))
                    .await;

                // Motor type: 2-phase stepper, 50 pole pairs.
                let _ = backend
                    .write_register(MOTOR_TYPE_N_POLE_PAIRS, (2 << 16) | 50)
                    .await;

                // Clear offsets, enable outputs.
                let _ = backend.write_register(ADC_I0_SCALE_OFFSET, 256 << 16).await;
                let _ = backend.write_register(ADC_I1_SCALE_OFFSET, 256 << 16).await;
                backend.set_enable_pin(true);

                let deadline = backend.elapsed_us().wrapping_add(ADC_CAL_SETTLE_US);
                *cal_state = AdcCalState::Settle {
                    deadline_us: deadline,
                };
                StepResult::Continue
            }

            AdcCalState::Settle { deadline_us } => {
                if backend.elapsed_us().wrapping_sub(*deadline_us) < 0x8000_0000 {
                    // Deadline passed (unsigned comparison).
                    *cal_state = AdcCalState::Sample {
                        sum_i0: 0,
                        sum_i1: 0,
                        count: 0,
                    };
                }
                StepResult::Continue
            }

            AdcCalState::Sample {
                sum_i0,
                sum_i1,
                count,
            } => {
                // One sample per step() call (~1ms between samples).
                match backend.read_register(ADC_IWY_IUX).await {
                    Ok(raw) => {
                        let i_ux = (raw & 0xFFFF) as i16 as i32;
                        let i_wy = ((raw >> 16) & 0xFFFF) as i16 as i32;
                        *sum_i0 += i_ux;
                        *sum_i1 += i_wy;
                        *count += 1;
                    }
                    Err(_) => {
                        return Self::fail_inner(phase, CommissionError::Spi);
                    }
                }
                if *count >= ADC_CAL_SAMPLES {
                    *cal_state = AdcCalState::Finalize {
                        sum_i0: *sum_i0,
                        sum_i1: *sum_i1,
                    };
                }
                StepResult::Continue
            }

            AdcCalState::Finalize { sum_i0, sum_i1 } => {
                let n = ADC_CAL_SAMPLES as i32;
                let offset_i0 = (*sum_i0 / n) as i16 as u16;
                let offset_i1 = (*sum_i1 / n) as i16 as u16;

                // Validate offsets in expected range.
                if !(ADC_OFFSET_MIN..=ADC_OFFSET_MAX).contains(&offset_i0)
                    || !(ADC_OFFSET_MIN..=ADC_OFFSET_MAX).contains(&offset_i1)
                {
                    return Self::fail_inner(phase, CommissionError::AdcFault);
                }

                // Program calibrated offsets.
                let _ = backend
                    .write_register(ADC_I0_SCALE_OFFSET, (256 << 16) | offset_i0 as u32)
                    .await;
                let _ = backend
                    .write_register(ADC_I1_SCALE_OFFSET, (256 << 16) | offset_i1 as u32)
                    .await;

                // Re-enable PWM chopping.
                let _ = backend.write_register(PWM_SV_CHOP, 7).await;

                // Capture encoder baseline for later presence check.
                let encoder_baseline = backend.read_encoder_count().await;

                // Transition to CoilCheck.
                *phase = InnerPhase::CoilCheck {
                    state: CoilCheckState::CoilA(CoilTestState::Setup { phase: 0 }),
                    encoder_baseline,
                    coil_a: None,
                };
                StepResult::PhaseChanged(PhaseId::CoilCheck)
            }
        }
    }

    // -------------------------------------------------------------------
    // Phase 2: Coil Connectivity Check
    // -------------------------------------------------------------------

    /// Advance coil check sub-state.
    ///
    /// Tests coil A then coil B via `UQ_UD_EXT` (MODE=8). Validates
    /// current exceeds `MOTOR_CURRENT_THRESHOLD` on the expected
    /// ADC channel. Carries observations forward for phase wiring
    /// check.
    async fn step_coil_check<B: CommissioningBackend>(
        phase: &mut InnerPhase,
        backend: &mut B,
    ) -> StepResult {
        use tmc4671::registers::*;

        // Extract mutable fields from phase.
        let (state, encoder_baseline, coil_a_obs) = match phase {
            InnerPhase::CoilCheck {
                state,
                encoder_baseline,
                coil_a,
            } => (state, *encoder_baseline, coil_a),
            _ => return StepResult::Continue,
        };

        let is_coil_b = matches!(state, CoilCheckState::CoilB(_));

        match state {
            CoilCheckState::CoilA(ref mut coil_test) | CoilCheckState::CoilB(ref mut coil_test) => {
                match coil_test {
                    CoilTestState::Setup { phase: coil_phase } => {
                        let p = *coil_phase;
                        // Set PHI_E source to external.
                        if backend.write_register(PHI_E_SELECTION, 1).await.is_err() {
                            return Self::fail_inner(phase, CommissionError::Spi);
                        }
                        // Angle: 0 for coil A (D-axis), 16384 for coil B (Q-axis).
                        let phi_e = if p == 0 { 0u32 } else { 16384 };
                        if backend.write_register(PHI_E_EXT, phi_e).await.is_err() {
                            return Self::fail_inner(phase, CommissionError::Spi);
                        }
                        // Apply voltage.
                        let voltage = backend.excitation_voltage() as u32;
                        if backend
                            .write_register(UQ_UD_EXT, voltage & 0xFFFF)
                            .await
                            .is_err()
                        {
                            return Self::fail_inner(phase, CommissionError::Spi);
                        }
                        if backend
                            .write_register(MODE_RAMP_MODE_MOTION, 8)
                            .await
                            .is_err()
                        {
                            return Self::fail_inner(phase, CommissionError::Spi);
                        }
                        backend.set_enable_pin(true);

                        let deadline = backend.elapsed_us().wrapping_add(COIL_SETTLE_US);
                        *coil_test = CoilTestState::Settle {
                            phase: p,
                            deadline_us: deadline,
                        };
                        StepResult::Continue
                    }

                    CoilTestState::Settle {
                        phase: coil_phase,
                        deadline_us,
                    } => {
                        if backend.elapsed_us().wrapping_sub(*deadline_us) < 0x8000_0000 {
                            *coil_test = CoilTestState::Read { phase: *coil_phase };
                        }
                        StepResult::Continue
                    }

                    CoilTestState::Read { phase: coil_phase } => {
                        let p = *coil_phase;
                        let raw = match backend.read_register(ADC_IWY_IUX).await {
                            Ok(v) => v,
                            Err(_) => {
                                return Self::fail_inner(phase, CommissionError::Spi);
                            }
                        };

                        // Stop motor and disable.
                        let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;
                        let _ = backend.write_register(UQ_UD_EXT, 0).await;
                        backend.set_enable_pin(false);

                        let i_ux = (raw & 0xFFFF) as i16;
                        let i_wy = ((raw >> 16) & 0xFFFF) as i16;

                        let obs = if p == 0 {
                            CoilObservation {
                                expected: i_ux,
                                other: i_wy,
                            }
                        } else {
                            CoilObservation {
                                expected: i_wy,
                                other: i_ux,
                            }
                        };

                        // Validate coil presence.
                        if obs.expected.unsigned_abs() <= MOTOR_CURRENT_THRESHOLD {
                            return Self::fail_inner(phase, CommissionError::CoilFault);
                        }

                        let deadline = backend.elapsed_us().wrapping_add(COIL_COOLDOWN_US);
                        *coil_test = CoilTestState::Cooldown {
                            phase: p,
                            deadline_us: deadline,
                        };

                        // Store observation.
                        if !is_coil_b {
                            *coil_a_obs = Some(obs);
                        } else {
                            // Both coils tested. Transition to PhaseWiringCheck.
                            let a = coil_a_obs.unwrap_or(CoilObservation {
                                expected: 0,
                                other: 0,
                            });
                            *phase = InnerPhase::PhaseWiringCheck {
                                coil_a: a,
                                coil_b: obs,
                                encoder_baseline,
                            };
                            return StepResult::PhaseChanged(PhaseId::PhaseWiringCheck);
                        }
                        StepResult::Continue
                    }

                    CoilTestState::Cooldown {
                        phase: _coil_phase,
                        deadline_us,
                    } => {
                        if backend.elapsed_us().wrapping_sub(*deadline_us) < 0x8000_0000 {
                            // Coil A done, move to coil B.
                            *state = CoilCheckState::CoilB(CoilTestState::Setup { phase: 1 });
                        }
                        StepResult::Continue
                    }
                }
            }
        }
    }

    // -------------------------------------------------------------------
    // Phase 3: Phase Wiring Check
    // -------------------------------------------------------------------

    /// Validate phase wiring from coil test observations.
    ///
    /// Current on the expected ADC channel must exceed current on
    /// the other channel. If reversed, the motor phases are swapped.
    fn step_phase_wiring(
        phase: &mut InnerPhase,
        coil_a: CoilObservation,
        coil_b: CoilObservation,
        encoder_baseline: u32,
    ) -> StepResult {
        // Phase wiring check: expected channel must dominate.
        if coil_a.other.unsigned_abs() >= coil_a.expected.unsigned_abs() {
            return Self::fail_inner(phase, CommissionError::PhaseFault);
        }
        if coil_b.other.unsigned_abs() >= coil_b.expected.unsigned_abs() {
            return Self::fail_inner(phase, CommissionError::PhaseFault);
        }

        // Transition to encoder check.
        *phase = InnerPhase::EncoderCheck {
            state: EncoderCheckState::CheckPresence,
            encoder_baseline,
        };
        StepResult::PhaseChanged(PhaseId::EncoderCheck)
    }

    // -------------------------------------------------------------------
    // Phase 4: Encoder Presence and Direction
    // -------------------------------------------------------------------

    /// Advance encoder check sub-state.
    ///
    /// Presence: compares current ABN count against baseline captured
    /// before coil tests. Any non-zero delta = encoder present.
    ///
    /// Direction: sweeps one electrical revolution via PHI_E_EXT in
    /// resumable sub-steps, then checks whether count increased or
    /// decreased.
    async fn step_encoder_check<B: CommissioningBackend>(
        phase: &mut InnerPhase,
        backend: &mut B,
    ) -> StepResult {
        use tmc4671::registers::*;

        let (state, encoder_baseline) = match phase {
            InnerPhase::EncoderCheck {
                state,
                encoder_baseline,
            } => (state, *encoder_baseline),
            _ => return StepResult::Continue,
        };

        match state {
            EncoderCheckState::CheckPresence => {
                let count_now = backend.read_encoder_count().await;
                let delta = (count_now as i32)
                    .wrapping_sub(encoder_baseline as i32)
                    .unsigned_abs();
                if delta == 0 {
                    return Self::fail_inner(phase, CommissionError::EncoderFault);
                }
                *state = EncoderCheckState::DirSetup;
                StepResult::Continue
            }

            EncoderCheckState::DirSetup => {
                let count_start = backend.read_encoder_count().await as i32;

                // Set up open-loop voltage mode.
                if backend.write_register(PHI_E_SELECTION, 1).await.is_err()
                    || backend
                        .write_register(UQ_UD_EXT, backend.excitation_voltage() as u32 & 0xFFFF)
                        .await
                        .is_err()
                    || backend
                        .write_register(MODE_RAMP_MODE_MOTION, 8)
                        .await
                        .is_err()
                {
                    return Self::fail_inner(phase, CommissionError::Spi);
                }
                backend.set_enable_pin(true);

                // Write initial PHI_E_EXT=0.
                let _ = backend.write_register(PHI_E_EXT, 0).await;

                let deadline = backend.elapsed_us().wrapping_add(ENCODER_DIR_STEP_US);
                *state = EncoderCheckState::DirSweep {
                    step: 0,
                    deadline_us: deadline,
                    count_start,
                };
                StepResult::Continue
            }

            EncoderCheckState::DirSweep {
                step,
                deadline_us,
                count_start,
            } => {
                if backend.elapsed_us().wrapping_sub(*deadline_us) >= 0x8000_0000 {
                    // Not yet time for next step.
                    return StepResult::Continue;
                }

                *step += 1;
                if *step > ENCODER_DIR_STEPS {
                    let cs = *count_start;
                    *state = EncoderCheckState::DirFinalize { count_start: cs };
                    return StepResult::Continue;
                }

                // Advance PHI_E_EXT.
                let phi = (65536u64 * *step as u64 / ENCODER_DIR_STEPS as u64) as u32;
                let _ = backend.write_register(PHI_E_EXT, phi).await;

                *deadline_us = backend.elapsed_us().wrapping_add(ENCODER_DIR_STEP_US);
                StepResult::Continue
            }

            EncoderCheckState::DirFinalize { count_start } => {
                let count_end = backend.read_encoder_count().await as i32;

                // Stop and disable (always, regardless of result).
                let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;
                let _ = backend.write_register(UQ_UD_EXT, 0).await;
                backend.set_enable_pin(false);

                let delta = count_end.wrapping_sub(*count_start);
                if delta == 0 {
                    return Self::fail_inner(phase, CommissionError::EncoderFault);
                }

                // Direction detected. Transition to ElectricalId.
                // Direction value (0=forward, 1=reversed) is carried as
                // commissioning-local data; persistence is outside this plan.
                let voltage = backend.excitation_voltage() as u32;
                *phase = InnerPhase::ElectricalId {
                    state: ElectricalIdState::ResSetup { coil: 0 },
                    data: ElectricalIdData::new(voltage),
                };
                StepResult::PhaseChanged(PhaseId::ElectricalId)
            }
        }
    }

    // -------------------------------------------------------------------
    // Phase 5: Electrical Identification
    // -------------------------------------------------------------------

    /// Advance electrical identification sub-state.
    ///
    /// Measures per-coil resistance (1000 samples at 160us in steady
    /// state), per-coil inductance (square-wave di/dt), and delay
    /// (transient capture with two-point fit). Assembles an
    /// `ElectricalModel` and validates before transitioning.
    async fn step_elec_id<B: CommissioningBackend>(
        phase: &mut InnerPhase,
        buf: &mut TransientBuffer,
        backend: &mut B,
    ) -> StepResult {
        use tmc4671::registers::*;

        let (elec_id_state, data) = match phase {
            InnerPhase::ElectricalId { state, data } => (state, data),
            _ => return StepResult::Continue,
        };

        match elec_id_state {
            // ---------------------------------------------------------------
            // Resistance measurement (per coil)
            // ---------------------------------------------------------------
            ElectricalIdState::ResSetup { coil } => {
                let coil_idx = *coil;
                // Configure open-loop voltage mode.
                if backend.write_register(PHI_E_SELECTION, 1).await.is_err() {
                    return Self::fail_inner(phase, CommissionError::Spi);
                }
                let phi_e = if coil_idx == 0 { 0u32 } else { 16384 };
                if backend.write_register(PHI_E_EXT, phi_e).await.is_err() {
                    return Self::fail_inner(phase, CommissionError::Spi);
                }
                if backend
                    .write_register(UQ_UD_EXT, data.voltage & 0xFFFF)
                    .await
                    .is_err()
                {
                    return Self::fail_inner(phase, CommissionError::Spi);
                }
                if backend
                    .write_register(MODE_RAMP_MODE_MOTION, 8)
                    .await
                    .is_err()
                {
                    return Self::fail_inner(phase, CommissionError::Spi);
                }
                backend.set_enable_pin(true);

                let deadline = backend.elapsed_us().wrapping_add(RES_SETTLE_US);
                *elec_id_state = ElectricalIdState::ResSettle {
                    coil: coil_idx,
                    deadline_us: deadline,
                };
                StepResult::Continue
            }

            ElectricalIdState::ResSettle { coil, deadline_us } => {
                if backend.elapsed_us().wrapping_sub(*deadline_us) < 0x8000_0000 {
                    let next = backend.elapsed_us().wrapping_add(ELEC_SAMPLE_PERIOD_US);
                    *elec_id_state = ElectricalIdState::ResSample {
                        coil: *coil,
                        next_sample_us: next,
                    };
                }
                StepResult::Continue
            }

            ElectricalIdState::ResSample {
                coil,
                next_sample_us,
            } => {
                let coil_idx = *coil as usize;
                // Take as many samples as fit within this step's time budget.
                loop {
                    if data.res_sample_count[coil_idx] >= RES_SAMPLE_COUNT {
                        // All samples collected. Disable motor.
                        let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;
                        let _ = backend.write_register(UQ_UD_EXT, 0).await;
                        backend.set_enable_pin(false);
                        let deadline = backend.elapsed_us().wrapping_add(ELEC_COOLDOWN_US);
                        *elec_id_state = ElectricalIdState::ResDisable {
                            coil: *coil,
                            deadline_us: deadline,
                        };
                        return StepResult::Continue;
                    }
                    // Wait for next sample deadline.
                    if backend.elapsed_us().wrapping_sub(*next_sample_us) >= 0x8000_0000 {
                        break; // Not yet time, yield.
                    }
                    // Take one sample.
                    match backend.read_register(PID_TORQUE_FLUX_ACTUAL).await {
                        Ok(actual) => {
                            let current = (actual & 0xFFFF) as i16;
                            data.res_current_sum[coil_idx] += current as i32;
                            data.res_sample_count[coil_idx] += 1;
                        }
                        Err(_) => {
                            return Self::fail_inner(phase, CommissionError::Spi);
                        }
                    }
                    *next_sample_us = next_sample_us.wrapping_add(ELEC_SAMPLE_PERIOD_US);
                }
                StepResult::Continue
            }

            ElectricalIdState::ResDisable { coil, deadline_us } => {
                if backend.elapsed_us().wrapping_sub(*deadline_us) >= 0x8000_0000 {
                    return StepResult::Continue; // Not yet.
                }
                let coil_idx = *coil;
                if coil_idx == 0 {
                    // Move to coil B resistance.
                    *elec_id_state = ElectricalIdState::ResSetup { coil: 1 };
                } else {
                    // Both resistance measurements done. Start inductance.
                    *elec_id_state = ElectricalIdState::IndSetup { coil: 0 };
                }
                StepResult::Continue
            }

            // ---------------------------------------------------------------
            // Inductance measurement (per coil, square-wave di/dt)
            // ---------------------------------------------------------------
            ElectricalIdState::IndSetup { coil } => {
                let coil_idx = *coil;
                if backend.write_register(PHI_E_SELECTION, 1).await.is_err() {
                    return Self::fail_inner(phase, CommissionError::Spi);
                }
                let phi_e = if coil_idx == 0 { 0u32 } else { 16384 };
                if backend.write_register(PHI_E_EXT, phi_e).await.is_err() {
                    return Self::fail_inner(phase, CommissionError::Spi);
                }
                // Start at zero voltage.
                if backend.write_register(UQ_UD_EXT, 0).await.is_err() {
                    return Self::fail_inner(phase, CommissionError::Spi);
                }
                if backend
                    .write_register(MODE_RAMP_MODE_MOTION, 8)
                    .await
                    .is_err()
                {
                    return Self::fail_inner(phase, CommissionError::Spi);
                }
                backend.set_enable_pin(true);

                let deadline = backend.elapsed_us().wrapping_add(DIDT_PRE_SETTLE_US);
                *elec_id_state = ElectricalIdState::IndSettle {
                    coil: coil_idx,
                    deadline_us: deadline,
                };
                StepResult::Continue
            }

            ElectricalIdState::IndSettle { coil, deadline_us } => {
                if backend.elapsed_us().wrapping_sub(*deadline_us) >= 0x8000_0000 {
                    return StepResult::Continue; // Not yet.
                }
                // Read initial current before toggling.
                let initial_current = match backend.read_register(PID_TORQUE_FLUX_ACTUAL).await {
                    Ok(actual) => (actual & 0xFFFF) as i16,
                    Err(_) => return Self::fail_inner(phase, CommissionError::Spi),
                };
                let total_cycles = 5000u32; // From TmcConfig::didt_cycles default
                let start = backend.elapsed_us();
                *elec_id_state = ElectricalIdState::IndToggle {
                    coil: *coil,
                    remaining_cycles: total_cycles,
                    prev_current: initial_current,
                    start_us: start,
                };
                StepResult::Continue
            }

            ElectricalIdState::IndToggle {
                coil,
                remaining_cycles,
                prev_current,
                start_us,
            } => {
                let coil_idx = *coil as usize;
                let voltage = data.voltage;
                let v_pos = voltage & 0xFFFF;
                let v_neg = (-(voltage as i32) as u32) & 0xFFFF;

                let mut cycles_this_step = 0u32;

                while *remaining_cycles > 0 && cycles_this_step < DIDT_CYCLES_PER_STEP {
                    // Determine polarity based on which cycle we're on.
                    let total_done = 5000u32.saturating_sub(*remaining_cycles);
                    let v = if total_done % 2 == 0 { v_pos } else { v_neg };
                    let _ = backend.write_register(UQ_UD_EXT, v).await;

                    if let Ok(actual) = backend.read_register(PID_TORQUE_FLUX_ACTUAL).await {
                        let current = (actual & 0xFFFF) as i16;
                        let delta = (current as i32 - *prev_current as i32).unsigned_abs();
                        data.ind_sum_delta[coil_idx] += delta as u64;
                        data.ind_count[coil_idx] += 1;
                        *prev_current = current;
                    }
                    *remaining_cycles -= 1;
                    cycles_this_step += 1;
                }

                if *remaining_cycles == 0 {
                    // All cycles done. Record elapsed time.
                    let end = backend.elapsed_us();
                    data.ind_elapsed_us[coil_idx] = end.wrapping_sub(*start_us);

                    // Disable motor.
                    let _ = backend.write_register(UQ_UD_EXT, 0).await;
                    let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;
                    backend.set_enable_pin(false);

                    let deadline = backend.elapsed_us().wrapping_add(ELEC_COOLDOWN_US);
                    *elec_id_state = ElectricalIdState::IndDisable {
                        coil: *coil,
                        deadline_us: deadline,
                    };
                }
                StepResult::Continue
            }

            ElectricalIdState::IndDisable { coil, deadline_us } => {
                if backend.elapsed_us().wrapping_sub(*deadline_us) >= 0x8000_0000 {
                    return StepResult::Continue; // Not yet.
                }
                let coil_idx = *coil;
                if coil_idx == 0 {
                    // Move to coil B inductance.
                    *elec_id_state = ElectricalIdState::IndSetup { coil: 1 };
                } else {
                    // Both inductance measurements done. Start delay capture.
                    *elec_id_state = ElectricalIdState::DelaySetup;
                }
                StepResult::Continue
            }

            // ---------------------------------------------------------------
            // Delay identification (transient capture)
            // ---------------------------------------------------------------
            ElectricalIdState::DelaySetup => {
                // Configure for step response on coil A (D-axis).
                if backend.write_register(PHI_E_SELECTION, 1).await.is_err()
                    || backend.write_register(PHI_E_EXT, 0).await.is_err()
                    || backend.write_register(UQ_UD_EXT, 0).await.is_err()
                    || backend
                        .write_register(MODE_RAMP_MODE_MOTION, 8)
                        .await
                        .is_err()
                {
                    return Self::fail_inner(phase, CommissionError::Spi);
                }
                backend.set_enable_pin(true);

                // Reset transient buffer for 160us sampling.
                buf.reset(ELEC_SAMPLE_PERIOD_US as u16);

                let deadline = backend.elapsed_us().wrapping_add(DIDT_PRE_SETTLE_US);
                *elec_id_state = ElectricalIdState::DelaySettle {
                    deadline_us: deadline,
                };
                StepResult::Continue
            }

            ElectricalIdState::DelaySettle { deadline_us } => {
                if backend.elapsed_us().wrapping_sub(*deadline_us) >= 0x8000_0000 {
                    return StepResult::Continue; // Not yet.
                }
                *elec_id_state = ElectricalIdState::DelayStep;
                StepResult::Continue
            }

            ElectricalIdState::DelayStep => {
                // Read baseline current (should be ~0 with UQ_UD_EXT=0).
                let baseline = match backend.read_register(PID_TORQUE_FLUX_ACTUAL).await {
                    Ok(actual) => (actual & 0xFFFF) as i16,
                    Err(_) => return Self::fail_inner(phase, CommissionError::Spi),
                };

                // Apply voltage step.
                if backend
                    .write_register(UQ_UD_EXT, data.voltage & 0xFFFF)
                    .await
                    .is_err()
                {
                    return Self::fail_inner(phase, CommissionError::Spi);
                }

                let next = backend.elapsed_us().wrapping_add(ELEC_SAMPLE_PERIOD_US);
                *elec_id_state = ElectricalIdState::DelaySample {
                    baseline,
                    next_sample_us: next,
                };
                StepResult::Continue
            }

            ElectricalIdState::DelaySample {
                baseline,
                next_sample_us,
            } => {
                // Capture samples into TransientBuffer at 160us intervals.
                loop {
                    if buf.len() >= DELAY_SAMPLE_COUNT || buf.is_full() {
                        // Capture complete. Disable motor.
                        let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;
                        let _ = backend.write_register(UQ_UD_EXT, 0).await;
                        backend.set_enable_pin(false);
                        let deadline = backend.elapsed_us().wrapping_add(ELEC_COOLDOWN_US);
                        *elec_id_state = ElectricalIdState::DelayDisable {
                            deadline_us: deadline,
                        };
                        return StepResult::Continue;
                    }
                    if backend.elapsed_us().wrapping_sub(*next_sample_us) >= 0x8000_0000 {
                        break; // Not yet time, yield.
                    }
                    match backend.read_register(PID_TORQUE_FLUX_ACTUAL).await {
                        Ok(actual) => {
                            // Store absolute current relative to baseline.
                            let current = (actual & 0xFFFF) as i16;
                            let relative = (current as i32 - *baseline as i32).unsigned_abs();
                            buf.push(relative);
                        }
                        Err(_) => {
                            return Self::fail_inner(phase, CommissionError::Spi);
                        }
                    }
                    *next_sample_us = next_sample_us.wrapping_add(ELEC_SAMPLE_PERIOD_US);
                }
                StepResult::Continue
            }

            ElectricalIdState::DelayDisable { deadline_us } => {
                if backend.elapsed_us().wrapping_sub(*deadline_us) >= 0x8000_0000 {
                    return StepResult::Continue; // Not yet.
                }
                *elec_id_state = ElectricalIdState::Validate;
                StepResult::Continue
            }

            // ---------------------------------------------------------------
            // Model assembly and validation
            // ---------------------------------------------------------------
            ElectricalIdState::Validate => {
                let voltage = data.voltage;

                // --- Resistance ---
                // Per-coil: R_int = V / |I_avg|
                let mut r_int_per_coil = [0u32; 2];
                let mut r_mohm_per_coil = [0u32; 2];
                for coil_idx in 0..2 {
                    let count = data.res_sample_count[coil_idx] as i32;
                    if count == 0 {
                        return Self::fail_inner(phase, CommissionError::ElectricalIdFailed);
                    }
                    let avg_current = (data.res_current_sum[coil_idx] / count) as i16;
                    let abs_current = avg_current.unsigned_abs() as u32;
                    if abs_current == 0 {
                        return Self::fail_inner(phase, CommissionError::ElectricalIdFailed);
                    }
                    r_int_per_coil[coil_idx] = voltage / abs_current;
                    r_mohm_per_coil[coil_idx] = (voltage as u64 * 1000 / abs_current as u64) as u32;
                }
                // Average of both coils.
                let r_int = (r_int_per_coil[0] + r_int_per_coil[1]) / 2;
                let r_mohm = (r_mohm_per_coil[0] + r_mohm_per_coil[1]) / 2;
                if r_int == 0 {
                    return Self::fail_inner(phase, CommissionError::ElectricalIdFailed);
                }

                // --- Inductance (from di/dt) ---
                // L_int = V * total_time_s / sum_delta
                // tau_e_us = L_int / R_int * 1e6
                // Since L_int = V * elapsed_us * 1e-6 / (sum_delta / count):
                //   L_int = V * elapsed_us * count / (sum_delta * 1e6)
                // And tau_e_us = L_int / R_int * 1e6:
                //   tau_e_us = V * elapsed_us * count / (sum_delta * R_int)
                let mut tau_per_coil = [0u32; 2];
                for (tau_result, ((count, sum_delta), elapsed)) in tau_per_coil
                    .iter_mut()
                    .zip(data.ind_count.iter().zip(data.ind_sum_delta.iter()))
                    .zip(data.ind_elapsed_us.iter())
                    .map(|((t, (c, s)), e)| (t, ((c, s), e)))
                {
                    if *count == 0 || *sum_delta == 0 || *elapsed == 0 {
                        continue;
                    }
                    let avg_delta = (*sum_delta / *count as u64) as u32;
                    if avg_delta < DIDT_MIN_DELTA {
                        continue;
                    }
                    // dt_per_cycle_us = elapsed / count
                    // L_int = V * dt_per_cycle_us * 1e-6 / avg_delta
                    // tau_us = L_int / R_int * 1e6 = V * dt_per_cycle_us / (avg_delta * R_int)
                    let dt_per_cycle_us = *elapsed / *count;
                    *tau_result = (voltage as u64 * dt_per_cycle_us as u64
                        / (avg_delta as u64 * r_int as u64))
                        as u32;
                }

                // Use the valid coil(s) for tau.
                let tau_e_us = match (tau_per_coil[0] > 0, tau_per_coil[1] > 0) {
                    (true, true) => (tau_per_coil[0] + tau_per_coil[1]) / 2,
                    (true, false) => tau_per_coil[0],
                    (false, true) => tau_per_coil[1],
                    (false, false) => {
                        return Self::fail_inner(phase, CommissionError::ElectricalIdFailed);
                    }
                };

                // L_int from tau: L_int = R_int * tau_e_us (internal units,
                // such that tau_e_us = L_int / R_int holds directly).
                let l_int = r_int * tau_e_us;

                // Physical L: L_uh = R_mohm * tau_e_us / 1000
                // (since L = R*tau; mohm*us = nH; nH/1000 = uH)
                let l_uh = r_mohm as u64 * tau_e_us as u64 / 1000;

                if l_int == 0 || l_uh == 0 {
                    return Self::fail_inner(phase, CommissionError::ElectricalIdFailed);
                }

                // --- Delay identification (from transient buffer) ---
                let samples = buf.samples();
                let sample_count = samples.len();

                // Find steady-state from last 10 samples.
                let steady_state = if sample_count >= 20 {
                    let tail_start = sample_count - 10;
                    let tail_sum: u32 = samples[tail_start..].iter().sum();
                    tail_sum / 10
                } else if !samples.is_empty() {
                    // Short capture: use last sample as approximation.
                    samples[sample_count - 1]
                } else {
                    return Self::fail_inner(phase, CommissionError::ElectricalIdFailed);
                };

                // Onset: first sample above noise threshold.
                let onset_threshold = DELAY_ONSET_THRESHOLD as u32;
                let mut theta_e_us = 0u32;
                for (i, &sample) in samples.iter().enumerate() {
                    if sample > onset_threshold {
                        theta_e_us = i as u32 * ELEC_SAMPLE_PERIOD_US;
                        break;
                    }
                }

                // 63% rise: first sample >= 0.632 * steady_state.
                let rise_threshold = (steady_state as u64 * RISE_63_PCT_Q10 as u64 / 1024) as u32;
                let mut tau_crosscheck_us = 0u32;
                for (i, &sample) in samples.iter().enumerate() {
                    if sample >= rise_threshold {
                        tau_crosscheck_us = i as u32 * ELEC_SAMPLE_PERIOD_US;
                        break;
                    }
                }

                // Default theta_e to at least one sample period if zero.
                if theta_e_us == 0 {
                    theta_e_us = ELEC_SAMPLE_PERIOD_US;
                }

                // --- Assemble ElectricalModel ---
                let model = ElectricalModel {
                    r_int,
                    l_int,
                    tau_e_us,
                    theta_e_us,
                    r_mohm,
                    l_uh: l_uh as u32,
                    tau_e_crosscheck_us: tau_crosscheck_us,
                };

                // Transition to CurrentTune carrying the model.
                *phase = InnerPhase::CurrentTune { model };
                StepResult::PhaseChanged(PhaseId::CurrentTune)
            }
        }
    }

    // -------------------------------------------------------------------
    // Phase 6: Current-Loop SIMC Synthesis
    // -------------------------------------------------------------------

    /// Thin phase: read the electrical model, compute SIMC gains, and
    /// transition immediately to validation.
    async fn step_current_tune<B: CommissioningBackend>(
        phase: &mut InnerPhase,
        profile: CommissionProfile,
        backend: &mut B,
    ) -> StepResult {
        let model = match phase {
            InnerPhase::CurrentTune { model } => *model,
            _ => return StepResult::Continue,
        };

        let config = backend.tmc_config();
        let gains = foci_core::autotune::simc_current_gains(&model, profile, config);

        // Compute step amplitude: 15% of run_current.
        let current_limit = backend.current_limit() & 0xFFFF;
        let step_amplitude = if current_limit > 0 {
            (current_limit * VALIDATION_STEP_PERCENT / 100).max(1) as u16
        } else {
            // No current limit configured — cannot validate.
            return Self::fail_inner(phase, CommissionError::CurrentValidationFailed);
        };

        *phase = InnerPhase::CurrentValidate {
            state: CurrentValidateState::WriteGains,
            data: CurrentValidateData {
                model,
                gains,
                attempt: 0,
                step_amplitude,
                forward_metrics: None,
                reverse_metrics: None,
                final_status: CommissionStatus::Accepted,
                saturation_count: 0,
                pidout_limit: {
                    // Read PIDOUT_UQ_UD_LIMITS for saturation detection.
                    backend
                        .read_register(tmc4671::registers::PIDOUT_UQ_UD_LIMITS)
                        .await
                        .unwrap_or(0)
                        & 0xFFFF
                },
            },
        };
        StepResult::PhaseChanged(PhaseId::CurrentValidate)
    }

    // -------------------------------------------------------------------
    // Phase 7: Current-Loop Validation
    // -------------------------------------------------------------------

    /// Advance current-loop validation sub-state.
    ///
    /// Writes SIMC gains, runs bidirectional step-response captures,
    /// classifies the response, retries once if marginal, and falls
    /// back to conservative defaults on failure.
    async fn step_current_validate<B: CommissioningBackend>(
        phase: &mut InnerPhase,
        buf: &mut TransientBuffer,
        profile: CommissionProfile,
        backend: &mut B,
    ) -> StepResult {
        use tmc4671::registers::*;

        let (validate_state, data) = match phase {
            InnerPhase::CurrentValidate { state, data } => (state, data),
            _ => return StepResult::Continue,
        };

        match validate_state {
            CurrentValidateState::WriteGains => {
                // Write gains to TMC4671 PI registers.
                // PID_FLUX_P_FLUX_I: P in [31:16], I in [15:0]
                let flux_reg = ((data.gains.flux_p as u32) << 16) | data.gains.flux_i as u32;
                let torque_reg = ((data.gains.torque_p as u32) << 16) | data.gains.torque_i as u32;

                if backend
                    .write_register(PID_FLUX_P_FLUX_I, flux_reg)
                    .await
                    .is_err()
                    || backend
                        .write_register(PID_TORQUE_P_TORQUE_I, torque_reg)
                        .await
                        .is_err()
                {
                    return Self::fail_inner(phase, CommissionError::Spi);
                }

                // Clear any previous metrics for this attempt.
                data.forward_metrics = None;
                data.reverse_metrics = None;
                data.saturation_count = 0;

                *validate_state = CurrentValidateState::StepSetup {
                    direction: CaptureDirection::Forward,
                };
                StepResult::Continue
            }

            CurrentValidateState::StepSetup { direction } => {
                let dir = *direction;
                // Switch to torque mode (MODE=1).
                if backend
                    .write_register(MODE_RAMP_MODE_MOTION, 1)
                    .await
                    .is_err()
                {
                    return Self::fail_inner(phase, CommissionError::Spi);
                }
                backend.set_enable_pin(true);

                // Apply current step via PID_TORQUE_FLUX_TARGET.
                // Torque target in [31:16], flux target in [15:0].
                let torque_val = match dir {
                    CaptureDirection::Forward => data.step_amplitude as u32,
                    CaptureDirection::Reverse => (-(data.step_amplitude as i16) as u16) as u32,
                };
                let target = torque_val << 16;
                if backend
                    .write_register(PID_TORQUE_FLUX_TARGET, target)
                    .await
                    .is_err()
                {
                    return Self::fail_inner(phase, CommissionError::Spi);
                }

                // Reset buffer for this capture.
                buf.reset(ELEC_SAMPLE_PERIOD_US as u16);

                let next = backend.elapsed_us().wrapping_add(ELEC_SAMPLE_PERIOD_US);
                *validate_state = CurrentValidateState::StepCapture {
                    direction: dir,
                    next_sample_us: next,
                };
                StepResult::Continue
            }

            CurrentValidateState::StepSettle {
                direction,
                deadline_us,
            } => {
                if backend.elapsed_us().wrapping_sub(*deadline_us) >= 0x8000_0000 {
                    return StepResult::Continue;
                }
                // Settling done. Continue based on direction.
                let dir = *direction;
                match dir {
                    CaptureDirection::Forward => {
                        // Move to reverse capture.
                        *validate_state = CurrentValidateState::StepSetup {
                            direction: CaptureDirection::Reverse,
                        };
                    }
                    CaptureDirection::Reverse => {
                        // Both captures done. Classify.
                        *validate_state = CurrentValidateState::Classify;
                    }
                }
                StepResult::Continue
            }

            CurrentValidateState::StepCapture {
                direction,
                next_sample_us,
            } => {
                let dir = *direction;
                // Capture samples at 160us intervals.
                loop {
                    if buf.is_full() || buf.len() >= DELAY_SAMPLE_COUNT {
                        // Capture complete. Extract metrics.
                        // Saturation is counted from the PID output (voltage),
                        // not from measured current, so disable the helper's
                        // sample-level saturation check.
                        let target = data.step_amplitude as u32;
                        let metrics = foci_core::autotune::extract_step_metrics(buf, target, 0);

                        match dir {
                            CaptureDirection::Forward => {
                                data.forward_metrics = Some(metrics);
                            }
                            CaptureDirection::Reverse => {
                                data.reverse_metrics = Some(metrics);
                            }
                        }

                        // Return to zero target.
                        let _ = backend.write_register(PID_TORQUE_FLUX_TARGET, 0).await;
                        let deadline = backend.elapsed_us().wrapping_add(VALIDATION_SETTLE_US);
                        *validate_state = CurrentValidateState::StepSettle {
                            direction: dir,
                            deadline_us: deadline,
                        };
                        return StepResult::Continue;
                    }
                    if backend.elapsed_us().wrapping_sub(*next_sample_us) >= 0x8000_0000 {
                        break; // Not yet time, yield.
                    }
                    match backend.read_register(PID_TORQUE_FLUX_ACTUAL).await {
                        Ok(actual) => {
                            // PID_TORQUE_ACTUAL is in [31:16], PID_FLUX_ACTUAL in [15:0].
                            // We stepped the torque axis, so read the upper half.
                            let torque_actual = ((actual >> 16) & 0xFFFF) as i16;
                            buf.push(torque_actual.unsigned_abs() as u32);
                        }
                        Err(_) => {
                            return Self::fail_inner(phase, CommissionError::Spi);
                        }
                    }
                    // Detect current-PID output saturation: read the limited
                    // FOC voltage output (FOC_UQ_UD_LIMITED) and count when
                    // it's near the PIDOUT_UQ_UD_LIMITS voltage ceiling.
                    if data.pidout_limit > 0 {
                        let sub = tmc4671::registers::INTERIM_FOC_UQ_UD_LIMITED;
                        let _ = backend.write_register(sub.addr, sub.sub_addr as u32).await;
                        if let Ok(foc_limited) = backend.read_register(sub.data).await {
                            // FOC_UQ_UD_LIMITED packs UQ in [31:16], UD in [15:0].
                            // Check the torque-axis voltage (UQ) since we stepped torque.
                            let uq = ((foc_limited >> 16) & 0xFFFF) as i16;
                            let abs_uq = uq.unsigned_abs() as u32;
                            if abs_uq >= data.pidout_limit * 9 / 10 {
                                data.saturation_count += 1;
                            }
                        }
                    }
                    *next_sample_us = next_sample_us.wrapping_add(ELEC_SAMPLE_PERIOD_US);
                }
                StepResult::Continue
            }

            CurrentValidateState::Classify => {
                let forward = match data.forward_metrics {
                    Some(m) => m,
                    None => {
                        return Self::fail_inner(phase, CommissionError::CurrentValidationFailed);
                    }
                };
                let reverse = match data.reverse_metrics {
                    Some(m) => m,
                    None => {
                        return Self::fail_inner(phase, CommissionError::CurrentValidationFailed);
                    }
                };

                // Compute symmetry from forward and reverse rise times.
                let symmetry = foci_core::autotune::compute_symmetry_pct(
                    forward.rise_time_us,
                    reverse.rise_time_us,
                );

                // Use the forward metrics with symmetry filled in for classification.
                let combined = CurrentValidation {
                    symmetry_pct: symmetry,
                    // Use worst-case of forward and reverse for other metrics.
                    overshoot_pct: forward.overshoot_pct.max(reverse.overshoot_pct),
                    ringing_count: forward.ringing_count.max(reverse.ringing_count),
                    settling_time_us: forward.settling_time_us.max(reverse.settling_time_us),
                    // Use firmware-counted PID-output saturation, not the
                    // helper's current-level saturation.
                    saturation_samples: data.saturation_count,
                    rise_time_us: forward.rise_time_us,
                };

                let total_samples = foci_core::autotune::TRANSIENT_LEN as u16 * 2;
                let result = foci_core::autotune::classify_current_validation(
                    &combined,
                    data.model.tau_e_us,
                    total_samples,
                );

                match result {
                    ValidationResult::Pass => {
                        // Gains accepted.
                        data.final_status = if data.attempt > 0 {
                            CommissionStatus::AcceptedWithWarnings
                        } else {
                            CommissionStatus::Accepted
                        };
                    }
                    ValidationResult::Marginal if data.attempt == 0 => {
                        // Retry: increase lambda by 50%, recompute gains.
                        data.attempt = 1;
                        let new_lambda = data.gains.lambda_us * RETRY_LAMBDA_MULTIPLIER_NUM
                            / RETRY_LAMBDA_MULTIPLIER_DEN;

                        // Adjust theta_e_us so the SIMC formula for the
                        // original profile yields the desired larger lambda.
                        //   Conservative: lambda = 4*theta → theta = lambda/4
                        //   Balanced:     lambda = 2*theta → theta = lambda/2
                        //   Stiff:        lambda = max(theta, tau/4) → theta = lambda
                        let mut retry_model = data.model;
                        retry_model.theta_e_us = match profile {
                            CommissionProfile::Conservative => new_lambda / 4,
                            CommissionProfile::Balanced => new_lambda / 2,
                            CommissionProfile::Stiff => new_lambda,
                        };

                        let config = backend.tmc_config();
                        let retry_gains =
                            foci_core::autotune::simc_current_gains(&retry_model, profile, config);
                        data.gains = retry_gains;
                        *validate_state = CurrentValidateState::WriteGains;
                        return StepResult::Continue;
                    }
                    ValidationResult::Marginal | ValidationResult::Fail => {
                        // Fall back to conservative defaults.
                        data.gains = CurrentGains {
                            flux_p: foci_core::pid::DEFAULT_INNER_GAINS.flux_p,
                            flux_i: foci_core::pid::DEFAULT_INNER_GAINS.flux_i,
                            torque_p: foci_core::pid::DEFAULT_INNER_GAINS.torque_p,
                            torque_i: foci_core::pid::DEFAULT_INNER_GAINS.torque_i,
                            effective_bandwidth_hz: 0,
                            lambda_us: 0,
                        };
                        data.final_status = CommissionStatus::AcceptedWithWarnings;

                        // Write the default gains to TMC4671 so outer
                        // commissioning uses the reported conservative gains.
                        let flux_reg =
                            ((data.gains.flux_p as u32) << 16) | data.gains.flux_i as u32;
                        let torque_reg =
                            ((data.gains.torque_p as u32) << 16) | data.gains.torque_i as u32;
                        let _ = backend.write_register(PID_FLUX_P_FLUX_I, flux_reg).await;
                        let _ = backend
                            .write_register(PID_TORQUE_P_TORQUE_I, torque_reg)
                            .await;
                    }
                }

                // Restore safe state.
                let _ = backend.write_register(PID_TORQUE_FLUX_TARGET, 0).await;
                let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;
                backend.set_enable_pin(false);

                let deadline = backend.elapsed_us().wrapping_add(VALIDATION_SETTLE_US);
                *validate_state = CurrentValidateState::SafeState {
                    deadline_us: deadline,
                };
                StepResult::Continue
            }

            CurrentValidateState::SafeState { deadline_us } => {
                if backend.elapsed_us().wrapping_sub(*deadline_us) >= 0x8000_0000 {
                    return StepResult::Continue;
                }
                *validate_state = CurrentValidateState::Finalize;
                StepResult::Continue
            }

            CurrentValidateState::Finalize => {
                let result = InnerResult {
                    status: data.final_status,
                    r_mohm: data.model.r_mohm,
                    l_uh: data.model.l_uh,
                    flux_p: data.gains.flux_p,
                    flux_i: data.gains.flux_i,
                    torque_p: data.gains.torque_p,
                    torque_i: data.gains.torque_i,
                };
                *phase = InnerPhase::Done(result.clone());
                StepResult::InnerDone(result)
            }
        }
    }

    // -------------------------------------------------------------------
    // Helpers
    // -------------------------------------------------------------------

    /// Transition inner phase to Failed and restore safe state.
    fn fail_inner(phase: &mut InnerPhase, err: CommissionError) -> StepResult {
        *phase = InnerPhase::Failed(err);
        StepResult::InnerFailed(err)
    }

    // -------------------------------------------------------------------
    // Outer phase dispatch and safety
    // -------------------------------------------------------------------

    /// Step outer phase state machine.
    ///
    /// Dispatches to per-phase handlers. Every tick runs the safety
    /// envelope check before the phase-specific logic.
    #[allow(clippy::too_many_arguments)]
    async fn step_outer<B: CommissioningBackend>(
        phase: &mut OuterPhase,
        buf: &mut TransientBuffer,
        _profile: CommissionProfile,
        _mode: OperatingMode,
        backend: &mut B,
    ) -> StepResult {
        // Run per-tick safety envelope for all active phases.
        if let Some(safety) = Self::outer_safety_mut(phase) {
            if Self::check_outer_safety(safety, backend).await {
                // Zero excitation immediately on any safety violation.
                let _ = backend
                    .write_register(tmc4671::registers::PID_TORQUE_FLUX_TARGET, 0)
                    .await;
                let _ = backend
                    .write_register(tmc4671::registers::MODE_RAMP_MODE_MOTION, 0)
                    .await;
                let err = Self::safety_error_for_phase(phase);
                return Self::fail_outer(phase, err);
            }
        }

        match phase {
            OuterPhase::MechanicalId { .. } => Self::step_mechanical_id(phase, buf, backend).await,
            OuterPhase::VelocityTune { .. }
            | OuterPhase::VelocityValidate { .. }
            | OuterPhase::PositionTune { .. }
            | OuterPhase::FilterTune { .. }
            | OuterPhase::Commit => {
                // Placeholder for later phases — not in scope for this plan.
                StepResult::Continue
            }
            OuterPhase::Done(result) => StepResult::OuterDone(result.clone()),
            OuterPhase::Failed(err) => StepResult::OuterFailed(*err),
        }
    }

    /// Get a mutable reference to the safety limits for the current phase.
    ///
    /// Returns `None` for terminal phases (Done/Failed/Commit) that don't
    /// need per-tick safety checks.
    fn outer_safety_mut(phase: &mut OuterPhase) -> Option<&mut OuterSafetyLimits> {
        match phase {
            OuterPhase::MechanicalId { safety, .. }
            | OuterPhase::VelocityTune { safety, .. }
            | OuterPhase::VelocityValidate { safety, .. }
            | OuterPhase::PositionTune { safety, .. }
            | OuterPhase::FilterTune { safety, .. } => Some(safety),
            OuterPhase::Commit | OuterPhase::Done(_) | OuterPhase::Failed(_) => None,
        }
    }

    /// Per-tick safety envelope check for all outer phases.
    ///
    /// Reads the current encoder position, estimates velocity from the
    /// delta since the last tick, and checks:
    /// - Velocity within cap
    /// - Position within window from start
    ///
    /// Returns `true` if a safety limit is violated.
    async fn check_outer_safety<B: CommissioningBackend>(
        safety: &mut OuterSafetyLimits,
        backend: &mut B,
    ) -> bool {
        let encoder_raw = backend.read_encoder_count().await;
        let encoder_count = encoder_raw as i32;

        // Velocity estimate: delta encoder counts per 1ms tick.
        let velocity = encoder_count.wrapping_sub(safety.prev_encoder_count);
        safety.prev_encoder_count = encoder_count;

        // Check velocity cap (absolute value).
        let abs_velocity = if velocity < 0 { -velocity } else { velocity };
        if abs_velocity > safety.velocity_cap {
            defmt::warn!(
                "outer safety: velocity {} exceeds cap {}",
                abs_velocity,
                safety.velocity_cap
            );
            return true;
        }

        // Check position window.
        let position_delta = encoder_count.wrapping_sub(safety.start_position);
        let abs_position = if position_delta < 0 {
            -position_delta
        } else {
            position_delta
        };
        if abs_position > safety.position_window {
            defmt::warn!(
                "outer safety: position {} exceeds window {}",
                abs_position,
                safety.position_window
            );
            return true;
        }

        false
    }

    /// Map a safety violation to the appropriate phase-specific error.
    fn safety_error_for_phase(phase: &OuterPhase) -> CommissionError {
        match phase {
            OuterPhase::MechanicalId { .. } => CommissionError::MechanicalIdFailed,
            OuterPhase::VelocityTune { .. } | OuterPhase::VelocityValidate { .. } => {
                CommissionError::VelocityValidationFailed
            }
            OuterPhase::PositionTune { .. } | OuterPhase::FilterTune { .. } => {
                CommissionError::PositionTuneFailed
            }
            OuterPhase::Commit { .. } | OuterPhase::Done(_) | OuterPhase::Failed(_) => {
                CommissionError::Shutdown // Terminal phases; safety shouldn't fire here.
            }
        }
    }

    /// Transition outer phase to Failed.
    fn fail_outer(phase: &mut OuterPhase, err: CommissionError) -> StepResult {
        *phase = OuterPhase::Failed(err);
        StepResult::OuterFailed(err)
    }

    // -------------------------------------------------------------------
    // Phase 8: Mechanical Identification
    // -------------------------------------------------------------------

    /// Advance mechanical identification sub-state.
    ///
    /// Implements the spec's acceleration test procedure:
    /// 1. Switch to torque mode with tuned current-loop gains active
    /// 2. Record starting position
    /// 3. Apply torque step, sample encoder at 1 kHz
    /// 4. Return to zero torque, wait for settling
    /// 5. Repeat in opposite direction
    /// 6. Estimate J_eff, B_eff, tau_mech, theta_mech
    /// 7. Validate gates and transition
    async fn step_mechanical_id<B: CommissioningBackend>(
        phase: &mut OuterPhase,
        buf: &mut TransientBuffer,
        backend: &mut B,
    ) -> StepResult {
        use tmc4671::registers::*;

        let (state, data, safety) = match phase {
            OuterPhase::MechanicalId {
                state,
                data,
                safety,
            } => (state, data, safety),
            _ => return StepResult::Continue,
        };

        match state {
            MechanicalIdState::Setup => {
                // Read current encoder position as the reference point.
                let encoder_raw = backend.read_encoder_count().await;
                let start_position = encoder_raw as i32;

                // Compute torque step from configured run_current.
                let current_limit = backend.current_limit() & 0xFFFF;
                let torque_step = if current_limit > 0 {
                    (current_limit * MECH_ID_TORQUE_STEP_PERCENT / 100).max(1) as u16
                } else {
                    return Self::fail_outer(phase, CommissionError::NoCurrent);
                };

                // Initialize safety limits with real values.
                safety.start_position = start_position;
                safety.prev_encoder_count = start_position;
                safety.torque_cap = current_limit as u16;

                // Initialize data with real values.
                data.start_position = start_position;
                data.torque_step = torque_step;

                // Ensure we are in torque mode (MODE=1) with current-loop
                // gains already written by inner commissioning.
                if backend
                    .write_register(MODE_RAMP_MODE_MOTION, 1)
                    .await
                    .is_err()
                {
                    return Self::fail_outer(phase, CommissionError::Spi);
                }

                // Zero torque target before starting capture.
                let _ = backend.write_register(PID_TORQUE_FLUX_TARGET, 0).await;

                // Prepare transient buffer for forward capture.
                buf.reset(MECH_ID_SAMPLE_PERIOD_US as u16);

                defmt::info!(
                    "mech-id: start pos={}, torque_step={}, current_limit={}",
                    start_position,
                    torque_step,
                    current_limit
                );

                *state = MechanicalIdState::ForwardStart;
                StepResult::PhaseChanged(PhaseId::MechanicalId)
            }

            MechanicalIdState::ForwardStart => {
                // Apply positive torque step.
                let torque = data.torque_step as u32;
                // PID_TORQUE_FLUX_TARGET: torque in [31:16], flux=0 in [15:0].
                let target = torque << 16;
                if backend
                    .write_register(PID_TORQUE_FLUX_TARGET, target)
                    .await
                    .is_err()
                {
                    return Self::fail_outer(phase, CommissionError::Spi);
                }

                let next_sample_us = backend.elapsed_us().wrapping_add(MECH_ID_SAMPLE_PERIOD_US);
                *state = MechanicalIdState::ForwardCapture { next_sample_us };
                StepResult::Continue
            }

            MechanicalIdState::ForwardCapture { next_sample_us } => {
                // Check if it's time for the next sample.
                let now = backend.elapsed_us();
                if now.wrapping_sub(*next_sample_us) >= 0x8000_0000 {
                    // Not yet time for next sample.
                    return StepResult::Continue;
                }

                // Sample encoder position.
                let encoder_raw = backend.read_encoder_count().await;
                buf.push(encoder_raw);

                if buf.is_full() {
                    // Forward capture complete — zero torque.
                    let _ = backend.write_register(PID_TORQUE_FLUX_TARGET, 0).await;

                    // Extract forward metrics from buffer before reset.
                    let (initial_accel, peak_velocity, delay_samples) =
                        Self::extract_mech_metrics(buf, data.start_position);
                    data.forward_initial_accel = initial_accel;
                    data.forward_peak_velocity = peak_velocity;
                    data.forward_delay_samples = delay_samples;

                    defmt::info!(
                        "mech-id fwd: accel={}, peak_vel={}, delay={}",
                        initial_accel,
                        peak_velocity,
                        delay_samples
                    );

                    let deadline = backend.elapsed_us().wrapping_add(MECH_ID_SETTLE_US);
                    *state = MechanicalIdState::ForwardSettle {
                        deadline_us: deadline,
                    };
                } else {
                    *next_sample_us = now.wrapping_add(MECH_ID_SAMPLE_PERIOD_US);
                }
                StepResult::Continue
            }

            MechanicalIdState::ForwardSettle { deadline_us } => {
                if backend.elapsed_us().wrapping_sub(*deadline_us) < 0x8000_0000 {
                    // Settled — prepare for reverse capture.
                    buf.reset(MECH_ID_SAMPLE_PERIOD_US as u16);
                    *state = MechanicalIdState::ReverseStart;
                }
                StepResult::Continue
            }

            MechanicalIdState::ReverseStart => {
                // Apply negative torque step.
                // Torque is signed i16 in upper 16 bits.
                let neg_torque = (-(data.torque_step as i16)) as u16;
                let target = (neg_torque as u32) << 16;
                if backend
                    .write_register(PID_TORQUE_FLUX_TARGET, target)
                    .await
                    .is_err()
                {
                    return Self::fail_outer(phase, CommissionError::Spi);
                }

                let next_sample_us = backend.elapsed_us().wrapping_add(MECH_ID_SAMPLE_PERIOD_US);
                *state = MechanicalIdState::ReverseCapture { next_sample_us };
                StepResult::Continue
            }

            MechanicalIdState::ReverseCapture { next_sample_us } => {
                let now = backend.elapsed_us();
                if now.wrapping_sub(*next_sample_us) >= 0x8000_0000 {
                    return StepResult::Continue;
                }

                let encoder_raw = backend.read_encoder_count().await;
                buf.push(encoder_raw);

                if buf.is_full() {
                    // Reverse capture complete — zero torque.
                    let _ = backend.write_register(PID_TORQUE_FLUX_TARGET, 0).await;

                    // For reverse capture, use the position at capture start
                    // (first sample) as the reference, not the original start.
                    let reverse_start = if buf.len() > 0 {
                        buf.samples()[0] as i32
                    } else {
                        data.start_position
                    };
                    let (initial_accel, peak_velocity, delay_samples) =
                        Self::extract_mech_metrics(buf, reverse_start);
                    // Store as absolute values for symmetry comparison.
                    data.reverse_initial_accel = if initial_accel < 0 {
                        -initial_accel
                    } else {
                        initial_accel
                    };
                    data.reverse_peak_velocity = if peak_velocity < 0 {
                        -peak_velocity
                    } else {
                        peak_velocity
                    };
                    data.reverse_delay_samples = delay_samples;

                    defmt::info!(
                        "mech-id rev: accel={}, peak_vel={}, delay={}",
                        data.reverse_initial_accel,
                        data.reverse_peak_velocity,
                        delay_samples
                    );

                    let deadline = backend.elapsed_us().wrapping_add(MECH_ID_SETTLE_US);
                    *state = MechanicalIdState::ReverseSettle {
                        deadline_us: deadline,
                    };
                } else {
                    *next_sample_us = now.wrapping_add(MECH_ID_SAMPLE_PERIOD_US);
                }
                StepResult::Continue
            }

            MechanicalIdState::ReverseSettle { deadline_us } => {
                if backend.elapsed_us().wrapping_sub(*deadline_us) < 0x8000_0000 {
                    *state = MechanicalIdState::Estimate;
                }
                StepResult::Continue
            }

            MechanicalIdState::Estimate => {
                // Ensure motor is stopped.
                let _ = backend.write_register(PID_TORQUE_FLUX_TARGET, 0).await;
                let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;

                *state = MechanicalIdState::Finalize;
                StepResult::Continue
            }

            MechanicalIdState::Finalize => {
                // Assemble and validate mechanical model.
                let torque = data.torque_step as u32;

                // Forward metrics (already absolute for accel).
                let fwd_accel = if data.forward_initial_accel < 0 {
                    (-data.forward_initial_accel) as u32
                } else {
                    data.forward_initial_accel as u32
                };
                let rev_accel = data.reverse_initial_accel as u32;

                // Gate: both accelerations must be positive and non-zero.
                if fwd_accel == 0 || rev_accel == 0 {
                    defmt::warn!("mech-id: zero acceleration detected");
                    return Self::fail_outer(phase, CommissionError::MechanicalIdFailed);
                }

                // J_eff = torque / initial_acceleration.
                // Scale: torque is in DAC units, accel is in counts/ms^2.
                // We multiply torque by 1000 to get better fixed-point resolution.
                let j_eff_fwd = (torque * 1000) / fwd_accel;
                let j_eff_rev = (torque * 1000) / rev_accel;

                // Symmetry gate: |J_fwd - J_rev| / avg <= 30%.
                let j_avg = (j_eff_fwd + j_eff_rev) / 2;
                if j_avg == 0 {
                    defmt::warn!("mech-id: zero average inertia");
                    return Self::fail_outer(phase, CommissionError::MechanicalIdFailed);
                }
                let j_diff = if j_eff_fwd > j_eff_rev {
                    j_eff_fwd - j_eff_rev
                } else {
                    j_eff_rev - j_eff_fwd
                };
                let asymmetry_pct = (j_diff * 100) / j_avg;
                if asymmetry_pct > MECH_ID_SYMMETRY_THRESHOLD_PCT {
                    defmt::warn!(
                        "mech-id: asymmetry {}% exceeds {}%",
                        asymmetry_pct,
                        MECH_ID_SYMMETRY_THRESHOLD_PCT
                    );
                    return Self::fail_outer(phase, CommissionError::MechanicalIdFailed);
                }

                let j_eff = j_avg;

                // B_eff from peak velocity (steady-state approximation).
                let fwd_peak = if data.forward_peak_velocity < 0 {
                    (-data.forward_peak_velocity) as u32
                } else {
                    data.forward_peak_velocity as u32
                };
                let rev_peak = data.reverse_peak_velocity as u32;
                let peak_avg = (fwd_peak + rev_peak) / 2;

                // B_eff = torque / steady_state_velocity.
                // Scale to match J_eff units.
                let b_eff = if peak_avg > 0 {
                    (torque * 1000) / peak_avg
                } else {
                    // No steady-state velocity reached — pure inertial load.
                    // Use a small minimum to avoid division by zero in tau.
                    1
                };

                // tau_mech = J_eff / B_eff in milliseconds.
                let tau_mech_ms = if b_eff > 0 { j_eff / b_eff } else { 1000 };

                // theta_mech from delay samples (time from torque step to
                // first measurable velocity change).
                let avg_delay_samples =
                    (data.forward_delay_samples + data.reverse_delay_samples) / 2;
                let theta_mech_us = avg_delay_samples as u32 * MECH_ID_SAMPLE_PERIOD_US;

                let model = MechanicalModel {
                    j_eff,
                    b_eff,
                    tau_mech_ms,
                    theta_mech_us,
                    t_f: 0, // Static friction not measured in this phase.
                };

                defmt::info!(
                    "mech-id: J={}, B={}, tau={}ms, theta={}us",
                    model.j_eff,
                    model.b_eff,
                    model.tau_mech_ms,
                    model.theta_mech_us
                );

                // Carry safety limits forward to VelocityTune.
                let carried_safety = match phase {
                    OuterPhase::MechanicalId { safety, .. } => *safety,
                    _ => unreachable!(),
                };

                *phase = OuterPhase::VelocityTune {
                    model,
                    safety: carried_safety,
                };
                StepResult::PhaseChanged(PhaseId::VelocityTune)
            }
        }
    }

    /// Extract mechanical metrics from a position transient buffer.
    ///
    /// Computes velocity as first-order differences of position samples,
    /// then derives:
    /// - Initial acceleration (from first `MECH_ID_ACCEL_SAMPLES` velocities)
    /// - Peak velocity (maximum absolute velocity in the capture)
    /// - Delay samples (number of samples before first velocity onset)
    ///
    /// # Parameters
    /// - `buf`: Transient buffer filled with encoder position samples
    /// - `reference_position`: Starting position for velocity computation
    fn extract_mech_metrics(buf: &TransientBuffer, reference_position: i32) -> (i32, i32, u16) {
        let samples = buf.samples();
        if samples.len() < 2 {
            return (0, 0, 0);
        }

        // Compute velocity as position differences (counts per sample period).
        // First velocity is relative to the reference position.
        let first_velocity = (samples[0] as i32).wrapping_sub(reference_position);

        let mut peak_velocity: i32 = first_velocity;
        let mut delay_samples: u16 = 0;
        let mut found_onset = false;

        // Track velocity onset.
        let abs_first = if first_velocity < 0 {
            -first_velocity
        } else {
            first_velocity
        };
        if abs_first >= MECH_ID_VELOCITY_ONSET_THRESHOLD {
            found_onset = true;
        }

        // Accumulate acceleration from early samples.
        let accel_limit = MECH_ID_ACCEL_SAMPLES.min(samples.len());
        let mut accel_sum: i32 = 0;
        let mut accel_count: u32 = 0;

        // First acceleration point.
        if accel_limit >= 2 {
            let vel0 = first_velocity;
            let vel1 = (samples[1] as i32).wrapping_sub(samples[0] as i32);
            accel_sum += vel1 - vel0;
            accel_count += 1;
        }

        let mut prev_velocity = first_velocity;

        for i in 1..samples.len() {
            let velocity = (samples[i] as i32).wrapping_sub(samples[i - 1] as i32);

            // Track velocity onset.
            if !found_onset {
                let abs_vel = if velocity < 0 { -velocity } else { velocity };
                if abs_vel >= MECH_ID_VELOCITY_ONSET_THRESHOLD {
                    found_onset = true;
                    delay_samples = i as u16;
                } else {
                    delay_samples = i as u16;
                }
            }

            // Peak velocity (signed, tracking the direction of motion).
            let abs_cur = if velocity < 0 { -velocity } else { velocity };
            let abs_peak = if peak_velocity < 0 {
                -peak_velocity
            } else {
                peak_velocity
            };
            if abs_cur > abs_peak {
                peak_velocity = velocity;
            }

            // Acceleration from early samples.
            if i < accel_limit && i >= 2 {
                let accel = velocity - prev_velocity;
                accel_sum += accel;
                accel_count += 1;
            }

            prev_velocity = velocity;
        }

        // Average initial acceleration.
        let initial_accel = if accel_count > 0 {
            accel_sum / accel_count as i32
        } else {
            first_velocity // Fallback: use first velocity as proxy.
        };

        // If onset was never found, the motor didn't move.
        if !found_onset {
            delay_samples = samples.len() as u16;
        }

        (initial_accel, peak_velocity, delay_samples)
    }
}
