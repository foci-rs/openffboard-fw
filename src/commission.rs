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
    CommissionError, CommissionProfile, CommissionStatus, OperatingMode, PhaseId, TransientBuffer,
};
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
    ElectricalId,
    /// Phase 6: Current-loop SIMC gain synthesis.
    CurrentTune,
    /// Phase 7: Current-loop step-response validation.
    CurrentValidate { attempt: u8 },
    /// All inner phases completed successfully.
    Done(InnerResult),
    /// A phase failed.
    Failed(CommissionError),
}

/// Outer commissioning phase state machine.
#[derive(Debug, defmt::Format)]
enum OuterPhase {
    MechanicalId,
    VelocityTune,
    VelocityValidate { attempt: u8 },
    PositionTune,
    FilterTune,
    Commit,
    Done(OuterResult),
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
        Self {
            oid,
            engine_mode: EngineMode::Outer {
                phase: OuterPhase::MechanicalId,
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
                profile: _,
            } => Self::step_inner(phase, &mut self.buf, backend).await,
            EngineMode::Outer {
                ref mut phase,
                profile: _,
                mode: _,
            } => Self::step_outer(phase, &mut self.buf, backend).await,
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
        _buf: &mut TransientBuffer,
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
            InnerPhase::ElectricalId => {
                // Placeholder — filled by electrical-ID plan.
                StepResult::Continue
            }
            InnerPhase::CurrentTune => {
                // Placeholder — filled by current-tune plan.
                StepResult::Continue
            }
            InnerPhase::CurrentValidate { .. } => {
                // Placeholder — filled by current-validation plan.
                StepResult::Continue
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
                *phase = InnerPhase::ElectricalId;
                StepResult::PhaseChanged(PhaseId::ElectricalId)
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

    /// Step outer phase state machine.
    ///
    /// Phase logic bodies are placeholder stubs.
    async fn step_outer<B: CommissioningBackend>(
        _phase: &mut OuterPhase,
        _buf: &mut TransientBuffer,
        _backend: &mut B,
    ) -> StepResult {
        // TODO: dispatch on phase variant, implement phase logic
        StepResult::OuterDone(OuterResult {
            status: CommissionStatus::Accepted,
            velocity_p: 0,
            velocity_i: 0,
            velocity_limit: 0,
            position_p: 0,
            position_i: 0,
            velocity_filter_hz: 0,
            position_filter_hz: 0,
            flux_filter_hz: 0,
            torque_filter_hz: 0,
        })
    }
}
