//! Commissioning state machine and hardware backend.
//!
//! The [`CommissioningEngine`] is a cooperative state machine invoked
//! every 1ms by the `tmc_control` task. Each `step()` call advances
//! the current phase by one chunk of work (max 1ms runtime).
//!
//! Phase logic bodies are placeholder stubs — filled in by the inner
//! and outer commissioning implementation plans.

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

/// Inner commissioning phase state machine.
///
/// Each variant carries its own local state. Phase logic bodies
/// are stubs -- implemented by the inner commissioning plan.
#[derive(Debug, defmt::Format)]
enum InnerPhase {
    AdcCalibrate,
    CoilCheck,
    PhaseWiringCheck,
    EncoderCheck,
    ElectricalId,
    CurrentTune,
    CurrentValidate { attempt: u8 },
    Done(InnerResult),
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
                phase: InnerPhase::AdcCalibrate,
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
    /// Phase logic bodies are placeholder stubs.
    async fn step_inner<B: CommissioningBackend>(
        _phase: &mut InnerPhase,
        _buf: &mut TransientBuffer,
        _backend: &mut B,
    ) -> StepResult {
        // TODO: dispatch on phase variant, implement phase logic
        // Each phase processes a bounded amount of work per step(),
        // transitions to the next phase when done, or fails.
        //
        // Placeholder: immediately report done with zeros.
        StepResult::InnerDone(InnerResult {
            status: CommissionStatus::Accepted,
            r_mohm: 0,
            l_uh: 0,
            flux_p: 0,
            flux_i: 0,
            torque_p: 0,
            torque_i: 0,
        })
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
