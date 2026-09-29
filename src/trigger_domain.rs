//! Trigger domain: the critical-section-guarded coordination point
//! between the step-queue producer (P2 context), the TIM5 scheduler ISR
//! (P3 context), and the trsync/endstop subsystem.

use crate::state::stepper::isr_stepper;
use foci_firmware::context::trsync::{self, TriggerDispatchBoard, TriggerDomainState};
use foci_firmware::diagnostic::StepperEventReason;
use foci_klipper::step_queue::Consumer;
use foci_klipper::stepper::StepMove;

/// Detach `STEP_QUEUE_CHANNEL`'s consumer. Returns `None` if already detached.
fn detach_consumer() -> Option<Consumer<'static, StepMove>> {
    cortex_m::interrupt::free(|_| {
        // SAFETY: this critical section provides the exclusive access
        // `take_consumer` requires.
        unsafe { crate::state::stepper::STEP_QUEUE_CHANNEL.take_consumer() }
    })
}

fn attach_consumer(consumer: Consumer<'static, StepMove>) {
    cortex_m::interrupt::free(|_| {
        // SAFETY: exclusive access from the critical section; `consumer`
        // is `STEP_QUEUE_CHANNEL`'s own value.
        unsafe { crate::state::stepper::STEP_QUEUE_CHANNEL.give_consumer(consumer) }
    })
}

/// Outcome of `with_consumer_locked`.
pub(crate) enum ConsumerBorrow<R> {
    Borrowed(R),
    /// A masked stop detached the consumer and `trigger_drain_task` has not yet drained it.
    PendingHandoff,
    /// A P1 stop holds the consumer detached while it drains inline.
    DrainInFlight,
}

/// Borrow `STEP_QUEUE_CHANNEL`'s consumer in place.
pub(crate) fn with_consumer_locked<R>(
    f: impl FnOnce(&mut Consumer<'static, StepMove>) -> R,
) -> ConsumerBorrow<R> {
    cortex_m::interrupt::free(|_| {
        // SAFETY: see detach_consumer.
        match unsafe { crate::state::stepper::STEP_QUEUE_CHANNEL.borrow_consumer_mut(f) } {
            Some(result) => ConsumerBorrow::Borrowed(result),
            // SAFETY: see `store_pending_drain`.
            None if unsafe { (*TRIGGER_DRAIN_SLOT.0.get()).is_some() } => {
                ConsumerBorrow::PendingHandoff
            }
            None => ConsumerBorrow::DrainInFlight,
        }
    })
}

/// Physical stepper indices that drive real motors via TIM5.
pub(crate) const PHYSICAL_STEPPER_INDICES: &[usize] = &[0];

/// A physical stepper's detached step queue and stop-time stats, awaiting
/// drain by `trigger_drain_task`.
struct PendingStopDrain {
    consumer: Consumer<'static, StepMove>,
    reason: StepperEventReason,
    remaining_events: u32,
    queue_len: u16,
}

/// Pending drain slot, written by `stop_physical_stepper_masked` and taken
/// by `trigger_drain_task`.
struct TriggerDrainSlot(core::cell::UnsafeCell<Option<PendingStopDrain>>);

// SAFETY: every access goes through `cortex_m::interrupt::free` (see
// `store_pending_drain` and `take_pending_drain`).
unsafe impl Sync for TriggerDrainSlot {}

static TRIGGER_DRAIN_SLOT: TriggerDrainSlot = TriggerDrainSlot(core::cell::UnsafeCell::new(None));

pub(crate) static TRIGGER_DRAIN_WAKE: embassy_sync::signal::Signal<
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    (),
> = embassy_sync::signal::Signal::new();

/// Store a freshly detached consumer and wake the drain task.
///
/// Must be called inside `cortex_m::interrupt::free`.
fn store_pending_drain(drain: PendingStopDrain) {
    // SAFETY: caller holds `cortex_m::interrupt::free`.
    unsafe {
        *TRIGGER_DRAIN_SLOT.0.get() = Some(drain);
    }
    TRIGGER_DRAIN_WAKE.signal(());
}

fn take_pending_drain() -> Option<PendingStopDrain> {
    cortex_m::interrupt::free(|_| {
        // SAFETY: see `store_pending_drain`.
        unsafe { (*TRIGGER_DRAIN_SLOT.0.get()).take() }
    })
}

/// Drain and reattach the pending stop, if any. Called from `trigger_drain_task` (P2).
pub(crate) fn drain_pending_stop() {
    let Some(pending) = take_pending_drain() else {
        return;
    };
    let mut consumer = pending.consumer;
    let mut drained_segments = 0u32;
    let mut drained_steps = 0u32;
    while let Some(m) = consumer.dequeue() {
        drained_segments = drained_segments.saturating_add(1);
        drained_steps = drained_steps.saturating_add(u32::from(m.count));
    }
    crate::state::stepper_diag::record_physical_stop(
        0,
        pending.reason,
        pending.remaining_events,
        pending.queue_len,
        drained_segments,
        drained_steps,
    );
    attach_consumer(consumer);
}

fn tmc_state() -> &'static foci_firmware::state::tmc::TmcChannelState {
    foci_firmware::state::tmc::tmc_channel0()
}

fn stepper_state() -> &'static foci_firmware::state::stepper::StepperChannelState {
    foci_firmware::state::stepper::stepper_channel0()
}

/// Unified resource owning all trsync, endstop, and stepper-stop state.
///
/// Accessed via `with_trigger_domain()`, an interrupt-disabled critical
/// section. Not an RTIC shared resource.
pub(crate) struct TriggerDomain {
    state: TriggerDomainState,
}

impl TriggerDomain {
    pub(crate) const fn new() -> Self {
        Self {
            state: TriggerDomainState::new(),
        }
    }

    pub(crate) fn state_mut(&mut self) -> &mut TriggerDomainState {
        &mut self.state
    }

    pub(crate) fn state(&self) -> &TriggerDomainState {
        &self.state
    }

    /// Attempt to trigger a trsync, dispatching stop signals to all
    /// linked steppers. Returns the trigger result.
    pub(crate) fn trigger(
        &mut self,
        trsync_oid: u8,
        reason: u8,
        clock: u32,
    ) -> foci_klipper::trigger::TriggerResult {
        let mut board = OpenFfboardDispatchBoard;
        trsync::dispatch_trigger(&mut self.state, &mut board, trsync_oid, reason, clock)
    }
}

pub(crate) struct OpenFfboardDispatchBoard;

impl TriggerDispatchBoard for OpenFfboardDispatchBoard {
    fn stop_physical_stepper(
        &mut self,
        _state: &TriggerDomainState,
        stepper_idx: usize,
        reason: StepperEventReason,
    ) {
        stop_physical_stepper_masked(stepper_idx, reason);
    }

    fn record_trsync_stop(&mut self, stepper_idx: usize, clock: u32, trsync_oid: u8, reason: u8) {
        crate::helpers::stepper_diag::record_stepper_event(
            StepperEventReason::TrsyncStop,
            stepper_idx as u8,
            clock,
            0,
            trsync_oid as u32,
            reason as u32,
        );
    }

    fn set_timer_active(
        &mut self,
        state: &mut TriggerDomainState,
        stepper_idx: usize,
        active: bool,
    ) {
        set_timer_active(state, stepper_idx, active);
    }
}

/// Set `timer_active` for a stepper, updating the P1 mirror for physical steppers.
pub(crate) fn set_timer_active(state: &mut TriggerDomainState, stepper_idx: usize, active: bool) {
    state.stepper_stop[stepper_idx].timer_active = active;
    if stepper_idx == 0 {
        stepper_state()
            .timer_active_mirror
            .store(active, core::sync::atomic::Ordering::Relaxed);
    }
}

/// Read `timer_active` for a stepper.
pub(crate) fn timer_active(state: &TriggerDomainState, stepper_idx: usize) -> bool {
    state.stepper_stop[stepper_idx].timer_active
}

/// Arm TIM5 CC1 for the next unified scheduler deadline.
pub(crate) fn arm_timer_at(state: &mut TriggerDomainState, clock: u32) {
    let tim5 = embassy_stm32::pac::TIM5;
    tim5.ccr(0).write_value(clock);
    tim5.sr().modify(|w| {
        w.set_ccif(0, false);
        w.set_ccif(1, false);
    });
    tim5.dier().modify(|w| {
        w.set_ccie(0, true);
        w.set_ccie(1, false);
    });
    unsafe {
        cortex_m::peripheral::NVIC::unmask(embassy_stm32::pac::Interrupt::TIM5);
    }
    state.scheduler.set_cc2_active(true);
}

/// Disable the unified TIM5 scheduler compare.
pub(crate) fn disarm_timer(state: &mut TriggerDomainState) {
    let tim5 = embassy_stm32::pac::TIM5;
    tim5.dier().modify(|w| {
        w.set_ccie(0, false);
        w.set_ccie(1, false);
    });
    tim5.sr().modify(|w| {
        w.set_ccif(0, false);
        w.set_ccif(1, false);
    });
    state.scheduler.set_cc2_active(false);
}

/// Reprogram TIM5 CC1 from the scheduler's earliest deadline, clamping
/// deadlines that are too close to arm.
pub(crate) fn reprogram_timer(state: &mut TriggerDomainState) {
    if let Some(earliest) = state.scheduler.earliest_deadline() {
        let tim5 = embassy_stm32::pac::TIM5;
        let now = tim5.cnt().read();
        let clamped =
            foci_klipper::scheduler::timer_arm_clock(now, earliest, TRIGGER_TIMER_MIN_LEAD_TICKS);
        arm_timer_at(state, clamped);
    } else {
        disarm_timer(state);
    }
}

/// Minimum lead time for TIM5 scheduler arming.
pub(crate) const TRIGGER_TIMER_MIN_LEAD_TICKS: u32 = 128;

pub(crate) static mut TRIGGER_DOMAIN: core::mem::MaybeUninit<TriggerDomain> =
    core::mem::MaybeUninit::uninit();

#[inline]
pub(crate) fn with_trigger_domain<F, R>(f: F) -> R
where
    F: FnOnce(&mut TriggerDomain) -> R,
{
    cortex_m::interrupt::free(|_| {
        // SAFETY: All access to TRIGGER_DOMAIN goes through this function.
        // interrupt::free ensures exclusive access across all priorities.
        unsafe { f(&mut *core::ptr::addr_of_mut!(TRIGGER_DOMAIN).cast::<TriggerDomain>()) }
    })
}

enum QueueAccess {
    /// Detach, drain with interrupts enabled, reattach.
    Unmasked,
    /// Detach and hand off to `trigger_drain_task`.
    LockedInPlace,
}

/// Stop a physical stepper. Only acts on physical stepper indices.
///
/// Does not disable the unified TIM5 scheduler; callers cancel the stepper
/// scheduler event through `TriggerDomain`.
fn stop_physical_stepper_inner(
    stepper_idx: usize,
    reason: StepperEventReason,
    access: QueueAccess,
) {
    use core::sync::atomic::Ordering::Relaxed;
    if !PHYSICAL_STEPPER_INDICES.contains(&stepper_idx) {
        return;
    }
    stepper_state().interval.store(0, Relaxed);
    stepper_state().interval_add.store(0, Relaxed);
    tmc_state().pid_error_sums_clear.store(true, Relaxed);
    let remaining_events = unsafe {
        isr_stepper(0)
            .expect("OpenFFBoard channel 0 exists")
            .active_step_count()
    };
    let queue_len = crate::state::stepper::STEP_QUEUE_CHANNEL
        .len()
        .min(u16::MAX as usize) as u16;
    // SAFETY: Triggered stops call this inside `with_trigger_domain()` with
    // interrupts disabled. P1 stops clear `motor_armed` before calling this,
    // and the unified ISR checks that gate before touching stepper state.
    unsafe {
        isr_stepper(0)
            .expect("OpenFFBoard channel 0 exists")
            .reset();
    }
    let mut drained_segments = 0u32;
    let mut drained_steps = 0u32;
    let mut deferred_record = false;
    match access {
        QueueAccess::Unmasked => {
            if let Some(mut consumer) = detach_consumer() {
                while let Some(m) = consumer.dequeue() {
                    drained_segments = drained_segments.saturating_add(1);
                    drained_steps = drained_steps.saturating_add(u32::from(m.count));
                }
                attach_consumer(consumer);
            }
        }
        QueueAccess::LockedInPlace => {
            if let Some(consumer) = detach_consumer() {
                store_pending_drain(PendingStopDrain {
                    consumer,
                    reason,
                    remaining_events: u32::from(remaining_events),
                    queue_len,
                });
                deferred_record = true;
            }
        }
    }
    if !deferred_record {
        crate::state::stepper_diag::record_physical_stop(
            stepper_idx as u8,
            reason,
            u32::from(remaining_events),
            queue_len,
            drained_segments,
            drained_steps,
        );
    }
    let gpio_a = embassy_stm32::pac::GPIOA;
    gpio_a
        .odr()
        .modify(|w| w.set_odr(0, embassy_stm32::pac::gpio::vals::Odr::LOW));
    stepper_state().direction.store(true, Relaxed);
    // TMC4671 counts the STEP rising edge; a pin left HIGH by a partial
    // pulse would swallow the next segment's first step.
    let gpio_b = embassy_stm32::pac::GPIOB;
    gpio_b
        .odr()
        .modify(|w| w.set_odr(11, embassy_stm32::pac::gpio::vals::Odr::LOW));
}

/// Stop a physical stepper from the P1 direct-stop path, with no critical
/// section held.
pub(crate) fn stop_physical_stepper_unmasked(stepper_idx: usize, reason: StepperEventReason) {
    stop_physical_stepper_inner(stepper_idx, reason, QueueAccess::Unmasked);
}

/// Stop a physical stepper from inside an active `with_trigger_domain`
/// closure; the queue drain is handed off to `trigger_drain_task`.
pub(crate) fn stop_physical_stepper_masked(stepper_idx: usize, reason: StepperEventReason) {
    stop_physical_stepper_inner(stepper_idx, reason, QueueAccess::LockedInPlace);
}
