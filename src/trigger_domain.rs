//! Trigger domain: the critical-section-guarded coordination point
//! between the step-queue producer (P2 context), the TIM5 scheduler ISR
//! (P3 context), and the trsync/endstop subsystem.
//!
//! Lives outside `mod app`, alongside the subsystem `state/*` modules, so
//! the coordination surface is inspectable independently of RTIC's macro
//! expansion.

use core::ops::{Deref, DerefMut};

use crate::state::stepper::isr_stepper;
use foci_firmware::context::trsync::{self, TriggerDispatchBoard, TriggerDomainState};
use foci_firmware::diagnostic::StepperEventReason;
use foci_klipper::step_queue::Consumer;
use foci_klipper::stepper::StepMove;

/// Detach `STEP_QUEUE_CHANNEL`'s consumer under a short, independent
/// critical section, for a caller about to drain it outside any masked
/// region. Returns `None` if already detached.
fn detach_consumer() -> Option<Consumer<'static, StepMove>> {
    cortex_m::interrupt::free(|_| {
        // SAFETY: this critical section provides the exclusive access
        // `take_consumer` requires. It is independent of
        // `with_trigger_domain` (never touches `TriggerDomain`), so it is
        // safe to nest inside an outer `interrupt::free` region if the
        // caller happens to be in one.
        unsafe { crate::state::stepper::STEP_QUEUE_CHANNEL.take_consumer() }
    })
}

/// Reattach a consumer previously detached from `STEP_QUEUE_CHANNEL` by
/// `detach_consumer`. Private: the only correct caller is the function
/// that just called `detach_consumer` and is handing its own local
/// variable straight back.
fn attach_consumer(consumer: Consumer<'static, StepMove>) {
    cortex_m::interrupt::free(|_| {
        // SAFETY: exclusive access from the critical section; `consumer`
        // is `STEP_QUEUE_CHANNEL`'s own value per this function's contract
        // (OpenFFBoard has exactly one channel, so there is no other
        // channel's consumer this could be).
        unsafe { crate::state::stepper::STEP_QUEUE_CHANNEL.give_consumer(consumer) }
    })
}

/// Borrow `STEP_QUEUE_CHANNEL`'s consumer in place, under a short
/// independent critical section, for a caller already inside an outer
/// mask that stays in place here (the stepper-reset command path is the
/// only remaining caller). Returns `None` if a concurrent unmasked drain
/// (`stop_physical_stepper_unmasked`) currently holds the consumer
/// detached, or if a pending hand-off from `stop_physical_stepper_masked`
/// is sitting in `TRIGGER_DRAIN_SLOT`, detached but not yet taken by
/// `trigger_drain_task` (reachable since this function's caller and
/// `trigger_drain_task` both run at P2) -- callers must perform their
/// non-queue side effects and report zero *additional* drained work in
/// either case: whichever in-flight drain currently owns the consumer
/// already owns draining this queue.
pub(crate) fn with_consumer_locked<R>(
    f: impl FnOnce(&mut Consumer<'static, StepMove>) -> R,
) -> Option<R> {
    cortex_m::interrupt::free(|_| {
        // SAFETY: see detach_consumer.
        unsafe { crate::state::stepper::STEP_QUEUE_CHANNEL.borrow_consumer_mut(f) }
    })
}

// ---- TriggerDomain: unified trsync/endstop/stepper-stop resource ----

/// Physical stepper indices that drive real motors via TIM5.
/// OpenFFBoard has one physical stepper (index 0).
pub(crate) const PHYSICAL_STEPPER_INDICES: &[usize] = &[0];

/// A physical stepper's detached step queue, captured at stop time,
/// awaiting drain by `trigger_drain_task`. Everything `record_physical_stop`
/// needs is captured here because it is only known at stop time (before the
/// drain), not at drain time.
struct PendingStopDrain {
    consumer: Consumer<'static, StepMove>,
    reason: StepperEventReason,
    remaining_events: u32,
    queue_len: u16,
}

/// The single physical stepper channel's pending drain slot, written by
/// `stop_physical_stepper` (masked, O(1)) and taken by `trigger_drain_task`
/// (its own short masked window, never across the drain loop).
///
/// `UnsafeCell` is `!Sync` by default, so this newtype exists specifically
/// to carry an explicit `unsafe impl Sync` -- a bare `UnsafeCell` field
/// doesn't get that impl on its own.
struct TriggerDrainSlot(core::cell::UnsafeCell<Option<PendingStopDrain>>);

// SAFETY: every access goes through `cortex_m::interrupt::free` (see
// `store_pending_drain` and `take_pending_drain` below), mirroring
// `STEP_QUEUE_CHANNEL`'s own access discipline in this file.
unsafe impl Sync for TriggerDrainSlot {}

static TRIGGER_DRAIN_SLOT: TriggerDrainSlot = TriggerDrainSlot(core::cell::UnsafeCell::new(None));

/// Wakes `trigger_drain_task`. `signal(())` is lossless regardless of the
/// task's current execution state.
pub(crate) static TRIGGER_DRAIN_WAKE: embassy_sync::signal::Signal<
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    (),
> = embassy_sync::signal::Signal::new();

/// Store a freshly detached consumer and wake the drain task. Called only
/// from inside `stop_physical_stepper`'s existing masked window -- never
/// call this outside `cortex_m::interrupt::free`.
fn store_pending_drain(drain: PendingStopDrain) {
    // SAFETY: caller holds `cortex_m::interrupt::free` already -- see
    // `stop_physical_stepper_inner`'s `LockedInPlace` arm, the only caller.
    unsafe {
        *TRIGGER_DRAIN_SLOT.0.get() = Some(drain);
    }
    TRIGGER_DRAIN_WAKE.signal(());
}

/// Take the pending drain, if any, under its own short masked window --
/// never held across the drain loop below. Mirrors `detach_consumer`'s own
/// critical-section shape.
fn take_pending_drain() -> Option<PendingStopDrain> {
    cortex_m::interrupt::free(|_| {
        // SAFETY: see `store_pending_drain`.
        unsafe { (*TRIGGER_DRAIN_SLOT.0.get()).take() }
    })
}

/// Drain and reattach the pending stop, if any. Called only from
/// `trigger_drain_task` (P2), never from an ISR.
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
/// Accessed via `with_trigger_domain()` which uses
/// `cortex_m::interrupt::free()` for an interrupt-disabled critical
/// section, equivalent to RTIC P3 ceiling. This is NOT an RTIC shared
/// resource — it is a `static mut` with manual locking.
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

    /// Set `timer_active` for a stepper, updating the P1 mirror
    /// for physical steppers.
    pub(crate) fn set_timer_active(&mut self, stepper_idx: usize, active: bool) {
        set_timer_active(&mut self.state, stepper_idx, active);
    }

    /// Read `timer_active` for a stepper.
    pub(crate) fn timer_active(&self, stepper_idx: usize) -> bool {
        timer_active(&self.state, stepper_idx)
    }

    /// Attempt to trigger a trsync, dispatching stop signals to all
    /// linked steppers. Returns the trigger result.
    ///
    /// Must be called inside `with_trigger_domain()`.
    pub(crate) fn trigger(
        &mut self,
        trsync_oid: u8,
        reason: u8,
        clock: u32,
    ) -> foci_klipper::trigger::TriggerResult {
        let mut board = OpenFfboardDispatchBoard;
        trsync::dispatch_trigger(&mut self.state, &mut board, trsync_oid, reason, clock)
    }

    /// Arm TIM5 CC1 for the next unified scheduler deadline.
    ///
    /// Called inside the TriggerDomain lock from both P2 command handlers and
    /// the P3 ISR after processing events.
    pub(crate) fn arm_timer_at(&mut self, clock: u32) {
        arm_timer_at(&mut self.state, clock);
    }

    /// Disable the unified TIM5 scheduler compare.
    pub(crate) fn disarm_timer(&mut self) {
        disarm_timer(&mut self.state);
    }

    /// Reprogram TIM5 CC1 from the scheduler's earliest deadline.
    ///
    /// Called inside the TriggerDomain lock from P2 command handlers. Deadlines
    /// that are too close to arm reliably are clamped forward instead of
    /// forcing the TIM5 ISR to busy-wait with interrupts disabled.
    pub(crate) fn reprogram_timer(&mut self) {
        reprogram_timer(&mut self.state);
    }
}

impl Deref for TriggerDomain {
    type Target = TriggerDomainState;

    fn deref(&self) -> &Self::Target {
        &self.state
    }
}

impl DerefMut for TriggerDomain {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.state
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

pub(crate) fn set_timer_active(state: &mut TriggerDomainState, stepper_idx: usize, active: bool) {
    state.stepper_stop[stepper_idx].timer_active = active;
    // Mirror to P1-readable atomic for physical steppers
    if stepper_idx == 0 {
        stepper_state()
            .timer_active_mirror
            .store(active, core::sync::atomic::Ordering::Relaxed);
    }
}

pub(crate) fn timer_active(state: &TriggerDomainState, stepper_idx: usize) -> bool {
    state.stepper_stop[stepper_idx].timer_active
}

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
    // Ensure NVIC is unmasked (idempotent)
    unsafe {
        cortex_m::peripheral::NVIC::unmask(embassy_stm32::pac::Interrupt::TIM5);
    }
    state.scheduler.set_cc2_active(true);
}

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
/// 128 ticks at 84 MHz = ~1.5 us.
pub(crate) const TRIGGER_TIMER_MIN_LEAD_TICKS: u32 = 128;

/// TriggerDomain static. Initialized in RTIC `init`, accessed via
/// `with_trigger_domain()`.
pub(crate) static mut TRIGGER_DOMAIN: core::mem::MaybeUninit<TriggerDomain> =
    core::mem::MaybeUninit::uninit();

/// Lock TriggerDomain with interrupt-disabled critical section.
///
/// Equivalent to P3 ceiling since P3 is the highest application priority.
/// Callable from P2 command handlers and P3 ISR.
///
/// Uses `addr_of_mut!` to avoid the `static_mut_refs` lint.
#[inline]
pub(crate) fn with_trigger_domain<F, R>(f: F) -> R
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

/// Distinguishes how `stop_physical_stepper_inner` may access the step
/// queue's consumer, based on what the caller already holds.
enum QueueAccess {
    /// P1 direct-stop path: detach the consumer, drain with interrupts
    /// enabled, reattach.
    Unmasked,
    /// Trigger-dispatch/shutdown: caller already holds an outer mask this
    /// call is not unmasking. Detaches the consumer and hands it off to
    /// `trigger_drain_task` via `store_pending_drain` instead of draining
    /// it here.
    LockedInPlace,
}

/// Stop a physical stepper. Only acts on physical stepper indices (GPIO
/// and queue).
///
/// Per spec: clear interval, reset ISR stepper engine, drain queue (or hand
/// it off, for `LockedInPlace`), reset direction GPIO. The unified TIM5
/// scheduler is not disabled here because it may also be carrying endstop
/// and trsync timers; callers cancel the stepper scheduler event through
/// `TriggerDomain`.
fn stop_physical_stepper_inner(
    stepper_idx: usize,
    reason: StepperEventReason,
    access: QueueAccess,
) {
    use core::sync::atomic::Ordering::Relaxed;
    if !PHYSICAL_STEPPER_INDICES.contains(&stepper_idx) {
        return;
    }
    // 1. Clear step interval
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
    // 2. Reset ISR stepper engine state (clear stale partial segment).
    // SAFETY: Triggered stops call this inside `with_trigger_domain()` with
    // interrupts disabled. P1 stops clear `motor_armed` before calling this,
    // and the unified ISR checks that gate before touching stepper state.
    unsafe {
        isr_stepper(0)
            .expect("OpenFFBoard channel 0 exists")
            .reset();
    }
    // 3. Drain queue -- access mode depends on caller context. The
    // LockedInPlace arm hands the detached consumer to trigger_drain_task
    // instead of draining here.
    let mut drained_segments = 0u32;
    let mut drained_steps = 0u32;
    let mut deferred_record = false;
    match access {
        QueueAccess::Unmasked => {
            let mut consumer = detach_consumer();
            if let Some(consumer) = consumer.as_mut() {
                while let Some(m) = consumer.dequeue() {
                    drained_segments = drained_segments.saturating_add(1);
                    drained_steps = drained_steps.saturating_add(u32::from(m.count));
                }
            }
            if let Some(consumer) = consumer {
                attach_consumer(consumer);
            }
            // else: `detach_consumer` returned `None` -- possible because
            // a pending hand-off is sitting in `TRIGGER_DRAIN_SLOT`, or
            // because `trigger_drain_task` itself currently holds the
            // consumer detached mid-drain. Neither is reachable from this
            // Unmasked (P1) caller in practice: `trigger_drain_task` runs
            // at priority 2, which outranks P1, and the hand-off/drain
            // sequence contains no `.await` point where a lower-priority
            // P1 caller could interleave and observe the detached window.
            // The detach/no-op fallback keeps this branch total regardless.
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
            // else: a P1 unmasked drain, or trigger_drain_task itself,
            // currently holds the consumer detached, or a prior hand-off
            // is still sitting untaken in TRIGGER_DRAIN_SLOT (no drain in
            // flight at all -- just waiting for trigger_drain_task to be
            // scheduled) -- fall through and record zero additional
            // drained work below, matching the pre-existing recovery-rule
            // behavior for this case: whichever drain already owns the
            // consumer, in flight or queued to be taken by
            // trigger_drain_task, already owns draining this queue.
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
    // 4. Reset direction GPIO (PA0) to known low state
    let gpio_a = embassy_stm32::pac::GPIOA;
    gpio_a
        .odr()
        .modify(|w| w.set_odr(0, embassy_stm32::pac::gpio::vals::Odr::LOW));
    stepper_state().direction.store(true, Relaxed); // Forward = default
    // 5. Reset STEP GPIO (PB11) to LOW.
    //
    // Full-mode scheduling relies on the ISR toggling from a known
    // starting state: the first event of each segment must drive STEP
    // LOW → HIGH (the rising edge the TMC4671 counts). If the previous
    // segment ended mid-pulse (shutdown or stop between step and unstep
    // events) the pin could still be HIGH, which would flip the parity
    // for the next segment and lose the first motor step.
    let gpio_b = embassy_stm32::pac::GPIOB;
    gpio_b
        .odr()
        .modify(|w| w.set_odr(11, embassy_stm32::pac::gpio::vals::Odr::LOW));
}

/// Stop a physical stepper from the P1 direct-stop path
/// (`stepper_stop_from_p1`), with no critical section held. The queue
/// drain runs entirely with interrupts enabled.
///
/// This is the only correct caller. Calling this from inside an active
/// `with_trigger_domain` closure is a bug (not compiler-prevented --
/// enforced by this being this function's single call site, verified by
/// workspace reference search as part of code review).
pub(crate) fn stop_physical_stepper_unmasked(stepper_idx: usize, reason: StepperEventReason) {
    stop_physical_stepper_inner(stepper_idx, reason, QueueAccess::Unmasked);
}

/// Stop a physical stepper from inside an active `with_trigger_domain`
/// closure (trigger-dispatch, shutdown). Detaches the consumer under the
/// mask the caller already holds and hands it off to `trigger_drain_task`
/// via `store_pending_drain`; the queue drain itself runs later, outside
/// this mask, with interrupts enabled.
///
/// This is the only correct caller. Calling this without an active mask
/// already held is a bug (not compiler-prevented -- enforced by single
/// call sites, verified by workspace reference search as part of code
/// review).
pub(crate) fn stop_physical_stepper_masked(stepper_idx: usize, reason: StepperEventReason) {
    stop_physical_stepper_inner(stepper_idx, reason, QueueAccess::LockedInPlace);
}
