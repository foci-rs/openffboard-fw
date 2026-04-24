//! Handle `TmcRequest` messages that don't need main-loop motor state.
//!
//! Returned from the P2 protocol handler via the tmc-control channel.
//! `SetEnable`, `Init`, `TestSpin`, `SetPidGains`, `SetPositionGains`,
//! `Commission`, `Calibrate`, and `Tune` remain in the tmc_control main
//! loop because they mutate `motor_enabled` / `enable_pin` / engine
//! state. Everything else — SetCurrent / SetEncoder / SetEncoderDir /
//! DumpRegisters / TraceInfo / TraceFetch, and the `#[cfg(feature =
//! "dev")]` Read / Write pair — dispatches here.

use foci_firmware::tmc::TmcRequest;
use foci_tmc4671::spi::Tmc4671Spi;
use rtic_monotonics::Monotonic;
use rtic_monotonics::fugit::ExtU64;

use crate::app::Mono;
use crate::app_types::TmcSpiDevice;
use crate::board_config::CURRENT_SCALE_MA_PER_LSB;
use crate::state::trace::trace_buf;

/// Handle a TMC request that does not require main-loop state (motor_enabled, enable_pin).
///
/// SetEnable, Init, and TestSpin are handled inline by the tmc_control main loop.
/// Returns `Ok(())` on success, `Err(())` if the caller should enter shutdown.
pub(crate) async fn handle_tmc_request(
    req: TmcRequest,
    tmc: &mut Tmc4671Spi<TmcSpiDevice>,
) -> Result<(), ()> {
    match req {
        #[cfg(feature = "dev")]
        TmcRequest::Write { addr, value } => {
            let reg = foci_tmc4671::registers::Register::new(addr);
            if tmc.write_register(reg, value).await.is_err() {
                defmt::error!("TMC4671 SPI write failed: addr={:#04x}", addr);
            }
        }
        #[cfg(feature = "dev")]
        TmcRequest::Read { oid, addr } => {
            let reg = foci_tmc4671::registers::Register::new(addr);
            match tmc.read_register(reg).await {
                Ok(value) => {
                    ::ankyra::klipper_reply_from!(
                        &mut crate::_ankyra_config::Sender,
                        foci_firmware::ankyra::replies::TmcRegisterValue,
                        oid: u8 = oid,
                        addr: u8 = addr,
                        value: u32 = value,
                    );
                }
                Err(_) => {
                    defmt::error!("TMC4671 SPI read failed: addr={:#04x}", addr);
                }
            }
        }
        TmcRequest::SetCurrent { run_ma } => {
            foci_firmware::tmc_ops::apply_set_current(tmc, run_ma, CURRENT_SCALE_MA_PER_LSB)
                .await?;
        }
        TmcRequest::SetEncoder { ppr } => {
            foci_firmware::tmc_ops::apply_set_encoder(tmc, ppr).await?;
        }
        TmcRequest::SetEncoderDir { invert } => {
            foci_firmware::tmc_ops::apply_set_encoder_dir(tmc, invert).await?;
        }
        TmcRequest::SelfTest { .. } => {
            defmt::warn!("handle_tmc_request: SelfTest handled by main loop");
        }
        TmcRequest::DumpRegisters { oid } => {
            for &addr in foci_firmware::dump::DUMP_REGISTERS {
                // Yield between SPI reads to let USB task drain TX buffer.
                Mono::delay(1.millis()).await;

                let reg = foci_tmc4671::registers::Register::new(addr);
                let value = match tmc.read_register(reg).await {
                    Ok(v) => v,
                    Err(_) => {
                        defmt::error!("TMC4671 SPI read failed: addr={:#04x}", addr);
                        0
                    }
                };
                ::ankyra::klipper_reply_from!(
                    &mut crate::_ankyra_config::Sender,
                    foci_firmware::ankyra::replies::FociDumpValue,
                    oid: u8 = oid,
                    addr: u8 = addr,
                    value: u32 = value,
                );
            }
            // Sub-registers (INTERIM, PID_ERROR) use synthetic addrs 0x80+.
            for &(synthetic_addr, sub_reg) in foci_firmware::dump::DUMP_SUB_REGISTERS {
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
                ::ankyra::klipper_reply_from!(
                    &mut crate::_ankyra_config::Sender,
                    foci_firmware::ankyra::replies::FociDumpValue,
                    oid: u8 = oid,
                    addr: u8 = synthetic_addr,
                    value: u32 = value,
                );
            }
            Mono::delay(1.millis()).await;
            ::ankyra::klipper_reply_from!(
                &mut crate::_ankyra_config::Sender,
                foci_firmware::ankyra::replies::FociDumpDone,
                oid: u8 = oid,
            );
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
            ::ankyra::klipper_reply_from!(
                &mut crate::_ankyra_config::Sender,
                foci_firmware::ankyra::replies::FociTraceInfoResult,
                oid: u8 = oid,
                state: u8 = info.state,
                generation: u8 = info.generation,
                preset: u8 = info.preset,
                count: u16 = info.count,
                phase: u8 = info.phase_id,
                sample_period_us: u16 = info.sample_period_us,
                dropped: u16 = info.dropped,
            );
        }
        TmcRequest::TraceFetch {
            oid,
            offset,
            generation,
        } => {
            let buf = unsafe { trace_buf() };
            match buf.fetch(oid, offset, generation) {
                foci_firmware::trace::FetchResult::Ok(ring_idx) => {
                    let data = if buf.preset() == foci_firmware::trace::PRESET_FULL {
                        buf.sample_bytes_full(ring_idx)
                    } else {
                        buf.sample_bytes_fast(ring_idx)
                    };
                    ::ankyra::klipper_reply_from!(
                        &mut crate::_ankyra_config::Sender,
                        foci_firmware::ankyra::replies::FociTraceData,
                        oid: u8 = oid,
                        offset: u16 = offset,
                        status: u8 = 0,
                        data: &[u8] = data,
                    );
                }
                foci_firmware::trace::FetchResult::Busy => {
                    ::ankyra::klipper_reply_from!(
                        &mut crate::_ankyra_config::Sender,
                        foci_firmware::ankyra::replies::FociTraceData,
                        oid: u8 = oid,
                        offset: u16 = offset,
                        status: u8 = 1,
                        data: &[u8] = &[],
                    );
                }
                foci_firmware::trace::FetchResult::Invalid => {
                    ::ankyra::klipper_reply_from!(
                        &mut crate::_ankyra_config::Sender,
                        foci_firmware::ankyra::replies::FociTraceData,
                        oid: u8 = oid,
                        offset: u16 = offset,
                        status: u8 = 2,
                        data: &[u8] = &[],
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
