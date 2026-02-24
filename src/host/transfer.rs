//! Control, bulk, and interrupt transfer implementations.
//!
//! Contains the async methods that build qTD chains, link QHs to the
//! appropriate schedule, and wait for completion.

use crate::ehci::{
    self, QueueHead, TransferDescriptor, PID_IN, PID_OUT, PID_SETUP, SPEED_FULL, SPEED_HIGH,
    SPEED_LOW,
};
use crate::{cache, ral};
use core::cell::Cell;
use cotton_usb_host::host_controller::{DataPhase, TransferExtras, TransferType, UsbError};
use cotton_usb_host::wire::SetupPacket;

use super::controller::Imxrt1062HostController;
use super::futures::TransferComplete;
use super::interrupt_pipe::{Imxrt1062InterruptPipe, Pipe};
use super::NUM_QH;

impl Imxrt1062HostController {
    // -----------------------------------------------------------------------
    // EHCI error mapping
    // -----------------------------------------------------------------------

    /// Return a short string describing a `UsbError` variant (for logging,
    /// since `UsbError` does not implement `Debug` in `no_std`).
    pub(super) fn usb_error_str(e: &UsbError) -> &'static str {
        match e {
            UsbError::Stall => "Stall",
            UsbError::Timeout => "Timeout",
            UsbError::Overflow => "Overflow",
            UsbError::BitStuffError => "BitStuffError",
            UsbError::CrcError => "CrcError",
            UsbError::DataSeqError => "DataSeqError",
            UsbError::BufferTooSmall => "BufferTooSmall",
            UsbError::AllPipesInUse => "AllPipesInUse",
            UsbError::ProtocolError => "ProtocolError",
            UsbError::TooManyDevices => "TooManyDevices",
            UsbError::NoSuchEndpoint => "NoSuchEndpoint",
            _ => "Unknown",
        }
    }

    /// Map EHCI qTD status bits to a `UsbError`.
    pub(super) fn map_qtd_error(token: u32) -> UsbError {
        if token & ehci::QTD_TOKEN_HALTED != 0 {
            if token & ehci::QTD_TOKEN_BABBLE != 0 {
                return UsbError::Overflow;
            }
            if token & ehci::QTD_TOKEN_BUFFER_ERR != 0 {
                return UsbError::Overflow;
            }
            if token & ehci::QTD_TOKEN_XACT_ERR != 0 {
                return UsbError::ProtocolError;
            }
            // Halted with no other error bits set → STALL
            return UsbError::Stall;
        }
        if token & ehci::QTD_TOKEN_MISSED_UFRAME != 0 {
            return UsbError::Timeout;
        }
        UsbError::ProtocolError
    }

    // -----------------------------------------------------------------------
    // Cache maintenance wrappers
    // -----------------------------------------------------------------------

    /// Clean and invalidate a QH for DMA.
    pub(super) fn cache_clean_qh(qh: *const QueueHead) {
        cache::clean_invalidate_dcache_by_address(qh as usize, core::mem::size_of::<QueueHead>());
    }

    /// Clean and invalidate a qTD for DMA.
    pub(super) fn cache_clean_qtd(qtd: *const TransferDescriptor) {
        cache::clean_invalidate_dcache_by_address(
            qtd as usize,
            core::mem::size_of::<TransferDescriptor>(),
        );
    }

    /// Clean and invalidate a data buffer for DMA.
    pub(super) fn cache_clean_buffer(addr: *const u8, len: usize) {
        if len > 0 {
            cache::clean_invalidate_dcache_by_address(addr as usize, len);
        }
    }

    // -----------------------------------------------------------------------
    // Diagnostic helpers
    // -----------------------------------------------------------------------

    /// Log the periodic schedule chain for debugging.
    ///
    /// Walks `frame_list[0]` → QH → QH → ... and logs the chain of QH indices.
    /// When logging is disabled (no `log` or `defmt-03` feature), this compiles
    /// to nothing because `debug!` expands to an empty statement.
    fn log_periodic_chain(&self) {
        use crate::ehci::{link_address, link_is_terminate};

        let head = self.statics.frame_list.entries[0].read();
        if link_is_terminate(head) {
            debug!("[HC]   periodic chain: [empty]");
        } else {
            let mut chain_buf = [0u8; 64];
            let mut pos = 0;
            let mut link = head;
            for _ in 0..NUM_QH + 1 {
                if link_is_terminate(link) {
                    break;
                }
                let addr = link_address(link);
                // Find which QH index this address corresponds to
                let mut found_idx: i8 = -1;
                for qi in 0..=NUM_QH {
                    let qa = self.statics.qh_ptr(qi) as u32;
                    if qa == addr {
                        found_idx = qi as i8;
                        break;
                    }
                }
                if pos + 8 < chain_buf.len() {
                    if pos > 0 {
                        chain_buf[pos] = b'-';
                        chain_buf[pos + 1] = b'>';
                        pos += 2;
                    }
                    chain_buf[pos] = b'Q';
                    chain_buf[pos + 1] = b'H';
                    if found_idx >= 0 {
                        chain_buf[pos + 2] = b'0' + (found_idx as u8);
                    } else {
                        chain_buf[pos + 2] = b'?';
                    }
                    pos += 3;
                }
                // Follow horizontal_link
                let qh_ptr = addr as *const QueueHead;
                cache::clean_invalidate_dcache_by_address(
                    qh_ptr as usize,
                    core::mem::size_of::<QueueHead>(),
                );
                // SAFETY: `addr` is extracted from the frame list or a QH's
                // horizontal_link, which points to a QH in the static pool.
                // Cache was just invalidated above.
                link = unsafe { (*qh_ptr).horizontal_link.read() };
            }
            let chain_str = core::str::from_utf8(&chain_buf[..pos]).unwrap_or("??");
            debug!("[HC]   periodic chain: {}", chain_str);
        }
    }

    // -----------------------------------------------------------------------
    // Control transfer implementation
    // -----------------------------------------------------------------------

    /// Perform an EHCI control transfer using a qTD chain.
    ///
    /// Builds 2–3 qTDs (setup + optional data + status), configures a QH,
    /// links it to the async schedule, and waits for completion.
    pub(super) async fn do_control_transfer(
        &self,
        address: u8,
        transfer_extras: TransferExtras,
        packet_size: u8,
        setup: &SetupPacket,
        data_phase: &mut DataPhase<'_>,
    ) -> Result<usize, UsbError> {
        // Allocate a QH (index 1 is reserved for control transfers)
        let qh_index = 1;
        // SAFETY: index 1 is exclusively used for control transfers (single
        // concurrent control pipe).  Returns a valid `*mut QueueHead` from the
        // static pool via `UnsafeCell::get()`.
        let qh = unsafe { self.qh_mut(qh_index) };

        // Determine device speed.
        // WithPreamble indicates a Low Speed device behind a Full Speed hub.
        // In that case, override port_speed() (which reports the root port
        // speed, not the target device speed) to SPEED_LOW.
        let speed = match transfer_extras {
            TransferExtras::WithPreamble => SPEED_LOW,
            TransferExtras::Normal => match self.port_speed() {
                0 => SPEED_FULL,
                1 => SPEED_LOW,
                2 => SPEED_HIGH,
                _ => SPEED_FULL,
            },
        };

        // Build QH characteristics
        let characteristics = ehci::qh_characteristics(
            address,
            0, // endpoint 0 (control)
            speed,
            packet_size as u16,
            true,  // is_control
            false, // not head of reclamation
        );

        // Build QH capabilities.
        // For WithPreamble (LS behind FS hub), the NXP EHCI embedded TT
        // handles the FS↔LS conversion at the root port. No explicit
        // hub_addr/hub_port is needed when the hub is at Full Speed on
        // the root port (PFSC=1 mode).
        let capabilities = ehci::qh_capabilities(0, 0, 0, 0, 1);

        // Log QH configuration for debugging.
        // Use debug! for hub-connected devices (addr > 1) to help diagnose hub issues.
        let speed_str = match speed {
            SPEED_FULL => "FS",
            SPEED_LOW => "LS",
            SPEED_HIGH => "HS",
            _ => "??",
        };
        let extras_str = match transfer_extras {
            TransferExtras::WithPreamble => "WithPreamble(LS)",
            TransferExtras::Normal => "Normal",
        };
        debug!(
            "[HC] control xfer: addr={} pkt={} speed={} extras={} char=0x{:08X} caps=0x{:08X}",
            address, packet_size, speed_str, extras_str, characteristics, capabilities
        );

        // Initialise the QH
        // SAFETY: `qh` is a valid pool pointer (see qh_mut call above).
        // No DMA is active on this QH yet — it hasn't been linked to a schedule.
        unsafe { (*qh).init_endpoint(characteristics, capabilities) };

        // ---- Build the qTD chain ----

        // We need up to 3 qTDs: setup, data (optional), status.
        // RAII guards auto-free on early return (e.g. if a later alloc fails).
        let setup_slot = self.alloc_qtd().ok_or(UsbError::AllPipesInUse)?;
        let data_slot = match data_phase {
            DataPhase::In(_) | DataPhase::Out(_) => {
                Some(self.alloc_qtd().ok_or(UsbError::AllPipesInUse)?)
            }
            DataPhase::None => None,
        };
        let status_slot = self.alloc_qtd().ok_or(UsbError::AllPipesInUse)?;

        // Setup qTD: PID=SETUP, 8 bytes, data toggle=0, no IOC
        let setup_qtd = setup_slot.ptr();
        let setup_bytes = setup as *const SetupPacket as *const u8;
        let setup_token = ehci::qtd_token(PID_SETUP, 8, false, false);
        // SAFETY: All qTD pointers are obtained from QtdSlot::ptr() which
        // returns valid, aligned `*mut TransferDescriptor` from the static pool.
        // The RAII guard guarantees exclusive ownership of each slot.
        // `setup_bytes` points to a valid SetupPacket on the caller's stack;
        // `buf.as_ptr()` points to a valid DMA-accessible data buffer.
        // Buffer pointers must remain valid until the transfer completes
        // (guaranteed by the await below, which blocks until DMA finishes).
        unsafe { (*setup_qtd).init(setup_token, setup_bytes, 8) };

        // Data qTD (if present)
        //
        // For IN control transfers, the data qTD's Total Bytes must be capped
        // to wLength from the setup packet. cotton-usb-host may pass a buffer
        // larger than wLength (e.g., 18-byte buffer with wLength=8 for initial
        // GET_DESCRIPTOR). If Total Bytes exceeds wLength, the EHCI controller
        // will issue additional IN tokens after the device has sent all its data,
        // causing the device to STALL.
        let data_len: usize;
        match data_phase {
            DataPhase::In(ref buf) => {
                // Cap transfer size to wLength to avoid requesting more data
                // than the device will send.
                let wlength = setup.wLength as usize;
                data_len = if wlength > 0 && wlength < buf.len() {
                    wlength
                } else {
                    buf.len()
                };
                let data_qtd = data_slot.as_ref().unwrap().ptr();
                let data_token = ehci::qtd_token(PID_IN, data_len as u32, true, false);
                // SAFETY: data qTD pointer from QtdSlot; buffer from caller (DMA-accessible).
                unsafe {
                    (*data_qtd).init(data_token, buf.as_ptr(), data_len as u32);
                }
            }
            DataPhase::Out(buf) => {
                data_len = buf.len();
                let data_qtd = data_slot.as_ref().unwrap().ptr();
                let data_token = ehci::qtd_token(PID_OUT, data_len as u32, true, false);
                // SAFETY: data qTD pointer from QtdSlot; buffer from caller (DMA-accessible).
                unsafe {
                    (*data_qtd).init(data_token, buf.as_ptr(), data_len as u32);
                }
            }
            DataPhase::None => {
                data_len = 0;
            }
        }

        // Status qTD: opposite direction of data (or IN if no data), 0 bytes,
        // data toggle=1, IOC=true
        let status_pid = match data_phase {
            DataPhase::In(_) => PID_OUT,
            DataPhase::Out(_) | DataPhase::None => PID_IN,
        };
        let status_qtd = status_slot.ptr();
        let status_token = ehci::qtd_token(status_pid, 0, true, true);
        // SAFETY: status qTD pointer from QtdSlot; no data buffer (zero-length transfer).
        unsafe { (*status_qtd).init(status_token, core::ptr::null(), 0) };

        // Chain qTDs: setup → data (optional) → status
        // SAFETY: All qTD pointers from QtdSlot, valid and exclusively owned.
        // Writing `next` links the qTDs into a chain for the EHCI DMA engine.
        match &data_slot {
            Some(slot) => {
                let data_qtd = slot.ptr();
                unsafe {
                    (*setup_qtd).next.write(data_qtd as u32);
                    (*data_qtd).next.write(status_qtd as u32);
                }
            }
            None => unsafe {
                (*setup_qtd).next.write(status_qtd as u32);
            },
        }

        // Attach the first qTD to the QH
        // SAFETY: `qh` and `setup_qtd` are valid pool pointers.  attach_qtd()
        // writes the qTD address into the QH overlay and clears the halt bit.
        unsafe { (*qh).attach_qtd(setup_qtd) };

        // ---- Cache maintenance before DMA ----

        // Clean the setup packet data (it's on the stack, needs to be in RAM)
        Self::cache_clean_buffer(setup_bytes, 8);

        // Clean outgoing data buffer if applicable
        if let DataPhase::Out(buf) = data_phase {
            Self::cache_clean_buffer(buf.as_ptr(), buf.len());
        }

        // Clean all qTDs
        Self::cache_clean_qtd(setup_qtd);
        if let Some(ref slot) = data_slot {
            Self::cache_clean_qtd(slot.ptr());
        }
        Self::cache_clean_qtd(status_qtd);

        // ---- Link QH to async schedule, clean, and enable ----

        // SAFETY: QH and its qTD chain are fully initialized and cache-cleaned.
        // link_and_start_async handles sentinel linkage and schedule enable.
        unsafe { self.link_and_start_async(qh) };

        // ---- Poll for completion ----

        let result = TransferComplete {
            usb: &self.usb,
            shared: self.shared,
            statics: self.statics,
            status_qtd_index: status_slot.index(),
            data_qtd_index: data_slot.as_ref().map(|s| s.index()),
            qh_index,
            waker_index: 0, // control pipe uses waker slot 0
        }
        .await;

        // ---- Unlink QH from async schedule ----

        // SAFETY: QH was linked by link_and_start_async above.  The await
        // ensures the controller acknowledges it has stopped accessing the QH.
        unsafe { self.unlink_and_stop_async(qh) }.await;

        // ---- Copy data for IN transfers ----

        let bytes_transferred = match result {
            Ok(()) => {
                match data_phase {
                    DataPhase::In(ref mut buf) => {
                        // Invalidate cache for the IN data buffer
                        Self::cache_clean_buffer(buf.as_ptr(), buf.len());

                        // Read how many bytes were actually transferred
                        if let Some(ref slot) = data_slot {
                            let data_qtd_ptr = slot.ptr();
                            // Invalidate cache to read updated qTD token
                            Self::cache_clean_qtd(data_qtd_ptr);
                            // SAFETY: data qTD pointer from QtdSlot; cache just invalidated.
                            let remaining = unsafe { (*data_qtd_ptr).bytes_remaining() } as usize;
                            Ok(data_len - remaining)
                        } else {
                            Ok(0)
                        }
                    }
                    DataPhase::Out(buf) => Ok(buf.len()),
                    DataPhase::None => Ok(0),
                }
            }
            Err(e) => {
                // Log raw qTD tokens for diagnosis
                // SAFETY: All qTD pointers from QtdSlot, valid pool entries.
                // Cache is cleaned before each read to see hardware-updated values.
                Self::cache_clean_qtd(setup_qtd);
                let setup_token_val = unsafe { (*setup_qtd).token.read() };
                let status_token_val = unsafe { (*status_qtd).token.read() };
                let data_token_val = data_slot
                    .as_ref()
                    .map(|s| {
                        let p = s.ptr();
                        Self::cache_clean_qtd(p);
                        unsafe { (*p).token.read() }
                    })
                    .unwrap_or(0);
                let portsc = ral::read_reg!(ral::usb, self.usb, PORTSC1);
                debug!("[HC] control xfer FAILED: err={} setup_tok=0x{:08X} data_tok=0x{:08X} status_tok=0x{:08X} PORTSC1=0x{:08X}",
                    Self::usb_error_str(&e),
                    setup_token_val,
                    data_token_val,
                    status_token_val,
                    portsc);
                Err(e)
            }
        };

        // ---- Free resources ----
        // qTD slots are automatically freed when setup_slot, data_slot,
        // status_slot drop at the end of this function.
        // SAFETY: QH pointer from pool, transfer complete, QH unlinked from schedule.
        unsafe { (*qh).sw_flags.write(0) };

        bytes_transferred
    }

    // -----------------------------------------------------------------------
    // Bulk transfer implementation
    // -----------------------------------------------------------------------

    /// Perform an EHCI bulk transfer (IN or OUT) on the async schedule.
    ///
    /// Uses one qTD for transfers up to ~20 KB. Data toggle is hardware-managed
    /// (DTC=0 in QH) and tracked across calls via the `data_toggle` Cell.
    ///
    /// For VariableSize OUT transfers where `data_len` is an exact multiple of
    /// `packet_size`, an extra zero-length qTD is chained to signal end-of-transfer
    /// per USB 2.0 §5.8.
    ///
    /// # Safety (internal)
    /// `data` must be valid for `data_len` bytes and DMA-accessible (not DTCM).
    #[allow(clippy::too_many_arguments)]
    pub(super) async fn do_bulk_transfer(
        &self,
        address: u8,
        endpoint: u8,
        packet_size: u16,
        data: *mut u8,
        data_len: usize,
        pid: u32,
        is_in: bool,
        transfer_type: TransferType,
        data_toggle: &Cell<bool>,
    ) -> Result<usize, UsbError> {
        // 1. Allocate a pipe from the bulk_pipes pool.
        let pipe = Pipe::new(self.statics.bulk_pipes.alloc().await, 1);
        let qh_index = pipe.which() as usize + 1;
        let waker_idx = pipe.which() as usize;

        // 2. Determine port speed.
        let speed = match self.port_speed() {
            0 => SPEED_FULL,
            1 => SPEED_LOW,
            2 => SPEED_HIGH,
            _ => SPEED_FULL,
        };

        // Workaround: cotton-usb-host v0.2.1 hardcodes packet_size=64 for bulk
        // transfers (a TODO in their code). For High Speed bulk, USB 2.0 §5.8.3
        // mandates wMaxPacketSize=512 as the only valid value. Override here.
        let actual_packet_size = if speed == SPEED_HIGH && packet_size < 512 {
            512
        } else {
            packet_size
        };

        // 3. Initialize the QH with DTC=0 (hardware manages data toggle in overlay).
        // SAFETY: `qh_index` is derived from a pool allocation (pipe.which() + 1),
        // guaranteed unique.  Returns valid `*mut QueueHead` from the static pool.
        let qh = unsafe { self.qh_mut(qh_index) };
        let characteristics = ehci::qh_characteristics(
            address,
            endpoint,
            speed,
            actual_packet_size,
            false, // not a control endpoint → DTC=0
            false, // not head of reclamation list
        );
        let capabilities = ehci::qh_capabilities(0, 0, 0, 0, 1);
        // SAFETY: `qh` is a valid pool pointer; not yet linked to any schedule.
        unsafe { (*qh).init_endpoint(characteristics, capabilities) };

        // 4. Determine if a ZLP is needed.
        // Per USB 2.0 §5.8: a VariableSize OUT transfer that fills an exact number
        // of packets must append a zero-length packet to signal end-of-transfer.
        let need_zlp = !is_in
            && transfer_type == TransferType::VariableSize
            && data_len > 0
            && actual_packet_size as usize > 0
            && data_len % actual_packet_size as usize == 0;

        // 5. Allocate qTD(s). RAII guards auto-free on early return.
        let data_slot = self.alloc_qtd().ok_or(UsbError::AllPipesInUse)?;
        let zlp_slot = if need_zlp {
            Some(self.alloc_qtd().ok_or(UsbError::AllPipesInUse)?)
        } else {
            None
        };

        // 6. Initialize the data qTD. IOC is set on the last qTD in the chain.
        let data_qtd = data_slot.ptr();
        let data_token = ehci::qtd_token(pid, data_len as u32, false, !need_zlp);
        // SAFETY: qTD pointer from QtdSlot (valid, aligned, exclusively owned).
        // `data` is guaranteed valid for `data_len` bytes by the caller.
        unsafe { (*data_qtd).init(data_token, data as *const u8, data_len as u32) };

        // 7. If a ZLP is needed: initialize the ZLP qTD and chain data → ZLP.
        // SAFETY: ZLP qTD pointer from QtdSlot; data qTD from above.  Both are
        // valid static pool entries with exclusive ownership via RAII guards.
        if let Some(ref zlp) = zlp_slot {
            let zlp_qtd = zlp.ptr();
            let zlp_token = ehci::qtd_token(pid, 0, false, true); // IOC on ZLP
            unsafe { (*zlp_qtd).init(zlp_token, core::ptr::null(), 0) };
            // Chain: data qTD → ZLP qTD
            unsafe { (*data_qtd).next.write(zlp_qtd as u32) };
        }

        // 8. Attach first qTD to QH. attach_qtd() clears overlay_token to 0;
        //    then set the initial data toggle bit per data_toggle.get().
        // SAFETY: `qh` and `data_qtd` are valid pool pointers.  attach_qtd()
        // writes qTD address into QH overlay.  set_overlay_toggle() sets bit 31
        // of the overlay token.  No DMA is active yet (QH not linked to schedule).
        unsafe { (*qh).attach_qtd(data_qtd) };
        if data_toggle.get() {
            // SAFETY: `qh` is a valid pool pointer (see above).  No DMA is
            // active yet (QH not linked to schedule).
            unsafe { (*qh).set_overlay_toggle(true) };
        }

        // 9. For OUT: clean the outgoing data buffer before DMA starts.
        if !is_in && data_len > 0 {
            Self::cache_clean_buffer(data as *const u8, data_len);
        }

        // 10. Clean qTDs, then link+clean+enable via helper.
        Self::cache_clean_qtd(data_qtd);
        if let Some(ref zlp) = zlp_slot {
            Self::cache_clean_qtd(zlp.ptr());
        }

        // SAFETY: QH and qTD chain fully initialized and cache-cleaned above.
        unsafe { self.link_and_start_async(qh) };

        // 12. Poll for transfer completion.
        let result = TransferComplete {
            usb: &self.usb,
            shared: self.shared,
            statics: self.statics,
            status_qtd_index: if let Some(ref zlp) = zlp_slot {
                zlp.index()
            } else {
                data_slot.index()
            },
            data_qtd_index: if zlp_slot.is_some() {
                Some(data_slot.index())
            } else {
                None
            },
            qh_index,
            waker_index: waker_idx,
        }
        .await;

        // 13. Compute the byte count from the transfer result.
        if let Err(ref e) = result {
            debug!(
                "[HC] bulk {} addr={} ep={} len={} -> Err({})",
                if is_in { "IN" } else { "OUT" },
                address,
                endpoint,
                data_len,
                Self::usb_error_str(e),
            );
        }
        let byte_result: Result<usize, UsbError> = match result {
            Ok(()) => {
                if is_in {
                    // Invalidate the IN data buffer (DMA wrote to it; don't write back).
                    if data_len > 0 {
                        cache::invalidate_dcache_by_address(data as usize, data_len);
                    }
                    // Invalidate+read the data qTD token to compute bytes received.
                    Self::cache_clean_qtd(data_qtd);
                    // SAFETY: data qTD pointer from QtdSlot; cache just invalidated.
                    let token = unsafe { (*data_qtd).token.read() };
                    let remaining = ehci::qtd_token_bytes_remaining(token) as usize;
                    Ok(data_len.saturating_sub(remaining))
                } else {
                    Ok(data_len)
                }
            }
            Err(e) => Err(e),
        };

        // 14. Unlink QH from async schedule and wait for hardware acknowledgement.
        // SAFETY: QH was linked by link_and_start_async above; transfer complete.
        unsafe { self.unlink_and_stop_async(qh) }.await;

        // 15. Read the data toggle from the QH overlay for the next transfer.
        //     The controller writes the next expected toggle into the overlay_token
        //     DT bit (bit 31) when it updates the overlay at end-of-transfer.
        Self::cache_clean_qh(qh);
        // SAFETY: QH pointer from pool; cache just invalidated; QH unlinked.
        let new_toggle = unsafe { (*qh).overlay_token.read() } & (1 << 31) != 0;
        data_toggle.set(new_toggle);

        // 16. Free resources.
        // qTD slots auto-free when data_slot and zlp_slot drop.
        // SAFETY: QH pointer from pool, transfer complete, QH unlinked from schedule.
        unsafe { (*qh).sw_flags.write(0) };

        // `pipe` drops here, returning the bulk_pipes slot to the pool.
        byte_result
    }

    // -----------------------------------------------------------------------
    // Interrupt pipe implementation
    // -----------------------------------------------------------------------

    /// Set up and return an interrupt pipe for polling an IN endpoint.
    ///
    /// Called by both [`alloc_interrupt_pipe`] (after an async pool allocation)
    /// and [`try_alloc_interrupt_pipe`] (after a synchronous try-alloc).
    ///
    /// # Panics
    ///
    /// Panics if the qTD pool is exhausted (all `NUM_QTD` slots are in use).
    /// This should not happen in normal operation because the pipe pool
    /// (`bulk_pipes`) limits the maximum number of concurrent pipes, which
    /// bounds qTD consumption.
    pub(super) fn do_alloc_interrupt_pipe(
        &self,
        pipe: Pipe,
        address: u8,
        transfer_extras: TransferExtras,
        endpoint: u8,
        max_packet_size: u16,
        _interval_ms: u8,
    ) -> Imxrt1062InterruptPipe {
        // Map pipe slot to pool indices.
        // bulk_pipes tokens are 0..NUM_QH-2; Pipe::new(pooled, 1) makes which=1..NUM_QH-1.
        //   QH index  = pipe.which() as usize + 1  → qh_pool[2..=NUM_QH]
        //   recv_buf  = pipe.which() as usize - 1  → recv_bufs[0..NUM_QH-2]
        //   waker idx = pipe.which() as usize       → pipe_wakers[1..NUM_QH-1]
        let qh_index = pipe.which() as usize + 1;
        let recv_buf_idx = pipe.which() as usize - 1;

        // Allocate a qTD for the receive buffer.
        // We use a dedicated qTD for the lifetime of the pipe (one in-flight at a time).
        let qtd_slot = self
            .alloc_qtd()
            .expect("qTD pool exhausted for interrupt pipe");
        let qtd_index = qtd_slot.index();

        // Determine device speed.
        // WithPreamble indicates a Low Speed device behind a Full Speed hub.
        let speed = match transfer_extras {
            TransferExtras::WithPreamble => ehci::SPEED_LOW,
            TransferExtras::Normal => match self.port_speed() {
                0 => ehci::SPEED_FULL,
                1 => ehci::SPEED_LOW,
                _ => ehci::SPEED_HIGH,
            },
        };

        // Build QH endpoint characteristics.
        // DTC = 0 (hardware-managed data toggle in QH overlay) for non-control endpoints.
        // RL (NAK Reload) MUST be 0 for periodic schedule QHs (EHCI §3.6).
        // qh_characteristics() sets RL=15 for async schedule use; clear it here.
        let characteristics = ehci::qh_characteristics(
            address,
            endpoint,
            speed,
            max_packet_size,
            false, // not a control endpoint
            false, // not head of reclamation list
        ) & !(0xF << 28); // Clear RL bits [31:28] — must be 0 for periodic QHs

        // Build QH endpoint capabilities.
        // S-mask = 0x01: poll in micro-frame 0 of each scheduled frame.
        // C-mask = 0: no split-completion mask (not a split transaction).
        // hub_addr/hub_port = 0: device is directly connected (no TT).
        let capabilities = ehci::qh_capabilities(0x01, 0, 0, 0, 1);

        // Initialise the QH.
        // SAFETY: `qh_index` derived from pipe pool allocation (unique).
        // QH and qTD pointers from the static pool via UnsafeCell::get(), valid
        // and aligned.  No DMA is active on these descriptors yet.
        let qh = unsafe { self.qh_mut(qh_index) };
        // SAFETY: `qh` is a valid pool pointer (from qh_mut above).
        // No DMA is active on this QH yet — it hasn't been linked to a schedule.
        unsafe { (*qh).init_endpoint(characteristics, capabilities) };

        // Set up the initial qTD: PID=IN, Active, max_packet_size bytes, IOC.
        let recv_buf_ptr = self.statics.recv_bufs[recv_buf_idx].as_ptr();
        // DIAG Step 1: Confirm recv_buf is in DMA-accessible memory (OCRAM 0x2020_xxxx/0x2024_xxxx).
        // If address is 0x2000_xxxx (DTCM), EHCI DMA cannot write there → always zeros.
        debug!(
            "[HC] recv_buf[{}] @ 0x{:08x} (OCRAM=0x2020_0000..0x202F_FFFF, DTCM=0x2000_xxxx)",
            recv_buf_idx, recv_buf_ptr as u32,
        );
        let token = ehci::qtd_token(PID_IN, max_packet_size as u32, false, true);
        let qtd = qtd_slot.ptr();
        // SAFETY: qTD pointer from QtdSlot (valid, aligned, exclusively owned).
        // `recv_buf_ptr` points to a static DMA-accessible RecvBuf.
        unsafe { (*qtd).init(token, recv_buf_ptr, max_packet_size as u32) };

        // Attach qTD to QH (sets overlay_next, clears halt — OK for first attach).
        // SAFETY: `qh` and `qtd` are valid pool pointers; no DMA active yet.
        unsafe { (*qh).attach_qtd(qtd) };

        // Cache maintenance: clean qTD, recv_buf, QH, and frame list before linking.
        Self::cache_clean_qtd(qtd);
        Self::cache_clean_buffer(recv_buf_ptr, max_packet_size as usize);
        Self::cache_clean_qh(qh);
        cache::clean_invalidate_dcache_by_address(
            self.statics.frame_list.entries.as_ptr() as usize,
            core::mem::size_of::<ehci::FrameList>(),
        );

        // Insert QH at the head of the periodic schedule.
        // SAFETY: QH fully initialized with qTD attached, all cache-cleaned above.
        unsafe { self.link_qh_to_periodic_schedule(qh) };

        // Cache-clean QH (horizontal_link changed) and frame list (all entries changed).
        Self::cache_clean_qh(qh);
        cache::clean_invalidate_dcache_by_address(
            self.statics.frame_list.entries.as_ptr() as usize,
            core::mem::size_of::<ehci::FrameList>(),
        );

        // Diagnostic: log speed, QH config, and periodic schedule chain.
        let extras_str = match transfer_extras {
            TransferExtras::WithPreamble => "WithPreamble(LS)",
            TransferExtras::Normal => "Normal",
        };
        let speed_str = match speed {
            ehci::SPEED_FULL => "FS",
            ehci::SPEED_LOW => "LS",
            ehci::SPEED_HIGH => "HS",
            _ => "??",
        };
        debug!(
            "[HC] interrupt pipe allocated: addr={} ep={} mps={} qh={} qtd={} extras={} speed={}",
            address, endpoint, max_packet_size, qh_index, qtd_index, extras_str, speed_str
        );
        debug!(
            "[HC]   QH char=0x{:08X} caps=0x{:08X}",
            characteristics, capabilities
        );

        // Log the periodic schedule chain for diagnostics.
        self.log_periodic_chain();

        Imxrt1062InterruptPipe {
            pipe,
            qh_index,
            qtd_slot,
            recv_buf_idx,
            address,
            endpoint,
            max_packet_size,
            statics: self.statics,
            shared: self.shared,
            usb_base: self.usb.addr as u32,
        }
    }
}
