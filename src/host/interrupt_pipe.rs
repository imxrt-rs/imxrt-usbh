//! Interrupt IN pipe — periodic schedule endpoint stream with RAII cleanup.

use crate::ehci::{self, PID_IN};
use crate::ral;
use core::pin::Pin;
use core::task::{Context, Poll};
use cotton_usb_host::host_controller::InterruptPacket;
use futures_core::Stream;

use super::controller::ImxrtHostController;
use super::schedule::QtdSlot;
use super::shared::UsbShared;
use super::statics::UsbStatics;

// ---------------------------------------------------------------------------
// Pipe — RAII pipe allocation wrapper
// ---------------------------------------------------------------------------

/// Wraps a pool allocation for a pipe. When dropped, returns the resource
/// to the pool.
pub(super) struct Pipe {
    _pooled: cotton_usb_host::async_pool::Pooled<'static>,
    which: u8,
}

impl Pipe {
    pub(super) fn new(pooled: cotton_usb_host::async_pool::Pooled<'static>, offset: u8) -> Self {
        let which = pooled.which() + offset;
        Self {
            _pooled: pooled,
            which,
        }
    }

    pub(super) fn which(&self) -> u8 {
        self.which
    }
}

// ---------------------------------------------------------------------------
// ImxrtInterruptPipe — periodic schedule interrupt endpoint stream
// ---------------------------------------------------------------------------

/// Interrupt IN pipe for i.MX RT 1062.
///
/// Wraps a single QH + qTD polling an interrupt IN endpoint via the EHCI
/// periodic schedule. Implements `Stream<Item = InterruptPacket>` so callers
/// can `await` the next packet with standard async combinators.
///
/// # Lifecycle
///
/// Created by `ImxrtHostController::alloc_interrupt_pipe` or
/// `try_alloc_interrupt_pipe`. The pipe occupies one slot from the
/// `bulk_pipes` pool and one slot from the `qtd_pool` for its entire lifetime.
///
/// On `Drop`, the QH is unlinked from the periodic frame list and the qTD
/// slot is freed. A brief (~1 ms) busy-wait ensures the EHCI controller has
/// crossed at least one frame boundary before resources are released.
pub struct ImxrtInterruptPipe {
    /// Pool allocation (RAII — frees the `bulk_pipes` slot on Drop).
    pub(super) pipe: Pipe,
    /// Index into `statics.qh_pool` for this pipe's QH.
    pub(super) qh_index: usize,
    /// RAII guard for this pipe's qTD slot.  Automatically frees the qTD
    /// when the pipe is dropped (after the explicit Drop body runs).
    pub(super) qtd_slot: QtdSlot,
    /// Index into `statics.recv_bufs` for the DMA receive buffer.
    pub(super) recv_buf_idx: usize,
    /// USB device address.
    pub(super) address: u8,
    /// Endpoint number.
    pub(super) endpoint: u8,
    /// Maximum packet size (used when re-arming the qTD).
    pub(super) max_packet_size: u16,
    /// Static resource pools.
    pub(super) statics: &'static UsbStatics,
    /// ISR ↔ async shared state.
    pub(super) shared: &'static UsbShared,
    /// USB OTG register block base address (stored as `u32` to keep the struct
    /// `Send` without a manual `unsafe impl`).
    pub(super) usb_base: u32,
}

impl Stream for ImxrtInterruptPipe {
    type Item = InterruptPacket;

    fn poll_next(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<Self::Item>> {
        // Register waker before checking status (race-free pattern).
        self.shared
            .pipe_waker(self.pipe.which() as usize)
            .register(cx.waker());

        // Reconstruct Instance from base address (cheap, no allocation).
        let usb_inst = ral::usb::Instance {
            addr: self.usb_base as *const ral::usb::RegisterBlock,
        };

        let qtd_ptr = self.qtd_slot.ptr();

        // SAFETY: qTD pointer from QtdSlot::ptr() — valid, aligned static pool
        // entry exclusively owned by this pipe.  DMA buffers are in non-cacheable
        // memory so hardware writes are immediately visible.
        let token = unsafe { (*qtd_ptr).token.read() };

        if token & ehci::QTD_TOKEN_ACTIVE != 0 {
            // Transfer still in progress — re-enable transfer interrupts and wait.
            ral::modify_reg!(ral::usb, usb_inst, USBINTR, UE: 1, UEE: 1, UAIE: 1, UPIE: 1);
            return Poll::Pending;
        }

        // If the qTD halted, the device is likely disconnected or the endpoint
        // stalled. Terminate the stream (return None) so the application's
        // inner poll loop breaks out and the outer event loop can poll
        // DeviceDetect to handle the disconnect. Do NOT re-arm the qTD —
        // re-arming a halted pipe when the device is absent creates an
        // infinite busy-loop of halt → re-arm → halt.
        if token & ehci::QTD_TOKEN_HALTED != 0 {
            debug!(
                "[HC] InterruptPipe: qTD halted (token=0x{:08x}), terminating stream",
                token,
            );
            // Re-enable port change interrupt so DeviceDetect can fire.
            ral::modify_reg!(ral::usb, usb_inst, USBINTR, PCE: 1);
            return Poll::Ready(None);
        }

        // --- Transfer complete (Active cleared, no halt) ---

        let recv_buf = &self.statics.recv_bufs[self.recv_buf_idx];

        debug!(
            "[HC] qTD done: token=0x{:08x} rem={} buf=[{:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x}]",
            token,
            ehci::qtd_token_bytes_remaining(token),
            recv_buf[0], recv_buf[1], recv_buf[2], recv_buf[3],
            recv_buf[4], recv_buf[5], recv_buf[6], recv_buf[7],
        );

        // Compute how many bytes were actually received.
        let remaining = ehci::qtd_token_bytes_remaining(token) as usize;
        let received = (self.max_packet_size as usize).saturating_sub(remaining);
        let copy_len = received.min(64);

        // Build the InterruptPacket.
        let mut packet = InterruptPacket::new();
        packet.address = self.address;
        packet.endpoint = self.endpoint;
        packet.size = copy_len as u8;
        packet.data[..copy_len].copy_from_slice(&recv_buf[..copy_len]);

        // --- Re-arm the qTD for the next poll ---
        //
        // Fully re-initialise the qTD (token, buffer pointers, next/alt_next).
        // This is necessary because the EHCI controller writes back the overlay
        // area to the original qTD after completion, which advances buffer[0]'s
        // current-offset field by the number of bytes received. If we only reset
        // the token, subsequent transfers write to advancing addresses while we
        // always read from the original recv_buf base. See phase2b_debugging.md.
        //
        // The data toggle is managed by the controller in the QH overlay (DTC=0),
        // so we do NOT write overlay_token here — `reattach_qtd_preserve_toggle`
        // only updates overlay_next, leaving the controller-managed toggle intact.
        let rearm_token = ehci::qtd_token(PID_IN, self.max_packet_size as u32, false, true);
        let rearm_qtd = self.qtd_slot.ptr();
        let qh_ptr = self.statics.qh_ptr(self.qh_index);

        // SAFETY: qTD pointer from QtdSlot (exclusively owned); QH pointer from
        // static pool (this pipe exclusively uses this QH index).  Reinitialising
        // the qTD and reattaching it re-arms the interrupt poll for the next frame.
        // reattach_qtd_preserve_toggle only writes overlay_next, preserving the
        // controller-managed data toggle in the overlay token.
        unsafe {
            (*rearm_qtd).init(rearm_token, recv_buf.as_ptr(), self.max_packet_size as u32);
            (*qh_ptr).reattach_qtd_preserve_toggle(rearm_qtd);
        }

        // Re-enable transfer interrupts so the next completion wakes us.
        ral::modify_reg!(ral::usb, usb_inst, USBINTR, UE: 1, UEE: 1, UAIE: 1, UPIE: 1);

        Poll::Ready(Some(packet))
    }
}

impl Drop for ImxrtInterruptPipe {
    fn drop(&mut self) {
        // 1. Remove the QH from the periodic frame list.
        let qh_ptr = self.statics.qh_ptr(self.qh_index);
        // SAFETY: QH pointer from static pool.  unlink_qh_from_periodic_schedule
        // removes all references to this QH from the frame list and any
        // predecessor QH's horizontal_link.
        unsafe {
            ImxrtHostController::unlink_qh_from_periodic_schedule(self.statics, qh_ptr);
        }

        // 2. Wait ≥1 ms for the controller to cross a frame boundary.
        //
        // After unlinking, the controller may complete an in-progress access
        // to this QH for the current frame. A ~1 ms busy-wait (one EHCI frame
        // at full speed = 1 ms) ensures no further DMA accesses will occur
        // before we release the memory.
        //
        // Note: cortex_m::asm::delay may complete in half the expected time
        // on Cortex-M7 due to the dual-issue pipeline, so we use 2× the
        // nominal cycle count to ensure we meet the minimum delay.
        // See: https://github.com/rust-embedded/cortex-m/issues/430
        #[cfg(target_os = "none")]
        cortex_m::asm::delay(1_200_000); // ≥1 ms at 600 MHz (2× for Cortex-M7)

        // 3. qTD cleanup is handled automatically by QtdSlot::drop() when
        //    `self.qtd_slot` is dropped after this Drop body finishes.

        // 4. Mark the QH as unused (cleared on next init_endpoint() call too,
        //    but explicit clear guards against stale flag reads).
        // SAFETY: QH pointer from static pool; QH unlinked above; 1 ms delay
        // ensures no further DMA accesses.  Safe to write sw_flags.
        unsafe { (*qh_ptr).sw_flags.write(0) };

        // 5. Field destructors run after this body:
        //    - `self.qtd_slot` drops → frees qTD allocation + zeroes hardware fields
        //    - `self.pipe` drops → returns the bulk_pipes pool slot
    }
}
