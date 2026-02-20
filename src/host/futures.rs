//! Async futures for EHCI transfer completion and async advance doorbell.

use crate::ehci::{self, QTD_TOKEN_ACTIVE, QTD_TOKEN_HALTED};
use crate::ral;
use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};

use super::controller::Imxrt1062HostController;
use super::shared::UsbShared;
use super::statics::UsbStatics;

// ---------------------------------------------------------------------------
// TransferComplete — future that waits for a qTD chain to complete
// ---------------------------------------------------------------------------

/// Future that polls an EHCI qTD for completion.
///
/// Checks the status qTD's Active bit. When cleared by the controller,
/// the transfer is complete. Error bits are mapped to `UsbError`.
///
/// Stores pool indices rather than raw pointers so that the struct is `Send`
/// without a manual `unsafe impl`.
pub(super) struct TransferComplete<'a> {
    pub(super) usb: &'a ral::usb::Instance,
    pub(super) shared: &'a UsbShared,
    pub(super) statics: &'a UsbStatics,
    /// Index into `statics.qtd_pool` for the status (IOC) qTD.
    pub(super) status_qtd_index: usize,
    /// Optional index into `statics.qtd_pool` for the data qTD.
    pub(super) data_qtd_index: Option<usize>,
    /// Index into `statics.qh_pool` for the QH.
    pub(super) qh_index: usize,
    /// Index into `pipe_wakers` to register with. 0 = control pipe; 1..N = bulk/interrupt.
    pub(super) waker_index: usize,
}

impl Future for TransferComplete<'_> {
    type Output = Result<(), cotton_usb_host::host_controller::UsbError>;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        // Register waker with the appropriate pipe waker slot.
        self.shared.pipe_waker(self.waker_index).register(cx.waker());

        // Derive pointers from pool indices for cache maintenance + reads.
        let status_qtd_ptr = self.statics.qtd_ptr(self.status_qtd_index);
        let data_qtd_ptr = self.data_qtd_index.map(|i| self.statics.qtd_ptr(i));
        let qh_ptr = self.statics.qh_ptr(self.qh_index);

        // Invalidate cache to see hardware updates
        Imxrt1062HostController::cache_clean_qtd(status_qtd_ptr);
        if let Some(dp) = data_qtd_ptr {
            Imxrt1062HostController::cache_clean_qtd(dp);
        }
        Imxrt1062HostController::cache_clean_qh(qh_ptr);

        // SAFETY: All pointers derived from UsbStatics pool via qtd_ptr()/qh_ptr()
        // (valid, aligned, `'static` lifetime).  Cache was just invalidated above
        // so we read DMA-updated values.  Each index is exclusively owned by the
        // transfer that created this future.
        let token = unsafe { (*status_qtd_ptr).token.read() };

        if token & QTD_TOKEN_ACTIVE != 0 {
            // Still active — check if the QH overlay is halted.
            //
            // When the EHCI controller halts a qTD (e.g. setup phase got no
            // response from a disconnected device, CERR exhausted), it copies
            // the halted qTD's token into the QH overlay and stops advancing
            // the chain. The status_qtd (last in the chain) remains Active
            // because the controller never reached it.
            //
            // Without this check, TransferComplete hangs forever when the
            // setup qTD halts — this happens when cotton-usb-host tries a
            // control transfer to a hub that was just physically disconnected.
            // SAFETY: qh_ptr from statics pool, cache invalidated above.
            let overlay = unsafe { (*qh_ptr).overlay_token.read() };
            if overlay & QTD_TOKEN_HALTED != 0 {
                debug!("[HC] TransferComplete: QH overlay halted (overlay=0x{:08x}), aborting", overlay);
                return Poll::Ready(Err(Imxrt1062HostController::map_qtd_error(overlay)));
            }

            // Also check if data qTD has errored (early exit)
            if let Some(dp) = data_qtd_ptr {
                // SAFETY: data qTD pointer from statics pool, cache invalidated above.
                let data_token = unsafe { (*dp).token.read() };
                if data_token & QTD_TOKEN_HALTED != 0 {
                    // Data phase halted — map error
                    return Poll::Ready(Err(Imxrt1062HostController::map_qtd_error(
                        data_token,
                    )));
                }
            }

            // Re-enable transfer completion interrupts
            ral::modify_reg!(ral::usb, self.usb, USBINTR, |v| v
                | (1 << 0)   // UE
                | (1 << 1)   // UEE
                | (1 << 18)  // UAIE
                | (1 << 19)  // UPIE
            );
            return Poll::Pending;
        }

        // Transfer complete — check for errors
        if token & ehci::QTD_TOKEN_ERROR_MASK != 0 {
            return Poll::Ready(Err(Imxrt1062HostController::map_qtd_error(token)));
        }

        // Also check the data qTD if present
        if let Some(dp) = data_qtd_ptr {
            // SAFETY: data qTD pointer from statics pool, cache invalidated above.
            let data_token = unsafe { (*dp).token.read() };
            if data_token & ehci::QTD_TOKEN_ERROR_MASK != 0 {
                return Poll::Ready(Err(Imxrt1062HostController::map_qtd_error(
                    data_token,
                )));
            }
        }

        Poll::Ready(Ok(()))
    }
}


// ---------------------------------------------------------------------------
// AsyncAdvanceWait — future for async advance doorbell
// ---------------------------------------------------------------------------

/// Future that waits for the EHCI async advance doorbell to be acknowledged.
///
/// After unlinking a QH from the async schedule, the caller rings the doorbell
/// (sets USBCMD.IAA) and waits for USBSTS.AAI. This future polls for that.
pub(super) struct AsyncAdvanceWait<'a> {
    pub(super) usb: &'a ral::usb::Instance,
    pub(super) shared: &'a UsbShared,
}

impl Future for AsyncAdvanceWait<'_> {
    type Output = ();

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        self.shared.async_advance_waker().register(cx.waker());

        // Check if AAI (bit 5) is already set in USBSTS
        let status = ral::read_reg!(ral::usb, self.usb, USBSTS);
        if status & (1 << 5) != 0 {
            // Clear it (W1C)
            ral::write_reg!(ral::usb, self.usb, USBSTS, 1 << 5);
            return Poll::Ready(());
        }

        // Re-enable AAE interrupt
        ral::modify_reg!(ral::usb, self.usb, USBINTR, |v| v | (1 << 5));
        Poll::Pending
    }
}

// Safety: AsyncAdvanceWait holds references to static memory and register blocks.
unsafe impl Send for AsyncAdvanceWait<'_> {}
