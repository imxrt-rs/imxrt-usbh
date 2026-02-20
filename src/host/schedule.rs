//! QH/qTD pool allocation, RAII guards, and EHCI schedule management.
//!
//! This module contains the low-level helpers for allocating QHs and qTDs
//! from the static pools, linking/unlinking them from the EHCI async
//! and periodic schedules, and safe wrappers that combine link+cache
//! operations.

use crate::ehci::{
    self, link_pointer, link_type, QueueHead, TransferDescriptor,
    LINK_TERMINATE,
};
use crate::ral;

use super::controller::Imxrt1062HostController;
use super::futures::AsyncAdvanceWait;
use super::statics::UsbStatics;
use super::{NUM_QH, NUM_QTD};

// ---------------------------------------------------------------------------
// QtdSlot — RAII guard for qTD pool allocation
// ---------------------------------------------------------------------------

/// RAII guard that owns a single qTD slot from the pool.
///
/// When dropped, the qTD is freed: the allocation bitmap flag is cleared and
/// all hardware fields are zeroed so the slot is ready for reuse.  This
/// replaces the manual `free_qtd()` calls and eliminates unsafe cleanup on
/// error paths — when an allocation fails partway through, the already-
/// allocated guards drop automatically.
///
/// # Typical usage
///
/// ```ignore
/// let slot = hc.alloc_qtd().ok_or(UsbError::AllPipesInUse)?;
/// let ptr = slot.ptr();
/// unsafe { (*ptr).init(token, buffer, len) };
/// // ... use the qTD ...
/// // slot drops here → qTD freed automatically
/// ```
pub(super) struct QtdSlot {
    index: usize,
    statics: &'static UsbStatics,
}

impl QtdSlot {
    /// Get the pool index of this qTD slot.
    #[inline]
    pub(super) fn index(&self) -> usize {
        self.index
    }

    /// Get a mutable pointer to the qTD in the pool.
    #[inline]
    pub(super) fn ptr(&self) -> *mut TransferDescriptor {
        self.statics.qtd_ptr(self.index)
    }
}

impl Drop for QtdSlot {
    fn drop(&mut self) {
        self.statics.qtd_allocated[self.index].set(false);
        // SAFETY: `qtd_ptr(index)` returns a valid, aligned `*mut TransferDescriptor`
        // from the static pool.  This slot was exclusively owned by this guard
        // (enforced by the allocation bitmap), so no aliasing access exists.
        // Zeroing the hardware fields prevents stale DMA descriptors.
        unsafe {
            let qtd = self.statics.qtd_ptr(self.index);
            (*qtd).next.write(LINK_TERMINATE);
            (*qtd).alt_next.write(LINK_TERMINATE);
            (*qtd).token.write(0);
            for buf in &mut (*qtd).buffer {
                buf.write(0);
            }
        }
    }
}

impl Imxrt1062HostController {
    // -----------------------------------------------------------------------
    // QH / qTD allocation helpers
    // -----------------------------------------------------------------------

    /// Get a mutable pointer to a QH from the pool by index.
    ///
    /// Returns `*mut QueueHead` via `UnsafeCell::get()`.  Index 0 is reserved
    /// for the async schedule sentinel.
    ///
    /// # Safety
    /// The caller must ensure no aliasing `&mut QueueHead` exists for the
    /// same index.
    pub(super) unsafe fn qh_mut(&self, index: usize) -> *mut QueueHead {
        self.statics.qh_ptr(index)
    }

    /// Find a free qTD slot and return an RAII guard that owns it.
    ///
    /// Returns `None` if all slots are in use.  On success, the returned
    /// `QtdSlot` will automatically free the slot when dropped.
    pub(super) fn alloc_qtd(&self) -> Option<QtdSlot> {
        for i in 0..NUM_QTD {
            if !self.statics.qtd_allocated[i].get() {
                self.statics.qtd_allocated[i].set(true);
                return Some(QtdSlot {
                    index: i,
                    statics: self.statics,
                });
            }
        }
        None
    }

    // -----------------------------------------------------------------------
    // Async schedule management
    // -----------------------------------------------------------------------

    /// Link a QH into the async schedule (after the sentinel at index 0).
    ///
    /// # Safety
    /// - The QH must be fully initialized.
    /// - Cache must be cleaned after this call.
    pub(super) unsafe fn link_qh_to_async_schedule(&self, qh: *mut QueueHead) {
        // SAFETY: Both `qh` and the sentinel (index 0) are valid pointers from
        // the static QH pool.  The sentinel is always in the async schedule and
        // is never freed.  The caller guarantees `qh` is fully initialized.
        unsafe {
            let sentinel = self.qh_mut(0);

            // new_qh → sentinel's old successor
            (*qh).horizontal_link.write((*sentinel).horizontal_link.read());

            // sentinel → new_qh
            let qh_addr = qh as u32;
            (*sentinel)
                .horizontal_link
                .write(link_pointer(qh_addr, link_type::QH));
        }
    }

    /// Unlink a QH from the async schedule.
    ///
    /// Finds the QH that points to `qh` and updates its horizontal_link
    /// to skip over `qh`.
    ///
    /// # Safety
    /// - The QH must be in the async schedule.
    pub(super) unsafe fn unlink_qh_from_async_schedule(&self, qh: *mut QueueHead) {
        // SAFETY: All QH pointers in the circular list are from the static pool
        // and remain valid for `'static`.  The walk is bounded by the circular
        // structure (terminates when we reach sentinel again).  The caller
        // guarantees `qh` is currently in the async schedule.
        unsafe {
            let qh_addr = qh as u32;
            let sentinel = self.qh_mut(0);

            // Walk the circular list starting from sentinel to find the predecessor
            let mut prev = sentinel;
            loop {
                let next_link = (*prev).horizontal_link.read();
                let next_addr = ehci::link_address(next_link);
                if next_addr == (qh_addr & !0x1F) {
                    // Found it — point prev around qh
                    (*prev)
                        .horizontal_link
                        .write((*qh).horizontal_link.read());
                    break;
                }
                prev = next_addr as *mut QueueHead;

                // Safety: if we wrap around to sentinel without finding qh, the QH
                // wasn't in the list — this shouldn't happen if called correctly.
                if prev == sentinel {
                    break;
                }
            }
        }
    }

    // -----------------------------------------------------------------------
    // Periodic schedule management
    // -----------------------------------------------------------------------

    /// Link a QH into the periodic schedule.
    ///
    /// Inserts `qh` at the head of the chain: all 32 frame list entries are
    /// updated to point to `qh`, and `qh.horizontal_link` is set to whatever
    /// the frame list entries previously pointed to (the old head, or TERMINATE).
    ///
    /// # Safety
    /// - The QH must be fully initialized (characteristics, capabilities, qTD attached).
    /// - Cache must be cleaned after this call.
    pub(super) unsafe fn link_qh_to_periodic_schedule(&self, qh: *mut QueueHead) {
        // SAFETY: `qh` is a valid pointer from the static QH pool.  Frame list
        // entries are stable `VCell<u32>` values in the static `FrameList`.  The
        // caller guarantees the QH is fully initialized with qTD attached.
        unsafe {
            // Read the current head from the first entry (all entries are kept in sync)
            let old_head = self.statics.frame_list.entries[0].read();
            // New QH → old head (or TERMINATE if list was empty)
            (*qh).horizontal_link.write(old_head);
            // All frame list entries → new QH
            let new_link = ehci::link_pointer(qh as u32, ehci::link_type::QH);
            for entry in &self.statics.frame_list.entries {
                entry.write(new_link);
            }
        }
    }

    /// Unlink a QH from the periodic schedule.
    ///
    /// Finds all references to `qh` — either directly in the frame list entries
    /// or via the `horizontal_link` of a predecessor QH — and replaces them with
    /// `qh`'s own `horizontal_link` (its successor, or TERMINATE).
    ///
    /// # Safety
    /// - `qh` must currently be in the periodic schedule.
    /// - Cache must be cleaned after this call.
    pub(super) unsafe fn unlink_qh_from_periodic_schedule(
        statics: &'static UsbStatics,
        qh: *const QueueHead,
    ) {
        // SAFETY: `qh` is a valid pointer from the static QH pool.  Frame list
        // entries and predecessor QHs are also from static storage.  The chain
        // walk is bounded to `NUM_QH` steps to prevent infinite loops from
        // corrupted link values.
        unsafe {
            let target_addr = ehci::link_address(qh as u32);
            let successor = (*qh).horizontal_link.read();

            // Update any frame list entries that point directly to this QH (head case).
            for entry in &statics.frame_list.entries {
                if ehci::link_address(entry.read()) == target_addr {
                    entry.write(successor);
                }
            }

            // Walk the chain from the (possibly updated) first frame list entry to find
            // any QH whose horizontal_link points to the target (mid-chain removal).
            let head_link = statics.frame_list.entries[0].read();
            if !ehci::link_is_terminate(head_link) {
                let mut prev = ehci::link_address(head_link) as *mut QueueHead;
                // Bound the walk to at most NUM_QH steps to guard against corruption.
                for _ in 0..NUM_QH {
                    let next_link = (*prev).horizontal_link.read();
                    if ehci::link_is_terminate(next_link) {
                        break;
                    }
                    if ehci::link_address(next_link) == target_addr {
                        (*prev).horizontal_link.write(successor);
                        break;
                    }
                    prev = ehci::link_address(next_link) as *mut QueueHead;
                }
            }
        }
    }

    // -----------------------------------------------------------------------
    // Async schedule control
    // -----------------------------------------------------------------------

    /// Enable the async schedule if not already enabled.
    pub(super) fn enable_async_schedule(&self) {
        let cmd = ral::read_reg!(ral::usb, self.usb, USBCMD);
        if cmd & (1 << 5) == 0 {
            // ASE bit 5
            ral::modify_reg!(ral::usb, self.usb, USBCMD, |v| v | (1 << 5));
        }
    }

    /// Ring the async advance doorbell and wait for acknowledgement.
    ///
    /// This must be called after unlinking a QH from the async schedule
    /// to ensure the controller is no longer accessing it before freeing.
    pub(super) async fn wait_async_advance(&self) {
        // Register waker before ringing doorbell to avoid race
        let waker_future = AsyncAdvanceWait {
            usb: &self.usb,
            shared: self.shared,
        };
        // Set IAA (Interrupt on Async Advance) bit in USBCMD
        ral::modify_reg!(ral::usb, self.usb, USBCMD, |v| v | (1 << 6));
        waker_future.await;
    }

    // -----------------------------------------------------------------------
    // Combined link + cache + enable helpers
    // -----------------------------------------------------------------------

    /// Link a QH into the async schedule with full cache maintenance.
    ///
    /// Cleans both the QH and sentinel before and after linking, then
    /// enables the async schedule.  This replaces the repeated 7-line
    /// link→clean→clean→enable sequence in control and bulk transfers.
    ///
    /// # Safety
    /// The QH and its attached qTD chain must be fully initialized and
    /// cache-cleaned before calling this.
    pub(super) unsafe fn link_and_start_async(&self, qh: *mut QueueHead) {
        // SAFETY: `qh` and sentinel are valid pool pointers.  Cache maintenance
        // is applied before and after linking so the DMA engine always sees
        // coherent descriptor memory.  The caller guarantees the QH and its
        // qTD chain are fully initialized and cache-cleaned.
        unsafe {
            let sentinel = self.qh_mut(0);
            Self::cache_clean_qh(qh);
            Self::cache_clean_qh(sentinel);

            self.link_qh_to_async_schedule(qh);

            // Re-clean after linking (horizontal_links of both QH and sentinel changed)
            Self::cache_clean_qh(qh);
            Self::cache_clean_qh(sentinel);
        }

        self.enable_async_schedule();
    }

    /// Unlink a QH from the async schedule and wait for hardware acknowledgement.
    ///
    /// Unlinks the QH, cleans the sentinel, rings the async advance doorbell,
    /// and waits for the controller to acknowledge it is no longer accessing
    /// the QH.  This replaces the repeated unlink→clean→doorbell→await
    /// sequence in control and bulk transfers.
    ///
    /// # Safety
    /// The QH must currently be in the async schedule.
    pub(super) async unsafe fn unlink_and_stop_async(&self, qh: *mut QueueHead) {
        // SAFETY: `qh` is in the async schedule (linked by a prior
        // `link_and_start_async`).  Sentinel is cleaned after unlinking so the
        // DMA engine sees the updated circular list.  The doorbell+await
        // sequence ensures the controller has stopped accessing `qh` before
        // the caller frees it.
        unsafe {
            let sentinel = self.qh_mut(0);
            self.unlink_qh_from_async_schedule(qh);
            Self::cache_clean_qh(sentinel);
        }

        self.wait_async_advance().await;
    }
}
