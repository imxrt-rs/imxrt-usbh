//! Interrupt-safe shared state between the USB ISR and async tasks.

use crate::ral;
use rtic_common::waker_registration::CriticalSectionWakerRegistration;

use super::NUM_PIPE_WAKERS;

// ---------------------------------------------------------------------------
// UsbShared — ISR ↔ async task bridge
// ---------------------------------------------------------------------------

/// Interrupt-safe shared state between the USB ISR and async tasks.
///
/// This struct lives in a `static` and is accessed from both the
/// `USB_OTG2` interrupt handler (via [`on_irq`](Self::on_irq)) and
/// from async task context (via waker registration in `poll()` methods).
///
/// # Design
///
/// The ISR reads hardware interrupt status, wakes the appropriate wakers,
/// and **masks** the serviced interrupts to prevent IRQ storms. The async
/// poll functions re-enable interrupts before returning `Pending`.
///
/// This is the "disable-on-handle / re-enable-on-poll" pattern from the
/// RP2040 cotton-usb-host implementation.
pub struct UsbShared {
    /// Waker for the device-detect stream (port change events).
    device_waker: CriticalSectionWakerRegistration,

    /// Per-pipe wakers. Index 0 is the control pipe; indices 1..N are
    /// bulk/interrupt pipes. Woken by the ISR on transfer completion
    /// or error for the corresponding pipe.
    pipe_wakers: [CriticalSectionWakerRegistration; NUM_PIPE_WAKERS],

    /// Waker for async advance doorbell (used during QH removal).
    async_advance_waker: CriticalSectionWakerRegistration,
}

impl Default for UsbShared {
    fn default() -> Self {
        Self::new()
    }
}

impl UsbShared {
    #[allow(clippy::declare_interior_mutable_const)]
    const WAKER: CriticalSectionWakerRegistration = CriticalSectionWakerRegistration::new();

    /// Create a new `UsbShared` instance.
    ///
    /// All wakers are initially empty. This is `const` so it can be placed
    /// directly in a `static`.
    pub const fn new() -> Self {
        Self {
            device_waker: CriticalSectionWakerRegistration::new(),
            pipe_wakers: [Self::WAKER; NUM_PIPE_WAKERS],
            async_advance_waker: CriticalSectionWakerRegistration::new(),
        }
    }

    /// Called from the `USB_OTG2` interrupt handler.
    ///
    /// Reads USBSTS, wakes the appropriate wakers, and masks the serviced
    /// interrupts in USBINTR to prevent re-entry until the async task
    /// re-enables them.
    ///
    /// # Safety
    ///
    /// Must be called from interrupt context (or with interrupts disabled).
    /// The caller must provide a valid USB register instance.
    pub unsafe fn on_irq(&self, usb: &ral::usb::Instance) {
        // Read which interrupts fired
        let status = ral::read_reg!(ral::usb, usb, USBSTS);

        // Acknowledge pending status bits (W1C), but NOT AAI (bit 5).
        // AsyncAdvanceWait::poll() reads USBSTS.AAI directly to detect
        // completion — if we cleared it here, the poll function would
        // miss it and hang forever.
        ral::write_reg!(ral::usb, usb, USBSTS, status & !(1 << 5));

        // Port Change Interrupt — wake the device-detect stream
        if status & (1 << 2) != 0 {
            self.device_waker.wake();
        }

        // USB Interrupt (USBINT, bit 0) — async/periodic transfer completion.
        // USB Error Interrupt (USBERRINT, bit 1) — transfer error.
        // On the i.MX RT (NXP/ChipIdea), bits 18 (UAI) and 19 (UPI) provide
        // finer-grained async vs periodic completion, but we also check the
        // standard USBINT bit for compatibility.
        if status & ((1 << 0) | (1 << 1) | (1 << 18) | (1 << 19)) != 0 {
            // Wake all pipe wakers — the poll functions will check their
            // individual QH/qTD status to determine which pipe completed.
            //
            // TODO(phase 2): Use a per-pipe completion bitmap to wake only
            // the relevant pipe waker, avoiding unnecessary wakeups.
            for waker in &self.pipe_wakers {
                waker.wake();
            }
        }

        // Async Advance Interrupt (bit 5) — doorbell acknowledged,
        // safe to free unlinked QHs.
        if status & (1 << 5) != 0 {
            self.async_advance_waker.wake();
        }

        // Mask the interrupts we just serviced to prevent re-entry.
        // The async poll functions will re-enable them when they go Pending.
        let serviced = status & 0x000F_003F; // mask to defined interrupt bits
        ral::modify_reg!(ral::usb, usb, USBINTR, |intr| intr & !serviced);
    }

    /// Called from the `USB_OTG2` interrupt handler (public API).
    ///
    /// This is a convenience wrapper around [`on_irq`](Self::on_irq) that
    /// accepts a raw pointer to the USB OTG register block, avoiding the need
    /// to reference crate-internal RAL types from application code.
    ///
    /// # Arguments
    ///
    /// - `usb_base` — pointer to the USB OTG core register block. For USB2 on
    ///   the i.MX RT 1062, this is `0x402E_0200`.
    ///
    /// # Safety
    ///
    /// - Must be called from interrupt context (or with interrupts disabled).
    /// - `usb_base` must point to a valid USB OTG register block.
    pub unsafe fn on_usb_irq(&self, usb_base: *const ()) {
        let usb = ral::usb::Instance {
            addr: usb_base as *const ral::usb::RegisterBlock,
        };
        // Safety: caller guarantees usb_base points to a valid register block.
        unsafe { self.on_irq(&usb) };
    }

    /// Get a reference to the device waker (for registering in DeviceDetect stream).
    pub fn device_waker(&self) -> &CriticalSectionWakerRegistration {
        &self.device_waker
    }

    /// Get a reference to a pipe waker by index.
    ///
    /// # Panics
    ///
    /// Panics if `index >= NUM_PIPE_WAKERS` (currently 5).
    pub fn pipe_waker(&self, index: usize) -> &CriticalSectionWakerRegistration {
        &self.pipe_wakers[index]
    }

    /// Get a reference to the async advance waker.
    pub fn async_advance_waker(&self) -> &CriticalSectionWakerRegistration {
        &self.async_advance_waker
    }
}

// Safety: UsbShared is designed to be shared between ISR and task context.
// All fields use CriticalSectionWakerRegistration which is ISR-safe.
unsafe impl Sync for UsbShared {}
