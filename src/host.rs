//! USB host controller driver for i.MX RT EHCI.
//!
//! This module contains the core types for the USB host controller:
//!
//! - [`UsbShared`] — interrupt-safe state shared between ISR and async tasks.
//! - [`UsbStatics`] — static-lifetime resource pools (not shared with ISR).
//! - [`Imxrt1062HostController`] — the main controller implementing
//!   [`HostController`](cotton_usb_host::host_controller::HostController).
//!
//! # Architecture
//!
//! The design follows the pattern established by the RP2040 host controller in
//! `cotton-usb-host`:
//!
//! ```text
//!   static UsbShared      ←──── ISR calls on_irq(), wakes pipe/device wakers
//!   static UsbStatics     ←──── Pool-based pipe allocation (not ISR-accessed)
//!   Imxrt1062HostController ──→ owns register blocks, references shared/statics
//! ```
//!
//! Both `UsbShared` and `UsbStatics` are `const`-constructible and designed to
//! live in `static` storage (typically via `ConstStaticCell`).

use crate::ehci::{FrameList, QueueHead, TransferDescriptor};
use crate::ral;
use cotton_usb_host::async_pool::Pool;
use rtic_common::waker_registration::CriticalSectionWakerRegistration;

// ---------------------------------------------------------------------------
// Pool sizing constants
// ---------------------------------------------------------------------------

/// Number of QH slots available for endpoint pipes.
///
/// 1 is reserved for control (EP0), the rest for bulk/interrupt endpoints.
/// With 4 QHs: 1 control + 3 concurrent bulk/interrupt pipes.
pub const NUM_QH: usize = 4;

/// Number of qTD slots available for transfers.
///
/// Each control transfer uses 2–3 qTDs (setup + data + status).
/// Each bulk/interrupt transfer uses 1–N qTDs.
/// 16 qTDs supports 1 control + 3 concurrent bulk, with room for chaining.
pub const NUM_QTD: usize = 16;

/// Number of pipe wakers.
///
/// Must be ≥ `NUM_QH + 1` (control pipe at index 0, bulk/interrupt at indices 1..N).
/// We use NUM_QH + 1 to match: 1 control + NUM_QH bulk/interrupt slots.
const NUM_PIPE_WAKERS: usize = NUM_QH + 1;

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

impl UsbShared {
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

        // Acknowledge all pending status bits (W1C)
        ral::write_reg!(ral::usb, usb, USBSTS, status);

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

    /// Get a reference to the device waker (for registering in DeviceDetect stream).
    pub fn device_waker(&self) -> &CriticalSectionWakerRegistration {
        &self.device_waker
    }

    /// Get a reference to a pipe waker by index.
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

// ---------------------------------------------------------------------------
// UsbStatics — static resource pools
// ---------------------------------------------------------------------------

/// Static-lifetime resource pools for the USB host controller.
///
/// This struct owns the pre-allocated pools of QHs, qTDs, and the periodic
/// frame list. It is **not** accessed from the ISR — only from async task
/// context.
///
/// # Placement
///
/// Must live in a `static` (typically via `ConstStaticCell`) because:
/// - Pipe allocations (`Pooled`) borrow the pool with `'static` lifetime
/// - DMA structures must have stable addresses for the controller
///
/// # Memory
///
/// All DMA-visible arrays (`qh_pool`, `qtd_pool`, `frame_list`) must be in
/// normal RAM (not TCM) if using cache management, or in DTCM if bypassing
/// the data cache.
pub struct UsbStatics {
    /// Pool for control pipe slots (1 slot — only one EP0 at a time).
    pub control_pipes: Pool,

    /// Pool for bulk/interrupt pipe slots.
    pub bulk_pipes: Pool,

    /// Pre-allocated Queue Head storage.
    ///
    /// Index 0 is reserved for the async schedule sentinel.
    /// Indices 1..NUM_QH are for endpoint pipes.
    pub qh_pool: [QueueHead; NUM_QH + 1], // +1 for sentinel

    /// Pre-allocated Transfer Descriptor storage.
    pub qtd_pool: [TransferDescriptor; NUM_QTD],

    /// Periodic frame list (4096-byte aligned).
    pub frame_list: FrameList,
}

impl UsbStatics {
    /// Create a new `UsbStatics` with all resources free and structures zeroed.
    ///
    /// This is `const` so it can be placed in a `static`.
    pub const fn new() -> Self {
        Self {
            control_pipes: Pool::new(1),
            bulk_pipes: Pool::new(NUM_QH as u8),
            qh_pool: {
                const QH: QueueHead = QueueHead::new();
                [QH; NUM_QH + 1]
            },
            qtd_pool: {
                const QTD: TransferDescriptor = TransferDescriptor::new();
                [QTD; NUM_QTD]
            },
            frame_list: FrameList::new(),
        }
    }
}

// ---------------------------------------------------------------------------
// Imxrt1062HostController
// ---------------------------------------------------------------------------

/// USB host controller for i.MX RT 1062.
///
/// This is the main driver type that implements the
/// [`HostController`](cotton_usb_host::host_controller::HostController) trait.
/// It owns the USB register blocks and holds references to the shared and
/// static resources.
///
/// # Construction
///
/// ```ignore
/// use imxrt_usbh::host::{Imxrt1062HostController, UsbShared, UsbStatics};
///
/// static SHARED: UsbShared = UsbShared::new();
/// static STATICS: StaticCell<UsbStatics> = StaticCell::new();
///
/// let statics = STATICS.init(UsbStatics::new());
/// let host = Imxrt1062HostController::new(peripherals, &SHARED, statics);
/// ```
pub struct Imxrt1062HostController {
    /// USB OTG core registers (owned).
    usb: ral::usb::Instance,

    /// USB PHY registers (owned).
    usbphy: ral::usbphy::Instance,

    /// Interrupt-safe shared state (borrowed, lives in a static).
    shared: &'static UsbShared,

    /// Resource pools and DMA structures (borrowed, lives in a static).
    statics: &'static UsbStatics,
}

impl Imxrt1062HostController {
    /// Create a new host controller from peripheral instances and static resources.
    ///
    /// # Arguments
    ///
    /// - `peripherals` — implementation of [`Peripherals`](crate::Peripherals) providing
    ///   register block pointers
    /// - `shared` — reference to [`UsbShared`] in a `static`
    /// - `statics` — reference to [`UsbStatics`] in a `static`
    ///
    /// # Note
    ///
    /// This does **not** initialise the hardware. Call `init()` (phase 1.3)
    /// after construction to set up the controller.
    pub fn new<P: crate::Peripherals>(
        peripherals: P,
        shared: &'static UsbShared,
        statics: &'static UsbStatics,
    ) -> Self {
        let instances = ral::instances(peripherals);
        Self {
            usb: instances.usb,
            usbphy: instances.usbphy,
            shared,
            statics,
        }
    }

    /// Get a reference to the USB register block.
    pub(crate) fn usb(&self) -> &ral::usb::Instance {
        &self.usb
    }

    /// Get a mutable reference to the USB register block.
    pub(crate) fn usb_mut(&mut self) -> &mut ral::usb::Instance {
        &mut self.usb
    }

    /// Get a reference to the shared state.
    pub(crate) fn shared(&self) -> &'static UsbShared {
        self.shared
    }

    /// Get a reference to the static resources.
    pub(crate) fn statics(&self) -> &'static UsbStatics {
        self.statics
    }
}
