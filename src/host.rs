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

    // -----------------------------------------------------------------------
    // Initialization
    // -----------------------------------------------------------------------

    /// Initialise the USB host controller hardware.
    ///
    /// This performs the full EHCI initialisation sequence for the i.MX RT 1062
    /// USB OTG2 controller in host mode. The sequence is derived from the NXP
    /// reference manual, USBHost_t36, and TinyUSB, adapted for this crate's
    /// EHCI data structures.
    ///
    /// # Prerequisites
    ///
    /// The caller (BSP / board crate) **must** have already:
    /// 1. Enabled the USB2 PLL (`CCM_ANALOG_PLL_USB2`) and waited for lock.
    /// 2. Un-gated the USB OTG2 clock (`CCM_CCGR6_USBOH3`).
    /// 3. Configured the VBUS power GPIO (Teensy 4.1: `GPIO_EMC_40`).
    ///
    /// # Sequence
    ///
    /// 1. Reset & power-on USB PHY
    /// 2. Reset USB controller
    /// 3. Set host mode (must be immediately after reset — NXP errata)
    /// 4. Configure system bus interface (SBUSCFG)
    /// 5. Initialise async schedule (sentinel QH → ASYNCLISTADDR)
    /// 6. Initialise periodic schedule (frame list → PERIODICLISTBASE)
    /// 7. Disable all interrupts, clear pending status
    /// 8. Configure USBCMD (ITC, frame list size, async park, periodic schedule)
    /// 9. Enable controller (Run)
    /// 10. Enable port power
    /// 11. Enable host disconnect detection in PHY
    /// 12. Enable interrupts
    ///
    /// # Safety
    ///
    /// - Must be called exactly once after construction.
    /// - The `UsbStatics` referenced by this controller must be in a `static`
    ///   with stable addresses (required for DMA).
    /// - Interrupts for USB OTG2 should be disabled (or not yet enabled in the
    ///   NVIC) when this is called.
    pub unsafe fn init(&mut self) {
        // ---- Step 1: Reset and power-on USB PHY ----
        //
        // Assert soft-reset (clears PWD, TX, RX, CTRL to defaults).
        ral::write_reg!(ral::usbphy, self.usbphy, CTRL_SET, SFTRST: 1);

        // De-assert soft-reset and un-gate UTMI clocks.
        // Using CTRL_CLR (write-1-to-clear) avoids a read-modify-write race.
        ral::write_reg!(ral::usbphy, self.usbphy, CTRL_CLR, SFTRST: 1, CLKGATE: 1);

        // Enable UTMI+ Level 2 and Level 3 (required for low-speed device support
        // through a high-speed hub, per USB 2.0 §11.8).
        ral::write_reg!(ral::usbphy, self.usbphy, CTRL_SET, ENUTMILEVEL2: 1, ENUTMILEVEL3: 1);

        // Power up the PHY — writing 0 clears all power-down bits in PWD.
        ral::write_reg!(ral::usbphy, self.usbphy, PWD, 0);

        // ---- Step 2: Reset USB controller ----
        //
        // Assert controller reset. RST is self-clearing.
        ral::modify_reg!(ral::usb, self.usb, USBCMD, |cmd| cmd | (1 << 1));

        // Spin until the controller completes reset (RST self-clears to 0).
        while ral::read_reg!(ral::usb, self.usb, USBCMD, RST == 1) {}

        // ---- Step 3: Set host mode (immediately after reset) ----
        //
        // Per NXP errata, USBMODE must be written immediately after the controller
        // reset completes, before any other USBCMD writes. CM=0b11 = Host Controller.
        ral::write_reg!(ral::usb, self.usb, USBMODE, CM: 0b11);

        // ---- Step 4: Configure system bus interface ----
        //
        // SBUSCFG = INCR4 burst then single transfer. This matches USBHost_t36's
        // SBUSCFG=1 setting and provides good AHB bus utilisation.
        ral::write_reg!(ral::usb, self.usb, SBUSCFG, AHBBRST: 0b001);

        // ---- Step 5: Initialise async schedule ----
        //
        // Set up the sentinel QH (index 0) as a self-referencing circular list.
        // This is the idle state of the async schedule per EHCI §4.8.
        let sentinel = &self.statics.qh_pool[0] as *const _ as *mut crate::ehci::QueueHead;
        unsafe { (*sentinel).init_sentinel() };

        // Write the sentinel's physical address to ASYNCLISTADDR.
        let sentinel_addr = sentinel as u32;
        ral::write_reg!(ral::usb, self.usb, ASYNCLISTADDR, sentinel_addr);

        // ---- Step 6: Initialise periodic schedule ----
        //
        // The frame list is already zeroed (all entries = terminate) from
        // FrameList::new(). Write the frame list base address.
        //
        // DEVICEADDR and PERIODICLISTBASE share the same register offset.
        // In host mode, bits 31:12 (BASEADR) hold the periodic frame list
        // base address.
        let frame_list_addr = &self.statics.frame_list as *const _ as u32;
        ral::write_reg!(ral::usb, self.usb, DEVICEADDR, frame_list_addr);

        // Reset the frame index to 0.
        ral::write_reg!(ral::usb, self.usb, FRINDEX, 0);

        // ---- Step 7: Disable interrupts and clear pending status ----
        //
        // Ensure no spurious interrupts fire during init.
        ral::write_reg!(ral::usb, self.usb, USBINTR, 0);

        // Clear all pending status bits by reading and writing back (W1C).
        let status = ral::read_reg!(ral::usb, self.usb, USBSTS);
        ral::write_reg!(ral::usb, self.usb, USBSTS, status);

        // ---- Step 8: Configure and start the controller (USBCMD) ----
        //
        // Build the USBCMD value:
        //
        //   ITC  = 1 micro-frame (125μs interrupt coalescing)
        //   FS   = 32-entry frame list (FS_2=1, FS_1=0b01)
        //   PSE  = 1 (enable periodic schedule)
        //   ASP  = 0b11 (async schedule park count = 3 — max service transactions)
        //   ASPE = 1 (enable async schedule park mode)
        //   RS   = 1 (run)
        //
        // Frame list size encoding for 32 entries:
        //   FS[2:0] = 0b101 → FS_2 (bit 15) = 1, FS_1 (bits 3:2) = 0b01
        //
        // Note: ASE (async schedule enable) is NOT set here. It will be enabled
        // when the first endpoint pipe is added in phase 2, because running the
        // async schedule with only a sentinel QH wastes bus bandwidth.
        let usbcmd: u32 = (1 << 0)    // RS: Run
            | (0b01 << 2)             // FS_1: frame list size low bits
            | (1 << 4)               // PSE: periodic schedule enable
            | (0b11 << 8)            // ASP: async park count = 3
            | (1 << 11)              // ASPE: async park mode enable
            | (1 << 15)              // FS_2: frame list size high bit
            | (1 << 16);             // ITC: 1 micro-frame threshold
        ral::write_reg!(ral::usb, self.usb, USBCMD, usbcmd);

        // ---- Step 9: Enable port power ----
        //
        // PP (Port Power) must be set for the root port to supply power.
        // Use modify to preserve other PORTSC1 bits.
        ral::modify_reg!(ral::usb, self.usb, PORTSC1, PP: 1);

        // ---- Step 10: Enable host disconnect detection in PHY ----
        //
        // ENHOSTDISCONDETECT enables the PHY's high-speed disconnect detector,
        // which is needed for the host to detect when a device is unplugged.
        ral::write_reg!(ral::usbphy, self.usbphy, CTRL_SET, ENHOSTDISCONDETECT: 1);

        // ---- Step 11: Enable interrupts ----
        //
        // Enable the interrupt sources we care about:
        //   PCE  (bit 2)  — Port Change Detect (connect/disconnect)
        //   UE   (bit 0)  — USB Interrupt (transfer complete)
        //   UEE  (bit 1)  — USB Error Interrupt (transfer error)
        //   AAE  (bit 5)  — Async Advance (QH removal doorbell)
        //   SEE  (bit 4)  — System Error (AHB bus error — should never happen)
        //   UAIE (bit 18) — NXP async completion (finer-grained than UE)
        //   UPIE (bit 19) — NXP periodic completion (finer-grained than UE)
        //
        // GP Timer interrupts (TIE0, TIE1) are NOT enabled here — they will be
        // enabled on-demand when timers are started (e.g. port debounce in phase 2).
        //
        // The NVIC interrupt for USB_OTG2 (IRQ #112) must be enabled separately
        // by the caller (typically via RTIC or cortex_m::peripheral::NVIC).
        ral::write_reg!(ral::usb, self.usb, USBINTR,
            UE: 1,    // USB Interrupt Enable
            UEE: 1,   // USB Error Interrupt Enable
            PCE: 1,   // Port Change Detect Enable
            SEE: 1,   // System Error Enable
            AAE: 1,   // Async Advance Enable
            UAIE: 1,  // USB Host Async Interrupt Enable (NXP)
            UPIE: 1   // USB Host Periodic Interrupt Enable (NXP)
        );
    }
}
