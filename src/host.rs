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

use crate::cache;
use crate::ehci::{
    self, link_pointer, link_type, FrameList, QueueHead, TransferDescriptor, LINK_TERMINATE,
    PID_IN, PID_OUT, PID_SETUP, QTD_TOKEN_ACTIVE, QTD_TOKEN_BABBLE, QTD_TOKEN_BUFFER_ERR,
    QTD_TOKEN_HALTED, QTD_TOKEN_MISSED_UFRAME, QTD_TOKEN_XACT_ERR, SPEED_FULL, SPEED_HIGH,
    SPEED_LOW,
};
use crate::ral;
use core::cell::Cell;
use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};
use cotton_usb_host::async_pool::Pool;
use cotton_usb_host::host_controller::{
    DataPhase, DeviceStatus, HostController, InterruptPacket, TransferExtras, TransferType,
    UsbError, UsbSpeed,
};
use cotton_usb_host::wire::SetupPacket;
use futures::Stream;
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
            addr: usb_base.cast(),
        };
        // Safety: caller guarantees usb_base points to a valid register block.
        unsafe { self.on_irq(&usb) };
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
    /// Index 1 is reserved for the single control pipe (EP0).
    /// Indices 2..=NUM_QH are for bulk/interrupt pipes (NUM_QH−1 slots).
    pub qh_pool: [QueueHead; NUM_QH + 1], // +1 for sentinel

    /// Pre-allocated Transfer Descriptor storage.
    pub qtd_pool: [TransferDescriptor; NUM_QTD],

    /// Periodic frame list (4096-byte aligned).
    pub frame_list: FrameList,

    /// Receive buffers for interrupt pipes.
    ///
    /// One 64-byte buffer per interrupt pipe slot. These live in a `static`
    /// so their addresses are stable for the DMA engine. Index matches the
    /// `bulk_pipes` pool token (0..NUM_QH-2).
    pub recv_bufs: [[u8; 64]; NUM_QH - 1],
}

impl UsbStatics {
    /// Create a new `UsbStatics` with all resources free and structures zeroed.
    ///
    /// This is `const` so it can be placed in a `static`.
    pub const fn new() -> Self {
        Self {
            control_pipes: Pool::new(1),
            // NUM_QH - 1 slots: indices 2..=NUM_QH in qh_pool (index 0 = sentinel, 1 = control)
            bulk_pipes: Pool::new((NUM_QH - 1) as u8),
            qh_pool: {
                const QH: QueueHead = QueueHead::new();
                [QH; NUM_QH + 1]
            },
            qtd_pool: {
                const QTD: TransferDescriptor = TransferDescriptor::new();
                [QTD; NUM_QTD]
            },
            frame_list: FrameList::new(),
            recv_bufs: [[0u8; 64]; NUM_QH - 1],
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

// Safety: Imxrt1062HostController is designed for single-task usage.
// The raw pointer in ral::usb::Instance is stable (points to MMIO registers).
// &'static UsbStatics is safe to send because UsbStatics lives in a static and
// is only accessed from async task context (never from ISR).
// &'static UsbShared uses CriticalSection-based synchronization.
unsafe impl Send for Imxrt1062HostController {}

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

    // -----------------------------------------------------------------------
    // Port speed detection
    // -----------------------------------------------------------------------

    /// Read the currently detected port speed from PORTSC1.
    ///
    /// The port speed is available in PORTSC1[PSPD] (bits 27:26) after
    /// the port is enabled following a reset. Before reset, speed can be
    /// inferred from the line state.
    fn port_speed(&self) -> u32 {
        ral::read_reg!(ral::usb, self.usb, PORTSC1, PSPD)
    }

    /// Convert the PORTSC1 PSPD field to a `DeviceStatus`.
    fn device_status(&self) -> DeviceStatus {
        let portsc = ral::read_reg!(ral::usb, self.usb, PORTSC1);
        let connected = (portsc & 1) != 0; // CCS bit 0
        if connected {
            let pspd = (portsc >> 26) & 0x3;
            let speed = match pspd {
                0 => UsbSpeed::Full12,
                1 => UsbSpeed::Low1_5,
                2 => UsbSpeed::High480,
                _ => UsbSpeed::Full12, // shouldn't happen
            };
            DeviceStatus::Present(speed)
        } else {
            DeviceStatus::Absent
        }
    }

    // -----------------------------------------------------------------------
    // Re-enable port change interrupt
    // -----------------------------------------------------------------------

    /// Re-enable the port change interrupt (PCE, bit 2) in USBINTR.
    ///
    /// Called from poll functions after checking device status, following
    /// the disable-on-handle / re-enable-on-poll pattern.
    fn reenable_port_change_interrupt(&self) {
        ral::modify_reg!(ral::usb, self.usb, USBINTR, |v| v | (1 << 2));
    }

    /// Re-enable transfer completion interrupts in USBINTR.
    ///
    /// Re-enables: UE (bit 0), UEE (bit 1), UAIE (bit 18), UPIE (bit 19).
    fn reenable_transfer_interrupts(&self) {
        ral::modify_reg!(ral::usb, self.usb, USBINTR, |v| v
            | (1 << 0)   // UE
            | (1 << 1)   // UEE
            | (1 << 18)  // UAIE
            | (1 << 19)  // UPIE
        );
    }

    /// Re-enable the async advance interrupt (AAE, bit 5) in USBINTR.
    fn reenable_async_advance_interrupt(&self) {
        ral::modify_reg!(ral::usb, self.usb, USBINTR, |v| v | (1 << 5));
    }

    // -----------------------------------------------------------------------
    // PORTSC1 W1C-safe writes
    // -----------------------------------------------------------------------

    /// PORTSC1 write-1-to-clear bit mask.
    ///
    /// When doing a read-modify-write on PORTSC1, these bits must be masked
    /// off to avoid accidentally clearing them. Per EHCI spec, W1C bits:
    /// - CSC (bit 1) — Connect Status Change
    /// - PEC (bit 3) — Port Enable/Disable Change  (not in NXP, but safe)
    /// - OCC (bit 5) — Over-current Change
    /// - FPR (bit 6) — Force Port Resume (read as 0 when not suspended)
    const PORTSC1_W1C_MASK: u32 = (1 << 1) | (1 << 3) | (1 << 5) | (1 << 6);

    /// Read PORTSC1 with W1C bits cleared to prevent accidental clear.
    ///
    /// This should be used before any modify_reg! on PORTSC1 to ensure
    /// we don't accidentally write 1 to a W1C bit.
    fn portsc1_read_safe(&self) -> u32 {
        ral::read_reg!(ral::usb, self.usb, PORTSC1) & !Self::PORTSC1_W1C_MASK
    }

    // -----------------------------------------------------------------------
    // QH / qTD allocation helpers
    // -----------------------------------------------------------------------

    /// Allocate a QH from the pool by index.
    ///
    /// Returns a mutable pointer to the QH. Index 0 is reserved for sentinel.
    /// Caller must ensure the index is valid (1..=NUM_QH).
    ///
    /// # Safety
    /// The caller must ensure exclusive access to the QH at the given index.
    unsafe fn qh_mut(&self, index: usize) -> *mut QueueHead {
        &self.statics.qh_pool[index] as *const QueueHead as *mut QueueHead
    }

    /// Get a mutable pointer to a qTD from the pool by index.
    ///
    /// # Safety
    /// The caller must ensure exclusive access to the qTD at the given index.
    unsafe fn qtd_mut(&self, index: usize) -> *mut TransferDescriptor {
        &self.statics.qtd_pool[index] as *const TransferDescriptor as *mut TransferDescriptor
    }

    /// Find a free qTD slot in the pool.
    ///
    /// Scans the qtd_pool for an entry that doesn't have the Active bit set
    /// and isn't otherwise in use. Returns the index, or `None` if all slots
    /// are in use.
    fn alloc_qtd(&self) -> Option<usize> {
        for i in 0..NUM_QTD {
            let qtd = &self.statics.qtd_pool[i];
            // A free qTD will have token == 0 (not active or halted with no state)
            if qtd.token.read() == 0 && qtd.next.read() == LINK_TERMINATE {
                // Mark as allocated immediately to prevent double-allocation.
                // Without this, a second alloc_qtd() call before the first qTD
                // is init'd would return the same index.
                unsafe {
                    let qtd_ptr = self.qtd_mut(i);
                    (*qtd_ptr).token.write(ehci::QTD_TOKEN_ACTIVE);
                }
                return Some(i);
            }
        }
        None
    }

    /// Return a qTD to the pool (mark it as free).
    ///
    /// # Safety
    /// The caller must ensure the qTD is no longer referenced by any QH.
    unsafe fn free_qtd(&self, index: usize) {
        unsafe {
            let qtd = self.qtd_mut(index);
            (*qtd).next.write(LINK_TERMINATE);
            (*qtd).alt_next.write(LINK_TERMINATE);
            (*qtd).token.write(0);
            for buf in &mut (*qtd).buffer {
                buf.write(0);
            }
        }
    }

    // -----------------------------------------------------------------------
    // Async schedule management
    // -----------------------------------------------------------------------

    /// Link a QH into the async schedule (after the sentinel at index 0).
    ///
    /// # Safety
    /// - The QH must be fully initialized.
    /// - Cache must be cleaned after this call.
    unsafe fn link_qh_to_async_schedule(&self, qh: *mut QueueHead) {
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
    unsafe fn unlink_qh_from_async_schedule(&self, qh: *mut QueueHead) {
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
    unsafe fn link_qh_to_periodic_schedule(&self, qh: *mut QueueHead) {
        // Read the current head from the first entry (all entries are kept in sync)
        let old_head = self.statics.frame_list.entries[0].read();
        // New QH → old head (or TERMINATE if list was empty)
        (*qh).horizontal_link.write(old_head);
        // All frame list entries → new QH (volatile write via raw pointer)
        let new_link = ehci::link_pointer(qh as u32, ehci::link_type::QH);
        let entries = self.statics.frame_list.entries.as_ptr() as *mut u32;
        for i in 0..ehci::FRAME_LIST_LEN {
            core::ptr::write_volatile(entries.add(i), new_link);
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
    unsafe fn unlink_qh_from_periodic_schedule(
        statics: &'static UsbStatics,
        qh: *const QueueHead,
    ) {
        let target_addr = ehci::link_address(qh as u32);
        let successor = (*qh).horizontal_link.read();

        // Update any frame list entries that point directly to this QH (head case).
        let entries = statics.frame_list.entries.as_ptr() as *mut u32;
        for i in 0..ehci::FRAME_LIST_LEN {
            let val = core::ptr::read_volatile(entries.add(i));
            if ehci::link_address(val) == target_addr {
                core::ptr::write_volatile(entries.add(i), successor);
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

    // -----------------------------------------------------------------------
    // Async schedule management
    // -----------------------------------------------------------------------

    /// Enable the async schedule if not already enabled.
    fn enable_async_schedule(&self) {
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
    async fn wait_async_advance(&self) {
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
    // EHCI error mapping
    // -----------------------------------------------------------------------

    /// Map EHCI qTD status bits to a `UsbError`.
    fn map_qtd_error(token: u32) -> UsbError {
        if token & QTD_TOKEN_HALTED != 0 {
            if token & QTD_TOKEN_BABBLE != 0 {
                return UsbError::Overflow;
            }
            if token & QTD_TOKEN_BUFFER_ERR != 0 {
                return UsbError::Overflow;
            }
            if token & QTD_TOKEN_XACT_ERR != 0 {
                return UsbError::ProtocolError;
            }
            // Halted with no other error bits set → STALL
            return UsbError::Stall;
        }
        if token & QTD_TOKEN_MISSED_UFRAME != 0 {
            return UsbError::Timeout;
        }
        UsbError::ProtocolError
    }

    // -----------------------------------------------------------------------
    // Cache maintenance wrappers
    // -----------------------------------------------------------------------

    /// Clean and invalidate a QH for DMA.
    fn cache_clean_qh(qh: *const QueueHead) {
        cache::clean_invalidate_dcache_by_address(qh as usize, core::mem::size_of::<QueueHead>());
    }

    /// Clean and invalidate a qTD for DMA.
    fn cache_clean_qtd(qtd: *const TransferDescriptor) {
        cache::clean_invalidate_dcache_by_address(
            qtd as usize,
            core::mem::size_of::<TransferDescriptor>(),
        );
    }

    /// Clean and invalidate a data buffer for DMA.
    fn cache_clean_buffer(addr: *const u8, len: usize) {
        if len > 0 {
            cache::clean_invalidate_dcache_by_address(addr as usize, len);
        }
    }

    // -----------------------------------------------------------------------
    // Control transfer implementation
    // -----------------------------------------------------------------------

    /// Perform an EHCI control transfer using a qTD chain.
    ///
    /// Builds 2–3 qTDs (setup + optional data + status), configures a QH,
    /// links it to the async schedule, and waits for completion.
    async fn do_control_transfer(
        &self,
        address: u8,
        transfer_extras: TransferExtras,
        packet_size: u8,
        setup: &SetupPacket,
        data_phase: &mut DataPhase<'_>,
    ) -> Result<usize, UsbError> {
        // Allocate a QH (index 1 is reserved for control transfers)
        let qh_index = 1;
        let qh = unsafe { self.qh_mut(qh_index) };

        // Determine port speed
        let speed = match self.port_speed() {
            0 => SPEED_FULL,
            1 => SPEED_LOW,
            2 => SPEED_HIGH,
            _ => SPEED_FULL,
        };

        // Build QH characteristics
        let characteristics = ehci::qh_characteristics(
            address,
            0,              // endpoint 0 (control)
            speed,
            packet_size as u16,
            true,           // is_control
            false,          // not head of reclamation
        );

        // Build QH capabilities — handle TransferExtras::WithPreamble for
        // split transactions (FS/LS device behind HS hub)
        let capabilities = match transfer_extras {
            TransferExtras::Normal => ehci::qh_capabilities(0, 0, 0, 0, 1),
            TransferExtras::WithPreamble => {
                // For split transactions, we need the hub address and port.
                // WithPreamble is used for LS devices behind FS hubs.
                // In EHCI, split transactions require hub_addr and hub_port
                // in the QH capabilities, plus S-mask/C-mask.
                // For now, set default values — proper hub support requires
                // additional context from the caller.
                ehci::qh_capabilities(0, 0, 0, 0, 1)
            }
        };

        // Initialise the QH
        unsafe { (*qh).init_endpoint(characteristics, capabilities) };

        // ---- Build the qTD chain ----

        // We need up to 3 qTDs: setup, data (optional), status
        let setup_qtd_idx = self.alloc_qtd().ok_or(UsbError::AllPipesInUse)?;
        let data_qtd_idx = match data_phase {
            DataPhase::In(_) | DataPhase::Out(_) => {
                Some(self.alloc_qtd().ok_or_else(|| {
                    unsafe { self.free_qtd(setup_qtd_idx) };
                    UsbError::AllPipesInUse
                })?)
            }
            DataPhase::None => None,
        };
        let status_qtd_idx = self.alloc_qtd().ok_or_else(|| {
            if let Some(idx) = data_qtd_idx {
                unsafe { self.free_qtd(idx) };
            }
            unsafe { self.free_qtd(setup_qtd_idx) };
            UsbError::AllPipesInUse
        })?;

        // Setup qTD: PID=SETUP, 8 bytes, data toggle=0, no IOC
        let setup_qtd = unsafe { self.qtd_mut(setup_qtd_idx) };
        let setup_bytes = setup as *const SetupPacket as *const u8;
        let setup_token = ehci::qtd_token(PID_SETUP, 8, false, false);
        unsafe { (*setup_qtd).init(setup_token, setup_bytes, 8) };

        // Data qTD (if present)
        let data_len: usize;
        match data_phase {
            DataPhase::In(ref buf) => {
                data_len = buf.len();
                let data_qtd = unsafe { self.qtd_mut(data_qtd_idx.unwrap()) };
                let data_token =
                    ehci::qtd_token(PID_IN, data_len as u32, true, false);
                unsafe {
                    (*data_qtd).init(data_token, buf.as_ptr(), data_len as u32);
                }
            }
            DataPhase::Out(ref buf) => {
                data_len = buf.len();
                let data_qtd = unsafe { self.qtd_mut(data_qtd_idx.unwrap()) };
                let data_token =
                    ehci::qtd_token(PID_OUT, data_len as u32, true, false);
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
        let status_qtd = unsafe { self.qtd_mut(status_qtd_idx) };
        let status_token = ehci::qtd_token(status_pid, 0, true, true);
        unsafe { (*status_qtd).init(status_token, core::ptr::null(), 0) };

        // Chain qTDs: setup → data (optional) → status
        match data_qtd_idx {
            Some(data_idx) => {
                let data_qtd_ptr = unsafe { self.qtd_mut(data_idx) };
                unsafe {
                    (*setup_qtd).next.write(data_qtd_ptr as u32);
                    (*data_qtd_ptr).next.write(status_qtd as u32);
                }
            }
            None => {
                unsafe {
                    (*setup_qtd).next.write(status_qtd as u32);
                }
            }
        }

        // Attach the first qTD to the QH
        unsafe { (*qh).attach_qtd(setup_qtd) };

        // ---- Cache maintenance before DMA ----

        // Clean the setup packet data (it's on the stack, needs to be in RAM)
        Self::cache_clean_buffer(setup_bytes, 8);

        // Clean outgoing data buffer if applicable
        if let DataPhase::Out(ref buf) = data_phase {
            Self::cache_clean_buffer(buf.as_ptr(), buf.len());
        }

        // Clean all qTDs
        Self::cache_clean_qtd(setup_qtd);
        if let Some(data_idx) = data_qtd_idx {
            Self::cache_clean_qtd(unsafe { self.qtd_mut(data_idx) });
        }
        Self::cache_clean_qtd(status_qtd);

        // Clean the QH
        Self::cache_clean_qh(qh);

        // Clean the sentinel QH (we're about to modify its horizontal_link)
        let sentinel = unsafe { self.qh_mut(0) };
        Self::cache_clean_qh(sentinel);

        // ---- Link QH to async schedule and enable ----

        unsafe { self.link_qh_to_async_schedule(qh) };

        // Clean both QH and sentinel after linking (both horizontal_links changed)
        Self::cache_clean_qh(qh);
        Self::cache_clean_qh(sentinel);

        self.enable_async_schedule();

        // ---- Poll for completion ----

        let result = TransferComplete {
            usb: &self.usb,
            shared: self.shared,
            status_qtd: status_qtd as *const TransferDescriptor,
            data_qtd: data_qtd_idx.map(|i| &self.statics.qtd_pool[i] as *const TransferDescriptor),
            qh: qh as *const QueueHead,
        }
        .await;

        // ---- Unlink QH from async schedule ----

        unsafe { self.unlink_qh_from_async_schedule(qh) };
        Self::cache_clean_qh(sentinel);

        // Ring the async advance doorbell and wait
        self.wait_async_advance().await;

        // ---- Copy data for IN transfers ----

        let bytes_transferred = match result {
            Ok(()) => {
                match data_phase {
                    DataPhase::In(ref mut buf) => {
                        // Invalidate cache for the IN data buffer
                        Self::cache_clean_buffer(buf.as_ptr(), buf.len());

                        // Read how many bytes were actually transferred
                        if let Some(data_idx) = data_qtd_idx {
                            let data_qtd_ptr = &self.statics.qtd_pool[data_idx];
                            // Invalidate cache to read updated qTD token
                            Self::cache_clean_qtd(data_qtd_ptr);
                            let remaining = data_qtd_ptr.bytes_remaining() as usize;
                            Ok(data_len - remaining)
                        } else {
                            Ok(0)
                        }
                    }
                    DataPhase::Out(ref buf) => Ok(buf.len()),
                    DataPhase::None => Ok(0),
                }
            }
            Err(e) => Err(e),
        };

        // ---- Free resources ----

        unsafe {
            self.free_qtd(setup_qtd_idx);
            if let Some(idx) = data_qtd_idx {
                self.free_qtd(idx);
            }
            self.free_qtd(status_qtd_idx);
            // Clear the QH state
            (*qh).sw_flags.write(0);
        }

        bytes_transferred
    }

    // -----------------------------------------------------------------------
    // Interrupt pipe implementation
    // -----------------------------------------------------------------------

    /// Set up and return an interrupt pipe for polling an IN endpoint.
    ///
    /// Called by both [`alloc_interrupt_pipe`] (after an async pool allocation)
    /// and [`try_alloc_interrupt_pipe`] (after a synchronous try-alloc).
    fn do_alloc_interrupt_pipe(
        &self,
        pipe: Pipe,
        address: u8,
        _transfer_extras: TransferExtras,
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
        let qtd_index = self.alloc_qtd().expect("qTD pool exhausted for interrupt pipe");

        // Determine port speed.
        let speed = match self.port_speed() {
            0 => ehci::SPEED_FULL,
            1 => ehci::SPEED_LOW,
            _ => ehci::SPEED_HIGH,
        };

        // Build QH endpoint characteristics.
        // DTC = 0 (hardware-managed data toggle in QH overlay) for non-control endpoints.
        let characteristics = ehci::qh_characteristics(
            address,
            endpoint,
            speed,
            max_packet_size,
            false, // not a control endpoint
            false, // not head of reclamation list
        );

        // Build QH endpoint capabilities.
        // S-mask = 0x01: poll in micro-frame 0 of each scheduled frame.
        // C-mask = 0: no split-completion mask (not a split transaction).
        // hub_addr/hub_port = 0: device is directly connected (no TT).
        let capabilities = ehci::qh_capabilities(0x01, 0, 0, 0, 1);

        // Initialise the QH.
        let qh = unsafe { self.qh_mut(qh_index) };
        unsafe { (*qh).init_endpoint(characteristics, capabilities) };

        // Set up the initial qTD: PID=IN, Active, max_packet_size bytes, IOC.
        let recv_buf_ptr = self.statics.recv_bufs[recv_buf_idx].as_ptr();
        // DIAG Step 1: Confirm recv_buf is in DMA-accessible memory (OCRAM 0x2020_xxxx/0x2024_xxxx).
        // If address is 0x2000_xxxx (DTCM), EHCI DMA cannot write there → always zeros.
        log::info!(
            "[HC] recv_buf[{}] @ 0x{:08x} (OCRAM=0x2020_0000..0x202F_FFFF, DTCM=0x2000_xxxx)",
            recv_buf_idx,
            recv_buf_ptr as u32,
        );
        let token = ehci::qtd_token(PID_IN, max_packet_size as u32, false, true);
        let qtd = unsafe { self.qtd_mut(qtd_index) };
        unsafe { (*qtd).init(token, recv_buf_ptr, max_packet_size as u32) };

        // Attach qTD to QH (sets overlay_next, clears halt — OK for first attach).
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
        unsafe { self.link_qh_to_periodic_schedule(qh) };

        // Cache-clean QH (horizontal_link changed) and frame list (all entries changed).
        Self::cache_clean_qh(qh);
        cache::clean_invalidate_dcache_by_address(
            self.statics.frame_list.entries.as_ptr() as usize,
            core::mem::size_of::<ehci::FrameList>(),
        );

        info!(
            "[HC] interrupt pipe allocated: addr={} ep={} mps={} qh={} qtd={}",
            address, endpoint, max_packet_size, qh_index, qtd_index
        );

        Imxrt1062InterruptPipe {
            pipe,
            qh_index,
            qtd_index,
            recv_buf_idx,
            address,
            endpoint,
            max_packet_size,
            statics: self.statics,
            shared: self.shared,
            usb: self.usb.addr as *const ral::usb::RegisterBlock,
        }
    }
}

// ---------------------------------------------------------------------------
// TransferComplete — future that waits for a qTD chain to complete
// ---------------------------------------------------------------------------

/// Future that polls an EHCI qTD for completion.
///
/// Checks the status qTD's Active bit. When cleared by the controller,
/// the transfer is complete. Error bits are mapped to `UsbError`.
struct TransferComplete<'a> {
    usb: &'a ral::usb::Instance,
    shared: &'a UsbShared,
    status_qtd: *const TransferDescriptor,
    data_qtd: Option<*const TransferDescriptor>,
    qh: *const QueueHead,
}

impl Future for TransferComplete<'_> {
    type Output = Result<(), UsbError>;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        // Register waker with pipe 0 (control pipe)
        self.shared.pipe_wakers[0].register(cx.waker());

        // Invalidate cache to see hardware updates
        Imxrt1062HostController::cache_clean_qtd(self.status_qtd);
        if let Some(data_qtd) = self.data_qtd {
            Imxrt1062HostController::cache_clean_qtd(data_qtd);
        }
        Imxrt1062HostController::cache_clean_qh(self.qh);

        let status_qtd = unsafe { &*self.status_qtd };
        let token = status_qtd.token.read();

        if token & QTD_TOKEN_ACTIVE != 0 {
            // Still active — check if data qTD has errored (early exit)
            if let Some(data_qtd_ptr) = self.data_qtd {
                let data_qtd = unsafe { &*data_qtd_ptr };
                let data_token = data_qtd.token.read();
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
        if let Some(data_qtd_ptr) = self.data_qtd {
            let data_qtd = unsafe { &*data_qtd_ptr };
            let data_token = data_qtd.token.read();
            if data_token & ehci::QTD_TOKEN_ERROR_MASK != 0 {
                return Poll::Ready(Err(Imxrt1062HostController::map_qtd_error(
                    data_token,
                )));
            }
        }

        Poll::Ready(Ok(()))
    }
}

// Safety: TransferComplete holds references to static memory (QH/qTD pools)
// and register blocks. The qTD/QH pointers point to static pool entries.
unsafe impl Send for TransferComplete<'_> {}

// ---------------------------------------------------------------------------
// AsyncAdvanceWait — future for async advance doorbell
// ---------------------------------------------------------------------------

/// Future that waits for the EHCI async advance doorbell to be acknowledged.
///
/// After unlinking a QH from the async schedule, the caller rings the doorbell
/// (sets USBCMD.IAA) and waits for USBSTS.AAI. This future polls for that.
struct AsyncAdvanceWait<'a> {
    usb: &'a ral::usb::Instance,
    shared: &'a UsbShared,
}

impl Future for AsyncAdvanceWait<'_> {
    type Output = ();

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        self.shared.async_advance_waker.register(cx.waker());

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

// ---------------------------------------------------------------------------
// Imxrt1062DeviceDetect — Stream<Item = DeviceStatus>
// ---------------------------------------------------------------------------

/// Device detection stream for the i.MX RT 1062 USB host controller.
///
/// Monitors the root port for connect/disconnect events by polling PORTSC1.
/// Yields `DeviceStatus::Present(speed)` when a device is connected, and
/// `DeviceStatus::Absent` when disconnected.
///
/// Follows the RP2040 pattern: stores the previous status and only returns
/// `Ready` when the status changes.
#[derive(Copy, Clone)]
pub struct Imxrt1062DeviceDetect {
    usb: *const ral::usb::RegisterBlock,
    waker: &'static CriticalSectionWakerRegistration,
    status: DeviceStatus,
}

impl Imxrt1062DeviceDetect {
    fn new(usb: &ral::usb::Instance, waker: &'static CriticalSectionWakerRegistration) -> Self {
        Self {
            usb: usb.addr as *const ral::usb::RegisterBlock,
            waker,
            status: DeviceStatus::Absent,
        }
    }

    /// Read the current device status from PORTSC1.
    fn read_device_status(&self) -> DeviceStatus {
        let usb = unsafe { &*(self.usb as *const ral::usb::RegisterBlock) };
        let usb_instance = ral::usb::Instance {
            addr: usb as *const _ as *mut _,
        };
        let portsc = ral::read_reg!(ral::usb, usb_instance, PORTSC1);
        let connected = (portsc & 1) != 0; // CCS bit 0
        if connected {
            let pspd = (portsc >> 26) & 0x3;
            match pspd {
                0 => DeviceStatus::Present(UsbSpeed::Full12),
                1 => DeviceStatus::Present(UsbSpeed::Low1_5),
                2 => DeviceStatus::Present(UsbSpeed::High480),
                _ => DeviceStatus::Present(UsbSpeed::Full12),
            }
        } else {
            DeviceStatus::Absent
        }
    }

    /// Re-enable the port change interrupt.
    fn reenable_interrupt(&self) {
        let usb_instance = ral::usb::Instance {
            addr: self.usb as *mut _,
        };
        ral::modify_reg!(ral::usb, usb_instance, USBINTR, |v| v | (1 << 2));
    }
}

impl Stream for Imxrt1062DeviceDetect {
    type Item = DeviceStatus;

    fn poll_next(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<Self::Item>> {
        self.waker.register(cx.waker());

        let device_status = self.read_device_status();

        if device_status != self.status {
            let usb_instance = ral::usb::Instance {
                addr: self.usb as *mut _,
            };
            let portsc = ral::read_reg!(ral::usb, usb_instance, PORTSC1);
            info!("[HC] DeviceDetect: status change  PORTSC1=0x{:08X}", portsc);
            self.reenable_interrupt();
            self.status = device_status;
            Poll::Ready(Some(device_status))
        } else {
            self.reenable_interrupt();
            Poll::Pending
        }
    }
}

// Safety: The USB register pointer is derived from a static instance.
unsafe impl Send for Imxrt1062DeviceDetect {}

// ---------------------------------------------------------------------------
// Pipe — RAII pipe allocation wrapper
// ---------------------------------------------------------------------------

/// Wraps a pool allocation for a pipe. When dropped, returns the resource
/// to the pool.
struct Pipe {
    _pooled: cotton_usb_host::async_pool::Pooled<'static>,
    which: u8,
}

impl Pipe {
    fn new(pooled: cotton_usb_host::async_pool::Pooled<'static>, offset: u8) -> Self {
        let which = pooled.which() + offset;
        Self {
            _pooled: pooled,
            which,
        }
    }

    fn which(&self) -> u8 {
        self.which
    }
}

// ---------------------------------------------------------------------------
// Imxrt1062InterruptPipe — periodic schedule interrupt endpoint stream
// ---------------------------------------------------------------------------

/// Interrupt IN pipe for i.MX RT 1062.
///
/// Wraps a single QH + qTD polling an interrupt IN endpoint via the EHCI
/// periodic schedule. Implements `Stream<Item = InterruptPacket>` so callers
/// can `await` the next packet with standard async combinators.
///
/// # Lifecycle
///
/// Created by [`Imxrt1062HostController::alloc_interrupt_pipe`] or
/// [`try_alloc_interrupt_pipe`]. The pipe occupies one slot from the
/// `bulk_pipes` pool and one slot from the `qtd_pool` for its entire lifetime.
///
/// On `Drop`, the QH is unlinked from the periodic frame list and the qTD
/// slot is freed. A brief (~1 ms) busy-wait ensures the EHCI controller has
/// crossed at least one frame boundary before resources are released.
pub struct Imxrt1062InterruptPipe {
    /// Pool allocation (RAII — frees the `bulk_pipes` slot on Drop).
    pipe: Pipe,
    /// Index into `statics.qh_pool` for this pipe's QH.
    qh_index: usize,
    /// Index into `statics.qtd_pool` for this pipe's receive qTD.
    qtd_index: usize,
    /// Index into `statics.recv_bufs` for the DMA receive buffer.
    recv_buf_idx: usize,
    /// USB device address.
    address: u8,
    /// Endpoint number.
    endpoint: u8,
    /// Maximum packet size (used when re-arming the qTD).
    max_packet_size: u16,
    /// Static resource pools.
    statics: &'static UsbStatics,
    /// ISR ↔ async shared state.
    shared: &'static UsbShared,
    /// Raw pointer to the USB OTG register block (for re-enabling interrupts).
    usb: *const ral::usb::RegisterBlock,
}

impl Stream for Imxrt1062InterruptPipe {
    type Item = InterruptPacket;

    fn poll_next(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<Self::Item>> {
        // Register waker before checking status (race-free pattern).
        self.shared
            .pipe_waker(self.pipe.which() as usize)
            .register(cx.waker());

        // Invalidate cache to see any hardware updates to the qTD token.
        let qtd = &self.statics.qtd_pool[self.qtd_index];
        Imxrt1062HostController::cache_clean_qtd(qtd as *const TransferDescriptor);

        let token = qtd.token.read();

        if token & ehci::QTD_TOKEN_ACTIVE != 0 {
            // Transfer still in progress — re-enable transfer interrupts and wait.
            let usb_inst = ral::usb::Instance {
                addr: self.usb as *mut _,
            };
            ral::modify_reg!(ral::usb, usb_inst, USBINTR, |v| v
                | (1 << 0)   // UE
                | (1 << 1)   // UEE
                | (1 << 18)  // UAIE
                | (1 << 19)  // UPIE
            );
            return Poll::Pending;
        }

        // --- Transfer complete (Active cleared by controller) ---

        // Invalidate receive buffer cache before reading the data.
        // Use invalidate-only (DCIMVAC), NOT clean+invalidate (DCCIMVAC),
        // because for DMA receive buffers we must NOT write back any stale
        // dirty cache lines — the RAM contents written by DMA are authoritative.
        let recv_buf = &self.statics.recv_bufs[self.recv_buf_idx];
        cache::invalidate_dcache_by_address(recv_buf.as_ptr() as usize, recv_buf.len());

        // DIAG Step 2: Log raw qTD token and first 8 recv_buf bytes.
        // H1 (DTCM): bytes always 00 even with key held.
        // H3 (Error): token bit6=Halted or bit3=XACT_ERR, remaining=mps (0 bytes transferred).
        // Working: token=0x0000_0000, remaining=0, non-zero bytes on key press.
        log::info!(
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
        let qtd_ptr =
            &self.statics.qtd_pool[self.qtd_index] as *const TransferDescriptor as *mut TransferDescriptor;
        let qh_ptr =
            &self.statics.qh_pool[self.qh_index] as *const QueueHead as *mut QueueHead;

        unsafe {
            (*qtd_ptr).init(rearm_token, recv_buf.as_ptr(), self.max_packet_size as u32);
            (*qh_ptr).reattach_qtd_preserve_toggle(qtd_ptr);
        }

        // Flush both the qTD and QH back to RAM for the DMA engine.
        Imxrt1062HostController::cache_clean_qtd(qtd_ptr);
        Imxrt1062HostController::cache_clean_qh(qh_ptr);

        // Re-enable transfer interrupts so the next completion wakes us.
        let usb_inst = ral::usb::Instance {
            addr: self.usb as *mut _,
        };
        ral::modify_reg!(ral::usb, usb_inst, USBINTR, |v| v
            | (1 << 0)   // UE
            | (1 << 1)   // UEE
            | (1 << 18)  // UAIE
            | (1 << 19)  // UPIE
        );

        Poll::Ready(Some(packet))
    }
}

impl Drop for Imxrt1062InterruptPipe {
    fn drop(&mut self) {
        // 1. Remove the QH from the periodic frame list.
        let qh_ptr =
            &self.statics.qh_pool[self.qh_index] as *const QueueHead as *mut QueueHead;
        unsafe {
            Imxrt1062HostController::unlink_qh_from_periodic_schedule(self.statics, qh_ptr);
        }

        // Clean the frame list after modification.
        cache::clean_invalidate_dcache_by_address(
            self.statics.frame_list.entries.as_ptr() as usize,
            core::mem::size_of::<ehci::FrameList>(),
        );
        // Clean any predecessor QHs whose horizontal_link we may have changed.
        for qh in &self.statics.qh_pool[2..=NUM_QH] {
            Imxrt1062HostController::cache_clean_qh(qh as *const QueueHead);
        }

        // 2. Wait ≥1 ms for the controller to cross a frame boundary.
        //
        // After unlinking, the controller may complete an in-progress access
        // to this QH for the current frame. A ~1 ms busy-wait (one EHCI frame
        // at full speed = 1 ms) ensures no further DMA accesses will occur
        // before we release the memory.
        cortex_m::asm::delay(600_000); // 1 ms at 600 MHz

        // 3. Free the qTD back to the pool.
        unsafe {
            let qtd =
                &self.statics.qtd_pool[self.qtd_index] as *const TransferDescriptor as *mut TransferDescriptor;
            (*qtd).next.write(ehci::LINK_TERMINATE);
            (*qtd).alt_next.write(ehci::LINK_TERMINATE);
            (*qtd).token.write(0);
            for buf in &mut (*qtd).buffer {
                buf.write(0);
            }
        }

        // 4. Mark the QH as unused (cleared on next init_endpoint() call too,
        //    but explicit clear guards against stale flag reads).
        unsafe { (*qh_ptr).sw_flags.write(0) };

        // 5. `self.pipe` drops here, returning the bulk_pipes pool slot.
    }
}

// Safety: Imxrt1062InterruptPipe holds *const RegisterBlock (stable MMIO address)
// and &'static references. It is safe to send between tasks.
unsafe impl Send for Imxrt1062InterruptPipe {}

// ---------------------------------------------------------------------------
// HostController trait implementation
// ---------------------------------------------------------------------------

impl HostController for Imxrt1062HostController {
    type InterruptPipe = Imxrt1062InterruptPipe;
    type DeviceDetect = Imxrt1062DeviceDetect;

    fn device_detect(&self) -> Self::DeviceDetect {
        Imxrt1062DeviceDetect::new(&self.usb, self.shared.device_waker())
    }

    fn reset_root_port(&self, rst: bool) {
        if rst {
            // Set PORTSC1.PR (bit 8) — begin USB reset signaling.
            // Must preserve other bits and avoid clearing W1C bits.
            let portsc = self.portsc1_read_safe();
            ral::write_reg!(ral::usb, self.usb, PORTSC1, portsc | (1 << 8));
        } else {
            // Clear PORTSC1.PR (bit 8) — end USB reset signaling.
            // On EHCI, the controller may auto-clear PR and set PE (port enabled).
            let portsc = self.portsc1_read_safe();
            ral::write_reg!(ral::usb, self.usb, PORTSC1, portsc & !(1 << 8));
        }
    }

    async fn control_transfer<'a>(
        &self,
        address: u8,
        transfer_extras: TransferExtras,
        packet_size: u8,
        setup: SetupPacket,
        mut data_phase: DataPhase<'a>,
    ) -> Result<usize, UsbError> {
        let data_len = match &data_phase {
            DataPhase::In(buf) => buf.len() as i32,
            DataPhase::Out(buf) => -(buf.len() as i32),
            DataPhase::None => 0,
        };
        // Allocate a control pipe (serializes control transfers)
        let _pipe = Pipe::new(self.statics.control_pipes.alloc().await, 0);

        let result = self.do_control_transfer(address, transfer_extras, packet_size, &setup, &mut data_phase)
            .await;

        if let Ok(n) = &result {
            info!("[HC] control_transfer -> Ok({})", n);
        } else {
            warn!("[HC] control_transfer -> Err");
        }
        result
    }

    async fn bulk_in_transfer(
        &self,
        _address: u8,
        _endpoint: u8,
        _packet_size: u16,
        _data: &mut [u8],
        _transfer_type: TransferType,
        _data_toggle: &Cell<bool>,
    ) -> Result<usize, UsbError> {
        // Phase 2b stub
        Err(UsbError::ProtocolError)
    }

    async fn bulk_out_transfer(
        &self,
        _address: u8,
        _endpoint: u8,
        _packet_size: u16,
        _data: &[u8],
        _transfer_type: TransferType,
        _data_toggle: &Cell<bool>,
    ) -> Result<usize, UsbError> {
        // Phase 2b stub
        Err(UsbError::ProtocolError)
    }

    async fn alloc_interrupt_pipe(
        &self,
        address: u8,
        transfer_extras: TransferExtras,
        endpoint: u8,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Imxrt1062InterruptPipe {
        let pipe = Pipe::new(self.statics.bulk_pipes.alloc().await, 1);
        self.do_alloc_interrupt_pipe(
            pipe,
            address,
            transfer_extras,
            endpoint,
            max_packet_size,
            interval_ms,
        )
    }

    fn try_alloc_interrupt_pipe(
        &self,
        address: u8,
        transfer_extras: TransferExtras,
        endpoint: u8,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::InterruptPipe, UsbError> {
        let pooled = self
            .statics
            .bulk_pipes
            .try_alloc()
            .ok_or(UsbError::AllPipesInUse)?;
        Ok(self.do_alloc_interrupt_pipe(
            Pipe::new(pooled, 1),
            address,
            transfer_extras,
            endpoint,
            max_packet_size,
            interval_ms,
        ))
    }
}
