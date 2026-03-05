//! `ImxrtHostController` struct definition, construction, and initialization.

use crate::ral;

use super::shared::UsbShared;
use super::statics::UsbStatics;

// ---------------------------------------------------------------------------
// ImxrtHostController
// ---------------------------------------------------------------------------

/// USB host controller for i.MX RT 1062.
///
/// This is the main driver type that implements the
/// [`HostController`](crate::host_controller::HostController) trait.
/// It owns the USB register blocks and holds references to the shared and
/// static resources.
///
/// # Speed and Hub Support
///
/// When the **`hub-support`** cargo feature is enabled, the controller sets
/// the NXP-specific `PFSC` (Port Force Full Speed Connect) bit, forcing
/// **all** devices — including those connected directly — to Full Speed
/// (12 Mbps). This is necessary because `cotton-usb-host` does not support
/// EHCI split transactions required for High Speed hubs.
///
/// With `hub-support` enabled (`PFSC=1`):
/// - All devices (including hubs) connect at Full Speed
/// - Full Speed hubs act as simple repeaters (no Transaction Translator)
/// - Low Speed devices behind FS hubs use `WithPreamble` (PRE PID)
/// - Maximum throughput is 12 Mbps
///
/// Without `hub-support` (the default):
/// - High Speed (480 Mbps) devices can negotiate at full speed
/// - Hubs should **not** be used (HS hubs require unsupported split
///   transactions; FS hubs would not connect at FS without PFSC)
///
/// # Construction
///
/// ```ignore
/// use imxrt_usbh::host::{ImxrtHostController, UsbShared, UsbStatics};
///
/// static SHARED: UsbShared = UsbShared::new();
/// static STATICS: StaticCell<UsbStatics> = StaticCell::new();
///
/// let statics = STATICS.init(UsbStatics::new());
/// let usb = unsafe { imxrt_ral::usb::USB2::instance() };
/// let usbphy = unsafe { imxrt_ral::usbphy::USBPHY2::instance() };
/// let host = ImxrtHostController::new(usb, usbphy, &SHARED, statics);
/// ```
pub struct ImxrtHostController {
    /// USB OTG core registers (owned).
    pub(super) usb: ral::usb::Instance,

    /// USB PHY registers (owned).
    pub(super) usbphy: ral::usbphy::Instance,

    /// Interrupt-safe shared state (borrowed, lives in a static).
    pub(super) shared: &'static UsbShared,

    /// Resource pools and DMA structures (borrowed, lives in a static).
    pub(super) statics: &'static UsbStatics,
}

// Safety: ImxrtHostController is designed for single-task usage.
// The raw pointer in ral::usb::Instance is stable (points to MMIO registers).
// &'static UsbStatics is safe to send because UsbStatics lives in a static and
// is only accessed from async task context (never from ISR).
// &'static UsbShared uses CriticalSection-based synchronization.
unsafe impl Send for ImxrtHostController {}

impl ImxrtHostController {
    /// Create a new host controller from `imxrt-ral` register instances and
    /// static resources.
    ///
    /// # Arguments
    ///
    /// - `usb` — `imxrt-ral` USB OTG register instance (e.g. `USB2::instance()`)
    /// - `usbphy` — `imxrt-ral` USBPHY register instance (e.g. `USBPHY2::instance()`)
    /// - `shared` — reference to [`UsbShared`] in a `static`
    /// - `statics` — reference to [`UsbStatics`] in a `static`
    ///
    /// # Note
    ///
    /// This does **not** initialise the hardware. Call `init()` after
    /// construction to set up the controller.
    pub fn new<const N: u8>(
        usb: imxrt_ral::usb::Instance<N>,
        usbphy: imxrt_ral::usbphy::Instance<N>,
        shared: &'static UsbShared,
        statics: &'static UsbStatics,
    ) -> Self {
        Self {
            usb: ral::usb::Instance::from_ral(usb),
            usbphy: ral::usbphy::Instance::from_ral(usbphy),
            shared,
            statics,
        }
    }

    /// Get a reference to the USB register block.
    #[allow(dead_code)]
    pub(super) fn usb(&self) -> &ral::usb::Instance {
        &self.usb
    }

    /// Get a mutable reference to the USB register block.
    #[allow(dead_code)]
    pub(super) fn usb_mut(&mut self) -> &mut ral::usb::Instance {
        &mut self.usb
    }

    /// Get a reference to the shared state.
    #[allow(dead_code)]
    pub(super) fn shared(&self) -> &'static UsbShared {
        self.shared
    }

    /// Get a reference to the static resources.
    #[allow(dead_code)]
    pub(super) fn statics(&self) -> &'static UsbStatics {
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
    /// # Non-Cacheable Memory
    ///
    /// The driver assumes all DMA structures (QH, qTD, frame list, data
    /// buffers) are in non-cacheable memory. The driver does not perform
    /// any cache maintenance. Users must either disable the D-cache or
    /// use the MPU to mark DMA regions as non-cacheable.
    ///
    /// # Register Alias: `DEVICEADDR` / `PERIODICLISTBASE`
    ///
    /// The `DEVICEADDR` and `PERIODICLISTBASE` registers share the same offset
    /// (`0x154`).  In host mode, this register holds the periodic frame list
    /// base address (bits 31:12).  The RAL module only defines `DEVICEADDR`,
    /// so we write the frame list address via that name.
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
        ral::modify_reg!(ral::usb, self.usb, USBCMD, RST: 1);

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
        let sentinel = self.statics.qh_ptr(0);
        // SAFETY: `sentinel` points to QH[0] in the static pool — valid, aligned,
        // and exclusively ours during init (no schedule is running yet).
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
        ral::write_reg!(ral::usb, self.usb, USBCMD,
            RS: 1,          // Run
            FS_1: 0b01,     // Frame list size low bits (32 entries)
            PSE: 1,         // Periodic schedule enable
            ASP: 0b11,      // Async park count = 3
            ASPE: 1,        // Async park mode enable
            FS_2: 1,        // Frame list size high bit (32 entries)
            ITC: 1          // 1 micro-frame interrupt threshold
        );

        // ---- Step 9: Enable port power ----
        //
        // PP (Port Power) must be set for the root port to supply power.
        //
        // PFSC (Port Force Full Speed Connect, bit 24) is an NXP extension
        // (not in the EHCI specification). When set, it prevents HS chirp
        // negotiation, forcing all devices (including hubs) to connect at
        // Full Speed. This is required for hub support because:
        //   - FS hubs act as simple repeaters (no Transaction Translator)
        //   - LS devices behind FS hubs use WithPreamble (PRE PID)
        //   - HS hubs require EHCI split transactions which cotton-usb-host
        //     doesn't support
        //
        // When the `hub-support` feature is disabled, PFSC is not set,
        // allowing High Speed (480 Mbps) devices to negotiate at full speed.
        let portsc = self.portsc1_read_safe();
        #[cfg(feature = "hub-support")]
        let portsc_val = portsc | ral::usb::PORTSC1::PP::mask | ral::usb::PORTSC1::PFSC::mask;
        #[cfg(not(feature = "hub-support"))]
        let portsc_val = portsc | ral::usb::PORTSC1::PP::mask;
        ral::write_reg!(ral::usb, self.usb, PORTSC1, portsc_val);

        // ---- Step 10: High-speed disconnect detection ----
        //
        // ENHOSTDISCONDETECT must NOT be set until a High Speed device is
        // connected. Setting it prematurely can cause false disconnect events
        // and interfere with FS→HS chirp negotiation during port reset.
        // Per USBHost_t36 (ehci.cpp:405): set only after HSP=1 in PORTSC1.
        //
        // This is handled in ImxrtDeviceDetect::poll_next():
        //   - Set ENHOSTDISCONDETECT when device_status is Present(High480)
        //   - Clear ENHOSTDISCONDETECT on disconnect or FS/LS connection

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
    pub(super) fn port_speed(&self) -> u32 {
        ral::read_reg!(ral::usb, self.usb, PORTSC1, PSPD)
    }

    /// Convert the PORTSC1 PSPD field to a `DeviceStatus`.
    #[allow(dead_code)]
    pub(super) fn device_status(&self) -> cotton_usb_host::host_controller::DeviceStatus {
        let (connected, pspd) = ral::read_reg!(ral::usb, self.usb, PORTSC1, CCS, PSPD);
        if connected != 0 {
            let speed = match pspd {
                0 => cotton_usb_host::host_controller::UsbSpeed::Full12,
                1 => cotton_usb_host::host_controller::UsbSpeed::Low1_5,
                2 => cotton_usb_host::host_controller::UsbSpeed::High480,
                _ => cotton_usb_host::host_controller::UsbSpeed::Full12,
            };
            cotton_usb_host::host_controller::DeviceStatus::Present(speed)
        } else {
            cotton_usb_host::host_controller::DeviceStatus::Absent
        }
    }

    // -----------------------------------------------------------------------
    // Re-enable port change interrupt
    // -----------------------------------------------------------------------

    /// Re-enable the port change interrupt (PCE) in USBINTR.
    ///
    /// Called from poll functions after checking device status, following
    /// the disable-on-handle / re-enable-on-poll pattern.
    #[allow(dead_code)]
    pub(super) fn reenable_port_change_interrupt(&self) {
        ral::modify_reg!(ral::usb, self.usb, USBINTR, PCE: 1);
    }

    /// Re-enable transfer completion interrupts in USBINTR.
    ///
    /// Re-enables: UE, UEE, UAIE, UPIE.
    #[allow(dead_code)]
    pub(super) fn reenable_transfer_interrupts(&self) {
        ral::modify_reg!(ral::usb, self.usb, USBINTR, UE: 1, UEE: 1, UAIE: 1, UPIE: 1);
    }

    /// Re-enable the async advance interrupt (AAE) in USBINTR.
    #[allow(dead_code)]
    pub(super) fn reenable_async_advance_interrupt(&self) {
        ral::modify_reg!(ral::usb, self.usb, USBINTR, AAE: 1);
    }

    // -----------------------------------------------------------------------
    // PORTSC1 W1C-safe writes
    // -----------------------------------------------------------------------

    /// PORTSC1 write-1-to-clear bit mask.
    ///
    /// When doing a read-modify-write on PORTSC1, these bits must be masked
    /// off to avoid accidentally clearing them. Per EHCI spec, W1C bits:
    /// - CSC — Connect Status Change
    /// - PEC — Port Enable/Disable Change
    /// - OCC — Over-current Change
    /// - FPR — Force Port Resume (read as 0 when not suspended)
    pub(super) const PORTSC1_W1C_MASK: u32 = ral::usb::PORTSC1::CSC::mask
        | ral::usb::PORTSC1::PEC::mask
        | ral::usb::PORTSC1::OCC::mask
        | ral::usb::PORTSC1::FPR::mask;

    /// Read PORTSC1 with W1C bits cleared to prevent accidental clear.
    ///
    /// This should be used before any modify_reg! on PORTSC1 to ensure
    /// we don't accidentally write 1 to a W1C bit.
    pub(super) fn portsc1_read_safe(&self) -> u32 {
        ral::read_reg!(ral::usb, self.usb, PORTSC1) & !Self::PORTSC1_W1C_MASK
    }
}
