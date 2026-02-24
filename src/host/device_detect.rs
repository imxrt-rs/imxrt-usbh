//! Device detection stream for the root port.

use crate::ral;
use core::pin::Pin;
use core::task::{Context, Poll};
use cotton_usb_host::host_controller::{DeviceStatus, UsbSpeed};
use futures_core::Stream;
use rtic_common::waker_registration::CriticalSectionWakerRegistration;

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
    /// USB OTG register block base address (stored as `u32` to keep the struct
    /// `Send` without a manual `unsafe impl`).
    usb_base: u32,
    /// USB PHY register block base address.
    usbphy_base: u32,
    waker: &'static CriticalSectionWakerRegistration,
    status: DeviceStatus,
}

impl Imxrt1062DeviceDetect {
    pub(super) fn new(
        usb: &ral::usb::Instance,
        usbphy: &ral::usbphy::Instance,
        waker: &'static CriticalSectionWakerRegistration,
    ) -> Self {
        Self {
            usb_base: usb.addr as usize as u32,
            usbphy_base: usbphy.addr as usize as u32,
            waker,
            status: DeviceStatus::Absent,
        }
    }

    /// Reconstruct a temporary `ral::usb::Instance` from the stored base address.
    fn usb_instance(&self) -> ral::usb::Instance {
        ral::usb::Instance {
            addr: self.usb_base as *const ral::usb::RegisterBlock,
        }
    }

    /// Reconstruct a temporary `ral::usbphy::Instance` from the stored base address.
    fn usbphy_instance(&self) -> ral::usbphy::Instance {
        ral::usbphy::Instance {
            addr: self.usbphy_base as *const ral::usbphy::RegisterBlock,
        }
    }

    /// Read the current device status from PORTSC1.
    fn read_device_status(&self) -> DeviceStatus {
        let usb = self.usb_instance();
        let portsc = ral::read_reg!(ral::usb, usb, PORTSC1);
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
        let usb = self.usb_instance();
        ral::modify_reg!(ral::usb, usb, USBINTR, |v| v | (1 << 2));
    }

    /// Set ENHOSTDISCONDETECT in the USBPHY CTRL register.
    ///
    /// Must only be called when a High Speed device is connected (HSP=1).
    /// Enables the PHY's HS disconnect detector.
    fn set_enhostdiscondetect(&self) {
        let usbphy = self.usbphy_instance();
        ral::write_reg!(ral::usbphy, usbphy, CTRL_SET, ENHOSTDISCONDETECT: 1);
    }

    /// Clear ENHOSTDISCONDETECT in the USBPHY CTRL register.
    ///
    /// Called on device disconnect to prevent false disconnect detection
    /// when no device is connected.
    fn clear_enhostdiscondetect(&self) {
        let usbphy = self.usbphy_instance();
        ral::write_reg!(ral::usbphy, usbphy, CTRL_CLR, ENHOSTDISCONDETECT: 1);
    }
}

impl Stream for Imxrt1062DeviceDetect {
    type Item = DeviceStatus;

    fn poll_next(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<Self::Item>> {
        self.waker.register(cx.waker());

        let device_status = self.read_device_status();

        // Determine whether this is a connect/disconnect transition.
        // We intentionally suppress speed-change-only events because EHCI
        // reports FS before port reset (PSPD from line state) then HS after
        // reset (PSPD from chirp negotiation). Without this filter, a HS
        // device triggers two DeviceDetect events: Present(Full12) then
        // Present(High480), and the second one causes cotton-usb-host to
        // re-reset the port and re-enumerate, disrupting hub state.
        let was_connected = matches!(self.status, DeviceStatus::Present(_));
        let is_connected = matches!(device_status, DeviceStatus::Present(_));
        let connection_changed = was_connected != is_connected;

        if connection_changed {
            let usb = self.usb_instance();
            let portsc = ral::read_reg!(ral::usb, usb, PORTSC1);
            debug!("[HC] DeviceDetect: status change  PORTSC1=0x{:08X}", portsc);

            // Manage ENHOSTDISCONDETECT based on connection state.
            // Per i.MX RT reference manual and USBHost_t36: set only when a
            // High Speed device is connected (HSP=1), clear on disconnect.
            match device_status {
                DeviceStatus::Present(UsbSpeed::High480) => {
                    self.set_enhostdiscondetect();
                    debug!("[HC] ENHOSTDISCONDETECT set (HS device connected)");
                }
                DeviceStatus::Absent => {
                    self.clear_enhostdiscondetect();
                }
                _ => {
                    // FS/LS device — ensure disconnect detector is off.
                    self.clear_enhostdiscondetect();
                }
            }

            self.reenable_interrupt();
            self.status = device_status;
            Poll::Ready(Some(device_status))
        } else {
            // Silently track any speed change (e.g. FS→HS after reset) and
            // manage ENHOSTDISCONDETECT without firing a new event.
            if device_status != self.status {
                if matches!(device_status, DeviceStatus::Present(UsbSpeed::High480)) {
                    self.set_enhostdiscondetect();
                }
                self.status = device_status;
            }
            self.reenable_interrupt();
            Poll::Pending
        }
    }
}
