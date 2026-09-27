//! Device detection stream for the root port.

use super::controller::ImxrtHostController;
use crate::ral;
use core::pin::Pin;
use core::task::{Context, Poll};
use cotton_usb_host::host_controller::{DeviceStatus, UsbSpeed};
use futures_core::Stream;
use rtic_common::waker_registration::CriticalSectionWakerRegistration;

// ---------------------------------------------------------------------------
// What to report: pure logic, no registers
// ---------------------------------------------------------------------------

/// Decode the device status from a PORTSC1 value.
fn status_from_portsc(portsc: u32) -> DeviceStatus {
    use ral::usb::PORTSC1::{CCS, PSPD};

    if portsc & CCS::mask == 0 {
        return DeviceStatus::Absent;
    }
    match (portsc & PSPD::mask) >> PSPD::offset {
        1 => DeviceStatus::Present(UsbSpeed::Low1_5),
        2 => DeviceStatus::Present(UsbSpeed::High480),
        // 0 is full speed; 3 means "not connected" and cannot occur with CCS
        // set, so it is treated as full speed rather than invented.
        _ => DeviceStatus::Present(UsbSpeed::Full12),
    }
}

/// Decide what a poll of the root port should report, if anything.
///
/// `previous` is the status last reported, `now` is the port as it reads at
/// this moment, and `changed` is PORTSC1.CSC: the controller's latched record
/// that a device connected or disconnected since the flag was last cleared.
///
/// Comparing `previous` with `now` finds a device that arrived or left. It
/// cannot find one that left *and came back* between two polls, and the
/// stream is not polled while an enumeration is in flight. Some devices do
/// exactly that: a Donner StarryCtrl (Jieli chipset) drops off the bus after
/// its first reset and request, then re-attaches. The level reads "connected"
/// before and after, but the controller has disabled the port and the device
/// has lost its state, so every request fails until the port is reset again.
/// The latched flag is the only evidence, and it turns the bounce into a
/// disconnect now and, on the next poll, a connect.
///
/// A change of speed alone is never reported. EHCI reads full speed from the
/// line state before a port reset and high speed from the chirp after it, and
/// reporting that would make cotton-usb-host reset and enumerate a second
/// time.
fn port_event(previous: DeviceStatus, now: DeviceStatus, changed: bool) -> Option<DeviceStatus> {
    let was_connected = matches!(previous, DeviceStatus::Present(_));
    let is_connected = matches!(now, DeviceStatus::Present(_));

    if was_connected != is_connected {
        Some(now)
    } else if is_connected && changed {
        Some(DeviceStatus::Absent)
    } else {
        None
    }
}

// ---------------------------------------------------------------------------
// ImxrtDeviceDetect — Stream<Item = DeviceStatus>
// ---------------------------------------------------------------------------

/// Device detection stream for the i.MX RT 1062 USB host controller.
///
/// Monitors the root port for connect/disconnect events by polling PORTSC1.
/// Yields `DeviceStatus::Present(speed)` when a device is connected, and
/// `DeviceStatus::Absent` when disconnected.
///
/// Follows the RP2040 pattern: stores the previous status and only returns
/// `Ready` when the status changes. In addition it reads the controller's
/// latched connect-change flag, so a device that disconnects and reconnects
/// between two polls is reported as `Absent` and then `Present`; see
/// [`port_event`].
#[derive(Copy, Clone)]
pub struct ImxrtDeviceDetect {
    /// USB OTG register block base address (stored as `u32` to keep the struct
    /// `Send` without a manual `unsafe impl`).
    usb_base: u32,
    /// USB PHY register block base address.
    usbphy_base: u32,
    waker: &'static CriticalSectionWakerRegistration,
    status: DeviceStatus,
}

impl ImxrtDeviceDetect {
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

    /// Clear the latched connect-change flag, and nothing else.
    ///
    /// PORTSC1 mixes ordinary bits with write-one-to-clear flags, so the value
    /// written back has every such flag masked out except the one being
    /// cleared.
    fn clear_connect_change(&self, portsc: u32) {
        let usb = self.usb_instance();
        ral::write_reg!(
            ral::usb,
            usb,
            PORTSC1,
            (portsc & !ImxrtHostController::PORTSC1_W1C_MASK) | ral::usb::PORTSC1::CSC::mask
        );
    }

    /// Re-enable the port change interrupt.
    fn reenable_interrupt(&self) {
        let usb = self.usb_instance();
        ral::modify_reg!(ral::usb, usb, USBINTR, PCE: 1);
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

impl Stream for ImxrtDeviceDetect {
    type Item = DeviceStatus;

    fn poll_next(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<Self::Item>> {
        self.waker.register(cx.waker());

        let usb = self.usb_instance();
        let portsc = ral::read_reg!(ral::usb, usb, PORTSC1);
        let device_status = status_from_portsc(portsc);
        let changed = portsc & ral::usb::PORTSC1::CSC::mask != 0;

        // The flag has been read; clear it so that the next one to be seen is
        // a new event.
        if changed {
            self.clear_connect_change(portsc);
        }

        match port_event(self.status, device_status, changed) {
            Some(report) => {
                if report != device_status {
                    debug!(
                        "[HC] DeviceDetect: device left and came back  PORTSC1=0x{:08X}",
                        portsc
                    );
                } else {
                    debug!("[HC] DeviceDetect: status change  PORTSC1=0x{:08X}", portsc);
                }

                // Manage ENHOSTDISCONDETECT based on connection state.
                // Per i.MX RT reference manual and USBHost_t36: set only when a
                // High Speed device is connected (HSP=1), clear on disconnect.
                match report {
                    DeviceStatus::Present(UsbSpeed::High480) => {
                        self.set_enhostdiscondetect();
                        debug!("[HC] ENHOSTDISCONDETECT set (HS device connected)");
                    }
                    // Absent, or a FS/LS device: the disconnect detector is off.
                    _ => self.clear_enhostdiscondetect(),
                }

                self.reenable_interrupt();
                self.status = report;
                Poll::Ready(Some(report))
            }
            None => {
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
}

#[cfg(test)]
mod tests {
    use super::*;

    const CCS: u32 = ral::usb::PORTSC1::CCS::mask;
    const PSPD_SHIFT: u32 = ral::usb::PORTSC1::PSPD::offset;

    const ABSENT: DeviceStatus = DeviceStatus::Absent;
    const FULL: DeviceStatus = DeviceStatus::Present(UsbSpeed::Full12);
    const LOW: DeviceStatus = DeviceStatus::Present(UsbSpeed::Low1_5);
    const HIGH: DeviceStatus = DeviceStatus::Present(UsbSpeed::High480);

    #[test]
    fn decode_nothing_connected() {
        assert!(status_from_portsc(0) == ABSENT);
        // Speed bits read 3 with nothing attached; still absent.
        assert!(status_from_portsc(3 << PSPD_SHIFT) == ABSENT);
    }

    #[test]
    fn decode_each_speed() {
        assert!(status_from_portsc(CCS) == FULL);
        assert!(status_from_portsc(CCS | (1 << PSPD_SHIFT)) == LOW);
        assert!(status_from_portsc(CCS | (2 << PSPD_SHIFT)) == HIGH);
    }

    #[test]
    fn decode_values_seen_on_the_bench() {
        // Teensy 4.1, full-speed device: at attach, and enabled after reset.
        assert!(status_from_portsc(0x1000_1803) == FULL);
        assert!(status_from_portsc(0x1000_1807) == FULL);
        // After an unplug.
        assert!(status_from_portsc(0x1C00_100A) == ABSENT);
    }

    #[test]
    fn a_device_arriving_is_reported() {
        assert!(port_event(ABSENT, FULL, true) == Some(FULL));
        // The level alone is enough; the flag may already have been cleared.
        assert!(port_event(ABSENT, FULL, false) == Some(FULL));
    }

    #[test]
    fn a_device_leaving_is_reported() {
        assert!(port_event(FULL, ABSENT, true) == Some(ABSENT));
        assert!(port_event(HIGH, ABSENT, false) == Some(ABSENT));
    }

    #[test]
    fn nothing_happening_reports_nothing() {
        assert!(port_event(ABSENT, ABSENT, false).is_none());
        assert!(port_event(FULL, FULL, false).is_none());
    }

    #[test]
    fn a_speed_change_alone_reports_nothing() {
        // Full speed before the port reset, high speed after the chirp.
        assert!(port_event(FULL, HIGH, false).is_none());
    }

    #[test]
    fn a_device_that_left_and_came_back_is_reported_absent_first() {
        assert!(port_event(FULL, FULL, true) == Some(ABSENT));
        // Whatever speed it came back at.
        assert!(port_event(FULL, HIGH, true) == Some(ABSENT));
    }

    #[test]
    fn after_a_bounce_the_next_poll_reports_the_device() {
        // First poll: flag set, reported absent, flag cleared.
        let first = port_event(FULL, FULL, true);
        assert!(first == Some(ABSENT));
        // Second poll: status is now absent, the device reads present.
        assert!(port_event(ABSENT, FULL, false) == Some(FULL));
    }

    #[test]
    fn a_device_that_came_and_went_reports_nothing() {
        // Nothing was reported as present, so there is nothing to retract.
        assert!(port_event(ABSENT, ABSENT, true).is_none());
    }
}
