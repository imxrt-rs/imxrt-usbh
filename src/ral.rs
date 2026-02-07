//! Register access layer for USB host mode.
//!
//! This module provides register definitions for the i.MX RT USB OTG controller
//! and USB PHY peripherals. The definitions are sourced from `imxrt-usbd` and use
//! the `ral-registers` crate for `read_reg!`/`write_reg!`/`modify_reg!` macros.
//!
//! The USB core register block (`usb::RegisterBlock`) includes both device-mode and
//! host-mode field definitions at the same offsets. Key host-mode mappings:
//!
//! | EHCI Name | RAL Register | RAL Field | Purpose |
//! |-----------|-------------|-----------|---------|
//! | `PERIODICLISTBASE` | `DEVICEADDR` | `BASEADR` | Periodic frame list base address |
//! | `ASYNCLISTADDR` | `ASYNCLISTADDR` | `ASYBASE` | Async schedule list pointer |
//! | `PORTSC` | `PORTSC1` | `PSPD`, `CCS`, `PE`, etc. | Port status and control |

pub mod usb;
pub mod usbphy;

pub use ral_registers::{modify_reg, read_reg, write_reg, RORegister, RWRegister};

use crate::Peripherals;

/// Typed register block instances for USB core and PHY peripherals.
pub struct Instances {
    /// USB OTG core registers (EHCI-compatible).
    pub usb: usb::Instance,
    /// USB PHY registers.
    pub usbphy: usbphy::Instance,
}

/// Convert a [`Peripherals`] implementation into typed register block instances.
///
/// This consumes the `Peripherals` and returns [`Instances`] with typed pointers
/// to the USB core and PHY register blocks.
pub fn instances<P: Peripherals>(peripherals: P) -> Instances {
    let usb = usb::Instance {
        addr: peripherals.usb().cast(),
    };
    let usbphy = usbphy::Instance {
        addr: peripherals.usbphy().cast(),
    };
    Instances { usb, usbphy }
}
