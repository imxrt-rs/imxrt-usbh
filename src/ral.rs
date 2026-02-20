//! Register access layer for USB host mode.
//!
//! These definitions are vendored from `imxrt-usbd` (which itself derives from
//! `imxrt-ral`) because the upstream `imxrt-ral` crate models USB registers for
//! device mode only. The EHCI host-mode registers (ASYNCLISTADDR,
//! PERIODICLISTBASE, PORTSC1 host bits, FRINDEX, etc.) are either absent or
//! incorrect in the SVD-generated definitions. Additionally, the [`Instance`]
//! types here implement `Send`, which is required for sharing register access
//! across async task boundaries and ISR contexts.
//!
//! If `imxrt-ral` gains proper USB host-mode support in the future, these
//! files should be replaced with upstream imports.
//!
//! The `ral-registers` crate provides the `read_reg!`/`write_reg!`/`modify_reg!`
//! macros used throughout.
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
