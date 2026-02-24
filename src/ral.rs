//! Register access layer for USB host mode.
//!
//! Re-exports register definitions from `imxrt-ral` for USB core and PHY
//! peripherals. Provides thin `Instance` wrapper types that erase the
//! const-generic instance number from `imxrt-ral`, keeping the rest of
//! the driver non-generic.
//!
//! The `ral-registers` macros (`read_reg!`/`write_reg!`/`modify_reg!`) are
//! re-exported from `imxrt-ral` and work with these wrapper types because
//! they implement `Deref<Target = RegisterBlock>`.
//!
//! The USB core register block includes both device-mode and host-mode field
//! definitions at the same offsets. Key host-mode mappings:
//!
//! | EHCI Name | RAL Register | RAL Field | Purpose |
//! |-----------|-------------|-----------|---------|
//! | `PERIODICLISTBASE` | `DEVICEADDR` | (raw write) | Periodic frame list base address |
//! | `ASYNCLISTADDR` | `ASYNCLISTADDR` | `ASYBASE` | Async schedule list pointer |
//! | `PORTSC` | `PORTSC1` | `PSPD`, `CCS`, `PE`, etc. | Port status and control |

pub use imxrt_ral::{modify_reg, read_reg, write_reg};

/// USB OTG core registers and field definitions.
///
/// All register field sub-modules (`USBCMD`, `PORTSC1`, `USBSTS`, etc.) are
/// re-exported from `imxrt-ral`. The [`Instance`] type is a non-generic
/// wrapper that erases `imxrt-ral`'s const-generic instance number.
pub mod usb {
    // Glob-import all register field modules and RegisterBlock from imxrt-ral.
    // Our local `Instance` struct shadows `imxrt_ral::usb::Instance<N>`.
    pub use imxrt_ral::usb::*;

    /// A non-generic USB register instance.
    ///
    /// Wraps a raw pointer to `RegisterBlock`, providing `Deref` so it works
    /// with `read_reg!`/`write_reg!`/`modify_reg!` macros. This erases the
    /// const-generic `N` from `imxrt_ral::Instance<T, N>` so the driver's
    /// internal types don't need to be generic over the instance number.
    pub struct Instance {
        pub(crate) addr: *const RegisterBlock,
    }

    impl Instance {
        /// Create a wrapper from a typed `imxrt-ral` USB instance.
        pub fn from_ral<const N: u8>(inst: imxrt_ral::usb::Instance<N>) -> Self {
            // Extract the pointer before the instance is consumed.
            let ptr: &RegisterBlock = &inst;
            Self {
                addr: ptr as *const RegisterBlock,
            }
        }
    }

    impl core::ops::Deref for Instance {
        type Target = RegisterBlock;
        #[inline]
        fn deref(&self) -> &Self::Target {
            unsafe { &*self.addr }
        }
    }

    // Safety: Instance holds a pointer to static MMIO registers.
    // The driver is designed for single-task usage with ISR synchronization.
    unsafe impl Send for Instance {}
}

/// USB PHY registers and field definitions.
///
/// All register field sub-modules (`PWD`, `CTRL_SET`, `CTRL_CLR`, etc.) are
/// re-exported from `imxrt-ral`. The [`Instance`] type is a non-generic
/// wrapper analogous to [`usb::Instance`].
pub mod usbphy {
    pub use imxrt_ral::usbphy::*;

    /// A non-generic USBPHY register instance.
    pub struct Instance {
        pub(crate) addr: *const RegisterBlock,
    }

    impl Instance {
        /// Create a wrapper from a typed `imxrt-ral` USBPHY instance.
        pub fn from_ral<const N: u8>(inst: imxrt_ral::usbphy::Instance<N>) -> Self {
            let ptr: &RegisterBlock = &inst;
            Self {
                addr: ptr as *const RegisterBlock,
            }
        }
    }

    impl core::ops::Deref for Instance {
        type Target = RegisterBlock;
        #[inline]
        fn deref(&self) -> &Self::Target {
            unsafe { &*self.addr }
        }
    }

    unsafe impl Send for Instance {}
}
