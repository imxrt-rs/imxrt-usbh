//! A USB **host** driver for i.MX RT processors.
//!
//! `imxrt-usbh` provides a USB host controller implementation for i.MX RT 1060/1062
//! microcontrollers (e.g. Teensy 4.1). It drives the EHCI-compatible USB OTG peripheral
//! in host mode.
//!
//! # Getting started
//!
//! To use this library, implement the [`Peripherals`] trait to provide pointers to the
//! USB core and USB PHY register blocks. See the trait documentation for details.
//!
//! # Clock configuration
//!
//! The driver does **not** configure any CCM or CCM_ANALOG registers. You are responsible
//! for configuring PLLs and clock gates for proper USB functionality before initialising
//! the host controller.
//!
//! # Debugging features
//!
//! Enable the `defmt-03` feature to activate internal logging using defmt (version 0.3).

#![no_std]
#![warn(unsafe_op_in_unsafe_fn)]

#[macro_use]
mod log;

mod cache;

/// EHCI data structures: Queue Heads, Transfer Descriptors, and frame lists.
///
/// These are the hardware-defined DMA structures that the EHCI controller
/// reads and writes directly. See the EHCI specification sections 3.5 (qTD)
/// and 3.6 (QH) for the canonical layout.
pub mod ehci;

/// USB host controller driver and supporting types.
///
/// Start here: [`Imxrt1062HostController`](crate::host::Imxrt1062HostController) is
/// the main driver. [`UsbShared`](crate::host::UsbShared) and
/// [`UsbStatics`](crate::host::UsbStatics) provide the static resources it needs.
pub mod host;

pub(crate) mod ral;
mod vcell;

/// General Purpose Timer (GPT) abstraction.
///
/// Thin wrapper around the USB OTG controller's built-in general purpose
/// timers (GPT0 and GPT1). Currently unused by the host driver but available
/// for timeout or watchdog functionality.
pub mod gpt;

/// A type that owns all USB register blocks.
///
/// An implementation of `Peripherals` is expected to own the USB1 or USB2
/// registers. This includes:
///
/// - USB core registers (the EHCI-compatible OTG register block)
/// - USB PHY registers
///
/// When an instance of `Peripherals` exists, you must make sure that nothing
/// else accesses those registers.
///
/// # Safety
///
/// `Peripherals` should only be implemented on a type that owns the various
/// register blocks required for USB operation. Incorrect usage, or failure to
/// ensure exclusive ownership, could lead to data races and incorrect USB
/// functionality.
///
/// All pointers must point at the starting register block for the specified
/// peripheral. Calls to the functions must return the same value every time
/// they're called.
///
/// # Example
///
/// A safe implementation of `Peripherals` that works with the `imxrt-ral`
/// register access layer:
///
/// ```no_run
/// # mod imxrt_ral {
/// #   pub struct RegisterBlock;
/// #   use core::ops::Deref; pub struct Instance; impl Deref for Instance { type Target = RegisterBlock; fn deref(&self) -> &RegisterBlock { unsafe { &*(0x402e0200 as *const RegisterBlock)} } }
/// #   pub fn take() -> Result<Instance, ()> { Ok(Instance) }
/// #   pub mod usb { pub use super::{Instance, RegisterBlock}; pub mod USB1 { pub use super::super::take; } }
/// #   pub mod usbphy { pub use super::{Instance, RegisterBlock}; pub mod USBPHY1 { pub use super::super::take; } }
/// # }
/// use imxrt_ral as ral;
///
/// struct Peripherals {
///     usb: ral::usb::Instance,
///     phy: ral::usbphy::Instance,
/// }
///
/// impl Peripherals {
///     /// Panics if the instances are already taken.
///     fn usb1() -> Peripherals {
///         Self {
///             usb: ral::usb::USB1::take().unwrap(),
///             phy: ral::usbphy::USBPHY1::take().unwrap(),
///         }
///     }
/// }
///
/// // SAFETY: `Peripherals` owns the imxrt-ral singleton instances,
/// // which are guaranteed to be unique. No one else can safely access
/// // the USB registers while this object exists.
/// unsafe impl imxrt_usbh::Peripherals for Peripherals {
///     fn usb(&self) -> *const () {
///         let rb: &ral::usb::RegisterBlock = &self.usb;
///         (rb as *const ral::usb::RegisterBlock).cast()
///     }
///     fn usbphy(&self) -> *const () {
///         let rb: &ral::usbphy::RegisterBlock = &self.phy;
///         (rb as *const ral::usbphy::RegisterBlock).cast()
///     }
/// }
/// ```
pub unsafe trait Peripherals {
    /// Returns the pointer to the USB OTG core register block.
    fn usb(&self) -> *const ();
    /// Returns the pointer to the USB PHY register block.
    fn usbphy(&self) -> *const ();
}
