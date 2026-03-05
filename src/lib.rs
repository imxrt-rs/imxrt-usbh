//! A USB **host** driver for i.MX RT processors.
//!
//! `imxrt-usbh` provides a USB host controller implementation for i.MX RT
//! microcontrollers (e.g. Teensy 4.1). It drives the EHCI-compatible USB OTG
//! peripheral in host mode.
//!
//! # Getting started
//!
//! Create an [`ImxrtHostController`](crate::host::ImxrtHostController)
//! by passing `imxrt-ral` USB and USBPHY register instances along with static
//! resource pools. See the controller documentation for details.
//!
//! # Chip support
//!
//! This crate uses `imxrt-ral` for register access. Enable exactly one chip
//! feature on `imxrt-ral`:
//!
//! ```toml
//! [dependencies]
//! imxrt-usbh = "0.1"
//! imxrt-ral = { version = "0.6", features = ["imxrt1062"] }
//! ```
//!
//! # Clock configuration
//!
//! The driver does **not** configure any CCM or CCM_ANALOG registers. You are
//! responsible for configuring PLLs and clock gates for proper USB functionality
//! before initialising the host controller.
//!
//! # Debugging features
//!
//! Enable the `defmt-03` feature to activate internal logging using defmt (version 0.3).

#![cfg_attr(target_os = "none", no_std)]
#![warn(unsafe_op_in_unsafe_fn)]

#[macro_use]
mod log;

/// EHCI data structures: Queue Heads, Transfer Descriptors, and frame lists.
///
/// These are the hardware-defined DMA structures that the EHCI controller
/// reads and writes directly. See the EHCI specification sections 3.5 (qTD)
/// and 3.6 (QH) for the canonical layout.
pub mod ehci;

/// USB host controller driver and supporting types.
///
/// Start here: [`ImxrtHostController`](crate::host::ImxrtHostController) is
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

// ── Cotton USB host stack re-exports ─────────────────────────────────
//
// Re-export the consumer-facing modules from `cotton-usb-host` so that
// users of this crate can reach USB bus types and descriptor types
// without adding `cotton-usb-host` as a direct dependency.

/// The [`HostController`](host_controller::HostController) trait that
/// this crate implements.
///
/// Most users interact through [`usb_bus::UsbBus`] rather than the trait
/// directly, but it is available for generic code or alternative
/// implementations.
pub use cotton_usb_host::host_controller;

/// High-level USB bus abstraction and device events.
///
/// Key types: [`UsbBus`](usb_bus::UsbBus),
/// [`DeviceEvent`](usb_bus::DeviceEvent).
pub use cotton_usb_host::usb_bus;

/// USB descriptor types straight from the USB specification.
///
/// Includes [`SetupPacket`](wire::SetupPacket),
/// [`ConfigurationDescriptor`](wire::ConfigurationDescriptor),
/// [`InterfaceDescriptor`](wire::InterfaceDescriptor),
/// [`EndpointDescriptor`](wire::EndpointDescriptor), and the
/// [`DescriptorVisitor`](wire::DescriptorVisitor) trait.
pub use cotton_usb_host::wire;
