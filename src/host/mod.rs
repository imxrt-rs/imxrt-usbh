//! USB host controller driver for i.MX RT EHCI.
//!
//! This module contains the core types for the USB host controller:
//!
//! - [`UsbShared`](crate::host::UsbShared) — interrupt-safe state shared between ISR and async tasks.
//! - [`UsbStatics`](crate::host::UsbStatics) — static-lifetime resource pools (not shared with ISR).
//! - [`Imxrt1062HostController`](crate::host::Imxrt1062HostController) — the main controller implementing
//!   [`HostController`](cotton_usb_host::host_controller::HostController).
//!
//! # Architecture
//!
//! The design follows the pattern established by the RP2040 host controller in
//! `cotton-usb-host`:
//!
//! ```text
//!   static UsbShared      <---- ISR calls on_irq(), wakes pipe/device wakers
//!   static UsbStatics     <---- Pool-based pipe allocation (not ISR-accessed)
//!   Imxrt1062HostController --> owns register blocks, references shared/statics
//! ```
//!
//! Both `UsbShared` and `UsbStatics` are `const`-constructible and designed to
//! live in `static` storage (typically via `ConstStaticCell`).
//!
//! # Module Structure
//!
//! The implementation is split across several submodules:
//!
//! - `shared` — ISR <-> async bridge (`UsbShared`)
//! - `statics` — resource pools and DMA buffers (`UsbStatics`, `RecvBuf`)
//! - `controller` — struct definition, construction, and initialization
//! - `schedule` — QH/qTD allocation and EHCI schedule management
//! - `transfer` — control, bulk, and interrupt transfer implementations
//! - `futures` — async futures for transfer/doorbell completion
//! - `device_detect` — root port device detection stream
//! - `interrupt_pipe` — interrupt IN pipe implementation
//! - `trait_impl` — `HostController` trait implementation

// ---------------------------------------------------------------------------
// Submodules
// ---------------------------------------------------------------------------

mod shared;
mod statics;
mod controller;
mod schedule;
mod transfer;
mod futures;
mod device_detect;
mod interrupt_pipe;
mod trait_impl;

// ---------------------------------------------------------------------------
// Re-exports (public API)
// ---------------------------------------------------------------------------

pub use shared::UsbShared;
pub use statics::{RecvBuf, UsbStatics};
pub use controller::Imxrt1062HostController;
pub use device_detect::Imxrt1062DeviceDetect;
pub use interrupt_pipe::Imxrt1062InterruptPipe;

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
/// Each control transfer uses 2-3 qTDs (setup + data + status).
/// Each bulk/interrupt transfer uses 1-N qTDs.
/// 16 qTDs supports 1 control + 3 concurrent bulk, with room for chaining.
pub const NUM_QTD: usize = 16;

/// Number of pipe wakers.
///
/// Must be >= `NUM_QH + 1` (control pipe at index 0, bulk/interrupt at indices 1..N).
/// We use NUM_QH + 1 to match: 1 control + NUM_QH bulk/interrupt slots.
const NUM_PIPE_WAKERS: usize = NUM_QH + 1;



