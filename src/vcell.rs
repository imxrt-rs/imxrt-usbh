//! Volatile cell with interior mutability for DMA-accessible memory.
//!
//! `VCell` wraps a value `T` behind `UnsafeCell` and performs all reads
//! and writes using volatile operations. This makes it suitable for
//! EHCI DMA structures (QueueHead, TransferDescriptor, FrameList) where:
//!
//! - Hardware (the EHCI DMA engine) reads and writes the memory
//! - Software needs volatile access to observe hardware changes
//! - Both `&self` (shared) and `&mut self` access patterns are needed
//!
//! `VCell` is `!Sync` by default (due to `UnsafeCell`), which is correct
//! for DMA structures — synchronization is handled by EHCI schedule
//! management and non-cacheable memory placement.

use core::cell::UnsafeCell;
use core::ptr;

/// A memory location that requires volatile reads and writes.
///
/// Uses `UnsafeCell` internally to support interior mutability, allowing
/// volatile writes through shared references. This is necessary for
/// DMA structures behind `&'static` references (e.g. `UsbStatics`).
#[repr(transparent)]
pub struct VCell<T>(UnsafeCell<T>);

impl<T> VCell<T> {
    /// Construct a `VCell` that's initialized to `val`
    pub const fn new(val: T) -> Self {
        VCell(UnsafeCell::new(val))
    }
}

impl<T: Copy> VCell<T> {
    /// Perform a volatile read from this memory location
    pub fn read(&self) -> T {
        // Safety: volatile read from a valid, initialized memory location.
        unsafe { ptr::read_volatile(self.0.get()) }
    }
    /// Perform a volatile write at this memory location
    ///
    /// Takes `&self` (not `&mut self`) because DMA structures often need
    /// to be written through shared references. The volatile semantics
    /// ensure the write is not elided or reordered by the compiler.
    pub fn write(&self, val: T) {
        // Safety: volatile write to a valid memory location. The caller
        // is responsible for ensuring no data races (via EHCI schedule
        // management and non-cacheable memory placement).
        unsafe { ptr::write_volatile(self.0.get(), val) }
    }
}
