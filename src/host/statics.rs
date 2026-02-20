//! Static-lifetime resource pools and DMA receive buffers.

use core::cell::{Cell, UnsafeCell};
use crate::ehci::{FrameList, QueueHead, TransferDescriptor};
use cotton_usb_host::async_pool::Pool;

use super::{NUM_QH, NUM_QTD};

// ---------------------------------------------------------------------------
// RecvBuf — DMA-aligned receive buffer
// ---------------------------------------------------------------------------

/// A 32-byte-aligned receive buffer for DMA.
///
/// Each buffer is 64 bytes (2 cache lines). The 32-byte alignment ensures
/// that cache maintenance operations on a receive buffer cannot corrupt
/// adjacent data in the same cache line.
#[repr(C, align(32))]
pub struct RecvBuf(pub [u8; 64]);

impl RecvBuf {
    /// Create a zeroed receive buffer.
    pub const fn new() -> Self {
        Self([0u8; 64])
    }

    /// Pointer to the start of the buffer.
    pub fn as_ptr(&self) -> *const u8 {
        self.0.as_ptr()
    }

    /// Length of the buffer.
    pub const fn len(&self) -> usize {
        64
    }

    /// Returns `true` if the buffer has zero length (always `false`).
    pub const fn is_empty(&self) -> bool {
        false
    }
}

impl AsRef<[u8]> for RecvBuf {
    fn as_ref(&self) -> &[u8] {
        &self.0
    }
}

impl core::ops::Index<usize> for RecvBuf {
    type Output = u8;
    fn index(&self, idx: usize) -> &u8 {
        &self.0[idx]
    }
}

impl core::ops::Index<core::ops::RangeTo<usize>> for RecvBuf {
    type Output = [u8];
    fn index(&self, range: core::ops::RangeTo<usize>) -> &[u8] {
        &self.0[range]
    }
}

// ---------------------------------------------------------------------------
// UsbStatics — static resource pools
// ---------------------------------------------------------------------------

/// Static-lifetime resource pools for the USB host controller.
///
/// This struct owns the pre-allocated pools of QHs, qTDs, the periodic
/// frame list, and DMA receive buffers. It is **not** accessed from the
/// ISR — only from async task context.
///
/// # Placement
///
/// Must live in a `static` (typically via `ConstStaticCell`) because:
/// - Pipe allocations (`Pooled`) borrow the pool with `'static` lifetime
/// - DMA structures must have stable addresses for the controller
///
/// # Memory
///
/// All DMA-visible arrays (`qh_pool`, `qtd_pool`, `frame_list`) must be in
/// normal RAM (not TCM) if using cache management, or in DTCM if bypassing
/// the data cache.
pub struct UsbStatics {
    /// Pool for control pipe slots (1 slot — only one EP0 at a time).
    pub control_pipes: Pool,

    /// Pool for bulk/interrupt pipe slots.
    pub bulk_pipes: Pool,

    /// Pre-allocated Queue Head storage (interior-mutable for DMA).
    ///
    /// Wrapped in `UnsafeCell` because QHs are shared-mutable DMA structures:
    /// the CPU writes to initialise them and the EHCI controller reads/writes
    /// them concurrently via DMA.  `UnsafeCell` makes this intent explicit and
    /// eliminates the `const→mut` pointer cast that was previously needed.
    ///
    /// Index 0 is reserved for the async schedule sentinel.
    /// Index 1 is reserved for the single control pipe (EP0).
    /// Indices 2..=NUM_QH are for bulk/interrupt pipes (NUM_QH−1 slots).
    pub qh_pool: [UnsafeCell<QueueHead>; NUM_QH + 1], // +1 for sentinel

    /// Pre-allocated Transfer Descriptor storage (interior-mutable for DMA).
    ///
    /// Wrapped in `UnsafeCell` for the same reason as `qh_pool`: qTDs are
    /// written by the CPU and read/written by the EHCI DMA engine.
    pub qtd_pool: [UnsafeCell<TransferDescriptor>; NUM_QTD],

    /// Periodic frame list (4096-byte aligned).
    pub frame_list: FrameList,

    /// qTD allocation bitmap.
    ///
    /// `true` means the slot is allocated. This separates allocation tracking
    /// from the hardware `token` field, avoiding an unsafe write to mark a
    /// slot as in-use during allocation.
    pub qtd_allocated: [Cell<bool>; NUM_QTD],

    /// Receive buffers for interrupt pipes.
    ///
    /// One 64-byte buffer per interrupt pipe slot, 32-byte aligned for cache
    /// line safety. These live in a `static` so their addresses are stable
    /// for the DMA engine. Index matches the `bulk_pipes` pool token
    /// (0..NUM_QH-2).
    pub recv_bufs: [RecvBuf; NUM_QH - 1],
}

impl UsbStatics {
    /// Create a new `UsbStatics` with all resources free and structures zeroed.
    ///
    /// This is `const` so it can be placed in a `static`.
    pub const fn new() -> Self {
        Self {
            control_pipes: Pool::new(1),
            // NUM_QH - 1 slots: indices 2..=NUM_QH in qh_pool (index 0 = sentinel, 1 = control)
            bulk_pipes: Pool::new((NUM_QH - 1) as u8),
            qh_pool: {
                const QH: UnsafeCell<QueueHead> = UnsafeCell::new(QueueHead::new());
                [QH; NUM_QH + 1]
            },
            qtd_pool: {
                const QTD: UnsafeCell<TransferDescriptor> = UnsafeCell::new(TransferDescriptor::new());
                [QTD; NUM_QTD]
            },
            frame_list: FrameList::new(),
            qtd_allocated: {
                // Cell<bool> doesn't have a const new(), so use array init
                const FREE: Cell<bool> = Cell::new(false);
                [FREE; NUM_QTD]
            },
            recv_bufs: {
                const BUF: RecvBuf = RecvBuf::new();
                [BUF; NUM_QH - 1]
            },
        }
    }

    /// Get a mutable pointer to a QH from the pool.
    ///
    /// Returns `*mut QueueHead` via `UnsafeCell::get()`, which is sound for
    /// shared-mutable DMA structures.  The caller must ensure no aliasing
    /// `&mut QueueHead` references exist for the same index.
    ///
    /// # Panics
    ///
    /// Panics if `index >= NUM_QH + 1` (i.e. `index > 4`).
    #[inline]
    pub fn qh_ptr(&self, index: usize) -> *mut QueueHead {
        self.qh_pool[index].get()
    }

    /// Get a mutable pointer to a qTD from the pool.
    ///
    /// Returns `*mut TransferDescriptor` via `UnsafeCell::get()`.  Same
    /// aliasing requirements as [`qh_ptr`](Self::qh_ptr).
    ///
    /// # Panics
    ///
    /// Panics if `index >= NUM_QTD` (i.e. `index > 15`).
    #[inline]
    pub fn qtd_ptr(&self, index: usize) -> *mut TransferDescriptor {
        self.qtd_pool[index].get()
    }
}
