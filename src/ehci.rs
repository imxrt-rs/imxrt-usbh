//! EHCI data structures for DMA-based USB transfers.
//!
//! This module defines the Queue Head (QH) and Queue Element Transfer Descriptor (qTD)
//! structures used by the EHCI controller's DMA engine. These structures are placed in
//! RAM and accessed directly by hardware — strict alignment, layout, and cache coherency
//! rules apply.
//!
//! # Alignment Requirements
//!
//! - **QH**: Must be 64-byte aligned. The EHCI spec requires 32-byte alignment for the
//!   horizontal link pointer, but the QH is 48 bytes of hardware-visible fields plus
//!   software fields, so we use 64 bytes to avoid straddling two QHs.
//! - **qTD**: Must be 32-byte aligned (EHCI §3.5), which also matches the Cortex-M7
//!   cache line size (32 bytes).
//! - **Frame List**: Must be 4096-byte aligned (page-aligned). Size is configurable
//!   (8, 16, 32, … 1024 entries); we use [`FRAME_LIST_LEN`](crate::ehci::FRAME_LIST_LEN) entries.
//!
//! # Non-Cacheable Memory Requirement
//!
//! The implementation assumes that all global memory allocated for DMA structures
//! (QH, qTD, frame list, data buffers) is placed in non-cacheable memory. The
//! driver will not flush any cache lines before coordinating with the DMA
//! controller. Users must either disable the D-cache or use the MPU to mark
//! DMA regions as non-cacheable.
//!
//! # References
//!
//! - EHCI Specification §3.5 (qTD), §3.6 (QH)
//! - i.MX RT 1060 Reference Manual, Chapter 56 (USB OTG)

use crate::vcell::VCell;

// ---------------------------------------------------------------------------
// Link pointer helpers
// ---------------------------------------------------------------------------

/// Terminate bit — indicates an invalid (end-of-list) pointer.
pub const LINK_TERMINATE: u32 = 1 << 0;

/// Link pointer type field values (bits \[2:1\]).
pub mod link_type {
    /// Isochronous Transfer Descriptor (iTD).
    pub const ITD: u32 = 0b00 << 1;
    /// Queue Head (QH).
    pub const QH: u32 = 0b01 << 1;
    /// Split Transaction Isochronous Transfer Descriptor (siTD).
    pub const SITD: u32 = 0b10 << 1;
    /// Frame Span Traversal Node (FSTN).
    pub const FSTN: u32 = 0b11 << 1;
}

/// Build a link pointer from a 32-byte-aligned physical address and a type.
///
/// The low 5 bits of `addr` are replaced with `typ | terminate`.
#[inline]
pub const fn link_pointer(addr: u32, typ: u32) -> u32 {
    (addr & !0x1F) | typ
}

/// Extract the 32-byte-aligned address from a link pointer.
#[inline]
pub const fn link_address(link: u32) -> u32 {
    link & !0x1F
}

/// Returns `true` if the terminate bit is set.
#[inline]
pub const fn link_is_terminate(link: u32) -> bool {
    link & LINK_TERMINATE != 0
}

// ---------------------------------------------------------------------------
// PID codes (used in qTD token and QH overlay)
// ---------------------------------------------------------------------------

/// PID code for OUT token (host → device).
pub const PID_OUT: u32 = 0;
/// PID code for IN token (device → host).
pub const PID_IN: u32 = 1;
/// PID code for SETUP token (control transfers).
pub const PID_SETUP: u32 = 2;

// ---------------------------------------------------------------------------
// Endpoint speed codes (used in QH endpoint characteristics)
// ---------------------------------------------------------------------------

/// Full Speed (12 Mbps).
pub const SPEED_FULL: u32 = 0;
/// Low Speed (1.5 Mbps).
pub const SPEED_LOW: u32 = 1;
/// High Speed (480 Mbps).
pub const SPEED_HIGH: u32 = 2;

// ---------------------------------------------------------------------------
// Queue Element Transfer Descriptor (qTD) — EHCI §3.5
// ---------------------------------------------------------------------------

/// Queue Element Transfer Descriptor (qTD).
///
/// A 32-byte, 32-byte-aligned structure that describes a single data transfer
/// phase. Multiple qTDs are chained via [`next`](Self::next) pointers to form a
/// transfer. The EHCI controller processes them in order, updating the
/// [`token`](Self::token) field as it goes.
///
/// # Layout (EHCI §3.5)
///
/// | Word | Field | Description |
/// |------|-------|-------------|
/// | 0 | `next` | Next qTD pointer (T-bit terminates) |
/// | 1 | `alt_next` | Alternate next qTD (used on short packet) |
/// | 2 | `token` | Status, PID, error count, bytes, data toggle, IOC |
/// | 3–7 | `buffer[5]` | Buffer page pointers (4K-aligned, `buffer[0]` has byte offset) |
#[repr(C, align(32))]
pub struct TransferDescriptor {
    /// Next qTD pointer. Set [`LINK_TERMINATE`] to end the chain.
    pub next: VCell<u32>,
    /// Alternate next qTD pointer. The controller follows this on a short packet
    /// (when `total_bytes` reaches 0 before expected). Usually set to terminate.
    pub alt_next: VCell<u32>,
    /// qTD token — contains status, PID code, error count, total bytes, data toggle, IOC.
    /// See [`QTD_TOKEN_ACTIVE`], [`qtd_token`], and the `qtd_token` field helpers.
    pub token: VCell<u32>,
    /// Buffer page pointer list (5 entries).
    ///
    /// `buffer[0]` contains the starting physical address (any alignment).
    /// `buffer[1..5]` must be 4K-page-aligned (address of the next page boundary).
    pub buffer: [VCell<u32>; 5],
}

// qTD token bit positions and masks
/// Active bit — set by software, cleared by controller on completion.
pub const QTD_TOKEN_ACTIVE: u32 = 1 << 7;
/// Halted bit — set by controller on error or STALL.
pub const QTD_TOKEN_HALTED: u32 = 1 << 6;
/// Data buffer error (overrun or underrun).
pub const QTD_TOKEN_BUFFER_ERR: u32 = 1 << 5;
/// Babble detected.
pub const QTD_TOKEN_BABBLE: u32 = 1 << 4;
/// Transaction error (timeout, CRC, bad PID, etc.).
pub const QTD_TOKEN_XACT_ERR: u32 = 1 << 3;
/// Missed micro-frame (split transaction).
pub const QTD_TOKEN_MISSED_UFRAME: u32 = 1 << 2;
/// Split transaction state.
pub const QTD_TOKEN_SPLIT_STATE: u32 = 1 << 1;
/// Ping state / error indicator.
pub const QTD_TOKEN_PING_ERR: u32 = 1 << 0;

/// Interrupt On Complete — generates an interrupt when this qTD completes.
pub const QTD_TOKEN_IOC: u32 = 1 << 15;

/// Mask for the status byte (bits \[7:0\]).
pub const QTD_TOKEN_STATUS_MASK: u32 = 0xFF;
/// Mask for all error bits (bits \[6:0\], excludes Active).
pub const QTD_TOKEN_ERROR_MASK: u32 = QTD_TOKEN_HALTED
    | QTD_TOKEN_BUFFER_ERR
    | QTD_TOKEN_BABBLE
    | QTD_TOKEN_XACT_ERR
    | QTD_TOKEN_MISSED_UFRAME;

/// Bit offset for PID code in the token word.
const QTD_TOKEN_PID_SHIFT: u32 = 8;
/// Bit offset for error counter in the token word.
const QTD_TOKEN_CERR_SHIFT: u32 = 10;
/// Bit offset for current page index in the token word.
/// Not used by the driver (hardware manages C_Page during DMA), but kept
/// for completeness alongside the other EHCI token field offsets.
#[allow(dead_code)]
const QTD_TOKEN_CPAGE_SHIFT: u32 = 12;
/// Bit offset for total bytes to transfer in the token word.
const QTD_TOKEN_TOTAL_BYTES_SHIFT: u32 = 16;
/// Bit offset for data toggle in the token word.
const QTD_TOKEN_DT_SHIFT: u32 = 31;

/// Maximum transfer size per qTD (5 pages × 4096 − 1 byte offset ≈ 20 KB,
/// but EHCI spec limits `total_bytes` to 0x4FFF = 20479 bytes).
pub const QTD_MAX_TRANSFER_SIZE: u32 = 0x4FFF;

/// Build a qTD token word.
///
/// # Arguments
/// - `pid` — [`PID_OUT`], [`PID_IN`], or [`PID_SETUP`]
/// - `total_bytes` — number of bytes to transfer (max [`QTD_MAX_TRANSFER_SIZE`])
/// - `data_toggle` — `true` for DATA1, `false` for DATA0
/// - `ioc` — interrupt on complete
#[inline]
pub const fn qtd_token(pid: u32, total_bytes: u32, data_toggle: bool, ioc: bool) -> u32 {
    let dt = if data_toggle { 1u32 } else { 0u32 };
    let ioc_bit = if ioc { QTD_TOKEN_IOC } else { 0 };
    QTD_TOKEN_ACTIVE
        | (pid << QTD_TOKEN_PID_SHIFT)
        | (3 << QTD_TOKEN_CERR_SHIFT) // 3 retries before halting
        | (total_bytes << QTD_TOKEN_TOTAL_BYTES_SHIFT)
        | (dt << QTD_TOKEN_DT_SHIFT)
        | ioc_bit
}

/// Extract the remaining `total_bytes` from a token word.
#[inline]
pub const fn qtd_token_bytes_remaining(token: u32) -> u32 {
    (token >> QTD_TOKEN_TOTAL_BYTES_SHIFT) & 0x7FFF
}

impl Default for TransferDescriptor {
    fn default() -> Self {
        Self::new()
    }
}

impl TransferDescriptor {
    /// Create a zeroed-out transfer descriptor (all fields 0, terminated).
    pub const fn new() -> Self {
        Self {
            next: VCell::new(LINK_TERMINATE),
            alt_next: VCell::new(LINK_TERMINATE),
            token: VCell::new(0),
            buffer: [
                VCell::new(0),
                VCell::new(0),
                VCell::new(0),
                VCell::new(0),
                VCell::new(0),
            ],
        }
    }

    /// Initialise this qTD for a transfer.
    ///
    /// # Arguments
    /// - `token` — pre-built token word (use [`qtd_token`])
    /// - `data` — pointer to the data buffer (or null for zero-length)
    /// - `len` — transfer length in bytes (must match `total_bytes` in token)
    ///
    /// # Safety
    /// - `data` must be valid for `len` bytes and remain valid until the controller
    ///   completes this qTD.
    /// - The qTD must be in non-cacheable memory accessible by the DMA engine.
    pub unsafe fn init(&mut self, token: u32, data: *const u8, _len: u32) {
        self.next.write(LINK_TERMINATE);
        self.alt_next.write(LINK_TERMINATE);
        self.token.write(token);

        let addr = data as u32;
        self.buffer[0].write(addr);
        // Remaining buffer pointers: next 4K page boundaries
        for i in 1..5 {
            self.buffer[i].write((addr & !0xFFF) + (i as u32) * 4096);
        }
    }

    /// Returns `true` if the controller has cleared the Active bit (transfer complete).
    #[inline]
    pub fn is_complete(&self) -> bool {
        self.token.read() & QTD_TOKEN_ACTIVE == 0
    }

    /// Returns `true` if any error bits are set in the token.
    #[inline]
    pub fn has_error(&self) -> bool {
        self.token.read() & QTD_TOKEN_ERROR_MASK != 0
    }

    /// Returns the remaining bytes to transfer (decremented by the controller).
    #[inline]
    pub fn bytes_remaining(&self) -> u32 {
        qtd_token_bytes_remaining(self.token.read())
    }
}

// ---------------------------------------------------------------------------
// Queue Head (QH) — EHCI §3.6
// ---------------------------------------------------------------------------

/// Queue Head (QH).
///
/// A 64-byte, 64-byte-aligned structure that describes an endpoint and contains
/// an overlay area (embedded qTD) for the currently-executing transfer. QHs are
/// linked into the async schedule (circular list for control/bulk) or the periodic
/// schedule (tree structure for interrupt endpoints).
///
/// # Layout (EHCI §3.6)
///
/// | Word | Field | Description |
/// |------|-------|-------------|
/// | 0 | `horizontal_link` | Next QH in the schedule (circular for async) |
/// | 1 | `characteristics` | Device address, endpoint, speed, max packet size |
/// | 2 | `capabilities` | Hub/port (split transactions), interrupt schedule mask |
/// | 3 | `current_qtd` | Pointer to current qTD (managed by controller) |
/// | 4–11 | `overlay` | Transfer overlay — embedded qTD (managed by controller) |
/// | 12–15 | (software) | Driver-private fields (not seen by hardware) |
#[repr(C, align(64))]
pub struct QueueHead {
    // -- Hardware-visible fields (48 bytes = words 0-11) --
    /// Horizontal link pointer — next QH in the schedule.
    /// Use [`link_pointer`] with [`link_type::QH`] to build.
    /// Set [`LINK_TERMINATE`] for the last entry.
    pub horizontal_link: VCell<u32>,

    /// Endpoint characteristics (word 1).
    /// Built with [`qh_characteristics`].
    pub characteristics: VCell<u32>,

    /// Endpoint capabilities (word 2).
    /// Built with [`qh_capabilities`].
    pub capabilities: VCell<u32>,

    /// Current qTD pointer (word 3) — managed by the controller.
    pub current_qtd: VCell<u32>,

    // Transfer overlay area (words 4–11) — same layout as a qTD.
    // Inlined as individual fields to avoid alignment padding that would
    // occur if we embedded a `TransferDescriptor` (which has `align(32)`).
    /// Overlay: next qTD pointer (word 4).
    pub overlay_next: VCell<u32>,
    /// Overlay: alternate next qTD pointer (word 5).
    pub overlay_alt_next: VCell<u32>,
    /// Overlay: qTD token (word 6).
    pub overlay_token: VCell<u32>,
    /// Overlay: buffer page pointers (words 7–11).
    pub overlay_buffer: [VCell<u32>; 5],

    // -- Software-private fields (16 bytes = words 12-15) --
    // These occupy the padding between the 48-byte hardware area and the
    // 64-byte alignment. The controller never reads or writes past word 11.
    /// Software: which qTD is currently attached (for completion tracking).
    /// Stored as a raw pointer so we can find the qTD in the ISR.
    pub attached_qtd: VCell<u32>,

    /// Software: original buffer address (for cache maintenance on completion).
    pub attached_buffer: VCell<u32>,

    /// Software: packed byte with status flags.
    /// - Bit 0: `used` — this QH slot is allocated
    /// - Bit 1: `removing` — awaiting async advance doorbell before freeing
    pub sw_flags: VCell<u8>,

    /// Software: cached PID direction for this endpoint.
    pub sw_pid: VCell<u8>,

    /// Software: polling interval in frames (interrupt endpoints).
    pub sw_interval_ms: VCell<u8>,

    /// Software: reserved padding to fill to 64 bytes.
    _pad: [u8; 5],
}

// QH software flag bits
/// The QH slot is currently allocated (in use).
pub const QH_FLAG_USED: u8 = 1 << 0;
/// The QH has been unlinked and is awaiting async advance doorbell.
pub const QH_FLAG_REMOVING: u8 = 1 << 1;

// ---------------------------------------------------------------------------
// QH Endpoint Characteristics (word 1) — EHCI §3.6.2
// ---------------------------------------------------------------------------

/// Bit offset for device address in characteristics word.
const QH_CHAR_ADDR_SHIFT: u32 = 0;
/// Bit offset for endpoint number in characteristics word.
const QH_CHAR_ENDPT_SHIFT: u32 = 8;
/// Bit offset for endpoint speed in characteristics word.
const QH_CHAR_SPEED_SHIFT: u32 = 12;
/// Data toggle control bit — 1 = use DT from qTD, 0 = use DT from QH overlay.
const QH_CHAR_DTC_SHIFT: u32 = 14;
/// Head of reclamation list flag.
const QH_CHAR_HEAD_SHIFT: u32 = 15;
/// Bit offset for maximum packet size in characteristics word.
const QH_CHAR_MAX_PKT_SHIFT: u32 = 16;
/// Control endpoint flag — must be set for FS/LS control endpoints.
const QH_CHAR_CTRL_EP_SHIFT: u32 = 27;
/// Bit offset for NAK count reload in characteristics word.
const QH_CHAR_NAK_RL_SHIFT: u32 = 28;

/// Build the QH endpoint characteristics word (word 1).
///
/// # Arguments
/// - `address` — USB device address (0–127)
/// - `endpoint` — endpoint number (0–15)
/// - `speed` — [`SPEED_FULL`], [`SPEED_LOW`], or [`SPEED_HIGH`]
/// - `max_packet_size` — maximum packet size (0–1024)
/// - `is_control` — `true` for control endpoints (enables DTC and control EP flag for FS/LS)
/// - `is_head` — `true` if this is the head of the async reclamation list
#[inline]
pub const fn qh_characteristics(
    address: u8,
    endpoint: u8,
    speed: u32,
    max_packet_size: u16,
    is_control: bool,
    is_head: bool,
) -> u32 {
    let dtc = if is_control { 1u32 } else { 0u32 };
    let ctrl_ep = if is_control && speed != SPEED_HIGH {
        1u32
    } else {
        0u32
    };
    let head = if is_head { 1u32 } else { 0u32 };

    (address as u32) << QH_CHAR_ADDR_SHIFT
        | (endpoint as u32) << QH_CHAR_ENDPT_SHIFT
        | speed << QH_CHAR_SPEED_SHIFT
        | dtc << QH_CHAR_DTC_SHIFT
        | head << QH_CHAR_HEAD_SHIFT
        | (max_packet_size as u32) << QH_CHAR_MAX_PKT_SHIFT
        | ctrl_ep << QH_CHAR_CTRL_EP_SHIFT
        | 15u32 << QH_CHAR_NAK_RL_SHIFT
}

// ---------------------------------------------------------------------------
// QH Endpoint Capabilities (word 2) — EHCI §3.6.2
// ---------------------------------------------------------------------------

/// Bit offset for interrupt schedule mask (S-mask) in capabilities word.
const QH_CAP_SMASK_SHIFT: u32 = 0;
/// Bit offset for split completion mask (C-mask) in capabilities word.
const QH_CAP_CMASK_SHIFT: u32 = 8;
/// Bit offset for hub address in capabilities word.
const QH_CAP_HUB_ADDR_SHIFT: u32 = 16;
/// Bit offset for hub port number in capabilities word.
const QH_CAP_HUB_PORT_SHIFT: u32 = 23;
/// Bit offset for high-bandwidth pipe multiplier in capabilities word.
const QH_CAP_MULT_SHIFT: u32 = 30;

/// Build the QH endpoint capabilities word (word 2).
///
/// # Arguments
/// - `smask` — interrupt schedule mask (micro-frame bitmask, 0 for async)
/// - `cmask` — split completion mask (0 for non-split or async)
/// - `hub_addr` — hub address for split transactions (0 if not behind a hub)
/// - `hub_port` — hub port for split transactions (0 if not behind a hub)
/// - `mult` — high-bandwidth multiplier (1 for most transfers)
#[inline]
pub const fn qh_capabilities(smask: u8, cmask: u8, hub_addr: u8, hub_port: u8, mult: u8) -> u32 {
    (smask as u32) << QH_CAP_SMASK_SHIFT
        | (cmask as u32) << QH_CAP_CMASK_SHIFT
        | (hub_addr as u32) << QH_CAP_HUB_ADDR_SHIFT
        | (hub_port as u32) << QH_CAP_HUB_PORT_SHIFT
        | (mult as u32) << QH_CAP_MULT_SHIFT
}

impl Default for QueueHead {
    fn default() -> Self {
        Self::new()
    }
}

impl QueueHead {
    /// Create a zeroed-out queue head (terminated, inactive).
    pub const fn new() -> Self {
        Self {
            horizontal_link: VCell::new(LINK_TERMINATE),
            characteristics: VCell::new(0),
            capabilities: VCell::new(0),
            current_qtd: VCell::new(0),
            overlay_next: VCell::new(LINK_TERMINATE),
            overlay_alt_next: VCell::new(LINK_TERMINATE),
            overlay_token: VCell::new(0),
            overlay_buffer: [
                VCell::new(0),
                VCell::new(0),
                VCell::new(0),
                VCell::new(0),
                VCell::new(0),
            ],
            attached_qtd: VCell::new(0),
            attached_buffer: VCell::new(0),
            sw_flags: VCell::new(0),
            sw_pid: VCell::new(0),
            sw_interval_ms: VCell::new(0),
            _pad: [0; 5],
        }
    }

    /// Initialise this QH as the async schedule sentinel (head of reclamation list).
    ///
    /// The sentinel QH points to itself (circular list) and has its overlay halted
    /// so the controller never tries to execute it. This is required by EHCI §4.8.
    ///
    /// # Safety
    /// The caller must ensure `self` is at a stable address (e.g. in a `static`)
    /// before calling this, since the horizontal link points back to `self`.
    pub unsafe fn init_sentinel(&mut self) {
        let self_addr = (self as *const Self) as u32;
        self.horizontal_link
            .write(link_pointer(self_addr, link_type::QH));
        // Mark as head of reclamation list, no real endpoint
        self.characteristics.write(qh_characteristics(
            0, // address 0 (unused)
            0, // endpoint 0 (unused)
            SPEED_HIGH, 0, // max packet 0 (unused)
            false, true, // head of reclamation list
        ));
        self.capabilities.write(qh_capabilities(0, 0, 0, 0, 1));
        self.current_qtd.write(0);
        // Halt the overlay so the controller skips this QH
        self.overlay_token.write(QTD_TOKEN_HALTED);
        self.overlay_next.write(LINK_TERMINATE);
        self.overlay_alt_next.write(LINK_TERMINATE);
    }

    /// Initialise this QH for an endpoint.
    ///
    /// # Arguments
    /// - `characteristics` — built with [`qh_characteristics`]
    /// - `capabilities` — built with [`qh_capabilities`]
    ///
    /// The overlay is set to halted (no active transfer). The caller should
    /// link the first qTD via [`attach_qtd`](Self::attach_qtd) after this.
    pub fn init_endpoint(&mut self, characteristics: u32, capabilities: u32) {
        self.horizontal_link.write(LINK_TERMINATE);
        self.characteristics.write(characteristics);
        self.capabilities.write(capabilities);
        self.current_qtd.write(0);
        self.overlay_next.write(LINK_TERMINATE);
        self.overlay_alt_next.write(LINK_TERMINATE);
        self.overlay_token.write(QTD_TOKEN_HALTED);
        for buf in &mut self.overlay_buffer {
            buf.write(0);
        }
        self.attached_qtd.write(0);
        self.attached_buffer.write(0);
        self.sw_flags.write(QH_FLAG_USED);
        self.sw_pid.write(0);
        self.sw_interval_ms.write(0);
    }

    /// Attach a qTD to this QH for execution.
    ///
    /// Writes the qTD address into the overlay's `next` pointer and clears
    /// the halted status so the controller will pick it up.
    ///
    /// # Safety
    /// - `qtd` must point to a valid, initialised [`TransferDescriptor`].
    /// - The qTD and its buffers must remain valid until the transfer completes.
    /// - The QH must be in non-cacheable memory accessible by the DMA engine.
    pub unsafe fn attach_qtd(&mut self, qtd: *const TransferDescriptor) {
        let qtd_addr = qtd as u32;
        self.attached_qtd.write(qtd_addr);
        // Write the qTD pointer into the overlay's next field
        self.overlay_next.write(qtd_addr);
        // Clear halt to let the controller fetch the qTD
        self.overlay_token.write(0);
    }

    /// Re-attach a qTD to this QH after it completes (interrupt pipe re-arm).
    ///
    /// Unlike [`QueueHead::attach_qtd`], this does **not** clear `overlay_token`, preserving
    /// the data toggle managed by the controller (`DTC = 0` mode). Used to
    /// re-arm an interrupt endpoint after each received packet.
    ///
    /// # Safety
    /// - `qtd` must point to a valid [`TransferDescriptor`] with Active=1.
    /// - The QH must be in non-cacheable memory accessible by the DMA engine.
    pub unsafe fn reattach_qtd_preserve_toggle(&mut self, qtd: *const TransferDescriptor) {
        self.overlay_next.write(qtd as u32);
        // overlay_token intentionally NOT written — controller manages DT bit (DTC=0)
    }

    /// Set the data toggle bit in the QH overlay without disturbing other overlay fields.
    ///
    /// Used to initialize the hardware-managed toggle (DTC=0) before a bulk transfer.
    /// Must be called after [`attach_qtd`](Self::attach_qtd) (which clears overlay_token
    /// to 0) and before the cache clean / schedule link.
    ///
    /// # Safety
    /// - `self` must be fully initialized and not currently in use by the controller.
    pub unsafe fn set_overlay_toggle(&mut self, toggle: bool) {
        let t = self.overlay_token.read();
        self.overlay_token.write(if toggle {
            t | (1 << 31)
        } else {
            t & !(1u32 << 31)
        });
    }

    /// Link this QH into the async schedule after `prev`.
    ///
    /// Inserts `self` between `prev` and whatever `prev` currently points to.
    ///
    /// # Safety
    /// - Both `self` and `prev` must be valid QHs in stable, non-cacheable memory.
    pub unsafe fn link_after(&mut self, prev: &mut QueueHead) {
        let self_addr = (self as *const Self) as u32;
        // self → prev's old successor
        self.horizontal_link.write(prev.horizontal_link.read());
        // prev → self
        prev.horizontal_link
            .write(link_pointer(self_addr, link_type::QH));
    }
}

// ---------------------------------------------------------------------------
// Periodic Frame List
// ---------------------------------------------------------------------------

/// Number of entries in the periodic frame list.
///
/// The i.MX RT controller supports 8, 16, 32, 64, 128, 256, 512, or 1024 entries.
/// We use 32 entries (matching USBHost_t36) for a good balance between memory and
/// scheduling granularity. Each entry corresponds to one USB frame (1 ms at FS/LS).
pub const FRAME_LIST_LEN: usize = 32;

/// The periodic frame list — an array of link pointers indexed by frame number.
///
/// Each entry is either [`LINK_TERMINATE`] (no transfer scheduled for that frame)
/// or a [`link_pointer`] to a QH chain for interrupt endpoints.
///
/// Must be 4096-byte aligned per EHCI spec. The i.MX RT controller reads the base
/// address from `DEVICEADDR::BASEADR` (host-mode alias for `PERIODICLISTBASE`).
#[repr(C, align(4096))]
pub struct FrameList {
    /// Frame list entries.
    pub entries: [VCell<u32>; FRAME_LIST_LEN],
}

impl Default for FrameList {
    fn default() -> Self {
        Self::new()
    }
}

impl FrameList {
    /// Create a frame list with all entries terminated (no scheduled transfers).
    #[allow(clippy::declare_interior_mutable_const)]
    pub const fn new() -> Self {
        const TERM: VCell<u32> = VCell::new(LINK_TERMINATE);
        Self {
            entries: [TERM; FRAME_LIST_LEN],
        }
    }
}

// ---------------------------------------------------------------------------
// Compile-time size and alignment assertions
// ---------------------------------------------------------------------------

const _: () = {
    assert!(core::mem::size_of::<TransferDescriptor>() == 32);
    assert!(core::mem::align_of::<TransferDescriptor>() == 32);
    assert!(core::mem::size_of::<QueueHead>() == 64);
    assert!(core::mem::align_of::<QueueHead>() == 64);
    // FrameList is padded to 4096 bytes due to align(4096), but the useful
    // content is only FRAME_LIST_LEN * 4 bytes. The size_of includes padding.
    assert!(core::mem::size_of::<FrameList>() == 4096);
    assert!(core::mem::align_of::<FrameList>() == 4096);
};

// ---------------------------------------------------------------------------
// Unit tests (host-only)
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use core::mem;

    // -----------------------------------------------------------------------
    // Link pointer helpers
    // -----------------------------------------------------------------------

    #[test]
    fn link_pointer_constructs_qh_link() {
        let addr: u32 = 0x2000_0040; // 64-byte aligned
        let lp = link_pointer(addr, link_type::QH);
        assert_eq!(link_address(lp), addr);
        assert!(!link_is_terminate(lp));
        // type bits [2:1] = 0b01 for QH
        assert_eq!(lp & 0b110, link_type::QH);
    }

    #[test]
    fn link_pointer_masks_low_bits() {
        // Address with low bits set — they should be masked off.
        let lp = link_pointer(0x2000_001F, link_type::ITD);
        assert_eq!(link_address(lp), 0x2000_0000);
        assert_eq!(lp & 0b110, link_type::ITD);
    }

    #[test]
    fn link_terminate_detected() {
        assert!(link_is_terminate(LINK_TERMINATE));
        assert!(link_is_terminate(0xFFFF_FFFF));
        assert!(!link_is_terminate(0x2000_0000 | link_type::QH));
    }

    #[test]
    fn link_pointer_all_types() {
        let addr: u32 = 0x2000_0020;
        assert_eq!(link_pointer(addr, link_type::ITD) & 0b110, link_type::ITD);
        assert_eq!(link_pointer(addr, link_type::QH) & 0b110, link_type::QH);
        assert_eq!(link_pointer(addr, link_type::SITD) & 0b110, link_type::SITD);
        assert_eq!(link_pointer(addr, link_type::FSTN) & 0b110, link_type::FSTN);
    }

    // -----------------------------------------------------------------------
    // qTD token builder
    // -----------------------------------------------------------------------

    #[test]
    fn qtd_token_out_64_bytes_data0_no_ioc() {
        let tok = qtd_token(PID_OUT, 64, false, false);
        // Active bit set
        assert_ne!(tok & QTD_TOKEN_ACTIVE, 0);
        // PID = OUT (0) at bits [9:8]
        assert_eq!((tok >> 8) & 0x3, PID_OUT);
        // CERR = 3 at bits [11:10]
        assert_eq!((tok >> 10) & 0x3, 3);
        // total_bytes = 64 at bits [30:16]
        assert_eq!(qtd_token_bytes_remaining(tok), 64);
        // DT = 0 at bit 31
        assert_eq!(tok >> 31, 0);
        // IOC not set
        assert_eq!(tok & QTD_TOKEN_IOC, 0);
    }

    #[test]
    fn qtd_token_in_512_bytes_data1_ioc() {
        let tok = qtd_token(PID_IN, 512, true, true);
        assert_ne!(tok & QTD_TOKEN_ACTIVE, 0);
        assert_eq!((tok >> 8) & 0x3, PID_IN);
        assert_eq!(qtd_token_bytes_remaining(tok), 512);
        assert_eq!(tok >> 31, 1); // DT = 1
        assert_ne!(tok & QTD_TOKEN_IOC, 0);
    }

    #[test]
    fn qtd_token_setup_8_bytes() {
        let tok = qtd_token(PID_SETUP, 8, false, false);
        assert_eq!((tok >> 8) & 0x3, PID_SETUP);
        assert_eq!(qtd_token_bytes_remaining(tok), 8);
    }

    #[test]
    fn qtd_token_zero_length() {
        let tok = qtd_token(PID_IN, 0, false, true);
        assert_eq!(qtd_token_bytes_remaining(tok), 0);
        assert_ne!(tok & QTD_TOKEN_IOC, 0);
    }

    #[test]
    fn qtd_token_max_transfer() {
        let tok = qtd_token(PID_IN, QTD_MAX_TRANSFER_SIZE, false, false);
        assert_eq!(qtd_token_bytes_remaining(tok), QTD_MAX_TRANSFER_SIZE);
    }

    #[test]
    fn qtd_token_bytes_remaining_extracts_correctly() {
        // Manually construct a token with known total_bytes
        let bytes: u32 = 1234;
        let tok = bytes << 16 | QTD_TOKEN_ACTIVE;
        assert_eq!(qtd_token_bytes_remaining(tok), 1234);
    }

    // -----------------------------------------------------------------------
    // QH Characteristics builder
    // -----------------------------------------------------------------------

    #[test]
    fn qh_char_control_endpoint_full_speed() {
        let ch = qh_characteristics(1, 0, SPEED_FULL, 64, true, false);
        // Device address = 1 at bits [6:0]
        assert_eq!(ch & 0x7F, 1);
        // Endpoint = 0 at bits [11:8]
        assert_eq!((ch >> 8) & 0xF, 0);
        // Speed = FULL at bits [13:12]
        assert_eq!((ch >> 12) & 0x3, SPEED_FULL);
        // DTC = 1 at bit 14 (control endpoint)
        assert_eq!((ch >> 14) & 1, 1);
        // Head = 0 at bit 15
        assert_eq!((ch >> 15) & 1, 0);
        // Max packet = 64 at bits [26:16]
        assert_eq!((ch >> 16) & 0x7FF, 64);
        // Control EP flag = 1 at bit 27 (FS control)
        assert_eq!((ch >> 27) & 1, 1);
        // NAK RL = 15 at bits [31:28]
        assert_eq!((ch >> 28) & 0xF, 15);
    }

    #[test]
    fn qh_char_bulk_high_speed() {
        let ch = qh_characteristics(5, 2, SPEED_HIGH, 512, false, false);
        assert_eq!(ch & 0x7F, 5); // address
        assert_eq!((ch >> 8) & 0xF, 2); // endpoint
        assert_eq!((ch >> 12) & 0x3, SPEED_HIGH); // speed
        assert_eq!((ch >> 14) & 1, 0); // DTC=0 (not control)
        assert_eq!((ch >> 16) & 0x7FF, 512); // max packet
        assert_eq!((ch >> 27) & 1, 0); // control EP flag = 0 (HS)
    }

    #[test]
    fn qh_char_control_high_speed_no_ctrl_flag() {
        // HS control endpoint: DTC=1 but control EP flag should be 0
        let ch = qh_characteristics(1, 0, SPEED_HIGH, 64, true, false);
        assert_eq!((ch >> 14) & 1, 1); // DTC=1 (control)
        assert_eq!((ch >> 27) & 1, 0); // ctrl_ep flag = 0 (HS doesn't set it)
    }

    #[test]
    fn qh_char_head_of_reclamation() {
        let ch = qh_characteristics(0, 0, SPEED_HIGH, 0, false, true);
        assert_eq!((ch >> 15) & 1, 1); // head flag
    }

    #[test]
    fn qh_char_low_speed_control() {
        let ch = qh_characteristics(3, 0, SPEED_LOW, 8, true, false);
        assert_eq!((ch >> 12) & 0x3, SPEED_LOW);
        assert_eq!((ch >> 27) & 1, 1); // LS control → ctrl_ep flag set
        assert_eq!((ch >> 16) & 0x7FF, 8);
    }

    // -----------------------------------------------------------------------
    // QH Capabilities builder
    // -----------------------------------------------------------------------

    #[test]
    fn qh_cap_async_no_split() {
        let cap = qh_capabilities(0, 0, 0, 0, 1);
        assert_eq!(cap & 0xFF, 0); // smask = 0
        assert_eq!((cap >> 8) & 0xFF, 0); // cmask = 0
        assert_eq!((cap >> 16) & 0x7F, 0); // hub addr = 0
        assert_eq!((cap >> 23) & 0x7F, 0); // hub port = 0
        assert_eq!((cap >> 30) & 0x3, 1); // mult = 1
    }

    #[test]
    fn qh_cap_interrupt_with_split() {
        let cap = qh_capabilities(0x04, 0x1C, 7, 1, 1);
        assert_eq!(cap & 0xFF, 0x04); // smask
        assert_eq!((cap >> 8) & 0xFF, 0x1C); // cmask
        assert_eq!((cap >> 16) & 0x7F, 7); // hub addr
        assert_eq!((cap >> 23) & 0x7F, 1); // hub port
        assert_eq!((cap >> 30) & 0x3, 1); // mult
    }

    #[test]
    fn qh_cap_high_bandwidth_mult() {
        let cap = qh_capabilities(0, 0, 0, 0, 3);
        assert_eq!((cap >> 30) & 0x3, 3);
    }

    // -----------------------------------------------------------------------
    // TransferDescriptor
    // -----------------------------------------------------------------------

    #[test]
    fn td_new_is_terminated() {
        let td = TransferDescriptor::new();
        assert_eq!(td.next.read(), LINK_TERMINATE);
        assert_eq!(td.alt_next.read(), LINK_TERMINATE);
        assert_eq!(td.token.read(), 0);
        for buf in &td.buffer {
            assert_eq!(buf.read(), 0);
        }
    }

    #[test]
    fn td_default_matches_new() {
        let a = TransferDescriptor::new();
        let b = TransferDescriptor::default();
        assert_eq!(a.next.read(), b.next.read());
        assert_eq!(a.alt_next.read(), b.alt_next.read());
        assert_eq!(a.token.read(), b.token.read());
    }

    #[test]
    fn td_size_and_alignment() {
        assert_eq!(mem::size_of::<TransferDescriptor>(), 32);
        assert_eq!(mem::align_of::<TransferDescriptor>(), 32);
    }

    #[test]
    fn td_init_sets_fields() {
        let mut td = TransferDescriptor::new();
        let buffer = [0u8; 64];
        let tok = qtd_token(PID_IN, 64, true, true);

        // SAFETY: buffer is valid for 64 bytes, td is local.
        unsafe { td.init(tok, buffer.as_ptr(), 64) };

        assert_eq!(td.next.read(), LINK_TERMINATE);
        assert_eq!(td.alt_next.read(), LINK_TERMINATE);
        assert_eq!(td.token.read(), tok);
        assert_eq!(td.buffer[0].read(), buffer.as_ptr() as u32);
    }

    #[test]
    fn td_is_complete_when_active_cleared() {
        let td = TransferDescriptor::new();
        td.token.write(QTD_TOKEN_ACTIVE);
        assert!(!td.is_complete());

        td.token.write(0); // active cleared
        assert!(td.is_complete());
    }

    #[test]
    fn td_has_error_flags() {
        let td = TransferDescriptor::new();
        td.token.write(0);
        assert!(!td.has_error());

        td.token.write(QTD_TOKEN_HALTED);
        assert!(td.has_error());

        td.token.write(QTD_TOKEN_BABBLE);
        assert!(td.has_error());

        td.token.write(QTD_TOKEN_XACT_ERR);
        assert!(td.has_error());
    }

    #[test]
    fn td_bytes_remaining() {
        let td = TransferDescriptor::new();
        let tok = qtd_token(PID_IN, 256, false, false);
        td.token.write(tok);
        assert_eq!(td.bytes_remaining(), 256);

        // Simulate controller decrementing bytes
        td.token.write(tok & !(0x7FFF << 16) | (128 << 16));
        assert_eq!(td.bytes_remaining(), 128);
    }

    // -----------------------------------------------------------------------
    // QueueHead
    // -----------------------------------------------------------------------

    #[test]
    fn qh_new_is_terminated() {
        let qh = QueueHead::new();
        assert_eq!(qh.horizontal_link.read(), LINK_TERMINATE);
        assert_eq!(qh.characteristics.read(), 0);
        assert_eq!(qh.capabilities.read(), 0);
        assert_eq!(qh.current_qtd.read(), 0);
        assert_eq!(qh.overlay_next.read(), LINK_TERMINATE);
        assert_eq!(qh.overlay_alt_next.read(), LINK_TERMINATE);
        assert_eq!(qh.overlay_token.read(), 0);
        assert_eq!(qh.sw_flags.read(), 0);
    }

    #[test]
    fn qh_default_matches_new() {
        let a = QueueHead::new();
        let b = QueueHead::default();
        assert_eq!(a.horizontal_link.read(), b.horizontal_link.read());
        assert_eq!(a.characteristics.read(), b.characteristics.read());
        assert_eq!(a.overlay_token.read(), b.overlay_token.read());
    }

    #[test]
    fn qh_size_and_alignment() {
        assert_eq!(mem::size_of::<QueueHead>(), 64);
        assert_eq!(mem::align_of::<QueueHead>(), 64);
    }

    #[test]
    fn qh_init_endpoint_sets_used_flag() {
        let mut qh = QueueHead::new();
        let ch = qh_characteristics(1, 0, SPEED_FULL, 64, true, false);
        let cap = qh_capabilities(0, 0, 0, 0, 1);
        qh.init_endpoint(ch, cap);

        assert_eq!(qh.characteristics.read(), ch);
        assert_eq!(qh.capabilities.read(), cap);
        assert_eq!(qh.overlay_token.read(), QTD_TOKEN_HALTED);
        assert_ne!(qh.sw_flags.read() & QH_FLAG_USED, 0);
    }

    #[test]
    fn qh_attach_qtd_clears_halt() {
        let mut qh = QueueHead::new();
        let ch = qh_characteristics(1, 1, SPEED_HIGH, 512, false, false);
        qh.init_endpoint(ch, qh_capabilities(0, 0, 0, 0, 1));
        assert_ne!(qh.overlay_token.read() & QTD_TOKEN_HALTED, 0);

        let td = TransferDescriptor::new();
        // SAFETY: td is local, we're just testing pointer propagation.
        unsafe { qh.attach_qtd(&td) };
        assert_eq!(qh.overlay_next.read(), &td as *const _ as u32);
        assert_eq!(qh.overlay_token.read(), 0); // halt cleared
    }

    #[test]
    fn qh_set_overlay_toggle() {
        let mut qh = QueueHead::new();
        qh.overlay_token.write(0);

        unsafe { qh.set_overlay_toggle(true) };
        assert_eq!(qh.overlay_token.read() >> 31, 1);

        unsafe { qh.set_overlay_toggle(false) };
        assert_eq!(qh.overlay_token.read() >> 31, 0);
    }

    // -----------------------------------------------------------------------
    // FrameList
    // -----------------------------------------------------------------------

    #[test]
    fn frame_list_new_all_terminated() {
        let fl = FrameList::new();
        for entry in &fl.entries {
            assert!(link_is_terminate(entry.read()));
        }
    }

    #[test]
    fn frame_list_default_matches_new() {
        let fl = FrameList::default();
        for entry in &fl.entries {
            assert!(link_is_terminate(entry.read()));
        }
    }

    #[test]
    fn frame_list_length() {
        assert_eq!(FRAME_LIST_LEN, 32);
    }

    #[test]
    fn frame_list_size_and_alignment() {
        assert_eq!(mem::size_of::<FrameList>(), 4096);
        assert_eq!(mem::align_of::<FrameList>(), 4096);
    }

    // -----------------------------------------------------------------------
    // Constants sanity checks
    // -----------------------------------------------------------------------

    #[test]
    fn error_mask_covers_all_error_bits() {
        assert_ne!(QTD_TOKEN_ERROR_MASK & QTD_TOKEN_HALTED, 0);
        assert_ne!(QTD_TOKEN_ERROR_MASK & QTD_TOKEN_BUFFER_ERR, 0);
        assert_ne!(QTD_TOKEN_ERROR_MASK & QTD_TOKEN_BABBLE, 0);
        assert_ne!(QTD_TOKEN_ERROR_MASK & QTD_TOKEN_XACT_ERR, 0);
        assert_ne!(QTD_TOKEN_ERROR_MASK & QTD_TOKEN_MISSED_UFRAME, 0);
        // Active bit should NOT be in error mask
        assert_eq!(QTD_TOKEN_ERROR_MASK & QTD_TOKEN_ACTIVE, 0);
    }

    #[test]
    fn status_mask_covers_full_byte() {
        assert_eq!(QTD_TOKEN_STATUS_MASK, 0xFF);
    }

    #[test]
    fn pid_codes_are_distinct() {
        assert_ne!(PID_OUT, PID_IN);
        assert_ne!(PID_IN, PID_SETUP);
        assert_ne!(PID_OUT, PID_SETUP);
    }

    #[test]
    fn speed_codes_are_distinct() {
        assert_ne!(SPEED_FULL, SPEED_LOW);
        assert_ne!(SPEED_LOW, SPEED_HIGH);
        assert_ne!(SPEED_FULL, SPEED_HIGH);
    }
}
