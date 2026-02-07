# USB Control Transfers and EHCI Data Structures

## Overview

This document provides an in-depth explanation of USB control transfers in the context of EHCI (Enhanced Host Controller Interface) hardware, specifically for implementing USB host functionality on i.MX RT microcontrollers. Understanding these concepts is essential for implementing the `HostController` trait from `cotton-usb-host`.

## Table of Contents

1. [USB Control Transfer Basics](#usb-control-transfer-basics)
2. [EHCI Architecture Overview](#ehci-architecture-overview)
3. [Queue Head (QH) Structure](#queue-head-qh-structure)
4. [Queue Transfer Descriptor (qTD) Structure](#queue-transfer-descriptor-qtd-structure)
5. [Control Transfer Flow](#control-transfer-flow)
6. [Complete Control Transfer Example](#complete-control-transfer-example)
7. [Error Handling](#error-handling)
8. [Implementation Patterns](#implementation-patterns)
9. [Advanced Topics](#advanced-topics)
10. [References](#references)

---

## USB Control Transfer Basics

### What is a Control Transfer?

A **control transfer** is a special type of USB transaction used for:
- Device enumeration and configuration
- Getting/setting device descriptors
- Vendor-specific commands
- Standard USB requests (Get Status, Set Configuration, etc.)

All USB devices must support control transfers on **endpoint 0**.

### Control Transfer Phases

Every control transfer consists of 2 or 3 phases:

```
┌──────────┐     ┌───────────┐     ┌────────────┐
│  SETUP   │ --> │   DATA    │ --> │   STATUS   │
│  Stage   │     │  Stage    │     │   Stage    │
│          │     │ (optional)│     │            │
└──────────┘     └───────────┘     └────────────┘
    8 bytes       0-N bytes          0 bytes
   OUT always    IN or OUT         opposite of DATA
```

#### 1. SETUP Stage
- **Direction**: Always HOST → DEVICE (OUT)
- **Size**: Always exactly 8 bytes
- **Content**: Setup packet (request type, request, value, index, length)
- **PID**: SETUP token

#### 2. DATA Stage (Optional)
- **Direction**: IN (device → host) or OUT (host → device), depending on request
- **Size**: 0 to N bytes (specified in SETUP packet's wLength field)
- **Content**: Request-specific data
- **PID**: DATA1 initially, then toggles

#### 3. STATUS Stage
- **Direction**: Opposite of DATA stage (or IN if no DATA stage)
- **Size**: Always 0 bytes (ZLP - Zero Length Packet)
- **Content**: None (just acknowledgment)
- **PID**: DATA1 always

### Setup Packet Format

The 8-byte SETUP packet (USB 2.0 section 9.3):

```rust
#[repr(C, packed)]
struct SetupPacket {
    bmRequestType: u8,    // Bit 7: Direction (0=OUT, 1=IN)
                          // Bits 6-5: Type (0=Std, 1=Class, 2=Vendor)
                          // Bits 4-0: Recipient (0=Device, 1=Interface, 2=Endpoint)
    bRequest: u8,         // Request code (e.g., 6=GET_DESCRIPTOR)
    wValue: u16,          // Request-specific value (e.g., descriptor type)
    wIndex: u16,          // Request-specific index (e.g., interface number)
    wLength: u16,         // Length of data stage (0 if none)
}
```

Example - Get Device Descriptor:
```
bmRequestType: 0x80  (Device-to-Host, Standard, Device)
bRequest:      0x06  (GET_DESCRIPTOR)
wValue:        0x0100 (Descriptor Type = Device, Index = 0)
wIndex:        0x0000
wLength:       0x0012 (18 bytes)
```

---

## EHCI Architecture Overview

### The Big Picture

EHCI (Enhanced Host Controller Interface) is the hardware specification for USB 2.0 host controllers. It uses a sophisticated DMA-based architecture:

```
┌────────────────────────────────────────────────────────┐
│                     USB Host Controller                │
│  ┌──────────────────────────────────────────────────┐  │
│  │         DMA Engine                               │  │
│  │  Reads QH/qTD structures from memory             │  │
│  │  Executes USB transactions                       │  │
│  │  Writes status back to memory                    │  │
│  └──────────────────────────────────────────────────┘  │
└────────────────────────────────────────────────────────┘
                          ▲ │
                          │ │ DMA (bypasses CPU)
                          │ ▼
┌────────────────────────────────────────────────────────┐
│                     System Memory                      │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │  Queue Head  │──│  Queue Head  │──│  Queue Head  │  │
│  │      #1      │  │      #2      │  │      #3      │  │
│  └──────┬───────┘  └──────────────┘  └──────────────┘  │
│         │                                              │
│         ├──> ┌──────────────┐                          │
│         │    │     qTD      │                          │
│         │    │  (Setup)     │                          │
│         │    └──────┬───────┘                          │
│         │           │                                  │
│         │    ┌──────▼───────┐                          │
│         │    │     qTD      │                          │
│         │    │   (Data)     │                          │
│         │    └──────┬───────┘                          │
│         │           │                                  │
│         └───>┌──────▼───────┐                          │
│              │     qTD      │                          │
│              │  (Status)    │                          │
│              └──────────────┘                          │
└────────────────────────────────────────────────────────┘
```

### Key Concepts

1. **Queue Heads (QH)**: Represent USB endpoints. Persistent structures that describe endpoint characteristics.

2. **Queue Transfer Descriptors (qTD)**: Represent individual transactions. Temporary structures linked in chains.

3. **Asynchronous Schedule**: A circular linked list of QHs for control and bulk transfers. USB controller continuously processes this list.

4. **Periodic Schedule**: A frame list + tree of QHs for interrupt transfers. (Separate from async schedule)

5. **DMA Operation**: USB controller reads QH/qTD structures from memory, executes USB transactions, updates status fields in memory.

---

## Queue Head (QH) Structure

### QH Memory Layout

The Queue Head is a 48-byte structure (EHCI spec section 3.6):

```rust
#[repr(C, align(64))]
pub struct QueueHead {
    // ===== DWORD 0: Horizontal Link Pointer =====
    horizontal_link: u32,
    
    // ===== DWORD 1: Endpoint Characteristics =====
    endpoint_characteristics: u32,
    
    // ===== DWORD 2: Endpoint Capabilities =====
    endpoint_capabilities: u32,
    
    // ===== DWORD 3: Current qTD Pointer =====
    current_qtd_pointer: u32,
    
    // ===== DWORDS 4-11: Transfer Overlay Area =====
    // This area has the same format as a qTD.
    // The USB controller copies the active qTD here and updates it
    // as the transfer progresses.
    overlay_next_qtd: u32,              // DWORD 4
    overlay_alt_next_qtd: u32,          // DWORD 5
    overlay_token: u32,                 // DWORD 6
    overlay_buffer_pointer_0: u32,      // DWORD 7
    overlay_buffer_pointer_1: u32,      // DWORD 8
    overlay_buffer_pointer_2: u32,      // DWORD 9
    overlay_buffer_pointer_3: u32,      // DWORD 10
    overlay_buffer_pointer_4: u32,      // DWORD 11
    
    // Pad from 48 bytes to 64 bytes (2 full cache lines)
    // to prevent false sharing with adjacent structures.
    _padding: [u8; 16],
}
```

> **Alignment**: EHCI requires QH to be 64-byte aligned (the hardware ignores
> the low 6 bits of any QH pointer). The `align(64)` also ensures each QH
> occupies exactly 2 cache lines (32 bytes each), simplifying cache management.

### Detailed Field Descriptions

#### Horizontal Link Pointer (DWORD 0)

Links QHs together in the asynchronous schedule:

```
Bits 31-5: Physical address of next QH (32-byte aligned)
Bits 4-3:  Reserved
Bits 2-1:  Type (00 = iTD, 01 = QH, 10 = siTD, 11 = FSTN)
Bit 0:     Terminate (T) (1 = end of list / no more entries, 0 = pointer is valid)
```

For a circular list (required for async schedule):
```rust
// Last QH points back to first QH (circular list for async schedule)
last_qh.horizontal_link = (first_qh_addr & !0x1F) | (0b01 << 1); // Type = QH, T = 0
```

#### Endpoint Characteristics (DWORD 1)

Describes the target USB endpoint:

```
Bits 31-28: NAK Count Reload (RL) - retry count
Bit 27:     Control Endpoint Flag (C) - 1 for control endpoints
Bits 26-16: Maximum Packet Length (MPL) - in bytes
Bit 15:     Head of Reclamation List Flag (H) - 1 for first QH in async list
Bit 14:     Data Toggle Control (DTC) - 1 to get toggle from qTD
Bits 13-12: Endpoint Speed (EPS) - 00=Full, 01=Low, 10=High
Bits 11-8:  Endpoint Number (ENDPT)
Bit 7:      Inactivate on Next Transaction (I)
Bits 6-0:   Device Address (DEVADDR)
```

Example for control endpoint 0 on device address 5, full speed:
```rust
qh.endpoint_characteristics = 
    (15 << 28) |          // NAK reload = 15 (retries before yielding to next QH)
    (1 << 27) |           // Control endpoint flag (C=1 required for FS/LS control EPs)
    (8 << 16) |           // Max packet = 8 bytes (minimum for EP0; updated after first descriptor read)
    (0 << 15) |           // Not head of reclamation list
    (1 << 14) |           // DTC = 1 (get data toggle from qTD, not QH overlay)
    (0b00 << 12) |        // Full speed (00=FS, 01=LS, 10=HS)
    (0 << 8) |            // Endpoint 0
    (0 << 7) |            // Don't inactivate on next transaction
    (5 << 0);             // Device address 5
```

> **NAK Reload (RL)**: Controls how many NAKs the controller accepts before moving
> to the next QH in the async schedule. Setting RL=0 means the controller will
> keep retrying indefinitely (only appropriate when using the async advance doorbell).
> A value of 15 is a reasonable default for control endpoints.
>
> **Control Endpoint Flag (C)**: Must be set to 1 for control endpoints at full-speed
> or low-speed. This tells the controller to check the data toggle from the qTD (when
> DTC=1) rather than relying on the QH overlay's toggle. This is how SETUP packets
> can always use DATA0 regardless of previous toggle state.

#### Endpoint Capabilities (DWORD 2)

Additional endpoint configuration:

```
Bits 31-30: High-Bandwidth Pipe Multiplier (MULT) - for high-speed only
Bit 29-23:  Port Number - for split transactions (FS/LS device behind HS hub)
Bits 22-16: Hub Address - for split transactions
Bits 15-8:  Split Completion Mask (uFrame C-mask)
Bits 7-0:   Interrupt Schedule Mask (uFrame S-mask)
```

For full-speed control endpoint (no split transactions):
```rust
qh.endpoint_capabilities = 
    (1 << 30) |           // Multiplier = 1 (one transaction per micro-frame)
    (0 << 23) |           // Port = 0 (unused without split transactions)
    (0 << 16) |           // Hub addr = 0 (unused without split transactions)
    (0x00 << 8) |         // C-mask = 0 (not a periodic endpoint)
    (0x00 << 0);          // S-mask = 0 (not a periodic endpoint)
```

> **Split transactions**: When a full-speed or low-speed device is connected behind
> a high-speed hub, the host must use split transactions. In that case, set Hub Address
> and Port Number to identify the hub/port, and set S-mask/C-mask for the micro-frame
> scheduling. See [Advanced Topics](#split-transactions) for details.

#### Current qTD Pointer (DWORD 3)

Points to the qTD currently being processed by hardware:
```rust
// Initially NULL (0x00000000)
// Hardware updates this as it processes qTDs
qh.current_qtd_pointer = 0x00000000;
```

#### Transfer Overlay Area (DWORDs 4-11)

The "overlay area" is where the USB controller copies the active qTD for processing. It has the same format as a qTD (see next section). This is how you monitor transfer progress and status.

**Key insight**: The overlay area is **read-write** for the USB controller. It starts with the first qTD's contents and gets updated as the transfer progresses.

---

## Queue Transfer Descriptor (qTD) Structure

### qTD Memory Layout

The qTD is a 32-byte structure (EHCI spec section 3.5):

```rust
#[repr(C, align(32))]
pub struct QueueTransferDescriptor {
    // ===== DWORD 0: Next qTD Pointer =====
    next_qtd_pointer: u32,
    
    // ===== DWORD 1: Alternate Next qTD Pointer =====
    alternate_next_qtd_pointer: u32,
    
    // ===== DWORD 2: Token =====
    token: u32,
    
    // ===== DWORDS 3-7: Buffer Pointers =====
    buffer_pointer_0: u32,
    buffer_pointer_1: u32,
    buffer_pointer_2: u32,
    buffer_pointer_3: u32,
    buffer_pointer_4: u32,
}
```

### Detailed Field Descriptions

#### Next qTD Pointer (DWORD 0)

Links qTDs in a chain:

```
Bits 31-5: Physical address of next qTD (32-byte aligned)
Bits 4-1:  Reserved
Bit 0:     Terminate (1 = last qTD, 0 = more qTDs)
```

Example:
```rust
// Link to next qTD
qtd_setup.next_qtd_pointer = qtd_data_addr & !0x1F;

// Last qTD in chain
qtd_status.next_qtd_pointer = 0x00000001;  // Terminate bit set
```

#### Alternate Next qTD Pointer (DWORD 1)

Used for error recovery (short packet detection):

```
Bits 31-5: Physical address of alternate qTD
Bits 4-1:  Reserved
Bit 0:     Terminate (1 = no alternate, 0 = alternate valid)
```

For most control transfers, set to terminate:
```rust
qtd.alternate_next_qtd_pointer = 0x00000001;
```

#### Token (DWORD 2)

The most important field - contains transfer parameters and status:

```
Bit 31:     Data Toggle (DT) - 0 or 1
Bits 30-16: Total Bytes to Transfer - remaining bytes (hardware decrements)
Bit 15:     Interrupt on Complete (IOC) - 1 to generate interrupt
Bits 14-12: Current Page (C_Page) - current buffer pointer index (0-4)
Bits 11-10: Error Counter (CERR) - errors before halting (start with 3)
Bits 9-8:   PID Code - 00=OUT, 01=IN, 10=SETUP
Bit 7:      Active (Status) - 1=active, 0=complete
Bit 6:      Halted - 1 if error occurred
Bit 5:      Data Buffer Error - 1 if buffer error
Bit 4:      Babble Detected - 1 if device sent too much data
Bit 3:      Transaction Error (XactErr) - 1 if timeout/CRC/etc.
Bit 2:      Missed Micro-Frame - 1 if periodic transfer missed
Bit 1:      Split Transaction State - for split transactions
Bit 0:      Ping State / ERR - various error uses
```

**PID Code Values**:
- `0b00` (0): OUT token
- `0b01` (1): IN token
- `0b10` (2): SETUP token

Example - SETUP qTD token:
```rust
qtd_setup.token = 
    (0 << 31) |           // Data toggle = 0 (SETUP always uses DATA0)
    (8 << 16) |           // 8 bytes to transfer
    (1 << 15) |           // Interrupt on complete
    (0 << 12) |           // Current page = 0
    (3 << 10) |           // Error counter = 3
    (0b10 << 8) |         // PID = SETUP
    (1 << 7) |            // Active
    (0 << 0);             // Clear status bits
```

**Reading Status** (after transfer completes):
```rust
// Check if transfer is done
if (qtd.token & (1 << 7)) == 0 {
    // Active bit clear = transfer complete
    
    // Check for errors
    if (qtd.token & (1 << 6)) != 0 {
        // Halted bit set = error occurred
        if (qtd.token & (1 << 5)) != 0 {
            // Data buffer error
        }
        if (qtd.token & (1 << 4)) != 0 {
            // Babble detected
        }
        if (qtd.token & (1 << 3)) != 0 {
            // Transaction error
        }
    } else {
        // Success!
        let bytes_remaining = (qtd.token >> 16) & 0x7FFF;
        let bytes_transferred = total_bytes - bytes_remaining;
    }
}
```

#### Buffer Pointers (DWORDs 3-7)

Up to 5 buffer pointers, each pointing to a 4KB page:

```
Buffer Pointer 0 (bits 31-12): Physical address of first page
                 (bits 11-0):  Current offset in page

Buffer Pointers 1-4 (bits 31-12): Physical addresses of additional pages
                     (bits 11-0):  Reserved (must be 0)
```

A single qTD can transfer up to **20KB** (5 pages × 4KB each).

Example for 18-byte transfer:
```rust
qtd.buffer_pointer_0 = buffer_addr; // Can be unaligned
qtd.buffer_pointer_1 = 0;           // Only need one page
qtd.buffer_pointer_2 = 0;
qtd.buffer_pointer_3 = 0;
qtd.buffer_pointer_4 = 0;
```

Example for 8KB transfer (spans pages):
```rust
let base_addr = buffer_addr & !0xFFF; // Align to 4KB
qtd.buffer_pointer_0 = buffer_addr;                  // First page + offset
qtd.buffer_pointer_1 = base_addr + 0x1000;          // Second page
qtd.buffer_pointer_2 = 0;
qtd.buffer_pointer_3 = 0;
qtd.buffer_pointer_4 = 0;
```

---

## Control Transfer Flow

### Step-by-Step Process

#### Phase 1: Setup Data Structures

```rust
// 1. Allocate and initialize QH for endpoint 0
let qh = QueueHead::new();
qh.endpoint_characteristics = /* endpoint 0, device address, speed */;
qh.endpoint_capabilities = /* default for control */;
qh.current_qtd_pointer = 0;

// 2. Create qTDs for the three stages
let qtd_setup = QueueTransferDescriptor::new();
let qtd_data = QueueTransferDescriptor::new();
let qtd_status = QueueTransferDescriptor::new();

// 3. Initialize SETUP qTD
qtd_setup.next_qtd_pointer = &qtd_data as *const _ as u32;
qtd_setup.alternate_next_qtd_pointer = 0x00000001; // Terminate
qtd_setup.token = 
    (0 << 31) |        // DATA0
    (8 << 16) |        // 8 bytes
    (0 << 15) |        // No interrupt on complete (continue to next)
    (0 << 12) |        // Current page 0
    (3 << 10) |        // Error counter 3
    (0b10 << 8) |      // PID = SETUP
    (1 << 7);          // Active
qtd_setup.buffer_pointer_0 = setup_packet_addr;

// 4. Initialize DATA qTD (example: IN direction)
qtd_data.next_qtd_pointer = &qtd_status as *const _ as u32;
qtd_data.alternate_next_qtd_pointer = 0x00000001;
qtd_data.token = 
    (1 << 31) |        // DATA1 (data stage always starts with DATA1)
    (data_len << 16) | // Requested bytes
    (0 << 15) |        // No interrupt yet
    (0 << 12) |
    (3 << 10) |
    (0b01 << 8) |      // PID = IN
    (1 << 7);          // Active
qtd_data.buffer_pointer_0 = data_buffer_addr;

// 5. Initialize STATUS qTD (opposite direction = OUT for IN transfer)
qtd_status.next_qtd_pointer = 0x00000001; // Terminate
qtd_status.alternate_next_qtd_pointer = 0x00000001;
qtd_status.token = 
    (1 << 31) |        // DATA1 (status always DATA1)
    (0 << 16) |        // 0 bytes (ZLP)
    (1 << 15) |        // Interrupt on complete!
    (0 << 12) |
    (3 << 10) |
    (0b00 << 8) |      // PID = OUT (opposite of data)
    (1 << 7);          // Active
qtd_status.buffer_pointer_0 = 0; // No buffer needed for ZLP
```

#### Phase 2: Link QH to Schedule

```rust
// 6. Point QH overlay to first qTD in the chain.
// The overlay's Next qTD Pointer tells the controller which qTD to fetch first.
// The overlay's token must have Active=0 so the controller fetches the qTD
// rather than executing the overlay.
qh.overlay_next_qtd = (&qtd_setup as *const _ as u32) & !0x1F;
qh.overlay_alt_next_qtd = 0x00000001; // Terminate
qh.overlay_token = 0; // Active=0 → controller will advance to next_qtd

// 7. Ensure cache coherency
cache_clean(&qtd_setup, size_of::<QTD>());
cache_clean(&qtd_data, size_of::<QTD>());
cache_clean(&qtd_status, size_of::<QTD>());
cache_clean(&qh, size_of::<QH>());
cache_clean(setup_packet, 8);

// 8. Link QH into asynchronous schedule
// Option A: Insert at head
qh.horizontal_link = async_list_head.horizontal_link;
cache_clean(&qh.horizontal_link, 4);
async_list_head.horizontal_link = (&qh as *const _ as u32) | 0b010;
cache_clean(&async_list_head.horizontal_link, 4);

// 9. Ensure async schedule is enabled
usb_regs.USBCMD.modify(|r| r | (1 << 5)); // Async Schedule Enable
```

#### Phase 3: Wait for Completion

```rust
// 10. Wait for interrupt or poll for completion
loop {
    // Invalidate the entire QH to see hardware updates to the overlay.
    // The controller may update multiple overlay fields (token, buffer pointers, etc.).
    cache_invalidate(&qh as *const _ as *const u8, size_of::<QH>());
    
    let token = qh.overlay_token;
    
    // Check if Active bit is clear
    if (token & (1 << 7)) == 0 {
        // Transfer complete!
        break;
    }
    
    // Optional: timeout check
    if timeout_expired() {
        return Err(UsbError::Timeout);
    }
    
    // Yield to async runtime
    yield_now().await;
}
```

#### Phase 4: Check Status and Clean Up

```rust
// 11. Invalidate cache for final status
cache_invalidate(&qtd_setup, size_of::<QTD>());
cache_invalidate(&qtd_data, size_of::<QTD>());
cache_invalidate(&qtd_status, size_of::<QTD>());

// 12. Check for errors
if (qtd_status.token & (1 << 6)) != 0 {
    // Halted bit set
    if (qtd_status.token & (1 << 3)) != 0 {
        return Err(UsbError::TransactionError);
    }
    // Check other error bits...
}

// 13. Invalidate data buffer cache before reading
cache_invalidate(data_buffer, data_len);

// 14. Extract transferred bytes
let bytes_remaining = (qtd_data.token >> 16) & 0x7FFF;
let bytes_transferred = data_len - bytes_remaining;

// 15. Remove QH from async schedule.
// WARNING: Cannot simply unlink and reuse — the controller may still be reading
// the QH. Must use the Async Advance Doorbell (USBCMD[IAA]) mechanism:
//   a) Unlink QH from the circular list
//   b) Set USBCMD[IAA] (Interrupt on Async Advance)
//   c) Wait for USBSTS[AAI] interrupt (confirms controller has advanced past the QH)
//   d) NOW it's safe to free/reuse the QH and its qTDs
async_list_head.horizontal_link = qh.horizontal_link;
cache_clean(&async_list_head.horizontal_link, 4);

// Ring the doorbell and wait for hardware acknowledgment
usb_regs.USBCMD.modify(|r| r | (1 << 6)); // Set IAA doorbell
wait_for_async_advance().await;             // Wait for USBSTS[AAI]

// 16. Return data to caller
Ok(bytes_transferred)
```

### Hardware Execution Flow

What the USB controller does after QH is linked:

```
1. Controller traverses async schedule circular list
2. Finds QH with Active qTD
3. Copies qTD → QH overlay area
4. Starts SETUP transaction:
   - Sends SETUP token + device address + endpoint 0
   - Sends 8 bytes of setup packet (DATA0)
   - Receives ACK from device
5. Clears Active bit in overlay token
6. Advances to next qTD (DATA stage):
   - Copies next qTD to overlay
   - Sends IN token + device address + endpoint 0
   - Receives data from device (DATA1, then DATA0, alternating)
   - Sends ACK to device
   - Decrements "Total Bytes" in overlay token
   - Repeats until all data received or short packet
7. Clears Active bit in overlay token
8. Advances to next qTD (STATUS stage):
   - Copies next qTD to overlay
   - Sends OUT token + device address + endpoint 0
   - Sends zero-length DATA1 packet
   - Receives ACK from device
9. Clears Active bit in overlay token
10. Sees Terminate bit (no more qTDs)
11. Generates interrupt (if IOC was set)
12. Moves to next QH in schedule
```

---

## Complete Control Transfer Example

### Example: Get Device Descriptor

Let's implement a complete `GET_DESCRIPTOR` request to read the device descriptor (18 bytes):

```rust
use core::mem::size_of;

// Setup packet for GET_DESCRIPTOR (Device)
#[repr(C, packed)]
struct GetDeviceDescriptorSetup {
    bmRequestType: u8,   // 0x80 = Device-to-Host, Standard, Device
    bRequest: u8,        // 0x06 = GET_DESCRIPTOR
    wValue: u16,         // 0x0100 = Device Descriptor
    wIndex: u16,         // 0x0000
    wLength: u16,        // 0x0012 = 18 bytes
}

async fn get_device_descriptor(
    usb_regs: &UsbRegisters,
    device_address: u8,
) -> Result<[u8; 18], UsbError> {
    const DEVICE_DESCRIPTOR_SIZE: usize = 18;
    
    // Allocate aligned structures
    let mut qh = Box::new(QueueHead::new());
    let mut qtd_setup = Box::new(QTD::new());
    let mut qtd_data = Box::new(QTD::new());
    let mut qtd_status = Box::new(QTD::new());
    
    // Setup packet
    let setup_packet = GetDeviceDescriptorSetup {
        bmRequestType: 0x80,
        bRequest: 0x06,
        wValue: 0x0100_u16.to_le(),
        wIndex: 0x0000_u16.to_le(),
        wLength: (DEVICE_DESCRIPTOR_SIZE as u16).to_le(),
    };
    
    // Data buffer
    let mut descriptor_buffer = [0u8; DEVICE_DESCRIPTOR_SIZE];
    
    // ==== Configure QH ====
    qh.endpoint_characteristics = 
        (15 << 28) |             // NAK reload = 15
        (1 << 27) |              // Control endpoint flag (C=1 for FS/LS control)
        (64 << 16) |             // Max packet size (assume 64 for now)
        (0 << 15) |              // Not head of reclamation list
        (1 << 14) |              // DTC = 1 (get toggle from qTD)
        ((0b00 as u32) << 12) |  // Full speed
        (0 << 8) |               // Endpoint 0
        (device_address as u32); // Device address
    
    qh.endpoint_capabilities = 
        (1 << 30) |              // Multiplier = 1
        (0 << 0);                // No split transaction
    
    qh.current_qtd_pointer = 0;
    
    // ==== Configure SETUP qTD ====
    qtd_setup.next_qtd_pointer = 
        (&*qtd_data as *const _ as u32) & !0x1F;
    qtd_setup.alternate_next_qtd_pointer = 0x00000001;
    qtd_setup.token = 
        (0 << 31) |              // DATA0 for SETUP
        (8 << 16) |              // 8 bytes
        (0 << 15) |              // No IOC yet
        (0 << 12) |              // C_Page = 0
        (3 << 10) |              // CERR = 3
        (0b10 << 8) |            // PID = SETUP
        (1 << 7) |               // Active
        (0 << 0);                // Clear status bits
    qtd_setup.buffer_pointer_0 = 
        (&setup_packet as *const _ as u32);
    qtd_setup.buffer_pointer_1 = 0;
    qtd_setup.buffer_pointer_2 = 0;
    qtd_setup.buffer_pointer_3 = 0;
    qtd_setup.buffer_pointer_4 = 0;
    
    // ==== Configure DATA qTD (IN direction) ====
    qtd_data.next_qtd_pointer = 
        (&*qtd_status as *const _ as u32) & !0x1F;
    qtd_data.alternate_next_qtd_pointer = 0x00000001;
    qtd_data.token = 
        (1 << 31) |              // DATA1 for first data packet
        (DEVICE_DESCRIPTOR_SIZE << 16) | // 18 bytes
        (0 << 15) |              // No IOC yet
        (0 << 12) |              // C_Page = 0
        (3 << 10) |              // CERR = 3
        (0b01 << 8) |            // PID = IN
        (1 << 7) |               // Active
        (0 << 0);                // Clear status bits
    qtd_data.buffer_pointer_0 = 
        descriptor_buffer.as_ptr() as u32;
    qtd_data.buffer_pointer_1 = 0;
    qtd_data.buffer_pointer_2 = 0;
    qtd_data.buffer_pointer_3 = 0;
    qtd_data.buffer_pointer_4 = 0;
    
    // ==== Configure STATUS qTD (OUT direction - opposite of DATA) ====
    qtd_status.next_qtd_pointer = 0x00000001; // Terminate
    qtd_status.alternate_next_qtd_pointer = 0x00000001;
    qtd_status.token = 
        (1 << 31) |              // DATA1 for STATUS
        (0 << 16) |              // 0 bytes (ZLP)
        (1 << 15) |              // IOC = 1 (generate interrupt)
        (0 << 12) |              // C_Page = 0
        (3 << 10) |              // CERR = 3
        (0b00 << 8) |            // PID = OUT
        (1 << 7) |               // Active
        (0 << 0);                // Clear status bits
    qtd_status.buffer_pointer_0 = 0; // No buffer for ZLP
    qtd_status.buffer_pointer_1 = 0;
    qtd_status.buffer_pointer_2 = 0;
    qtd_status.buffer_pointer_3 = 0;
    qtd_status.buffer_pointer_4 = 0;
    
    // ==== Link qTDs to QH ====
    // Point overlay's Next qTD to the FIRST qTD (setup), not setup's next pointer.
    // The overlay token must be inactive (Active=0) so the controller fetches
    // the qTD pointed to by overlay_next_qtd.
    qh.overlay_next_qtd = (&*qtd_setup as *const _ as u32) & !0x1F;
    qh.overlay_alt_next_qtd = 0x00000001;  // Terminate
    qh.overlay_token = 0;  // Active=0 → controller advances to overlay_next_qtd
    
    // ==== Cache Management ====
    unsafe {
        cache_clean(&setup_packet as *const _ as *const u8, 8);
        cache_clean(&*qtd_setup as *const _ as *const u8, size_of::<QTD>());
        cache_clean(&*qtd_data as *const _ as *const u8, size_of::<QTD>());
        cache_clean(&*qtd_status as *const _ as *const u8, size_of::<QTD>());
        cache_clean(&*qh as *const _ as *const u8, size_of::<QH>());
    }
    
    // ==== Add to Async Schedule ====
    // (Simplified - real code would properly manage the circular list)
    let async_list_addr = usb_regs.ASYNCLISTADDR.read();
    let old_head: *mut QH = async_list_addr as *mut QH;
    
    unsafe {
        qh.horizontal_link = (*old_head).horizontal_link;
        cache_clean(&qh.horizontal_link as *const _ as *const u8, 4);
        
        (*old_head).horizontal_link = 
            (&*qh as *const _ as u32 & !0x1F) | 0b010;
        cache_clean(&(*old_head).horizontal_link as *const _ as *const u8, 4);
    }
    
    // ==== Wait for Completion ====
    let start_time = now();
    loop {
        // Invalidate QH overlay to see hardware updates
        unsafe {
            cache_invalidate(&qh.overlay_token as *const _ as *const u8, 4);
        }
        
        let overlay_token = qh.overlay_token;
        
        // Check if transfer is complete (Active bit clear)
        if (overlay_token & (1 << 7)) == 0 {
            break;
        }
        
        // Timeout check
        if now() - start_time > Duration::from_secs(1) {
            // Clean up and return error
            return Err(UsbError::Timeout);
        }
        
        // Yield to async executor
        yield_now().await;
    }
    
    // ==== Check Status ====
    unsafe {
        cache_invalidate(&*qtd_setup as *const _ as *const u8, size_of::<QTD>());
        cache_invalidate(&*qtd_data as *const _ as *const u8, size_of::<QTD>());
        cache_invalidate(&*qtd_status as *const _ as *const u8, size_of::<QTD>());
    }
    
    // Check each qTD for errors
    if (qtd_setup.token & (1 << 6)) != 0 {
        return Err(UsbError::TransactionError);
    }
    if (qtd_data.token & (1 << 6)) != 0 {
        return Err(UsbError::TransactionError);
    }
    if (qtd_status.token & (1 << 6)) != 0 {
        return Err(UsbError::Stall);
    }
    
    // Check bytes transferred
    let bytes_remaining = (qtd_data.token >> 16) & 0x7FFF;
    let bytes_transferred = DEVICE_DESCRIPTOR_SIZE - (bytes_remaining as usize);
    
    if bytes_transferred != DEVICE_DESCRIPTOR_SIZE {
        return Err(UsbError::ProtocolError);
    }
    
    // ==== Read Result ====
    unsafe {
        cache_invalidate(
            descriptor_buffer.as_ptr() as *const u8,
            DEVICE_DESCRIPTOR_SIZE
        );
    }
    
    // ==== Clean Up ====
    // Remove QH from async schedule using Async Advance Doorbell.
    // Step 1: Unlink QH from the circular list
    unsafe {
        (*old_head).horizontal_link = qh.horizontal_link;
        cache_clean(&(*old_head).horizontal_link as *const _ as *const u8, 4);
    }
    // Step 2: Ring the doorbell and wait for hardware to acknowledge
    // (guarantees the controller is no longer accessing this QH)
    usb_regs.USBCMD.modify(|r| r | (1 << 6)); // Set IAA (Interrupt on Async Advance)
    wait_for_async_advance().await;             // Wait for USBSTS[AAI]
    
    // Step 3: Now safe to free/reuse the QH and qTDs
    
    Ok(descriptor_buffer)
}
```

---

## Error Handling

### Common Error Conditions

#### 1. STALL

The device responds with a STALL handshake (endpoint halted):

```rust
// STALL is indicated by Halted=1 with no other error bits set.
// The EHCI controller sets Halted when the device returns a STALL PID.
let token = qtd.token;
if (token & (1 << 6)) != 0 {  // Halted
    let other_errors = token & ((1 << 5) | (1 << 4) | (1 << 3));  // DBE, Babble, XactErr
    if other_errors == 0 {
        // Halted with no other errors → device STALLed
        return Err(UsbError::Stall);
    }
    // Otherwise, check specific error bits below
}
```

**Cause**: Device doesn't support the request, or endpoint is halted.

**Recovery**: 
- For control EP0: STALL is self-clearing (USB 2.0 §8.5.3.4) — next SETUP clears it
- For other endpoints: send `CLEAR_FEATURE(ENDPOINT_HALT)` to clear stall
- Also clear the QH overlay's Halted bit before re-using the QH

#### 2. Transaction Error

Timeout, CRC error, or other bus error:

```rust
if (qtd.token & (1 << 3)) != 0 {
    return Err(UsbError::TransactionError);
}
```

**Cause**: Bus noise, device disconnected, timing issues.

**Recovery**: 
- Retry the transfer
- Check device connection
- Reset bus if persistent

#### 3. Babble

Device sent more data than expected:

```rust
if (qtd.token & (1 << 4)) != 0 {
    return Err(UsbError::Babble);
}
```

**Cause**: Device firmware bug or protocol violation.

**Recovery**: Halt endpoint, possibly reset device.

#### 4. NAK Timeout

Device keeps responding with NAK:

```rust
// Check if we've been Active too long
if elapsed > TIMEOUT && (qtd.token & (1 << 7)) != 0 {
    return Err(UsbError::Timeout);
}
```

**Cause**: Device not ready (legitimate) or device hung.

**Recovery**: 
- For control transfers: consider it a timeout
- For bulk/interrupt: this is normal, keep waiting

### Error Recovery Strategy

```rust
fn handle_qtd_error(qtd: &QTD) -> Result<(), UsbError> {
    let token = qtd.token;
    
    // Check Active bit first
    if (token & (1 << 7)) != 0 {
        return Err(UsbError::Timeout); // Still active = timeout
    }
    
    // Check Halted bit
    if (token & (1 << 6)) == 0 {
        return Ok(()); // No halt = success
    }
    
    // Halted - check specific error bits
    if (token & (1 << 5)) != 0 {
        return Err(UsbError::DataBufferError);
    }
    if (token & (1 << 4)) != 0 {
        return Err(UsbError::Babble);
    }
    if (token & (1 << 3)) != 0 {
        return Err(UsbError::TransactionError);
    }
    
    // Halted but no specific error = STALL
    Err(UsbError::Stall)
}
```

---

## Implementation Patterns

> **Note**: The code examples in this section use `Box` and `Vec` for clarity, but
> the actual `imxrt-usbh` implementation is `no_std` + `no_alloc`. All QH and qTD
> structures will be pre-allocated in static pools (see Pattern 1) and accessed by
> index, similar to the RP2040 implementation's `async_pool::Pool`.

### Pattern 1: QH Pool Management

Pre-allocate QHs in a static pool (no heap allocation):

```rust
pub struct QHPool {
    qhs: [QueueHead; 8],
    in_use: [bool; 8],
}

impl QHPool {
    pub fn alloc(&mut self) -> Option<&'static mut QueueHead> {
        for (i, in_use) in self.in_use.iter_mut().enumerate() {
            if !*in_use {
                *in_use = true;
                return Some(unsafe {
                    &mut *(&mut self.qhs[i] as *mut QueueHead)
                });
            }
        }
        None
    }
    
    pub fn free(&mut self, qh: &mut QueueHead) {
        let addr = qh as *mut QueueHead as usize;
        let base = &self.qhs[0] as *const QueueHead as usize;
        let index = (addr - base) / size_of::<QueueHead>();
        self.in_use[index] = false;
    }
}
```

### Pattern 2: qTD Chain Builder

Helper for constructing qTD chains:

```rust
pub struct QTDChain {
    qtds: Vec<Box<QTD>>, // Or use fixed-size array
}

impl QTDChain {
    pub fn new() -> Self {
        Self { qtds: Vec::new() }
    }
    
    pub fn add_setup(&mut self, setup: &SetupPacket) -> &mut Self {
        let mut qtd = Box::new(QTD::new());
        qtd.token = /* ... SETUP token ... */;
        qtd.buffer_pointer_0 = setup as *const _ as u32;
        self.qtds.push(qtd);
        self
    }
    
    pub fn add_data_in(&mut self, buffer: &mut [u8], toggle: bool) -> &mut Self {
        let mut qtd = Box::new(QTD::new());
        qtd.token = /* ... IN token with toggle ... */;
        qtd.buffer_pointer_0 = buffer.as_ptr() as u32;
        self.qtds.push(qtd);
        self
    }
    
    pub fn add_status_out(&mut self, toggle: bool) -> &mut Self {
        let mut qtd = Box::new(QTD::new());
        qtd.token = /* ... OUT ZLP token ... */;
        self.qtds.push(qtd);
        self
    }
    
    pub fn link(&mut self) {
        for i in 0..self.qtds.len()-1 {
            self.qtds[i].next_qtd_pointer = 
                &*self.qtds[i+1] as *const _ as u32 & !0x1F;
        }
        self.qtds.last_mut().unwrap().next_qtd_pointer = 0x00000001;
    }
}
```

### Pattern 3: Async Control Transfer

Wrapper that handles all the details:

```rust
pub async fn control_transfer(
    &self,
    device_addr: u8,
    speed: UsbSpeed,
    setup: SetupPacket,
    data: Option<&mut [u8]>,
) -> Result<usize, UsbError> {
    // Allocate resources
    let qh = self.qh_pool.alloc().await;
    let qtd_setup = Box::new(QTD::new());
    let qtd_data = data.as_ref().map(|_| Box::new(QTD::new()));
    let qtd_status = Box::new(QTD::new());
    
    // Configure structures
    // ... (as shown in previous examples) ...
    
    // Cache management
    self.flush_all();
    
    // Link to schedule
    self.link_qh_to_async_schedule(qh);
    
    // Wait for completion
    self.wait_for_qh_complete(qh).await?;
    
    // Check errors
    self.check_qtd_errors(&[qtd_setup, qtd_data, qtd_status])?;
    
    // Extract result
    let bytes_transferred = self.get_bytes_transferred(qtd_data);
    
    // Clean up
    self.unlink_qh_from_async_schedule(qh);
    self.qh_pool.free(qh);
    
    Ok(bytes_transferred)
}
```

---

## Advanced Topics

### Short Packet Detection

When a device sends less data than requested (a "short packet"), the EHCI controller
follows the **Alternate Next qTD Pointer** instead of the normal Next qTD Pointer.
This allows you to skip remaining data qTDs and jump directly to the status stage:

```rust
// Point alternate to the STATUS qTD — if device sends a short packet
// during the data stage, the controller skips directly to status.
qtd_data.alternate_next_qtd_pointer = (&qtd_status as *const _ as u32) & !0x1F;

// After transfer, check bytes remaining
let bytes_remaining = (qtd_data.token >> 16) & 0x7FFF;
if bytes_remaining > 0 {
    // Short packet received — device sent less than requested.
    // This is normal for many descriptors (e.g., string descriptors
    // may be shorter than the requested wLength).
    let actual_bytes = requested_bytes - bytes_remaining;
}
```

> **When to use alternate pointer**: For control transfers with a single data qTD,
> setting the alternate pointer to the status qTD ensures the status stage always
> executes even on short packets. For multi-qTD data stages (large transfers split
> across multiple qTDs), the alternate pointer on each data qTD should point to the
> status qTD so the controller can short-circuit to status from any point.
>
> If you set the alternate pointer to Terminate (0x00000001), a short packet will
> cause the controller to halt the QH without executing the status stage.

### Data Toggle Synchronization

The data toggle (DATA0/DATA1) is critical for reliable transfers. Each side uses
it to detect duplicate or missed packets.

**Control transfer toggle rules** (USB 2.0 §8.5.3, §8.6.1):
```
SETUP stage:  Always DATA0 (SETUP PID resets the device's toggle)
DATA stage:   Starts at DATA1, then alternates (DATA1, DATA0, DATA1, ...)
STATUS stage: Always DATA1
```

With `DTC=1` in the QH (Data Toggle Control from qTD), you set the toggle explicitly
in each qTD's token, so the above rules are encoded directly in the qTD chain.

**Non-control endpoint toggle rules**:
```rust
// Bulk/interrupt endpoints maintain a running toggle.
// cotton-usb-host tracks this via `data_toggle: &Cell<bool>`.
// With DTC=0 in the QH, the controller uses the QH overlay's toggle bit.

// After SET_CONFIGURATION or SET_INTERFACE, all non-control
// endpoints reset their toggle to DATA0 (USB 2.0 §9.4.5, §9.4.10).
data_toggle.set(false);
```

### Split Transactions

For full-/low-speed devices behind high-speed hubs:

```rust
qh.endpoint_capabilities = 
    (hub_address << 16) |    // Hub address
    (port_number << 23) |    // Port number
    (0x1C << 8) |            // C-mask (complete split)
    (0x01 << 0);             // S-mask (start split)
```

### High-Bandwidth Endpoints

For high-speed bulk/interrupt with >1 transaction per microframe:

```rust
qh.endpoint_capabilities = 
    (2 << 30) |  // Multiplier = 2 (2 transactions per microframe)
    /* ... other fields ... */;
```

---

## References

### Official Specifications

1. **EHCI Specification** (Enhanced Host Controller Interface for USB)
   - Section 3.5: Queue Element Transfer Descriptor (qTD)
   - Section 3.6: Queue Head (QH)
   - Section 4.10: Operational Model (Control Transfers)

2. **USB 2.0 Specification**
   - Chapter 5: USB Data Flow Model
   - Chapter 8: Protocol Layer
   - Section 8.5.3: Control Transfers

3. **i.MX RT 1060 Reference Manual**
   - Chapter 56: USB

### Code References

1. **Linux Kernel**: `drivers/usb/host/ehci-q.c`
2. **TinyUSB**: `src/host/ehci/ehci.c`
3. **FreeBSD**: `sys/dev/usb/controller/ehci.c`

### Additional Reading

- "USB Complete" by Jan Axelson (4th Edition)
- "USB System Architecture" by Don Anderson
- EHCI FAQ and errata documents

---

## Summary

**Key Takeaways**:

1. **Control transfers have 3 stages**: SETUP, DATA (optional), STATUS
2. **QH represents an endpoint**: Contains endpoint characteristics and overlay area
3. **qTD represents a transaction**: Contains token (PID, length, toggle) and buffer pointers
4. **qTDs are chained**: Hardware walks the chain automatically
5. **Overlay area shows status**: CPU must invalidate cache before reading
6. **Cache coherency is critical**: Flush before hardware reads, invalidate before CPU reads
7. **Error handling is important**: Check all status bits after transfer
8. **Async schedule is circular**: QHs form a ring that hardware traverses continuously

**Implementation Checklist**:

- [ ] Allocate QH (64-byte aligned) and qTD (32-byte aligned) from static pools
- [ ] Configure QH endpoint characteristics (address, speed, max packet, NAK reload, C flag)
- [ ] Build qTD chain: SETUP → DATA (optional) → STATUS
- [ ] Link qTDs together via `next_qtd_pointer`
- [ ] Set alternate qTD pointers for short packet handling (point data qTDs → status qTD)
- [ ] Set correct PID codes: SETUP=0b10, IN=0b01, OUT=0b00
- [ ] Set correct data toggles: SETUP=DATA0, first DATA=DATA1, STATUS=DATA1
- [ ] Set IOC (Interrupt on Complete) on the STATUS qTD
- [ ] Point QH overlay's `next_qtd` to the first qTD (SETUP), with `overlay_token` Active=0
- [ ] Flush cache: qTDs, QH, setup packet, TX buffer
- [ ] Link QH into async schedule circular list
- [ ] Wait for completion: invalidate QH, poll overlay Active bit (or use interrupt + waker)
- [ ] Invalidate cache: qTDs, RX buffer
- [ ] Check error bits: Halted, XactErr, Babble, Data Buffer Error → map to `UsbError`
- [ ] Extract transferred byte count from data qTD's `Total Bytes to Transfer` field
- [ ] Unlink QH from schedule **using Async Advance Doorbell** (USBCMD[IAA] → wait USBSTS[AAI])
- [ ] Return QH and qTDs to static pools

---

**Document Version**: 1.1  
**Date**: 2025-10-06  
**Last Updated**: 2026-02-06
