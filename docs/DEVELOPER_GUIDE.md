# imxrt-usbh Developer Guide

This document provides the architectural context, technical details, and practical
knowledge needed to maintain and extend the `imxrt-usbh` crate. It assumes
familiarity with embedded Rust and USB basics; see the [References](#references)
section for learning resources.

## Table of Contents

1. [Architecture Overview](#architecture-overview)
2. [Source Code Structure](#source-code-structure)
3. [EHCI Host Controller Primer](#ehci-host-controller-primer)
4. [Key Data Structures](#key-data-structures)
5. [Transfer Lifecycle](#transfer-lifecycle)
6. [Interrupt Handling and Async Pattern](#interrupt-handling-and-async-pattern)
7. [Cache Coherency](#cache-coherency)
8. [Register Access (RAL Module)](#register-access-ral-module)
9. [Hardware Setup — Teensy 4.1](#hardware-setup--teensy-41)
10. [i.MX RT EHCI Quirks](#imx-rt-ehci-quirks)
11. [Known Limitations and Future Work](#known-limitations-and-future-work)
12. [Common Pitfalls](#common-pitfalls)
13. [References](#references)

---

## Architecture Overview

`imxrt-usbh` implements the `HostController` trait from `cotton-usb-host`, enabling
the cotton USB host stack to enumerate devices, read HID reports, transfer bulk data,
and manage USB hubs. The stack is layered as follows:

```
┌─────────────────────────────────────────────────┐
│           Application Code (RTIC v2)            │
├─────────────────────────────────────────────────┤
│  cotton-usb-host-hid / cotton-usb-host-msc      │  (device class drivers)
├─────────────────────────────────────────────────┤
│     UsbBus<HC: HostController>                  │  (hardware-agnostic bus logic)
│  - device enumeration, hub support              │
│  - address assignment, configuration            │
│  - device_events() stream                       │
├─────────────────────────────────────────────────┤
│     HostController trait                        │  (abstraction boundary)
├─────────────────────────────────────────────────┤
│  ImxrtHostController      (this crate)      │
│  UsbShared (ISR ↔ async task)                   │
│  UsbStatics (pipe/QH/qTD pools)                 │
│  Cache coherency layer                          │
├─────────────────────────────────────────────────┤
│  imxrt-ral USB2/USBPHY2       (register access) │
│  EHCI hardware (QH/qTD DMA engine)              │
└─────────────────────────────────────────────────┘
```

The driver is designed around three core types that mirror the RP2040 reference
implementation in cotton-usb-host:

- **`UsbShared`** — Interrupt-safe state shared between the ISR and async tasks.
  Contains waker registrations. Lives in a `static`.
- **`UsbStatics`** — Static-lifetime resource pools for QHs, qTDs, pipe slots, and
  receive buffers. Not accessed by the ISR. Lives in a `ConstStaticCell`.
- **`ImxrtHostController`** — The main driver. Owns the USB register block
  instances and references to `UsbShared` and `UsbStatics`.

### Key Dependencies

| Crate | Purpose |
|-------|---------|
| `cotton-usb-host` (0.2.1+) | `HostController` trait, `UsbBus`, `Pool`/`Pooled` allocation |
| `imxrt-ral` (0.6) | Register definitions and `read_reg!`/`write_reg!`/`modify_reg!` macros |
| `cortex-m` (0.7+) | Cache management intrinsics, memory barriers |
| `rtic-common` (1) | `CriticalSectionWakerRegistration` for ISR↔async waker communication |
| `static-cell` | `ConstStaticCell` for `'static` lifetime pools |
| `defmt` / `log` | Optional, feature-gated debug logging |

---

## Source Code Structure

```
src/
├── lib.rs              — Crate root: #![no_std], module declarations, cotton re-exports
├── ehci.rs             — EHCI DMA structures: QueueHead, TransferDescriptor, FrameList,
│                         link pointer helpers, token/characteristic builders (~590 lines)
├── cache.rs            — D-cache clean + invalidate wrappers for DMA coherency
├── vcell.rs            — VCell<T>: volatile cell using UnsafeCell for DMA-visible structures
├── gpt.rs              — USB OTG built-in general purpose timer abstraction
├── log.rs              — Conditional defmt/log macros (feature-gated)
├── ral.rs              — RAL adapter: re-exports from imxrt-ral, non-generic Instance wrappers
└── host/
    ├── mod.rs           — Module declarations, re-exports, pool sizing constants
    ├── shared.rs        — UsbShared: ISR↔async bridge, on_irq(), wakers
    ├── statics.rs       — UsbStatics, RecvBuf: resource pools and DMA buffers
    ├── controller.rs    — ImxrtHostController: struct, new(), init(), port helpers
    ├── schedule.rs      — QH/qTD allocation (QtdSlot RAII), async/periodic schedule management
    ├── transfer.rs      — do_control_transfer, do_bulk_transfer, do_alloc_interrupt_pipe,
    │                      cache wrappers, periodic chain diagnostics
    ├── futures.rs       — TransferComplete future, AsyncAdvanceWait future
    ├── device_detect.rs — ImxrtDeviceDetect: Stream<Item = DeviceStatus>
    ├── interrupt_pipe.rs — ImxrtInterruptPipe: Stream<Item = InterruptPacket>, with Drop cleanup
    └── trait_impl.rs    — impl HostController for ImxrtHostController
```

### Pool Sizing Constants (host/mod.rs)

| Constant | Value | Meaning |
|----------|-------|---------|
| `NUM_QH` | 4 | QH slots for endpoint pipes (1 control + 3 bulk/interrupt) |
| `NUM_QTD` | 16 | qTD slots (each control transfer uses 2–3; each bulk/interrupt uses 1+) |
| `NUM_PIPE_WAKERS` | 5 | NUM_QH + 1 (control pipe at index 0, bulk/interrupt at 1..N) |

---

## EHCI Host Controller Primer

EHCI (Enhanced Host Controller Interface) is a hardware-level USB 2.0 host
controller specification. Key concepts:

- The host controller has a **DMA engine** that reads transfer descriptors from
  memory, executes USB transactions on the bus, and writes completion status
  back to memory — all without CPU involvement.
- **Queue Heads (QH)** represent USB endpoints. They are persistent, linked in
  circular lists, and contain endpoint characteristics plus an "overlay area"
  where the controller copies the active transfer's state.
- **Queue Transfer Descriptors (qTD)** represent individual transfer operations.
  They are chained together and linked to a QH. The controller walks the chain
  automatically.
- Two schedule types exist:
  - **Asynchronous schedule** — a circular linked list of QHs for control and
    bulk transfers. The controller continuously traverses this list.
  - **Periodic schedule** — a frame list (array of pointers) indexed by the
    current frame number, used for interrupt transfers. Each frame list entry
    points to a chain of QHs.

The controller is told about these schedules via two registers:
- `ASYNCLISTADDR` — points to the head of the async QH circular list
- `DEVICEADDR` (host-mode alias for `PERIODICLISTBASE`) — points to the frame list

Transfers are initiated by building qTD chains, linking them to a QH, and
inserting the QH into the appropriate schedule. The controller picks them up
on the next traversal cycle.

---

## Key Data Structures

### QueueHead (`ehci.rs`)

64 bytes, `#[repr(C, align(64))]`. Contains:

- **Horizontal link** (DWORD 0): Links QHs in a circular (async) or linear
  (periodic) list. Bits [4:1] encode the link type (QH=0b01). Bit 0 is
  the Terminate flag.
- **Endpoint characteristics** (DWORD 1): Device address, endpoint number,
  speed, max packet size, NAK reload count, control endpoint flag (`C`),
  data toggle control (`DTC`), head-of-reclamation-list flag (`H`).
- **Endpoint capabilities** (DWORD 2): Hub address/port for split transactions,
  S-mask/C-mask for periodic scheduling, high-bandwidth multiplier.
- **Current qTD pointer** (DWORD 3): Updated by hardware as it processes qTDs.
- **Overlay area** (DWORDs 4–11): Same format as a qTD. The controller copies
  the active qTD here and updates it during execution. Check the overlay's
  `token` field to monitor transfer status.
- **Software words** (DWORDs 12–15): Extra fields used by the driver (attached
  qTD pointer, buffer pointer, flags, PID, interval). These are in the 16-byte
  padding area and are never touched by hardware.

The overlay fields are individual `VCell<u32>` fields rather than an embedded
`TransferDescriptor`, because `TransferDescriptor`'s `align(32)` would insert
padding and break the 64-byte QH layout.

### TransferDescriptor / qTD (`ehci.rs`)

32 bytes, `#[repr(C, align(32))]`. Contains:

- **Next qTD pointer** (DWORD 0): Links qTDs in a chain. Bit 0 is Terminate.
- **Alternate next qTD** (DWORD 1): Followed on short packet detection. Point
  this to the status qTD in control transfers so short packets still complete
  the status stage.
- **Token** (DWORD 2): The critical field. Contains PID code (SETUP/IN/OUT),
  total bytes to transfer, data toggle bit, IOC (Interrupt on Complete) flag,
  error counter (CERR), and status bits (Active, Halted, errors).
- **Buffer pointers** (DWORDs 3–7): Up to 5 physical addresses, each pointing
  to a 4KB page. A single qTD can transfer up to 20KB.

### FrameList (`ehci.rs`)

4096-byte aligned array of 32 `VCell<u32>` entries (one per frame). Each entry
is either a terminate link or a pointer to a QH in the periodic schedule.
32 entries gives 32ms scheduling granularity.

### QtdSlot (RAII Guard, `schedule.rs`)

An allocation guard for qTD pool slots. On `Drop`, it clears the allocation
bitmap flag and zeroes all hardware fields. This eliminates manual `free_qtd()`
calls and ensures cleanup on error paths.

---

## Transfer Lifecycle

### Control Transfer (`do_control_transfer`)

1. **Allocate** a control pipe from the `Pool` (serializes EP0 access).
2. **Allocate** 2–3 qTDs from the qTD pool via `QtdSlot` RAII guards.
3. **Configure QH**: Set endpoint characteristics (address, EP0, speed, max
   packet size), capabilities (hub addr/port for `WithPreamble`), link overlay
   to first qTD.
4. **Build qTD chain**:
   - SETUP qTD: PID=SETUP, 8 bytes, data toggle=DATA0.
   - DATA qTD (optional): PID=IN or OUT, toggle=DATA1, buffer points to caller's data.
   - STATUS qTD: PID opposite of data direction, 0 bytes (ZLP), toggle=DATA1, IOC=1.
5. **Cache clean**: Flush setup packet, all qTDs, QH, and any outgoing data buffer.
6. **Link QH** to async schedule (after sentinel QH), enable ASE if needed.
7. **Await completion**: `TransferComplete` future registers a waker, re-enables
   transfer interrupts, and polls the status qTD's token for Active=0.
8. **Check errors**: Map EHCI error bits to `UsbError` variants.
9. **Unlink QH**: Remove from async schedule, ring the Async Advance Doorbell
   (`USBCMD[IAA]`), await `USBSTS[AAI]` via `AsyncAdvanceWait` future.
10. **Return**: qTDs are freed automatically when `QtdSlot` guards drop.

### Bulk Transfer (`do_bulk_transfer`)

Similar to control, but:
- Single qTD (no SETUP/STATUS stages).
- Data toggle tracked externally via `&Cell<bool>` (provided by cotton-usb-host).
- `TransferType::VariableSize` allows short packets; `FixedSize` does not.

### Interrupt Pipe (`do_alloc_interrupt_pipe`)

1. **Allocate** a bulk/interrupt pipe slot and a qTD.
2. **Configure QH**: Set endpoint characteristics, S-mask / C-mask for periodic
   scheduling based on the requested interval.
3. **Link QH** to the periodic schedule frame list.
4. **Return** an `ImxrtInterruptPipe` stream.

The stream's `poll_next()` checks the qTD's token. On completion, it copies
received data into an `InterruptPacket`, re-arms the qTD (preserving the data
toggle from the overlay), flushes it, and returns `Poll::Ready`. On `Drop`,
the pipe unlinks the QH from the periodic schedule and frees all resources.

---

## Interrupt Handling and Async Pattern

The ISR uses a **disable-on-handle / re-enable-on-poll** pattern (from the
RP2040 reference implementation) to prevent IRQ storms:

### ISR (`UsbShared::on_irq`)

1. Read `USBSTS` (active interrupt flags).
2. W1C-acknowledge handled bits.
3. **Disable** those same interrupt bits in `USBINTR`.
4. Wake the appropriate wakers:
   - `PCI` (Port Change) → `device_waker`
   - `USBINT` or `USBERRINT` → all `pipe_wakers` (EHCI doesn't identify which QH completed)
   - `AAI` (Async Advance) → `async_advance_waker`

### Async Futures (`poll()`)

1. Register waker with the appropriate `CriticalSectionWakerRegistration`.
2. **Re-enable** the relevant interrupt bits in `USBINTR`.
3. Cache-invalidate the QH/qTD and check hardware status.
4. Return `Pending` (wait for next wake) or `Ready` (transfer done).

The waker registration happens **before** re-enabling interrupts to avoid a race
where the interrupt fires between enable and register.

### ISR Binding

The hardware examples (in `imxrt-hal`) use manual ISR installation (vector table
patching at address `0x2000_0000 + 16*4 + 112*4` for `USB_OTG2` IRQ #112) rather than RTIC
`#[task(binds = USB_OTG2)]`. This is because RTIC's dispatchers use different
interrupts. The public API for the ISR is `UsbShared::on_usb_irq(usb_base)`,
which avoids exposing RAL types to application code.

---

## Cache Coherency

Cache coherency is the **#1 source of bugs** in this driver. The Cortex-M7 has a
32KB write-back L1 data cache that operates on 32-byte cache lines. The USB EHCI
controller's DMA engine bypasses this cache entirely.

### Rules

| Situation | Required Operation | Function |
|-----------|--------------------|----------|
| CPU wrote data that DMA will read | **Clean** (flush dirty lines to memory) | `cache_clean_qh()`, `cache_clean_qtd()`, `cache_clean_buffer()` |
| DMA wrote data that CPU will read | **Invalidate** (discard cached copy) | `invalidate_dcache_by_address()` |
| Bidirectional structure (QH overlay) | **Clean + Invalidate** | `clean_invalidate_dcache_by_address()` |

### Alignment

DMA structures must be aligned to cache line boundaries to prevent **false sharing**
(where a cache operation on one structure corrupts an adjacent one):

- QueueHead: `#[repr(C, align(64))]` — 2 full cache lines, padded from 48 to 64 bytes.
- TransferDescriptor: `#[repr(C, align(32))]` — exactly 1 cache line.
- FrameList: `#[repr(C, align(4096))]` — page-aligned.

### Alternative: DTCM Placement

The i.MX RT's DTCM (Data Tightly-Coupled Memory, at `0x2000_0000`) is not cached
and has single-cycle deterministic access. Placing QH/qTD pools in DTCM via a
linker section (`#[link_section = ".dtcm_dma"]`) would eliminate cache coherency
concerns for descriptors entirely, though data buffers (caller-provided) would
still need cache management. This approach is not currently used but is a viable
optimization.

### Debugging Cache Issues

- **Symptom**: "Works sometimes" or "works at low speed but fails at high speed."
- **Quick test**: Disable the D-cache entirely — if the problem disappears, it's
  a cache coherency bug.
- **Nuclear option**: `scb.clean_invalidate_dcache()` (flush entire cache) before
  each DMA boundary — slow but proves correctness.
- **Traps**: Adding `defmt` prints or delays can "fix" the issue by causing
  incidental cache evictions.

For a comprehensive treatment, see `docs/design/CACHE_COHERENCY.md`.

---

## Register Access (RAL Module)

The RAL module uses `imxrt-ral` 0.6 for register definitions and the
`read_reg!` / `write_reg!` / `modify_reg!` macros (from `ral-registers`,
re-exported by `imxrt-ral`).

The `src/ral.rs` adapter module glob-imports all register field definitions
from `imxrt_ral::usb` and `imxrt_ral::usbphy`, then provides non-generic
`Instance` wrapper types that shadow the const-generic
`imxrt_ral::Instance<T, N>`. This lets the rest of the driver avoid being
generic over the USB instance number while still working with the
`read_reg!`/`write_reg!`/`modify_reg!` macros.

All host-mode register fields are present in `imxrt-ral`:

| EHCI Spec Name | RAL Name | Notes |
|----------------|----------|-------|
| USBCMD | `USBCMD` | `RS`, `PSE`, `ASE`, `IAA` all present |
| USBSTS | `USBSTS` | `UI`, `PCI`, `AAI`, `HCH` all present |
| USBINTR | `USBINTR` | `UE`, `PCE`, `AAE`, `UAIE`, `UPIE` all present |
| PERIODICLISTBASE | `DEVICEADDR` | Use `BASEADR` field (bits [31:12]) — host-mode alias |
| ASYNCLISTADDR | `ASYNCLISTADDR` | Use `ASYBASE` field (bits [31:5]) |
| PORTSC | `PORTSC1` | `CCS`, `CSC`, `PE`, `PR`, `PP`, `PSPD` all present |
| USBMODE | `USBMODE` | `CM_3` = host mode |

### Register Coding Guidelines

- **Always use symbolic RAL names** — raw pointer access with hardcoded addresses
  has caused subtle bugs (e.g., using GPIO7 address when GPIO8 was intended).
- When raw access is needed (e.g., debug readback of registers owned by a driver),
  document why in a `// NOTE:` comment.
- Register names are in `imxrt-ral/svd/imxrt1062.svd` and
  `imxrt-ral/src/blocks/imxrt1061/`.

### Hardware Instances

| Resource | Base Address | NVIC IRQ |
|----------|-------------|----------|
| USB OTG2 (host) | `0x402E_0200` | 112 (`USB_OTG2`) |
| USBPHY2 | `0x400D_A000` | 66 |
| USB OTG1 (device, not used) | `0x402E_0000` | 113 |

---

## Hardware Setup — Teensy 4.1

### USB2 Host Port

The secondary USB port uses a **5-pin header** on the Teensy 4.1 (directly behind
the Ethernet jack). The host port is **not powered from the programming USB
connector** — you must supply external 5V to VBUS.

| Pin | Signal | Notes |
|-----|--------|-------|
| 1 | GND | |
| 2 | D+ | |
| 3 | D- | |
| 4 | VBUS | Connect to external 5V supply |
| 5 | ID | Leave unconnected (host mode) |

### VBUS Power Control

The Teensy 4.1 has an on-board load switch gating 5V VBUS. The enable input is
connected to `GPIO_EMC_40` (ALT5 = GPIO3_IO26 / fast GPIO8_IO26). The driver
configures this pin as a GPIO output driven HIGH:

```rust
// IOMUXC: set pad to ALT5 (GPIO), slow speed, weak pull
IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_40 = 5;
IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_40 = 0x0008;
// GPIO8: direction = output, drive HIGH
GPIO8_GDIR |= 1 << 26;
GPIO8_DR_SET = 1 << 26;
```

### Clock Prerequisites (Caller Responsibility)

Before calling `ImxrtHostController::init()`:

1. **USB2 PLL**: Set `CCM_ANALOG_PLL_USB2` — enable, set power, wait for lock.
2. **Clock gate**: Enable USB OTG2 in `CCM_CCGR6`.
3. **VBUS GPIO**: Drive `GPIO_EMC_40` HIGH as shown above.

See the RTIC examples in the `imxrt-hal` repository for complete initialization
sequences using the `board` crate.

---

## i.MX RT EHCI Quirks

The i.MX RT USB controller is EHCI-compatible but has several NXP-specific behaviors:

1. **`USBMODE` must be written immediately after controller reset.** The mode
   register locks after a few cycles — delayed writes may silently fail.

2. **`PERIODICLISTBASE` shares a register offset with `DEVICEADDR`.** In host
   mode, use `DEVICEADDR::BASEADR` (bits [31:12]) to set the periodic frame list
   base address.

3. **`ASYNCLISTADDR` uses `ASYBASE`** (bits [31:5]) for the async list pointer
   in host mode.

4. **`PORTSC` is named `PORTSC1`** in the RAL. Only one port exists per controller.

5. **Port speed is in `PORTSC1::PSPD`** (bits [27:26]) — this is an NXP extension
   not in the standard EHCI spec. Values: 0=FS, 1=LS, 2=HS.

6. **NXP-specific interrupt bits**: `USBSTS` bits 18 (`UAI`) and 19 (`UPI`)
   indicate async/periodic completion. The ISR checks these in addition to the
   standard `USBINT` (bit 0).

7. **Bus configuration**: `SBUSCFG` should be set to `AHBBRST=0b001` (INCR4 burst)
   during initialization — matches the Teensyduino USBHost_t36 reference.

8. **EHCI `CONFIGFLAG`** is read-only and always 1 on this controller.

### Initialization Sequence Summary

| Step | Register(s) | Purpose |
|------|-------------|---------|
| 1 | USBPHY2 `CTRL_SET/CLR` | PHY soft-reset, clear clock gate |
| 2 | USBPHY2 `CTRL_SET` | Enable UTMI levels (LS through HS hub) |
| 3 | USBPHY2 `PWD = 0` | Clear all power-down bits |
| 4 | `USBCMD` RST | Controller reset (self-clearing) |
| 5 | `USBMODE = CM_3` | **Immediately** set host mode |
| 6 | `SBUSCFG` | INCR4 burst mode |
| 7 | Init sentinel QH → `ASYNCLISTADDR` | Empty async schedule |
| 8 | `DEVICEADDR` (frame list) + `FRINDEX=0` | Periodic schedule base |
| 9 | `USBINTR=0`, W1C `USBSTS` | Clear pending interrupts |
| 10 | `USBCMD` | RS + PSE + ITC(1) + FS(32) + ASP(3) + ASPE |
| 11 | `PORTSC1` PP=1 | Root port power enable |
| 12 | USBPHY2 `CTRL_SET` | Enable disconnect detect |
| 13 | `USBINTR` | Enable: UE, UEE, PCE, SEE, AAE, UAIE, UPIE |

ASE (Async Schedule Enable) is deliberately **not** set during init — it's enabled
when the first endpoint pipe is added, to avoid wasting bus bandwidth traversing
an empty schedule.

---

## Known Limitations and Future Work

### Current Limitations

- **Hub support requires Full Speed**: The `hub-support` feature sets `PFSC=1`,
  forcing all connections to Full Speed (12 Mbps). Full EHCI split transaction
  scheduling for FS/LS devices behind HS hubs is not yet implemented in
  `cotton-usb-host`.
- **No isochronous transfers**: Audio/video class devices are not supported
  (requires iTD/siTD descriptors).
- **No USB suspend/resume or power management**.
- **Single USB port**: Only USB OTG2 (host port) is supported.
- **EHCI doesn't identify which QH completed**: The ISR wakes all pipe wakers
  on any transfer completion. Each future individually re-checks its own status.
  This is correct but not maximally efficient.

### Future Enhancement Opportunities

1. **High Speed hub support** — split transactions (TT scheduling in QH capabilities).
2. **Isochronous transfer support** — iTD/siTD descriptors for audio/video.
3. **USB1 controller support** — same register layout, different base address.
4. **i.MX RT 1050/1064/1170 support** — similar EHCI controllers, minor adaptations.
5. **DTCM placement** for QH/qTD pools — eliminates cache coherency concerns.
6. **Proper periodic schedule tree** — binary scheduling tree for optimal bandwidth.
7. **Performance optimization** — batch cache operations, reduce ISR wake scope.

---

## Common Pitfalls

### Cache Coherency (the #1 bug source)

- Always flush (clean) outgoing structures before DMA reads them.
- Always invalidate incoming structures before the CPU reads them.
- Adding `defmt` prints or delays can mask cache bugs by causing incidental evictions.
- If a transfer "works sometimes," suspect cache coherency first.

### PORTSC1 Write-1-to-Clear Bits

`PORTSC1` has W1C bits (`CSC`, `PEC`, `OCC`, `FPR`). A careless read-modify-write
will accidentally clear these status bits. Always mask them out when writing other
fields.

### QH/qTD Alignment

EHCI requires 64-byte alignment for QHs and 32-byte alignment for qTDs. The
hardware **silently ignores low-order address bits** — misaligned structures will
cause mysterious failures with no error indication.

### Async Advance Doorbell

After unlinking a QH from the async schedule, you **must** ring the Async Advance
Doorbell (`USBCMD[IAA]`) and wait for `USBSTS[AAI]` before freeing or reusing the
QH. The DMA engine may still be reading the QH during the current traversal cycle.

### Overlay vs. qTD Confusion

The QH overlay area has the same format as a qTD but is **not** a separate qTD
in memory. It's the controller's working copy. Write to qTDs linked via the
`next_qtd` chain; read completion status from the overlay or the original qTDs
after cache invalidation.

### RTIC Priority Starvation

All RTIC tasks at the same priority cannot preempt each other. If the USB task
runs continuously (e.g., tight polling loops), logging and other tasks will starve.
Use `Mono::delay()` or async yields to allow lower-priority work to execute. See
the examples for correct priority assignments.

---

## References

### Specifications

1. **i.MX RT 1060 Reference Manual** (IMXRT1060RM)
   - Chapter 56: USB OTG controller registers, QH/qTD formats
   - Chapter 57: USBPHY
   - Chapter 14: CCM / USB PLL configuration

2. **EHCI Specification** (Intel, 2002)
   - Section 3.5: qTD structure
   - Section 3.6: QH structure
   - Section 4.8: Async schedule (control/bulk)
   - Section 4.9: Periodic schedule (interrupt)
   - Section 4.10: Async Advance Doorbell

3. **USB 2.0 Specification**
   - Chapter 8: Protocol layer (data toggles, packet formats)
   - Chapter 9: Device framework (standard requests, descriptors)

### Code References

1. **[cotton-usb-host RP2040 implementation](https://github.com/pdh11/cotton/blob/main/cotton-usb-host/src/host/rp2040.rs)**
   — Primary reference for the `HostController` trait pattern, `UsbShared`/`UsbStatics`
   split, disable-on-handle ISR, `Pool`-based pipe allocation.

2. **[TinyUSB EHCI](https://github.com/hathach/tinyusb/blob/master/src/portable/ehci/ehci.c)**
   — Clean, minimal EHCI reference.

3. **[imxrt-usbd](https://github.com/imxrt-rs/imxrt-usbd/tree/master/src)**
   — Source of the RAL module. Shows i.MX RT PHY initialization, register naming conventions.

4. **[USBHost_t36](https://github.com/PaulStoffregen/USBHost_t36)** (Teensyduino)
   — C++ USB host for the same hardware. Useful for i.MX RT-specific initialization
   quirks and practical workarounds.

5. **[Linux EHCI driver](https://github.com/torvalds/linux/tree/master/drivers/usb/host)**
   (`ehci-hcd.c`, `ehci-q.c`, `ehci-sched.c`) — Authoritative, handles all edge cases.

### Learning Resources

- **RTIC v2 Book**: https://rtic.rs/2/book/en/ — RTIC async tasks, hardware/software
  task binding, resource sharing, monotonics.
- **The Embedded Rust Book**: https://docs.rust-embedded.org/book/ — `#![no_std]`,
  PAC/HAL/BSP layering, interrupt handling, peripheral singletons.

### Design Documents (in this repository)

- `docs/design/CACHE_COHERENCY.md` — Comprehensive cache coherency guide with
  examples, problem taxonomy, and solution patterns.
- `docs/design/USB_CONTROL_TRANSFERS.md` — Detailed walkthrough of EHCI control
  transfer mechanics including complete annotated code examples.
