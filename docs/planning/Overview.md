# Implementation Plan: i.MX RT USB Host Support for cotton-usb-host

## Overview

This document outlines the implementation plan for adding Teensy 4 (i.MX RT 1060/1062) 
USB host support to work with the `cotton-usb-host` crate (v0.2.1+). The implementation 
follows the pattern established by the RP2040 host controller implementation in 
`cotton-usb-host::host::rp2040`.


**Document Version**: 4.4
**Date**: 2026-02-18
**Status**: Phase 2b COMPLETE — HID keyboard key presses received correctly on hardware. Beginning Phase 2c.
**Next Step**: Implement `bulk_in_transfer()` and `bulk_out_transfer()` in `src/host.rs`. Milestone: read a sector from a USB flash drive.

## Goals

1. Implement the `HostController` trait from `cotton-usb-host` for i.MX RT 1060/1062
2. Support Teensy 4.1's USB2 host port (secondary 5-pin header port, not the primary USB1 OTG)
3. Enable async operation with RTIC v2 (`thumbv7-backend`)
4. Support Full Speed (12 Mbps) and Low Speed (1.5 Mbps) USB devices
5. Provide control, interrupt, and bulk transfer support
6. Handle `TransferExtras::WithPreamble` for low-speed devices behind full-speed hubs

## Architecture Overview

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
│  Imxrt1062HostController      (this crate)      │
│  UsbShared (ISR ↔ async task)                   │
│  UsbStatics (pipe/QH/qTD pools)                 │
│  Cache coherency layer                          │
├─────────────────────────────────────────────────┤
│  imxrt-ral USB2/USBPHY2       (register access) │
│  EHCI hardware (QH/qTD DMA engine)              │
└─────────────────────────────────────────────────┘
```

## Key Differences: i.MX RT vs RP2040

### Hardware Architecture

| Feature | RP2040 | i.MX RT 1060/1062 |
|---------|--------|-------------------|
| USB Controller | Simple custom USB controller | EHCI-compatible USB OTG controller |
| Buffer Management | 4KB DPRAM, direct access | DMA with endpoint queues (QH/qTD) |
| PHY | Integrated | Separate USBPHY peripheral (USBPHY2) |
| Transfer Engine | Software-managed SIE | Hardware DMA engine with linked descriptor lists |
| Interrupts | Direct buffer/status flags | EHCI-style interrupt registers (USBSTS/USBINTR) |
| Cache | No cache (Cortex-M0+) | 32KB L1 D-cache (Cortex-M7) — coherency required |
| Scheduling | Software per-endpoint | Hardware async schedule (bulk/control) + periodic schedule (interrupt) |
| Pipe Count (RP2040) | 1 control + 15 bulk/interrupt | TBD: 1 control QH + N bulk/interrupt QHs (pool-sized) |

### Register Structure

**RP2040** (`imxrt-ral` is NOT used — RP2040 uses `rp2040-pac`):
- `USBCTRL_REGS`: Main control registers
- `USBCTRL_DPRAM`: Dual-port RAM for buffers and endpoint control
- Simple endpoint buffer control registers

**i.MX RT** (via RAL module copied from `imxrt-usbd`, using `ral-registers` v0.1):
- `ral::usb::Instance`: Main USB OTG controller registers (EHCI-compatible)
  - Key registers: `USBCMD`, `USBSTS`, `USBINTR`, `FRINDEX`, `ASYNCLISTADDR`, `PORTSC1`, `USBMODE`
  - Host-mode fields already defined: `DEVICEADDR::BASEADR`, `ASYNCLISTADDR::ASYBASE`, `PORTSC1::PSPD`, `USBCMD::ASE`/`PSE`, etc.
- `ral::usbphy::Instance`: Separate PHY control (power-down, TX/RX tuning, `CTRL` with soft-reset/clock-gate)
- Clock configuration (`usb_analog`, `ccm`) handled by caller (BSP/board crate)
- Queue Head (QH) and Queue Transfer Descriptor (qTD) structures are **in RAM**, DMA-accessed by hardware

## HostController Trait Reference

The trait we must implement (from `cotton-usb-host` v0.2.1):

```rust
pub trait HostController {
    type InterruptPipe: Stream<Item = InterruptPacket> + Unpin;
    type DeviceDetect: Stream<Item = DeviceStatus>;

    fn device_detect(&self) -> Self::DeviceDetect;
    fn reset_root_port(&self, rst: bool);

    fn control_transfer(
        &self, address: u8, transfer_extras: TransferExtras,
        packet_size: u8, setup: SetupPacket, data_phase: DataPhase<'_>,
    ) -> impl Future<Output = Result<usize, UsbError>>;

    fn bulk_in_transfer(
        &self, address: u8, endpoint: u8, packet_size: u16,
        data: &mut [u8], transfer_type: TransferType, data_toggle: &Cell<bool>,
    ) -> impl Future<Output = Result<usize, UsbError>>;

    fn bulk_out_transfer(
        &self, address: u8, endpoint: u8, packet_size: u16,
        data: &[u8], transfer_type: TransferType, data_toggle: &Cell<bool>,
    ) -> impl Future<Output = Result<usize, UsbError>>;

    fn alloc_interrupt_pipe(
        &self, address: u8, transfer_extras: TransferExtras,
        endpoint: u8, max_packet_size: u16, interval_ms: u8,
    ) -> impl Future<Output = Self::InterruptPipe>;

    fn try_alloc_interrupt_pipe(
        &self, address: u8, transfer_extras: TransferExtras,
        endpoint: u8, max_packet_size: u16, interval_ms: u8,
    ) -> Result<Self::InterruptPipe, UsbError>;
}
```

**Key types we must handle**:
- `TransferExtras` — `Normal` or `WithPreamble` (for low-speed devices behind FS hubs)
- `DataPhase<'a>` — `In(&'a mut [u8])`, `Out(&'a [u8])`, `None`
- `TransferType` — `FixedSize`, `VariableSize` (affects short-packet handling)
- `InterruptPacket` — `address: u8`, `endpoint: u8`, `size: u8`, `data: [u8; 64]`
- `UsbError` — `Stall`, `Timeout`, `Overflow`, `BitStuffError`, `CrcError`, `DataSeqError`, `BufferTooSmall`, `AllPipesInUse`, `ProtocolError`, `TooManyDevices`, `NoSuchEndpoint`
- `UsbSpeed` — `Low1_5`, `Full12`, `High480`

## Implementation Phases

### Phase 1: Foundation and Hardware Initialization ✅ COMPLETE

**Estimated effort**: 2-3 days | **Milestone**: Controller in host mode, no faults  
📄 **[Full details →](phase1_foundation.md)** | 📄 **[Debugging log →](phase1_debugging.md)**

- ✅ Copy RAL module from `imxrt-usbd` (includes all host-mode register fields); document EHCI-to-RAL name mappings
- ✅ Define core data structures: `UsbShared` (ISR ↔ async), `UsbStatics` (pools), `Imxrt1062HostController`
- ✅ Define EHCI DMA structures: `QueueHead` (64-byte aligned) and `TransferDescriptor` (32-byte aligned)
- ✅ Implement 13-step initialization sequence: clocks → PHY reset → host mode → schedule init → interrupts → run
- ✅ Create source files: `src/ehci.rs`, `src/host.rs`, `src/cache.rs`, `src/vcell.rs`, `src/gpt.rs`
- ✅ Device detection confirmed: low-speed keyboard detected (CCS=1, PSPD=1) with external 5V power
- ⚠️ VBUS GPIO (GPIO_EMC_40 → load switch) deferred — registers correct but pin not driving; using external 5V

### Phase 2: Core HostController Trait Implementation

**Estimated effort**: 5-7 days  
📄 **[Full details →](phases/phase2_host_controller_trait.md)**

| Sub-phase | Scope | Milestone |
|-----------|-------|-----------|
| **2a** (3-4 days) | Device detection, port reset, control transfers | ✅ COMPLETE — Full USB enumeration working (VID=045e PID=00db) |
| **2b** (2-3 days) | `alloc_interrupt_pipe()`, `try_alloc_interrupt_pipe()` | ✅ COMPLETE — HID keyboard input received on hardware |
| **2c** (2-3 days) | `bulk_in_transfer()`, `bulk_out_transfer()` | USB flash drive sector read |

- Implement `Imxrt1062DeviceDetect` stream (PORTSC1 monitoring, speed detection)
- Implement `reset_root_port()` with W1C-safe register writes
- Implement control transfer async state machine (qTD chain: setup → data → status)
- Implement bulk transfers with data toggle tracking via `Cell<bool>`
- Implement `Imxrt1062InterruptPipe` stream with RAII cleanup on Drop
- Map EHCI qTD error bits to `UsbError` variants

### Phase 3: Interrupt Handling and Async Support

**Estimated effort**: 2-3 days | **Milestone**: Reliable operation, no corruption  
📄 **[Full details →](phases/phase3_interrupts_and_async.md)**

- Implement `UsbShared::on_irq()` with disable-on-handle / re-enable-on-poll pattern
- Set up waker registration: `device_waker`, `pipe_wakers[N]`, `async_advance_waker`
- Bind to `USB_OTG2` (IRQ #112) via RTIC
- Implement cache management wrappers: `cache_clean()`, `cache_invalidate()`, `cache_clean_invalidate()`
- Establish cache operation call sites for all DMA boundaries

### Phase 4: Testing and Validation

**Estimated effort**: 3-5 days | **Milestone**: All device types working  
📄 **[Full details →](phases/phase4_testing.md)**

- Compile-time validation: struct size/alignment/offset assertions
- 11-step incremental hardware bring-up (clock init → device detect → control → bulk → interrupt → hub)
- Create example applications: `enumerate.rs`, `keyboard.rs`, `mass_storage.rs`
- Document Teensy 4.1 USB2 host port wiring and power requirements

### Phase 5: Documentation and Polish

**Estimated effort**: 1-2 days | **Milestone**: Complete docs and examples  
📄 **[Full details →](phases/phase5_documentation.md)**

- Add `///` doc comments to all public APIs with `# Safety` and `# Panics` sections
- Document initialization order, register alias workarounds, cache coherency requirements
- Update README.md with working usage example
- Consider `defmt` feature flag for debug logging

## Dependencies

### Required Crate Dependencies

| Crate | Version | Purpose |
|-------|---------|---------|
| `cotton-usb-host` | 0.2.1+ | `HostController` trait, `UsbBus`, device class drivers |
| `ral-registers` | 0.1 | `read_reg!`/`write_reg!`/`modify_reg!` macros (used by copied RAL module from `imxrt-usbd`) |
| `imxrt-hal` | local | Clock configuration, GPIO for VBUS control |
| `cortex-m` | 0.7+ | Cache management intrinsics (`SCB::clean_dcache_by_address`, etc.), barriers |
| `rtic` | 2.0 | Async executor, interrupt binding (`thumbv7-backend` feature) |
| `rtic-common` | — | `CriticalSectionWakerRegistration` for ISR ↔ async waker communication |
| `static-cell` | — | `ConstStaticCell` for `'static` lifetime pools |
| `defmt` | 0.3 | Structured debug logging (optional, feature-gated) |
| `log` | 0.4 | Standard logging facade (optional) |

### Build Dependencies (already in Cargo.toml)

| Crate | Purpose |
|-------|---------|
| `imxrt-rt` | Runtime/startup for i.MX RT |
| `teensy4-fcb` | Flash configuration block for Teensy 4.x |
| `teensy4-panic` | Panic handler |
| `board` | BSP for Teensy 4.1 (from imxrt-hal) |

## Resources and References

### Official Documentation

1. **i.MX RT 1060 Reference Manual** (NXP document IMXRT1060RM)
   - Local: [`docs/external/IMXRT1060RM_rev2.pdf`](../external/IMXRT1060RM_rev2.pdf)
   - Chapter 56: USB OTG controller registers, QH/qTD formats, operational model
   - Chapter 57: USBPHY — PHY power, clock gating, soft reset, disconnect detect
   - Chapter 14: CCM — USB PLL (PLL3) configuration, clock gating registers

2. **EHCI Specification** (Intel/Compaq/NEC/Microsoft, Revision 1.0)
   - Local: [`docs/external/ehci-specification-for-usb.pdf`](../external/ehci-specification-for-usb.pdf)
   - Section 2.3: Host Controller Register Space (USBCMD, USBSTS, USBINTR, etc.)
   - Section 3.5: Queue Element Transfer Descriptor (qTD) — 32-byte structure format
   - Section 3.6: Queue Head (QH) — 48-byte structure + overlay area
   - Section 4.1: Initialization — reset, mode, schedule setup
   - Section 4.8: Async Schedule — control/bulk transfer execution
   - Section 4.9: Periodic Schedule — interrupt transfer execution
   - Section 4.10: Async Advance Doorbell — safe QH removal

3. **USB 2.0 Specification**
   - Local: [`docs/external/USB 2.0/usb_20.pdf`](../external/USB%202.0/usb_20.pdf)
   - Chapter 8: Protocol Layer (data toggle, packet formats)
   - Chapter 9: USB Device Framework (standard requests, descriptor types)
   - Chapter 11: Hub Specification (split transactions, TT)

4. **RTIC v2 Book** (Real-Time Interrupt-driven Concurrency)
   - URL: https://rtic.rs/2/book/en/
   - Summary: Official documentation for RTIC v2, the hardware-accelerated Rust RTOS used by this project for async task execution and interrupt management on Cortex-M7. RTIC leverages ARM NVIC hardware for zero-cost priority-based scheduling using the Stack Resource Policy (SRP), providing deadlock-free, race-free resource access with compile-time analysis.
   - **Key sections for this project**:
     - **The `#[app]` attribute** — Defining the RTIC application: `device` PAC argument, `dispatchers` list for software task priority levels
     - **Hardware tasks** — Binding tasks to interrupt vectors with `#[task(binds = InterruptName)]` (used for `USB_OTG2` IRQ handler)
     - **Software tasks & spawn** — `async fn` software tasks, `spawn()` API, dispatcher interrupt assignment (used for USB enumeration task)
     - **Resource usage** — `#[shared]` resources with `lock()` (Mutex trait / SRP ceiling), `#[local]` resources, `#[lock_free]` for same-priority access
     - **App initialization (`#[init]`)** — Returning `(Shared, Local)` resources, `'static` lifetime locals, peripheral access via `cx.device`/`cx.core`
     - **Channels** — `rtic_sync::channel` for inter-task communication (`Sender`/`Receiver`, `make_channel!`)
     - **Delay and Timeout using Monotonics** — `Mono::delay()`, `Mono::timeout_at()`, `select_biased!` for async timeouts
     - **Target Architecture (Cortex-M7)** — BASEPRI-based priority ceiling (ARMv7-M), mapping SRP to NVIC hardware

5. **The Embedded Rust Book**
   - URL: https://docs.rust-embedded.org/book/
   - Summary: Official introductory book for bare-metal embedded Rust on ARM Cortex-M. Covers the foundational concepts underlying this project: `#![no_std]` environments, memory-mapped register access patterns (PAC → HAL → BSP layering), interrupt handling, peripheral singletons, concurrency primitives, and the `embedded-hal` trait ecosystem.
   - **Key sections for this project**:
     - **`no_std` Rust Environment** — `#![no_std]`/`#![no_main]`, `libcore` vs `libstd`, runtime differences for bare-metal targets
     - **Memory Mapped Registers** — PAC/HAL/BSP crate layering model (directly applicable to `imxrt-ral` → `imxrt-hal` → `board` crate architecture); `read()`/`write()`/`modify()` register access patterns
     - **Exceptions & Interrupts** — Cortex-M exception model, `#[exception]`/`#[interrupt]` attributes, NVIC priority configuration, `static mut` safety in handlers
     - **Concurrency** — Critical sections (`cortex_m::interrupt::free`), `Mutex<RefCell<Option<T>>>` pattern for sharing peripherals between main and ISR, atomics, `Send`/`Sync` trait requirements
     - **Singletons & Peripherals** — Ownership model for hardware peripherals (`Peripherals::take()`), ensuring single-instance access, type-state patterns
     - **Portability (`embedded-hal`)** — Trait-based hardware abstraction, HAL crate conventions (`constrain()`/`split()`), driver portability
     - **Tips for embedded C developers** — Volatile access (`read_volatile`/`write_volatile`), `#[repr(C)]`/`#[repr(align)]` for DMA structures, packed types, build system integration

### Code References

> **Local checkouts**: `cotton`, `imxrt-hal`, `teensy-rs`, and `USBHost_t36` are checked out in the
> parent directory (`../`) alongside this project and are also in the VS Code Workspace for this project.

1. **cotton-usb-host RP2040 implementation** (primary reference for trait implementation pattern)
   - Repository: https://github.com/pdh11/cotton
   - Local: `../cotton`
   - File: `cotton-usb-host/src/host/rp2040.rs` (~1625 lines)
   - Key patterns: `UsbShared`/`UsbStatics` split, disable-on-handle ISR, `Pool`-based pipe allocation, `CriticalSectionWakerRegistration` wakers
   - File: `cotton-usb-host/src/host.rs` — `HostController` trait definition

2. **TinyUSB EHCI implementation** (clean, minimal EHCI reference)
   - Repository: https://github.com/hathach/tinyusb
   - Files: `src/portable/ehci/ehci.c`, `src/portable/ehci/ehci.h`
   - Useful for: QH/qTD structure setup, async/periodic schedule management, i.MX RT-specific initialization

3. **imxrt-hal USB device implementation** (i.MX RT register access patterns)
   - Repository: https://github.com/imxrt-rs/imxrt-hal
   - Local: `../imxrt-hal`
   - Shows PHY initialization, clock setup, register naming conventions

4. **imxrt-ral USB device implementation** (i.MX RT register access definitions)
   - Repository: https://github.com/imxrt-rs/imxrt-ral
   - Local: `../imxrt-ral`

6. **Teensy-specific HAL/RAL modifications***
   - Repository: https://github.com/mciantyre/teensy4-rs
   - Local: `../teensy-rs`
   - Teensy-specific hardware definitions that modify `imxrt-hal` and `imxrt-ral`.

7. **Linux EHCI driver** (authoritative, handles all edge cases)
   - Kernel source: `drivers/usb/host/ehci-hcd.c`, `ehci-q.c`, `ehci-sched.c`
   - Most comprehensive EHCI reference; useful for debugging tricky issues

8. **Teensyduino USB host implementation** (C++ reference for same hardware)
   - Repository: https://github.com/PaulStoffregen/USBHost_t36
   - Local: `../USBHost_t36`
   - Directly targets the i.MX RT 1062 (Teensy 4.x) EHCI controller
   - Useful for: i.MX RT-specific initialization quirks, PHY setup, register usage, practical workarounds
   - Handles hub support, multiple device classes (HID, MIDI, serial, mass storage, etc.)

### i.MX RT Register Mapping Quick Reference

The RAL module copied from `imxrt-usbd` provides all needed register and field definitions.
See [imxrt-usbd-reuse-analysis.md](planning/imxrt-usbd-reuse-analysis.md) for full details.

| EHCI Spec Name | RAL Name | Host-mode Field | Notes |
|----------------|----------|-----------------|-------|
| USBCMD | `USBCMD` | `RS`, `PSE`, `ASE`, `IAA`, `FS_1`/`FS_2`, `ITC` | ✅ All host fields present |
| USBSTS | `USBSTS` | `UI`, `PCI`, `AAI`, `HCH`, `RCL`, `PS`, `AS`, `FRI` | ✅ All host fields present |
| USBINTR | `USBINTR` | `UE`, `PCE`, `AAE`, `UAIE`, `UPIE` | ✅ All host fields present |
| FRINDEX | `FRINDEX` | `FRINDEX` | ✅ Direct match |
| PERIODICLISTBASE | `DEVICEADDR` | `BASEADR` (bits [31:12]) | ✅ Host-mode field defined |
| ASYNCLISTADDR | `ASYNCLISTADDR` | `ASYBASE` (bits [31:5]) | ✅ Host-mode field defined |
| PORTSC | `PORTSC1` | `CCS`, `CSC`, `PE`, `PR`, `PP`, `PSPD`, `SUSP`, `FPR` | ✅ Including NXP `PSPD` extension |
| USBMODE | `USBMODE` | `CM` (`CM_3` = host mode) | ✅ Direct match |
| CONFIGFLAG | `CONFIGFLAG` | — | ✅ Present (read-only, always 1 on this controller) |

### Hardware Instances

| Resource | Instance | Base Address | NVIC IRQ |
|----------|----------|-------------|----------|
| USB OTG1 (device/primary) | `usb::USB1` | `0x402E_0000` | 113 (`USB_OTG1`) |
| USB OTG2 (host/secondary) | `usb::USB2` | `0x402E_0200` | 112 (`USB_OTG2`) |
| USBPHY1 | `usbphy::USBPHY1` | `0x400D_9000` | 65 |
| USBPHY2 | `usbphy::USBPHY2` | `0x400D_A000` | 66 |

### Coding Guidelines: Prefer Symbolic RAL Access

**Always use symbolic RAL names** instead of raw memory addresses to avoid off-by-one
or wrong-address bugs. Raw pointer access with hardcoded addresses is error-prone
and has caused debugging time in this project (e.g., using GPIO7 address when GPIO8
was intended).

**Available RAL modules for i.MX RT 1062** (via `imxrt-ral` crate with `imxrt1062` feature):

| Peripheral | RAL Module | Usage |
|------------|------------|-------|
| USB OTG | `ral::usb::{USB1, USB2}` | Host/device controller registers |
| USBPHY | `ral::usbphy::{USBPHY1, USBPHY2}` | PHY control (power, reset) |
| GPIO (standard) | `ral::gpio::{GPIO1..GPIO5}` | Standard GPIO banks |
| GPIO (fast) | `ral::gpio::{GPIO6..GPIO9}` | Fast GPIO banks (GPIO6=GPIO1, GPIO7=GPIO2, **GPIO8=GPIO3**, GPIO9=GPIO4) |
| IOMUXC | `ral::iomuxc::IOMUXC` | Pad mux/config (e.g., `SW_MUX_CTL_PAD_GPIO_EMC_40`) |
| IOMUXC_GPR | `ral::iomuxc_gpr::IOMUXC_GPR` | General purpose registers |
| CCM_ANALOG | `ral::ccm_analog::CCM_ANALOG` | PLL control (e.g., `PLL_USB2`) |

**Example: GPIO8 access (correct)**:
```rust
use imxrt_ral as ral;

let gpio8 = unsafe { ral::gpio::GPIO8::instance() };
ral::modify_reg!(ral::gpio, gpio8, GDIR, |v| v | (1 << 26));
ral::write_reg!(ral::gpio, gpio8, DR_SET, 1 << 26);
```

**Example: IOMUXC access (correct)**:
```rust
let iomuxc = unsafe { ral::iomuxc::IOMUXC::instance() };
ral::write_reg!(ral::iomuxc, iomuxc, SW_MUX_CTL_PAD_GPIO_EMC_40, 5);  // ALT5
ral::write_reg!(ral::iomuxc, iomuxc, SW_PAD_CTL_PAD_GPIO_EMC_40, 0x0008);
```

**When raw access is acceptable**:
- Debug readback of registers owned by a driver (e.g., USB2 registers after host controller consumes the instance)
- Always document why raw access is necessary in a comment

**Finding register names**:
- `imxrt-ral` SVDs: `../imxrt-ral/svd/imxrt1062.svd`
- `imxrt-ral` generated blocks: `../imxrt-ral/src/blocks/imxrt1061/` (1061/1062 share most blocks)
- Reference manual chapters for register descriptions

## Build Tools

### build_example.ps1

A PowerShell helper script at the project root (`build_example.ps1`) builds an example,
converts the ELF to a HEX file, and prints **only error lines** from cargo — keeping
build output short when pasting results into the context window.

**Usage**:
```powershell
# Basic — produces <example>.hex in the current directory
.\build_example.ps1 -Example rtic_usb_enumerate

# Custom output file name
.\build_example.ps1 -Example rtic_usb_enumerate -HexFile usb_enumerate.hex
```

The script:
1. Runs `cargo build --release --target thumbv7em-none-eabihf --example <name>`
2. Filters stderr/stdout, keeping only `error[…]:` blocks (warnings suppressed)
3. On success, runs `rust-objcopy -O ihex` to produce the HEX file
4. Exits non-zero on any failure so CI / manual inspection is straightforward

**Expected output on a clean build** (nothing to paste):
```
Building example 'rtic_usb_enumerate' -> 'rtic_usb_enumerate.hex' ...
Build succeeded. Converting ELF to HEX ...
Done: rtic_usb_enumerate.hex
```

**Expected output on a compiler error** (paste only this into context):
```
Building example 'rtic_usb_enumerate' -> 'rtic_usb_enumerate.hex' ...

BUILD ERRORS:
error[E0308]: mismatched types
  --> src/host.rs:42:5
   |
42 |     return 0u32;
   |     ^^^^^^^^^^^^ expected `()`, found `u32`

Build FAILED for example 'rtic_usb_enumerate'.
```

## Risk Mitigation

### ~~Risk: `PERIODICLISTBASE`/`DEVICEADDR` Register Aliasing~~ ✅ Resolved

**Impact**: ~~Medium~~ **None** — resolved by using `imxrt-usbd`'s RAL module
**Resolution**:
- The `imxrt-usbd` RAL defines `DEVICEADDR::BASEADR` (bits [31:12]) for host-mode access
- `ASYNCLISTADDR::ASYBASE` (bits [31:5]) is also defined for the async schedule pointer
- `PORTSC1::PSPD` (bits [27:26]) is defined with enumerated speed values
- All host-mode `USBCMD`/`USBSTS`/`USBINTR` fields (`ASE`, `PSE`, `HCH`, `AAI`, etc.) are present
- See [imxrt-usbd-reuse-analysis.md](planning/imxrt-usbd-reuse-analysis.md) for the full analysis

### Risk: EHCI Complexity Underestimation

**Impact**: High — could significantly extend timeline
**Mitigation**:
- Build strictly incrementally: device detect → control → interrupt → bulk
- First milestone is `GET_DESCRIPTOR` — proves entire DMA pipeline
- Use simplified periodic schedule initially
- Reference TinyUSB (simple) and Linux (comprehensive) simultaneously
- Accept that first implementation may not handle all error cases perfectly

### Risk: Cache Coherency Bugs (Silent Data Corruption)

**Impact**: Critical — causes intermittent, hard-to-reproduce failures
**Mitigation**:
- Implement cache utilities first and test them in isolation
- Start with defensive over-flushing (flush/invalidate more than strictly necessary)
- Consider DTCM placement for descriptors to eliminate this class of bugs entirely
- Add `defmt` trace logging at every cache operation during development
- Verify with USB protocol analyzer if available
- Known symptom: "works sometimes" or "works at low speed but not high speed"

### Risk: USBNC2 Register Coverage

**Impact**: Low-Medium — may affect OTG2 control features
**Mitigation**:
- Audit USBNC2 registers early (Phase 1.1)
- If the RAL's USBNC2 instance is incorrect, use raw pointer access as fallback
- USBNC is primarily for OTG features (ID pin, VBUS) which may not be needed for dedicated host mode

### Risk: Hardware Availability and Wiring

**Impact**: Low — Teensy 4.1 is readily available
**Mitigation**:
- Primary testing on Teensy 4.1 with USB2 host header
- Document exact wiring: 5-pin header pinout (VBUS/5V, D−, D+, ID, GND)
- Note: Teensy 4.1 USB2 port needs external 5V power supply for VBUS (not powered from main USB)
- Keep a selection of test devices: FS flash drive, LS keyboard, FS hub

## Timeline Estimate

| Phase | Description | Duration | Key Milestone |
|-------|-------------|----------|---------------|
| **Phase 1** | **Foundation and initialization** | **2-3 days** | **✅ Controller in host mode, device detected** |
| **Phase 2a** | **Device detect + port reset + control transfers** | **3-4 days** | **✅ COMPLETE — Full USB enumeration working (VID=045e PID=00db)** |
| **Phase 2b** | **Interrupt pipes** | **2-3 days** | **✅ COMPLETE — HID keyboard reports received** |
| Phase 2c | Bulk transfers | 2-3 days | USB flash drive sector read |
| Phase 3 | Interrupts, async, cache polish | 2-3 days | Reliable operation, no corruption |
| Phase 4 | Testing and examples | 3-5 days | All device types working |
| Phase 5 | Documentation | 1-2 days | Complete docs and examples |
| **Total** | | **15-23 days** | |

This assumes:
- Full-time focused development
- Prior familiarity with Rust embedded and async (RTIC v2)
- Access to Teensy 4.1 with USB2 host header soldered and test USB devices
- Some EHCI knowledge (learning time included in estimates)
- Phase 2a is the critical path — everything else builds on the first successful control transfer

## Success Criteria

1. ✅ `GET_DESCRIPTOR(Device)` to address 0 succeeds (proves QH/qTD/DMA/cache pipeline)
2. ✅ Full device enumeration via `UsbBus::device_events()` completes
3. ✅ Successfully read sectors from a USB mass storage flash drive
4. ✅ Successfully read key events from a USB HID keyboard
5. ✅ Hot-plug and hot-unplug handled cleanly (no panics, no resource leaks)
6. ✅ Both full-speed and low-speed devices work
7. ✅ Device behind a USB hub works (tests `TransferExtras::WithPreamble`)
8. ✅ No cache coherency or memory corruption issues under sustained operation
9. ✅ `cargo doc` generates complete documentation with no warnings
10. ✅ At least one working RTIC example compiles and runs on Teensy 4.1

## Future Enhancements (Beyond Initial Implementation)

1. **High Speed (480 Mbps) support** — requires different QH endpoint characteristics, micro-frame scheduling
2. **Isochronous transfer support** — uses iTD/siTD descriptors (separate from QH/qTD), needed for audio/video
3. **Split transaction support** — for FS/LS devices behind a HS hub (TT scheduling in QH capabilities)
4. **Multiple concurrent device optimization** — better pipe waker granularity, reduce cache operations
5. **Power management** — USB suspend/resume, low-power modes, selective suspend
6. **USB1 controller support** — primary OTG port (same register layout, different instance)
7. **i.MX RT 1050/1064/1170 support** — similar EHCI controllers, may need minor adaptations
8. **Performance optimization** — batch cache operations, reduce ISR latency, optimize QH scanning
9. **Proper periodic schedule tree** — binary scheduling tree for optimal bandwidth allocation
10. **USB hub TT (Transaction Translator) scheduling** — proper split transaction budget management

## Open Questions (To Be Resolved During Implementation)

Phase-specific open questions have been moved to their respective phase documents. The following are cross-cutting questions:

1. **Q**: Who is responsible for USB clock/PLL initialization — this crate or the caller?
   **A**: Follow `imxrt-hal` convention: the caller (BSP/board crate) initializes clocks, this crate takes ownership of already-clocked peripheral instances. Document the prerequisite clearly. **Decision point**: Phase 1.3.

## Notes for Future Maintainers

- The **EHCI specification** (revision 1.0) is essential reading — especially sections 3.5 (qTD), 3.6 (QH), 4.8 (async schedule), 4.9 (periodic schedule)
- **Cache coherency is the #1 source of bugs** — when in doubt, flush/invalidate. See [CACHE_COHERENCY.md](CACHE_COHERENCY.md)
- QH alignment (64-byte) and qTD alignment (32-byte) are enforced by hardware — the controller will silently malfunction if alignment is wrong (it ignores low-order address bits)
- **RAL module** is copied from `imxrt-usbd/src/ral/` (not the upstream `imxrt-ral` crate). It uses `ral-registers` v0.1 for `read_reg!`/`write_reg!`/`modify_reg!` macros. See [imxrt-usbd-reuse-analysis.md](planning/imxrt-usbd-reuse-analysis.md).
- The i.MX RT USB controller has minor quirks vs. standard EHCI:
  - `USBMODE` must be written immediately after controller reset
  - `PERIODICLISTBASE` is accessed via `DEVICEADDR::BASEADR` (host-mode field at same register offset)
  - `ASYNCLISTADDR` uses `ASYBASE` field for host-mode async list pointer
  - `PORTSC` is named `PORTSC1`
  - Port speed is in `PORTSC1::PSPD` bits [27:26] (NXP extension, not in EHCI spec)
- USB2 controller on Teensy 4.1 uses the secondary 5-pin header — different from USB1 (micro-USB connector)
- Test with multiple device types — a flash drive exercises different code paths than a keyboard
- Interrupt endpoint support is harder than bulk — the periodic schedule adds complexity
- The disable-on-handle / re-enable-on-poll interrupt pattern (from RP2040 implementation) prevents IRQ storms and is critical for correct async waker behavior


