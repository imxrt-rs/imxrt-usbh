# Phase 1: Foundation and Hardware Initialization

**Estimated effort**: 2-3 days  
**Key milestone**: Controller in host mode, no faults

## 1.1 Register Access Setup ✅ DONE

- [x] ~~Audit `imxrt-ral` v0.6.1 USB register definitions~~ → Using `imxrt-usbd` RAL instead
  - **Decision**: Copy the RAL module from `imxrt-usbd/src/ral/` rather than depending on
    upstream `imxrt-ral`. The `imxrt-usbd` RAL is a standalone module using `ral-registers`
    v0.1, with complete host-mode field definitions already present.
    See [imxrt-usbd-reuse-analysis.md](imxrt-usbd-reuse-analysis.md) for full analysis.
  - **Confirmed available** (all host-mode fields):
    - `USBCMD`: `RS`, `PSE`, `ASE`, `IAA`, `ASP`, `ASPE`, `FS_1`/`FS_2`, `ITC`
    - `USBSTS`: `UI`, `UEI`, `PCI`, `FRI`, `SEI`, `AAI`, `HCH`, `RCL`, `PS`, `AS`, `TI0`/`TI1`
    - `USBINTR`: `UE`, `UEE`, `PCE`, `FRE`, `AAE`, `URE`, `UAIE`, `UPIE`, `TIE0`/`TIE1`
    - `DEVICEADDR`: `BASEADR` (host: periodic frame list base), `USBADRA`/`USBADR` (device)
    - `ASYNCLISTADDR`: `ASYBASE` (host: async schedule pointer), `EPBASE` (device)
    - `PORTSC1`: `CCS`, `CSC`, `PE`, `PEC`, `PR`, `PP`, `PSPD`, `SUSP`, `FPR`, `HSP`, `PFSC`, plus wake fields
    - `USBMODE`: `CM` (`CM_3` = host mode), `SLOM`, `ES`, `SDIS`
    - `OTGSC`: full OTG status/control fields
    - GPT timer registers: `GPTIMER0LD`, `GPTIMER0CTRL`, `GPTIMER1LD`, `GPTIMER1CTRL`
  - **Naming difference**: EHCI `PORTSC` is `PORTSC1` in the RAL
- [x] ~~Verify USBPHY2 register coverage~~ → PHY registers fully covered in `imxrt-usbd/src/ral/usbphy.rs`
  - Includes `CTRL_SET`/`CTRL_CLR` (soft-reset, clock-gate) and `PWD` (power-down)
- [ ] Check USBNC2 register layout correctness (OTG2 control register at offset 0x4 from USBNC1)
  - Note: USBNC is **not** covered by the `imxrt-usbd` RAL — may need raw pointer access or `imxrt-ral`
- [x] ~~Define constants for any bit-field values not provided by the RAL~~ → All needed fields are defined
- [x] ~~Document all register name mappings~~ → See Overview.md register mapping table

**Files created/modified** (all verified compiling with `cargo check --target thumbv7em-none-eabihf`):
- [x] `src/lib.rs` — Crate root with `#![no_std]`, `Peripherals` trait, module declarations
- [x] `src/ral.rs` — RAL glue module: re-exports `ral_registers` macros, `Instances` struct, `instances()` converter
- [x] `src/ral/usb.rs` — USB core register definitions (3381 lines, copied from `imxrt-usbd`)
- [x] `src/ral/usbphy.rs` — USB PHY register definitions (1694 lines, copied from `imxrt-usbd`)
- [x] `src/cache.rs` — D-cache clean+invalidate for DMA coherency (copied from `imxrt-usbd`)
- [x] `src/vcell.rs` — `VCell<T>` volatile cell for DMA-visible data structures (copied from `imxrt-usbd`)
- [x] `src/gpt.rs` — USB general purpose timers with host-mode documentation (adapted from `imxrt-usbd`)
- [x] `src/log.rs` — Conditional defmt macros, feature-gated behind `defmt-03` (copied from `imxrt-usbd`)
- [ ] `Cargo.toml` — Updated: `ral-registers = "0.1"`, `cortex-m = "0.7"`, `bitflags = "2"`, optional `defmt-03`

**Not yet created** (deferred to phases 1.2 and 1.3):
- `src/ehci.rs` — EHCI data structures (QH, qTD, frame list) → Phase 1.2
- `src/host.rs` — `Imxrt1062HostController`, `UsbShared`, `UsbStatics` → Phase 1.2
- `src/pool.rs` — Async resource pool → Phase 1.2

## 1.2 Data Structures ✅ DONE

Following the RP2040 pattern of `UsbShared` / `UsbStatics` / `HostController`:

- [x] Define `UsbShared` structure (interrupt-handler ↔ async task shared data)
  - `device_waker: CriticalSectionWakerRegistration` — woken on port change (PORTSC CSC)
  - `pipe_wakers: [CriticalSectionWakerRegistration; NUM_PIPE_WAKERS]` — woken on transfer completion per-pipe (NUM_PIPE_WAKERS = NUM_QH + 1 = 5)
  - `async_advance_waker: CriticalSectionWakerRegistration` — woken on async advance doorbell (QH removal)
  - `fn on_irq(&self, usb: &ral::usb::Instance)` — called from `USB_OTG2` ISR (IRQ #112)
  - `const fn new()` — all wakers initialized empty
  - Implements disable-on-handle / re-enable-on-poll interrupt pattern
  - `unsafe impl Sync` for ISR ↔ task sharing
- [x] Define `UsbStatics` structure (static lifetime, not shared with ISR)
  - `control_pipes: Pool` — Pool of 1 (only one EP0 control transfer at a time)
  - `bulk_pipes: Pool` — Pool of NUM_QH (4) bulk/interrupt pipe slots
  - `qh_pool: [QueueHead; NUM_QH + 1]` — Pre-allocated, 64-byte aligned QH storage (+1 for sentinel)
  - `qtd_pool: [TransferDescriptor; NUM_QTD]` — Pre-allocated, 32-byte aligned qTD storage (16 slots)
  - `frame_list: FrameList` — 4KB-aligned periodic frame list (32 entries)
  - `const fn new()` — all pools empty, structures zeroed
- [x] Define `Imxrt1062HostController` structure
  - `usb: ral::usb::Instance` — USB OTG core registers (owned)
  - `usbphy: ral::usbphy::Instance` — PHY registers (owned)
  - `shared: &'static UsbShared` — ISR-safe shared state
  - `statics: &'static UsbStatics` — resource pools
  - `fn new<P: Peripherals>(peripherals, shared, statics)` — construction from Peripherals trait

### EHCI DMA Structures — Implemented in `src/ehci.rs`

All `#[repr(C)]` with compile-time size/alignment assertions:

- [x] `TransferDescriptor` (qTD) — 32 bytes, `#[repr(C, align(32))]`
  - Fields: `next`, `alt_next`, `token`, `buffer[5]` (all `VCell<u32>`)
  - Helper: `qtd_token(pid, total_bytes, data_toggle, ioc)` builds the token word
  - Helper: `qtd_token_bytes_remaining(token)` extracts remaining bytes
  - Methods: `new()`, `init()`, `is_complete()`, `has_error()`, `bytes_remaining()`
  - Constants: `QTD_TOKEN_ACTIVE`, `QTD_TOKEN_HALTED`, `QTD_TOKEN_IOC`, `QTD_TOKEN_ERROR_MASK`, etc.
- [x] `QueueHead` (QH) — 64 bytes, `#[repr(C, align(64))]`
  - Hardware words 0–11: `horizontal_link`, `characteristics`, `capabilities`, `current_qtd`,
    overlay fields inlined (`overlay_next`, `overlay_alt_next`, `overlay_token`, `overlay_buffer[5]`)
  - Software words 12–15: `attached_qtd`, `attached_buffer`, `sw_flags`, `sw_pid`, `sw_interval_ms`, padding
  - Note: overlay fields are inlined (not an embedded `TransferDescriptor`) to avoid `align(32)` padding
  - Helper: `qh_characteristics(address, endpoint, speed, max_packet_size, is_control, is_head)`
  - Helper: `qh_capabilities(smask, cmask, hub_addr, hub_port, mult)`
  - Methods: `new()`, `init_sentinel()`, `init_endpoint()`, `attach_qtd()`, `link_after()`
- [x] `FrameList` — 4096-byte aligned, 32 entries (configurable via `FRAME_LIST_LEN`)
  - Each entry is `VCell<u32>` — either `LINK_TERMINATE` or a link pointer to a QH
- [x] Link pointer helpers: `link_pointer()`, `link_address()`, `link_is_terminate()`, `link_type::*`
- [x] PID codes: `PID_OUT`, `PID_IN`, `PID_SETUP`
- [x] Speed codes: `SPEED_FULL`, `SPEED_LOW`, `SPEED_HIGH`

### Pool Allocation — Using `cotton-usb-host::async_pool`

- [x] ~~Implement custom pool~~ → Reusing cotton-usb-host's public `Pool`/`Pooled`/`BitSet`
  - `cotton-usb-host` is a path dependency with `default-features = false` (no RP2040 code pulled in)
  - `Pool` provides async `alloc()` and sync `try_alloc()` with RAII `Pooled` return type
  - `Pooled` auto-returns resource to pool on Drop
  - `CriticalSectionWakerRegistration` from `rtic-common = "1"` for ISR-safe waker storage

### Dependencies Added (Cargo.toml)

- `cotton-usb-host = { path = "../cotton/cotton-usb-host", default-features = false }` — Pool, HostController trait
- `rtic-common = "1"` — `CriticalSectionWakerRegistration`
- `critical-section = "1.1"` — critical section primitives

**Files created/modified** (all verified compiling with `cargo check --target thumbv7em-none-eabihf`):
- [x] `src/ehci.rs` — EHCI DMA structures (QH, qTD, FrameList), link pointer helpers, token/characteristic builders (~590 lines)
- [x] `src/host.rs` — `UsbShared`, `UsbStatics`, `Imxrt1062HostController` (~315 lines)
- [x] `src/lib.rs` — Added `pub mod ehci; pub mod host;`
- [x] `Cargo.toml` — Added `cotton-usb-host`, `rtic-common`, `critical-section`

### Design Decisions Made

1. **Overlay inlining**: QH overlay fields are individual `VCell<u32>` fields rather than an embedded
   `TransferDescriptor`, because `TransferDescriptor`'s `align(32)` would insert 16 bytes of padding
   inside the QH (at offset 16, the overlay needs to reach a 32-byte boundary), breaking the 64-byte
   layout.

2. **Pool reuse**: `cotton-usb-host::async_pool::Pool` is public API and works in `no_std` with no
   feature gates. No need to reimplement. `Pool` uses `Cell<BitSet>` + `RefCell<Option<Waker>>`,
   which is `!Send`/`!Sync` — correct for single-core Cortex-M7.

3. **Frame list size**: 32 entries (matching USBHost_t36), providing 32ms of scheduling granularity.
   This is a good balance between memory (128 bytes of useful data, 4096 with alignment) and interrupt
   endpoint scheduling flexibility.

4. **ISR pattern**: `UsbShared::on_irq()` reads USBSTS, W1C-acknowledges, wakes wakers, and masks
   serviced interrupts in USBINTR. NXP-specific bits 18 (UAI) and 19 (UPI) are checked for
   async/periodic completion in addition to standard USBINT (bit 0).

5. **DTCM vs OCRAM**: Deferred. Starting with regular RAM + cache management. Can switch to DTCM
   (non-cached) placement later if cache coherency bugs are persistent (see Open Question 1).

## 1.3 Initialization Sequence

- [ ] Implement `Imxrt1062HostController::new(usb, usbphy, shared, statics)` function

Detailed initialization steps (order matters):

1. **Enable USB clocks** — CCM clock gating for USB OTG2 and USBPHY2
   - Note: May need to be done by the caller (BSP/board crate) before constructing the host controller
   - USB PLL (PLL3, 480 MHz) must be enabled and stable
2. **Reset USBPHY2** — Write `CTRL_SET` to assert `SFTRST`, then `CTRL_CLR` to release `SFTRST` and `CLKGATE`
3. **Power on USBPHY2** — Write `PWD_CLR = 0xFFFF_FFFF` to enable all PHY sections
4. **Reset USB controller** — Set `USBCMD[RST]`, wait for it to self-clear
5. **Set host mode** — Write `USBMODE[CM] = 0b11` (host mode). **Must be done immediately after reset, before any other USBCMD writes** (per NXP errata)
6. **Initialize async schedule** — Set up a dummy/sentinel QH that points to itself (circular list), write its physical address to `ASYNCLISTADDR`
7. **Initialize periodic schedule** — Zero out the frame list (all entries = T-bit/terminate), write frame list base via `DEVICEADDR::BASEADR` field (host-mode alias for `PERIODICLISTBASE`), set frame list size in `USBCMD[FS]`
8. **Configure interrupts** — Write `USBINTR` to enable: Port Change Detect (PCI), USB Interrupt (USBINT), USB Error Interrupt (USBERRINT), Async Advance (AAI)
9. **Unmask NVIC interrupt** — Enable `USB_OTG2` (IRQ #112) in the NVIC
10. **Enable controller** — Set `USBCMD[RS]` (Run/Stop = Run)
11. **Enable port power** — Set `PORTSC1[PP]` (Port Power) if not already set
12. **Enable VBUS** — Board-specific: Teensy 4.1 USB2 host port may need GPIO to enable VBUS supply

## Reference Materials

- i.MX RT 1060 Reference Manual (IMXRT1060RM), Chapter 56 (USB), Chapter 57 (USBPHY)
- EHCI Specification sections 2.3 (Host Controller Registers), 4.1 (Initialization)
- NXP errata: USBMODE must be set immediately after controller reset
- Teensy 4.1 schematic for USB2 host port pin configuration and VBUS control

## Open Questions

1. **Q**: Should we use DTCM or regular OCRAM for QH/qTD structures?
   **A**: TBD — DTCM avoids cache issues but is limited. Start with OCRAM + cache management. Profile and switch if cache bugs are persistent. **Decision point**: Phase 1.3 or later.

2. **Q**: ~~How many QH/qTD should we pre-allocate?~~
   **A**: ✅ **Resolved.** Using **NUM_QH=4 QH + 1 sentinel = 5 QH slots**, **NUM_QTD=16 qTD slots**.
   The sentinel QH is always allocated (async schedule head). 1 QH for control (EP0), up to 3 for
   concurrent bulk/interrupt. Pool sizes can be adjusted by changing constants in `src/host.rs`.

3. **Q**: ~~How to handle the `PERIODICLISTBASE`/`DEVICEADDR` register alias?~~
   **A**: ✅ **Resolved.** The `imxrt-usbd` RAL already defines `DEVICEADDR::BASEADR` for
   host-mode access. Use `ral::write_reg!(ral::usb, usb, DEVICEADDR, BASEADR: (addr >> 12))`.

4. **Q**: ~~Should we use `cotton-usb-host`'s internal `async_pool::Pool` or implement our own?~~
   **A**: ✅ **Resolved.** `Pool`, `Pooled`, and `BitSet` are public API in `cotton-usb-host`.
   Added as a path dependency with `default-features = false` (pulls in only `core`, `critical-section`,
   `futures` — no RP2040-specific code). No need to reimplement.
