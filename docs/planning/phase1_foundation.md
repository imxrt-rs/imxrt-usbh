# Phase 1: Foundation and Hardware Initialization ✅ COMPLETE

**Estimated effort**: 2-3 days  
**Key milestone**: Controller in host mode, no faults  
**Status**: ✅ COMPLETE (2026-02-08) — Device detection confirmed with external 5V power

### Milestone Evidence

With external 5V power supplied to the USB device, the EHCI host controller
successfully detects a low-speed keyboard:

```
>>> DEVICE CONNECTED <<<
    CCS=1 CSC=1 PE=0 PEC=0 PP=1 PR=0 SUSP=0 PSPD=1 (Low (1.5M))
```

Post-connect register state confirms healthy controller operation:
- `USBCMD = 0x00018B15` — RS=1 (running), PSE=1, ITC=1
- `USBSTS = 0x0000408C` — HCH=0 (not halted), PCI=1 (port change), SEI=0 (no errors)
- `PORTSC1 = 0x14001401` — CCS=1, PP=1, PSPD=1 (low-speed)
- `USBMODE = 0x00000003` — CM=3 (host mode)
- `ASYNCLISTADDR = 0x20201200` — valid, 64-byte aligned sentinel QH

**Note**: VBUS GPIO power control via GPIO_EMC_40 is deferred (registers correct
but pin not driving the load switch). Development continues with external 5V.
See [phase1_debugging.md](phase1_debugging.md) for full debugging history.

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

## 1.3 Initialization Sequence ✅ DONE

- [x] Implement `Imxrt1062HostController::init()` method (hardware initialisation)

The `init()` method is separate from `new()` — construction only stores references,
`init()` performs the actual hardware register writes. Implemented in `src/host.rs`.

### Init Sequence (as implemented)

| Step | What | Register(s) | Notes |
|------|------|-------------|-------|
| 1 | PHY soft-reset | `CTRL_SET` (SFTRST), `CTRL_CLR` (SFTRST + CLKGATE) | SET/CLR avoids RMW race |
| 2 | PHY UTMI levels | `CTRL_SET` (ENUTMILEVEL2, ENUTMILEVEL3) | Required for LS through HS hub |
| 3 | PHY power-up | `PWD = 0` | Clears all power-down bits |
| 4 | Controller reset | `USBCMD` RST → spin | RST is self-clearing |
| 5 | Host mode | `USBMODE` CM=0b11 | **Must be immediately after reset** (NXP errata) |
| 6 | Bus config | `SBUSCFG` AHBBRST=0b001 | INCR4 burst (matches USBHost_t36) |
| 7 | Async schedule | `init_sentinel()` → `ASYNCLISTADDR` | Sentinel QH at qh_pool[0] |
| 8 | Periodic schedule | `DEVICEADDR` (frame list addr), `FRINDEX=0` | DEVICEADDR aliases PERIODICLISTBASE |
| 9 | Clear status | `USBINTR=0`, W1C `USBSTS` | Prevent spurious IRQs during init |
| 10 | USBCMD | RS + PSE + ITC(1) + FS(32) + ASP(3) + ASPE | ASE deferred until first pipe added |
| 11 | Port power | `PORTSC1` PP=1 | Root port power enable |
| 12 | PHY disconnect detect | `CTRL_SET` ENHOSTDISCONDETECT | Needed for unplug detection |
| 13 | Interrupts | `USBINTR`: UE, UEE, PCE, SEE, AAE, UAIE, UPIE | GP timer IRQs enabled on demand |

### Design Decisions

6. **ASE deferred**: The async schedule enable (ASE) bit is NOT set during init. Running the async
   schedule with only the sentinel QH wastes bus bandwidth. ASE will be enabled when the first
   endpoint pipe is added in phase 2.

7. **USBCMD written once**: Rather than using modify_reg! to set individual fields (which would
   trigger the controller between writes), USBCMD is written as a single word with all fields
   (RS, PSE, ITC, FS, ASP, ASPE) set atomically.

8. **ENHOSTDISCONDETECT**: The PHY's high-speed disconnect detector is enabled after the controller
   starts. This is needed for the host to detect device unplugging (USBHost_t36 enables it in its
   ISR on connect, but enabling at init is simpler and safe since no device is connected yet).

9. **Prerequisites are caller responsibility**: USB PLL (CCM_ANALOG_PLL_USB2), clock gating
   (CCM_CCGR6), VBUS GPIO (Teensy 4.1 GPIO_EMC_40), and NVIC enable for USB_OTG2 (IRQ #112)
   must be done by the BSP/board crate before calling `init()`.

### Differences from USBHost_t36

- USBHost_t36 sets `ASYNCLISTADDR = 0` initially; we set it to the sentinel QH address immediately.
  Both are valid — USBHost_t36 writes the address later when the first device connects.
- USBHost_t36 enables GP Timer interrupts (TIE0, TIE1) at init; we defer them until timers are
  actually started (reduces spurious timer IRQs).
- USBHost_t36 writes the PLL setup inline; we require the caller to do it (cleaner separation).

**Files modified** (verified compiling with `cargo check --target thumbv7em-none-eabihf`):
- [x] `src/host.rs` — Added `init()` method (~110 lines) to `Imxrt1062HostController`

## Reference Materials

- i.MX RT 1060 Reference Manual (IMXRT1060RM), Chapter 56 (USB), Chapter 57 (USBPHY)
- EHCI Specification sections 2.3 (Host Controller Registers), 4.1 (Initialization)
- NXP errata: USBMODE must be set immediately after controller reset
- Teensy 4.1 schematic for USB2 host port pin configuration and VBUS control

## Open Questions

1. **Q**: Should we use DTCM or regular OCRAM for QH/qTD structures?
   **A**: TBD — Starting with regular RAM + cache management (OCRAM). DTCM avoids cache issues
   but is limited (512 KB shared with stack/data). Will switch to DTCM if cache coherency bugs
   are persistent. No cache maintenance calls needed during init (structures are written before
   DMA starts). **Decision deferred to Phase 2 testing.**

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
