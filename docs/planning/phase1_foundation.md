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

## 1.2 Data Structures

Following the RP2040 pattern of `UsbShared` / `UsbStatics` / `HostController`:

- [ ] Define `UsbShared` structure (interrupt-handler ↔ async task shared data)
  - `device_waker: CriticalSectionWakerRegistration` — woken on port change (PORTSC CSC)
  - `pipe_wakers: [CriticalSectionWakerRegistration; N]` — woken on transfer completion per-pipe
  - `fn on_irq(&self)` — called from `USB_OTG2` ISR (IRQ #112)
  - Must be `const fn new()` so it can be placed in a `static`
- [ ] Define `UsbStatics` structure (static lifetime, not shared with ISR)
  - `control_pipes: Pool` — Pool of 1 (only one EP0 control transfer at a time)
  - `bulk_pipes: Pool` — Pool of N bulk/interrupt pipe slots
  - `qh_pool: [QueueHead; N]` — Pre-allocated, 64-byte aligned QH storage
  - `qtd_pool: [TransferDescriptor; M]` — Pre-allocated, 32-byte aligned qTD storage
  - `frame_list: [u32; 1024]` — 4KB-aligned periodic frame list (or smaller power-of-2)
  - Must be `const fn new()` for use with `ConstStaticCell`
- [ ] Define `Imxrt1062HostController` structure
  - `shared: &'static UsbShared`
  - `statics: &'static UsbStatics`
  - `usb: imxrt_ral::usb::USB2` — USB OTG2 register block (ownership, not reference)
  - `usbphy: imxrt_ral::usbphy::USBPHY2` — PHY register block (ownership)

### EHCI DMA Structures

Must be `#[repr(C)]` for hardware compatibility:

```rust
/// Queue Head — 48 bytes minimum, must be 64-byte aligned (EHCI 3.6)
/// Contains endpoint characteristics + overlay area for active qTD
#[repr(C, align(64))]
struct QueueHead {
    horizontal_link: u32,       // Next QH pointer (or T-bit for termination)
    endpoint_characteristics: u32, // Address, endpoint, speed, max packet size
    endpoint_capabilities: u32, // Split transaction fields, interrupt schedule mask
    current_qtd: u32,          // Pointer to current qTD being processed
    // Overlay area (copied from qTD when transfer starts):
    next_qtd: u32,
    alt_next_qtd: u32,
    token: u32,                // Status, PID, error count, bytes to transfer
    buffer_ptrs: [u32; 5],     // Buffer page pointers (4KB aligned)
    // Padding to 64 bytes
}

/// Queue Transfer Descriptor — 32 bytes, must be 32-byte aligned (EHCI 3.5)
#[repr(C, align(32))]
struct TransferDescriptor {
    next_qtd: u32,             // Next qTD pointer (or T-bit)
    alt_next_qtd: u32,         // Alternate next qTD (used on short packets)
    token: u32,                // Active bit, PID, error count, bytes to transfer, data toggle
    buffer_ptrs: [u32; 5],     // Buffer page pointers
}
```

### Key Alignment and Cache Considerations

- QH must be 64-byte aligned (EHCI spec requires 32-byte, but 64 avoids cache-line sharing between QHs — each QH gets its own cache line pair)
- qTD must be 32-byte aligned (matches cache line size, prevents false sharing)
- Frame list must be 4KB-aligned (or smaller: 256/512/1024 entries)
- All these structures are DMA-accessed — every read/write by CPU requires cache invalidate/flush

### Challenge: DMA Buffer Management and Lifetimes

**Problem**: EHCI requires linked lists of QH/qTD descriptors in DMA-accessible memory with stable physical addresses. Rust's ownership model makes this tricky.

**Solution**:
- Pre-allocate fixed pools of QH and qTD structures in `UsbStatics` (via `ConstStaticCell`)
- Use index-based pool allocation (similar to RP2040's `async_pool::Pool` with `BitSet`)
- Pool sizes are compile-time constants — no heap allocation
- `Pooled` wrapper type with `Drop` impl returns resources to pool automatically
- Suggested initial pool sizes: **4 QHs, 16 qTDs** (supports 1 control + 3 concurrent bulk/interrupt)
  - Each control transfer uses 1 QH + 2-3 qTDs
  - Each interrupt pipe uses 1 QH + 1 qTD (re-armed)
  - Each bulk transfer uses 1 QH + 1-N qTDs (depending on transfer size)

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
   **A**: TBD — DTCM avoids cache issues but is limited. Start with OCRAM + cache management. Profile and switch if cache bugs are persistent. **Decision point**: Phase 1.2 (data structure placement).

2. **Q**: How many QH/qTD should we pre-allocate?
   **A**: Start with **4 QH, 16 qTD**. Rationale: 1 QH for control (EP0), up to 3 for concurrent bulk/interrupt. Each transfer uses 1-3 qTDs. Scale up if `AllPipesInUse` errors occur frequently. The RP2040 uses 1 control + 15 bulk/interrupt = 16 total pipes.

3. **Q**: ~~How to handle the `PERIODICLISTBASE`/`DEVICEADDR` register alias?~~
   **A**: ✅ **Resolved.** The `imxrt-usbd` RAL already defines `DEVICEADDR::BASEADR` for
   host-mode access. Use `ral::write_reg!(ral::usb, usb, DEVICEADDR, BASEADR: (addr >> 12))`.

4. **Q**: Should we use `cotton-usb-host`'s internal `async_pool::Pool` or implement our own?
   **A**: Check if `async_pool` is re-exported/public. If not, implement a similar pool using `AtomicU32` bitset + `CriticalSectionWakerRegistration`. The pattern is simple (~100 lines). **Decision point**: Phase 1.2.
