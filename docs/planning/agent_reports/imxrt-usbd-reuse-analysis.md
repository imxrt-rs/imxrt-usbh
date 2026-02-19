# Reuse Analysis: `imxrt-usbd` for `imxrt-usbh`

This document examines every module in the `imxrt-usbd` crate (USB device driver) and
evaluates which pieces can be reused, adapted, or serve as reference for the `imxrt-usbh`
(USB host driver) project.

Repository analyzed: `imxrt-usbd` v0.3.0

---

## Summary

| Module | Verdict | Notes |
|--------|---------|-------|
| `ral/usb.rs` | **Copy & extend** | Complete USB register definitions including host-mode fields |
| `ral/usbphy.rs` | **Copy as-is** | PHY registers are identical for host and device |
| `ral.rs` (macros/glue) | **Adapt** | `endpoint_control` helper is device-specific; `Instances` struct is reusable |
| `cache.rs` | **Copy as-is** | Cache maintenance is identical for host and device |
| `vcell.rs` | **Copy as-is** | Generic volatile cell, mode-independent |
| `td.rs` | **Reference only** | Device-mode dTD layout; host uses a different qTD/iTD/siTD format |
| `qh.rs` | **Reference only** | Device-mode dQH layout; host uses a different QH format |
| `buffer.rs` | **Adapt** | Allocator useful, but host needs different memory management |
| `state.rs` | **Reference only** | Device endpoint state model; host has pipes, not endpoints |
| `endpoint.rs` | **Reference only** | Device endpoint abstraction; useful for patterns only |
| `driver.rs` | **Reference only** | Device-mode initialization sequence is informative |
| `bus.rs` | **Reference only** | `usb-device` trait impl; not applicable to host |
| `gpt.rs` | **Copy & adapt** | USB GPT timers are useful for host timeouts |
| `log.rs` | **Copy as-is** | defmt logging macros, mode-independent |
| `lib.rs` | **Reference only** | `Peripherals` trait pattern is worth adopting |

---

## Detailed Analysis

### 1. `src/ral/usb.rs` — USB Core Registers (3381 lines)

**Verdict: Copy and extend**

This is the most valuable file. It contains a complete `ral-registers`-compatible definition
of all USB core registers for the i.MX RT, including both device-mode and host-mode fields.
The `imxrt-usbd` project wrote its own RAL rather than depending on `imxrt-ral`, using the
`ral-registers` crate (v0.1) for the `read_reg!`/`write_reg!`/`modify_reg!` macros.

#### Already includes host-mode register fields

The register definitions already contain host-mode fields that are not present in the
upstream `imxrt-ral`:

| Register | Field | Bits | Host-mode purpose |
|----------|-------|------|-------------------|
| `DEVICEADDR` | `BASEADR` | [31:12] | Periodic frame list base address (host) |
| `ASYNCLISTADDR` | `ASYBASE` | [31:5] | Async schedule list pointer (host) |
| `USBCMD` | `PSE` | [4] | Periodic Schedule Enable (host) |
| `USBCMD` | `ASE` | [5] | Async Schedule Enable (host) |
| `USBCMD` | `IAA` | [6] | Interrupt on Async Advance doorbell (host) |
| `USBCMD` | `ASP` | [9:8] | Async Schedule Park Mode count (host) |
| `USBCMD` | `ASPE` | [11] | Async Schedule Park Mode enable (host) |
| `USBCMD` | `FS_1`, `FS_2` | [3:2], [15] | Frame List Size (host) |
| `USBSTS` | `HCH` | [12] | HC Halted (host) |
| `USBSTS` | `RCL` | [13] | Reclamation (host) |
| `USBSTS` | `PS` | [14] | Periodic Schedule Status (host) |
| `USBSTS` | `AS` | [15] | Async Schedule Status (host) |
| `USBSTS` | `AAI` | [5] | Async Advance Interrupt (host) |
| `USBSTS` | `PCI` | [2] | Port Change Detect (host) |
| `USBSTS` | `FRI` | [3] | Frame List Rollover (host) |
| `USBINTR` | `PCE` | [2] | Port Change Detect enable (host) |
| `USBINTR` | `AAE` | [5] | Async Advance enable (host) |
| `USBINTR` | `UAIE` | [18] | USB Host Async interrupt enable |
| `USBINTR` | `UPIE` | [19] | USB Host Periodic interrupt enable |
| `PORTSC1` | `PSPD` | [27:26] | Port Speed (NXP extension, host) |
| `PORTSC1` | `PP` | [12] | Port Power (host) |
| `PORTSC1` | `PR` | [8] | Port Reset (host) |
| `PORTSC1` | `PE` | [2] | Port Enabled (host) |
| `PORTSC1` | `CCS` | [0] | Current Connect Status (host) |
| `PORTSC1` | `CSC` | [1] | Connect Status Change (host) |
| `PORTSC1` | `PEC` | [3] | Port Enable/Disable Change (host) |
| `PORTSC1` | `OCA` | [4] | Over-current Active (host) |
| `PORTSC1` | `OCC` | [5] | Over-current Change (host) |
| `PORTSC1` | `FPR` | [6] | Force Port Resume (host) |
| `PORTSC1` | `SUSP` | [7] | Suspend (host) |
| `PORTSC1` | `WKCN` | [20] | Wake on Connect (host) |
| `PORTSC1` | `WKDC` | [21] | Wake on Disconnect (host) |
| `PORTSC1` | `WKOC` | [22] | Wake on Over-current (host) |
| `USBMODE` | `CM` | [1:0] | Controller Mode (`CM_3` = host) |
| `OTGSC` | (various) | | OTG status & control fields |

This resolves two items from `imxrt-ral-changes.md`:
- **Issue #1** (`PERIODICLISTBASE`): `DEVICEADDR::BASEADR` already exists
- **Issue #2** (`PSPD`): `PORTSC1::PSPD` already exists with enumerated values

#### What needs to be added for host mode

The `RegisterBlock` struct and field modules cover all the registers needed. However, the
host driver will also need:

- A `CONFIGFLAG` field module (register exists in `RegisterBlock` but no field definitions
  were found — standard EHCI has a `CF` bit at [0])
- Possibly `HCSPARAMS` and `HCCPARAMS` field modules for reading host controller
  capabilities at init time (the registers are in `RegisterBlock` as `RORegister` but
  no field submodules were defined beyond the existing `HWHOST` module)

#### Approach

Copy `usb.rs` into `imxrt-usbh/src/ral/usb.rs`. Remove unused device-only endpoint
registers if desired (though keeping them is harmless). Add any missing host-mode field
definitions. The file already uses `ral-registers` v0.1 — keep the same dependency.

---

### 2. `src/ral/usbphy.rs` — USB PHY Registers (1694 lines)

**Verdict: Copy as-is**

PHY initialization is the same for host and device mode. The `imxrt-usbd` driver does:

```rust
ral::write_reg!(ral::usbphy, self.phy, CTRL_SET, SFTRST: 1);
ral::write_reg!(ral::usbphy, self.phy, CTRL_CLR, SFTRST: 1);
ral::write_reg!(ral::usbphy, self.phy, CTRL_CLR, CLKGATE: 1);
ral::write_reg!(ral::usbphy, self.phy, PWD, 0);
```

The host driver needs exactly the same PHY bring-up sequence. Copy without changes.

---

### 3. `src/ral.rs` — RAL Glue Module

**Verdict: Adapt**

This module provides:

1. **Re-exports of `ral-registers` macros** (`read_reg!`, `write_reg!`, `modify_reg!`,
   `RORegister`, `RWRegister`) — copy as-is.

2. **`endpoint_control` helper** — provides indexed access to `ENDPTCTRL0..7` registers.
   This is device-mode specific (device endpoints), but the pattern of indexed register
   access is useful. The host driver won't use `ENDPTCTRLn` for data transfer, but may
   reference `ENDPTCTRL0` during initialization.

3. **`Instances` struct and `instances()` function** — converts a `Peripherals` impl into
   typed register instances. Directly reusable.

#### Approach

Copy the module. Keep the `Instances`/`instances()` pattern. Remove or simplify the
`endpoint_control` helper since the host doesn't manage device endpoints the same way.

---

### 4. `src/cache.rs` — DCache Maintenance

**Verdict: Copy as-is**

Provides `clean_invalidate_dcache_by_address()` — essential for DMA coherency on the
Cortex-M7 when working with USB data structures (QH, TD, transfer buffers). The host
driver needs this for the same reason: EHCI data structures live in main memory and are
accessed by the USB controller via DMA.

The implementation is self-contained (no dependencies beyond `cortex-m` v0.7) and
mode-independent. Copy without changes.

---

### 5. `src/vcell.rs` — Volatile Cell

**Verdict: Copy as-is**

A `#[repr(transparent)]` wrapper providing volatile read/write on owned memory. Used by
both `Qh` and `Td` to ensure correct access to DMA-visible data structures.

The host driver's QH and TD structures will need the same volatile access pattern. Copy
without changes.

---

### 6. `src/td.rs` — Transfer Descriptors (Device Mode)

**Verdict: Reference only — host uses different TD formats**

The device-mode transfer descriptor (dTD) layout:

```
offset 0x00: NEXT         — pointer to next dTD
offset 0x04: TOKEN        — status, IOC, total bytes
offset 0x08: BUFFER_POINTERS[5]  — 5 buffer page pointers
offset 0x1C: last_transfer_size  — software field (not hardware)
```

The EHCI host controller uses **different** data structures:

| Structure | Used for | Size | Key differences |
|-----------|----------|------|-----------------|
| **qTD** (Queue Element Transfer Descriptor) | Bulk/Control/Interrupt | 32 bytes | Has `CERR` (error counter), `PID` code, `C_Page`, alternate next pointer |
| **iTD** (Isochronous Transfer Descriptor) | High-speed isochronous | 64 bytes | Completely different layout with 8 transaction slots |
| **siTD** (Split Isochronous TD) | Full/low-speed isochronous | 28 bytes | Split transaction fields |

#### What to reuse

- The **pattern** of using `VCell` for DMA-visible fields
- The **pattern** of using RAL-style field modules (`mod TOKEN { ... }`) for bitfield access
- The `clean_invalidate_dcache()` method pattern
- The test patterns for verifying bit manipulation

The actual struct layout and field definitions must be written from scratch per EHCI spec.

---

### 7. `src/qh.rs` — Queue Heads (Device Mode)

**Verdict: Reference only — host QH format is different**

The device-mode Queue Head (dQH) is 64-byte aligned and contains:

```
CAPABILITIES:      max packet len, ZLT, IOS
_current_td_pointer
overlay:           embedded TD for current transfer
setup:             8-byte setup buffer (control endpoints)
```

The EHCI host-mode Queue Head has a substantially different layout:

| Field | Device QH | Host QH |
|-------|-----------|---------|
| Link pointer | implicit (array index) | Explicit horizontal pointer + type bits |
| Endpoint characteristics | max packet len, IOS | device addr, endpoint, speed, max packet len, NAK counter, control/bulk flag |
| Endpoint capabilities | (none) | split transaction fields (hub addr, port, uFrame scheduling) |
| Current TD pointer | yes | yes |
| Overlay area | embedded dTD | embedded qTD (different layout) |
| Setup buffer | 8 bytes | (none — host sends setup, doesn't receive) |

#### What to reuse

- 64-byte alignment requirement (`#[repr(C, align(64))]`)
- `VCell`-based field access pattern
- Cache maintenance pattern
- RAL-style bitfield modules
- Compile-time size assertions (`const _: [(); 1] = ...`)

---

### 8. `src/buffer.rs` — Endpoint Memory Allocator

**Verdict: Adapt the allocator; rethink the buffer**

The allocator is a simple bump-allocator that carves out chunks from a static byte array:

```rust
static EP_MEMORY: EndpointMemory<4096> = EndpointMemory::new();
```

The host driver needs memory allocation for:
- **QH array** — must be aligned and contiguous
- **qTD pool** — individual TDs, 32-byte aligned
- **Transfer buffers** — for data being sent to / received from devices
- **Periodic frame list** — 4KB-aligned array of frame list entries

The bump allocator pattern is useful, but the host has different alignment and sizing
requirements. The `Buffer` struct (volatile read/write + DCache clean/invalidate) is
directly reusable for transfer buffers.

#### What to reuse

- `EndpointMemory<SIZE>` pattern for static allocation with `AtomicBool` taken guard
- `Allocator` bump-allocator logic
- `Buffer` struct for volatile I/O with cache maintenance
- Consider extending with alignment-aware allocation

---

### 9. `src/state.rs` — Endpoint State Management

**Verdict: Reference only — host state model is fundamentally different**

The device driver manages up to 8 bidirectional endpoints (16 total: 8 OUT + 8 IN).
Each endpoint has a QH, TD, and is indexed by address + direction.

The host driver manages **pipes** (connections to device endpoints):
- Multiple devices, each with its own address
- Multiple endpoints per device
- Transfers are queued via QH → qTD linked lists, not single-QH-to-single-TD
- The async schedule is a circular linked list of QHs
- The periodic schedule is a frame list pointing to QHs/iTDs

#### What to reuse

- The `AtomicU32` allocation tracking pattern
- The `UnsafeCell<MaybeUninit<T>>` pattern for deferred initialization
- The allocator/accessor separation (`EndpointAllocator` borrows from `EndpointState`)
- The compile-time size parameterization with const generics

---

### 10. `src/endpoint.rs` — Endpoint Abstraction

**Verdict: Reference only**

Device-mode endpoint logic: priming, stalling, reading/writing buffers, checking
completion, managing ENDPTCTRL registers. None of this maps directly to host-mode
operation, but the code is well-structured and demonstrates good patterns.

#### Useful patterns

- Using `ral::usb::Instance` reference for register access (borrow-based ownership)
- Separating "has the hardware completed?" checks from data transfer
- The setup tripwire protocol in `read_setup()` (reading `USBCMD::SUTW`)
- The `endpoint_control::register()` indexed access pattern

---

### 11. `src/driver.rs` — Internal USB Driver

**Verdict: Reference for initialization sequence**

The device-mode initialization sequence in `initialize()` is directly informative for
the host driver:

```rust
// PHY init
ral::write_reg!(ral::usbphy, self.phy, CTRL_SET, SFTRST: 1);
ral::write_reg!(ral::usbphy, self.phy, CTRL_CLR, SFTRST: 1);
ral::write_reg!(ral::usbphy, self.phy, CTRL_CLR, CLKGATE: 1);
ral::write_reg!(ral::usbphy, self.phy, PWD, 0);

// Controller reset
ral::write_reg!(ral::usb, self.usb, USBCMD, RST: 1);
while ral::read_reg!(ral::usb, self.usb, USBCMD, RST == 1) {}

// Set mode (device uses CM_2; host will use CM_3)
ral::write_reg!(ral::usb, self.usb, USBMODE, CM: CM_2, SLOM: 1);

// Write endpoint/async list base address
ral::write_reg!(ral::usb, self.usb, ASYNCLISTADDR, qh_addr as u32);
```

The host driver's initialization will follow the same pattern but diverge at mode
selection (`CM_3` for host mode) and schedule setup.

---

### 12. `src/bus.rs` — `usb-device` Bus Adapter

**Verdict: Reference only**

Implements the `usb_device::bus::UsbBus` trait. The host driver won't implement this
trait — it'll need its own host-side API or implement a host trait (if one exists in the
ecosystem). The `Mutex<RefCell<Driver>>` + `interrupt::free()` pattern is a common
embedded concurrency approach worth noting.

---

### 13. `src/gpt.rs` — General Purpose Timers

**Verdict: Copy and adapt**

The USB OTG peripheral includes two GPT timers with 1µs resolution and 24-bit counters.
These are invaluable for the host driver:

- **Port reset timing** — USB spec requires specific reset pulse durations
- **Transfer timeouts** — detecting unresponsive devices
- **Debouncing** — connection detect debounce
- **SOF timing** — if manual SOF generation is needed

The `Gpt` struct and its methods (`run`, `stop`, `reset`, `set_load`, `set_mode`,
`is_elapsed`, `clear_elapsed`, `set_interrupt_enabled`) can be copied directly. The only
change needed is that `Gpt::new()` takes `&mut ral::usb::Instance` which is the same
type used in the host driver.

---

### 14. `src/log.rs` — Logging Macros

**Verdict: Copy as-is**

Conditional `defmt` logging macros (`trace!`, `debug!`, `info!`, `warn!`) gated behind
a `defmt-03` feature flag. Mode-independent and directly reusable.

---

### 15. `src/lib.rs` — Crate Root / `Peripherals` Trait

**Verdict: Reference the `Peripherals` pattern**

The `Peripherals` trait pattern is excellent:

```rust
pub unsafe trait Peripherals {
    fn usb(&self) -> *const ();
    fn usbphy(&self) -> *const ();
}
```

This decouples the driver from any specific RAL crate, allowing users to bring their own
register block instances. The host driver should adopt the same pattern, possibly with
the same trait signature so that users can share peripheral ownership setup between device
and host drivers.

---

## Dependencies Comparison

| Dependency | `imxrt-usbd` | `imxrt-usbh` (proposed) |
|------------|-------------|------------------------|
| `ral-registers` | v0.1 — macros for register access | Same |
| `cortex-m` | v0.7 — interrupts, cache maintenance | Same |
| `bitflags` | v2 — TD status flags | Same or similar |
| `usb-device` | v0.3 — device-mode USB trait | **Not needed** |
| `defmt` | v0.3 (optional) — logging | Same |

---

## Recommended Approach

### Phase 1: Copy and adapt the foundation

1. Copy these files directly:
   - `ral/usb.rs` → `imxrt-usbh/src/ral/usb.rs`
   - `ral/usbphy.rs` → `imxrt-usbh/src/ral/usbphy.rs`
   - `cache.rs` → `imxrt-usbh/src/cache.rs`
   - `vcell.rs` → `imxrt-usbh/src/vcell.rs`
   - `log.rs` → `imxrt-usbh/src/log.rs`
   - `gpt.rs` → `imxrt-usbh/src/gpt.rs`

2. Adapt `ral.rs` — keep `Instances`, remove `endpoint_control`, keep re-exports.

3. Adopt the `Peripherals` trait from `lib.rs`.

### Phase 2: Write host-specific modules (using `imxrt-usbd` as patterns)

4. Write host-mode QH (`src/qh.rs`) — follow the `VCell` + RAL-field-module pattern
   from `imxrt-usbd`'s device QH.

5. Write host-mode qTD (`src/qtd.rs`) — follow the TD pattern but with EHCI host fields.

6. Write host-mode state management — follow the `EndpointState` allocator pattern but
   for pipe/QH/TD pools.

7. Write the host driver — follow the `Driver` struct pattern but with host initialization,
   async/periodic schedule management, and port control.

### Phase 3: Consider sharing

If both `imxrt-usbd` and `imxrt-usbh` are used in the same project (OTG scenarios with
one port as host, one as device), consider extracting the shared modules into a common
crate (e.g., `imxrt-usb-common`) to avoid duplicate register definitions.

---

## Impact on `imxrt-ral-changes.md`

The existence of `imxrt-usbd`'s RAL module resolves several concerns:

| Issue | Status |
|-------|--------|
| #1 — `PERIODICLISTBASE` (`BASEADR`) | **Already defined** in `DEVICEADDR` module |
| #2 — `PORTSC1[PSPD]` | **Already defined** with enumerated values |
| #3 — USBNC2 layout | Not covered by `imxrt-usbd` RAL (USBNC not included) |
| #4 — `PORTSC1` naming | Same naming used |

If `imxrt-usbh` copies the `imxrt-usbd` RAL instead of depending on upstream `imxrt-ral`,
issues #1 and #2 require **no upstream changes**. Issue #3 may still need investigation
if USBNC2 registers are required for host-mode OTG2 configuration.
