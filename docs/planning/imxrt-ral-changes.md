# Proposed `imxrt-ral` Changes for USB Host Mode

This document tracks register definition gaps and inaccuracies in `imxrt-ral` v0.6.1
(feature `imxrt1062`) that affect USB host controller development, along with proposed
upstream changes.

Repository: https://github.com/imxrt-rs/imxrt-ral

---

## 1. ~~Add `PERIODICLISTBASE` Host-Mode Alias for `DEVICEADDR`~~ ✅ Resolved

**Severity**: ~~High~~ **Low** — the host-mode `BASEADR` field already exists  
**Affected register**: USB offset `0x154`  
**Current state**: **Resolved.** The `imxrt-usbd` crate's own RAL module
(`imxrt-usbd/src/ral/usb.rs`, line 1519) already defines `DEVICEADDR::BASEADR`
(bits [31:12]) alongside the device-mode `USBADRA`/`USBADR` fields.

### What `imxrt-usbd` already provides

The `DEVICEADDR` module in `imxrt-usbd/src/ral/usb.rs` defines three fields:

| Field | Bits | Mask | Mode |
|-------|------|------|------|
| `USBADRA` | [24] | `1 << 24` | Device — address advance |
| `USBADR` | [31:25] | `0x7f << 25` | Device — 7-bit device address |
| `BASEADR` | [31:12] | `0xfffff << 12` | **Host** — periodic frame list base (4KB-aligned) |

The `RegisterBlock` struct field and doc comment acknowledge the dual-purpose nature:
```
/// DEVICEADDR and PERIODICLISTBASE
/// DEVICEADDR: Device Address
/// PERIODICLISTBASE: Frame List Base Address
pub DEVICEADDR: RWRegister<u32>,
```

The `ASYNCLISTADDR` register follows the same pattern, with `ASYBASE` (host: async list
pointer, bits [31:5]) and `EPBASE` (device: endpoint list pointer, bits [31:11]).

### Approach for `imxrt-usbh`

Since `imxrt-usbh` will copy the `imxrt-usbd` RAL module (see
[imxrt-usbd-reuse-analysis.md](imxrt-usbd-reuse-analysis.md)), the `BASEADR` field is
available directly. No workaround or upstream change is needed:

```rust
/// Set the periodic frame list base address (host mode).
fn set_periodic_list_base(usb: &ral::usb::Instance, addr: u32) {
    debug_assert!(addr & 0xFFF == 0, "frame list must be 4KB-aligned");
    ral::write_reg!(ral::usb, usb, DEVICEADDR, BASEADR: (addr >> 12));
}
```

### Upstream change (nice-to-have)

A `pub mod PERIODICLISTBASE` alias in upstream `imxrt-ral` would improve ergonomics
but is not blocking.

---

## 2. ~~Add `PSPD` Field Definition to `PORTSC1`~~ ✅ Resolved

**Severity**: ~~Medium~~ **None** — already fully defined  
**Affected register**: `PORTSC1`  
**Current state**: **Resolved.** The `imxrt-usbd` RAL already defines `PORTSC1::PSPD`
at bits [27:26] with enumerated values.

### What `imxrt-usbd` already provides

The `PORTSC1` module in `imxrt-usbd/src/ral/usb.rs` (line ~2136) defines:

```rust
pub mod PSPD {
    pub const offset: u32 = 26;
    pub const mask: u32 = 0b11 << offset;
    pub mod RW {
        pub const PSPD_0: u32 = 0b00;  // Full Speed
        pub const PSPD_1: u32 = 0b01;  // Low Speed
        pub const PSPD_2: u32 = 0b10;  // High Speed
        pub const PSPD_3: u32 = 0b11;  // Undefined
    }
}
```

The doc comment reads: "Port Speed - Read Only. This register field indicates the speed
at which the port is operating."

### Approach for `imxrt-usbh`

With the copied RAL, speed detection is straightforward:

```rust
let speed = ral::read_reg!(ral::usb, usb, PORTSC1, PSPD);
match speed {
    PORTSC1::PSPD::RW::PSPD_0 => Speed::Full,
    PORTSC1::PSPD::RW::PSPD_1 => Speed::Low,
    PORTSC1::PSPD::RW::PSPD_2 => Speed::High,
    _ => unreachable!(),
}
```

No workaround or upstream change is needed.

---

## 3. Verify/Fix USBNC2 Register Instance Layout

**Severity**: Low-Medium — may affect OTG2 control features  
**Affected peripheral**: `USBNC` (USB OTG Non-Core registers)  
**Current state**: Needs audit — the USBNC2 instance offset may be incorrect

### Problem

The i.MX RT 1062 has two USB Non-Core register blocks:
- **USBNC1** at base `0x402E_0800` (for USB OTG1)
- **USBNC2** at offset `+0x04` from USBNC1, i.e., `0x402E_0804` (for USB OTG2)

These registers control OTG-specific features (over-current detection, VBUS power,
ID pin configuration, UTMI+ interface settings). The `imxrt-ral` may not correctly
expose the USBNC2 instance, or its register offsets may be based on USBNC1's layout.

### Key USBNC Registers

| Register | Offset (from USBNC base) | Purpose |
|----------|--------------------------|---------|
| `USB_OTGn_CTRL` | `0x00` / `0x04` | Over-current, VBUS, WKUP enables |
| `USB_OTGn_PHY_CTRL_0` | `0x18` / `0x1C` | UTMI+ level shifters, low-power mode |

### Proposed Change

Verify that the USBNC2 register instance in the RAL correctly points to the USB OTG2
non-core registers. If not, either fix the base address/offsets or add a second instance.

### Workaround

If the RAL's USBNC2 instance is incorrect, use raw pointer access:

```rust
const USBNC2_CTRL: *mut u32 = 0x402E_0804 as *mut u32;
unsafe { core::ptr::write_volatile(USBNC2_CTRL, value); }
```

### Notes

USBNC is primarily for OTG features (ID pin sensing, VBUS power switching, over-current
detection). For dedicated host mode with external VBUS power, USBNC configuration may
not be strictly required. Audit during Phase 1.1 to determine if any USBNC2 registers
are needed.

---

## 4. `PORTSC` Naming Convention (`PORTSC1`)

**Severity**: Informational — no functional impact, just naming  
**Affected register**: `PORTSC1`

### Observation

The EHCI specification names this register `PORTSC` (Port Status and Control). The
`imxrt-ral` names it `PORTSC1`, presumably because the i.MX RT reference manual uses
the `1` suffix (since there's only one port per controller). This isn't a bug — just
a naming convention difference that should be documented in driver code so future
maintainers can map between EHCI spec references and RAL field names.

No upstream change needed.

---

## Summary

| # | Issue | Severity | Blocks | Status |
|---|-------|----------|--------|--------|
| 1 | `PERIODICLISTBASE` alias missing | ~~High~~ Low | ~~Interrupt pipes (Phase 2c)~~ | ✅ `DEVICEADDR::BASEADR` exists in `imxrt-usbd` RAL |
| 2 | `PORTSC1[PSPD]` field definition | ~~Medium~~ None | ~~Speed detection (Phase 2a)~~ | ✅ `PORTSC1::PSPD` exists in `imxrt-usbd` RAL |
| 3 | USBNC2 instance correctness | Low-Medium | OTG2 control (Phase 1.1) | ⬜ Still needs audit (not covered by `imxrt-usbd` RAL) |
| 4 | `PORTSC` → `PORTSC1` naming | Info | Nothing | ℹ️ No change needed — just document |

### RAL Strategy

Rather than depending on upstream `imxrt-ral`, `imxrt-usbh` will copy the RAL module
from `imxrt-usbd/src/ral/` (see [imxrt-usbd-reuse-analysis.md](imxrt-usbd-reuse-analysis.md)).
This approach:

- **Resolves issues #1 and #2** immediately — the `imxrt-usbd` RAL already has all
  needed host-mode fields (`BASEADR`, `PSPD`, `ASE`, `PSE`, `HCH`, `RCL`, etc.)
- **Uses `ral-registers` v0.1** for the `read_reg!`/`write_reg!`/`modify_reg!` macros
  instead of depending on the full `imxrt-ral` crate
- **Is self-contained** — no upstream dependency to track or contribute to
- **Issue #3** (USBNC2) is not covered by `imxrt-usbd`'s RAL and still needs investigation

Upstream `imxrt-ral` fixes remain nice-to-have for the broader ecosystem but are not
blocking for this project.
