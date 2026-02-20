# Phase 2.5: Cleanup Before Phase 3

**Date**: 2026-02-19
**Status**: In progress

Cleanup tasks to address before moving on to Phase 3 (robustness, cache audit,
polish). These are known issues, deferred fixes, and debug artifacts accumulated
during Phases 1–2c.

---

## C1: VBUS GPIO — Load Switch Not Driving

**Priority**: Medium
**Status**: Fixed
**Origin**: Phase 1 (deferred since initial hardware bring-up)
**Test example**: `rtic_usb_hid_keyboard` (known-working with USBHost_t36 on
this board + keyboard combo)

### Problem

The Teensy 4.1 USB2 host port has an on-board load switch that gates 5V VBUS
power to the USB connector. The enable input is connected to `GPIO_EMC_40`
(ALT5 = GPIO3_IO26 / fast GPIO8_IO26). Our code configures the pad mux, sets
the GPIO direction to output, and drives it HIGH — but the USB device does not
receive power.

Testing with external 5V power confirms all USB functionality works. The
load switch is the only thing not working.

**Key constraint**: USBHost_t36 **works on this exact board with this exact
keyboard**. This rules out board revision differences, wiring issues, and
hardware faults. The problem is purely a software/register configuration
difference between our code and USBHost_t36's initialization sequence.

### USBHost_t36 Reference Code

From `ehci.cpp` lines 208–213:
```c
IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_40 = 5;
IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_40 = 0x0008;  // slow speed, weak 150 ohm
GPIO8_GDIR |= 1<<26;
GPIO8_DR_SET = 1<<26;
```

Four register writes. No IOMUXC_GPR modification.

### Current Code (after GPR27 removal)

```rust
ral::write_reg!(ral::iomuxc, iomuxc, SW_MUX_CTL_PAD_GPIO_EMC_40, 5);
ral::write_reg!(ral::iomuxc, iomuxc, SW_PAD_CTL_PAD_GPIO_EMC_40, 0x0008);
ral::modify_reg!(ral::gpio, gpio8, GDIR, |v| v | (1 << 26));
ral::write_reg!(ral::gpio, gpio8, DR_SET, 1 << 26);
```

Register readback confirms all writes take effect. The pin still doesn't
drive the load switch.

### Analysis — What's Been Tried

| Attempt | Result |
|---------|--------|
| Original code with GPR27[26] set | ❌ No power |
| Removed GPR27 write (match USBHost_t36) | ❌ No power |

### Analysis — GPR Register Investigation

The original code set `IOMUXC_GPR::GPR27[26]`, but this was the **wrong
register**. The fast GPIO mapping for GPIO3 ↔ GPIO8 is controlled by GPR28:

| GPR Register | Controls |
|-------------|----------|
| GPR26 | GPIO1 ↔ GPIO6 |
| GPR27 | GPIO2 ↔ GPIO7 |
| **GPR28** | **GPIO3 ↔ GPIO8** |
| GPR29 | GPIO4 ↔ GPIO9 |

However, USBHost_t36 doesn't set ANY GPR register and still works. For GPIO
output operations, both GPIO3 and GPIO8 can drive the same pin without GPR
configuration — the GPR registers only affect input reads. So the GPR write
was unnecessary and its removal didn't change behavior.

### Remaining Hypotheses

Since our four register writes are identical values to USBHost_t36's four
register writes, the difference must be in the **environment** — something
the Teensy Arduino core sets up before `USBHost::begin()` that our bare-metal
`imxrt-rt` startup does not:

#### E1: Clock gating for GPIO8

The Teensy Arduino core's `startup.c` may enable clock gates for GPIO
peripherals (via CCM_CCGR registers) that our runtime doesn't. If GPIO8's
clock is not enabled, register writes would appear to succeed (readback from
the bus works) but the GPIO output driver wouldn't actually toggle the pin.

**Check**: Read `CCM_CCGR` registers that gate GPIO clocks and compare with
Teensy's startup values.

#### E2: Teensy startup configures GPIO_EMC_40 differently

The Teensy Arduino core may pre-configure GPIO_EMC_40 during `startup.c`
(e.g., as part of SDRAM init since EMC pins are used for external memory).
If the startup code sets a different pad mux (ALT0 = SEMC) and our ALT5
write doesn't take effect for some reason, the pin wouldn't be in GPIO mode.

**Check**: Read back the MUX register after our write to confirm ALT5 is set.
(Already added to the keyboard example's `enable_vbus_power()`.)

#### E3: GPIO register base address mismatch

If our RAL maps GPIO8 to a different address than the Teensy's `GPIO8_GDIR`
macro, our writes would go to the wrong hardware. RAL says GPIO8 base =
`0x4200_8000`. Need to verify the Teensy core uses the same address.

**Check**: Read GPIO8 GDIR and DR after our writes and confirm bit 26 is set
in both.

#### E4: The pin needs a different pad configuration

The pad config `0x0008` sets minimal drive strength (150Ω, slow slew). If
the load switch enable input requires a stronger drive, the GPIO output
voltage might be marginal. This seems unlikely since USBHost_t36 uses the
same value, but the Teensy startup might set additional pad properties.

**Check**: Try `0x10B0` (strong drive, fast slew, pull-up keeper enabled).

#### E5: SEMC/SDRAM controller is claiming the pin

GPIO_EMC_40 is an SEMC (Smart External Memory Controller) pin. If the Teensy
4.1's SDRAM initialization claims this pin for memory interface use, our
ALT5 mux write might be overridden or the pad driver might be in a conflict
state. The Teensy Arduino core's `startup.c` initializes SDRAM on Teensy 4.1.

If `imxrt-rt` or the `board` crate also initializes SDRAM, the SEMC controller
might hold ownership of the pin and our IOMUXC write might not fully release
it. Or the SEMC might re-claim it after our write.

**Check**: Read SEMC registers to see if the controller is active and using
EMC pins.

### Requested Debug Output from USBHost_t36

To compare environments, it would be very helpful to add temporary debug
prints to the working USBHost_t36 Arduino sketch right after `myusb.begin()`
returns. The following register dumps would let us directly compare the
hardware state:

```cpp
// Add this after myusb.begin() in the Arduino sketch:
Serial.printf("GPIO8_DR    = 0x%08X\n", GPIO8_DR);
Serial.printf("GPIO8_GDIR  = 0x%08X\n", GPIO8_GDIR);
Serial.printf("IOMUXC MUX  = 0x%08X\n", IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_40);
Serial.printf("IOMUXC PAD  = 0x%08X\n", IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_40);
Serial.printf("CCM_CCGR0   = 0x%08X\n", CCM_CCGR0);
Serial.printf("CCM_CCGR1   = 0x%08X\n", CCM_CCGR1);
Serial.printf("CCM_CCGR2   = 0x%08X\n", CCM_CCGR2);
Serial.printf("CCM_CCGR3   = 0x%08X\n", CCM_CCGR3);
Serial.printf("GPR26       = 0x%08X\n", IOMUXC_GPR_GPR26);
Serial.printf("GPR27       = 0x%08X\n", IOMUXC_GPR_GPR27);
Serial.printf("GPR28       = 0x%08X\n", IOMUXC_GPR_GPR28);
Serial.printf("GPR29       = 0x%08X\n", IOMUXC_GPR_GPR29);
```

We'll add the same register dump to our keyboard example so the two outputs
can be compared side-by-side.

#### USBHost_t36 regsiter dump
```
GPIO8_DR    = 0x0DD5D008
GPIO8_GDIR  = 0x04000000
IOMUXC MUX  = 0x00000005
IOMUXC PAD  = 0x00000008
CCM_CCGR0   = 0xC0C00FFF
CCM_CCGR1   = 0xFCFFF300
CCM_CCGR2   = 0x0C3FF033
CCM_CCGR3   = 0xF00FF300
GPR26       = 0xFFFFFFFF
GPR27       = 0xFFFFFFFF
GPR28       = 0xFFFFFFFF
GPR29       = 0xFFFFFFFF
```

#### rtic_usb_hid_keyboard register dump
```
[INFO rtic_usb_hid_keyboard::app]: USB2 PLL locked
[INFO rtic_usb_hid_keyboard::app]: VBUS GPIO: MUX=0x00000005 PAD=0x00000008 GDIR=0x04000000 DR=0x0DD5D008
[INFO rtic_usb_hid_keyboard::app]: VBUS power enabled
```

### Resolution

The GPIO and IOMUXC registers are identical. But look at GPR26–29: all 0xFFFFFFFF in USBHost_t36. The Teensy Arduino core's startup.c sets these during boot, routing ALL pins to fast GPIO (GPIO6–9). Our bare-metal runtime doesn't do this.

This is the root cause. With GPR28[26] = 0 (our default), GPIO_EMC_40 is driven by GPIO3, not GPIO8. Our writes to
GPIO8_DR_SET set bit 26 in GPIO8's register file (which is why readback looks correct), but the actual pin output follows
GPIO3_DR[26], which is 0. The pin never goes high.

The fix: set GPR28[26] = 1 before writing to GPIO8. We had the right idea originally but used the wrong GPR register (GPR27
instead of GPR28).


---

## C2: Double Enumeration (FS→HS Transition)

**Priority**: Medium
**Origin**: Phase 2c
**Resolution**: Don't fix (option 3)

### Problem

When a High Speed USB device is plugged in, it is enumerated **twice**:

1. **First enumeration** — device initially presents Full Speed signaling
   (standard USB 2.0 behavior — HS devices always start with FS signaling
   before chirp). PORTSC1 shows `CCS=1, PE=0, PSPD=0` (Full Speed, port
   not yet enabled). cotton-usb-host's `device_detect()` yields
   `Present(Full12)`, triggering a full enumeration cycle at FS. The port
   reset then completes the FS→HS chirp negotiation.

2. **Second enumeration** — after chirp, PORTSC1 changes to `CCS=1, PE=1,
   HSP=1, PSPD=2` (High Speed, port enabled). `device_detect()` yields
   `Present(High480)`, triggering a second full enumeration cycle at HS.

Both enumerations succeed (the device responds correctly at both speeds),
but the first one is wasted work. Observed in logs:

```
DeviceDetect: status change  PORTSC1=0x10001803   ← FS, PE=0
DeviceEvent::Connect  addr=1  VID=1908 PID=1320   ← first enum
...
DeviceDetect: status change  PORTSC1=0x18001207   ← HS, PE=1
DeviceEvent::Connect  addr=1  VID=1908 PID=1320   ← second enum (same device)
```

### Analysis

This is partly a cotton-usb-host behavior: it reacts to every
`DeviceStatus::Present(speed)` event by starting enumeration. On EHCI, the
initial connect (CCS=1) fires before port reset/chirp, so the first speed
report is FS even for HS devices.

The RP2040 reference implementation doesn't have this issue because RP2040
is Full Speed only — there's no chirp/speed negotiation.

### Possible Fixes

1. **Suppress pre-reset device_detect events** — in our `DeviceDetect`
   stream implementation, only yield `Present` when `PE=1` (port enabled,
   meaning reset is complete and speed is final). This is the cleanest fix
   and matches EHCI semantics: the port speed in PSPD is only valid after
   port enable.

2. **Debounce in device_detect** — after yielding a `Present`, ignore
   further port status changes for a short window (e.g., 200ms) to let the
   chirp/reset settle.

3. **Accept it** — the double enumeration is harmless (both succeed, the
   second one is at the correct HS speed). The overhead is ~10ms of extra
   control transfers. This may be acceptable for now.

### Recommendation

Option 1 is the best long-term fix. However, it needs careful testing to
ensure we don't break the initial connect detection. The current code fires
on `CSC` (Connect Status Change) regardless of `PE`. The fix would be to
check `PE=1` before reporting `Present`, or to defer the `Present` report
until after `reset_root_port()` completes.

### Impact If Not Fixed

Low — cosmetic issue. Both enumerations succeed, and the device works
correctly on the second pass. The only cost is ~10ms of extra enumeration
time and a few extra log messages.

---

## C3: Remove Debug Diagnostic Logging from host.rs

**Priority**: Low
**Status**: Fixed
**Origin**: Phase 2c debugging

### Problem

During bulk transfer debugging, enhanced error diagnostics were added to
`do_bulk_transfer()` in `host.rs`. These read back the raw qTD token, buffer
pointer, QH characteristics, and overlay token on every error. While useful
for debugging, this adds code size and unnecessary cache operations in normal
operation.

### Current Code (host.rs, in `do_bulk_transfer` error path)

```rust
if let Err(ref e) = result {
    cache::clean_invalidate_dcache_by_address(...);  // qTD
    cache::clean_invalidate_dcache_by_address(...);  // QH
    let raw_token = ...;
    let buf0 = ...;
    let qh_char = ...;
    let qh_overlay_tok = ...;
    debug!("[HC] bulk {} ... token=0x{:08X} buf0=0x{:08X} char=0x{:08X} overlay=0x{:08X}", ...);
}
```

### Fix

Replace with a concise error log matching the style of `control_transfer`:
```rust
if let Err(ref e) = result {
    debug!("[HC] bulk {} addr={} ep={} len={} -> Err({})",
        if is_in { "IN" } else { "OUT" },
        address, endpoint, data_len,
        Self::usb_error_str(e));
}
```

Remove the extra cache operations and raw register reads from the error path.

---

## C4: Clean Up Mass Storage Example

**Priority**: Low
**Status**: Fixed
**Origin**: Phase 2c debugging

### Items to Clean Up

1. **Remove heartbeat log message** — the `[heartbeat] tick #N` log message
   in the PIT ISR was added for logging debugging. Keep the LED blink and
   the `poller.poll()` call (both are useful), but remove the counter and
   the `log::info!` line.

2. **Keep `log::set_max_level(Debug)`** — this is a legitimate fix, not a
   debug artifact. TRACE-level messages from the host controller overflow
   the 1024-byte log buffer during rapid transfer sequences. All examples
   that use USB host should set this.

3. **Apply `set_max_level` to other examples** — the enumerate and keyboard
   examples don't have this yet. They worked without it because they have
   natural pauses between transfers, but they should have it for robustness.

4. **Keep `usb_err()` helper** — this is useful for production error
   reporting since `UsbError` doesn't implement `Debug`.

---

## C5: Set ENHOSTDISCONDETECT After HS Connection

**Priority**: Medium
**Status**: Fixed
**Origin**: Phase 2c (removed from init, needs proper placement)

### Problem

The PHY's `ENHOSTDISCONDETECT` bit enables the high-speed disconnect
detector. It was originally set during `init()` but this caused false
disconnect events and interfered with FS→HS chirp negotiation. It was
removed entirely as a fix.

Per the i.MX RT reference manual:
> Do not set this bit when there is no device connected. It may cause a
> false disconnect event.

Per USBHost_t36 (`ehci.cpp` line 405):
```c
if (USBHS_PORTSC1 & USBHS_PORTSC_HSP) {
    USBPHY_CTRL_SET = USBPHY_CTRL_ENHOSTDISCONDETECT;
}
```

### What Should Happen

1. **Set** `ENHOSTDISCONDETECT` when PORTSC1 shows `HSP=1` (High Speed Port
   bit, indicating a HS device is connected and the port is enabled).
2. **Clear** `ENHOSTDISCONDETECT` when the device disconnects.

### Current State

The bit is never set. HS devices work fine without it in testing, but
without the disconnect detector, the controller may be slower to detect
device removal at high speed, or may not detect certain HS signaling
errors that indicate a disconnection.

### Recommended Implementation

Add logic to `DeviceDetect::poll()` or the port status change handler:
- On port change with `PE=1, HSP=1`: set `ENHOSTDISCONDETECT`
- On port change with `CCS=0` (disconnect): clear `ENHOSTDISCONDETECT`

This requires access to the USBPHY instance from the DeviceDetect stream,
which currently only holds a USB register pointer. May need to store
the USBPHY base address as well.

---

## C6: recv_bufs Cache Line Alignment

**Priority**: Medium-High
**Status**: Fixed
**Origin**: Identified during logging debugging (D7 in usb_logging_debugging.md)

### Problem

The `recv_bufs` field in `UsbStatics` is declared as:
```rust
pub recv_bufs: [[u8; 64]; NUM_QH - 1],
```

This has alignment 1 (byte-aligned). When the linker places `UsbStatics` in
memory, `recv_bufs` may share a 32-byte cache line with adjacent fields or
other statics. Cache operations on `recv_bufs` (or on adjacent structures)
could corrupt neighboring data.

All other DMA structures are properly aligned:
- `QueueHead`: 64-byte aligned (EHCI requirement)
- `TransferDescriptor`: 32-byte aligned (EHCI requirement)
- `FrameList`: 4096-byte aligned

### Fix

Wrap `recv_bufs` entries in a 32-byte aligned struct:
```rust
#[repr(C, align(32))]
pub struct RecvBuf([u8; 64]);
```

This ensures each receive buffer starts on a cache line boundary and
occupies exactly 2 cache lines (64 bytes), preventing cross-contamination.

### Impact If Not Fixed

Potential silent data corruption if a `recv_buf` shares a cache line with
another structure. This is the kind of bug that causes intermittent,
hard-to-reproduce failures — exactly the class of issue Phase 3's cache
audit is meant to catch. Fixing it now is defensive and low-risk.

---

## C7: Per-Pipe Waker Granularity

**Priority**: Low
**Origin**: TODO comment in host.rs:148

### Problem

The ISR currently wakes **all** pipe wakers on any USB transfer completion
interrupt:
```rust
// TODO(phase 2): Use a per-pipe completion bitmap to wake only
// the relevant pipe waker, avoiding unnecessary wakeups.
for waker in &self.pipe_wakers {
    waker.wake();
}
```

This causes unnecessary wakeups — every pipe's poll function runs and checks
its QH/qTD status, even if only one pipe completed a transfer.

### Impact If Not Fixed

Low — the wakeup is cheap (just checks a token word) and there are typically
only 1–2 active pipes. This is a performance optimization, not a correctness
issue. Could matter more with many concurrent devices.

### Recommendation

Defer to Phase 3 or later. The current approach is correct, just not optimal.

---

## Suggested Order

| Task | Priority | Effort | Status |
|------|----------|--------|--------|
| C3: Remove debug diagnostics from host.rs | Low | 5 min | ✅ Fixed |
| C4: Clean up mass storage example | Low | 10 min | ✅ Fixed |
| C6: recv_bufs cache line alignment | Med-High | 15 min | ✅ Fixed |
| C5: ENHOSTDISCONDETECT after HS connect | Medium | 30 min | ✅ Fixed |
| C2: Double enumeration fix | Medium | 1–2 hrs | Won't fix |
| C1: VBUS GPIO investigation | Medium | 1–2 hrs | ✅ Fixed |
| C7: Per-pipe waker granularity | Low | 30 min | Deferred to Phase 3 |
