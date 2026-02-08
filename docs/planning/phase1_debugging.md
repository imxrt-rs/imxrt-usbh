# Phase 1 Debugging & Testing Plan

**Date**: 2026-02-08  
**Status**: ✅ Device detected — low-speed keyboard connected via external 5V power  
**Goal**: Identify and fix all issues preventing USB device detection after `init()`

## Hardware Verification

**Important**: The same keyboard powers up correctly when using a Teensyduino-compiled
application (USBHost_t36 library). This confirms the hardware is working — the issue
is in the Rust code, not the Teensy 4.1 board, USB port, or keyboard.

## Symptom

The `hal_usb_host_init` example:
- ✅ Prints all expected log messages (logging over USB1 CDC works)
- ✅ LED blinks (PIT timer, GPIO, and board init all functional)
- ✅ `init()` returns without hanging (controller reset, PHY, mode, run — all complete)
- ❌ Plugging a keyboard into the USB2 host port → **no power to the keyboard**

Since the keyboard receives no power at all, the USB bus never becomes active and
no further host-mode behaviour (connect detect, bus reset, enumeration) can be
verified yet.

## Root Cause Analysis

### Issue 1 (Critical): Missing VBUS GPIO Power Switch

**Diagnosis**: The Teensy 4.1 USB2 host port uses an external power switch controlled
by `GPIO_EMC_40`. The `init()` sequence sets `PORTSC1.PP` (port power bit), which
tells the EHCI controller that port power is available, but this only affects the
controller's internal state. The **physical 5V supply** to the USB connector is gated
by the board-level power switch and requires driving `GPIO_EMC_40` high.

**Evidence from USBHost_t36** (local: `../USBHost_t36/ehci.cpp`, lines 209–212):

```cpp
#ifdef ARDUINO_TEENSY41
IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_40 = 5;        // ALT5 = GPIO3_IO26
IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_40 = 0x0008;   // slow slew, weak 150Ω drive
GPIO8_GDIR |= 1<<26;                           // output
GPIO8_DR_SET = 1<<26;                           // drive HIGH → VBUS on
#endif
```

This must be done **before** (or during) host controller initialisation for the
connected device to receive 5V power.

**The board crate does not touch USB2 at all** — it only sets up USB1 (the
programming port). There is no `gpio_emc` pad configuration, no `GPIO_EMC_40`
mux, no VBUS control anywhere in the board or HAL crate for USB2. This is
entirely the responsibility of the host driver or the example code.

**Fix options** (ordered by preference):

| Option | Description | Pros | Cons |
|--------|-------------|------|------|
| **A** | Add VBUS GPIO setup to the example, before `host.init()` | Simple, explicit, matches USBHost_t36 pattern | Board-specific code in example |
| **B** | Accept a VBUS GPIO pin as a parameter to `init()` | Portable, works for different boards | Adds complexity to the `Peripherals` trait |
| **C** | Document as a prerequisite (caller responsibility) | Minimal API change | Easy to forget, as we just demonstrated |

**Recommendation**: Start with **Option A** for immediate debugging (unblocks
testing), then migrate to **Option B** or **Option C** once the init sequence is
validated end-to-end.

#### Option A Implementation Steps

1. In `hal_usb_host_init.rs`, add VBUS GPIO setup between the PLL enable and
   `host.init()` call:
   - Configure `IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_40` = ALT5 (GPIO3_IO26)
   - Configure `IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_40` = `0x0008` (slow slew, weak drive)
   - Set `GPIO8_GDIR` bit 26 (output)
   - Set `GPIO8_DR_SET` bit 26 (drive high → enable VBUS)
   - Log the action: `log::info!("VBUS power enabled (GPIO_EMC_40 HIGH)")`

2. The register addresses for raw access are:
   - `IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_40`: `0x401F_80C4`
   - `IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_40`: `0x401F_82B4`
   - `GPIO8_GDIR`: `0x4200_8004` (GPIO8 is at `0x4200_8xxx`, NOT `0x4200_4xxx` which is GPIO7!)
   - `GPIO8_DR_SET`: `0x4200_8084`

   Alternatively, use `imxrt-ral` and `imxrt-iomuxc` (already in dev-dependencies)
   for type-safe access.

3. Add a brief delay after enabling VBUS (USBHost_t36 uses 10ms) to allow the
   power switch to stabilise before the controller starts looking for devices.

### Issue 2 (Possible): ENHOSTDISCONDETECT Timing

**Diagnosis**: The `init()` sequence enables `ENHOSTDISCONDETECT` in the PHY
immediately, even though no device is connected. USBHost_t36 only enables this
bit **after** a device connects and the port is enabled — specifically in the ISR
when `PORTSC.PE` becomes set.

**Risk**: The PHY's disconnect detector may interfere with initial connection
detection if enabled too early. The i.MX RT reference manual (Chapter 57,
USBPHY CTRL register) notes that the host disconnect detect circuit should only
be enabled when a device is connected.

**Impact**: Low-medium. This likely doesn't prevent power delivery but could cause
the controller to fail to detect the initial device connection.

**Fix**: Move `ENHOSTDISCONDETECT` enable from `init()` to the device-detect
state machine (Phase 2a), enabling it only after the port reaches the enabled
state. For now, it's fine to leave as-is during debugging — it may work correctly
regardless.

### Issue 3 (Possible): PORTSC1 Modify-Reg W1C Hazard

**Diagnosis**: The `init()` code uses `modify_reg!` on `PORTSC1` to set `PP`:

```rust
ral::modify_reg!(ral::usb, self.usb, PORTSC1, PP: 1);
```

`PORTSC1` contains several Write-1-to-Clear (W1C) status bits: `CSC` (bit 1),
`PEC` (bit 3), `OCC` (bit 5), `FPR` (bit 6). A read-modify-write on this
register risks inadvertently clearing pending status bits by reading a `1` and
writing it back.

**Impact**: Low during init (no device is connected so no status bits should be
set), but this pattern must be fixed before Phase 2 to avoid dropping connect/
disconnect events.

**Fix**: Use a direct write with W1C bits explicitly zeroed:

```rust
// Write PP=1 with all W1C bits written as 0 to avoid clearing them
ral::write_reg!(ral::usb, self.usb, PORTSC1,
    PP: 1  // Set port power
    // W1C bits (CSC, PEC, OCC) are implicitly 0 in a write_reg
);
```

Or use a raw write with a known-safe mask. This is a common EHCI pitfall
documented in the EHCI spec §2.3.9 and the design doc
[USB_CONTROL_TRANSFERS.md](../design/USB_CONTROL_TRANSFERS.md).

### Issue 4 (Informational): No NVIC Interrupt Enabled

**Diagnosis**: The example does not enable the `USB_OTG2` interrupt (IRQ #112)
in the NVIC. The `init()` sequence enables interrupt sources in `USBINTR`, but
without the NVIC interrupt, the ISR (`UsbShared::on_irq()`) will never be called.

**Impact**: None for Phase 1 (we only care about power and LED blink). Critical
for Phase 2 (device detection, transfer completion). This is documented as a
prerequisite in `init()` and will be addressed when moving to RTIC.

### Issue 5 (Informational): No Cache Maintenance on Init Structures

**Diagnosis**: During `init()`, the sentinel QH is written by the CPU and its
address is given to the controller via `ASYNCLISTADDR`. Similarly, the frame list
address is written to `DEVICEADDR`. If these structures are in cached RAM (OCRAM),
the controller may read stale data from main memory.

**Impact**: Low during init — the controller is not yet running (RS=0) when the
sentinel and frame list are set up, and the CPU writes are committed before RS=1
because they occur as part of the same `init()` call with register writes (which
are strongly-ordered) in between. However, adding explicit cache maintenance
would make the code more robust and avoid subtle issues if the code is reordered
in the future.

**Fix**: Add `clean_invalidate_dcache_by_address()` calls after writing the
sentinel QH and before writing `ASYNCLISTADDR`:

```rust
cache::clean_invalidate_dcache_by_address(
    sentinel as usize,
    core::mem::size_of::<QueueHead>(),
);
```

Same for the frame list before writing `DEVICEADDR`.

## Debugging Steps (Ordered)

### Step 1: Add VBUS GPIO Power Control

This is the most likely root cause. Implement Option A above in the example and
re-test. If the keyboard powers up (LEDs light, draws current), this confirms
the issue.

**Expected result**: Keyboard receives 5V power, LEDs on keyboard light up.

### Step 2: Read Back and Log Key Registers After Init

Add register read-back logging after `init()` to verify the controller state:

```rust
// After host.init():
let usbcmd = /* read USBCMD */;
let usbsts = /* read USBSTS */;
let portsc = /* read PORTSC1 */;
let usbmode = /* read USBMODE */;
log::info!("USBCMD  = {:#010X}", usbcmd);
log::info!("USBSTS  = {:#010X}", usbsts);
log::info!("PORTSC1 = {:#010X}", portsc);
log::info!("USBMODE = {:#010X}", usbmode);
```

**Expected values** (approximate):

| Register | Expected | Key bits |
|----------|----------|----------|
| `USBCMD` | `0x0001_8B15` | RS=1, PSE=1, ASE=0 (no transfers yet), FS=32, ASPMC=3, ASPE=1, ITC=1 |
| `USBSTS` | HCH=0 (running) | RS=1 should clear HCH after a short delay |
| `PORTSC1` | PP=1 | Port power enabled; CCS=0 if no device |
| `USBMODE` | CM=3 | Host mode |

If any of these are wrong, that narrows the problem.

Here's the debug output before anything is plugged in:
```
[INFO hal_usb_host_init]: === imxrt-usbh: USB Host Init Example ===
[INFO hal_usb_host_init]: Board initialised, logging over USB CDC serial
[INFO hal_usb_host_init]: Enabling USB2 PLL (PLL_USB2)...
[INFO hal_usb_host_init]: USB2 PLL locked and running at 480 MHz
[INFO hal_usb_host_init]: Enabling VBUS power (GPIO_EMC_40 → HIGH)...
[INFO hal_usb_host_init]: VBUS power enabled
[INFO hal_usb_host_init]: Acquiring USB2 and USBPHY2 peripheral instances...
[INFO hal_usb_host_init]: Constructing USB host controller...
[INFO hal_usb_host_init]: Initialising EHCI host controller...
[INFO hal_usb_host_init]:   - PHY reset and power-up
[INFO hal_usb_host_init]:   - Controller reset
[INFO hal_usb_host_init]:   - Set host mode (CM=3)
[INFO hal_usb_host_init]:   - Configure async + periodic schedules
[INFO hal_usb_host_init]:   - Enable interrupts and run controller
[INFO hal_usb_host_init]: USB host controller initialised successfully!
[INFO hal_usb_host_init]: --- USB2 register dump ---
[INFO hal_usb_host_init]:   USBCMD        = 0x00018B15
[INFO hal_usb_host_init]:     RS=1 ASE=0 PSE=1 IAA=0 ITC=1
[INFO hal_usb_host_init]:   USBSTS        = 0x00004088
[INFO hal_usb_host_init]:     HCH=0 PCI=0 SEI=0 AAI=0 UI=0 UEI=0
[INFO hal_usb_host_init]:   USBINTR       = 0x000C0037
[INFO hal_usb_host_init]:   PORTSC1       = 0x1C001000
[INFO hal_usb_host_init]:     CCS=0 CSC=0 PE=0 PEC=0 PP=1 PR=0 SUSP=0 PSPD=3 (??)
[INFO hal_usb_host_init]:   USBMODE       = 0x00000003  (CM=3)
[INFO hal_usb_host_init]:   ASYNCLISTADDR = 0x20201200
[INFO hal_usb_host_init]: --- end register dump ---
[INFO hal_usb_host_init]: Sentinel QH addr  = 0x20201200 (64B-aligned: true)
[INFO hal_usb_host_init]: Frame list addr   = 0x20200000 (4096B-aligned: true)
[INFO hal_usb_host_init]: Entering main loop — polling PORTSC1 for device events
[INFO hal_usb_host_init]: Blinking LED at 4 Hz to indicate success
```

**Analysis**: The register values are correct for a properly initialized host controller
with no device connected:

| Register | Value | Assessment |
|----------|-------|------------|
| `USBCMD` | `0x00018B15` | ✅ RS=1 (running), PSE=1, ITC=1, FS=32. ASE=0 is correct (no async transfers yet). |
| `USBSTS` | `0x00004088` | ✅ HCH=0 (not halted), no error bits. SRI and FLR are normal operational status. |
| `PORTSC1` | `0x1C001000` | ✅ PP=1 (port power on), CCS=0 (no device — expected). PSPD=3 = "not connected". |
| `USBMODE` | `0x00000003` | ✅ CM=3 = host mode. |
| `ASYNCLISTADDR` | `0x20201200` | ✅ Matches sentinel QH address, 64B-aligned. |
| Frame list | `0x20200000` | ✅ 4096B-aligned. |

**Next step**: Plug a keyboard into the USB2 host port and check if
`>>> DEVICE CONNECTED <<<` appears in the log output. If not, the VBUS power
may not be reaching the device — investigate using fast GPIO8 bank instead of
GPIO3.

#### Device Connection Test Result (2026-02-08)

**Result**: ❌ FAILED — no `>>> DEVICE CONNECTED <<<` message, keyboard receives no power.

**Root cause hypothesis**: The current VBUS GPIO code uses the **slow GPIO3 bank**
(`0x401C_xxxx`), but USBHost_t36 uses the **fast GPIO8 bank** (`0x4200_4xxx`). On
i.MX RT 1062, GPIO3 and GPIO8 map to the same physical pins (GPIO3_IO26 = GPIO8_IO26
= `GPIO_EMC_40`), but the register interfaces are different. The slow GPIO registers
may not be functioning as expected for this use case.

**Next steps**:

1. **Switch to GPIO8 (fast GPIO)** — Change the VBUS power setup to use `GPIO8_GDIR`
   at `0x4200_4004` and `GPIO8_DR_SET` at `0x4200_4084` instead of the GPIO3 registers.
   This matches USBHost_t36 exactly.
   
   **Result (Build 1.01)**: ❌ FAILED — Still no power to keyboard.

2. **Verify IOMUXC pad configuration** — Read back the mux register after writing
   to confirm the write took effect.
   
   **Result (Build 1.02)**:
   ```
   IOMUXC MUX  = 0x00000005 (expect 5)       ✅
   IOMUXC PAD  = 0x00000008 (expect 0x0008)  ✅
   GPIO8 GDIR  = 0x04000000 (bit 26 set?)    ✅
   GPIO8 DR    = 0xAC10C001 (bit 26 set?)    ✅
   ```
   All registers looked correct... but **we were reading the wrong GPIO bank!**
   
   **Root cause found**: We were using `0x4200_4xxx` which is **GPIO7**, not GPIO8!
   The i.MX RT 1062 fast GPIO mapping is:
   - GPIO6 (GPIO1 fast): 0x4200_0000
   - GPIO7 (GPIO2 fast): 0x4200_4000  ← we were writing here!
   - GPIO8 (GPIO3 fast): 0x4200_8000  ← correct address
   - GPIO9 (GPIO4 fast): 0x4200_C000
   
   `GPIO_EMC_40` = GPIO3_IO26 = GPIO8_IO26. We need GPIO8 at `0x4200_8xxx`.

3. **Fix GPIO8 address** — Use correct base `0x4200_8000` for GPIO8.
   Switched to RAL symbolic access (`ral::gpio::GPIO8::instance()`) to avoid
   future address errors.
   
   **Result (Build 1.05)**: ❌ FAILED — Still no power to keyboard.
   GPIO8 readback confirms correct registers are now being used, but the
   load switch is not enabling VBUS.

4. **Enable IOMUXC_GPR_GPR27 fast GPIO routing** — `IOMUXC_GPR_GPR27` controls
   whether GPIO3 pins are driven by the regular GPIO3 bank or the fast GPIO8 bank.
   Teensyduino's startup code sets this to `0xFFFFFFFF`; without it, GPIO8 writes
   update the internal register but the physical pin stays controlled by GPIO3.
   
   **Result (Build 1.06)**: ❌ FAILED — Still no power to keyboard.
   Readback (all correct):
   ```
   IOMUXC MUX  = 0x00000005 (expect 5)                            ✅
   IOMUXC PAD  = 0x00000008 (expect 0x0008)                       ✅
   GPR27       = 0x04000000 (bit 26 routes GPIO3→GPIO8)           ✅
   GPIO8 GDIR  = 0x04000000 (bit 26 = output)                     ✅
   GPIO8 DR    = 0x0DD00008 (bit 26 = 1, pin should be high)      ✅
   ```
   All registers are now provably correct — IOMUXC mux, pad config, GPR27
   fast GPIO routing, GPIO8 direction, and GPIO8 data all match USBHost_t36.
   The issue is likely hardware-level (load switch circuit, pin routing on
   this specific Teensy revision, or something in boot/startup that we're
   not replicating).

5. **Hardware debugging** — use multimeter/oscilloscope to verify:
   - Voltage on GPIO_EMC_40 pad (should be 3.3V if driving high)
   - Voltage on VBUS pin of USB2 host header (should be 5V if load switch on)
   - Continuity between GPIO_EMC_40 and the load switch enable input
   
   **Status**: Deferred — using external 5V power supply instead.

**Decision**: The VBUS GPIO issue is deferred. All registers read back correctly
and match USBHost_t36 exactly. Further debugging requires hardware tools
(multimeter/oscilloscope) to determine if the pin is physically driving high
and if the load switch is responding. In the meantime, external 5V power is
supplied to the USB device via the prototyping harness, allowing development
to continue with device detection and enumeration.

**Remaining VBUS hypotheses** (for future investigation):
- Teensy board revision differences in load switch wiring
- Teensyduino startup.c may configure additional registers we haven't identified
  (e.g., GPIO_EMC_40 could have a different function on the specific board)
- The load switch may require a specific power-on sequencing order
- Possible `imxrt-rt` startup code difference vs Teensyduino startup that affects
  pad/GPIO default states

6. **External power test** — Supply 5V externally to USB device, bypassing the
   on-board load switch entirely.
   
   **Result (Build 1.06 + external 5V)**: ✅ **SUCCESS — Device detected!**
   ```
   >>> DEVICE CONNECTED <<<
       CCS=1 CSC=1 PE=0 PEC=0 PP=1 PR=0 SUSP=0 PSPD=1 (Low (1.5M))
   ```
   Post-connect register dump:
   ```
   USBCMD        = 0x00018B15   RS=1 ASE=0 PSE=1 IAA=0 ITC=1
   USBSTS        = 0x0000408C   HCH=0 PCI=1 SEI=0 AAI=0 UI=0 UEI=0
   USBINTR       = 0x000C0037
   PORTSC1       = 0x14001401   CCS=1 CSC=0 PE=0 PEC=0 PP=1 PR=0 SUSP=0 PSPD=1 (Low)
   USBMODE       = 0x00000003   CM=3 (Host)
   ASYNCLISTADDR = 0x20201200
   ```
   
   **Analysis**:
   - **CCS=1**: Connection detected — the EHCI controller sees the device on the bus
   - **PSPD=1 (Low-speed, 1.5 Mbps)**: Correct for a USB HID keyboard
   - **PE=0**: Port not yet enabled — expected since we haven't performed a bus reset
   - **PCI=1 in USBSTS**: Port Change Interrupt fired (would trigger ISR if NVIC enabled)
   - **HCH=0, SEI=0**: Controller running, no system errors
   - **ASYNCLISTADDR=0x20201200**: Valid DTCM address for sentinel QH
   - **CSC=0 in final dump**: Our polling code correctly W1C-cleared the CSC bit
   
   **Conclusion**: The USB2 EHCI host controller initialisation is fully working.
   Device connection detection works. The only outstanding issue is VBUS power
   control via GPIO_EMC_40, which is deferred. Phase 1 is **COMPLETE**.

### Step 3: Verify PLL Lock

Log the PLL_USB2 register after PLL setup to confirm it is locked:

```rust
let pll = /* read CCM_ANALOG_PLL_USB2 */;
log::info!("PLL_USB2 = {:#010X}", pll);
// Expected: LOCK=1, ENABLE=1, POWER=1, BYPASS=0, EN_USB_CLKS=1
```

### Step 4: Check for HardFault / Bus Error

If the controller silently faults (e.g. bus error due to misaligned DMA
structures), the `SEE` (System Error) bit in `USBSTS` will be set. Check this
in the register dump from Step 2.

Also verify that the QH pool and frame list addresses are correctly aligned:

```rust
log::info!("Sentinel QH addr = {:#010X}", &statics.qh_pool[0] as *const _ as u32);
log::info!("Frame list addr  = {:#010X}", &statics.frame_list as *const _ as u32);
// Sentinel must be 64-byte aligned (low 6 bits = 0)
// Frame list must be 4096-byte aligned (low 12 bits = 0)
```

### Step 5: Continuous PORTSC1 Polling (After VBUS Fix)

Once VBUS power is working, poll `PORTSC1` in the main loop to watch for
device connection:

```rust
loop {
    poller.poll();
    if blink_timer.is_elapsed() {
        while blink_timer.is_elapsed() {
            blink_timer.clear_elapsed();
        }
        led.toggle();
        let portsc = /* read PORTSC1 */;
        log::info!("PORTSC1 = {:#010X}", portsc);
    }
}
```

**Expected on device plug-in**: `CCS` (bit 0) transitions from 0→1, `CSC`
(bit 1) is set (W1C), and `PSPD` (bits 27:26) indicates the device speed.

### Step 6: Verify Interrupt Delivery (Phase 2 Prerequisite)

When ready to move beyond polling, enable the `USB_OTG2` interrupt in the NVIC
and verify that `UsbShared::on_irq()` is called on port change events. This will
be done as part of Phase 2a (RTIC integration).

## Test Matrix

| Test | Precondition | Action | Expected |
|------|-------------|--------|----------|
| VBUS power | GPIO_EMC_40 configured | Plug in keyboard | Keyboard LEDs light up | ⚠️ Deferred — using external 5V |
| VBUS current | Multimeter on VBUS pin | Plug in keyboard | 5V present, ~100mA draw | ⚠️ Deferred |
| PORTSC1 CCS | VBUS working | Plug in keyboard | CCS=1 in register dump | ✅ CCS=1 confirmed |
| PORTSC1 PSPD | VBUS working, device connected | Read PORTSC1 | PSPD=00 (FS) or 01 (LS) | ✅ PSPD=1 (Low-speed) |
| No faults | Init complete | Check USBSTS | SEE=0, HCH=0 | ✅ Confirmed |
| PLL lock | PLL setup complete | Read PLL_USB2 | LOCK=1 | ✅ (PLL brings up USB clocks) |
| Alignment | Before init | Log QH/frame list addresses | QH 64B-aligned, FL 4096B-aligned | ✅ ASYNCLISTADDR=0x20201200 (64B-aligned) |

## Hardware Notes

### Teensy 4.1 USB2 Host Port Wiring

The USB2 host port is the **5-pin header** on the Teensy 4.1 board (not the
micro-USB connector used for programming). The pins are directly on the
board — no adapter needed, but you must solder header pins and provide a
USB-A breakout or similar.

| Pin | Signal | Notes |
|-----|--------|-------|
| 1 | VBUS (5V) | Switched by GPIO_EMC_40 via on-board load switch |
| 2 | D− | USB data minus |
| 3 | D+ | USB data plus |
| 4 | ID | OTG ID pin (tie to GND for host mode, or leave floating) |
| 5 | GND | Ground |

**Important**: The 5V VBUS on this header is **not always powered**. It is gated
by an on-board load switch controlled by `GPIO_EMC_40` (ALT5 = GPIO3_IO26 /
fast GPIO8_IO26). You **must** drive this pin high to supply power to the
connected device. See Issue 1 above.

### GPIO_EMC_40 → VBUS Power Switch Details

| Property | Value |
|----------|-------|
| Pad | `GPIO_EMC_40` |
| IOMUX ALT | ALT5 (GPIO3_IO26) |
| Fast GPIO | GPIO8_IO26 |
| MUX register | `IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_40` @ `0x401F_80C4` |
| PAD register | `IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_40` @ `0x401F_82B4` |
| Pad config | `0x0008` (slow slew, 150Ω drive, no pull) |
| Direction | Output |
| Active level | HIGH = VBUS on |

### Reference Schematic

See the [Teensy 4.1 schematic (PJRC)](https://www.pjrc.com/store/teensy41.html)
for the USB2 host port circuit. The load switch is connected between the main 5V
rail and the USB host connector VBUS pin, with `GPIO_EMC_40` as the enable input.

## References

- i.MX RT 1060 Reference Manual (local: [`docs/external/IMXRT1060RM_rev2.pdf`](../external/IMXRT1060RM_rev2.pdf))
  - Chapter 56: USB OTG controller — PORTSC1 register (§56.7.15), USBCMD (§56.7.1)
  - Chapter 57: USBPHY — CTRL register, ENHOSTDISCONDETECT
  - Chapter 12: IOMUXC — `GPIO_EMC_40` pad mux and control
- EHCI Specification (local: [`docs/external/ehci-specification-for-usb.pdf`](../external/ehci-specification-for-usb.pdf))
  - §2.3.9: PORTSC register W1C bit handling
  - §4.1: Initialization
- USBHost_t36 (local: `../USBHost_t36/ehci.cpp`)
  - Lines 107–270: `USBHost::begin()` — full init sequence including VBUS GPIO
  - Lines 301–430: ISR port state machine
- Phase 1 foundation doc: [phase1_foundation.md](phase1_foundation.md)
- Cache coherency design: [CACHE_COHERENCY.md](../design/CACHE_COHERENCY.md)

---

**Next steps**: Phase 1 is complete. Proceed to Phase 2a:
1. Implement bus reset (`reset_root_port()`) — drive PORTSC1.PR=1 for ≥50ms, then PR=0
2. After reset, PE should become 1 (port enabled) — read PSPD for device speed
3. Implement `device_detect()` stream using PORTSC1 polling or PCI interrupt
4. Implement first control transfer: `GET_DESCRIPTOR(Device)` to address 0
5. Enable USB_OTG2 interrupt (IRQ #112) in NVIC for interrupt-driven operation

VBUS GPIO fix is deferred — use external 5V power supply for testing.
