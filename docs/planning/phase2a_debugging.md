# Phase 2a Debugging Log

**Date**: 2026-02-15 → 2026-02-17
**Status**: ✅ Incremental debugging complete — all 6 steps passed. Root cause identified and fixed.
**Goal**: Validate phase 2a implementation (device detect, port reset, control transfers) on hardware

## Test Setup

- **Board**: Teensy 4.1 (i.MX RT 1062)
- **USB port**: USB2 host port (5-pin header), external VBUS via GPIO_EMC_40 load switch
- **Test device**: Low-speed USB keyboard
- **Example**: `hal_usb_host_init` — manual hardware init + PORTSC1 polling (does NOT use `HostController` trait)
- **Logging**: USB1 CDC serial (programming port)

## Entry 1: Device Detection — Register Dump Analysis

### Raw Output

```
[INFO hal_usb_host_init]: >>> DEVICE CONNECTED <<<
[INFO hal_usb_host_init]:     CCS=1 CSC=1 PE=0 PEC=0 PP=1 PR=0 SUSP=0 PSPD=1 (Low (1.5M))
[INFO hal_usb_host_init]: --- USB2 register dump ---
[INFO hal_usb_host_init]:   USBCMD        = 0x00018B15
[INFO hal_usb_host_init]:     RS=1 ASE=0 PSE=1 IAA=0 ITC=1
[INFO hal_usb_host_init]:   USBSTS        = 0x0000408C
[INFO hal_usb_host_init]:     HCH=0 PCI=1 SEI=0 AAI=0 UI=0 UEI=0
[INFO hal_usb_host_init]:   USBINTR       = 0x000C0037
[INFO hal_usb_host_init]:   PORTSC1       = 0x14001401
[INFO hal_usb_host_init]:     CCS=1 CSC=0 PE=0 PEC=0 PP=1 PR=0 SUSP=0 PSPD=1 (Low (1.5M))
[INFO hal_usb_host_init]:   USBMODE       = 0x00000003  (CM=3)
[INFO hal_usb_host_init]:   ASYNCLISTADDR = 0x20201200
[INFO hal_usb_host_init]: --- end register dump ---
```

### Overall Assessment

**✅ All registers correct.** Hardware init (phase 1) fully working. See register decode details below.

<details>
<summary>Full register decode (click to expand)</summary>

#### USBCMD = 0x00018B15

| Bit(s) | Field | Value | Expected | Status |
|--------|-------|-------|----------|--------|
| 0 | RS (Run/Stop) | 1 | 1 | ✅ Controller running |
| 1 | RST (Reset) | 0 | 0 | ✅ Not in reset |
| 3:2 | FS_1 (Frame List Size low) | 0b01 | 0b01 | ✅ |
| 4 | PSE (Periodic Schedule Enable) | 1 | 1 | ✅ Periodic schedule running |
| 5 | ASE (Async Schedule Enable) | 0 | 0 | ✅ Not enabled yet (no active transfers) |
| 6 | IAA (Interrupt on Async Advance) | 0 | 0 | ✅ No doorbell pending |
| 9:8 | ASP (Async Schedule Park count) | 0b11 | 0b11 | ✅ Park count = 3 |
| 11 | ASPE (Async Park Mode Enable) | 1 | 1 | ✅ |
| 15 | FS_2 (Frame List Size high) | 1 | 1 | ✅ FS=0b101 → 32-entry frame list |
| 23:16 | ITC (Interrupt Threshold) | 0x01 | 0x01 | ✅ 1 micro-frame |

#### USBSTS = 0x0000408C

| Bit | Field | Value | Notes |
|-----|-------|-------|-------|
| 0 | UI (USB Interrupt) | 0 | No transfer completion pending |
| 1 | UEI (USB Error) | 0 | No transfer errors |
| 2 | PCI (Port Change) | 1 | ✅ Expected — device just connected |
| 3 | FRI (Frame List Rollover) | 1 | Normal — frame list has rolled over at least once |
| 4 | SEI (System Error) | 0 | ✅ No AHB bus errors |
| 5 | AAI (Async Advance) | 0 | No doorbell pending |
| 7 | SRI (SOF Received) | 1 | Normal — SOF packets being sent |
| 12 | HCH (HC Halted) | 0 | ✅ Controller running |
| 14 | PS (Periodic Schedule Status) | 1 | ✅ Periodic schedule is active |
| 15 | AS (Async Schedule Status) | 0 | ✅ Async schedule not running (ASE=0) |

#### USBINTR = 0x000C0037

All expected interrupts enabled (UE, UEE, PCE, SEE, AAE, UAIE, UPIE). ✅

#### PORTSC1 = 0x14001401

CCS=1, PE=0 (expected — port enables after reset), PP=1, PSPD=1 (Low Speed). ✅

#### USBMODE = 0x00000003 → CM=3 (Host Controller mode). ✅

#### ASYNCLISTADDR = 0x20201200 → Sentinel QH in OCRAM2, 64-byte aligned. ✅

</details>

## Entry 2: `rtic_usb_enumerate` — No Output At All

**Date**: 2026-02-15
**Symptom**: Flashing `rtic_usb_enumerate` produces zero serial output. No banner, no log messages. The LED does not blink (no heartbeat in this example). The `rtic_heartbeat` example works perfectly — LED blinks, heartbeat messages appear on serial.

**Observation**: The failure is **not** in USB host enumeration — it's earlier than that. The example doesn't even print its banner, which happens *before* any USB2 host code runs.

## Incremental Debugging Plan

The strategy was to start from the working `rtic_heartbeat` example and add one feature at a time until the breaking step was found. Each step keeps the heartbeat LED + periodic log.

### Results Summary

| Step | Example | Added Feature | Result |
|------|---------|---------------|--------|
| 0 | `rtic_heartbeat` | Baseline | ✅ Works |
| 1 | `rtic_heartbeat_pll` | USB2 PLL enable | ✅ Passed |
| 2 | `rtic_heartbeat_vbus` | VBUS GPIO power | ✅ Passed |
| 3 | `rtic_heartbeat_init` | EHCI host.init() | ✅ Passed |
| 4 | `rtic_heartbeat_irq` | USB_OTG2 ISR + NVIC (priority 0xE0) | ✅ Passed |
| 5 | `rtic_heartbeat_spawn` | Async task spawn | ✅ Passed |
| 6 | `rtic_heartbeat_detect` | DeviceDetect stream | ✅ Passed |

### Root Cause: NVIC Priority 0 Bypasses RTIC BASEPRI

**The original `rtic_usb_enumerate` example did not set the NVIC priority for USB_OTG2.** The NVIC default priority is 0 (highest). On ARMv7-M, BASEPRI cannot mask priority 0 interrupts. RTIC uses BASEPRI for its critical sections (resource locking).

When `host.init()` enables interrupt sources in USBINTR and then `NVIC::unmask()` is called, any pending USB2 status bits immediately trigger the ISR at priority 0. This ISR preempts all RTIC critical sections, including those protecting the logging subsystem, causing a deadlock.

**Fix**: Set NVIC priority for USB_OTG2 to 0xE0 (hardware priority level 14, RTIC logical priority 2) *before* unmasking:

```rust
// Write to NVIC_IPRn byte register
let irq_num = ral::interrupt::USB_OTG2 as u32;
core::ptr::write_volatile((0xE000_E400 + irq_num) as *mut u8, 0xE0);
```

This puts the ISR within RTIC's BASEPRI-managed range so it is properly masked during critical sections.

### Init Restructuring

A second improvement discovered during debugging: **move all USB2 initialization from `init` to `idle`**. RTIC's init runs with PRIMASK set (all interrupts disabled), so USB1 CDC cannot enumerate on the host PC during init. By doing only USB1 CDC logging setup in init, then performing USB2 init in idle after a 5-second delay, all log messages are visible on the serial monitor.

### Step 4 Detailed Results

**Hardware test**: ISR count = 0 at boot, jumps to 1 on device plug, stays at 1 on unplug/replug. This is correct — the ISR's disable-on-handle pattern masks PCE after the first invocation, and no async poll re-enables it.

### Step 6 Detailed Results

**Hardware test** (low-speed keyboard):
```
DeviceStatus::Present(Low1_5)           ← plug in (clean, single event)
heartbeat #29-32  (usb2_isr count: 1)  ← stable
DeviceStatus::Absent                    ← unplug begins
DeviceStatus::Present(Low1_5)           ← contact bounce
DeviceStatus::Absent                    ← bounce
DeviceStatus::Present(Low1_5)           ← bounce
DeviceStatus::Absent                    ← bounce
DeviceStatus::Present(Low1_5)           ← bounce
DeviceStatus::Absent                    ← settled (device removed)
heartbeat #33  (usb2_isr count: 8)     ← 7 additional ISR firings from bounces
```

The multiple Absent/Present transitions during unplug are **USB contact bounce** — normal hardware behavior. The `UsbBus` layer in cotton-usb-host handles this with debounce delays (the `delay_ms` parameter to `device_events()`). The raw `device_detect()` stream intentionally does not debounce.

Key confirmations:
- ISR → waker → DeviceDetect → async poll chain works end-to-end
- Port change interrupts are correctly re-enabled after each poll (ISR count climbs from 1→8)
- `Imxrt1062DeviceDetect` stream correctly tracks state changes

## Entry 3: Enumeration Test — Control Transfer Hang

**Date**: 2026-02-17
**Status**: ✅ RESOLVED — Full USB enumeration working. Four bugs found and fixed.

### Test 1: No diagnostic logging

`rtic_usb_enumerate` with NVIC/init fixes but no `--features log`. Banner and init messages printed, "Entering enumeration loop..." appeared, but **no output on device plug or at boot**. No crash, just silence.

### Test 2: With diagnostic logging (`--features log`)

```
=== imxrt-usbh: USB Enumerate Example ===
USB2 PLL locked
VBUS power enabled
USB host controller initialised
USB_OTG2 ISR installed (NVIC priority 0xE0)
Entering enumeration loop...
[HC] device_detect() created
[HC] DeviceDetect: status change  PORTSC1=0x14001403
[HC] reset_root_port(true)  PORTSC1=0x14001401
[HC] reset_root_port(false) PORTSC1=0x14001007
[HC] control_transfer addr=0 pkt=8 bReq=6 wVal=0x0100 wLen=8 data_len=18
[HC] pipe allocated, starting transfer
[HC] async linked: CMD=0x00018B35 STS=0x0000C080 INTR=0x000C0033 PORTSC=0x14001407
[HC] QH: char=0x08085000 overlay_token=0x00000000  setup_qTD token=0x80008C80
```


### Register Analysis

| Register | Value | Decode |
|----------|-------|--------|
| PORTSC1 (after detect) | 0x14001403 | CCS=1, CSC=1, PE=0, PSPD=Low |
| PORTSC1 (before reset) | 0x14001401 | CCS=1, CSC=0, PE=0, PSPD=Low |
| PORTSC1 (after reset) | 0x14001007 | CCS=1, CSC=1, **PE=1**, PSPD=Low — **port enabled** |
| USBCMD | 0x00018B35 | RS=1, **ASE=1** (async schedule enabled), PSE=1 |
| USBSTS | 0x0000C080 | **AS=1** (async schedule running), PS=1, HCH=0 |
| USBINTR | 0x000C0033 | UE, UEE, SEE, AAE, UAIE, UPIE (PCE disabled by ISR — expected) |
| PORTSC1 (after link) | 0x14001407 | CCS=1, PE=1, PEC=1, PSPD=Low |
| QH char | 0x08085000 | addr=0, EP0, EPS=Low, DTC=1, C=1, max_pkt=8, **NAK_RL=0** |
| overlay_token | 0x00000000 | Active=0, Halted=0 (attach_qtd cleared halt) |

**Conclusion**: Device detect and port reset work perfectly. The first control transfer (GET_DESCRIPTOR 8 bytes to address 0) hangs — the EHCI async schedule is enabled and running but qTDs are not processed.

### Root Cause: Two Bugs

#### Bug 1: QH horizontal_link not flushed after linking (cache coherency)

In `do_control_transfer()`:
```rust
Self::cache_clean_qh(qh);           // ← Flushes QH to RAM (horizontal_link = TERMINATE)
unsafe { self.link_qh_to_async_schedule(qh) };  // ← Modifies BOTH qh.horizontal_link AND sentinel.horizontal_link
Self::cache_clean_qh(sentinel);      // ← Only flushes sentinel! QH is STALE in RAM!
```

`link_qh_to_async_schedule()` sets `qh.horizontal_link` to point back at the sentinel, but only the sentinel is re-flushed. The DMA engine reads the QH's stale `horizontal_link = LINK_TERMINATE` from RAM, breaking the circular async schedule. The EHCI controller stops traversal at the QH instead of looping back.

**Fix**: Add `Self::cache_clean_qh(qh)` after `link_qh_to_async_schedule()`.

#### Bug 2: NAK Count Reload = 0 (differs from reference implementation)

`qh_characteristics()` left NAK_RL at 0. USBHost_t36 (NXP's working reference for the same i.MX RT 1062 hardware) uses NAK_RL = 15 (bits [31:28] = 0xF).

- Our QH char: `0x08085000` (NAK_RL=0)
- USBHost_t36: `0xF8085000` (NAK_RL=15)

**Fix**: Set NAK_RL to 15 in `qh_characteristics()`.

### Fixes Applied (for Test 3)

1. `src/host.rs`: Added `Self::cache_clean_qh(qh)` after `link_qh_to_async_schedule(qh)` — both QH and sentinel are now flushed.
2. `src/ehci.rs`: Added `| 15u32 << QH_CHAR_NAK_RL_SHIFT` to `qh_characteristics()`.

### Test 3: Re-test with cache flush + NAK_RL fixes

**Result**: Still hangs. Output truncates mid-line:
```
[HC] QH: char=0xF8085000 overlay_token=0x00000000  setup_qTD token
```

NAK_RL=15 fix confirmed applied (0xF8085000). Truncation is serial buffer not flushing before hang — the real hang is in `TransferComplete` polling.

### Root Cause Analysis of Test 3

The test 2 output showed `setup_qTD token=0x80008C80`. But the expected setup token value is:

```
qtd_token(PID_SETUP=2, total_bytes=8, data_toggle=false, ioc=false)
= ACTIVE | (2<<8) | (3<<10) | (8<<16) | 0 | 0
= 0x00080E80
```

The observed value `0x80008C80` matches exactly the **status qTD token**:

```
qtd_token(PID_OUT=0, total_bytes=0, data_toggle=true, ioc=true)
= ACTIVE | 0 | (3<<10) | 0 | (1<<31) | IOC
= 0x80008C80
```

#### Bug 3: `alloc_qtd()` double-allocates — no reservation marking

`alloc_qtd()` finds free qTD slots by checking `token == 0 && next == TERMINATE`, but does NOT mark the slot before returning. All three qTD allocations (setup, data, status) happen *before* any `init()` calls:

```rust
let setup_qtd_idx = self.alloc_qtd()?;      // → pool[0] (token=0) → returns 0
let data_qtd_idx = Some(self.alloc_qtd()?);  // → pool[0] (token STILL 0!) → returns 0 again!
let status_qtd_idx = self.alloc_qtd()?;      // → pool[0] (token STILL 0!) → returns 0 again!
// ... only NOW do we call init() on each ...
```

All three indices are 0 — the same memory. The last `init()` (status qTD) overwrites the setup and data qTDs. The EHCI controller sees a single qTD with PID=OUT, 0 bytes, IOC=1, DT=1 — not a valid SETUP transaction. The controller never gets a valid response and hangs.

**Fix**: Mark the qTD as allocated in `alloc_qtd()` by writing `QTD_TOKEN_ACTIVE` to the token before returning. Subsequent calls will see `token != 0` and skip the slot.

```rust
fn alloc_qtd(&self) -> Option<usize> {
    for i in 0..NUM_QTD {
        let qtd = &self.statics.qtd_pool[i];
        if qtd.token.read() == 0 && qtd.next.read() == LINK_TERMINATE {
            // Mark as allocated immediately to prevent double-allocation
            unsafe {
                let qtd_ptr = self.qtd_mut(i);
                (*qtd_ptr).token.write(QTD_TOKEN_ACTIVE);
            }
            return Some(i);
        }
    }
    None
}
```

### Tests 4–6: Log buffer exhaustion

Tests 4–6 confirmed qTD allocation fix worked (indices 0/1/2, token 0x00080E80) but
all diagnostic output was silently dropped after filling the 1024-byte log buffer.
The `log` crate + `bbqueue`-backed USB CDC logger (from `imxrt-log`) is non-blocking —
`info!()` silently discards messages when the buffer is full. Since `poll_logger` runs
at RTIC priority 1 (same as the enumeration task), it can't preempt to drain the buffer.

**Resolution**: Stripped verbose internal logging, kept only essential entry/exit messages.

### Test 7: Spin-wait confirmed EHCI hardware works

After stripping diagnostics, added a spin-wait loop directly polling the setup qTD's
Active bit, bypassing the async waker machinery completely.

```
[HC] setup done spins=7919 tok=0x80000E00
```

**Result**: EHCI hardware completed the setup qTD in ~7919 spins (~13μs at 600 MHz).
Token decode: Active=0, total_bytes=0, CERR=3, DT=1, no errors — perfect.

**Conclusion**: Hardware works. Problem is in the async waker/ISR path.

### Test 8: TransferComplete polled, but hangs after Ok

Added lightweight `info!()` in `TransferComplete::poll()`:

```
[HC] poll tok=0x80008C80 STS=0x0000C088 INTR=0x000C0033
[HC] poll tok=0x80008C80 STS=0x0000C088 INTR=0x000C0033
[HC] poll tok=0x00008C00 STS=0x0000C000 INTR=0x00080032
```

**Analysis**: Three polls — first two: status_qtd Active (qTD chain still in progress).
Third: Active=0, no errors. `TransferComplete` returned Ok. But system hung afterward.
The hang was in `wait_async_advance().await` — the next `.await` point.

#### Bug 4: ISR clears USBSTS.AAI before `AsyncAdvanceWait::poll()` reads it

**Root cause**: The ISR acknowledged ALL pending USBSTS bits via `write_reg!(USBSTS, status)`.
This W1C write cleared AAI (bit 5). Then `AsyncAdvanceWait::poll()` (woken by the ISR's
`async_advance_waker.wake()`) read USBSTS.AAI = 0 (already gone), re-enabled AAE, returned
`Pending`, and hung forever.

**Fix**: ISR now writes `status & !(1 << 5)` to USBSTS — clears all status bits EXCEPT AAI.
`AsyncAdvanceWait::poll()` reads AAI directly and clears it itself.

```rust
// ISR: don't clear AAI — let AsyncAdvanceWait::poll() read and clear it
ral::write_reg!(ral::usb, usb, USBSTS, status & !(1 << 5));
```

### Test 9: Full enumeration success ✅

```
[INFO rtic_usb_enumerate::app]: Entering enumeration loop...
[INFO imxrt_usbh::host]: [HC] DeviceDetect: status change  PORTSC1=0x14001403
[INFO imxrt_usbh::host]: [HC] control_transfer -> Ok(8)
[INFO imxrt_usbh::host]: [HC] control_transfer -> Ok(18)
[INFO imxrt_usbh::host]: [HC] control_transfer -> Ok(0)
[INFO rtic_usb_enumerate::app]: DeviceEvent::Connect  addr=1  VID=045e PID=00db class=0 subclass=0
```

**Device identified**: Microsoft keyboard (VID=045e, PID=00db). Class=0/subclass=0
indicates class is defined at interface level (standard for HID).

**Enumeration sequence**:
1. `Ok(8)` — GET_DESCRIPTOR(Device) 8 bytes to address 0 (extracts bMaxPacketSize0)
2. `Ok(18)` — GET_DESCRIPTOR(Device) full 18 bytes
3. `Ok(0)` — SET_ADDRESS (assigns address 1, no data phase)
4. `DeviceEvent::Connect` — cotton-usb-host reports successful enumeration

### All Bugs Summary

| Bug | Category | Root Cause | Fix |
|-----|----------|------------|-----|
| 1 | Cache coherency | QH horizontal_link not flushed after linking to async schedule | Flush both QH and sentinel after `link_qh_to_async_schedule()` |
| 2 | EHCI config | NAK Count Reload = 0 (should be 15, per USBHost_t36 reference) | Set NAK_RL=15 in `qh_characteristics()` |
| 3 | Resource mgmt | `alloc_qtd()` returned same index for all 3 qTDs (no reservation mark) | Write `QTD_TOKEN_ACTIVE` on alloc before returning |
| 4 | ISR/async race | ISR cleared USBSTS.AAI before `AsyncAdvanceWait::poll()` could read it | Don't W1C AAI in ISR; let poll function clear it |

### Key operational finding: 1024-byte log buffer

The `imxrt-log` USB CDC backend uses a 1024-byte `bbqueue` ring buffer. At RTIC priority 1,
the logger's poll task can't preempt the enumeration task. Any synchronous burst of `info!()`
calls exceeding ~1KB will silently drop messages. Future logging should use `defmt` (structured
binary logging) or increase the buffer size via `IMXRT_LOG_BUFFER_SIZE`.

## Reference: RTIC v2 `delay_ms` Implementation (from cotton RP2040 examples)

The `device_events()` and `device_events_no_hubs()` methods require a delay function
with signature `Fn(usize) -> impl Future<Output = ()> + 'static + Clone`.

### The exact `rtic_delay` function

From `cotton/cross/rp2040-w5500-rtic2/src/bin/rp2040-usb-msc.rs` (line 120) and
`rp2040-usb-otge100.rs` (line 231) — identical in both:

```rust
fn rtic_delay(ms: usize) -> impl Future<Output = ()> {
    Mono::delay(<Mono as rtic_monotonics::Monotonic>::Duration::millis(
        ms as u64,
    ))
}
```

### For i.MX RT adaptation

- Replace `thumbv6-backend` with `thumbv7-backend`
- Replace `rp2040_timer_monotonic!(Mono)` with a GPT-based monotonic or `systick_monotonic!(Mono, 1000)` (from `rtic-monotonics` `cortex-m-systick` feature)
- The `rtic_delay` function body is identical regardless of platform — it only depends on the `Mono` type
- The ISR binds to `USB_OTG2` (interrupt #112) instead of `USBCTRL_IRQ`
- The current `rtic_usb_enumerate` uses a busy-wait `delay_ms()` that blocks the CPU. A proper monotonic-based delay would allow the executor to run other tasks during the wait.
