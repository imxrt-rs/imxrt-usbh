# Phase 2c Debugging: USB Mass Storage Control Transfer Failure

**Date**: 2026-02-18
**Symptom**: `rtic_usb_mass_storage` example detects USB flash drive (High Speed),
but `control_transfer` fails during enumeration. Device never reaches the MSC bulk
transfer phase. Both enumeration attempts (FS-detected, then HS-detected) fail.

## Log Output

```
[INFO rtic_usb_mass_storage::app]: === imxrt-usbh: USB Mass Storage Example ===
[INFO rtic_usb_mass_storage::app]: USB2 PLL locked
[INFO rtic_usb_mass_storage::app]: VBUS power enabled
[INFO rtic_usb_mass_storage::app]: USB host controller initialised
[INFO rtic_usb_mass_storage::app]: USB_OTG2 ISR installed (NVIC priority 0xE0)
[INFO rtic_usb_mass_storage::app]: Entering device event loop...
[INFO imxrt_usbh::host]: [HC] DeviceDetect: status change  PORTSC1=0x10001803
[WARN imxrt_usbh::host]: [HC] control_transfer -> Err
[WARN rtic_usb_mass_storage::app]: DeviceEvent::EnumerationError  hub=0 port=1
[INFO imxrt_usbh::host]: [HC] DeviceDetect: status change  PORTSC1=0x18001207
[WARN imxrt_usbh::host]: [HC] control_transfer -> Err
[WARN rtic_usb_mass_storage::app]: DeviceEvent::EnumerationError  hub=0 port=1
```

## Confirmed Working

- PLL, VBUS, PHY, host controller init: ✅ (identical to keyboard example)
- NVIC priority 0xE0 for USB_OTG2: ✅ (ISR can preempt RTIC priority-1 tasks)
- Device detect stream: ✅ (two events received)
- Port reset (PR → PE): ✅ (second PORTSC shows PE=1, HSP=1)
- Async schedule machinery: ✅ (proven working in phases 2a/2b with LS keyboard)

## PORTSC1 Decode

### First detection: `PORTSC1 = 0x10001803`

| Field | Bits | Value | Meaning |
|-------|------|-------|---------|
| CCS   | 0    | 1     | Device connected |
| CSC   | 1    | 1     | Connect status changed (W1C) |
| PE    | 2    | 0     | Port NOT enabled (pre-reset) |
| LS    | 11:10| 0b10  | D+ high → Full Speed device signaling |
| PP    | 12   | 1     | Port power on |
| PSPD  | 27:26| 0b00  | Full Speed (pre-reset, before chirp) |

**cotton-usb-host sees**: `Present(Full12)` → starts enumeration with speed=FS.

### Second detection: `PORTSC1 = 0x18001207`

| Field | Bits | Value | Meaning |
|-------|------|-------|---------|
| CCS   | 0    | 1     | Device connected |
| CSC   | 1    | 1     | Connect status changed (from FS→HS transition) |
| PE    | 2    | 1     | **Port enabled** (reset completed successfully) |
| HSP   | 9    | 1     | **High Speed Port** |
| PP    | 12   | 1     | Port power on |
| PSPD  | 27:26| 0b10  | **High Speed (480 Mbps)** |

**cotton-usb-host sees**: `Present(High480)` → second enumeration attempt.

### Key observation

The USB flash drive connects initially as Full Speed (standard USB 2.0 behavior
— HS devices present FS signaling before chirp). After port reset, the EHCI
controller negotiates High Speed via chirp protocol. Both speed transitions
succeed, but `control_transfer` fails both times.

## Enumeration Flow (cotton-usb-host `device_events_no_hubs`)

```
1. device_detect() yields Present(Full12)
2. reset_root_port(true)              ← assert reset
3. delay_ms(50)                       ← EHCI auto-completes reset + chirp
4. reset_root_port(false)             ← PR already auto-cleared by hardware
5. delay_ms(10)                       ← TRSTRCY recovery
6. control_transfer(addr=0, pkt=8, GET_DESCRIPTOR)  ← FAILS
7. EnumerationError returned
8. device_detect() yields Present(High480)  ← port changed from FS→HS
9. Steps 2-7 repeat                   ← FAILS again
```

**Note**: cotton-usb-host uses the speed from `device_detect()` for internal
bookkeeping, but does NOT pass speed to `control_transfer()`. Our driver reads
the actual speed from `port_speed()` (PORTSC1.PSPD), which correctly returns
`SPEED_HIGH` after port reset. The QH is built with the correct HS speed.

[More details →](agent_reports/cotton_enumeration_flow.md)**

## Root Cause Hypotheses

### H1: ENHOSTDISCONDETECT Set Too Early ⭐ Most Likely

Our `init()` sets `ENHOSTDISCONDETECT` in the PHY during controller initialization,
before any device connects:

```rust
// Step 10 in init():
ral::write_reg!(ral::usbphy, self.usbphy, CTRL_SET, ENHOSTDISCONDETECT: 1);
```

**USBHost_t36 (working reference for the same hardware)** sets this flag ONLY
after confirming HSP=1 in PORTSC1 during the port change ISR:

```c
// ehci.cpp line 405:
if (USBHS_PORTSC1 & USBHS_PORTSC_HSP) {
    USBPHY_CTRL_SET = USBPHY_CTRL_ENHOSTDISCONDETECT;
}
```

The i.MX RT reference manual notes:
> Do not set this bit when there is no device connected. It may cause a false
> disconnect event.

**Impact**: Setting ENHOSTDISCONDETECT during init could:
1. Interfere with the FS→HS chirp negotiation during port reset
2. Cause false disconnect events during HS operation
3. Cause PHY-level signal integrity issues that lead to transaction errors

**Why LS keyboard worked**: The "high-speed disconnect detector" only applies to
HS signaling. LS/FS operation is unaffected because those speeds use different
electrical signaling.

**Fix**: Remove `ENHOSTDISCONDETECT` from `init()`. For now, omit it entirely
to test basic HS functionality. Later, add it after confirming HSP=1 in PORTSC1
(matching USBHost_t36).

[More details on USBHost_t36](agent_reports/usbhost_t36_highspeed.md)

---

### H2: Transaction Error Due to Timing / PHY Issue

If H1 doesn't fully explain the failure, the qTD might show specific error bits:

- **XACT_ERR (bit 3)**: Device not responding (timeout, CRC, bad PID). Could
  indicate PHY misconfiguration for HS mode.
- **Halted (bit 6) with no other bits**: STALL — device rejected the request.
  Unlikely for GET_DESCRIPTOR to address 0.
- **Buffer Error (bit 5)**: DMA can't access the buffer. Would indicate a
  memory-region issue.

**Diagnosis**: Log the raw qTD token and the specific `UsbError` variant.

---

### H3: Insufficient Delay After Port Reset for HS Devices

USB 2.0 specifies TRSTRCY (reset recovery time) = 10ms (§7.1.7.5). Our
enumeration delays 10ms after `reset_root_port(false)`. But on EHCI, the
hardware auto-clears PR before our 50ms delay expires. So the total recovery
time is really `50ms - (time_for_reset_completion) + 10ms`.

USB flash drives typically need the standard 10ms recovery. This should be
sufficient.

**Check**: If errors persist after H1 fix, try increasing the recovery delay.

---

### H4: QH max_packet_size=8 for HS EP0

cotton-usb-host passes `packet_size=8` for the first GET_DESCRIPTOR (to read
the 8-byte header containing `bMaxPacketSize0`). USB 2.0 §5.5.3 says all HS
devices must support 64-byte EP0. The QH's max_packet_size=8 tells the EHCI
controller to expect 8-byte packets.

For HS, the controller expects 64-byte packets on EP0. If the device responds
with a packet size based on its actual max packet size (64 bytes), but the QH
says max_packet_size=8, the controller might misinterpret the data boundaries.

**Check**: If H1 fix resolves the issue, no action needed. Otherwise, investigate
whether HS EP0 requires max_packet_size=64 in the QH even for the initial
8-byte transfer.

---

## Diagnostic Steps

### Step 1: Log Specific Error Details

In `control_transfer()`, change the error log from:
```rust
warn!("[HC] control_transfer -> Err");
```
to:
```rust
warn!("[HC] control_transfer -> {:?}", result);
```

And in `do_control_transfer()`, add before unlink:
```rust
info!("[HC] PORTSC1=0x{:08X} speed={} char=0x{:08X}",
    ral::read_reg!(ral::usb, self.usb, PORTSC1),
    speed, characteristics);
```

**Expected output**: Error variant (Stall, ProtocolError, Timeout, etc.) and
the port/QH state that caused it.

### Step 2: Remove ENHOSTDISCONDETECT from init()

Comment out step 10 in `init()`:
```rust
// REMOVED: set only after HSP=1 confirmed, per USBHost_t36
// ral::write_reg!(ral::usbphy, self.usbphy, CTRL_SET, ENHOSTDISCONDETECT: 1);
```

### Step 3: Flash and Test

Build and flash `rtic_usb_mass_storage` with a USB flash drive connected.

**Expected outcomes after H1 fix**:
- If ENHOSTDISCONDETECT was the root cause: enumeration succeeds, MSC sector
  read proceeds.
- If not the root cause: error details from Step 1 will narrow down the issue.

---

## Action Plan

1. ✅ **Remove ENHOSTDISCONDETECT from init()** (Step 2)
2. ✅ **Add error detail logging** (Step 1) — log UsbError variant, raw qTD tokens, PORTSC1, QH characteristics
3. ✅ **Build verified** — `rtic_usb_mass_storage.hex` compiles cleanly
4. ✅ **Flashed and tested** — Round 1 results below
5. ✅ **Root cause identified** — data qTD Total Bytes exceeds wLength (see analysis)
6. ✅ **Fix applied** — cap IN data qTD Total Bytes to `min(buf.len(), setup.wLength)`
7. ✅ **Build verified** — `rtic_usb_mass_storage.hex` compiles cleanly with fix
8. ✅ **Flash and test Round 2** — enumeration succeeds, MSC interface found
9. ✅ **Demoted verbose diag logs** — `trace!`/`debug!` level to reduce log pressure
10. ✅ **Logging starvation identified** — RTIC priority starvation (see Round 3 analysis)
11. ✅ **Fix applied** — raise USB1/DMA ISR to priority 2, eliminate poll_logger task
12. ✅ **Build verified** — all 3 examples compile cleanly
13. ✅ **Flash and test Round 3** — logging starvation NOT fixed by priority change
14. ✅ **Added heartbeat LED** — PIT timer at priority 3, toggles LED every 500 ms
15. ✅ **Build verified** — `rtic_usb_mass_storage.hex` compiles cleanly
16. ✅ **Round 4 tested** — LED blinks, CPU alive, logging still broken
17. ✅ **Round 5 tested** — force `poller.poll()` from PIT ISR, still broken
18. **Round 6** — narrow down USB1 disruption cause (see diagnostic steps D1–D7)

---

## Test Results — Round 1

### Log Output

```
[INFO rtic_usb_mass_storage::app]: === imxrt-usbh: USB Mass Storage Example ===
[INFO rtic_usb_mass_storage::app]: USB2 PLL locked
[INFO rtic_usb_mass_storage::app]: VBUS power enabled
[INFO rtic_usb_mass_storage::app]: USB host controller initialised
[INFO rtic_usb_mass_storage::app]: USB_OTG2 ISR installed (NVIC priority 0xE0)
[INFO rtic_usb_mass_storage::app]: Entering device event loop...
[INFO imxrt_usbh::host]: [HC] DeviceDetect: status change  PORTSC1=0x10001803
[INFO imxrt_usbh::host]: [HC] control xfer: addr=0 pkt=8 PORTSC1=0x18001207 speed=2 char=0xF0086000 caps=0x40000000
[WARN imxrt_usbh::host]: [HC] control xfer FAILED: err=Stall setup_tok=0x80000E00 data_tok=0x000A0D40 status_tok=0x80008C80 PORTSC1=0x18001207
[WARN imxrt_usbh::host]: [HC] control_transfer -> Err(Stall)
[WARN rtic_usb_mass_storage::app]: DeviceEvent::EnumerationError  hub=0 port=1
[INFO imxrt_usbh::host]: [HC] DeviceDetect: status change  PORTSC1=0x18001207
[INFO imxrt_usbh::host]: [HC] control xfer: addr=0 pkt=8 PORTSC1=0x18001207 speed=2 char=0xF0086000 caps=0x40000000
[WARN imxrt_usbh::host]: [HC] control xfer FAILED: err=Stall setup_tok=0x80000E00 data_tok=0x000A0D40 status_tok=0x80008C80 PORTSC1=0x18001207
[WARN imxrt_usbh::host]: [HC] control_transfer -> Err(Stall)
[WARN rtic_usb_mass_storage::app]: DeviceEvent::EnumerationError  hub=0 port=1
```

### Token Decode

#### PORTSC1 = 0x18001207

- CCS=1, CSC=1 (bit 1, but already cleared by detect), PE=1, HSP=1 (bit 9), PP=1, PSPD=2 (HS)
- **Port is enabled, High Speed** ✅

#### char = 0xF0086000 (QH Characteristics)

| Field | Bits | Value | Correct? |
|-------|------|-------|----------|
| RL    | 31:28| 15    | ✅ NAK retry limit |
| C     | 27   | 0     | ✅ Not FS/LS control with non-64 MPS |
| MPS   | 26:16| 8     | ⚠️ Should be 64 for HS EP0 (but cotton passes 8) |
| H     | 15   | 0     | ✅ Not head of reclamation |
| DTC   | 14   | 1     | ✅ Toggle from qTD |
| EPS   | 13:12| 2     | ✅ High Speed |
| EP    | 11:8 | 0     | ✅ Endpoint 0 |
| Addr  | 6:0  | 0     | ✅ Address 0 |

#### setup_tok = 0x80000E00 — **SETUP qTD completed OK**

| Field | Value | Meaning |
|-------|-------|---------|
| DT    | 1     | DATA1 (after completion) |
| Total Bytes | 0 | All 8 bytes transferred ✅ |
| CERR  | 3     | Full retries remaining ✅ |
| PID   | SETUP | ✅ |
| Status| 0x00  | No errors, not Active, not Halted ✅ |

**SETUP phase succeeded** — device accepted the 8-byte setup packet.

#### data_tok = 0x000A0D40 — **⭐ DATA qTD STALLED**

| Field | Value | Meaning |
|-------|-------|---------|
| DT    | 0     | Data toggle (hardware-modified) |
| Total Bytes | **10** | 10 remaining out of 18 → **8 bytes received** |
| CERR  | 3     | Not decremented → confirms STALL (not XACT error) |
| PID   | IN    | ✅ |
| Status| 0x40  | **Halted (bit 6)** only — no XACT/Babble/Buffer errors |

**Halted + CERR=3 + no other error bits = STALL handshake from device.**

#### status_tok = 0x80008C80 — Status qTD never executed

- Active=1 (bit 7 set in 0x80) → hardware never reached this qTD
- Expected: data qTD halted, so controller stopped before status phase

### Root Cause Analysis — CONFIRMED: New Hypothesis H5

**H5: Data qTD Total Bytes exceeds wLength → extra IN tokens → STALL**

cotton-usb-host's initial `GET_DESCRIPTOR(Device)` call passes:
- `wLength = 8` in the SetupPacket (request only 8 bytes)
- `DataPhase::In(&mut [0u8; 18])` — an **18-byte** buffer

Our `do_control_transfer` was setting data qTD `Total Bytes = buf.len() = 18`.

**What happens on the bus:**
1. SETUP phase: host sends 8-byte setup packet → device ACKs ✅
2. First IN token: device sends 8 bytes of descriptor (DATA1) → received OK
3. Remaining = 18 - 8 = 10. QH MPS=8, so 8 bytes = full packet. Not short → EHCI sends another IN.
4. Second IN token: device has no more data (wLength was 8) → **device STALLs**
5. EHCI halts the data qTD → our code maps Halted+no-errors to `UsbError::Stall`

**Why this worked with the LS keyboard (Phase 2a):**
With LS keyboard, cotton-usb-host also passes an 18-byte buffer with wLength=8.
The LS keyboard's EP0 MPS = 8. After receiving 8 bytes, remaining = 10, and
8 bytes = full packet. The same situation should occur... but LS/FS use different
EHCI transaction handling. Possibly the LS keyboard returned all 18 bytes of its
descriptor regardless of wLength (some non-compliant devices do this), or the
timing was different enough to avoid the issue.

**Fix applied:** Cap the data IN qTD `Total Bytes` to `min(buf.len(), setup.wLength)`:
```rust
let wlength = setup.wLength as usize;
data_len = if wlength > 0 && wlength < buf.len() {
    wlength
} else {
    buf.len()
};
```

This ensures the EHCI controller requests exactly what the setup packet tells the
device to send, avoiding extra IN tokens.

---

## Test Results — Round 2

### Log Output (after wLength cap fix + ENHOSTDISCONDETECT fix)

```
[INFO imxrt_usbh::host]: [HC] DeviceDetect: status change  PORTSC1=0x10001803
[INFO imxrt_usbh::host]: [HC] control xfer: addr=0 pkt=8 ...
[INFO imxrt_usbh::host]: [HC] control_transfer -> Ok(8)          ← GET_DESCRIPTOR(Dev, 8)
[INFO imxrt_usbh::host]: [HC] control xfer: addr=0 pkt=64 ...
[INFO imxrt_usbh::host]: [HC] control_transfer -> Ok(18)         ← GET_DESCRIPTOR(Dev, 18)
[INFO imxrt_usbh::host]: [HC] control xfer: addr=0 pkt=64 ...
[INFO imxrt_usbh::host]: [HC] control_transfer -> Ok(0)          ← SET_ADDRESS(1)
[INFO rtic_usb_mass_storage::app]: DeviceEvent::Connect  addr=1  VID=1908 PID=1320 class=0
[INFO imxrt_usbh::host]: [HC] control_transfer -> Ok(9)          ← GET_DESCRIPTOR(Config, 9)
[INFO imxrt_usbh::host]: [HC] control_transfer -> Ok(32)         ← GET_DESCRIPTOR(Config, 32)
[INFO rtic_usb_mass_storage::app]: Found MSC interface: class=8 sub=6 proto=0x50 bulk_in=2 (mps=512) bulk_out=1 (mps=512)
[INFO imxrt_usbh::host]: [HC] control xfer: addr=<TRUNCATED — log buffer overflow>
```

Second enumeration (after apparent disconnect/reconnect):
```
[INFO imxrt_usbh::host]: [HC] control_transfer -> Ok(8)
...
[INFO rtic_usb_mass_storage::app]: Found MSC interface ...
[INFO imxrt_usbh::host]: [HC] control_transfer -> Ok(0)          ← SET_CONFIGURATION
[INFO imxrt_usbh::host]: [HC] control_transfer -> Ok(9)          ← ?
[INFO imxrt_usbh::host]: [HC] control xfer: addr=1<TRUNCATED — no further output>
```

### Analysis

**Enumeration fully working!** ✅

- GET_DESCRIPTOR(Device, 8 bytes) → Ok(8) ✅ (wLength cap fix working)
- GET_DESCRIPTOR(Device, 18 bytes) → Ok(18) ✅
- SET_ADDRESS → Ok(0) ✅
- GET_DESCRIPTOR(Config, 9 + 32 bytes) → ✅
- MSC interface found: bulk_in=2 (512), bulk_out=1 (512) ✅
- SET_CONFIGURATION → Ok(0) ✅
- Flash drive activity LED flashing (device responding to USB traffic) ✅

**Remaining issue: Log buffer overflow.**

The verbose per-transfer diagnostic logs fill the log buffer faster than the
USB-CDC serial poller can drain it. The `usb_task` and `poll_logger` are both
at RTIC priority 1 — during intensive control/bulk transfer sequences, the
async executor may not yield to the log poller often enough.

**Fix applied:** Demoted diagnostic logs to `trace!`/`debug!` level:
- Pre-transfer details (`control xfer: addr=...`) → `trace!`
- `control_transfer -> Ok(N)` → `trace!`
- Detailed error tokens → `debug!`
- `DeviceDetect: status change` → `debug!`
- Interrupt pipe diag logs → `debug!`
- Only `control_transfer -> Err(...)` remains at `warn!`

---

## Logging Issue — RESOLVED

**Root cause**: Log buffer overflow from TRACE-level messages, not USB1
disruption. The 1024-byte bbqueue buffer filled faster than the USB CDC
backend could drain during rapid USB transfer sequences. Messages were
silently truncated/dropped, appearing as if logging "died."

**Fix**: `log::set_max_level(log::LevelFilter::Debug)` in the example's
`init()` to filter out TRACE messages. Heartbeat logging in the PIT ISR
confirmed USB1 was healthy throughout.

Details in [usb_logging_debugging.md](./usb_logging_debugging.md).

---

## Test Results — Round 3: Bulk Transfer Errors

### Log Output

```
DeviceEvent::Connect  addr=1  VID=1908 PID=1320 class=0
Found MSC interface: bulk_in=2 (mps=512) bulk_out=1 (mps=512)
Opening bulk endpoints...
Sending CBW READ(10) LBA=0...
CBW sent: 31 bytes
[HC] bulk IN addr=1 ep=2 len=512 -> Err(Overflow)
Data IN failed: Overflow
```

### Analysis

- **CBW OUT succeeded** (31 bytes) ✅ — bulk OUT transfer works
- **Data IN failed with `Overflow`** ❌ — 512-byte bulk IN transfer fails

`UsbError::Overflow` is mapped from qTD status bit 5 (Buffer Error) in
`map_qtd_error()`. This typically means the EHCI controller couldn't access
the data buffer via DMA — either the buffer address is wrong, the buffer
crosses a 4K page boundary in a way the controller can't handle, or a cache
coherency issue is corrupting the qTD/buffer pointers.

### Hypotheses

#### B1: Data buffer address issue (stack vs static)

The 512-byte `SECTOR_BUF` is declared `static mut` in the example, which
should place it in a DMA-accessible memory region. However, if the linker
places it in DTCM (tightly-coupled memory), the EHCI DMA engine may not be
able to access it.

**Check**: Log the address of `SECTOR_BUF` and verify it's in a DMA-accessible
region (typically OCRAM at 0x2020_0000–0x2027_FFFF or FlexRAM at
0x2000_0000–0x2007_FFFF).

#### B2: qTD buffer pointer setup for large transfers

The qTD has 5 buffer pointers (page-aligned) for scatter-gather. For a
512-byte transfer starting at a non-page-aligned address, the data may span
two 4K pages, requiring two buffer pointers. If `TransferDescriptor::init()`
only sets the first buffer pointer, transfers >4096 bytes or those crossing a
page boundary will fail.

**Check**: Review `TransferDescriptor::init()` to verify it correctly sets up
multiple buffer pointers for transfers that cross page boundaries.

#### B3: Cache coherency on the IN data buffer

For a bulk IN transfer, the EHCI DMA writes data to RAM. If the CPU cache
contains stale data for those addresses, the cache invalidation sequence may
be wrong — e.g., cleaning (writing back) dirty cache lines over the DMA data,
or not invalidating at all before reading.

**Check**: Verify the cache operation sequence for bulk IN buffers.

### Diagnosis

Enhanced error logging revealed:
```
token=0x02008D50 buf0=0x20203000 char=0xF0402201 overlay=0x02008D50
```

- **token**: Total Bytes = 512 (no data transferred), Status = Halted + **Babble** (bit 4)
- **char**: MPS bits 26:16 = 0x040 = **64** ❌ — should be 512 for HS bulk
- **buf0**: 0x20203000 (OCRAM2, DMA-accessible) ✅

### Root Cause: cotton-usb-host hardcodes packet_size=64

cotton-usb-host v0.2.1 `usb_bus.rs` line 893:
```rust
self.driver.bulk_in_transfer(
    ep.usb_address,
    ep.endpoint,
    64, // @TODO max packet size     ← HARDCODED
    data,
    transfer_type,
    &ep.data_toggle,
)
```

This works for RP2040 (Full Speed only, max bulk MPS=64) but is wrong for
High Speed where USB 2.0 §5.8.3 mandates wMaxPacketSize=512 for bulk.

The QH was built with MPS=64. When the HS device sent a 512-byte bulk packet,
the EHCI controller detected babble (packet exceeds MPS).

### Fix Applied

Workaround in `do_bulk_transfer`: override `packet_size` to 512 when the port
speed is High Speed:

```rust
let actual_packet_size = if speed == SPEED_HIGH && packet_size < 512 {
    512
} else {
    packet_size
};
```

Also updated ZLP calculation to use `actual_packet_size`.

---

## Test Results — Round 4: MSC Sector Read SUCCESS

```
DeviceEvent::Connect  addr=1  VID=1908 PID=1320 class=0
Found MSC interface: bulk_in=2 (mps=512) bulk_out=1 (mps=512)
Opening bulk endpoints...
Sending CBW READ(10) LBA=0...
CBW sent: 31 bytes
Data received: 512 bytes
Sector 0: 33 c0 8e d0 bc 00 7c 8e c0 8e d8 be 00 7c bf 00
CSW: status=0 (success)
```

**Phase 2c bulk transfers fully working.** ✅

- CBW OUT (31 bytes) → success ✅
- Data IN (512 bytes) → success ✅
- CSW IN (13 bytes) → status=0 (success) ✅
- Sector 0 data reads correctly (starts with x86 boot code `33 c0 8e d0`) ✅
- Heartbeat ticks continue after transfer ✅
- Device re-enumerates (FS→HS transition) and succeeds on second attempt too ✅

---

## Summary of All Issues Found and Fixed in Phase 2c

| Issue | Root Cause | Fix |
|-------|-----------|-----|
| Control transfer STALL on HS device | Data qTD Total Bytes = buf.len() exceeded wLength → extra IN tokens → STALL | Cap to `min(buf.len(), setup.wLength)` |
| ENHOSTDISCONDETECT false disconnects | Set during init before any device connected | Removed from init (set after HSP=1 later) |
| Logging appeared to "die" | TRACE-level messages overflowed 1024-byte log buffer | `log::set_max_level(Debug)` in example |
| Bulk IN babble error | cotton-usb-host hardcodes packet_size=64; HS bulk requires 512 | Override to 512 when speed=HS |

## Proposed Next Steps

1. **Clean up diagnostic logging** — remove the enhanced error diagnostics
   added during debugging (raw token/char/overlay dumps). Keep the concise
   error-type log.

2. **Remove heartbeat logging** — the PIT heartbeat served its diagnostic
   purpose. Keep the LED blink but remove the periodic log message to reduce
   noise.

3. **Handle the FS→HS double-enumeration** — the device enumerates twice
   (once at FS before chirp, once at HS after). This is harmless but wasteful.
   Could be addressed by ignoring the first connect if PORTSC shows PE=0
   (port not yet enabled/reset). This is a cotton-usb-host behavior, so may
   need investigation into whether we can suppress the first detection.

4. **Update Overview.md** — mark Phase 2c as COMPLETE, update status and
   next phase description.

5. **Commit the working code** — snapshot this milestone before moving to
   Phase 3 (interrupt handling polish, cache coherency audit, robustness).