
## Logging Starvation

### Symptoms Observed

The reduced-logging build revealed a deeper problem: **log output is unreliable.**

- **Scenario A** (nothing attached at boot): Logs appear up to "Entering device
  event loop…", but when a thumb drive is then plugged in (hot-plug), **no further
  log output appears** even though the drive's activity LED flashes (proving USB
  transfers are occurring).

- **Scenario B** (thumb drive attached at boot): Logs appear for a while (some
  enumeration messages), then stop.

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

The behavior is **not repeatable** — sometimes more logs appear, sometimes fewer.

### Root Cause: RTIC Priority Starvation

All tasks were at RTIC priority 1:

```
usb_task          (software task, priority 1 → BOARD_SWTASK0 dispatcher)
poll_logger       (software task, priority 1 → BOARD_SWTASK0 dispatcher)
usb1_interrupt    (hardware task, priority 1 → NVIC priority 0xF0)
dma_interrupt     (hardware task, priority 1 → NVIC priority 0xF0)
```

On ARM Cortex-M, **same-priority interrupts cannot preempt each other.** When
the BOARD_SWTASK0 dispatcher ISR is running (polling `usb_task`), the USB1 and
DMA hardware interrupts — which trigger log flushing — are blocked:

1. `usb_task` enters `delay_ms(50)` which busy-waits via `cortex_m::asm::delay()`
   — the dispatcher ISR is blocked for 50ms. USB1 ISR can't fire.
2. Between transfers, `usb_task` may cycle rapidly through Pending→Ready→Pending
   without the dispatcher ever truly returning. USB1 ISR can't fire.
3. The log buffer fills but never drains → overflow → lost messages.

**Why the keyboard example worked**: After enumeration, the HID keyboard enters
`pipe.next().await` which returns Pending for ~10ms (the interrupt poll interval).
During those gaps, the dispatcher exits and USB1 can fire. Mass storage does
rapid back-to-back bulk transfers with no natural pauses.

**Why USB2 ISR was unaffected**: The USB2 ISR is manually installed at NVIC
priority 0xE0, which is higher priority than the RTIC priority-1 dispatcher
(0xF0). USB2 ISR can always preempt the dispatcher.

### Fix Applied: Raise Logging Priority

Eliminated the `poll_logger` software task. Moved logging directly into the
USB1 and DMA hardware ISRs at **priority 2**:

```rust
#[task(binds = BOARD_USB1, shared = [poller], priority = 2)]
fn usb1_interrupt(mut cx: usb1_interrupt::Context) {
    cx.shared.poller.lock(|poller| poller.poll());
}

#[task(binds = BOARD_DMA_A, shared = [poller], priority = 2)]
fn dma_interrupt(mut cx: dma_interrupt::Context) {
    cx.shared.poller.lock(|poller| poller.poll());
}
```

**Priority layout after fix:**

| Component | RTIC Priority | NVIC Priority | Can preempt USB task? |
|-----------|--------------|---------------|----------------------|
| USB2 ISR (host) | — (manual) | 0xE0 | ✅ Yes |
| usb1_interrupt | 2 | ~0xE0 | ✅ Yes |
| dma_interrupt | 2 | ~0xE0 | ✅ Yes |
| usb_task | 1 | 0xF0 | — (is the task) |
| idle | — (thread) | lowest | — |

**Advantages:**
- No second RTIC dispatcher needed (eliminated software `poll_logger` task)
- USB1/DMA ISRs preempt the USB task to flush logs immediately
- `lock()` on the `poller` shared resource is effectively free (ceiling = 2,
  both accessors at priority 2)

**Applied to all three examples:** `rtic_usb_enumerate`, `rtic_usb_hid_keyboard`,
`rtic_usb_mass_storage`.

---

## Test Results — Round 3

Priority-2 logging fix **did not resolve the logging starvation.** Same symptoms
persist — logs stop after "Entering device event loop…" on hot-plug.

This rules out simple RTIC priority starvation as the sole cause.

---

## Rounds 4–5: Heartbeat LED + Force-Flush Diagnostic

### Round 4: LED at priority 3

Added a PIT timer interrupt at **priority 3** (highest in the app) that blinks
the onboard LED every 500 ms.

**Result: LED blinks.** CPU is alive. Not stuck in a hard fault or infinite loop.

### Round 5: Force `poller.poll()` from PIT ISR

Added `cx.shared.poller.lock(|poller| poller.poll())` to the PIT ISR (priority 3).
This force-drains the log buffer every 500 ms from the highest-priority context
in the system.

**Result: Logging is STILL broken.** Same symptoms.

### Conclusions So Far

| What we know | Evidence |
|---|---|
| CPU is alive, NVIC works | LED blinks at priority 3 |
| `poller.poll()` doesn't hang | LED continues blinking after poll() returns |
| The problem is NOT RTIC priority starvation | Raising ISR priorities didn't help |
| The problem is NOT the poller not being called | Force-polling from PIT didn't help |
| The USB1 CDC serial connection itself is disrupted | `poll()` completes but no data reaches PC |
| Disruption correlates with USB2 host activity | Logging works before device connect |

Something about USB2 host controller activity is breaking the USB1 CDC serial
connection. The PC either stops receiving data or USB1 drops off the bus entirely.

---

## Round 6 - Narrowing Down the USB1 Disruption
  
### Hypotheses

#### H1: Cache coherency — USB2 cache ops corrupt USB1 DMA buffers ⭐

Our USB2 host driver performs frequent `clean_invalidate_dcache_by_address()` on
QH, qTD, and data buffers. On Cortex-M7, cache operations work at **32-byte
cache-line granularity**. If a USB1 CDC driver buffer happens to share a cache
line with any USB2 DMA structure, our cache invalidation could:

- Discard dirty USB1 data that hasn't been written to RAM yet
- Overwrite RAM data that USB1 DMA just wrote

This would corrupt USB1 driver state and is directly correlated with USB2
activity.

**Likelihood**: High — this is the #1 class of bug on cached Cortex-M7 systems.

#### H2: USB2 EHCI DMA writing to incorrect addresses

A misconfigured QH or qTD could cause the EHCI DMA engine to write to
arbitrary memory. If it overwrites USB1 driver structures, RTIC state, or the
log buffer, USB1 would break.

**Likelihood**: Medium — we verified QH/qTD setup works for control transfers,
but bulk transfer QH setup is newer and less tested.

#### H3: BASEPRI left elevated

If `poller.poll()` or the USB1 CDC driver encounters a fault inside an RTIC
`lock()` block, BASEPRI could be stuck at the ceiling value (0xD0 for the
priority-3 PIT ISR accessing the ceiling-3 `poller` resource). This would
mask all interrupts at priority ≤ 3 (i.e., everything except hard fault).

But: the LED blinks (PIT ISR at priority 3), and BASEPRI=0xD0 would block
priority 3 (NVIC ≥ 0xD0). So BASEPRI is either 0x00 (normal) or higher than
0xD0 (which would only block priority 1–2). The LED test doesn't distinguish
BASEPRI=0x00 from BASEPRI=0xE0.

If BASEPRI=0xE0: USB1 ISR (priority 2 = NVIC 0xE0) is blocked, USB2 ISR
(NVIC 0xE0) is blocked, USB task (priority 1 = NVIC 0xF0) is blocked. But
`poll()` IS being called from PIT ISR (priority 3) and still doesn't produce
output. So BASEPRI alone doesn't explain the problem — even with poll() being
called, data isn't getting out. Unless poll() needs USB1 interrupts to complete
the CDC transaction.

**Likelihood**: Low-Medium — could be a contributing factor but doesn't fully
explain the symptoms.

#### H4: USB1 device controller drops off the bus

If USB1 loses its connection to the PC (e.g., the USB device controller enters
an error state, or the PHY gets disrupted), the COM port would vanish from the
PC. `poller.poll()` would try to send data but the USB1 hardware wouldn't have
an active host connection to send to.

Possible triggers: shared power rail noise, shared clock domain issues,
electrical coupling between USB1 and USB2 on the PCB.

**Likelihood**: Low — USB1 and USB2 are separate controllers with separate PHYs.

#### H5: USB1 CDC needs its ISR to complete transactions

The USB1 CDC serial driver is interrupt-driven. A CDC serial write involves:
1. Application writes to log buffer
2. `poller.poll()` copies from log buffer into a USB endpoint buffer
3. USB1 device controller transmits the endpoint buffer when the PC sends an
   IN token
4. USB1 hardware fires an interrupt on completion → ISR processes the event
5. Next `poll()` call can queue more data

If step 4 is blocked (USB1 ISR can't fire because BASEPRI masks it, or USB1
hardware doesn't generate the interrupt), step 5 never makes progress.
`poller.poll()` might only be able to push one endpoint-buffer's worth of data
before it stalls waiting for the previous transfer to complete.

This would explain why force-polling from PIT doesn't help: the poll() call
fills the USB endpoint buffer, but the completion interrupt never fires, so the
next poll() has nowhere to put data.

**Likelihood**: Medium-High — this is a plausible mechanism, especially if
BASEPRI is elevated enough to block USB1 ISR (0xE0) but not PIT (0xD0).

### Diagnostic Steps (Ordered by Effort / Informativeness)

#### D1: Check if COM port disappears on PC (no code change)

Have the user watch the COM port in Device Manager or `ls /dev/tty*` while
reproducing the problem. Does the COM port vanish when logging stops?

- **COM port vanishes**: USB1 has electrically disconnected → H4 (PHY/power issue)
- **COM port stays**: USB1 is connected but the CDC driver is stuck → H3/H5

##### Result:

The COM port does not disappear. The serial monitor can be disconnected and reconnected after logging stops, so the underlying port is still there.

#### D2: Binary search — isolate the trigger point

Progressively remove USB2 activity to find what triggers USB1 disruption:

| Test | What to change | If logging works |
|------|---------------|-----------------|
| D2a | Init USB2 host but comment out `usb_task::spawn()` | USB2 init breaks USB1 |
| D2b | Spawn usb_task, but have it only wait on `device_detect()` (don't plug anything in) | Something about having the async task running breaks USB1 |
| D2c | Allow enumeration, but skip bulk transfers (return after `configure()`) | Bulk transfers break USB1 |
| D2d | Full example as-is | (baseline — known broken) |

If D2a already breaks logging, the problem is in the init sequence itself
(unlikely but rules out a large surface area). If D2a works but D2d doesn't,
we narrow down to the code path between them.

#### D3: Read BASEPRI from PIT ISR

In the PIT ISR, read `cortex_m::register::basepri::read()` and signal via
LED blink pattern:

- **Normal blink** (500 ms): BASEPRI == 0 (all interrupts enabled)
- **Fast blink** (100 ms): BASEPRI != 0 (something left it elevated)

This directly tests H3.

#### D4: Switch to LPUART logging (requires serial adapter)

Change logging backend:
```rust
const BACKEND: board::logging::Backend = board::logging::Backend::Lpuart;
```

This uses the LPUART2 TX pin (Teensy 4.1 pin 14 = `GPIO_AD_B1_02`) instead
of USB CDC. Requires a USB-to-serial adapter connected to that pin.

- **LPUART logging works**: Confirms USB1 is being disrupted; all USB1-related
  hypotheses are live.
- **LPUART logging also breaks**: The problem is in the log buffer or software
  layer, not USB1 hardware.

This is the single most informative test but requires extra hardware.

#### D5: Disable all cache operations

Add a compile-time flag to disable every `cache::clean_invalidate_dcache_by_address`
and `cache::invalidate_dcache_by_address` call. USB2 transfers will likely fail
(DMA coherency issues), but if USB1 logging survives, we've confirmed H1.

#### D6: Check USB2 DMA buffer addresses for cache-line conflicts

In the PIT ISR (or at init time), log the addresses of:
- `STATICS.qh_pool` (QH array)
- `STATICS.qtd_pool` (qTD array)
- `STATICS.frame_list`
- `STATICS.recv_bufs`

And check whether any of them share a 32-byte cache line with USB1 driver
structures. The USB1 driver's buffers are in the `board` and `imxrt-hal`
crates — their addresses can be read at runtime.

This requires logging to work first (chicken-and-egg), so it should be done
with LPUART or at init time before USB2 starts, or signaled via LED.

#### D7: Align all USB2 DMA structures to cache-line boundaries (32 bytes)

Ensure every DMA-visible structure (`QueueHead`, `TransferDescriptor`,
`FrameList`, `recv_bufs`, data buffers) is 32-byte aligned AND sized to a
multiple of 32 bytes. This prevents cache operations on USB2 structures from
affecting adjacent memory.

Currently:
- `QueueHead`: 64-byte aligned ✅ (EHCI requirement, exceeds cache line)
- `TransferDescriptor`: 32-byte aligned ✅ (EHCI requirement = cache line)
- `FrameList`: 4096-byte aligned ✅
- `recv_bufs`: **no explicit alignment** ⚠️ — `[[u8; 64]; N]` has alignment 1
- `UsbStatics` as a whole: inherits max alignment from fields, but padding
  between fields may leave gaps that share cache lines with other statics

This is the most likely fix for H1 and can be applied defensively.

### Recommended Order

1. **D1** (free — no code changes, just check COM port on PC)
2. **D2a** (trivial — comment out one line)
3. **D3** (small code change — read BASEPRI, change LED pattern)
4. **D5** (medium — disable cache ops)
5. **D7** (medium — align DMA structures)
6. **D4** (requires serial adapter hardware)
7. **D6** (requires working logging or alternate output)

---

## Round 7: D2a — USB2 init only, no usb_task

### Changes

- Added heartbeat log message in PIT ISR: logs `[heartbeat] tick #N` every 4th
  beat (every 2 seconds) so we have continuous proof that logging is alive even
  without the USB task running.
- Commented out `usb_task::spawn(host)` in `idle` — USB2 host controller is
  fully initialised (PLL, PHY, EHCI, NVIC ISR) but no async task is running
  and no USB transfers occur.

### Expected Behaviour

Initial log messages should appear (banner, PLL, VBUS, host init, ISR install),
followed by heartbeat ticks every 2 seconds. No USB device interaction occurs.

- **If heartbeat ticks appear indefinitely**: USB2 init alone does NOT break
  USB1 logging. Proceed to D2b (re-enable spawn but only wait on device_detect
  without plugging anything in).
- **If heartbeat ticks stop after a few seconds**: USB2 init itself disrupts
  USB1. Investigate the init sequence — likely cache ops during init, or the
  USB2 ISR firing spuriously and causing problems.

### Result

**Logging continues.** Heartbeat ticks appear indefinitely. USB2 init alone
does NOT break USB1 logging. Proceed to D2b.

---

## Round 8: D2b — Spawn usb_task, wait on device_detect only

### Changes

- Re-enabled `usb_task::spawn(host)`.
- Gutted `usb_task` body: creates `UsbBus`, enters `device_events_no_hubs()`
  stream loop, but only logs connect/disconnect events — no descriptor walks,
  no configure, no bulk transfers.
- **Do NOT plug in any USB device.** The task should sit idle in
  `device_detect().await` with the heartbeat ticking.

### Expected Behaviour

Heartbeat ticks should appear every 2 seconds. The USB task is spawned and
waiting on the async schedule for a port-status-change interrupt that never
comes.

- **If heartbeat ticks continue**: The async task + USB2 ISR idling is fine.
  Proceed to D2c (plug in a device, allow enumeration but skip bulk transfers).
- **If heartbeat ticks stop**: Something about having the USB2 async schedule
  active (or the ISR firing spuriously) disrupts USB1.

### Result

**Logging continues.** Heartbeat ticks appear after enumeration completes.
The async task + USB2 ISR + control transfers during enumeration do NOT break
USB1 logging. Proceed to D2c.

---

## Round 9: D2c — Enumerate + configure, skip bulk transfers

### Changes

- Restored descriptor walking (`get_configuration` + `MscFinder`) and
  `bus.configure()`.
- After configure succeeds, log a message and `continue` — no bulk endpoint
  opens, no CBW/CSW transfers.

### Expected Behaviour

Full enumeration + SET_CONFIGURATION, then heartbeat ticks continue.

- **If heartbeat ticks continue**: Configure is fine. The problem is bulk
  transfers. Proceed to isolate which bulk operation triggers the failure.
- **If heartbeat ticks stop**: Something in `get_configuration` or `configure`
  (which are just control transfers) breaks USB1 — unexpected given D2b passed.

### Result

**Logging continues.** Heartbeat ticks appear after enumeration + configure.
The "Found MSC" message was truncated by log buffer overflow from TRACE-level
control transfer messages — USB1 was never actually broken.

### Root Cause Identified: Log Buffer Overflow

The 1024-byte `bbqueue` log buffer overflows when TRACE-level messages from
rapid USB control transfers fill it faster than the USB1 CDC backend can drain.
Messages are silently truncated/dropped, giving the appearance that logging
"died." The heartbeat diagnostic proved USB1 was healthy the whole time.

**Fix:** Set `log::set_max_level(log::LevelFilter::Debug)` in init to filter
out TRACE messages. Full bulk transfer code re-enabled.

---

## Round 10: Full bulk transfer test with DEBUG log level

### Changes

- Added `log::set_max_level(log::LevelFilter::Debug)` in `init` to suppress
  TRACE-level messages that overflow the log buffer.
- Restored full `usb_task` body: descriptor walk, configure, open bulk
  endpoints, CBW/data/CSW transfer sequence.
- Heartbeat logging still active for continued monitoring.

### Expected Behaviour

Full MSC sector read should succeed:
```
DeviceEvent::Connect  addr=1  VID=1908 PID=1320 class=0
Found MSC interface: bulk_in=N (mps=512) bulk_out=M (mps=512)
Opening bulk endpoints...
Sending CBW READ(10) LBA=0...
CBW sent: 31 bytes
Data received: 512 bytes
Sector 0: xx xx xx xx ...
CSW: status=0 (success)
```

Heartbeat ticks should continue throughout.

### Result

**Bulk IN fails with `Overflow`.** CBW OUT succeeds (31 bytes) but the 512-byte
data IN phase returns `UsbError::Overflow`. Heartbeat ticks continue — logging
is fully functional now.

**Logging problem resolved.** Root cause was log buffer overflow from TRACE-level
messages, not USB1 disruption. Fix: `log::set_max_level(Debug)`.

Debugging continues in [phase2c_debugging.md](./phase2c_debugging.md).
