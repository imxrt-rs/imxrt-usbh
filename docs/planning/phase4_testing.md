# Phase 4: Testing and Validation

**Estimated effort**: 3-5 days  
**Key milestone**: All device types working  
**Status**: Mostly complete — steps 1–8, 11 validated on hardware. Steps 9–10 not yet systematically tested.

## 4.1 Compile-Time Validation — ✅ COMPLETE

Implemented in `src/ehci.rs` as `const` assertions:

- [x] `assert!(core::mem::size_of::<QueueHead>() == 64)`
- [x] `assert!(core::mem::align_of::<QueueHead>() == 64)`
- [x] `assert!(core::mem::size_of::<TransferDescriptor>() == 32)`
- [x] `assert!(core::mem::align_of::<TransferDescriptor>() == 32)`
- [x] `assert!(core::mem::size_of::<FrameList>() == 4096)`
- [x] `assert!(core::mem::align_of::<FrameList>() == 4096)`
- [ ] Verify `#[repr(C)]` field offsets match EHCI spec (not yet done — would use `core::mem::offset_of!` or `memoffset` crate)
- [x] Pool allocation/deallocation — tested implicitly through working control, bulk, and interrupt transfers
- [x] Cache utility functions — tested implicitly through working DMA transfers

**Note**: Most testing requires actual hardware — the USB controller cannot be meaningfully mocked.

## 4.2 Hardware Bring-Up (Incremental) — Steps 1–8 ✅ COMPLETE

Test in this order, each step building on the previous:

1. **[x] Clock and PHY init** — verified: no hard faults, USB PLL locks, USBPHY2 status OK (phase 2a debugging step 1)
2. **[x] Host mode entry** — verified: `USBMODE[CM]` = host mode, `USBCMD[RS]` set (phase 2a debugging step 2)
3. **[x] Device detection** — verified: LS keyboard detected (CCS=1, PSPD=1), HS flash drive detected (phase 2a/2c)
4. **[x] Port reset** — verified: `PORTSC1[PE]` set after reset, speed negotiation works (phase 2a debugging step 3)
5. **[x] First control transfer** — verified: `GET_DESCRIPTOR(Device)` to address 0 succeeds (phase 2a debugging step 5)
6. **[x] Device enumeration** — verified: full `SET_ADDRESS` + `GET_DESCRIPTOR` + `SET_CONFIGURATION` via `UsbBus` (phase 2a, VID=045e PID=00db)
7. **[x] Bulk transfers** — verified: USB mass storage sector 0 read via SCSI READ(10) over BOT protocol (phase 2c)
8. **[x] Interrupt transfers** — verified: HID keyboard input received via interrupt pipe streaming (phase 2b)
9. **[ ] Hot-plug/unplug** — not yet systematically tested. Device detection stream handles connect/disconnect events, but robustness of repeated plug cycles is untested.
10. **[ ] Low-speed device** — LS keyboard works for enumeration, but LS-specific timing has not been stress-tested independently
11. **[x] Hub support** — validated. Hub detected, downstream devices enumerated and data transfers work. Serial processing (one device at a time) is a cotton-usb-host architectural limitation, not a driver bug. See §4.6.

## 4.3 Example Applications — ✅ COMPLETE

All planned examples exist and have been validated on hardware:

- [x] `examples/rtic_usb_enumerate.rs` — RTIC app that enumerates a USB device and prints descriptors via LPUART. Uses `board` crate BSP, `imxrt-log` for logging, demonstrates full init sequence.
- [x] `examples/rtic_usb_hid_keyboard.rs` — reads HID keyboard input (uses `cotton-usb-host-hid`). Validated on hardware with Microsoft Keyboard (VID=045e PID=00db).
- [x] `examples/rtic_usb_mass_storage.rs` — reads USB flash drive sector 0 (uses `cotton-usb-host-msc` / `cotton-scsi`). Validated on hardware.
- [ ] Document hardware setup: Teensy 4.1 USB2 host port wiring (5-pin header), power requirements, VBUS load switch control. Currently spread across planning docs but not in a user-facing document.

Additional examples (not originally planned):
- `examples/hal_logging.rs` — basic logging test (defmt/log)
- `examples/hal_usb_host_init.rs` — minimal USB host init (pre-RTIC)
- `examples/rtic_heartbeat.rs` — RTIC baseline with LED blink

## 4.4 Outstanding Items

| Item | Priority | Notes |
|------|----------|-------|
| Hub disconnect recovery (4.7) | **Active** | Fix implemented, ready for hardware test |
| Hub support test (4.2 #11) | **Done** | ✅ Validated — see §4.6 analysis |
| Multi-device concurrent use | Low | Needs app-level `select!` — cotton-usb-host architectural limitation |
| Field offset assertions (4.1) | Low | Would catch `#[repr(C)]` layout bugs at compile time |
| Hardware setup documentation (4.3) | Medium | User-facing doc for Teensy 4.1 USB2 host port wiring |

---

## 4.5 Debugging: Hot-Plug Failure (Interrupt Pipe Halted on Disconnect) — ✅ COMPLETE

**Date**: 2026-02-19
**Status**: Fixed

### Symptom

1. Keyboard working normally with `rtic_usb_hid_keyboard` example.
2. User unplugs the keyboard.
3. Log output: `[DEBUG imxrt_usbh::host]: [HC] qTD done: token=0x00088141 rem=8 buf=[00 00 00 00 00 00 00 00]`
4. Plug keyboard back in → nothing happens. No re-enumeration.

### Token Decode: 0x00088141

| Field | Bits | Value | Meaning |
|-------|------|-------|---------|
| Active | 7 | 0 | Transfer complete (not in progress) |
| Halted | 6 | **1** | **Pipe halted** |
| Ping/ERR | 0 | **1** | Transaction error occurred |
| CERR | 11:10 | **0** | Error counter exhausted (tried 3 times) |
| PID | 9:8 | 1 | IN token |
| Total Bytes | 30:16 | 8 | 0 of 8 bytes transferred |
| IOC | 15 | 1 | Interrupt on Complete set |

This is the interrupt pipe's qTD. The controller tried to send IN tokens to the
(now absent) keyboard, got no response 3 times, exhausted the error counter, and
halted the QH.

### Root Cause Analysis

Two intertwined problems:

#### Problem 1: InterruptPipe re-arms a halted qTD (busy-loop)

`Imxrt1062InterruptPipe::poll_next()` **does not check the Halted bit**. When
the qTD completes with Halted=1, the code falls through to the "transfer
complete" path, computes `received = 0` bytes, builds a 0-byte `InterruptPacket`,
re-arms the qTD, and returns `Poll::Ready(Some(packet))`.

Since the device is unplugged, the re-armed qTD immediately halts again. The
pipe returns another 0-byte packet. This creates an **infinite busy-loop** of
halted → re-arm → halted → re-arm, delivering an infinite stream of 0-byte
packets.

The example's inner loop does `pipe.next().await` and filters `pkt.size < 8`
with `continue`, so it silently spins forever without ever breaking out to the
outer event loop.

#### Problem 2: DeviceDetect stream is starved

The `device_events_no_hubs()` function uses `.then()` on the `DeviceDetect`
stream. The example's control flow is:

```
loop {                                  // outer: polls events.next().await
    Connect → {
        loop {                          // inner: polls pipe.next().await
            pipe.next().await → ...     // ← stuck here forever
        }
    }
    Disconnect → { ... }               // ← never reached
}
```

The inner loop is spinning on the halted interrupt pipe, so the outer loop never
calls `events.next().await`, which means `DeviceDetect` is **never polled
again**. Even though the device physically disconnected (PORTSC1 CCS=0), the
disconnect event never fires.

On the RP2040, this doesn't happen because the RP2040's `InterruptPipe` returns
`Poll::Pending` when no buffer is filled (device gone → no buffers fill → pipe
goes dormant). The RTIC executor then polls `DeviceDetect` via the waker for the
port change interrupt.

### Reference: RP2040 Behavior

The RP2040's `Rp2040InterruptPipe::poll_next()`:
- **Never checks error status bits**
- Returns `Poll::Pending` when no completed buffer is available
- The stream **never terminates** (never returns `None`)
- If the device is unplugged, no buffers fill → pipe returns `Pending` forever
- This is by design: the cotton-usb-host framework expects interrupt pipes to be
  infinite streams of successful packets, with disconnect handled separately via
  `DeviceDetect`

### Fix — Implemented

Modified `Imxrt1062InterruptPipe::poll_next()` in `src/host.rs` to check the
Halted bit before the "transfer complete" path:

```rust
// If the qTD halted, the device is likely disconnected or the endpoint
// stalled. Terminate the stream (return None) so the application's
// inner poll loop breaks out and the outer event loop can poll
// DeviceDetect to handle the disconnect.
if token & QTD_TOKEN_HALTED != 0 {
    debug!("[HC] InterruptPipe: qTD halted (token=0x{:08x}), terminating stream", token);
    // Re-enable port change interrupt so DeviceDetect can fire.
    ral::modify_reg!(ral::usb, usb_inst, USBINTR, |v| v | (1 << 2));
    return Poll::Ready(None);
}
```

Returns `Poll::Ready(None)` (stream terminated) rather than `Poll::Pending`.
The inner `loop { pipe.next().await }` in the example handles `None` with
`break`, falling through to the outer event loop where `DeviceDetect` fires.

This is a minor deviation from the RP2040 contract (which never terminates the
stream), but is the simplest correct fix for the `.then()` event loop structure
used by `device_events_no_hubs()`.

### Expected Test Results

After flashing `rtic_usb_hid_keyboard.hex`:

1. **Plug in keyboard** → normal enumeration and key reporting (unchanged)
2. **Unplug keyboard** → log should show:
   ```
   [DEBUG imxrt_usbh::host]: [HC] InterruptPipe: qTD halted (token=0x00088141), terminating stream
   [WARN  rtic_usb_hid_keyboard::app]: Interrupt pipe stream ended
   [INFO  rtic_usb_hid_keyboard::app]: DeviceEvent::Disconnect
   ```
3. **Plug keyboard back in** → should re-enumerate and resume key reporting

### Additional Considerations

1. **Port change interrupt masking**: When the device disconnect fires as a PCI
   interrupt simultaneous with the transfer error, `on_irq()` wakes the device
   waker AND the pipe wakers, then masks both PCI and UE/UEE. The pipe's
   `poll_next()` re-enables UE/UEE but does NOT re-enable PCI. The fix
   explicitly re-enables PCI (bit 2) in the halt path so `DeviceDetect` can
   receive the disconnect event.

2. **Stalled endpoints**: A STALL response from a functioning device also sets
   the Halted bit. With this fix, a stalled interrupt endpoint terminates the
   stream rather than retrying. This matches the RP2040's behavior where pipe
   errors are silently ignored and disconnect is the only cleanup path.

### Implementation Plan

1. ~~In `InterruptPipe::poll_next()`: check `QTD_TOKEN_HALTED` before the
   "transfer complete" path. Return `Poll::Ready(None)` to terminate the
   stream. Re-enable PCI interrupt.~~ **DONE**
2. Test: unplug keyboard → inner loop breaks → outer loop polls DeviceDetect →
   `Disconnect` event fires → plug keyboard back in → `Connect` + re-enumerate.

## Challenges for This Phase

### Challenge: Testing Without Device-Mode Loopback

**Problem**: Can't easily create automated tests — need real USB devices.

**Solution** (implemented):
- Structured bring-up sequence (section 4.2) validated each layer incrementally
- First milestone (`GET_DESCRIPTOR` to address 0) proved the entire pipeline
- LPUART logging (`imxrt-log`) used extensively during development
- Three working example applications serve as integration tests

---

## 4.6 Debugging: Hub Support (USB Hub + Keyboard)

**Date**: 2026-02-19
**Status**: In progress — iteration 1

### Symptom

Plugged in a USB hub (Belkin VID=050d PID=0234) with a mouse and keyboard
attached. The example (`rtic_usb_hid_keyboard`) produced:

```
[DEBUG imxrt_usbh::host]: [HC] DeviceDetect: status change  PORTSC1=0x10001803
[INFO rtic_usb_hid_keyboard::app]: DeviceEvent::Connect  addr=1  VID=050d PID=0234 class=9 subclass=0
[INFO rtic_usb_hid_keyboard::app]: Found HID interface: iface=0 ep=1 mps=1 interval=12
[INFO rtic_usb_hid_keyboard::app]: Opening interrupt IN stream...
[DEBUG imxrt_usbh::host]: [HC] recv_buf[0] @ 0x20201340 (OCRAM=0x2020_0000..0x202F_FFFF, DTCM=0x2000_xxxx)
[DEBUG imxrt_usbh::host]: [HC] interrupt pipe allocated: addr=1 ep=1 mps=1 qh=2 qtd=0
[DEBUG imxrt_usbh::host]: [HC] InterruptPipe: qTD halted (token=0x00018148), terminating stream
[WARN rtic_usb_hid_keyboard::app]: Interrupt pipe stream ended
[DEBUG imxrt_usbh::host]: [HC] DeviceDetect: status change  PORTSC1=0x18001207
```

No keyboard input was received. The hub was detected but treated as a HID
device (its status-change endpoint was opened as an HID interrupt pipe).

### PORTSC1 Decode

| Timestamp | PORTSC1 | CCS | PE | PSPD | Meaning |
|-----------|---------|-----|-----|------|---------|
| Initial connect | 0x10001803 | 1 | 0 | 00 (FS) | Hub connected, pre-reset speed |
| After pipe halt | 0x18001207 | 1 | 1 | 10 (HS) | Hub negotiated High Speed after port reset |

### Token Decode: 0x00018148

| Field | Bits | Value | Meaning |
|-------|------|-------|---------|
| Total Bytes | 31:16 | 1 | All 1 byte remaining (0 transferred) |
| IOC | 15 | 1 | Interrupt on Complete set |
| C_Page | 14:12 | 0 | Current buffer page 0 |
| CERR | 11:10 | **0** | **Error counter exhausted** (started at 3, all 3 retries failed) |
| PID | 9:8 | 01 | IN token |
| Active | 7 | 0 | Transfer not in progress |
| Halted | 6 | **1** | **Pipe halted** |
| Data Buffer Error | 5 | 0 | — |
| Babble | 4 | 0 | — |
| Transaction Error | 3 | **1** | **Signaling error on all 3 attempts** |
| Missed µ-frame | 2 | 0 | — |
| Split xact state | 1 | 0 | — |
| Ping/ERR | 0 | 0 | — |

The hub's interrupt endpoint (addr=1, ep=1, mps=1) received 3 consecutive
Transaction Errors and halted. The hub never responded to the IN tokens.

### Root Cause Analysis

Four intertwined problems:

#### Problem 1: Example uses `device_events_no_hubs()` — treats hub as HID

The example calls `bus.device_events_no_hubs(delay_ms)`, which provides no
hub management. The hub (class=9, subclass=0) is reported to the application
as a regular device. The `HidFinder` descriptor visitor finds the hub's
status-change interrupt endpoint (ep=1, mps=1, interval=12) and treats it
as an HID keyboard endpoint.

Even if the pipe didn't halt, it would only carry hub port status bitmaps
(1 byte), not 8-byte HID boot protocol reports.

**Fix**: Switch to `bus.device_events(&hub_state, delay_ms)` with a
`HubState::default()`. The cotton-usb-host crate handles hub detection,
configuration, port power, and downstream device enumeration internally.

#### Problem 2: Periodic QH has NAK Reload = 15 (should be 0)

The `qh_characteristics()` function always sets RL=15 (bits [31:28] of the
QH Characteristics word). Per EHCI spec §3.6:

> *For endpoints in the periodic schedule, this field must be 0.*

Our `do_alloc_interrupt_pipe()` uses `qh_characteristics()` with is_control=false,
which produces RL=15. Non-zero RL on periodic QHs causes undefined EHCI
controller behavior. The NXP implementation appears to generate Transaction
Errors when RL≠0 on periodic QHs.

The LS keyboard directly-connected to the root port worked despite RL=15,
likely because the LS timing is more tolerant. For HS endpoints (like the
hub's), the issue manifests immediately.

**Fix**: Clear RL bits in `do_alloc_interrupt_pipe()`:
```rust
let characteristics = ehci::qh_characteristics(...) & !(0xF << 28);
```

#### Problem 3: DeviceDetect fires twice (speed changes after port reset)

The DeviceDetect stream reports `Present(Full12)` on initial connection
(PSPD=00 before port reset). After cotton-usb-host resets the port, the
hub negotiates High Speed (PSPD=10). The DeviceDetect stream sees the speed
change (Full12 → High480) and fires a second `Present(High480)` event.

This second event triggers another port reset and re-enumeration, disrupting
any hub state that cotton-usb-host set up.

**Fix**: Only fire `DeviceDetect` on connect/disconnect transitions (CCS
changes), not speed changes within the same connection.

#### Problem 4: EHCI HS split transactions not implemented

When the hub negotiates at High Speed, downstream Full Speed and Low Speed
devices require EHCI split transactions (hub_addr, hub_port, C-mask in the QH
capabilities word). The cotton-usb-host `TransferExtras` API only has
`Normal` and `WithPreamble` (a FS concept: PRE PID), which doesn't carry the
hub routing information EHCI needs.

Additionally, `port_speed()` always reads the root port PSPD, but downstream
devices may run at a different speed than the root port.

**Workaround**: Force the root port to Full Speed using the NXP-specific PFSC
bit (Port Force Full Speed Connect, PORTSC1 bit 24). This prevents HS
negotiation, making the hub connect at FS. With a FS hub:
- FS hub endpoints work with EPS=FS (same as directly-connected FS devices)
- Downstream FS devices are handled transparently by the FS hub
- Downstream LS devices need `WithPreamble` (PRE PID), which cotton-usb-host
  provides and our code needs to handle in the QH

Later, proper EHCI HS split transaction support can be added.

### Implementation Plan — Iteration 1

1. **Fix RL=0 for periodic QHs** — clear bits [31:28] of characteristics in
   `do_alloc_interrupt_pipe()`
2. **Fix DeviceDetect** — suppress speed-change events, only fire on CCS transitions
3. **Add PFSC=1** — force FS at root port during init, ensuring hub connects at FS
4. **Switch example** — use `device_events()` with `HubState` for hub-aware enumeration
5. **Handle WithPreamble** — set QH EPS=LS for WithPreamble in `do_alloc_interrupt_pipe()`
   and `do_control_transfer()`

### Expected Results After Iteration 1

1. Hub detected as class=9 → `DeviceEvent::HubConnect` (logged)
2. Hub configured, ports powered (cotton-usb-host handles this internally)
3. Hub interrupt pipe active, receiving status changes (no more halt)
4. Downstream device detected via hub status change interrupt
5. Downstream device enumeration **may succeed** (FS devices) or **may fail**
   (LS devices requiring WithPreamble QH handling)
6. If keyboard is LS: likely fails at control transfer (wrong speed in QH)
7. If keyboard is FS: may succeed through to HID report reading

### Actual results

When the teensy is started with the hub attached and a keyboard in the first port and a mouse in the second, only the keyboard is detected. Here are the log lines:

```
[DEBUG imxrt_usbh::host]: [HC] DeviceDetect: status change  PORTSC1=0x11001803
[DEBUG imxrt_usbh::host]: [HC] recv_buf[0] @ 0x20201340 (OCRAM=0x2020_0000..0x202F_FFFF, DTCM=0x2000_xxxx)
[DEBUG imxrt_usbh::host]: [HC] interrupt pipe allocated: addr=1 ep=1 mps=64 qh=2 qtd=0
[INFO rtic_usb_hid_keyboard::app]: DeviceEvent::HubConnect  addr=1
[DEBUG imxrt_usbh::host]: [HC] qTD done: token=0x803f8d00 rem=63 buf=[02 00 00 00 00 00 00 00]
[INFO rtic_usb_hid_keyboard::app]: DeviceEvent::Connect  addr=31  VID=3297 PID=4974 class=0 subclass=0
[INFO rtic_usb_hid_keyboard::app]: Found HID interface: iface=0 ep=1 mps=8 interval=1
[INFO rtic_usb_hid_keyboard::app]: Opening interrupt IN stream...
[DEBUG imxrt_usbh::host]: [HC] recv_buf[1] @ 0x20201380 (OCRAM=0x2020_0000..0x202F_FFFF, DTCM=0x2000_xxxx)
[DEBUG imxrt_usbh::host]: [HC] interrupt pipe allocated: addr=31 ep=1 mps=8 qh=3 qtd=1
```

When then the keyboard is unplugged, the mouse is detected.

```
[DEBUG imxrt_usbh::host]: [HC] InterruptPipe: qTD halted (token=0x00088141), terminating stream
[WARN rtic_usb_hid_keyboard::app]: Interrupt pipe stream ended
[DEBUG imxrt_usbh::host]: [HC] qTD done: token=0x003f8d00 rem=63 buf=[06 00 00 00 00 00 00 00]
[INFO rtic_usb_hid_keyboard::app]: DeviceEvent::Disconnect
[DEBUG imxrt_usbh::host]: [HC] qTD done: token=0x803f8d00 rem=63 buf=[06 00 00 00 00 00 00 00]
[INFO rtic_usb_hid_keyboard::app]: DeviceEvent::Connect  addr=31  VID=046d PID=c077 class=0 subclass=0
[INFO rtic_usb_hid_keyboard::app]: Found HID interface: iface=0 ep=1 mps=4 interval=10
[INFO rtic_usb_hid_keyboard::app]: Opening interrupt IN stream...
[DEBUG imxrt_usbh::host]: [HC] recv_buf[1] @ 0x20201380 (OCRAM=0x2020_0000..0x202F_FFFF, DTCM=0x2000_xxxx)
[DEBUG imxrt_usbh::host]: [HC] interrupt pipe allocated: addr=31 ep=1 mps=4 qh=3 qtd=1
```

If the keyboard is then plugged in again, nothing happens unless the mouse is unplugged, and then you get
```
[DEBUG imxrt_usbh::host]: [HC] InterruptPipe: qTD halted (token=0x00048141), terminating stream
[WARN rtic_usb_hid_keyboard::app]: Interrupt pipe stream ended
[DEBUG imxrt_usbh::host]: [HC] qTD done: token=0x003f8d00 rem=63 buf=[06 00 00 00 00 00 00 00]
[INFO rtic_usb_hid_keyboard::app]: DeviceEvent::Connect  addr=30  VID=3297 PID=4974 class=0 subclass=0
[INFO rtic_usb_hid_keyboard::app]: Found HID interface: iface=0 ep=1 mps=8 interval=1
[INFO rtic_usb_hid_keyboard::app]: Opening interrupt IN stream...
[DEBUG imxrt_usbh::host]: [HC] recv_buf[1] @ 0x20201380 (OCRAM=0x2020_0000..0x202F_FFFF, DTCM=0x2000_xxxx)
[DEBUG imxrt_usbh::host]: [HC] interrupt pipe allocated: addr=30 ep=1 mps=8 qh=3 qtd=1
```

### References

- EHCI Specification §3.6 — QH Characteristics and Capabilities
- EHCI Specification §4.9 — Periodic Schedule traversal
- i.MX RT 1060 Reference Manual §56 — PORTSC1 PFSC bit
- cotton-usb-host v0.2.1 `usb_bus.rs` — `device_events()`, `new_hub()`, `handle_hub_packet()`
- USBHost_t36 — reference implementation for same hardware with hub support

---

### Analysis of Iteration 1 Results

**Date**: 2026-02-19
**Status**: ✅ Hub support validated. All core functionality works correctly.

#### What worked (Hub support fundamentals validated)

1. **Hub detection**: Hub (Belkin VID=050d PID=0234) detected as class=9,
   `DeviceEvent::HubConnect` fired correctly. ✅
2. **Hub interrupt pipe**: RL=0 fix worked — hub status changes received without
   Transaction Errors. Status byte `0x02` (port 1) decoded correctly. ✅
3. **PFSC=1**: Hub connected at Full Speed (PORTSC1=0x11001803 → FS pre-reset,
   not HS). No longer getting speed-change churn. ✅
4. **Downstream device enumeration**: Keyboard (VID=3297 PID=4974, ZSA brand)
   enumerated at addr=31 through the hub. All control transfers (SET_ADDRESS,
   GET_DESCRIPTOR, SET_CONFIGURATION) succeeded through the FS hub. ✅
5. **HID keyboard input through hub**: Key press events are received correctly
   when keys are pressed on the active keyboard. The ZSA keyboard does NOT send
   idle reports (it only reports when keys are pressed/released), which is normal
   for keyboards that default to report protocol mode. ✅
6. **Hot-plug through hub**: Unplugging keyboard → `DeviceEvent::Disconnect` →
   mouse detected → `DeviceEvent::Connect`. Working correctly. ✅

#### Serial device processing (expected cotton-usb-host behavior)

Only one downstream device is detected at a time. When the keyboard is connected,
the mouse on port 2 is not detected until the keyboard is unplugged. This is
**expected behavior** inherent to the cotton-usb-host architecture, not a driver bug.

**Root cause**: cotton-usb-host's `device_events()` internally combines root port
detection and hub interrupt pipe polling in a single async stream. All cotton
examples (MSC, AX88772 ethernet) block inside the `DeviceEvent::Connect` handler—
there is no built-in concurrent polling mechanism. When the example enters
`loop { pipe.next().await }` for the keyboard, `device_events()` is no longer
polled, so the hub's interrupt pipe stops being serviced and port 2 status
changes cannot be processed.

**This means concurrent multi-device use through a hub requires application-level
restructuring, not changes to cotton-usb-host.** The application would need to
use `futures::select!` or similar to poll both the `device_events` stream and
any active interrupt pipes simultaneously, e.g.:

```rust
// Pseudo-code for concurrent multi-device polling
loop {
    futures::select_biased! {
        event = events.next() => match event {
            Some(DeviceEvent::Connect(dev, info)) => {
                // Store device, open pipe, add to active_pipes vec
            }
            Some(DeviceEvent::Disconnect(_)) => {
                // Remove device from active_pipes
            }
            _ => {}
        },
        pkt = active_pipes.next() => {
            // Process HID report from whichever pipe completed
        },
    }
}
```

This is out of scope for Phase 4 validation (the goal is to prove the driver
works correctly, not to build a complete multi-device application). The serial
model is sufficient to validate that hub support, enumeration, and data transfers
all function correctly for each downstream device.

#### Conclusion: Hub support (4.2 #11) — ✅ VALIDATED

Hub support is fully functional. The driver correctly:
- Detects and configures USB hubs via cotton-usb-host's `device_events()` API
- Handles hub interrupt pipes (periodic schedule, RL=0)
- Enumerates downstream devices through the hub
- Delivers HID keyboard input from hub-attached devices
- Handles hot-plug/unplug of downstream devices through the hub
- Forces Full Speed at root port (PFSC=1) to avoid HS split transaction complexity

The only limitation (serial device processing) is a cotton-usb-host architectural
choice, not a driver bug, and can be addressed at the application level with
`futures::select!` if concurrent multi-device support is needed.

---

## 4.7 Debugging: Hub Disconnect Hangs (Control Transfer to Absent Device)

**Date**: 2026-02-19
**Status**: Fix implemented — ready for hardware test

### Symptom

With a hub + keyboard connected, physically unplugging the **hub** from the
Teensy produces:

```
[DEBUG imxrt_usbh::host]: [HC] InterruptPipe: qTD halted (token=0x00088141), terminating stream
[WARN rtic_usb_hid_keyboard::app]: Interrupt pipe stream ended
[DEBUG imxrt_usbh::host]: [HC] qTD done: token=0x003f8d00 rem=63 buf=[06 00 00 00 00 00 00 00]
[DEBUG imxrt_usbh::host]: [HC] control xfer: addr=1 pkt=8 speed=FS extras=Normal char=0xF8084001 caps=0x40000000
```

After this, plugging the hub back in produces no output. The system is hung.

### Sequence of Events

1. Hub physically disconnected from root port
2. Keyboard's interrupt pipe halts (qTD error, no device) → stream terminates
3. App breaks out of inner `pipe.next().await` loop, resumes `events.next().await`
4. Hub's interrupt pipe (qh=2) also completed — its buffer contains `0x06`
   (ports 1 and 2 status changed), filled just before the physical disconnect
5. `device_events()` internally uses `futures::stream::select` to merge the
   root port `DeviceDetect` stream and the `HubStateStream` (hub interrupt pipes).
   Due to round-robin fairness, the hub packet is polled first
6. cotton-usb-host's `.then()` calls `handle_hub_packet()`, which issues
   `GET_PORT_STATUS` — a control transfer to addr=1 (the now-absent hub)
7. **The control transfer hangs forever** (see root cause below)
8. `DeviceDetect` (root port Absent) is never polled → no `Disconnect` event → stuck

### Root Cause: TransferComplete Misses Setup-Phase Halt

When the EHCI controller processes the control transfer's 3-qTD chain
(setup → data → status) to a disconnected device:

1. **Setup qTD**: Controller sends SETUP token, gets no response, retries 3×
   (CERR exhausted), sets Halted + XactErr in the setup qTD token
2. **Data qTD**: Never processed (controller stops advancing on halt)
3. **Status qTD**: Never processed

The controller copies the halted setup qTD's token into the **QH overlay_token**.

`TransferComplete::poll()` checked:
- `status_qtd.token` → Active=1 (never reached) → not complete
- `data_qtd.token` → Active=1, not Halted (never reached) → early exit doesn't trigger

**Neither the setup qTD's token nor the QH overlay_token was checked.** Result:
`TransferComplete` returns `Poll::Pending` indefinitely. The USB Error Interrupt
fires and wakes the task, but re-polling sees the same state. **Infinite loop of
wake → poll → Pending.**

This is also a latent bug for `DataPhase::None` transfers (e.g. SET_FEATURE,
CLEAR_FEATURE) where `data_qtd` is `None`, so the data qTD early-exit path
is skipped entirely.

### Fix (§4.7 fix 1 — TransferComplete overlay check) — ✅ VERIFIED

Added a QH overlay_token halt check in `TransferComplete::poll()`, before the
existing data qTD check:

```rust
if token & QTD_TOKEN_ACTIVE != 0 {
    let qh = unsafe { &*self.qh };
    let overlay = qh.overlay_token.read();
    if overlay & QTD_TOKEN_HALTED != 0 {
        return Poll::Ready(Err(map_qtd_error(overlay)));
    }
    // ... existing data_qtd check ...
}
```

**Test result**: The control transfer to the absent hub now fails promptly with
`ProtocolError`. Disconnect fires. But plugging the hub back in fails to detect
downstream devices — see §4.7 fix 2 below.

### Actual Results After Fix 1

Disconnect sequence works correctly:
```
[HC] InterruptPipe: qTD halted (token=0x00088141), terminating stream
Interrupt pipe stream ended
[HC] qTD done: token=0x003f8d00 rem=63 buf=[06 ...]
[HC] control xfer: addr=1 ...
[HC] TransferComplete: QH overlay halted (overlay=0x00080249), aborting
control_transfer -> Err(ProtocolError)
DeviceEvent::EnumerationError  hub=0 port=1
DeviceDetect: status change  PORTSC1=0x1D00100A
DeviceEvent::Disconnect
[HC] InterruptPipe: qTD halted (token=0x00408141), terminating stream
```

Reconnect works partially — hub is enumerated and HubConnect fires, but no
downstream devices are ever detected despite devices being physically present.

### Root Cause 2: Stale Hub Pipe Poisons HubStateStream

After the hub is disconnected:

1. The hub's interrupt pipe (qh=2, slot 0 in `HubState.pipes`) halts.
2. cotton-usb-host's disconnect handler updates Topology but **does NOT clear
   `HubState.pipes[0]`** — the stale pipe stays as `Some(...)`.
3. When the hub is plugged back in, `try_add()` finds slot 0 is occupied
   and stores the new hub pipe at slot 1.
4. `HubStateStream::poll_next()` iterates:
   - Polls slot 0 (stale halted pipe) → gets `Poll::Ready(None)`
   - **Immediately returns `Ready(None)`** without checking slot 1
5. `futures::stream::select` treats `Ready(None)` as "HubStateStream terminated"
   and **never polls it again**.
6. New hub's interrupt pipe (slot 1) is never polled → no downstream device
   detection ever occurs.

Pool evidence: On first connect, hub gets qh=2 (pool token 0). On reconnect,
hub gets qh=3 (pool token 1). Token 0 was never freed — confirming the old
pipe was never dropped.

### Fix 2: cotton-usb-host HubStateStream + disconnect cleanup

Two changes in `cotton-usb-host/src/usb_bus.rs`:

**a) `HubStateStream::poll_next`** — handle `Ready(None)` by dropping the pipe
and continuing to the next slot, instead of propagating the None:

```rust
fn poll_next(...) -> Poll<Option<Self::Item>> {
    for slot in self.state.pipes.borrow_mut().iter_mut() {
        if let Some(pipe) = slot {
            match pipe.poll_next_unpin(cx) {
                Poll::Ready(Some(packet)) => return Poll::Ready(Some(packet)),
                Poll::Ready(None) => { *slot = None; }  // drop terminated pipe
                Poll::Pending => {}
            }
        }
    }
    Poll::Pending
}
```

**b) Disconnect handler** — explicitly drop all hub pipes on root disconnect:

```rust
// In the Absent branch of .then():
for slot in hub_state.pipes.borrow_mut().iter_mut() {
    *slot = None;
}
DeviceEvent::Disconnect(BitSet(0xFFFF_FFFF))
```

### Expected Results After Fix 2

1. **Disconnect hub** → Disconnect event fires (same as fix 1)
2. **Reconnect hub** → HubConnect fires, downstream devices detected:
   ```
   DeviceEvent::HubConnect  addr=1
   [HC] qTD done: token=0x803f8d00 rem=63 buf=[02 ...]
   DeviceEvent::Connect  addr=31  VID=3297 PID=4974 ...
   ```
3. **Repeated cycles** → each disconnect/reconnect should work identically
