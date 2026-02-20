# Research: How cotton-usb-host Handles Concurrent Device Event & Interrupt Pipe Polling

**Date**: 2026-02-19  
**Request**: Understand how `device_events()` works internally and how to structure an application that needs both hub event monitoring and interrupt pipe reading.

---

## 1. How `device_events()` Works Internally

### Source: `cotton-usb-host/src/usb_bus.rs` lines 479–566

The `device_events()` method creates a **merged stream** of two sub-streams using `futures::stream::select`:

```rust
pub fn device_events<'a, D, F>(
    &'a self,
    hub_state: &'a HubState<HC>,
    delay_ms_in: F,
) -> impl Stream<Item = DeviceEvent> + 'a {
    let root_device = self.driver.device_detect();   // Stream<Item=DeviceStatus>

    enum InternalEvent {
        Root(DeviceStatus),
        Packet(InterruptPacket),
    }

    futures::stream::select(
        root_device.map(InternalEvent::Root),
        HubStateStream { state: hub_state }.map(InternalEvent::Packet),
    )
    .then(move |ev| { ... })
}
```

**Two sub-streams are merged:**

1. **`root_device`** — the `HC::DeviceDetect` stream from `device_detect()`. This fires when a device is connected/disconnected at the root port.

2. **`HubStateStream`** — a custom stream that polls ALL currently-active hub interrupt pipes. When any hub reports a port status change, this fires.

### HubStateStream Implementation (lines 384–405)

```rust
struct HubStateStream<'a, HC: HostController> {
    state: &'a HubState<HC>,
}

impl<HC: HostController> Stream for HubStateStream<'_, HC> {
    type Item = InterruptPacket;

    fn poll_next(self: Pin<&mut Self>, cx: &mut Context) -> Poll<Option<Self::Item>> {
        for pipe in self.state.pipes.borrow_mut().iter_mut().flatten() {
            let poll = pipe.poll_next_unpin(cx);
            if poll.is_ready() {
                return poll;
            }
        }
        Poll::Pending
    }
}
```

This iterates over all allocated hub interrupt pipes (up to 15), polling each one. If any is ready, it returns that packet. The waker is registered with each pipe (via `poll_next_unpin(cx)`), so the task will be woken when **any** hub pipe has data.

### HubState Structure (lines 329–382)

```rust
pub struct HubState<HC: HostController> {
    topology: RefCell<Topology>,
    pipes: RefCell<[Option<HC::InterruptPipe>; 15]>,
}
```

Hub interrupt pipes are allocated during `new_hub()` via `try_add()`:

```rust
fn try_add(&self, hc: &HC, address: u8, endpoint: u8, 
           max_packet_size: u8, interval_ms: u8) -> Result<(), UsbError> {
    for p in self.pipes.borrow_mut().iter_mut() {
        if p.is_none() {
            *p = Some(hc.try_alloc_interrupt_pipe(
                address, TransferExtras::Normal,
                endpoint, max_packet_size as u16, interval_ms,
            )?);
            return Ok(());
        }
    }
    Err(UsbError::TooManyDevices)
}
```

### Event Processing (lines 499–566)

The `.then()` combinator processes each event:

- **Root connect**: Resets root port, reads device descriptor, assigns address. If device is a hub, calls `new_hub()` which configures it, allocates its interrupt pipe into `HubState.pipes`, and powers its ports.
- **Root disconnect**: Returns `Disconnect(BitSet(0xFFFF_FFFF))`.
- **Hub interrupt packet**: Calls `handle_hub_packet()` which reads hub port status, resets the port, enumerates the new downstream device, and returns a `DeviceEvent::Connect` (or Disconnect/EnumerationError).

**Key insight**: The `.then()` combinator is an async transform — it can `.await` internally (for delays, control transfers, etc.) and the outer stream will not produce the next event until the current one finishes processing.

---

## 2. HID / Interrupt Pipe Usage in cotton-usb-host

There is **no `cotton-usb-host-hid` crate**. The workspace has `cotton-usb-host` and `cotton-usb-host-msc` only.

The `UsbBus::interrupt_endpoint_in()` method (line 935) is the API for opening an interrupt pipe for application use:

```rust
pub fn interrupt_endpoint_in(
    &self,
    device: &UsbDevice,
    endpoint: u8,
    max_packet_size: u16,
    interval_ms: u8,
) -> impl Stream<Item = InterruptPacket> + '_ {
    let transfer_extras = self.get_transfer_extras(device.usb_speed);
    self.driver
        .alloc_interrupt_pipe(device.usb_address, transfer_extras,
                              endpoint, max_packet_size, interval_ms)
        .flatten_stream()
}
```

This returns a `Stream<Item = InterruptPacket>` — an application interrupt pipe, separate from the hub interrupt pipes managed by `HubState`.

---

## 3. How Mass Storage Uses the Bus

From `cotton-usb-host-msc/src/mass_storage.rs`:

- `MassStorage::new()` takes a `&UsbBus<HC>` and a `UsbDevice`
- It opens bulk IN and bulk OUT endpoints
- It implements `ScsiTransport` using `bus.bulk_in_transfer()` and `bus.bulk_out_transfer()`
- All operations are sequential async — send CBW, receive data, receive CSW

**Critical pattern in `rp2040-usb-msc.rs` example** (lines 142–240):

```rust
let mut p = pin!(stack.device_events(&hub_state, rtic_delay));

loop {
    let device = p.next().await;  // blocks until next event
    
    if let Some(DeviceEvent::Connect(device, info)) = device {
        // ... identify, configure, then DO ALL MSC WORK HERE:
        let ms = MassStorage::new(&stack, device)?;
        let mut device = ScsiDevice::new(ms);
        device.inquiry().await;
        device.test_unit_ready().await;
        // ... read/write blocks ...
        // THEN fall back to p.next().await
    }
}
```

The application **blocks inside the Connect handler** doing all mass storage operations. During this time, the `device_events` stream is NOT being polled, so:
- No new device connections are detected
- No hub port changes are processed
- No disconnect events are seen

This works for the MSC example because it does a finite amount of work (read a block, write a block, done).

---

## 4. Examples Using `device_events()` with Hub + Interrupt Pipes

### Our own `rtic_usb_hid_keyboard.rs` (the current test example)

```rust
let hub_state: HubState<Imxrt1062HostController> = HubState::default();
let bus = UsbBus::new(host);
let mut events = pin!(bus.device_events(&hub_state, delay_ms));

loop {
    match events.next().await {
        Some(DeviceEvent::Connect(device, info)) => {
            // ... configure device ...
            let mut pipe = pin!(bus.interrupt_endpoint_in(&usb_device, ep, ep_mps, ep_interval));
            loop {
                match pipe.next().await {
                    Some(pkt) => { /* process HID report */ }
                    None => break,
                }
            }
        }
        // ...
    }
}
```

**Problem**: Once we enter the inner `pipe.next().await` loop, we stop polling `events`. This means:
- Hub port changes are NOT monitored
- New device connections are NOT detected
- Disconnect events are NOT received

### Cotton's RP2040 examples (`rp2040-usb-msc.rs`, `rp2040-usb-otge100.rs`)

All follow the same pattern: block inside Connect handler, return when done. None of them show concurrent interrupt pipe pollingwith device_events.

---

## 5. Connect Handler Expectations: Block or Return Quickly?

Based on all evidence:

**The cotton-usb-host design expects the application to block inside the Connect handler.** The docstring shows:

```rust
/// loop {
///     let event = device_stream.next().await;
///     if let Some(DeviceEvent::Connect(device, info)) = event {
///         // ... process the device ...
///     }
/// }
```

All real examples block in the handler:
- MSC: Does full SCSI inquiry, read/write, then returns
- AX88772: Does full network adapter initialization, then returns
- HID keyboard (ours): Enters infinite loop reading interrupt pipe, **never** returns to the event loop

**This is the intended pattern for simple single-device scenarios.** The design assumes either:
1. You do finite work in the handler and return (MSC, network adapter init)
2. You have only one device of interest and can monopolize the event loop (HID keyboard directly connected)

---

## 6. Does cotton-usb-host Provide `select!` or Multiplexed Polling?

**No.** There is no built-in mechanism for simultaneously polling `device_events` AND an application interrupt pipe.

### Why This Matters for Hub + HID

When a keyboard is behind a hub, the flow is:
1. Hub connects → `device_events()` detects it, configures it, allocates hub interrupt pipe
2. Hub reports downstream device → `device_events()` processes hub interrupt, enumerates keyboard
3. `DeviceEvent::Connect(keyboard)` is yielded
4. App enters HID polling loop → **`device_events()` stream is no longer polled**
5. Hub interrupt pipe is no longer polled (it's inside `HubStateStream` which is part of `device_events`)
6. If a second device connects to the hub, it won't be detected

### Possible Solutions

**Option A: `futures::stream::select` at application level**

The application can merge the `device_events` stream with the interrupt pipe stream:

```rust
use futures::stream::{select, StreamExt};

enum AppEvent {
    Device(DeviceEvent),
    HidReport(InterruptPacket),
}

let device_stream = bus.device_events(&hub_state, delay_ms).map(AppEvent::Device);
// After getting a Connect event and configuring:
let hid_stream = bus.interrupt_endpoint_in(&device, ep, mps, interval).map(AppEvent::HidReport);

let mut merged = pin!(select(device_stream, hid_stream));
loop {
    match merged.next().await {
        Some(AppEvent::Device(DeviceEvent::Connect(..))) => { /* handle new device */ }
        Some(AppEvent::HidReport(pkt)) => { /* handle HID report */ }
        _ => {}
    }
}
```

**Problem**: This requires knowing about the HID device *before* creating the merged stream. The stream types are different and can't be easily combined after the fact due to pinning/lifetime constraints.

**Option B: Separate RTIC tasks**

Since RTIC v2 supports multiple async tasks, the correct architecture might be:
1. One task polls `device_events()` and spawns/signals other tasks when devices connect
2. Another task handles HID keyboard polling via an interrupt pipe

But `UsbBus` is `!Send` (contains `Cell`/`RefCell`), so sharing it between tasks requires careful design.

**Option C: Accept the limitation for the current phase**

For Phase 4 testing, the current pattern (blocking in the Connect handler) is fine for validating that:
- Hub enumeration works
- Keyboard behind hub is detected
- Interrupt pipe data is received

The fact that no new devices would be detected during HID polling is acceptable for a single-keyboard test scenario.

---

## Summary

| Question | Answer |
|----------|--------|
| How does `device_events()` combine root + hub polling? | `futures::stream::select` merging root `DeviceDetect` stream with `HubStateStream` that polls all hub interrupt pipes |
| Can HID interrupt pipes run concurrently with device_events? | **No built-in mechanism**. Application must choose one or the other, or build custom multiplexing. |
| How does MSC use the bus? | Blocks in Connect handler doing all work, then returns |
| Any examples showing concurrent polling? | **None**. All examples block in Connect handler. |
| Should Connect handler block or return quickly? | **Block** — all examples and docs show this pattern |
| Built-in `select!` or multiplexing? | **No**. Not provided by cotton-usb-host. |
| Impact on Phase 4 hub+keyboard test? | **Minimal**. Our current blocking-loop pattern works fine for single device behind hub. Hub events have already been processed before we enter the HID loop. |

---

## Recommendation for Current Phase

The current `rtic_usb_hid_keyboard.rs` architecture is correct for Phase 4 testing:
1. `device_events()` detects the hub and configures it (automatic, internal to `device_events`)
2. `device_events()` detects the keyboard behind the hub via hub interrupt pipe
3. App blocks in HID polling loop — this is fine because we only care about one device

For future multi-device support, we would need to implement Option A (application-level stream merging) or Option B (separate RTIC tasks with shared bus access).
