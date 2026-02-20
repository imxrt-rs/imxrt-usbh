# Cotton-usb-host device_events() and Transfer Extras Analysis

**Date**: 2026-02-19
**Context**: Hub support debugging, iteration 2

## How device_events() Works

`device_events()` uses `futures::stream::select` to merge two sub-streams:

1. **Root port detection** (`HC::DeviceDetect` from `device_detect()`) — fires on root port connect/disconnect
2. **`HubStateStream`** — a custom stream that round-robin polls all allocated hub interrupt pipes (up to 15 stored in `HubState.pipes`)

The `HubStateStream::poll_next()` iterates over all hub pipes, calling `poll_next_unpin(cx)` on each — any hub reporting a port change wakes the task. Events are then processed by an async `.then()` combinator that handles resets, enumeration, address assignment, etc.

## Serial Processing Model

**No concurrent polling mechanism exists.** All cotton examples (MSC, AX88772 ethernet) block inside the `DeviceEvent::Connect` handler doing all their work before returning to `events.next().await`. There is no `select!`, no multiplexed polling, no built-in way to poll `device_events` and an application interrupt pipe simultaneously.

This means: once our HID keyboard example enters the inner `pipe.next().await` loop, the `device_events` stream (and its internal hub polling) stops. This is correct for single-device testing.

## How transfer_extras is Determined

### get_transfer_extras() — the core logic

```rust
fn get_transfer_extras(&self, speed: UsbSpeed) -> TransferExtras {
    if self.root_device_speed.get() == Some(UsbSpeed::Full12)
        && speed == UsbSpeed::Low1_5
    {
        TransferExtras::WithPreamble
    } else {
        TransferExtras::Normal
    }
}
```

Logic: If root device is Full-Speed and target device is Low-Speed, there must be a hub involved → use WithPreamble.

### Speed Detection for Hub-Attached Devices

From GET_PORT_STATUS `wPortStatus` bits 9-10:
- `state & 0x600 == 0` → Full-Speed
- `state & 0x600 == 0x400` → High-Speed
- Otherwise → Low-Speed

### Summary Table

| Scenario | transfer_extras | How determined |
|---|---|---|
| Device directly on root port | Normal | root speed == device speed |
| Hub's own status endpoint | Normal | Hardcoded in HubState::try_add |
| LS device behind FS hub (all transfers) | WithPreamble | root=FS, device=LS |
| FS device behind FS hub | Normal | root=FS, device=FS |

### Hub Status Interrupt Pipe

cotton-usb-host passes `device.packet_size_ep0` (typically 64 for FS hubs) as the max_packet_size to `try_alloc_interrupt_pipe`, not the actual interrupt endpoint's mps. This is why we see mps=64 for the hub's status endpoint. Works fine because short packets (1 byte) are detected as transfer completion.
