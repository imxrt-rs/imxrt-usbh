# Research: cotton-usb-host Hub Disconnect Race Condition

**Date**: 2026-02-19  
**Query**: How does cotton-usb-host handle hub disconnect in `device_events()`, and what race condition exists when the hub is physically disconnected?

## 1. The `device_events()` Stream Architecture

In [usb_bus.rs](cotton-usb-host/src/usb_bus.rs#L479-L566), `device_events()` creates a merged stream:

```rust
futures::stream::select(
    root_device.map(InternalEvent::Root),           // DeviceDetect → Root port changes
    HubStateStream { state: hub_state }.map(InternalEvent::Packet), // Hub interrupt pipes
)
.then(move |ev| {
    async move {
        match ev {
            InternalEvent::Root(status) => {
                if let DeviceStatus::Present(speed) = status {
                    // ... enumerate root device, configure hub, etc.
                } else {
                    // Root disconnect
                    hub_state.topology.borrow_mut().device_disconnect(0, 1);
                    DeviceEvent::Disconnect(BitSet(0xFFFF_FFFF))
                }
            }
            InternalEvent::Packet(packet) => self
                .handle_hub_packet(hub_state, &packet, delay_ms)
                .await
                .unwrap_or_else(|e| {
                    DeviceEvent::EnumerationError(0, 1, e)
                }),
        }
    }
})
```

### Key semantics of `.then()`

`StreamExt::then()` processes items **sequentially**. When an item is yielded by the inner `select()` stream, the `.then()` closure runs to completion (including all `await` points inside it) before the next item is processed. This is critical for the race condition.

### `HubStateStream`

```rust
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

It round-robin polls all allocated hub interrupt pipes. When any pipe has data, it yields it immediately.

## 2. The Race Condition

When the hub (root port device, addr=1) is physically disconnected:

### Timeline

1. **Hub physically disconnected** — USB cable removed
2. **Two things happen nearly simultaneously:**
   - PORTSC1.CCS → 0, port change interrupt fires → `DeviceDetect` waker is woken
   - Hub's interrupt pipe qTD (on periodic schedule) may have **already received data** into its DMA buffer before the physical disconnect propagated. The EHCI controller completes the qTD (Active→0) and fires a transfer completion interrupt → pipe waker is woken
3. **Keyboard's interrupt pipe** also halts (qTD error, CERR exhausted) → stream returns `None` → app's inner loop breaks → app calls `events.next().await`
4. **`futures::stream::select` runs**. Both the Root stream and the Packet stream have ready items. `select` uses round-robin fairness — **either one could be polled first**.

### Case A: Root(Absent) polled first ✅
- `DeviceEvent::Disconnect(0xFFFF_FFFF)` is yielded
- The hub interrupt packet is still in the merged stream but hasn't been processed yet
- Application sees Disconnect, cleans up, goes back to waiting for Connect
- Next poll picks up the stale hub packet, `handle_hub_packet` tries `get_hub_port_status` to the disconnected hub — but the **control transfer will fail** (see §3)
- `handle_hub_packet` returns `Err(UsbError)` → `.unwrap_or_else` → `DeviceEvent::EnumerationError(0, 1, e)`
- Application sees EnumerationError, ignores it, continues

### Case B: Packet polled first ⚠️
- `handle_hub_packet` runs with the stale status change byte (e.g., `0x06` = ports 1 and 2 changed)
- It calls `get_hub_port_status(addr=1, port=1)` — a **control transfer to the disconnected hub**
- This control transfer goes through our `do_control_transfer()` → builds qTDs → links to async schedule → `TransferComplete.await`

**This is where the question is: does the control transfer complete or hang?**

## 3. What Happens When a Control Transfer Targets a Disconnected Device

### EHCI Behavior

When the EHCI controller processes a qTD in the async schedule targeting a disconnected device:

1. **SETUP token is sent** on the bus
2. **No device responds** (device is physically gone)
3. EHCI decrements the **CERR** (error counter) field in the qTD token. The counter starts at 3 (set in `qtd_token()`)
4. After 3 consecutive failures, CERR reaches 0 and the controller sets:
   - **Halted bit** (bit 6) = 1
   - **Transaction Error** (bit 3) = 1  
   - Clears **Active** (bit 7)
5. Controller generates a USB Error Interrupt (UEE, bit 1 of USBSTS)

### Our `TransferComplete` future

```rust
fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
    // ...
    let token = status_qtd.token.read();

    if token & QTD_TOKEN_ACTIVE != 0 {
        // Check data qTD early error...
        return Poll::Pending;
    }

    // Transfer complete — check for errors
    if token & ehci::QTD_TOKEN_ERROR_MASK != 0 {
        return Poll::Ready(Err(Imxrt1062HostController::map_qtd_error(token)));
    }
    // ...
}
```

**However**, there's a subtlety. The status qTD (the last in the chain) might still have Active=1 when the setup or data qTD halts. The EHCI controller **stops processing the chain** when a qTD halts — it does NOT advance to the next qTD. So the status qTD remains Active forever.

But we handle this! The `TransferComplete` future has early error detection:

```rust
if token & QTD_TOKEN_ACTIVE != 0 {
    // Still active — check if data qTD has errored (early exit)
    if let Some(data_qtd_ptr) = self.data_qtd {
        let data_qtd = unsafe { &*data_qtd_ptr };
        let data_token = data_qtd.token.read();
        if data_token & QTD_TOKEN_HALTED != 0 {
            return Poll::Ready(Err(...));
        }
    }
    return Poll::Pending;
}
```

**BUT** — for a control transfer with `DataPhase::In` (like GET_PORT_STATUS), the `data_qtd` field in `TransferComplete` is set. So:
- If the **setup qTD** halts (CERR=0, Halted=1, Active=0), the controller never advances to the data or status qTD
- The data qTD still has Active=1 (hasn't been processed) — **it is NOT halted**
- The status qTD still has Active=1
- The early error check only looks at the **data qTD** for Halted — the data qTD is Active, not Halted
- So `TransferComplete::poll()` says `status_qtd Active=1`, `data_qtd Active=1` (not halted) → returns `Poll::Pending`

### ⚠️ This means: the TransferComplete future gets stuck!

The setup qTD is halted (Active=0, Halted=1), but neither the status_qtd nor the data_qtd trigger the exit condition:
- `status_qtd` Active=1 → goes into the `if token & QTD_TOKEN_ACTIVE != 0` branch
- `data_qtd` Active=1, not Halted → early exit check doesn't trigger
- Returns `Poll::Pending`
- No further interrupts will wake it because the controller has stopped (the QH is halted in the async schedule — the controller skips it)

**The future hangs forever.**

Wait — let me re-examine. The EHCI async schedule processes QHs in a round-robin. When the controller encounters the QH whose overlay has halted, it should:
- Per EHCI spec §4.10.2: "If the status field of the qTD Token reports an error... the QH records the information... [and] the host controller moves on to the next QH"
- The overlay_token in the QH gets the halted setup qTD's token (Halted=1, Active=0)
- The setup qTD itself also has Halted=1, Active=0

Actually, let me reconsider the flow. The EHCI controller uses the **QH overlay**. When it fetches the setup qTD into the overlay, executes it, and it halts:
- The overlay_token = setup qTD token (Active=0, Halted=1)
- The controller writes back the overlay to the setup qTD
- The controller does NOT advance to the data qTD (halted state stops advancement)
- The setup qTD now has Active=0, Halted=1
- The data qTD still has the token as-written by software: Active=1, not halted — but the controller **never processes it**

In `TransferComplete`, we check `self.status_qtd` (which is the **last** qTD in the chain — the status phase qTD), and `self.data_qtd` (the middle one). The setup qTD is NOT directly checked.

So the status qTD token reads: Active=1 (never processed by controller). The data qTD token reads: Active=1 (never processed). Neither is halted. **`TransferComplete` returns `Pending` indefinitely.**

### Correction: does our ISR generate a wakeup?

The UEE (USB Error Interrupt) fires when a qTD halts. Our ISR wakes all pipe_wakers. But `TransferComplete` re-checks and still sees status_qtd Active=1, data_qtd not halted → Pending again.

**This is the bug: `TransferComplete` doesn't check the SETUP qTD for halt.**

Actually, for `DataPhase::None` transfers (like SET_FEATURE, CLEAR_FEATURE), there are only 2 qTDs: setup + status. The `data_qtd` field is `None`. If the setup qTD halts:
- status_qtd Active=1 (never processed)
- data_qtd is None → early error check skipped
- Returns Pending forever

**Same bug for `DataPhase::None` transfers.**

### What about `DataPhase::In` where data_qtd IS checked?

For GET_PORT_STATUS (DataPhase::In), `data_qtd` is `Some(...)`. But the data qTD has Active=1 (set by software, never cleared by controller because setup halted first). The early check is:
```rust
if data_token & QTD_TOKEN_HALTED != 0 {
    return Poll::Ready(Err(...));
}
```
Active=1 and NOT halted → doesn't trigger. Returns Pending.

## 4. Does cotton-usb-host Handle This and Continue?

**No, it gets stuck.** Specifically:

1. `handle_hub_packet` calls `get_hub_port_status` (control transfer to disconnected hub)
2. Control transfer awaits `TransferComplete` which never completes
3. The `.then()` combinator is blocked processing this item
4. The Root(Absent) event is queued but cannot be processed until the current `.then()` future completes
5. **The entire `device_events` stream is frozen**

The application never sees `DeviceEvent::Disconnect`. It hangs forever waiting for the control transfer to the disconnected hub.

## 5. Is There a Timeout Mechanism?

**In cotton-usb-host's trait**: `control_transfer` returns `Result<usize, UsbError>` where `UsbError::Timeout` exists. But there is **no timeout implemented in our EHCI driver**. The `TransferComplete` future has no timer; it relies purely on EHCI hardware signaling completion.

**In the RP2040 driver**: The RP2040 hardware has built-in rx_timeout detection in its SIE status registers — the hardware itself reports timeout. Our EHCI controller handles timeouts differently: it decrements CERR and eventually halts the qTD. But if the qTD that halts isn't the one we're monitoring, we miss it.

**The RP2040 driver `control_transfer_inner`** (rp2040.rs line 948) directly reads hardware status bits for each phase (setup, data, status) and returns errors immediately. It doesn't have the "monitor only the last qTD" problem.

## 6. Summary and Fix Required

### The bug

`TransferComplete` only monitors the **status qTD** (last in chain) and optionally the **data qTD** for early halt detection. It does NOT check the **setup qTD**. When the setup qTD halts because the device is disconnected, the data and status qTDs remain Active (never processed), and the future hangs.

### Fix options

**Option A: Check all qTDs in the chain**

Add the setup qTD pointer to `TransferComplete` and check it for halt:
```rust
if let Some(setup_qtd_ptr) = self.setup_qtd {
    let setup_token = unsafe { &*setup_qtd_ptr }.token.read();
    if setup_token & QTD_TOKEN_HALTED != 0 {
        return Poll::Ready(Err(map_qtd_error(setup_token)));
    }
}
```

**Option B: Check the QH overlay**

The QH overlay_token reflects the current/last executed qTD. When the setup qTD halts, the overlay_token has Halted=1. This is a single check:
```rust
let qh = unsafe { &*self.qh };
let overlay = qh.overlay_token.read();
if overlay & QTD_TOKEN_HALTED != 0 {
    return Poll::Ready(Err(map_qtd_error(overlay)));
}
```

This is simpler and catches halts at any stage (setup, data, or status).

**Option C: Add a timeout**

Wrap the TransferComplete future with a timeout (e.g., using the USB controller's GPT timer). If the transfer doesn't complete within N ms, return `Err(UsbError::Timeout)`. This is a defense-in-depth measure that handles all edge cases.

### Recommended approach

**Option B + Option C**: Check the QH overlay for halt status (catches the immediate case), and add a timeout as defense-in-depth (catches any other unforeseen hang scenarios).

### Impact on the race condition

With the fix, the control transfer to the disconnected hub returns `Err(UsbError::ProtocolError)` (from `map_qtd_error` with XACT_ERR bit set) within a few milliseconds (3 CERR retries). Then:
- `handle_hub_packet` propagates the error via `?`
- `.unwrap_or_else` produces `DeviceEvent::EnumerationError(0, 1, UsbError::ProtocolError)`
- Application ignores EnumerationError
- Next `events.next().await` polls the stream again
- `select` now yields Root(Absent)
- `DeviceEvent::Disconnect(0xFFFF_FFFF)` fires
- Application handles disconnect normally
