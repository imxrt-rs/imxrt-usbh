# Hub Support Research: cotton-usb-host & imxrt-usbh

## 1. `device_events()` vs `device_events_no_hubs()`

### `device_events_no_hubs()` (simple path)
**Location:** [usb_bus.rs](cotton-usb-host/src/usb_bus.rs) lines 611–646

Listens only on `driver.device_detect()` (root port). On `DeviceStatus::Present(speed)`:
1. Records root device speed
2. Resets root port (50ms + 10ms delay)
3. Calls `new_device(speed)` — reads device descriptor at address 0
4. Calls `set_address(device, 1)` — always address 1
5. Yields `DeviceEvent::Connect(device, info)`

No hub awareness. Doesn't check `class == 9`. Doesn't allocate interrupt pipes. Doesn't handle hub state changes.

### `device_events()` (hub-aware path)
**Location:** [usb_bus.rs](cotton-usb-host/src/usb_bus.rs) lines 479–571

Uses `futures::stream::select()` to merge TWO event sources:
1. `driver.device_detect()` → `InternalEvent::Root(DeviceStatus)` 
2. `HubStateStream { state: hub_state }` → `InternalEvent::Packet(InterruptPacket)`

**Root device handling (InternalEvent::Root):**
Same reset/enumerate flow, but adds:
- Checks `info.class == HUB_CLASSCODE` (class 9)
- If hub: calls `topology.device_connect(0, 1, is_hub)` for address assignment
- Calls `set_address(device, address)` (may be >1 due to topology)
- If hub: calls `self.new_hub(hub_state, device)` → yields `DeviceEvent::HubConnect`
- If not hub: yields `DeviceEvent::Connect(device, info)` as before

**Hub interrupt handling (InternalEvent::Packet):**
Dispatches to `self.handle_hub_packet(hub_state, &packet, delay_ms)` which:
- Parses the port bitmap from the interrupt data
- For each flagged port, calls `get_hub_port_status(hub_address, port)` 
- Clears the change bit via `clear_port_feature`
- If connection change and now connected: resets port, enumerates, assigns address
- If connection change and now disconnected: removes from topology, yields `DeviceEvent::Disconnect` with bitmap of all downstream devices
- Recursively detects hubs-behind-hubs

## 2. `TransferExtras::WithPreamble`

**Location:** [host_controller.rs](cotton-usb-host/src/host_controller.rs) lines 87–93

```rust
pub enum TransferExtras {
    /// Normal transfer
    Normal,
    /// Low-speed transfer to a high-speed hub (USB 2.0 s8.6.5)
    WithPreamble,
}
```

It's a simple fieldless enum — no extra data (no hub address, no port number). Just a flag.

**How it's determined** — [usb_bus.rs](cotton-usb-host/src/usb_bus.rs) lines 649–658:

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

Logic: If root port negotiated FS and the target device is LS, there MUST be a hub in between, so preamble is needed. No need to track the hub topology — it's a simple boolean deduction.

## 3. RP2040 Implementation of `WithPreamble`

### Control transfer
**Location:** [rp2040.rs](cotton-usb-host/src/host/rp2040.rs) lines 882–888

```rust
self.regs.sie_ctrl().modify(|_, w| {
    w.receive_data().clear_bit();
    w.send_data().clear_bit();
    w.preamble_en()
        .bit(transfer_extras == TransferExtras::WithPreamble);
    w.send_setup().set_bit()
});
```

It's a single hardware bit (`preamble_en`) in the SIE control register. The RP2040 hardware handles the actual preamble PID generation.

### Interrupt pipe
**Location:** [rp2040.rs](cotton-usb-host/src/host/rp2040.rs) lines 1249–1262

```rust
regs.host_addr_endp((n - 1) as usize).write(|w| unsafe {
    w.address()
        .bits(address)
        .endpoint()
        .bits(endpoint)
        .intep_preamble()
        .bit(transfer_extras == TransferExtras::WithPreamble)
        .intep_dir()
        .clear_bit() // IN
});
```

Again a single hardware bit (`intep_preamble`) in the endpoint register for interrupt pipes.

## 4. imxrt-usbh (EHCI) Current Handling of `WithPreamble`

### control_transfer
**Location:** [host.rs](imxrt-usbh/src/host.rs) lines 1005–1020

```rust
let capabilities = match transfer_extras {
    TransferExtras::Normal => ehci::qh_capabilities(0, 0, 0, 0, 1),
    TransferExtras::WithPreamble => {
        // For split transactions, we need the hub address and port.
        // WithPreamble is used for LS devices behind FS hubs.
        // In EHCI, split transactions require hub_addr and hub_port
        // in the QH capabilities, plus S-mask/C-mask.
        // For now, set default values — proper hub support requires
        // additional context from the caller.
        ehci::qh_capabilities(0, 0, 0, 0, 1)
    }
};
```

**Currently a no-op** — WithPreamble produces the same capabilities as Normal. The comment acknowledges that EHCI split transactions require `hub_addr` and `hub_port` but these aren't available from the trait interface.

### alloc_interrupt_pipe / try_alloc_interrupt_pipe
**Location:** [host.rs](imxrt-usbh/src/host.rs) lines 1450–1458

```rust
fn do_alloc_interrupt_pipe(
    &self,
    pipe: Pipe,
    address: u8,
    _transfer_extras: TransferExtras,   // <-- IGNORED with underscore prefix
    endpoint: u8,
    max_packet_size: u16,
    _interval_ms: u8,                    // <-- also ignored
) -> Imxrt1062InterruptPipe {
```

`transfer_extras` is completely ignored. The QH capabilities are hardcoded:
```rust
let capabilities = ehci::qh_capabilities(0x01, 0, 0, 0, 1);
// S-mask = 0x01, C-mask = 0, hub_addr = 0, hub_port = 0
```

### QH capabilities function
**Location:** [ehci.rs](imxrt-usbh/src/ehci.rs) lines 423–444

```rust
pub const fn qh_capabilities(
    smask: u8, cmask: u8, hub_addr: u8, hub_port: u8, mult: u8,
) -> u32
```

The hardware infrastructure for split transactions IS present — `hub_addr` and `hub_port` fields exist in the QH capabilities word. But they're never populated with real values.

### EHCI Split Transaction Requirements (for future hub support)
For EHCI to talk to FS/LS devices behind a HS hub, it needs **split transactions**:
- QH characteristics must set `EPS` field to the device's actual speed (LS or FS)
- QH capabilities must contain the **TT hub address** and **TT hub port**
- `S-mask` (start-split) and `C-mask` (complete-split) must be set in capabilities
- The `C` (control endpoint) bit in characteristics enables certain FS/LS scheduling rules

**Key issue**: The `TransferExtras::WithPreamble` enum carries NO hub address/port. The RP2040 doesn't need them (its hardware handles preamble PID natively). EHCI needs them for split transactions. This is a gap in the trait interface for EHCI-based implementations.

**However**: The Teensy 4.1 USB2 host port operates in host mode but is **not** a high-speed hub — it's a root port. If the root port negotiates Full Speed (because a FS hub is connected), then all downstream LS devices behind that FS hub are handled by FS preamble, not EHCI split transactions. EHCI split transactions are only needed when the root port is HS and there's a FS/LS device behind a HS hub.

**Practical implication**: Since the Teensy 4.1 USB2 port appears to negotiate FS with FS hubs (not HS), `WithPreamble` may actually need the EHCI `EPS` field set to Low Speed but does NOT need split transaction hub_addr/hub_port. The key thing is setting the correct device speed in the QH.

## 5. Hub Enumeration Flow in cotton-usb-host

When `device_events()` detects a root device with `class == 9` (HUB_CLASSCODE):

### `new_hub()` — [usb_bus.rs](cotton-usb-host/src/usb_bus.rs) lines 1042–1110
1. `get_basic_configuration()` — reads config descriptors
2. `configure(device, bc.configuration_value)` — sets configuration (moves to Configured state)
3. **`hub_state.try_add()`** — allocates an interrupt pipe for the hub's status-change endpoint:
   ```rust
   hub_state.try_add(
       &self.driver,
       device.address(),
       bc.in_endpoints.trailing_zeros() as u8,  // first IN endpoint
       device.packet_size_ep0,
       9,  // poll interval 9ms
   )?;
   ```
   This calls `hc.try_alloc_interrupt_pipe(address, TransferExtras::Normal, endpoint, max_packet_size, interval_ms)`

4. Reads the **Hub Descriptor** (class-specific GET_DESCRIPTOR with type 0x29)
5. Extracts `ports` count from descriptor byte 2
6. **Powers up each port**: `set_port_feature(address, port, PORT_POWER)` for port 1..=N
7. Returns `UsbDevice` (the hub itself), yielded as `DeviceEvent::HubConnect`

### Downstream device detection
After the hub is configured and its interrupt pipe is active, `HubStateStream` polls all active hub interrupt pipes. When a hub sends a status-change interrupt:

**`handle_hub_packet()`** — [usb_bus.rs](cotton-usb-host/src/usb_bus.rs) lines 1177–1284
1. Parses port bitmap from interrupt data
2. For each flagged port:
   - `get_hub_port_status(hub_address, port)` — class-specific GET_STATUS
   - If `C_PORT_CONNECTION` change with connection:
     - `set_port_feature(hub_address, port, PORT_RESET)` — reset that port
     - Wait 50ms
     - `get_hub_port_status()` again to check if port is now enabled
     - Determine speed from port status bits
     - `new_device(speed)` — enumerate at address 0
     - If the new device is also a hub → recurse (calls `new_hub()` again)
     - Otherwise → `set_address()` and yield `DeviceEvent::Connect`
   - If disconnection: `topology.device_disconnect()` → yield `DeviceEvent::Disconnect`

## 6. Trait Methods Called by `device_events()` but NOT by `device_events_no_hubs()`

| Method | `device_events_no_hubs` | `device_events` |
|--------|------------------------|-----------------|
| `device_detect()` | ✅ | ✅ |
| `reset_root_port()` | ✅ | ✅ |
| `control_transfer()` (addr 0, GET_DESCRIPTOR) | ✅ | ✅ |
| `control_transfer()` (SET_ADDRESS) | ✅ | ✅ |
| `control_transfer()` (SET_CONFIGURATION) | ❌ | ✅ (for hubs) |
| `control_transfer()` (class-specific GET_DESCRIPTOR for hub) | ❌ | ✅ |
| `control_transfer()` (SET_FEATURE PORT_POWER) | ❌ | ✅ |
| `control_transfer()` (GET_STATUS per port) | ❌ | ✅ |
| `control_transfer()` (CLEAR_FEATURE C_PORT_xxx) | ❌ | ✅ |
| `control_transfer()` (SET_FEATURE PORT_RESET) | ❌ | ✅ |
| **`try_alloc_interrupt_pipe()`** | ❌ | ✅ (hub status-change EP) |
| `alloc_interrupt_pipe()` | ❌ | ❌ (uses try_ variant) |

**Key finding**: `device_events()` calls `try_alloc_interrupt_pipe()` (not `alloc_interrupt_pipe()`) for hub status-change endpoints. It always passes `TransferExtras::Normal` for the hub's own interrupt pipe (the hub itself is FS, not behind another hub in the simple case).

The `TransferExtras::WithPreamble` only shows up when doing control transfers to LS devices that are physically behind a FS hub — determined by `get_transfer_extras()` which checks root_device_speed vs target device speed.
