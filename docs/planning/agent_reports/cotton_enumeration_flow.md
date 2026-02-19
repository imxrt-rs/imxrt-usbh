# Reference: Cotton USB Host Enumeration Flow Analysis

**Date**: 2026-02-18
**Source**: Analysis of `cotton-usb-host` codebase at `C:\Users\tacer\GitHub\cotton\`

## 1. `device_events_no_hubs` Method Enumeration Sequence

**File:** `cotton-usb-host/src/usb_bus.rs`

**Lines 611-643:**
```rust
pub fn device_events_no_hubs<
    D: Future<Output = ()>,
    F: Fn(usize) -> D + 'static + Clone,
>(
    &self,
    delay_ms_in: F,
) -> impl Stream<Item = DeviceEvent> + '_ {
    let root_device = self.driver.device_detect();        // Line 618
    root_device.then(move |status| {
        let delay_ms = delay_ms_in.clone();
        async move {
            if let DeviceStatus::Present(speed) = status { // Line 622
                self.root_device_speed.set(Some(speed));    // Line 623
                self.driver.reset_root_port(true);          // Line 624: Assert reset
                delay_ms(50).await;                         // Line 625
                self.driver.reset_root_port(false);         // Line 626: Deassert reset
                delay_ms(10).await;                         // Line 627
                match self.new_device(speed).await {        // Line 628: Uses INITIAL speed
                    Ok((device, info)) => match self
                        .set_address(device, 1)
                        .await
                    {
                        Ok(device) => DeviceEvent::Connect(device, info),
                        Err(e) => DeviceEvent::EnumerationError(0, 1, e),
                    },
                    Err(e) => DeviceEvent::EnumerationError(0, 1, e),
                }
            } else {
                DeviceEvent::Disconnect(BitSet(0xFFFF_FFFF))
            }
        }
    })
}
```

**Key Enumeration Sequence:**
1. Device detect (line 618)
2. Port reset assertion (line 624) - 50ms delay
3. Port reset deassertion (line 626) - 10ms delay
4. Call `new_device(speed)` with the **initial detected speed** (line 628)
5. SET_ADDRESS (via `set_address()`)

## 2. Speed Detection - Does NOT Re-detect After Port Reset

**Critical Finding:** The code uses the speed from the initial `DeviceDetect` event and **does not re-detect speed after port reset**.

Looking at the RP2040 implementation:

**File:** `cotton-usb-host/src/host/rp2040.rs`

**Lines 125-164 (Rp2040DeviceDetect):**
```rust
impl Stream for Rp2040DeviceDetect {
    type Item = DeviceStatus;

    fn poll_next(
        mut self: Pin<&mut Self>,
        cx: &mut Context<'_>,
    ) -> Poll<Option<Self::Item>> {
        self.waker.register(cx.waker());

        let regs = unsafe { pac::USBCTRL_REGS::steal() };
        let status = regs.sie_status().read();
        let device_status = match status.speed().bits() {  // Lines 137-141
            0 => DeviceStatus::Absent,
            1 => DeviceStatus::Present(UsbSpeed::Low1_5),
            _ => DeviceStatus::Present(UsbSpeed::Full12),  // Note: NO High480 detection here!
        };

        if device_status != self.status {
            // ... returns the detected status
            Poll::Ready(Some(device_status))
        }
        // ...
    }
}
```

**Note:** The RP2040 `device_detect()` only detects **Low1_5 or Full12**. It does NOT detect **High480** (RP2040 doesn't support High Speed anyway).

## 3. packet_size Passed to First control_transfer (GET_DESCRIPTOR at address 0)

**File:** `cotton-usb-host/src/usb_bus.rs`

**Lines 698-765 (new_device function):**
```rust
async fn new_device(
    &self,
    speed: UsbSpeed,
) -> Result<(UnaddressedDevice, DeviceInfo), UsbError> {
    let transfer_extras = self.get_transfer_extras(speed);
    // Read prefix of device descriptor
    let mut descriptors = [0u8; 18];
    let sz = self
        .driver
        .control_transfer(
            0,                              // Line 708: address 0
            transfer_extras,
            8,                              // Line 710: PACKET SIZE = 8 BYTES!
            SetupPacket {
                bmRequestType: DEVICE_TO_HOST,
                bRequest: GET_DESCRIPTOR,
                wValue: ((DEVICE_DESCRIPTOR as u16) << 8),
                wIndex: 0,
                wLength: 8,
            },
            DataPhase::In(&mut descriptors),
        )
        .await?;
    if sz < 8 {
        debug::println!("control in {}/8", sz);
        return Err(UsbError::ProtocolError);
    }

    let packet_size_ep0 = descriptors[7];   // Line 726: Extracts real EP0 packet size

    // Fetch rest of device descriptor with the discovered packet size
    let sz = self
        .driver
        .control_transfer(
            0,
            transfer_extras,
            packet_size_ep0,                // Line 734: Uses discovered packet size
            SetupPacket {
                bmRequestType: DEVICE_TO_HOST,
                bRequest: GET_DESCRIPTOR,
                wValue: ((DEVICE_DESCRIPTOR as u16) << 8),
                wIndex: 0,
                wLength: 18,
            },
            DataPhase::In(&mut descriptors),
        )
        .await?;
```

**Key Point:** The first control transfer uses **packet_size=8** (line 710), which is a safe universal value that works for all USB speeds.

## 4. High Speed Handling: Hub Port Speed Detection vs Root Port

**File:** `cotton-usb-host/src/usb_bus.rs`

**Lines 1254-1264 (handle_hub_packet for downstream devices):**
```rust
if (state & 2) != 0 {
    // port is now ENABLED i.e. operational

    // USB 2.0 table 11-21
    let speed = match state & 0x600 {      // Lines 1258-1262
        0 => UsbSpeed::Full12,
        0x400 => UsbSpeed::High480,       // High Speed properly detected here!
        _ => UsbSpeed::Low1_5,
    };

    let (device, info) = self.new_device(speed).await?;
```

**Critical Contrast:**
- For **hub downstream ports**: Speed is correctly detected from the hub port status (including 0x400 = High480)
- For **root port**: Speed is NOT re-detected after port reset, only the initial speed is used

## 5. TransferExtras for Different Speeds

**File:** `cotton-usb-host/src/usb_bus.rs`

```rust
fn get_transfer_extras(&self, speed: UsbSpeed) -> TransferExtras {
    if speed == UsbSpeed::Low1_5 {
        TransferExtras::WithPreamble
    } else {
        TransferExtras::Normal
    }
}
```

Both Full Speed and High Speed use `TransferExtras::Normal`. Only Low Speed uses `WithPreamble`.

## 6. Implications for i.MX RT Host Controller

### Speed Mismatch Between cotton's View and Actual Port Speed

For our i.MX RT implementation:
- Before port reset: PSPD=0 (Full Speed) → `device_detect()` returns `Present(Full12)`
- After port reset: PSPD=2 (High Speed) → but cotton already recorded Full12
- Cotton calls `control_transfer(packet_size=8, TransferExtras::Normal)`

**This is actually OK** because:
1. Our `do_control_transfer()` reads the actual speed from `port_speed()` (PORTSC1.PSPD)
2. The QH is built with the correct HS speed regardless of what cotton thinks
3. `packet_size=8` works for the initial 8-byte GET_DESCRIPTOR at any speed
4. `TransferExtras::Normal` is correct for both FS and HS

### Double Detection Events

After port reset, the FS→HS transition sets CSC again, generating a second
`DeviceDetect` event: `Present(High480)`. Cotton's `.then()` combinator processes
events sequentially, so:
1. First event: `Present(Full12)` → enumeration attempt → fails → `EnumerationError`
2. Second event: `Present(High480)` → second enumeration attempt → also fails

This explains the two error events in the log.
