# Phase 4: Testing and Validation

**Estimated effort**: 3-5 days  
**Key milestone**: All device types working

## 4.1 Compile-Time Validation

- [ ] Test QH/qTD structure layout: `assert!(core::mem::size_of::<QueueHead>() == 64)`
- [ ] Test alignment: `assert!(core::mem::align_of::<QueueHead>() == 64)`
- [ ] Test qTD size/alignment: `size_of == 32`, `align_of == 32`
- [ ] Verify `#[repr(C)]` field offsets match EHCI spec (use `core::mem::offset_of!` or `memoffset` crate)
- [ ] Test pool allocation/deallocation logic
- [ ] Test cache utility functions (clean, invalidate) on known memory regions

**Note**: Most testing requires actual hardware — the USB controller cannot be meaningfully mocked.

## 4.2 Hardware Bring-Up (Incremental)

Test in this order, each step building on the previous:

1. **[ ] Clock and PHY init** — verify no hard faults, USB PLL locks, USBPHY2 status register reads OK
2. **[ ] Host mode entry** — verify `USBMODE[CM]` reads back as host mode, `USBCMD[RS]` is set
3. **[ ] Device detection** — plug in a USB device, verify `device_detect()` stream yields `Present(speed)`
4. **[ ] Port reset** — reset the port, verify `PORTSC1[PE]` (Port Enabled) is set after reset
5. **[ ] First control transfer** — `GET_DESCRIPTOR(Device)` to address 0 (default address)
   - This is the "hello world" of USB host — if this works, the entire QH/qTD/cache/DMA pipeline is functional
6. **[ ] Device enumeration** — let `UsbBus` run full enumeration (SET_ADDRESS, GET_DESCRIPTOR, SET_CONFIGURATION)
7. **[ ] Bulk transfers** — read sectors from a USB mass storage device
8. **[ ] Interrupt transfers** — read key events from a USB HID keyboard
9. **[ ] Hot-plug/unplug** — verify clean handling of connect/disconnect events
10. **[ ] Low-speed device** — test with a LS device (e.g., basic USB mouse)
11. **[ ] Hub support** — test device behind a USB hub (requires `TransferExtras::WithPreamble` for LS-behind-FS)

## 4.3 Example Applications

- [ ] Create `examples/enumerate.rs` — minimal RTIC app that enumerates a USB device and prints descriptors via LPUART
  - Uses `board` crate for Teensy 4.1 BSP
  - Uses `imxrt-log` for LPUART logging
  - Demonstrates full initialization sequence
- [ ] Create `examples/keyboard.rs` — reads HID keyboard input (uses `cotton-usb-host-hid`)
- [ ] Create `examples/mass_storage.rs` — reads USB flash drive (uses `cotton-usb-host-msc`)
- [ ] Document hardware setup: Teensy 4.1 USB2 host port wiring (5-pin header: VBUS, D-, D+, ID, GND), power requirements

## Challenges for This Phase

### Challenge: Testing Without Device-Mode Loopback

**Problem**: Can't easily create automated tests — need real USB devices.

**Solution**:
- Create a structured bring-up sequence (section 4.2 above) that validates each layer incrementally
- First milestone: `GET_DESCRIPTOR(Device)` to address 0 — proves entire pipeline works
- Use LPUART logging (`imxrt-log`) extensively during development
- Use `defmt` for efficient structured logging
- Consider using a USB protocol analyzer (hardware tool) for debugging
- Compile-time tests for data structure layout and alignment
