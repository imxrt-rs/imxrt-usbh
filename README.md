# imxrt-usbh

[![License: MIT OR Apache-2.0](https://img.shields.io/badge/License-MIT%20OR%20Apache--2.0-blue.svg)](https://opensource.org/licenses/MIT)

## A USB Host Controller Driver for i.MX RT Microcontrollers

`imxrt-usbh` provides a USB host controller implementation for NXP i.MX RT
1060/1062 microcontrollers (e.g. Teensy 4.x). It implements the
[`cotton-usb-host`] `HostController` trait, allowing the `cotton-usb-host`
stack to enumerate devices, read HID reports, transfer bulk data, and manage
USB hubs — all in async Rust with no allocator.

The driver targets the **EHCI-compatible USB OTG2 controller** in host mode
and is designed for use with [RTIC v2](https://rtic.rs).

**Note**: This crate targets the **secondary USB2 port** on the Teensy 4.0/4.1
(the host-capable port with a 5-pin header), not the primary USB1 OTG port
used for programming.

[`cotton-usb-host`]: https://crates.io/crates/cotton-usb-host

## Target Hardware

- **Primary target**: Teensy 4.1 (i.MX RT 1062) — USB2 host port (secondary 5-pin header)
- **Future targets**:
  - Teensy 4.0 (i.MX RT 1062) — USB2 host port (SMT pads on underside)
  - Other i.MX RT 1060/1062 boards with USB host capability

### Hardware Setup

The Teensy 4.1 USB2 host port uses a 5-pin header (directly behind the
Ethernet jack). You must supply **external 5V** to the VBUS pin — the host
port is not powered from the programming USB connector.

| Pin | Signal | Notes |
|-----|--------|-------|
| 1   | GND    |       |
| 2   | D+     |       |
| 3   | D-     |       |
| 4   | VBUS   | Connect to external 5V supply |
| 5   | ID     | Leave unconnected (host mode) |

Connect a USB device (keyboard, flash drive, or hub) via a USB-A breakout
wired to D+, D-, GND, and VBUS.

## Supported Devices

The driver supports **Low Speed** (1.5 Mbps), **Full Speed** (12 Mbps), and
**High Speed** (480 Mbps) USB devices. High Speed is available by default
for directly-connected devices. To use USB hubs, enable the `hub-support`
feature (see [Features](#features)), which forces all connections to Full
Speed.

Tested device types:
- HID keyboards (low-speed and full-speed)
- USB mass storage flash drives (full-speed)
- USB hubs with devices behind them (requires `hub-support` feature)

## Usage

Add the crate to your `Cargo.toml`:

```toml
[dependencies]
imxrt-usbh = { path = "../imxrt-usbh" }  # or git URL
cotton-usb-host = "0.2"
```

### Minimal Example

```rust,ignore
use imxrt_usbh::host::{Imxrt1062HostController, UsbShared, UsbStatics};
use cotton_usb_host::usb_bus::UsbBus;
use static_cell::ConstStaticCell;

// Static resources — must live for 'static
static SHARED: UsbShared = UsbShared::new();
static STATICS: ConstStaticCell<UsbStatics> = ConstStaticCell::new(UsbStatics::new());

// In your init code (after configuring USB2 PLL and clock gates):
let statics = STATICS.take();
let host = Imxrt1062HostController::new(peripherals, &SHARED, statics);
unsafe { host.init() };

// Create the USB bus and handle device events
let usb_bus = UsbBus::new(host);
let mut events = usb_bus.device_events_no_hubs();
// ... poll events in an async task
```

The `peripherals` argument must implement the [`Peripherals`](imxrt_usbh::Peripherals)
trait. See the trait documentation for how to provide USB register block pointers.

### Clock Prerequisites

Before calling `init()`, you must configure:

1. **USB2 PLL** (`CCM_ANALOG_PLL_USB2`) — enable and wait for lock
2. **USB OTG2 clock gate** (`CCM_CCGR6`) — un-gate the clock
3. **VBUS power GPIO** (Teensy 4.1: `GPIO_EMC_40` → HIGH via load switch)

See the RTIC examples for complete initialization sequences.

## Features

| Feature       | Default | Description |
|---------------|---------|-------------|
| `log`         | Yes     | Internal logging via the `log` crate (0.4) |
| `defmt-03`    | No      | Internal logging via `defmt` (0.3) — mutually exclusive with `log` |
| `hub-support` | No      | Enable USB hub support. Sets `PFSC=1`, forcing all connections to Full Speed (12 Mbps). Without this feature, devices can negotiate High Speed (480 Mbps) but hubs must not be used. |

To use `defmt` logging instead of `log`:

```toml
[dependencies]
imxrt-usbh = { path = "../imxrt-usbh", default-features = false, features = ["defmt-03"] }
```

To enable hub support (with `log` logging):

```toml
[dependencies]
imxrt-usbh = { path = "../imxrt-usbh", features = ["hub-support"] }
```

## Examples

All examples target the Teensy 4.1 and log over USB CDC serial on the
programming port (USB1). Build with:

```sh
cargo build --release --target thumbv7em-none-eabihf --example <name>
rust-objcopy -O ihex target/thumbv7em-none-eabihf/release/examples/<name> <name>.hex
```

Flash with:

```sh
teensy_loader_cli --mcu=TEENSY41 -w -v <name>.hex
```

| Example | Description |
|---------|-------------|
| [`rtic_usb_enumerate`](examples/rtic_usb_enumerate.rs) | Device detection and enumeration — logs VID/PID on connect |
| [`rtic_usb_hid_keyboard`](examples/rtic_usb_hid_keyboard.rs) | HID keyboard input — decodes keycodes, flashes LED in morse code (no hub support) |
| [`rtic_usb_hub`](examples/rtic_usb_hub.rs) | Hub enumeration — discovers devices behind a hub, logs HID reports (requires `--features hub-support`) |
| [`rtic_usb_mass_storage`](examples/rtic_usb_mass_storage.rs) | Mass storage sector read — SCSI READ(10) over bulk transport |

## Limitations

- **Hub support requires Full Speed**: With the `hub-support` feature, all
  connections are forced to Full Speed (12 Mbps) via the `PFSC` bit. High
  Speed hubs require EHCI split transaction scheduling, which is not yet
  implemented in `cotton-usb-host`.
- **No isochronous transfers**: Audio/video class devices are not supported.
- **No USB suspend/resume**: Power management is not implemented.
- **Single USB port**: Only USB OTG2 (the host port) is supported. USB OTG1
  (the programming port) is not usable as a host with this driver.

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  http://www.apache.org/licenses/LICENSE-2.0)
- MIT License ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.
