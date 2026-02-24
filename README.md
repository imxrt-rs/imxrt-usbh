# imxrt-usbh

[![CI](https://github.com/imxrt-rs/imxrt-usbh/actions/workflows/ci.yml/badge.svg)](https://github.com/imxrt-rs/imxrt-usbh/actions/workflows/ci.yml)
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
imxrt-usbh = "0.1"
imxrt-ral = { version = "0.6", features = ["imxrt1062"] }
```

> **Note:** `cotton-usb-host` types (`UsbBus`, `DeviceEvent`, descriptor types,
> etc.) are re-exported at the crate root, so you do **not** need to add
> `cotton-usb-host` as a direct dependency.

### Minimal Example

```rust,ignore
use imxrt_usbh::host::{Imxrt1062HostController, UsbShared, UsbStatics};
use imxrt_usbh::usb_bus::UsbBus;
use static_cell::ConstStaticCell;

// Static resources — must live for 'static
static SHARED: UsbShared = UsbShared::new();
static STATICS: ConstStaticCell<UsbStatics> = ConstStaticCell::new(UsbStatics::new());

// In your init code (after configuring USB2 PLL and clock gates):
let statics = STATICS.take();
let usb2 = unsafe { imxrt_ral::usb::USB2::instance() };
let usbphy2 = unsafe { imxrt_ral::usbphy::USBPHY2::instance() };
let host = Imxrt1062HostController::new(usb2, usbphy2, &SHARED, statics);
unsafe { host.init() };

// Create the USB bus and handle device events
let usb_bus = UsbBus::new(host);
let mut events = usb_bus.device_events_no_hubs();
// ... poll events in an async task
```

The `Imxrt1062HostController::new()` method takes `imxrt-ral` register instances
directly. Enable a chip feature on `imxrt-ral` (e.g. `imxrt1062`) to get the
correct register definitions.

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

Hardware examples live in the [`imxrt-hal`](https://github.com/imxrt-rs/imxrt-hal)
repository, which provides the board support infrastructure (clock setup, pin
muxing, logging). All examples target the **Teensy 4.1** and log over USB CDC serial.

Build from the `imxrt-hal` repo root:

```sh
cargo build --example rtic_usb_host_enumerate --features=board/teensy4 --target=thumbv7em-none-eabihf
```

Flash with:

```sh
rust-objcopy -O ihex target/thumbv7em-none-eabihf/debug/examples/rtic_usb_host_enumerate fw.hex
teensy_loader_cli --mcu=TEENSY41 -w -v fw.hex
```

| Example | Description |
|---------|-------------|
| `rtic_usb_host_enumerate` | Device detection and enumeration — logs VID/PID on connect |
| `rtic_usb_host_hid_keyboard` | HID keyboard input — decodes keycodes, flashes LED in morse code |
| `rtic_usb_host_hub` | Hub enumeration — discovers devices behind a hub (requires `imxrt-usbh/hub-support`) |
| `rtic_usb_host_mass_storage` | Mass storage sector read — SCSI READ(10) over bulk transport |

For local development, the `imxrt-hal` workspace uses a path dependency on
this crate. Changes to `imxrt-usbh` are picked up automatically when building
examples.

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
