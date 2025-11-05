# imxrt-usbh

[![License: MIT OR Apache-2.0](https://img.shields.io/badge/License-MIT%20OR%20Apache--2.0-blue.svg)](https://opensource.org/licenses/MIT)

## A USB Host Controller Driver for i.MX RT Microcontrollers

**Work in progress - not ready for use yet**

`imxrt-usbh` provides a hardware-specific implementation of the [`cotton-usb-host`] 
`HostController` trait for NXP i.MX RT microcontrollers, specifically targeting the 
Teensy 4.x series (i.MX RT 1060/1062).

This crate enables USB host functionality on i.MX RT devices, allowing them to 
communicate with USB peripherals such as keyboards, mass storage devices, hubs, 
and other USB accessories.

**Note**: This package targets the **secondary USB2 port** on the Teensy 4.0/4.1 (the 
host-capable port), not the primary USB1 OTG port. It uses standard USB host 
protocol rather than USB OTG.

This crate implements the `cotton-usb-host` `HostController` trait and is initially being
tested with the RTIC execution framework.

## Target Hardware

- **Primary target**: Teensy 4.1 (i.MX RT 1062) USB2 host port (secondary port with 5-pin header)
- **Future targets**: 
  - Teensy 4.0 (i.MX RT 1062) USB2 host port (secondary port on SMT pads)
  - Other i.MX RT 1060/1062 development boards with USB host capability
  - Other i.MX RT processor families (?)



## Usage

This crate is intended to be used with the [`cotton-usb-host`] USB stack:

```rust
use imxrt_usbh::Imxrt1062HostController;
use cotton_usb_host::usb_bus::UsbBus;

// Initialize the host controller
let host_controller = Imxrt1062HostController::new(
    usb_instance,
    usbphy_instance,
    shared_state,
    static_resources,
);

// Create the USB bus
let usb_bus = UsbBus::new(host_controller);

// Handle device events
let mut events = usb_bus.device_events().await;
// ...
```



## Status

This is currently in the planning phase. 

## Examples

_(Coming soon)_


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
