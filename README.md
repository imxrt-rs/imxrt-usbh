imxrt-usbh
=========

**Work in progress - not ready for use yet**

A USB host driver for i.MX RT processors. `imxrt-usbh` adapts [`cotton-usb-host`]
to the i.MX RT microcontroller family. The first goal is to get it working on
a Teensy 4.1, on the USB2 port.

[`cotton-usb-host`]: https://github.com/pdh11/cotton/tree/main/cotton-usb-host

Development
-----------

This repository contains the driver. The driver can build when targeting your
development system or an i.MX RT microcontroller.

When you make a change, you can check that it builds like this:

```
cargo build --features=imxrt-ral/imxrt1062
```

That built the library for your development system, not the MCU! This means you
can run unit and documentation tests on your development system.

```
cargo test --features=imxrt-ral/imxrt1062
```

To generate code for the MCU, append a target specifier. Hopefully, the code is
portable, so this shouldn't always be necessary.

```
cargo build --features=imxrt-ral/imxrt1062 --target=thumbv7em-none-eabihf
```

Hardware testing
----------------

This repository does not contain hardware examples. Those are in the [imxrt-hal]
repository. See the imxrt-hal documentation to learn how to run examples on your
board.

[imxrt-hal]: https://github.com/imxrt-rs/imxrt-hal

To test your changes, clone the imxrt-hal repository. In `imxrt-hal/Cargo.toml`,
find the `[patch.crates-io.imxrt-usbh]` directive, and point the patch to your
copy of this driver. Now, the changes you make _here_, in this driver, will be
incorporated into the hardware example you're running from imxrt-hal.

License
-------

Licensed under either of

- [Apache License, Version 2.0](http://www.apache.org/licenses/LICENSE-2.0) ([LICENSE-APACHE](./LICENSE-APACHE))
- [MIT License](http://opensource.org/licenses/MIT) ([LICENSE-MIT](./LICENSE-MIT))

at your option.

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.
