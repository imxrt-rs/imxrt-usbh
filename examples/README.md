# Examples

## Prerequisites

- A Rust toolchain with the `thumbv7em-none-eabihf` target installed
- `rust-objcopy` (part of [`cargo-binutils`](https://github.com/rust-embedded/cargo-binutils))

```bash
rustup target add thumbv7em-none-eabihf
cargo install cargo-binutils
rustup component add llvm-tools
```

## Building

Build an example and convert the ELF to an Intel HEX file:

### hal_logging

```bash
cargo build --release --target thumbv7em-none-eabihf --example hal_logging
rust-objcopy -O ihex target/thumbv7em-none-eabihf/release/examples/hal_logging hal_logging.hex
```

### hal_usb_host_init

```bash
cargo build --release --target thumbv7em-none-eabihf --example hal_usb_host_init
rust-objcopy -O ihex target/thumbv7em-none-eabihf/release/examples/hal_usb_host_init hal_usb_host_init.hex
```

## Reading log output over USB

Both examples log over a USB CDC serial interface on the Teensy's programming
USB port. After flashing, the board enumerates as a USB serial device.

### hal_usb_host_init

This example uses the `log` crate frontend. The output is plain text, so any
serial monitor will work. Open the serial port at any baud rate (USB CDC
ignores baud rate settings):

```bash
# Linux / macOS (screen, minicom, or picocom)
picocom /dev/ttyACM0

# Windows (PuTTY, or the built-in mode command — replace COM3 with your port)
# Open Device Manager to find the COM port number.
```

### hal_logging

This example defaults to the `defmt` frontend, which encodes log messages in a
compact binary format. Use `defmt-print` to decode the serial stream:

```bash
cargo install defmt-print

# Linux / macOS
defmt-print -e target/thumbv7em-none-eabihf/release/examples/hal_logging < /dev/ttyACM0

# Windows MingW bash (replace ttyS4 with your port - note that tty ports are zero-based, ttyS4 = COM5)
dd if=/dev/ttyS4 bs=1 | defmt-print.exe -e target/thumbv7em-none-eabihf/release/examples/hal_logging
```

> **Tip:** You can switch `hal_logging` to plain-text output by changing the
> `FRONTEND` constant in the example source from `Frontend::Defmt` to
> `Frontend::Log`, then rebuilding.
