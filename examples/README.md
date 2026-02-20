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

```bash
cargo build --release --target thumbv7em-none-eabihf --example <name>
rust-objcopy -O ihex target/thumbv7em-none-eabihf/release/examples/<name> <name>.hex
```

For examples that require hub support, add `--features hub-support`:

```bash
cargo build --release --target thumbv7em-none-eabihf --features hub-support --example <name>
```

## Flashing

```bash
teensy_loader_cli --mcu=TEENSY41 -w -v <name>.hex
```

## Reading Log Output

All examples log over USB CDC serial on the Teensy's programming USB port
(USB1). After flashing, the board enumerates as a USB serial device.

All examples use plain-text logging via the `log` crate. Any serial monitor
will work — the baud rate is ignored (USB CDC):

```bash
# Linux / macOS
picocom /dev/ttyACM0

# Windows — use PuTTY, or open the COM port in Device Manager
```

## Examples

All examples use RTIC v2 and require a USB device connected to the Teensy
4.1's USB2 host port (5-pin header) with external 5V on the VBUS pin.

### rtic_usb_enumerate

Device detection and enumeration. Logs VID/PID and device class on connect.
Does not open any endpoints — useful for verifying basic USB host operation.

```bash
cargo build --release --target thumbv7em-none-eabihf --example rtic_usb_enumerate
```

### rtic_usb_hid_keyboard

HID keyboard input (no hub support). Enumerates a keyboard, opens an interrupt
IN pipe on its first interrupt endpoint, and logs decoded keycodes (letters,
modifiers, and up to 3 simultaneous keys). Flashes the on-board LED in morse
code for alphanumeric key presses. Panics if a hub is connected.

```bash
cargo build --release --target thumbv7em-none-eabihf --example rtic_usb_hid_keyboard
```

### rtic_usb_hub

Hub enumeration with HID input. Connects to a USB hub, discovers devices
behind it, and logs hub and device events. For HID devices behind the hub,
opens an interrupt IN pipe and logs raw reports. Requires `hub-support`.

```bash
cargo build --release --target thumbv7em-none-eabihf --features hub-support --example rtic_usb_hub
```

### rtic_usb_mass_storage

USB mass storage sector read. Enumerates a flash drive, finds bulk IN/OUT
endpoints, and reads sector 0 using a raw SCSI READ(10) over Bulk-Only
Transport (CBW/data/CSW). Logs the first 16 bytes of the sector.

```bash
cargo build --release --target thumbv7em-none-eabihf --example rtic_usb_mass_storage
```

