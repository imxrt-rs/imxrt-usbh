//! Minimal RTIC v2 heartbeat example.
//!
//! Boots the Teensy 4.1, sets up USB CDC logging over USB1 (the programming
//! port), and blinks the LED while printing a periodic log message. Use this
//! to verify that RTIC, the board crate, and USB CDC logging are all working
//! before testing more complex examples.
//!
//! # Expected output
//!
//! ```text
//! === imxrt-usbh: RTIC Heartbeat ===
//! heartbeat #0
//! heartbeat #1
//! ...
//! ```
//!
//! # Flash
//!
//! ```sh
//! cargo objcopy --example rtic_heartbeat --target thumbv7em-none-eabihf --release -- -O ihex fw.hex
//! teensy_loader_cli --mcu=TEENSY41 -w -v fw.hex
//! ```

#![no_std]
#![no_main]

#[rtic::app(device = board, peripherals = false, dispatchers = [BOARD_SWTASK0])]
mod app {
    use imxrt_hal as hal;

    /// Logging front-end.
    const FRONTEND: board::logging::Frontend = board::logging::Frontend::Log;
    /// Logging back-end — USB CDC serial over the programming port.
    const BACKEND: board::logging::Backend = board::logging::Backend::Usbd;
    /// LED toggle / log interval: 500 ms.
    const HEARTBEAT_MS: u32 = board::PIT_FREQUENCY / 1_000 * 500;

    #[local]
    struct Local {
        led: board::Led,
        timer: hal::pit::Pit<2>,
    }

    #[shared]
    struct Shared {
        poller: board::logging::Poller,
    }

    #[init]
    fn init(_cx: init::Context) -> (Shared, Local) {
        let (
            board::Common {
                pit: (_, _, mut timer, _),
                usb1,
                usbnc1,
                usbphy1,
                mut dma,
                ..
            },
            board::Specifics { led, console, .. },
        ) = board::new();

        // USB CDC logging setup.
        let usbd = hal::usbd::Instances {
            usb: usb1,
            usbnc: usbnc1,
            usbphy: usbphy1,
        };
        let dma_a = dma[board::BOARD_DMA_A_INDEX].take().unwrap();
        let poller = board::logging::init(FRONTEND, BACKEND, console, dma_a, usbd);

        // Heartbeat timer — fires every 500 ms.
        timer.set_load_timer_value(HEARTBEAT_MS);
        timer.set_interrupt_enable(true);
        timer.enable();

        (Shared { poller }, Local { led, timer })
    }

    // --- Logging interrupts ---

    #[task(binds = BOARD_USB1, priority = 1)]
    fn usb_interrupt(_cx: usb_interrupt::Context) {
        poll_logger::spawn().ok();
    }

    #[task(binds = BOARD_DMA_A, priority = 1)]
    fn dma_interrupt(_cx: dma_interrupt::Context) {
        poll_logger::spawn().ok();
    }

    #[task(shared = [poller], priority = 2)]
    async fn poll_logger(mut cx: poll_logger::Context) {
        cx.shared.poller.lock(|poller| poller.poll());
    }

    // --- Heartbeat ---

    #[task(binds = BOARD_PIT, local = [led, timer, counter: u32 = 0], priority = 1)]
    fn pit_interrupt(cx: pit_interrupt::Context) {
        let pit_interrupt::LocalResources {
            led,
            timer,
            counter,
            ..
        } = cx.local;

        if timer.is_elapsed() {
            while timer.is_elapsed() {
                timer.clear_elapsed();
            }
            led.toggle();
            log::info!("heartbeat #{counter}");
            *counter += 1;
        }
    }
}
