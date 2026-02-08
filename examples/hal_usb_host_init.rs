//! USB host controller initialisation example.
//!
//! Initialises the USB OTG2 controller in host mode on a Teensy 4.1 and blinks
//! the LED on success. Logging is performed over the USB1 device-mode CDC
//! serial port (the same USB port used for programming).
//!
//! # What it does
//!
//! 1. Sets up console logging over USB1 (CDC serial).
//! 2. Enables the USB2 PLL (`PLL_USB2` — 480 MHz) for the host controller.
//! 3. Constructs the USB host controller from USB2/USBPHY2 peripherals.
//! 4. Calls `init()` to perform the full EHCI initialisation sequence.
//! 5. Blinks the LED to indicate success.
//!
//! Connect a serial monitor to the Teensy's USB port to see the log messages.

#![no_std]
#![no_main]

use imxrt_hal as hal;
use imxrt_ral as ral;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Logging front-end (log crate).
const FRONTEND: board::logging::Frontend = board::logging::Frontend::Log;
/// Logging back-end — USB device (CDC serial over the programming port).
const BACKEND: board::logging::Backend = board::logging::Backend::Usbd;

/// LED blink interval after successful init (in PIT ticks).
/// 250 ms at 75 MHz PIT clock.
const BLINK_INTERVAL: u32 = board::PIT_FREQUENCY / 1_000 * 250;

// ---------------------------------------------------------------------------
// USB2 Peripherals bridge
// ---------------------------------------------------------------------------

/// Owns the USB OTG2 and USBPHY2 register instances and implements
/// the `imxrt_usbh::Peripherals` trait.
struct Usb2Peripherals {
    usb: ral::usb::USB2,
    usbphy: ral::usbphy::USBPHY2,
}

// SAFETY: We own the imxrt-ral singleton instances for USB2 and USBPHY2.
// No other code may access these registers while this struct exists.
unsafe impl imxrt_usbh::Peripherals for Usb2Peripherals {
    fn usb(&self) -> *const () {
        let rb: &ral::usb::RegisterBlock = &self.usb;
        (rb as *const ral::usb::RegisterBlock).cast()
    }
    fn usbphy(&self) -> *const () {
        let rb: &ral::usbphy::RegisterBlock = &self.usbphy;
        (rb as *const ral::usbphy::RegisterBlock).cast()
    }
}

// ---------------------------------------------------------------------------
// PLL_USB2 setup
// ---------------------------------------------------------------------------

/// Enable and lock the USB2 PLL (PLL7 / `CCM_ANALOG_PLL_USB2`).
///
/// This is the 480 MHz PLL that clocks the USB OTG2 controller and USBPHY2.
/// The sequence matches the USBHost_t36 `begin()` PLL setup, adapted for the
/// imxrt-ral register access layer.
///
/// The USBOH3 clock gate (CCGR6 CG0) is already enabled by the board crate's
/// `configure()` — it gates the clock for the entire USBOH3 module, which
/// includes both USB OTG1 and USB OTG2.
fn enable_usb2_pll() {
    // SAFETY: CCM_ANALOG is only accessed here during init, before interrupts
    // are enabled. No other code is racing on these registers at this point.
    let ccm_analog = unsafe { ral::ccm_analog::CCM_ANALOG::instance() };

    log::info!("Enabling USB2 PLL (PLL_USB2)...");

    // State-machine loop: bring up the PLL step by step, matching the
    // USBHost_t36 pattern. Each iteration checks one condition and takes
    // one corrective action, then re-checks from the top.
    loop {
        // If DIV_SELECT is set (528 MHz mode), clear it and start fresh.
        if ral::read_reg!(ral::ccm_analog, ccm_analog, PLL_USB2, DIV_SELECT == 1) {
            ral::write_reg!(ral::ccm_analog, ccm_analog, PLL_USB2_SET, BYPASS: 1);
            ral::write_reg!(ral::ccm_analog, ccm_analog, PLL_USB2_CLR,
                POWER: 1, DIV_SELECT: 1, ENABLE: 1, EN_USB_CLKS: 1);
            continue;
        }

        // Enable the PLL.
        if ral::read_reg!(ral::ccm_analog, ccm_analog, PLL_USB2, ENABLE == 0) {
            ral::write_reg!(ral::ccm_analog, ccm_analog, PLL_USB2_SET, ENABLE: 1);
            continue;
        }

        // Power up the PLL.
        if ral::read_reg!(ral::ccm_analog, ccm_analog, PLL_USB2, POWER == 0) {
            ral::write_reg!(ral::ccm_analog, ccm_analog, PLL_USB2_SET, POWER: 1);
            continue;
        }

        // Wait for PLL lock.
        if ral::read_reg!(ral::ccm_analog, ccm_analog, PLL_USB2, LOCK == 0) {
            continue;
        }

        // Disable bypass.
        if ral::read_reg!(ral::ccm_analog, ccm_analog, PLL_USB2, BYPASS == 1) {
            ral::write_reg!(ral::ccm_analog, ccm_analog, PLL_USB2_CLR, BYPASS: 1);
            continue;
        }

        // Enable USB clock outputs.
        if ral::read_reg!(ral::ccm_analog, ccm_analog, PLL_USB2, EN_USB_CLKS == 0) {
            ral::write_reg!(ral::ccm_analog, ccm_analog, PLL_USB2_SET, EN_USB_CLKS: 1);
            continue;
        }

        break; // PLL is up and running.
    }

    log::info!("USB2 PLL locked and running at 480 MHz");
}

// ---------------------------------------------------------------------------
// Static resources for the USB host controller
// ---------------------------------------------------------------------------

use imxrt_usbh::host::{UsbShared, UsbStatics};

/// Interrupt-safe shared state (lives in `.bss`).
static SHARED: UsbShared = UsbShared::new();

// We use a `static mut` for UsbStatics because it contains 4KB-aligned data
// that must have a stable address for DMA. In a real application you would
// use `ConstStaticCell` from the `static_cell` crate.
static mut STATICS: UsbStatics = UsbStatics::new();

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

#[imxrt_rt::entry]
fn main() -> ! {
    // ---- Board init (clocks, GPIO, logging) ----
    let (
        board::Common {
            pit: (_, _, mut blink_timer, _),
            usb1,
            usbnc1,
            usbphy1,
            mut dma,
            ..
        },
        board::Specifics { led, console, .. },
    ) = board::new();

    // Set up logging over USB1 device-mode CDC serial.
    let usbd = hal::usbd::Instances {
        usb: usb1,
        usbnc: usbnc1,
        usbphy: usbphy1,
    };
    let dma_a = dma[board::BOARD_DMA_A_INDEX].take().unwrap();
    let mut poller = board::logging::init(FRONTEND, BACKEND, console, dma_a, usbd);

    // Give the USB CDC serial a moment to enumerate so the host PC can
    // connect a serial monitor before we start printing.
    let startup_delay = board::PIT_FREQUENCY / 1_000 * 2_000; // 2 seconds
    blink_timer.set_load_timer_value(startup_delay);
    blink_timer.set_interrupt_enable(false);
    blink_timer.enable();
    while !blink_timer.is_elapsed() {
        poller.poll();
    }
    blink_timer.clear_elapsed();
    blink_timer.disable();

    log::info!("=== imxrt-usbh: USB Host Init Example ===");
    log::info!("Board initialised, logging over USB CDC serial");

    // ---- Step 1: Enable USB2 PLL ----
    enable_usb2_pll();

    // ---- Step 2: Acquire USB2 peripherals ----
    log::info!("Acquiring USB2 and USBPHY2 peripheral instances...");
    let peripherals = Usb2Peripherals {
        usb: unsafe { ral::usb::USB2::instance() },
        usbphy: unsafe { ral::usbphy::USBPHY2::instance() },
    };

    // ---- Step 3: Construct the host controller ----
    log::info!("Constructing USB host controller...");
    let statics: &'static UsbStatics = unsafe { &*core::ptr::addr_of!(STATICS) };
    let mut host = imxrt_usbh::host::Imxrt1062HostController::new(
        peripherals,
        &SHARED,
        statics,
    );

    // ---- Step 4: Initialise the hardware ----
    log::info!("Initialising EHCI host controller...");
    log::info!("  - PHY reset and power-up");
    log::info!("  - Controller reset");
    log::info!("  - Set host mode (CM=3)");
    log::info!("  - Configure async + periodic schedules");
    log::info!("  - Enable interrupts and run controller");
    unsafe { host.init() };
    log::info!("USB host controller initialised successfully!");

    // ---- Step 5: Blink LED to indicate success ----
    log::info!("Blinking LED at 4 Hz to indicate success");
    blink_timer.set_load_timer_value(BLINK_INTERVAL);
    blink_timer.set_interrupt_enable(false);
    blink_timer.enable();

    loop {
        poller.poll();
        if blink_timer.is_elapsed() {
            while blink_timer.is_elapsed() {
                blink_timer.clear_elapsed();
            }
            led.toggle();
        }
    }
}
