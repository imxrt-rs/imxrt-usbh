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
//! 3. Enables VBUS power on the USB2 host port (GPIO_EMC_40 → HIGH).
//! 4. Constructs the USB host controller from USB2/USBPHY2 peripherals.
//! 5. Calls `init()` to perform the full EHCI initialisation sequence.
//! 6. Blinks the LED to indicate success.
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
// USB2 register debug readback
// ---------------------------------------------------------------------------

/// USB OTG2 register block base address.
///
/// We use raw pointer access for USB2 debug reads because the USB2 RAL instance
/// is owned by the host controller after init. This is debug-only code.
const USB2_BASE: u32 = 0x402E_0200;

/// Read a USB2 register by raw pointer.
///
/// We use raw pointer access instead of the RAL because the USB2 instance is
/// owned by the host controller. This is only for debug register dumps.
unsafe fn usb2_reg(offset: usize) -> u32 {
    core::ptr::read_volatile((USB2_BASE as *const u8).add(offset) as *const u32)
}

// Register offsets within the USB OTG register block.
// (Derived from the `RegisterBlock` layout in src/ral/usb.rs.)
const OFF_USBCMD: usize        = 0x140;
const OFF_USBSTS: usize        = 0x144;
const OFF_USBINTR: usize       = 0x148;
const OFF_PORTSC1: usize       = 0x184;
const OFF_USBMODE: usize       = 0x1A8;
const OFF_ASYNCLISTADDR: usize = 0x158;

/// Log the key EHCI registers for debugging.
unsafe fn dump_usb2_registers() {
    let usbcmd  = usb2_reg(OFF_USBCMD);
    let usbsts  = usb2_reg(OFF_USBSTS);
    let usbintr = usb2_reg(OFF_USBINTR);
    let portsc  = usb2_reg(OFF_PORTSC1);
    let usbmode = usb2_reg(OFF_USBMODE);
    let asynclist = usb2_reg(OFF_ASYNCLISTADDR);

    log::info!("--- USB2 register dump ---");
    log::info!("  USBCMD        = {:#010X}", usbcmd);
    log::info!("    RS={} ASE={} PSE={} IAA={} ITC={}",
        usbcmd & 1, (usbcmd >> 5) & 1, (usbcmd >> 4) & 1,
        (usbcmd >> 6) & 1, (usbcmd >> 16) & 0xFF);
    log::info!("  USBSTS        = {:#010X}", usbsts);
    log::info!("    HCH={} PCI={} SEI={} AAI={} UI={} UEI={}",
        (usbsts >> 12) & 1, (usbsts >> 2) & 1, (usbsts >> 4) & 1,
        (usbsts >> 5) & 1, usbsts & 1, (usbsts >> 1) & 1);
    log::info!("  USBINTR       = {:#010X}", usbintr);
    log::info!("  PORTSC1       = {:#010X}", portsc);
    log_portsc(portsc);
    log::info!("  USBMODE       = {:#010X}  (CM={})", usbmode, usbmode & 3);
    log::info!("  ASYNCLISTADDR = {:#010X}", asynclist);
    log::info!("--- end register dump ---");
}

/// Decode and log PORTSC1 fields.
fn log_portsc(portsc: u32) {
    let ccs  = portsc & 1;           // Current Connect Status
    let csc  = (portsc >> 1) & 1;    // Connect Status Change
    let pe   = (portsc >> 2) & 1;    // Port Enabled
    let pec  = (portsc >> 3) & 1;    // Port Enable Change
    let pp   = (portsc >> 12) & 1;   // Port Power
    let pr   = (portsc >> 8) & 1;    // Port Reset
    let susp = (portsc >> 7) & 1;    // Suspend
    let pspd = (portsc >> 26) & 3;   // Port Speed
    let speed_str = match pspd {
        0 => "Full (12M)",
        1 => "Low (1.5M)",
        2 => "High (480M)",
        3 => "Not connected",
        _ => "??",
    };
    log::info!("    CCS={} CSC={} PE={} PEC={} PP={} PR={} SUSP={} PSPD={} ({})",
        ccs, csc, pe, pec, pp, pr, susp, pspd, speed_str);
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

    // Give the USB CDC serial time to enumerate so the host PC can
    // connect a serial monitor before we start printing.
    let startup_delay = board::PIT_FREQUENCY / 1_000 * 5_000; // 5 seconds
    blink_timer.set_load_timer_value(startup_delay);
    blink_timer.set_interrupt_enable(false);
    blink_timer.enable();
    while !blink_timer.is_elapsed() {
        poller.poll();
    }
    blink_timer.clear_elapsed();
    blink_timer.disable();

    // Helper: flush the log buffer by polling USB CDC for a fixed duration.
    // The logging backend uses a 1024-byte bbqueue ring buffer that can only
    // send ~512 bytes per poll() call (one USB packet). If we log faster than
    // poll() can drain, writes are silently dropped. This helper busy-loops
    // for `ms` milliseconds, giving the USB stack time to transfer everything.
    let flush_log = |poller: &mut board::logging::Poller, timer: &mut _, ms: u32| {
        use hal::pit::Pit;
        let ticks = board::PIT_FREQUENCY / 1_000 * ms;
        Pit::set_load_timer_value(timer, ticks);
        Pit::set_interrupt_enable(timer, false);
        Pit::enable(timer);
        while !Pit::is_elapsed(timer) {
            poller.poll();
        }
        Pit::clear_elapsed(timer);
        Pit::disable(timer);
    };

    log::info!("=== imxrt-usbh: USB Host Init Example ===");
    log::info!("=== Build version 1.06 ===");
    log::info!("Board initialised, logging over USB CDC serial");
    flush_log(&mut poller, &mut blink_timer, 50);

    // ---- Step 1: Enable USB2 PLL ----
    enable_usb2_pll();
    flush_log(&mut poller, &mut blink_timer, 50);

    // ---- Step 2: Enable VBUS power (Teensy 4.1) ----
    //
    // The Teensy 4.1 USB2 host port has an on-board load switch that gates 5V
    // to the USB connector. The switch is controlled by GPIO_EMC_40 (ALT5 =
    // GPIO3_IO26 / fast GPIO8_IO26). We must drive it HIGH to supply VBUS power.
    //
    // Without this, the connected device receives no power at all.
    //
    // Reference: USBHost_t36 ehci.cpp lines 209-212 (uses GPIO8, the fast bank).
    log::info!("Enabling VBUS power (GPIO_EMC_40 → HIGH)...");
    
    // IOMUXC pad mux: GPIO_EMC_40 → ALT5 (GPIO3_IO26 / GPIO8_IO26)
    let iomuxc = unsafe { ral::iomuxc::IOMUXC::instance() };
    ral::write_reg!(ral::iomuxc, iomuxc, SW_MUX_CTL_PAD_GPIO_EMC_40, 5);
    ral::write_reg!(ral::iomuxc, iomuxc, SW_PAD_CTL_PAD_GPIO_EMC_40, 0x0008);  // slow slew, weak drive
    
    // Enable fast GPIO routing for GPIO3→GPIO8.
    // IOMUXC_GPR_GPR27 controls which bank drives GPIO3 pins: each bit selects
    // between the regular GPIO3 bank (0) and the fast GPIO8 bank (1).
    // Teensyduino's startup code sets this to 0xFFFFFFFF; without it, GPIO8
    // writes have no effect on the physical pin.
    let iomuxc_gpr = unsafe { ral::iomuxc_gpr::IOMUXC_GPR::instance() };
    ral::modify_reg!(ral::iomuxc_gpr, iomuxc_gpr, GPR27, |v| v | (1 << 26));
    
    // Use the RAL for GPIO8 — type-safe access with correct base address (0x4200_8000)
    let gpio8 = unsafe { ral::gpio::GPIO8::instance() };
    ral::modify_reg!(ral::gpio, gpio8, GDIR, |v| v | (1 << 26));  // bit 26 = output
    ral::write_reg!(ral::gpio, gpio8, DR_SET, 1 << 26);           // drive HIGH
    
    // Readback verification — confirm the writes took effect
    let mux_readback = ral::read_reg!(ral::iomuxc, iomuxc, SW_MUX_CTL_PAD_GPIO_EMC_40);
    let pad_readback = ral::read_reg!(ral::iomuxc, iomuxc, SW_PAD_CTL_PAD_GPIO_EMC_40);
    let gpr27_readback = ral::read_reg!(ral::iomuxc_gpr, iomuxc_gpr, GPR27);
    let gdir_readback = ral::read_reg!(ral::gpio, gpio8, GDIR);
    let dr_readback = ral::read_reg!(ral::gpio, gpio8, DR);
    log::info!("  IOMUXC MUX  = {:#010X} (expect 5)", mux_readback);
    log::info!("  IOMUXC PAD  = {:#010X} (expect 0x0008)", pad_readback);
    log::info!("  GPR27       = {:#010X} (bit 26 routes GPIO3→GPIO8)", gpr27_readback);
    log::info!("  GPIO8 GDIR  = {:#010X} (bit 26 = {:#010X})", gdir_readback, 1u32 << 26);
    log::info!("  GPIO8 DR    = {:#010X} (bit 26 = {:#010X})", dr_readback, 1u32 << 26);
    
    log::info!("VBUS power enabled");
    flush_log(&mut poller, &mut blink_timer, 50);

    // ---- Step 3: Acquire USB2 peripherals ----
    log::info!("Acquiring USB2 and USBPHY2 peripheral instances...");
    let peripherals = Usb2Peripherals {
        usb: unsafe { ral::usb::USB2::instance() },
        usbphy: unsafe { ral::usbphy::USBPHY2::instance() },
    };

    // ---- Step 4: Construct the host controller ----
    log::info!("Constructing USB host controller...");
    flush_log(&mut poller, &mut blink_timer, 50);
    let statics: &'static UsbStatics = unsafe { &*core::ptr::addr_of!(STATICS) };
    let mut host = imxrt_usbh::host::Imxrt1062HostController::new(
        peripherals,
        &SHARED,
        statics,
    );

    // ---- Step 5: Initialise the hardware ----
    log::info!("Initialising EHCI host controller...");
    log::info!("  - PHY reset and power-up");
    log::info!("  - Controller reset");
    log::info!("  - Set host mode (CM=3)");
    log::info!("  - Configure async + periodic schedules");
    log::info!("  - Enable interrupts and run controller");
    unsafe { host.init() };
    log::info!("USB host controller initialised successfully!");
    flush_log(&mut poller, &mut blink_timer, 50);

    // ---- Step 5b: Dump registers to verify init ----
    unsafe { dump_usb2_registers() };
    flush_log(&mut poller, &mut blink_timer, 50);

    // Log DMA structure alignment for sanity check.
    let sentinel_addr = &statics.qh_pool[0] as *const _ as u32;
    let frame_list_addr = &statics.frame_list as *const _ as u32;
    log::info!("Sentinel QH addr  = {:#010X} (64B-aligned: {})",
        sentinel_addr, sentinel_addr % 64 == 0);
    log::info!("Frame list addr   = {:#010X} (4096B-aligned: {})",
        frame_list_addr, frame_list_addr % 4096 == 0);
    flush_log(&mut poller, &mut blink_timer, 50);

    // ---- Step 6: Blink LED + poll PORTSC1 for connect/disconnect ----
    log::info!("Entering main loop — polling PORTSC1 for device events");
    log::info!("Blinking LED at 4 Hz to indicate success");
    blink_timer.set_load_timer_value(BLINK_INTERVAL);
    blink_timer.set_interrupt_enable(false);
    blink_timer.enable();

    // Track previous connect status so we only log transitions.
    let mut prev_ccs: u32 = 0;

    loop {
        poller.poll();

        // Check PORTSC1 for connect/disconnect events.
        let portsc = unsafe { usb2_reg(OFF_PORTSC1) };
        let ccs = portsc & 1; // Current Connect Status
        let csc = (portsc >> 1) & 1; // Connect Status Change (W1C)

        if csc != 0 {
            // Clear the Connect Status Change bit (W1C) — write 1 to bit 1
            // while writing 0 to all other W1C bits to avoid clearing them.
            // W1C bits in PORTSC1: CSC(1), PEC(3), OCC(5), FPR(6).
            // We must also preserve the non-W1C bits, so we read, mask off
            // all W1C bits, then set only CSC.
            //
            // Note: Using raw pointer because USB2 instance is owned by host controller.
            let w1c_mask: u32 = (1 << 1) | (1 << 3) | (1 << 5) | (1 << 6);
            let clear_val = (portsc & !w1c_mask) | (1 << 1); // set CSC only
            unsafe {
                core::ptr::write_volatile(
                    (USB2_BASE as *mut u8).add(OFF_PORTSC1) as *mut u32,
                    clear_val,
                );
            }
        }

        if ccs != prev_ccs {
            if ccs != 0 {
                log::info!(">>> DEVICE CONNECTED <<<");
                log_portsc(portsc);
                // Full register dump on connect for debugging.
                unsafe { dump_usb2_registers() };
            } else {
                log::info!(">>> DEVICE DISCONNECTED <<<");
                log_portsc(portsc);
            }
            prev_ccs = ccs;
        }

        if blink_timer.is_elapsed() {
            while blink_timer.is_elapsed() {
                blink_timer.clear_elapsed();
            }
            led.toggle();
            //log::info!("Toggle LED");
        }
    }
}
