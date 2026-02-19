//! USB host enumeration example using RTIC v2.
//!
//! Exercises phase 2a of the `imxrt-usbh` driver: device detection,
//! port reset, and control transfers (GET_DESCRIPTOR, SET_ADDRESS) via
//! the `cotton-usb-host` framework's [`UsbBus`] abstraction.
//!
//! # What it does
//!
//! 1. Sets up console logging over USB1 (CDC serial — the programming port).
//! 2. Waits 5 seconds in `idle` for the host PC to enumerate the CDC device
//!    and the serial monitor to connect.
//! 3. Enables the USB2 PLL (`PLL_USB2` — 480 MHz) for the host controller.
//! 4. Enables VBUS power on the USB2 host port (GPIO_EMC_40 → HIGH).
//! 5. Constructs and initialises `Imxrt1062HostController`.
//! 6. Installs the USB_OTG2 ISR with NVIC priority 0xE0 (RTIC logical
//!    priority 2) to stay within RTIC's BASEPRI-managed range.
//! 7. Wraps the host controller in `cotton_usb_host::usb_bus::UsbBus`.
//! 8. Spawns an async RTIC task that calls `UsbBus::device_events_no_hubs()`.
//! 9. On `DeviceEvent::Connect`, logs the VID/PID and device class.
//! 10. On `DeviceEvent::EnumerationError`, logs the error for debugging.
//!
//! The USB_OTG2 ISR calls `SHARED.on_usb_irq()` to wake async wakers on
//! port-change, transfer-complete, and error interrupts.
//!
//! # Init structure
//!
//! Only USB1 CDC logging is set up in RTIC init (interrupts disabled).
//! All USB2 host initialization happens in `idle` (interrupts enabled)
//! after a 5-second delay for serial monitor connectivity.
//!
//! # Expected output (with a USB keyboard plugged in)
//!
//! ```text
//! === imxrt-usbh: USB Enumerate Example ===
//! USB2 PLL locked
//! VBUS power enabled
//! USB host controller initialised
//! USB_OTG2 ISR installed (NVIC priority 0xE0)
//! Entering enumeration loop...
//! DeviceEvent::Connect  addr=1  VID=xxxx PID=xxxx class=xx subclass=xx
//! ```
//!
//! # Flash
//!
//! ```sh
//! cargo objcopy --example rtic_usb_enumerate --target thumbv7em-none-eabihf --release -- -O ihex fw.hex
//! teensy_loader_cli --mcu=TEENSY41 -w -v fw.hex
//! ```

#![no_std]
#![no_main]

#[rtic::app(device = board, peripherals = false, dispatchers = [BOARD_SWTASK0])]
mod app {
    use core::pin::pin;
    use cotton_usb_host::usb_bus::{DeviceEvent, UsbBus};
    use futures::StreamExt;
    use imxrt_hal as hal;
    use imxrt_ral as ral;
    use imxrt_usbh::host::{Imxrt1062HostController, UsbShared, UsbStatics};

    // -----------------------------------------------------------------------
    // Configuration
    // -----------------------------------------------------------------------

    /// Logging front-end (log crate).
    const FRONTEND: board::logging::Frontend = board::logging::Frontend::Log;
    /// Logging back-end — USB device (CDC serial over the programming port).
    const BACKEND: board::logging::Backend = board::logging::Backend::Usbd;

    /// USB OTG2 register block base address (for ISR access).
    const USB2_BASE: *const () = 0x402E_0200usize as *const ();

    /// NVIC priority for USB_OTG2 interrupt (RTIC logical priority 2).
    ///
    /// Must be within RTIC's BASEPRI-managed range to avoid deadlocks.
    /// On ARMv7-M, BASEPRI cannot mask priority 0 (the NVIC default),
    /// so we must explicitly set a lower-than-0 priority before unmasking.
    const USB2_NVIC_PRIORITY: u8 = 0xE0;

    // -----------------------------------------------------------------------
    // USB2 Peripherals bridge
    // -----------------------------------------------------------------------

    /// Owns the USB OTG2 and USBPHY2 register instances and implements
    /// the `imxrt_usbh::Peripherals` trait.
    struct Usb2Peripherals {
        usb: ral::usb::USB2,
        usbphy: ral::usbphy::USBPHY2,
    }

    // SAFETY: We own the imxrt-ral singleton instances for USB2 and USBPHY2.
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

    // -----------------------------------------------------------------------
    // PLL_USB2 setup
    // -----------------------------------------------------------------------

    fn enable_usb2_pll() {
        let ccm_analog = unsafe { ral::ccm_analog::CCM_ANALOG::instance() };
        loop {
            if ral::read_reg!(ral::ccm_analog, ccm_analog, PLL_USB2, DIV_SELECT == 1) {
                ral::write_reg!(ral::ccm_analog, ccm_analog, PLL_USB2_SET, BYPASS: 1);
                ral::write_reg!(ral::ccm_analog, ccm_analog, PLL_USB2_CLR,
                    POWER: 1, DIV_SELECT: 1, ENABLE: 1, EN_USB_CLKS: 1);
                continue;
            }
            if ral::read_reg!(ral::ccm_analog, ccm_analog, PLL_USB2, ENABLE == 0) {
                ral::write_reg!(ral::ccm_analog, ccm_analog, PLL_USB2_SET, ENABLE: 1);
                continue;
            }
            if ral::read_reg!(ral::ccm_analog, ccm_analog, PLL_USB2, POWER == 0) {
                ral::write_reg!(ral::ccm_analog, ccm_analog, PLL_USB2_SET, POWER: 1);
                continue;
            }
            if ral::read_reg!(ral::ccm_analog, ccm_analog, PLL_USB2, LOCK == 0) {
                continue;
            }
            if ral::read_reg!(ral::ccm_analog, ccm_analog, PLL_USB2, BYPASS == 1) {
                ral::write_reg!(ral::ccm_analog, ccm_analog, PLL_USB2_CLR, BYPASS: 1);
                continue;
            }
            if ral::read_reg!(ral::ccm_analog, ccm_analog, PLL_USB2, EN_USB_CLKS == 0) {
                ral::write_reg!(ral::ccm_analog, ccm_analog, PLL_USB2_SET, EN_USB_CLKS: 1);
                continue;
            }
            break;
        }
    }

    // -----------------------------------------------------------------------
    // VBUS power enable (Teensy 4.1 USB2 host port)
    // -----------------------------------------------------------------------

    fn enable_vbus_power() {
        let iomuxc = unsafe { ral::iomuxc::IOMUXC::instance() };
        ral::write_reg!(ral::iomuxc, iomuxc, SW_MUX_CTL_PAD_GPIO_EMC_40, 5);
        ral::write_reg!(ral::iomuxc, iomuxc, SW_PAD_CTL_PAD_GPIO_EMC_40, 0x0008);

        let iomuxc_gpr = unsafe { ral::iomuxc_gpr::IOMUXC_GPR::instance() };
        ral::modify_reg!(ral::iomuxc_gpr, iomuxc_gpr, GPR27, |v| v | (1 << 26));

        let gpio8 = unsafe { ral::gpio::GPIO8::instance() };
        ral::modify_reg!(ral::gpio, gpio8, GDIR, |v| v | (1 << 26));
        ral::write_reg!(ral::gpio, gpio8, DR_SET, 1 << 26);
    }

    // -----------------------------------------------------------------------
    // Blocking delay function for UsbBus::device_events_no_hubs
    // -----------------------------------------------------------------------

    /// Simple busy-wait delay that returns a ready future.
    ///
    /// This blocks the CPU for `ms` milliseconds using `cortex_m::asm::delay()`.
    /// Adequate for enumeration testing (total ~60ms), but not power-efficient.
    /// A proper implementation would use a monotonic timer.
    fn delay_ms(ms: usize) -> impl core::future::Future<Output = ()> {
        // Teensy 4.1: ARM Cortex-M7 at 600 MHz.
        // cortex_m::asm::delay() counts CPU cycles.
        cortex_m::asm::delay((ms as u32) * 600_000);
        core::future::ready(())
    }

    // -----------------------------------------------------------------------
    // Static resources
    // -----------------------------------------------------------------------

    /// Interrupt-safe shared state (lives in `.bss`).
    static SHARED: UsbShared = UsbShared::new();

    /// DMA resource pools. Uses `static mut` because the pool contains
    /// 4KB-aligned DMA structures that must have a stable address.
    static mut STATICS: UsbStatics = UsbStatics::new();

    // -----------------------------------------------------------------------
    // RTIC resources
    // -----------------------------------------------------------------------

    #[local]
    struct Local {}

    #[shared]
    struct Shared {
        /// The logging poller drives the USB CDC backend.
        poller: board::logging::Poller,
    }

    // -----------------------------------------------------------------------
    // Init — minimal: USB1 CDC logging only
    // -----------------------------------------------------------------------

    #[init]
    fn init(_cx: init::Context) -> (Shared, Local) {
        let (
            board::Common {
                usb1,
                usbnc1,
                usbphy1,
                mut dma,
                ..
            },
            board::Specifics { console, .. },
        ) = board::new();

        let usbd = hal::usbd::Instances {
            usb: usb1,
            usbnc: usbnc1,
            usbphy: usbphy1,
        };
        let dma_a = dma[board::BOARD_DMA_A_INDEX].take().unwrap();
        let poller = board::logging::init(FRONTEND, BACKEND, console, dma_a, usbd);

        (Shared { poller }, Local {})
    }

    // -----------------------------------------------------------------------
    // USB_OTG2 ISR (manually installed — not an RTIC task)
    // -----------------------------------------------------------------------

    /// USB OTG2 interrupt handler.
    ///
    /// Reads USBSTS, acknowledges pending bits, and wakes the appropriate
    /// async wakers in `UsbShared`. Installed into the vector table during
    /// idle via direct vector table patching.
    unsafe extern "C" fn usb2_isr() {
        SHARED.on_usb_irq(USB2_BASE);
    }

    // -----------------------------------------------------------------------
    // Idle — USB2 host init after serial monitor connects
    // -----------------------------------------------------------------------

    /// Runs after init returns with interrupts enabled.
    ///
    /// Waits 5 seconds for the USB1 CDC device to enumerate on the host
    /// PC (so the serial monitor can connect), then performs all USB2
    /// host controller initialization with full logging visibility.
    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        // 5-second delay — USB1 interrupts fire during this wait so the
        // CDC device enumerates on the host PC and the serial monitor
        // can connect.
        cortex_m::asm::delay(600_000 * 5_000);

        log::info!("=== imxrt-usbh: USB Enumerate Example ===");

        enable_usb2_pll();
        log::info!("USB2 PLL locked");

        enable_vbus_power();
        log::info!("VBUS power enabled");

        let peripherals = Usb2Peripherals {
            usb: unsafe { ral::usb::USB2::instance() },
            usbphy: unsafe { ral::usbphy::USBPHY2::instance() },
        };

        let statics: &'static UsbStatics = unsafe { &*core::ptr::addr_of!(STATICS) };
        let mut host = Imxrt1062HostController::new(peripherals, &SHARED, statics);
        unsafe { host.init() };
        log::info!("USB host controller initialised");

        // Install the USB_OTG2 interrupt handler.
        //
        // Order matters:
        //   1. Set NVIC priority (before unmasking, to avoid firing at priority 0)
        //   2. Patch vector table entry
        //   3. Memory barriers
        //   4. Unmask interrupt
        unsafe {
            let irq_num = ral::interrupt::USB_OTG2 as u32;
            core::ptr::write_volatile(
                (0xE000_E400 + irq_num) as *mut u8,
                USB2_NVIC_PRIORITY,
            );

            extern "C" {
                static __INTERRUPTS: [core::cell::UnsafeCell<unsafe extern "C" fn()>; 240];
            }
            let usb_otg2_irq = ral::interrupt::USB_OTG2 as usize;
            __INTERRUPTS[usb_otg2_irq]
                .get()
                .write_volatile(usb2_isr);

            cortex_m::asm::dsb();
            cortex_m::asm::isb();

            cortex_m::peripheral::NVIC::unmask(ral::interrupt::USB_OTG2);
        }
        log::info!(
            "USB_OTG2 ISR installed (NVIC priority 0x{:02X})",
            USB2_NVIC_PRIORITY
        );

        enumerate::spawn(host).ok();

        loop {
            cortex_m::asm::wfi();
        }
    }

    // -----------------------------------------------------------------------
    // USB1 ISR (for logging)
    // -----------------------------------------------------------------------

    #[task(binds = BOARD_USB1, shared = [poller], priority = 2)]
    fn usb1_interrupt(mut cx: usb1_interrupt::Context) {
        cx.shared.poller.lock(|poller| poller.poll());
    }

    #[task(binds = BOARD_DMA_A, shared = [poller], priority = 2)]
    fn dma_interrupt(mut cx: dma_interrupt::Context) {
        cx.shared.poller.lock(|poller| poller.poll());
    }

    // -----------------------------------------------------------------------
    // Enumeration task
    // -----------------------------------------------------------------------

    /// Async task that runs the cotton-usb-host device enumeration loop.
    ///
    /// Waits for device connect/disconnect events and logs the result.
    /// On `DeviceEvent::Connect`, the framework has already performed:
    ///   1. `device_detect()` → `DeviceStatus::Present(speed)`
    ///   2. `reset_root_port(true)` → 50ms delay → `reset_root_port(false)` → 10ms delay
    ///   3. `GET_DESCRIPTOR` to address 0 (8-byte read) → extracts max packet size
    ///   4. `SET_ADDRESS` → assigns address 1
    ///   5. `GET_DESCRIPTOR` to address 1 (full 18-byte read) → extracts VID/PID
    #[task(priority = 1)]
    async fn enumerate(_cx: enumerate::Context, host: Imxrt1062HostController) {
        log::info!("Entering enumeration loop...");

        let bus = UsbBus::new(host);
        let mut events = pin!(bus.device_events_no_hubs(delay_ms));

        loop {
            match events.next().await {
                Some(DeviceEvent::Connect(device, info)) => {
                    log::info!(
                        "DeviceEvent::Connect  addr={}  VID={:04x} PID={:04x} class={} subclass={}",
                        device.address(),
                        info.vid,
                        info.pid,
                        info.class,
                        info.subclass,
                    );
                }
                Some(DeviceEvent::Disconnect(_)) => {
                    log::info!("DeviceEvent::Disconnect");
                }
                Some(DeviceEvent::EnumerationError(hub, port, _err)) => {
                    log::warn!(
                        "DeviceEvent::EnumerationError  hub={} port={}",
                        hub, port,
                    );
                }
                Some(DeviceEvent::HubConnect(_)) => {
                    log::info!("DeviceEvent::HubConnect (unexpected in no-hubs mode)");
                }
                Some(DeviceEvent::None) => {}
                None => {
                    log::warn!("Device event stream ended unexpectedly");
                    break;
                }
            }
        }
    }
}
