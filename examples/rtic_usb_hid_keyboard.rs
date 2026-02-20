//! HID keyboard input example using RTIC v2 — Phase 2b validation.
//!
//! Enumerates a USB HID keyboard on the USB2 host port, allocates an
//! interrupt IN pipe on its first interrupt endpoint, and logs each raw
//! HID report received.
//!
//! # What it does
//!
//! 1. Initialises USB2 host controller (same as `rtic_usb_enumerate`).
//! 2. Waits for a device connect event via `UsbBus::device_events_no_hubs()`.
//! 3. On `DeviceEvent::Connect`, uses `UsbBus::get_configuration()` with a
//!    `DescriptorVisitor` to find the first interrupt IN endpoint.
//! 4. Calls `UsbBus::configure()` which issues SET_CONFIGURATION and builds
//!    the endpoint bitmap.
//! 5. Calls `UsbBus::interrupt_endpoint_in()` to open an interrupt IN stream.
//! 6. Polls the stream in a loop, logging each raw HID report as hex.
//!
//! # Expected output (with a USB keyboard plugged in)
//!
//! ```text
//! === imxrt-usbh: HID Keyboard Example ===
//! USB2 PLL locked
//! VBUS power enabled
//! USB host controller initialised
//! USB_OTG2 ISR installed (NVIC priority 0xE0)
//! Entering device event loop...
//! DeviceEvent::Connect  addr=1  VID=045e PID=00db class=0 subclass=0
//! Found HID interface: iface=0 ep=1 mps=8 interval=10
//! Opening interrupt IN stream...
//! key: A
//! key: Shift+A
//! key: Ctrl+C
//! key: Enter
//! key: F1
//! key: A B        <- A and B simultaneously
//! ```
//!
//! Idle reports (no keys pressed) are suppressed.
//! Up to 3 simultaneous keycodes are shown per report (full boot protocol has 6).
//!
//! # Flash
//!
//! ```sh
//! .\build_example.ps1 -Example rtic_usb_hid_keyboard -HexFile hid_keyboard.hex
//! teensy_loader_cli --mcu=TEENSY41 -w -v hid_keyboard.hex
//! ```

#![no_std]
#![no_main]

#[rtic::app(device = board, peripherals = false, dispatchers = [BOARD_SWTASK0])]
mod app {
    use core::pin::pin;
    use cotton_usb_host::usb_bus::{DeviceEvent, UsbBus};
    use cotton_usb_host::wire::{
        ConfigurationDescriptor, DescriptorVisitor, EndpointDescriptor, InterfaceDescriptor,
    };
    use futures::StreamExt;
    use imxrt_hal as hal;
    use imxrt_ral as ral;
    use imxrt_usbh::host::{Imxrt1062HostController, UsbShared, UsbStatics};

    // -----------------------------------------------------------------------
    // Configuration
    // -----------------------------------------------------------------------

    const FRONTEND: board::logging::Frontend = board::logging::Frontend::Log;
    const BACKEND: board::logging::Backend = board::logging::Backend::Usbd;

    const USB2_BASE: *const () = 0x402E_0200usize as *const ();
    const USB2_NVIC_PRIORITY: u8 = 0xE0;

    // -----------------------------------------------------------------------
    // HID keycode decoder (USB HID Usage Tables 1.12, section 10)
    // -----------------------------------------------------------------------

    /// Map a USB HID boot-protocol keycode to a printable name.
    ///
    /// Returns `""` for keycode 0 (no key), `"?"` for unknown codes.
    fn keycode_name(code: u8) -> &'static str {
        match code {
            0x00 => "",
            0x04 => "A",    0x05 => "B",    0x06 => "C",    0x07 => "D",
            0x08 => "E",    0x09 => "F",    0x0A => "G",    0x0B => "H",
            0x0C => "I",    0x0D => "J",    0x0E => "K",    0x0F => "L",
            0x10 => "M",    0x11 => "N",    0x12 => "O",    0x13 => "P",
            0x14 => "Q",    0x15 => "R",    0x16 => "S",    0x17 => "T",
            0x18 => "U",    0x19 => "V",    0x1A => "W",    0x1B => "X",
            0x1C => "Y",    0x1D => "Z",
            0x1E => "1",    0x1F => "2",    0x20 => "3",    0x21 => "4",
            0x22 => "5",    0x23 => "6",    0x24 => "7",    0x25 => "8",
            0x26 => "9",    0x27 => "0",
            0x28 => "Enter",
            0x29 => "Esc",
            0x2A => "Backspace",
            0x2B => "Tab",
            0x2C => "Space",
            0x2D => "-",    0x2E => "=",    0x2F => "[",    0x30 => "]",
            0x31 => "\\",   0x33 => ";",    0x34 => "'",    0x35 => "`",
            0x36 => ",",    0x37 => ".",    0x38 => "/",
            0x39 => "CapsLock",
            0x3A => "F1",   0x3B => "F2",   0x3C => "F3",   0x3D => "F4",
            0x3E => "F5",   0x3F => "F6",   0x40 => "F7",   0x41 => "F8",
            0x42 => "F9",   0x43 => "F10",  0x44 => "F11",  0x45 => "F12",
            0x46 => "PrtSc", 0x47 => "ScrollLock", 0x48 => "Pause",
            0x49 => "Insert", 0x4A => "Home", 0x4B => "PageUp",
            0x4C => "Delete", 0x4D => "End",  0x4E => "PageDown",
            0x4F => "Right", 0x50 => "Left", 0x51 => "Down", 0x52 => "Up",
            0x53 => "NumLock",
            _ => "?",
        }
    }

    // -----------------------------------------------------------------------
    // Descriptor visitor: finds first interrupt IN endpoint
    // -----------------------------------------------------------------------

    /// Walks a configuration descriptor set and records the first interrupt
    /// IN endpoint found, along with the interface number and config value.
    struct HidFinder {
        config_value: u8,
        ep_num: Option<u8>,
        ep_mps: u16,
        ep_interval: u8,
        iface_num: u8,
    }

    impl Default for HidFinder {
        fn default() -> Self {
            Self {
                config_value: 1,
                ep_num: None,
                ep_mps: 8,
                ep_interval: 10,
                iface_num: 0,
            }
        }
    }

    impl DescriptorVisitor for HidFinder {
        fn on_configuration(&mut self, c: &ConfigurationDescriptor) {
            self.config_value = c.bConfigurationValue;
        }

        fn on_interface(&mut self, i: &InterfaceDescriptor) {
            if self.ep_num.is_none() {
                self.iface_num = i.bInterfaceNumber;
            }
        }

        fn on_endpoint(&mut self, e: &EndpointDescriptor) {
            // First interrupt IN endpoint: direction bit = 1, transfer type bits [1:0] = 3
            if self.ep_num.is_none()
                && (e.bEndpointAddress & 0x80) != 0
                && (e.bmAttributes & 0x03) == 0x03
            {
                self.ep_num = Some(e.bEndpointAddress & 0x0F);
                self.ep_mps = u16::from_le_bytes(e.wMaxPacketSize);
                self.ep_interval = e.bInterval;
            }
        }
    }

    // -----------------------------------------------------------------------
    // USB2 Peripherals bridge
    // -----------------------------------------------------------------------

    struct Usb2Peripherals {
        usb: ral::usb::USB2,
        usbphy: ral::usbphy::USBPHY2,
    }

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
    // VBUS power enable
    // -----------------------------------------------------------------------

    fn enable_vbus_power() {
        let iomuxc = unsafe { ral::iomuxc::IOMUXC::instance() };
        ral::write_reg!(ral::iomuxc, iomuxc, SW_MUX_CTL_PAD_GPIO_EMC_40, 5);
        ral::write_reg!(ral::iomuxc, iomuxc, SW_PAD_CTL_PAD_GPIO_EMC_40, 0x0008);

        // Route GPIO_EMC_40 (GPIO3_IO26) to fast GPIO8_IO26.
        // The Teensy Arduino startup sets GPR26-29 = 0xFFFFFFFF; our bare-metal
        // runtime doesn't, so GPR28[26] defaults to 0 (pin driven by GPIO3).
        // Without this, writes to GPIO8 registers update the register file but
        // don't actually drive the pin.
        let iomuxc_gpr = unsafe { ral::iomuxc_gpr::IOMUXC_GPR::instance() };
        ral::modify_reg!(ral::iomuxc_gpr, iomuxc_gpr, GPR28, |v| v | (1 << 26));

        let gpio8 = unsafe { ral::gpio::GPIO8::instance() };
        ral::modify_reg!(ral::gpio, gpio8, GDIR, |v| v | (1 << 26));
        ral::write_reg!(ral::gpio, gpio8, DR_SET, 1 << 26);
    }

    // -----------------------------------------------------------------------
    // Delay helper
    // -----------------------------------------------------------------------

    fn delay_ms(ms: usize) -> impl core::future::Future<Output = ()> {
        cortex_m::asm::delay((ms as u32) * 600_000);
        core::future::ready(())
    }

    // -----------------------------------------------------------------------
    // Static resources
    // -----------------------------------------------------------------------

    static SHARED: UsbShared = UsbShared::new();
    static mut STATICS: UsbStatics = UsbStatics::new();

    // -----------------------------------------------------------------------
    // RTIC resources
    // -----------------------------------------------------------------------

    #[local]
    struct Local {}

    #[shared]
    struct Shared {
        poller: board::logging::Poller,
    }

    // -----------------------------------------------------------------------
    // Init
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
        // Filter out TRACE-level messages to avoid overflowing the 1024-byte log buffer
        // during rapid USB transfer sequences.
        log::set_max_level(log::LevelFilter::Debug);

        (Shared { poller }, Local {})
    }

    // -----------------------------------------------------------------------
    // USB_OTG2 ISR
    // -----------------------------------------------------------------------

    unsafe extern "C" fn usb2_isr() {
        SHARED.on_usb_irq(USB2_BASE);
    }

    // -----------------------------------------------------------------------
    // Idle — USB2 host init
    // -----------------------------------------------------------------------

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        cortex_m::asm::delay(600_000 * 5_000);

        log::info!("=== imxrt-usbh: HID Keyboard Example ===");

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
            __INTERRUPTS[usb_otg2_irq].get().write_volatile(usb2_isr);

            cortex_m::asm::dsb();
            cortex_m::asm::isb();

            cortex_m::peripheral::NVIC::unmask(ral::interrupt::USB_OTG2);
        }
        log::info!(
            "USB_OTG2 ISR installed (NVIC priority 0x{:02X})",
            USB2_NVIC_PRIORITY
        );

        hid_keyboard::spawn(host).ok();

        loop {
            cortex_m::asm::wfi();
        }
    }

    // -----------------------------------------------------------------------
    // USB1 ISR (logging) — priority 2 so log flushing preempts USB task
    // -----------------------------------------------------------------------
    //
    // The USB host task runs at priority 1 via the BOARD_SWTASK0 dispatcher.
    // At priority 2, USB1 and DMA interrupts preempt the USB task to flush
    // logs promptly. We poll the logger directly in the ISR, avoiding the
    // need for a second RTIC dispatcher.

    #[task(binds = BOARD_USB1, shared = [poller], priority = 2)]
    fn usb1_interrupt(mut cx: usb1_interrupt::Context) {
        cx.shared.poller.lock(|poller| poller.poll());
    }

    #[task(binds = BOARD_DMA_A, shared = [poller], priority = 2)]
    fn dma_interrupt(mut cx: dma_interrupt::Context) {
        cx.shared.poller.lock(|poller| poller.poll());
    }

    // -----------------------------------------------------------------------
    // HID keyboard task
    // -----------------------------------------------------------------------

    /// Async task: enumerate device, find HID interrupt endpoint, poll for reports.
    ///
    /// Uses the cotton-usb-host high-level API:
    ///  - `UsbBus::get_configuration()` to parse descriptors via `HidFinder`
    ///  - `UsbBus::configure()` to issue SET_CONFIGURATION
    ///  - `UsbBus::interrupt_endpoint_in()` to open the interrupt IN stream
    #[task(priority = 1)]
    async fn hid_keyboard(_cx: hid_keyboard::Context, host: Imxrt1062HostController) {
        log::info!("Entering device event loop...");

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

                    // Walk configuration descriptors to find first interrupt IN endpoint.
                    let mut finder = HidFinder::default();
                    if let Err(_e) = bus.get_configuration(&device, &mut finder).await {
                        log::warn!("get_configuration failed");
                        continue;
                    }

                    let (ep, ep_mps, ep_interval) = match finder.ep_num {
                        Some(n) => (n, finder.ep_mps, finder.ep_interval),
                        None => {
                            log::warn!("No interrupt IN endpoint found");
                            continue;
                        }
                    };

                    log::info!(
                        "Found HID interface: iface={} ep={} mps={} interval={}",
                        finder.iface_num,
                        ep,
                        ep_mps,
                        ep_interval,
                    );

                    // Issue SET_CONFIGURATION and transition to Configured state.
                    let usb_device = match bus.configure(device, finder.config_value).await {
                        Ok(d) => d,
                        Err(_e) => {
                            log::warn!("configure failed");
                            continue;
                        }
                    };

                    log::info!("Opening interrupt IN stream...");

                    // Allocate interrupt IN pipe and poll for HID reports.
                    // alloc_interrupt_pipe() is awaited lazily on first poll_next().
                    let mut pipe =
                        pin!(bus.interrupt_endpoint_in(&usb_device, ep, ep_mps, ep_interval));

                    loop {
                        match pipe.next().await {
                            Some(pkt) => {
                                // HID boot protocol (8 bytes):
                                //   byte 0: modifier bitmap
                                //   byte 1: reserved
                                //   bytes 2-7: up to 6 simultaneous keycodes
                                if pkt.size < 8 {
                                    continue; // unexpected short report
                                }
                                let mods = pkt.data[0];
                                // Suppress idle reports (no keys, no modifiers).
                                if mods == 0 && pkt.data[2] == 0 {
                                    continue;
                                }
                                // Modifier byte bits: LCtrl=0 LShift=1 LAlt=2 LGUI=3
                                //                    RCtrl=4 RShift=5 RAlt=6 RGUI=7
                                log::info!(
                                    "key: {}{}{}{}{}{}{}{}{}",
                                    if mods & 0x02 != 0 || mods & 0x20 != 0 { "Shift+" } else { "" },
                                    if mods & 0x01 != 0 || mods & 0x10 != 0 { "Ctrl+" } else { "" },
                                    if mods & 0x04 != 0 || mods & 0x40 != 0 { "Alt+" } else { "" },
                                    if mods & 0x08 != 0 || mods & 0x80 != 0 { "GUI+" } else { "" },
                                    keycode_name(pkt.data[2]),
                                    if pkt.data[3] != 0 { " " } else { "" },
                                    keycode_name(pkt.data[3]),
                                    if pkt.data[4] != 0 { " " } else { "" },
                                    keycode_name(pkt.data[4]),
                                );
                            }
                            None => {
                                log::warn!("Interrupt pipe stream ended");
                                break;
                            }
                        }
                    }
                }
                Some(DeviceEvent::Disconnect(_)) => {
                    log::info!("DeviceEvent::Disconnect");
                }
                Some(DeviceEvent::EnumerationError(hub, port, _err)) => {
                    log::warn!(
                        "DeviceEvent::EnumerationError  hub={} port={}",
                        hub,
                        port
                    );
                }
                Some(DeviceEvent::HubConnect(_)) => {}
                Some(DeviceEvent::None) => {}
                None => {
                    log::warn!("Device event stream ended");
                    break;
                }
            }
        }
    }
}
