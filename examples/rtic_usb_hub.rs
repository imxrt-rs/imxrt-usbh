//! USB hub enumeration example using RTIC v2.
//!
//! Demonstrates hub-aware device enumeration on the USB2 host port.
//! Connects to a USB hub, discovers devices behind it, and for any HID
//! device found, opens an interrupt IN pipe and logs raw reports.
//!
//! **Requires the `hub-support` feature** — this forces Full Speed (12 Mbps)
//! connections via the PFSC bit, which is necessary because `cotton-usb-host`
//! does not yet implement EHCI split transactions for LS/FS devices behind
//! HS hubs.
//!
//! # What it does
//!
//! 1. Initialises USB2 host controller.
//! 2. Waits for device events via `UsbBus::device_events()` with `HubState`
//!    for hub-aware enumeration.
//! 3. Logs `HubConnect` when a hub is detected.
//! 4. On `DeviceEvent::Connect` for a device behind the hub, walks
//!    configuration descriptors to find an interrupt IN endpoint.
//! 5. If found, opens an interrupt IN stream and logs each HID report.
//!
//! # Expected output (hub + keyboard)
//!
//! ```text
//! === imxrt-usbh: USB Hub Example ===
//! USB2 PLL locked
//! VBUS power enabled
//! USB host controller initialised
//! USB_OTG2 ISR installed (NVIC priority 0xE0)
//! Entering device event loop...
//! DeviceEvent::HubConnect  addr=1
//! DeviceEvent::Connect  addr=2  VID=045e PID=00db class=0 subclass=0
//! Found HID interface: iface=0 ep=1 mps=8 interval=10
//! Opening interrupt IN stream...
//! HID report: 00 00 04 00 00 00 00 00
//! HID report: 00 00 00 00 00 00 00 00
//! ```
//!
//! # Build and flash
//!
//! ```sh
//! cargo build --release --target thumbv7em-none-eabihf --features hub-support --example rtic_usb_hub
//! rust-objcopy -O ihex target/thumbv7em-none-eabihf/release/examples/rtic_usb_hub rtic_usb_hub.hex
//! teensy_loader_cli --mcu=TEENSY41 -w -v rtic_usb_hub.hex
//! ```

#![no_std]
#![no_main]

#[rtic::app(device = board, peripherals = false, dispatchers = [BOARD_SWTASK0])]
mod app {
    use core::pin::pin;
    use cotton_usb_host::usb_bus::{DeviceEvent, HubState, UsbBus};
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
        log::set_max_level(log::LevelFilter::Debug);

        hub_task::spawn().ok();

        (Shared { poller }, Local {})
    }

    // -----------------------------------------------------------------------
    // USB_OTG2 ISR
    // -----------------------------------------------------------------------

    unsafe extern "C" fn usb2_isr() {
        SHARED.on_usb_irq(USB2_BASE);
    }

    // -----------------------------------------------------------------------
    // USB1 ISR (logging) — priority 2 so log flushing preempts USB task
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
    // Hub enumeration task
    // -----------------------------------------------------------------------

    /// Async task: hub-aware device enumeration and HID report logging.
    ///
    /// Uses the cotton-usb-host high-level API:
    ///  - `UsbBus::device_events()` with `HubState` for hub-aware enumeration
    ///  - `UsbBus::get_configuration()` to parse descriptors via `HidFinder`
    ///  - `UsbBus::configure()` to issue SET_CONFIGURATION
    ///  - `UsbBus::interrupt_endpoint_in()` to open the interrupt IN stream
    #[task(priority = 1)]
    async fn hub_task(_cx: hub_task::Context) {
        cortex_m::asm::delay(600_000 * 5_000);

        log::info!("=== imxrt-usbh: USB Hub Example ===");

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

        log::info!("Entering device event loop...");

        let hub_state: HubState<Imxrt1062HostController> = HubState::default();
        let bus = UsbBus::new(host);
        let mut events = pin!(bus.device_events(&hub_state, delay_ms));

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
                            log::info!("No interrupt IN endpoint found (not HID)");
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

                    let mut pipe =
                        pin!(bus.interrupt_endpoint_in(&usb_device, ep, ep_mps, ep_interval));

                    loop {
                        match pipe.next().await {
                            Some(pkt) => {
                                // Log the raw HID report bytes.
                                let n = (pkt.size as usize).min(8);
                                if n >= 8 {
                                    log::info!(
                                        "HID report: {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x}",
                                        pkt.data[0], pkt.data[1], pkt.data[2], pkt.data[3],
                                        pkt.data[4], pkt.data[5], pkt.data[6], pkt.data[7],
                                    );
                                } else {
                                    log::info!("HID report: {} bytes", pkt.size);
                                }
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
                Some(DeviceEvent::HubConnect(hub)) => {
                    log::info!("DeviceEvent::HubConnect  addr={}", hub.address());
                }
                Some(DeviceEvent::None) => {}
                None => {
                    log::warn!("Device event stream ended");
                    break;
                }
            }
        }
    }
}
