//! USB Mass Storage (MSC) sector-read example using RTIC v2 — Phase 2c validation.
//!
//! Enumerates a USB flash drive on the USB2 host port, finds its bulk IN and bulk
//! OUT endpoints, and reads sector 0 using a raw SCSI READ(10) command over the
//! USB Mass Storage Bulk-Only Transport (BBB/BOT) protocol.
//!
//! # What it does
//!
//! 1. Initialises USB2 host controller (same as other examples).
//! 2. Waits for a device connect event via `UsbBus::device_events_no_hubs()`.
//! 3. On `DeviceEvent::Connect`, walks configuration descriptors with `MscFinder`
//!    to locate an MSC interface (class=8, subclass=6, protocol=0x50) and its
//!    bulk IN / bulk OUT endpoints.
//! 4. Issues `UsbBus::configure()` to transition device to Configured state.
//! 5. Opens the bulk IN and bulk OUT endpoints via `UsbDevice::open_in_endpoint()`
//!    and `open_out_endpoint()`.
//! 6. Sends SCSI READ(10) via CBW/CSW:
//!    a. Send 31-byte Command Block Wrapper (CBW) → bulk OUT
//!    b. Receive 512-byte sector data ← bulk IN
//!    c. Receive 13-byte Command Status Wrapper (CSW) ← bulk IN
//! 7. Verifies CSW signature and status, then logs the first 16 bytes of sector 0.
//!
//! # Expected output (with a USB flash drive plugged in)
//!
//! ```text
//! === imxrt-usbh: USB Mass Storage Example ===
//! USB2 PLL locked
//! VBUS power enabled
//! USB host controller initialised
//! USB_OTG2 ISR installed (NVIC priority 0xE0)
//! Entering device event loop...
//! DeviceEvent::Connect  addr=1  VID=xxxx PID=xxxx class=0
//! Found MSC interface: class=8 sub=6 proto=0x50 bulk_in=1 bulk_out=2
//! Opening bulk endpoints...
//! Sending CBW READ(10) LBA=0...
//! Data received: 512 bytes
//! Sector 0: eb 58 90 4e 54 46 53 20 20 20 20 00 02 08 00 00
//! CSW: status=0 (success)
//! ```
//!
//! # Build and flash
//!
//! ```sh
//! cargo build --release --target thumbv7em-none-eabihf --example rtic_usb_mass_storage
//! rust-objcopy -O ihex target/thumbv7em-none-eabihf/release/examples/rtic_usb_mass_storage rtic_usb_mass_storage.hex
//! teensy_loader_cli --mcu=TEENSY41 -w -v rtic_usb_mass_storage.hex
//! ```

#![no_std]
#![no_main]

#[rtic::app(device = board, peripherals = false, dispatchers = [BOARD_SWTASK0])]
mod app {
    use core::pin::pin;
    use cotton_usb_host::usb_bus::{BulkIn, BulkOut, DeviceEvent, TransferType, UsbBus};
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
    // MSC (Mass Storage Class) descriptor visitor
    // -----------------------------------------------------------------------

    /// Walks a configuration descriptor set and records the first MSC interface
    /// (class=8, subclass=6, protocol=0x50 BOT) plus its bulk IN and OUT endpoints.
    struct MscFinder {
        config_value: u8,
        /// True if we found an MSC-compatible interface.
        found_msc_interface: bool,
        /// Bulk IN endpoint number (1–15), or None if not yet found.
        ep_in: Option<u8>,
        /// Bulk OUT endpoint number (1–15), or None if not yet found.
        ep_out: Option<u8>,
        /// Maximum packet size advertised by the bulk IN endpoint.
        ep_in_mps: u16,
        /// Maximum packet size advertised by the bulk OUT endpoint.
        ep_out_mps: u16,
    }

    impl Default for MscFinder {
        fn default() -> Self {
            Self {
                config_value: 1,
                found_msc_interface: false,
                ep_in: None,
                ep_out: None,
                ep_in_mps: 64,
                ep_out_mps: 64,
            }
        }
    }

    impl DescriptorVisitor for MscFinder {
        fn on_configuration(&mut self, c: &ConfigurationDescriptor) {
            self.config_value = c.bConfigurationValue;
        }

        fn on_interface(&mut self, i: &InterfaceDescriptor) {
            // USB Mass Storage Class: class=8, subclass=6 (SCSI), protocol=0x50 (BOT)
            if i.bInterfaceClass == 8
                && i.bInterfaceSubClass == 6
                && i.bInterfaceProtocol == 0x50
            {
                self.found_msc_interface = true;
            }
        }

        fn on_endpoint(&mut self, e: &EndpointDescriptor) {
            // Only capture endpoints belonging to the MSC interface.
            if !self.found_msc_interface {
                return;
            }
            // Bulk transfer type: bmAttributes[1:0] == 0b10
            if (e.bmAttributes & 0x03) != 0x02 {
                return;
            }
            let ep_num = e.bEndpointAddress & 0x0F;
            let mps = u16::from_le_bytes(e.wMaxPacketSize);
            if (e.bEndpointAddress & 0x80) != 0 {
                // IN endpoint
                if self.ep_in.is_none() {
                    self.ep_in = Some(ep_num);
                    self.ep_in_mps = mps;
                }
            } else {
                // OUT endpoint
                if self.ep_out.is_none() {
                    self.ep_out = Some(ep_num);
                    self.ep_out_mps = mps;
                }
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
        // Match USBHost_t36 ehci.cpp lines 208-213 exactly:
        //   IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_40 = 5;
        //   IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_40 = 0x0008;
        //   GPIO8_GDIR |= 1<<26;
        //   GPIO8_DR_SET = 1<<26;
        // Note: USBHost_t36 does NOT set any IOMUXC_GPR register.
        let iomuxc = unsafe { ral::iomuxc::IOMUXC::instance() };
        ral::write_reg!(ral::iomuxc, iomuxc, SW_MUX_CTL_PAD_GPIO_EMC_40, 5);
        ral::write_reg!(ral::iomuxc, iomuxc, SW_PAD_CTL_PAD_GPIO_EMC_40, 0x0008);

        let gpio8 = unsafe { ral::gpio::GPIO8::instance() };
        ral::modify_reg!(ral::gpio, gpio8, GDIR, |v| v | (1 << 26));
        ral::write_reg!(ral::gpio, gpio8, DR_SET, 1 << 26);
    }

    // -----------------------------------------------------------------------
    // Error formatting helper (UsbError has no Debug impl)
    // -----------------------------------------------------------------------

    fn usb_err(e: &cotton_usb_host::usb_bus::UsbError) -> &'static str {
        use cotton_usb_host::usb_bus::UsbError;
        match e {
            UsbError::Stall => "Stall",
            UsbError::Timeout => "Timeout",
            UsbError::Overflow => "Overflow",
            UsbError::BitStuffError => "BitStuffError",
            UsbError::CrcError => "CrcError",
            UsbError::DataSeqError => "DataSeqError",
            UsbError::BufferTooSmall => "BufferTooSmall",
            UsbError::AllPipesInUse => "AllPipesInUse",
            UsbError::ProtocolError => "ProtocolError",
            UsbError::TooManyDevices => "TooManyDevices",
            UsbError::NoSuchEndpoint => "NoSuchEndpoint",
            _ => "Unknown",
        }
    }

    // -----------------------------------------------------------------------
    // Delay helper
    // -----------------------------------------------------------------------

    fn delay_ms(ms: usize) -> impl core::future::Future<Output = ()> {
        cortex_m::asm::delay((ms as u32) * 600_000);
        core::future::ready(())
    }

    // -----------------------------------------------------------------------
    // CBW / CSW helpers
    // -----------------------------------------------------------------------

    /// Build a 31-byte Command Block Wrapper (CBW) for SCSI READ(10).
    ///
    /// Reads `num_blocks` 512-byte blocks starting at `lba` from `lun`.
    /// Direction is IN (data flows from device to host), so `bmCBWFlags = 0x80`.
    fn build_read10_cbw(tag: u32, lba: u32, num_blocks: u16, lun: u8) -> [u8; 31] {
        let mut cbw = [0u8; 31];
        // dCBWSignature = 0x43425355 ("USBC" little-endian)
        cbw[0] = 0x55;
        cbw[1] = 0x53;
        cbw[2] = 0x42;
        cbw[3] = 0x43;
        // dCBWTag
        cbw[4] = (tag & 0xFF) as u8;
        cbw[5] = ((tag >> 8) & 0xFF) as u8;
        cbw[6] = ((tag >> 16) & 0xFF) as u8;
        cbw[7] = ((tag >> 24) & 0xFF) as u8;
        // dCBWDataTransferLength = num_blocks * 512 (little-endian)
        let data_len = (num_blocks as u32) * 512;
        cbw[8]  = (data_len & 0xFF) as u8;
        cbw[9]  = ((data_len >> 8) & 0xFF) as u8;
        cbw[10] = ((data_len >> 16) & 0xFF) as u8;
        cbw[11] = ((data_len >> 24) & 0xFF) as u8;
        // bmCBWFlags = 0x80 (IN direction — data flows device → host)
        cbw[12] = 0x80;
        // bCBWLUN
        cbw[13] = lun & 0x0F;
        // bCBWCBLength = 10 (READ(10) is a 10-byte CDB)
        cbw[14] = 10;
        // CBWCB: READ(10) = 0x28
        cbw[15] = 0x28; // opcode
        cbw[16] = 0;    // service action / reserved
        // LBA (big-endian, 4 bytes)
        cbw[17] = ((lba >> 24) & 0xFF) as u8;
        cbw[18] = ((lba >> 16) & 0xFF) as u8;
        cbw[19] = ((lba >> 8) & 0xFF) as u8;
        cbw[20] = (lba & 0xFF) as u8;
        cbw[21] = 0; // group number
        // Transfer length (big-endian, 2 bytes)
        cbw[22] = ((num_blocks >> 8) & 0xFF) as u8;
        cbw[23] = (num_blocks & 0xFF) as u8;
        cbw[24] = 0; // control
        // bytes 25-30: remaining CBWCB bytes (zeroed)
        cbw
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
    struct Local {
        led: board::Led,
        pit: hal::pit::Pit<2>,
    }

    #[shared]
    struct Shared {
        poller: board::logging::Poller,
    }

    // -----------------------------------------------------------------------
    // Init
    // -----------------------------------------------------------------------

    /// LED blink interval: 500 ms.
    const HEARTBEAT_TICKS: u32 = board::PIT_FREQUENCY / 1_000 * 500;

    #[init]
    fn init(_cx: init::Context) -> (Shared, Local) {
        let (
            board::Common {
                pit: (_, _, mut pit, _),
                usb1,
                usbnc1,
                usbphy1,
                mut dma,
                ..
            },
            board::Specifics { led, console, .. },
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

        // Start a 500 ms heartbeat timer to prove the system is alive.
        pit.set_load_timer_value(HEARTBEAT_TICKS);
        pit.set_interrupt_enable(true);
        pit.enable();

        (Shared { poller }, Local { led, pit })
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

        log::info!("=== imxrt-usbh: USB Mass Storage Example ===");

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

        usb_task::spawn(host).ok();

        loop {
            cortex_m::asm::wfi();
        }
    }

    // -----------------------------------------------------------------------
    // USB1 ISR (logging) — priority 2 so log flushing preempts USB task
    // -----------------------------------------------------------------------
    //
    // The USB host task runs at priority 1 via the BOARD_SWTASK0 dispatcher.
    // If these logging ISRs were also at priority 1, they couldn't preempt
    // the dispatcher while it's running usb_task (e.g. during busy-wait
    // delays or rapid transfer sequences), starving the log output.
    //
    // At priority 2, USB1 and DMA interrupts preempt the USB task to flush
    // logs promptly. We poll the logger directly in the ISR (no software
    // task needed), avoiding the need for a second RTIC dispatcher.

    #[task(binds = BOARD_USB1, shared = [poller], priority = 2)]
    fn usb1_interrupt(mut cx: usb1_interrupt::Context) {
        cx.shared.poller.lock(|poller| poller.poll());
    }

    #[task(binds = BOARD_DMA_A, shared = [poller], priority = 2)]
    fn dma_interrupt(mut cx: dma_interrupt::Context) {
        cx.shared.poller.lock(|poller| poller.poll());
    }

    // -----------------------------------------------------------------------
    // Heartbeat — LED blink at high priority to prove system is alive
    // -----------------------------------------------------------------------

    #[task(binds = BOARD_PIT, local = [led, pit], shared = [poller], priority = 3)]
    fn pit_interrupt(mut cx: pit_interrupt::Context) {
        let pit_interrupt::LocalResources { led, pit, .. } = cx.local;
        if pit.is_elapsed() {
            while pit.is_elapsed() {
                pit.clear_elapsed();
            }
            led.toggle();
            // Force-drain the log buffer from a known-good high-priority context.
            cx.shared.poller.lock(|poller| poller.poll());
        }
    }

    // -----------------------------------------------------------------------
    // USB mass storage task
    // -----------------------------------------------------------------------

    /// Async task: enumerate MSC device, find bulk endpoints, read sector 0.
    #[task(priority = 1)]
    async fn usb_task(_cx: usb_task::Context, host: Imxrt1062HostController) {
        log::info!("Entering device event loop...");

        let bus = UsbBus::new(host);
        let mut events = pin!(bus.device_events_no_hubs(delay_ms));

        loop {
            match events.next().await {
                Some(DeviceEvent::Connect(device, info)) => {
                    log::info!(
                        "DeviceEvent::Connect  addr={}  VID={:04x} PID={:04x} class={}",
                        device.address(),
                        info.vid,
                        info.pid,
                        info.class,
                    );

                    // Walk descriptors to find MSC interface and bulk endpoints.
                    let mut finder = MscFinder::default();
                    if let Err(_e) = bus.get_configuration(&device, &mut finder).await {
                        log::warn!("get_configuration failed");
                        continue;
                    }

                    let (ep_in_num, ep_out_num) = match (finder.ep_in, finder.ep_out) {
                        (Some(i), Some(o)) => (i, o),
                        _ => {
                            if !finder.found_msc_interface {
                                log::info!("Not an MSC device, skipping");
                            } else {
                                log::warn!("MSC interface found but missing bulk endpoint(s)");
                            }
                            continue;
                        }
                    };

                    log::info!(
                        "Found MSC interface: bulk_in={} (mps={}) bulk_out={} (mps={})",
                        ep_in_num, finder.ep_in_mps,
                        ep_out_num, finder.ep_out_mps,
                    );

                    // Configure the device (SET_CONFIGURATION).
                    let mut usb_device = match bus.configure(device, finder.config_value).await {
                        Ok(d) => d,
                        Err(_e) => {
                            log::warn!("configure failed");
                            continue;
                        }
                    };

                    log::info!("Opening bulk endpoints...");

                    // Open bulk IN and OUT endpoints (consumes them from the device bitmap).
                    let ep_in: BulkIn = match usb_device.open_in_endpoint(ep_in_num) {
                        Ok(ep) => ep,
                        Err(_) => {
                            log::warn!("Failed to open bulk IN endpoint {}", ep_in_num);
                            continue;
                        }
                    };
                    let ep_out: BulkOut = match usb_device.open_out_endpoint(ep_out_num) {
                        Ok(ep) => ep,
                        Err(_) => {
                            log::warn!("Failed to open bulk OUT endpoint {}", ep_out_num);
                            continue;
                        }
                    };

                    // --- SCSI READ(10) via CBW/CSW ---

                    // Build CBW: READ(10) LBA=0, 1 block (512 bytes), tag=1.
                    let cbw = build_read10_cbw(1, 0, 1, 0);

                    log::info!("Sending CBW READ(10) LBA=0...");

                    // Step 1: Send CBW via bulk OUT.
                    match bus.bulk_out_transfer(&ep_out, &cbw, TransferType::FixedSize).await {
                        Ok(n) => log::info!("CBW sent: {} bytes", n),
                        Err(e) => {
                            log::warn!("CBW OUT failed: {}", usb_err(&e));
                            continue;
                        }
                    }

                    // Step 2: Receive 512 bytes of sector data via bulk IN.
                    // Use a static buffer so it's not in DTCM (DMA-accessible).
                    static mut SECTOR_BUF: [u8; 512] = [0u8; 512];
                    let sector_buf = unsafe { &mut *core::ptr::addr_of_mut!(SECTOR_BUF) };

                    match bus.bulk_in_transfer(&ep_in, sector_buf, TransferType::FixedSize).await {
                        Ok(n) => {
                            log::info!("Data received: {} bytes", n);
                            log::info!(
                                "Sector 0: {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} \
                                 {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x}",
                                sector_buf[0],  sector_buf[1],  sector_buf[2],  sector_buf[3],
                                sector_buf[4],  sector_buf[5],  sector_buf[6],  sector_buf[7],
                                sector_buf[8],  sector_buf[9],  sector_buf[10], sector_buf[11],
                                sector_buf[12], sector_buf[13], sector_buf[14], sector_buf[15],
                            );
                        }
                        Err(e) => {
                            log::warn!("Data IN failed: {}", usb_err(&e));
                            continue;
                        }
                    }

                    // Step 3: Receive 13-byte Command Status Wrapper (CSW) via bulk IN.
                    let mut csw = [0u8; 13];
                    match bus.bulk_in_transfer(&ep_in, &mut csw, TransferType::FixedSize).await {
                        Ok(n) => {
                            if n < 13 {
                                log::warn!("CSW short: {} bytes", n);
                                continue;
                            }
                            // Verify CSW signature: "USBS" = 0x53425355 (little-endian)
                            let sig = u32::from_le_bytes([csw[0], csw[1], csw[2], csw[3]]);
                            if sig != 0x53425355 {
                                log::warn!("CSW bad signature: 0x{:08x}", sig);
                                continue;
                            }
                            let status = csw[12];
                            if status == 0 {
                                log::info!("CSW: status=0 (success)");
                            } else {
                                log::warn!("CSW: status={} (error)", status);
                            }
                        }
                        Err(e) => {
                            log::warn!("CSW IN failed: {}", usb_err(&e));
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
