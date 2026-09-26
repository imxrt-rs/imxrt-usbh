//! USB MIDI keyboard input example using RTIC v2.
//!
//! Enumerates a USB MIDI device (e.g. a MIDI keyboard) on the USB2 host port,
//! finds its bulk IN endpoint, and reads USB-MIDI event packets in a loop.
//! Note On/Off, Control Change, and Pitch Bend messages are decoded and logged.
//! The on-board LED lights up on Note On and turns off on Note Off.
//!
//! USB MIDI devices use **bulk endpoints** (not interrupt or isochronous).
//! Each bulk transfer contains one or more 4-byte USB-MIDI Event Packets:
//!   Byte 0: [Cable Number (4 bits)][Code Index Number (4 bits)]
//!   Bytes 1-3: MIDI data bytes
//!
//! # Expected output (with a USB MIDI keyboard plugged in)
//!
//! ```text
//! === imxrt-usbh: USB MIDI Keyboard Example ===
//! USB2 PLL locked
//! VBUS power enabled
//! USB host controller initialised
//! USB_OTG2 ISR installed (NVIC priority 0xE0)
//! Entering device event loop...
//! DeviceEvent::Connect  addr=1  VID=xxxx PID=xxxx class=0
//! MIDI Streaming interface found: bulk_in=1 bulk_out=2
//! MIDI device ready, reading packets...
//! Note ON  ch=1 note=60 vel=100
//! Note OFF ch=1 note=60 vel=0
//! CC       ch=1 cc=1 val=64
//! PitchBend ch=1 val=8192
//! ```
//!
//! # Build and flash
//!
//! ```sh
//! cargo build --release --target thumbv7em-none-eabihf --example rtic_usb_midi_keyboard --features=imxrt-ral/imxrt1062
//! rust-objcopy -O ihex target/thumbv7em-none-eabihf/release/examples/rtic_usb_midi_keyboard rtic_usb_midi_keyboard.hex
//! teensy_loader_cli --mcu=TEENSY41 -w -v rtic_usb_midi_keyboard.hex
//! ```

#![no_std]
#![no_main]

#[rtic::app(device = board, peripherals = false, dispatchers = [BOARD_SWTASK0])]
mod app {
    use core::pin::pin;
    use cotton_usb_host::device::identify::IdentifyFromDescriptors;
    use cotton_usb_host::usb_bus::{DeviceEvent, UsbBus};
    use cotton_usb_host_midi::{IdentifyMidi, Midi, UsbMidiEventPacket};
    use futures::StreamExt;
    use imxrt_ral as ral;
    use imxrt_usbh::host::{ImxrtHostController, UsbShared, UsbStatics};

    // -----------------------------------------------------------------------
    // Configuration
    // -----------------------------------------------------------------------

    const FRONTEND: board::logging::Frontend = board::logging::Frontend::Log;
    const BACKEND: board::logging::Backend = board::logging::Backend::Usbd;

    const USB2_BASE: *const () = 0x402E_0200usize as *const ();
    const USB2_NVIC_PRIORITY: u8 = 0xE0;

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
    // Error formatting helper
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
    // MIDI packet decoder for logging
    // -----------------------------------------------------------------------

    /// Decode and log a USB-MIDI event packet.
    /// Returns true if a Note On was received (for LED control).
    fn log_midi_packet(pkt: &UsbMidiEventPacket) -> bool {
        let cin = pkt.code_index_number();
        let b = pkt.midi_bytes();
        match cin {
            0x09 => {
                // Note On
                let ch = (b[0] & 0x0F) + 1;
                let note = b[1];
                let vel = b[2];
                if vel > 0 {
                    log::info!("Note ON  ch={} note={} vel={}", ch, note, vel);
                    return true;
                } else {
                    // Note On with velocity 0 is equivalent to Note Off
                    log::info!("Note OFF ch={} note={} vel=0", ch, note);
                }
            }
            0x08 => {
                // Note Off
                let ch = (b[0] & 0x0F) + 1;
                let note = b[1];
                let vel = b[2];
                log::info!("Note OFF ch={} note={} vel={}", ch, note, vel);
            }
            0x0B => {
                // Control Change
                let ch = (b[0] & 0x0F) + 1;
                let cc = b[1];
                let val = b[2];
                log::info!("CC       ch={} cc={} val={}", ch, cc, val);
            }
            0x0E => {
                // Pitch Bend
                let ch = (b[0] & 0x0F) + 1;
                let val = (b[1] as u16) | ((b[2] as u16) << 7);
                log::info!("PitchBend ch={} val={}", ch, val);
            }
            0x0C => {
                // Program Change
                let ch = (b[0] & 0x0F) + 1;
                let prog = b[1];
                log::info!("PgmChg   ch={} prog={}", ch, prog);
            }
            0x0D => {
                // Channel Pressure (Aftertouch)
                let ch = (b[0] & 0x0F) + 1;
                let pressure = b[1];
                log::info!("ChanPres ch={} pressure={}", ch, pressure);
            }
            0x0A => {
                // Poly Key Pressure
                let ch = (b[0] & 0x0F) + 1;
                let note = b[1];
                let pressure = b[2];
                log::info!("PolyPres ch={} note={} pressure={}", ch, note, pressure);
            }
            _ => {
                // Hex dump for other/unknown CIN values
                let raw = pkt.as_bytes();
                log::info!(
                    "MIDI [{:02x} {:02x} {:02x} {:02x}]",
                    raw[0],
                    raw[1],
                    raw[2],
                    raw[3]
                );
            }
        }
        false
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
            board::Specifics { led, console, .. },
        ) = board::new();

        let usbd = imxrt_usbd::Instances {
            usb: usb1,
            usbnc: usbnc1,
            usbphy: usbphy1,
        };
        let dma_a = dma[board::BOARD_DMA_A_INDEX].take().unwrap();
        let poller = board::logging::init(FRONTEND, BACKEND, console, dma_a, usbd);
        log::set_max_level(log::LevelFilter::Debug);

        midi_task::spawn(led).ok();

        (Shared { poller }, Local {})
    }

    // -----------------------------------------------------------------------
    // USB_OTG2 ISR
    // -----------------------------------------------------------------------

    unsafe extern "C" fn usb2_isr() {
        SHARED.on_usb_irq(USB2_BASE);
    }

    // -----------------------------------------------------------------------
    // Logging ISRs — priority 2 so log flushing preempts USB task
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
    // USB MIDI task
    // -----------------------------------------------------------------------

    #[task(priority = 1)]
    async fn midi_task(_cx: midi_task::Context, led: board::Led) {
        cortex_m::asm::delay(600_000 * 5_000);

        log::info!("=== imxrt-usbh: USB MIDI Keyboard Example ===");

        enable_usb2_pll();
        log::info!("USB2 PLL locked");

        enable_vbus_power();
        log::info!("VBUS power enabled");

        let usb2 = unsafe { ral::usb::USB2::instance() };
        let usbphy2 = unsafe { ral::usbphy::USBPHY2::instance() };

        let statics: &'static UsbStatics = unsafe { &*core::ptr::addr_of!(STATICS) };
        let mut host = ImxrtHostController::new(usb2, usbphy2, &SHARED, statics);
        unsafe { host.init() };
        log::info!("USB host controller initialised");

        unsafe {
            let irq_num = ral::interrupt::USB_OTG2 as u32;
            core::ptr::write_volatile((0xE000_E400 + irq_num) as *mut u8, USB2_NVIC_PRIORITY);

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

        let bus = UsbBus::new(host);
        let mut events = pin!(bus.device_events_no_hubs(delay_ms));

        loop {
            match events.next().await {
                Some(DeviceEvent::Connect(device, info)) => {
                    if info.class == 9 {
                        log::warn!("Hub detected — not supported in this example");
                        continue;
                    }

                    log::info!(
                        "DeviceEvent::Connect  addr={}  VID={:04x} PID={:04x} class={}",
                        device.address(),
                        info.vid,
                        info.pid,
                        info.class,
                    );

                    // Walk configuration descriptors to find MIDI interface.
                    let mut identifier = IdentifyMidi::default();
                    if let Err(_e) = bus.get_configuration(&device, &mut identifier).await {
                        log::warn!("get_configuration failed");
                        continue;
                    }

                    let config_value = match identifier.identify() {
                        Some(v) => v,
                        None => {
                            log::info!("Not a MIDI device, skipping");
                            continue;
                        }
                    };

                    let in_ep = match identifier.in_endpoint() {
                        Some(ep) => ep,
                        None => {
                            log::warn!("MIDI interface found but no bulk IN endpoint");
                            continue;
                        }
                    };
                    let out_ep = identifier.out_endpoint();

                    log::info!(
                        "MIDI Streaming interface found: bulk_in={} bulk_out={}",
                        in_ep,
                        out_ep.map_or(-1i8, |e| e as i8),
                    );

                    // Configure the device (SET_CONFIGURATION).
                    let usb_device = match bus.configure(device, config_value).await {
                        Ok(d) => d,
                        Err(_e) => {
                            log::warn!("configure failed");
                            continue;
                        }
                    };

                    // Create the MIDI driver.
                    let midi = match Midi::new(&bus, usb_device, in_ep, out_ep) {
                        Ok(m) => m,
                        Err(e) => {
                            log::warn!("Midi::new failed: {}", usb_err(&e));
                            continue;
                        }
                    };

                    log::info!("MIDI device ready, reading packets...");

                    // Bulk IN receive buffer — must be in static memory for DMA.
                    // 64 bytes = max full-speed bulk packet = up to 16 MIDI events.
                    static mut RECV_BUF: [u8; 64] = [0u8; 64];
                    let recv_buf = unsafe { &mut *core::ptr::addr_of_mut!(RECV_BUF) };

                    let mut packet_buf = [UsbMidiEventPacket::from_bytes([0; 4]); 16];

                    loop {
                        match midi.read_packets(recv_buf, &mut packet_buf).await {
                            Ok(count) => {
                                for i in 0..count {
                                    let note_on = log_midi_packet(&packet_buf[i]);
                                    if note_on {
                                        led.set();
                                    } else if packet_buf[i].code_index_number() == 0x08
                                        || (packet_buf[i].code_index_number() == 0x09
                                            && packet_buf[i].midi_bytes()[2] == 0)
                                    {
                                        led.clear();
                                    }
                                }
                            }
                            Err(e) => {
                                log::warn!("MIDI read error: {}", usb_err(&e));
                                break;
                            }
                        }
                    }
                }
                Some(DeviceEvent::Disconnect(_)) => {
                    log::info!("DeviceEvent::Disconnect");
                }
                Some(DeviceEvent::EnumerationError(hub, port, _err)) => {
                    log::warn!("DeviceEvent::EnumerationError  hub={} port={}", hub, port);
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
