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
//! # or, to reach a device through a hub (forces Full Speed):
//! cargo build --release --target thumbv7em-none-eabihf --example rtic_usb_midi_keyboard --features=imxrt-ral/imxrt1062,hub-support
//! rust-objcopy -O ihex target/thumbv7em-none-eabihf/release/examples/rtic_usb_midi_keyboard rtic_usb_midi_keyboard.hex
//! teensy_loader_cli --mcu=TEENSY41 -w -v rtic_usb_midi_keyboard.hex
//! ```

#![no_std]
#![no_main]

#[rtic::app(device = board, peripherals = false, dispatchers = [BOARD_SWTASK0])]
mod app {
    use core::pin::pin;
    use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
    use cotton_usb_host::device::identify::IdentifyFromDescriptors;
    #[cfg(feature = "hub-support")]
    use cotton_usb_host::usb_bus::HubState;
    use cotton_usb_host::usb_bus::{DeviceEvent, UsbBus};
    use cotton_usb_host_midi::{IdentifyMidi, Midi, UsbMidiEventPacket};
    use futures::StreamExt;
    use imxrt_hal as hal;
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
    // Enumeration retry
    // -----------------------------------------------------------------------
    //
    // Some devices do not answer the first GET_DESCRIPTOR after a port reset.
    // The one this was written for (a Donner StarryCtrl, Jieli chipset) ACKs the
    // SETUP packet and then fails the IN data phase three times, after which the
    // controller disables the port. It does the same to other host stacks, where
    // the cure is to unplug it and plug it in again until it takes.
    //
    // On an enumeration error the task therefore resets the port and
    // enumerates again. The timing of each attempt is cotton-usb-host's own.
    // Bench, StarryCtrl on a Teensy 4.1: the first attempt fails and the second
    // works, every time. Waiting longer after the reset, or before it, does
    // not help; it is the second reset that does.

    /// How many times to enumerate a device before asking for a replug.
    const ENUM_ATTEMPTS: usize = 5;

    /// Pause before each retry, in ms.
    const ENUM_RETRY_PAUSE_MS: usize = 200;

    /// The delay handed to cotton-usb-host for enumeration.
    ///
    /// It waits exactly as long as it is asked to. The only addition is a log
    /// of the root port's state after the two waits cotton makes around a port
    /// reset, 50 ms holding it and 10 ms of recovery, which is the only place
    /// an application can see the port between the reset and the first
    /// request. The two are recognised by their length; if cotton changes
    /// them, the log lines stop and nothing else does.
    fn enum_delay_ms(ms: usize) -> impl core::future::Future<Output = ()> {
        let what = match ms {
            50 => "end of reset hold",
            10 => "end of recovery",
            _ => "",
        };
        // `delay_ms` does its waiting when it is called, not when awaited.
        let done = delay_ms(ms);
        if !what.is_empty() {
            log_port(what);
        }
        done
    }

    /// Log the root port's status register with the fields that matter during
    /// a reset picked out: connected, connect-change latched, enabled, enable
    /// change latched, reset in progress, high-speed, and the negotiated speed
    /// (0 full, 1 low, 2 high).
    fn log_port(what: &str) {
        let usb = unsafe { ral::usb::USB2::instance() };
        let portsc = ral::read_reg!(ral::usb, usb, PORTSC1);
        log::info!(
            "port at {}: PORTSC1=0x{:08X} CCS={} CSC={} PE={} PEC={} PR={} HSP={} PSPD={}",
            what,
            portsc,
            portsc & 1,
            (portsc >> 1) & 1,
            (portsc >> 2) & 1,
            (portsc >> 3) & 1,
            (portsc >> 8) & 1,
            (portsc >> 9) & 1,
            (portsc >> 26) & 3,
        );
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
        pit: hal::pit::Pit,
    }

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
                mut pit,
                ..
            },
            board::Specifics { led, console, .. },
        ) = board::new();

        pit.set_load_timer_value(HEARTBEAT_CHANNEL, board::PIT_FREQUENCY * HEARTBEAT_S);
        pit.set_interrupt_enable(HEARTBEAT_CHANNEL, true);
        pit.enable(HEARTBEAT_CHANNEL);

        let usbd = imxrt_usbd::Instances {
            usb: usb1,
            usbnc: usbnc1,
            usbphy: usbphy1,
        };
        let dma_a = dma[board::BOARD_DMA_A_INDEX].take().unwrap();
        let poller = board::logging::init(FRONTEND, BACKEND, console, dma_a, usbd);
        log::set_max_level(log::LevelFilter::Debug);

        midi_task::spawn(led).ok();

        (Shared { poller }, Local { pit })
    }

    // -----------------------------------------------------------------------
    // Heartbeat
    // -----------------------------------------------------------------------
    //
    // A line every few seconds, whether or not anything is happening, so that
    // silence on the console can be told apart from a port with nothing on it.
    // It runs from a timer interrupt above the MIDI task's priority, so it
    // also prints if that task is stuck.

    /// Seconds between heartbeat lines.
    const HEARTBEAT_S: u32 = 5;
    const HEARTBEAT_CHANNEL: hal::pit::Channel = hal::pit::Channel::Chan2;

    /// The host controller is initialised and its registers may be read.
    static HOST_READY: AtomicBool = AtomicBool::new(false);
    /// A MIDI device is configured and being read.
    static MIDI_READY: AtomicBool = AtomicBool::new(false);
    /// USB-MIDI event packets received from the current device.
    static MIDI_MESSAGES: AtomicU32 = AtomicU32::new(0);

    #[task(binds = BOARD_PIT, local = [pit, seconds: u32 = 0], priority = 2)]
    fn heartbeat(cx: heartbeat::Context) {
        let pit = cx.local.pit;
        while pit.is_elapsed(HEARTBEAT_CHANNEL) {
            pit.clear_elapsed(HEARTBEAT_CHANNEL);
        }
        *cx.local.seconds += HEARTBEAT_S;
        let seconds = *cx.local.seconds;

        if !HOST_READY.load(Ordering::Relaxed) {
            log::info!("[{:>5}s] starting up", seconds);
            return;
        }

        let usb = unsafe { ral::usb::USB2::instance() };
        let portsc = ral::read_reg!(ral::usb, usb, PORTSC1);
        // PCE: is the port-change interrupt armed? The ISR masks it and the
        // device-detect stream re-arms it, so 0 here for long means nobody is
        // listening for a plug-in.
        let pce = ral::read_reg!(ral::usb, usb, USBINTR, PCE);

        if MIDI_READY.load(Ordering::Relaxed) {
            log::info!(
                "[{:>5}s] USB-MIDI connected, {} messages  (port: CCS={} CSC={} PE={} PEC={} PSPD={} PCE={})",
                seconds,
                MIDI_MESSAGES.load(Ordering::Relaxed),
                portsc & 1,
                (portsc >> 1) & 1,
                (portsc >> 2) & 1,
                (portsc >> 3) & 1,
                (portsc >> 26) & 3,
                pce,
            );
        } else {
            log::info!(
                "[{:>5}s] no USB-MIDI device on the host port  (port: CCS={} CSC={} PE={} PEC={} PSPD={} PCE={})",
                seconds,
                portsc & 1,
                (portsc >> 1) & 1,
                (portsc >> 2) & 1,
                (portsc >> 3) & 1,
                (portsc >> 26) & 3,
                pce,
            );
        }
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
        HOST_READY.store(true, Ordering::Relaxed);
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
        let mut attempt: usize = 0;

        'port: loop {
            // A fresh stream starts out believing the port is empty, so a device
            // that is still plugged in is reported as a new connection and gets
            // a port reset and a full enumeration. That is the retry.
            //
            // With `hub-support` the stream is the hub-aware one, and a retry
            // resets the root port, which is the hub: everything behind it is
            // enumerated again from a fresh `HubState`.
            #[cfg(feature = "hub-support")]
            let hub_state: HubState<ImxrtHostController> = HubState::default();
            #[cfg(feature = "hub-support")]
            let mut events = pin!(bus.device_events(&hub_state, enum_delay_ms));
            #[cfg(not(feature = "hub-support"))]
            let mut events = pin!(bus.device_events_no_hubs(enum_delay_ms));

            loop {
                match events.next().await {
                    Some(DeviceEvent::Connect(device, info)) => {
                        if attempt > 0 {
                            log::info!("Enumerated on attempt {}", attempt + 1);
                        }
                        attempt = 0;

                        if info.class == 9 {
                            log::warn!("Hub detected: rebuild with --features=hub-support to look behind it");
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
                        MIDI_MESSAGES.store(0, Ordering::Relaxed);
                        MIDI_READY.store(true, Ordering::Relaxed);

                        // Bulk IN receive buffer — must be in static memory for DMA.
                        // 64 bytes = max full-speed bulk packet = up to 16 MIDI events.
                        static mut RECV_BUF: [u8; 64] = [0u8; 64];
                        let recv_buf = unsafe { &mut *core::ptr::addr_of_mut!(RECV_BUF) };

                        let mut packet_buf = [UsbMidiEventPacket::from_bytes([0; 4]); 16];

                        loop {
                            match midi.read_packets(recv_buf, &mut packet_buf).await {
                                Ok(count) => {
                                    MIDI_MESSAGES.fetch_add(count as u32, Ordering::Relaxed);
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
                                    MIDI_READY.store(false, Ordering::Relaxed);
                                    break;
                                }
                            }
                        }
                    }
                    Some(DeviceEvent::Disconnect(_)) => {
                        log::info!("DeviceEvent::Disconnect");
                        MIDI_READY.store(false, Ordering::Relaxed);
                        attempt = 0;
                    }
                    Some(DeviceEvent::EnumerationError(hub, port, err)) => {
                        log::warn!(
                            "DeviceEvent::EnumerationError  hub={} port={} err={}  (attempt {} of {})",
                            hub,
                            port,
                            usb_err(&err),
                            attempt + 1,
                            ENUM_ATTEMPTS,
                        );
                        if attempt + 1 < ENUM_ATTEMPTS {
                            attempt += 1;
                            delay_ms(ENUM_RETRY_PAUSE_MS).await;
                            log::info!("Retrying with a fresh port reset");
                            continue 'port;
                        }
                        log::warn!(
                            "Giving up after {} attempts. Unplug the device and plug it in again.",
                            ENUM_ATTEMPTS
                        );
                    }
                    Some(DeviceEvent::HubConnect(hub)) => {
                        log::info!("DeviceEvent::HubConnect  addr={}", hub.address());
                    }
                    Some(DeviceEvent::None) => {}
                    None => {
                        log::warn!("Device event stream ended");
                        break 'port;
                    }
                }
            }
        }
    }
}
