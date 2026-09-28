use cotton_usb_host::{
    device::identify::IdentifyFromDescriptors,
    host_controller::{HostController, UsbError},
    usb_bus::{BulkIn, BulkOut, TransferType, UsbBus, UsbDevice},
    wire::{ConfigurationDescriptor, DescriptorVisitor, EndpointDescriptor, InterfaceDescriptor},
};

use crate::debug;

/// A 4-byte USB-MIDI Event Packet as defined in USB MIDI 1.0 spec section 4.
///
/// Every USB MIDI transfer consists of one or more of these 4-byte packets:
/// - Byte 0: `[Cable Number (4 bits)][Code Index Number (4 bits)]`
/// - Bytes 1-3: MIDI data bytes (padded with 0x00 for shorter messages)
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct UsbMidiEventPacket {
    data: [u8; 4],
}

impl UsbMidiEventPacket {
    /// Create a packet from raw bytes.
    pub fn from_bytes(bytes: [u8; 4]) -> Self {
        Self { data: bytes }
    }

    /// Cable Number (bits 7:4 of byte 0).
    pub fn cable_number(&self) -> u8 {
        self.data[0] >> 4
    }

    /// Code Index Number (bits 3:0 of byte 0).
    ///
    /// Classifies the MIDI event type per USB MIDI 1.0 Table 4-1.
    pub fn code_index_number(&self) -> u8 {
        self.data[0] & 0x0F
    }

    /// Number of valid MIDI data bytes (1-3) based on the CIN.
    ///
    /// Per USB MIDI 1.0 Table 4-1:
    /// - CIN 0x00: reserved (1 byte)
    /// - CIN 0x01: reserved (1 byte)
    /// - CIN 0x02: 2 bytes (two-byte system common)
    /// - CIN 0x03: 3 bytes (three-byte system common)
    /// - CIN 0x04: 3 bytes (SysEx starts or continues)
    /// - CIN 0x05: 1 byte (single-byte system common, or SysEx ends with 1 byte)
    /// - CIN 0x06: 2 bytes (SysEx ends with 2 bytes)
    /// - CIN 0x07: 3 bytes (SysEx ends with 3 bytes)
    /// - CIN 0x08: 3 bytes (Note Off)
    /// - CIN 0x09: 3 bytes (Note On)
    /// - CIN 0x0A: 3 bytes (Poly KeyPress)
    /// - CIN 0x0B: 3 bytes (Control Change)
    /// - CIN 0x0C: 2 bytes (Program Change)
    /// - CIN 0x0D: 2 bytes (Channel Pressure)
    /// - CIN 0x0E: 3 bytes (PitchBend Change)
    /// - CIN 0x0F: 1 byte (Single Byte)
    pub fn midi_data_len(&self) -> usize {
        match self.code_index_number() {
            0x00 | 0x01 => 1,
            0x02 => 2,
            0x03 => 3,
            0x04 => 3,
            0x05 => 1,
            0x06 => 2,
            0x07 => 3,
            0x08 => 3,
            0x09 => 3,
            0x0A => 3,
            0x0B => 3,
            0x0C => 2,
            0x0D => 2,
            0x0E => 3,
            0x0F => 1,
            _ => 1,
        }
    }

    /// The raw MIDI data bytes (bytes 1-3 of the packet).
    pub fn midi_bytes(&self) -> &[u8] {
        &self.data[1..1 + self.midi_data_len()]
    }

    /// Returns true if this is an empty/padding packet (all zeros).
    pub fn is_empty(&self) -> bool {
        self.data == [0, 0, 0, 0]
    }

    /// The raw 4-byte packet data.
    pub fn as_bytes(&self) -> &[u8; 4] {
        &self.data
    }
}

/// Descriptor visitor that identifies USB MIDI Streaming interfaces.
///
/// Matches Audio class (0x01), MIDI Streaming subclass (0x03).
/// Records the first bulk IN and bulk OUT endpoint addresses found.
#[derive(Default)]
pub struct IdentifyMidi {
    current_configuration: Option<u8>,
    midi_configuration: Option<u8>,
    midi_interface: bool,
    ep_in: Option<u8>,
    ep_out: Option<u8>,
}

impl IdentifyMidi {
    /// USB Audio class code.
    pub const AUDIO_CLASS: u8 = 0x01;
    /// USB MIDI Streaming subclass code.
    pub const MIDI_STREAMING_SUBCLASS: u8 = 0x03;

    /// Bulk IN endpoint number, if found.
    pub fn in_endpoint(&self) -> Option<u8> {
        self.ep_in
    }

    /// Bulk OUT endpoint number, if found.
    pub fn out_endpoint(&self) -> Option<u8> {
        self.ep_out
    }
}

impl DescriptorVisitor for IdentifyMidi {
    fn on_configuration(&mut self, c: &ConfigurationDescriptor) {
        self.current_configuration = Some(c.bConfigurationValue);
    }

    fn on_interface(&mut self, i: &InterfaceDescriptor) {
        if i.bInterfaceClass == Self::AUDIO_CLASS
            && i.bInterfaceSubClass == Self::MIDI_STREAMING_SUBCLASS
        {
            debug::println!(
                "MIDI Streaming interface: iface={} alt={}",
                i.bInterfaceNumber,
                i.bAlternateSetting
            );
            self.midi_interface = true;
            self.midi_configuration = self.current_configuration;
        } else {
            self.midi_interface = false;
        }
    }

    fn on_endpoint(&mut self, e: &EndpointDescriptor) {
        if !self.midi_interface {
            return;
        }
        // Bulk transfer type: bmAttributes[1:0] == 0b10
        if (e.bmAttributes & 0x03) != 0x02 {
            return;
        }
        let ep_num = e.bEndpointAddress & 0x0F;
        if (e.bEndpointAddress & 0x80) != 0 {
            // IN endpoint
            if self.ep_in.is_none() {
                debug::println!("MIDI bulk IN endpoint: {}", ep_num);
                self.ep_in = Some(ep_num);
            }
        } else {
            // OUT endpoint
            if self.ep_out.is_none() {
                debug::println!("MIDI bulk OUT endpoint: {}", ep_num);
                self.ep_out = Some(ep_num);
            }
        }
    }

    fn on_other(&mut self, d: &[u8]) {
        if d.len() >= 2 && d[1] == 0x24 {
            // CS_INTERFACE descriptor (Audio class-specific)
            let subtype = if d.len() >= 3 { d[2] } else { 0 };
            if subtype == 0x01 {
                debug::println!("  CS_INTERFACE: MS Header");
            } else if subtype == 0x02 {
                debug::println!("  CS_INTERFACE: MIDI IN Jack");
            } else if subtype == 0x03 {
                debug::println!("  CS_INTERFACE: MIDI OUT Jack");
            } else if subtype == 0x04 {
                debug::println!("  CS_INTERFACE: Element");
            } else {
                debug::println!("  CS_INTERFACE: subtype={}", subtype);
            }
        }
    }
}

impl IdentifyFromDescriptors for IdentifyMidi {
    fn identify(&self) -> Option<u8> {
        // Only identify as MIDI if we found at least a bulk IN endpoint
        if self.ep_in.is_some() {
            self.midi_configuration
        } else {
            None
        }
    }
}

/// Copy as many whole packets from `packets` into `buf` as it holds, and
/// return how many that was.
fn pack(buf: &mut [u8], packets: &[UsbMidiEventPacket]) -> usize {
    let count = packets.len().min(buf.len() / 4);
    for (chunk, packet) in buf.chunks_exact_mut(4).zip(&packets[..count]) {
        chunk.copy_from_slice(packet.as_bytes());
    }
    count
}

/// USB MIDI device driver.
///
/// Holds bulk endpoint handles and provides packet-based read and write
/// access. [`read_packets`](Self::read_packets) and
/// [`write_packets`](Self::write_packets) both take `&self` and use different
/// endpoints, so they can run concurrently, for example under
/// `futures::future::select`, with a read left pending while a write goes out.
pub struct Midi<'a, HC: HostController> {
    bus: &'a UsbBus<HC>,
    bulk_in: BulkIn,
    bulk_out: Option<BulkOut>,
}

impl<'a, HC: HostController> Midi<'a, HC> {
    /// Create a new MIDI driver instance.
    ///
    /// Opens the bulk IN endpoint (required) and optionally the bulk OUT endpoint.
    /// `in_ep` and `out_ep` are endpoint numbers (1-15), not addresses.
    pub fn new(
        bus: &'a UsbBus<HC>,
        mut device: UsbDevice,
        in_ep: u8,
        out_ep: Option<u8>,
    ) -> Result<Self, UsbError> {
        let bulk_in = device.open_in_endpoint(in_ep)?;
        let bulk_out = if let Some(ep) = out_ep {
            Some(device.open_out_endpoint(ep)?)
        } else {
            None
        };
        Ok(Self {
            bus,
            bulk_in,
            bulk_out,
        })
    }

    /// Whether the device has a bulk OUT endpoint, so that
    /// [`write_packets`](Self::write_packets) can send to it.
    pub fn has_out_endpoint(&self) -> bool {
        self.bulk_out.is_some()
    }

    /// Perform one bulk IN transfer and parse the received data into
    /// USB-MIDI event packets.
    ///
    /// - `recv_buf`: Buffer for the raw USB bulk transfer. Should be a multiple
    ///   of 4 bytes and large enough for one USB packet (typically 64 bytes).
    ///   Must be in DMA-accessible memory (not stack/DTCM on Cortex-M7).
    /// - `packet_buf`: Output buffer for parsed non-empty packets.
    ///
    /// Returns the number of non-empty packets written to `packet_buf`,
    /// or an error if the bulk transfer failed.
    pub async fn read_packets(
        &self,
        recv_buf: &mut [u8],
        packet_buf: &mut [UsbMidiEventPacket],
    ) -> Result<usize, UsbError> {
        let bytes_read = self
            .bus
            .bulk_in_transfer(&self.bulk_in, recv_buf, TransferType::VariableSize)
            .await?;

        let mut count = 0;
        // Parse 4-byte packets from the received data
        let num_packets = bytes_read / 4;
        for i in 0..num_packets {
            let offset = i * 4;
            let pkt = UsbMidiEventPacket::from_bytes([
                recv_buf[offset],
                recv_buf[offset + 1],
                recv_buf[offset + 2],
                recv_buf[offset + 3],
            ]);
            if !pkt.is_empty() {
                if count < packet_buf.len() {
                    packet_buf[count] = pkt;
                    count += 1;
                }
            }
        }
        Ok(count)
    }

    /// Send USB-MIDI event packets to the device over the bulk OUT endpoint.
    ///
    /// - `send_buf`: Buffer for the raw USB bulk transfers. It must be in
    ///   DMA-accessible memory (not stack/DTCM on Cortex-M7), like `recv_buf`
    ///   in [`read_packets`](Self::read_packets), and hold at least one
    ///   4-byte packet. Each transfer carries as many whole packets as it
    ///   holds; 64 bytes, one full-speed bulk packet, carries 16.
    /// - `packets`: The packets to send, in order. Each carries its own cable
    ///   number and Code Index Number; see [`UsbMidiEventPacket`].
    ///
    /// Sends `packets` in as many transfers as it takes and returns how many
    /// packets were sent, which is all of them unless there is an error. On
    /// an error, the transfers before it have been sent.
    ///
    /// Fails with [`UsbError::NoSuchEndpoint`] if the device has no bulk OUT
    /// endpoint (see [`has_out_endpoint`](Self::has_out_endpoint)), and with
    /// [`UsbError::BufferTooSmall`] if `send_buf` cannot hold one packet.
    pub async fn write_packets(
        &self,
        send_buf: &mut [u8],
        packets: &[UsbMidiEventPacket],
    ) -> Result<usize, UsbError> {
        let bulk_out = self.bulk_out.as_ref().ok_or(UsbError::NoSuchEndpoint)?;
        if send_buf.len() < 4 {
            return Err(UsbError::BufferTooSmall);
        }

        let mut sent = 0;
        while sent < packets.len() {
            let count = pack(send_buf, &packets[sent..]);
            // USB-MIDI needs no zero-length packet after a transfer that
            // fills whole packets, so the transfer is fixed-size.
            self.bus
                .bulk_out_transfer(bulk_out, &send_buf[..count * 4], TransferType::FixedSize)
                .await?;
            sent += count;
        }
        Ok(sent)
    }
}

#[cfg(all(test, feature = "std"))]
mod tests {
    use super::*;

    fn note_on(note: u8) -> UsbMidiEventPacket {
        UsbMidiEventPacket::from_bytes([0x09, 0x90, note, 127])
    }

    #[test]
    fn test_pack_fits_everything() {
        let packets = [note_on(1), note_on(2)];
        let mut buf = [0xAA; 16];
        assert_eq!(pack(&mut buf, &packets), 2);
        assert_eq!(&buf[..8], &[0x09, 0x90, 1, 127, 0x09, 0x90, 2, 127]);
        // Beyond the packed packets, the buffer is untouched.
        assert_eq!(&buf[8..], &[0xAA; 8]);
    }

    #[test]
    fn test_pack_stops_at_buffer_capacity() {
        let packets: Vec<_> = (0..20).map(note_on).collect();
        let mut buf = [0; 64];
        assert_eq!(pack(&mut buf, &packets), 16);
        assert_eq!(&buf[60..64], &[0x09, 0x90, 15, 127]);
    }

    #[test]
    fn test_pack_ignores_a_partial_packet_of_space() {
        let packets = [note_on(1), note_on(2)];
        let mut buf = [0; 7];
        assert_eq!(pack(&mut buf, &packets), 1);
        let mut small = [0; 3];
        assert_eq!(pack(&mut small, &packets), 0);
    }

    #[test]
    fn test_pack_nothing() {
        let mut buf = [0; 64];
        assert_eq!(pack(&mut buf, &[]), 0);
    }

    #[test]
    fn test_note_on_packet() {
        // CIN=0x09 (Note On), cable=0, channel 1, note 60, velocity 100
        let pkt = UsbMidiEventPacket::from_bytes([0x09, 0x90, 60, 100]);
        assert_eq!(pkt.cable_number(), 0);
        assert_eq!(pkt.code_index_number(), 0x09);
        assert_eq!(pkt.midi_data_len(), 3);
        assert_eq!(pkt.midi_bytes(), &[0x90, 60, 100]);
        assert!(!pkt.is_empty());
    }

    #[test]
    fn test_note_off_packet() {
        // CIN=0x08 (Note Off), cable=0, channel 1, note 60, velocity 0
        let pkt = UsbMidiEventPacket::from_bytes([0x08, 0x80, 60, 0]);
        assert_eq!(pkt.cable_number(), 0);
        assert_eq!(pkt.code_index_number(), 0x08);
        assert_eq!(pkt.midi_data_len(), 3);
        assert_eq!(pkt.midi_bytes(), &[0x80, 60, 0]);
    }

    #[test]
    fn test_control_change_packet() {
        // CIN=0x0B (CC), cable=0, channel 1, CC#1 (mod wheel), value 64
        let pkt = UsbMidiEventPacket::from_bytes([0x0B, 0xB0, 1, 64]);
        assert_eq!(pkt.code_index_number(), 0x0B);
        assert_eq!(pkt.midi_data_len(), 3);
        assert_eq!(pkt.midi_bytes(), &[0xB0, 1, 64]);
    }

    #[test]
    fn test_pitch_bend_packet() {
        // CIN=0x0E (Pitch Bend), cable=0, channel 1, LSB=0, MSB=64 (center)
        let pkt = UsbMidiEventPacket::from_bytes([0x0E, 0xE0, 0x00, 0x40]);
        assert_eq!(pkt.code_index_number(), 0x0E);
        assert_eq!(pkt.midi_data_len(), 3);
        assert_eq!(pkt.midi_bytes(), &[0xE0, 0x00, 0x40]);
    }

    #[test]
    fn test_program_change_packet() {
        // CIN=0x0C (Program Change), cable=0, channel 1, program 5
        let pkt = UsbMidiEventPacket::from_bytes([0x0C, 0xC0, 5, 0]);
        assert_eq!(pkt.code_index_number(), 0x0C);
        assert_eq!(pkt.midi_data_len(), 2);
        assert_eq!(pkt.midi_bytes(), &[0xC0, 5]);
    }

    #[test]
    fn test_empty_packet() {
        let pkt = UsbMidiEventPacket::from_bytes([0, 0, 0, 0]);
        assert!(pkt.is_empty());
    }

    #[test]
    fn test_cable_number() {
        // Cable 3, CIN=0x09 (Note On)
        let pkt = UsbMidiEventPacket::from_bytes([0x39, 0x90, 60, 100]);
        assert_eq!(pkt.cable_number(), 3);
        assert_eq!(pkt.code_index_number(), 0x09);
    }

    #[test]
    fn test_identify_midi_audio_class() {
        use cotton_usb_host::wire::parse_descriptors;

        // Minimal configuration descriptor with Audio class, MIDI Streaming interface
        // and one bulk IN endpoint
        let desc: &[u8] = &[
            // Configuration descriptor (9 bytes)
            9, 2, 32, 0, 1, 1, 0, 0x80, 50,
            // Interface descriptor (9 bytes): class=1 (Audio), subclass=3 (MIDI Streaming)
            9, 4, 0, 0, 1, 0x01, 0x03, 0, 0, // Endpoint descriptor (7 bytes): bulk IN, ep 1
            7, 5, 0x81, 0x02, 64, 0, 0,
        ];
        let mut id = IdentifyMidi::default();
        parse_descriptors(desc, &mut id);
        assert_eq!(id.identify(), Some(1));
        assert_eq!(id.in_endpoint(), Some(1));
        assert_eq!(id.out_endpoint(), None);
    }

    #[test]
    fn test_identify_midi_with_both_endpoints() {
        use cotton_usb_host::wire::parse_descriptors;

        let desc: &[u8] = &[
            // Configuration descriptor
            9, 2, 39, 0, 1, 1, 0, 0x80, 50,
            // Interface descriptor: Audio class, MIDI Streaming
            9, 4, 0, 0, 2, 0x01, 0x03, 0, 0, // Endpoint descriptor: bulk IN, ep 1
            7, 5, 0x81, 0x02, 64, 0, 0, // Endpoint descriptor: bulk OUT, ep 2
            7, 5, 0x02, 0x02, 64, 0, 0,
        ];
        let mut id = IdentifyMidi::default();
        parse_descriptors(desc, &mut id);
        assert_eq!(id.identify(), Some(1));
        assert_eq!(id.in_endpoint(), Some(1));
        assert_eq!(id.out_endpoint(), Some(2));
    }

    #[test]
    fn test_identify_non_midi() {
        use cotton_usb_host::wire::parse_descriptors;

        // HID device, not MIDI
        let desc: &[u8] = &[
            // Configuration descriptor
            9, 2, 25, 0, 1, 1, 0, 0x80, 50, // Interface descriptor: HID class
            9, 4, 0, 0, 1, 0x03, 0x01, 0x01, 0, // Endpoint descriptor: interrupt IN
            7, 5, 0x81, 0x03, 8, 0, 10,
        ];
        let mut id = IdentifyMidi::default();
        parse_descriptors(desc, &mut id);
        assert_eq!(id.identify(), None);
    }

    #[test]
    fn test_identify_midi_ignores_interrupt_endpoints() {
        use cotton_usb_host::wire::parse_descriptors;

        // MIDI interface but only interrupt endpoints (not bulk)
        let desc: &[u8] = &[
            9, 2, 25, 0, 1, 1, 0, 0x80, 50, 9, 4, 0, 0, 1, 0x01, 0x03, 0, 0,
            // Interrupt IN endpoint (bmAttributes=0x03, not bulk=0x02)
            7, 5, 0x81, 0x03, 64, 0, 10,
        ];
        let mut id = IdentifyMidi::default();
        parse_descriptors(desc, &mut id);
        // Should not identify because no bulk endpoint found
        assert_eq!(id.identify(), None);
    }
}
