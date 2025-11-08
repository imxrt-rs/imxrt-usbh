#![cfg_attr(target_os = "none", no_std)]

pub use cotton_usb_host::{async_pool, bitset, host_controller, topology, usb_bus, wire};
use imxrt_ral as ral;

#[expect(dead_code)]
pub struct HostController<const N: u8> {
    // Not sure if we need more or fewer instances!
    // We'll figure that out later.
    usb: ral::usb::Instance<N>,
    phy: ral::usbphy::Instance<N>,
    nc: ral::usbnc::Instance<N>,
}

impl<const N: u8> HostController<N> {
    pub fn new(
        usb: ral::usb::Instance<N>,
        phy: ral::usbphy::Instance<N>,
        nc: ral::usbnc::Instance<N>,
    ) -> Self {
        Self { usb, phy, nc }
    }
}

pub struct InterruptPipe {}
pub struct DeviceDetect {}

impl futures_core::Stream for InterruptPipe {
    type Item = host_controller::InterruptPacket;

    fn poll_next(
        self: core::pin::Pin<&mut Self>,
        _: &mut core::task::Context<'_>,
    ) -> core::task::Poll<Option<Self::Item>> {
        defmt::todo!()
    }
}

impl futures_core::Stream for DeviceDetect {
    type Item = host_controller::DeviceStatus;

    fn poll_next(
        self: core::pin::Pin<&mut Self>,
        _: &mut core::task::Context<'_>,
    ) -> core::task::Poll<Option<Self::Item>> {
        defmt::todo!()
    }
}

#[expect(unused_variables, reason = "Autofill filled it in")]
impl<const N: u8> host_controller::HostController for HostController<N> {
    type InterruptPipe = InterruptPipe;
    type DeviceDetect = DeviceDetect;

    fn device_detect(&self) -> Self::DeviceDetect {
        defmt::todo!()
    }

    fn reset_root_port(&self, rst: bool) {
        defmt::todo!()
    }

    async fn control_transfer(
        &self,
        address: u8,
        transfer_extras: usb_bus::TransferExtras,
        packet_size: u8,
        setup: wire::SetupPacket,
        data_phase: usb_bus::DataPhase<'_>,
    ) -> Result<usize, usb_bus::UsbError> {
        defmt::todo!()
    }

    async fn bulk_in_transfer(
        &self,
        address: u8,
        endpoint: u8,
        packet_size: u16,
        data: &mut [u8],
        transfer_type: usb_bus::TransferType,
        data_toggle: &core::cell::Cell<bool>,
    ) -> Result<usize, usb_bus::UsbError> {
        defmt::todo!()
    }

    async fn bulk_out_transfer(
        &self,
        address: u8,
        endpoint: u8,
        packet_size: u16,
        data: &[u8],
        transfer_type: usb_bus::TransferType,
        data_toggle: &core::cell::Cell<bool>,
    ) -> Result<usize, usb_bus::UsbError> {
        defmt::todo!()
    }

    async fn alloc_interrupt_pipe(
        &self,
        address: u8,
        transfer_extras: usb_bus::TransferExtras,
        endpoint: u8,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Self::InterruptPipe {
        defmt::todo!()
    }

    fn try_alloc_interrupt_pipe(
        &self,
        address: u8,
        transfer_extras: usb_bus::TransferExtras,
        endpoint: u8,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::InterruptPipe, usb_bus::UsbError> {
        defmt::todo!()
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn you_can_test_the_units() {
        assert_eq!(1 + 1, 2);
    }
}
