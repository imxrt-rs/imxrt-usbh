//! `HostController` trait implementation for `Imxrt1062HostController`.

use core::cell::Cell;
use cotton_usb_host::host_controller::{
    DataPhase, HostController, TransferExtras, TransferType, UsbError,
};
use cotton_usb_host::wire::SetupPacket;

use super::controller::Imxrt1062HostController;
use super::device_detect::Imxrt1062DeviceDetect;
use super::interrupt_pipe::{Imxrt1062InterruptPipe, Pipe};

use crate::ehci::{PID_IN, PID_OUT};

impl HostController for Imxrt1062HostController {
    type InterruptPipe = Imxrt1062InterruptPipe;
    type DeviceDetect = Imxrt1062DeviceDetect;

    fn device_detect(&self) -> Self::DeviceDetect {
        Imxrt1062DeviceDetect::new(&self.usb, &self.usbphy, self.shared.device_waker())
    }

    fn reset_root_port(&self, rst: bool) {
        if rst {
            // Set PORTSC1.PR (bit 8) — begin USB reset signaling.
            // Must preserve other bits and avoid clearing W1C bits.
            let portsc = self.portsc1_read_safe();
            crate::ral::write_reg!(crate::ral::usb, self.usb, PORTSC1, portsc | (1 << 8));
        } else {
            // Clear PORTSC1.PR (bit 8) — end USB reset signaling.
            // On EHCI, the controller may auto-clear PR and set PE (port enabled).
            let portsc = self.portsc1_read_safe();
            crate::ral::write_reg!(crate::ral::usb, self.usb, PORTSC1, portsc & !(1 << 8));
        }
    }

    async fn control_transfer<'a>(
        &self,
        address: u8,
        transfer_extras: TransferExtras,
        packet_size: u8,
        setup: SetupPacket,
        mut data_phase: DataPhase<'a>,
    ) -> Result<usize, UsbError> {
        let _data_len = match &data_phase {
            DataPhase::In(buf) => buf.len() as i32,
            DataPhase::Out(buf) => -(buf.len() as i32),
            DataPhase::None => 0,
        };
        // Allocate a control pipe (serializes control transfers)
        let _pipe = Pipe::new(self.statics.control_pipes.alloc().await, 0);

        let result = self.do_control_transfer(address, transfer_extras, packet_size, &setup, &mut data_phase)
            .await;

        if let Ok(n) = &result {
            trace!("[HC] control_transfer -> Ok({})", n);
        } else if let Err(ref e) = result {
            warn!("[HC] control_transfer -> Err({})", Self::usb_error_str(e));
        }
        result
    }

    async fn bulk_in_transfer(
        &self,
        address: u8,
        endpoint: u8,
        packet_size: u16,
        data: &mut [u8],
        transfer_type: TransferType,
        data_toggle: &Cell<bool>,
    ) -> Result<usize, UsbError> {
        self.do_bulk_transfer(
            address,
            endpoint,
            packet_size,
            data.as_mut_ptr(),
            data.len(),
            PID_IN,
            true,
            transfer_type,
            data_toggle,
        )
        .await
    }

    async fn bulk_out_transfer(
        &self,
        address: u8,
        endpoint: u8,
        packet_size: u16,
        data: &[u8],
        transfer_type: TransferType,
        data_toggle: &Cell<bool>,
    ) -> Result<usize, UsbError> {
        self.do_bulk_transfer(
            address,
            endpoint,
            packet_size,
            data.as_ptr() as *mut u8,
            data.len(),
            PID_OUT,
            false,
            transfer_type,
            data_toggle,
        )
        .await
    }

    async fn alloc_interrupt_pipe(
        &self,
        address: u8,
        transfer_extras: TransferExtras,
        endpoint: u8,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Imxrt1062InterruptPipe {
        let pipe = Pipe::new(self.statics.bulk_pipes.alloc().await, 1);
        self.do_alloc_interrupt_pipe(
            pipe,
            address,
            transfer_extras,
            endpoint,
            max_packet_size,
            interval_ms,
        )
    }

    fn try_alloc_interrupt_pipe(
        &self,
        address: u8,
        transfer_extras: TransferExtras,
        endpoint: u8,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::InterruptPipe, UsbError> {
        let pooled = self
            .statics
            .bulk_pipes
            .try_alloc()
            .ok_or(UsbError::AllPipesInUse)?;
        Ok(self.do_alloc_interrupt_pipe(
            Pipe::new(pooled, 1),
            address,
            transfer_extras,
            endpoint,
            max_packet_size,
            interval_ms,
        ))
    }
}
