//! USB general purpose timers.
//!
//! Each USB OTG peripheral has two general purpose timers (GPT) with 1µs resolution
//! and 24-bit counters. In host mode, these are useful for:
//!
//! - Port reset timing (USB spec requires specific reset pulse durations)
//! - Transfer timeouts (detecting unresponsive devices)
//! - Connection debouncing
//!
//! # Usage
//!
//! ```no_run,ignore
//! use imxrt_usbh::gpt;
//!
//! // Configure GPT0 for a 50ms one-shot timeout
//! let mut gpt = gpt::Gpt::new(&mut usb, gpt::Instance::Gpt0);
//! gpt.stop();
//! gpt.clear_elapsed();
//! gpt.set_mode(gpt::Mode::OneShot);
//! gpt.set_load(50_000); // 50ms
//! gpt.reset();
//! gpt.run();
//!
//! // Later, check if the timer has elapsed
//! if gpt.is_elapsed() {
//!     gpt.clear_elapsed();
//!     // Timeout!
//! }
//! ```

use crate::ral;

/// GPT timer mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum Mode {
    /// In one shot mode, the timer will count down to zero, generate an interrupt,
    /// and stop until the counter is reset by software.
    OneShot = 0,
    /// In repeat mode, the timer will count down to zero, generate an interrupt and
    /// automatically reload the counter value to start again.
    Repeat = 1,
}

/// GPT instance identifiers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum Instance {
    /// The GPT0 timer instance.
    Gpt0,
    /// The GPT1 timer instance.
    Gpt1,
}

/// General purpose timer (GPT).
///
/// USB GPTs have a 1µs resolution. The counter is 24 bits wide. GPTs can generate
/// USB interrupts that are independent of USB protocol interrupts.
pub struct Gpt<'a> {
    usb: &'a mut ral::usb::Instance,
    gpt: Instance,
}

impl<'a> Gpt<'a> {
    /// Create a GPT instance over the USB core registers.
    ///
    /// Takes a mutable reference to the USB register block to prevent
    /// aliasing the same GPT instance.
    ///
    /// # Example
    ///
    /// ```ignore
    /// let mut gpt = Gpt::new(host.usb_mut(), Instance::Gpt0);
    /// gpt.set_mode(Mode::OneShot);
    /// gpt.set_load(50_000); // 50ms
    /// gpt.reset();
    /// gpt.run();
    /// ```
    pub fn new(usb: &'a mut ral::usb::Instance, gpt: Instance) -> Self {
        Self { usb, gpt }
    }

    /// Returns the GPT instance identifier.
    pub fn instance(&self) -> Instance {
        self.gpt
    }

    /// Run the GPT timer.
    ///
    /// Starts counting down. Use [`stop()`](Gpt::stop) to cancel a running timer.
    pub fn run(&mut self) {
        match self.gpt {
            Instance::Gpt0 => ral::modify_reg!(ral::usb, self.usb, GPTIMER0CTRL, GPTRUN: 1),
            Instance::Gpt1 => ral::modify_reg!(ral::usb, self.usb, GPTIMER1CTRL, GPTRUN: 1),
        }
    }

    /// Indicates if the timer is running (`true`) or stopped (`false`).
    pub fn is_running(&self) -> bool {
        match self.gpt {
            Instance::Gpt0 => ral::read_reg!(ral::usb, self.usb, GPTIMER0CTRL, GPTRUN == 1),
            Instance::Gpt1 => ral::read_reg!(ral::usb, self.usb, GPTIMER1CTRL, GPTRUN == 1),
        }
    }

    /// Stop the timer.
    pub fn stop(&mut self) {
        match self.gpt {
            Instance::Gpt0 => ral::modify_reg!(ral::usb, self.usb, GPTIMER0CTRL, GPTRUN: 0),
            Instance::Gpt1 => ral::modify_reg!(ral::usb, self.usb, GPTIMER1CTRL, GPTRUN: 0),
        }
    }

    /// Reset the timer.
    ///
    /// Loads the counter value. Does not stop a running counter.
    pub fn reset(&mut self) {
        match self.gpt {
            Instance::Gpt0 => ral::modify_reg!(ral::usb, self.usb, GPTIMER0CTRL, GPTRST: 1),
            Instance::Gpt1 => ral::modify_reg!(ral::usb, self.usb, GPTIMER1CTRL, GPTRST: 1),
        }
    }

    /// Set the timer mode.
    pub fn set_mode(&mut self, mode: Mode) {
        match self.gpt {
            Instance::Gpt0 => {
                ral::modify_reg!(ral::usb, self.usb, GPTIMER0CTRL, GPTMODE: mode as u32)
            }
            Instance::Gpt1 => {
                ral::modify_reg!(ral::usb, self.usb, GPTIMER1CTRL, GPTMODE: mode as u32)
            }
        }
    }

    /// Returns the timer mode.
    ///
    /// # Panics
    ///
    /// Panics if the hardware returns an unexpected value (should be unreachable
    /// since the GPTMODE field is a single bit).
    pub fn mode(&self) -> Mode {
        let mode: u32 = match self.gpt {
            Instance::Gpt0 => ral::read_reg!(ral::usb, self.usb, GPTIMER0CTRL, GPTMODE),
            Instance::Gpt1 => ral::read_reg!(ral::usb, self.usb, GPTIMER1CTRL, GPTMODE),
        };

        if mode == (Mode::Repeat as u32) {
            Mode::Repeat
        } else if mode == (Mode::OneShot as u32) {
            Mode::OneShot
        } else {
            unreachable!()
        }
    }

    /// Set the counter load value.
    ///
    /// `us` is the number of microseconds to count. Saturates at a 24-bit value
    /// (0xFFFFFF, or ~16.78 seconds). A value of `0` results in a 1µs delay.
    ///
    /// The load value is not applied until the next call to [`reset()`](Gpt::reset)
    /// (one shot mode) or until the timer elapses (repeat mode).
    pub fn set_load(&mut self, us: u32) {
        let count = us.clamp(1, 0xFF_FFFF).saturating_sub(1);
        match self.gpt {
            Instance::Gpt0 => ral::write_reg!(ral::usb, self.usb, GPTIMER0LD, count),
            Instance::Gpt1 => ral::write_reg!(ral::usb, self.usb, GPTIMER1LD, count),
        }
    }

    /// Returns the counter load value.
    pub fn load(&self) -> u32 {
        match self.gpt {
            Instance::Gpt0 => ral::read_reg!(ral::usb, self.usb, GPTIMER0LD),
            Instance::Gpt1 => ral::read_reg!(ral::usb, self.usb, GPTIMER1LD),
        }
    }

    /// Indicates if the timer has elapsed.
    ///
    /// If elapsed, clear the flag with [`clear_elapsed()`](Gpt::clear_elapsed).
    pub fn is_elapsed(&self) -> bool {
        match self.gpt {
            Instance::Gpt0 => ral::read_reg!(ral::usb, self.usb, USBSTS, TI0 == 1),
            Instance::Gpt1 => ral::read_reg!(ral::usb, self.usb, USBSTS, TI1 == 1),
        }
    }

    /// Clear the elapsed flag.
    pub fn clear_elapsed(&mut self) {
        match self.gpt {
            Instance::Gpt0 => ral::write_reg!(ral::usb, self.usb, USBSTS, TI0: 1),
            Instance::Gpt1 => ral::write_reg!(ral::usb, self.usb, USBSTS, TI1: 1),
        }
    }

    /// Enable or disable interrupt generation when the timer elapses.
    ///
    /// If enabled (`true`), an elapsed GPT will generate an interrupt regardless
    /// of other USB interrupt enable state.
    pub fn set_interrupt_enabled(&mut self, enable: bool) {
        match self.gpt {
            Instance::Gpt0 => ral::modify_reg!(ral::usb, self.usb, USBINTR, TIE0: enable as u32),
            Instance::Gpt1 => ral::modify_reg!(ral::usb, self.usb, USBINTR, TIE1: enable as u32),
        }
    }

    /// Indicates if interrupt generation is enabled.
    pub fn is_interrupt_enabled(&self) -> bool {
        match self.gpt {
            Instance::Gpt0 => ral::read_reg!(ral::usb, self.usb, USBINTR, TIE0 == 1),
            Instance::Gpt1 => ral::read_reg!(ral::usb, self.usb, USBINTR, TIE1 == 1),
        }
    }
}
