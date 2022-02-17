//! FSM for a USB-C DFP implementation. USB UFP or DRP are not supported supported as of now

use super::fusb302::{self, Fusb302};

use embedded_hal::{
    blocking::{
        delay::DelayUs,
        i2c::{Read, Write},
    },
    digital::v2::InputPin,
};
#[derive(defmt::Format)]
enum State {
    Disconnected,
    DebounceConnection,
    ApplyVbusAndVconn,
    Connected,
}

pub enum Error<T>
where
    T: Read + Write,
{
    DeviceError(fusb302::Error<T>),
}

impl<T: Read + Write> core::fmt::Debug for Error<T> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::DeviceError(inner) => write!(f, "DeviceError({:?})", inner),
        }
    }
}

impl<T: Read + Write> From<fusb302::Error<T>> for Error<T> {
    fn from(error: fusb302::Error<T>) -> Self {
        Self::DeviceError(error)
    }
}

pub struct UsbFsm<T, U>
where
    T: Read + Write,
    U: InputPin,
{
    pd_controller: Fusb302<T>,
    irq_pin: U,
    current_state: State,
}

impl<T, U> UsbFsm<T, U>
where
    T: Read + Write,
    U: InputPin,
{
    pub fn new(mut pd_controller: Fusb302<T>, irq_pin: U) -> Result<Self, Error<T>> {
        pd_controller.init()?;
        pd_controller.reset_pd()?;
        pd_controller.configure_cc_mode(fusb302::PortType::Open)?;
        pd_controller.enable_interrupts()?;

        Ok(Self {
            pd_controller,
            irq_pin,
            current_state: State::Disconnected,
        })
    }

    pub fn run<V: DelayUs<u32>>(&mut self, timer: &mut V) -> Result<(), Error<T>> {
        // No interrupts should be received yet, since they are off
        if self.irq_pin.is_low().unwrap_or(false) {
            defmt::info!("Irq from fusb302");
        }

        match self.current_state {
            State::Disconnected => match self.pd_controller.detect_cc_orientation(timer)? {
                Some(orientation) => {
                    defmt::info!("Detected line {}", orientation);
                    self.trigger_transition(State::DebounceConnection);
                }
                None => {}
            },
            State::DebounceConnection => {
                let detected_orientation = self.pd_controller.get_polarity().unwrap();
                match self.pd_controller.detect_cc_orientation(timer)? {
                    Some(orientation) if orientation == detected_orientation => {
                        self.trigger_transition(State::ApplyVbusAndVconn);
                    }
                    Some(orientation) => {
                        defmt::warn!("Orientation {} does not match", orientation);
                        self.trigger_transition(State::Disconnected);
                    }
                    _ => {
                        defmt::info!("Device disconnected");
                        self.trigger_transition(State::Disconnected);
                    }
                }
            }
            State::ApplyVbusAndVconn => {
                self.pd_controller
                    .configure_cc_mode(fusb302::PortType::DFP)?;
                self.trigger_transition(State::Connected);
            }
            State::Connected => {
                if !self.pd_controller.monitor_connection(timer)? {
                    defmt::info!("Device disconnected");
                    self.pd_controller
                        .configure_cc_mode(fusb302::PortType::Open)?;
                    self.trigger_transition(State::Disconnected);
                }
            }
        }

        Ok(())
    }

    fn trigger_transition(&mut self, state: State) {
        defmt::info!("{} => {}", self.current_state, state);
        self.current_state = state;
    }
}
