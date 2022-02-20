//! FSM for a USB-C DFP implementation. USB UFP or DRP are not supported supported as of now

use super::fusb302::{
    self, AcceptMessage, ControlMessageType, DataMessageType, Fusb302, MessageType, PortType,
    PsRdyMessage, SopTarget, SourceCapabilitiesMessage,
};

use embedded_hal::{
    blocking::{
        delay::DelayUs,
        i2c::{Write, WriteRead},
    },
    digital::v2::InputPin,
};
#[derive(defmt::Format)]
enum State {
    Disconnected,
    DebounceConnection,
    ApplyVbus,
    SendSourceCapabilities,
    PowerNegotiation,
    ConfigurePS,
    Connected,
}

pub enum Error<T>
where
    T: WriteRead + Write,
{
    DeviceError(fusb302::Error<T>),
}

impl<T: WriteRead + Write> core::fmt::Debug for Error<T> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::DeviceError(inner) => write!(f, "DeviceError({:?})", inner),
        }
    }
}

impl<T: WriteRead + Write> From<fusb302::Error<T>> for Error<T> {
    fn from(error: fusb302::Error<T>) -> Self {
        Self::DeviceError(error)
    }
}

pub struct UsbFsm<T, U>
where
    T: WriteRead + Write,
    U: InputPin,
{
    pd_controller: Fusb302<T>,
    irq_pin: U,
    current_state: State,
}

impl<T, U> UsbFsm<T, U>
where
    T: WriteRead + Write,
    U: InputPin,
{
    pub fn new(mut pd_controller: Fusb302<T>, irq_pin: U) -> Result<Self, Error<T>> {
        pd_controller.init()?;
        pd_controller.reset_pd()?;
        pd_controller.configure_port_type(PortType::Open)?;
        pd_controller.enable_interrupts()?;
        pd_controller.configure_tx_sent_irq(true)?;
        pd_controller.configure_rx_recv_irq(true)?;

        Ok(Self {
            pd_controller,
            irq_pin,
            current_state: State::Disconnected,
        })
    }

    pub fn run<V: DelayUs<u32>>(&mut self, timer: &mut V) -> Result<(), Error<T>> {
        // No interrupts should be received yet, since they are off
        let received_messages = if self.irq_pin.is_low().unwrap_or(false) {
            self.pd_controller.handle_irq()?
        } else {
            false
        };

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
                        self.trigger_transition(State::ApplyVbus);
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
            State::ApplyVbus => {
                self.pd_controller.configure_port_type(PortType::DFP)?;
                // TODO(javier-varez): Enable Vbus here. It's unclear how to do this with the
                // current design
                self.trigger_transition(State::SendSourceCapabilities);
            }
            State::SendSourceCapabilities => {
                if received_messages {
                    while !self.pd_controller.is_rx_fifo_empty()? {
                        let (target, header, _objects) = self.pd_controller.receive_message()?;
                        match (target, header.message_type()?) {
                            (
                                SopTarget::SOP,
                                MessageType::DataMessage(DataMessageType::Request, 1),
                            ) => {
                                // TODO(javier-varez): Wait for Ack before transitioning
                                self.trigger_transition(State::PowerNegotiation);
                            }
                            (_, MessageType::ControlMessage(ControlMessageType::GoodCrc)) => {
                                // Ignore good crc
                            }
                            _ => {
                                // Received unexpected message
                                defmt::warn!(
                                    "Received unexpected message in SendSourceCapabilities"
                                );
                            }
                        }
                    }
                }

                let caps = SourceCapabilitiesMessage::new_no_capabilities();
                match self.pd_controller.send_message(SopTarget::SOP, &caps) {
                    Err(fusb302::Error::TxInProgress) => {
                        // Silently ignore these
                    }
                    Ok(()) => {}
                    Err(error) => return Err(Error::DeviceError(error)),
                };
            }
            State::PowerNegotiation => {
                let accept = AcceptMessage::new();
                match self.pd_controller.send_message(SopTarget::SOP, &accept) {
                    Err(fusb302::Error::TxInProgress) => {
                        // Silently ignore these
                    }
                    Ok(()) => {
                        self.trigger_transition(State::ConfigurePS);
                    }
                    Err(error) => return Err(Error::DeviceError(error)),
                };
            }
            State::ConfigurePS => {
                let ps_rdy = PsRdyMessage::new();
                match self.pd_controller.send_message(SopTarget::SOP, &ps_rdy) {
                    Err(fusb302::Error::TxInProgress) => {
                        // Silently ignore these
                    }
                    Ok(()) => {
                        self.trigger_transition(State::Connected);
                    }
                    Err(error) => return Err(Error::DeviceError(error)),
                };
            }
            State::Connected => {
                if !self.pd_controller.monitor_connection(timer)? {
                    defmt::info!("Device disconnected");
                    self.pd_controller.configure_port_type(PortType::Open)?;
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
