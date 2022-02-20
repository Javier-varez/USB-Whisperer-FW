//! FSM for a USB-C DFP implementation. USB UFP or DRP are not supported supported as of now

use super::fusb302::{
    self, AcceptMessage, ControlMessageType, DataMessageType, Fusb302, MessageType, PortType,
    PsRdyMessage, SopTarget, SourceCapabilitiesMessage,
};

use crate::system_timer;

use embedded_hal::{
    blocking::{
        delay::DelayUs,
        i2c::{Write, WriteRead},
    },
    digital::v2::{InputPin, OutputPin},
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
    RecoverError,
    WaitRecovery,
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

pub struct UsbFsm<T, U, X>
where
    T: WriteRead + Write,
    U: InputPin,
    X: OutputPin,
{
    pd_controller: Fusb302<T>,
    irq_pin: U,
    vbus_pin: X,
    current_state: State,
    entry_time: system_timer::Instant,
}

impl<T, U, X> UsbFsm<T, U, X>
where
    T: WriteRead + Write,
    U: InputPin,
    X: OutputPin,
{
    pub fn new(pd_controller: Fusb302<T>, irq_pin: U, vbus_pin: X) -> Result<Self, Error<T>> {
        let mut fsm = Self {
            pd_controller,
            irq_pin,
            vbus_pin,
            current_state: State::Disconnected,
            entry_time: system_timer::get_ms(),
        };

        fsm.reinit()?;

        Ok(fsm)
    }

    pub fn run<V: DelayUs<u32>>(&mut self, timer: &mut V) -> Result<(), Error<T>> {
        // No interrupts should be received yet, since they are off
        let received_messages = if self.irq_pin.is_low().unwrap_or(false) {
            match self.pd_controller.handle_irq() {
                Ok(recv_messages) => recv_messages,
                Err(fusb302::Error::ErrorState) => {
                    self.trigger_transition(State::RecoverError);
                    return Ok(());
                }
                Err(err) => return Err(Error::DeviceError(err)),
            }
        } else {
            false
        };

        match self.current_state {
            State::Disconnected => {
                match self.pd_controller.detect_cc_orientation(timer)? {
                    Some(orientation) => {
                        defmt::info!("Detected line {}", orientation);
                        self.trigger_transition(State::DebounceConnection);
                    }
                    None => {}
                };
            }
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
                self.vbus_pin.set_high().unwrap_or_default();
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

                // Start sending source capabilities after 300 ms
                if self.elapsed_since_entry().into_ms() > 300 {
                    let caps = SourceCapabilitiesMessage::new_no_capabilities();
                    match self.pd_controller.send_message(SopTarget::SOP, &caps) {
                        Err(fusb302::Error::TxInProgress) => {
                            // Silently ignore these
                        }
                        Ok(()) => {}
                        Err(error) => return Err(Error::DeviceError(error)),
                    };
                }

                // Timeout after 2 s
                if self.elapsed_since_entry().into_ms() > 2000 {
                    defmt::warn!("No response to capabilities request, giving up");
                    self.trigger_transition(State::RecoverError);
                }
            }
            State::PowerNegotiation => {
                // Start sending accept msg after 10 ms
                if self.elapsed_since_entry().into_ms() > 10 {
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
            }
            State::ConfigurePS => {
                // Start sending ps ready after 10 ms
                if self.elapsed_since_entry().into_ms() > 10 {
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
            }
            State::Connected => {
                if !self.pd_controller.monitor_connection(timer)? {
                    defmt::info!("Device disconnected");
                    self.vbus_pin.set_low().unwrap_or_default();
                    self.pd_controller.configure_port_type(PortType::Open)?;
                    self.trigger_transition(State::Disconnected);
                }
            }
            State::RecoverError => {
                self.pd_controller.print_status_regs()?;
                self.vbus_pin.set_low().unwrap_or_default();

                self.pd_controller.send_hard_reset()?;
                self.reinit()?;

                system_timer::blocking_wait_ms(10);

                self.trigger_transition(State::WaitRecovery);
            }
            State::WaitRecovery => {
                // Go back to disconnected after 100 ms
                if self.elapsed_since_entry().into_ms() > 500 {
                    self.trigger_transition(State::Disconnected);
                }
            }
        }

        Ok(())
    }

    fn reinit(&mut self) -> Result<(), Error<T>> {
        self.pd_controller.init()?;
        self.pd_controller.reset_pd()?;
        system_timer::blocking_wait_ms(10);
        self.pd_controller.configure_port_type(PortType::Open)?;
        self.pd_controller.enable_interrupts()?;
        self.pd_controller.configure_tx_sent_irq(true)?;
        self.pd_controller.configure_rx_recv_irq(true)?;
        self.pd_controller.configure_retry_fail_irq(true)?;
        Ok(())
    }

    fn trigger_transition(&mut self, state: State) {
        defmt::info!("{} => {}", self.current_state, state);
        self.current_state = state;
        self.entry_time = system_timer::get_ms();
    }

    fn elapsed_since_entry(&self) -> system_timer::Duration {
        system_timer::elapsed_since(self.entry_time)
    }
}
