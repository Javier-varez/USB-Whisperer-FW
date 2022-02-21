//! FSM for a USB-C DFP implementation. USB UFP or DRP are not supported supported as of now

use super::fusb302::{
    self, AcceptMessage, ControlMessageType, DataMessageType, Fusb302, InterruptStatus, Message,
    MessageType, PortType, PsRdyMessage, SopTarget, SourceCapabilitiesMessage,
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
    WaitVbus,
    SendSourceCapabilities,
    WaitForRequest,
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
    interval_time: system_timer::Instant,
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
            interval_time: system_timer::get_ms(),
        };

        fsm.reinit()?;

        Ok(fsm)
    }

    fn send_with_interval(
        &mut self,
        sop_target: SopTarget,
        message: &dyn Message,
        interval_ms: u32,
    ) -> Result<bool, Error<T>> {
        if system_timer::elapsed_since(self.interval_time).into_ms() > interval_ms {
            self.interval_time = system_timer::get_ms();
            self.pd_controller.send_message(sop_target, message)?;
            Ok(true)
        } else {
            Ok(false)
        }
    }

    pub fn run<V: DelayUs<u32>>(&mut self, timer: &mut V) -> Result<(), Error<T>> {
        match self.run_inner(timer) {
            Err(Error::DeviceError(fusb302::Error::HardResetRequested)) => {
                defmt::error!("Hard reset requested");
                self.trigger_transition(State::RecoverError);
                Ok(())
            }
            Err(Error::DeviceError(fusb302::Error::SoftResetRequested)) => {
                defmt::error!("Soft reset requested");
                self.trigger_transition(State::RecoverError);
                Ok(())
            }
            Err(Error::DeviceError(fusb302::Error::TxRetryFailed)) => {
                defmt::error!("Max retries reached");
                self.trigger_transition(State::RecoverError);
                Ok(())
            }
            Err(Error::DeviceError(fusb302::Error::IOReadError(_)))
            | Err(Error::DeviceError(fusb302::Error::IOWriteError(_))) => {
                defmt::error!("IO Error, trying to recover");
                self.trigger_transition(State::RecoverError);
                Ok(())
            }
            val => val,
        }
    }

    fn run_inner<V: DelayUs<u32>>(&mut self, timer: &mut V) -> Result<(), Error<T>> {
        // No interrupts should be received yet, since they are off
        let InterruptStatus {
            received_messages,
            vbus_ok,
        } = if self.irq_pin.is_low().unwrap_or(false) {
            self.pd_controller.handle_irq()?
        } else {
            InterruptStatus::default()
        };

        if vbus_ok {
            if self.pd_controller.get_vbus_state()? {
                defmt::dbg!("VBUS high");
            } else {
                defmt::dbg!("VBUS low");
                if matches!(
                    self.current_state,
                    State::SendSourceCapabilities
                        | State::WaitForRequest
                        | State::PowerNegotiation
                        | State::ConfigurePS
                        | State::Connected
                ) {
                    self.vbus_pin.set_low().unwrap_or_default();
                    self.pd_controller.configure_port_type(PortType::Open)?;
                    self.trigger_transition(State::Disconnected);
                }
            }
        }

        match self.current_state {
            State::Disconnected => {
                match self.pd_controller.detect_cc_orientation(timer)? {
                    Some(orientation) => {
                        defmt::dbg!("Detected line {}", orientation);
                        self.trigger_transition(State::DebounceConnection);
                    }
                    None => {}
                };
            }
            State::DebounceConnection => {
                if self.elapsed_since_entry().into_ms() > 100 {
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
            }
            State::ApplyVbus => {
                self.trigger_transition(State::WaitVbus);

                self.pd_controller.reset_pd()?;
                self.pd_controller.configure_port_type(PortType::DFP)?;
                self.vbus_pin.set_high().unwrap_or_default();
            }
            State::WaitVbus => {
                if vbus_ok {
                    self.trigger_transition(State::SendSourceCapabilities);
                }

                if self.elapsed_since_entry().into_ms() > 100 {
                    defmt::warn!("Timeout waiting for vbus to be enabled");
                    self.trigger_transition(State::RecoverError);
                }
            }
            State::SendSourceCapabilities => {
                let mut caps = SourceCapabilitiesMessage::new_no_capabilities();
                caps.set_fixed_current(0);

                match self.send_with_interval(SopTarget::SOP, &caps, 150) {
                    Ok(true) => {
                        self.trigger_transition(State::WaitForRequest);
                        Ok(())
                    }
                    Ok(false) => Ok(()),
                    Err(Error::DeviceError(fusb302::Error::TxAlreadyInProgress)) => Ok(()),
                    Err(err) => Err(err),
                }?;

                if self.elapsed_since_entry().into_ms() > 2000 {
                    defmt::warn!("Could not send capabilities, giving up");
                    self.trigger_transition(State::RecoverError);
                }
            }
            State::WaitForRequest => {
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

                // Timeout after 2 s
                if self.elapsed_since_entry().into_ms() > 2000 {
                    defmt::warn!("No request, giving up");
                    self.trigger_transition(State::RecoverError);
                }
            }
            State::PowerNegotiation => {
                let accept = AcceptMessage::new();
                match self.send_with_interval(SopTarget::SOP, &accept, 5) {
                    Ok(true) => {
                        self.trigger_transition(State::ConfigurePS);
                        Ok(())
                    }
                    Ok(false) => Ok(()),
                    Err(Error::DeviceError(fusb302::Error::TxAlreadyInProgress)) => Ok(()),
                    Err(err) => Err(err),
                }?;

                // Timeout after 2 s
                if self.elapsed_since_entry().into_ms() > 2000 {
                    defmt::warn!("Accept message not sent, giving up");
                    self.trigger_transition(State::RecoverError);
                }
            }
            State::ConfigurePS => {
                let ps_rdy = PsRdyMessage::new();
                match self.send_with_interval(SopTarget::SOP, &ps_rdy, 5) {
                    Ok(true) => {
                        self.trigger_transition(State::Connected);
                        Ok(())
                    }
                    Ok(false) => Ok(()),
                    Err(Error::DeviceError(fusb302::Error::TxAlreadyInProgress)) => Ok(()),
                    Err(err) => Err(err),
                }?;

                // Timeout after 2 s
                if self.elapsed_since_entry().into_ms() > 2000 {
                    defmt::warn!("PS Rdy not sent, giving up");
                    self.trigger_transition(State::RecoverError);
                }
            }
            State::Connected => {
                if !self.pd_controller.monitor_connection(timer)? {
                    defmt::info!("Device disconnected");
                    self.vbus_pin.set_low().unwrap_or_default();
                    self.pd_controller.configure_port_type(PortType::Open)?;
                    self.trigger_transition(State::WaitRecovery);
                }

                // Send reboot order as test
                let vdm_message =
                    fusb302::StructuredVdmMessage::new(0x5ac, 0x12, &[0x0105, 0x80000000]);
                self.send_with_interval(SopTarget::SOP2DB, &vdm_message, 2000)?;

                if received_messages {
                    while !self.pd_controller.is_rx_fifo_empty()? {
                        let (_target, _header, _objects) = self.pd_controller.receive_message()?;
                    }
                }
            }
            State::RecoverError => {
                // self.pd_controller.print_status_regs()?;
                self.vbus_pin.set_low().unwrap_or_default();

                self.pd_controller.send_hard_reset()?;
                self.reinit()?;

                system_timer::blocking_wait_ms(10);

                self.trigger_transition(State::WaitRecovery);
            }
            State::WaitRecovery => {
                // Go back to disconnected after 100 ms
                if self.elapsed_since_entry().into_ms() > 100 {
                    self.trigger_transition(State::Disconnected);
                }
            }
        }

        // If any messages left, drain them here
        if received_messages {
            while !self.pd_controller.is_rx_fifo_empty()? {
                let (target, header, _objects) = self.pd_controller.receive_message()?;
                defmt::warn!("Received unexpected message {}, {}", target, header);
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
        self.pd_controller.configure_vbus_irq(true)?;
        Ok(())
    }

    fn trigger_transition(&mut self, state: State) {
        defmt::info!("{} => {}", self.current_state, state);
        self.current_state = state;
        self.entry_time = system_timer::get_ms();
        self.interval_time = self.entry_time;
    }

    fn elapsed_since_entry(&self) -> system_timer::Duration {
        system_timer::elapsed_since(self.entry_time)
    }
}
