use serialport::{self, FlowControl, Parity, SerialPort, SerialPortInfo, StopBits};
use std::str::FromStr;
use structopt::StructOpt;

use usb_whisperer_lib::message::Message;

use postcard::{
    flavors::{Cobs, Slice},
    from_bytes_cobs, serialize_with_flavor,
};

#[derive(thiserror::Error, Debug)]
enum Error {
    #[error("Device not found")]
    DeviceNotFound,
}

#[derive(Debug)]
enum UartAutoConfiguration {
    Disabled,
    Enabled,
}

impl FromStr for UartAutoConfiguration {
    type Err = String;
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s {
            "enable" => Ok(UartAutoConfiguration::Enabled),
            "disable" => Ok(UartAutoConfiguration::Disabled),
            "1" => Ok(UartAutoConfiguration::Enabled),
            "0" => Ok(UartAutoConfiguration::Disabled),
            "true" => Ok(UartAutoConfiguration::Enabled),
            "false" => Ok(UartAutoConfiguration::Disabled),
            s => Err(format!("Invalid string {}", s)),
        }
    }
}

impl From<UartAutoConfiguration> for bool {
    fn from(u: UartAutoConfiguration) -> bool {
        match u {
            UartAutoConfiguration::Disabled => false,
            UartAutoConfiguration::Enabled => true,
        }
    }
}

#[derive(Debug, StructOpt)]
#[structopt(
    author,
    about = "Tool to control the M1 macs closed-case debug channel via USB PD"
)]
enum Commands {
    #[structopt(name = "reboot", about = "Reboots the M1 mac via a USB-PD VDM command")]
    Reboot,
    #[structopt(
        name = "config-uart",
        about = "Configures the UART on the M1 mac. Uses Primary D+/D- lines"
    )]
    ConfigureUart,
    #[structopt(
        name = "reboot-with-uart",
        about = "Sets the autoconfiguration flag for the UART and reboots."
    )]
    RebootAndConfigureUart,
    #[structopt(name = "get-state", about = "Gets the device state")]
    GetDeviceState,
    #[structopt(
        name = "config-attach",
        about = "Sets the autoconfiguration flag for the UART"
    )]
    ConfigureAttach {
        #[structopt(short = "u", long = "uart")]
        uart_autoconf: UartAutoConfiguration,
    },
}

fn find_port() -> Option<SerialPortInfo> {
    const EXPECTED_VID: u16 = 0x1209;
    const EXPECTED_PID: u16 = 0x0001;

    let available_ports = serialport::available_ports().ok()?;
    let mut port: Option<SerialPortInfo> = None;

    for p in available_ports {
        match p.port_type {
            serialport::SerialPortType::UsbPort(ref port_info)
                if port_info.vid == EXPECTED_VID && port_info.pid == EXPECTED_PID =>
            {
                port = Some(p);
                break;
            }
            _ => {}
        }
    }

    port
}

fn send_message(port: &mut dyn SerialPort, message: &Message) -> anyhow::Result<Message> {
    let mut buffer = [0; 128];
    let serialized = serialize_with_flavor::<Message, Cobs<Slice>, &mut [u8]>(
        message,
        Cobs::try_new(Slice::new(&mut buffer)).unwrap(),
    )?;

    port.write(serialized)?;

    // Expect ACK
    let size = port.read(&mut buffer)?;
    let message = from_bytes_cobs::<'_, Message>(&mut buffer[..size])?;

    Ok(message)
}

fn main() -> anyhow::Result<()> {
    let commands = Commands::from_args();

    let port_info = find_port().ok_or(Error::DeviceNotFound)?;

    let mut port = serialport::new(port_info.port_name, 115200u32)
        .parity(Parity::None)
        .stop_bits(StopBits::One)
        .flow_control(FlowControl::None)
        .open()?;

    port.set_timeout(std::time::Duration::from_millis(500))?;
    port.write_data_terminal_ready(true)?;

    match commands {
        Commands::Reboot => {
            println!("Rebooting device");
            let message = Message::Reboot;
            match send_message(&mut *port, &message)? {
                Message::Ack => {
                    println!("=> ACK");
                }
                Message::Nack => {
                    println!("=> NACK");
                }
                m => {
                    println!("Unexpected Message: {:?}", m);
                    port.write_data_terminal_ready(false)?;
                    std::process::exit(1);
                }
            }
        }
        Commands::ConfigureUart => {
            println!("Configuring UART");
            let message = Message::ConfigureUart;
            match send_message(&mut *port, &message)? {
                Message::Ack => {
                    println!("=> ACK");
                }
                Message::Nack => {
                    println!("=> NACK");
                }
                m => {
                    println!("Unexpected Message: {:?}", m);
                    port.write_data_terminal_ready(false)?;
                    std::process::exit(1);
                }
            }
        }
        Commands::ConfigureAttach { uart_autoconf } => {
            println!("Requesting uart autoconfiguration");
            let message = Message::SetAutoConfigureUart(uart_autoconf.into());
            match send_message(&mut *port, &message)? {
                Message::Ack => {
                    println!("=> ACK");
                }
                Message::Nack => {
                    println!("=> NACK");
                }
                m => {
                    println!("Unexpected Message: {:?}", m);
                    port.write_data_terminal_ready(false)?;
                    std::process::exit(1);
                }
            }
        }
        Commands::RebootAndConfigureUart => {
            println!("Enabling UART autoconfiguration");
            let message = Message::SetAutoConfigureUart(true);
            match send_message(&mut *port, &message)? {
                Message::Ack => {
                    println!("=> ACK");
                }
                Message::Nack => {
                    println!("=> NACK");
                }
                m => {
                    println!("Unexpected Message: {:?}", m);
                    port.write_data_terminal_ready(false)?;
                    std::process::exit(1);
                }
            }
            println!("Rebooting device");
            let message = Message::Reboot;
            match send_message(&mut *port, &message)? {
                Message::Ack => {
                    println!("=> ACK");
                }
                Message::Nack => {
                    println!("=> NACK");
                }
                m => {
                    println!("Unexpected Message: {:?}", m);
                    port.write_data_terminal_ready(false)?;
                    std::process::exit(1);
                }
            }
        }
        Commands::GetDeviceState => {
            println!("Getting device state");
            let message = Message::RequestState;
            match send_message(&mut *port, &message)? {
                Message::ReportState(state, uart_autoconf) => {
                    println!("=> State: {:?}", state);
                    println!("=> Uart Autoconfiguration: {:?}", uart_autoconf);
                }
                Message::Nack => {
                    println!("=> NACK");
                }
                m => {
                    println!("Unexpected Message: {:?}", m);
                    port.write_data_terminal_ready(false)?;
                    std::process::exit(1);
                }
            }
        }
    }

    port.write_data_terminal_ready(false)?;

    Ok(())
}
