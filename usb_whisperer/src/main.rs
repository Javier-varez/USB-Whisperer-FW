use serialport::{self, FlowControl, Parity, SerialPort, SerialPortInfo, StopBits};
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

#[derive(Debug, StructOpt)]
enum Commands {
    Reboot,
    ConfigureUart,
    GetDeviceState,
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
        Commands::GetDeviceState => {
            println!("Getting device state");
            let message = Message::RequestState;
            match send_message(&mut *port, &message)? {
                Message::ReportState(state) => {
                    println!("=> State {:?}", state);
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
