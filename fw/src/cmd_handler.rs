use embedded_hal::serial::Write;
use postcard::{
    flavors::{Cobs, Slice},
    serialize_with_flavor,
};
use serde::{Deserialize, Serialize};

use super::app::UsbSerial;
// use usbd_serial::SerialPort;

#[derive(Serialize, Deserialize)]
enum State {
    Disconnected,
    Negotiating,
    Active,
}

#[derive(Serialize, Deserialize)]
enum Message {
    RequestStatus,
    ReportStatus(State, &'static str),
}

#[derive(Debug)]
pub enum Error {
    Bleh,
}

fn send_message(writer: &mut UsbSerial, message: &Message) -> Result<(), Error> {
    let mut buffer = [0u8; 128];

    let serialized = serialize_with_flavor::<Message, Cobs<Slice>, &mut [u8]>(
        message,
        Cobs::try_new(Slice::new(&mut buffer)).unwrap(),
    )
    .unwrap();

    writer.write(&serialized);
    Ok(())
}

pub fn send_state(writer: &mut UsbSerial) -> Result<(), Error> {
    let message = Message::ReportStatus(State::Disconnected, "Hi there\n");
    send_message(writer, &message)
}
