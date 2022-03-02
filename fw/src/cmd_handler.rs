use postcard::{
    flavors::{Cobs, Slice},
    from_bytes_cobs, serialize_with_flavor,
};

use usb_whisperer_lib::message::{Error, Message};

pub struct CommandHandler {
    buffer: [u8; 128],
    cursor: usize,
}

impl CommandHandler {
    pub fn new() -> Self {
        Self {
            buffer: [0; 128],
            cursor: 0,
        }
    }

    pub fn serialize_message<'a>(
        &self,
        message: &Message,
        buffer: &'a mut [u8],
    ) -> Result<&'a mut [u8], Error> {
        let serialized = serialize_with_flavor::<Message, Cobs<Slice>, &mut [u8]>(
            message,
            Cobs::try_new(Slice::new(buffer)).unwrap(),
        )
        .unwrap();

        Ok(serialized)
    }

    pub fn deserialize_message(&mut self, buffer: &mut [u8]) -> Result<Message, Error> {
        let mut last_index = buffer.len();
        let mut has_message = false;

        for (index, byte) in buffer.iter().enumerate() {
            self.buffer[self.cursor] = *byte;
            self.cursor += 1;

            if *byte == 0 {
                // Store the message and keep the rest of the data
                has_message = true;
                last_index = index;
                break;
            }
        }

        if has_message {
            let message = from_bytes_cobs::<'_, Message>(&mut self.buffer[..self.cursor]).unwrap();
            self.cursor = 0;

            // Store the rest of the data in the buffer
            if last_index != buffer.len() - 1 {
                let buffer = &buffer[last_index + 1..];
                for byte in buffer {
                    self.buffer[self.cursor] = *byte;
                    self.cursor += 1;

                    if *byte == 0 {
                        panic!("More than one message in the same packet! This is not supported!");
                    }
                }
            }

            return Ok(message);
        }

        Err(Error::WouldBlock)
    }
}
