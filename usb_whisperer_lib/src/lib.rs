#![no_std]

pub mod message {
    use serde::{Deserialize, Serialize};

    #[derive(Serialize, Deserialize, Debug, defmt::Format, Clone, Copy)]
    pub enum State {
        Disconnected,
        Negotiating,
        Attached,
        Busy,
    }

    #[derive(Serialize, Deserialize, Debug, defmt::Format)]
    pub enum Message {
        Reboot,
        ConfigureUart,
        RequestState,
        ReportState(State),
        Ack,
        Nack,
    }

    #[derive(Debug, defmt::Format)]
    pub enum Error {
        WouldBlock,
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        let result = 2 + 2;
        assert_eq!(result, 4);
    }
}
