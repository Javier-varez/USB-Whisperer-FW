#![no_std]

pub mod message {
    use heapless::Vec;
    use serde::{Deserialize, Serialize};

    pub const MAX_VDM_ARGS: usize = 6;

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
        ReportState(State, bool),
        SetAutoConfigureUart(bool),
        Ack,
        Nack,
        RawVdm(u16, u16, Vec<u32, MAX_VDM_ARGS>),
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
