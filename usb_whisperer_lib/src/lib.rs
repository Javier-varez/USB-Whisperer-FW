#![no_std]

pub mod message {
    use serde::{Deserialize, Serialize};

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
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        let result = 2 + 2;
        assert_eq!(result, 4);
    }
}
