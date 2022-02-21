const DEVICE_SLAVE_ADDR: u8 = 0x22;
const PD_RETRIES_COUNT: u8 = 3;

mod registers;

use embedded_hal::blocking::i2c::{Write, WriteRead};

use heapless::Vec;

pub enum Error<T: WriteRead + Write> {
    DeviceNotFound,
    IOReadError(<T as WriteRead>::Error),
    IOWriteError(<T as Write>::Error),
    UnknownPolarity,
    InvalidHeader,
    TxRetryFailed,
    SoftResetRequested,
    HardResetRequested,
}

impl<T: WriteRead + Write> core::fmt::Debug for Error<T> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::IOReadError(_) => write!(f, "IO Error (Read)"),
            Error::IOWriteError(_) => write!(f, "IO Error (Write)"),
            Error::DeviceNotFound => write!(f, "DeviceNotFound"),
            Error::UnknownPolarity => write!(f, "UnknownPolarity"),
            Error::InvalidHeader => write!(f, "InvalidHeader"),
            Error::TxRetryFailed => write!(f, "TxRetryFailed"),
            Error::SoftResetRequested => write!(f, "SoftResetRequested"),
            Error::HardResetRequested => write!(f, "HardResetRequested"),
        }
    }
}

#[derive(Default)]
pub struct InterruptStatus {
    pub vbus_ok: bool,
    pub received_messages: bool,
}

#[derive(defmt::Format, Debug, Clone, Copy, PartialEq)]
pub enum CcLine {
    Cc1,
    Cc2,
}

#[derive(defmt::Format, Debug, Clone, Copy, PartialEq)]
pub enum PortType {
    Open,
    DFP,
    _UFP,
}

pub struct Fusb302<T: WriteRead + Write> {
    i2c_bus: T,
    polarity: Option<CcLine>,
    port_type: PortType,
    current_msg_id: u8,
}

impl<T: WriteRead + Write> Fusb302<T> {
    pub fn new(i2c_bus: T) -> Result<Self, Error<T>> {
        // Try to read device ID and verify it is connected, otherwise return an error

        let mut instance = Self {
            i2c_bus,
            polarity: None,
            port_type: PortType::Open,
            current_msg_id: 7,
        };
        let id = instance.read_register(registers::device_id::ADDR)?;

        // Check if device ID is valid
        if id & 0x80 == 0 {
            return Err(Error::DeviceNotFound);
        }

        defmt::info!("Found Fusb302 with ID {}", id);

        Ok(instance)
    }

    pub fn init(&mut self) -> Result<(), Error<T>> {
        // Trigger a SW reset
        self.write_register(registers::reset::ADDR, registers::reset::SW_RESET)?;
        self.current_msg_id = 7;

        // Set auto retry and the num_retries to 3
        self.modify_register(registers::control3::ADDR, |value| {
            (value & !registers::control3::N_RETRIES_MASK)
                | (PD_RETRIES_COUNT << registers::control3::N_RETRIES_OFFSET)
                | registers::control3::AUTO_RETRY
        })?;

        self.modify_register(registers::control0::ADDR, |reg| {
            let off = reg & !registers::control0::HOST_CUR_MASK;
            off | registers::control0::HOST_CUR_DEF
        })?;

        self.write_register(
            registers::control1::ADDR,
            registers::control1::ENSOP1DB
                | registers::control1::ENSOP2DB
                | registers::control1::RX_FLUSH,
        )?;

        self.modify_register(registers::switches1::ADDR, |reg| {
            reg | registers::switches1::POWERROLE | registers::switches1::DATAROLE
        })?;

        self.auto_goodcrc_enable(false)?;

        self.mask_all_interrutps()?;

        // Unmask soft reset and hard reset
        self.modify_register(registers::maska::ADDR, |reg| {
            reg & !(registers::maska::SOFTRST | registers::maska::HARDRST)
        })?;

        // Enable power
        self.write_register(registers::power::ADDR, registers::power::PWR_ALL_MASK)
    }

    pub fn flush_fifo(&mut self) -> Result<(), Error<T>> {
        self.modify_register(registers::control1::ADDR, |reg| {
            reg | registers::control1::RX_FLUSH
        })?;

        self.modify_register(registers::control0::ADDR, |reg| {
            reg | registers::control0::TX_FLUSH
        })
    }

    pub fn print_status_regs(&mut self) -> Result<(), Error<T>> {
        let status0 = self.read_register(registers::status0::ADDR)?;
        let status1 = self.read_register(registers::status1::ADDR)?;
        let status0a = self.read_register(registers::status0a::ADDR)?;
        let status1a = self.read_register(registers::status1a::ADDR)?;
        let interrupt = self.read_register(registers::interrupt::ADDR)?;
        let interrupta = self.read_register(registers::interrupta::ADDR)?;
        let interruptb = self.read_register(registers::interruptb::ADDR)?;

        defmt::error!("status0 = 0x{:x}", status0);
        defmt::error!("status1 = 0x{:x}", status1);
        defmt::error!("status0a = 0x{:x}", status0a);
        defmt::error!("status1a = 0x{:x}", status1a);
        defmt::error!("interrupt = 0x{:x}", interrupt);
        defmt::error!("interrupta = 0x{:x}", interrupta);
        defmt::error!("interruptb = 0x{:x}", interruptb);
        Ok(())
    }

    fn mask_all_interrutps(&mut self) -> Result<(), Error<T>> {
        self.write_register(registers::mask::ADDR, 0xff)?;
        self.write_register(registers::maska::ADDR, 0xff)?;
        self.write_register(registers::maskb::ADDR, 0xff)
    }

    pub fn reset_pd(&mut self) -> Result<(), Error<T>> {
        self.current_msg_id = 7;
        self.write_register(registers::reset::ADDR, registers::reset::PD_RESET)
    }

    pub fn configure_port_type(&mut self, port_type: PortType) -> Result<(), Error<T>> {
        self.modify_register(registers::control2::ADDR, |reg| {
            reg & !registers::control2::TOGGLE
        })?;

        let mut polarity = CcLine::Cc1;
        if port_type != PortType::Open {
            polarity = self.polarity.ok_or(Error::UnknownPolarity)?;
        }

        self.modify_register(registers::switches0::ADDR, |reg| {
            let open = reg
                & !(registers::switches0::CC2_PU_EN
                    | registers::switches0::CC1_PU_EN
                    | registers::switches0::CC2_VCONN_EN
                    | registers::switches0::CC1_VCONN_EN
                    | registers::switches0::CC2_PD_EN
                    | registers::switches0::CC1_PD_EN);

            match (polarity, port_type) {
                (CcLine::Cc1, PortType::DFP) => {
                    open | registers::switches0::CC1_PU_EN | registers::switches0::CC2_PU_EN
                }
                (CcLine::Cc2, PortType::DFP) => {
                    open | registers::switches0::CC2_PU_EN | registers::switches0::CC1_PU_EN
                }
                (_, PortType::_UFP) => {
                    open | registers::switches0::CC1_PD_EN | registers::switches0::CC2_PD_EN
                }
                (_, PortType::Open) => {
                    open | registers::switches0::CC1_PU_EN | registers::switches0::CC2_PU_EN
                }
            }
        })?;

        self.configure_cc_tx_driver(port_type, polarity)?;

        match port_type {
            PortType::DFP => {
                self.configure_disconnect_monitoring()?;
                self.auto_goodcrc_enable(true)?;
            }
            PortType::_UFP => {
                unimplemented!();
            }
            PortType::Open => {
                self.auto_goodcrc_enable(false)?;
                self.configure_disconnect_monitoring()?;
            }
        }

        self.port_type = port_type;

        Ok(())
    }

    fn configure_cc_tx_driver(
        &mut self,
        port_type: PortType,
        polarity: CcLine,
    ) -> Result<(), Error<T>> {
        self.modify_register(registers::switches1::ADDR, |reg| {
            let open = reg & !(registers::switches1::TXCC1_EN | registers::switches1::TXCC2_EN);

            match (polarity, port_type) {
                (_, PortType::Open) => open,
                (CcLine::Cc1, _) => open | registers::switches1::TXCC1_EN,
                (CcLine::Cc2, _) => open | registers::switches1::TXCC2_EN,
            }
        })
    }

    fn configure_disconnect_monitoring(&mut self) -> Result<(), Error<T>> {
        let polarity = self.polarity;

        self.modify_register(registers::switches0::ADDR, |reg| {
            let off = reg & !(registers::switches0::MEAS_CC1 | registers::switches0::MEAS_CC2);
            match polarity {
                Some(CcLine::Cc1) => off | registers::switches0::MEAS_CC1,
                Some(CcLine::Cc2) => off | registers::switches0::MEAS_CC2,
                None => off,
            }
        })
    }

    pub fn get_vbus_state(&mut self) -> Result<bool, Error<T>> {
        let reg = self.read_register(registers::status0::ADDR)?;
        Ok((reg & registers::status0::VBUS_OK) != 0)
    }

    pub fn detect_cc_orientation<U>(&mut self, timer: &mut U) -> Result<Option<CcLine>, Error<T>>
    where
        U: embedded_hal::blocking::delay::DelayUs<u32>,
    {
        let cc1 = self.detect_rd_sink(CcLine::Cc1, timer)?;
        timer.delay_us(100);
        let cc2 = self.detect_rd_sink(CcLine::Cc2, timer)?;

        if cc1 && cc2 {
            defmt::warn!("Both CC lines are low!");
            self.polarity = None
        } else if cc1 {
            self.polarity = Some(CcLine::Cc1);
        } else if cc2 {
            self.polarity = Some(CcLine::Cc2);
        } else {
            self.polarity = None
        }

        Ok(self.polarity)
    }

    pub fn get_polarity(&self) -> Option<CcLine> {
        self.polarity
    }

    fn detect_rd_sink<U>(&mut self, line: CcLine, timer: &mut U) -> Result<bool, Error<T>>
    where
        U: embedded_hal::blocking::delay::DelayUs<u32>,
    {
        let saved_switches0 = self.read_register(registers::switches0::ADDR)?;
        self.modify_register(registers::switches0::ADDR, |reg| {
            let open = reg
                & !(registers::switches0::MEAS_CC1
                    | registers::switches0::MEAS_CC2
                    | registers::switches0::CC1_PU_EN
                    | registers::switches0::CC2_PU_EN
                    | registers::switches0::CC1_PD_EN
                    | registers::switches0::CC2_PD_EN);

            match line {
                CcLine::Cc1 => {
                    open | registers::switches0::MEAS_CC1 | registers::switches0::CC1_PU_EN
                }
                CcLine::Cc2 => {
                    open | registers::switches0::MEAS_CC2 | registers::switches0::CC2_PU_EN
                }
            }
        })?;

        let detected = self.monitor_connection(timer)?;

        self.write_register(registers::switches0::ADDR, saved_switches0)?;
        Ok(detected)
    }

    pub fn monitor_connection<U>(&mut self, timer: &mut U) -> Result<bool, Error<T>>
    where
        U: embedded_hal::blocking::delay::DelayUs<u32>,
    {
        // TODO(javier-varez): Only USB default current is supported (80 uA current source for
        // pull-up) Seet table 3 (Host interrupt summary) in the datasheet
        const THRESHOLD_MV: u32 = 1600;

        self.write_register(
            registers::measure::ADDR,
            registers::measure::mdac_threshold_from_mv(THRESHOLD_MV),
        )?;

        // Apply some blocking delay to stabilize the line
        timer.delay_us(250);

        if self.read_comp_status()? {
            // No connection detected
            return Ok(false);
        }

        // If some low voltage is detected it could be either due to Rd (sink) or Ra (cable). We
        // need to reconfigure the mdac to sense the difference between the two

        const RA_THRESHOLD_MV: u32 = 150;

        self.write_register(
            registers::measure::ADDR,
            registers::measure::mdac_threshold_from_mv(RA_THRESHOLD_MV),
        )?;

        // Apply some blocking delay to stabilize the line
        timer.delay_us(250);

        if self.read_comp_status()? {
            // Rd detected!
            return Ok(true);
        }

        defmt::info!("Ra detected, is your cable an active cable?");
        Ok(false)
    }

    fn read_comp_status(&mut self) -> Result<bool, Error<T>> {
        self.read_register(registers::status0::ADDR)
            .map(|val| (val & registers::status0::COMP) != 0)
    }

    fn auto_goodcrc_enable(&mut self, enable: bool) -> Result<(), Error<T>> {
        self.modify_register(registers::switches1::ADDR, |reg| {
            // Clear the spec rev. defaults are wrong
            let reg = reg & !registers::switches1::SPECREV_MASK;
            if enable {
                reg | registers::switches1::AUTO_CRC
            } else {
                reg & !registers::switches1::AUTO_CRC
            }
        })
    }

    pub fn enable_interrupts(&mut self) -> Result<(), Error<T>> {
        self.modify_register(registers::control0::ADDR, |reg| {
            reg & !registers::control0::INT_MASK
        })
    }

    pub fn _disable_interrupts(&mut self) -> Result<(), Error<T>> {
        self.modify_register(registers::control0::ADDR, |reg| {
            reg | registers::control0::INT_MASK
        })
    }

    fn read_register(&mut self, address: u8) -> Result<u8, Error<T>> {
        let addr_buffer = [address];

        let mut register_value = [0];
        self.i2c_bus
            .write_read(DEVICE_SLAVE_ADDR, &addr_buffer, &mut register_value)
            .map_err(|err| Error::IOReadError(err))?;

        Ok(register_value[0])
    }

    fn modify_register<U>(&mut self, address: u8, closure: U) -> Result<(), Error<T>>
    where
        U: Fn(u8) -> u8,
    {
        let value = self.read_register(address)?;
        let value = closure(value);
        self.write_register(address, value)
    }

    fn write_register(&mut self, address: u8, value: u8) -> Result<(), Error<T>> {
        let addr_buffer = [address, value];
        self.i2c_bus
            .write(DEVICE_SLAVE_ADDR, &addr_buffer)
            .map_err(|err| Error::IOWriteError(err))?;

        Ok(())
    }

    pub fn configure_vbus_irq(&mut self, enable: bool) -> Result<(), Error<T>> {
        self.modify_register(registers::mask::ADDR, |reg| {
            if enable {
                reg & !(registers::mask::VBUS_OK)
            } else {
                reg | registers::mask::VBUS_OK
            }
        })
    }

    pub fn configure_tx_sent_irq(&mut self, enable: bool) -> Result<(), Error<T>> {
        self.modify_register(registers::maska::ADDR, |reg| {
            if enable {
                reg & !(registers::maska::TX_SENT)
            } else {
                reg | registers::maska::TX_SENT
            }
        })
    }

    pub fn configure_rx_recv_irq(&mut self, enable: bool) -> Result<(), Error<T>> {
        self.modify_register(registers::maskb::ADDR, |reg| {
            if enable {
                reg & !(registers::maskb::GCRCSENT)
            } else {
                reg | registers::maskb::GCRCSENT
            }
        })
    }

    pub fn configure_retry_fail_irq(&mut self, enable: bool) -> Result<(), Error<T>> {
        self.modify_register(registers::maska::ADDR, |reg| {
            if enable {
                reg & !(registers::maska::RETRYFAIL)
            } else {
                reg | registers::maska::RETRYFAIL
            }
        })
    }

    pub fn handle_irq(&mut self) -> Result<InterruptStatus, Error<T>> {
        let mut received_messages = false;
        let mut vbus_ok = false;

        let irq = self.read_register(registers::interrupt::ADDR)?;
        let irqa = self.read_register(registers::interrupta::ADDR)?;
        let irqb = self.read_register(registers::interruptb::ADDR)?;
        defmt::dbg!("irq 0x{:x}", irq);
        defmt::dbg!("irqa 0x{:x}", irqa);
        defmt::dbg!("irqb 0x{:x}", irqb);

        if (irqa & registers::interrupta::TX_SENT) != 0 {
            defmt::info!("TX Sent IRQ received");
        }

        if (irqb & registers::interruptb::GCRCSENT) != 0 {
            defmt::info!("Good crc sent, message has been received");
            received_messages = true;
        }

        if (irqa & registers::interrupta::HARDRST) != 0 {
            return Err(Error::HardResetRequested);
        }

        if (irqa & registers::interrupta::SOFTRST) != 0 {
            return Err(Error::SoftResetRequested);
        }

        if (irq & registers::interrupt::VBUS_OK) != 0 {
            defmt::info!("VBUS ok irq");
            vbus_ok = true;
        }

        Ok(InterruptStatus {
            vbus_ok,
            received_messages,
        })
    }

    pub fn send_hard_reset(&mut self) -> Result<(), Error<T>> {
        self.modify_register(registers::control3::ADDR, |reg| {
            reg | registers::control3::SEND_HARD_RESET
        })
    }
    pub fn send_message(
        &mut self,
        sop_target: SopTarget,
        message: &dyn Message,
    ) -> Result<(), Error<T>> {
        const MAX_MSG_SIZE: usize = 42;

        let mut header = message.get_header();
        let objects = message.get_data();

        self.current_msg_id += 1;
        if self.current_msg_id > 7 {
            self.current_msg_id = 0;
        }
        header.set_id(self.current_msg_id);

        let mut buffer: Vec<u8, MAX_MSG_SIZE> = Vec::new();
        buffer.push(registers::fifo::ADDR).unwrap();

        // Send sop target tokens
        for token in sop_target.into_tokens() {
            buffer.push(token).unwrap();
        }

        // At least 2 for the header + num_objects * 4
        let num_bytes =
            (core::mem::size_of::<u16>() + objects.len() * core::mem::size_of::<u32>()) as u8;
        assert!(num_bytes < 30);

        // Send data packets
        buffer
            .push(registers::fifo::tx_tokens::PACKSYM | num_bytes)
            .unwrap();

        // Send header
        let header = header.into_inner();
        buffer.push((header & 0xff) as u8).unwrap();
        buffer.push((header >> 8) as u8).unwrap();

        // Push data
        for obj in objects {
            buffer.push((obj & 0xFF) as u8).unwrap();
            buffer.push(((obj >> 8) & 0xFF) as u8).unwrap();
            buffer.push(((obj >> 16) & 0xFF) as u8).unwrap();
            buffer.push(((obj >> 24) & 0xFF) as u8).unwrap();
        }

        buffer.push(registers::fifo::tx_tokens::JAM_CRC).unwrap();
        buffer.push(registers::fifo::tx_tokens::EOP).unwrap();
        buffer.push(registers::fifo::tx_tokens::TX_OFF).unwrap();

        // Turn on TX
        buffer.push(registers::fifo::tx_tokens::TX_ON).unwrap();

        self.i2c_bus
            .write(DEVICE_SLAVE_ADDR, &buffer)
            .map_err(|err| Error::IOWriteError(err))

        // match self.wait_tx_done() {
        //     Err(Error::TxRetryFailed) => {
        //         // We didn't actually send the message so need to handle it here
        //         if self.current_msg_id == 0 {
        //             self.current_msg_id = 7;
        //         } else {
        //             self.current_msg_id -= 1;
        //         }
        //         Err(Error::TxRetryFailed)
        //     }
        //     val => val,
        // }
    }

    fn wait_tx_done(&mut self) -> Result<(), Error<T>> {
        loop {
            let status0a = self.read_register(registers::status0a::ADDR)?;
            if (status0a & registers::status0a::RETRYFAIL) != 0 {
                return Err(Error::TxRetryFailed);
            }

            if (status0a & registers::status0a::SOFTRST) != 0 {
                return Err(Error::SoftResetRequested);
            }

            if (status0a & registers::status0a::HARDRST) != 0 {
                return Err(Error::HardResetRequested);
            }

            let irqa = self.read_register(registers::interrupta::ADDR)?;
            if (irqa & registers::interrupta::TX_SENT) != 0 {
                defmt::info!("Message sent blocking");
                // Read GoodCrc from fifo
                self.receive_message()?;
                return Ok(());
            }
        }
    }

    pub fn is_rx_fifo_empty(&mut self) -> Result<bool, Error<T>> {
        let status1 = self.read_register(registers::status1::ADDR)?;
        Ok((status1 & registers::status1::RX_EMPTY) != 0)
    }

    pub fn receive_message(&mut self) -> Result<(SopTarget, MessageHeader, Vec<u8, 28>), Error<T>> {
        // Receive token and header
        let reg_addr = [registers::fifo::ADDR];
        let mut token_and_header = [0; 3];
        self.i2c_bus
            .write_read(DEVICE_SLAVE_ADDR, &reg_addr, &mut token_and_header)
            .map_err(|err| Error::IOReadError(err))?;

        let header = MessageHeader::from_inner(
            (token_and_header[1] as u16) | ((token_and_header[2] as u16) << 8),
        );

        let msg_type = header.message_type()?;
        let sop_target = match token_and_header[0] & registers::fifo::rx_tokens::MASK {
            registers::fifo::rx_tokens::SOP => {
                defmt::error!("SOP recv {}", msg_type);
                SopTarget::SOP
            }
            registers::fifo::rx_tokens::SOP1 => {
                defmt::info!("SOP' recv {}", msg_type);
                SopTarget::SOP1
            }
            registers::fifo::rx_tokens::SOP2 => {
                defmt::info!("SOP'' recv {}", msg_type);
                SopTarget::SOP2
            }
            registers::fifo::rx_tokens::SOP1DB => {
                defmt::info!("SOP'Debug recv {}", msg_type);
                SopTarget::SOP1DB
            }
            registers::fifo::rx_tokens::SOP2DB => {
                defmt::error!("SOP''Debug recv {}", msg_type);
                SopTarget::SOP2DB
            }
            _ => panic!("Unknown packet received {:?}", msg_type),
        };

        // Now read the rest of the message from the fifo
        let num_objects = header.num_objects();
        let num_bytes = num_objects * core::mem::size_of::<u32>();

        let mut raw_objects: Vec<u8, 28> = Vec::new();
        // Add 4 for the CRC
        raw_objects.resize(num_bytes + 4, 0).unwrap(); // Should always be < 7 by design

        self.i2c_bus
            .write_read(DEVICE_SLAVE_ADDR, &reg_addr, &mut raw_objects)
            .map_err(|err| Error::IOReadError(err))?;

        Ok((sop_target, header, raw_objects))
    }
}

#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum SopTarget {
    SOP,
    SOP1,
    SOP2,
    SOP1DB,
    SOP2DB,
}

impl SopTarget {
    fn into_tokens(self) -> [u8; 4] {
        match self {
            SopTarget::SOP => [
                registers::fifo::tx_tokens::SYNC1,
                registers::fifo::tx_tokens::SYNC1,
                registers::fifo::tx_tokens::SYNC1,
                registers::fifo::tx_tokens::SYNC2,
            ],
            SopTarget::SOP1 => [
                registers::fifo::tx_tokens::SYNC1,
                registers::fifo::tx_tokens::SYNC1,
                registers::fifo::tx_tokens::SYNC3,
                registers::fifo::tx_tokens::SYNC3,
            ],
            SopTarget::SOP2 => [
                registers::fifo::tx_tokens::SYNC1,
                registers::fifo::tx_tokens::SYNC3,
                registers::fifo::tx_tokens::SYNC1,
                registers::fifo::tx_tokens::SYNC3,
            ],
            SopTarget::SOP1DB => [
                registers::fifo::tx_tokens::SYNC1,
                registers::fifo::tx_tokens::RESET2,
                registers::fifo::tx_tokens::RESET2,
                registers::fifo::tx_tokens::SYNC3,
            ],
            SopTarget::SOP2DB => [
                registers::fifo::tx_tokens::SYNC1,
                registers::fifo::tx_tokens::RESET2,
                registers::fifo::tx_tokens::SYNC3,
                registers::fifo::tx_tokens::SYNC2,
            ],
        }
    }
}

const MAX_DATA_OBJECTS: usize = 7;

pub trait Message {
    fn get_header(&self) -> MessageHeader;
    fn get_data(&self) -> Vec<u32, MAX_DATA_OBJECTS>;
}

#[repr(u16)]
#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum ControlMessageType {
    // Control messages
    GoodCrc = 1,
    GotoMin = 2,
    Accept = 3,
    Reject = 4,
    Ping = 5,
    PsRdy = 6,
    GetSourceCap = 7,
    GetSinkCap = 8,
    DrSwap = 9,
    PrSwap = 10,
    VconnSwap = 11,
    Wait = 12,
    SoftReset = 13,
}

impl TryFrom<u16> for ControlMessageType {
    type Error = ();
    fn try_from(value: u16) -> Result<Self, Self::Error> {
        match value {
            1 => Ok(ControlMessageType::GoodCrc),
            2 => Ok(ControlMessageType::GotoMin),
            3 => Ok(ControlMessageType::Accept),
            4 => Ok(ControlMessageType::Reject),
            5 => Ok(ControlMessageType::Ping),
            6 => Ok(ControlMessageType::PsRdy),
            7 => Ok(ControlMessageType::GetSourceCap),
            8 => Ok(ControlMessageType::GetSinkCap),
            9 => Ok(ControlMessageType::DrSwap),
            10 => Ok(ControlMessageType::PrSwap),
            11 => Ok(ControlMessageType::VconnSwap),
            12 => Ok(ControlMessageType::Wait),
            13 => Ok(ControlMessageType::SoftReset),
            _ => Err(()),
        }
    }
}

#[repr(u16)]
#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum DataMessageType {
    SourceCapabilities = 1,
    Request = 2,
    BIST = 3,
    SinkCapabilities = 4,
    VendorDefined = 15,
}

impl TryFrom<u16> for DataMessageType {
    type Error = ();
    fn try_from(value: u16) -> Result<Self, Self::Error> {
        match value {
            1 => Ok(DataMessageType::SourceCapabilities),
            2 => Ok(DataMessageType::Request),
            3 => Ok(DataMessageType::BIST),
            4 => Ok(DataMessageType::SinkCapabilities),
            5 => Ok(DataMessageType::VendorDefined),
            _ => Err(()),
        }
    }
}

#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum MessageType {
    // Arg0: the type of the control message
    ControlMessage(ControlMessageType),
    // Arg0: the type of the data message
    // Arg1: the number of data objects
    DataMessage(DataMessageType, u16),
}

#[derive(Clone, Copy, Debug)]
#[repr(u16)]
enum Revision {
    R1_0 = 0b00,
    R2_0 = 0b01,
}

#[derive(defmt::Format)]
pub struct MessageHeader(u16);

impl MessageHeader {
    const NUM_DATA_OBJ_OFFSET: u8 = 12;
    const NUM_DATA_OBJ_MASK: u16 = 0x7 << Self::NUM_DATA_OBJ_OFFSET;
    const SPEC_REVISION_OFFSET: u8 = 6;
    const ID_OFFSET: u8 = 9;
    const ID_MASK: u16 = 7 << 9;
    const PDR_MASK: u16 = 1 << 5;
    const PPR_MASK: u16 = 1 << 8;
    const TYPE_MASK: u16 = 0x0f << 0;

    // Assumptions:
    // DFP is always source
    // We are never cable plug
    const fn new(msg_type: MessageType, port_type: Option<PortType>, revision: Revision) -> Self {
        let mut header_bits = match msg_type {
            MessageType::ControlMessage(r#type) => r#type as u16,
            MessageType::DataMessage(r#type, num_objects) => {
                (r#type as u16) | (num_objects << Self::NUM_DATA_OBJ_OFFSET)
            }
        };

        header_bits |= (revision as u16) << Self::SPEC_REVISION_OFFSET;

        if matches!(port_type, Some(PortType::DFP)) {
            header_bits |= Self::PDR_MASK | Self::PPR_MASK;
        }

        Self(header_bits)
    }

    const fn from_inner(inner: u16) -> Self {
        Self(inner)
    }

    fn set_id(&mut self, id: u8) {
        self.0 = self.0 | ((id as u16) << Self::ID_OFFSET) & Self::ID_MASK;
    }

    const fn into_inner(&self) -> u16 {
        self.0
    }

    const fn num_objects(&self) -> usize {
        ((self.0 & Self::NUM_DATA_OBJ_MASK) >> Self::NUM_DATA_OBJ_OFFSET) as usize
    }

    pub fn message_type<T: WriteRead + Write>(&self) -> Result<MessageType, Error<T>> {
        let r#type = self.0 & Self::TYPE_MASK;
        let num_objects = self.num_objects();
        if num_objects == 0 {
            let ctrl_message = r#type.try_into().map_err(|_| Error::InvalidHeader)?;
            Ok(MessageType::ControlMessage(ctrl_message))
        } else {
            let data_message = r#type.try_into().map_err(|_| Error::InvalidHeader)?;
            Ok(MessageType::DataMessage(data_message, num_objects as u16))
        }
    }
}

struct PowerDataObject(u32);

impl PowerDataObject {
    fn new_fixed_safe5v(current_ma: u32) -> Self {
        const FIXED_VOLTAGE_OFFSET: usize = 10;
        const VOLTAGE_LSB_MV: u32 = 50;

        const VSAFE5V_MV: u32 = 5000;

        const MAX_CURRENT_OFFSET: usize = 0;
        const MAX_CURRENT_LSB_MA: u32 = 10;

        let current_code = current_ma / MAX_CURRENT_LSB_MA;

        let voltage = ((VSAFE5V_MV / VOLTAGE_LSB_MV) << FIXED_VOLTAGE_OFFSET)
            | (current_code << MAX_CURRENT_OFFSET)
            | (1 << 26); // Usb comms capable
                         // | (1 << 28)  // Usb suspend
                         // | (1 << 29)  // Usb DRP
                         // | (1 << 25); // Usb DRD

        Self(voltage)
    }
}

// TODO(javier-varez): Add support to other supplies when we have them.
pub struct SourceCapabilitiesMessage {
    current_ma: u32,
}

impl SourceCapabilitiesMessage {
    pub const fn new_no_capabilities() -> Self {
        Self { current_ma: 0 }
    }

    pub fn set_fixed_current(&mut self, ma: u32) {
        self.current_ma = ma;
    }
}

impl Message for SourceCapabilitiesMessage {
    fn get_header(&self) -> MessageHeader {
        let msg_type = MessageType::DataMessage(DataMessageType::SourceCapabilities, 1);
        let header = MessageHeader::new(msg_type, Some(PortType::DFP), Revision::R2_0);
        header
    }

    fn get_data(&self) -> Vec<u32, MAX_DATA_OBJECTS> {
        let mut objects = Vec::new();
        let fixed_supply = PowerDataObject::new_fixed_safe5v(self.current_ma);
        objects.push(fixed_supply.0).unwrap();
        objects
    }
}

pub struct AcceptMessage {}

impl AcceptMessage {
    pub fn new() -> Self {
        Self {}
    }
}

impl Message for AcceptMessage {
    fn get_header(&self) -> MessageHeader {
        let msg_type = MessageType::ControlMessage(ControlMessageType::Accept);
        let header = MessageHeader::new(msg_type, Some(PortType::DFP), Revision::R2_0);
        header
    }

    fn get_data(&self) -> Vec<u32, MAX_DATA_OBJECTS> {
        Vec::new()
    }
}

pub struct PsRdyMessage {}

impl PsRdyMessage {
    pub fn new() -> Self {
        Self {}
    }
}

impl Message for PsRdyMessage {
    fn get_header(&self) -> MessageHeader {
        let msg_type = MessageType::ControlMessage(ControlMessageType::PsRdy);
        let header = MessageHeader::new(msg_type, Some(PortType::DFP), Revision::R2_0);
        header
    }

    fn get_data(&self) -> Vec<u32, MAX_DATA_OBJECTS> {
        Vec::new()
    }
}

pub struct StructuredVdmMessage {
    vdm_header: u32,
    objects: Vec<u32, { MAX_DATA_OBJECTS - 1 }>,
}

impl StructuredVdmMessage {
    pub fn new(svid: u16, command: u16, objects: &[u32]) -> Self {
        let vdm_header = ((svid as u32) << 16) | 0x8000 | (command as u32);
        let mut vector: Vec<u32, { MAX_DATA_OBJECTS - 1 }> = Vec::new();
        for object in objects {
            vector.push(*object).unwrap();
        }

        Self {
            vdm_header,
            objects: vector,
        }
    }
}

impl Message for StructuredVdmMessage {
    fn get_header(&self) -> MessageHeader {
        let msg_type = MessageType::DataMessage(
            DataMessageType::VendorDefined,
            1 + self.objects.len() as u16,
        );
        MessageHeader::new(msg_type, Some(PortType::DFP), Revision::R2_0)
    }

    fn get_data(&self) -> Vec<u32, MAX_DATA_OBJECTS> {
        let mut vector = Vec::new();

        vector.push(self.vdm_header).unwrap();
        for object in &self.objects {
            vector.push(*object).unwrap();
        }
        vector
    }
}
