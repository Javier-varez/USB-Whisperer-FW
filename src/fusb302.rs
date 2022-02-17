const DEVICE_SLAVE_ADDR: u8 = 0x22;
const PD_RETRIES_COUNT: u8 = 3;

mod registers;

use embedded_hal::blocking::i2c::{Read, Write};

pub enum Error<T: Read + Write> {
    DeviceNotFound,
    IOReadError(<T as Read>::Error),
    IOWriteError(<T as Write>::Error),
    UnknownPolarity,
}

impl<T: Read + Write> core::fmt::Debug for Error<T> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::IOReadError(_) => write!(f, "IO Error (Read)"),
            Error::IOWriteError(_) => write!(f, "IO Error (Write)"),
            Error::DeviceNotFound => write!(f, "DeviceNotFound"),
            Error::UnknownPolarity => write!(f, "UnknownPolarity"),
        }
    }
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
    UFP,
}

pub struct Fusb302<T: Read + Write> {
    i2c_bus: T,
    polarity: Option<CcLine>,
    port_type: PortType,
}

impl<T: Read + Write> Fusb302<T> {
    pub fn new(i2c_bus: T) -> Result<Self, Error<T>> {
        // Try to read device ID and verify it is connected, otherwise return an error

        let mut instance = Self {
            i2c_bus,
            polarity: None,
            port_type: PortType::Open,
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

        // Set auto retry and the num_retries to 3
        self.modify_register(registers::control3::ADDR, |value| {
            (value & !registers::control3::N_RETRIES_MASK)
                | (PD_RETRIES_COUNT << registers::control3::N_RETRIES_OFFSET)
                | registers::control3::AUTO_RETRY
        })?;

        self.write_register(
            registers::control1::ADDR,
            registers::control1::ENSOP1DB
                | registers::control1::ENSOP2DB
                | registers::control1::RX_FLUSH,
        )?;

        self.auto_goodcrc_enable(false)?;

        self.mask_all_interrutps()?;

        // Enable power
        self.write_register(registers::power::ADDR, registers::power::PWR_ALL_MASK)
    }

    fn mask_all_interrutps(&mut self) -> Result<(), Error<T>> {
        self.write_register(registers::mask::ADDR, 0xff)?;
        self.write_register(registers::maska::ADDR, 0xff)?;
        self.write_register(registers::maskb::ADDR, 0xff)
    }

    pub fn reset_pd(&mut self) -> Result<(), Error<T>> {
        self.write_register(registers::reset::ADDR, registers::reset::PD_RESET)
    }

    pub fn configure_cc_mode(&mut self, port_type: PortType) -> Result<(), Error<T>> {
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
                (CcLine::Cc1, PortType::DFP) => open | registers::switches0::CC1_PU_EN,
                (CcLine::Cc2, PortType::DFP) => open | registers::switches0::CC2_PU_EN,
                (_, PortType::UFP) => {
                    open | registers::switches0::CC1_PD_EN | registers::switches0::CC2_PD_EN
                }
                (_, PortType::Open) => open,
            }
        })?;

        match port_type {
            PortType::DFP => {
                self.configure_disconnect_monitoring()?;
            }
            PortType::UFP => {
                unimplemented!();
            }
            PortType::Open => {
                self.mask_all_interrutps()?;
                self.configure_disconnect_monitoring()?;
            }
        }

        self.port_type = port_type;

        Ok(())
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

    pub fn detect_cc_orientation<U>(&mut self, timer: &mut U) -> Result<Option<CcLine>, Error<T>>
    where
        U: embedded_hal::blocking::delay::DelayUs<u32>,
    {
        let cc1 = self.detect_rd_sink(CcLine::Cc1, timer)?;
        timer.delay_us(100);
        let cc2 = self.detect_rd_sink(CcLine::Cc2, timer)?;

        if cc1 && cc2 {
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

    pub fn is_attached(&self) -> bool {
        self.polarity.is_some()
    }

    pub fn get_polarity(&self) -> Option<CcLine> {
        self.polarity
    }

    fn detect_rd_sink<U>(&mut self, line: CcLine, timer: &mut U) -> Result<bool, Error<T>>
    where
        U: embedded_hal::blocking::delay::DelayUs<u32>,
    {
        self.modify_register(registers::switches0::ADDR, |reg| {
            let open = reg
                & !(registers::switches0::CC2_PU_EN
                    | registers::switches0::CC1_PU_EN
                    | registers::switches0::MEAS_CC1
                    | registers::switches0::MEAS_CC2
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

        self.monitor_connection(timer)
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

    pub fn disable_interrupts(&mut self) -> Result<(), Error<T>> {
        self.modify_register(registers::control0::ADDR, |reg| {
            reg | registers::control0::INT_MASK
        })
    }

    fn read_register(&mut self, address: u8) -> Result<u8, Error<T>> {
        let addr_buffer = [address];
        self.i2c_bus
            .write(DEVICE_SLAVE_ADDR, &addr_buffer)
            .map_err(|err| Error::IOWriteError(err))?;

        let mut register_value = [0];
        self.i2c_bus
            .read(DEVICE_SLAVE_ADDR, &mut register_value)
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
}
