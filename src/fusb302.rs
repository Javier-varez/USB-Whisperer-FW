const DEVICE_SLAVE_ADDR: u8 = 0x22;
const DEVICE_ID_REG: u8 = 0x01;

use embedded_hal::blocking::i2c::{Read, Write};

pub enum Error<T: Read + Write> {
    DeviceNotFound,
    IOReadError(<T as Read>::Error),
    IOWriteError(<T as Write>::Error),
}

impl<T: Read + Write> core::fmt::Debug for Error<T> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::IOReadError(_) => write!(f, "IO Error (Read)"),
            Error::IOWriteError(_) => write!(f, "IO Error (Write)"),
            Error::DeviceNotFound => write!(f, "DeviceNotFound"),
        }
    }
}

pub struct Fusb302<T: Read + Write> {
    i2c_bus: T,
}

impl<T: Read + Write> Fusb302<T> {
    pub fn new(i2c_bus: T) -> Result<Self, Error<T>> {
        // Try to read device ID and verify it is connected, otherwise return an error

        let mut instance = Self { i2c_bus };
        let id = instance.read_register(DEVICE_ID_REG)?;

        // Check if device ID is valid
        if id & 0x80 == 0 {
            return Err(Error::DeviceNotFound);
        }
        defmt::info!("Found Fusb302 with ID {}", id);

        Ok(instance)
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

    fn write_register(&mut self, address: u8, value: u8) -> Result<(), Error<T>> {
        let addr_buffer = [address];
        self.i2c_bus
            .write(DEVICE_SLAVE_ADDR, &addr_buffer)
            .map_err(|err| Error::IOWriteError(err))?;

        let register_value = [value];
        self.i2c_bus
            .write(DEVICE_SLAVE_ADDR, &register_value)
            .map_err(|err| Error::IOWriteError(err))?;

        Ok(())
    }
}
