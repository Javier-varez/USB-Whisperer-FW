#![no_std]
#![no_main]

mod fusb302;

use defmt_rtt as _; // global logger
use panic_probe as _;

use nrf52840_hal::{self as hal, prelude::*, twim::Instance};

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

/// Terminates the application and makes `probe-run` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    defmt::println!("Hello, world!");

    let peripherals = hal::pac::Peripherals::take().unwrap();

    let p1 = hal::gpio::p1::Parts::new(peripherals.P1);
    let scl = p1.p1_06.into_floating_input().degrade();
    let sda = p1.p1_05.into_floating_input().degrade();

    let pins = hal::twim::Pins { scl, sda };
    let frequency = hal::twim::Frequency::K400;

    let twim = hal::twim::Twim::new(peripherals.TWIM0, pins, frequency);
    let _fusb302 = fusb302::Fusb302::new(twim).unwrap();

    exit()
}
