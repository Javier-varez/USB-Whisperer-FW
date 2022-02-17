#![no_std]
#![no_main]

mod fusb302;
mod usb_fsm;

use defmt_rtt as _; // global logger
use panic_probe as _;

use nrf52840_hal as hal;

use embedded_hal::blocking::delay::DelayUs;

use embedded_hal::digital::v2::InputPin;

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
    let peripherals = hal::pac::Peripherals::take().unwrap();
    let _clocks = hal::clocks::Clocks::new(peripherals.CLOCK).enable_ext_hfosc();

    let p0 = hal::gpio::p0::Parts::new(peripherals.P0);
    let p1 = hal::gpio::p1::Parts::new(peripherals.P1);
    let scl = p1.p1_06.into_floating_input().degrade();
    let sda = p1.p1_05.into_floating_input().degrade();

    let pins = hal::twim::Pins { scl, sda };
    let frequency = hal::twim::Frequency::K400;

    let twim = hal::twim::Twim::new(peripherals.TWIM0, pins, frequency);

    let irq_pin = p0.p0_19.into_pullup_input().degrade();

    let mut timer = hal::timer::Timer::new(peripherals.TIMER0);

    let fusb302 = fusb302::Fusb302::new(twim).unwrap();
    let mut state_machine = usb_fsm::UsbFsm::new(fusb302, irq_pin).unwrap();

    loop {
        state_machine.run(&mut timer).unwrap();

        timer.delay_us(10u32);
    }
}
