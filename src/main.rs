#![no_std]
#![no_main]

mod fusb302;

use defmt_rtt as _; // global logger
use panic_probe as _;

use nrf52840_hal as hal;

use embedded_hal::blocking::delay::DelayUs;

use embedded_hal::digital::v2::InputPin;

use fusb302::{CcLine, PortType};

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

    let mut fusb302 = fusb302::Fusb302::new(twim).unwrap();
    fusb302.init().unwrap();
    fusb302.reset_pd().unwrap();
    fusb302.configure_cc_mode(PortType::DFP).unwrap();

    let mut orientation = None;
    loop {
        // No interrupts should be received yet, since they are off
        if irq_pin.is_low().unwrap() {
            defmt::info!("Irq from fusb302");
        }

        // Make sure we are still connected to the right source
        let new_orientation = fusb302.detect_cc_orientation(&mut timer).unwrap();
        if new_orientation != orientation {
            orientation = new_orientation;
            match orientation {
                Some(line) => {
                    defmt::info!("Usb connected. {:?}", line);
                }
                None => {
                    defmt::info!("Usb disconnected.");
                }
            }
        }

        timer.delay_us(1000u32);
    }
}
