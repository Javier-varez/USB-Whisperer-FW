#![no_std]
#![no_main]

mod fusb302;
mod system_timer;
mod usb_fsm;

use defmt_rtt as _; // global logger
use panic_probe as _;

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

#[rtic::app(device = nrf52840_hal::pac, dispatchers = [NFCT])]
mod app {
    use super::{fusb302, usb_fsm};
    use nrf52840_hal as hal;
    use systick_monotonic::*;
    
    use hal::{Twim, pac::{TIMER0, TWIM0}, gpio::{Pin, Input, Output, PullUp, PushPull}, timer::Timer};
    
    defmt::timestamp!("{=u64:us}", {
        monotonics::now().ticks() * 1000
    });

    
    // Local resources to specific tasks (cannot be shared)
    #[local]
    struct Local {
        state_machine: usb_fsm::UsbFsm<Twim<TWIM0>, Pin<Input<PullUp>>, Pin<Output<PushPull>>>,
        timer: Timer<TIMER0>
    }

    // Resources shared between tasks
    #[shared]
    struct Shared {}

    #[monotonic(binds = SysTick, default = true, priority = 7)]
    type SystickMonotonic = Systick<1000>; // 1000 Hz / 1 ms granularity

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let cm_peripherals = cx.core;
        let peripherals = cx.device;

        let _clocks = hal::clocks::Clocks::new(peripherals.CLOCK).enable_ext_hfosc();

        let mono = Systick::new(cm_peripherals.SYST, 64_000_000);

        let p0 = hal::gpio::p0::Parts::new(peripherals.P0);
        let p1 = hal::gpio::p1::Parts::new(peripherals.P1);
        let scl = p1.p1_06.into_floating_input().degrade();
        let sda = p1.p1_05.into_floating_input().degrade();

        let pins = hal::twim::Pins { scl, sda };
        let frequency = hal::twim::Frequency::K400;

        let twim = hal::twim::Twim::new(peripherals.TWIM0, pins, frequency);

        let irq_pin = p0.p0_19.into_pullup_input().degrade();
        let vbus_pin = p0
            .p0_20
            .into_push_pull_output(hal::gpio::Level::Low)
            .degrade();

        let timer = hal::timer::Timer::new(peripherals.TIMER0);

        let fusb302 = fusb302::Fusb302::new(twim).unwrap();
        let state_machine = usb_fsm::UsbFsm::new(fusb302, irq_pin, vbus_pin).unwrap();
        
        run_state_machine::spawn().unwrap();

        (Shared {}, Local {state_machine, timer}, init::Monotonics(mono))
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }
    
    #[task(local = [state_machine, timer], shared = [])]
    fn run_state_machine(cx: run_state_machine::Context) {
        let timer = cx.local.timer;
        let state_machine = cx.local.state_machine;
        
        state_machine.run(timer).unwrap();
        
        // Spawn again after 4 ms
        run_state_machine::spawn_after(4.millis()).unwrap();
    }    
}
