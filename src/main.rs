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
    use nrf52840_hal::{self as hal, gpiote::Gpiote};
    use systick_monotonic::*;

    use hal::{
        gpio::{Input, Output, Pin, PullUp, PushPull},
        pac::{TIMER0, TWIM0},
        timer::Timer,
        Twim,
    };

    defmt::timestamp!("{=u64:us}", { monotonics::now().ticks() * 1000 });

    // Local resources to specific tasks (cannot be shared)
    #[local]
    struct Local {
        state_machine: usb_fsm::UsbFsm<Twim<TWIM0>, Pin<Input<PullUp>>, Pin<Output<PushPull>>>,
        timer: Timer<TIMER0>,
        gpiote: Gpiote,
    }

    // Resources shared between tasks
    #[shared]
    struct Shared {
        fsm_spawn_handle: Option<run_state_machine::SpawnHandle>,
    }

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

        let gpiote = hal::gpiote::Gpiote::new(peripherals.GPIOTE);
        gpiote
            .channel0()
            .input_pin(&irq_pin)
            .hi_to_lo()
            .enable_interrupt();

        let fusb302 = fusb302::Fusb302::new(twim).unwrap();
        let state_machine = usb_fsm::UsbFsm::new(fusb302, irq_pin, vbus_pin).unwrap();

        run_state_machine::spawn().unwrap();

        (
            Shared {
                fsm_spawn_handle: None,
            },
            Local {
                state_machine,
                timer,
                gpiote,
            },
            init::Monotonics(mono),
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(local = [gpiote], shared = [fsm_spawn_handle], binds = GPIOTE, priority = 2)]
    fn gpiote_irq(cx: gpiote_irq::Context) {
        // We need to handle the IRQ here and let the state machine now it needs to run
        let gpiote = cx.local.gpiote;
        let mut spawn_handle = cx.shared.fsm_spawn_handle;

        if gpiote.channel0().is_event_triggered() {
            defmt::info!("IRQ: FUSB302");
            // Cancel pending state machine run if any
            spawn_handle.lock(|handle| {
                if let Some(handle) = handle.take() {
                    handle.cancel().ok();
                }
            });

            run_state_machine::spawn().unwrap();
            gpiote.channel0().reset_events();
        }
    }

    #[task(local = [state_machine, timer], shared = [fsm_spawn_handle], capacity = 1, priority = 1)]
    fn run_state_machine(cx: run_state_machine::Context) {
        let run_state_machine::LocalResources {
            timer,
            state_machine,
        } = cx.local;
        let mut spawn_handle = cx.shared.fsm_spawn_handle;

        state_machine.run(timer).unwrap();

        // Spawn again after 4 ms
        let handle = run_state_machine::spawn_after(4.millis()).unwrap();
        spawn_handle.lock(|hdl| hdl.replace(handle));
    }
}
