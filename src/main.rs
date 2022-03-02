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

#[rtic::app(device = nrf52840_hal::pac, dispatchers = [NFCT, USBD])]
mod app {
    use super::{fusb302, usb_fsm};
    use nrf52840_hal::{self as hal, gpiote::Gpiote};
    use systick_monotonic::*;

    use hal::{
        clocks,
        gpio::{Input, Output, Pin, PullUp, PushPull},
        pac::{TIMER0, TWIM0},
        timer::Timer,
        Twim,
    };

    use usb_device::prelude::*;
    use usbd_serial::SerialPort;

    defmt::timestamp!("{=u64:us}", { monotonics::now().ticks() * 1000 });

    // Type shorthands
    type Clocks =
        clocks::Clocks<clocks::ExternalOscillator, clocks::Internal, clocks::LfOscStopped>;
    type UsbBus =
        usb_device::bus::UsbBusAllocator<hal::usbd::Usbd<hal::usbd::UsbPeripheral<'static>>>;
    type UsbSerial = SerialPort<'static, hal::usbd::Usbd<hal::usbd::UsbPeripheral<'static>>>;
    type UsbDevice =
        usb_device::device::UsbDevice<'static, hal::usbd::Usbd<hal::usbd::UsbPeripheral<'static>>>;
    type UsbFsm = usb_fsm::UsbFsm<Twim<TWIM0>, Pin<Input<PullUp>>, Pin<Output<PushPull>>>;

    // Local resources to specific tasks (cannot be shared)
    #[local]
    struct Local {
        state_machine: UsbFsm,
        timer: Timer<TIMER0>,
        gpiote: Gpiote,
        usb_device: UsbDevice,
        usb_serial: UsbSerial,
    }

    // Resources shared between tasks
    #[shared]
    struct Shared {
        fsm_spawn_handle: Option<run_state_machine::SpawnHandle>,
    }

    #[monotonic(binds = SysTick, default = true, priority = 7)]
    type SystickMonotonic = Systick<1000>; // 1000 Hz / 1 ms granularity

    #[init(local = [clocks: Option<Clocks> = None, usb_bus: Option<UsbBus> = None])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let cm_peripherals = cx.core;
        let peripherals = cx.device;

        let clocks = hal::clocks::Clocks::new(peripherals.CLOCK).enable_ext_hfosc();
        cx.local.clocks.replace(clocks);

        let clocks = cx.local.clocks.as_ref().unwrap();

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

        let usb_peripheral = hal::usbd::UsbPeripheral::new(peripherals.USBD, &clocks);
        let usb_bus = hal::usbd::Usbd::new(usb_peripheral);
        cx.local.usb_bus.replace(usb_bus);
        let usb_bus = cx.local.usb_bus.as_ref().unwrap();

        let usb_serial = SerialPort::new(&usb_bus);

        // TODO(javier-varez): Modify VID-PID pair. Currently using a test set from pid.codes
        let usb_device = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x1209, 0x0001))
            .manufacturer("AllThingsEmbedded")
            .product("USB Whisperer")
            .device_class(usbd_serial::USB_CLASS_CDC)
            .serial_number("A000_0000")
            .build();

        run_state_machine::spawn().unwrap();
        usb_task::spawn().unwrap();

        (
            Shared {
                fsm_spawn_handle: None,
            },
            Local {
                state_machine,
                timer,
                gpiote,
                usb_device,
                usb_serial,
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

    #[task(local = [usb_device, usb_serial], priority = 3)]
    fn usb_task(cx: usb_task::Context) {
        let usb_task::LocalResources {
            usb_device,
            usb_serial,
        } = cx.local;

        if usb_device.poll(&mut [usb_serial]) {
            usb_serial.write("hello world!\n".as_bytes()).ok();
        }

        usb_task::spawn_after(2.millis()).unwrap();
    }

    #[task(local = [gpiote], shared = [fsm_spawn_handle], binds = GPIOTE, priority = 1)]
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
