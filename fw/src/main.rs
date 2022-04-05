#![no_std]
#![no_main]

mod cmd_handler;
mod fusb302;
mod system_timer;
mod usb_pd_fsm;

use defmt_rtt as _; // global logger

use core::panic::PanicInfo;

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    use embedded_hal::blocking::delay::DelayMs;
    use embedded_hal::digital::v2::{OutputPin, StatefulOutputPin};
    use nrf52840_hal as hal;

    let peripherals = unsafe { hal::pac::Peripherals::steal() };
    let p0 = hal::gpio::p0::Parts::new(peripherals.P0);

    let mut timer = hal::timer::Timer::new(peripherals.TIMER0);

    let mut led = p0
        .p0_30
        .into_push_pull_output(hal::gpio::Level::High)
        .degrade();

    defmt::error!("Panicked with message: {}", defmt::Display2Format(info));

    for _ in 0..10 {
        if led.is_set_high().unwrap() {
            led.set_low().unwrap();
        } else {
            led.set_high().unwrap();
        }
        timer.delay_ms(1000u32);
    }
    led.set_high().unwrap();

    cortex_m::asm::udf();
}

/// Terminates the application and makes `probe-run` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}

#[rtic::app(device = nrf52840_hal::pac, dispatchers = [NFCT, USBD, TIMER0])]
mod app {
    use super::{cmd_handler, fusb302, usb_pd_fsm};
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

    use usb_whisperer_lib::message::Message;

    use embedded_hal::digital::v2::{OutputPin, StatefulOutputPin};

    defmt::timestamp!("{=u64:us}", { monotonics::now().ticks() * 1000 });

    // Type shorthands
    type Clocks =
        clocks::Clocks<clocks::ExternalOscillator, clocks::Internal, clocks::LfOscStopped>;
    type UsbBus =
        usb_device::bus::UsbBusAllocator<hal::usbd::Usbd<hal::usbd::UsbPeripheral<'static>>>;
    pub type UsbSerial = SerialPort<'static, hal::usbd::Usbd<hal::usbd::UsbPeripheral<'static>>>;
    type UsbDevice =
        usb_device::device::UsbDevice<'static, hal::usbd::Usbd<hal::usbd::UsbPeripheral<'static>>>;
    type UsbFsm = usb_pd_fsm::UsbPdFsm<Twim<TWIM0>, Pin<Input<PullUp>>, Pin<Output<PushPull>>>;

    // Local resources to specific tasks (cannot be shared)
    #[local]
    struct Local {
        timer: Timer<TIMER0>,
        gpiote: Gpiote,
        usb_device: UsbDevice,
        command_handler: cmd_handler::CommandHandler,
        led: Pin<Output<PushPull>>,
    }

    // Resources shared between tasks
    #[shared]
    struct Shared {
        usb_pd_fsm_spawn_handle: Option<usb_pd_fsm_task::SpawnHandle>,
        usb_serial: UsbSerial,
        state_machine: UsbFsm,
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

        let led = p0
            .p0_29
            .into_push_pull_output(hal::gpio::Level::High)
            .degrade();

        let gpiote = hal::gpiote::Gpiote::new(peripherals.GPIOTE);
        gpiote
            .channel0()
            .input_pin(&irq_pin)
            .hi_to_lo()
            .enable_interrupt();

        let fusb302 = fusb302::Fusb302::new(twim).unwrap();
        let state_machine = usb_pd_fsm::UsbPdFsm::new(fusb302, irq_pin, vbus_pin).unwrap();

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
            .max_power(500)
            .build();

        usb_pd_fsm_task::spawn().unwrap();
        usb_device_task::spawn().unwrap();
        led_task::spawn_after(1.secs()).unwrap();

        let command_handler = cmd_handler::CommandHandler::new();

        (
            Shared {
                usb_pd_fsm_spawn_handle: None,
                usb_serial,
                state_machine,
            },
            Local {
                timer,
                gpiote,
                usb_device,
                command_handler,
                led,
            },
            init::Monotonics(mono),
        )
    }

    #[idle]
    fn idle_task(_: idle_task::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(local = [usb_device], shared = [usb_serial], priority = 3)]
    fn usb_device_task(cx: usb_device_task::Context) {
        let usb_device_task::LocalResources { usb_device } = cx.local;
        let usb_device_task::SharedResources { mut usb_serial } = cx.shared;

        usb_serial.lock(|usb_serial| {
            if usb_device.poll(&mut [usb_serial]) {
                // Make sure the message handler will process the event
                msg_handler_task::spawn().ok();
            }
        });

        usb_device_task::spawn_after(2.millis()).unwrap();
    }

    #[task(shared = [usb_serial, state_machine], local = [command_handler], priority = 2)]
    fn msg_handler_task(cx: msg_handler_task::Context) {
        let msg_handler_task::SharedResources {
            mut usb_serial,
            mut state_machine,
        } = cx.shared;
        let msg_handler_task::LocalResources { command_handler } = cx.local;

        let mut buffer = [0u8; 128];
        match usb_serial.lock(|usb_serial| usb_serial.read(&mut buffer)) {
            Ok(size) => match command_handler.deserialize_message(&mut buffer[..size]) {
                Ok(message) => {
                    let mut send_message = |message: &Message| {
                        let message_buffer = command_handler
                            .serialize_message(message, &mut buffer)
                            .unwrap();
                        usb_serial
                            .lock(|usb_serial| usb_serial.write(message_buffer))
                            .unwrap()
                    };

                    let response = state_machine
                        .lock(|sm| sm.run_command(&message))
                        .unwrap_or(Message::Nack);
                    send_message(&response);
                }
                Err(usb_whisperer_lib::message::Error::WouldBlock) => {}
            },
            Err(UsbError::WouldBlock) => {}
            Err(_) => {
                defmt::warn!("Read from usb failed");
            }
        }
    }

    #[task(local = [gpiote], shared = [usb_pd_fsm_spawn_handle], binds = GPIOTE, priority = 1)]
    fn gpiote_irq_task(cx: gpiote_irq_task::Context) {
        // We need to handle the IRQ here and let the state machine now it needs to run
        let gpiote = cx.local.gpiote;
        let mut spawn_handle = cx.shared.usb_pd_fsm_spawn_handle;

        if gpiote.channel0().is_event_triggered() {
            defmt::info!("IRQ: FUSB302");
            // Cancel pending state machine run if any
            spawn_handle.lock(|handle| {
                if let Some(handle) = handle.take() {
                    handle.cancel().ok();
                }
            });

            usb_pd_fsm_task::spawn().ok();
            gpiote.channel0().reset_events();
        }
    }

    #[task(local = [timer], shared = [usb_pd_fsm_spawn_handle, state_machine], capacity = 1, priority = 1)]
    fn usb_pd_fsm_task(cx: usb_pd_fsm_task::Context) {
        let usb_pd_fsm_task::LocalResources { timer } = cx.local;
        let usb_pd_fsm_task::SharedResources {
            mut usb_pd_fsm_spawn_handle,
            mut state_machine,
        } = cx.shared;

        state_machine.lock(|m| m.run(timer).unwrap());

        // Spawn again after 4 ms
        let handle = usb_pd_fsm_task::spawn_after(4.millis()).unwrap();
        usb_pd_fsm_spawn_handle.lock(|hdl| hdl.replace(handle));
    }

    #[task(local = [led], priority = 1)]
    fn led_task(cx: led_task::Context) {
        let led_task::LocalResources { led } = cx.local;
        if led.is_set_high().unwrap() {
            led.set_low().unwrap();
        } else {
            led.set_high().unwrap();
        }
        led_task::spawn_after(1.secs()).unwrap();
    }
}
