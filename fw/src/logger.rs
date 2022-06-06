mod channel;

use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

use cortex_m::{interrupt, register};

use channel::Channel;

pub const BUF_SIZE: usize = 1024;

#[defmt::global_logger]
struct Logger;

/// Global logger lock.
static TAKEN: AtomicBool = AtomicBool::new(false);
static INTERRUPTS_ACTIVE: AtomicBool = AtomicBool::new(false);
static mut ENCODER: defmt::Encoder = defmt::Encoder::new();
static mut USB_PRODUCER: Option<heapless::spsc::Producer<'static, u8, BUF_SIZE>> = None;

unsafe impl defmt::Logger for Logger {
    fn acquire() {
        let primask = register::primask::read();
        interrupt::disable();

        if TAKEN.load(Ordering::Relaxed) {
            panic!("defmt logger taken reentrantly")
        }

        // no need for CAS because interrupts are disabled
        TAKEN.store(true, Ordering::Relaxed);

        INTERRUPTS_ACTIVE.store(primask.is_active(), Ordering::Relaxed);

        // safety: accessing the `static mut` is OK because we have disabled interrupts.
        unsafe { ENCODER.start_frame(do_write) }
    }

    unsafe fn flush() {
        // SAFETY: if we get here, the global logger mutex is currently acquired
        handle().flush();
    }

    unsafe fn release() {
        // safety: accessing the `static mut` is OK because we have disabled interrupts.
        ENCODER.end_frame(do_write);

        TAKEN.store(false, Ordering::Relaxed);
        if INTERRUPTS_ACTIVE.load(Ordering::Relaxed) {
            // re-enable interrupts
            interrupt::enable()
        }
    }

    unsafe fn write(bytes: &[u8]) {
        // safety: accessing the `static mut` is OK because we have disabled interrupts.
        ENCODER.write(bytes, do_write);
    }
}

pub unsafe fn register_usb_queue(producer: heapless::spsc::Producer<'static, u8, BUF_SIZE>) {
    USB_PRODUCER.replace(producer);
}

fn do_write(bytes: &[u8]) {
    unsafe {
        handle().write_all(bytes);
        // Also append to the usb buffer
        if let Some(producer) = USB_PRODUCER.as_mut() {
            for byte in bytes {
                producer.enqueue(*byte).unwrap();
            }
        }
    }
}

#[repr(C)]
struct Header {
    id: [u8; 16],
    max_up_channels: usize,
    max_down_channels: usize,
    up_channel: Channel,
}

const MODE_MASK: usize = 0b11;
/// Block the application if the RTT buffer is full, wait for the host to read data.
const MODE_BLOCK_IF_FULL: usize = 2;
/// Don't block if the RTT buffer is full. Truncate data to output as much as fits.
const MODE_NON_BLOCKING_TRIM: usize = 1;

// make sure we only get shared references to the header/channel (avoid UB)
/// # Safety
/// `Channel` API is not re-entrant; this handle should not be held from different execution
/// contexts (e.g. thread-mode, interrupt context)
unsafe fn handle() -> &'static Channel {
    // NOTE the `rtt-target` API is too permissive. It allows writing arbitrary data to any
    // channel (`set_print_channel` + `rprint*`) and that can corrupt defmt log frames.
    // So we declare the RTT control block here and make it impossible to use `rtt-target` together
    // with this crate.
    #[no_mangle]
    static mut _SEGGER_RTT: Header = Header {
        id: *b"SEGGER RTT\0\0\0\0\0\0",
        max_up_channels: 1,
        max_down_channels: 0,
        up_channel: Channel {
            name: NAME as *const _ as *const u8,
            buffer: unsafe { &mut BUFFER as *mut _ as *mut u8 },
            size: BUF_SIZE,
            write: AtomicUsize::new(0),
            read: AtomicUsize::new(0),
            flags: AtomicUsize::new(MODE_NON_BLOCKING_TRIM),
        },
    };

    #[cfg_attr(target_os = "macos", link_section = ".uninit,defmt-rtt.BUFFER")]
    #[cfg_attr(not(target_os = "macos"), link_section = ".uninit.defmt-rtt.BUFFER")]
    static mut BUFFER: [u8; BUF_SIZE] = [0; BUF_SIZE];

    static NAME: &[u8] = b"defmt\0";

    &_SEGGER_RTT.up_channel
}
