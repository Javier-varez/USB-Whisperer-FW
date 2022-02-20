use cortex_m::peripheral::syst::SystClkSource;
use cortex_m::peripheral::SYST;

use core::sync::atomic::{AtomicU32, Ordering::Relaxed};

const SYST_FREQ_HZ: u32 = 1000;

struct SystemTimer {
    _systick: SYST,
    count: AtomicU32,
}

impl SystemTimer {
    fn new(mut timer: SYST, clock_hz: u32) -> Self {
        timer.set_clock_source(SystClkSource::Core);
        timer.set_reload(clock_hz / SYST_FREQ_HZ);
        timer.enable_counter();
        timer.enable_interrupt();

        Self {
            _systick: timer,
            count: AtomicU32::new(0),
        }
    }

    // At least should last for 49 days without overflow...
    // In practice I don't expect this to be on for that long and if it is, a spurious failure
    // should be fine.
    fn get_ms(&self) -> u32 {
        self.count.load(Relaxed)
    }

    // To be called only from ISR handler
    fn isr(&self) {
        self.count.fetch_add(1, Relaxed);
    }
}

static mut SYSTEM_TIMER: Option<SystemTimer> = None;

#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct Instant(u32);

#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct Duration(u32);

impl Instant {
    pub fn into_ms(self) -> u32 {
        self.0
    }
}

impl Duration {
    pub fn into_ms(self) -> u32 {
        self.0
    }
}

pub fn initialize(timer: SYST, clock_hz: u32) {
    let timer = SystemTimer::new(timer, clock_hz);
    unsafe { SYSTEM_TIMER.replace(timer) };
}

pub fn get_ms() -> Instant {
    let timer = unsafe { SYSTEM_TIMER.as_ref().unwrap() };
    Instant(timer.get_ms())
}

pub fn blocking_wait_ms(ms: u32) {
    let start = get_ms().into_ms();

    loop {
        let now = get_ms().into_ms();
        let (spent_time_ms, _) = now.overflowing_sub(start);
        if spent_time_ms >= ms {
            break;
        }

        // Enter low power mode
        cortex_m::asm::wfi();
    }
}

pub fn elapsed_since(instant: Instant) -> Duration {
    let (elapsed, _) = get_ms().into_ms().overflowing_sub(instant.into_ms());
    Duration(elapsed)
}

pub(super) fn isr() {
    let timer = unsafe { SYSTEM_TIMER.as_ref().unwrap() };
    timer.isr()
}
