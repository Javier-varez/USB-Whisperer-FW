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

use super::app::monotonics;
pub fn get_ms() -> Instant {
    Instant(monotonics::now().ticks() as u32)
}

pub fn elapsed_since(instant: Instant) -> Duration {
    let (elapsed, _) = get_ms().into_ms().overflowing_sub(instant.into_ms());
    Duration(elapsed)
}
