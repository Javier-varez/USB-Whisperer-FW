pub mod device_id {
    pub const ADDR: u8 = 0x01;
}

pub mod reset {
    pub const ADDR: u8 = 0x0C;
    pub const SW_RESET: u8 = 0x01;
    pub const PD_RESET: u8 = 0x02;
}

pub mod switches0 {
    pub const ADDR: u8 = 0x02;
    pub const CC2_PU_EN: u8 = 1 << 7;
    pub const CC1_PU_EN: u8 = 1 << 6;
    pub const CC2_VCONN_EN: u8 = 1 << 5;
    pub const CC1_VCONN_EN: u8 = 1 << 4;
    pub const MEAS_CC2: u8 = 1 << 3;
    pub const MEAS_CC1: u8 = 1 << 2;
    pub const CC2_PD_EN: u8 = 1 << 1;
    pub const CC1_PD_EN: u8 = 1 << 0;
}

pub mod switches1 {
    pub const ADDR: u8 = 0x03;
    pub const AUTO_CRC: u8 = 1 << 2;
    pub const SPECREV_MASK: u8 = 0x03 << 5;
}

pub mod mask {
    pub const ADDR: u8 = 0x0A;
    pub const MASK_BC_LVL: u8 = 0x01;
}

pub mod control0 {
    pub const ADDR: u8 = 0x06;
    pub const INT_MASK: u8 = 1 << 5;
}

pub mod control1 {
    pub const ADDR: u8 = 0x07;
    pub const ENSOP1DB: u8 = 1 << 5;
    pub const ENSOP2DB: u8 = 1 << 6;
    pub const RX_FLUSH: u8 = 1 << 2;
}

pub mod control2 {
    pub const ADDR: u8 = 0x08;
    pub const TOGGLE: u8 = 1 << 0;
}

pub mod control3 {
    pub const ADDR: u8 = 0x09;
    pub const AUTO_RETRY: u8 = 0x01;
    pub const N_RETRIES_MASK: u8 = 0x06;
    pub const N_RETRIES_OFFSET: u8 = 1;
}

pub mod power {
    pub const ADDR: u8 = 0x0B;
    pub const PWR_ALL_MASK: u8 = 0x0F;
}

pub mod measure {
    pub const ADDR: u8 = 0x04;
    pub const MDAC_MASK: u8 = 0x3F;
    pub const MDAC_RESOLUTION_MV: u32 = 42;

    pub const fn mdac_threshold_from_mv(mv: u32) -> u8 {
        ((mv / MDAC_RESOLUTION_MV) & MDAC_MASK as u32) as u8
    }
}

pub mod status0 {
    pub const ADDR: u8 = 0x40;
    pub const COMP: u8 = 1 << 5;
}
