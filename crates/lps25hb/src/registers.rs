pub(crate) const WHO_AM_I: u8 = 0x0F;
pub(crate) const RES_CONF: u8 = 0x10;
pub(crate) const CTRL_REG1: u8 = 0x20;
pub(crate) const CTRL_REG2: u8 = 0x21;
pub(crate) const STATUS_REG: u8 = 0x27;
pub(crate) const PRESS_OUT_XL: u8 = 0x28;
pub(crate) const RPDS_L: u8 = 0x39;

pub(crate) const CTRL_REG1_ACTIVE: u8 = 0b1000_0000;
pub(crate) const CTRL_REG1_ODR_MASK: u8 = 0b0111_0000;
pub(crate) const CTRL_REG1_DIFF_EN: u8 = 0b0000_1000;
pub(crate) const CTRL_REG1_BDU: u8 = 0b0000_0100;
pub(crate) const CTRL_REG2_BOOT: u8 = 0b1000_0000;
pub(crate) const CTRL_REG2_SWRESET: u8 = 0b0000_0100;
pub(crate) const CTRL_REG2_AUTOZERO: u8 = 0b0000_0010;
pub(crate) const CTRL_REG2_ONE_SHOT: u8 = 0b0000_0001;
