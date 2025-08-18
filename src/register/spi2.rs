use crate::Error;
use crate::Lsm6dsv16x;
use bitfield_struct::bitfield;
use derive_more::TryFrom;
use embedded_hal::delay::DelayNs;
use st_mem_bank_macro::named_register;
use st_mem_bank_macro::register;
use st_mems_bus::BusOperation;

#[repr(u8)]
#[derive(Clone, Copy, PartialEq)]
pub enum Spi2Reg {
    WhoAmI = 0x0F,
    StatusRegOis = 0x1E,
    OutTempL = 0x20,
    OutTempH = 0x21,
    OutxLGOis = 0x22,
    OutxHGOis = 0x23,
    OutyLGOis = 0x24,
    OutyHGOis = 0x25,
    OutzLGOis = 0x26,
    OutzHGOis = 0x27,
    OutxLAOis = 0x28,
    OutxHAOis = 0x29,
    OutyLAOis = 0x2A,
    OutyHAOis = 0x2B,
    OutzLAOis = 0x2C,
    OutzHAOis = 0x2D,
    HandshakeCtrl = 0x6E,
    IntOis = 0x6F,
    Ctrl1Ois = 0x70,
    Ctrl2Ois = 0x71,
    Ctrl3Ois = 0x72,
}

/// SPI2_WHO_AM_I (0x0F)
///
/// WHO_AM_I register (R), read-only, fixed value 0x70.
#[register(address = Spi2Reg::WhoAmI, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Spi2WhoAmI {
    /// Device identification value.
    #[bits(8)]
    pub id: u8,
}

/// SPI2_STATUS_REG_OIS (0x1E)
///
/// SPI2 status register for OIS (R)
#[register(address = Spi2Reg::StatusRegOis, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Spi2StatusRegOis {
    /// Accelerometer data available (reset when one of the high parts of the output data is read)
    #[bits(1)]
    pub xlda: u8,
    /// Gyroscope data available (reset when one of the high parts of the output data is read)
    #[bits(1)]
    pub gda: u8,
    /// High when the gyroscope output is in the settling phase
    #[bits(1)]
    pub gyro_settling: u8,
    #[bits(5, access = RO)]
    pub not_used0: u8,
}

/// SPI2_OUT_TEMP_L (0x20)
///
/// Temperature data output register low byte (R)
#[register(address = Spi2Reg::OutTempL, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Spi2OutTempL {
    /// Temperature sensor output data (LSByte)
    #[bits(8)]
    pub temp: u8,
}

/// SPI2_OUT_TEMP_H (0x21)
///
/// Temperature data output register high byte (R)
#[register(address = Spi2Reg::OutTempH, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Spi2OutTempH {
    /// Temperature sensor output data (MSByte)
    #[bits(8)]
    pub temp: u8,
}

/// SPI2_OUTX_L_G_OIS - SPI2_OUTZ_H_G_OIS (0x22 - 0x27)
///
/// Angular rate sensor pitch axis (X, Y, Z) angular rate output registers (R)
/// Data according to gyroscope full-scale and ODR (7.68 kHz) settings of OIS gyroscope.
#[named_register(address = Spi2Reg::OutxLGOis, access_type = Lsm6dsv16x, generics = 2)]
pub struct Spi2OutXYZGOis {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

/// SPI2_OUTX_L_A_OIS (0x28)
///
/// Linear acceleration sensor X-axis output register low byte (R)
/// Data according to accelerometer full scale and ODR (7.68 kHz) settings of OIS accelerometer.
#[register(address = Spi2Reg::OutxLAOis, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Spi2OutxLAOis {
    /// Accelerometer OIS chain X-axis linear acceleration output value (LSByte)
    #[bits(8)]
    pub spi2_outx_a_ois: u8,
}

/// SPI2_OUTX_H_A_OIS (0x29)
///
/// Linear acceleration sensor X-axis output register high byte (R)
#[register(address = Spi2Reg::OutxHAOis, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Spi2OutxHAOis {
    /// Accelerometer OIS chain X-axis linear acceleration output value (MSByte)
    #[bits(8)]
    pub spi2_outx_a_ois: u8,
}

/// SPI2_OUTY_L_A_OIS (0x2A)
///
/// Linear acceleration sensor Y-axis output register low byte (R)
#[register(address = Spi2Reg::OutyLAOis, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Spi2OutyLAOis {
    /// Accelerometer OIS chain Y-axis linear acceleration output value (LSByte)
    #[bits(8)]
    pub spi2_outy_a_ois: u8,
}

/// SPI2_OUTY_H_A_OIS (0x2B)
///
/// Linear acceleration sensor Y-axis output register high byte (R)
#[register(address = Spi2Reg::OutyHAOis, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Spi2OutyHAOis {
    /// Accelerometer OIS chain Y-axis linear acceleration output value (MSByte)
    #[bits(8)]
    pub spi2_outy_a_ois: u8,
}

/// SPI2_OUTZ_L_A_OIS (0x2C)
///
/// Linear acceleration sensor Z-axis output register low byte (R)
#[register(address = Spi2Reg::OutzLAOis, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Spi2OutzLAOis {
    /// Accelerometer OIS chain Z-axis linear acceleration output value (LSByte)
    #[bits(8)]
    pub spi2_outz_a_ois: u8,
}

/// SPI2_OUTZ_H_A_OIS (0x2D)
///
/// Linear acceleration sensor Z-axis output register high byte (R)
#[register(address = Spi2Reg::OutzHAOis, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Spi2OutzHAOis {
    /// Accelerometer OIS chain Z-axis linear acceleration output value (MSByte)
    #[bits(8)]
    pub spi2_outz_a_ois: u8,
}

/// SPI2_HANDSHAKE_CTRL (0x6E)
///
/// Control register (SPI2 side) for UI / SPI2 shared registers (R/W)
#[register(address = Spi2Reg::HandshakeCtrl, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Spi2HandshakeCtrl {
    /// Auxiliary SPI (SPI2) interface side acknowledges the handshake.
    /// Set to 1 by device if primary interface is not accessing shared registers.
    #[bits(1)]
    pub spi2_shared_ack: u8,
    /// Used by auxiliary SPI (SPI2) interface master to request access to UI_SPI2_SHARED_0 through UI_SPI2_SHARED_5.
    /// Must be reset after R/W operation is finished.
    #[bits(1)]
    pub spi2_shared_req: u8,
    #[bits(6, access = RO)]
    pub not_used0: u8,
}

/// SPI2_INT_OIS (0x6F)
///
/// OIS interrupt configuration register and self-test setting
/// Writable by auxiliary SPI interface when OIS_CTRL_FROM_UI bit is 0 (SPI2 full-control mode).
/// Read-only when OIS_CTRL_FROM_UI bit is 1 (primary IF full-control mode).
#[register(address = Spi2Reg::IntOis, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Spi2IntOis {
    /// Accelerometer OIS chain self-test selection (2 bits)
    #[bits(2)]
    pub st_xl_ois: u8,
    /// Gyroscope OIS chain self-test selection (2 bits)
    #[bits(2)]
    pub st_g_ois: u8,
    /// Disables OIS chain clamp during self-test.
    /// 0: All OIS chain outputs = 8000h during self-test (default)
    /// 1: OIS chain self-test outputs
    #[bits(1)]
    pub st_ois_clampdis: u8,
    #[bits(1, access = RO)]
    pub not_used0: u8,
    /// Enables/masks OIS data available.
    /// 0: disabled (default)
    /// 1: masks OIS DRDY signals until filter settling ends (accelerometer and gyroscope independently masked)
    #[bits(1)]
    pub drdy_mask_ois: u8,
    /// Enables OIS chain DRDY on INT2 pin.
    /// This setting has priority over all other INT2 settings.
    #[bits(1)]
    pub int2_drdy_ois: u8,
}

/// SPI2_CTRL1_OIS (0x70)
///
/// OIS configuration register
/// Writable by auxiliary SPI interface when OIS_CTRL_FROM_UI bit is 0 (SPI2 full-control mode).
/// Read-only when OIS_CTRL_FROM_UI bit is 1 (primary IF full-control mode).
#[register(address = Spi2Reg::Ctrl1Ois, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Spi2Ctrl1Ois {
    /// Enables auxiliary SPI for reading OIS data.
    /// 0: OIS data read from auxiliary SPI disabled (default)
    /// 1: OIS data read from auxiliary SPI enabled
    #[bits(1)]
    pub spi2_read_en: u8,
    /// Enables gyroscope OIS chain.
    /// 0: disabled (default)
    /// 1: enabled
    #[bits(1)]
    pub ois_g_en: u8,
    /// Enables accelerometer OIS chain.
    /// 0: disabled (default)
    /// 1: enabled
    #[bits(1)]
    pub ois_xl_en: u8,
    #[bits(2, access = RO)]
    pub not_used0: u8,
    /// SPI2 3- or 4-wire interface.
    /// 0: 4-wire SPI2 (default)
    /// 1: 3-wire SPI2
    #[bits(1)]
    pub sim_ois: u8,
    #[bits(2, access = RO)]
    pub not_used1: u8,
}

/// SPI2_CTRL2_OIS (0x71)
///
/// OIS configuration register
/// Writable by auxiliary SPI interface when OIS_CTRL_FROM_UI bit is 0 (SPI2 full-control mode).
/// Read-only when OIS_CTRL_FROM_UI bit is 1 (primary IF full-control mode).
#[register(address = Spi2Reg::Ctrl2Ois, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Spi2Ctrl2Ois {
    /// Gyroscope OIS full-scale selection (3 bits):
    /// 000: ±125 dps (default)
    /// 001: ±250 dps
    /// 010: ±500 dps
    /// 011: ±1000 dps
    /// 100: ±2000 dps
    /// 101-111: reserved
    #[bits(3)]
    pub fs_g_ois: u8,
    /// Gyroscope OIS digital LPF1 filter bandwidth selection (2 bits)
    #[bits(2)]
    pub lpf1_g_ois_bw: u8,
    #[bits(3, access = RO)]
    pub not_used0: u8,
}

/// SPI2_CTRL3_OIS (0x72)
///
/// OIS configuration register
/// Writable by auxiliary SPI interface when OIS_CTRL_FROM_UI bit is 0 (SPI2 full-control mode).
/// Read-only when OIS_CTRL_FROM_UI bit is 1 (primary IF full-control mode).
#[register(address = Spi2Reg::Ctrl3Ois, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Spi2Ctrl3Ois {
    /// Accelerometer OIS channel full-scale selection (2 bits):
    /// 00: ±2 g (default)
    /// 01: ±4 g
    /// 10: ±8 g
    /// 11: ±16 g
    #[bits(2)]
    pub fs_xl_ois: u8,
    #[bits(1, access = RO)]
    pub not_used0: u8,
    /// Accelerometer OIS channel bandwidth selection (3 bits)
    #[bits(3)]
    pub lpf_xl_ois_bw: u8,
    #[bits(2, access = RO)]
    pub not_used1: u8,
}

/// OIS gyroscope self-test modes
///
/// Defines the self-test modes for the OIS gyroscope.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum OisGySelfTest {
    /// Self-test disabled (default)
    #[default]
    Disable = 0x0,
    /// Positive self-test
    Positive = 0x1,
    /// Negative self-test
    Negative = 0x2,
    /// Clamp positive self-test
    ClampPos = 0x5,
    /// Clamp negative self-test
    ClampNeg = 0x6,
}

#[derive(Clone, Copy, Default)]
pub struct FiltOisSettlingMask {
    pub ois_drdy: u8,
}
