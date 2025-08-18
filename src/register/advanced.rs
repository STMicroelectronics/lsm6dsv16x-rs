use crate::Error;
use crate::Lsm6dsv16x;
use bitfield_struct::bitfield;
use derive_more::TryFrom;
use embedded_hal::delay::DelayNs;
use st_mem_bank_macro::{MultiRegister, adv_register};
use st_mems_bus::{BusOperation, EmbAdvFunctions};

#[repr(u16)]
#[derive(Clone, Copy, PartialEq)]
pub enum AdvPage {
    _0 = 0x000,
    _1 = 0x100,
    _2 = 0x200,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq)]
pub enum EmbAdv0Reg {
    SflpGameGbiasxL = 0x6E,
    SflpGameGbiasxH = 0x6F,
    SflpGameGbiasyL = 0x70,
    SflpGameGbiasyH = 0x71,
    SflpGameGbiaszL = 0x72,
    SflpGameGbiaszH = 0x73,
    FsmStartAddL = 0x7E,
    FsmStartAddH = 0x7F,
    FsmExtSensitivityL = 0xBA,
    FsmExtSensitivityH = 0xBB,
    FsmExtOffxL = 0xC0,
    FsmExtOffxH = 0xC1,
    FsmExtOffyL = 0xC2,
    FsmExtOffyH = 0xC3,
    FsmExtOffzL = 0xC4,
    FsmExtOffzH = 0xC5,
    FsmExtMatrixXxL = 0xC6,
    FsmExtMatrixXxH = 0xC7,
    FsmExtMatrixXyL = 0xC8,
    FsmExtMatrixXyH = 0xC9,
    FsmExtMatrixXzL = 0xCA,
    FsmExtMatrixXzH = 0xCB,
    FsmExtMatrixYyL = 0xCC,
    FsmExtMatrixYyH = 0xCD,
    FsmExtMatrixYzL = 0xCE,
    FsmExtMatrixYzH = 0xCF,
    FsmExtMatrixZzL = 0xD0,
    FsmExtMatrixZzH = 0xD1,
    ExtCfgA = 0xD4,
    ExtCfgB = 0xD5,
}
#[repr(u8)]
#[derive(Clone, Copy, PartialEq)]
pub enum EmbAdv1Reg {
    FsmLcTimeoutL = 0x7A,
    FsmLcTimeoutH = 0x7B,
    FsmPrograms = 0x7C,
    PedoCmdReg = 0x83,
    PedoDebStepsConf = 0x84,
    PedoScDeltatL = 0xD0,
    PedoScDeltatH = 0xD1,
    MlcExtSensitivityL = 0xE8,
    MlcExtSensitivityH = 0xE9,
}
#[repr(u8)]
#[derive(Clone, Copy, PartialEq)]
pub enum EmbAdv2Reg {
    ExtFormat = 0x00,
    Ext3byteSensitivityL = 0x02,
    Ext3byteSensitivityH = 0x03,
    Ext3byteOffsetXl = 0x06,
    Ext3byteOffsetL = 0x07,
    Ext3byteOffsetH = 0x08,
}

/// SFLP_GAME_GBIASX_L - SFLP_GAME_GBIASZ_H (0x6E - 0x73)
///
/// SFLP game algorithm (X, Y, Z)-axis gyroscope bias registers (R/W)
#[adv_register(base_address = AdvPage::_0, address = EmbAdv0Reg::SflpGameGbiasxL, access_type = Lsm6dsv16x, generics = 2)]
pub struct SflpGameGbiasXYZ(pub [u16; 3]);

/// FSM_START_ADD_L - FSM_START_ADD_H (0x7E - 0x7F)
///
/// FSM start address (R/W)
///
/// First available address is 0x35C.
#[adv_register(base_address = AdvPage::_0, address = EmbAdv0Reg::FsmStartAddL, access_type = Lsm6dsv16x, generics = 2)]
pub struct FsmStartAdd(pub u16);

/// FSM_EXT_SENSITIVITY_L - FSM_EXT_SENSITIVITY_H (0xBA - 0xBB)
///
/// External sensor sensitivity value for the finite state machine (R/W)
#[adv_register(base_address = AdvPage::_0, address = EmbAdv0Reg::FsmExtSensitivityL, access_type = Lsm6dsv16x, generics = 2)]
pub struct FsmExtSensitivity(pub u16);

/// FSM_EXT_OFFX_L - FSM_EXT_OFFZ_H (0xC0 - 0xC5)
///
/// External sensor (X, Y, Z)-axis offset (R/W)
#[adv_register(base_address = AdvPage::_0, address = EmbAdv0Reg::FsmExtOffxL, access_type = Lsm6dsv16x, generics = 2)]
pub struct FsmExtOffXYZ(pub [u16; 3]);

/// FSM_EXT_MATRIX (0xC6 - 0XD1)
///
/// Data organization:
/// XX XY XZ YY YZ ZZ
///
/// Use `FsmExtSensMatrix to convert the data
///
/// External sensor transformation matrix coefficient (R/W)
#[adv_register(base_address = AdvPage::_0, address = EmbAdv0Reg::FsmExtMatrixXxL, access_type = Lsm6dsv16x, generics = 2)]
pub struct FsmExtMatrix(pub [u8; 12]);

/// EXT_CFG_A (0xD4)
///
/// External sensor coordinates (Z and Y axes) rotation register (R/W)
#[adv_register(base_address = AdvPage::_0, address = EmbAdv0Reg::ExtCfgA, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct ExtCfgA {
    /// External sensor Z-axis coordinates rotation (3 bits)
    #[bits(3)]
    pub ext_z_axis: u8,
    #[bits(1, access = RO)]
    pub not_used0: u8,
    /// External sensor Y-axis coordinates rotation (3 bits)
    #[bits(3)]
    pub ext_y_axis: u8,
    #[bits(1, access = RO)]
    pub not_used1: u8,
}

/// EXT_CFG_B (0xD5)
///
/// External sensor coordinates (X-axis) rotation register (R/W)
#[adv_register(base_address = AdvPage::_0, address = EmbAdv0Reg::ExtCfgB, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct ExtCfgB {
    /// External sensor X-axis coordinates rotation (3 bits)
    #[bits(3)]
    pub ext_x_axis: u8,
    #[bits(5, access = RO)]
    pub not_used0: u8,
}

/// FSM_LC_TIMEOUT_L - FSM_LC_TIMEOUT_H (0x7A - 0x7B)
///
/// FSM long counter timeout register (R/W)
///
/// The long counter timeout value is an unsigned 16-bit integer. When the long counter reaches this value,
/// the FSM generates an interrupt.
#[adv_register(base_address = AdvPage::_1, address = EmbAdv1Reg::FsmLcTimeoutL, access_type = Lsm6dsv16x, generics = 2)]
pub struct FsmLcTimeout(pub u16);

/// FSM_PROGRAMS (0x7C)
///
/// FSM number of programs register (R/W)
///
/// Number of FSM programs; must be less than or equal to 8.
#[adv_register(base_address = AdvPage::_1, address = EmbAdv1Reg::FsmPrograms, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct FsmPrograms {
    /// Number of FSM programs
    #[bits(8)]
    pub fsm_n_prog: u8,
}

/// PEDO_CMD_REG (0x83)
///
/// Pedometer configuration register (R/W)
#[adv_register(base_address = AdvPage::_1, address = EmbAdv1Reg::PedoCmdReg, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct PedoCmdReg {
    #[bits(2, access = RO)]
    pub not_used0: u8,
    /// Enables the false-positive rejection feature
    #[bits(1)]
    pub fp_rejection_en: u8,
    /// Set when user wants to generate interrupt only on count overflow event
    #[bits(1)]
    pub carry_count_en: u8,
    #[bits(4, access = RO)]
    pub not_used1: u8,
}

/// PEDO_DEB_STEPS_CONF (0x84)
///
/// Pedometer debounce configuration register (R/W)
///
/// Minimum number of steps to increment the step counter (debounce).
#[adv_register(base_address = AdvPage::_1, address = EmbAdv1Reg::PedoDebStepsConf, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct PedoDebStepsConf {
    /// Debounce threshold
    #[bits(8)]
    pub deb_step: u8,
}

/// PEDO_SC_DELTAT (0xD0 - 0xD1)
///
/// Time period register low byte for step detection on delta time (R/W)
/// Time period 2 bytes (1LSB = 6.4 ms)
#[adv_register(base_address = AdvPage::_1, address = EmbAdv1Reg::PedoScDeltatL, access_type = Lsm6dsv16x, generics = 2)]
pub struct PedoScDeltaT(pub u16);

/// MLC_EXT_SENSITIVITY_L - MLC_EXT_SENSITIVITY_H (0xE8 - 0xE9)
///
/// External sensor sensitivity value registers for the machine learning core (R/W)
#[adv_register(base_address = AdvPage::_1, address = EmbAdv1Reg::MlcExtSensitivityL, access_type = Lsm6dsv16x, generics = 2)]
pub struct MlcExtSensitivity(pub u16);

/// EXT_FORMAT (0x00)
///
/// AH / Qvar / external sensor data format register (R/W)
#[adv_register(base_address = AdvPage::_2, address = EmbAdv2Reg::ExtFormat, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct ExtFormat {
    #[bits(2, access = RO)]
    pub not_used0: u8,
    /// Selects the format of AH / Qvar / external sensor data for FSM and MLC processing (0: 2-byte format; 1: 3-byte format)
    #[bits(1)]
    pub ext_format_sel: u8,
    #[bits(5, access = RO)]
    pub not_used1: u8,
}

/// EXT_3BYTE_SENSITIVITY_L (0x02)
///
/// External sensor (3-byte output data) sensitivity value register low byte (R/W)
#[adv_register(base_address = AdvPage::_2, address = EmbAdv2Reg::Ext3byteSensitivityL, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Ext3byteSensitivityL {
    /// External sensor sensitivity low byte
    #[bits(8)]
    pub ext_3byte_s: u8,
}

/// EXT_3BYTE_SENSITIVITY_H (0x03)
///
/// External sensor (3-byte output data) sensitivity value register high byte (R/W)
#[adv_register(base_address = AdvPage::_2, address = EmbAdv2Reg::Ext3byteSensitivityH, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Ext3byteSensitivityH {
    /// External sensor sensitivity high byte
    #[bits(8)]
    pub ext_3byte_s: u8,
}

/// EXT_3BYTE_OFFSET_XL (0x06)
///
/// External sensor (3-byte output data) offset value register low byte (R/W)
#[adv_register(base_address = AdvPage::_2, address = EmbAdv2Reg::Ext3byteOffsetXl, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Ext3byteOffsetXl {
    /// External sensor offset low byte
    #[bits(8)]
    pub ext_3byte_off: u8,
}

/// EXT_3BYTE_OFFSET_L (0x07)
///
/// External sensor (3-byte output data) offset value register mid byte (R/W)
#[adv_register(base_address = AdvPage::_2, address = EmbAdv2Reg::Ext3byteOffsetL, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Ext3byteOffsetL {
    /// External sensor offset mid byte
    #[bits(8)]
    pub ext_3byte_off: u8,
}

/// EXT_3BYTE_OFFSET_H (0x08)
///
/// External sensor (3-byte output data) offset value register high byte (R/W)
#[adv_register(base_address = AdvPage::_2, address = EmbAdv2Reg::Ext3byteOffsetH, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Ext3byteOffsetH {
    /// External sensor offset high byte
    #[bits(8)]
    pub ext_3byte_off: u8,
}

/// FSM external sensor Z orientation
///
/// Selects the orientation mapping of the external sensor Z axis.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum FsmExtSensZOrient {
    /// Z equals Y (default)
    #[default]
    ZEqY = 0x0,
    /// Z equals -Y
    ZEqMinY = 0x1,
    /// Z equals X
    ZEqX = 0x2,
    /// Z equals -X
    ZEqMinX = 0x3,
    /// Z equals -Z
    ZEqMinZ = 0x4,
    /// Z equals Z
    ZEqZ = 0x5,
}

/// FSM external sensor Y orientation
///
/// Defines the mapping of the external sensor Y axis.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum FsmExtSensYOrient {
    /// Y equals Y (default)
    #[default]
    YEqY = 0x0,
    /// Y equals -Y
    YEqMinY = 0x1,
    /// Y equals X
    YEqX = 0x2,
    /// Y equals -X
    YEqMinX = 0x3,
    /// Y equals -Z
    YEqMinZ = 0x4,
    /// Y equals Z
    YEqZ = 0x5,
}

/// FSM external sensor X orientation
///
/// Defines the mapping of the external sensor X axis.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum FsmExtSensXOrient {
    /// X equals Y (default)
    #[default]
    XEqY = 0x0,
    /// X equals -Y
    XEqMinY = 0x1,
    /// X equals X
    XEqX = 0x2,
    /// X equals -X
    XEqMinX = 0x3,
    /// X equals -Z
    XEqMinZ = 0x4,
    /// X equals Z
    XEqZ = 0x5,
}

#[derive(Clone, Copy, Default)]
pub struct XlFsmExtSensOffset {
    pub z: u16,
    pub y: u16,
    pub x: u16,
}

#[derive(Clone, Copy, Default, MultiRegister)]
pub struct XlFsmExtSensMatrix {
    pub xx: u16,
    pub xy: u16,
    pub xz: u16,
    pub yy: u16,
    pub yz: u16,
    pub zz: u16,
}

#[derive(Clone, Copy, Default)]
pub struct SflpGbias {
    pub gbias_x: f32,
    pub gbias_y: f32,
    pub gbias_z: f32,
}
