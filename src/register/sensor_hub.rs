use crate::Error;
use crate::SensorHubState;
use bitfield_struct::bitfield;
use derive_more::TryFrom;
use embedded_hal::delay::DelayNs;
use st_mem_bank_macro::register;
use st_mems_bus::BusOperation;

#[repr(u8)]
#[derive(Clone, Copy, PartialEq)]
pub enum SnsHubReg {
    SensorHub1 = 0x2,
    SensorHub2 = 0x3,
    SensorHub3 = 0x4,
    SensorHub4 = 0x5,
    SensorHub5 = 0x6,
    SensorHub6 = 0x7,
    SensorHub7 = 0x8,
    SensorHub8 = 0x9,
    SensorHub9 = 0x0A,
    SensorHub10 = 0x0B,
    SensorHub11 = 0x0C,
    SensorHub12 = 0x0D,
    SensorHub13 = 0x0E,
    SensorHub14 = 0x0F,
    SensorHub15 = 0x10,
    SensorHub16 = 0x11,
    SensorHub17 = 0x12,
    SensorHub18 = 0x13,
    MasterConfig = 0x14,
    Slv0Subadd = 0x16,
    Slv0Config = 0x17,
    Slv1Subadd = 0x19,
    Slv1Config = 0x1A,
    Slv2Subadd = 0x1C,
    Slv2Config = 0x1D,
    Slv3Subadd = 0x1F,
    Slv3Config = 0x20,
    DatawriteSlv0 = 0x21,
    StatusMaster = 0x22,
    Slv0Add = 0x15,
    Slv1Add = 0x18,
    Slv2Add = 0x1B,
    Slv3Add = 0x1E,
}

/// SENSOR_HUB_1 (0x02)
///
/// Sensor hub output register 1 (R)
#[register(address = SnsHubReg::SensorHub1, access_type = SensorHubState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct SensorHub1 {
    #[bits(8)]
    pub sensorhub1: u8,
}

/// SENSOR_HUB_2 (0x03)
///
/// Sensor hub output register 2 (R)
#[register(address = SnsHubReg::SensorHub2, access_type = SensorHubState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct SensorHub2 {
    #[bits(8)]
    pub sensorhub2: u8,
}

/// SENSOR_HUB_3 (0x04)
///
/// Sensor hub output register 3 (R)
#[register(address = SnsHubReg::SensorHub3, access_type = SensorHubState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct SensorHub3 {
    #[bits(8)]
    pub sensorhub3: u8,
}

/// SENSOR_HUB_4 (0x05)
///
/// Sensor hub output register 4 (R)
#[register(address = SnsHubReg::SensorHub4, access_type = SensorHubState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct SensorHub4 {
    #[bits(8)]
    pub sensorhub4: u8,
}

/// SENSOR_HUB_5 (0x06)
///
/// Sensor hub output register 5 (R)
#[register(address = SnsHubReg::SensorHub5, access_type = SensorHubState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct SensorHub5 {
    #[bits(8)]
    pub sensorhub5: u8,
}

/// SENSOR_HUB_6 (0x07)
///
/// Sensor hub output register 6 (R)
#[register(address = SnsHubReg::SensorHub6, access_type = SensorHubState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct SensorHub6 {
    #[bits(8)]
    pub sensorhub6: u8,
}

/// SENSOR_HUB_7 (0x08)
///
/// Sensor hub output register 7 (R)
#[register(address = SnsHubReg::SensorHub7, access_type = SensorHubState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct SensorHub7 {
    #[bits(8)]
    pub sensorhub7: u8,
}

/// SENSOR_HUB_8 (0x09)
///
/// Sensor hub output register 8 (R)
#[register(address = SnsHubReg::SensorHub8, access_type = SensorHubState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct SensorHub8 {
    #[bits(8)]
    pub sensorhub8: u8,
}

/// SENSOR_HUB_9 (0x0A)
///
/// Sensor hub output register 9 (R)
#[register(address = SnsHubReg::SensorHub9, access_type = SensorHubState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct SensorHub9 {
    #[bits(8)]
    pub sensorhub9: u8,
}

/// SENSOR_HUB_10 (0x0B)
///
/// Sensor hub output register 10 (R)
#[register(address = SnsHubReg::SensorHub10, access_type = SensorHubState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct SensorHub10 {
    #[bits(8)]
    pub sensorhub10: u8,
}

/// SENSOR_HUB_11 (0x0C)
///
/// Sensor hub output register 11 (R)
#[register(address = SnsHubReg::SensorHub11, access_type = SensorHubState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct SensorHub11 {
    #[bits(8)]
    pub sensorhub11: u8,
}

/// SENSOR_HUB_12 (0x0D)
///
/// Sensor hub output register 12 (R)
#[register(address = SnsHubReg::SensorHub12, access_type = SensorHubState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct SensorHub12 {
    #[bits(8)]
    pub sensorhub12: u8,
}

/// SENSOR_HUB_13 (0x0E)
///
/// Sensor hub output register 13 (R)
#[register(address = SnsHubReg::SensorHub13, access_type = SensorHubState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct SensorHub13 {
    #[bits(8)]
    pub sensorhub13: u8,
}

/// SENSOR_HUB_14 (0x0F)
///
/// Sensor hub output register 14 (R)
#[register(address = SnsHubReg::SensorHub14, access_type = SensorHubState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct SensorHub14 {
    #[bits(8)]
    pub sensorhub14: u8,
}

/// SENSOR_HUB_15 (0x10)
///
/// Sensor hub output register 15 (R)
#[register(address = SnsHubReg::SensorHub15, access_type = SensorHubState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct SensorHub15 {
    #[bits(8)]
    pub sensorhub15: u8,
}

/// SENSOR_HUB_16 (0x11)
///
/// Sensor hub output register 16 (R)
#[register(address = SnsHubReg::SensorHub16, access_type = SensorHubState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct SensorHub16 {
    #[bits(8)]
    pub sensorhub16: u8,
}

/// SENSOR_HUB_17 (0x12)
///
/// Sensor hub output register 17 (R)
#[register(address = SnsHubReg::SensorHub17, access_type = SensorHubState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct SensorHub17 {
    #[bits(8)]
    pub sensorhub17: u8,
}

/// SENSOR_HUB_18 (0x13)
///
/// Sensor hub output register 18 (R)
#[register(address = SnsHubReg::SensorHub18, access_type = SensorHubState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct SensorHub18 {
    #[bits(8)]
    pub sensorhub18: u8,
}

/// MASTER_CONFIG (0x14)
///
/// Master configuration register (R/W)
#[register(address = SnsHubReg::MasterConfig, access_type = SensorHubState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct MasterConfig {
    /// Number of external sensors to be read by the sensor hub (2 bits)
    #[bits(2)]
    pub aux_sens_on: u8,
    /// Enables sensor hub I²C master (1: enabled)
    #[bits(1)]
    pub master_on: u8,
    #[bits(1, access = RO)]
    pub not_used0: u8,
    /// I²C interface pass-through mode (1: enabled)
    #[bits(1)]
    pub pass_through_mode: u8,
    /// Sensor hub trigger signal selection (1: external from INT2 pin)
    #[bits(1)]
    pub start_config: u8,
    /// Slave 0 write operation control (1: write only at first sensor hub cycle)
    #[bits(1)]
    pub write_once: u8,
    /// Resets master logic and output registers (write 1 then 0)
    #[bits(1)]
    pub rst_master_regs: u8,
}

/// SLV0_ADD (0x15)
///
/// I²C slave address of the first external sensor (sensor 0) register (R/W)
#[register(address = SnsHubReg::Slv0Add, access_type = SensorHubState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Slv0Add {
    /// Read/write operation on sensor 0 (1: read operation)
    #[bits(1)]
    pub rw_0: u8,
    /// I²C slave address of sensor 0 (7 bits)
    #[bits(7)]
    pub slave0_add: u8,
}

/// SLV0_SUBADD (0x16)
///
/// Address of register on the first external sensor (sensor 0) (R/W)
#[register(address = SnsHubReg::Slv0Subadd, access_type = SensorHubState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Slv0Subadd {
    /// Register address on sensor 0
    #[bits(8)]
    pub slave0_reg: u8,
}

/// SLV0_CONFIG (0x17)
///
/// First external sensor (sensor 0) configuration and sensor hub settings register (R/W)
#[register(address = SnsHubReg::Slv0Config, access_type = SensorHubState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Slv0Config {
    /// Number of read operations on sensor 0 (3 bits)
    #[bits(3)]
    pub slave0_numop: u8,
    /// Enables FIFO data batching of first slave (1: enabled)
    #[bits(1)]
    pub batch_ext_sens_0_en: u8,
    #[bits(1, access = RO)]
    pub not_used0: u8,
    /// Rate at which the master communicates (3 bits)
    #[bits(3)]
    pub shub_odr: u8,
}

/// SLV1_ADD (0x18)
///
/// I²C slave address of the second external sensor (sensor 1) register (R/W)
#[register(address = SnsHubReg::Slv1Add, access_type = SensorHubState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Slv1Add {
    /// Enables read operation on sensor 1 (1: enabled)
    #[bits(1)]
    pub r_1: u8,
    /// I²C slave address of sensor 1 (7 bits)
    #[bits(7)]
    pub slave1_add: u8,
}

/// SLV1_SUBADD (0x19)
///
/// Address of register on the second external sensor (sensor 1) (R/W)
#[register(address = SnsHubReg::Slv1Subadd, access_type = SensorHubState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Slv1Subadd {
    /// Register address on sensor 1
    #[bits(8)]
    pub slave1_reg: u8,
}

/// SLV1_CONFIG (0x1A)
///
/// Second external sensor (sensor 1) configuration register (R/W)
#[register(address = SnsHubReg::Slv1Config, access_type = SensorHubState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Slv1Config {
    /// Number of read operations on sensor 1 (3 bits)
    #[bits(3)]
    pub slave1_numop: u8,
    /// Enables FIFO data batching of second slave (1: enabled)
    #[bits(1)]
    pub batch_ext_sens_1_en: u8,
    #[bits(4, access = RO)]
    pub not_used0: u8,
}

/// SLV2_ADD (0x1B)
///
/// I²C slave address of the third external sensor (sensor 2) register (R/W)
#[register(address = SnsHubReg::Slv2Add, access_type = SensorHubState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Slv2Add {
    /// Enables read operation on sensor 2 (1: enabled)
    #[bits(1)]
    pub r_2: u8,
    /// I²C slave address of sensor 2 (7 bits)
    #[bits(7)]
    pub slave2_add: u8,
}

/// SLV2_SUBADD (0x1C)
///
/// Address of register on the third external sensor (sensor 2) (R/W)
#[register(address = SnsHubReg::Slv2Subadd, access_type = SensorHubState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Slv2Subadd {
    /// Register address on sensor 2
    #[bits(8)]
    pub slave2_reg: u8,
}

/// SLV2_CONFIG (0x1D)
///
/// Third external sensor (sensor 2) configuration register (R/W)
#[register(address = SnsHubReg::Slv2Config, access_type = SensorHubState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Slv2Config {
    /// Number of read operations on sensor 2 (3 bits)
    #[bits(3)]
    pub slave2_numop: u8,
    /// Enables FIFO data batching of third slave (1: enabled)
    #[bits(1)]
    pub batch_ext_sens_2_en: u8,
    #[bits(4, access = RO)]
    pub not_used0: u8,
}

/// SLV3_ADD (0x1E)
///
/// I²C slave address of the fourth external sensor (sensor 3) register (R/W)
#[register(address = SnsHubReg::Slv3Add, access_type = SensorHubState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Slv3Add {
    /// Enables read operation on sensor 3 (1: enabled)
    #[bits(1)]
    pub r_3: u8,
    /// I²C slave address of sensor 3 (7 bits)
    #[bits(7)]
    pub slave3_add: u8,
}

/// SLV3_SUBADD (0x1F)
///
/// Address of register on the fourth external sensor (sensor 3) (R/W)
#[register(address = SnsHubReg::Slv3Subadd, access_type = SensorHubState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Slv3Subadd {
    /// Register address on sensor 3
    #[bits(8)]
    pub slave3_reg: u8,
}

/// SLV3_CONFIG (0x20)
///
/// Fourth external sensor (sensor 3) configuration register (R/W)
#[register(address = SnsHubReg::Slv3Config, access_type = SensorHubState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Slv3Config {
    /// Number of read operations on sensor 3 (3 bits)
    #[bits(3)]
    pub slave3_numop: u8,
    /// Enables FIFO data batching of fourth slave (1: enabled)
    #[bits(1)]
    pub batch_ext_sens_3_en: u8,
    #[bits(4, access = RO)]
    pub not_used0: u8,
}

/// DATAWRITE_SLV0 (0x21)
///
/// Data to be written into the slave 0 device register (R/W)
#[register(address = SnsHubReg::DatawriteSlv0, access_type = SensorHubState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct DatawriteSlv0 {
    /// Data to be written into slave 0 device
    #[bits(8)]
    pub slave0_dataw: u8,
}

/// STATUS_MASTER (0x22)
///
/// Sensor hub source register (R)
#[register(address = SnsHubReg::StatusMaster, access_type = SensorHubState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct StatusMaster {
    /// Sensor hub communication status (1: communication concluded)
    #[bits(1)]
    pub sens_hub_endop: u8,
    #[bits(2, access = RO)]
    pub not_used0: u8,
    /// Not acknowledge on slave 0 communication (1: NACK occurred)
    #[bits(1)]
    pub slave0_nack: u8,
    /// Not acknowledge on slave 1 communication (1: NACK occurred)
    #[bits(1)]
    pub slave1_nack: u8,
    /// Not acknowledge on slave 2 communication (1: NACK occurred)
    #[bits(1)]
    pub slave2_nack: u8,
    /// Not acknowledge on slave 3 communication (1: NACK occurred)
    #[bits(1)]
    pub slave3_nack: u8,
    /// Write operation on slave 0 performed and completed (1: done)
    #[bits(1)]
    pub wr_once_done: u8,
}

/// Sensor hub slave connected devices
///
/// Defines the number of slaves connected to the sensor hub.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum ShSlaveConnected {
    /// Slave 0 only (default)
    #[default]
    _0 = 0x0,
    /// Slaves 0 and 1
    _01 = 0x1,
    /// Slaves 0, 1, and 2
    _012 = 0x2,
    /// Slaves 0, 1, 2, and 3
    _0123 = 0x3,
}

/// Sensor hub synchronization mode
///
/// Selects the synchronization mode for the sensor hub.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum ShSyncroMode {
    /// Triggered by accelerometer and gyroscope data ready (default)
    #[default]
    ShTrgXlGyDrdy = 0x0,
    /// Triggered by INT2 pin
    ShTrigInt2 = 0x1,
}

/// Sensor hub write mode
///
/// Selects the write mode for the sensor hub.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum ShWriteMode {
    /// Write on each sensor hub cycle (default)
    #[default]
    EachShCycle = 0x0,
    /// Write only on first cycle
    OnlyFirstCycle = 0x1,
}

/// Sensor hub data rate
///
/// Defines the data rate for the sensor hub.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum ShDataRate {
    /// 15 Hz (default)
    #[default]
    _15hz = 0x1,
    /// 30 Hz
    _30hz = 0x2,
    /// 60 Hz
    _60hz = 0x3,
    /// 120 Hz
    _120hz = 0x4,
    /// 240 Hz
    _240hz = 0x5,
    /// 480 Hz
    _480hz = 0x6,
}

#[derive(Clone, Copy, Default)]
pub struct ShCfgWrite {
    pub slv0_add: u8,
    pub slv0_subadd: u8,
    pub slv0_data: u8,
}

#[derive(Clone, Copy, Default)]
pub struct ShCfgRead {
    pub slv_add: u8,
    pub slv_subadd: u8,
    pub slv_len: u8,
}
