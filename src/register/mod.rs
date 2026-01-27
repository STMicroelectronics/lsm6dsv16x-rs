pub mod advanced;
pub mod embedded;
pub mod main;
pub mod sensor_hub;
pub mod spi2;

use super::{BusOperation, DelayNs, Error, Lsm6dsv16x, MemBankFunctions, only_async, only_sync};

use st_mem_bank_macro::mem_bank;

/// Memory bank selection
///
/// Selects the memory bank for operations.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Debug)]
#[mem_bank(Lsm6dsv16x, generics = 2)]
pub enum MemBank {
    /// Main memory bank
    #[main]
    MainMemBank = 0x0,
    /// Embedded function memory bank
    #[state(EmbBank, fn_name = "operate_over_embed")]
    EmbedFuncMemBank = 0x1,
    /// Sensor hub memory bank
    #[state(SensHubBank, fn_name = "operate_over_sensorhub")]
    SensorHubMemBank = 0x2,
}
