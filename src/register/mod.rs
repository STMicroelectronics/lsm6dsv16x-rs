pub mod advanced;
pub mod embedded;
pub mod main;
pub mod sensor_hub;
pub mod spi2;

use crate::Error;
use crate::Lsm6dsv16x;
use embedded_hal::delay::DelayNs;
use st_mem_bank_macro::mem_bank;
use st_mems_bus::{BusOperation, MemBankFunctions};

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
    #[state(EmbedFuncState, fn_name = "operate_over_embed")]
    EmbedFuncMemBank = 0x1,
    /// Sensor hub memory bank
    #[state(SensorHubState, fn_name = "operate_over_sensorhub")]
    SensorHubMemBank = 0x2,
}
