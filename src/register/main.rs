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
pub enum Reg {
    FuncCfgAccess = 0x1,
    PinCtrl = 0x2,
    IfCfg = 0x3,
    OdrTrigCfg = 0x6,
    FifoCtrl1 = 0x7,
    FifoCtrl2 = 0x8,
    FifoCtrl3 = 0x9,
    FifoCtrl4 = 0x0A,
    CounterBdrReg1 = 0x0B,
    CounterBdrReg2 = 0x0C,
    Int1Ctrl = 0x0D,
    Int2Ctrl = 0x0E,
    WhoAmI = 0x0F,
    Ctrl1 = 0x10,
    Ctrl2 = 0x11,
    Ctrl3 = 0x12,
    Ctrl4 = 0x13,
    Ctrl5 = 0x14,
    Ctrl6 = 0x15,
    Ctrl7 = 0x16,
    Ctrl8 = 0x17,
    Ctrl9 = 0x18,
    Ctrl10 = 0x19,
    CtrlStatus = 0x1A,
    FifoStatus1 = 0x1B,
    FifoStatus2 = 0x1C,
    AllIntSrc = 0x1D,
    StatusReg = 0x1E,
    OutTempL = 0x20,
    OutTempH = 0x21,
    OutxLG = 0x22,
    OutxHG = 0x23,
    OutyLG = 0x24,
    OutyHG = 0x25,
    OutzLG = 0x26,
    OutzHG = 0x27,
    OutxLA = 0x28,
    OutxHA = 0x29,
    OutyLA = 0x2A,
    OutyHA = 0x2B,
    OutzLA = 0x2C,
    OutzHA = 0x2D,
    UiOutxLGOisEis = 0x2E,
    UiOutxHGOisEis = 0x2F,
    UiOutyLGOisEis = 0x30,
    UiOutyHGOisEis = 0x31,
    UiOutzLGOisEis = 0x32,
    UiOutzHGOisEis = 0x33,
    UiOutxLAOisDualc = 0x34,
    UiOutxHAOisDualc = 0x35,
    UiOutyLAOisDualc = 0x36,
    UiOutyHAOisDualc = 0x37,
    UiOutzLAOisDualc = 0x38,
    UiOutzHAOisDualc = 0x39,
    AhQvarOutL = 0x3A,
    AhQvarOutH = 0x3B,
    Timestamp0 = 0x40,
    Timestamp1 = 0x41,
    Timestamp2 = 0x42,
    Timestamp3 = 0x43,
    UiStatusRegOis = 0x44,
    WakeUpSrc = 0x45,
    TapSrc = 0x46,
    D6dSrc = 0x47,
    StatusMasterMainPage = 0x48,
    EmbFuncStatusMainPage = 0x49,
    FsmStatusMainPage = 0x4A,
    MlcStatusMainPage = 0x4B,
    InternalFreq = 0x4F,
    FunctionsEnable = 0x50,
    Den = 0x51,
    InactivityDur = 0x54,
    InactivityThs = 0x55,
    TapCfg0 = 0x56,
    TapCfg1 = 0x57,
    TapCfg2 = 0x58,
    TapThs6d = 0x59,
    TapDur = 0x5A,
    WakeUpThs = 0x5B,
    WakeUpDur = 0x5C,
    FreeFall = 0x5D,
    Md1Cfg = 0x5E,
    Md2Cfg = 0x5F,
    HaodrCfg = 0x62,
    EmbFuncCfg = 0x63,
    UiHandshakeCtrl = 0x64,
    UiSpi2Shared0 = 0x65,
    UiSpi2Shared1 = 0x66,
    UiSpi2Shared2 = 0x67,
    UiSpi2Shared3 = 0x68,
    UiSpi2Shared4 = 0x69,
    UiSpi2Shared5 = 0x6A,
    CtrlEis = 0x6B,
    UiIntOis = 0x6F,
    UiCtrl1Ois = 0x70,
    UiCtrl2Ois = 0x71,
    UiCtrl3Ois = 0x72,
    XOfsUsr = 0x73,
    YOfsUsr = 0x74,
    ZOfsUsr = 0x75,
    FifoDataOutTag = 0x78,
    FifoDataOutXL = 0x79,
    FifoDataOutXH = 0x7A,
    FifoDataOutYL = 0x7B,
    FifoDataOutYH = 0x7C,
    FifoDataOutZL = 0x7D,
    FifoDataOutZH = 0x7E,
}

/// CTRL1 (0x10)
///
/// Accelerometer control register 1 (R/W)
#[register(address = Reg::Ctrl1, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Ctrl1 {
    /// Accelerometer output data rate selection (4 bits).
    /// See Table 52 for ODR values.
    #[bits(4)]
    pub odr_xl: u8,
    /// Accelerometer operating mode selection (3 bits).
    /// 000: high-performance mode (default);
    /// 001: high-accuracy ODR mode;
    /// 010: reserved;
    /// 011: ODR-triggered mode;
    /// 100: low-power mode 1;
    /// 101: low-power mode 2;
    /// 110: low-power mode 3;
    /// 111: normal mode.
    #[bits(3)]
    pub op_mode_xl: u8,
    #[bits(1, access = RO)]
    pub not_used0: u8,
}

/// FUNC_CFG_ACCESS (0x01)
///
/// Enable embedded functions register (R/W)
#[register(address = Reg::FuncCfgAccess, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct FuncCfgAccess {
    /// Enables full control of OIS configurations from the primary interface.
    /// (0: OIS chain full control from primary interface disabled;
    /// 1: OIS chain full control from primary interface enabled)
    #[bits(1)]
    pub ois_ctrl_from_ui: u8,
    /// Resets the control registers of SPI2 from the primary interface.
    /// This bit must be set to 1 and then back to 0 (not automatically cleared).
    #[bits(1)]
    pub spi2_reset: u8,
    /// Global reset of the device.
    #[bits(1)]
    pub sw_por: u8,
    /// Enables the control of the CTRL registers to FSM (FSM can change some configurations autonomously).
    /// Default: 0 (disabled)
    #[bits(1)]
    pub fsm_wr_ctrl_en: u8,
    #[bits(2, access = RO)]
    pub not_used0: u8,
    /// Enables access to the sensor hub (I²C master) configuration registers.
    /// Default: 0 (disabled)
    #[bits(1)]
    pub shub_reg_access: u8,
    /// Enables access to the embedded functions configuration registers.
    /// Default: 0 (disabled)
    #[bits(1)]
    pub emb_func_reg_access: u8,
}

/// PIN_CTRL (0x02)
///
/// SDO, OCS_Aux, SDO_Aux pins pull-up register (R/W).
/// This register is not reset during software reset.
#[register(address = Reg::PinCtrl, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct PinCtrl {
    #[bits(5, access = RO)]
    pub not_used0: u8,
    /// Selects the action the device performs after "reset whole chip" I3C pattern.
    /// Default: 1 (global reset (POR reset))
    #[bits(1)]
    pub ibhr_por_en: u8,
    /// Enables pull-up on SDO pin.
    /// Default: 0 (pull-up disconnected)
    #[bits(1)]
    pub sdo_pu_en: u8,
    /// Disables pull-up on both OCS_Aux and SDO_Aux pins (for mode 1 and mode 2).
    /// Default: 0 (pull-up enabled)
    #[bits(1)]
    pub ois_pu_dis: u8,
}

/// IF_CFG (0x03)
///
/// Interface configuration register (R/W).
/// This register is not reset during software reset.
#[register(address = Reg::IfCfg, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct IfCfg {
    /// Disables I²C and MIPI I3C interfaces.
    /// Default: 0 (SPI, I²C and MIPI I3C interfaces enabled)
    #[bits(1)]
    pub i2c_i3c_disable: u8,
    #[bits(1, access = RO)]
    pub not_used0: u8,
    /// SPI serial interface mode selection.
    /// Default: 0 (4-wire interface)
    #[bits(1)]
    pub sim: u8,
    /// Push-pull/open-drain selection on INT1 and INT2 pins.
    /// Default: 0 (push-pull mode)
    #[bits(1)]
    pub pp_od: u8,
    /// Interrupt activation level.
    /// Default: 0 (interrupt output pins active high)
    #[bits(1)]
    pub h_lactive: u8,
    /// Enables anti-spike filters.
    /// Default: 0 (anti-spike filters managed by protocol)
    #[bits(1)]
    pub asf_ctrl: u8,
    /// Enables master I²C pull-up.
    /// Default: 0 (internal pull-up on auxiliary I²C line disabled)
    #[bits(1)]
    pub shub_pu_en: u8,
    /// Enables pull-up on SDA pin.
    /// Default: 0 (SDA pin pull-up disconnected)
    #[bits(1)]
    pub sda_pu_en: u8,
}

/// ODR_TRIG_CFG (0x06)
///
/// ODR-triggered mode configuration register (R/W)
#[register(address = Reg::OdrTrigCfg, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct OdrTrigCfg {
    /// When ODR-triggered mode is set, defines the number of data generated in the reference period.
    /// Allowed values: 0 (default) or 4 to 255.
    #[bits(8)]
    pub odr_trig_nodr: u8,
}

/// FIFO_CTRL1 (0x07)
///
/// FIFO control register 1 (R/W)
#[register(address = Reg::FifoCtrl1, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct FifoCtrl1 {
    /// FIFO watermark threshold: 1 LSB = TAG (1 byte) + 1 sensor (6 bytes) written in FIFO.
    /// Watermark flag rises when FIFO bytes >= threshold.
    #[bits(8)]
    pub wtm: u8,
}

/// FIFO_CTRL2 (0x08)
///
/// FIFO control register 2 (R/W)
#[register(address = Reg::FifoCtrl2, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct FifoCtrl2 {
    /// When dual-channel mode is enabled, enables FSM-triggered batching in FIFO of accelerometer channel 2.
    /// Default: 0 (disabled)
    #[bits(1)]
    pub xl_dualc_batch_from_fsm: u8,
    /// Configures compression algorithm to write uncompressed data at each rate.
    /// 0: uncompressed data writing not forced (default);
    /// 1: uncompressed data every 8 batch data rate;
    /// 2: every 16 batch data rate;
    /// 3: every 32 batch data rate.
    #[bits(2)]
    pub uncompr_rate: u8,
    #[bits(1, access = RO)]
    pub not_used0: u8,
    /// Enables ODR CHANGE virtual sensor to be batched in FIFO.
    /// Default: 0 (not batched)
    #[bits(1)]
    pub odr_chg_en: u8,
    #[bits(1, access = RO)]
    pub not_used1: u8,
    /// Enables/disables compression algorithm runtime.
    /// Default: 0 (disabled)
    #[bits(1)]
    pub fifo_compr_rt_en: u8,
    /// Sensing chain FIFO stop values memorization at threshold level.
    /// Default: 0 (FIFO depth not limited)
    #[bits(1)]
    pub stop_on_wtm: u8,
}

/// FIFO_CTRL3 (0x09)
///
/// FIFO control register 3 (R/W)
#[register(address = Reg::FifoCtrl3, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct FifoCtrl3 {
    /// Selects batch data rate (write frequency in FIFO) for accelerometer data (4 bits).
    /// 0000: accelerometer not batched in FIFO (default);
    /// 0001: 1.875 Hz; 0010: 7.5 Hz; 0011: 15 Hz; 0100: 30 Hz; 0101: 60 Hz; 0110: 120 Hz; 0111: 240 Hz;
    /// 1000: 480 Hz; 1001: 960 Hz; 1010: 1.92 kHz; 1011: 3.84 kHz; 1100: 7.68 kHz; 1101-1111: reserved.
    #[bits(4)]
    pub bdr_xl: u8,
    /// Selects batch data rate (write frequency in FIFO) for gyroscope data (4 bits).
    /// Same encoding as accelerometer batch data rate.
    #[bits(4)]
    pub bdr_gy: u8,
}

/// FIFO_CTRL4 (0x0A)
///
/// FIFO control register 4 (R/W)
#[register(address = Reg::FifoCtrl4, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct FifoCtrl4 {
    /// FIFO mode selection (3 bits):
    /// 000: bypass mode (FIFO disabled, default);
    /// 001: FIFO mode (stops collecting data when FIFO is full);
    /// 010: continuous WTM-to-full mode;
    /// 011: continuous-to-FIFO mode;
    /// 100: bypass-to-continuous mode;
    /// 101: reserved;
    /// 110: continuous mode (new sample overwrites older one if FIFO full);
    /// 111: bypass-to-FIFO mode.
    #[bits(3)]
    pub fifo_mode: u8,
    /// Enables FIFO batching of enhanced EIS gyroscope output values.
    /// Default: 0 (disabled)
    #[bits(1)]
    pub g_eis_fifo_en: u8,
    /// Selects batch data rate for temperature data (2 bits):
    /// 00: temperature not batched in FIFO (default);
    /// 01: 1.875 Hz; 10: 15 Hz; 11: 60 Hz.
    #[bits(2)]
    pub odr_t_batch: u8,
    /// Selects decimation for timestamp batching in FIFO (2 bits):
    /// 00: timestamp not batched (default);
    /// 01: decimation 1: max(BDR_XL, BDR_GY);
    /// 10: decimation 8: max(BDR_XL, BDR_GY)/8;
    /// 11: decimation 32: max(BDR_XL, BDR_GY)/32.
    #[bits(2)]
    pub dec_ts_batch: u8,
}

/// COUNTER_BDR_REG1 - COUNTER_BDR_REG2 (0x0B - 0x0C)
///
/// Counter batch data rate register 1 and 2 (R/W)
#[register(address = Reg::CounterBdrReg1, access_type = Lsm6dsv16x, generics = 2, override_type = u16, order = Inverse)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u16, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u16, order = Lsb))]
pub struct CounterBdrReg {
    /// Threshold bits [9:0] for internal counter of batch events.
    #[bits(10)]
    pub cnt_bdr_th: u16,
    #[bits(3, access = RO)]
    pub not_used0: u8,
    /// Selects the trigger for the internal counter of batch events (2 bits):
    /// 00: accelerometer batch event;
    /// 01: gyroscope batch event;
    /// 10-11: gyroscope EIS batch event.
    #[bits(2)]
    pub trig_counter_bdr: u8,
    #[bits(1, access = RO)]
    pub not_used1: u8,
}

/// INT1_CTRL (0x0D)
///
/// INT1 pin control register (R/W)
/// Each bit enables a signal to be carried over INT1 pin.
#[register(address = Reg::Int1Ctrl, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Int1Ctrl {
    /// Enables accelerometer data-ready interrupt on INT1 pin.
    #[bits(1)]
    pub int1_drdy_xl: u8,
    /// Enables gyroscope data-ready interrupt on INT1 pin.
    #[bits(1)]
    pub int1_drdy_g: u8,
    #[bits(1, access = RO)]
    pub not_used0: u8,
    /// Enables FIFO threshold interrupt on INT1 pin.
    #[bits(1)]
    pub int1_fifo_th: u8,
    /// Enables FIFO overrun interrupt on INT1 pin.
    #[bits(1)]
    pub int1_fifo_ovr: u8,
    /// Enables FIFO full flag interrupt on INT1 pin.
    #[bits(1)]
    pub int1_fifo_full: u8,
    /// Enables COUNTER_BDR_IA interrupt on INT1 pin.
    #[bits(1)]
    pub int1_cnt_bdr: u8,
    #[bits(1, access = RO)]
    pub not_used1: u8,
}

/// INT2_CTRL (0x0E)
///
/// INT2 pin control register (R/W)
/// Each bit enables a signal to be carried over INT2 pin.
#[register(address = Reg::Int2Ctrl, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Int2Ctrl {
    /// Enables accelerometer data-ready interrupt on INT2 pin.
    #[bits(1)]
    pub int2_drdy_xl: u8,
    /// Enables gyroscope data-ready interrupt on INT2 pin.
    #[bits(1)]
    pub int2_drdy_g: u8,
    /// Enables gyroscope EIS data-ready interrupt on INT2 pin.
    #[bits(1)]
    pub int2_drdy_g_eis: u8,
    /// Enables FIFO threshold interrupt on INT2 pin.
    #[bits(1)]
    pub int2_fifo_th: u8,
    /// Enables FIFO overrun interrupt on INT2 pin.
    #[bits(1)]
    pub int2_fifo_ovr: u8,
    /// Enables FIFO full flag interrupt on INT2 pin.
    #[bits(1)]
    pub int2_fifo_full: u8,
    /// Enables COUNTER_BDR_IA interrupt on INT2 pin.
    #[bits(1)]
    pub int2_cnt_bdr: u8,
    /// Enables routing the embedded functions end of operations signal to INT2 pin.
    #[bits(1)]
    pub int2_emb_func_endop: u8,
}

/// WHO_AM_I (0x0F)
///
/// WHO_AM_I register (R), read-only, fixed value 0x70.
#[register(address = Reg::WhoAmI, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct WhoAmI {
    /// Device identification value.
    #[bits(8)]
    pub id: u8,
}

/// CTRL2 (0x11)
///
/// Gyroscope control register 2 (R/W)
#[register(address = Reg::Ctrl2, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Ctrl2 {
    /// Gyroscope output data rate selection (4 bits).
    /// See Table 55 for ODR values.
    #[bits(4)]
    pub odr_g: u8,
    /// Gyroscope operating mode selection (3 bits).
    /// 000: high-performance mode (default);
    /// 001: high-accuracy ODR mode;
    /// 010: reserved;
    /// 011: ODR-triggered mode;
    /// 100: sleep mode;
    /// 101: low-power mode;
    /// 110-111: reserved.
    #[bits(3)]
    pub op_mode_g: u8,
    #[bits(1, access = RO)]
    pub not_used0: u8,
}

/// CTRL3 (0x12)
///
/// Control register 3 (R/W)
/// Controls reboot, block data update, register address increment, and software reset.
#[register(address = Reg::Ctrl3, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Ctrl3 {
    /// Software reset, resets all control registers to their default value.
    /// This bit is automatically cleared.
    /// Default: 0 (normal mode)
    #[bits(1)]
    pub sw_reset: u8,
    #[bits(1, access = RO)]
    pub not_used0: u8,
    /// Register address automatically incremented during multiple byte access with serial interface (I²C, MIPI I3C, or SPI).
    /// Default: 1 (enabled)
    #[bits(1)]
    pub if_inc: u8,
    #[bits(3, access = RO)]
    pub not_used1: u8,
    /// Block data update.
    /// When set to 1, output registers are not updated until LSB and MSB have been read.
    /// Default: 0 (continuous update)
    #[bits(1)]
    pub bdu: u8,
    /// Reboots memory content.
    /// This bit is automatically cleared.
    /// Default: 0 (normal mode)
    #[bits(1)]
    pub boot: u8,
}

/// CTRL4 (0x13)
///
/// Control register 4 (R/W)
/// Controls interrupt routing, data-ready masking, and data-ready pulse mode.
#[register(address = Reg::Ctrl4, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Ctrl4 {
    /// Set to 1 to change the polarity of the INT2 pin input trigger for DEN or embedded functions.
    /// Default: 0 (active low)
    #[bits(1)]
    pub int2_in_lh: u8,
    /// Enables pulsed data-ready mode.
    /// Default: 0 (data-ready latched mode)
    #[bits(1)]
    pub drdy_pulsed: u8,
    /// Enables temperature sensor data-ready interrupt on the INT2 pin.
    /// Can also trigger an IBI when MIPI I3C interface is used and INT2_ON_INT1 = 1.
    /// Default: 0 (disabled)
    #[bits(1)]
    pub int2_drdy_temp: u8,
    /// Enables/masks data-ready signal.
    /// Default: 0 (disabled)
    #[bits(1)]
    pub drdy_mask: u8,
    /// Enables routing embedded functions interrupt signals to the INT1 pin.
    /// Corresponding bits in INT2 control registers need to be enabled.
    /// Default: 0 (disabled)
    #[bits(1)]
    pub int2_on_int1: u8,
    #[bits(3, access = RO)]
    pub not_used0: u8,
}

/// CTRL5 (0x14)
///
/// Control register 5 (R/W)
/// Controls bus available time selection for IBI and INT pin enable when I3C is enabled.
#[register(address = Reg::Ctrl5, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Ctrl5 {
    /// Enables INT pin when I3C is enabled.
    /// Default: 0 (disabled)
    #[bits(1)]
    pub int_en_i3c: u8,
    /// Bus available time selection for IBI (in-band interrupt).
    /// 00: 2 μs; 01: 50 μs (default); 10: 1 ms; 11: 25 ms.
    #[bits(2)]
    pub bus_act_sel: u8,
    #[bits(5, access = RO)]
    pub not_used0: u8,
}

/// CTRL6 (0x15)
///
/// Control register 6 (R/W)
/// Controls gyroscope full-scale and low-pass filter bandwidth selection.
#[register(address = Reg::Ctrl6, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Ctrl6 {
    /// Gyroscope full-scale selection (4 bits):
    /// 0000: ±125 dps (default);
    /// 0001: ±250 dps;
    /// 0010: ±500 dps;
    /// 0011: ±1000 dps;
    /// 0100: ±2000 dps;
    /// 1100: ±4000 dps (requires OIS disabled).
    #[bits(4)]
    pub fs_g: u8,
    /// Gyroscope low-pass filter (LPF1) bandwidth selection (3 bits).
    /// See Table 64 for bandwidth values.
    #[bits(3)]
    pub lpf1_g_bw: u8,
    #[bits(1, access = RO)]
    pub not_used0: u8,
}

/// CTRL7 (0x16)
///
/// Control register 7 (R/W)
/// Controls analog hub and Qvar chain enable, input impedance, and gyroscope digital LPF1 filter enable.
#[register(address = Reg::Ctrl7, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Ctrl7 {
    /// Enables gyroscope digital LPF1 filter.
    /// If OIS chain is disabled, bandwidth can be selected through CTRL6.
    /// Default: 0 (disabled)
    #[bits(1)]
    pub lpf1_g_en: u8,
    #[bits(3, access = RO)]
    pub not_used0: u8,
    /// Configures equivalent input impedance of analog hub and Qvar buffers (2 bits):
    /// 00: 2.4 GΩ (default);
    /// 01: 730 MΩ;
    /// 10: 300 MΩ;
    /// 11: 235 MΩ.
    #[bits(2)]
    pub ah_qvar_c_zin: u8,
    /// Analog hub and Qvar data-ready interrupt on INT2 pin.
    /// Default: 0 (disabled)
    #[bits(1)]
    pub int2_drdy_ah_qvar: u8,
    /// Enables analog hub and Qvar chain.
    /// Accelerometer and gyroscope must be in power-down mode before enabling.
    /// Default: 0 (disabled)
    #[bits(1)]
    pub ah_qvar_en: u8,
}

/// CTRL8 (0x17)
///
/// Control register 8 (R/W)
/// Controls accelerometer full-scale, dual-channel mode, and filter configuration.
#[register(address = Reg::Ctrl8, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Ctrl8 {
    /// Accelerometer full-scale selection (2 bits):
    /// 00: ±2 g;
    /// 01: ±4 g;
    /// 10: ±8 g;
    /// 11: ±16 g.
    #[bits(2)]
    pub fs_xl: u8,
    #[bits(1, access = RO)]
    pub not_used0: u8,
    /// Enables dual-channel mode.
    /// When set, data with maximum full scale are sent to output registers 0x34 to 0x39.
    /// Default: 0 (disabled)
    #[bits(1)]
    pub xl_dualc_en: u8,
    #[bits(1, access = RO)]
    pub not_used1: u8,
    /// Accelerometer LPF2 and HP filter configuration and cutoff setting (3 bits).
    /// See Table 69 for bandwidth configurations.
    #[bits(3)]
    pub hp_lpf2_xl_bw: u8,
}

/// CTRL9 (0x18)
///
/// Control register 9 (R/W)
/// Controls accelerometer user offset correction, filter selection, and fast-settling mode.
#[register(address = Reg::Ctrl9, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Ctrl9 {
    /// Enables accelerometer user offset correction block output.
    /// Default: 0 (disabled)
    #[bits(1)]
    pub usr_off_on_out: u8,
    /// Weight of accelerometer user offset bits.
    /// 0: 2^-10 g/LSB; 1: 2^-6 g/LSB.
    /// Default: 0
    #[bits(1)]
    pub usr_off_w: u8,
    #[bits(1, access = RO)]
    pub not_used0: u8,
    /// Accelerometer high-resolution selection.
    /// 0: output from first stage digital filtering;
    /// 1: output from LPF2 second filtering stage.
    /// Default: 0
    #[bits(1)]
    pub lpf2_xl_en: u8,
    /// Accelerometer slope filter / high-pass filter selection.
    /// 0: low-pass filter path selected;
    /// 1: high-pass filter path selected.
    /// Default: 0
    #[bits(1)]
    pub hp_slope_xl_en: u8,
    /// Enables accelerometer LPF2 and HPF fast-settling mode.
    /// Active only during device exit from power-down mode.
    /// Default: 0 (disabled)
    #[bits(1)]
    pub xl_fastsettl_mode: u8,
    /// Enables accelerometer high-pass filter reference mode.
    /// Valid only if HP_SLOPE_XL_EN bit is 1.
    /// Default: 0 (disabled)
    #[bits(1)]
    pub hp_ref_mode_xl: u8,
    #[bits(1, access = RO)]
    pub not_used1: u8,
}

/// CTRL10 (0x19)
///
/// Control register 10 (R/W)
/// Controls self-test selection and embedded functions debug mode.
#[register(address = Reg::Ctrl10, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Ctrl10 {
    /// Accelerometer self-test selection (2 bits):
    /// 00: normal mode (default);
    /// 01: positive sign self-test;
    /// 10: negative sign self-test;
    /// 11: reserved.
    #[bits(2)]
    pub st_xl: u8,
    /// Gyroscope self-test selection (2 bits):
    /// 00: normal mode (default);
    /// 01: positive sign self-test;
    /// 10: negative sign self-test;
    /// 11: reserved.
    #[bits(2)]
    pub st_g: u8,
    #[bits(2, access = RO)]
    pub not_used0: u8,
    /// Enables debug mode for the embedded functions.
    /// Default: 0 (disabled)
    #[bits(1)]
    pub emb_func_debug: u8,
    #[bits(1, access = RO)]
    pub not_used1: u8,
}

/// CTRL_STATUS (0x1A)
///
/// Control status register (R)
/// Indicates current controller of device configuration registers.
#[register(address = Reg::CtrlStatus, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct CtrlStatus {
    #[bits(2, access = RO)]
    pub not_used0: u8,
    /// Flag indicating current controller of device configuration registers.
    /// 0: all registers writable from standard interface;
    /// 1: some registers under FSM control and read-only from standard interface.
    #[bits(1)]
    pub fsm_wr_ctrl_status: u8,
    #[bits(5, access = RO)]
    pub not_used1: u8,
}

/// FIFO_STATUS1-2 (0x1B - 0x1C)
///
/// Read both FIFO_STATUS1 and FIFO_STATUS2 registers
///
/// FIFO status register 1 (R)
/// Number of unread sensor data stored in FIFO (lower 8 bits).
///
/// FIFO status register 2 (R)
/// FIFO status flags and higher bit of unread data count.
#[register(address = Reg::FifoStatus1, access_type = Lsm6dsv16x, generics = 2, override_type = u16)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u16, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u16, order = Lsb))]
pub struct FifoStatusReg {
    #[bits(9)]
    pub diff_fifo: u16,
    #[bits(2, access = RO)]
    pub not_used0: u8,
    #[bits(1)]
    pub fifo_ovr_latched: u8,
    #[bits(1)]
    pub counter_bdr_ia: u8,
    #[bits(1)]
    pub fifo_full_ia: u8,
    #[bits(1)]
    pub fifo_ovr_ia: u8,
    #[bits(1)]
    pub fifo_wtm_ia: u8,
}

/// ALL_INT_SRC (0x1D)
///
/// Source register for all interrupts (R)
/// Indicates status of various interrupt events.
#[register(address = Reg::AllIntSrc, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct AllIntSrc {
    /// Free-fall event status.
    /// 0: not detected; 1: detected.
    #[bits(1)]
    pub ff_ia: u8,
    /// Wake-up event status.
    /// 0: not detected; 1: detected.
    #[bits(1)]
    pub wu_ia: u8,
    /// Tap event detection status.
    /// 0: not detected; 1: detected.
    #[bits(1)]
    pub tap_ia: u8,
    #[bits(1, access = RO)]
    pub not_used0: u8,
    /// Interrupt active for change in position (portrait, landscape, face-up, face-down).
    /// 0: not detected; 1: detected.
    #[bits(1)]
    pub d6d_ia: u8,
    /// Detects change event in activity/inactivity status.
    /// 0: not detected; 1: detected.
    #[bits(1)]
    pub sleep_change_ia: u8,
    /// Sensor hub (I²C master) interrupt status.
    /// 0: not generated; 1: generated.
    #[bits(1)]
    pub shub_ia: u8,
    /// Embedded functions interrupt status.
    /// 0: not detected; 1: detected.
    #[bits(1)]
    pub emb_func_ia: u8,
}

/// STATUS_REG (0x1E)
///
/// Status register (R)
/// Indicates availability of new data and timestamp overflow.
#[register(address = Reg::StatusReg, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct StatusReg {
    /// Accelerometer new data available.
    /// 0: no new data; 1: new data available.
    #[bits(1)]
    pub xlda: u8,
    /// Gyroscope new data available.
    /// 0: no new data; 1: new data available.
    #[bits(1)]
    pub gda: u8,
    /// Temperature new data available.
    /// 0: no new data; 1: new data available.
    #[bits(1)]
    pub tda: u8,
    /// Analog hub or Qvar new data available.
    /// 0: no new data; 1: new data available.
    #[bits(1)]
    pub ah_qvarda: u8,
    /// Enhanced EIS gyroscope new data available.
    /// 0: no new data; 1: new data available.
    #[bits(1)]
    pub gda_eis: u8,
    /// Accelerometer OIS or gyroscope OIS new output data available.
    /// 0: no new data; 1: new data available.
    #[bits(1)]
    pub ois_drdy: u8,
    #[bits(1, access = RO)]
    pub not_used0: u8,
    /// Alerts timestamp overflow within 5.6 ms.
    #[bits(1)]
    pub timestamp_endcount: u8,
}

/// OUT_TEMP (0x20 - 0x21)
///
/// Temperature data output registers: signed 16 bits
#[register(address = Reg::OutTempL, access_type = Lsm6dsv16x, generics = 2)]
pub struct OutTemp(pub i16);

/// OUTX_L_G - OUTZ_H_G (0x22 - 0x27)
///
/// Angular rate sensor pitch axis (X, Y, Z) angular rate output registers (R)
#[named_register(address = Reg::OutxLG, access_type = Lsm6dsv16x, generics = 2)]
pub struct OutXYZG {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

/// OUTX_L_A - OUTZ_H_A (0x28 - 0x2D)
///
/// Linear acceleration sensor (X, Y, Z)-axis output registers (R)
#[named_register(address = Reg::OutxLA, access_type = Lsm6dsv16x, generics = 2)]
pub struct OutXYZA {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

/// UI_OUTX_L_G_OIS_EIS - UI_OUTZ_H_G_OIS_EIS (0x2E - 0x33)
///
/// Gyroscope pitch axis (X, Y, Z) angular rate output registers (R)
/// Data according to gyroscope full-scale and ODR settings of OIS or EIS gyroscope channel.
#[named_register(address = Reg::UiOutxLGOisEis, access_type = Lsm6dsv16x, generics = 2)]
pub struct UiOutXYZGOisEis {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

/// UI_OUTX_L_A_OIS_DualC - UI_OUTZ_H_A_OIS_DualC (0x34 - 0x39)
///
/// Linear acceleration sensor (X, Y, Z)-axis output registers (R)
/// Data according to accelerometer full-scale and ODR settings of OIS accelerometer or dual-channel mode.
#[named_register(address = Reg::UiOutxLAOisDualc, access_type = Lsm6dsv16x, generics = 2)]
pub struct UiOutXYZAOisDualc {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

/// AH_QVAR_OUT_L - AH_QVAR_OUT_H (0x3A - 0x3B)
///
/// Analog hub and Qvar data output registers (R)
#[register(address = Reg::AhQvarOutL, access_type = Lsm6dsv16x, generics = 2)]
pub struct AhQvarOut(pub i16);

/// TIMESTAMP (0x40 - 0x43)
///
/// Timestamp output registers (R)
/// Timestamp is a 32-bit word with bit resolution 21.75 μs (typical).
#[register(address = Reg::Timestamp0, access_type = Lsm6dsv16x, generics = 2)]
pub struct Timestamp(pub u32);

/// UI_STATUS_REG_OIS (0x44)
///
/// OIS status register (R)
#[register(address = Reg::UiStatusRegOis, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct UiStatusRegOis {
    /// Accelerometer OIS data available (reset when one of the high parts of the output data is read)
    #[bits(1)]
    pub xlda_ois: u8,
    /// Gyroscope OIS data available (reset when one of the high parts of the output data is read)
    #[bits(1)]
    pub gda_ois: u8,
    /// High when the gyroscope output is in the settling phase
    #[bits(1)]
    pub gyro_settling: u8,
    #[bits(5, access = RO)]
    pub not_used0: u8,
}

/// WAKE_UP_SRC (0x45)
///
/// Wake-up interrupt source register (R)
#[register(address = Reg::WakeUpSrc, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct WakeUpSrc {
    /// Wake-up event detection status on Z-axis
    #[bits(1)]
    pub z_wu: u8,
    /// Wake-up event detection status on Y-axis
    #[bits(1)]
    pub y_wu: u8,
    /// Wake-up event detection status on X-axis
    #[bits(1)]
    pub x_wu: u8,
    /// Wake-up event detection status
    #[bits(1)]
    pub wu_ia: u8,
    /// Sleep status bit (0: Activity status; 1: Inactivity status)
    #[bits(1)]
    pub sleep_state: u8,
    /// Free-fall event detection status
    #[bits(1)]
    pub ff_ia: u8,
    /// Detects change event in activity/inactivity status
    #[bits(1)]
    pub sleep_change_ia: u8,
    #[bits(1, access = RO)]
    pub not_used0: u8,
}

/// TAP_SRC (0x46)
///
/// Tap source register (R)
#[register(address = Reg::TapSrc, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct TapSrc {
    /// Tap event detection status on Z-axis
    #[bits(1)]
    pub z_tap: u8,
    /// Tap event detection status on Y-axis
    #[bits(1)]
    pub y_tap: u8,
    /// Tap event detection status on X-axis
    #[bits(1)]
    pub x_tap: u8,
    /// Sign of acceleration detected by tap event (0: positive; 1: negative)
    #[bits(1)]
    pub tap_sign: u8,
    /// Double-tap event detection status
    #[bits(1)]
    pub double_tap: u8,
    /// Single-tap event status
    #[bits(1)]
    pub single_tap: u8,
    /// Tap event detection status
    #[bits(1)]
    pub tap_ia: u8,
    #[bits(1, access = RO)]
    pub not_used0: u8,
}

/// D6D_SRC (0x47)
///
/// Portrait, landscape, face-up and face-down source register (R)
#[register(address = Reg::D6dSrc, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct D6dSrc {
    /// X-axis low event (under threshold)
    #[bits(1)]
    pub xl: u8,
    /// X-axis high event (over threshold)
    #[bits(1)]
    pub xh: u8,
    /// Y-axis low event (under threshold)
    #[bits(1)]
    pub yl: u8,
    /// Y-axis high event (over threshold)
    #[bits(1)]
    pub yh: u8,
    /// Z-axis low event (under threshold)
    #[bits(1)]
    pub zl: u8,
    /// Z-axis high event (over threshold)
    #[bits(1)]
    pub zh: u8,
    /// Interrupt active for change position portrait, landscape, face-up, face-down
    #[bits(1)]
    pub d6d_ia: u8,
    #[bits(1, access = RO)]
    pub not_used0: u8,
}

/// STATUS_MASTER_MAINPAGE (0x48)
///
/// Sensor hub source register (R)
#[register(address = Reg::StatusMasterMainPage, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct StatusMasterMainPage {
    /// Sensor hub communication status (0: not concluded; 1: concluded)
    #[bits(1)]
    pub sens_hub_endop: u8,
    #[bits(2, access = RO)]
    pub not_used0: u8,
    /// Not acknowledge occurs on slave 0 communication
    #[bits(1)]
    pub slave0_nack: u8,
    /// Not acknowledge occurs on slave 1 communication
    #[bits(1)]
    pub slave1_nack: u8,
    /// Not acknowledge occurs on slave 2 communication
    #[bits(1)]
    pub slave2_nack: u8,
    /// Not acknowledge occurs on slave 3 communication
    #[bits(1)]
    pub slave3_nack: u8,
    /// Write operation on slave 0 performed and completed (when WRITE_ONCE bit is set)
    #[bits(1)]
    pub wr_once_done: u8,
}

/// EMB_FUNC_STATUS_MAINPAGE (0x49)
///
/// Embedded function status register (R)
#[register(address = Reg::EmbFuncStatusMainPage, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct EmbFuncStatusMainPage {
    #[bits(3, access = RO)]
    pub not_used0: u8,
    /// Interrupt status bit for step detection (1: interrupt detected)
    #[bits(1)]
    pub is_step_det: u8,
    /// Interrupt status bit for tilt detection (1: interrupt detected)
    #[bits(1)]
    pub is_tilt: u8,
    /// Interrupt status bit for significant motion detection (1: interrupt detected)
    #[bits(1)]
    pub is_sigmot: u8,
    #[bits(1, access = RO)]
    pub not_used1: u8,
    /// Interrupt status bit for FSM long counter timeout interrupt event (1: interrupt detected)
    #[bits(1)]
    pub is_fsm_lc: u8,
}

/// FSM_STATUS_MAINPAGE (0x4A)
///
/// Finite state machine status register (R)
#[register(address = Reg::FsmStatusMainPage, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct FsmStatusMainPage {
    /// Interrupt status bit for FSM1 interrupt event
    #[bits(1)]
    pub is_fsm1: u8,
    /// Interrupt status bit for FSM2 interrupt event
    #[bits(1)]
    pub is_fsm2: u8,
    /// Interrupt status bit for FSM3 interrupt event
    #[bits(1)]
    pub is_fsm3: u8,
    /// Interrupt status bit for FSM4 interrupt event
    #[bits(1)]
    pub is_fsm4: u8,
    /// Interrupt status bit for FSM5 interrupt event
    #[bits(1)]
    pub is_fsm5: u8,
    /// Interrupt status bit for FSM6 interrupt event
    #[bits(1)]
    pub is_fsm6: u8,
    /// Interrupt status bit for FSM7 interrupt event
    #[bits(1)]
    pub is_fsm7: u8,
    /// Interrupt status bit for FSM8 interrupt event
    #[bits(1)]
    pub is_fsm8: u8,
}

/// MLC_STATUS_MAINPAGE (0x4B)
///
/// Machine learning core status register (R)
#[register(address = Reg::MlcStatusMainPage, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct MlcStatusMainPage {
    /// Interrupt status bit for MLC1 interrupt event
    #[bits(1)]
    pub is_mlc1: u8,
    /// Interrupt status bit for MLC2 interrupt event
    #[bits(1)]
    pub is_mlc2: u8,
    /// Interrupt status bit for MLC3 interrupt event
    #[bits(1)]
    pub is_mlc3: u8,
    /// Interrupt status bit for MLC4 interrupt event
    #[bits(1)]
    pub is_mlc4: u8,
    #[bits(4, access = RO)]
    pub not_used0: u8,
}

/// INTERNAL_FREQ_FINE (0x4F)
///
/// Internal frequency register (R)
#[register(address = Reg::InternalFreq, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct InternalFreq {
    /// Difference in percentage of the effective ODR (and timestamp rate) with respect to the typical.
    /// Step: 0.13%. 8-bit format, two's complement.
    #[bits(8)]
    pub freq_fine: u8,
}

/// FUNCTIONS_ENABLE (0x50)
///
/// Enable interrupt functions register (R/W)
#[register(address = Reg::FunctionsEnable, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct FunctionsEnable {
    /// Enables activity/inactivity (sleep) function (2 bits):
    /// 00: stationary/motion-only interrupts generated, accelerometer and gyroscope configuration do not change;
    /// 01: accelerometer to low-power mode 1 with ODR selected through INACTIVITY_DUR register, gyroscope unchanged;
    /// 10: accelerometer to low-power mode 1 with ODR selected, gyroscope in sleep mode;
    /// 11: accelerometer to low-power mode 1 with ODR selected, gyroscope in power-down mode.
    #[bits(2)]
    pub inact_en: u8,
    #[bits(1, access = RO)]
    pub not_used0: u8,
    /// When set to 1, reading ALL_INT_SRC register does not reset latched interrupt signals.
    /// Useful to avoid resetting status flags before reading corresponding status register.
    #[bits(1)]
    pub dis_rst_lir_all_int: u8,
    #[bits(2, access = RO)]
    pub not_used1: u8,
    /// Enables timestamp counter.
    /// Counter readable in TIMESTAMP0-3 registers.
    #[bits(1)]
    pub timestamp_en: u8,
    /// Enables basic interrupts (6D/4D, free-fall, wake-up, tap, activity/inactivity).
    #[bits(1)]
    pub interrupts_enable: u8,
}

/// DEN (0x51)
///
/// DEN configuration register (R/W)
#[register(address = Reg::Den, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Den {
    /// DEN stamping sensor selection.
    /// Default: 0 (DEN pin info stamped in the gyroscope axis selected by bits DEN_X, DEN_Y, DEN_Z)
    #[bits(1)]
    pub den_xl_g: u8,
    /// DEN value stored in LSB of Z-axis.
    /// Default: 1 (DEN stored in Z-axis LSB)
    #[bits(1)]
    pub den_z: u8,
    /// DEN value stored in LSB of Y-axis.
    /// Default: 1 (DEN stored in Y-axis LSB)
    #[bits(1)]
    pub den_y: u8,
    /// DEN value stored in LSB of X-axis.
    /// Default: 1 (DEN stored in X-axis LSB)
    #[bits(1)]
    pub den_x: u8,
    /// Extends DEN functionality to accelerometer sensor.
    /// Default: 0 (disabled)
    #[bits(1)]
    pub den_xl_en: u8,
    /// Enables DEN level-sensitive latched mode.
    #[bits(1)]
    pub lvl2_en: u8,
    /// Enables DEN data level-sensitive trigger mode.
    #[bits(1)]
    pub lvl1_en: u8,
    #[bits(1, access = RO)]
    pub not_used0: u8,
}

/// INACTIVITY_DUR (0x54)
///
/// Activity/inactivity configuration register (R/W)
#[register(address = Reg::InactivityDur, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct InactivityDur {
    /// Duration in the transition from stationary to motion (from inactivity to activity).
    /// Default: 00 (immediate transition at first overthreshold event)
    #[bits(2)]
    pub inact_dur: u8,
    /// Selects the ODR_XL target during inactivity.
    /// Default: 01 (15 Hz)
    #[bits(2)]
    pub xl_inact_odr: u8,
    /// Weight of 1 LSB of wake-up (WU_THS) and activity/inactivity (INACT_THS) threshold.
    /// Default: 000 (7.8125 mg/LSB)
    #[bits(3)]
    pub wu_inact_ths_w: u8,
    /// Activity/inactivity interrupt mode configuration.
    /// Default: 0 (sleep change notification on INT pin)
    #[bits(1)]
    pub sleep_status_on_int: u8,
}

/// INACTIVITY_THS (0x55)
///
/// Activity/inactivity threshold setting register (R/W)
#[register(address = Reg::InactivityThs, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct InactivityThs {
    /// Activity/inactivity threshold.
    /// Resolution depends on WU_INACT_THS_W bits in INACTIVITY_DUR register.
    /// Default: 000000
    #[bits(6)]
    pub inact_ths: u8,
    #[bits(2, access = RO)]
    pub not_used0: u8,
}

/// TAP_CFG0 (0x56)
///
/// Tap configuration register 0 (R/W)
#[register(address = Reg::TapCfg0, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct TapCfg0 {
    /// Latched interrupt.
    /// Default: 0 (interrupt request not latched)
    #[bits(1)]
    pub lir: u8,
    /// Enables Z direction in tap recognition.
    /// Default: 0 (disabled)
    #[bits(1)]
    pub tap_z_en: u8,
    /// Enables Y direction in tap recognition.
    /// Default: 0 (disabled)
    #[bits(1)]
    pub tap_y_en: u8,
    /// Enables X direction in tap recognition.
    /// Default: 0 (disabled)
    #[bits(1)]
    pub tap_x_en: u8,
    /// HPF or slope filter selection on wake-up and activity/inactivity functions.
    /// Default: 0 (slope filter applied)
    #[bits(1)]
    pub slope_fds: u8,
    /// Enables masking the execution trigger of basic interrupt functions when accelerometer data are settling.
    /// Default: 0 (disabled)
    #[bits(1)]
    pub hw_func_mask_xl_settl: u8,
    /// LPF2 filter on 6D function selection.
    /// Default: 0 (ODR/2 low-pass filtered data sent to 6D interrupt function)
    #[bits(1)]
    pub low_pass_on_6d: u8,
    #[bits(1, access = RO)]
    pub not_used1: u8,
}

/// TAP_CFG1 (0x57)
///
/// Tap configuration register 1 (R/W)
#[register(address = Reg::TapCfg1, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct TapCfg1 {
    /// X-axis tap recognition threshold.
    /// Default: 0
    #[bits(5)]
    pub tap_ths_x: u8,
    /// Selection of axis priority for tap detection.
    #[bits(3)]
    pub tap_priority: u8,
}

/// TAP_CFG2 (0x58)
///
/// Tap configuration register 2 (R/W)
#[register(address = Reg::TapCfg2, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct TapCfg2 {
    /// Y-axis tap recognition threshold.
    /// Default: 0
    #[bits(5)]
    pub tap_ths_y: u8,
    #[bits(3, access = RO)]
    pub not_used0: u8,
}

/// TAP_THS_6D (0x59)
///
/// Portrait/landscape position and tap function threshold register (R/W)
#[register(address = Reg::TapThs6d, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct TapThs6d {
    /// Z-axis recognition threshold.
    /// Default: 0
    #[bits(5)]
    pub tap_ths_z: u8,
    /// Threshold for 4D/6D function.
    /// Default: 00 (80 degrees)
    #[bits(2)]
    pub sixd_ths: u8,
    /// Enables 4D orientation detection.
    /// Default: 0 (disabled)
    #[bits(1)]
    pub d4d_en: u8,
}

/// TAP_DUR (0x5A)
///
/// Tap recognition function setting register (R/W)
#[register(address = Reg::TapDur, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct TapDur {
    /// Maximum duration of overthreshold event.
    /// Default: 00 (4/ODR_XL time)
    #[bits(2)]
    pub shock: u8,
    /// Expected quiet time after a tap detection.
    /// Default: 00 (2/ODR_XL time)
    #[bits(2)]
    pub quiet: u8,
    /// Duration of maximum time gap for double-tap recognition.
    /// Default: 0000 (16/ODR_XL time)
    #[bits(4)]
    pub dur: u8,
}

/// WAKE_UP_THS (0x5B)
///
/// Single/double-tap selection and wake-up configuration (R/W)
#[register(address = Reg::WakeUpThs, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct WakeUpThs {
    /// Wake-up threshold.
    /// Resolution depends on WU_INACT_THS_W bits in INACTIVITY_DUR register.
    /// Default: 000000
    #[bits(6)]
    pub wk_ths: u8,
    /// Drives the low-pass filtered data with user offset correction to wake-up and activity/inactivity functions.
    /// Default: 0 (disabled)
    #[bits(1)]
    pub usr_off_on_wu: u8,
    /// Enables single/double-tap event.
    /// Default: 0 (only single-tap event enabled)
    #[bits(1)]
    pub single_double_tap: u8,
}

/// WAKE_UP_DUR (0x5C)
///
/// Free-fall, wake-up, and sleep mode functions duration setting register (R/W)
#[register(address = Reg::WakeUpDur, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct WakeUpDur {
    /// Duration to go in sleep mode.
    /// Default: 0000 (16 ODR)
    #[bits(4)]
    pub sleep_dur: u8,
    #[bits(1, access = RO)]
    pub not_used0: u8,
    /// Wake-up duration event.
    /// Default: 00 (1/ODR_XL time)
    #[bits(2)]
    pub wake_dur: u8,
    /// Free-fall duration event.
    /// Default: 0
    #[bits(1)]
    pub ff_dur: u8,
}

/// FREE_FALL (0x5D)
///
/// Free-fall function duration setting register (R/W)
#[register(address = Reg::FreeFall, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct FreeFall {
    /// Free-fall threshold setting.
    /// Default: 000 (156 mg)
    #[bits(3)]
    pub ff_ths: u8,
    /// Free-fall duration event.
    /// Default: 00000
    #[bits(5)]
    pub ff_dur: u8,
}

/// MD1_CFG (0x5E)
///
/// Functions routing to INT1 pin register (R/W)
#[register(address = Reg::Md1Cfg, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Md1Cfg {
    /// Routing sensor hub communication concluded event to INT1.
    #[bits(1)]
    pub int1_shub: u8,
    /// Routing embedded functions event to INT1.
    #[bits(1)]
    pub int1_emb_func: u8,
    /// Routing 6D event to INT1.
    #[bits(1)]
    pub int1_6d: u8,
    /// Routing double-tap event to INT1.
    #[bits(1)]
    pub int1_double_tap: u8,
    /// Routing free-fall event to INT1.
    #[bits(1)]
    pub int1_ff: u8,
    /// Routing wake-up event to INT1.
    #[bits(1)]
    pub int1_wu: u8,
    /// Routing single-tap event to INT1.
    #[bits(1)]
    pub int1_single_tap: u8,
    /// Routing activity/inactivity recognition event to INT1.
    #[bits(1)]
    pub int1_sleep_change: u8,
}

/// MD2_CFG (0x5F)
///
/// Functions routing to INT2 pin register (R/W)
#[register(address = Reg::Md2Cfg, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Md2Cfg {
    /// Enables routing the alert for timestamp overflow within 5.6 ms to the INT2 pin.
    #[bits(1)]
    pub int2_timestamp: u8,
    /// Routing embedded functions event to INT2.
    #[bits(1)]
    pub int2_emb_func: u8,
    /// Routing 6D event to INT2.
    #[bits(1)]
    pub int2_6d: u8,
    /// Routing double-tap event to INT2.
    #[bits(1)]
    pub int2_double_tap: u8,
    /// Routing free-fall event to INT2.
    #[bits(1)]
    pub int2_ff: u8,
    /// Routing wake-up event to INT2.
    #[bits(1)]
    pub int2_wu: u8,
    /// Routing single-tap event to INT2.
    #[bits(1)]
    pub int2_single_tap: u8,
    /// Routing activity/inactivity recognition event to INT2.
    #[bits(1)]
    pub int2_sleep_change: u8,
}

/// HAODR_CFG (0x62)
///
/// HAODR data rate configuration register (R/W)
#[register(address = Reg::HaodrCfg, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct HaodrCfg {
    /// Selects the ODR set supported when high-accuracy ODR (HAODR) mode is enabled.
    /// Default: 00
    #[bits(2)]
    pub haodr_sel: u8,
    #[bits(6, access = RO)]
    pub not_used0: u8,
}

/// EMB_FUNC_CFG (0x63)
///
/// Embedded functions configuration register (R/W)
#[register(address = Reg::EmbFuncCfg, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct EmbFuncCfg {
    #[bits(3, access = RO)]
    pub not_used0: u8,
    /// Disables execution of the embedded functions.
    /// Default: 0 (disabled)
    #[bits(1)]
    pub emb_func_disable: u8,
    /// Enables/masks execution trigger of the embedded functions when accelerometer data are settling.
    /// Default: 0 (disabled)
    #[bits(1)]
    pub emb_func_irq_mask_xl_settl: u8,
    /// Enables/masks execution trigger of the embedded functions when gyroscope data are settling.
    /// Default: 0 (disabled)
    #[bits(1)]
    pub emb_func_irq_mask_g_settl: u8,
    #[bits(2, access = RO)]
    pub not_used1: u8,
}

/// UI_HANDSHAKE_CTRL (0x64)
///
/// Control register (UI side) for UI / SPI2 shared registers (R/W)
#[register(address = Reg::UiHandshakeCtrl, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct UiHandshakeCtrl {
    /// Primary interface master requests access to UI_SPI2_SHARED_0 through UI_SPI2_SHARED_5 registers.
    /// Must be reset after R/W operation is finished.
    #[bits(1)]
    pub ui_shared_req: u8,
    /// Primary interface side acknowledges the handshake.
    /// Set to 1 by device if secondary interface is not accessing shared registers.
    #[bits(1)]
    pub ui_shared_ack: u8,
    #[bits(6, access = RO)]
    pub not_used0: u8,
}

/// UI_SPI2_SHARED_0 - UI_SPI2_SHARED_5 (0x65 - 0x6A)
///
/// UI / SPI2 shared register 0 - 5 (R/W)
/// Volatile bytes used as a contact point between primary and secondary interface hosts.
#[register(address = Reg::UiSpi2Shared0, access_type = Lsm6dsv16x, generics = 2)]
pub struct UiSpi2Shared(pub [u8; 6]);

/// CTRL_EIS (0x6B)
///
/// Gyroscope EIS channel control register (R/W)
#[register(address = Reg::CtrlEis, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct CtrlEis {
    /// Gyroscope full-scale selection for EIS channel (3 bits).
    #[bits(3)]
    pub fs_g_eis: u8,
    /// Enables routing gyroscope EIS output to OIS from UI output addresses (2Eh – 33h).
    /// When set to 1, gyroscope OIS data cannot be read from primary interface.
    #[bits(1)]
    pub g_eis_on_g_ois_out_reg: u8,
    /// Gyroscope digital LPF_EIS filter bandwidth selection.
    #[bits(1)]
    pub lpf_g_eis_bw: u8,
    #[bits(1, access = RO)]
    pub not_used0: u8,
    /// Enables and selects the ODR of the gyroscope EIS channel (2 bits).
    #[bits(2)]
    pub odr_g_eis: u8,
}

/// UI_INT_OIS (0x6F)
///
/// OIS interrupt configuration register
/// Writable by primary interface when OIS_CTRL_FROM_UI bit is 1 (primary IF full-control mode).
/// Read-only when OIS_CTRL_FROM_UI bit is 0 (SPI2 full-control mode).
#[register(address = Reg::UiIntOis, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct UiIntOis {
    #[bits(4, access = RO)]
    pub not_used0: u8,
    /// Disables OIS chain clamp during self-test.
    /// 0: All OIS chain outputs = 8000h during self-test (default).
    /// 1: OIS chain self-test outputs.
    #[bits(1)]
    pub st_ois_clampdis: u8,
    #[bits(1, access = RO)]
    pub not_used1: u8,
    /// Enables/masks OIS data available.
    /// 0: disabled; 1: masks OIS DRDY signals until filter settling ends.
    #[bits(1)]
    pub drdy_mask_ois: u8,
    /// Enables OIS chain DRDY on INT2 pin.
    /// This setting has priority over all other INT2 settings.
    #[bits(1)]
    pub int2_drdy_ois: u8,
}

/// UI_CTRL1_OIS (0x70)
///
/// OIS configuration register
/// Writable by primary interface when OIS_CTRL_FROM_UI bit is 1 (primary IF full-control mode).
/// Read-only when OIS_CTRL_FROM_UI bit is 0 (SPI2 full-control mode).
#[register(address = Reg::UiCtrl1Ois, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct UiCtrl1Ois {
    /// Enables auxiliary SPI for reading OIS data.
    /// 0: disabled; 1: enabled.
    #[bits(1)]
    pub spi2_read_en: u8,
    /// Enables gyroscope OIS chain.
    /// 0: disabled; 1: enabled.
    #[bits(1)]
    pub ois_g_en: u8,
    /// Enables accelerometer OIS chain.
    /// 0: disabled; 1: enabled.
    #[bits(1)]
    pub ois_xl_en: u8,
    #[bits(2, access = RO)]
    pub not_used0: u8,
    /// SPI2 3- or 4-wire interface.
    /// 0: 4-wire SPI2; 1: 3-wire SPI2.
    #[bits(1)]
    pub sim_ois: u8,
    #[bits(2, access = RO)]
    pub not_used1: u8,
}

/// UI_CTRL2_OIS (0x71)
///
/// OIS configuration register
/// Writable by primary interface when OIS_CTRL_FROM_UI bit is 1 (primary IF full-control mode).
/// Read-only when OIS_CTRL_FROM_UI bit is 0 (SPI2 full-control mode).
#[register(address = Reg::UiCtrl2Ois, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct UiCtrl2Ois {
    /// Gyroscope OIS full-scale selection (3 bits).
    #[bits(3)]
    pub fs_g_ois: u8,
    /// Gyroscope OIS digital LPF1 filter bandwidth selection (2 bits).
    #[bits(2)]
    pub lpf1_g_ois_bw: u8,
    #[bits(3, access = RO)]
    pub not_used0: u8,
}

/// UI_CTRL3_OIS (0x72)
///
/// OIS configuration register
/// Writable by primary interface when OIS_CTRL_FROM_UI bit is 1 (primary IF full-control mode).
/// Read-only when OIS_CTRL_FROM_UI bit is 0 (SPI2 full-control mode).
#[register(address = Reg::UiCtrl3Ois, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct UiCtrl3Ois {
    /// Accelerometer OIS channel full-scale selection (2 bits).
    #[bits(2)]
    pub fs_xl_ois: u8,
    #[bits(1, access = RO)]
    pub not_used0: u8,
    /// Accelerometer OIS channel bandwidth selection (3 bits).
    #[bits(3)]
    pub lpf_xl_ois_bw: u8,
    #[bits(2, access = RO)]
    pub not_used1: u8,
}

/// X_OFS_USR (0x73)
///
/// Accelerometer X-axis user offset correction (R/W)
#[register(address = Reg::XOfsUsr, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct XOfsUsr {
    /// Accelerometer X-axis user offset correction expressed in two’s complement.
    /// Weight depends on USR_OFF_W in CTRL9 (18h).
    /// Value range: [-127, 127].
    #[bits(8)]
    pub x_ofs_usr: u8,
}

/// Y_OFS_USR (0x74)
///
/// Accelerometer Y-axis user offset correction (R/W)
#[register(address = Reg::YOfsUsr, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct YOfsUsr {
    /// Accelerometer Y-axis user offset correction expressed in two’s complement.
    /// Weight depends on USR_OFF_W in CTRL9 (18h).
    /// Value range: [-127, 127].
    #[bits(8)]
    pub y_ofs_usr: u8,
}

/// Z_OFS_USR (0x75)
///
/// Accelerometer Z-axis user offset correction (R/W)
#[register(address = Reg::ZOfsUsr, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct ZOfsUsr {
    /// Accelerometer Z-axis user offset correction expressed in two’s complement.
    /// Weight depends on USR_OFF_W in CTRL9 (18h).
    /// Value range: [-127, 127].
    #[bits(8)]
    pub z_ofs_usr: u8,
}

/// FIFO_DATA_OUT_TAG (0x78)
///
/// FIFO tag register (R)
#[register(address = Reg::FifoDataOutTag, access_type = Lsm6dsv16x, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct FifoDataOutTag {
    #[bits(1, access = RO)]
    pub not_used0: u8,
    /// 2-bit counter which identifies sensor time slot.
    #[bits(2)]
    pub tag_cnt: u8,
    /// 5-bit FIFO tag identifying the sensor.
    #[bits(5)]
    pub tag_sensor: u8,
}

/// FIFO_DATA_OUT_X_L - FIFO_DATA_OUT_Z_H (0x79 - 0x7E)
///
/// FIFO data output X, Y, Z (R)
#[register(address = Reg::FifoDataOutXL, access_type = Lsm6dsv16x, generics = 2)]
pub struct FifoDataOutXYZ(pub [u8; 6]);

/// Reset commands for the device
///
/// Used to control device reset and restore operations.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum Reset {
    /// Ready state, no reset
    Ready = 0x0,
    /// Global reset (default)
    #[default]
    GlobalRst = 0x1,
    /// Restore calibration parameters
    RestoreCalParam = 0x2,
    /// Restore control registers
    RestoreCtrlRegs = 0x4,
}

/// Output data rate (ODR) selection
///
/// Defines the output data rate for accelerometer and gyroscope.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum Odr {
    /// Output data rate off (default)
    #[default]
    Off = 0x0,
    /// Output data rate at 1.875 Hz
    _1_875hz = 0x1,
    /// Output data rate at 7.5 Hz
    _7_5hz = 0x2,
    /// Output data rate at 15 Hz
    _15hz = 0x3,
    /// Output data rate at 30 Hz
    _30hz = 0x4,
    /// Output data rate at 60 Hz
    _60hz = 0x5,
    /// Output data rate at 120 Hz
    _120hz = 0x6,
    /// Output data rate at 240 Hz
    _240hz = 0x7,
    /// Output data rate at 480 Hz
    _480hz = 0x8,
    /// Output data rate at 960 Hz
    _960hz = 0x9,
    /// Output data rate at 1920 Hz
    _1920hz = 0xA,
    /// Output data rate at 3840 Hz
    _3840hz = 0xB,
    /// Output data rate at 7680 Hz
    _7680hz = 0xC,
    /// High-accuracy ODR at 15.625 Hz
    Ha01At15_625hz = 0x13,
    /// High-accuracy ODR at 31.25 Hz
    Ha01At31_25hz = 0x14,
    /// High-accuracy ODR at 62.5 Hz
    Ha01At62_5hz = 0x15,
    /// High-accuracy ODR at 125 Hz
    Ha01_125hz = 0x16,
    /// High-accuracy ODR at 250 Hz
    Ha01_250hz = 0x17,
    /// High-accuracy ODR at 500 Hz
    Ha01_500hz = 0x18,
    /// High-accuracy ODR at 1000 Hz
    Ha01_1000hz = 0x19,
    /// High-accuracy ODR at 2000 Hz
    Ha01_2000hz = 0x1A,
    /// High-accuracy ODR at 4000 Hz
    Ha01_4000hz = 0x1B,
    /// High-accuracy ODR at 8000 Hz
    Ha01_8000hz = 0x1C,
    /// High-accuracy ODR at 12.5 Hz (HA02)
    Ha02At12_5hz = 0x23,
    /// High-accuracy ODR at 25 Hz (HA02)
    Ha02_25hz = 0x24,
    /// High-accuracy ODR at 50 Hz (HA02)
    Ha02_50hz = 0x25,
    /// High-accuracy ODR at 100 Hz (HA02)
    Ha02_100hz = 0x26,
    /// High-accuracy ODR at 200 Hz (HA02)
    Ha02_200hz = 0x27,
    /// High-accuracy ODR at 400 Hz (HA02)
    Ha02_400hz = 0x28,
    /// High-accuracy ODR at 800 Hz (HA02)
    Ha02_800hz = 0x29,
    /// High-accuracy ODR at 1600 Hz (HA02)
    Ha02_1600hz = 0x2A,
    /// High-accuracy ODR at 3200 Hz (HA02)
    Ha02_3200hz = 0x2B,
    /// High-accuracy ODR at 6400 Hz (HA02)
    Ha02_6400hz = 0x2C,
}

/// Accelerometer operating modes
///
/// Defines the power and performance modes for the accelerometer.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum XlMode {
    /// High-performance mode (default)
    #[default]
    HighPerformanceMd = 0x0,
    /// High-accuracy ODR mode
    HighAccuracyOdrMd = 0x1,
    /// ODR-triggered mode
    OdrTriggeredMd = 0x3,
    /// Low-power mode with 2 averages
    LowPower2AvgMd = 0x4,
    /// Low-power mode with 4 averages
    LowPower4AvgMd = 0x5,
    /// Low-power mode with 8 averages
    LowPower8AvgMd = 0x6,
    /// Normal mode
    NormalMd = 0x7,
}

/// Gyroscope operating modes
///
/// Defines the power and performance modes for the gyroscope.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum GyMode {
    /// High-performance mode (default)
    #[default]
    HighPerformanceMd = 0x0,
    /// High-accuracy ODR mode
    HighAccuracyOdrMd = 0x1,
    /// Sleep mode
    SleepMd = 0x4,
    /// Low-power mode
    LowPowerMd = 0x5,
}

/// Data ready signal modes
///
/// Defines the mode of the data ready signal.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum DataReadyMode {
    /// Latched mode (default)
    #[default]
    Latched = 0x0,
    /// Pulsed mode
    Pulsed = 0x1,
}

/// Gyroscope full scale selection
///
/// Defines the full scale range for the gyroscope.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum GyFullScale {
    /// ±125 dps (default)
    #[default]
    _125dps = 0x0,
    /// ±250 dps
    _250dps = 0x1,
    /// ±500 dps
    _500dps = 0x2,
    /// ±1000 dps
    _1000dps = 0x3,
    /// ±2000 dps
    _2000dps = 0x4,
    /// ±4000 dps
    _4000dps = 0xc,
}

/// Accelerometer full scale selection
///
/// Defines the full scale range for the accelerometer.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum XlFullScale {
    /// ±2 g (default)
    #[default]
    _2g = 0x0,
    /// ±4 g
    _4g = 0x1,
    /// ±8 g
    _8g = 0x2,
    /// ±16 g
    _16g = 0x3,
}

/// Accelerometer self-test modes
///
/// Defines the self-test modes for the accelerometer.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum XlSelfTest {
    /// Self-test disabled (default)
    #[default]
    Disable = 0x0,
    /// Positive self-test
    Positive = 0x1,
    /// Negative self-test
    Negative = 0x2,
}

/// OIS accelerometer self-test modes
///
/// Defines the self-test modes for the OIS accelerometer.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum OisXlSelfTest {
    /// Self-test disabled (default)
    #[default]
    Disable = 0x0,
    /// Positive self-test
    Positive = 0x1,
    /// Negative self-test
    Negative = 0x2,
}

/// Gyroscope self-test modes
///
/// Defines the self-test modes for the gyroscope.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum GySelfTest {
    /// Self-test disabled (default)
    #[default]
    Disable = 0x0,
    /// Positive self-test
    Positive = 0x1,
    /// Negative self-test
    Negative = 0x2,
}

/// Data enable (DEN) polarity
///
/// Defines the active level of the data enable signal.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum DenPolarity {
    /// Data enable active low (default)
    #[default]
    ActiveLow = 0x0,
    /// Data enable active high
    ActiveHigh = 0x1,
}

/// EIS gyroscope full scale selection
///
/// Defines the full scale range for the EIS gyroscope.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum EisGyFullScale {
    /// ±125 dps (default)
    #[default]
    _125dps = 0x0,
    /// ±250 dps
    _250dps = 0x1,
    /// ±500 dps
    _500dps = 0x2,
    /// ±1000 dps
    _1000dps = 0x3,
    /// ±2000 dps
    _2000dps = 0x4,
}

/// EIS gyroscope data rate
///
/// Defines the output data rate for the EIS gyroscope.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum GyEisDataRate {
    /// EIS output data rate off (default)
    #[default]
    Off = 0x0,
    /// EIS output data rate 1920 Hz
    _1920hz = 0x1,
    /// EIS output data rate 960 Hz
    _960hz = 0x2,
}

/// FIFO compression algorithm
///
/// Defines the compression algorithm used in FIFO.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum FifoCompressAlgo {
    /// Compression disabled (default)
    #[default]
    Disable = 0x0,
    /// Compression 8:1
    _8To1 = 0x1,
    /// Compression 16:1
    _16To1 = 0x2,
    /// Compression 32:1
    _32To1 = 0x3,
}

/// FIFO batching frequency options
///
/// Allows selecting the batch rate for accelerometer and gyroscope data in FIFO.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum FifoBatch {
    /// No batching (default)
    #[default]
    NotBatched = 0x0,
    /// Batched at 1.875 Hz
    _1_875hz = 0x1,
    /// Batched at 7.5 Hz
    _7_5hz = 0x2,
    /// Batched at 15 Hz
    _15hz = 0x3,
    /// Batched at 30 Hz
    _30hz = 0x4,
    /// Batched at 60 Hz
    _60hz = 0x5,
    /// Batched at 120 Hz
    _120hz = 0x6,
    /// Batched at 240 Hz
    _240hz = 0x7,
    /// Batched at 480 Hz
    _480hz = 0x8,
    /// Batched at 960 Hz
    _960hz = 0x9,
    /// Batched at 1920 Hz
    _1920hz = 0xa,
    /// Batched at 3840 Hz
    _3840hz = 0xb,
    /// Batched at 7680 Hz
    _7680hz = 0xc,
}

/// FIFO operating modes
///
/// Defines the behavior of the FIFO buffer.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum FifoMode {
    /// FIFO bypass mode, FIFO is not operational (default)
    #[default]
    BypassMode = 0x0,
    /// FIFO mode, data stored until FIFO is full
    FifoMode = 0x1,
    /// Continuous mode with watermark to full mode
    StreamWtmToFullMode = 0x2,
    /// Continuous to FIFO mode
    StreamToFifoMode = 0x3,
    /// Bypass to continuous mode
    BypassToStreamMode = 0x4,
    /// Continuous mode, FIFO updated continuously
    StreamMode = 0x6,
    /// Bypass to FIFO mode
    BypassToFifoMode = 0x7,
}

/// FIFO temperature sensor batching options
///
/// Allows selecting the batch rate for temperature sensor data in FIFO.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum FifoTempBatch {
    /// No batching (default)
    #[default]
    NotBatched = 0x0,
    /// Batched at 1.875 Hz
    _1_875hz = 0x1,
    /// Batched at 15 Hz
    _15hz = 0x2,
    /// Batched at 60 Hz
    _60hz = 0x3,
}

/// FIFO timestamp batching options
///
/// Allows selecting the decimation factor for timestamp batching in FIFO.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum FifoTimestampBatch {
    /// No batching (default)
    #[default]
    NotBatched = 0x0,
    /// Decimation factor 1 (no decimation)
    Dec1 = 0x1,
    /// Decimation factor 8
    Dec8 = 0x2,
    /// Decimation factor 32
    Dec32 = 0x3,
}

/// FIFO batch count event source
///
/// Selects which sensor's batch event triggers the FIFO batch counter.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum FifoBatchCntEvent {
    /// Accelerometer batch event (default)
    #[default]
    Xl = 0x0,
    /// Gyroscope batch event
    Gy = 0x1,
    /// Gyroscope EIS batch event
    GyEis = 0x2,
}

/// Anti-spike filter configuration
///
/// Controls the I2C anti-spike filter behavior.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum FiltAntiSpike {
    /// Auto management of anti-spike filter (default)
    #[default]
    Auto = 0x0,
    /// Always active anti-spike filter
    AlwaysActive = 0x1,
}

/// Gyroscope LPF1 bandwidth selection
///
/// Selects the bandwidth of the gyroscope LPF1 filter.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum FiltGyLp1Bandwidth {
    /// Ultra light bandwidth (default)
    #[default]
    UltraLight = 0x0,
    /// Very light bandwidth
    VeryLight = 0x1,
    /// Light bandwidth
    Light = 0x2,
    /// Medium bandwidth
    Medium = 0x3,
    /// Strong bandwidth
    Strong = 0x4,
    /// Very strong bandwidth
    VeryStrong = 0x5,
    /// Aggressive bandwidth
    Aggressive = 0x6,
    /// Extreme bandwidth
    Xtreme = 0x7,
}

/// Accelerometer LPF2 bandwidth selection
///
/// Selects the bandwidth of the accelerometer LPF2 filter.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum FiltXlLp2Bandwidth {
    /// Ultra light bandwidth (default)
    #[default]
    UltraLight = 0x0,
    /// Very light bandwidth
    VeryLight = 0x1,
    /// Light bandwidth
    Light = 0x2,
    /// Medium bandwidth
    Medium = 0x3,
    /// Strong bandwidth
    Strong = 0x4,
    /// Very strong bandwidth
    VeryStrong = 0x5,
    /// Aggressive bandwidth
    Aggressive = 0x6,
    /// Extreme bandwidth
    Xtreme = 0x7,
}

/// Accelerometer high-pass filter mode
///
/// Selects the mode of the accelerometer high-pass filter.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum FiltXlHpMode {
    /// Normal mode (default)
    #[default]
    Normal = 0x0,
    /// Reference mode
    Reference = 0x1,
}

/// Wake-up filter activity feed selection
///
/// Selects the filter used for wake-up activity detection.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum FiltWkupActFeed {
    /// Slope filter (default)
    #[default]
    Slope = 0x0,
    /// High-pass filter
    HighPass = 0x1,
    /// Low-pass filter with offset
    LpWithOffset = 0x2,
}

/// 6D orientation filter feed selection
///
/// Selects the filter used for 6D orientation detection.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum FiltSixdFeed {
    /// Output data rate divided by 2 (default)
    #[default]
    OdrDiv2 = 0x0,
    /// Low-pass filter
    LowPass = 0x1,
}

/// Gyroscope EIS low-pass filter bandwidth selection
///
/// Selects the bandwidth of the gyroscope EIS low-pass filter.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum FiltGyEisLpBandwidth {
    /// Normal bandwidth (default)
    #[default]
    Normal = 0x0,
    /// Light bandwidth
    Light = 0x1,
}

/// Gyroscope OIS low-pass filter bandwidth selection
///
/// Selects the bandwidth of the gyroscope OIS low-pass filter.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum FiltGyOisLpBandwidth {
    /// Normal bandwidth (default)
    #[default]
    Normal = 0x0,
    /// Strong bandwidth
    Strong = 0x1,
    /// Aggressive bandwidth
    Aggressive = 0x2,
    /// Light bandwidth
    Light = 0x3,
}

/// Accelerometer OIS low-pass filter bandwidth selection
///
/// Selects the bandwidth of the accelerometer OIS low-pass filter.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum FiltXlOisLpBandwidth {
    /// Ultra light bandwidth (default)
    #[default]
    UltraLight = 0x0,
    /// Very light bandwidth
    VeryLight = 0x1,
    /// Light bandwidth
    Light = 0x2,
    /// Normal bandwidth
    Normal = 0x3,
    /// Strong bandwidth
    Strong = 0x4,
    /// Very strong bandwidth
    VeryStrong = 0x5,
    /// Aggressive bandwidth
    Aggressive = 0x6,
    /// Extreme bandwidth
    Xtreme = 0x7,
}

/// Finite state machine permission
///
/// Controls access permissions for FSM to device registers.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum FsmPermission {
    /// Protect control registers (default)
    #[default]
    ProtectCtrlRegs = 0x0,
    /// Write control registers
    WriteCtrlReg = 0x1,
}

/// Free-fall threshold levels
///
/// Defines the threshold levels for free-fall detection.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum FfThreshold {
    /// Threshold 156 mg (default)
    #[default]
    _156mg = 0x0,
    /// Threshold 219 mg
    _219mg = 0x1,
    /// Threshold 250 mg
    _250mg = 0x2,
    /// Threshold 312 mg
    _312mg = 0x3,
    /// Threshold 344 mg
    _344mg = 0x4,
    /// Threshold 406 mg
    _406mg = 0x5,
    /// Threshold 469 mg
    _469mg = 0x6,
    /// Threshold 500 mg
    _500mg = 0x7,
}

/// OIS control mode
///
/// Selects the control interface for OIS functionality.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum OisCtrlMode {
    /// Auxiliary SPI full control (default)
    #[default]
    FromOis = 0x0,
    /// Primary interface full control
    FromUi = 0x1,
}

/// OIS gyroscope full scale
///
/// Defines the full scale range for the OIS gyroscope.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum OisGyFullScale {
    /// ±125 dps (default)
    #[default]
    _125dps = 0x0,
    /// ±250 dps
    _250dps = 0x1,
    /// ±500 dps
    _500dps = 0x2,
    /// ±1000 dps
    _1000dps = 0x3,
    /// ±2000 dps
    _2000dps = 0x4,
}

/// OIS accelerometer full scale
///
/// Defines the full scale range for the OIS accelerometer.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum OisXlFullScale {
    /// ±2 g (default)
    #[default]
    _2g = 0x0,
    /// ±4 g
    _4g = 0x1,
    /// ±8 g
    _8g = 0x2,
    /// ±16 g
    _16g = 0x3,
}

/// 6D orientation threshold
///
/// Defines the angle thresholds for 6D orientation detection.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum SixDThreshold {
    /// 80 degrees (default)
    #[default]
    _80deg = 0x0,
    /// 70 degrees
    _70deg = 0x1,
    /// 60 degrees
    _60deg = 0x2,
    /// 50 degrees
    _50deg = 0x3,
}

/// Analog hub Qvar input impedance
///
/// Selects the equivalent input impedance of the analog hub buffers.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum AhQvarZin {
    /// 2400 MΩ (default)
    #[default]
    _2400mohm = 0x0,
    /// 730 MΩ
    _730mohm = 0x1,
    /// 300 MΩ
    _300mohm = 0x2,
    /// 255 MΩ
    _255mohm = 0x3,
}

/// I3C reset mode
///
/// Selects the reset mode for the I3C interface.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum I3cResetMode {
    /// Software reset and dynamic address reset (default)
    #[default]
    SwRstDynAddressRst = 0x0,
    /// I3C global reset
    I3cGlobalRst = 0x1,
}

/// I3C In-Band Interrupt (IBI) timing
///
/// Defines the timing for I3C IBI.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum I3cIbiTime {
    /// 2 μs (default)
    #[default]
    _2us = 0x0,
    /// 50 μs
    _50us = 0x1,
    /// 1 ms
    _1ms = 0x2,
    /// 25 ms
    _25ms = 0x3,
}

/// UI I2C/I3C mode
///
/// Enables or disables I2C/I3C interface.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum UiI2cI3cMode {
    /// I2C/I3C enabled (default)
    #[default]
    Enable = 0x0,
    /// I2C/I3C disabled
    Disable = 0x1,
}

/// SPI mode
///
/// Selects SPI interface mode.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum SpiMode {
    /// SPI 4-wire mode (default)
    #[default]
    _4Wire = 0x0,
    /// SPI 3-wire mode
    _3Wire = 0x1,
}

/// SPI interface mode selection
///
/// SPI 4-wire or 3-wire interface mode.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum Spi2Mode {
    /// SPI 4-wire mode (default)
    #[default]
    _4Wire = 0x0,
    /// SPI 3-wire mode
    _3Wire = 0x1,
}

/// Tap axis priority
///
/// Defines the priority order of axes for tap detection.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum TapAxisPriority {
    /// X, Y, Z axis priority (default)
    #[default]
    Xyz = 0x0,
    /// Y, X, Z axis priority
    Yxz = 0x1,
    /// X, Z, Y axis priority
    Xzy = 0x2,
    /// Z, Y, X axis priority
    Zyx = 0x3,
    /// Y, Z, X axis priority
    Yzx = 0x5,
    /// Z, X, Y axis priority
    Zxy = 0x6,
}

/// Tap mode selection
///
/// Selects between single tap only or both single and double tap detection.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum TapMode {
    /// Only single tap detection (default)
    #[default]
    OnlySingle = 0x0,
    /// Both single and double tap detection
    BothSingleDouble = 0x1,
}

/// Activity mode selection
///
/// Defines the power mode behavior of accelerometer and gyroscope during activity detection.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum ActMode {
    /// Accelerometer and gyroscope not affected (default)
    #[default]
    XlAndGyNotAffected = 0x0,
    /// Accelerometer low power, gyroscope not affected
    XlLowPowerGyNotAffected = 0x1,
    /// Accelerometer low power, gyroscope sleep mode
    XlLowPowerGySleep = 0x2,
    /// Accelerometer low power, gyroscope power down
    XlLowPowerGyPowerDown = 0x3,
}

/// Activity mode sleep to active duration
///
/// Defines the number of samples before switching from sleep to active mode.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum ActFromSleepToActDur {
    /// Switch at first sample (default)
    #[default]
    _1stSample = 0x0,
    /// Switch at second sample
    _2ndSample = 0x1,
    /// Switch at third sample
    _3rdSample = 0x2,
    /// Switch at fourth sample
    _4thSample = 0x3,
}

/// Activity sleep accelerometer output data rate (ODR)
///
/// Defines the ODR of the accelerometer during sleep mode.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum ActSleepXlOdr {
    /// 1.875 Hz ODR (default)
    #[default]
    _1_875hz = 0x0,
    /// 15 Hz ODR
    _15hz = 0x1,
    /// 30 Hz ODR
    _30hz = 0x2,
    /// 60 Hz ODR
    _60hz = 0x3,
}

#[derive(Clone, Copy, Default)]
pub struct PinIntRoute {
    pub drdy_xl: u8,
    pub drdy_g: u8,
    pub drdy_g_eis: u8,
    pub drdy_temp: u8,
    pub drdy_ah_qvar: u8,
    pub fifo_th: u8,
    pub fifo_ovr: u8,
    pub fifo_full: u8,
    pub cnt_bdr: u8,
    pub emb_func_endop: u8,
    pub timestamp: u8,
    pub shub: u8,
    pub emb_func: u8,
    pub sixd: u8,
    pub single_tap: u8,
    pub double_tap: u8,
    pub wakeup: u8,
    pub freefall: u8,
    pub sleep_change: u8,
}

#[derive(Default)]
pub struct DataReady {
    pub drdy_xl: u8,
    pub drdy_gy: u8,
    pub drdy_temp: u8,
}

#[derive(Clone, Copy, Default)]
pub struct FiltSettlingMask {
    pub drdy: u8,
    pub ois_drdy: u8,
    pub irq_xl: u8,
    pub irq_g: u8,
}

#[derive(Clone, Copy, Default)]
pub struct OisHandshake {
    pub ack: u8,
    pub req: u8,
}

#[derive(Clone, Copy, Default)]
pub struct OisChain {
    pub gy: u8,
    pub xl: u8,
}

#[derive(Clone, Copy, Default)]
pub struct AhQvarMode {
    pub ah_qvar_en: u8,
}

#[derive(Clone, Copy, Default)]
pub struct TapDetection {
    pub tap_x_en: u8,
    pub tap_y_en: u8,
    pub tap_z_en: u8,
}

#[derive(Clone, Copy, Default)]
pub struct TapThresholds {
    pub x: u8,
    pub y: u8,
    pub z: u8,
}

#[derive(Clone, Copy, Default)]
pub struct TapTimeWindows {
    pub shock: u8,
    pub quiet: u8,
    pub tap_gap: u8,
}

#[derive(Clone, Copy, Default)]
pub struct ActWkupTimeWindows {
    pub shock: u8,
    pub quiet: u8,
}

#[derive(Clone, Copy, Default)]
pub struct XlOffsetMg {
    pub z_mg: f32,
    pub y_mg: f32,
    pub x_mg: f32,
}

#[derive(Clone, Copy, Default)]
pub struct InterruptMode {
    pub enable: u8,
    pub lir: u8,
}

#[derive(Clone, Copy, Default)]
pub struct FifoStatus {
    pub fifo_level: u16,
    pub fifo_bdr: u8,
    pub fifo_full: u8,
    pub fifo_ovr: u8,
    pub fifo_th: u8,
}

#[derive(Clone, Copy, Default)]
pub struct ActThresholds {
    pub inactivity_cfg: InactivityDur,
    pub inactivity_ths: u8,
    pub threshold: u8,
    pub duration: u8,
}

#[derive(Clone, Copy)]
pub struct DenConf {
    pub stamp_in_gy_data: u8,
    pub stamp_in_xl_data: u8,
    pub den_x: u8,
    pub den_y: u8,
    pub den_z: u8,
    pub mode: DenMode,
}

#[repr(u8)]
#[derive(Debug, Copy, Clone, Default, TryFrom)]
#[try_from(repr)]
pub enum DenMode {
    #[default]
    DenNotDefined = 0x00,
    LevelTrigger = 0x02,
    LevelLatched = 0x03,
}

#[derive(Clone, Copy)]
pub struct FifoOutRaw {
    pub tag: Tag,
    pub cnt: u8,
    pub data: [u8; 6],
}

#[repr(u8)]
#[derive(Default, PartialEq, Clone, Copy, TryFrom)]
#[try_from(repr)]
pub enum Tag {
    #[default]
    FifoEmpty = 0x0,
    GyNcTag = 0x1,
    XlNcTag = 0x2,
    TemperatureTag = 0x3,
    TimestampTag = 0x4,
    CfgChangeTag = 0x5,
    XlNcT2Tag = 0x6,
    XlNcT1Tag = 0x7,
    Xl2xcTag = 0x8,
    Xl3xcTag = 0x9,
    GyNcT2Tag = 0xA,
    GyNcT1Tag = 0xB,
    Gy2xcTag = 0xC,
    Gy3xcTag = 0xD,
    SensorhubSlave0Tag = 0xE,
    SensorhubSlave1Tag = 0xF,
    SensorhubSlave2Tag = 0x10,
    SensorhubSlave3Tag = 0x11,
    StepCounterTag = 0x12,
    SflpGameRotationVectorTag = 0x13,
    SflpGyroscopeBiasTag = 0x16,
    SflpGravityVectorTag = 0x17,
    SensorhubNackTag = 0x19,
    MlcResultTag = 0x1A,
    MlcFilter = 0x1B,
    MlcFeature = 0x1C,
    XlDualCore = 0x1D,
    GyEnhancedEis = 0x1E,
}
