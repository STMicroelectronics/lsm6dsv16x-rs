use crate::EmbedFuncState;
use crate::Error;
use bitfield_struct::bitfield;
use derive_more::TryFrom;
use embedded_hal::delay::DelayNs;
use st_mem_bank_macro::{MultiRegister, register};
use st_mems_bus::BusOperation;

#[repr(u8)]
#[derive(Clone, Copy, PartialEq)]
pub enum EmbReg {
    PageSel = 0x2,
    EmbFuncEnA = 0x4,
    EmbFuncEnB = 0x5,
    EmbFuncExecStatus = 0x7,
    PageValue = 0x9,
    EmbFuncInt1 = 0x0A,
    FsmInt1 = 0x0B,
    MlcInt1 = 0x0D,
    EmbFuncInt2 = 0x0E,
    FsmInt2 = 0x0F,
    MlcInt2 = 0x11,
    EmbFuncStatus = 0x12,
    FsmStatus = 0x13,
    MlcStatus = 0x15,
    PageRw = 0x17,
    EmbFuncFifoEnA = 0x44,
    EmbFuncFifoEnB = 0x45,
    FsmEnable = 0x46,
    FsmLongCounterL = 0x48,
    FsmLongCounterH = 0x49,
    IntAckMask = 0x4B,
    FsmOuts1 = 0x4C,
    FsmOuts2 = 0x4D,
    FsmOuts3 = 0x4E,
    FsmOuts4 = 0x4F,
    FsmOuts5 = 0x50,
    FsmOuts6 = 0x51,
    FsmOuts7 = 0x52,
    FsmOuts8 = 0x53,
    SflpOdr = 0x5E,
    FsmOdr = 0x5F,
    MlcOdr = 0x60,
    StepCounterL = 0x62,
    StepCounterH = 0x63,
    EmbFuncSrc = 0x64,
    EmbFuncInitA = 0x66,
    EmbFuncInitB = 0x67,
    Mlc1Src = 0x70,
    Mlc2Src = 0x71,
    Mlc3Src = 0x72,
    Mlc4Src = 0x73,
    PageAddress = 0x8,
}

/// PAGE_SEL (0x02)
///
/// Enable advanced features dedicated page (R/W)
#[register(address = EmbReg::PageSel, access_type = EmbedFuncState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct PageSel {
    #[bits(4, access = RO)]
    pub not_used0: u8,
    /// Selects the advanced features dedicated page.
    /// Default: 0000
    #[bits(4)]
    pub page_sel: u8,
}

/// EMB_FUNC_EN_A (0x04)
///
/// Enable embedded functions register A (R/W)
#[register(address = EmbReg::EmbFuncEnA, access_type = EmbedFuncState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct EmbFuncEnA {
    #[bits(1, access = RO)]
    pub not_used0: u8,
    /// Enables sensor fusion low-power algorithm for 6-axis (accelerometer + gyroscope) game rotation vector.
    /// Default: 0 (disabled)
    #[bits(1)]
    pub sflp_game_en: u8,
    #[bits(1, access = RO)]
    pub not_used2: u8,
    /// Enables pedometer algorithm.
    /// Default: 0 (disabled)
    #[bits(1)]
    pub pedo_en: u8,
    /// Enables tilt calculation.
    /// Default: 0 (disabled)
    #[bits(1)]
    pub tilt_en: u8,
    /// Enables significant motion detection function.
    /// Default: 0 (disabled)
    #[bits(1)]
    pub sign_motion_en: u8,
    #[bits(1, access = RO)]
    pub not_used1: u8,
    /// Enables machine learning core function executed before FSM programs.
    /// Default: 0 (disabled)
    #[bits(1)]
    pub mlc_before_fsm_en: u8,
}

/// EMB_FUNC_EN_B (0x05)
///
/// Enable embedded functions register B (R/W)
#[register(address = EmbReg::EmbFuncEnB, access_type = EmbedFuncState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct EmbFuncEnB {
    /// Enables finite state machine (FSM) function.
    /// Default: 0 (disabled)
    #[bits(1)]
    pub fsm_en: u8,
    #[bits(2, access = RO)]
    pub not_used0: u8,
    /// Enables FIFO compression function.
    /// Default: 0 (disabled)
    #[bits(1)]
    pub fifo_compr_en: u8,
    /// Enables machine learning core function executed after FSM programs.
    /// Default: 0 (disabled)
    #[bits(1)]
    pub mlc_en: u8,
    #[bits(3, access = RO)]
    pub not_used1: u8,
}

/// EMB_FUNC_EXEC_STATUS (0x07)
///
/// Embedded functions execution status register (R)
#[register(address = EmbReg::EmbFuncExecStatus, access_type = EmbedFuncState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct EmbFuncExecStatus {
    /// Indicates no embedded function is running.
    /// Default: 0
    #[bits(1)]
    pub emb_func_endop: u8,
    /// Indicates execution of embedded functions program exceeds maximum time.
    /// Default: 0
    #[bits(1)]
    pub emb_func_exec_ovr: u8,
    #[bits(6, access = RO)]
    pub not_used0: u8,
}

/// PAGE_ADDRESS (0x08)
///
/// Page address register (R/W)
#[register(address = EmbReg::PageAddress, access_type = EmbedFuncState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct PageAddress {
    /// Address of the register to be written/read in the selected advanced features page.
    #[bits(8)]
    pub page_addr: u8,
}

/// PAGE_VALUE (0x09)
///
/// Page value register (R/W)
#[register(address = EmbReg::PageValue, access_type = EmbedFuncState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct PageValue {
    /// Data to write or read at the address specified in PAGE_ADDRESS.
    #[bits(8)]
    pub page_value: u8,
}

/// EMB_FUNC_INT1 (0x0A)
///
/// INT1 pin control register for embedded functions (R/W)
#[register(address = EmbReg::EmbFuncInt1, access_type = EmbedFuncState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct EmbFuncInt1 {
    #[bits(3, access = RO)]
    pub not_used0: u8,
    /// Routing pedometer step recognition event to INT1.
    /// Default: 0 (disabled)
    #[bits(1)]
    pub int1_step_detector: u8,
    /// Routing tilt event to INT1.
    /// Default: 0 (disabled)
    #[bits(1)]
    pub int1_tilt: u8,
    /// Routing significant motion event to INT1.
    /// Default: 0 (disabled)
    #[bits(1)]
    pub int1_sig_mot: u8,
    #[bits(1, access = RO)]
    pub not_used1: u8,
    /// Routing FSM long counter timeout interrupt event to INT1.
    /// Default: 0 (disabled)
    #[bits(1)]
    pub int1_fsm_lc: u8,
}

/// FSM_INT1 (0x0B)
///
/// INT1 pin control register for FSM interrupts (R/W)
#[register(address = EmbReg::FsmInt1, access_type = EmbedFuncState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct FsmInt1 {
    /// Routing FSM1 interrupt event to INT1.
    #[bits(1)]
    pub int1_fsm1: u8,
    /// Routing FSM2 interrupt event to INT1.
    #[bits(1)]
    pub int1_fsm2: u8,
    /// Routing FSM3 interrupt event to INT1.
    #[bits(1)]
    pub int1_fsm3: u8,
    /// Routing FSM4 interrupt event to INT1.
    #[bits(1)]
    pub int1_fsm4: u8,
    /// Routing FSM5 interrupt event to INT1.
    #[bits(1)]
    pub int1_fsm5: u8,
    /// Routing FSM6 interrupt event to INT1.
    #[bits(1)]
    pub int1_fsm6: u8,
    /// Routing FSM7 interrupt event to INT1.
    #[bits(1)]
    pub int1_fsm7: u8,
    /// Routing FSM8 interrupt event to INT1.
    #[bits(1)]
    pub int1_fsm8: u8,
}

/// MLC_INT1 (0x0D)
///
/// INT1 pin control register for machine learning core interrupts (R/W)
#[register(address = EmbReg::MlcInt1, access_type = EmbedFuncState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct MlcInt1 {
    /// Routing MLC1 interrupt event to INT1.
    #[bits(1)]
    pub int1_mlc1: u8,
    /// Routing MLC2 interrupt event to INT1.
    #[bits(1)]
    pub int1_mlc2: u8,
    /// Routing MLC3 interrupt event to INT1.
    #[bits(1)]
    pub int1_mlc3: u8,
    /// Routing MLC4 interrupt event to INT1.
    #[bits(1)]
    pub int1_mlc4: u8,
    #[bits(4, access = RO)]
    pub not_used0: u8,
}

/// EMB_FUNC_INT2 (0x0E)
///
/// INT2 pin control register for embedded functions (R/W)
#[register(address = EmbReg::EmbFuncInt2, access_type = EmbedFuncState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct EmbFuncInt2 {
    #[bits(3, access = RO)]
    pub not_used0: u8,
    /// Routing pedometer step recognition event to INT2.
    #[bits(1)]
    pub int2_step_detector: u8,
    /// Routing tilt event to INT2.
    #[bits(1)]
    pub int2_tilt: u8,
    /// Routing significant motion event to INT2.
    #[bits(1)]
    pub int2_sig_mot: u8,
    #[bits(1, access = RO)]
    pub not_used1: u8,
    /// Routing FSM long counter timeout interrupt event to INT2.
    #[bits(1)]
    pub int2_fsm_lc: u8,
}

/// FSM_INT2 (0x0F)
///
/// INT2 pin control register for FSM interrupts (R/W)
#[register(address = EmbReg::FsmInt2, access_type = EmbedFuncState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct FsmInt2 {
    /// Routing FSM1 interrupt event to INT2.
    #[bits(1)]
    pub int2_fsm1: u8,
    /// Routing FSM2 interrupt event to INT2.
    #[bits(1)]
    pub int2_fsm2: u8,
    /// Routing FSM3 interrupt event to INT2.
    #[bits(1)]
    pub int2_fsm3: u8,
    /// Routing FSM4 interrupt event to INT2.
    #[bits(1)]
    pub int2_fsm4: u8,
    /// Routing FSM5 interrupt event to INT2.
    #[bits(1)]
    pub int2_fsm5: u8,
    /// Routing FSM6 interrupt event to INT2.
    #[bits(1)]
    pub int2_fsm6: u8,
    /// Routing FSM7 interrupt event to INT2.
    #[bits(1)]
    pub int2_fsm7: u8,
    /// Routing FSM8 interrupt event to INT2.
    #[bits(1)]
    pub int2_fsm8: u8,
}

/// MLC_INT2 (0x11)
///
/// INT2 pin control register for machine learning core interrupts (R/W)
#[register(address = EmbReg::MlcInt2, access_type = EmbedFuncState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct MlcInt2 {
    /// Routing MLC1 interrupt event to INT2.
    #[bits(1)]
    pub int2_mlc1: u8,
    /// Routing MLC2 interrupt event to INT2.
    #[bits(1)]
    pub int2_mlc2: u8,
    /// Routing MLC3 interrupt event to INT2.
    #[bits(1)]
    pub int2_mlc3: u8,
    /// Routing MLC4 interrupt event to INT2.
    #[bits(1)]
    pub int2_mlc4: u8,
    #[bits(4, access = RO)]
    pub not_used0: u8,
}

/// EMB_FUNC_STATUS (0x12)
///
/// Embedded function status register (R)
#[register(address = EmbReg::EmbFuncStatus, access_type = EmbedFuncState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct EmbFuncStatus {
    #[bits(3, access = RO)]
    pub not_used0: u8,
    /// Interrupt status bit for step detection.
    #[bits(1)]
    pub is_step_det: u8,
    /// Interrupt status bit for tilt detection.
    #[bits(1)]
    pub is_tilt: u8,
    /// Interrupt status bit for significant motion detection.
    #[bits(1)]
    pub is_sigmot: u8,
    #[bits(1, access = RO)]
    pub not_used1: u8,
    /// Interrupt status bit for FSM long counter timeout interrupt event.
    #[bits(1)]
    pub is_fsm_lc: u8,
}

/// FSM_STATUS (0x13)
///
/// Finite state machine status register (R)
#[register(address = EmbReg::FsmStatus, access_type = EmbedFuncState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct FsmStatus {
    /// Interrupt status bit for FSM1 interrupt event.
    #[bits(1)]
    pub is_fsm1: u8,
    /// Interrupt status bit for FSM2 interrupt event.
    #[bits(1)]
    pub is_fsm2: u8,
    /// Interrupt status bit for FSM3 interrupt event.
    #[bits(1)]
    pub is_fsm3: u8,
    /// Interrupt status bit for FSM4 interrupt event.
    #[bits(1)]
    pub is_fsm4: u8,
    /// Interrupt status bit for FSM5 interrupt event.
    #[bits(1)]
    pub is_fsm5: u8,
    /// Interrupt status bit for FSM6 interrupt event.
    #[bits(1)]
    pub is_fsm6: u8,
    /// Interrupt status bit for FSM7 interrupt event.
    #[bits(1)]
    pub is_fsm7: u8,
    /// Interrupt status bit for FSM8 interrupt event.
    #[bits(1)]
    pub is_fsm8: u8,
}

/// MLC_STATUS (0x4B)
///
/// Machine learning core status register (R)
///
/// Interrupt status bits for MLC interrupt events.
/// Each bit indicates if the corresponding MLC interrupt event is detected (1) or not (0).
#[register(address = EmbReg::MlcStatus, access_type = EmbedFuncState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct MlcStatus {
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

/// PAGE_RW (0x17)
///
/// Enable read and write mode of advanced features dedicated page (R/W)
///
/// Controls the read/write access modes for the advanced features dedicated page.
/// - PAGE_READ: Enables reads from the selected advanced features dedicated page.
/// - PAGE_WRITE: Enables writes to the selected advanced features dedicated page.
/// - EMB_FUNC_LIR: Latched interrupt mode for embedded functions.
#[register(address = EmbReg::PageRw, access_type = EmbedFuncState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct PageRw {
    #[bits(5, access = RO)]
    pub not_used0: u8,
    /// Enables reads from the selected advanced features dedicated page (0: disable; 1: enable)
    #[bits(1)]
    pub page_read: u8,
    /// Enables writes to the selected advanced features dedicated page (0: disable; 1: enable)
    #[bits(1)]
    pub page_write: u8,
    /// Latched interrupt request for embedded functions (0: not latched; 1: latched)
    #[bits(1)]
    pub emb_func_lir: u8,
}

/// EMB_FUNC_FIFO_EN_A (0x44)
///
/// Embedded functions FIFO configuration register A (R/W)
///
/// Enables batching of various embedded function results into the FIFO buffer.
#[register(address = EmbReg::EmbFuncFifoEnA, access_type = EmbedFuncState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct EmbFuncFifoEnA {
    #[bits(1, access = RO)]
    pub not_used0: u8,
    /// Enables batching the game rotation vector (quaternion) values computed by the SFLP algorithm in the FIFO buffer.
    #[bits(1)]
    pub sflp_game_fifo_en: u8,
    #[bits(2, access = RO)]
    pub not_used1: u8,
    /// Enables batching the gravity values computed by the SFLP algorithm in the FIFO buffer.
    #[bits(1)]
    pub sflp_gravity_fifo_en: u8,
    /// Enables batching the gyroscope bias values computed by the SFLP algorithm in the FIFO buffer.
    #[bits(1)]
    pub sflp_gbias_fifo_en: u8,
    /// Enables batching the step counter values in the FIFO buffer.
    #[bits(1)]
    pub step_counter_fifo_en: u8,
    /// Enables batching the machine learning core results in the FIFO buffer.
    #[bits(1)]
    pub mlc_fifo_en: u8,
}

/// EMB_FUNC_FIFO_EN_B (0x45)
///
/// Embedded functions FIFO configuration register B (R/W)
///
/// Enables batching the machine learning core filters and features in the FIFO buffer.
#[register(address = EmbReg::EmbFuncFifoEnB, access_type = EmbedFuncState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct EmbFuncFifoEnB {
    #[bits(1, access = RO)]
    pub not_used0: u8,
    /// Enables batching the machine learning core filters and features in the FIFO buffer.
    #[bits(1)]
    pub mlc_filter_feature_fifo_en: u8,
    #[bits(6, access = RO)]
    pub not_used1: u8,
}

/// FSM_ENABLE (0x46)
///
/// Enable FSM register (R/W)
///
/// Enables finite state machine (FSM) functions FSM1 through FSM8.
#[register(address = EmbReg::FsmEnable, access_type = EmbedFuncState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct FsmEnable {
    /// Enables FSM1 (0: disabled; 1: enabled)
    #[bits(1)]
    pub fsm1_en: u8,
    /// Enables FSM2 (0: disabled; 1: enabled)
    #[bits(1)]
    pub fsm2_en: u8,
    /// Enables FSM3 (0: disabled; 1: enabled)
    #[bits(1)]
    pub fsm3_en: u8,
    /// Enables FSM4 (0: disabled; 1: enabled)
    #[bits(1)]
    pub fsm4_en: u8,
    /// Enables FSM5 (0: disabled; 1: enabled)
    #[bits(1)]
    pub fsm5_en: u8,
    /// Enables FSM6 (0: disabled; 1: enabled)
    #[bits(1)]
    pub fsm6_en: u8,
    /// Enables FSM7 (0: disabled; 1: enabled)
    #[bits(1)]
    pub fsm7_en: u8,
    /// Enables FSM8 (0: disabled; 1: enabled)
    #[bits(1)]
    pub fsm8_en: u8,
}

/// FSM_LONG_COUNTER_L and FSM_LONG_COUNTER_H (0x48 - 0x49)
///
/// Fsm long counter status register
#[register(address = EmbReg::FsmLongCounterL, access_type = EmbedFuncState, generics = 2)]
pub struct FsmLongCounter(pub u16);

/// INT_ACK_MASK (0x4B)
///
/// Reset status register (R/W)
///
/// Controls whether bits of the status registers are reset when reading in latched mode.
/// If a bit is set to 1, the corresponding bit in the status register is not reset upon reading.
#[register(address = EmbReg::IntAckMask, access_type = EmbedFuncState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct IntAckMask {
    #[bits(8)]
    pub iack_mask: u8,
}

/// FSM_OUTS1-8 (0x4C - 0x53)
#[register(address = EmbReg::FsmOuts1, access_type = EmbedFuncState, init_fn = FsmOutsElement::new, generics = 2)]
pub struct FsmOut(pub [FsmOutsElement; 8]);

/// FSMx generic output register (R)
///
/// Indicates positive and negative events detected on X, Y, Z axes and vector for FSMx.
#[register(address = EmbReg::FsmOuts1, access_type = EmbedFuncState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct FsmOutsElement {
    /// FSM output: negative event detected on vector
    #[bits(1)]
    pub fsm_n_v: u8,
    /// FSM output: positive event detected on vector
    #[bits(1)]
    pub fsm_p_v: u8,
    /// FSM output: negative event detected on Z-axis
    #[bits(1)]
    pub fsm_n_z: u8,
    /// FSM output: positive event detected on Z-axis
    #[bits(1)]
    pub fsm_p_z: u8,
    /// FSM output: negative event detected on Y-axis
    #[bits(1)]
    pub fsm_n_y: u8,
    /// FSM output: positive event detected on Y-axis
    #[bits(1)]
    pub fsm_p_y: u8,
    /// FSM output: negative event detected on X-axis
    #[bits(1)]
    pub fsm_n_x: u8,
    /// FSM output: positive event detected on X-axis
    #[bits(1)]
    pub fsm_p_x: u8,
}

impl FsmOutsElement {
    pub fn from_le_bytes(val: [u8; 1]) -> Self {
        FsmOutsElement(val[0])
    }

    pub fn to_le_bytes(&self) -> [u8; 1] {
        [self.0]
    }
}

/// SFLP_ODR (0x5E)
///
/// Sensor fusion low-power output data rate configuration register (R/W)
#[register(address = EmbReg::SflpOdr, access_type = EmbedFuncState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct SflpOdr {
    #[bits(3, access = RO)]
    pub not_used0: u8,
    /// ODR configuration of the SFLP game algorithm (3 bits)
    #[bits(3)]
    pub sflp_game_odr: u8,
    #[bits(2, access = RO)]
    pub not_used1: u8,
}

/// FSM_ODR (0x5F)
///
/// Finite state machine output data rate configuration register (R/W)
#[register(address = EmbReg::FsmOdr, access_type = EmbedFuncState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct FsmOdr {
    #[bits(3, access = RO)]
    pub not_used0: u8,
    /// Finite state machine ODR configuration (3 bits)
    #[bits(3)]
    pub fsm_odr: u8,
    #[bits(2, access = RO)]
    pub not_used1: u8,
}

/// MLC_ODR (0x60)
///
/// Machine learning core output data rate configuration register (R/W)
#[register(address = EmbReg::MlcOdr, access_type = EmbedFuncState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct MlcOdr {
    #[bits(4, access = RO)]
    pub not_used0: u8,
    /// Machine learning core ODR configuration (3 bits)
    #[bits(3)]
    pub mlc_odr: u8,
    #[bits(1, access = RO)]
    pub not_used1: u8,
}

/// STEP_COUNTER_L (0x62 - 0x63)
///
/// Step counter output register  (R)
#[register(address = EmbReg::StepCounterL, access_type = EmbedFuncState, generics = 2)]
pub struct StepCounter(pub u16);

/// EMB_FUNC_SRC (0x64)
///
/// Embedded function source register (R/W)
#[register(address = EmbReg::EmbFuncSrc, access_type = EmbedFuncState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct EmbFuncSrc {
    #[bits(2, access = RO)]
    pub not_used0: u8,
    /// Step counter bit set flag (1: step count increased)
    #[bits(1)]
    pub stepcounter_bit_set: u8,
    /// Step counter overflow status (1: overflow occurred)
    #[bits(1)]
    pub step_overflow: u8,
    /// Pedometer step recognition on delta time status (1: at least one step recognized)
    #[bits(1)]
    pub step_count_delta_ia: u8,
    /// Step detector event detection status (1: step detected)
    #[bits(1)]
    pub step_detected: u8,
    #[bits(1, access = RO)]
    pub not_used1: u8,
    /// Reset pedometer step counter (write 1 to reset)
    #[bits(1)]
    pub pedo_rst_step: u8,
}

/// EMB_FUNC_INIT_A (0x66)
///
/// Embedded functions initialization register (R/W)
#[register(address = EmbReg::EmbFuncInitA, access_type = EmbedFuncState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct EmbFuncInitA {
    #[bits(1, access = RO)]
    pub not_used0: u8,
    /// SFLP game algorithm initialization request
    #[bits(1)]
    pub sflp_game_init: u8,
    #[bits(1, access = RO)]
    pub not_used2: u8,
    /// Pedometer step counter/detector algorithm initialization request
    #[bits(1)]
    pub step_det_init: u8,
    /// Tilt algorithm initialization request
    #[bits(1)]
    pub tilt_init: u8,
    /// Significant motion detection algorithm initialization request
    #[bits(1)]
    pub sig_mot_init: u8,
    #[bits(1, access = RO)]
    pub not_used1: u8,
    /// Machine learning core initialization request (MLC executed before FSM)
    #[bits(1)]
    pub mlc_before_fsm_init: u8,
}

/// EMB_FUNC_INIT_B (0x67)
///
/// Embedded functions initialization register (R/W)
#[register(address = EmbReg::EmbFuncInitB, access_type = EmbedFuncState, generics = 2)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct EmbFuncInitB {
    /// FSM initialization request
    #[bits(1)]
    pub fsm_init: u8,
    #[bits(2, access = RO)]
    pub not_used0: u8,
    /// FIFO compression feature initialization request
    #[bits(1)]
    pub fifo_compr_init: u8,
    /// Machine learning core initialization request (MLC executed after FSM)
    #[bits(1)]
    pub mlc_init: u8,
    #[bits(3, access = RO)]
    pub not_used1: u8,
}

/// MLC1_SRC - MLC4_SRC (0x70 - 0x73)
///
/// Machine learning core source registers 1 - 4 (R)
#[register(address = EmbReg::Mlc1Src, access_type = EmbedFuncState, generics = 2)]
pub struct MlcSrc(pub [u8; 4]);

/// Finite state machine data rate
///
/// Selects the output data rate for the FSM.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum FsmDataRate {
    /// 15 Hz (default)
    #[default]
    _15hz = 0x0,
    /// 30 Hz
    _30hz = 0x1,
    /// 60 Hz
    _60hz = 0x2,
    /// 120 Hz
    _120hz = 0x3,
    /// 240 Hz
    _240hz = 0x4,
    /// 480 Hz
    _480hz = 0x5,
    /// 960 Hz
    _960hz = 0x6,
}

/// Machine learning core mode
///
/// Controls the operating mode of the machine learning core.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum MlcMode {
    /// Machine learning core off (default)
    #[default]
    Off = 0x0,
    /// Machine learning core on
    On = 0x1,
    /// Machine learning core on before FSM
    OnBeforeFsm = 0x2,
}

/// Machine learning core data rate
///
/// Defines the output data rate for the machine learning core.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum MlcDataRate {
    /// 15 Hz (default)
    #[default]
    _15hz = 0x0,
    /// 30 Hz
    _30hz = 0x1,
    /// 60 Hz
    _60hz = 0x2,
    /// 120 Hz
    _120hz = 0x3,
    /// 240 Hz
    _240hz = 0x4,
    /// 480 Hz
    _480hz = 0x5,
    /// 960 Hz
    _960hz = 0x6,
}

/// Sensor fusion low power (SFLP) data rate
///
/// Output data rate for the sensor fusion low power algorithm.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum SflpDataRate {
    /// 15 Hz output data rate (default)
    #[default]
    _15hz = 0x0,
    /// 30 Hz output data rate
    _30hz = 0x1,
    /// 60 Hz output data rate
    _60hz = 0x2,
    /// 120 Hz output data rate
    _120hz = 0x3,
    /// 240 Hz output data rate
    _240hz = 0x4,
    /// 480 Hz output data rate
    _480hz = 0x5,
}

#[derive(Clone, Copy, Default)]
pub struct EmbeddedStatus {
    pub tilt: u8,
    pub sig_mot: u8,
    pub fsm_lc: u8,
    pub step_detector: u8,
    pub step_count_inc: u8,
    pub step_count_overflow: u8,
    pub step_on_delta_time: u8,
}

#[derive(Clone, Copy, Default)]
pub struct FifoSflpRaw {
    pub game_rotation: u8,
    pub gravity: u8,
    pub gbias: u8,
}

#[derive(Clone, Copy, Default)]
pub struct FsmMode {
    pub fsm1_en: u8,
    pub fsm2_en: u8,
    pub fsm3_en: u8,
    pub fsm4_en: u8,
    pub fsm5_en: u8,
    pub fsm6_en: u8,
    pub fsm7_en: u8,
    pub fsm8_en: u8,
}

#[derive(Clone, Copy, Default)]
pub struct StpcntMode {
    pub step_counter_enable: u8,
    pub false_step_rej: u8,
}

#[derive(Clone, Copy, Default, MultiRegister)]
pub struct MlcOut {
    pub mlc1_src: u8,
    pub mlc2_src: u8,
    pub mlc3_src: u8,
    pub mlc4_src: u8,
}

#[derive(Clone, Copy, Default)]
pub struct EmbPinIntRoute {
    pub step_det: u8,
    pub tilt: u8,
    pub sig_mot: u8,
    pub fsm_lc: u8,
}

#[repr(u8)]
#[derive(Debug, Copy, Clone, Default, TryFrom)]
#[try_from(repr)]
pub enum EmbeddedIntConf {
    #[default]
    Pulsed = 0x00,
    Latched = 0x01,
}
